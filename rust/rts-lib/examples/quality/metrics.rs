//! Probes. Two families, matching the two questions the scorecard asks.
//!
//! [`Run`] wraps a `Sim` and watches what the units actually *do* while it
//! steps: how far they walked, when they settled, how much they overlapped,
//! how often they gave up on a path. [`PathProbe`] never steps anything: it
//! calls `find_path` directly and scores the polyline it gets back.
//!
//! Everything here is pure observation. The probe holds `&Sim` by value and
//! never writes to it, so a scored run is bit-identical to the same run
//! without the harness.

use std::collections::BTreeMap;
use std::path::Path;

use godot::prelude::Vector2;
use rts_lib::astar::{AStarScratch, find_path, find_path_abstract, path_min_clearance};
use rts_lib::delaunay::CDT;
use rts_lib::sim::{
    BLOCK_TICKS, Command, DETOUR_TICKS, DT, NO_SLOT, STALL_REPATH_TICKS, Sim, UnitId,
};

// ── Geometry helpers ─────────────────────────────────────────────────────────

pub fn polyline_len(path: &[Vector2]) -> f32 {
    path.windows(2).map(|w| (w[1] - w[0]).length()).sum()
}

/// Total absolute turn at the interior vertices of a polyline, in radians.
pub fn polyline_turn(path: &[Vector2]) -> f32 {
    path.windows(3)
        .map(|w| {
            let a = w[1] - w[0];
            let b = w[2] - w[1];
            if a.length_squared() < 1e-12 || b.length_squared() < 1e-12 {
                0.0
            } else {
                angle_between(a, b)
            }
        })
        .sum()
}

fn angle_between(a: Vector2, b: Vector2) -> f32 {
    let cross = a.x * b.y - a.y * b.x;
    let dot = a.x * b.x + a.y * b.y;
    cross.atan2(dot).abs()
}

/// Signed twice-area of `a -> b -> p`: positive on one side of the directed
/// line `a -> b`, negative on the other. Public so a scenario can check which
/// way round to declare its [`Cfg::choke`].
pub fn side_of(a: Vector2, b: Vector2, p: Vector2) -> f32 {
    (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x)
}

/// Proper segment intersection: strictly opposite orientations on both sides,
/// so a shared endpoint or a collinear touch is not a crossing.
///
/// Deliberately the same formulation (f64 orientation, sign product) as
/// `assert_no_wall_crossing` in `sim.rs`'s tests. The penetration metric is
/// meant to count exactly what that assertion forbids; a slightly different
/// predicate here would make the score and the test disagree at the boundary
/// which is precisely where a unit sliding along a wall lives.
pub fn segments_cross(p1: Vector2, p2: Vector2, q1: Vector2, q2: Vector2) -> bool {
    let orient = |p: Vector2, q: Vector2, r: Vector2| {
        ((q.x - p.x) as f64 * (r.y - p.y) as f64) - ((q.y - p.y) as f64 * (r.x - p.x) as f64)
    };
    let (o1, o2) = (orient(p1, p2, q1), orient(p1, p2, q2));
    let (o3, o4) = (orient(q1, q2, p1), orient(q1, q2, p2));
    o1 * o2 < 0.0 && o3 * o4 < 0.0
}

/// Nearest-rank percentile of an already-collected sample (`q` in 0..1).
pub fn percentile(values: &mut [f64], q: f64) -> Option<f64> {
    if values.is_empty() {
        return None;
    }
    values.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let rank = ((q * values.len() as f64).ceil() as usize).clamp(1, values.len());
    Some(values[rank - 1])
}

pub fn mean(values: &[f64]) -> Option<f64> {
    (!values.is_empty()).then(|| values.iter().sum::<f64>() / values.len() as f64)
}

// ── Agent-level probe ────────────────────────────────────────────────────────

struct Tracked {
    id: UnitId,
    /// Reference optimum for this unit's route; `None` when the scenario has
    /// no ground truth for it (an unreachable goal, a combat approach).
    optimal: Option<f32>,
    /// Ticks a perfect unit would need: `optimal / (max_speed * DT)`.
    ideal_ticks: f64,
    deadline_ticks: f64,
    travelled: f64,
    arrived_tick: Option<u64>,
    // stuck counters, sampled for threshold crossings
    prev_stall: u8,
    prev_ally_stall: u8,
    prev_hold: u8,
    stall_trips: u32,
    ally_stall_trips: u32,
    hold_trips: u32,
    // path rebuilds
    prev_path: Vec<Vector2>,
    repaths: u32,
    // separation push received, summed over the run, in radii
    push_ally: f64,
    push_enemy: f64,
    // jitter
    prev_dir: Option<Vector2>,
    turn_sum: f64,
    moving_ticks: u64,
    // combat
    /// Last station this unit actually held. `None` before it has held one.
    /// Losing a station (every new order calls `clear_combat_state`) is not a
    /// change of mind, so it must not count as churn. Only walking from one
    /// held station to a *different* one does.
    prev_slot: Option<u16>,
    slot_changes: u32,
    prev_cooldown: u32,
    first_shot_tick: Option<u64>,
    /// Shots landed, and the cooldown they are landed on, so the pair can be
    /// read as a fraction of the unit's theoretical maximum rate of fire.
    shots: u32,
    cooldown_ticks: u32,
    in_range_ticks: u64,
    engaged_ticks: u64,
    /// Surface gap to the target, summed over engaged ticks and expressed in
    /// the unit's own weapon reaches, so 1.0 is exactly at maximum range.
    gap_sum: f64,
}

/// Scenario knobs the probe needs but cannot infer.
pub struct Cfg {
    /// Multiple of the ideal traversal time a unit gets to arrive in. Map size
    /// therefore never leaks into the arrival number.
    pub deadline_mult: f64,
    /// Line units are counted across, for throughput. Counted *net*, so a
    /// unit shoved back out of a doorway and in again doesn't inflate its
    /// rate. That makes the segment directed: orient it so that
    /// `side_of(a, b, goal) > 0`, and crossings toward the goal count `+1`.
    pub choke: Option<(Vector2, Vector2)>,
}

impl Default for Cfg {
    fn default() -> Cfg {
        Cfg {
            deadline_mult: 3.0,
            choke: None,
        }
    }
}

/// A `Sim` plus everything watching it.
pub struct Run {
    pub sim: Sim,
    cfg: Cfg,
    tracked: Vec<Tracked>,
    index: BTreeMap<UnitId, usize>,
    ticks: u64,
    // overlap
    overlap_depth_sum: f64,
    overlap_depth_max: f64,
    overlapping_unit_ticks: u64,
    unit_ticks: u64,
    /// Deepest overlap each unit was in, one entry per unit-tick, zeros
    /// included. The mean is diluted by every tick nothing is touching and the
    /// max is a single event out of hundreds of thousands; the percentile of
    /// this is what says how bad it is when it is bad.
    overlap_depths: Vec<f32>,
    // throughput
    net_crossings: i64,
    /// First and last tick anything crossed the choke, so the rate is the
    /// door's, not the tick budget's: a run that keeps stepping after the last
    /// unit is through would otherwise report an ever-lower throughput.
    crossing_window: Option<(u64, u64)>,
    prev_side: BTreeMap<UnitId, bool>,
    // cohesion
    gyration_sum: f64,
    gyration_samples: u64,
    last_gyration: Option<f64>,
    trace: Option<Trace>,
}

impl Run {
    pub fn new(sim: Sim, cfg: Cfg) -> Run {
        // The push split is a diagnostic the sim leaves off by default, since
        // accumulating it costs the separation pass a few percent and only the
        // harness reads it. Turned on here rather than in `main`, so it is on
        // for anything that builds a `Run` and cannot be forgotten.
        rts_lib::sim::PUSH_TRACKING.store(true, std::sync::atomic::Ordering::Relaxed);
        Run {
            sim,
            cfg,
            tracked: Vec::new(),
            index: BTreeMap::new(),
            ticks: 0,
            overlap_depth_sum: 0.0,
            overlap_depth_max: 0.0,
            overlapping_unit_ticks: 0,
            unit_ticks: 0,
            overlap_depths: Vec::new(),
            net_crossings: 0,
            crossing_window: None,
            prev_side: BTreeMap::new(),
            gyration_sum: 0.0,
            gyration_samples: 0,
            last_gyration: None,
            trace: None,
        }
    }

    /// Start dumping a per-tick trace; written by [`Run::write_trace`].
    ///
    /// Walls are read from the live navmesh rather than passed in, so the
    /// replay always draws the geometry the sim actually had, including
    /// obstacles added or removed part-way through.
    pub fn record_trace(&mut self, scenario: &str) {
        self.trace = Some(Trace {
            scenario: scenario.to_string(),
            wall_events: vec![WallEvent {
                tick: self.sim.tick(),
                segments: wall_dump(self.sim.navmesh()),
            }],
            nav_version: self.sim.navmesh().version(),
            ticks: Vec::new(),
        });
    }

    /// Watch `id` on its way to its current goal. `optimal` is the reference
    /// cost of that route; `None` when the scenario has no ground truth for it
    /// (an unreachable goal, a combat approach).
    ///
    /// The goal itself isn't passed in: `Unit::parked` already means "settled
    /// at the goal", so a second copy of it here could only disagree.
    pub fn track(&mut self, id: UnitId, optimal: Option<f32>) {
        let Some(u) = self.sim.units().get(id) else {
            return;
        };
        let ideal_ticks = optimal
            .map(|o| (o / (u.max_speed * DT)) as f64)
            .unwrap_or(0.0);
        self.index.insert(id, self.tracked.len());
        self.tracked.push(Tracked {
            id,
            optimal,
            ideal_ticks,
            deadline_ticks: ideal_ticks * self.cfg.deadline_mult,
            travelled: 0.0,
            arrived_tick: None,
            prev_stall: u.stall,
            prev_ally_stall: u.ally_stall,
            prev_hold: u.hold_ticks,
            stall_trips: 0,
            ally_stall_trips: 0,
            hold_trips: 0,
            prev_path: u.path[u.path_i as usize..].to_vec(),
            repaths: 0,
            push_ally: 0.0,
            push_enemy: 0.0,
            prev_dir: None,
            turn_sum: 0.0,
            moving_ticks: 0,
            prev_slot: (u.chase_slot != NO_SLOT).then_some(u.chase_slot),
            slot_changes: 0,
            prev_cooldown: u.cooldown_left,
            first_shot_tick: None,
            shots: 0,
            cooldown_ticks: u.attack_cooldown_ticks.max(1),
            in_range_ticks: 0,
            engaged_ticks: 0,
            gap_sum: 0.0,
        });
    }

    /// Step the sim one tick and fold the result into every probe.
    pub fn step(&mut self, commands: &[Command]) {
        self.sim.step(commands);
        self.ticks += 1;
        self.observe();
    }

    fn observe(&mut self) {
        // Relative to the start of the *run*, not to `sim.tick()`: fixture
        // setup (spawns, the first order, a settling warm-up) costs ticks that
        // have nothing to do with what is being scored.
        let tick = self.ticks;

        // Per-unit probes.
        for t in &mut self.tracked {
            // A despawned unit simply stops accumulating: its counters freeze
            // where they were and it never arrives. Every scenario here keeps
            // its tracked units alive (effectively infinite health), so this
            // is a definition, not a case any of them exercise.
            let Some(u) = self.sim.units().get(t.id) else {
                continue;
            };
            t.travelled += (u.pos - u.prev_pos).length() as f64;

            if u.parked && t.arrived_tick.is_none() {
                t.arrived_tick = Some(tick);
            }

            // A counter "trips" on the tick it first reaches the value the sim
            // acts at; the sim then resets it, so each episode counts once.
            let trip = |cur: u8, prev: u8, thresh: u8| cur >= thresh && prev < thresh;
            let stall_t = STALL_REPATH_TICKS.get();
            let ally_t = DETOUR_TICKS.get();
            let hold_t = BLOCK_TICKS.get();
            t.stall_trips += trip(u.stall, t.prev_stall, stall_t) as u32;
            t.ally_stall_trips += trip(u.ally_stall, t.prev_ally_stall, ally_t) as u32;
            t.hold_trips += trip(u.hold_ticks, t.prev_hold, hold_t) as u32;
            t.prev_stall = u.stall;
            t.prev_ally_stall = u.ally_stall;
            t.prev_hold = u.hold_ticks;

            // A path that merely advanced is a tail of last tick's; anything
            // else is a rebuild. A fresh order counts as one.
            let rest = &u.path[u.path_i as usize..];
            if !rest.is_empty() {
                let is_tail = t.prev_path.len() >= rest.len()
                    && t.prev_path[t.prev_path.len() - rest.len()..] == *rest;
                if !is_tail {
                    t.repaths += 1;
                }
            }
            t.prev_path.clear();
            t.prev_path.extend_from_slice(rest);

            // Jitter: heading change per moving tick. The threshold keeps
            // near-stationary noise (a shove against a wall) out of it.
            let vel = u.pos - u.prev_pos;
            if vel.length() > 0.05 * u.max_speed * DT {
                if let Some(prev) = t.prev_dir {
                    t.turn_sum += angle_between(prev, vel) as f64;
                    t.moving_ticks += 1;
                }
                t.prev_dir = Some(vel);
            }

            if u.chase_slot != NO_SLOT {
                if t.prev_slot.is_some_and(|prev| prev != u.chase_slot) {
                    t.slot_changes += 1;
                }
                t.prev_slot = Some(u.chase_slot);
            }
            // The cooldown only ever *rises* when a shot lands (it is set to
            // `attack_cooldown_ticks - 1` on firing and counts down otherwise),
            // so a rising edge is a shot.
            if u.cooldown_left > t.prev_cooldown {
                t.shots += 1;
                t.first_shot_tick.get_or_insert(tick);
            }
            t.prev_cooldown = u.cooldown_left;
            if let Some(target) = u.target
                && let Some(tu) = self.sim.units().get(target)
            {
                t.engaged_ticks += 1;
                let surf = (u.pos - tu.pos).length() - u.radius - tu.radius;
                if surf <= u.attack_range {
                    t.in_range_ticks += 1;
                }
                if u.attack_range > 1e-3 {
                    t.gap_sum += (surf / u.attack_range) as f64;
                }
            }
        }

        // Separation push, taken straight from the sim: it is computed
        // mid-step from positions and movement flags that are gone by the time
        // `step` returns, so it is the one quantity here that cannot be
        // re-derived by watching from outside.
        for (id, ally, enemy) in self.sim.last_push() {
            let Some(&t) = self.index.get(&id) else {
                continue;
            };
            let r = self.sim.units().get(id).map_or(1.0, |u| u.radius).max(1e-3);
            self.tracked[t].push_ally += (ally / r) as f64;
            self.tracked[t].push_enemy += (enemy / r) as f64;
        }

        // Overlap, over every live unit, not just tracked ones: a defender
        // standing in the crush is part of the packing.
        let mut positions: Vec<(Vector2, f32)> = Vec::new();
        for (_, u) in self.sim.units().iter() {
            positions.push((u.pos, u.radius));
        }
        self.unit_ticks += positions.len() as u64;
        // Per unit, the deepest overlap it is in this tick. `mean`, `p95` and
        // `max` are all statistics of *this* population, so the three columns
        // describe one distribution and share a scale: 0 is clear, 2.0 is
        // coincident centres. (They are not ordered: a short violent crush
        // followed by a long clean run puts the mean above the p95.) Summing
        // pair depths instead would scale with neighbour count rather than
        // depth, and could report a "mean overlap" larger than a whole body.
        let mut deepest = vec![0.0f32; positions.len()];
        for_each_overlapping_pair(&positions, |i, j, depth| {
            deepest[i] = deepest[i].max(depth);
            deepest[j] = deepest[j].max(depth);
        });
        for &d in &deepest {
            self.overlap_depth_sum += d as f64;
            self.overlap_depth_max = self.overlap_depth_max.max(d as f64);
        }
        self.overlapping_unit_ticks += deepest.iter().filter(|&&d| d > 0.0).count() as u64;
        self.overlap_depths.extend_from_slice(&deepest);

        // Throughput across the declared line, counted net so a unit shoved
        // back out and in again doesn't inflate the door's rate.
        if let Some((a, b)) = self.cfg.choke {
            for (id, u) in self.sim.units().iter() {
                let side = side_of(a, b, u.pos) > 0.0;
                match self.prev_side.insert(id, side) {
                    Some(prev) if prev != side && segments_cross(u.prev_pos, u.pos, a, b) => {
                        self.net_crossings += if side { 1 } else { -1 };
                        self.crossing_window = Some(match self.crossing_window {
                            Some((first, _)) => (first, tick),
                            None => (tick, tick),
                        });
                    }
                    _ => {}
                }
            }
        }

        // Cohesion: radius of gyration per flock, normalised by the radius a
        // packed disc of the same count would have, so group size cancels.
        let mut groups: BTreeMap<u32, Vec<(Vector2, f32)>> = BTreeMap::new();
        for (_, u) in self.sim.units().iter() {
            if u.group != 0 {
                groups.entry(u.group).or_default().push((u.pos, u.radius));
            }
        }
        let mut sum = 0.0;
        let mut count = 0u64;
        for members in groups.values() {
            if members.len() < 2 {
                continue;
            }
            let n = members.len() as f32;
            let mut c = Vector2::ZERO;
            for (p, _) in members {
                c += *p;
            }
            c /= n;
            let var: f32 = members.iter().map(|(p, _)| (*p - c).length_squared()).sum();
            let gyr = (var / n).sqrt();
            let r = members[0].1;
            sum += (gyr / (r * n.sqrt())) as f64;
            count += 1;
        }
        if count > 0 {
            let g = sum / count as f64;
            self.gyration_sum += g;
            self.gyration_samples += 1;
            self.last_gyration = Some(g);
        }

        if let Some(tr) = self.trace.as_mut() {
            // Geometry only moves when an obstacle is added or removed, which
            // bumps the navmesh version, so this costs one integer compare a
            // tick in the common case.
            let cdt = self.sim.navmesh();
            if cdt.version() != tr.nav_version {
                tr.nav_version = cdt.version();
                tr.wall_events.push(WallEvent {
                    tick: self.sim.tick(),
                    segments: wall_dump(cdt),
                });
            }
        }
        if let Some(tr) = &mut self.trace {
            let units = self
                .sim
                .units()
                .iter()
                .map(|(id, u)| TraceUnit {
                    id: id.raw(),
                    x: u.pos.x,
                    y: u.pos.y,
                    r: u.radius,
                    team: u.team,
                    parked: u.parked,
                    stall: u.stall,
                    ally_stall: u.ally_stall,
                    hold: u.hold_ticks,
                    engaged: u.engaged,
                    path: u.path[u.path_i as usize..]
                        .iter()
                        .map(|p| [p.x, p.y])
                        .collect(),
                })
                .collect();
            tr.ticks.push(TraceTick {
                tick: self.sim.tick(),
                units,
            });
        }
    }

    /// Write the collected trace to `<dir>/trace_<scenario>.json`; a no-op
    /// when tracing wasn't turned on.
    ///
    /// Reports the path and size on stderr, because a trace is one JSON object
    /// per unit per tick and `--trace all` can put a few hundred megabytes on
    /// disk without saying so.
    pub fn write_trace(&self, dir: &Path) -> Option<std::path::PathBuf> {
        let tr = self.trace.as_ref()?;
        std::fs::create_dir_all(dir).ok()?;
        let path = dir.join(format!("trace_{}.json", tr.scenario));
        let json = serde_json::to_string(tr).ok()?;
        let mb = json.len() as f64 / (1024.0 * 1024.0);
        std::fs::write(&path, json).ok()?;
        eprintln!(
            "  trace {} ({} ticks, {:.1} MB)",
            path.display(),
            tr.ticks.len(),
            mb
        );
        Some(path)
    }

    pub fn stats(&self) -> Stats {
        Stats::from(self)
    }
}

/// Overlap depth, in radii, below which a pair counts as merely *touching*: a
/// settled packing sits exactly at contact and f32 noise straddles zero either
/// way, so without this every parked blob would read as 100% overlapping.
const CONTACT_EPS_FRAC: f32 = 0.02;

/// Pairwise overlap via a uniform grid sized to the largest diameter, so a
/// crush of hundreds stays linear instead of quadratic.
///
/// Depth is reported in *radii* (of the smaller of the pair), not pixels, so
/// the numbers mean the same thing whatever size the units are and nothing
/// downstream has to go looking for a radius to divide by.
fn for_each_overlapping_pair(
    units: &[(Vector2, f32)],
    mut f: impl FnMut(usize, usize, f32),
) {
    if units.len() < 2 {
        return;
    }
    let max_r = units.iter().fold(0.0f32, |m, &(_, r)| m.max(r));
    let cell = (max_r * 2.0).max(1e-3);
    let mut lo = units[0].0;
    for &(p, _) in units {
        lo.x = lo.x.min(p.x);
        lo.y = lo.y.min(p.y);
    }
    let key = |p: Vector2| {
        (
            ((p.x - lo.x) / cell).floor() as i32,
            ((p.y - lo.y) / cell).floor() as i32,
        )
    };
    let mut buckets: BTreeMap<(i32, i32), Vec<usize>> = BTreeMap::new();
    for (i, &(p, _)) in units.iter().enumerate() {
        buckets.entry(key(p)).or_default().push(i);
    }
    for (&(cx, cy), here) in &buckets {
        for dy in -1..=1 {
            for dx in -1..=1 {
                let Some(there) = buckets.get(&(cx + dx, cy + dy)) else {
                    continue;
                };
                for &i in here {
                    for &j in there {
                        // Each unordered pair once, whichever cells it spans.
                        if j <= i {
                            continue;
                        }
                        let (pi, ri) = units[i];
                        let (pj, rj) = units[j];
                        let r = ri.min(rj);
                        let depth = (ri + rj - (pi - pj).length()) / r.max(1e-3);
                        if depth > CONTACT_EPS_FRAC {
                            f(i, j, depth);
                        }
                    }
                }
            }
        }
    }
}

/// Everything [`Run`] measured, in the units the scorecard reports.
pub struct Stats {
    /// Fraction of tracked units parked at their goal within the deadline.
    pub arrival: f64,
    /// Distance walked over the reference optimum, averaged over units that
    /// arrived. `None` when none did, since a unit that stopped early would
    /// otherwise score a flattering ratio below 1.
    pub detour: Option<f64>,
    /// Ticks taken over ideal ticks; a unit that never arrived is charged the
    /// whole run. p50 and p95 across units.
    ///
    /// Ideal is measured to the goal *point*, while a crowd parks at the edge
    /// of the blob around it (see `ARRIVAL_RADIUS_FACTOR`), so a large group
    /// reads slightly below 1.0 on an unobstructed march. The bias is constant
    /// per scenario, which is all a delta needs.
    pub lateness_p50: Option<f64>,
    pub lateness_p95: Option<f64>,
    /// Mean distance walked per tracked unit.
    pub travelled: f64,
    pub stall_trips: f64,
    pub ally_stall_trips: f64,
    pub hold_trips: f64,
    pub repaths: f64,
    /// Mean over every unit-tick of that unit's deepest overlap, in radii.
    /// Diluted by the ticks nothing is touching, which is what `overlap_frac`
    /// is for.
    pub overlap_mean: f64,
    /// 95th-percentile per-unit-tick overlap depth, in radii, over every
    /// unit-tick including the ones with no overlap at all.
    pub overlap_p95: f64,
    /// Deepest overlap any unit was in at any tick, in radii. 2.0 means two
    /// bodies with coincident centres.
    pub overlap_max: f64,
    /// Fraction of unit-ticks spent genuinely overlapping anything (contact
    /// alone doesn't count; see `CONTACT_EPS_FRAC`).
    pub overlap_frac: f64,
    /// Mean heading change per moving tick, in radians.
    pub jitter: f64,
    /// Mean separation push received per unit-tick, in radii, from allied and
    /// from enemy bodies respectively.
    ///
    /// This is the *shove*, where `overlap_*` is the *result*. They can move
    /// independently: a stiffer separation resolves overlap faster and pushes
    /// harder, so a change that improves one can easily worsen the other.
    pub push_ally: f64,
    pub push_enemy: f64,
    /// Mean flock radius of gyration over the run, in packed-blob radii.
    pub cohesion: Option<f64>,
    /// The same at the final tick.
    pub cohesion_final: Option<f64>,
    /// Net units per second across the declared line, over the window it was
    /// in use (first crossing to last), not over the whole tick budget.
    pub throughput: f64,
    /// Mean fraction of engaged attackers standing inside weapon reach.
    pub in_range: Option<f64>,
    /// Mean surface gap to the target while engaged, in weapon reaches: 1.0 is
    /// exactly at maximum range, above that is out of it. The continuous form
    /// of `in_range`, which a threshold hides: a group that settles two reaches
    /// back scores the same 0 on `in_range` as one that settles ten back.
    pub chase_gap: Option<f64>,
    /// p95 ticks from the first tick of the run to a unit's first shot.
    pub first_shot_p95: Option<f64>,
    /// Shots landed over shots the cooldown would have allowed, averaged over
    /// armed units. The payoff metric for a chase: a unit that keeps up and
    /// keeps firing approaches 1, one that spends the fight walking approaches
    /// 0. `None` when nothing in the scenario carries a weapon.
    pub fire_efficiency: Option<f64>,
    /// Station *switches* per attacker: times a unit walked off a station it
    /// held to take a different one. Losing a station to a new order doesn't
    /// count, so this reads the same whether or not the scenario spams orders.
    pub slot_churn: f64,
}

impl Stats {
    fn from(run: &Run) -> Stats {
        let n = run.tracked.len();
        let fdiv = |a: f64, b: usize| if b == 0 { 0.0 } else { a / b as f64 };

        let arrival = run
            .tracked
            .iter()
            .filter(|t| {
                t.arrived_tick
                    .is_some_and(|a| (a as f64) <= t.deadline_ticks.max(1.0))
            })
            .count() as f64;

        let detours: Vec<f64> = run
            .tracked
            .iter()
            .filter_map(|t| match (t.arrived_tick, t.optimal) {
                (Some(_), Some(o)) if o > 1e-3 => Some(t.travelled / o as f64),
                _ => None,
            })
            .collect();

        let mut lateness: Vec<f64> = run
            .tracked
            .iter()
            .filter(|t| t.ideal_ticks > 1.0)
            .map(|t| {
                let took = t.arrived_tick.map(|a| a as f64).unwrap_or(run.ticks as f64);
                took / t.ideal_ticks
            })
            .collect();

        let mut first_shots: Vec<f64> = run
            .tracked
            .iter()
            .filter(|t| t.prev_cooldown > 0 || t.first_shot_tick.is_some())
            .map(|t| {
                t.first_shot_tick
                    .map(|s| s as f64)
                    .unwrap_or(run.ticks as f64)
            })
            .collect();

        let in_range: Vec<f64> = run
            .tracked
            .iter()
            .filter(|t| t.engaged_ticks > 0)
            .map(|t| t.in_range_ticks as f64 / t.engaged_ticks as f64)
            .collect();

        let turn_sum: f64 = run.tracked.iter().map(|t| t.turn_sum).sum();
        let moving: u64 = run.tracked.iter().map(|t| t.moving_ticks).sum();

        // Span, not budget: +1 so a single-tick window isn't a division by zero.
        let seconds = run
            .crossing_window
            .map(|(a, b)| (b - a + 1) as f64 * DT as f64)
            .unwrap_or(0.0);

        Stats {
            arrival: fdiv(arrival, n),
            detour: mean(&detours),
            lateness_p50: percentile(&mut lateness, 0.50),
            lateness_p95: percentile(&mut lateness, 0.95),
            travelled: fdiv(run.tracked.iter().map(|t| t.travelled).sum(), n),
            // Per unit-tick, so the number does not scale with how long the
            // scenario happens to run.
            push_ally: fdiv(
                run.tracked.iter().map(|t| t.push_ally).sum::<f64>() / run.ticks.max(1) as f64,
                n,
            ),
            push_enemy: fdiv(
                run.tracked.iter().map(|t| t.push_enemy).sum::<f64>() / run.ticks.max(1) as f64,
                n,
            ),
            stall_trips: fdiv(run.tracked.iter().map(|t| t.stall_trips as f64).sum(), n),
            ally_stall_trips: fdiv(
                run.tracked.iter().map(|t| t.ally_stall_trips as f64).sum(),
                n,
            ),
            hold_trips: fdiv(run.tracked.iter().map(|t| t.hold_trips as f64).sum(), n),
            repaths: fdiv(run.tracked.iter().map(|t| t.repaths as f64).sum(), n),
            // Depths already arrive in radii, so these are comparable across
            // unit sizes and crowd sizes without further scaling.
            overlap_mean: if run.unit_ticks == 0 {
                0.0
            } else {
                run.overlap_depth_sum / run.unit_ticks as f64
            },
            overlap_p95: {
                let mut d: Vec<f64> = run.overlap_depths.iter().map(|&v| v as f64).collect();
                percentile(&mut d, 0.95).unwrap_or(0.0)
            },
            overlap_max: run.overlap_depth_max,
            overlap_frac: if run.unit_ticks == 0 {
                0.0
            } else {
                run.overlapping_unit_ticks as f64 / run.unit_ticks as f64
            },
            jitter: if moving == 0 {
                0.0
            } else {
                turn_sum / moving as f64
            },
            cohesion: (run.gyration_samples > 0)
                .then(|| run.gyration_sum / run.gyration_samples as f64),
            cohesion_final: run.last_gyration,
            throughput: if seconds > 0.0 {
                run.net_crossings as f64 / seconds
            } else {
                0.0
            },
            in_range: mean(&in_range),
            chase_gap: mean(
                &run.tracked
                    .iter()
                    .filter(|t| t.engaged_ticks > 0)
                    .map(|t| t.gap_sum / t.engaged_ticks as f64)
                    .collect::<Vec<f64>>(),
            ),
            first_shot_p95: percentile(&mut first_shots, 0.95),
            fire_efficiency: mean(
                &run.tracked
                    .iter()
                    .filter(|t| t.cooldown_ticks > 0 && t.engaged_ticks > 0)
                    .map(|t| {
                        let ceiling = run.ticks as f64 / t.cooldown_ticks as f64;
                        if ceiling > 0.0 {
                            (t.shots as f64 / ceiling).min(1.0)
                        } else {
                            0.0
                        }
                    })
                    .collect::<Vec<f64>>(),
            ),
            slot_churn: fdiv(run.tracked.iter().map(|t| t.slot_changes as f64).sum(), n),
        }
    }
}

// ── Path-level probe ─────────────────────────────────────────────────────────

/// One `find_path` query and what the reference says about it.
pub struct Query {
    pub start: Vector2,
    pub goal: Vector2,
    pub radius: f32,
    /// Reference optimum, or `None` when the reference says unreachable.
    pub reference: Option<f32>,
}

#[derive(Default)]
pub struct PathProbe {
    subopt: Vec<f64>,
    clearances: Vec<f64>,
    abstraction: Vec<f64>,
    waypoints: Vec<f64>,
    turn_per_len: Vec<f64>,
    queries: u32,
    refusals: u32,
    phantoms: u32,
}

impl PathProbe {
    /// Run `q` against `cdt`, scoring the returned polyline against the
    /// reference. Pass `abs` to also price the hierarchy.
    pub fn query(
        &mut self,
        cdt: &CDT,
        abs: Option<&rts_lib::abstraction::Abstraction>,
        scratch: &mut AStarScratch,
        q: &Query,
    ) {
        self.queries += 1;
        let path = find_path(cdt, q.start, q.goal, scratch, q.radius);
        match (path.is_empty(), q.reference) {
            // Refused a route the reference says exists.
            (true, Some(_)) => self.refusals += 1,
            // Returned one through space the reference says is too tight.
            (false, None) => self.phantoms += 1,
            _ => {}
        }
        if path.is_empty() {
            return;
        }
        let len = polyline_len(&path) as f64;
        if let Some(opt) = q.reference
            && opt > 1e-3
        {
            self.subopt.push(len / opt as f64);
        }
        if q.radius > 0.0 {
            self.clearances
                .push((path_min_clearance(cdt, &path) / q.radius) as f64);
        }
        self.waypoints.push(path.len() as f64);
        if len > 1e-3 {
            self.turn_per_len
                .push(polyline_turn(&path) as f64 / len * 100.0);
        }
        if let Some(abs) = abs {
            let ap = find_path_abstract(cdt, abs, q.start, q.goal, scratch, q.radius);
            if !ap.is_empty() && len > 1e-3 {
                self.abstraction.push(polyline_len(&ap) as f64 / len);
            }
        }
    }

    pub fn suboptimality(&self) -> Option<f64> {
        mean(&self.subopt)
    }
    pub fn refusals(&self) -> f64 {
        self.refusals as f64
    }
    pub fn phantoms(&self) -> f64 {
        self.phantoms as f64
    }
    /// Worst path clearance seen, in radii.
    pub fn min_clearance(&self) -> Option<f64> {
        self.clearances
            .iter()
            .copied()
            .fold(None, |m: Option<f64>, v| Some(m.map_or(v, |m| m.min(v))))
    }
    /// 5th-percentile path clearance, in radii: the body of the distribution,
    /// not just its worst single query.
    pub fn clearance_p5(&self) -> Option<f64> {
        percentile(&mut self.clearances.clone(), 0.05)
    }
    pub fn abstraction_penalty(&self) -> Option<f64> {
        mean(&self.abstraction)
    }
    pub fn waypoints(&self) -> Option<f64> {
        mean(&self.waypoints)
    }
    /// Total absolute turn per 100 units of path length, in radians.
    pub fn turn_per_len(&self) -> Option<f64> {
        mean(&self.turn_per_len)
    }
}

// ── Traces ───────────────────────────────────────────────────────────────────

#[derive(serde::Serialize)]
struct TraceUnit {
    id: u64,
    x: f32,
    y: f32,
    r: f32,
    team: u32,
    parked: bool,
    stall: u8,
    ally_stall: u8,
    hold: u8,
    engaged: bool,
    path: Vec<[f32; 2]>,
}

#[derive(serde::Serialize)]
struct TraceTick {
    tick: u64,
    units: Vec<TraceUnit>,
}

/// The complete wall set from `tick` onward, not a delta.
///
/// Timed rather than captured once, because a scenario can add or remove
/// obstacles mid-run: `obstacle_drop` inserts a building at tick 150, and a
/// static list drawn from frame 0 showed units walking through a wall that did
/// not exist yet.
#[derive(serde::Serialize)]
struct WallEvent {
    tick: u64,
    /// Flattened wall segments `[ax, ay, bx, by]`.
    segments: Vec<[f32; 4]>,
}

#[derive(serde::Serialize)]
struct Trace {
    scenario: String,
    wall_events: Vec<WallEvent>,
    /// Navmesh version the last event was taken at; a change means the
    /// geometry moved and a fresh event is due. Not part of the file.
    #[serde(skip)]
    nav_version: u64,
    ticks: Vec<TraceTick>,
}

// ── Shared fixture helpers ───────────────────────────────────────────────────

/// Every constrained edge of a navmesh, flattened for the trace.
fn wall_dump(cdt: &CDT) -> Vec<[f32; 4]> {
    let mut out = Vec::new();
    cdt.for_each_constrained_edge(|a, b| out.push([a.x, a.y, b.x, b.y]));
    out
}

/// Build a prepared CDT the same way the sim does, for path-level probes.
pub fn build_cdt(points: Vec<Vector2>, constraints: &[(u32, u32)]) -> CDT {
    let mut cdt = CDT::from_points(points);
    for &(a, b) in constraints {
        cdt.insert_constraint(a, b);
    }
    cdt.remove_super_triangle();
    cdt.build_grid_index();
    cdt.compute_widths();
    cdt
}

pub fn wall_segments(points: &[Vector2], constraints: &[(u32, u32)]) -> Vec<(Vector2, Vector2)> {
    constraints
        .iter()
        .map(|&(a, b)| (points[a as usize], points[b as usize]))
        .collect()
}

/// Every unit id in slot order.
pub fn unit_ids(sim: &Sim) -> Vec<UnitId> {
    sim.units().iter().map(|(id, _)| id).collect()
}

/// The default fixture unit: unarmed, so movement scenarios measure movement.
pub fn spawn_cmd(pos: Vector2, radius: f32, speed: f32) -> Command {
    Command::Spawn {
        pos,
        radius,
        max_speed: speed,
        team: 0,
        max_health: f32::MAX,
        damage: 0.0,
        attack_range: 0.0,
        attack_cooldown_ticks: 1,
    }
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f32, y: f32) -> Vector2 {
        Vector2::new(x, y)
    }

    #[test]
    fn test_segments_cross_is_proper_intersection_only() {
        assert!(segments_cross(v(0.0, 0.0), v(10.0, 10.0), v(0.0, 10.0), v(10.0, 0.0)));
        assert!(!segments_cross(v(0.0, 0.0), v(1.0, 1.0), v(5.0, 5.0), v(6.0, 6.0)));
        // Touching at an endpoint is not a crossing: a unit sliding along a
        // wall must not read as having gone through it.
        assert!(!segments_cross(v(0.0, 0.0), v(5.0, 0.0), v(5.0, 0.0), v(5.0, 5.0)));
        // Collinear overlap is not a crossing either.
        assert!(!segments_cross(v(0.0, 0.0), v(10.0, 0.0), v(5.0, 0.0), v(15.0, 0.0)));
    }

    #[test]
    fn test_side_of_orients_the_choke_line() {
        // The convention `Cfg::choke` documents: orient so the goal is positive.
        let (a, b) = (v(300.0, 215.0), v(300.0, 185.0));
        assert!(side_of(a, b, v(450.0, 200.0)) > 0.0, "goal side");
        assert!(side_of(a, b, v(150.0, 200.0)) < 0.0, "start side");
    }

    #[test]
    fn test_polyline_len_and_turn() {
        assert_eq!(polyline_len(&[v(0.0, 0.0), v(3.0, 0.0), v(3.0, 4.0)]), 7.0);
        assert_eq!(polyline_len(&[v(1.0, 1.0)]), 0.0);
        // One right-angle corner.
        let turn = polyline_turn(&[v(0.0, 0.0), v(3.0, 0.0), v(3.0, 4.0)]);
        assert!((turn - std::f32::consts::FRAC_PI_2).abs() < 1e-5, "{turn}");
        // A straight line has no turn, however many points describe it.
        assert!(polyline_turn(&[v(0.0, 0.0), v(1.0, 0.0), v(2.0, 0.0)]).abs() < 1e-5);
    }

    #[test]
    fn test_percentile_is_nearest_rank() {
        let mut xs = vec![5.0, 1.0, 4.0, 2.0, 3.0];
        assert_eq!(percentile(&mut xs.clone(), 0.0), Some(1.0));
        assert_eq!(percentile(&mut xs.clone(), 0.5), Some(3.0));
        assert_eq!(percentile(&mut xs, 1.0), Some(5.0));
        assert_eq!(percentile(&mut [], 0.5), None);
        assert_eq!(mean(&[1.0, 2.0, 6.0]), Some(3.0));
        assert_eq!(mean(&[]), None);
    }

    #[test]
    fn test_overlapping_pairs_ignore_mere_contact() {
        // Exactly touching: not an overlap.
        let touching = [(v(0.0, 0.0), 5.0), (v(10.0, 0.0), 5.0)];
        let mut hits = 0;
        for_each_overlapping_pair(&touching, |_, _, _| hits += 1);
        assert_eq!(hits, 0, "a settled packing must not read as overlapping");

        // Genuinely interpenetrating: one pair, at the right depth.
        let overlapping = [(v(0.0, 0.0), 5.0), (v(6.0, 0.0), 5.0)];
        let mut found = Vec::new();
        for_each_overlapping_pair(&overlapping, |i, j, d| found.push((i, j, d)));
        assert_eq!(found.len(), 1);
        // 4px of interpenetration between radius-5 bodies is 0.8 radii.
        assert!((found[0].2 - 0.8).abs() < 1e-4, "{found:?}");
    }

    /// End to end through a real `Sim`, because overlap is the metric the
    /// scorecard leans on hardest. The three columns are statistics of one
    /// population (per-unit-tick deepest overlap, in radii), so they share a
    /// scale and a cap: 2.0 radii is coincident centres and nothing can exceed
    /// it. They are deliberately *not* asserted to be ordered; a short crush
    /// followed by a long clean run legitimately puts the mean above the p95,
    /// which is exactly what this fixture produces.
    #[test]
    fn test_overlap_stats_describe_one_distribution() {
        let (points, constraints) = crate::maps::box_map(200.0, 200.0);
        let mut run = Run::new(
            Sim::new(points, &constraints, 1),
            Cfg::default(),
        );
        // Three bodies spawned almost on top of each other: deep overlap that
        // separation then resolves, so the run spans both regimes.
        run.sim.step(&[
            spawn_cmd(Vector2::new(100.0, 100.0), 5.0, 20.0),
            spawn_cmd(Vector2::new(100.5, 100.0), 5.0, 20.0),
            spawn_cmd(Vector2::new(100.0, 100.5), 5.0, 20.0),
        ]);
        for id in unit_ids(&run.sim) {
            run.track(id, None);
        }
        for _ in 0..120 {
            run.step(&[]);
        }
        let s = run.stats();
        assert!(s.overlap_max > 1.0, "spawned coincident: {}", s.overlap_max);
        assert!(s.overlap_max <= 2.0, "2.0 radii is coincident centres, the cap");
        for (name, v) in [
            ("mean", s.overlap_mean),
            ("p95", s.overlap_p95),
            ("max", s.overlap_max),
        ] {
            assert!(
                (0.0..=2.0).contains(&v),
                "{name} {v} is outside the 0..2 radii scale"
            );
        }
        assert!(s.overlap_mean <= s.overlap_max);
        assert!(s.overlap_p95 <= s.overlap_max);
        assert!(s.overlap_frac > 0.0 && s.overlap_frac <= 1.0);
        // Separation resolves the spawn crush rather than tolerating it: the
        // typical tick ends up far cleaner than the worst one.
        assert!(
            s.overlap_mean < s.overlap_max * 0.2,
            "mean {} vs max {}",
            s.overlap_mean,
            s.overlap_max
        );
    }

    #[test]
    fn test_overlapping_pairs_counted_once_across_grid_cells() {
        // Three mutually overlapping units spanning more than one grid cell:
        // three pairs, each reported exactly once.
        let units = [
            (v(0.0, 0.0), 5.0),
            (v(6.0, 0.0), 5.0),
            (v(12.0, 0.0), 5.0),
        ];
        let mut pairs = Vec::new();
        for_each_overlapping_pair(&units, |i, j, _| pairs.push((i, j)));
        pairs.sort();
        assert_eq!(pairs, vec![(0, 1), (1, 2)]);
    }
}
