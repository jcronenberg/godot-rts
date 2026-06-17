//! Deterministic fixed-tick unit simulation (see `simulation_plan.md`).
//!
//! `Sim::step` is a pure state transition: commands in, state out. Pacing,
//! threading and snapshots live in [`crate::sim_runner`]. Determinism rules:
//! f32 add/mul/div/sqrt only, slot-order iteration, no hash iteration over
//! state, pairwise forces buffered before application, one seeded RNG.

use godot::prelude::Vector2;

use crate::abstraction::Abstraction;
use crate::astar::{AStarScratch, closest_on_segment, find_path_abstract};
use crate::delaunay::{CDT, FNV_OFFSET, FNV_PRIME};
use crate::navmesh::{DynamicNavmesh, Obstacle, ObstacleId};

/// Simulation ticks per second.
pub const TICK_RATE: u32 = 30;
/// Fixed timestep in seconds.
pub const DT: f32 = 1.0 / TICK_RATE as f32;
/// Fraction of pairwise overlap corrected per tick. Firm enough to keep pace
/// with path convergence (so groups pack without lingering overlap), but below
/// the point where summed pushes in a clump overshoot and ping-pong.
const SEPARATION_RELAX: f32 = 0.6;
/// Per-tick separation displacement cap, as a fraction of the unit's speed.
/// Above a full step, so separation can out-push a unit's own path/cohesion
/// convergence and resolve overlap rather than tolerate it.
const SEPARATION_MAX_FRAC: f32 = 1.5;
/// Cohesion radius as a multiple of the largest unit radius: same-group
/// neighbours within it pull toward their shared centroid.
const COHESION_RADIUS_FRAC: f32 = 5.0;
/// Per-tick fraction of the centroid offset applied as a cohesion pull.
const COHESION_GAIN: f32 = 0.05;
/// Per-tick cohesion displacement cap, as a fraction of the unit's speed. Kept
/// well below the path step (and the separation cap) so cohesion stays a gentle
/// bias and never pulls group-mates back into overlap.
const COHESION_MAX_FRAC: f32 = 0.15;
/// A moving unit joins a parked group-mate's cluster (and stops) when within
/// this multiple of touching distance of it — *and* within its arrival radius
/// of the goal (below). So a group settles into a blob around its goal instead
/// of every unit driving to the exact goal point and crushing inward.
const ARRIVAL_TOUCH_FRAC: f32 = 1.15;
/// A group's arrival radius is `r * max(ARRIVAL_MIN_RADII, FACTOR*sqrt(N))`.
/// A unit only crowd-stops once inside it; units still outside keep pushing
/// toward the goal, so they pack into a tight central core (which separation
/// then expands cleanly and symmetrically) and the blob centres on the goal
/// instead of tailing back toward the approach side. Below the packed radius
/// on purpose; the `MIN` floor keeps small groups (whose followers sit ~2r
/// out) able to stop at all.
const ARRIVAL_RADIUS_FACTOR: f32 = 0.7;
const ARRIVAL_MIN_RADII: f32 = 3.0;
/// Wall-clamped ticks without progress before a moving unit is treated as stuck
/// (shoved off its path onto a corner) and repathed from its current position.
/// ~0.27 s at 30 Hz — long enough to ignore transient clamps.
const STALL_REPATH_TICKS: u8 = 8;
/// Remaining-path-length improvement that counts as real progress (and resets
/// the stall counter); below it the unit is treated as not advancing.
const STALL_PROGRESS_EPS: f32 = 0.1;

// ── RNG ───────────────────────────────────────────────────────────────────────

/// PCG-XSH-RR 32-bit generator (O'Neill 2014); the sim's only randomness.
#[derive(Clone, Debug)]
pub struct Pcg32 {
    state: u64,
    inc: u64,
}

impl Pcg32 {
    pub fn new(seed: u64) -> Pcg32 {
        let mut rng = Pcg32 {
            state: 0,
            inc: 0xda3e_39cb_94b9_5bdb, // default stream, forced odd
        };
        rng.next_u32();
        rng.state = rng.state.wrapping_add(seed);
        rng.next_u32();
        rng
    }

    pub fn next_u32(&mut self) -> u32 {
        let old = self.state;
        self.state = old
            .wrapping_mul(6364136223846793005)
            .wrapping_add(self.inc | 1);
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rot = (old >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    /// Uniform in `[0, 1)` with 24 bits of precision.
    pub fn next_f32(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 * (1.0 / 16_777_216.0)
    }

    fn hash_into(&self, h: &mut Fnv) {
        h.write_u64(self.state);
        h.write_u64(self.inc);
    }
}

// ── Units: slot storage with generational ids ────────────────────────────────

/// Generational unit handle; stale ids (despawned slots) are ignored.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct UnitId {
    index: u32,
    generation: u32,
}

impl UnitId {
    /// Packed form for FFI layers; pairs with [`UnitId::from_raw`].
    pub fn raw(self) -> u64 {
        (self.generation as u64) << 32 | self.index as u64
    }

    pub fn from_raw(raw: u64) -> UnitId {
        UnitId {
            index: raw as u32,
            generation: (raw >> 32) as u32,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Unit {
    pub pos: Vector2,
    /// Position at the start of the current tick (velocity = `(pos - prev_pos) / DT`).
    pub prev_pos: Vector2,
    pub radius: f32,
    pub max_speed: f32,
    /// Remaining waypoints `[next, …, goal]`; empty when idle.
    pub path: Vec<Vector2>,
    /// Index of the next waypoint to reach.
    pub path_i: u32,
    /// Cohesion group, stamped per `Move` command; 0 = ungrouped (no cohesion).
    pub group: u32,
    /// Settled at the goal — either reached it, or joined a parked group-mate's
    /// cluster (crowd-arrival). Cleared on a new `Move`. Distinguishes a real
    /// goal arrival from a unit that merely idles (e.g. unreachable goal), so
    /// only true arrivals seed the cluster others stop against.
    pub parked: bool,
    /// Distance from the goal within which this unit may crowd-stop — sized to
    /// the group so the settled blob centres on the goal. Stamped per `Move`.
    pub arrival_r: f32,
    /// Ticks wall-clamped without beating [`Unit::min_remaining`]. A group
    /// shove can push a unit off its cleared path so its line to the next
    /// waypoint cuts a corner; once this trips it repaths from where it
    /// actually is. Reset on real progress or a new path.
    pub stall: u8,
    /// Best (smallest) remaining path length seen since the path was set — the
    /// jitter-proof progress yardstick for the stall detector (`MAX` = unset).
    pub min_remaining: f32,
}

impl Unit {
    pub fn is_moving(&self) -> bool {
        !self.path.is_empty()
    }

    /// Next waypoint while moving, own position when idle.
    pub fn waypoint(&self) -> Vector2 {
        match self.path.get(self.path_i as usize) {
            Some(&w) => w,
            None => self.pos,
        }
    }

    /// Polyline distance from the current position through all remaining
    /// waypoints — monotonically decreasing with real progress, so a flat value
    /// means the unit isn't advancing. Cheap: paths hold a handful of points.
    fn remaining_len(&self) -> f32 {
        let mut total = 0.0;
        let mut prev = self.pos;
        for &p in &self.path[self.path_i as usize..] {
            let (dx, dy) = (p.x - prev.x, p.y - prev.y);
            total += (dx * dx + dy * dy).sqrt();
            prev = p;
        }
        total
    }
}

/// Slot storage iterated in slot order — stable, hash-free determinism.
#[derive(Default)]
pub struct Units {
    slots: Vec<Option<Unit>>,
    generations: Vec<u32>,
    free: Vec<u32>,
    len: usize,
}

impl Units {
    pub fn spawn(&mut self, unit: Unit) -> UnitId {
        self.len += 1;
        match self.free.pop() {
            Some(i) => {
                self.slots[i as usize] = Some(unit);
                UnitId {
                    index: i,
                    generation: self.generations[i as usize],
                }
            }
            None => {
                self.slots.push(Some(unit));
                self.generations.push(0);
                UnitId {
                    index: (self.slots.len() - 1) as u32,
                    generation: 0,
                }
            }
        }
    }

    pub fn despawn(&mut self, id: UnitId) -> bool {
        match self.slots.get_mut(id.index as usize) {
            Some(slot @ Some(_)) if self.generations[id.index as usize] == id.generation => {
                *slot = None;
                self.generations[id.index as usize] += 1;
                self.free.push(id.index);
                self.len -= 1;
                true
            }
            _ => false,
        }
    }

    pub fn get(&self, id: UnitId) -> Option<&Unit> {
        match self.slots.get(id.index as usize) {
            Some(Some(u)) if self.generations[id.index as usize] == id.generation => Some(u),
            _ => None,
        }
    }

    pub fn get_mut(&mut self, id: UnitId) -> Option<&mut Unit> {
        match self.slots.get_mut(id.index as usize) {
            Some(Some(u)) if self.generations[id.index as usize] == id.generation => Some(u),
            _ => None,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn iter(&self) -> impl Iterator<Item = (UnitId, &Unit)> {
        self.slots.iter().enumerate().filter_map(|(i, s)| {
            s.as_ref().map(|u| {
                (
                    UnitId {
                        index: i as u32,
                        generation: self.generations[i],
                    },
                    u,
                )
            })
        })
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (UnitId, &mut Unit)> {
        self.slots.iter_mut().enumerate().filter_map(|(i, s)| {
            s.as_mut().map(|u| {
                (
                    UnitId {
                        index: i as u32,
                        generation: self.generations[i],
                    },
                    u,
                )
            })
        })
    }
}

// ── Commands ──────────────────────────────────────────────────────────────────

/// All sim mutation goes through commands, batched per tick — the replay,
/// determinism-test and lockstep seam.
#[derive(Clone, Debug)]
pub enum Command {
    Spawn {
        pos: Vector2,
        radius: f32,
        speed: f32,
    },
    Move {
        units: Vec<UnitId>,
        goal: Vector2,
    },
    AddObstacle {
        points: Vec<Vector2>,
    },
    RemoveObstacle {
        id: ObstacleId,
    },
}

// ── Spatial grid (separation broad-phase) ─────────────────────────────────────

/// Uniform grid over alive-unit positions, rebuilt each tick by counting sort
/// into flat persistent vecs (allocation-free in steady state).
#[derive(Default)]
struct SpatialGrid {
    origin: Vector2,
    inv_cell: f32,
    cols: u32,
    rows: u32,
    /// Prefix sums per cell into `entries`; length `cols * rows + 1`.
    starts: Vec<u32>,
    /// Dense-unit indices sorted by cell.
    entries: Vec<u32>,
    /// Scratch: cell index per dense unit.
    cell_of: Vec<u32>,
}

impl SpatialGrid {
    /// `cell_size` should be at least the largest unit diameter so all
    /// overlapping pairs sit in adjacent cells.
    fn rebuild(&mut self, positions: &[Vector2], cell_size: f32) {
        let n = positions.len();
        self.entries.clear();
        self.cell_of.clear();
        if n == 0 {
            self.cols = 0;
            self.rows = 0;
            return;
        }
        let (mut min, mut max) = (positions[0], positions[0]);
        for &p in positions {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
        }
        self.origin = min;
        self.inv_cell = 1.0 / cell_size;
        self.cols = (((max.x - min.x) * self.inv_cell) as u32 + 1).min(2048);
        self.rows = (((max.y - min.y) * self.inv_cell) as u32 + 1).min(2048);

        let cells = (self.cols * self.rows) as usize;
        self.starts.clear();
        self.starts.resize(cells + 1, 0);
        for &p in positions {
            let c = self.cell_index(p);
            self.cell_of.push(c);
            self.starts[c as usize + 1] += 1;
        }
        for i in 1..=cells {
            self.starts[i] += self.starts[i - 1];
        }
        self.entries.resize(n, 0);
        // Scatter via a moving cursor per cell; restore prefix sums after.
        for i in 0..n {
            let c = self.cell_of[i] as usize;
            self.entries[self.starts[c] as usize] = i as u32;
            self.starts[c] += 1;
        }
        for i in (1..=cells).rev() {
            self.starts[i] = self.starts[i - 1];
        }
        self.starts[0] = 0;
    }

    fn cell_coords(&self, p: Vector2) -> (u32, u32) {
        let cx = (((p.x - self.origin.x) * self.inv_cell) as u32).min(self.cols - 1);
        let cy = (((p.y - self.origin.y) * self.inv_cell) as u32).min(self.rows - 1);
        (cx, cy)
    }

    fn cell_index(&self, p: Vector2) -> u32 {
        let (cx, cy) = self.cell_coords(p);
        cy * self.cols + cx
    }

    fn cell_entries(&self, cx: u32, cy: u32) -> &[u32] {
        let c = (cy * self.cols + cx) as usize;
        let (a, b) = (self.starts[c] as usize, self.starts[c + 1] as usize);
        &self.entries[a..b]
    }
}

// ── FNV-1a hashing ────────────────────────────────────────────────────────────

struct Fnv(u64);

impl Fnv {
    fn new() -> Fnv {
        Fnv(FNV_OFFSET)
    }

    fn write_u64(&mut self, v: u64) {
        for b in v.to_le_bytes() {
            self.0 ^= b as u64;
            self.0 = self.0.wrapping_mul(FNV_PRIME);
        }
    }

    fn write_f32(&mut self, v: f32) {
        self.write_u64(v.to_bits() as u64);
    }

    fn write_v2(&mut self, v: Vector2) {
        self.write_u64((v.x.to_bits() as u64) << 32 | v.y.to_bits() as u64);
    }
}

// ── Sim ───────────────────────────────────────────────────────────────────────

/// Buffers reused across ticks so steady-state `step` never allocates.
#[derive(Default)]
struct StepScratch {
    /// Dense arrays over alive units, rebuilt each tick in slot order.
    ids: Vec<UnitId>,
    positions: Vec<Vector2>,
    radii: Vec<f32>,
    speeds: Vec<f32>,
    /// Separation+cohesion displacement accumulator, parallel to the dense arrays.
    disp: Vec<Vector2>,
    /// Per dense unit: its group, and whether it is moving / parked at goal.
    groups: Vec<u32>,
    moving: Vec<bool>,
    parked: Vec<bool>,
    /// Whether a moving unit is within its arrival radius of its goal (so it
    /// may crowd-stop); false for idle/parked units.
    within_arrival: Vec<bool>,
    /// Crowd-arrival marks: set when a moving unit touches a parked group-mate.
    arrive: Vec<bool>,
    /// Sum of same-group moving-neighbour positions, and their count, per unit.
    coh_sum: Vec<Vector2>,
    coh_n: Vec<u32>,
    /// Wall-clamp face frontier and visited list (tiny per unit).
    faces: Vec<u32>,
    visited: Vec<u32>,
    /// Units the wall clamp found stuck this tick, to repath (usually empty).
    repath: Vec<UnitId>,
}

pub struct Sim {
    tick: u64,
    nav: DynamicNavmesh,
    /// Always matches `nav.navmesh().version()` after construction/rebuild.
    abstraction: Abstraction,
    abstraction_version: u64,
    units: Units,
    scratch: AStarScratch,
    rng: Pcg32,
    grid: SpatialGrid,
    step_scratch: StepScratch,
    /// Monotonic group id; bumped per `Move`, stamped onto its units.
    group_seq: u32,
}

impl Sim {
    /// Build a sim over static map geometry (`constraints` index into `points`).
    pub fn new(points: Vec<Vector2>, constraints: &[(u32, u32)], seed: u64) -> Sim {
        let nav = DynamicNavmesh::new(points, constraints);
        let abstraction = Abstraction::build(nav.navmesh());
        let abstraction_version = nav.navmesh().version();
        Sim {
            tick: 0,
            nav,
            abstraction,
            abstraction_version,
            units: Units::default(),
            scratch: AStarScratch::new(),
            rng: Pcg32::new(seed),
            grid: SpatialGrid::default(),
            step_scratch: StepScratch::default(),
            group_seq: 0,
        }
    }

    pub fn tick(&self) -> u64 {
        self.tick
    }

    pub fn units(&self) -> &Units {
        &self.units
    }

    pub fn navmesh(&self) -> &CDT {
        self.nav.navmesh()
    }

    /// Id the next `AddObstacle` command will assign.
    pub fn next_obstacle_id(&self) -> u64 {
        self.nav.next_obstacle_id()
    }

    /// Advance one fixed tick, applying all commands queued since the last.
    ///
    /// Order: store prev positions → apply commands → rebuild navmesh if the
    /// obstacle set changed (refresh abstraction, repath all moving units) →
    /// integrate along paths → flock (separation + cohesion) → wall clamp →
    /// repath units stuck against a corner → advance tick.
    pub fn step(&mut self, commands: &[Command]) {
        for (_, u) in self.units.iter_mut() {
            u.prev_pos = u.pos;
        }
        for cmd in commands {
            self.apply(cmd);
        }
        let mesh_changed = self.rebuild_and_repath();
        self.integrate();
        self.flock();
        self.wall_clamp(mesh_changed);
        self.repath_stalled();
        self.tick += 1;
    }

    fn apply(&mut self, cmd: &Command) {
        match cmd {
            Command::Spawn { pos, radius, speed } => {
                self.units.spawn(Unit {
                    pos: *pos,
                    prev_pos: *pos,
                    radius: *radius,
                    max_speed: *speed,
                    path: Vec::new(),
                    path_i: 0,
                    group: 0,
                    parked: false,
                    arrival_r: 0.0,
                    stall: 0,
                    min_remaining: f32::MAX,
                });
            }
            Command::Move { units, goal } => {
                // One group per Move: its units cohere, separate orders don't.
                // wrapping_add keeps group_seq away from 0 (the ungrouped sentinel).
                self.group_seq = self.group_seq.wrapping_add(1).max(1);
                let group = self.group_seq;
                // Arrival radius: scales with the live group's packed-disk radius,
                // floored so small groups can still crowd-stop. Count only IDs that
                // resolve to live units — stale IDs are silently skipped below.
                let live_n = units.iter().filter(|&&id| self.units.get(id).is_some()).count();
                let radii = (ARRIVAL_RADIUS_FACTOR * (live_n as f32).sqrt())
                    .max(ARRIVAL_MIN_RADII);
                let cdt = self.nav.navmesh();
                for &id in units {
                    let Some(unit) = self.units.get_mut(id) else {
                        continue;
                    };
                    let path = find_path_abstract(
                        cdt,
                        &self.abstraction,
                        unit.pos,
                        *goal,
                        &mut self.scratch,
                        unit.radius,
                    );
                    unit.group = group;
                    unit.arrival_r = unit.radius * radii;
                    set_path(unit, path);
                }
            }
            Command::AddObstacle { points } => {
                self.nav.add_obstacle(Obstacle::polygon(points.clone()));
            }
            Command::RemoveObstacle { id } => {
                self.nav.remove_obstacle(*id);
            }
        }
    }

    /// Rebuild the navmesh after obstacle changes; on a version move, refresh
    /// the abstraction and repath every moving unit toward its existing goal
    /// (phase-1 policy: repath all, no hysteresis). Returns whether the mesh
    /// actually changed.
    fn rebuild_and_repath(&mut self) -> bool {
        if !self.nav.is_dirty() {
            return false;
        }
        let cdt = self.nav.rebuild();
        if cdt.version() == self.abstraction_version {
            return false;
        }
        self.abstraction = Abstraction::build(cdt);
        self.abstraction_version = cdt.version();
        for (_, unit) in self.units.iter_mut() {
            let Some(&goal) = unit.path.last() else {
                continue;
            };
            let path = find_path_abstract(
                cdt,
                &self.abstraction,
                unit.pos,
                goal,
                &mut self.scratch,
                unit.radius,
            );
            set_path(unit, path);
        }
        true
    }

    /// Advance each moving unit `max_speed * DT` along its polyline, carrying
    /// leftover distance across waypoints; snap and idle at the end.
    fn integrate(&mut self) {
        for (_, unit) in self.units.iter_mut() {
            if unit.path.is_empty() {
                continue;
            }
            let mut remaining = unit.max_speed * DT;
            while remaining > 0.0 {
                // Waypoints exhausted: the post-loop guard clears the path.
                let Some(&target) = unit.path.get(unit.path_i as usize) else {
                    break;
                };
                let delta = target - unit.pos;
                let dist = (delta.x * delta.x + delta.y * delta.y).sqrt();
                if dist <= remaining {
                    unit.pos = target;
                    unit.path_i += 1;
                    remaining -= dist;
                } else {
                    unit.pos += delta * (remaining / dist);
                    break;
                }
            }
            if unit.path_i as usize >= unit.path.len() {
                unit.path.clear();
                unit.path_i = 0;
                unit.parked = true; // reached the goal: seed for crowd-arrival
            }
        }
    }

    /// One grid pass over start-of-tick positions, deriving three per-unit
    /// effects applied afterwards (fixed order = bit-exact):
    ///
    /// - **Separation** (all neighbours, at `r_i + r_j`): corrects
    ///   [`SEPARATION_RELAX`] of each overlap per tick.
    /// - **Cohesion** (same-group moving neighbours, at [`COHESION_RADIUS_FRAC`]
    ///   radii): pulls toward the neighbour centroid by [`COHESION_GAIN`], with
    ///   any heading-opposing component dropped so stragglers rejoin but
    ///   leaders are never braked.
    /// - **Crowd-arrival** (same-group, at [`ARRIVAL_TOUCH_FRAC`] of touching):
    ///   a moving unit touching a *parked* group-mate parks too, so a group
    ///   settles into a blob at its goal rather than each unit crushing toward
    ///   the exact goal point.
    ///
    /// The combined displacement is capped at [`SEPARATION_MAX_FRAC`] of the
    /// unit's own step, well below the path advance, so pathing always wins.
    fn flock(&mut self) {
        let s = &mut self.step_scratch;
        s.ids.clear();
        s.positions.clear();
        s.radii.clear();
        s.speeds.clear();
        s.disp.clear();
        s.groups.clear();
        s.moving.clear();
        s.parked.clear();
        s.within_arrival.clear();
        s.arrive.clear();
        s.coh_sum.clear();
        s.coh_n.clear();
        let mut max_radius = 0.0f32;
        for (id, u) in self.units.iter() {
            s.ids.push(id);
            s.positions.push(u.prev_pos);
            s.radii.push(u.radius);
            s.speeds.push(u.max_speed);
            s.disp.push(Vector2::ZERO);
            s.groups.push(u.group);
            s.moving.push(u.is_moving());
            s.parked.push(u.parked);
            // Within arrival radius of the goal? (Moving units only.)
            let within = match u.path.last() {
                Some(&g) if u.arrival_r > 0.0 => {
                    let (dx, dy) = (u.pos.x - g.x, u.pos.y - g.y);
                    dx * dx + dy * dy < u.arrival_r * u.arrival_r
                }
                _ => false,
            };
            s.within_arrival.push(within);
            s.arrive.push(false);
            s.coh_sum.push(Vector2::ZERO);
            s.coh_n.push(0);
            max_radius = max_radius.max(u.radius);
        }
        if s.ids.len() < 2 || max_radius <= 0.0 {
            return;
        }
        let max_diameter = max_radius * 2.0;
        let r_coh = max_radius * COHESION_RADIUS_FRAC;
        let r_coh2 = r_coh * r_coh;
        // Cell covers the larger radius so the 3×3 scan still finds every pair.
        self.grid.rebuild(&s.positions, max_diameter.max(r_coh));

        for i in 0..s.ids.len() {
            let p = s.positions[i];
            let r_i = s.radii[i];
            let g_i = s.groups[i];
            let mv_i = s.moving[i];
            let pk_i = s.parked[i];
            let (cx, cy) = self.grid.cell_coords(p);
            for ny in cy.saturating_sub(1)..=(cy + 1).min(self.grid.rows - 1) {
                for nx in cx.saturating_sub(1)..=(cx + 1).min(self.grid.cols - 1) {
                    for &j in self.grid.cell_entries(nx, ny) {
                        let j = j as usize;
                        if j <= i {
                            continue;
                        }
                        let delta = p - s.positions[j];
                        let d2 = delta.x * delta.x + delta.y * delta.y;
                        let min_dist = r_i + s.radii[j];
                        if d2 < min_dist * min_dist {
                            let d = d2.sqrt();
                            // Coincident circles: deterministic x-axis tiebreak.
                            let dir = if d > 1e-6 {
                                delta * (1.0 / d)
                            } else {
                                Vector2::new(1.0, 0.0)
                            };
                            let push = dir * ((min_dist - d) * 0.5 * SEPARATION_RELAX);
                            s.disp[i] += push;
                            s.disp[j] -= push;
                        }
                        if g_i == 0 || g_i != s.groups[j] {
                            continue; // remaining effects are same-group only
                        }
                        // Cohesion: between two moving group-mates.
                        // Store the offset (neighbor - self) so the apply loop can
                        // compute the pull directly without subtracting own position.
                        if mv_i && s.moving[j] && d2 < r_coh2 {
                            s.coh_sum[i] += s.positions[j] - p;
                            s.coh_sum[j] += p - s.positions[j];
                            s.coh_n[i] += 1;
                            s.coh_n[j] += 1;
                        }
                        // Crowd-arrival: a moving unit that is within its
                        // arrival radius of the goal and touches a parked
                        // group-mate parks too (whichever side is moving). The
                        // radius gate lets units still far from the goal keep
                        // pushing in, so the blob centres rather than tailing
                        // back along the approach.
                        let touch = min_dist * ARRIVAL_TOUCH_FRAC;
                        if d2 < touch * touch {
                            if mv_i && s.within_arrival[i] && s.parked[j] {
                                s.arrive[i] = true;
                            } else if pk_i && s.moving[j] && s.within_arrival[j] {
                                s.arrive[j] = true;
                            }
                        }
                    }
                }
            }
        }

        for i in 0..s.ids.len() {
            let unit = self.units.get_mut(s.ids[i]).expect("dense id alive");
            if s.arrive[i] {
                // Stop where it is, against the cluster — don't drive to centre.
                unit.path.clear();
                unit.path_i = 0;
                unit.parked = true;
            }
            let mut d = s.disp[i];
            if !s.arrive[i] && s.coh_n[i] > 0 {
                let mut pull = s.coh_sum[i] * (COHESION_GAIN / s.coh_n[i] as f32);
                // Zero any pull opposing the heading: rejoin laterally/forward,
                // never brake. coh_n[i] > 0 implies the unit is moving.
                let h = unit.waypoint() - unit.pos;
                let h2 = h.x * h.x + h.y * h.y;
                if h2 > 1e-12 {
                    let along = (pull.x * h.x + pull.y * h.y) / h2;
                    if along < 0.0 {
                        pull -= h * along;
                    }
                }
                // Cap cohesion on its own (small) budget so it can't overpower
                // separation and pull group-mates back into overlap.
                let coh_cap = s.speeds[i] * DT * COHESION_MAX_FRAC;
                let pl2 = pull.x * pull.x + pull.y * pull.y;
                if pl2 > coh_cap * coh_cap {
                    pull *= coh_cap / pl2.sqrt();
                }
                d += pull;
            }
            let len2 = d.x * d.x + d.y * d.y;
            if len2 == 0.0 {
                continue;
            }
            let max_step = s.speeds[i] * DT * SEPARATION_MAX_FRAC;
            let len = len2.sqrt();
            if len > max_step {
                d *= max_step / len;
            }
            unit.pos += d;
        }
    }

    /// Project unit circles out of nearby constrained edges. Paths already
    /// respect radius; this only cleans up separation pushes, so units that
    /// didn't move this tick are skipped — unless the mesh changed, which can
    /// put a new wall under a stationary unit.
    fn wall_clamp(&mut self, mesh_changed: bool) {
        let cdt = self.nav.navmesh();
        let s = &mut self.step_scratch;
        for (id, unit) in self.units.iter_mut() {
            if unit.radius <= 0.0 || (!mesh_changed && unit.pos == unit.prev_pos) {
                continue;
            }
            // A push moves the circle and changes which faces it overlaps,
            // invalidating the walk in progress; re-walk until a pass applies
            // no push (bounded against corner ping-pong). `out` accumulates the
            // outward wall normals so we can tell a wall the unit faces from one
            // merely beside it. `pushed_any` tracks whether any pass pushed, so
            // the stall detector fires even when opposite normals cancel in `out`.
            let mut out = Vector2::ZERO;
            let mut pushed_any = false;
            for _ in 0..4 {
                let Some(start) = cdt.locate_face(unit.pos) else {
                    break;
                };
                s.faces.clear();
                s.visited.clear();
                s.faces.push(start);
                s.visited.push(start);
                let mut pushed = false;
                while let Some(face) = s.faces.pop() {
                    for he in face * 3..face * 3 + 3 {
                        let a = cdt.points()[cdt.he_origin(he) as usize];
                        let b = cdt.points()[cdt.he_dest(he) as usize];
                        let closest = closest_on_segment(unit.pos, a, b);
                        let delta = unit.pos - closest;
                        let d2 = delta.x * delta.x + delta.y * delta.y;
                        if d2 >= unit.radius * unit.radius {
                            continue;
                        }
                        if cdt.he_is_constrained(he) {
                            let d = d2.sqrt();
                            if d > 1e-6 {
                                unit.pos = closest + delta * (unit.radius / d);
                                pushed = true;
                                pushed_any = true;
                                out += delta * (1.0 / d);
                            }
                        } else if let Some(twin) = cdt.he_twin(he) {
                            // Circle reaches past a free edge: also check the
                            // neighbor face's walls.
                            let nb = cdt.face_of_he(twin);
                            if !s.visited.contains(&nb) {
                                s.visited.push(nb);
                                s.faces.push(nb);
                            }
                        }
                    }
                }
                if !pushed {
                    break;
                }
            }
            // Stuck-on-corner detection: a moving unit shoved off its cleared
            // route so its line to the next waypoint cuts a wall — i.e. it's
            // pressed against a wall (`pushed_any`), heading *into* that wall
            // (waypoint on its far side), and not shrinking its remaining path.
            // The wall-facing test is what separates this from a unit merely
            // jammed sideways against a wall by neighbours (where a repath
            // wouldn't help). Skipped in the arrival zone (crowding, not walls,
            // holds it). After a few such ticks, repath from where it now is.
            // Free ticks (no wall contact) reset the counter so only consecutive
            // clamped ticks accumulate toward the threshold.
            if pushed_any && unit.is_moving() {
                let goal = *unit.path.last().expect("moving ⇒ non-empty path");
                let (gx, gy) = (goal.x - unit.pos.x, goal.y - unit.pos.y);
                let in_arrival = gx * gx + gy * gy <= unit.arrival_r * unit.arrival_r;
                let rem = unit.remaining_len();
                let w = unit.waypoint();
                let into_wall = (w.x - unit.pos.x) * out.x + (w.y - unit.pos.y) * out.y < 0.0;
                if in_arrival {
                    unit.stall = 0;
                    unit.min_remaining = rem;
                } else if rem < unit.min_remaining - STALL_PROGRESS_EPS {
                    unit.min_remaining = rem;
                    unit.stall = 0;
                } else if into_wall {
                    unit.stall = unit.stall.saturating_add(1);
                    if unit.stall >= STALL_REPATH_TICKS {
                        s.repath.push(id);
                        unit.stall = 0;
                    }
                }
            } else {
                unit.stall = 0; // no wall contact this tick: don't carry stall forward
            }
        }
    }

    /// Repath units the wall clamp flagged as stuck against a corner, from
    /// their current (shoved) positions, so their next waypoint is reachable in
    /// a straight line again. Usually a no-op (empty list).
    fn repath_stalled(&mut self) {
        if self.step_scratch.repath.is_empty() {
            return;
        }
        let cdt = self.nav.navmesh();
        for i in 0..self.step_scratch.repath.len() {
            let id = self.step_scratch.repath[i];
            let Some(unit) = self.units.get_mut(id) else {
                continue;
            };
            let Some(&goal) = unit.path.last() else {
                continue;
            };
            let path = find_path_abstract(
                cdt,
                &self.abstraction,
                unit.pos,
                goal,
                &mut self.scratch,
                unit.radius,
            );
            set_path(unit, path);
        }
        self.step_scratch.repath.clear();
    }

    /// FNV-1a over tick, unit slots (slot order) and RNG — divergence detector
    /// for determinism tests and future lockstep.
    pub fn state_hash(&self) -> u64 {
        let mut h = Fnv::new();
        h.write_u64(self.tick);
        h.write_u64(self.group_seq as u64);
        h.write_u64(self.nav.num_obstacles() as u64);
        h.write_u64(self.units.slots.len() as u64);
        for (i, slot) in self.units.slots.iter().enumerate() {
            h.write_u64(self.units.generations[i] as u64);
            let Some(u) = slot else {
                h.write_u64(u64::MAX);
                continue;
            };
            h.write_v2(u.pos);
            h.write_v2(u.prev_pos);
            h.write_f32(u.radius);
            h.write_f32(u.max_speed);
            h.write_u64(u.group as u64);
            h.write_u64(u.parked as u64);
            h.write_f32(u.arrival_r);
            h.write_u64(u.stall as u64);
            h.write_f32(u.min_remaining);
            h.write_u64(u.path_i as u64);
            h.write_u64(u.path.len() as u64);
            for &p in &u.path {
                h.write_v2(p);
            }
        }
        self.rng.hash_into(&mut h);
        h.0
    }
}

/// Install a freshly computed path, dropping the leading start point (the
/// unit is already there; nothing reads it back); an empty/degenerate path
/// means idle.
fn set_path(unit: &mut Unit, mut path: Vec<Vector2>) {
    unit.path_i = 0;
    unit.parked = false; // re-tasked: no longer settled at a goal
    unit.stall = 0; // fresh path: clear any stuck-on-corner accrual
    unit.min_remaining = f32::MAX;
    if path.len() >= 2 {
        path.remove(0);
        unit.path = path;
    } else {
        unit.path.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapgen::{ROOM_SIZE, rooms_map};

    fn v(x: f32, y: f32) -> Vector2 {
        Vector2::new(x, y)
    }

    fn rooms_sim(cols: usize, rows: usize, seed: u64) -> Sim {
        let (points, constraints) = rooms_map(cols, rows);
        Sim::new(points, &constraints, seed)
    }

    /// Spawn now and return the id (commands carry no return channel).
    fn spawn(sim: &mut Sim, pos: Vector2, radius: f32, speed: f32) -> UnitId {
        sim.step(&[Command::Spawn { pos, radius, speed }]);
        sim.units().iter().last().unwrap().0
    }

    fn unit(sim: &Sim, id: UnitId) -> &Unit {
        sim.units().get(id).unwrap()
    }

    fn step_n(sim: &mut Sim, n: usize) {
        for _ in 0..n {
            sim.step(&[]);
        }
    }

    fn dist(a: Vector2, b: Vector2) -> f32 {
        ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
    }


    #[test]
    fn test_spawn_and_idle() {
        let mut sim = rooms_sim(2, 2, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 20.0);
        step_n(&mut sim, 5);
        let u = unit(&sim, id);
        assert_eq!(u.pos, v(50.0, 50.0));
        assert!(!u.is_moving());
    }

    #[test]
    fn test_reaches_goal_within_tick_bound() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
        let goal = v(250.0, 50.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal,
        }]);
        let len = unit(&sim, id).remaining_len();
        assert!(len > 0.0, "path must exist");
        let bound = (len / (30.0 * DT)).ceil() as usize + 2;
        step_n(&mut sim, bound);
        let u = unit(&sim, id);
        assert_eq!(u.pos, goal, "must snap exactly onto the goal");
        assert!(!u.is_moving());
    }

    #[test]
    fn test_waypoint_carry_covers_exact_distance() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 25.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(250.0, 60.0),
        }]);
        // While en route (not the arrival tick), each tick moves exactly
        // speed * DT of polyline distance, including across corners.
        let total = unit(&sim, id).remaining_len();
        let full_ticks = (total / (25.0 * DT)) as usize - 1;
        for _ in 0..full_ticks {
            let before = unit(&sim, id).pos;
            let i_before = unit(&sim, id).path_i;
            sim.step(&[]);
            let u = unit(&sim, id);
            let mut travelled = 0.0;
            let mut prev = before;
            for wp in i_before..u.path_i {
                travelled += dist(prev, u.path[wp as usize]);
                prev = u.path[wp as usize];
            }
            travelled += dist(prev, u.pos);
            assert!(
                (travelled - 25.0 * DT).abs() < 1e-3,
                "tick travelled {travelled}, want {}",
                25.0 * DT
            );
        }
    }

    /// Does segment a-b strictly cross segment c-d?
    fn segments_cross(a: Vector2, b: Vector2, c: Vector2, d: Vector2) -> bool {
        let orient = |p: Vector2, q: Vector2, r: Vector2| {
            ((q.x - p.x) as f64 * (r.y - p.y) as f64) - ((q.y - p.y) as f64 * (r.x - p.x) as f64)
        };
        let (o1, o2) = (orient(a, b, c), orient(a, b, d));
        let (o3, o4) = (orient(c, d, a), orient(c, d, b));
        o1 * o2 < 0.0 && o3 * o4 < 0.0
    }

    fn assert_no_wall_crossing(sim: &Sim, walls: &[(Vector2, Vector2)]) {
        for (_, u) in sim.units().iter() {
            for &(wa, wb) in walls {
                assert!(
                    !segments_cross(u.prev_pos, u.pos, wa, wb),
                    "unit segment {:?}->{:?} crosses wall {wa:?}-{wb:?}",
                    u.prev_pos,
                    u.pos
                );
            }
        }
    }

    fn wall_segments(points: &[Vector2], constraints: &[(u32, u32)]) -> Vec<(Vector2, Vector2)> {
        constraints
            .iter()
            .map(|&(a, b)| (points[a as usize], points[b as usize]))
            .collect()
    }

    #[test]
    fn test_movement_never_crosses_walls() {
        let (points, constraints) = rooms_map(3, 3);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 7);
        let mut ids = Vec::new();
        for i in 0..6 {
            let p = v(30.0 + 40.0 * (i % 3) as f32, 30.0 + 40.0 * (i / 3) as f32);
            ids.push(spawn(&mut sim, p, 6.0, 40.0));
        }
        sim.step(&[Command::Move {
            units: ids,
            goal: v(2.5 * ROOM_SIZE, 2.5 * ROOM_SIZE),
        }]);
        for _ in 0..600 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
    }

    #[test]
    fn test_mid_walk_move_replaces_path() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 20.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(250.0, 50.0),
        }]);
        step_n(&mut sim, 10);
        let new_goal = v(20.0, 20.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: new_goal,
        }]);
        assert_eq!(*unit(&sim, id).path.last().unwrap(), new_goal);
        step_n(&mut sim, 600);
        assert_eq!(unit(&sim, id).pos, new_goal);
    }

    #[test]
    fn test_unreachable_goal_idles() {
        let mut sim = rooms_sim(2, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 20.0);
        // Far outside the mesh.
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(-500.0, -500.0),
        }]);
        let u = unit(&sim, id);
        assert!(!u.is_moving());
        assert_eq!(u.pos, v(50.0, 50.0));
        step_n(&mut sim, 5);
    }

    #[test]
    fn test_stale_unit_id_ignored() {
        let mut sim = rooms_sim(2, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 20.0);
        let stale = UnitId {
            index: id.index,
            generation: id.generation + 1,
        };
        sim.step(&[Command::Move {
            units: vec![stale],
            goal: v(150.0, 50.0),
        }]);
        assert!(!unit(&sim, id).is_moving());
    }

    #[test]
    fn test_obstacle_insert_detours_and_remove_restores() {
        let mut sim = rooms_sim(2, 1, 1);
        // Radius 2: fits the ~5-wide gaps the obstacle leaves in the doorway.
        let id = spawn(&mut sim, v(50.0, 50.0), 2.0, 20.0);
        let goal = v(150.0, 50.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal,
        }]);
        let base_len = unit(&sim, id).remaining_len();

        // Narrow the doorway (gap x=100, y in [35, 65]).
        sim.step(&[Command::AddObstacle {
            points: vec![v(98.0, 40.0), v(102.0, 40.0), v(102.0, 60.0), v(98.0, 60.0)],
        }]);
        let u = unit(&sim, id);
        assert!(u.is_moving(), "detour must exist");
        let detour_len = u.remaining_len();
        assert!(
            detour_len > base_len,
            "detour {detour_len} not longer than base {base_len}"
        );

        sim.step(&[Command::RemoveObstacle {
            id: ObstacleId::from_raw(0),
        }]);
        let u = unit(&sim, id);
        assert!(u.is_moving());
        step_n(&mut sim, 600);
        assert_eq!(unit(&sim, id).pos, goal);
    }

    #[test]
    fn test_obstacle_making_goal_unreachable_idles_unit() {
        let mut sim = rooms_sim(2, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 8.0, 20.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(150.0, 50.0),
        }]);
        assert!(unit(&sim, id).is_moving());
        // Leaves ≤5-wide gaps beside the door — too tight for radius 8.
        sim.step(&[Command::AddObstacle {
            points: vec![v(98.0, 40.0), v(102.0, 40.0), v(102.0, 60.0), v(98.0, 60.0)],
        }]);
        assert!(!unit(&sim, id).is_moving(), "blocked unit must idle");
    }

    #[test]
    fn test_determinism_same_stream_same_hashes() {
        let script = |sim: &mut Sim| {
            let mut hashes = Vec::new();
            let mut ids = Vec::new();
            for t in 0..120u32 {
                let mut cmds = Vec::new();
                if t < 20 {
                    cmds.push(Command::Spawn {
                        pos: v(20.0 + 7.0 * t as f32, 30.0 + 5.0 * (t % 3) as f32),
                        radius: 5.0,
                        speed: 20.0 + t as f32,
                    });
                }
                if t == 25 {
                    ids = sim.units().iter().map(|(id, _)| id).collect();
                    cmds.push(Command::Move {
                        units: ids.clone(),
                        goal: v(250.0, 150.0),
                    });
                }
                if t == 40 {
                    cmds.push(Command::AddObstacle {
                        points: vec![
                            v(120.0, 40.0),
                            v(140.0, 40.0),
                            v(140.0, 60.0),
                            v(120.0, 60.0),
                        ],
                    });
                }
                if t == 80 {
                    cmds.push(Command::RemoveObstacle {
                        id: ObstacleId::from_raw(0),
                    });
                }
                if t == 90 {
                    cmds.push(Command::Move {
                        units: ids.clone(),
                        goal: v(50.0, 250.0),
                    });
                }
                sim.step(&cmds);
                hashes.push(sim.state_hash());
            }
            hashes
        };
        let a = script(&mut rooms_sim(3, 3, 42));
        let b = script(&mut rooms_sim(3, 3, 42));
        assert_eq!(a, b, "same command stream must reproduce every tick hash");

        let c = script(&mut rooms_sim(3, 3, 43));
        assert_eq!(a.len(), c.len());
    }

    #[test]
    fn test_determinism_bitwise_state() {
        let run = || {
            let mut sim = rooms_sim(2, 2, 9);
            let mut ids = Vec::new();
            for i in 0..8 {
                ids.push(spawn(&mut sim, v(25.0 + 6.0 * i as f32, 40.0), 5.0, 25.0));
            }
            sim.step(&[Command::Move {
                units: ids,
                goal: v(160.0, 160.0),
            }]);
            step_n(&mut sim, 200);
            sim
        };
        let (a, b) = (run(), run());
        assert_eq!(a.state_hash(), b.state_hash());
        for ((ia, ua), (ib, ub)) in a.units().iter().zip(b.units().iter()) {
            assert_eq!(ia, ib);
            assert_eq!(ua.pos.x.to_bits(), ub.pos.x.to_bits());
            assert_eq!(ua.pos.y.to_bits(), ub.pos.y.to_bits());
            assert_eq!(ua.path_i, ub.path_i);
            assert_eq!(ua.path.len(), ub.path.len());
        }
    }

    #[test]
    fn test_separation_disperses_clump_without_overlap() {
        let mut sim = rooms_sim(2, 2, 5);
        // 9 units stacked nearly on top of each other.
        for i in 0..9 {
            spawn(
                &mut sim,
                v(100.0 + 0.01 * i as f32, 100.0 + 0.013 * i as f32),
                4.0,
                30.0,
            );
        }
        step_n(&mut sim, 400);
        let units: Vec<&Unit> = sim.units().iter().map(|(_, u)| u).collect();
        for i in 0..units.len() {
            for j in (i + 1)..units.len() {
                let d = dist(units[i].pos, units[j].pos);
                let min = units[i].radius + units[j].radius;
                assert!(
                    d >= min - 0.1,
                    "units {i},{j} still overlap: dist {d} < {min}"
                );
            }
        }
    }

    #[test]
    fn test_separation_disperses_without_ping_pong() {
        // Soft separation: while a stacked clump eases apart, no unit may
        // reverse direction tick to tick (the visual jitter signature of
        // overshooting hard resolution).
        let mut sim = rooms_sim(2, 2, 5);
        for i in 0..9 {
            spawn(
                &mut sim,
                v(100.0 + 0.01 * i as f32, 100.0 + 0.013 * i as f32),
                4.0,
                30.0,
            );
        }
        let mut prev_disp: Vec<Vector2> = vec![Vector2::ZERO; 9];
        for tick in 0..100 {
            sim.step(&[]);
            for (i, (_, u)) in sim.units().iter().enumerate() {
                let d = u.pos - u.prev_pos;
                let moved = d.length_squared() > 1e-6;
                let was_moving = prev_disp[i].length_squared() > 1e-6;
                if moved && was_moving {
                    assert!(
                        d.dot(prev_disp[i]) >= 0.0,
                        "unit {i} reversed direction at tick {tick}: {:?} -> {d:?}",
                        prev_disp[i]
                    );
                }
                prev_disp[i] = d;
            }
        }
    }

    #[test]
    fn test_separation_never_pushes_through_walls() {
        let (points, constraints) = rooms_map(2, 2);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 5);
        // Clump right next to the interior wall junction at (100, 100).
        for i in 0..12 {
            spawn(
                &mut sim,
                v(93.0 + 0.01 * i as f32, 93.0 + 0.01 * i as f32),
                5.0,
                30.0,
            );
        }
        for _ in 0..300 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
        // And the clamp holds them clear of the walls (not just non-crossing).
        for (_, u) in sim.units().iter() {
            for &(wa, wb) in &walls {
                let c = closest_on_segment(u.pos, wa, wb);
                assert!(
                    dist(u.pos, c) >= u.radius - 1e-3,
                    "unit at {:?} overlaps wall {wa:?}-{wb:?}",
                    u.pos
                );
            }
        }
    }

    /// Mean distance of the alive units from their centroid (group spread).
    fn spread(sim: &Sim) -> f32 {
        let ps: Vec<Vector2> = sim.units().iter().map(|(_, u)| u.pos).collect();
        let n = ps.len() as f32;
        let mut c = Vector2::ZERO;
        for &p in &ps {
            c += p;
        }
        c *= 1.0 / n;
        ps.iter().map(|&p| dist(p, c)).sum::<f32>() / n
    }

    /// Ticks until `id` stops moving (arrives); panics if it never does.
    fn arrival_tick(sim: &mut Sim, id: UnitId, max: u64) -> u64 {
        for t in 1..=max {
            sim.step(&[]);
            if !unit(sim, id).is_moving() {
                return t;
            }
        }
        panic!("unit did not arrive within {max} ticks");
    }


    #[test]
    fn test_crowd_arrival_stops_short_of_goal() {
        // A reaches the goal and parks; B, behind it and same group, stops on
        // contact instead of driving onto the exact goal point.
        let mut sim = rooms_sim(2, 1, 8);
        let goal = v(150.0, 50.0);
        let a = spawn(&mut sim, v(135.0, 50.0), 5.0, 30.0);
        let b = spawn(&mut sim, v(110.0, 50.0), 5.0, 30.0);
        sim.step(&[Command::Move {
            units: vec![a, b],
            goal,
        }]);
        step_n(&mut sim, 200);
        let (ua, ub) = (unit(&sim, a), unit(&sim, b));
        assert!(ua.parked && ub.parked, "both must settle (parked)");
        assert!(!ua.is_moving() && !ub.is_moving());
        let (da, db) = (dist(ua.pos, goal), dist(ub.pos, goal));
        assert!(da.min(db) < 6.0, "one unit settles at the goal: {}", da.min(db));
        assert!(
            da.max(db) > 7.0,
            "the other stops short, not crammed onto the goal: {}",
            da.max(db)
        );
        assert!(dist(ua.pos, ub.pos) >= 9.5, "must not hard-overlap");
    }

    #[test]
    fn test_group_settles_as_blob_without_crush() {
        // A group arriving at one goal settles into a packed blob around it,
        // not a hard pile crammed onto the goal point.
        let mut sim = rooms_sim(1, 1, 13);
        let goal = v(50.0, 50.0);
        let mut ids = Vec::new();
        for i in 0..16 {
            ids.push(spawn(
                &mut sim,
                v(12.0 + 4.0 * (i % 4) as f32, 12.0 + 4.0 * (i / 4) as f32),
                5.0,
                30.0,
            ));
        }
        sim.step(&[Command::Move { units: ids, goal }]);
        step_n(&mut sim, 300);
        let us: Vec<&Unit> = sim.units().iter().map(|(_, u)| u).collect();
        assert!(
            us.iter().all(|u| u.parked && !u.is_moving()),
            "whole group must settle"
        );
        // No hard clump: nothing overlaps beyond tolerance.
        for i in 0..us.len() {
            for j in (i + 1)..us.len() {
                let d = dist(us[i].pos, us[j].pos);
                assert!(
                    d >= us[i].radius + us[j].radius - 0.5,
                    "units {i},{j} overlap: {d}"
                );
            }
        }
        // Only a couple reach the goal centre; the rest stop around it.
        let at_goal = us.iter().filter(|u| dist(u.pos, goal) < 5.0).count();
        assert!(at_goal <= 2, "units crammed onto the goal centre: {at_goal}");
        // The blob is *centred* on the goal — the radius-gated crowd-stop lets
        // units pack around it rather than tailing back along the approach, so
        // the centroid lands within ~one unit-diameter of the goal.
        let mut c = Vector2::ZERO;
        for u in &us {
            c += u.pos;
        }
        c *= 1.0 / us.len() as f32;
        assert!(
            dist(c, goal) < 12.0,
            "group centre not at the goal: {}",
            dist(c, goal)
        );
    }

    #[test]
    fn test_new_move_unparks_settled_group() {
        // A re-order clears `parked` and the group moves off again.
        let mut sim = rooms_sim(2, 1, 2);
        let mut ids = Vec::new();
        for i in 0..4 {
            ids.push(spawn(&mut sim, v(120.0 + 5.0 * i as f32, 50.0), 5.0, 30.0));
        }
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal: v(150.0, 50.0),
        }]);
        step_n(&mut sim, 150);
        assert!(sim.units().iter().all(|(_, u)| u.parked), "group settles");
        sim.step(&[Command::Move {
            units: ids,
            goal: v(50.0, 50.0),
        }]);
        assert!(
            sim.units().iter().all(|(_, u)| !u.parked),
            "re-move must clear parked"
        );
        step_n(&mut sim, 250);
        let (mut c, n) = (Vector2::ZERO, sim.units().len() as f32);
        for (_, u) in sim.units().iter() {
            c += u.pos;
        }
        c *= 1.0 / n;
        assert!(dist(c, v(50.0, 50.0)) < 20.0, "group reaches the new goal");
    }

    #[test]
    fn test_group_rounding_corner_none_stuck() {
        // A group rounding the end of a wall: the pack shoves some units onto
        // the wall's face, where their straight line to the next waypoint cuts
        // through it and the wall clamp pins them — stuck. The stall-repath
        // (heading-into-wall, no progress) must route them around and every
        // unit must reach the goal. Without it ~2 units stay pinned on the wall.
        let mut sim = rooms_sim(1, 1, 1);
        sim.step(&[Command::AddObstacle {
            points: vec![v(72.0, 10.0), v(78.0, 10.0), v(78.0, 75.0), v(72.0, 75.0)],
        }]);
        let goal = v(90.0, 40.0);
        let mut ids = Vec::new();
        for i in 0..12 {
            ids.push(spawn(
                &mut sim,
                v(20.0 + 5.0 * (i % 6) as f32, 35.0 + 5.0 * (i / 6) as f32),
                5.0,
                25.0,
            ));
        }
        sim.step(&[Command::Move { units: ids, goal }]);
        step_n(&mut sim, 900);
        for (_, u) in sim.units().iter() {
            assert!(
                dist(u.pos, goal) < 45.0,
                "unit stuck short of the goal at {:?} (dist {:.1})",
                u.pos,
                dist(u.pos, goal)
            );
        }
    }

    #[test]
    fn test_cohesion_tightens_group_spread() {
        // Same scenario twice: one Move groups all units (cohesion on); one
        // Move per unit gives each its own group (no shared group → cohesion
        // off). Paths and separation are identical, so the spread difference
        // is cohesion alone. Both variants are a single tick, so timing aligns.
        let run = |grouped: bool| -> f32 {
            let mut sim = rooms_sim(3, 1, 7);
            let mut ids = Vec::new();
            for i in 0..6 {
                ids.push(spawn(&mut sim, v(30.0, 20.0 + 12.0 * i as f32), 5.0, 40.0));
            }
            let goal = v(250.0, 50.0);
            let cmds: Vec<Command> = if grouped {
                vec![Command::Move { units: ids, goal }]
            } else {
                ids.into_iter()
                    .map(|id| Command::Move {
                        units: vec![id],
                        goal,
                    })
                    .collect()
            };
            sim.step(&cmds);
            step_n(&mut sim, 70);
            spread(&sim)
        };
        let tight = run(true);
        let loose = run(false);
        assert!(
            tight < loose,
            "cohesion must tighten the group: grouped {tight} vs ungrouped {loose}"
        );
    }

    #[test]
    fn test_cohesion_does_not_brake_leader() {
        // A fast leader grouped with slow units behind it must arrive about
        // when it would solo: the cohesion pull toward the lagging group is
        // backward, and the heading-opposing component is dropped.
        let goal = v(250.0, 50.0);
        let solo = {
            let mut sim = rooms_sim(3, 1, 3);
            let lead = spawn(&mut sim, v(30.0, 50.0), 5.0, 60.0);
            sim.step(&[Command::Move {
                units: vec![lead],
                goal,
            }]);
            arrival_tick(&mut sim, lead, 400)
        };
        let grouped = {
            let mut sim = rooms_sim(3, 1, 3);
            let lead = spawn(&mut sim, v(30.0, 50.0), 5.0, 60.0);
            let mut ids = vec![lead];
            for i in 0..5 {
                ids.push(spawn(&mut sim, v(20.0 - 3.0 * i as f32, 50.0), 5.0, 12.0));
            }
            sim.step(&[Command::Move { units: ids, goal }]);
            arrival_tick(&mut sim, lead, 400)
        };
        assert!(
            grouped.abs_diff(solo) <= 1,
            "cohesion braked the leader: solo {solo}, grouped {grouped}"
        );
    }

    #[test]
    fn test_stragglers_do_not_drag_group() {
        // The front of a moving pack must not wait for a slow straggler added
        // to its group: its arrival barely changes whether the straggler is
        // present (backward pull dropped, no group-speed coupling).
        let goal = v(250.0, 50.0);
        let front_arrival = |with_straggler: bool| -> u64 {
            let mut sim = rooms_sim(3, 1, 11);
            let mut ids = Vec::new();
            for i in 0..4 {
                ids.push(spawn(&mut sim, v(30.0 + 12.0 * i as f32, 50.0), 5.0, 50.0));
            }
            let front = ids[3];
            if with_straggler {
                ids.push(spawn(&mut sim, v(10.0, 50.0), 5.0, 8.0));
            }
            sim.step(&[Command::Move { units: ids, goal }]);
            arrival_tick(&mut sim, front, 600)
        };
        let alone = front_arrival(false);
        let dragged = front_arrival(true);
        assert!(
            dragged.abs_diff(alone) <= 2,
            "straggler dragged the group: without {alone}, with {dragged}"
        );
    }

    #[test]
    fn test_cross_group_independence() {
        // Two units within cohesion range marching to a shared goal. Same
        // group: cohesion pulls them together beyond what their converging
        // paths do. Different groups: no cross-group pull, so they stay as
        // far apart as the paths alone leave them.
        let gap_after = |same_group: bool| -> f32 {
            let mut sim = rooms_sim(3, 1, 4);
            let a = spawn(&mut sim, v(30.0, 40.0), 5.0, 30.0);
            let b = spawn(&mut sim, v(30.0, 60.0), 5.0, 30.0);
            let goal = v(250.0, 50.0);
            let cmds = if same_group {
                vec![Command::Move {
                    units: vec![a, b],
                    goal,
                }]
            } else {
                vec![
                    Command::Move {
                        units: vec![a],
                        goal,
                    },
                    Command::Move {
                        units: vec![b],
                        goal,
                    },
                ]
            };
            sim.step(&cmds);
            step_n(&mut sim, 20);
            dist(unit(&sim, a).pos, unit(&sim, b).pos)
        };
        let same = gap_after(true);
        let cross = gap_after(false);
        assert!(
            same < cross,
            "different groups must not cohere: same-group gap {same}, cross-group {cross}"
        );
    }

    #[test]
    fn test_idle_units_do_not_cohere() {
        // An idle unit (never commanded → group 0, no path) ignores cohesion:
        // a group marching within cohesion range must not drag it.
        let mut sim = rooms_sim(3, 1, 6);
        let idle = spawn(&mut sim, v(60.0, 82.0), 5.0, 30.0);
        let idle_pos = unit(&sim, idle).pos;
        let mut ids = Vec::new();
        for i in 0..5 {
            ids.push(spawn(&mut sim, v(30.0 + 8.0 * i as f32, 60.0), 5.0, 30.0));
        }
        sim.step(&[Command::Move {
            units: ids,
            goal: v(250.0, 60.0),
        }]);
        step_n(&mut sim, 20);
        assert_eq!(unit(&sim, idle).pos, idle_pos, "idle unit was dragged");
    }

    #[test]
    fn test_steady_state_step_is_allocation_free() {
        // Open single room (no interior corners → no stuck-on-corner repaths,
        // which legitimately allocate a fresh path and aren't steady state).
        let mut sim = rooms_sim(1, 1, 3);
        let mut ids = Vec::new();
        for i in 0..32 {
            ids.push(spawn(
                &mut sim,
                v(15.0 + 6.0 * (i % 8) as f32, 25.0 + 6.0 * (i / 8) as f32),
                5.0,
                20.0,
            ));
        }
        sim.step(&[Command::Move {
            units: ids,
            goal: v(50.0, 50.0),
        }]);
        // Warm scratch buffers (grid, dense arrays, locate paths) and let the
        // group settle into its blob, so the measured window is steady state.
        step_n(&mut sim, 40);
        let allocs = crate::alloc_counter::count_allocs(|| {
            for _ in 0..20 {
                sim.step(&[]);
            }
        });
        assert_eq!(allocs, 0, "steady-state step must not allocate");
    }

    #[test]
    fn test_pcg32_reference_stream() {
        // Two instances agree; stream is stable across runs.
        let mut a = Pcg32::new(0xCAFE);
        let mut b = Pcg32::new(0xCAFE);
        let xs: Vec<u32> = (0..8).map(|_| a.next_u32()).collect();
        let ys: Vec<u32> = (0..8).map(|_| b.next_u32()).collect();
        assert_eq!(xs, ys);
        let mut c = Pcg32::new(1);
        let f = c.next_f32();
        assert!((0.0..1.0).contains(&f));
    }

    #[test]
    fn test_unit_id_raw_roundtrip() {
        let id = UnitId {
            index: 1234,
            generation: 56,
        };
        assert_eq!(UnitId::from_raw(id.raw()), id);
    }

    #[test]
    fn test_slot_reuse_bumps_generation() {
        let mut units = Units::default();
        let a = units.spawn(Unit {
            pos: Vector2::ZERO,
            prev_pos: Vector2::ZERO,
            radius: 1.0,
            max_speed: 1.0,
            path: Vec::new(),
            path_i: 0,
            group: 0,
            parked: false,
            arrival_r: 0.0,
            stall: 0,
            min_remaining: f32::MAX,
        });
        assert!(units.despawn(a));
        assert!(!units.despawn(a), "double despawn must fail");
        let b = units.spawn(Unit {
            pos: Vector2::ONE,
            prev_pos: Vector2::ONE,
            radius: 1.0,
            max_speed: 1.0,
            path: Vec::new(),
            path_i: 0,
            group: 0,
            parked: false,
            arrival_r: 0.0,
            stall: 0,
            min_remaining: f32::MAX,
        });
        assert_eq!(a.index, b.index);
        assert_ne!(a.generation, b.generation);
        assert!(units.get(a).is_none());
        assert!(units.get(b).is_some());
    }
}

#[cfg(test)]
mod conflict_tests {
    use super::*;
    use crate::mapgen::rooms_map;

    /// A building crossing a map wall: the navmesh rejects the conflicting
    /// constraints via report_error! (collected, never engine-printed — the
    /// sim thread must not touch Godot FFI) and the sim stays functional.
    #[test]
    fn test_wall_crossing_obstacle_reports_error_and_sim_survives() {
        crate::report::install_collector();
        let (points, constraints) = rooms_map(2, 1);
        let mut sim = Sim::new(points, &constraints, 1);
        // Edges strictly cross the wall segment (100,0)-(100,35).
        sim.step(&[Command::AddObstacle {
            points: vec![
                Vector2::new(90.0, 10.0),
                Vector2::new(110.0, 10.0),
                Vector2::new(110.0, 30.0),
                Vector2::new(90.0, 30.0),
            ],
        }]);
        let errors = crate::report::drain();
        assert!(
            errors
                .iter()
                .any(|e| e.contains("intersects existing constraint")),
            "conflict must be reported: {errors:?}"
        );

        sim.step(&[Command::Spawn {
            pos: Vector2::new(50.0, 50.0),
            radius: 5.0,
            speed: 20.0,
        }]);
        let id = sim.units().iter().last().unwrap().0;
        sim.step(&[Command::Move {
            units: vec![id],
            goal: Vector2::new(150.0, 50.0),
        }]);
        assert!(sim.units().get(id).unwrap().is_moving());
        assert!(crate::report::drain().is_empty(), "no further errors");
    }
}
