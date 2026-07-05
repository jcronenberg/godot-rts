//! Deterministic fixed-tick unit simulation (see `simulation_plan.md`).
//!
//! `Sim::step` is a pure state transition: commands in, state out. Pacing,
//! threading and snapshots live in [`crate::sim_runner`]. Determinism rules:
//! f32 add/mul/div/sqrt only, slot-order iteration, no hash iteration over
//! state, pairwise forces buffered before application, one seeded RNG.

use std::collections::VecDeque;

use godot::prelude::Vector2;

use crate::abstraction::Abstraction;
use crate::astar::{
    AStarScratch, clear_los, clip_ray_to_walls, closest_on_segment, dist_segment_segment,
    find_path_abstract,
};
use crate::delaunay::{CDT, FNV_OFFSET, FNV_PRIME};
use crate::navmesh::{DynamicNavmesh, Obstacle, ObstacleId};

/// Simulation ticks per second.
pub const TICK_RATE: u32 = 30;
/// Fixed timestep in seconds.
pub const DT: f32 = 1.0 / TICK_RATE as f32;

// ── Tunables ─────────────────────────────────────────────────────────────────
// Runtime-settable (via `.get()`/`.set()`) so values can be swept without
// recompiling. If the atomic load ever shows up in profiles, these can drop
// back to plain `const`s (call sites would lose the `.get()`).

/// Fraction of pairwise overlap corrected per tick — the separation push force.
/// Soft enough that path pressure wins while units are *moving* (they tolerate a
/// little transient overlap squeezing through crowds/funnels), but with no path
/// pull at rest it still converges to a fully non-overlapping packing. Below the
/// point where summed pushes in a clump overshoot and ping-pong.
pub static SEPARATION_RELAX: TunableF32 = TunableF32::new(0.4);
/// Per-tick separation displacement cap, as a fraction of the unit's speed.
/// Above a full step, so separation can out-push a unit's own path/cohesion
/// convergence and resolve overlap rather than tolerate it.
pub static SEPARATION_MAX_FRAC: TunableF32 = TunableF32::new(1.5);
/// Cohesion radius as a multiple of the largest unit radius: same-group
/// neighbours within it pull toward their shared centroid.
pub static COHESION_RADIUS_FRAC: TunableF32 = TunableF32::new(5.0);
/// Per-tick fraction of the centroid offset applied as a cohesion pull.
pub static COHESION_GAIN: TunableF32 = TunableF32::new(0.05);
/// Per-tick cohesion displacement cap, as a fraction of the unit's speed. Kept
/// well below the path step (and the separation cap) so cohesion stays a gentle
/// bias and never pulls group-mates back into overlap.
pub static COHESION_MAX_FRAC: TunableF32 = TunableF32::new(0.15);
/// A moving unit joins a parked group-mate's cluster (and stops) when within
/// this multiple of touching distance of it — *and* within its arrival radius
/// of the goal (below). So a group settles into a blob around its goal instead
/// of every unit driving to the exact goal point and crushing inward.
pub static ARRIVAL_TOUCH_FRAC: TunableF32 = TunableF32::new(1.15);
/// A group's arrival radius is `r * max(ARRIVAL_MIN_RADII, FACTOR*sqrt(N))`.
/// A unit only crowd-stops once inside it. `sqrt(N)` tracks the packed-disk
/// radius of `N` circles; `FACTOR` > 1 pads it so units crowd-stop at the
/// blob's outer edge rather than shoving through to the core to reach the
/// goal; the `MIN` floor keeps small groups (whose followers sit ~2r out)
/// able to stop at all.
pub static ARRIVAL_RADIUS_FACTOR: TunableF32 = TunableF32::new(1.5);
pub static ARRIVAL_MIN_RADII: TunableF32 = TunableF32::new(3.0);
/// Fraction of a flock's lateral spread applied as corner-fan offset. Below 1.0
/// so outer units round a touch tighter and their separation doesn't shove them
/// onto the corridor edge.
pub static FAN_FRAC: TunableF32 = TunableF32::new(0.34);
/// Fraction of a flock's lateral spread held while marching a straight (corner-
/// free) leg. Unlike the bend fan (which spreads units *out* from the apex), here
/// units start at full spread, so this *relaxes* it: <1 lets them draw a bit
/// closer than their start formation while still marching side-by-side instead of
/// single-filing the line.
pub static STRAIGHT_FAN_FRAC: TunableF32 = TunableF32::new(0.6);
/// Wall-clamped ticks heading into a wall without progress before a moving unit
/// is treated as stuck (shoved off its path onto a corner) and repathed from its
/// current position. Eager (~0.1 s at 30 Hz) so displaced units recover a valid
/// route quickly; transient clamps self-resolve before they trip it.
pub static STALL_REPATH_TICKS: TunableU8 = TunableU8::new(3);
/// Remaining-path-length improvement that counts as real progress (and resets
/// the stall counter); below it the unit is treated as not advancing.
pub static STALL_PROGRESS_EPS: TunableF32 = TunableF32::new(0.1);

/// Max `wall_clamp` passes per unit per tick before giving up as unresolved.
/// Internal convergence detail, not a gameplay knob — plain const rather than
/// a `Tunable`.
const WALL_CLAMP_PASSES: u32 = 8;
/// Clearance slack (fraction of radius) below which a clamped position still
/// counts as overlapping a wall. Just enough for f32 noise in an exact-fit
/// (`gap == 2r`) corridor; more would let genuinely-too-tight gaps pass.
const WALL_CLAMP_REVERT_EPS_FRAC: f32 = 0.0005;

/// Runtime-settable f32, stored as bits in an atomic. Backs the tunables above.
pub struct TunableF32(std::sync::atomic::AtomicU32);
impl TunableF32 {
    const fn new(v: f32) -> Self {
        Self(std::sync::atomic::AtomicU32::new(v.to_bits()))
    }
    #[inline]
    pub fn get(&self) -> f32 {
        f32::from_bits(self.0.load(std::sync::atomic::Ordering::Relaxed))
    }
    pub fn set(&self, v: f32) {
        self.0
            .store(v.to_bits(), std::sync::atomic::Ordering::Relaxed)
    }
}

/// Runtime-settable u8, stored in an atomic. Backs [`STALL_REPATH_TICKS`].
pub struct TunableU8(std::sync::atomic::AtomicU8);
impl TunableU8 {
    const fn new(v: u8) -> Self {
        Self(std::sync::atomic::AtomicU8::new(v))
    }
    #[inline]
    pub fn get(&self) -> u8 {
        self.0.load(std::sync::atomic::Ordering::Relaxed)
    }
    pub fn set(&self, v: u8) {
        self.0.store(v, std::sync::atomic::Ordering::Relaxed)
    }
}

/// Sets a tunable above by its lowercased name (e.g. `"separation_relax"`);
/// returns `false` if `name` doesn't match one. Used by the sim_test debug UI
/// via `Simulation.set_tuning` — see `rust/rts/src/simulation.rs`.
pub fn set_tuning(name: &str, value: f32) -> bool {
    match name {
        "separation_relax" => SEPARATION_RELAX.set(value),
        "separation_max_frac" => SEPARATION_MAX_FRAC.set(value),
        "cohesion_radius_frac" => COHESION_RADIUS_FRAC.set(value),
        "cohesion_gain" => COHESION_GAIN.set(value),
        "cohesion_max_frac" => COHESION_MAX_FRAC.set(value),
        "arrival_touch_frac" => ARRIVAL_TOUCH_FRAC.set(value),
        "arrival_radius_factor" => ARRIVAL_RADIUS_FACTOR.set(value),
        "arrival_min_radii" => ARRIVAL_MIN_RADII.set(value),
        "fan_frac" => FAN_FRAC.set(value),
        "straight_fan_frac" => STRAIGHT_FAN_FRAC.set(value),
        "stall_repath_ticks" => STALL_REPATH_TICKS.set(value.round().clamp(0.0, 255.0) as u8),
        "stall_progress_eps" => STALL_PROGRESS_EPS.set(value),
        _ => return false,
    }
    true
}

/// Reads a tunable above by its lowercased name; `None` if `name` doesn't
/// match one. Lets UI (or anything else) start from the live value instead of
/// a second hardcoded copy of the default.
pub fn get_tuning(name: &str) -> Option<f32> {
    Some(match name {
        "separation_relax" => SEPARATION_RELAX.get(),
        "separation_max_frac" => SEPARATION_MAX_FRAC.get(),
        "cohesion_radius_frac" => COHESION_RADIUS_FRAC.get(),
        "cohesion_gain" => COHESION_GAIN.get(),
        "cohesion_max_frac" => COHESION_MAX_FRAC.get(),
        "arrival_touch_frac" => ARRIVAL_TOUCH_FRAC.get(),
        "arrival_radius_factor" => ARRIVAL_RADIUS_FACTOR.get(),
        "arrival_min_radii" => ARRIVAL_MIN_RADII.get(),
        "fan_frac" => FAN_FRAC.get(),
        "straight_fan_frac" => STRAIGHT_FAN_FRAC.get(),
        "stall_repath_ticks" => STALL_REPATH_TICKS.get() as f32,
        "stall_progress_eps" => STALL_PROGRESS_EPS.get(),
        _ => return None,
    })
}

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
    /// Pending orders, executed in turn once the current action finishes. The
    /// currently-active order isn't stored here — it's reflected in `path` etc.
    pub orders: VecDeque<Order>,
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

// ── Orders ────────────────────────────────────────────────────────────────────

/// A task a unit works through. Units hold a FIFO queue of these; the front one
/// drives behaviour until it completes, then the next begins (see
/// [`Sim::advance_orders`]). Extend with `Attack`, `HoldPosition`, … — each new
/// variant adds an arm to [`Sim::begin_order`] and to the order hash/snapshot.
#[derive(Clone, Debug, PartialEq)]
pub enum Order {
    Move { goal: Vector2 },
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
    /// Move now: clears each unit's order queue and paths immediately (a plain
    /// right-click that interrupts whatever the unit was doing).
    Move {
        units: Vec<UnitId>,
        goal: Vector2,
    },
    /// Append an order to each unit's queue (a shift-click). The general
    /// queue-append seam — every future order type queues through here.
    Queue {
        units: Vec<UnitId>,
        order: Order,
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
    /// Goal (last waypoint) per dense unit; junk for non-moving units, only read
    /// when both sides of a pair are moving (merge detection).
    goals: Vec<Vector2>,
    /// Group-id pairs to merge this tick, as `(min, max)`; usually empty.
    merge_pairs: Vec<(u32, u32)>,
    /// Wall-clamp face frontier and visited list (tiny per unit).
    faces: Vec<u32>,
    visited: Vec<u32>,
    /// Wall-clamp: constrained half-edges near the unit this tick (gate
    /// prefilter, then per-pass discoveries; tiny per unit).
    clamp_walls: Vec<u32>,
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
    /// integrate along paths → flock (separation + cohesion) → start the next
    /// queued order for any unit that just finished → wall clamp → repath units
    /// stuck against a corner → advance tick.
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
        self.advance_orders();
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
                    orders: VecDeque::new(),
                    group: 0,
                    parked: false,
                    arrival_r: 0.0,
                    stall: 0,
                    min_remaining: f32::MAX,
                });
            }
            Command::Move { units, goal } => {
                // Plain move interrupts: drop any queued orders, path now.
                for &id in units {
                    if let Some(u) = self.units.get_mut(id) {
                        u.orders.clear();
                    }
                }
                self.start_move(units, *goal);
            }
            Command::Queue { units, order } => {
                for &id in units {
                    if let Some(u) = self.units.get_mut(id) {
                        u.orders.push_back(order.clone());
                    }
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

    /// Execute a move now: partition the live selection into spatial flocks, give
    /// each its own group id and one shared channel, then string-pull that
    /// channel per unit so the flock spreads across corridor width instead of
    /// single-filing the inside corner. A unit whose first leg into the channel
    /// fails line-of-sight (straggler / no useful channel) paths individually.
    /// See `group_pathing_plan.md`.
    fn start_move(&mut self, units: &[UnitId], goal: Vector2) {
        // Live selected units, slot order (stable, deterministic).
        let mut sel: Vec<UnitId> = Vec::new();
        for &id in units {
            if self.units.get(id).is_some() {
                sel.push(id);
            }
        }
        let n = sel.len();
        if n == 0 {
            return;
        }
        let mut pos: Vec<Vector2> = Vec::with_capacity(n);
        let mut rad: Vec<f32> = Vec::with_capacity(n);
        for &id in &sel {
            let u = self.units.get(id).expect("filtered to live");
            pos.push(u.pos);
            rad.push(u.radius);
        }
        let max_radius = rad.iter().copied().fold(0.0f32, f32::max);
        let r_coh = max_radius * COHESION_RADIUS_FRAC.get();

        // Connected components under "same radius, within R_COH and clear
        // line-of-sight", via the spatial grid. Keying on radius lets each unit
        // size route by its own clearance, so a smaller unit can take a narrower,
        // shorter corridor the larger ones can't. Union-find over dense indices.
        let cdt = self.nav.navmesh();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        let cell = r_coh.max(max_radius * 2.0).max(1.0);
        self.grid.rebuild(&pos, cell);
        for i in 0..n {
            let (cx, cy) = self.grid.cell_coords(pos[i]);
            for ny in cy.saturating_sub(1)..=(cy + 1).min(self.grid.rows - 1) {
                for nx in cx.saturating_sub(1)..=(cx + 1).min(self.grid.cols - 1) {
                    for &j in self.grid.cell_entries(nx, ny) {
                        let j = j as usize;
                        if j <= i {
                            continue;
                        }
                        if rad[i] != rad[j] {
                            continue; // different sizes route as separate flocks
                        }
                        let d = pos[i] - pos[j];
                        if d.x * d.x + d.y * d.y > r_coh * r_coh {
                            continue;
                        }
                        if uf_find(&mut parent, i as u32) == uf_find(&mut parent, j as u32) {
                            continue;
                        }
                        if clear_los(cdt, pos[i], pos[j], rad[i]) {
                            uf_union(&mut parent, i as u32, j as u32);
                        }
                    }
                }
            }
        }

        // Assign a fresh group id per component, in first-appearance (slot)
        // order, and collect each component's member indices.
        let mut comps: Vec<(u32, Vec<usize>)> = Vec::new();
        let mut roots: Vec<(u32, usize)> = Vec::new(); // (root, comps index)
        for i in 0..n {
            let r = uf_find(&mut parent, i as u32);
            match roots.iter().find(|&&(rr, _)| rr == r) {
                Some(&(_, ci)) => comps[ci].1.push(i),
                None => {
                    self.group_seq = self.group_seq.wrapping_add(1).max(1);
                    roots.push((r, comps.len()));
                    comps.push((self.group_seq, vec![i]));
                }
            }
        }

        // Reusable per-corner geometry of the seed's funnel path.
        let mut corners: Vec<Vector2> = Vec::new();
        let mut outward: Vec<Vector2> = Vec::new();
        let mut lanes: Vec<f32> = Vec::new();
        for (group, members) in &comps {
            let size = members.len();
            // All members share one radius (clustering keys on it), so the flock's
            // channel fits them and smaller units cluster — and route — separately.
            let flock_r = rad[members[0]];
            let arrival_mult =
                (ARRIVAL_RADIUS_FACTOR.get() * (size as f32).sqrt()).max(ARRIVAL_MIN_RADII.get());
            // One shortest (funnel) path for the flock, seeded from the member
            // nearest the goal (a real unit position is on the navmesh). Its
            // interior waypoints are the corner apexes every unit would otherwise
            // single-file through.
            let seed = *members
                .iter()
                .min_by(|&&a, &&b| {
                    let da = pos[a] - goal;
                    let db = pos[b] - goal;
                    let (da, db) = (da.x * da.x + da.y * da.y, db.x * db.x + db.y * db.y);
                    da.partial_cmp(&db).unwrap()
                })
                .expect("non-empty component");
            let seed_path = find_path_abstract(
                cdt,
                &self.abstraction,
                pos[seed],
                goal,
                &mut self.scratch,
                flock_r,
            );

            // Empty seed path ⇒ the goal is unreachable for the seed (the member
            // nearest it). Cluster members are mutually clear-LoS, so they share
            // the seed's navmesh component and the goal is unreachable for all of
            // them: idle the whole cluster. Skipping this would fall through to
            // the straight-shot branch below, which can't tell "unreachable" from
            // "direct clear shot" — it would synthesise a converge waypoint on the
            // reachable side and route_onto_channel would append the unreachable
            // goal leg, handing non-seed units a path that pokes across the wall.
            if seed_path.is_empty() {
                for &i in members {
                    let unit = self.units.get_mut(sel[i]).expect("filtered to live");
                    unit.group = *group;
                    unit.arrival_r = rad[i] * arrival_mult;
                    set_path(unit, Vec::new());
                }
                continue;
            }

            // Corner apexes (drop start and goal) and the outward direction at
            // each — the external bisector, pointing into the bend's free side
            // (away from the wall vertex the apex hugs). Units fan along it.
            corners.clear();
            outward.clear();
            if seed_path.len() > 2 {
                for w in seed_path.windows(3) {
                    let (prev, cur, next) = (w[0], w[1], w[2]);
                    corners.push(cur);
                    outward.push(external_bisector(prev, cur, next));
                }
            }

            // No corners ⇒ a straight shot to the goal, where the flock would
            // otherwise single-file the line and crush together. Synthesise one
            // waypoint near the goal with a perpendicular axis so each unit holds
            // its lateral lane until the final approach: it marches parallel and
            // only converges over the last leg (the blob zone). The waypoint sits
            // a converge distance (≈ the arrival radius, capped to half the path)
            // back from the goal along the line. Fanning is symmetric about the
            // line (both sides), unlike a bend's one-sided fan into its free side.
            let straight = corners.is_empty() && {
                let dir = goal - pos[seed];
                let len2 = dir.x * dir.x + dir.y * dir.y;
                if len2 > 1e-6 {
                    let len = len2.sqrt();
                    let converge = (flock_r * arrival_mult).min(len * 0.5);
                    corners.push(goal - dir * (converge / len));
                    outward.push(Vector2::new(-dir.y, dir.x) * (1.0 / len));
                    true
                } else {
                    false
                }
            };

            // Each unit's lane = its lateral offset from the flock along the fan
            // axis. At a bend, shift so the most wall-ward unit sits at the apex
            // (offset 0) and the rest fan into the free side; on a straight leg,
            // keep the sign so they fan symmetrically and hold their spread.
            let has_corners = !corners.is_empty();
            let axis = if has_corners {
                outward[0]
            } else {
                Vector2::ZERO
            };
            lanes.clear();
            let mut lane_min = f32::INFINITY;
            for &i in members {
                let d = pos[i] - pos[seed];
                let l = d.x * axis.x + d.y * axis.y;
                lanes.push(l);
                lane_min = lane_min.min(l);
            }

            for (k, &i) in members.iter().enumerate() {
                let radius = rad[i];
                // The seed already has its shortest path (offset 0, same start);
                // reuse it (this also covers singleton clusters with no fan).
                let path = if i == seed {
                    seed_path.clone()
                } else {
                    let offset = if straight {
                        // Hold most of the marching spread, relaxed a touch.
                        lanes[k] * STRAIGHT_FAN_FRAC.get()
                    } else if has_corners {
                        (lanes[k] - lane_min) * FAN_FRAC.get()
                    } else {
                        0.0
                    };
                    build_offset_path(cdt, pos[i], &corners, &outward, goal, offset, radius)
                        .unwrap_or_else(|| {
                            route_onto_channel(
                                cdt,
                                &self.abstraction,
                                &mut self.scratch,
                                pos[i],
                                &corners,
                                goal,
                                radius,
                            )
                        })
                };
                let unit = self.units.get_mut(sel[i]).expect("filtered to live");
                unit.group = *group;
                unit.arrival_r = radius * arrival_mult;
                set_path(unit, path);
            }
        }
    }

    /// Begin one order for a batch of units, dispatching on its type. The
    /// execution seam for queued orders — new `Order` variants add an arm here.
    fn begin_order(&mut self, units: &[UnitId], order: &Order) {
        match order {
            Order::Move { goal } => self.start_move(units, *goal),
        }
    }

    /// Start the next queued order for every unit that just finished its current
    /// action (now idle with a non-empty queue). Units that finished on the same
    /// tick with an *identical* front order are begun as one batch, so a group
    /// re-clusters and re-channels around the next waypoint instead of single-
    /// filing it; stragglers that finish a tick later re-merge via the flock
    /// merge pass. Deterministic: slot-order scan, exact-order batching, no map
    /// iteration.
    fn advance_orders(&mut self) {
        let mut batches: Vec<(Order, Vec<UnitId>)> = Vec::new();
        for (id, u) in self.units.iter() {
            if u.is_moving() {
                continue;
            }
            let Some(order) = u.orders.front() else {
                continue;
            };
            match batches.iter_mut().find(|(o, _)| o == order) {
                Some((_, members)) => members.push(id),
                None => batches.push((order.clone(), vec![id])),
            }
        }
        for (order, members) in &batches {
            for &id in members {
                self.units
                    .get_mut(id)
                    .expect("batched id alive")
                    .orders
                    .pop_front();
            }
            self.begin_order(members, order);
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
    ///
    /// Each unit first **re-anchors**: it advances past any intermediate waypoint
    /// it has already rounded, so crowd pressure that shoves it *through* a
    /// waypoint doesn't make it double back to the one it overshot. A waypoint is
    /// rounded once the unit is past its gate (on the next leg's side) *and* has
    /// clear line-of-sight to the following waypoint — the LoS test is what stops
    /// it cutting an unrounded corner.
    fn integrate(&mut self) {
        let cdt = self.nav.navmesh();
        for (_, unit) in self.units.iter_mut() {
            if unit.path.is_empty() {
                continue;
            }
            while (unit.path_i as usize) + 1 < unit.path.len() {
                let i = unit.path_i as usize;
                let (w, nxt) = (unit.path[i], unit.path[i + 1]);
                let (d, seg) = (unit.pos - w, nxt - w);
                if d.x * seg.x + d.y * seg.y >= 0.0 && clear_los(cdt, unit.pos, nxt, unit.radius) {
                    unit.path_i += 1;
                } else {
                    break;
                }
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
        s.goals.clear();
        s.merge_pairs.clear();
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
            s.goals
                .push(u.path.last().copied().unwrap_or(Vector2::ZERO));
            max_radius = max_radius.max(u.radius);
        }
        if s.ids.len() < 2 || max_radius <= 0.0 {
            return;
        }
        let max_diameter = max_radius * 2.0;
        let r_coh = max_radius * COHESION_RADIUS_FRAC.get();
        let r_coh2 = r_coh * r_coh;
        // Cell covers the larger radius so the 3×3 scan still finds every pair.
        self.grid.rebuild(&s.positions, max_diameter.max(r_coh));
        let cdt = self.nav.navmesh();
        let separation_relax = SEPARATION_RELAX.get();
        let arrival_touch_frac = ARRIVAL_TOUCH_FRAC.get();

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
                            let push = dir * ((min_dist - d) * 0.5 * separation_relax);
                            s.disp[i] += push;
                            s.disp[j] -= push;
                        }
                        // Merge: two adjacent, both-moving, same-goal, same-size
                        // units from *different* flocks continue as one. R_COH +
                        // clear-LoS adjacency (the wall gate stops merging flocks
                        // that are close but wall-separated); exact-goal match is
                        // the deterministic "commanded together" test; same radius
                        // keeps differently-sized flocks (which route apart) from
                        // fusing. Recorded as (min, max), unioned after the pass (v1).
                        let g_j = s.groups[j];
                        if g_i != 0
                            && g_j != 0
                            && g_i != g_j
                            && mv_i
                            && s.moving[j]
                            && r_i == s.radii[j]
                            && d2 < r_coh2
                            && s.goals[i] == s.goals[j]
                            && clear_los(cdt, p, s.positions[j], r_i)
                        {
                            let pair = if g_i < g_j { (g_i, g_j) } else { (g_j, g_i) };
                            s.merge_pairs.push(pair);
                        }
                        if g_i == 0 || g_i != g_j {
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
                        let touch = min_dist * arrival_touch_frac;
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
                let mut pull = s.coh_sum[i] * (COHESION_GAIN.get() / s.coh_n[i] as f32);
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
                let coh_cap = s.speeds[i] * DT * COHESION_MAX_FRAC.get();
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
            let max_step = s.speeds[i] * DT * SEPARATION_MAX_FRAC.get();
            let len = len2.sqrt();
            if len > max_step {
                d *= max_step / len;
            }
            // Anti-tunnel: a flock push is a raw position add with no path/LoS
            // guarantee, so it can shove a unit clean across a constrained edge —
            // wall_clamp only inspects the final position and would then amplify
            // the tunnel (projecting the centre out the *wrong* side). It bites
            // when the goal sits near a wall, so integrate has already driven the
            // centre to within a radius of it. Clip the move to stop at the wall.
            unit.pos = clip_ray_to_walls(cdt, unit.pos, unit.pos + d);
        }

        if !self.step_scratch.merge_pairs.is_empty() {
            self.apply_merges();
        }
    }

    /// Relabel converging flocks recorded this tick (`flock`'s merge pass) onto
    /// one group id: union the `(min, max)` pairs and rewrite each member to its
    /// canonical (min) id. Each merged unit keeps its own already-valid path —
    /// only its group changes, so it coheres / crowd-arrives with the joined
    /// flock from next tick (v1; no re-channel). Deterministic: sorted-pair
    /// order, min canonicalisation, slot-order relabel, no map iteration.
    fn apply_merges(&mut self) {
        use std::collections::HashMap;
        let pairs = &mut self.step_scratch.merge_pairs;
        pairs.sort_unstable();
        pairs.dedup();

        // Union-find over the (few) involved group ids. The map is only ever
        // point-queried, never iterated, so its order can't affect results.
        let mut parent: HashMap<u32, u32> = HashMap::new();
        fn find(parent: &mut HashMap<u32, u32>, x: u32) -> u32 {
            let mut r = x;
            while let Some(&p) = parent.get(&r) {
                if p == r {
                    break;
                }
                r = p;
            }
            let mut c = x;
            while c != r {
                let next = *parent.get(&c).unwrap_or(&c);
                parent.insert(c, r);
                c = next;
            }
            r
        }
        for &(a, b) in pairs.iter() {
            let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
            if ra != rb {
                let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                parent.insert(hi, lo);
            }
        }
        for (_, u) in self.units.iter_mut() {
            if u.group != 0 {
                let g = find(&mut parent, u.group);
                if g != u.group {
                    u.group = g;
                }
            }
        }
        self.step_scratch.merge_pairs.clear();
    }

    /// Collect (dedup) every constrained half-edge within `radius` of `p` into
    /// `walls`, without moving anything — a read-only version of the radius
    /// BFS `wall_clamp`'s pass loop uses while pushing. Reuses `faces`/
    /// `visited` as scratch (cleared internally).
    fn gather_nearby_walls(
        cdt: &CDT,
        p: Vector2,
        radius: f32,
        faces: &mut Vec<u32>,
        visited: &mut Vec<u32>,
        walls: &mut Vec<u32>,
    ) {
        let Some(start) = cdt.locate_face(p) else {
            return;
        };
        faces.clear();
        visited.clear();
        faces.push(start);
        visited.push(start);
        while let Some(face) = faces.pop() {
            for he in face * 3..face * 3 + 3 {
                let a = cdt.points()[cdt.he_origin(he) as usize];
                let b = cdt.points()[cdt.he_dest(he) as usize];
                let closest = closest_on_segment(p, a, b);
                let delta = p - closest;
                if delta.x * delta.x + delta.y * delta.y >= radius * radius {
                    continue;
                }
                if cdt.he_is_constrained(he) {
                    if !walls.contains(&he) {
                        walls.push(he);
                    }
                } else if let Some(twin) = cdt.he_twin(he) {
                    let nb = cdt.face_of_he(twin);
                    if !visited.contains(&nb) {
                        visited.push(nb);
                        faces.push(nb);
                    }
                }
            }
        }
    }

    /// Whether the tick's end-to-end motion (chord `prev → pos`) passed
    /// within `radius` of two walls that don't share an endpoint — squeezed
    /// through a gap the body doesn't fit. Adjacent walls are exempt: they
    /// form one obstacle corner, which paths may legitimately cut close (see
    /// `astar.rs`'s clearance contract) trusting the clamp to push back out.
    fn swept_through_pinch(
        cdt: &CDT,
        s: &mut StepScratch,
        prev: Vector2,
        pos: Vector2,
        radius: f32,
    ) -> bool {
        // Midpoint gather with radius expanded by half the chord length covers
        // every wall within `radius` of any point of the chord in one BFS.
        let half = (pos - prev) * 0.5;
        Self::gather_nearby_walls(
            cdt,
            prev + half,
            radius + half.length(),
            &mut s.faces,
            &mut s.visited,
            &mut s.clamp_walls,
        );
        let min_clear = radius - WALL_CLAMP_REVERT_EPS_FRAC * radius;
        s.clamp_walls.retain(|&he| {
            let a = cdt.points()[cdt.he_origin(he) as usize];
            let b = cdt.points()[cdt.he_dest(he) as usize];
            dist_segment_segment(prev, pos, a, b) < min_clear
        });
        let shares_vertex = |he1: u32, he2: u32| {
            let (a1, b1) = (cdt.he_origin(he1), cdt.he_dest(he1));
            let (a2, b2) = (cdt.he_origin(he2), cdt.he_dest(he2));
            a1 == a2 || a1 == b2 || b1 == a2 || b1 == b2
        };
        s.clamp_walls.iter().enumerate().any(|(i, &he1)| {
            s.clamp_walls[i + 1..]
                .iter()
                .any(|&he2| !shares_vertex(he1, he2))
        })
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
            // Fast path: distance to a wall changes at most 1:1 with distance
            // moved, so with no wall within `radius` + chord length of the
            // endpoint, the pass loop below finds nothing to push and no wall
            // can lie within `radius` of any point of the tick's chord — no
            // possible pinch either. Skip the unit.
            let reach = unit.radius + (unit.pos - unit.prev_pos).length();
            s.clamp_walls.clear();
            Self::gather_nearby_walls(
                cdt,
                unit.pos,
                reach,
                &mut s.faces,
                &mut s.visited,
                &mut s.clamp_walls,
            );
            if s.clamp_walls.is_empty() {
                unit.stall = 0;
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
            // Check every push against every wall any pass touched so far: a
            // push snaps the circle to exactly `radius` from one wall — at a
            // segment endpoint, radially from that corner — and near a
            // sub-diameter gap that snap can leap past the *other* wall's
            // exclusion disk in one discrete step ("corner-teleport").
            s.clamp_walls.clear();
            let mut violated = false;
            let min_clear = unit.radius - WALL_CLAMP_REVERT_EPS_FRAC * unit.radius;
            'passes: for _ in 0..WALL_CLAMP_PASSES {
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
                            if !s.clamp_walls.contains(&he) {
                                s.clamp_walls.push(he);
                            }
                            let d = d2.sqrt();
                            if d > 1e-6 {
                                unit.pos = closest + delta * (unit.radius / d);
                                pushed = true;
                                pushed_any = true;
                                out += delta * (1.0 / d);
                                if s.clamp_walls.iter().any(|&wh| {
                                    if wh == he {
                                        return false;
                                    }
                                    let wa = cdt.points()[cdt.he_origin(wh) as usize];
                                    let wb = cdt.points()[cdt.he_dest(wh) as usize];
                                    let c = closest_on_segment(unit.pos, wa, wb);
                                    let dd = unit.pos - c;
                                    dd.x * dd.x + dd.y * dd.y < min_clear * min_clear
                                }) {
                                    violated = true;
                                    break 'passes;
                                }
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
            // Revert when a push re-violated a touched wall or the chord swept
            // a gap the body doesn't fit: `prev_pos` is clear by induction, so
            // crowd pressure can never ratchet a unit into or through a pinch.
            // Skipped on `mesh_changed` — a new obstacle can invalidate
            // `prev_pos` itself, so best-effort projection is correct there.
            if !mesh_changed
                && (violated
                    || Self::swept_through_pinch(cdt, s, unit.prev_pos, unit.pos, unit.radius))
            {
                unit.pos = unit.prev_pos;
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
                } else if rem < unit.min_remaining - STALL_PROGRESS_EPS.get() {
                    unit.min_remaining = rem;
                    unit.stall = 0;
                } else if into_wall {
                    unit.stall = unit.stall.saturating_add(1);
                    if unit.stall >= STALL_REPATH_TICKS.get() {
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
            h.write_u64(u.orders.len() as u64);
            for order in &u.orders {
                match order {
                    Order::Move { goal } => {
                        h.write_u64(0);
                        h.write_v2(*goal);
                    }
                }
            }
        }
        self.rng.hash_into(&mut h);
        h.0
    }
}

fn norm(v: Vector2) -> Vector2 {
    let l2 = v.x * v.x + v.y * v.y;
    if l2 > 1e-12 {
        v * (1.0 / l2.sqrt())
    } else {
        Vector2::ZERO
    }
}

/// Outward direction at a path corner `cur` between `prev` and `next`: the unit
/// vector pointing away from the wall vertex the shortest-path apex wraps, into
/// the bend's free side. `norm(prev−cur)+norm(next−cur)` bisects toward the
/// inside (the vertex), so negate it. `ZERO` for a (near-)straight corner.
fn external_bisector(prev: Vector2, cur: Vector2, next: Vector2) -> Vector2 {
    -norm(norm(prev - cur) + norm(next - cur))
}

/// A unit's path through the flock's shared corner apexes, each displaced
/// `outward[j] * offset` into the bend's free side so the flock fans out and
/// rounds bends side-by-side instead of single-filing the apex. Tries the full
/// offset, then halves toward the apex; returns the first polyline whose every
/// leg clears walls, or `None` so the caller paths the unit alone.
///
/// Fanned legs (offset > 0) are validated at an inflated radius so a unit only
/// takes a lane where the corridor has room for parallel lanes — in a tight
/// squeeze every offset fails the inflated check and it falls back to the apex
/// (single-file, the only thing that fits). The apex itself (scale 0) is
/// validated at the true radius, so the real shortest route is always allowed.
fn build_offset_path(
    cdt: &CDT,
    start: Vector2,
    corners: &[Vector2],
    outward: &[Vector2],
    goal: Vector2,
    offset: f32,
    radius: f32,
) -> Option<Vec<Vector2>> {
    // One extra radius of clearance ⇒ a fanned lane has ≈ a full diameter of
    // room beside the apex before it's accepted.
    let fan_radius = radius * 2.0;
    for &scale in &[1.0f32, 0.5, 0.25, 0.0] {
        let o = offset * scale;
        // Offset can be signed (straight-leg fanning spreads to both sides of the
        // line); the inflated lane check keys on magnitude, not direction.
        let check_r = if o.abs() > 1e-6 { fan_radius } else { radius };
        let mut path = Vec::with_capacity(corners.len() + 2);
        path.push(start);
        for (c, n) in corners.iter().zip(outward) {
            path.push(*c + *n * o);
        }
        path.push(goal);
        if path.windows(2).all(|w| clear_los(cdt, w[0], w[1], check_r)) {
            return Some(path);
        }
    }
    None
}

/// Fallback when a unit can't follow the flock's shared corners (its leg to the
/// first one is blocked): route it onto the flock's route *via* that first
/// corner, rather than down a private shortest path — otherwise an outer unit
/// whose own shortest rounds an obstacle the *other* way splits off from the
/// group. With no corners (straight shot), or if even the leg to the first
/// corner is blocked, fall back to a plain shortest path to the goal.
fn route_onto_channel(
    cdt: &CDT,
    abstraction: &Abstraction,
    scratch: &mut AStarScratch,
    start: Vector2,
    corners: &[Vector2],
    goal: Vector2,
    radius: f32,
) -> Vec<Vector2> {
    let mut p = if corners.is_empty() {
        Vec::new()
    } else {
        find_path_abstract(cdt, abstraction, start, corners[0], scratch, radius)
    };
    if p.len() >= 2 {
        p.extend_from_slice(&corners[1..]);
        p.push(goal);
        p
    } else {
        find_path_abstract(cdt, abstraction, start, goal, scratch, radius)
    }
}

/// Union-find root with path halving (over dense indices, for `start_move`'s
/// spatial clustering).
fn uf_find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        parent[x as usize] = parent[parent[x as usize] as usize];
        x = parent[x as usize];
    }
    x
}

/// Union two sets, keeping the smaller root as canonical (deterministic).
fn uf_union(parent: &mut [u32], a: u32, b: u32) {
    let (ra, rb) = (uf_find(parent, a), uf_find(parent, b));
    if ra != rb {
        let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
        parent[hi as usize] = lo;
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
    fn test_queued_moves_run_consecutively() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
        let (g1, g2) = (v(250.0, 50.0), v(150.0, 50.0));
        sim.step(&[Command::Move {
            units: vec![id],
            goal: g1,
        }]);
        sim.step(&[Command::Queue {
            units: vec![id],
            order: Order::Move { goal: g2 },
        }]);
        assert_eq!(
            unit(&sim, id).orders.len(),
            1,
            "second move is queued, not run"
        );

        // The queue drains the tick g1 is reached; g2 must not begin before then.
        let mut reached_g1 = false;
        for _ in 0..1000 {
            sim.step(&[]);
            let u = unit(&sim, id);
            if u.orders.is_empty() && !reached_g1 {
                reached_g1 = true;
                assert_eq!(
                    u.pos, g1,
                    "first goal must be reached before the second begins"
                );
            }
            if reached_g1 && !u.is_moving() {
                break;
            }
        }
        assert!(reached_g1, "first queued goal was never reached");
        assert_eq!(unit(&sim, id).pos, g2, "must end at the second queued goal");
    }

    #[test]
    fn test_queue_on_idle_unit_starts_at_once() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
        let goal = v(150.0, 50.0);
        // advance_orders runs in the same step the command lands, so an idle
        // unit begins a queued order immediately (no idle tick).
        sim.step(&[Command::Queue {
            units: vec![id],
            order: Order::Move { goal },
        }]);
        let u = unit(&sim, id);
        assert!(u.is_moving());
        assert!(u.orders.is_empty());
        assert_eq!(*u.path.last().unwrap(), goal);
    }

    #[test]
    fn test_plain_move_clears_queue() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(250.0, 50.0),
        }]);
        sim.step(&[Command::Queue {
            units: vec![id],
            order: Order::Move {
                goal: v(150.0, 50.0),
            },
        }]);
        assert_eq!(unit(&sim, id).orders.len(), 1);
        let new_goal = v(200.0, 50.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: new_goal,
        }]);
        let u = unit(&sim, id);
        assert!(
            u.orders.is_empty(),
            "a plain move must cancel queued orders"
        );
        assert_eq!(*u.path.last().unwrap(), new_goal);
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
                    // Queue a follow-on order so the order queue is hashed too.
                    cmds.push(Command::Queue {
                        units: ids.clone(),
                        order: Order::Move {
                            goal: v(60.0, 60.0),
                        },
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
                // At rest (no path pull), separation resolves overlap fully.
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
    fn test_group_packed_against_edge_stays_inside() {
        // A large, fast group ordered to a goal hard against the map's right
        // edge (x=200 wall), with the same move re-issued repeatedly (the
        // player spam-clicking the same spot). The crowd packs against the wall
        // and the per-tick separation push — large for fast units — must never
        // shove a unit clean across the boundary to the outside of the map.
        let (points, constraints) = rooms_map(2, 2); // 200×200 open-ish map
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 5);
        let goal = v(197.0, 100.0); // ~3 px from the x=200 boundary
        let mut ids = Vec::new();
        for i in 0..50 {
            ids.push(spawn(
                &mut sim,
                v(110.0 + 6.0 * (i % 10) as f32, 70.0 + 6.0 * (i / 10) as f32),
                5.0,
                200.0, // fast: large per-tick separation push
            ));
        }
        for _ in 0..400 {
            // Re-issue the same move every few ticks (spam-click).
            let cmd = if sim.tick().is_multiple_of(5) {
                vec![Command::Move {
                    units: ids.clone(),
                    goal,
                }]
            } else {
                vec![]
            };
            sim.step(&cmd);
            assert_no_wall_crossing(&sim, &walls);
            // Every unit centre must stay inside the map (no unit past x=200).
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 200.0 + 1e-3,
                    "unit pushed across the edge wall to {:?}",
                    u.pos
                );
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

    /// 200x200 map with a 3px-thick wall from (150,`gap`) up to the top edge,
    /// leaving a `gap`-tall slot to the bottom boundary.
    fn thin_wall_map(gap: f32) -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let pts = vec![
            v(0.0, 0.0),
            v(200.0, 0.0),
            v(200.0, 200.0),
            v(0.0, 200.0),
            v(150.0, gap),
            v(153.0, gap),
            v(153.0, 200.0),
            v(150.0, 200.0),
        ];
        let cons = vec![
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
        ];
        (pts, cons)
    }

    #[test]
    fn test_crowd_never_squeezes_through_subdiameter_gap() {
        // narrow_gap_clearance_findings.md repro: a 6px slot, too tight for
        // radius-5 (diameter-10) units. A single unit stalls at the entrance;
        // sustained crowd pressure must not squeeze the front units through
        // it either.
        let (points, constraints) = thin_wall_map(6.0);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 7);
        let goal = v(175.0, 100.0);
        // Spawn all 40 units in one step (spawning one-at-a-time via the
        // `spawn()` helper each runs its own `sim.step`, which would let
        // early-spawned units collide/settle over dozens of unchecked ticks
        // before the crowd-pressure loop below even starts).
        let spawn_cmds: Vec<Command> = (0..40)
            .map(|i| Command::Spawn {
                // 8 columns * 3px stay well clear of the wall at x=150.
                pos: v(115.0 + 3.0 * (i % 8) as f32, 20.0 + 4.0 * (i / 8) as f32),
                radius: 5.0,
                speed: 60.0,
            })
            .collect();
        sim.step(&spawn_cmds);
        let ids: Vec<UnitId> = sim.units().iter().map(|(id, _)| id).collect();
        assert_no_wall_crossing(&sim, &walls);
        for _ in 0..1000 {
            let cmd = if sim.tick().is_multiple_of(5) {
                vec![Command::Move {
                    units: ids.clone(),
                    goal,
                }]
            } else {
                vec![]
            };
            sim.step(&cmd);
            assert_no_wall_crossing(&sim, &walls);
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 150.0 + 1e-3,
                    "unit squeezed through the sub-diameter gap to {:?}",
                    u.pos
                );
            }
        }
    }

    #[test]
    fn test_crowd_never_squeezes_through_near_exact_gap() {
        // Harsher than the 6px-vs-10px repro above: the gap is only *just*
        // too small (deficit well under a diameter), and the crowd spam-clicks
        // right at the gap entrance every tick instead of every 5th — maximum
        // sustained pressure against the weakest case (near-exact-fit, where
        // the clamp's float-noise slack is largest relative to the deficit).
        let (points, constraints) = thin_wall_map(9.9); // diameter 10: 0.1px too tight.
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 11);
        let goal = v(151.5, 3.0); // dead centre of the gap: max pressure at the pinch.
        let spawn_cmds: Vec<Command> = (0..60)
            .map(|i| Command::Spawn {
                // Columns stay well clear (max x=133.5) of the wall at x=150.
                pos: v(120.0 + 1.5 * (i % 10) as f32, 20.0 + 3.0 * (i / 10) as f32),
                radius: 5.0,
                speed: 80.0,
            })
            .collect();
        sim.step(&spawn_cmds);
        let ids: Vec<UnitId> = sim.units().iter().map(|(id, _)| id).collect();
        assert_no_wall_crossing(&sim, &walls);
        for _ in 0..3000 {
            sim.step(&[Command::Move {
                units: ids.clone(),
                goal,
            }]);
            assert_no_wall_crossing(&sim, &walls);
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 150.0 + 1e-3,
                    "unit squeezed through the near-exact gap to {:?}",
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
        assert!(
            da.min(db) < 6.0,
            "one unit settles at the goal: {}",
            da.min(db)
        );
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
        assert!(
            at_goal <= 2,
            "units crammed onto the goal centre: {at_goal}"
        );
        // Blob straddles the goal (not piled up short of it), centroid within
        // the group's own arrival radius.
        let axis = (goal - v(12.0, 12.0)).normalized();
        let (mut behind, mut past) = (f32::MAX, f32::MIN);
        for u in &us {
            let t = (u.pos - goal).dot(axis);
            behind = behind.min(t);
            past = past.max(t);
        }
        assert!(
            behind < 0.0 && past > 0.0,
            "blob must straddle the goal, not stop short: behind={behind} past={past}"
        );
        let r = us[0].radius;
        let arrival_r = r
            * (ARRIVAL_RADIUS_FACTOR.get() * (us.len() as f32).sqrt()).max(ARRIVAL_MIN_RADII.get());
        let mut c = Vector2::ZERO;
        for u in &us {
            c += u.pos;
        }
        c *= 1.0 / us.len() as f32;
        assert!(
            dist(c, goal) < arrival_r,
            "group centre outside its arrival radius: {} (arrival_r={arrival_r})",
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
        // A trailing unit in line behind a leader, both marching to a shared goal
        // (placed along the travel axis so the straight-line fan stays neutral and
        // cohesion is the only differing effect). Same group: cohesion pulls the
        // straggler forward and it closes the gap. Different groups: no cross-group
        // pull, so the gap holds at what the parallel paths leave it.
        let gap_after = |same_group: bool| -> f32 {
            let mut sim = rooms_sim(3, 1, 4);
            let a = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
            let b = spawn(&mut sim, v(30.0, 50.0), 5.0, 30.0);
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
            step_n(&mut sim, 40);
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
        // group fully settle into its blob, so the measured window is steady
        // state (the soft separation push settles gradually).
        step_n(&mut sim, 120);
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
            orders: VecDeque::new(),
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
            orders: VecDeque::new(),
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

    // ── Group pathing (shared channel / clustering / merge) ──────────────────

    fn group_of(sim: &Sim, id: UnitId) -> u32 {
        unit(sim, id).group
    }

    /// L-bend: horizontal arm [0,200]×[0,50] joined to vertical arm
    /// [150,200]×[0,200]; inside corner at (150,50). Single boundary polygon.
    fn lbend_map() -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let pts = vec![
            v(0.0, 0.0),
            v(200.0, 0.0),
            v(200.0, 200.0),
            v(150.0, 200.0),
            v(150.0, 50.0),
            v(0.0, 50.0),
        ];
        let cons = vec![(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0)];
        (pts, cons)
    }

    /// 200×200 box split by a solid interior wall at x=100 (no door): the left
    /// half (x<100) and right half (x>100) are fully disconnected.
    fn partitioned_map() -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let pts = vec![
            v(0.0, 0.0),     // 0
            v(200.0, 0.0),   // 1
            v(200.0, 200.0), // 2
            v(0.0, 200.0),   // 3
            v(100.0, 0.0),   // 4
            v(100.0, 200.0), // 5
        ];
        let cons = vec![
            (0, 4),
            (4, 1), // bottom, split at the wall foot
            (1, 2), // right
            (2, 5),
            (5, 3), // top, split at the wall head
            (3, 0), // left
            (4, 5), // interior wall
        ];
        (pts, cons)
    }

    #[test]
    fn test_move_into_sealed_region_never_crosses_wall() {
        // A goal inside a fully sealed-off region (no door) is unreachable.
        // Whatever the units do (idle, or shuffle on their own side), none may
        // end up across the sealing wall — a move toward an unreachable spot
        // must never tunnel units into the closed-off area.
        let (points, constraints) = partitioned_map();
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 3);
        let goal = v(150.0, 100.0); // right half — sealed off from the units
        let mut ids = Vec::new();
        for i in 0..16 {
            ids.push(spawn(
                &mut sim,
                v(70.0 + 6.0 * (i % 4) as f32, 80.0 + 6.0 * (i / 4) as f32),
                5.0,
                30.0,
            ));
        }
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);
        for _ in 0..400 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 100.0 + 1e-3,
                    "unit crossed the sealing wall into the closed-off area: {:?}",
                    u.pos
                );
            }
        }
    }

    #[test]
    fn test_group_move_to_sealed_goal_near_edge_does_not_cross() {
        // Goal just inside a sealed-off area, close to the constraint edge. A
        // single unit correctly idles (it is its own seed and gets the empty
        // path), but a *group*'s fan synthesises a converge waypoint backed off
        // toward the seed — which for a near-edge goal lands on the reachable
        // side — and route_onto_channel then appends the unreachable goal leg,
        // producing a path that pokes across the wall. Fast units follow it
        // through. No unit may ever cross the sealing wall.
        let (points, constraints) = partitioned_map(); // solid wall x=100, no door
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 1);
        let goal = v(103.0, 100.0); // just inside the sealed right half
        let a = spawn(&mut sim, v(88.0, 100.0), 5.0, 300.0); // fast
        let b = spawn(&mut sim, v(94.0, 100.0), 5.0, 300.0);
        sim.step(&[Command::Move {
            units: vec![a, b],
            goal,
        }]);
        for _ in 0..120 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 100.0 + 1e-3,
                    "unit crossed the sealing wall into the closed area: {:?}",
                    u.pos
                );
            }
        }
    }

    #[test]
    fn test_lbend_flock_fans_across_bend() {
        let (points, constraints) = lbend_map();
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 1);
        let goal = v(175.0, 190.0);
        // A row across the corridor width (y = 10..40); the inside lane (y=40,
        // nearest the inside corner) should round tight, the outside (y=10) wide.
        let ids: Vec<_> = (0..4)
            .map(|i| spawn(&mut sim, v(15.0, 10.0 + 10.0 * i as f32), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);

        // Bend waypoint (the apex unit-0 would single-file through) fans out, one
        // distinct crossing per lane, monotone in spawn (lateral) order.
        let bend_x: Vec<f32> = ids
            .iter()
            .map(|&id| {
                let p = &unit(&sim, id).path;
                p[p.len() - 2].x
            })
            .collect();
        assert!(
            bend_x[0] > bend_x[1] && bend_x[1] > bend_x[2] && bend_x[2] > bend_x[3],
            "bend crossings must fan monotonically across the corridor: {bend_x:?}"
        );
        assert!(
            bend_x[0] - bend_x[3] > 4.0,
            "flock must spread across the bend, got {}",
            bend_x[0] - bend_x[3]
        );

        for _ in 0..400 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
        for &id in &ids {
            assert!(
                dist(unit(&sim, id).pos, goal) < 40.0,
                "unit short of goal: {:?}",
                unit(&sim, id).pos
            );
        }
    }

    #[test]
    fn test_straight_flock_fans_into_lanes() {
        // On open ground (no corners) a flock heading straight to the goal must
        // still fan into parallel lanes instead of single-filing the line and
        // crushing together — the same spread a corridor bend gives.
        let mut sim = rooms_sim(1, 1, 1);
        let goal = v(85.0, 40.0);
        // A column spread laterally to the (horizontal) travel direction.
        let ids: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(15.0, 20.0 + 20.0 * i as f32), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);
        // Each non-seed unit holds a near-goal waypoint at most of its lane, so a
        // good fraction of the start spread (40) is kept rather than collapsing.
        let mid_y: Vec<f32> = ids.iter().map(|&id| unit(&sim, id).path[0].y).collect();
        let spread = mid_y.iter().cloned().fold(f32::MIN, f32::max)
            - mid_y.iter().cloned().fold(f32::MAX, f32::min);
        assert!(
            spread > 20.0,
            "flock collapsed instead of holding its lanes: {mid_y:?}"
        );
    }

    #[test]
    fn test_outer_unit_does_not_split_around_obstacle() {
        // A wide flock vs a central obstacle with two ways around. The flock
        // commits to one side (the seed's); an outer unit whose own shortest
        // path rounds the *other* way must still follow the group, not split off
        // onto a private path. (Regression: build_offset_path failed for that
        // unit and the fallback gave it an individual, other-side route.)
        let mut sim = rooms_sim(1, 1, 1);
        sim.step(&[Command::AddObstacle {
            points: vec![v(47.0, 30.0), v(53.0, 30.0), v(53.0, 70.0), v(47.0, 70.0)],
        }]);
        let goal = v(90.0, 50.0);
        let ids: Vec<_> = (0..10)
            .map(|i| spawn(&mut sim, v(15.0, 18.0 + 7.0 * i as f32), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);
        // The flock rounds the bottom (below y=30); no unit detours over the top
        // (above y=70) on a private route.
        for (k, &id) in ids.iter().enumerate() {
            let maxy = unit(&sim, id).path.iter().fold(0.0f32, |m, p| m.max(p.y));
            assert!(
                maxy < 70.0,
                "unit {k} split to the far side of the obstacle: maxy {maxy}"
            );
        }
    }

    /// Box 0..200 x 0..100 with a vertical wall at x=100: wide passages around
    /// the ends (y<25, y>75) and a narrow middle gap (y 45..55). Small units fit
    /// the gap (short straight route); big units must detour around an end.
    fn narrow_gap_map() -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let pts = vec![
            v(0.0, 0.0),
            v(200.0, 0.0),
            v(200.0, 100.0),
            v(0.0, 100.0),
            v(100.0, 25.0),
            v(100.0, 45.0),
            v(100.0, 55.0),
            v(100.0, 75.0),
        ];
        let cons = vec![(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (6, 7)];
        (pts, cons)
    }

    #[test]
    fn test_mixed_sizes_route_by_clearance() {
        // Big and small units commanded together: the small ones fit the narrow
        // middle gap (short, straight) while the big ones must detour around an
        // end. The group must not all adhere to the largest size.
        let (points, constraints) = narrow_gap_map();
        let mut sim = Sim::new(points, &constraints, 1);
        let goal = v(185.0, 50.0);
        let big: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(15.0, 44.0 + 4.0 * i as f32), 8.0, 25.0))
            .collect();
        let small: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(30.0, 44.0 + 4.0 * i as f32), 4.0, 25.0))
            .collect();
        let mut all = big.clone();
        all.extend(small.clone());
        sim.step(&[Command::Move { units: all, goal }]);

        // Small units cross straight through the gap (y stays in 45..55).
        for &id in &small {
            for w in &unit(&sim, id).path {
                assert!(
                    (45.0..=55.0).contains(&w.y),
                    "small unit left the gap route at {w:?}"
                );
            }
        }
        // Big units can't fit the gap, so they detour past an end (below y=30).
        for &id in &big {
            let miny = unit(&sim, id).path.iter().fold(999.0f32, |m, p| m.min(p.y));
            assert!(
                miny < 30.0,
                "big unit didn't detour around the end: miny {miny}"
            );
        }
        // Different sizes are separate flocks.
        assert_ne!(group_of(&sim, big[0]), group_of(&sim, small[0]));
    }

    #[test]
    fn test_move_clusters_separate_far_groups() {
        // One Move over two clusters far apart (> R_COH and wall-separated):
        // each cluster gets its own group id and shared channel.
        let mut sim = rooms_sim(3, 1, 1);
        let goal = v(150.0, 50.0);
        let a: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(20.0 + 6.0 * i as f32, 50.0), 5.0, 30.0))
            .collect();
        let b: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(260.0 + 6.0 * i as f32, 50.0), 5.0, 30.0))
            .collect();
        let all: Vec<_> = a.iter().chain(&b).copied().collect();
        sim.step(&[Command::Move { units: all, goal }]);
        let (ga, gb) = (group_of(&sim, a[0]), group_of(&sim, b[0]));
        assert!(
            a.iter().all(|&id| group_of(&sim, id) == ga),
            "cluster A split"
        );
        assert!(
            b.iter().all(|&id| group_of(&sim, id) == gb),
            "cluster B split"
        );
        assert_ne!(ga, gb, "far clusters must get distinct groups");
    }

    #[test]
    fn test_no_cluster_across_wall() {
        // Two units within R_COH but separated by a wall (not the door) must not
        // share a flock — the clear-LoS gate keeps them apart.
        let mut sim = rooms_sim(2, 1, 1); // wall x=100, door y∈[35,65]
        let a = spawn(&mut sim, v(90.0, 20.0), 5.0, 30.0);
        let b = spawn(&mut sim, v(110.0, 20.0), 5.0, 30.0);
        sim.step(&[Command::Move {
            units: vec![a, b],
            goal: v(150.0, 50.0),
        }]);
        assert_ne!(
            group_of(&sim, a),
            group_of(&sim, b),
            "wall-separated units must not cluster"
        );
    }

    #[test]
    fn test_convergence_merge_same_goal() {
        // Two distinct same-goal flocks that meet relabel to one group and stay
        // merged thereafter.
        let mut sim = rooms_sim(1, 1, 1); // open 100×100 room
        let goal = v(50.0, 50.0);
        let a: Vec<_> = (0..4)
            .map(|i| spawn(&mut sim, v(15.0 + 6.0 * i as f32, 20.0), 5.0, 25.0))
            .collect();
        let b: Vec<_> = (0..4)
            .map(|i| spawn(&mut sim, v(15.0 + 6.0 * i as f32, 80.0), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: a.clone(),
            goal,
        }]);
        sim.step(&[Command::Move {
            units: b.clone(),
            goal,
        }]);
        assert_ne!(group_of(&sim, a[0]), group_of(&sim, b[0]), "start distinct");
        step_n(&mut sim, 120);
        let g = group_of(&sim, a[0]);
        assert!(
            a.iter().chain(&b).all(|&id| group_of(&sim, id) == g),
            "converging same-goal flocks must merge to one group"
        );
    }

    #[test]
    fn test_no_merge_different_goals() {
        // Flocks that cross paths but head to different goals never merge.
        let mut sim = rooms_sim(1, 1, 1);
        let a: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(15.0, 40.0 + 6.0 * i as f32), 5.0, 25.0))
            .collect();
        let b: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(40.0 + 6.0 * i as f32, 15.0), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: a.clone(),
            goal: v(85.0, 50.0),
        }]);
        sim.step(&[Command::Move {
            units: b.clone(),
            goal: v(50.0, 85.0),
        }]);
        let (ga, gb) = (group_of(&sim, a[0]), group_of(&sim, b[0]));
        assert_ne!(ga, gb);
        for _ in 0..60 {
            sim.step(&[]);
            assert_eq!(group_of(&sim, a[0]), ga, "flock A group changed");
            assert_eq!(group_of(&sim, b[0]), gb, "flock B group changed");
        }
    }

    #[test]
    fn test_reanchor_skips_overshot_waypoint() {
        // A unit shoved past an intermediate waypoint advances onto the next leg
        // instead of doubling back to the waypoint it overshot.
        let (points, constraints) = lbend_map();
        let mut sim = Sim::new(points, &constraints, 1);
        let goal = v(175.0, 190.0);
        let id = spawn(&mut sim, v(15.0, 25.0), 5.0, 25.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal,
        }]);
        assert!(
            unit(&sim, id).path.len() >= 2,
            "need an intermediate waypoint"
        );
        let corner = unit(&sim, id).path[0];
        let next = unit(&sim, id).path[1];
        // Teleport just past the corner toward the next waypoint (a crowd shove).
        let dir = norm(next - corner);
        {
            let u = sim.units.get_mut(id).unwrap();
            u.pos = corner + dir * 3.0;
            u.prev_pos = u.pos;
        }
        sim.step(&[]);
        assert!(
            unit(&sim, id).path_i >= 1,
            "must advance past the overshot waypoint, not steer back to it"
        );
    }

    #[test]
    fn test_determinism_multicluster_and_merge() {
        // A two-cluster move that converges and merges replays bit-identically.
        let script = || -> Vec<u64> {
            let mut sim = rooms_sim(1, 1, 42);
            let goal = v(50.0, 50.0);
            let a: Vec<_> = (0..4)
                .map(|i| spawn(&mut sim, v(15.0 + 6.0 * i as f32, 25.0), 5.0, 25.0))
                .collect();
            let b: Vec<_> = (0..4)
                .map(|i| spawn(&mut sim, v(15.0 + 6.0 * i as f32, 75.0), 5.0, 25.0))
                .collect();
            sim.step(&[Command::Move { units: a, goal }]);
            sim.step(&[Command::Move { units: b, goal }]);
            (0..150)
                .map(|_| {
                    sim.step(&[]);
                    sim.state_hash()
                })
                .collect()
        };
        assert_eq!(
            script(),
            script(),
            "multi-cluster + merge must be deterministic"
        );
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
