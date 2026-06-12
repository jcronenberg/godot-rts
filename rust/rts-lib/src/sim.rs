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
    /// Separation displacement accumulator, parallel to the dense arrays.
    disp: Vec<Vector2>,
    /// Wall-clamp face frontier and visited list (tiny per unit).
    faces: Vec<u32>,
    visited: Vec<u32>,
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
    /// integrate along paths → separation → wall clamp → advance tick.
    pub fn step(&mut self, commands: &[Command]) {
        for (_, u) in self.units.iter_mut() {
            u.prev_pos = u.pos;
        }
        for cmd in commands {
            self.apply(cmd);
        }
        let mesh_changed = self.rebuild_and_repath();
        self.integrate();
        self.separate();
        self.wall_clamp(mesh_changed);
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
                });
            }
            Command::Move { units, goal } => {
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
            }
        }
    }

    /// Pairwise push-out of overlapping unit circles. Forces are computed
    /// from start-of-tick positions into a displacement buffer and applied
    /// afterwards, so slot order can't leak into the result; per-unit
    /// displacement is clamped to `max_speed * DT`.
    fn separate(&mut self) {
        let s = &mut self.step_scratch;
        s.ids.clear();
        s.positions.clear();
        s.radii.clear();
        s.speeds.clear();
        s.disp.clear();
        let mut max_diameter = 0.0f32;
        for (id, u) in self.units.iter() {
            s.ids.push(id);
            s.positions.push(u.prev_pos);
            s.radii.push(u.radius);
            s.speeds.push(u.max_speed);
            s.disp.push(Vector2::ZERO);
            max_diameter = max_diameter.max(u.radius + u.radius);
        }
        if s.ids.len() < 2 || max_diameter <= 0.0 {
            return;
        }
        self.grid.rebuild(&s.positions, max_diameter);

        for i in 0..s.ids.len() {
            let p = s.positions[i];
            let (cx, cy) = self.grid.cell_coords(p);
            for ny in cy.saturating_sub(1)..=(cy + 1).min(self.grid.rows - 1) {
                for nx in cx.saturating_sub(1)..=(cx + 1).min(self.grid.cols - 1) {
                    for &j in self.grid.cell_entries(nx, ny) {
                        let j = j as usize;
                        if j <= i {
                            continue;
                        }
                        let delta = p - s.positions[j];
                        let min_dist = s.radii[i] + s.radii[j];
                        let d2 = delta.x * delta.x + delta.y * delta.y;
                        if d2 >= min_dist * min_dist {
                            continue;
                        }
                        let d = d2.sqrt();
                        // Coincident circles: deterministic x-axis tiebreak.
                        let dir = if d > 1e-6 {
                            delta * (1.0 / d)
                        } else {
                            Vector2::new(1.0, 0.0)
                        };
                        let push = dir * ((min_dist - d) * 0.5);
                        s.disp[i] += push;
                        s.disp[j] -= push;
                    }
                }
            }
        }

        for i in 0..s.ids.len() {
            let d = s.disp[i];
            let len2 = d.x * d.x + d.y * d.y;
            if len2 == 0.0 {
                continue;
            }
            let max_step = s.speeds[i] * DT;
            let len = len2.sqrt();
            let d = if len > max_step {
                d * (max_step / len)
            } else {
                d
            };
            let unit = self.units.get_mut(s.ids[i]).expect("dense id alive");
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
        for (_, unit) in self.units.iter_mut() {
            if unit.radius <= 0.0 || (!mesh_changed && unit.pos == unit.prev_pos) {
                continue;
            }
            // A push moves the circle and changes which faces it overlaps,
            // invalidating the walk in progress; re-walk until a pass applies
            // no push (bounded against corner ping-pong).
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
        }
    }

    /// FNV-1a over tick, unit slots (slot order) and RNG — divergence detector
    /// for determinism tests and future lockstep.
    pub fn state_hash(&self) -> u64 {
        let mut h = Fnv::new();
        h.write_u64(self.tick);
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

    /// Polyline length from the unit's position through its remaining waypoints.
    fn remaining_len(u: &Unit) -> f32 {
        let mut prev = u.pos;
        let mut total = 0.0;
        for &p in &u.path[u.path_i as usize..] {
            total += dist(prev, p);
            prev = p;
        }
        total
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
        let len = remaining_len(unit(&sim, id));
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
        let total = remaining_len(unit(&sim, id));
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
        let base_len = remaining_len(unit(&sim, id));

        // Narrow the doorway (gap x=100, y in [35, 65]).
        sim.step(&[Command::AddObstacle {
            points: vec![v(98.0, 40.0), v(102.0, 40.0), v(102.0, 60.0), v(98.0, 60.0)],
        }]);
        let u = unit(&sim, id);
        assert!(u.is_moving(), "detour must exist");
        let detour_len = remaining_len(u);
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

    #[test]
    fn test_steady_state_step_is_allocation_free() {
        let mut sim = rooms_sim(4, 4, 3);
        let mut ids = Vec::new();
        for i in 0..32 {
            ids.push(spawn(
                &mut sim,
                v(20.0 + 10.0 * (i % 8) as f32, 20.0 + 15.0 * (i / 8) as f32),
                5.0,
                20.0,
            ));
        }
        sim.step(&[Command::Move {
            units: ids,
            goal: v(350.0, 350.0),
        }]);
        // Warm scratch buffers (grid, dense arrays, locate paths).
        step_n(&mut sim, 10);
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
