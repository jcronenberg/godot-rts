use std::sync::Arc;

use godot::classes::Node;
use godot::prelude::*;
use rts_lib::navmesh::ObstacleId;
use rts_lib::sim::{Command, Order, Sim, TICK_RATE, UnitId};
use rts_lib::sim_runner::{Interpolator, SimHandle, Snapshot, display_pos};

/// Godot view of the Rust sim thread: commands in, snapshots out.
///
/// Per frame, call `poll(delta)` once to fetch fresh snapshots and advance the
/// render clock, then read `get_positions(alpha)` / ids / radii with the alpha
/// it returned. All gameplay state lives in the sim; this node only holds an
/// [`Interpolator`] over recent snapshots (alpha may exceed 1.0 — bounded
/// extrapolation). Getters read the bracket's newer snapshot.
#[derive(GodotClass)]
#[class(base=Node)]
pub struct Simulation {
    handle: Option<SimHandle>,
    interp: Interpolator,
    speed: f64,
    base: Base<Node>,
}

#[godot_api]
impl INode for Simulation {
    fn init(base: Base<Node>) -> Self {
        Simulation {
            handle: None,
            interp: Interpolator::new(Arc::new(Snapshot::default())),
            speed: 1.0,
            base,
        }
    }

    fn exit_tree(&mut self) {
        // Dropping the handle flags the sim thread and joins it.
        self.handle = None;
    }
}

#[godot_api]
impl Simulation {
    /// Build the sim over static map geometry (constraint index pairs into
    /// `points`) and start the sim thread.
    #[func]
    pub fn load_map(
        &mut self,
        points: PackedVector2Array,
        constraints: PackedInt32Array,
        seed: i64,
    ) {
        let constraints: Vec<(u32, u32)> = constraints
            .as_slice()
            .chunks_exact(2)
            .map(|c| (c[0] as u32, c[1] as u32))
            .collect();
        let sim = Sim::new(points.as_slice().to_vec(), &constraints, seed as u64);
        let handle = SimHandle::start(sim);
        self.interp = Interpolator::new(handle.snapshot());
        self.handle = Some(handle);
    }

    fn enqueue(&self, cmd: Command) {
        match &self.handle {
            Some(h) => h.enqueue(cmd),
            None => godot_warn!("Simulation: no map loaded; call load_map() first"),
        }
    }

    /// Queue a unit spawn; it appears in snapshots after the next tick.
    #[func]
    pub fn spawn_unit(&self, pos: Vector2, radius: f32, speed: f32) {
        self.enqueue(Command::Spawn { pos, radius, speed });
    }

    /// Order the given units (ids from `get_unit_ids`) to `goal`, interrupting
    /// and clearing any queued orders.
    #[func]
    pub fn move_units(&self, ids: PackedInt64Array, goal: Vector2) {
        self.enqueue(Command::Move {
            units: Self::unit_ids(&ids),
            goal,
        });
    }

    /// Append a move order to the given units' queues; they path to it after
    /// finishing their current and earlier-queued orders.
    #[func]
    pub fn queue_move(&self, ids: PackedInt64Array, goal: Vector2) {
        self.enqueue(Command::Queue {
            units: Self::unit_ids(&ids),
            order: Order::Move { goal },
        });
    }

    fn unit_ids(ids: &PackedInt64Array) -> Vec<UnitId> {
        ids.as_slice()
            .iter()
            .map(|&raw| UnitId::from_raw(raw as u64))
            .collect()
    }

    /// Queue a closed-polygon obstacle; returns the id the sim will assign
    /// (usable with `remove_obstacle` immediately), or -1 with no map.
    #[func]
    pub fn add_obstacle(&self, points: PackedVector2Array) -> i64 {
        match &self.handle {
            Some(h) => h.add_obstacle(points.as_slice().to_vec()).raw() as i64,
            None => {
                godot_warn!("Simulation: no map loaded; call load_map() first");
                -1
            }
        }
    }

    #[func]
    pub fn remove_obstacle(&self, id: i64) {
        self.enqueue(Command::RemoveObstacle {
            id: ObstacleId::from_raw(id as u64),
        });
    }

    #[func]
    pub fn set_paused(&self, paused: bool) {
        if let Some(h) = &self.handle {
            h.set_paused(paused);
        }
    }

    /// Sim speed multiplier (also drives the render clock).
    #[func]
    pub fn set_speed(&mut self, speed: f32) {
        self.speed = speed.clamp(0.01, 64.0) as f64;
        if let Some(h) = &self.handle {
            h.set_speed(speed);
        }
    }

    /// While on, snapshots carry per-unit paths for `get_unit_paths`.
    #[func]
    pub fn set_debug_overlay(&self, on: bool) {
        if let Some(h) = &self.handle {
            h.set_debug_overlay(on);
        }
    }

    /// Fetch the latest snapshot and advance the render clock by `delta`
    /// seconds. Returns the interpolation alpha for `get_positions`.
    #[func]
    pub fn poll(&mut self, delta: f64) -> f32 {
        let Some(handle) = &self.handle else {
            return 0.0;
        };
        // Surface sim-thread errors here, where engine printing is allowed.
        for error in handle.take_errors() {
            godot_error!("sim: {error}");
        }
        self.interp.push(handle.snapshot());
        self.interp.advance(delta * self.speed * TICK_RATE as f64)
    }

    /// Current tick of the latest snapshot.
    #[func]
    pub fn tick(&self) -> i64 {
        self.interp.cur().tick as i64
    }

    /// Wall-clock cost (ms) of the sim step behind the latest snapshot.
    #[func]
    pub fn step_ms(&self) -> f32 {
        self.interp.cur().step_ms
    }

    /// Unit ids in snapshot order; rows of the other getters align with this.
    #[func]
    pub fn get_unit_ids(&self) -> PackedInt64Array {
        self.interp
            .cur()
            .ids
            .iter()
            .map(|id| id.raw() as i64)
            .collect()
    }

    /// Positions interpolated between the snapshots bracketing render time;
    /// alpha > 1 extrapolates waypoint-ward along the last pair (see
    /// [`display_pos`]). Units with no sample in the older snapshot render
    /// at their current position.
    #[func]
    pub fn get_positions(&self, alpha: f32) -> PackedVector2Array {
        let (prev, cur) = (self.interp.prev(), self.interp.cur());
        let mut out = PackedVector2Array::new();
        out.resize(cur.ids.len());
        let slice = out.as_mut_slice();
        let mut p = 0;
        for (i, &id) in cur.ids.iter().enumerate() {
            let cur_pos = cur.positions[i];
            // Both id lists are in slot order: advance a single cursor.
            while p < prev.ids.len() && (prev.ids[p].raw() as u32) < (id.raw() as u32) {
                p += 1;
            }
            slice[i] = match prev.ids.get(p) {
                Some(&pid) if pid == id => {
                    display_pos(prev.positions[p], cur_pos, cur.waypoints[i], alpha)
                }
                _ => cur_pos,
            };
        }
        out
    }

    #[func]
    pub fn get_radii(&self) -> PackedFloat32Array {
        self.interp.cur().radii.as_slice().into()
    }

    #[func]
    pub fn get_velocities(&self) -> PackedVector2Array {
        PackedVector2Array::from(self.interp.cur().velocities.as_slice())
    }

    /// Next waypoint per unit (own position when idle).
    #[func]
    pub fn get_waypoints(&self) -> PackedVector2Array {
        PackedVector2Array::from(self.interp.cur().waypoints.as_slice())
    }

    /// Remaining path per unit; empty unless the debug overlay is on.
    #[func]
    pub fn get_unit_paths(&self) -> Array<PackedVector2Array> {
        let mut out = Array::new();
        if let Some(paths) = &self.interp.cur().debug_paths {
            for path in paths {
                out.push(&PackedVector2Array::from(path.as_slice()));
            }
        }
        out
    }

    /// Queued (not-yet-started) order goals per unit; empty unless the debug
    /// overlay is on. Rows align with `get_unit_ids`.
    #[func]
    pub fn get_unit_order_goals(&self) -> Array<PackedVector2Array> {
        let mut out = Array::new();
        if let Some(goals) = &self.interp.cur().debug_order_goals {
            for unit_goals in goals {
                out.push(&PackedVector2Array::from(unit_goals.as_slice()));
            }
        }
        out
    }

    /// Navmesh geometry token of the latest snapshot; when it moves and the
    /// overlay is visible, call `request_mesh_dump` and poll `take_mesh_dump`.
    #[func]
    pub fn nav_version(&self) -> i64 {
        self.interp.cur().nav_version as i64
    }

    #[func]
    pub fn request_mesh_dump(&self) {
        if let Some(h) = &self.handle {
            h.request_mesh_dump();
        }
    }

    /// The requested mesh dump, or an empty Dictionary while not ready.
    /// Keys: `version: int`, `points: PackedVector2Array`,
    /// `edges: PackedVector2Array` (endpoint pairs), `indices: PackedInt32Array`.
    #[func]
    pub fn take_mesh_dump(&self) -> VarDictionary {
        let mut out = VarDictionary::new();
        let Some(dump) = self.handle.as_ref().and_then(|h| h.take_mesh_dump()) else {
            return out;
        };
        out.set("version", dump.version as i64);
        out.set("points", PackedVector2Array::from(dump.points.as_slice()));
        out.set(
            "edges",
            PackedVector2Array::from(dump.constrained_edges.as_slice()),
        );
        let indices: PackedInt32Array = dump.indices.iter().map(|&i| i as i32).collect();
        out.set("indices", indices);
        out
    }
}
