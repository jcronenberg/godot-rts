//! Sim thread runner: deadline loop around [`Sim::step`], command queue in,
//! `Arc`-swapped snapshots out. Pacing lives here, never in the sim core —
//! tests/benches/replays call `step` directly. No Godot dependencies.

use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use godot::prelude::Vector2;

use crate::sim::{Command, DT, Sim, UnitId};

/// Per-tick view state. Cheap parallel arrays; the view lerps two of these.
#[derive(Default)]
pub struct Snapshot {
    pub tick: u64,
    /// Navmesh geometry token; the view requests a mesh dump when it moves.
    pub nav_version: u64,
    pub ids: Vec<UnitId>,
    pub positions: Vec<Vector2>,
    pub velocities: Vec<Vector2>,
    /// Next waypoint per unit (own position when idle) — enables path-aware
    /// view-side extrapolation without extra round-trips.
    pub waypoints: Vec<Vector2>,
    pub radii: Vec<f32>,
    /// Full remaining path per unit; filled only while the debug overlay is on.
    pub debug_paths: Option<Vec<Vec<Vector2>>>,
    /// Wall-clock cost of the `step()` that produced this snapshot, in ms.
    /// Instrumentation only — never feeds back into sim state.
    pub step_ms: f32,
}

impl Snapshot {
    fn capture(sim: &Sim, debug_overlay: bool) -> Snapshot {
        let n = sim.units().len();
        let mut snap = Snapshot {
            tick: sim.tick(),
            nav_version: sim.navmesh().version(),
            ids: Vec::with_capacity(n),
            positions: Vec::with_capacity(n),
            velocities: Vec::with_capacity(n),
            waypoints: Vec::with_capacity(n),
            radii: Vec::with_capacity(n),
            debug_paths: debug_overlay.then(|| Vec::with_capacity(n)),
            step_ms: 0.0,
        };
        for (id, u) in sim.units().iter() {
            snap.ids.push(id);
            snap.positions.push(u.pos);
            snap.velocities.push((u.pos - u.prev_pos) / DT);
            snap.waypoints.push(u.waypoint());
            snap.radii.push(u.radius);
            if let Some(paths) = &mut snap.debug_paths {
                // Prepend the live position so the polyline starts at the
                // unit, not at its next corner.
                let mut path = Vec::new();
                if let Some(rest) = u.path.get(u.path_i as usize..)
                    && !rest.is_empty()
                {
                    path.reserve(rest.len() + 1);
                    path.push(u.pos);
                    path.extend_from_slice(rest);
                }
                paths.push(path);
            }
        }
        snap
    }
}

/// Render-side clock mapping frame time onto snapshot intervals.
///
/// Holds ticks elapsed past the previous snapshot; the view lerps with the
/// returned alpha. Leftover time carries across swaps so motion stays
/// continuous, but a lead of a full tick or more (frame stall, pause,
/// fast-forward) is discarded and the clock resyncs to the interval start:
/// such a lead can never be caught up smoothly (alpha is already pinned),
/// and carrying it would pin alpha at 1.0 permanently.
#[derive(Default)]
pub struct RenderClock {
    ticks: f64,
}

impl RenderClock {
    pub fn new() -> RenderClock {
        RenderClock::default()
    }

    /// Call when a new snapshot becomes current (one interval consumed).
    pub fn on_swap(&mut self) {
        let leftover = self.ticks - 1.0;
        self.ticks = if leftover < 1.0 {
            leftover.max(0.0)
        } else {
            0.0
        };
    }

    /// Advance by elapsed ticks (`delta * speed * TICK_RATE`); returns alpha.
    pub fn advance(&mut self, ticks: f64) -> f32 {
        self.ticks += ticks;
        self.ticks.min(1.0) as f32
    }
}

/// Navmesh geometry for the debug overlay, produced on request only.
pub struct MeshDump {
    pub version: u64,
    pub points: Vec<Vector2>,
    /// Constrained edges as endpoint pairs (2 entries per edge).
    pub constrained_edges: Vec<Vector2>,
    /// Triangle vertex indices, 3 per face.
    pub indices: Vec<u32>,
}

struct Shared {
    commands: Mutex<Vec<Command>>,
    snapshot: Mutex<Arc<Snapshot>>,
    quit: AtomicBool,
    paused: AtomicBool,
    /// Sim speed multiplier as f32 bits (1.0 = real time).
    speed_bits: AtomicU32,
    debug_overlay: AtomicBool,
    mesh_dump_requested: AtomicBool,
    mesh_dump: Mutex<Option<MeshDump>>,
    /// Errors reported on the sim thread (engine printing is main-thread
    /// only); the view drains and prints them.
    errors: Mutex<Vec<String>>,
    /// Mirrors the sim's sequential obstacle-id assignment (see `add_obstacle`).
    next_obstacle_id: AtomicU64,
}

/// Handle owned by the view side; dropping it (or calling [`SimHandle::shutdown`])
/// stops and joins the sim thread.
pub struct SimHandle {
    shared: Arc<Shared>,
    thread: Option<JoinHandle<()>>,
}

impl SimHandle {
    /// Take ownership of `sim` and start stepping it at [`crate::sim::TICK_RATE`].
    pub fn start(sim: Sim) -> SimHandle {
        let shared = Arc::new(Shared {
            commands: Mutex::new(Vec::new()),
            snapshot: Mutex::new(Arc::new(Snapshot::capture(&sim, false))),
            quit: AtomicBool::new(false),
            paused: AtomicBool::new(false),
            speed_bits: AtomicU32::new(1.0f32.to_bits()),
            debug_overlay: AtomicBool::new(false),
            mesh_dump_requested: AtomicBool::new(false),
            mesh_dump: Mutex::new(None),
            errors: Mutex::new(Vec::new()),
            next_obstacle_id: AtomicU64::new(sim.next_obstacle_id()),
        });
        let thread = {
            let shared = Arc::clone(&shared);
            std::thread::Builder::new()
                .name("sim".into())
                .spawn(move || run_loop(sim, &shared))
                .expect("spawn sim thread")
        };
        SimHandle {
            shared,
            thread: Some(thread),
        }
    }

    /// Queue a command for the next tick.
    pub fn enqueue(&self, cmd: Command) {
        self.shared.commands.lock().unwrap().push(cmd);
    }

    /// Queue an `AddObstacle` and return the id the sim will assign to it.
    ///
    /// `DynamicNavmesh` hands out sequential ids and the sim is its only
    /// writer, so a counter seeded from the sim at start and bumped per
    /// enqueued `AddObstacle` (under the queue lock, preserving order)
    /// predicts the id exactly.
    pub fn add_obstacle(&self, points: Vec<Vector2>) -> crate::navmesh::ObstacleId {
        let mut queue = self.shared.commands.lock().unwrap();
        let id = self.shared.next_obstacle_id.fetch_add(1, Ordering::Relaxed);
        queue.push(Command::AddObstacle { points });
        crate::navmesh::ObstacleId::from_raw(id)
    }

    /// Latest published snapshot (initial pre-tick snapshot before the first step).
    pub fn snapshot(&self) -> Arc<Snapshot> {
        Arc::clone(&self.shared.snapshot.lock().unwrap())
    }

    pub fn set_paused(&self, paused: bool) {
        self.shared.paused.store(paused, Ordering::Relaxed);
    }

    /// Sim speed multiplier (clamped to a sane positive range).
    pub fn set_speed(&self, speed: f32) {
        let speed = speed.clamp(0.01, 64.0);
        self.shared
            .speed_bits
            .store(speed.to_bits(), Ordering::Relaxed);
    }

    /// While on, snapshots carry per-unit remaining paths.
    pub fn set_debug_overlay(&self, on: bool) {
        self.shared.debug_overlay.store(on, Ordering::Relaxed);
    }

    /// Ask the sim thread to publish a [`MeshDump`] after its next step.
    pub fn request_mesh_dump(&self) {
        self.shared
            .mesh_dump_requested
            .store(true, Ordering::Relaxed);
    }

    /// Collect a previously requested mesh dump, if ready.
    pub fn take_mesh_dump(&self) -> Option<MeshDump> {
        self.shared.mesh_dump.lock().unwrap().take()
    }

    /// Drain errors the sim thread reported since the last call; the caller
    /// is expected to surface them (e.g. `godot_error!` on the main thread).
    pub fn take_errors(&self) -> Vec<String> {
        std::mem::take(&mut self.shared.errors.lock().unwrap())
    }

    /// Stop and join the sim thread.
    pub fn shutdown(mut self) {
        self.join();
    }

    fn join(&mut self) {
        self.shared.quit.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

impl Drop for SimHandle {
    fn drop(&mut self) {
        self.join();
    }
}

/// Deadline loop: sleep to the next tick deadline, drain the queue, step,
/// publish. On overrun the loop catches up by at most one extra tick; once
/// it falls more than one period behind wall clock the deadline resets and
/// the lost ticks are dropped instead of replayed.
fn run_loop(mut sim: Sim, shared: &Shared) {
    // Queue report_error! messages instead of engine-printing (main-thread only).
    crate::report::install_collector();
    let mut commands: Vec<Command> = Vec::new();
    let mut deadline = Instant::now();
    while !shared.quit.load(Ordering::Relaxed) {
        let speed = f32::from_bits(shared.speed_bits.load(Ordering::Relaxed));
        let period = Duration::from_secs_f64(DT as f64 / speed as f64);
        deadline += period;
        let now = Instant::now();
        if let Some(wait) = deadline.checked_duration_since(now) {
            std::thread::sleep(wait);
        } else if now - deadline > period {
            deadline = now; // >1 period behind: drop lost ticks instead of bursting
        }
        if shared.paused.load(Ordering::Relaxed) {
            continue;
        }

        {
            let mut queue = shared.commands.lock().unwrap();
            std::mem::swap(&mut commands, &mut *queue);
        }
        let step_start = Instant::now();
        sim.step(&commands);
        let step_ms = step_start.elapsed().as_secs_f32() * 1000.0;
        commands.clear();

        let errors = crate::report::drain();
        if !errors.is_empty() {
            shared.errors.lock().unwrap().extend(errors);
        }

        let debug = shared.debug_overlay.load(Ordering::Relaxed);
        let mut snap = Snapshot::capture(&sim, debug);
        snap.step_ms = step_ms;
        *shared.snapshot.lock().unwrap() = Arc::new(snap);

        if shared.mesh_dump_requested.swap(false, Ordering::Relaxed) {
            let cdt = sim.navmesh();
            let mut indices = Vec::with_capacity(cdt.num_faces() as usize * 3);
            for f in 0..cdt.num_faces() {
                indices.extend(cdt.face_vertices(f));
            }
            let mut edges = Vec::new();
            cdt.for_each_constrained_edge(|a, b| {
                edges.push(a);
                edges.push(b);
            });
            *shared.mesh_dump.lock().unwrap() = Some(MeshDump {
                version: cdt.version(),
                points: cdt.points().to_vec(),
                constrained_edges: edges,
                indices,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapgen::rooms_map;

    fn v(x: f32, y: f32) -> Vector2 {
        Vector2::new(x, y)
    }

    fn start_rooms(cols: usize, rows: usize) -> SimHandle {
        let (points, constraints) = rooms_map(cols, rows);
        SimHandle::start(Sim::new(points, &constraints, 1))
    }

    fn wait_for(handle: &SimHandle, mut pred: impl FnMut(&Snapshot) -> bool) -> Arc<Snapshot> {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            let snap = handle.snapshot();
            if pred(&snap) {
                return snap;
            }
            assert!(Instant::now() < deadline, "condition not met within 5s");
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    #[test]
    fn test_render_clock_steady_state_is_smooth() {
        // 2 frames per tick: alpha must advance every frame, no pinning.
        let mut clock = RenderClock::new();
        for _ in 0..100 {
            let a = clock.advance(0.5);
            let b = clock.advance(0.5);
            assert!(b > a, "alpha must advance within the interval");
            clock.on_swap();
        }
    }

    #[test]
    fn test_render_clock_recovers_from_long_stall() {
        // Regression: a lead of >= 1 tick at swap (long frame, pause,
        // fast-forward) must resync, not lock alpha at 1.0 forever.
        let mut clock = RenderClock::new();
        clock.advance(2.5); // frame stall spanning multiple ticks
        clock.on_swap();
        for _ in 0..10 {
            let a = clock.advance(0.5);
            let b = clock.advance(0.5);
            assert!(a < 1.0, "alpha pinned after stall: interpolation dead");
            assert!(b > a, "alpha must advance within the interval");
            clock.on_swap();
        }
    }

    #[test]
    fn test_render_clock_carries_sub_tick_leftover() {
        // A short hiccup carries over instead of resyncing.
        let mut clock = RenderClock::new();
        let alpha = clock.advance(1.4);
        assert_eq!(alpha, 1.0);
        clock.on_swap();
        // 0.4 carried + 0.1 advanced.
        assert!((clock.advance(0.1) - 0.5).abs() < 1e-9);
    }

    #[test]
    fn test_runner_ticks_and_moves_units() {
        let handle = start_rooms(2, 1);
        handle.set_speed(8.0); // keep the test fast
        handle.enqueue(Command::Spawn {
            pos: v(50.0, 50.0),
            radius: 5.0,
            speed: 20.0,
        });
        let snap = wait_for(&handle, |s| !s.ids.is_empty());
        handle.enqueue(Command::Move {
            units: vec![snap.ids[0]],
            goal: v(150.0, 50.0),
        });
        let moved = wait_for(&handle, |s| {
            !s.positions.is_empty() && s.positions[0].x > 60.0
        });
        assert!(moved.tick > 0);
        assert!(moved.step_ms > 0.0, "step cost must be measured");
        // Moving units report a forward velocity and a real waypoint.
        assert!(moved.velocities[0].length() > 1.0);
        assert_ne!(moved.waypoints[0], moved.positions[0]);
        handle.shutdown();
    }

    #[test]
    fn test_pause_freezes_ticks() {
        let handle = start_rooms(1, 1);
        handle.set_speed(8.0);
        let snap = wait_for(&handle, |s| s.tick > 2);
        handle.set_paused(true);
        std::thread::sleep(Duration::from_millis(60));
        let a = handle.snapshot().tick;
        std::thread::sleep(Duration::from_millis(120));
        let b = handle.snapshot().tick;
        assert!(b <= a + 1, "ticks advanced while paused: {a} -> {b}");
        handle.set_paused(false);
        wait_for(&handle, |s| s.tick > snap.tick + 4);
    }

    #[test]
    fn test_obstacle_id_prediction_matches_sim() {
        let handle = start_rooms(2, 1);
        handle.set_speed(8.0);
        let a = handle.add_obstacle(vec![v(20.0, 20.0), v(30.0, 20.0), v(25.0, 30.0)]);
        let b = handle.add_obstacle(vec![v(60.0, 60.0), v(70.0, 60.0), v(65.0, 70.0)]);
        assert_eq!(a.raw(), 0);
        assert_eq!(b.raw(), 1);
        let v0 = handle.snapshot().nav_version;
        // Removing by the predicted id must take the navmesh back (version moves
        // on every rebuild; obstacle count returning to 0 is observable via dump).
        wait_for(&handle, |s| s.nav_version != v0);
        handle.enqueue(Command::RemoveObstacle { id: a });
        handle.enqueue(Command::RemoveObstacle { id: b });
        let v1 = handle.snapshot().nav_version;
        wait_for(&handle, |s| s.nav_version != v1);
        handle.request_mesh_dump();
        let deadline = Instant::now() + Duration::from_secs(5);
        let dump = loop {
            if let Some(d) = handle.take_mesh_dump() {
                break d;
            }
            assert!(Instant::now() < deadline, "mesh dump not produced");
            std::thread::sleep(Duration::from_millis(5));
        };
        // Back to the bare map: same geometry as a fresh navmesh, including
        // every constrained edge (hull edges have no twin — must not be lost).
        let (points, constraints) = rooms_map(2, 1);
        let fresh = crate::navmesh::DynamicNavmesh::new(points, &constraints);
        assert_eq!(dump.points.len(), fresh.navmesh().points().len());
        assert_eq!(dump.indices.len() as u32, fresh.navmesh().num_faces() * 3);
        let mut fresh_edges = 0;
        fresh
            .navmesh()
            .for_each_constrained_edge(|_, _| fresh_edges += 2);
        assert_eq!(dump.constrained_edges.len(), fresh_edges);
        handle.shutdown();
    }

    #[test]
    fn test_sim_thread_errors_are_forwarded() {
        let handle = start_rooms(2, 1);
        handle.set_speed(8.0);
        // Crosses the wall segment (100,0)-(100,35): rejected with an error.
        handle.add_obstacle(vec![
            v(90.0, 10.0),
            v(110.0, 10.0),
            v(110.0, 30.0),
            v(90.0, 30.0),
        ]);
        let deadline = Instant::now() + Duration::from_secs(5);
        let errors = loop {
            let errors = handle.take_errors();
            if !errors.is_empty() {
                break errors;
            }
            assert!(Instant::now() < deadline, "error not forwarded within 5s");
            std::thread::sleep(Duration::from_millis(5));
        };
        assert!(
            errors
                .iter()
                .any(|e| e.contains("intersects existing constraint")),
            "unexpected errors: {errors:?}"
        );
        handle.shutdown();
    }

    #[test]
    fn test_debug_overlay_paths_in_snapshot() {
        let handle = start_rooms(2, 1);
        handle.set_speed(8.0);
        handle.set_debug_overlay(true);
        handle.enqueue(Command::Spawn {
            pos: v(50.0, 50.0),
            radius: 5.0,
            speed: 20.0,
        });
        let snap = wait_for(&handle, |s| !s.ids.is_empty());
        handle.enqueue(Command::Move {
            units: vec![snap.ids[0]],
            goal: v(150.0, 50.0),
        });
        let snap = wait_for(&handle, |s| {
            s.debug_paths
                .as_ref()
                .is_some_and(|p| p.first().is_some_and(|p| !p.is_empty()))
        });
        let path = &snap.debug_paths.as_ref().unwrap()[0];
        assert!(path.len() >= 2, "debug path must be drawable: {path:?}");
        assert_eq!(path[0], snap.positions[0], "path starts at the unit");
        assert_eq!(*path.last().unwrap(), v(150.0, 50.0));
        handle.set_debug_overlay(false);
        wait_for(&handle, |s| s.debug_paths.is_none());
    }
}
