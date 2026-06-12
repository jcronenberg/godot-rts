//! Sim thread runner: deadline loop around [`Sim::step`], command queue in,
//! `Arc`-swapped snapshots out. Pacing lives here, never in the sim core —
//! tests/benches/replays call `step` directly. No Godot dependencies.

use std::collections::VecDeque;
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

/// View-side snapshot buffer with a continuous render clock that targets
/// estimated *current* sim time, not the newest snapshot. Inside a snapshot
/// interval the clock runs past the newest tick and the view extrapolates
/// (alpha > 1 on the last pair, capped at one tick), so display latency is
/// near zero at any sim speed; mispredictions are bounded by one tick of
/// unit movement. Drift is corrected by nudging playback rate, never by
/// stepping backwards, so rendered time is monotonic.
pub struct Interpolator {
    /// Snapshots in tick order; `[0]` is the current bracket start.
    buf: VecDeque<Arc<Snapshot>>,
    /// Continuous render position in tick units.
    render_tick: f64,
    /// Estimated continuous sim time: `push` resyncs it to the arriving
    /// snapshot's tick; `advance` keeps it in `[newest, newest + MAX_EXTRAP]`.
    sim_now: f64,
}

impl Interpolator {
    /// Ticks the view may extrapolate past the newest snapshot.
    const MAX_EXTRAP: f64 = 1.0;
    /// Phase error tolerated before nudging the playback rate.
    const DEADBAND: f64 = 0.25;
    /// Playback rate nudge outside the deadband.
    const SLEW: f64 = 0.05;
    /// Phase lag beyond which the clock snaps forward to the target.
    const SNAP: f64 = 1.5;

    pub fn new(initial: Arc<Snapshot>) -> Interpolator {
        let render_tick = initial.tick as f64;
        Interpolator {
            buf: VecDeque::from([initial]),
            render_tick,
            sim_now: render_tick,
        }
    }

    /// Buffer a snapshot; ignored unless newer than the newest held.
    pub fn push(&mut self, snap: Arc<Snapshot>) {
        if snap.tick > self.buf.back().unwrap().tick {
            // Authoritative resync: the sim just published this tick. Must
            // overwrite, not max(): max() would let frame-clock drift
            // accumulate until sim_now saturates at the extrapolation cap
            // and rendered time pins there. The transient negative err an
            // overwrite can cause is absorbed by the deadband.
            self.sim_now = snap.tick as f64;
            self.buf.push_back(snap);
        }
    }

    /// Older snapshot of the current bracket.
    pub fn prev(&self) -> &Arc<Snapshot> {
        &self.buf[0]
    }

    /// Newer snapshot of the current bracket; the view renders its rows.
    pub fn cur(&self) -> &Arc<Snapshot> {
        self.buf.get(1).unwrap_or(&self.buf[0])
    }

    /// Advance the render clock by elapsed ticks (`delta * speed * TICK_RATE`)
    /// and return the alpha between [`Self::prev`] and [`Self::cur`].
    /// Alpha may exceed 1.0: extrapolation past the newest snapshot.
    pub fn advance(&mut self, ticks: f64) -> f32 {
        let newest = self.buf.back().unwrap().tick as f64;
        let oldest = self.buf.front().unwrap().tick as f64;
        // Never assume more than one unseen tick (stall, pause).
        self.sim_now = (self.sim_now + ticks).min(newest + Self::MAX_EXTRAP);
        let err = self.sim_now - self.render_tick;
        if err > Self::SNAP {
            self.render_tick += err; // hopelessly behind (stall, fast-forward)
        } else if err.abs() > Self::DEADBAND {
            self.render_tick += ticks * (1.0 + Self::SLEW * err.signum());
        } else {
            self.render_tick += ticks;
        }
        self.render_tick = self.render_tick.clamp(oldest, newest + Self::MAX_EXTRAP);
        // Trim to the bracket, but keep two snapshots: extrapolation past
        // `newest` needs the last pair as its motion segment.
        while self.buf.len() > 2 && (self.buf[1].tick as f64) <= self.render_tick {
            self.buf.pop_front();
        }
        let prev_tick = self.buf[0].tick as f64;
        let span = self.cur().tick as f64 - prev_tick;
        if span > 0.0 {
            ((self.render_tick - prev_tick) / span) as f32
        } else {
            1.0
        }
    }
}

/// Display position for one unit. Alpha <= 1 lerps; beyond that only the
/// waypoint-ward component of the last displacement is extrapolated, capped
/// at the waypoint, so collision shoves aren't amplified and idle units
/// (waypoint = own position) stay put.
pub fn display_pos(prev: Vector2, cur: Vector2, waypoint: Vector2, alpha: f32) -> Vector2 {
    if alpha <= 1.0 {
        return prev.lerp(cur, alpha);
    }
    let extra = (cur - prev) * (alpha - 1.0);
    let to_wp = waypoint - cur;
    let len2 = to_wp.length_squared();
    if len2 <= f32::EPSILON {
        return cur;
    }
    let frac = (extra.dot(to_wp) / len2).clamp(0.0, 1.0);
    cur + to_wp * frac
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

    fn snap_at(tick: u64) -> Arc<Snapshot> {
        Arc::new(Snapshot {
            tick,
            ..Default::default()
        })
    }

    /// Continuous rendered time implied by a bracket + alpha.
    fn rendered(prev: &Snapshot, cur: &Snapshot, alpha: f32) -> f64 {
        prev.tick as f64 + alpha as f64 * (cur.tick - prev.tick) as f64
    }

    #[test]
    fn test_interpolator_steady_state_is_smooth() {
        // 2 frames per tick: rendered time advances every frame, no pin/jump.
        let mut it = Interpolator::new(snap_at(0));
        let mut last = 0.0;
        for tick in 1..100 {
            it.push(snap_at(tick));
            for _ in 0..2 {
                let alpha = it.advance(0.5);
                let t = rendered(it.prev(), it.cur(), alpha);
                assert!(t > last, "rendered time stalled: {last} -> {t}");
                assert!(t <= tick as f64 + 1.0, "extrapolated more than one tick");
                last = t;
            }
        }
        assert!(it.buf.len() <= 2, "buffer must stay trimmed");
    }

    #[test]
    fn test_interpolator_low_speed_tracks_sim_time() {
        // 8 frames per tick (sim speed 0.25): rendered time must stay smooth
        // and track estimated sim time closely — display latency must not
        // grow to a full (long) tick like the delay-based scheme.
        let mut it = Interpolator::new(snap_at(0));
        let mut last = 0.0;
        for tick in 1..100 {
            it.push(snap_at(tick));
            for _ in 0..8 {
                let alpha = it.advance(0.125);
                let t = rendered(it.prev(), it.cur(), alpha);
                assert!(t > last, "rendered time stalled: {last} -> {t}");
                last = t;
                if tick > 30 {
                    let lag = tick as f64 - t;
                    assert!(lag < 0.5, "latency too high at tick {tick}: {lag}");
                }
            }
        }
    }

    #[test]
    fn test_interpolator_absorbs_clock_drift() {
        // Frame clock 2% fast vs snapshot cadence: the rate nudge must keep
        // rendered time strictly advancing (no pin at the newest snapshot,
        // which was the old stutter at sim speeds < 1).
        let mut it = Interpolator::new(snap_at(0));
        let mut last = 0.0;
        for tick in 1..200 {
            it.push(snap_at(tick));
            for _ in 0..2 {
                let alpha = it.advance(0.51);
                let t = rendered(it.prev(), it.cur(), alpha);
                assert!(t > last, "rendered time stalled: {last} -> {t}");
                last = t;
            }
        }
    }

    #[test]
    fn test_interpolator_recovers_from_stall() {
        let mut it = Interpolator::new(snap_at(0));
        it.push(snap_at(1));
        // Sim stalls: render clock runs one extrapolated tick past the
        // newest snapshot, then holds.
        let mut last = 0.0;
        for _ in 0..10 {
            let alpha = it.advance(0.5);
            last = rendered(it.prev(), it.cur(), alpha);
        }
        assert!(
            (last - 2.0).abs() < 1e-9,
            "must hold one tick past newest: {last}"
        );
        // Snapshots resume: rendered time moves forward again, never back.
        for tick in 2..10 {
            it.push(snap_at(tick));
            for _ in 0..2 {
                let alpha = it.advance(0.5);
                let t = rendered(it.prev(), it.cur(), alpha);
                assert!(t > last, "rendered time stalled or reversed: {last} -> {t}");
                last = t;
            }
        }
        assert!(last > 1.0, "must resume after stall");
    }

    #[test]
    fn test_display_pos_extrapolation() {
        let wp = v(10.0, 0.0);
        // Below alpha 1: plain lerp.
        assert_eq!(display_pos(v(0.0, 0.0), v(2.0, 0.0), wp, 0.5), v(1.0, 0.0));
        // Straight waypoint-ward motion extrapolates linearly.
        assert_eq!(display_pos(v(0.0, 0.0), v(1.0, 0.0), wp, 1.5), v(1.5, 0.0));
        // Idle (waypoint = own position): never extrapolate.
        let cur = v(3.0, 4.0);
        assert_eq!(display_pos(v(2.0, 4.0), cur, cur, 2.0), cur);
        // Sideways collision shove: no waypoint-ward component, no extrapolation.
        assert_eq!(
            display_pos(v(0.0, 0.0), v(0.0, 1.0), v(10.0, 1.0), 2.0),
            v(0.0, 1.0)
        );
        // Backward displacement is dropped, not mirrored.
        assert_eq!(display_pos(v(1.0, 0.0), v(0.0, 0.0), wp, 2.0), v(0.0, 0.0));
        // Extrapolation is capped at the waypoint.
        assert_eq!(
            display_pos(v(0.0, 0.0), v(1.0, 0.0), v(1.2, 0.0), 2.0),
            v(1.2, 0.0)
        );
    }

    #[test]
    fn test_interpolator_brackets_across_tick_gaps() {
        // Snapshot gaps (fast-forward skips polls) lerp across the gap.
        let mut it = Interpolator::new(snap_at(10));
        it.push(snap_at(13));
        // Far behind sim time (13): snaps forward, brackets (10, 13).
        let alpha = it.advance(0.0);
        assert_eq!((it.prev().tick, it.cur().tick), (10, 13));
        assert!((rendered(it.prev(), it.cur(), alpha) - 13.0).abs() < 1e-6);
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
