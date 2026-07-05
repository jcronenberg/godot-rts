use std::time::{Duration, Instant};

use criterion::{BenchmarkId, Criterion, criterion_group, criterion_main};
use godot::prelude::*;

use rts_lib::mapgen::{ROOM_SIZE, rooms_map};
use rts_lib::navmesh::ObstacleId;
use rts_lib::sim::{Command, Sim, UnitId};

const SIDE: usize = 20;
const RADIUS: f32 = 5.0;
const SPEED: f32 = 10.0;

/// `n` units spread over the rooms map, each en route to the opposite corner
/// of the map (long paths through doorways), warmed past first-tick allocs.
fn marching_sim(n: usize) -> Sim {
    let (points, constraints) = rooms_map(SIDE, SIDE);
    let mut sim = Sim::new(points, &constraints, 0xBEEF);
    let spawns: Vec<Command> = (0..n)
        .map(|i| {
            let (rx, ry) = (i % SIDE, (i / SIDE) % SIDE);
            let k = (i / (SIDE * SIDE)) as f32;
            Command::Spawn {
                pos: Vector2::new(
                    (rx as f32 + 0.3) * ROOM_SIZE + 11.0 * k,
                    (ry as f32 + 0.3) * ROOM_SIZE + 7.0 * k,
                ),
                radius: RADIUS,
                speed: SPEED,
            }
        })
        .collect();
    sim.step(&spawns);
    let moves: Vec<Command> = unit_ids(&sim)
        .into_iter()
        .enumerate()
        .map(|(i, id)| {
            let (rx, ry) = (i % SIDE, (i / SIDE) % SIDE);
            Command::Move {
                units: vec![id],
                goal: Vector2::new(
                    (SIDE - 1 - rx) as f32 * ROOM_SIZE + 50.0,
                    (SIDE - 1 - ry) as f32 * ROOM_SIZE + 50.0,
                ),
            }
        })
        .collect();
    sim.step(&moves);
    for _ in 0..4 {
        sim.step(&[]);
    }
    sim
}

fn unit_ids(sim: &Sim) -> Vec<UnitId> {
    sim.units().iter().map(|(id, _)| id).collect()
}

/// Time `iters` steps (command construction excluded) in chunks, rebuilding
/// a fresh sim between chunks so long benches don't drift out of steady
/// state; `chunk` stays below each scenario's drift horizon.
fn chunked<F>(iters: u64, chunk: u64, mut make: impl FnMut() -> Sim, mut commands: F) -> Duration
where
    F: FnMut(u64, &Sim) -> Vec<Command>,
{
    let mut total = Duration::ZERO;
    let mut done = 0;
    while done < iters {
        let mut sim = make();
        let chunk = chunk.min(iters - done);
        for k in 0..chunk {
            let cmds = commands(k, &sim);
            let start = Instant::now();
            sim.step(&cmds);
            total += start.elapsed();
        }
        std::hint::black_box(sim.state_hash());
        done += chunk;
    }
    total
}

/// Steady-state tick: integration + separation + wall clamp, no commands.
fn bench_step(c: &mut Criterion) {
    let mut group = c.benchmark_group("sim/step");
    group.sample_size(20);
    for &n in &[100usize, 500, 1_000, 2_000] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_custom(|iters| chunked(iters, 1024, || marching_sim(n), |_, _| Vec::new()));
        });
    }
    group.finish();
}

/// Steady state plus a burst of ~50 move commands every tick (APM spike).
fn bench_step_burst(c: &mut Criterion) {
    let mut group = c.benchmark_group("sim/step_burst_50");
    group.sample_size(10);
    for &n in &[500usize, 2_000] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_custom(|iters| {
                chunked(
                    iters,
                    1024,
                    || marching_sim(n),
                    |k, sim| {
                        let ids = unit_ids(sim);
                        (0..50u64)
                            .map(|j| {
                                let i = ((k * 50 + j) % ids.len() as u64) as usize;
                                let goal = if (k + j) % 2 == 0 {
                                    Vector2::new(50.0, 50.0)
                                } else {
                                    Vector2::new(
                                        SIDE as f32 * ROOM_SIZE - 50.0,
                                        SIDE as f32 * ROOM_SIZE - 50.0,
                                    )
                                };
                                Command::Move {
                                    units: vec![ids[i]],
                                    goal,
                                }
                            })
                            .collect()
                    },
                )
            });
        });
    }
    group.finish();
}

/// One `Move` over the whole selection every tick — clustering,
/// one-funnel-per-flock, per-unit corner offsets — vs `step_burst`'s
/// per-unit moves.
fn bench_group_move(c: &mut Criterion) {
    let mut group = c.benchmark_group("sim/group_move");
    group.sample_size(10);
    for &n in &[500usize, 2_000] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_custom(|iters| {
                chunked(
                    iters,
                    256,
                    || marching_sim(n),
                    |k, sim| {
                        let goal = if k % 2 == 0 {
                            Vector2::new(50.0, 50.0)
                        } else {
                            Vector2::new(
                                SIDE as f32 * ROOM_SIZE - 50.0,
                                SIDE as f32 * ROOM_SIZE - 50.0,
                            )
                        };
                        vec![Command::Move {
                            units: unit_ids(sim),
                            goal,
                        }]
                    },
                )
            });
        });
    }
    group.finish();
}

/// One obstacle change per tick: navmesh rebuild + abstraction refresh +
/// repath-all. Decides whether repath-all needs staggering.
fn bench_step_repath(c: &mut Criterion) {
    let mut group = c.benchmark_group("sim/step_repath");
    group.sample_size(10);
    for &n in &[100usize, 500, 1_000, 2_000] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_custom(|iters| {
                chunked(
                    iters,
                    1024,
                    || marching_sim(n),
                    |k, _| {
                        // Alternate add/remove of a rect in a room interior;
                        // ids are sequential, so remove targets the last add.
                        let cmd = if k % 2 == 0 {
                            let (x, y) = (
                                (k / 2 % SIDE as u64) as f32 * ROOM_SIZE + 40.0,
                                (k / 2 / SIDE as u64 % SIDE as u64) as f32 * ROOM_SIZE + 40.0,
                            );
                            Command::AddObstacle {
                                points: vec![
                                    Vector2::new(x, y),
                                    Vector2::new(x + 15.0, y),
                                    Vector2::new(x + 15.0, y + 15.0),
                                    Vector2::new(x, y + 15.0),
                                ],
                            }
                        } else {
                            Command::RemoveObstacle {
                                id: ObstacleId::from_raw(k / 2),
                            }
                        };
                        vec![cmd]
                    },
                )
            });
        });
    }
    group.finish();
}

/// Dense clumps: separation + wall clamp dominated workload. Short chunks —
/// clumps disperse, so long chunks would decay into the plain step bench.
fn bench_separation(c: &mut Criterion) {
    let mut group = c.benchmark_group("sim/separation");
    group.sample_size(20);
    for &n in &[100usize, 500, 1_000, 2_000] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_custom(|iters| {
                let make = || {
                    let (points, constraints) = rooms_map(SIDE, SIDE);
                    let mut sim = Sim::new(points, &constraints, 0xC10D);
                    // ~60 units per room centre — heavily overlapping clumps.
                    let spawns: Vec<Command> = (0..n)
                        .map(|i| {
                            let room = i / 60;
                            let (rx, ry) = (room % SIDE, room / SIDE % SIDE);
                            let j = (i % 60) as f32;
                            Command::Spawn {
                                pos: Vector2::new(
                                    (rx as f32 + 0.5) * ROOM_SIZE + 0.13 * j,
                                    (ry as f32 + 0.5) * ROOM_SIZE + 0.07 * j,
                                ),
                                radius: RADIUS,
                                speed: SPEED,
                            }
                        })
                        .collect();
                    sim.step(&spawns);
                    sim.step(&[]);
                    sim
                };
                chunked(iters, 64, make, |_, _| Vec::new())
            });
        });
    }
    group.finish();
}

/// Worst case for cohesion: a dense, same-group, *moving* clump, where the
/// flock broad-phase scans the most same-group neighbours per cell. Short
/// chunks — the clump streams toward its goal and thins out.
fn bench_flock(c: &mut Criterion) {
    let mut group = c.benchmark_group("sim/flock");
    group.sample_size(20);
    for &n in &[100usize, 500, 1_000, 2_000] {
        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, &n| {
            b.iter_custom(|iters| {
                let make = || {
                    let (points, constraints) = rooms_map(SIDE, SIDE);
                    let mut sim = Sim::new(points, &constraints, 0xF10C);
                    // ~60 units per room centre — heavily overlapping clumps.
                    let spawns: Vec<Command> = (0..n)
                        .map(|i| {
                            let room = i / 60;
                            let (rx, ry) = (room % SIDE, room / SIDE % SIDE);
                            let j = (i % 60) as f32;
                            Command::Spawn {
                                pos: Vector2::new(
                                    (rx as f32 + 0.5) * ROOM_SIZE + 0.13 * j,
                                    (ry as f32 + 0.5) * ROOM_SIZE + 0.07 * j,
                                ),
                                radius: RADIUS,
                                speed: SPEED,
                            }
                        })
                        .collect();
                    sim.step(&spawns);
                    // One Move groups every unit, so the whole clump coheres.
                    sim.step(&[Command::Move {
                        units: unit_ids(&sim),
                        goal: Vector2::new(50.0, 50.0),
                    }]);
                    sim.step(&[]);
                    sim
                };
                chunked(iters, 64, make, |_, _| Vec::new())
            });
        });
    }
    group.finish();
}

criterion_group!(
    benches,
    bench_step,
    bench_step_burst,
    bench_group_move,
    bench_step_repath,
    bench_separation,
    bench_flock,
);
criterion_main!(benches);
