use criterion::{BatchSize, BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use godot::prelude::*;

use rts_lib::abstraction::Abstraction;
use rts_lib::delaunay::CDT;
use rts_lib::navmesh::{DynamicNavmesh, Obstacle};

mod common;

fn generate_random_points(count: usize, seed: u64) -> Vec<Vector2> {
    let mut rng = seed;
    let mut points = Vec::with_capacity(count);

    for _ in 0..count {
        rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
        let x = ((rng / 65536) % 1000) as f32;
        rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
        let y = ((rng / 65536) % 1000) as f32;
        points.push(Vector2::new(x, y));
    }

    points
}

fn generate_grid_points(size: usize) -> Vec<Vector2> {
    let mut points = Vec::with_capacity(size * size);
    let spacing = 10.0;

    for i in 0..size {
        for j in 0..size {
            points.push(Vector2::new(i as f32 * spacing, j as f32 * spacing));
        }
    }

    points
}

fn generate_circle_points(count: usize) -> Vec<Vector2> {
    let mut points = Vec::with_capacity(count);
    let radius = 100.0;

    for i in 0..count {
        let angle = (i as f32 / count as f32) * std::f32::consts::TAU;
        points.push(Vector2::new(
            radius * angle.cos() + 500.0,
            radius * angle.sin() + 500.0,
        ));
    }

    points
}

fn bench_random_points(c: &mut Criterion) {
    let mut group = c.benchmark_group("random_points");

    for size in [100, 200, 500, 1000, 10000].iter() {
        let points = generate_random_points(*size, 12345);

        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, _| {
            b.iter(|| {
                let _ = CDT::triangulate(black_box(points.clone()));
            });
        });
    }

    group.finish();
}

fn bench_grid_points(c: &mut Criterion) {
    let mut group = c.benchmark_group("grid_points");

    for size in [10, 15, 20, 30].iter() {
        let points = generate_grid_points(*size);
        let total = size * size;

        group.bench_with_input(BenchmarkId::from_parameter(total), &total, |b, _| {
            b.iter(|| {
                let _ = CDT::triangulate(black_box(points.clone()));
            });
        });
    }

    group.finish();
}

fn bench_circle_points(c: &mut Criterion) {
    let mut group = c.benchmark_group("circle_points");

    for size in [100, 200, 500].iter() {
        let points = generate_circle_points(*size);

        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, _| {
            b.iter(|| {
                let _ = CDT::triangulate(black_box(points.clone()));
            });
        });
    }

    group.finish();
}

fn bench_clustered_points(c: &mut Criterion) {
    let mut group = c.benchmark_group("clustered_points");

    let centers = [
        Vector2::new(100.0, 100.0),
        Vector2::new(300.0, 100.0),
        Vector2::new(500.0, 300.0),
        Vector2::new(200.0, 400.0),
        Vector2::new(400.0, 400.0),
    ];

    let points: Vec<Vector2> = centers
        .iter()
        .flat_map(|center| {
            generate_random_points(20, center.x as u64)
                .into_iter()
                .map(|p| {
                    Vector2::new(
                        center.x + (p.x - 500.0) * 0.2,
                        center.y + (p.y - 500.0) * 0.2,
                    )
                })
        })
        .collect();

    group.bench_function("5_clusters_20pts", |b| {
        b.iter(|| {
            let _ = CDT::triangulate(black_box(points.clone()));
        });
    });

    group.finish();
}

/// Query locate_face for random points on a pre-built CDT.
/// Construction cost is excluded — measures only point location.
fn bench_locate_face(c: &mut Criterion) {
    let mut group = c.benchmark_group("locate_face");

    for &size in &[100usize, 500, 1000, 10000] {
        let points = generate_random_points(size, 42);
        let cdt = CDT::triangulate(points);

        // Fixed query set (seed 99) reused across all CDT sizes so comparisons are apples-to-apples.
        // generate_random_points always produces points in [0,1000)x[0,1000) regardless of count,
        // so queries fall within the same domain as the CDT at every size.
        let query_points = generate_random_points(1000, 99);

        group.bench_with_input(BenchmarkId::from_parameter(size), &size, |b, _| {
            b.iter(|| {
                let mut found = 0u32;
                for &pt in &query_points {
                    if cdt.locate_face(black_box(pt)).is_some() {
                        found = found.wrapping_add(1);
                    }
                }
                black_box(found)
            });
        });
    }

    group.finish();
}

/// Traverse every face's neighbors on a pre-built CDT.
/// Construction cost is excluded — measures only graph traversal.
fn bench_graph_traversal(c: &mut Criterion) {
    let mut group = c.benchmark_group("graph_traversal");

    for &size in &[100usize, 500, 1000, 10000] {
        let points = generate_random_points(size, 42424);
        let cdt = CDT::triangulate(points);

        group.bench_with_input(BenchmarkId::from_parameter(size), &size, |b, _| {
            b.iter(|| {
                let mut acc = 0u32;
                for f in 0..cdt.num_faces() {
                    cdt.for_each_neighbor(black_box(f), |n, _| acc = acc.wrapping_add(n));
                }
                black_box(acc)
            });
        });
    }

    group.finish();
}

fn bench_constrained(c: &mut Criterion) {
    let mut group = c.benchmark_group("constrained");

    group.bench_function("grid_9_with_diagonal", |b| {
        let points = generate_grid_points(3); // 9 points
        b.iter(|| {
            let mut cdt = CDT::from_points(black_box(points.clone()));
            cdt.insert_constraint(0, 8);
        });
    });

    group.bench_function("random_100_with_constraints", |b| {
        let points = generate_random_points(100, 99999);
        b.iter(|| {
            let mut cdt = CDT::from_points(black_box(points.clone()));
            cdt.insert_constraint(0, 50);
            cdt.insert_constraint(25, 75);
        });
    });

    group.finish();
}

/// Random points with a constraint every 10th index, prepared for width queries
/// (super-triangle removed, grid built) but *without* `compute_widths`.
fn prepared_constrained(n: usize, seed: u64) -> CDT {
    let mut cdt = CDT::from_points(generate_random_points(n, seed));
    common::insert_chain_constraints(&mut cdt, n);
    cdt.remove_super_triangle();
    cdt.build_grid_index();
    cdt
}

/// Per-portal passability precomputation — a one-time load cost (a 32-step
/// binary search per half-edge) that the pathfinders depend on but no other
/// bench measures.
fn bench_compute_widths(c: &mut Criterion) {
    let mut group = c.benchmark_group("compute_widths");

    for &n in &[200usize, 1_000, 5_000] {
        let mut cdt = prepared_constrained(n, 0x00C0_FFEE);

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| cdt.compute_widths());
        });
    }

    group.finish();
}

/// Cost of inserting a batch of constraints (the edge-flip work), isolated from
/// the initial triangulation via `iter_batched` so only the insertions are timed.
fn bench_insert_constraints(c: &mut Criterion) {
    let mut group = c.benchmark_group("insert_constraints");

    for &n in &[200usize, 1_000, 5_000] {
        let points = generate_random_points(n, 0x0BAD_BEEF);

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter_batched(
                || CDT::from_points(points.clone()),
                |mut cdt| {
                    common::insert_chain_constraints(&mut cdt, n);
                    cdt
                },
                BatchSize::SmallInput,
            );
        });
    }

    group.finish();
}

// ---------------------------------------------------------------------------
// Dynamic obstacle rebuild benches
// ---------------------------------------------------------------------------

struct Lcg(u64);
impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.0 >> 33
    }
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * (self.next() % 10001) as f32 / 10000.0
    }
}

/// Random rect in a quadrant of a [`common::rooms_map`] room. Each slot owns
/// one 50x50 quadrant, so obstacles never overlap or touch walls.
fn slot_rect(cols: usize, slot: usize, rng: &mut Lcg) -> Obstacle {
    let room = slot / 4;
    let q = slot % 4;
    let qx = (room % cols) as f32 * 100.0 + (q % 2) as f32 * 50.0;
    let qy = (room / cols) as f32 * 100.0 + (q / 2) as f32 * 50.0;
    let x0 = rng.range(qx + 8.0, qx + 25.0);
    let y0 = rng.range(qy + 8.0, qy + 25.0);
    let w = rng.range(5.0, 17.0);
    let h = rng.range(5.0, 17.0);
    Obstacle::polygon(vec![
        Vector2::new(x0, y0),
        Vector2::new(x0 + w, y0),
        Vector2::new(x0 + w, y0 + h),
        Vector2::new(x0, y0 + h),
    ])
}

/// (map label, room grid size, obstacle counts) — k is capped at 4 per room.
const OBSTACLE_SCENARIOS: &[(&str, usize, &[usize])] = &[
    ("10x10", 10, &[10, 100]),
    ("20x20", 20, &[10, 100, 500, 1000]),
    ("40x40", 40, &[10, 100, 500, 1000]),
];

fn obstacle_set(cols: usize, k: usize) -> Vec<Obstacle> {
    let mut rng = Lcg(0x0B57_AC1E ^ (cols as u64) << 32 ^ k as u64);
    (0..k).map(|s| slot_rect(cols, s, &mut rng)).collect()
}

/// Full declarative rebuild with k live obstacles: clone base → insert points
/// and constraints → super removal → grid index → widths. One iteration is
/// also the cost of a single obstacle change while k obstacles are live (the
/// `delta` signal from the plan): removal is "not re-inserting", so any change
/// triggers exactly this rebuild.
fn bench_obstacle_rebuild(c: &mut Criterion) {
    let mut group = c.benchmark_group("obstacles");
    group.sample_size(10);

    for &(label, cols, ks) in OBSTACLE_SCENARIOS {
        let (points, constraints) = common::rooms_map(cols, cols);
        for &k in ks {
            let mut nav = DynamicNavmesh::new(points.clone(), &constraints);
            let obstacles = obstacle_set(cols, k);
            let mut ids: Vec<_> = obstacles
                .iter()
                .map(|o| nav.add_obstacle(o.clone()))
                .collect();
            nav.rebuild();

            group.bench_with_input(
                BenchmarkId::new(format!("rebuild/{label}"), k),
                &k,
                |b, _| {
                    b.iter(|| {
                        // Swap one obstacle out and back in: marks dirty, keeps k live.
                        nav.remove_obstacle(ids[0]);
                        ids[0] = nav.add_obstacle(obstacles[0].clone());
                        black_box(nav.rebuild().num_faces())
                    });
                },
            );
        }
    }

    group.finish();
}

/// Rebuild plus abstraction rebuild — the end-to-end cost when TRA* is in use.
fn bench_obstacle_rebuild_with_abstraction(c: &mut Criterion) {
    let mut group = c.benchmark_group("obstacles");
    group.sample_size(10);

    for &(label, cols, ks) in OBSTACLE_SCENARIOS {
        let (points, constraints) = common::rooms_map(cols, cols);
        for &k in ks {
            let mut nav = DynamicNavmesh::new(points.clone(), &constraints);
            let obstacles = obstacle_set(cols, k);
            let mut ids: Vec<_> = obstacles
                .iter()
                .map(|o| nav.add_obstacle(o.clone()))
                .collect();
            nav.rebuild();

            group.bench_with_input(
                BenchmarkId::new(format!("rebuild_with_abstraction/{label}"), k),
                &k,
                |b, _| {
                    b.iter(|| {
                        nav.remove_obstacle(ids[0]);
                        ids[0] = nav.add_obstacle(obstacles[0].clone());
                        let cdt = nav.rebuild();
                        black_box(Abstraction::build(cdt))
                    });
                },
            );
        }
    }

    group.finish();
}

/// Stage breakdown of the rebuild pipeline (representative 20x20 map) to
/// direct the optimization pass. Each stage's setup replays the prior stages
/// untimed; obstacle geometry is replicated via the same public CDT calls
/// `DynamicNavmesh::rebuild` uses.
fn bench_obstacle_stages(c: &mut Criterion) {
    let mut group = c.benchmark_group("obstacles");
    group.sample_size(10);

    let cols = 20;
    let (points, constraints) = common::rooms_map(cols, cols);
    let mut base = CDT::from_points(points);
    for &(a, b) in &constraints {
        base.insert_constraint(a, b);
    }

    let insert_points = |cdt: &mut CDT, obstacles: &[Obstacle]| -> Vec<Vec<u32>> {
        let mut hint = 0;
        obstacles
            .iter()
            .map(|obs| {
                obs.points
                    .iter()
                    .map(|&p| {
                        let v = cdt.insert_point(p, hint);
                        hint = cdt.face_of_vertex(v);
                        v
                    })
                    .collect()
            })
            .collect()
    };
    let insert_constraints = |cdt: &mut CDT, obstacles: &[Obstacle], ids: &[Vec<u32>]| {
        for (obs, vs) in obstacles.iter().zip(ids) {
            for &(a, b) in &obs.edges {
                cdt.insert_constraint(vs[a as usize], vs[b as usize]);
            }
        }
    };

    for &k in &[100usize, 1000] {
        let obstacles = obstacle_set(cols, k);

        group.bench_with_input(BenchmarkId::new("stage/clone/20x20", k), &k, |b, _| {
            b.iter(|| black_box(base.clone()));
        });

        group.bench_with_input(
            BenchmarkId::new("stage/insert_points/20x20", k),
            &k,
            |b, _| {
                b.iter_batched(
                    || base.clone(),
                    |mut cdt| {
                        black_box(insert_points(&mut cdt, &obstacles));
                        cdt
                    },
                    BatchSize::SmallInput,
                );
            },
        );

        group.bench_with_input(
            BenchmarkId::new("stage/insert_constraints/20x20", k),
            &k,
            |b, _| {
                b.iter_batched(
                    || {
                        let mut cdt = base.clone();
                        let ids = insert_points(&mut cdt, &obstacles);
                        (cdt, ids)
                    },
                    |(mut cdt, ids)| {
                        insert_constraints(&mut cdt, &obstacles, &ids);
                        cdt
                    },
                    BatchSize::SmallInput,
                );
            },
        );

        let constrained = || {
            let mut cdt = base.clone();
            let ids = insert_points(&mut cdt, &obstacles);
            insert_constraints(&mut cdt, &obstacles, &ids);
            cdt
        };

        group.bench_with_input(
            BenchmarkId::new("stage/remove_super/20x20", k),
            &k,
            |b, _| {
                b.iter_batched(
                    constrained,
                    |mut cdt| {
                        cdt.remove_super_triangle();
                        cdt
                    },
                    BatchSize::SmallInput,
                );
            },
        );

        let mut removed = constrained();
        removed.remove_super_triangle();

        group.bench_with_input(BenchmarkId::new("stage/grid/20x20", k), &k, |b, _| {
            b.iter(|| removed.build_grid_index());
        });

        removed.build_grid_index();
        group.bench_with_input(BenchmarkId::new("stage/widths/20x20", k), &k, |b, _| {
            b.iter(|| removed.compute_widths());
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_random_points,
    bench_grid_points,
    bench_circle_points,
    bench_clustered_points,
    bench_constrained,
    bench_locate_face,
    bench_graph_traversal,
    bench_compute_widths,
    bench_insert_constraints,
    bench_obstacle_rebuild,
    bench_obstacle_rebuild_with_abstraction,
    bench_obstacle_stages,
);

criterion_main!(benches);
