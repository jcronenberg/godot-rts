use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use godot::prelude::*;

use rts_lib::abstraction::Abstraction;
use rts_lib::astar::{self, AStarScratch};
use rts_lib::delaunay::CDT;

fn lcg(state: &mut u64) -> u64 {
    *state = state.wrapping_mul(1103515245).wrapping_add(12345);
    *state
}

fn random_points(count: usize, seed: u64) -> Vec<Vector2> {
    let mut rng = seed;
    (0..count)
        .map(|_| {
            let x = (lcg(&mut rng) / 65536 % 1000) as f32;
            let y = (lcg(&mut rng) / 65536 % 1000) as f32;
            Vector2::new(x, y)
        })
        .collect()
}

fn grid_points(size: usize) -> Vec<Vector2> {
    let spacing = 10.0_f32;
    (0..size)
        .flat_map(|i| (0..size).map(move |j| Vector2::new(i as f32 * spacing, j as f32 * spacing)))
        .collect()
}

fn constrained_cdt(n: usize, seed: u64) -> CDT {
    let pts = random_points(n, seed);
    let mut cdt = CDT::from_points(pts);
    for i in (0..n.saturating_sub(1)).step_by(10) {
        cdt.insert_constraint(i as u32, (i + 1) as u32);
    }
    cdt.remove_super_triangle();
    cdt.build_grid_index();
    cdt.compute_widths();
    cdt
}

/// N evenly-spaced (start, goal) pairs drawn from face centroids.
/// Pairs are spaced half the face count apart for long expected paths.
fn make_pairs(cdt: &CDT, n: usize) -> Vec<(Vector2, Vector2)> {
    let nf = cdt.num_faces() as usize;
    (0..n)
        .map(|i| {
            let sf = (i * nf / n) as u32;
            let gf = ((i * nf / n + nf / 2) % nf) as u32;
            (cdt.face_centroid(sf), cdt.face_centroid(gf))
        })
        .collect()
}

// ── find_path (A* + funnel) ───────────────────────────────────────────────────

fn bench_astar_random(c: &mut Criterion) {
    let mut group = c.benchmark_group("astar/random");

    for &n in &[100usize, 500, 1_000, 5_000, 10_000] {
        let cdt = CDT::triangulate(random_points(n, 0xDEAD_BEEF));
        let start = cdt.face_centroid(0);
        let goal = cdt.face_centroid(cdt.num_faces() - 1);
        let mut sc = AStarScratch::new();

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| {
                astar::find_path(
                    black_box(&cdt),
                    black_box(start),
                    black_box(goal),
                    &mut sc,
                    0.0,
                )
            });
        });
    }
    group.finish();
}

fn bench_astar_grid(c: &mut Criterion) {
    let mut group = c.benchmark_group("astar/grid");

    for &side in &[10usize, 20, 30, 50] {
        let cdt = CDT::triangulate(grid_points(side));
        let far = (side as f32 - 1.0) * 10.0;
        let start = Vector2::new(0.5, 0.5);
        let goal = Vector2::new(far - 0.5, far - 0.5);
        let n = side * side;
        let mut sc = AStarScratch::new();

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| {
                astar::find_path(
                    black_box(&cdt),
                    black_box(start),
                    black_box(goal),
                    &mut sc,
                    0.0,
                )
            });
        });
    }
    group.finish();
}

fn bench_astar_radius(c: &mut Criterion) {
    let mut group = c.benchmark_group("astar/radius");

    for &n in &[200usize, 500, 1_000] {
        let cdt = constrained_cdt(n, 0x1234_5678);
        let start = cdt.face_centroid(0);
        let goal = cdt.face_centroid(cdt.num_faces() - 1);
        let mut sc = AStarScratch::new();

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| {
                astar::find_path(
                    black_box(&cdt),
                    black_box(start),
                    black_box(goal),
                    &mut sc,
                    5.0,
                )
            });
        });
    }
    group.finish();
}

fn bench_single_query(c: &mut Criterion) {
    let mut group = c.benchmark_group("astar/single_query");

    for &n in &[500usize, 1_000, 5_000, 10_000] {
        let cdt = CDT::triangulate(random_points(n, 0xDEAD_BEEF));
        let start = cdt.face_centroid(0);
        let goal = cdt.face_centroid(cdt.num_faces() - 1);
        let mut sc = AStarScratch::new();

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| {
                astar::find_path(
                    black_box(&cdt),
                    black_box(start),
                    black_box(goal),
                    &mut sc,
                    0.0,
                )
            });
        });
    }
    group.finish();
}

fn bench_multi_agent(c: &mut Criterion) {
    const AGENTS: usize = 16;
    let mut group = c.benchmark_group("astar/multi_agent_16");

    for &n in &[500usize, 1_000, 5_000, 10_000] {
        let cdt = CDT::triangulate(random_points(n, 0xDEAD_BEEF));
        let pairs = make_pairs(&cdt, AGENTS);
        let mut scratches: Vec<AStarScratch> = (0..AGENTS).map(|_| AStarScratch::new()).collect();

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| {
                for (i, &(start, goal)) in pairs.iter().enumerate() {
                    black_box(astar::find_path(
                        black_box(&cdt),
                        start,
                        goal,
                        &mut scratches[i],
                        0.0,
                    ));
                }
            });
        });
    }
    group.finish();
}

// ── find_path_abstract (TRA* query) ──────────────────────────────────────────

/// Steady-state TRA* query cost: build the abstraction once, then run a batch
/// of queries reusing a single scratch (mirrors the Godot wrapper, which keeps
/// one persistent scratch on the object).  Exercises the per-query allocations
/// in `find_local_l3` / `tra_star` / `reconstruct_channel`.
fn bench_tra_star_query(c: &mut Criterion) {
    const QUERIES: usize = 16;
    let mut group = c.benchmark_group("astar/tra_star_query_16");

    for &n in &[200usize, 500, 1_000] {
        let cdt = constrained_cdt(n, 0x1234_5678);
        let abs = Abstraction::build(&cdt);
        let pairs = make_pairs(&cdt, QUERIES);
        let mut sc = AStarScratch::new();

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| {
                for &(start, goal) in &pairs {
                    black_box(astar::find_path_abstract(
                        black_box(&cdt),
                        black_box(&abs),
                        start,
                        goal,
                        &mut sc,
                        5.0,
                    ));
                }
            });
        });
    }
    group.finish();
}

// ── Abstraction build (one-time cost) ────────────────────────────────────────

fn bench_abstraction_build(c: &mut Criterion) {
    let mut group = c.benchmark_group("abstraction/build");

    for &n in &[200usize, 500, 1_000, 5_000] {
        let cdt = constrained_cdt(n, 0xABCD_1234);

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| Abstraction::build(black_box(&cdt)));
        });
    }
    group.finish();
}

// ── portal_max_radius microbenchmark ─────────────────────────────────────────

fn bench_portal_max_radius(c: &mut Criterion) {
    let mut group = c.benchmark_group("abstraction/portal_max_radius");

    for &n in &[200usize, 1_000, 5_000] {
        let cdt = constrained_cdt(n, 0xBEEF_CAFE);
        let nf = cdt.num_faces();

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            b.iter(|| {
                let mut acc = 0.0f32;
                for f in 0..nf {
                    for j in 0..3u32 {
                        let he = f * 3 + j;
                        acc += cdt.portal_max_radius(black_box(he));
                    }
                }
                black_box(acc)
            });
        });
    }
    group.finish();
}

// ── neighbor iteration ────────────────────────────────────────────────────────

fn bench_neighbor_iteration(c: &mut Criterion) {
    let mut group = c.benchmark_group("astar/neighbor_iteration");
    let cdt = CDT::triangulate(random_points(1_000, 0xCAFE_F00D));

    group.bench_function("for_each_neighbor", |b| {
        b.iter(|| {
            let mut acc = 0u32;
            for f in 0..cdt.num_faces() {
                cdt.for_each_neighbor(black_box(f), |nb, _he| acc = acc.wrapping_add(nb));
            }
            black_box(acc)
        });
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_astar_random,
    bench_astar_grid,
    bench_astar_radius,
    bench_abstraction_build,
    bench_portal_max_radius,
    bench_single_query,
    bench_multi_agent,
    bench_tra_star_query,
    bench_neighbor_iteration,
);
criterion_main!(benches);
