use criterion::{BatchSize, BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use godot::prelude::*;

use rts_lib::abstraction::Abstraction;
use rts_lib::astar::{self, AStarScratch};
use rts_lib::delaunay::CDT;
use rts_lib::navmesh::DynamicNavmesh;

mod common;

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

fn constrained_cdt(n: usize, seed: u64) -> CDT {
    let pts = random_points(n, seed);
    let mut cdt = CDT::from_points(pts);
    common::insert_chain_constraints(&mut cdt, n);
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

/// Plain A* with a nonzero radius — the path `find_path_tra` falls back to
/// when no abstraction is built.
fn bench_astar_radius(c: &mut Criterion) {
    let mut group = c.benchmark_group("astar/radius");

    for &n in &[200usize, 500, 1_000, 5_000] {
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

// ── find_path_abstract (TRA* query) ──────────────────────────────────────────

/// Steady-state TRA* query cost: one abstraction build, then a batch of
/// queries reusing a single scratch (mirrors the Godot wrapper's persistent
/// scratch).
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

/// TRA* queries on a rooms-and-corridors map (game-like mesh, every doorway
/// a choke point). Parameterised by room count.
fn bench_tra_star_query_rooms(c: &mut Criterion) {
    const QUERIES: usize = 16;
    let mut group = c.benchmark_group("astar/tra_star_query_rooms_16");

    for &side in &[4usize, 8, 16] {
        let cdt = common::rooms_cdt(side, side);
        let abs = Abstraction::build(&cdt);
        let pairs = make_pairs(&cdt, QUERIES);
        let mut sc = AStarScratch::new();

        group.bench_with_input(BenchmarkId::from_parameter(side * side), &side, |b, _| {
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

// ── full level-load pipeline ──────────────────────────────────────────────────

/// End-to-end level load cost via the same entry points the Godot wrapper
/// uses: `DynamicNavmesh::new` plus `Abstraction::build`.
fn bench_load_pipeline(c: &mut Criterion) {
    let mut group = c.benchmark_group("pipeline/load");

    for &n in &[200usize, 1_000, 5_000] {
        let pts = random_points(n, 0x10AD_5EED);
        let constraints = common::chain_constraints(n);

        group.bench_with_input(BenchmarkId::from_parameter(n), &n, |b, _| {
            // Clone in setup so only the pipeline is timed.
            b.iter_batched(
                || pts.clone(),
                |pts| {
                    let nav = DynamicNavmesh::new(pts, &constraints);
                    let abs = Abstraction::build(nav.navmesh());
                    black_box((nav, abs));
                },
                BatchSize::SmallInput,
            );
        });
    }

    group.finish();
}

/// `bench_load_pipeline` on a rooms-and-corridors map instead of random
/// points. Parameterised by room count.
fn bench_load_pipeline_rooms(c: &mut Criterion) {
    let mut group = c.benchmark_group("pipeline/load_rooms");

    for &side in &[4usize, 8, 16] {
        let (pts, constraints) = common::rooms_map(side, side);

        group.bench_with_input(BenchmarkId::from_parameter(side * side), &side, |b, _| {
            b.iter_batched(
                || pts.clone(),
                |pts| {
                    let nav = DynamicNavmesh::new(pts, &constraints);
                    let abs = Abstraction::build(nav.navmesh());
                    black_box((nav, abs));
                },
                BatchSize::SmallInput,
            );
        });
    }

    group.finish();
}

criterion_group!(
    benches,
    bench_astar_radius,
    bench_abstraction_build,
    bench_tra_star_query,
    bench_tra_star_query_rooms,
    bench_load_pipeline,
    bench_load_pipeline_rooms,
);
criterion_main!(benches);
