use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use godot::prelude::*;

use rts_lib::delaunay::CDT;

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

    for size in [10, 50, 100, 200, 500, 1000].iter() {
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

    for size in [5, 10, 15, 20, 30].iter() {
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

    for size in [10, 50, 100, 200, 500].iter() {
        let points = generate_circle_points(*size);

        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, _| {
            b.iter(|| {
                let _ = CDT::triangulate(black_box(points.clone()));
            });
        });
    }

    group.finish();
}

fn bench_worst_case(c: &mut Criterion) {
    let mut group = c.benchmark_group("worst_case");

    for size in [20, 50, 100, 200].iter() {
        let points = generate_circle_points(*size);

        group.bench_with_input(BenchmarkId::from_parameter(size), size, |b, _| {
            b.iter(|| {
                let _ = CDT::triangulate(black_box(points.clone()));
            });
        });
    }

    group.finish();
}

fn bench_incremental_addition(c: &mut Criterion) {
    let mut group = c.benchmark_group("incremental_addition");

    let all_points = generate_random_points(100, 54321);

    group.bench_function("batch_100", |b| {
        b.iter(|| {
            let _ = CDT::triangulate(black_box(all_points.clone()));
        });
    });

    group.finish();
}

fn bench_point_distributions(c: &mut Criterion) {
    let mut group = c.benchmark_group("point_distributions");
    let size = 100;

    group.bench_function("random", |b| {
        let points = generate_random_points(size, 11111);
        b.iter(|| {
            let _ = CDT::triangulate(black_box(points.clone()));
        });
    });

    group.bench_function("grid", |b| {
        let points = generate_grid_points(10);
        b.iter(|| {
            let _ = CDT::triangulate(black_box(points.clone()));
        });
    });

    group.bench_function("circle", |b| {
        let points = generate_circle_points(size);
        b.iter(|| {
            let _ = CDT::triangulate(black_box(points.clone()));
        });
    });

    group.bench_function("clustered", |b| {
        let mut points = Vec::new();
        let centers = [
            Vector2::new(100.0, 100.0),
            Vector2::new(300.0, 100.0),
            Vector2::new(500.0, 300.0),
            Vector2::new(200.0, 400.0),
            Vector2::new(400.0, 400.0),
        ];

        for center in centers.iter() {
            let cluster = generate_random_points(20, center.x as u64);
            for p in cluster {
                points.push(Vector2::new(
                    center.x + (p.x - 500.0) * 0.2,
                    center.y + (p.y - 500.0) * 0.2,
                ));
            }
        }

        b.iter(|| {
            let _ = CDT::triangulate(black_box(points.clone()));
        });
    });

    group.finish();
}

fn bench_memory_patterns(c: &mut Criterion) {
    let mut group = c.benchmark_group("memory_patterns");

    group.bench_function("small_preallocated", |b| {
        let points = generate_random_points(50, 77777);
        b.iter(|| {
            let _ = CDT::triangulate(black_box(points.clone()));
        });
    });

    group.bench_function("large_preallocated", |b| {
        let points = generate_random_points(500, 88888);
        b.iter(|| {
            let _ = CDT::triangulate(black_box(points.clone()));
        });
    });

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

criterion_group!(
    benches,
    bench_random_points,
    bench_grid_points,
    bench_circle_points,
    bench_worst_case,
    bench_incremental_addition,
    bench_point_distributions,
    bench_memory_patterns,
    bench_constrained,
);

criterion_main!(benches);
