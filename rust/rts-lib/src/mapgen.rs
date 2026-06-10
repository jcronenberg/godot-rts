//! Synthetic map generators shared by tests and benches.

use std::collections::HashMap;

use godot::prelude::*;

/// A `cols x rows` grid of square rooms with a centred door gap in every
/// shared wall (solid outer boundary) — game-like mesh statistics that
/// uniform random points don't produce.
pub fn rooms_map(cols: usize, rows: usize) -> (Vec<Vector2>, Vec<(u32, u32)>) {
    const ROOM: f32 = ROOM_SIZE;
    const DOOR: f32 = 30.0;
    let jamb_lo = (ROOM - DOOR) / 2.0;
    let jamb_hi = (ROOM + DOOR) / 2.0;

    let mut points: Vec<Vector2> = Vec::new();
    let mut index: HashMap<(u32, u32), u32> = HashMap::new();
    let mut constraints: Vec<(u32, u32)> = Vec::new();
    let mut seg = |p: Vector2, q: Vector2| {
        let mut id = |v: Vector2| {
            *index
                .entry((v.x.to_bits(), v.y.to_bits()))
                .or_insert_with(|| {
                    points.push(v);
                    (points.len() - 1) as u32
                })
        };
        let (a, b) = (id(p), id(q));
        constraints.push((a, b));
    };

    for i in 0..=cols {
        let x = i as f32 * ROOM;
        for j in 0..rows {
            let y = j as f32 * ROOM;
            if i == 0 || i == cols {
                seg(Vector2::new(x, y), Vector2::new(x, y + ROOM));
            } else {
                seg(Vector2::new(x, y), Vector2::new(x, y + jamb_lo));
                seg(Vector2::new(x, y + jamb_hi), Vector2::new(x, y + ROOM));
            }
        }
    }
    for j in 0..=rows {
        let y = j as f32 * ROOM;
        for i in 0..cols {
            let x = i as f32 * ROOM;
            if j == 0 || j == rows {
                seg(Vector2::new(x, y), Vector2::new(x + ROOM, y));
            } else {
                seg(Vector2::new(x, y), Vector2::new(x + jamb_lo, y));
                seg(Vector2::new(x + jamb_hi, y), Vector2::new(x + ROOM, y));
            }
        }
    }

    (points, constraints)
}

/// Room side length used by [`rooms_map`].
pub const ROOM_SIZE: f32 = 100.0;

/// Deterministic LCG points in `[0, 1000)²`, matching the bench generator.
pub fn random_points(count: usize, seed: u64) -> Vec<Vector2> {
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
