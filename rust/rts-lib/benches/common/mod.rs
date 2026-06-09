//! Helpers shared by the `astar` and `delaunay` bench targets.
#![allow(dead_code)] // not every target uses every helper

use std::collections::HashMap;

use godot::prelude::*;

use rts_lib::delaunay::CDT;

/// Standard bench constraint pattern: an edge between every 10th point index
/// and its successor. Shared so the isolated benches decompose `pipeline/load`.
pub fn insert_chain_constraints(cdt: &mut CDT, n: usize) {
    for i in (0..n.saturating_sub(1)).step_by(10) {
        cdt.insert_constraint(i as u32, (i + 1) as u32);
    }
}

/// A `cols x rows` grid of square rooms with a centred door gap in every
/// shared wall (solid outer boundary) — game-like mesh statistics that
/// uniform random points don't produce.
pub fn rooms_map(cols: usize, rows: usize) -> (Vec<Vector2>, Vec<(u32, u32)>) {
    const ROOM: f32 = 100.0;
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

/// Fully prepared CDT for a [`rooms_map`] level (constraints inserted, super
/// triangle removed, grid index and portal widths built).
pub fn rooms_cdt(cols: usize, rows: usize) -> CDT {
    let (points, constraints) = rooms_map(cols, rows);
    let mut cdt = CDT::from_points(points);
    for (a, b) in constraints {
        cdt.insert_constraint(a, b);
    }
    cdt.remove_super_triangle();
    cdt.build_grid_index();
    cdt.compute_widths();
    cdt
}
