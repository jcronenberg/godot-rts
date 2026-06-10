//! Helpers shared by the `astar` and `delaunay` bench targets.
#![allow(dead_code)] // not every target uses every helper

use rts_lib::delaunay::CDT;
pub use rts_lib::mapgen::rooms_map;

/// Standard bench constraint pattern: an edge between every 10th point index
/// and its successor. Shared so the isolated benches decompose `pipeline/load`.
pub fn insert_chain_constraints(cdt: &mut CDT, n: usize) {
    for i in (0..n.saturating_sub(1)).step_by(10) {
        cdt.insert_constraint(i as u32, (i + 1) as u32);
    }
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
