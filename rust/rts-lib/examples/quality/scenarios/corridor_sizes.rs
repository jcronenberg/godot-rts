//! `test_maps/test_unit_size_corridors.json`, swept over unit radius.
//!
//! Three gaps in one wall (50, 20 and 10 units wide) and seven radii from
//! comfortably through all of them to fitting none. Pure path level: no sim,
//! no crowd, just `find_path` against the reference's answer for the same
//! radius. This is where a passability-gate change shows up first.

use godot::prelude::Vector2;
use rts_lib::astar::AStarScratch;

use crate::harness::{Ctx, metric as m, Reading, ScenarioSpec};
use crate::maps::load_test_map;
use crate::metrics::{PathProbe, Query, build_cdt, wall_segments};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "corridor_sizes",
    run,
};

/// Query lanes across the wall column at x = 150..200. Three aim straight at
/// a gap's centre line, three cross diagonally (so the funnel has corners to
/// round), and two run back the other way. A big enough set that the 5th
/// percentile of clearance is a distribution and not just the minimum again.
const LANES: [(Vector2, Vector2); 8] = [
    (Vector2::new(75.0, 175.0), Vector2::new(325.0, 175.0)), // straight, 50-wide gap
    (Vector2::new(75.0, 310.0), Vector2::new(325.0, 310.0)), // straight, 20-wide gap
    (Vector2::new(75.0, 445.0), Vector2::new(325.0, 445.0)), // straight, 10-wide gap
    (Vector2::new(75.0, 60.0), Vector2::new(325.0, 500.0)),  // corner to corner
    (Vector2::new(75.0, 500.0), Vector2::new(325.0, 60.0)),  // and back the other way
    (Vector2::new(75.0, 260.0), Vector2::new(325.0, 400.0)), // between two gaps
    (Vector2::new(325.0, 175.0), Vector2::new(75.0, 445.0)), // right to left
    (Vector2::new(325.0, 500.0), Vector2::new(75.0, 175.0)), // right to left, long
];

/// 5 is the exact fit for the narrowest gap, 10 for the middle one, 25 for the
/// widest, and 30 fits none, so the sweep straddles every threshold the map
/// has. Nothing sits *on* a threshold the reference cannot resolve: its
/// free-space sampling has half a cell of slack, so a radius whose fit is
/// decided within a unit of the gap half-width would score the harness's
/// tolerance rather than the sim's gate.
const RADII: [f32; 7] = [2.0, 4.0, 5.0, 8.0, 10.0, 15.0, 30.0];

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = load_test_map("test_unit_size_corridors");
    let walls = wall_segments(&points, &constraints);
    let cdt = build_cdt(points, &constraints);
    let abs = rts_lib::abstraction::Abstraction::build(&cdt);
    let mut probe = PathProbe::default();
    let mut scratch = AStarScratch::new();

    for radius in RADII {
        for (start, goal) in LANES {
            let field = super::field(ctx, &walls, radius, goal);
            probe.query(
                &cdt,
                Some(&abs),
                &mut scratch,
                &Query {
                    start,
                    goal,
                    radius,
                    reference: field.optimal_len(start),
                },
            );
        }
    }
    //                               name                 unit      good    bad  weight
    vec![
        m("suboptimality",       "ratio", 1.02,   1.40, 3.0).or_bad(probe.suboptimality()),
        m("refusals",            "count", 0.00,   8.00, 3.0).at(probe.refusals()),
        m("phantoms",            "count", 0.00,   8.00, 3.0).at(probe.phantoms()),
        m("min_clearance",       "radii", 0.60,   0.20, 2.0).or_bad(probe.min_clearance()),
        m("clearance_p5",        "radii", 0.60,   0.20, 1.0).or_bad(probe.clearance_p5()),
        m("abstraction_penalty", "ratio", 1.00,   1.30, 1.0).or_bad(probe.abstraction_penalty()),
        m("waypoints",           "count", 2.00, 100.00, 0.0).or_bad(probe.waypoints()),
        m("turn_per_100",        "rad",   0.00,   3.00, 0.0).or_bad(probe.turn_per_len()),
    ]
}
