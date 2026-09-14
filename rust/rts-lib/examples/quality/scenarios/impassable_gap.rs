//! Forty radius-5 units ordered through a 6px slot they cannot fit. The gap
//! being impassable is the point; nothing here expects a unit to reach the far
//! side. Crowd pressure forcing a body through is a separate guarantee, held
//! by `sim::tests::test_crowd_never_squeezes_through_subdiameter_gap`.
//!
//! Two unrelated jobs behind the single score:
//!
//! **A standing watch on an open planner defect.** `phantoms` counts paths
//! returned through space the reference calls too tight, which is still open:
//! the binding constriction is mid-triangle, so no portal gates it.
//! `min_clearance` is the same defect from the other side. The radius sweep
//! straddles the fit threshold on purpose, so a change to the passability gate
//! shows up as a phantom or a refusal.
//!
//! **The hardest crowd pressure in the suite.** Forty bodies against geometry
//! that never yields, re-ordered every five ticks for a thousand ticks.
//!
//! Wall penetration is deliberately *not* scored: the crate's own test asserts
//! it on this geometry and fails the build, which beats a metric that scores
//! 100 forever.

use godot::prelude::Vector2;
use rts_lib::astar::AStarScratch;
use rts_lib::sim::{Command, Sim};

use crate::harness::{Ctx, metric as m, Reading, ScenarioSpec};
use crate::maps::{thin_wall, v};
use crate::metrics::{Cfg, PathProbe, Query, Route, Run, build_cdt, unit_ids, wall_segments};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "impassable_gap_40",
    run,
};

const RADIUS: f32 = 5.0;
const SPEED: f32 = 60.0;
const GAP: f32 = 6.0;
const UNITS: usize = 40;
const TICKS: u64 = 1_000;
const GOAL: Vector2 = Vector2::new(175.0, 100.0);

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = thin_wall(GAP);
    let walls = wall_segments(&points, &constraints);

    // Path level, swept over radius: 2 and 3 fit the slot (diameter 4 and 6
    // against a 6px gap), 5 does not.
    let cdt = build_cdt(points.clone(), &constraints);
    let abs = rts_lib::abstraction::Abstraction::build(&cdt);
    let mut probe = PathProbe::default();
    let mut scratch = AStarScratch::new();
    for radius in [2.0f32, 3.0, RADIUS] {
        let field = super::field(ctx, &walls, radius, GOAL);
        for start in [v(120.0, 20.0), v(120.0, 150.0)] {
            probe.query(
                &cdt,
                Some(&abs),
                &mut scratch,
                &Query {
                    start,
                    goal: GOAL,
                    radius,
                    reference: field.optimal_len(start),
                },
            );
        }
    }

    let mut run = Run::new(
        Sim::new(points, &constraints, 7),
        Cfg::default(),
    );
    // All 40 in one step, or the early ones settle before the pressure starts.
    run.sim.step(
        &(0..UNITS)
            .map(|i| {
                crate::metrics::spawn_cmd(
                    v(115.0 + 3.0 * (i % 8) as f32, 20.0 + 4.0 * (i / 8) as f32),
                    RADIUS,
                    SPEED,
                )
            })
            .collect::<Vec<_>>(),
    );
    let ids = unit_ids(&run.sim);
    for &id in &ids {
        run.track(id, Route::none()); // unreachable at this radius: nothing to price
    }
    if ctx.trace {
        run.record_trace(SPEC.name);
    }
    for t in 0..TICKS {
        // Re-click every fifth tick: the sustained pressure the report needed.
        let cmds = if t % 5 == 0 {
            vec![Command::Move {
                units: ids.clone(),
                goal: GOAL,
            }]
        } else {
            Vec::new()
        };
        run.step(&cmds);
    }
    if ctx.trace {
        run.write_trace(&ctx.out_dir);
    }
    let s = run.stats();

    //                          name              unit         good    bad  weight
    vec![
        m("phantoms",        "count",      0.00,   2.00, 3.0).at(probe.phantoms()),
        m("refusals",        "count",      0.00,   2.00, 1.0).at(probe.refusals()),
        // No `clearance_p5`: six queries make it the minimum by construction,
        // and two columns of one measurement read as corroboration.
        m("min_clearance",   "radii",      0.50,   0.05, 2.0).or_bad(probe.min_clearance()),
        m("stall_trips",     "per unit",   0.00,   0.00, 0.0).at(s.stall_trips),
        m("travel",          "radii",      0.00,   0.00, 0.0).at(s.travelled / RADIUS as f64),
        m("push_ally",    "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Funnel, &s))
    .collect()
}
