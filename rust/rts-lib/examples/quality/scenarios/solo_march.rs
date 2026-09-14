//! The control: one unit, one long route, nothing in its way. Every other
//! scenario adds a crowd, a wall or a fight on top. If this row moves, the
//! change touched pathing itself rather than how units negotiate each other.
//!
//! It is also where `suboptimality` has real room today. `rooms_map` centres
//! every door, making a door-to-door staircase *exactly* as long as the
//! straight diagonal (`1900 * sqrt(2)`), which is what the reference finds.
//! `find_path` runs down one column of doors and along one row instead, for
//! ~39% more: the bounded-refinement limit its own docs warn about, with a
//! number on it.
//!
//! `detour` sitting on top of `suboptimality` is the tell that the unit walks
//! its plan faithfully; the two coming apart would mean the opposite.

use godot::prelude::Vector2;
use rts_lib::astar::AStarScratch;
use rts_lib::mapgen::rooms_map;
use rts_lib::sim::{Command, Sim};

use crate::harness::{Ctx, metric as m, Reading, ScenarioSpec};
use crate::metrics::{self, Cfg, PathProbe, Query, Route, Run, build_cdt, wall_segments};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "solo_march",
    run,
};

const RADIUS: f32 = 5.0;
const SPEED: f32 = 40.0;
const START: Vector2 = Vector2::new(50.0, 50.0);
const GOAL: Vector2 = Vector2::new(1950.0, 1950.0);

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = rooms_map(20, 20);
    let walls = wall_segments(&points, &constraints);
    let field = super::field(ctx, &walls, RADIUS, GOAL);
    let optimal = field.optimal_len(START);

    // Path level: the polyline itself, before anyone walks it.
    let cdt = build_cdt(points.clone(), &constraints);
    let abs = rts_lib::abstraction::Abstraction::build(&cdt);
    let mut probe = PathProbe::default();
    probe.query(
        &cdt,
        Some(&abs),
        &mut AStarScratch::new(),
        &Query {
            start: START,
            goal: GOAL,
            radius: RADIUS,
            reference: optimal,
        },
    );

    // Agent level: what the unit does with it.
    let mut run = Run::new(
        Sim::new(points, &constraints, 0x50_10),
        Cfg {
            deadline_mult: 3.0,
            ..Cfg::default()
        },
    );
    run.sim
        .step(&[metrics::spawn_cmd(START, RADIUS, SPEED)]);
    let id = metrics::unit_ids(&run.sim)[0];
    run.sim.step(&[Command::Move {
        units: vec![id],
        goal: GOAL,
    }]);
    run.track(id, Route::to(&field, START));
    if ctx.trace {
        run.record_trace(SPEC.name);
    }

    // Budget: three ideal traversals plus a settling margin, so a unit that
    // is merely slow is scored as late rather than as never arriving.
    let budget = optimal
        .map(|o| (o / (SPEED * rts_lib::sim::DT)) as u64 * 3 + 200)
        .unwrap_or(2_000);
    for _ in 0..budget {
        run.step(&[]);
    }
    if ctx.trace {
        run.write_trace(&ctx.out_dir);
    }
    let s = run.stats();

    //                     name                unit        good   bad  weight
    vec![
        m("suboptimality",       "ratio",    1.02,   2.00, 2.0).or_bad(probe.suboptimality()),
        m("min_clearance",       "radii",    0.60,   0.20, 1.0).or_bad(probe.min_clearance()),
        m("abstraction_penalty", "ratio",    1.00,   1.30, 1.0).or_bad(probe.abstraction_penalty()),
        m("waypoints",           "count",    2.00, 200.00, 0.0).or_bad(probe.waypoints()),
        m("turn_per_100",        "rad",      0.00,   3.00, 0.0).or_bad(probe.turn_per_len()),
        m("arrival",             "frac",     1.00,   0.00, 2.0).at(s.arrival),
        m("residual",            "radii",    0.00,  20.00, 2.0).or_bad(s.residual),
        m("detour",              "ratio",    1.05,   2.00, 2.0).or_bad(s.detour),
        m("lateness_p50",        "ratio",    1.10,   2.50, 1.0).or_bad(s.lateness_p50),
        m("stall_trips",         "per unit", 0.00,  20.00, 1.0).at(s.stall_trips),
        m("jitter",              "rad/tick", 0.01,   0.40, 1.0).at(s.jitter),
    ]
}
