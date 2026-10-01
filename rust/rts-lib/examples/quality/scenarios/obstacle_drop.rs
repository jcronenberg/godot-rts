//! A building lands in front of a marching group, mid-run.
//!
//! The navmesh rebuild repaths every moving unit at once, so a regression in
//! repath cost or in group re-forming shows up here, as does a group that
//! splits to pass the building on both sides. The reference optimum is
//! taken against *post-drop* geometry, so `detour` reads high by the distance
//! walked before the drop, consistently run to run.

use godot::prelude::Vector2;
use rts_lib::sim::{Command, Sim};

use crate::harness::{Ctx, Reading, ScenarioSpec, metric as m};
use crate::maps::{box_map, polygon_walls, rect_obstacle, v};
use crate::metrics::{Cfg, Route, Run, unit_ids, wall_segments};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "obstacle_drop",
    variants: super::VARIANTS,
    run,
};

const RADIUS: f32 = 5.0;
const SPEED: f32 = 40.0;
const UNITS: usize = 30;
const DROP_TICK: u64 = 150;
const TICKS: u64 = 2_500;
const GOAL: Vector2 = Vector2::new(740.0, 200.0);

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = box_map(800.0, 400.0);
    let building = rect_obstacle(380.0, 60.0, 460.0, 340.0);
    let mut walls = wall_segments(&points, &constraints);
    walls.extend(polygon_walls(&building));
    let field = super::field(ctx, &walls, RADIUS, GOAL);

    let mut run = Run::new(
        Sim::new(points, &constraints, 0x0B_5D),
        Cfg {
            deadline_mult: 5.0,
            ..Cfg::default()
        },
    );
    // The building lands a little earlier or later, catching the group at a
    // different point in its march.
    let drop_tick = (DROP_TICK as f32 + ctx.offset(2, Vector2::splat(15.0)).x).round() as u64;
    run.sim.step(&super::spawn_block(
        ctx,
        v(40.0, 130.0) + ctx.offset(1, Vector2::splat(10.0)),
        5,
        14.0,
        UNITS,
        RADIUS,
        SPEED,
    ));
    let ids = unit_ids(&run.sim);
    run.sim.step(&[Command::Move {
        units: ids.clone(),
        goal: GOAL,
    }]);
    for &id in &ids {
        let start = run.sim.units().get(id).map(|u| u.pos).unwrap_or(GOAL);
        run.track(id, Route::to(&field, start));
    }
    if ctx.trace {
        run.record_trace(SPEC.name);
    }
    for t in 0..TICKS {
        if t == drop_tick {
            run.step(&[Command::AddObstacle {
                points: building.clone(),
            }]);
        } else {
            run.step(&[]);
        }
    }
    if ctx.trace {
        run.write_trace(&ctx.out_dir);
    }
    let s = run.stats();

    //                     name             unit        good    bad  weight
    vec![
        m("detour", "ratio", 1.05, 1.80, 3.0).or_bad(s.detour),
        m("arrival", "frac", 1.00, 0.00, 2.0).at(s.arrival),
        m("residual", "radii", 4.00, 60.00, 2.0).or_bad(s.residual),
        m("residual_p95", "radii", 6.00, 80.00, 1.0).or_bad(s.residual_p95),
        m("centroid_offset", "radii", 0.00, 12.00, 2.0).or_bad(s.centroid_offset),
        m("settle_p95", "frac run", 0.20, 1.00, 1.0).or_bad(s.settle_p95),
        m("lateness_p95", "ratio", 1.30, 6.00, 1.0).or_bad(s.lateness_p95),
        m("stall_trips", "per unit", 0.00, 10.00, 2.0).at(s.stall_trips),
        // One is the floor (the drop's rebuild); the march order is issued
        // before tracking starts.
        m("repaths", "per unit", 2.00, 20.00, 2.0).at(s.repaths),
        m("jitter", "rad/tick", 0.05, 0.50, 1.0).at(s.jitter),
        // A marching column reads ~1.9; a crowd that splits round both sides
        // of the building reads 3 and up, and re-merges before the end.
        m("spread_p95", "radii", 2.00, 3.50, 2.0).or_bad(s.spread_p95),
        m("spread_at_end", "radii", 0.00, 0.00, 0.0).or_bad(s.cohesion_final),
        m("push_ally", "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Open, &s))
    .collect()
}
