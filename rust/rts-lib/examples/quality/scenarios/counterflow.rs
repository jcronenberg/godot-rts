//! Two groups swapping ends through one corridor. Head-on traffic is where
//! arrival and lateness come apart from throughput: everyone can still get
//! there while taking four times as long to do it.

use godot::prelude::Vector2;
use rts_lib::sim::{Command, Sim};

use crate::harness::{Ctx, metric as m, Reading, ScenarioSpec};
use crate::maps::{corridor, v};
use crate::metrics::{Cfg, Run, unit_ids, wall_segments};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "counterflow_2x60",
    run,
};

const RADIUS: f32 = 5.0;
const SPEED: f32 = 40.0;
const WIDTH: f32 = 60.0;
const PER_SIDE: usize = 60;
const TICKS: u64 = 3_000;
const LEFT_GOAL: Vector2 = Vector2::new(600.0, 150.0);
const RIGHT_GOAL: Vector2 = Vector2::new(100.0, 150.0);

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = corridor(WIDTH);
    let walls = wall_segments(&points, &constraints);
    let to_right = super::field(ctx, &walls, RADIUS, LEFT_GOAL);
    let to_left = super::field(ctx, &walls, RADIUS, RIGHT_GOAL);

    let mut run = Run::new(
        Sim::new(points, &constraints, 0xC0_FF),
        Cfg {
            deadline_mult: 6.0,
            // No choke: both directions cross it, so a net rate would cancel out.
            choke: None,
        },
    );
    let mut spawns = super::spawn_block(v(40.0, 60.0), 6, 15.0, PER_SIDE, RADIUS, SPEED);
    spawns.extend(super::spawn_block(
        v(585.0, 60.0),
        6,
        15.0,
        PER_SIDE,
        RADIUS,
        SPEED,
    ));
    run.sim.step(&spawns);
    let ids = unit_ids(&run.sim);
    let (left, right) = ids.split_at(PER_SIDE);
    // Two commands, so the two crowds are two cohesion groups.
    run.sim.step(&[
        Command::Move {
            units: left.to_vec(),
            goal: LEFT_GOAL,
        },
        Command::Move {
            units: right.to_vec(),
            goal: RIGHT_GOAL,
        },
    ]);
    for (group, goal, field) in [
        (left, LEFT_GOAL, &to_right),
        (right, RIGHT_GOAL, &to_left),
    ] {
        for &id in group {
            let start = run.sim.units().get(id).map(|u| u.pos).unwrap_or(goal);
            run.track(id, field.optimal_len(start));
        }
    }
    if ctx.trace {
        run.record_trace(SPEC.name);
    }
    for _ in 0..TICKS {
        run.step(&[]);
    }
    if ctx.trace {
        run.write_trace(&ctx.out_dir);
    }
    let s = run.stats();

    //                          name             unit        good    bad  weight
    vec![
        m("arrival",           "frac",      1.00,  0.00, 3.0).at(s.arrival),
        m("lateness_p50",      "ratio",     1.05,  3.00, 1.0).or_bad(s.lateness_p50),
        m("lateness_p95",      "ratio",     1.20,  6.00, 2.0).or_bad(s.lateness_p95),
        m("detour",            "ratio",     1.05,  1.80, 2.0).or_bad(s.detour),
        m("stall_trips",       "per unit",  0.00, 10.00, 2.0).at(s.stall_trips),
        m("ally_stall_trips",  "per unit",  0.00, 10.00, 1.0).at(s.ally_stall_trips),
        m("repaths",           "per unit",  1.00, 20.00, 1.0).at(s.repaths),
        m("jitter",            "rad/tick",  0.05,  0.60, 1.0).at(s.jitter),
        m("cohesion",          "radii",     0.00,  0.00, 0.0).or_bad(s.cohesion),
        m("push_ally",    "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Open, &s))
    .collect()
}
