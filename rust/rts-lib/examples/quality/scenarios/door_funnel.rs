//! 200 units, one door. The number that decides whether a funnel *feels* good
//! is throughput, and nothing in `benches/` can see it.
//!
//! Separation and throughput trade directly against each other here: push
//! units apart harder and fewer get through the door per second. That is the
//! point of printing both raw columns next to the score.

use godot::prelude::Vector2;
use rts_lib::sim::{Command, Sim};

use crate::harness::{Ctx, metric as m, Reading, ScenarioSpec};
use crate::maps::{two_rooms, v};
use crate::metrics::{Cfg, Run, unit_ids, wall_segments};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "door_funnel_200",
    run,
};

const RADIUS: f32 = 5.0;
const SPEED: f32 = 40.0;
const DOOR: f32 = 30.0;
const UNITS: usize = 200;
const TICKS: u64 = 2_500;
const GOAL: Vector2 = Vector2::new(450.0, 200.0);

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = two_rooms(DOOR);
    let walls = wall_segments(&points, &constraints);
    let field = super::field(ctx, &walls, RADIUS, GOAL);

    let mut run = Run::new(
        Sim::new(points, &constraints, 0xD0_0B),
        Cfg {
            deadline_mult: 6.0, // 200 units through one door is a queue, not a march
            // Oriented so `side_of(a, b, GOAL) > 0`: crossings that leave the
            // start room count +1, the ones shoved back out count -1.
            choke: Some((v(300.0, 215.0), v(300.0, 185.0))),
        },
    );
    run.sim
        .step(&super::spawn_block(v(30.0, 30.0), 10, 13.0, UNITS, RADIUS, SPEED));
    let ids = unit_ids(&run.sim);
    // One command, so the whole crowd is one cohesion group.
    run.sim.step(&[Command::Move {
        units: ids.clone(),
        goal: GOAL,
    }]);
    for &id in &ids {
        let start = run.sim.units().get(id).map(|u| u.pos).unwrap_or(GOAL);
        run.track(id, field.optimal_len(start));
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

    //                     name              unit        good    bad  weight
    vec![
        // Over the window the door is in use, so it reads the door's rate and
        // not the tick budget's. High only *looks* free: a compressed crowd
        // beats the 3-abreast geometry and pays for it in the overlap rows.
        m("throughput",   "units/s",  36.00,  6.00, 3.0).at(s.throughput),
        m("arrival",      "frac",      1.00,  0.00, 2.0).at(s.arrival),
        m("lateness_p95", "ratio",     2.00, 12.00, 1.0).or_bad(s.lateness_p95),
        m("detour",       "ratio",     1.05,  1.80, 1.0).or_bad(s.detour),
        m("jitter",       "rad/tick",  0.05,  0.60, 2.0).at(s.jitter),
        m("stall_trips",  "per unit",  0.00, 20.00, 1.0).at(s.stall_trips),
        m("cohesion",     "radii",     0.00,  0.00, 0.0).or_bad(s.cohesion),
        m("spread_at_end","radii",     0.00,  0.00, 0.0).or_bad(s.cohesion_final),
        m("push_ally",    "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Funnel, &s))
    .collect()
}
