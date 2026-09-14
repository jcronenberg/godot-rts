//! Kiting a retreating target. A defender walks a straight line slower than
//! its pursuers, which follow by alternating a batched `Attack` (commit,
//! stand, fire) with a batched `Move` onto its current position: trading
//! standing time for distance so a fleeing enemy neither escapes nor gets a
//! free ride.
//!
//! `fire_efficiency` is the metric that matters, the payoff the manoeuvre
//! buys, bounded by how much of the chase is spent inside reach. `in_range`
//! says the same about position rather than output; the two coming apart means
//! units standing in reach without shooting, or shooting off station.
//!
//! Commands go out every [`CYCLE`] ticks. Every tick (the `benches/sim.rs`
//! cadence) leaves the sim no two consecutive ticks to act in, pinning every
//! metric at its floor and reporting the fixture instead of the simulation.

use godot::prelude::Vector2;
use rts_lib::sim::{Command, Sim};

use crate::harness::{Ctx, metric as m, Reading, ScenarioSpec};
use crate::maps::{box_map, v};
use crate::metrics::{Cfg, Route, Run, unit_ids};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "stutter_step",
    run,
};

const RADIUS: f32 = 5.0;
/// Attacker speed against [`PREY_SPEED`]. Deliberately a thin margin: a large
/// one lets a unit that never stutter-steps keep up anyway.
const SPEED: f32 = 10.0;
const PREY_SPEED: f32 = 7.0;
const ATTACKERS: usize = 8;
/// Two body diameters, so the formation *could* shoot as a body rather than a
/// front rank. That it does not is an open positioning defect, and reach is
/// not the cause: 8 through 24 all leave the same two or three units firing.
const RANGE: f32 = 20.0;
const COOLDOWN: u32 = 10;
/// Ticks between clicks: six commands a second at 30 Hz.
const CYCLE: u64 = 5;
/// Sized to keep the defender walking for the whole run, not arriving early.
const TICKS: u64 = 1_300;
const PREY_START: Vector2 = Vector2::new(120.0, 150.0);
const PREY_GOAL: Vector2 = Vector2::new(420.0, 150.0);

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = box_map(600.0, 300.0);

    let mut run = Run::new(Sim::new(points, &constraints, 0x57_07), Cfg::default());
    // Defender first (slot 0): unarmed, effectively unkillable, and slower than
    // its pursuers, so the fight neither ends nor runs away.
    let mut spawns = vec![Command::Spawn {
        pos: PREY_START,
        radius: RADIUS,
        max_speed: PREY_SPEED,
        team: 1,
        max_health: 1.0e6,
        damage: 0.0,
        attack_range: 0.0,
        attack_cooldown_ticks: 1,
    }];
    // A 4x2 block behind it, front rank already inside reach (surface gap 7).
    // Eight bodies cannot all sit in reach behind one target, so this starts
    // as a real following formation. In contact on purpose: the scenario
    // scores whether contact is *held*, not closed from cold.
    spawns.extend((0..ATTACKERS).map(|i| Command::Spawn {
        pos: v(
            PREY_START.x - 52.0 + 12.0 * (i % 4) as f32,
            PREY_START.y - 6.0 + 12.0 * (i / 4) as f32,
        ),
        radius: RADIUS,
        max_speed: SPEED,
        team: 0,
        max_health: 1.0e6,
        damage: 5.0,
        attack_range: RANGE,
        attack_cooldown_ticks: COOLDOWN,
    }));
    run.sim.step(&spawns);

    let ids = unit_ids(&run.sim);
    let (prey, attackers) = (ids[0], ids[1..].to_vec());
    for &id in &attackers {
        run.track(id, Route::none());
    }
    // One order, never re-issued: the defender simply walks its line.
    run.sim.step(&[Command::Move {
        units: vec![prey],
        goal: PREY_GOAL,
    }]);
    if ctx.trace {
        run.record_trace(SPEC.name);
    }
    for k in 0..TICKS {
        let cmds = if !k.is_multiple_of(CYCLE) {
            Vec::new()
        } else if (k / CYCLE).is_multiple_of(2) {
            // Attack-click: stand and shoot.
            vec![Command::Attack {
                units: attackers.clone(),
                target: prey,
            }]
        } else {
            // Step-click: close onto where the target is *now*, buying back
            // the ground the standing half of the cycle cost.
            let Some(at) = run.sim.units().get(prey).map(|u| u.pos) else {
                break;
            };
            vec![Command::Move {
                units: attackers.clone(),
                goal: at,
            }]
        };
        run.step(&cmds);
    }
    if ctx.trace {
        run.write_trace(&ctx.out_dir);
    }
    let s = run.stats();

    //                          name              unit         good     bad  weight
    vec![
        m("fire_efficiency", "frac",      0.45,    0.05, 3.0).or_bad(s.fire_efficiency),
        m("in_range",        "frac",      0.70,    0.10, 3.0).or_bad(s.in_range),
        m("chase_gap",       "reaches",   0.70,    2.00, 2.0).or_bad(s.chase_gap),
        m("slot_churn",      "per unit",  2.00,   40.00, 2.0).at(s.slot_churn),
        m("jitter",          "rad/tick",  0.10,    1.20, 2.0).at(s.jitter),
        m("hold_trips",      "per unit",  0.00,   20.00, 1.0).at(s.hold_trips),
        m("repaths",         "per unit",  0.00,    0.00, 0.0).at(s.repaths),
        m("push_ally",       "radii/tick", 0.00,   0.00, 0.0).at(s.push_ally),
        m("push_enemy",      "radii/tick", 0.00,   0.00, 0.0).at(s.push_enemy),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Open, &s))
    .collect()
}
