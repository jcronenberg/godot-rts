//! Forty attackers crossing open ground to converge on one defender, then the
//! steady-state crush. Same shape as `benches/sim.rs`'s `combat_blob`, scored
//! instead of timed, with two deliberate differences from the bench fixture.
//!
//! The bench spawns the block nine units off the defender at a 3-unit pitch,
//! because it is timing a settled crush and wants to reach one immediately.
//! Both of those poison a *quality* measurement. Starting on top of the target
//! skips the approach entirely, so nothing scores how a mob crosses ground and
//! picks its stations, which is most of what the player actually watches. And
//! a 3-unit pitch between radius-5 bodies means they spawn already
//! interpenetrating, so the overlap block would be reading a spawn artifact
//! rather than the sim's behaviour.
//!
//! So: a 12-unit pitch (clear of `2r`) and a block far enough away that the
//! nearest attacker walks about 12 seconds and the farthest about 21, on open
//! ground. Open, not through a doorway, on purpose: walls in the approach
//! would turn first-shot time into a measurement of queuing through a door,
//! which is what `door_funnel_200` is for.
//!
//! `combat_positioning_plan.md` is asking for exactly these numbers and has no
//! way to get them today: how much of the mob can actually shoot, how long the
//! back rank waits for its first shot, and whether units commit to a station
//! or shuffle between them.
//!
//! `in_range` has a hard geometric ceiling here, and the anchors respect it.
//! A melee attacker (`attack_range` 2, radius 5) reaches a radius-5 defender
//! from 12 units out; that ring is `2 * pi * 12` ≈ 75 long and each body takes
//! `2r` = 10 of it, so at most ~7 of the 40 can be inside reach at once, or
//! 0.175. The approach dilutes that further, since a unit walking toward the
//! fight already has a target and so counts in the denominator: with roughly
//! a third of the run spent closing, the practical ceiling is nearer 0.115.
//! The metric is still worth scoring, it just never approaches 1.
//!
//! `first_shot_p95` is anchored against the *travel* floor, not against zero.
//! The farthest attacker is 209 units out at speed 10, so nothing can put 95%
//! of the mob in action before about 630 ticks however good the positioning
//! is. Scoring against that floor makes the metric read as queuing delay
//! rather than as a restatement of how far away the block was spawned.

use godot::prelude::Vector2;
use rts_lib::sim::{Command, Sim};

use crate::harness::{Ctx, metric as m, Reading, ScenarioSpec};
use crate::maps::{box_map, v};
use crate::metrics::{Cfg, Run, unit_ids};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "combat_blob",
    run,
};

const RADIUS: f32 = 5.0;
const SPEED: f32 = 10.0;
const ATTACKERS: usize = 40;
/// Long enough that the approach is a minority of the run: roughly 350 ticks
/// for the front rank to make contact and 650 for the back, leaving the rest
/// as the steady-state crush the positioning metrics are really about.
const TICKS: u64 = 1_400;
const DEFENDER: Vector2 = Vector2::new(300.0, 200.0);
/// Block origin, 8 wide at a [`PITCH`] that clears `2 * RADIUS`.
const BLOCK: Vector2 = Vector2::new(100.0, 140.0);
const PITCH: f32 = 12.0;

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = box_map(400.0, 400.0);

    let mut run = Run::new(
        Sim::new(points, &constraints, 0xB1_0B),
        Cfg::default(),
    );
    // Defender first (slot 0), then the attackers in a block across the map.
    // Both sides carry effectively infinite health so the fight never decays
    // into an idle.
    let mut spawns = vec![Command::Spawn {
        pos: DEFENDER,
        radius: RADIUS,
        max_speed: SPEED,
        team: 1,
        max_health: f32::MAX,
        damage: 0.0,
        attack_range: 0.0,
        attack_cooldown_ticks: 1,
    }];
    spawns.extend((0..ATTACKERS).map(|k| Command::Spawn {
        pos: v(
            BLOCK.x + PITCH * (k % 8) as f32,
            BLOCK.y + PITCH * (k / 8) as f32,
        ),
        radius: RADIUS,
        max_speed: SPEED,
        team: 0,
        max_health: f32::MAX,
        damage: 5.0,
        attack_range: 2.0,
        attack_cooldown_ticks: 10,
    }));
    run.sim.step(&spawns);

    let ids = unit_ids(&run.sim);
    let (defender, attackers) = (ids[0], &ids[1..]);
    run.sim.step(&[Command::Attack {
        units: attackers.to_vec(),
        target: defender,
    }]);
    // Only the attackers are scored: the defender is scenery.
    for &id in attackers {
        run.track(id, None);
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

    //                          name              unit         good    bad  weight
    vec![
        m("in_range",       "frac",      0.115,    0.03, 3.0).or_bad(s.in_range),
        m("first_shot_p95", "ticks",   650.00, 1400.00, 2.0).or_bad(s.first_shot_p95),
        m("chase_gap",      "reaches",   0.00,    0.00, 0.0).or_bad(s.chase_gap),
        m("slot_churn",     "per unit",  0.00,   20.00, 2.0).at(s.slot_churn),
        m("hold_trips",     "per unit",  0.00,  10.00, 1.0).at(s.hold_trips),
        m("jitter",         "rad/tick",  0.10,   1.20, 1.0).at(s.jitter),
        m("push_ally",    "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
        m("push_enemy",   "radii/tick", 0.00, 0.00, 0.0).at(s.push_enemy),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Crush, &s))
    .collect()
}
