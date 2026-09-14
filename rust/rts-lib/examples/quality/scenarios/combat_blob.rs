//! Forty attackers crossing open ground to converge on one defender, then the
//! steady-state crush. `benches/sim.rs`'s `combat_blob` scored instead of
//! timed, with two changes: a 12-unit pitch (clear of `2r`, so overlap reads
//! the sim and not a spawn artifact) and a block far enough out that the
//! nearest attacker walks ~12s and the farthest ~21. Open ground on purpose,
//! since walls would make first-shot time a measure of queuing through a door,
//! which is `door_funnel_200`'s job.
//!
//! `in_range` has a hard geometric ceiling here and the anchors respect it. A
//! melee attacker reaches the defender from 12 units out; that ring is ~75
//! long and each body takes `2r` = 10 of it, so at most ~7 of 40 fit at once
//! (0.175). Closing units already have a target and count in the denominator,
//! putting the practical ceiling nearer 0.115. Worth scoring, never near 1.
//!
//! `first_shot_p95` is anchored against the *travel* floor, not zero: the
//! farthest attacker is 209 units out at speed 10, so nothing beats ~630
//! ticks. That makes it read as queuing delay rather than as a restatement of
//! how far away the block spawned.

use godot::prelude::Vector2;
use rts_lib::sim::{Command, Sim};

use crate::harness::{Ctx, Reading, ScenarioSpec, metric as m};
use crate::maps::{box_map, v};
use crate::metrics::{Cfg, Route, Run, unit_ids};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "combat_blob",
    run,
};

const RADIUS: f32 = 5.0;
const SPEED: f32 = 10.0;
const ATTACKERS: usize = 40;
/// Long enough that the approach (~350 ticks for the front rank, 650 for the
/// back) is a minority of the run, the rest being the steady-state crush.
const TICKS: u64 = 1_400;
const DEFENDER: Vector2 = Vector2::new(300.0, 200.0);
/// Block origin, 8 wide at a [`PITCH`] that clears `2 * RADIUS`.
const BLOCK: Vector2 = Vector2::new(100.0, 140.0);
const PITCH: f32 = 12.0;

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = box_map(400.0, 400.0);

    let mut run = Run::new(Sim::new(points, &constraints, 0xB1_0B), Cfg::default());
    // Defender first (slot 0), then the attacker block. Both sides carry
    // effectively infinite health, so the fight never decays into an idle.
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
        run.track(id, Route::none());
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
        m("in_range", "frac", 0.115, 0.03, 3.0).or_bad(s.in_range),
        m("first_shot_p95", "ticks", 650.00, 1400.00, 2.0).or_bad(s.first_shot_p95),
        m("chase_gap", "reaches", 0.00, 0.00, 0.0).or_bad(s.chase_gap),
        m("slot_churn", "per unit", 0.00, 20.00, 2.0).at(s.slot_churn),
        m("hold_trips", "per unit", 0.00, 10.00, 1.0).at(s.hold_trips),
        m("jitter", "rad/tick", 0.10, 1.20, 1.0).at(s.jitter),
        m("push_ally", "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
        m("push_enemy", "radii/tick", 0.00, 0.00, 0.0).at(s.push_enemy),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Crush, &s))
    .collect()
}
