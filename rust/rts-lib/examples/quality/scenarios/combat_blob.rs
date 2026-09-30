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
//! how far away the block spawned. With every body immortal only the ring
//! (~6) ever fires, so it sits on the bad anchor.
//!
//! `retreat` is how far attackers walk *away* from the defender while still
//! closing on it, per attacker. Going round a crowd is fine; steering so far
//! round it that the unit walks back out reads as backing off the fight.
//!
//! `defender_drift` must stay 0: enemies may block but never push.

use std::f32::consts::TAU;

use godot::prelude::Vector2;
use rts_lib::sim::{Command, Sim, UnitId};

use crate::harness::{Ctx, Reading, ScenarioSpec, metric as m};
use crate::maps::{box_map, v};
use crate::metrics::{Cfg, Route, Run, unit_ids};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "combat_blob",
    variants: super::VARIANTS,
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
/// Widest bearing gap in the ring that still counts as surrounded.
const SURROUND_GAP: f32 = TAU / 4.0;

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = box_map(400.0, 400.0);

    let mut run = Run::new(Sim::new(points, &constraints, 0xB1_0B), Cfg::default());
    // The defender shifted a little, so the block meets it off-centre.
    let defender_at = DEFENDER + ctx.offset(1, Vector2::splat(8.0));
    // Defender first (slot 0), then the attacker block. Both sides carry
    // effectively infinite health, so the fight never decays into an idle.
    let mut spawns = vec![Command::Spawn {
        pos: defender_at,
        radius: RADIUS,
        max_speed: SPEED,
        team: 1,
        max_health: f32::MAX,
        damage: 0.0,
        attack_range: 0.0,
        attack_cooldown_ticks: 1,
    }];
    spawns.extend((0..ATTACKERS).map(|k| {
        let at = v(
            BLOCK.x + PITCH * (k % 8) as f32,
            BLOCK.y + PITCH * (k / 8) as f32,
        );
        Command::Spawn {
            pos: at + super::cell_jitter(ctx, at, PITCH, RADIUS),
            radius: RADIUS,
            max_speed: SPEED,
            team: 0,
            max_health: f32::MAX,
            damage: 5.0,
            attack_range: 2.0,
            attack_cooldown_ticks: 10,
        }
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
    let (mut contact, mut surrounded) = (None, None);
    let (mut lean_sum, mut lean_n) = (0.0f64, 0u32);
    let mut retreat = 0.0f64;
    let mut before = vec![None; attackers.len()];
    for k in 0..TICKS {
        for (b, &id) in before.iter_mut().zip(attackers) {
            *b = closing_dist(&run.sim, defender, id);
        }
        run.step(&[]);
        for (&b, &id) in before.iter().zip(attackers) {
            if let (Some(b), Some(a)) = (b, closing_dist(&run.sim, defender, id)) {
                retreat += (a - b).max(0.0) as f64;
            }
        }
        if k >= TICKS / 2 {
            lean_sum += lean(&run.sim, defender, attackers) as f64;
            lean_n += 1;
        }
        if surrounded.is_none()
            && let Some(widest) = widest_gap(&run.sim, defender, attackers)
        {
            contact.get_or_insert(run.sim.tick());
            if widest <= SURROUND_GAP {
                surrounded = Some(run.sim.tick());
            }
        }
    }
    if ctx.trace {
        run.write_trace(&ctx.out_dir);
    }
    let s = run.stats();
    // Never closing scores as the whole run.
    let surround = match (contact, surrounded) {
        (Some(a), Some(b)) => (b - a) as f64,
        _ => TICKS as f64,
    };
    let drift = run
        .sim
        .units()
        .get(defender)
        .map_or(f64::INFINITY, |u| (u.pos - defender_at).length() as f64);

    //                          name              unit         good    bad  weight
    vec![
        m("in_range", "frac", 0.115, 0.03, 3.0).or_bad(s.in_range),
        m("first_shot_p95", "ticks", 650.00, 1400.00, 2.0).or_bad(s.first_shot_p95),
        m("chase_gap", "reaches", 0.00, 0.00, 0.0).or_bad(s.chase_gap),
        m("side_churn", "per unit", 0.00, 20.00, 2.0).at(s.side_churn),
        m("give_ups", "per unit", 0.00, 10.00, 1.0).at(s.give_ups),
        m("jitter", "rad/tick", 0.10, 1.20, 1.0).at(s.jitter),
        m("push_ally", "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
        m("push_enemy", "radii/tick", 0.00, 0.00, 0.0).at(s.push_enemy),
        m("defender_drift", "units", 0.00, 10.00, 2.0).at(drift),
        m("surround", "ticks", 150.00, 700.00, 2.0).at(surround),
        m("lean", "radii", 0.00, 4.00, 2.0).at(lean_sum / lean_n.max(1) as f64),
        m("retreat", "radii/unit", 0.00, 10.00, 2.0)
            .at(retreat / (attackers.len() as f64 * RADIUS as f64)),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Crush, &s))
    .collect()
}

/// How far `id` is from `defender`, while it is still closing on it.
fn closing_dist(sim: &Sim, defender: UnitId, id: UnitId) -> Option<f32> {
    let (u, d) = (sim.units().get(id)?, sim.units().get(defender)?);
    (!u.engaged && !u.waiting).then(|| (u.pos - d.pos).length())
}

/// Widest bearing gap between attackers in reach of `defender`, if any.
fn widest_gap(sim: &Sim, defender: UnitId, attackers: &[UnitId]) -> Option<f32> {
    let d = sim.units().get(defender)?;
    let mut bearings: Vec<f32> = attackers
        .iter()
        .filter_map(|&id| sim.units().get(id))
        .filter(|u| (u.pos - d.pos).length() - u.radius - d.radius <= u.attack_range)
        .map(|u| {
            let o = u.pos - d.pos;
            o.y.atan2(o.x)
        })
        .collect();
    bearings.sort_by(f32::total_cmp);
    let wrap = bearings.first()? + TAU - bearings.last()?;
    Some(
        bearings
            .windows(2)
            .map(|w| w[1] - w[0])
            .fold(wrap, f32::max),
    )
}

/// Attacker centroid's offset from the defender, in radii: 0 for an even wrap.
fn lean(sim: &Sim, defender: UnitId, attackers: &[UnitId]) -> f32 {
    let Some(d) = sim.units().get(defender) else {
        return 0.0;
    };
    let (mut sum, mut n) = (Vector2::ZERO, 0);
    for u in attackers.iter().filter_map(|&id| sim.units().get(id)) {
        sum += u.pos - d.pos;
        n += 1;
    }
    if n == 0 {
        return 0.0;
    }
    (sum / n as f32).length() / RADIUS
}
