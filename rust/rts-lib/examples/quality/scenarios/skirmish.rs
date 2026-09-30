//! Two mixed armies attack-move into each other across a cluttered field and
//! fight to the end: the engagement a player sees most, and the first scenario
//! here where anything dies. Deaths are the point. Targets die, holes open in
//! the line, attackers re-acquire and the crowd has to re-form around what is
//! left, none of which a fight between immortals ever exercises.
//!
//! Each army is two melee units to one ranged, spawned mixed the way a
//! box-selected army is, so the ranged units have to find their way behind
//! their own front line rather than starting there.
//!
//! The two sides are exact point mirrors through the map centre, spawn jitter
//! included, and differ only in which one the sim updates first; odd variants
//! swap that. A fair sim therefore draws, and `outcome_gap` is how far from a
//! draw it lands. Variants also shift both armies, mirrored.
//!
//! What it showed when it was written: a unit only notices an enemy within
//! `ACQUISITION_RANGE_MULT` weapon reaches, which for melee is about a body
//! width, so one in seven unit-ticks next to the fight has no target and the
//! odd fight breaks up unfinished (`bystanders`, `resolve`). And the rear
//! ranks crush into their own front line while the enemy holds it: the
//! overlap rows are almost entirely allied pairs.

use std::collections::BTreeMap;

use godot::prelude::Vector2;
use rts_lib::astar::clear_los;
use rts_lib::sim::{Command, Sim, Unit, UnitId};

use crate::harness::{Ctx, Reading, ScenarioSpec, metric as m};
use crate::maps::{battlefield, v};
use crate::metrics::{Cfg, Route, Run, unit_ids};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "skirmish_2x36",
    variants: super::VARIANTS,
    run,
};

const PER_SIDE: usize = 36;
/// Files across the army's front; ranks run back from it.
const FILES: usize = 6;
const PITCH: f32 = 12.0;
const TICKS: u64 = 2_400;
/// Centre of army A's front rank. Army B's is its mirror through [`CENTRE`].
const FRONT: Vector2 = Vector2::new(200.0, 300.0);
const CENTRE: Vector2 = Vector2::new(500.0, 300.0);
/// How near an enemy has to be, surface to surface and in sight, for a unit
/// to count as in the fight: about twelve body radii, well inside what a
/// player would expect a unit to notice.
const FIGHT_RADIUS: f32 = 60.0;

struct Kind {
    radius: f32,
    speed: f32,
    health: f32,
    damage: f32,
    range: f32,
    cooldown: u32,
}

const MELEE: Kind = Kind {
    radius: 5.0,
    speed: 45.0,
    health: 100.0,
    damage: 10.0,
    range: 2.0,
    cooldown: 15,
};

const RANGED: Kind = Kind {
    radius: 4.0,
    speed: 40.0,
    health: 60.0,
    damage: 8.0,
    range: 30.0,
    cooldown: 20,
};

fn kind(k: usize) -> &'static Kind {
    if k % 3 == 2 { &RANGED } else { &MELEE }
}

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = battlefield();
    let mut run = Run::new(Sim::new(points, &constraints, 0x5C_14), Cfg::default());

    let mirror = |p: Vector2| CENTRE * 2.0 - p;
    let front = FRONT + ctx.offset(1, Vector2::splat(15.0));
    let layout: Vec<(Vector2, &Kind)> = (0..PER_SIDE)
        .map(|k| {
            let at = front
                + v(
                    -PITCH * (k / FILES) as f32,
                    PITCH * ((k % FILES) as f32 - (FILES - 1) as f32 * 0.5),
                );
            (
                at + super::cell_jitter(ctx, at, PITCH, MELEE.radius),
                kind(k),
            )
        })
        .collect();
    let spawn = |team: u32, pos: Vector2, k: &Kind| Command::Spawn {
        pos,
        radius: k.radius,
        max_speed: k.speed,
        team,
        max_health: k.health,
        damage: k.damage,
        attack_range: k.range,
        attack_cooldown_ticks: k.cooldown,
    };
    let army = |team: u32| -> Vec<Command> {
        layout
            .iter()
            .map(|&(p, k)| spawn(team, if team == 0 { p } else { mirror(p) }, k))
            .collect()
    };
    // The team the sim updates first.
    let first = ctx.variant % 2;
    run.sim.step(&[army(first), army(1 - first)].concat());

    let ids = unit_ids(&run.sim);
    let team_of = |sim: &Sim, id: UnitId| sim.units().get(id).map(|u| u.team);
    let (a, b): (Vec<UnitId>, Vec<UnitId>) = ids
        .iter()
        .partition(|&&id| team_of(&run.sim, id) == Some(0));
    run.sim.step(&[
        Command::AttackMove {
            units: a,
            goal: mirror(front),
        },
        Command::AttackMove {
            units: b,
            goal: front,
        },
    ]);
    for &id in &ids {
        run.track(id, Route::none());
    }
    if ctx.trace {
        run.record_trace(SPEC.name);
    }

    let army_hp: f32 = layout.iter().map(|(_, k)| k.health).sum();
    let mut prev_cooldown: BTreeMap<UnitId, u32> = BTreeMap::new();
    let (mut started, mut ended) = (None, None);
    let (mut shots, mut shot_ceiling) = (0u64, 0.0f64);
    let (mut fight_ticks, mut bystanding) = (0u64, 0u64);
    let (mut ranged_ticks, mut ranged_in_contact) = (0u64, 0u64);
    for t in 0..TICKS {
        run.step(&[]);
        let sim = &run.sim;
        let units: Vec<_> = sim.units().iter().collect();
        let mut alive = [0u32; 2];
        for (id, u) in &units {
            alive[u.team as usize] += 1;
            // A cooldown only rises when a shot lands (see `metrics::Tracked`).
            let fired = prev_cooldown
                .insert(*id, u.cooldown_left)
                .is_some_and(|p| u.cooldown_left > p);
            if fired {
                started.get_or_insert(t);
            }
            let gap = |e: &Unit| (e.pos - u.pos).length() - e.radius - u.radius;
            let enemies = || units.iter().map(|(_, e)| *e).filter(|e| e.team != u.team);
            // Only while an enemy is close and in sight: before contact and
            // after one side is gone there is nothing to fight, and a unit's
            // share of that time says nothing about how it fights.
            if !enemies()
                .any(|e| gap(e) <= FIGHT_RADIUS && clear_los(sim.navmesh(), u.pos, e.pos, 0.0))
            {
                continue;
            }
            fight_ticks += 1;
            shots += fired as u64;
            shot_ceiling += 1.0 / u.attack_cooldown_ticks as f64;
            bystanding += u.target.is_none() as u64;
            if u.attack_range > MELEE.range {
                ranged_ticks += 1;
                let nearest = enemies().map(gap).fold(f32::INFINITY, f32::min);
                ranged_in_contact += (nearest < u.radius) as u64;
            }
        }
        if ended.is_none() && alive.contains(&0) {
            ended = Some(t);
        }
    }
    if ctx.trace {
        run.write_trace(&ctx.out_dir);
    }
    let s = run.stats();

    let hp = |team: u32| -> f64 {
        let left: f32 = run
            .sim
            .units()
            .iter()
            .filter(|(_, u)| u.team == team)
            .map(|(_, u)| u.health)
            .sum();
        (left / army_hp) as f64
    };
    let first_edge = hp(first) - hp(1 - first);
    let resolve = match (started, ended) {
        (Some(a), Some(b)) => (b - a) as f64,
        _ => TICKS as f64,
    };
    let frac = |n: u64, d: u64| if d == 0 { 0.0 } else { n as f64 / d as f64 };
    let dead = (2 * PER_SIDE - run.sim.units().iter().count()) as u64;

    //                          name              unit          good     bad  weight
    vec![
        // Shots over what every unit in the fight could have fired. Not all
        // of a melee army can reach the front at once, so 1.0 is out of reach.
        m("fire_efficiency", "frac", 0.70, 0.20, 3.0).at(if shot_ceiling > 0.0 {
            shots as f64 / shot_ceiling
        } else {
            0.0
        }),
        // Units in the fight with no target: walking past it, or standing by
        // next to it.
        m("bystanders", "frac", 0.00, 0.40, 3.0).at(frac(bystanding, fight_ticks)),
        // First shot to the last unit of one side dying; the whole run if
        // neither side ever does, which is a fight that broke up unfinished.
        m("resolve", "ticks", 400.00, 2000.00, 2.0).at(resolve),
        // Ranged units touching an enemy: fighting from a melee unit's place.
        m("ranged_contact", "frac", 0.05, 0.50, 1.0).at(frac(ranged_in_contact, ranged_ticks)),
        // Health left on the winning side. Mirrored armies should draw, so
        // anything else is the sim's order of play, amplified by the fight.
        m("outcome_gap", "hp frac", 0.00, 0.50, 1.0).at(first_edge.abs()),
        m("jitter", "rad/tick", 0.05, 0.60, 1.0).at(s.jitter),
        // The same with a sign: positive favours the side updated first, so
        // its mean over variants is the bias, and the swings cancel.
        m("first_mover_edge", "hp frac", 0.00, 0.00, 0.0).at(first_edge),
        m("dead", "frac", 0.00, 0.00, 0.0).at(frac(dead, 2 * PER_SIDE as u64)),
        m("side_churn", "per unit", 0.00, 0.00, 0.0).at(s.side_churn),
        m("push_enemy", "radii/tick", 0.00, 0.00, 0.0).at(s.push_enemy),
        m("push_ally", "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Funnel, &s))
    .collect()
}
