//! One module per scenario: a seeded fixture plus the metrics it is *about*.
//! Nothing scores everything.
//!
//! Every metric here needs a *gradient*, a meaningful "somewhat better"
//! between its anchors. A quantity that is either right or broken belongs in a
//! `#[cfg(test)]` module instead, where it fails the build rather than scoring
//! 100 forever; `unreachable_goal` and the wall-penetration counts moved out
//! of this directory on exactly that reasoning.

use std::rc::Rc;

use godot::prelude::Vector2;
use rts_lib::sim::Command;

use crate::harness::{Reading, ScenarioSpec, metric as m};
use crate::metrics::{Stats, spawn_cmd};
use crate::reference;

mod combat_blob;
mod corridor_sizes;
mod counterflow;
mod door_funnel;
mod impassable_gap;
mod obstacle_drop;
mod queued_corridors;
mod solo_march;
mod stutter_step;

/// Registry, in the order the table prints them: the control first, then the
/// movement family, then the clearance family, then combat.
pub const ALL: &[ScenarioSpec] = &[
    solo_march::SPEC,
    door_funnel::SPEC,
    counterflow::SPEC,
    queued_corridors::SPEC,
    obstacle_drop::SPEC,
    impassable_gap::SPEC,
    corridor_sizes::SPEC,
    combat_blob::SPEC,
    stutter_step::SPEC,
];

/// How much crowding a fixture *forces*, which the overlap zero point scales
/// to. Eight units on a roomy ring and forty converging on one melee target
/// are different problems, and one anchor pair would either excuse the former
/// or condemn the latter for geometry it cannot avoid. The price: overlap
/// columns compare *within* a scenario across runs, not *between* scenarios.
pub enum Crowding {
    /// Units have somewhere else to be. Any sustained overlap is a choice the
    /// flocking made, so the bar is high.
    Open,
    /// Forced through an aperture narrower than the crowd. Some compression is
    /// inherent; a lot of it is not.
    Funnel,
    /// Many bodies converging on one point, where packing is unavoidable and
    /// the ring around the target is genuinely oversubscribed.
    Crush,
}

impl Crowding {
    /// `(good, bad)` anchors for `(mean, p95, max, frac)`.
    ///
    /// `mean` and `p95` scale to the tier. `max` and `frac` do not: their zero
    /// points are the *physics* (2.0 radii is coincident centres, 1.0 is every
    /// unit overlapping every tick), so they stay comparable across scenarios
    /// and a scenario at the cap is reporting a real cap, not a bad anchor.
    fn anchors(&self) -> [(f64, f64); 4] {
        match self {
            //                    mean          p95           max           frac
            Crowding::Open => [(0.00, 0.20), (0.00, 0.90), (0.10, 2.00), (0.00, 1.00)],
            Crowding::Funnel => [(0.00, 0.30), (0.00, 1.20), (0.20, 2.00), (0.05, 1.00)],
            Crowding::Crush => [(0.05, 0.80), (0.10, 1.60), (0.30, 2.00), (0.20, 1.00)],
        }
    }
}

/// The overlap block: four readings at a combined weight of 6, roughly a third
/// of a scenario's score. All four because each hides what the others show:
/// `mean` is diluted by quiet ticks, `p95` says how bad it gets, `max` catches
/// the single worst moment, `frac` says how much of the run overlapped at all.
pub fn overlap_readings(crowding: Crowding, s: &Stats) -> Vec<Reading> {
    let [mean, p95, max, frac] = crowding.anchors();
    vec![
        m("overlap_mean", "radii", mean.0, mean.1, 2.0).at(s.overlap_mean),
        m("overlap_p95", "radii", p95.0, p95.1, 2.0).at(s.overlap_p95),
        m("overlap_max", "radii", max.0, max.1, 1.0).at(s.overlap_max),
        m("overlap_frac", "frac", frac.0, frac.1, 1.0).at(s.overlap_frac),
    ]
}

/// Reference raster pitch for a query radius. Finer than a quarter-radius
/// buys nothing and costs quadratically; coarser than 2 units blurs a doorway.
/// Clamped so a huge map with small units stays quick.
pub fn ref_cell(radius: f32) -> f32 {
    (radius / 4.0).clamp(0.5, 2.0)
}

/// Cost-to-goal field over `walls` at `radius`, cached under `ctx.cache_dir`.
/// Shared, because a tracked unit holds on to the field that prices its route
/// (see [`crate::metrics::Route`]).
pub fn field(
    ctx: &crate::harness::Ctx,
    walls: &[(Vector2, Vector2)],
    radius: f32,
    goal: Vector2,
) -> Rc<reference::Field> {
    Rc::new(reference::field(
        walls,
        radius,
        ref_cell(radius),
        goal,
        &ctx.cache_dir,
    ))
}

/// `count` unarmed units in a `cols`-wide grid at `pitch` spacing, anchored at
/// `origin`. Deterministic and collision-free as long as `pitch > 2 * radius`.
pub fn spawn_block(
    origin: Vector2,
    cols: usize,
    pitch: f32,
    count: usize,
    radius: f32,
    speed: f32,
) -> Vec<Command> {
    (0..count)
        .map(|i| {
            let p = Vector2::new(
                origin.x + (i % cols) as f32 * pitch,
                origin.y + (i / cols) as f32 * pitch,
            );
            spawn_cmd(p, radius, speed)
        })
        .collect()
}
