//! One module per scenario.
//!
//! A scenario is a seeded fixture plus the list of metrics it is *about*.
//! Nothing scores everything: `door_funnel` has no combat metrics, and
//! `impassable_gap` weights phantoms and clearance far above how quickly
//! anyone gets anywhere.
//!
//! Every scenario here has to have a *gradient*. The scorecard quantifies how
//! the simulation should feel, which means each metric needs a meaningful
//! "somewhat better" between its anchors. A quantity that is either right or
//! broken has no such middle, and belongs in a `#[cfg(test)]` module next to
//! the code, where it fails the build instead of scoring 100 forever. Two
//! things moved out of this directory on exactly that reasoning:
//! `unreachable_goal` (now `sim::tests::test_sealed_goal_is_refused_without_a_spin`)
//! and the wall-penetration counts (`test_crowd_never_squeezes_through_*` and
//! `test_dense_crowd_through_a_doorway_never_crosses_a_wall`).

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
mod solo_march;
mod stutter_step;

/// Registry, in the order the table prints them: the control first, then the
/// movement family, then the clearance family, then combat.
pub const ALL: &[ScenarioSpec] = &[
    solo_march::SPEC,
    door_funnel::SPEC,
    counterflow::SPEC,
    obstacle_drop::SPEC,
    impassable_gap::SPEC,
    corridor_sizes::SPEC,
    combat_blob::SPEC,
    stutter_step::SPEC,
];

/// How much crowding a fixture *forces*, which is what the overlap zero point
/// is scaled to.
///
/// Overlap is scored in every scenario where units can crowd, but "how much is
/// too much" is not one number across all of them: eight units on a ring with
/// room to spare and forty converging on a single melee target are physically
/// different problems, and a single anchor pair would either excuse the march
/// scenarios or condemn the crush ones for geometry they cannot avoid. The
/// price is that the overlap columns are comparable *within* a scenario across
/// runs, which is what a scorecard is for, and not *between* scenarios.
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
    /// `mean` and `p95` are scaled to the tier, since what counts as too deep
    /// depends on how much crowding the fixture forces. `max` and `frac` are
    /// not: their zero points are the *physics*, 2.0 radii being two coincident
    /// centres and 1.0 being every unit overlapping on every tick. Nothing can
    /// be worse than either, so those two columns stay comparable across
    /// scenarios and a scenario sitting at the cap is reporting a real cap and
    /// not a badly chosen anchor.
    fn anchors(&self) -> [(f64, f64); 4] {
        match self {
            //                    mean          p95           max           frac
            Crowding::Open => [(0.00, 0.20), (0.00, 0.90), (0.10, 2.00), (0.00, 1.00)],
            Crowding::Funnel => [(0.00, 0.30), (0.00, 1.20), (0.20, 2.00), (0.05, 1.00)],
            Crowding::Crush => [(0.05, 0.80), (0.10, 1.60), (0.30, 2.00), (0.20, 1.00)],
        }
    }
}

/// The overlap block: four readings at a combined weight of 6, which lands
/// overlap at roughly a third of a scenario's score.
///
/// All four, because each hides something the others show. `mean` is diluted
/// by every tick nothing is touching; `p95` says how bad it is when it is bad;
/// `max` catches the one moment two bodies nearly coincided; `frac` says how
/// much of the run was spent overlapping at all rather than how deeply.
pub fn overlap_readings(crowding: Crowding, s: &Stats) -> Vec<Reading> {
    let [mean, p95, max, frac] = crowding.anchors();
    vec![
        m("overlap_mean", "radii", mean.0, mean.1, 2.0).at(s.overlap_mean),
        m("overlap_p95", "radii", p95.0, p95.1, 2.0).at(s.overlap_p95),
        m("overlap_max", "radii", max.0, max.1, 1.0).at(s.overlap_max),
        m("overlap_frac", "frac", frac.0, frac.1, 1.0).at(s.overlap_frac),
    ]
}

/// Reference raster pitch for a query radius.
///
/// Finer than a quarter-radius buys nothing (the sampling slack is already
/// half a cell) and costs quadratically; coarser than 2 units starts blurring
/// a doorway. Clamped at both ends so a huge map with small units doesn't turn
/// one Dijkstra into a coffee break.
pub fn ref_cell(radius: f32) -> f32 {
    (radius / 4.0).clamp(0.5, 2.0)
}

/// Cost-to-goal field over `walls` at `radius`, cached under `ctx.cache_dir`.
pub fn field(
    ctx: &crate::harness::Ctx,
    walls: &[(Vector2, Vector2)],
    radius: f32,
    goal: Vector2,
) -> reference::Field {
    reference::field(walls, radius, ref_cell(radius), goal, &ctx.cache_dir)
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
