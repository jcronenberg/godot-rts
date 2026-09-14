//! A crowd given its whole route up front: one `Move` and three shift-queued
//! moves, around the three corridors and two blind turns of the serpentine.
//!
//! What the single-order scenarios cannot see. Each leg ends mid-corridor, so
//! the crowd re-forms against a wall with nowhere to spill sideways, and the
//! next order starts from whatever shape that left it in. Three things are
//! being asked at once, and they trade: arrive quickly (`lateness_*`), arrive
//! as one body (`spread*`), and stop centred on the point that was ordered
//! (`centroid_offset`). A march that strings out down a corridor scores the
//! first and loses the second; one that waits for every straggler at every
//! waypoint does the reverse.
//!
//! What it caught when it was written: one unit of sixty was given a place in
//! the first waypoint's blob that lay inside the wall's clearance, wedged
//! against that wall trying to reach it, and so never parked and never started
//! the legs queued behind it — `stall_trips` never saw it, because a unit
//! inside its arrival zone is held to be crowded rather than stuck. `assign_blob`
//! now drops a place a body cannot stand on, so `stranded` holds at zero; a
//! regression there is a blob laid out over ground its units cannot occupy.

use godot::prelude::Vector2;
use rts_lib::sim::{Command, Order, Sim};

use crate::harness::{Ctx, Reading, ScenarioSpec, metric as m};
use crate::maps::{serpentine, v};
use crate::metrics::{Cfg, Route, Run, unit_ids, wall_segments};

pub const SPEC: ScenarioSpec = ScenarioSpec {
    name: "queued_corridors_60",
    run,
};

const RADIUS: f32 = 5.0;
const SPEED: f32 = 40.0;
const LANE: f32 = 60.0;
const UNITS: usize = 60;
const TICKS: u64 = 3_500;

/// The serpentine's extent, fixed by `maps::serpentine`.
const W: f32 = 600.0;
const H: f32 = 400.0;

/// The ordered route. Every waypoint sits in the middle of a stretch rather
/// than in a corner, so what queues up at one is the crowd's own depth and not
/// the geometry of a bend. The last is a crowd's half-length short of the far
/// wall, leaving room to settle *around* it rather than against it.
const LEGS: [Vector2; 4] = [
    Vector2::new(W - LANE * 0.5, (H + LANE) * 0.25), // down the right link
    Vector2::new(W * 0.5, H * 0.5),                  // back along the middle lane
    Vector2::new(LANE * 0.5, (3.0 * H - LANE) * 0.25), // down the left link
    Vector2::new(W - 100.0, H - LANE * 0.5),         // out along the bottom lane
];

fn run(ctx: &Ctx) -> Vec<Reading> {
    let (points, constraints) = serpentine(LANE);
    let walls = wall_segments(&points, &constraints);
    // One field per leg: a queued march is priced leg by leg, or the detour
    // around the S reads as the crowd's own dawdling.
    let fields: Vec<_> = LEGS
        .iter()
        .map(|&goal| super::field(ctx, &walls, RADIUS, goal))
        .collect();

    let mut run = Run::new(
        Sim::new(points, &constraints, 0x5E_47),
        Cfg {
            // Tight enough to bind inside the tick budget: everything that
            // walks the route at all is parked by ~1.2 ideal traversals, so a
            // unit that misses this deadline is in trouble rather than slow,
            // and `arrival` and `stranded` measure different things.
            deadline_mult: 2.0,
            // Across the bottom lane, which the route runs down once and in
            // one direction; oriented so `side_of(a, b, GOAL) > 0`. Past the
            // walls at either end, so a unit hugging one still counts.
            choke: Some((v(W * 0.5, H + 5.0), v(W * 0.5, H - LANE - 5.0))),
        },
    );
    // Four abreast, all a 60-wide lane takes, and fifteen ranks deep: the
    // crowd starts as a column and has to stay one.
    run.sim.step(&super::spawn_block(
        v(20.0, 10.0),
        15,
        12.0,
        UNITS,
        RADIUS,
        SPEED,
    ));
    let ids = unit_ids(&run.sim);
    // The whole route before the first step of it: a `Move` and three
    // shift-clicks, so the sim advances the queue itself.
    let mut orders = vec![Command::Move {
        units: ids.clone(),
        goal: LEGS[0],
    }];
    orders.extend(LEGS[1..].iter().map(|&goal| Command::Queue {
        units: ids.clone(),
        order: Order::Move { goal },
    }));
    run.sim.step(&orders);
    for &id in &ids {
        let start = run.sim.units().get(id).map(|u| u.pos).unwrap_or(LEGS[0]);
        run.track(id, Route::via(&fields, start));
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
        // A march that leaves bodies behind is the worst thing this scenario
        // can catch, so it is priced twice and steeply. `stranded` runs out of
        // road at three units of sixty, which is a ladder of 100/67/33/0 and
        // not a cliff; `arrival` adds the same units back at a tenth of the
        // slope, and picks up merely-late ones that `stranded` forgives.
        m("stranded", "frac", 0.00, 0.05, 4.0).at(s.stranded),
        m("arrival", "frac", 1.00, 0.90, 3.0).at(s.arrival),
        // Both anchored at the reference optimum itself, so 100 means the
        // queue cost the crowd nothing. What lies between the anchors is a
        // march that waits out every straggler at every waypoint.
        m("lateness_p50", "ratio", 1.00, 2.00, 2.0).or_bad(s.lateness_p50),
        m("lateness_p95", "ratio", 1.00, 4.00, 2.0).or_bad(s.lateness_p95),
        m("throughput", "units/s", 20.00, 3.00, 1.0).at(s.throughput),
        m("settle_p95", "frac run", 0.20, 1.00, 1.0).or_bad(s.settle_p95),
        // 0.71 is a packed disc and ~0.97 is the tightest a crowd this size
        // can be in a lane this wide, so 1.0 is "as good as the corridor
        // allows" and 3.0 is a column strung out over a third of the map.
        m("spread", "radii", 1.00, 3.00, 3.0).or_bad(s.spread),
        m("spread_p95", "radii", 1.20, 4.00, 2.0).or_bad(s.spread_p95),
        m("centroid_offset", "radii", 0.00, 12.00, 3.0).or_bad(s.centroid_offset),
        m("residual", "radii", 6.00, 50.00, 2.0).or_bad(s.residual),
        m("residual_p95", "radii", 9.00, 60.00, 1.0).or_bad(s.residual_p95),
        m("detour", "ratio", 1.05, 1.80, 1.0).or_bad(s.detour),
        m("stall_trips", "per unit", 0.00, 10.00, 1.0).at(s.stall_trips),
        m("jitter", "rad/tick", 0.05, 0.60, 1.0).at(s.jitter),
        // The same shape as `spread`, but per `Unit::group`: reading tighter
        // than `spread` is the tell that the crowd came apart and the sim
        // relabelled the pieces. `repaths` counts the queue itself, three
        // fresh paths before anything has gone wrong.
        m("cohesion", "radii", 0.00, 0.00, 0.0).or_bad(s.cohesion),
        m("cohesion_at_end", "radii", 0.00, 0.00, 0.0).or_bad(s.cohesion_final),
        m("repaths", "per unit", 0.00, 0.00, 0.0).at(s.repaths),
        m("push_ally", "radii/tick", 0.00, 0.00, 0.0).at(s.push_ally),
    ]
    .into_iter()
    .chain(super::overlap_readings(super::Crowding::Funnel, &s))
    .collect()
}
