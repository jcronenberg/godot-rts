//! Deterministic fixed-tick unit simulation.
//!
//! `Sim::step` is a pure state transition: commands in, state out. Pacing,
//! threading and snapshots live in [`crate::sim_runner`]. Determinism rules:
//! f32 add/mul/div/sqrt only, slot-order iteration, no hash iteration over
//! state, pairwise forces buffered before application, one seeded RNG.

use std::collections::VecDeque;

use godot::prelude::Vector2;

use crate::abstraction::Abstraction;
use crate::astar::{
    AStarScratch, clear_los, clip_ray_to_walls, closest_on_segment, dist_segment_segment,
    find_path_abstract,
};
use crate::delaunay::{CDT, FNV_OFFSET, FNV_PRIME};
use crate::navmesh::{DynamicNavmesh, Obstacle, ObstacleId};

/// Simulation ticks per second.
pub const TICK_RATE: u32 = 30;
/// Fixed timestep in seconds.
pub const DT: f32 = 1.0 / TICK_RATE as f32;

// ── Tunables ─────────────────────────────────────────────────────────────────
// Runtime-settable (via `.get()`/`.set()`) so values can be swept without
// recompiling; drop back to plain `const`s if the atomic load ever shows up
// in profiles (call sites would lose the `.get()`).

/// Fraction of pairwise overlap corrected per tick — the separation push force.
/// Soft enough that path pressure wins while units are *moving* (tolerating a
/// little transient overlap through crowds/funnels), but with no path pull at
/// rest it still converges to a fully non-overlapping packing. Below the level
/// where summed pushes in a clump overshoot and ping-pong.
pub static SEPARATION_RELAX: TunableF32 = TunableF32::new(0.4);
/// Per-tick separation displacement cap, as a fraction of the unit's speed.
/// Above a full step, so separation can out-push a unit's own path/cohesion
/// convergence and resolve overlap rather than tolerate it.
pub static SEPARATION_MAX_FRAC: TunableF32 = TunableF32::new(1.5);
/// Cohesion radius as a multiple of the largest unit radius: same-group
/// neighbours within it pull toward their shared centroid.
pub static COHESION_RADIUS_FRAC: TunableF32 = TunableF32::new(5.0);
/// Per-tick fraction of the centroid offset applied as a cohesion pull.
pub static COHESION_GAIN: TunableF32 = TunableF32::new(0.05);
/// Per-tick cohesion displacement cap, as a fraction of the unit's speed. Kept
/// well below the path step (and the separation cap) so cohesion stays a gentle
/// bias and never pulls group-mates back into overlap.
pub static COHESION_MAX_FRAC: TunableF32 = TunableF32::new(0.15);
/// A moving unit joins a parked group-mate's cluster (and stops) when within
/// this multiple of touching distance of it — *and* within its arrival radius
/// of the goal (below). So a group settles into a blob around its goal instead
/// of every unit driving to the exact goal point and crushing inward.
pub static ARRIVAL_TOUCH_FRAC: TunableF32 = TunableF32::new(1.15);
/// Arrival radius = `r * max(ARRIVAL_MIN_RADII, FACTOR*sqrt(N))`; a unit only
/// crowd-stops inside it. `sqrt(N)` ~ packed-disk radius of N circles; FACTOR>1
/// pads it so units stop at the blob's edge, not the core; MIN keeps small
/// groups (followers sit ~2r out) stoppable at all.
pub static ARRIVAL_RADIUS_FACTOR: TunableF32 = TunableF32::new(1.5);
pub static ARRIVAL_MIN_RADII: TunableF32 = TunableF32::new(3.0);
/// Fraction of a flock's lateral spread applied as corner-fan offset. Below 1.0
/// so outer units round a touch tighter and their separation doesn't shove them
/// onto the corridor edge.
pub static FAN_FRAC: TunableF32 = TunableF32::new(0.34);
/// Fraction of a flock's lateral spread held while marching a straight (corner-
/// free) leg. Unlike the bend fan (spreads units *out* from the apex), units
/// already start at full spread here, so this *relaxes* it: <1 lets them draw
/// closer than their start formation while still marching side-by-side, not
/// single-file.
pub static STRAIGHT_FAN_FRAC: TunableF32 = TunableF32::new(0.6);
/// Wall-clamped ticks heading into a wall without progress before a moving unit
/// is treated as stuck (shoved off its path onto a corner) and repathed from its
/// current position. Eager (~0.1 s at 30 Hz) so displaced units recover a valid
/// route quickly; transient clamps self-resolve before they trip it.
pub static STALL_REPATH_TICKS: TunableU8 = TunableU8::new(3);
/// Remaining-path-length improvement that counts as real progress (and resets
/// the stall counter); below it the unit is treated as not advancing.
pub static STALL_PROGRESS_EPS: TunableF32 = TunableF32::new(0.1);
/// How far a unit will let itself be drawn from its post, in its own radii,
/// before it breaks off a fight it started itself and walks home. Floored at
/// the acquisition radius, so a unit can always reach what it noticed —
/// otherwise a long-reach unit would acquire at a distance it isn't allowed to
/// walk and spend the fight acquiring and breaking off.
///
/// Only self-defence leashes. A commanded `Attack` is the player's decision
/// and chases to the end of the map; an attack-mover has a march goal that
/// already bounds it.
pub static LEASH_RADII: TunableF32 = TunableF32::new(12.0);
/// Multiple of a unit's own `attack_range` used as its attack-move
/// acquisition radius — how far it "notices" an enemy before being in
/// weapon range, so it starts closing the distance rather than only
/// reacting once already adjacent.
pub static ACQUISITION_RANGE_MULT: TunableF32 = TunableF32::new(3.0);
/// Distance a chased target may drift from the anchor its current chase path
/// was built toward before that path is rebuilt — chase-repath hysteresis,
/// distance half (see [`CHASE_REPATH_TICKS`] for the time half).
pub static CHASE_REPATH_DIST: TunableF32 = TunableF32::new(15.0);
/// Ticks between forced chase-path rebuilds regardless of drift, so a target
/// weaving right at [`CHASE_REPATH_DIST`] still gets a fresh path
/// periodically.
pub static CHASE_REPATH_TICKS: TunableU8 = TunableU8::new(15);
/// Station-hold hysteresis, in full path steps (`max_speed * DT`): a unit
/// takes up its firing station at `attack_range` and only gives it up (and
/// walks again) past `attack_range + this`.
///
/// It has to clear the per-tick jitter it is there to absorb, which is
/// [`SEPARATION_MAX_FRAC`] of a step — one step, the obvious reading of
/// "sized to per-tick crowd displacement", is *under* that and still flapped.
/// Widening it costs nothing in reach: firing re-checks the true range every
/// tick, so a unit shoved out of range holds its ground and its cooldown but
/// lands no hits until it is back inside.
pub static FIRE_SLACK_STEPS: TunableF32 = TunableF32::new(3.0);
/// Fraction of a full step by which a chasing unit must close its surface gap
/// to count as advancing. A fraction, not an absolute distance: a unit
/// grinding through a crush still covers real ground every tick, so any
/// absolute epsilon reads that as progress and the block signal never fires.
pub static BLOCK_PROGRESS_FRAC: TunableF32 = TunableF32::new(0.25);
/// Consecutive non-advancing chase ticks before a unit concedes the direct
/// approach and re-goals onto a free approach slot.
pub static BLOCK_TICKS: TunableU8 = TunableU8::new(10);
/// How far *inside* weapon reach an approach station sits, in the unit's own
/// radii: a station is at `r_self + r_target + attack_range - this * r_self`.
///
/// A margin in body units, deliberately not a fraction of `attack_range`: the
/// margin exists to absorb the crowd shoving a stationed unit around, which is
/// a body-scale effect and has nothing to do with how far the weapon reaches.
/// As a fraction of range it vanishes exactly where it is needed most — a
/// melee station would sit 0.4 units inside a reach of 2, so the first shove
/// puts the front rank out of range and it walks back in, shove after shove,
/// which is the stutter. Both radii stay in the formula: dropping them (the
/// classic "ring attractor" bug) parks attackers just out of reach instead.
pub static SLOT_STANDOFF: TunableF32 = TunableF32::new(0.5);
/// Flat cost added to reserve-ring (out of weapon range) slot candidates, so
/// a unit only waits in reserve when every in-range station is genuinely
/// crowded. Well above a single body's worth of occupancy: standing where you
/// can shoot beats standing somewhere roomy where you can't.
pub static SLOT_OUTER_PENALTY: TunableF32 = TunableF32::new(2.0);
/// Cost per ring stepped inward from weapon reach, so units fill the outermost
/// ring first and only crowd closer when it is full.
pub static SLOT_INNER_PENALTY: TunableF32 = TunableF32::new(0.35);
/// Weight on the arc a unit would have to walk to reach a candidate station
/// (`1 - cos` between its own bearing from the target and the candidate's), so
/// a unit takes the free station nearest to where it already stands rather
/// than crossing the fight for an equally free one.
///
/// Low on purpose: it is the only term pulling *against* spreading out, and
/// the walk it saves is short compared to what an evenly surrounded target is
/// worth. Measured, dropping it from 0.5 to 0.15 took ten ranged attackers
/// from four compass sectors to seven with no loss of damage.
pub static SLOT_TURN_COST: TunableF32 = TunableF32::new(0.15);
/// Cost a new station must beat the held one by before a unit switches.
///
/// Above one body's worth of occupancy, deliberately: the crowd around a
/// station changes every tick, so a margin thinner than a body has units
/// swapping stations on every re-score, walking the long way round to each new
/// one and never arriving anywhere. Measured, raising it from a quarter of a
/// body to one and a half took a 12-attacker melee from 6.3 to 8.6 damage a
/// tick, purely by letting units commit.
pub static SLOT_SWITCH_MARGIN: TunableF32 = TunableF32::new(1.5);
/// Separation weight of a *holding* unit (firing, parked or idle) against a
/// moving one's weight of 1: each side takes `w_other / (w_i + w_j)` of the
/// pairwise correction, so a mover shoves a stationary unit only `1/(1+w)` as
/// far as it moves itself. At rest both sides hold, the split is exactly half
/// and packing behaviour is bit-for-bit unchanged.
///
/// Swept against a 12-attacker blob *and* a unit crossing a stationary ally
/// line at once. The two want opposite ends (a stiff front rank helps the
/// blob, a soft one lets a traveller through), and 4 is where neither loses. Past ~8 the transit slows back past baseline;
/// below ~2 the blob's stutter returns.
pub static HOLD_WEIGHT: TunableF32 = TunableF32::new(4.0);
/// Fraction of a full step of remaining-path progress below which a moving
/// unit in ally contact counts as blocked by bodies (the detour's trigger).
pub static DETOUR_FRAC: TunableF32 = TunableF32::new(0.25);
/// Consecutive blocked ticks before a moving unit inserts a lateral detour
/// waypoint instead of grinding on through its allies.
pub static DETOUR_TICKS: TunableU8 = TunableU8::new(10);
/// Detour waypoint offset in unit radii: `pos + lat * L + heading * L/2`.
/// Deliberately short — a still-blocked unit trips again and the detour
/// deepens, which handles a wide wall without over-committing to a narrow one.
pub static DETOUR_LEN_RADII: TunableF32 = TunableF32::new(6.0);
/// Acquisition penalty per ally already targeting a candidate, in body
/// diameters of standoff. Only ~`2 pi d / 2r` attackers fit around one
/// defender; without this the surplus queues behind them forever instead of
/// picking another enemy.
pub static TARGET_SPREAD_PENALTY: TunableF32 = TunableF32::new(1.0);

/// Diagnostic switch for [`Sim::last_push`]: while on, `flock` records how much
/// separation push each unit received, split by ally and enemy.
///
/// Not a `set_tuning` knob, since it changes no behaviour. Off by default
/// because accumulating it costs the separation pass ~3% at 2000 units (~7% at
/// 100) and only `examples/quality` reads it.
pub static PUSH_TRACKING: std::sync::atomic::AtomicBool =
    std::sync::atomic::AtomicBool::new(false);

/// Max `wall_clamp` passes per unit per tick before giving up as unresolved.
/// Internal convergence detail, not a gameplay knob — plain const rather than
/// a `Tunable`.
const WALL_CLAMP_PASSES: u32 = 8;
/// Clearance slack (fraction of radius) below which a clamped position still
/// counts as overlapping a wall. Just enough for f32 noise in an exact-fit
/// (`gap == 2r`) corridor; more would let genuinely-too-tight gaps pass.
const WALL_CLAMP_REVERT_EPS_FRAC: f32 = 0.0005;

/// Runtime-settable f32, stored as bits in an atomic. Backs the tunables above.
pub struct TunableF32(std::sync::atomic::AtomicU32);
impl TunableF32 {
    const fn new(v: f32) -> Self {
        Self(std::sync::atomic::AtomicU32::new(v.to_bits()))
    }
    #[inline]
    pub fn get(&self) -> f32 {
        f32::from_bits(self.0.load(std::sync::atomic::Ordering::Relaxed))
    }
    pub fn set(&self, v: f32) {
        self.0
            .store(v.to_bits(), std::sync::atomic::Ordering::Relaxed)
    }
}

/// Runtime-settable u8, stored in an atomic. Backs [`STALL_REPATH_TICKS`].
pub struct TunableU8(std::sync::atomic::AtomicU8);
impl TunableU8 {
    const fn new(v: u8) -> Self {
        Self(std::sync::atomic::AtomicU8::new(v))
    }
    #[inline]
    pub fn get(&self) -> u8 {
        self.0.load(std::sync::atomic::Ordering::Relaxed)
    }
    pub fn set(&self, v: u8) {
        self.0.store(v, std::sync::atomic::Ordering::Relaxed)
    }
}

/// Sets a tunable above by its lowercased name (e.g. `"separation_relax"`);
/// returns `false` if `name` doesn't match one. Used by the sim_test debug UI
/// via `Simulation.set_tuning` — see `rust/rts/src/simulation.rs`.
pub fn set_tuning(name: &str, value: f32) -> bool {
    match name {
        "separation_relax" => SEPARATION_RELAX.set(value),
        "separation_max_frac" => SEPARATION_MAX_FRAC.set(value),
        "cohesion_radius_frac" => COHESION_RADIUS_FRAC.set(value),
        "cohesion_gain" => COHESION_GAIN.set(value),
        "cohesion_max_frac" => COHESION_MAX_FRAC.set(value),
        "arrival_touch_frac" => ARRIVAL_TOUCH_FRAC.set(value),
        "arrival_radius_factor" => ARRIVAL_RADIUS_FACTOR.set(value),
        "arrival_min_radii" => ARRIVAL_MIN_RADII.set(value),
        "fan_frac" => FAN_FRAC.set(value),
        "straight_fan_frac" => STRAIGHT_FAN_FRAC.set(value),
        "stall_repath_ticks" => STALL_REPATH_TICKS.set(value.round().clamp(0.0, 255.0) as u8),
        "stall_progress_eps" => STALL_PROGRESS_EPS.set(value),
        "acquisition_range_mult" => ACQUISITION_RANGE_MULT.set(value),
        "leash_radii" => LEASH_RADII.set(value),
        "chase_repath_dist" => CHASE_REPATH_DIST.set(value),
        "chase_repath_ticks" => CHASE_REPATH_TICKS.set(value.round().clamp(0.0, 255.0) as u8),
        "fire_slack_steps" => FIRE_SLACK_STEPS.set(value),
        "block_progress_frac" => BLOCK_PROGRESS_FRAC.set(value),
        "block_ticks" => BLOCK_TICKS.set(value.round().clamp(0.0, 255.0) as u8),
        "slot_standoff" => SLOT_STANDOFF.set(value),
        "slot_outer_penalty" => SLOT_OUTER_PENALTY.set(value),
        "slot_inner_penalty" => SLOT_INNER_PENALTY.set(value),
        "slot_turn_cost" => SLOT_TURN_COST.set(value),
        "slot_switch_margin" => SLOT_SWITCH_MARGIN.set(value),
        "hold_weight" => HOLD_WEIGHT.set(value),
        "detour_frac" => DETOUR_FRAC.set(value),
        "detour_ticks" => DETOUR_TICKS.set(value.round().clamp(0.0, 255.0) as u8),
        "detour_len_radii" => DETOUR_LEN_RADII.set(value),
        "target_spread_penalty" => TARGET_SPREAD_PENALTY.set(value),
        _ => return false,
    }
    true
}

/// Reads a tunable above by its lowercased name; `None` if `name` doesn't
/// match one. Lets UI (or anything else) start from the live value instead of
/// a second hardcoded copy of the default.
pub fn get_tuning(name: &str) -> Option<f32> {
    Some(match name {
        "separation_relax" => SEPARATION_RELAX.get(),
        "separation_max_frac" => SEPARATION_MAX_FRAC.get(),
        "cohesion_radius_frac" => COHESION_RADIUS_FRAC.get(),
        "cohesion_gain" => COHESION_GAIN.get(),
        "cohesion_max_frac" => COHESION_MAX_FRAC.get(),
        "arrival_touch_frac" => ARRIVAL_TOUCH_FRAC.get(),
        "arrival_radius_factor" => ARRIVAL_RADIUS_FACTOR.get(),
        "arrival_min_radii" => ARRIVAL_MIN_RADII.get(),
        "fan_frac" => FAN_FRAC.get(),
        "straight_fan_frac" => STRAIGHT_FAN_FRAC.get(),
        "stall_repath_ticks" => STALL_REPATH_TICKS.get() as f32,
        "stall_progress_eps" => STALL_PROGRESS_EPS.get(),
        "acquisition_range_mult" => ACQUISITION_RANGE_MULT.get(),
        "leash_radii" => LEASH_RADII.get(),
        "chase_repath_dist" => CHASE_REPATH_DIST.get(),
        "chase_repath_ticks" => CHASE_REPATH_TICKS.get() as f32,
        "fire_slack_steps" => FIRE_SLACK_STEPS.get(),
        "block_progress_frac" => BLOCK_PROGRESS_FRAC.get(),
        "block_ticks" => BLOCK_TICKS.get() as f32,
        "slot_standoff" => SLOT_STANDOFF.get(),
        "slot_outer_penalty" => SLOT_OUTER_PENALTY.get(),
        "slot_inner_penalty" => SLOT_INNER_PENALTY.get(),
        "slot_turn_cost" => SLOT_TURN_COST.get(),
        "slot_switch_margin" => SLOT_SWITCH_MARGIN.get(),
        "hold_weight" => HOLD_WEIGHT.get(),
        "detour_frac" => DETOUR_FRAC.get(),
        "detour_ticks" => DETOUR_TICKS.get() as f32,
        "detour_len_radii" => DETOUR_LEN_RADII.get(),
        "target_spread_penalty" => TARGET_SPREAD_PENALTY.get(),
        _ => return None,
    })
}

// ── RNG ───────────────────────────────────────────────────────────────────────

/// PCG-XSH-RR 32-bit generator (O'Neill 2014); the sim's only randomness.
#[derive(Clone, Debug)]
pub struct Pcg32 {
    state: u64,
    inc: u64,
}

impl Pcg32 {
    pub fn new(seed: u64) -> Pcg32 {
        let mut rng = Pcg32 {
            state: 0,
            inc: 0xda3e_39cb_94b9_5bdb, // default stream, forced odd
        };
        rng.next_u32();
        rng.state = rng.state.wrapping_add(seed);
        rng.next_u32();
        rng
    }

    pub fn next_u32(&mut self) -> u32 {
        let old = self.state;
        self.state = old
            .wrapping_mul(6364136223846793005)
            .wrapping_add(self.inc | 1);
        let xorshifted = (((old >> 18) ^ old) >> 27) as u32;
        let rot = (old >> 59) as u32;
        xorshifted.rotate_right(rot)
    }

    /// Uniform in `[0, 1)` with 24 bits of precision.
    pub fn next_f32(&mut self) -> f32 {
        (self.next_u32() >> 8) as f32 * (1.0 / 16_777_216.0)
    }

    fn hash_into(&self, h: &mut Fnv) {
        h.write_u64(self.state);
        h.write_u64(self.inc);
    }
}

// ── Units: slot storage with generational ids ────────────────────────────────

/// Generational unit handle; stale ids (despawned slots) are ignored.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct UnitId {
    index: u32,
    generation: u32,
}

impl UnitId {
    /// Packed form for FFI layers; pairs with [`UnitId::from_raw`].
    pub fn raw(self) -> u64 {
        (self.generation as u64) << 32 | self.index as u64
    }

    pub fn from_raw(raw: u64) -> UnitId {
        UnitId {
            index: raw as u32,
            generation: (raw >> 32) as u32,
        }
    }
}

#[derive(Clone, Debug)]
pub struct Unit {
    pub pos: Vector2,
    /// Position at the start of the current tick (velocity = `(pos - prev_pos) / DT`).
    pub prev_pos: Vector2,
    pub radius: f32,
    pub max_speed: f32,
    /// Remaining waypoints `[next, …, goal]`; empty when idle.
    pub path: Vec<Vector2>,
    /// Index of the next waypoint to reach.
    pub path_i: u32,
    /// Pending orders, executed in turn once the current action finishes. The
    /// currently-active order isn't stored here — it's reflected in `path` etc.
    pub orders: VecDeque<Order>,
    /// Cohesion group, stamped per `Move` command; 0 = ungrouped (no cohesion).
    pub group: u32,
    /// Settled at the goal — either reached it, or joined a parked group-mate's
    /// cluster (crowd-arrival). Cleared on a new `Move`. Distinguishes a real
    /// goal arrival from a unit that merely idles (e.g. unreachable goal), so
    /// only true arrivals seed the cluster others stop against.
    pub parked: bool,
    /// Distance from the goal within which this unit may crowd-stop — sized to
    /// the group so the settled blob centres on the goal. Stamped per `Move`.
    pub arrival_r: f32,
    /// Ticks wall-clamped without beating [`Unit::min_remaining`]. A group
    /// shove can push a unit off its cleared path so its line to the next
    /// waypoint cuts a corner; once this trips it repaths from where it
    /// actually is. Reset on real progress or a new path.
    pub stall: u8,
    /// Best (smallest) remaining path length seen since the path was set — the
    /// jitter-proof progress yardstick for the stall detector (`MAX` = unset).
    pub min_remaining: f32,
    /// Team affiliation, stamped at spawn and immutable for now. Relations
    /// between teams live on [`Sim`], not here (see [`Sim::relation`]).
    pub team: u32,
    pub max_health: f32,
    pub health: f32,
    pub damage: f32,
    /// Surface-to-surface (centre distance minus both radii) engagement range.
    pub attack_range: f32,
    /// Ticks between attacks; authored and clamped to at least 1 at spawn.
    /// Kept in ticks, not seconds, so combat stays exactly reproducible.
    pub attack_cooldown_ticks: u32,
    /// Ticks remaining before this unit may fire again.
    pub cooldown_left: u32,
    /// Active combat target while executing `Attack` or an acquired
    /// `AttackMove` engagement; `None` when idle or marching without a
    /// target yet. Chase/fire state (in range or not) is derived from this
    /// plus current positions, not stored separately.
    pub target: Option<UnitId>,
    /// March goal of the active `AttackMove` order, kept alongside `target`
    /// so the march resumes once an acquired target dies. `None` while
    /// executing a plain `Move` or `Attack`, or when idle.
    pub attack_move_goal: Option<Vector2>,
    /// Target position the current chase path was built toward, and a
    /// countdown to the next scheduled rebuild — hysteresis so a moving
    /// target doesn't trigger a repath every tick. Meaningless while `target`
    /// is `None`.
    pub chase_anchor: Vector2,
    pub chase_repath_in: u8,
    /// Where this unit was standing when it last picked a fight of its own
    /// accord — the post it walks back to once that fight is over, so
    /// self-defence doesn't slowly disperse an army across the map one
    /// pursuit at a time. `None` for a unit that has never self-defended
    /// since its last order; a player order is what establishes a new post.
    ///
    /// Kept across a chain of fights, so a unit dragged twice still returns to
    /// where it originally stood, not to the last body it stood over.
    pub post: Option<Vector2>,
    /// Whether [`Unit::target`] came from an explicit `Attack` order rather
    /// than acquisition. The player's kill choice outranks the unit's own
    /// judgement: a commanded fight blocks queued orders until the target
    /// dies, where a unit that merely defended itself lets the queue proceed.
    pub target_commanded: bool,
    /// Latched "standing at my firing station". A *position* state, not a
    /// licence to fire: damage still needs a true `surf <= attack_range` on
    /// the tick it lands, so the slack that keeps this latched (see
    /// [`FIRE_SLACK_STEPS`]) can be as wide as crowd jitter needs without ever
    /// extending a weapon's effective reach.
    ///
    /// Explicit rather than derived from `surf <= attack_range` (which flapped
    /// every tick as the press jostled a unit across the boundary) or from
    /// `path.is_empty()` (which made a firing unit look idle to cohesion,
    /// merge and crowd arrival).
    pub engaged: bool,
    /// Approach slot this unit is walking to instead of the target itself,
    /// encoded `ring * 32 + direction index` ([`NO_SLOT`] = chase the target
    /// directly). Stored as a code, not a point, so the ring is re-derived from
    /// the target's *current* position each tick and a moving target simply
    /// drags its ring along.
    pub chase_slot: u16,
    /// Smallest distance to the station this unit is walking to (its target's
    /// surface, when it has no station) seen since the block counter last
    /// reset — the jitter-proof yardstick for "am I actually closing?"
    /// (`MAX` = unset). Same shape as [`Unit::min_remaining`] for walls.
    pub best_gap: f32,
    /// One counter serving both halves of "how long have I been stuck here",
    /// which are mutually exclusive — a unit is either walking to a station or
    /// standing on one, never both, so the two never need to be live at once:
    ///
    /// - **chasing**: consecutive ticks without closing [`BLOCK_PROGRESS_FRAC`]
    ///   of a step on the station; at [`BLOCK_TICKS`] it re-goals onto a free
    ///   approach slot.
    /// - **stationed**: consecutive ticks held at a station it can't shoot
    ///   from; at [`BLOCK_TICKS`] it gives the station up and re-scores.
    ///
    /// Crossing between the two resets it (and [`Unit::best_gap`] with it),
    /// since a value accrued under one reading means nothing under the other.
    pub hold_ticks: u8,
    /// Consecutive ticks moving in ally contact without shortening the
    /// remaining path by [`DETOUR_FRAC`] of a step; trips a lateral detour.
    pub ally_stall: u8,
    /// Best remaining path length seen since the ally-stall counter reset
    /// (`MAX` = unset).
    pub ally_min_remaining: f32,
}

/// [`Unit::chase_slot`] value meaning "no slot: walk at the target itself".
pub const NO_SLOT: u16 = u16::MAX;

impl Unit {
    pub fn is_moving(&self) -> bool {
        !self.path.is_empty()
    }

    /// Next waypoint while moving, own position when idle.
    pub fn waypoint(&self) -> Vector2 {
        match self.path.get(self.path_i as usize) {
            Some(&w) => w,
            None => self.pos,
        }
    }

    /// Polyline distance from the current position through all remaining
    /// waypoints — monotonically decreasing with real progress, so a flat value
    /// means the unit isn't advancing. Cheap: paths hold a handful of points.
    fn remaining_len(&self) -> f32 {
        let mut total = 0.0;
        let mut prev = self.pos;
        for &p in &self.path[self.path_i as usize..] {
            let (dx, dy) = (p.x - prev.x, p.y - prev.y);
            total += (dx * dx + dy * dy).sqrt();
            prev = p;
        }
        total
    }
}

/// Slot storage iterated in slot order — stable, hash-free determinism.
#[derive(Default)]
pub struct Units {
    slots: Vec<Option<Unit>>,
    generations: Vec<u32>,
    free: Vec<u32>,
    len: usize,
}

impl Units {
    pub fn spawn(&mut self, unit: Unit) -> UnitId {
        self.len += 1;
        match self.free.pop() {
            Some(i) => {
                self.slots[i as usize] = Some(unit);
                UnitId {
                    index: i,
                    generation: self.generations[i as usize],
                }
            }
            None => {
                self.slots.push(Some(unit));
                self.generations.push(0);
                UnitId {
                    index: (self.slots.len() - 1) as u32,
                    generation: 0,
                }
            }
        }
    }

    pub fn despawn(&mut self, id: UnitId) -> bool {
        match self.slots.get_mut(id.index as usize) {
            Some(slot @ Some(_)) if self.generations[id.index as usize] == id.generation => {
                *slot = None;
                self.generations[id.index as usize] += 1;
                self.free.push(id.index);
                self.len -= 1;
                true
            }
            _ => false,
        }
    }

    pub fn get(&self, id: UnitId) -> Option<&Unit> {
        match self.slots.get(id.index as usize) {
            Some(Some(u)) if self.generations[id.index as usize] == id.generation => Some(u),
            _ => None,
        }
    }

    pub fn get_mut(&mut self, id: UnitId) -> Option<&mut Unit> {
        match self.slots.get_mut(id.index as usize) {
            Some(Some(u)) if self.generations[id.index as usize] == id.generation => Some(u),
            _ => None,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn iter(&self) -> impl Iterator<Item = (UnitId, &Unit)> {
        self.slots.iter().enumerate().filter_map(|(i, s)| {
            s.as_ref().map(|u| {
                (
                    UnitId {
                        index: i as u32,
                        generation: self.generations[i],
                    },
                    u,
                )
            })
        })
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = (UnitId, &mut Unit)> {
        self.slots.iter_mut().enumerate().filter_map(|(i, s)| {
            s.as_mut().map(|u| {
                (
                    UnitId {
                        index: i as u32,
                        generation: self.generations[i],
                    },
                    u,
                )
            })
        })
    }
}

// ── Orders ────────────────────────────────────────────────────────────────────

/// A task a unit works through. Units hold a FIFO queue of these; the front one
/// drives behaviour until it completes, then the next begins (see
/// [`Sim::advance_orders`]). Extend with `HoldPosition`, … — each new
/// variant adds an arm to [`Sim::begin_order`] and to the order hash/snapshot.
#[derive(Clone, Debug, PartialEq)]
pub enum Order {
    Move {
        goal: Vector2,
    },
    /// Engage one specific unit (right-click on an enemy). Completes (leaves
    /// the attacker idle) when the target dies.
    Attack {
        target: UnitId,
    },
    /// Walk toward `goal`, engaging the first enemy acquired along the way,
    /// then resume marching once it dies.
    AttackMove {
        goal: Vector2,
    },
}

/// Diplomatic stance between two teams; see [`Sim::relation`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Relation {
    Ally,
    Enemy,
    Neutral,
}

// ── Commands ──────────────────────────────────────────────────────────────────

/// All sim mutation goes through commands, batched per tick — the replay,
/// determinism-test and lockstep seam.
#[derive(Clone, Debug)]
pub enum Command {
    Spawn {
        pos: Vector2,
        radius: f32,
        max_speed: f32,
        team: u32,
        max_health: f32,
        damage: f32,
        attack_range: f32,
        attack_cooldown_ticks: u32,
    },
    /// Move now: clears each unit's order queue and paths immediately (a plain
    /// right-click that interrupts whatever the unit was doing).
    Move {
        units: Vec<UnitId>,
        goal: Vector2,
    },
    /// Attack now: as `Move`, but engages `target` directly instead of
    /// pathing to a point.
    Attack {
        units: Vec<UnitId>,
        target: UnitId,
    },
    /// Attack-move now: as `Move`, but engages the first enemy acquired en
    /// route instead of marching straight through.
    AttackMove {
        units: Vec<UnitId>,
        goal: Vector2,
    },
    /// Append an order to each unit's queue (a shift-click). The general
    /// queue-append seam — every order type queues through here.
    Queue {
        units: Vec<UnitId>,
        order: Order,
    },
    AddObstacle {
        points: Vec<Vector2>,
    },
    RemoveObstacle {
        id: ObstacleId,
    },
    /// Override the default relation ("same team allied, different team
    /// enemy") between two teams; symmetric, and a no-op for `a == b`.
    SetRelation {
        team_a: u32,
        team_b: u32,
        relation: Relation,
    },
    /// Debug/test seam: damage a unit directly, bypassing targeting and
    /// range entirely. Goes through the same buffered-damage/despawn path
    /// combat uses, so death and despawn can be exercised without any
    /// combat logic running.
    Damage {
        unit: UnitId,
        amount: f32,
    },
}

// ── Spatial grid (separation broad-phase) ─────────────────────────────────────

/// Uniform grid over alive-unit positions, rebuilt each tick by counting sort
/// into flat persistent vecs (allocation-free in steady state).
#[derive(Default)]
struct SpatialGrid {
    origin: Vector2,
    inv_cell: f32,
    cols: u32,
    rows: u32,
    /// Prefix sums per cell into `entries`; length `cols * rows + 1`.
    starts: Vec<u32>,
    /// Dense-unit indices sorted by cell.
    entries: Vec<u32>,
    /// Scratch: cell index per dense unit.
    cell_of: Vec<u32>,
}

impl SpatialGrid {
    /// `cell_size` should be at least the largest unit diameter so all
    /// overlapping pairs sit in adjacent cells.
    fn rebuild(&mut self, positions: &[Vector2], cell_size: f32) {
        let n = positions.len();
        self.entries.clear();
        self.cell_of.clear();
        if n == 0 {
            self.cols = 0;
            self.rows = 0;
            return;
        }
        let (mut min, mut max) = (positions[0], positions[0]);
        for &p in positions {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
        }
        self.origin = min;
        self.inv_cell = 1.0 / cell_size;
        self.cols = (((max.x - min.x) * self.inv_cell) as u32 + 1).min(2048);
        self.rows = (((max.y - min.y) * self.inv_cell) as u32 + 1).min(2048);

        let cells = (self.cols * self.rows) as usize;
        self.starts.clear();
        self.starts.resize(cells + 1, 0);
        for &p in positions {
            let c = self.cell_index(p);
            self.cell_of.push(c);
            self.starts[c as usize + 1] += 1;
        }
        for i in 1..=cells {
            self.starts[i] += self.starts[i - 1];
        }
        self.entries.resize(n, 0);
        // Scatter via a moving cursor per cell; restore prefix sums after.
        for i in 0..n {
            let c = self.cell_of[i] as usize;
            self.entries[self.starts[c] as usize] = i as u32;
            self.starts[c] += 1;
        }
        for i in (1..=cells).rev() {
            self.starts[i] = self.starts[i - 1];
        }
        self.starts[0] = 0;
    }

    fn cell_coords(&self, p: Vector2) -> (u32, u32) {
        let cx = (((p.x - self.origin.x) * self.inv_cell) as u32).min(self.cols - 1);
        let cy = (((p.y - self.origin.y) * self.inv_cell) as u32).min(self.rows - 1);
        (cx, cy)
    }

    fn cell_index(&self, p: Vector2) -> u32 {
        let (cx, cy) = self.cell_coords(p);
        cy * self.cols + cx
    }

    fn cell_entries(&self, cx: u32, cy: u32) -> &[u32] {
        let c = (cy * self.cols + cx) as usize;
        let (a, b) = (self.starts[c] as usize, self.starts[c + 1] as usize);
        &self.entries[a..b]
    }
}

// ── FNV-1a hashing ────────────────────────────────────────────────────────────

struct Fnv(u64);

impl Fnv {
    fn new() -> Fnv {
        Fnv(FNV_OFFSET)
    }

    fn write_u64(&mut self, v: u64) {
        for b in v.to_le_bytes() {
            self.0 ^= b as u64;
            self.0 = self.0.wrapping_mul(FNV_PRIME);
        }
    }

    fn write_f32(&mut self, v: f32) {
        self.write_u64(v.to_bits() as u64);
    }

    fn write_v2(&mut self, v: Vector2) {
        self.write_u64((v.x.to_bits() as u64) << 32 | v.y.to_bits() as u64);
    }
}

// ── Sim ───────────────────────────────────────────────────────────────────────

/// Buffers reused across ticks so steady-state `step` never allocates.
#[derive(Default)]
struct StepScratch {
    /// Dense arrays over alive units, rebuilt each tick in slot order.
    ids: Vec<UnitId>,
    positions: Vec<Vector2>,
    radii: Vec<f32>,
    speeds: Vec<f32>,
    /// Separation+cohesion displacement accumulator, parallel to the dense arrays.
    disp: Vec<Vector2>,
    /// Per dense unit: its group, team, and whether it is moving / parked at goal.
    groups: Vec<u32>,
    teams: Vec<u32>,
    moving: Vec<bool>,
    parked: Vec<bool>,
    /// Whether this unit overlapped an *ally* during the flock pair pass — the
    /// "blocked by bodies, not walls" half of the detour trigger.
    ally_contact: Vec<bool>,
    /// Separation push received per unit this tick, in world units, split by
    /// ally and enemy. Summed as magnitudes, not vectors: equal shoves from
    /// both sides are two shoves, not none. Pre-cap, so it is what separation
    /// *asked* for (see [`SEPARATION_MAX_FRAC`]).
    ///
    /// Written during `flock` and never read back into sim state, so it is not
    /// part of [`Sim::state_hash`]. Empty unless [`PUSH_TRACKING`] is on.
    push_ally: Vec<f32>,
    push_enemy: Vec<f32>,
    /// Whether a moving unit is within its arrival radius of its goal (so it
    /// may crowd-stop); false for idle/parked units.
    within_arrival: Vec<bool>,
    /// Crowd-arrival marks: set when a moving unit touches a parked group-mate.
    arrive: Vec<bool>,
    /// Sum of same-group moving-neighbour positions, and their count, per unit.
    coh_sum: Vec<Vector2>,
    coh_n: Vec<u32>,
    /// Goal (last waypoint) per dense unit; junk for non-moving units, only read
    /// when both sides of a pair are moving (merge detection).
    goals: Vec<Vector2>,
    /// Group-id pairs to merge this tick, as `(min, max)`; usually empty.
    merge_pairs: Vec<(u32, u32)>,
    /// Wall-clamp face frontier and visited list (tiny per unit).
    faces: Vec<u32>,
    visited: Vec<u32>,
    /// Wall-clamp: constrained half-edges near the unit this tick (gate
    /// prefilter, then per-pass discoveries; tiny per unit).
    clamp_walls: Vec<u32>,
    /// Units the wall clamp — or a chase-path rebuild — found needing a
    /// repath this tick (usually empty); both feed the same repath pass.
    repath: Vec<UnitId>,
    /// Combat: units that hit their leash this tick and the post to walk back
    /// to. Almost always empty.
    leash_home: Vec<(UnitId, Vector2)>,
    /// Combat: `(target, slot code)` chosen so far this tick, so units
    /// deciding on the same tick spread around the ring instead of all
    /// reading the same free slot. Slot-order, so it stays deterministic.
    slot_claims: Vec<(UnitId, u16)>,
    /// Combat: allies already targeting each dense unit, for acquisition
    /// spreading. Rebuilt per tick in [`Sim::acquire_targets`].
    target_count: Vec<u32>,
    /// Combat: acquisition results this tick, parallel (attacker, target).
    /// Almost always empty — most ticks acquire nothing.
    acquired_by: Vec<UnitId>,
    acquired_target: Vec<UnitId>,
    /// Combat: fire/chase decisions read against pre-tick state, applied in a
    /// second pass (looking up a unit's target needs a second live borrow of
    /// `units`, so decide-then-apply avoids aliasing).
    combat_decisions: Vec<CombatDecision>,
    /// Combat: buffered (target, damage) pairs from this tick's fire
    /// resolution, applied after every unit has acted so two units that kill
    /// each other the same tick both die (no attacker gets a slot-order edge).
    damage: Vec<(UnitId, f32)>,
    /// Combat: units whose health reached zero this tick, in slot order.
    dead: Vec<UnitId>,
}

/// One unit's combat decision for the tick, computed read-only against
/// pre-tick state in [`Sim::engage`] and applied afterward.
///
/// `Copy` so the apply pass can lift one out of `step_scratch` and still write
/// through `units` and push to `step_scratch.damage`/`repath`.
#[derive(Clone, Copy)]
struct CombatDecision {
    id: UnitId,
    /// Hold station: stop here and keep the target under the weapon.
    station: bool,
    /// Target is genuinely inside `attack_range` this tick. Damage needs
    /// both; `station` alone only stops the unit and ticks its cooldown.
    in_range: bool,
    /// Target position at decision time; only meaningful when `!fire`.
    target_pos: Vector2,
    /// Where the unit is walking: its approach slot, or the target itself.
    /// Only meaningful when `!fire`.
    goal: Vector2,
    /// Approach slot chosen this tick ([`NO_SLOT`] = walk at the target).
    slot: u16,
    /// Only meaningful when `!fire`: whether the chase path needs rebuilding.
    need_repath: bool,
    /// Whether the re-scoring cadence came due this tick (it restarts even
    /// when nothing needed rebuilding, so a held slot is re-scored on a fixed
    /// period rather than every tick).
    cadence: bool,
    /// Block-progress bookkeeping, carried out of the read-only pass; see
    /// [`Unit::hold_ticks`] for which of its two readings applies.
    best_gap: f32,
    hold_ticks: u8,
}

pub struct Sim {
    tick: u64,
    nav: DynamicNavmesh,
    /// Always matches `nav.navmesh().version()` after construction/rebuild.
    abstraction: Abstraction,
    abstraction_version: u64,
    units: Units,
    scratch: AStarScratch,
    rng: Pcg32,
    grid: SpatialGrid,
    step_scratch: StepScratch,
    /// Monotonic group id; bumped per `Move`, stamped onto its units.
    group_seq: u32,
    /// Team-relation overrides, keyed `(min(a,b), max(a,b))`. Point-queried
    /// only (never iterated for a result), so insertion order doesn't affect
    /// lookups — only `state_hash`, where it's read back in that same order.
    /// Small (one entry per diplomacy change), so a linear scan beats a map.
    relations: Vec<((u32, u32), Relation)>,
}

impl Sim {
    /// Build a sim over static map geometry (`constraints` index into `points`).
    pub fn new(points: Vec<Vector2>, constraints: &[(u32, u32)], seed: u64) -> Sim {
        let nav = DynamicNavmesh::new(points, constraints);
        let abstraction = Abstraction::build(nav.navmesh());
        let abstraction_version = nav.navmesh().version();
        Sim {
            tick: 0,
            nav,
            abstraction,
            abstraction_version,
            units: Units::default(),
            scratch: AStarScratch::new(),
            rng: Pcg32::new(seed),
            grid: SpatialGrid::default(),
            step_scratch: StepScratch::default(),
            group_seq: 0,
            relations: Vec::new(),
        }
    }

    pub fn tick(&self) -> u64 {
        self.tick
    }

    pub fn units(&self) -> &Units {
        &self.units
    }

    /// Diplomatic stance between two teams: an explicit override if one was
    /// set via [`Command::SetRelation`], else the default rule (same team
    /// allied, different team enemy).
    pub fn relation(&self, a: u32, b: u32) -> Relation {
        relation_of(&self.relations, a, b)
    }

    fn set_relation(&mut self, a: u32, b: u32, relation: Relation) {
        if a == b {
            return; // same-team relation is fixed Ally; ignore
        }
        let key = if a < b { (a, b) } else { (b, a) };
        match self.relations.iter_mut().find(|(k, _)| *k == key) {
            Some(entry) => entry.1 = relation,
            None => self.relations.push((key, relation)),
        }
    }

    pub fn navmesh(&self) -> &CDT {
        self.nav.navmesh()
    }

    /// Separation push each unit received on the most recent [`Sim::step`], as
    /// `(id, from_allies, from_enemies)` in world units.
    ///
    /// A pure diagnostic: the shove separation asked for, before the per-tick
    /// cap, summed as magnitudes so opposing shoves add rather than cancel.
    /// Empty before the first step.
    pub fn last_push(&self) -> impl Iterator<Item = (UnitId, f32, f32)> + '_ {
        let s = &self.step_scratch;
        s.ids
            .iter()
            .zip(&s.push_ally)
            .zip(&s.push_enemy)
            .map(|((&id, &a), &e)| (id, a, e))
    }

    /// Id the next `AddObstacle` command will assign.
    pub fn next_obstacle_id(&self) -> u64 {
        self.nav.next_obstacle_id()
    }

    /// Advance one fixed tick, applying all commands queued since the last.
    ///
    /// Order: store prev positions → apply commands → rebuild navmesh if the
    /// obstacle set changed (refresh abstraction, repath all moving units) →
    /// integrate along paths → flock (separation + cohesion) → ally detours →
    /// combat
    /// (acquire, chase or fire, apply damage, despawn dead) → start the next
    /// queued order for any unit that just finished → wall clamp → repath units
    /// stuck against a corner or mid-chase → advance tick.
    pub fn step(&mut self, commands: &[Command]) {
        for (_, u) in self.units.iter_mut() {
            u.prev_pos = u.pos;
        }
        for cmd in commands {
            self.apply(cmd);
        }
        let mesh_changed = self.rebuild_and_repath();
        self.integrate();
        self.flock();
        self.detour();
        self.combat();
        self.advance_orders();
        self.wall_clamp(mesh_changed);
        self.repath_stalled();
        self.tick += 1;
    }

    fn apply(&mut self, cmd: &Command) {
        match cmd {
            Command::Spawn {
                pos,
                radius,
                max_speed,
                team,
                max_health,
                damage,
                attack_range,
                attack_cooldown_ticks,
            } => {
                self.units.spawn(Unit {
                    pos: *pos,
                    prev_pos: *pos,
                    radius: *radius,
                    max_speed: *max_speed,
                    path: Vec::new(),
                    path_i: 0,
                    orders: VecDeque::new(),
                    group: 0,
                    parked: false,
                    arrival_r: 0.0,
                    stall: 0,
                    min_remaining: f32::MAX,
                    team: *team,
                    max_health: *max_health,
                    health: *max_health,
                    damage: *damage,
                    // Negative range would make `engage`'s range check unsatisfiable forever.
                    attack_range: attack_range.max(0.0),
                    // A mis-authored zero cooldown would fire every tick
                    // forever; clamp instead of trusting the caller.
                    attack_cooldown_ticks: (*attack_cooldown_ticks).max(1),
                    cooldown_left: 0,
                    target: None,
                    attack_move_goal: None,
                    chase_anchor: Vector2::ZERO,
                    chase_repath_in: 0,
                    post: None,
                    target_commanded: false,
                    engaged: false,
                    chase_slot: NO_SLOT,
                    best_gap: f32::MAX,
                    hold_ticks: 0,
                    ally_stall: 0,
                    ally_min_remaining: f32::MAX,
                });
            }
            Command::Move { units, goal } => {
                // Plain move interrupts: drop any queued orders, path now.
                self.interrupt_orders(units);
                self.start_move(units, *goal);
            }
            Command::Attack { units, target } => {
                self.interrupt_orders(units);
                self.start_attack(units, *target);
            }
            Command::AttackMove { units, goal } => {
                self.interrupt_orders(units);
                self.start_attack_move(units, *goal);
            }
            Command::Queue { units, order } => {
                for &id in units {
                    if let Some(u) = self.units.get_mut(id) {
                        u.orders.push_back(order.clone());
                    }
                }
            }
            Command::AddObstacle { points } => {
                self.nav.add_obstacle(Obstacle::polygon(points.clone()));
            }
            Command::RemoveObstacle { id } => {
                self.nav.remove_obstacle(*id);
            }
            Command::SetRelation {
                team_a,
                team_b,
                relation,
            } => {
                self.set_relation(*team_a, *team_b, *relation);
            }
            Command::Damage { unit, amount } => {
                self.step_scratch.damage.push((*unit, *amount));
            }
        }
    }

    /// Drop each unit's queued orders — the shared first step of every
    /// "now" command (`Move`/`Attack`/`AttackMove`), which interrupts
    /// whatever was queued behind the previous order.
    fn interrupt_orders(&mut self, units: &[UnitId]) {
        for &id in units {
            if let Some(u) = self.units.get_mut(id) {
                u.orders.clear();
            }
        }
    }

    /// Execute a move now: partition the live selection into spatial flocks, give
    /// each its own group id and one shared channel, then string-pull that
    /// channel per unit so the flock spreads across corridor width instead of
    /// single-filing the inside corner. A unit whose first leg into the channel
    /// fails line-of-sight (straggler / no useful channel) paths individually.
    fn start_move(&mut self, units: &[UnitId], goal: Vector2) {
        // Live selected units, slot order (stable, deterministic). A fresh
        // move interrupts any combat engagement, same as it clears a path.
        let mut sel: Vec<UnitId> = Vec::new();
        for &id in units {
            if let Some(u) = self.units.get_mut(id) {
                u.target = None;
                u.attack_move_goal = None;
                clear_combat_state(u);
                sel.push(id);
            }
        }
        let n = sel.len();
        if n == 0 {
            return;
        }
        let mut pos: Vec<Vector2> = Vec::with_capacity(n);
        let mut rad: Vec<f32> = Vec::with_capacity(n);
        for &id in &sel {
            let u = self.units.get(id).expect("filtered to live");
            pos.push(u.pos);
            rad.push(u.radius);
        }
        let max_radius = rad.iter().copied().fold(0.0f32, f32::max);
        let r_coh = max_radius * COHESION_RADIUS_FRAC.get();

        // Connected components under "same radius, within R_COH and clear
        // line-of-sight", via the spatial grid. Keying on radius lets each unit
        // size route by its own clearance, so a smaller unit can take a narrower,
        // shorter corridor the larger ones can't. Union-find over dense indices.
        let cdt = self.nav.navmesh();
        let mut parent: Vec<u32> = (0..n as u32).collect();
        let cell = r_coh.max(max_radius * 2.0).max(1.0);
        self.grid.rebuild(&pos, cell);
        for i in 0..n {
            let (cx, cy) = self.grid.cell_coords(pos[i]);
            for ny in cy.saturating_sub(1)..=(cy + 1).min(self.grid.rows - 1) {
                for nx in cx.saturating_sub(1)..=(cx + 1).min(self.grid.cols - 1) {
                    for &j in self.grid.cell_entries(nx, ny) {
                        let j = j as usize;
                        if j <= i {
                            continue;
                        }
                        if rad[i] != rad[j] {
                            continue; // different sizes route as separate flocks
                        }
                        let d = pos[i] - pos[j];
                        if d.x * d.x + d.y * d.y > r_coh * r_coh {
                            continue;
                        }
                        if uf_find(&mut parent, i as u32) == uf_find(&mut parent, j as u32) {
                            continue;
                        }
                        if clear_los(cdt, pos[i], pos[j], rad[i]) {
                            uf_union(&mut parent, i as u32, j as u32);
                        }
                    }
                }
            }
        }

        // Fresh group id per component, first-appearance (slot) order — determinism.
        let mut comps: Vec<(u32, Vec<usize>)> = Vec::new();
        let mut roots: Vec<(u32, usize)> = Vec::new(); // (root, comps index)
        for i in 0..n {
            let r = uf_find(&mut parent, i as u32);
            match roots.iter().find(|&&(rr, _)| rr == r) {
                Some(&(_, ci)) => comps[ci].1.push(i),
                None => {
                    self.group_seq = self.group_seq.wrapping_add(1).max(1);
                    roots.push((r, comps.len()));
                    comps.push((self.group_seq, vec![i]));
                }
            }
        }

        // Reusable per-corner geometry of the seed's funnel path.
        let mut corners: Vec<Vector2> = Vec::new();
        let mut outward: Vec<Vector2> = Vec::new();
        let mut lanes: Vec<f32> = Vec::new();
        for (group, members) in &comps {
            let size = members.len();
            // All members share one radius (clustering keys on it), so the flock's
            // channel fits them and smaller units cluster — and route — separately.
            let flock_r = rad[members[0]];
            let arrival_mult =
                (ARRIVAL_RADIUS_FACTOR.get() * (size as f32).sqrt()).max(ARRIVAL_MIN_RADII.get());
            // One shortest (funnel) path for the flock, seeded from the member
            // nearest the goal (a real unit position is on the navmesh). Its
            // interior waypoints are the corner apexes every unit would otherwise
            // single-file through.
            let seed = *members
                .iter()
                .min_by(|&&a, &&b| {
                    let da = pos[a] - goal;
                    let db = pos[b] - goal;
                    let (da, db) = (da.x * da.x + da.y * da.y, db.x * db.x + db.y * db.y);
                    da.partial_cmp(&db).unwrap()
                })
                .expect("non-empty component");
            let seed_path = find_path_abstract(
                cdt,
                &self.abstraction,
                pos[seed],
                goal,
                &mut self.scratch,
                flock_r,
            );

            // Empty seed path ⇒ goal unreachable for the seed; since cluster members
            // share clear LoS (same navmesh component) it's unreachable for all —
            // idle the whole cluster. Skipping this would fall through to the
            // straight-shot branch, which can't tell "unreachable" from "direct
            // clear shot": it would synthesise a converge waypoint on the reachable
            // side and route_onto_channel would append the unreachable goal leg,
            // handing non-seed units a path that pokes across the wall.
            if seed_path.is_empty() {
                for &i in members {
                    let unit = self.units.get_mut(sel[i]).expect("filtered to live");
                    unit.group = *group;
                    unit.arrival_r = rad[i] * arrival_mult;
                    set_path(unit, Vec::new());
                }
                continue;
            }

            // Corner apexes (drop start and goal) and the outward direction at
            // each — the external bisector, pointing into the bend's free side
            // (away from the wall vertex the apex hugs). Units fan along it.
            corners.clear();
            outward.clear();
            if seed_path.len() > 2 {
                for w in seed_path.windows(3) {
                    let (prev, cur, next) = (w[0], w[1], w[2]);
                    corners.push(cur);
                    outward.push(external_bisector(prev, cur, next));
                }
            }

            // No corners ⇒ straight shot: synthesise one waypoint near the goal,
            // offset along the perpendicular axis, so each unit holds its lateral
            // lane until the final approach and only converges over the last leg
            // (the blob zone). It sits a converge distance (≈ arrival radius,
            // capped to half the path) back from the goal. Fanning is symmetric
            // (both sides), unlike a bend's one-sided fan into the free side.
            let straight = corners.is_empty() && {
                let dir = goal - pos[seed];
                let len2 = dir.x * dir.x + dir.y * dir.y;
                if len2 > 1e-6 {
                    let len = len2.sqrt();
                    let converge = (flock_r * arrival_mult).min(len * 0.5);
                    corners.push(goal - dir * (converge / len));
                    outward.push(Vector2::new(-dir.y, dir.x) * (1.0 / len));
                    true
                } else {
                    false
                }
            };

            // Lane = lateral offset from the flock along the fan axis. At a bend,
            // shift so the most wall-ward unit sits at the apex (offset 0) and the
            // rest fan into the free side; on a straight leg, keep the sign so
            // they fan symmetrically and hold their spread.
            let has_corners = !corners.is_empty();
            let axis = if has_corners {
                outward[0]
            } else {
                Vector2::ZERO
            };
            lanes.clear();
            let mut lane_min = f32::INFINITY;
            for &i in members {
                let d = pos[i] - pos[seed];
                let l = d.x * axis.x + d.y * axis.y;
                lanes.push(l);
                lane_min = lane_min.min(l);
            }

            for (k, &i) in members.iter().enumerate() {
                let radius = rad[i];
                // The seed already has its shortest path (offset 0, same start);
                // reuse it (this also covers singleton clusters with no fan).
                let path = if i == seed {
                    seed_path.clone()
                } else {
                    let offset = if straight {
                        // Hold most of the marching spread, relaxed a touch.
                        lanes[k] * STRAIGHT_FAN_FRAC.get()
                    } else if has_corners {
                        (lanes[k] - lane_min) * FAN_FRAC.get()
                    } else {
                        0.0
                    };
                    build_offset_path(cdt, pos[i], &corners, &outward, goal, offset, radius)
                        .unwrap_or_else(|| {
                            route_onto_channel(
                                cdt,
                                &self.abstraction,
                                &mut self.scratch,
                                pos[i],
                                &corners,
                                goal,
                                radius,
                            )
                        })
                };
                let unit = self.units.get_mut(sel[i]).expect("filtered to live");
                unit.group = *group;
                unit.arrival_r = radius * arrival_mult;
                set_path(unit, path);
            }
        }
    }

    /// Begin one order for a batch of units, dispatching on its type. The
    /// execution seam for queued orders — new `Order` variants add an arm here.
    fn begin_order(&mut self, units: &[UnitId], order: &Order) {
        match order {
            Order::Move { goal } => self.start_move(units, *goal),
            Order::Attack { target } => self.start_attack(units, *target),
            Order::AttackMove { goal } => self.start_attack_move(units, *goal),
        }
    }

    /// Begin engaging `target` directly: no acquisition, no march goal — the
    /// order completes (leaving the attacker idle) once `target` dies. Chase
    /// pathing is deferred to [`Sim::engage`]/[`Sim::repath_stalled`] like any
    /// other combat path, so this just records the target and idles the path.
    fn start_attack(&mut self, units: &[UnitId], target: UnitId) {
        // Bail on a dead target (resolve_deaths only clears stale targets on
        // ticks where something dies, so latching on here would soft-lock).
        let Some(target_team) = self.units.get(target).map(|t| t.team) else {
            return;
        };
        for &id in units {
            let Some(unit) = self.units.get_mut(id) else {
                continue;
            };
            // Same-team relation is fixed `Ally` (see `relation_of`), so this
            // also rejects `id == target`: a unit can't be its own enemy.
            if relation_of(&self.relations, unit.team, target_team) != Relation::Enemy {
                continue;
            }
            unit.target = Some(target);
            unit.attack_move_goal = None;
            unit.group = 0;
            unit.path.clear();
            unit.path_i = 0;
            unit.parked = false;
            clear_combat_state(unit);
            unit.target_commanded = true; // the player's kill choice, not the unit's
        }
    }

    /// Begin marching toward `goal`, engaging the first enemy acquired along
    /// the way. Reuses `start_move` for the march itself (clustering, group
    /// id, channel pathing) — group ids stay per `Move`; acquisition later
    /// breaks a unit out of its march individually rather than re-grouping.
    fn start_attack_move(&mut self, units: &[UnitId], goal: Vector2) {
        self.start_move(units, goal);
        for &id in units {
            if let Some(unit) = self.units.get_mut(id) {
                unit.attack_move_goal = Some(goal);
            }
        }
    }

    /// Start the next queued order for every unit that just finished its current
    /// one (now idle with a non-empty queue). Units finishing the same tick with
    /// an *identical* front order begin as one batch, so a group re-clusters and
    /// re-channels around the next waypoint instead of single-filing it;
    /// stragglers finishing a tick later re-merge via the flock merge pass.
    /// Deterministic: slot-order scan, exact-order batching, no map iteration.
    ///
    /// A unit with a live combat target isn't idle even with an empty path —
    /// firing holds it stationary — so an ordered fight also excludes it, or a
    /// queued order behind an `Attack`/`AttackMove` would cut the fight short
    /// the moment it started firing instead of waiting for a kill.
    ///
    /// A target the unit picked up *itself* while idle (self defence) does not
    /// block the queue: the order is what the player asked for, and a unit
    /// that shoots back at whatever wanders past would otherwise sit on its
    /// orders indefinitely.
    fn advance_orders(&mut self) {
        let mut batches: Vec<(Order, Vec<UnitId>)> = Vec::new();
        for (id, u) in self.units.iter() {
            let ordered_fight =
                u.target.is_some() && (u.target_commanded || u.attack_move_goal.is_some());
            if u.is_moving() || ordered_fight {
                continue;
            }
            let Some(order) = u.orders.front() else {
                continue;
            };
            match batches.iter_mut().find(|(o, _)| o == order) {
                Some((_, members)) => members.push(id),
                None => batches.push((order.clone(), vec![id])),
            }
        }
        for (order, members) in &batches {
            for &id in members {
                self.units
                    .get_mut(id)
                    .expect("batched id alive")
                    .orders
                    .pop_front();
            }
            self.begin_order(members, order);
        }
    }

    /// Rebuild the navmesh after obstacle changes; on a version move, refresh
    /// the abstraction and repath every moving unit toward its existing goal
    /// (phase-1 policy: repath all, no hysteresis). Returns whether the mesh
    /// actually changed.
    fn rebuild_and_repath(&mut self) -> bool {
        if !self.nav.is_dirty() {
            return false;
        }
        let cdt = self.nav.rebuild();
        if cdt.version() == self.abstraction_version {
            return false;
        }
        self.abstraction = Abstraction::build(cdt);
        self.abstraction_version = cdt.version();
        for (_, unit) in self.units.iter_mut() {
            let Some(&goal) = unit.path.last() else {
                continue;
            };
            let path = find_path_abstract(
                cdt,
                &self.abstraction,
                unit.pos,
                goal,
                &mut self.scratch,
                unit.radius,
            );
            set_path(unit, path);
        }
        true
    }

    /// Advance each moving unit `max_speed * DT` along its polyline, carrying
    /// leftover distance across waypoints; snap and idle at the end.
    ///
    /// Each unit first **re-anchors**: it skips any waypoint already rounded, so
    /// a crowd shove that pushes it *through* one doesn't make it double back.
    /// A waypoint counts as rounded once the unit is past its gate (on the next
    /// leg's side) *and* has clear line-of-sight to the following waypoint — the
    /// LoS test is what stops it cutting an unrounded corner.
    fn integrate(&mut self) {
        let cdt = self.nav.navmesh();
        for (_, unit) in self.units.iter_mut() {
            if unit.path.is_empty() {
                continue;
            }
            while (unit.path_i as usize) + 1 < unit.path.len() {
                let i = unit.path_i as usize;
                let (w, nxt) = (unit.path[i], unit.path[i + 1]);
                let (d, seg) = (unit.pos - w, nxt - w);
                if d.x * seg.x + d.y * seg.y >= 0.0 && clear_los(cdt, unit.pos, nxt, unit.radius) {
                    unit.path_i += 1;
                } else {
                    break;
                }
            }
            let mut remaining = unit.max_speed * DT;
            while remaining > 0.0 {
                // Waypoints exhausted: the post-loop guard clears the path.
                let Some(&target) = unit.path.get(unit.path_i as usize) else {
                    break;
                };
                let delta = target - unit.pos;
                let dist = (delta.x * delta.x + delta.y * delta.y).sqrt();
                if dist <= remaining {
                    unit.pos = target;
                    unit.path_i += 1;
                    remaining -= dist;
                } else {
                    unit.pos += delta * (remaining / dist);
                    break;
                }
            }
            if unit.path_i as usize >= unit.path.len() {
                if unit.target.is_some() {
                    // Reached an approach slot, not a march goal. Combat keeps
                    // its own hold state (`engaged`, `chase_slot`), and parking
                    // here would seed crowd-arrival for same-group units — an
                    // attack-moving rear rank would stop at an arbitrary
                    // distance from the enemy, since `arrival_r` is sized for
                    // the march, not for a fight.
                    unit.path.clear();
                    unit.path_i = 0;
                } else {
                    arrive_at_goal(unit);
                }
            }
        }
    }

    /// One grid pass over start-of-tick positions, deriving three per-unit
    /// effects applied afterwards (fixed order = bit-exact):
    ///
    /// - **Separation** (all neighbours, at `r_i + r_j`): corrects
    ///   [`SEPARATION_RELAX`] of each overlap per tick, split between the pair
    ///   by [`HOLD_WEIGHT`] — a mover yields to a unit that is holding station
    ///   (firing, parked or idle) instead of bulldozing it across the map.
    /// - **Cohesion** (same-group moving neighbours with no target, at
    ///   [`COHESION_RADIUS_FRAC`]
    ///   radii): pulls toward the neighbour centroid by [`COHESION_GAIN`], with
    ///   any heading-opposing component dropped so stragglers rejoin but
    ///   leaders are never braked.
    /// - **Crowd-arrival** (same-group, at [`ARRIVAL_TOUCH_FRAC`] of touching):
    ///   a moving unit touching a *parked* group-mate parks too, so a group
    ///   settles into a blob at its goal rather than each unit crushing toward
    ///   the exact goal point.
    ///
    /// The combined displacement is capped at [`SEPARATION_MAX_FRAC`] of the
    /// unit's own step, well below the path advance, so pathing always wins.
    fn flock(&mut self) {
        let s = &mut self.step_scratch;
        s.ids.clear();
        s.positions.clear();
        s.radii.clear();
        s.speeds.clear();
        s.disp.clear();
        s.groups.clear();
        s.teams.clear();
        s.moving.clear();
        s.parked.clear();
        s.ally_contact.clear();
        s.push_ally.clear();
        s.push_enemy.clear();
        s.within_arrival.clear();
        s.arrive.clear();
        s.coh_sum.clear();
        s.coh_n.clear();
        s.goals.clear();
        s.merge_pairs.clear();
        let mut max_radius = 0.0f32;
        for (id, u) in self.units.iter() {
            s.ids.push(id);
            s.positions.push(u.prev_pos);
            s.radii.push(u.radius);
            s.speeds.push(u.max_speed);
            s.disp.push(Vector2::ZERO);
            s.groups.push(u.group);
            s.teams.push(u.team);
            s.moving.push(u.is_moving());
            s.parked.push(u.parked);
            s.ally_contact.push(false);
            let within = match u.path.last() {
                Some(&g) if u.arrival_r > 0.0 => {
                    let (dx, dy) = (u.pos.x - g.x, u.pos.y - g.y);
                    dx * dx + dy * dy < u.arrival_r * u.arrival_r
                }
                _ => false,
            };
            s.within_arrival.push(within);
            s.arrive.push(false);
            s.coh_sum.push(Vector2::ZERO);
            s.coh_n.push(0);
            s.goals
                .push(u.path.last().copied().unwrap_or(Vector2::ZERO));
            max_radius = max_radius.max(u.radius);
        }
        if s.ids.len() < 2 || max_radius <= 0.0 {
            return;
        }
        let max_diameter = max_radius * 2.0;
        let r_coh = max_radius * COHESION_RADIUS_FRAC.get();
        let r_coh2 = r_coh * r_coh;
        // Cell covers the larger radius so the 3×3 scan still finds every pair.
        self.grid.rebuild(&s.positions, max_diameter.max(r_coh));
        let cdt = self.nav.navmesh();
        let separation_relax = SEPARATION_RELAX.get();
        // Sized here, not pushed per unit above, so the default path does no
        // per-unit work.
        let track_push = PUSH_TRACKING.load(std::sync::atomic::Ordering::Relaxed);
        if track_push {
            s.push_ally.resize(s.ids.len(), 0.0);
            s.push_enemy.resize(s.ids.len(), 0.0);
        }
        let arrival_touch_frac = ARRIVAL_TOUCH_FRAC.get();
        let hold_weight = HOLD_WEIGHT.get();

        for i in 0..s.ids.len() {
            let p = s.positions[i];
            let r_i = s.radii[i];
            let g_i = s.groups[i];
            let t_i = s.teams[i];
            let mv_i = s.moving[i];
            let pk_i = s.parked[i];
            let (cx, cy) = self.grid.cell_coords(p);
            for ny in cy.saturating_sub(1)..=(cy + 1).min(self.grid.rows - 1) {
                for nx in cx.saturating_sub(1)..=(cx + 1).min(self.grid.cols - 1) {
                    for &j in self.grid.cell_entries(nx, ny) {
                        let j = j as usize;
                        if j <= i {
                            continue;
                        }
                        let delta = p - s.positions[j];
                        let d2 = delta.x * delta.x + delta.y * delta.y;
                        let min_dist = r_i + s.radii[j];
                        if d2 < min_dist * min_dist {
                            let d = d2.sqrt();
                            // Coincident circles: deterministic x-axis tiebreak.
                            let dir = if d > 1e-6 {
                                delta * (1.0 / d)
                            } else {
                                Vector2::new(1.0, 0.0)
                            };
                            // Asymmetric split: each side takes the *other*'s
                            // weight share, so the mover absorbs most of the
                            // correction. Equal weights give exactly 0.5 each
                            // (`w / (w + w)` is exact in f32), which is why a
                            // packing at rest is bit-for-bit unaffected.
                            let (w_i, w_j) = (
                                if mv_i { 1.0 } else { hold_weight },
                                if s.moving[j] { 1.0 } else { hold_weight },
                            );
                            let sum = w_i + w_j;
                            let overlap = min_dist - d;
                            let push_i = overlap * (w_j / sum) * separation_relax;
                            let push_j = overlap * (w_i / sum) * separation_relax;
                            s.disp[i] += dir * push_i;
                            s.disp[j] -= dir * push_j;
                            // Body contact with an ally: the detour's "blocked
                            // by units, not walls" precondition. The same test
                            // buckets the diagnostic push, so the split costs
                            // no extra relation lookup.
                            let ally = relation_of(&self.relations, t_i, s.teams[j])
                                == Relation::Ally;
                            if ally {
                                s.ally_contact[i] = true;
                                s.ally_contact[j] = true;
                            }
                            if track_push {
                                if ally {
                                    s.push_ally[i] += push_i;
                                    s.push_ally[j] += push_j;
                                } else {
                                    s.push_enemy[i] += push_i;
                                    s.push_enemy[j] += push_j;
                                }
                            }
                        }
                        // Merge: adjacent, both-moving, same-goal, same-size units
                        // from *different* flocks continue as one. R_COH + clear-LoS
                        // adjacency excludes flocks that are close but wall-separated;
                        // exact-goal match is the deterministic "commanded together"
                        // test; same radius keeps differently-routed flocks apart.
                        // Recorded as (min, max), unioned after the pass (v1).
                        let g_j = s.groups[j];
                        if g_i != 0
                            && g_j != 0
                            && g_i != g_j
                            && mv_i
                            && s.moving[j]
                            && r_i == s.radii[j]
                            && d2 < r_coh2
                            && s.goals[i] == s.goals[j]
                            && clear_los(cdt, p, s.positions[j], r_i)
                        {
                            let pair = if g_i < g_j { (g_i, g_j) } else { (g_j, g_i) };
                            s.merge_pairs.push(pair);
                        }
                        if g_i == 0 || g_i != g_j {
                            continue; // remaining effects are same-group only
                        }
                        // Cohesion between moving group-mates: store the offset
                        // (neighbor - self) so the apply loop can pull directly
                        // without re-subtracting own position.
                        if mv_i && s.moving[j] && d2 < r_coh2 {
                            s.coh_sum[i] += s.positions[j] - p;
                            s.coh_sum[j] += p - s.positions[j];
                            s.coh_n[i] += 1;
                            s.coh_n[j] += 1;
                        }
                        // Crowd-arrival: a moving unit within arrival radius that
                        // touches a parked group-mate parks too (whichever side is
                        // moving). The radius gate lets units still far out keep
                        // pushing in, so the blob centres rather than tailing back.
                        let touch = min_dist * arrival_touch_frac;
                        if d2 < touch * touch {
                            if mv_i && s.within_arrival[i] && s.parked[j] {
                                s.arrive[i] = true;
                            } else if pk_i && s.moving[j] && s.within_arrival[j] {
                                s.arrive[j] = true;
                            }
                        }
                    }
                }
            }
        }

        for i in 0..s.ids.len() {
            let unit = self.units.get_mut(s.ids[i]).expect("dense id alive");
            if s.arrive[i] {
                // Stop where it is, against the cluster — don't drive to
                // centre. Crowd-arrival also completes the march order.
                arrive_at_goal(unit);
            }
            let mut d = s.disp[i];
            // Cohesion is off while engaged: a persistent inward pull applied
            // exactly when attackers should be fanning out around a target.
            if !s.arrive[i] && s.coh_n[i] > 0 && unit.target.is_none() {
                let mut pull = s.coh_sum[i] * (COHESION_GAIN.get() / s.coh_n[i] as f32);
                // Zero any pull opposing the heading: rejoin laterally/forward,
                // never brake. coh_n[i] > 0 implies the unit is moving.
                let h = unit.waypoint() - unit.pos;
                let h2 = h.x * h.x + h.y * h.y;
                if h2 > 1e-12 {
                    let along = (pull.x * h.x + pull.y * h.y) / h2;
                    if along < 0.0 {
                        pull -= h * along;
                    }
                }
                // Cap cohesion on its own (small) budget so it can't overpower
                // separation and pull group-mates back into overlap.
                let coh_cap = s.speeds[i] * DT * COHESION_MAX_FRAC.get();
                let pl2 = pull.x * pull.x + pull.y * pull.y;
                if pl2 > coh_cap * coh_cap {
                    pull *= coh_cap / pl2.sqrt();
                }
                d += pull;
            }
            let len2 = d.x * d.x + d.y * d.y;
            if len2 == 0.0 {
                continue;
            }
            let max_step = s.speeds[i] * DT * SEPARATION_MAX_FRAC.get();
            let len = len2.sqrt();
            if len > max_step {
                d *= max_step / len;
            }
            // Anti-tunnel: a flock push is a raw position add with no path/LoS
            // guarantee, so it can shove a unit clean across a constrained edge —
            // wall_clamp only inspects the final position and would then amplify
            // the tunnel (projecting the centre out the *wrong* side). Bites when
            // the goal sits near a wall; clip the move to stop at it instead.
            unit.pos = clip_ray_to_walls(cdt, unit.pos, unit.pos + d);
        }

        if !self.step_scratch.merge_pairs.is_empty() {
            self.apply_merges();
        }
    }

    /// Relabel converging flocks recorded this tick (`flock`'s merge pass) onto
    /// one group id: union the `(min, max)` pairs and rewrite each member to its
    /// canonical (min) id. Each merged unit keeps its own already-valid path —
    /// only its group changes, so it coheres / crowd-arrives with the joined
    /// flock from next tick (v1; no re-channel). Deterministic: sorted pairs,
    /// min canonicalisation, slot-order relabel, no map iteration.
    fn apply_merges(&mut self) {
        use std::collections::HashMap;
        let pairs = &mut self.step_scratch.merge_pairs;
        pairs.sort_unstable();
        pairs.dedup();

        // Union-find over the (few) involved group ids. The map is only ever
        // point-queried, never iterated, so its order can't affect results.
        let mut parent: HashMap<u32, u32> = HashMap::new();
        fn find(parent: &mut HashMap<u32, u32>, x: u32) -> u32 {
            let mut r = x;
            while let Some(&p) = parent.get(&r) {
                if p == r {
                    break;
                }
                r = p;
            }
            let mut c = x;
            while c != r {
                let next = *parent.get(&c).unwrap_or(&c);
                parent.insert(c, r);
                c = next;
            }
            r
        }
        for &(a, b) in pairs.iter() {
            let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
            if ra != rb {
                let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
                parent.insert(hi, lo);
            }
        }
        for (_, u) in self.units.iter_mut() {
            if u.group != 0 {
                let g = find(&mut parent, u.group);
                if g != u.group {
                    u.group = g;
                }
            }
        }
        self.step_scratch.merge_pairs.clear();
    }

    /// Acquire targets, chase-or-fire, apply buffered damage and despawn the
    /// dead. Slots in after `flock` and before `advance_orders` so an order
    /// completing because its target died is retired the same tick.
    fn combat(&mut self) {
        self.acquire_targets();
        self.engage();
        self.leash_home();
        self.resolve_deaths();
    }

    /// For every attack-moving or idle unit without a target, scan the flock grid
    /// (already rebuilt this tick by [`Sim::flock`], so this is a reuse, not
    /// a second broad-phase build) for the nearest eligible enemy within
    /// acquisition range and lock onto it.
    ///
    /// Candidate filter: alive, enemy by relation, within acquisition range,
    /// clear line of sight at the attacker's radius. Ranking is by surface
    /// distance *plus* [`TARGET_SPREAD_PENALTY`] per ally already engaging the
    /// candidate — only a handful of attackers fit around one body, so without
    /// spreading the surplus queues behind them forever instead of taking the
    /// enemy next to it. Ties break toward the lowest slot index (`UnitId`'s
    /// derived `Ord` sorts by index before generation) — deterministic
    /// regardless of grid scan order.
    ///
    /// Spreading is acquisition-only: an explicit [`Command::Attack`] is the
    /// player's decision and focuses whatever it was pointed at.
    fn acquire_targets(&mut self) {
        let s = &mut self.step_scratch;
        s.acquired_by.clear();
        s.acquired_target.clear();
        if s.ids.len() < 2 {
            return;
        }
        // Allies already committed to each candidate, counted once per tick
        // and bumped as this scan hands out targets, so units acquiring on the
        // same tick spread instead of all picking the same body. The counts
        // have to be complete before any unit scans, so this can't be folded
        // into the scan below — but it does double as the census that tells us
        // whether the scan is worth entering at all, which is the common case
        // on a marching army and in a fight everyone is already committed to.
        s.target_count.clear();
        s.target_count.resize(s.ids.len(), 0);
        let mut any_acquirer = false;
        for i in 0..s.ids.len() {
            let Some(u) = self.units.get(s.ids[i]) else {
                continue;
            };
            match u.target {
                Some(t) => {
                    if let Ok(k) = s.ids.binary_search(&t) {
                        s.target_count[k] += 1;
                    }
                }
                // Same eligibility test the scan applies below, kept in step
                // with it — see the comment there for why each clause is there.
                None => {
                    let idle = !u.is_moving() && u.orders.is_empty();
                    any_acquirer |= u.attack_range > 0.0 && (u.attack_move_goal.is_some() || idle);
                }
            }
        }
        if !any_acquirer {
            return;
        }
        let spread = TARGET_SPREAD_PENALTY.get();
        let cdt = self.nav.navmesh();
        for i in 0..s.ids.len() {
            let id = s.ids[i];
            let Some(unit) = self.units.get(id) else {
                continue;
            };
            // Who scans: attack-movers, and units standing idle (self
            // defence). Deliberately not a unit executing a plain `Move` —
            // "move here" means move here, which is the whole distinction
            // between `Move` and `AttackMove` — and not one with orders
            // queued behind it, since `advance_orders` waits on a live target
            // and self-defence would stall the queue behind it forever.
            let idle = !unit.is_moving() && unit.orders.is_empty();
            if unit.target.is_some()
                || unit.attack_range <= 0.0
                || (unit.attack_move_goal.is_none() && !idle)
            {
                continue;
            }
            let acq_range = unit.attack_range * ACQUISITION_RANGE_MULT.get();
            let (team, radius, pos) = (unit.team, unit.radius, unit.pos);
            let (cx, cy) = self.grid.cell_coords(s.positions[i]);
            // Clamp to the grid's extent: scanning further is a no-op, and it
            // keeps `cy + rings` / `cx + rings` safe from overflow.
            let rings = ((acq_range * self.grid.inv_cell).ceil() as u32)
                .max(1)
                .min(self.grid.rows.max(self.grid.cols));
            let mut best: Option<(f32, UnitId, usize)> = None;
            for ny in cy.saturating_sub(rings)..=(cy + rings).min(self.grid.rows - 1) {
                for nx in cx.saturating_sub(rings)..=(cx + rings).min(self.grid.cols - 1) {
                    for &j in self.grid.cell_entries(nx, ny) {
                        let cand_id = s.ids[j as usize];
                        if cand_id == id {
                            continue;
                        }
                        let Some(cand) = self.units.get(cand_id) else {
                            continue;
                        };
                        if relation_of(&self.relations, team, cand.team) != Relation::Enemy {
                            continue;
                        }
                        let delta = cand.pos - pos;
                        let surf = (delta.length() - radius - cand.radius).max(0.0);
                        if surf > acq_range {
                            continue;
                        }
                        if !clear_los(cdt, pos, cand.pos, radius) {
                            continue;
                        }
                        // Crowding penalty in body diameters of standoff, so it
                        // scales with how much room a candidate's ring has.
                        let score = surf
                            + s.target_count[j as usize] as f32 * spread * (radius + cand.radius);
                        let better = match best {
                            None => true,
                            Some((bd, bid, _)) => score < bd || (score == bd && cand_id < bid),
                        };
                        if better {
                            best = Some((score, cand_id, j as usize));
                        }
                    }
                }
            }
            if let Some((_, target_id, k)) = best {
                s.target_count[k] += 1;
                s.acquired_by.push(id);
                s.acquired_target.push(target_id);
            }
        }
        for k in 0..self.step_scratch.acquired_by.len() {
            let (id, target_id) = (
                self.step_scratch.acquired_by[k],
                self.step_scratch.acquired_target[k],
            );
            let Some(target_pos) = self.units.get(target_id).map(|t| t.pos) else {
                continue;
            };
            if let Some(unit) = self.units.get_mut(id) {
                // A unit that starts a fight from a standing start remembers
                // where it stood (or keeps the post it already had, if this is
                // the second fight of a chain). An attack-mover has its march
                // goal to resume instead.
                let post = match unit.attack_move_goal {
                    Some(_) => None,
                    None => unit.post.or(Some(unit.pos)),
                };
                unit.target = Some(target_id);
                clear_combat_state(unit);
                unit.post = post;
                unit.path.clear();
                unit.path.push(target_pos);
                unit.path_i = 0;
                unit.chase_anchor = target_pos;
                unit.chase_repath_in = CHASE_REPATH_TICKS.get();
            }
            self.step_scratch.repath.push(id);
        }
    }

    /// For every unit with a live target: fire if in range (stop, tick the
    /// cooldown, buffer damage on zero), else chase. Decisions are computed
    /// read-only first — resolving a unit's target needs a second live borrow
    /// of `units` — then applied.
    ///
    /// Everything here beyond "walk at the target and shoot" exists because
    /// one shared goal point turns an approach into a crush:
    ///
    /// - **Stations, not a goal point.** A unit walking to a target picks a
    ///   free station on a ring around it ([`pick_slot`]) — so a swarm spreads
    ///   around its target instead of everyone converging on one point and
    ///   shoving. Stations are stored as a direction code, not a position, so
    ///   the ring is re-derived from the target's current position each tick
    ///   and simply travels with it.
    /// - **Stop when there's nothing to gain.** In weapon range with elbow
    ///   room is a fine place to stand, and short walks stay short. Only a
    ///   unit that can't stop — in range but shoulder to shoulder with allies
    ///   — keeps walking, to its station, which is what spreads a swarm.
    /// - **Reserves.** Past a certain count the ring is full; the overflow
    ///   parks a body behind the front rank rather than shoving into it, and
    ///   re-scores on the repath cadence, so when a front-rank unit dies the
    ///   ring simply reads free and a reserve walks in. No death events, no
    ///   blocker ids, no retry timer.
    /// - **Hysteresis on the station, not on the range.** Damage re-checks the
    ///   true range every tick, so the slack that keeps a jostled unit at its
    ///   post ([`FIRE_SLACK_STEPS`]) never quietly extends a weapon's reach —
    ///   the trap that turned a 2.0 melee reach into 4.5 in an earlier spike.
    fn engage(&mut self) {
        let s = &mut self.step_scratch;
        s.combat_decisions.clear();
        s.slot_claims.clear();
        s.leash_home.clear();
        let cdt = self.nav.navmesh();
        let fire_slack_steps = FIRE_SLACK_STEPS.get();
        let block_progress_frac = BLOCK_PROGRESS_FRAC.get();
        let block_limit = BLOCK_TICKS.get();
        let chase_repath_dist = CHASE_REPATH_DIST.get();
        let leash_radii = LEASH_RADII.get();
        let acq_mult = ACQUISITION_RANGE_MULT.get();
        for i in 0..s.ids.len() {
            let id = s.ids[i];
            let Some(unit) = self.units.get(id) else {
                continue;
            };
            let Some(target_id) = unit.target else {
                continue;
            };
            let Some(target) = self.units.get(target_id) else {
                continue; // stale; cleaned up in resolve_deaths
            };
            // Leashed: a unit that started this fight itself has been drawn
            // far enough from its post. Break off and walk home — the one
            // thing a retreating target can otherwise do is tow a defender off
            // the map, since the return only triggers on a kill.
            if let Some(post) = unit.post {
                let leash = (leash_radii * unit.radius).max(acq_mult * unit.attack_range);
                if (unit.pos - post).length_squared() > leash * leash {
                    s.leash_home.push((id, post));
                    continue;
                }
            }
            let delta = target.pos - unit.pos;
            let surf = (delta.length() - unit.radius - target.radius).max(0.0);
            let step = unit.max_speed * DT;
            let in_range = surf <= unit.attack_range;
            let (rings, n_rings) = slot_rings(unit.radius, target.radius, unit.attack_range);
            // Derived fresh each tick rather than stored: the station tracks
            // the target, so "am I standing on it" has to be re-asked whenever
            // the target moves. Only the outer (reserve) ring is out of weapon
            // range — an inner station is in range by construction — so a unit
            // aiming at an inner one is not allowed to settle for "close
            // enough" and stop short of its own reach.
            // Asymmetric tolerance, which is where the hysteresis lives.
            // *Arriving* is tight — the standoff only leaves
            // `(1 - SLOT_STANDOFF) * attack_range` of margin inside weapon
            // range, so a unit that calls a station reached from a body away
            // has stopped somewhere it cannot shoot from. *Staying* is loose,
            // so crowd jitter around a station no longer rebuilds a path every
            // tick. Neither buys reach: `in_range` re-decides every shot.
            let arrive_tol = if unit.engaged {
                unit.radius + step * fire_slack_steps
            } else {
                station_margin(unit.radius, unit.attack_range).max(unit.radius * 0.05)
            };
            let (at_station, reserve_station, to_station) = match unit.chase_slot {
                NO_SLOT => (false, false, surf),
                code => {
                    let p = slot_point(target.pos, code, &rings);
                    let d = rings[((code >> 5) as usize).min(MAX_RINGS - 1)];
                    let gap = (p - unit.pos).length();
                    (
                        gap <= arrive_tol,
                        d - unit.radius - target.radius > unit.attack_range,
                        gap,
                    )
                }
            };

            // Block signal: is this unit closing on the place it is trying to
            // stand? Measured against the *station*, not the target — a unit
            // walking around a ring is not closing on the target at all, and
            // scoring it against the target's surface reads that as blocked
            // (or, once it starts circling, as progress) either way by
            // accident.
            //
            // A fraction of a step per tick, never an absolute epsilon: a unit
            // grinding through bodies still covers real ground. The bar grows
            // with the window (`blocked_for` ticks must buy `blocked_for`
            // fractions of a step), so slow-but-steady closing keeps the
            // counter down while a compressing crush doesn't.
            let (mut best_gap, mut blocked_for) = (unit.best_gap, unit.hold_ticks);
            blocked_for = blocked_for.saturating_add(1);
            if to_station < best_gap - step * block_progress_frac * blocked_for as f32 {
                best_gap = to_station;
                blocked_for = 0;
            }
            let tripped = blocked_for >= block_limit;
            let cadence = unit.chase_repath_in == 0;

            // Three ways to be standing still with a target:
            //
            // - **holding** — already stationed and still by its station. The
            //   patience counter below, not the range check, is what ends
            //   this: a melee station sits only a fraction of a body inside
            //   reach, so requiring `in_range` every tick would un-station a
            //   front-rank unit the moment the press jostled it, and it would
            //   repath, shove back in and be jostled again — the stutter.
            // - **arrived** — standing on the station this unit chose. A unit
            //   that drew an outer-ring station is a *reserve*: it parks a body
            //   behind the front rank rather than shoving into it, and keeps
            //   re-scoring below, so it walks in the moment the inner ring
            //   frees up. An inner station is in weapon range by construction,
            //   so arriving near one without being in range means the unit
            //   stopped short — it keeps closing instead.
            // - **in range with elbow room** — nothing to gain by walking on.
            //   This is what fills a ranged unit's whole in-range disc rather
            //   than a one-body-thick ring, and what keeps short walks short.
            //   The room test is the point: stopping on *contact* is what packs
            //   a swarm into a facing arc and has everyone shoving inward.
            // - **stuck but able to shoot** — in range and demonstrably not
            //   advancing. Stopping beats grinding on toward a station it
            //   cannot reach: it can already fire from here.
            let mut stationed = (at_station && (unit.engaged || in_range || reserve_station))
                || (in_range && (unit.chase_slot == NO_SLOT || !s.ally_contact[i] || tripped));
            // Patience, not a latch: a unit that is standing still and cannot
            // actually shoot — shoved a body past its own reach by the press,
            // say — gives it a moment for the jostling to settle, then walks
            // back onto its station instead of parking outside its range for
            // the rest of the fight. A unit standing on a *reserve* station is
            // meant to be out of range and waits there.
            let mut station_wait = unit.hold_ticks;
            let mut give_up_station = false;
            if stationed {
                if in_range || (at_station && reserve_station) {
                    station_wait = 0;
                } else {
                    station_wait = station_wait.saturating_add(1);
                    if station_wait >= block_limit {
                        stationed = false;
                        station_wait = 0;
                        // Re-score on the way out: the station it holds is one
                        // it evidently can't shoot from, and walking back into
                        // an occupied spot only to be pushed out again is the
                        // stutter this whole pass exists to remove.
                        give_up_station = true;
                    }
                }
            }

            // A stationed unit doesn't re-score — its station is where it
            // wants to be — unless it is out of weapon range, i.e. a reserve
            // waiting for room. That re-score is what refills the ring after a
            // front-rank death: no death events, no blocker ids, no timer.
            // Room turned up: give the reserve station back and walk in. The
            // chase path below does the rest — the slot is carried across in
            // `rescored` rather than re-picked, which would only pay for the
            // same scan twice.
            let mut rescored = None;
            if stationed && !in_range && cadence {
                let picked = pick_slot(
                    cdt,
                    &self.grid,
                    &s.positions,
                    &s.radii,
                    i,
                    unit.pos,
                    unit.radius,
                    unit.attack_range,
                    target.pos,
                    target.radius,
                    &rings,
                    n_rings,
                    unit.chase_slot,
                    target_id,
                    &s.slot_claims,
                );
                if picked != unit.chase_slot {
                    stationed = false;
                    rescored = Some(picked);
                }
            }
            if stationed {
                s.combat_decisions.push(CombatDecision {
                    id,
                    station: true,
                    in_range,
                    target_pos: target.pos,
                    goal: target.pos,
                    slot: unit.chase_slot,
                    need_repath: false,
                    cadence,
                    best_gap: f32::MAX,
                    hold_ticks: station_wait,
                });
                s.slot_claims.push((target_id, unit.chase_slot));
                continue;
            }
            let drift = target.pos - unit.chase_anchor;
            let far_drift = drift.length_squared() > chase_repath_dist * chase_repath_dist;
            // Every chaser walks to a station on the ring, not at the target
            // itself: one shared goal point for everyone is what turns an
            // approach into a crush. Re-scored on the existing cadence (or the
            // moment the unit concedes), never every tick — the goal itself
            // would otherwise become a new stutter source.
            let mut slot = unit.chase_slot;
            if let Some(picked) = rescored {
                slot = picked;
                // Leaving a station: `best_gap`/`blocked_for` above were
                // derived from `hold_ticks` while it held `station_wait`, so
                // they mean nothing here. Fresh station, fresh yardstick.
                best_gap = f32::MAX;
                blocked_for = 0;
            } else if tripped || cadence || far_drift || give_up_station || slot == NO_SLOT {
                slot = pick_slot(
                    cdt,
                    &self.grid,
                    &s.positions,
                    &s.radii,
                    i,
                    unit.pos,
                    unit.radius,
                    unit.attack_range,
                    target.pos,
                    target.radius,
                    &rings,
                    n_rings,
                    slot,
                    target_id,
                    &s.slot_claims,
                );
            }
            s.slot_claims.push((target_id, slot));
            if tripped {
                blocked_for = 0;
                best_gap = f32::MAX; // fresh station, fresh yardstick
                // Blocked, and re-scoring turned up nothing better than the
                // station it already can't reach: stand still rather than keep
                // shoving. Not a latch — a stationed unit out of weapon range
                // re-scores on the cadence above, so it walks in as soon as
                // the ring frees up.
                if slot == unit.chase_slot {
                    s.combat_decisions.push(CombatDecision {
                        id,
                        station: true,
                        in_range,
                        target_pos: target.pos,
                        goal: target.pos,
                        slot,
                        need_repath: false,
                        cadence,
                        best_gap: f32::MAX,
                        hold_ticks: 0,
                    });
                    continue;
                }
            }
            // Reaching this branch at all means the unit is *not* stationed,
            // so an empty path below is always something to fix: walk on. An
            // "arrived, near enough" test here would deadlock a unit that
            // stopped a body short of a station it is not in range from — it
            // would hold a position it cannot shoot from and never close.
            let goal = slot_goal(unit.pos, target.pos, slot, &rings);
            // The cadence exists to refresh a path toward a target that has
            // *moved*; against a stationary one it would rebuild an identical
            // path forever, which costs an allocation every time.
            let need_repath = slot != unit.chase_slot
                || unit.path.is_empty()
                || far_drift
                || (cadence && drift.length_squared() > 0.0);
            s.combat_decisions.push(CombatDecision {
                id,
                station: false,
                in_range,
                target_pos: target.pos,
                goal,
                slot,
                need_repath,
                cadence,
                best_gap,
                hold_ticks: blocked_for,
            });
        }
        for i in 0..self.step_scratch.combat_decisions.len() {
            // Copied out, not borrowed: applying a decision needs `units`
            // mutably while `step_scratch` stays live for `damage`/`repath`.
            let d = self.step_scratch.combat_decisions[i];
            let Some(unit) = self.units.get_mut(d.id) else {
                continue;
            };
            unit.engaged = d.station;
            unit.chase_slot = d.slot;
            unit.best_gap = d.best_gap;
            unit.hold_ticks = d.hold_ticks;
            if d.station {
                unit.path.clear();
                unit.path_i = 0;
                // The cooldown runs while stationed whether or not the shot
                // lands, so a unit briefly jostled out of range comes back
                // ready to fire instead of restarting its wind-up.
                if unit.cooldown_left == 0 && d.in_range {
                    let dmg = unit.damage;
                    let target_id = unit.target.expect("a decision implies a target");
                    self.step_scratch.damage.push((target_id, dmg));
                    unit.cooldown_left = unit.attack_cooldown_ticks.saturating_sub(1);
                } else {
                    unit.cooldown_left = unit.cooldown_left.saturating_sub(1);
                }
            } else if d.need_repath {
                // Reuse the path buffer: a settled fight must not allocate.
                unit.path.clear();
                unit.path.push(d.goal);
                unit.path_i = 0;
                unit.chase_anchor = d.target_pos;
                unit.chase_repath_in = CHASE_REPATH_TICKS.get();
                self.step_scratch.repath.push(d.id);
            } else if d.cadence {
                unit.chase_repath_in = CHASE_REPATH_TICKS.get();
            } else {
                unit.chase_repath_in -= 1;
            }
        }
    }

    /// Walk the units that hit their leash this tick back to their posts.
    ///
    /// A plain `Move`, deliberately — unlike the walk home after a kill, which
    /// is an attack-move so an ambush on the way back gets dealt with. The
    /// thing that leashed this unit is by definition still alive and nearby,
    /// so an attack-move home would re-acquire it on the next tick and the
    /// pair would loop: chase, break off, chase. Walking home deaf ends it.
    fn leash_home(&mut self) {
        for i in 0..self.step_scratch.leash_home.len() {
            let (id, post) = self.step_scratch.leash_home[i];
            self.start_move(&[id], post);
        }
        self.step_scratch.leash_home.clear();
    }

    /// Insert a lateral waypoint for any moving unit that has been in ally
    /// contact and making under [`DETOUR_FRAC`] of a step of path progress for
    /// [`DETOUR_TICKS`] ticks — the "walk around the blob" half of the
    /// asymmetric-separation pair.
    ///
    /// A goal, not a force: a lateral steering term inside [`Sim::flock`]
    /// argues with the path pull and loses at the next repath, so a unit
    /// pushed sideways still ends up grinding straight through. Both legs are
    /// line-of-sight checked at the unit's radius, so a detour never routes
    /// through a wall; if neither side validates, the unit keeps grinding and
    /// tries again after another [`DETOUR_TICKS`]. Units in their arrival zone
    /// are exempt — a settling blob is *meant* to slow to a stop.
    fn detour(&mut self) {
        let s = &mut self.step_scratch;
        let cdt = self.nav.navmesh();
        let detour_frac = DETOUR_FRAC.get();
        let detour_ticks = DETOUR_TICKS.get();
        let len_radii = DETOUR_LEN_RADII.get();
        for i in 0..s.ids.len() {
            let id = s.ids[i];
            let Some(unit) = self.units.get_mut(id) else {
                continue;
            };
            if !s.ally_contact[i] || !unit.is_moving() || s.within_arrival[i] {
                unit.ally_stall = 0;
                unit.ally_min_remaining = f32::MAX;
                continue;
            }
            let step = unit.max_speed * DT;
            let remaining = unit.remaining_len();
            // Same windowed yardstick as the chase block signal: the required
            // improvement scales with how long the unit has been at it, so a
            // unit crawling forward at a fraction of a step per tick still
            // reads as blocked instead of resetting the counter every few
            // ticks.
            unit.ally_stall = unit.ally_stall.saturating_add(1);
            if remaining < unit.ally_min_remaining - step * detour_frac * unit.ally_stall as f32 {
                unit.ally_min_remaining = remaining;
                unit.ally_stall = 0;
                continue;
            }
            if unit.ally_stall < detour_ticks {
                continue;
            }
            unit.ally_stall = 0;
            let (pos, radius) = (unit.pos, unit.radius);
            let next = unit.waypoint();
            let heading = norm(next - pos);
            if heading == Vector2::ZERO {
                continue;
            }
            let lat = Vector2::new(-heading.y, heading.x);
            // Cap the offset against the distance left to the waypoint being
            // detoured around: `integrate` re-anchors past any waypoint whose
            // gate the unit is already on the far side of, and the geometry
            // `pos + lat * L + heading * L/2` sits past that gate as soon as
            // `L > 0.4 * dist`. An over-long detour would be skipped the very
            // next tick and the unit would grind on, re-tripping forever.
            let to_next = (next - pos).length();
            let l = (radius * len_radii).min(to_next * (1.0 / 3.0));
            if l < radius {
                continue; // too little room left to be worth a detour
            }
            let ahead = pos + heading * (l * 0.5);
            // Same occupancy score the approach slots use: go around the
            // emptier side. Ties (an even wall) break to `+lat`.
            let (a, b) = (ahead + lat * l, ahead - lat * l);
            let (ca, cb) = (
                occupancy(&self.grid, &s.positions, &s.radii, i, a, radius),
                occupancy(&self.grid, &s.positions, &s.radii, i, b, radius),
            );
            let first = if ca <= cb { a } else { b };
            let second = if ca <= cb { b } else { a };
            let ok = |w: Vector2| clear_los(cdt, pos, w, radius) && clear_los(cdt, w, next, radius);
            let waypoint = if ok(first) {
                first
            } else if ok(second) {
                second
            } else {
                continue; // boxed in by walls: keep grinding, retry later
            };
            // Drop waypoints already rounded before inserting: it keeps the
            // path Vec inside the spare capacity `set_path` reserved, so a
            // deepening detour never reallocates mid-tick.
            unit.path.drain(..unit.path_i as usize);
            unit.path_i = 0;
            unit.path.insert(0, waypoint);
            // The detour lengthens the route by design; re-seed both progress
            // yardsticks so neither reads it as a stall.
            let remaining = unit.remaining_len();
            unit.ally_min_remaining = remaining;
            unit.min_remaining = remaining;
            unit.stall = 0;
        }
    }

    /// Apply this tick's buffered damage, despawn anything at or below zero
    /// health (slot order, so two units that kill each other the same tick
    /// both die — buffered above for the same reason), then clear any
    /// now-stale target references and resume the march for attack-movers
    /// whose target just died. The cleanup scan only runs when something
    /// actually died: a target reference can only go stale via a despawn,
    /// and every despawn is handled the same tick it happens, so a quiet
    /// tick has nothing to clean up.
    fn resolve_deaths(&mut self) {
        let s = &mut self.step_scratch;
        s.dead.clear();
        // Health only changes via the damage buffer, so no damage means no new deaths.
        if s.damage.is_empty() {
            return;
        }
        for &(target, dmg) in s.damage.iter() {
            if let Some(u) = self.units.get_mut(target) {
                u.health -= dmg;
            }
        }
        s.damage.clear();
        for (id, u) in self.units.iter() {
            if u.health <= 0.0 {
                s.dead.push(id);
            }
        }
        if s.dead.is_empty() {
            return;
        }
        for i in 0..self.step_scratch.dead.len() {
            let id = self.step_scratch.dead[i];
            self.units.despawn(id);
        }
        // A stale target leaves a unit doing one of three things: resuming an
        // attack-move's march, walking back to the post it left to fight, or
        // simply going idle where it stands. `start_attack_move` clears
        // `target` itself, so only the idle case needs the write done here.
        //
        // The walk home goes out as an attack-move, not a plain move, so a
        // unit jumped on the way back fights and *then* carries on home
        // instead of walking through the ambush; and the post survives that
        // second fight, so it still ends up where it started.
        let mut idle: Vec<UnitId> = Vec::new();
        let mut resume_march: Vec<(UnitId, Vector2)> = Vec::new();
        let mut return_to_post: Vec<(UnitId, Vector2)> = Vec::new();
        for (id, u) in self.units.iter() {
            if let Some(t) = u.target
                && self.units.get(t).is_none()
            {
                match (u.attack_move_goal, u.post) {
                    (Some(goal), _) => resume_march.push((id, goal)),
                    // Already home (it never had to leave): nothing to walk.
                    (None, Some(post)) if (post - u.pos).length_squared() > u.radius * u.radius => {
                        return_to_post.push((id, post))
                    }
                    _ => idle.push(id),
                }
            }
        }
        for id in idle {
            if let Some(u) = self.units.get_mut(id) {
                u.target = None;
                clear_combat_state(u);
            }
        }
        for (id, goal) in resume_march {
            self.start_attack_move(&[id], goal);
        }
        for (id, post) in return_to_post {
            self.start_attack_move(&[id], post);
            self.units.get_mut(id).expect("alive").post = Some(post);
        }
    }

    /// Collect (dedup) every constrained half-edge within `radius` of `p` into
    /// `walls`, without moving anything — a read-only version of the radius
    /// BFS `wall_clamp`'s pass loop uses while pushing. Reuses `faces`/
    /// `visited` as scratch (cleared internally).
    fn gather_nearby_walls(
        cdt: &CDT,
        p: Vector2,
        radius: f32,
        faces: &mut Vec<u32>,
        visited: &mut Vec<u32>,
        walls: &mut Vec<u32>,
    ) {
        let Some(start) = cdt.locate_face(p) else {
            return;
        };
        faces.clear();
        visited.clear();
        faces.push(start);
        visited.push(start);
        while let Some(face) = faces.pop() {
            for he in face * 3..face * 3 + 3 {
                let a = cdt.points()[cdt.he_origin(he) as usize];
                let b = cdt.points()[cdt.he_dest(he) as usize];
                let closest = closest_on_segment(p, a, b);
                let delta = p - closest;
                if delta.x * delta.x + delta.y * delta.y >= radius * radius {
                    continue;
                }
                if cdt.he_is_constrained(he) {
                    if !walls.contains(&he) {
                        walls.push(he);
                    }
                } else if let Some(twin) = cdt.he_twin(he) {
                    let nb = cdt.face_of_he(twin);
                    if !visited.contains(&nb) {
                        visited.push(nb);
                        faces.push(nb);
                    }
                }
            }
        }
    }

    /// Whether the tick's end-to-end motion (chord `prev → pos`) passed
    /// within `radius` of two walls that don't share an endpoint — squeezed
    /// through a gap the body doesn't fit. Adjacent walls are exempt: they
    /// form one obstacle corner, which paths may legitimately cut close (see
    /// `astar.rs`'s clearance contract) trusting the clamp to push back out.
    fn swept_through_pinch(
        cdt: &CDT,
        s: &mut StepScratch,
        prev: Vector2,
        pos: Vector2,
        radius: f32,
    ) -> bool {
        // Midpoint gather with radius expanded by half the chord length covers
        // every wall within `radius` of any point of the chord in one BFS.
        let half = (pos - prev) * 0.5;
        Self::gather_nearby_walls(
            cdt,
            prev + half,
            radius + half.length(),
            &mut s.faces,
            &mut s.visited,
            &mut s.clamp_walls,
        );
        let min_clear = radius - WALL_CLAMP_REVERT_EPS_FRAC * radius;
        s.clamp_walls.retain(|&he| {
            let a = cdt.points()[cdt.he_origin(he) as usize];
            let b = cdt.points()[cdt.he_dest(he) as usize];
            dist_segment_segment(prev, pos, a, b) < min_clear
        });
        let shares_vertex = |he1: u32, he2: u32| {
            let (a1, b1) = (cdt.he_origin(he1), cdt.he_dest(he1));
            let (a2, b2) = (cdt.he_origin(he2), cdt.he_dest(he2));
            a1 == a2 || a1 == b2 || b1 == a2 || b1 == b2
        };
        s.clamp_walls.iter().enumerate().any(|(i, &he1)| {
            s.clamp_walls[i + 1..]
                .iter()
                .any(|&he2| !shares_vertex(he1, he2))
        })
    }

    /// Project unit circles out of nearby constrained edges. Paths already
    /// respect radius; this only cleans up separation pushes, so units that
    /// didn't move this tick are skipped — unless the mesh changed, which can
    /// put a new wall under a stationary unit.
    fn wall_clamp(&mut self, mesh_changed: bool) {
        let cdt = self.nav.navmesh();
        let s = &mut self.step_scratch;
        for (id, unit) in self.units.iter_mut() {
            if unit.radius <= 0.0 || (!mesh_changed && unit.pos == unit.prev_pos) {
                continue;
            }
            // Fast path: wall distance changes at most 1:1 with distance moved, so
            // with no wall within `radius` + chord length of the endpoint, the
            // pass loop below finds nothing to push and no wall can lie within
            // `radius` of the chord either — no possible pinch. Skip the unit.
            let reach = unit.radius + (unit.pos - unit.prev_pos).length();
            s.clamp_walls.clear();
            Self::gather_nearby_walls(
                cdt,
                unit.pos,
                reach,
                &mut s.faces,
                &mut s.visited,
                &mut s.clamp_walls,
            );
            if s.clamp_walls.is_empty() {
                unit.stall = 0;
                continue;
            }
            // A push moves the circle and changes which faces it overlaps,
            // invalidating the walk in progress, so re-walk until a pass pushes
            // nothing (bounded against corner ping-pong). `out` accumulates
            // outward wall normals to tell a faced wall from one merely beside
            // the unit; `pushed_any` tracks whether any pass pushed, so the
            // stall detector still fires when opposing normals cancel in `out`.
            let mut out = Vector2::ZERO;
            let mut pushed_any = false;
            // Check every push against every wall touched so far: a push snaps
            // the circle to exactly `radius` from one wall — at a segment
            // endpoint, radially from that corner — and near a sub-diameter gap
            // that snap can leap past the *other* wall's exclusion disk in one
            // discrete step ("corner-teleport").
            s.clamp_walls.clear();
            let mut violated = false;
            let min_clear = unit.radius - WALL_CLAMP_REVERT_EPS_FRAC * unit.radius;
            'passes: for _ in 0..WALL_CLAMP_PASSES {
                let Some(start) = cdt.locate_face(unit.pos) else {
                    break;
                };
                s.faces.clear();
                s.visited.clear();
                s.faces.push(start);
                s.visited.push(start);
                let mut pushed = false;
                while let Some(face) = s.faces.pop() {
                    for he in face * 3..face * 3 + 3 {
                        let a = cdt.points()[cdt.he_origin(he) as usize];
                        let b = cdt.points()[cdt.he_dest(he) as usize];
                        let closest = closest_on_segment(unit.pos, a, b);
                        let delta = unit.pos - closest;
                        let d2 = delta.x * delta.x + delta.y * delta.y;
                        if d2 >= unit.radius * unit.radius {
                            continue;
                        }
                        if cdt.he_is_constrained(he) {
                            if !s.clamp_walls.contains(&he) {
                                s.clamp_walls.push(he);
                            }
                            let d = d2.sqrt();
                            if d > 1e-6 {
                                unit.pos = closest + delta * (unit.radius / d);
                                pushed = true;
                                pushed_any = true;
                                out += delta * (1.0 / d);
                                if s.clamp_walls.iter().any(|&wh| {
                                    if wh == he {
                                        return false;
                                    }
                                    let wa = cdt.points()[cdt.he_origin(wh) as usize];
                                    let wb = cdt.points()[cdt.he_dest(wh) as usize];
                                    let c = closest_on_segment(unit.pos, wa, wb);
                                    let dd = unit.pos - c;
                                    dd.x * dd.x + dd.y * dd.y < min_clear * min_clear
                                }) {
                                    violated = true;
                                    break 'passes;
                                }
                            }
                        } else if let Some(twin) = cdt.he_twin(he) {
                            // Circle reaches past a free edge: also check the
                            // neighbor face's walls.
                            let nb = cdt.face_of_he(twin);
                            if !s.visited.contains(&nb) {
                                s.visited.push(nb);
                                s.faces.push(nb);
                            }
                        }
                    }
                }
                if !pushed {
                    break;
                }
            }
            // Revert when a push re-violated a touched wall or the chord swept a
            // gap the body doesn't fit: `prev_pos` is clear by induction, so crowd
            // pressure can never ratchet a unit into or through a pinch. Skipped
            // on `mesh_changed` since a new obstacle can invalidate `prev_pos`
            // itself, where best-effort projection is the correct fallback.
            if !mesh_changed
                && (violated
                    || Self::swept_through_pinch(cdt, s, unit.prev_pos, unit.pos, unit.radius))
            {
                unit.pos = unit.prev_pos;
            }
            // Stuck-on-corner detection: a unit shoved off its cleared route so
            // its line to the next waypoint cuts a wall is pressed against that
            // wall (`pushed_any`), heading *into* it, and not shrinking its
            // remaining path. The wall-facing test separates this from a unit
            // merely jammed sideways by neighbours (repath wouldn't help there).
            // Skipped in the arrival zone (crowding, not walls, holds it there).
            // Free ticks reset the counter so only consecutive clamped ticks
            // accumulate toward the repath threshold.
            if pushed_any && unit.is_moving() {
                let goal = *unit.path.last().expect("moving ⇒ non-empty path");
                let (gx, gy) = (goal.x - unit.pos.x, goal.y - unit.pos.y);
                let in_arrival = gx * gx + gy * gy <= unit.arrival_r * unit.arrival_r;
                let rem = unit.remaining_len();
                let w = unit.waypoint();
                let into_wall = (w.x - unit.pos.x) * out.x + (w.y - unit.pos.y) * out.y < 0.0;
                if in_arrival {
                    unit.stall = 0;
                    unit.min_remaining = rem;
                } else if rem < unit.min_remaining - STALL_PROGRESS_EPS.get() {
                    unit.min_remaining = rem;
                    unit.stall = 0;
                } else if into_wall {
                    unit.stall = unit.stall.saturating_add(1);
                    if unit.stall >= STALL_REPATH_TICKS.get() {
                        s.repath.push(id);
                        unit.stall = 0;
                    }
                }
            } else {
                unit.stall = 0; // no wall contact this tick: don't carry stall forward
            }
        }
    }

    /// Repath units the wall clamp flagged as stuck against a corner, from
    /// their current (shoved) positions, so their next waypoint is reachable in
    /// a straight line again. Usually a no-op (empty list).
    fn repath_stalled(&mut self) {
        if self.step_scratch.repath.is_empty() {
            return;
        }
        let cdt = self.nav.navmesh();
        for i in 0..self.step_scratch.repath.len() {
            let id = self.step_scratch.repath[i];
            let Some(unit) = self.units.get_mut(id) else {
                continue;
            };
            let Some(&goal) = unit.path.last() else {
                continue;
            };
            let path = find_path_abstract(
                cdt,
                &self.abstraction,
                unit.pos,
                goal,
                &mut self.scratch,
                unit.radius,
            );
            set_path(unit, path);
        }
        self.step_scratch.repath.clear();
    }

    /// FNV-1a over tick, unit slots (slot order) and RNG — divergence detector
    /// for determinism tests and future lockstep.
    pub fn state_hash(&self) -> u64 {
        let mut h = Fnv::new();
        h.write_u64(self.tick);
        h.write_u64(self.group_seq as u64);
        h.write_u64(self.nav.num_obstacles() as u64);
        h.write_u64(self.relations.len() as u64);
        for &((a, b), rel) in &self.relations {
            h.write_u64(a as u64);
            h.write_u64(b as u64);
            h.write_u64(rel as u64);
        }
        h.write_u64(self.units.slots.len() as u64);
        for (i, slot) in self.units.slots.iter().enumerate() {
            h.write_u64(self.units.generations[i] as u64);
            let Some(u) = slot else {
                h.write_u64(u64::MAX);
                continue;
            };
            h.write_v2(u.pos);
            h.write_v2(u.prev_pos);
            h.write_f32(u.radius);
            h.write_f32(u.max_speed);
            h.write_u64(u.group as u64);
            h.write_u64(u.parked as u64);
            h.write_f32(u.arrival_r);
            h.write_u64(u.stall as u64);
            h.write_f32(u.min_remaining);
            h.write_u64(u.path_i as u64);
            h.write_u64(u.path.len() as u64);
            for &p in &u.path {
                h.write_v2(p);
            }
            h.write_u64(u.team as u64);
            h.write_f32(u.max_health);
            h.write_f32(u.health);
            h.write_f32(u.damage);
            h.write_f32(u.attack_range);
            h.write_u64(u.attack_cooldown_ticks as u64);
            h.write_u64(u.cooldown_left as u64);
            h.write_u64(u.target.map_or(u64::MAX, |t| t.raw()));
            h.write_u64(u.target_commanded as u64);
            match u.post {
                Some(p) => {
                    h.write_u64(1);
                    h.write_v2(p);
                }
                None => h.write_u64(0),
            }
            match u.attack_move_goal {
                Some(g) => {
                    h.write_u64(1);
                    h.write_v2(g);
                }
                None => h.write_u64(0),
            }
            h.write_v2(u.chase_anchor);
            h.write_u64(u.chase_repath_in as u64);
            h.write_u64(u.engaged as u64);
            h.write_u64(u.chase_slot as u64);
            h.write_f32(u.best_gap);
            h.write_u64(u.hold_ticks as u64);
            h.write_u64(u.ally_stall as u64);
            h.write_f32(u.ally_min_remaining);
            h.write_u64(u.orders.len() as u64);
            for order in &u.orders {
                match order {
                    Order::Move { goal } => {
                        h.write_u64(0);
                        h.write_v2(*goal);
                    }
                    Order::Attack { target } => {
                        h.write_u64(1);
                        h.write_u64(target.raw());
                    }
                    Order::AttackMove { goal } => {
                        h.write_u64(2);
                        h.write_v2(*goal);
                    }
                }
            }
        }
        self.rng.hash_into(&mut h);
        h.0
    }
}

/// Core of [`Sim::relation`], as a free function over just the override
/// table — lets combat's inner loops (which hold a mutable borrow of
/// `step_scratch`) query relations without needing a whole-`&self` borrow.
fn relation_of(relations: &[((u32, u32), Relation)], a: u32, b: u32) -> Relation {
    if a == b {
        return Relation::Ally;
    }
    let key = if a < b { (a, b) } else { (b, a) };
    relations
        .iter()
        .find(|(k, _)| *k == key)
        .map_or(Relation::Enemy, |&(_, r)| r)
}

fn norm(v: Vector2) -> Vector2 {
    let l2 = v.x * v.x + v.y * v.y;
    if l2 > 1e-12 {
        v * (1.0 / l2.sqrt())
    } else {
        Vector2::ZERO
    }
}

/// Outward direction at a path corner `cur` between `prev` and `next`: the unit
/// vector pointing away from the wall vertex the shortest-path apex wraps, into
/// the bend's free side. `norm(prev−cur)+norm(next−cur)` bisects toward the
/// inside (the vertex), so negate it. `ZERO` for a (near-)straight corner.
fn external_bisector(prev: Vector2, cur: Vector2, next: Vector2) -> Vector2 {
    -norm(norm(prev - cur) + norm(next - cur))
}

/// A unit's path through the flock's shared corner apexes, each displaced
/// `outward[j] * offset` into the bend's free side so the flock fans out
/// instead of single-filing the apex. Tries the full offset, then halves
/// toward the apex; returns the first polyline whose every leg clears walls,
/// or `None` so the caller paths the unit alone.
///
/// Fanned legs (offset > 0) validate at an inflated radius so a unit only
/// takes a lane with room for parallel lanes — in a tight squeeze every
/// offset fails this and it falls back to the apex (single-file, the only
/// fit). The apex itself (scale 0) validates at the true radius, so the
/// shortest route is always allowed.
fn build_offset_path(
    cdt: &CDT,
    start: Vector2,
    corners: &[Vector2],
    outward: &[Vector2],
    goal: Vector2,
    offset: f32,
    radius: f32,
) -> Option<Vec<Vector2>> {
    // One extra radius of clearance ⇒ a fanned lane has ≈ a full diameter of
    // room beside the apex before it's accepted.
    let fan_radius = radius * 2.0;
    for &scale in &[1.0f32, 0.5, 0.25, 0.0] {
        let o = offset * scale;
        // Offset can be signed (straight-leg fanning spreads to both sides of the
        // line); the inflated lane check keys on magnitude, not direction.
        let check_r = if o.abs() > 1e-6 { fan_radius } else { radius };
        let mut path = Vec::with_capacity(corners.len() + 2);
        path.push(start);
        for (c, n) in corners.iter().zip(outward) {
            path.push(*c + *n * o);
        }
        path.push(goal);
        if path.windows(2).all(|w| clear_los(cdt, w[0], w[1], check_r)) {
            return Some(path);
        }
    }
    None
}

/// Fallback when a unit can't reach the flock's shared corners directly: route
/// it onto the channel *via* the first corner instead of a private shortest
/// path — otherwise an outer unit whose own shortest rounds an obstacle the
/// *other* way splits off from the group. Falls back to a plain shortest path
/// with no corners, or if even the first leg is blocked.
fn route_onto_channel(
    cdt: &CDT,
    abstraction: &Abstraction,
    scratch: &mut AStarScratch,
    start: Vector2,
    corners: &[Vector2],
    goal: Vector2,
    radius: f32,
) -> Vec<Vector2> {
    let mut p = if corners.is_empty() {
        Vec::new()
    } else {
        find_path_abstract(cdt, abstraction, start, corners[0], scratch, radius)
    };
    if p.len() >= 2 {
        p.extend_from_slice(&corners[1..]);
        p.push(goal);
        p
    } else {
        find_path_abstract(cdt, abstraction, start, goal, scratch, radius)
    }
}

// ── Approach slots ────────────────────────────────────────────────────────────

/// Unit directions at 15° steps, as literal `(cos, sin)` pairs: the sim's
/// determinism contract is f32 add/mul/div/sqrt only, so no trig may run at
/// runtime. Exact axis values are written as literal zeroes rather than the
/// 1e-16 residue a real cosine would return.
const SLOT_DIRS: [(f32, f32); 24] = [
    (1.0, 0.0),
    (COS15, SIN15),
    (COS30, 0.5),
    (SQRT_HALF, SQRT_HALF),
    (0.5, COS30),
    (SIN15, COS15),
    (0.0, 1.0),
    (-SIN15, COS15),
    (-0.5, COS30),
    (-SQRT_HALF, SQRT_HALF),
    (-COS30, 0.5),
    (-COS15, SIN15),
    (-1.0, 0.0),
    (-COS15, -SIN15),
    (-COS30, -0.5),
    (-SQRT_HALF, -SQRT_HALF),
    (-0.5, -COS30),
    (-SIN15, -COS15),
    (0.0, -1.0),
    (SIN15, -COS15),
    (0.5, -COS30),
    (SQRT_HALF, -SQRT_HALF),
    (COS30, -0.5),
    (COS15, -SIN15),
];

const COS15: f32 = 0.9659258;
const SIN15: f32 = 0.25881904;
const COS30: f32 = 0.8660254;
const SQRT_HALF: f32 = std::f32::consts::FRAC_1_SQRT_2;

/// τ, for the ring-circumference stride below (no runtime trig, so it's a
/// literal).
const TAU: f32 = 6.283_185_5;

/// How far inside weapon reach a station sits — see [`SLOT_STANDOFF`]. Capped
/// at the reach itself, so a melee station lands against the target's body
/// rather than behind the unit.
fn station_margin(r_self: f32, attack_range: f32) -> f32 {
    (SLOT_STANDOFF.get() * r_self).min(attack_range)
}

/// Most approach rings a unit will consider (the last is always the
/// out-of-range reserve).
const MAX_RINGS: usize = 4;

/// Radii of the approach rings around a target, outermost first.
///
/// Ring 0 sits at `r_self + r_target + SLOT_STANDOFF * attack_range` — inside
/// weapon range, which is the whole point; dropping either radius from the
/// standoff (as a naive "ring attractor" does) parks attackers just out of
/// reach. Further rings step *inward* a body diameter at a time while they
/// still clear the target's own body, so a long-reach unit's whole in-range
/// disc gets used instead of a single one-body-thick shell — with 24 attackers
/// on one target, one ring leaves a third of them standing outside their own
/// range doing nothing. The final entry is one body diameter *beyond* ring 0:
/// the reserve ring, out of range by construction, where overflow waits.
fn slot_rings(r_self: f32, r_target: f32, attack_range: f32) -> ([f32; MAX_RINGS], usize) {
    let mut rings = [0.0; MAX_RINGS];
    let outer = r_self + r_target + attack_range - station_margin(r_self, attack_range);
    let floor = r_self + r_target;
    let mut n = 0;
    while n < MAX_RINGS - 1 {
        let d = outer - 2.0 * r_self * n as f32;
        if d < floor {
            break;
        }
        rings[n] = d;
        n += 1;
    }
    if n == 0 {
        // Degenerate (zero radii): one ring at the standoff, whatever it is.
        rings[0] = outer;
        n = 1;
    }
    rings[n] = outer + 2.0 * r_self;
    (rings, n + 1)
}

/// Ring index of a slot code (`ring * 32 + direction index`), clamped so a
/// code from a call with more rings than this one can still be read.
fn ring_of(code: u16) -> usize {
    ((code >> 5) as usize).min(MAX_RINGS - 1)
}

/// World point of a slot code (`ring * 32 + direction index`).
fn slot_point(target_pos: Vector2, code: u16, rings: &[f32; MAX_RINGS]) -> Vector2 {
    let (c, s) = SLOT_DIRS[(code & 31) as usize];
    let d = rings[ring_of(code)];
    Vector2::new(target_pos.x + c * d, target_pos.y + s * d)
}

/// Where a unit holding slot `code` should walk next: its station, or the arc
/// step toward it when the target's own body sits on the direct chord (see
/// [`orbit_step`]). [`NO_SLOT`] walks at the target itself.
fn slot_goal(pos: Vector2, target_pos: Vector2, code: u16, rings: &[f32; MAX_RINGS]) -> Vector2 {
    match code {
        NO_SLOT => target_pos,
        code => orbit_step(
            pos,
            target_pos,
            slot_point(target_pos, code, rings),
            rings[ring_of(code)],
        ),
    }
}

/// How crowded a point is: summed normalised overlap with every unit whose
/// body would touch a unit of `r_self` standing there, over the flock grid's
/// start-of-tick snapshot (never live positions — a score against positions
/// mutating mid-pass isn't deterministic). `skip` is the asking unit's own
/// dense index.
fn occupancy(
    grid: &SpatialGrid,
    positions: &[Vector2],
    radii: &[f32],
    skip: usize,
    p: Vector2,
    r_self: f32,
) -> f32 {
    if grid.cols == 0 || grid.rows == 0 {
        return 0.0;
    }
    let (cx, cy) = grid.cell_coords(p);
    let mut total = 0.0;
    for ny in cy.saturating_sub(1)..=(cy + 1).min(grid.rows - 1) {
        for nx in cx.saturating_sub(1)..=(cx + 1).min(grid.cols - 1) {
            for &j in grid.cell_entries(nx, ny) {
                let j = j as usize;
                if j == skip {
                    continue;
                }
                let touch = r_self + radii[j];
                let d = positions[j] - p;
                let d2 = d.x * d.x + d.y * d.y;
                if d2 < touch * touch {
                    total += (touch - d2.sqrt()) / touch;
                }
            }
        }
    }
    total
}

/// Pick the approach slot a blocked attacker should walk to, as a slot code
/// (or [`NO_SLOT`] to keep walking at the target).
///
/// World-anchored rings of sampled directions, scored by ally occupancy at the
/// point, plus the arc the unit would have to walk to get there, plus a
/// per-ring penalty; lowest wins, ties by table index. No assignment and no
/// capacity — beyond a within-tick claim so units deciding together don't all
/// read the same station as free, two units may pick the same one and
/// separation sorts it out.
///
/// The sampling stride comes from the ring circumference, never from the table
/// directly: 24 evenly spaced directions around a small ring sit well under a
/// body diameter apart, so every candidate reads as occupied by the units on
/// its neighbouring slots and nobody ever moves in.
///
/// A candidate needs clear line of sight from the *target* (the slot has to be
/// somewhere the unit could actually stand and shoot from); with a target in a
/// doorway or against a wall most candidates fail that and the unit falls back
/// to chasing the target directly.
#[allow(clippy::too_many_arguments)]
fn pick_slot(
    cdt: &CDT,
    grid: &SpatialGrid,
    positions: &[Vector2],
    radii: &[f32],
    self_i: usize,
    pos: Vector2,
    r_self: f32,
    attack_range: f32,
    target_pos: Vector2,
    r_target: f32,
    rings: &[f32; MAX_RINGS],
    n_rings: usize,
    current: u16,
    target_id: UnitId,
    claims: &[(UnitId, u16)],
) -> u16 {
    // Stations already claimed against this target on this tick. Bodies not
    // yet standing there are invisible to the occupancy scan, so without this
    // every unit deciding on the same tick reads the same station as free and
    // they queue behind each other instead of fanning out. Counted once per
    // call, not once per candidate.
    let mut claimed = [0u8; 128];
    for &(t, code) in claims {
        if t == target_id && code != NO_SLOT {
            let k = (code & 127) as usize;
            claimed[k] = claimed[k].saturating_add(1);
        }
    }
    // The unit's own bearing *as seen from the target*: the arc from here to a
    // candidate is what the unit has to walk (see `orbit_step`). Measuring the
    // turn from the unit's viewpoint instead makes the station directly behind
    // the target look like "straight ahead" and picks it, sending the unit on
    // a lap around the target it never needed to make.
    let bearing = norm(pos - target_pos);
    let turn_cost = SLOT_TURN_COST.get();
    let outer_penalty = SLOT_OUTER_PENALTY.get();
    let inner_penalty = SLOT_INNER_PENALTY.get();
    // `cutoff` is the best score so far: a candidate that can't beat it is
    // dropped before the line-of-sight walk, which is two face locations plus
    // a walk across the triangulation where everything above it is one 3×3
    // grid sweep. Skipping it changes no outcome — the candidate had already
    // lost — but it takes the mesh walk off most of the ~32 candidates a call
    // scores. `None` forces the full evaluation, for the held station below.
    let cost_of = |code: u16, cutoff: Option<f32>| -> Option<f32> {
        let p = slot_point(target_pos, code, rings);
        let mut cost = occupancy(grid, positions, radii, self_i, p, r_self)
            + claimed[(code & 127) as usize] as f32;
        let (dx, dy) = SLOT_DIRS[(code & 31) as usize];
        cost += turn_cost * (1.0 - (bearing.x * dx + bearing.y * dy));
        // Prefer standing at reach: each ring inward is a step closer than
        // this unit needs to be, and the last ring can't shoot at all.
        let ring = ring_of(code);
        let reserve = rings[ring] - r_self - r_target > attack_range;
        cost += if reserve {
            outer_penalty
        } else {
            ring as f32 * inner_penalty
        };
        if cutoff.is_some_and(|bc| cost >= bc) {
            return None;
        }
        if !clear_los(cdt, target_pos, p, r_self) {
            return None;
        }
        Some(cost)
    };

    let mut best: Option<(f32, u16)> = None;
    for (ring, &d) in rings.iter().enumerate().take(n_rings) {
        if d <= 0.0 {
            continue;
        }
        // Slots at least a body diameter apart along the ring.
        let stride = (SLOT_DIRS.len() as f32 * 2.0 * r_self / (TAU * d)).ceil() as usize;
        let stride = stride.clamp(1, SLOT_DIRS.len());
        let mut k = 0;
        while k < SLOT_DIRS.len() {
            let code = (ring as u16) * 32 + k as u16;
            if let Some(cost) = cost_of(code, best.map(|(bc, _)| bc)) {
                best = Some((cost, code));
            }
            k += stride;
        }
    }
    let Some((best_cost, best_code)) = best else {
        return NO_SLOT; // nowhere to stand (walled-in target): chase directly
    };
    // Switch margin: re-scoring on the repath cadence must not turn the goal
    // itself into a stutter source.
    if current != NO_SLOT
        && let Some(held) = cost_of(current, None)
        && best_cost > held - SLOT_SWITCH_MARGIN.get()
    {
        return current;
    }
    best_code
}

/// Where to actually walk next on the way to a station.
///
/// A station on the far side of the target is not reachable in a straight
/// line: the target's own body sits on the chord, so a unit aimed straight at
/// it grinds into the target and stays there (paths are computed against
/// walls, not bodies). So a unit more than 45° of bearing away from its
/// station walks the ring instead — one 45° arc step at a time, re-derived
/// every repath, which reads as circling the target rather than shoving
/// through it.
///
/// 45° is the widest step whose chord still clears both bodies: the chord of a
/// `d` ring subtends `d * cos(22.5°)` ≈ `0.92 d` at closest approach, and the
/// ring itself starts at `r_self + r_target`. No trig at runtime — a 45°
/// rotation is a fixed matrix built from [`SQRT_HALF`].
fn orbit_step(pos: Vector2, target_pos: Vector2, slot_pos: Vector2, ring: f32) -> Vector2 {
    let from = norm(pos - target_pos);
    let to = norm(slot_pos - target_pos);
    if from == Vector2::ZERO || to == Vector2::ZERO {
        return slot_pos;
    }
    // Within one arc step of the station: go straight there.
    if from.x * to.x + from.y * to.y >= SQRT_HALF {
        return slot_pos;
    }
    // Rotate the unit's own bearing 45° toward the station.
    let sin = if from.x * to.y - from.y * to.x >= 0.0 {
        SQRT_HALF
    } else {
        -SQRT_HALF
    };
    let stepped = Vector2::new(
        from.x * SQRT_HALF - from.y * sin,
        from.x * sin + from.y * SQRT_HALF,
    );
    target_pos + stepped * ring
}

/// Reset everything about one engagement: the post to return to, whose choice
/// the target was, the station latch, the held station and both
/// blocked-progress counters. Every
/// path out of a fight (a new order, a fresh target, a target dying) goes
/// through this, so a resumed march can never inherit stale combat state.
fn clear_combat_state(unit: &mut Unit) {
    unit.post = None;
    unit.target_commanded = false;
    unit.engaged = false;
    unit.chase_slot = NO_SLOT;
    unit.best_gap = f32::MAX;
    unit.hold_ticks = 0;
    unit.ally_stall = 0;
    unit.ally_min_remaining = f32::MAX;
}

/// Spare capacity kept on every installed path for [`Sim::detour`]'s inserts.
const PATH_DETOUR_SPARE: usize = 8;

/// Union-find root with path halving (over dense indices, for `start_move`'s
/// spatial clustering).
fn uf_find(parent: &mut [u32], mut x: u32) -> u32 {
    while parent[x as usize] != x {
        parent[x as usize] = parent[parent[x as usize] as usize];
        x = parent[x as usize];
    }
    x
}

/// Union two sets, keeping the smaller root as canonical (deterministic).
fn uf_union(parent: &mut [u32], a: u32, b: u32) {
    let (ra, rb) = (uf_find(parent, a), uf_find(parent, b));
    if ra != rb {
        let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
        parent[hi as usize] = lo;
    }
}

/// Install a freshly computed path, dropping the leading start point (the
/// unit is already there; nothing reads it back); an empty/degenerate path
/// means idle.
fn set_path(unit: &mut Unit, mut path: Vec<Vector2>) {
    unit.path_i = 0;
    unit.parked = false; // re-tasked: no longer settled at a goal
    unit.stall = 0; // fresh path: clear any stuck-on-corner accrual
    unit.min_remaining = f32::MAX;
    unit.ally_stall = 0; // …and any ally-blocked accrual
    unit.ally_min_remaining = f32::MAX;
    if path.len() >= 2 {
        path.remove(0);
        unit.path = path;
        // Headroom for `Sim::detour`'s waypoint inserts: a reallocation there
        // would break the allocation-free steady-state contract.
        unit.path.reserve(PATH_DETOUR_SPARE);
    } else {
        unit.path.clear();
    }
}

/// Settle a unit at its current position: clear the path and mark it parked
/// (the seed for crowd-arrival), and — unless it's mid-combat — clear a
/// stale attack-move goal, since reaching it is also how an `AttackMove`
/// completes. Shared by [`Sim::integrate`] (ran out of waypoints) and
/// [`Sim::flock`] (crowd-arrival touched a parked group-mate): both are
/// "reached the goal", just detected differently.
fn arrive_at_goal(unit: &mut Unit) {
    unit.path.clear();
    unit.path_i = 0;
    unit.parked = true;
    if unit.target.is_none() {
        unit.attack_move_goal = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mapgen::{ROOM_SIZE, rooms_map};

    fn v(x: f32, y: f32) -> Vector2 {
        Vector2::new(x, y)
    }

    fn rooms_sim(cols: usize, rows: usize, seed: u64) -> Sim {
        let (points, constraints) = rooms_map(cols, rows);
        Sim::new(points, &constraints, seed)
    }

    /// Spawn now and return the id (commands carry no return channel). No
    /// combat stats: team 0, huge health, harmless — the ~40 movement tests
    /// go through this and shouldn't have to know about combat.
    fn spawn(sim: &mut Sim, pos: Vector2, radius: f32, speed: f32) -> UnitId {
        spawn_stats(sim, pos, radius, speed, 0, f32::MAX, 0.0, 0.0, 1)
    }

    /// Spawn now with full combat stats and return the id.
    #[allow(clippy::too_many_arguments)]
    fn spawn_stats(
        sim: &mut Sim,
        pos: Vector2,
        radius: f32,
        speed: f32,
        team: u32,
        max_health: f32,
        damage: f32,
        attack_range: f32,
        attack_cooldown_ticks: u32,
    ) -> UnitId {
        sim.step(&[Command::Spawn {
            pos,
            radius,
            max_speed: speed,
            team,
            max_health,
            damage,
            attack_range,
            attack_cooldown_ticks,
        }]);
        sim.units().iter().last().unwrap().0
    }

    /// Spawn on team 1 (enemy of team 0 by the default rule) with combat
    /// stats reasonable for direct-attack/attack-move tests.
    fn spawn_enemy(sim: &mut Sim, pos: Vector2, radius: f32, speed: f32) -> UnitId {
        spawn_stats(sim, pos, radius, speed, 1, 20.0, 5.0, 8.0, 5)
    }

    fn unit(sim: &Sim, id: UnitId) -> &Unit {
        sim.units().get(id).unwrap()
    }

    fn step_n(sim: &mut Sim, n: usize) {
        for _ in 0..n {
            sim.step(&[]);
        }
    }

    fn dist(a: Vector2, b: Vector2) -> f32 {
        ((a.x - b.x).powi(2) + (a.y - b.y).powi(2)).sqrt()
    }

    #[test]
    fn test_spawn_and_idle() {
        let mut sim = rooms_sim(2, 2, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 20.0);
        step_n(&mut sim, 5);
        let u = unit(&sim, id);
        assert_eq!(u.pos, v(50.0, 50.0));
        assert!(!u.is_moving());
    }

    #[test]
    fn test_reaches_goal_within_tick_bound() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
        let goal = v(250.0, 50.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal,
        }]);
        let len = unit(&sim, id).remaining_len();
        assert!(len > 0.0, "path must exist");
        let bound = (len / (30.0 * DT)).ceil() as usize + 2;
        step_n(&mut sim, bound);
        let u = unit(&sim, id);
        assert_eq!(u.pos, goal, "must snap exactly onto the goal");
        assert!(!u.is_moving());
    }

    #[test]
    fn test_waypoint_carry_covers_exact_distance() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 25.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(250.0, 60.0),
        }]);
        // While en route (not the arrival tick), each tick moves exactly
        // speed * DT of polyline distance, including across corners.
        let total = unit(&sim, id).remaining_len();
        let full_ticks = (total / (25.0 * DT)) as usize - 1;
        for _ in 0..full_ticks {
            let before = unit(&sim, id).pos;
            let i_before = unit(&sim, id).path_i;
            sim.step(&[]);
            let u = unit(&sim, id);
            let mut travelled = 0.0;
            let mut prev = before;
            for wp in i_before..u.path_i {
                travelled += dist(prev, u.path[wp as usize]);
                prev = u.path[wp as usize];
            }
            travelled += dist(prev, u.pos);
            assert!(
                (travelled - 25.0 * DT).abs() < 1e-3,
                "tick travelled {travelled}, want {}",
                25.0 * DT
            );
        }
    }

    /// Does segment a-b strictly cross segment c-d?
    fn segments_cross(a: Vector2, b: Vector2, c: Vector2, d: Vector2) -> bool {
        let orient = |p: Vector2, q: Vector2, r: Vector2| {
            ((q.x - p.x) as f64 * (r.y - p.y) as f64) - ((q.y - p.y) as f64 * (r.x - p.x) as f64)
        };
        let (o1, o2) = (orient(a, b, c), orient(a, b, d));
        let (o3, o4) = (orient(c, d, a), orient(c, d, b));
        o1 * o2 < 0.0 && o3 * o4 < 0.0
    }

    fn assert_no_wall_crossing(sim: &Sim, walls: &[(Vector2, Vector2)]) {
        for (_, u) in sim.units().iter() {
            for &(wa, wb) in walls {
                assert!(
                    !segments_cross(u.prev_pos, u.pos, wa, wb),
                    "unit segment {:?}->{:?} crosses wall {wa:?}-{wb:?}",
                    u.prev_pos,
                    u.pos
                );
            }
        }
    }

    fn wall_segments(points: &[Vector2], constraints: &[(u32, u32)]) -> Vec<(Vector2, Vector2)> {
        constraints
            .iter()
            .map(|&(a, b)| (points[a as usize], points[b as usize]))
            .collect()
    }

    #[test]
    fn test_movement_never_crosses_walls() {
        let (points, constraints) = rooms_map(3, 3);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 7);
        let mut ids = Vec::new();
        for i in 0..6 {
            let p = v(30.0 + 40.0 * (i % 3) as f32, 30.0 + 40.0 * (i / 3) as f32);
            ids.push(spawn(&mut sim, p, 6.0, 40.0));
        }
        sim.step(&[Command::Move {
            units: ids,
            goal: v(2.5 * ROOM_SIZE, 2.5 * ROOM_SIZE),
        }]);
        for _ in 0..600 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
    }

    #[test]
    fn test_mid_walk_move_replaces_path() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 20.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(250.0, 50.0),
        }]);
        step_n(&mut sim, 10);
        let new_goal = v(20.0, 20.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: new_goal,
        }]);
        assert_eq!(*unit(&sim, id).path.last().unwrap(), new_goal);
        step_n(&mut sim, 600);
        assert_eq!(unit(&sim, id).pos, new_goal);
    }

    #[test]
    fn test_queued_moves_run_consecutively() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
        let (g1, g2) = (v(250.0, 50.0), v(150.0, 50.0));
        sim.step(&[Command::Move {
            units: vec![id],
            goal: g1,
        }]);
        sim.step(&[Command::Queue {
            units: vec![id],
            order: Order::Move { goal: g2 },
        }]);
        assert_eq!(
            unit(&sim, id).orders.len(),
            1,
            "second move is queued, not run"
        );

        // The queue drains the tick g1 is reached; g2 must not begin before then.
        let mut reached_g1 = false;
        for _ in 0..1000 {
            sim.step(&[]);
            let u = unit(&sim, id);
            if u.orders.is_empty() && !reached_g1 {
                reached_g1 = true;
                assert_eq!(
                    u.pos, g1,
                    "first goal must be reached before the second begins"
                );
            }
            if reached_g1 && !u.is_moving() {
                break;
            }
        }
        assert!(reached_g1, "first queued goal was never reached");
        assert_eq!(unit(&sim, id).pos, g2, "must end at the second queued goal");
    }

    #[test]
    fn test_queue_on_idle_unit_starts_at_once() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
        let goal = v(150.0, 50.0);
        // advance_orders runs in the same step the command lands, so an idle
        // unit begins a queued order immediately (no idle tick).
        sim.step(&[Command::Queue {
            units: vec![id],
            order: Order::Move { goal },
        }]);
        let u = unit(&sim, id);
        assert!(u.is_moving());
        assert!(u.orders.is_empty());
        assert_eq!(*u.path.last().unwrap(), goal);
    }

    #[test]
    fn test_plain_move_clears_queue() {
        let mut sim = rooms_sim(3, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(250.0, 50.0),
        }]);
        sim.step(&[Command::Queue {
            units: vec![id],
            order: Order::Move {
                goal: v(150.0, 50.0),
            },
        }]);
        assert_eq!(unit(&sim, id).orders.len(), 1);
        let new_goal = v(200.0, 50.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: new_goal,
        }]);
        let u = unit(&sim, id);
        assert!(
            u.orders.is_empty(),
            "a plain move must cancel queued orders"
        );
        assert_eq!(*u.path.last().unwrap(), new_goal);
    }

    #[test]
    fn test_unreachable_goal_idles() {
        let mut sim = rooms_sim(2, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 20.0);
        // Far outside the mesh.
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(-500.0, -500.0),
        }]);
        let u = unit(&sim, id);
        assert!(!u.is_moving());
        assert_eq!(u.pos, v(50.0, 50.0));
        step_n(&mut sim, 5);
    }

    #[test]
    fn test_stale_unit_id_ignored() {
        let mut sim = rooms_sim(2, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 5.0, 20.0);
        let stale = UnitId {
            index: id.index,
            generation: id.generation + 1,
        };
        sim.step(&[Command::Move {
            units: vec![stale],
            goal: v(150.0, 50.0),
        }]);
        assert!(!unit(&sim, id).is_moving());
    }

    #[test]
    fn test_obstacle_insert_detours_and_remove_restores() {
        let mut sim = rooms_sim(2, 1, 1);
        // Radius 2: fits the ~5-wide gaps the obstacle leaves in the doorway.
        let id = spawn(&mut sim, v(50.0, 50.0), 2.0, 20.0);
        let goal = v(150.0, 50.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal,
        }]);
        let base_len = unit(&sim, id).remaining_len();

        // Narrow the doorway (gap x=100, y in [35, 65]).
        sim.step(&[Command::AddObstacle {
            points: vec![v(98.0, 40.0), v(102.0, 40.0), v(102.0, 60.0), v(98.0, 60.0)],
        }]);
        let u = unit(&sim, id);
        assert!(u.is_moving(), "detour must exist");
        let detour_len = u.remaining_len();
        assert!(
            detour_len > base_len,
            "detour {detour_len} not longer than base {base_len}"
        );

        sim.step(&[Command::RemoveObstacle {
            id: ObstacleId::from_raw(0),
        }]);
        let u = unit(&sim, id);
        assert!(u.is_moving());
        step_n(&mut sim, 600);
        assert_eq!(unit(&sim, id).pos, goal);
    }

    #[test]
    fn test_obstacle_making_goal_unreachable_idles_unit() {
        let mut sim = rooms_sim(2, 1, 1);
        let id = spawn(&mut sim, v(50.0, 50.0), 8.0, 20.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal: v(150.0, 50.0),
        }]);
        assert!(unit(&sim, id).is_moving());
        // Leaves ≤5-wide gaps beside the door — too tight for radius 8.
        sim.step(&[Command::AddObstacle {
            points: vec![v(98.0, 40.0), v(102.0, 40.0), v(102.0, 60.0), v(98.0, 60.0)],
        }]);
        assert!(!unit(&sim, id).is_moving(), "blocked unit must idle");
    }

    #[test]
    fn test_determinism_same_stream_same_hashes() {
        let script = |sim: &mut Sim| -> (Vec<u64>, usize) {
            let mut hashes = Vec::new();
            let mut ids = Vec::new();
            for t in 0..120u32 {
                let mut cmds = Vec::new();
                if t < 20 {
                    cmds.push(Command::Spawn {
                        pos: v(20.0 + 7.0 * t as f32, 30.0 + 5.0 * (t % 3) as f32),
                        radius: 5.0,
                        max_speed: 20.0 + t as f32,
                        team: t % 2,
                        max_health: 30.0,
                        damage: 4.0,
                        attack_range: 6.0,
                        attack_cooldown_ticks: 6,
                    });
                }
                if t == 25 {
                    ids = sim.units().iter().map(|(id, _)| id).collect();
                    // Pair each team-0 unit directly against a team-1 unit so
                    // they clash mid-stream regardless of where a march would
                    // have taken them — kills exercise death, cooldown and
                    // target-handle hashing, not just movement.
                    let (team0, team1): (Vec<UnitId>, Vec<UnitId>) = ids
                        .iter()
                        .partition(|&&id| sim.units().get(id).unwrap().team == 0);
                    for (&x, &y) in team0.iter().zip(team1.iter()) {
                        cmds.push(Command::Attack {
                            units: vec![x],
                            target: y,
                        });
                        cmds.push(Command::Attack {
                            units: vec![y],
                            target: x,
                        });
                    }
                    // Queue a follow-on order so the order queue is hashed too.
                    cmds.push(Command::Queue {
                        units: ids.clone(),
                        order: Order::Move {
                            goal: v(60.0, 60.0),
                        },
                    });
                }
                if t == 40 {
                    cmds.push(Command::AddObstacle {
                        points: vec![
                            v(120.0, 40.0),
                            v(140.0, 40.0),
                            v(140.0, 60.0),
                            v(120.0, 60.0),
                        ],
                    });
                }
                if t == 80 {
                    cmds.push(Command::RemoveObstacle {
                        id: ObstacleId::from_raw(0),
                    });
                }
                if t == 90 {
                    cmds.push(Command::Move {
                        units: ids.clone(),
                        goal: v(50.0, 250.0),
                    });
                }
                sim.step(&cmds);
                hashes.push(sim.state_hash());
            }
            (hashes, sim.units().len())
        };
        let a = script(&mut rooms_sim(3, 3, 42));
        let b = script(&mut rooms_sim(3, 3, 42));

        assert_eq!(a, b, "same command stream must reproduce every tick hash");
        assert!(
            a.1 < 20,
            "the mid-stream clash must actually kill someone (population {}), or deaths/cooldowns/target handles never exercise state_hash",
            a.1
        );

        let c = script(&mut rooms_sim(3, 3, 43));
        assert_eq!(a.0.len(), c.0.len());
    }

    /// The clash above is a line of duels; this one is the positioning path —
    /// a blob that crushes, concedes onto approach slots, detours around
    /// allies and refills the ring after a death. Every field those mechanisms
    /// added has to reach `state_hash`, or a divergence here goes unseen.
    #[test]
    fn test_determinism_blocked_fight_same_hashes() {
        let script = || {
            let (mut sim, attackers, defender) = blob_fight(12, false);
            let mut hashes = Vec::new();
            for t in 0..400u32 {
                let mut cmds = Vec::new();
                if t == 250 {
                    // Kill the front rank: the survivors must re-slot and
                    // close, which is the refill path.
                    for &id in &attackers {
                        if sim.units().get(id).is_some_and(|u| u.engaged) {
                            cmds.push(Command::Damage {
                                unit: id,
                                amount: 1.0e9,
                            });
                        }
                    }
                }
                sim.step(&cmds);
                hashes.push(sim.state_hash());
            }
            let survivors = attackers
                .iter()
                .filter(|&&id| sim.units().get(id).is_some())
                .count();
            (hashes, survivors, unit(&sim, defender).health)
        };
        let (a, b) = (script(), script());
        assert_eq!(a, b, "a blocked, re-slotting fight must replay bit-exactly");
        assert!(
            a.1 < 12 && a.1 > 0,
            "the scripted kill must remove part of the ring: {} left",
            a.1
        );
    }

    #[test]
    fn test_determinism_bitwise_state() {
        let run = || {
            let mut sim = rooms_sim(2, 2, 9);
            let mut ids = Vec::new();
            for i in 0..8 {
                ids.push(spawn(&mut sim, v(25.0 + 6.0 * i as f32, 40.0), 5.0, 25.0));
            }
            sim.step(&[Command::Move {
                units: ids,
                goal: v(160.0, 160.0),
            }]);
            step_n(&mut sim, 200);
            sim
        };
        let (a, b) = (run(), run());
        assert_eq!(a.state_hash(), b.state_hash());
        for ((ia, ua), (ib, ub)) in a.units().iter().zip(b.units().iter()) {
            assert_eq!(ia, ib);
            assert_eq!(ua.pos.x.to_bits(), ub.pos.x.to_bits());
            assert_eq!(ua.pos.y.to_bits(), ub.pos.y.to_bits());
            assert_eq!(ua.path_i, ub.path_i);
            assert_eq!(ua.path.len(), ub.path.len());
        }
    }

    #[test]
    fn test_separation_disperses_clump_without_overlap() {
        let mut sim = rooms_sim(2, 2, 5);
        // 9 units stacked nearly on top of each other.
        for i in 0..9 {
            spawn(
                &mut sim,
                v(100.0 + 0.01 * i as f32, 100.0 + 0.013 * i as f32),
                4.0,
                30.0,
            );
        }
        step_n(&mut sim, 400);
        let units: Vec<&Unit> = sim.units().iter().map(|(_, u)| u).collect();
        for i in 0..units.len() {
            for j in (i + 1)..units.len() {
                let d = dist(units[i].pos, units[j].pos);
                let min = units[i].radius + units[j].radius;
                // At rest (no path pull), separation resolves overlap fully.
                assert!(
                    d >= min - 0.1,
                    "units {i},{j} still overlap: dist {d} < {min}"
                );
            }
        }
    }

    #[test]
    fn test_separation_disperses_without_ping_pong() {
        // Soft separation: while a stacked clump eases apart, no unit may
        // reverse direction tick to tick (the visual jitter signature of
        // overshooting hard resolution).
        let mut sim = rooms_sim(2, 2, 5);
        for i in 0..9 {
            spawn(
                &mut sim,
                v(100.0 + 0.01 * i as f32, 100.0 + 0.013 * i as f32),
                4.0,
                30.0,
            );
        }
        let mut prev_disp: Vec<Vector2> = vec![Vector2::ZERO; 9];
        for tick in 0..100 {
            sim.step(&[]);
            for (i, (_, u)) in sim.units().iter().enumerate() {
                let d = u.pos - u.prev_pos;
                let moved = d.length_squared() > 1e-6;
                let was_moving = prev_disp[i].length_squared() > 1e-6;
                if moved && was_moving {
                    assert!(
                        d.dot(prev_disp[i]) >= 0.0,
                        "unit {i} reversed direction at tick {tick}: {:?} -> {d:?}",
                        prev_disp[i]
                    );
                }
                prev_disp[i] = d;
            }
        }
    }

    #[test]
    fn test_group_packed_against_edge_stays_inside() {
        // A large, fast group spam-clicked to a goal hard against the map's
        // right edge (x=200): the per-tick separation push — large for fast
        // units — must never shove a unit clean across the boundary.
        let (points, constraints) = rooms_map(2, 2); // 200×200 open-ish map
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 5);
        let goal = v(197.0, 100.0); // ~3 px from the x=200 boundary
        let mut ids = Vec::new();
        for i in 0..50 {
            ids.push(spawn(
                &mut sim,
                v(110.0 + 6.0 * (i % 10) as f32, 70.0 + 6.0 * (i / 10) as f32),
                5.0,
                200.0, // fast: large per-tick separation push
            ));
        }
        for _ in 0..400 {
            // Re-issue the same move every few ticks (spam-click).
            let cmd = if sim.tick().is_multiple_of(5) {
                vec![Command::Move {
                    units: ids.clone(),
                    goal,
                }]
            } else {
                vec![]
            };
            sim.step(&cmd);
            assert_no_wall_crossing(&sim, &walls);
            // Every unit centre must stay inside the map (no unit past x=200).
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 200.0 + 1e-3,
                    "unit pushed across the edge wall to {:?}",
                    u.pos
                );
            }
        }
    }

    #[test]
    fn test_separation_never_pushes_through_walls() {
        let (points, constraints) = rooms_map(2, 2);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 5);
        // Clump right next to the interior wall junction at (100, 100).
        for i in 0..12 {
            spawn(
                &mut sim,
                v(93.0 + 0.01 * i as f32, 93.0 + 0.01 * i as f32),
                5.0,
                30.0,
            );
        }
        for _ in 0..300 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
        // And the clamp holds them clear of the walls (not just non-crossing).
        for (_, u) in sim.units().iter() {
            for &(wa, wb) in &walls {
                let c = closest_on_segment(u.pos, wa, wb);
                assert!(
                    dist(u.pos, c) >= u.radius - 1e-3,
                    "unit at {:?} overlaps wall {wa:?}-{wb:?}",
                    u.pos
                );
            }
        }
    }

    /// 200x200 map with a 3px-thick wall from (150,`gap`) up to the top edge,
    /// leaving a `gap`-tall slot to the bottom boundary.
    fn thin_wall_map(gap: f32) -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let pts = vec![
            v(0.0, 0.0),
            v(200.0, 0.0),
            v(200.0, 200.0),
            v(0.0, 200.0),
            v(150.0, gap),
            v(153.0, gap),
            v(153.0, 200.0),
            v(150.0, 200.0),
        ];
        let cons = vec![
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
        ];
        (pts, cons)
    }

    #[test]
    fn test_crowd_never_squeezes_through_subdiameter_gap() {
        // A 6px slot, too tight for radius-5 (diameter-10) units. A single
        // unit stalls at the entrance; sustained crowd pressure must not
        // squeeze the front units through it either.
        let (points, constraints) = thin_wall_map(6.0);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 7);
        let goal = v(175.0, 100.0);
        // Spawn all 40 in one step — the `spawn()` helper runs its own
        // `sim.step` each, which would let early spawns settle over dozens of
        // unchecked ticks before the crowd-pressure loop below even starts.
        let spawn_cmds: Vec<Command> = (0..40)
            .map(|i| Command::Spawn {
                // 8 columns * 3px stay well clear of the wall at x=150.
                pos: v(115.0 + 3.0 * (i % 8) as f32, 20.0 + 4.0 * (i / 8) as f32),
                radius: 5.0,
                max_speed: 60.0,
                team: 0,
                max_health: f32::MAX,
                damage: 0.0,
                attack_range: 0.0,
                attack_cooldown_ticks: 1,
            })
            .collect();
        sim.step(&spawn_cmds);
        let ids: Vec<UnitId> = sim.units().iter().map(|(id, _)| id).collect();
        assert_no_wall_crossing(&sim, &walls);
        for _ in 0..1000 {
            let cmd = if sim.tick().is_multiple_of(5) {
                vec![Command::Move {
                    units: ids.clone(),
                    goal,
                }]
            } else {
                vec![]
            };
            sim.step(&cmd);
            assert_no_wall_crossing(&sim, &walls);
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 150.0 + 1e-3,
                    "unit squeezed through the sub-diameter gap to {:?}",
                    u.pos
                );
            }
        }
    }

    #[test]
    fn test_sealed_goal_is_refused_without_a_spin() {
        // A goal in a closed room must be noticed once and dropped, not
        // repathed at forever, even when the order is re-issued.
        //
        // Unlike `test_unreachable_goal_idles`, the goal here sits on a good
        // face in a different component (the other refusal path), with a crowd
        // and 900 ticks of re-clicking that a single idle unit cannot show.
        // Every quantity is a flat must-be-zero, which is why this is a test
        // and not a scenario: there is no "slightly better" spinning.
        let mut b = vec![
            v(0.0, 0.0),
            v(400.0, 0.0),
            v(400.0, 400.0),
            v(0.0, 400.0),
            v(160.0, 160.0),
            v(240.0, 160.0),
            v(240.0, 240.0),
            v(160.0, 240.0),
        ];
        let cons = vec![
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
        ];
        let walls = wall_segments(&b, &cons);
        let mut sim = Sim::new(std::mem::take(&mut b), &cons, 0x5EA1);
        let goal = v(200.0, 200.0); // dead centre of the sealed room
        let mut ids = Vec::new();
        let mut starts = Vec::new();
        for i in 0..20 {
            let p = v(40.0 + 14.0 * (i % 5) as f32, 40.0 + 14.0 * (i / 5) as f32);
            ids.push(spawn(&mut sim, p, 5.0, 40.0));
            starts.push(p);
        }
        for t in 0..900u32 {
            let cmd = if t.is_multiple_of(100) {
                vec![Command::Move {
                    units: ids.clone(),
                    goal,
                }]
            } else {
                Vec::new()
            };
            sim.step(&cmd);
            assert_no_wall_crossing(&sim, &walls);
        }
        for (&id, &start) in ids.iter().zip(&starts) {
            let u = unit(&sim, id);
            assert!(u.path.is_empty(), "a refused goal must leave no path");
            assert!(!u.parked, "never reached, so never parked");
            assert_eq!(u.stall, 0, "not stuck against anything: it never set off");
            assert_eq!(u.ally_stall, 0);
            assert!(
                (u.pos - start).length() < 1e-3,
                "walked {:?} toward an unreachable goal",
                u.pos - start
            );
        }
    }

    #[test]
    fn test_dense_crowd_through_a_doorway_never_crosses_a_wall() {
        // `test_movement_never_crosses_walls` under funnel pressure: sixty
        // bodies on one 30-wide door, where the clamp holds hardest.
        let (points, constraints) = rooms_map(2, 2);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 0xD00B);
        let mut ids = Vec::new();
        for i in 0..60 {
            let p = v(12.0 + 12.0 * (i % 7) as f32, 12.0 + 12.0 * (i / 7) as f32);
            ids.push(spawn(&mut sim, p, 5.0, 40.0));
        }
        sim.step(&[Command::Move {
            units: ids,
            goal: v(150.0, 150.0),
        }]);
        for _ in 0..900 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
    }

    #[test]
    fn test_crowd_never_squeezes_through_near_exact_gap() {
        // Harsher than the 6px-vs-10px repro above: the gap is only *just* too
        // small, and the crowd spam-clicks the entrance every tick — max
        // pressure against the case where clamp float-noise slack matters most.
        let (points, constraints) = thin_wall_map(9.9); // diameter 10: 0.1px too tight.
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 11);
        let goal = v(151.5, 3.0); // dead centre of the gap: max pressure at the pinch.
        let spawn_cmds: Vec<Command> = (0..60)
            .map(|i| Command::Spawn {
                // Columns stay well clear (max x=133.5) of the wall at x=150.
                pos: v(120.0 + 1.5 * (i % 10) as f32, 20.0 + 3.0 * (i / 10) as f32),
                radius: 5.0,
                max_speed: 80.0,
                team: 0,
                max_health: f32::MAX,
                damage: 0.0,
                attack_range: 0.0,
                attack_cooldown_ticks: 1,
            })
            .collect();
        sim.step(&spawn_cmds);
        let ids: Vec<UnitId> = sim.units().iter().map(|(id, _)| id).collect();
        assert_no_wall_crossing(&sim, &walls);
        for _ in 0..3000 {
            sim.step(&[Command::Move {
                units: ids.clone(),
                goal,
            }]);
            assert_no_wall_crossing(&sim, &walls);
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 150.0 + 1e-3,
                    "unit squeezed through the near-exact gap to {:?}",
                    u.pos
                );
            }
        }
    }

    /// Mean distance of the alive units from their centroid (group spread).
    fn spread(sim: &Sim) -> f32 {
        let ps: Vec<Vector2> = sim.units().iter().map(|(_, u)| u.pos).collect();
        let n = ps.len() as f32;
        let mut c = Vector2::ZERO;
        for &p in &ps {
            c += p;
        }
        c *= 1.0 / n;
        ps.iter().map(|&p| dist(p, c)).sum::<f32>() / n
    }

    /// Ticks until `id` stops moving (arrives); panics if it never does.
    fn arrival_tick(sim: &mut Sim, id: UnitId, max: u64) -> u64 {
        for t in 1..=max {
            sim.step(&[]);
            if !unit(sim, id).is_moving() {
                return t;
            }
        }
        panic!("unit did not arrive within {max} ticks");
    }

    #[test]
    fn test_crowd_arrival_stops_short_of_goal() {
        // A reaches the goal and parks; B, behind it and same group, stops on
        // contact instead of driving onto the exact goal point.
        let mut sim = rooms_sim(2, 1, 8);
        let goal = v(150.0, 50.0);
        let a = spawn(&mut sim, v(135.0, 50.0), 5.0, 30.0);
        let b = spawn(&mut sim, v(110.0, 50.0), 5.0, 30.0);
        sim.step(&[Command::Move {
            units: vec![a, b],
            goal,
        }]);
        step_n(&mut sim, 200);
        let (ua, ub) = (unit(&sim, a), unit(&sim, b));
        assert!(ua.parked && ub.parked, "both must settle (parked)");
        assert!(!ua.is_moving() && !ub.is_moving());
        let (da, db) = (dist(ua.pos, goal), dist(ub.pos, goal));
        assert!(
            da.min(db) < 6.0,
            "one unit settles at the goal: {}",
            da.min(db)
        );
        assert!(
            da.max(db) > 7.0,
            "the other stops short, not crammed onto the goal: {}",
            da.max(db)
        );
        assert!(dist(ua.pos, ub.pos) >= 9.5, "must not hard-overlap");
    }

    #[test]
    fn test_group_settles_as_blob_without_crush() {
        // A group arriving at one goal settles into a packed blob around it,
        // not a hard pile crammed onto the goal point.
        let mut sim = rooms_sim(1, 1, 13);
        let goal = v(50.0, 50.0);
        let mut ids = Vec::new();
        for i in 0..16 {
            ids.push(spawn(
                &mut sim,
                v(12.0 + 4.0 * (i % 4) as f32, 12.0 + 4.0 * (i / 4) as f32),
                5.0,
                30.0,
            ));
        }
        sim.step(&[Command::Move { units: ids, goal }]);
        step_n(&mut sim, 300);
        let us: Vec<&Unit> = sim.units().iter().map(|(_, u)| u).collect();
        assert!(
            us.iter().all(|u| u.parked && !u.is_moving()),
            "whole group must settle"
        );
        // No hard clump: nothing overlaps beyond tolerance.
        for i in 0..us.len() {
            for j in (i + 1)..us.len() {
                let d = dist(us[i].pos, us[j].pos);
                assert!(
                    d >= us[i].radius + us[j].radius - 0.5,
                    "units {i},{j} overlap: {d}"
                );
            }
        }
        // Only a couple reach the goal centre; the rest stop around it.
        let at_goal = us.iter().filter(|u| dist(u.pos, goal) < 5.0).count();
        assert!(
            at_goal <= 2,
            "units crammed onto the goal centre: {at_goal}"
        );
        // Blob straddles the goal (not piled up short of it), centroid within
        // the group's own arrival radius.
        let axis = (goal - v(12.0, 12.0)).normalized();
        let (mut behind, mut past) = (f32::MAX, f32::MIN);
        for u in &us {
            let t = (u.pos - goal).dot(axis);
            behind = behind.min(t);
            past = past.max(t);
        }
        assert!(
            behind < 0.0 && past > 0.0,
            "blob must straddle the goal, not stop short: behind={behind} past={past}"
        );
        let r = us[0].radius;
        let arrival_r = r
            * (ARRIVAL_RADIUS_FACTOR.get() * (us.len() as f32).sqrt()).max(ARRIVAL_MIN_RADII.get());
        let mut c = Vector2::ZERO;
        for u in &us {
            c += u.pos;
        }
        c *= 1.0 / us.len() as f32;
        assert!(
            dist(c, goal) < arrival_r,
            "group centre outside its arrival radius: {} (arrival_r={arrival_r})",
            dist(c, goal)
        );
    }

    #[test]
    fn test_new_move_unparks_settled_group() {
        // A re-order clears `parked` and the group moves off again.
        let mut sim = rooms_sim(2, 1, 2);
        let mut ids = Vec::new();
        for i in 0..4 {
            ids.push(spawn(&mut sim, v(120.0 + 5.0 * i as f32, 50.0), 5.0, 30.0));
        }
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal: v(150.0, 50.0),
        }]);
        step_n(&mut sim, 150);
        assert!(sim.units().iter().all(|(_, u)| u.parked), "group settles");
        sim.step(&[Command::Move {
            units: ids,
            goal: v(50.0, 50.0),
        }]);
        assert!(
            sim.units().iter().all(|(_, u)| !u.parked),
            "re-move must clear parked"
        );
        step_n(&mut sim, 250);
        let (mut c, n) = (Vector2::ZERO, sim.units().len() as f32);
        for (_, u) in sim.units().iter() {
            c += u.pos;
        }
        c *= 1.0 / n;
        assert!(dist(c, v(50.0, 50.0)) < 20.0, "group reaches the new goal");
    }

    #[test]
    fn test_group_rounding_corner_none_stuck() {
        // A group rounding a wall's end: the pack shoves some units onto its
        // face, where their line to the next waypoint cuts through it and the
        // wall clamp pins them. The stall-repath must route them around —
        // without it ~2 units stay pinned on the wall.
        let mut sim = rooms_sim(1, 1, 1);
        sim.step(&[Command::AddObstacle {
            points: vec![v(72.0, 10.0), v(78.0, 10.0), v(78.0, 75.0), v(72.0, 75.0)],
        }]);
        let goal = v(90.0, 40.0);
        let mut ids = Vec::new();
        for i in 0..12 {
            ids.push(spawn(
                &mut sim,
                v(20.0 + 5.0 * (i % 6) as f32, 35.0 + 5.0 * (i / 6) as f32),
                5.0,
                25.0,
            ));
        }
        sim.step(&[Command::Move { units: ids, goal }]);
        step_n(&mut sim, 900);
        for (_, u) in sim.units().iter() {
            assert!(
                dist(u.pos, goal) < 45.0,
                "unit stuck short of the goal at {:?} (dist {:.1})",
                u.pos,
                dist(u.pos, goal)
            );
        }
    }

    #[test]
    fn test_cohesion_tightens_group_spread() {
        // Same scenario twice: one Move groups all units (cohesion on) vs one
        // Move per unit (own group each, cohesion off). Paths and separation
        // are identical, so the spread difference is cohesion alone.
        let run = |grouped: bool| -> f32 {
            let mut sim = rooms_sim(3, 1, 7);
            let mut ids = Vec::new();
            for i in 0..6 {
                ids.push(spawn(&mut sim, v(30.0, 20.0 + 12.0 * i as f32), 5.0, 40.0));
            }
            let goal = v(250.0, 50.0);
            let cmds: Vec<Command> = if grouped {
                vec![Command::Move { units: ids, goal }]
            } else {
                ids.into_iter()
                    .map(|id| Command::Move {
                        units: vec![id],
                        goal,
                    })
                    .collect()
            };
            sim.step(&cmds);
            step_n(&mut sim, 70);
            spread(&sim)
        };
        let tight = run(true);
        let loose = run(false);
        assert!(
            tight < loose,
            "cohesion must tighten the group: grouped {tight} vs ungrouped {loose}"
        );
    }

    #[test]
    fn test_cohesion_does_not_brake_leader() {
        // A fast leader grouped with slow units behind it must arrive about
        // when it would solo: the cohesion pull toward the lagging group is
        // backward, and the heading-opposing component is dropped.
        let goal = v(250.0, 50.0);
        let solo = {
            let mut sim = rooms_sim(3, 1, 3);
            let lead = spawn(&mut sim, v(30.0, 50.0), 5.0, 60.0);
            sim.step(&[Command::Move {
                units: vec![lead],
                goal,
            }]);
            arrival_tick(&mut sim, lead, 400)
        };
        let grouped = {
            let mut sim = rooms_sim(3, 1, 3);
            let lead = spawn(&mut sim, v(30.0, 50.0), 5.0, 60.0);
            let mut ids = vec![lead];
            for i in 0..5 {
                ids.push(spawn(&mut sim, v(20.0 - 3.0 * i as f32, 50.0), 5.0, 12.0));
            }
            sim.step(&[Command::Move { units: ids, goal }]);
            arrival_tick(&mut sim, lead, 400)
        };
        assert!(
            grouped.abs_diff(solo) <= 1,
            "cohesion braked the leader: solo {solo}, grouped {grouped}"
        );
    }

    #[test]
    fn test_stragglers_do_not_drag_group() {
        // The front of a moving pack must not wait for a slow straggler added
        // to its group: its arrival barely changes whether the straggler is
        // present (backward pull dropped, no group-speed coupling).
        let goal = v(250.0, 50.0);
        let front_arrival = |with_straggler: bool| -> u64 {
            let mut sim = rooms_sim(3, 1, 11);
            let mut ids = Vec::new();
            for i in 0..4 {
                ids.push(spawn(&mut sim, v(30.0 + 12.0 * i as f32, 50.0), 5.0, 50.0));
            }
            let front = ids[3];
            if with_straggler {
                ids.push(spawn(&mut sim, v(10.0, 50.0), 5.0, 8.0));
            }
            sim.step(&[Command::Move { units: ids, goal }]);
            arrival_tick(&mut sim, front, 600)
        };
        let alone = front_arrival(false);
        let dragged = front_arrival(true);
        assert!(
            dragged.abs_diff(alone) <= 2,
            "straggler dragged the group: without {alone}, with {dragged}"
        );
    }

    #[test]
    fn test_cross_group_independence() {
        // A trailing unit behind a leader, goal on the travel axis (so the
        // straight-line fan stays neutral and cohesion is the only differing
        // effect). Same group: cohesion pulls the straggler forward, closing
        // the gap. Different groups: no cross-group pull, gap holds.
        let gap_after = |same_group: bool| -> f32 {
            let mut sim = rooms_sim(3, 1, 4);
            let a = spawn(&mut sim, v(50.0, 50.0), 5.0, 30.0);
            let b = spawn(&mut sim, v(30.0, 50.0), 5.0, 30.0);
            let goal = v(250.0, 50.0);
            let cmds = if same_group {
                vec![Command::Move {
                    units: vec![a, b],
                    goal,
                }]
            } else {
                vec![
                    Command::Move {
                        units: vec![a],
                        goal,
                    },
                    Command::Move {
                        units: vec![b],
                        goal,
                    },
                ]
            };
            sim.step(&cmds);
            step_n(&mut sim, 40);
            dist(unit(&sim, a).pos, unit(&sim, b).pos)
        };
        let same = gap_after(true);
        let cross = gap_after(false);
        assert!(
            same < cross,
            "different groups must not cohere: same-group gap {same}, cross-group {cross}"
        );
    }

    #[test]
    fn test_idle_units_do_not_cohere() {
        // An idle unit (never commanded → group 0, no path) ignores cohesion:
        // a group marching within cohesion range must not drag it.
        let mut sim = rooms_sim(3, 1, 6);
        let idle = spawn(&mut sim, v(60.0, 82.0), 5.0, 30.0);
        let idle_pos = unit(&sim, idle).pos;
        let mut ids = Vec::new();
        for i in 0..5 {
            ids.push(spawn(&mut sim, v(30.0 + 8.0 * i as f32, 60.0), 5.0, 30.0));
        }
        sim.step(&[Command::Move {
            units: ids,
            goal: v(250.0, 60.0),
        }]);
        step_n(&mut sim, 20);
        assert_eq!(unit(&sim, idle).pos, idle_pos, "idle unit was dragged");
    }

    #[test]
    fn test_steady_state_step_is_allocation_free() {
        // Open single room (no interior corners → no stuck-on-corner repaths,
        // which legitimately allocate a fresh path and aren't steady state).
        let mut sim = rooms_sim(1, 1, 3);
        let mut ids = Vec::new();
        for i in 0..32 {
            ids.push(spawn(
                &mut sim,
                v(15.0 + 6.0 * (i % 8) as f32, 25.0 + 6.0 * (i / 8) as f32),
                5.0,
                20.0,
            ));
        }
        sim.step(&[Command::Move {
            units: ids,
            goal: v(50.0, 50.0),
        }]);
        // Warm scratch buffers (grid, dense arrays, locate paths) and let the
        // group fully settle into its blob, so the measured window is steady
        // state (the soft separation push settles gradually).
        step_n(&mut sim, 120);
        let allocs = crate::alloc_counter::count_allocs(|| {
            for _ in 0..20 {
                sim.step(&[]);
            }
        });
        assert_eq!(allocs, 0, "steady-state step must not allocate");
    }

    /// The marching case above never touches combat's per-tick lists, the
    /// station scan or the detour's `path.insert` — all of which live in a
    /// fight. A settled fight is one where every attacker is standing on a
    /// station it can shoot from, and that must cost nothing per tick.
    ///
    /// Deliberately *not* the melee blob: there, more attackers want stations
    /// than the ring around one body has, so the surplus keeps re-goaling —
    /// which legitimately allocates the one `Vec` `find_path_abstract`
    /// returns, and would leave this test measuring how crowded the fixture is
    /// rather than whether the tick allocates.
    #[test]
    fn test_settled_fight_is_allocation_free() {
        let mut sim = arena_sim(600.0, 600.0, 31);
        let defender = spawn_stats(&mut sim, DEF, 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        let attackers: Vec<UnitId> = (0..8)
            .map(|i| {
                spawn_stats(
                    &mut sim,
                    v(
                        250.0 - (i / 4) as f32 * 12.0,
                        DEF.y - 18.0 + (i % 4) as f32 * 12.0,
                    ),
                    5.0,
                    30.0,
                    0,
                    1.0e6,
                    1.0,
                    20.0,
                    1,
                )
            })
            .collect();
        sim.step(&[Command::Attack {
            units: attackers.clone(),
            target: defender,
        }]);
        step_n(&mut sim, 300);
        assert!(
            attackers.iter().all(|&id| unit(&sim, id).engaged),
            "every attacker must be stationed before measuring"
        );
        let allocs = crate::alloc_counter::count_allocs(|| {
            for _ in 0..20 {
                sim.step(&[]);
            }
        });
        assert_eq!(allocs, 0, "a settled fight must not allocate");
    }

    #[test]
    fn test_pcg32_reference_stream() {
        // Two instances agree; stream is stable across runs.
        let mut a = Pcg32::new(0xCAFE);
        let mut b = Pcg32::new(0xCAFE);
        let xs: Vec<u32> = (0..8).map(|_| a.next_u32()).collect();
        let ys: Vec<u32> = (0..8).map(|_| b.next_u32()).collect();
        assert_eq!(xs, ys);
        let mut c = Pcg32::new(1);
        let f = c.next_f32();
        assert!((0.0..1.0).contains(&f));
    }

    #[test]
    fn test_unit_id_raw_roundtrip() {
        let id = UnitId {
            index: 1234,
            generation: 56,
        };
        assert_eq!(UnitId::from_raw(id.raw()), id);
    }

    /// A minimal, harmless `Unit` at `pos` for tests that exercise `Units`
    /// storage directly rather than going through `Sim`.
    fn bare_unit(pos: Vector2) -> Unit {
        Unit {
            pos,
            prev_pos: pos,
            radius: 1.0,
            max_speed: 1.0,
            path: Vec::new(),
            path_i: 0,
            orders: VecDeque::new(),
            group: 0,
            parked: false,
            arrival_r: 0.0,
            stall: 0,
            min_remaining: f32::MAX,
            team: 0,
            max_health: 1.0,
            health: 1.0,
            damage: 0.0,
            attack_range: 0.0,
            attack_cooldown_ticks: 1,
            cooldown_left: 0,
            target: None,
            attack_move_goal: None,
            chase_anchor: Vector2::ZERO,
            chase_repath_in: 0,
            post: None,
            target_commanded: false,
            engaged: false,
            chase_slot: NO_SLOT,
            best_gap: f32::MAX,
            hold_ticks: 0,
            ally_stall: 0,
            ally_min_remaining: f32::MAX,
        }
    }

    #[test]
    fn test_slot_reuse_bumps_generation() {
        let mut units = Units::default();
        let a = units.spawn(bare_unit(Vector2::ZERO));
        assert!(units.despawn(a));
        assert!(!units.despawn(a), "double despawn must fail");
        let b = units.spawn(bare_unit(Vector2::ONE));
        assert_eq!(a.index, b.index);
        assert_ne!(a.generation, b.generation);
        assert!(units.get(a).is_none());
        assert!(units.get(b).is_some());
    }

    // ── Group pathing (shared channel / clustering / merge) ──────────────────

    fn group_of(sim: &Sim, id: UnitId) -> u32 {
        unit(sim, id).group
    }

    /// L-bend: horizontal arm [0,200]×[0,50] joined to vertical arm
    /// [150,200]×[0,200]; inside corner at (150,50). Single boundary polygon.
    fn lbend_map() -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let pts = vec![
            v(0.0, 0.0),
            v(200.0, 0.0),
            v(200.0, 200.0),
            v(150.0, 200.0),
            v(150.0, 50.0),
            v(0.0, 50.0),
        ];
        let cons = vec![(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 0)];
        (pts, cons)
    }

    /// 200×200 box split by a solid interior wall at x=100 (no door): the left
    /// half (x<100) and right half (x>100) are fully disconnected.
    fn partitioned_map() -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let pts = vec![
            v(0.0, 0.0),     // 0
            v(200.0, 0.0),   // 1
            v(200.0, 200.0), // 2
            v(0.0, 200.0),   // 3
            v(100.0, 0.0),   // 4
            v(100.0, 200.0), // 5
        ];
        let cons = vec![
            (0, 4),
            (4, 1), // bottom, split at the wall foot
            (1, 2), // right
            (2, 5),
            (5, 3), // top, split at the wall head
            (3, 0), // left
            (4, 5), // interior wall
        ];
        (pts, cons)
    }

    #[test]
    fn test_move_into_sealed_region_never_crosses_wall() {
        // A goal inside a fully sealed-off region (no door) is unreachable.
        // Whatever the units do, none may end up across the sealing wall — a
        // move toward an unreachable spot must never tunnel units through it.
        let (points, constraints) = partitioned_map();
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 3);
        let goal = v(150.0, 100.0); // right half — sealed off from the units
        let mut ids = Vec::new();
        for i in 0..16 {
            ids.push(spawn(
                &mut sim,
                v(70.0 + 6.0 * (i % 4) as f32, 80.0 + 6.0 * (i / 4) as f32),
                5.0,
                30.0,
            ));
        }
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);
        for _ in 0..400 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 100.0 + 1e-3,
                    "unit crossed the sealing wall into the closed-off area: {:?}",
                    u.pos
                );
            }
        }
    }

    #[test]
    fn test_group_move_to_sealed_goal_near_edge_does_not_cross() {
        // Goal just inside a sealed-off area, near the constraint edge. A single
        // unit correctly idles, but a *group*'s fan synthesises a converge
        // waypoint backed off toward the seed — which for a near-edge goal lands
        // on the reachable side — so route_onto_channel appends the unreachable
        // goal leg, producing a path that pokes across the wall. No unit may
        // ever cross the sealing wall.
        let (points, constraints) = partitioned_map(); // solid wall x=100, no door
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 1);
        let goal = v(103.0, 100.0); // just inside the sealed right half
        let a = spawn(&mut sim, v(88.0, 100.0), 5.0, 300.0); // fast
        let b = spawn(&mut sim, v(94.0, 100.0), 5.0, 300.0);
        sim.step(&[Command::Move {
            units: vec![a, b],
            goal,
        }]);
        for _ in 0..120 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
            for (_, u) in sim.units().iter() {
                assert!(
                    u.pos.x <= 100.0 + 1e-3,
                    "unit crossed the sealing wall into the closed area: {:?}",
                    u.pos
                );
            }
        }
    }

    #[test]
    fn test_lbend_flock_fans_across_bend() {
        let (points, constraints) = lbend_map();
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 1);
        let goal = v(175.0, 190.0);
        // A row across the corridor width (y = 10..40); the inside lane (y=40,
        // nearest the inside corner) should round tight, the outside (y=10) wide.
        let ids: Vec<_> = (0..4)
            .map(|i| spawn(&mut sim, v(15.0, 10.0 + 10.0 * i as f32), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);

        // Bend waypoint (the apex unit-0 would single-file through) fans out, one
        // distinct crossing per lane, monotone in spawn (lateral) order.
        let bend_x: Vec<f32> = ids
            .iter()
            .map(|&id| {
                let p = &unit(&sim, id).path;
                p[p.len() - 2].x
            })
            .collect();
        assert!(
            bend_x[0] > bend_x[1] && bend_x[1] > bend_x[2] && bend_x[2] > bend_x[3],
            "bend crossings must fan monotonically across the corridor: {bend_x:?}"
        );
        assert!(
            bend_x[0] - bend_x[3] > 4.0,
            "flock must spread across the bend, got {}",
            bend_x[0] - bend_x[3]
        );

        for _ in 0..400 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
        for &id in &ids {
            assert!(
                dist(unit(&sim, id).pos, goal) < 40.0,
                "unit short of goal: {:?}",
                unit(&sim, id).pos
            );
        }
    }

    #[test]
    fn test_straight_flock_fans_into_lanes() {
        // On open ground (no corners) a flock heading straight to the goal must
        // still fan into parallel lanes instead of single-filing the line and
        // crushing together — the same spread a corridor bend gives.
        let mut sim = rooms_sim(1, 1, 1);
        let goal = v(85.0, 40.0);
        // A column spread laterally to the (horizontal) travel direction.
        let ids: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(15.0, 20.0 + 20.0 * i as f32), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);
        // Each non-seed unit holds a near-goal waypoint at most of its lane, so a
        // good fraction of the start spread (40) is kept rather than collapsing.
        let mid_y: Vec<f32> = ids.iter().map(|&id| unit(&sim, id).path[0].y).collect();
        let spread = mid_y.iter().cloned().fold(f32::MIN, f32::max)
            - mid_y.iter().cloned().fold(f32::MAX, f32::min);
        assert!(
            spread > 20.0,
            "flock collapsed instead of holding its lanes: {mid_y:?}"
        );
    }

    #[test]
    fn test_outer_unit_does_not_split_around_obstacle() {
        // A wide flock vs a central obstacle with two ways around. The flock
        // commits to one side (the seed's); an outer unit whose own shortest
        // path rounds the *other* way must still follow the group. (Regression:
        // build_offset_path failed for that unit and the fallback gave it an
        // individual, other-side route.)
        let mut sim = rooms_sim(1, 1, 1);
        sim.step(&[Command::AddObstacle {
            points: vec![v(47.0, 30.0), v(53.0, 30.0), v(53.0, 70.0), v(47.0, 70.0)],
        }]);
        let goal = v(90.0, 50.0);
        let ids: Vec<_> = (0..10)
            .map(|i| spawn(&mut sim, v(15.0, 18.0 + 7.0 * i as f32), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);
        // The flock rounds the bottom (below y=30); no unit detours over the top
        // (above y=70) on a private route.
        for (k, &id) in ids.iter().enumerate() {
            let maxy = unit(&sim, id).path.iter().fold(0.0f32, |m, p| m.max(p.y));
            assert!(
                maxy < 70.0,
                "unit {k} split to the far side of the obstacle: maxy {maxy}"
            );
        }
    }

    /// Box 0..200 x 0..100 with a vertical wall at x=100: wide passages around
    /// the ends (y<25, y>75) and a narrow middle gap (y 45..55). Small units fit
    /// the gap (short straight route); big units must detour around an end.
    fn narrow_gap_map() -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let pts = vec![
            v(0.0, 0.0),
            v(200.0, 0.0),
            v(200.0, 100.0),
            v(0.0, 100.0),
            v(100.0, 25.0),
            v(100.0, 45.0),
            v(100.0, 55.0),
            v(100.0, 75.0),
        ];
        let cons = vec![(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (6, 7)];
        (pts, cons)
    }

    #[test]
    fn test_mixed_sizes_route_by_clearance() {
        // Big and small units commanded together: the small ones fit the narrow
        // middle gap (short, straight) while the big ones must detour around an
        // end. The group must not all adhere to the largest size.
        let (points, constraints) = narrow_gap_map();
        let mut sim = Sim::new(points, &constraints, 1);
        let goal = v(185.0, 50.0);
        let big: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(15.0, 44.0 + 4.0 * i as f32), 8.0, 25.0))
            .collect();
        let small: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(30.0, 44.0 + 4.0 * i as f32), 4.0, 25.0))
            .collect();
        let mut all = big.clone();
        all.extend(small.clone());
        sim.step(&[Command::Move { units: all, goal }]);

        // Small units cross straight through the gap (y stays in 45..55).
        for &id in &small {
            for w in &unit(&sim, id).path {
                assert!(
                    (45.0..=55.0).contains(&w.y),
                    "small unit left the gap route at {w:?}"
                );
            }
        }
        // Big units can't fit the gap, so they detour past an end (below y=30).
        for &id in &big {
            let miny = unit(&sim, id).path.iter().fold(999.0f32, |m, p| m.min(p.y));
            assert!(
                miny < 30.0,
                "big unit didn't detour around the end: miny {miny}"
            );
        }
        // Different sizes are separate flocks.
        assert_ne!(group_of(&sim, big[0]), group_of(&sim, small[0]));
    }

    #[test]
    fn test_move_clusters_separate_far_groups() {
        // One Move over two clusters far apart (> R_COH and wall-separated):
        // each cluster gets its own group id and shared channel.
        let mut sim = rooms_sim(3, 1, 1);
        let goal = v(150.0, 50.0);
        let a: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(20.0 + 6.0 * i as f32, 50.0), 5.0, 30.0))
            .collect();
        let b: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(260.0 + 6.0 * i as f32, 50.0), 5.0, 30.0))
            .collect();
        let all: Vec<_> = a.iter().chain(&b).copied().collect();
        sim.step(&[Command::Move { units: all, goal }]);
        let (ga, gb) = (group_of(&sim, a[0]), group_of(&sim, b[0]));
        assert!(
            a.iter().all(|&id| group_of(&sim, id) == ga),
            "cluster A split"
        );
        assert!(
            b.iter().all(|&id| group_of(&sim, id) == gb),
            "cluster B split"
        );
        assert_ne!(ga, gb, "far clusters must get distinct groups");
    }

    #[test]
    fn test_no_cluster_across_wall() {
        // Two units within R_COH but separated by a wall (not the door) must not
        // share a flock — the clear-LoS gate keeps them apart.
        let mut sim = rooms_sim(2, 1, 1); // wall x=100, door y∈[35,65]
        let a = spawn(&mut sim, v(90.0, 20.0), 5.0, 30.0);
        let b = spawn(&mut sim, v(110.0, 20.0), 5.0, 30.0);
        sim.step(&[Command::Move {
            units: vec![a, b],
            goal: v(150.0, 50.0),
        }]);
        assert_ne!(
            group_of(&sim, a),
            group_of(&sim, b),
            "wall-separated units must not cluster"
        );
    }

    #[test]
    fn test_convergence_merge_same_goal() {
        // Two distinct same-goal flocks that meet relabel to one group and stay
        // merged thereafter.
        let mut sim = rooms_sim(1, 1, 1); // open 100×100 room
        let goal = v(50.0, 50.0);
        let a: Vec<_> = (0..4)
            .map(|i| spawn(&mut sim, v(15.0 + 6.0 * i as f32, 20.0), 5.0, 25.0))
            .collect();
        let b: Vec<_> = (0..4)
            .map(|i| spawn(&mut sim, v(15.0 + 6.0 * i as f32, 80.0), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: a.clone(),
            goal,
        }]);
        sim.step(&[Command::Move {
            units: b.clone(),
            goal,
        }]);
        assert_ne!(group_of(&sim, a[0]), group_of(&sim, b[0]), "start distinct");
        step_n(&mut sim, 120);
        let g = group_of(&sim, a[0]);
        assert!(
            a.iter().chain(&b).all(|&id| group_of(&sim, id) == g),
            "converging same-goal flocks must merge to one group"
        );
    }

    #[test]
    fn test_no_merge_different_goals() {
        // Flocks that cross paths but head to different goals never merge.
        let mut sim = rooms_sim(1, 1, 1);
        let a: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(15.0, 40.0 + 6.0 * i as f32), 5.0, 25.0))
            .collect();
        let b: Vec<_> = (0..3)
            .map(|i| spawn(&mut sim, v(40.0 + 6.0 * i as f32, 15.0), 5.0, 25.0))
            .collect();
        sim.step(&[Command::Move {
            units: a.clone(),
            goal: v(85.0, 50.0),
        }]);
        sim.step(&[Command::Move {
            units: b.clone(),
            goal: v(50.0, 85.0),
        }]);
        let (ga, gb) = (group_of(&sim, a[0]), group_of(&sim, b[0]));
        assert_ne!(ga, gb);
        for _ in 0..60 {
            sim.step(&[]);
            assert_eq!(group_of(&sim, a[0]), ga, "flock A group changed");
            assert_eq!(group_of(&sim, b[0]), gb, "flock B group changed");
        }
    }

    #[test]
    fn test_reanchor_skips_overshot_waypoint() {
        // A unit shoved past an intermediate waypoint advances onto the next leg
        // instead of doubling back to the waypoint it overshot.
        let (points, constraints) = lbend_map();
        let mut sim = Sim::new(points, &constraints, 1);
        let goal = v(175.0, 190.0);
        let id = spawn(&mut sim, v(15.0, 25.0), 5.0, 25.0);
        sim.step(&[Command::Move {
            units: vec![id],
            goal,
        }]);
        assert!(
            unit(&sim, id).path.len() >= 2,
            "need an intermediate waypoint"
        );
        let corner = unit(&sim, id).path[0];
        let next = unit(&sim, id).path[1];
        // Teleport just past the corner toward the next waypoint (a crowd shove).
        let dir = norm(next - corner);
        {
            let u = sim.units.get_mut(id).unwrap();
            u.pos = corner + dir * 3.0;
            u.prev_pos = u.pos;
        }
        sim.step(&[]);
        assert!(
            unit(&sim, id).path_i >= 1,
            "must advance past the overshot waypoint, not steer back to it"
        );
    }

    #[test]
    fn test_determinism_multicluster_and_merge() {
        // A two-cluster move that converges and merges replays bit-identically.
        let script = || -> Vec<u64> {
            let mut sim = rooms_sim(1, 1, 42);
            let goal = v(50.0, 50.0);
            let a: Vec<_> = (0..4)
                .map(|i| spawn(&mut sim, v(15.0 + 6.0 * i as f32, 25.0), 5.0, 25.0))
                .collect();
            let b: Vec<_> = (0..4)
                .map(|i| spawn(&mut sim, v(15.0 + 6.0 * i as f32, 75.0), 5.0, 25.0))
                .collect();
            sim.step(&[Command::Move { units: a, goal }]);
            sim.step(&[Command::Move { units: b, goal }]);
            (0..150)
                .map(|_| {
                    sim.step(&[]);
                    sim.state_hash()
                })
                .collect()
        };
        assert_eq!(
            script(),
            script(),
            "multi-cluster + merge must be deterministic"
        );
    }

    #[test]
    fn test_counterflow_groups_pass_through_corridor() {
        // Two flocks meeting head-on in a 30px door: both must work past and
        // arrive. A jam that never resolves is the failure this catches.
        let (points, constraints) = rooms_map(3, 1);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 3);
        let west = v(30.0, 50.0);
        let east = v(270.0, 50.0);
        let spawn_block = |sim: &mut Sim, x0: f32| -> Vec<UnitId> {
            (0..6)
                .map(|i| {
                    let p = v(x0 + 12.0 * (i % 3) as f32, 35.0 + 14.0 * (i / 3) as f32);
                    spawn(sim, p, 5.0, 30.0)
                })
                .collect()
        };
        let eastbound = spawn_block(&mut sim, 20.0);
        let westbound = spawn_block(&mut sim, 250.0);
        sim.step(&[Command::Move {
            units: eastbound.clone(),
            goal: east,
        }]);
        sim.step(&[Command::Move {
            units: westbound.clone(),
            goal: west,
        }]);
        // ~240px at 1px/tick, so at 2000 ticks anything short is stuck.
        for _ in 0..2000 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
        for (ids, goal, dir) in [
            (&eastbound, east, "eastbound"),
            (&westbound, west, "westbound"),
        ] {
            for &id in ids {
                let u = unit(&sim, id);
                let d = dist(u.pos, goal);
                assert!(
                    !u.is_moving() && d < 45.0,
                    "{dir} unit never got through: at {:?}, {d} from goal {goal:?}",
                    u.pos
                );
            }
        }
    }

    #[test]
    fn test_flock_repaths_when_obstacle_seals_its_door() {
        // A building dropped across the doorway a moving flock is headed for.
        // The upper rooms still connect, so it must re-route, not pile up.
        crate::report::install_collector();
        let (points, constraints) = rooms_map(3, 2);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 9);
        let goal = v(250.0, 50.0);
        let ids: Vec<UnitId> = (0..6)
            .map(|i| {
                let p = v(20.0 + 12.0 * (i % 3) as f32, 35.0 + 14.0 * (i / 3) as f32);
                spawn(&mut sim, p, 5.0, 30.0)
            })
            .collect();
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);
        // Under way and past the first door, but well short of the second.
        step_n(&mut sim, 60);
        for &id in &ids {
            assert!(unit(&sim, id).pos.x < 190.0, "flock reached the door early");
        }
        // Seal the bottom-row door at x=200 (gap y 35..65) with a slab flush
        // against the wall plane. Flush, not crossing: crossing is rejected.
        sim.step(&[Command::AddObstacle {
            points: vec![
                v(200.0, 30.0),
                v(212.0, 30.0),
                v(212.0, 70.0),
                v(200.0, 70.0),
            ],
        }]);
        assert!(
            crate::report::drain().is_empty(),
            "a building that doesn't cross a wall must be accepted"
        );
        for _ in 0..2000 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
        for &id in &ids {
            let u = unit(&sim, id);
            let d = dist(u.pos, goal);
            assert!(
                !u.is_moving() && d < 45.0,
                "unit never re-routed around the sealed door: at {:?}, {d} from goal {goal:?}",
                u.pos
            );
        }
    }

    #[test]
    fn test_mixed_radius_clump_separates_without_overlap() {
        // `test_separation_disperses_clump_without_overlap` with two sizes:
        // a pair must clear by the *sum* of their radii, not a shared one.
        let (points, constraints) = rooms_map(3, 3);
        let mut sim = Sim::new(points, &constraints, 17);
        // Stacked in the middle room's interior, big and small interleaved.
        for i in 0..9 {
            let radius = if i % 3 == 0 { 10.0 } else { 4.0 };
            spawn(
                &mut sim,
                v(150.0 + 0.01 * i as f32, 150.0 + 0.013 * i as f32),
                radius,
                30.0,
            );
        }
        step_n(&mut sim, 600);
        let units: Vec<&Unit> = sim.units().iter().map(|(_, u)| u).collect();
        for i in 0..units.len() {
            for j in (i + 1)..units.len() {
                let d = dist(units[i].pos, units[j].pos);
                let min = units[i].radius + units[j].radius;
                assert!(
                    d >= min - 0.1,
                    "units {i} (r {}) and {j} (r {}) still overlap: dist {d} < {min}",
                    units[i].radius,
                    units[j].radius
                );
            }
        }
    }

    // ── Combat (teams, stats, targeting, engagement) ─────────────────────────

    #[test]
    fn test_relation_default_and_override_round_trips() {
        let mut sim = rooms_sim(1, 1, 1);
        assert_eq!(sim.relation(0, 0), Relation::Ally);
        assert_eq!(sim.relation(0, 1), Relation::Enemy);
        assert_eq!(sim.relation(1, 0), Relation::Enemy, "must be symmetric");
        sim.step(&[Command::SetRelation {
            team_a: 0,
            team_b: 1,
            relation: Relation::Ally,
        }]);
        assert_eq!(sim.relation(0, 1), Relation::Ally);
        assert_eq!(
            sim.relation(1, 0),
            Relation::Ally,
            "override must be symmetric too"
        );
        // An unrelated pair keeps the default rule.
        assert_eq!(sim.relation(0, 2), Relation::Enemy);
    }

    #[test]
    fn test_mixed_teams_do_not_perturb_movement() {
        // Same movement scenario, once with every unit on team 0 and once
        // split across four teams: final positions must match exactly. Not a
        // `state_hash` comparison — team is itself hashed, so that would
        // trivially differ; this compares the movement outcome instead.
        let run = |teams: [u32; 4]| -> Vec<Vector2> {
            let mut sim = rooms_sim(3, 3, 9);
            let mut ids = Vec::new();
            for (i, &team) in teams.iter().enumerate() {
                ids.push(spawn_stats(
                    &mut sim,
                    v(30.0 + 20.0 * i as f32, 40.0),
                    5.0,
                    25.0,
                    team,
                    f32::MAX,
                    0.0,
                    0.0,
                    1,
                ));
            }
            sim.step(&[Command::Move {
                units: ids.clone(),
                goal: v(200.0, 200.0),
            }]);
            step_n(&mut sim, 150);
            ids.iter().map(|&id| unit(&sim, id).pos).collect()
        };
        assert_eq!(run([0, 0, 0, 0]), run([0, 1, 2, 3]));
    }

    #[test]
    fn test_zero_cooldown_clamped_to_one() {
        let mut sim = rooms_sim(1, 1, 1);
        let id = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 20.0, 0, 10.0, 1.0, 5.0, 0);
        assert_eq!(unit(&sim, id).attack_cooldown_ticks, 1);
    }

    #[test]
    fn test_damage_drives_health_down_despawns_and_bumps_generation() {
        let mut sim = rooms_sim(1, 1, 1);
        let a = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 20.0, 0, 10.0, 0.0, 0.0, 1);
        sim.step(&[Command::Damage {
            unit: a,
            amount: 4.0,
        }]);
        assert_eq!(unit(&sim, a).health, 6.0, "damage must drive health down");
        sim.step(&[Command::Damage {
            unit: a,
            amount: 6.0,
        }]);
        assert!(
            sim.units().get(a).is_none(),
            "id must go stale once health reaches zero"
        );
        let b = spawn_stats(&mut sim, v(60.0, 60.0), 5.0, 20.0, 0, 10.0, 0.0, 0.0, 1);
        assert_eq!(a.index, b.index, "slot must be reused");
        assert_ne!(
            a.generation, b.generation,
            "a combat death must bump the generation like an explicit despawn"
        );
    }

    #[test]
    fn test_attack_closes_to_range_and_stops() {
        let mut sim = rooms_sim(3, 1, 1);
        let attacker = spawn_stats(&mut sim, v(30.0, 50.0), 5.0, 30.0, 0, 100.0, 5.0, 8.0, 5);
        // Huge health: this test is about closing distance, not the kill.
        let target = spawn_stats(&mut sim, v(220.0, 50.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        sim.step(&[Command::Attack {
            units: vec![attacker],
            target,
        }]);
        step_n(&mut sim, 400);
        let (a, t) = (unit(&sim, attacker), unit(&sim, target));
        let surf = dist(a.pos, t.pos) - a.radius - t.radius;
        assert!(
            surf <= a.attack_range + 0.5,
            "must close to within attack range: {surf}"
        );
        assert!(!a.is_moving(), "must stop once in range");
    }

    #[test]
    fn test_queued_order_waits_for_attack_to_finish_even_while_firing() {
        // A firing unit has an empty path (stationary), same as an idle one —
        // `advance_orders` must not mistake that for "done" and start a
        // queued order out from under an active fight.
        let mut sim = rooms_sim(1, 1, 1);
        let attacker = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 20.0, 0, 100.0, 4.0, 20.0, 6);
        let target = spawn_stats(&mut sim, v(60.0, 50.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        sim.step(&[Command::Attack {
            units: vec![attacker],
            target,
        }]); // already in range: fires and goes stationary this very tick
        sim.step(&[Command::Queue {
            units: vec![attacker],
            order: Order::Move {
                goal: v(20.0, 20.0),
            },
        }]);
        step_n(&mut sim, 20);
        let a = unit(&sim, attacker);
        assert_eq!(
            a.target,
            Some(target),
            "a firing unit must not look idle to advance_orders"
        );
        assert_eq!(a.orders.len(), 1, "queued move must wait for the kill");
        assert_eq!(
            a.pos,
            v(50.0, 50.0),
            "must not wander toward the queued goal"
        );
    }

    #[test]
    fn test_damage_lands_only_on_cooldown_boundaries() {
        let mut sim = rooms_sim(1, 1, 1);
        let cooldown = 6u32;
        let dmg = 4.0f32;
        let attacker = spawn_stats(
            &mut sim,
            v(50.0, 50.0),
            5.0,
            20.0,
            0,
            100.0,
            dmg,
            20.0,
            cooldown,
        );
        let target = spawn_stats(&mut sim, v(70.0, 50.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        // Already in range: fires the same tick, no chase to fold in. The
        // baseline health is taken *after* the order lands, because the
        // attacker has been defending itself since the two spawned adjacent
        // (see `test_idle_unit_defends_itself`) and has already got shots off.
        sim.step(&[Command::Attack {
            units: vec![attacker],
            target,
        }]);
        assert!(!unit(&sim, attacker).is_moving(), "already in range");
        let health_before = unit(&sim, target).health;
        // Fires are periodic once engaged, so a window of exactly this many
        // ticks holds exactly this many of them, whatever the starting phase.
        let periods = 10;
        step_n(&mut sim, cooldown as usize * periods);
        assert_eq!(
            unit(&sim, target).health,
            health_before - periods as f32 * dmg,
            "damage must land exactly on cooldown boundaries"
        );
    }

    #[test]
    fn test_simultaneous_mutual_kill_both_die() {
        let mut sim = rooms_sim(1, 1, 1);
        // Neutral while they spawn, so idle self-defence doesn't start the
        // fight before the test does; hostilities open in the same batch as
        // the two attack orders (commands apply in order).
        sim.step(&[Command::SetRelation {
            team_a: 0,
            team_b: 1,
            relation: Relation::Neutral,
        }]);
        // Lethal in one hit each, already in range: no attacker may get a
        // slot-order edge — both must die the same tick.
        let a = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 20.0, 0, 5.0, 10.0, 20.0, 1);
        let b = spawn_stats(&mut sim, v(60.0, 50.0), 5.0, 20.0, 1, 5.0, 10.0, 20.0, 1);
        sim.step(&[
            Command::SetRelation {
                team_a: 0,
                team_b: 1,
                relation: Relation::Enemy,
            },
            Command::Attack {
                units: vec![a],
                target: b,
            },
            Command::Attack {
                units: vec![b],
                target: a,
            },
        ]);
        assert!(sim.units().get(a).is_none(), "a must die");
        assert!(sim.units().get(b).is_none(), "b must die");
    }

    #[test]
    fn test_acquisition_ignores_allies() {
        let mut sim = rooms_sim(1, 1, 1);
        // Same team, close enough to be well within acquisition range at the
        // closest approach, but off the direct path (not blocking): never
        // acquired or damaged even as the mover passes near it.
        let ally = spawn_stats(&mut sim, v(60.0, 65.0), 5.0, 0.0, 0, 10.0, 0.0, 0.0, 1);
        let mover = spawn_stats(&mut sim, v(30.0, 50.0), 5.0, 20.0, 0, 10.0, 5.0, 8.0, 5);
        sim.step(&[Command::AttackMove {
            units: vec![mover],
            goal: v(90.0, 50.0),
        }]);
        step_n(&mut sim, 200);
        assert!(
            unit(&sim, mover).target.is_none(),
            "ally must never be acquired"
        );
        assert_eq!(unit(&sim, ally).health, 10.0, "ally must never take damage");
        assert_eq!(unit(&sim, mover).pos, v(90.0, 50.0), "must reach the goal");
    }

    #[test]
    fn test_direct_attack_on_ally_is_rejected() {
        let mut sim = rooms_sim(1, 1, 1);
        let attacker = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 20.0, 0, 100.0, 5.0, 8.0, 1);
        let ally = spawn_stats(&mut sim, v(55.0, 50.0), 5.0, 0.0, 0, 10.0, 0.0, 0.0, 1);
        sim.step(&[Command::Attack {
            units: vec![attacker],
            target: ally,
        }]);
        step_n(&mut sim, 20);
        assert!(
            unit(&sim, attacker).target.is_none(),
            "ally must never be locked onto as a target"
        );
        assert_eq!(unit(&sim, ally).health, 10.0, "ally must never take damage");
    }

    #[test]
    fn test_direct_attack_on_dead_target_does_not_softlock() {
        let mut sim = rooms_sim(1, 1, 1);
        let target = spawn_stats(&mut sim, v(60.0, 50.0), 5.0, 0.0, 1, 1.0, 0.0, 0.0, 1);
        sim.step(&[Command::Damage {
            unit: target,
            amount: 5.0,
        }]);
        assert!(sim.units().get(target).is_none(), "target must be dead");
        let attacker = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 20.0, 0, 100.0, 5.0, 8.0, 1);
        sim.step(&[Command::Attack {
            units: vec![attacker],
            target,
        }]);
        step_n(&mut sim, 5);
        assert!(
            unit(&sim, attacker).target.is_none(),
            "must never latch onto an already-dead target"
        );
    }

    #[test]
    fn test_chase_of_moving_target_uses_hysteresis_not_every_tick_repath() {
        let mut sim = rooms_sim(3, 1, 1);
        let attacker = spawn_stats(&mut sim, v(30.0, 50.0), 5.0, 15.0, 0, 1.0e6, 5.0, 8.0, 5);
        // Faster than the attacker and periodically re-routed, so it's
        // never caught and the chase path constantly needs updating.
        let target = spawn_enemy(&mut sim, v(230.0, 50.0), 5.0, 40.0);
        sim.step(&[Command::Attack {
            units: vec![attacker],
            target,
        }]);
        let mut repaths = 0u32;
        let mut prev = unit(&sim, attacker).chase_repath_in;
        let ticks = 200u32;
        for t in 0..ticks {
            let cmd = if t % 40 == 0 {
                let goal = if (t / 40) % 2 == 0 {
                    v(250.0, 150.0)
                } else {
                    v(250.0, 20.0)
                };
                vec![Command::Move {
                    units: vec![target],
                    goal,
                }]
            } else {
                vec![]
            };
            sim.step(&cmd);
            let cur = unit(&sim, attacker).chase_repath_in;
            if cur > prev {
                repaths += 1;
            }
            prev = cur;
        }
        assert!(
            repaths < ticks,
            "hysteresis must avoid a repath every tick: {repaths} over {ticks} ticks"
        );
        assert!(
            repaths > 0,
            "target motion must trigger at least one repath"
        );
    }

    #[test]
    fn test_attack_move_acquires_kills_and_resumes_goal() {
        let mut sim = rooms_sim(3, 1, 1);
        // One-shot kill, no cooldown lag, so the encounter resolves fast.
        let mover = spawn_stats(&mut sim, v(30.0, 50.0), 5.0, 30.0, 0, 100.0, 100.0, 8.0, 1);
        let goal = v(220.0, 50.0);
        let enemy = spawn_stats(&mut sim, v(90.0, 50.0), 5.0, 0.0, 1, 5.0, 0.0, 0.0, 1);
        sim.step(&[Command::AttackMove {
            units: vec![mover],
            goal,
        }]);
        step_n(&mut sim, 600);
        assert!(
            sim.units().get(enemy).is_none(),
            "scattered enemy must be killed along the way"
        );
        let m = unit(&sim, mover);
        assert_eq!(m.pos, goal, "must resume and reach the original goal");
        assert!(!m.is_moving());
        assert!(m.attack_move_goal.is_none(), "order completes on arrival");
    }

    /// Idle self-defence: a unit standing still shoots an enemy that walks up
    /// to it. Without this a garrison watches its own team get killed beside
    /// it, and every defensive unit needs a babysitting attack-move.
    #[test]
    fn test_idle_unit_defends_itself() {
        let mut sim = arena_sim(600.0, 600.0, 41);
        let guard = spawn_stats(&mut sim, v(300.0, 300.0), 5.0, 30.0, 0, 1.0e6, 5.0, 20.0, 5);
        let enemy = spawn_stats(&mut sim, v(325.0, 300.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        let hp0 = unit(&sim, enemy).health;
        step_n(&mut sim, 30);
        assert_eq!(
            unit(&sim, guard).target,
            Some(enemy),
            "an idle unit must acquire an enemy that comes to it"
        );
        assert!(
            unit(&sim, enemy).health < hp0,
            "an idle unit must shoot what it acquired"
        );
    }

    /// The leash: a unit that leaves its post to deal with something walks
    /// back afterwards. Without it, self-defence disperses an army across the
    /// map one pursuit at a time.
    #[test]
    fn test_idle_defender_returns_to_post_after_pursuit() {
        let mut sim = arena_sim(600.0, 600.0, 59);
        let guard = spawn_stats(&mut sim, v(300.0, 300.0), 5.0, 30.0, 0, 1.0e6, 20.0, 5.0, 5);
        let post = unit(&sim, guard).pos;
        // Inside acquisition range (3 × reach) but well outside weapon range,
        // so the guard has to leave its post to reach it.
        let enemy = spawn_stats(&mut sim, v(322.0, 300.0), 5.0, 0.0, 1, 100.0, 0.0, 0.0, 1);
        let mut furthest = 0.0f32;
        for _ in 0..400 {
            sim.step(&[]);
            furthest = furthest.max(dist(unit(&sim, guard).pos, post));
        }
        assert!(sim.units().get(enemy).is_none(), "the guard must win");
        assert!(
            furthest > 5.0,
            "fixture: the guard must actually have to pursue (moved {furthest:.1})"
        );
        let u = unit(&sim, guard);
        assert!(u.target.is_none(), "the fight is over");
        assert!(
            dist(u.pos, post) < u.radius,
            "must walk back to its post: {:.1} away",
            dist(u.pos, post)
        );
    }

    /// Dragged twice, and it still ends up where it originally stood — not
    /// over the last body. The walk home goes out as an attack-move, so a unit
    /// jumped on the way back fights and *then* carries on home.
    #[test]
    fn test_defender_returns_to_original_post_after_a_second_fight() {
        let mut sim = arena_sim(600.0, 600.0, 61);
        let guard = spawn_stats(&mut sim, v(300.0, 300.0), 5.0, 30.0, 0, 1.0e6, 20.0, 5.0, 5);
        let post = unit(&sim, guard).pos;
        let first = spawn_stats(&mut sim, v(322.0, 300.0), 5.0, 0.0, 1, 100.0, 0.0, 0.0, 1);
        for _ in 0..400 {
            sim.step(&[]);
            if sim.units().get(first).is_none() {
                break;
            }
        }
        assert!(sim.units().get(first).is_none(), "first must die");
        // Jumped on the way home, right where it now stands.
        let here = unit(&sim, guard).pos;
        assert!(
            unit(&sim, guard).post == Some(post),
            "the post must survive the first fight"
        );
        let second = spawn_stats(
            &mut sim,
            v(here.x + 14.0, here.y),
            5.0,
            0.0,
            1,
            100.0,
            0.0,
            0.0,
            1,
        );
        step_n(&mut sim, 600);
        assert!(sim.units().get(second).is_none(), "second must die too");
        let u = unit(&sim, guard);
        assert!(
            dist(u.pos, post) < u.radius,
            "must end at the original post, not the last kill: {:.1} away",
            dist(u.pos, post)
        );
    }

    /// The leash radius: a target that retreats instead of dying can otherwise
    /// tow a defender off the map, since the walk home only triggers on a
    /// kill. The guard follows to the end of its leash, gives up, and comes
    /// back.
    #[test]
    fn test_defender_breaks_off_when_kited_past_the_leash() {
        let mut sim = arena_sim(1400.0, 600.0, 71);
        let guard = spawn_stats(&mut sim, v(300.0, 300.0), 5.0, 30.0, 0, 1.0e6, 20.0, 5.0, 5);
        let post = unit(&sim, guard).pos;
        // As fast as the guard and running: never catchable, never killable.
        let kiter = spawn_stats(&mut sim, v(322.0, 300.0), 5.0, 30.0, 1, 1.0e6, 0.0, 0.0, 1);
        step_n(&mut sim, 5);
        assert_eq!(
            unit(&sim, guard).target,
            Some(kiter),
            "fixture: the guard must take the bait"
        );
        sim.step(&[Command::Move {
            units: vec![kiter],
            goal: v(1300.0, 300.0),
        }]);
        let leash = 12.0 * 5.0; // LEASH_RADII × radius, over the acquisition floor
        let mut furthest = 0.0f32;
        for _ in 0..900 {
            sim.step(&[]);
            furthest = furthest.max(dist(unit(&sim, guard).pos, post));
        }
        assert!(
            furthest > 0.5 * leash,
            "fixture: the guard must actually give chase ({furthest:.1})"
        );
        assert!(
            furthest < leash * 1.5,
            "the guard was towed {furthest:.1} from its post, past a {leash:.0} leash"
        );
        let u = unit(&sim, guard);
        assert!(
            dist(u.pos, post) < u.radius,
            "must come back to its post: {:.1} away",
            dist(u.pos, post)
        );
    }

    /// The leash is self-defence only: an explicit `Attack` is the player's
    /// decision and chases as far as it takes.
    #[test]
    fn test_commanded_attack_ignores_the_leash() {
        let mut sim = arena_sim(1400.0, 600.0, 73);
        let attacker = spawn_stats(&mut sim, v(300.0, 300.0), 5.0, 30.0, 0, 1.0e6, 20.0, 5.0, 5);
        let target = spawn_stats(&mut sim, v(340.0, 300.0), 5.0, 20.0, 1, 1.0e6, 0.0, 0.0, 1);
        sim.step(&[Command::Attack {
            units: vec![attacker],
            target,
        }]);
        sim.step(&[Command::Move {
            units: vec![target],
            goal: v(1300.0, 300.0),
        }]);
        step_n(&mut sim, 900);
        let a = unit(&sim, attacker);
        assert_eq!(
            a.target,
            Some(target),
            "a commanded kill order never lapses"
        );
        assert!(
            a.pos.x > 300.0 + 12.0 * 5.0 * 1.5,
            "and chases past any leash: only reached {:.0}",
            a.pos.x
        );
    }

    /// A player order is what sets a unit's post: after being sent somewhere,
    /// that somewhere is home — it must not snap back to where it spawned.
    #[test]
    fn test_a_move_order_replaces_the_post() {
        let mut sim = arena_sim(600.0, 600.0, 67);
        let guard = spawn_stats(&mut sim, v(150.0, 300.0), 5.0, 30.0, 0, 1.0e6, 20.0, 5.0, 5);
        let station = v(300.0, 300.0);
        sim.step(&[Command::Move {
            units: vec![guard],
            goal: station,
        }]);
        step_n(&mut sim, 300);
        assert_eq!(unit(&sim, guard).pos, station, "must reach the new station");
        let enemy = spawn_stats(&mut sim, v(322.0, 300.0), 5.0, 0.0, 1, 100.0, 0.0, 0.0, 1);
        step_n(&mut sim, 400);
        assert!(sim.units().get(enemy).is_none(), "the guard must win");
        let u = unit(&sim, guard);
        assert!(
            dist(u.pos, station) < u.radius,
            "must return to where it was posted, not where it spawned: {:.1} away",
            dist(u.pos, station)
        );
    }

    /// The line between `Move` and `AttackMove`: a unit crossing a map under a
    /// plain move order walks past enemies. Only attack-movers and *idle*
    /// units pick fights.
    #[test]
    fn test_plain_move_does_not_auto_acquire() {
        let mut sim = arena_sim(600.0, 600.0, 43);
        let enemy = spawn_stats(&mut sim, v(300.0, 320.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        let mover = spawn_stats(&mut sim, v(100.0, 300.0), 5.0, 30.0, 0, 1.0e6, 5.0, 20.0, 5);
        let goal = v(500.0, 300.0);
        sim.step(&[Command::Move {
            units: vec![mover],
            goal,
        }]);
        let hp0 = unit(&sim, enemy).health;
        let mut acquired = false;
        for _ in 0..600 {
            sim.step(&[]);
            acquired |= unit(&sim, mover).target.is_some();
            if !unit(&sim, mover).is_moving() {
                break;
            }
        }
        assert!(!acquired, "a plain Move must walk past enemies, not fight");
        assert_eq!(unit(&sim, enemy).health, hp0, "and must not shoot them");
        assert_eq!(unit(&sim, mover).pos, goal, "it must reach its goal");
    }

    /// Self-defence must not swallow a unit's order queue: `advance_orders`
    /// waits on a live target, so a unit that acquires while idle with orders
    /// pending would never start them.
    #[test]
    fn test_idle_self_defence_does_not_stall_queued_orders() {
        let mut sim = arena_sim(600.0, 600.0, 47);
        let _enemy = spawn_stats(&mut sim, v(320.0, 300.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        let unit_id = spawn_stats(&mut sim, v(300.0, 300.0), 5.0, 30.0, 0, 1.0e6, 5.0, 20.0, 5);
        let goal = v(120.0, 300.0);
        sim.step(&[Command::Queue {
            units: vec![unit_id],
            order: Order::Move { goal },
        }]);
        step_n(&mut sim, 400);
        let u = unit(&sim, unit_id);
        assert_eq!(u.pos, goal, "the queued move must still run");
        assert!(u.orders.is_empty(), "and must complete");
    }

    /// Idle self-defence ends where it started when the enemy comes to it: a
    /// guard that kills its attacker holds its ground rather than drifting.
    #[test]
    fn test_idle_defender_holds_its_ground() {
        let mut sim = arena_sim(600.0, 600.0, 53);
        let guard = spawn_stats(
            &mut sim,
            v(300.0, 300.0),
            5.0,
            30.0,
            0,
            1.0e6,
            20.0,
            20.0,
            5,
        );
        let post = unit(&sim, guard).pos;
        // Enough health to outlive its own spawn tick — the guard opens fire
        // immediately, and `spawn_stats` reads back the last live slot.
        let attacker = spawn_stats(&mut sim, v(316.0, 300.0), 5.0, 0.0, 1, 100.0, 0.0, 0.0, 1);
        assert_ne!(guard, attacker, "fixture: both units must be alive");
        step_n(&mut sim, 60);
        assert!(sim.units().get(attacker).is_none(), "the guard must win");
        let u = unit(&sim, guard);
        assert!(u.target.is_none(), "and go idle again");
        assert!(
            dist(u.pos, post) < 2.0 * u.radius,
            "a guard that never had to close must stay at its post: moved {:.1}",
            dist(u.pos, post)
        );
    }

    #[test]
    fn test_acquisition_picks_nearest_enemy() {
        let mut sim = rooms_sim(1, 1, 1);
        let mover = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 0.0, 0, 100.0, 5.0, 8.0, 5);
        let far = spawn_stats(&mut sim, v(80.0, 50.0), 5.0, 0.0, 1, 10.0, 0.0, 0.0, 1);
        let near = spawn_stats(&mut sim, v(70.0, 50.0), 5.0, 0.0, 1, 10.0, 0.0, 0.0, 1);
        sim.step(&[Command::AttackMove {
            units: vec![mover],
            goal: v(50.0, 50.0),
        }]);
        step_n(&mut sim, 3);
        assert_eq!(unit(&sim, mover).target, Some(near));
        assert_ne!(unit(&sim, mover).target, Some(far));
    }

    #[test]
    fn test_acquisition_ties_break_by_lowest_slot_index() {
        let mut sim = rooms_sim(1, 1, 1);
        let mover = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 0.0, 0, 100.0, 5.0, 8.0, 5);
        // Both enemies exactly the same distance from `mover`: the tie must
        // break toward the lower slot index regardless of grid scan order.
        let first = spawn_stats(&mut sim, v(70.0, 50.0), 5.0, 0.0, 1, 10.0, 0.0, 0.0, 1);
        let second = spawn_stats(&mut sim, v(30.0, 50.0), 5.0, 0.0, 1, 10.0, 0.0, 0.0, 1);
        sim.step(&[Command::AttackMove {
            units: vec![mover],
            goal: v(50.0, 50.0),
        }]);
        step_n(&mut sim, 3);
        assert_eq!(unit(&sim, mover).target, Some(first));
        let _ = second;
    }

    #[test]
    fn test_acquisition_respects_line_of_sight() {
        let mut sim = rooms_sim(1, 1, 1);
        let mover = spawn_stats(&mut sim, v(30.0, 50.0), 5.0, 0.0, 0, 100.0, 5.0, 8.0, 5);
        let _enemy = spawn_stats(&mut sim, v(50.0, 50.0), 5.0, 0.0, 1, 10.0, 0.0, 0.0, 1);
        // Wall directly between them, well inside acquisition range.
        sim.step(&[Command::AddObstacle {
            points: vec![v(40.0, 30.0), v(42.0, 30.0), v(42.0, 70.0), v(40.0, 70.0)],
        }]);
        sim.step(&[Command::AttackMove {
            units: vec![mover],
            goal: v(30.0, 50.0),
        }]);
        step_n(&mut sim, 5);
        assert!(
            unit(&sim, mover).target.is_none(),
            "enemy behind a wall must not be acquired"
        );
    }

    // ── Combat positioning ───────────────────────────────────────────────
    //
    // Thresholds below are absolute, with the pre-change baseline quoted in a
    // comment beside each — never a comparison against a recorded run, which
    // would drift with every tuning change.

    /// Open box arena: encircling a defender needs room that a 100×100
    /// `rooms_map` cell doesn't have.
    fn arena_sim(w: f32, h: f32, seed: u64) -> Sim {
        let pts = vec![v(0.0, 0.0), v(w, 0.0), v(w, h), v(0.0, h)];
        Sim::new(pts, &[(0, 1), (1, 2), (2, 3), (3, 0)], seed)
    }

    const DEF: Vector2 = Vector2::new(350.0, 300.0);

    /// `n` melee attackers (radius 5, reach 2, one damage per tick) converging
    /// from the left on one immobile, effectively indestructible defender: the
    /// scenario every threshold in this section was measured on. `group` picks
    /// `AttackMove` (one flock, so cohesion is in play, the worse case) over a
    /// direct `Attack`.
    fn blob_fight(n: usize, group: bool) -> (Sim, Vec<UnitId>, UnitId) {
        let mut sim = arena_sim(600.0, 600.0, 7);
        let defender = spawn_stats(&mut sim, DEF, 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        let mut attackers = Vec::new();
        for i in 0..n {
            let (col, row) = ((i / 4) as f32, (i % 4) as f32);
            attackers.push(spawn_stats(
                &mut sim,
                v(280.0 - col * 12.0, DEF.y - 18.0 + row * 12.0),
                5.0,
                30.0,
                0,
                1.0e6,
                1.0,
                2.0,
                1,
            ));
        }
        let cmd = if group {
            Command::AttackMove {
                units: attackers.clone(),
                goal: DEF,
            }
        } else {
            Command::Attack {
                units: attackers.clone(),
                target: defender,
            }
        };
        sim.step(&[cmd]);
        (sim, attackers, defender)
    }

    #[derive(Debug)]
    struct FightStats {
        /// Unit-ticks whose displacement opposes the previous tick's, over
        /// unit-ticks where the unit moved at all: stutter, specifically.
        reversal: f32,
        /// Damage delivered to the defender per tick over the window.
        dps: f32,
        /// Smallest attacker-to-defender surface gap seen (negative =
        /// bodies interpenetrating).
        min_surf: f32,
        defender_moved: f32,
        /// Fewest / mean attackers firing on one tick of the window.
        min_firing: usize,
        mean_firing: f32,
        /// Distinct eighths of the compass occupied by a firing attacker.
        sectors: usize,
    }

    /// Run a blob fight and measure it over `window` ticks after `warmup`.
    fn measure_fight(n: usize, group: bool, warmup: usize, window: usize) -> FightStats {
        let (mut sim, attackers, defender) = blob_fight(n, group);
        step_n(&mut sim, warmup);
        let hp0 = unit(&sim, defender).health;
        let mut prev: Vec<Vector2> = vec![Vector2::ZERO; n];
        let (mut rev, mut moved_ticks) = (0.0f32, 0.0f32);
        let mut min_surf = f32::MAX;
        let (mut min_firing, mut sum_firing) = (usize::MAX, 0usize);
        let mut sectors = [false; 8];
        for _ in 0..window {
            sim.step(&[]);
            let dp = unit(&sim, defender).pos;
            let mut firing = 0;
            for (k, &id) in attackers.iter().enumerate() {
                let u = unit(&sim, id);
                let d = u.pos - u.prev_pos;
                if d.length_squared() > 0.0 && prev[k].length_squared() > 0.0 {
                    moved_ticks += 1.0;
                    if d.x * prev[k].x + d.y * prev[k].y < 0.0 {
                        rev += 1.0;
                    }
                }
                prev[k] = d;
                min_surf = min_surf.min((u.pos - dp).length() - u.radius - 5.0);
                if u.engaged {
                    firing += 1;
                    let a = u.pos - dp;
                    let mut sector = if a.y < 0.0 { 4 } else { 0 };
                    if a.x.abs() < a.y.abs() {
                        sector += 2;
                    }
                    if (a.x < 0.0) != (a.y < 0.0) {
                        sector += 1;
                    }
                    sectors[sector] = true;
                }
            }
            min_firing = min_firing.min(firing);
            sum_firing += firing;
        }
        FightStats {
            reversal: if moved_ticks > 0.0 {
                rev / moved_ticks
            } else {
                0.0
            },
            dps: (hp0 - unit(&sim, defender).health) / window as f32,
            min_surf,
            defender_moved: dist(unit(&sim, defender).pos, DEF),
            min_firing,
            mean_firing: sum_firing as f32 / window as f32,
            sectors: sectors.iter().filter(|&&b| b).count(),
        }
    }

    /// Stutter — the complaint this plan exists for — measured directly:
    /// what fraction of moving unit-ticks reverse the previous tick's step.
    /// A unit walking the long way round a blob scores 0; one vibrating in
    /// place scores ~1. Both order shapes, since grouping (cohesion) is the
    /// worse case.
    #[test]
    fn test_blob_on_one_defender_does_not_stutter() {
        for group in [false, true] {
            let s = measure_fight(12, group, 200, 300);
            // Baseline 0.56 (Attack) / 0.64 (AttackMove); measured 0.11 / 0.10.
            assert!(
                s.reversal < 0.25,
                "grouped={group}: reversal {:.2} — units are vibrating: {s:?}",
                s.reversal
            );
        }
    }

    /// The other half of the outcome: positioning exists to get weapons on
    /// target, so a calmer blob that stops shooting is a regression. This is
    /// the assertion a "concede by standing still" implementation fails.
    ///
    /// The bar is the *geometry*, not the old number. Only
    /// `floor(2 pi (r_a + r_d) / 2 r_a)` = 6 attackers of this size can touch
    /// a radius-5 defender at reach 2, so 6 damage a tick is the honest
    /// ceiling here. The 6.62 the baseline scored was partly fictional: it
    /// stood bodies up to 4.65 units *inside* the defender (see
    /// `test_swarm_never_displaces_or_embeds_defender`), and packing that
    /// stops clipping hands that damage back. Surplus attackers earn their
    /// keep on other targets — see `test_acquisition_spreads_across_targets`.
    #[test]
    fn test_blob_on_one_defender_keeps_dealing_damage() {
        for group in [false, true] {
            let s = measure_fight(12, group, 200, 300);
            // Ceiling 6.0 for this geometry; measured 4.94 / 4.94 (baseline
            // 6.62 / 7.18, with bodies interpenetrating).
            assert!(
                s.dps >= 4.5,
                "grouped={group}: dps {:.2} — the ring stopped delivering: {s:?}",
                s.dps
            );
            assert!(s.sectors == 8, "attackers must encircle: {s:?}");
        }
    }

    /// Approach from one side only — the case that generates the press. The
    /// defender must not be shoved, *and* no attacker may end up standing
    /// inside it: the baseline's apparent DPS came partly from bodies
    /// embedded 4+ units into the defender (min surf -4.65 of a possible -5).
    #[test]
    fn test_swarm_never_displaces_or_embeds_defender() {
        let s = measure_fight(12, false, 200, 300);
        assert_eq!(
            s.defender_moved, 0.0,
            "a speed-0 defender must never be pushed: {s:?}"
        );
        // Baseline -4.65 (a centre 0.35 from being inside); measured -1.82.
        assert!(
            s.min_surf > -5.0,
            "attacker centre inside the defender's radius: {s:?}"
        );
    }

    /// Step 3, slot spacing: with more attackers than the inner ring holds,
    /// the ring must still *fill*. Slots spaced under a body diameter make
    /// every candidate read as occupied by its own neighbours and nobody moves
    /// in — invisible to every other assertion here, so it gets its own test.
    /// Capacity is `floor(2 pi d / 2r)` = 7 for this geometry.
    #[test]
    fn test_inner_ring_fills_to_capacity() {
        let s = measure_fight(12, false, 200, 300);
        // Measured mean 7.44 stationed, never fewer than 4 on any tick.
        assert!(
            s.mean_firing >= 6.5,
            "inner ring is not filling (slots spaced under a body diameter?): {s:?}"
        );
        assert!(s.min_firing >= 4, "ring collapsed on some tick: {s:?}");
    }

    /// Step 1: the fire/chase flap. A front-rank unit held right at
    /// `attack_range` by the press behind it used to cross the range boundary
    /// every tick — firing cleared its path, chasing rebuilt it, and each flip
    /// cost a full `find_path_abstract`.
    #[test]
    fn test_fire_hysteresis_bounds_fire_chase_flapping() {
        let (mut sim, attackers, _) = blob_fight(12, false);
        step_n(&mut sim, 200);
        let mut prev: Vec<bool> = attackers.iter().map(|&id| unit(&sim, id).engaged).collect();
        let mut flips = 0;
        for _ in 0..300 {
            sim.step(&[]);
            for (k, &id) in attackers.iter().enumerate() {
                let e = unit(&sim, id).engaged;
                if e != prev[k] {
                    flips += 1;
                }
                prev[k] = e;
            }
        }
        // Baseline ~1654 over this window (12 units × 300 ticks); measured 250.
        assert!(
            flips < 500,
            "fire/chase still flapping: {flips} transitions"
        );
    }

    /// Step 2: a settled fight must not rebuild paths every tick. Costs only
    /// CPU, so nothing else here would catch it — a unit parked on its slot
    /// re-scoring (and repathing) every tick is invisible in position space.
    #[test]
    fn test_settled_fight_does_not_churn_paths() {
        let (mut sim, attackers, _) = blob_fight(12, false);
        step_n(&mut sim, 200);
        let mut prev: Vec<Vec<Vector2>> = attackers
            .iter()
            .map(|&id| unit(&sim, id).path.clone())
            .collect();
        let mut changed = vec![0u32; attackers.len()];
        for _ in 0..300 {
            sim.step(&[]);
            for (k, &id) in attackers.iter().enumerate() {
                let u = unit(&sim, id);
                if u.path != prev[k] {
                    changed[k] += 1;
                    prev[k] = u.path.clone();
                }
            }
        }
        let worst = *changed.iter().max().expect("non-empty");
        // Measured 50 path changes for the busiest unit over 300 ticks.
        assert!(
            worst < 90,
            "a unit is rebuilding its path most ticks: {changed:?}"
        );
    }

    /// Step 3: a unit that can't reach the ring leaves for a free slot rather
    /// than pressing (the old concede-and-freeze failure) or queueing behind
    /// the front rank forever. Approach is from the left, so anything that
    /// ends up past the defender got there by going *around*.
    #[test]
    fn test_blocked_units_go_around_instead_of_pressing() {
        let (mut sim, attackers, defender) = blob_fight(12, false);
        step_n(&mut sim, 200);
        let mut travel = vec![0.0f32; attackers.len()];
        let mut fired = vec![false; attackers.len()];
        for _ in 0..300 {
            sim.step(&[]);
            for (k, &id) in attackers.iter().enumerate() {
                let u = unit(&sim, id);
                travel[k] += dist(u.pos, u.prev_pos);
                fired[k] |= u.engaged;
            }
        }
        let def_x = unit(&sim, defender).pos.x;
        let far_side = attackers
            .iter()
            .filter(|&&id| unit(&sim, id).pos.x > def_x)
            .count();
        assert!(
            far_side >= 3,
            "nobody went around the defender: {far_side} of 12 on the far side"
        );
        for k in 0..attackers.len() {
            assert!(
                fired[k] || travel[k] > 5.0,
                "unit {k} latched in place without ever firing (travel {:.2})",
                travel[k]
            );
        }
    }

    /// Step 3: the station is re-scored on the normal cadence, so a unit queued
    /// behind the front rank simply reads the inner ring as free once the unit
    /// ahead dies and walks in. No death events, no blocker ids, no retry
    /// timer — and a permanent concede latch fails this outright.
    #[test]
    fn test_ring_refills_after_front_rank_dies() {
        let (mut sim, attackers, _) = blob_fight(12, false);
        step_n(&mut sim, 300);
        let front: Vec<UnitId> = attackers
            .iter()
            .copied()
            .filter(|&id| unit(&sim, id).engaged)
            .collect();
        assert!(front.len() >= 5, "front rank never formed: {}", front.len());
        let kill: Vec<Command> = front
            .iter()
            .map(|&unit| Command::Damage {
                unit,
                amount: 1.0e9,
            })
            .collect();
        sim.step(&kill);
        assert!(
            front.iter().all(|&id| sim.units().get(id).is_none()),
            "front rank must be dead"
        );
        step_n(&mut sim, 120);
        let firing = attackers
            .iter()
            .filter(|&&id| sim.units().get(id).is_some_and(|u| u.engaged))
            .count();
        assert!(
            firing >= 4,
            "survivors never closed onto the freed ring: {firing} firing"
        );
    }

    /// Step 3, degenerate geometry: with the target flat against a wall most
    /// ring candidates fail line of sight, and the fallback is chasing the
    /// target directly. Attackers must still engage — and must not be pushed
    /// through the wall while doing it.
    #[test]
    fn test_attackers_engage_target_against_a_wall() {
        let (w, h) = (600.0f32, 600.0f32);
        let pts = vec![v(0.0, 0.0), v(w, 0.0), v(w, h), v(0.0, h)];
        let cons = [(0u32, 1u32), (1, 2), (2, 3), (3, 0)];
        let walls = wall_segments(&pts, &cons);
        let mut sim = Sim::new(pts, &cons, 11);
        let defender = spawn_stats(&mut sim, v(300.0, 5.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        let mut attackers = Vec::new();
        for i in 0..8 {
            attackers.push(spawn_stats(
                &mut sim,
                v(220.0 + (i / 4) as f32 * 12.0, 60.0 + (i % 4) as f32 * 12.0),
                5.0,
                30.0,
                0,
                1.0e6,
                1.0,
                2.0,
                1,
            ));
        }
        sim.step(&[Command::Attack {
            units: attackers.clone(),
            target: defender,
        }]);
        let hp0 = unit(&sim, defender).health;
        for _ in 0..400 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
        assert!(
            unit(&sim, defender).health < hp0 - 100.0,
            "attackers never engaged a wall-backed target"
        );
    }

    /// Step 3, degenerate geometry: a target in a doorway, where the ring is
    /// mostly walls and the approach is single-file.
    #[test]
    fn test_attackers_engage_target_in_doorway() {
        let (points, constraints) = rooms_map(2, 1);
        let walls = wall_segments(&points, &constraints);
        let mut sim = Sim::new(points, &constraints, 3);
        let defender = spawn_stats(
            &mut sim,
            v(ROOM_SIZE, ROOM_SIZE * 0.5),
            5.0,
            0.0,
            1,
            1.0e6,
            0.0,
            0.0,
            1,
        );
        let mut attackers = Vec::new();
        for i in 0..6 {
            attackers.push(spawn_stats(
                &mut sim,
                v(40.0 - (i / 3) as f32 * 12.0, 38.0 + (i % 3) as f32 * 12.0),
                5.0,
                30.0,
                0,
                1.0e6,
                1.0,
                2.0,
                1,
            ));
        }
        sim.step(&[Command::Attack {
            units: attackers.clone(),
            target: defender,
        }]);
        let hp0 = unit(&sim, defender).health;
        for _ in 0..400 {
            sim.step(&[]);
            assert_no_wall_crossing(&sim, &walls);
        }
        assert!(
            unit(&sim, defender).health < hp0 - 100.0,
            "attackers never engaged a target in a doorway"
        );
    }

    /// Step 3, moving target: the ring is anchored to the target's *current*
    /// position (slots are stored as a direction code, not a point), so a
    /// kiting target drags it along. Attackers must keep landing hits and must
    /// not thrash between slots while doing it.
    #[test]
    fn test_moving_target_drags_ring_without_slot_thrash() {
        let mut sim = arena_sim(900.0, 600.0, 5);
        let kiter = spawn_stats(&mut sim, v(300.0, 300.0), 5.0, 8.0, 1, 1.0e6, 0.0, 0.0, 1);
        let mut attackers = Vec::new();
        for i in 0..10 {
            attackers.push(spawn_stats(
                &mut sim,
                v(180.0 - (i / 5) as f32 * 12.0, 270.0 + (i % 5) as f32 * 12.0),
                5.0,
                30.0,
                0,
                1.0e6,
                1.0,
                2.0,
                1,
            ));
        }
        sim.step(&[Command::Attack {
            units: attackers.clone(),
            target: kiter,
        }]);
        step_n(&mut sim, 120);
        let hp0 = unit(&sim, kiter).health;
        let mut prev: Vec<u16> = attackers
            .iter()
            .map(|&id| unit(&sim, id).chase_slot)
            .collect();
        let mut changes = vec![0u32; attackers.len()];
        for t in 0..300 {
            // Re-order the kiter every 50 ticks so it keeps running.
            let cmds = if t % 50 == 0 {
                let goal = if (t / 50) % 2 == 0 {
                    v(800.0, 200.0)
                } else {
                    v(800.0, 400.0)
                };
                vec![Command::Move {
                    units: vec![kiter],
                    goal,
                }]
            } else {
                vec![]
            };
            sim.step(&cmds);
            for (k, &id) in attackers.iter().enumerate() {
                let slot = unit(&sim, id).chase_slot;
                if slot != prev[k] {
                    changes[k] += 1;
                    prev[k] = slot;
                }
            }
        }
        assert!(
            unit(&sim, kiter).health < hp0 - 200.0,
            "attackers stopped hitting a moving target"
        );
        let worst = *changes.iter().max().expect("non-empty");
        assert!(
            worst < 60,
            "slot thrash on a moving target: {changes:?} changes over 300 ticks"
        );
    }

    /// One unit crossing a stationary line of allies. Baseline goes straight
    /// through and bulldozes an idle ally 118 units down the map — and gives
    /// *identical* numbers for a 7-wide and a 15-wide wall, the proof that
    /// nothing was ever considering going around. Both halves matter: arriving
    /// alone is what the baseline already did.
    fn transit_through_ally_line(wall: usize) -> (Option<usize>, f32) {
        let mut sim = arena_sim(400.0, 400.0, 13);
        let y = 200.0;
        let allies: Vec<UnitId> = (0..wall)
            .map(|i| {
                let off = (i as f32 - (wall as f32 - 1.0) * 0.5) * 10.0;
                spawn(&mut sim, v(150.0, y + off), 5.0, 30.0)
            })
            .collect();
        let starts: Vec<Vector2> = allies.iter().map(|&a| unit(&sim, a).pos).collect();
        let mover = spawn(&mut sim, v(60.0, y), 5.0, 30.0);
        let goal = v(260.0, y);
        sim.step(&[Command::Move {
            units: vec![mover],
            goal,
        }]);
        let mut arrives = None;
        for t in 0..600 {
            sim.step(&[]);
            if arrives.is_none() && dist(unit(&sim, mover).pos, goal) < 1.0 {
                arrives = Some(t);
            }
        }
        let worst = allies
            .iter()
            .zip(&starts)
            .map(|(&a, &s)| dist(unit(&sim, a).pos, s))
            .fold(0.0f32, f32::max);
        (arrives, worst)
    }

    #[test]
    fn test_transit_through_ally_line_goes_around() {
        let narrow = transit_through_ally_line(7);
        let wide = transit_through_ally_line(15);
        for (wall, (arrives, worst)) in [(7, narrow), (15, wide)] {
            // Baseline: arrives 314, worst ally displaced 118.4 (both widths).
            // Measured: 278 / 5.8 and 292 / 7.1.
            let t = arrives.unwrap_or_else(|| panic!("wall {wall}: never arrived"));
            assert!(t < 400, "wall {wall}: arrived late ({t} ticks)");
            assert!(
                worst < 10.0,
                "wall {wall}: bulldozed an ally {worst:.1} units"
            );
        }
        assert_ne!(
            narrow, wide,
            "identical numbers for both wall widths mean the detour never fired"
        );
    }

    /// A group funnelling through a doorway has ally contact and slow progress
    /// by design — exactly the detour's trigger condition. Every unit must
    /// still arrive, and not by way of a scenic route.
    #[test]
    fn test_doorway_funnel_has_no_false_detours() {
        let mut sim = rooms_sim(2, 1, 4);
        let mut ids = Vec::new();
        for i in 0..8 {
            ids.push(spawn(
                &mut sim,
                v(30.0 + (i / 4) as f32 * 12.0, 30.0 + (i % 4) as f32 * 12.0),
                5.0,
                30.0,
            ));
        }
        let goal = v(ROOM_SIZE * 1.5, ROOM_SIZE * 0.5);
        sim.step(&[Command::Move {
            units: ids.clone(),
            goal,
        }]);
        let mut travel = vec![0.0f32; ids.len()];
        for _ in 0..600 {
            sim.step(&[]);
            for (k, &id) in ids.iter().enumerate() {
                let u = unit(&sim, id);
                travel[k] += dist(u.pos, u.prev_pos);
            }
        }
        for (k, &id) in ids.iter().enumerate() {
            let u = unit(&sim, id);
            assert!(
                dist(u.pos, goal) < u.arrival_r + 2.0 * u.radius,
                "unit {k} never made it through the doorway"
            );
            // Straight-line distance is ~130; measured worst travel ~165.
            assert!(
                travel[k] < 260.0,
                "unit {k} took a scenic route: {:.1} travelled",
                travel[k]
            );
        }
    }

    /// Step 6: cohesion is a persistent inward pull, applied exactly when
    /// attackers should be fanning out.
    ///
    /// One straggler marching with its flock, its chase target placed dead
    /// ahead so the chase path and the march path point the same way: any
    /// difference in its lateral drift is cohesion and nothing else.
    #[test]
    fn test_engaged_units_do_not_cohere() {
        let drift = |with_enemy: bool| -> f32 {
            let mut sim = arena_sim(1200.0, 400.0, 17);
            let start = v(150.0, 185.0);
            if with_enemy {
                // Dead ahead of the straggler and inside its acquisition
                // radius (3 × reach), so it acquires on the first tick and
                // chases along its own march heading.
                spawn_stats(&mut sim, v(450.0, 185.0), 5.0, 0.0, 1, 1.0e9, 0.0, 0.0, 1);
            }
            // Reach 100 ⇒ acquisition 300; the mates carry no weapon at all,
            // so only the straggler ever engages.
            let straggler = spawn_stats(&mut sim, start, 5.0, 30.0, 0, 1.0e6, 0.0, 100.0, 1);
            let mut ids = vec![straggler];
            for i in 0..3 {
                ids.push(spawn(
                    &mut sim,
                    v(150.0, 197.0 + i as f32 * 12.0),
                    5.0,
                    30.0,
                ));
            }
            sim.step(&[Command::AttackMove {
                units: ids,
                goal: v(1000.0, 200.0),
            }]);
            assert_eq!(
                unit(&sim, straggler).target.is_some(),
                with_enemy,
                "scenario setup: the straggler must engage iff an enemy exists"
            );
            step_n(&mut sim, 150);
            unit(&sim, straggler).pos.y - start.y
        };
        let (marching, engaged) = (drift(false), drift(true));
        assert!(
            marching > engaged + 2.0,
            "an engaged unit is still being pulled toward its group centroid: \
             drifted {engaged:.1} engaged vs {marching:.1} marching"
        );
    }

    /// Step 7: only a handful of attackers fit around one body, so acquisition
    /// penalises already-popular targets. The surplus takes the enemy next to
    /// it instead of queueing.
    #[test]
    fn test_acquisition_spreads_across_targets() {
        let mut sim = arena_sim(600.0, 600.0, 23);
        let enemies: Vec<UnitId> = (0..3)
            .map(|i| {
                spawn_stats(
                    &mut sim,
                    v(320.0 + i as f32 * 25.0, 300.0),
                    5.0,
                    0.0,
                    1,
                    1.0e6,
                    0.0,
                    0.0,
                    1,
                )
            })
            .collect();
        let attackers: Vec<UnitId> = (0..9)
            .map(|i| {
                spawn_stats(
                    &mut sim,
                    v(250.0 - (i / 3) as f32 * 12.0, 276.0 + (i % 3) as f32 * 12.0),
                    5.0,
                    30.0,
                    0,
                    1.0e6,
                    1.0,
                    40.0,
                    1,
                )
            })
            .collect();
        sim.step(&[Command::AttackMove {
            units: attackers.clone(),
            goal: v(500.0, 300.0),
        }]);
        step_n(&mut sim, 20);
        let mut engaged: Vec<usize> = Vec::new();
        for &e in &enemies {
            engaged.push(
                attackers
                    .iter()
                    .filter(|&&a| unit(&sim, a).target == Some(e))
                    .count(),
            );
        }
        assert!(
            engaged.iter().filter(|&&c| c > 0).count() >= 2,
            "acquisition mobbed one target: {engaged:?}"
        );
        assert!(
            *engaged.iter().max().expect("non-empty") <= 6,
            "acquisition mobbed one target: {engaged:?}"
        );

        // An explicit order is the player's decision and still focuses.
        let mut sim = arena_sim(600.0, 600.0, 23);
        let target = spawn_stats(&mut sim, v(320.0, 300.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        let _other = spawn_stats(&mut sim, v(350.0, 300.0), 5.0, 0.0, 1, 1.0e6, 0.0, 0.0, 1);
        let attackers: Vec<UnitId> = (0..6)
            .map(|i| {
                spawn_stats(
                    &mut sim,
                    v(250.0, 285.0 + i as f32 * 12.0),
                    5.0,
                    30.0,
                    0,
                    1.0e6,
                    1.0,
                    8.0,
                    1,
                )
            })
            .collect();
        sim.step(&[Command::Attack {
            units: attackers.clone(),
            target,
        }]);
        step_n(&mut sim, 30);
        assert!(
            attackers
                .iter()
                .all(|&a| unit(&sim, a).target == Some(target)),
            "an explicit Attack must override spreading"
        );
    }

    /// Cross-cutting: a resumed march must not inherit combat state. The
    /// units in this fight are blocked and holding slots when their target
    /// dies, which is the state most likely to leak.
    #[test]
    fn test_resumed_march_inherits_no_combat_state() {
        let mut sim = arena_sim(900.0, 600.0, 29);
        let defender = spawn_stats(&mut sim, v(350.0, 300.0), 5.0, 0.0, 1, 1200.0, 0.0, 0.0, 1);
        let attackers: Vec<UnitId> = (0..12)
            .map(|i| {
                spawn_stats(
                    &mut sim,
                    v(280.0 - (i / 4) as f32 * 12.0, 282.0 + (i % 4) as f32 * 12.0),
                    5.0,
                    30.0,
                    0,
                    1.0e6,
                    1.0,
                    6.0,
                    1,
                )
            })
            .collect();
        let goal = v(800.0, 300.0);
        sim.step(&[Command::AttackMove {
            units: attackers.clone(),
            goal,
        }]);
        // Long enough to settle into a ring with blocked units conceding onto
        // slots — the state most likely to leak into the resumed march.
        let mut ever_slotted = false;
        for _ in 0..250 {
            sim.step(&[]);
            ever_slotted |= attackers
                .iter()
                .any(|&id| unit(&sim, id).chase_slot != NO_SLOT);
        }
        assert!(
            ever_slotted,
            "scenario must actually produce slot-holding units"
        );
        step_n(&mut sim, 800);
        assert!(sim.units().get(defender).is_none(), "defender must die");
        for &id in &attackers {
            let u = unit(&sim, id);
            assert!(u.target.is_none(), "target must be cleared");
            assert!(!u.engaged, "engaged must be cleared");
            assert_eq!(u.chase_slot, NO_SLOT, "slot must be released");
            assert_eq!(u.hold_ticks, 0, "block counter must be cleared");
            assert!(
                dist(u.pos, goal) < u.arrival_r + 2.0 * u.radius,
                "must resume and reach the march goal"
            );
        }
    }

    #[test]
    fn test_attack_move_through_empty_space_behaves_like_move() {
        let move_ticks = {
            let mut sim = rooms_sim(3, 1, 1);
            let id = spawn(&mut sim, v(30.0, 50.0), 5.0, 25.0);
            sim.step(&[Command::Move {
                units: vec![id],
                goal: v(220.0, 50.0),
            }]);
            arrival_tick(&mut sim, id, 600)
        };
        let attack_move_ticks = {
            let mut sim = rooms_sim(3, 1, 1);
            let id = spawn_stats(&mut sim, v(30.0, 50.0), 5.0, 25.0, 0, 100.0, 5.0, 8.0, 5);
            sim.step(&[Command::AttackMove {
                units: vec![id],
                goal: v(220.0, 50.0),
            }]);
            arrival_tick(&mut sim, id, 600)
        };
        assert_eq!(
            move_ticks, attack_move_ticks,
            "attack-move through empty space must behave exactly like a plain move"
        );
    }
}

#[cfg(test)]
mod conflict_tests {
    use super::*;
    use crate::mapgen::rooms_map;

    /// A building crossing a map wall: the navmesh rejects the conflicting
    /// constraints via report_error! (collected, never engine-printed — the
    /// sim thread must not touch Godot FFI) and the sim stays functional.
    #[test]
    fn test_wall_crossing_obstacle_reports_error_and_sim_survives() {
        crate::report::install_collector();
        let (points, constraints) = rooms_map(2, 1);
        let mut sim = Sim::new(points, &constraints, 1);
        // Edges strictly cross the wall segment (100,0)-(100,35).
        sim.step(&[Command::AddObstacle {
            points: vec![
                Vector2::new(90.0, 10.0),
                Vector2::new(110.0, 10.0),
                Vector2::new(110.0, 30.0),
                Vector2::new(90.0, 30.0),
            ],
        }]);
        let errors = crate::report::drain();
        assert!(
            errors
                .iter()
                .any(|e| e.contains("intersects existing constraint")),
            "conflict must be reported: {errors:?}"
        );

        sim.step(&[Command::Spawn {
            pos: Vector2::new(50.0, 50.0),
            radius: 5.0,
            max_speed: 20.0,
            team: 0,
            max_health: 100.0,
            damage: 0.0,
            attack_range: 0.0,
            attack_cooldown_ticks: 1,
        }]);
        let id = sim.units().iter().last().unwrap().0;
        sim.step(&[Command::Move {
            units: vec![id],
            goal: Vector2::new(150.0, 50.0),
        }]);
        assert!(sim.units().get(id).unwrap().is_moving());
        assert!(crate::report::drain().is_empty(), "no further errors");
    }
}
