use std::cmp::Reverse;
use std::collections::BinaryHeap;

use crate::delaunay::{CDT, NONE};
use godot::prelude::Vector2;

// ── scratch buffer ────────────────────────────────────────────────────────────

/// Reusable per-agent buffers that eliminate heap allocation on repeated calls.
///
/// Create one `AStarScratch` per agent (or per thread), keep it alive, and
/// pass `&mut` to `find_path`.  On the first call the buffers are allocated
/// and the centroid cache is built.  Every subsequent call on the same CDT
/// reuses them with no allocation.
pub struct AStarScratch {
    g_score: Vec<f32>,
    /// Incoming half-edge of each face reached by `channel_search`.
    came_from: Vec<u32>,
    /// `generation[f] == current_gen` iff face `f` was touched this search.
    generation: Vec<u32>,
    current_gen: u32,
    heap: BinaryHeap<Reverse<(u32, u32, u32)>>,
    /// Centroid cache — rebuilt whenever the CDT's [`CDT::version`] changes
    /// (covers both face-count changes and same-count rebuilds of a new mesh).
    centroids: Vec<Vector2>,
    /// CDT version the `centroids` and `corner_r` caches were built for
    /// (0 = never built).
    centroids_version: u64,
    /// Per vertex: the largest radius every incident edge passes (negative
    /// when one is a wall). A larger agent finds a blocked edge there, so its
    /// path may bend round the vertex.
    corner_r: Vec<f32>,
    /// Reusable portal sequence buffer for SSFA.
    portals: Vec<u32>,
    /// Reusable left/right portal-endpoint buffers, shrunk once per funnel call.
    funnel_left: Vec<Vector2>,
    funnel_right: Vec<Vector2>,
    /// The straight channel's path, then the result; cloned out.
    best_path: Vec<Vector2>,
    /// Funnel output for the searched channel.
    cand_path: Vec<Vector2>,
    /// Node arena of the interval search.
    nodes: Vec<INode>,
    /// One expansion's successors, before they go to the heap.
    succ: Vec<(u32, u32, u32)>,
    valid: ValidMemo,
    /// Best `g` reaching each turning point, keyed `he * 2 + end`.
    root_g: Vec<f32>,
    /// `root_gen[k] == current_gen` iff `root_g[k]` is from this search.
    root_gen: Vec<u32>,
    /// Cheapest arrival of a root on a portal's own line, keyed
    /// `he * 3 + position` (see `interval_search`).
    fan_g: Vec<f32>,
    fan_gen: Vec<u32>,
}

/// Agent-valid range ([`CDT::portal_valid_range`]) per half-edge, memoised
/// across queries on one mesh at one radius: both the search and the funnel
/// ask for the same portals over and over, and each answer walks two
/// vertex rings.
#[derive(Default)]
struct ValidMemo {
    range: Vec<(f32, f32)>,
    /// `stamp[he] == epoch` iff `range[he]` is current.
    stamp: Vec<u32>,
    epoch: u32,
    /// `(CDT::version, radius bits)` the memo holds.
    key: (u64, u32),
}

impl ValidMemo {
    /// Point the memo at `cdt` and `radius`, dropping it if either changed.
    fn sync(&mut self, cdt: &CDT, radius: f32) {
        let n = cdt.num_faces() as usize * 3;
        if self.stamp.len() < n {
            self.range.resize(n, (0.0, 1.0));
            self.stamp.resize(n, 0);
        }
        let key = (cdt.version(), radius.to_bits());
        if self.key != key {
            self.key = key;
            self.epoch = self.epoch.wrapping_add(1);
            if self.epoch == 0 {
                self.stamp.fill(0);
                self.epoch = 1;
            }
        }
    }

    /// Valid range of portal `he` at the synced radius; an empty one is
    /// collapsed to the midpoint, as the funnel has always treated it.
    #[inline(always)]
    fn get(&mut self, cdt: &CDT, he: u32, radius: f32) -> (f32, f32) {
        if radius <= 0.0 {
            return (0.0, 1.0);
        }
        let i = he as usize;
        if self.stamp[i] != self.epoch {
            let (lo, hi) = cdt.portal_valid_range(he, radius);
            self.range[i] = if lo > hi { (0.5, 0.5) } else { (lo, hi) };
            self.stamp[i] = self.epoch;
        }
        self.range[i]
    }
}

impl Default for AStarScratch {
    fn default() -> Self {
        Self::new()
    }
}

impl AStarScratch {
    pub fn new() -> Self {
        Self {
            g_score: Vec::new(),
            came_from: Vec::new(),
            generation: Vec::new(),
            current_gen: 1,
            heap: BinaryHeap::new(),
            centroids: Vec::new(),
            centroids_version: 0,
            corner_r: Vec::new(),
            portals: Vec::new(),
            funnel_left: Vec::new(),
            funnel_right: Vec::new(),
            best_path: Vec::new(),
            cand_path: Vec::new(),
            nodes: Vec::new(),
            succ: Vec::new(),
            valid: ValidMemo::default(),
            root_g: Vec::new(),
            root_gen: Vec::new(),
            fan_g: Vec::new(),
            fan_gen: Vec::new(),
        }
    }

    fn prepare(&mut self, cdt: &CDT) {
        let n = cdt.num_faces() as usize;

        if n > self.g_score.len() {
            self.g_score.resize(n, 0.0);
            self.came_from.resize(n, NONE);
            self.generation.resize(n, 0);
            self.root_g.resize(n * 6, 0.0);
            self.root_gen.resize(n * 6, 0);
            self.fan_g.resize(n * 9, 0.0);
            self.fan_gen.resize(n * 9, 0);
        }

        self.current_gen = self.current_gen.wrapping_add(1);
        if self.current_gen == 0 {
            self.generation.fill(0);
            self.root_gen.fill(0);
            self.fan_gen.fill(0);
            self.current_gen = 1;
        }

        self.heap.clear();

        // Rebuild on version change, not just length change — two distinct
        // meshes can share a face count but differ in centroids, which would
        // otherwise silently corrupt pathfinding.
        if self.centroids_version != cdt.version() || self.centroids.len() != n {
            self.centroids.clear();
            self.centroids.reserve(n);
            for f in 0..n as u32 {
                self.centroids.push(cdt.face_centroid(f));
            }
            self.corner_r.clear();
            self.corner_r
                .resize(cdt.num_vertices() as usize, f32::INFINITY);
            for he in 0..n as u32 * 3 {
                let pass = if cdt.he_twin(he).is_none() || cdt.he_is_constrained(he) {
                    -1.0
                } else {
                    cdt.portal_radius(he)
                };
                for v in [cdt.he_origin(he), cdt.he_dest(he)] {
                    let c = &mut self.corner_r[v as usize];
                    *c = c.min(pass);
                }
            }
            self.centroids_version = cdt.version();
        }
    }
}

// ── unified pathfinding ───────────────────────────────────────────────────────

/// Find the shortest path for a circular agent of the given `radius`.
///
/// `radius = 0.0` ignores agent size (every portal passable regardless of
/// width).
///
/// Portals too narrow for the agent (`radius > portal_radius(he)`) are
/// impassable. The chosen triangle channel is smoothed by the Simple Stupid
/// Funnel Algorithm; waypoints wrapping a constraint-edge corner are offset
/// by `radius` along the bisector into free space, so the agent circle
/// clears the wall.
///
/// Pipeline: a straight-segment walk first (line-of-sight queries finish
/// immediately); otherwise [`interval_search`] picks the channel by the
/// length its funnelled path will have, within [`H_WEIGHT`] of the shortest.
///
/// Returns `[start, …, goal]`, or empty when either endpoint is off-mesh or
/// no passable route exists.
pub fn find_path(
    cdt: &CDT,
    start: Vector2,
    goal: Vector2,
    scratch: &mut AStarScratch,
    radius: f32,
) -> Vec<Vector2> {
    let start_face = match cdt.locate_face(start) {
        Some(f) => f,
        None => return Vec::new(),
    };
    let goal_face = match cdt.locate_face(goal) {
        Some(f) => f,
        None => return Vec::new(),
    };

    if start_face == goal_face {
        return vec![start, goal];
    }

    route(
        cdt, start, goal, start_face, goal_face, scratch, radius, H_WEIGHT,
    )
}

/// [`find_path`] past the endpoint checks: the straight line when it is
/// clear, else the [`interval_search`] channel, funnelled.
#[allow(clippy::too_many_arguments)]
fn route(
    cdt: &CDT,
    start: Vector2,
    goal: Vector2,
    start_face: u32,
    goal_face: u32,
    scratch: &mut AStarScratch,
    radius: f32,
    weight: f32,
) -> Vec<Vector2> {
    scratch.valid.sync(cdt, radius);
    // Fast path: if the raw segment crosses only passable portals, its
    // channel proves reachability and usually funnels to the straight line
    // itself, ending the query. Near a wall it bends round the radius
    // offsets, and another channel can be shorter.
    let straight = straight_channel(
        cdt,
        start_face,
        goal_face,
        start,
        goal,
        radius,
        &mut scratch.portals,
    );
    let mut best_len = f32::INFINITY;
    if straight {
        let s = &mut *scratch;
        funnel(
            cdt,
            start,
            goal,
            &s.portals,
            &mut s.valid,
            &mut s.funnel_left,
            &mut s.funnel_right,
            radius,
            &mut s.best_path,
        );
        best_len = polyline_len(&s.best_path);
        if best_len <= dist(start, goal) * 1.0001 + 1e-3 {
            return s.best_path.clone();
        }
    }
    let search = interval_search(
        cdt, start, goal, start_face, goal_face, scratch, radius, weight,
    );
    if search == Search::Found {
        let s = &mut *scratch;
        funnel(
            cdt,
            start,
            goal,
            &s.portals,
            &mut s.valid,
            &mut s.funnel_left,
            &mut s.funnel_right,
            radius,
            &mut s.cand_path,
        );
        // Copy rather than swap: swapping would ping-pong the two pooled
        // buffers' capacities and re-allocate on later queries.
        if polyline_len(&s.cand_path) < best_len {
            s.best_path.clear();
            s.best_path.extend_from_slice(&s.cand_path);
        }
    } else if !straight {
        // Only after the search gave up on a degenerate blow-up: the plain
        // search is complete.
        if search == Search::Unreachable
            || !channel_search(cdt, start_face, goal_face, scratch, radius)
        {
            return Vec::new();
        }
        let s = &mut *scratch;
        funnel(
            cdt,
            start,
            goal,
            &s.portals,
            &mut s.valid,
            &mut s.funnel_left,
            &mut s.funnel_right,
            radius,
            &mut s.best_path,
        );
    }
    scratch.best_path.clone()
}

/// Walk the faces crossed by the segment `start → goal`; succeeds iff every
/// crossing is a passable portal.  Fills `portals` (start → goal order).
///
/// Conservative: any degenerate crossing (segment through a vertex) or a
/// blocked/constrained edge aborts with `false` and the caller falls back to
/// the full channel search.
fn straight_channel(
    cdt: &CDT,
    start_face: u32,
    goal_face: u32,
    start: Vector2,
    goal: Vector2,
    radius: f32,
    portals: &mut Vec<u32>,
) -> bool {
    portals.clear();
    walk_segment(cdt, start_face, goal_face, start, goal, radius, |he| {
        portals.push(he)
    })
}

/// Walk the faces crossed by `start → goal`, invoking `on_portal(he)` for each
/// crossed half-edge in order; returns `true` iff every crossing is a passable
/// portal (same predicate and conservatism as [`straight_channel`]).
fn walk_segment(
    cdt: &CDT,
    start_face: u32,
    goal_face: u32,
    start: Vector2,
    goal: Vector2,
    radius: f32,
    mut on_portal: impl FnMut(u32),
) -> bool {
    let pts = cdt.points();
    let mut face = start_face;
    let mut prev = NONE;
    let mut steps = 0u32;
    while face != goal_face {
        steps += 1;
        if steps > cdt.num_faces() {
            return false;
        }
        let mut exit_he = NONE;
        let mut exit_nb = NONE;
        cdt.for_each_neighbor(face, |nb, he| {
            // Release builds stop at the first exit; debug builds keep
            // scanning so the uniqueness assert below stays meaningful.
            if nb == prev || (!cfg!(debug_assertions) && exit_he != NONE) {
                return;
            }
            if radius > 0.0 && radius > cdt.portal_radius(he) {
                return;
            }
            let a = pts[cdt.he_origin(he) as usize];
            let b = pts[cdt.he_dest(he) as usize];
            // Strict crossing only (degenerate = through a vertex → no exit
            // found → bail); same predicate as constraint insertion.
            if crate::delaunay::segments_intersect_proper(start, goal, a, b) {
                debug_assert!(exit_he == NONE, "two exit crossings in one face");
                exit_he = he;
                exit_nb = nb;
            }
        });
        if exit_he == NONE {
            return false;
        }
        on_portal(exit_he);
        prev = face;
        face = exit_nb;
    }
    true
}

/// Radius-aware capsule line-of-sight: `true` iff the segment `a → b` crosses
/// only passable portals (any constrained or too-narrow edge blocks). Reuses
/// the [`straight_channel`] walk but discards the portal list. Allocation-free.
///
/// The validity gate for shared-channel assignment and the merge adjacency
/// test.
pub fn clear_los(cdt: &CDT, a: Vector2, b: Vector2, radius: f32) -> bool {
    let Some(fa) = cdt.locate_face(a) else {
        return false;
    };
    clear_los_from(cdt, fa, a, b, radius)
}

/// [`clear_los`] with the face containing `a` already located. For a caller
/// testing many segments out of one fixed point, where locating `a` every time
/// is the bulk of the cost.
pub fn clear_los_from(cdt: &CDT, fa: u32, a: Vector2, b: Vector2, radius: f32) -> bool {
    let Some(fb) = cdt.locate_face(b) else {
        return false;
    };
    walk_segment(cdt, fa, fb, a, b, radius, |_| {})
}

/// Farthest point along ray `a → b` reachable without crossing a wall.
/// Returns `b` if the segment is wall-clear, else the point just shy of the
/// first wall crossing (stopping *at* the wall rather than tunnelling through
/// it). A wall is any constrained or mesh-boundary edge (interior walls are
/// hull edges — rooms connect only through doors, never across a wall).
/// Portal width is ignored (unlike [`clear_los`]), so a unit already squeezed
/// into a sub-radius gap can still slide along it. For clipping a point-like
/// separation push out of a wall, not for radius-aware routing. Allocation-free.
///
/// Assumes `a` is on the mesh (the unit's current position); if not, the move
/// is dropped (`a` returned).
pub fn clip_ray_to_walls(cdt: &CDT, a: Vector2, b: Vector2) -> Vector2 {
    let pts = cdt.points();
    let Some(mut face) = cdt.locate_face(a) else {
        return a;
    };
    // Half-edge of `face` we crossed in through; never an exit (no doubling back).
    let mut entered = NONE;
    for _ in 0..cdt.num_faces() {
        let mut exit = NONE;
        for he in face * 3..face * 3 + 3 {
            if he == entered {
                continue;
            }
            let p = pts[cdt.he_origin(he) as usize];
            let q = pts[cdt.he_dest(he) as usize];
            if crate::delaunay::segments_intersect_proper(a, b, p, q) {
                exit = he;
                break;
            }
        }
        // No edge crosses: either `b` lies in this face, or the segment grazed
        // a vertex out of it. Take the move only if `b` is actually inside —
        // a graze toward the outside would otherwise return an off-mesh point
        // (cheap point-in-triangle check avoids a second locate).
        if exit == NONE {
            let p0 = pts[cdt.he_origin(face * 3) as usize];
            let p1 = pts[cdt.he_origin(face * 3 + 1) as usize];
            let p2 = pts[cdt.he_origin(face * 3 + 2) as usize];
            return if crate::delaunay::is_point_in_triangle(b, p0, p1, p2) {
                b
            } else {
                a
            };
        }
        match cdt.he_twin(exit) {
            // Interior portal (a door): cross into the neighbour face.
            Some(tw) if !cdt.he_is_constrained(exit) => {
                entered = tw;
                face = cdt.face_of_he(tw);
            }
            // Constrained or boundary edge = wall: stop just shy of it.
            _ => {
                let p = pts[cdt.he_origin(exit) as usize];
                let q = pts[cdt.he_dest(exit) as usize];
                let t = segment_cross_param(a, b, p, q);
                return a + (b - a) * (t * 0.999);
            }
        }
    }
    a
}

/// Parameter `t ∈ [0,1]` where `a→b` crosses line `p→q` (caller guarantees a
/// proper crossing, so the denominator is non-zero).
fn segment_cross_param(a: Vector2, b: Vector2, p: Vector2, q: Vector2) -> f32 {
    let r = b - a;
    let s = q - p;
    let denom = r.x * s.y - r.y * s.x;
    let pa = p - a;
    (pa.x * s.y - pa.y * s.x) / denom
}

/// Fallback channel search: face-keyed A* over centroid-to-centroid costs.
/// Cheap and complete, but its channel is not necessarily the shortest — the
/// centroid polyline mis-measures real path length by up to the triangle
/// size, and on a grid of rooms ties every monotone route.
///
/// Fills `scratch.portals` with the channel's half-edges (start → goal) and
/// returns whether the goal is reachable.
fn channel_search(
    cdt: &CDT,
    start_face: u32,
    goal_face: u32,
    scratch: &mut AStarScratch,
    radius: f32,
) -> bool {
    scratch.prepare(cdt);
    let epoch = scratch.current_gen;

    scratch.generation[start_face as usize] = epoch;
    scratch.g_score[start_face as usize] = 0.0;
    scratch.came_from[start_face as usize] = NONE;

    let h0 = dist(
        scratch.centroids[start_face as usize],
        scratch.centroids[goal_face as usize],
    );
    scratch.heap.push(Reverse((h0.to_bits(), 0u32, start_face)));

    while let Some(Reverse((_, g_bits, current))) = scratch.heap.pop() {
        let g_cur = if scratch.generation[current as usize] == epoch {
            scratch.g_score[current as usize]
        } else {
            f32::INFINITY
        };
        if g_bits != g_cur.to_bits() {
            continue;
        }
        if current == goal_face {
            break;
        }

        let c_cur = scratch.centroids[current as usize];

        cdt.for_each_neighbor(current, |nb, he| {
            // O(1) passability gate: `portal_radius` is precomputed
            // (`compute_widths`) to the exact threshold the funnel enforces.
            if radius > 0.0 && radius > cdt.portal_radius(he) {
                return;
            }

            // g(nb): centroid-to-centroid cost — a proxy for path length,
            // good enough to find *a* channel.
            let tg = g_cur + dist(c_cur, scratch.centroids[nb as usize]);

            // h(nb): centroid-to-goal, consistent so each face expands once.
            let h_nb = dist(
                scratch.centroids[nb as usize],
                scratch.centroids[goal_face as usize],
            );

            let g_nb = if scratch.generation[nb as usize] == epoch {
                scratch.g_score[nb as usize]
            } else {
                f32::INFINITY
            };
            if tg < g_nb {
                scratch.generation[nb as usize] = epoch;
                scratch.g_score[nb as usize] = tg;
                scratch.came_from[nb as usize] = he;
                let f_val = tg + h_nb;
                scratch
                    .heap
                    .push(Reverse((f_val.to_bits(), tg.to_bits(), nb)));
            }
        });
    }

    if scratch.generation[goal_face as usize] != epoch {
        return false;
    }

    // Reconstruct the ordered portal sequence (forward: start → goal).
    scratch.portals.clear();
    let mut cur = goal_face;
    while cur != start_face {
        let he = scratch.came_from[cur as usize];
        scratch.portals.push(he);
        cur = cdt.face_of_he(he);
    }
    scratch.portals.reverse();
    true
}

// ── Interval search (Polyanya) ────────────────────────────────────────────────

/// Search node of [`interval_search`]: everything on `[t0, t1]` of portal
/// `he` is visible from `root`, which the search reached at cost `g`.
#[derive(Clone, Copy)]
struct INode {
    root: Vector2,
    g: f32,
    /// Portal half-edge, on the side being left; `NONE` marks the goal node.
    he: u32,
    t0: f32,
    t1: f32,
    /// The path may bend round this end (it is a radius-shrunk corner, not
    /// the shadow edge of an earlier one).
    turn0: bool,
    turn1: bool,
    parent: u32,
}

/// Weight on the interval search's heuristic. A route is accepted once no
/// open node could beat it by more than this factor, so paths are at most
/// that much longer than the shortest. A grid of rooms is a plateau of
/// near-equal staircases, which the exact search (1.0) explores in full.
pub const H_WEIGHT: f32 = 1.1;

/// Slack, in portal parameter, for "this end is where the valid range clips":
/// a shadow ray through a vertex lands on the next portal's end only up to
/// rounding.
const T_EPS: f32 = 1e-4;

#[inline(always)]
fn cross(a: Vector2, b: Vector2) -> f32 {
    a.x * b.y - a.y * b.x
}

/// Narrow `[lo, hi]` to where `c0 + s * c1 >= 0`.
#[inline(always)]
fn clip_halfplane(c0: f32, c1: f32, lo: &mut f32, hi: &mut f32) {
    if c1 > 0.0 {
        *lo = lo.max(-c0 / c1);
    } else if c1 < 0.0 {
        *hi = hi.min(-c0 / c1);
    } else if c0 < 0.0 {
        *hi = f32::NEG_INFINITY;
    }
}

/// Lower bound on the length from `root` through `[a, b]` to `goal`
/// (Polyanya's heuristic: a goal on the root's side is mirrored across the
/// portal line first).
fn interval_h(root: Vector2, a: Vector2, b: Vector2, goal: Vector2) -> f32 {
    let d = b - a;
    let side_g = cross(d, goal - a);
    let side_r = cross(d, root - a);
    let goal = if side_g * side_r > 0.0 {
        let len2 = d.x * d.x + d.y * d.y;
        goal - Vector2::new(-d.y, d.x) * (2.0 * side_g / len2)
    } else {
        goal
    };
    let rg = goal - root;
    let denom = cross(d, rg);
    if denom != 0.0 {
        let t = cross(root - a, rg) / denom;
        if (0.0..=1.0).contains(&t) {
            return dist(root, goal);
        }
    }
    (dist(root, a) + dist(a, goal)).min(dist(root, b) + dist(b, goal))
}

/// Search for the channel whose funnelled path is shortest: Polyanya (Cui,
/// Harabor & Grastien 2017) over the radius-shrunk portals that [`funnel`]
/// narrows each portal to, so at weight 1 its optimum is the minimum of
/// `funnel` over every channel.
///
/// A node is a root point plus the interval of one portal it sees; paths bend
/// only at portal ends the agent cannot pass (a radius-shrunk end, or a
/// vertex with a blocked edge). Turning points keep a best `g` (root-level
/// pruning). The heuristic is multiplied by `weight` ([`H_WEIGHT`] outside
/// tests), trading a bounded excess over the shortest length for a far
/// narrower search.
///
/// On [`Search::Found`], `scratch.portals` holds the winning channel.
#[allow(clippy::too_many_arguments)]
fn interval_search(
    cdt: &CDT,
    start: Vector2,
    goal: Vector2,
    start_face: u32,
    goal_face: u32,
    scratch: &mut AStarScratch,
    radius: f32,
    weight: f32,
) -> Search {
    scratch.prepare(cdt);
    scratch.valid.sync(cdt, radius);
    let epoch = scratch.current_gen;
    let pts = cdt.points();
    let AStarScratch {
        heap,
        nodes,
        succ,
        valid,
        root_g,
        root_gen,
        fan_g,
        fan_gen,
        corner_r,
        portals,
        ..
    } = scratch;
    heap.clear();
    nodes.clear();

    let corner = |v: u32| {
        let c = corner_r[v as usize];
        c < 0.0 || radius > c
    };

    // Push the node for `[lo, hi]` (shape-clipped, before the valid range)
    // on portal `he`, rooted at `root`, onto `succ` as `(priority, h, idx)`.
    let push = |nodes: &mut Vec<INode>,
                succ: &mut Vec<(u32, u32, u32)>,
                valid: &mut ValidMemo,
                root: Vector2,
                g: f32,
                he: u32,
                lo: f32,
                hi: f32,
                parent: u32| {
        let (vlo, vhi) = valid.get(cdt, he, radius);
        let (t0, t1) = (lo.max(0.0).max(vlo), hi.min(1.0).min(vhi));
        // A shadow ray grazing a vertex lands on its portals as a point at
        // the end; carried on, it would circle the vertex's fan forever. A
        // portal whose valid range is itself that point is a real squeeze.
        if t0 > t1 || (t1 - t0 <= T_EPS && vhi - vlo > T_EPS && (t1 <= T_EPS || t0 >= 1.0 - T_EPS))
        {
            return;
        }
        let pa = pts[cdt.he_origin(he) as usize];
        let pb = pts[cdt.he_dest(he) as usize];
        let a = pa + (pb - pa) * t0;
        let b = pa + (pb - pa) * t1;
        let h = interval_h(root, a, b, goal);
        let idx = nodes.len() as u32;
        nodes.push(INode {
            root,
            g,
            he,
            t0,
            t1,
            // An end the valid range clips: shrunk off a wall, or a vertex
            // with a blocked edge.
            turn0: vlo >= lo - T_EPS && (vlo > 0.0 || (t0 <= T_EPS && corner(cdt.he_origin(he)))),
            turn1: vhi <= hi + T_EPS
                && (vhi < 1.0 || (t1 >= 1.0 - T_EPS && corner(cdt.he_dest(he)))),
            parent,
        });
        succ.push(((g + h * weight).to_bits(), h.to_bits(), idx));
    };

    let passable = |he: u32| -> bool {
        cdt.he_twin(he).is_some()
            && !cdt.he_is_constrained(he)
            && !(radius > 0.0 && radius > cdt.portal_radius(he))
    };

    for he in start_face * 3..start_face * 3 + 3 {
        if passable(he) {
            push(nodes, succ, valid, start, 0.0, he, 0.0, 1.0, NONE);
        }
    }
    heap.extend(succ.drain(..).map(Reverse));

    // Safety net against a degenerate blow-up; the caller falls back to the
    // plain channel search. Real searches stay a few nodes per face.
    let node_cap = 16 * cdt.num_faces() as usize + 4096;
    let mut found = NONE;
    let mut next = heap.pop().map(|Reverse((_, _, idx))| idx);
    while let Some(idx) = next {
        let n = nodes[idx as usize];
        if n.he == NONE {
            found = idx;
            break;
        }
        if nodes.len() > node_cap {
            return Search::GaveUp;
        }
        let pa = pts[cdt.he_origin(n.he) as usize];
        let pb = pts[cdt.he_dest(n.he) as usize];
        let a = pa + (pb - pa) * n.t0;
        let b = pa + (pb - pa) * n.t1;
        let r = n.root;
        let tw = cdt.he_twin(n.he).expect("portal");
        let face = cdt.face_of_he(tw);

        // Degenerate: a root on the portal's line. On the portal itself it is
        // on the face's boundary and sees all of it; beyond either end it sees
        // the portal edge-on, and only bending round the nearer end gets in.
        let pq = pb - pa;
        let side = cross(pq, r - pa);
        let on_line = side.abs() <= 1e-3 * dist(pa, pb);
        let sgn = side.signum();
        let u = (r - pa).dot(pq) / pq.dot(pq);
        let edge_on = on_line && !(-1e-4..=1.0 + 1e-4).contains(&u);
        let near_end = u < 0.0;
        let in_cone = |y: Vector2| {
            !edge_on
                && (on_line
                    || (sgn * cross(a - r, y - r) >= 0.0 && sgn * cross(b - r, y - r) <= 0.0))
        };

        if face == goal_face {
            // Straight in, or round whichever turning end the goal hides behind.
            let reach = if in_cone(goal) {
                Some(n.g + dist(r, goal))
            } else if n.turn0
                && (if edge_on {
                    near_end
                } else {
                    sgn * cross(a - r, goal - r) < 0.0
                })
            {
                Some(n.g + dist(r, a) + dist(a, goal))
            } else if n.turn1
                && (if edge_on {
                    !near_end
                } else {
                    sgn * cross(b - r, goal - r) > 0.0
                })
            {
                Some(n.g + dist(r, b) + dist(b, goal))
            } else {
                None
            };
            if let Some(len) = reach {
                let gidx = nodes.len() as u32;
                nodes.push(INode {
                    he: NONE,
                    parent: idx,
                    g: len,
                    ..n
                });
                heap.push(Reverse((len.to_bits(), 0, gidx)));
                next = heap.pop().map(|Reverse((_, _, idx))| idx);
                continue;
            }
        }

        // Turning ends still worth bending round: root-level pruning drops
        // any whose point was already reached at no more cost. Keyed per
        // directed portal, so every node sharing a key sheds its shadow into
        // the same face.
        let mut turn_at = |end: u32, p: Vector2| -> Option<f32> {
            let g = n.g + dist(r, p);
            let k = (n.he * 2 + end) as usize;
            if root_gen[k] == epoch && root_g[k] <= g {
                return None;
            }
            root_gen[k] = epoch;
            root_g[k] = g;
            Some(g)
        };
        let g0 = if n.turn0 && (!on_line || (edge_on && near_end)) {
            turn_at(0, a)
        } else {
            None
        };
        let g1 = if n.turn1 && (!on_line || (edge_on && !near_end)) {
            turn_at(1, b)
        } else {
            None
        };

        // A root on the portal sees all of the face; from a vertex that walk
        // goes on round its fan. Keep it to the cheapest arrival per portal
        // and root position (either end or inside), or a free vertex's fan
        // would be circled forever.
        if on_line && !edge_on {
            let class = if u <= 1e-4 {
                1
            } else if u >= 1.0 - 1e-4 {
                2
            } else {
                0
            };
            let k = (tw * 3 + class) as usize;
            if fan_gen[k] == epoch && fan_g[k] <= n.g {
                next = heap.pop().map(|Reverse((_, _, idx))| idx);
                continue;
            }
            fan_gen[k] = epoch;
            fan_g[k] = n.g;
        }
        let base = face * 3;
        for k in 1..3 {
            let e = base + (tw - base + k) % 3;
            if !passable(e) {
                continue;
            }
            let ea = pts[cdt.he_origin(e) as usize];
            let eb = pts[cdt.he_dest(e) as usize];
            let ed = eb - ea;
            if on_line {
                if !edge_on {
                    push(nodes, succ, valid, r, n.g, e, 0.0, 1.0, idx);
                } else if let Some(g) = g0 {
                    push(nodes, succ, valid, a, g, e, 0.0, 1.0, idx);
                } else if let Some(g) = g1 {
                    push(nodes, succ, valid, b, g, e, 0.0, 1.0, idx);
                }
                continue;
            }
            // Signed side of `y(s) = ea + s * ed` w.r.t. the rays r→a, r→b.
            let (ca0, ca1) = (sgn * cross(a - r, ea - r), sgn * cross(a - r, ed));
            let (cb0, cb1) = (-sgn * cross(b - r, ea - r), -sgn * cross(b - r, ed));

            let (mut lo, mut hi) = (0.0f32, 1.0f32);
            clip_halfplane(ca0, ca1, &mut lo, &mut hi);
            clip_halfplane(cb0, cb1, &mut lo, &mut hi);
            push(nodes, succ, valid, r, n.g, e, lo, hi, idx);

            if let Some(g) = g0 {
                let (mut lo, mut hi) = (0.0f32, 1.0f32);
                clip_halfplane(-ca0, -ca1, &mut lo, &mut hi);
                push(nodes, succ, valid, a, g, e, lo, hi, idx);
            }
            if let Some(g) = g1 {
                let (mut lo, mut hi) = (0.0f32, 1.0f32);
                clip_halfplane(-cb0, -cb1, &mut lo, &mut hi);
                push(nodes, succ, valid, b, g, e, lo, hi, idx);
            }
        }
        // Intermediate pruning (Polyanya §4.3): a lone successor, mostly a
        // view running down a corridor of triangles, is expanded at once
        // rather than round-tripping through the heap.
        next = if succ.len() == 1 {
            Some(succ.pop().expect("one successor").2)
        } else {
            heap.extend(succ.drain(..).map(Reverse));
            heap.pop().map(|Reverse((_, _, idx))| idx)
        };
    }

    if found == NONE {
        return Search::Unreachable;
    }
    portals.clear();
    let mut cur = nodes[found as usize].parent;
    while cur != NONE {
        portals.push(nodes[cur as usize].he);
        cur = nodes[cur as usize].parent;
    }
    portals.reverse();
    Search::Found
}

/// How an [`interval_search`] ended.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Search {
    Found,
    /// Every reachable portal was searched.
    Unreachable,
    /// Hit the node cap; says nothing about reachability.
    GaveUp,
}

// ── abstraction-assisted query ───────────────────────────────────────────────

/// [`find_path`] with the pre-built [`Abstraction`] answering "different
/// components" up front, without a search that would flood the start's
/// whole component to prove it.
///
/// [`Abstraction`]: crate::abstraction::Abstraction
pub fn find_path_abstract(
    cdt: &CDT,
    abs: &crate::abstraction::Abstraction,
    start: Vector2,
    goal: Vector2,
    scratch: &mut AStarScratch,
    radius: f32,
) -> Vec<Vector2> {
    let start_face = match cdt.locate_face(start) {
        Some(f) => f,
        None => return Vec::new(),
    };
    let goal_face = match cdt.locate_face(goal) {
        Some(f) => f,
        None => return Vec::new(),
    };

    if start_face == goal_face {
        return vec![start, goal];
    }

    if abs.component_of(start_face) != abs.component_of(goal_face) {
        return Vec::new();
    }

    route(
        cdt, start, goal, start_face, goal_face, scratch, radius, H_WEIGHT,
    )
}

// ── Funnel smoothing ──────────────────────────────────────────────────────────

/// 2-D signed area of the triangle (O, A, B).
#[inline(always)]
fn area2(o: Vector2, a: Vector2, b: Vector2) -> f32 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/// Smooth the triangle channel into a polyline path, written into `out`.
///
/// Runs the Simple Stupid Funnel Algorithm on portal endpoints shrunk to
/// the agent-valid range along each portal — for radius `r`, each portal's
/// `[t_lo, t_hi]` (from [`CDT::portal_valid_range`], through `valid`) marks
/// where the agent's centre stays `≥ r` away from every wall incident to the
/// portal endpoints.
#[allow(clippy::too_many_arguments)]
fn funnel(
    cdt: &CDT,
    start: Vector2,
    goal: Vector2,
    portals: &[u32],
    valid: &mut ValidMemo,
    left_buf: &mut Vec<Vector2>,
    right_buf: &mut Vec<Vector2>,
    radius: f32,
    out: &mut Vec<Vector2>,
) {
    // Shrink each portal to its agent-valid endpoints exactly once.  `portal_valid_range`
    // is the expensive O(degree) call; fetching it per portal here (rather than per
    // SSFA access) keeps the funnel linear even with restarts.
    let pts = cdt.points();
    left_buf.clear();
    right_buf.clear();
    for &he in portals {
        let pa = pts[cdt.he_origin(he) as usize];
        let pb = pts[cdt.he_dest(he) as usize];
        let (left, right) = if radius <= 0.0 {
            (pa, pb)
        } else {
            // An empty range (corridor barely admits the agent) comes back
            // collapsed to its midpoint.
            let (tl, tr) = valid.get(cdt, he, radius);
            let at = |t: f32| Vector2::new(pa.x + t * (pb.x - pa.x), pa.y + t * (pb.y - pa.y));
            (at(tl), at(tr))
        };
        left_buf.push(left);
        right_buf.push(right);
    }

    let portal_left = |i: usize| -> Vector2 { if i < portals.len() { left_buf[i] } else { goal } };
    let portal_right = |i: usize| -> Vector2 {
        if i < portals.len() {
            right_buf[i]
        } else {
            goal
        }
    };

    // Pre-size to the maximum possible waypoint count (start + one per portal +
    // goal) so the output never reallocates as the funnel pushes corners.
    out.clear();
    out.reserve(portals.len() + 2);
    out.push(start);
    if portals.is_empty() {
        out.push(goal);
        return;
    }

    let mut apex = start;
    let mut left = portal_left(0);
    let mut right = portal_right(0);
    let mut left_idx = 0usize;
    let mut right_idx = 0usize;

    let n = portals.len() + 1;
    let mut i = 1usize;

    while i < n {
        let pl = portal_left(i);
        let pr = portal_right(i);

        if area2(apex, right, pr) <= 0.0 {
            if right == apex || area2(apex, left, pr) > 0.0 {
                right = pr;
                right_idx = i;
            } else {
                out.push(left);
                apex = left;
                let restart = left_idx + 1;
                left_idx = restart;
                right_idx = restart;
                left = portal_left(restart);
                right = portal_right(restart);
                i = restart + 1;
                continue;
            }
        }

        if area2(apex, left, pl) >= 0.0 {
            if left == apex || area2(apex, right, pl) < 0.0 {
                left = pl;
                left_idx = i;
            } else {
                out.push(right);
                apex = right;
                let restart = right_idx + 1;
                left_idx = restart;
                right_idx = restart;
                left = portal_left(restart);
                right = portal_right(restart);
                i = restart + 1;
                continue;
            }
        }

        i += 1;
    }

    if out.last() != Some(&goal) {
        out.push(goal);
    }
}

// ── shared helpers ────────────────────────────────────────────────────────────

#[inline(always)]
fn dist(a: Vector2, b: Vector2) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    (dx * dx + dy * dy).sqrt()
}

/// Closest point to `p` on segment `ab`.
#[inline(always)]
pub(crate) fn closest_on_segment(p: Vector2, a: Vector2, b: Vector2) -> Vector2 {
    let ab = b - a;
    let len2 = ab.x * ab.x + ab.y * ab.y;
    if len2 <= f32::EPSILON {
        return a;
    }
    let t = (((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2).clamp(0.0, 1.0);
    a + ab * t
}

/// Minimum distance between segments `p1q1` and `p2q2` — for checking that a
/// moving body's *swept path* (not just its endpoint) clears a wall, since
/// individually-valid endpoint positions can still sweep through a gap too
/// tight for the body. Endpoint-clamped closest points per segment (Ericson,
/// *Real-Time Collision Detection* §5.1.9); exact for non-degenerate
/// segments, falling back to point/segment distance when either is a point.
pub(crate) fn dist_segment_segment(p1: Vector2, q1: Vector2, p2: Vector2, q2: Vector2) -> f32 {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.x * d1.x + d1.y * d1.y;
    let e = d2.x * d2.x + d2.y * d2.y;
    let f = d2.x * r.x + d2.y * r.y;

    let (s, t);
    if a <= f32::EPSILON && e <= f32::EPSILON {
        s = 0.0;
        t = 0.0;
    } else if a <= f32::EPSILON {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = d1.x * r.x + d1.y * r.y;
        if e <= f32::EPSILON {
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            let b = d1.x * d2.x + d1.y * d2.y;
            let denom = a * e - b * b;
            let s0 = if denom.abs() > f32::EPSILON {
                ((b * f - c * e) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let t0 = (b * s0 + f) / e;
            if t0 < 0.0 {
                t = 0.0;
                s = (-c / a).clamp(0.0, 1.0);
            } else if t0 > 1.0 {
                t = 1.0;
                s = ((b - c) / a).clamp(0.0, 1.0);
            } else {
                t = t0;
                s = s0;
            }
        }
    }
    let c1 = p1 + d1 * s;
    let c2 = p2 + d2 * t;
    dist(c1, c2)
}

#[inline(always)]
fn polyline_len(path: &[Vector2]) -> f32 {
    path.windows(2).map(|w| dist(w[0], w[1])).sum()
}

/// Minimum distance from any path SEGMENT to any constraint EDGE (segment).
///
/// O(path * faces): a diagnostic, not a hot-path query. Used by the clearance
/// tests below and by `examples/quality`, which scores it instead.
pub fn path_min_clearance(cdt: &CDT, path: &[Vector2]) -> f32 {
    let pt_seg = |p: Vector2, a: Vector2, b: Vector2| -> f32 {
        let ab = b - a;
        let len2 = ab.x * ab.x + ab.y * ab.y;
        if len2 < 1e-12 {
            let dx = p.x - a.x;
            let dy = p.y - a.y;
            return (dx * dx + dy * dy).sqrt();
        }
        let t = (((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2).clamp(0.0, 1.0);
        let cx = a.x + t * ab.x;
        let cy = a.y + t * ab.y;
        let dx = p.x - cx;
        let dy = p.y - cy;
        (dx * dx + dy * dy).sqrt()
    };
    // For non-intersecting segments the closest pair always involves an
    // endpoint, so the min of the four endpoint-to-segment distances is exact.
    let seg_seg = |p1: Vector2, p2: Vector2, a: Vector2, b: Vector2| -> f32 {
        pt_seg(p1, a, b)
            .min(pt_seg(p2, a, b))
            .min(pt_seg(a, p1, p2))
            .min(pt_seg(b, p1, p2))
    };
    let mut min_dist = f32::INFINITY;
    for w in path.windows(2) {
        let (p1, p2) = (w[0], w[1]);
        for f in 0..cdt.num_faces() {
            for j in 0..3u32 {
                let he = f * 3 + j;
                if !cdt.he_is_constrained(he) {
                    continue;
                }
                let a = cdt.points()[cdt.he_origin(he) as usize];
                let b = cdt.points()[cdt.he_dest(he) as usize];
                min_dist = min_dist.min(seg_seg(p1, p2, a, b));
            }
        }
    }
    min_dist
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn scratch() -> AStarScratch {
        AStarScratch::new()
    }

    // ── constraint-intersection check ────────────────────────────────────────

    fn path_crosses_constraint(cdt: &CDT, p: Vector2, q: Vector2) -> bool {
        let cross = |o: Vector2, a: Vector2, b: Vector2| -> f32 {
            (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
        };
        let segments_intersect = |a: Vector2, b: Vector2, c: Vector2, d: Vector2| -> bool {
            let d1 = cross(c, d, a);
            let d2 = cross(c, d, b);
            let d3 = cross(a, b, c);
            let d4 = cross(a, b, d);
            (d1 > 0.0 && d2 < 0.0 || d1 < 0.0 && d2 > 0.0)
                && (d3 > 0.0 && d4 < 0.0 || d3 < 0.0 && d4 > 0.0)
        };

        let n = cdt.num_faces();
        for f in 0..n {
            let base = f * 3;
            for j in 0..3u32 {
                let he = base + j;
                if !cdt.he_is_constrained(he) {
                    continue;
                }
                let a = cdt.points()[cdt.he_origin(he) as usize];
                let b = cdt.points()[cdt.he_dest(he) as usize];
                if segments_intersect(p, q, a, b) {
                    return true;
                }
            }
        }
        false
    }

    fn assert_no_constraint_crossing(cdt: &CDT, path: &[Vector2]) {
        for w in path.windows(2) {
            assert!(
                !path_crosses_constraint(cdt, w[0], w[1]),
                "path segment {:?}→{:?} crosses a constraint edge",
                w[0],
                w[1],
            );
        }
    }

    /// Assert the path stays at least `radius * 0.35` from every constraint
    /// edge. The polyline contract forbids arc tessellation, so chords around
    /// a corner dip to `r·cos(θ/2)`; worst observed is ~0.388r (θ ≈ 134°,
    /// U-turn around a wall tip). A real hugging/crossing bug shows as ≈ 0.
    fn assert_min_clearance(cdt: &CDT, path: &[Vector2], radius: f32) {
        let clearance = path_min_clearance(cdt, path);
        let threshold = radius * 0.35;
        assert!(
            clearance >= threshold,
            "path comes within {clearance:.3} of a constraint edge (radius={radius}, min allowed={threshold:.3})"
        );
    }

    // ── corridor map tests ────────────────────────────────────────────────────

    // Map layout (y increases downward):
    //   outer rectangle 0..400 × 0..550
    //   internal column x=150..200 with three gaps:
    //     top    gap: y=150..200  → 50 units wide
    //     middle gap: y=300..320  → 20 units wide
    //     bottom gap: y=440..450  → 10 units wide

    fn corridors_cdt() -> crate::delaunay::CDT {
        crate::test_utils::build_cdt("test_unit_size_corridors")
    }

    #[test]
    fn test_corridor_bottom_radius5_fits() {
        // bottom corridor is 10 units; radius 5 just fits (2*5 == 10)
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 445.0);
        let goal = Vector2::new(325.0, 445.0);
        let path = find_path(&cdt, start, goal, &mut scratch(), 5.0);
        assert!(
            !path.is_empty(),
            "radius 5 should fit through 10-unit corridor"
        );
        assert_eq!(*path.first().unwrap(), start);
        assert_eq!(*path.last().unwrap(), goal);
        assert_no_constraint_crossing(&cdt, &path);
    }

    #[test]
    fn test_corridor_bottom_radius5_1_detours() {
        // radius 5.1 can't fit through the bottom gap (10 units) but the middle
        // and top gaps are still open, so a valid detour must be found.
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 445.0);
        let goal = Vector2::new(325.0, 445.0);
        let path = find_path(&cdt, start, goal, &mut scratch(), 5.1);
        assert!(!path.is_empty(), "radius 5.1 should detour via a wider gap");
        assert_no_constraint_crossing(&cdt, &path);
    }

    #[test]
    fn test_corridor_middle_radius10_fits() {
        // middle corridor is 20 units; radius 10 just fits
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 310.0);
        let goal = Vector2::new(325.0, 310.0);
        let path = find_path(&cdt, start, goal, &mut scratch(), 10.0);
        assert!(
            !path.is_empty(),
            "radius 10 should fit through 20-unit corridor"
        );
        assert_eq!(*path.first().unwrap(), start);
        assert_eq!(*path.last().unwrap(), goal);
        assert_no_constraint_crossing(&cdt, &path);
    }

    #[test]
    fn test_corridor_top_radius25_fits() {
        // top corridor is 50 units; radius 25 just fits
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let path = find_path(&cdt, start, goal, &mut scratch(), 25.0);
        assert!(
            !path.is_empty(),
            "radius 25 should fit through 50-unit corridor"
        );
        assert_eq!(*path.first().unwrap(), start);
        assert_eq!(*path.last().unwrap(), goal);
        assert_no_constraint_crossing(&cdt, &path);
    }

    #[test]
    fn test_corridor_top_radius25_1_no_path() {
        // 2*25.1 = 50.2 > 50 (widest corridor) → no path at all
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let path = find_path(&cdt, start, goal, &mut scratch(), 25.1);
        assert!(
            path.is_empty(),
            "radius 25.1 must find no path (all corridors too narrow)"
        );
    }

    #[test]
    fn test_shortest_gap_chosen() {
        // Regression: lower-left → lower-right must use the bottom gap (~335
        // units), not the middle gap (~511). The old centroid-cost A* preferred
        // the middle channel because its centroid polyline measured shorter.
        let cdt = corridors_cdt();
        let start = Vector2::new(11.89206, 524.4971);
        let goal = Vector2::new(318.6512, 498.6447);
        let mut sc = scratch();
        for &r in &[0.0f32, 1.0, 5.0] {
            let path = find_path(&cdt, start, goal, &mut sc, r);
            assert!(!path.is_empty());
            assert_no_constraint_crossing(&cdt, &path);
            let len = polyline_len(&path);
            assert!(
                len < 400.0,
                "r={r}: path length {len:.1} detoured (bottom route is ~335)"
            );
        }
    }

    #[test]
    fn test_path_length_symmetric() {
        // Shortest-path length is direction-independent; the old channel search
        // wasn't, because its centroid costs depended on the start face. The
        // exact search (weight 1) must agree both ways; the weighted one may
        // settle on different near-shortest routes, each within its bound.
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let mut sc = scratch();
            let exact = |sc: &mut AStarScratch, s: Vector2, g: Vector2, r: f32| {
                let (sf, gf) = (cdt.locate_face(s).unwrap(), cdt.locate_face(g).unwrap());
                if sf == gf {
                    vec![s, g]
                } else {
                    route(&cdt, s, g, sf, gf, sc, r, 1.0)
                }
            };
            for (s, g) in centroid_pairs(&cdt, 8) {
                for &r in &[0.0f32, 5.0] {
                    let fwd = exact(&mut sc, s, g, r);
                    let rev = exact(&mut sc, g, s, r);
                    if fwd.is_empty() || rev.is_empty() {
                        continue; // reachability symmetry is tested elsewhere
                    }
                    let (lf, lr) = (polyline_len(&fwd), polyline_len(&rev));
                    assert!(
                        (lf - lr).abs() <= 0.01 * lf.max(lr),
                        "[{map}] asymmetric lengths {lf:.2} vs {lr:.2} s={s:?} g={g:?} r={r}"
                    );
                    for (a, b, exact_len) in [(s, g, lf), (g, s, lr)] {
                        let len = polyline_len(&find_path(&cdt, a, b, &mut sc, r));
                        assert!(
                            len <= exact_len * H_WEIGHT + 1e-3,
                            "[{map}] weighted {len:.2} beyond bound of exact {exact_len:.2} r={r}"
                        );
                    }
                }
            }
        }
    }

    // ── additional tests ──────────────────────────────────────────────────────

    #[test]
    fn test_radius_same_face_returns_direct() {
        let cdt = corridors_cdt();
        let start = Vector2::new(50.0, 100.0);
        let goal = Vector2::new(60.0, 110.0);
        if cdt.locate_face(start) == cdt.locate_face(goal) {
            let path = find_path(&cdt, start, goal, &mut scratch(), 5.0);
            assert_eq!(path.len(), 2);
            assert_eq!(path[0], start);
            assert_eq!(path[1], goal);
        }
    }

    #[test]
    fn test_radius_zero_same_as_unaware() {
        // With radius 0 all portals are passable; path must exist if one does.
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 275.0);
        let goal = Vector2::new(325.0, 275.0);
        let mut sc = scratch();
        let path_r0 = find_path(&cdt, start, goal, &mut sc, 0.0);
        assert!(!path_r0.is_empty());
        assert_no_constraint_crossing(&cdt, &path_r0);
    }

    #[test]
    fn test_path_does_not_cross_constraints_small_radius() {
        let cdt = corridors_cdt();
        let mut sc = scratch();
        for (sx, sy, gx, gy) in [
            (75.0f32, 100.0, 325.0, 100.0),
            (75.0, 445.0, 325.0, 445.0),
            (75.0, 310.0, 325.0, 310.0),
            (75.0, 175.0, 325.0, 175.0),
        ] {
            let start = Vector2::new(sx, sy);
            let goal = Vector2::new(gx, gy);
            let path = find_path(&cdt, start, goal, &mut sc, 1.0);
            assert_no_constraint_crossing(&cdt, &path);
        }
    }

    #[test]
    fn test_large_radius_blocks_narrow_corridors() {
        let cdt = corridors_cdt();
        // radius 11 blocks bottom (10) and middle (20) corridors; top (50) still open
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let path = find_path(&cdt, start, goal, &mut scratch(), 11.0);
        assert!(
            !path.is_empty(),
            "radius 11 should still fit through 50-unit corridor"
        );
        assert_no_constraint_crossing(&cdt, &path);
    }

    #[test]
    fn test_outside_mesh_returns_empty() {
        let cdt = corridors_cdt();
        let mut sc = scratch();
        let outside = Vector2::new(-100.0, -100.0);
        let inside = Vector2::new(75.0, 275.0);
        assert!(find_path(&cdt, outside, inside, &mut sc, 5.0).is_empty());
        assert!(find_path(&cdt, inside, outside, &mut sc, 5.0).is_empty());
    }

    #[test]
    fn test_scratch_reuse() {
        // Ensure scratch can be reused across multiple calls without corruption.
        let cdt = corridors_cdt();
        let mut sc = AStarScratch::new();
        let pairs = [
            (
                Vector2::new(75.0, 175.0),
                Vector2::new(325.0, 175.0),
                25.0f32,
            ),
            (Vector2::new(75.0, 310.0), Vector2::new(325.0, 310.0), 10.0),
            (Vector2::new(75.0, 445.0), Vector2::new(325.0, 445.0), 5.0),
        ];
        for (start, goal, r) in pairs {
            let path = find_path(&cdt, start, goal, &mut sc, r);
            assert!(!path.is_empty());
            assert_eq!(*path.first().unwrap(), start);
            assert_eq!(*path.last().unwrap(), goal);
            assert_no_constraint_crossing(&cdt, &path);
        }
    }

    #[test]
    fn test_scratch_centroid_cache_refreshes_on_mesh_change() {
        // Regression for the stale centroid cache: a scratch reused across two
        // meshes with identical topology (same face count) but different
        // geometry must rebuild its centroid cache for the second mesh.  The old
        // length-only check kept the first mesh's centroids, corrupting the A*
        // heuristic/g for the second.
        let square = |ox: f32, oy: f32| {
            CDT::triangulate(vec![
                Vector2::new(ox, oy),
                Vector2::new(ox + 1.0, oy),
                Vector2::new(ox + 1.0, oy + 1.0),
                Vector2::new(ox, oy + 1.0),
            ])
        };
        let a = square(0.0, 0.0);
        let b = square(100.0, 100.0); // same topology, every centroid shifted
        assert_eq!(a.num_faces(), b.num_faces());
        assert_ne!(a.version(), b.version(), "distinct meshes must differ");

        let mut sc = AStarScratch::new();
        // Prime the cache on mesh A, then switch to B. (`find_path` itself
        // can skip `prepare`: a clear straight line needs no search.)
        sc.prepare(&a);
        sc.prepare(&b);

        assert_eq!(sc.centroids_version, b.version());
        assert_eq!(sc.corner_r.len(), b.num_vertices() as usize);
        for f in 0..b.num_faces() {
            assert_eq!(
                sc.centroids[f as usize],
                b.face_centroid(f),
                "centroid {f} is stale after the mesh changed"
            );
        }
    }

    // ── Phase 1: arc-path tests ───────────────────────────────────────────────

    /// Every waypoint must be at least `radius` from every constraint vertex.
    /// `portal_valid_range` places waypoints at exactly r from their relevant
    /// constraint vertex so only floating-point rounding is tolerated.
    fn assert_path_clears_constraint_vertices(cdt: &CDT, path: &[Vector2], radius: f32) {
        for &wp in path {
            for f in 0..cdt.num_faces() {
                for j in 0..3u32 {
                    let he = f * 3 + j;
                    if !cdt.he_is_constrained(he) {
                        continue;
                    }
                    let v = cdt.he_origin(he);
                    let pv = cdt.points()[v as usize];
                    let dx = wp.x - pv.x;
                    let dy = wp.y - pv.y;
                    let d = (dx * dx + dy * dy).sqrt();
                    assert!(
                        d >= radius - 1e-3,
                        "waypoint {:?} is only {:.3} from constraint vertex {:?} (need >= {:.1})",
                        wp,
                        d,
                        pv,
                        radius,
                    );
                }
            }
        }
    }

    #[test]
    fn test_arc_path_clears_corners_top_gap() {
        // Top gap (50 units wide) with radius 20.
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let radius = 20.0;
        let path = find_path(&cdt, start, goal, &mut scratch(), radius);
        assert!(!path.is_empty());
        assert_no_constraint_crossing(&cdt, &path);
        assert_path_clears_constraint_vertices(&cdt, &path, radius);
    }

    #[test]
    fn test_arc_path_clears_corners_middle_gap() {
        // Path through middle gap (20 units wide) with radius 8.
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 310.0);
        let goal = Vector2::new(325.0, 310.0);
        let radius = 8.0;
        let path = find_path(&cdt, start, goal, &mut scratch(), radius);
        assert!(!path.is_empty());
        assert_no_constraint_crossing(&cdt, &path);
        assert_path_clears_constraint_vertices(&cdt, &path, radius);
    }

    #[test]
    fn test_arc_path_radius_zero_matches_no_radius() {
        // radius 0 skips portal shrinking but still runs the SSFA funnel.
        let cdt = corridors_cdt();
        let mut sc = scratch();
        for (sx, sy, gx, gy) in [
            (75.0f32, 175.0, 325.0, 175.0),
            (75.0, 310.0, 325.0, 310.0),
            (75.0, 445.0, 325.0, 445.0),
        ] {
            let path = find_path(
                &cdt,
                Vector2::new(sx, sy),
                Vector2::new(gx, gy),
                &mut sc,
                0.0,
            );
            assert!(!path.is_empty());
            assert_no_constraint_crossing(&cdt, &path);
        }
    }

    // ── abstraction-assisted query (`find_path_abstract`) ─────────────────────

    fn corridors_abs() -> (CDT, crate::abstraction::Abstraction) {
        let cdt = corridors_cdt();
        let abs = crate::abstraction::Abstraction::build(&cdt);
        (cdt, abs)
    }

    #[test]
    fn test_tra_star_finds_path_top_gap() {
        let (cdt, abs) = corridors_abs();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let path = find_path_abstract(&cdt, &abs, start, goal, &mut scratch(), 25.0);
        assert!(
            !path.is_empty(),
            "abstract query should find path through top gap (radius 25)"
        );
        assert_eq!(*path.first().unwrap(), start);
        assert_eq!(*path.last().unwrap(), goal);
        assert_no_constraint_crossing(&cdt, &path);
    }

    #[test]
    fn test_tra_star_query_pooled_steady_state_allocs() {
        // P1 regression guard: after warmup, an abstract query must not
        // allocate its internal scratch (search nodes, heap, memo, channel) —
        // only the returned path Vec.
        let (cdt, abs) = corridors_abs();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let radius = 5.0;
        let mut sc = AStarScratch::new();

        // Warm up: first call sizes every pooled buffer.
        let p = find_path_abstract(&cdt, &abs, start, goal, &mut sc, radius);
        assert!(!p.is_empty(), "query must traverse the abstract path");

        for _ in 0..4 {
            let allocs = crate::alloc_counter::count_allocs(|| {
                let path = find_path_abstract(&cdt, &abs, start, goal, &mut sc, radius);
                std::hint::black_box(path);
            });
            assert_eq!(
                allocs, 1,
                "steady-state abstract query should allocate exactly once \
                 (the returned path Vec); got {allocs}"
            );
        }
    }

    #[test]
    fn test_tra_star_blocked_all_corridors() {
        let (cdt, abs) = corridors_abs();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        // radius 25.1 blocks even the top gap (50 units wide)
        let path = find_path_abstract(&cdt, &abs, start, goal, &mut scratch(), 25.1);
        assert!(
            path.is_empty(),
            "abstract query should return empty when all corridors too narrow"
        );
    }

    #[test]
    fn test_tra_matches_regular_astar() {
        // The abstract and plain queries must agree on reachability; the
        // abstract path must also stay clear of constraint edges.
        let (cdt, abs) = corridors_abs();
        let mut sc = scratch();
        let cases = [
            (
                Vector2::new(75.0, 175.0),
                Vector2::new(325.0, 175.0),
                10.0f32,
            ),
            (Vector2::new(75.0, 310.0), Vector2::new(325.0, 310.0), 8.0),
            (Vector2::new(75.0, 445.0), Vector2::new(325.0, 445.0), 4.0),
        ];
        for (start, goal, r) in cases {
            let p_tra = find_path_abstract(&cdt, &abs, start, goal, &mut sc, r);
            let p_astar = find_path(&cdt, start, goal, &mut sc, r);
            let tra_empty = p_tra.is_empty();
            let astar_empty = p_astar.is_empty();
            assert_eq!(
                tra_empty, astar_empty,
                "abstract and plain queries disagree on passability for start={start:?} goal={goal:?} r={r}"
            );
            if !tra_empty {
                assert_no_constraint_crossing(&cdt, &p_tra);
            }
        }
    }

    #[test]
    fn test_tra_different_components_empty() {
        let (cdt, abs) = corridors_abs();
        let inside = Vector2::new(75.0, 275.0);
        let outside = Vector2::new(-100.0, -100.0);
        let path = find_path_abstract(&cdt, &abs, inside, outside, &mut scratch(), 0.0);
        assert!(
            path.is_empty(),
            "abstract query should return empty for different components"
        );
    }

    #[test]
    fn test_tra_is_sound_against_astar() {
        // The abstraction only short-circuits "different components", so the
        // abstract query must agree with the full-mesh search on reachability
        // both ways, and its path must not cross a constraint. (The TRA* query
        // it replaced gated each corridor on its narrowest portal and could
        // both deny routes and, through a collapsed portal, invent them.)
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let abs = crate::abstraction::Abstraction::build(&cdt);
            let mut sc = scratch();
            let nf = cdt.num_faces();
            let step = (nf / 20).max(1);
            for i in (0..nf).step_by(step as usize) {
                for j in (0..nf).step_by(step as usize) {
                    if i == j {
                        continue;
                    }
                    let start = cdt.face_centroid(i);
                    let goal = cdt.face_centroid(j);
                    for &r in &[0.0f32, 1.0, 4.0, 8.0, 12.0, 25.0] {
                        let t = find_path_abstract(&cdt, &abs, start, goal, &mut sc, r);
                        let a = find_path(&cdt, start, goal, &mut sc, r);
                        assert_eq!(
                            t.is_empty(),
                            a.is_empty(),
                            "abstract and plain queries disagree: map={map} i={i} j={j} r={r}"
                        );
                        if !t.is_empty() {
                            assert_no_constraint_crossing(&cdt, &t);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn test_abstraction_levels_cover_all_faces() {
        use crate::abstraction::NodeLevel;
        let (cdt, abs) = corridors_abs();
        // Every face should be classified (no Unclassified remains).
        for f in 0..cdt.num_faces() {
            let lvl = abs.level_of(f);
            assert!(
                lvl == NodeLevel::Island
                    || lvl == NodeLevel::DeadEnd
                    || lvl == NodeLevel::Corridor
                    || lvl == NodeLevel::DecisionPoint,
                "face {f} has unexpected level {lvl:?}"
            );
        }
    }

    #[test]
    fn test_tra_path_no_constraint_crossing_radius5() {
        // Regression: path (106, 509) → (251, 432) with radius 5 was crossing constraint edges.
        let (cdt, abs) = corridors_abs();
        let start = Vector2::new(106.0007, 509.7196);
        let goal = Vector2::new(251.7516, 432.1262);
        let path = find_path_abstract(&cdt, &abs, start, goal, &mut scratch(), 5.0);
        assert!(!path.is_empty(), "TRA* should find a path");
        assert_no_constraint_crossing(&cdt, &path);
    }

    #[test]
    fn test_tra_path_clearance_near_goal_radius5() {
        let (cdt, abs) = corridors_abs();
        let start = Vector2::new(106.0007, 509.7196);
        let goal = Vector2::new(220.8347, 439.6963);
        let radius = 5.0f32;
        let path = find_path_abstract(&cdt, &abs, start, goal, &mut scratch(), radius);
        assert!(!path.is_empty(), "TRA* should find a path");
        assert_no_constraint_crossing(&cdt, &path);
        assert_min_clearance(&cdt, &path, radius);
    }

    #[test]
    fn test_tra_path_arc_through_bottom_gap_radius5() {
        let (cdt, abs) = corridors_abs();
        let start = Vector2::new(141.9652, 537.4766);
        let goal = Vector2::new(206.3227, 529.2757);
        let radius = 5.0f32;
        let path = find_path_abstract(&cdt, &abs, start, goal, &mut scratch(), radius);
        assert!(!path.is_empty(), "TRA* should find a path");
        assert_no_constraint_crossing(&cdt, &path);
        assert_min_clearance(&cdt, &path, radius);
    }

    #[test]
    fn test_abstraction_components_consistent() {
        let (cdt, abs) = corridors_abs();
        // All faces reachable from each other through free edges must share a component.
        for f in 0..cdt.num_faces() {
            cdt.for_each_neighbor(f, |nb, _| {
                assert_eq!(
                    abs.component_of(f),
                    abs.component_of(nb),
                    "face {f} and neighbour {nb} in different components"
                );
            });
        }
    }

    // ── non-square walls ──────────────────────────────────────────────────────
    //
    // Same outer rectangle and gap layout as the square corridor map, but the
    // column boundary is a chain of slanted segments instead of vertical lines.
    // The right side of the column stays at x=200, but the left side zig-zags
    // around x≈191. Gaps:
    //     top    gap (right edge): y=150..200  → 50 units
    //     middle gap (right edge): y=300..320  → 20 units
    //     bottom gap (right edge): y=440..450  → 10 units
    // The left-side gap openings are wider because of the slant.

    fn slanted_cdt() -> crate::delaunay::CDT {
        crate::test_utils::build_cdt("non_square_walls")
    }

    #[test]
    fn test_slanted_around_bottom_radius5_finds_path() {
        // Reported failure: the user reproduced "No path" between these
        // points at radius 5 even though the corridor easily fits an agent
        // of diameter 10.
        let cdt = slanted_cdt();
        let start = Vector2::new(149.5367, 522.3365);
        let goal = Vector2::new(217.049, 522.9673);
        let radius = 5.0f32;
        let path = find_path(&cdt, start, goal, &mut scratch(), radius);
        assert!(
            !path.is_empty(),
            "radius 5 must find a path around the column"
        );
        assert_eq!(*path.first().unwrap(), start);
        assert_eq!(*path.last().unwrap(), goal);
        assert_no_constraint_crossing(&cdt, &path);
        assert_min_clearance(&cdt, &path, radius);
    }

    #[test]
    fn test_slanted_around_bottom_radius1_finds_path() {
        // Tiny agent: should obviously fit.
        let cdt = slanted_cdt();
        let start = Vector2::new(149.5367, 522.3365);
        let goal = Vector2::new(217.049, 522.9673);
        let radius = 1.0f32;
        let path = find_path(&cdt, start, goal, &mut scratch(), radius);
        assert!(!path.is_empty(), "radius 1 must find a path");
        assert_no_constraint_crossing(&cdt, &path);
        assert_min_clearance(&cdt, &path, radius);
    }

    #[test]
    fn test_slanted_around_middle_radius10_finds_path() {
        // Goes through the middle gap (right edge 20 wide, left side wider).
        let cdt = slanted_cdt();
        let start = Vector2::new(75.0, 310.0);
        let goal = Vector2::new(325.0, 310.0);
        let radius = 10.0f32;
        let path = find_path(&cdt, start, goal, &mut scratch(), radius);
        assert!(
            !path.is_empty(),
            "radius 10 must fit through the middle gap"
        );
        assert_no_constraint_crossing(&cdt, &path);
        assert_min_clearance(&cdt, &path, radius);
    }

    #[test]
    fn test_slanted_around_top_radius25_finds_path() {
        // Goes through the top gap (right edge 50 wide).
        let cdt = slanted_cdt();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let radius = 25.0f32;
        let path = find_path(&cdt, start, goal, &mut scratch(), radius);
        assert!(!path.is_empty(), "radius 25 must fit through the top gap");
        assert_no_constraint_crossing(&cdt, &path);
        assert_min_clearance(&cdt, &path, radius);
    }

    #[test]
    fn test_slanted_radius_zero_unaware() {
        let cdt = slanted_cdt();
        let start = Vector2::new(149.5367, 522.3365);
        let goal = Vector2::new(217.049, 522.9673);
        let path = find_path(&cdt, start, goal, &mut scratch(), 0.0);
        assert!(!path.is_empty(), "radius 0 must always find a path");
        assert_no_constraint_crossing(&cdt, &path);
    }

    // ── general-correctness properties (independent of the search internals) ──

    /// Evenly-spaced face-centroid pairs across a mesh, skipping the diagonal.
    fn centroid_pairs(cdt: &CDT, buckets: u32) -> Vec<(Vector2, Vector2)> {
        let nf = cdt.num_faces();
        let step = (nf / buckets).max(1);
        let mut pairs = Vec::new();
        for i in (0..nf).step_by(step as usize) {
            for j in (0..nf).step_by(step as usize) {
                if i != j {
                    pairs.push((cdt.face_centroid(i), cdt.face_centroid(j)));
                }
            }
        }
        pairs
    }

    #[test]
    fn test_find_path_deterministic() {
        // Repeated identical queries must return byte-identical paths; any
        // dependence on heap tie-break order or stale scratch would break this.
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let mut sc = scratch();
            for (s, g) in centroid_pairs(&cdt, 10) {
                for &r in &[0.0f32, 4.0, 10.0] {
                    let a = find_path(&cdt, s, g, &mut sc, r);
                    let b = find_path(&cdt, s, g, &mut sc, r);
                    assert_eq!(a, b, "[{map}] nondeterministic path s={s:?} g={g:?} r={r}");
                }
            }
        }
    }

    #[test]
    fn test_find_path_reachability_symmetric() {
        // Passability is undirected (the portal gate is the same edge width from
        // either side), so a goal is reachable from a start iff the reverse holds.
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let mut sc = scratch();
            for (s, g) in centroid_pairs(&cdt, 10) {
                for &r in &[0.0f32, 5.0, 10.0] {
                    let fwd = find_path(&cdt, s, g, &mut sc, r).is_empty();
                    let rev = find_path(&cdt, g, s, &mut sc, r).is_empty();
                    assert_eq!(
                        fwd, rev,
                        "[{map}] asymmetric reachability s={s:?} g={g:?} r={r}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_find_path_radius_monotonic_passability() {
        // The passable subgraph only grows as the agent shrinks, so reachability
        // must be monotone: if some radius reaches the goal, every smaller one does.
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let mut sc = scratch();
            let radii = [25.0f32, 10.0, 5.0, 1.0, 0.0]; // strictly decreasing
            for (s, g) in centroid_pairs(&cdt, 8) {
                let mut larger_reached = false;
                for &r in &radii {
                    let reached = !find_path(&cdt, s, g, &mut sc, r).is_empty();
                    if larger_reached {
                        assert!(
                            reached,
                            "[{map}] radius>{r} reached the goal but r={r} did not (s={s:?} g={g:?})"
                        );
                    }
                    larger_reached = reached;
                }
            }
        }
    }

    #[test]
    fn test_radius0_path_bends_only_at_mesh_vertices() {
        // A radius-0 funnel path is taut: straight between bends, with every
        // bend exactly on a portal endpoint (a triangulation vertex). An
        // interior waypoint off the mesh would mean the funnel invented a
        // corner. The path is channel-optimal, not globally optimal, so
        // length isn't asserted here.
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let pts = cdt.points().to_vec();
            let mut sc = scratch();
            for (s, g) in centroid_pairs(&cdt, 10) {
                let path = find_path(&cdt, s, g, &mut sc, 0.0);
                if path.len() <= 2 {
                    continue; // start + goal only: nothing to bend at
                }
                // Bit-exact on purpose: the funnel must copy apex vertices,
                // never recompute them.
                for &wp in &path[1..path.len() - 1] {
                    assert!(
                        pts.contains(&wp),
                        "[{map}] interior waypoint {wp:?} is not a mesh vertex"
                    );
                }
            }
        }
    }

    #[test]
    fn test_find_path_abstract_deterministic() {
        // Same contract as `test_find_path_deterministic`, but for the TRA*
        // entry point, which has its own heap, tie-breaking, and scratch pools.
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let abs = crate::abstraction::Abstraction::build(&cdt);
            let mut sc = scratch();
            for (s, g) in centroid_pairs(&cdt, 10) {
                for &r in &[0.0f32, 4.0, 10.0] {
                    let a = find_path_abstract(&cdt, &abs, s, g, &mut sc, r);
                    let b = find_path_abstract(&cdt, &abs, s, g, &mut sc, r);
                    assert_eq!(
                        a, b,
                        "[{map}] nondeterministic TRA* path s={s:?} g={g:?} r={r}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_find_path_clearance_sweep() {
        // Every routable sampled query must keep the clearance floor (see
        // `assert_min_clearance`). Pairs with an endpoint within r of a wall
        // are skipped — no clearance promise applies there.
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let abs = crate::abstraction::Abstraction::build(&cdt);
            let mut sc = scratch();
            for (s, g) in centroid_pairs(&cdt, 8) {
                for &r in &[2.0f32, 5.0] {
                    // A degenerate one-segment "path" measures point clearance.
                    if path_min_clearance(&cdt, &[s, s]) < r
                        || path_min_clearance(&cdt, &[g, g]) < r
                    {
                        continue;
                    }
                    let path = find_path(&cdt, s, g, &mut sc, r);
                    if !path.is_empty() {
                        assert_min_clearance(&cdt, &path, r);
                    }
                    let tra = find_path_abstract(&cdt, &abs, s, g, &mut sc, r);
                    if !tra.is_empty() {
                        assert_min_clearance(&cdt, &tra, r);
                    }
                }
            }
        }
    }

    #[test]
    fn test_find_path_steady_state_allocs() {
        // Companion to the abstract-query allocation guard: once the scratch is
        // warm, a find_path call's only heap allocation is the returned Vec —
        // all search state (nodes, heap, memo, portals, funnel buffers) is pooled.
        let cdt = corridors_cdt();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let radius = 5.0;
        let mut sc = AStarScratch::new();

        let warm = find_path(&cdt, start, goal, &mut sc, radius);
        assert!(!warm.is_empty(), "query must traverse a corridor");

        for _ in 0..4 {
            let allocs = crate::alloc_counter::count_allocs(|| {
                std::hint::black_box(find_path(&cdt, start, goal, &mut sc, radius));
            });
            assert_eq!(
                allocs, 1,
                "steady-state find_path should allocate exactly once (the returned path Vec); got {allocs}"
            );
        }
    }

    fn rooms(side: usize) -> CDT {
        let (points, constraints) = crate::mapgen::rooms_map(side, side);
        let mut cdt = CDT::from_points(points);
        for (a, b) in constraints {
            cdt.insert_constraint(a, b);
        }
        cdt.remove_super_triangle();
        cdt.build_grid_index();
        cdt.compute_widths();
        cdt
    }

    /// `find_path` at weight 1: the exact minimum of the funnel over channels.
    fn exact_path(
        cdt: &CDT,
        sc: &mut AStarScratch,
        s: Vector2,
        g: Vector2,
        r: f32,
    ) -> Vec<Vector2> {
        let (sf, gf) = (cdt.locate_face(s).unwrap(), cdt.locate_face(g).unwrap());
        if sf == gf {
            return vec![s, g];
        }
        route(cdt, s, g, sf, gf, sc, r, 1.0)
    }

    #[test]
    fn test_rooms_diagonal_takes_the_staircase() {
        // Regression for `solo_march`: with centred doors, a door-to-door
        // staircase from corner room to corner room is as short as the
        // straight diagonal. The centroid channel search went down one column
        // and along one row instead (~39% longer), and its bounded refinement
        // never found the way out of that plateau.
        let cdt = rooms(20);
        let abs = crate::abstraction::Abstraction::build(&cdt);
        let mut sc = scratch();
        let (s, g) = (Vector2::new(50.0, 50.0), Vector2::new(1950.0, 1950.0));
        let diagonal = dist(s, g);
        let exact = polyline_len(&exact_path(&cdt, &mut sc, s, g, 5.0));
        assert!(
            exact < diagonal * 1.01,
            "exact {exact:.1} vs diagonal {diagonal:.1}"
        );
        for path in [
            find_path(&cdt, s, g, &mut sc, 5.0),
            find_path_abstract(&cdt, &abs, s, g, &mut sc, 5.0),
        ] {
            assert_no_constraint_crossing(&cdt, &path);
            let len = polyline_len(&path);
            assert!(
                len <= exact * H_WEIGHT,
                "{len:.1} beyond the weight's bound of {exact:.1}"
            );
        }
    }

    #[test]
    fn test_search_beats_centroid_channel_within_weight() {
        // The exact search minimises the funnel over every channel, so it never
        // loses to the plain centroid channel; the weighted search used by
        // `find_path` stays within `H_WEIGHT` of it.
        let maps = [
            ("rooms6", rooms(6)),
            (
                "test_unit_size_corridors",
                crate::test_utils::build_cdt("test_unit_size_corridors"),
            ),
            (
                "non_square_walls",
                crate::test_utils::build_cdt("non_square_walls"),
            ),
        ];
        for (map, cdt) in &maps {
            let mut sc = scratch();
            for (s, g) in centroid_pairs(cdt, 12) {
                for &r in &[0.0f32, 5.0, 12.0] {
                    let exact = exact_path(cdt, &mut sc, s, g, r);
                    let weighted = find_path(cdt, s, g, &mut sc, r);
                    let (sf, gf) = (cdt.locate_face(s).unwrap(), cdt.locate_face(g).unwrap());
                    let reachable = sf == gf || channel_search(cdt, sf, gf, &mut sc, r);
                    assert_eq!(
                        !exact.is_empty(),
                        reachable,
                        "[{map}] reachability s={s:?} g={g:?} r={r}"
                    );
                    assert_eq!(
                        !weighted.is_empty(),
                        reachable,
                        "[{map}] reachability s={s:?} g={g:?} r={r}"
                    );
                    if !reachable || sf == gf {
                        continue;
                    }
                    let mut plain = Vec::new();
                    {
                        let s_ = &mut sc;
                        s_.valid.sync(cdt, r);
                        funnel(
                            cdt,
                            s,
                            g,
                            &s_.portals,
                            &mut s_.valid,
                            &mut s_.funnel_left,
                            &mut s_.funnel_right,
                            r,
                            &mut plain,
                        );
                    }
                    let (le, lw, lp) = (
                        polyline_len(&exact),
                        polyline_len(&weighted),
                        polyline_len(&plain),
                    );
                    assert!(
                        le <= lp * 1.0001 + 1e-3,
                        "[{map}] exact {le:.2} > centroid channel {lp:.2} s={s:?} g={g:?} r={r}"
                    );
                    assert!(
                        lw <= le * H_WEIGHT + 1e-3,
                        "[{map}] weighted {lw:.2} beyond bound of {le:.2} s={s:?} g={g:?} r={r}"
                    );
                    assert_no_constraint_crossing(cdt, &weighted);
                }
            }
        }
    }
}
