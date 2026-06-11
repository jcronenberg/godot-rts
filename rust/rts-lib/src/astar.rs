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
    /// Predecessor face or half-edge.
    // INVARIANT: `find_path` stores the incoming half-edge; `tra_star` stores the
    // predecessor face index.  The two are always separate calls and never read
    // each other's values, so the buffer is safely reused.
    came_from: Vec<u32>,
    /// `generation[f] == current_gen` iff face `f` was touched this search.
    generation: Vec<u32>,
    current_gen: u32,
    heap: BinaryHeap<Reverse<(u32, u32, u32)>>,
    /// Centroid cache — rebuilt whenever the CDT's [`CDT::version`] changes
    /// (covers both face-count changes and same-count rebuilds of a new mesh).
    centroids: Vec<Vector2>,
    /// CDT version the `centroids` cache was built for (0 = never built).
    centroids_version: u64,
    /// Reusable portal sequence buffer for SSFA.
    portals: Vec<u32>,
    /// Reusable left/right portal-endpoint buffers, shrunk once per funnel call.
    funnel_left: Vec<Vector2>,
    funnel_right: Vec<Vector2>,
    /// Node arena for the anytime channel refinement (TA*).
    nodes: Vec<TaNode>,
    /// Best (lowest) midpoint-polyline distance seen per entry half-edge.
    edge_dom: Vec<f32>,
    /// `edge_gen[he] == current_gen` iff `edge_dom[he]` is from this search.
    edge_gen: Vec<u32>,
    /// Best path found so far during refinement; cloned into the result.
    best_path: Vec<Vector2>,
    /// Funnel output buffer for candidate channels during refinement.
    cand_path: Vec<Vector2>,
    /// Reusable generation-marked buffers for the abstraction's local searches.
    abs_scratch: crate::abstraction::AbsScratch,
    /// Reusable buffers for the TRA* query path. Held separately so they can be
    /// `mem::take`n out as locals during a query — letting them coexist with a
    /// `&mut AStarScratch` borrow without aliasing.
    tra: TraScratch,
}

/// Search state of the anytime refinement: one node per distinct channel
/// prefix, keyed by the half-edge it crossed to enter `face`.
#[derive(Clone, Copy)]
struct TaNode {
    face: u32,
    /// Half-edge crossed to enter `face` (`NONE` for the start node).
    entry_he: u32,
    /// Arena index of the parent node (`NONE` for the start node).
    parent: u32,
    /// Lower bound on the true path length from `start` to `entry_he`.
    g: f32,
    /// Midpoint-polyline distance from `start` to `entry_he` — a cheap
    /// true-length estimate used only for portal dominance.
    d: f32,
}

/// Per-query work buffers for [`find_path_abstract`], pooled to keep TRA*
/// queries allocation-free in steady state.
#[derive(Default)]
struct TraScratch {
    /// Level-3 nodes bordering the start region.
    start_l3s: Vec<u32>,
    /// Level-3 nodes bordering the goal region.
    goal_l3s: Vec<u32>,
    /// Abstract path of level-3 faces from `tra_star`.
    l3_path: Vec<u32>,
    /// Full reconstructed triangle channel.
    channel: Vec<u32>,
    /// Scratch for individual channel segments during reconstruction.
    seg: Vec<u32>,
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
            portals: Vec::new(),
            funnel_left: Vec::new(),
            funnel_right: Vec::new(),
            nodes: Vec::new(),
            edge_dom: Vec::new(),
            edge_gen: Vec::new(),
            best_path: Vec::new(),
            cand_path: Vec::new(),
            abs_scratch: crate::abstraction::AbsScratch::default(),
            tra: TraScratch::default(),
        }
    }

    fn prepare(&mut self, cdt: &CDT) {
        let n = cdt.num_faces() as usize;

        if n > self.g_score.len() {
            self.g_score.resize(n, 0.0);
            self.came_from.resize(n, NONE);
            self.generation.resize(n, 0);
            self.edge_dom.resize(n * 3, 0.0);
            self.edge_gen.resize(n * 3, 0);
        }

        self.current_gen = self.current_gen.wrapping_add(1);
        if self.current_gen == 0 {
            self.generation.fill(0);
            self.edge_gen.fill(0);
            self.current_gen = 1;
        }

        self.heap.clear();

        // Rebuild on a version change (new or mutated mesh), not just a length
        // change — two distinct meshes can share a face count yet have different
        // centroids, which would otherwise silently corrupt pathfinding.
        if self.centroids_version != cdt.version() || self.centroids.len() != n {
            self.centroids.clear();
            self.centroids.reserve(n);
            for f in 0..n as u32 {
                self.centroids.push(cdt.face_centroid(f));
            }
            self.centroids_version = cdt.version();
        }
    }
}

// ── unified pathfinding ───────────────────────────────────────────────────────

/// Find the shortest path for a circular agent of the given `radius`.
///
/// Pass `radius = 0.0` to ignore agent size (any portal is considered
/// passable regardless of width).
///
/// Portals that cannot admit the agent (`radius > portal_radius(he)`) are
/// skipped during A*.  The resulting triangle channel is then smoothed by the
/// Simple Stupid Funnel Algorithm; waypoints that wrap around constraint-edge
/// corners are offset by `radius` along the bisector toward free space so the
/// agent circle clears the wall.
///
/// Search pipeline: a straight-segment walk first (free-line-of-sight queries
/// finish immediately); otherwise a fast centroid-cost channel search supplies
/// a feasible channel, and an anytime TA* refinement (Demyen 2006, §5.4–5.5)
/// then funnel-evaluates alternative channels by true length, so the path is
/// chosen by real distance instead of the centroid proxy.  Refinement effort
/// is bounded (see `refine_channel`), making the result best-effort optimal
/// rather than proven optimal on very large meshes.
///
/// Returns `[start, …, goal]`, or an empty `Vec` when either endpoint is
/// outside the mesh or no passable route exists.
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

    // Fast path: when the raw segment crosses only passable portals, its
    // channel proves reachability and funnels to a (near-)tight upper bound —
    // typically the straight line itself, which ends the query right away.
    if straight_channel(
        cdt,
        start_face,
        goal_face,
        start,
        goal,
        radius,
        &mut scratch.portals,
    ) {
        scratch.prepare(cdt); // advances the epoch and sizes edge_dom/edge_gen for refine_channel
    } else if !channel_search(cdt, start_face, goal_face, scratch, radius) {
        return Vec::new();
    }

    // Initial candidate: funnel the channel for a true-length upper bound
    // that prunes the refinement.
    {
        let s = &mut *scratch;
        funnel(
            cdt,
            start,
            goal,
            &s.portals,
            &mut s.funnel_left,
            &mut s.funnel_right,
            radius,
            &mut s.best_path,
        );
    }
    let best_len = polyline_len(&scratch.best_path);

    refine_channel(
        cdt, start, goal, start_face, goal_face, scratch, radius, best_len,
    );

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
        portals.push(exit_he);
        prev = face;
        face = exit_nb;
    }
    true
}

/// Phase 1: face-keyed A* over centroid-to-centroid costs.  Cheap and
/// complete, but its channel is not necessarily the shortest — the centroid
/// polyline mis-measures real path length by up to the triangle size.
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
            // O(1) passability gate using the precomputed crossing radius — the
            // exact threshold the funnel later enforces (see `compute_widths`).
            if radius > 0.0 && radius > cdt.portal_radius(he) {
                return;
            }

            // g(nb): centroid-to-centroid accumulated cost.  A proxy for path
            // length, good enough to find *a* channel; `refine_channel` fixes
            // any mis-ranking against the true funneled length.
            let tg = g_cur + dist(c_cur, scratch.centroids[nb as usize]);

            // h(nb): centroid-to-goal — consistent, so each face expands at most once.
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

/// Phase 2: anytime TA* refinement (Demyen 2006, §5.4–5.5).
///
/// Searches channel prefixes — one node per (face, entry edge, parent chain),
/// so a face may be reached by several channels — ordered by `f = g + h`
/// where both terms are admissible Euclidean lower bounds.  Each time the goal
/// face is popped, the channel is funneled for its true polyline length; the
/// search stops once the queue front's `f` cannot beat the best length found.
/// `best_len` starts as the phase-1 upper bound and `scratch.best_path` holds
/// the corresponding path.
///
/// Two prunes keep the prefix search from exploding in open space, where the
/// Euclidean bounds are loose and countless interchangeable prefixes exist:
/// a portal is only re-entered by a strictly better midpoint-distance
/// (collapsing same-homotopy duplicates), and an adaptive node budget caps
/// the worst case — it grows only while better channels keep being found, so
/// hopeless searches stop early and the anytime result stands.  Both prunes
/// can in principle hide an alternative channel, so the result is best-effort
/// optimal rather than proven optimal, and never worse than the phase-1
/// channel.
#[allow(clippy::too_many_arguments)]
fn refine_channel(
    cdt: &CDT,
    start: Vector2,
    goal: Vector2,
    start_face: u32,
    goal_face: u32,
    scratch: &mut AStarScratch,
    radius: f32,
    mut best_len: f32,
) {
    let h0 = dist(start, goal);
    // Near-optimal or near-collocated: refinement can't gain >0.01% or >1e-3.
    if best_len <= h0 * 1.0001 + 1e-3 {
        return;
    }

    let pts = cdt.points();
    // Adaptive anytime budget: cheap by default, extended while improving.
    let mut budget = 512usize;
    const BUDGET_MAX: usize = 8192;
    let AStarScratch {
        heap,
        nodes,
        edge_dom,
        edge_gen,
        current_gen,
        best_path,
        cand_path,
        portals,
        funnel_left,
        funnel_right,
        ..
    } = scratch;
    let epoch = *current_gen;

    heap.clear();
    nodes.clear();
    nodes.push(TaNode {
        face: start_face,
        entry_he: NONE,
        parent: NONE,
        g: 0.0,
        d: 0.0,
    });
    heap.push(Reverse((h0.to_bits(), 0u32, 0u32)));

    while let Some(Reverse((f_bits, _, idx))) = heap.pop() {
        // Every unexplored channel has a prefix node in the heap, and each
        // node's `f` lower-bounds all completions through it — so once the
        // heap minimum can't beat `best_len`, no remaining channel can.
        if f32::from_bits(f_bits) >= best_len || nodes.len() >= budget {
            break;
        }
        let TaNode {
            face,
            entry_he,
            g,
            d,
            ..
        } = nodes[idx as usize];

        if face == goal_face {
            portals.clear();
            let mut cur = idx as usize;
            while nodes[cur].entry_he != NONE {
                portals.push(nodes[cur].entry_he);
                cur = nodes[cur].parent as usize;
            }
            portals.reverse();
            funnel(
                cdt,
                start,
                goal,
                portals,
                funnel_left,
                funnel_right,
                radius,
                cand_path,
            );
            let len = polyline_len(cand_path);
            // Copy rather than swap: swapping would ping-pong the two pooled
            // buffers' capacities and re-allocate on later queries.
            if len < best_len {
                best_len = len;
                best_path.clear();
                best_path.extend_from_slice(cand_path);
                // Reward progress with more search room.
                budget = budget.max((nodes.len() * 2).min(BUDGET_MAX));
            }
            continue;
        }

        // Reference point of the portal this node entered through.
        let prev_pt = if entry_he == NONE {
            start
        } else {
            let a = pts[cdt.he_origin(entry_he) as usize];
            let b = pts[cdt.he_dest(entry_he) as usize];
            Vector2::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5)
        };

        cdt.for_each_neighbor(face, |nb, he| {
            if nodes.len() >= budget {
                return;
            }
            if radius > 0.0 && radius > cdt.portal_radius(he) {
                return;
            }

            let pa = pts[cdt.he_origin(he) as usize];
            let pb = pts[cdt.he_dest(he) as usize];
            // h: goal to the entry portal.  g: max of two lower bounds on the
            // walked distance to this portal — straight line from the start,
            // and the parent's bound (path length is monotone along a channel).
            // Demyen's third bound (g + h - h') is NOT admissible for a
            // point-to-edge h and is deliberately omitted.
            let h_nb = dist_point_seg(goal, pa, pb);
            let g_nb = dist_point_seg(start, pa, pb).max(g);
            let f_nb = g_nb + h_nb;
            if f_nb >= best_len {
                return;
            }
            // Portal dominance: when channels converge on a portal, keep only
            // the prefix with the shortest midpoint polyline.  The g-bound
            // can't rank converging prefixes (it's the same straight line for
            // all of them); the midpoint metric tracks the walked route.
            // Goal portals are excepted so every distinct final approach is
            // still funnel-evaluated.
            let mid = Vector2::new((pa.x + pb.x) * 0.5, (pa.y + pb.y) * 0.5);
            let d_nb = d + dist(prev_pt, mid);
            if nb != goal_face && edge_gen[he as usize] == epoch && edge_dom[he as usize] <= d_nb {
                return;
            }
            // A shortest channel never revisits a face (Demyen Thm 4.3.4):
            // drop children whose face is already on this node's chain.
            let mut a = idx as usize;
            loop {
                if nodes[a].face == nb {
                    return;
                }
                let p = nodes[a].parent;
                if p == NONE {
                    break;
                }
                a = p as usize;
            }
            // Recorded only for pushed nodes: a dominance entry from a
            // cycle-pruned prefix could block a valid channel with no live
            // node left at this portal.
            edge_gen[he as usize] = epoch;
            edge_dom[he as usize] = d_nb;
            let nidx = nodes.len() as u32;
            nodes.push(TaNode {
                face: nb,
                entry_he: he,
                parent: idx,
                g: g_nb,
                d: d_nb,
            });
            heap.push(Reverse((f_nb.to_bits(), g_nb.to_bits(), nidx)));
        });
    }
}

// ── TRA* (Triangulation-Reduced A*) ──────────────────────────────────────────

/// Find the shortest path using the pre-built graph abstraction.
///
/// Performs a fast component check, then A* over level-3 (decision-point)
/// triangles only, reconstructs the full triangle channel, and runs
/// the funnel for the final smooth path.
///
/// Falls back to [`find_path`] when the start or goal is in a region with
/// no level-3 ancestor (dead-end tree or ring without a decision point).
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

    // Fast no-path check via component IDs.
    if abs.component_of(start_face) != abs.component_of(goal_face) {
        return Vec::new();
    }

    // Borrow the pooled query buffers out as locals so they can be used
    // alongside `&mut scratch` without aliasing, then restore them.
    let mut tra = std::mem::take(&mut scratch.tra);
    let result = tra_query(
        cdt, abs, start, goal, start_face, goal_face, scratch, &mut tra, radius,
    );
    scratch.tra = tra;
    result
}

/// Worker for [`find_path_abstract`]: `tra` is a detached set of pooled buffers,
/// kept separate from `scratch` so both can be mutably borrowed at once.
#[allow(clippy::too_many_arguments)]
fn tra_query(
    cdt: &CDT,
    abs: &crate::abstraction::Abstraction,
    start: Vector2,
    goal: Vector2,
    start_face: u32,
    goal_face: u32,
    scratch: &mut AStarScratch,
    tra: &mut TraScratch,
    radius: f32,
) -> Vec<Vector2> {
    // Find all level-3 nodes adjacent to start and goal's local regions.
    abs.find_local_l3(
        cdt,
        start_face,
        &mut tra.start_l3s,
        &mut scratch.abs_scratch,
    );
    abs.find_local_l3(cdt, goal_face, &mut tra.goal_l3s, &mut scratch.abs_scratch);

    if tra.start_l3s.is_empty() || tra.goal_l3s.is_empty() {
        return find_path(cdt, start, goal, scratch, radius); // ring or island
    }

    // Both endpoints share a local l3 node (same corridor / region).
    if tra.start_l3s.iter().any(|s| tra.goal_l3s.contains(s)) {
        return find_path(cdt, start, goal, scratch, radius);
    }

    // Multi-source A* over level-3 nodes.
    if !tra_star(
        cdt,
        abs,
        &tra.start_l3s,
        &tra.goal_l3s,
        &mut tra.l3_path,
        scratch,
        radius,
    ) {
        return Vec::new();
    }

    // Reconstruct the full triangle channel from the abstract path.
    if !crate::abstraction::reconstruct_channel(
        cdt,
        abs,
        start_face,
        &tra.l3_path,
        goal_face,
        &mut tra.channel,
        &mut tra.seg,
        &mut scratch.abs_scratch,
    ) {
        return Vec::new();
    }

    // Convert face sequence to portal half-edges and run the funnel.
    scratch.portals.clear();
    scratch.portals.extend(
        tra.channel
            .windows(2)
            .filter_map(|w| cdt.shared_edge_between(w[0], w[1])),
    );
    let s = &mut *scratch;
    funnel(
        cdt,
        start,
        goal,
        &s.portals,
        &mut s.funnel_left,
        &mut s.funnel_right,
        radius,
        &mut s.cand_path,
    );
    s.cand_path.clone()
}

/// Multi-source, multi-target A* restricted to level-3 (decision-point) nodes.
///
/// Seeds the heap with all `start_l3s` (g=0) and stops when any node in
/// `goal_l3s` is popped.  Writes the level-3 face sequence from the chosen
/// start to the chosen goal into `out_path` (cleared first) and returns `true`,
/// or returns `false` when no passable route exists.
///
/// `scratch.came_from` stores predecessor face indices here (not half-edges).
#[allow(clippy::too_many_arguments)]
fn tra_star(
    cdt: &CDT,
    abs: &crate::abstraction::Abstraction,
    start_l3s: &[u32],
    goal_l3s: &[u32],
    out_path: &mut Vec<u32>,
    scratch: &mut AStarScratch,
    radius: f32,
) -> bool {
    scratch.prepare(cdt);
    let epoch = scratch.current_gen;

    // Average goal centroid for the heuristic.
    let goal_centroid: Vector2 = {
        let mut gx = 0.0f32;
        let mut gy = 0.0f32;
        for &g in goal_l3s {
            let c = scratch.centroids[g as usize];
            gx += c.x;
            gy += c.y;
        }
        let n = goal_l3s.len() as f32;
        Vector2::new(gx / n, gy / n)
    };

    let h_of = |f: u32| -> f32 { dist(scratch.centroids[f as usize], goal_centroid) };

    for &s in start_l3s {
        scratch.generation[s as usize] = epoch;
        scratch.g_score[s as usize] = 0.0;
        scratch.came_from[s as usize] = NONE;
        let h = h_of(s);
        scratch.heap.push(Reverse((h.to_bits(), 0u32, s)));
    }

    let mut reached_goal = NONE;

    while let Some(Reverse((_, g_bits, current))) = scratch.heap.pop() {
        let g_cur = if scratch.generation[current as usize] == epoch {
            scratch.g_score[current as usize]
        } else {
            f32::INFINITY
        };
        if g_bits != g_cur.to_bits() {
            continue;
        }
        if goal_l3s.contains(&current) {
            reached_goal = current;
            break;
        }

        for slot in 0..3u32 {
            let nb = abs.l3_neighbor(current, slot);
            if nb == NONE {
                continue;
            }
            let choke = abs.l3_choke_at(current, slot);
            if radius > 0.0 && radius > choke {
                continue; // corridor too narrow
            }

            let tg = g_cur
                + dist(
                    scratch.centroids[current as usize],
                    scratch.centroids[nb as usize],
                );
            let h_nb = h_of(nb);

            let g_nb = if scratch.generation[nb as usize] == epoch {
                scratch.g_score[nb as usize]
            } else {
                f32::INFINITY
            };
            if tg < g_nb {
                scratch.generation[nb as usize] = epoch;
                scratch.g_score[nb as usize] = tg;
                scratch.came_from[nb as usize] = current;
                let f_val = tg + h_nb;
                scratch
                    .heap
                    .push(Reverse((f_val.to_bits(), tg.to_bits(), nb)));
            }
        }
    }

    if reached_goal == NONE {
        return false;
    }

    // Back-track to reconstruct the level-3 path.
    out_path.clear();
    out_path.push(reached_goal);
    let mut cur = reached_goal;
    loop {
        let prev = scratch.came_from[cur as usize];
        if prev == NONE {
            break; // reached a start node
        }
        out_path.push(prev);
        cur = prev;
        if start_l3s.contains(&cur) {
            break;
        }
    }
    out_path.reverse();
    true
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
/// `[t_lo, t_hi]` (from [`CDT::portal_valid_range`]) marks where the agent's
/// centre stays `≥ r` away from every wall incident to the portal endpoints.
#[allow(clippy::too_many_arguments)]
fn funnel(
    cdt: &CDT,
    start: Vector2,
    goal: Vector2,
    portals: &[u32],
    left_buf: &mut Vec<Vector2>,
    right_buf: &mut Vec<Vector2>,
    radius: f32,
    out: &mut Vec<Vector2>,
) {
    // Shrink each portal to its agent-valid endpoints exactly once.  `portal_valid_range`
    // is the expensive O(degree) call; computing it per portal here (rather than per
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
            let (t_lo, t_hi) = cdt.portal_valid_range(he, radius);
            // Empty range (corridor barely admits the agent): collapse to midpoint.
            let (tl, tr) = if t_lo > t_hi {
                (0.5, 0.5)
            } else {
                (t_lo, t_hi)
            };
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

/// Distance from `p` to the closest point on segment `ab`.
#[inline(always)]
fn dist_point_seg(p: Vector2, a: Vector2, b: Vector2) -> f32 {
    let ab = b - a;
    let len2 = ab.x * ab.x + ab.y * ab.y;
    if len2 <= f32::EPSILON {
        return dist(p, a);
    }
    let t = (((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2).clamp(0.0, 1.0);
    dist(p, Vector2::new(a.x + t * ab.x, a.y + t * ab.y))
}

#[inline(always)]
fn polyline_len(path: &[Vector2]) -> f32 {
    path.windows(2).map(|w| dist(w[0], w[1])).sum()
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

    /// Minimum distance from any path SEGMENT to any constraint EDGE (segment).
    fn path_min_clearance(cdt: &CDT, path: &[Vector2]) -> f32 {
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
        // For non-intersecting segments, the closest pair is always one endpoint
        // and one segment-interior point (or two endpoints), so taking the
        // minimum of the four endpoint-to-other-segment distances is exact.
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
        // wasn't, because its centroid costs depended on the start face.
        for map in ["test_unit_size_corridors", "non_square_walls"] {
            let cdt = crate::test_utils::build_cdt(map);
            let mut sc = scratch();
            for (s, g) in centroid_pairs(&cdt, 8) {
                for &r in &[0.0f32, 5.0] {
                    let fwd = find_path(&cdt, s, g, &mut sc, r);
                    let rev = find_path(&cdt, g, s, &mut sc, r);
                    if fwd.is_empty() || rev.is_empty() {
                        continue; // reachability symmetry is tested elsewhere
                    }
                    let (lf, lr) = (polyline_len(&fwd), polyline_len(&rev));
                    assert!(
                        (lf - lr).abs() <= 0.01 * lf.max(lr),
                        "[{map}] asymmetric lengths {lf:.2} vs {lr:.2} s={s:?} g={g:?} r={r}"
                    );
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
        // Prime the cache on mesh A (faces 0 and 1 are distinct → runs prepare()).
        let _ = find_path(&a, a.face_centroid(0), a.face_centroid(1), &mut sc, 0.0);
        // Now query mesh B with the same scratch.
        let _ = find_path(&b, b.face_centroid(0), b.face_centroid(1), &mut sc, 0.0);

        assert_eq!(sc.centroids_version, b.version());
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
        // Path through top gap (50 units wide) with radius 20.
        // All waypoints should stay ≥ 20 units from every constraint vertex.
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
        // radius=0 should delegate to ssfa; path must not be empty and
        // must not cross constraints.
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

    // ── Phase 4: TRA* tests ───────────────────────────────────────────────────

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
            "TRA* should find path through top gap (radius 25)"
        );
        assert_eq!(*path.first().unwrap(), start);
        assert_eq!(*path.last().unwrap(), goal);
        assert_no_constraint_crossing(&cdt, &path);
    }

    #[test]
    fn test_tra_star_query_pooled_steady_state_allocs() {
        // P1 regression guard: after warmup, a TRA* query must not allocate its
        // internal scratch (l3 sets, abstract path, channel, BFS queue). The
        // only allocation that may remain is the returned path Vec and the
        // growth of the reused portal/funnel buffers — bounded by a small
        // constant, independent of the internal search work.
        let (cdt, abs) = corridors_abs();
        let start = Vector2::new(75.0, 175.0);
        let goal = Vector2::new(325.0, 175.0);
        let radius = 5.0;
        let mut sc = AStarScratch::new();

        // Warm up: first call sizes every pooled buffer (centroids, l3 sets,
        // channel, funnel buffers, …).
        let p = find_path_abstract(&cdt, &abs, start, goal, &mut sc, radius);
        assert!(!p.is_empty(), "query must traverse the abstract path");

        // Steady state: run several identical queries. The only remaining
        // allocation is the single pre-sized result Vec that escapes to the
        // caller; all internal scratch is pooled, so the count never grows.
        for _ in 0..4 {
            let allocs = crate::alloc_counter::count_allocs(|| {
                let path = find_path_abstract(&cdt, &abs, start, goal, &mut sc, radius);
                std::hint::black_box(path);
            });
            assert_eq!(
                allocs, 1,
                "steady-state TRA* query should allocate exactly once (the \
                 returned path Vec); got {allocs}"
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
            "TRA* should return empty when all corridors too narrow"
        );
    }

    #[test]
    fn test_tra_matches_regular_astar() {
        // TRA* and regular A* should both find non-empty paths for passable routes.
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
                "TRA* and A* disagree on passability for start={start:?} goal={goal:?} r={r}"
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
            "TRA* should return empty for different components"
        );
    }

    #[test]
    fn test_tra_is_sound_against_astar() {
        // TRA* must be *sound* against the ground-truth full-mesh A* over many
        // start/goal centroid pairs and radii — whenever it returns a path, the
        // exact search must agree one exists, and the path must not cross a
        // constraint.
        //
        // Note it is NOT required to be *complete*: TRA* gates each corridor on
        // its narrowest `portal_radius` (min over the whole corridor), so near the
        // limiting radius it may deny a route that the exact search threads through
        // open space using only part of a corridor.  That conservativeness is a
        // property of the abstraction, never a false positive.
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
                    for &r in &[0.0f32, 1.0, 4.0, 8.0, 25.0] {
                        let t = find_path_abstract(&cdt, &abs, start, goal, &mut sc, r);
                        if t.is_empty() {
                            continue;
                        }
                        let a = find_path(&cdt, start, goal, &mut sc, r);
                        assert!(
                            !a.is_empty(),
                            "TRA* invented a path the exact search rejects: map={map} i={i} j={j} r={r}"
                        );
                        assert_no_constraint_crossing(&cdt, &t);
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
        // A radius-0 funnel path is a taut polyline through the triangle channel:
        // straight between bends, with every bend sitting exactly on a portal
        // endpoint — i.e. a triangulation vertex. An interior waypoint that is not
        // a mesh vertex would mean the funnel invented a corner. (This is the
        // tautness guarantee; the path is channel-optimal, not globally optimal,
        // so its total length is deliberately not asserted.)
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
        // Companion to the TRA* allocation guard: once the scratch is warm, a
        // find_path call's only heap allocation is the returned Vec — all search
        // state (heap, g/came_from/generation, portals, funnel buffers) is pooled.
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
}
