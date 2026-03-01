use std::cmp::Reverse;
use std::collections::BinaryHeap;

use godot::prelude::Vector2;
use crate::delaunay::{CDT, NONE};

// ── scratch buffer ────────────────────────────────────────────────────────────

/// Reusable per-agent buffers that eliminate heap allocation on repeated calls.
///
/// Create one `AStarScratch` per agent (or per OS thread if agents share a
/// thread), keep it alive, and pass `&mut` to `find_path_with_scratch`.
///
/// On the first call the buffers are allocated and the centroid cache is
/// built.  Every subsequent call on the same CDT reuses them with no
/// allocation: "invalidation" is a single counter increment rather than a
/// memset of the g-score and came-from arrays.
pub struct AStarScratch {
    g_score:      Vec<f32>,
    came_from_he: Vec<u32>,
    /// `generation[f] == current_gen` iff face `f` was touched this search.
    generation:   Vec<u32>,
    current_gen:  u32,
    heap:         BinaryHeap<Reverse<(u32, u32, u32)>>,
    /// Centroid cache — rebuilt only when `num_faces` changes.
    centroids:    Vec<Vector2>,
}

impl Default for AStarScratch {
    fn default() -> Self { Self::new() }
}

impl AStarScratch {
    pub fn new() -> Self {
        Self {
            g_score:      Vec::new(),
            came_from_he: Vec::new(),
            generation:   Vec::new(),
            current_gen:  1,
            heap:         BinaryHeap::new(),
            centroids:    Vec::new(),
        }
    }

    fn prepare(&mut self, cdt: &CDT) {
        let n = cdt.num_faces() as usize;

        // Grow arrays on first use or if the CDT gained faces (never shrink).
        if n > self.g_score.len() {
            self.g_score.resize(n, 0.0);
            self.came_from_he.resize(n, NONE);
            self.generation.resize(n, 0);
        }

        // Bump generation — previous entries are now implicitly stale (O(1)).
        self.current_gen = self.current_gen.wrapping_add(1);
        if self.current_gen == 0 {
            // Wrapping: reset so generation[f]==0 never matches current_gen==1.
            self.generation.fill(0);
            self.current_gen = 1;
        }

        // Discard any leftover heap items from the previous call.
        self.heap.clear();

        // Rebuild centroids only when the mesh changes.
        if self.centroids.len() != n {
            self.centroids.resize(n, Vector2::ZERO);
            for f in 0..n as u32 {
                self.centroids[f as usize] = cdt.face_centroid(f);
            }
        }
    }
}

// ── scratch-based pathfinding ─────────────────────────────────────────────────

/// A* using pre-allocated scratch buffers — zero allocation in steady state.
///
/// Returns `[start, edge_midpoint, …, goal]`.
/// Empty vec if either point is outside the mesh or no path exists.
pub fn find_path_with_scratch(
    cdt:     &CDT,
    start:   Vector2,
    goal:    Vector2,
    scratch: &mut AStarScratch,
) -> Vec<Vector2> {
    let start_face = match cdt.locate_face(start) { Some(f) => f, None => return Vec::new() };
    let goal_face  = match cdt.locate_face(goal)  { Some(f) => f, None => return Vec::new() };

    if start_face == goal_face {
        return vec![start, goal];
    }

    scratch.prepare(cdt);
    let epoch = scratch.current_gen;

    // Seed the start face.
    scratch.generation  [start_face as usize] = epoch;
    scratch.g_score     [start_face as usize] = 0.0;
    scratch.came_from_he[start_face as usize] = NONE;

    let h0 = dist(scratch.centroids[start_face as usize],
                  scratch.centroids[goal_face  as usize]);
    scratch.heap.push(Reverse((h0.to_bits(), 0u32, start_face)));

    while let Some(Reverse((_, g_bits, current))) = scratch.heap.pop() {
        // Lazy-deletion: skip if a better path already superseded this entry.
        let g_cur = if scratch.generation[current as usize] == epoch {
            scratch.g_score[current as usize]
        } else {
            f32::INFINITY
        };
        if g_bits != g_cur.to_bits() { continue; }
        if current == goal_face { break; }

        let c_cur = scratch.centroids[current as usize];

        cdt.for_each_neighbor(current, |nb, he| {
            // All fields accessed here are distinct — Rust allows disjoint
            // field borrows through &mut even within a closure.
            let c_nb = scratch.centroids[nb as usize];           // Copy → borrow ends
            let tg   = g_cur + dist(c_cur, c_nb);
            let g_nb = if scratch.generation[nb as usize] == epoch { // Copy → borrow ends
                scratch.g_score[nb as usize]
            } else {
                f32::INFINITY
            };
            if tg < g_nb {
                scratch.generation  [nb as usize] = epoch;
                scratch.g_score     [nb as usize] = tg;
                scratch.came_from_he[nb as usize] = he;
                let f_val = tg + dist(c_nb, goal);
                scratch.heap.push(Reverse((f_val.to_bits(), tg.to_bits(), nb)));
            }
        });
    }

    // goal_face generation was never updated → no path.
    if scratch.generation[goal_face as usize] != epoch {
        return Vec::new();
    }

    // Reconstruct: walk came_from_he goal→start, collect edge midpoints.
    let mut mids = Vec::new();
    let mut cur  = goal_face;
    while cur != start_face {
        let he = scratch.came_from_he[cur as usize];
        mids.push(cdt.edge_midpoint(he));
        cur = cdt.face_of_he(he);
    }
    mids.reverse();

    let mut path = Vec::with_capacity(mids.len() + 2);
    path.push(start);
    path.extend(mids);
    path.push(goal);
    path
}

// ── one-shot pathfinding ──────────────────────────────────────────────────────

/// Find a path allocating fresh buffers each call.
///
/// Simple entry point for one-off queries.  For repeated queries on the same
/// CDT prefer `find_path_with_scratch` to avoid per-call allocation.
pub fn find_path(cdt: &CDT, start: Vector2, goal: Vector2) -> Vec<Vector2> {
    let start_face = match cdt.locate_face(start) { Some(f) => f, None => return Vec::new() };
    let goal_face  = match cdt.locate_face(goal)  { Some(f) => f, None => return Vec::new() };

    if start_face == goal_face {
        return vec![start, goal];
    }

    let n = cdt.num_faces() as usize;

    let centroids: Vec<Vector2> = (0..n as u32).map(|f| cdt.face_centroid(f)).collect();
    let mut g_score:    Vec<f32> = vec![f32::INFINITY; n];
    let mut came_from_he: Vec<u32> = vec![NONE; n];
    let mut heap: BinaryHeap<Reverse<(u32, u32, u32)>> =
        BinaryHeap::with_capacity(n / 4 + 8);

    g_score[start_face as usize] = 0.0;
    let h0 = dist(centroids[start_face as usize], centroids[goal_face as usize]);
    heap.push(Reverse((h0.to_bits(), 0u32, start_face)));

    while let Some(Reverse((_, g_bits, current))) = heap.pop() {
        if g_bits != g_score[current as usize].to_bits() { continue; }
        if current == goal_face { break; }

        let g_cur = g_score[current as usize];
        let c_cur = centroids[current as usize];

        cdt.for_each_neighbor(current, |nb, he| {
            let c_nb = centroids[nb as usize];
            let tg   = g_cur + dist(c_cur, c_nb);
            if tg < g_score[nb as usize] {
                g_score[nb as usize]      = tg;
                came_from_he[nb as usize] = he;
                let f = tg + dist(c_nb, goal);
                heap.push(Reverse((f.to_bits(), tg.to_bits(), nb)));
            }
        });
    }

    if came_from_he[goal_face as usize] == NONE { return Vec::new(); }

    let mut mids = Vec::new();
    let mut cur  = goal_face;
    while cur != start_face {
        let he = came_from_he[cur as usize];
        mids.push(cdt.edge_midpoint(he));
        cur = cdt.face_of_he(he);
    }
    mids.reverse();

    let mut path = Vec::with_capacity(mids.len() + 2);
    path.push(start);
    path.extend(mids);
    path.push(goal);
    path
}

// ── shared helper ─────────────────────────────────────────────────────────────

#[inline(always)]
fn dist(a: Vector2, b: Vector2) -> f32 {
    let dx = a.x - b.x;
    let dy = a.y - b.y;
    (dx * dx + dy * dy).sqrt()
}
