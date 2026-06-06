use std::collections::VecDeque;

use crate::delaunay::{CDT, NONE};

// ── Reusable search scratch ───────────────────────────────────────────────────

/// Generation-marked buffers for the abstraction's per-query local searches
/// (`find_local_l3`, `find_channel_between`).  Reusing them turns the previous
/// `vec![..; num_faces]` allocate-and-init on every query into an O(1) epoch
/// bump, so TRA* queries stay sublinear in the mesh size.
#[derive(Default)]
pub struct AbsScratch {
    /// `mark[f] == gen` iff face `f` was visited in the current search.
    mark: Vec<u32>,
    /// Predecessor face for path reconstruction (valid only when `mark[f] == epoch`).
    pred: Vec<u32>,
    epoch: u32,
    /// Reusable BFS frontier for `find_channel_between`.
    queue: VecDeque<u32>,
    /// Reusable DFS frontier for `find_local_l3`.
    stack: Vec<u32>,
}

impl AbsScratch {
    /// Begin a fresh search over `n` faces; returns the epoch to stamp into `mark`.
    fn begin(&mut self, n: usize) -> u32 {
        if self.mark.len() < n {
            self.mark.resize(n, 0);
            self.pred.resize(n, NONE);
        }
        self.epoch = self.epoch.wrapping_add(1);
        if self.epoch == 0 {
            self.mark.fill(0);
            self.epoch = 1;
        }
        self.epoch
    }
}

// ── Triangle classification ───────────────────────────────────────────────────

/// Four-level classification for triangles in the CDT (Demyen 2006, §6).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum NodeLevel {
    /// All 3 edges constrained — isolated pocket with no connections.
    Island = 0,
    /// At most 1 free exit — forms a tree hanging off a corridor or decision point.
    DeadEnd = 1,
    /// Exactly 2 free exits — links two decision points (or forms a ring).
    Corridor = 2,
    /// 3 free exits — junction between multiple corridors.
    DecisionPoint = 3,
}

// ── Abstraction ───────────────────────────────────────────────────────────────

/// Pre-computed graph abstraction over the CDT.
///
/// Classifies every triangle and builds an abstract graph whose nodes are
/// level-3 (decision-point) triangles.  TRA* searches this graph, then
/// reconstructs the full triangle channel for the funnel algorithm.
pub struct Abstraction {
    /// Level of each face (parallel to CDT faces).
    pub levels: Vec<NodeLevel>,
    /// Connected component ID for each face.  Faces in different components
    /// have no path between them.
    pub components: Vec<u32>,
    /// For each face × 3 edge slots: the level-3 face reachable through that
    /// edge (following a corridor if needed).  NONE when the edge is blocked
    /// or leads into a dead-end tree.  Indexed as `l3_adj[face * 3 + slot]`.
    pub l3_adj: Vec<u32>,
    /// Maximum agent radius that can traverse the corridor to the adjacent
    /// level-3 face (the corridor's narrowest [`CDT::portal_radius`]).
    pub l3_choke: Vec<f32>,
}

impl Abstraction {
    /// Build the abstraction from a fully triangulated CDT (after
    /// `compute_widths()` has been called).
    pub fn build(cdt: &CDT) -> Self {
        let n = cdt.num_faces() as usize;
        if n == 0 {
            return Abstraction {
                levels: Vec::new(),
                components: Vec::new(),
                l3_adj: Vec::new(),
                l3_choke: Vec::new(),
            };
        }

        // ── Step 1: count free exits per face ────────────────────────────────
        // A "free" edge is unconstrained and has a neighbour (for_each_neighbor).
        let mut free = vec![0u8; n];
        for f in 0..n as u32 {
            cdt.for_each_neighbor(f, |_, _| {
                free[f as usize] += 1;
            });
        }

        // ── Step 2: seed level-0/1, propagate dead-ends ──────────────────────
        let mut levels = vec![NodeLevel::Corridor; n]; // placeholder
        let mut queue: VecDeque<u32> = VecDeque::new();

        for f in 0..n as u32 {
            match free[f as usize] {
                0 => levels[f as usize] = NodeLevel::Island,
                1 => {
                    levels[f as usize] = NodeLevel::DeadEnd;
                    queue.push_back(f);
                }
                _ => {} // will be set after propagation
            }
        }

        while let Some(f) = queue.pop_front() {
            // Notify f's free neighbours that one exit is now blocked.
            cdt.for_each_neighbor(f, |nb, _| {
                let nb = nb as usize;
                if levels[nb] != NodeLevel::Corridor {
                    return; // already classified
                }
                free[nb] -= 1;
                if free[nb] == 0 {
                    levels[nb] = NodeLevel::Island; // (degenerate; shouldn't happen in practice)
                } else if free[nb] == 1 {
                    levels[nb] = NodeLevel::DeadEnd;
                    queue.push_back(nb as u32);
                }
            });
        }

        // ── Step 3: classify remaining faces as Corridor or DecisionPoint ────
        for f in 0..n {
            // free[f] == 2 stays Corridor
            if levels[f] == NodeLevel::Corridor && free[f] >= 3 {
                levels[f] = NodeLevel::DecisionPoint;
            }
        }

        // ── Step 4: flood-fill component IDs ─────────────────────────────────
        let components = flood_fill_components(cdt, &levels, n);

        // ── Step 5: build abstract graph via corridor traversal ───────────────
        let mut l3_adj = vec![NONE; n * 3];
        let mut l3_choke = vec![0.0f32; n * 3];

        for f in 0..n as u32 {
            if levels[f as usize] != NodeLevel::DecisionPoint {
                continue;
            }
            cdt.for_each_neighbor(f, |nb, he| {
                let slot = (he % 3) as usize;
                match levels[nb as usize] {
                    NodeLevel::Island | NodeLevel::DeadEnd => {} // skip
                    NodeLevel::DecisionPoint => {
                        l3_adj[f as usize * 3 + slot] = nb;
                        l3_choke[f as usize * 3 + slot] = cdt.portal_radius(he);
                    }
                    NodeLevel::Corridor => {
                        let entry_choke = cdt.portal_radius(he);
                        if let Some((dest, choke)) =
                            follow_corridor(cdt, &levels, f, nb, entry_choke)
                        {
                            l3_adj[f as usize * 3 + slot] = dest;
                            l3_choke[f as usize * 3 + slot] = choke;
                        }
                    }
                }
            });
        }

        Abstraction {
            levels,
            components,
            l3_adj,
            l3_choke,
        }
    }

    /// Connected component of face `f`.
    #[inline]
    pub fn component_of(&self, f: u32) -> u32 {
        self.components[f as usize]
    }

    /// Classification level of face `f`.
    #[inline]
    pub fn level_of(&self, f: u32) -> NodeLevel {
        self.levels[f as usize]
    }

    /// Level-3 neighbour reachable through edge slot `s` of face `f`.
    /// Returns `NONE` when the corridor is blocked or leads into dead-end space.
    #[inline]
    pub fn l3_neighbor(&self, f: u32, s: u32) -> u32 {
        self.l3_adj[f as usize * 3 + s as usize]
    }

    /// Maximum agent radius that can traverse the corridor through slot `s` of face `f`.
    #[inline]
    pub fn l3_choke_at(&self, f: u32, s: u32) -> f32 {
        self.l3_choke[f as usize * 3 + s as usize]
    }

    /// Find all level-3 (decision-point) faces adjacent to the local
    /// corridor/dead-end sub-region containing `face`.
    ///
    /// BFS expands through level-1 and level-2 faces but **stops** at
    /// level-3 faces (collecting them as the boundary).  This way a
    /// corridor face between two decision points returns both of them,
    /// while a dead-end tree returns only its single root decision point.
    ///
    /// Writes the result into `out` (cleared first); empty for islands or rings
    /// with no decision point. Uses the pooled buffers in `sc` — no allocation.
    pub fn find_local_l3(&self, cdt: &CDT, face: u32, out: &mut Vec<u32>, sc: &mut AbsScratch) {
        out.clear();
        match self.levels[face as usize] {
            NodeLevel::Island => return,
            NodeLevel::DecisionPoint => {
                out.push(face);
                return;
            }
            _ => {}
        }

        let epoch = sc.begin(cdt.num_faces() as usize);
        let levels = &self.levels;
        let mark = &mut sc.mark;
        let stack = &mut sc.stack;
        stack.clear();
        stack.push(face);
        mark[face as usize] = epoch;

        // Each face is marked the first time it is seen, so every decision point
        // is collected exactly once — no post-hoc dedup needed.
        while let Some(f) = stack.pop() {
            cdt.for_each_neighbor(f, |nb, _| {
                if mark[nb as usize] == epoch {
                    return;
                }
                mark[nb as usize] = epoch;
                match levels[nb as usize] {
                    NodeLevel::Island => {}
                    NodeLevel::DecisionPoint => out.push(nb),
                    _ => stack.push(nb),
                }
            });
        }
    }
}

// ── Internal helpers ─────────────────────────────────────────────────────────

/// Walk a corridor from `start_corridor` (entered from `prev_face`), invoking
/// `on_step(next_face, entry_half_edge)` for each face stepped into (including
/// the terminal decision point or the dead-end face that stops the walk).
///
/// Returns `Some(decision_point_face)` when the corridor ends at a level-3 node,
/// or `None` if it leads into a dead-end/island or forms a ring without one.
fn walk_corridor(
    cdt: &CDT,
    levels: &[NodeLevel],
    prev_face: u32,
    start_corridor: u32,
    mut on_step: impl FnMut(u32, u32),
) -> Option<u32> {
    let mut prev = prev_face;
    let mut cur = start_corridor;
    let max_steps = cdt.num_faces() as usize;

    for _ in 0..max_steps {
        // The exit is the single free edge not leading back to `prev`.
        let mut next = NONE;
        let mut next_he = NONE;
        cdt.for_each_neighbor(cur, |nb, he| {
            if nb != prev && next == NONE {
                next = nb;
                next_he = he;
            }
        });

        if next == NONE {
            return None;
        }
        on_step(next, next_he);

        match levels[next as usize] {
            NodeLevel::DecisionPoint => return Some(next),
            NodeLevel::Corridor => {
                prev = cur;
                cur = next;
            }
            _ => return None, // dead-end or island
        }
    }
    None // ring
}

/// Follow a corridor from `start_corridor` (entered from `prev_face`) and
/// return `(destination_l3_face, min_choke_width)`, or `None` if the corridor
/// leads into a dead-end or forms a ring without a decision point.
fn follow_corridor(
    cdt: &CDT,
    levels: &[NodeLevel],
    prev_face: u32,
    start_corridor: u32,
    entry_choke: f32,
) -> Option<(u32, f32)> {
    let mut min_choke = entry_choke;
    let dest = walk_corridor(cdt, levels, prev_face, start_corridor, |_, he| {
        min_choke = min_choke.min(cdt.portal_radius(he));
    })?;
    Some((dest, min_choke))
}

/// Follow a corridor, appending the full triangle sequence (from
/// `start_corridor` to the first level-3 face reached) onto `channel` via
/// [`push_unique`]. Returns `false` if the corridor is a ring or dead-end.
pub fn follow_corridor_channel(
    cdt: &CDT,
    levels: &[NodeLevel],
    prev_face: u32,
    start_corridor: u32,
    channel: &mut Vec<u32>,
) -> bool {
    push_unique(channel, start_corridor);
    walk_corridor(cdt, levels, prev_face, start_corridor, |face, _| {
        push_unique(channel, face);
    })
    .is_some()
}

/// BFS from `from_face` to `to_face` over free edges, writing the face sequence
/// (inclusive of both endpoints) into `out` (cleared first). Returns `false` if
/// unreachable. Uses the pooled buffers in `sc` — no allocation.
pub fn find_channel_between(
    cdt: &CDT,
    from: u32,
    to: u32,
    out: &mut Vec<u32>,
    sc: &mut AbsScratch,
) -> bool {
    out.clear();
    if from == to {
        out.push(from);
        return true;
    }

    let epoch = sc.begin(cdt.num_faces() as usize);
    let mark = &mut sc.mark;
    let pred = &mut sc.pred;
    let queue = &mut sc.queue;
    queue.clear();
    mark[from as usize] = epoch;
    queue.push_back(from);

    while let Some(f) = queue.pop_front() {
        if mark[to as usize] == epoch {
            break; // `to` already reached and its predecessor recorded
        }
        cdt.for_each_neighbor(f, |nb, _| {
            if mark[nb as usize] == epoch {
                return;
            }
            mark[nb as usize] = epoch;
            pred[nb as usize] = f;
            queue.push_back(nb);
        });
    }

    if mark[to as usize] != epoch {
        return false;
    }

    out.push(to);
    let mut cur = to;
    while cur != from {
        cur = pred[cur as usize];
        out.push(cur);
    }
    out.reverse();
    true
}

/// Reconstruct the full triangle channel for a TRA* abstract path.
///
/// `l3_path` is a sequence of level-3 faces from nearest-to-start to
/// nearest-to-goal.  The channel (written into `channel`, cleared first) runs
/// from `start_face` to `goal_face`, passing through those level-3 nodes.
/// `seg` is a reusable scratch buffer. Returns `false` if reconstruction fails.
#[allow(clippy::too_many_arguments)]
pub fn reconstruct_channel(
    cdt: &CDT,
    abs: &Abstraction,
    start_face: u32,
    l3_path: &[u32],
    goal_face: u32,
    channel: &mut Vec<u32>,
    seg: &mut Vec<u32>,
    sc: &mut AbsScratch,
) -> bool {
    assert!(!l3_path.is_empty());
    let start_l3 = l3_path[0];
    let goal_l3 = *l3_path.last().unwrap();

    channel.clear();

    // 1. start_face → start_l3
    if !find_channel_between(cdt, start_face, start_l3, seg, sc) {
        return false;
    }
    channel.extend_from_slice(seg);

    // 2. Walk the abstract path, appending corridors between l3 nodes.
    for i in 0..l3_path.len().saturating_sub(1) {
        let l3_a = l3_path[i];
        let l3_b = l3_path[i + 1];

        // Find which free edge of l3_a leads toward l3_b (directly or via corridor).
        let mut first_nb = NONE;
        cdt.for_each_neighbor(l3_a, |nb, he| {
            if first_nb != NONE {
                return;
            }
            let slot = he % 3;
            if abs.l3_neighbor(l3_a, slot) == l3_b {
                first_nb = nb;
            }
        });

        if first_nb == NONE {
            return false;
        }

        if abs.levels[first_nb as usize] == NodeLevel::DecisionPoint {
            // Direct l3→l3 edge, no corridor.
            push_unique(channel, l3_b);
        } else if !follow_corridor_channel(cdt, &abs.levels, l3_a, first_nb, channel) {
            // Follow corridor from first_nb to l3_b.
            return false;
        }
    }

    // 3. goal_l3 → goal_face
    if goal_face != goal_l3 {
        if !find_channel_between(cdt, goal_l3, goal_face, seg, sc) {
            return false;
        }
        for &f in seg.iter().skip(1) {
            push_unique(channel, f);
        }
    }

    true
}

/// Flood-fill to assign a connected component ID to each face.
/// Constrained and boundary edges are treated as walls.
fn flood_fill_components(cdt: &CDT, levels: &[NodeLevel], n: usize) -> Vec<u32> {
    let mut components = vec![NONE; n];
    let mut next_comp = 0u32;

    for start in 0..n as u32 {
        if components[start as usize] != NONE {
            continue;
        }
        let comp = next_comp;
        next_comp += 1;
        components[start as usize] = comp;

        if levels[start as usize] == NodeLevel::Island {
            continue; // islands don't connect through free edges
        }

        let mut stack = vec![start];
        while let Some(f) = stack.pop() {
            cdt.for_each_neighbor(f, |nb, _| {
                if components[nb as usize] == NONE {
                    components[nb as usize] = comp;
                    stack.push(nb);
                }
            });
        }
    }

    components
}

// Deduplicates only consecutive equal values.  Duplicates can arise at the
// seam between a corridor segment and the decision-point face it ends on,
// since both `follow_corridor_channel` (which appends the terminal face) and
// the outer loop (which is about to append the same face again) contribute it.
#[inline]
fn push_unique(v: &mut Vec<u32>, x: u32) {
    if v.last() != Some(&x) {
        v.push(x);
    }
}
