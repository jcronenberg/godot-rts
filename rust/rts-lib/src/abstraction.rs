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
/// `on_step(next_face, entry_half_edge)` for each face stepped into. Only
/// steps onto Corridor/DecisionPoint faces; dead-end branches are ignored.
///
/// Returns `Some(decision_point_face)`, or `None` for a ring without one.
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
        // The exit is the single core neighbour besides `prev`; dead-end
        // branches hanging off the corridor must be skipped, not followed.
        let mut next = NONE;
        let mut next_he = NONE;
        cdt.for_each_neighbor(cur, |nb, he| {
            let in_core = matches!(
                levels[nb as usize],
                NodeLevel::Corridor | NodeLevel::DecisionPoint
            );
            if nb != prev && in_core && next == NONE {
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
            _ => unreachable!("the walk only steps onto Corridor/DecisionPoint faces"),
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::test_utils::build_cdt;
    use godot::prelude::Vector2;
    use std::collections::{HashMap, HashSet};

    /// Real maps with walls, corridors, dead-ends, and (for `wall_no_route`)
    /// disconnected regions. Every property below must hold on all of them.
    const MAPS: &[&str] = &[
        "test_unit_size_corridors",
        "non_square_walls",
        "wall_no_route",
        "square",
    ];

    /// Free-edge adjacency built only from the public traversal API — the ground
    /// truth the abstraction is supposed to summarise.
    fn free_adj(cdt: &CDT) -> Vec<Vec<u32>> {
        (0..cdt.num_faces())
            .map(|f| {
                let mut v = Vec::new();
                cdt.for_each_neighbor(f, |nb, _| v.push(nb));
                v
            })
            .collect()
    }

    /// Independent connected components over free edges (each isolated face is
    /// its own component). Shares no code with `flood_fill_components`.
    fn ref_components(adj: &[Vec<u32>]) -> Vec<u32> {
        let n = adj.len();
        let mut comp = vec![u32::MAX; n];
        let mut next = 0u32;
        for s in 0..n {
            if comp[s] != u32::MAX {
                continue;
            }
            comp[s] = next;
            let mut stack = vec![s as u32];
            while let Some(f) = stack.pop() {
                for &nb in &adj[f as usize] {
                    if comp[nb as usize] == u32::MAX {
                        comp[nb as usize] = next;
                        stack.push(nb);
                    }
                }
            }
            next += 1;
        }
        comp
    }

    /// Independent 4-level classification via iterated leaf removal (the 2-core):
    /// islands have no free exit; dead-ends are the tree faces peeled away; the
    /// surviving core splits into corridors (degree 2) and decision points
    /// (degree >= 3). This is the *definition* Demyen's classification realises,
    /// computed by a different algorithm than `Abstraction::build`.
    fn ref_levels(adj: &[Vec<u32>]) -> Vec<NodeLevel> {
        let n = adj.len();
        let deg0: Vec<usize> = adj.iter().map(|a| a.len()).collect();
        let mut deg: Vec<i64> = deg0.iter().map(|&d| d as i64).collect();
        let mut removed = vec![false; n];
        let mut queue: Vec<u32> = (0..n as u32)
            .filter(|&f| deg0[f as usize] >= 1 && deg[f as usize] <= 1)
            .collect();
        while let Some(f) = queue.pop() {
            if removed[f as usize] {
                continue;
            }
            removed[f as usize] = true;
            for &nb in &adj[f as usize] {
                if !removed[nb as usize] {
                    deg[nb as usize] -= 1;
                    if deg[nb as usize] <= 1 {
                        queue.push(nb);
                    }
                }
            }
        }
        (0..n)
            .map(|f| {
                if deg0[f] == 0 {
                    NodeLevel::Island
                } else if removed[f] {
                    NodeLevel::DeadEnd
                } else if deg[f] == 2 {
                    NodeLevel::Corridor
                } else {
                    NodeLevel::DecisionPoint
                }
            })
            .collect()
    }

    /// Equality for choke widths that treats `f32::INFINITY` (an unconstrained,
    /// freely-passable portal) as a first-class value.
    fn choke_eq(a: f32, b: f32) -> bool {
        if a.is_infinite() || b.is_infinite() {
            a == b
        } else {
            (a - b).abs() <= 1e-3 * a.abs().max(b.abs()).max(1.0)
        }
    }

    /// Hand-rolled corridor walk from decision point `f` out through free
    /// neighbour `start` (reached via half-edge `entry`). Returns the terminal
    /// decision point and the narrowest portal radius along the way, or `None`
    /// for a dead-end or a ring with no decision point — the contract
    /// `l3_neighbor`/`l3_choke_at` must satisfy, derived here independently.
    fn walk(cdt: &CDT, levels: &[NodeLevel], f: u32, start: u32, entry: u32) -> Option<(u32, f32)> {
        let mut min_choke = cdt.portal_radius(entry);
        match levels[start as usize] {
            NodeLevel::DecisionPoint => return Some((start, min_choke)),
            NodeLevel::Island | NodeLevel::DeadEnd => return None,
            NodeLevel::Corridor => {}
        }
        let mut prev = f;
        let mut cur = start;
        for _ in 0..cdt.num_faces() {
            // Continue along the unique non-`prev` core neighbour; dead-end
            // branches off the corridor must be skipped.
            let mut next = NONE;
            let mut next_he = NONE;
            cdt.for_each_neighbor(cur, |nb, he| {
                let in_core = matches!(
                    levels[nb as usize],
                    NodeLevel::Corridor | NodeLevel::DecisionPoint
                );
                if nb != prev && in_core && next == NONE {
                    next = nb;
                    next_he = he;
                }
            });
            if next == NONE {
                return None;
            }
            min_choke = min_choke.min(cdt.portal_radius(next_he));
            match levels[next as usize] {
                NodeLevel::DecisionPoint => return Some((next, min_choke)),
                NodeLevel::Corridor => {
                    prev = cur;
                    cur = next;
                }
                _ => unreachable!("walk only steps onto core faces"),
            }
        }
        None // ring
    }

    /// Reference for `find_local_l3`: the set of decision points reachable from
    /// `f` over free edges without passing *through* another decision point.
    fn ref_local_l3(adj: &[Vec<u32>], levels: &[NodeLevel], f: u32) -> HashSet<u32> {
        let mut out = HashSet::new();
        match levels[f as usize] {
            NodeLevel::Island => return out,
            NodeLevel::DecisionPoint => {
                out.insert(f);
                return out;
            }
            _ => {}
        }
        let mut seen = HashSet::from([f]);
        let mut stack = vec![f];
        while let Some(c) = stack.pop() {
            for &nb in &adj[c as usize] {
                if !seen.insert(nb) {
                    continue;
                }
                match levels[nb as usize] {
                    NodeLevel::DecisionPoint => {
                        out.insert(nb);
                    }
                    NodeLevel::Island => {}
                    _ => stack.push(nb),
                }
            }
        }
        out
    }

    #[test]
    fn components_partition_matches_reference_bfs() {
        // `component_of` must induce exactly the connected components of the
        // free-edge graph. IDs are arbitrary, so compare the *partition* via a
        // bijection rather than the raw numbers.
        for &map in MAPS {
            let cdt = build_cdt(map);
            let abs = Abstraction::build(&cdt);
            let reference = ref_components(&free_adj(&cdt));

            let mut abs_to_ref: HashMap<u32, u32> = HashMap::new();
            let mut ref_to_abs: HashMap<u32, u32> = HashMap::new();
            for f in 0..cdt.num_faces() {
                let a = abs.component_of(f);
                let r = reference[f as usize];
                assert_eq!(
                    *abs_to_ref.entry(a).or_insert(r),
                    r,
                    "[{map}] face {f}: abstraction merges components the reference keeps apart"
                );
                assert_eq!(
                    *ref_to_abs.entry(r).or_insert(a),
                    a,
                    "[{map}] face {f}: abstraction splits a single reference component"
                );
            }
        }
    }

    #[test]
    fn levels_match_two_core_reference() {
        for &map in MAPS {
            let cdt = build_cdt(map);
            let abs = Abstraction::build(&cdt);
            let reference = ref_levels(&free_adj(&cdt));
            for f in 0..cdt.num_faces() {
                assert_eq!(
                    abs.level_of(f),
                    reference[f as usize],
                    "[{map}] face {f} misclassified"
                );
            }
        }
    }

    #[test]
    fn l3_adjacency_matches_independent_corridor_walk() {
        for &map in MAPS {
            let cdt = build_cdt(map);
            let abs = Abstraction::build(&cdt);
            for f in 0..cdt.num_faces() {
                if abs.level_of(f) != NodeLevel::DecisionPoint {
                    // Only decision points carry abstract edges.
                    for s in 0..3u32 {
                        assert_eq!(
                            abs.l3_neighbor(f, s),
                            NONE,
                            "[{map}] non-decision face {f} slot {s} has an l3 edge"
                        );
                    }
                    continue;
                }

                let mut slot_nb = [NONE; 3];
                let mut slot_he = [NONE; 3];
                cdt.for_each_neighbor(f, |nb, he| {
                    let s = (he % 3) as usize;
                    slot_nb[s] = nb;
                    slot_he[s] = he;
                });

                for s in 0..3usize {
                    let got = abs.l3_neighbor(f, s as u32);
                    if slot_nb[s] == NONE {
                        assert_eq!(
                            got, NONE,
                            "[{map}] f{f} slot{s}: blocked edge has an l3 dest"
                        );
                        continue;
                    }
                    match walk(&cdt, &abs.levels, f, slot_nb[s], slot_he[s]) {
                        None => assert_eq!(
                            got, NONE,
                            "[{map}] f{f} slot{s}: corridor leads nowhere but l3 dest is {got}"
                        ),
                        Some((dest, choke)) => {
                            assert_eq!(got, dest, "[{map}] f{f} slot{s}: wrong l3 dest");
                            assert_eq!(
                                abs.level_of(dest),
                                NodeLevel::DecisionPoint,
                                "[{map}] f{f} slot{s}: l3 dest {dest} is not a decision point"
                            );
                            assert!(
                                choke_eq(abs.l3_choke_at(f, s as u32), choke),
                                "[{map}] f{f} slot{s}: choke {} != walked {choke}",
                                abs.l3_choke_at(f, s as u32)
                            );
                            // Corridors are bidirectional: the destination must
                            // carry an abstract edge back to f.
                            let back = (0..3u32).any(|s2| abs.l3_neighbor(dest, s2) == f);
                            assert!(back, "[{map}] l3 edge f{f}->{dest} has no reverse");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn find_local_l3_matches_reachability_spec() {
        for &map in MAPS {
            let cdt = build_cdt(map);
            let abs = Abstraction::build(&cdt);
            let adj = free_adj(&cdt);
            let mut sc = AbsScratch::default();
            let mut out = Vec::new();
            for f in 0..cdt.num_faces() {
                abs.find_local_l3(&cdt, f, &mut out, &mut sc);

                let got: HashSet<u32> = out.iter().copied().collect();
                assert_eq!(got.len(), out.len(), "[{map}] f{f}: duplicate in local l3");
                for &g in &out {
                    assert_eq!(
                        abs.level_of(g),
                        NodeLevel::DecisionPoint,
                        "[{map}] f{f}: returned {g} is not a decision point"
                    );
                }
                assert_eq!(
                    got,
                    ref_local_l3(&adj, &abs.levels, f),
                    "[{map}] f{f}: local l3 set disagrees with reachability"
                );
            }
        }
    }

    #[test]
    fn find_channel_between_is_contiguous_and_respects_components() {
        for &map in MAPS {
            let cdt = build_cdt(map);
            let abs = Abstraction::build(&cdt);
            let nf = cdt.num_faces();
            let mut sc = AbsScratch::default();
            let mut out = Vec::new();
            for a in 0..nf {
                for b in 0..nf {
                    let ok = find_channel_between(&cdt, a, b, &mut out, &mut sc);
                    assert_eq!(
                        ok,
                        abs.component_of(a) == abs.component_of(b),
                        "[{map}] reachability {a}->{b} disagrees with component ids"
                    );
                    if !ok {
                        continue;
                    }
                    assert_eq!(
                        *out.first().unwrap(),
                        a,
                        "[{map}] channel must start at {a}"
                    );
                    assert_eq!(*out.last().unwrap(), b, "[{map}] channel must end at {b}");
                    for w in out.windows(2) {
                        assert!(
                            cdt.shared_edge_between(w[0], w[1]).is_some(),
                            "[{map}] channel faces {} and {} are not adjacent",
                            w[0],
                            w[1]
                        );
                    }
                    let uniq: HashSet<u32> = out.iter().copied().collect();
                    assert_eq!(uniq.len(), out.len(), "[{map}] channel revisits a face");
                }
            }
        }
    }

    #[test]
    fn build_on_minimal_mesh_is_consistent() {
        // Smallest possible triangulation: a single triangle bounded entirely by
        // the convex hull (no free edges) is an island with no abstract edges.
        let cdt = CDT::triangulate(vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(10.0, 0.0),
            Vector2::new(5.0, 10.0),
        ]);
        let abs = Abstraction::build(&cdt);
        assert_eq!(cdt.num_faces(), 1);
        assert_eq!(abs.levels.len(), 1);
        assert_eq!(abs.level_of(0), NodeLevel::Island);
        for s in 0..3u32 {
            assert_eq!(abs.l3_neighbor(0, s), NONE);
        }
    }
}
