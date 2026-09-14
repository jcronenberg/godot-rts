//! Grid ground truth: what an *optimal* route for a disc of a given radius
//! costs, and whether one exists. The sim's pathfinder is under test, so it
//! cannot also be the yardstick; this shares no code with `astar`, rasterising
//! free space at the query radius and running Dijkstra from the goal.
//!
//! Accuracy:
//!
//! - **Metrication.** Grid distance overestimates Euclidean. The 16-neighbour
//!   stencil here caps the error near 1.3% (plain 8-connectivity is 8.2%), so
//!   a *true* optimum scores a suboptimality ratio a hair below 1.
//! - **Sampling.** A cell is free when its centre clears every wall by
//!   `radius - cell/2`. The half-cell slack keeps an exact-fit corridor from
//!   reading as sealed, biasing toward *reachable*: `phantoms` conservative
//!   and `refusals` eager, which is the right way round.
//! - **Endpoints** snap to the nearest free cell within two cells, past which
//!   the endpoint is treated as not fitting. Queries are answered at snapped
//!   cell *centres*, so each contributes up to `cell * sqrt(2)` either way.
//!
//! Budget: +1.3% relative plus `±2 * cell * sqrt(2)` absolute, at a pitch of a
//! quarter radius, which is well under a percent here. Fixed per (map, radius,
//! goal), so it biases absolute ratios but never scorecard *deltas*.
//!
//! Fields are keyed by (walls, radius, cell, goal) and cached under
//! `target/quality/ref_*.bin`.

use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};

use godot::prelude::Vector2;

/// Cells whose centre clears every wall by this much less than the true
/// radius still count as free; see the module doc.
const SAMPLE_SLACK_CELLS: f32 = 0.5;

/// How far an endpoint may be nudged to land on a free cell.
const SNAP_CELLS: i32 = 2;

const MAGIC: &[u8; 4] = b"QREF";
const FORMAT: u32 = 2;

/// Free-space raster at one radius, plus the cost-to-goal field over it.
pub struct Field {
    origin: Vector2,
    cell: f32,
    cols: i32,
    rows: i32,
    goal: Vector2,
    /// Cost from each cell to the goal; `INFINITY` for blocked or cut off.
    dist: Vec<f32>,
}

impl Field {
    /// The point every cost here is measured to.
    pub fn goal(&self) -> Vector2 {
        self.goal
    }

    /// Cost of the optimal route from `start` to the field's goal, or `None`
    /// when no route exists for a disc of this radius.
    pub fn optimal_len(&self, start: Vector2) -> Option<f32> {
        let c = self.snap(start)?;
        let d = self.dist[c];
        d.is_finite().then_some(d)
    }

    fn idx(&self, cx: i32, cy: i32) -> Option<usize> {
        (cx >= 0 && cy >= 0 && cx < self.cols && cy < self.rows)
            .then(|| (cy * self.cols + cx) as usize)
    }

    fn cell_of(&self, p: Vector2) -> (i32, i32) {
        (
            ((p.x - self.origin.x) / self.cell).floor() as i32,
            ((p.y - self.origin.y) / self.cell).floor() as i32,
        )
    }

    /// Cell nearest `p` that `ok` accepts, within [`SNAP_CELLS`]. *Nearest*,
    /// not first-found: the field seed comes through here too, and a corner of
    /// the search square would shift every distance by a couple of cells.
    fn nearest(&self, p: Vector2, ok: impl Fn(usize) -> bool) -> Option<usize> {
        let (cx, cy) = self.cell_of(p);
        if let Some(i) = self.idx(cx, cy)
            && ok(i)
        {
            return Some(i);
        }
        let mut best: Option<(i32, usize)> = None;
        for dy in -SNAP_CELLS..=SNAP_CELLS {
            for dx in -SNAP_CELLS..=SNAP_CELLS {
                let Some(i) = self.idx(cx + dx, cy + dy) else {
                    continue;
                };
                if !ok(i) {
                    continue;
                }
                // Strict `<` and a fixed scan order break ties identically.
                let d2 = dx * dx + dy * dy;
                if best.is_none_or(|(bd, _)| d2 < bd) {
                    best = Some((d2, i));
                }
            }
        }
        best.map(|(_, i)| i)
    }

    /// Nearest cell to `p` with a finite cost, within [`SNAP_CELLS`].
    fn snap(&self, p: Vector2) -> Option<usize> {
        self.nearest(p, |i| self.dist[i].is_finite())
    }
}

/// One stencil entry: `(dx, dy, cost, cells the move passes over)`. Those two
/// cells reject a knight move that cuts a wall corner; straight and diagonal
/// moves leave them `(0, 0)`, meaning "no check".
type Step = (i32, i32, f32, [(i32, i32); 2]);

/// The 16-neighbour stencil.
const STENCIL: [Step; 16] = [
    (1, 0, 1.0, [(0, 0), (0, 0)]),
    (-1, 0, 1.0, [(0, 0), (0, 0)]),
    (0, 1, 1.0, [(0, 0), (0, 0)]),
    (0, -1, 1.0, [(0, 0), (0, 0)]),
    (1, 1, std::f32::consts::SQRT_2, [(1, 0), (0, 1)]),
    (1, -1, std::f32::consts::SQRT_2, [(1, 0), (0, -1)]),
    (-1, 1, std::f32::consts::SQRT_2, [(-1, 0), (0, 1)]),
    (-1, -1, std::f32::consts::SQRT_2, [(-1, 0), (0, -1)]),
    (2, 1, 2.236_068, [(1, 0), (1, 1)]),
    (2, -1, 2.236_068, [(1, 0), (1, -1)]),
    (-2, 1, 2.236_068, [(-1, 0), (-1, 1)]),
    (-2, -1, 2.236_068, [(-1, 0), (-1, -1)]),
    (1, 2, 2.236_068, [(0, 1), (1, 1)]),
    (-1, 2, 2.236_068, [(0, 1), (-1, 1)]),
    (1, -2, 2.236_068, [(0, -1), (1, -1)]),
    (-1, -2, 2.236_068, [(0, -1), (-1, -1)]),
];

/// Cost-to-`goal` field for a disc of `radius` over `walls`.
///
/// `cell` is the raster pitch; smaller is more faithful and slower. Loaded
/// from `cache_dir` when a previous run left a matching field there.
pub fn field(
    walls: &[(Vector2, Vector2)],
    radius: f32,
    cell: f32,
    goal: Vector2,
    cache_dir: &Path,
) -> Field {
    let key = cache_key(walls, radius, cell, goal);
    let path = cache_dir.join(format!("ref_{key:016x}.bin"));
    if let Some(f) = load(&path) {
        return f;
    }
    let f = compute(walls, radius, cell, goal);
    let _ = std::fs::create_dir_all(cache_dir);
    let _ = store(&path, &f);
    f
}

fn compute(walls: &[(Vector2, Vector2)], radius: f32, cell: f32, goal: Vector2) -> Field {
    // Bounds cover every wall endpoint plus the goal, with a margin so the
    // outermost free cells still have neighbours to relax from.
    let mut lo = goal;
    let mut hi = goal;
    for &(a, b) in walls {
        for p in [a, b] {
            lo.x = lo.x.min(p.x);
            lo.y = lo.y.min(p.y);
            hi.x = hi.x.max(p.x);
            hi.y = hi.y.max(p.y);
        }
    }
    let margin = cell * 4.0;
    let origin = Vector2::new(lo.x - margin, lo.y - margin);
    let cols = (((hi.x + margin) - origin.x) / cell).ceil() as i32 + 1;
    let rows = (((hi.y + margin) - origin.y) / cell).ceil() as i32 + 1;
    let n = (cols * rows) as usize;

    // Free everywhere, then stamp each wall's radius-dilated neighbourhood
    // shut. Per-segment, so this is linear in wall length, not cells * walls.
    let clear = (radius - cell * SAMPLE_SLACK_CELLS).max(0.0);
    let mut free = vec![true; n];
    for &(a, b) in walls {
        let (x0, x1) = (a.x.min(b.x) - clear, a.x.max(b.x) + clear);
        let (y0, y1) = (a.y.min(b.y) - clear, a.y.max(b.y) + clear);
        let cx0 = (((x0 - origin.x) / cell).floor() as i32).max(0);
        let cx1 = (((x1 - origin.x) / cell).ceil() as i32).min(cols - 1);
        let cy0 = (((y0 - origin.y) / cell).floor() as i32).max(0);
        let cy1 = (((y1 - origin.y) / cell).ceil() as i32).min(rows - 1);
        for cy in cy0..=cy1 {
            for cx in cx0..=cx1 {
                let i = (cy * cols + cx) as usize;
                if !free[i] {
                    continue;
                }
                let p = Vector2::new(
                    origin.x + (cx as f32 + 0.5) * cell,
                    origin.y + (cy as f32 + 0.5) * cell,
                );
                if point_seg_dist(p, a, b) < clear {
                    free[i] = false;
                }
            }
        }
    }
    // Seal the border: off-map space has no walls, but must not be a shortcut
    // around the outside of a boundary wall.
    for cx in 0..cols {
        free[cx as usize] = false;
        free[((rows - 1) * cols + cx) as usize] = false;
    }
    for cy in 0..rows {
        free[(cy * cols) as usize] = false;
        free[(cy * cols + cols - 1) as usize] = false;
    }

    let mut f = Field {
        origin,
        cell,
        cols,
        rows,
        goal,
        dist: vec![f32::INFINITY; n],
    };

    // Seed from the goal, snapped onto free space exactly the way a query
    // endpoint is.
    let Some(seed) = f.nearest(goal, |i| free[i]) else {
        return f; // goal itself doesn't fit at this radius: nothing is reachable
    };

    f.dist[seed] = 0.0;
    let mut heap: BinaryHeap<Reverse<(u32, u32)>> = BinaryHeap::new();
    heap.push(Reverse((0u32, seed as u32)));
    while let Some(Reverse((dbits, ci))) = heap.pop() {
        let ci = ci as usize;
        // Non-negative f32 compares as its bit pattern: no wrapper needed.
        if f32::from_bits(dbits) > f.dist[ci] {
            continue;
        }
        let (cx, cy) = ((ci as i32) % cols, (ci as i32) / cols);
        for &(dx, dy, cost, over) in &STENCIL {
            let Some(ni) = f.idx(cx + dx, cy + dy) else {
                continue;
            };
            if !free[ni] {
                continue;
            }
            if over.iter().any(|&(ox, oy)| {
                (ox, oy) != (0, 0) && f.idx(cx + ox, cy + oy).is_none_or(|i| !free[i])
            }) {
                continue;
            }
            let nd = f.dist[ci] + cost * cell;
            if nd < f.dist[ni] {
                f.dist[ni] = nd;
                heap.push(Reverse((nd.to_bits(), ni as u32)));
            }
        }
    }
    f
}

fn point_seg_dist(p: Vector2, a: Vector2, b: Vector2) -> f32 {
    let ab = b - a;
    let len2 = ab.x * ab.x + ab.y * ab.y;
    if len2 < 1e-12 {
        return (p - a).length();
    }
    let t = (((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / len2).clamp(0.0, 1.0);
    (p - (a + ab * t)).length()
}

fn cache_key(walls: &[(Vector2, Vector2)], radius: f32, cell: f32, goal: Vector2) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |v: u32| {
        h ^= v as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    };
    for &(a, b) in walls {
        for p in [a, b] {
            mix(p.x.to_bits());
            mix(p.y.to_bits());
        }
    }
    mix(radius.to_bits());
    mix(cell.to_bits());
    mix(goal.x.to_bits());
    mix(goal.y.to_bits());
    h
}

fn load(path: &PathBuf) -> Option<Field> {
    let mut buf = Vec::new();
    std::fs::File::open(path).ok()?.read_to_end(&mut buf).ok()?;
    // 36 = MAGIC + FORMAT + origin + cell + cols + rows + goal; short means
    // truncated.
    if buf.len() < 36 || &buf[..4] != MAGIC {
        return None;
    }
    let u32_at = |o: usize| u32::from_le_bytes(buf[o..o + 4].try_into().unwrap());
    if u32_at(4) != FORMAT {
        return None;
    }
    let origin = Vector2::new(f32::from_bits(u32_at(8)), f32::from_bits(u32_at(12)));
    let cell = f32::from_bits(u32_at(16));
    let cols = u32_at(20) as i32;
    let rows = u32_at(24) as i32;
    let goal = Vector2::new(f32::from_bits(u32_at(28)), f32::from_bits(u32_at(32)));
    let n = (cols as usize).checked_mul(rows as usize)?;
    if buf.len() != 36 + n * 4 {
        return None;
    }
    let dist = buf[36..]
        .as_chunks::<4>()
        .0
        .iter()
        .map(|&c| f32::from_le_bytes(c))
        .collect();
    Some(Field {
        origin,
        cell,
        cols,
        rows,
        goal,
        dist,
    })
}

fn store(path: &PathBuf, f: &Field) -> std::io::Result<()> {
    let mut out = Vec::with_capacity(36 + f.dist.len() * 4);
    out.extend_from_slice(MAGIC);
    for v in [
        FORMAT,
        f.origin.x.to_bits(),
        f.origin.y.to_bits(),
        f.cell.to_bits(),
        f.cols as u32,
        f.rows as u32,
        f.goal.x.to_bits(),
        f.goal.y.to_bits(),
    ] {
        out.extend_from_slice(&v.to_le_bytes());
    }
    for d in &f.dist {
        out.extend_from_slice(&d.to_le_bytes());
    }
    // Rename in, so an interrupted write leaves no cache rather than a stub.
    let tmp = path.with_extension("tmp");
    std::fs::File::create(&tmp)?.write_all(&out)?;
    std::fs::rename(&tmp, path)
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn v(x: f32, y: f32) -> Vector2 {
        Vector2::new(x, y)
    }

    fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<(Vector2, Vector2)> {
        vec![
            (v(x0, y0), v(x1, y0)),
            (v(x1, y0), v(x1, y1)),
            (v(x1, y1), v(x0, y1)),
            (v(x0, y1), v(x0, y0)),
        ]
    }

    /// `compute` directly, so tests never touch the on-disk cache.
    fn field_of(walls: &[(Vector2, Vector2)], radius: f32, goal: Vector2) -> Field {
        compute(walls, radius, radius / 4.0, goal)
    }

    #[test]
    fn test_open_room_costs_the_straight_line() {
        let f = field_of(&rect(0.0, 0.0, 400.0, 400.0), 5.0, v(350.0, 350.0));
        let got = f.optimal_len(v(50.0, 50.0)).expect("reachable");
        let euclid = (v(350.0, 350.0) - v(50.0, 50.0)).length();
        // The module's stated budget: +1.3% relative plus `2 * cell * sqrt(2)`
        // absolute from snapping both endpoints (cell = `radius / 4` = 1.25).
        let snap = 2.0 * 1.25 * std::f32::consts::SQRT_2;
        assert!(
            got >= euclid - snap && got <= euclid * 1.013 + snap,
            "{got} vs euclid {euclid} (snap budget {snap})"
        );
    }

    #[test]
    fn test_detour_round_a_wall_is_longer_than_the_straight_line() {
        // Full-height wall with a gap at the bottom: the only route is under it.
        let mut walls = rect(0.0, 0.0, 400.0, 400.0);
        walls.push((v(200.0, 0.0), v(200.0, 340.0)));
        let f = field_of(&walls, 5.0, v(350.0, 200.0));
        let got = f.optimal_len(v(50.0, 200.0)).expect("reachable under the wall");
        assert!(got > 400.0, "must route round the wall, got {got}");
    }

    #[test]
    fn test_sealed_room_is_unreachable() {
        let mut walls = rect(0.0, 0.0, 400.0, 400.0);
        walls.extend(rect(160.0, 160.0, 240.0, 240.0));
        let f = field_of(&walls, 5.0, v(200.0, 200.0));
        assert_eq!(f.optimal_len(v(50.0, 50.0)), None);
        // …and from inside, the outside is equally unreachable.
        let g = field_of(&walls, 5.0, v(50.0, 50.0));
        assert_eq!(g.optimal_len(v(200.0, 200.0)), None);
    }

    #[test]
    fn test_reachability_follows_the_query_radius() {
        // 6-unit slot to the bottom boundary, as in `impassable_gap_40`.
        let mut walls = rect(0.0, 0.0, 200.0, 200.0);
        walls.extend(rect(150.0, 6.0, 153.0, 200.0));
        let goal = v(175.0, 100.0);
        assert!(
            field_of(&walls, 2.0, goal).optimal_len(v(120.0, 20.0)).is_some(),
            "diameter 4 fits a 6-unit slot"
        );
        assert!(
            field_of(&walls, 3.0, goal).optimal_len(v(120.0, 20.0)).is_some(),
            "diameter 6 is the exact fit and must not read as sealed"
        );
        assert_eq!(
            field_of(&walls, 5.0, goal).optimal_len(v(120.0, 20.0)),
            None,
            "diameter 10 cannot pass a 6-unit slot"
        );
    }

    #[test]
    fn test_never_leaks_around_the_outside_of_a_boundary_wall() {
        // Two rooms, no door. Without the sealed border the field would
        // happily route around the outside of the box.
        let mut walls = rect(0.0, 0.0, 400.0, 200.0);
        walls.push((v(200.0, 0.0), v(200.0, 200.0)));
        let f = field_of(&walls, 5.0, v(300.0, 100.0));
        assert_eq!(f.optimal_len(v(100.0, 100.0)), None);
    }

    #[test]
    fn test_cache_round_trips() {
        let dir = std::env::temp_dir().join("quality_ref_cache_test");
        let _ = std::fs::remove_dir_all(&dir);
        let walls = rect(0.0, 0.0, 200.0, 200.0);
        let goal = v(150.0, 150.0);
        let fresh = field(&walls, 5.0, 1.25, goal, &dir);
        let loaded = field(&walls, 5.0, 1.25, goal, &dir);
        assert_eq!(fresh.dist, loaded.dist, "a cached field must be identical");
        assert_eq!((fresh.cols, fresh.rows), (loaded.cols, loaded.rows));
        assert_eq!(fresh.goal, loaded.goal);
        // A different radius is a different key, not a cache hit.
        let other = field(&walls, 9.0, 2.25, goal, &dir);
        assert_ne!(other.dist.len(), 0);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
