//! Declarative dynamic-obstacle navmesh: `navmesh = f(base, obstacle set)`.
//!
//! The base CDT (static map geometry) keeps its super-triangle so obstacle
//! insertion reuses the exact construction code path. Obstacle add/remove
//! marks the set dirty; `rebuild` reconstructs the active navmesh from the
//! base at most once per batch of changes. Removal is "not re-inserting",
//! so no constraint/vertex removal is ever needed in the CDT.

use std::collections::BTreeMap;

use godot::prelude::*;

use crate::delaunay::CDT;

/// A set of constraint edges over its own point list (`edges` hold local
/// indices into `points`). Obstacle edges must not cross other constraint
/// edges (map walls or other obstacles); touching at vertices or lying on
/// existing edges is allowed.
#[derive(Debug, Clone)]
pub struct Obstacle {
    pub points: Vec<Vector2>,
    pub edges: Vec<(u32, u32)>,
}

impl Obstacle {
    /// Closed ring over `points` (each point connects to the next, last to first).
    pub fn polygon(points: Vec<Vector2>) -> Obstacle {
        let n = points.len() as u32;
        let edges = if n >= 2 {
            (0..n).map(|i| (i, (i + 1) % n)).collect()
        } else {
            Vec::new()
        };
        Obstacle { points, edges }
    }
}

/// Stable handle for a live obstacle. Ids are never reused.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObstacleId(u64);

impl ObstacleId {
    /// Raw id for FFI layers (e.g. GDScript); pairs with [`ObstacleId::from_raw`].
    pub fn raw(self) -> u64 {
        self.0
    }

    pub fn from_raw(raw: u64) -> ObstacleId {
        ObstacleId(raw)
    }
}

/// Immutable base CDT plus a mutable obstacle set, rebuilt on demand.
pub struct DynamicNavmesh {
    /// Static map geometry with constraints inserted, super-triangle kept so
    /// obstacle points never hit boundary-edge cases during insertion.
    base: CDT,
    /// Live obstacles in id order — deterministic rebuild for lockstep sim.
    obstacles: BTreeMap<u64, Obstacle>,
    next_id: u64,
    navmesh: CDT,
    dirty: bool,
}

impl DynamicNavmesh {
    /// Build the base from static map geometry (`constraints` index into `points`).
    pub fn new(points: Vec<Vector2>, constraints: &[(u32, u32)]) -> Self {
        let mut base = CDT::from_points(points);
        for &(a, b) in constraints {
            base.insert_constraint(a, b);
        }
        // One-time width precomputation; rebuilds reuse it for every portal
        // the obstacle set doesn't touch (compute_widths_from).
        base.compute_widths();
        let navmesh = build_active(&base, std::iter::empty());
        DynamicNavmesh {
            base,
            obstacles: BTreeMap::new(),
            next_id: 0,
            navmesh,
            dirty: false,
        }
    }

    /// Add an obstacle; takes effect on the next `rebuild`.
    pub fn add_obstacle(&mut self, obs: Obstacle) -> ObstacleId {
        let id = self.next_id;
        self.next_id += 1;
        self.obstacles.insert(id, obs);
        self.dirty = true;
        ObstacleId(id)
    }

    /// Remove an obstacle; takes effect on the next `rebuild`. Unknown ids are ignored.
    pub fn remove_obstacle(&mut self, id: ObstacleId) {
        if self.obstacles.remove(&id.0).is_some() {
            self.dirty = true;
        }
    }

    /// Rebuild the active navmesh if the obstacle set changed; returns it.
    pub fn rebuild(&mut self) -> &CDT {
        if self.dirty {
            self.navmesh = build_active(&self.base, self.obstacles.values());
            self.dirty = false;
        }
        &self.navmesh
    }

    /// Current navmesh without rebuilding (may be stale if `is_dirty`).
    pub fn navmesh(&self) -> &CDT {
        &self.navmesh
    }

    pub fn is_dirty(&self) -> bool {
        self.dirty
    }

    pub fn num_obstacles(&self) -> usize {
        self.obstacles.len()
    }
}

/// Clone the base, insert all obstacle vertices then all obstacle constraints,
/// and finish the navmesh (super-triangle removal, grid index, portal widths).
fn build_active<'a>(base: &CDT, obstacles: impl Iterator<Item = &'a Obstacle>) -> CDT {
    let mut cdt = base.clone();

    // Insert points first so constraint insertion sees every obstacle vertex.
    // Chain locate hints: obstacle vertices are spatially local, so the
    // walking locator stays cheap.
    let mut hint = 0u32;
    let mut all_ids: Vec<(Vec<u32>, &Obstacle)> = Vec::new();
    for obs in obstacles {
        let mut ids = Vec::with_capacity(obs.points.len());
        for &p in &obs.points {
            let v = cdt.insert_point(p, hint);
            hint = cdt.face_of_vertex(v);
            ids.push(v);
        }
        all_ids.push((ids, obs));
    }

    for (ids, obs) in &all_ids {
        for &(a, b) in &obs.edges {
            cdt.insert_constraint(ids[a as usize], ids[b as usize]);
        }
    }

    // Widths before super removal so half-edge slots still align with `base`;
    // removal then carries the table through its compaction.
    cdt.compute_widths_from(base);
    cdt.remove_super_triangle();
    cdt.build_grid_index();
    cdt
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::astar::{AStarScratch, find_path};
    use crate::mapgen::rooms_map;
    use crate::test_utils::{
        assert_local_delaunay, assert_navmesh_equiv, assert_valid, constrained_edge_set, load_raw,
        vertex_set, vkey,
    };

    fn v(x: f32, y: f32) -> Vector2 {
        Vector2::new(x, y)
    }

    fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> Obstacle {
        Obstacle::polygon(vec![v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y1)])
    }

    /// Incrementally-reused portal widths must be bitwise identical to a full
    /// recompute on the same mesh.
    fn assert_widths_match_full(cdt: &CDT) {
        let mut full = cdt.clone();
        full.compute_widths();
        for he in 0..cdt.num_faces() * 3 {
            assert_eq!(
                cdt.portal_radius(he).to_bits(),
                full.portal_radius(he).to_bits(),
                "incremental width diverged at he {he}"
            );
        }
    }

    /// From-scratch CDT over map + obstacle geometry in one batch.
    /// Only valid when obstacle points don't coincide with map points.
    fn oracle(points: &[Vector2], constraints: &[(u32, u32)], obstacles: &[&Obstacle]) -> CDT {
        let mut pts = points.to_vec();
        let mut cons = constraints.to_vec();
        for obs in obstacles {
            let base = pts.len() as u32;
            pts.extend_from_slice(&obs.points);
            cons.extend(obs.edges.iter().map(|&(a, b)| (base + a, base + b)));
        }
        let mut cdt = CDT::from_points(pts);
        for (a, b) in cons {
            cdt.insert_constraint(a, b);
        }
        cdt.remove_super_triangle();
        cdt.build_grid_index();
        cdt.compute_widths();
        cdt
    }

    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> u64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            self.0 >> 33
        }
        fn range(&mut self, lo: f32, hi: f32) -> f32 {
            lo + (hi - lo) * (self.next() % 10001) as f32 / 10000.0
        }
    }

    /// Random rect inside the interior of room (i, j), clear of the walls.
    fn room_rect(rng: &mut Lcg, i: usize, j: usize) -> Obstacle {
        let (rx, ry) = (i as f32 * 100.0, j as f32 * 100.0);
        let x0 = rng.range(rx + 10.0, rx + 60.0);
        let y0 = rng.range(ry + 10.0, ry + 60.0);
        let w = rng.range(5.0, 28.0);
        let h = rng.range(5.0, 28.0);
        rect(x0, y0, x0 + w, y0 + h)
    }

    #[test]
    fn test_oracle_rooms_rect_obstacles() {
        let (points, constraints) = rooms_map(4, 4);
        let mut rng = Lcg(42);
        let obstacles: Vec<Obstacle> = (0..4)
            .flat_map(|i| (0..4).map(move |j| (i, j)))
            .map(|(i, j)| room_rect(&mut rng, i, j))
            .collect();

        let mut nav = DynamicNavmesh::new(points.clone(), &constraints);
        for obs in &obstacles {
            nav.add_obstacle(obs.clone());
        }
        let got = nav.rebuild();
        let want = oracle(&points, &constraints, &obstacles.iter().collect::<Vec<_>>());
        assert_navmesh_equiv(got, &want);
    }

    #[test]
    fn test_oracle_random_map() {
        let points = crate::mapgen::random_points(80, 0xFEED);
        let mut obstacles = Vec::new();
        let mut rng = Lcg(7);
        for k in 0..5 {
            let cx = 250.0 + 100.0 * k as f32 + rng.range(0.0, 30.0);
            let cy = rng.range(300.0, 700.0);
            obstacles.push(Obstacle::polygon(vec![
                v(cx, cy),
                v(cx + 20.5, cy + 5.5),
                v(cx + 8.5, cy + 22.5),
            ]));
        }

        let mut nav = DynamicNavmesh::new(points.clone(), &[]);
        for obs in &obstacles {
            nav.add_obstacle(obs.clone());
        }
        let got = nav.rebuild();
        let want = oracle(&points, &[], &obstacles.iter().collect::<Vec<_>>());
        assert_navmesh_equiv(got, &want);
    }

    #[test]
    fn test_remove_all_returns_to_base() {
        let (points, constraints) = rooms_map(3, 3);
        let mut nav = DynamicNavmesh::new(points.clone(), &constraints);
        let mut rng = Lcg(99);
        let ids: Vec<ObstacleId> = (0..3)
            .map(|i| nav.add_obstacle(room_rect(&mut rng, i, i)))
            .collect();
        nav.rebuild();
        for id in ids {
            nav.remove_obstacle(id);
        }
        nav.rebuild();

        let fresh = DynamicNavmesh::new(points, &constraints);
        assert_navmesh_equiv(nav.navmesh(), fresh.navmesh());
    }

    #[test]
    fn test_rebuild_idempotent() {
        let (points, constraints) = rooms_map(2, 2);
        let mut rng = Lcg(5);
        let obstacles: Vec<Obstacle> = (0..2).map(|i| room_rect(&mut rng, i, 1)).collect();

        let mut nav1 = DynamicNavmesh::new(points.clone(), &constraints);
        let mut nav2 = DynamicNavmesh::new(points, &constraints);
        for obs in &obstacles {
            nav1.add_obstacle(obs.clone());
            nav2.add_obstacle(obs.clone());
        }
        nav1.rebuild();
        // Clean rebuild is a no-op: same geometry token.
        let version = nav1.navmesh().version();
        nav1.rebuild();
        assert_eq!(nav1.navmesh().version(), version);

        nav2.rebuild();
        assert_navmesh_equiv(nav1.navmesh(), nav2.navmesh());
    }

    #[test]
    fn test_fuzz_add_remove_rects() {
        let (points, constraints) = rooms_map(4, 4);
        let mut nav = DynamicNavmesh::new(points, &constraints);
        let map_edges = constrained_edge_set(nav.navmesh());

        let mut rng = Lcg(0xD1CE);
        let mut slots: Vec<Option<(ObstacleId, Obstacle)>> = vec![None; 16];
        for iter in 0..300 {
            let r = (rng.next() % 16) as usize;
            match slots[r].take() {
                Some((id, _)) => nav.remove_obstacle(id),
                None => {
                    let obs = room_rect(&mut rng, r % 4, r / 4);
                    let id = nav.add_obstacle(obs.clone());
                    slots[r] = Some((id, obs));
                }
            }
            if iter % 10 != 9 {
                continue;
            }
            let cdt = nav.rebuild();
            assert_valid(cdt);
            assert_local_delaunay(cdt);
            assert_widths_match_full(cdt);
            let edges = constrained_edge_set(cdt);
            assert!(
                edges.is_superset(&map_edges),
                "map wall missing after iter {iter}"
            );
            for (_, obs) in slots.iter().flatten() {
                for &(a, b) in &obs.edges {
                    let (ka, kb) = (vkey(obs.points[a as usize]), vkey(obs.points[b as usize]));
                    let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
                    assert!(edges.contains(&key), "obstacle edge missing at iter {iter}");
                }
            }
        }
    }

    #[test]
    fn test_incremental_widths_match_full() {
        // Plain insertion plus the wall-set-changing paths: a corner reusing a
        // map vertex and a vertex splitting a constrained edge.
        let (points, constraints) = rooms_map(2, 2);
        let mut nav = DynamicNavmesh::new(points, &constraints);
        assert_widths_match_full(nav.navmesh());
        let id = nav.add_obstacle(rect(20.0, 20.0, 40.0, 40.0));
        nav.add_obstacle(Obstacle::polygon(vec![
            v(100.0, 100.0),
            v(70.0, 80.0),
            v(80.0, 70.0),
        ]));
        assert_widths_match_full(nav.rebuild());
        nav.remove_obstacle(id);
        assert_widths_match_full(nav.rebuild());

        // (100, 20) lies on the wall segment (100,0)-(100,35).
        let (points, constraints) = rooms_map(2, 1);
        let mut nav = DynamicNavmesh::new(points, &constraints);
        nav.add_obstacle(Obstacle::polygon(vec![
            v(100.0, 20.0),
            v(70.0, 10.0),
            v(70.0, 30.0),
        ]));
        assert_widths_match_full(nav.rebuild());
    }

    #[test]
    fn test_obstacle_vertex_on_map_vertex() {
        let (points, constraints) = rooms_map(2, 2);
        let n_map = points.len() as u32;
        let mut nav = DynamicNavmesh::new(points, &constraints);
        // (100, 100) is the interior wall junction — must dedupe, not duplicate.
        nav.add_obstacle(Obstacle::polygon(vec![
            v(100.0, 100.0),
            v(70.0, 80.0),
            v(80.0, 70.0),
        ]));
        let cdt = nav.rebuild();
        assert_valid(cdt);
        assert_eq!(
            cdt.num_vertices(),
            n_map + 2,
            "shared vertex must be reused"
        );
        assert_local_delaunay(cdt);
    }

    #[test]
    fn test_obstacle_vertex_on_constrained_edge_splits_flags() {
        let (points, constraints) = rooms_map(2, 1);
        let mut nav = DynamicNavmesh::new(points, &constraints);
        // (100, 20) lies on the wall segment (100,0)-(100,35).
        nav.add_obstacle(Obstacle::polygon(vec![
            v(100.0, 20.0),
            v(70.0, 10.0),
            v(70.0, 30.0),
        ]));
        let cdt = nav.rebuild();
        assert_valid(cdt);
        let edges = constrained_edge_set(cdt);
        for (a, b) in [
            (v(100.0, 0.0), v(100.0, 20.0)),
            (v(100.0, 20.0), v(100.0, 35.0)),
        ] {
            let (ka, kb) = (vkey(a), vkey(b));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            assert!(edges.contains(&key), "split wall half {a:?}-{b:?} missing");
        }
    }

    #[test]
    fn test_obstacle_vertex_on_unconstrained_edge() {
        let (points, _) = load_raw("square");
        let mut nav = DynamicNavmesh::new(points, &[]);
        // (0.5, 0.5) lies on the square's diagonal, whichever way it points.
        nav.add_obstacle(Obstacle::polygon(vec![
            v(0.5, 0.5),
            v(0.75, 0.55),
            v(0.6, 0.8),
        ]));
        let cdt = nav.rebuild();
        assert_valid(cdt);
        assert!(vertex_set(cdt).contains(&vkey(v(0.5, 0.5))));
        assert_local_delaunay(cdt);
    }

    #[test]
    fn test_obstacle_edge_collinear_adjacent_to_constraint() {
        let points = vec![
            v(0.0, 0.0),
            v(1.0, 0.0),
            v(2.0, 0.0),
            v(2.0, 1.0),
            v(0.0, 1.0),
        ];
        let mut nav = DynamicNavmesh::new(points, &[(0, 1)]);
        // Collinear continuation of constraint (0,0)-(1,0), sharing (1,0);
        // (1.5, 0) lands on the unconstrained hull edge (1,0)-(2,0).
        nav.add_obstacle(Obstacle {
            points: vec![v(1.0, 0.0), v(1.5, 0.0)],
            edges: vec![(0, 1)],
        });
        let cdt = nav.rebuild();
        assert_valid(cdt);
        let edges = constrained_edge_set(cdt);
        for (a, b) in [(v(0.0, 0.0), v(1.0, 0.0)), (v(1.0, 0.0), v(1.5, 0.0))] {
            let (ka, kb) = (vkey(a), vkey(b));
            let key = if ka <= kb { (ka, kb) } else { (kb, ka) };
            assert!(edges.contains(&key), "edge {a:?}-{b:?} missing");
        }
    }

    #[test]
    fn test_degenerate_obstacles() {
        let (points, _) = load_raw("square");
        let mut nav = DynamicNavmesh::new(points, &[]);
        // All points coincide — zero area, duplicate consecutive points.
        nav.add_obstacle(Obstacle::polygon(vec![
            v(0.5, 0.5),
            v(0.5, 0.5),
            v(0.5, 0.5),
        ]));
        // Duplicate consecutive point in an otherwise valid triangle.
        nav.add_obstacle(Obstacle::polygon(vec![
            v(0.2, 0.2),
            v(0.2, 0.2),
            v(0.4, 0.2),
            v(0.3, 0.4),
        ]));
        let cdt = nav.rebuild();
        assert_valid(cdt);
        assert_local_delaunay(cdt);
    }

    #[test]
    fn test_obstacle_touching_hull_boundary() {
        let (points, _) = load_raw("square");
        let mut nav = DynamicNavmesh::new(points, &[]);
        // One corner on the hull vertex (0,0), one on the hull edge y=0.
        nav.add_obstacle(Obstacle::polygon(vec![
            v(0.0, 0.0),
            v(0.5, 0.0),
            v(0.2, 0.3),
        ]));
        let cdt = nav.rebuild();
        assert_valid(cdt);
        assert!(vertex_set(cdt).contains(&vkey(v(0.5, 0.0))));
        assert_local_delaunay(cdt);
    }

    #[test]
    fn test_obstacle_on_large_coordinates_map() {
        let (points, _) = load_raw("large_coordinates");
        let mut nav = DynamicNavmesh::new(points, &[]);
        nav.add_obstacle(Obstacle::polygon(vec![
            v(820.0, 850.0),
            v(832.0, 851.0),
            v(826.0, 861.0),
        ]));
        let cdt = nav.rebuild();
        assert_valid(cdt);
        assert_eq!(cdt.num_vertices(), 6);
        assert_local_delaunay(cdt);
    }

    fn path_len(path: &[Vector2]) -> f32 {
        path.windows(2)
            .map(|w| ((w[1].x - w[0].x).powi(2) + (w[1].y - w[0].y).powi(2)).sqrt())
            .sum()
    }

    #[test]
    fn test_path_detours_around_obstacle_and_restores() {
        let (points, constraints) = rooms_map(2, 1);
        let mut nav = DynamicNavmesh::new(points, &constraints);
        let mut scratch = AStarScratch::new();
        let (start, goal) = (v(50.0, 50.0), v(150.0, 50.0));

        let base = find_path(nav.rebuild(), start, goal, &mut scratch, 0.0);
        assert!(!base.is_empty());
        let base_len = path_len(&base);

        // Rect in the doorway (door gap is x=100, y in [35,65]).
        let id = nav.add_obstacle(rect(98.0, 40.0, 102.0, 60.0));
        let blocked = find_path(nav.rebuild(), start, goal, &mut scratch, 0.0);
        assert!(!blocked.is_empty(), "narrowed door must still be passable");
        assert!(
            path_len(&blocked) > base_len + 1.0,
            "path must detour around the obstacle"
        );

        nav.remove_obstacle(id);
        let restored = find_path(nav.rebuild(), start, goal, &mut scratch, 0.0);
        assert!(
            (path_len(&restored) - base_len).abs() < 1e-3,
            "removing the obstacle must restore the original path"
        );
    }

    #[test]
    fn test_obstacle_narrows_corridor_below_agent_radius() {
        let (points, constraints) = rooms_map(2, 1);
        let mut nav = DynamicNavmesh::new(points, &constraints);
        let mut scratch = AStarScratch::new();
        let (start, goal) = (v(50.0, 50.0), v(150.0, 50.0));
        let radius = 8.0;

        let before = find_path(nav.rebuild(), start, goal, &mut scratch, radius);
        assert!(!before.is_empty(), "radius 8 fits through the 30-wide door");

        // Leaves ≤5-wide gaps on either side of the doorway — too tight for r=8.
        nav.add_obstacle(rect(98.0, 40.0, 102.0, 60.0));
        let after = find_path(nav.rebuild(), start, goal, &mut scratch, radius);
        assert!(after.is_empty(), "radius 8 must no longer fit");
    }
}
