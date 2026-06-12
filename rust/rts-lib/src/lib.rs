pub mod abstraction;
pub mod astar;
pub mod delaunay;
pub mod mapgen;
pub mod navmesh;
pub mod report;
pub mod sim;
pub mod sim_runner;

/// Per-thread allocation counter used by tests to verify that pooled-scratch
/// code paths are allocation-free in steady state. Counts only the current
/// thread's allocations (via a `const`-initialised thread-local, so it never
/// itself allocates), making it immune to parallel-test interference.
#[cfg(test)]
pub(crate) mod alloc_counter {
    use std::alloc::{GlobalAlloc, Layout, System};
    use std::cell::Cell;

    thread_local! {
        static COUNT: Cell<u64> = const { Cell::new(0) };
        static ARMED: Cell<bool> = const { Cell::new(false) };
    }

    pub struct CountingAlloc;

    unsafe impl GlobalAlloc for CountingAlloc {
        unsafe fn alloc(&self, l: Layout) -> *mut u8 {
            if ARMED.with(Cell::get) {
                COUNT.with(|c| c.set(c.get() + 1));
            }
            unsafe { System.alloc(l) }
        }
        unsafe fn dealloc(&self, p: *mut u8, l: Layout) {
            unsafe { System.dealloc(p, l) }
        }
        unsafe fn realloc(&self, p: *mut u8, l: Layout, new_size: usize) -> *mut u8 {
            if ARMED.with(Cell::get) {
                COUNT.with(|c| c.set(c.get() + 1));
            }
            unsafe { System.realloc(p, l, new_size) }
        }
    }

    /// Run `f`, returning the number of allocations it made on this thread.
    pub fn count_allocs<F: FnOnce()>(f: F) -> u64 {
        COUNT.with(|c| c.set(0));
        ARMED.with(|a| a.set(true));
        f();
        ARMED.with(|a| a.set(false));
        COUNT.with(Cell::get)
    }
}

#[cfg(test)]
#[global_allocator]
static COUNTING_ALLOCATOR: alloc_counter::CountingAlloc = alloc_counter::CountingAlloc;

#[cfg(test)]
pub(crate) mod test_utils {
    use crate::delaunay::CDT;
    use godot::prelude::Vector2;

    #[derive(serde::Deserialize)]
    struct MapData {
        points: Vec<[f32; 2]>,
        constraints: Vec<u32>,
    }

    pub(crate) fn load_raw(name: &str) -> (Vec<Vector2>, Vec<(u32, u32)>) {
        let path = format!("{}/../test_maps/{}.json", env!("CARGO_MANIFEST_DIR"), name);
        let content =
            std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
        let data: MapData =
            serde_json::from_str(&content).unwrap_or_else(|e| panic!("cannot parse {path}: {e}"));
        let points = data
            .points
            .iter()
            .map(|[x, y]| Vector2::new(*x, *y))
            .collect();
        let chunks = data.constraints.chunks_exact(2);
        assert!(
            chunks.remainder().is_empty(),
            "constraints in {path} has odd length {}",
            data.constraints.len()
        );
        let constraints = chunks.map(|c| (c[0], c[1])).collect();
        (points, constraints)
    }

    pub(crate) fn build_cdt(name: &str) -> CDT {
        let (points, constraints) = load_raw(name);
        let mut cdt = CDT::from_points(points);
        for (a, b) in constraints {
            cdt.insert_constraint(a, b);
        }
        cdt.remove_super_triangle();
        cdt.build_grid_index();
        cdt.compute_widths();
        cdt
    }

    // --- Triangulation comparison helpers (oracle/equivalence tests) ---

    use std::collections::BTreeSet;

    pub(crate) type VKey = (u32, u32);

    pub(crate) fn vkey(p: Vector2) -> VKey {
        (p.x.to_bits(), p.y.to_bits())
    }

    pub(crate) fn ekey(a: Vector2, b: Vector2) -> (VKey, VKey) {
        let (ka, kb) = (vkey(a), vkey(b));
        if ka <= kb { (ka, kb) } else { (kb, ka) }
    }

    pub(crate) fn vertex_set(cdt: &CDT) -> BTreeSet<VKey> {
        cdt.points().iter().map(|&p| vkey(p)).collect()
    }

    pub(crate) fn constrained_edge_set(cdt: &CDT) -> BTreeSet<(VKey, VKey)> {
        let mut set = BTreeSet::new();
        for he in 0..cdt.num_faces() * 3 {
            if cdt.he_is_constrained(he) {
                let a = cdt.points()[cdt.he_origin(he) as usize];
                let b = cdt.points()[cdt.he_dest(he) as usize];
                set.insert(ekey(a, b));
            }
        }
        set
    }

    /// No NaN points and a structurally valid DCEL.
    pub(crate) fn assert_valid(cdt: &CDT) {
        for p in cdt.points() {
            assert!(p.x.is_finite() && p.y.is_finite(), "non-finite point {p:?}");
        }
        #[cfg(debug_assertions)]
        cdt.assert_dcel_valid();
    }

    /// Every unconstrained interior edge passes the in-circumcircle test
    /// (f64 predicate with a relative tolerance, so f32 round-off and
    /// cocircular degeneracies don't trip it).
    pub(crate) fn assert_local_delaunay(cdt: &CDT) {
        let pt = |v: u32| cdt.points()[v as usize];
        for f in 0..cdt.num_faces() {
            cdt.for_each_neighbor(f, |nb, he| {
                let (a, b) = (cdt.he_origin(he), cdt.he_dest(he));
                let d = cdt
                    .face_vertices(nb)
                    .into_iter()
                    .find(|&v| v != a && v != b)
                    .unwrap();
                let [v0, v1, v2] = cdt.face_vertices(f);
                assert!(
                    !in_circle_strict_f64(pt(v0), pt(v1), pt(v2), pt(d)),
                    "edge {a}-{b} of face {f} violates Delaunay (opposite vertex {d})"
                );
            });
        }
    }

    fn in_circle_strict_f64(a: Vector2, b: Vector2, c: Vector2, p: Vector2) -> bool {
        let (ax, ay) = ((a.x - p.x) as f64, (a.y - p.y) as f64);
        let (bx, by) = ((b.x - p.x) as f64, (b.y - p.y) as f64);
        let (cx, cy) = ((c.x - p.x) as f64, (c.y - p.y) as f64);
        let det = (ax * ax + ay * ay) * (bx * cy - cx * by)
            - (bx * bx + by * by) * (ax * cy - cx * ay)
            + (cx * cx + cy * cy) * (ax * by - bx * ay);
        let orient = ax * (by - cy) - ay * (bx - cx) + (bx * cy - cx * by);
        let det = if orient < 0.0 { -det } else { det };
        let scale = [ax, ay, bx, by, cx, cy]
            .iter()
            .fold(1.0f64, |m, v| m.max(v.abs()));
        det > 1e-7 * scale.powi(4)
    }

    /// Structural equivalence for triangulations of the same input: identical
    /// vertex and constrained-edge sets (by coordinates), same face count,
    /// valid DCEL, and the local Delaunay property. Triangle sets may differ
    /// in cocircular cases, so they are deliberately not compared.
    pub(crate) fn assert_navmesh_equiv(a: &CDT, b: &CDT) {
        assert_valid(a);
        assert_valid(b);
        assert_eq!(vertex_set(a), vertex_set(b), "vertex sets differ");
        assert_eq!(
            constrained_edge_set(a),
            constrained_edge_set(b),
            "constrained edge sets differ"
        );
        assert_eq!(a.num_faces(), b.num_faces(), "face counts differ");
        assert_local_delaunay(a);
        assert_local_delaunay(b);
    }
}
