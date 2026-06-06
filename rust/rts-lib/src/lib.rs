pub mod abstraction;
pub mod astar;
pub mod delaunay;

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
}
