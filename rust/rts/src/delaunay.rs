use godot::prelude::*;
use rts_lib::abstraction::Abstraction;
use rts_lib::astar::AStarScratch;
use rts_lib::delaunay::CDT;

#[derive(GodotClass)]
#[class(base=RefCounted)]
pub struct DelaunayTriangulator {
    points: PackedVector2Array,
    constraints: PackedInt32Array,
    cdt: Option<CDT>,
    abstraction: Option<Abstraction>,
    scratch: AStarScratch,
}

#[godot_api]
impl IRefCounted for DelaunayTriangulator {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            points: PackedVector2Array::new(),
            constraints: PackedInt32Array::new(),
            cdt: None,
            abstraction: None,
            scratch: AStarScratch::new(),
        }
    }
}

#[godot_api]
impl DelaunayTriangulator {
    #[func]
    pub fn set_points(&mut self, points: PackedVector2Array) {
        self.points = points;
        self.cdt = None;
        self.abstraction = None;
    }

    #[func]
    pub fn set_constraints(&mut self, constraints: PackedInt32Array) {
        self.constraints = constraints;
        self.cdt = None;
        self.abstraction = None;
    }

    #[func]
    pub fn triangulate(&mut self) {
        let point_vec: Vec<Vector2> = self.points.as_slice().to_vec();

        if point_vec.len() < 3 {
            godot_warn!("Need at least 3 points for triangulation");
            self.cdt = None;
            self.abstraction = None;
            return;
        }

        let mut cdt = CDT::from_points(point_vec);

        for pair in self.constraints.as_slice().chunks(2) {
            if pair.len() == 2 {
                cdt.insert_constraint(pair[0] as u32, pair[1] as u32);
            }
        }

        cdt.remove_super_triangle();
        cdt.build_grid_index();
        cdt.compute_widths();
        self.cdt = Some(cdt);
        self.abstraction = None;
    }

    /// Build the graph abstraction for TRA* pathfinding.
    ///
    /// Call this once after `triangulate()`.  The abstraction is invalidated
    /// whenever `triangulate()` is called again.
    #[func]
    pub fn build_abstraction(&mut self) {
        let Some(cdt) = &self.cdt else {
            godot_warn!("No triangulation result; call triangulate() first");
            return;
        };
        self.abstraction = Some(Abstraction::build(cdt));
    }

    #[func]
    pub fn get_mesh_vertices(&self) -> PackedVector2Array {
        match &self.cdt {
            Some(cdt) => cdt.get_mesh_vertices(),
            None => {
                godot_warn!("No triangulation result; call triangulate() first");
                PackedVector2Array::new()
            }
        }
    }

    #[func]
    pub fn get_indices(&self) -> Array<PackedInt32Array> {
        let Some(cdt) = &self.cdt else {
            godot_warn!("No triangulation result; call triangulate() first");
            return Array::new();
        };

        let mut result = Array::new();
        for f in 0..cdt.num_triangles() {
            let verts = cdt.face_vertices(f);
            let mut indices = PackedInt32Array::new();
            indices.push(verts[0] as i32);
            indices.push(verts[1] as i32);
            indices.push(verts[2] as i32);
            result.push(&indices);
        }
        result
    }

    #[func]
    pub fn get_triangle_count(&self) -> i32 {
        match &self.cdt {
            Some(cdt) => cdt.num_triangles() as i32,
            None => 0,
        }
    }

    /// Find a path for a circular agent of the given `radius`.
    ///
    /// Pass `radius = 0.0` to find the shortest path without any width
    /// constraint (equivalent to the old single-argument `find_path`).
    #[func]
    pub fn find_path(&mut self, start: Vector2, goal: Vector2, radius: f32) -> PackedVector2Array {
        let Some(cdt) = &self.cdt else {
            godot_warn!("No triangulation result; call triangulate() first");
            return PackedVector2Array::new();
        };
        let path = rts_lib::astar::find_path(cdt, start, goal, &mut self.scratch, radius);
        let mut result = PackedVector2Array::new();
        for pt in &path {
            result.push(*pt);
        }
        result
    }

    /// Find a path using TRA* (requires `build_abstraction()` to have been called).
    ///
    /// Falls back to the regular triangle A* when the abstraction is absent or
    /// when start/goal is in a region with no level-3 decision point.
    #[func]
    pub fn find_path_tra(
        &mut self,
        start: Vector2,
        goal: Vector2,
        radius: f32,
    ) -> PackedVector2Array {
        let Some(cdt) = &self.cdt else {
            godot_warn!("No triangulation result; call triangulate() first");
            return PackedVector2Array::new();
        };
        let path = match &self.abstraction {
            Some(abs) => {
                rts_lib::astar::find_path_abstract(cdt, abs, start, goal, &mut self.scratch, radius)
            }
            None => {
                godot_warn!(
                    "No abstraction; call build_abstraction() first (falling back to regular A*)"
                );
                rts_lib::astar::find_path(cdt, start, goal, &mut self.scratch, radius)
            }
        };
        let mut result = PackedVector2Array::new();
        for pt in &path {
            result.push(*pt);
        }
        result
    }
}
