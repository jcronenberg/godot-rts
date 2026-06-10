use godot::prelude::*;
use rts_lib::abstraction::Abstraction;
use rts_lib::astar::AStarScratch;
use rts_lib::delaunay::CDT;
use rts_lib::navmesh::{DynamicNavmesh, Obstacle, ObstacleId};

#[derive(GodotClass)]
#[class(base=RefCounted)]
pub struct DelaunayTriangulator {
    points: PackedVector2Array,
    constraints: PackedInt32Array,
    navmesh: Option<DynamicNavmesh>,
    /// Abstraction plus the navmesh version it was built for; rebuilt lazily
    /// when the navmesh version moves (e.g. after an obstacle change).
    abstraction: Option<(u64, Abstraction)>,
    scratch: AStarScratch,
}

#[godot_api]
impl IRefCounted for DelaunayTriangulator {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            points: PackedVector2Array::new(),
            constraints: PackedInt32Array::new(),
            navmesh: None,
            abstraction: None,
            scratch: AStarScratch::new(),
        }
    }
}

impl DelaunayTriangulator {
    /// Current navmesh, rebuilt first if the obstacle set changed.
    fn cdt_mut(&mut self) -> Option<&CDT> {
        match &mut self.navmesh {
            Some(nav) => Some(nav.rebuild()),
            None => {
                godot_warn!("No triangulation result; call triangulate() first");
                None
            }
        }
    }
}

#[godot_api]
impl DelaunayTriangulator {
    #[func]
    pub fn set_points(&mut self, points: PackedVector2Array) {
        self.points = points;
        self.navmesh = None;
        self.abstraction = None;
    }

    #[func]
    pub fn set_constraints(&mut self, constraints: PackedInt32Array) {
        self.constraints = constraints;
        self.navmesh = None;
        self.abstraction = None;
    }

    #[func]
    pub fn triangulate(&mut self) {
        let point_vec: Vec<Vector2> = self.points.as_slice().to_vec();

        if point_vec.len() < 3 {
            godot_warn!("Need at least 3 points for triangulation");
            self.navmesh = None;
            self.abstraction = None;
            return;
        }

        let constraints: Vec<(u32, u32)> = self
            .constraints
            .as_slice()
            .chunks_exact(2)
            .map(|pair| (pair[0] as u32, pair[1] as u32))
            .collect();

        self.navmesh = Some(DynamicNavmesh::new(point_vec, &constraints));
        self.abstraction = None;
    }

    /// Add an obstacle as a closed polygon; returns its id (-1 on error).
    /// Takes effect on the next rebuild (lazy on queries, or explicit).
    #[func]
    pub fn add_obstacle(&mut self, points: PackedVector2Array) -> i64 {
        let Some(nav) = &mut self.navmesh else {
            godot_warn!("No triangulation result; call triangulate() first");
            return -1;
        };
        let obs = Obstacle::polygon(points.as_slice().to_vec());
        nav.add_obstacle(obs).raw() as i64
    }

    /// Remove an obstacle by the id `add_obstacle` returned. Unknown ids are ignored.
    #[func]
    pub fn remove_obstacle(&mut self, id: i64) {
        let Some(nav) = &mut self.navmesh else {
            godot_warn!("No triangulation result; call triangulate() first");
            return;
        };
        nav.remove_obstacle(ObstacleId::from_raw(id as u64));
    }

    /// Rebuild the navmesh now if the obstacle set changed (no-op when clean).
    /// Queries also rebuild lazily, so calling this is optional.
    #[func]
    pub fn rebuild(&mut self) {
        if let Some(nav) = &mut self.navmesh {
            nav.rebuild();
        }
    }

    /// Build the graph abstraction for TRA* pathfinding.
    ///
    /// Call this once after `triangulate()`. It is re-built automatically when
    /// the navmesh changes (obstacle add/remove or re-triangulation).
    #[func]
    pub fn build_abstraction(&mut self) {
        let Some(cdt) = self.cdt_mut() else {
            return;
        };
        let version = cdt.version();
        let abs = Abstraction::build(cdt);
        self.abstraction = Some((version, abs));
    }

    #[func]
    pub fn get_mesh_vertices(&mut self) -> PackedVector2Array {
        match self.cdt_mut() {
            Some(cdt) => cdt.get_mesh_vertices(),
            None => PackedVector2Array::new(),
        }
    }

    /// All vertex positions in the triangulation, including inserted obstacle vertices.
    #[func]
    pub fn get_points(&mut self) -> PackedVector2Array {
        match self.cdt_mut() {
            Some(cdt) => PackedVector2Array::from(cdt.points()),
            None => PackedVector2Array::new(),
        }
    }

    /// Constrained edges as endpoint pairs (2 vertices per edge) for rendering.
    #[func]
    pub fn get_constrained_edges(&mut self) -> PackedVector2Array {
        match self.cdt_mut() {
            Some(cdt) => cdt.get_constrained_edge_vertices(),
            None => PackedVector2Array::new(),
        }
    }

    #[func]
    pub fn get_indices(&mut self) -> Array<PackedInt32Array> {
        let Some(cdt) = self.cdt_mut() else {
            return Array::new();
        };

        let mut result = Array::new();
        for f in 0..cdt.num_faces() {
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
    pub fn get_triangle_count(&mut self) -> i32 {
        match self.cdt_mut() {
            Some(cdt) => cdt.num_faces() as i32,
            None => 0,
        }
    }

    /// Find a path for a circular agent of the given `radius`.
    ///
    /// Pass `radius = 0.0` to find the shortest path without any width
    /// constraint (equivalent to the old single-argument `find_path`).
    #[func]
    pub fn find_path(&mut self, start: Vector2, goal: Vector2, radius: f32) -> PackedVector2Array {
        let Some(nav) = &mut self.navmesh else {
            godot_warn!("No triangulation result; call triangulate() first");
            return PackedVector2Array::new();
        };
        let cdt = nav.rebuild();
        let path = rts_lib::astar::find_path(cdt, start, goal, &mut self.scratch, radius);
        PackedVector2Array::from(path.as_slice())
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
        let Some(nav) = &mut self.navmesh else {
            godot_warn!("No triangulation result; call triangulate() first");
            return PackedVector2Array::new();
        };
        let cdt = nav.rebuild();

        // Refresh a stale abstraction (navmesh changed since it was built).
        if let Some((version, _)) = &self.abstraction
            && *version != cdt.version()
        {
            let v = cdt.version();
            self.abstraction = Some((v, Abstraction::build(cdt)));
        }

        let cdt = self.navmesh.as_ref().unwrap().navmesh();
        let path = match &self.abstraction {
            Some((_, abs)) => {
                rts_lib::astar::find_path_abstract(cdt, abs, start, goal, &mut self.scratch, radius)
            }
            None => {
                godot_warn!(
                    "No abstraction; call build_abstraction() first (falling back to regular A*)"
                );
                rts_lib::astar::find_path(cdt, start, goal, &mut self.scratch, radius)
            }
        };
        PackedVector2Array::from(path.as_slice())
    }
}
