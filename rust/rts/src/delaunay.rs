use godot::prelude::*;
use rts_lib::delaunay::CDT;

#[derive(GodotClass)]
#[class(base=RefCounted)]
pub struct DelaunayTriangulator {
    points: PackedVector2Array,
    constraints: PackedInt32Array,
    cdt: Option<CDT>,
}

#[godot_api]
impl IRefCounted for DelaunayTriangulator {
    fn init(_base: Base<RefCounted>) -> Self {
        Self {
            points: PackedVector2Array::new(),
            constraints: PackedInt32Array::new(),
            cdt: None,
        }
    }
}

#[godot_api]
impl DelaunayTriangulator {
    #[func]
    pub fn set_points(&mut self, points: PackedVector2Array) {
        self.points = points;
        self.cdt = None;
    }

    #[func]
    pub fn set_constraints(&mut self, constraints: PackedInt32Array) {
        self.constraints = constraints;
        self.cdt = None;
    }

    #[func]
    pub fn triangulate(&mut self) {
        let point_vec: Vec<Vector2> = self.points.as_slice().to_vec();

        if point_vec.len() < 3 {
            godot_warn!("Need at least 3 points for triangulation");
            self.cdt = None;
            return;
        }

        let mut cdt = CDT::from_points(point_vec.clone());

        let constraint_slice = self.constraints.as_slice();
        for pair in constraint_slice.chunks(2) {
            if pair.len() == 2 {
                cdt.insert_constraint(pair[0] as u32, pair[1] as u32);
            }
        }

        cdt.remove_super_triangle();
        self.cdt = Some(cdt);
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
}
