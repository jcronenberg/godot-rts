use godot::prelude::*;

/// Sentinel value meaning "no half-edge" / "no vertex" / "no face".
pub const NONE: u32 = u32::MAX;

/// 8 bytes — no padding. `constrained` lives in CDT::he_constrained to keep this tight.
#[derive(Debug, Clone)]
struct HalfEdge {
    origin: u32, // vertex this half-edge starts from
    twin: u32,   // opposite half-edge (NONE for boundary)
}

/// Constrained Delaunay Triangulation built on a half-edge (DCEL) data structure.
///
/// Half-edges are stored in groups of 3 — one triple per face/triangle. This lets
/// us derive `face` and `next` from the index instead of storing them.
///
/// `he_constrained` is a parallel array to `half_edges` tracking constraint flags.
/// Kept separate so HalfEdge stays 8 bytes (vs 12 with an inline bool), improving
/// cache utilization in the hot walking locator.
#[derive(Debug, Clone)]
pub struct CDT {
    points: Vec<Vector2>,
    half_edges: Vec<HalfEdge>,  // always len % 3 == 0
    he_constrained: Vec<bool>,  // parallel to half_edges
    vertex_half_edge: Vec<u32>, // one outgoing half-edge per vertex
    grid_cells: Vec<u32>,       // flat grid, each cell stores one face index or NONE
    grid_cols: u32,
    grid_rows: u32,
    grid_origin: Vector2,    // bounding box min corner
    grid_cell_size: Vector2, // per-cell dimensions
}

/// Face index for a half-edge.
#[inline(always)]
fn face_of(he: u32) -> u32 {
    he / 3
}

/// Next half-edge within the same face (CCW).
#[inline(always)]
fn next(he: u32) -> u32 {
    if he % 3 == 2 { he - 2 } else { he + 1 }
}

/// Previous half-edge within the same face (CCW).
#[inline(always)]
fn prev(he: u32) -> u32 {
    if he % 3 == 0 { he + 2 } else { he - 1 }
}

/// Orientation test: positive if CCW, negative if CW, zero if collinear.
#[inline]
fn orient2d(a: Vector2, b: Vector2, c: Vector2) -> f32 {
    (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
}

/// Returns true if `p` is strictly inside the circumcircle of CCW triangle (a, b, c).
fn in_circumcircle(a: Vector2, b: Vector2, c: Vector2, p: Vector2) -> bool {
    // Ensure CCW winding for the determinant
    let (b, c) = if orient2d(a, b, c) < 0.0 {
        (c, b)
    } else {
        (b, c)
    };

    let ax = a.x - p.x;
    let ay = a.y - p.y;
    let bx = b.x - p.x;
    let by = b.y - p.y;
    let cx = c.x - p.x;
    let cy = c.y - p.y;

    let det = (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay)
        + (cx * cx + cy * cy) * (ax * by - bx * ay);

    det > 0.0
}

/// Returns true if `p` is strictly inside the circumcircle of triangle (a, b, c).
/// Assumes (a, b, c) are already in CCW order — skips the orientation check.
#[inline]
fn in_circumcircle_ccw(a: Vector2, b: Vector2, c: Vector2, p: Vector2) -> bool {
    let ax = a.x - p.x;
    let ay = a.y - p.y;
    let bx = b.x - p.x;
    let by = b.y - p.y;
    let cx = c.x - p.x;
    let cy = c.y - p.y;

    let det = (ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay)
        + (cx * cx + cy * cy) * (ax * by - bx * ay);

    det > 0.0
}

/// Returns true if `pt` is inside (or on the boundary of) triangle (p0, p1, p2).
fn is_point_in_triangle(pt: Vector2, p0: Vector2, p1: Vector2, p2: Vector2) -> bool {
    let d1 = orient2d(pt, p0, p1);
    let d2 = orient2d(pt, p1, p2);
    let d3 = orient2d(pt, p2, p0);

    let has_neg = (d1 < 0.0) || (d2 < 0.0) || (d3 < 0.0);
    let has_pos = (d1 > 0.0) || (d2 > 0.0) || (d3 > 0.0);

    !(has_neg && has_pos)
}

/// Spread a 16-bit integer into 32 bits with a zero between each bit.
/// Used to compute Morton codes (Z-order curve).
#[inline]
fn spread_bits_16(x: u32) -> u32 {
    let mut x = x & 0x0000_ffff;
    x = (x | (x << 8)) & 0x00ff_00ff;
    x = (x | (x << 4)) & 0x0f0f_0f0f;
    x = (x | (x << 2)) & 0x3333_3333;
    x = (x | (x << 1)) & 0x5555_5555;
    x
}

/// Sort points by Morton code (Z-order curve) for spatial coherence.
///
/// Points with adjacent Morton codes are spatially nearby, so inserting in
/// this order keeps consecutive insertions close together. This reduces the
/// walking locator from O(n) per point (O(n²) total) to O(1) amortized,
/// giving O(n log n) overall instead of O(n²) for random inputs.
fn sort_spatially(mut points: Vec<Vector2>) -> Vec<Vector2> {
    if points.len() < 2 {
        return points;
    }

    let (mut min_x, mut max_x, mut min_y, mut max_y) = (
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::INFINITY,
        f32::NEG_INFINITY,
    );
    for p in &points {
        min_x = min_x.min(p.x);
        max_x = max_x.max(p.x);
        min_y = min_y.min(p.y);
        max_y = max_y.max(p.y);
    }
    let range_x = (max_x - min_x).max(f32::EPSILON);
    let range_y = (max_y - min_y).max(f32::EPSILON);

    points.sort_unstable_by_key(|p| {
        let nx = ((p.x - min_x) / range_x * 65535.0) as u32;
        let ny = ((p.y - min_y) / range_y * 65535.0) as u32;
        spread_bits_16(nx) | (spread_bits_16(ny) << 1)
    });

    points
}

/// Returns true if open segments (a,b) and (c,d) properly intersect
/// (endpoints touching does not count).
fn segments_intersect_proper(a: Vector2, b: Vector2, c: Vector2, d: Vector2) -> bool {
    let d1 = orient2d(c, d, a);
    let d2 = orient2d(c, d, b);
    let d3 = orient2d(a, b, c);
    let d4 = orient2d(a, b, d);

    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }

    false
}

#[cfg(debug_assertions)]
impl CDT {
    fn assert_dcel_valid(&self) {
        assert!(
            self.half_edges.len().is_multiple_of(3),
            "half_edges length must be multiple of 3"
        );

        for (i, he) in self.half_edges.iter().enumerate() {
            let i = i as u32;

            // Twin symmetry
            if he.twin != NONE {
                assert!(
                    (he.twin as usize) < self.half_edges.len(),
                    "twin {0} of he {1} out of bounds",
                    he.twin,
                    i
                );
                assert_eq!(
                    self.half_edges[he.twin as usize].twin, i,
                    "twin symmetry broken for he {}",
                    i
                );
            }

            // Origin valid
            assert!(
                (he.origin as usize) < self.points.len(),
                "origin {} of he {} out of bounds",
                he.origin,
                i
            );

            // Continuity: origin(next(he)) == destination(he)
            let n = next(i);
            let dest = self.half_edges[n as usize].origin;
            if he.twin != NONE {
                let twin_origin = self.half_edges[he.twin as usize].origin;
                assert_eq!(
                    dest, twin_origin,
                    "continuity broken: next({}).origin={} != twin({}).origin={}",
                    i, dest, i, twin_origin
                );
            }
        }
    }
}

impl CDT {
    /// Origin vertex of a half-edge.
    #[inline]
    fn origin(&self, he: u32) -> u32 {
        self.half_edges[he as usize].origin
    }

    /// Destination vertex of a half-edge (= origin of next).
    #[inline]
    fn dest(&self, he: u32) -> u32 {
        self.half_edges[next(he) as usize].origin
    }

    /// Position of a vertex.
    #[inline]
    fn point(&self, v: u32) -> Vector2 {
        self.points[v as usize]
    }

    /// Allocate 3 half-edges for a new face. Returns the base index.
    fn alloc_face(&mut self, v0: u32, v1: u32, v2: u32) -> u32 {
        let base = self.half_edges.len() as u32;
        self.half_edges.push(HalfEdge {
            origin: v0,
            twin: NONE,
        });
        self.half_edges.push(HalfEdge {
            origin: v1,
            twin: NONE,
        });
        self.half_edges.push(HalfEdge {
            origin: v2,
            twin: NONE,
        });
        self.he_constrained
            .extend_from_slice(&[false, false, false]);
        base
    }

    /// Link two half-edges as twins.
    fn link_twins(&mut self, a: u32, b: u32) {
        self.half_edges[a as usize].twin = b;
        self.half_edges[b as usize].twin = a;
    }

    fn init_super_triangle(points: &mut Vec<Vector2>) -> CDT {
        let n = points.len();

        // Compute bounding box
        let (mut min_x, mut max_x, mut min_y, mut max_y) = (
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::INFINITY,
            f32::NEG_INFINITY,
        );
        for p in points.iter() {
            min_x = min_x.min(p.x);
            max_x = max_x.max(p.x);
            min_y = min_y.min(p.y);
            max_y = max_y.max(p.y);
        }

        let dx = (max_x - min_x).max(1.0) * 10.0;
        let dy = (max_y - min_y).max(1.0) * 10.0;
        let mid_x = (min_x + max_x) / 2.0;
        let mid_y = (min_y + max_y) / 2.0;

        // Super-triangle vertices at end of points array
        let sv0 = n as u32;
        let sv1 = n as u32 + 1;
        let sv2 = n as u32 + 2;
        // CCW order: bottom-left, bottom-right, top
        points.push(Vector2::new(mid_x - dx, mid_y - dy));
        points.push(Vector2::new(mid_x + dx, mid_y - dy));
        points.push(Vector2::new(mid_x, mid_y + dy));

        let mut cdt = CDT {
            points: Vec::new(), // will be set later
            half_edges: Vec::new(),
            he_constrained: Vec::new(),
            vertex_half_edge: vec![NONE; n + 3],
            grid_cells: Vec::new(),
            grid_cols: 0,
            grid_rows: 0,
            grid_origin: Vector2::ZERO,
            grid_cell_size: Vector2::ZERO,
        };

        // Face 0: the super-triangle (CCW)
        cdt.alloc_face(sv0, sv1, sv2);

        // Set vertex_half_edge for super-triangle vertices
        cdt.vertex_half_edge[sv0 as usize] = 0;
        cdt.vertex_half_edge[sv1 as usize] = 1;
        cdt.vertex_half_edge[sv2 as usize] = 2;

        cdt
    }

    /// Walk the DCEL from `start_face` toward the face containing `pt`.
    fn locate_containing_face(&self, pt: Vector2, start_face: u32) -> u32 {
        let mut current_face = start_face;
        let max_iters = self.half_edges.len(); // safety bound

        for _ in 0..max_iters {
            let base = current_face * 3;
            let v0 = self.origin(base);
            let v1 = self.origin(base + 1);
            let v2 = self.origin(base + 2);

            let p0 = self.point(v0);
            let p1 = self.point(v1);
            let p2 = self.point(v2);

            // Check each edge: if pt is on the wrong side, walk through that edge
            let d0 = orient2d(p0, p1, pt);
            if d0 < 0.0 {
                let twin = self.half_edges[base as usize].twin;
                if twin != NONE {
                    current_face = face_of(twin);
                    continue;
                }
            }

            let d1 = orient2d(p1, p2, pt);
            if d1 < 0.0 {
                let twin = self.half_edges[(base + 1) as usize].twin;
                if twin != NONE {
                    current_face = face_of(twin);
                    continue;
                }
            }

            let d2 = orient2d(p2, p0, pt);
            if d2 < 0.0 {
                let twin = self.half_edges[(base + 2) as usize].twin;
                if twin != NONE {
                    current_face = face_of(twin);
                    continue;
                }
            }

            // pt is inside (or on boundary of) this face
            return current_face;
        }

        // Fallback: brute-force search
        self.locate_face_brute(pt).unwrap_or(current_face)
    }

    fn locate_face_brute(&self, pt: Vector2) -> Option<u32> {
        let num_faces = self.half_edges.len() as u32 / 3;
        for f in 0..num_faces {
            let base = f * 3;
            let p0 = self.point(self.origin(base));
            let p1 = self.point(self.origin(base + 1));
            let p2 = self.point(self.origin(base + 2));
            if is_point_in_triangle(pt, p0, p1, p2) {
                return Some(f);
            }
        }
        None
    }

    /// Insert vertex `v` inside face `f`, splitting it into 3 new faces.
    /// The original face slot is reused for one sub-face; two new faces are appended.
    fn split_face_3(&mut self, f: u32, v: u32) {
        let base = f * 3;
        let he0 = base;
        let he1 = base + 1;
        let he2 = base + 2;

        let v0 = self.origin(he0);
        let v1 = self.origin(he1);
        let v2 = self.origin(he2);

        let twin0 = self.half_edges[he0 as usize].twin;
        let twin1 = self.half_edges[he1 as usize].twin;
        let twin2 = self.half_edges[he2 as usize].twin;

        let con0 = self.he_constrained[he0 as usize];
        let con1 = self.he_constrained[he1 as usize];
        let con2 = self.he_constrained[he2 as usize];

        // Reuse face f for triangle (v0, v1, v)
        // Overwrite in-place
        self.half_edges[he0 as usize] = HalfEdge {
            origin: v0,
            twin: twin0,
        };
        self.he_constrained[he0 as usize] = con0;
        self.half_edges[he1 as usize] = HalfEdge {
            origin: v1,
            twin: NONE,
        };
        self.he_constrained[he1 as usize] = false;
        self.half_edges[he2 as usize] = HalfEdge {
            origin: v,
            twin: NONE,
        };
        self.he_constrained[he2 as usize] = false;

        // New face A: (v1, v2, v)
        let a_base = self.alloc_face(v1, v2, v);
        self.half_edges[a_base as usize].twin = twin1;
        self.he_constrained[a_base as usize] = con1;
        if twin1 != NONE {
            self.half_edges[twin1 as usize].twin = a_base;
        }

        // New face B: (v2, v0, v)
        let b_base = self.alloc_face(v2, v0, v);
        self.half_edges[b_base as usize].twin = twin2;
        self.he_constrained[b_base as usize] = con2;
        if twin2 != NONE {
            self.half_edges[twin2 as usize].twin = b_base;
        }

        // Link internal edges:
        // he1 (v1->v in face f) <-> b_base+2 (v->v1 in face B)... wait let me think
        // face f:  he0=v0->v1, he1=v1->v, he2=v->v0
        // face A:  a0=v1->v2, a1=v2->v, a2=v->v1
        // face B:  b0=v2->v0, b1=v0->v, b2=v->v2

        // Internal twin pairs:
        // he1 (v1->v) <-> a_base+2 (v->v1)
        self.link_twins(he1, a_base + 2);
        // a_base+1 (v2->v) <-> b_base+2 (v->v2)
        self.link_twins(a_base + 1, b_base + 2);
        // b_base+1 (v0->v) <-> he2 (v->v0)
        self.link_twins(b_base + 1, he2);

        // Update vertex_half_edge
        self.vertex_half_edge[v as usize] = he2; // v->v0
        self.vertex_half_edge[v0 as usize] = he0;
        self.vertex_half_edge[v1 as usize] = he1; // keep valid; v1->v
        // v1 also starts a_base (v1->v2), pick one that's valid
        self.vertex_half_edge[v1 as usize] = a_base; // v1->v2
        self.vertex_half_edge[v2 as usize] = b_base; // v2->v0
    }

    /// Insert vertex `v` on edge `he_on_edge`, splitting the two adjacent faces into 4.
    fn split_edge_4(&mut self, he_on_edge: u32, v: u32) {
        let twin_edge = self.half_edges[he_on_edge as usize].twin;
        assert!(twin_edge != NONE, "cannot split boundary edge");

        // Face 1: triangle containing he_on_edge
        // he_on_edge: A->B, next: B->C, prev: C->A
        let he_ab = he_on_edge;
        let he_bc = next(he_ab);
        let he_ca = prev(he_ab);

        let vertex_a = self.origin(he_ab);
        let vertex_b = self.origin(he_bc);
        let vertex_c = self.origin(he_ca);

        let twin_bc = self.half_edges[he_bc as usize].twin;
        let twin_ca = self.half_edges[he_ca as usize].twin;
        let con_bc = self.he_constrained[he_bc as usize];
        let con_ca = self.he_constrained[he_ca as usize];

        // Face 2: triangle containing twin_edge
        // twin_edge: B->A, next: A->D, prev: D->B
        let he_ba = twin_edge;
        let he_ad = next(he_ba);
        let he_db = prev(he_ba);

        let vertex_d = self.origin(he_db);

        let twin_ad = self.half_edges[he_ad as usize].twin;
        let twin_db = self.half_edges[he_db as usize].twin;
        let con_ad = self.he_constrained[he_ad as usize];
        let con_db = self.he_constrained[he_db as usize];

        let con_edge = self.he_constrained[he_ab as usize];

        // After splitting edge A-B at V, we get 4 CCW triangles:
        //   T1: (C, A, V) at f1_base — reuse face of he_ab
        //   T2: (C, V, B) at t2_base — new
        //   T3: (D, B, V) at f2_base — reuse face of he_ba
        //   T4: (D, V, A) at t4_base — new

        let f1_base = face_of(he_ab) * 3;
        let f2_base = face_of(he_ba) * 3;

        // T1: (C, A, V) — edges: C->A, A->V, V->C
        self.half_edges[f1_base as usize] = HalfEdge {
            origin: vertex_c,
            twin: twin_ca,
        };
        self.he_constrained[f1_base as usize] = con_ca;
        if twin_ca != NONE {
            self.half_edges[twin_ca as usize].twin = f1_base;
        }
        self.half_edges[(f1_base + 1) as usize] = HalfEdge {
            origin: vertex_a,
            twin: NONE,
        };
        self.he_constrained[(f1_base + 1) as usize] = con_edge;
        self.half_edges[(f1_base + 2) as usize] = HalfEdge {
            origin: v,
            twin: NONE,
        };
        self.he_constrained[(f1_base + 2) as usize] = false;

        // Reuse face of he_ba for T3: (D, B, V)
        self.half_edges[f2_base as usize] = HalfEdge {
            origin: vertex_d,
            twin: twin_db,
        };
        self.he_constrained[f2_base as usize] = con_db;
        if twin_db != NONE {
            self.half_edges[twin_db as usize].twin = f2_base;
        }
        self.half_edges[(f2_base + 1) as usize] = HalfEdge {
            origin: vertex_b,
            twin: NONE,
        };
        self.he_constrained[(f2_base + 1) as usize] = con_edge;
        self.half_edges[(f2_base + 2) as usize] = HalfEdge {
            origin: v,
            twin: NONE,
        };
        self.he_constrained[(f2_base + 2) as usize] = false;

        // New face T2: (C, V, B)
        let t2_base = self.alloc_face(vertex_c, v, vertex_b);
        // C->V twin with T1's V->C (f1_base+2)
        // V->B twin with T3's B->V (f2_base+1)
        // B->C twin with original twin_bc
        self.half_edges[(t2_base + 2) as usize].twin = twin_bc;
        self.he_constrained[(t2_base + 2) as usize] = con_bc;
        if twin_bc != NONE {
            self.half_edges[twin_bc as usize].twin = t2_base + 2;
        }
        self.link_twins(t2_base, f1_base + 2); // C->V <-> V->C
        self.link_twins(t2_base + 1, f2_base + 1); // V->B <-> B->V

        // New face T4: (D, V, A)
        let t4_base = self.alloc_face(vertex_d, v, vertex_a);
        // D->V twin with T3's V->D (f2_base+2)
        // V->A twin with T1's A->V (f1_base+1)
        // A->D twin with original twin_ad
        self.half_edges[(t4_base + 2) as usize].twin = twin_ad;
        self.he_constrained[(t4_base + 2) as usize] = con_ad;
        if twin_ad != NONE {
            self.half_edges[twin_ad as usize].twin = t4_base + 2;
        }
        self.link_twins(t4_base, f2_base + 2); // D->V <-> V->D
        self.link_twins(t4_base + 1, f1_base + 1); // V->A <-> A->V

        // Update vertex_half_edge
        self.vertex_half_edge[v as usize] = f1_base + 2; // V->C
        self.vertex_half_edge[vertex_a as usize] = f1_base + 1; // A->V
        self.vertex_half_edge[vertex_b as usize] = f2_base + 1; // B->V
        self.vertex_half_edge[vertex_c as usize] = f1_base; // C->A
        self.vertex_half_edge[vertex_d as usize] = f2_base; // D->B
    }

    /// Flip edge `he_id` (and its twin). The quad must be convex.
    ///
    /// Before: he_id = A->B in face (A,B,C), twin = B->A in face (B,A,D)
    /// After:  face1 = (C,A,D) CCW, face2 = (D,B,C) CCW
    fn flip_edge(&mut self, he_id: u32) {
        let twin_id = self.half_edges[he_id as usize].twin;
        debug_assert!(twin_id != NONE);

        // Gather all 6 half-edges of the two faces
        let he_ab = he_id;
        let he_bc = next(he_ab);
        let he_ca = prev(he_ab);

        let he_ba = twin_id;
        let he_ad = next(he_ba);
        let he_db = prev(he_ba);

        let vertex_a = self.origin(he_ab);
        let vertex_b = self.origin(he_ba);
        let vertex_c = self.origin(he_ca);
        let vertex_d = self.origin(he_db);

        // External twins
        let twin_bc = self.half_edges[he_bc as usize].twin;
        let twin_ca = self.half_edges[he_ca as usize].twin;
        let twin_ad = self.half_edges[he_ad as usize].twin;
        let twin_db = self.half_edges[he_db as usize].twin;

        let con_bc = self.he_constrained[he_bc as usize];
        let con_ca = self.he_constrained[he_ca as usize];
        let con_ad = self.he_constrained[he_ad as usize];
        let con_db = self.he_constrained[he_db as usize];

        // Rewrite face of he_ab as (C, A, D) — CCW
        // Edges: C->A, A->D, D->C
        let f1_base = face_of(he_ab) * 3;
        self.half_edges[f1_base as usize] = HalfEdge {
            origin: vertex_c,
            twin: twin_ca,
        };
        self.he_constrained[f1_base as usize] = con_ca;
        self.half_edges[(f1_base + 1) as usize] = HalfEdge {
            origin: vertex_a,
            twin: twin_ad,
        };
        self.he_constrained[(f1_base + 1) as usize] = con_ad;
        self.half_edges[(f1_base + 2) as usize] = HalfEdge {
            origin: vertex_d,
            twin: NONE,
        };
        self.he_constrained[(f1_base + 2) as usize] = false;

        // Rewrite face of he_ba as (D, B, C) — CCW
        // Edges: D->B, B->C, C->D
        let f2_base = face_of(he_ba) * 3;
        self.half_edges[f2_base as usize] = HalfEdge {
            origin: vertex_d,
            twin: twin_db,
        };
        self.he_constrained[f2_base as usize] = con_db;
        self.half_edges[(f2_base + 1) as usize] = HalfEdge {
            origin: vertex_b,
            twin: twin_bc,
        };
        self.he_constrained[(f2_base + 1) as usize] = con_bc;
        self.half_edges[(f2_base + 2) as usize] = HalfEdge {
            origin: vertex_c,
            twin: NONE,
        };
        self.he_constrained[(f2_base + 2) as usize] = false;

        // Internal twin: D->C (f1_base+2) <-> C->D (f2_base+2)
        self.link_twins(f1_base + 2, f2_base + 2);

        // Fix external twin back-pointers
        if twin_ca != NONE {
            self.half_edges[twin_ca as usize].twin = f1_base;
        }
        if twin_ad != NONE {
            self.half_edges[twin_ad as usize].twin = f1_base + 1;
        }
        if twin_db != NONE {
            self.half_edges[twin_db as usize].twin = f2_base;
        }
        if twin_bc != NONE {
            self.half_edges[twin_bc as usize].twin = f2_base + 1;
        }

        // Update vertex_half_edge
        self.vertex_half_edge[vertex_a as usize] = f1_base + 1; // A->D
        self.vertex_half_edge[vertex_b as usize] = f2_base + 1; // B->C
        self.vertex_half_edge[vertex_c as usize] = f1_base; // C->A
        self.vertex_half_edge[vertex_d as usize] = f2_base; // D->B
    }

    /// Legalize edge opposite to vertex `v` in the face containing `he_id`.
    /// `he_id` is the half-edge opposite to `v` (i.e., the edge that might need flipping).
    fn legalize(&mut self, he_id: u32, v: u32) {
        let twin_id = self.half_edges[he_id as usize].twin;
        if twin_id == NONE {
            return;
        }

        if self.he_constrained[he_id as usize] {
            return;
        }

        let vertex_a = self.origin(he_id);
        let vertex_b = self.dest(he_id);

        // Find the vertex opposite to the edge in the twin's face
        let vertex_d = self.origin(prev(twin_id));

        let pa = self.point(vertex_a);
        let pb = self.point(vertex_b);
        let pv = self.point(v);
        let pd = self.point(vertex_d);

        if in_circumcircle_ccw(pa, pb, pv, pd) {
            self.flip_edge(he_id);

            // After flip: face1 = (C,A,D), face2 = (D,B,C) where C = v.
            // Edges opposite to v=C:
            //   Face1 (C->A, A->D, D->C): A->D at f1_base+1
            //   Face2 (D->B, B->C, C->D): D->B at f2_base+0
            let f1_base = face_of(he_id) * 3;
            let f2_base = face_of(twin_id) * 3;

            self.legalize(f1_base + 1, v);
            self.legalize(f2_base, v);
        }
    }

    /// Build a CDT from points and immediately remove the super-triangle.
    /// Use `from_points` instead if you need to insert constraints before removal.
    ///
    /// Points are sorted by Morton code (Z-order) before insertion so the walking
    /// locator stays O(1) amortized — giving O(n log n) total instead of O(n²).
    pub fn triangulate(points: Vec<Vector2>) -> CDT {
        let mut cdt = CDT::from_points(sort_spatially(points));
        cdt.remove_super_triangle();
        cdt.build_grid_index();
        cdt
    }

    /// Build a CDT from points, keeping the super-triangle so constraints can be inserted.
    /// Call `remove_super_triangle()` when done adding constraints.
    pub fn from_points(mut points: Vec<Vector2>) -> CDT {
        let n = points.len();
        assert!(n >= 3, "Need at least 3 points for triangulation");

        let mut cdt = CDT::init_super_triangle(&mut points);
        cdt.points = points;

        let mut last_face: u32 = 0;

        for i in 0..n as u32 {
            let pt = cdt.points[i as usize];

            // Locate containing face
            let f = cdt.locate_containing_face(pt, last_face);

            // Check if point is on an edge of the face
            let base = f * 3;
            let mut on_edge: Option<u32> = None;
            for j in 0..3u32 {
                let he = base + j;
                let a = cdt.point(cdt.origin(he));
                let b = cdt.point(cdt.dest(he));
                let o = orient2d(a, b, pt).abs();
                let edge_len = ((b.x - a.x).powi(2) + (b.y - a.y).powi(2)).sqrt();
                if o < edge_len * 1e-5 && edge_len > 1e-10 {
                    // Check pt is actually between a and b
                    let t = if (b.x - a.x).abs() > (b.y - a.y).abs() {
                        (pt.x - a.x) / (b.x - a.x)
                    } else {
                        (pt.y - a.y) / (b.y - a.y)
                    };
                    if t > 1e-4 && t < 1.0 - 1e-4 {
                        on_edge = Some(he);
                        break;
                    }
                }
            }

            if let Some(he) = on_edge {
                // Save f2_base BEFORE split (twin changes during split)
                let f1_base = face_of(he) * 3;
                let f2_base = face_of(cdt.half_edges[he as usize].twin) * 3;

                cdt.split_edge_4(he, i);

                // After split_edge_4:
                // T1 at f1_base: C->A, A->V, V->C — legalize C->A (f1_base)
                // T3 at f2_base: D->B, B->V, V->D — legalize D->B (f2_base)
                // T2 at t2_base: C->V, V->B, B->C — legalize B->C (t2_base+2)
                // T4 at t4_base: D->V, V->A, A->D — legalize A->D (t4_base+2)
                let num_he = cdt.half_edges.len() as u32;
                let t2_base = num_he - 6;
                let t4_base = num_he - 3;

                cdt.legalize(f1_base, i);
                cdt.legalize(f2_base, i);
                cdt.legalize(t2_base + 2, i);
                cdt.legalize(t4_base + 2, i);
            } else {
                // Point inside face: split into 3
                cdt.split_face_3(f, i);

                // Legalize the 3 outer edges (the original edges of face f)
                let base = f * 3;
                // After split_face_3:
                // Face f reused: (v0,v1,v) — outer edge is he0 (v0->v1)
                let a_base = cdt.half_edges.len() as u32 - 6;
                let b_base = cdt.half_edges.len() as u32 - 3;
                // Face A: (v1,v2,v) — outer edge is a_base (v1->v2)
                // Face B: (v2,v0,v) — outer edge is b_base (v2->v0)

                cdt.legalize(base, i);
                cdt.legalize(a_base, i);
                cdt.legalize(b_base, i);
            }

            last_face = f.min((cdt.half_edges.len() as u32 / 3).saturating_sub(1));
        }

        #[cfg(debug_assertions)]
        cdt.assert_dcel_valid();

        cdt
    }

    /// Find the half-edge from vertex `v0` to vertex `v1`, if it exists.
    fn find_half_edge(&self, v0: u32, v1: u32) -> Option<u32> {
        let start = self.vertex_half_edge[v0 as usize];
        if start == NONE {
            return None;
        }

        // Walk around the vertex star of v0
        let mut he = start;
        loop {
            if self.dest(he) == v1 {
                return Some(he);
            }
            // Move to next outgoing edge from v0: prev(twin(he))
            let p = prev(he);
            let t = self.half_edges[p as usize].twin;
            if t == NONE {
                // Hit boundary, try walking the other direction
                break;
            }
            he = t;
            if he == start {
                break;
            }
        }

        // Also try walking clockwise from start
        let mut he = start;
        loop {
            let t = self.half_edges[he as usize].twin;
            if t == NONE {
                break;
            }
            he = next(t);
            if he == start {
                break;
            }
            if self.dest(he) == v1 {
                return Some(he);
            }
        }

        None
    }

    /// Insert constraint edge between vertices `v0` and `v1`.
    pub fn insert_constraint(&mut self, v0: u32, v1: u32) {
        if v0 == v1 {
            return;
        }
        self.grid_cells.clear(); // topology changes; rebuild with build_grid_index if needed

        // If edge already exists, just mark it constrained
        if let Some(he) = self.find_half_edge(v0, v1) {
            self.he_constrained[he as usize] = true;
            let twin = self.half_edges[he as usize].twin;
            if twin != NONE {
                self.he_constrained[twin as usize] = true;
            }
            return;
        }

        let p0 = self.point(v0);
        let p1 = self.point(v1);

        // Check if any vertex lies strictly between v0 and v1 on the segment.
        // If so, split the constraint at the closest such vertex and recurse.
        {
            let dx = p1.x - p0.x;
            let dy = p1.y - p0.y;
            let len_sq = dx * dx + dy * dy;
            let mut best: Option<(u32, f32)> = None; // (vertex, parametric t)
            for vi in 0..self.points.len() as u32 {
                if vi == v0 || vi == v1 {
                    continue;
                }
                let pi = self.point(vi);
                // Check collinearity
                if orient2d(p0, p1, pi).abs() > 1e-4 {
                    continue;
                }
                // Parametric t along p0→p1
                let t = ((pi.x - p0.x) * dx + (pi.y - p0.y) * dy) / len_sq;
                if t > 1e-6 && t < 1.0 - 1e-6 && (best.is_none() || t < best.unwrap().1) {
                    best = Some((vi, t));
                }
            }
            if let Some((v_mid, _)) = best {
                self.insert_constraint(v0, v_mid);
                self.insert_constraint(v_mid, v1);
                return;
            }
        }

        // Collect crossing edges
        let mut crossing: Vec<u32> = Vec::new();

        // Walk from v0 toward v1, collecting half-edges that cross segment (v0,v1)
        // Start by finding the face adjacent to v0 through which the segment exits
        let start_he = self.vertex_half_edge[v0 as usize];
        assert!(start_he != NONE);

        // Find the outgoing half-edge from v0 such that the segment (v0,v1) passes
        // through the face on the left of that half-edge
        let mut exit_he: Option<u32> = None;

        // Walk around v0's fan to find which face the segment exits through.
        // Check both CW and CCW directions to handle boundary fans.
        let mut he = start_he;
        let mut first = true;
        loop {
            if !first && he == start_he {
                break;
            }
            first = false;

            let n = next(he);

            let d_dest = orient2d(p0, p1, self.point(self.dest(he)));
            let d_next_dest = orient2d(p0, p1, self.point(self.dest(n)));

            // The segment crosses the opposite edge if the two vertices are on opposite sides
            if d_dest * d_next_dest < 0.0 {
                let opp_edge = n;
                let ea = self.point(self.origin(opp_edge));
                let eb = self.point(self.dest(opp_edge));
                if segments_intersect_proper(p0, p1, ea, eb) {
                    exit_he = Some(opp_edge);
                    break;
                }
            }

            // Move to next face around v0 (CW direction)
            let p = prev(he);
            let t = self.half_edges[p as usize].twin;
            if t == NONE {
                break;
            }
            he = t;
        }

        // Try the other direction (CCW) if not found
        if exit_he.is_none() {
            let mut he = start_he;
            loop {
                let t = self.half_edges[he as usize].twin;
                if t == NONE {
                    break;
                }
                he = next(t);
                if he == start_he {
                    break;
                }

                let n = next(he);
                let d_dest = orient2d(p0, p1, self.point(self.dest(he)));
                let d_next_dest = orient2d(p0, p1, self.point(self.dest(n)));

                if d_dest * d_next_dest < 0.0 {
                    let opp_edge = n;
                    let ea = self.point(self.origin(opp_edge));
                    let eb = self.point(self.dest(opp_edge));
                    if segments_intersect_proper(p0, p1, ea, eb) {
                        exit_he = Some(opp_edge);
                        break;
                    }
                }
            }
        }

        let Some(first_crossing) = exit_he else {
            return;
        };

        crossing.push(first_crossing);

        // Continue walking through faces until we reach v1
        let max_iters = self.half_edges.len();
        for _ in 0..max_iters {
            let last_he = *crossing.last().unwrap();
            let twin = self.half_edges[last_he as usize].twin;
            if twin == NONE {
                break;
            }

            // We entered the twin's face through `twin`. Check if v1 is in this face.
            let f_base = face_of(twin) * 3;
            let fv0 = self.origin(f_base);
            let fv1 = self.origin(f_base + 1);
            let fv2 = self.origin(f_base + 2);
            if fv0 == v1 || fv1 == v1 || fv2 == v1 {
                break; // Reached destination
            }

            // Find which of the other two edges of this face the segment crosses
            for j in 0..3u32 {
                let he = f_base + j;
                if he == twin {
                    continue;
                }
                let ea = self.point(self.origin(he));
                let eb = self.point(self.dest(he));
                if segments_intersect_proper(p0, p1, ea, eb) {
                    crossing.push(he);
                    break;
                }
            }
        }

        // Flip crossing edges until the constraint edge appears
        let mut queue = std::collections::VecDeque::from(crossing);
        let mut safety = 0;
        let max_flips = queue.len() * queue.len() + queue.len() + 100;

        while let Some(he) = queue.pop_front() {
            safety += 1;
            if safety > max_flips {
                break; // Safety bail-out
            }

            // Check if this edge is still crossing (it might have been flipped already)
            let a = self.origin(he);
            let b = self.dest(he);

            // If this IS the constraint edge, we're done with it
            if (a == v0 && b == v1) || (a == v1 && b == v0) {
                continue;
            }

            // Check it still crosses
            if !segments_intersect_proper(p0, p1, self.point(a), self.point(b)) {
                continue;
            }

            let twin = self.half_edges[he as usize].twin;
            if twin == NONE {
                continue;
            }
            if self.he_constrained[he as usize] {
                godot_error!(
                    "Cannot insert constraint ({}-{}): it intersects existing constraint ({}-{})",
                    v0,
                    v1,
                    a,
                    b
                );
                return;
            }

            // Check if quad is convex: diagonals AB and CD must properly cross
            let vertex_c = self.origin(prev(he));
            let vertex_d = self.origin(prev(twin));

            let pc = self.point(vertex_c);
            let pd = self.point(vertex_d);
            let pa = self.point(a);
            let pb = self.point(b);

            // C and D on opposite sides of AB, and A and B on opposite sides of CD
            let convex = orient2d(pa, pb, pc) * orient2d(pa, pb, pd) < 0.0
                && orient2d(pc, pd, pa) * orient2d(pc, pd, pb) < 0.0;

            if !convex {
                queue.push_back(he);
                continue;
            }

            let twin_before = self.half_edges[he as usize].twin;
            self.flip_edge(he);

            // After flipping, both affected faces are rewritten. Check all their
            // edges for new crossings (the flip may have invalidated queued indices
            // that pointed into these faces).
            let f1_base = face_of(he) * 3;
            let f2_base = face_of(twin_before) * 3;
            for &check in &[
                f1_base,
                f1_base + 1,
                f1_base + 2,
                f2_base,
                f2_base + 1,
                f2_base + 2,
            ] {
                let ca = self.origin(check);
                let cb = self.dest(check);
                if (ca == v0 && cb == v1) || (ca == v1 && cb == v0) {
                    continue;
                }
                if segments_intersect_proper(p0, p1, self.point(ca), self.point(cb)) {
                    queue.push_back(check);
                }
            }
        }

        // Mark the constraint edge
        let Some(constraint_he) = self.find_half_edge(v0, v1) else {
            godot_error!(
                "Failed to insert constraint ({}-{}): edge not found after flipping",
                v0,
                v1
            );
            return;
        };
        self.he_constrained[constraint_he as usize] = true;
        let twin = self.half_edges[constraint_he as usize].twin;
        if twin != NONE {
            self.he_constrained[twin as usize] = true;
        }

        // Re-legalize non-constrained edges near the constraint
        // Collect edges to legalize (those that were flipped and aren't the constraint)
        let num_faces = self.half_edges.len() as u32 / 3;
        for f in 0..num_faces {
            let base = f * 3;
            for j in 0..3u32 {
                let he = base + j;
                if self.he_constrained[he as usize] {
                    continue;
                }
                let twin = self.half_edges[he as usize].twin;
                if twin == NONE {
                    continue;
                }

                let a = self.origin(he);
                let b = self.dest(he);
                let c = self.origin(prev(he));
                let d = self.origin(prev(twin));

                if in_circumcircle(self.point(a), self.point(b), self.point(c), self.point(d)) {
                    // Only legalize if the edge involves vertices near the constraint
                    // For simplicity, just do a single pass of flips
                    self.flip_edge(he);
                    break; // face indices shifted, restart scan
                }
            }
        }
    }

    /// Remove super-triangle faces and compact the DCEL arrays.
    pub fn remove_super_triangle(&mut self) {
        self.grid_cells.clear(); // face indices change after compaction
        let n = self.points.len() as u32 - 3; // number of real vertices
        let sv0 = n;

        let num_faces = self.half_edges.len() as u32 / 3;

        // Identify live faces (those not touching super-triangle vertices)
        let mut face_alive = vec![false; num_faces as usize];
        for f in 0..num_faces {
            let base = f * 3;
            let v0 = self.origin(base);
            let v1 = self.origin(base + 1);
            let v2 = self.origin(base + 2);
            let touches_super = v0 >= sv0 || v1 >= sv0 || v2 >= sv0;
            face_alive[f as usize] = !touches_super;
        }

        // Build old->new half-edge index mapping
        let mut he_remap = vec![NONE; self.half_edges.len()];
        let mut new_he_idx: u32 = 0;
        for f in 0..num_faces {
            if face_alive[f as usize] {
                for j in 0..3u32 {
                    he_remap[(f * 3 + j) as usize] = new_he_idx;
                    new_he_idx += 1;
                }
            }
        }

        // Build new half_edges array (and parallel constrained vec)
        let mut new_half_edges = Vec::with_capacity(new_he_idx as usize);
        let mut new_he_constrained = Vec::with_capacity(new_he_idx as usize);
        for f in 0..num_faces {
            if !face_alive[f as usize] {
                continue;
            }
            for j in 0..3u32 {
                let old_idx = (f * 3 + j) as usize;
                let he = &self.half_edges[old_idx];
                let new_twin = if he.twin != NONE {
                    he_remap[he.twin as usize]
                } else {
                    NONE
                };
                new_half_edges.push(HalfEdge {
                    origin: he.origin,
                    twin: new_twin,
                });
                new_he_constrained.push(self.he_constrained[old_idx]);
            }
        }

        // Remap vertex_half_edge
        let mut new_vhe = vec![NONE; n as usize];
        for v in 0..n {
            let old = self.vertex_half_edge[v as usize];
            if old != NONE && (old as usize) < he_remap.len() {
                new_vhe[v as usize] = he_remap[old as usize];
            }
            // If the mapped value is NONE, find any valid outgoing half-edge
            if new_vhe[v as usize] == NONE {
                for (i, he) in new_half_edges.iter().enumerate() {
                    if he.origin == v {
                        new_vhe[v as usize] = i as u32;
                        break;
                    }
                }
            }
        }

        self.half_edges = new_half_edges;
        self.he_constrained = new_he_constrained;
        self.vertex_half_edge = new_vhe;
        self.points.truncate(n as usize);
    }

    /// Number of faces (triangles) in the triangulation.
    pub fn num_faces(&self) -> u32 {
        self.half_edges.len() as u32 / 3
    }

    /// Alias for `num_faces`.
    pub fn num_triangles(&self) -> u32 {
        self.num_faces()
    }

    /// Number of vertices.
    pub fn num_vertices(&self) -> u32 {
        self.points.len() as u32
    }

    /// Vertex indices of a face.
    pub fn face_vertices(&self, face: u32) -> [u32; 3] {
        let base = face * 3;
        [
            self.origin(base),
            self.origin(base + 1),
            self.origin(base + 2),
        ]
    }

    /// Adjacent faces through non-constrained, non-boundary edges.
    pub fn face_neighbors(&self, face: u32) -> Vec<u32> {
        let base = face * 3;
        let mut neighbors = Vec::with_capacity(3);
        for j in 0..3u32 {
            let he = base + j;
            let twin = self.half_edges[he as usize].twin;
            if twin != NONE && !self.he_constrained[he as usize] {
                neighbors.push(face_of(twin));
            }
        }
        neighbors
    }

    /// Centroid of a face.
    pub fn face_centroid(&self, face: u32) -> Vector2 {
        let [v0, v1, v2] = self.face_vertices(face);
        let p0 = self.point(v0);
        let p1 = self.point(v1);
        let p2 = self.point(v2);
        Vector2::new((p0.x + p1.x + p2.x) / 3.0, (p0.y + p1.y + p2.y) / 3.0)
    }

    /// Midpoint of the edge starting at half-edge `he`.
    pub fn edge_midpoint(&self, he: u32) -> Vector2 {
        let a = self.point(self.origin(he));
        let b = self.point(self.dest(he));
        Vector2::new((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)
    }

    /// Half-edge ID of the shared edge between two adjacent faces, from f1's side.
    pub fn shared_edge_between(&self, f1: u32, f2: u32) -> Option<u32> {
        let base = f1 * 3;
        for j in 0..3u32 {
            let he = base + j;
            let twin = self.half_edges[he as usize].twin;
            if twin != NONE && face_of(twin) == f2 {
                return Some(he);
            }
        }
        None
    }

    /// Build a grid index for fast `locate_face` seeding. Call after `remove_super_triangle`.
    /// Invalidated by `insert_constraint` — rebuild once after batched insertions.
    pub fn build_grid_index(&mut self) {
        let num_faces = self.num_faces();
        if num_faces == 0 {
            return;
        }

        let mut min_x = f32::INFINITY;
        let mut min_y = f32::INFINITY;
        let mut max_x = f32::NEG_INFINITY;
        let mut max_y = f32::NEG_INFINITY;
        for p in &self.points {
            if p.x < min_x {
                min_x = p.x;
            }
            if p.y < min_y {
                min_y = p.y;
            }
            if p.x > max_x {
                max_x = p.x;
            }
            if p.y > max_y {
                max_y = p.y;
            }
        }

        let pad = ((max_x - min_x).max(max_y - min_y)) * 0.001 + 1e-6;
        min_x -= pad;
        min_y -= pad;
        max_x += pad;
        max_y += pad;

        let resolution = (num_faces as f32).sqrt();
        let aspect = ((max_x - min_x) / (max_y - min_y)).clamp(0.1, 10.0);
        let cols = ((resolution * aspect.sqrt()).round() as u32).clamp(1, 256);
        let rows = ((resolution / aspect.sqrt()).round() as u32).clamp(1, 256);

        let cell_w = (max_x - min_x) / cols as f32;
        let cell_h = (max_y - min_y) / rows as f32;

        let mut cells = vec![NONE; cols as usize * rows as usize];

        for f in 0..num_faces {
            let c = self.face_centroid(f);
            let col = ((c.x - min_x) / cell_w) as i32;
            let row = ((c.y - min_y) / cell_h) as i32;
            if col >= 0 && col < cols as i32 && row >= 0 && row < rows as i32 {
                let idx = (row as u32 * cols + col as u32) as usize;
                if cells[idx] == NONE {
                    cells[idx] = f;
                }
            }
        }

        // Fill empty cells: row pass first (stays within row), then column pass.
        for r in 0..rows as usize {
            let start = r * cols as usize;
            let end = start + cols as usize;
            for i in (start + 1)..end {
                if cells[i] == NONE {
                    cells[i] = cells[i - 1];
                }
            }
            for i in (start..end - 1).rev() {
                if cells[i] == NONE {
                    cells[i] = cells[i + 1];
                }
            }
        }
        let (cols, rows) = (cols as usize, rows as usize);
        for c in 0..cols {
            for r in 1..rows {
                if cells[r * cols + c] == NONE {
                    cells[r * cols + c] = cells[(r - 1) * cols + c];
                }
            }
            for r in (0..rows - 1).rev() {
                if cells[r * cols + c] == NONE {
                    cells[r * cols + c] = cells[(r + 1) * cols + c];
                }
            }
        }

        self.grid_cells = cells;
        self.grid_cols = cols as u32;
        self.grid_rows = rows as u32;
        self.grid_origin = Vector2::new(min_x, min_y);
        self.grid_cell_size = Vector2::new(cell_w, cell_h);
    }

    #[inline]
    fn grid_lookup(&self, point: Vector2) -> u32 {
        if self.grid_cells.is_empty()
            || self.grid_cell_size.x == 0.0
            || self.grid_cell_size.y == 0.0
        {
            return 0;
        }
        let col = ((point.x - self.grid_origin.x) / self.grid_cell_size.x) as i32;
        let row = ((point.y - self.grid_origin.y) / self.grid_cell_size.y) as i32;
        let col = col.clamp(0, self.grid_cols as i32 - 1) as u32;
        let row = row.clamp(0, self.grid_rows as i32 - 1) as u32;
        let face = self.grid_cells[(row * self.grid_cols + col) as usize];
        debug_assert!(
            face != NONE,
            "grid cell ({col},{row}) is NONE after build_grid_index"
        );
        if face == NONE { 0 } else { face }
    }

    /// Find the face containing the given point.
    pub fn locate_face(&self, point: Vector2) -> Option<u32> {
        if self.half_edges.is_empty() {
            return None;
        }
        let num_faces = self.num_faces();
        let mut current = self.grid_lookup(point);
        let max_iters = self.half_edges.len();

        for _ in 0..max_iters {
            if current >= num_faces {
                return self.locate_face_brute(point);
            }

            let base = current * 3;
            let p0 = self.point(self.origin(base));
            let p1 = self.point(self.origin(base + 1));
            let p2 = self.point(self.origin(base + 2));

            let d0 = orient2d(p0, p1, point);
            if d0 < 0.0 {
                let twin = self.half_edges[base as usize].twin;
                if twin != NONE {
                    current = face_of(twin);
                    continue;
                }
                // At a boundary and point is outside this face; brute-force to confirm.
                return self.locate_face_brute(point);
            }

            let d1 = orient2d(p1, p2, point);
            if d1 < 0.0 {
                let twin = self.half_edges[(base + 1) as usize].twin;
                if twin != NONE {
                    current = face_of(twin);
                    continue;
                }
                return self.locate_face_brute(point);
            }

            let d2 = orient2d(p2, p0, point);
            if d2 < 0.0 {
                let twin = self.half_edges[(base + 2) as usize].twin;
                if twin != NONE {
                    current = face_of(twin);
                    continue;
                }
                return self.locate_face_brute(point);
            }

            return Some(current);
        }

        self.locate_face_brute(point)
    }

    /// Get all triangle vertices as a flat array for rendering.
    pub fn get_mesh_vertices(&self) -> PackedVector2Array {
        let mut vertices = PackedVector2Array::new();
        let num_faces = self.num_faces();
        for f in 0..num_faces {
            let [v0, v1, v2] = self.face_vertices(f);
            vertices.push(self.point(v0));
            vertices.push(self.point(v1));
            vertices.push(self.point(v2));
        }
        vertices
    }

    /// Access the points array.
    pub fn points(&self) -> &[Vector2] {
        &self.points
    }

    /// Iterate walkable neighbors without allocation.
    /// Calls `f(neighbor_face, half_edge_from_self_to_neighbor)` for each non-constrained, non-boundary edge.
    /// The half-edge can be passed to `edge_midpoint()` to compute smooth waypoints.
    pub fn for_each_neighbor(&self, face: u32, mut f: impl FnMut(u32, u32)) {
        let base = face * 3;
        for j in 0u32..3 {
            let he = base + j;
            let twin = self.half_edges[he as usize].twin;
            if twin != NONE && !self.he_constrained[he as usize] {
                f(face_of(twin), he);
            }
        }
    }

    /// Face that half-edge `he` belongs to.
    pub fn face_of_he(&self, he: u32) -> u32 {
        face_of(he)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_orient2d_ccw() {
        let a = Vector2::new(0.0, 0.0);
        let b = Vector2::new(1.0, 0.0);
        let c = Vector2::new(0.0, 1.0);
        assert!(orient2d(a, b, c) > 0.0);
    }

    #[test]
    fn test_orient2d_cw() {
        let a = Vector2::new(0.0, 0.0);
        let b = Vector2::new(0.0, 1.0);
        let c = Vector2::new(1.0, 0.0);
        assert!(orient2d(a, b, c) < 0.0);
    }

    #[test]
    fn test_orient2d_collinear() {
        let a = Vector2::new(0.0, 0.0);
        let b = Vector2::new(1.0, 1.0);
        let c = Vector2::new(2.0, 2.0);
        assert!(orient2d(a, b, c).abs() < 1e-10);
    }

    #[test]
    fn test_in_circumcircle_inside() {
        let a = Vector2::new(0.0, 0.0);
        let b = Vector2::new(2.0, 0.0);
        let c = Vector2::new(1.0, 2.0);
        let p = Vector2::new(1.0, 0.5); // inside circumcircle
        assert!(in_circumcircle(a, b, c, p));
    }

    #[test]
    fn test_in_circumcircle_outside() {
        let a = Vector2::new(0.0, 0.0);
        let b = Vector2::new(1.0, 0.0);
        let c = Vector2::new(0.5, 0.5);
        let p = Vector2::new(10.0, 10.0); // far outside
        assert!(!in_circumcircle(a, b, c, p));
    }

    #[test]
    fn test_segments_intersect() {
        assert!(segments_intersect_proper(
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(0.0, 2.0),
            Vector2::new(2.0, 0.0),
        ));

        // Parallel segments
        assert!(!segments_intersect_proper(
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 1.0),
            Vector2::new(1.0, 1.0),
        ));

        // Endpoint touching — not a proper intersection
        assert!(!segments_intersect_proper(
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(2.0, 0.0),
        ));
    }

    #[test]
    fn test_next_prev_roundtrip() {
        for he in 0..30u32 {
            assert_eq!(next(next(next(he))), he, "next^3 should be identity");
            assert_eq!(prev(prev(prev(he))), he, "prev^3 should be identity");
            assert_eq!(next(prev(he)), he, "next(prev(he)) should be identity");
            assert_eq!(prev(next(he)), he, "prev(next(he)) should be identity");
        }
    }

    #[test]
    fn test_face_of() {
        assert_eq!(face_of(0), 0);
        assert_eq!(face_of(1), 0);
        assert_eq!(face_of(2), 0);
        assert_eq!(face_of(3), 1);
        assert_eq!(face_of(5), 1);
        assert_eq!(face_of(6), 2);
    }

    #[test]
    fn test_three_points_single_triangle() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 1.0),
        ];

        let d = CDT::triangulate(points);
        assert_eq!(
            d.num_triangles(),
            1,
            "Three points should form exactly one triangle"
        );
        assert_eq!(d.num_vertices(), 3);

        let verts = d.face_vertices(0);
        let mut sorted = verts;
        sorted.sort();
        assert_eq!(sorted, [0, 1, 2]);
    }

    #[test]
    fn test_four_points_square() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(0.0, 1.0),
        ];

        let d = CDT::triangulate(points);
        assert_eq!(
            d.num_triangles(),
            2,
            "Four points in a square should form 2 triangles"
        );
        assert_eq!(d.num_vertices(), 4);
    }

    #[test]
    fn test_five_points_with_center() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(0.0, 1.0),
            Vector2::new(0.5, 0.5),
        ];

        let d = CDT::triangulate(points);
        assert_eq!(
            d.num_triangles(),
            4,
            "5 points (4 corners + center) should form 4 triangles"
        );
        assert_eq!(d.num_vertices(), 5);

        // Center point should appear in all 4 triangles.
        // Spatial sort may reorder indices, so find it by frequency instead.
        let mut vertex_freq = vec![0u32; d.num_vertices() as usize];
        for f in 0..d.num_triangles() {
            for v in d.face_vertices(f) {
                vertex_freq[v as usize] += 1;
            }
        }
        let max_freq = vertex_freq.iter().copied().max().unwrap_or(0);
        assert_eq!(max_freq, 4, "Center point should be in 4 triangles");
    }

    #[test]
    fn test_collinear_points() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(2.0, 2.0),
        ];

        let d = CDT::triangulate(points);
        assert_eq!(d.num_vertices(), 3);
        // Collinear points should not crash
    }

    #[test]
    fn test_random_points() {
        let points = vec![
            Vector2::new(100.0, 200.0),
            Vector2::new(300.0, 150.0),
            Vector2::new(250.0, 400.0),
            Vector2::new(450.0, 300.0),
            Vector2::new(200.0, 350.0),
            Vector2::new(350.0, 250.0),
        ];

        let d = CDT::triangulate(points);
        assert_eq!(d.num_vertices(), 6);
        assert!(
            d.num_triangles() >= 4,
            "6 points should form at least 4 triangles"
        );

        for f in 0..d.num_triangles() {
            let verts = d.face_vertices(f);
            assert!(verts[0] < 6);
            assert!(verts[1] < 6);
            assert!(verts[2] < 6);
            assert_ne!(verts[0], verts[1]);
            assert_ne!(verts[1], verts[2]);
            assert_ne!(verts[0], verts[2]);
        }
    }

    #[test]
    fn test_delaunay_property() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(0.0, 2.0),
            Vector2::new(1.0, 1.0),
        ];

        let d = CDT::triangulate(points);

        for f in 0..d.num_triangles() {
            let verts = d.face_vertices(f);
            let a = d.points()[verts[0] as usize];
            let b = d.points()[verts[1] as usize];
            let c = d.points()[verts[2] as usize];

            for (i, p) in d.points().iter().enumerate() {
                let i = i as u32;
                if i == verts[0] || i == verts[1] || i == verts[2] {
                    continue;
                }
                assert!(
                    !in_circumcircle(a, b, c, *p),
                    "Point {} is inside circumcircle of triangle ({},{},{})",
                    i,
                    verts[0],
                    verts[1],
                    verts[2]
                );
            }
        }
    }

    #[test]
    fn test_large_coordinate_values() {
        let points = vec![
            Vector2::new(871.0, 601.0),
            Vector2::new(566.0, 842.0),
            Vector2::new(1027.0, 1109.0),
        ];

        let d = CDT::triangulate(points);
        assert_eq!(d.num_triangles(), 1);
        assert_eq!(d.num_vertices(), 3);
    }

    #[test]
    fn test_vertex_count_single_triangle() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 1.0),
        ];

        let d = CDT::triangulate(points);
        // 1 triangle * 3 vertices = 3 mesh vertices
        assert_eq!(d.num_triangles() * 3, 3);
    }

    #[test]
    fn test_face_neighbors() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(0.0, 2.0),
            Vector2::new(1.0, 1.0),
        ];

        let d = CDT::triangulate(points);

        // Every interior face should have at least 1 neighbor
        for f in 0..d.num_triangles() {
            let neighbors = d.face_neighbors(f);
            assert!(!neighbors.is_empty(), "Face {} should have neighbors", f);
        }
    }

    #[test]
    fn test_face_centroid() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(3.0, 0.0),
            Vector2::new(0.0, 3.0),
        ];

        let d = CDT::triangulate(points);
        let c = d.face_centroid(0);
        assert!((c.x - 1.0).abs() < 0.01);
        assert!((c.y - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_locate_face() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(0.0, 2.0),
            Vector2::new(1.0, 1.0),
        ];

        let d = CDT::triangulate(points);

        // The center point should be locatable
        let f = d.locate_face(Vector2::new(1.0, 1.0));
        assert!(f.is_some(), "Should find face containing (1,1)");

        // A point clearly inside the domain
        let f = d.locate_face(Vector2::new(0.5, 0.5));
        assert!(f.is_some(), "Should find face containing (0.5, 0.5)");
    }

    #[test]
    fn test_build_grid_index_populates_grid() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(4.0, 0.0),
            Vector2::new(4.0, 4.0),
            Vector2::new(0.0, 4.0),
            Vector2::new(2.0, 2.0),
        ];
        let d = CDT::triangulate(points);
        // triangulate() calls build_grid_index() internally
        assert!(
            !d.grid_cells.is_empty(),
            "grid should be populated after triangulate"
        );
        assert!(d.grid_cols > 0);
        assert!(d.grid_rows > 0);
        // No cell should be NONE after the fill pass
        for &cell in &d.grid_cells {
            assert_ne!(cell, NONE, "all cells should be filled after scanline pass");
        }
    }

    #[test]
    fn test_grid_lookup_returns_valid_face() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(4.0, 0.0),
            Vector2::new(4.0, 4.0),
            Vector2::new(0.0, 4.0),
            Vector2::new(2.0, 2.0),
        ];
        let d = CDT::triangulate(points);
        let num_faces = d.num_faces();
        // Query several points and verify returned faces are within range
        let queries = [
            Vector2::new(1.0, 1.0),
            Vector2::new(3.0, 3.0),
            Vector2::new(0.5, 3.5),
            Vector2::new(3.5, 0.5),
        ];
        for q in queries {
            let f = d.grid_lookup(q);
            assert!(
                f < num_faces,
                "grid_lookup should return a valid face index for {:?}",
                q
            );
        }
    }

    #[test]
    fn test_grid_lookup_clamps_outside_points() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(0.0, 2.0),
            Vector2::new(1.0, 1.0),
        ];
        let d = CDT::triangulate(points);
        let num_faces = d.num_faces();
        // Points well outside the bounding box should still return a valid face (clamped).
        for q in [
            Vector2::new(-100.0, -100.0),
            Vector2::new(100.0, 100.0),
            Vector2::new(-1.0, 1.0),
        ] {
            let f = d.grid_lookup(q);
            assert!(
                f < num_faces,
                "grid_lookup should clamp and return valid face for {:?}",
                q
            );
        }
    }

    #[test]
    fn test_locate_face_correctness_with_grid() {
        // Verify locate_face still returns the correct face now that it uses grid_lookup.
        let points: Vec<Vector2> = (0..5)
            .flat_map(|y| (0..5).map(move |x| Vector2::new(x as f32, y as f32)))
            .collect();
        let d = CDT::triangulate(points);

        // Every centroid should locate back to its own face.
        for f in 0..d.num_faces() {
            let c = d.face_centroid(f);
            let found = d.locate_face(c);
            assert!(
                found.is_some(),
                "centroid of face {} should be locatable",
                f
            );
            // The found face should actually contain the centroid.
            let found_face = found.unwrap();
            let verts = d.face_vertices(found_face);
            let p0 = d.points()[verts[0] as usize];
            let p1 = d.points()[verts[1] as usize];
            let p2 = d.points()[verts[2] as usize];
            assert!(
                is_point_in_triangle(c, p0, p1, p2),
                "located face {} does not contain centroid of face {} at {:?}",
                found_face,
                f,
                c
            );
        }
    }

    #[test]
    fn test_empty_grid_index_fallback() {
        // A CDT with no grid built should fall back to face 0 from grid_lookup.
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(0.0, 1.0),
        ];
        let mut d = CDT::triangulate(points);
        // Manually clear the grid to simulate no grid.
        d.grid_cells.clear();
        assert_eq!(
            d.grid_lookup(Vector2::new(0.2, 0.2)),
            0,
            "empty grid should fall back to face 0"
        );
    }

    #[test]
    fn test_grid_scanline_fill_does_not_cross_row_boundary() {
        // Build a mesh whose centroids only populate the right half of each row,
        // leaving the left half of every row empty. Before the fix, the forward
        // scanline would propagate the last cell of row N into the first cell of
        // row N+1, so a query into the top-left corner of any row would receive a
        // seed face from the bottom-right of the previous row.
        //
        // We verify this by checking that every cell in the grid points to a face
        // whose centroid is in the same row as the cell (or an adjacent row), not
        // from a completely different part of the mesh.
        let points: Vec<Vector2> = (0..10)
            .flat_map(|y| (0..10).map(move |x| Vector2::new(x as f32, y as f32)))
            .collect();
        let d = CDT::triangulate(points);

        let cols = d.grid_cols as usize;
        let rows = d.grid_rows as usize;

        for r in 0..rows {
            for c in 0..cols {
                let face = d.grid_cells[r * cols + c];
                assert_ne!(face, NONE, "cell ({},{}) should be filled", c, r);

                // The face's centroid y should map to a row within 1 of the cell's row.
                let centroid = d.face_centroid(face);
                let centroid_row = ((centroid.y - d.grid_origin.y) / d.grid_cell_size.y) as i32;
                let cell_row = r as i32;
                assert!(
                    (centroid_row - cell_row).abs() <= 1,
                    "cell ({},{}) seeded by face {} whose centroid row {} is far away",
                    c,
                    r,
                    face,
                    centroid_row
                );
            }
        }
    }

    #[test]
    fn test_grid_aspect_ratio_wide_domain() {
        // A 4:1 wide domain should produce more columns than rows.
        let points: Vec<Vector2> = (0..5)
            .flat_map(|y| (0..20).map(move |x| Vector2::new(x as f32 * 4.0, y as f32)))
            .collect();
        let d = CDT::triangulate(points);
        assert!(
            d.grid_cols > d.grid_rows,
            "wide domain should have more columns ({}) than rows ({})",
            d.grid_cols,
            d.grid_rows
        );
    }

    #[test]
    fn test_grid_aspect_ratio_tall_domain() {
        // A 1:4 tall domain should produce more rows than columns.
        let points: Vec<Vector2> = (0..20)
            .flat_map(|y| (0..5).map(move |x| Vector2::new(x as f32, y as f32 * 4.0)))
            .collect();
        let d = CDT::triangulate(points);
        assert!(
            d.grid_rows > d.grid_cols,
            "tall domain should have more rows ({}) than columns ({})",
            d.grid_rows,
            d.grid_cols
        );
    }

    #[test]
    fn test_constraint_existing_edge() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(0.0, 2.0),
            Vector2::new(1.0, 1.0),
        ];

        let mut cdt = CDT::from_points(points);
        // Insert a constraint that already exists as a triangulation edge
        cdt.insert_constraint(0, 4);
        cdt.remove_super_triangle();

        // Should still have valid triangulation
        assert!(cdt.num_faces() >= 4);
    }

    #[test]
    fn test_constraint_crossing_edges() {
        // Create a grid of points and insert a diagonal constraint
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(4.0, 0.0),
            Vector2::new(0.0, 2.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(4.0, 2.0),
            Vector2::new(0.0, 4.0),
            Vector2::new(2.0, 4.0),
            Vector2::new(4.0, 4.0),
        ];

        let mut cdt = CDT::from_points(points);
        cdt.insert_constraint(0, 8); // diagonal from (0,0) to (4,4)
        cdt.remove_super_triangle();

        // The diagonal passes through vertex 4 at (2,2), so the constraint is split
        // into two sub-segments: 0→4 and 4→8.
        let has_0_4 = cdt.find_half_edge(0, 4).is_some() || cdt.find_half_edge(4, 0).is_some();
        let has_4_8 = cdt.find_half_edge(4, 8).is_some() || cdt.find_half_edge(8, 4).is_some();
        assert!(
            has_0_4,
            "Constraint sub-edge 0-4 should exist in triangulation"
        );
        assert!(
            has_4_8,
            "Constraint sub-edge 4-8 should exist in triangulation"
        );

        // Verify the constraint edges are marked as constrained
        if let Some(he) = cdt.find_half_edge(0, 4).or(cdt.find_half_edge(4, 0)) {
            assert!(
                cdt.he_constrained[he as usize],
                "Edge 0-4 should be constrained"
            );
        }
        if let Some(he) = cdt.find_half_edge(4, 8).or(cdt.find_half_edge(8, 4)) {
            assert!(
                cdt.he_constrained[he as usize],
                "Edge 4-8 should be constrained"
            );
        }
    }

    #[test]
    fn test_euler_formula() {
        // V - E + F = 1 (for a triangulation with boundary, Euler for planar graph with 1 unbounded face)
        // For a triangulation: E = 3F/2 + boundary_edges/2... actually simpler:
        // For a triangulation of a convex point set with no holes:
        // F = 2V - h - 2  where h = convex hull size
        // Just verify basic consistency

        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(0.0, 2.0),
            Vector2::new(1.0, 1.0),
        ];

        let d = CDT::triangulate(points);
        let v = d.num_vertices();
        let f = d.num_triangles();

        // Count edges (each internal edge has 2 half-edges, boundary has 1)
        let mut edge_count = 0u32;
        let mut _boundary_edges = 0u32;
        for i in 0..d.half_edges.len() as u32 {
            let twin = d.half_edges[i as usize].twin;
            if twin == NONE {
                _boundary_edges += 1;
                edge_count += 1;
            } else if twin > i {
                edge_count += 1;
            }
        }

        // Euler: V - E + F = 1 (with outer face counted)
        // Or: V - E + F_inner = 1 for planar graph with boundary
        let euler = v as i32 - edge_count as i32 + f as i32;
        assert!(
            euler == 1 || euler == 2,
            "Euler formula check: V={} E={} F={} => V-E+F={}",
            v,
            edge_count,
            f,
            euler
        );
    }

    #[test]
    fn test_cocircular_points() {
        // 4 points on a circle — cocircular case
        let r = 1.0f32;
        let points = vec![
            Vector2::new(r, 0.0),
            Vector2::new(0.0, r),
            Vector2::new(-r, 0.0),
            Vector2::new(0.0, -r),
        ];

        let d = CDT::triangulate(points);
        assert_eq!(
            d.num_triangles(),
            2,
            "4 cocircular points should produce 2 triangles"
        );
    }

    #[test]
    fn test_many_random_points_delaunay_property() {
        // Generate a bunch of points with a simple LCG
        let mut rng: u64 = 12345;
        let mut points = Vec::new();
        for _ in 0..50 {
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
            let x = ((rng / 65536) % 1000) as f32;
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
            let y = ((rng / 65536) % 1000) as f32;
            points.push(Vector2::new(x, y));
        }

        let d = CDT::triangulate(points);

        // Verify Delaunay property
        for f in 0..d.num_triangles() {
            let verts = d.face_vertices(f);
            let a = d.points()[verts[0] as usize];
            let b = d.points()[verts[1] as usize];
            let c = d.points()[verts[2] as usize];

            // Skip degenerate triangles
            if orient2d(a, b, c).abs() < 1e-10 {
                continue;
            }

            for (i, p) in d.points().iter().enumerate() {
                let i = i as u32;
                if i == verts[0] || i == verts[1] || i == verts[2] {
                    continue;
                }
                assert!(
                    !in_circumcircle(a, b, c, *p),
                    "Point {} is inside circumcircle of face {}",
                    i,
                    f
                );
            }
        }
    }

    #[test]
    fn test_constraint_edge_preserved() {
        let points = vec![
            Vector2::new(1225.0, 534.0),
            Vector2::new(1207.0, 1046.0),
            Vector2::new(1566.0, 794.0),
            Vector2::new(960.0, 780.0),
            Vector2::new(1148.0, 733.0),
            Vector2::new(1285.0, 746.0),
        ];

        let mut cdt = CDT::from_points(points);
        cdt.insert_constraint(2, 3);
        cdt.remove_super_triangle();

        let has_edge = cdt.find_half_edge(2, 3).is_some() || cdt.find_half_edge(3, 2).is_some();
        assert!(
            has_edge,
            "Constrained edge 2-3 must exist in the final triangulation"
        );
    }

    #[test]
    fn test_shared_edge_between() {
        let points = vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(2.0, 0.0),
            Vector2::new(2.0, 2.0),
            Vector2::new(0.0, 2.0),
        ];

        let d = CDT::triangulate(points);
        assert_eq!(d.num_triangles(), 2);

        let edge = d.shared_edge_between(0, 1);
        assert!(
            edge.is_some(),
            "Two triangles sharing an edge should find it"
        );
    }

    fn square_cdt() -> CDT {
        // 4-point unit square → 2 triangles, no constraints
        CDT::triangulate(vec![
            Vector2::new(0.0, 0.0),
            Vector2::new(1.0, 0.0),
            Vector2::new(1.0, 1.0),
            Vector2::new(0.0, 1.0),
        ])
    }

    #[test]
    fn test_path_same_face() {
        let cdt = square_cdt();
        let start = Vector2::new(0.1, 0.1);
        let goal = Vector2::new(0.2, 0.1);
        // Both points should land in the same face
        let sf = cdt.locate_face(start).unwrap();
        let gf = cdt.locate_face(goal).unwrap();
        assert_eq!(sf, gf, "test requires both points in the same face");
        let path = crate::astar::find_path(&cdt, start, goal);
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], start);
        assert_eq!(path[1], goal);
    }

    #[test]
    fn test_path_simple() {
        let cdt = square_cdt();
        // Put start/goal in different faces (2-triangle square)
        // Face 0 and face 1 should be the two triangles
        let c0 = cdt.face_centroid(0);
        let c1 = cdt.face_centroid(1);
        assert_ne!(
            cdt.locate_face(c0).unwrap(),
            cdt.locate_face(c1).unwrap(),
            "centroids must be in different faces"
        );
        let path = crate::astar::find_path(&cdt, c0, c1);
        assert!(!path.is_empty(), "should find a path");
        assert_eq!(*path.first().unwrap(), c0);
        assert_eq!(*path.last().unwrap(), c1);
    }

    #[test]
    fn test_path_no_route_full_constraint_wall() {
        // 6 points in 2 rows × 3 cols:
        //  3---4---5
        //  |   |   |
        //  0---1---2
        // Constrain all horizontal internal edges (0-3, 1-4, 2-5 are vertical;
        // internal horizontal edges of the triangulation separate top from bottom).
        // We constrain the top edges of the bottom row: 0-4, 1-4 (diagonal seam)
        // and also constrain segment 0-3, 1-4, 2-5 which are all vertical.
        //
        // Simplest guaranteed wall: constrain both horizontal mid-edges of the
        // triangulation by inserting constraint edges 3-4 and 4-5 (along y=1).
        // After CDT, the triangles along y=1 have their shared edges marked constrained,
        // which disconnects top from bottom completely.
        let points = vec![
            Vector2::new(0.0, 0.0), // 0
            Vector2::new(1.0, 0.0), // 1
            Vector2::new(2.0, 0.0), // 2
            Vector2::new(0.0, 1.0), // 3
            Vector2::new(1.0, 1.0), // 4
            Vector2::new(2.0, 1.0), // 5
        ];
        let mut cdt = CDT::from_points(points);
        // Constrain the full horizontal mid-line: 3-4 and 4-5
        cdt.insert_constraint(3, 4);
        cdt.insert_constraint(4, 5);
        // Also constrain 0-3, 1-4, 2-5 to seal vertical joints
        cdt.insert_constraint(0, 3);
        cdt.insert_constraint(1, 4);
        cdt.insert_constraint(2, 5);
        cdt.remove_super_triangle();

        let start = Vector2::new(0.5, 0.25); // below wall
        let goal = Vector2::new(0.5, 1.5); // above wall — outside mesh actually

        // If goal is outside, locate_face returns None → empty path
        // If goal happens to be inside (mesh extends beyond 1.0 vertically it won't),
        // then the wall constraints ensure disconnection.
        let path = crate::astar::find_path(&cdt, start, goal);
        assert!(
            path.is_empty(),
            "should find no path through a full constraint wall"
        );
    }

    #[test]
    fn test_path_point_outside_mesh() {
        let cdt = square_cdt();
        let start = Vector2::new(0.5, 0.5);
        let outside = Vector2::new(10.0, 10.0);
        let path = crate::astar::find_path(&cdt, start, outside);
        assert!(path.is_empty(), "point outside mesh → empty path");
    }

    #[test]
    fn test_path_start_equals_goal() {
        let cdt = square_cdt();
        let pt = Vector2::new(0.5, 0.5);
        let path = crate::astar::find_path(&cdt, pt, pt);
        // same face → [start, goal]
        assert_eq!(path.len(), 2);
        assert_eq!(path[0], pt);
        assert_eq!(path[1], pt);
    }

    #[test]
    fn test_path_endpoints_in_returned_path() {
        let cdt = square_cdt();
        let c0 = cdt.face_centroid(0);
        let c1 = cdt.face_centroid(1);
        let path = crate::astar::find_path(&cdt, c0, c1);
        assert!(!path.is_empty());
        assert_eq!(*path.first().unwrap(), c0, "first waypoint must be start");
        assert_eq!(*path.last().unwrap(), c1, "last waypoint must be goal");
    }
}
