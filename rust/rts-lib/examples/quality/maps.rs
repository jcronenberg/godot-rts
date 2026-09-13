//! Scenario geometry, returning the crate's `(points, constraints)` map pair.
//!
//! The `test_maps/*.json` loader is a local copy of `test_utils::load_raw`
//! (`#[cfg(test)] pub(crate)`). Reaching the real one would put `serde` in
//! `[dependencies]` and into the shipped `cdylib`, a poor trade for 15 lines.

use std::collections::BTreeMap;

use godot::prelude::Vector2;

pub fn v(x: f32, y: f32) -> Vector2 {
    Vector2::new(x, y)
}

/// Accumulates wall segments into a deduplicated point set.
#[derive(Default)]
pub struct MapBuilder {
    points: Vec<Vector2>,
    index: BTreeMap<(u32, u32), u32>,
    constraints: Vec<(u32, u32)>,
}

impl MapBuilder {
    pub fn new() -> MapBuilder {
        MapBuilder::default()
    }

    fn id(&mut self, p: Vector2) -> u32 {
        *self.index.entry((p.x.to_bits(), p.y.to_bits())).or_insert_with(|| {
            self.points.push(p);
            (self.points.len() - 1) as u32
        })
    }

    pub fn seg(&mut self, a: Vector2, b: Vector2) -> &mut Self {
        let (a, b) = (self.id(a), self.id(b));
        self.constraints.push((a, b));
        self
    }

    /// Closed axis-aligned rectangle.
    pub fn rect(&mut self, x0: f32, y0: f32, x1: f32, y1: f32) -> &mut Self {
        self.seg(v(x0, y0), v(x1, y0))
            .seg(v(x1, y0), v(x1, y1))
            .seg(v(x1, y1), v(x0, y1))
            .seg(v(x0, y1), v(x0, y0))
    }

    pub fn finish(&self) -> (Vec<Vector2>, Vec<(u32, u32)>) {
        (self.points.clone(), self.constraints.clone())
    }
}

/// Plain rectangular arena.
pub fn box_map(w: f32, h: f32) -> (Vec<Vector2>, Vec<(u32, u32)>) {
    let mut b = MapBuilder::new();
    b.rect(0.0, 0.0, w, h);
    b.finish()
}

/// Two rooms of `600 x 400` joined by one `door`-wide doorway at mid-height,
/// which everything has to go through.
pub fn two_rooms(door: f32) -> (Vec<Vector2>, Vec<(u32, u32)>) {
    let (w, h) = (600.0, 400.0);
    let (lo, hi) = (h * 0.5 - door * 0.5, h * 0.5 + door * 0.5);
    let mut b = MapBuilder::new();
    b.rect(0.0, 0.0, w, h)
        .seg(v(300.0, 0.0), v(300.0, lo))
        .seg(v(300.0, hi), v(300.0, h));
    b.finish()
}

/// Two rooms joined by a `200 x width` corridor, wide enough for several
/// abreast, so two groups meeting inside must resolve rather than queue.
pub fn corridor(width: f32) -> (Vec<Vector2>, Vec<(u32, u32)>) {
    let (w, h) = (700.0, 300.0);
    let (lo, hi) = (h * 0.5 - width * 0.5, h * 0.5 + width * 0.5);
    let mut b = MapBuilder::new();
    b.rect(0.0, 0.0, w, h)
        .seg(v(250.0, 0.0), v(250.0, lo))
        .seg(v(250.0, hi), v(250.0, h))
        .seg(v(450.0, 0.0), v(450.0, lo))
        .seg(v(450.0, hi), v(450.0, h))
        .seg(v(250.0, lo), v(450.0, lo))
        .seg(v(250.0, hi), v(450.0, hi));
    b.finish()
}

/// A 200x200 box with a 3px-thick wall leaving a `gap`-wide slot to the
/// bottom boundary.
pub fn thin_wall(gap: f32) -> (Vec<Vector2>, Vec<(u32, u32)>) {
    let points = vec![
        v(0.0, 0.0),
        v(200.0, 0.0),
        v(200.0, 200.0),
        v(0.0, 200.0),
        v(150.0, gap),
        v(153.0, gap),
        v(153.0, 200.0),
        v(150.0, 200.0),
    ];
    let constraints = vec![
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
    ];
    (points, constraints)
}

/// Rectangle as a standalone obstacle polygon, for `Command::AddObstacle`.
pub fn rect_obstacle(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<Vector2> {
    vec![v(x0, y0), v(x1, y0), v(x1, y1), v(x0, y1)]
}

/// Consecutive-pair wall segments of a closed polygon.
pub fn polygon_walls(poly: &[Vector2]) -> Vec<(Vector2, Vector2)> {
    (0..poly.len())
        .map(|i| (poly[i], poly[(i + 1) % poly.len()]))
        .collect()
}

#[derive(serde::Deserialize)]
struct MapData {
    points: Vec<[f32; 2]>,
    constraints: Vec<u32>,
}

/// Load `test_maps/<name>.json`, the same fixtures the crate's own tests use.
pub fn load_test_map(name: &str) -> (Vec<Vector2>, Vec<(u32, u32)>) {
    let path = format!("{}/../test_maps/{name}.json", env!("CARGO_MANIFEST_DIR"));
    let text =
        std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"));
    let data: MapData =
        serde_json::from_str(&text).unwrap_or_else(|e| panic!("cannot parse {path}: {e}"));
    let points = data.points.iter().map(|&[x, y]| v(x, y)).collect();
    let (pairs, rest) = data.constraints.as_chunks::<2>();
    assert!(rest.is_empty(), "odd constraint list in {path}");
    (points, pairs.iter().map(|&[a, b]| (a, b)).collect())
}
