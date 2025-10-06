use godot::prelude::*;

mod delaunay;

pub struct RTS;

#[gdextension]
unsafe impl ExtensionLibrary for RTS {}
