use godot::prelude::*;

mod delaunay;
mod simulation;

pub struct RTS;

#[gdextension]
unsafe impl ExtensionLibrary for RTS {}
