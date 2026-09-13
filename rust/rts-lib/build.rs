//! Restores `<target-dir>/.gdignore`, which keeps Godot from indexing and
//! exporting the build artifacts.
//!
//! The guard lives inside the directory `cargo clean` deletes, so re-arming it
//! from the build that refills that directory is the one place a clean cannot
//! undo.

use std::path::{Path, PathBuf};

fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    let Some(target) = target_dir() else {
        return;
    };
    let guard = target.join(".gdignore");
    if !guard.exists() {
        // Best effort: not worth failing an otherwise good build.
        let _ = std::fs::write(&guard, "");
    }
}

/// Target root: the `OUT_DIR` ancestor holding cargo's `CACHEDIR.TAG`. By
/// marker, not by level count, since `--target` nests a triple directory in.
fn target_dir() -> Option<PathBuf> {
    let out = PathBuf::from(std::env::var_os("OUT_DIR")?);
    out.ancestors()
        .find(|dir| dir.join("CACHEDIR.TAG").exists())
        .map(Path::to_path_buf)
}
