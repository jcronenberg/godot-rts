#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

# Editor/debug Godot binaries load the linux.debug gdextension path, so stage
# the release build there for the run (original restored on exit).
cargo build --release --manifest-path rust/Cargo.toml
debug_lib=rust/target/debug/librts.so
release_lib=rust/target/release/librts.so
mkdir -p rust/target/debug
if [[ -f "$debug_lib" ]]; then
    mv "$debug_lib" "$debug_lib.bench-bak"
    trap 'mv "$debug_lib.bench-bak" "$debug_lib"' EXIT
else
    trap 'rm -f "$debug_lib"' EXIT
fi
cp "$release_lib" "$debug_lib"

"${GODOT_BIN:-godot}" --headless --path . --script benches/bench_runner.gd
