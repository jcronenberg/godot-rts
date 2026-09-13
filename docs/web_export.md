# Web export

How to build and export this project (Godot 4.7 + Rust GDExtension) to the browser.

## Prerequisites

| Tool | Version | Why |
| --- | --- | --- |
| Rust | nightly | `-Zbuild-std` and the `-Z` linker flags in `.cargo/config.toml`. |
| emsdk | match the export template | Godot 4.7.stable's official template reports **Emscripten 4.0.20**. |
| Godot | 4.7 + web export templates | Editor > Manage Export Templates. |

```bash
rustup toolchain install nightly
rustup component add rust-src --toolchain nightly
rustup target add wasm32-unknown-emscripten --toolchain nightly
```

The authoritative emsdk number is printed by the running game itself, in the
browser console: `Build configuration: Emscripten <version>, ...`. Trust that
over Godot's CI workflow files, which have disagreed with the shipped template.
Patch versions within 4.0.x interoperate fine.

In a clone of emsdk do:
```bash
./emsdk install $VERSION
./emsdk activate $VERSION
source emsdk_env.sh
```

## What is already configured

| File | Setting | Why |
| --- | --- | --- |
| `project.godot` | `rendering_method="gl_compatibility"` | Forward+ needs Vulkan; the web has WebGL2. Costs nothing here, the game is 2D with no custom shaders. |
| `rts/Cargo.toml` | `experimental-wasm`, `experimental-wasm-nothreads` | Web support, single-threaded variant. |
| `.cargo/config.toml` | `-sSIDE_MODULE=2`, `panic=abort` | The extension is dylink'd into Godot's wasm module. See "panic=abort" below. |
| `rts.gdextension` | `web.{debug,release}.wasm32` | Where Godot looks for the `.wasm`. |
| `export_presets.cfg` | `variant/extensions_support=true` | **Off by default.** Without it the extension is silently not shipped. |
| `export_presets.cfg` | `variant/thread_support=false` | Matches the `-nothreads` build. |

### No threads

The browser build has no sim thread. `SimHandle::pump` (`rts-lib/src/sim_runner.rs`)
steps the sim inline from `Simulation::poll` once per frame, paying off tick debt
with a catch-up cap. `run_loop` and `pump` share `tick_once`, so the tick body is
identical either way; only pacing differs.

Consequence: no `SharedArrayBuffer`, so **no COOP/COEP headers needed**. Any static
host works, `python3 -m http.server` included.

### panic=abort

rustc 1.98 made wasm exception handling unconditional on emscripten and removed
`-Zemscripten-wasm-eh`. Godot's template predates wasm EH, so an unwinding
extension fails to load with `LinkError: '__cpp_exception' is not a Tag`. An
extension that never unwinds never imports that tag.

The cost: gdext normally catches panics and reports them as Godot errors. Here a
panic aborts the wasm instance and the page needs a reload. Scoped to this target,
so native builds keep unwinding. Guard degenerate input rather than relying on
being caught (see `Simulation::load_map`).

## Build and export

```bash
cargo +nightly build -Zbuild-std=std,panic_abort \
  --target wasm32-unknown-emscripten --release

mkdir -p build/web
godot --headless --export-release "Web" build/web/index.html

cd build/web && python3 -m http.server 8000
```
