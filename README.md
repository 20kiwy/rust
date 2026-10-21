# rust-roadgen — two road-generation engines joined in one pure-Rust crate

RoadArchitect (Unity C#, MIT) and godot-road-generator (GDScript, MIT) both
core-ported to Rust, rendered by a WebGL2 viewer written in Rust.

**The entire app is Rust.** There is no TypeScript and no hand-written
JavaScript — the only non-Rust file is `crates/viewer/index.html` (an HTML
shell + CSS that the Rust code drives through `wasm-bindgen`/`web-sys`).

| Crate | What it ports |
|---|---|
| [`crates/roadgen`](crates/roadgen/src/arch.rs) | `arch` — MicroGSD/RoadArchitect: Hermite/Catmull-Rom spline (tension 0.5, per-node EaseIO), ribbon mesh with shoulders + lane markings |
| [`crates/roadgen`](crates/roadgen/src/godot.rs) | `godot` — TheDuckCow/godot-road-generator: RoadPoint / RoadSegment / RoadManager (Bézier handles, density loop cuts, 8-column trimsheet UVs, gutters, underside, per-lane paths) **+ junctions** |
| [`crates/viewer`](crates/viewer/src/app.rs) | WebGL2 renderer (procedural GLSL trimsheet), orbit camera, node drag / add / edit, full RoadPoint inspector, both engines switchable live |

## Junctions (intersections)

Roads whose RoadPoints share coordinates (RoadManager proximity parity)
form junctions: approaches trim back when
`generate_mesh_at_intersections` is on (extended by
`edge_edge_clipping_offset`), a chamfered pavement n-gon fills the gap, and
every arm gets a stop line + zebra crosswalk. Try the **4-Way / T-Junction /
Town** presets.

## Build

```bash
rustup target add wasm32-unknown-unknown
cargo install trunk
trunk build --release          # → dist/
trunk serve                    # dev server
```

## Live preview

<https://20kiwy.github.io/rust/> — deployed by GitHub Actions
(`.github/workflows/pages.yml`) on every push to `main`.

Full property/feature parity inventory for the godot port:
[docs/godot-road-parity.md](docs/godot-road-parity.md)
