//! roadgen — two road-generation cores joined in one pure-Rust crate.
//!
//! * [`arch`]  — faithful port of MicroGSD/RoadArchitect's core spline math
//!   (Hermite/Catmull-Rom, tension 0.5, per-node EaseIO, ribbon mesh with
//!   shoulders + lane markings).
//! * [`godot`] — faithful port of TheDuckCow/godot-road-generator's core
//!   geometry (RoadPoint / RoadSegment / RoadManager model: Bézier handles,
//!   density loop cuts, 8-column trimsheet UVs, gutters, underside, lanes).
//!
//! The viewer crate drives both directly — no TypeScript, no JSON, no glue.

pub mod arch;
pub mod godot;

pub const VERSION: &str =
    "rust-roadgen 3.0 (joined: RoadArchitect hermite + godot-road-generator Bézier)";
