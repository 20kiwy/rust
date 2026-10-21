//! viewer — pure-Rust WebGL2 preview for the joined `roadgen` cores.
//! Built with trunk; the only non-Rust file in the whole app is index.html.

mod app;
mod gl;
mod math;
mod shaders;

use wasm_bindgen::prelude::*;

#[wasm_bindgen(start)]
pub fn main() -> Result<(), JsValue> {
    app::start()
}
