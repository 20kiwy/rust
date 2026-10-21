//! Pure-Rust viewer application: WebGL2 rendering, orbit camera, node
//! editing, and direct calls into the joined `roadgen` cores.

use crate::gl::{dom_canvas, interleave, sphere_geometry, LineMesh, Program, SurfaceMesh, Gl};
use crate::math::{project, Orbit};
use crate::shaders;
use roadgen::{arch, godot};
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::closure::Closure;
use wasm_bindgen::{JsCast, JsValue};
use web_sys::*;

const SKY: [f32; 3] = [0.847, 0.867, 0.827];
const AMBER: [f32; 3] = [0.96, 0.62, 0.04];
const EMERALD: [f32; 3] = [0.06, 0.72, 0.51];
const GREY_LINE: [f32; 3] = [0.42, 0.44, 0.42];
const WHITE_L: [f32; 3] = [0.92, 0.92, 0.9];
const AMBER_L: [f32; 3] = [1.0, 0.82, 0.4];
const TEAL: [f32; 3] = [0.02, 0.66, 0.5];
const ROSE: [f32; 3] = [0.94, 0.28, 0.42];

enum Drag {
    Orbit { x: f64, y: f64 },
    Node(usize),
}

type LaneLine = (u8, LineMesh);

pub struct App {
    gl: Gl,
    canvas: HtmlCanvasElement,
    perf: Performance,
    surf: Program,
    line: Program,
    ground: SurfaceMesh,
    road: SurfaceMesh,
    markings: SurfaceMesh,
    handle: SurfaceMesh,
    edge_c: LineMesh,
    edge_f: LineMesh,
    edge_r: LineMesh,
    center: LineMesh,
    lanes: Vec<LaneLine>,
    ctrl: LineMesh,
    mode: u8, // 0 = godot (Bézier), 1 = RoadArchitect (Hermite)
    g_points: Vec<godot::RoadPointData>,
    g_density: f32,
    g_under: f32,
    g_lowpoly: bool,
    g_auto: bool,
    a_nodes: Vec<arch::Node>,
    a_width: f32,
    a_shoulder: f32,
    a_spm: f32,
    a_mark: bool,
    a_shoulder_on: bool,
    show_edges: bool,
    show_lanes: bool,
    show_center: bool,
    selected: Option<usize>,
    cam: Orbit,
    drag: Option<Drag>,
    closures: Vec<Closure<dyn FnMut(JsValue)>>,
}

// ---------------------------------------------------------------------------
// Presets — (name, [(x, y, z)...], traffic_dir per point)
// ---------------------------------------------------------------------------

fn godot_presets() -> Vec<(&'static str, Vec<[f32; 3]>, Vec<u8>)> {
    vec![
        (
            "S-Curve",
            vec![
                [-70.0, 0.0, -55.0],
                [-30.0, 2.0, -18.0],
                [0.0, 0.0, 12.0],
                [32.0, 4.0, 18.0],
                [72.0, 0.0, 48.0],
            ],
            vec![2, 2, 1, 1],
        ),
        (
            "2-Lane",
            vec![
                [-60.0, 0.0, -50.0],
                [-15.0, 0.0, -10.0],
                [25.0, 1.0, 8.0],
                [65.0, 0.0, 40.0],
            ],
            vec![2, 1],
        ),
        (
            "Highway",
            vec![
                [-80.0, 0.0, -60.0],
                [-30.0, 0.0, -25.0],
                [0.0, 0.0, 5.0],
                [35.0, 0.0, 15.0],
                [85.0, 0.0, 50.0],
            ],
            vec![2, 2, 2, 1, 1, 1],
        ),
        (
            "Switchback",
            vec![
                [-70.0, 0.0, -60.0],
                [-10.0, 1.0, -25.0],
                [60.0, 2.0, -20.0],
                [5.0, 3.0, 12.0],
                [-55.0, 4.0, 18.0],
                [10.0, 5.0, 50.0],
                [70.0, 4.0, 55.0],
            ],
            vec![2, 1],
        ),
        (
            "Loop",
            vec![
                [0.0, 0.0, -55.0],
                [42.0, 1.0, -34.0],
                [55.0, 0.0, 8.0],
                [30.0, 0.0, 48.0],
                [-12.0, 1.0, 54.0],
                [-48.0, 0.0, 22.0],
                [-52.0, 0.0, -22.0],
                [-28.0, 1.0, -48.0],
            ],
            vec![2, 2, 1, 1],
        ),
    ]
}

fn arch_presets() -> Vec<(&'static str, Vec<[f32; 3]>, Vec<u8>)> {
    vec![
        (
            "S-Curve",
            vec![
                [-70.0, 0.0, -55.0],
                [-30.0, 2.0, -18.0],
                [0.0, 0.0, 12.0],
                [32.0, 4.0, 18.0],
                [72.0, 0.0, 48.0],
            ],
            vec![],
        ),
        (
            "Highway",
            vec![
                [-80.0, 0.0, -60.0],
                [-30.0, 0.0, -25.0],
                [0.0, 0.0, 5.0],
                [35.0, 0.0, 15.0],
                [85.0, 0.0, 50.0],
            ],
            vec![],
        ),
        (
            "Switchback",
            vec![
                [-70.0, 0.0, -60.0],
                [-10.0, 1.0, -25.0],
                [60.0, 2.0, -20.0],
                [5.0, 3.0, 12.0],
                [-55.0, 4.0, 18.0],
                [10.0, 5.0, 50.0],
                [70.0, 4.0, 55.0],
            ],
            vec![],
        ),
    ]
}

fn godot_point(x: f32, y: f32, z: f32, dir: Option<&[u8]>) -> godot::RoadPointData {
    godot::RoadPointData {
        x,
        y,
        z,
        prior_mag: 5.0,
        next_mag: 5.0,
        lane_width: 4.0,
        shoulder_width_l: 2.0,
        shoulder_width_r: 2.0,
        gutter_profile_x: 2.0,
        gutter_profile_y: -0.5,
        traffic_dir: dir
            .map(|d| d.to_vec())
            .unwrap_or_else(|| vec![2, 2, 1, 1]),
        alignment: 0,
        underside_thickness: -1.0,
        terminated: false,
    }
}

// ---------------------------------------------------------------------------
// DOM helpers
// ---------------------------------------------------------------------------

fn doc() -> Document {
    web_sys::window().unwrap().document().unwrap()
}

fn el(id: &str) -> Element {
    doc().get_element_by_id(id).unwrap_or_else(|| panic!("missing #{id}"))
}

fn el_input(id: &str) -> HtmlInputElement {
    el(id).dyn_into::<HtmlInputElement>().unwrap()
}

fn set_text(id: &str, s: &str) {
    el(id).set_text_content(Some(s));
}

fn set_display(id: &str, disp: &str) {
    let e: HtmlElement = el(id).dyn_into().unwrap();
    e.style().set_property("display", disp).unwrap();
}

fn chip(html: &str) -> String {
    format!("<span class=\"chip\">{html}</span>")
}

// ---------------------------------------------------------------------------
// App
// ---------------------------------------------------------------------------

impl App {
    fn new() -> Result<App, JsValue> {
        let canvas = dom_canvas(&el("gl"));
        let gl: Gl = canvas
            .get_context("webgl2")?
            .ok_or_else(|| JsValue::from_str("WebGL2 unavailable"))?
            .dyn_into()?;
        gl.enable(Gl::DEPTH_TEST);
        let surf = Program::new(&gl, shaders::SURFACE_VS, shaders::SURFACE_FS)?;
        let line = Program::new(&gl, shaders::LINE_VS, shaders::LINE_FS)?;

        let mut ground = SurfaceMesh::new();
        let (gi, gx) = ground_geometry();
        ground.upload(&gl, &gi, &gx);

        let mut handle = SurfaceMesh::new();
        let (hi, hx) = sphere_geometry(0.9, 14, 10);
        handle.upload(&gl, &hi, &hx);

        let perf = web_sys::window().unwrap().performance().unwrap();

        let mut app = App {
            gl,
            canvas,
            perf,
            surf,
            line,
            ground,
            road: SurfaceMesh::new(),
            markings: SurfaceMesh::new(),
            handle,
            edge_c: LineMesh::new(),
            edge_f: LineMesh::new(),
            edge_r: LineMesh::new(),
            center: LineMesh::new(),
            lanes: Vec::new(),
            ctrl: LineMesh::new(),
            mode: 0,
            g_points: Vec::new(),
            g_density: 4.0,
            g_under: 0.0,
            g_lowpoly: false,
            g_auto: true,
            a_nodes: Vec::new(),
            a_width: 8.0,
            a_shoulder: 1.5,
            a_spm: 2.0,
            a_mark: true,
            a_shoulder_on: true,
            show_edges: true,
            show_lanes: true,
            show_center: true,
            selected: None,
            cam: Orbit::default(),
            drag: None,
            closures: Vec::new(),
        };
        app.apply_preset(0);
        Ok(app)
    }

    // -- state ------------------------------------------------------------

    fn node_positions(&self) -> Vec<[f32; 3]> {
        match self.mode {
            0 => self
                .g_points
                .iter()
                .map(|p| [p.x, p.y, p.z])
                .collect(),
            _ => self.a_nodes.iter().map(|n| [n.x, n.y, n.z]).collect(),
        }
    }

    fn set_selected(&mut self, idx: Option<usize>) {
        self.selected = idx;
        self.sync_sel();
    }

    fn apply_preset(&mut self, i: usize) {
        let presets = if self.mode == 0 {
            godot_presets()
        } else {
            arch_presets()
        };
        let (_, pts, dir) = {
            let p = &presets[i.min(presets.len() - 1)];
            (p.0, p.1.clone(), p.2.clone())
        };
        match self.mode {
            0 => {
                self.g_points = pts
                    .iter()
                    .map(|p| godot_point(p[0], p[1], p[2], Some(&dir)))
                    .collect();
            }
            _ => {
                self.a_nodes = pts
                    .iter()
                    .map(|p| arch::Node {
                        x: p[0],
                        y: p[1],
                        z: p[2],
                        ease_in: 0.5,
                        ease_out: 0.5,
                    })
                    .collect();
            }
        }
        self.set_selected(None);
        self.sync_point_count();
        self.regenerate();
        set_text("modeTitle", if self.mode == 0 { "Road manager" } else { "Road architect" });
    }

    fn add_node(&mut self, x: f32, z: f32) {
        match self.mode {
            0 => {
                if self.g_points.len() >= 32 {
                    return;
                }
                let dir = self.g_points.last().map(|p| p.traffic_dir.clone());
                self.g_points.push(godot_point(x, 0.0, z, dir.as_deref()));
                self.set_selected(Some(self.g_points.len() - 1));
            }
            _ => {
                if self.a_nodes.len() >= 32 {
                    return;
                }
                self.a_nodes.push(arch::Node {
                    x,
                    y: 0.0,
                    z,
                    ease_in: 0.5,
                    ease_out: 0.5,
                });
                self.set_selected(Some(self.a_nodes.len() - 1));
            }
        }
        self.sync_point_count();
        self.regenerate();
    }

    fn add_node_continuing(&mut self) {
        let nodes = self.node_positions();
        let (last, prev) = match (nodes.last(), nodes.iter().rev().nth(1)) {
            (Some(l), Some(p)) => (*l, *p),
            (Some(l), None) => (*l, [l[0] - 20.0, l[1], l[2]]),
            (None, _) => return,
        };
        let dx = last[0] - prev[0];
        let dz = last[2] - prev[2];
        self.add_node(last[0] + dx * 0.8, last[2] + dz * 0.8);
    }

    fn delete_selected(&mut self) {
        let len_ok = match self.mode {
            0 => self.g_points.len() > 2,
            _ => self.a_nodes.len() > 2,
        };
        if !len_ok {
            return;
        }
        if let Some(i) = self.selected {
            match self.mode {
                0 => {
                    self.g_points.remove(i);
                }
                _ => {
                    self.a_nodes.remove(i);
                }
            }
            self.set_selected(None);
            self.sync_point_count();
            self.regenerate();
        }
    }

    fn switch_mode(&mut self, m: u8) {
        if self.mode == m {
            return;
        }
        let pos = self.node_positions();
        self.mode = m;
        self.set_selected(None);
        match m {
            0 => {
                self.g_points = pos.iter().map(|p| godot_point(p[0], p[1], p[2], None)).collect();
            }
            _ => {
                self.a_nodes = pos
                    .iter()
                    .map(|p| arch::Node {
                        x: p[0],
                        y: p[1],
                        z: p[2],
                        ease_in: 0.5,
                        ease_out: 0.5,
                    })
                    .collect();
            }
        }
        let body = doc().body().unwrap();
        body.set_class_name(if m == 0 { "mode-godot" } else { "mode-arch" });
        // Active button styling.
        let nl = doc().query_selector_all(".modebtn").unwrap();
        for k in 0..nl.length() {
            let b: Element = nl.get(k).unwrap().dyn_into().unwrap();
            let on = b.get_attribute("data-mode").as_deref()
                == Some(if m == 0 { "godot" } else { "arch" });
            b.set_class_name(if on { "modebtn active" } else { "modebtn" });
        }
        self.build_presets();
        set_text("modeTitle", if m == 0 { "Road manager" } else { "Road architect" });
        self.sync_point_count();
        self.sync_sel();
        self.regenerate();
    }

    // -- regeneration -------------------------------------------------------

    fn regenerate(&mut self) {
        let t0 = self.perf.now();
        match self.mode {
            0 => {
                let req = godot::GenRequest {
                    manager: godot::ManagerData {
                        density: self.g_density,
                        underside_thickness: if self.g_under > 0.0 {
                            self.g_under
                        } else {
                            -1.0
                        },
                        low_poly: self.g_lowpoly,
                        auto_lanes: self.g_auto,
                    },
                    points: self.g_points.clone(),
                };
                let resp = godot::generate_road(&req);
                self.road.upload(
                    &self.gl,
                    &interleave(&resp.road.positions, &resp.road.normals, &resp.road.uvs),
                    &resp.road.indices,
                );
                self.edge_c.upload(&self.gl, &resp.edge_c);
                self.edge_f.upload(&self.gl, &resp.edge_f);
                self.edge_r.upload(&self.gl, &resp.edge_r);
                self.center.upload(&self.gl, &resp.centerline);
                self.lanes = resp
                    .lanes
                    .iter()
                    .map(|l| {
                        let mut m = LineMesh::new();
                        m.upload(&self.gl, &l.path);
                        (l.dir, m)
                    })
                    .collect();
                let verts = resp.vertex_count;
                let tris = resp.triangle_count;
                let len = resp.length;
                let loops = resp.loop_count;
                let nl = resp.lanes.len();
                self.ctrl_from(&resp.centerline);
                self.set_stats(&[
                    chip("wasm32-unknown-unknown"),
                    chip(&format!("{} m · {} loops", len as i32, loops)),
                    chip(&format!("{} verts", verts)),
                    chip(&format!("{} tris · {} lanes", tris, nl)),
                    chip(&format!("rust {:.2} ms", self.perf.now() - t0)),
                ]);
            }
            _ => {
                let req = arch::GenRequest {
                    nodes: self.a_nodes.clone(),
                    road_width: self.a_width,
                    shoulder_width: self.a_shoulder,
                    samples_per_meter: self.a_spm,
                    magnitude_threshold: 300.0,
                    lane_markings: self.a_mark,
                    shoulders: self.a_shoulder_on,
                };
                let resp = arch::generate_road(&req);
                self.road.upload(
                    &self.gl,
                    &interleave(&resp.road.positions, &resp.road.normals, &resp.road.uvs),
                    &resp.road.indices,
                );
                self.markings.upload(
                    &self.gl,
                    &interleave(
                        &resp.markings.positions,
                        &resp.markings.normals,
                        &resp.markings.uvs,
                    ),
                    &resp.markings.indices,
                );
                self.center.upload(&self.gl, &resp.centerline);
                self.edge_c.upload(&self.gl, &[]);
                self.edge_f.upload(&self.gl, &[]);
                self.edge_r.upload(&self.gl, &[]);
                self.lanes.clear();
                let verts = resp.vertex_count;
                let tris = resp.triangle_count;
                let len = resp.length;
                self.ctrl_from(&resp.centerline);
                self.set_stats(&[
                    chip("RoadArchitect hermite"),
                    chip(&format!("{} m", len as i32)),
                    chip(&format!("{} verts", verts)),
                    chip(&format!("{} tris", tris)),
                    chip(&format!("rust {:.2} ms", self.perf.now() - t0)),
                ]);
            }
        }
    }

    fn ctrl_from(&mut self, _centerline: &[f32]) {
        let pts: Vec<f32> = self
            .node_positions()
            .iter()
            .flat_map(|p| vec![p[0], p[1] + 0.12, p[2]])
            .collect();
        self.ctrl.upload(&self.gl, &pts);
    }

    fn set_stats(&mut self, chips: &[String]) {
        el("stats").set_inner_html(&chips.join(""));
    }

    fn sync_point_count(&mut self) {
        let n = match self.mode {
            0 => self.g_points.len(),
            _ => self.a_nodes.len(),
        };
        set_text("pcount", &format!("Points: {n}"));
    }

    fn sync_sel(&mut self) {
        match self.selected {
            Some(i) => {
                let n = match self.mode {
                    0 => self.g_points.len(),
                    _ => self.a_nodes.len(),
                };
                if i >= n {
                    set_display("selBox", "none");
                    return;
                }
                set_display("selBox", "block");
                set_text(
                    "selTitle",
                    &format!(
                        "{} #{}",
                        if self.mode == 0 { "RoadPoint" } else { "Node" },
                        i
                    ),
                );
                match self.mode {
                    0 => {
                        let p = &self.g_points[i];
                        el_input("sel-prior").set_value_as_number(p.prior_mag as f64);
                        el_input("sel-next").set_value_as_number(p.next_mag as f64);
                        el_input("sel-elev").set_value_as_number(p.y as f64);
                        set_text("sel-prior-v", &format!("{:.1}", p.prior_mag));
                        set_text("sel-next-v", &format!("{:.1}", p.next_mag));
                        set_text("sel-elev-v", &format!("{:.1} m", p.y));
                    }
                    _ => {
                        let n = &self.a_nodes[i];
                        el_input("sel-easein").set_value_as_number(n.ease_in as f64);
                        el_input("sel-easeout").set_value_as_number(n.ease_out as f64);
                        el_input("sel-elev").set_value_as_number(n.y as f64);
                        set_text("sel-easein-v", &format!("{:.2}", n.ease_in));
                        set_text("sel-easeout-v", &format!("{:.2}", n.ease_out));
                        set_text("sel-elev-v", &format!("{:.1} m", n.y));
                    }
                }
            }
            None => {
                set_display("selBox", "none");
            }
        }
    }

    fn patch_selected_y(&mut self, y: f32) {
        match (self.selected, self.mode) {
            (Some(i), 0) => self.g_points[i].y = y,
            (Some(i), _) => self.a_nodes[i].y = y,
            _ => {}
        }
        self.regenerate();
    }

    // -- ui wiring ----------------------------------------------------------

    fn build_presets(&mut self) {
        let cont = el("presets");
        cont.set_inner_html("");
        let presets = if self.mode == 0 {
            godot_presets()
        } else {
            arch_presets()
        };
        for i in 0..presets.len() {
            let b = doc()
                .create_element("button")
                .unwrap()
                .dyn_into::<HtmlButtonElement>()
                .unwrap();
            b.set_class_name("pbtn");
            b.set_text_content(Some(presets[i].0));
            b.set_attribute("data-i", &i.to_string()).unwrap();
            let cb = {
                let app = APP.with(|a| a.borrow().clone()).unwrap();
                let i = i;
                Closure::<dyn FnMut(JsValue)>::new(move |_e| {
                    app.borrow_mut().apply_preset(i);
                })
            };
            b.add_event_listener_with_callback("click", cb.as_ref().unchecked_ref())
                .unwrap();
            cont.append_child(&b).unwrap();
            self.closures.push(cb);
        }
    }

    fn wire(&mut self, app: Rc<RefCell<App>>) {
        // Mode switch buttons.
        let nl = doc().query_selector_all(".modebtn").unwrap();
        for k in 0..nl.length() {
            let btn: HtmlButtonElement = nl.get(k).unwrap().dyn_into().unwrap();
            let m: u8 = if btn.get_attribute("data-mode").as_deref() == Some("godot") {
                0
            } else {
                1
            };
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |_e| {
                    app.borrow_mut().switch_mode(m);
                })
            };
            btn.add_event_listener_with_callback("click", cb.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb);
        }

        // Sliders / toggles.
        let bind_slider = |app: &Rc<RefCell<App>>,
                           id: &'static str,
                           lab: &'static str,
                           f: fn(&mut App, f32)| {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |_e| {
                    let v = el_input(id).value_as_number() as f32;
                    let lab_v = el_input(id).value();
                    set_text(lab, &lab_v);
                    app.borrow_mut().regen_with(|a| f(a, v));
                })
            };
            el_input(id)
                .add_event_listener_with_callback("input", cb.as_ref().unchecked_ref())
                .unwrap();
            cb
        };

        macro_rules! keep {
            ($cb:expr) => {
                self.closures.push($cb);
            };
        }

        keep!(bind_slider(&app, "density", "density-v", |a, v| a.g_density = v));
        keep!(bind_slider(&app, "underside", "underside-v", |a, v| a.g_under = v));
        keep!(bind_slider(&app, "roadw", "roadw-v", |a, v| a.a_width = v));
        keep!(bind_slider(&app, "shoulder", "shoulder-v", |a, v| a.a_shoulder = v));
        keep!(bind_slider(&app, "spm", "spm-v", |a, v| a.a_spm = v));
        keep!(bind_slider(&app, "sel-prior", "sel-prior-v", |a, v| a
            .patch_sel_godot(|p| p.prior_mag = v)));
        keep!(bind_slider(&app, "sel-next", "sel-next-v", |a, v| a
            .patch_sel_godot(|p| p.next_mag = v)));
        keep!(bind_slider(&app, "sel-easein", "sel-easein-v", |a, v| a
            .patch_sel_arch(|n| n.ease_in = v)));
        keep!(bind_slider(&app, "sel-easeout", "sel-easeout-v", |a, v| a
            .patch_sel_arch(|n| n.ease_out = v)));
        keep!(bind_slider(&app, "sel-elev", "sel-elev-v", |a, v| {
            a.patch_selected_y(v)
        }));

        let bind_check = |app: &Rc<RefCell<App>>,
                          id: &'static str,
                          f: fn(&mut App, bool)| {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |_e| {
                    let v = el_input(id).checked();
                    app.borrow_mut().regen_with(|a| f(a, v));
                })
            };
            el_input(id)
                .add_event_listener_with_callback("change", cb.as_ref().unchecked_ref())
                .unwrap();
            cb
        };

        keep!(bind_check(&app, "lowpoly", |a, v| a.g_lowpoly = v));
        keep!(bind_check(&app, "autolanes", |a, v| a.g_auto = v));
        keep!(bind_check(&app, "edges", |a, v| a.show_edges = v));
        keep!(bind_check(&app, "lanes", |a, v| a.show_lanes = v));
        keep!(bind_check(&app, "center", |a, v| a.show_center = v));
        keep!(bind_check(&app, "markings", |a, v| a.a_mark = v));
        keep!(bind_check(&app, "ashoulders", |a, v| a.a_shoulder_on = v));

        // Buttons.
        let bind_click = |app: &Rc<RefCell<App>>,
                          id: &'static str,
                          f: fn(&mut App)| {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |_e| {
                    app.borrow_mut().regen_with(f);
                })
            };
            el(id)
                .add_event_listener_with_callback("click", cb.as_ref().unchecked_ref())
                .unwrap();
            cb
        };
        keep!(bind_click(&app, "addPt", |a| a.add_node_continuing()));
        keep!(bind_click(&app, "resetRoad", |a| a.apply_preset(0)));
        keep!(bind_click(&app, "sel-del", |a| a.delete_selected()));
        keep!(bind_click(&app, "hidePanel", |_a| {
            set_display("panel", "none");
            set_display("showPanel", "inline-flex");
        }));
        keep!(bind_click(&app, "showPanel", |_a| {
            set_display("panel", "block");
            set_display("showPanel", "none");
        }));

        // Build initial preset buttons (needs APP thread-local ready).
        APP.with(|a| *a.borrow_mut() = Some(app.clone()));
        self.build_presets();

        // Canvas pointer events.
        {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |e: JsValue| {
                    if let Ok(ev) = e.dyn_into::<PointerEvent>() {
                        app.borrow_mut().on_pointer_down(&ev);
                    }
                })
            };
            self.canvas
                .add_event_listener_with_callback("pointerdown", cb.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb);
        }
        {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |e: JsValue| {
                    if let Ok(ev) = e.dyn_into::<PointerEvent>() {
                        let mut a = app.borrow_mut();
                        a.on_pointer_move(&ev);
                    }
                })
            };
            web_sys::window()
                .unwrap()
                .add_event_listener_with_callback("pointermove", cb.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb);
        }
        {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |_e: JsValue| {
                    app.borrow_mut().drag = None;
                })
            };
            web_sys::window()
                .unwrap()
                .add_event_listener_with_callback("pointerup", cb.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb);
        }
        {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |e: JsValue| {
                    if let Ok(ev) = e.dyn_into::<MouseEvent>() {
                        app.borrow_mut().on_dblclick(&ev);
                    }
                })
            };
            self.canvas
                .add_event_listener_with_callback("dblclick", cb.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb);
        }
        {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |e: JsValue| {
                    if let Ok(ev) = e.dyn_into::<WheelEvent>() {
                        ev.prevent_default();
                        app.borrow_mut().on_wheel(&ev);
                    }
                })
            };
            self.canvas
                .add_event_listener_with_callback("wheel", cb.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb);
        }
        {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |e: JsValue| {
                    if let Ok(ev) = e.dyn_into::<KeyboardEvent>() {
                        let k = ev.key();
                        if k == "Delete" || k == "Backspace" {
                            app.borrow_mut().delete_selected();
                        }
                    }
                })
            };
            web_sys::window()
                .unwrap()
                .add_event_listener_with_callback("keydown", cb.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb);
        }
    }

    fn regen_with(&mut self, f: impl FnOnce(&mut App)) {
        f(self);
        self.regenerate();
        self.sync_sel();
        self.sync_point_count();
    }

    fn patch_sel_godot(&mut self, f: impl FnOnce(&mut godot::RoadPointData)) {
        if let Some(i) = self.selected {
            if i < self.g_points.len() {
                f(&mut self.g_points[i]);
            }
        }
    }

    fn patch_sel_arch(&mut self, f: impl FnOnce(&mut arch::Node)) {
        if let Some(i) = self.selected {
            if i < self.a_nodes.len() {
                f(&mut self.a_nodes[i]);
            }
        }
    }

    // -- pointer interaction ------------------------------------------------

    fn canvas_px(&self, ev: &MouseEvent) -> (f64, f64) {
        let rect = self.canvas.get_bounding_client_rect();
        (
            ev.client_x() as f64 - rect.left(),
            ev.client_y() as f64 - rect.top(),
        )
    }

    fn pick_node(&self, px: f64, py: f64) -> Option<usize> {
        let aspect = self.canvas.client_width() as f32 / self.canvas.client_height() as f32;
        let (vp, _) = self.cam.view_proj(aspect);
        let w = self.canvas.client_width() as f64;
        let h = self.canvas.client_height() as f64;
        let mut best: Option<(usize, f64)> = None;
        for (i, p) in self.node_positions().iter().enumerate() {
            let (nx, ny, w_) = project(&vp, *p);
            if w_ <= 0.0 {
                continue;
            }
            let sx = (nx as f64 * 0.5 + 0.5) * w;
            let sy = (0.5 - ny as f64 * 0.5) * h;
            let d = ((sx - px).powi(2) + (sy - py).powi(2)).sqrt();
            if d < 20.0 && best.map(|(_, bd)| d < bd).unwrap_or(true) {
                best = Some((i, d));
            }
        }
        best.map(|(i, _)| i)
    }

    fn on_pointer_down(&mut self, ev: &PointerEvent) {
        let (px, py) = self.canvas_px(ev);
        if let Some(i) = self.pick_node(px, py) {
            self.drag = Some(Drag::Node(i));
            self.set_selected(Some(i));
        } else {
            self.drag = Some(Drag::Orbit { x: px, y: py });
        }
    }

    fn on_pointer_move(&mut self, ev: &PointerEvent) {
        match &self.drag {
            Some(Drag::Orbit { .. }) => {
                let (px, py) = self.canvas_px(ev);
                if let Some(Drag::Orbit { x, y }) = &mut self.drag {
                    self.cam.yaw -= ((px - *x) as f32) * 0.006;
                    self.cam.pitch = (self.cam.pitch + ((py - *y) as f32) * 0.006).clamp(0.06, 1.5);
                    *x = px;
                    *y = py;
                }
            }
            Some(Drag::Node(i)) => {
                let i = *i;
                let aspect = self.canvas.client_width() as f32 / self.canvas.client_height() as f32;
                let (px, py) = self.canvas_px(ev);
                let w = self.canvas.client_width() as f64;
                let h = self.canvas.client_height() as f64;
                let ndc_x = (px / w * 2.0 - 1.0) as f32;
                let ndc_y = (1.0 - py / h * 2.0) as f32;
                let (eye, dir) = self.cam.ray_through(ndc_x, ndc_y, aspect);
                if dir[1] < -1e-4 {
                    let t = -eye[1] / dir[1];
                    let x = (eye[0] + dir[0] * t).clamp(-400.0, 400.0);
                    let z = (eye[2] + dir[2] * t).clamp(-400.0, 400.0);
                    match self.mode {
                        0 => {
                            if i < self.g_points.len() {
                                self.g_points[i].x = x;
                                self.g_points[i].z = z;
                            }
                        }
                        _ => {
                            if i < self.a_nodes.len() {
                                self.a_nodes[i].x = x;
                                self.a_nodes[i].z = z;
                            }
                        }
                    }
                    self.regenerate();
                }
            }
            None => {}
        }
    }

    fn on_dblclick(&mut self, ev: &MouseEvent) {
        let (px, py) = self.canvas_px(ev);
        if self.pick_node(px, py).is_some() {
            return;
        }
        let aspect = self.canvas.client_width() as f32 / self.canvas.client_height() as f32;
        let w = self.canvas.client_width() as f64;
        let h = self.canvas.client_height() as f64;
        let ndc_x = (px / w * 2.0 - 1.0) as f32;
        let ndc_y = (1.0 - py / h * 2.0) as f32;
        let (eye, dir) = self.cam.ray_through(ndc_x, ndc_y, aspect);
        if dir[1] < -1e-4 {
            let t = -eye[1] / dir[1];
            let x = (eye[0] + dir[0] * t).clamp(-400.0, 400.0);
            let z = (eye[2] + dir[2] * t).clamp(-400.0, 400.0);
            self.add_node(x, z);
        }
    }

    fn on_wheel(&mut self, ev: &WheelEvent) {
        let d = ev.delta_y();
        self.cam.dist = (self.cam.dist * (d * 0.0012).exp() as f32).clamp(14.0, 420.0);
    }

    // -- rendering ----------------------------------------------------------

    fn resize_if_needed(&mut self) {
        let dpr = web_sys::window().unwrap().device_pixel_ratio();
        let w = (self.canvas.client_width() as f64 * dpr) as u32;
        let h = (self.canvas.client_height() as f64 * dpr) as u32;
        if self.canvas.width() != w || self.canvas.height() != h {
            self.canvas.set_width(w.max(1));
            self.canvas.set_height(h.max(1));
        }
    }

    fn draw(&mut self) {
        self.resize_if_needed();
        let gl = &self.gl;
        let w = self.canvas.width();
        let h = self.canvas.height();
        gl.viewport(0, 0, w as i32, h as i32);
        gl.clear_color(SKY[0], SKY[1], SKY[2], 1.0);
        gl.clear(Gl::COLOR_BUFFER_BIT | Gl::DEPTH_BUFFER_BIT);

        let aspect = w as f32 / h as f32;
        let (vp, eye) = self.cam.view_proj(aspect);

        // Surfaces.
        self.surf.use_prog(gl);
        gl.uniform3f(self.surf.u_offset.as_ref(), 0.0, 0.0, 0.0);
        gl.uniform_matrix4fv_with_f32_array(self.surf.u_vp.as_ref(), false, &vp.0);
        gl.uniform3f(self.surf.u_eye.as_ref(), eye[0], eye[1], eye[2]);
        gl.uniform3f(self.surf.u_light.as_ref(), 0.45, 0.85, 0.3);

        gl.uniform1i(self.surf.u_kind.as_ref(), 3);
        self.ground.draw(gl);

        gl.uniform1i(self.surf.u_kind.as_ref(), if self.mode == 0 { 0 } else { 2 });
        self.road.draw(gl);

        if self.mode == 1 {
            gl.uniform1i(self.surf.u_kind.as_ref(), 1);
            gl.uniform3f(self.surf.u_color.as_ref(), 0.92, 0.92, 0.9);
            self.markings.draw(gl);
        }

        // Lines.
        self.line.use_prog(gl);
        gl.uniform_matrix4fv_with_f32_array(self.line.u_vp.as_ref(), false, &vp.0);
        gl.uniform3f(self.line.u_eye.as_ref(), eye[0], eye[1], eye[2]);

        if self.show_edges && self.mode == 0 {
            gl.uniform3f(self.line.u_color.as_ref(), AMBER_L[0], AMBER_L[1], AMBER_L[2]);
            self.edge_c.draw(gl);
            gl.uniform3f(self.line.u_color.as_ref(), WHITE_L[0], WHITE_L[1], WHITE_L[2]);
            self.edge_f.draw(gl);
            self.edge_r.draw(gl);
        }
        if self.show_lanes && self.mode == 0 {
            for (dir, mesh) in &self.lanes {
                let c = match *dir {
                    1 => TEAL,
                    2 => ROSE,
                    _ => WHITE_L,
                };
                gl.uniform3f(self.line.u_color.as_ref(), c[0], c[1], c[2]);
                mesh.draw(gl);
            }
        }
        if self.show_center {
            gl.uniform3f(self.line.u_color.as_ref(), EMERALD[0], EMERALD[1], EMERALD[2]);
            self.center.draw(gl);
        }
        gl.uniform3f(self.line.u_color.as_ref(), GREY_LINE[0], GREY_LINE[1], GREY_LINE[2]);
        self.ctrl.draw(gl);

        // Node handles.
        self.surf.use_prog(gl);
        gl.uniform_matrix4fv_with_f32_array(self.surf.u_vp.as_ref(), false, &vp.0);
        gl.uniform3f(self.surf.u_eye.as_ref(), eye[0], eye[1], eye[2]);
        gl.uniform3f(self.surf.u_light.as_ref(), 0.45, 0.85, 0.3);
        gl.uniform1i(self.surf.u_kind.as_ref(), 1);
        for (i, p) in self.node_positions().iter().enumerate() {
            let c = if self.selected == Some(i) { EMERALD } else { AMBER };
            gl.uniform3f(self.surf.u_color.as_ref(), c[0], c[1], c[2]);
            gl.uniform3f(self.surf.u_offset.as_ref(), p[0], p[1], p[2]);
            self.handle.draw(gl);
        }
    }
}

// ---------------------------------------------------------------------------
// bootstrap
// ---------------------------------------------------------------------------

thread_local! {
    static APP: RefCell<Option<Rc<RefCell<App>>>> = const { RefCell::new(None) };
}

fn ground_geometry() -> (Vec<f32>, Vec<u32>) {
    let s = 900.0f32;
    let y = -0.35f32;
    let v = |x: f32, z: f32| vec![x, y, z, 0.0, 1.0, 0.0, 0.0, 0.0];
    let mut inter = Vec::new();
    inter.extend(v(-s, -s));
    inter.extend(v(s, -s));
    inter.extend(v(s, s));
    inter.extend(v(-s, s));
    (inter, vec![0, 1, 2, 0, 2, 3])
}

pub fn start() -> Result<(), JsValue> {
    let app = Rc::new(RefCell::new(App::new()?));
    app.borrow_mut().wire(app.clone());

    // rAF loop with a single persistent closure.
    let slot: Rc<RefCell<Option<Closure<dyn FnMut()>>>> = Rc::new(RefCell::new(None));
    let slot2 = slot.clone();
    let app2 = app.clone();
    let cb = Closure::<dyn FnMut()>::new(move || {
        app2.borrow_mut().draw();
        let ls = slot2.clone();
        let guard = ls.borrow();
        if let Some(c) = guard.as_ref() {
            web_sys::window()
                .unwrap()
                .request_animation_frame(c.as_ref().unchecked_ref())
                .unwrap();
        }
        drop(guard);
    });
    *slot.borrow_mut() = Some(cb);
    let guard = slot.borrow();
    if let Some(c) = guard.as_ref() {
        web_sys::window()
            .unwrap()
            .request_animation_frame(c.as_ref().unchecked_ref())
            .unwrap();
    }
    drop(guard);

    // Hide the loading overlay once the first frame has been scheduled.
    set_display("loading", "none");
    Ok(())
}
