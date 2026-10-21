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
    /// Flat node index + optional junction partner (kept glued while dragging).
    Node(usize, Option<usize>),
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
    edge_c: Vec<LineMesh>,
    edge_f: Vec<LineMesh>,
    edge_r: Vec<LineMesh>,
    center: Vec<LineMesh>,
    lanes: Vec<LaneLine>,
    ctrl: LineMesh,
    junction: SurfaceMesh,
    jmarks: SurfaceMesh,
    mode: u8, // 0 = godot (Bézier), 1 = RoadArchitect (Hermite)
    /// Road network — g_roads[0] is the main road; points shared with
    /// roads[1..] (within JUNCTION_EPS) build junctions in the core.
    g_roads: Vec<Vec<godot::RoadPointData>>,
    g_density: f32,
    g_under: f32,
    g_lowpoly: bool,
    g_auto: bool,
    g_jmarks: bool,
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
// Presets — (name, roads: [ [ (x, y, z), ... ] main first ], traffic_dir)
// Roads whose points share coordinates with the main road build junctions.
// ---------------------------------------------------------------------------

fn godot_presets() -> Vec<(&'static str, Vec<Vec<[f32; 3]>>, Vec<u8>)> {
    vec![
        (
            "S-Curve",
            vec![vec![
                [-70.0, 0.0, -55.0],
                [-30.0, 2.0, -18.0],
                [0.0, 0.0, 12.0],
                [32.0, 4.0, 18.0],
                [72.0, 0.0, 48.0],
            ]],
            vec![2, 2, 1, 1],
        ),
        (
            "2-Lane",
            vec![vec![
                [-60.0, 0.0, -50.0],
                [-15.0, 0.0, -10.0],
                [25.0, 1.0, 8.0],
                [65.0, 0.0, 40.0],
            ]],
            vec![2, 1],
        ),
        (
            "Highway",
            vec![vec![
                [-80.0, 0.0, -60.0],
                [-30.0, 0.0, -25.0],
                [0.0, 0.0, 5.0],
                [35.0, 0.0, 15.0],
                [85.0, 0.0, 50.0],
            ]],
            vec![2, 2, 2, 1, 1, 1],
        ),
        (
            "4-Way",
            vec![
                vec![
                    [-90.0, 0.0, 0.0],
                    [-45.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [45.0, 0.0, 0.0],
                    [90.0, 0.0, 0.0],
                ],
                vec![[0.0, 0.0, -55.0], [0.0, 0.0, 0.0], [0.0, 0.0, 55.0]],
            ],
            vec![2, 2, 1, 1],
        ),
        (
            "T-Junction",
            vec![
                vec![
                    [-90.0, 0.0, 0.0],
                    [-45.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [45.0, 0.0, 0.0],
                    [90.0, 0.0, 0.0],
                ],
                vec![[0.0, 0.0, -60.0], [0.0, 0.0, 0.0]],
            ],
            vec![2, 2, 1, 1],
        ),
        (
            "Town",
            vec![
                vec![
                    [-100.0, 0.0, 0.0],
                    [-50.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [50.0, 0.0, 0.0],
                    [100.0, 0.0, 0.0],
                ],
                vec![[0.0, 0.0, -55.0], [0.0, 0.0, 0.0], [0.0, 0.0, 55.0]],
                vec![[50.0, 0.0, 50.0], [50.0, 0.0, 0.0]],
            ],
            vec![2, 2, 1, 1],
        ),
        (
            "Switchback",
            vec![vec![
                [-70.0, 0.0, -60.0],
                [-10.0, 1.0, -25.0],
                [60.0, 2.0, -20.0],
                [5.0, 3.0, 12.0],
                [-55.0, 4.0, 18.0],
                [10.0, 5.0, 50.0],
                [70.0, 4.0, 55.0],
            ]],
            vec![2, 1],
        ),
        (
            "Loop",
            vec![vec![
                [0.0, 0.0, -55.0],
                [42.0, 1.0, -34.0],
                [55.0, 0.0, 8.0],
                [30.0, 0.0, 48.0],
                [-12.0, 1.0, 54.0],
                [-48.0, 0.0, 22.0],
                [-52.0, 0.0, -22.0],
                [-28.0, 1.0, -48.0],
            ]],
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
        generate_mesh_at_intersections: true,
        edge_edge_clipping_offset: 0.0,
    }
}

fn dir_name(d: u8) -> &'static str {
    match d {
        1 => "FORWARD",
        2 => "REVERSE",
        3 => "BOTH",
        _ => "NONE",
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
            edge_c: Vec::new(),
            edge_f: Vec::new(),
            edge_r: Vec::new(),
            center: Vec::new(),
            lanes: Vec::new(),
            ctrl: LineMesh::new(),
            junction: SurfaceMesh::new(),
            jmarks: SurfaceMesh::new(),
            mode: 0,
            g_roads: Vec::new(),
            g_density: 4.0,
            g_under: 0.0,
            g_lowpoly: false,
            g_auto: true,
            g_jmarks: true,
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
                .g_roads
                .iter()
                .flatten()
                .map(|p| [p.x, p.y, p.z])
                .collect(),
            _ => self.a_nodes.iter().map(|n| [n.x, n.y, n.z]).collect(),
        }
    }

    fn main_positions(&self) -> Vec<[f32; 3]> {
        match self.mode {
            0 => self.g_roads[0].iter().map(|p| [p.x, p.y, p.z]).collect(),
            _ => self.a_nodes.iter().map(|n| [n.x, n.y, n.z]).collect(),
        }
    }

    /// Total editable points across all roads (flat selection space).
    fn flat_len(&self) -> usize {
        match self.mode {
            0 => self.g_roads.iter().map(|r| r.len()).sum(),
            _ => self.a_nodes.len(),
        }
    }

    /// Flat index → (road, point) — godot network mode only.
    fn flat_loc(&self, k: usize) -> (usize, usize) {
        match self.mode {
            0 => {
                let mut acc = 0usize;
                for (ri, r) in self.g_roads.iter().enumerate() {
                    if k < acc + r.len() {
                        return (ri, k - acc);
                    }
                    acc += r.len();
                }
                (0, 0)
            }
            _ => (0, k),
        }
    }

    /// Junction kind (2/3/4) at a point shared with another road, if any.
    fn junction_kind_at(&self, ri: usize, pi: usize) -> Option<u8> {
        if self.mode != 0 {
            return None;
        }
        let p = &self.g_roads[ri][pi];
        let road = &self.g_roads[ri];
        let arms_on = |len: usize, idx: usize| -> u8 {
            if len < 2 {
                0
            } else if idx > 0 && idx < len - 1 {
                2
            } else {
                1
            }
        };
        let mut arms = arms_on(road.len(), pi);
        for (r2, road2) in self.g_roads.iter().enumerate() {
            if r2 == ri {
                continue;
            }
            for (j2, q) in road2.iter().enumerate() {
                let dx = p.x - q.x;
                let dy = p.y - q.y;
                let dz = p.z - q.z;
                if (dx * dx + dy * dy + dz * dz).sqrt() < 0.75 {
                    arms += arms_on(road2.len(), j2);
                    return Some(arms.min(4));
                }
            }
        }
        None
    }

    /// Flat index of the junction partner of a point, if any.
    fn find_partner(&self, k: usize) -> Option<usize> {
        if self.mode != 0 {
            return None;
        }
        let pts = self.node_positions();
        let (ri, _) = self.flat_loc(k);
        let mut best: Option<(usize, f32)> = None;
        let mut acc = 0usize;
        for (r2, road) in self.g_roads.iter().enumerate() {
            for j2 in 0..road.len() {
                let m = acc + j2;
                if m != k && r2 != ri {
                    let d = ((pts[m][0] - pts[k][0]).powi(2)
                        + (pts[m][2] - pts[k][2]).powi(2))
                        .sqrt();
                    if d < 0.75 && best.map(|(_, bd)| d < bd).unwrap_or(true) {
                        best = Some((m, d));
                    }
                }
            }
            acc += road.len();
        }
        best.map(|(m, _)| m)
    }

    fn set_selected(&mut self, idx: Option<usize>) {
        self.selected = idx;
        self.sync_sel();
    }

    fn apply_preset(&mut self, i: usize) {
        let presets: Vec<(&'static str, Vec<Vec<[f32; 3]>>, Vec<u8>)> = if self.mode == 0 {
            godot_presets()
        } else {
            arch_presets()
                .into_iter()
                .map(|(n, pts, d)| (n, vec![pts], d))
                .collect()
        };
        let (_, roads, dir) = {
            let p = &presets[i.min(presets.len() - 1)];
            (p.0, p.1.clone(), p.2.clone())
        };
        match self.mode {
            0 => {
                self.g_roads = roads
                    .iter()
                    .map(|r| {
                        r.iter()
                            .map(|p| godot_point(p[0], p[1], p[2], Some(&dir)))
                            .collect()
                    })
                    .collect();
            }
            _ => {
                self.a_nodes = roads[0]
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
                if self.g_roads.is_empty() {
                    self.g_roads.push(Vec::new());
                }
                if self.g_roads[0].len() >= 32 {
                    return;
                }
                let dir = self.g_roads[0].last().map(|p| p.traffic_dir.clone());
                self.g_roads[0].push(godot_point(x, 0.0, z, dir.as_deref()));
                self.set_selected(Some(self.flat_len() - 1));
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
        let nodes = self.main_positions();
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
        if let Some(k) = self.selected {
            match self.mode {
                0 => {
                    if self.flat_len() <= 2 {
                        return;
                    }
                    let (ri, pi) = self.flat_loc(k);
                    self.g_roads[ri].remove(pi);
                    // Drop a road that fell below 2 points (unless it is the
                    // only one — the len guard above already protected it).
                    if self.g_roads[ri].len() < 2 && self.g_roads.len() > 1 {
                        self.g_roads.remove(ri);
                    }
                }
                _ => {
                    if self.a_nodes.len() > 2 {
                        self.a_nodes.remove(k);
                    }
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
                self.g_roads = vec![pos
                    .iter()
                    .map(|p| godot_point(p[0], p[1], p[2], None))
                    .collect()];
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
                    points: vec![],
                    roads: self.g_roads.clone(),
                };
                let resp = godot::generate_road(&req);
                self.road.upload(
                    &self.gl,
                    &interleave(&resp.road.positions, &resp.road.normals, &resp.road.uvs),
                    &resp.road.indices,
                );
                let mk_strips = |strips: &Vec<Vec<f32>>| -> Vec<LineMesh> {
                    strips
                        .iter()
                        .map(|s| {
                            let mut m = LineMesh::new();
                            m.upload(&self.gl, s);
                            m
                        })
                        .collect()
                };
                self.edge_c = mk_strips(&resp.edge_c);
                self.edge_f = mk_strips(&resp.edge_f);
                self.edge_r = mk_strips(&resp.edge_r);
                self.center = mk_strips(&resp.centerline);
                self.junction.upload(
                    &self.gl,
                    &interleave(
                        &resp.junctions.positions,
                        &resp.junctions.normals,
                        &resp.junctions.uvs,
                    ),
                    &resp.junctions.indices,
                );
                self.jmarks.upload(
                    &self.gl,
                    &interleave(
                        &resp.junction_marks.positions,
                        &resp.junction_marks.normals,
                        &resp.junction_marks.uvs,
                    ),
                    &resp.junction_marks.indices,
                );
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
                let jc = resp.junction_count;
                self.ctrl_from();
                let mut chips = vec![
                    chip("wasm32-unknown-unknown"),
                    chip(&format!("{} m · {} loops", len as i32, loops)),
                    chip(&format!("{} verts", verts)),
                    chip(&format!("{} tris · {} lanes", tris, nl)),
                ];
                if jc > 0 {
                    chips.push(chip(&format!("{} junctions", jc)));
                }
                chips.push(chip(&format!("rust {:.2} ms", self.perf.now() - t0)));
                self.set_stats(&chips);
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
                self.center = vec![{
                    let mut m = LineMesh::new();
                    m.upload(&self.gl, &resp.centerline);
                    m
                }];
                self.edge_c.clear();
                self.edge_f.clear();
                self.edge_r.clear();
                self.lanes.clear();
                let verts = resp.vertex_count;
                let tris = resp.triangle_count;
                let len = resp.length;
                self.ctrl_from();
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

    fn ctrl_from(&mut self) {
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
        let n = self.flat_len();
        set_text("pcount", &format!("Points: {n}"));
    }

    fn sync_sel(&mut self) {
        match self.selected {
            Some(k) => {
                let n = self.flat_len();
                if k >= n {
                    self.selected = None;
                    set_display("selBox", "none");
                    return;
                }
                set_display("selBox", "block");
                match self.mode {
                    0 => {
                        let (ri, pi) = self.flat_loc(k);
                        let p = &self.g_roads[ri][pi];
                        set_text(
                            "selTitle",
                            &if ri == 0 {
                                format!("RoadPoint #{pi}")
                            } else {
                                format!("RoadPoint #{pi} · road {ri}")
                            },
                        );
                        el_input("sel-prior").set_value_as_number(p.prior_mag as f64);
                        el_input("sel-next").set_value_as_number(p.next_mag as f64);
                        el_input("sel-elev").set_value_as_number(p.y as f64);
                        el_input("sel-lw").set_value_as_number(p.lane_width as f64);
                        el_input("sel-shl").set_value_as_number(p.shoulder_width_l as f64);
                        el_input("sel-shr").set_value_as_number(p.shoulder_width_r as f64);
                        el_input("sel-gx").set_value_as_number(p.gutter_profile_x as f64);
                        el_input("sel-gy").set_value_as_number(p.gutter_profile_y as f64);
                        el_input("sel-und").set_value_as_number(p.underside_thickness as f64);
                        el_input("sel-eeo")
                            .set_value_as_number(p.edge_edge_clipping_offset as f64);
                        el_input("sel-term").set_checked(p.terminated);
                        el_input("sel-gmi").set_checked(p.generate_mesh_at_intersections);
                        set_text("sel-prior-v", &format!("{:.1}", p.prior_mag));
                        set_text("sel-next-v", &format!("{:.1}", p.next_mag));
                        set_text("sel-elev-v", &format!("{:.1} m", p.y));
                        set_text("sel-lw-v", &format!("{:.2}", p.lane_width));
                        set_text("sel-shl-v", &format!("{:.2}", p.shoulder_width_l));
                        set_text("sel-shr-v", &format!("{:.2}", p.shoulder_width_r));
                        set_text("sel-gx-v", &format!("{:.2}", p.gutter_profile_x));
                        set_text("sel-gy-v", &format!("{:.2}", p.gutter_profile_y));
                        set_text(
                            "sel-und-v",
                            &if p.underside_thickness < -0.95 {
                                "inherit".to_string()
                            } else {
                                format!("{:.2}", p.underside_thickness)
                            },
                        );
                        set_text("sel-eeo-v", &format!("{:.2}", p.edge_edge_clipping_offset));
                        set_text(
                            "sel-pos",
                            &format!("x {:.1} · z {:.1}", p.x, p.z),
                        );
                        let geo: HtmlButtonElement = el("alignGeo").dyn_into().unwrap();
                        let dv: HtmlButtonElement = el("alignDiv").dyn_into().unwrap();
                        geo.set_class_name(if p.alignment == 0 {
                            "segbtn active"
                        } else {
                            "segbtn"
                        });
                        dv.set_class_name(if p.alignment == 1 {
                            "segbtn active"
                        } else {
                            "segbtn"
                        });
                        let badge: HtmlElement = el("selBadge").dyn_into().unwrap();
                        match self.junction_kind_at(ri, pi) {
                            Some(kind) => {
                                badge.set_text_content(Some(&format!("{kind}-way")));
                                badge.set_class_name(if kind >= 4 {
                                    "jbadge"
                                } else {
                                    "jbadge t"
                                });
                                badge
                                    .set_attribute("style", "display:inline-block")
                                    .ok();
                            }
                            None => {
                                badge.set_attribute("style", "display:none").ok();
                            }
                        }
                        self.build_lane_rows();
                    }
                    _ => {
                        let node = &self.a_nodes[k];
                        set_text("selTitle", &format!("Node #{k}"));
                        el_input("sel-easein").set_value_as_number(node.ease_in as f64);
                        el_input("sel-easeout").set_value_as_number(node.ease_out as f64);
                        el_input("sel-elev").set_value_as_number(node.y as f64);
                        set_text("sel-easein-v", &format!("{:.2}", node.ease_in));
                        set_text("sel-easeout-v", &format!("{:.2}", node.ease_out));
                        set_text("sel-elev-v", &format!("{:.1} m", node.y));
                    }
                }
            }
            None => {
                set_display("selBox", "none");
            }
        }
    }

    // -- lanes editor (godot traffic_dir parity) ----------------------------

    fn build_lane_rows(&mut self) {
        let cont = el("laneRows");
        cont.set_inner_html("");
        let Some(k) = self.selected else { return };
        if self.mode != 0 {
            return;
        }
        let (ri, pi) = self.flat_loc(k);
        let dirs = self.g_roads[ri][pi].traffic_dir.clone();
        let app = APP.with(|a| a.borrow().clone()).unwrap();
        for (li, d) in dirs.iter().enumerate() {
            let row = doc()
                .create_element("div")
                .unwrap()
                .dyn_into::<HtmlDivElement>()
                .unwrap();
            row.set_class_name("lanerow");
            let chip_b = doc()
                .create_element("button")
                .unwrap()
                .dyn_into::<HtmlButtonElement>()
                .unwrap();
            chip_b.set_class_name(&format!("dirchip d{d}"));
            chip_b
                .set_attribute("title", &format!("Cycle direction (now {})", dir_name(*d)))
                .ok();
            let lab = doc()
                .create_element("span")
                .unwrap()
                .dyn_into::<HtmlSpanElement>()
                .unwrap();
            lab.set_class_name("lanelab");
            lab.set_text_content(Some(&format!("Lane {li} — {}", dir_name(*d))));
            let del = doc()
                .create_element("button")
                .unwrap()
                .dyn_into::<HtmlButtonElement>()
                .unwrap();
            del.set_class_name("lanedel");
            del.set_text_content(Some("×"));
            del.set_attribute("title", "Remove lane").ok();
            let cb_chip = {
                let app = app.clone();
                let li = li;
                Closure::<dyn FnMut(JsValue)>::new(move |_e| {
                    app.borrow_mut().cycle_lane(li);
                })
            };
            let cb_del = {
                let app = app.clone();
                let li = li;
                Closure::<dyn FnMut(JsValue)>::new(move |_e| {
                    app.borrow_mut().remove_lane(li);
                })
            };
            chip_b
                .add_event_listener_with_callback("click", cb_chip.as_ref().unchecked_ref())
                .unwrap();
            del.add_event_listener_with_callback("click", cb_del.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb_chip);
            self.closures.push(cb_del);
            row.append_child(&chip_b).unwrap();
            row.append_child(&lab).unwrap();
            row.append_child(&del).unwrap();
            cont.append_child(&row).unwrap();
        }
        if dirs.is_empty() {
            let s = doc()
                .create_element("div")
                .unwrap()
                .dyn_into::<HtmlDivElement>()
                .unwrap();
            s.set_class_name("lanehint");
            s.set_text_content(Some("No lanes — add one below"));
            cont.append_child(&s).unwrap();
        }
    }

    fn cycle_lane(&mut self, li: usize) {
        self.patch_sel_godot(|p| {
            if let Some(d) = p.traffic_dir.get_mut(li) {
                *d = (*d + 1) % 4;
            }
        });
        self.regen_keep();
    }

    fn remove_lane(&mut self, li: usize) {
        self.patch_sel_godot(|p| {
            if p.traffic_dir.len() > 1 {
                p.traffic_dir.remove(li);
            }
        });
        self.regen_keep();
    }

    fn add_lane(&mut self) {
        self.patch_sel_godot(|p| {
            let d = p.traffic_dir.last().copied().unwrap_or(1);
            p.traffic_dir.push(d);
        });
        self.regen_keep();
    }

    fn set_alignment(&mut self, al: u8) {
        self.patch_sel_godot(|p| p.alignment = al);
        self.regen_keep();
    }

    fn regen_keep(&mut self) {
        self.regenerate();
        self.sync_sel();
    }

    fn patch_selected_y(&mut self, y: f32) {
        match (self.selected, self.mode) {
            (Some(k), 0) => {
                let (ri, pi) = self.flat_loc(k);
                self.g_roads[ri][pi].y = y;
            }
            (Some(k), _) => {
                if k < self.a_nodes.len() {
                    self.a_nodes[k].y = y;
                }
            }
            _ => {}
        }
        self.regenerate();
    }

    // -- ui wiring ----------------------------------------------------------

    fn build_presets(&mut self) {
        let cont = el("presets");
        cont.set_inner_html("");
        let presets: Vec<(&'static str, Vec<Vec<[f32; 3]>>, Vec<u8>)> = if self.mode == 0 {
            godot_presets()
        } else {
            arch_presets()
                .into_iter()
                .map(|(n, pts, d)| (n, vec![pts], d))
                .collect()
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
        keep!(bind_slider(&app, "sel-lw", "sel-lw-v", |a, v| a
            .patch_sel_godot(|p| p.lane_width = v)));
        keep!(bind_slider(&app, "sel-shl", "sel-shl-v", |a, v| a
            .patch_sel_godot(|p| p.shoulder_width_l = v)));
        keep!(bind_slider(&app, "sel-shr", "sel-shr-v", |a, v| a
            .patch_sel_godot(|p| p.shoulder_width_r = v)));
        keep!(bind_slider(&app, "sel-gx", "sel-gx-v", |a, v| a
            .patch_sel_godot(|p| p.gutter_profile_x = v)));
        keep!(bind_slider(&app, "sel-gy", "sel-gy-v", |a, v| a
            .patch_sel_godot(|p| p.gutter_profile_y = v)));
        keep!(bind_slider(&app, "sel-eeo", "sel-eeo-v", |a, v| a
            .patch_sel_godot(|p| p.edge_edge_clipping_offset = v)));
        // underside override: -1 renders as "inherit"
        {
            let cb = {
                let app = app.clone();
                Closure::<dyn FnMut(JsValue)>::new(move |_e| {
                    let v = el_input("sel-und").value_as_number() as f32;
                    set_text(
                        "sel-und-v",
                        &if v < -0.95 {
                            "inherit".to_string()
                        } else {
                            format!("{v:.2}")
                        },
                    );
                    app.borrow_mut()
                        .regen_with(|a| a.patch_sel_godot(|p| p.underside_thickness = v));
                })
            };
            el_input("sel-und")
                .add_event_listener_with_callback("input", cb.as_ref().unchecked_ref())
                .unwrap();
            self.closures.push(cb);
        }

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
        keep!(bind_check(&app, "jmarks", |a, v| a.g_jmarks = v));
        keep!(bind_check(&app, "sel-term", |a, v| a
            .patch_sel_godot(|p| p.terminated = v)));
        keep!(bind_check(&app, "sel-gmi", |a, v| a
            .patch_sel_godot(|p| p.generate_mesh_at_intersections = v)));

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
        keep!(bind_click(&app, "laneAdd", |a| a.add_lane()));
        keep!(bind_click(&app, "alignGeo", |a| a.set_alignment(0)));
        keep!(bind_click(&app, "alignDiv", |a| a.set_alignment(1)));
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
        if let Some(k) = self.selected {
            if self.mode == 0 {
                let (ri, pi) = self.flat_loc(k);
                if ri < self.g_roads.len() && pi < self.g_roads[ri].len() {
                    f(&mut self.g_roads[ri][pi]);
                }
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
            let partner = self.find_partner(i);
            self.drag = Some(Drag::Node(i, partner));
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
            Some(Drag::Node(i, partner)) => {
                let i = *i;
                let partner = *partner;
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
                            let (ri, pi) = self.flat_loc(i);
                            if ri < self.g_roads.len() && pi < self.g_roads[ri].len() {
                                self.g_roads[ri][pi].x = x;
                                self.g_roads[ri][pi].z = z;
                            }
                            // Junction partner stays glued to the shared point.
                            if let Some(m) = partner {
                                let (rj, pj) = self.flat_loc(m);
                                if rj < self.g_roads.len() && pj < self.g_roads[rj].len() {
                                    self.g_roads[rj][pj].x = x;
                                    self.g_roads[rj][pj].z = z;
                                }
                            }
                            let (ri, pi) = self.flat_loc(i);
                            if ri < self.g_roads.len() && pi < self.g_roads[ri].len() {
                                let p = &self.g_roads[ri][pi];
                                set_text("sel-pos", &format!("x {:.1} · z {:.1}", p.x, p.z));
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

        if self.mode == 0 {
            // Junction pavement n-gons + stop lines / crosswalks.
            gl.uniform1i(self.surf.u_kind.as_ref(), 1);
            gl.uniform3f(self.surf.u_color.as_ref(), 0.21, 0.22, 0.23);
            self.junction.draw(gl);
            if self.g_jmarks {
                gl.uniform3f(self.surf.u_color.as_ref(), 0.93, 0.93, 0.9);
                self.jmarks.draw(gl);
            }
        }

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
            for m in &self.edge_c {
                m.draw(gl);
            }
            gl.uniform3f(self.line.u_color.as_ref(), WHITE_L[0], WHITE_L[1], WHITE_L[2]);
            for m in self.edge_f.iter().chain(self.edge_r.iter()) {
                m.draw(gl);
            }
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
            for m in &self.center {
                m.draw(gl);
            }
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
