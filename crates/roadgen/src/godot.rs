//! roadgen v2 — Rust port of `TheDuckCow/godot-road-generator` core geometry.
//!
//! Parity targets (see docs/godot-road-parity.md):
//!   * RoadPoint: lane_width, shoulder_width_l/r, gutter_profile(x,y),
//!     prior_mag/next_mag (Bézier handles), traffic_dir lanes, alignment,
//!     underside_thickness, terminated, generate_mesh_at_intersections,
//!     edge_edge_clipping_offset.
//!   * RoadSegment: loop cuts every `density` meters (DEFAULT_DENSITY=4.0),
//!     LOWPOLY_FACTOR=3.0, 8-column trimsheet UVs with uv_mul
//!     = [7,0,1,2,3,4,5,6,7,7], UV flow snap rules from _build_geo(),
//!     edge_C/edge_F/edge_R curves, per-lane paths (RoadLane parity).
//!   * RoadManager: density, underside_thickness, auto_lanes, low_poly.
//!   * RoadIntersection / intersection_ngon (Phase 3): roads that share a
//!     RoadPoint (proximity join, RoadManager parity) form junctions —
//!     approaches are trimmed back when generate_mesh_at_intersections is on
//!     (extended by edge_edge_clipping_offset) and the gap is filled with a
//!     chamfered pavement n-gon plus stop lines and zebra crosswalks.
//!
//! Documented deltas: lateral frame is tangent-derived (Godot eases endpoint
//! bases); lane transitions blend widths linearly instead of the full
//! _match_lanes TRANSITION_ADD/REM matcher (Phase 2); junction n-gon is a
//! chamfered octagon instead of the full dynamic re-paving solver.

#![allow(clippy::needless_range_loop)]

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

// --- godot-road-generator constants (verbatim parity) ---
pub const DEFAULT_LANE_WIDTH: f32 = 4.0; // RoadPoint.DEFAULT_LANE_WIDTH
pub const DEFAULT_DENSITY: f32 = 4.0; // RoadSegment.DEFAULT_DENSITY
pub const LOWPOLY_FACTOR: f32 = 3.0; // RoadSegment.LOWPOLY_FACTOR
pub const DEFAULT_MAG: f32 = 5.0; // RoadPoint.prior_mag / next_mag
/// RoadSegment.uv_mul — LaneType enum → trimsheet UV column.
pub const UV_MUL: [u8; 10] = [7, 0, 1, 2, 3, 4, 5, 6, 7, 7];

// LaneType (godot enum order)
pub const LT_NO_MARKING: u8 = 0;
pub const LT_SHOULDER: u8 = 1;
pub const LT_SLOW: u8 = 2;
pub const LT_MIDDLE: u8 = 3;
pub const LT_FAST: u8 = 4;
pub const LT_TWO_WAY: u8 = 5;
pub const LT_ONE_WAY: u8 = 6;
pub const LT_SINGLE_LINE: u8 = 7;
// 8 TRANSITION_ADD / 9 TRANSITION_REM — Phase 2 matcher.

// LaneDir (godot enum order)
pub const LD_NONE: u8 = 0;
pub const LD_FORWARD: u8 = 1;
pub const LD_REVERSE: u8 = 2;
pub const LD_BOTH: u8 = 3;

// Alignment
pub const AL_GEOMETRIC: u8 = 0;
pub const AL_DIVIDER: u8 = 1;

// ---------------------------------------------------------------------------
// Small math (Vec3 / bezier)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    fn new(x: f32, y: f32, z: f32) -> Self {
        Vec3 { x, y, z }
    }
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
    fn scale(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
    fn dist(self, o: Vec3) -> f32 {
        self.sub(o).mag()
    }
    fn mag(self) -> f32 {
        self.dot(self).sqrt()
    }
    fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    fn normalized(self) -> Vec3 {
        let m = self.mag();
        if m < 1e-8 {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            self.scale(1.0 / m)
        }
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Cubic Bézier with explicit handles (Godot Curve3D add_point(pos, in, out)).
#[derive(Clone, Copy)]
struct Bezier {
    p0: Vec3,
    c1: Vec3, // p0 + fwd0 * next_mag0
    c2: Vec3, // p1 - fwd1 * prior_mag1
    p1: Vec3,
}

impl Bezier {
    fn eval(&self, t: f32) -> Vec3 {
        let u = 1.0 - t;
        let a = u * u * u;
        let b = 3.0 * u * u * t;
        let c = 3.0 * u * t * t;
        let d = t * t * t;
        self.p0
            .scale(a)
            .add(self.c1.scale(b))
            .add(self.c2.scale(c))
            .add(self.p1.scale(d))
    }
    fn deriv(&self, t: f32) -> Vec3 {
        let u = 1.0 - t;
        self.c1
            .sub(self.p0)
            .scale(3.0 * u * u)
            .add(self.c2.sub(self.c1).scale(6.0 * u * t))
            .add(self.p1.sub(self.c2).scale(3.0 * t * t))
    }
    fn approx_length(&self) -> f32 {
        let mut len = 0.0;
        let mut prev = self.p0;
        for i in 1..=48 {
            let p = self.eval(i as f32 / 48.0);
            len += p.dist(prev);
            prev = p;
        }
        len
    }
}

// ---------------------------------------------------------------------------
// Request model — mirrors RoadManager/RoadContainer/RoadPoint properties
// ---------------------------------------------------------------------------

#[derive(Deserialize, Clone)]
pub struct RoadPointData {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    #[serde(default = "d_mag")]
    pub prior_mag: f32,
    #[serde(default = "d_mag")]
    pub next_mag: f32,
    #[serde(default = "d_lane_width")]
    pub lane_width: f32,
    #[serde(default = "d_shoulder")]
    pub shoulder_width_l: f32,
    #[serde(default = "d_shoulder")]
    pub shoulder_width_r: f32,
    #[serde(default = "d_gutter_x")]
    pub gutter_profile_x: f32,
    #[serde(default = "d_gutter_y")]
    pub gutter_profile_y: f32,
    /// Array[LaneDir]: 0=NONE 1=FORWARD 2=REVERSE 3=BOTH (godot order).
    #[serde(default = "d_traffic_dir")]
    pub traffic_dir: Vec<u8>,
    #[serde(default)]
    pub alignment: u8, // 0=GEOMETRIC 1=DIVIDER
    #[serde(default)]
    pub underside_thickness: f32, // -1 → inherit manager
    #[serde(default)]
    pub terminated: bool,
    /// godot RoadPoint.generate_mesh_at_intersections — trim the mesh back
    /// where this point joins another road so the junction n-gon can fill it.
    #[serde(default = "d_true")]
    pub generate_mesh_at_intersections: bool,
    /// godot RoadPoint.edge_edge_clipping_offset — extra meters trimmed past
    /// the junction pavement edge.
    #[serde(default)]
    pub edge_edge_clipping_offset: f32,
}

#[derive(Deserialize)]
pub struct ManagerData {
    #[serde(default = "d_density")]
    pub density: f32,
    #[serde(default)]
    pub underside_thickness: f32, // -1 = none (godot manager default)
    #[serde(default)]
    pub low_poly: bool,
    #[serde(default = "d_true")]
    pub auto_lanes: bool,
}

#[derive(Deserialize)]
pub struct GenRequest {
    #[serde(default)]
    pub manager: ManagerData,
    /// Legacy single-road input (used when `roads` is empty).
    #[serde(default)]
    pub points: Vec<RoadPointData>,
    /// Road network: roads[0] is the main road; any point of roads[1..]
    /// coinciding with a main-road point forms a junction (RoadManager
    /// proximity parity).
    #[serde(default)]
    pub roads: Vec<Vec<RoadPointData>>,
}

fn d_mag() -> f32 {
    DEFAULT_MAG
}
fn d_lane_width() -> f32 {
    DEFAULT_LANE_WIDTH
}
fn d_shoulder() -> f32 {
    2.0
}
fn d_gutter_x() -> f32 {
    2.0
}
fn d_gutter_y() -> f32 {
    -0.5
}
fn d_density() -> f32 {
    DEFAULT_DENSITY
}
fn d_true() -> bool {
    true
}
fn d_traffic_dir() -> Vec<u8> {
    vec![LD_REVERSE, LD_REVERSE, LD_FORWARD, LD_FORWARD]
}

impl Default for ManagerData {
    fn default() -> Self {
        ManagerData {
            density: DEFAULT_DENSITY,
            underside_thickness: -1.0,
            low_poly: false,
            auto_lanes: true,
        }
    }
}

// ---------------------------------------------------------------------------
// Cross-section model (per RoadPoint — godot _build_geo / GeoLoopInfo parity)
// ---------------------------------------------------------------------------

#[derive(Clone, Copy)]
struct ColSpec {
    kind: u8, // LaneType → trimsheet column
    width: f32,
    y_drop: f32,
    mirror: bool, // flip UV.x inside the trimsheet column
    is_lane: bool,
    dir: u8, // LaneDir for lane paths
}

/// Builds the cross-section for a RoadPoint, columns ordered left→right:
/// gutterL | shoulderL | REVERSE lanes (outer SLOW … divider-adjacent FAST) |
/// FORWARD lanes (divider-adjacent FAST, mirrored … outer SLOW) |
/// shoulderR | gutterR.
fn build_section(p: &RoadPointData) -> Vec<ColSpec> {
    let mut cols: Vec<ColSpec> = Vec::new();
    let gutter = |mirror: bool| ColSpec {
        kind: LT_NO_MARKING,
        width: p.gutter_profile_x.max(0.0),
        y_drop: p.gutter_profile_y,
        mirror,
        is_lane: false,
        dir: LD_NONE,
    };
    let shoulder = |mirror: bool, w: f32| ColSpec {
        kind: LT_SHOULDER,
        width: w.max(0.0),
        y_drop: 0.0,
        mirror,
        is_lane: false,
        dir: LD_NONE,
    };

    let n_rev = p.traffic_dir.iter().filter(|d| **d == LD_REVERSE).count();
    let n_fwd = p
        .traffic_dir
        .iter()
        .filter(|d| **d == LD_FORWARD || **d == LD_BOTH)
        .count();

    // Lane trimsheet column by position (0 = outermost on the side):
    // divider-adjacent lane is always FAST (double-yellow faces the divider),
    // outermost is SLOW (solid white on the outer edge), middles are MIDDLE.
    let lane_col = |k: usize, n: usize| -> u8 {
        if n == 1 || k == n - 1 {
            LT_FAST
        } else if k == 0 {
            LT_SLOW
        } else {
            LT_MIDDLE
        }
    };

    cols.push(gutter(false));
    cols.push(shoulder(false, p.shoulder_width_l));
    for k in 0..n_rev {
        cols.push(ColSpec {
            kind: lane_col(k, n_rev),
            width: p.lane_width,
            y_drop: 0.0,
            mirror: false,
            is_lane: true,
            dir: LD_REVERSE,
        });
    }
    for k in 0..n_fwd {
        cols.push(ColSpec {
            kind: lane_col(k, n_fwd),
            width: p.lane_width,
            y_drop: 0.0,
            mirror: true, // mirrored so yellow/white lines face correctly
            is_lane: true,
            dir: LD_FORWARD,
        });
    }
    cols.push(shoulder(true, p.shoulder_width_r));
    cols.push(gutter(true));
    cols.retain(|c| c.width > 0.02);
    cols
}

/// Half of the full surface width (junction sizing).
fn section_half_width(p: &RoadPointData) -> f32 {
    build_section(p).iter().map(|c| c.width).sum::<f32>() * 0.5
}

// ---------------------------------------------------------------------------
// Unified column (start/end cross-section blend — _match_lanes approximation)
// ---------------------------------------------------------------------------

struct UniCol {
    kind_s: u8,
    kind_e: u8,
    w_s: f32,
    w_e: f32,
    y_s: f32,
    y_e: f32,
    mirror_s: bool,
    mirror_e: bool,
    is_lane: bool,
    dir: u8,
}

impl UniCol {
    fn is_lane_fwd(&self) -> bool {
        self.is_lane && self.dir == LD_FORWARD
    }
    fn is_lane_rev(&self) -> bool {
        self.is_lane && self.dir == LD_REVERSE
    }
}

/// Pairs columns from the divider outward on each side so mismatched lane
/// counts collapse/expand smoothly across a segment (approximation of
/// godot's _match_lanes TRANSITION_ADD/REM — full matcher is Phase 2).
fn unify_sections(sec_s: &[ColSpec], sec_e: &[ColSpec]) -> Vec<UniCol> {
    let split = |sec: &[ColSpec]| -> (Vec<ColSpec>, Vec<ColSpec>) {
        let div = sec
            .iter()
            .position(|c| c.is_lane && c.dir == LD_FORWARD)
            .unwrap_or(sec.len());
        (sec[..div].to_vec(), sec[div..].to_vec())
    };
    let (ls, rs) = split(sec_s);
    let (le, re) = split(sec_e);

    let mut out: Vec<UniCol> = Vec::new();

    // LEFT side: pair from the divider outward, then reverse for left→right.
    let na = ls.len();
    let nb = le.len();
    let count = na.max(nb);
    let mut left_div_out: Vec<UniCol> = Vec::new();
    for k in 0..count {
        let ia = na.checked_sub(1 + k); // divider-outward index
        let ib = nb.checked_sub(1 + k);
        left_div_out.push(pair_cols(
            ls.get(ia.unwrap_or(0)),
            ia.is_some(),
            le.get(ib.unwrap_or(0)),
            ib.is_some(),
        ));
    }
    left_div_out.reverse(); // outer → divider (left→right order)
    out.extend(left_div_out);

    // RIGHT side: pair from the divider outward, already left→right order.
    let na = rs.len();
    let nb = re.len();
    let count = na.max(nb);
    for k in 0..count {
        out.push(pair_cols(rs.get(k), k < na, re.get(k), k < nb));
    }
    out
}

fn pair_cols(a: Option<&ColSpec>, a_ok: bool, b: Option<&ColSpec>, b_ok: bool) -> UniCol {
    let missing = ColSpec {
        kind: LT_NO_MARKING,
        width: 0.0,
        y_drop: 0.0,
        mirror: false,
        is_lane: false,
        dir: LD_NONE,
    };
    let ca = if a_ok { a.copied().unwrap_or(missing) } else { missing };
    let cb = if b_ok { b.copied().unwrap_or(missing) } else { missing };
    let (ks, ke, ms, me) = if !a_ok {
        (cb.kind, cb.kind, cb.mirror, cb.mirror)
    } else if !b_ok {
        (ca.kind, ca.kind, ca.mirror, ca.mirror)
    } else {
        (ca.kind, cb.kind, ca.mirror, cb.mirror)
    };
    UniCol {
        kind_s: ks,
        kind_e: ke,
        w_s: ca.width,
        w_e: cb.width,
        y_s: ca.y_drop,
        y_e: cb.y_drop,
        mirror_s: ms,
        mirror_e: me,
        is_lane: ca.is_lane || cb.is_lane,
        dir: if ca.is_lane { ca.dir } else { cb.dir },
    }
}

// ---------------------------------------------------------------------------
// Mesh output
// ---------------------------------------------------------------------------

#[derive(Serialize, Default, Clone)]
pub struct MeshData {
    pub positions: Vec<f32>,
    pub normals: Vec<f32>,
    pub uvs: Vec<f32>,
    pub indices: Vec<u32>,
}

impl MeshData {
    fn push_vertex(&mut self, p: Vec3, n: Vec3, u: f32, v: f32) -> u32 {
        let idx = (self.positions.len() / 3) as u32;
        self.positions.extend_from_slice(&[p.x, p.y, p.z]);
        self.normals.extend_from_slice(&[n.x, n.y, n.z]);
        self.uvs.extend_from_slice(&[u, v]);
        idx
    }
    fn push_quad(&mut self, a: u32, b: u32, c: u32, d: u32) {
        self.indices.extend_from_slice(&[a, c, b, b, c, d]);
    }
}

#[derive(Serialize, Clone)]
pub struct LanePath {
    /// x,y,z triplets — parity of a godot RoadLane Path3D curve.
    pub path: Vec<f32>,
    pub dir: u8,
    pub lane_type: u8,
}

/// One detected junction (RoadIntersection parity).
#[derive(Serialize, Clone)]
pub struct JunctionInfo {
    /// Number of meeting road arms: 4 = cross, 3 = T, 2 = corner.
    pub kind: u8,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    /// Index into roads[0] (main road) of the shared RoadPoint.
    pub main_idx: usize,
    /// Cross road index (≥1) and point index of the shared RoadPoint.
    pub cross_road: usize,
    pub cross_idx: usize,
}

#[derive(Serialize, Default)]
pub struct GenResponse {
    pub road: MeshData,
    /// edge_C / edge_F / edge_R polylines per segment (RoadSegment edge-curve
    /// parity; split at junction trims so lines never cross the n-gon).
    pub edge_c: Vec<Vec<f32>>,
    pub edge_f: Vec<Vec<f32>>,
    pub edge_r: Vec<Vec<f32>>,
    /// Per-lane center paths (RoadLane parity).
    pub lanes: Vec<LanePath>,
    /// Centerline polyline per segment (editor visual).
    pub centerline: Vec<Vec<f32>>,
    /// Junction pavement n-gons (render as flat asphalt).
    pub junctions: MeshData,
    /// Junction markings: stop lines + zebra crosswalks (flat white).
    pub junction_marks: MeshData,
    pub junction_count: u32,
    pub junction_info: Vec<JunctionInfo>,
    pub loop_count: u32,
    pub length: f32,
    pub vertex_count: u32,
    pub triangle_count: u32,
    pub elapsed_us: u32,
    pub error: Option<String>,
}

const UP: Vec3 = Vec3 {
    x: 0.0,
    y: 1.0,
    z: 0.0,
};

// ---------------------------------------------------------------------------
// Junctions (RoadIntersection / intersection_ngon parity — Phase 3)
// ---------------------------------------------------------------------------

/// Two points closer than this share a RoadPoint (RoadManager proximity).
pub const JUNCTION_EPS: f32 = 0.75;
/// Pavement padding past the road edges.
const JUNCTION_MARGIN: f32 = 1.2;
/// Pavement overlap into the trimmed road ends (hides the seam).
const JUNCTION_SLACK: f32 = 0.45;
const JUNCTION_LIFT: f32 = 0.025;
const MARK_LIFT: f32 = 0.05;

struct Junction {
    info: JunctionInfo,
    /// Horizontal tangents at the junction point.
    main_tan: Vec3,
    cross_tan: Vec3,
    main_half: f32,
    cross_half: f32,
    /// Unit arm directions pointing AWAY from the junction per road.
    main_arms: Vec<Vec3>,
    cross_arms: Vec<Vec3>,
}

/// Flattens a vector to the horizontal plane (junction frames are 2D).
fn horiz(v: Vec3) -> Vec3 {
    let h = Vec3::new(v.x, 0.0, v.z);
    if h.mag() < 1e-6 {
        Vec3::new(1.0, 0.0, 0.0)
    } else {
        h.normalized()
    }
}

/// Node forward vectors (central difference; RoadPoint basis.z parity).
fn forwards(pts: &[RoadPointData]) -> Vec<Vec3> {
    let n = pts.len();
    let mut fwd: Vec<Vec3> = Vec::with_capacity(n);
    for i in 0..n {
        let p = Vec3::new(pts[i].x, pts[i].y, pts[i].z);
        let f = if i == 0 {
            Vec3::new(pts[1].x, pts[1].y, pts[1].z).sub(p)
        } else if i == n - 1 {
            p.sub(Vec3::new(pts[i - 1].x, pts[i - 1].y, pts[i - 1].z))
        } else {
            let a = p
                .sub(Vec3::new(pts[i - 1].x, pts[i - 1].y, pts[i - 1].z))
                .normalized();
            let b = Vec3::new(pts[i + 1].x, pts[i + 1].y, pts[i + 1].z)
                .sub(p)
                .normalized();
            a.add(b)
        };
        fwd.push(f.normalized());
    }
    fwd
}

fn detect_junctions(roads: &[&[RoadPointData]], fwds: &[Vec<Vec3>]) -> Vec<Junction> {
    let mut out: Vec<Junction> = Vec::new();
    for (ri, road) in roads.iter().enumerate().skip(1) {
        if road.is_empty() {
            continue;
        }
        for (j, cp) in road.iter().enumerate() {
            let cpos = Vec3::new(cp.x, cp.y, cp.z);
            let mut best: Option<(usize, f32)> = None;
            for (i, mp) in roads[0].iter().enumerate() {
                let d = cpos.dist(Vec3::new(mp.x, mp.y, mp.z));
                if d < JUNCTION_EPS && best.map(|(_, bd)| d < bd).unwrap_or(true) {
                    best = Some((i, d));
                }
            }
            let Some((i, _)) = best else { continue };
            if out
                .iter()
                .any(|jn: &Junction| jn.info.cross_road == ri && jn.info.main_idx == i)
            {
                continue;
            }
            let main_tan = horiz(fwds[0][i]);
            let cross_tan = horiz(fwds[ri][j]);

            // Arms pointing away from the junction along each road.
            let main_arms: Vec<Vec3> = if roads[0].len() < 2 {
                vec![]
            } else if i == 0 {
                vec![main_tan]
            } else if i == roads[0].len() - 1 {
                vec![main_tan.scale(-1.0)]
            } else {
                vec![main_tan, main_tan.scale(-1.0)]
            };
            let cross_arms: Vec<Vec3> = if road.len() < 2 {
                vec![]
            } else if j == 0 {
                vec![cross_tan]
            } else if j == road.len() - 1 {
                vec![cross_tan.scale(-1.0)]
            } else {
                vec![cross_tan, cross_tan.scale(-1.0)]
            };

            out.push(Junction {
                info: JunctionInfo {
                    kind: (main_arms.len() + cross_arms.len()).min(4) as u8,
                    x: cp.x,
                    y: cp.y,
                    z: cp.z,
                    main_idx: i,
                    cross_road: ri,
                    cross_idx: j,
                },
                main_tan,
                cross_tan,
                main_half: section_half_width(&roads[0][i]),
                cross_half: section_half_width(cp),
                main_arms,
                cross_arms,
            });
        }
    }
    out
}

/// Chamfered octagon pavement n-gon covering the meeting of both road
/// corridors (intersection_ngon parity, simplified to a convex plate).
fn push_junction_pavement(jn: &Junction, md: &mut MeshData) {
    let c = Vec3::new(jn.info.x, jn.info.y, jn.info.z);
    let tx = jn.main_tan; // along the main road
    let tz = jn.cross_tan; // along the cross road
    let a = jn.cross_half + JUNCTION_MARGIN; // half-extent along main tangent
    let b = jn.main_half + JUNCTION_MARGIN; // half-extent along cross tangent
    let ch = (a.min(b) * 0.5).min(2.6); // corner chamfer
    let y = c.y + JUNCTION_LIFT;

    let rim: [[f32; 2]; 8] = [
        [a - ch, b],
        [a, b - ch],
        [a, -(b - ch)],
        [a - ch, -b],
        [-(a - ch), -b],
        [-a, -(b - ch)],
        [-a, b - ch],
        [-(a - ch), b],
    ];
    let center = md.push_vertex(Vec3::new(c.x, y, c.z), UP, 0.0, 0.0);
    let mut ring: Vec<u32> = Vec::with_capacity(8);
    for [sx, sz] in rim {
        let p = c.add(tx.scale(sx)).add(tz.scale(sz));
        ring.push(md.push_vertex(Vec3::new(p.x, y, p.z), UP, 0.0, 0.0));
    }
    for k in 0..8 {
        let n2 = (k + 1) % 8;
        md.indices.extend_from_slice(&[center, ring[k], ring[n2]]);
    }
}

/// Flat quad in the junction frame: t = along-arm axis, l = lateral axis.
#[allow(clippy::too_many_arguments)]
fn push_flat_quad(
    md: &mut MeshData,
    c: Vec3,
    t_dir: Vec3,
    l_dir: Vec3,
    t0: f32,
    t1: f32,
    l0: f32,
    l1: f32,
    y: f32,
) {
    let p = |t: f32, l: f32| {
        Vec3::new(
            c.x + t_dir.x * t + l_dir.x * l,
            y,
            c.z + t_dir.z * t + l_dir.z * l,
        )
    };
    let a = md.push_vertex(p(t0, l0), UP, 0.0, 0.0);
    let b = md.push_vertex(p(t1, l0), UP, 0.0, 0.0);
    let d = md.push_vertex(p(t1, l1), UP, 0.0, 0.0);
    let e = md.push_vertex(p(t0, l1), UP, 0.0, 0.0);
    md.push_quad(a, b, d, e);
}

/// Stop line + zebra crosswalk on one junction arm.
fn push_arm_markings(md: &mut MeshData, c: Vec3, arm: Vec3, lateral: Vec3, half_w: f32, edge_dist: f32, y: f32) {
    let s0 = edge_dist + 0.35;
    let inner = (half_w - 0.5).max(0.8);
    push_flat_quad(md, c, arm, lateral, s0, s0 + 0.45, -inner, inner, y);

    let span = (half_w - 0.7) * 2.0;
    if span <= 0.5 {
        return;
    }
    let n = ((span / 1.15).floor() as i32).max(2);
    let step = span / (n as f32 * 2.0 - 1.0);
    let z0 = edge_dist + 1.4;
    for k in 0..n {
        let l0 = -span * 0.5 + 2.0 * k as f32 * step;
        push_flat_quad(md, c, arm, lateral, z0, z0 + 2.4, l0, l0 + step, y);
    }
}

fn push_junction_markings(jn: &Junction, md: &mut MeshData) {
    let c = Vec3::new(jn.info.x, jn.info.y, jn.info.z);
    let y = c.y + MARK_LIFT;
    // Main-road arms: pavement edge sits at cross_half + margin along main.
    for arm in &jn.main_arms {
        push_arm_markings(
            md,
            c,
            *arm,
            jn.cross_tan,
            jn.main_half,
            jn.cross_half + JUNCTION_MARGIN,
            y,
        );
    }
    // Cross-road arms: pavement edge sits at main_half + margin along cross.
    for arm in &jn.cross_arms {
        push_arm_markings(
            md,
            c,
            *arm,
            jn.main_tan,
            jn.cross_half,
            jn.main_half + JUNCTION_MARGIN,
            y,
        );
    }
}

// ---------------------------------------------------------------------------
// Generation — RoadSegment pipeline per consecutive point pair
// ---------------------------------------------------------------------------

/// Trim distance (meters) a junction carves out of each meeting road.
fn junction_trim(jn: &Junction, main_side: bool, p: &RoadPointData) -> f32 {
    let half = if main_side {
        jn.cross_half
    } else {
        jn.main_half
    };
    half + JUNCTION_MARGIN + JUNCTION_SLACK + p.edge_edge_clipping_offset.max(0.0)
}

#[allow(clippy::too_many_arguments)]
fn generate_one_road(
    pts: &[RoadPointData],
    manager: &ManagerData,
    density: f32,
    manager_thick: f32,
    trims: &HashMap<usize, f32>,
    resp: &mut GenResponse,
) {
    let n = pts.len();
    let fwd = forwards(pts);
    let sections: Vec<Vec<ColSpec>> = pts.iter().map(build_section).collect();

    for seg in 0..n - 1 {
        let pa = &pts[seg];
        let pb = &pts[seg + 1];
        let p0 = Vec3::new(pa.x, pa.y, pa.z);
        let p1 = Vec3::new(pb.x, pb.y, pb.z);
        if p0.dist(p1) < 0.01 {
            continue;
        }
        let bez = Bezier {
            p0,
            c1: p0.add(fwd[seg].scale(pa.next_mag.max(0.0))),
            c2: p1.sub(fwd[seg + 1].scale(pb.prior_mag.max(0.0))),
            p1,
        };
        let seg_len = bez.approx_length();
        if seg_len < 0.05 {
            continue;
        }

        // Junction trims at either end of this segment (u ≈ meters / length).
        let t_start = trims.get(&seg).copied().unwrap_or(0.0) / seg_len;
        let t_end = trims.get(&(seg + 1)).copied().unwrap_or(0.0) / seg_len;
        if t_start + t_end >= 0.98 {
            continue; // segment fully consumed by a junction
        }

        resp.length += seg_len;

        // loops = ⌊length / density⌋ (÷LOWPOLY_FACTOR when low-poly).
        let mut loops = ((seg_len / density).floor().max(1.0)) as usize;
        if manager.low_poly {
            loops = ((seg_len / density / LOWPOLY_FACTOR).floor().max(1.0)) as usize;
        }
        resp.loop_count += loops as u32;

        // UV flow (godot _build_geo): snapped tiles over segment length.
        let min_lane_width = pa.lane_width.min(pb.lane_width).max(0.5);
        let single_uv_height = min_lane_width * 4.0;
        let tiles = snap_uv_tiles(seg_len / single_uv_height);
        let per_loop_uv = tiles / loops as f32;

        let unified = unify_sections(&sections[seg], &sections[seg + 1]);
        let ncols = unified.len();
        if ncols == 0 {
            continue;
        }

        // Effective underside thickness (point override > manager; -1 = off).
        let thick_s = if pa.underside_thickness >= 0.0 {
            pa.underside_thickness
        } else {
            manager_thick
        };
        let thick_e = if pb.underside_thickness >= 0.0 {
            pb.underside_thickness
        } else {
            manager_thick
        };
        let has_underside = thick_s > 0.0 && thick_e > 0.0;
        let thickness = lerp(thick_s.max(0.0), thick_e.max(0.0), 0.5).max(0.05);

        let align = pa.alignment;
        let mut prev_top: Option<Vec<u32>> = None;
        let mut prev_bot: Option<Vec<u32>> = None;
        let mut uv_y = 0.0f32;
        let mut edge_c: Vec<f32> = Vec::new();
        let mut edge_f: Vec<f32> = Vec::new();
        let mut edge_r: Vec<f32> = Vec::new();
        let mut lane_paths: Vec<(usize, Vec<Vec3>)> =
            (0..ncols).map(|i| (i, Vec::new())).collect();

        for loop_i in 0..=loops {
            let u = loop_i as f32 / loops as f32;
            if u < t_start || u > 1.0 - t_end {
                // Inside a junction keep-out zone: break the strip so nothing
                // stretches across the pavement n-gon.
                prev_top = None;
                prev_bot = None;
                continue;
            }
            let pos = bez.eval(u);
            let tan = bez.deriv(u).normalized();
            let left = UP.cross(tan).normalized(); // lateral basis (documented delta)
            if loop_i > 0 {
                uv_y += per_loop_uv;
            }

            let widths: Vec<f32> = unified.iter().map(|c| lerp(c.w_s, c.w_e, u)).collect();
            let drops: Vec<f32> = unified.iter().map(|c| lerp(c.y_s, c.y_e, u)).collect();

            // Origin per Alignment enum (start point governs the segment).
            let total: f32 = widths.iter().sum();
            let origin = match align {
                AL_DIVIDER => {
                    let mut acc = 0.0;
                    let mut done = false;
                    for (i, c) in unified.iter().enumerate() {
                        if c.is_lane_fwd() && !done {
                            done = true;
                            break;
                        }
                        acc += widths[i];
                    }
                    if done {
                        -acc
                    } else {
                        -total * 0.5
                    }
                }
                _ => -total * 0.5, // GEOMETRIC
            };

            let mut bounds: Vec<f32> = Vec::with_capacity(ncols + 1);
            bounds.push(origin);
            for w in &widths {
                let last = *bounds.last().unwrap();
                bounds.push(last + w);
            }

            // Edge curves (RoadSegment edge_F/edge_R/edge_C parity):
            let idx_div = unified.iter().position(|c| c.is_lane_fwd()).unwrap_or(ncols);
            let idx_r_outer = unified
                .iter()
                .position(|c| c.is_lane_rev())
                .unwrap_or(idx_div);
            let idx_f_outer = unified
                .iter()
                .rposition(|c| c.is_lane_fwd())
                .unwrap_or(idx_div);
            let bnd = |bi: usize| {
                let clamped = bi.min(ncols);
                pos.add(left.scale(bounds[clamped])).add(UP.scale(0.06))
            };
            edge_c.extend_from_slice(&{
                let v = bnd(idx_div);
                [v.x, v.y, v.z]
            });
            edge_f.extend_from_slice(&{
                let v = bnd(idx_f_outer + 1);
                [v.x, v.y, v.z]
            });
            edge_r.extend_from_slice(&{
                let v = bnd(idx_r_outer);
                [v.x, v.y, v.z]
            });

            // Top surface: 2 verts per column (per-lane trimsheet UVs).
            let mut row: Vec<u32> = Vec::with_capacity(ncols * 2);
            for (ci, c) in unified.iter().enumerate() {
                let bl = bounds[ci];
                let br = bounds[ci + 1];
                let pl = pos.add(left.scale(bl)).add(UP.scale(drops[ci]));
                let pr = pos.add(left.scale(br)).add(UP.scale(drops[ci]));
                // Trimsheet column slides smoothly from start to end kind
                // (identical configs produce constant UVs — exact parity).
                let uvx_l = lerp(
                    uv_col_x(c.kind_s, 0.0, c.mirror_s),
                    uv_col_x(c.kind_e, 0.0, c.mirror_e),
                    u,
                );
                let uvx_r = lerp(
                    uv_col_x(c.kind_s, 1.0, c.mirror_s),
                    uv_col_x(c.kind_e, 1.0, c.mirror_e),
                    u,
                );
                row.push(resp.road.push_vertex(pl, UP, uvx_l, uv_y));
                row.push(resp.road.push_vertex(pr, UP, uvx_r, uv_y));
                if c.is_lane {
                    lane_paths[ci]
                        .1
                        .push(pos.add(left.scale((bl + br) * 0.5)).add(UP.scale(0.05)));
                }
            }
            if let Some(prv) = prev_top {
                for ci in 0..ncols {
                    resp.road.push_quad(
                        prv[ci * 2],
                        prv[ci * 2 + 1],
                        row[ci * 2],
                        row[ci * 2 + 1],
                    );
                }
            }
            prev_top = Some(row);

            // Underside: side walls + flat bottom (underside_thickness parity).
            if has_underside {
                let ol = bounds[0];
                let orr = bounds[ncols];
                let tl = pos.add(left.scale(ol));
                let tr = pos.add(left.scale(orr));
                let bl = tl.sub(UP.scale(thickness));
                let br = tr.sub(UP.scale(thickness));
                let dark = |f: f32| uv_col_x(LT_NO_MARKING, f, false);
                let wl = resp.road.push_vertex(tl, left, dark(0.0), uv_y);
                let wr = resp.road.push_vertex(tr, left, dark(1.0), uv_y);
                let hl = resp.road.push_vertex(bl, left, dark(0.0), uv_y);
                let hr = resp.road.push_vertex(br, left, dark(1.0), uv_y);
                if let Some(pv) = prev_bot {
                    resp.road.push_quad(pv[0], pv[1], wl, wr); // top edge strip
                    resp.road.push_quad(pv[2], pv[3], hl, hr); // bottom
                    resp.road.push_quad(pv[0], pv[2], wl, hl); // left wall
                    resp.road.push_quad(pv[1], pv[3], wr, hr); // right wall
                }
                prev_bot = Some(vec![wl, wr, hl, hr]);
            }
        }

        // Emit lane paths (RoadLane parity).
        for (ci, cols_pts) in lane_paths.iter() {
            if cols_pts.len() < 2 {
                continue;
            }
            let c = &unified[*ci];
            if !c.is_lane {
                continue;
            }
            let flat: Vec<f32> = cols_pts.iter().flat_map(|v| [v.x, v.y, v.z]).collect();
            resp.lanes.push(LanePath {
                path: flat,
                dir: c.dir,
                lane_type: if c.w_e > 0.02 { c.kind_e } else { c.kind_s },
            });
        }

        if !edge_c.is_empty() {
            resp.edge_c.push(edge_c);
        }
        if !edge_f.is_empty() {
            resp.edge_f.push(edge_f);
        }
        if !edge_r.is_empty() {
            resp.edge_r.push(edge_r);
        }

        // Centerline overlay for this segment, trimmed to the same zone.
        let mut cl: Vec<f32> = Vec::new();
        for s in 0..=24 {
            let t = lerp(t_start, 1.0 - t_end, s as f32 / 24.0);
            let p = bez.eval(t);
            cl.extend_from_slice(&[p.x, p.y + 0.09, p.z]);
        }
        resp.centerline.push(cl);
    }
}

/// RoadSegment._build_geo UV flow snapping (verbatim rules).
fn snap_uv_tiles(target: f32) -> f32 {
    if target < 0.3 {
        target // free slide, no snapping
    } else if target < 0.75 {
        0.5
    } else if target < 1.25 {
        1.0
    } else if target < 1.75 {
        1.5
    } else {
        target.round()
    }
}

fn uv_col_x(kind: u8, frac: f32, mirror: bool) -> f32 {
    let col = UV_MUL[(kind as usize).min(9)] as f32;
    let f = if mirror { 1.0 - frac } else { frac };
    (col + f.clamp(0.0, 1.0)) * 0.125
}

pub fn generate_road(req: &GenRequest) -> GenResponse {
    let mut resp = GenResponse::default();

    let roads: Vec<&[RoadPointData]> = if !req.roads.is_empty() {
        req.roads.iter().map(|r| r.as_slice()).collect()
    } else {
        vec![&req.points[..]]
    };
    if roads.is_empty() || roads[0].len() < 2 {
        resp.error = Some("At least 2 RoadPoints are required.".into());
        return resp;
    }
    let density = req.manager.density.clamp(0.5, 64.0);
    let manager_thick = req.manager.underside_thickness;

    let fwds: Vec<Vec<Vec3>> = roads.iter().map(|r| forwards(r)).collect();
    let junctions = detect_junctions(&roads, &fwds);

    // Per-road keep-out zones: point index → trim meters. Honors
    // generate_mesh_at_intersections on the shared RoadPoint.
    let mut trims: Vec<HashMap<usize, f32>> = vec![HashMap::new(); roads.len()];
    for jn in &junctions {
        let mp = &roads[0][jn.info.main_idx];
        let cp = &roads[jn.info.cross_road][jn.info.cross_idx];
        if mp.generate_mesh_at_intersections {
            let m = junction_trim(jn, true, mp);
            let e = trims[0].entry(jn.info.main_idx).or_insert(0.0f32);
            *e = e.max(m);
        }
        if cp.generate_mesh_at_intersections {
            let c = junction_trim(jn, false, cp);
            let e = trims[jn.info.cross_road]
                .entry(jn.info.cross_idx)
                .or_insert(0.0f32);
            *e = e.max(c);
        }
    }

    for (ri, road) in roads.iter().enumerate() {
        if road.len() < 2 {
            continue;
        }
        generate_one_road(road, &req.manager, density, manager_thick, &trims[ri], &mut resp);
    }

    // Junction pavement + markings (RoadIntersection parity).
    for jn in &junctions {
        push_junction_pavement(jn, &mut resp.junctions);
        push_junction_markings(jn, &mut resp.junction_marks);
    }
    resp.junction_count = junctions.len() as u32;
    resp.junction_info = junctions.iter().map(|j| j.info.clone()).collect();

    resp.vertex_count = (resp.road.positions.len() / 3
        + resp.junctions.positions.len() / 3
        + resp.junction_marks.positions.len() / 3) as u32;
    resp.triangle_count = (resp.road.indices.len() / 3
        + resp.junctions.indices.len() / 3
        + resp.junction_marks.indices.len() / 3) as u32;
    resp.elapsed_us = 0;
    resp
}
