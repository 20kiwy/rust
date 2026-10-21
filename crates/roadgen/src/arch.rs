//! roadgen — Proof-of-concept Rust port of MicroGSD/RoadArchitect's core
//! road-generation math, compiled to `wasm32-unknown-unknown`.
//!
//! Ported faithfully from:
//!   * `GSDSplineC.cs`      — Hermite (Catmull-Rom, tension 0.5) spline with
//!                             per-node EaseIO and tangent magnitude clamping.
//!   * `GSDRootUtil.cs`     — the `Ease()` function (k1/k2 ease in/out).
//!   * `GSDRoadUtility*.cs` — ribbon-mesh extrusion along the spline with
//!                             shoulders, lane markings and UV flow.
//!
//! The engine-facing 60% of RoadArchitect (Terrain ops, MeshFilter wiring,
//! Gizmos, EditorWindows) is replaced here by a tiny JSON-in/JSON-out ABI so
//! any renderer (Three.js here, Bevy natively) can consume the geometry.

#![allow(clippy::needless_range_loop)]

use serde::{Deserialize, Serialize};

const PI: f32 = std::f32::consts::PI;

// ---------------------------------------------------------------------------
// Math helpers (ports of Unity's Vector3 + GSDRootUtil.Ease)
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
    fn mag(self) -> f32 {
        self.dot(self).sqrt()
    }
    fn dist(self, o: Vec3) -> f32 {
        self.sub(o).mag()
    }
    fn normalized(self) -> Vec3 {
        let m = self.mag();
        if m < 1e-8 {
            Vec3::new(0.0, 1.0, 0.0)
        } else {
            self.scale(1.0 / m)
        }
    }
    fn clamp_magnitude(self, max: f32) -> Vec3 {
        let m = self.mag();
        if m > max && m > 1e-8 {
            self.scale(max / m)
        } else {
            self
        }
    }
    fn lerp(self, o: Vec3, t: f32) -> Vec3 {
        self.add(o.sub(self).scale(t.clamp(0.0, 1.0)))
    }
}

#[allow(dead_code)]
fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Direct port of `GSDRootUtil.Ease(t, k1, k2)`.
#[inline]
fn ease(t: f32, k1: f32, k2: f32) -> f32 {
    let k1 = k1.clamp(0.0, 1.0);
    let k2 = k2.clamp(0.0, 1.0);
    let f = k1 * 2.0 / PI + k2 - k1 + (1.0 - k2) * 2.0 / PI;
    if f <= 1e-6 {
        return t;
    }
    let s = if t < k1 {
        k1 * (2.0 / PI) * (((t / k1) * (PI / 2.0) - PI / 2.0).sin() + 1.0)
    } else if t < k2 {
        2.0 * k1 / PI + t - k1
    } else {
        let denom = (1.0 - k2).max(1e-6);
        2.0 * k1 / PI + k2 - k1 + ((1.0 - k2) * (2.0 / PI)) * (((t - k2) / denom) * (PI / 2.0)).sin()
    };
    s / f
}

// ---------------------------------------------------------------------------
// Spline — port of GSDSplineC.GetSplineValue / GetHermiteInternal
// ---------------------------------------------------------------------------

pub struct Spline {
    nodes: Vec<Node>,
    t_time: Vec<f32>,
    length: f32,
    threshold: f32, // GSDRoad.opt_MagnitudeThreshold (default 300)
}

#[derive(Clone, Copy, Deserialize)]
pub struct Node {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    #[serde(default = "d_half")]
    pub ease_in: f32, // EaseIO.x
    #[serde(default = "d_half")]
    pub ease_out: f32, // EaseIO.y
}

fn d_half() -> f32 {
    0.5
}

impl Spline {
    /// Mirrors RoadArchitect: node `tTime` = cumulative polyline distance
    /// normalized to 0..1.
    pub fn new(nodes: Vec<Node>, threshold: f32) -> Self {
        let mut t_time = vec![0.0f32; nodes.len()];
        let mut total = 0.0f32;
        for i in 1..nodes.len() {
            total += nodes[i - 1].pos().dist(nodes[i].pos());
            t_time[i] = total;
        }
        if total > 1e-6 {
            for t in t_time.iter_mut() {
                *t /= total;
            }
        }
        Spline {
            nodes,
            t_time,
            length: total,
            threshold,
        }
    }

    fn node_count(&self) -> usize {
        self.nodes.len()
    }

    /// Port of `GSDSplineC.NGI(i,o) = Mathf.Clamp(i+o, 0, count-1)`.
    fn ngi(&self, i: i32) -> usize {
        i.clamp(0, self.node_count() as i32 - 1) as usize
    }

    /// Exact port of `GSDSplineC.GetHermiteInternal(i, t, bTangent)`:
    ///
    /// ```text
    /// CM  = [ 2,-3,0,1 | -2,3,0,0 | 1,-2,1,0 | 1,-1,0,0 ]  (Hermite basis)
    /// NI  = [0, 1, -1, 2]   → P0=nodes[i], P1=nodes[i+1], P2=nodes[i-1], P3=nodes[i+2]
    /// tVect = BL0·P0 + BL1·P1 + BL2·m1 + BL3·m2
    /// m1 = (P1-P2)·0.5 , m2 = (P3-P0)·0.5 , tension = 0.5 (Catmull-Rom)
    /// ```
    fn hermite_internal(&self, i: usize, t: f64, b_tangent: bool) -> Vec3 {
        let t = t as f32;
        let t2 = t * t;
        let t3 = t2 * t;

        // Vectors exactly as the C# names them:
        let p0 = self.nodes[self.ngi(i as i32)].pos(); // NI[0] = 0
        let p1 = self.nodes[self.ngi(i as i32 + 1)].pos(); // NI[1] = 1
        let p2 = self.nodes[self.ngi(i as i32 - 1)].pos(); // NI[2] = -1
        let p3 = self.nodes[self.ngi(i as i32 + 2)].pos(); // NI[3] = 2

        // Tension hardcoded to 0.5 (Catmull-Rom) in the original:
        let tension = 0.5f32;
        let mut x1 = p1.sub(p2).scale(tension); // m1
        let mut x2 = p3.sub(p0).scale(tension); // m2

        // Tangent magnitude-threshold clamping (opt_MagnitudeThreshold = 300):
        if p1.dist(p3) > self.threshold {
            x1 = x1.clamp_magnitude(self.threshold);
            x2 = x2.clamp_magnitude(self.threshold);
        } else if p0.dist(p2) > self.threshold {
            x1 = x1.clamp_magnitude(self.threshold);
            x2 = x2.clamp_magnitude(self.threshold);
        }

        let (b0, b1, b2, b3) = if !b_tangent {
            (
                2.0 * t3 - 3.0 * t2 + 1.0, // CM[0..3]
                -2.0 * t3 + 3.0 * t2,      // CM[4..7]
                t3 - 2.0 * t2 + t,         // CM[8..11]
                t3 - t2,                   // CM[12..15]
            )
        } else {
            // Derivative branch (C#: t*=2, t2=3t², t3=0 trick → true derivative):
            (
                6.0 * t2 - 6.0 * t,
                -6.0 * t2 + 6.0 * t,
                3.0 * t2 - 4.0 * t + 1.0,
                3.0 * t2 - 2.0 * t,
            )
        };

        let out = p0
            .scale(b0)
            .add(p1.scale(b1))
            .add(x1.scale(b2))
            .add(x2.scale(b3));

        // Original clamps the road above y=0 (terrain floor):
        if !b_tangent && out.y < 0.0 {
            return Vec3::new(out.x, 0.0, out.z);
        }
        out
    }

    fn segment_for(&self, f: f32) -> (usize, f32) {
        let n = self.node_count();
        if n <= 1 {
            return (0, 0.0);
        }
        let mut idx = n - 2;
        for i in 0..n {
            if i == n - 1 {
                idx = i - 1;
                break;
            }
            if self.t_time[i] > f {
                idx = i.saturating_sub(1);
                break;
            }
        }
        let seg = self.t_time[idx + 1] - self.t_time[idx];
        let raw = if seg.abs() < 1e-8 {
            0.0
        } else {
            (f - self.t_time[idx]) / seg
        };
        (idx, raw.clamp(0.0, 1.0))
    }

    /// Port of `GetSplineValue(f, false)`.
    pub fn value(&self, f: f32) -> Vec3 {
        if self.node_count() == 0 {
            return Vec3::default();
        }
        if self.node_count() == 1 {
            return self.nodes[0].pos();
        }
        let (idx, raw) = self.segment_for(f);
        let n = self.nodes[idx];
        let param = ease(raw, n.ease_in, n.ease_out) as f64;
        self.hermite_internal(idx, param, false)
    }

    /// Port of `GetSplineValue(f, true)` (tangent).
    pub fn tangent(&self, f: f32) -> Vec3 {
        if self.node_count() <= 1 {
            return Vec3::new(0.0, 0.0, 1.0);
        }
        let (idx, raw) = self.segment_for(f);
        let n = self.nodes[idx];
        let param = ease(raw, n.ease_in, n.ease_out) as f64;
        self.hermite_internal(idx, param, true)
    }

    pub fn length(&self) -> f32 {
        self.length
    }
}

impl Node {
    fn pos(self) -> Vec3 {
        Vec3::new(self.x, self.y, self.z)
    }
}

// ---------------------------------------------------------------------------
// Mesh generation — simplified port of GSDRoadUtility ribbon extrusion
// ---------------------------------------------------------------------------

#[derive(Serialize, Default)]
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
        // CCW seen from above.
        self.indices.extend_from_slice(&[a, c, b, b, c, d]);
    }
}

#[derive(Deserialize)]
pub struct GenRequest {
    pub nodes: Vec<Node>,
    #[serde(default = "d_road_width")]
    pub road_width: f32, // GSDRoad.opt_RoadWidth
    #[serde(default = "d_shoulder")]
    pub shoulder_width: f32,
    #[serde(default = "d_subdiv")]
    pub samples_per_meter: f32,
    #[serde(default = "d_threshold")]
    pub magnitude_threshold: f32,
    #[serde(default = "d_true")]
    pub lane_markings: bool,
    #[serde(default = "d_true")]
    pub shoulders: bool,
}

fn d_road_width() -> f32 {
    8.0
}
fn d_shoulder() -> f32 {
    1.5
}
fn d_subdiv() -> f32 {
    2.0
}
fn d_threshold() -> f32 {
    300.0
}
fn d_true() -> bool {
    true
}

#[derive(Serialize, Default)]
pub struct GenResponse {
    pub road: MeshData,
    pub markings: MeshData,
    /// Polyline of the spline center (x,y,z * n) for editor overlay.
    pub centerline: Vec<f32>,
    /// Node positions as generated (for handle sync).
    pub nodes_out: Vec<f32>,
    pub length: f32,
    pub vertex_count: u32,
    pub triangle_count: u32,
    pub elapsed_us: u32,
    pub error: Option<String>,
}

const UP: Vec3 = Vec3 { x: 0.0, y: 1.0, z: 0.0 };

/// Emits a ribbon strip centered at `offset` (meters, positive = spline-left)
/// with the given width, lifted `lift` meters above the road surface.
/// If `dash` is Some((dash_len, gap_len)), emits dashed segments instead.
fn emit_strip(
    out: &mut MeshData,
    samples: &[(Vec3, Vec3)], // (position, left-unit)
    v_flow: &[f32],           // uv.y per sample (distance along)
    offset: f32,
    width: f32,
    lift: f32,
    dash: Option<(f32, f32)>,
) {
    let half = width * 0.5;
    let n = samples.len();
    if n < 2 {
        return;
    }

    match dash {
        None => {
            let mut prev: Option<(u32, u32)> = None;
            for i in 0..n {
                let (p, left) = &samples[i];
                let c = p.add(left.scale(offset)).add(UP.scale(lift));
                let a = c.sub(left.scale(half));
                let b = c.add(left.scale(half));
                let ia = out.push_vertex(a, UP, 0.0, v_flow[i]);
                let ib = out.push_vertex(b, UP, 1.0, v_flow[i]);
                if let Some((pa, pb)) = prev {
                    out.push_quad(pa, pb, ia, ib);
                }
                prev = Some((ia, ib));
            }
        }
        Some((dash_len, gap_len)) => {
            let period = dash_len + gap_len;
            let mut i = 0usize;
            let mut dist = 0.0f32;
            while i < n - 1 {
                // distance along this segment:
                let seg_len = samples[i].0.dist(samples[i + 1].0);
                let seg_start = dist;
                // dash window within [seg_start, seg_start+seg_len] intersect [k*period, k*period+dash_len]
                let k0 = (seg_start / period).floor() as i64;
                for k in k0..=k0 + 1 {
                    let ds = k as f32 * period;
                    let de = ds + dash_len;
                    let s = seg_start.max(ds);
                    let e = (seg_start + seg_len).min(de);
                    if e - s < 0.05 {
                        continue;
                    }
                    let ts = ((s - seg_start) / seg_len).clamp(0.0, 1.0);
                    let te = ((e - seg_start) / seg_len).clamp(0.0, 1.0);
                    let p0 = samples[i].0.lerp(samples[i + 1].0, ts);
                    let p1 = samples[i].0.lerp(samples[i + 1].0, te);
                    let l0 = samples[i].1.lerp(samples[i + 1].1, ts).normalized();
                    let l1 = samples[i].1.lerp(samples[i + 1].1, te).normalized();
                    let c0 = p0.add(l0.scale(offset)).add(UP.scale(lift));
                    let c1 = p1.add(l1.scale(offset)).add(UP.scale(lift));
                    let a0 = c0.sub(l0.scale(half));
                    let b0 = c0.add(l0.scale(half));
                    let a1 = c1.sub(l1.scale(half));
                    let b1 = c1.add(l1.scale(half));
                    let ia0 = out.push_vertex(a0, UP, 0.0, s);
                    let ib0 = out.push_vertex(b0, UP, 1.0, s);
                    let ia1 = out.push_vertex(a1, UP, 0.0, e);
                    let ib1 = out.push_vertex(b1, UP, 1.0, e);
                    out.push_quad(ia0, ib0, ia1, ib1);
                }
                dist += seg_len;
                i += 1;
            }
        }
    }
}

pub fn generate_road(req: &GenRequest) -> GenResponse {

    let mut resp = GenResponse {
        road: MeshData::default(),
        markings: MeshData::default(),
        centerline: Vec::new(),
        nodes_out: Vec::new(),
        length: 0.0,
        vertex_count: 0,
        triangle_count: 0,
        elapsed_us: 0,
        error: None,
    };

    if req.nodes.len() < 2 {
        resp.error = Some("At least 2 nodes are required.".into());
        return resp;
    }

    let spline = Spline::new(req.nodes.clone(), req.magnitude_threshold.max(1.0));
    let total_len = spline.length();
    if total_len < 0.5 {
        resp.error = Some("Nodes are too close together.".into());
        return resp;
    }
    resp.length = total_len;

    // Sample the spline (RoadArchitect subdivides per meter as well).
    let spm = req.samples_per_meter.clamp(0.1, 24.0);
    let steps = ((total_len * spm).ceil() as usize).clamp(8, 200_000);
    let half_w = req.road_width.max(1.0) * 0.5;
    let sh_w = if req.shoulders { req.shoulder_width.max(0.0) } else { 0.0 };

    let mut samples: Vec<(Vec3, Vec3)> = Vec::with_capacity(steps + 1);
    let mut v_flow: Vec<f32> = Vec::with_capacity(steps + 1);
    let mut dist_acc = 0.0f32;
    let mut prev_p: Option<Vec3> = None;

    for s in 0..=steps {
        let f = s as f32 / steps as f32;
        let p = spline.value(f);
        let t = spline.tangent(f).normalized();
        // left unit = cross(up, tangent) — mirrors RoadArchitect's lateral basis
        let left = UP.cross(t).normalized();
        if let Some(pp) = prev_p {
            dist_acc += p.dist(pp);
        }
        prev_p = Some(p);
        samples.push((p, left));
        v_flow.push(dist_acc / 4.0); // texture repeat every 4 m
    }

    // ---- Road surface: shoulder-left | road | shoulder-right (4 columns) ----
    let road = &mut resp.road;
    let mut row_idx: [u32; 4] = [0; 4];
    let mut prev_row: Option<[u32; 4]> = None;
    let n_cols = 4usize;
    // lateral offsets & normals per column
    let cols: [(f32, f32, f32); 4] = [
        // (offset, y-drop, uv.x)
        (-(half_w + sh_w), -0.12, 0.0),
        (-half_w, 0.0, sh_w / (half_w * 2.0 + sh_w * 2.0)),
        (half_w, 0.0, (sh_w + half_w * 2.0) / (half_w * 2.0 + sh_w * 2.0)),
        (half_w + sh_w, -0.12, 1.0),
    ];
    for (i, (p, left)) in samples.iter().enumerate() {
        for (c, (off, drop, u)) in cols.iter().enumerate() {
            let v = p.add(left.scale(*off)).add(UP.scale(*drop));
            let nrm = if *drop < 0.0 {
                Vec3::new(0.0, 1.0, 0.0)
                    .add(left.scale(off.signum() * 0.35))
                    .normalized()
            } else {
                UP
            };
            row_idx[c] = road.push_vertex(v, nrm, *u, v_flow[i]);
        }
        if let Some(pr) = prev_row {
            for c in 0..n_cols - 1 {
                road.push_quad(pr[c], pr[c + 1], row_idx[c], row_idx[c + 1]);
            }
        }
        prev_row = Some(row_idx);
    }

    // ---- Markings ----
    if req.lane_markings {
        let m = &mut resp.markings;
        // Solid edge lines just inside the asphalt:
        emit_strip(m, &samples, &v_flow, -(half_w - 0.45), 0.14, 0.02, None);
        emit_strip(m, &samples, &v_flow, half_w - 0.45, 0.14, 0.02, None);
        // Dashed center line (3 m dash / 6 m gap like typical highways):
        emit_strip(m, &samples, &v_flow, 0.0, 0.16, 0.02, Some((3.0, 6.0)));
    }

    // ---- Centerline overlay + node echo ----
    let step = (samples.len() / 120).clamp(1, usize::MAX);
    for i in (0..samples.len()).step_by(step) {
        let (p, _) = samples[i];
        resp.centerline.extend_from_slice(&[p.x, p.y + 0.08, p.z]);
    }
    let (last_p, _) = samples[samples.len() - 1];
    resp.centerline
        .extend_from_slice(&[last_p.x, last_p.y + 0.08, last_p.z]);
    for nd in &spline.nodes {
        resp.nodes_out.extend_from_slice(&[nd.x, nd.y, nd.z]);
    }

    resp.vertex_count = (resp.road.positions.len() / 3) as u32 + (resp.markings.positions.len() / 3) as u32;
    resp.triangle_count = (resp.road.indices.len() / 3) as u32 + (resp.markings.indices.len() / 3) as u32;
    resp.elapsed_us = 0;
    resp
}

