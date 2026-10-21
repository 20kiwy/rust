//! Minimal column-major 4x4 math for the viewer (no external deps).

#[derive(Clone, Copy, Debug)]
pub struct Mat4(pub [f32; 16]);

pub fn perspective(fovy_rad: f32, aspect: f32, near: f32, far: f32) -> Mat4 {
    let f = 1.0 / (fovy_rad / 2.0).tan();
    let nf = 1.0 / (near - far);
    Mat4([
        f / aspect, 0.0, 0.0, 0.0, //
        0.0, f, 0.0, 0.0, //
        0.0, 0.0, (far + near) * nf, -1.0, //
        0.0, 0.0, 2.0 * far * near * nf, 0.0,
    ])
}

pub fn look_at(eye: [f32; 3], center: [f32; 3], up: [f32; 3]) -> Mat4 {
    let norm = |v: [f32; 3]| -> [f32; 3] {
        let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-9);
        [v[0] / l, v[1] / l, v[2] / l]
    };
    let cross = |a: [f32; 3], b: [f32; 3]| -> [f32; 3] {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let sub = |a: [f32; 3], b: [f32; 3]| -> [f32; 3] {
        [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    };
    let dot = |a: [f32; 3], b: [f32; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    let z = norm(sub(eye, center)); // backward
    let x = norm(cross(up, z));
    let y = cross(z, x);

    Mat4([
        x[0], y[0], z[0], 0.0, //
        x[1], y[1], z[1], 0.0, //
        x[2], y[2], z[2], 0.0, //
        -dot(x, eye), -dot(y, eye), -dot(z, eye), 1.0,
    ])
}

/// a * b (both column-major).
pub fn mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut o = [0.0f32; 16];
    for c in 0..4 {
        for r in 0..4 {
            let mut s = 0.0;
            for k in 0..4 {
                s += a.0[k * 4 + r] * b.0[c * 4 + k];
            }
            o[c * 4 + r] = s;
        }
    }
    Mat4(o)
}

/// Clip-space transform of a world point: returns (ndc_x, ndc_y, w).
pub fn project(m: &Mat4, p: [f32; 3]) -> (f32, f32, f32) {
    let x = m.0[0] * p[0] + m.0[4] * p[1] + m.0[8] * p[2] + m.0[12];
    let y = m.0[1] * p[0] + m.0[5] * p[1] + m.0[9] * p[2] + m.0[13];
    let _z = m.0[2] * p[0] + m.0[6] * p[1] + m.0[10] * p[2] + m.0[14];
    let w = m.0[3] * p[0] + m.0[7] * p[1] + m.0[11] * p[2] + m.0[15];
    if w.abs() < 1e-6 {
        return (0.0, 0.0, 0.0);
    }
    (x / w, y / w, w)
}

/// Orbit camera.
#[derive(Clone, Copy)]
pub struct Orbit {
    pub yaw: f32,
    pub pitch: f32,
    pub dist: f32,
    pub target: [f32; 3],
}

impl Default for Orbit {
    fn default() -> Self {
        Orbit {
            yaw: 0.62,
            pitch: 0.72,
            dist: 150.0,
            target: [0.0, 0.0, 6.0],
        }
    }
}

impl Orbit {
    pub fn eye(&self) -> [f32; 3] {
        let cp = self.pitch.cos();
        [
            self.target[0] + self.dist * cp * self.yaw.sin(),
            self.target[1] + self.dist * self.pitch.sin(),
            self.target[2] + self.dist * cp * self.yaw.cos(),
        ]
    }

    pub fn view_proj(&self, aspect: f32) -> (Mat4, [f32; 3]) {
        let eye = self.eye();
        let view = look_at(eye, self.target, [0.0, 1.0, 0.0]);
        let proj = perspective(52.0f32.to_radians(), aspect, 0.5, 1600.0);
        (mul(&proj, &view), eye)
    }

    /// World-space ray through a pixel (normalized device coords in [-1,1]).
    pub fn ray_through(&self, ndc_x: f32, ndc_y: f32, aspect: f32) -> ([f32; 3], [f32; 3]) {
        let eye = self.eye();
        let fwd = {
            let f = [
                self.target[0] - eye[0],
                self.target[1] - eye[1],
                self.target[2] - eye[2],
            ];
            let l = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt().max(1e-9);
            [f[0] / l, f[1] / l, f[2] / l]
        };
        let cross = |a: [f32; 3], b: [f32; 3]| -> [f32; 3] {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let norm = |v: [f32; 3]| -> [f32; 3] {
            let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt().max(1e-9);
            [v[0] / l, v[1] / l, v[2] / l]
        };
        let up0 = [0.0f32, 1.0, 0.0];
        let right = norm(cross(fwd, up0));
        let up2 = cross(right, fwd);
        let t = (52.0f32.to_radians() / 2.0).tan();
        let dx = ndc_x * t * aspect;
        let dy = ndc_y * t;
        let dir = norm([
            fwd[0] + right[0] * dx + up2[0] * dy,
            fwd[1] + right[1] * dx + up2[1] * dy,
            fwd[2] + right[2] * dx + up2[2] * dy,
        ]);
        (eye, dir)
    }
}
