//! Thin WebGL2 wrappers: programs, interleaved surface meshes, line meshes.

use wasm_bindgen::JsCast;
use web_sys::*;

pub type Gl = WebGl2RenderingContext;

pub struct Program {
    pub prog: WebGlProgram,
    pub u_vp: Option<WebGlUniformLocation>,
    pub u_kind: Option<WebGlUniformLocation>,
    pub u_color: Option<WebGlUniformLocation>,
    pub u_eye: Option<WebGlUniformLocation>,
    pub u_light: Option<WebGlUniformLocation>,
    pub u_offset: Option<WebGlUniformLocation>,
}

fn compile(gl: &Gl, ty: u32, src: &str) -> Result<WebGlShader, String> {
    let s = gl
        .create_shader(ty)
        .ok_or_else(|| "create_shader failed".to_string())?;
    gl.shader_source(&s, src);
    gl.compile_shader(&s);
    if gl.get_shader_parameter(&s, Gl::COMPILE_STATUS).as_bool().unwrap_or(false) {
        Ok(s)
    } else {
        let log = gl
            .get_shader_info_log(&s)
            .unwrap_or_else(|| "no log".into());
        Err(format!("shader compile error: {log}"))
    }
}

impl Program {
    pub fn new(gl: &Gl, vs_src: &str, fs_src: &str) -> Result<Program, String> {
        let vs = compile(gl, Gl::VERTEX_SHADER, vs_src)?;
        let fs = compile(gl, Gl::FRAGMENT_SHADER, fs_src)?;
        let prog = gl
            .create_program()
            .ok_or_else(|| "create_program failed".to_string())?;
        gl.attach_shader(&prog, &vs);
        gl.attach_shader(&prog, &fs);
        gl.link_program(&prog);
        if !gl
            .get_program_parameter(&prog, Gl::LINK_STATUS)
            .as_bool()
            .unwrap_or(false)
        {
            let log = gl
                .get_program_info_log(&prog)
                .unwrap_or_else(|| "no log".into());
            return Err(format!("program link error: {log}"));
        }
        Ok(Program {
            u_vp: gl.get_uniform_location(&prog, "u_vp"),
            u_kind: gl.get_uniform_location(&prog, "u_kind"),
            u_color: gl.get_uniform_location(&prog, "u_color"),
            u_eye: gl.get_uniform_location(&prog, "u_eye"),
            u_light: gl.get_uniform_location(&prog, "u_light"),
            u_offset: gl.get_uniform_location(&prog, "u_offset"),
            prog,
        })
    }

    pub fn use_prog(&self, gl: &Gl) {
        gl.use_program(Some(&self.prog));
    }
}

/// Interleaved surface mesh: [px,py,pz, nx,ny,nz, u,v] * n + u32 indices.
pub struct SurfaceMesh {
    vao: Option<WebGlVertexArrayObject>,
    vbo: Option<WebGlBuffer>,
    ibo: Option<WebGlBuffer>,
    pub count: i32,
}

impl SurfaceMesh {
    pub fn new() -> Self {
        SurfaceMesh {
            vao: None,
            vbo: None,
            ibo: None,
            count: 0,
        }
    }

    pub fn upload(&mut self, gl: &Gl, inter: &[f32], indices: &[u32]) {
        if self.vao.is_none() {
            self.vao = gl.create_vertex_array();
            self.vbo = gl.create_buffer();
            self.ibo = gl.create_buffer();
        }
        let vao = match &self.vao {
            Some(v) => v,
            None => return,
        };
        gl.bind_vertex_array(Some(vao));
        if let Some(vbo) = &self.vbo {
            gl.bind_buffer(Gl::ARRAY_BUFFER, Some(vbo));
            let arr = js_sys::Float32Array::from(inter);
            gl.buffer_data_with_array_buffer_view(Gl::ARRAY_BUFFER, &arr, Gl::DYNAMIC_DRAW);
            let stride = 8 * 4;
            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_with_i32(0, 3, Gl::FLOAT, false, stride, 0);
            gl.enable_vertex_attrib_array(1);
            gl.vertex_attrib_pointer_with_i32(1, 3, Gl::FLOAT, false, stride, 12);
            gl.enable_vertex_attrib_array(2);
            gl.vertex_attrib_pointer_with_i32(2, 2, Gl::FLOAT, false, stride, 24);
        }
        if let Some(ibo) = &self.ibo {
            gl.bind_buffer(Gl::ELEMENT_ARRAY_BUFFER, Some(ibo));
            let arr = js_sys::Uint32Array::from(indices);
            gl.buffer_data_with_array_buffer_view(Gl::ELEMENT_ARRAY_BUFFER, &arr, Gl::DYNAMIC_DRAW);
        }
        self.count = indices.len() as i32;
        gl.bind_vertex_array(None);
    }

    pub fn draw(&self, gl: &Gl) {
        if self.count > 0 {
            if let Some(vao) = &self.vao {
                gl.bind_vertex_array(Some(vao));
                gl.draw_elements_with_i32(Gl::TRIANGLES, self.count, Gl::UNSIGNED_INT, 0);
                gl.bind_vertex_array(None);
            }
        }
    }
}

impl Default for SurfaceMesh {
    fn default() -> Self {
        Self::new()
    }
}

/// Single LINE_STRIP polyline (flat xyz, stride 12 bytes).
pub struct LineMesh {
    vao: Option<WebGlVertexArrayObject>,
    vbo: Option<WebGlBuffer>,
    pub count: i32,
}

impl LineMesh {
    pub fn new() -> Self {
        LineMesh {
            vao: None,
            vbo: None,
            count: 0,
        }
    }

    pub fn upload(&mut self, gl: &Gl, flat: &[f32]) {
        if flat.is_empty() {
            self.count = 0;
            return;
        }
        if self.vao.is_none() {
            self.vao = gl.create_vertex_array();
            self.vbo = gl.create_buffer();
        }
        gl.bind_vertex_array(self.vao.as_ref());
        if let Some(vbo) = &self.vbo {
            gl.bind_buffer(Gl::ARRAY_BUFFER, Some(vbo));
            let arr = js_sys::Float32Array::from(flat);
            gl.buffer_data_with_array_buffer_view(Gl::ARRAY_BUFFER, &arr, Gl::DYNAMIC_DRAW);
            gl.enable_vertex_attrib_array(0);
            gl.vertex_attrib_pointer_with_i32(0, 3, Gl::FLOAT, false, 12, 0);
        }
        self.count = (flat.len() / 3) as i32;
        gl.bind_vertex_array(None);
    }

    pub fn draw(&self, gl: &Gl) {
        if self.count >= 2 {
            if let Some(vao) = &self.vao {
                gl.bind_vertex_array(Some(vao));
                gl.draw_arrays(Gl::LINE_STRIP, 0, self.count);
                gl.bind_vertex_array(None);
            }
        }
    }
}

impl Default for LineMesh {
    fn default() -> Self {
        Self::new()
    }
}

/// Low-poly UV sphere, interleaved like SurfaceMesh (uv zeros).
pub fn sphere_geometry(radius: f32, seg_u: usize, seg_v: usize) -> (Vec<f32>, Vec<u32>) {
    let mut inter: Vec<f32> = Vec::new();
    let mut idx: Vec<u32> = Vec::new();
    for j in 0..=seg_v {
        let phi = (j as f32 / seg_v as f32) * std::f32::consts::PI;
        for i in 0..=seg_u {
            let th = (i as f32 / seg_u as f32) * 2.0 * std::f32::consts::PI;
            let nx = phi.sin() * th.cos();
            let ny = phi.cos();
            let nz = phi.sin() * th.sin();
            inter.extend_from_slice(&[
                nx * radius,
                ny * radius,
                nz * radius,
                nx,
                ny,
                nz,
                0.0,
                0.0,
            ]);
        }
    }
    let row = (seg_u + 1) as u32;
    for j in 0..seg_v as u32 {
        for i in 0..seg_u as u32 {
            let a = j * row + i;
            idx.extend_from_slice(&[a, a + 1, a + row, a + 1, a + row + 1, a + row]);
        }
    }
    (inter, idx)
}

/// Convert a roadgen MeshData into the interleaved layout.
pub fn interleave(positions: &[f32], normals: &[f32], uvs: &[f32]) -> Vec<f32> {
    let n = positions.len() / 3;
    let mut out = Vec::with_capacity(n * 8);
    for i in 0..n {
        out.extend_from_slice(&[
            positions[i * 3],
            positions[i * 3 + 1],
            positions[i * 3 + 2],
            normals.get(i * 3).copied().unwrap_or(0.0),
            normals.get(i * 3 + 1).copied().unwrap_or(1.0),
            normals.get(i * 3 + 2).copied().unwrap_or(0.0),
            uvs.get(i * 2).copied().unwrap_or(0.0),
            uvs.get(i * 2 + 1).copied().unwrap_or(0.0),
        ]);
    }
    out
}

pub fn dom_canvas(el: &web_sys::Element) -> HtmlCanvasElement {
    el.clone().dyn_into::<HtmlCanvasElement>().unwrap()
}
