//! GLSL (WebGL2 / GLSL ES 3.00) programs. The godot trimsheet columns are
//! reproduced procedurally in the fragment shader — zero textures, zero TS.

pub const SURFACE_VS: &str = r#"#version 300 es
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_nrm;
layout(location=2) in vec2 a_uv;
uniform mat4 u_vp;
uniform vec3 u_offset;
out vec3 v_pos;
out vec3 v_nrm;
out vec2 v_uv;
void main() {
  vec3 wp = a_pos + u_offset;
  v_pos = wp;
  v_nrm = a_nrm;
  v_uv = a_uv;
  gl_Position = u_vp * vec4(wp, 1.0);
}
"#;

/// u_kind: 0 = godot road (8-column trimsheet in uv.x)
///         1 = flat u_color (arch markings / handles / misc)
///         2 = arch road (shoulder bands at uv.x edges)
///         3 = ground grid (procedural, world xz)
pub const SURFACE_FS: &str = r#"#version 300 es
precision highp float;
in vec3 v_pos;
in vec3 v_nrm;
in vec2 v_uv;
uniform int u_kind;
uniform vec3 u_color;
uniform vec3 u_eye;
uniform vec3 u_light;
out vec4 o;

const vec3 SKY     = vec3(0.847, 0.867, 0.827);
const vec3 ASPHALT = vec3(0.155, 0.160, 0.170);
const vec3 EARTH   = vec3(0.330, 0.300, 0.260);
const vec3 SHOULDER= vec3(0.480, 0.470, 0.450);
const vec3 WHITE   = vec3(0.900);
const vec3 YELLOW  = vec3(0.850, 0.640, 0.120);
const vec3 GRASS   = vec3(0.490, 0.604, 0.427);

vec3 shade(vec3 base) {
  vec3 n = normalize(v_nrm);
  float d = max(dot(n, normalize(u_light)), 0.0);
  float back = 0.18 * max(dot(n, normalize(-u_light)), 0.0);
  return base * (0.52 + 0.72 * d + back);
}

float dash_on(float freq, float duty) {
  return 1.0 - step(duty, fract(v_uv.y * freq));
}

void main() {
  vec3 base;
  if (u_kind == 0) {
    // godot 8-column trimsheet (uv.x band * 8): 0 shoulder, 1 slow,
    // 2 middle, 3 fast, 4 two-way, 5 one-way, 6 single-line, 7 none.
    float b = clamp(v_uv.x, 0.0, 0.99999) * 8.0;
    float col = floor(b);
    float f = fract(b);
    base = (col < 0.5) ? SHOULDER : ASPHALT;
    if (v_nrm.y < 0.5) { base = EARTH; }          // gutters' walls + underside
    float edge = step(f, 0.075) + step(0.925, f);
    float dash = dash_on(4.0, 0.55);
    vec3 line_col = (col > 2.5 && col < 3.5) ? YELLOW : WHITE;
    float line = 0.0;
    if (col > 0.5 && col < 6.5) {
      if (col < 1.5)      { line = edge; }                 // SLOW: solid white
      else if (col < 2.5) { line = edge * dash; }          // MIDDLE: dashed white
      else if (col < 3.5) { line = edge; }                 // FAST: solid yellow
      else                { line = edge * dash; }          // TWO_WAY/ONE_WAY/SINGLE
    }
    base = mix(base, line_col, clamp(line, 0.0, 1.0) * 0.95);
  } else if (u_kind == 2) {
    float sh = step(v_uv.x, 0.17) + step(0.83, v_uv.x);
    base = mix(ASPHALT, SHOULDER, clamp(sh, 0.0, 1.0));
    if (v_nrm.y < 0.5) { base = EARTH; }
  } else if (u_kind == 3) {
    // Ground: grass + subtle grid.
    vec2 g = abs(fract(v_pos.xz / 10.0) - 0.5);
    float grid = 1.0 - smoothstep(0.0, 0.035, min(g.x, g.y));
    base = mix(GRASS, GRASS * 0.92, grid);
  } else {
    base = u_color;
  }

  vec3 c = shade(base);
  float fd = length(u_eye - v_pos);
  c = mix(c, SKY, smoothstep(240.0, 700.0, fd));
  o = vec4(c, 1.0);
}
"#;

pub const LINE_VS: &str = r#"#version 300 es
layout(location=0) in vec3 a_pos;
uniform mat4 u_vp;
out vec3 v_pos;
void main() {
  v_pos = a_pos;
  gl_Position = u_vp * vec4(a_pos, 1.0);
}
"#;

pub const LINE_FS: &str = r#"#version 300 es
precision highp float;
in vec3 v_pos;
uniform vec3 u_color;
uniform vec3 u_eye;
out vec4 o;
const vec3 SKY = vec3(0.847, 0.867, 0.827);
void main() {
  float fd = length(u_eye - v_pos);
  o = vec4(mix(u_color, SKY, smoothstep(240.0, 700.0, fd)), 1.0);
}
"#;
