use roadgen::{godot, godot::RoadPointData};
fn pt(x: f32, y: f32, z: f32) -> RoadPointData {
    RoadPointData { x, y, z, prior_mag: 5.0, next_mag: 5.0, lane_width: 4.0,
        shoulder_width_l: 2.0, shoulder_width_r: 2.0, gutter_profile_x: 2.0,
        gutter_profile_y: -0.5, traffic_dir: vec![2, 2, 1, 1], alignment: 0,
        underside_thickness: -1.0, terminated: false }
}
fn main() {
    let req = godot::GenRequest {
        manager: godot::ManagerData { density: 4.0, underside_thickness: -1.0, low_poly: false, auto_lanes: true },
        points: vec![pt(-70.0,0.0,-55.0), pt(-30.0,2.0,-18.0), pt(0.0,0.0,12.0), pt(32.0,4.0,18.0), pt(72.0,0.0,48.0)],
    };
    let r = godot::generate_road(&req);
    let p = &r.road.positions;
    println!("verts={} tris={} err={:?}", p.len()/3, r.road.indices.len()/3, r.error);
    let mut min = [f32::MAX;3]; let mut max = [f32::MIN;3]; let mut nan = 0;
    for v in p { if v.is_nan() { nan += 1; } }
    for i in 0..p.len()/3 {
        for k in 0..3 {
            min[k] = min[k].min(p[i*3+k]); max[k] = max[k].max(p[i*3+k]);
        }
    }
    println!("bbox min={min:?} max={max:?} nan={nan}");
    println!("first 3 verts: {:?} {:?} {:?}", &p[0..3], &p[3..6], &p[6..9]);
    println!("first 12 idx: {:?}", &r.road.indices[0..12]);
}
