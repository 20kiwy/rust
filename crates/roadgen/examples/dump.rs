use roadgen::{godot, godot::RoadPointData};
fn pt(x: f32, y: f32, z: f32) -> RoadPointData {
    RoadPointData {
        x, y, z,
        prior_mag: 5.0, next_mag: 5.0, lane_width: 4.0,
        shoulder_width_l: 2.0, shoulder_width_r: 2.0,
        gutter_profile_x: 2.0, gutter_profile_y: -0.5,
        traffic_dir: vec![2, 2, 1, 1], alignment: 0,
        underside_thickness: -1.0, terminated: false,
        generate_mesh_at_intersections: true,
        edge_edge_clipping_offset: 0.0,
    }
}
fn main() {
    // Single road (regression vs previous build).
    let req = godot::GenRequest {
        manager: godot::ManagerData { density: 4.0, underside_thickness: -1.0, low_poly: false, auto_lanes: true },
        points: vec![pt(-70.0,0.0,-55.0), pt(-30.0,2.0,-18.0), pt(0.0,0.0,12.0), pt(32.0,4.0,18.0), pt(72.0,0.0,48.0)],
        roads: vec![],
    };
    let r = godot::generate_road(&req);
    println!("single: verts={} tris={} len={:.1} jct={} err={:?}",
        r.vertex_count, r.triangle_count, r.length, r.junction_count, r.error);

    // Network: 4-way + T junctions.
    let main = vec![pt(-100.0,0.0,0.0), pt(-50.0,0.0,0.0), pt(0.0,0.0,0.0), pt(50.0,0.0,0.0), pt(100.0,0.0,0.0)];
    let cross1 = vec![pt(0.0,0.0,-55.0), pt(0.0,0.0,0.0), pt(0.0,0.0,55.0)];
    let cross2 = vec![pt(50.0,0.0,50.0), pt(50.0,0.0,0.0)];
    let req2 = godot::GenRequest {
        manager: godot::ManagerData { density: 4.0, underside_thickness: -1.0, low_poly: false, auto_lanes: true },
        points: vec![],
        roads: vec![main, cross1, cross2],
    };
    let r2 = godot::generate_road(&req2);
    println!("network: verts={} tris={} len={:.1} jct={} kinds={:?} err={:?}",
        r2.vertex_count, r2.triangle_count, r2.length, r2.junction_count,
        r2.junction_info.iter().map(|j| j.kind).collect::<Vec<_>>(), r2.error);
    println!("pavement tris={} marks tris={}", r2.junctions.indices.len()/3, r2.junction_marks.indices.len()/3);
    let nan = r2.junctions.positions.iter().chain(r2.junction_marks.positions.iter()).filter(|v| v.is_nan()).count();
    println!("junction nan count: {nan}");

    // Trimmed-vs-untrimmed parity check: disable generate_mesh_at_intersections.
    let mut main2 = vec![pt(-100.0,0.0,0.0), pt(-50.0,0.0,0.0), pt(0.0,0.0,0.0), pt(50.0,0.0,0.0), pt(100.0,0.0,0.0)];
    main2[2].generate_mesh_at_intersections = false;
    let r3 = godot::generate_road(&godot::GenRequest {
        manager: godot::ManagerData { density: 4.0, underside_thickness: -1.0, low_poly: false, auto_lanes: true },
        points: vec![],
        roads: vec![main2, vec![pt(0.0,0.0,-55.0), pt(0.0,0.0,0.0), pt(0.0,0.0,55.0)]],
    });
    println!("untrimmed: road tris={} (should exceed trimmed network road tris={})",
        r3.road.indices.len()/3, r2.road.indices.len()/3);
}
