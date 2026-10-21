# godot-road-generator → Rust: 100% Parity Matrix & Migration Plan

Source analyzed: `TheDuckCow/godot-road-generator` @ master (414 files, addon ≈ 8.9k LOC GDScript).
Core model: **RoadManager → RoadContainer → RoadSegment → RoadPoint (+ RoadLane, RoadIntersection, RoadGraphNode)**.
Geometry: per-segment cubic **Bézier** (Godot `Curve3D`) lofted with **loop cuts every `density` meters** and an **8-column trimsheet** UV layout.

---

## 1. Class & property inventory (verbatim from source)

### RoadPoint (extends RoadGraphNode → Node3D) — `nodes/road_point.gd`, 1,411 LOC

| Property | Type | Default | Parity notes |
|---|---|---|---|
| `traffic_dir` | `Array[LaneDir]` | `[REV,REV,FWD,FWD]` | per-lane direction; drives lane widths & mesh |
| `lanes` | `Array[LaneType]` | auto | per-lane trimsheet slice (auto from `auto_lanes`) |
| `auto_lanes` | `bool` | `true` | auto-assign `lanes` from `traffic_dir` |
| `terminated` | `bool` | `false` | point is an open road end |
| `prior_mag` | `float` | `5.0` | Bézier handle magnitude toward prior point |
| `next_mag` | `float` | `5.0` | Bézier handle magnitude toward next point |
| `create_geo` | `bool` | `true` | toggle procedural mesh |
| `flatten_terrain` | `bool` | `true` | terrain flattening (engine-side) |
| `lane_width` | `float` | `4.0` (`DEFAULT_LANE_WIDTH`) | per-lane width, meters |
| `shoulder_width_l` | `float` | `2.0` | left shoulder |
| `shoulder_width_r` | `float` | `2.0` | right shoulder |
| `gutter_profile` | `Vector2` | `(2.0, -0.5)` | x = extra widening, y = sink below road |
| `alignment` | `Alignment` | `GEOMETRIC` | GEOMETRIC (center) / DIVIDER (align divider) |
| `underside_thickness` | `float` | `-1.0` | -1 = no underside box |
| `prior_pt_init` / `next_pt_init` | `NodePath` | — | graph connections |
| `use_parent_setting` | `bool` | `true` | inherit manager/container settings |

**Enums:** `LaneType {NO_MARKING, SHOULDER, SLOW, MIDDLE, FAST, TWO_WAY, ONE_WAY, SINGLE_LINE, TRANSITION_ADD, TRANSITION_REM}` · `LaneDir {NONE, FORWARD, REVERSE, BOTH}` · `TrafficUpdate {ADD_FORWARD, ADD_REVERSE, REM_FORWARD, REM_REVERSE, MOVE_DIVIDER_LEFT, MOVE_DIVIDER_RIGHT}` · `PointInit {NEXT, PRIOR, NEITHER}` · `Alignment {GEOMETRIC, DIVIDER}`

**Signal:** `on_transform(node, low_poly)` · **Const:** `SEG_DIST_MULT=8.0`, `DEFAULT_LANE_WIDTH=4.0`

### RoadSegment (extends Node3D) — `nodes/road_segment.gd`, 1,710 LOC

| Property / Const | Type | Default |
|---|---|---|
| `start_init` / `end_init` | `NodePath` | — |
| `DEFAULT_DENSITY` | `float` | `4.0` (meters between loop cuts) |
| `LOWPOLY_FACTOR` | `float` | `3.0` (÷3 loops in low-poly) |
| `uv_mul` | `Array` | `[7,0,1,2,3,4,5,6,7,7]` (LaneType→trimsheet col) |
| `EDGE_F_NAME` / `EDGE_R_NAME` / `EDGE_C_NAME` | — | forward/reverse/center divider edge curves |

**Signal:** `seg_ready(road_segment)`

Pipeline (`_update_curve` → `_build_geo`): builds a 2-point Bézier with handles
`± basis.z * mag` per end, `bake_interval = density`; loops = `⌊length/density⌋`
(÷`LOWPOLY_FACTOR` if low-poly); UV flow snap rules: `tiles<0.3` free slide,
`<0.75→0.5`, `<1.25→1.0`, `<1.75→1.5`, else `round`; `single_uv_height = min(lane_width)*4`;
`uv_width = 0.125` (8 columns). Cross-section per loop: gutters → shoulders → lanes,
**lane matching between endpoints** (`_match_lanes`) with `TRANSITION_ADD/REM` lanes
when endpoint lane configs differ; consistent-width lateral basis
(`_normal_for_offset_eased`); optional underside surface; edge curves
(`edge_F`, `edge_R`, `edge_C`) exposed as `Path3D`s for AI.

### RoadManager (extends Node3D) — `nodes/road_manager.gd`, 297 LOC

| Property | Type | Default |
|---|---|---|
| `material_resource` | `Material` | trimsheet material |
| `material_underside` | `Material` | — |
| `density` | `float` | `4.0` |
| `underside_thickness` | `float` | `-1.0` |
| `physics_material` | `PhysicsMaterial` | — |
| `collider_group_name` / `collider_meta_name` | `String` | `""` |
| `collision_layer` / `collision_mask` | `int` | `1` |
| `ai_lane_group` | `String` | `"road_lanes"` |
| `auto_refresh` | `bool` | `true` |

**Signals:** `on_road_updated(updated_segments)`, `on_container_transformed(rc)`

### RoadContainer (extends Node3D) — 1,498 LOC
`create_geo=true`, `material_resource`, `material_top_meshes[]`, `material_underside`,
`density=-1.0` (inherit), `use_lowpoly_preview=false`, `flatten_terrain=true`,
`underside_thickness=-1.0`, `physics_material`, `collider_group/meta_name`,
`override_collision_layers`, `generate_ai_lanes=false`, `ai_lane_group`, `auto_free_vehicles=true`,
`draw_lanes_editor=false`. Signal: `on_road_updated`, `on_transform`.

### RoadLane (extends Path3D) — 311 LOC
`lane_left/right/next/prior: NodePath`, `lane_next_tag/lane_prior_tag: String`,
`draw_in_game=false`, `draw_in_editor=false`, `lane_pointer_mesh`, `auto_free_vehicles=true`,
`reverse_direction=false`. Signal: `on_transform`. One `Path3D` per generated lane
for AI/vehicle navigation (`offset_curve` with per-end transition offsets).

### RoadIntersection (extends RoadGraphNode) — 375 LOC + `procgen/intersection_ngon.gd` (976 LOC)
N-gon intersection: N segments meet at one node; fan-triangulated hub, per-edge
transition geometry, corner curbs. Signal: `on_transform`.

### Ecosystem beyond geometry
Connectors (Terrain3D), `road_lane_agent` (369 LOC, vehicle agent), editor UI suite
(toolbar, point panel, gizmos, connection tool), demos (navigation A* graph, procedural
generator, performance stress test), 30+ unit test files (WAT/GUT).

---

## 2. Parity matrix

| Capability | Godot addon | Rust core (this repo) | Status |
|---|---|---|---|
| Spline: 2-point Bézier, per-point `prior_mag`/`next_mag` | ✅ | ✅ bezier-mag port | **P1 shipped** |
| Loop cuts every `density` m | ✅ | ✅ | **P1 shipped** |
| `low_poly` ÷3 loops | ✅ | ✅ | **P1 shipped** |
| Per-point `lane_width`, shoulders L/R | ✅ | ✅ per-node | **P1 shipped** |
| `gutter_profile` (x widen, y sink) | ✅ | ✅ | **P1 shipped** |
| Per-lane `traffic_dir` (REV/FWD) | ✅ | ✅ per-node lane editor | **P1 shipped** |
| `auto_lanes` → trimsheet columns | ✅ | ✅ (`uv_mul` mapping) | **P1 shipped** |
| 8-col trimsheet UV + UV snap rules | ✅ | ✅ + procedural 8-col texture | **P1 shipped** |
| `Alignment` GEOMETRIC/DIVIDER | ✅ | ✅ | **P1 shipped** |
| `underside_thickness` underside mesh | ✅ | ✅ | **P1 shipped** |
| Lane `edge_F/R/C` curves for AI | ✅ | ✅ line renderables | **P1 shipped** |
| Per-lane `RoadLane` paths | ✅ | ✅ lane paths output | **P1 shipped** |
| Lane transitions (`TRANSITION_ADD/REM`) | ✅ full matcher | linear cross-section blend | 🟡 approximation |
| Junctions: proximity-shared RoadPoints | ✅ RoadGraphNode join | ✅ `roads[]` networks, ε-join | **P3 shipped** |
| `generate_mesh_at_intersections` trim | ✅ | ✅ per-point trim zones | **P3 shipped** |
| `edge_edge_clipping_offset` | ✅ | ✅ extends trim | **P3 shipped** |
| `RoadIntersection` n-gon hub | ✅ fan hub + corner curbs + transition stitching | ✅ chamfered pavement n-gon + arm markings | 🟡 P3 simplified (convex plate) |
| Stop lines / crosswalks at junctions | ✅ (editor decor) | ✅ stop line + zebra per arm | **P3 shipped** |
| Terrain flatten / Terrain3D connector | ✅ | N/A (flat ground) | ⛔ engine-side |
| Collision layers, StaticBody3D | ✅ | N/A (web viewer) | ⛔ engine-side |
| RoadLane agents (vehicles) | ✅ | — | ⛔ Phase 4 |
| Editor UI parity (panels/gizmos) | ✅ | property panel parity subset | 🟡 Phase 2 |
| Signals (`on_road_updated`, …) | ✅ | JS callbacks possible | 🟡 Phase 2 |

Legend: ✅ = implemented · 🟡 = partial/approximation (documented) · ⛔ = planned

## 3. Phased shift plan

- **Phase 1 (shipped in this commit):** RoadPoint/RoadSegment/Manager data model in
  `rust/roadgen` v2 — Bézier-with-mag spline, density loop cuts, per-point widths,
  traffic_dir lanes, gutters, underside, trimsheet UVs, edge/lane curve outputs,
  low-poly mode. Web viewer exposes every generated property.
- **Phase 2:** full `_match_lanes` transition matcher (TRANSITION_ADD/REM lanes),
  signal-equivalent callbacks (`onRoadUpdated` into JS), manager/container inheritance
  chain (`use_parent_setting`), savable scene format (JSON ⇄ Godot `.tscn` mapper).
- **Phase 3 (shipped):** junction system — `roads[]` networks in `godot.rs`, ε-proximity
  junction detection, `generate_mesh_at_intersections`/`edge_edge_clipping_offset` trims,
  chamfered pavement n-gon (`intersection_ngon` parity, simplified convex plate), stop
  lines + zebra crosswalks per arm, junction-aware edge/centerline strips, full RoadPoint
  inspector in the viewer (lanes editor, junction badges 2/3/4-way). Remaining delta:
  dynamic re-paving of non-convex hubs and per-edge transition stitching.
- **Phase 4:** AI lanes: `RoadLane` graph + `road_lane_agent` vehicles, A* demo parity.
- **Phase 5 (optional native):** same core linked into Bevy/rend3 via cdylib — parity
  without the browser.

Key documented deltas: lateral basis uses tangent-derived frame (Godot eases between
endpoint bases — visually equivalent for edited roads); lane transition uses linear
blending until the full matcher lands.
