# Headless consistency checks of the Moon terrain mesh with the procedural surface detail (moon procedural surface plan,
# Task 7; needs data/ with the lunar terrain and the Diviner rock map):
# - a level-18 node at the IM-2 site: its mesh vertices are the core's band-limited heights (surface_samples_detail at
#   the level's vertex spacing) within 0.004 m;
# - parent / child: the child fully morphed is its parent's mesh (vertices and normals), unmorphed it carries the finer
#   detail; two neighbours of a level share their edge (unmorphed and morphed);
# - horizon maps are baked only down to the measured data's level; deeper nodes draw with their owner's map, whose grid
#   they address through horizon_grid_scale / offset (same directions);
# - the tree refines to level 22 (1 cm spacing) 2 m above the ground with the detail on, and stops at the LOLA level as
#   before with it off;
# - detail off: no node samples the detail (band-limited path off) and every node owns its horizon.
# - Hapke (Task 9): the normalisation identity (phase-0 disc, Ruling T9 I1) of Atmosphere.regolith_reflectance (the eye adaptation's mirror of the shaders)
#   with Hapke x hapke_normalisation against the Lommel-Seeliger law within 3 %; with a GPU (run without --headless) the
#   shader parity: a flat patch drawn through moon_hapke.gdshaderinc and through the production wiring (moon_regolith, configure_moon_brdf globals) at four geometries against the core's
#   hapke_reflectance (StarsimCore.moon_hapke_reflectance) within 2 %, after a readback calibration.
# - Regolith detail shading (Task 10; MoonDetailChecks): the summed mesh + texture roughness at Guo's 125 mm baseline
#   (4 mm +- 10 % at levels 22 .. 17), the residual theta_bar and light factors continuous in the node spacing, the rock
#   abundance grid; with a GPU flat patches through moon_detail_patch.gdshader (the production detail include) against
#   the core: (a) the statistical radiance (residual theta_bar, Smith, rock albedo / shadow / lit faces) with and without
#   the texture modulation within 2 %, (b) an earthshine-only patch (Sun below the horizon) equal to the core's
#   earthshine with no shadow share at any abundance and spacing.
# - Drawn rocks (Task 11; MoonRockChecks): a node's rock buffers against the core's rocks found cell by cell (ids,
#   centres within 1 mm); with a GPU the radiance gate with real rocks crossing the threshold: a 120 m patch with the
#   core's rocks drawn through moon_rock.gdshader above a fixed threshold diameter (growth band to 2 x) on the
#   statistical ground of moon_detail_patch.gdshader, mean against the core's all-statistical expectation within 2 %;
#   a camera jump of 10 km with a streamed tree and the rock layer: the main thread under 5 ms per frame.
#   --rocks_only: only these (and the Hapke calibration step).
# Run: godot --headless --path game --script res://tests/moon_surface_check.gd [-- --measure]
#      godot --path game --script res://tests/moon_surface_check.gd [-- --hapke_only]  (GPU: + the shader parity;
#      --hapke_only: only the Hapke checks, no terrain data needed; --detail_only: the Task 10 detail checks and the Hapke
#      checks, no quadtree runs)
# --measure also prints node counts and summed build time of a synchronous tree converged 2 / 20 / 200 / 2000 m above
# the site (detail off / on), the time of a 67 x 67 surface_samples_detail batch per level and moon_rocks on rock-rich
# cells (the Diviner map's maxima and Tycho).
extends SceneTree

const R := HeightSource.MOON_RADIUS_M
const CELLS := TerrainNodeBuilder.CELLS
const ROW := TerrainNodeBuilder.ROW
const MESH_TOLERANCE_M := 0.004  ## the brief's mesh-vs-core tolerance (float32 vertices, a few um at level 18)
const SEAM_TOLERANCE_M := 1.0e-4  ## float32 vertices relative to two node centres tens of metres apart
const MAX_FRAMES := 4000
const CLIMB_HEIGHTS_M := [2.0, 20.0, 200.0, 2000.0]
const FULL_MOON_TOLERANCE := 0.03  ## the plan's full-Moon disc tolerance (Hapke against the former law)
const PARITY_TOLERANCE := 0.02  ## the plan's shader-vs-core tolerance
const PARITY_GAIN := 2.0  ## ALBEDO = gain x I/F: values 0.1-0.6 (HDR readback, half-float ~5e-4 relative)
const PARITY_CALIBRATION := 0.25  ## hapke_patch.gdshader's constant in calibrate mode
const PARITY_FRAMES := 4  ## frames rendered after each uniform change before the readback
## (incidence, emission, phase) in deg: the standard geometry, oblique, grazing view, near opposition.
const PARITY_GEOMETRIES := [Vector3(30, 0, 30), Vector3(60, 30, 50), Vector3(20, 75, 85), Vector3(10, 8, 3)]
const PARITY_THETA_OVERRIDE_RAD := 0.1  ## a terrain node's hapke_theta_bar_rad for the production override check
const DETAIL_VIEWPORT := 512  ## the detail patches' readback (whole viewport mean)
## Detail patches: (incidence, emission, phase) deg, patch edge m, node spacing m, horizon spacing m, abundance,
## textures, sun / earth on (an earthshine-only patch has the Sun 5 deg below the horizon).
const DETAIL_STEPS := [
	[Vector3(83, 30, 60), 50.0, 0.3264, 20.0, 1.0, false, true],
	[Vector3(75, 0, 75), 50.0, 0.3264, 20.0, 1.0, false, true],
	[Vector3(60, 45, 30), 50.0, 0.0102, 20.0, 3.0, false, true],
	[Vector3(83, 30, 60), 50.0, 0.3264, 20.0, 1.0, true, true],
	[Vector3(75, 0, 75), 50.0, 0.3264, 20.0, 1.0, true, true],
	[Vector3(83, 30, 60), 2.0, 0.0102, 20.0, 1.0, true, true],
	[Vector3(60, 45, 30), 2.0, 0.0102, 20.0, 1.0, true, true],
	[Vector3(83, 30, 60), 50.0, 0.0102, 20.0, 1.0, false, false],
	[Vector3(83, 30, 60), 50.0, 0.0102, 20.0, 40.0, true, false],
	[Vector3(83, 30, 60), 50.0, 3000.0, 3000.0, 0.0, false, false],
]
## The Diviner rock abundance map's highest valid 3 x 3 pixel means (selenographic deg; RA 0.357 / 0.349 / 0.343,
## searched over the whole 60 S - 60 N map in fix round 1) and Tycho's floor (-43.31, -11.36) for comparison.
## Rock patches (Task 11): (incidence, emission, phase) deg, threshold diameter m, sun / earth on. Patch edge
## ROCK_PATCH_EDGE_M (the rocks gathered over it), viewed ROCK_PATCH_VIEW_M wide, node spacing 0.3264 m, horizon 20 m,
## abundance 1 (the site), regolith textures off, rock textures on.
const ROCK_STEPS := [
	[Vector3(83, 30, 60), 0.1, true],
	[Vector3(83, 30, 60), 0.2, true],
	[Vector3(83, 30, 60), 0.4, true],
	[Vector3(83, 30, 60), 0.8, true],
	[Vector3(75, 0, 75), 0.1, true],
	[Vector3(75, 0, 75), 0.4, true],
	[Vector3(60, 45, 30), 0.1, true],
	[Vector3(60, 45, 30), 0.4, true],
	[Vector3(83, 30, 60), 0.1, false],
]
const ROCK_PATCH_EDGE_M := 140.0
const ROCK_PATCH_VIEW_M := 100.0  ## the patch plane (the camera shows its central half)
const ROCK_PATCH_SPACING_M := 0.3264
const ROCK_PATCH_HORIZON_M := 20.0
const ROCK_PATCH_MIN_D_M := 0.1  ## the smallest threshold of ROCK_STEPS: the rocks gathered
## [D] The rock patch's eye-adaptation gain (any value != 1: the gate then tests that the rock shader applies it).
const ROCK_PATCH_EXPOSURE_GAIN := 4.0
const ROCKY_POINTS := [Vector2(39.3594, -0.1719), Vector2(8.1094, -53.9844), Vector2(-24.8281, -45.1406),
		Vector2(-43.31, -11.36)]

var _failures := 0
var _core: StarsimCore
var _site := Vector2.ZERO
var _config_dir := ""
## Shader parity state (_start_parity / _process).
var _hapke: Dictionary = {}
var _parity_viewport: SubViewport
var _parity_material: ShaderMaterial
var _parity_production: ShaderMaterial  ## through moon_regolith.gdshaderinc and the configure_moon_brdf globals
var _parity_patch: MeshInstance3D
var _parity_steps: Array = []  ## {kind, geometry, theta}: calibration, include-only patch, production (globe / override)
var _parity_camera: Camera3D
var _parity_step := -1
var _parity_wait := 0
var _parity_srgb := false  ## the readback came back sRGB-encoded (calibration), decoded before the comparison
var _detail_stats: Dictionary = {}  ## StarsimCore.moon_unresolved_shading (full runs)
var _detail_material: ShaderMaterial
var _detail_tiles: Array = []
var _rock_layer: MoonRockLayer  ## the rock patches' and transform checks' layer (templates, material)
var _rock_patch: MultiMeshInstance3D
var _rock_material: ShaderMaterial
var _rock_cover := 0.0  ## realised plan-view cover of the patch's rocks (D >= ROCK_PATCH_MIN_D_M) in the viewed square
var _rock_diameters := PackedFloat64Array()  ## the viewed square's rock diameters (the realised expectation)
var _rock_view_edge_m := 1.0
var _rocks_only := false
var _rock_patch_abundance := 1.0  ## the rock patch's abundance factor (--rock_patch_abundance: a boulder-field flank)


func _initialize() -> void:
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	var site: Dictionary = TomlLite.load_file(config_dir.path_join("lunar_sites.toml"))["site"][0]
	_site = Vector2(float(site["latitude_deg"]), float(site["longitude_deg"]))
	_core = StarsimCore.new()
	_config_dir = config_dir
	if OS.get_cmdline_user_args().has("--hapke_only"):
		_hapke_checks()
		return
	var loaded := _core.load_terrain(config_dir)
	if not bool(loaded.get("moon", false)) or not bool(_core.moon_surface_statistics().get("present", false)):
		print("  note  data/ lunar terrain or the detailed surface absent: checks skipped")
		print("MOON SURFACE CHECK SKIPPED")
		quit(0)
		return
	_core.set_moon_surface_detail(true)
	var detail_only := OS.get_cmdline_user_args().has("--detail_only")
	_rocks_only = OS.get_cmdline_user_args().has("--rocks_only")
	if _rocks_only:
		_detail_stats = _core.moon_unresolved_shading()
		_rock_checks()
		_hapke_checks()
		return
	if not detail_only:
		_check_mesh_matches_core()
		_check_parent_child()
		_check_seams_face_edge_and_pole()
		_check_tree_and_horizons()
	_detail_stats = _core.moon_unresolved_shading()
	_check(bool(_detail_stats.get("present", false)), "unresolved-surface statistics built (moon_unresolved_shading)")
	if not _detail_stats.is_empty() and bool(_detail_stats["present"]):
		MoonDetailChecks.continuity(_core, _check)
		MoonDetailChecks.abundance_grid(_core, _check)
		MoonDetailChecks.summed_roughness(_core, _site, _check)
	if not detail_only:
		_rock_checks()
	_core.set_moon_surface_detail(false)
	if not detail_only:
		_check_detail_off()
	if OS.get_cmdline_user_args().has("--measure"):
		_measure()
	_hapke_checks()


## Task 11: the rock layer (on a terrain material configured like the game's), its node buffers against the core, the
## camera jump with a streamed tree.
func _rock_checks() -> void:
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain_moon.gdshader")
	MoonDetailShading.configure(_core, material, true)
	_rock_layer = MoonRockLayer.new(_core, material, false)  # lists and the patch only: no pool
	MoonRockChecks.transforms(_core, _rock_layer, _site, _check)
	MoonRockChecks.budget(_check)
	if OS.get_cmdline_user_args().has("--no_jump"):  # diagnostics: the rock patches only
		return
	var site_fixed := CubeSphere.geodetic_direction(_site.x, _site.y) * (R + _ground_m(true) + 2.0)
	if OS.get_cmdline_user_args().has("--jump_baseline"):  # the same jump without the rock layer (info)
		var bare := TerrainQuadtree.new()
		root.add_child(bare)
		bare.setup(_heights(), null, R, material)
		MoonRockChecks.camera_jump(bare, null, site_fixed, _check)
		bare.free()
	var tree := TerrainQuadtree.new()
	root.add_child(tree)
	var layer := MoonRockLayer.new(_core, material)
	layer.pixel_angle_rad = MoonRockChecks.CHECK_PIXEL_ANGLE
	tree.rock_layer = layer
	tree.setup(_heights(), null, R, material)
	MoonRockChecks.camera_jump(tree, layer, site_fixed, _check)
	tree.free()
	# The same jump in the Diviner boulder field (review R3: the longest lists, chunked; the pool bound).
	var rocky: Vector2 = ROCKY_POINTS[0]
	var rocky_dir := CubeSphere.geodetic_direction(rocky.x, rocky.y)
	var rocky_ground: Dictionary = _core.surface_samples_detail("moon", PackedFloat64Array([rocky.x]),
			PackedFloat64Array([rocky.y]), 0.0, INF)
	var rocky_fixed := rocky_dir * (R + (rocky_ground["height_m"] as PackedFloat64Array)[0] + 2.0)
	var rocky_tree := TerrainQuadtree.new()
	root.add_child(rocky_tree)
	var rocky_layer := MoonRockLayer.new(_core, material)
	rocky_layer.pixel_angle_rad = MoonRockChecks.CHECK_PIXEL_ANGLE
	rocky_tree.rock_layer = rocky_layer
	rocky_tree.setup(_heights(), null, R, material)
	print("  info  boulder field (%.2f N, %.2f E, abundance %.1f): the same camera jump" % [rocky.x, rocky.y,
			rocky_layer.abundance_at(rocky_dir)])
	MoonRockChecks.camera_jump(rocky_tree, rocky_layer, rocky_fixed, _check)
	rocky_tree.free()


## The Hapke checks; the shader parity (GPU only) continues in _process, then _finish.
func _hapke_checks() -> void:
	_check_hapke_full_moon()
	if DisplayServer.get_name() == "headless":
		print("  note  Hapke shader parity needs a GPU (run without --headless): skipped")
		_finish()
		return
	_start_parity()


func _finish() -> void:
	print("MOON SURFACE CHECK ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


func _check(ok: bool, what: String) -> void:
	print("  %s  %s" % ["ok  " if ok else "FAIL", what])
	if not ok:
		_failures += 1


func _heights() -> HeightSource:
	var heights := HeightSource.new()
	heights.body = "moon"
	heights.core = _core
	return heights


func _site_key(level: int) -> Vector4i:
	return CubeSphere.key_at(CubeSphere.geodetic_direction(_site.x, _site.y), level)


## Body-fixed position of grid vertex v of a built node (morph 0, or fully morphed with morph 1).
static func _vertex_fixed(node: Dictionary, v: int, morph := 0.0) -> Vector3:
	var p: Vector3 = node["vertices"][v]
	var m: PackedFloat32Array = node["morph"]
	p += Vector3(m[4 * v], m[4 * v + 1], m[4 * v + 2]) * morph
	return node["center_fixed"] + Frames.from_render(p)


# Level 18 at the site: every grid vertex's height above the sphere equals surface_samples_detail at the level's
# spacing at that vertex (lat / lon from the vertex direction).
func _check_mesh_matches_core() -> void:
	var key := _site_key(18)
	var node := TerrainNodeBuilder.build(key, _heights(), null, R)
	var lats := PackedFloat64Array()
	var lons := PackedFloat64Array()
	var drawn := PackedFloat64Array()
	for v in TerrainNodeBuilder.GRID_VERTICES:
		var p := _vertex_fixed(node, v)
		var ll := CubeSphere.lat_lon_deg(p.normalized())
		lats.append(ll.x)
		lons.append(ll.y)
		drawn.append(p.length() - R)
	var spacing := TerrainNodeBuilder.level_spacing_m(18, R)
	var core_h: PackedFloat64Array = _core.surface_samples_detail("moon", lats, lons, spacing, INF)["height_m"]
	var lola: PackedFloat64Array = _core.surface_samples("moon", lats, lons)["height_m"]
	var worst := 0.0
	var detail := 0.0
	for v in drawn.size():
		worst = maxf(worst, absf(drawn[v] - core_h[v]))
		detail = maxf(detail, absf(core_h[v] - lola[v]))
	_check(worst < MESH_TOLERANCE_M and detail > 10.0 * MESH_TOLERANCE_M,
			"level 18 at the site: mesh = band-limited core heights within %.4f m (worst %s m; detail vs LOLA up to %.3f m)"
			% [MESH_TOLERANCE_M, String.num_scientific(worst), detail])


# Parent (level 17) and child (level 18) at the site, and the child's neighbour of its level.
func _check_parent_child() -> void:
	var heights := _heights()
	var child_key := _site_key(18)
	var parent_key := CubeSphere.parent(child_key)
	var parent := TerrainNodeBuilder.build(parent_key, heights, null, R)
	var child := TerrainNodeBuilder.build(child_key, heights, null, R)
	var off_a := (child_key.z & 1) * (CELLS / 2)
	var off_b := (child_key.w & 1) * (CELLS / 2)
	var worst_morphed := 0.0
	var worst_unmorphed := 0.0
	var worst_normal := 0.0
	var parent_normals: PackedVector3Array = parent["normals"]
	var custom3: PackedFloat32Array = child["custom3"]
	for b in ROW:
		for a in ROW:
			var v := b * ROW + a
			# The parent's surface at the child vertex: its vertex (even) or the midpoint of the parent edge / diagonal
			# the vertex lies on (the parent's triangles pass there).
			var pa := off_a + (a >> 1)
			var pb := off_b + (b >> 1)
			var target: Vector3
			if a & 1 and b & 1:
				target = (_vertex_fixed(parent, pb * ROW + pa + 1) + _vertex_fixed(parent, (pb + 1) * ROW + pa)) * 0.5
			elif b & 1:
				target = (_vertex_fixed(parent, pb * ROW + pa) + _vertex_fixed(parent, (pb + 1) * ROW + pa)) * 0.5
			elif a & 1:
				target = (_vertex_fixed(parent, pb * ROW + pa) + _vertex_fixed(parent, pb * ROW + pa + 1)) * 0.5
			else:
				target = _vertex_fixed(parent, pb * ROW + pa)
				var n := Vector3(custom3[3 * v], custom3[3 * v + 1], custom3[3 * v + 2])
				worst_normal = maxf(worst_normal, n.angle_to(parent_normals[pb * ROW + pa]))
			worst_morphed = maxf(worst_morphed, _vertex_fixed(child, v, 1.0).distance_to(target))
			worst_unmorphed = maxf(worst_unmorphed, _vertex_fixed(child, v).distance_to(target))
	_check(worst_morphed < SEAM_TOLERANCE_M and worst_normal < 1.0e-5,
			"child fully morphed = parent mesh (worst %s m, parent normals %s rad); unmorphed it differs by up to %.3f m"
			% [String.num_scientific(worst_morphed), String.num_scientific(worst_normal), worst_unmorphed])
	_check(worst_unmorphed > 10.0 * SEAM_TOLERANCE_M, "the child carries detail its parent does not (%.3f m)"
			% worst_unmorphed)
	# The child built from its parent's band heights (the quadtree's path) equals the child that samples the band.
	var from_parent := TerrainNodeBuilder.build(child_key, heights, null, R, Vector4i(-1, -1, -1, -1),
			parent["band_heights"])
	var same := true
	for channel in ["vertices", "normals", "morph", "custom3"]:
		same = same and from_parent[channel] == child[channel]
	_check(same and (parent["band_heights"] as PackedFloat64Array).size() == TerrainNodeBuilder.LATTICE ** 2,
			"child from the parent's band heights = child sampling the parent band (vertices, normals, morph, custom3 equal)")
	# Same-level neighbour (+s): its a = 0 column is the child's a = 64 column, unmorphed and morphed.
	var right_key := Vector4i(child_key.x, child_key.y, child_key.z + 1, child_key.w)
	var right := TerrainNodeBuilder.build(right_key, heights, null, R)
	var worst_edge := 0.0
	for b in ROW:
		for morph in [0.0, 0.5, 1.0]:
			worst_edge = maxf(worst_edge, _vertex_fixed(child, b * ROW + CELLS, morph).distance_to(
					_vertex_fixed(right, b * ROW, morph)))
	_check(worst_edge < SEAM_TOLERANCE_M, "neighbours of a level share their edge at morph 0 / 0.5 / 1 (worst %s m)"
			% String.num_scientific(worst_edge))


# Detail on across a cube face edge (faces 0 / 1 just north of lat 0 at lon 45) and around the +z pole (the four level-L nodes
# meeting at the face centre): neighbours share every vertex of their common edge at morph 0 / 0.5 / 1.
func _check_seams_face_edge_and_pole() -> void:
	var heights := _heights()
	const L := 16
	var west := CubeSphere.key_at(CubeSphere.geodetic_direction(1.0e-4, 45.0 - 1.0e-6), L)
	var east := CubeSphere.key_at(CubeSphere.geodetic_direction(1.0e-4, 45.0 + 1.0e-6), L)
	var edge := _shared_edge(TerrainNodeBuilder.build(west, heights, null, R), TerrainNodeBuilder.build(east, heights, null, R))
	_check(west.x != east.x and edge[0] == ROW and edge[1] < SEAM_TOLERANCE_M,
			"cube face edge %s / %s: %d shared vertices (want %d), worst gap %s m at morph 0 / 0.5 / 1"
			% [west, east, edge[0], ROW, String.num_scientific(edge[1])])
	var pole_key := CubeSphere.key_at(Vector3(0.0, 0.0, 1.0), L)
	var c := 1 << (L - 1)
	var around: Array[Dictionary] = []
	for k: Vector2i in [Vector2i(c - 1, c - 1), Vector2i(c, c - 1), Vector2i(c, c), Vector2i(c - 1, c)]:
		around.append(TerrainNodeBuilder.build(Vector4i(pole_key.x, L, k.x, k.y), heights, null, R))
	var counts: Array[int] = []
	var worst := 0.0
	for n in 4:
		var pair := _shared_edge(around[n], around[(n + 1) % 4])
		counts.append(pair[0])
		worst = maxf(worst, pair[1])
	_check(counts == [ROW, ROW, ROW, ROW] and worst < SEAM_TOLERANCE_M,
			"pole: the four level-%d nodes around it share their edges (%s vertices each), worst gap %s m at morph 0 / 0.5 / 1"
			% [L, counts, String.num_scientific(worst)])


## [shared border vertices of two built nodes (matched by position at morph 0 within SEAM_TOLERANCE_M), worst distance
## of those pairs at morph 0 / 0.5 / 1].
func _shared_edge(a: Dictionary, b: Dictionary) -> Array:
	var border: Array[int] = []
	for e in TerrainNodeBuilder.SKIRT_EDGE:
		border.append(e)
	var b_points: Array[Vector3] = []
	for e in border:
		b_points.append(_vertex_fixed(b, e))
	var shared := 0
	var worst := 0.0
	for ea in border:
		var pa := _vertex_fixed(a, ea)
		var best := -1
		var best_d := INF
		for i in border.size():
			var d := pa.distance_to(b_points[i])
			if d < best_d:
				best_d = d
				best = i
		if best_d >= SEAM_TOLERANCE_M:
			continue
		shared += 1
		for morph in [0.0, 0.5, 1.0]:
			worst = maxf(worst, _vertex_fixed(a, ea, morph).distance_to(_vertex_fixed(b, border[best], morph)))
	return [shared, worst]


# A synchronous tree converged 2 m above the site with the detail on: the leaf under the camera is level 22; every
# uploaded node either owns its horizon map (its parent splits on LOLA alone) or draws with an ancestor's that owns one
# and whose children do not bake; horizon bakes are counted; a deep node's grid maps onto the owner's grid.
func _check_tree_and_horizons() -> void:
	var tree := _make_tree(_heights())
	var ground := _ground_m(true)
	var converge_ms := _converge(tree, _heights().position_fixed(_site.x, _site.y, ground + 2.0))
	var leaf := _leaf_at(tree, _site.x, _site.y)
	_check(leaf.y == TerrainQuadtree.MAX_LEVEL, "detail on, 2 m above the site: leaf level %d (spacing %.4f m), %d nodes, %d drawn, %.1f s"
			% [leaf.y, TerrainNodeBuilder.level_spacing_m(leaf.y, R), tree.uploaded_keys().size(),
			tree.drawn_keys().size(), converge_ms / 1000.0])
	var owners_ok := true
	var owned := 0
	var borrowed := 0
	var owner_levels := {}
	var deep_key := Vector4i(-1, -1, -1, -1)
	for key in tree.uploaded_keys():
		var info := tree.node_info(key)
		var owner: Vector4i = info["horizon_owner"]
		if owner == key:
			owned += 1
			owners_ok = owners_ok and info["horizon_map"] != null
			if key.y > TerrainQuadtree.ROOT_LEVEL:
				owners_ok = owners_ok and tree.node_info(CubeSphere.parent(key))["children_bake"]
		else:
			borrowed += 1
			var ancestor := key
			while ancestor.y > owner.y:
				ancestor = CubeSphere.parent(ancestor)
			var owner_info := tree.node_info(owner)
			owners_ok = owners_ok and ancestor == owner and not owner_info.is_empty() \
					and owner_info["horizon_owner"] == owner and not owner_info["children_bake"] \
					and info["horizon_map"] == owner_info["horizon_map"] and info["horizon_map"] != null
			owner_levels[owner.y] = true
			if key.y > deep_key.y:
				deep_key = key
	_check(owners_ok and borrowed > 0 and tree.stats["horizon_bakes"] == owned,
			"horizon maps: %d nodes bake (stats %d), %d draw with an ancestor's (owner levels %s)"
			% [owned, tree.stats["horizon_bakes"], borrowed, owner_levels.keys()])
	_check(not owner_levels.has(leaf.y) and owner_levels.size() > 0 and owner_levels.keys().max() <= 13,
			"owners stop at the LOLA level (deepest owner level %s; the 20 m tile ends at level 12)"
			% owner_levels.keys().max())
	# The deepest node's grid in its owner's grid: the same face coordinates at every corner and the centre.
	var info := tree.node_info(deep_key)
	var owner_key: Vector4i = info["horizon_owner"]
	var material: ShaderMaterial = info["material"]
	var scale: float = material.get_shader_parameter("horizon_grid_scale")
	var offset: Vector2 = material.get_shader_parameter("horizon_grid_offset")
	var worst := 0.0
	for g: Vector2 in [Vector2(0, 0), Vector2(CELLS, 0), Vector2(0, CELLS), Vector2(CELLS, CELLS), Vector2(32, 32)]:
		var o := g * scale + offset
		var s_node := -1.0 + 2.0 * (deep_key.z * CELLS + g.x) / float(CELLS << deep_key.y)
		var s_owner := -1.0 + 2.0 * (owner_key.z * CELLS + o.x) / float(CELLS << owner_key.y)
		var t_node := -1.0 + 2.0 * (deep_key.w * CELLS + g.y) / float(CELLS << deep_key.y)
		var t_owner := -1.0 + 2.0 * (owner_key.w * CELLS + o.y) / float(CELLS << owner_key.y)
		worst = maxf(worst, maxf(absf(s_node - s_owner), absf(t_node - t_owner)))
	_check(worst < 1.0e-12 and scale < 1.0, "level %d node addresses its level-%d owner's grid (scale %s, offset %s, "
			% [deep_key.y, owner_key.y, scale, offset] + "face coordinate error %s)" % String.num_scientific(worst))
	tree.free()


# Detail off: the tree stops at the LOLA level (20 m tile: level 12, 10.4 m) 2 m above the site, every node owns its
# horizon map, no node is band-limited (even vertices do not morph).
func _check_detail_off() -> void:
	var tree := _make_tree(_heights())
	_converge(tree, _heights().position_fixed(_site.x, _site.y, _ground_m(false) + 2.0))
	var leaf := _leaf_at(tree, _site.x, _site.y)
	var all_owned := true
	for key in tree.uploaded_keys():
		all_owned = all_owned and tree.node_info(key)["horizon_owner"] == key
	var node := TerrainNodeBuilder.build(_site_key(12), _heights(), null, R)
	var even_still := true
	var morph: PackedFloat32Array = node["morph"]
	for b in range(0, ROW, 2):
		for a in range(0, ROW, 2):
			var v := b * ROW + a
			even_still = even_still and morph[4 * v] == 0.0 and morph[4 * v + 1] == 0.0 and morph[4 * v + 2] == 0.0
	_check(leaf.y == 12 and all_owned and even_still and tree.stats["horizon_bakes"] == tree.stats["builds"],
			"detail off: leaf level %d, every node bakes its own horizon (%d of %d builds), even vertices do not morph"
			% [leaf.y, tree.stats["horizon_bakes"], tree.stats["builds"]])
	tree.free()


func _make_tree(heights: HeightSource) -> TerrainQuadtree:
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain_moon.gdshader")
	var tree := TerrainQuadtree.new()
	tree.synchronous = true
	root.add_child(tree)
	tree.setup(heights, null, R, material)
	return tree


## Updates until nothing is loading; returns the time taken (ms).
func _converge(tree: TerrainQuadtree, camera_fixed_m: Vector3) -> int:
	var start := Time.get_ticks_msec()
	for frame in MAX_FRAMES:
		tree.update_for_camera(camera_fixed_m)
		if tree.stats["nodes_loading"] == 0 and tree.stats["uploads_last_frame"] == 0:
			return Time.get_ticks_msec() - start
	_check(false, "quadtree converged within %d frames" % MAX_FRAMES)
	return Time.get_ticks_msec() - start


func _leaf_at(tree: TerrainQuadtree, lat: float, lon: float) -> Vector4i:
	var dir := CubeSphere.geodetic_direction(lat, lon)
	for key in tree.drawn_keys():
		if CubeSphere.key_at(dir, key.y) == key:
			return key
	return Vector4i(-1, -1, -1, -1)


## The site's surface height: the full detail (rocks left out) or LOLA.
func _ground_m(detail: bool) -> float:
	var lat := PackedFloat64Array([_site.x])
	var lon := PackedFloat64Array([_site.y])
	var samples: Dictionary = _core.surface_samples_detail("moon", lat, lon, 0.0, INF) if detail \
			else _core.surface_samples("moon", lat, lon)
	return (samples["height_m"] as PackedFloat64Array)[0]


## Selenographic (lat, lon) in deg of a polar stereographic tile pixel (line, sample) (terrain_quadtree_test.gd).
static func _tile_point(tile: Dictionary, line: float, sample: float) -> Vector2:
	var scale := float(tile["scale_m"])
	var x := (sample + float(tile["first_sample"]) - float(tile["pole_sample"])) * scale
	var y := (line + float(tile["first_line"]) - float(tile["pole_line"])) * scale
	var rho := sqrt(x * x + y * y)
	var lat := 2.0 * atan(rho / (2.0 * float(tile["sphere_radius_m"]))) - PI / 2.0
	return Vector2(rad_to_deg(lat), rad_to_deg(atan2(x, -y)))


func _measure() -> void:
	print("  measure  synchronous tree above the site (one thread; build = summed TerrainNodeBuilder time)")
	for detail in [false, true]:
		_core.set_moon_surface_detail(detail)
		for h: float in CLIMB_HEIGHTS_M:
			var tree := _make_tree(_heights())
			var ms := _converge(tree, _heights().position_fixed(_site.x, _site.y, _ground_m(detail) + h))
			var levels := {}
			for key in tree.drawn_keys():
				levels[key.y] = int(levels.get(key.y, 0)) + 1
			var builds: int = tree.stats["builds"]
			print("  measure  detail %s, %5.0f m: %4d nodes uploaded, %4d drawn, leaf level %2d, %4d horizon bakes, "
					% ["on " if detail else "off", h, tree.uploaded_keys().size(), tree.drawn_keys().size(),
					_leaf_at(tree, _site.x, _site.y).y, tree.stats["horizon_bakes"]]
					+ "build %.1f s (%.1f ms/node), converge %.1f s" % [tree.stats["build_ms"] / 1000.0,
					tree.stats["build_ms"] / maxf(builds, 1), ms / 1000.0])
			tree.free()
	_core.set_moon_surface_detail(true)
	print("  measure  surface_samples_detail, 67 x 67 lattice (band limit = level spacing, rocks off)")
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	var site: Dictionary = TomlLite.load_file(config_dir.path_join("lunar_sites.toml"))["site"][0]
	var tile: Dictionary = TomlLite.load_file(config_dir.path_join("../data/terrain").simplify_path()
			.path_join(str(site["id"]) + ".toml"))["grid"]
	var tile_edge := _tile_point(tile, 0.5 * (int(tile["lines"]) - 1), int(tile["samples"]) - 1 - 500.0
			/ float(tile["scale_m"]))
	var places := []
	for level in [6, 9, 12, 15, 18, 20, 22]:
		places.append(["site", _site_key(level)])
	for level in [12, 18, 22]:
		places.append(["tile blend band (500 m inside the tile edge)",
				CubeSphere.key_at(CubeSphere.geodetic_direction(tile_edge.x, tile_edge.y), level)])
		places.append(["cube face edge (0, 45)", CubeSphere.key_at(CubeSphere.geodetic_direction(0.0, 45.0), level)])
	for place: Array in places:
		var key: Vector4i = place[1]
		var level := key.y
		var lats := PackedFloat64Array()
		var lons := PackedFloat64Array()
		for b in TerrainNodeBuilder.LATTICE:
			for a in TerrainNodeBuilder.LATTICE:
				var d := CubeSphere.direction(key.x, CubeSphere.lattice_coordinate(key.z, level, a - 1, CELLS),
						CubeSphere.lattice_coordinate(key.w, level, b - 1, CELLS))
				var ll := CubeSphere.lat_lon_deg(d)
				lats.append(ll.x)
				lons.append(ll.y)
		var spacing := TerrainNodeBuilder.level_spacing_m(level, R)
		var t0 := Time.get_ticks_usec()
		_core.surface_samples_detail("moon", lats, lons, spacing, INF)
		var t1 := Time.get_ticks_usec()
		_core.surface_samples_detail("moon", lats, lons, spacing, INF)
		var t2 := Time.get_ticks_usec()
		_core.surface_samples("moon", lats, lons)
		var t3 := Time.get_ticks_usec()
		print("  measure  %s level %2d (%.4f m): detail cold %.1f ms, warm %.1f ms (%.1f us/point); LOLA %.2f ms"
				% [place[0], level, spacing, (t1 - t0) / 1000.0, (t2 - t1) / 1000.0, (t2 - t1) / float(lats.size()),
				(t3 - t2) / 1000.0])
	var heights := _heights()
	for place: Array in places:
		var key: Vector4i = place[1]
		var node := TerrainNodeBuilder.build(key, heights, null, R, CubeSphere.parent(key))
		var t0 := Time.get_ticks_usec()
		TerrainNodeBuilder.build(key, heights, null, R, CubeSphere.parent(key))
		print("  measure  node build (no horizon bake) %s level %d: %.1f ms" % [place[0], key.y,
				(Time.get_ticks_usec() - t0) / 1000.0])
		node.clear()
	print("  measure  moon_rocks on rock-rich cells (Diviner maxima, Tycho), 3 x 3 cells per point and level, D >= 0.01 m")
	for level in range(16, 26):
		var worst_us := 0
		var most := 0
		var total_us := 0
		var cells := 0
		var rocks := 0
		for point: Vector2 in ROCKY_POINTS:
			var key := CubeSphere.key_at(CubeSphere.geodetic_direction(point.x, point.y), level)
			for dj in range(-1, 2):
				for di in range(-1, 2):
					var t0 := Time.get_ticks_usec()
					var found: Dictionary = _core.moon_rocks(key.x, level, key.z + di, key.w + dj, 0.01)
					var us := Time.get_ticks_usec() - t0
					var n := (found["ids"] as PackedInt64Array).size()
					worst_us = maxi(worst_us, us)
					most = maxi(most, n)
					total_us += us
					rocks += n
					cells += 1
		print("  measure  rocks level %d: %d cells, %d rocks (most %d in a cell), %.1f us/cell mean, worst %d us"
				% [level, cells, rocks, most, total_us / float(cells), worst_us])


## Mean of Atmosphere.regolith_reflectance over the projected disc of a uniform sphere at phase angle g (rad): eye on
## +Z, light in the XZ plane; midpoint rule in photometric latitude / longitude over the lit, visible lune (weight
## mu cos(latitude), the disc's area pi).
static func _disc_mean(g: float) -> float:
	const N := 64
	var to_eye := Vector3(0, 0, 1)
	var to_light := Vector3(sin(g), 0, cos(g))
	var lambda_lo := g - PI / 2.0
	var d_lambda := (PI / 2.0 - lambda_lo) / N
	var d_beta := PI / N
	var total := 0.0
	for a in N:
		var lam := lambda_lo + (a + 0.5) * d_lambda
		for b in N:
			var beta := -PI / 2.0 + (b + 0.5) * d_beta
			var n := Vector3(cos(beta) * sin(lam), sin(beta), cos(beta) * cos(lam))
			var mu := n.z
			if mu <= 0.0 or n.dot(to_light) <= 0.0:
				continue
			total += Atmosphere.regolith_reflectance(n, to_light, to_eye) * mu * cos(beta) * d_lambda * d_beta
	return total / PI


## Normalisation identity (not an independent physics check; Ruling T9 I1): Hapke x hapke_normalisation (the eye
## adaptation's mirror, through the core) equals the former Lommel-Seeliger disc at phase 0, the point the gain is
## defined at. Other phases printed (0.96-0.995 at 2-5 deg: accepted, it moves toward ROLO).
func _check_hapke_full_moon() -> void:
	Atmosphere.configure_moon_brdf(_core, _config_dir, "hapke")
	_check(Atmosphere.moon_brdf_hapke, "Hapke parameters loaded (config/moon_surface.toml) and the BRDF set to Hapke")
	if not Atmosphere.moon_brdf_hapke:
		return
	for g_deg: float in [0.0, 2.0, 5.0, 10.0, 30.0, 60.0, 90.0]:
		var g := deg_to_rad(g_deg)
		Atmosphere.moon_brdf_hapke = true
		var hapke := _disc_mean(g)
		Atmosphere.moon_brdf_hapke = false
		var lommel := _disc_mean(g)
		Atmosphere.moon_brdf_hapke = true
		var line := "disc at phase %.0f deg: Hapke %.4f, Lommel-Seeliger %.4f (ratio %.4f)" \
				% [g_deg, hapke, lommel, hapke / lommel]
		if g_deg == 0.0:
			_check(absf(hapke / lommel - 1.0) < FULL_MOON_TOLERANCE,
					"normalisation identity (Ruling T9 I1, holds by construction): " + line + " within 3 %")
		else:
			print("  info  " + line)


## The shader parity: one SubViewport (HDR: linear half floats) with a flat patch (normal +Y) and an orthographic camera.
func _start_parity() -> void:
	_hapke = _core.load_moon_hapke(_config_dir)
	_parity_viewport = SubViewport.new()
	_parity_viewport.size = Vector2i(32, 32)
	_parity_viewport.use_hdr_2d = true
	_parity_viewport.own_world_3d = true
	_parity_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	var world := World3D.new()
	var environment := Environment.new()
	environment.tonemap_mode = Environment.TONE_MAPPER_LINEAR
	environment.background_mode = Environment.BG_COLOR
	environment.background_color = Color.BLACK
	world.environment = environment
	_parity_viewport.world_3d = world
	var patch := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(4, 4)
	patch.mesh = plane
	_parity_material = ShaderMaterial.new()
	_parity_material.shader = load("res://tests/hapke_patch.gdshader")
	_parity_material.set_shader_parameter("albedo_phase", Vector4(float(_hapke["w"]), float(_hapke["b"]),
			float(_hapke["c"]), float(_hapke["porosity_k"])))
	_parity_material.set_shader_parameter("opposition", Vector4(float(_hapke["bs0"]), float(_hapke["hs"]),
			float(_hapke["bc0"]), float(_hapke["hc"])))
	_parity_material.set_shader_parameter("theta_bar", float(_hapke["theta_bar_rad"]))
	_parity_material.set_shader_parameter("gain", PARITY_GAIN)
	patch.material_override = _parity_material
	_parity_patch = patch
	_parity_production = ShaderMaterial.new()
	_parity_production.shader = load("res://tests/hapke_patch_production.gdshader")
	_parity_production.set_shader_parameter("gain", PARITY_GAIN)
	_parity_steps = [{"kind": "calibrate", "geometry": PARITY_GEOMETRIES[0], "theta": -1.0}]
	for geometry: Vector3 in PARITY_GEOMETRIES:
		_parity_steps.append({"kind": "patch", "geometry": geometry, "theta": -1.0})
	for geometry: Vector3 in PARITY_GEOMETRIES:
		_parity_steps.append({"kind": "production", "geometry": geometry, "theta": -1.0})
	_parity_steps.append({"kind": "production", "geometry": PARITY_GEOMETRIES[1], "theta": PARITY_THETA_OVERRIDE_RAD})
	if bool(_detail_stats.get("present", false)):
		_detail_material = ShaderMaterial.new()
		_detail_material.shader = load("res://tests/moon_detail_patch.gdshader")
		_detail_tiles = MoonDetailShading.configure(_core, _detail_material, true).get("tiles", [])
		_detail_material.set_shader_parameter("gain", PARITY_GAIN)
		for entry: Array in DETAIL_STEPS:
			_parity_steps.append({"kind": "detail", "geometry": entry[0], "theta": -1.0, "size": entry[1],
					"spacing": entry[2], "horizon": entry[3], "abundance": entry[4], "textures": entry[5],
					"sun": entry[6]})
	if _rock_layer != null and _detail_material != null:
		_start_rock_patch()
		for entry: Array in ROCK_STEPS:
			_parity_steps.append({"kind": "rocks", "geometry": entry[0], "theta": -1.0, "size": ROCK_PATCH_VIEW_M,
					"spacing": ROCK_PATCH_SPACING_M, "horizon": ROCK_PATCH_HORIZON_M, "abundance": _rock_patch_abundance,
					"textures": false,
					"sun": entry[2], "threshold": entry[1]})
	_parity_viewport.add_child(patch)
	_parity_camera = Camera3D.new()
	_parity_camera.projection = Camera3D.PROJECTION_ORTHOGONAL
	_parity_camera.size = 1.0
	_parity_viewport.add_child(_parity_camera)
	root.add_child(_parity_viewport)
	_parity_step = 0
	_set_parity_geometry(0)


## Step 0: calibration (constant output); then the include-only patch, then the production wiring (_parity_steps).
func _set_parity_geometry(step: int) -> void:
	var entry: Dictionary = _parity_steps[step]
	var geometry: Vector3 = entry["geometry"]
	var production: bool = entry["kind"] == "production"
	var rocks: bool = entry["kind"] == "rocks"
	var detail: bool = entry["kind"] == "detail" or rocks
	var material := _parity_production if production else _parity_material
	if detail:
		material = _detail_material
		var edge: float = entry["size"]
		(_parity_patch.mesh as PlaneMesh).size = Vector2(edge, edge)
		_parity_camera.size = edge / 2.0
		_parity_viewport.size = Vector2i(DETAIL_VIEWPORT, DETAIL_VIEWPORT)
		material.set_shader_parameter("detail_cell_m", edge / 64.0)
		material.set_shader_parameter("detail_origin", [Vector4.ZERO, Vector4.ZERO, Vector4.ZERO, Vector4.ZERO])
		material.set_shader_parameter("patch_spacing_m", float(entry["spacing"]))
		material.set_shader_parameter("detail_horizon_spacing_m", float(entry["horizon"]))
		material.set_shader_parameter("patch_abundance", float(entry["abundance"]))
		material.set_shader_parameter("detail_textures", bool(entry["textures"]) and not _detail_tiles.is_empty())
		material.set_shader_parameter("sun_on", 1.0 if entry["sun"] else 0.0)
		material.set_shader_parameter("earth_on", 0.0 if entry["sun"] else 1.0)
		material.set_shader_parameter("rock_geometry_override_m", float(entry["threshold"]) if rocks else -1.0)
		if _rock_patch != null:
			_rock_patch.visible = rocks
			_rock_patch.material_override.set_shader_parameter("rock_geometry_override_m",
					float(entry["threshold"]) if rocks else -1.0)
			ShaderGlobals.set_value(&"moon_sun_scene", 1.0 if entry["sun"] else 0.0)
			ShaderGlobals.set_value(&"moon_earthshine", 0.0 if entry["sun"] else 1.0)
	_parity_patch.material_override = material
	if production:
		material.set_shader_parameter("hapke_theta_bar_rad", float(entry["theta"]))
	var i := deg_to_rad(geometry.x)
	var e := deg_to_rad(geometry.y)
	var g := deg_to_rad(geometry.z)
	var cos_psi := 1.0
	if sin(i) * sin(e) > 1.0e-9:
		cos_psi = clampf((cos(g) - cos(i) * cos(e)) / (sin(i) * sin(e)), -1.0, 1.0)
	var psi := acos(cos_psi)
	var to_eye := Vector3(sin(e), cos(e), 0.0)
	var to_light := Vector3(sin(i) * cos(psi), cos(i), sin(i) * sin(psi))
	material.set_shader_parameter("light_world", to_light)
	if detail:
		# The earthshine-only patch: the Earth where the Sun was, the Sun 5 deg below the horizon.
		material.set_shader_parameter("earth_world", to_light)
		if not entry["sun"]:
			material.set_shader_parameter("light_world", Vector3(cos(deg_to_rad(5.0)), -sin(deg_to_rad(5.0)), 0.0))
		if rocks:  # the rock shader's lights: far along the patch's directions (the Earth clear of the Sun)
			var sun_dir: Vector3 = material.get_shader_parameter("light_world")
			var earth_dir: Vector3 = to_light if not entry["sun"] else -to_light
			ShaderGlobals.set_value(&"moon_sun_position_local", sun_dir.normalized() * Atmosphere.ASTRONOMICAL_UNIT_M)
			ShaderGlobals.set_value(&"moon_earth_position_local", earth_dir.normalized() * 3.844e8)  # [S] mean Earth-Moon distance (m)
	_parity_material.set_shader_parameter("calibrate", step == 0)
	# VIEW in a fragment is the direction to the camera point even with an orthographic camera: far away for the detail
	# patches (12.5 m views), so the emission angle varies < 0.2 deg over the image.
	var distance := 200.0 * _parity_camera.size if detail else 10.0
	_parity_camera.far = 2.0 * distance + 100.0
	_parity_camera.look_at_from_position(to_eye * distance, Vector3.ZERO, Vector3(0, 0, 1))  # before it enters the tree
	_parity_wait = PARITY_FRAMES


## Mean red of the viewport's central 8 x 8 pixels (linear; sRGB-decoded when the calibration found it encoded); the
## detail patches: the whole viewport.
func _parity_read() -> float:
	var image := _parity_viewport.get_texture().get_image()
	var total := 0.0
	if _parity_viewport.size.x == DETAIL_VIEWPORT:
		for y in DETAIL_VIEWPORT:
			for x in DETAIL_VIEWPORT:
				var c := image.get_pixel(x, y)
				total += c.srgb_to_linear().r if _parity_srgb else c.r
		return total / float(DETAIL_VIEWPORT * DETAIL_VIEWPORT)
	for y in range(12, 20):
		for x in range(12, 20):
			var c := image.get_pixel(x, y)
			total += c.srgb_to_linear().r if _parity_srgb else c.r
	return total / 64.0


func _process(_delta: float) -> bool:
	if _parity_step < 0:
		return false
	if _parity_wait > 0:
		_parity_wait -= 1
		return false
	var value := _parity_read()
	if _parity_step == 0:
		var expected := PARITY_GAIN * PARITY_CALIBRATION
		if absf(value / expected - 1.0) > 0.005:
			var image := _parity_viewport.get_texture().get_image()
			var decoded := image.get_pixel(16, 16).srgb_to_linear().r
			if absf(decoded / expected - 1.0) < 0.01:
				_parity_srgb = true
				value = decoded
		_check(absf(value / expected - 1.0) < 0.01, "parity readback calibration: %.5f for %.5f%s (format %d)"
				% [value, expected, " (sRGB-decoded)" if _parity_srgb else "",
				_parity_viewport.get_texture().get_image().get_format()])
	elif _parity_steps[_parity_step]["kind"] == "detail" or _parity_steps[_parity_step]["kind"] == "rocks":
		_check_detail_patch(_parity_steps[_parity_step], value / PARITY_GAIN)
	else:
		var entry: Dictionary = _parity_steps[_parity_step]
		var geometry: Vector3 = entry["geometry"]
		var production: bool = entry["kind"] == "production"
		var theta := float(_hapke["theta_bar_rad"]) if float(entry["theta"]) < 0.0 else float(entry["theta"])
		var core := _core.moon_hapke_reflectance(cos(deg_to_rad(geometry.x)), cos(deg_to_rad(geometry.y)),
				deg_to_rad(geometry.z), theta)
		if production:  # regolith_reflectance_rough = moon_hapke_norm x Hapke I/F
			core *= Atmosphere.hapke_normalisation(float(_hapke["disc_reflectance_full"]))
		var shader := value / PARITY_GAIN
		_check(absf(shader / core - 1.0) < PARITY_TOLERANCE,
				"Hapke %s parity (i %.0f, e %.0f, g %.0f deg, theta_bar %.3f rad): shader %.5f core %.5f (%+.2f %%)"
				% ["production-wiring" if production else "include", geometry.x, geometry.y, geometry.z, theta, shader,
				core, 100.0 * (shader / core - 1.0)])
	_parity_step += 1
	if _parity_step >= _parity_steps.size():
		_parity_step = -1
		_finish()
		return false
	_set_parity_geometry(_parity_step)
	return false


## A detail patch against the core: moon_hapke_norm x I/F(mu0, mu, g, residual theta_bar) x the light factor (sun or
## earthshine), the textures' modulation being mean-preserving.
func _check_detail_patch(entry: Dictionary, shader: float) -> void:
	var geometry: Vector3 = entry["geometry"]
	var spacing: float = entry["spacing"]
	var theta := _core.moon_residual_theta_bar(spacing)
	var norm := Atmosphere.hapke_normalisation(float(_hapke["disc_reflectance_full"]))
	# The active light (Sun, or the Earth in the earthshine-only cases) at elevation 90 - i; each factor depends only on
	# its own light's elevation.
	var factors := _core.moon_unresolved_light_factors(float(entry["abundance"]), INF, spacing, float(entry["horizon"]),
			deg_to_rad(90.0 - geometry.x), deg_to_rad(90.0 - geometry.x))
	var core := norm * _core.moon_hapke_reflectance(cos(deg_to_rad(geometry.x)), cos(deg_to_rad(geometry.y)),
			deg_to_rad(geometry.z), theta)
	var what := ""
	if entry["kind"] == "rocks":
		what = "with drawn rocks from %.2f m (growth to %.2f m; realised cover >= %.2f m %.4f, law %.4f): " % [
				float(entry["threshold"]), 2.0 * float(entry["threshold"]), ROCK_PATCH_MIN_D_M, _rock_cover,
				_law_cover(ROCK_PATCH_MIN_D_M)]
	var factor := factors.x if entry["sun"] else factors.y
	if entry["sun"]:
		what += "Sun %.0f deg" % (90.0 - geometry.x)
	else:
		what += "earthshine only (Sun 5 deg below), Earth %.0f deg" % (90.0 - geometry.x)
	if entry["kind"] == "rocks":
		# Review I2: the law's mean against the patch's REALISED drawn cover (the viewed square's rocks above the
		# threshold, area-weighted by the growth share x), so the realisation's cover excess and the model are separated.
		# Drawn rocks' mean radiance from the core's closure (all = e^-Cg ground + (1 - e^-Cg) rock with the law's Cg).
		var d := float(entry["threshold"])
		var elevation := deg_to_rad(90.0 - geometry.x)
		var ground_v := _core.moon_unresolved_light_factors(float(entry["abundance"]), d, spacing, float(entry["horizon"]),
				elevation, elevation)
		var ground := ground_v.x if entry["sun"] else ground_v.y
		var c_law := float(entry["abundance"]) * _law_drawn_cover(d)  # the law at the patch's abundance
		var rock := (factor - exp(-c_law) * ground) / (1.0 - exp(-c_law))
		var c_real := 0.0
		for diameter in _rock_diameters:
			c_real += 0.25 * PI * diameter * diameter * clampf(log(diameter / d) / log(2.0), 0.0, 1.0)
		c_real /= _rock_view_edge_m * _rock_view_edge_m
		var realised := exp(-c_real) * ground + (1.0 - exp(-c_real)) * rock
		print("  info  rock patch d %.2f m, %s: drawn cover realised %.5f, law %.5f; shader against the law's mean %+.2f %%, against the realised drawn cover %+.2f %%"
				% [d, "Sun" if entry["sun"] else "earthshine", c_real, c_law,
				100.0 * (shader / (core * factor) - 1.0), 100.0 * (shader / (core * realised) - 1.0)])
		what += " [against the realised drawn cover %.5f (law %.5f)]" % [c_real, c_law]
		factor = realised
	core *= factor
	_check(absf(shader / core - 1.0) < PARITY_TOLERANCE,
			"detail patch %s, e %.0f g %.0f, %.0f m, spacing %.4f m, horizon %.0f m, abundance %.0f, textures %s: shader %.5f "
			% [what, geometry.y, geometry.z, entry["size"], spacing, entry["horizon"], entry["abundance"],
			"on" if entry["textures"] else "off", shader]
			+ "core %.5f (%+.2f %%; theta_bar %.2f deg, factor %.3f)" % [core, 100.0 * (shader / core - 1.0),
			rad_to_deg(theta), factors.x if entry["sun"] else factors.y])


## The rock patch: the core's rocks around the site (D >= ROCK_PATCH_MIN_D_M, ROCK_PATCH_EDGE_M square) as one LOD-1
## MultiMesh through the production rock shader, on the flat patch's frame (the Moon-local frame = the patch's world:
## moon_world_to_local identity; Sun / Earth far along the patch's light directions, set per step).
func _start_rock_patch() -> void:
	_core.set_moon_surface_detail(true)  # the full run switched it off for the detail-off checks; the rocks need it
	var dir := CubeSphere.geodetic_direction(_site.x, _site.y)
	# --rock_patch_abundance=A (review R1: the drawn / statistical balance at abundance 3, 10): the patch moves south
	# from the boulder field ROCKY_POINTS[0] to where the abundance map has fallen to A (bisection on the meridian).
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--rock_patch_abundance="):
			var wanted := float(arg.get_slice("=", 1))
			var rocky: Vector2 = ROCKY_POINTS[0]
			var near := 0.0
			var far := 1.0  # deg south of the field
			for k in 60:
				var mid := 0.5 * (near + far)
				if _rock_layer.abundance_at(CubeSphere.geodetic_direction(rocky.x - mid, rocky.y)) > wanted:
					near = mid
				else:
					far = mid
			dir = CubeSphere.geodetic_direction(rocky.x - near, rocky.y)
			_rock_patch_abundance = _rock_layer.abundance_at(dir)
			print("  info  rock patch at %.5f N, %.5f E: abundance %.2f (asked %.2f)" % [rocky.x - near, rocky.y,
					_rock_patch_abundance, wanted])
	var data :=MoonRockChecks.patch_rocks(_core, _rock_layer.top_z, dir, ROCK_PATCH_EDGE_M, ROCK_PATCH_MIN_D_M)
	var viewed := MoonRockChecks.patch_rocks(_core, _rock_layer.top_z, dir, 0.5 * ROCK_PATCH_VIEW_M, ROCK_PATCH_MIN_D_M)
	_rock_cover = float(viewed["cover"])
	_rock_diameters = viewed["diameters"]
	_rock_view_edge_m = 0.5 * ROCK_PATCH_VIEW_M
	_core.set_moon_surface_detail(false)
	var multimesh := MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D
	multimesh.use_colors = true
	multimesh.use_custom_data = true
	multimesh.mesh = _rock_layer.meshes[1]
	multimesh.instance_count = int(data["count"])
	multimesh.buffer = data["buffer"]
	_rock_material = _rock_layer.material_template.duplicate()
	for param in MoonRockLayer.COPIED_TEMPLATE_PARAMETERS:
		if param != &"albedo_texture" and param != &"sun_radius_m" and param != &"earth_radius_m":
			_rock_material.set_shader_parameter(param, _detail_material.get_shader_parameter(param))
	var white := Image.create(1, 1, false, Image.FORMAT_RGBF)
	white.set_pixel(0, 0, Color(1, 1, 1))
	_rock_material.set_shader_parameter("albedo_texture", ImageTexture.create_from_image(white))
	var abundance := Image.create(1, 1, false, Image.FORMAT_RF)
	abundance.set_pixel(0, 0, Color(_rock_patch_abundance, 0, 0))
	_rock_material.set_shader_parameter("detail_rock_abundance", ImageTexture.create_from_image(abundance))
	_rock_material.set_shader_parameter("sun_radius_m", World.SUN_RADIUS_M)
	_rock_material.set_shader_parameter("earth_radius_m", World.EARTH_MEAN_RADIUS_M)
	_rock_material.set_shader_parameter("detail_cell_m", ROCK_PATCH_SPACING_M)
	_rock_material.set_shader_parameter("detail_horizon_spacing_m", ROCK_PATCH_HORIZON_M)
	# Diagnostics: --rock_vertex_normals draws the interpolated vertex normals instead of the exact facets.
	_rock_material.set_shader_parameter("rock_facet_normals", not OS.get_cmdline_user_args().has("--rock_vertex_normals"))
	_rock_patch = MultiMeshInstance3D.new()
	_rock_patch.multimesh = multimesh
	_rock_patch.material_override = _rock_material
	_rock_patch.visible = false
	_parity_viewport.add_child(_rock_patch)
	# The rocks get the eye adaptation's exposure gain like the ground (fix C): split PARITY_GAIN into a regolith scale
	# and ROCK_PATCH_EXPOSURE_GAIN != 1, so a rock shader without the gain misses the gate by that factor.
	ShaderGlobals.set_value(&"moon_regolith_scale", PARITY_GAIN / ROCK_PATCH_EXPOSURE_GAIN)
	ShaderGlobals.set_value(&"atmosphere_enabled", true)
	ShaderGlobals.set_value(&"atmosphere_exposure_gain", ROCK_PATCH_EXPOSURE_GAIN)
	ShaderGlobals.set_value(&"moon_world_to_local", Basis.IDENTITY)
	ShaderGlobals.set_value(&"moon_rock_threshold_px", 0.0)
	print("  info  rock patch: %d rocks >= %.2f m over %.0f m (cover in the viewed %.0f m square: %.4f, law %.4f)" % [
			int(data["count"]), ROCK_PATCH_MIN_D_M, ROCK_PATCH_EDGE_M, 0.5 * ROCK_PATCH_VIEW_M, _rock_cover,
			_law_cover(ROCK_PATCH_MIN_D_M)])


## The law's rock cover from d up (abundance 1): the statistics' closed form (coef (D_max^q - d^q)).
## The law's drawn cover at threshold d (abundance 1): every rock from 2 d, the band [d, 2 d] weighted by the growth
## share log2(D / d) (= C_all - the core's rock_statistical_cover), midpoint rule in ln D.
func _law_drawn_cover(d: float) -> float:
	var q := float(_detail_stats["rock_cover_exponent"])
	var coef := float(_detail_stats["rock_cover_coefficient"])
	var band := 0.0
	var steps := 2000
	for k in steps:
		var diameter := d * pow(2.0, (k + 0.5) / steps)
		band += coef * q * pow(diameter, q) * log(2.0) / steps * (k + 0.5) / steps
	return _law_cover(2.0 * d) + band


func _law_cover(d: float) -> float:
	var q := float(_detail_stats["rock_cover_exponent"])
	return float(_detail_stats["rock_cover_coefficient"]) * (pow(float(_detail_stats["rock_max_diameter_m"]), q) - pow(d, q))
