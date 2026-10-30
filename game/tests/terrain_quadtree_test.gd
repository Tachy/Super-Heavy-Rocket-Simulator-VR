# Headless test of the terrain quadtree (TerrainQuadtree): coverage, refinement, throttled loading, merge hysteresis,
# morph ranges (Earth and Moon) and (with data/) the finest mesh against the core's surface on the Earth and the Moon.
# Nodes are built synchronously (no worker threads).
# Run: godot --headless --path game --script res://tests/terrain_quadtree_test.gd
extends SceneTree

const EARTH_RADIUS_M := 6378137.0
const MOON_RADIUS_M := HeightSource.MOON_RADIUS_M
const EYE_HEIGHT_M := 2.0
const MOON_EYE_HEIGHT_M := 10.0  ## the brief's camera height above the lunar surface
const MOON_EDGE_INSET_M := 500.0  ## the tile-edge point lies this far inside the tile (in its 1 km blend band)
const MAX_FRAMES := 4000
const CELLS := TerrainNodeBuilder.CELLS
const ROW := TerrainNodeBuilder.ROW

var _failures := 0
var _sites: Array = []


## Rolling relief (+-300 m, ~20 km wavelength): parent and child meshes differ by tens of metres between their vertices.
class WaveSource extends HeightSource:
	const AMPLITUDE_M := 300.0
	const WAVES_PER_DEG := 5.0

	func sample(lats_deg: PackedFloat64Array, lons_deg: PackedFloat64Array) -> Dictionary:
		var heights := PackedFloat64Array()
		heights.resize(lats_deg.size())
		var water := PackedByteArray()
		water.resize(lats_deg.size())
		for k in lats_deg.size():
			heights[k] = AMPLITUDE_M * sin(TAU * WAVES_PER_DEG * lats_deg[k]) * cos(TAU * WAVES_PER_DEG * lons_deg[k])
		return {"height_m": heights, "water": water}
var _core: StarsimCore = null
var _terrain := false
var _moon_terrain := false
var _config_dir := ""


func _initialize() -> void:
	_config_dir = ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	_sites = TomlLite.load_file(_config_dir.path_join("launch_sites.toml")).get("site", [])
	_core = StarsimCore.new()
	var loaded := _core.load_terrain(_config_dir)
	_terrain = bool(loaded.get("earth", false))
	_moon_terrain = bool(loaded.get("moon", false))
	_check_morph_ranges()
	_check_flat_rule()
	_check_hysteresis()
	_check_morph_continuity()
	_check_morph_fade_in()
	_check_camera_precision()
	_check_jump()
	_check_fresh_checkout()
	_check_moon_without_water()
	_check_no_horizon_without_terrain()
	if _terrain:
		_check_data()
	else:
		print("  note  data/ Earth terrain absent: 'refines towards the camera' (data) and 'finest mesh matches the "
				+ "core' skipped")
	if _moon_terrain:
		_check_moon_data()
		_check_moon_horizon()
	else:
		print("  note  data/ Moon terrain absent: 'moon refines to the 20 m tile' and 'moon finest mesh matches the "
				+ "core' skipped")
	print("TERRAIN QUADTREE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


## A quadtree on a height source (core null: the flat rule), synchronous loading, no imagery. The Moon (heights.body
## "moon") gets its radius and shader.
func _make_tree(with_core: bool, heights := HeightSource.new()) -> TerrainQuadtree:
	heights.sites = _sites
	heights.core = _core if with_core else null
	var moon := heights.body == "moon"
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain_moon.gdshader" if moon else "res://shaders/terrain_earth.gdshader")
	var tree := TerrainQuadtree.new()
	tree.synchronous = true
	root.add_child(tree)
	tree.setup(heights, null, MOON_RADIUS_M if moon else EARTH_RADIUS_M, material)
	return tree


func _moon_heights() -> HeightSource:
	var heights := HeightSource.new()
	heights.body = "moon"
	return heights


func _site(id: String) -> Dictionary:
	for site: Dictionary in _sites:
		if site["id"] == id:
			return site
	return {}


func _above(lat_deg: float, lon_deg: float, height_m: float) -> Vector3:
	return HeightSource.geodetic_to_fixed(lat_deg, lon_deg, height_m)


## Updates until nothing is loading; returns false if coverage broke on any frame.
func _converge(tree: TerrainQuadtree, camera_fixed_m: Vector3) -> bool:
	var covered := true
	for frame in MAX_FRAMES:
		tree.update_for_camera(camera_fixed_m)
		covered = covered and absf(_coverage(tree) - 4.0 * PI) < 1e-9
		if tree.stats["nodes_loading"] == 0 and tree.stats["uploads_last_frame"] == 0:
			return covered
	_check(false, "quadtree converged within %d frames" % MAX_FRAMES)
	return covered


func _coverage(tree: TerrainQuadtree) -> float:
	var total := 0.0
	for key: Vector4i in tree.drawn_keys():
		total += _solid_angle(key)
	return total


## The drawn leaf containing a geodetic point.
func _leaf_at(tree: TerrainQuadtree, lat_deg: float, lon_deg: float) -> Vector4i:
	var dir := CubeSphere.geodetic_direction(lat_deg, lon_deg)
	var drawn := tree.drawn_keys()
	for level in range(TerrainQuadtree.MAX_LEVEL, -1, -1):
		var key := CubeSphere.key_at(dir, level)
		if drawn.has(key):
			return key
	return Vector4i(-1, -1, -1, -1)


# Without terrain: the leaf under the Starbase camera is the finest level the flat rule's curvature limit allows; the
# drawn surface there is at the site height; coverage holds.
func _check_flat_rule() -> void:
	var tree := _make_tree(false)
	var site := _site("starbase_olp1")
	var covered := _converge(tree, _above(site["latitude_deg"], site["longitude_deg"], site["height_m"] + EYE_HEIGHT_M))
	_check(covered and absf(_coverage(tree) - 4.0 * PI) < 1e-9, "coverage is complete (flat rule, %d leaves)"
			% tree.drawn_keys().size())
	var leaf := _leaf_at(tree, site["latitude_deg"], site["longitude_deg"])
	var spacing := CubeSphere.node_size_m(leaf, EARTH_RADIUS_M) / CELLS
	var parent_spacing := CubeSphere.node_size_m(CubeSphere.parent(leaf), EARTH_RADIUS_M) / CELLS
	_check(spacing <= tree.curvature_spacing_m and parent_spacing > tree.curvature_spacing_m,
			"refines towards the camera: flat rule leaf level %d (spacing %.0f m, curvature limit %.0f m)"
			% [leaf.y, spacing, tree.curvature_spacing_m])
	var error := _surface_error_m(tree, leaf, site["latitude_deg"], site["longitude_deg"], site["height_m"])
	_check(error < 0.05, "quadtree without terrain uses the flat rule: surface at the site height (error %.4f m)" % error)
	tree.queue_free()


# A camera oscillating +-5 % around the split distance of a level-4 node: it splits once and stays split.
func _check_hysteresis() -> void:
	var tree := _make_tree(false)
	var key := CubeSphere.key_at(CubeSphere.geodetic_direction(10.0, 20.0), 4)
	var info := tree.node_info(key)
	var tries := 0
	while info.is_empty() and tries < 8:  # the node is built once an ancestor splits: approach it first
		_converge(tree, CubeSphere.node_center(key) * EARTH_RADIUS_M * 1.0001)
		info = tree.node_info(key)
		tries += 1
	if info.is_empty():
		_check(false, "merge hysteresis: node %s never built" % key)
		return
	var center: Vector3 = info["center_fixed"]
	var up := center.normalized()
	var split_distance_m := TerrainQuadtree.LOD_FACTOR * float(info["size_m"])
	var near_camera := center + up * (float(info["radius_m"]) + 0.95 * split_distance_m)
	var far_camera := center + up * (float(info["radius_m"]) + 1.05 * split_distance_m)
	_converge(tree, near_camera)
	var changes := 0
	var was_split: bool = tree.node_info(key)["split"]
	for frame in 40:
		tree.update_for_camera(far_camera if frame % 2 == 0 else near_camera)
		var split: bool = tree.node_info(key)["split"]
		if split != was_split:
			changes += 1
		was_split = split
	_check(was_split and changes == 0, "merge hysteresis: split at 0.95, stays split at 1.05 x split distance "
			+ "(%d changes in 40 frames)" % changes)
	tree.update_for_camera(center + up * (float(info["radius_m"]) + 1.2 * split_distance_m))
	_check(not tree.node_info(key)["split"], "merges beyond 1.15 x split distance")
	tree.queue_free()


# Split and merge frames of a level-6 node on rolling relief, camera above it and off to the side of a corner: the
# children's morphed surface equals the parent's morphed surface at every child vertex (no pop at the swap).
func _check_morph_continuity() -> void:
	var tree := _make_tree(false, WaveSource.new())
	var key := CubeSphere.key_at(CubeSphere.geodetic_direction(10.0, 20.0), 6)
	var tries := 0
	while tree.node_info(key).is_empty() and tries < 10:
		_converge(tree, CubeSphere.node_center(key) * EARTH_RADIUS_M * 1.0001)
		tries += 1
	var info := tree.node_info(key)
	if info.is_empty():
		_check(false, "morph continuity: node %s never built" % key)
		return
	var center: Vector3 = info["center_fixed"]
	var radius: float = info["radius_m"]
	var size: float = info["size_m"]
	var up := center.normalized()
	var corner := CubeSphere.node_corners(key)[0] * EARTH_RADIUS_M
	var side := (up * 0.3 + (center - corner).normalized()).normalized()  # corner 0 is the far corner
	var parent_data := TerrainNodeBuilder.build(key, tree.height_source, null, EARTH_RADIUS_M)
	var children_data := []
	for child in CubeSphere.children(key):
		children_data.append(TerrainNodeBuilder.build(child, tree.height_source, null, EARTH_RADIUS_M))
	for direction: Vector3 in [up, side]:
		var label := "above" if direction == up else "beside the far corner"
		var split_camera := center + direction * (radius + TerrainQuadtree.LOD_FACTOR * size * 0.9999)
		_converge(tree, split_camera)
		var result := _swap_error(tree, key, parent_data, children_data, split_camera)
		_check(tree.node_info(key)["split"] and result.x < 1e-3, "morph continuity at the split, camera %s: error %s m "
				% [label, String.num_scientific(result.x)] + "(parent morph max %.4f, child morph min %.4f)"
				% [result.y, result.z])
		var merge_camera := center + direction * (radius + TerrainQuadtree.MERGE_FACTOR * TerrainQuadtree.LOD_FACTOR
				* size * 0.9999)
		tree.update_for_camera(merge_camera)
		result = _swap_error(tree, key, parent_data, children_data, merge_camera)
		_check(tree.node_info(key)["split"] and result.x < 1e-3, "morph continuity at the merge, camera %s: error %s m "
				% [label, String.num_scientific(result.x)] + "(parent morph max %.4f, child morph min %.4f)"
				% [result.y, result.z])
	tree.queue_free()


# A node drawn for the first time starts fully morphed (morph floor 1: its parent's surface, even when its upload came
# late and the camera is already inside its morph range) and fades to its own surface within MORPH_IN_S; a parent shown
# again after a merge gets no floor (it is unmorphed there by its morph range).
func _check_morph_fade_in() -> void:
	var tree := _make_tree(false)
	var key := CubeSphere.key_at(CubeSphere.geodetic_direction(10.0, 20.0), 4)
	var tries := 0
	while tree.node_info(key).is_empty() and tries < 8:
		_converge(tree, CubeSphere.node_center(key) * EARTH_RADIUS_M * 1.0001)
		tries += 1
	var info := tree.node_info(key)
	if info.is_empty():
		_check(false, "morph fade-in: node %s never built" % key)
		return
	var center: Vector3 = info["center_fixed"]
	var up := center.normalized()
	var split_distance_m := TerrainQuadtree.LOD_FACTOR * float(info["size_m"])
	var near_camera := center + up * (float(info["radius_m"]) + 0.5 * split_distance_m)
	var far_camera := center + up * (float(info["radius_m"]) + 1.2 * split_distance_m)
	_converge(tree, near_camera)
	var child := CubeSphere.children(key)[0]
	var first := float(tree.node_info(child)["morph_floor"])
	tree.update_for_camera(near_camera, 0.5 * TerrainQuadtree.MORPH_IN_S)
	var half := float(tree.node_info(child)["morph_floor"])
	tree.update_for_camera(near_camera, TerrainQuadtree.MORPH_IN_S)
	var done := float(tree.node_info(child)["morph_floor"])
	_check(first == 1.0 and absf(half - 0.5) < 1e-9 and done == 0.0,
			"a new node starts fully morphed and fades in (floor %.2f -> %.2f -> %.2f)" % [first, half, done])
	tree.update_for_camera(far_camera, 0.0)
	var parent_floor := float(tree.node_info(key)["morph_floor"])
	_check(not tree.node_info(key)["split"] and parent_floor == 0.0,
			"a parent shown again after a merge gets no morph floor (%.2f)" % parent_floor)
	tree.queue_free()


# The camera reaches the shader as float32 hi + lo parts of the body-local position (one global per frame) and each
# node's centre likewise (per instance, once): the shader's camera_local = (hi_c - hi_n) + (lo_c - lo_n) in float32
# holds about float32 precision of the camera-node distance, also 6400 km from the body's centre.
func _check_camera_precision() -> void:
	var camera := Frames.to_render(_above(25.99686, -97.15394, 2.0))
	var worst := 0.0
	for offset: Vector3 in [Vector3(30.123456, -2.718282, 41.421356),
			Vector3(-5000.314159, 300.577216, 1200.693147), Vector3(2.0e6 + 0.1234, -1.0e6 - 0.5678, 3.0e5 + 0.9012)]:
		var node := camera + offset
		var c := TerrainQuadtree.split_float32(camera)
		var n := TerrainQuadtree.split_float32(node)
		var local := _f32(_f32(c[0] - n[0]) + _f32(c[1] - n[1]))
		var exact := camera - node
		worst = maxf(worst, (local - exact).length() / exact.length())
	var hi_lo := TerrainQuadtree.split_float32(camera)
	_check(worst < 3.0e-7 and hi_lo[0] == _f32(hi_lo[0]) and (hi_lo[0] + hi_lo[1] - camera).length() < 1e-6,
			"camera hi/lo split: camera-node vector to float32 precision (worst relative error %s)"
			% String.num_scientific(worst))


## v rounded to float32 per component.
static func _f32(v: Vector3) -> Vector3:
	var a := PackedFloat32Array([v.x, v.y, v.z])
	return Vector3(a[0], a[1], a[2])


## (max distance between each child's morphed grid vertex and the parent's morphed surface there, the parent's largest
## morph factor, the children's smallest) for a camera at camera_fixed (the shader's morph formula).
func _swap_error(tree: TerrainQuadtree, key: Vector4i, parent_data: Dictionary, children_data: Array,
		camera_fixed: Vector3) -> Vector3:
	var p_range: Vector2 = tree.node_info(key)["morph_range_m"]
	var p_center: Vector3 = parent_data["center_fixed"]
	var p_morphed := _morphed(parent_data, p_range, camera_fixed)
	var max_error := 0.0
	var parent_max := 0.0
	for v: Vector3 in parent_data["vertices"]:
		parent_max = maxf(parent_max, TerrainQuadtree.morph_factor(v.distance_to(Frames.to_render(camera_fixed - p_center)),
				p_range))
	var child_min := 1.0
	for c in 4:
		var child: Dictionary = children_data[c]
		var child_key: Vector4i = child["key"]
		var c_range: Vector2 = tree.node_info(child_key)["morph_range_m"]
		var c_center: Vector3 = child["center_fixed"]
		var c_morphed := _morphed(child, c_range, camera_fixed)
		for v: Vector3 in child["vertices"]:
			child_min = minf(child_min, TerrainQuadtree.morph_factor(v.distance_to(Frames.to_render(camera_fixed
					- c_center)), c_range))
		for b in ROW:
			for a in ROW:
				var target := c_center + Frames.from_render(c_morphed[b * ROW + a])
				var px := ((child_key.z & 1) * CELLS + a) / 2.0
				var py := ((child_key.w & 1) * CELLS + b) / 2.0
				var cx := mini(int(px), CELLS - 1)
				var cy := mini(int(py), CELLS - 1)
				var fx := px - cx
				var fy := py - cy
				var v00 := p_morphed[cy * ROW + cx]
				var v10 := p_morphed[cy * ROW + cx + 1]
				var v01 := p_morphed[(cy + 1) * ROW + cx]
				var v11 := p_morphed[(cy + 1) * ROW + cx + 1]
				var on_parent: Vector3
				if fx + fy <= 1.0:
					on_parent = v00 + (v10 - v00) * fx + (v01 - v00) * fy
				else:
					on_parent = v11 + (v01 - v11) * (1.0 - fx) + (v10 - v11) * (1.0 - fy)
				max_error = maxf(max_error, target.distance_to(p_center + Frames.from_render(on_parent)))
	return Vector3(max_error, parent_max, child_min)


## Node vertices (render axes, relative to the centre) moved by the shader's morph for a camera at camera_fixed.
static func _morphed(data: Dictionary, range_m: Vector2, camera_fixed: Vector3) -> PackedVector3Array:
	var vertices: PackedVector3Array = data["vertices"]
	var morph: PackedFloat32Array = data["morph"]
	var camera_local := Frames.to_render(camera_fixed - data["center_fixed"])
	var result := PackedVector3Array()
	result.resize(vertices.size())
	for k in vertices.size():
		var f := TerrainQuadtree.morph_factor(vertices[k].distance_to(camera_local), range_m)
		result[k] = vertices[k] + Vector3(morph[4 * k], morph[4 * k + 1], morph[4 * k + 2]) * f
	return result


# A fresh checkout (no data/): load_terrain on a directory without terrain reports nothing loaded and does not crash;
# surface samples then report no data (the renderer falls back to the flat rule). Exercised on every machine.
func _check_fresh_checkout() -> void:
	var core := StarsimCore.new()
	var missing := _config_dir.path_join("../nonexistent_%d/config" % Time.get_ticks_usec()).simplify_path()
	var loaded := core.load_terrain(missing)
	var samples: Dictionary = core.surface_samples("earth", PackedFloat64Array([25.99]), PackedFloat64Array([-97.15]))
	_check(loaded.get("earth", true) == false and loaded.get("moon", true) == false and not bool(samples["present"]),
			"fresh checkout: load_terrain(%s) -> %s, surface samples absent" % [missing, loaded])


# A body without water (the Moon) builds no water mask per node and its tree uploads without one.
func _check_moon_without_water() -> void:
	var moon := _moon_heights()
	var key := CubeSphere.key_at(CubeSphere.geodetic_direction(-84.0, 0.0), 8)
	var built := TerrainNodeBuilder.build(key, moon, null, MOON_RADIUS_M)
	var earth := TerrainNodeBuilder.build(key, HeightSource.new(), null, EARTH_RADIUS_M)
	var tree := _make_tree(false, moon)
	var converged := _converge(tree, moon.position_fixed(-84.0, 0.0, 2000.0))
	var earth_has_mask: bool = earth["water_mask"] is Image
	_check(moon.has_water() == false and built["water_mask"] == null and earth_has_mask
			and converged and absf(_coverage(tree) - 4.0 * PI) < 1e-9,
			"moon: no water mask built (earth node has one: %s), tree uploads and covers without one" % earth_has_mask)
	tree.queue_free()


# Earth nodes and Moon nodes without the core's terrain build no horizon map (nothing changes in their path).
func _check_no_horizon_without_terrain() -> void:
	var key := CubeSphere.key_at(CubeSphere.geodetic_direction(10.0, 20.0), 6)
	var earth := TerrainNodeBuilder.build(key, HeightSource.new(), null, EARTH_RADIUS_M)
	var moon := TerrainNodeBuilder.build(key, _moon_heights(), null, MOON_RADIUS_M)
	var earth_water: bool = (earth["custom2"] as PackedFloat32Array)[2] < 1.0  # water mask uv, not a grid position
	_check(earth["horizon"] == null and moon["horizon"] == null and earth_water,
			"no horizon map on an Earth node or on a Moon node without the core's terrain")


# With the Moon terrain: a node at the IM-2 site (leaf and level 7) bakes a horizon map of the stated size; its texels
# are the core's elevations (half-float precision, single-ray reference within the banded bake's accuracy), the shared
# edge of two neighbours is identical, the vertices' CUSTOM2 is their grid position, skirts take their edge vertex's.
func _check_moon_horizon() -> void:
	var site: Dictionary = TomlLite.load_file(_config_dir.path_join("lunar_sites.toml"))["site"][0]
	var lat := float(site["latitude_deg"])
	var lon := float(site["longitude_deg"])
	var heights := _moon_heights()
	heights.core = _core
	var direction := CubeSphere.geodetic_direction(lat, lon)
	var leaf_level := 7
	while CubeSphere.node_size_m(CubeSphere.key_at(direction, leaf_level), MOON_RADIUS_M) / CELLS > 12.0:
		leaf_level += 1
	for level in [leaf_level, 7]:
		var key := CubeSphere.key_at(direction, level)
		var node := TerrainNodeBuilder.build(key, heights, null, MOON_RADIUS_M)
		var image: Image = node["horizon"]
		_check(image != null and image.get_width() == ROW * TerrainNodeBuilder.HORIZON_BLOCKS
				and image.get_height() == ROW and image.get_format() == Image.FORMAT_RGBAH,
				"moon level %d: horizon map %s (%.0f ms bake and encode)" % [level, "%dx%d" % [image.get_width(),
				image.get_height()] if image != null else "missing", node["horizon_ms"]])
		if image == null:
			continue
		var worst_deg := 0.0
		var sum_deg := 0.0
		var count := 0
		var max_elevation_deg := 0.0
		for point: Vector2i in [Vector2i(32, 32), Vector2i(10, 50), Vector2i(0, 0), Vector2i(64, 64), Vector2i(20, 3)]:
			var vertex_lat_lon := _vertex_lat_lon(key, point.x, point.y)
			var samples: Dictionary = _core.surface_samples("moon", PackedFloat64Array([vertex_lat_lon.x]),
					PackedFloat64Array([vertex_lat_lon.y]))
			var height: float = (samples["height_m"] as PackedFloat64Array)[0]
			for k in TerrainNodeBuilder.N_AZ:
				var reference: float = _core.horizon_elevation_rad("moon", vertex_lat_lon.x, vertex_lat_lon.y, height,
						k * TAU / TerrainNodeBuilder.N_AZ)
				var stored := TerrainNodeBuilder.horizon_value(image, point.x, point.y, k)
				max_elevation_deg = maxf(max_elevation_deg, absf(rad_to_deg(stored)))
				if absf(reference) < deg_to_rad(5.0):
					var error_deg := absf(rad_to_deg(stored - reference))
					worst_deg = maxf(worst_deg, error_deg)
					sum_deg += error_deg
					count += 1
		# The single ray starts from the data resolution, the bake from the node spacing: at the 373 m spacing the two
		# sample sets differ (the reference itself moves by up to 3 deg when the observer shifts by 3 m, core task 2a).
		var worst_limit_deg := 0.5 if level == leaf_level else 5.0
		var mean_limit_deg := 0.1 if level == leaf_level else 1.0
		_check(count > 0 and worst_deg < worst_limit_deg and sum_deg / count < mean_limit_deg,
				"moon level %d: horizon texels match the core's "
				% level + "single-ray march (%d samples below 5 deg: worst %.2f deg, mean %.3f deg; largest stored "
				% [count, worst_deg, sum_deg / maxf(count, 1)] + "elevation %.1f deg)" % max_elevation_deg)
		var vertices_ok := true
		var custom2: PackedFloat32Array = node["custom2"]
		for s in TerrainNodeBuilder.SKIRT_EDGE.size():
			var e := TerrainNodeBuilder.SKIRT_EDGE[s]
			var v := TerrainNodeBuilder.GRID_VERTICES + s
			vertices_ok = vertices_ok and custom2[2 * v] == custom2[2 * e] and custom2[2 * v + 1] == custom2[2 * e + 1] 					and custom2[2 * e] == float(e % ROW) and custom2[2 * e + 1] == float(e / ROW)
		_check(vertices_ok, "moon level %d: CUSTOM2 is the grid position, skirts take their edge vertex's" % level)
	var key := CubeSphere.key_at(direction, 7)
	var east_key := Vector4i(key.x, key.y, key.z + 1, key.w)
	var west := TerrainNodeBuilder.build(key, heights, null, MOON_RADIUS_M)["horizon"] as Image
	var east := TerrainNodeBuilder.build(east_key, heights, null, MOON_RADIUS_M)["horizon"] as Image
	# In half-float steps (2^-10 of the value): the two nodes' vertex coordinates differ by ~1e-12 (measured: 2 steps).
	var seam := 0.0
	for b in ROW:
		for k in TerrainNodeBuilder.N_AZ:
			var west_value := TerrainNodeBuilder.horizon_value(west, CELLS, b, k)
			var difference := absf(west_value - TerrainNodeBuilder.horizon_value(east, 0, b, k))
			seam = maxf(seam, difference / maxf(absf(west_value) / 1024.0, 1e-7))
	_check(seam <= 4.0, "moon: the shared edge of two neighbouring nodes has the same horizons (max %.2f half-float "
			% seam + "steps)")


## Selenographic (lat, lon) in deg of grid vertex (a, b) of a node.
static func _vertex_lat_lon(key: Vector4i, a: int, b: int) -> Vector2:
	var s := CubeSphere.warp(CubeSphere.lattice_coordinate(key.z, key.y, a, CELLS))
	var t := CubeSphere.warp(CubeSphere.lattice_coordinate(key.w, key.y, b, CELLS))
	return CubeSphere.lat_lon_deg(CubeSphere.direction_warped(key.x, s, t))


# Camera jumps (pad -> 1000 km up -> the far side of the Earth) with one upload per frame: coverage on every frame.
func _check_jump() -> void:
	var tree := _make_tree(false)
	var site := _site("starbase_olp1")
	var lat: float = site["latitude_deg"]
	var lon: float = site["longitude_deg"]
	_converge(tree, _above(lat, lon, site["height_m"] + EYE_HEIGHT_M))
	tree.uploads_per_frame = 1
	var covered := true
	var drawn_is_visible := true
	var frames := 0
	var max_uploads := 0
	for camera: Vector3 in [_above(lat, lon, 1.0e6), _above(-lat, lon + 180.0, EYE_HEIGHT_M)]:
		for frame in MAX_FRAMES:
			tree.update_for_camera(camera)
			frames += 1
			covered = covered and absf(_coverage(tree) - 4.0 * PI) < 1e-9
			# what is drawn is exactly what is visible: no stale instance shows, no drawn leaf is hidden
			var drawn := tree.drawn_keys()
			var shown := tree.visible_keys()
			drawn.sort()
			shown.sort()
			drawn_is_visible = drawn_is_visible and drawn == shown
			max_uploads = maxi(max_uploads, tree.stats["uploads_last_frame"])
			if tree.stats["nodes_loading"] == 0 and tree.stats["uploads_last_frame"] == 0:
				break
	_check(covered and drawn_is_visible and max_uploads == 1,
			"jump keeps coverage (4 pi on each of %d frames, drawn set = visible instances %s, max %d upload per frame)"
			% [frames, drawn_is_visible, max_uploads])
	tree.queue_free()


# With the Earth terrain: level >= 17 under the OLP-1 camera; the finest mesh equals core.surface_samples at the pad
# centre, an LZ-40 plateau point and a dune point 2 km west of OLP-1.
func _check_data() -> void:
	var tree := _make_tree(true)
	var olp := _site("starbase_olp1")
	var lz := _site("ccsfs_lz40")
	var t0 := Time.get_ticks_msec()
	_converge(tree, _above(olp["latitude_deg"], olp["longitude_deg"], olp["height_m"] + EYE_HEIGHT_M))
	var leaf := _leaf_at(tree, olp["latitude_deg"], olp["longitude_deg"])
	_check(leaf.y >= 17, "refines towards the camera: leaf level %d at OLP-1 with data (%d leaves, %d ms)"
			% [leaf.y, tree.drawn_keys().size(), Time.get_ticks_msec() - t0])
	var metres_per_deg_lon := EARTH_RADIUS_M * PI / 180.0 * cos(deg_to_rad(olp["latitude_deg"]))
	var points := [["OLP-1 pad centre", olp["latitude_deg"], olp["longitude_deg"]],
			["dune 2 km west of OLP-1", olp["latitude_deg"], olp["longitude_deg"] - 2000.0 / metres_per_deg_lon],
			["LZ-40 plateau", lz["latitude_deg"], lz["longitude_deg"] + 20.0 / metres_per_deg_lon]]
	for point: Array in points:
		var lat: float = point[1]
		var lon: float = point[2]
		var samples: Dictionary = _core.surface_samples("earth", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
		var height: float = (samples["height_m"] as PackedFloat64Array)[0]
		_converge(tree, _above(lat, lon, height + EYE_HEIGHT_M))
		var key := _leaf_at(tree, lat, lon)
		var error := _surface_error_m(tree, key, lat, lon, height)
		_check(error < 0.05, "finest mesh matches the core: %s, level %d, %.3f m (error %s m)"
				% [point[0], key.y, height, String.num_scientific(error)])
	tree.queue_free()


# Morph ranges: the level-1 roots have no parent mesh and never morph; every deeper node's range is open (start below
# end, morph_range_m's safety clamp not reached) at a face centre and a face corner on the Earth and on the Moon (flat
# surfaces; the data checks repeat it on the relief).
func _check_morph_ranges() -> void:
	for moon: bool in [false, true]:
		var heights := _moon_heights() if moon else HeightSource.new()
		var radius := MOON_RADIUS_M if moon else EARTH_RADIUS_M
		var worst := INF  # smallest (end - start) / end
		var worst_level := -1
		for level in range(TerrainQuadtree.ROOT_LEVEL + 1, TerrainQuadtree.MAX_LEVEL + 1):
			var middle := (1 << level) / 2
			for key: Vector4i in [Vector4i(0, level, middle, middle), Vector4i(0, level, 0, 0)]:
				var node := TerrainNodeBuilder.build(key, heights, null, radius)
				var range_m := TerrainQuadtree.node_morph_range_m(key, _bounding_radius_m(node), radius)
				var margin := (range_m.y - range_m.x) / range_m.y
				if margin < worst:
					worst = margin
					worst_level = level
		_check(worst > 1.0 - TerrainQuadtree.MORPH_START_CLAMP, "morph start < end for every level >= 2 (%s: smallest "
				% ("Moon" if moon else "Earth") + "margin %.3f of the end, level %d)" % [worst, worst_level])
	var tree := _make_tree(false, _moon_heights())
	var roots_unmorphed := tree.drawn_keys().size() == 24
	for key in tree.drawn_keys():
		roots_unmorphed = roots_unmorphed and tree.node_info(key)["morph_range_m"] == TerrainQuadtree.NO_MORPH
	_check(roots_unmorphed, "level-1 roots never morph (no parent mesh)")
	tree.queue_free()


## Largest vertex distance from the node centre (TerrainQuadtree's bounding radius).
static func _bounding_radius_m(node: Dictionary) -> float:
	var radius_sq := 0.0
	for v: Vector3 in node["vertices"]:
		radius_sq = maxf(radius_sq, v.length_squared())
	return sqrt(radius_sq)


# With the Moon terrain: 10 m above the IM-2 site and above a point inside the tile's edge the leaf spacing is at most
# the tile's 20 m and the finest mesh equals core.surface_samples there; above a far-side point the tree stops at the
# coarse limit (the LDEM_16 spacing or the curvature limit, whichever is finer), far above 20 m. Morph ranges of every
# drawn node stay open on the relief.
func _check_moon_data() -> void:
	var tree := _make_tree(true, _moon_heights())
	var site: Dictionary = TomlLite.load_file(_config_dir.path_join("lunar_sites.toml"))["site"][0]
	var tile: Dictionary = TomlLite.load_file(_config_dir.path_join("../data/terrain").simplify_path()
			.path_join(str(site["id"]) + ".toml"))["grid"]
	var tile_m := float(tile["scale_m"])
	var edge := _tile_point(tile, 0.5 * (int(tile["lines"]) - 1), int(tile["samples"]) - 1 - MOON_EDGE_INSET_M / tile_m)
	var points := [["IM-2 site", float(site["latitude_deg"]), float(site["longitude_deg"])],
			["%.0f m inside the tile edge" % MOON_EDGE_INSET_M, edge.x, edge.y]]
	var ranges_open := true
	for point: Array in points:
		var lat: float = point[1]
		var lon: float = point[2]
		var samples: Dictionary = _core.surface_samples("moon", PackedFloat64Array([lat]), PackedFloat64Array([lon]))
		var height: float = (samples["height_m"] as PackedFloat64Array)[0]
		var t0 := Time.get_ticks_msec()
		_converge(tree, tree.height_source.position_fixed(lat, lon, height + MOON_EYE_HEIGHT_M))
		var key := _leaf_at(tree, lat, lon)
		var spacing := CubeSphere.node_size_m(key, MOON_RADIUS_M) / CELLS
		_check(spacing <= tile_m, "moon refines to the 20 m tile: %s, leaf level %d, spacing %.1f m (%d leaves, %d ms)"
				% [point[0], key.y, spacing, tree.drawn_keys().size(), Time.get_ticks_msec() - t0])
		var error := _surface_error_m(tree, key, lat, lon, height)
		_check(error < 0.05, "moon finest mesh matches the core: %s, %.3f m (error %s m)"
				% [point[0], height, String.num_scientific(error)])
		for drawn in tree.drawn_keys():
			var range_m: Vector2 = tree.node_info(drawn)["morph_range_m"]
			ranges_open = ranges_open and range_m.x < TerrainQuadtree.MORPH_START_CLAMP * range_m.y
	_check(ranges_open, "moon morph ranges open on the relief (every drawn node)")
	var far_lat := 0.0
	var far_lon := 180.0
	var far_height: float = (_core.surface_samples("moon", PackedFloat64Array([far_lat]),
			PackedFloat64Array([far_lon]))["height_m"] as PackedFloat64Array)[0]
	_converge(tree, tree.height_source.position_fixed(far_lat, far_lon, far_height + MOON_EYE_HEIGHT_M))
	var far_key := _leaf_at(tree, far_lat, far_lon)
	var ldem_m := _core.surface_resolution_m("moon", far_lat, far_lon)
	var limit := minf(ldem_m, tree.curvature_spacing_m)
	var far_spacing := CubeSphere.node_size_m(far_key, MOON_RADIUS_M) / CELLS
	var parent_spacing := CubeSphere.node_size_m(CubeSphere.parent(far_key), MOON_RADIUS_M) / CELLS
	_check(far_spacing <= limit and parent_spacing > limit, "moon far side stops at the coarse limit: leaf level %d, "
			% far_key.y + "spacing %.0f m (LDEM_16 %.0f m, curvature limit %.0f m)" % [far_spacing, ldem_m,
			tree.curvature_spacing_m])
	tree.queue_free()


## Selenographic (lat, lon) in deg of a polar stereographic tile pixel (line, sample): the inverse of the core's
## PolarStereographicTile::position (south pole: rho = 2 R tan(pi/4 + lat/2), x = rho sin lon, y = -rho cos lon).
static func _tile_point(tile: Dictionary, line: float, sample: float) -> Vector2:
	var scale := float(tile["scale_m"])
	var x := (sample + float(tile["first_sample"]) - float(tile["pole_sample"])) * scale
	var y := (line + float(tile["first_line"]) - float(tile["pole_line"])) * scale
	var rho := sqrt(x * x + y * y)
	var lat := 2.0 * atan(rho / (2.0 * float(tile["sphere_radius_m"]))) - PI / 2.0
	return Vector2(rad_to_deg(lat), rad_to_deg(atan2(x, -y)))


## |drawn surface height - expected| at a geodetic point (Earth: WGS84, Moon: sphere): the node rebuilt (same data as
## drawn), the point's geodetic normal intersected with the node's triangles (morph 0).
func _surface_error_m(tree: TerrainQuadtree, key: Vector4i, lat: float, lon: float, expected_m: float) -> float:
	if key.x < 0:
		return INF
	var node := TerrainNodeBuilder.build(key, tree.height_source, null, tree.radius_m)
	var vertices: PackedVector3Array = node["vertices"]
	var center: Vector3 = node["center_fixed"]
	var origin := tree.height_source.position_fixed(lat, lon, 0.0)
	var up := CubeSphere.geodetic_direction(lat, lon)
	var face := key.x
	var major := up.dot(CubeSphere.FACE_NORMAL[face])
	var scale := float(CELLS << key.y) * 0.5
	var a0 := int(floor((atan(up.dot(CubeSphere.FACE_U[face]) / major) * 4.0 / PI + 1.0) * scale)) - key.z * CELLS
	var b0 := int(floor((atan(up.dot(CubeSphere.FACE_V[face]) / major) * 4.0 / PI + 1.0) * scale)) - key.w * CELLS
	for b in range(maxi(b0 - 1, 0), mini(b0 + 2, CELLS)):
		for a in range(maxi(a0 - 1, 0), mini(a0 + 2, CELLS)):
			var k := b * ROW + a
			for tri: Array in [[k, k + ROW, k + 1], [k + 1, k + ROW, k + ROW + 1]]:
				var p0: Vector3 = center + Frames.from_render(vertices[tri[0]])
				var p1: Vector3 = center + Frames.from_render(vertices[tri[1]])
				var p2: Vector3 = center + Frames.from_render(vertices[tri[2]])
				var n := (p1 - p0).cross(p2 - p0)
				var h := n.dot(p0 - origin) / n.dot(up)
				var q := origin + up * h
				var w1 := (q - p0).cross(p2 - p0).dot(n) / n.length_squared()
				var w2 := (p1 - p0).cross(q - p0).dot(n) / n.length_squared()
				if w1 >= -1e-9 and w2 >= -1e-9 and w1 + w2 <= 1.0 + 1e-9:
					return absf(h - expected_m)
	return INF


static func _solid_angle(key: Vector4i) -> float:
	var c := CubeSphere.node_corners(key)
	return _triangle_solid_angle(c[0], c[1], c[2]) + _triangle_solid_angle(c[0], c[2], c[3])


static func _triangle_solid_angle(a: Vector3, b: Vector3, c: Vector3) -> float:
	return 2.0 * atan2(absf(a.dot(b.cross(c))), 1.0 + a.dot(b) + b.dot(c) + c.dot(a))


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1
