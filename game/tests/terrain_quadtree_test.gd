# Headless test of the terrain quadtree (TerrainQuadtree): coverage, refinement, throttled loading, merge hysteresis and
# (with data/) the finest mesh against the core's surface. Nodes are built synchronously (no worker threads).
# Run: godot --headless --path game --script res://tests/terrain_quadtree_test.gd
extends SceneTree

const EARTH_RADIUS_M := 6378137.0
const EYE_HEIGHT_M := 2.0
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


func _initialize() -> void:
	var config_dir := ProjectSettings.globalize_path("res://").path_join("../config").simplify_path()
	_sites = TomlLite.load_file(config_dir.path_join("launch_sites.toml")).get("site", [])
	_core = StarsimCore.new()
	_terrain = bool(_core.load_terrain(config_dir).get("earth", false))
	_check_flat_rule()
	_check_hysteresis()
	_check_morph_continuity()
	_check_jump()
	if _terrain:
		_check_data()
	else:
		print("  note  data/ Earth terrain absent: 'refines towards the camera' (data) and 'finest mesh matches the "
				+ "core' skipped")
	print("TERRAIN QUADTREE TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


## A quadtree on a height source (core null: the flat rule), synchronous loading, no imagery.
func _make_tree(with_core: bool, heights := HeightSource.new()) -> TerrainQuadtree:
	heights.sites = _sites
	heights.core = _core if with_core else null
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain_earth.gdshader")
	var tree := TerrainQuadtree.new()
	tree.synchronous = true
	root.add_child(tree)
	tree.setup(heights, null, EARTH_RADIUS_M, material)
	return tree


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


# Camera jumps (pad -> 1000 km up -> the far side of the Earth) with one upload per frame: coverage on every frame.
func _check_jump() -> void:
	var tree := _make_tree(false)
	var site := _site("starbase_olp1")
	var lat: float = site["latitude_deg"]
	var lon: float = site["longitude_deg"]
	_converge(tree, _above(lat, lon, site["height_m"] + EYE_HEIGHT_M))
	tree.uploads_per_frame = 1
	var covered := true
	var frames := 0
	var max_uploads := 0
	for camera: Vector3 in [_above(lat, lon, 1.0e6), _above(-lat, lon + 180.0, EYE_HEIGHT_M)]:
		for frame in MAX_FRAMES:
			tree.update_for_camera(camera)
			frames += 1
			covered = covered and absf(_coverage(tree) - 4.0 * PI) < 1e-9
			max_uploads = maxi(max_uploads, tree.stats["uploads_last_frame"])
			if tree.stats["nodes_loading"] == 0 and tree.stats["uploads_last_frame"] == 0:
				break
	_check(covered and max_uploads == 1, "jump keeps coverage (4 pi on each of %d frames, max %d upload per frame)"
			% [frames, max_uploads])
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


## |drawn surface height - expected| at a geodetic point: the node rebuilt (same data as drawn), the point's geodetic
## normal intersected with the node's triangles (morph 0).
func _surface_error_m(tree: TerrainQuadtree, key: Vector4i, lat: float, lon: float, expected_m: float) -> float:
	if key.x < 0:
		return INF
	var node := TerrainNodeBuilder.build(key, tree.height_source, null, EARTH_RADIUS_M)
	var vertices: PackedVector3Array = node["vertices"]
	var center: Vector3 = node["center_fixed"]
	var origin := HeightSource.geodetic_to_fixed(lat, lon, 0.0)
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
