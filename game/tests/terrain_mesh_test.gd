# Headless test of the cube-sphere quadtree (CubeSphere), the per-node mesh builder (TerrainNodeBuilder), the height
# source's flat rule (HeightSource) and the imagery crop fallback (ImagerySource). Synthetic heights, no data, no GPU.
# Run: godot --headless --path game --script res://tests/terrain_mesh_test.gd
extends SceneTree

const EARTH_RADIUS_M := 6378137.0
const ROW := TerrainNodeBuilder.ROW
const CELLS := TerrainNodeBuilder.CELLS

var _failures := 0


## Height source of a quadratic hill around a centre direction (h = peak - curvature * horizontal distance^2).
class HillSource extends HeightSource:
	var center_dir := Vector3(1, 0, 0)
	var peak_m := 300.0
	var curvature_per_m := 1e-4
	const RADIUS_M := 6378137.0

	func sample(lats_deg: PackedFloat64Array, lons_deg: PackedFloat64Array) -> Dictionary:
		var heights := PackedFloat64Array()
		heights.resize(lats_deg.size())
		var water := PackedByteArray()
		water.resize(lats_deg.size())
		for k in lats_deg.size():
			var d := HeightSource.geodetic_to_fixed(lats_deg[k], lons_deg[k]).normalized()
			var distance_m := d.angle_to(center_dir) * RADIUS_M
			heights[k] = peak_m - curvature_per_m * distance_m * distance_m
		return {"height_m": heights, "water": water}


## Height source whose surface is the exact plane normal . p = offset (Earth-fixed metres).
class PlaneSource extends HeightSource:
	var normal := Vector3(1, 0, 0)
	var offset_m := 0.0

	func sample(lats_deg: PackedFloat64Array, lons_deg: PackedFloat64Array) -> Dictionary:
		var heights := PackedFloat64Array()
		heights.resize(lats_deg.size())
		var water := PackedByteArray()
		water.resize(lats_deg.size())
		for k in lats_deg.size():
			var base := HeightSource.geodetic_to_fixed(lats_deg[k], lons_deg[k])
			var up := HeightSource.geodetic_to_fixed(lats_deg[k], lons_deg[k], 1.0) - base  # unit geodetic normal
			heights[k] = (offset_m - normal.dot(base)) / normal.dot(up)
		return {"height_m": heights, "water": water}


## Height source of a straight coast along a meridian: water west of coast_lon_deg (sea level 0 m), land east of it.
class CoastSource extends HeightSource:
	var coast_lon_deg := 0.0

	func is_water(lon_deg: float) -> bool:
		return lon_deg < coast_lon_deg

	func sample(lats_deg: PackedFloat64Array, lons_deg: PackedFloat64Array) -> Dictionary:
		var heights := PackedFloat64Array()
		heights.resize(lats_deg.size())
		var water := PackedByteArray()
		water.resize(lats_deg.size())
		for k in lats_deg.size():
			heights[k] = 0.0 if is_water(lons_deg[k]) else 5.0
			water[k] = 1 if is_water(lons_deg[k]) else 0
		return {"height_m": heights, "water": water}


func _initialize() -> void:
	_check_cover()
	_check_children()
	_check_antimeridian()
	_check_pole()
	_check_morph()
	_check_skirts()
	_check_plane_normals()
	_check_missing_tile()
	_check_flat_rule()
	_check_water_mask()
	print("TERRAIN MESH TEST ", "PASSED" if _failures == 0 else "FAILED (%d)" % _failures)
	quit(0 if _failures == 0 else 1)


# Level 3: 6 * 64 nodes whose solid angles sum to 4 pi; corners on shared face edges are bit-identical.
func _check_cover() -> void:
	var total := 0.0
	for face in 6:
		for i in 8:
			for j in 8:
				total += _solid_angle(Vector4i(face, 3, i, j))
	_check(absf(total - 4.0 * PI) < 1e-9, "cube sphere covers the sphere once (sum %.15f, error %s)"
			% [total, String.num_scientific(total - 4.0 * PI)])
	# Every face-edge direction must appear on exactly two faces with identical bits.
	var edge_points := {}
	for face in 6:
		for k in 9:
			var p := -1.0 + 2.0 * k / 8.0
			for st: Vector2 in [Vector2(-1.0, p), Vector2(1.0, p), Vector2(p, -1.0), Vector2(p, 1.0)]:
				var d := CubeSphere.direction(face, st.x, st.y)
				edge_points[d] = int(edge_points.get(d, 0)) + 1
	var shared := true
	for d: Vector3 in edge_points:
		var corner := absf(absf(d.x) - absf(d.y)) < 1e-12 and absf(absf(d.y) - absf(d.z)) < 1e-12
		var expected := 3 * 2 if corner else 2  # a cube corner: 3 faces x 2 edges each
		shared = shared and int(edge_points[d]) == expected
	_check(shared and edge_points.size() == 8 + 12 * 7, "adjacent faces share edge directions exactly (%d points)"
			% edge_points.size())


func _check_children() -> void:
	var key := Vector4i(2, 4, 5, 11)
	var kids := CubeSphere.children(key)
	var sum := 0.0
	var parents_ok := kids.size() == 4
	for kid: Vector4i in kids:
		sum += _solid_angle(kid)
		parents_ok = parents_ok and CubeSphere.parent(kid) == key
	var corners := CubeSphere.node_corners(key)
	var corner_ok := CubeSphere.node_corners(kids[0])[0] == corners[0] \
			and CubeSphere.node_corners(kids[3])[2] == corners[2]
	_check(parents_ok and corner_ok and absf(sum - _solid_angle(key)) < 1e-15,
			"children tile the parent (solid angle error %s)" % String.num_scientific(sum - _solid_angle(key)))
	_check(CubeSphere.key_at(CubeSphere.direction(2, -0.3, 0.7), 6) == Vector4i(2, 6, 22, 54),
			"key_at finds the node under a direction")


# Nodes on the antimeridian: global u continuous inside the node, the box east of 180 deg, no NaN.
func _check_antimeridian() -> void:
	var source := HeightSource.new()
	for key: Vector4i in [Vector4i(1, 0, 0, 0), Vector4i(1, 3, 3, 4), Vector4i(1, 3, 4, 4)]:
		var node := TerrainNodeBuilder.build(key, source, null, EARTH_RADIUS_M)
		var custom1: PackedFloat32Array = node["custom1"]
		var max_step := 0.0
		var finite := true
		for b in ROW:
			for a in ROW - 1:
				var k := b * ROW + a
				max_step = maxf(max_step, absf(custom1[4 * (k + 1)] - custom1[4 * k]))
		for value in custom1:
			finite = finite and is_finite(value)
		for v: Vector3 in node["vertices"]:
			finite = finite and v.is_finite()
		_check(finite and max_step < 0.01, "antimeridian node %s: u continuous (max step %.5f), no NaN" % [key, max_step])
	var box := CubeSphere.lat_lon_box(Vector4i(1, 0, 0, 0))
	_check(box.end.x > 180.0 and box.position.x < 180.0, "antimeridian box reaches east of 180 deg (%s)" % box)


func _check_pole() -> void:
	var key := Vector4i(4, 2, 2, 2)  # touches the north pole at its (s0, t0) corner
	var node := TerrainNodeBuilder.build(key, HeightSource.new(), null, EARTH_RADIUS_M)
	var finite := true
	for v: Vector3 in node["vertices"]:
		finite = finite and v.is_finite()
	for n: Vector3 in node["normals"]:
		finite = finite and n.is_finite()
	var custom1: PackedFloat32Array = node["custom1"]
	var pole_lon := custom1[0] * 360.0 - 180.0  # grid vertex (0, 0) is the pole
	var center := CubeSphere.lat_lon_deg(CubeSphere.direction(4, 0.25, 0.25))
	_check(finite, "pole node: all vertices and normals finite")
	_check(absf(pole_lon - center.y) < 1e-4, "pole vertex longitude = node-centre longitude (%.6f vs %.6f)"
			% [pole_lon, center.y])


# Child of a quadratic hill: every grid vertex + morph lies on the parent's triangle mesh.
func _check_morph() -> void:
	var child := Vector4i(0, 12, 2049, 2050)
	var parent := CubeSphere.parent(child)
	var source := HillSource.new()
	source.center_dir = CubeSphere.direction(0, CubeSphere.lattice_coordinate(child.z, 12, 32, CELLS),
			CubeSphere.lattice_coordinate(child.w, 12, 32, CELLS))
	var c := TerrainNodeBuilder.build(child, source, null, EARTH_RADIUS_M)
	var p := TerrainNodeBuilder.build(parent, source, null, EARTH_RADIUS_M)
	var c_vertices: PackedVector3Array = c["vertices"]
	var p_vertices: PackedVector3Array = p["vertices"]
	var morph: PackedFloat32Array = c["morph"]
	var max_error := 0.0
	var max_morph := 0.0
	var even_zero := true
	for b in ROW:
		for a in ROW:
			var k := b * ROW + a
			var m := Vector3(morph[4 * k], morph[4 * k + 1], morph[4 * k + 2])
			max_morph = maxf(max_morph, m.length())
			if a % 2 == 0 and b % 2 == 0:
				even_zero = even_zero and m == Vector3.ZERO
			var target: Vector3 = c["center_fixed"] + Frames.from_render(c_vertices[k] + m)
			var px := ((child.z & 1) * CELLS + a) / 2.0
			var py := ((child.w & 1) * CELLS + b) / 2.0
			var cx := mini(int(px), CELLS - 1)
			var cy := mini(int(py), CELLS - 1)
			var fx := px - cx
			var fy := py - cy
			var v00 := p_vertices[cy * ROW + cx]
			var v10 := p_vertices[cy * ROW + cx + 1]
			var v01 := p_vertices[(cy + 1) * ROW + cx]
			var v11 := p_vertices[(cy + 1) * ROW + cx + 1]
			var on_parent: Vector3
			if fx + fy <= 1.0:
				on_parent = v00 + (v10 - v00) * fx + (v01 - v00) * fy
			else:
				on_parent = v11 + (v01 - v11) * (1.0 - fx) + (v10 - v11) * (1.0 - fy)
			max_error = maxf(max_error, target.distance_to(p["center_fixed"] + Frames.from_render(on_parent)))
	_check(even_zero and max_morph > 1e-3, "morph zero on even vertices, non-zero on odd ones (max %.4f m)" % max_morph)
	_check(max_error < 1e-3, "morph target equals the parent surface (max error %s m)" % String.num_scientific(max_error))
	_check_parent_normals(c)


# CUSTOM3 (parent-level normal): an even vertex's own normal; an odd vertex the normalised mean of the normals of the
# neighbours its morph target interpolates (edge: the two along the edge; cell centre: the parent diagonal).
func _check_parent_normals(node: Dictionary) -> void:
	var normals: PackedVector3Array = node["normals"]
	var custom3: PackedFloat32Array = node["custom3"]
	var ok := custom3.size() == 3 * normals.size()
	var max_error := 0.0
	var max_tilt := 0.0
	for b in ROW:
		for a in ROW:
			var k := b * ROW + a
			var expected := normals[k]
			if a % 2 == 1 and b % 2 == 1:
				expected = (normals[k + 1 - ROW] + normals[k - 1 + ROW]).normalized()
			elif b % 2 == 1:
				expected = (normals[k - ROW] + normals[k + ROW]).normalized()
			elif a % 2 == 1:
				expected = (normals[k - 1] + normals[k + 1]).normalized()
			var got := Vector3(custom3[3 * k], custom3[3 * k + 1], custom3[3 * k + 2])
			max_error = maxf(max_error, got.distance_to(expected))
			max_tilt = maxf(max_tilt, got.distance_to(normals[k]))
	for s in TerrainNodeBuilder.SKIRT_EDGE.size():
		var e := TerrainNodeBuilder.SKIRT_EDGE[s]
		var v := ROW * ROW + s
		ok = ok and custom3[3 * v] == custom3[3 * e] and custom3[3 * v + 2] == custom3[3 * e + 2]
	_check(ok and max_error < 1e-6 and max_tilt > 0.0,
			"CUSTOM3 parent normals: even = own normal, odd = mean of the parent neighbours (error %s, max tilt %s)"
			% [String.num_scientific(max_error), String.num_scientific(max_tilt)])


func _check_skirts() -> void:
	var key := Vector4i(3, 10, 300, 700)
	var hill := HillSource.new()
	hill.center_dir = CubeSphere.node_center(key)  # a hill on the node: heights of a few hundred metres
	var node := TerrainNodeBuilder.build(key, hill, null, EARTH_RADIUS_M)
	var vertices: PackedVector3Array = node["vertices"]
	var morph: PackedFloat32Array = node["morph"]
	var depth := CubeSphere.node_size_m(key, EARTH_RADIUS_M) / CELLS * TerrainNodeBuilder.SKIRT_DEPTH_CELLS
	var edge := TerrainNodeBuilder.SKIRT_EDGE
	var ok := vertices.size() == ROW * ROW + 4 * CELLS and edge.size() == 4 * CELLS
	var max_error := 0.0
	for s in edge.size():
		var k := ROW * ROW + s
		var top := vertices[edge[s]]
		var bottom := vertices[k]
		var a := edge[s] % ROW
		var b := edge[s] / ROW
		ok = (ok and (a == 0 or b == 0 or a == CELLS or b == CELLS) and morph[4 * k + 3] == 1.0
				and morph[4 * edge[s] + 3] == 0.0)
		var up := Frames.to_render(CubeSphere.direction(key.x, CubeSphere.lattice_coordinate(key.z, key.y, a, CELLS),
				CubeSphere.lattice_coordinate(key.w, key.y, b, CELLS)))
		max_error = maxf(max_error, (top - bottom).distance_to(up * depth))
	_check(ok and max_error < 1e-6, "skirts hang %.2f m below the edge (error %s m), CUSTOM0.w = 1"
			% [depth, String.num_scientific(max_error)])
	var indices: PackedInt32Array = node["indices"]
	_check(indices.size() == 6 * CELLS * CELLS + 12 * 4 * CELLS and indices == TerrainNodeBuilder.INDICES,
			"indices: the shared constant array (grid + double-sided skirts)")


func _check_plane_normals() -> void:
	var key := Vector4i(0, 14, 9000, 8100)
	var source := PlaneSource.new()
	var center := CubeSphere.direction(0, CubeSphere.lattice_coordinate(key.z, 14, 32, CELLS),
			CubeSphere.lattice_coordinate(key.w, 14, 32, CELLS))
	var east := Vector3(0, 0, 1).cross(center).normalized()
	source.normal = (center + east * 0.3).normalized()  # ~17 deg tilt towards east
	source.offset_m = source.normal.dot(center * EARTH_RADIUS_M)
	var node := TerrainNodeBuilder.build(key, source, null, EARTH_RADIUS_M)
	var expected := Frames.to_render(source.normal)
	var max_error := 0.0
	var normals: PackedVector3Array = node["normals"]
	for k in ROW * ROW:
		max_error = maxf(max_error, normals[k].distance_to(expected))
	_check(max_error < 1e-4, "normals of a tilted plane (max error %s)" % String.num_scientific(max_error))


# A temporary imagery index with one source: level 17 has a gap, level 16 the tile; outside every source -> {}.
func _check_missing_tile() -> void:
	var dir := OS.get_temp_dir().path_join("terrain_mesh_test_%d_%d" % [OS.get_process_id(), Time.get_ticks_usec()])
	var size17 := 180.0 / (1 << 17)
	var size16 := 180.0 / (1 << 16)
	var x17 := int(floor((-97.15 + 180.0) / size17))
	var y17 := int(floor((90.0 - 25.99) / size17))
	var lon := (x17 + 0.5) * size17 - 180.0  # centre of the level-17 gap tile
	var lat := 90.0 - (y17 + 0.5) * size17
	var cover := Rect2(lon - 0.01, lat - 0.01, 0.02, 0.02)
	var tile := Image.create(256, 256, false, Image.FORMAT_RGB8)
	tile.fill(Color(0.2, 0.4, 0.6))
	for z: int in [16, 17]:
		var x := x17 >> (17 - z)
		var y := y17 >> (17 - z)
		for dx: int in [-1, 0, 1]:
			for dy: int in [-1, 0, 1]:
				if z == 17 and dx == 0 and dy == 0:
					continue  # the gap
				var path := dir.path_join("test_src/%d/%d/%d.jpg" % [z, x + dx, y + dy])
				DirAccess.make_dir_recursive_absolute(path.get_base_dir())
				tile.save_jpg(path)
	var index := FileAccess.open(dir.path_join("index.toml"), FileAccess.WRITE)
	index.store_string("[[source]]
id = \"test_src\"
min_level = 16
max_level = 17
north_deg = %.7f
"
			% cover.end.y + "south_deg = %.7f
west_deg = %.7f
east_deg = %.7f
fade_m = 100.0
"
			% [cover.position.y, cover.position.x, cover.end.x])
	index.close()
	var imagery := ImagerySource.new(dir.path_join("index.toml"))
	var box := Rect2(lon - size17 * 0.1, lat - size17 * 0.1, size17 * 0.2, size17 * 0.2)
	var crop := imagery.crop(box, size17 / 256.0)
	var image: Image = crop.get("image")
	_check(not crop.is_empty() and int(crop["level"]) == 16 and absf(crop["box_deg"].size.x - size16) < 1e-12
			and image.get_width() == 256 and image.has_mipmaps() and image.get_format() == Image.FORMAT_RGB8,
			"missing tile falls back to level 16 (%s)" % [crop.get("box_deg")])
	_check(box.intersects(crop["box_deg"]) and crop["box_deg"].encloses(box) and crop["fade_m"] == 100.0,
			"crop box encloses the request, fade_m from the index")
	var full := imagery.crop(Rect2(lon - size17 * 0.1, lat + size17 * 0.9, size17 * 0.2, size17 * 0.2), size17 / 256.0)
	_check(int(full.get("level", -1)) == 17, "a box over present level-17 tiles stays at level 17")
	_check(imagery.crop(Rect2(10.0, 40.0, 0.001, 0.001), size17 / 256.0).is_empty(),
			"a box outside every source returns {}")
	_check(ImagerySource.new(dir.path_join("absent.toml")).crop(box, size17 / 256.0).is_empty(), "absent index: empty")
	_remove_tree(dir)


func _check_flat_rule() -> void:
	var source := HeightSource.new()
	source.sites = TomlLite.load_file(Paths.config_dir().path_join("launch_sites.toml")).get("site", [])
	var starbase := {}
	for site: Dictionary in source.sites:
		if site["id"] == "starbase_olp1":
			starbase = site
	var site_height: float = starbase.get("height_m", NAN)
	for place: Array in [[starbase.get("latitude_deg", 0.0), starbase.get("longitude_deg", 0.0), site_height, "Starbase"],
			[30.0, -40.0, 0.0, "Atlantic"]]:
		var key := CubeSphere.key_at(CubeSphere.geodetic_direction(place[0], place[1]), 12)
		var node := TerrainNodeBuilder.build(key, source, null, EARTH_RADIUS_M)
		var ll := CubeSphere.lat_lon_deg(CubeSphere.direction(key.x, CubeSphere.lattice_coordinate(key.z, 12, 32, CELLS),
				CubeSphere.lattice_coordinate(key.w, 12, 32, CELLS)))
		var expected := source.position_fixed(ll.x, ll.y, place[2])
		var error: float = node["center_fixed"].distance_to(expected)
		var water: Image = node["water_mask"]
		_check(error < 1e-6 and water.get_width() == 128 and water.get_format() == Image.FORMAT_L8
				and water.get_data().count(0) == 128 * 128,
				"flat rule without terrain: %s vertex at %.2f m (error %s m), no water"
				% [place[3], place[2], String.num_scientific(error)])
		_check(node["max_resolution_m"] == INF, "without terrain the data resolution is infinite")


# Water mask follows the core: a straight coast through a node; every texel equals the source's water flag at the
# texel centre (255 water, 0 land), and each grid vertex's CUSTOM2 uv lands on a texel of its own side.
func _check_water_mask() -> void:
	var key := CubeSphere.key_at(CubeSphere.geodetic_direction(30.0, -40.0), 10)
	var box := CubeSphere.lat_lon_box(key)
	var source := CoastSource.new()
	source.coast_lon_deg = box.position.x + 0.37 * box.size.x  # not on a texel boundary
	var node := TerrainNodeBuilder.build(key, source, null, EARTH_RADIUS_M)
	var mask: Image = node["water_mask"]
	var px := TerrainNodeBuilder.WATER_MASK_PX
	var texels_ok := mask.get_width() == px and mask.get_height() == px and mask.get_format() == Image.FORMAT_L8
	var water_texels := 0
	var data := mask.get_data()
	for r in px:
		for c in px:
			var lon := box.position.x + (c + 0.5) / px * box.size.x
			var value := data[r * px + c]
			var expected := 255 if source.is_water(lon) else 0
			texels_ok = texels_ok and value == expected
			water_texels += 1 if value == 255 else 0
	_check(texels_ok and water_texels > 0 and water_texels < px * px,
			"water mask follows the core: texels match the coast at their centres (%d of %d water)"
			% [water_texels, px * px])
	var custom1: PackedFloat32Array = node["custom1"]
	var custom2: PackedFloat32Array = node["custom2"]
	var texel_deg := box.size.x / px
	var uv_ok := true
	for v in TerrainNodeBuilder.GRID_VERTICES:
		var lon := custom1[4 * v] * 360.0 - 180.0
		if absf(lon - source.coast_lon_deg) < 1.5 * texel_deg:
			continue  # the texel under a vertex next to the coast may lie on the other side
		var c := clampi(int(custom2[2 * v] * px), 0, px - 1)
		var r := clampi(int(custom2[2 * v + 1] * px), 0, px - 1)
		uv_ok = uv_ok and (data[r * px + c] == 255) == source.is_water(lon)
	_check(uv_ok, "CUSTOM2 uv of each grid vertex lands on a mask texel of its own side of the coast")


static func _solid_angle(key: Vector4i) -> float:
	var c := CubeSphere.node_corners(key)
	return _triangle_solid_angle(c[0], c[1], c[2]) + _triangle_solid_angle(c[0], c[2], c[3])


static func _triangle_solid_angle(a: Vector3, b: Vector3, c: Vector3) -> float:
	return 2.0 * atan2(absf(a.dot(b.cross(c))), 1.0 + a.dot(b) + b.dot(c) + c.dot(a))


static func _remove_tree(path: String) -> void:
	for sub in DirAccess.get_directories_at(path):
		_remove_tree(path.path_join(sub))
	for file in DirAccess.get_files_at(path):
		DirAccess.remove_absolute(path.path_join(file))
	DirAccess.remove_absolute(path)


func _check(condition: bool, label: String) -> void:
	print(("  ok    " if condition else "  FAIL  ") + label)
	if not condition:
		_failures += 1
