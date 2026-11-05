class_name MoonRockLayer
extends RefCounted
## The Moon's rocks drawn as instanced meshes by projected size (moon procedural surface plan Task 11; shader
## moon_rock.gdshader, docs/subsystems/moon-surface.md "Rocks drawn"). No physics here: the rocks are the core's
## (StarsimCore.moon_rocks_in_node, the same LunarRocks the contact and the touchdown checks query); this class only
## lays them out for the GPU.
##
## Per terrain node (TerrainNodeBuilder.build calls build_node on its worker): the rocks whose base lies in the node
## with D >= the node's list limit, packed into one MultiMesh buffer per template LOD (transform relative to the node
## centre, colour = base grid position + albedo-map uv, custom = D, exposed height, albedo factor, template). A rock
## goes into every LOD whose projected-diameter band it can reach while the node is drawn; the shader draws it only in
## the LOD whose band holds its current size (one reference point: the camera / head centre), and only from the
## threshold diameter up (growth band to 2 x). TerrainQuadtree attaches the MultiMeshInstance3Ds to the node's mesh
## instance at the upload (attach) and removes them at its release (detach).
##
## The node's list limit: rocks smaller than ROCK_PIXEL_THRESHOLD pixels at the nearest distance the camera can have to
## a drawn leaf of that level (NODE_NEAR_FACTOR x its size: it splits at TerrainQuadtree.LOD_FACTOR x size) can never
## be geometry there; the leaf under the camera (MAX_LEVEL) lists every rock from PEBBLE_GEOMETRY_LIMIT_M. The limit
## goes to both shaders (rock_node_min_m): the threshold diameter is never below it, so a rock a node does not list
## stays in the ground's statistical layer. Budget: update() raises rock_threshold_scale smoothly while the measured GPU
## frame time is over the target (never a count cut) and lowers it back to 1 below it, logged.

## [E] Projected size (px) from which a rock is drawn as geometry (plan Global Constraints: 1.5 px).
const ROCK_PIXEL_THRESHOLD := 1.5
## [E, measured: Task 11 report] Rocks below this diameter are never instances; they stay in the ground's statistical
## layer (albedo share, cast shadows, lit faces).
const PEBBLE_GEOMETRY_LIMIT_M := 0.02
## [E] Nearest camera distance to a drawn node of a level, in node sizes (the split comes at LOD_FACTOR 2.5 x size from
## the bounding sphere; 2.0 leaves margin for a late child upload while the camera approaches).
const NODE_NEAR_FACTOR := 2.0
## [E] Farthest distance of a drawn node's rock from the camera in node sizes (the parent merges at MERGE_FACTOR x
## LOD_FACTOR x its size = ~5.8 x the node's size, plus the node's extent).
const NODE_FAR_FACTOR := 8.0
## The core's rock templates (RockTemplates::count) and LODs (RockTemplates::lods); one more LOD here: the bare
## icosahedron (the first 12 vertices of every icosphere), for rocks below LOD_MIN_PX[3].
const TEMPLATES := 32
const CORE_LODS := 4
const LODS := 5
## [E] Lower edge of each LOD's projected-diameter band (px; LOD 0 = 2562 vertices, LOD 4 = 12): the chord sagitta of the
## icosphere's edges stays below ~0.3 px at every band's lower edge (subdivision s: sagitta ~ (1.107 / 2^s / 2)^2 / 2 of
## the radius).
const LOD_MIN_PX: Array[float] = [100.0, 30.0, 10.0, 4.0, 0.0]
## Vertex count of LOD 0 (the template texture's width).
const MAX_VERTICES := 2562
const FLOATS_PER_INSTANCE := 20  ## transform 12, colour 4, custom 4
## [S] docs/subsystems/vr-performance.md: GPU budget 9 ms per frame in VR (11.1 ms frame at 90 Hz). [E] Desktop: 14 ms.
const GPU_TARGET_MS_VR := 9.0
const GPU_TARGET_MS_DESKTOP := 14.0
## [E] Budget control: the threshold scale's log changes at most this fast (1/s) per unit of relative overload; it
## relaxes back at RELAX_RATE (1/s) once the frame is RELAX_MARGIN below the target. At 1.5 px a 0.5/s change moves a
## rock's growth by < 1 % of the band per 90 Hz frame: no visible pop.
const SCALE_RATE := 0.5
const RELAX_RATE := 0.1
const RELAX_MARGIN := 0.1
const SCALE_MAX := 8.0
## [E] Log a scale change once it moved this factor since the last line.
const LOG_STEP := 1.1892  # 2^(1/4)
## [E] The visible counts take the node's nearest point this much closer (the camera moves within a frame, the
## counts lag one frame): a rock about to cross the threshold is already counted.
const VISIBLE_DISTANCE_MARGIN := 0.8
## [E] A node's counts are recomputed once the camera moved this fraction of its distance (within the margin's
## 20 %), and the loop stops after COUNTS_BUDGET_US of main-thread time per frame (round robin).
const COUNTS_MOVE_FRACTION := 0.05
const COUNTS_BUDGET_US := 500
const TEXTURE_DIR := "textures/moon_detail"
## Material parameters copied from the node's terrain material (the quadtree's horizon set and MoonDetailShading's).
const COPIED_NODE_PARAMETERS: Array[StringName] = [&"horizon_map", &"horizon_stride", &"horizon_max_rad",
		&"horizon_enabled", &"horizon_grid_scale", &"horizon_grid_offset", &"detail_cell_m", &"detail_horizon_spacing_m"]
const COPIED_TEMPLATE_PARAMETERS: Array[StringName] = [&"detail_shading", &"detail_total_slope_variance",
		&"detail_drawn_table", &"detail_drawn_axis", &"detail_rock_law", &"detail_rock_shape", &"detail_rock_lit_gain",
		&"detail_rock_abundance", &"albedo_texture", &"sun_radius_m", &"earth_radius_m"]

## Diagnostics switches (render_check): rocks drawn at all (false: every rock statistical, no lists built) and the
## budget control (false: rock_threshold_scale stays where it is - deterministic images).
static var geometry_enabled := true
static var budget_enabled := true

var core: StarsimCore
## Rad per pixel of the active render target (update(); read by the workers for the node limits).
var pixel_angle_rad := 1.0e-3
var threshold_scale := 1.0
## {nodes_built, rocks_built, build_ms (summed worker time of the rock lists), max_node_ms, max_node_rocks,
## attached_nodes, attached_instances, gpu_ms (last measured), main_ms (summed main-thread time of attach / detach)}
var stats := {"nodes_built": 0, "rocks_built": 0, "build_ms": 0.0, "max_node_ms": 0.0, "max_node_rocks": 0,
		"attached_nodes": 0, "attached_instances": 0, "gpu_ms": 0.0, "main_ms": 0.0}
var material_template: ShaderMaterial  ## the rock material every node duplicates (tests: the rock patch)
var meshes: Array[ArrayMesh] = []  ## per LOD
var top_z := PackedFloat32Array()  ## per template: the highest LOD-0 vertex z (unit template)
var _stats_mutex := Mutex.new()
var _logged_scale := 1.0
var _measuring := false
var _abundance: Image  ## the terrain's abundance map (detail_rock_abundance), read by the workers
var _cover_exponent := 0.0  ## the rock law's cover exponent q (detail_rock_law.y)
## Attached nodes: mesh instance id -> {instance, center (Moon-local render), half_diagonal, node_min_m, lods: [[MultiMesh,
## -D ascending, lod]]}: the per-frame visible counts.
var _entries := {}
## The same entries as a list (swap-removed) for the round-robin count update; the next entry to look at.
var _list: Array = []
var _cursor := 0


## Template meshes and textures from the core, the material template from the Moon terrain's (its detail uniforms,
## albedo map, light radii). Call after MoonDetailShading.configure.
func _init(starsim_core: StarsimCore, terrain_template: ShaderMaterial) -> void:
	core = starsim_core
	material_template = ShaderMaterial.new()
	material_template.shader = load("res://shaders/moon_rock.gdshader")
	for param in COPIED_TEMPLATE_PARAMETERS:
		material_template.set_shader_parameter(param, terrain_template.get_shader_parameter(param))
	_build_templates()
	material_template.set_shader_parameter("rock_top_z", top_z)
	var abundance: Texture2D = terrain_template.get_shader_parameter("detail_rock_abundance")
	_abundance = abundance.get_image() if abundance != null else null
	var law: Variant = terrain_template.get_shader_parameter("detail_rock_law")
	_cover_exponent = (law as Vector4).y if law is Vector4 else 0.0
	_load_textures()


## Meshes per LOD (unit icosphere directions; LOD 4: the icosahedron) and the template texture (normal, radius per
## vertex), from StarsimCore.rock_template_mesh.
func _build_templates() -> void:
	var texels := PackedFloat32Array()
	texels.resize(MAX_VERTICES * LODS * TEMPLATES * 4)
	texels.fill(0.0)
	top_z.resize(TEMPLATES)
	for lod in LODS:
		var core_lod := mini(lod, CORE_LODS - 1)
		var directions := PackedVector3Array()
		var indices := PackedInt32Array()
		for t in TEMPLATES:
			var mesh: Dictionary = core.rock_template_mesh(t, core_lod)
			var vertices: PackedVector3Array = mesh["vertices"]
			var normals: PackedVector3Array = mesh["normals"]
			var count := vertices.size() if lod < CORE_LODS else 12
			if t == 0:
				for k in count:
					directions.append(vertices[k].normalized())
				indices = mesh["indices"] if lod < CORE_LODS else _icosahedron_indices(mesh["indices"])
			for k in count:
				var i := ((lod * TEMPLATES + t) * MAX_VERTICES + k) * 4
				texels[i] = normals[k].x
				texels[i + 1] = normals[k].y
				texels[i + 2] = normals[k].z
				texels[i + 3] = vertices[k].length()
				if lod == 0:
					top_z[t] = maxf(top_z[t], vertices[k].z) if k > 0 else vertices[k].z
		var arrays := []
		arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX] = directions
		arrays[Mesh.ARRAY_NORMAL] = directions
		var white := PackedColorArray()
		white.resize(directions.size())
		white.fill(Color(1, 1, 1, 1))
		arrays[Mesh.ARRAY_COLOR] = white  # COLOR in the shader = the instance colour
		# The core's triangles are counter-clockwise seen from outside; Godot's front faces are clockwise.
		var clockwise := PackedInt32Array()
		clockwise.resize(indices.size())
		for k in range(0, indices.size(), 3):
			clockwise[k] = indices[k]
			clockwise[k + 1] = indices[k + 2]
			clockwise[k + 2] = indices[k + 1]
		arrays[Mesh.ARRAY_INDEX] = clockwise
		# CUSTOM0: the LOD row and its projected-diameter band (px), the same on every vertex.
		var band := PackedFloat32Array()
		for k in directions.size():
			band.append_array([float(lod), LOD_MIN_PX[lod], 1.0e30 if lod == 0 else LOD_MIN_PX[lod - 1], 0.0])
		arrays[Mesh.ARRAY_CUSTOM0] = band
		var array_mesh := ArrayMesh.new()
		array_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays, [], {},
				Mesh.ARRAY_CUSTOM_RGBA_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM0_SHIFT)
		meshes.append(array_mesh)
	var image := Image.create_from_data(MAX_VERTICES, LODS * TEMPLATES, false, Image.FORMAT_RGBAF,
			texels.to_byte_array())
	material_template.set_shader_parameter("rock_template_data", ImageTexture.create_from_image(image))


## The icosahedron's 20 faces from a subdivided icosphere's index list: subdivision splits face (a, b, c) into
## (a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca) in that order (core make_icosphere), so the original face's
## corners are the first index of the group's first three triangles; repeated subdivision keeps the order, so for
## subdivision s the corners sit 4^(s-1) triangles apart. Here s = 1 (LOD 3).
static func _icosahedron_indices(lod3: PackedInt32Array) -> PackedInt32Array:
	var out := PackedInt32Array()
	for f in lod3.size() / 12:
		out.append_array([lod3[12 * f], lod3[12 * f + 3], lod3[12 * f + 6]])
	return out


## The rock textures as two Texture2DArrays (relative albedo, normals); absent: untextured rocks (a warning).
func _load_textures() -> void:
	var dir := Paths.data_dir().path_join(TEXTURE_DIR)
	var albedo: Array[Image] = []
	var normal: Array[Image] = []
	for v in 4:
		var a := Image.load_from_file(dir.path_join("rock_albedo_%d.png" % v))
		var n := Image.load_from_file(dir.path_join("rock_normal_%d.png" % v))
		if a == null or n == null:
			push_warning("MoonRockLayer: Felstexturen fehlen (python tools/data_pipeline/fetch.py moon_detail) - Felsen "
					+ "ohne Textur")
			return
		a.convert(Image.FORMAT_R8)
		n.convert(Image.FORMAT_RGB8)
		a.generate_mipmaps()
		n.generate_mipmaps()
		albedo.append(a)
		normal.append(n)
	var albedo_array := Texture2DArray.new()
	albedo_array.create_from_images(albedo)
	var normal_array := Texture2DArray.new()
	normal_array.create_from_images(normal)
	material_template.set_shader_parameter("rock_albedo_maps", albedo_array)
	material_template.set_shader_parameter("rock_normal_maps", normal_array)
	material_template.set_shader_parameter("rock_texture_layers", albedo.size())
	material_template.set_shader_parameter("rock_textures", true)


## The node's list limit (m): ROCK_PIXEL_THRESHOLD pixels at NODE_NEAR_FACTOR x its size (0 for MAX_LEVEL leaves),
## at least PEBBLE_GEOMETRY_LIMIT_M.
static func node_min_diameter_m(level: int, size_m: float, pixel_angle: float, factor := 1.0) -> float:
	var near := 0.0 if level >= TerrainQuadtree.MAX_LEVEL else NODE_NEAR_FACTOR * size_m
	return maxf(PEBBLE_GEOMETRY_LIMIT_M, ROCK_PIXEL_THRESHOLD * pixel_angle * near * factor)


## The threshold's density factor (moon_detail.gdshaderinc detail_rock_abundance_factor): abundance^(1 / b) above the
## mean population (b = 2 - the cover exponent q), 1 at abundance <= 1 - the number of drawn rocks per pixel area
## stays the mean population's in boulder fields ([E] load rule: a Diviner maximum, abundance ~52, would list 30
## million rocks at 2 m and draw at 62 ms in the stereo mock without it).
func abundance_threshold_factor(abundance: float) -> float:
	return pow(abundance, 1.0 / (2.0 - _cover_exponent)) if abundance > 1.0 else 1.0


## The smallest abundance factor at the node's centre and corners (the abundance map, bilinear).
func node_abundance(key: Vector4i) -> float:
	if _abundance == null:
		return 1.0
	var lowest := INF
	var points := CubeSphere.node_corners(key)
	points.append(CubeSphere.node_center(key))
	for p in points:
		var ll := CubeSphere.lat_lon_deg(p)
		var x := (ll.y + 180.0) / 360.0 * _abundance.get_width() - 0.5
		var y := (90.0 - ll.x) / 180.0 * _abundance.get_height() - 0.5
		var x0 := floori(x)
		var y0 := clampi(floori(y), 0, _abundance.get_height() - 2)
		var fx := x - x0
		var fy := clampf(y - y0, 0.0, 1.0)
		var w := _abundance.get_width()
		var a := lerpf(_abundance.get_pixel(posmod(x0, w), y0).r, _abundance.get_pixel(posmod(x0 + 1, w), y0).r, fx)
		var b := lerpf(_abundance.get_pixel(posmod(x0, w), y0 + 1).r, _abundance.get_pixel(posmod(x0 + 1, w), y0 + 1).r, fx)
		lowest = minf(lowest, lerpf(a, b, fy))
	return lowest


## Worker: the node's rock list as MultiMesh buffers. {node_min_m, count, buffers: Array[PackedFloat32Array] per LOD,
## counts: PackedInt32Array, aabb (node-local render axes), ids: PackedInt64Array, build_ms}; {} without rocks.
func build_node(key: Vector4i, center_fixed: Vector3, radius_m: float) -> Dictionary:
	if not geometry_enabled:
		return {}
	var start_us := Time.get_ticks_usec()
	var size_m := CubeSphere.node_size_m(key, radius_m)
	var pixel_angle := pixel_angle_rad
	var factor := abundance_threshold_factor(node_abundance(key))
	var min_d := node_min_diameter_m(key.y, size_m, pixel_angle, factor)
	var data: Dictionary = core.moon_rocks_in_node(key.x, key.y, key.z, key.w, min_d)
	if data.has("error"):
		push_warning("MoonRockLayer: %s (%s)" % [data["error"], key])
	var ids: PackedInt64Array = data["ids"]
	var n := ids.size()
	if n == 0:
		return {"node_min_m": min_d, "count": 0}
	var base: PackedFloat64Array = data["base_fixed_m"]
	var basis: PackedFloat64Array = data["basis"]
	var axes: PackedFloat64Array = data["axes_m"]
	var diameter: PackedFloat64Array = data["diameter_m"]
	var burial: PackedFloat64Array = data["burial_m"]
	var albedo: PackedFloat64Array = data["albedo"]
	var templates: PackedInt32Array = data["template"]
	var near := 0.0 if key.y >= TerrainQuadtree.MAX_LEVEL else NODE_NEAR_FACTOR * size_m
	var far := NODE_FAR_FACTOR * size_m
	var face := key.x
	var fn: Vector3 = CubeSphere.FACE_NORMAL[face]
	var fu: Vector3 = CubeSphere.FACE_U[face]
	var fv: Vector3 = CubeSphere.FACE_V[face]
	var cells := float(TerrainNodeBuilder.CELLS)
	var per_face := float(1 << key.y)
	var center_lon := rad_to_deg(atan2(center_fixed.y, center_fixed.x))
	var buffers: Array[PackedFloat32Array] = []
	var counts := PackedInt32Array()
	var minus_d: Array[PackedFloat32Array] = []  # per LOD: -D in buffer order (ascending: the core sorts D down)
	for lod in LODS:
		buffers.append(PackedFloat32Array())
		counts.append(0)
		minus_d.append(PackedFloat32Array())
	var lo := Vector3(INF, INF, INF)
	var hi := -lo
	var instance := PackedFloat32Array()
	instance.resize(FLOATS_PER_INSTANCE)
	for r in n:
		var b3 := 3 * r
		var b9 := 9 * r
		var up := Vector3(basis[b9 + 6], basis[b9 + 7], basis[b9 + 8])
		var c := axes[b3 + 2]
		var base_point := Vector3(base[b3], base[b3 + 1], base[b3 + 2])
		var o := base_point - up * burial[r] - center_fixed
		var ea := Vector3(basis[b9], basis[b9 + 1], basis[b9 + 2]) * axes[b3]
		var eb := Vector3(basis[b9 + 3], basis[b9 + 4], basis[b9 + 5]) * axes[b3 + 1]
		var ec := up * c
		# Render axes (x, y, z) -> (y, z, x); MultiMesh buffer rows: basis row k then origin k.
		instance[0] = ea.y
		instance[1] = eb.y
		instance[2] = ec.y
		instance[3] = o.y
		instance[4] = ea.z
		instance[5] = eb.z
		instance[6] = ec.z
		instance[7] = o.z
		instance[8] = ea.x
		instance[9] = eb.x
		instance[10] = ec.x
		instance[11] = o.x
		# Base: the node's vertex-grid position (the horizon map) and the albedo map's uv (continuous in the node).
		var dir := base_point.normalized()
		var major := dir.dot(fn)
		var s := atan(dir.dot(fu) / major) * (4.0 / PI)
		var t := atan(dir.dot(fv) / major) * (4.0 / PI)
		instance[12] = ((s + 1.0) * 0.5 * per_face - key.z) * cells
		instance[13] = ((t + 1.0) * 0.5 * per_face - key.w) * cells
		var lon := rad_to_deg(atan2(dir.y, dir.x))
		lon = center_lon + wrapf(lon - center_lon, -180.0, 180.0)
		instance[14] = (lon + 180.0) / 360.0
		instance[15] = (90.0 - rad_to_deg(asin(clampf(dir.z, -1.0, 1.0)))) / 180.0
		var d := diameter[r]
		var template_id := templates[r]
		instance[16] = d
		instance[17] = top_z[template_id] - burial[r] / c
		instance[18] = albedo[r]
		instance[19] = template_id
		# LOD bands this rock can reach while the node is drawn.
		var px_max := INF if near <= 0.0 else d / (pixel_angle * near)
		var px_min := d / (pixel_angle * far)
		for lod in LODS:
			var band_lo: float = LOD_MIN_PX[lod]
			var band_hi: float = INF if lod == 0 else LOD_MIN_PX[lod - 1]
			if px_max >= band_lo and px_min < band_hi:
				buffers[lod].append_array(instance)
				counts[lod] += 1
				minus_d[lod].append(-d)
		var reach := Vector3.ONE * axes[b3]
		var origin := Vector3(o.y, o.z, o.x)
		lo = lo.min(origin - reach)
		hi = hi.max(origin + reach)
	var ms := (Time.get_ticks_usec() - start_us) * 1.0e-3
	_stats_mutex.lock()
	stats["nodes_built"] += 1
	stats["rocks_built"] += n
	stats["build_ms"] += ms
	stats["max_node_ms"] = maxf(stats["max_node_ms"], ms)
	stats["max_node_rocks"] = maxi(stats["max_node_rocks"], n)
	_stats_mutex.unlock()
	return {"node_min_m": min_d, "count": n, "buffers": buffers, "counts": counts, "aabb": AABB(lo, hi - lo),
			"ids": ids, "build_ms": ms, "minus_d": minus_d, "factor": factor}


## Main thread, at the node's upload: its MultiMeshInstance3Ds as children of the node's mesh instance (they show and
## hide with it), the node material's horizon and detail parameters copied, the list limit set on both materials.
func attach(instance: MeshInstance3D, rocks: Dictionary, terrain_material: ShaderMaterial) -> void:
	var start_us := Time.get_ticks_usec()
	var min_d: float = rocks.get("node_min_m", 1.0e30) if not rocks.is_empty() else 1.0e30
	var count: int = rocks.get("count", 0)
	terrain_material.set_shader_parameter("rock_node_min_m", min_d if count > 0 else 1.0e30)
	if count == 0:
		return
	var material: ShaderMaterial = material_template.duplicate()
	for param in COPIED_NODE_PARAMETERS:
		material.set_shader_parameter(param, terrain_material.get_shader_parameter(param))
	material.set_shader_parameter("rock_node_min_m", min_d)
	material.set_shader_parameter("node_center_local", instance.position)
	var t_material := Time.get_ticks_usec()
	var t_mm := 0
	var t_node := 0
	var buffers: Array = rocks["buffers"]
	var counts: PackedInt32Array = rocks["counts"]
	var aabb: AABB = rocks["aabb"]
	var minus_d: Array = rocks["minus_d"]
	var entry := {"instance": instance, "center": instance.position + aabb.get_center(), "factor": rocks["factor"],
			"half_diagonal": 0.5 * aabb.size.length(), "node_min_m": min_d, "lods": []}
	for lod in LODS:
		if counts[lod] == 0:
			continue
		var t0 := Time.get_ticks_usec()
		var multimesh := MultiMesh.new()
		multimesh.transform_format = MultiMesh.TRANSFORM_3D
		multimesh.use_colors = true
		multimesh.use_custom_data = true
		multimesh.mesh = meshes[lod]
		multimesh.instance_count = counts[lod]
		multimesh.buffer = buffers[lod]
		multimesh.custom_aabb = aabb
		var t1 := Time.get_ticks_usec()
		t_mm += t1 - t0
		var node := MultiMeshInstance3D.new()
		node.multimesh = multimesh
		node.material_override = material
		node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		node.set_meta(&"moon_rocks", true)
		instance.add_child(node)
		t_node += Time.get_ticks_usec() - t1
		(entry["lods"] as Array).append([multimesh, minus_d[lod], lod])
		stats["attached_instances"] += counts[lod]
	stats["attached_nodes"] += 1
	var total_ms := (Time.get_ticks_usec() - start_us) * 1.0e-3
	if total_ms > float(stats.get("max_attach_ms", 0.0)):
		stats["max_attach_ms"] = total_ms
		stats["max_attach_parts_ms"] = [(t_material - start_us) * 1.0e-3, t_mm * 1.0e-3, t_node * 1.0e-3]
	entry["camera"] = Vector3(1.0e30, 1.0e30, 1.0e30)
	entry["near"] = 0.0
	entry["threshold"] = -1.0
	entry["index"] = _list.size()
	_list.append(entry)
	_entries[instance.get_instance_id()] = entry
	stats["main_ms"] += (Time.get_ticks_usec() - start_us) * 1.0e-3


## Main thread, at the node's release: its rock instances removed (the mesh instance returns to the pool).
func detach(instance: MeshInstance3D) -> void:
	var start_us := Time.get_ticks_usec()
	var entry: Dictionary = _entries.get(instance.get_instance_id(), {})
	if not entry.is_empty():
		var index: int = entry["index"]
		var last: Dictionary = _list.back()
		_list[index] = last
		last["index"] = index
		_list.pop_back()
		_entries.erase(instance.get_instance_id())
	for child in instance.get_children():
		if child.has_meta(&"moon_rocks"):
			var mmi := child as MultiMeshInstance3D
			stats["attached_instances"] -= mmi.multimesh.instance_count
			instance.remove_child(child)
			child.queue_free()
	stats["main_ms"] += (Time.get_ticks_usec() - start_us) * 1.0e-3


## The rock MultiMeshes attached to a quadtree node instance (tests; the LOD is the mesh's CUSTOM0.x).
static func attached(instance: MeshInstance3D) -> Array:
	var out := []
	for child in instance.get_children():
		if child.has_meta(&"moon_rocks"):
			out.append((child as MultiMeshInstance3D).multimesh)
	return out


## Per frame (World, after the Moon terrain's update): the pixel angle of the active render target, the budget control,
## the globals moon_rock_pixel_angle / moon_rock_threshold_px / moon_world_to_local.
func update(viewport: Viewport, moon_local_to_world: Basis, delta_s: float, camera_fixed_m := Vector3.ZERO) -> void:
	pixel_angle_rad = pixel_angle(viewport)
	var xr := viewport.use_xr
	if budget_enabled and geometry_enabled:
		if not _measuring:
			RenderingServer.viewport_set_measure_render_time(viewport.get_viewport_rid(), true)
			_measuring = true
		var gpu_ms := RenderingServer.viewport_get_measured_render_time_gpu(viewport.get_viewport_rid())
		stats["gpu_ms"] = gpu_ms
		threshold_scale = budget_step(threshold_scale, gpu_ms, GPU_TARGET_MS_VR if xr else GPU_TARGET_MS_DESKTOP,
				delta_s)
		if threshold_scale > _logged_scale * LOG_STEP or threshold_scale < _logged_scale / LOG_STEP:
			print("[MoonRockLayer] rock_threshold_scale %.2f (GPU %.2f ms, target %.1f ms)" % [threshold_scale, gpu_ms,
					GPU_TARGET_MS_VR if xr else GPU_TARGET_MS_DESKTOP])
			_logged_scale = threshold_scale
	ShaderGlobals.set_value(&"moon_rock_pixel_angle", pixel_angle_rad)
	ShaderGlobals.set_value(&"moon_rock_threshold_px",
			ROCK_PIXEL_THRESHOLD * threshold_scale if geometry_enabled else 0.0)
	ShaderGlobals.set_value(&"moon_world_to_local", moon_local_to_world.orthonormalized().transposed())
	var start_us := Time.get_ticks_usec()
	_update_visible_counts(Frames.to_render(camera_fixed_m))
	stats["counts_ms"] = (Time.get_ticks_usec() - start_us) * 1.0e-3


## Per drawn node and LOD: the MultiMesh draws only the prefix of its (diameter-sorted) list that can be geometry this
## frame - D at least the threshold diameter at the node's nearest point (VISIBLE_DISTANCE_MARGIN closer) and, for LOD
## k, at least its band's lower edge there. The shader still decides per rock (growth, band); the count only spares the
## vertex work of instances it would collapse. Hidden nodes keep their counts.
func _update_visible_counts(camera_render: Vector3) -> void:
	var per_px := pixel_angle_rad * VISIBLE_DISTANCE_MARGIN
	var threshold_rad := ROCK_PIXEL_THRESHOLD * threshold_scale * per_px if geometry_enabled else INF
	var n := _list.size()
	var start_us := Time.get_ticks_usec()
	for k in n:
		var entry: Dictionary = _list[(_cursor + k) % n]
		if k % 32 == 31 and Time.get_ticks_usec() - start_us > COUNTS_BUDGET_US:
			_cursor = (_cursor + k) % n  # continue here next frame
			return
		var instance: MeshInstance3D = entry["instance"]
		if not instance.visible:
			continue
		var last_near: float = entry["near"]
		if entry["threshold"] == threshold_rad and (entry["camera"] as Vector3).distance_squared_to(camera_render) \
				< COUNTS_MOVE_FRACTION * COUNTS_MOVE_FRACTION * last_near * last_near:
			continue  # the camera moved less than COUNTS_MOVE_FRACTION of the node's distance: counts still valid
		var near := maxf((entry["center"] as Vector3).distance_to(camera_render) - float(entry["half_diagonal"]), 0.0)
		entry["camera"] = camera_render
		entry["near"] = near
		entry["threshold"] = threshold_rad
		var cut := maxf(threshold_rad * near * float(entry["factor"]), float(entry["node_min_m"]))
		for item: Array in entry["lods"]:
			var multimesh: MultiMesh = item[0]
			var minus_d: PackedFloat32Array = item[1]
			var count := minus_d.bsearch(-maxf(cut, LOD_MIN_PX[int(item[2])] * per_px * near), false)
			if multimesh.visible_instance_count != count:
				multimesh.visible_instance_count = count



## One budget-control step: the scale's log rises by SCALE_RATE x the relative overload per second, falls by RELAX_RATE
## per second once the GPU time is RELAX_MARGIN below the target; [1, SCALE_MAX]. gpu_ms <= 0 (not measured yet): kept.
static func budget_step(scale: float, gpu_ms: float, target_ms: float, delta_s: float) -> float:
	if gpu_ms <= 0.0 or delta_s <= 0.0:
		return scale
	var over := (gpu_ms - target_ms) / target_ms
	if over > 0.0:
		scale *= exp(SCALE_RATE * over * delta_s)
	elif over < -RELAX_MARGIN:
		scale *= exp(-RELAX_RATE * delta_s)
	return clampf(scale, 1.0, SCALE_MAX)


## Rad per pixel of the 3D view: the XR interface's render target and view-0 projection when the viewport renders XR,
## else the viewport camera's projection over the 3D render height.
static func pixel_angle(viewport: Viewport) -> float:
	if viewport.use_xr and XRServer.primary_interface != null and XRServer.primary_interface.is_initialized():
		var interface := XRServer.primary_interface
		var size := interface.get_render_target_size()
		var projection := interface.get_projection_for_view(0, size.x / size.y, 0.05, 1000.0)
		return 2.0 / (projection.y.y * size.y)
	var camera := viewport.get_camera_3d()
	if camera == null:
		return 1.0e-3
	var height := viewport.get_visible_rect().size.y * viewport.scaling_3d_scale
	return 2.0 / (camera.get_camera_projection().y.y * maxf(height, 1.0))
