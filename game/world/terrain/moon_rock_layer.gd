class_name MoonRockLayer
extends RefCounted
## The Moon's rocks drawn as instanced meshes by projected size (moon procedural surface plan Task 11; shader
## moon_rock.gdshader, docs/subsystems/moon-surface.md "Rocks drawn"). No physics here: the rocks are the core's
## (StarsimCore.moon_rocks_in_node, the same LunarRocks the contact and the touchdown checks query); this class only
## lays them out for the GPU.
##
## Per terrain node (TerrainNodeBuilder.build calls build_node on its worker): the rocks whose base lies in the node
## with D >= the node's list limit, packed into one MultiMesh buffer per template LOD (transform relative to the node
## centre, colour = base grid position + albedo-map uv, custom = D, exposed height, albedo factor, template), each
## buffer zero-padded on the worker to its pool bucket's capacity. A rock goes into every LOD whose projected-diameter
## band it can reach while the node is drawn; the shader draws it only in the LOD whose band holds its current size (one
## reference point: the camera / head centre), and only from the threshold diameter up (growth band to 2 x).
##
## Main thread (TerrainQuadtree): attach() at the node's upload only queues the node (its terrain material keeps
## rock_node_min_m = 1e30: every rock statistical, radiance-continuous); process_queue() (each quadtree update) attaches
## queued nodes within ATTACH_BUDGET_US per frame, taking preallocated MultiMeshes from a pool of power-of-two capacity
## buckets (a buffer upload, no allocation; at most ALLOCATIONS_PER_FRAME new pool entries per frame) and pooled
## materials; the quadtree shows a node's children only once their rocks are attached (no rock pops at a split).
## detach() at the node's release returns everything to the pools.
##
## The node's list limit: rocks smaller than ROCK_PIXEL_THRESHOLD pixels x the abundance load factor at the nearest
## distance the camera can have to a drawn leaf of that level (NODE_NEAR_FACTOR x its size: it splits at
## TerrainQuadtree.LOD_FACTOR x size) can never be geometry there; the leaf under the camera (MAX_LEVEL) lists every rock
## from PEBBLE_GEOMETRY_LIMIT_M. The limit goes to both shaders (rock_node_min_m): the threshold diameter is never below
## it, so a rock a node does not list stays in the ground's statistical layer.
##
## Threshold: ROCK_PIXEL_THRESHOLD x max(rock_threshold_scale, abundance load factor) pixels. The load factor is the
## static floor of the control (list memory and worker load in boulder fields); rock_threshold_scale is the GPU floor
## guard (user 2026-11-05: on the Moon quality first - the scale rises only while the GPU frame is over GPU_FLOOR_MS and
## the rocks' own estimated cost can bring it back; terrain overload is logged, not compensated). Both logged.

## [E] Projected size (px) from which a rock is drawn as geometry (plan Global Constraints: 1.5 px).
const ROCK_PIXEL_THRESHOLD := 1.5
## [E, measured: Task 11 report] Rocks below this diameter are never instances; they stay in the ground's statistical
## layer (albedo share, cast shadows, lit faces) and, as discrete clasts, in the regolith texture relief (the pebble
## bumps of MoonDetailShading's finest scale).
const PEBBLE_GEOMETRY_LIMIT_M := 0.02
## [E] Nearest camera distance to a drawn node of a level, in node sizes (the split comes at LOD_FACTOR 2.5 x size from
## the bounding sphere; 2.0 leaves margin for a late child upload while the camera approaches).
const NODE_NEAR_FACTOR := 2.0
## [E] Farthest distance of a drawn node's rock from the camera in node sizes (the parent merges at MERGE_FACTOR x
## LOD_FACTOR x its size = ~5.8 x the node's size, plus the node's extent).
const NODE_FAR_FACTOR := 8.0
## The core's rock templates (RockTemplates::count, max_planes) and LODs (RockTemplates::lods); one more LOD here: the
## bare icosahedron (the first 12 vertices of every icosphere), for rocks below LOD_MIN_PX[3].
const TEMPLATES := 32
const MAX_PLANES := 24
const CORE_LODS := 4
const LODS := 5
## [E] Lower edge of each LOD's projected-diameter band (px; LOD 0 = 2562 vertices, LOD 4 = 12): the chord sagitta of the
## icosphere's edges stays below ~0.3 px at every band's lower edge (subdivision s: sagitta ~ (1.107 / 2^s / 2)^2 / 2 of
## the radius).
const LOD_MIN_PX: Array[float] = [100.0, 30.0, 10.0, 4.0, 0.0]
const LOD_VERTICES: Array[int] = [2562, 642, 162, 42, 12]
## Vertex count of LOD 0 (the template texture's width).
const MAX_VERTICES := 2562
const FLOATS_PER_INSTANCE := 20  ## transform 12, colour 4, custom 4
## Texels per template row of rock_shape_data: MAX_PLANES planes (n, h), 3 quadric rows, (plane count, bottom, top, 0).
const SHAPE_TEXELS := MAX_PLANES + 4
## Samples of the plan-area tables (rock_area_data): forward A(z) / A_max over [bottom_z, top_z], inverse z(a) over
## a in [0, 1].
const AREA_SAMPLES := 64
## [S] User decision 2026-11-05 (moon procedural surface plan, Task 11 fix brief): on the Moon below 90 fps is
## acceptable; the rock budget control acts only as a floor guard at 22.2 ms GPU (45 fps, the motion-smoothing floor).
const GPU_FLOOR_MS := 22.2
## [E] Budget control: the threshold scale's log changes at most this fast (1/s) per unit of relative overload; it
## relaxes back at RELAX_RATE (1/s) once the frame is RELAX_MARGIN below the floor. At 1.5 px a 0.5/s change moves a
## rock's growth by < 1 % of the band per 90 Hz frame: no visible pop.
const SCALE_RATE := 0.5
const RELAX_RATE := 0.1
const RELAX_MARGIN := 0.1
const SCALE_MAX := 8.0
## [E, measured: Task 11 fix B report, stereo mock] GPU time per drawn rock vertex (the visible-count prefix x the
## LOD's vertex count): the rocks' own share of the frame for the budget control. Measured 0.14 ns at IM-2 2 m (+0.63 ms
## for 4.5 M vertices) and 0.21 ns in the Diviner boulder field (+20.4 ms for 97 M); the higher value, so a frame the
## rocks could fix is never mistaken for terrain cost.
const ROCK_NS_PER_VERTEX := 0.21
## [E] Log a scale / load-factor change once it moved this factor since the last line.
const LOG_STEP := 1.1892  # 2^(1/4)
## [E] The visible counts take the node's nearest point this much closer (the camera moves within a frame, the
## counts lag one frame): a rock about to cross the threshold is already counted.
const VISIBLE_DISTANCE_MARGIN := 0.8
## [E] A node's counts are recomputed once the camera moved this fraction of its distance (within the margin's
## 20 %), and the loop stops after COUNTS_BUDGET_US of main-thread time per frame (round robin).
const COUNTS_MOVE_FRACTION := 0.05
const COUNTS_BUDGET_US := 500
## [E] Main-thread budget of the attach queue per frame (the camera-jump gate is 5 ms for the whole rock layer), and new
## pool MultiMeshes per frame (a fresh GPU buffer; the rest of the attaches only upload into pooled ones).
const ATTACH_BUDGET_US := 2000
const ALLOCATIONS_PER_FRAME := 12
## Pool buckets per LOD: capacities BUCKET_MIN x 2^k up to BUCKET_MAX instances; larger LOD lists get an exact,
## unpooled MultiMesh (counted in stats.unpooled). Key = LOD x POOL_KEY_PER_LOD + capacity (pool_key).
const BUCKET_MIN := 64
const BUCKET_MAX := 16384
const POOL_KEY_PER_LOD := 1000000
## [E, measured: Task 11 fix B report, IM-2 2 m camera jump] Pool entries allocated at start-up per pool key (about the
## peak use after a 10 km jump; the pool grows beyond on demand within the attach budget, ~0.15 ms per allocation).
const PREWARM := {64: 290, 1000064: 1160, 2000064: 1660, 3000064: 460, 3000128: 620, 3000256: 570, 3000512: 160,
		4000064: 200, 4000128: 80, 4000256: 170, 4000512: 320, 4001024: 560, 4002048: 680}
const PREWARM_MATERIALS := 1500
## [E] Main-thread time per frame the pool warming may take (process_queue, before the attaches).
const PREWARM_BUDGET_US := 1500
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
## attached_nodes, attached_instances, gpu_ms (last measured), main_ms (summed main-thread time of attach / detach /
## queue), queued, pool_instances / pool_mb (allocated pool capacity), allocations, unpooled, rock_gpu_ms_estimate,
## load_factor (at the camera), terrain_over_floor (frames the frame was over the floor without the rocks)}
var stats := {"nodes_built": 0, "rocks_built": 0, "build_ms": 0.0, "max_node_ms": 0.0, "max_node_rocks": 0,
		"attached_nodes": 0, "attached_instances": 0, "gpu_ms": 0.0, "main_ms": 0.0, "queued": 0, "pool_instances": 0,
		"pool_mb": 0.0, "allocations": 0, "unpooled": 0, "rock_gpu_ms_estimate": 0.0, "load_factor": 1.0,
		"terrain_over_floor": 0, "bucket_peak": {}}
var material_template: ShaderMaterial  ## the rock material the pooled node materials copy (tests: the rock patch)
var meshes: Array[ArrayMesh] = []  ## per LOD
var top_z := PackedFloat32Array()  ## per template: the highest LOD-0 vertex z (unit template)
var _stats_mutex := Mutex.new()
var _logged_scale := 1.0
var _logged_factor := 1.0
var _logged_terrain := false
var _measuring := false
var _abundance: Image  ## the terrain's abundance map (detail_rock_abundance), read by the workers
var _cover_exponent := 0.0  ## the rock law's cover exponent q (detail_rock_law.y)
## Attached nodes: mesh instance id -> {instance, center (Moon-local render), half_diagonal, node_min_m, factor,
## material, lods: [[MultiMesh, -D ascending, lod, MultiMeshInstance3D, capacity]], work (drawn vertices)}.
var _entries := {}
## The same entries as a list (swap-removed) for the round-robin count update; the next entry to look at.
var _list: Array = []
var _cursor := 0
var _cycle_work := 0  ## drawn vertices summed over the visible entries in the current round-robin cycle
var _drawn_vertices := 0  ## the last full cycle's sum
## Attach queue: mesh instance id -> [instance, rocks, terrain material] (FIFO order in _queue_order).
var _queue := {}
var _queue_order: Array[int] = []
## Pools: capacity -> Array of free MultiMeshInstance3D; free materials.
var _free := {}
var _free_materials: Array[ShaderMaterial] = []
var _in_use := {}  ## pool key -> pooled MultiMeshes attached now
var _prewarm_left := {}  ## pool key -> entries still to allocate (_prewarm_step)
var _prewarm_materials_left := 0
var _allocations_this_frame := 0


## Template meshes and textures from the core, the material template from the Moon terrain's (its detail uniforms,
## albedo map, light radii), the pools warmed. Call after MoonDetailShading.configure.
func _init(starsim_core: StarsimCore, terrain_template: ShaderMaterial, prewarm := true) -> void:
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
	if prewarm and geometry_enabled:
		_prewarm_left = PREWARM.duplicate()
		_prewarm_materials_left = PREWARM_MATERIALS


## Warms the pools off the critical path (process_queue, once the Moon terrain is active): pool entries and materials
## until PREWARM_BUDGET_US of this frame is spent. True while some are left.
func _prewarm_step() -> bool:
	if _prewarm_left.is_empty() and _prewarm_materials_left <= 0:
		return false
	var start_us := Time.get_ticks_usec()
	while Time.get_ticks_usec() - start_us < PREWARM_BUDGET_US:
		if _prewarm_materials_left > 0:
			_free_materials.append(material_template.duplicate())
			_prewarm_materials_left -= 1
			continue
		if _prewarm_left.is_empty():
			break
		var key: int = _prewarm_left.keys()[0]
		_release_to_pool(_allocate(key / POOL_KEY_PER_LOD, key % POOL_KEY_PER_LOD), key)
		stats["prewarm_allocations"] = int(stats.get("prewarm_allocations", 0)) + 1
		stats["allocations"] -= 1  # counted apart from the on-demand ones
		_prewarm_left[key] = int(_prewarm_left[key]) - 1
		if int(_prewarm_left[key]) <= 0:
			_prewarm_left.erase(key)
	stats["main_ms"] += (Time.get_ticks_usec() - start_us) * 1.0e-3
	return true


## Meshes per LOD (unit icosphere directions; LOD 4: the icosahedron), the template texture (normal, radius per vertex),
## the shape texture (the convex body's constraints: exact facet normals per fragment) and the plan-area tables (the
## growth band's rise), from StarsimCore.rock_template_mesh / rock_template_exposed_area.
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
	_build_shape_tables()


## rock_shape_data (RGBAF, SHAPE_TEXELS x TEMPLATES): per face (n, h) - unused faces never active -, the quadric's rows,
## (plane count, bottom_z, top_z, 0). rock_area_data (RGF, AREA_SAMPLES x TEMPLATES): R = the exposed plan area A(z) /
## A_max at z = bottom + (top - bottom) k / (AREA_SAMPLES - 1), G = its inverse, the highest z with A(z) >= a A_max at
## a = k / (AREA_SAMPLES - 1) (A is non-increasing in z).
func _build_shape_tables() -> void:
	var shape := PackedFloat32Array()
	shape.resize(SHAPE_TEXELS * TEMPLATES * 4)
	shape.fill(0.0)
	var area := PackedFloat32Array()
	area.resize(AREA_SAMPLES * TEMPLATES * 2)
	for t in TEMPLATES:
		var mesh: Dictionary = core.rock_template_mesh(t, 0)
		var planes: PackedFloat64Array = mesh["planes"]
		var quadric: PackedFloat64Array = mesh["quadric"]
		var row := t * SHAPE_TEXELS * 4
		var count := planes.size() / 4
		for k in MAX_PLANES:
			var i := row + 4 * k
			if k < count:
				for c in 4:
					shape[i + c] = planes[4 * k + c]
			else:
				shape[i + 3] = 1.0e9  # 0 . p <= 1e9: never the active constraint
		for r in 3:
			for c in 3:
				shape[row + 4 * (MAX_PLANES + r) + c] = quadric[3 * r + c]
		var bottom: float = mesh["bottom_z"]
		var top: float = mesh["top_z"]
		shape[row + 4 * (MAX_PLANES + 3)] = count
		shape[row + 4 * (MAX_PLANES + 3) + 1] = bottom
		shape[row + 4 * (MAX_PLANES + 3) + 2] = top
		var forward := PackedFloat64Array()
		for k in AREA_SAMPLES:
			forward.append(core.rock_template_exposed_area(t, bottom + (top - bottom) * k / (AREA_SAMPLES - 1.0)))
		var widest := forward[0]
		for k in AREA_SAMPLES:
			forward[k] = minf(forward[k] / widest, 1.0) if widest > 0.0 else 0.0
			if k > 0:
				forward[k] = minf(forward[k], forward[k - 1])  # non-increasing (the column grid's noise)
		for k in AREA_SAMPLES:
			var a := float(k) / (AREA_SAMPLES - 1.0)
			area[2 * (t * AREA_SAMPLES + k)] = forward[k]
			area[2 * (t * AREA_SAMPLES + k) + 1] = _inverse_area(forward, a, bottom, top)
	var shape_image := Image.create_from_data(SHAPE_TEXELS, TEMPLATES, false, Image.FORMAT_RGBAF, shape.to_byte_array())
	material_template.set_shader_parameter("rock_shape_data", ImageTexture.create_from_image(shape_image))
	var area_image := Image.create_from_data(AREA_SAMPLES, TEMPLATES, false, Image.FORMAT_RGF, area.to_byte_array())
	material_template.set_shader_parameter("rock_area_data", ImageTexture.create_from_image(area_image))


## The highest z in [bottom, top] with A(z) / A_max >= a on the forward table (linear between samples).
static func _inverse_area(forward: PackedFloat64Array, a: float, bottom: float, top: float) -> float:
	var n := forward.size()
	if a <= forward[n - 1]:
		return top
	for k in range(n - 2, -1, -1):
		if forward[k] >= a:
			var f := (forward[k] - a) / maxf(forward[k] - forward[k + 1], 1.0e-12)
			return bottom + (top - bottom) * (k + clampf(f, 0.0, 1.0)) / (n - 1.0)
	return bottom


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


## The node's list limit (m): ROCK_PIXEL_THRESHOLD pixels x the load factor at NODE_NEAR_FACTOR x its size (0 for
## MAX_LEVEL leaves), at least PEBBLE_GEOMETRY_LIMIT_M. The threshold is ROCK_PIXEL_THRESHOLD x max(scale, factor) >= this
## at every distance the node is drawn from.
static func node_min_diameter_m(level: int, size_m: float, pixel_angle: float, factor := 1.0) -> float:
	var near := 0.0 if level >= TerrainQuadtree.MAX_LEVEL else NODE_NEAR_FACTOR * size_m
	return maxf(PEBBLE_GEOMETRY_LIMIT_M, ROCK_PIXEL_THRESHOLD * pixel_angle * near * factor)


## The abundance load factor (moon_detail.gdshaderinc detail_rock_abundance_factor): abundance^(1 / b) above the mean
## population (b = 2 - the cover exponent q), 1 at abundance <= 1 - the number of drawn rocks per pixel area stays the
## mean population's in boulder fields ([E] load rule: a Diviner maximum, abundance ~52, would list 30 million rocks at
## 2 m and draw at 62 ms in the stereo mock without it). The static floor of the threshold control: the effective
## threshold is ROCK_PIXEL_THRESHOLD x max(rock_threshold_scale, factor).
func abundance_threshold_factor(abundance: float) -> float:
	return pow(abundance, 1.0 / (2.0 - _cover_exponent)) if abundance > 1.0 else 1.0


## The abundance map (bilinear) at a Moon-fixed direction.
func abundance_at(dir: Vector3) -> float:
	if _abundance == null:
		return 1.0
	var ll := CubeSphere.lat_lon_deg(dir)
	var w := _abundance.get_width()
	var x := (ll.y + 180.0) / 360.0 * w - 0.5
	var y := (90.0 - ll.x) / 180.0 * _abundance.get_height() - 0.5
	var x0 := floori(x)
	var y0 := clampi(floori(y), 0, _abundance.get_height() - 2)
	var fx := x - x0
	var fy := clampf(y - y0, 0.0, 1.0)
	var a := lerpf(_abundance.get_pixel(posmod(x0, w), y0).r, _abundance.get_pixel(posmod(x0 + 1, w), y0).r, fx)
	var b := lerpf(_abundance.get_pixel(posmod(x0, w), y0 + 1).r, _abundance.get_pixel(posmod(x0 + 1, w), y0 + 1).r, fx)
	return lerpf(a, b, fy)


## The smallest abundance factor over the node: its centre and corners, and every abundance-map pixel centre inside its
## lat/lon box (review M1: a local minimum inside a large node must not fall below the list's cut).
func node_abundance(key: Vector4i) -> float:
	if _abundance == null:
		return 1.0
	var points := CubeSphere.node_corners(key)
	points.append(CubeSphere.node_center(key))
	var lowest := INF
	var lat_lo := INF
	var lat_hi := -INF
	var lon_lo := INF
	var lon_hi := -INF
	var center_lon := CubeSphere.lat_lon_deg(CubeSphere.node_center(key)).y
	for p in points:
		lowest = minf(lowest, abundance_at(p))
		var ll := CubeSphere.lat_lon_deg(p)
		var lon := center_lon + wrapf(ll.y - center_lon, -180.0, 180.0)
		lat_lo = minf(lat_lo, ll.x)
		lat_hi = maxf(lat_hi, ll.x)
		lon_lo = minf(lon_lo, lon)
		lon_hi = maxf(lon_hi, lon)
	var w := _abundance.get_width()
	var h := _abundance.get_height()
	var i0 := ceili((lon_lo + 180.0) / 360.0 * w - 0.5)
	var i1 := floori((lon_hi + 180.0) / 360.0 * w - 0.5)
	var j0 := maxi(ceili((90.0 - lat_hi) / 180.0 * h - 0.5), 0)
	var j1 := mini(floori((90.0 - lat_lo) / 180.0 * h - 0.5), h - 1)
	if (i1 - i0 + 1) * (j1 - j0 + 1) <= 4096:  # coarse nodes (no rock lists that matter) skip the scan
		for j in range(j0, j1 + 1):
			for i in range(i0, i1 + 1):
				lowest = minf(lowest, _abundance.get_pixel(posmod(i, w), j).r)
	return lowest


## Pool bucket capacity for a LOD list of n instances (0: unpooled, n itself).
static func bucket_capacity(n: int) -> int:
	if n > BUCKET_MAX:
		return 0
	var capacity := BUCKET_MIN
	while capacity < n:
		capacity *= 2
	return capacity


## Worker: the node's rock list as MultiMesh buffers. {node_min_m, count, buffers: Array[PackedFloat32Array] per LOD
## (zero-padded to capacities[lod] instances), counts, capacities: PackedInt32Array, aabb (node-local render axes), ids,
## build_ms, minus_d, factor}; {} without the geometry.
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
	# Zero-pad each list to its pool bucket (the main thread only uploads; zero transforms draw nothing).
	var capacities := PackedInt32Array()
	for lod in LODS:
		var capacity := bucket_capacity(counts[lod]) if counts[lod] > 0 else 0
		capacities.append(capacity)
		if capacity > counts[lod]:
			var pad := PackedFloat32Array()
			pad.resize((capacity - counts[lod]) * FLOATS_PER_INSTANCE)
			pad.fill(0.0)
			buffers[lod].append_array(pad)
	var ms := (Time.get_ticks_usec() - start_us) * 1.0e-3
	_stats_mutex.lock()
	stats["nodes_built"] += 1
	stats["rocks_built"] += n
	stats["build_ms"] += ms
	stats["max_node_ms"] = maxf(stats["max_node_ms"], ms)
	stats["max_node_rocks"] = maxi(stats["max_node_rocks"], n)
	_stats_mutex.unlock()
	return {"node_min_m": min_d, "count": n, "buffers": buffers, "counts": counts, "capacities": capacities,
			"aabb": AABB(lo, hi - lo), "ids": ids, "build_ms": ms, "minus_d": minus_d, "factor": factor}


## Main thread, at the node's upload: the node's terrain material starts with every rock statistical; a node with rocks
## goes into the attach queue (process_queue).
func attach(instance: MeshInstance3D, rocks: Dictionary, terrain_material: ShaderMaterial) -> void:
	var start_us := Time.get_ticks_usec()
	terrain_material.set_shader_parameter("rock_node_min_m", 1.0e30)
	if int(rocks.get("count", 0)) > 0:
		var id := instance.get_instance_id()
		_queue[id] = [instance, rocks, terrain_material]
		_queue_order.append(id)
	stats["queued"] = _queue.size()
	stats["main_ms"] += (Time.get_ticks_usec() - start_us) * 1.0e-3


## True while the node's rocks wait in the attach queue (TerrainQuadtree keeps its parent drawn meanwhile).
func is_pending(instance: MeshInstance3D) -> bool:
	return _queue.has(instance.get_instance_id())


## Main thread, once per quadtree update: queued nodes attached in FIFO order until ATTACH_BUDGET_US is spent (at least
## one when nothing is over budget) or the pool cannot grow further this frame.
func process_queue() -> void:
	_allocations_this_frame = 0
	var start_us := Time.get_ticks_usec()  # the warming shares the attach budget
	var warm_us := start_us
	_prewarm_step()
	if _queue_order.is_empty():
		return
	warm_us = Time.get_ticks_usec() - warm_us
	var done := 0
	while not _queue_order.is_empty():
		if done > 0 and Time.get_ticks_usec() - start_us >= ATTACH_BUDGET_US:
			break
		var id: int = _queue_order[0]
		var item: Array = _queue.get(id, [])
		if item.is_empty():
			_queue_order.pop_front()
			continue
		if not _can_attach(item[1]):
			break  # the pool may grow by ALLOCATIONS_PER_FRAME per frame: wait for the next frame
		_queue_order.pop_front()
		_queue.erase(id)
		_attach_now(item[0], item[1], item[2])
		done += 1
	stats["queued"] = _queue.size()
	stats["main_ms"] += (Time.get_ticks_usec() - start_us - warm_us) * 1.0e-3  # the warming counted itself


## Whether every LOD list of the node finds a pooled MultiMesh or may allocate one this frame.
func _can_attach(rocks: Dictionary) -> bool:
	var capacities: PackedInt32Array = rocks["capacities"]
	var counts: PackedInt32Array = rocks["counts"]
	var needed := 0
	var wanted := {}
	for lod in LODS:
		if counts[lod] == 0:
			continue
		var key := pool_key(lod, capacities[lod])
		wanted[key] = int(wanted.get(key, 0)) + 1
	for key: int in wanted:
		var free: Array = _free.get(key, [])
		needed += maxi(int(wanted[key]) - (free.size() if key % POOL_KEY_PER_LOD > 0 else 0), 0)
	return needed == 0 or _allocations_this_frame + needed <= ALLOCATIONS_PER_FRAME


## Pool key of a LOD's bucket: a pooled MultiMesh keeps its LOD mesh for life (setting MultiMesh.mesh on a filled
## MultiMesh measured 16 ms on the main thread: Task 11 fix B report).
static func pool_key(lod: int, capacity: int) -> int:
	return lod * POOL_KEY_PER_LOD + capacity


func _attach_now(instance: MeshInstance3D, rocks: Dictionary, terrain_material: ShaderMaterial) -> void:
	var start_us := Time.get_ticks_usec()
	var min_d: float = rocks["node_min_m"]
	var material: ShaderMaterial = _free_materials.pop_back() if not _free_materials.is_empty() \
			else material_template.duplicate()
	for param in COPIED_NODE_PARAMETERS:
		material.set_shader_parameter(param, terrain_material.get_shader_parameter(param))
	material.set_shader_parameter("rock_node_min_m", min_d)
	material.set_shader_parameter("node_center_local", instance.position)
	var t_material := Time.get_ticks_usec()
	var buffers: Array = rocks["buffers"]
	var counts: PackedInt32Array = rocks["counts"]
	var capacities: PackedInt32Array = rocks["capacities"]
	var aabb: AABB = rocks["aabb"]
	var minus_d: Array = rocks["minus_d"]
	var entry := {"instance": instance, "center": instance.position + aabb.get_center(), "factor": rocks["factor"],
			"half_diagonal": 0.5 * aabb.size.length(), "node_min_m": min_d, "lods": [], "material": material,
			"work": 0}
	var t_mm := 0
	var t_take := 0
	var t_buffer := 0
	for lod in LODS:
		if counts[lod] == 0:
			continue
		var t0 := Time.get_ticks_usec()
		var key := pool_key(lod, capacities[lod])
		var node := _take(lod, capacities[lod], counts[lod])
		var multimesh := node.multimesh
		var t1 := Time.get_ticks_usec()
		RenderingServer.multimesh_set_buffer(multimesh.get_rid(), buffers[lod])
		var t2 := Time.get_ticks_usec()
		multimesh.custom_aabb = aabb
		multimesh.visible_instance_count = counts[lod]
		node.material_override = material
		instance.add_child(node)
		t_mm += Time.get_ticks_usec() - t0
		t_take += t1 - t0
		t_buffer += t2 - t1
		(entry["lods"] as Array).append([multimesh, minus_d[lod], lod, node, key])
		entry["work"] = int(entry["work"]) + counts[lod] * LOD_VERTICES[lod]
		stats["attached_instances"] += counts[lod]
	terrain_material.set_shader_parameter("rock_node_min_m", min_d)
	stats["attached_nodes"] += 1
	var total_ms := (Time.get_ticks_usec() - start_us) * 1.0e-3
	if total_ms > float(stats.get("max_attach_ms", 0.0)):
		stats["max_attach_ms"] = total_ms
		stats["max_attach_parts_ms"] = {"material": (t_material - start_us) * 1.0e-3, "multimeshes": t_mm * 1.0e-3,
				"take": t_take * 1.0e-3, "buffer": t_buffer * 1.0e-3}
	entry["camera"] = Vector3(1.0e30, 1.0e30, 1.0e30)
	entry["near"] = 0.0
	entry["threshold"] = -1.0
	entry["index"] = _list.size()
	_list.append(entry)
	_entries[instance.get_instance_id()] = entry


## A pooled MultiMeshInstance3D of the LOD and capacity (allocated when the bucket is empty; capacity 0: an exact
## unpooled one).
func _take(lod: int, capacity: int, count: int) -> MultiMeshInstance3D:
	if capacity == 0:
		stats["unpooled"] += 1
		_allocations_this_frame += 1
		return _allocate(lod, count)
	var key := pool_key(lod, capacity)
	var used := int(_in_use.get(key, 0)) + 1
	_in_use[key] = used
	var peak: Dictionary = stats["bucket_peak"]
	peak[key] = maxi(int(peak.get(key, 0)), used)
	var free: Array = _free.get(key, [])
	if not free.is_empty():
		return free.pop_back()
	_allocations_this_frame += 1
	return _allocate(lod, capacity)


func _allocate(lod: int, capacity: int) -> MultiMeshInstance3D:
	var start_us := Time.get_ticks_usec()
	var node := _allocate_now(lod, capacity)
	stats["max_allocate_ms"] = maxf(float(stats.get("max_allocate_ms", 0.0)), (Time.get_ticks_usec() - start_us) * 1.0e-3)
	return node


func _allocate_now(lod: int, capacity: int) -> MultiMeshInstance3D:
	var multimesh := MultiMesh.new()
	multimesh.transform_format = MultiMesh.TRANSFORM_3D
	multimesh.use_colors = true
	multimesh.use_custom_data = true
	multimesh.mesh = meshes[lod]  # before the data: an empty MultiMesh takes its mesh for free
	multimesh.instance_count = capacity
	multimesh.visible_instance_count = 0
	var node := MultiMeshInstance3D.new()
	node.multimesh = multimesh
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	node.set_meta(&"moon_rocks", true)
	stats["allocations"] += 1
	stats["pool_instances"] += capacity
	stats["pool_mb"] = float(stats["pool_instances"]) * FLOATS_PER_INSTANCE * 4.0 / 1048576.0
	return node


func _release_to_pool(node: MultiMeshInstance3D, key: int) -> void:
	node.multimesh.visible_instance_count = 0
	if key % POOL_KEY_PER_LOD == 0:  # unpooled
		stats["pool_instances"] -= node.multimesh.instance_count
		stats["pool_mb"] = float(stats["pool_instances"]) * FLOATS_PER_INSTANCE * 4.0 / 1048576.0
		node.queue_free()
		return
	if not _free.has(key):
		_free[key] = []
	(_free[key] as Array).append(node)
	if _in_use.has(key):
		_in_use[key] = maxi(int(_in_use[key]) - 1, 0)


## Main thread, at the node's release: its queue entry dropped or its rock instances returned to the pools (the mesh
## instance returns to the quadtree's pool).
func detach(instance: MeshInstance3D) -> void:
	var start_us := Time.get_ticks_usec()
	var id := instance.get_instance_id()
	_queue.erase(id)  # its _queue_order slot is skipped
	var entry: Dictionary = _entries.get(id, {})
	if not entry.is_empty():
		var index: int = entry["index"]
		var last: Dictionary = _list.back()
		_list[index] = last
		last["index"] = index
		_list.pop_back()
		_entries.erase(id)
		for item: Array in entry["lods"]:
			var node: MultiMeshInstance3D = item[3]
			stats["attached_instances"] -= (item[1] as PackedFloat32Array).size()
			instance.remove_child(node)
			node.material_override = null
			_release_to_pool(node, int(item[4]))
		_free_materials.append(entry["material"])
		stats["attached_nodes"] -= 1
	stats["queued"] = _queue.size()
	stats["main_ms"] += (Time.get_ticks_usec() - start_us) * 1.0e-3


## The rock MultiMeshes attached to a quadtree node instance (tests; the LOD is the mesh's CUSTOM0.x).
static func attached(instance: MeshInstance3D) -> Array:
	var out := []
	for child in instance.get_children():
		if child.has_meta(&"moon_rocks"):
			out.append((child as MultiMeshInstance3D).multimesh)
	return out


## Per frame (World, after the Moon terrain's update): the pixel angle of the active render target, the budget control,
## the load factor at the camera, the globals moon_rock_pixel_angle / moon_rock_threshold_px / moon_world_to_local.
func update(viewport: Viewport, moon_local_to_world: Basis, delta_s: float, camera_fixed_m := Vector3.ZERO) -> void:
	pixel_angle_rad = pixel_angle(viewport)
	var factor := abundance_threshold_factor(abundance_at(camera_fixed_m)) if camera_fixed_m != Vector3.ZERO else 1.0
	stats["load_factor"] = factor
	var rock_ms := _drawn_vertices * ROCK_NS_PER_VERTEX * 1.0e-6
	stats["rock_gpu_ms_estimate"] = rock_ms
	stats["drawn_vertices"] = _drawn_vertices
	var gpu_ms := 0.0
	if budget_enabled and geometry_enabled:
		if not _measuring:
			RenderingServer.viewport_set_measure_render_time(viewport.get_viewport_rid(), true)
			_measuring = true
		gpu_ms = RenderingServer.viewport_get_measured_render_time_gpu(viewport.get_viewport_rid())
		stats["gpu_ms"] = gpu_ms
		# The effective threshold is 1.5 px x max(scale, load factor): a rise starts from the load factor at the camera
		# (below it the scale changes nothing there).
		if gpu_ms > GPU_FLOOR_MS and gpu_ms - rock_ms < GPU_FLOOR_MS and threshold_scale < factor:
			threshold_scale = minf(factor, SCALE_MAX)
		threshold_scale = budget_step(threshold_scale, gpu_ms, GPU_FLOOR_MS, delta_s, rock_ms)
		var terrain_over := gpu_ms - rock_ms >= GPU_FLOOR_MS
		if terrain_over:
			stats["terrain_over_floor"] += 1
		if terrain_over != _logged_terrain:
			print("[MoonRockLayer] frame %s the %.1f ms floor without the rocks (GPU %.2f ms, rocks ~%.2f ms): %s" % [
					"over" if terrain_over else "back under", GPU_FLOOR_MS, gpu_ms, rock_ms,
					"the rock scale holds (terrain cost, T13)" if terrain_over else "rock control active"])
			_logged_terrain = terrain_over
	if threshold_scale > _logged_scale * LOG_STEP or threshold_scale < _logged_scale / LOG_STEP \
			or factor > _logged_factor * LOG_STEP or factor < _logged_factor / LOG_STEP:
		print("[MoonRockLayer] rock_threshold_scale %.2f, abundance load factor %.2f (effective %.2f px; GPU %.2f ms, rocks ~%.2f ms, floor %.1f ms)"
				% [threshold_scale, factor, ROCK_PIXEL_THRESHOLD * maxf(threshold_scale, factor), gpu_ms, rock_ms,
				GPU_FLOOR_MS])
		_logged_scale = threshold_scale
		_logged_factor = factor
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
## vertex work of instances it would collapse. Hidden nodes keep their counts. The visible nodes' drawn vertices are
## summed over each full round-robin cycle (the budget control's rock cost estimate).
func _update_visible_counts(camera_render: Vector3) -> void:
	var per_px := pixel_angle_rad * VISIBLE_DISTANCE_MARGIN
	var scale_rad := ROCK_PIXEL_THRESHOLD * threshold_scale * per_px if geometry_enabled else INF
	var n := _list.size()
	if n == 0:
		_drawn_vertices = 0
		return
	var start_us := Time.get_ticks_usec()
	for k in n:
		var index := (_cursor + k) % n
		if k % 32 == 31 and Time.get_ticks_usec() - start_us > COUNTS_BUDGET_US:
			_cursor = index  # continue here next frame
			return
		if index == 0:  # a cycle completed: publish its sum
			_drawn_vertices = _cycle_work
			_cycle_work = 0
		var entry: Dictionary = _list[index]
		var instance: MeshInstance3D = entry["instance"]
		if not instance.visible:
			continue
		var last_near: float = entry["near"]
		if entry["threshold"] == scale_rad and (entry["camera"] as Vector3).distance_squared_to(camera_render) \
				< COUNTS_MOVE_FRACTION * COUNTS_MOVE_FRACTION * last_near * last_near:
			_cycle_work += int(entry["work"])
			continue  # the camera moved less than COUNTS_MOVE_FRACTION of the node's distance: counts still valid
		var near := maxf((entry["center"] as Vector3).distance_to(camera_render) - float(entry["half_diagonal"]), 0.0)
		entry["camera"] = camera_render
		entry["near"] = near
		entry["threshold"] = scale_rad
		# Effective threshold: ROCK_PIXEL_THRESHOLD x max(scale, the node's load factor) pixels.
		var threshold_rad := maxf(scale_rad, ROCK_PIXEL_THRESHOLD * per_px * float(entry["factor"]))
		var cut := maxf(threshold_rad * near, float(entry["node_min_m"]))
		var work := 0
		for item: Array in entry["lods"]:
			var multimesh: MultiMesh = item[0]
			var minus_d: PackedFloat32Array = item[1]
			var lod := int(item[2])
			var count := minus_d.bsearch(-maxf(cut, LOD_MIN_PX[lod] * per_px * near), false)
			if multimesh.visible_instance_count != count:
				multimesh.visible_instance_count = count
			work += count * LOD_VERTICES[lod]
		entry["work"] = work
		_cycle_work += work
	_cursor = 0


## One budget-control step (the GPU floor guard): the scale's log rises by SCALE_RATE x the relative overload per second
## while the GPU frame is over the floor AND the frame without the rocks' estimated cost would be under it (raising the
## threshold can help); it falls by RELAX_RATE per second once the frame is RELAX_MARGIN below the floor, and holds
## otherwise - also while the terrain alone is over the floor (no wind-up). [1, SCALE_MAX]. gpu_ms <= 0 (not measured
## yet): kept.
static func budget_step(scale: float, gpu_ms: float, floor_ms: float, delta_s: float, rock_ms := INF) -> float:
	if gpu_ms <= 0.0 or delta_s <= 0.0:
		return scale
	var over := (gpu_ms - floor_ms) / floor_ms
	if over > 0.0:
		if gpu_ms - rock_ms < floor_ms:
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
