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
## queued nodes within ATTACH_BUDGET_US per frame, taking preallocated MultiMeshes from a pool of half-octave capacity
## buckets (a buffer upload, no allocation; at most ALLOCATIONS_PER_FRAME new pool entries per frame; lists longer than
## BUCKET_MAX come as several chunks) and pooled materials; the quadtree shows a node's children only once their rocks
## are attached (no rock pops at a split). A split node's hidden rock lists go back to the pools (park) and are queued
## again before it merges (ready_to_merge; the quadtree keeps the children drawn until then). The pool is bounded
## (POOL_MAX_MB; idle free entries trimmed). detach() at the node's release returns everything to the pools.
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
## The core's rock templates (RockTemplates::count) and LODs (RockTemplates::lods: icospheres 5..1); one more LOD here:
## the bare icosahedron (the first 12 vertices of every icosphere), for rocks below LOD_MIN_PX[4].
const TEMPLATES := 32
const CORE_LODS := 5
const LODS := 6
## [E] Lower edge of each LOD's projected-diameter band (px; LOD 0 = 10242 vertices, LOD 5 = 12): the chord sagitta of
## the icosphere's edges stays below ~0.3 px at every band's lower edge from LOD 1 on (subdivision s: sagitta ~ (1.107 /
## 2^s / 2)^2 / 2 of the radius); LOD 0 (Task 17) draws the fractured relief's finest steps and chips (core spacing
## ~3.5 cm per metre of rock) for rocks above 300 px - the near boulders only.
const LOD_MIN_PX: Array[float] = [300.0, 100.0, 30.0, 10.0, 4.0, 0.0]
const LOD_VERTICES: Array[int] = [10242, 2562, 642, 162, 42, 12]
## Vertex count of LOD 0 (the template textures' width).
const MAX_VERTICES := 10242
const FLOATS_PER_INSTANCE := 20  ## transform 12, colour 4, custom 4
## [E] moon_detail.gdshaderinc DETAIL_SHADOW_CASTER_FACTOR (core shadow_caster_factor): the drawn rocks cast the near-field
## shadow from this many times the geometry threshold (moon_rock.gdshader's shadow pass), below it statistically.
const SHADOW_CASTER_FACTOR := 4.0
## Shadow casters only near the eye (Task 12 fix 1, review C1): a node's shadow-only MultiMeshes are attached while the
## node lies within the near-field shadow map's reach - MoonNearShadow.caster_range_m plus its tallest rock's shadow
## length (height / tan of the Sun's elevation, at least CASTER_TAN_FLOOR), that extra at most the range - and released
## beyond CASTER_RELEASE_RATIO x the reach (hysteresis). Godot's directional map ends at the range: casters beyond it
## and their shadows' reach are never sampled. The farthest-node sweep (_far_scan) moves nodes in and out, at most
## CASTER_ATTACHES_PER_FRAME attaches per frame.
const CASTER_RELEASE_RATIO := 1.2  ## [E]
const CASTER_ATTACHES_PER_FRAME := 2  ## [E]
const CASTER_TAN_FLOOR := 0.0175  ## [E] tan 1 deg: below, the reach is the range's double anyway
## Texels per template row of rock_shape_data: (0, bottom, top, 0).
const SHAPE_TEXELS := 1
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
## [E, measured: Task 12 boulder jump] Pool instances newly allocated per frame (an allocation's main-thread cost grows
## with its buffer: up to ~2.5 ms for the largest buckets); the first attach of a frame may exceed it.
const ALLOCATED_INSTANCES_PER_FRAME := 16384
## Pool buckets per LOD: capacities BUCKET_MIN x 2^(k/2) (half-octave steps: padding waste < 41 %, ~17 % mean; review
## R3) up to BUCKET_MAX instances; a longer LOD list is split on the worker into BUCKET_MAX chunks, each a pooled
## MultiMesh (no exact, unpooled allocation on the main thread). Key = LOD x POOL_KEY_PER_LOD + capacity (pool_key).
const BUCKET_MIN := 64
const BUCKET_MAX := 16384
const POOL_KEY_PER_LOD := 1000000
## [E] Bound of the pool's GPU buffers (review R3: ~2 % of the user's 24 GB). An attach that would exceed it first
## trims free pool entries; if still over, the queued node stays statistical (its rocks in the ground's statistical
## layer: radiance-continuous), counted in stats.over_bound_nodes and logged.
const POOL_MAX_MB := 512.0
## [E] Over the bound (review N1 of the Task 11 fix C re-review) the farthest attached node gives its lists up for a
## nearer queued one, only when it is this much farther (no thrashing between nodes at similar distances); nodes that
## find none are kept statistical in a retry set and requeued nearest first.
const EVICT_DISTANCE_RATIO := 1.25
## [E] Frames between two looks at the retry set while the pool has no room (a scan of the attached nodes).
const RETRY_INTERVAL_FRAMES := 15
## [E, measured] Evictions per frame (counted against ATTACH_BUDGET_US). An eviction hands the farther node's pooled
## MultiMeshes straight to the waiting node (_transfer_to: same LOD, capacity at least the chunk's) and allocates nothing;
## releasing them to the pool instead (Task 12 first version) let the trim free and re-allocate GPU buffers: 34.7 ms
## spikes in the boulder jump. No donor covering the waiting node's buckets: no eviction, the node stays in the retry set.
const EVICTIONS_PER_FRAME := 1
## [E] Farther nodes one eviction may take together to cover the waiting node's buckets (a near node in a boulder field
## lists far more rocks than a far one).
const EVICT_DONORS_MAX := 4
## [E] Attached nodes looked at per frame for the farthest ones, and how many of them a sweep keeps.
const FAR_SCAN_PER_FRAME := 128
const FAR_KEEP := 16
## [E] Free pool entries beyond the key's prewarm count are released once their key saw no take for TRIM_IDLE_S, one
## per frame.
const TRIM_IDLE_S := 10.0
## [E] Free entries released per frame to make room under the bound (each a queued GPU buffer release).
const TRIM_PER_FRAME := 16
## [E] The pools warm only below this height above the Moon's mean sphere (not from orbit).
const PREWARM_MAX_HEIGHT_M := 50000.0
## [E, measured: Task 11 fix B report, IM-2 2 m camera jump] Pool entries allocated at start-up per pool key (about the
## peak use after a 10 km jump; the pool grows beyond on demand within the attach budget, ~0.15 ms per allocation).
## Measured with power-of-two buckets: each count is split half and half between its bucket and the half-octave bucket
## below (_init).
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
		&"detail_rock_shadow", &"detail_rock_view", &"detail_rock_view_present", &"detail_view_step",
		&"detail_hemispherical_albedo_mu",
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
var _cycle_min_factor := INF  ## the lowest load factor of the visible entries in the current cycle
var _min_visible_factor := INF  ## the last full cycle's (INF: none seen yet)
## Attach queue: mesh instance id -> [instance, rocks, terrain material] (FIFO order in _queue_order).
var _queue := {}
var _queue_order: Array[int] = []
## Pools: capacity -> Array of free MultiMeshInstance3D; free materials.
var _free := {}
var _free_materials: Array[ShaderMaterial] = []
var _in_use := {}  ## pool key -> pooled MultiMeshes attached now
var _prewarm_left := {}  ## pool key -> entries still to allocate (_prewarm_step)
var _prewarm_materials_left := 0
var _prewarm_target := {}  ## pool key -> its prewarm count (the trim keeps that many free entries)
var _last_take_ms := {}  ## pool key -> Time.get_ticks_msec() of its last take
var _allocations_this_frame := 0
var _allocated_instances_this_frame := 0
var _frame_upload_bytes := 0  ## MultiMesh buffer bytes uploaded by this frame's attaches
var _trimmed_this_frame := 0
## Hidden split parents whose rock lists were released (park): mesh instance id -> [rocks, terrain material]; requeued
## when the node is about to merge (ready_to_merge).
var _parked := {}
var _height_m := 0.0  ## camera height above the Moon's mean sphere (update(); 0 until known: the pools may warm)
var _logged_bound := 0
## Nodes kept statistical at the bound (review N1): mesh instance id -> [instance, rocks, terrain material]; requeued
## nearest first when the pool has room or a farther attached node can give its lists up.
var _retry := {}
var _camera_render := Vector3.ZERO  ## the camera in the Moon-local render frame (update())
var _retry_frame := 0
var _far_ready: Array = []  ## the last complete sweep's farthest attached nodes, [distance, entry], farthest first
var _far_sweep: Array = []  ## the running sweep's
var _far_cursor := 0
var _evictions_this_frame := 0
var _caster_attaches_this_frame := 0
## An eviction's hand-over (_transfer_to): requested pool key -> Array of [MultiMeshInstance3D, its own pool key].
var _donated := {}
## Review m1: the nodes counted in stats.over_bound_nodes (a requeued node going over the bound again counts once).
var _over_counted := {}
## Released nodes' worker rock lists (MBs each in a boulder field): their last references go to a worker task, which
## frees them off the main thread (Task 12 fix 1: freeing them in detach() stalled the main thread up to 3 ms per node).
var _graveyard: Array = []
var _disposal_tasks: Array[int] = []


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
	for key: int in PREWARM:
		var capacity := key % POOL_KEY_PER_LOD
		var count: int = PREWARM[key]
		var lower := bucket_capacity(int(capacity * 0.7071)) if capacity > BUCKET_MIN else capacity
		var lower_key := key - capacity + lower
		var upper := count - count / 2 if lower != capacity else count
		_prewarm_target[key] = int(_prewarm_target.get(key, 0)) + upper
		if lower != capacity:
			_prewarm_target[lower_key] = int(_prewarm_target.get(lower_key, 0)) + count / 2
	if prewarm and geometry_enabled:
		_prewarm_left = _prewarm_target.duplicate()
		_prewarm_materials_left = PREWARM_MATERIALS


## Warms the pools off the critical path (process_queue, once the Moon terrain is active): pool entries and materials
## until PREWARM_BUDGET_US of this frame is spent. True while some are left.
func _prewarm_step() -> bool:
	if (_prewarm_left.is_empty() and _prewarm_materials_left <= 0) or _height_m > PREWARM_MAX_HEIGHT_M:
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
	_part_ms("warm", Time.get_ticks_usec() - start_us)
	return true


## Meshes per LOD (unit icosphere directions; LOD 5: the icosahedron), the template textures (normal, radius per vertex;
## the cavity term per vertex: the dust), the shape texture (vertical extent) and the plan-area tables (the growth
## band's rise), from StarsimCore.rock_template_mesh / rock_template_exposed_area.
func _build_templates() -> void:
	var texels := PackedFloat32Array()
	texels.resize(MAX_VERTICES * LODS * TEMPLATES * 4)
	texels.fill(0.0)
	var cavities := PackedFloat32Array()
	cavities.resize(MAX_VERTICES * LODS * TEMPLATES)
	cavities.fill(0.0)
	top_z.resize(TEMPLATES)
	for lod in LODS:
		var core_lod := mini(lod, CORE_LODS - 1)
		var directions := PackedVector3Array()
		var indices := PackedInt32Array()
		for t in TEMPLATES:
			var mesh: Dictionary = core.rock_template_mesh(t, core_lod)
			var vertices: PackedVector3Array = mesh["vertices"]
			var normals: PackedVector3Array = mesh["normals"]
			var cavity: PackedFloat32Array = mesh["cavity"]
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
				cavities[(lod * TEMPLATES + t) * MAX_VERTICES + k] = cavity[k]
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
	var cavity_image := Image.create_from_data(MAX_VERTICES, LODS * TEMPLATES, false, Image.FORMAT_RF,
			cavities.to_byte_array())
	material_template.set_shader_parameter("rock_template_cavity", ImageTexture.create_from_image(cavity_image))
	_build_shape_tables()


## rock_shape_data (RGBAF, SHAPE_TEXELS x TEMPLATES): (0, bottom_z, top_z, 0). rock_area_data (RGF, AREA_SAMPLES x TEMPLATES): R = the exposed plan area A(z) /
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
		var row := t * SHAPE_TEXELS * 4
		var bottom: float = mesh["bottom_z"]
		var top: float = mesh["top_z"]
		shape[row + 1] = bottom
		shape[row + 2] = top
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


## Pool bucket capacity for a list (chunk) of n <= BUCKET_MAX instances: the smallest BUCKET_MIN x 2^(k/2) >= n.
static func bucket_capacity(n: int) -> int:
	var k := 0
	var capacity := BUCKET_MIN
	while capacity < n and capacity < BUCKET_MAX:
		k += 1
		capacity = roundi(BUCKET_MIN * pow(2.0, 0.5 * k))
	return mini(capacity, BUCKET_MAX)


## Worker: the node's rock list as MultiMesh buffers. {node_min_m, count, buffers: per LOD an Array of chunk buffers
## (PackedFloat32Array, <= BUCKET_MAX instances each, zero-padded to capacities[lod][chunk]), counts (per LOD),
## chunk_counts / capacities (per LOD: PackedInt32Array per chunk), aabb (node-local render axes), ids, build_ms,
## minus_d (per LOD, the whole list), factor}; {} without the geometry.
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
	var max_height_m := 0.0  # the tallest rock's top over its base (the shadow casters' reach)
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
		max_height_m = maxf(max_height_m, c * top_z[template_id] - burial[r])
		var reach := Vector3.ONE * axes[b3]
		var origin := Vector3(o.y, o.z, o.x)
		lo = lo.min(origin - reach)
		hi = hi.max(origin + reach)
	# The near-field shadow's casters (Task 12): per LOD the prefix of the D-sorted list from SHADOW_CASTER_FACTOR x the
	# node's limit up (at most BUCKET_MAX), its own pooled MultiMesh drawn only in the shadow pass (the main ones cast
	# none): the shadow pass's vertex work stays with the few rocks whose shadows the map resolves.
	var shadow_buffers: Array = []
	var shadow_counts := PackedInt32Array()
	var shadow_capacities := PackedInt32Array()
	for lod in LODS:
		var n_shadow := 0
		if MoonNearShadow.rocks_cast:
			n_shadow = mini((minus_d[lod] as PackedFloat32Array).bsearch(-SHADOW_CASTER_FACTOR * min_d, false), BUCKET_MAX)
		var shadow := PackedFloat32Array()
		var shadow_capacity := 0
		if n_shadow > 0:
			shadow_capacity = bucket_capacity(n_shadow)
			shadow = buffers[lod].slice(0, n_shadow * FLOATS_PER_INSTANCE)
			shadow.resize(shadow_capacity * FLOATS_PER_INSTANCE)  # zero-padded: zero transforms draw nothing
		shadow_buffers.append(shadow)
		shadow_counts.append(n_shadow)
		shadow_capacities.append(shadow_capacity)
	# Split each list into chunks of at most BUCKET_MAX and zero-pad each chunk to its pool bucket (the main thread only
	# uploads; zero transforms draw nothing).
	var chunks: Array = []  # per LOD: Array of PackedFloat32Array
	var chunk_counts: Array = []  # per LOD: PackedInt32Array
	var capacities: Array = []  # per LOD: PackedInt32Array
	for lod in LODS:
		var lod_chunks: Array[PackedFloat32Array] = []
		var lod_counts := PackedInt32Array()
		var lod_capacities := PackedInt32Array()
		var start := 0
		while start < counts[lod]:
			var n_chunk := mini(BUCKET_MAX, counts[lod] - start)
			var capacity := bucket_capacity(n_chunk)
			var chunk: PackedFloat32Array = buffers[lod] if n_chunk == counts[lod] else buffers[lod].slice(
					start * FLOATS_PER_INSTANCE, (start + n_chunk) * FLOATS_PER_INSTANCE)
			if capacity > n_chunk:
				var pad := PackedFloat32Array()
				pad.resize((capacity - n_chunk) * FLOATS_PER_INSTANCE)
				pad.fill(0.0)
				chunk.append_array(pad)
			lod_chunks.append(chunk)
			lod_counts.append(n_chunk)
			lod_capacities.append(capacity)
			start += n_chunk
		chunks.append(lod_chunks)
		chunk_counts.append(lod_counts)
		capacities.append(lod_capacities)
	var ms := (Time.get_ticks_usec() - start_us) * 1.0e-3
	_stats_mutex.lock()
	stats["nodes_built"] += 1
	stats["rocks_built"] += n
	stats["build_ms"] += ms
	stats["max_node_ms"] = maxf(stats["max_node_ms"], ms)
	stats["max_node_rocks"] = maxi(stats["max_node_rocks"], n)
	_stats_mutex.unlock()
	return {"node_min_m": min_d, "count": n, "buffers": chunks, "counts": counts, "chunk_counts": chunk_counts,
			"capacities": capacities, "aabb": AABB(lo, hi - lo), "ids": ids, "build_ms": ms, "minus_d": minus_d,
			"factor": factor, "shadow_buffers": shadow_buffers, "shadow_counts": shadow_counts,
			"shadow_capacities": shadow_capacities, "max_height_m": max_height_m}


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
	_part_ms("attach", Time.get_ticks_usec() - start_us)


## True while the node's rocks wait in the attach queue (TerrainQuadtree keeps its parent drawn meanwhile).
func is_pending(instance: MeshInstance3D) -> bool:
	return _queue.has(instance.get_instance_id())


## Main thread, when a split node's four children are drawn (TerrainQuadtree): the hidden parent's rock lists go back to
## the pools (review R3: hidden ancestors held most of the pool); its worker lists are kept for ready_to_merge.
func park(instance: MeshInstance3D) -> void:
	var id := instance.get_instance_id()
	var entry: Dictionary = _entries.get(id, {})
	if entry.is_empty():
		return  # no rocks, or still queued (it attaches and stays until the next split)
	var start_us := Time.get_ticks_usec()
	_parked[id] = [entry["rocks"], entry["terrain_material"]]
	_remove_entry(instance, entry)
	(entry["terrain_material"] as ShaderMaterial).set_shader_parameter("rock_node_min_m", 1.0e30)
	stats["parked_nodes"] = _parked.size()
	stats["main_ms"] += (Time.get_ticks_usec() - start_us) * 1.0e-3
	_part_ms("park", Time.get_ticks_usec() - start_us)


## Main thread, before a split node merges (TerrainQuadtree): true when its rocks are attached; a parked node is queued
## again (false until attached: the quadtree keeps its children drawn meanwhile, as at a split).
func ready_to_merge(instance: MeshInstance3D) -> bool:
	var id := instance.get_instance_id()
	if _parked.has(id):
		var parked: Array = _parked[id]
		_parked.erase(id)
		stats["parked_nodes"] = _parked.size()
		attach(instance, parked[0], parked[1])
		return false
	return not _queue.has(id)


## Main thread, once per quadtree update: the pool warming and trimming, then queued nodes attached in FIFO order until
## ATTACH_BUDGET_US is spent (at least one when nothing is over budget) or the pool cannot grow further this frame. A node
## whose attach would take the pool over POOL_MAX_MB (after trimming free entries) stays statistical.
func process_queue(camera_fixed_m := Vector3.ZERO) -> void:
	if camera_fixed_m != Vector3.ZERO:
		_camera_render = Frames.to_render(camera_fixed_m)  # the eviction / retry distances (also without update())
	_evictions_this_frame = 0
	_caster_attaches_this_frame = 0
	_allocations_this_frame = 0
	_allocated_instances_this_frame = 0
	_trimmed_this_frame = 0
	_frame_upload_bytes = 0
	var start_us := Time.get_ticks_usec()  # the warming shares the attach budget
	var warm_us := start_us
	_prewarm_step()
	warm_us = Time.get_ticks_usec() - warm_us
	_trim_idle()
	_dispose_released()
	var t_far := Time.get_ticks_usec()
	_far_scan()
	var t_requeue := Time.get_ticks_usec()
	_requeue_retry()
	var t_loop := Time.get_ticks_usec()
	stats["max_far_ms"] = maxf(float(stats.get("max_far_ms", 0.0)), (t_requeue - t_far) * 1.0e-3)
	stats["max_requeue_ms"] = maxf(float(stats.get("max_requeue_ms", 0.0)), (t_loop - t_requeue) * 1.0e-3)
	var done := 0
	while not _queue_order.is_empty():
		if done > 0 and Time.get_ticks_usec() - start_us >= ATTACH_BUDGET_US:
			break
		var id: int = _queue_order[0]
		var item: Array = _queue.get(id, [])
		if item.is_empty():
			_queue_order.pop_front()
			continue
		var verdict := _can_attach(item)
		if verdict == ATTACH_WAIT:
			break  # the pool may grow by ALLOCATIONS_PER_FRAME per frame: wait for the next frame
		_queue_order.pop_front()
		_queue.erase(id)
		done += 1  # a node kept statistical counts too: the loop stays inside the frame's budget
		if verdict == ATTACH_OVER_BOUND:
			# A farther attached node hands its pooled MultiMeshes over (nothing allocated, nothing freed).
			if _transfer_to(item, _queued_distance(item) * EVICT_DISTANCE_RATIO):
				_over_counted.erase(id)
				continue
			if not _over_counted.has(id):
				_over_counted[id] = true
				stats["over_bound_nodes"] = int(stats.get("over_bound_nodes", 0)) + 1
			_retry[id] = item  # its terrain material keeps rock_node_min_m = 1e30: every rock statistical
			continue
		_over_counted.erase(id)
		_attach_now(item[0], item[1], item[2])
	var t_shadow := Time.get_ticks_usec()
	stats["max_queue_loop_ms"] = maxf(float(stats.get("max_queue_loop_ms", 0.0)), (t_shadow - t_loop) * 1.0e-3)
	stats["max_shadow_ms"] = maxf(float(stats.get("max_shadow_ms", 0.0)), (Time.get_ticks_usec() - t_shadow) * 1.0e-3)
	stats["retry_nodes"] = _retry.size()
	var over := int(stats.get("over_bound_nodes", 0))
	if over > _logged_bound:
		var nearest := INF
		for retry_id: int in _retry:
			nearest = minf(nearest, _queued_distance(_retry[retry_id]))
		print("[MoonRockLayer] pool at its %.0f MB bound: %d nodes kept statistical so far, %d waiting, the nearest %.0f m away (pool %.0f MB, %d evicted; max ms: far %.2f, requeue %.2f, queue %.2f, shadows %.2f, detach %.2f)"
				% [POOL_MAX_MB, over, _retry.size(), nearest, float(stats["pool_mb"]), int(stats.get("evicted", 0)),
				float(stats.get("max_far_ms", 0.0)), float(stats.get("max_requeue_ms", 0.0)),
				float(stats.get("max_queue_loop_ms", 0.0)), float(stats.get("max_shadow_ms", 0.0)),
				float(stats.get("max_detach_ms", 0.0))])
		_logged_bound = over * 2  # log again when it doubled
	stats["queued"] = _queue.size()
	# Review m3: the buffer bytes this frame's attaches uploaded (against the RenderingDevice staging pool).
	stats["max_frame_upload_mb"] = maxf(float(stats.get("max_frame_upload_mb", 0.0)), _frame_upload_bytes / 1048576.0)
	var end_us := Time.get_ticks_usec()
	stats["main_ms"] += (end_us - start_us - warm_us) * 1.0e-3  # the warming counted itself
	_part_ms("queue", end_us - start_us - warm_us)
	if (end_us - start_us) * 1.0e-3 > float((stats.get("worst_queue_frame", {"ms": 0.0}) as Dictionary)["ms"]):
		# Diagnostics (the camera-jump gate): the slowest process_queue frame's parts.
		stats["worst_queue_frame"] = {"ms": (end_us - start_us) * 1.0e-3, "warm": warm_us * 1.0e-3,
				"far": (t_requeue - t_far) * 1.0e-3, "requeue": (t_loop - t_requeue) * 1.0e-3,
				"loop": (t_shadow - t_loop) * 1.0e-3, "items": done, "allocations": _allocations_this_frame,
				"allocated_instances": _allocated_instances_this_frame, "caster_attaches": _caster_attaches_this_frame,
				"evictions": _evictions_this_frame}


## Diagnostics (the camera-jump check): main-thread time per part, summed (stats.parts_ms).
func _part_ms(part: String, us: int) -> void:
	if not stats.has("parts_ms"):
		stats["parts_ms"] = {}
	var parts: Dictionary = stats["parts_ms"]
	parts[part] = float(parts.get(part, 0.0)) + us * 1.0e-3


const ATTACH_NOW := 0
const ATTACH_WAIT := 1
const ATTACH_OVER_BOUND := 2


## Distance (m) of a queued item's rocks from the camera (Moon-local render frame).
func _queued_distance(item: Array) -> float:
	var instance: MeshInstance3D = item[0]
	if not is_instance_valid(instance):
		return INF
	return (instance.position + (item[1]["aabb"] as AABB).get_center()).distance_to(_camera_render)


## Review N1 / Task 12 fix 1: the farthest attached node beyond limit_m hands its pooled MultiMeshes straight to the
## waiting node item (each of the item's buckets - the chunks and, near the eye, the shadow casters - from the free pool
## of its exact key or from the donor's node of the same LOD with the smallest capacity >= the chunk's; zero padding
## fills the larger buffer) and waits in the retry set (statistical meanwhile: radiance-continuous). Nothing is allocated
## or freed (the free-and-reallocate step made the 34.7 ms spikes). False (no eviction) when the donor cannot cover every
## bucket. The item is attached here.
func _transfer_to(item: Array, limit_m: float) -> bool:
	if _evictions_this_frame >= EVICTIONS_PER_FRAME:
		return false
	var rocks: Dictionary = item[1]
	var wanted: Array = []  # [lod, capacity]
	var capacities: Array = rocks["capacities"]
	for lod in LODS:
		for capacity: int in capacities[lod]:
			wanted.append([lod, capacity])
	if _casts(item[0], rocks, 1.0):
		var shadow_capacities: PackedInt32Array = rocks["shadow_capacities"]
		for lod in LODS:
			if shadow_capacities[lod] > 0:
				wanted.append([lod, shadow_capacities[lod]])
	wanted.sort_custom(func(a: Array, b: Array) -> bool: return int(a[1]) > int(b[1]))
	# The donors: the farthest nodes of the last sweep (FAR_KEEP of them, farthest first) beyond limit_m, taken one after
	# the other (at most EVICT_DONORS_MAX) until their pooled MultiMeshes cover the item's buckets.
	var donors: Array = []  # [index in _far_ready, entry]
	var offered: Array = []  # [node, its pool key, donor number]
	var assigned := {-1: true}  # offered index -> the requested pool key
	var at := 0
	while at < _far_ready.size() and donors.size() < EVICT_DONORS_MAX:
		var candidate: Dictionary = _far_ready[at][1]
		if not _is_live(candidate):
			_far_ready.remove_at(at)
			continue
		if float(_far_ready[at][0]) <= limit_m:
			break  # farthest first: the rest are nearer
		for slot: Array in candidate["lods"]:
			offered.append([slot[3], int(slot[4]), donors.size()])
		for slot: Array in candidate["shadow_lods"]:
			offered.append([slot[3], int(slot[4]), donors.size()])
		donors.append([at, candidate])
		at += 1
		assigned = _plan_transfer(wanted, offered)
		if not assigned.has(-1):
			break
	if assigned.has(-1):
		var reason := "transfer_uncovered" if not donors.is_empty() else ("transfer_donor_near"
				if not _far_ready.is_empty() else "transfer_no_donor")
		stats[reason] = int(stats.get(reason, 0)) + 1
		return false
	for k in range(donors.size() - 1, -1, -1):
		_far_ready.remove_at(int(donors[k][0]))
	_evictions_this_frame += 1
	_donated = {}
	for i in offered.size():
		var donor: Dictionary = donors[int(offered[i][2])][1]
		var node: MultiMeshInstance3D = offered[i][0]
		(donor["instance"] as MeshInstance3D).remove_child(node)
		node.material_override = null
		node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		if assigned.has(i):
			var key: int = assigned[i]
			if not _donated.has(key):
				_donated[key] = []
			(_donated[key] as Array).append(offered[i])
		else:
			_release_to_pool(node, int(offered[i][1]))
	for pair: Array in donors:
		var donor: Dictionary = pair[1]
		var donor_instance: MeshInstance3D = donor["instance"]
		_unlink_entry(donor_instance, donor)
		(donor["terrain_material"] as ShaderMaterial).set_shader_parameter("rock_node_min_m", 1.0e30)
		_retry[donor_instance.get_instance_id()] = [donor_instance, donor["rocks"], donor["terrain_material"]]
	_attach_now(item[0], item[1], item[2])
	for key: int in _donated:  # none expected: every assigned node is taken
		for slot: Array in _donated[key]:
			_release_to_pool(slot[0], int(slot[1]))
	_donated = {}
	stats["evicted_donors"] = int(stats.get("evicted_donors", 0)) + donors.size()
	stats["evicted"] = int(stats.get("evicted", 0)) + 1
	return true


## The hand-over plan: per offered node (index) the requested pool key it serves, the requests (sorted by capacity,
## largest first) served first from the free pool of their exact key, else by the offered node of the same LOD with the
## smallest capacity >= the request's. {-1: true} when some request finds none.
func _plan_transfer(wanted: Array, offered: Array) -> Dictionary:
	var from_free := {}  # pool key -> requests served by the free pool
	var assigned := {}
	for request: Array in wanted:
		var key := pool_key(int(request[0]), int(request[1]))
		if (_free.get(key, []) as Array).size() > int(from_free.get(key, 0)):
			from_free[key] = int(from_free.get(key, 0)) + 1
			continue
		var best := -1
		for i in offered.size():
			if assigned.has(i):
				continue
			var own: int = offered[i][1]
			var fits := own / POOL_KEY_PER_LOD == int(request[0]) and own % POOL_KEY_PER_LOD >= int(request[1])
			if fits and (best < 0 or own < int(offered[best][1])):
				best = i
		if best < 0:
			return {-1: true}
		assigned[best] = key
	return assigned


## The attached node farthest from the camera ({} with none), its distance in "far_m": from the last complete sweep
## (_far_scan, FAR_SCAN_PER_FRAME attached nodes per frame keeping the FAR_KEEP farthest: a full scan of thousands of
## nodes costs ms in GDScript), skipping those detached since.
func _farthest_entry() -> Dictionary:
	while not _far_ready.is_empty():
		var item: Array = _far_ready[0]
		var entry: Dictionary = item[1]
		if _is_live(entry):
			entry["far_m"] = item[0]
			return entry
		_far_ready.pop_front()
	return {}


## Whether a sweep's entry is still the attached one of its node (Task 12 fix 2, review N1): a node detached, handed over
## or parked and attached again has a NEW entry Dictionary under the same instance id; the old one in _far_ready /
## _far_sweep lists MultiMeshes that are no longer its own. Identity, not the id or content (== compares Dictionaries by
## value).
func _is_live(entry: Dictionary) -> bool:
	if not is_instance_valid(entry["instance"]):
		return false
	return is_same(_entries.get((entry["instance"] as MeshInstance3D).get_instance_id()), entry)


## One step of the farthest-node sweep (each process_queue).
func _far_scan() -> void:
	var n := _list.size()
	if n == 0:
		_far_ready.clear()
		_far_sweep.clear()
		_far_cursor = 0
		return
	for k in mini(FAR_SCAN_PER_FRAME, n):
		if _far_cursor >= n:
			_far_cursor = 0
		var entry: Dictionary = _list[_far_cursor]
		var d := (entry["center"] as Vector3).distance_to(_camera_render)
		_sweep_casters(entry, d)
		if _far_sweep.size() < FAR_KEEP or d > float(_far_sweep.back()[0]):
			var at := 0
			while at < _far_sweep.size() and float(_far_sweep[at][0]) >= d:
				at += 1
			_far_sweep.insert(at, [d, entry])
			if _far_sweep.size() > FAR_KEEP:
				_far_sweep.pop_back()
		_far_cursor += 1
		if _far_cursor >= n:  # a sweep complete: publish it
			_far_ready = _far_sweep
			_far_sweep = []
			_far_cursor = 0
			break


## Task 12 fix 1 (review C1): an attached node's shadow casters on as it comes within their reach, off beyond
## CASTER_RELEASE_RATIO x the reach (d: its centre's distance from the camera; one node of the sweep).
func _sweep_casters(entry: Dictionary, d: float) -> void:
	var rocks: Dictionary = entry["rocks"]
	if not _has_casters(rocks):
		return
	var reach := _caster_reach_m(rocks)
	var near := maxf(d - float(entry["half_diagonal"]), 0.0)
	var on := not (entry["shadow_lods"] as Array).is_empty()
	if on and (reach < 0.0 or near > reach * CASTER_RELEASE_RATIO):
		_release_casters(entry)
		stats["casters_released"] = int(stats.get("casters_released", 0)) + 1
	elif not on and reach >= 0.0 and near < reach and _caster_attaches_this_frame < CASTER_ATTACHES_PER_FRAME \
			and _caster_room(rocks):
		_attach_casters(entry)
		_caster_attaches_this_frame += 1
		stats["casters_attached_late"] = int(stats.get("casters_attached_late", 0)) + 1


## Whether the node's rock list has shadow casters at all.
static func _has_casters(rocks: Dictionary) -> bool:
	for n: int in rocks.get("shadow_counts", PackedInt32Array()):
		if n > 0:
			return true
	return false


## The casters' reach (m from the camera to the node's nearest point; < 0: no near-field shadows, no casters).
func _caster_reach_m(rocks: Dictionary) -> float:
	var range_m := MoonNearShadow.caster_range_m
	if range_m <= 0.0:
		return -1.0
	var tan_e := maxf(MoonNearShadow.caster_tan_sun, CASTER_TAN_FLOOR)
	return range_m + minf(float(rocks.get("max_height_m", 0.0)) / tan_e, range_m)


## Whether a node (its mesh instance and rock list) casts now: within ratio x the casters' reach.
func _casts(instance: MeshInstance3D, rocks: Dictionary, ratio: float) -> bool:
	if not _has_casters(rocks) or not is_instance_valid(instance):
		return false
	var reach := _caster_reach_m(rocks)
	if reach < 0.0:
		return false
	var aabb: AABB = rocks["aabb"]
	var near := maxf((instance.position + aabb.get_center()).distance_to(_camera_render) - 0.5 * aabb.size.length(), 0.0)
	return near < reach * ratio


## Whether the node's casters find pooled MultiMeshes or may allocate them this frame (no trimming: a node without room
## tries again at its next sweep).
func _caster_room(rocks: Dictionary) -> bool:
	var wanted := {}
	var shadow_capacities: PackedInt32Array = rocks["shadow_capacities"]
	for lod in LODS:
		if shadow_capacities[lod] > 0:
			var key := pool_key(lod, shadow_capacities[lod])
			wanted[key] = int(wanted.get(key, 0)) + 1
	var needed := 0
	var needed_instances := 0
	for key: int in wanted:
		var missing := maxi(int(wanted[key]) - (_free.get(key, []) as Array).size(), 0)
		needed += missing
		needed_instances += missing * (key % POOL_KEY_PER_LOD)
	if needed == 0:
		return true
	if _mb(int(stats["pool_instances"]) + needed_instances) > POOL_MAX_MB:
		return false
	if _allocated_instances_this_frame > 0 and _allocated_instances_this_frame + needed_instances > ALLOCATED_INSTANCES_PER_FRAME:
		return false
	return _allocations_this_frame + needed <= ALLOCATIONS_PER_FRAME


## The node's shadow-only MultiMeshes (build_node's caster prefix per LOD) attached: [MultiMesh, -D of the LOD list, lod,
## node, pool key, 0, instances]; the visible counts are recomputed at the next round (entry threshold reset).
func _attach_casters(entry: Dictionary) -> void:
	var rocks: Dictionary = entry["rocks"]
	var instance: MeshInstance3D = entry["instance"]
	var shadow_buffers: Array = rocks.get("shadow_buffers", [])
	var minus_d: Array = rocks["minus_d"]
	var lods: Array = []
	for lod in shadow_buffers.size():
		var n_shadow: int = (rocks["shadow_counts"] as PackedInt32Array)[lod]
		if n_shadow <= 0:
			continue
		var taken := _take_slot(lod, (rocks["shadow_capacities"] as PackedInt32Array)[lod])
		var node: MultiMeshInstance3D = taken[0]
		var buffer := _fit(shadow_buffers[lod], int(taken[1]) % POOL_KEY_PER_LOD)
		RenderingServer.multimesh_set_buffer(node.multimesh.get_rid(), buffer)
		_frame_upload_bytes += buffer.size() * 4
		node.multimesh.custom_aabb = rocks["aabb"]
		node.multimesh.visible_instance_count = n_shadow
		node.material_override = entry["material"]
		node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_SHADOWS_ONLY
		instance.add_child(node)
		lods.append([node.multimesh, minus_d[lod], lod, node, int(taken[1]), 0, n_shadow])
	entry["shadow_lods"] = lods
	entry["threshold"] = -1.0  # the casters' visible prefix at the next count round


## The node's shadow casters back to the pools.
func _release_casters(entry: Dictionary) -> void:
	var instance: MeshInstance3D = entry["instance"]
	for item: Array in entry["shadow_lods"]:
		var node: MultiMeshInstance3D = item[3]
		instance.remove_child(node)
		node.material_override = null
		node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		_release_to_pool(node, int(item[4]))
	entry["shadow_lods"] = []


## A buffer zero-padded to capacity instances (a handed-over MultiMesh may be a larger bucket than the chunk's).
static func _fit(buffer: PackedFloat32Array, capacity: int) -> PackedFloat32Array:
	var size := capacity * FLOATS_PER_INSTANCE
	if buffer.size() == size:
		return buffer
	var padded := buffer.duplicate()
	var from := padded.size()
	padded.resize(size)
	for i in range(from, size):
		padded[i] = 0.0
	return padded


## Review N1: the nearest node of the retry set back into the attach queue when the pool has room or (every
## RETRY_INTERVAL_FRAMES, with evictions on) when some attached node is EVICT_DISTANCE_RATIO farther.
func _requeue_retry() -> void:
	if _retry.is_empty():
		return
	_retry_frame += 1
	var room := float(stats["pool_mb"]) < POOL_MAX_MB * 0.9
	if not room and (EVICTIONS_PER_FRAME == 0 or _retry_frame % RETRY_INTERVAL_FRAMES != 0):
		return
	var best_id := -1
	var best_m := INF
	for id: int in _retry:
		var d := _queued_distance(_retry[id])
		if d < best_m:
			best_m = d
			best_id = id
	if best_id < 0:
		return
	if not room:
		var farthest := _farthest_entry()
		if farthest.is_empty() or float(farthest["far_m"]) <= best_m * EVICT_DISTANCE_RATIO:
			return
	var item: Array = _retry[best_id]
	_retry.erase(best_id)
	if not is_instance_valid(item[0]):
		return
	_queue[best_id] = item
	_queue_order.push_front(best_id)


## The released rock lists' last references handed to a worker task (it drops them: the buffers are freed there); the
## finished tasks collected.
func _dispose_released() -> void:
	var running: Array[int] = []
	for task_id in _disposal_tasks:
		if WorkerThreadPool.is_task_completed(task_id):
			WorkerThreadPool.wait_for_task_completion(task_id)
		else:
			running.append(task_id)
	_disposal_tasks = running
	if _graveyard.is_empty():
		return
	var batch := _graveyard
	_graveyard = []
	_disposal_tasks.append(WorkerThreadPool.add_task(func() -> void: batch.clear()))


## Review n3 (Task 11 fix C re-review): frees the pools' free entries (MultiMeshInstance3D nodes outside the tree) and
## drops the queues; the attached ones are children of the quadtree's node instances and go with them. Called by
## TerrainQuadtree._exit_tree.
func free_all() -> void:
	for key: int in _free:
		for node: MultiMeshInstance3D in _free[key]:
			if is_instance_valid(node):
				node.free()
	_free.clear()
	_free_materials.clear()
	for task_id in _disposal_tasks:
		WorkerThreadPool.wait_for_task_completion(task_id)
	_disposal_tasks.clear()
	_graveyard.clear()
	_parked.clear()
	_queue.clear()
	_queue_order.clear()
	_retry.clear()


## Whether every chunk of the node finds a pooled MultiMesh or may allocate one this frame (ATTACH_NOW / ATTACH_WAIT),
## or whether the allocations would take the pool over POOL_MAX_MB even after trimming free entries (ATTACH_OVER_BOUND).
func _can_attach(item: Array) -> int:
	var rocks: Dictionary = item[1]
	var capacities: Array = rocks["capacities"]
	var casts := _casts(item[0], rocks, 1.0)  # the casters come with the node only within their reach (C1)
	var wanted := {}
	for lod in LODS:
		for capacity: int in capacities[lod]:
			var key := pool_key(lod, capacity)
			wanted[key] = int(wanted.get(key, 0)) + 1
		var shadow_capacity: int = (rocks.get("shadow_capacities", PackedInt32Array([0, 0, 0, 0, 0])) as PackedInt32Array)[lod]
		if casts and shadow_capacity > 0:  # the shadow casters' bucket (attached with the node)
			var shadow_key := pool_key(lod, shadow_capacity)
			wanted[shadow_key] = int(wanted.get(shadow_key, 0)) + 1
	var needed := 0
	var needed_instances := 0
	for key: int in wanted:
		var free: Array = _free.get(key, [])
		var missing := maxi(int(wanted[key]) - free.size(), 0)
		needed += missing
		needed_instances += missing * (key % POOL_KEY_PER_LOD)
	if needed == 0:
		return ATTACH_NOW
	var over_mb := _mb(int(stats["pool_instances"]) + needed_instances) - POOL_MAX_MB
	if over_mb > 0.0:
		var freeable := _trim_free(over_mb, wanted)
		if _mb(int(stats["pool_instances"]) + needed_instances) > POOL_MAX_MB:
			# More free entries left than this frame may release: try again next frame; none left: over the bound.
			return ATTACH_WAIT if freeable else ATTACH_OVER_BOUND
	if _allocated_instances_this_frame > 0 and _allocated_instances_this_frame + needed_instances > ALLOCATED_INSTANCES_PER_FRAME:
		return ATTACH_WAIT  # the GPU buffers this frame already allocated (Task 12: an attach with its shadow casters)
	return ATTACH_NOW if _allocations_this_frame + needed <= ALLOCATIONS_PER_FRAME else ATTACH_WAIT


static func _mb(instances: int) -> float:
	return instances * FLOATS_PER_INSTANCE * 4.0 / 1048576.0


## Frees free pool entries of other keys than `keep` (largest buckets first) until at least `mb` are released, at most
## TRIM_PER_FRAME per frame. True while free entries of other keys are left (the bound can still be met later).
func _trim_free(mb: float, keep: Dictionary) -> bool:
	var keys := _free.keys()
	keys.sort_custom(func(a: int, b: int) -> bool: return a % POOL_KEY_PER_LOD > b % POOL_KEY_PER_LOD)
	var freed := 0.0
	var left := false
	for key: int in keys:
		if keep.has(key):
			continue
		var free: Array = _free[key]
		while not free.is_empty() and freed < mb and _trimmed_this_frame < TRIM_PER_FRAME:
			_free_entry(free.pop_back(), key)
			freed += _mb(key % POOL_KEY_PER_LOD)
			_trimmed_this_frame += 1
		left = left or not free.is_empty()
	return left


## One free entry beyond its key's prewarm count, of a key without a take for TRIM_IDLE_S, released per frame.
func _trim_idle() -> void:
	var now := Time.get_ticks_msec()
	for key: int in _free:
		var free: Array = _free[key]
		if free.size() > int(_prewarm_target.get(key, 0)) and now - int(_last_take_ms.get(key, 0)) > TRIM_IDLE_S * 1000.0:
			_free_entry(free.pop_back(), key)
			stats["trimmed"] = int(stats.get("trimmed", 0)) + 1
			return


func _free_entry(node: MultiMeshInstance3D, key: int) -> void:
	stats["pool_instances"] -= key % POOL_KEY_PER_LOD
	stats["pool_mb"] = _mb(int(stats["pool_instances"]))
	node.queue_free()


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
	var chunk_counts: Array = rocks["chunk_counts"]
	var capacities: Array = rocks["capacities"]
	var aabb: AABB = rocks["aabb"]
	var minus_d: Array = rocks["minus_d"]
	var entry := {"instance": instance, "center": instance.position + aabb.get_center(), "factor": rocks["factor"],
			"half_diagonal": 0.5 * aabb.size.length(), "node_min_m": min_d, "lods": [], "material": material,
			"work": 0, "rocks": rocks, "terrain_material": terrain_material}
	var t_mm := 0
	var t_take := 0
	var t_buffer := 0
	for lod in LODS:
		var lod_counts: PackedInt32Array = chunk_counts[lod]
		var offset := 0
		for c in lod_counts.size():
			var t0 := Time.get_ticks_usec()
			var capacity: int = (capacities[lod] as PackedInt32Array)[c]
			var taken := _take_slot(lod, capacity)
			var node: MultiMeshInstance3D = taken[0]
			var key: int = taken[1]
			var multimesh := node.multimesh
			var t1 := Time.get_ticks_usec()
			var chunk := _fit((buffers[lod] as Array)[c], key % POOL_KEY_PER_LOD)
			RenderingServer.multimesh_set_buffer(multimesh.get_rid(), chunk)
			_frame_upload_bytes += chunk.size() * 4
			var t2 := Time.get_ticks_usec()
			multimesh.custom_aabb = aabb
			multimesh.visible_instance_count = lod_counts[c]
			node.material_override = material
			node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
			instance.add_child(node)
			t_mm += Time.get_ticks_usec() - t0
			t_take += t1 - t0
			t_buffer += t2 - t1
			# [MultiMesh, -D of the whole LOD list, lod, node, pool key, first list index, instances in the chunk]
			(entry["lods"] as Array).append([multimesh, minus_d[lod], lod, node, key, offset, lod_counts[c]])
			entry["work"] = int(entry["work"]) + lod_counts[c] * LOD_VERTICES[lod]
			stats["attached_instances"] += lod_counts[c]
			offset += lod_counts[c]
	# The shadow pass's casters (build_node), only within their reach (review C1; _sweep_casters moves them later).
	entry["shadow_lods"] = []
	if _casts(instance, rocks, 1.0):
		_attach_casters(entry)
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


## [node, its pool key] for a chunk of the LOD and capacity: an eviction's handed-over node assigned to that bucket
## (_transfer_to; it may be a larger bucket), else _take.
func _take_slot(lod: int, capacity: int) -> Array:
	var key := pool_key(lod, capacity)
	var donated: Array = _donated.get(key, [])
	if not donated.is_empty():
		var slot: Array = donated.pop_back()
		_last_take_ms[int(slot[1])] = Time.get_ticks_msec()  # the trim keeps its bucket
		return slot
	return [_take(lod, capacity), key]


## A pooled MultiMeshInstance3D of the LOD and capacity (allocated when the bucket is empty).
func _take(lod: int, capacity: int) -> MultiMeshInstance3D:
	var key := pool_key(lod, capacity)
	var used := int(_in_use.get(key, 0)) + 1
	_in_use[key] = used
	_last_take_ms[key] = Time.get_ticks_msec()
	var peak: Dictionary = stats["bucket_peak"]
	peak[key] = maxi(int(peak.get(key, 0)), used)
	var free: Array = _free.get(key, [])
	if not free.is_empty():
		return free.pop_back()
	_allocations_this_frame += 1
	_allocated_instances_this_frame += capacity
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
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF  # the attach sets the shadow casters' (Task 12)
	node.set_meta(&"moon_rocks", true)
	stats["allocations"] += 1
	stats["pool_instances"] += capacity
	stats["pool_mb"] = _mb(int(stats["pool_instances"]))
	stats["pool_peak_mb"] = maxf(float(stats.get("pool_peak_mb", 0.0)), float(stats["pool_mb"]))
	return node


func _release_to_pool(node: MultiMeshInstance3D, key: int) -> void:
	node.multimesh.visible_instance_count = 0
	if not _free.has(key):
		_free[key] = []
	(_free[key] as Array).append(node)
	if _in_use.has(key):
		_in_use[key] = maxi(int(_in_use[key]) - 1, 0)


## Main thread, at the node's release: its queue entry dropped, its parked lists forgotten, or its rock instances returned
## to the pools (the mesh instance returns to the quadtree's pool).
func detach(instance: MeshInstance3D) -> void:
	var start_us := Time.get_ticks_usec()
	var id := instance.get_instance_id()
	# The node's rock lists (queued, waiting, parked or attached) freed on a worker (_dispose_released).
	# Only the rock lists (Dictionaries of Packed arrays and AABBs): never the items' terrain ShaderMaterial, whose last
	# reference must not drop on the worker (review n2 of Task 12 fix 1).
	var queued: Variant = _queue.get(id)
	var waiting: Variant = _retry.get(id)
	var parked: Variant = _parked.get(id)
	if queued is Array and (queued as Array).size() > 1:
		_graveyard.append(queued[1])
	if waiting is Array and (waiting as Array).size() > 1:
		_graveyard.append(waiting[1])
	if parked is Array and not (parked as Array).is_empty():
		_graveyard.append(parked[0])
	if _entries.has(id):
		_graveyard.append((_entries[id] as Dictionary)["rocks"])
	_queue.erase(id)  # its _queue_order slot is skipped
	_retry.erase(id)
	_over_counted.erase(id)
	if _parked.erase(id):
		stats["parked_nodes"] = _parked.size()
	var entry: Dictionary = _entries.get(id, {})
	var multimeshes := 0
	if not entry.is_empty():
		multimeshes = (entry["lods"] as Array).size() + (entry["shadow_lods"] as Array).size()
		_remove_entry(instance, entry)
	stats["queued"] = _queue.size()
	var us := Time.get_ticks_usec() - start_us
	if us * 1.0e-3 > float(stats.get("max_detach_ms", 0.0)):
		stats["max_detach_multimeshes"] = multimeshes  # diagnostics: the slowest detach's MultiMesh count
	stats["main_ms"] += (Time.get_ticks_usec() - start_us) * 1.0e-3
	_part_ms("detach", Time.get_ticks_usec() - start_us)
	stats["max_detach_ms"] = maxf(float(stats.get("max_detach_ms", 0.0)), (Time.get_ticks_usec() - start_us) * 1.0e-3)


## An attached node's MultiMeshes and material back to the pools, the entry out of the lists.
func _remove_entry(instance: MeshInstance3D, entry: Dictionary) -> void:
	_unlink_entry(instance, entry)
	for item: Array in entry["lods"]:
		var node: MultiMeshInstance3D = item[3]
		instance.remove_child(node)
		node.material_override = null
		_release_to_pool(node, int(item[4]))
	for item: Array in entry.get("shadow_lods", []):
		var node: MultiMeshInstance3D = item[3]
		instance.remove_child(node)
		node.material_override = null
		node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
		_release_to_pool(node, int(item[4]))


## The entry out of the lists and the counts, its material back to the pool (its MultiMeshes stay with the caller).
func _unlink_entry(instance: MeshInstance3D, entry: Dictionary) -> void:
	var index: int = entry["index"]
	var last: Dictionary = _list.back()
	_list[index] = last
	last["index"] = index
	_list.pop_back()
	_entries.erase(instance.get_instance_id())
	for item: Array in entry["lods"]:
		stats["attached_instances"] -= int(item[6])
	_free_materials.append(entry["material"])
	stats["attached_nodes"] -= 1


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
	if camera_fixed_m != Vector3.ZERO:
		_height_m = camera_fixed_m.length() - Atmosphere.MOON_RADIUS_M
	var factor :=abundance_threshold_factor(abundance_at(camera_fixed_m)) if camera_fixed_m != Vector3.ZERO else 1.0
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
		# The effective threshold is 1.5 px x max(scale, load factor): a rise starts from the lowest load factor of the
		# visible nodes (below it the scale changes nothing anywhere in view, so the jump pops no rock; review m4).
		var floor_factor := minf(factor, _min_visible_factor)
		if gpu_ms > GPU_FLOOR_MS and gpu_ms - rock_ms < GPU_FLOOR_MS and threshold_scale < floor_factor:
			threshold_scale = minf(floor_factor, SCALE_MAX)
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
	_camera_render = Frames.to_render(camera_fixed_m)
	_update_visible_counts(_camera_render)
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
		if index == 0:  # a cycle completed: publish its sums
			_drawn_vertices = _cycle_work
			_cycle_work = 0
			_min_visible_factor = _cycle_min_factor
			_cycle_min_factor = INF
		var entry: Dictionary = _list[index]
		var instance: MeshInstance3D = entry["instance"]
		if not instance.visible:
			continue
		_cycle_min_factor = minf(_cycle_min_factor, float(entry["factor"]))
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
			var listed := minus_d.bsearch(-maxf(cut, LOD_MIN_PX[lod] * per_px * near), false)
			var count := clampi(listed - int(item[5]), 0, int(item[6]))  # this chunk's share of the visible prefix
			if multimesh.visible_instance_count != count:
				multimesh.visible_instance_count = count
			work += count * LOD_VERTICES[lod]
		for item: Array in entry["shadow_lods"]:
			var multimesh: MultiMesh = item[0]
			var lod := int(item[2])
			var listed := (item[1] as PackedFloat32Array).bsearch(-maxf(SHADOW_CASTER_FACTOR * cut,
					LOD_MIN_PX[lod] * per_px * near), false)
			var count := clampi(listed, 0, int(item[6]))
			if multimesh.visible_instance_count != count:
				multimesh.visible_instance_count = count
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
