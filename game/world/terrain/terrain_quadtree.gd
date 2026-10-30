class_name TerrainQuadtree
extends Node3D
## Streamed terrain of one body (child of its CelestialBody): a cube-sphere quadtree whose leaves are drawn as
## TerrainNodeBuilder meshes, refined towards the camera, built on worker threads and uploaded within a time budget.
##
## Selection: a node splits when the camera is closer to its bounding sphere than LOD_FACTOR * its size, its mesh
## spacing (size / 64) is still coarser than the data (max_resolution_m) or, where the data is coarser or absent, than
## the curvature limit, and its level is below MAX_LEVEL; it merges again beyond MERGE_FACTOR * LOD_FACTOR * its size.
## A parent stays drawn until all four children are uploaded, so the drawn leaves always cover the whole body. Uploaded
## nodes hold their mesh until their subtree is merged; ancestors of drawn leaves keep theirs (hidden) for an instant
## merge.
## Each node is a MeshInstance3D at its centre (body-fixed float64, relative position in render axes), vertices float32
## relative to it. The shader (terrain_*.gdshader, terrain_morph.gdshaderinc) gets the node centre and the morph range
## per instance (once, at the upload) and the camera's body-local position as one global per frame, both as float32
## hi + lo parts (split_float32): the per-frame cost does not grow with the number of drawn nodes.

const LOD_FACTOR := 2.5  ## [E] split distance / node size (Task 9 measurement: see docs/subsystems.md)
const MERGE_FACTOR := 1.15  ## [E] merge hysteresis
const MAX_LEVEL := 21  ## [E] safety cap: ~7 cm cells at the Earth
const ROOT_LEVEL := 1  ## the 6 x 4 level-1 nodes are built at setup and never freed
## [E] Main-thread time per frame for uploads (mesh + crop textures + water mask, ~0.3-1 ms each; at least one per frame).
const UPLOAD_BUDGET_MS := 2.0
const UPLOADS_PER_FRAME := 16  ## [E] cap on uploads per frame on top of the time budget
const MAX_BUILDS_IN_FLIGHT := 8  ## [E] worker tasks at once (~20-40 ms each); leaves cores for the engine and the sim
## [E] Largest chord sagitta of a mesh cell on the body's curvature (m): without finer data a node still splits until its
## cells sag at most this far below the surface (flat rule: vehicles stand on the drawn ground within 1 cm).
const MAX_SAGITTA_M := 0.01
## [E] A child's morph ends this fraction inside its parent's split distance: the child's vertices lie on the parent's
## surface, inside the parent's bounding sphere up to finer relief, so they are fully morphed when the child appears.
const MORPH_END_MARGIN := 0.97
## [E] A node drawn for the first time starts at its parent's surface (morph floor 1) and fades to its own morph within
## this time: children whose upload came late (the camera already inside their morph range) slide in instead of popping.
const MORPH_IN_S := 0.5
## [E] Pooled MeshInstance3Ds kept for reuse (more are freed): about the nodes of a full ground view (leaves + hidden
## ancestors), so a jump between sites recycles instead of freeing and creating thousands of nodes.
const POOL_MAX := 4096
## [E] Main-thread time per frame for releasing dropped nodes, and the least released per frame (the queue drains).
const RELEASE_BUDGET_MS := 1.0
const RELEASE_MIN_PER_FRAME := 16
## Morph start is pulled back to at most this fraction of the end (a safety net: real nodes stay below it, see the
## quadtree test).
const MORPH_START_CLAMP := 0.99
## Morph range of the level-1 roots: they have no parent mesh, so they never morph (the shader's default: factor 0).
const NO_MORPH := Vector2(1.0e30, 2.0e30)

## False while the body is in scaled space (centre beyond CelestialBody.SCALED_SPACE_DISTANCE_M): then nothing is drawn
## and the body's ellipsoid mesh stands in.
var active := false
## Builds on the calling thread instead of WorkerThreadPool (tests).
var synchronous := false
var uploads_per_frame := UPLOADS_PER_FRAME
var upload_budget_ms := UPLOAD_BUDGET_MS
## Prefix of the global shader uniforms <prefix>_hi / <prefix>_lo that receive the camera's body-local render position
## (project.godot [shader_globals], read by the body's terrain shader); "" (tests): not written.
var camera_global := ""
## {nodes_drawn, nodes_loading, uploads_last_frame, update_ms (main-thread time of the last update)}
var stats := {"nodes_drawn": 0, "nodes_loading": 0, "uploads_last_frame": 0, "update_ms": 0.0}
var height_source: HeightSource
var imagery: ImagerySource
var radius_m := 1.0
## Mesh spacing (m) below which the curvature alone no longer splits a node: sqrt(8 R MAX_SAGITTA_M).
var curvature_spacing_m := INF

var _template: ShaderMaterial
var _nodes := {}  ## Vector4i -> _Node (uploaded)
var _roots: Array[_Node] = []
var _drawn: Array[_Node] = []  ## this frame's drawn leaves
var _previous_drawn: Array[_Node] = []  ## last frame's (their instances are visible until hidden here)
var _fading: Array[_Node] = []  ## nodes whose morph floor is still above 0
var _frame := 0
var _reached_count := 0  ## nodes reached this frame (all uploaded nodes: nothing to free)
var _selection_changed := true  ## a node split or merged this frame: the drawn set differs from the last one
var _wanted := {}  ## requested children this frame
var _pending := {}  ## Vector4i -> true: requested, not yet started
var _in_flight := {}  ## Vector4i -> WorkerThreadPool task id
var _finished := {}  ## Vector4i -> build result waiting for upload
var _results := {}  ## Vector4i -> build result, written by worker tasks
var _results_mutex := Mutex.new()
var _pool: Array[MeshInstance3D] = []
var _releasing: Array[_Node] = []  ## dropped nodes whose resources are released a few per frame
var _camera_fixed_m := Vector3.ZERO
var _camera_hi_name := &""
var _camera_lo_name := &""


class _Node:
	var key: Vector4i
	var center_fixed: Vector3
	var radius_m := 0.0  ## bounding sphere around center_fixed (grid and skirts)
	var size_m := 0.0
	## Below MAX_LEVEL and still coarser than the data, or where the data is coarser or absent, the curvature limit
	## (mesh spacing size_m / CELLS above the finer of the two).
	var can_split := false
	## (LOD_FACTOR * size_m + radius_m)^2: the camera closer to the centre than this (than the sphere by LOD_FACTOR *
	## size_m) splits the node; merge_distance_sq likewise with MERGE_FACTOR * LOD_FACTOR * size_m.
	var split_distance_sq := 0.0
	var merge_distance_sq := 0.0
	var morph_range_m := Vector2.ZERO  ## (start, end): morph 0 closer than start, 1 beyond end
	var max_resolution_m := INF
	var split := false
	var children: Array[_Node] = []  ## the four children while split (all uploaded)
	var reached_frame := -1
	var drawn_frame := -1
	var shown := false  ## instance visible
	var appeared := false  ## drawn at least once since its upload
	var morph_floor := 0.0
	var instance: MeshInstance3D  ## null once freed
	var material: ShaderMaterial


## heights / imagery (null: none) feed TerrainNodeBuilder; radius_m is the body's mean radius; the template material
## (terrain shader with its global textures) is duplicated per node.
func setup(heights: HeightSource, imagery_source: ImagerySource, body_radius_m: float,
		material_template: ShaderMaterial) -> void:
	height_source = heights
	imagery = imagery_source
	radius_m = body_radius_m
	curvature_spacing_m = sqrt(8.0 * radius_m * MAX_SAGITTA_M)
	_template = material_template
	_template.set_shader_parameter("metres_per_deg", radius_m * PI / 180.0)
	if camera_global != "":
		_camera_hi_name = StringName(camera_global + "_hi")
		_camera_lo_name = StringName(camera_global + "_lo")
	# Static initialisers run on first access: here on the main thread, never racing in two worker tasks.
	assert(TerrainNodeBuilder.INDICES.size() > 0 and TerrainNodeBuilder.SKIRT_EDGE.size() > 0)
	for face in 6:
		for j in 1 << ROOT_LEVEL:
			for i in 1 << ROOT_LEVEL:
				var key := Vector4i(face, ROOT_LEVEL, i, j)
				_upload(_build(key, height_source, imagery, radius_m))
				_roots.append(_nodes[key])
	for node in _roots:
		node.instance.visible = true
		node.shown = true
		node.appeared = true
		_drawn.append(node)


## Morph range (start, end; m from the camera) of a node of size size_m, bounding radius node_radius_m, whose parent has
## size parent_size_m. Start: the node is unmorphed out to its merge distance plus its sphere's diameter, so it shows
## its own surface whenever its children appear (LOD_FACTOR * size) or vanish (MERGE_FACTOR * LOD_FACTOR * size). End:
## fully morphed (its parent's surface) wherever it can be when its parent splits (LOD_FACTOR * parent size from the
## parent's sphere). End must exceed start; if a node's sphere is too large for that (never for nodes below the roots,
## see the quadtree test) start is pulled back to MORPH_START_CLAMP * end.
static func morph_range_m(size_m: float, node_radius_m: float, parent_size_m: float) -> Vector2:
	var end := MORPH_END_MARGIN * LOD_FACTOR * parent_size_m
	var start := MERGE_FACTOR * LOD_FACTOR * size_m + 2.0 * node_radius_m
	return Vector2(minf(start, MORPH_START_CLAMP * end), end)


## Morph range of node `key` with bounding radius node_radius_m on a body of mean radius body_radius_m: NO_MORPH for
## the roots (no parent mesh), morph_range_m below them.
static func node_morph_range_m(key: Vector4i, node_radius_m: float, body_radius_m: float) -> Vector2:
	if key.y <= ROOT_LEVEL:
		return NO_MORPH
	return morph_range_m(CubeSphere.node_size_m(key, body_radius_m), node_radius_m,
			CubeSphere.node_size_m(CubeSphere.parent(key), body_radius_m))


## Morph factor of a vertex distance_m from the camera (the shader's terrain_morph_factor without the morph floor).
static func morph_factor(distance_m: float, range_m: Vector2) -> float:
	return clampf((distance_m - range_m.x) / (range_m.y - range_m.x), 0.0, 1.0)


## [hi, lo]: hi = v rounded to float32 per component, lo = v - hi (float32 holds it to ~1e-14 m at planet scale).
## The shader forms (hi_a - hi_b) + (lo_a - lo_b): the hi difference of nearby points is exact in float32, so a
## camera-node vector keeps float32 precision relative to its length although both lie ~6400 km from the centre.
static func split_float32(v: Vector3) -> Array[Vector3]:
	var rounded := PackedFloat32Array([v.x, v.y, v.z])
	var hi := Vector3(rounded[0], rounded[1], rounded[2])
	return [hi, v - hi]


## Per frame: camera position in body-fixed metres and the frame time (s; drives the morph fade-in of new nodes).
## Uploads finished nodes, selects the drawn leaves, requests children, updates visibility and the camera global.
func update_for_camera(camera_fixed_m: Vector3, delta_s := 0.0) -> void:
	var start_us := Time.get_ticks_usec()
	_update(camera_fixed_m, delta_s)
	stats["update_ms"] = (Time.get_ticks_usec() - start_us) * 1.0e-3


func _update(camera_fixed_m: Vector3, delta_s: float) -> void:
	_camera_fixed_m = camera_fixed_m
	active = camera_fixed_m.length() < CelestialBody.SCALED_SPACE_DISTANCE_M
	visible = active
	stats["uploads_last_frame"] = 0
	if not active:
		_wanted.clear()
		_pending.clear()
		stats["nodes_drawn"] = 0
		stats["nodes_loading"] = _in_flight.size()
		_collect_builds()
		return
	if camera_global != "":
		var camera_render := split_float32(Frames.to_render(camera_fixed_m))
		ShaderGlobals.set_value(_camera_hi_name, camera_render[0])
		ShaderGlobals.set_value(_camera_lo_name, camera_render[1])
	_collect_builds()
	_upload_finished()
	_frame += 1
	_reached_count = 0
	_wanted.clear()
	var swap := _previous_drawn
	_previous_drawn = _drawn
	_drawn = swap
	_drawn.clear()
	_selection_changed = false
	var camera := _camera_fixed_m
	for node in _roots:
		node.reached_frame = _frame
		if node.can_split and node.center_fixed.distance_squared_to(camera) \
				< (node.merge_distance_sq if node.split else node.split_distance_sq):
			_split_or_draw(node)
		else:
			_draw_leaf(node)
	_reached_count += _roots.size()
	if _reached_count < _nodes.size():
		_free_unreached()
	_dispatch()
	_update_visibility(delta_s)
	_release_some()
	stats["nodes_drawn"] = _drawn.size()
	stats["nodes_loading"] = _pending.size() + _in_flight.size() + _finished.size()


## Keys of the drawn leaves (they tile the sphere).
func drawn_keys() -> Array[Vector4i]:
	var keys: Array[Vector4i] = []
	for node in _drawn:
		keys.append(node.key)
	return keys


## Keys of the nodes whose MeshInstance3D is visible (uploaded and released ones; tests: equals drawn_keys() after an
## update).
func visible_keys() -> Array[Vector4i]:
	var keys: Array[Vector4i] = []
	for node: _Node in _nodes.values() + _releasing:
		if node.instance != null and node.instance.visible:
			keys.append(node.key)
	return keys


## {center_fixed, radius_m, size_m, split, morph_range_m, morph_floor} of an uploaded node, {} otherwise (tests,
## diagnostics).
func node_info(key: Vector4i) -> Dictionary:
	if not _nodes.has(key):
		return {}
	var node: _Node = _nodes[key]
	return {"center_fixed": node.center_fixed, "radius_m": node.radius_m, "size_m": node.size_m, "split": node.split,
			"morph_range_m": node.morph_range_m, "morph_floor": node.morph_floor}


func _exit_tree() -> void:
	for task_id: int in _in_flight.values():
		WorkerThreadPool.wait_for_task_completion(task_id)
	_in_flight.clear()


## Hot path (every uploaded node of the drawn tree, every frame; GDScript costs ~1 us per node, so the loop is flat):
## node (reached, within its split distance) splits if its four children are uploaded, else stays a leaf and requests
## them. Children that stay leaves are handled inline; only children that split again recurse.
func _split_or_draw(node: _Node) -> void:
	if node.children.is_empty():
		_gather_children(node)
		if node.children.is_empty():
			_draw_leaf(node)
			return
	if not node.split:
		node.split = true
		_selection_changed = true
	var camera := _camera_fixed_m
	for child in node.children:
		child.reached_frame = _frame
		if child.can_split and child.center_fixed.distance_squared_to(camera) \
				< (child.merge_distance_sq if child.split else child.split_distance_sq):
			_split_or_draw(child)
		elif child.split:
			_draw_leaf(child)
		else:
			_drawn.append(child)
	_reached_count += 4


## A drawn leaf; a node that was split merges (its children are no longer reached and are freed this frame).
func _draw_leaf(node: _Node) -> void:
	if node.split:
		node.split = false
		node.children.clear()
		_selection_changed = true
	_drawn.append(node)


## The node's children once all four are uploaded; until then the missing ones are requested and the uploaded ones kept.
func _gather_children(node: _Node) -> void:
	var keys := CubeSphere.children(node.key)
	var ready := true
	for key in keys:
		if not _nodes.has(key):
			ready = false
			_request(key)
	for key in keys:
		var child: _Node = _nodes.get(key)
		if child == null:
			continue
		if ready:
			node.children.append(child)
		elif child.reached_frame != _frame:  # uploaded while its siblings load: keep it
			child.reached_frame = _frame
			_reached_count += 1


func _request(key: Vector4i) -> void:
	_wanted[key] = true
	if not _in_flight.has(key) and not _finished.has(key):
		_pending[key] = true


## Visibility from the drawn sets (only changes touch instances); a node drawn for the first time starts its morph
## fade-in (not the roots: they never morph).
func _update_visibility(delta_s: float) -> void:
	if _selection_changed:
		_show_drawn()
	if _fading.is_empty() or delta_s <= 0.0:
		return
	var still: Array[_Node] = []
	for node in _fading:
		if node.instance == null:
			continue
		node.morph_floor = maxf(node.morph_floor - delta_s / MORPH_IN_S, 0.0)
		node.instance.set_instance_shader_parameter(&"morph_floor", node.morph_floor)
		if node.morph_floor > 0.0:
			still.append(node)
	_fading = still


## Shows the drawn leaves that were hidden and hides last frame's leaves that are no longer drawn.
func _show_drawn() -> void:
	for node in _drawn:
		node.drawn_frame = _frame
		if node.shown:
			continue
		node.shown = true
		node.instance.visible = true
		if not node.appeared:
			node.appeared = true
			node.morph_floor = 1.0
			node.instance.set_instance_shader_parameter(&"morph_floor", 1.0)
			_fading.append(node)
	for node in _previous_drawn:
		if node.drawn_frame != _frame and node.shown and node.instance != null:
			node.shown = false
			node.instance.visible = false


## Nodes no longer reached (subtrees of merged nodes) are hidden and leave the tree at once; their mesh, material and
## textures are released by _release_some over the next frames (a jump between sites drops thousands at once).
func _free_unreached() -> void:
	for key: Vector4i in _nodes.keys():
		var node: _Node = _nodes[key]
		if node.reached_frame == _frame:
			continue
		node.instance.visible = false
		node.shown = false
		node.children.clear()
		_nodes.erase(key)
		_releasing.append(node)


## Releases dropped nodes until RELEASE_BUDGET_MS is spent (at least RELEASE_MIN_PER_FRAME); their MeshInstance3D returns
## to the pool (up to POOL_MAX).
func _release_some() -> void:
	if _releasing.is_empty():
		return
	var start_us := Time.get_ticks_usec()
	var released := 0
	while not _releasing.is_empty():
		if released >= RELEASE_MIN_PER_FRAME and Time.get_ticks_usec() - start_us >= RELEASE_BUDGET_MS * 1000.0:
			break
		var node: _Node = _releasing.pop_back()
		node.instance.mesh = null
		node.instance.material_override = null
		if _pool.size() < POOL_MAX:
			_pool.append(node.instance)
		else:
			node.instance.queue_free()
		node.instance = null
		node.material = null
		released += 1


## Drops requests no longer wanted, then starts builds, coarsest and nearest first.
func _dispatch() -> void:
	if not _finished.is_empty():
		for key: Vector4i in _finished.keys():
			if not _wanted.has(key):
				_finished.erase(key)
	if _pending.is_empty():
		return
	for key: Vector4i in _pending.keys():
		if not _wanted.has(key):
			_pending.erase(key)
	var order: Array[Vector4i] = []
	order.assign(_pending.keys())
	var priority := {}  # level first, then the distance of the node centre to the camera
	for key in order:
		priority[key] = Vector2(key.y, (CubeSphere.node_center(key) * radius_m).distance_to(_camera_fixed_m))
	order.sort_custom(func(a: Vector4i, b: Vector4i) -> bool: return priority[a] < priority[b])
	for key in order:
		if not synchronous and _in_flight.size() >= MAX_BUILDS_IN_FLIGHT:
			break
		_pending.erase(key)
		if synchronous:
			_finished[key] = _build(key, height_source, imagery, radius_m)
		else:
			_in_flight[key] = WorkerThreadPool.add_task(_build_task.bind(key, height_source, imagery, radius_m),
					false, "terrain node")


## Worker task: builds one node and stores the result for the main thread.
func _build_task(key: Vector4i, heights: HeightSource, imagery_source: ImagerySource, body_radius_m: float) -> void:
	var result := _build(key, heights, imagery_source, body_radius_m)
	_results_mutex.lock()
	_results[key] = result
	_results_mutex.unlock()


## TerrainNodeBuilder.build plus the node's bounding radius (thread-safe, pure data).
static func _build(key: Vector4i, heights: HeightSource, imagery_source: ImagerySource,
		body_radius_m: float) -> Dictionary:
	var result := TerrainNodeBuilder.build(key, heights, imagery_source, body_radius_m)
	var radius_sq := 0.0
	for v: Vector3 in result["vertices"]:
		radius_sq = maxf(radius_sq, v.length_squared())
	result["radius_m"] = sqrt(radius_sq)
	return result


## Completed worker tasks: results move to the upload queue (unwanted ones are dropped at the next dispatch).
func _collect_builds() -> void:
	if _in_flight.is_empty():
		return
	for key: Vector4i in _in_flight.keys():
		var task_id: int = _in_flight[key]
		if not WorkerThreadPool.is_task_completed(task_id):
			continue
		WorkerThreadPool.wait_for_task_completion(task_id)
		_in_flight.erase(key)
		_results_mutex.lock()
		var result: Dictionary = _results[key]
		_results.erase(key)
		_results_mutex.unlock()
		_finished[key] = result


## Uploads wanted results, coarsest first, until upload_budget_ms is spent (at least one) or uploads_per_frame are done.
func _upload_finished() -> void:
	if _finished.is_empty():
		return
	var start_us := Time.get_ticks_usec()
	var budget_us := upload_budget_ms * 1000.0
	var order: Array[Vector4i] = []
	order.assign(_finished.keys())
	order.sort_custom(func(a: Vector4i, b: Vector4i) -> bool: return a.y < b.y)
	for key in order:
		var done: int = stats["uploads_last_frame"]
		if done >= uploads_per_frame or (done > 0 and Time.get_ticks_usec() - start_us >= budget_us):
			return
		var result: Dictionary = _finished[key]
		_finished.erase(key)
		if _wanted.has(key) and not _nodes.has(key):
			_upload(result)
			stats["uploads_last_frame"] = done + 1


## Mesh, textures and material of a built node; hidden until selected.
func _upload(result: Dictionary) -> void:
	var key: Vector4i = result["key"]
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = result["vertices"]
	arrays[Mesh.ARRAY_NORMAL] = result["normals"]
	arrays[Mesh.ARRAY_TEX_UV] = result["uv"]
	arrays[Mesh.ARRAY_TEX_UV2] = result["uv2"]
	arrays[Mesh.ARRAY_CUSTOM0] = result["morph"]
	arrays[Mesh.ARRAY_CUSTOM1] = result["custom1"]
	arrays[Mesh.ARRAY_CUSTOM2] = result["custom2"]
	arrays[Mesh.ARRAY_CUSTOM3] = result["custom3"]
	arrays[Mesh.ARRAY_INDEX] = result["indices"]
	var format := (Mesh.ARRAY_CUSTOM_RGBA_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM0_SHIFT) \
			| (Mesh.ARRAY_CUSTOM_RG_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM1_SHIFT) \
			| (Mesh.ARRAY_CUSTOM_RG_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM2_SHIFT) \
			| (Mesh.ARRAY_CUSTOM_RGB_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM3_SHIFT)
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays, [], {}, format)

	var material: ShaderMaterial = _template.duplicate()
	for crop_name: String in ["fine", "base"]:
		var crop: Dictionary = result["imagery_" + crop_name]
		if crop.is_empty():
			continue
		var coverage: Rect2 = crop["coverage_deg"]  # in the node's longitude frame (the frame of CUSTOM1.u)
		material.set_shader_parameter("imagery_" + crop_name, ImageTexture.create_from_image(crop["image"]))
		material.set_shader_parameter(crop_name + "_coverage_deg", Vector4(coverage.position.x, coverage.position.y,
				coverage.end.x, coverage.end.y))
		material.set_shader_parameter(crop_name + "_fade_m", float(crop["fade_m"]))
	if result["horizon"] != null:  # bodies without horizon maps (the Earth) build none
		var horizon_map := Texture2DArray.new()
		horizon_map.create_from_images(result["horizon"])
		material.set_shader_parameter("horizon_map", horizon_map)
		material.set_shader_parameter("horizon_stride", float(TerrainNodeBuilder.HORIZON_STRIDE))
		material.set_shader_parameter("horizon_max_rad", float(result["horizon_max_rad"]))
		material.set_shader_parameter("horizon_enabled", true)
	if result["water_mask"] != null:  # bodies without water (the Moon) build none
		material.set_shader_parameter("water_mask", ImageTexture.create_from_image(result["water_mask"]))

	var node := _Node.new()
	node.key = key
	node.center_fixed = result["center_fixed"]
	node.radius_m = result["radius_m"]
	node.size_m = CubeSphere.node_size_m(key, radius_m)
	node.morph_range_m = node_morph_range_m(key, node.radius_m, radius_m)
	node.max_resolution_m = result["max_resolution_m"]
	node.can_split = key.y < MAX_LEVEL \
			and node.size_m / TerrainNodeBuilder.CELLS > minf(node.max_resolution_m, curvature_spacing_m)
	node.split_distance_sq = pow(LOD_FACTOR * node.size_m + node.radius_m, 2.0)
	node.merge_distance_sq = pow(MERGE_FACTOR * LOD_FACTOR * node.size_m + node.radius_m, 2.0)
	node.material = material
	node.instance = _pool.pop_back() if not _pool.is_empty() else _new_instance()
	node.instance.mesh = mesh
	node.instance.material_override = material
	node.instance.position = Frames.to_render(node.center_fixed)
	# Morph offsets move vertices up to about a cell; the skirt depth (2 cells) bounds them.
	node.instance.extra_cull_margin = node.size_m / TerrainNodeBuilder.CELLS * TerrainNodeBuilder.SKIRT_DEPTH_CELLS
	var offset := split_float32(node.instance.position)
	node.instance.set_instance_shader_parameter(&"vertex_offset", offset[0])
	node.instance.set_instance_shader_parameter(&"vertex_offset_lo", offset[1])
	node.instance.set_instance_shader_parameter(&"morph_start_m", node.morph_range_m.x)
	node.instance.set_instance_shader_parameter(&"morph_end_m", node.morph_range_m.y)
	node.instance.set_instance_shader_parameter(&"morph_floor", 0.0)
	node.instance.visible = false
	_nodes[key] = node


func _new_instance() -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(instance)
	return instance
