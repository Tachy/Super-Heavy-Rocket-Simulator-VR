class_name TerrainQuadtree
extends Node3D
## Streamed terrain of one body (child of its CelestialBody): a cube-sphere quadtree whose leaves are drawn as
## TerrainNodeBuilder meshes, refined towards the camera, built on worker threads and uploaded a few per frame.
##
## Selection: a node splits when the camera is closer to its bounding sphere than LOD_FACTOR * its size, its mesh
## spacing (size / 64) is still coarser than the data (max_resolution_m) or, where the data is coarser or absent, than
## the curvature limit, and its level is below MAX_LEVEL; it merges again beyond MERGE_FACTOR * LOD_FACTOR * its size.
## A parent stays drawn until all four children are uploaded, so the drawn leaves always cover the whole body. Uploaded
## nodes hold their mesh until their subtree is merged; ancestors of drawn leaves keep theirs (hidden) for an instant
## merge.
## Each node is a MeshInstance3D at its centre (body-fixed float64, relative position in render axes), vertices float32
## relative to it; the shader (terrain_*.gdshader) gets vertex_offset, camera_local and the morph range per instance.

const LOD_FACTOR := 3.0  ## [E] split distance / node size (tuned in Task 9; the morph range follows from it)
const MERGE_FACTOR := 1.15  ## [E] merge hysteresis
const MAX_LEVEL := 21  ## [E] safety cap: ~7 cm cells at the Earth
const ROOT_LEVEL := 1  ## the 6 x 4 level-1 nodes are built at setup and never freed
const UPLOADS_PER_FRAME := 4  ## [E] main-thread budget (~1-2 ms per upload: mesh + crop textures)
const MAX_BUILDS_IN_FLIGHT := 8  ## [E] worker tasks at once (~20-40 ms each); leaves cores for the engine and the sim
## [E] Largest chord sagitta of a mesh cell on the body's curvature (m): without finer data a node still splits until its
## cells sag at most this far below the surface (flat rule: vehicles stand on the drawn ground within 1 cm).
const MAX_SAGITTA_M := 0.01
## [E] A child's morph ends this fraction inside its parent's split distance: the child's vertices lie on the parent's
## surface, inside the parent's bounding sphere up to finer relief, so they are fully morphed when the child appears.
const MORPH_END_MARGIN := 0.97
## [E] Pooled MeshInstance3Ds kept for reuse (more are freed).
const POOL_MAX := 256
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
## {nodes_drawn, nodes_loading, uploads_last_frame}
var stats := {"nodes_drawn": 0, "nodes_loading": 0, "uploads_last_frame": 0}
var height_source: HeightSource
var imagery: ImagerySource
var radius_m := 1.0
## Mesh spacing (m) below which the curvature alone no longer splits a node: sqrt(8 R MAX_SAGITTA_M).
var curvature_spacing_m := INF

var _template: ShaderMaterial
var _nodes := {}  ## Vector4i -> _Node (uploaded)
var _roots: Array[Vector4i] = []
var _drawn: Array[Vector4i] = []
var _shown := {}  ## keys whose instance is visible (the previous frame's drawn set)
var _reached := {}  ## nodes kept this frame
var _wanted := {}  ## requested children this frame
var _pending := {}  ## Vector4i -> true: requested, not yet started
var _in_flight := {}  ## Vector4i -> WorkerThreadPool task id
var _finished := {}  ## Vector4i -> build result waiting for upload
var _results := {}  ## Vector4i -> build result, written by worker tasks
var _results_mutex := Mutex.new()
var _pool: Array[MeshInstance3D] = []
var _camera_fixed_m := Vector3.ZERO


class _Node:
	var key: Vector4i
	var center_fixed: Vector3
	var radius_m := 0.0  ## bounding sphere around center_fixed (grid and skirts)
	var size_m := 0.0
	var morph_range_m := Vector2.ZERO  ## (start, end): morph 0 closer than start, 1 beyond end
	var max_resolution_m := INF
	var split := false
	var instance: MeshInstance3D
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
	# Static initialisers run on first access: here on the main thread, never racing in two worker tasks.
	assert(TerrainNodeBuilder.INDICES.size() > 0 and TerrainNodeBuilder.SKIRT_EDGE.size() > 0)
	for face in 6:
		for j in 1 << ROOT_LEVEL:
			for i in 1 << ROOT_LEVEL:
				var key := Vector4i(face, ROOT_LEVEL, i, j)
				_roots.append(key)
				_upload(_build(key, height_source, imagery, radius_m))
	_drawn = _roots.duplicate()
	for key in _roots:
		_nodes[key].instance.visible = true
		_shown[key] = true


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


## Morph factor of a vertex distance_m from the camera (the shader's terrain_morph_factor).
static func morph_factor(distance_m: float, range_m: Vector2) -> float:
	return clampf((distance_m - range_m.x) / (range_m.y - range_m.x), 0.0, 1.0)


## Per frame: camera position in body-fixed metres. Uploads finished nodes, selects the drawn leaves, requests children.
func update_for_camera(camera_fixed_m: Vector3) -> void:
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
	_collect_builds()
	_upload_finished()
	_reached.clear()
	_wanted.clear()
	_drawn.clear()
	for key in _roots:
		_visit(_nodes[key])
	_free_unreached()
	_dispatch()
	var shown := {}
	for key in _drawn:
		shown[key] = true
	for key: Vector4i in _shown:
		if not shown.has(key) and _nodes.has(key):
			_nodes[key].instance.visible = false
	for key in _drawn:
		var node: _Node = _nodes[key]
		if not _shown.has(key):
			node.instance.visible = true
		node.instance.set_instance_shader_parameter("camera_local", Frames.to_render(camera_fixed_m - node.center_fixed))
	_shown = shown
	stats["nodes_drawn"] = _drawn.size()
	stats["nodes_loading"] = _pending.size() + _in_flight.size() + _finished.size()


## Keys of the drawn leaves (they tile the sphere).
func drawn_keys() -> Array[Vector4i]:
	return _drawn.duplicate()


## {center_fixed, radius_m, size_m, split} of an uploaded node, {} otherwise (tests, diagnostics).
func node_info(key: Vector4i) -> Dictionary:
	if not _nodes.has(key):
		return {}
	var node: _Node = _nodes[key]
	return {"center_fixed": node.center_fixed, "radius_m": node.radius_m, "size_m": node.size_m, "split": node.split,
			"morph_range_m": node.morph_range_m}


func _exit_tree() -> void:
	for task_id: int in _in_flight.values():
		WorkerThreadPool.wait_for_task_completion(task_id)
	_in_flight.clear()


func _visit(node: _Node) -> void:
	_reached[node.key] = true
	var threshold := LOD_FACTOR * node.size_m * (MERGE_FACTOR if node.split else 1.0)
	var distance := maxf(node.center_fixed.distance_to(_camera_fixed_m) - node.radius_m, 0.0)
	var spacing := node.size_m / TerrainNodeBuilder.CELLS
	# Finer than the data only where the curvature still asks for it (coarse or absent data).
	var want := node.key.y < MAX_LEVEL and spacing > minf(node.max_resolution_m, curvature_spacing_m) \
			and distance < threshold
	if want:
		var children := CubeSphere.children(node.key)
		var ready := true
		for child in children:
			if _nodes.has(child):
				_reached[child] = true  # uploaded while its siblings load: keep it
			else:
				ready = false
				_request(child)
		if ready:
			node.split = true
			for child in children:
				_visit(_nodes[child])
			return
	node.split = false
	_drawn.append(node.key)


func _request(key: Vector4i) -> void:
	_wanted[key] = true
	if not _in_flight.has(key) and not _finished.has(key):
		_pending[key] = true


## Nodes no longer reached (subtrees of merged nodes) release mesh, material and textures; their MeshInstance3D returns
## to the pool (up to POOL_MAX).
func _free_unreached() -> void:
	for key: Vector4i in _nodes.keys():
		if _reached.has(key):
			continue
		var node: _Node = _nodes[key]
		node.instance.mesh = null
		node.instance.material_override = null
		node.instance.visible = false
		if _pool.size() < POOL_MAX:
			_pool.append(node.instance)
		else:
			node.instance.queue_free()
		_nodes.erase(key)
		_shown.erase(key)


## Drops requests no longer wanted, then starts builds, coarsest and nearest first.
func _dispatch() -> void:
	for key: Vector4i in _pending.keys():
		if not _wanted.has(key):
			_pending.erase(key)
	for key: Vector4i in _finished.keys():
		if not _wanted.has(key):
			_finished.erase(key)
	if _pending.is_empty():
		return
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


## Uploads up to uploads_per_frame wanted results (coarsest first).
func _upload_finished() -> void:
	var order: Array[Vector4i] = []
	order.assign(_finished.keys())
	order.sort_custom(func(a: Vector4i, b: Vector4i) -> bool: return a.y < b.y)
	for key in order:
		if stats["uploads_last_frame"] >= uploads_per_frame:
			return
		var result: Dictionary = _finished[key]
		_finished.erase(key)
		if _wanted.has(key) and not _nodes.has(key):
			_upload(result)
			stats["uploads_last_frame"] += 1


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
			| (Mesh.ARRAY_CUSTOM_RGBA_FLOAT << Mesh.ARRAY_FORMAT_CUSTOM1_SHIFT) \
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
	material.set_shader_parameter("water_mask", ImageTexture.create_from_image(result["water_mask"]))

	var node := _Node.new()
	node.key = key
	node.center_fixed = result["center_fixed"]
	node.radius_m = result["radius_m"]
	node.size_m = CubeSphere.node_size_m(key, radius_m)
	node.morph_range_m = node_morph_range_m(key, node.radius_m, radius_m)
	node.max_resolution_m = result["max_resolution_m"]
	node.material = material
	node.instance = _pool.pop_back() if not _pool.is_empty() else _new_instance()
	node.instance.mesh = mesh
	node.instance.material_override = material
	node.instance.position = Frames.to_render(node.center_fixed)
	# Morph offsets move vertices up to about a cell; the skirt depth (2 cells) bounds them.
	node.instance.extra_cull_margin = node.size_m / TerrainNodeBuilder.CELLS * TerrainNodeBuilder.SKIRT_DEPTH_CELLS
	node.instance.set_instance_shader_parameter("vertex_offset", node.instance.position)
	node.instance.set_instance_shader_parameter("morph_start_m", node.morph_range_m.x)
	node.instance.set_instance_shader_parameter("morph_end_m", node.morph_range_m.y)
	node.instance.visible = false
	_nodes[key] = node
	_shown.erase(key)


func _new_instance() -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(instance)
	return instance
