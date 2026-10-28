class_name CloudSpike
extends RefCounted
## World volume plan Task 2, spike S (throw-away: removed in Task 13 with its branch in CloudEffect and render_check's
## --spike / --spike_scale): the far tier alone - rays [R_NEAR_M, end] from the cyclops point over the cloud pass's
## centre view (the union of both eye frusta, CloudCentreView) - marched over scratch volumes built once for the view
## (shaders/cloud_spike.gdshaderinc): an exact occupancy (dense taps per cell, a Chebyshev distance field dilated by one
## cell) and, for variant (b), a single-level bake of the meso density plus a dummy light volume. Variant (a) samples
## today's procedural density per step. The build runs on the render thread over several frames (one dispatch per
## frame: a dispatch of seconds would trip the driver's timeout); `ready` says when the march reads finished volumes.
## Owned by CloudEffect (render thread); the static settings are written by render_check before the first frame.

enum Mode { OFF, PROCEDURAL, BAKED }

## (a) PROCEDURAL / (b) BAKED; OFF: the cloud pass's own march.
static var mode := Mode.OFF
## The reference run of the sanity check (preflight C18): today's march clipped to [R_NEAR_M, end] (CloudEffect's
## diagnostic define CLOUD_SPIKE_CLIP_M).
static var reference_clip := false
## The weather's cloud heights (m above the ellipsoid; render_check: the slab's bottom and top): the occupancy's band,
## one cell wider on each side. Without a slab the whole cloud shell.
static var band_m := Vector2(CLOUD_BOTTOM_M, CLOUD_TOP_M)
## Render thread -> main thread: the volumes are built (the march reads them); the plan's figures for the log.
static var ready := false
## Render thread -> main thread: the plan was refused (render_check exits 1 instead of waiting for `ready`).
static var failed := false
static var plan_report := ""

## R_near (spec D): the far tier starts here.
const R_NEAR_M := 2000.0
## The occupancy's cell (the brief: <= 200 m), horizontal and vertical.
const OCCUPANCY_CELL_M := 200.0
## The bake's cell: the median far footprint, at least this (the brief: >= 100 m).
const BAKE_CELL_MIN_M := 100.0
## [E] The distance field's cap (cells): sphere tracing steps up to 16 cells (3.2 km) through the band's gaps; above and
## below the band the march jumps to its shells.
const DISTANCE_CAP_CELLS := 16
## The march's cloud shell (cloud_pass.gdshaderinc CLOUD_BOTTOM_M, clouds.gdshaderinc CLOUD_TOP_M).
const CLOUD_BOTTOM_M := -500.0
const CLOUD_TOP_M := 20000.0
## WGS84 equatorial radius and the model space's polar stretch (clouds.gdshaderinc CLOUD_A_M, CloudLayer).
const A_M := 6378137.0
## D3D12's 3-D texture limit per axis (D3D12_REQ_TEXTURE3D_U_V_OR_W_DIMENSION): the tiles' size.
const TILE := 2048
## The brief's scratch memory cap [E] and D3D12's resource size cap (D3D12_REQ_RESOURCE_SIZE_IN_MEGABYTES_EXPRESSION_C
## _TERM, 2048 MB).
const SCRATCH_MAX_BYTES := 6.0e9
const RESOURCE_MAX_BYTES := 2048.0 * 1024.0 * 1024.0
## [E] Rays per axis over the centre view that plan the region and the median footprint.
const PLAN_RAYS := 65
## [E] Points per band segment of a planning ray (the region's bounding box); its margin in occupancy cells.
const PLAN_POINTS := 17
const PLAN_MARGIN_CELLS := 4
## [E] Texture slices per build dispatch (one dispatch per frame): the taps ~40-100 ms, the distance passes and the
## light fill a few ms per slice.
const SLICES := {"taps": 4, "distance": 64, "bake": 8, "light": 256}
## The kernels' set 1 bindings in the march (cloud_spike.gdshaderinc CLOUD_SPIKE_FAR).
const BINDING_OCCUPANCY := 30
const BINDING_BAKE := 31
const BINDING_LIGHT := 32
const BINDING_GRID := 33
const GRID_BLOCK_SIZE := 10 * 16
const FILES := ["res://shaders/cloud_spike.gdshaderinc"]

var far_shader := RID()  ## the march kernel (CloudEffect binds it in its march's place)
var far_pipeline := RID()
var _rd: RenderingDevice
var _shaders := {}
var _pipelines := {}
var _grid_buffer := RID()
var _textures := {}  ## occupancy_a, occupancy_b, bake, light
var _plan := {}
var _phases: Array[Dictionary] = []  ## the build's dispatches still to run: {kernel, pass, slices, total, next}


## Compiles the spike's kernels (the march's files + cloud_spike.gdshaderinc: set 0 is the march's). false on an error.
func setup(rd: RenderingDevice, stereo: bool) -> bool:
	_rd = rd
	var files := PackedStringArray(CloudEffect.MARCH_FILES + FILES)
	var far_defines := PackedStringArray(["CLOUD_MARCH", "CLOUD_SPIKE_FAR"])
	if stereo:
		far_defines.append("CLOUD_CENTRE")
	var kernels := {"far": far_defines, "occupancy": PackedStringArray(["CLOUD_SPIKE_OCCUPANCY"]),
		"bake": PackedStringArray(["CLOUD_SPIKE_BAKE"])}
	for name: String in kernels:
		var src := RDShaderSource.new()
		src.language = RenderingDevice.SHADER_LANGUAGE_GLSL
		src.source_compute = CloudShaderPort.build(kernels[name], files).source
		var spirv := rd.shader_compile_spirv_from_source(src)
		if spirv.compile_error_compute != "":
			push_error("CloudSpike %s: %s" % [name, spirv.compile_error_compute])
			return false
		_shaders[name] = rd.shader_create_from_spirv(spirv)
		_pipelines[name] = rd.compute_pipeline_create(_shaders[name])
	far_shader = _shaders["far"]
	far_pipeline = _pipelines["far"]
	_grid_buffer = rd.uniform_buffer_create(GRID_BLOCK_SIZE)
	return true


## Render thread, per frame (before the march): plans and creates the volumes on the first call, runs the next build
## dispatch until all are done; returns the march's extra set 1 uniforms. Empty on a failed plan (error printed).
## camera: float64 Earth-local; head_to_earth: the head's axes -> Earth-local; projection / pixel_angle: the centre view's.
func frame(set0: Array[RDUniform], samplers: Dictionary, camera: Vector3, head_to_earth: Basis,
		projection: Projection, pixel_angle: float) -> Array[RDUniform]:
	if _plan.is_empty():
		_plan = _plan_region(camera, head_to_earth, projection, pixel_angle)
		if _plan.is_empty() or not _create_volumes():
			_plan = {"failed": true}
			failed = true
	if _plan.has("failed"):
		return []
	if not _phases.is_empty():
		_build_step(set0, samplers)
	var nearest: RID = samplers["false|false|false"]
	var linear: RID = samplers["true|false|false"]
	var occupancy: RID = _textures["occupancy_b"]
	var uniforms: Array[RDUniform] = [CloudEffect._tex(BINDING_OCCUPANCY, nearest, occupancy),
		CloudEffect._tex(BINDING_BAKE, linear, _textures.get("bake", occupancy)),
		CloudEffect._tex(BINDING_LIGHT, linear, _textures.get("light", occupancy)),
		CloudEffect._ubo(_grid_buffer, BINDING_GRID)]
	return uniforms


## Frees the kernels and volumes.
func free_all() -> void:
	if _rd == null:
		return
	for rid: RID in _textures.values():
		_rd.free_rid(rid)
	for rid: RID in _shaders.values():  # (frees their pipelines too)
		_rd.free_rid(rid)
	if _grid_buffer.is_valid():
		_rd.free_rid(_grid_buffer)
	_textures.clear()
	_shaders.clear()


## The region the far rays cross in the band and the volumes' layout: the grid's axes (the view's forward azimuth), the
## bounding box of PLAN_RAYS^2 rays' band segments beyond R_NEAR_M (in front of the ellipsoid) plus a margin, the bake
## cell from the median footprint at the middle of each ray's first band segment. Empty when the view has no band
## segment or the volumes exceed the caps (error printed).
func _plan_region(camera: Vector3, head_to_earth: Basis, projection: Projection, pixel_angle: float) -> Dictionary:
	var stretch := Vector3(1.0, CloudLayer.POLAR_STRETCH, 1.0)
	var cam_m := camera * stretch
	var up0 := cam_m.normalized()
	var forward := (head_to_earth * CloudCentreView._ray(projection, Vector2.ZERO)) * stretch
	var level := forward - up0 * forward.dot(up0)
	var axis_x := level.normalized() if level.length() > 1.0e-6 * forward.length() \
			else up0.cross(Vector3(0.0, 1.0, 0.0) if absf(up0.y) < 0.9 else Vector3(1.0, 0.0, 0.0)).normalized()
	var axis_y := up0.cross(axis_x)
	var z_lo := maxf(band_m.x - OCCUPANCY_CELL_M, CLOUD_BOTTOM_M)
	var z_hi := minf(band_m.y + OCCUPANCY_CELL_M, CLOUD_TOP_M)
	var lo_xy := Vector2(INF, INF)
	var hi_xy := Vector2(-INF, -INF)
	var footprints := PackedFloat64Array()
	for j in range(PLAN_RAYS):
		for i in range(PLAN_RAYS):
			var ndc := Vector2(2.0 * i / (PLAN_RAYS - 1) - 1.0, 2.0 * j / (PLAN_RAYS - 1) - 1.0)
			var dir_m := (head_to_earth * CloudCentreView._ray(projection, ndc)) * stretch
			var scale := dir_m.length()
			dir_m /= scale
			var start := R_NEAR_M * scale  # model distances
			# The ray's end on the ellipsoid (the march ends on the scene): its first crossing ahead; an eye at or below
			# the ellipsoid (the pad: model height ~0) ends every ray pointing below its horizon at once.
			var ground := _inside(cam_m, dir_m, A_M)
			var end := INF
			if ground.x <= ground.y and ground.y > 0.0:
				end = ground.x if ground.x > 0.0 else (0.0 if dir_m.dot(up0) < 0.0 else INF)
			var outer := _inside(cam_m, dir_m, A_M + z_hi)
			if outer.x > outer.y:
				continue
			var inner := _inside(cam_m, dir_m, A_M + z_lo)
			var segments: Array[Vector2] = []
			if inner.x > inner.y:
				segments.append(outer)
			else:
				segments.append(Vector2(outer.x, minf(outer.y, inner.x)))
				segments.append(Vector2(maxf(outer.x, inner.y), outer.y))
			var first := true
			for s in segments:
				var a := maxf(s.x, start)
				var b := minf(s.y, end)
				if b <= a:
					continue
				for k in range(PLAN_POINTS):
					var xy := _map(cam_m + dir_m * lerpf(a, b, float(k) / (PLAN_POINTS - 1)), up0, axis_x, axis_y)
					lo_xy = lo_xy.min(xy)
					hi_xy = hi_xy.max(xy)
				if first:
					footprints.append(pixel_angle * 0.5 * (a + b) / scale)
					first = false
	if footprints.is_empty():
		push_error("CloudSpike: no far ray crosses the band %s" % band_m)
		return {}
	footprints.sort()
	var median_footprint_m := footprints[footprints.size() / 2]
	var margin := PLAN_MARGIN_CELLS * OCCUPANCY_CELL_M
	lo_xy -= Vector2(margin, margin)
	hi_xy += Vector2(margin, margin)
	var bake_cell := maxf(BAKE_CELL_MIN_M, median_footprint_m)
	var plan := {"up0": up0, "axis_x": axis_x, "axis_y": axis_y, "min_xy": lo_xy, "z": Vector2(z_lo, z_hi),
		"bake_cell": bake_cell, "median_footprint_m": median_footprint_m, "rays": footprints.size(),
		"pixel_angle": pixel_angle,
		"occupancy": _layout(hi_xy - lo_xy, z_hi - z_lo, OCCUPANCY_CELL_M),
		"bake": _layout(hi_xy - lo_xy, z_hi - z_lo, bake_cell)}
	var occupancy: Dictionary = plan["occupancy"]
	var bake: Dictionary = plan["bake"]
	var texels_o: float = occupancy["texels"]
	var texels_b: float = bake["texels"]
	var bytes := {"occupancy_a": 4.0 * texels_o, "occupancy_b": 4.0 * texels_o}
	if mode == Mode.BAKED:
		bytes["bake"] = 2.0 * texels_b
		bytes["light"] = 8.0 * texels_o
	var total := 0.0
	for name: String in bytes:
		total += bytes[name]
	plan["bytes"] = bytes
	plan_report = ("spike plan: band %.0f..%.0f m, region x %.0f..%.0f km y %.0f..%.0f km, rays %d, median far "
			+ "footprint %.1f m, occupancy cell %.0f m %s, bake cell %.0f m %s, scratch %.2f GB %s") % [z_lo, z_hi,
			lo_xy.x / 1000.0, hi_xy.x / 1000.0, lo_xy.y / 1000.0, hi_xy.y / 1000.0, footprints.size(),
			median_footprint_m, OCCUPANCY_CELL_M, _layout_text(occupancy), bake_cell, _layout_text(bake),
			total / 1.0e9, bytes]
	print("CloudSpike: " + plan_report)
	var too_deep: bool = occupancy["tex"].z > TILE or (mode == Mode.BAKED and bake["tex"].z > TILE)
	var too_big := total > SCRATCH_MAX_BYTES
	for name: String in bytes:
		too_big = too_big or bytes[name] > RESOURCE_MAX_BYTES
	if too_deep or too_big:
		push_error("CloudSpike: the volumes exceed the caps (depth %d texels, %.2f GB, %.2f GB per resource)" % [TILE,
				SCRATCH_MAX_BYTES / 1.0e9, RESOURCE_MAX_BYTES / 1.0e9])
		return {}
	return plan


## The tiled layout of a volume over extent (m, horizontal) x height (m) at cell (m): cells, tiles, texture size.
static func _layout(extent: Vector2, height_m: float, cell_m: float) -> Dictionary:
	var cells := Vector3i(ceili(extent.x / cell_m), ceili(extent.y / cell_m), maxi(ceili(height_m / cell_m), 1))
	var tiles := Vector2i(_tiles(cells.x), _tiles(cells.y))
	var tex := Vector3i(mini(cells.x, TILE), mini(cells.y, TILE), tiles.x * tiles.y * (cells.z + 2))
	return {"cells": cells, "tiles": tiles, "tex": tex, "texels": float(tex.x) * tex.y * tex.z}


static func _tiles(n: int) -> int:
	return 1 if n <= TILE else ceili(float(n - 1) / (TILE - 1))


static func _layout_text(layout: Dictionary) -> String:
	return "(cells %s, tiles %s, texture %s)" % [layout["cells"], layout["tiles"], layout["tex"]]


## The model distances where the line cam_m + dir_m s (dir_m unit) is inside the sphere of radius r: x > y when never.
static func _inside(cam_m: Vector3, dir_m: Vector3, r: float) -> Vector2:
	var b := cam_m.dot(dir_m)
	var disc := b * b - (cam_m.length_squared() - r * r)
	if disc < 0.0:
		return Vector2(1.0, -1.0)
	var q := sqrt(disc)
	return Vector2(-b - q, -b + q)


## The grid's horizontal coordinates (m) of a model-space point (cloud_spike.gdshaderinc cloud_spike_map).
static func _map(p_m: Vector3, up0: Vector3, axis_x: Vector3, axis_y: Vector3) -> Vector2:
	var u := p_m.normalized()
	var xy := Vector2(u.dot(axis_x), u.dot(axis_y))
	var s := xy.length()
	return xy * (A_M * atan2(s, u.dot(up0)) / s) if s > 0.0 else Vector2.ZERO


func _create_volumes() -> bool:
	var occupancy: Dictionary = _plan["occupancy"]
	var bake: Dictionary = _plan["bake"]
	_textures["occupancy_a"] = _volume(occupancy["tex"], RenderingDevice.DATA_FORMAT_R16G16_SFLOAT)
	_textures["occupancy_b"] = _volume(occupancy["tex"], RenderingDevice.DATA_FORMAT_R16G16_SFLOAT)
	var occupancy_slices: int = occupancy["tex"].z
	_phases = [{"kernel": "occupancy", "pass": 0, "slices": SLICES["taps"], "total": occupancy_slices,
			"out": "occupancy_a", "in": "occupancy_b"},
		{"kernel": "occupancy", "pass": 1, "slices": SLICES["distance"], "total": occupancy_slices,
			"out": "occupancy_b", "in": "occupancy_a"},
		{"kernel": "occupancy", "pass": 2, "slices": SLICES["distance"], "total": occupancy_slices,
			"out": "occupancy_a", "in": "occupancy_b"},
		{"kernel": "occupancy", "pass": 3, "slices": SLICES["distance"], "total": occupancy_slices,
			"out": "occupancy_b", "in": "occupancy_a"}]
	if mode == Mode.BAKED:
		_textures["bake"] = _volume(bake["tex"], RenderingDevice.DATA_FORMAT_R16_SFLOAT)
		_textures["light"] = _volume(occupancy["tex"], RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
		var bake_slices: int = bake["tex"].z
		_phases.append({"kernel": "bake", "pass": 0, "slices": SLICES["bake"], "total": bake_slices})
		_phases.append({"kernel": "bake", "pass": 1, "slices": SLICES["light"], "total": occupancy_slices})
	for phase in _phases:
		phase["next"] = 0
	for rid: RID in _textures.values():
		if not rid.is_valid():
			push_error("CloudSpike: a volume was not created")
			return false
	ready = false
	return true


func _volume(size: Vector3i, format: int) -> RID:
	var tf := RDTextureFormat.new()
	tf.texture_type = RenderingDevice.TEXTURE_TYPE_3D
	tf.width = size.x
	tf.height = size.y
	tf.depth = size.z
	tf.format = format
	tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT
	return _rd.texture_create(tf, RDTextureView.new())


## The next build dispatch (its own compute list, the grid block updated before it); the last one sets `ready` and
## frees the ping-pong volume.
func _build_step(set0: Array[RDUniform], samplers: Dictionary) -> void:
	var phase: Dictionary = _phases[0]
	var first: int = phase["next"]
	var slices: int = mini(phase["slices"], phase["total"] - first)
	_rd.buffer_update(_grid_buffer, 0, GRID_BLOCK_SIZE, _grid_block(phase["pass"], first))
	var kernel: String = phase["kernel"]
	var nearest: RID = samplers["false|false|false"]
	var set1: Array[RDUniform] = [CloudEffect._ubo(_grid_buffer)]
	var size: Vector3i = _plan["occupancy"]["tex"]
	if kernel == "occupancy":
		set1.append_array([CloudEffect._image(1, _textures[phase["out"]]),
				CloudEffect._tex(2, nearest, _textures[phase["in"]])])
	else:
		if phase["pass"] == 0:
			size = _plan["bake"]["tex"]
		set1.append_array([CloudEffect._image(1, _textures["bake"]), CloudEffect._image(3, _textures["light"])])
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _pipelines[kernel])
	_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders[kernel], 0, set0), 0)
	_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders[kernel], 1, set1), 1)
	_rd.compute_list_dispatch(list, (size.x - 1) / 8 + 1, (size.y - 1) / 8 + 1, slices)
	_rd.compute_list_end()
	phase["next"] = first + slices
	if phase["next"] >= phase["total"]:
		_phases.pop_front()
	if _phases.is_empty():
		_rd.free_rid(_textures["occupancy_a"])
		_textures.erase("occupancy_a")
		ready = true
		print("CloudSpike: volumes built")


## The CloudSpikeGrid block (cloud_spike.gdshaderinc) for a build pass and its first slice.
func _grid_block(pass_index: int, first_slice: int) -> PackedByteArray:
	var p := _plan
	var occupancy: Dictionary = p["occupancy"]
	var bake: Dictionary = p["bake"]
	var up0: Vector3 = p["up0"]
	var axis_x: Vector3 = p["axis_x"]
	var axis_y: Vector3 = p["axis_y"]
	var min_xy: Vector2 = p["min_xy"]
	var z: Vector2 = p["z"]
	var floats := PackedFloat32Array([up0.x, up0.y, up0.z, OCCUPANCY_CELL_M, axis_x.x, axis_x.y, axis_x.z, min_xy.x,
			axis_y.x, axis_y.y, axis_y.z, min_xy.y, z.x, z.y, p["bake_cell"], R_NEAR_M])
	var bytes := floats.to_byte_array()
	var ints := PackedInt32Array()
	for layout: Dictionary in [occupancy, bake]:
		var cells: Vector3i = layout["cells"]
		var tiles: Vector2i = layout["tiles"]
		var tex: Vector3i = layout["tex"]
		ints.append_array([cells.x, cells.y, cells.z, tiles.x, tex.x, tex.y, tex.z, tiles.y])
	ints.append_array([pass_index, first_slice, int(mode), DISTANCE_CAP_CELLS])
	bytes.append_array(ints.to_byte_array())
	bytes.append_array(PackedFloat32Array([p["pixel_angle"], 0.0, 0.0, 0.0]).to_byte_array())
	return bytes
