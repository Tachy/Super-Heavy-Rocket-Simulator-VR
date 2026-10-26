class_name CloudLightCache
extends RefCounted
## The light cache (static clouds Tasks 1-2; shaders/cloud_light_cache.gdshaderinc): camera-anchored cascades of
## tau_light toward the Sun over the cloud shell (CloudLightCacheSchedule: layout, re-anchoring, generations), one 3D
## texture per cascade and generation slot (textures[slot * cascades + cascade]), filled progressively by the
## CLOUD_LIGHT_FILL kernel with a converged deterministic light march (SAMPLES near + far, the stratified midpoints).
## The cloud pass looks the light up instead of marching it per sample (cloud_pass.gdshaderinc cloud_light_source;
## outside the cascades, and for the Moon, the march stays).
## Threads: CloudLayer (main thread) calls plan() every frame - the schedule decides this frame's jobs and the lookup
## state they lead to, handed over as one batch; CloudEffect (render thread) calls update() before the march, which runs
## the batch's jobs and only then adopts its state (lookup_globals) - the march never reads a voxel before its fill ran,
## nor with another frame than the one it was filled for. A batch not yet run holds the schedule back (no backlog).

const FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_light_cache.gdshaderinc"]
const GROUP := 4
const FILL_BLOCK_SIZE := 10 * 16
## [E] The fill's light march (spec A: the cache is the converged reference, not a noisy one): 96 near samples through
## the shapes and 32 far ones through the grid mean, the preset's intervals (first interval, near length:
## CloudLayer.PRESETS) - deterministic: the stratified midpoints (jitter 0.5). Measured (cloud_check.gd section 8,
## broken cf 0.4, Sun 20 deg up): 24 + 8 left a pixel 30 % off the converged integral (the geometric intervals reach
## ~740 m, one point each across a cumulus), 48 + 16 10.6 %, 96 + 32 < 9 % (the cache's voxels included).
const SAMPLES := Vector2i(96, 32)
## [E] The texel format: R16F (5e-4 relative: tau_light 10 +- 0.005, 0.5 % of the transmitted light; cloud_check.gd
## section 8 near the camera: broken cloud 2.7 / 4.6 % worst, Task 1's R32F 3.1 / 6.4 % on a wider view; the slab
## within 1.3 % in every cascade). VRAM: 5 cascades x 2 generations x 16 MB = 160 MB, inside the light cache's budget
## of 192 MB [E] (controller ruling 2026-10-26, raised from 128 MB for the x2 clipmap: a x8 one failed broken cloud in
## its far cascades by far more).
const FORMAT := RenderingDevice.DATA_FORMAT_R16_SFLOAT

var schedule := CloudLightCacheSchedule.new()
var textures: Array[RID] = []  ## per generation slot and cascade: [slot * cascades + cascade] (render thread)
## The light march's first interval and near length [m] of the fill (CloudLayer sets them from its preset).
var march_first_m := 20.0
var march_near_m := 6000.0
var last_fill_voxels := 0  ## voxels filled in the last update (render thread; diagnostics)
var planned_fill_frames := 0  ## plans that carried fill jobs (main thread; checks)

var _port: CloudShaderPort
var _shader := RID()
var _pipeline := RID()
var _globals_buffers: Array[RID] = []  ## per distinct blend of a batch
var _fill_buffers: Array[RID] = []  ## per job of a batch
var _failed := false
var _blend_offset := -1  ## cloud_blend's byte offset in the globals block
var _state := {}  ## the lookup state of the last batch run (render thread)
## The batch handed over (main thread writes, render thread takes): {jobs, state}; {} = none.
var _mutex := Mutex.new()
var _batch := {}
## What the schedule has not seen while a batch waited (main thread).
var _held_jump := false
var _held_delta_s := 0.0
## near_samples' cache (render thread): the first interval it holds and the samples per cascade.
var _near_first_m := -1.0  ## none yet
var _near_samples := PackedInt32Array()
var _near_ratio_first_m := -1.0
var _near_ratio_value := 1.0


## The near march's samples of cascade k at the preset's first interval (CloudLightCacheSchedule.near_samples; 0: pure;
## computed once per first interval: the render thread asks every frame).
func near_samples(k: int) -> int:
	var first := march_first_m
	if first != _near_first_m:
		var samples := PackedInt32Array()
		for i in range(CloudLightCacheSchedule.CASCADES.size()):
			samples.append(CloudLightCacheSchedule.near_samples(i, first))
		_near_samples = samples
		_near_first_m = first
	return _near_samples[k]


## The near march's geometric ratio at the preset's first interval (cached with near_samples).
func _near_ratio() -> float:
	near_samples(0)
	if _near_ratio_first_m != _near_first_m:
		_near_ratio_value = CloudLayer.light_march_ratio(_near_first_m, CloudLightCacheSchedule.NEAR_GEOMETRIC_M,
				CloudLightCacheSchedule.NEAR_GEOMETRIC_SAMPLES)
		_near_ratio_first_m = _near_first_m
	return _near_ratio_value


## The length of cascade k's near march [m] (0: pure).
func near_length_m(k: int) -> float:
	return CloudLightCacheSchedule.near_length_m(near_samples(k), march_first_m)


## All voxels of one generation.
func voxel_count() -> int:
	return CloudLightCacheSchedule.voxel_count()


## Bytes of all textures (both generations).
static func texture_bytes() -> int:
	var bytes_per_texel := 2 if FORMAT == RenderingDevice.DATA_FORMAT_R16_SFLOAT else 4
	return 2 * CloudLightCacheSchedule.voxel_count() * bytes_per_texel


## The texture of generation slot `slot` of cascade k (render thread; RID() before the first fill).
func texture(slot: int, k: int) -> RID:
	var i := slot * CloudLightCacheSchedule.CASCADES.size() + k
	return textures[i] if i < textures.size() else RID()


## Main thread, every frame: the schedule's next batch (ground point, light, blend as CloudLightCacheSchedule.update).
## While the last batch has not run, its jobs stay and the schedule waits (the jump and the time are kept for it).
func plan(ground_earth_m: Vector3, light_dir_earth: Vector3, blend: float, clock_jumped: bool, delta_s: float) -> void:
	_held_jump = _held_jump or clock_jumped
	_held_delta_s += delta_s
	_mutex.lock()
	var waiting := not _batch.is_empty() and not (_batch["jobs"] as Array).is_empty()
	_mutex.unlock()
	if waiting:
		return
	var jobs := schedule.update(ground_earth_m, light_dir_earth, blend, _held_jump, _held_delta_s)
	var state := schedule.lookup_state()
	_held_jump = false
	_held_delta_s = 0.0
	if not jobs.is_empty():
		planned_fill_frames += 1
	_mutex.lock()
	_batch = {"jobs": jobs, "state": state}
	_mutex.unlock()


## Main thread: invalidates everything at the next plan (the field changed).
func reset() -> void:
	schedule.reset()


## True when no jobs wait and the planned state holds every cascade's window (tests: cloud_check.gd waits for it, then
## one more frame for the render thread to adopt the state).
func settled() -> bool:
	_mutex.lock()
	var idle := _batch.is_empty() or (_batch["jobs"] as Array).is_empty()
	_mutex.unlock()
	return idle and schedule.complete()


## Render thread, before the march: runs the waiting batch's jobs, then adopts its state.
## samplers / fallback: the effect's sampler states ("linear|repeat|mipmap") and fallback textures (by sampler type).
func update(rd: RenderingDevice, samplers: Dictionary, fallback: Dictionary) -> void:
	last_fill_voxels = 0
	_mutex.lock()
	var batch := _batch
	_batch = {}
	_mutex.unlock()
	if batch.is_empty() or not _ensure(rd):
		return
	var jobs: Array[Dictionary] = batch["jobs"]
	if not jobs.is_empty():
		_run(rd, jobs, batch["state"], samplers, fallback)
	_state = batch["state"]


## Render thread: the lookup's globals for this frame (CloudShaderPort.pack overrides) - the frame relative to the
## camera (float64) and each cascade's state as of the last batch run.
func lookup_globals(camera_earth_m: Vector3) -> Dictionary:
	if _state.is_empty() or textures.is_empty():
		return {&"cloud_light_cache_valid": false}
	var any := false
	var result := {}
	var cascades: Array = _state["cascades"]
	for k in range(cascades.size()):
		var c: Dictionary = cascades[k]
		var held: Array = c["valid"]
		var columns: Array[Vector4] = []
		for r: Rect2i in held:
			any = any or r.has_area()
			columns.append(Vector4(r.position.x, r.position.y, r.end.x, r.end.y) if r.has_area()
					else Vector4(0.0, 0.0, -1.0, -1.0))
		var size: Vector3i = c["size"]
		var centre: Vector2 = c["centre"]
		result[StringName("cloud_light_cache_cascade%d" % k)] = Projection(
				Vector4(c["dx_m"], near_samples(k), size.z, c["weight"]), columns[0], columns[1],
				Vector4(centre.x, centre.y, c["radius_voxels"], c["band_voxels"]))
	result[&"cloud_light_cache_valid"] = any
	result[&"cloud_light_cache_near"] = Vector4(march_first_m, _near_ratio(),
			CloudLightCacheSchedule.NEAR_INTERVAL_M, CloudLightCacheSchedule.NEAR_GEOMETRIC_SAMPLES)
	result[&"cloud_light_cache_origin"] = (_state["origin"] as Vector3) - camera_earth_m
	result[&"cloud_light_cache_east"] = _state["east"]
	result[&"cloud_light_cache_north"] = _state["north"]
	result[&"cloud_light_cache_shell"] = Vector4(CloudLightCacheSchedule.SHELL_BOTTOM_M,
			CloudLightCacheSchedule.SHELL_TOP_M, CloudLightCacheSchedule.CASCADES[0]["size"].x, 0.0)
	return result


## Render thread (the effect's teardown): frees the cache's GPU resources.
func free_resources(rd: RenderingDevice) -> void:
	if rd == null:
		return
	for rid: RID in textures + _globals_buffers + _fill_buffers + [_shader]:
		if rid.is_valid():
			rd.free_rid(rid)  # the shader frees its pipeline
	textures.clear()
	_globals_buffers.clear()
	_fill_buffers.clear()
	_shader = RID()
	_state = {}


## Runs one batch's jobs in one compute list (all buffers updated first).
func _run(rd: RenderingDevice, jobs: Array[Dictionary], state: Dictionary, samplers: Dictionary,
		fallback: Dictionary) -> void:
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var origin := (state["origin"] as Vector3) - (hi + lo)  # float64: the frame's origin from the camera
	var east: Vector3 = state["east"]
	var north: Vector3 = state["north"]
	var up: Vector3 = state["up"]
	var globals := _port.pack()
	var blends: Array[float] = []
	var texels_across: int = CloudLightCacheSchedule.CASCADES[0]["size"].x
	var ratio := CloudLayer.light_march_ratio(march_first_m, march_near_m, SAMPLES.x)
	var job_blend: Array[int] = []
	for i in range(jobs.size()):
		var job: Dictionary = jobs[i]
		var k: int = job["cascade"]
		var cascade: Dictionary = state["cascades"][k]
		var dx: float = cascade["dx_m"]
		var layers: int = (cascade["size"] as Vector3i).z
		var region: AABB = job["region"]
		var light: Vector3 = job["light"]
		var blend := float(job["blend"])
		var b := blends.find(blend)
		if b < 0:
			b = blends.size()
			blends.append(blend)
			if b >= _globals_buffers.size():
				_globals_buffers.append(rd.uniform_buffer_create(_port.block_size))
			var bytes := globals.duplicate()
			if _blend_offset >= 0:
				bytes.encode_float(_blend_offset, blend)
			rd.buffer_update(_globals_buffers[b], 0, _port.block_size, bytes)
		job_blend.append(b)
		if i >= _fill_buffers.size():
			_fill_buffers.append(rd.uniform_buffer_create(FILL_BLOCK_SIZE))
		var block := PackedFloat32Array([origin.x, origin.y, origin.z, dx,
				east.x, east.y, east.z, dx,
				north.x, north.y, north.z, 0.0,
				up.x, up.y, up.z, 0.0,
				CloudLightCacheSchedule.SHELL_BOTTOM_M, CloudLightCacheSchedule.SHELL_TOP_M, layers, 0.0])
		var bytes := block.to_byte_array()
		bytes.append_array(PackedInt32Array([int(region.position.x), int(region.position.y), int(region.position.z),
				texels_across, int(region.size.x), int(region.size.y), int(region.size.z), 0]).to_byte_array())
		bytes.append_array(PackedFloat32Array([light.x, light.y, light.z, 0.0,
				march_first_m, ratio, march_near_m, 0.0]).to_byte_array())
		bytes.append_array(PackedInt32Array([SAMPLES.x, SAMPLES.y, 0, 0]).to_byte_array())
		rd.buffer_update(_fill_buffers[i], 0, FILL_BLOCK_SIZE, bytes)
		last_fill_voxels += int(region.get_volume())
	var sets0: Array[RID] = []
	for b in range(blends.size()):
		sets0.append(UniformSetCacheRD.get_cache(_shader, 0, _set0(_globals_buffers[b], samplers, fallback)))
	var list := rd.compute_list_begin()
	rd.compute_list_bind_compute_pipeline(list, _pipeline)
	for i in range(jobs.size()):
		var job: Dictionary = jobs[i]
		var region: AABB = job["region"]
		var fill_block := RDUniform.new()
		fill_block.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
		fill_block.binding = 0
		fill_block.add_id(_fill_buffers[i])
		var image := RDUniform.new()
		image.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
		image.binding = 1
		image.add_id(texture(int(job["generation"]), int(job["cascade"])))
		rd.compute_list_bind_uniform_set(list, sets0[job_blend[i]], 0)
		rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shader, 1, [fill_block, image]), 1)
		rd.compute_list_dispatch(list, (int(region.size.x) - 1) / GROUP + 1, (int(region.size.y) - 1) / GROUP + 1,
				(int(region.size.z) - 1) / GROUP + 1)
	rd.compute_list_end()


## Set 0 of the fill: the globals block and the samplers (the cache's own textures never: the fallback).
func _set0(globals_buffer: RID, samplers: Dictionary, fallback: Dictionary) -> Array[RDUniform]:
	var set0: Array[RDUniform] = []
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	u.binding = 0
	u.add_id(globals_buffer)
	set0.append(u)
	var sampler_textures := _port.sampler_textures()
	for i in range(_port.samplers.size()):
		var s: Dictionary = _port.samplers[i]
		var t := RDUniform.new()
		t.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
		t.binding = s["binding"]
		t.add_id(samplers["%s|%s|%s" % [s["linear"], s["repeat"], s["mipmap"]]])
		var tex: RID = sampler_textures[i]
		if String(s["id"]).begins_with("cloud_light_cache") or not tex.is_valid():  # never a texture being written
			tex = fallback.get(s["type"], fallback["sampler2D"])
		t.add_id(tex)
		set0.append(t)
	return set0


func _ensure(rd: RenderingDevice) -> bool:
	if _shader.is_valid():
		return true
	if _failed or rd == null:
		return false
	var defines := PackedStringArray(["CLOUD_LIGHT_FILL"])
	if FORMAT == RenderingDevice.DATA_FORMAT_R16_SFLOAT:
		defines.append("CLOUD_LIGHT_FILL_R16F")
	_port = CloudShaderPort.build(defines, PackedStringArray(FILES))
	for m: Dictionary in _port.members:
		if m["id"] == &"cloud_blend":
			_blend_offset = m["offset"]
	var src := RDShaderSource.new()
	src.language = RenderingDevice.SHADER_LANGUAGE_GLSL
	src.source_compute = _port.source
	var spirv := rd.shader_compile_spirv_from_source(src)
	if spirv.compile_error_compute != "":
		push_error("CloudLightCache fill: %s" % spirv.compile_error_compute)
		_failed = true
		return false
	_shader = rd.shader_create_from_spirv(spirv)
	if not _shader.is_valid():
		push_error("CloudLightCache fill: shader not created")
		_failed = true
		return false
	_pipeline = rd.compute_pipeline_create(_shader)
	for slot in range(2):
		for c: Dictionary in CloudLightCacheSchedule.CASCADES:
			var size: Vector3i = c["size"]
			var tf := RDTextureFormat.new()
			tf.texture_type = RenderingDevice.TEXTURE_TYPE_3D
			tf.width = size.x
			tf.height = size.y
			tf.depth = size.z
			tf.format = FORMAT
			tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT \
					| RenderingDevice.TEXTURE_USAGE_CAN_COPY_TO_BIT  # texture_clear
			var t := rd.texture_create(tf, RDTextureView.new())
			if not t.is_valid():
				push_error("CloudLightCache: texture not created")
				_failed = true
				return false
			# Never-filled texels are read with weight 0 at a held region's edge: zero, not undefined (NaN * 0 = NaN).
			rd.texture_clear(t, Color(0, 0, 0, 0), 0, 1, 0, 1)
			textures.append(t)
	return true
