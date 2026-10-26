class_name CloudLightCache
extends RefCounted
## The light cache (static clouds, Task 1; shaders/cloud_light_cache.gdshaderinc): one cascade - an Earth-fixed box of
## size voxels over extent_m (east, north, the shell's height from SHELL_BOTTOM_M) anchored on the ground below the
## camera - holding tau_light toward the Sun at its voxel centres (R32F), filled by the CLOUD_LIGHT_FILL kernel with a
## converged deterministic light march (SAMPLES near + far, the stratified midpoints). The cloud pass looks the light up
## instead of marching it per sample (cloud_pass.gdshaderinc cloud_light_source; outside the box, and for the Moon, the
## march stays).
## Threads: CloudLayer (main thread) asks for a fill with request() whenever the anchor, the Sun or the field changed
## beyond its thresholds; CloudEffect (render thread) calls update() before the march, which fills the pending request
## (all voxels at once in Task 1) and reports the box's frame for this frame's lookup (lookup_globals) - the march only
## ever reads a complete fill together with the frame it was filled for.

const FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_light_cache.gdshaderinc"]
const GROUP := 4
const BLOCK_SIZE := 8 * 16
## The shell's bottom (cloud_pass.gdshaderinc CLOUD_BOTTOM_M: below every ground) - the box's lowest height.
const SHELL_BOTTOM_M := -500.0
## [E] The fill's light march (spec A: the cache is the converged reference, not a noisy one): 96 near samples through
## the shapes and 32 far ones through the grid mean, the preset's intervals (first interval, near length:
## CloudLayer.PRESETS) - deterministic: the stratified midpoints (jitter 0.5). Measured (cloud_check.gd section 8,
## broken cf 0.4, Sun 20 deg up): 24 + 8 left a pixel 30 % off the converged integral (the geometric intervals reach
## ~740 m, one point each across a cumulus), 48 + 16 10.6 %, 96 + 32 < 9 % (the cache's voxels included); the fill
## is paid once per refresh (5-8 ms GPU for the whole cascade on an RTX 4090, the check's slabs).
const SAMPLES := Vector2i(96, 32)

var size := Vector3i.ZERO  ## voxels (east, north, up)
var extent_m := Vector3.ZERO  ## (east, north, shell height) [m]
var texture := RID()  ## R32F tau_light at the voxel centres (render thread)
## The light march's first interval and near length [m] of the fill (CloudLayer sets them from its preset).
var march_first_m := 20.0
var march_near_m := 6000.0

var _port: CloudShaderPort
var _shader := RID()
var _pipeline := RID()
var _globals_buffer := RID()
var _fill_buffer := RID()
var _failed := false
## The fill in progress / complete (render thread): anchor (Earth-local, m), axes, light direction, next layer.
var _anchor := Vector3(NAN, NAN, NAN)
var _east := Vector3.RIGHT
var _north := Vector3.BACK
var _up := Vector3.UP
var _light := Vector3.ZERO
var _next_layer := 0
var _complete := false
## The request (main thread writes, render thread reads): {anchor, light}, its serial and the serial filled.
var _mutex := Mutex.new()
var _request := {}
var _request_serial := 0
var _filled_serial := -1


func _init(voxels: Vector3i, extent: Vector3) -> void:
	size = voxels
	extent_m = extent


func voxel_count() -> int:
	return size.x * size.y * size.z


## Main thread: (re)fill the box anchored at anchor_earth_m (a ground point, Earth-local render axes, m) for the light
## direction (Earth-local, unit) on the next frame - also with an unchanged anchor and light (the field changed).
## One cascade without generations (Task 1): the fill is complete within that frame (all voxels).
func request(anchor_earth_m: Vector3, light_dir_earth: Vector3) -> void:
	_mutex.lock()
	_request = {"anchor": anchor_earth_m, "light": light_dir_earth.normalized()}
	_request_serial += 1
	_mutex.unlock()


## Render thread, before the march: fills the pending request (all voxels: one cascade at once, Task 1).
## samplers / fallback: the effect's sampler states ("linear|repeat|mipmap") and fallback textures (by sampler type).
func update(rd: RenderingDevice, samplers: Dictionary, fallback: Dictionary) -> void:
	_mutex.lock()
	var pending := _request.duplicate()
	var serial := _request_serial
	_mutex.unlock()
	if pending.is_empty() or serial == _filled_serial:
		return
	_begin(pending["anchor"], pending["light"])
	if fill(rd, pending["anchor"], pending["light"], voxel_count(), samplers, fallback):
		_filled_serial = serial


## Render thread: fills up to budget_voxels voxels (whole layers, at least one) of the box anchored at anchor_earth_m
## for light_dir_earth; a new anchor or light starts the fill over. True when the box is complete.
func fill(rd: RenderingDevice, anchor_earth_m: Vector3, light_dir_earth: Vector3, budget_voxels: int,
		samplers := {}, fallback := {}) -> bool:
	if not _ensure(rd):
		return false
	if anchor_earth_m != _anchor or light_dir_earth != _light:
		_begin(anchor_earth_m, light_dir_earth)
	if _complete:
		return true
	var layers := clampi(budget_voxels / maxi(size.x * size.y, 1), 1, size.z - _next_layer)
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var origin := _anchor - (hi + lo)  # float64: the anchor from the camera
	var block := PackedFloat32Array([origin.x, origin.y, origin.z, extent_m.x / size.x,
			_east.x, _east.y, _east.z, _next_layer,
			_north.x, _north.y, _north.z, _next_layer + layers,
			_up.x, _up.y, _up.z, 0.0,
			extent_m.x, extent_m.y, SHELL_BOTTOM_M, SHELL_BOTTOM_M + extent_m.z,
			_light.x, _light.y, _light.z, 0.0,
			march_first_m, CloudLayer.light_march_ratio(march_first_m, march_near_m, SAMPLES.x), march_near_m, 0.0])
	var bytes := block.to_byte_array()
	var samples := PackedInt32Array([SAMPLES.x, SAMPLES.y, 0, 0]).to_byte_array()
	bytes.append_array(samples)
	rd.buffer_update(_fill_buffer, 0, BLOCK_SIZE, bytes)
	rd.buffer_update(_globals_buffer, 0, _port.block_size, _port.pack())
	var set0: Array[RDUniform] = []
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	u.binding = 0
	u.add_id(_globals_buffer)
	set0.append(u)
	var textures := _port.sampler_textures()
	for i in range(_port.samplers.size()):
		var s: Dictionary = _port.samplers[i]
		var t := RDUniform.new()
		t.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
		t.binding = s["binding"]
		t.add_id(samplers["%s|%s|%s" % [s["linear"], s["repeat"], s["mipmap"]]])
		var tex: RID = textures[i]
		if s["id"] == &"cloud_light_cache" or not tex.is_valid():  # never the box being written
			tex = fallback.get(s["type"], fallback["sampler2D"])
		t.add_id(tex)
		set0.append(t)
	var fill_block := RDUniform.new()
	fill_block.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	fill_block.binding = 0
	fill_block.add_id(_fill_buffer)
	var image := RDUniform.new()
	image.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	image.binding = 1
	image.add_id(texture)
	var list := rd.compute_list_begin()
	rd.compute_list_bind_compute_pipeline(list, _pipeline)
	rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shader, 0, set0), 0)
	rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shader, 1, [fill_block, image]), 1)
	rd.compute_list_dispatch(list, (size.x - 1) / GROUP + 1, (size.y - 1) / GROUP + 1, (layers - 1) / GROUP + 1)
	rd.compute_list_end()
	_next_layer += layers
	_complete = _next_layer >= size.z
	return _complete


## Render thread: the lookup's globals for this frame (CloudShaderPort.pack overrides) - the box's frame relative to the
## camera (float64), valid only for a complete fill.
func lookup_globals(camera_earth_m: Vector3) -> Dictionary:
	var valid := _complete and texture.is_valid()
	return {&"cloud_light_cache_valid": valid,
		&"cloud_light_cache_origin": (_anchor - camera_earth_m) if valid else Vector3.ZERO,
		&"cloud_light_cache_east": _east, &"cloud_light_cache_north": _north,
		&"cloud_light_cache_extent": Vector4(extent_m.x, extent_m.y, SHELL_BOTTOM_M, SHELL_BOTTOM_M + extent_m.z),
		&"cloud_light_cache_size": Vector3(size)}


## Render thread: the anchor and light of the complete fill (tests; anchor NaN before the first).
func filled() -> Dictionary:
	return {"complete": _complete, "anchor": _anchor, "light": _light}


## Render thread (the effect's teardown): frees the cache's GPU resources.
func free_resources(rd: RenderingDevice) -> void:
	if rd == null:
		return
	for rid: RID in [texture, _globals_buffer, _fill_buffer, _shader]:
		if rid.is_valid():
			rd.free_rid(rid)  # the shader frees its pipeline
	texture = RID()
	_shader = RID()


## Starts a fill over: the anchor's frame (east, north, geodetic up), the light, no layer done.
func _begin(anchor_earth_m: Vector3, light_dir_earth: Vector3) -> void:
	_anchor = anchor_earth_m
	_light = light_dir_earth
	_up = _geodetic_up(anchor_earth_m)
	_east = Vector3.UP.cross(_up)
	_east = _east.normalized() if _east.length_squared() > 1e-12 else Vector3.RIGHT
	_north = _up.cross(_east).normalized()
	_next_layer = 0
	_complete = false


## The geodetic up of an Earth-local point (render axes: polar axis +Y; WGS84, as CloudLayer's shadow frame).
static func _geodetic_up(p: Vector3) -> Vector3:
	var stretch := CloudLayer.POLAR_STRETCH
	return Vector3(p.x, p.y * stretch * stretch, p.z).normalized()


func _ensure(rd: RenderingDevice) -> bool:
	if _shader.is_valid():
		return true
	if _failed or rd == null:
		return false
	_port = CloudShaderPort.build(PackedStringArray(["CLOUD_LIGHT_FILL"]), PackedStringArray(FILES))
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
	_globals_buffer = rd.uniform_buffer_create(_port.block_size)
	_fill_buffer = rd.uniform_buffer_create(BLOCK_SIZE)
	var tf := RDTextureFormat.new()
	tf.texture_type = RenderingDevice.TEXTURE_TYPE_3D
	tf.width = size.x
	tf.height = size.y
	tf.depth = size.z
	tf.format = RenderingDevice.DATA_FORMAT_R32_SFLOAT
	tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT
	texture = rd.texture_create(tf, RDTextureView.new())
	return texture.is_valid()
