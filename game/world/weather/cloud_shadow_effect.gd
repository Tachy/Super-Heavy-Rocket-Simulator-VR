class_name CloudShadowEffect
extends CompositorEffect
## The cloud shadow maps (M8 weather and clouds, Task 9; from the baked volume since 2026-10-30): before the opaque
## pass one compute kernel (cloud_baked.gdshaderinc CLOUD_SHADOW_BAKED_FILL with atmosphere.gdshaderinc and
## clouds.gdshaderinc, ported by CloudShaderPort) fills two maps along the Sun - the near one and the far one, each
## texel one read of the baked volume's Sun transmittance S (the shapes it draws; the slanted column beyond its
## levels) - which the Earth's surfaces sample in the same frame (shaders/cloud_light.gdshaderinc). Their frame (anchors,
## axes, the Sun, extents) comes from the `cloud_shadow_*` globals CloudLayer sets on the main thread before the frame
## is drawn, so the kernel and the surfaces read the same numbers. View-independent: dispatched once per frame for all
## views. It runs only while the baked volume draws (CloudBaked.shadow_fill); otherwise - headless, the kernels still
## being built, before the first set - the layer takes the shadows from the column (cloud_shadow_valid off).
## Its kernel is built on workers (CloudKernels: the driver binary cached on disk; fast start 2026-10-30).
## The Sun probe (2026-10-30, the vehicles' and the eye's direct Sun): the same S read at up to PROBE_MAX points the
## main thread names (request_sun_probe), a tiny dispatch after the maps' and an asynchronous read-back - its result
## arrives one to two frames later (sun_probe), never a stall.

const FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_baked.gdshaderinc"]
const GROUP := 8
const BLOCK_SIZE := 4 * 16
## The Sun probe's points at most (one work group; cloud_baked.gdshaderinc CLOUD_BAKED_SUN_PROBE).
const PROBE_MAX := 64

## The maps (Texture2DRD; their RD textures are made on the render thread, bound by bind_textures on the main thread).
var near_texture := Texture2DRD.new()
var far_texture := Texture2DRD.new()
## Engine.get_frames_drawn() of the last frame rendered with the maps (render thread writes, main thread reads); the
## layer takes the maps as valid only a few frames after it. Counted in frames, not wall-clock time: a slow frame (a check's ~100 ms full-resolution march) outlasted a millisecond window and switched the
## ground's and the air's cloud shadows off and on between frames of an unchanged scene.
var ran_frame := -1000000
## Render-thread GPU-side numbers of the last frame (tests): {record_ms}.
var stats := {}
## The cloud pass (its baked volume fills the maps while it draws; set by CloudLayer).
var cloud_effect: CloudEffect

var _rd: RenderingDevice
var _kernels: CloudKernels  ## the fill's kernel, being built (null once created; under _mutex)
var _failed := false  ## the kernel failed: the pass stays off (the RIDs are kept for the predelete)
var _port: CloudShaderPort
var _shader := RID()
var _pipeline := RID()
var _globals_buffer := RID()
var _probe_shader := RID()
var _probe_pipeline := RID()
var _probe_in := RID()  ## the probe's points (storage buffer)
var _probe_out := RID()  ## its transmittances (storage buffer, read back asynchronously)
var _probe := Probe.new()


## The Sun probe's state; the read-back's callback is its method (a Callable on an object holds no reference: a callback
## that fires after the effect and this state are gone - quitting with a read-back in flight - is not called; a
## lambda's captures outlived the scripts at the engine's exit and crashed it).
class Probe:
	extends RefCounted
	var mutex := Mutex.new()  ## guards the three below (main thread / render thread)
	var request := {}  ## key -> the point (Earth-local render axes, float64 m), the main thread's latest
	var results := {}  ## key -> the last read-back transmittance (-1: beyond the volume levels)
	var busy := false  ## a read-back is in flight
	var generation := 0  ## raised by every clear: a read-back dispatched before it is dropped

	## The read-back of the transmittances of `keys` (in their order).
	func take(data: PackedByteArray, keys: Array, dispatched_generation: int) -> void:
		var values := data.to_float32_array()
		mutex.lock()
		if dispatched_generation == generation:
			for i in range(mini(keys.size(), values.size())):
				results[keys[i]] = values[i]
		busy = false
		mutex.unlock()
var _samplers := {}
var _fallback := {}
var _blocks: Array[RID] = []
var _maps: Array[RID] = []
var _size := 512
var _mutex := Mutex.new()
var _bound := false


func _init(size := 512) -> void:
	_size = size
	effect_callback_type = EFFECT_CALLBACK_TYPE_PRE_OPAQUE
	RenderingServer.call_on_render_thread(_create)


## Main thread: gives the maps' RD textures to the Texture2DRDs once they exist. True when bound.
func bind_textures() -> bool:
	if _bound:
		return true
	_mutex.lock()
	var maps := _maps.duplicate()
	_mutex.unlock()
	if maps.size() != 2:
		return false
	near_texture.texture_rd_rid = maps[0]
	far_texture.texture_rd_rid = maps[1]
	_bound = true
	return true


func _create() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	var defines := PackedStringArray(["CLOUD_SHADOW_BAKED_FILL"])
	defines.append_array(CloudBaked.extra_defines)  # (diagnostics: CLOUD_SHADOW_BAKED_DIAG)
	var probe := defines.duplicate()
	probe.append("CLOUD_BAKED_SUN_PROBE")
	var kernels := CloudKernels.new([["baked_fill", defines, PackedStringArray(FILES), true],
			["sun_probe", probe, PackedStringArray(FILES), true]], "shadow maps")
	_mutex.lock()
	_kernels = kernels
	_mutex.unlock()
	_samplers = CloudRD.create_samplers(_rd)
	_fallback = CloudRD.create_fallbacks(_rd)
	var maps: Array[RID] = []
	for i in range(2):
		var tf := RDTextureFormat.new()
		tf.texture_type = RenderingDevice.TEXTURE_TYPE_2D
		tf.width = _size
		tf.height = _size
		tf.format = RenderingDevice.DATA_FORMAT_R32G32B32A32_SFLOAT
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT 				| RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT  # (checks: render_check --shadow_dump)
		maps.append(_rd.texture_create(tf, RDTextureView.new()))
		_blocks.append(_rd.uniform_buffer_create(BLOCK_SIZE))
	_mutex.lock()
	_maps = maps
	_mutex.unlock()
	_probe_in = _rd.storage_buffer_create(32 + 16 * PROBE_MAX)
	_probe_out = _rd.storage_buffer_create(4 * PROBE_MAX)


## Render thread: the kernel created once built (false while building, while a rejected cached binary is rebuilt, or
## failed).
func _kernels_ready() -> bool:
	if _failed:
		return false
	if _kernels == null:
		return true
	if not _kernels.done():
		return false
	var shader := _kernels.create_shader(_rd, "baked_fill")
	var probe := _kernels.create_shader(_rd, "sun_probe")
	if not (shader.is_valid() and probe.is_valid()) and _kernels.rebuild_rejected():
		for rid: RID in [shader, probe]:
			if rid.is_valid():
				_rd.free_rid(rid)
		return false
	var kernels := _kernels
	_mutex.lock()
	_kernels = null
	_mutex.unlock()
	if not (shader.is_valid() and probe.is_valid()):
		_failed = true
		for rid: RID in [shader, probe]:
			if rid.is_valid():
				_rd.free_rid(rid)
		return false
	_probe_shader = probe
	_probe_pipeline = _rd.compute_pipeline_create(probe)
	_port = kernels.port("baked_fill")
	_shader = shader
	_pipeline = _rd.compute_pipeline_create(_shader)
	_globals_buffer = _rd.uniform_buffer_create(_port.block_size)
	return true


## Main thread (the loading screen's hold): true while the kernels are being built.
func building() -> bool:
	_mutex.lock()
	var kernels := _kernels
	_mutex.unlock()
	return kernels != null and not kernels.done()


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd != null:
		var rids: Array = []
		rids.append_array(_maps)
		rids.append_array(_blocks)
		rids.append_array(_fallback.values())
		rids.append_array(_samplers.values())
		for rid: RID in rids:
			if rid.is_valid():
				_rd.free_rid(rid)
		for rid: RID in [_globals_buffer, _shader, _probe_shader, _probe_in, _probe_out]:
			if rid.is_valid():
				_rd.free_rid(rid)


func _render_callback(callback_type: int, _render_data: RenderData) -> void:
	# diagnostics (terrain_perf --gpu_passes): GPU timestamps around this pass, named by the interval they start
	if CloudEffect.capture_timestamps and _rd != null:
		_rd.capture_timestamp("shadow_pass")
	_render_pass(callback_type, _render_data)
	if CloudEffect.capture_timestamps and _rd != null:
		_rd.capture_timestamp("opaque_and_sky")


## Main thread: the Sun probe at this point (Earth-local render axes, m; Frames.to_render of the Earth-fixed point)
## under `key` - kept and read each frame until the key is asked for another point.
func request_sun_probe(key: String, point_render_m: Vector3) -> void:
	_probe.mutex.lock()
	if _probe.request.has(key) or _probe.request.size() < PROBE_MAX:
		_probe.request[key] = point_render_m
	_probe.mutex.unlock()


## Main thread: the last read-back of the probe `key` - the Sun's cloud transmittance from the baked volume's S at its
## point one to two frames ago; NAN while none (no set shown, the kernels building, the point beyond the volume levels).
func sun_probe(key: String) -> float:
	_probe.mutex.lock()
	var t: float = _probe.results.get(key, -1.0)
	_probe.mutex.unlock()
	# (a value counts only while the pass keeps running: a stopped compositor must not leave the last one for good)
	if Engine.get_frames_drawn() - ran_frame > CloudLayer.SHADOW_RUN_WINDOW_FRAMES:
		return NAN
	return t if t >= 0.0 else NAN


## Checks (render_check --shadow_dump): map i's texels (0 near, 1 far; raw RGBA32F); empty without the maps.
func debug_map_data(i: int) -> PackedByteArray:
	if _rd == null or _maps.size() != 2:
		return PackedByteArray()
	return _rd.texture_get_data(_maps[i], 0)


## The baked volume's fill uniforms (CloudBaked.shadow_fill) while it draws; empty otherwise.
func _baked_fill_uniforms() -> Dictionary:
	if cloud_effect == null or cloud_effect.baked == null or not cloud_effect.baked_shown:
		return {}
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	return cloud_effect.baked.shadow_fill(hi + lo, _samplers["true|false|false"])


func _render_pass(_callback_type: int, _render_data: RenderData) -> void:
	var fill := {}
	if _rd != null and _maps.size() == 2 and ShaderGlobals.value(&"cloud_enabled") and _kernels_ready():
		fill = _baked_fill_uniforms()
	if fill.is_empty():  # (the maps go invalid: the layer takes the column; the probe has no value)
		_probe.mutex.lock()
		_probe.results.clear()
		_probe.generation += 1
		_probe.mutex.unlock()
		return
	var start_us := Time.get_ticks_usec()
	var set0 := CloudRD.set0(_rd, _port, _globals_buffer, _samplers, _fallback)
	var uniform_set0 := UniformSetCacheRD.get_cache(_shader, 0, set0)
	var e1: Vector3 = ShaderGlobals.value(&"cloud_shadow_e1")
	var e2: Vector3 = ShaderGlobals.value(&"cloud_shadow_e2")
	var sun: Vector3 = ShaderGlobals.value(&"cloud_shadow_sun")
	var extent: Vector4 = ShaderGlobals.value(&"cloud_shadow_extent")
	var anchors := [ShaderGlobals.value(&"cloud_shadow_near_anchor"), ShaderGlobals.value(&"cloud_shadow_far_anchor")]
	var halves := [extent.x, extent.y]
	var groups := (_size - 1) / GROUP + 1
	for i in range(2):  # the blocks first: no buffer updates inside a compute list
		var anchor: Vector3 = anchors[i]
		var sun_w := 1.0 if fill["fade"] else 0.0
		var bytes := PackedFloat32Array([anchor.x, anchor.y, anchor.z, halves[i], e1.x, e1.y, e1.z,
				0.0, e2.x, e2.y, e2.z, float(_size), sun.x, sun.y, sun.z, sun_w])
		_rd.buffer_update(_blocks[i], 0, BLOCK_SIZE, bytes.to_byte_array())
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _pipeline)
	_rd.compute_list_bind_uniform_set(list, uniform_set0, 0)
	for i in range(2):
		var ubo := RDUniform.new()
		ubo.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
		ubo.binding = 0
		ubo.add_id(_blocks[i])
		var image := RDUniform.new()
		image.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
		image.binding = 1
		image.add_id(_maps[i])
		var set1: Array[RDUniform] = [ubo, image]
		set1.append_array(fill["uniforms"])
		_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shader, 1, set1), 1)
		_rd.compute_list_dispatch(list, groups, groups, 1)
	_rd.compute_list_end()
	_dispatch_probe(set0, fill)
	ran_frame = Engine.get_frames_drawn()
	stats = {"record_ms": (Time.get_ticks_usec() - start_us) / 1000.0}


## Render thread: the Sun probe's points read from S (one work group) and read back asynchronously - unless the last
## read-back is still in flight (then the next frame's).
func _dispatch_probe(set0: Array[RDUniform], fill: Dictionary) -> void:
	_probe.mutex.lock()
	var keys: Array = _probe.request.keys()
	var points: Array = _probe.request.values()
	var busy := _probe.busy
	var generation := _probe.generation
	_probe.mutex.unlock()
	if busy or keys.is_empty() or not _probe_pipeline.is_valid():
		return
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var camera := hi + lo
	var sun: Vector3 = (ShaderGlobals.value(&"atmosphere_sun_direction") as Vector3).normalized()
	var floats := PackedFloat32Array([sun.x, sun.y, sun.z, 1.0 if fill["fade"] else 0.0, keys.size(), 0.0, 0.0, 0.0])
	for point: Vector3 in points:
		var offset := point - camera  # (float64: the offset is exact before it becomes float32)
		floats.append_array([offset.x, offset.y, offset.z, 0.0])
	_rd.buffer_update(_probe_in, 0, floats.size() * 4, floats.to_byte_array())
	var set1: Array[RDUniform] = [CloudRD.storage(0, _probe_in), CloudRD.storage(1, _probe_out)]
	set1.append_array(fill["uniforms"])
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _probe_pipeline)
	_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_probe_shader, 0, set0), 0)
	_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_probe_shader, 1, set1), 1)
	_rd.compute_list_dispatch(list, 1, 1, 1)
	_rd.compute_list_end()
	_probe.mutex.lock()
	_probe.busy = true
	_probe.mutex.unlock()
	var error := _rd.buffer_get_data_async(_probe_out, _probe.take.bind(keys, generation), 0, 4 * keys.size())
	if error != OK:  # (no callback is queued: free the probe for the next frame)
		_probe.mutex.lock()
		_probe.busy = false
		_probe.mutex.unlock()
