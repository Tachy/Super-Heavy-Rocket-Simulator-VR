class_name CloudShadowEffect
extends CompositorEffect
## The cloud shadow maps (M8 weather and clouds, Task 9): before the opaque pass, one compute kernel
## (shaders/cloud_shadow.gdshaderinc with atmosphere.gdshaderinc and clouds.gdshaderinc, ported by CloudShaderPort)
## fills two maps along the Sun - the near one with the visible clouds' shapes, the far one with the grid mean - which
## the Earth's surfaces sample in the same frame (shaders/cloud_light.gdshaderinc). Their frame (anchors, axes, the
## Sun, extents) comes from the `cloud_shadow_*` globals CloudLayer sets on the main thread before the frame is drawn,
## so the kernel and the surfaces read the same numbers. View-independent: dispatched once per frame for all views.
## Without a RenderingDevice (headless) it does nothing.

const FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_shadow.gdshaderinc"]
const GROUP := 8
const BLOCK_SIZE := 4 * 16

## The maps (Texture2DRD; their RD textures are made on the render thread, bound by bind_textures on the main thread).
var near_texture := Texture2DRD.new()
var far_texture := Texture2DRD.new()
## Engine.get_frames_drawn() of the last frame rendered with the maps (render thread writes, main thread reads); the
## layer takes the maps as valid only a few frames after it. Counted in frames, not wall-clock time (world-volume plan
## Task 7): a slow frame (a check's ~100 ms full-resolution march) outlasted a millisecond window and switched the
## ground's and the air's cloud shadows off and on between frames of an unchanged scene.
var ran_frame := -1000000
## Checks only (cloud_check.gd): the near map with the grid mean too (the maps' machinery against the CPU's
## Beer-Lambert, without the shapes' local departures from the cell mean).
var grid_mean_only := false
## Render-thread GPU-side numbers of the last frame (tests): {record_ms}.
var stats := {}

var _rd: RenderingDevice
var _port: CloudShaderPort
var _shader := RID()
var _pipeline := RID()
var _samplers := {}
var _fallback := {}
var _globals_buffer := RID()
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
	_port = CloudShaderPort.build(PackedStringArray(["CLOUD_SHADOW_MAP"]), PackedStringArray(FILES))
	var src := RDShaderSource.new()
	src.language = RenderingDevice.SHADER_LANGUAGE_GLSL
	src.source_compute = _port.source
	var spirv := _rd.shader_compile_spirv_from_source(src)
	if spirv.compile_error_compute != "":
		push_error("CloudShadowEffect: " + spirv.compile_error_compute)
		_rd = null
		return
	_shader = _rd.shader_create_from_spirv(spirv)
	_pipeline = _rd.compute_pipeline_create(_shader)
	_globals_buffer = _rd.uniform_buffer_create(_port.block_size)
	for linear in [false, true]:
		for repeat in [false, true]:
			var state := RDSamplerState.new()
			var filter := RenderingDevice.SAMPLER_FILTER_LINEAR if linear else RenderingDevice.SAMPLER_FILTER_NEAREST
			state.min_filter = filter
			state.mag_filter = filter
			var mode := RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT if repeat \
					else RenderingDevice.SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE
			state.repeat_u = mode
			state.repeat_v = mode
			state.repeat_w = mode
			_samplers["%s|%s|false" % [linear, repeat]] = _rd.sampler_create(state)
	_samplers["true|true|true"] = _rd.sampler_create(CloudShaderPort.mipmap_sampler_state())
	for type in ["sampler2D", "sampler3D"]:
		var tf := RDTextureFormat.new()
		tf.texture_type = RenderingDevice.TEXTURE_TYPE_2D if type == "sampler2D" else RenderingDevice.TEXTURE_TYPE_3D
		tf.width = 1
		tf.height = 1
		tf.depth = 1
		tf.format = RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
		_fallback[type] = _rd.texture_create(tf, RDTextureView.new(), [PackedByteArray([0, 0, 0, 0])])
	var maps: Array[RID] = []
	for i in range(2):
		var tf := RDTextureFormat.new()
		tf.texture_type = RenderingDevice.TEXTURE_TYPE_2D
		tf.width = _size
		tf.height = _size
		tf.format = RenderingDevice.DATA_FORMAT_R32G32B32A32_SFLOAT
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT
		maps.append(_rd.texture_create(tf, RDTextureView.new()))
		_blocks.append(_rd.uniform_buffer_create(BLOCK_SIZE))
	_mutex.lock()
	_maps = maps
	_mutex.unlock()


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
		if _globals_buffer.is_valid():
			_rd.free_rid(_globals_buffer)
		if _shader.is_valid():
			_rd.free_rid(_shader)


func _render_callback(callback_type: int, _render_data: RenderData) -> void:
	# diagnostics (terrain_perf --gpu_passes): GPU timestamps around this pass, named by the interval they start
	if CloudEffect.capture_timestamps and _rd != null:
		_rd.capture_timestamp("shadow_pass")
	_render_pass(callback_type, _render_data)
	if CloudEffect.capture_timestamps and _rd != null:
		_rd.capture_timestamp("opaque_and_sky")


func _render_pass(_callback_type: int, _render_data: RenderData) -> void:
	if _rd == null or _maps.size() != 2 or not ShaderGlobals.value(&"cloud_enabled"):
		return
	var start_us := Time.get_ticks_usec()
	_rd.buffer_update(_globals_buffer, 0, _port.block_size, _port.pack())
	var set0: Array[RDUniform] = []
	var block := RDUniform.new()
	block.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	block.binding = 0
	block.add_id(_globals_buffer)
	set0.append(block)
	var textures := _port.sampler_textures()
	for i in range(_port.samplers.size()):
		var s: Dictionary = _port.samplers[i]
		var u := RDUniform.new()
		u.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
		u.binding = s["binding"]
		u.add_id(_samplers["%s|%s|%s" % [s["linear"], s["repeat"], s["mipmap"]]])
		var texture: RID = textures[i]
		u.add_id(texture if texture.is_valid() else _fallback.get(s["type"], _fallback["sampler2D"]))
		set0.append(u)
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
		var bytes := PackedFloat32Array([anchor.x, anchor.y, anchor.z, halves[i], e1.x, e1.y, e1.z,
				1.0 if i == 0 and not grid_mean_only else 0.0, e2.x, e2.y, e2.z, float(_size), sun.x, sun.y, sun.z, 0.0])
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
		_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shader, 1, set1), 1)
		_rd.compute_list_dispatch(list, groups, groups, 1)
	_rd.compute_list_end()
	ran_frame = Engine.get_frames_drawn()
	stats = {"record_ms": (Time.get_ticks_usec() - start_us) / 1000.0}
