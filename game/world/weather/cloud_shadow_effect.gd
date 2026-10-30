class_name CloudShadowEffect
extends CompositorEffect
## The cloud shadow maps (M8 weather and clouds, Task 9): before the opaque pass, one compute kernel
## (shaders/cloud_shadow.gdshaderinc with atmosphere.gdshaderinc and clouds.gdshaderinc, ported by CloudShaderPort)
## fills two maps along the Sun - the near one with the visible clouds' shapes, the far one with the grid mean - which
## the Earth's surfaces sample in the same frame (shaders/cloud_light.gdshaderinc). Their frame (anchors, axes, the
## Sun, extents) comes from the `cloud_shadow_*` globals CloudLayer sets on the main thread before the frame is drawn,
## so the kernel and the surfaces read the same numbers. View-independent: dispatched once per frame for all views.
## Without a RenderingDevice (headless) it does nothing.
## The baked clouds (CloudEffect.MarchMethod.BAKED, shadow step 2026-10-30): while the baked volume draws, the maps are
## filled from its own Sun transmittance instead (cloud_baked.gdshaderinc CLOUD_SHADOW_BAKED_FILL: one S read per
## texel, the same texel semantics) - the shadows of the shapes it draws, out to the far map's edge.
## Its kernels are built on workers (CloudKernels: SPIR-V cached on disk; fast start 2026-10-30): until they are
## created the pass does not run (the layer then takes the shadows from the column, cloud_shadow_valid off).

const FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_shadow.gdshaderinc"]
const GROUP := 8
const BLOCK_SIZE := 4 * 16
const BAKED_FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_baked.gdshaderinc"]

## The maps (Texture2DRD; their RD textures are made on the render thread, bound by bind_textures on the main thread).
var near_texture := Texture2DRD.new()
var far_texture := Texture2DRD.new()
## Engine.get_frames_drawn() of the last frame rendered with the maps (render thread writes, main thread reads); the
## layer takes the maps as valid only a few frames after it. Counted in frames, not wall-clock time: a slow frame (a check's ~100 ms full-resolution march) outlasted a millisecond window and switched the
## ground's and the air's cloud shadows off and on between frames of an unchanged scene.
var ran_frame := -1000000
## Checks only (cloud_check.gd): the near map with the grid mean too (the maps' machinery against the CPU's
## Beer-Lambert, without the shapes' local departures from the cell mean).
var grid_mean_only := false
## Render-thread GPU-side numbers of the last frame (tests): {record_ms}.
var stats := {}
## The cloud pass (its baked volume fills the maps while it draws; set by CloudLayer).
var cloud_effect: CloudEffect
## The last frame's fill: true when it came from the baked volume (checks, diagnostics).
var baked_fill := false

var _rd: RenderingDevice
var _kernels: CloudKernels  ## the map's kernel and the baked fill's, being built (null once created; under _mutex)
var _failed := false  ## the map's kernel failed: the pass stays off (the RIDs are kept for the predelete)
var _port: CloudShaderPort
var _shader := RID()
var _pipeline := RID()
var _baked_port: CloudShaderPort
var _baked_shader := RID()
var _baked_pipeline := RID()
var _baked_globals := RID()
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
	var specs := [["map", PackedStringArray(["CLOUD_SHADOW_MAP"]), PackedStringArray(FILES), true]]
	if CloudBaked.shadow:
		var defines := PackedStringArray(["CLOUD_SHADOW_BAKED_FILL"])
		defines.append_array(CloudBaked.extra_defines)  # (diagnostics: CLOUD_SHADOW_BAKED_DIAG)
		specs.append(["baked_fill", defines, PackedStringArray(BAKED_FILES), true])
	var kernels := CloudKernels.new(specs, "shadow maps")
	_mutex.lock()
	_kernels = kernels
	_mutex.unlock()
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
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT 				| RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT  # (checks: render_check --shadow_dump)
		maps.append(_rd.texture_create(tf, RDTextureView.new()))
		_blocks.append(_rd.uniform_buffer_create(BLOCK_SIZE))
	_mutex.lock()
	_maps = maps
	_mutex.unlock()


## Render thread: the kernels created once built (false while building or while a rejected cached binary is rebuilt;
## the map's failing switches the pass off, the baked fill's only the fill).
func _kernels_ready() -> bool:
	if _failed:
		return false
	if _kernels == null:
		return true
	if not _kernels.done():
		return false
	var shader := _kernels.create_shader(_rd, "map")
	var baked_shader := _kernels.create_shader(_rd, "baked_fill") if CloudBaked.shadow else RID()
	if (not shader.is_valid() or (CloudBaked.shadow and not baked_shader.is_valid())) and _kernels.rebuild_rejected():
		for rid: RID in [shader, baked_shader]:
			if rid.is_valid():
				_rd.free_rid(rid)
		return false
	var kernels := _kernels
	_mutex.lock()
	_kernels = null
	_mutex.unlock()
	if not shader.is_valid():
		_failed = true
		if baked_shader.is_valid():
			_rd.free_rid(baked_shader)
		return false
	_port = kernels.port("map")
	_shader = shader
	_pipeline = _rd.compute_pipeline_create(_shader)
	_globals_buffer = _rd.uniform_buffer_create(_port.block_size)
	if baked_shader.is_valid():
		_baked_port = kernels.port("baked_fill")
		_baked_shader = baked_shader
		_baked_pipeline = _rd.compute_pipeline_create(_baked_shader)
		_baked_globals = _rd.uniform_buffer_create(_baked_port.block_size)
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
		for rid: RID in [_globals_buffer, _shader, _baked_globals, _baked_shader]:
			if rid.is_valid():
				_rd.free_rid(rid)


func _render_callback(callback_type: int, _render_data: RenderData) -> void:
	# diagnostics (terrain_perf --gpu_passes): GPU timestamps around this pass, named by the interval they start
	if CloudEffect.capture_timestamps and _rd != null:
		_rd.capture_timestamp("shadow_pass")
	_render_pass(callback_type, _render_data)
	if CloudEffect.capture_timestamps and _rd != null:
		_rd.capture_timestamp("opaque_and_sky")


## Checks (render_check --shadow_dump): map i's texels (0 near, 1 far; raw RGBA32F); empty without the maps.
func debug_map_data(i: int) -> PackedByteArray:
	if _rd == null or _maps.size() != 2:
		return PackedByteArray()
	return _rd.texture_get_data(_maps[i], 0)


## Set 0 of a port's kernel: its globals block (filled now) and its sampler globals.
func _set0(port: CloudShaderPort, buffer: RID) -> Array[RDUniform]:
	_rd.buffer_update(buffer, 0, port.block_size, port.pack())
	var set0: Array[RDUniform] = []
	var block := RDUniform.new()
	block.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	block.binding = 0
	block.add_id(buffer)
	set0.append(block)
	var textures := port.sampler_textures()
	for i in range(port.samplers.size()):
		var s: Dictionary = port.samplers[i]
		var u := RDUniform.new()
		u.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
		u.binding = s["binding"]
		u.add_id(_samplers["%s|%s|%s" % [s["linear"], s["repeat"], s["mipmap"]]])
		var texture: RID = textures[i]
		u.add_id(texture if texture.is_valid() else _fallback.get(s["type"], _fallback["sampler2D"]))
		set0.append(u)
	return set0


## The baked volume's fill uniforms (CloudBaked.shadow_fill) while it draws; empty otherwise.
func _baked_fill_uniforms() -> Dictionary:
	if not _baked_pipeline.is_valid() or cloud_effect == null or cloud_effect.baked == null 			or not cloud_effect.baked_shown:
		return {}
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	return cloud_effect.baked.shadow_fill(hi + lo, _samplers["true|false|false"])


func _render_pass(_callback_type: int, _render_data: RenderData) -> void:
	if _rd == null or _maps.size() != 2 or not ShaderGlobals.value(&"cloud_enabled") or not _kernels_ready():
		return
	var start_us := Time.get_ticks_usec()
	var fill := _baked_fill_uniforms()
	baked_fill = not fill.is_empty()
	var shader := _baked_shader if baked_fill else _shader
	var uniform_set0 := UniformSetCacheRD.get_cache(shader,
			0, _set0(_baked_port, _baked_globals) if baked_fill else _set0(_port, _globals_buffer))
	var e1: Vector3 = ShaderGlobals.value(&"cloud_shadow_e1")
	var e2: Vector3 = ShaderGlobals.value(&"cloud_shadow_e2")
	var sun: Vector3 = ShaderGlobals.value(&"cloud_shadow_sun")
	var extent: Vector4 = ShaderGlobals.value(&"cloud_shadow_extent")
	var anchors := [ShaderGlobals.value(&"cloud_shadow_near_anchor"), ShaderGlobals.value(&"cloud_shadow_far_anchor")]
	var halves := [extent.x, extent.y]
	var groups := (_size - 1) / GROUP + 1
	for i in range(2):  # the blocks first: no buffer updates inside a compute list
		var anchor: Vector3 = anchors[i]
		var sun_w := 1.0 if baked_fill and fill["fade"] else 0.0
		var bytes := PackedFloat32Array([anchor.x, anchor.y, anchor.z, halves[i], e1.x, e1.y, e1.z,
				1.0 if i == 0 and not grid_mean_only else 0.0, e2.x, e2.y, e2.z, float(_size), sun.x, sun.y, sun.z, sun_w])
		_rd.buffer_update(_blocks[i], 0, BLOCK_SIZE, bytes.to_byte_array())
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _baked_pipeline if baked_fill else _pipeline)
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
		if baked_fill:
			set1.append_array(fill["uniforms"])
		_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(shader, 1, set1), 1)
		_rd.compute_list_dispatch(list, groups, groups, 1)
	_rd.compute_list_end()
	ran_frame = Engine.get_frames_drawn()
	stats = {"record_ms": (Time.get_ticks_usec() - start_us) / 1000.0}
