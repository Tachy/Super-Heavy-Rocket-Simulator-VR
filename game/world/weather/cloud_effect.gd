class_name CloudEffect
extends CompositorEffect
## The cloud pass (M8 weather and clouds, Task 8): the raymarch at reduced resolution, per-eye temporal accumulation
## and the depth-aware upsampling to full resolution, as compute kernels (shaders/cloud_pass.gdshaderinc with
## atmosphere.gdshaderinc and clouds.gdshaderinc, ported by CloudShaderPort) after the opaque pass and the sky, before
## the transparent pass. The result (premultiplied colour + alpha, one layer per view) is `output`; the composite quad
## (CloudLayer, shaders/cloud_layer.gdshader) blends it over the scene first in the transparent pass, so plumes and
## the other transparent objects draw over the clouds (and MSAA's resolve keeps the clouds). Reads the resolved depth.
## Each view (VR eye) has its own low-resolution images, history and previous clip transform: the accumulation is
## reprojected per eye. The cloud shapes are Earth-fixed and the jitter a function of the pixel and the frame, so both
## eyes see the same clouds. Without a RenderingDevice (headless, Compatibility) it does nothing.

const MARCH_FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_pass.gdshaderinc"]
const PASS_FILES := ["res://shaders/cloud_pass.gdshaderinc"]
const GROUP := 8
## Golden ratio conjugate: the frame's jitter offset is frame x this, mod 1 (a low-discrepancy sequence in time).
const GOLDEN := 0.6180339887498949
const VIEW_BLOCK_SIZE := 4 * 64 + 3 * 16

## The full-resolution result for the composite quad (layer = view); re-created when the size changes (then
## output_generation counts up and the quad's material takes the texture again).
var output := Texture2DArrayRD.new()
var output_generation := 0
## Render-thread state of the last frame (tests): {valid, low_size, views, march_ms}.
var stats := {}

var _rd: RenderingDevice
var _march: CloudShaderPort
var _shaders := {}  ## kernel name -> shader RID
var _pipelines := {}
var _samplers := {}  ## "linear|repeat" -> sampler RID
var _fallback := {}  ## sampler type -> RD texture
var _globals_buffer := RID()
var _views: Array[Dictionary] = []  ## per view: textures, buffer, previous transform, history slot
var _size := Vector2i.ZERO
var _view_count := 0
var _output_rd := RID()
var _frame := 0
var _previous_camera := Vector3.ZERO  ## float64 Earth-local camera of the previous frame
## Render-thread copies of the quality (set_quality).
var _divisor := 2
var _temporal_weight := 0.1


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT
	access_resolved_depth = true
	RenderingServer.call_on_render_thread(_create)


## Main thread: the quality preset's resolution divisor and temporal weight (applied on the render thread).
func set_quality(divisor: int, temporal_weight: float) -> void:
	RenderingServer.call_on_render_thread(_set_quality.bind(divisor, temporal_weight))


## Main thread: forget the history (a jump of the camera, a pinned snapshot shown at once).
func reset_history() -> void:
	RenderingServer.call_on_render_thread(_reset_history)


func _set_quality(divisor: int, temporal_weight: float) -> void:
	_divisor = maxi(divisor, 1)
	_temporal_weight = clampf(temporal_weight, 0.0, 1.0)
	_size = Vector2i.ZERO  # re-create the images


func _reset_history() -> void:
	for v in _views:
		v["valid"] = false


func _create() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	_march = CloudShaderPort.build(PackedStringArray(["CLOUD_MARCH"]), PackedStringArray(MARCH_FILES))
	var kernels := {"march": _march.source,
		"temporal": CloudShaderPort.build(PackedStringArray(["CLOUD_TEMPORAL"]), PackedStringArray(PASS_FILES),
				false).source,
		"upsample": CloudShaderPort.build(PackedStringArray(["CLOUD_UPSAMPLE"]), PackedStringArray(PASS_FILES),
				false).source}
	for name: String in kernels:
		var src := RDShaderSource.new()
		src.language = RenderingDevice.SHADER_LANGUAGE_GLSL
		src.source_compute = kernels[name]
		var spirv := _rd.shader_compile_spirv_from_source(src)
		if spirv.compile_error_compute != "":
			push_error("CloudEffect %s: %s" % [name, spirv.compile_error_compute])
			_rd = null
			return
		var shader := _rd.shader_create_from_spirv(spirv)
		if not shader.is_valid():
			push_error("CloudEffect %s: shader not created" % name)
			_rd = null
			return
		_shaders[name] = shader
		_pipelines[name] = _rd.compute_pipeline_create(shader)
	_globals_buffer = _rd.uniform_buffer_create(_march.block_size)
	for linear in [false, true]:
		for repeat in [false, true]:
			var state := RDSamplerState.new()
			var filter := RenderingDevice.SAMPLER_FILTER_LINEAR if linear else RenderingDevice.SAMPLER_FILTER_NEAREST
			state.min_filter = filter
			state.mag_filter = filter
			state.mip_filter = RenderingDevice.SAMPLER_FILTER_NEAREST
			var mode := RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT if repeat \
					else RenderingDevice.SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE
			state.repeat_u = mode
			state.repeat_v = mode
			state.repeat_w = mode
			_samplers["%s|%s" % [linear, repeat]] = _rd.sampler_create(state)
	_fallback["sampler2D"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, Vector2i(1, 1), 1,
			RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM, PackedByteArray([0, 0, 0, 0]))
	_fallback["sampler3D"] = _texture(RenderingDevice.TEXTURE_TYPE_3D, Vector2i(1, 1), 1,
			RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM, PackedByteArray([0, 0, 0, 0]))


func _texture(type: int, size: Vector2i, layers: int, format: int, data := PackedByteArray()) -> RID:
	var tf := RDTextureFormat.new()
	tf.texture_type = type
	tf.width = size.x
	tf.height = size.y
	tf.depth = 1
	tf.array_layers = layers
	tf.format = format
	tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT \
			| RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
	if not data.is_empty():
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
		return _rd.texture_create(tf, RDTextureView.new(), [data])
	return _rd.texture_create(tf, RDTextureView.new())


func _free_images() -> void:
	for v in _views:
		for key in ["color", "depth", "history0", "history1", "buffer", "output_slice"]:
			if v.has(key) and (v[key] as RID).is_valid():
				_rd.free_rid(v[key])
	_views.clear()


func _ensure_images(size: Vector2i, view_count: int) -> void:
	if size == _size and view_count == _view_count and _output_rd.is_valid():
		return
	_free_images()
	var old_output := _output_rd
	_size = size
	_view_count = view_count
	var low := Vector2i(maxi((size.x + _divisor - 1) / _divisor, 1), maxi((size.y + _divisor - 1) / _divisor, 1))
	# At least two layers: a Texture2DArrayRD needs an array (the mono view uses layer 0).
	_output_rd = _texture(RenderingDevice.TEXTURE_TYPE_2D_ARRAY, size, maxi(view_count, 2),
			RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	for i in range(view_count):
		var v := {"low": low, "valid": false, "slot": 0, "previous": Projection.IDENTITY}
		v["color"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
		v["depth"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R32G32_SFLOAT)
		v["history0"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1,
				RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
		v["history1"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1,
				RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
		v["output_slice"] = _rd.texture_create_shared_from_slice(RDTextureView.new(), _output_rd, i, 0)
		v["buffer"] = _rd.uniform_buffer_create(VIEW_BLOCK_SIZE)
		_views.append(v)
	# Replace the texture under the same resource (the quad's material keeps it), then free the old one.
	output.texture_rd_rid = _output_rd
	output_generation += 1
	if old_output.is_valid():
		_rd.free_rid(old_output)


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd != null:
		# Inline (no method calls: the instance is being torn down). Slices go with their parent texture.
		for v in _views:
			for key in ["color", "depth", "history0", "history1", "buffer"]:
				_rd.free_rid(v[key])
		if _output_rd.is_valid():
			_rd.free_rid(_output_rd)
		for shader: RID in _shaders.values():
			_rd.free_rid(shader)  # frees the pipelines with them
		for sampler: RID in _samplers.values():
			_rd.free_rid(sampler)
		for texture: RID in _fallback.values():
			_rd.free_rid(texture)
		if _globals_buffer.is_valid():
			_rd.free_rid(_globals_buffer)


func _render_callback(_callback_type: int, render_data: RenderData) -> void:
	if _rd == null or not ShaderGlobals.value(&"cloud_enabled"):
		stats = {"valid": false}
		_reset_history()
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	var scene := render_data.get_render_scene_data()
	if buffers == null or scene == null:
		return
	var size := buffers.get_internal_size()
	if size.x == 0 or size.y == 0:
		return
	var start_us := Time.get_ticks_usec()
	var view_count := buffers.get_view_count()
	_ensure_images(size, view_count)
	_frame += 1

	# Set 0 of the march: the globals' block and samplers.
	_rd.buffer_update(_globals_buffer, 0, _march.block_size, _march.pack())
	var set0: Array[RDUniform] = []
	var block := RDUniform.new()
	block.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	block.binding = 0
	block.add_id(_globals_buffer)
	set0.append(block)
	var textures := _march.sampler_textures()
	for i in range(_march.samplers.size()):
		var s: Dictionary = _march.samplers[i]
		var u := RDUniform.new()
		u.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
		u.binding = s["binding"]
		u.add_id(_samplers["%s|%s" % [s["linear"], s["repeat"]]])
		var texture: RID = textures[i]
		u.add_id(texture if texture.is_valid() else _fallback.get(s["type"], _fallback["sampler2D"]))
		set0.append(u)
	var march_set0 := UniformSetCacheRD.get_cache(_shaders["march"], 0, set0)

	# The camera (float64 from the floating-origin globals) and its motion since the last frame.
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var camera := hi + lo
	var camera_delta := camera - _previous_camera
	_previous_camera = camera
	var world_to_earth: Basis = ShaderGlobals.value(&"atmosphere_world_to_earth")
	# Earth-local offsets from the camera -> render world (the camera's world position: the floating origin's offset).
	var earth_to_world := Projection(Transform3D(world_to_earth.transposed(), scene.get_cam_transform().origin))
	var debug := int(ShaderGlobals.value(&"cloud_debug_output"))
	var weight := 1.0 if debug != 0 else _temporal_weight
	var jitter := fmod(float(_frame) * GOLDEN, 1.0)
	var linear_clamp: RID = _samplers["true|false"]
	var nearest_clamp: RID = _samplers["false|false"]

	for view in range(view_count):
		var v: Dictionary = _views[view]
		var low: Vector2i = v["low"]
		var projection := scene.get_view_projection(view)
		var view_to_world := scene.get_cam_transform() * Transform3D(Basis.IDENTITY, scene.get_view_eye_offset(view))
		var clip_from_earth := projection * Projection(view_to_world.affine_inverse()) * earth_to_world
		var block_bytes := PackedFloat32Array()
		for m in [projection.inverse(), Projection(view_to_world), Projection(Transform3D(world_to_earth, Vector3.ZERO)),
				v["previous"]]:
			for c in range(4):
				var col: Vector4 = m[c]
				block_bytes.append_array([col.x, col.y, col.z, col.w])
		block_bytes.append_array([size.x, size.y, low.x, low.y])
		block_bytes.append_array([camera_delta.x, camera_delta.y, camera_delta.z, 1.0 if v["valid"] else 0.0])
		block_bytes.append_array([jitter, weight, debug, 0.0])
		_rd.buffer_update(v["buffer"], 0, VIEW_BLOCK_SIZE, block_bytes.to_byte_array())
		v["previous"] = clip_from_earth

		var depth := buffers.get_depth_layer(view, false)
		var slot: int = v["slot"]
		var history_in: RID = v["history%d" % slot]
		var history_out: RID = v["history%d" % (1 - slot)]
		v["slot"] = 1 - slot
		var groups_low := Vector2i((low.x - 1) / GROUP + 1, (low.y - 1) / GROUP + 1)
		var groups_full := Vector2i((size.x - 1) / GROUP + 1, (size.y - 1) / GROUP + 1)

		var list := _rd.compute_list_begin()
		_rd.compute_list_bind_compute_pipeline(list, _pipelines["march"])
		_rd.compute_list_bind_uniform_set(list, march_set0, 0)
		_rd.compute_list_bind_uniform_set(list, _set1("march", [_ubo(v["buffer"]), _tex(1, nearest_clamp, depth),
				_image(2, v["color"]), _image(3, v["depth"])]), 1)
		_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
		_rd.compute_list_add_barrier(list)
		_rd.compute_list_bind_compute_pipeline(list, _pipelines["temporal"])
		_rd.compute_list_bind_uniform_set(list, _set1("temporal", [_ubo(v["buffer"]),
				_tex(1, nearest_clamp, v["color"]), _tex(2, nearest_clamp, v["depth"]),
				_tex(3, linear_clamp, history_in), _image(4, history_out)]), 1)
		_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
		_rd.compute_list_add_barrier(list)
		_rd.compute_list_bind_compute_pipeline(list, _pipelines["upsample"])
		_rd.compute_list_bind_uniform_set(list, _set1("upsample", [_ubo(v["buffer"]), _tex(1, nearest_clamp, depth),
				_tex(2, nearest_clamp, history_out), _tex(3, nearest_clamp, v["depth"]),
				_image(4, v["output_slice"])]), 1)
		_rd.compute_list_dispatch(list, groups_full.x, groups_full.y, 1)
		_rd.compute_list_end()
		v["valid"] = true
	stats = {"valid": true, "low_size": _views[0]["low"], "views": view_count,
		"record_ms": (Time.get_ticks_usec() - start_us) / 1000.0}


func _set1(kernel: String, uniforms: Array) -> RID:
	var typed: Array[RDUniform] = []
	typed.assign(uniforms)
	return UniformSetCacheRD.get_cache(_shaders[kernel], 1, typed)


static func _ubo(buffer: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	u.binding = 0
	u.add_id(buffer)
	return u


static func _tex(binding: int, sampler: RID, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
	u.binding = binding
	u.add_id(sampler)
	u.add_id(texture)
	return u


static func _image(binding: int, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	u.binding = binding
	u.add_id(texture)
	return u
