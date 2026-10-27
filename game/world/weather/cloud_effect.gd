class_name CloudEffect
extends CompositorEffect
## The cloud pass (M8 weather and clouds, Task 8): the raymarch at reduced resolution, per-eye temporal accumulation
## and the depth-aware upsampling to full resolution, as compute kernels (shaders/cloud_pass.gdshaderinc with
## atmosphere.gdshaderinc and clouds.gdshaderinc, ported by CloudShaderPort) after the opaque pass and the sky, before
## the transparent pass. The result (premultiplied colour + the background's transmittance T = 1 - alpha in .a, one
## layer per view) is `output`; the composite quad
## (CloudLayer, shaders/cloud_layer.gdshader) blends it over the scene first in the transparent pass, so plumes and
## the other transparent objects draw over the clouds (and MSAA's resolve keeps the clouds). Reads the resolved depth.
## The march and the accumulation run once per frame in the centre view (VR performance part 2, user 2026-10-26: one
## march per eye with its own jitter and history looked grainy and different per eye): in VR the head with a frustum
## covering both eyes (CloudCentreView), its depth read from the eyes' depth buffers; on the desktop the view itself.
## Each eye's upsample then reads the one centre image at its own rays (the same noise and blur in both eyes, the
## cloud's disparity from its distance). Without a RenderingDevice (headless, Compatibility) it does nothing.

const MARCH_FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_light_cache.gdshaderinc", "res://shaders/cloud_far.gdshaderinc",
		"res://shaders/cloud_light.gdshaderinc", "res://shaders/cloud_pass.gdshaderinc"]
const PASS_FILES := ["res://shaders/cloud_pass.gdshaderinc"]
const GROUP := 8
const MAX_DISPATCH_GROUPS := 65535  # D3D12's limit per dispatch dimension
const CENTRE_IMAGES := ["color", "refined", "depth", "history0", "history1", "age0", "age1", "buffer", "outline"]
## Golden ratio conjugate: the frame's jitter offset is frame x this, mod 1 (a low-discrepancy sequence in time).
const GOLDEN := 0.6180339887498949
## The eyes' upsample tent's largest radius [centre pixels] (cloud_pass.gdshaderinc CLOUD_UPSAMPLE): 1 (bilinear) with
## the deterministic march (static clouds Task 3: nothing to hide; its final gather is then a Catmull-Rom filter, in
## the eyes and on the desktop - CloudCentre params.w), 2 with the jittered one (its soft edges; the tent's radius then
## follows CLOUD_EDGE_SOFT_M between 1 and this).
const TENT_RADIUS_DETERMINISTIC := 1.0
const TENT_RADIUS_JITTERED := 2.0
const VIEW_BLOCK_SIZE := 4 * 64 + 5 * 16
const EYES_BLOCK_SIZE := 4 * 64 + 16  ## CloudEyes: two clip_from_head, two inv_projection, eye sizes
const CENTRE_BLOCK_SIZE := 64 + 2 * 16  ## CloudCentre: clip_from_earth, sizes, params
## Render callbacks a replaced image waits before it is freed (frames in flight may still sample it).
const FREE_DELAY_CALLBACKS := 3
## The shape prefilter's kernel choice (_shape_prefilter_needed): its start footprint (core cloud_noise.hpp
## cloud_shape_prefilter_start_m, clouds.gdshaderinc), the WGS84 ellipsoid's largest radius of curvature a^2 / b (at
## the poles; conservative for the farthest cloud sample) and the cloud top above the ellipsoid.
const SHAPE_PREFILTER_START_M := 3500.0
const CURVATURE_RADIUS_MAX_M := 6378137.0 * 6378137.0 / 6356752.314
const CLOUD_TOP_M := 20000.0

## The full-resolution result for the composite quad (layer = view); re-created when the size changes and shown by
## swap_output (then output_generation counts up and the quad's material takes the texture again).
var output := Texture2DArrayRD.new()
var output_generation := 0
## Render-thread state of the last frame (tests): {valid, low_size, views, march_ms}.
var stats := {}
## Diagnostics (terrain_perf --gpu_passes): GPU timestamps around the cloud passes (RenderingDevice).
static var capture_timestamps := false
## Checks only (cloud_check.gd): >= 0 pins the frame index of the march's jitter (the same noise in two renders that
## differ in one setting); -1 in the game.
static var pinned_frame := -1
## The light cache (CloudLayer's; filled on the render thread before the march, cloud_light_cache.gd); null: none.
var light_cache: CloudLightCache

var _rd: RenderingDevice
var _march: CloudShaderPort
var _shaders := {}  ## kernel name -> shader RID
var _pipelines := {}
var _samplers := {}  ## "linear|repeat" -> sampler RID
var _fallback := {}  ## sampler type -> RD texture
var _globals_buffer := RID()
var _centre := {}  ## the centre view: low-resolution images, view buffer, previous transform, history slot
var _views: Array[Dictionary] = []  ## per eye (view): its upsample block and its layer of the result
var _low := Vector2i.ZERO  ## the centre image size
var _eyes_buffer := RID()  ## CloudEyes (the centre march's depth lookup in the eyes)
var _centre_buffer := RID()  ## CloudCentre (the eyes' upsample into the centre)
var _size := Vector2i.ZERO
var _view_count := 0
var _output_rd := RID()  ## the result the pass writes
var _output_mutex := Mutex.new()  ## guards the three below (render thread / main thread)
var _pending_output := RID()  ## a re-created result not yet shown
var _shown_output := RID()  ## the result `output` holds
var _retired_outputs: Array[RID] = []  ## replaced by swap_output, to be freed by the render thread
var _frame := 0
var _prefilter_warm := false  ## the prefilter kernel has been dispatched once (pre-warm)
var _callbacks := 0  ## render callbacks so far (the deferred free)
var _garbage: Array[Dictionary] = []  ## {callback, rids}: images to free once that callback is reached
var _previous_camera := Vector3.ZERO  ## float64 Earth-local camera of the previous frame
## Render-thread copies of the quality (set_quality).
var _divisor := 2
var _temporal_weight := 0.1
var _clip_gamma := 0.0


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT
	access_resolved_depth = true
	RenderingServer.call_on_render_thread(_create)


## Main thread: the quality preset's resolution divisor, temporal weight floor and the history's clip (0: the
## neighbourhood's min / max; > 0: its mean +- that many standard deviations), applied on the render thread.
func set_quality(divisor: int, temporal_weight: float, clip_gamma := 0.0) -> void:
	RenderingServer.call_on_render_thread(_set_quality.bind(divisor, temporal_weight, clip_gamma))


## Main thread: forget the history (a jump of the camera, a pinned snapshot shown at once).
func reset_history() -> void:
	RenderingServer.call_on_render_thread(_reset_history)


func _set_quality(divisor: int, temporal_weight: float, clip_gamma: float) -> void:
	if maxi(divisor, 1) != _divisor:
		_size = Vector2i.ZERO  # another resolution: re-create the images
	_divisor = maxi(divisor, 1)
	_temporal_weight = clampf(temporal_weight, 0.0, 1.0)
	_clip_gamma = maxf(clip_gamma, 0.0)


func _reset_history() -> void:
	if not _centre.is_empty():
		_centre["valid"] = false


func _create() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	_march = CloudShaderPort.build(PackedStringArray(["CLOUD_MARCH"]), PackedStringArray(MARCH_FILES))
	var kernels := {"march": _march.source,
		"temporal": CloudShaderPort.build(PackedStringArray(["CLOUD_TEMPORAL"]), PackedStringArray(PASS_FILES),
				false).source,
		"upsample": CloudShaderPort.build(PackedStringArray(["CLOUD_UPSAMPLE"]), PackedStringArray(PASS_FILES),
				false).source,
		"outline": CloudShaderPort.build(PackedStringArray(["CLOUD_OUTLINE"]), PackedStringArray(PASS_FILES),
				false).source}
	# The march's variants (one set 0): the centre view (VR), the shape prefilter, the outline pass (CLOUD_EDGE_REFINE).
	var flags := [["CLOUD_CENTRE", "_centre"], ["CLOUD_SHAPE_PREFILTER", "_prefilter"],
			["CLOUD_EDGE_REFINE", "_refine"]]
	for variant in range(1, 1 << flags.size()):
		var defines := PackedStringArray(["CLOUD_MARCH"])
		var name := "march"
		for i in range(flags.size()):
			if variant & (1 << i):
				defines.append(flags[i][0])
				name += flags[i][1]
		kernels[name] = CloudShaderPort.build(defines, PackedStringArray(MARCH_FILES)).source
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
	_eyes_buffer = _rd.uniform_buffer_create(EYES_BLOCK_SIZE)
	_centre_buffer = _rd.uniform_buffer_create(CENTRE_BLOCK_SIZE)
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
			_samplers["%s|%s|false" % [linear, repeat]] = _rd.sampler_create(state)
	_samplers["true|true|true"] = _rd.sampler_create(CloudShaderPort.mipmap_sampler_state())
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
	# Copy-from: the checks read the result back (cloud_check section 11: its alpha).
	tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT \
			| RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT | RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
	if not data.is_empty():
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
		return _rd.texture_create(tf, RDTextureView.new(), [data])
	return _rd.texture_create(tf, RDTextureView.new())


## Hands the images of the centre and the views to the deferred free (a resize or a quality change: the frame being
## drawn may still sample the old result through the composite quad).
func _free_images() -> void:
	var rids: Array[RID] = []
	for key in CENTRE_IMAGES:
		if _centre.has(key) and (_centre[key] as RID).is_valid():
			rids.append(_centre[key])
	for v in _views:
		for key in ["output_slice", "buffer"]:
			if v.has(key) and (v[key] as RID).is_valid():
				rids.append(v[key])
	_centre = {}
	_views.clear()
	_defer_free(rids)


func _defer_free(rids: Array[RID]) -> void:
	if not rids.is_empty():
		_garbage.append({"callback": _callbacks + FREE_DELAY_CALLBACKS, "rids": rids})


## Frees what was handed over FREE_DELAY_CALLBACKS render callbacks ago (no frame in flight samples it any more).
func _collect_garbage() -> void:
	while not _garbage.is_empty() and int(_garbage[0]["callback"]) <= _callbacks:
		for rid: RID in _garbage[0]["rids"]:
			if rid.is_valid():
				_rd.free_rid(rid)
		_garbage.remove_at(0)


func _ensure_images(size: Vector2i, view_count: int, low: Vector2i) -> void:
	if size == _size and view_count == _view_count and low == _low and _output_rd.is_valid():
		return
	_free_images()
	var old_output := _output_rd
	_size = size
	_view_count = view_count
	_low = low
	# At least two layers: a Texture2DArrayRD needs an array (the mono view uses layer 0).
	_output_rd = _texture(RenderingDevice.TEXTURE_TYPE_2D_ARRAY, size, maxi(view_count, 2),
			RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	var c := {"low": low, "valid": false, "slot": 0, "previous": Projection.IDENTITY}
	c["color"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	# The outline pass's result (CLOUD_EDGE_REFINE: the march's colour with the outline pixels marched again).
	c["refined"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	# Its outline list (CLOUD_OUTLINE appends, CLOUD_EDGE_REFINE runs over it): the count + one uint per march pixel.
	c["outline"] = _rd.storage_buffer_create(4 * (1 + low.x * low.y))
	c["depth"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R32G32_SFLOAT)
	c["history0"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	c["history1"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	for a in ["age0", "age1"]:  # frames each history holds (the temporal kernel's running mean)
		c[a] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R16_SFLOAT)
	c["buffer"] = _rd.uniform_buffer_create(VIEW_BLOCK_SIZE)
	_centre = c
	for i in range(view_count):
		_views.append({"output_slice": _rd.texture_create_shared_from_slice(RDTextureView.new(), _output_rd, i, 0),
				"buffer": _rd.uniform_buffer_create(VIEW_BLOCK_SIZE)})
	# The new result is shown from the main thread (swap_output, before a frame is drawn): replacing the resource's
	# texture inside this callback would invalidate the composite's material in the middle of the frame (a renderer
	# error and a frame without clouds). Until then the quad shows the old one (one frame stale); a result never shown
	# (two re-creations before a swap) is freed.
	# What to free is decided under the same lock as the hand-over: swap_output may otherwise show old_output between
	# the two (a double free_rid, once retired and once here).
	_output_mutex.lock()
	var unshown := _pending_output
	_pending_output = _output_rd
	var free_old := not (unshown.is_valid() and unshown != old_output) and old_output.is_valid() \
			and old_output != _shown_output
	_output_mutex.unlock()
	if unshown.is_valid() and unshown != old_output:
		var rids: Array[RID] = [unshown]
		_defer_free(rids)
	elif free_old:
		var rids: Array[RID] = [old_output]
		_defer_free(rids)


## Main thread, every frame (CloudLayer.update): shows a re-created result (the resource's texture is replaced before
## the next frame is drawn); the replaced one is freed by the render thread a few frames later. True when swapped.
func swap_output() -> bool:
	_output_mutex.lock()
	var pending := _pending_output
	_pending_output = RID()
	if pending.is_valid():
		if _shown_output.is_valid():
			_retired_outputs.append(_shown_output)
		_shown_output = pending
	_output_mutex.unlock()
	if not pending.is_valid():
		return false
	output.texture_rd_rid = pending
	output_generation += 1
	return true


## Render thread: the results the main thread replaced go to the deferred free.
func _retire_outputs() -> void:
	_output_mutex.lock()
	var rids: Array[RID] = _retired_outputs.duplicate()
	_retired_outputs.clear()
	_output_mutex.unlock()
	_defer_free(rids)


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd != null:
		# Inline (no method calls: the instance is being torn down). Slices go with their parent texture.
		for entry: Dictionary in _garbage:
			for rid: RID in entry["rids"]:
				if rid.is_valid():
					_rd.free_rid(rid)
		for key in CENTRE_IMAGES:
			if _centre.has(key):
				_rd.free_rid(_centre[key])
		for v in _views:
			_rd.free_rid(v["buffer"])
		var outputs: Array[RID] = [_output_rd]
		for rid: RID in _retired_outputs + [_shown_output, _pending_output]:
			if rid.is_valid() and not outputs.has(rid):
				outputs.append(rid)
		for rid: RID in outputs:
			if rid.is_valid():
				_rd.free_rid(rid)
		for shader: RID in _shaders.values():
			_rd.free_rid(shader)  # frees the pipelines with them
		for sampler: RID in _samplers.values():
			_rd.free_rid(sampler)
		for texture: RID in _fallback.values():
			_rd.free_rid(texture)
		if light_cache != null:
			light_cache.free_resources(_rd)
		for buffer: RID in [_globals_buffer, _eyes_buffer, _centre_buffer]:
			if buffer.is_valid():
				_rd.free_rid(buffer)


func _render_callback(callback_type: int, render_data: RenderData) -> void:
	# diagnostics (terrain_perf --gpu_passes): GPU timestamps around this pass, named by the interval they start
	if CloudEffect.capture_timestamps and _rd != null:
		_rd.capture_timestamp("cloud_pass")
	_render_pass(callback_type, render_data)
	if CloudEffect.capture_timestamps and _rd != null:
		_rd.capture_timestamp("transparent_and_post")


func _render_pass(_callback_type: int, render_data: RenderData) -> void:
	if _rd != null:
		_callbacks += 1
		_retire_outputs()
		_collect_garbage()
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
	var stereo := view_count > 1
	var projections: Array[Projection] = []
	for view in range(view_count):
		projections.append(scene.get_view_projection(view))
	var centre_view := CloudCentreView.build(projections, size, _divisor)
	_ensure_images(size, view_count, centre_view["low"])
	_frame += 1

	# The camera (float64 from the floating-origin globals).
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var camera := hi + lo
	# The light cache: the waiting batch's fill jobs first (their own compute list, before the march reads them), then
	# the state they lead to for the lookup (the march reads only voxels whose fill ran, with their frame).
	var cache_globals := {}
	if light_cache != null:
		if CloudEffect.capture_timestamps:
			_rd.capture_timestamp("cloud_light_fill")
		light_cache.update(_rd, _samplers, _fallback)
		if CloudEffect.capture_timestamps:
			_rd.capture_timestamp("cloud_pass")  # the march etc. (the fill's interval has its own name)
		cache_globals = light_cache.lookup_globals(camera)
	else:
		cache_globals = {&"cloud_light_cache_valid": false}

	# Set 0 of the march: the globals' block and samplers.
	_rd.buffer_update(_globals_buffer, 0, _march.block_size, _march.pack(cache_globals))
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
		u.add_id(_samplers["%s|%s|%s" % [s["linear"], s["repeat"], s["mipmap"]]])
		var texture: RID = textures[i]
		var name := String(s["id"])
		if light_cache != null and name.begins_with("cloud_light_cache_"):  # _a<k> / _b<k>: slot 0 / 1 of cascade k
			texture = light_cache.texture(0 if name[18] == "a" else 1, name.right(1).to_int())
		u.add_id(texture if texture.is_valid() else _fallback.get(s["type"], _fallback["sampler2D"]))
		set0.append(u)
	# The shape prefilter's kernel (clouds.gdshaderinc) only when a footprint can exceed its start: the farthest cloud
	# sample (_shape_prefilter_needed).
	var family := "march_centre" if stereo else "march"
	var pixel_angle: float = centre_view["pixel_angle"]
	var march := family + "_prefilter" if _shape_prefilter_needed(pixel_angle) else family
	var march_set0 := UniformSetCacheRD.get_cache(_shaders[march], 0, set0)

	# The camera's motion since the last frame.
	var camera_delta := camera - _previous_camera
	_previous_camera = camera
	var world_to_earth: Basis = ShaderGlobals.value(&"atmosphere_world_to_earth")
	# Earth-local offsets from the camera -> render world (the camera's world position: the floating origin's offset).
	var earth_to_world := Projection(Transform3D(world_to_earth.transposed(), scene.get_cam_transform().origin))
	var debug := int(ShaderGlobals.value(&"cloud_debug_output"))
	# The deterministic march (CloudLayer.deterministic_march) has nothing to average: the temporal kernel copies the
	# current frame, and the upsample is a Catmull-Rom filter (the soft-edge tent hid the jitter's noise; bilinear left
	# stair steps of a march pixel at the silhouettes). The debug outputs (the checks' physical quantities) stay
	# bilinear: in range and without the filter's overshoot, which amplified cloud_check section 8's per-pixel
	# differences at dark silhouette pixels (up to 38 % against 12 %).
	var deterministic: bool = ShaderGlobals.value(&"cloud_deterministic_march")
	var weight := 1.0 if debug != 0 or deterministic else _temporal_weight
	var tent_max := TENT_RADIUS_DETERMINISTIC if deterministic else TENT_RADIUS_JITTERED
	# The outline pass (static clouds Task 5b) after the deterministic march: CLOUD_OUTLINE lists its outline pixels,
	# CLOUD_EDGE_REFINE gives them three coverage-only sub-rays each (cloud_pass.gdshaderinc) - no stairs of a march
	# pixel. Not for the debug outputs (the checks' per-ray quantities) nor the jittered march (its temporal mean
	# averages the pixel already).
	var refine := deterministic and debug == 0
	var jitter_frame := _frame if CloudEffect.pinned_frame < 0 else CloudEffect.pinned_frame
	var params := Vector4(fmod(float(jitter_frame) * GOLDEN, 1.0), weight, debug, pixel_angle)
	var linear_clamp: RID = _samplers["true|false|false"]
	var nearest_clamp: RID = _samplers["false|false|false"]
	# The head's transform: the renderer's per-eye projection already contains the eye offset (applied once here).
	var view_to_world := scene.get_cam_transform()
	var low: Vector2i = _centre["low"]

	# The centre view's block: the desktop's view itself; in VR the head with the centre projection (no eye offset).
	var centre_projection: Projection = centre_view["projection"]
	var centre_eye := Vector3.ZERO if stereo \
			else world_to_earth * (view_to_world.basis * scene.get_view_eye_offset(0))
	var centre_full := Vector2(low * _divisor) if stereo else Vector2(size)
	var centre_clip := centre_projection * Projection(view_to_world.affine_inverse()) * earth_to_world
	_rd.buffer_update(_centre["buffer"], 0, VIEW_BLOCK_SIZE, _view_block(centre_projection, view_to_world,
			world_to_earth, _centre["previous"], centre_full, low, camera_delta, _centre["valid"], params,
			centre_eye).to_byte_array())
	_centre["previous"] = centre_clip
	var centre_bytes := _matrix_floats(centre_clip)
	centre_bytes.append_array([centre_full.x, centre_full.y, low.x, low.y, 1.0 if stereo else 0.0, pixel_angle,
			tent_max, 1.0 if deterministic and debug == 0 else 0.0])
	_rd.buffer_update(_centre_buffer, 0, CENTRE_BLOCK_SIZE, centre_bytes.to_byte_array())
	var march_set1: Array = [_ubo(_centre["buffer"]), _tex(1, nearest_clamp, buffers.get_depth_layer(0, false)),
			_image(2, _centre["color"]), _image(3, _centre["depth"])]
	if stereo:
		var eyes := PackedFloat32Array()
		for view in range(2):
			eyes.append_array(_matrix_floats(projections[view]))
		for view in range(2):
			eyes.append_array(_matrix_floats(projections[view].inverse()))
		eyes.append_array([size.x, size.y, 0.0, 0.0])
		_rd.buffer_update(_eyes_buffer, 0, EYES_BLOCK_SIZE, eyes.to_byte_array())
		march_set1.append_array([_tex(4, nearest_clamp, buffers.get_depth_layer(1, false)), _ubo(_eyes_buffer, 5)])

	var slot: int = _centre["slot"]
	var history_in: RID = _centre["history%d" % slot]
	var history_out: RID = _centre["history%d" % (1 - slot)]
	var age_in: RID = _centre["age%d" % slot]
	var age_out: RID = _centre["age%d" % (1 - slot)]
	_centre["slot"] = 1 - slot
	var groups_low := Vector2i((low.x - 1) / GROUP + 1, (low.y - 1) / GROUP + 1)
	var groups_full := Vector2i((size.x - 1) / GROUP + 1, (size.y - 1) / GROUP + 1)

	for view in range(view_count):  # the blocks first: no buffer updates inside a compute list
		# Each eye's block (its projection and offset; the upsample reads the centre's image through CloudCentre).
		var eye_earth := world_to_earth * (view_to_world.basis * scene.get_view_eye_offset(view))
		_rd.buffer_update(_views[view]["buffer"], 0, VIEW_BLOCK_SIZE, _view_block(projections[view], view_to_world,
				world_to_earth, Projection.IDENTITY, Vector2(size), low, camera_delta, _centre["valid"], params,
				eye_earth).to_byte_array())
	# The outline pass's set 1: the march's, its colour read (binding 6), the refined image written instead and the
	# outline list (binding 7, its count reset here: no buffer updates inside a compute list).
	var refine_set1 := []
	var warm_now := not _prefilter_warm and march == family  # the pre-warm below binds the refine set too
	if refine or warm_now:
		refine_set1 = march_set1.duplicate()
		refine_set1[2] = _image(2, _centre["refined"])
		refine_set1.append_array([_tex(6, nearest_clamp, _centre["color"]), _storage(7, _centre["outline"])])
		_rd.buffer_update(_centre["outline"], 0, 4, PackedInt32Array([0]).to_byte_array())
	var list := _rd.compute_list_begin()
	if warm_now:
		# Once: one group of the prefilter kernels so their first real use (the ascent's high limb, the scaled-space
		# hand-over) does not pay the pipeline's first dispatch (up to 5 ms); the march below overwrites their texels.
		_prefilter_warm = true
		for warm: String in [family + "_prefilter", family + "_prefilter_refine"]:
			_rd.compute_list_bind_compute_pipeline(list, _pipelines[warm])
			_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders[warm], 0, set0), 0)
			_rd.compute_list_bind_uniform_set(list, _set1(warm, refine_set1 if warm.ends_with("_refine")
					else march_set1), 1)
			_rd.compute_list_dispatch(list, 1, 1, 1)
			_rd.compute_list_add_barrier(list)
	_rd.compute_list_bind_compute_pipeline(list, _pipelines[march])
	_rd.compute_list_bind_uniform_set(list, march_set0, 0)
	_rd.compute_list_bind_uniform_set(list, _set1(march, march_set1), 1)
	_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
	_rd.compute_list_add_barrier(list)
	if refine:
		_rd.compute_list_bind_compute_pipeline(list, _pipelines["outline"])
		_rd.compute_list_bind_uniform_set(list, _set1("outline", [_ubo(_centre["buffer"]),
				_image(2, _centre["refined"]), _tex(6, nearest_clamp, _centre["color"]),
				_storage(7, _centre["outline"])]), 1)
		_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
		_rd.compute_list_add_barrier(list)
		var outline := march + "_refine"
		_rd.compute_list_bind_compute_pipeline(list, _pipelines[outline])
		_rd.compute_list_bind_uniform_set(list, UniformSetCacheRD.get_cache(_shaders[outline], 0, set0), 0)
		_rd.compute_list_bind_uniform_set(list, _set1(outline, refine_set1), 1)
		# One group per GROUP x GROUP list entries, for a full list: the groups past the count end at once. (The indirect
		# dispatch over a GPU-written group count read the count's reset value on D3D12 - no outline pass at all.)
		# (2-D: a dimension is capped at 65535 groups on D3D12.)
		var list_groups := (low.x * low.y - 1) / (GROUP * GROUP) + 1
		var list_width := mini(list_groups, MAX_DISPATCH_GROUPS)
		_rd.compute_list_dispatch(list, list_width, (list_groups - 1) / list_width + 1, 1)
		_rd.compute_list_add_barrier(list)
	_rd.compute_list_bind_compute_pipeline(list, _pipelines["temporal"])
	_rd.compute_list_bind_uniform_set(list, _set1("temporal", [_ubo(_centre["buffer"]),
			_tex(1, nearest_clamp, _centre["refined" if refine else "color"]),
			_tex(2, nearest_clamp, _centre["depth"]),
			_tex(3, linear_clamp, history_in), _image(4, history_out), _tex(5, nearest_clamp, age_in),
			_image(6, age_out)]), 1)
	_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
	_rd.compute_list_add_barrier(list)
	for view in range(view_count):
		var v: Dictionary = _views[view]
		_rd.compute_list_bind_compute_pipeline(list, _pipelines["upsample"])
		_rd.compute_list_bind_uniform_set(list, _set1("upsample", [_ubo(v["buffer"]),
				_tex(1, nearest_clamp, buffers.get_depth_layer(view, false)), _tex(2, nearest_clamp, history_out),
				_tex(3, nearest_clamp, _centre["depth"]), _image(4, v["output_slice"]), _ubo(_centre_buffer, 5)]), 1)
		_rd.compute_list_dispatch(list, groups_full.x, groups_full.y, 1)
	_rd.compute_list_end()
	_centre["valid"] = true
	stats = {"valid": true, "low_size": low, "views": view_count,
		"record_ms": (Time.get_ticks_usec() - start_us) / 1000.0}


## The CloudView block (cloud_pass.gdshaderinc): inverse projection, view -> world, world -> Earth axes, the previous
## frame's clip transform, sizes (full, low), the camera's motion and history flag, params, the eye's Earth-local offset
## and the frame index, the history's clip.
func _view_block(projection: Projection, view_to_world: Transform3D, world_to_earth: Basis, previous: Projection,
		full: Vector2, low: Vector2i, camera_delta: Vector3, valid: bool, params: Vector4,
		eye_earth: Vector3) -> PackedFloat32Array:
	var bytes := PackedFloat32Array()
	for m in [projection.inverse(), Projection(view_to_world), Projection(Transform3D(world_to_earth, Vector3.ZERO)),
			previous]:
		bytes.append_array(_matrix_floats(m))
	bytes.append_array([full.x, full.y, low.x, low.y])
	bytes.append_array([camera_delta.x, camera_delta.y, camera_delta.z, 1.0 if valid else 0.0])
	bytes.append_array([params.x, params.y, params.z, params.w])
	bytes.append_array([eye_earth.x, eye_earth.y, eye_earth.z, float(_frame % 4096)])
	bytes.append_array([_clip_gamma, 0.0, 0.0, 0.0])
	return bytes


static func _matrix_floats(m: Projection) -> PackedFloat32Array:
	var floats := PackedFloat32Array()
	for c in range(4):
		var col: Vector4 = m[c]
		floats.append_array([col.x, col.y, col.z, col.w])
	return floats


## Whether a cloud sample of this frame can have a pixel footprint above the shape prefilter's start (clouds.gdshaderinc
## CLOUD_SHAPE_PREFILTER_START_M): the march pixel's angle (the centre view's) times the farthest distance a cloud
## sample can have - the horizon distances of the camera and of the cloud top on the largest radius of curvature, at
## most the near / far hand-over's end (static clouds Task 8: the march ends there; the debug outputs march the whole
## segment), plus the light march's near length.
func _shape_prefilter_needed(pixel_angle: float) -> bool:
	var h := maxf(float(ShaderGlobals.value(&"cloud_camera_height_m")), 0.0)
	var far_m := sqrt(h * (2.0 * CURVATURE_RADIUS_MAX_M + h)) \
			+ sqrt(CLOUD_TOP_M * (2.0 * CURVATURE_RADIUS_MAX_M + CLOUD_TOP_M))
	if int(ShaderGlobals.value(&"cloud_debug_output")) == 0:
		far_m = minf(far_m, float(ShaderGlobals.value(&"cloud_handover_far_m")))
	far_m += (ShaderGlobals.value(&"cloud_light_march") as Vector4).z
	return pixel_angle * far_m > SHAPE_PREFILTER_START_M


func _set1(kernel: String, uniforms: Array) -> RID:
	var typed: Array[RDUniform] = []
	typed.assign(uniforms)
	return UniformSetCacheRD.get_cache(_shaders[kernel], 1, typed)


static func _ubo(buffer: RID, binding := 0) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	u.binding = binding
	u.add_id(buffer)
	return u


static func _tex(binding: int, sampler: RID, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
	u.binding = binding
	u.add_id(sampler)
	u.add_id(texture)
	return u


static func _storage(binding: int, buffer: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_STORAGE_BUFFER
	u.binding = binding
	u.add_id(buffer)
	return u


static func _image(binding: int, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	u.binding = binding
	u.add_id(texture)
	return u
