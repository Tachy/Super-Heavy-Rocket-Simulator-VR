class_name CloudEffect
extends CompositorEffect
## The cloud pass (M8 weather and clouds, Task 8; the baked volume since 2026-10-30): the march through the baked volume
## (CloudBaked, shaders/cloud_baked.gdshaderinc with cloud_pass.gdshaderinc, atmosphere.gdshaderinc and
## clouds.gdshaderinc, ported by CloudShaderPort) at reduced resolution and the depth-aware upsampling to full
## resolution, as compute kernels after the opaque pass and the sky, before the transparent pass. The result
## (premultiplied colour + the background's transmittance T = 1 - alpha in .a, one layer per view) is `output`; the
## composite quad (CloudLayer, shaders/cloud_layer.gdshader) blends it over the scene first in the transparent pass, so
## plumes and the other transparent objects draw over the clouds (and MSAA's resolve keeps the clouds). Reads the
## resolved depth.
## The march runs once per frame in the centre view (VR performance part 2, user 2026-10-26): in VR the head with a
## frustum covering both eyes (CloudCentreView, equiangular), its depth read from the eyes' depth buffers; on the desktop
## the view itself. Each eye's upsample then reads the one centre image at its own rays (the same image in both eyes,
## the cloud's disparity from its distance). Without a RenderingDevice (headless, Compatibility) it does nothing.
## Fast start (2026-10-30): the kernels are built off the main and the render thread (CloudKernels: glslang in
## parallel on workers, the driver binaries cached on disk). Until the kernels and the baked volume's first set are
## ready the frame shows no clouds.

const MARCH_FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_far.gdshaderinc", "res://shaders/cloud_light.gdshaderinc",
		"res://shaders/cloud_pass.gdshaderinc"]
const PASS_FILES := ["res://shaders/cloud_pass.gdshaderinc"]
## The loading screen's step texts while the kernels are built (kernels_status; German UI).
const STATUS_COMPILING := "Wolken-Shader übersetzen (einmalig nach einer Änderung, einige Sekunden)"
const STATUS_LOADING := "Wolken-Shader laden"
## The pass's own kernel (the upsample; CloudKernels specs from kernel_specs) - the march's are the baked volume's.
const PASS_KERNELS := ["upsample"]
const GROUP := 8
const CENTRE_IMAGES := ["color", "depth", "buffer"]
const VIEW_BLOCK_SIZE := 3 * 64 + 3 * 16  ## CloudView: three matrices, sizes, eye offset, params
const EYES_BLOCK_SIZE := 4 * 64 + 16  ## CloudEyes: two clip_from_head, two inv_projection, eye sizes
const CENTRE_BLOCK_SIZE := 64 + 3 * 16  ## CloudCentre: clip_from_earth, sizes, params, warp
## Render callbacks a replaced image waits before it is freed (frames in flight may still sample it).
const FREE_DELAY_CALLBACKS := 3
## The WGS84 ellipsoid's largest radius of curvature a^2 / b (at the poles) and the cloud top above the ellipsoid
## (clouds.gdshaderinc CLOUD_TOP_M; the game's one copy, CloudBaked and CloudLayer read it).
const CURVATURE_RADIUS_MAX_M := CloudLayer.WGS84_A_M * CloudLayer.POLAR_STRETCH
const CLOUD_TOP_M := 20000.0

## The full-resolution result for the composite quad (layer = view); re-created when the size changes and shown by
## swap_output (then output_generation counts up and the quad's material takes the texture again).
var output := Texture2DArrayRD.new()
var output_generation := 0
## Render-thread state of the last frame (tests): {valid, low_size, views, baked, record_ms}.
var stats := {}
## Diagnostics (PerfLog): image sets created so far (each one a new result shown a frame late), the centre image size.
var image_creations := 0
var centre_low := Vector2i.ZERO
## Diagnostics (terrain_perf --gpu_passes): GPU timestamps around the cloud passes (RenderingDevice).
static var capture_timestamps := false
## The baked volume (created once its kernels are built) and whether it drew the last frame.
var baked: CloudBaked
var baked_shown := false

var _rd: RenderingDevice
var _march: CloudShaderPort  ## the march's globals (set 0: the uniform block and the sampler globals)
var _shaders := {}  ## kernel name -> shader RID
var _pipelines := {}
var _samplers := {}  ## "linear|repeat|mipmap" -> sampler RID
var _fallback := {}  ## sampler type -> RD texture
var _globals_buffer := RID()
var _centre := {}  ## the centre view: its low-resolution images and view buffer
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
var _callbacks := 0  ## render callbacks so far (the deferred free)
var _garbage: Array[Dictionary] = []  ## {callback, rids}: images to free once that callback is reached
## Render-thread copy of the resolution divisor (set_quality; may be fractional: 1.5).
var _divisor := 2.0
## The kernel builds (CloudKernels; render thread, read by kernels_status on the main thread under _kernel_mutex): the
## upsample, the baked volume's per stereo flag; whether the upsample's shader is created (or failed).
var _kernel_mutex := Mutex.new()
var _pass_kernels: CloudKernels
var _baked_kernels := {}
var _pass_state := 0  ## 0 waiting, 1 created, -1 failed
var _baked_failed := false
var _baked_stereo := false  ## the stereo flag `baked` was created for
var _stereo_guess := false  ## the start's guess (an XR interface was up), corrected by the first frame


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT
	access_resolved_depth = true
	# The baked volume's kernels for the stereo march when an XR interface is up (the first frame corrects a wrong guess).
	var xr := XRServer.primary_interface
	_stereo_guess = xr != null and xr.is_initialized()
	RenderingServer.call_on_render_thread(_create)


## Main thread: the quality preset's resolution divisor, applied on the render thread.
func set_quality(divisor: float) -> void:
	RenderingServer.call_on_render_thread(_set_quality.bind(divisor))


## Main thread (the loading screen's hold): a step text while the kernels are still being built - the compile's when
## SPIR-V had to be compiled (no cached copy: the first start, a changed shader), else the cache read's; "" once they
## are built (or without a RenderingDevice).
func kernels_status() -> String:
	_kernel_mutex.lock()
	var sets: Array[CloudKernels] = []
	if _pass_kernels != null:
		sets.append(_pass_kernels)
	for k: CloudKernels in _baked_kernels.values():
		sets.append(k)
	_kernel_mutex.unlock()
	var waiting := false
	var compiling := false
	for k in sets:
		if not k.done():
			waiting = true
			compiling = compiling or k.misses() > 0
	if not waiting:
		return ""
	return STATUS_COMPILING if compiling else STATUS_LOADING


## Checks: the raw bytes of one of the centre view's images ("color": the march's colour + T, "depth": its scene and
## cloud distances) - empty before the first frame.
func debug_centre_data(key: String) -> PackedByteArray:
	if _rd == null or not _centre.has(key):
		return PackedByteArray()
	return _rd.texture_get_data(_centre[key], 0)


func _set_quality(divisor: float) -> void:
	if maxf(divisor, 1.0) != _divisor:
		_size = Vector2i.ZERO  # another resolution: re-create the images
	_divisor = maxf(divisor, 1.0)


func _create() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	_march = CloudShaderPort.build(PackedStringArray(["CLOUD_MARCH"]), PackedStringArray(MARCH_FILES))
	_kernel_mutex.lock()
	_pass_kernels = CloudKernels.new(kernel_specs(PackedStringArray(PASS_KERNELS)), "pass")
	_kernel_mutex.unlock()
	_request_baked(_stereo_guess)
	_globals_buffer = _rd.uniform_buffer_create(_march.block_size)
	_eyes_buffer = _rd.uniform_buffer_create(EYES_BLOCK_SIZE)
	_centre_buffer = _rd.uniform_buffer_create(CENTRE_BLOCK_SIZE)
	_samplers = CloudRD.create_samplers(_rd)
	_fallback = CloudRD.create_fallbacks(_rd)


## CloudKernels specs of the pass's kernels `names` (PASS_KERNELS): CLOUD_<NAME>, no globals of the includes needed.
static func kernel_specs(names: PackedStringArray) -> Array:
	var specs := []
	for name in names:
		specs.append([name, PackedStringArray(["CLOUD_" + name.to_upper()]), PackedStringArray(PASS_FILES), false])
	return specs


## Render thread: the baked volume's kernel build for a desktop or a stereo march (started once each).
func _request_baked(stereo: bool) -> CloudKernels:
	if _rd == null:
		return null
	_kernel_mutex.lock()
	if not _baked_kernels.has(stereo):
		_baked_kernels[stereo] = CloudKernels.new(CloudBaked.kernel_specs(stereo), "baked" + (" stereo" if stereo else ""))
	var kernels: CloudKernels = _baked_kernels[stereo]
	_kernel_mutex.unlock()
	return kernels


## Creates the shaders and pipelines of a built kernel set: 1, or (errors pushed, the set's shaders freed) 0 when a
## rejected cached binary is being rebuilt (CloudKernels.rebuild_rejected: poll and create again), -1 when it failed.
func _create_kernels(kernels: CloudKernels) -> int:
	var created: Array[String] = []
	for name: String in kernels.names():
		var shader := kernels.create_shader(_rd, name)
		if not shader.is_valid():
			for n in created:
				_rd.free_rid(_shaders[n])  # (frees its pipeline with it)
				_shaders.erase(n)
				_pipelines.erase(n)
			return 0 if kernels.rebuild_rejected() else -1
		_shaders[name] = shader
		_pipelines[name] = _rd.compute_pipeline_create(shader)
		created.append(name)
	return 1


## Render thread: the upsample created once built (false while building or failed). Never blocks: a render-thread wait
## for the workers deadlocked (the checks' measurements, 2026-10-30).
func _pass_ready() -> bool:
	if _pass_state == 0 and _pass_kernels.done():
		_pass_state = _create_kernels(_pass_kernels)
	return _pass_state == 1


## Render thread: the baked volume created once its kernels are built (false while building, while a rejected cached
## binary is rebuilt, or failed: then _baked_failed).
func _create_baked(stereo: bool) -> bool:
	var kernels := _request_baked(stereo)
	if not kernels.done():
		return false
	var start_us := Time.get_ticks_usec()
	baked = CloudBaked.new()
	_baked_stereo = stereo
	if not baked.setup(_rd, kernels):
		baked = null
		_baked_failed = not kernels.rebuild_rejected()
		return false
	print("CloudEffect: baked volume's shaders created in %.2f s" % ((Time.get_ticks_usec() - start_us) / 1.0e6))
	return true


## A frame without clouds (kernels or the first baked set not ready): the result cleared to T = 1.
func _skip_frame(view_count: int) -> void:
	_rd.texture_clear(_output_rd, Color(0.0, 0.0, 0.0, 1.0), 0, 1, 0, maxi(view_count, 2))
	stats = {"valid": false, "waiting": true}


func _texture(type: int, size: Vector2i, layers: int, format: int) -> RID:
	var tf := RDTextureFormat.new()
	tf.texture_type = type
	tf.width = size.x
	tf.height = size.y
	tf.depth = 1
	tf.array_layers = layers
	tf.format = format
	# Copy-from: the checks read the result back.
	tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_STORAGE_BIT \
			| RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT | RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT \
			| RenderingDevice.TEXTURE_USAGE_CAN_COPY_TO_BIT  # (copy-to: _skip_frame clears the result)
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
	image_creations += 1
	centre_low = low
	var old_output := _output_rd
	_size = size
	_view_count = view_count
	_low = low
	# At least two layers: a Texture2DArrayRD needs an array (the mono view uses layer 0).
	_output_rd = _texture(RenderingDevice.TEXTURE_TYPE_2D_ARRAY, size, maxi(view_count, 2),
			RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	var c := {"low": low}
	c["color"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT)
	c["depth"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R32G32_SFLOAT)
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
		_shaders.erase("baked")  # the baked volume's own
		if baked != null:
			baked.free_all()
		for shader: RID in _shaders.values():
			_rd.free_rid(shader)  # frees the pipelines with them
		for sampler: RID in _samplers.values():
			_rd.free_rid(sampler)
		for texture: RID in _fallback.values():
			_rd.free_rid(texture)
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
	# The size held through the runtime's field-of-view jitter (a new one re-creates the images, shown a frame late).
	var centre_view := CloudCentreView.keep_size(CloudCentreView.build(projections, size, _divisor),
			_low if size == _size and view_count == _view_count else Vector2i.ZERO)
	_ensure_images(size, view_count, centre_view["low"])

	# The camera (float64 from the floating-origin globals).
	var hi: Vector3 = ShaderGlobals.value(&"atmosphere_camera_hi")
	var lo: Vector3 = ShaderGlobals.value(&"atmosphere_camera_lo")
	var camera := hi + lo

	# Set 0 of the march: the globals' block and samplers.
	var set0 := CloudRD.set0(_rd, _march, _globals_buffer, _samplers, _fallback)
	# The march: the baked volume's (CloudBaked; its bake dispatches run in frame(), before the march). The frame shows
	# no clouds while its kernels or its first set are not ready (_skip_frame).
	var pixel_angle: float = centre_view["pixel_angle"]
	baked_shown = false
	if not _pass_ready() or _baked_failed:
		_skip_frame(view_count)
		return
	if baked != null and _baked_stereo != stereo:  # the view count changed (an XR session started or ended)
		_shaders.erase("baked")
		_pipelines.erase("baked")
		baked.free_all()
		baked = null
	if baked == null and not _create_baked(stereo):
		_skip_frame(view_count)  # its kernels are being built
		return
	var baked_set := baked.frame(set0, _samplers, camera, pixel_angle)
	if baked_set.is_empty():
		_skip_frame(view_count)  # its first set is being baked
		return
	var kernel := baked.march_kernel()
	_shaders["baked"] = baked.shader(kernel)
	_pipelines["baked"] = baked.pipeline(kernel)
	baked_shown = true
	var march_set0 := UniformSetCacheRD.get_cache(_shaders["baked"], 0, set0)

	var world_to_earth: Basis = ShaderGlobals.value(&"atmosphere_world_to_earth")
	# Earth-local offsets from the camera -> render world (the camera's world position: the floating origin's offset).
	var earth_to_world := Projection(Transform3D(world_to_earth.transposed(), scene.get_cam_transform().origin))
	var nearest_clamp: RID = _samplers["false|false|false"]
	# The head's transform: the renderer's per-eye projection already contains the eye offset (applied once here).
	var view_to_world := scene.get_cam_transform()
	var low: Vector2i = _centre["low"]

	# The centre view's block: the desktop's view itself; in VR the head with the centre projection (no eye offset).
	# The equiangular centre image (stereo only, CloudCentreView): the march's flag (CloudView params.x) and the eyes'
	# warp (CloudCentre warp; 0: rectilinear).
	var centre_projection: Projection = centre_view["projection"]
	var centre_eye := Vector3.ZERO if stereo \
			else world_to_earth * (view_to_world.basis * scene.get_view_eye_offset(0))
	var centre_full := Vector2(low) * _divisor if stereo else Vector2(size)
	var centre_clip := centre_projection * Projection(view_to_world.affine_inverse()) * earth_to_world
	var warp: Vector4 = centre_view.get("warp", Vector4.ZERO) if stereo else Vector4.ZERO
	_rd.buffer_update(_centre["buffer"], 0, VIEW_BLOCK_SIZE, _view_block(centre_projection, view_to_world,
			world_to_earth, centre_full, low, centre_eye, warp != Vector4.ZERO, pixel_angle).to_byte_array())
	var centre_bytes := _matrix_floats(centre_clip)
	centre_bytes.append_array([centre_full.x, centre_full.y, low.x, low.y, 1.0 if stereo else 0.0, 0.0, 0.0, 0.0])
	centre_bytes.append_array([warp.x, warp.y, warp.z, warp.w])
	_rd.buffer_update(_centre_buffer, 0, CENTRE_BLOCK_SIZE, centre_bytes.to_byte_array())
	var march_set1: Array = [CloudRD.ubo(_centre["buffer"]),
			CloudRD.tex(1, nearest_clamp, buffers.get_depth_layer(0, false)), CloudRD.image(2, _centre["color"]),
			CloudRD.image(3, _centre["depth"])]
	if stereo:
		var eyes := PackedFloat32Array()
		for view in range(2):
			eyes.append_array(_matrix_floats(projections[view]))
		for view in range(2):
			eyes.append_array(_matrix_floats(projections[view].inverse()))
		eyes.append_array([size.x, size.y, 0.0, 0.0])
		_rd.buffer_update(_eyes_buffer, 0, EYES_BLOCK_SIZE, eyes.to_byte_array())
		march_set1.append_array([CloudRD.tex(4, nearest_clamp, buffers.get_depth_layer(1, false)),
				CloudRD.ubo(_eyes_buffer, 5)])
	march_set1.append_array(baked_set)  # the baked volume's set 1 (its grids, volumes, flat levels, S)

	var groups_low := Vector2i((low.x - 1) / GROUP + 1, (low.y - 1) / GROUP + 1)
	var groups_full := Vector2i((size.x - 1) / GROUP + 1, (size.y - 1) / GROUP + 1)
	for view in range(view_count):  # the blocks first: no buffer updates inside a compute list
		# Each eye's block (its projection and offset; the upsample reads the centre's image through CloudCentre).
		var eye_earth := world_to_earth * (view_to_world.basis * scene.get_view_eye_offset(view))
		_rd.buffer_update(_views[view]["buffer"], 0, VIEW_BLOCK_SIZE, _view_block(projections[view], view_to_world,
				world_to_earth, Vector2(size), low, eye_earth, false, pixel_angle).to_byte_array())

	var list := _rd.compute_list_begin()
	list = _stamp(list, "cloud_pass_march")
	_rd.compute_list_bind_compute_pipeline(list, _pipelines["baked"])
	_rd.compute_list_bind_uniform_set(list, march_set0, 0)
	_rd.compute_list_bind_uniform_set(list, _set1("baked", march_set1), 1)
	_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
	_rd.compute_list_add_barrier(list)
	list = _stamp(list, "cloud_pass_upsample")
	for view in range(view_count):
		var v: Dictionary = _views[view]
		_rd.compute_list_bind_compute_pipeline(list, _pipelines["upsample"])
		_rd.compute_list_bind_uniform_set(list, _set1("upsample", [CloudRD.ubo(v["buffer"]),
				CloudRD.tex(1, nearest_clamp, buffers.get_depth_layer(view, false)),
				CloudRD.tex(2, nearest_clamp, _centre["color"]), CloudRD.tex(3, nearest_clamp, _centre["depth"]),
				CloudRD.image(4, v["output_slice"]), CloudRD.ubo(_centre_buffer, 5)]), 1)
		_rd.compute_list_dispatch(list, groups_full.x, groups_full.y, 1)
	_rd.compute_list_end()
	stats = {"valid": true, "low_size": low, "views": view_count, "baked": baked_shown,
		"record_ms": (Time.get_ticks_usec() - start_us) / 1000.0}


## Diagnostics (capture_timestamps): ends the compute list, stamps the GPU time where the interval `name` starts and
## begins a new list (a timestamp cannot be taken inside a list); the list unchanged otherwise. The cloud pass's
## kernels are named cloud_pass_<kernel> (terrain_perf / render_check --gpu_passes sum them as cloud_pass).
func _stamp(list: int, name: String) -> int:
	if not CloudEffect.capture_timestamps:
		return list
	_rd.compute_list_end()
	_rd.capture_timestamp(name)
	return _rd.compute_list_begin()


## The CloudView block (cloud_pass.gdshaderinc): inverse projection, view -> world, world -> Earth axes, sizes (full,
## low), the eye's Earth-local offset, params (x: 1 = the equiangular centre image, w: the march pixel's angle).
static func _view_block(projection: Projection, view_to_world: Transform3D, world_to_earth: Basis, full: Vector2,
		low: Vector2i, eye_earth: Vector3, equiangular: bool, pixel_angle: float) -> PackedFloat32Array:
	var bytes := PackedFloat32Array()
	for m in [projection.inverse(), Projection(view_to_world), Projection(Transform3D(world_to_earth, Vector3.ZERO))]:
		bytes.append_array(_matrix_floats(m))
	bytes.append_array([full.x, full.y, low.x, low.y])
	bytes.append_array([eye_earth.x, eye_earth.y, eye_earth.z, 0.0])
	bytes.append_array([1.0 if equiangular else 0.0, 0.0, 0.0, pixel_angle])
	return bytes


static func _matrix_floats(m: Projection) -> PackedFloat32Array:
	var floats := PackedFloat32Array()
	for c in range(4):
		var col: Vector4 = m[c]
		floats.append_array([col.x, col.y, col.z, col.w])
	return floats


func _set1(kernel: String, uniforms: Array) -> RID:
	var typed: Array[RDUniform] = []
	typed.assign(uniforms)
	return UniformSetCacheRD.get_cache(_shaders[kernel], 1, typed)

