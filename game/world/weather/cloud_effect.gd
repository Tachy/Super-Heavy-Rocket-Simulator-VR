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
## With the cloud view cache (cloud view cache Task 4, the default: CloudLayer.view_cache) the frame instead fills the
## planner's batch of cache tiles and each eye looks the cache up (CLOUD_CACHE_LOOKUP into the same result). Task 5:
## while a cloud is within the near shell's radius of an anchor a shown tile has (CloudLayer.nearest_cloud_m), the
## march runs too, ended per ray at the shell around that tile's anchor (CLOUD_NEAR_SHELL, the preset's divisor), and
## the lookup composites it in front of the cache, which holds what lies beyond (its fill starts at the radius).

const MARCH_FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_light_cache.gdshaderinc", "res://shaders/cloud_far.gdshaderinc",
		"res://shaders/cloud_light.gdshaderinc", "res://shaders/cloud_pass.gdshaderinc"]
const PASS_FILES := ["res://shaders/cloud_pass.gdshaderinc"]
## The cloud view cache's fill (cloud view cache Task 3): the march's files and its kernel (CLOUD_CACHE_FILL).
const CACHE_FILL_FILES := ["res://shaders/atmosphere.gdshaderinc", "res://shaders/clouds.gdshaderinc",
		"res://shaders/cloud_light_cache.gdshaderinc", "res://shaders/cloud_far.gdshaderinc",
		"res://shaders/cloud_light.gdshaderinc", "res://shaders/cloud_pass.gdshaderinc",
		"res://shaders/cloud_view_cache.gdshaderinc"]
## The near shell's march (cloud view cache Task 5; CLOUD_MARCH + CLOUD_NEAR_SHELL): the march's files and the cache's
## (its rays end at the shell around their tile's anchor).
const NEAR_SHELL_FILES := CACHE_FILL_FILES
## CloudNearShell (cloud_view_cache.gdshaderinc): face texels, tiles per side, the current anchor; R_near
const NEAR_SHELL_PUSH_SIZE := 32
## Its per-eye lookup (cloud view cache Task 4; CLOUD_CACHE_LOOKUP, instead of the march and the upsample).
const CACHE_LOOKUP_FILES := ["res://shaders/cloud_pass.gdshaderinc", "res://shaders/cloud_view_cache.gdshaderinc"]
## CloudCacheLookup: now; face texels, tiles per side, slots per row, flags (bit 0 covering: the march's whole rays are
## in the output; bit 1 the near shell's result is; bits 8-15 the current anchor)
const CACHE_LOOKUP_PUSH_SIZE := 32
const CACHE_FLAG_COVERING := 1
const CACHE_FLAG_NEAR_SHELL := 2
## [E] The view's sample rays per eye (CloudViewCacheSchedule.pick's view_dirs): GRID x GRID over the eye's ndc - ~17
## deg apart over a headset eye's ~100 deg, ~13 x 8 deg on the desktop (9 x 9 cost the pick ~0.3 ms more a frame).
const CACHE_VIEW_GRID := 7
## [E] Measured fills / frames the main thread may hold back unread (the oldest are dropped).
const CACHE_FEEDBACK_MAX := 256
## The view cache's GPU timestamps (always taken: the budget control reads them): a fill's start (named by its batch's
## fill_id), the start of the march that runs with the cache shown (the near shell, + COVER_SUFFIX while the batch
## covers the view: up to the lookup), the lookup's start (the end of either).
const CACHE_FILL_STAMP := "cloud_cache_fill_"
const CACHE_MARCH_STAMP := "cloud_cache_march"
const CACHE_COVER_SUFFIX := "_cover"
const CACHE_LOOKUP_STAMP := "cloud_pass_lookup"
## What the frame shows of the view cache (_prepare_view_cache): nothing yet, the lookup over the covering march (whole
## rays), the lookup alone, the lookup behind the near shell (its march; whole rays where a tile has nothing yet).
enum CacheMode { OFF, COVERED, ALONE, NEAR_SHELL }
const GROUP := 8
const MAX_DISPATCH_GROUPS := 65535  # D3D12's limit per dispatch dimension
const CENTRE_IMAGES := ["color", "refined", "depth", "history0", "history1", "age0", "age1", "buffer", "outline",
		"counters"]
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
## Diagnostics (PerfLog): image sets created so far (each one a new result shown a frame late), the centre image size.
var image_creations := 0
var centre_low := Vector2i.ZERO
## Diagnostics (terrain_perf --gpu_passes): GPU timestamps around the cloud passes (RenderingDevice).
static var capture_timestamps := false
## Checks only (cloud_check.gd): >= 0 pins the frame index of the march's jitter (the same noise in two renders that
## differ in one setting); -1 in the game.
static var pinned_frame := -1
## Diagnostics (world volume plan Task 0; set before the first CloudEffect is created: the kernels are compiled then):
## counters adds the march's counters (clouds.gdshaderinc CLOUD_COUNTERS: density / light fetches, occupied / empty
## steps per march pixel; debug_centre_data("counters"), render_check --counters). ablate lists ABLATIONS to switch
## off for timing runs (CLOUD_ABLATE_<NAME>; render_check --ablate=<names>, or the environment variable
## STARSIM_CLOUD_ABLATE=<names>, comma separated). Both change nothing in the game (off: the kernels are the game's).
static var counters := false
static var ablate := _ablate_from_environment()
## The ablation names (see clouds.gdshaderinc and cloud_pass.gdshaderinc for what each removes): the fetch classes
## density (the view samples' shape, detail and tables), light (a sample's light-source evaluation) and air (the
## atmosphere tables at the clouds); field (the field's secondary fetches), shape and detail (everywhere, the light
## march included), subsamples (one sample per step), outline (the outline pass), air_shadow, far_tier.
const ABLATIONS := ["density", "light", "air", "field", "shape", "detail", "subsamples", "outline", "air_shadow",
		"far_tier"]
## Set 1 binding of the counters image (cloud_pass.gdshaderinc out_counters).
const COUNTERS_BINDING := 24
## The light cache (CloudLayer's; filled on the render thread before the march, cloud_light_cache.gd); null: none.
var light_cache: CloudLightCache
## The cloud view cache (cloud view cache Tasks 3-4; render thread): created by the first fill (a batch of
## CloudLayer's planner, or a check's request_view_cache_fill), re-created for a new face or tile size; null until
## then. With the shader global cloud_view_cache (CloudLayer.view_cache) and no debug output the frame shows it: the
## batch's tiles filled under the budget, then each eye's lookup (CLOUD_CACHE_LOOKUP) - no march, no upsample.
var view_cache: CloudViewCache
## Fills run so far (checks: a request was served).
var view_cache_fills := 0
## Render thread, the last frame (diagnostics): the view cache was shown; the tiles its batch filled.
var view_cache_shown := false
var view_cache_last_tiles := 0
## Render thread, the last frame (diagnostics, checks): the near shell ran (cloud view cache Task 5); frames it ran in
## so far (the checks count them: the flag is reset at every frame's start).
var near_shell_shown := false
var near_shell_frames := 0
## Render thread, the last frame with the cache shown: the farthest offset (m) of the camera from an anchor a shown tile
## has (the near shell's reach beyond R_near; CloudLayer predicts the near shell's cost from it, fix round 1).
var last_farthest_m := 0.0

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
var _fill_request := {}  ## {layout, anchor, tiles}: the next frame's view cache fill (render thread)
## The view cache's hand-overs (Task 4), guarded by _cache_mutex: the planner's batch (main -> render thread, {} =
## none), the view the render thread saw (render -> main: sample rays, their spacing, the face size), the measured fills
## ((fill id, GPU ms) each), the measured frames' rest (GPU ms without the fill and the march run with the cache shown)
## and that march's GPU ms (-1: none), the read-back nearest distances.
var _cache_mutex := Mutex.new()
var _cache_batch := {}
var _cache_view := {}
var _cache_fills: Array[Vector2] = []
var _cache_rests: Array[float] = []
var _cache_marches: Array[Vector2] = []
var _cache_readbacks: Array[Dictionary] = []
## Render thread: the batch last run (anchor, now, covering), the timestamps' frame last read.
var _cache_shown := {}
var _cache_timestamps_frame := -1
## Main -> render thread (guarded by _cache_mutex): the field's nearest cloud to the camera (m, a lower bound) and the
## near shell's radius (CloudLayer).
var _nearest_cloud_m := INF
var _near_radius_m := 0.0
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


## Main thread: fill the tiles ({tile, level} each) of a view cache laid out as layout from anchor_earth_m (Earth-local,
## float64) in the next frame, before its march.
func request_view_cache_fill(layout: CloudViewCacheLayout, anchor_earth_m: Vector3, tiles: Array[Dictionary]) -> void:
	RenderingServer.call_on_render_thread(_set_fill_request.bind(layout, anchor_earth_m, tiles))


## Checks: the raw bytes of one of the centre view's images ("color": the march's colour + T, "depth": its scene and
## cloud distances) - empty before the first frame.
func debug_centre_data(key: String) -> PackedByteArray:
	if _rd == null or not _centre.has(key):
		return PackedByteArray()
	return _rd.texture_get_data(_centre[key], 0)


## Main thread, every frame: the field's nearest cloud to the camera (m; CloudLayer.nearest_cloud_m) and the near
## shell's radius (m) - the render thread decides from them and the shown tiles' anchors whether the near shell runs.
func set_nearest_cloud(nearest_m: float, radius_m: float) -> void:
	_cache_mutex.lock()
	_nearest_cloud_m = nearest_m
	_near_radius_m = radius_m
	_cache_mutex.unlock()


## Main thread: the planner's next batch (CloudViewCachePlanner.plan) for the render thread.
func submit_view_cache_batch(batch: Dictionary) -> void:
	_cache_mutex.lock()
	_cache_batch = batch
	_cache_mutex.unlock()


## True while a submitted batch has not been taken by the render thread (the planner waits: no backlog).
func view_cache_batch_waiting() -> bool:
	_cache_mutex.lock()
	var waiting := not _cache_batch.is_empty()
	_cache_mutex.unlock()
	return waiting


## Render thread (tests: in place of it): takes the waiting batch ({} = none).
func take_view_cache_batch() -> Dictionary:
	_cache_mutex.lock()
	var batch := _cache_batch
	_cache_batch = {}
	_cache_mutex.unlock()
	return batch


## The view the render thread last saw ({dirs: Array[Vector3] Earth-local unit rays, sample_rad, face_texels}; {}
## before the first frame). Set by the render thread every frame with the view cache on (tests set it themselves).
func view_cache_view() -> Dictionary:
	_cache_mutex.lock()
	var view := _cache_view
	_cache_mutex.unlock()
	return view


func set_view_cache_view(view: Dictionary) -> void:
	_cache_mutex.lock()
	_cache_view = view
	_cache_mutex.unlock()


## Main thread: what the render thread measured since the last call - {fills: Array[Vector2] (fill id, its GPU ms) per
## fill whose timestamps came back, rests: Array[float] (a frame's GPU ms without its fill and its march with the cache
## shown), marches: Array[Vector2] (one per rest: that march's GPU ms, -1 without; 1 while the batch covered the
## view, else 0), readbacks: [{epoch, tiles, bits}]}.
func take_view_cache_feedback() -> Dictionary:
	_cache_mutex.lock()
	var feedback := {"fills": _cache_fills, "rests": _cache_rests, "marches": _cache_marches,
		"readbacks": _cache_readbacks}
	_cache_fills = []
	_cache_rests = []
	_cache_marches = []
	_cache_readbacks = []
	_cache_mutex.unlock()
	return feedback


func _set_fill_request(layout: CloudViewCacheLayout, anchor_earth_m: Vector3, tiles: Array[Dictionary]) -> void:
	_fill_request = {"layout": layout, "anchor": anchor_earth_m, "tiles": tiles}


func _set_quality(divisor: int, temporal_weight: float, clip_gamma: float) -> void:
	if maxi(divisor, 1) != _divisor:
		_size = Vector2i.ZERO  # another resolution: re-create the images
	_divisor = maxi(divisor, 1)
	_temporal_weight = clampf(temporal_weight, 0.0, 1.0)
	_clip_gamma = maxf(clip_gamma, 0.0)


func _reset_history() -> void:
	if not _centre.is_empty():
		_centre["valid"] = false


static func _ablate_from_environment() -> PackedStringArray:
	var names := PackedStringArray()
	for n in OS.get_environment("STARSIM_CLOUD_ABLATE").split(",", false):
		names.append(n.strip_edges())
	return names


## The preprocessor names of the diagnostics builds of the march's kernels (CloudShaderPort.build): CLOUD_COUNTERS and
## CLOUD_ABLATE_<NAME> for each ablation; an unknown name is an error.
static func diagnostic_defines() -> PackedStringArray:
	var defines := PackedStringArray()
	if counters:
		defines.append("CLOUD_COUNTERS")
	for n in ablate:
		if n in ABLATIONS:
			defines.append("CLOUD_ABLATE_" + n.to_upper())
		else:
			push_error("CloudEffect: unknown ablation '%s' (%s)" % [n, ", ".join(ABLATIONS)])
	return defines


func _create() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	_march = CloudShaderPort.build(PackedStringArray(["CLOUD_MARCH"]), PackedStringArray(MARCH_FILES))
	var kernels := {"march": _march.source,
		"fill": CloudShaderPort.build(PackedStringArray(["CLOUD_MARCH", "CLOUD_CACHE_FILL"]),
				PackedStringArray(CACHE_FILL_FILES)).source,
		"fill_prefilter": CloudShaderPort.build(PackedStringArray(["CLOUD_MARCH", "CLOUD_CACHE_FILL",
				"CLOUD_SHAPE_PREFILTER"]), PackedStringArray(CACHE_FILL_FILES)).source,
		"temporal": CloudShaderPort.build(PackedStringArray(["CLOUD_TEMPORAL"]), PackedStringArray(PASS_FILES),
				false).source,
		"upsample": CloudShaderPort.build(PackedStringArray(["CLOUD_UPSAMPLE"]), PackedStringArray(PASS_FILES),
				false).source,
		"outline": CloudShaderPort.build(PackedStringArray(["CLOUD_OUTLINE"]), PackedStringArray(PASS_FILES),
				false).source,
		"cache_lookup": CloudShaderPort.build(PackedStringArray(["CLOUD_CACHE_LOOKUP"]),
				PackedStringArray(CACHE_LOOKUP_FILES), false).source}
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
	# The near shell's variants (cloud view cache Task 5): the centre view and the shape prefilter (no outline pass).
	for centre in ["", "_centre"]:
		for prefilter in ["", "_prefilter"]:
			var defines := PackedStringArray(["CLOUD_MARCH", "CLOUD_NEAR_SHELL"])
			if centre != "":
				defines.append("CLOUD_CENTRE")
			if prefilter != "":
				defines.append("CLOUD_SHAPE_PREFILTER")
			kernels["march" + centre + "_near" + prefilter] = CloudShaderPort.build(defines,
					PackedStringArray(NEAR_SHELL_FILES)).source
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
	image_creations += 1
	centre_low = low
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
	if CloudEffect.counters:  # density fetches, light fetches, occupied steps, empty steps (clouds.gdshaderinc)
		c["counters"] = _texture(RenderingDevice.TEXTURE_TYPE_2D, low, 1, RenderingDevice.DATA_FORMAT_R32G32B32A32_UINT)
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
		if view_cache != null:
			view_cache.free_resources(_rd)
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
	# The size held through the runtime's field-of-view jitter (a new one re-creates the images, shown a frame late).
	var centre_view := CloudCentreView.keep_size(CloudCentreView.build(projections, size, _divisor),
			_low if size == _size and view_count == _view_count else Vector2i.ZERO)
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
			var slot_cascade := name.trim_prefix("cloud_light_cache_")  # "a3": slot a (0) or b (1), cascade 3
			texture = light_cache.texture(0 if slot_cascade.begins_with("a") else 1, slot_cascade.substr(1).to_int())
		u.add_id(texture if texture.is_valid() else _fallback.get(s["type"], _fallback["sampler2D"]))
		set0.append(u)
	# The shape prefilter's kernel (clouds.gdshaderinc) only when a footprint can exceed its start: the farthest cloud
	# sample (_shape_prefilter_needed).
	var family := "march_centre" if stereo else "march"
	var pixel_angle: float = centre_view["pixel_angle"]
	var march := family + "_prefilter" if _shape_prefilter_needed(pixel_angle) else family
	var march_set0 := UniformSetCacheRD.get_cache(_shaders[march], 0, set0)
	if not _fill_request.is_empty():
		_fill_view_cache(set0, camera)

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
	# averages the pixel already). The air shadow's checks 9 / 10 (the loss alone) take it as the game does.
	var refine := deterministic and (debug == 0 or debug == 9 or debug == 10) and not "outline" in CloudEffect.ablate
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
	if CloudEffect.counters:
		march_set1.append(_image(COUNTERS_BINDING, _centre["counters"]))
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
	# The view cache (Task 4): the planner's batch filled, then each eye's lookup - instead of the march (the debug
	# outputs are the march's per-ray quantities). Until the first batch ran, the march shows the frame; while the
	# batch says covering, the march runs too and the lookup shows it where a tile has nothing yet (fix round 1).
	view_cache_shown = false
	near_shell_shown = false
	var cache_mode := CacheMode.OFF
	if ShaderGlobals.value(&"cloud_view_cache") == true and debug == 0:
		_read_cache_timestamps()
		_publish_cache_view(projections, size, view_to_world, world_to_earth)
		cache_mode = _prepare_view_cache(set0, camera)
		if cache_mode == CacheMode.ALONE:
			_cache_lookup(camera, buffers, size, view_count, false, false)
			view_cache_shown = true
			_centre["valid"] = false  # the march's history restarts when it is shown again
			stats = {"valid": true, "low_size": low, "views": view_count, "view_cache": true,
				"record_ms": (Time.get_ticks_usec() - start_us) / 1000.0}
			return
		if cache_mode != CacheMode.OFF:
			# The march with the cache shown (covering or the near shell; fix round 1): its GPU time up to the lookup is
			# measured on its own - the budget adds it where it is predicted for the frame (never lagging in the rest).
			_rd.capture_timestamp(CACHE_MARCH_STAMP + (CACHE_COVER_SUFFIX if bool(_cache_shown.get("covering", false))
					else ""))
	var near_shell := cache_mode == CacheMode.NEAR_SHELL
	if near_shell:
		# The near shell's kernel (its family and the prefilter choice as the march's); no outline pass.
		march = family + "_near" + ("_prefilter" if march.ends_with("_prefilter") else "")
		march_set0 = UniformSetCacheRD.get_cache(_shaders[march], 0, set0)
		march_set1.append_array([_storage(8, view_cache.states_buffer()), _storage(9, view_cache.anchors_buffer())])
		for level in range(CloudViewCacheLayout.LEVELS):  # the lookup's distances: its tile / anchor rule (fix round 1)
			march_set1.append(_tex(10 + level, nearest_clamp, view_cache.distance_texture(level)))
		refine = false
	# The outline pass's set 1: the march's, its colour read (binding 6), the refined image written instead and the
	# outline list (binding 7, its count reset here: no buffer updates inside a compute list).
	var refine_set1 := []
	var warm_now := not _prefilter_warm and march == family  # the pre-warm below binds the refine set too
	if refine or warm_now:
		refine_set1 = march_set1.filter(func(u: RDUniform) -> bool: return u.binding != COUNTERS_BINDING)  # no counters
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
	list = _stamp(list, "cloud_pass_march")
	_rd.compute_list_bind_compute_pipeline(list, _pipelines[march])
	_rd.compute_list_bind_uniform_set(list, march_set0, 0)
	_rd.compute_list_bind_uniform_set(list, _set1(march, march_set1), 1)
	if near_shell:
		var push := PackedInt32Array([view_cache.layout.face_texels, view_cache.layout.tiles_per_side,
				int(_cache_shown["anchor_index"]), 0, 0, 0, 0, 0]).to_byte_array()
		push.encode_float(16, _cache_shown["near_radius_m"])
		_rd.compute_list_set_push_constant(list, push, NEAR_SHELL_PUSH_SIZE)
	_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
	if near_shell:
		# A new list: the RenderingDevice re-applies a list's push constant to the kernels bound after it (they have none).
		_rd.compute_list_end()
		list = _rd.compute_list_begin()
	else:
		_rd.compute_list_add_barrier(list)
	if refine:
		list = _stamp(list, "cloud_pass_outline")
		_rd.compute_list_bind_compute_pipeline(list, _pipelines["outline"])
		_rd.compute_list_bind_uniform_set(list, _set1("outline", [_ubo(_centre["buffer"]),
				_image(2, _centre["refined"]), _tex(6, nearest_clamp, _centre["color"]),
				_storage(7, _centre["outline"])]), 1)
		_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
		_rd.compute_list_add_barrier(list)
		list = _stamp(list, "cloud_pass_refine")
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
	list = _stamp(list, "cloud_pass_temporal")
	_rd.compute_list_bind_compute_pipeline(list, _pipelines["temporal"])
	_rd.compute_list_bind_uniform_set(list, _set1("temporal", [_ubo(_centre["buffer"]),
			_tex(1, nearest_clamp, _centre["refined" if refine else "color"]),
			_tex(2, nearest_clamp, _centre["depth"]),
			_tex(3, linear_clamp, history_in), _image(4, history_out), _tex(5, nearest_clamp, age_in),
			_image(6, age_out)]), 1)
	_rd.compute_list_dispatch(list, groups_low.x, groups_low.y, 1)
	_rd.compute_list_add_barrier(list)
	list = _stamp(list, "cloud_pass_upsample")
	for view in range(view_count):
		var v: Dictionary = _views[view]
		_rd.compute_list_bind_compute_pipeline(list, _pipelines["upsample"])
		_rd.compute_list_bind_uniform_set(list, _set1("upsample", [_ubo(v["buffer"]),
				_tex(1, nearest_clamp, buffers.get_depth_layer(view, false)), _tex(2, nearest_clamp, history_out),
				_tex(3, nearest_clamp, _centre["depth"]), _image(4, v["output_slice"]), _ubo(_centre_buffer, 5)]), 1)
		_rd.compute_list_dispatch(list, groups_full.x, groups_full.y, 1)
	_rd.compute_list_end()
	_centre["valid"] = true
	if cache_mode == CacheMode.COVERED or near_shell:
		_cache_lookup(camera, buffers, size, view_count, bool(_cache_shown["covering"]), near_shell)
		view_cache_shown = true
		near_shell_shown = near_shell
		near_shell_frames += 1 if near_shell else 0
	stats = {"valid": true, "low_size": low, "views": view_count, "view_cache": view_cache_shown,
		"covering": view_cache_shown and bool(_cache_shown["covering"]), "near_shell": near_shell,
		"record_ms": (Time.get_ticks_usec() - start_us) / 1000.0}


## The requested view cache fill (request_view_cache_fill) with this frame's set 0, before the march: the anchor's
## offset from the camera differenced in float64. A new face or tile size re-creates the cache.
func _fill_view_cache(set0: Array[RDUniform], camera: Vector3) -> void:
	_ensure_view_cache(_fill_request["layout"])
	var anchor: Vector3 = _fill_request["anchor"]
	var tiles: Array[Dictionary] = _fill_request["tiles"]
	view_cache.fill(_rd, CloudViewCache.pack_tiles(tiles), anchor - camera, set0)
	view_cache_fills += 1
	_fill_request = {}


## A view cache for this layout (a new face or tile size re-creates it).
func _ensure_view_cache(layout: CloudViewCacheLayout) -> void:
	if view_cache != null and view_cache.layout.face_texels == layout.face_texels \
			and view_cache.layout.tile_texels == layout.tile_texels:
		view_cache.layout = layout  # the planner's instance (the same geometry)
		return
	if view_cache != null:
		view_cache.free_resources(_rd)  # the RenderingDevice frees it after the frames in flight
	var kernels := {}
	for name in ["fill", "fill_prefilter"]:
		kernels[name] = {"shader": _shaders[name], "pipeline": _pipelines[name]}
	view_cache = CloudViewCache.new(_rd, layout, kernels, _shape_prefilter_needed)
	_cache_shown = {}


## The view cache's frame (Task 4): the waiting batch's tiles filled (timestamped by its fill id: the fill's GPU time
## steers the budget; the rays from the near shell's radius on, Task 5) and its shown states taken, the anchor ring's
## offsets from this frame's camera uploaded. OFF: nothing to show yet (before the first batch, or a restart planned:
## the march, never another place's clouds); NEAR_SHELL: a cloud within the near shell's radius (+ NEAR_SHELL_MARGIN_M)
## of the camera plus the farthest shown anchor's offset, or the batch covers the view (fix round 1) - the near shell's
## march (whole rays where a tile has nothing), the lookup behind it; COVERED (no near radius set yet): the march runs
## and the lookup shows it where a tile has nothing; ALONE: the lookup alone.
func _prepare_view_cache(set0: Array[RDUniform], camera: Vector3) -> CacheMode:
	var batch := take_view_cache_batch()
	view_cache_last_tiles = 0
	if batch.has("reset"):
		_cache_shown = {}  # a restart is planned: the march until its first batch (no tile of the old place / instant)
	elif not batch.is_empty():
		_ensure_view_cache(batch["layout"])
		var tiles: PackedInt32Array = batch["tiles"]
		var anchor: Vector3 = batch["anchor"]
		if not tiles.is_empty():
			_rd.capture_timestamp(CACHE_FILL_STAMP + str(int(batch["fill_id"])))
			var order := view_cache.fill(_rd, tiles, anchor - camera, set0, CloudLayer.near_radius_m())
			view_cache_fills += 1
			view_cache_last_tiles = tiles.size() / 3
			_rd.buffer_get_data_async(view_cache.near_buffer(), _on_cache_near.bind(int(batch["epoch"]), order), 0,
					4 * order.size())
		var states: PackedByteArray = batch["states"]
		if not states.is_empty():
			view_cache.set_states(_rd, states)
		_cache_shown = {"anchor": anchor, "now_s": float(batch["now_s"]), "covering": bool(batch["covering"]),
			"anchors": batch["anchors"], "anchor_index": int(batch["anchor_index"]),
			"anchor_tiles": batch["anchor_tiles"]}
	if view_cache == null or _cache_shown.is_empty():
		return CacheMode.OFF
	# The ring's offsets from this frame's camera (float64 differences) and the farthest anchor a shown tile has.
	var anchors: PackedVector3Array = _cache_shown["anchors"]
	var anchor_tiles: PackedInt32Array = _cache_shown["anchor_tiles"]
	var current: int = _cache_shown["anchor_index"]
	var offsets := PackedFloat32Array()
	offsets.resize(4 * anchors.size())
	var farthest_m := 0.0
	for k in range(anchors.size()):
		var offset := camera - anchors[k]
		offsets[4 * k] = offset.x
		offsets[4 * k + 1] = offset.y
		offsets[4 * k + 2] = offset.z
		if k == current or anchor_tiles[k] > 0:
			farthest_m = maxf(farthest_m, offset.length())
	view_cache.set_anchors(_rd, offsets.to_byte_array())
	last_farthest_m = farthest_m
	_cache_mutex.lock()
	var nearest_m := _nearest_cloud_m
	var radius_m := _near_radius_m
	_cache_mutex.unlock()
	_cache_shown["near_radius_m"] = radius_m
	# Covering with the near shell's kernel too (fix round 1): its whole rays only where a tile has no level (or no
	# settled anchor), the shell elsewhere - not the whole view's march for a few tiles dropped in flight.
	if radius_m > 0.0 and (_cache_shown["covering"]
			or nearest_m < radius_m + farthest_m + CloudLayer.NEAR_SHELL_MARGIN_M):
		return CacheMode.NEAR_SHELL
	return CacheMode.COVERED if _cache_shown["covering"] else CacheMode.ALONE


## Each eye's lookup into its layer of the result (covering: the march's result is there already - the lookup keeps it
## where a tile has nothing and fades from it where a first fill fades in; near_shell: the near shell's result is there
## - composited in front of the cache, kept where a tile has nothing).
func _cache_lookup(_camera: Vector3, buffers: RenderSceneBuffersRD, size: Vector2i, view_count: int,
		covering: bool, near_shell: bool) -> void:
	var layout := view_cache.layout
	var push := PackedByteArray()
	push.resize(CACHE_LOOKUP_PUSH_SIZE)
	push.encode_float(0, _cache_shown["now_s"])
	push.encode_u32(16, layout.face_texels)
	push.encode_u32(20, layout.tiles_per_side)
	push.encode_u32(24, CloudViewCachePlanner.SLOTS_PER_ROW)
	push.encode_u32(28, (CACHE_FLAG_COVERING if covering else 0) | (CACHE_FLAG_NEAR_SHELL if near_shell else 0)
			| (int(_cache_shown["anchor_index"]) << 8))
	var linear_clamp: RID = _samplers["true|false|false"]
	var nearest_clamp: RID = _samplers["false|false|false"]
	var groups_full := Vector2i((size.x - 1) / GROUP + 1, (size.y - 1) / GROUP + 1)
	_rd.capture_timestamp(CACHE_LOOKUP_STAMP)
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _pipelines["cache_lookup"])
	for view in range(view_count):
		var v: Dictionary = _views[view]
		var set1 := [_ubo(v["buffer"]), _tex(1, nearest_clamp, buffers.get_depth_layer(view, false))]
		for level in range(CloudViewCacheLayout.LEVELS):
			set1.append(_tex(2 + level, linear_clamp, view_cache.texture(level)))
			set1.append(_tex(5 + level, nearest_clamp, view_cache.distance_texture(level)))
		set1.append_array([_tex(8, linear_clamp, view_cache.slots_texture()),
				_storage(9, view_cache.states_buffer()), _image(10, v["output_slice"]),
				_storage(11, view_cache.anchors_buffer())])
		_rd.compute_list_bind_uniform_set(list, _set1("cache_lookup", set1), 1)
		_rd.compute_list_set_push_constant(list, push, CACHE_LOOKUP_PUSH_SIZE)
		_rd.compute_list_dispatch(list, groups_full.x, groups_full.y, 1)
	_rd.compute_list_end()


## Render thread (RenderingDevice's async read-back): a fill's nearest cloud distances for the main thread.
func _on_cache_near(bits: PackedByteArray, epoch: int, order: PackedInt32Array) -> void:
	_cache_mutex.lock()
	_cache_readbacks.append({"epoch": epoch, "tiles": order, "bits": bits})
	if _cache_readbacks.size() > CACHE_FEEDBACK_MAX:
		_cache_readbacks.remove_at(0)
	_cache_mutex.unlock()


## The frame whose GPU timestamps came back (once per frame): its fill's interval (named by the fill id: a frame whose
## stamps were never read loses only its own measurement), its march with the cache shown (covering or the near shell:
## from its stamp to the lookup's; -1 without) and its rest - the span of all its stamps (the frame's GPU time) without
## the fill and that march (the steady frame; the budget adds the march where it is predicted for its frame).
func _read_cache_timestamps() -> void:
	var frame := _rd.get_captured_timestamps_frame()
	if frame == _cache_timestamps_frame:
		return
	_cache_timestamps_frame = frame
	var count := _rd.get_captured_timestamps_count()
	if count < 2:
		return
	var fill_ms := 0.0
	var march_start := -1
	var covering := false
	var lookup_start := -1
	for i in range(count):
		var name := _rd.get_captured_timestamp_name(i)
		if name.begins_with(CACHE_MARCH_STAMP):
			march_start = i
			covering = name.ends_with(CACHE_COVER_SUFFIX)
		elif name == CACHE_LOOKUP_STAMP:
			lookup_start = i
		elif name.begins_with(CACHE_FILL_STAMP) and i + 1 < count:
			fill_ms = float(_rd.get_captured_timestamp_gpu_time(i + 1) - _rd.get_captured_timestamp_gpu_time(i)) / 1.0e6
			_cache_mutex.lock()
			_cache_fills.append(Vector2(name.trim_prefix(CACHE_FILL_STAMP).to_int(), fill_ms))
			if _cache_fills.size() > CACHE_FEEDBACK_MAX:
				_cache_fills.remove_at(0)
			_cache_mutex.unlock()
	var march_ms := -1.0  # no march with the cache shown in that frame
	if march_start >= 0 and lookup_start > march_start:
		march_ms = float(_rd.get_captured_timestamp_gpu_time(lookup_start)
				- _rd.get_captured_timestamp_gpu_time(march_start)) / 1.0e6
	var span_ms := float(_rd.get_captured_timestamp_gpu_time(count - 1)
			- _rd.get_captured_timestamp_gpu_time(0)) / 1.0e6
	if lookup_start < 0:
		return  # that frame did not show the cache (before its first batch, the flag off): no rest of the cache's frame
	_cache_mutex.lock()
	_cache_rests.append(maxf(span_ms - fill_ms - maxf(march_ms, 0.0), 0.0))
	_cache_marches.append(Vector2(march_ms, 1.0 if covering else 0.0))
	if _cache_rests.size() > CACHE_FEEDBACK_MAX:
		_cache_rests.remove_at(0)
		_cache_marches.remove_at(0)
	_cache_mutex.unlock()


## The view the planner picks for: CACHE_VIEW_GRID^2 sample rays per eye (Earth-local), half their largest diagonal
## spacing, and the face size of the eyes' finest pixel density at their centres.
func _publish_cache_view(projections: Array[Projection], size: Vector2i, view_to_world: Transform3D,
		world_to_earth: Basis) -> void:
	var to_earth := world_to_earth * view_to_world.basis
	var dirs: Array[Vector3] = []
	var sample_rad := 0.0
	var face := 0
	var g := CACHE_VIEW_GRID
	for projection in projections:
		var inverse := projection.inverse()
		var grid: Array[Vector3] = []
		for iy in range(g):
			for ix in range(g):
				var ndc := Vector2(ix, iy) * (2.0 / (g - 1)) - Vector2.ONE
				var a: Vector4 = inverse * Vector4(ndc.x, ndc.y, 0.5, 1.0)
				var b: Vector4 = inverse * Vector4(ndc.x, ndc.y, 1.0, 1.0)
				grid.append((to_earth * (Vector3(a.x, a.y, a.z) / a.w - Vector3(b.x, b.y, b.z) / b.w)).normalized())
		for iy in range(g - 1):
			for ix in range(g - 1):
				var k := iy * g + ix
				sample_rad = maxf(sample_rad, 0.5 * maxf(grid[k].angle_to(grid[k + g + 1]),
						grid[k + 1].angle_to(grid[k + g])))
		dirs.append_array(grid)
		face = maxi(face, CloudViewCacheLayout.face_texels_for(projection, size, CloudViewCache.TILE_TEXELS))
	set_view_cache_view({"dirs": dirs, "sample_rad": sample_rad, "face_texels": face})


## Diagnostics (capture_timestamps): ends the compute list, stamps the GPU time where the interval `name` starts and
## begins a new list (a timestamp cannot be taken inside a list); the list unchanged otherwise. The cloud pass's
## kernels are named cloud_pass_<kernel> (terrain_perf / render_check --gpu_passes sum them as cloud_pass).
func _stamp(list: int, name: String) -> int:
	if not CloudEffect.capture_timestamps:
		return list
	_rd.compute_list_end()
	_rd.capture_timestamp(name)
	return _rd.compute_list_begin()


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
	var debug := int(ShaderGlobals.value(&"cloud_debug_output"))
	if debug == 0 or debug >= 9:  # the game's hand-over (the air shadow's checks 9-12 too)
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
