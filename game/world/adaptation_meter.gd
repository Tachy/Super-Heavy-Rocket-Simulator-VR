class_name AdaptationMeter
extends CompositorEffect
## Scene-referred eye adaptation (Task 14 of the Moon procedural surface plan, user 2026-11-05): the luminance of the
## field of view the eye is adapted to, measured on the rendered HDR scene. A compute step before the transparent pass
## (after the opaque pass, the sky and the cloud pass; before the rods' post step, glow and the tone mapper: the cones'
## image as the shaders drew it) samples the frame on a sparse grid, composites the cloud pass's result over each sample
## as the transparent pass's composite quad does (CloudEffect.frame_output_layer: T x scene + colour), de-gains it to
## physical units (scene value / atmosphere scene_per_cd_m2 of that frame: the pre-exposure is linear) and adds it,
## weighted toward the view's centre, into two log-luminance histograms (photopic BT.709, scotopic SCOTOPIC_WEIGHTS).
## Before the transparent pass on purpose (Task 14 fix round 1, review I-1): what is drawn there besides the cloud
## composite is display-referred (vision.gdshaderinc vision_display_referred: the plume, the stars' own brightness, orbit
## lines, labels, the VR panel - a fixed display value D, de-gained to D / gain) or a lamp lens (a highlight, cut
## anyway). Counted, such an emitter feeds the gain back on itself (log g_new = C + s log g, s its weight share): in a
## black field it is the only sample left (s = 1) and the gain runs to a clamp set by D, not by a physical luminance.
## Without them the field and so the target gain do not depend on the gain at all (render_check --gain_probe).
## Both VR eyes go into the same histograms: one adaptation
## for both eyes. The histograms come back asynchronously (RenderingDevice.buffer_get_data_async, no stall) a few
## frames later; field_log_average turns one into the field luminance: the centre-weighted geometric mean over the
## field, without the brightest HIGHLIGHT_SHARE of the weight (the Sun's disc, specular glints) and without what lies
## more than SIMULTANEOUS_RANGE_DECADES below the field's highlights (black sky: no signal at that adaptation).
## Atmosphere.update uses it in place of the white-plate rule (Atmosphere.scene_referred_adaptation; eye-adaptation.md).
## Without a RenderingDevice (headless, Compatibility) it measures nothing and the analytic rule stays.

## Histogram layout: HISTOGRAM_BINS per histogram, bin 0 everything at or below 10^LOG_MIN (black), bins 1 ..
## HISTOGRAM_BINS - 1 evenly in log10 over [LOG_MIN, LOG_MAX] (the top bin also takes everything above).
const HISTOGRAM_BINS := 128
## [call] The luminance range of the bins (log10 cd/m^2): 1e-6 (below the darkest night sky's 1.7e-4 cd/m^2) to 1e6
## (above any sunlit surface; the Sun's disc, 1.6e9, lands in the top bin and is clipped as a highlight).
const LOG_MIN := -6.0
const LOG_MAX := 6.0
## [E] The centre weighting: weight exp(-r^2 / (2 sigma^2)), r the distance from the image centre in half image
## diagonals. The adaptation state follows the luminance near the line of sight more than the periphery (Moon & Spencer
## 1945, "The visual effect of non-uniform surrounds", JOSA 35, 233: the surround's effect falls with eccentricity);
## sigma 0.5 keeps the corners at exp(-2) = 0.14 of the centre. [call] Per pixel, from the image centre: in VR each
## eye's projection centre is off the image centre (asymmetric frustum) and wide-angle pixels subtend less solid angle
## (the corners over-sampled ~cos^3); both are small against this Gaussian and not corrected.
const CENTRE_SIGMA := 0.5
## [E] Share of the weight dropped from the bright end before averaging: the Sun's disc and specular highlights must not
## set the adaptation (the 98th percentile, as the usual auto-exposure high cut).
const HIGHLIGHT_SHARE := 0.02
## [S: Kunkel & Reinhard 2010, "A reassessment of the simultaneous dynamic range of the human visual system", APGV
## 2010: ~3.7 log units at a fixed adaptation] luminances further below the field's highlights carry no signal at that
## adaptation state and do not pull it down (a black sky beside sunlit ground; the dark beyond a lamp-lit pad).
const SIMULTANEOUS_RANGE_DECADES := 3.7
## [call] The anchored rule (under evaluation, Atmosphere.FieldRule.ANCHOR; Gilchrist et al. 1999, "An anchoring theory
## of lightness perception", Psychol. Rev. 106, 795: the highest luminance of a sufficiently large area is seen as white):
## the anchor is this weighted percentile of the field left after the highlight cut - an area of more than ~5 % of the
## centre-weighted field (after the 2 % cut) sets white; a smaller bright spot does not.
const ANCHOR_PERCENTILE := 0.95
## [call] Samples per view about this many (a sparse grid; the histogram is a statistic, not an image).
const SAMPLES_PER_VIEW := 65536
## [call] The meter measures every this many frames (review I-3: the MSAA resolve it needs before the transparent pass
## cost 0.26 ms per frame in the stereo mock, 3534 x 2838 per eye; a result every 4 frames is 44 ms at 90 Hz, far
## inside the light adaptation's 0.5 s).
const MEASURE_EVERY := 4
## Weight quantisation of one sample (an integer histogram: atomic adds).
const WEIGHT_SCALE := 255.0
## The compute's workgroup edge (GROUP x GROUP invocations; each clears and flushes every GROUP^2-th shared bin).
const GROUP := 16

## TOTAL_BINS and GROUP_EDGE are set from HISTOGRAM_BINS and GROUP (shader_source).
const SHADER_CODE := """
#version 450
layout(local_size_x = GROUP_EDGE, local_size_y = GROUP_EDGE, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D color_image;
layout(set = 0, binding = 1, std430) restrict buffer Histogram {
	uint bins[];
} histogram;
// The cloud pass's result (CloudEffect: premultiplied colour, .a the background's transmittance); params.shape2.x = 0:
// no clouds this frame (then bound to the colour image itself, unread).
layout(rgba16f, set = 0, binding = 2) uniform restrict readonly image2D cloud_image;
layout(push_constant, std430) uniform Params {
	vec4 size;    // width, height, stride (px), cd/m^2 per scene value
	vec4 shape;   // 1 / (2 sigma^2), bins per histogram, log10 min, bins per decade
	vec4 shape2;  // clouds (1 / 0), -, -, -
} params;

const vec3 LUMINANCE_WEIGHTS = vec3(0.2126, 0.7152, 0.0722);  // [S: ITU-R BT.709]
const vec3 SCOTOPIC_WEIGHTS = vec3(0.0, 1.4730, 0.9238);      // EyeAdaptation.SCOTOPIC_WEIGHTS
shared uint local_bins[TOTAL_BINS];

uint bin_of(float luminance) {
	if (!(luminance > 0.0)) {
		return 0u;
	}
	float x = (log(luminance) / log(10.0) - params.shape.z) * params.shape.w;
	if (x < 0.0) {
		return 0u;
	}
	return uint(clamp(floor(x) + 1.0, 1.0, params.shape.y - 1.0));
}

void main() {
	uint index = gl_LocalInvocationIndex;
	for (uint k = index; k < uint(TOTAL_BINS); k += uint(GROUP_EDGE * GROUP_EDGE)) {
		local_bins[k] = 0u;
	}
	barrier();
	int stride = int(params.size.z);
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy) * stride + ivec2(stride / 2);
	if (pixel.x < int(params.size.x) && pixel.y < int(params.size.y)) {
		vec3 c = imageLoad(color_image, pixel).rgb;
		if (params.shape2.x > 0.5) {
			vec4 cloud = imageLoad(cloud_image, pixel);  // cloud_layer(_light).gdshader's composite
			c = c * cloud.a + cloud.rgb;
		}
		if (any(isnan(c))) {
			c = vec3(0.0);
		}
		c = clamp(c, vec3(0.0), vec3(65000.0));
		vec2 offset = (vec2(pixel) + 0.5 - 0.5 * params.size.xy) / (0.5 * length(params.size.xy));
		uint weight = uint(255.0 * exp(-dot(offset, offset) * params.shape.x) + 0.5);
		if (weight > 0u) {
			uint bins = uint(params.shape.y);
			atomicAdd(local_bins[bin_of(dot(LUMINANCE_WEIGHTS, c) * params.size.w)], weight);
			atomicAdd(local_bins[bins + bin_of(dot(SCOTOPIC_WEIGHTS, c) * params.size.w)], weight);
		}
	}
	barrier();
	for (uint k = index; k < uint(TOTAL_BINS); k += uint(GROUP_EDGE * GROUP_EDGE)) {
		if (local_bins[k] != 0u) {
			atomicAdd(histogram.bins[k], local_bins[k]);
		}
	}
}
"""

const _BYTES := HISTOGRAM_BINS * 2 * 4

## The cloud pass whose result the meter composites over the scene (world.gd; before it in the Compositor, same stage);
## null: no clouds composited.
var cloud_effect: CloudEffect
## Diagnostics (render_check --meter_post_transparent): measure after the transparent pass as Task 14 first did (the
## display-referred emitters counted, the cloud composite already in the image); set before the effect is attached.
var post_transparent := false:
	set(value):
		post_transparent = value
		effect_callback_type = EFFECT_CALLBACK_TYPE_POST_TRANSPARENT if value else EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT

var _rd: RenderingDevice
var _shader := RID()
var _pipeline := RID()
var _buffer := RID()
var _no_cloud := RID()
var _frame := 0  ## main thread: frames since the start (MEASURE_EVERY)
## Render-thread state (set through set_frame on the render thread, in order with the frame's globals).
var _cd_per_scene := 0.0
var _epoch := 0
var _loading := false
var _in_flight := false
## Results (written on the render thread by the read-back callback, read on the main thread).
var _mutex := Mutex.new()
var _result := PackedInt32Array()
var _result_epoch := -1
var _result_loading := false
var _result_serial := 0


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT
	access_resolved_color = false  # MSAA: the opaque scene resolved before this stage on measuring frames (set_frame)
	RenderingServer.call_on_render_thread(_create_pipeline)


## Main thread, per frame: the scene value of 1 cd/m^2 this frame draws with (Atmosphere.scene_per_cd_m2; <= 0: measure
## nothing), the adaptation's epoch (a new view: results of older epochs are dropped) and whether the view is still
## loading (streamed terrain: the result is not the settled view yet).
func set_frame(scene_per_cd_m2: float, epoch: int, loading: bool) -> void:
	var cd_per_scene := 1.0 / scene_per_cd_m2 if scene_per_cd_m2 > 0.0 else 0.0
	# Only every MEASURE_EVERY-th frame asks for the resolved colour (MSAA: an extra resolve before the transparent pass);
	# the flag and this frame's state go through the same render-thread queue, so they belong to the same frame.
	_frame += 1
	var measure := cd_per_scene > 0.0 and _frame % MEASURE_EVERY == 0
	if access_resolved_color != measure:
		access_resolved_color = measure
	RenderingServer.call_on_render_thread(_set_on_render_thread.bind(cd_per_scene if measure else 0.0, epoch, loading))


## Main thread: the newest histogram pair of that epoch not taken yet ([counts, loading]), else [].
func take_result(epoch: int, after_serial: int) -> Array:
	_mutex.lock()
	var out := []
	if _result_epoch == epoch and _result_serial > after_serial and _result.size() == HISTOGRAM_BINS * 2:
		out = [_result, _result_loading, _result_serial]
	_mutex.unlock()
	return out


## The field luminance of one histogram (counts[offset .. offset + HISTOGRAM_BINS)): the weighted geometric mean of the
## bins' centre luminances after the highlight cut and without the bins more than SIMULTANEOUS_RANGE_DECADES below the
## highest bin left. -1: no samples; 0: nothing above black (a dark field).
static func field_log_average(counts: PackedInt32Array, offset := 0) -> float:
	var field := _field_bins(counts, offset)
	if field.is_empty():
		return -1.0
	var weights: PackedFloat64Array = field[0]
	if int(field[2]) <= 0:
		return 0.0
	var sum_w := 0.0
	var sum_log := 0.0
	for k in range(int(field[1]), int(field[2]) + 1):
		sum_w += weights[k]
		sum_log += weights[k] * bin_log_centre(k)
	if sum_w <= 0.0:
		return 0.0
	return pow(10.0, sum_log / sum_w)


## The anchor luminance of one histogram (the anchored rule under evaluation, Atmosphere.FieldRule.ANCHOR): the
## ANCHOR_PERCENTILE-th weighted percentile of the same field as field_log_average (after the highlight cut, inside the
## simultaneous range below the highest bin left), each bin's weight spread evenly in log10 across the bin (no step when
## the percentile crosses a bin edge). -1: no samples; 0: nothing above black (a dark field).
static func field_anchor(counts: PackedInt32Array, offset := 0) -> float:
	var field := _field_bins(counts, offset)
	if field.is_empty():
		return -1.0
	var weights: PackedFloat64Array = field[0]
	var lowest := int(field[1])
	var top := int(field[2])
	if top <= 0:
		return 0.0
	var sum_w := 0.0
	for k in range(lowest, top + 1):
		sum_w += weights[k]
	if sum_w <= 0.0:
		return 0.0
	var width := (LOG_MAX - LOG_MIN) / float(HISTOGRAM_BINS - 1)
	var target := ANCHOR_PERCENTILE * sum_w
	var below := 0.0
	for k in range(lowest, top + 1):
		if weights[k] > 0.0 and below + weights[k] >= target:
			return pow(10.0, bin_log_centre(k) + ((target - below) / weights[k] - 0.5) * width)
		below += weights[k]
	return pow(10.0, bin_log_centre(top) + 0.5 * width)


## The field both statistics share: [weights after the highlight cut, lowest bin inside the simultaneous range, highest
## bin left (<= 0: nothing above black)]; [] without samples.
static func _field_bins(counts: PackedInt32Array, offset: int) -> Array:
	var total := 0.0
	for k in range(HISTOGRAM_BINS):
		total += float(counts[offset + k])
	if total <= 0.0:
		return []
	var weights := PackedFloat64Array()
	weights.resize(HISTOGRAM_BINS)
	for k in range(HISTOGRAM_BINS):
		weights[k] = float(counts[offset + k])
	# The highlight cut, from the top.
	var cut := HIGHLIGHT_SHARE * total
	var top := HISTOGRAM_BINS - 1
	while top > 0 and cut > 0.0:
		var take := minf(cut, weights[top])
		weights[top] -= take
		cut -= take
		if weights[top] > 0.0:
			break
		top -= 1
	while top > 0 and weights[top] <= 0.0:
		top -= 1
	var per_decade := float(HISTOGRAM_BINS - 1) / (LOG_MAX - LOG_MIN)
	var lowest := maxi(1, top - int(ceil(SIMULTANEOUS_RANGE_DECADES * per_decade)))
	return [weights, lowest, top]


## log10 of bin k's centre luminance (k >= 1).
static func bin_log_centre(k: int) -> float:
	return LOG_MIN + (float(k) - 0.5) * (LOG_MAX - LOG_MIN) / float(HISTOGRAM_BINS - 1)


## The bin a luminance falls in (the shader's bin_of; tests).
static func bin_of(luminance_cd_m2: float) -> int:
	if not (luminance_cd_m2 > 0.0):
		return 0
	var x := (log(luminance_cd_m2) / log(10.0) - LOG_MIN) * float(HISTOGRAM_BINS - 1) / (LOG_MAX - LOG_MIN)
	if x < 0.0:
		return 0
	return int(clampf(floor(x) + 1.0, 1.0, float(HISTOGRAM_BINS - 1)))


## The compute's GLSL with its sizes from HISTOGRAM_BINS and GROUP (review M-1: the shared histogram follows the
## constant; no tie between the bin count and the workgroup size).
static func shader_source() -> String:
	return SHADER_CODE.replace("TOTAL_BINS", str(HISTOGRAM_BINS * 2)).replace("GROUP_EDGE", str(GROUP))


func _set_on_render_thread(cd_per_scene: float, epoch: int, loading: bool) -> void:
	_cd_per_scene = cd_per_scene
	_epoch = epoch
	_loading = loading


func _create_pipeline() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	var source := RDShaderSource.new()
	source.language = RenderingDevice.SHADER_LANGUAGE_GLSL
	source.source_compute = shader_source()
	var spirv := _rd.shader_compile_spirv_from_source(source)
	if spirv.compile_error_compute != "":
		push_error("AdaptationMeter: " + spirv.compile_error_compute)
		_rd = null
		return
	_shader = _rd.shader_create_from_spirv(spirv)
	if _shader.is_valid():
		_pipeline = _rd.compute_pipeline_create(_shader)
		var zeros := PackedByteArray()
		zeros.resize(_BYTES)
		_buffer = _rd.storage_buffer_create(_BYTES, zeros)
		var tf := RDTextureFormat.new()  # bound in place of the cloud result when there is none
		tf.width = 1
		tf.height = 1
		tf.format = RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT
		_no_cloud = _rd.texture_create(tf, RDTextureView.new())


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd != null:
		if _buffer.is_valid():
			_rd.free_rid(_buffer)
		if _no_cloud.is_valid():
			_rd.free_rid(_no_cloud)
		if _shader.is_valid():
			_rd.free_rid(_shader)  # frees the pipeline with it


func _render_callback(_callback_type: int, render_data: RenderData) -> void:
	if _rd == null or not _pipeline.is_valid() or not _buffer.is_valid() or not _no_cloud.is_valid() \
			or _cd_per_scene <= 0.0 or _in_flight:
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	if size.x == 0 or size.y == 0:
		return
	var stride := maxi(1, int(round(sqrt(float(size.x) * float(size.y) / float(SAMPLES_PER_VIEW)))))
	var samples_x := (size.x - 1) / stride + 1
	var samples_y := (size.y - 1) / stride + 1
	var groups_x := (samples_x - 1) / GROUP + 1
	var groups_y := (samples_y - 1) / GROUP + 1
	_rd.buffer_clear(_buffer, 0, _BYTES)
	for view in range(buffers.get_view_count()):  # both VR eyes into one histogram
		# The cloud composite (before the transparent pass the image has no clouds yet; after it, it has them).
		var cloud := cloud_effect.frame_output_layer(view) if cloud_effect != null and not post_transparent else RID()
		var push := PackedFloat32Array([float(size.x), float(size.y), float(stride), _cd_per_scene,
				1.0 / (2.0 * CENTRE_SIGMA * CENTRE_SIGMA), float(HISTOGRAM_BINS), LOG_MIN,
				float(HISTOGRAM_BINS - 1) / (LOG_MAX - LOG_MIN), 1.0 if cloud.is_valid() else 0.0, 0.0, 0.0, 0.0])
		var bytes := push.to_byte_array()
		var image_uniform := RDUniform.new()
		image_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
		image_uniform.binding = 0
		image_uniform.add_id(buffers.get_color_layer(view))
		var buffer_uniform := RDUniform.new()
		buffer_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_STORAGE_BUFFER
		buffer_uniform.binding = 1
		buffer_uniform.add_id(_buffer)
		var cloud_uniform := RDUniform.new()
		cloud_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
		cloud_uniform.binding = 2
		cloud_uniform.add_id(cloud if cloud.is_valid() else _no_cloud)
		var uniform_set := UniformSetCacheRD.get_cache(_shader, 0, [image_uniform, buffer_uniform, cloud_uniform])
		if CloudEffect.capture_timestamps and view == 0:  # render_check --gpu_passes
			_rd.capture_timestamp("adaptation_meter")
		var list := _rd.compute_list_begin()
		_rd.compute_list_bind_compute_pipeline(list, _pipeline)
		_rd.compute_list_bind_uniform_set(list, uniform_set, 0)
		_rd.compute_list_set_push_constant(list, bytes, bytes.size())
		_rd.compute_list_dispatch(list, groups_x, groups_y, 1)
		_rd.compute_list_end()
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp("transparent_and_post")
	_in_flight = true
	if _rd.buffer_get_data_async(_buffer, _on_data.bind(_epoch, _loading)) != OK:
		_in_flight = false


func _on_data(data: PackedByteArray, epoch: int, loading: bool) -> void:
	_mutex.lock()
	_result = data.to_int32_array()
	_result_epoch = epoch
	_result_loading = loading
	_result_serial += 1
	_mutex.unlock()
	_in_flight = false
