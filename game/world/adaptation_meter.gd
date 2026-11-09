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
## [E] Share of the lit weight (bins above black) dropped from the bright end before averaging: the Sun's disc and
## specular highlights must not set the adaptation (the 98th percentile of the lit field, as the usual auto-exposure
## high cut). Of the lit weight only, so that black sky does not enlarge the cut (user 2026-11-07).
const HIGHLIGHT_SHARE := 0.02
## [S: Kunkel & Reinhard 2010, "A reassessment of the simultaneous dynamic range of the human visual system", APGV
## 2010: ~3.7 log units at a fixed adaptation] luminances further below the field's highlights carry no signal at that
## adaptation state and do not pull it down (a black sky beside sunlit ground; the dark beyond a lamp-lit pad).
const SIMULTANEOUS_RANGE_DECADES := 3.7
## [call] The anchored rule (Atmosphere.FieldRule.ANCHOR, and the anchor half of the default PROTECT rule; Gilchrist et al. 1999, "An anchoring theory
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

## TOTAL_BINS (the shared bins: the field's two histograms and the veil's two), COUNTER_BASE (the Sun counters' index in the
## buffer, after the field's two) and GROUP_EDGE are set from HISTOGRAM_BINS and GROUP (shader_source).
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
// The glare spread's veil of the last frame (GlareSpreadEffect: A_0, the image part, the Sun's analytic part is not in it);
// params.veil.x = 0: none (then bound to the colour image itself, unread).
layout(rgba16f, set = 0, binding = 3) uniform restrict readonly image2D veil_image;
layout(push_constant, std430) uniform Params {
	vec4 size;    // width, height, stride (px), cd/m^2 per scene value
	vec4 shape;   // 1 / (2 sigma^2), bins per histogram, log10 min, bins per decade
	vec4 shape2;  // clouds (1 / 0), the Sun disc's "seen" threshold (cd/m^2), -, -
	vec4 ray;     // view ray of NDC (x, y): (x a + b, y c + d, -1) - a, b, c, d from the eye's projection
	vec4 sun;     // the Sun's direction in the eye's view axes, cos of the mask radius (> 1: no mask)
	vec4 veil;    // veil image present (1 / 0), its pixels per texel, its columns, rows
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
		// The Sun disc by its known direction (look fixes T5b fix round 1): masked out of the histograms; its weight
		// and the part that shows the disc (above the threshold: not hidden) go to the two counters after them.
		vec2 ndc = vec2(2.0 * (float(pixel.x) + 0.5) / params.size.x - 1.0, 1.0 - 2.0 * (float(pixel.y) + 0.5) / params.size.y);
		vec3 ray = normalize(vec3(ndc.x * params.ray.x + params.ray.y, ndc.y * params.ray.z + params.ray.w, -1.0));
		bool masked = dot(ray, params.sun.xyz) > params.sun.w;
		float luminance = dot(LUMINANCE_WEIGHTS, c) * params.size.w;
		if (weight > 0u && !masked && params.veil.x > 0.5) {
			// The eye's own scatter of the sources the display cannot show (the glare spread, one frame old): its own
			// histograms (the mean veil over the field, MEAN_VEIL), not added to the sample - a veil under a black sky would
			// bring the black back into the log average and lower it.
			vec3 veil = imageLoad(veil_image, min(pixel / int(params.veil.y), ivec2(int(params.veil.z) - 1, int(params.veil.w) - 1))).rgb;
			atomicAdd(local_bins[2u * uint(params.shape.y) + bin_of(dot(LUMINANCE_WEIGHTS, veil) * params.size.w)], weight);
			atomicAdd(local_bins[3u * uint(params.shape.y) + bin_of(dot(SCOTOPIC_WEIGHTS, veil) * params.size.w)], weight);
		}
		// The Sun's counters are exact on the pixels, not on the sparse grid (bug: Sun halo flicker, 2026-11-08): on the grid
		// the mask held a few samples (~4 for a desktop Sun) and one sample more or less switched the veil's visibility
		// between its values as the Sun moved by a fraction of a pixel. A cell that can reach the mask (its centre within the
		// mask radius widened by the cell's diagonal) is walked pixel by pixel; the counters are the pixels' sums.
		if (params.sun.w <= 1.0) {
			float cell_angle = 1.5 * float(stride) * 2.0 * abs(params.ray.z) / params.size.y;
			float cos_reach = params.sun.w * cos(cell_angle) - sqrt(max(1.0 - params.sun.w * params.sun.w, 0.0)) * sin(cell_angle);
			if (dot(ray, params.sun.xyz) > cos_reach) {
				ivec2 origin = ivec2(gl_GlobalInvocationID.xy) * stride;
				for (int dy = 0; dy < stride; dy++) {
					for (int dx = 0; dx < stride; dx++) {
						ivec2 q = origin + ivec2(dx, dy);
						if (q.x >= int(params.size.x) || q.y >= int(params.size.y)) {
							continue;
						}
						vec2 qndc = vec2(2.0 * (float(q.x) + 0.5) / params.size.x - 1.0, 1.0 - 2.0 * (float(q.y) + 0.5) / params.size.y);
						vec3 qray = normalize(vec3(qndc.x * params.ray.x + params.ray.y, qndc.y * params.ray.z + params.ray.w, -1.0));
						if (dot(qray, params.sun.xyz) <= params.sun.w) {
							continue;
						}
						vec2 qoffset = (vec2(q) + 0.5 - 0.5 * params.size.xy) / (0.5 * length(params.size.xy));
						uint qweight = uint(255.0 * exp(-dot(qoffset, qoffset) * params.shape.x) + 0.5);
						vec3 qc = imageLoad(color_image, q).rgb;
						if (params.shape2.x > 0.5) {
							vec4 qcloud = imageLoad(cloud_image, q);
							qc = qc * qcloud.a + qcloud.rgb;
						}
						if (any(isnan(qc))) {
							qc = vec3(0.0);
						}
						qc = clamp(qc, vec3(0.0), vec3(65000.0));
						if (qweight > 0u) {
							atomicAdd(histogram.bins[COUNTER_BASE], qweight);
							if (dot(LUMINANCE_WEIGHTS, qc) * params.size.w >= params.shape2.y) {
								atomicAdd(histogram.bins[COUNTER_BASE + 1], qweight);
							}
						}
					}
				}
			}
		}
		if (weight > 0u && masked) {
			// in the Sun's mask: not in the histograms (its counters are above)
		} else if (weight > 0u) {
			uint bins = uint(params.shape.y);
			atomicAdd(local_bins[bin_of(luminance)], weight);
			atomicAdd(local_bins[bins + bin_of(dot(SCOTOPIC_WEIGHTS, c) * params.size.w)], weight);
		}
	}
	barrier();
	for (uint k = index; k < uint(TOTAL_BINS); k += uint(GROUP_EDGE * GROUP_EDGE)) {
		if (local_bins[k] != 0u) {
			atomicAdd(histogram.bins[k < uint(COUNTER_BASE) ? k : k + 2u], local_bins[k]);  // the Sun's counters lie between
		}
	}
}
"""

## The two histograms, then the Sun mask's two counters (its weight, the part above the "seen" threshold), then the veil's
## two histograms (photopic, scotopic; the glare spread's image part, one frame old).
const _BYTES := (HISTOGRAM_BINS * 4 + 2) * 4
## [call] The Sun's mask radius over its apparent radius (the limb's glow and the sparse grid's aliasing; look fixes T5b
## fix round 1). The disc covers 1 / 1.2^2 of the mask (SUN_DISC_SHARE).
const SUN_MASK_RADIUS_FACTOR := 1.2
const SUN_DISC_SHARE := 1.0 / (SUN_MASK_RADIUS_FACTOR * SUN_MASK_RADIUS_FACTOR)
## Fidelity flag: the Sun disc masked out of the histograms (render_check --glare=off: counted as before, the 2 %
## highlight cut alone).
static var sun_mask_enabled := true

## The cloud pass whose result the meter composites over the scene (world.gd; before it in the Compositor, same stage);
## null: no clouds composited.
var cloud_effect: CloudEffect
## The glare spread (look fixes T13): its veil image of the last frame goes into the veil histograms (the Earth's / Moon's
## veil raises the field as it does on the retina: Atmosphere adds veil_mean to the field); null: none.
var glare_effect: GlareSpreadEffect
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
var _view := {}
var _sun_seen_cd_m2 := 0.0
## Results (written on the render thread by the read-back callback, read on the main thread).
var _mutex := Mutex.new()
var _result := PackedInt32Array()
var _result_epoch := -1
var _result_loading := false
var _result_serial := 0
var _result_view := {}


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT
	access_resolved_color = false  # MSAA: the opaque scene resolved before this stage on measuring frames (set_frame)
	RenderingServer.call_on_render_thread(_create_pipeline)


## Main thread, per frame: the scene value of 1 cd/m^2 this frame draws with (Atmosphere.scene_per_cd_m2; <= 0: measure
## nothing), the adaptation's epoch (a new view: results of older epochs are dropped) and whether the view is still
## loading (streamed terrain: the result is not the settled view yet). view: the frame's view for AdaptationSurround
## (returned with its result: the out-of-frame part belongs to the view the histograms were measured in).
## sun_seen_cd_m2: a masked sample at least this bright shows the Sun disc (not hidden behind something).
func set_frame(scene_per_cd_m2: float, epoch: int, loading: bool, view := {}, sun_seen_cd_m2 := 0.0) -> void:
	var cd_per_scene := 1.0 / scene_per_cd_m2 if scene_per_cd_m2 > 0.0 else 0.0
	# Only every MEASURE_EVERY-th frame asks for the resolved colour (MSAA: an extra resolve before the transparent pass);
	# the flag and this frame's state go through the same render-thread queue, so they belong to the same frame.
	_frame += 1
	var measure := cd_per_scene > 0.0 and _frame % MEASURE_EVERY == 0
	if access_resolved_color != measure:
		access_resolved_color = measure
	RenderingServer.call_on_render_thread(_set_on_render_thread.bind(cd_per_scene if measure else 0.0, epoch, loading,
			view, sun_seen_cd_m2))


## Whether the next set_frame is a measuring frame (the view is only needed then).
func measures_next_frame() -> bool:
	return (_frame + 1) % MEASURE_EVERY == 0


## Main thread: the newest histogram pair of that epoch not taken yet ([counts, loading, serial, view, [Sun mask weight,
## its weight above the "seen" threshold], the veil's two histograms (photopic then scotopic, HISTOGRAM_BINS each)]), else [].
func take_result(epoch: int, after_serial: int) -> Array:
	_mutex.lock()
	var out := []
	if _result_epoch == epoch and _result_serial > after_serial and _result.size() == HISTOGRAM_BINS * 4 + 2:
		out = [_result.slice(0, HISTOGRAM_BINS * 2), _result_loading, _result_serial, _result_view,
				[_result[HISTOGRAM_BINS * 2], _result[HISTOGRAM_BINS * 2 + 1]], _result.slice(HISTOGRAM_BINS * 2 + 2)]
	_mutex.unlock()
	return out


## The field luminance of one histogram (counts[offset .. offset + HISTOGRAM_BINS)): the weighted geometric mean of the
## bins' centre luminances after the highlight cut and without the bins more than SIMULTANEOUS_RANGE_DECADES below the
## highest bin left. -1: no samples; 0: nothing above black (a dark field).
static func field_log_average(counts: PackedInt32Array, offset := 0, cut_of_whole := false) -> float:
	var field := _field_bins(counts, offset, cut_of_whole)
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


## The mean veil over the field of one veil histogram (cd/m^2, arithmetic: the veil is light added over the whole retinal
## image): the bins' linear centre luminances over the whole weight (black included: no veil there), counts[offset ..
## offset + HISTOGRAM_BINS). 0 without samples. The bin width (0.094 decades) is a 5 % error of the mean at most.
static func veil_mean(counts: PackedInt32Array, offset := 0) -> float:
	var total := 0.0
	var sum := 0.0
	for k in range(HISTOGRAM_BINS):
		var w := float(counts[offset + k])
		total += w
		if k >= 1:
			sum += w * pow(10.0, bin_log_centre(k))
	return sum / total if total > 0.0 else 0.0


## The anchor luminance of one histogram (Atmosphere.FieldRule.ANCHOR and the default PROTECT): the
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
## bin left (<= 0: nothing above black)]; [] without samples. cut_of_whole: the cut is HIGHLIGHT_SHARE of the whole weight
## (black included) - AdaptationSurround's visibility statistic, which must not see a small lit object only one of the
## two fields has (the Earth's disc in a lunar sky, absent from the surround model) as a disagreement.
static func _field_bins(counts: PackedInt32Array, offset: int, cut_of_whole := false) -> Array:
	var total := 0.0
	for k in range(HISTOGRAM_BINS):
		total += float(counts[offset + k])
	if total <= 0.0:
		return []
	var weights := PackedFloat64Array()
	weights.resize(HISTOGRAM_BINS)
	for k in range(HISTOGRAM_BINS):
		weights[k] = float(counts[offset + k])
	# The highlight cut, from the top: HIGHLIGHT_SHARE of the lit weight (bins above black), not of the whole field
	# (user 2026-11-07): a lit surround that is a small share of a mostly black field must not be cut whole.
	var lit := total - weights[0]
	var cut := HIGHLIGHT_SHARE * (total if cut_of_whole else lit)
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
	return SHADER_CODE.replace("TOTAL_BINS", str(HISTOGRAM_BINS * 4)).replace("COUNTER_BASE", str(HISTOGRAM_BINS * 2)) \
			.replace("GROUP_EDGE", str(GROUP))


## The shader's Sun mask for one rendered view: [ray a, b, c, d, Sun x, y, z in the eye's view axes, cos of the mask
## radius]; the cosine 2 (no mask) without a Sun or for a degenerate projection. The view ray of NDC (x, y) is
## (x a + b, y c + d, -1) (GlareSpreadEffect.eye_frame: NDC y up, image row 0 at the top, Vulkan's y flip taken out).
## The eye's frustum and orientation come from the RENDER data (projection: the view's render projection, head: the render
## camera's basis; render thread), as the glare spread's Sun mask does (GlareSpreadEffect.view_frame): never from the
## view dictionary's "eyes". Those are read on the main thread during _process, and in OpenXR the eye poses there are the
## ones the PREVIOUS frame was rendered with (OpenXRAPI::pre_render locates the views right before rendering), so a head
## turning by more than the mask's margin per frame put the mask beside the drawn disc: the "seen" counter dropped to 0 and
## the Sun's veil / halo switched off and on (bug: Sun flicker in VR, 2026-11-09). From view only "sun" (its render-world
## direction, constant over a frame) and "sun_mask_cos".
static func sun_mask_parameters(view: Dictionary, projection: Projection, head: Basis, size: Vector2i) -> PackedFloat32Array:
	var none := PackedFloat32Array([1.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 2.0])
	if not sun_mask_enabled or not view.has("sun"):
		return none
	var frame := GlareSpreadEffect.view_frame(projection, head, view["sun"], size)
	if frame.is_empty():
		return none
	var ray: PackedFloat32Array = frame[0]
	var sun: Vector3 = frame[2]
	return PackedFloat32Array([ray[0], ray[1], ray[2], ray[3], sun.x, sun.y, sun.z, float(view.get("sun_mask_cos", 2.0))])


func _set_on_render_thread(cd_per_scene: float, epoch: int, loading: bool, view: Dictionary, sun_seen: float) -> void:
	_cd_per_scene = cd_per_scene
	_epoch = epoch
	_loading = loading
	if cd_per_scene > 0.0:
		_view = view
		_sun_seen_cd_m2 = sun_seen


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
	var scene_data := render_data.get_render_scene_data()
	var head := scene_data.get_cam_transform().basis.orthonormalized()  # the pose this frame is rendered with
	for view in range(buffers.get_view_count()):  # both VR eyes into one histogram
		# The cloud composite (before the transparent pass the image has no clouds yet; after it, it has them).
		var cloud := cloud_effect.frame_output_layer(view) if cloud_effect != null and not post_transparent else RID()
		var mask := sun_mask_parameters(_view, scene_data.get_view_projection(view), head, size)
		var push := PackedFloat32Array([float(size.x), float(size.y), float(stride), _cd_per_scene,
				1.0 / (2.0 * CENTRE_SIGMA * CENTRE_SIGMA), float(HISTOGRAM_BINS), LOG_MIN,
				float(HISTOGRAM_BINS - 1) / (LOG_MAX - LOG_MIN), 1.0 if cloud.is_valid() else 0.0, _sun_seen_cd_m2, 0.0, 0.0])
		push.append_array(mask)
		var veil: Array = glare_effect.veil_image(view) if glare_effect != null and not post_transparent else []
		push.append_array(PackedFloat32Array([1.0 if not veil.is_empty() else 0.0, float(veil[1]) if not veil.is_empty() else 1.0,
				float(veil[2]) if not veil.is_empty() else 1.0, float(veil[3]) if not veil.is_empty() else 1.0]))
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
		var veil_uniform := RDUniform.new()
		veil_uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
		veil_uniform.binding = 3
		veil_uniform.add_id(veil[0] if not veil.is_empty() else _no_cloud)
		var uniform_set := UniformSetCacheRD.get_cache(_shader, 0, [image_uniform, buffer_uniform, cloud_uniform, veil_uniform])
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
	if _rd.buffer_get_data_async(_buffer, _on_data.bind(_epoch, _loading, _view)) != OK:
		_in_flight = false


func _on_data(data: PackedByteArray, epoch: int, loading: bool, view: Dictionary) -> void:
	_mutex.lock()
	_result = data.to_int32_array()
	_result_epoch = epoch
	_result_loading = loading
	_result_view = view
	_result_serial += 1
	_mutex.unlock()
	_in_flight = false
