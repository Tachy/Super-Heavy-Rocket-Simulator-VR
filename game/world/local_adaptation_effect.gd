class_name LocalAdaptationEffect
extends CompositorEffect
## Local adaptation on the GPU (look fixes T10, grid-phase invariant since T13; the model and its CPU mirror:
## LocalAdaptation). Two instances of this effect: the MAP stage before the transparent pass (after the field meter: the
## meter measures the unchanged image) builds, from the opaque image with the cloud composite (as the meter does), the
## piecewise-linear bilateral band maps (per log-luminance band the range weights of every full-resolution pixel,
## box-downsampled to the block grid, then a separable Gaussian blur); the APPLY stage after the transparent pass (after
## VisionEffect, before the display tone step, glow and AgX) reads, per pixel, its own two neighbouring bands, bilinearly
## upsampled, gets the local mean L_loc and multiplies the pixel by the factor c. What the transparent pass draws over a
## dark field (the display-referred emitters: plume, stars, labels, the VR panel; lamp lenses) is unlike every pixel
## there (small share) and keeps c = 1; over a locally adapted region it takes that region's factor. Per view (both VR
## eyes, each its own maps). Without a RenderingDevice (headless, Compatibility), with the eye adaptation off or
## LocalAdaptation.enabled false it does nothing. The MAP stage needs the resolved colour every frame (MSAA: an extra
## resolve before the transparent pass).

enum Stage { MAP, APPLY }

const DOWN_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D color_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict readonly image2D cloud_image;
layout(rgba16f, set = 0, binding = 2) uniform restrict writeonly image2D atlas_image;
layout(push_constant, std430) uniform Params {
	vec4 size;   // width, height, block, clouds (1 / 0)
	vec4 low;    // block columns, block rows, sample step (px; LocalAdaptation.sample_step), -
	vec4 bands;  // lowest band centre (log2), band spacing (log2), 1 / (2 sigma^2) at or below the band, ... above it
} params;
const vec3 LUMINANCE = vec3(@LW_R@, @LW_G@, @LW_B@);
const int BANDS = @BANDS@;

// LocalAdaptation.factors, step 1: the range weight of every pixel for every band, box-downsampled. The atlas holds the
// bands side by side (tile k = bands 2k, 2k+1): RGBA = (w L / Lc_j, w) of band 2k, then of band 2k+1.
void main() {
	ivec2 t = ivec2(gl_GlobalInvocationID.xy);
	int lw = int(params.low.x);
	if (t.x >= lw || t.y >= int(params.low.y)) {
		return;
	}
	int b = int(params.size.z);
	int step = int(params.low.z);
	float c0 = params.bands.x;
	float bstep = params.bands.y;
	float aw[BANDS];
	float al[BANDS];
	float centre[BANDS];
	float inv_centre[BANDS];
	for (int k = 0; k < BANDS; k++) {
		aw[k] = 0.0;
		al[k] = 0.0;
		centre[k] = c0 + float(k) * bstep;
		inv_centre[k] = exp2(-centre[k]);
	}
	float n = 0.0;
	for (int j = step / 2; j < b; j += step) {
		for (int i = step / 2; i < b; i += step) {
			ivec2 p = t * b + ivec2(i, j);
			if (p.x >= int(params.size.x) || p.y >= int(params.size.y)) {
				continue;
			}
			vec3 c = imageLoad(color_image, p).rgb;
			if (params.size.w > 0.5) {
				vec4 cloud = imageLoad(cloud_image, p);  // the transparent pass's composite (AdaptationMeter)
				c = c * cloud.a + cloud.rgb;
			}
			if (any(isnan(c))) {
				c = vec3(0.0);
			}
			float lum = dot(LUMINANCE, clamp(c, vec3(0.0), vec3(@MAXL@)));
			float x = log2(max(lum, 1.0e-30));
			for (int k = 0; k < BANDS; k++) {
				float d = x - centre[k];
				if (k == 0) {
					d = max(d, 0.0);
				}
				float w = exp(-d * d * (d > 0.0 ? params.bands.w : params.bands.z));
				aw[k] += w;
				al[k] += w * lum * inv_centre[k];
			}
			n += 1.0;
		}
	}
	float inv_n = n > 0.0 ? 1.0 / n : 0.0;
	for (int k = 0; k < BANDS / 2; k++) {
		imageStore(atlas_image, ivec2(k * lw + t.x, t.y), vec4(al[2 * k] * inv_n, aw[2 * k] * inv_n,
				al[2 * k + 1] * inv_n, aw[2 * k + 1] * inv_n));
	}
}
"""

const BLUR_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D source_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict writeonly image2D target_image;
layout(push_constant, std430) uniform Params {
	vec4 low;  // block columns, block rows, radius (texels), 1 / (2 sigma^2) (texels)
	vec4 dir;  // dx, dy, tiles, -
} params;

// One direction of the separable Gaussian, per tile of the atlas; in-bounds taps renormalised (LocalAdaptation._blur_plane).
void main() {
	ivec2 p = ivec2(gl_GlobalInvocationID.xy);
	int lw = int(params.low.x);
	int lh = int(params.low.y);
	if (p.x >= lw * int(params.dir.z) || p.y >= lh) {
		return;
	}
	int tile = p.x / lw;
	ivec2 q0 = ivec2(p.x - tile * lw, p.y);
	ivec2 d = ivec2(params.dir.xy);
	int radius = int(params.low.z);
	vec4 sum = vec4(0.0);
	float mass = 0.0;
	for (int i = -radius; i <= radius; i++) {
		ivec2 q = q0 + d * i;
		if (q.x < 0 || q.y < 0 || q.x >= lw || q.y >= lh) {
			continue;
		}
		float g = exp(-float(i * i) * params.low.w);
		sum += g * imageLoad(source_image, ivec2(tile * lw + q.x, q.y));
		mass += g;
	}
	imageStore(target_image, p, sum / mass);
}
"""

const APPLY_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform image2D color_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict readonly image2D atlas_image;
layout(push_constant, std430) uniform Params {
	vec4 size;    // width, height, block, block columns
	vec4 shape;   // block rows, lowest band centre (log2), band spacing (log2), band count
	vec4 factor;  // scene per cd/m^2, display gain, L_ref (cd/m^2), L_eff per L_loc
	vec4 extra;   // GAIN_MAX, MIN_WEIGHT_SHARE, FULL_WEIGHT_SHARE_RATIO, -
	vec4 cone_x;  // N = C M^-1: the cones' part of the post step's pixel (LocalAdaptation.cone_part), its columns
	vec4 cone_y;
	vec4 cone_z;
} params;
const vec3 LUMINANCE = vec3(@LW_R@, @LW_G@, @LW_B@);

// EyeAdaptation.cone_log_threshold [S: Ferwerda et al. 1996 Eq. 4]
float cone_log_threshold(float luminance) {
	float l = log(max(luminance, @LUM_FLOOR@)) / log(10.0);
	if (l <= @CONE_LOW@) {
		return @CONE_FLOOR@;
	}
	if (l >= @CONE_WEBER@) {
		return l - @WEBER_OFFSET@;
	}
	return pow(@MID_SLOPE@ * l + @MID_OFFSET@, @MID_EXPONENT@) + @CONE_FLOOR@;
}

// (w L / Lc, w) of one band, bilinear in space (the blurred low-res map brought to the pixel).
vec2 band_value(int band, ivec2 base, vec2 f, ivec2 last, int lw) {
	int tile = band / 2;
	bool second = (band - 2 * tile) == 1;
	ivec2 a = clamp(base, ivec2(0), last);
	ivec2 b = clamp(base + ivec2(1, 1), ivec2(0), last);
	vec4 v00 = imageLoad(atlas_image, ivec2(tile * lw + a.x, a.y));
	vec4 v10 = imageLoad(atlas_image, ivec2(tile * lw + b.x, a.y));
	vec4 v01 = imageLoad(atlas_image, ivec2(tile * lw + a.x, b.y));
	vec4 v11 = imageLoad(atlas_image, ivec2(tile * lw + b.x, b.y));
	vec4 v = mix(mix(v00, v10, f.x), mix(v01, v11, f.x), f.y);
	return second ? v.ba : v.rg;
}

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	if (pixel.x >= int(params.size.x) || pixel.y >= int(params.size.y)) {
		return;
	}
	vec4 color = imageLoad(color_image, pixel);
	vec3 x = clamp(color.rgb, vec3(0.0), vec3(@MAXL@));
	if (any(isnan(x))) {
		return;
	}
	float y = dot(LUMINANCE, x);
	if (!(y > 0.0)) {
		return;
	}
	int count = int(params.shape.w);
	float s = clamp((log2(y) - params.shape.y) / params.shape.z, 0.0, float(count - 1));
	int band = min(int(floor(s)), count - 2);
	float t = s - float(band);
	int lw = int(params.size.w);
	ivec2 last = ivec2(lw - 1, int(params.shape.x) - 1);
	vec2 tc = (vec2(pixel) + 0.5) / params.size.z - 0.5;
	ivec2 base = ivec2(floor(tc));
	vec2 f = tc - vec2(base);
	float loc = 0.0;
	float share = 0.0;
	for (int m = 0; m < 2; m++) {
		int k = band + m;
		float weight = m == 0 ? 1.0 - t : t;
		vec2 v = band_value(k, base, f, last, lw);
		float centre = exp2(params.shape.y + float(k) * params.shape.z);
		loc += weight * (v.y > @EMPTY@ ? centre * v.x / v.y : centre);
		share += weight * v.y;
	}
	// LocalAdaptation.smooth_share, then the factor of the pooled local luminance
	float blend = clamp((share / params.extra.y - 1.0) / (params.extra.z - 1.0), 0.0, 1.0);
	blend = blend * blend * (3.0 - 2.0 * blend);
	if (blend <= 0.0) {
		return;
	}
	float local_cd_m2 = loc / params.factor.x;
	float g = clamp(params.factor.z / pow(10.0, @WEBER_OFFSET@ + cone_log_threshold(local_cd_m2 * params.factor.w)), 1.0,
			params.extra.x);
	float c = exp2(blend * log2(min(1.0, g / params.factor.y)));
	// LocalAdaptation.apply_pixel: the cones' part x c, the rods' part x c^2 (saturated where the cones adapt locally)
	vec3 cones = mat3(params.cone_x.rgb, params.cone_y.rgb, params.cone_z.rgb) * x;
	imageStore(color_image, pixel, vec4(clamp(c * (cones + c * (x - cones)), vec3(0.0), vec3(@MAXL@)), color.a));
}
"""

## A GLSL float literal of a constant (always with a decimal point, full precision).
static func glsl_float(value: float) -> String:
	var text := String.num(value, 9)
	if not text.contains(".") and not text.contains("e"):
		text += ".0"
	return text


## The shaders' source with every number taken from EyeAdaptation / LocalAdaptation (one source of truth: no hand-copied
## constants; local_adaptation_test checks that none is left over, local_adaptation_gpu_check.gd compares the GPU's result
## with the CPU mirror).
static func _fill(template: String) -> String:
	var weights := LocalAdaptation.LUMINANCE_WEIGHTS
	var values := {
		"LW_R": weights.x, "LW_G": weights.y, "LW_B": weights.z,
		"MAXL": LocalAdaptation.MAX_SCENE_LUMINANCE, "EMPTY": LocalAdaptation.EMPTY_WEIGHT,
		"LUM_FLOOR": EyeAdaptation.LUMINANCE_FLOOR_CD_M2,
		"CONE_LOW": EyeAdaptation.CONE_LOG_LOW, "CONE_FLOOR": EyeAdaptation.CONE_LOG_FLOOR,
		"CONE_WEBER": EyeAdaptation.CONE_LOG_WEBER, "WEBER_OFFSET": EyeAdaptation.CONE_WEBER_OFFSET,
		"MID_SLOPE": EyeAdaptation.CONE_MID_SLOPE, "MID_OFFSET": EyeAdaptation.CONE_MID_OFFSET,
		"MID_EXPONENT": EyeAdaptation.CONE_MID_EXPONENT,
	}
	for key: String in values:
		template = template.replace("@" + key + "@", glsl_float(values[key]))
	return template.replace("@BANDS@", str(LocalAdaptation.band_count()))


static func down_shader_source() -> String:
	return _fill(DOWN_TEMPLATE)


static func blur_shader_source() -> String:
	return _fill(BLUR_TEMPLATE)


static func apply_shader_source() -> String:
	return _fill(APPLY_TEMPLATE)


## The blur radius (texels) and sigma for a sigma in blocks.
static func blur_radius(sigma_blocks: float) -> int:
	return int(ceil(LocalAdaptation.KERNEL_RADIUS_SIGMAS * sigma_blocks))


## Push constants of the dispatches (the layouts of the shaders above); shared with local_adaptation_gpu_check.gd.
static func down_push(size: Vector2i, block: int, clouds: bool, low: Vector2i) -> PackedFloat32Array:
	return PackedFloat32Array([size.x, size.y, block, 1.0 if clouds else 0.0,
			low.x, low.y, LocalAdaptation.sample_step(block), 0.0,
			LocalAdaptation.band_lowest_log2(), LocalAdaptation.band_step_log2(),
			LocalAdaptation.range_inverse_two_sigma_squared(),
			LocalAdaptation.range_inverse_two_sigma_squared(LocalAdaptation.BRIGHT_NEIGHBOUR_SIGMA_DECADES)])


static func blur_push(low: Vector2i, sigma_blocks: float, horizontal: bool) -> PackedFloat32Array:
	return PackedFloat32Array([low.x, low.y, blur_radius(sigma_blocks), 1.0 / (2.0 * sigma_blocks * sigma_blocks),
			1.0 if horizontal else 0.0, 0.0 if horizontal else 1.0, LocalAdaptation.band_count() / 2, 0.0])


static func apply_push(size: Vector2i, block: int, low: Vector2i, scene_per_cd_m2: float, display_gain: float,
		reference_cd_m2: float, cone_part: Basis) -> PackedFloat32Array:
	return PackedFloat32Array([size.x, size.y, block, low.x,
			low.y, LocalAdaptation.band_lowest_log2(), LocalAdaptation.band_step_log2(), LocalAdaptation.band_count(),
			scene_per_cd_m2, display_gain, reference_cd_m2,
			LocalAdaptation.effective_per_local(scene_per_cd_m2, display_gain, reference_cd_m2),
			EyeAdaptation.GAIN_MAX, LocalAdaptation.MIN_WEIGHT_SHARE, LocalAdaptation.FULL_WEIGHT_SHARE_RATIO, 0.0,
			cone_part.x.x, cone_part.x.y, cone_part.x.z, 0.0,
			cone_part.y.x, cone_part.y.y, cone_part.y.z, 0.0,
			cone_part.z.x, cone_part.z.y, cone_part.z.z, 0.0])


var stage := Stage.MAP
## APPLY: the MAP instance whose per-view maps it reads.
var source: LocalAdaptationEffect
## MAP: the cloud pass whose composite enters the blocks (as the field meter's).
var cloud_effect: CloudEffect

var _rd: RenderingDevice
var _shaders := []
var _pipelines := []
var _no_cloud := RID()
## Render-thread state (set through update).
var _active := false
var _spc := 0.0
var _display_gain := 0.0
var _reference_cd_m2 := 0.0
var _cone_part := Basis.IDENTITY
## MAP results, render thread: per view {"atlas": RID, "tmp": RID, "size": Vector2i, "low": Vector2i}, and the frame's
## block edge (px). The APPLY stage of the same frame reads them (same thread, after the transparent pass).
var _views := []
var _block := 1
var _map_valid := false


func _init(effect_stage := Stage.MAP, map_source: LocalAdaptationEffect = null) -> void:
	stage = effect_stage
	source = map_source
	effect_callback_type = EFFECT_CALLBACK_TYPE_PRE_TRANSPARENT if stage == Stage.MAP \
			else EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
	access_resolved_color = true
	RenderingServer.call_on_render_thread(_create_pipelines)


## Main thread, every frame (MAP instance; the APPLY instance follows its source): the frame's scene value per cd/m^2,
## display gain (cone gain x brightness) and the white plate under the TOA Sun (cd/m^2). Off (no work, no resolve) while
## the eye adaptation is off or LocalAdaptation.enabled is false. rod_scale and cone: the post step's (VisionEffect).
func update(scene_per_cd_m2: float, display_gain: float, reference_cd_m2: float, adaptation_on: bool,
		rod_scale := 0.0, cone := Basis.IDENTITY) -> void:
	var active := enabled and LocalAdaptation.enabled and adaptation_on and scene_per_cd_m2 > 0.0 \
			and display_gain > 1.0 and reference_cd_m2 > 0.0
	if access_resolved_color != active:
		access_resolved_color = active
	RenderingServer.call_on_render_thread(_set_on_render_thread.bind(active, scene_per_cd_m2, display_gain,
			reference_cd_m2, LocalAdaptation.cone_part(rod_scale, cone)))


func _set_on_render_thread(active: bool, spc: float, display_gain: float, reference_cd_m2: float,
		cone_part: Basis) -> void:
	_active = active
	_cone_part = cone_part
	_spc = spc
	_display_gain = display_gain
	_reference_cd_m2 = reference_cd_m2


func _create_pipelines() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	var sources := [down_shader_source(), blur_shader_source()] if stage == Stage.MAP else [apply_shader_source()]
	for code in sources:
		var shader_source := RDShaderSource.new()
		shader_source.language = RenderingDevice.SHADER_LANGUAGE_GLSL
		shader_source.source_compute = code
		var spirv := _rd.shader_compile_spirv_from_source(shader_source)
		if spirv.compile_error_compute != "":
			push_error("LocalAdaptationEffect: " + spirv.compile_error_compute)
			_rd = null
			return
		var shader := _rd.shader_create_from_spirv(spirv)
		_shaders.append(shader)
		_pipelines.append(_rd.compute_pipeline_create(shader))
	if stage == Stage.MAP:
		var format := RDTextureFormat.new()
		format.format = RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT
		format.width = 1
		format.height = 1
		format.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT
		_no_cloud = _rd.texture_create(format, RDTextureView.new(), [])


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd != null:
		for view: Dictionary in _views:  # inline: no method calls at PREDELETE
			for key in ["atlas", "tmp"]:
				if view.has(key) and (view[key] as RID).is_valid():
					_rd.free_rid(view[key])
		if _no_cloud.is_valid():
			_rd.free_rid(_no_cloud)
		for shader in _shaders:
			if (shader as RID).is_valid():
				_rd.free_rid(shader)  # frees the pipeline with it


func _free_view(view: Dictionary) -> void:
	for key in ["atlas", "tmp"]:
		if view.has(key) and (view[key] as RID).is_valid():
			_rd.free_rid(view[key])


func _render_callback(_callback_type: int, render_data: RenderData) -> void:
	if stage == Stage.MAP:
		_map(render_data)
	else:
		_apply(render_data)


func _view_textures(index: int, size: Vector2i, low: Vector2i) -> Dictionary:
	while _views.size() <= index:
		_views.append({})
	var view: Dictionary = _views[index]
	if view.get("low", Vector2i.ZERO) == low and view.get("size", Vector2i.ZERO) == size:
		return view
	_free_view(view)
	var format := RDTextureFormat.new()
	format.width = low.x * (LocalAdaptation.band_count() / 2)  # the bands' atlas: tile k = bands 2k, 2k+1
	format.height = low.y
	format.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT
	format.format = RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT
	view = {"size": size, "low": low, "atlas": _rd.texture_create(format, RDTextureView.new(), []),
			"tmp": _rd.texture_create(format, RDTextureView.new(), [])}
	_views[index] = view
	return view


func _image_uniform(binding: int, texture: RID) -> RDUniform:
	var uniform := RDUniform.new()
	uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	uniform.binding = binding
	uniform.add_id(texture)
	return uniform


func _dispatch(pipeline_index: int, uniforms: Array, push: PackedFloat32Array, groups: Vector2i) -> void:
	var bytes := push.to_byte_array()
	var uniform_set := UniformSetCacheRD.get_cache(_shaders[pipeline_index], 0, uniforms)
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _pipelines[pipeline_index])
	_rd.compute_list_bind_uniform_set(list, uniform_set, 0)
	_rd.compute_list_set_push_constant(list, bytes, bytes.size())
	_rd.compute_list_dispatch(list, groups.x, groups.y, 1)
	_rd.compute_list_end()


func _map(render_data: RenderData) -> void:
	_map_valid = false
	if _rd == null or _pipelines.size() < 2 or not _active:
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	if size.x == 0 or size.y == 0:
		return
	var projection := render_data.get_render_scene_data().get_view_projection(0)
	var ppd := LocalAdaptation.pixels_per_degree(float(size.y), absf(projection.y.y))
	_block = LocalAdaptation.block_size(ppd)
	var sigma_blocks := LocalAdaptation.POOL_SIGMA_DEG * ppd / float(_block)
	var low := Vector2i((size.x - 1) / _block + 1, (size.y - 1) / _block + 1)
	var tiles := LocalAdaptation.band_count() / 2
	var groups := Vector2i((low.x - 1) / 8 + 1, (low.y - 1) / 8 + 1)
	var wide := Vector2i((low.x * tiles - 1) / 8 + 1, (low.y - 1) / 8 + 1)
	if CloudEffect.capture_timestamps:  # render_check --gpu_passes
		_rd.capture_timestamp("local_adaptation_map")
	for index in range(buffers.get_view_count()):
		var view := _view_textures(index, size, low)
		var cloud := cloud_effect.frame_output_layer(index) if cloud_effect != null else RID()
		_dispatch(0, [_image_uniform(0, buffers.get_color_layer(index)),
				_image_uniform(1, cloud if cloud.is_valid() else _no_cloud), _image_uniform(2, view["atlas"])],
				down_push(size, _block, cloud.is_valid(), low), groups)
		_dispatch(1, [_image_uniform(0, view["atlas"]), _image_uniform(1, view["tmp"])],
				blur_push(low, sigma_blocks, true), wide)
		_dispatch(1, [_image_uniform(0, view["tmp"]), _image_uniform(1, view["atlas"])],
				blur_push(low, sigma_blocks, false), wide)
	_map_valid = true
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp("after_local_adaptation_map")


func _apply(render_data: RenderData) -> void:
	if _rd == null or _pipelines.is_empty() or source == null or not source._map_valid:
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	var groups := Vector2i((size.x - 1) / 8 + 1, (size.y - 1) / 8 + 1)
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp("local_adaptation_apply")
	for index in range(mini(buffers.get_view_count(), source._views.size())):
		var view: Dictionary = source._views[index]
		if view.get("size", Vector2i.ZERO) != size:
			continue
		var low: Vector2i = view["low"]
		_dispatch(0, [_image_uniform(0, buffers.get_color_layer(index)), _image_uniform(1, view["atlas"])],
				apply_push(size, source._block, low, source._spc, source._display_gain, source._reference_cd_m2,
				source._cone_part), groups)
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp("after_local_adaptation_apply")
