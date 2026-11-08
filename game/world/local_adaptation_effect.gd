class_name LocalAdaptationEffect
extends CompositorEffect
## Local adaptation on the GPU (look fixes T10; the model and its CPU mirror: LocalAdaptation). Two instances of this
## effect: the MAP stage before the transparent pass (after the field meter: the meter measures the unchanged image)
## averages the opaque image with the cloud composite (as the meter does) into blocks of mean log2 luminance and pools
## them bilaterally into a factor log2 c per block; the APPLY stage after the transparent pass (after VisionEffect, before
## the display tone step, glow and AgX) multiplies every pixel by c, joint-bilaterally upsampled against the pixel's own
## luminance. The map is built from the opaque image only, so what the transparent pass draws over a dark field (the
## display-referred emitters: plume, stars, labels, the VR panel; lamp lenses) is unlike every block there and keeps
## c = 1; over a locally adapted region (a label on the Earth's disc seen from the lunar night) it takes that region's
## factor. Per view (both VR eyes, each its own map). Without a RenderingDevice (headless, Compatibility), with the eye
## adaptation off or LocalAdaptation.enabled false it does nothing. The MAP stage needs the resolved colour every frame
## (MSAA: an extra resolve before the transparent pass).

enum Stage { MAP, APPLY }

const DOWN_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D color_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict readonly image2D cloud_image;
layout(r32f, set = 0, binding = 2) uniform restrict writeonly image2D block_image;
layout(push_constant, std430) uniform Params {
	vec4 size;  // width, height, block, clouds (1 / 0)
	vec4 low;   // block columns, block rows, log2 floor, sample step (px; LocalAdaptation.sample_step)
} params;
const vec3 LUMINANCE = vec3(@LW_R@, @LW_G@, @LW_B@);

void main() {
	ivec2 t = ivec2(gl_GlobalInvocationID.xy);
	if (t.x >= int(params.low.x) || t.y >= int(params.low.y)) {
		return;
	}
	int b = int(params.size.z);
	int step = int(params.low.w);
	float floor_log = params.low.z;
	float sum = 0.0;
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
			sum += dot(LUMINANCE, clamp(c, vec3(0.0), vec3(@MAXL@)));  // LINEAR block mean (area-exact)
			n += 1.0;
		}
	}
	float mean = n > 0.0 ? sum / n : 0.0;
	imageStore(block_image, t, vec4(mean > 0.0 ? max(log2(mean), floor_log) : floor_log));
}
"""

const POOL_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(r32f, set = 0, binding = 0) uniform restrict readonly image2D block_image;
layout(rg32f, set = 0, binding = 1) uniform restrict writeonly image2D map_image;
layout(push_constant, std430) uniform Params {
	vec4 low;     // block columns, block rows, radius (blocks), 1 / (2 sigma^2) (blocks)
	vec4 range;   // 1 / (2 sigma_r^2) (log2), log2 floor, scene per cd/m^2, display gain
	vec4 gain;    // L_ref (cd/m^2), L_eff per L_loc, GAIN_MAX, -
} params;

// EyeAdaptation.cone_log_threshold / cone_gain [S: Ferwerda et al. 1996 Eq. 4]
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

void main() {
	ivec2 t = ivec2(gl_GlobalInvocationID.xy);
	ivec2 low = ivec2(params.low.xy);
	if (t.x >= low.x || t.y >= low.y) {
		return;
	}
	float c0 = imageLoad(block_image, t).r;
	int radius = int(params.low.z);
	float sw = 0.0;
	float s = 0.0;
	for (int dy = -radius; dy <= radius; dy++) {
		for (int dx = -radius; dx <= radius; dx++) {
			ivec2 q = t + ivec2(dx, dy);
			if (q.x < 0 || q.y < 0 || q.x >= low.x || q.y >= low.y) {
				continue;
			}
			float v = imageLoad(block_image, q).r;
			float w = exp(-float(dx * dx + dy * dy) * params.low.w - (v - c0) * (v - c0) * params.range.x);
			sw += w;
			s += w * exp2(v);  // pooled in linear luminance
		}
	}
	float pooled = log2(s / sw);
	float log_c = 0.0;
	if (pooled > params.range.y + 0.5 && params.range.w > 0.0) {
		float local_cd_m2 = exp2(pooled) / params.range.z;
		float g = clamp(params.gain.x / pow(10.0, @WEBER_OFFSET@ + cone_log_threshold(local_cd_m2 * params.gain.y)), 1.0,
				params.gain.z);
		log_c = log2(min(1.0, g / params.range.w));
	}
	imageStore(map_image, t, vec4(c0, log_c, 0.0, 0.0));
}
"""

const APPLY_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform image2D color_image;
layout(rg32f, set = 0, binding = 1) uniform restrict readonly image2D map_image;
layout(push_constant, std430) uniform Params {
	vec4 size;   // width, height, block, block columns
	vec4 shape;  // block rows, 1 / (2 sigma_up^2), 1 / (2 sigma_r^2) (log2), log2 floor
	vec4 extra;  // MIN_WEIGHT_SHARE, 1 / (2 sigma_dim^2) (log2; pixel dimmer than the block), FULL_WEIGHT_SHARE_RATIO, -
	vec4 cone_x;  // N = C M^-1: the cones' part of the post step's pixel (LocalAdaptation.cone_part), its columns
	vec4 cone_y;
	vec4 cone_z;
} params;
const vec3 LUMINANCE = vec3(@LW_R@, @LW_G@, @LW_B@);

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	if (pixel.x >= int(params.size.x) || pixel.y >= int(params.size.y)) {
		return;
	}
	float block = params.size.z;
	vec2 tc = (vec2(pixel) + 0.5) / block - 0.5;
	ivec2 base = ivec2(floor(tc));
	ivec2 last = ivec2(int(params.size.w) - 1, int(params.shape.x) - 1);
	// The map first (small, cached): no compression near this pixel - its colour is not even read (T10 cost).
	vec2 m[16];
	float lowest = 0.0;
	for (int k = 0; k < 16; k++) {
		m[k] = imageLoad(map_image, clamp(base + ivec2(k % 4 - 1, k / 4 - 1), ivec2(0), last)).rg;
		lowest = min(lowest, m[k].y);
	}
	if (lowest >= 0.0) {
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
	float l = max(log2(y), params.shape.w);
	float ss = 0.0;
	float sw = 0.0;
	float s = 0.0;
	for (int k = 0; k < 16; k++) {
		vec2 d = vec2(base + ivec2(k % 4 - 1, k / 4 - 1)) - tc;
		float spatial = exp(-dot(d, d) * params.shape.y);
		float w = spatial * exp(-(m[k].x - l) * (m[k].x - l) * (m[k].x > l ? params.extra.y : params.shape.z));
		ss += spatial;
		sw += w;
		s += w * m[k].y;
	}
	if (sw < params.extra.x * ss) {
		return;  // no compression near this pixel, or no block like it (an emitter, a sub-block glint): c = 1
	}
	// LocalAdaptation.apply_pixel: the cones' part x c, the rods' part x c^2 (saturated where the cones adapt locally)
	float share = clamp((sw / ss / params.extra.x - 1.0) / (params.extra.z - 1.0), 0.0, 1.0);
	float c = exp2(share * share * (3.0 - 2.0 * share) * s / sw);  // LocalAdaptation.smooth_share
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
		"MAXL": LocalAdaptation.MAX_SCENE_LUMINANCE,
		"LUM_FLOOR": EyeAdaptation.LUMINANCE_FLOOR_CD_M2,
		"CONE_LOW": EyeAdaptation.CONE_LOG_LOW, "CONE_FLOOR": EyeAdaptation.CONE_LOG_FLOOR,
		"CONE_WEBER": EyeAdaptation.CONE_LOG_WEBER, "WEBER_OFFSET": EyeAdaptation.CONE_WEBER_OFFSET,
		"MID_SLOPE": EyeAdaptation.CONE_MID_SLOPE, "MID_OFFSET": EyeAdaptation.CONE_MID_OFFSET,
		"MID_EXPONENT": EyeAdaptation.CONE_MID_EXPONENT,
	}
	for key: String in values:
		template = template.replace("@" + key + "@", glsl_float(values[key]))
	return template


static func down_shader_source() -> String:
	return _fill(DOWN_TEMPLATE)


static func pool_shader_source() -> String:
	return _fill(POOL_TEMPLATE)


static func apply_shader_source() -> String:
	return _fill(APPLY_TEMPLATE)


## Push constants of the three dispatches (the layouts of the shaders above); shared with local_adaptation_gpu_check.gd.
static func down_push(size: Vector2i, block: int, clouds: bool, low: Vector2i, floor_log: float) -> PackedFloat32Array:
	return PackedFloat32Array([size.x, size.y, block, 1.0 if clouds else 0.0,
			low.x, low.y, floor_log, LocalAdaptation.sample_step(block)])


static func pool_push(low: Vector2i, radius: int, sigma_blocks: float, floor_log: float, scene_per_cd_m2: float,
		display_gain: float, reference_cd_m2: float, k_eff: float) -> PackedFloat32Array:
	return PackedFloat32Array([low.x, low.y, radius, 1.0 / (2.0 * sigma_blocks * sigma_blocks),
			LocalAdaptation.range_inverse_two_sigma_squared(), floor_log, scene_per_cd_m2, display_gain,
			reference_cd_m2, k_eff, EyeAdaptation.GAIN_MAX, 0.0])


static func apply_push(size: Vector2i, block: int, low: Vector2i, floor_log: float, cone_part: Basis) -> PackedFloat32Array:
	return PackedFloat32Array([size.x, size.y, block, low.x,
			low.y, 1.0 / (2.0 * LocalAdaptation.UPSAMPLE_SIGMA_BLOCKS * LocalAdaptation.UPSAMPLE_SIGMA_BLOCKS),
			LocalAdaptation.range_inverse_two_sigma_squared(), floor_log,
			LocalAdaptation.MIN_WEIGHT_SHARE,
			LocalAdaptation.range_inverse_two_sigma_squared(LocalAdaptation.UPSAMPLE_DIM_SIGMA_DECADES),
			LocalAdaptation.FULL_WEIGHT_SHARE_RATIO, 0.0,
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
## MAP results, render thread: per view {"block": RID, "map": RID, "size": Vector2i, "low": Vector2i}, and the frame's
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
	var sources := [down_shader_source(), pool_shader_source()] if stage == Stage.MAP else [apply_shader_source()]
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
			for key in ["block", "map"]:
				if view.has(key) and (view[key] as RID).is_valid():
					_rd.free_rid(view[key])
		if _no_cloud.is_valid():
			_rd.free_rid(_no_cloud)
		for shader in _shaders:
			if (shader as RID).is_valid():
				_rd.free_rid(shader)  # frees the pipeline with it


func _free_view(view: Dictionary) -> void:
	for key in ["block", "map"]:
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
	format.width = low.x
	format.height = low.y
	format.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT
	format.format = RenderingDevice.DATA_FORMAT_R32_SFLOAT
	view = {"size": size, "low": low, "block": _rd.texture_create(format, RDTextureView.new(), [])}
	format.format = RenderingDevice.DATA_FORMAT_R32G32_SFLOAT
	view["map"] = _rd.texture_create(format, RDTextureView.new(), [])
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
	_block = LocalAdaptation.block_size(LocalAdaptation.pixels_per_degree(float(size.y), absf(projection.y.y)))
	var sigma_blocks := LocalAdaptation.POOL_SIGMA_DEG * LocalAdaptation.pixels_per_degree(float(size.y),
			absf(projection.y.y)) / float(_block)
	var radius := int(ceil(LocalAdaptation.KERNEL_RADIUS_SIGMAS * sigma_blocks))
	var low := Vector2i((size.x - 1) / _block + 1, (size.y - 1) / _block + 1)
	var floor_log := LocalAdaptation.log2_floor(_spc)
	var k_eff := LocalAdaptation.effective_per_local(_spc, _display_gain, _reference_cd_m2)
	var groups := Vector2i((low.x - 1) / 8 + 1, (low.y - 1) / 8 + 1)
	if CloudEffect.capture_timestamps:  # render_check --gpu_passes
		_rd.capture_timestamp("local_adaptation_map")
	for index in range(buffers.get_view_count()):
		var view := _view_textures(index, size, low)
		var cloud := cloud_effect.frame_output_layer(index) if cloud_effect != null else RID()
		_dispatch(0, [_image_uniform(0, buffers.get_color_layer(index)),
				_image_uniform(1, cloud if cloud.is_valid() else _no_cloud), _image_uniform(2, view["block"])],
				down_push(size, _block, cloud.is_valid(), low, floor_log), groups)
		_dispatch(1, [_image_uniform(0, view["block"]), _image_uniform(1, view["map"])],
				pool_push(low, radius, sigma_blocks, floor_log, _spc, _display_gain, _reference_cd_m2, k_eff), groups)
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
	var floor_log := LocalAdaptation.log2_floor(source._spc)
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp("local_adaptation_apply")
	for index in range(mini(buffers.get_view_count(), source._views.size())):
		var view: Dictionary = source._views[index]
		if view.get("size", Vector2i.ZERO) != size:
			continue
		var low: Vector2i = view["low"]
		_dispatch(0, [_image_uniform(0, buffers.get_color_layer(index)), _image_uniform(1, view["map"])],
				apply_push(size, source._block, low, floor_log, source._cone_part), groups)
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp("after_local_adaptation_apply")
