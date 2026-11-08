class_name GlareSpreadEffect
extends CompositorEffect
## The physical glare spread on the GPU (look fixes T13; the model and its CPU mirror: GlareSpread, the sources and the
## pyramid are described there). Two instances, as the local adaptation's: the SPREAD stage after the transparent pass,
## before VisionEffect, reads the HDR frame as the shaders drew it (the composited clouds, lamp lenses, the plume; before the
## rods and before the local adaptation's compression: the sources' true scene values) and builds the veil pyramid; the ADD
## stage after the local adaptation's APPLY (before the display tone step) adds the veil through the rods' and cones' step of
## VisionEffect (the veil lies over dark-adapted retina: the local adaptation, which compresses what is bright in the image,
## must not dim it - added before it, the veil next to the Earth's disc was dimmed to a dark ring). Per view (both VR eyes):
##   SOURCE  the full-resolution frame -> level 0: per pixel the excess max(0, Y - SOURCE_WHITE) with the colour kept,
##           the Sun's disc masked out, x dOmega_px / dOmega_axis, tent-filtered into texels of f0 pixels;
##   DOWN    level k -> k + 1 with the tent [1, 3, 3, 1] / 8 (energy and centroid kept);
##   BLUR    every level, separable Gaussian of the same sigma in texels (a horizontal pass into a shared scratch image,
##           a vertical pass that also adds the weight w_k and the up-sampled coarser result: A_k = w_k blur(M_k) + up(A_k+1),
##           in place of M_k);
##   ADD     colour += M (bilinear(A_0) + the Sun's analytic veil of the pixel (GlareSpread.sun_veil_cd_m2 in scene units)),
##           M = VisionEffect's per-pixel map (cone map + rod_scale x tint x scotopic weights).
## The A_0 image stays until the next frame's SOURCE pass: the field meter (before the transparent pass) reads it one frame
## late (AdaptationMeter.glare_effect) so the Earth's / Moon's veil raises the adaptation. The Sun's part is not in it (the
## meter adds the scalar Sun veil itself). Without a RenderingDevice (headless, Compatibility), with GlareSpread.enabled
## false or no frame set it does nothing.

const SOURCE_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D color_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict writeonly image2D level_image;
layout(push_constant, std430) uniform Params {
	vec4 size;  // width, height, f0, white (scene units)
	vec4 low;   // level-0 columns, rows, -, -
	vec4 ray;   // view ray of NDC (x, y): (x a + b, y c + d, -1) - a, b, c, d
	vec4 sun;   // the Sun's direction in the eye's view axes, cos of the mask radius (> 1: no mask)
} params;
const vec3 LUMINANCE = vec3(@LW_R@, @LW_G@, @LW_B@);

void main() {
	ivec2 t = ivec2(gl_GlobalInvocationID.xy);
	if (t.x >= int(params.low.x) || t.y >= int(params.low.y)) {
		return;
	}
	float f = params.size.z;
	vec2 centre = (vec2(t) + 0.5) * f;
	vec3 sum = vec3(0.0);
	for (int y = int(ceil(centre.y - f - 0.5)); y <= int(floor(centre.y + f - 0.5)); y++) {
		float wy = 1.0 - abs(float(y) + 0.5 - centre.y) / f;
		if (y < 0 || y >= int(params.size.y) || wy <= 0.0) {
			continue;
		}
		for (int x = int(ceil(centre.x - f - 0.5)); x <= int(floor(centre.x + f - 0.5)); x++) {
			float wx = 1.0 - abs(float(x) + 0.5 - centre.x) / f;
			if (x < 0 || x >= int(params.size.x) || wx <= 0.0) {
				continue;
			}
			vec3 c = imageLoad(color_image, ivec2(x, y)).rgb;
			if (any(isnan(c))) {
				continue;
			}
			c = clamp(c, vec3(0.0), vec3(@MAXL@));
			float luminance = dot(LUMINANCE, c);
			if (luminance <= params.size.w) {
				continue;
			}
			vec2 ndc = vec2(2.0 * (float(x) + 0.5) / params.size.x - 1.0, 1.0 - 2.0 * (float(y) + 0.5) / params.size.y);
			vec3 ray = vec3(ndc.x * params.ray.x + params.ray.y, ndc.y * params.ray.z + params.ray.w, -1.0);
			float r2 = dot(ray, ray);
			if (dot(ray, params.sun.xyz) > params.sun.w * sqrt(r2)) {
				continue;  // the Sun's disc (clamped in fp16): its veil is analytic
			}
			sum += wx * wy * c * ((luminance - params.size.w) / luminance) * pow(r2, -1.5);
		}
	}
	imageStore(level_image, t, vec4(sum / (f * f), 0.0));
}
"""

const DOWN_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D in_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict writeonly image2D out_image;
layout(push_constant, std430) uniform Params {
	vec4 size;  // in width, in height, out width, out height
} params;

// GlareSpread.down_weights: the tent [1, 3, 3, 1] / 8; the share that would land on the texel beyond the edge is folded back.
float tent(int t, int x, int in_count, int out_count) {
	int d = x - (2 * t - 1);
	float w = (d == 0 || d == 3) ? 1.0 : 3.0;
	if (d < 0 || d > 3) {
		w = 0.0;
	}
	if (t == 0 && x == 0) {
		w += 1.0;
	}
	if (t == out_count - 1 && in_count == 2 * out_count && x == in_count - 1) {
		w += 1.0;
	}
	return w / 8.0;
}

void main() {
	ivec2 t = ivec2(gl_GlobalInvocationID.xy);
	ivec2 in_count = ivec2(int(params.size.x), int(params.size.y));
	ivec2 out_count = ivec2(int(params.size.z), int(params.size.w));
	if (t.x >= out_count.x || t.y >= out_count.y) {
		return;
	}
	vec3 sum = vec3(0.0);
	for (int y = max(0, 2 * t.y - 1); y < min(in_count.y, 2 * t.y + 3); y++) {
		float wy = tent(t.y, y, in_count.y, out_count.y);
		for (int x = max(0, 2 * t.x - 1); x < min(in_count.x, 2 * t.x + 3); x++) {
			sum += wy * tent(t.x, x, in_count.x, out_count.x) * imageLoad(in_image, ivec2(x, y)).rgb;
		}
	}
	imageStore(out_image, t, vec4(sum, 0.0));
}
"""

## Horizontal Gaussian blur of a level into the scratch image.
const BLUR_H_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D in_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict writeonly image2D out_image;
layout(push_constant, std430) uniform Params {
	vec4 size;  // width, height, radius (texels), 1 / (2 sigma^2) (texels)
} params;

void main() {
	ivec2 t = ivec2(gl_GlobalInvocationID.xy);
	if (t.x >= int(params.size.x) || t.y >= int(params.size.y)) {
		return;
	}
	int radius = int(params.size.z);
	vec3 sum = vec3(0.0);
	float total = 0.0;
	for (int d = -radius; d <= radius; d++) {
		float w = exp(-float(d * d) * params.size.w);
		total += w;
		int x = t.x + d;
		if (x >= 0 && x < int(params.size.x)) {
			sum += w * imageLoad(in_image, ivec2(x, t.y)).rgb;
		}
	}
	imageStore(out_image, t, vec4(sum / total, 0.0));
}
"""

## Vertical blur of the scratch image, times the level's weight, plus the up-sampled coarser result; written over the level.
const BLUR_V_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform restrict readonly image2D scratch_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict readonly image2D coarse_image;
layout(rgba16f, set = 0, binding = 2) uniform restrict writeonly image2D level_image;
layout(push_constant, std430) uniform Params {
	vec4 size;    // width, height, radius (texels), 1 / (2 sigma^2) (texels)
	vec4 coarse;  // coarse width, coarse height, weight w_k, coarser level present (1 / 0)
} params;

vec3 coarse_at(ivec2 p) {
	p = clamp(p, ivec2(0), ivec2(int(params.coarse.x) - 1, int(params.coarse.y) - 1));
	return imageLoad(coarse_image, p).rgb;
}

void main() {
	ivec2 t = ivec2(gl_GlobalInvocationID.xy);
	if (t.x >= int(params.size.x) || t.y >= int(params.size.y)) {
		return;
	}
	int radius = int(params.size.z);
	vec3 sum = vec3(0.0);
	float total = 0.0;
	for (int d = -radius; d <= radius; d++) {
		float w = exp(-float(d * d) * params.size.w);
		total += w;
		int y = t.y + d;
		if (y >= 0 && y < int(params.size.y)) {
			sum += w * imageLoad(scratch_image, ivec2(t.x, y)).rgb;
		}
	}
	vec3 result = params.coarse.z * sum / total;
	if (params.coarse.w > 0.5) {
		vec2 u = (vec2(t) - 0.5) * 0.5;  // the coarser level's texel index of this texel's centre
		ivec2 i0 = ivec2(floor(u));
		vec2 f = u - vec2(i0);
		result += (1.0 - f.y) * ((1.0 - f.x) * coarse_at(i0) + f.x * coarse_at(i0 + ivec2(1, 0)))
				+ f.y * ((1.0 - f.x) * coarse_at(i0 + ivec2(0, 1)) + f.x * coarse_at(i0 + ivec2(1, 1)));
	}
	imageStore(level_image, t, vec4(result, 0.0));
}
"""

const APPLY_TEMPLATE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform image2D color_image;
layout(rgba16f, set = 0, binding = 1) uniform restrict readonly image2D veil_image;
layout(rgba16f, set = 0, binding = 2) uniform restrict readonly image2D source_image;
layout(push_constant, std430) uniform Params {
	vec4 size;     // width, height, f0, flags (1: the image part, 2: the local adaptation's compression of the veil on a source)
	vec4 low;      // level-0 columns, rows, plateau (scene units), source white (scene units)
	vec4 ray;      // a, b, c, d (see SOURCE)
	vec4 sun;      // the Sun's direction in the eye's view axes, E_gl in scene units (lux x visibility x scene per cd/m^2)
	vec4 sun_rgb;  // the direct Sun's colour (luminance 1)
	vec4 map_x;    // M = C + rod_scale tint scotopic^T (VisionEffect), its columns
	vec4 map_y;
	vec4 map_z;
} params;
const vec3 LUMINANCE = vec3(@LW_R@, @LW_G@, @LW_B@);

// GlareSpread.cie146 [S: CIE 146:2002], theta in degrees
float cie146(float t) {
	return 10.0 / (t * t * t) + (5.0 / (t * t) + @P_TERM@ / t) * @AGE_FACTOR@ + @CONSTANT_TERM@;
}

ivec2 clamp_low(ivec2 p) {
	return clamp(p, ivec2(0), ivec2(int(params.low.x) - 1, int(params.low.y) - 1));
}

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	if (pixel.x >= int(params.size.x) || pixel.y >= int(params.size.y)) {
		return;
	}
	int flags = int(params.size.w);
	vec3 veil = vec3(0.0);
	if ((flags & 1) != 0) {
		vec2 u = (vec2(pixel) + 0.5) / params.size.z - 0.5;
		ivec2 i0 = ivec2(floor(u));
		vec2 f = u - vec2(i0);
		veil = (1.0 - f.y) * ((1.0 - f.x) * imageLoad(veil_image, clamp_low(i0)).rgb
						+ f.x * imageLoad(veil_image, clamp_low(i0 + ivec2(1, 0))).rgb)
				+ f.y * ((1.0 - f.x) * imageLoad(veil_image, clamp_low(i0 + ivec2(0, 1))).rgb
						+ f.x * imageLoad(veil_image, clamp_low(i0 + ivec2(1, 1))).rgb);
		if ((flags & 2) != 0) {
			// A pixel that is itself a source the local adaptation compresses (GlareSpread.self_compression): its veil
			// goes with it; the dark surround keeps the full veil.
			// the largest of the four texels around: a limb pixel (a part of its texel covered) is compressed as the disc
			float excess = max(max(dot(LUMINANCE, imageLoad(source_image, clamp_low(i0)).rgb),
							dot(LUMINANCE, imageLoad(source_image, clamp_low(i0 + ivec2(1, 0))).rgb)),
					max(dot(LUMINANCE, imageLoad(source_image, clamp_low(i0 + ivec2(0, 1))).rgb),
							dot(LUMINANCE, imageLoad(source_image, clamp_low(i0 + ivec2(1, 1))).rgb)));
			veil *= excess > 0.0 ? min(1.0, params.low.z / (params.low.w + excess)) : 1.0;
			// The halo next to a source lies over a retina adapted to it: the local adaptation shows such a region at
			// most at its plateau; the veil saturates softly there (monotonic in the veil, no ring at the limb).
			veil *= params.low.z / (params.low.z + dot(LUMINANCE, veil));
		}
	}
	if (params.sun.w > 0.0) {
		vec2 ndc = vec2(2.0 * (float(pixel.x) + 0.5) / params.size.x - 1.0, 1.0 - 2.0 * (float(pixel.y) + 0.5) / params.size.y);
		vec3 ray = normalize(vec3(ndc.x * params.ray.x + params.ray.y, ndc.y * params.ray.z + params.ray.w, -1.0));
		float cos_theta = dot(ray, params.sun.xyz);
		if (cos_theta > 0.0) {
			float theta = degrees(atan(length(cross(ray, params.sun.xyz)), cos_theta));
			veil += params.sun_rgb.rgb * (params.sun.w * cos_theta * cie146(max(theta, @MIN_ANGLE@)));
		}
	}
	vec4 color = imageLoad(color_image, pixel);
	veil = mat3(params.map_x.rgb, params.map_y.rgb, params.map_z.rgb) * veil;
	imageStore(color_image, pixel, vec4(clamp(color.rgb + veil, vec3(-@MAXL@), vec3(@MAXL@)), color.a));
}
"""

## A GLSL float literal of a constant (full precision, always with a decimal point).
static func glsl_float(value: float) -> String:
	var text := String.num(value, 9)
	if not text.contains(".") and not text.contains("e"):
		text += ".0"
	return text


## The shaders' numbers come from GlareSpread / GlareSpreadKernel / Atmosphere (one source of truth; glare_spread_test checks
## that none is left over, glare_spread_gpu_check.gd compares the GPU with the CPU mirror).
static func _fill(template: String) -> String:
	var weights := GlareSpread.LUMINANCE_WEIGHTS
	var values := {
		"LW_R": weights.x, "LW_G": weights.y, "LW_B": weights.z,
		"MAXL": GlareSpread.MAX_SCENE,
		"P_TERM": 0.1 * GlareSpreadKernel.PIGMENTATION,
		"AGE_FACTOR": GlareSpread.age_factor(),
		"CONSTANT_TERM": 0.0025 * GlareSpreadKernel.PIGMENTATION,
		"MIN_ANGLE": Atmosphere.GLARE_MIN_ANGLE_DEG,
	}
	for key: String in values:
		template = template.replace("@" + key + "@", glsl_float(values[key]))
	return template


static func shader_sources() -> Array:
	return [_fill(SOURCE_TEMPLATE), _fill(DOWN_TEMPLATE), _fill(BLUR_H_TEMPLATE), _fill(BLUR_V_TEMPLATE),
			_fill(APPLY_TEMPLATE)]


enum Kernel { SOURCE, DOWN, BLUR_H, BLUR_V, APPLY }
enum Stage { SPREAD, ADD }

var stage := Stage.SPREAD
## ADD: the SPREAD instance whose veil images and frame state it uses.
var source: GlareSpreadEffect
## The field meter (set by Atmosphere): its pre-transparent pass reads A_0 of the last frame. Held weakly: the meter holds this
## effect (a reference cycle would keep both alive to the engine's exit, with their RIDs).
var meter: AdaptationMeter:
	set(value):
		if value != _meter_ref.get_ref() if _meter_ref != null else value != null:
			_meter_ref = weakref(value) if value != null else null
	get:
		return _meter_ref.get_ref() as AdaptationMeter if _meter_ref != null else null
var _meter_ref: WeakRef
## The rods' and cones' step whose map the veil goes through (set by Atmosphere).
var vision: VisionEffect

var _rd: RenderingDevice
var _local := false
var _local_sets: Array[RID] = []
var _shaders := []
var _pipelines := []
## Render-thread state (set_frame).
var _active := false
var _white := GlareSpread.SOURCE_WHITE
var _sun_world := Vector3.ZERO  ## the Sun's direction in the render world
var _sun_scene_lux := 0.0  ## E_gl source: normal illuminance x visibility x scene per cd/m^2 (0: no Sun veil)
var _sun_rgb := Vector3.ONE
var _mask_cos := 2.0
## Per view: {"size": Vector2i, "factor": int, "levels": Array[RID], "sizes": Array[Vector2i], "scratch": RID}.
var _views := []
## SPREAD -> ADD, render thread: per view index of this frame's spread [ray parameters, the Sun's direction in the eye's
## axes]; cleared at the start of every SPREAD pass.
var _spread_sun := {}
## Whether the local adaptation runs (Atmosphere: eye adaptation on); the veil on a source is compressed with it.
var _local_active := false


func _init(attach := true, effect_stage := Stage.SPREAD, spread_source: GlareSpreadEffect = null) -> void:
	stage = effect_stage
	source = spread_source
	effect_callback_type = EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
	access_resolved_color = true
	if attach and stage == Stage.SPREAD:  # the ADD stage dispatches the SPREAD instance's pipelines
		RenderingServer.call_on_render_thread(_create_pipelines)


## Main thread, every frame: whether the spread runs, the Sun's direction in the render world, its veil's illuminance in
## scene units (0: none) and colour, and the Sun mask's cosine (Atmosphere.update).
func set_frame(active: bool, sun_world: Vector3, sun_scene_lux: float, sun_rgb: Vector3, mask_cos: float,
		local_adaptation_on := false) -> void:
	var on := active and GlareSpread.enabled
	access_resolved_color = on
	if source == null:  # the ADD instance reads the SPREAD instance's state
		RenderingServer.call_on_render_thread(_set_on_render_thread.bind(on, sun_world, sun_scene_lux, sun_rgb, mask_cos,
				local_adaptation_on))


func _set_on_render_thread(active: bool, sun_world: Vector3, sun_scene_lux: float, sun_rgb: Vector3, mask_cos: float,
		local_adaptation_on: bool) -> void:
	_local_active = local_adaptation_on
	_active = active
	_sun_world = sun_world
	_sun_scene_lux = sun_scene_lux
	_sun_rgb = sun_rgb
	_mask_cos = mask_cos


## Render thread (the field meter): A_0 of the last frame's view, [texture RID, factor, columns, rows]; [] when the spread
## did not run.
func veil_image(view: int) -> Array:
	if not _active or view >= _views.size() or _views[view].is_empty():
		return []
	var v: Dictionary = _views[view]
	return [v["a0"], v["factor"], v["sizes"][0].x, v["sizes"][0].y]


## local: a local RenderingDevice (glare_spread_gpu_check.gd): uniform sets are made and freed here (free_local_sets), not
## taken from the main device's UniformSetCacheRD.
func create_pipelines(rd: RenderingDevice, local := false) -> bool:
	_rd = rd
	_local = local
	if _rd == null:
		return false
	for code in shader_sources():
		var shader_source := RDShaderSource.new()
		shader_source.language = RenderingDevice.SHADER_LANGUAGE_GLSL
		shader_source.source_compute = code
		var spirv := _rd.shader_compile_spirv_from_source(shader_source)
		if spirv.compile_error_compute != "":
			push_error("GlareSpreadEffect: " + spirv.compile_error_compute)
			_rd = null
			return false
		var shader := _rd.shader_create_from_spirv(spirv)
		_shaders.append(shader)
		_pipelines.append(_rd.compute_pipeline_create(shader))
	return true


func free_local_sets() -> void:
	for set_rid in _local_sets:
		_rd.free_rid(set_rid)
	_local_sets.clear()


func _create_pipelines() -> void:
	create_pipelines(RenderingServer.get_rendering_device())


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd != null:
		for view: Dictionary in _views:  # inline: no method calls at PREDELETE
			for level: RID in view.get("levels", []):
				if level.is_valid():
					_rd.free_rid(level)
			for key in ["scratch", "a0"]:
				if view.has(key) and (view[key] as RID).is_valid():
					_rd.free_rid(view[key])
		for shader in _shaders:
			if (shader as RID).is_valid():
				_rd.free_rid(shader)


func _free_view(view: Dictionary) -> void:
	for level: RID in view.get("levels", []):
		if level.is_valid():
			_rd.free_rid(level)
	for key in ["scratch", "a0"]:
		if view.has(key) and (view[key] as RID).is_valid():
			_rd.free_rid(view[key])


func _texture(size: Vector2i) -> RID:
	var format := RDTextureFormat.new()
	format.format = RenderingDevice.DATA_FORMAT_R16G16B16A16_SFLOAT
	format.width = size.x
	format.height = size.y
	format.usage_bits = RenderingDevice.TEXTURE_USAGE_STORAGE_BIT | RenderingDevice.TEXTURE_USAGE_CAN_COPY_FROM_BIT
	return _rd.texture_create(format, RDTextureView.new(), [])


## The textures of one view (kept while the size and base factor stay).
func view_textures(index: int, size: Vector2i, factor: int, count := -1) -> Dictionary:
	while _views.size() <= index:
		_views.append({})
	var view: Dictionary = _views[index]
	var wanted := GlareSpread.level_sizes(size.x, size.y, factor, count)
	if not view.is_empty() and view["size"] == size and view["factor"] == factor and view["sizes"].size() == wanted.size():
		return view
	if not view.is_empty():
		_free_view(view)
	var sizes := GlareSpread.level_sizes(size.x, size.y, factor, count)
	var levels: Array[RID] = []
	for level_size in sizes:
		levels.append(_texture(level_size))
	view = {"size": size, "factor": factor, "sizes": sizes, "levels": levels, "scratch": _texture(sizes[0]),
			"a0": _texture(sizes[0])}
	_views[index] = view
	return view


func _image_uniform(binding: int, texture: RID) -> RDUniform:
	var uniform := RDUniform.new()
	uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	uniform.binding = binding
	uniform.add_id(texture)
	return uniform


func _dispatch(kernel: int, uniforms: Array, push: PackedFloat32Array, groups: Vector2i) -> void:
	var bytes := push.to_byte_array()
	var uniform_set: RID
	if _local:
		uniform_set = _rd.uniform_set_create(uniforms, _shaders[kernel], 0)
		_local_sets.append(uniform_set)
	else:
		uniform_set = UniformSetCacheRD.get_cache(_shaders[kernel], 0, uniforms)
	var list := _rd.compute_list_begin()
	_rd.compute_list_bind_compute_pipeline(list, _pipelines[kernel])
	_rd.compute_list_bind_uniform_set(list, uniform_set, 0)
	_rd.compute_list_set_push_constant(list, bytes, bytes.size())
	_rd.compute_list_dispatch(list, groups.x, groups.y, 1)
	_rd.compute_list_end()


static func _groups(size: Vector2i) -> Vector2i:
	return Vector2i((size.x - 1) / 8 + 1, (size.y - 1) / 8 + 1)


## The view ray parameters (a, b, c, d) of a projection (as AdaptationMeter.sun_mask_parameters).
static func ray_parameters(p: Projection) -> PackedFloat32Array:
	# The render data's projection can come with Vulkan's y flip (p.y.y < 0, p.z.y negated with it): the ratio p.z.y / p.y.y
	# is the same either way, the scale must be positive (image row 0 is the top, NDC y up). A sign here mirrored the Sun
	# vertically (a second disc and halo below the view axis).
	return PackedFloat32Array([1.0 / p.x.x, p.z.x / p.x.x, 1.0 / absf(p.y.y), p.z.y / p.y.y])


## The spread of one view's colour image: SOURCE, DOWN x (K - 1), BLUR x K (all but the APPLY step; the GPU check reads
## A_0 = levels[0] after this). ray: ray_parameters; sun: [direction in the eye's axes (3), mask cosine].
func build_veil(index: int, color: RID, size: Vector2i, ppd: float, ray: PackedFloat32Array, sun: PackedFloat32Array) -> Dictionary:
	var the_plan := GlareSpread.plan(ppd)
	var factor: int = the_plan["factor"]
	var view := view_textures(index, size, factor, the_plan["weights"].size())
	var sizes: Array = view["sizes"]
	var levels: Array = view["levels"]
	_dispatch(Kernel.SOURCE, [_image_uniform(0, color), _image_uniform(1, levels[0])],
			PackedFloat32Array([size.x, size.y, factor, _white, sizes[0].x, sizes[0].y, 0.0, 0.0,
			ray[0], ray[1], ray[2], ray[3], sun[0], sun[1], sun[2], sun[3]]), _groups(sizes[0]))
	for k in range(1, sizes.size()):
		_dispatch(Kernel.DOWN, [_image_uniform(0, levels[k - 1]), _image_uniform(1, levels[k])],
				PackedFloat32Array([sizes[k - 1].x, sizes[k - 1].y, sizes[k].x, sizes[k].y]), _groups(sizes[k]))
	for k in range(sizes.size() - 1, -1, -1):
		var level_size: Vector2i = sizes[k]
		var sigma: float = the_plan["sigmas"][k]
		var radius := GlareSpread.blur_radius(sigma)
		var inverse_two_sigma_squared := 0.5 / (sigma * sigma)
		_dispatch(Kernel.BLUR_H, [_image_uniform(0, levels[k]), _image_uniform(1, view["scratch"])],
				PackedFloat32Array([level_size.x, level_size.y, radius, inverse_two_sigma_squared]), _groups(level_size))
		var top := k == sizes.size() - 1
		var coarse: Vector2i = sizes[mini(k + 1, sizes.size() - 1)]
		_dispatch(Kernel.BLUR_V, [_image_uniform(0, view["scratch"]), _image_uniform(1, levels[mini(k + 1, sizes.size() - 1)]),
				_image_uniform(2, levels[k] if k > 0 else view["a0"])],
				PackedFloat32Array([level_size.x, level_size.y, radius, inverse_two_sigma_squared,
				coarse.x, coarse.y, float(the_plan["weights"][k]), 0.0 if top else 1.0]), _groups(level_size))
	return view


## colour += M (bilinear(A_0) + the Sun's veil), one view. local: the veil on a source is compressed as the local
## adaptation compresses it (GlareSpread.self_compression).
func apply_veil(view: Dictionary, color: RID, size: Vector2i, ray: PackedFloat32Array, sun_eye: Vector3, sun_scene_lux: float,
		sun_rgb: Vector3, image_part := true, map := Basis.IDENTITY, local := false) -> void:
	var sizes: Array = view["sizes"]
	var flags := (1 if image_part else 0) | (2 if local else 0)
	_dispatch(Kernel.APPLY, [_image_uniform(0, color), _image_uniform(1, view["a0"]), _image_uniform(2, view["levels"][0])],
			PackedFloat32Array([size.x, size.y, view["factor"], flags, sizes[0].x, sizes[0].y, GlareSpread.PLATEAU_SCENE,
			GlareSpread.SOURCE_WHITE, ray[0], ray[1], ray[2], ray[3], sun_eye.x, sun_eye.y, sun_eye.z, sun_scene_lux,
			sun_rgb.x, sun_rgb.y, sun_rgb.z, 0.0, map.x.x, map.x.y, map.x.z, 0.0, map.y.x, map.y.y, map.y.z, 0.0,
			map.z.x, map.z.y, map.z.z, 0.0]), _groups(size))


## VisionEffect's per-pixel map as one matrix: M v = C v + tint_scale (scotopic . v), its columns.
func vision_map() -> Basis:
	if vision == null:
		return Basis.IDENTITY
	var weights := vision._weights
	var tint := vision._tint_scale
	var cone := vision._cone
	return Basis(cone.x + tint * weights.x, cone.y + tint * weights.y, cone.z + tint * weights.z)


func _render_callback(_callback_type: int, render_data: RenderData) -> void:
	if stage == Stage.SPREAD:
		_spread(render_data)
	else:
		_add(render_data)


## The view's frame data: [ray parameters, ppd, Sun direction in the eye's axes]; [] for a degenerate projection.
func _frame_view(scene_data: RenderSceneData, head: Basis, index: int, size: Vector2i) -> Array:
	var projection := scene_data.get_view_projection(index)
	if projection.x.x == 0.0 or projection.y.y == 0.0:
		return []
	var eye_in_head := meter.eye_basis_in_head(index) if meter != null else Basis.IDENTITY
	var sun_eye := ((head * eye_in_head).transposed() * _sun_world).normalized() if _sun_world != Vector3.ZERO \
			else Vector3(0.0, 0.0, -1.0)
	return [ray_parameters(projection), GlareSpread.pixels_per_degree(float(size.y), absf(projection.y.y)), sun_eye]


func _spread(render_data: RenderData) -> void:
	_spread_sun.clear()
	if _rd == null or _pipelines.size() < 5 or not _active:
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	if size.x == 0 or size.y == 0:
		return
	var scene_data := render_data.get_render_scene_data()
	var head := scene_data.get_cam_transform().basis.orthonormalized()
	if CloudEffect.capture_timestamps:  # render_check --gpu_passes
		_rd.capture_timestamp("glare_spread")
	for index in range(buffers.get_view_count()):
		var frame := _frame_view(scene_data, head, index, size)
		if frame.is_empty():
			continue
		var mask := _mask_cos if _sun_world != Vector3.ZERO else 2.0
		var sun_eye: Vector3 = frame[2]
		build_veil(index, buffers.get_color_layer(index), size, frame[1], frame[0],
				PackedFloat32Array([sun_eye.x, sun_eye.y, sun_eye.z, mask]))
		_spread_sun[index] = [frame[0], sun_eye]
	if CloudEffect.capture_timestamps:
		_rd.capture_timestamp("after_glare_spread")


func _add(render_data: RenderData) -> void:
	if source == null or source._rd == null or source._pipelines.size() < 5 or source._spread_sun.is_empty():
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	var map := source.vision_map()
	if CloudEffect.capture_timestamps:
		source._rd.capture_timestamp("glare_add")
	for index: int in source._spread_sun:
		if index >= buffers.get_view_count() or index >= source._views.size() \
				or source._views[index].get("size", Vector2i.ZERO) != size:
			continue
		var frame: Array = source._spread_sun[index]
		var sun_eye: Vector3 = frame[1]
		source.apply_veil(source._views[index], buffers.get_color_layer(index), size, frame[0], sun_eye,
				source._sun_scene_lux if source._sun_world != Vector3.ZERO else 0.0, source._sun_rgb, true, map,
				LocalAdaptation.enabled and source._local_active)
	if CloudEffect.capture_timestamps:
		source._rd.capture_timestamp("after_glare_add")
