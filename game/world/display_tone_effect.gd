class_name DisplayToneEffect
extends CompositorEffect
## The display tone step (look fixes T5; the model and its CPU mirror: DisplayTone): a full-screen step on the HDR scene
## after the eye's post step (VisionEffect), before glow and Godot's AgX. It writes, per pixel, the scene value AgX maps
## to the chroma-preserving, hue-stable display colour with the daylight Hunt factor, so the engine's tone mapper, glow
## and encoding stay as they are. A per-pixel function with uniforms of the frame: identical for both VR eyes (every view
## layer of a multiview buffer). Active only while the environment's tone mapper is AgX (the linear camera exposures
## and the GPU checks with their own linear environments stay colourimetric) and DisplayTone.enabled; without a
## RenderingDevice (headless, Compatibility) it does nothing. update() also sets the globals of the emitters' inverse
## (vision.gdshaderinc): vision_hunt (1 while the step is off), vision_tone / vision_tone_white (0 while off).

const SHADER_CODE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform image2D color_image;
layout(push_constant, std430) uniform Params {
	vec4 curve;  // the allenwp curve: contrast, toe_a, slope, w
	vec4 shape;  // Hunt factor, log2 path-to-white start, log2 white, white
	vec4 size;   // width, height, gamut knee, unused
} params;

// DisplayTone (game/world/display_tone.gd) is the CPU mirror of everything below; keep them identical.
const vec3 LUMINANCE = vec3(0.2126, 0.7152, 0.0722);
const float X0 = 0.18;
const mat3 M1 = mat3(0.4122214708, 0.2119034982, 0.0883024619, 0.5363325363, 0.6806995451, 0.2817188376,
		0.0514459929, 0.1073969566, 0.6299787005);
const mat3 M2 = mat3(0.2104542553, 1.9779984951, 0.0259040371, 0.7936177850, -2.4285922050, 0.7827717662,
		-0.0040720468, 0.4505937099, -0.8086757660);
const mat3 INSET = mat3(0.544814746488245, 0.140416948464053, 0.0888104196149096, 0.373787398372697,
		0.754137554567394, 0.178871756420858, 0.0813978551390581, 0.105445496968552, 0.732317823964232);
const mat3 OUTSET = mat3(1.96488741169489, -0.299313364904742, -0.164352742528393, -0.855988495690215,
		1.32639796461980, -0.238183969428088, -0.108898916004672, -0.0270845997150571, 1.40253671195648);
const int GAMUT_STEPS = 12;
const float GAMUT_CHROMA_START = 0.4;
const float CURVE_INVERSE_MAX = 0.999999;
const float REACH_EPSILON = 1e-5;

float curve(float x) {
	x = max(x, 0.0);
	if (x < X0) {
		float t = pow(x, params.curve.x);
		return t / (t + params.curve.y);
	}
	float s = x - X0;
	float slope_s = params.curve.z * s;
	return min(slope_s * (1.0 + s / params.curve.w) / (1.0 + slope_s / (1.0 - X0)) + X0, 1.0);
}

float curve_inverse(float y) {
	y = clamp(y, 0.0, CURVE_INVERSE_MAX);
	if (y < X0) {
		return pow(params.curve.y * y / (1.0 - y), 1.0 / params.curve.x);
	}
	float d = y - X0;
	float a = params.curve.z / params.curve.w;
	float b = params.curve.z * (1.0 - d / (1.0 - X0));
	return 2.0 * d / (b + sqrt(max(b * b + 4.0 * a * d, 0.0))) + X0;
}

vec3 predistort(vec3 d, mat3 outset_inverse, mat3 inset_inverse) {
	vec3 u = outset_inverse * d;
	return inset_inverse * vec3(curve_inverse(u.x), curve_inverse(u.y), curve_inverse(u.z));
}

vec3 to_oklab(vec3 c) {
	vec3 lms = M1 * c;
	return M2 * (sign(lms) * pow(abs(lms), vec3(1.0 / 3.0)));
}

vec3 from_oklab(vec3 lab, mat3 m2_inverse, mat3 m1_inverse) {
	vec3 lms = m2_inverse * lab;
	return m1_inverse * (lms * lms * lms);
}

bool reachable(vec3 d, mat3 outset_inverse, mat3 inset_inverse) {
	float slack = REACH_EPSILON * max(d.x, max(d.y, d.z));
	if (any(lessThan(d, vec3(-slack))) || any(greaterThan(d, vec3(1.0 + REACH_EPSILON)))) {
		return false;
	}
	vec3 x = predistort(d, outset_inverse, inset_inverse);
	return all(greaterThanEqual(x, vec3(-REACH_EPSILON * max(x.x, max(x.y, x.z)))));
}

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	if (pixel.x >= int(params.size.x) || pixel.y >= int(params.size.y)) {
		return;
	}
	vec4 color = imageLoad(color_image, pixel);
	vec3 x = clamp(color.rgb, vec3(0.0), vec3(65000.0));
	if (any(isnan(x))) {
		x = vec3(0.0);
	}
	float y = dot(LUMINANCE, x);
	float white = params.shape.w;
	vec3 result = vec3(0.0);
	if (y >= white) {
		// display white: every AgX channel at least white, the magnitude kept for the glow
		vec3 c = INSET * x;
		result = x * max(1.0, white / min(c.x, min(c.y, c.z)));
	} else if (y > 0.0) {
		mat3 outset_inverse = inverse(OUTSET);
		mat3 inset_inverse = inverse(INSET);
		mat3 m1_inverse = inverse(M1);
		mat3 m2_inverse = inverse(M2);
		float g = curve(y);
		vec3 lab = to_oklab(x * (g / y));
		float lightness = min(lab.x, 1.0);
		float chroma = length(lab.yz);
		vec3 shown_rgb = vec3(g);
		if (chroma > 1e-7) {
			vec2 hue = lab.yz / chroma;
			chroma *= params.shape.x;
			float t = clamp((log2(y) - params.shape.y) / (params.shape.z - params.shape.y), 0.0, 1.0);
			chroma *= 1.0 - t * t * (3.0 - 2.0 * t);
			float knee = params.size.z;
			float shown = chroma;
			// Below the knee of the gamut (C_max >= C / knee) the chroma stays: one test, no bisection.
			if (!reachable(from_oklab(vec3(lightness, hue * (chroma / knee)), m2_inverse, m1_inverse), outset_inverse,
					inset_inverse)) {
				float lo = 0.0;
				float hi = GAMUT_CHROMA_START;
				for (int i = 0; i < GAMUT_STEPS; i++) {
					float mid = 0.5 * (lo + hi);
					if (reachable(from_oklab(vec3(lightness, hue * mid), m2_inverse, m1_inverse), outset_inverse,
							inset_inverse)) {
						lo = mid;
					} else {
						hi = mid;
					}
				}
				float r = lo > 1e-9 ? chroma / lo : 0.0;
				float compressed = r < knee ? r : 1.0 - (1.0 - knee) * exp(-(r - knee) / (1.0 - knee));
				shown = compressed * lo;
			}
			shown_rgb = clamp(from_oklab(vec3(lightness, hue * shown), m2_inverse, m1_inverse), vec3(0.0), vec3(1.0));
		}
		result = max(predistort(shown_rgb, outset_inverse, inset_inverse), vec3(0.0));
	}
	imageStore(color_image, pixel, vec4(result, color.a));
}
"""

var _rd: RenderingDevice
var _shader := RID()
var _pipeline := RID()
## Render-thread copies (set through update on the render thread).
var _active := false
var _push := PackedFloat32Array()


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
	access_resolved_color = true
	RenderingServer.call_on_render_thread(_create_pipeline)


## Main thread, every frame: the environment (its tone mapper and AgX parameters) and the eye's chroma factor
## (Atmosphere.chroma; its daylight part EyeAdaptation.display_hunt is applied here). Sets the global vision_hunt.
func update(environment: Environment, chroma_factor: float) -> void:
	var active := enabled and DisplayTone.enabled and environment != null \
			and environment.tonemap_mode == Environment.TONE_MAPPER_AGX
	var hunt := EyeAdaptation.display_hunt(chroma_factor) if active else 1.0
	ShaderGlobals.set_value(&"vision_hunt", hunt)
	var push := PackedFloat32Array()
	var tone := Vector4.ZERO
	var white := 0.0
	if active:
		var p := DisplayTone.curve_parameters(environment.tonemap_agx_contrast, environment.tonemap_agx_white)
		tone = Vector4(p[0], p[1], p[2], p[3])
		white = p[4]
		push = PackedFloat32Array([p[0], p[1], p[2], p[3], hunt, log(DisplayTone.PATH_TO_WHITE_START) / log(2.0),
				log(p[4]) / log(2.0), p[4], 0.0, 0.0, DisplayTone.GAMUT_KNEE, 0.0])
	ShaderGlobals.set_value(&"vision_tone", tone)  # the emitters' inverse of the step (vision.gdshaderinc)
	ShaderGlobals.set_value(&"vision_tone_white", white)
	RenderingServer.call_on_render_thread(_set_on_render_thread.bind(active, push))


func _set_on_render_thread(active: bool, push: PackedFloat32Array) -> void:
	_active = active
	_push = push


func _create_pipeline() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	var source := RDShaderSource.new()
	source.language = RenderingDevice.SHADER_LANGUAGE_GLSL
	source.source_compute = SHADER_CODE
	var spirv := _rd.shader_compile_spirv_from_source(source)
	if spirv.compile_error_compute != "":
		push_error("DisplayToneEffect: " + spirv.compile_error_compute)
		_rd = null
		return
	_shader = _rd.shader_create_from_spirv(spirv)
	if _shader.is_valid():
		_pipeline = _rd.compute_pipeline_create(_shader)


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd != null and _shader.is_valid():
		_rd.free_rid(_shader)  # frees the pipeline with it


func _render_callback(_callback_type: int, render_data: RenderData) -> void:
	if _rd == null or not _pipeline.is_valid() or not _active:
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	if size.x == 0 or size.y == 0:
		return
	var push := _push.duplicate()
	push[8] = float(size.x)
	push[9] = float(size.y)
	var bytes := push.to_byte_array()
	var groups_x := (size.x - 1) / 8 + 1
	var groups_y := (size.y - 1) / 8 + 1
	for view in range(buffers.get_view_count()):
		var image := buffers.get_color_layer(view)
		var uniform := RDUniform.new()
		uniform.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
		uniform.binding = 0
		uniform.add_id(image)
		var uniform_set := UniformSetCacheRD.get_cache(_shader, 0, [uniform])
		var list := _rd.compute_list_begin()
		_rd.compute_list_bind_compute_pipeline(list, _pipeline)
		_rd.compute_list_bind_uniform_set(list, uniform_set, 0)
		_rd.compute_list_set_push_constant(list, bytes, bytes.size())
		_rd.compute_list_dispatch(list, groups_x, groups_y, 1)
		_rd.compute_list_end()
