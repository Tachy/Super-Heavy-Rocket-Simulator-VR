class_name VisionEffect
extends CompositorEffect
## Night vision's rod signal and the cones' chroma (EyeAdaptation, user decisions 2026-10-25): a full-screen step on the
## HDR scene after the transparent pass, before glow and the tone mapper. Every pixel becomes
##     rgb' = Y + chroma x (rgb - Y) + rod_scale x (SCOTOPIC_WEIGHTS . rgb) x ROD_TINT,   Y = BT.709 luminance of rgb
## (Ferwerda et al. 1996 Eq. 6: the cones' image, already scaled by the cone gain and the brightness factor in the
## shaders, at the Hunt effect's chroma, plus the rods' achromatic, slightly blue image). Linear in the pixel (the
## scotopic weights are non-negative, so no clamp is needed for a scene pixel): the display-referred inverse is exact
## also under alpha and additive blending. A per-pixel function of the pixel alone with uniforms of the frame:
## identical for both VR eyes (every view layer of a multiview buffer), no view dependence. rod_scale 0 and chroma 1
## (daylight, or the adaptation off): nothing is dispatched. Display-referred emitters take the inverse in their shaders
## (shaders/vision.gdshaderinc, globals vision_rod_weights / vision_inverse_tint / vision_chroma), so they leave this
## step as they were drawn. Without a RenderingDevice (headless, Compatibility) it does nothing.

const SHADER_CODE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform image2D color_image;
layout(push_constant, std430) uniform Params {
	vec4 tint_scale;  // ROD_TINT x rod_scale, unused
	vec4 weights;     // SCOTOPIC_WEIGHTS, unused
	vec4 size;        // width, height, chroma, unused
} params;

const vec3 LUMINANCE_WEIGHTS = vec3(0.2126, 0.7152, 0.0722);  // [S: ITU-R BT.709]

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	if (pixel.x >= int(params.size.x) || pixel.y >= int(params.size.y)) {
		return;
	}
	vec4 color = imageLoad(color_image, pixel);
	// Half-float overflow (a bright source in a dark-adapted view) or a NaN must not spread through the sums.
	vec3 c = clamp(color.rgb, vec3(-65000.0), vec3(65000.0));
	if (any(isnan(c))) {
		c = vec3(0.0);
	}
	float y = dot(LUMINANCE_WEIGHTS, c);
	float rods = dot(params.weights.rgb, c);
	vec3 result = vec3(y) + params.size.z * (c - vec3(y)) + params.tint_scale.rgb * rods;
	imageStore(color_image, pixel, vec4(clamp(result, vec3(-65000.0), vec3(65000.0)), color.a));
}
"""

var _rd: RenderingDevice
var _shader := RID()
var _pipeline := RID()
## Render-thread copies (set through set_rods on the render thread, in order with the global shader parameters).
var _tint_scale := Vector3.ZERO
var _weights := Vector3.ZERO
var _chroma := 1.0


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
	access_resolved_color = true
	RenderingServer.call_on_render_thread(_create_pipeline)


## Main thread: this frame's rod scale (EyeAdaptation.rod_scale) and the cones' chroma factor (EyeAdaptation.chroma);
## applied on the render thread.
func set_rods(rod_scale: float, chroma := 1.0) -> void:
	var tint_scale := EyeAdaptation.ROD_TINT * rod_scale
	RenderingServer.call_on_render_thread(_set_on_render_thread.bind(tint_scale, EyeAdaptation.SCOTOPIC_WEIGHTS,
			chroma))


func _set_on_render_thread(tint_scale: Vector3, weights: Vector3, chroma: float) -> void:
	_tint_scale = tint_scale
	_weights = weights
	_chroma = chroma


func _create_pipeline() -> void:
	_rd = RenderingServer.get_rendering_device()
	if _rd == null:
		return
	var source := RDShaderSource.new()
	source.language = RenderingDevice.SHADER_LANGUAGE_GLSL
	source.source_compute = SHADER_CODE
	var spirv := _rd.shader_compile_spirv_from_source(source)
	if spirv.compile_error_compute != "":
		push_error("VisionEffect: " + spirv.compile_error_compute)
		_rd = null
		return
	_shader = _rd.shader_create_from_spirv(spirv)
	if _shader.is_valid():
		_pipeline = _rd.compute_pipeline_create(_shader)


func _notification(what: int) -> void:
	if what == NOTIFICATION_PREDELETE and _rd != null and _shader.is_valid():
		_rd.free_rid(_shader)  # frees the pipeline with it


func _render_callback(_callback_type: int, render_data: RenderData) -> void:
	if _rd == null or not _pipeline.is_valid() or (_tint_scale == Vector3.ZERO and _chroma == 1.0):
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	if size.x == 0 or size.y == 0:
		return
	var push := PackedFloat32Array([_tint_scale.x, _tint_scale.y, _tint_scale.z, 0.0,
			_weights.x, _weights.y, _weights.z, 0.0, float(size.x), float(size.y), _chroma, 0.0])
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
