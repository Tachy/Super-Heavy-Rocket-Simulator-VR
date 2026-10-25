class_name VisionEffect
extends CompositorEffect
## Night vision's rod signal (EyeAdaptation, user decision 2026-10-25): a full-screen step on the HDR scene after the
## transparent pass, before glow and the tone mapper. Every pixel gets
##     rgb += rod_scale x max(SCOTOPIC_WEIGHTS . rgb, 0) x ROD_TINT
## (Ferwerda et al. 1996 Eq. 6: the cones' image, already scaled by the cone gain in the shaders, plus the rods'
## achromatic, slightly blue image). A per-pixel function of the pixel alone with uniforms of the frame: identical for
## both VR eyes (every view layer of a multiview buffer), no view dependence. rod_scale 0 (daylight, or the adaptation
## off): nothing is dispatched. Display-referred emitters take the inverse in their shaders (shaders/vision.gdshaderinc,
## globals vision_rod_weights / vision_inverse_tint), so they leave this step as they were drawn.
## Without a RenderingDevice (headless, Compatibility) it does nothing.

const SHADER_CODE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform image2D color_image;
layout(push_constant, std430) uniform Params {
	vec4 tint_scale;  // ROD_TINT x rod_scale, unused
	vec4 weights;     // SCOTOPIC_WEIGHTS, unused
	vec4 size;        // width, height, unused, unused
} params;

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	if (pixel.x >= int(params.size.x) || pixel.y >= int(params.size.y)) {
		return;
	}
	vec4 color = imageLoad(color_image, pixel);
	// Half-float overflow (a bright source in a dark-adapted view) or a NaN must not spread through the sum.
	vec3 c = clamp(color.rgb, vec3(-65000.0), vec3(65000.0));
	if (any(isnan(c))) {
		c = vec3(0.0);
	}
	float rods = max(dot(params.weights.rgb, c), 0.0);
	c += params.tint_scale.rgb * rods;
	imageStore(color_image, pixel, vec4(min(c, vec3(65000.0)), color.a));
}
"""

var _rd: RenderingDevice
var _shader := RID()
var _pipeline := RID()
## Render-thread copies (set through set_rods on the render thread, in order with the global shader parameters).
var _tint_scale := Vector3.ZERO
var _weights := Vector3.ZERO


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
	access_resolved_color = true
	RenderingServer.call_on_render_thread(_create_pipeline)


## Main thread: this frame's rod scale (EyeAdaptation.rod_scale) and weights; applied on the render thread.
func set_rods(rod_scale: float) -> void:
	var tint_scale := EyeAdaptation.ROD_TINT * rod_scale
	RenderingServer.call_on_render_thread(_set_on_render_thread.bind(tint_scale, EyeAdaptation.SCOTOPIC_WEIGHTS))


func _set_on_render_thread(tint_scale: Vector3, weights: Vector3) -> void:
	_tint_scale = tint_scale
	_weights = weights


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
	if _rd == null or not _pipeline.is_valid() or _tint_scale == Vector3.ZERO:
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	if size.x == 0 or size.y == 0:
		return
	var push := PackedFloat32Array([_tint_scale.x, _tint_scale.y, _tint_scale.z, 0.0,
			_weights.x, _weights.y, _weights.z, 0.0, float(size.x), float(size.y), 0.0, 0.0])
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
