class_name VisionEffect
extends CompositorEffect
## Night vision's rod signal and the cones' white balance and chroma (EyeAdaptation, user decisions 2026-10-25; the
## white balance: look fixes T2, 2026-11-07): a full-screen step on the HDR scene after the transparent pass, before glow
## and the tone mapper. Every pixel becomes
##     rgb' = C rgb + rod_scale x (SCOTOPIC_WEIGHTS . rgb) x ROD_TINT,   C = EyeAdaptation.cone_map(chroma, white balance)
## (Ferwerda et al. 1996 Eq. 6: the cones' image, already scaled by the cone gain and the brightness factor in the
## shaders, adapted to the Sun's white and at the Hunt effect's chroma, plus the rods' achromatic, slightly blue image).
## Linear in the pixel (the
## scotopic weights are non-negative, so no clamp is needed for a scene pixel): the display-referred inverse is exact
## also under alpha and additive blending. A per-pixel function of the pixel alone with uniforms of the frame:
## identical for both VR eyes (every view layer of a multiview buffer), no view dependence. rod_scale 0 and C the
## identity (the adaptation off): nothing is dispatched. Display-referred emitters take the inverse in their shaders
## (shaders/vision.gdshaderinc, globals vision_rod_weights / vision_inverse_tint / vision_cone_inverse), so they leave this
## step as they were drawn. Without a RenderingDevice (headless, Compatibility) it does nothing.

const SHADER_CODE := """
#version 450
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
layout(rgba16f, set = 0, binding = 0) uniform image2D color_image;
layout(push_constant, std430) uniform Params {
	vec4 tint_scale;  // ROD_TINT x rod_scale, unused
	vec4 weights;     // SCOTOPIC_WEIGHTS, unused
	vec4 size;        // width, height, unused, unused
	vec4 cone_x;      // the cones' map C (EyeAdaptation.cone_map), its columns
	vec4 cone_y;
	vec4 cone_z;
} params;

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
	float rods = dot(params.weights.rgb, c);
	vec3 result = mat3(params.cone_x.rgb, params.cone_y.rgb, params.cone_z.rgb) * c + params.tint_scale.rgb * rods;
	imageStore(color_image, pixel, vec4(clamp(result, vec3(-65000.0), vec3(65000.0)), color.a));
}
"""

var _rd: RenderingDevice
var _shader := RID()
var _pipeline := RID()
## Render-thread copies (set through set_rods on the render thread, in order with the global shader parameters).
var _tint_scale := Vector3.ZERO
var _weights := Vector3.ZERO
var _cone := Basis.IDENTITY


func _init() -> void:
	effect_callback_type = EFFECT_CALLBACK_TYPE_POST_TRANSPARENT
	access_resolved_color = true
	RenderingServer.call_on_render_thread(_create_pipeline)


## Main thread: this frame's rod scale (EyeAdaptation.rod_scale) and the cones' map (EyeAdaptation.cone_map of the chroma
## factor and the white balance); applied on the render thread.
func set_rods(rod_scale: float, cone := Basis.IDENTITY) -> void:
	var tint_scale := EyeAdaptation.ROD_TINT * rod_scale
	RenderingServer.call_on_render_thread(_set_on_render_thread.bind(tint_scale, EyeAdaptation.SCOTOPIC_WEIGHTS,
			cone))


func _set_on_render_thread(tint_scale: Vector3, weights: Vector3, cone: Basis) -> void:
	_tint_scale = tint_scale
	_weights = weights
	_cone = cone


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
	if _rd == null or not _pipeline.is_valid() or (_tint_scale == Vector3.ZERO and _cone == Basis.IDENTITY):
		return
	var buffers := render_data.get_render_scene_buffers() as RenderSceneBuffersRD
	if buffers == null:
		return
	var size := buffers.get_internal_size()
	if size.x == 0 or size.y == 0:
		return
	var push := PackedFloat32Array([_tint_scale.x, _tint_scale.y, _tint_scale.z, 0.0,
			_weights.x, _weights.y, _weights.z, 0.0, float(size.x), float(size.y), 0.0, 0.0,
			_cone.x.x, _cone.x.y, _cone.x.z, 0.0, _cone.y.x, _cone.y.y, _cone.y.z, 0.0,
			_cone.z.x, _cone.z.y, _cone.z.z, 0.0])
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
