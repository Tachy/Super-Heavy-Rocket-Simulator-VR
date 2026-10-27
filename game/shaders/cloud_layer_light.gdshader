shader_type spatial;
// The composite's second pass (static clouds Task 9 fix round 1): the next pass of cloud_layer.gdshader's quad adds the
// cloud pass's premultiplied colour - the clouds' light and the air in front of them - to the scene that pass has
// multiplied by the background's transmittance: final = T_bg x scene + colour.
render_mode unshaded, blend_add, depth_draw_never, depth_test_disabled, cull_disabled, shadows_disabled,
		fog_disabled, skip_vertex_transform;

global uniform bool cloud_enabled;
uniform sampler2DArray cloud_image : filter_nearest, repeat_disable;  // CloudEffect.output: one layer per view

void vertex() {
	POSITION = vec4(VERTEX.xy, 0.5, 1.0);  // the quad covers the screen in every view (depth test off)
}

void fragment() {
	if (!cloud_enabled) {
		discard;
	}
	vec3 c = texture(cloud_image, vec3(SCREEN_UV, float(VIEW_INDEX))).rgb;
	if (dot(c, c) <= 0.0) {
		discard;
	}
	ALBEDO = c;
}
