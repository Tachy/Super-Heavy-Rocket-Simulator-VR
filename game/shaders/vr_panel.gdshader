shader_type spatial;
// VR overlay panel (game/xr/vr_overlay_panel.gd): a sphere section 2 m around the eye, drawn over the whole scene.
// The orbit camera's near plane grows with the camera distance (up to 1 km) and would clip the panel; a smaller near
// plane is no way out (OpenXR builds the eye projections in float32: near 0.1 m with far 3e7 m loses the far plane).
// The panel is projected normally (x, y, w: its direction on screen does not depend on near / far) and only its clip
// depth is pinned inside the depth range; depth test and depth writes are off, so the depth value is never used.
// The overlay texture holds premultiplied alpha (2D drawing onto a cleared transparent target).
render_mode unshaded, blend_premul_alpha, depth_test_disabled, depth_draw_never, cull_back, shadows_disabled, fog_disabled;

uniform sampler2D overlay : source_color, filter_linear, repeat_disable;

void vertex() {
	POSITION = PROJECTION_MATRIX * (MODELVIEW_MATRIX * vec4(VERTEX, 1.0));
	POSITION.z = 0.5 * POSITION.w;  // halfway between near and far (reverse-Z or not), never clipped
}

void fragment() {
	vec4 texel = texture(overlay, UV);
	ALBEDO = texel.rgb;
	ALPHA = texel.a;
}
