shader_type spatial;
// VR overlay panel (game/xr/vr_overlay_panel.gd): a sphere section 2 m around the eye, drawn over the whole scene.
// The orbit camera's near plane grows with the camera distance (up to 1 km) and would clip the panel; a smaller near
// plane is no way out (OpenXR builds the eye projections in float32: near 0.1 m with far 3e7 m loses the far plane).
// The panel is projected normally (x, y, w: its direction on screen does not depend on near / far) and only its clip
// depth is pinned inside the depth range; depth test and depth writes are off, so the depth value is never used.
// The overlay texture holds premultiplied alpha (2D drawing onto a cleared transparent target).
render_mode unshaded, blend_premul_alpha, depth_test_disabled, depth_draw_never, cull_back, shadows_disabled, fog_disabled;

#include "res://shaders/vision.gdshaderinc"

uniform sampler2D overlay : source_color, filter_linear, repeat_disable;
// The mouse pointer (VR only), composited over the overlay so it stays above embedded popups; tip at pointer_px.
uniform sampler2D pointer : source_color, filter_linear, repeat_disable;
uniform vec2 overlay_size_px = vec2(1600.0, 900.0);
uniform vec2 pointer_px = vec2(-1.0e6);  // no pointer until set
uniform vec2 pointer_size_px = vec2(32.0);

void vertex() {
	POSITION = PROJECTION_MATRIX * (MODELVIEW_MATRIX * vec4(VERTEX, 1.0));
	POSITION.z = 0.5 * POSITION.w;  // halfway between near and far (reverse-Z or not), never clipped
}

void fragment() {
	vec4 texel = texture(overlay, UV);
	vec2 local = (UV * overlay_size_px - pointer_px) / pointer_size_px;
	if (all(greaterThanEqual(local, vec2(0.0))) && all(lessThan(local, vec2(1.0)))) {
		vec4 arrow = texture(pointer, local);  // premultiplied, as the overlay
		texel = arrow + texel * (1.0 - arrow.a);
	}
	ALBEDO = vision_display_referred(texel.rgb);  // UI: past night vision's rod step unchanged
	ALPHA = texel.a;
}
