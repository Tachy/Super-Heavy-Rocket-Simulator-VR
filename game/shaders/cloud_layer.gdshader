shader_type spatial;
// Volumetric clouds near the camera (M8 weather and clouds, Task 8): the composite. A full-screen quad (CloudLayer)
// drawn first in the transparent pass (plumes and the other transparent objects over it) blends the cloud pass's
// full-resolution result (CloudEffect, compute: shaders/cloud_pass.gdshaderinc) over the opaque scene: premultiplied,
//     final = T_bg x scene + (1 - T_bg) S_c + T_c L
// (the pass's colour holds the clouds' light seen through the air T_c L plus the air in front of them (1 - T_bg) S_c,
// its .a the background's transmittance T_bg itself; the scene's own air light behind a cloud is dimmed with it), in
// two passes of the quad: this one multiplies the scene by T_bg (blend_mul), its next pass (cloud_layer_light.gdshader)
// adds the colour (blend_add). The premultiplied blend's factor 1 - alpha is rounded to the half-float target's
// precision (measured: 1 - T was kept to 2^-11, so the Sun's disk stayed at 4.9e-4 of itself behind any deck of a slant
// tau > 7.6); the multiply's factor T keeps the half float's relative precision.
//
// The march itself: the baked volume (cloud_baked.gdshaderinc, CloudBaked; docs/subsystems/weather-clouds.md).
render_mode unshaded, blend_mul, depth_draw_never, depth_test_disabled, cull_disabled, shadows_disabled,
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
	float transmittance = texture(cloud_image, vec3(SCREEN_UV, float(VIEW_INDEX))).a;  // T_bg
	if (transmittance >= 1.0) {
		discard;
	}
	ALBEDO = vec3(transmittance);
}
