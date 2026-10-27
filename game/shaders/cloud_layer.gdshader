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
// The raymarch itself (cloud_pass.gdshaderinc CLOUD_MARCH, with clouds.gdshaderinc and atmosphere.gdshaderinc):
// - Steps: outside the field's cloudy region coarse steps growing with the distance (the fraction that spans the
//   segment from 1 km to its end in cloud_steps steps; at most one height layer: 250 m below 4 km, 1 km above),
//   inside it fine steps (cloud_fine_step_fraction of the distance, >= 10 m), at most cloud_max_steps per pixel; each
//   sample at the blue-noise jittered point of its step.
// - Extinction: clouds.gdshaderinc (field with the vertical profile rule, soft-edged mean-preserving shapes, ice
//   sheets, faded detail). The background behind the cloud is seen through exp(-tau) (Beer-Lambert of the unscaled
//   extinction); the scattered light is attenuated on the scaled depths exp(-(1 - f) tau) (cloud_optics.hpp).
// - Light per sample: the Sun transmitted through the air to the sample (atm_sun_at x the eclipse's unocculted part) x
//   the Sun's 3D source (cloud_scattering_source_3d: the light march toward the Sun through the 3D density, the
//   diffuse field of the surface the light entered through - sunlit flanks and bases under a Sun below the local
//   horizon get their true incidence), plus the clear sky's horizontal irradiance there (atm_sky_light) x the sky
//   source; at night the Moon the same way (atmosphere_moon_light). Each step integrates the source exactly over its
//   constant extinction: L += T J (1 - exp(-(1 - f) dtau)) / (1 - f). Early exit when the scaled transmittance falls
//   below 0.01.
// - Compositing: the clouds' light seen through the air to their transmittance-weighted mean distance d_c
//   (atm_to_point_offset: T_c, S_c) - exact for cloud light at one distance; a deck seen at a slant spreads over its
//   depth.
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
