shader_type spatial;
// A flat regolith patch through the production detail shading (moon procedural surface plan Task 10) for
// moon_surface_check.gd: moon_regolith.gdshaderinc's regolith_reflectance_rough (globals from configure_moon_brdf) with
// moon_detail.gdshaderinc's residual theta_bar, light factors and texture modulation, composed as terrain_moon.gdshader
// composes them (albedo 1, moon_regolith_scale and the light scene values left out). MoonDetailShading.configure sets the
// detail uniforms; the patch's own: its node spacing (detail_cell_m x 64 = the patch edge, UV -> grid), the horizon
// spacing, the abundance, which lights shine. ALBEDO = gain x (sun term + earth term) (unshaded, linear, HDR viewport).
render_mode unshaded, cull_disabled;

#include "res://shaders/moon_regolith.gdshaderinc"
#include "res://shaders/moon_detail.gdshaderinc"

uniform vec3 light_world = vec3(0.0, 1.0, 0.0);  // unit direction to the Sun
uniform vec3 earth_world = vec3(0.0, 1.0, 0.0);  // unit direction to the Earth
uniform float sun_on = 1.0;
uniform float earth_on = 0.0;
uniform float patch_spacing_m = 1.0;  // the node spacing the statistics and texture weights see
uniform float patch_abundance = 1.0;
uniform float gain = 1.0;

void fragment() {
	vec3 n = normalize(NORMAL);
	vec3 to_sun = normalize((VIEW_MATRIX * vec4(light_world, 0.0)).xyz);
	vec3 to_earth = normalize((VIEW_MATRIX * vec4(earth_world, 0.0)).xyz);
	vec3 to_eye = normalize(VIEW);
	vec3 gradient = detail_texture_gradient(UV * 64.0, VERTEX, n, patch_spacing_m);
	float theta_bar = detail_shading ? detail_theta_bar(patch_spacing_m) : moon_hapke_theta_bar_rad;
	float elevation = asin(clamp(dot(n, to_sun), -1.0, 1.0));
	vec2 factors = vec2(1.0);
	float sun_modulation = 1.0;
	float earth_modulation = 1.0;
	if (detail_shading) {
		factors = detail_light_factors(patch_abundance, 1.0e30, patch_spacing_m, detail_horizon_spacing_m, elevation);
		sun_modulation = detail_modulation(gradient, n, to_sun, to_eye);
		earth_modulation = detail_modulation(gradient, n, to_earth, to_eye);
	}
	float value = sun_on * regolith_reflectance_rough(n, to_sun, to_eye, theta_bar) * factors.x * sun_modulation
			+ earth_on * regolith_reflectance_rough(n, to_earth, to_eye, theta_bar) * factors.y * earth_modulation;
	ALBEDO = vec3(gain * value);
}
