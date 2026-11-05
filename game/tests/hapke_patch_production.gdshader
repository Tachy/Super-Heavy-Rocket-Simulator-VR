shader_type spatial;
// A flat regolith patch through the PRODUCTION wiring for moon_surface_check.gd's parity (moon procedural surface plan
// Task 9 fix round 1): moon_regolith.gdshaderinc's regolith_reflectance_rough with the global uniforms that
// Atmosphere.configure_moon_brdf sets (moon_brdf, moon_hapke_albedo_phase, moon_hapke_opposition, moon_hapke_norm,
// moon_hapke_theta_bar_rad), and terrain_moon.gdshader's theta_bar resolution (hapke_theta_bar_rad < 0: the globe's).
// ALBEDO = gain x moon_hapke_norm x Hapke I/F (unshaded, linear; an HDR viewport keeps the value).
render_mode unshaded, cull_disabled;

#include "res://shaders/moon_regolith.gdshaderinc"

uniform float hapke_theta_bar_rad = -1.0;  // as terrain_moon.gdshader's uniform
uniform vec3 light_world = vec3(0.0, 1.0, 0.0);  // unit direction to the light
uniform float gain = 1.0;

void fragment() {
	vec3 n = normalize(NORMAL);
	vec3 to_light = normalize((VIEW_MATRIX * vec4(light_world, 0.0)).xyz);
	vec3 to_eye = normalize(VIEW);
	float theta_bar = hapke_theta_bar_rad < 0.0 ? moon_hapke_theta_bar_rad : hapke_theta_bar_rad;
	ALBEDO = vec3(gain * regolith_reflectance_rough(n, to_light, to_eye, theta_bar));
}
