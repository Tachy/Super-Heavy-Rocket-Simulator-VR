shader_type spatial;
// A flat regolith patch through moon_hapke.gdshaderinc for moon_surface_check.gd's shader parity (moon procedural surface
// plan Task 9): ALBEDO = gain x Hapke's I/F with the patch normal, the view vector and the light direction (unshaded,
// linear; an HDR viewport keeps the value). calibrate: a constant gain x 0.25 instead (the readback chain's check).
render_mode unshaded, cull_disabled;

#include "res://shaders/moon_hapke.gdshaderinc"

uniform vec4 albedo_phase;  // w, b, c, K
uniform vec4 opposition;  // B_S0, h_S, B_C0, h_C
uniform float theta_bar = 0.0;
uniform vec3 light_world = vec3(0.0, 1.0, 0.0);  // unit direction to the light
uniform float gain = 1.0;
uniform bool calibrate = false;

void fragment() {
	vec3 n = normalize(NORMAL);
	vec3 to_light = normalize((VIEW_MATRIX * vec4(light_world, 0.0)).xyz);
	vec3 to_eye = normalize(VIEW);
	float value = calibrate ? 0.25 : hapke_reflectance(albedo_phase, opposition, dot(n, to_light), dot(n, to_eye),
			dot(to_light, to_eye), theta_bar);
	ALBEDO = vec3(gain * value);
}
