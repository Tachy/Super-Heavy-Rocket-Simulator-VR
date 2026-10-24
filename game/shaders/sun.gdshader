shader_type spatial;
// Solar disk with limb darkening (linear law, u = 0.6), emissive for glow; seen through the atmosphere (opaque:
// color x T + the sky in front of it), so it reddens and dims toward the horizon from the transmittance alone.
render_mode unshaded, cull_back;

#include "res://shaders/atmosphere.gdshaderinc"

uniform float intensity = 30.0;

varying vec3 local_normal;

void vertex() {
	local_normal = NORMAL;
}

void fragment() {
	float mu = clamp(dot(normalize(NORMAL), vec3(0.0, 0.0, 1.0)), 0.0, 1.0);  // view space: towards camera
	float limb = 1.0 - 0.6 * (1.0 - mu);
	vec3 color = vec3(1.0, 0.97, 0.92) * limb * intensity;
	if (atmosphere_enabled) {
		vec3 direction_world = normalize((INV_VIEW_MATRIX * vec4(VERTEX, 0.0)).xyz);
		vec3 transmittance;
		vec3 sky = atm_sky(atm_camera_local(), atmosphere_world_to_earth * direction_world, transmittance);
		color = color * transmittance + sky;
	}
	ALBEDO = color;
}
