shader_type spatial;
// Solar disk with limb darkening (linear law, u = limb_darkening), emissive for glow; seen through the atmosphere
// (opaque: color x T + the sky in front of it), so it reddens and dims toward the horizon from the transmittance alone.
// Colour: with the atmosphere loaded, the model's top-of-atmosphere solar spectrum in the luminance mode
// (atmosphere_sun_illuminance_lux, the same colour the Sun light and the ground lighting use), normalised to unit
// luminance so `intensity` stays the disc centre's luminance on the scene scale; before that a neutral white.
// World.sun_disc_material sets both uniforms: the physical centre (World.sun_disc_scene_value, 8.67e4).
render_mode unshaded, cull_back;

#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/refraction.gdshaderinc"

uniform float intensity = 1.0;
uniform float limb_darkening = 0.6;

varying vec3 local_normal;

void vertex() {
	local_normal = NORMAL;
	// Drawn at its refracted direction: lifted and, near the horizon, flattened.
	vec3 world = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	VERTEX = (inverse(MODEL_MATRIX) * vec4(refracted_world_point(world, CAMERA_POSITION_WORLD), 1.0)).xyz;
}

void fragment() {
	float mu = clamp(dot(normalize(NORMAL), vec3(0.0, 0.0, 1.0)), 0.0, 1.0);  // view space: towards camera
	float limb = 1.0 - limb_darkening * (1.0 - mu);
	vec3 color = vec3(limb * intensity);
	if (atmosphere_enabled) {
		vec3 solar = atmosphere_sun_illuminance_lux;
		color *= solar / max(dot(solar, vec3(0.2126, 0.7152, 0.0722)), 1.0e-30);  // [S: ITU-R BT.709] luminance weights
		vec3 direction_world = normalize((INV_VIEW_MATRIX * vec4(VERTEX, 0.0)).xyz);
		vec3 transmittance;
		vec3 sky = atm_sky(atm_camera_local(), atmosphere_world_to_earth * direction_world, transmittance);
		color = color * atmosphere_exposure_gain * transmittance + sky;  // the eye adaptation's exposure gain
		// [call] Inside half-float range (the scene buffer; the Sun appearing in a dark-adapted view: cone gain up to
		// ~1.2e4), scaled as a whole so the colour stays: the physical centre exceeds it where T x the gain > 0.69 (a
		// high Sun, above the haze, in space - a known limit of the half-float scene buffer).
		color *= min(1.0, 6.0e4 / max(color.r, max(color.g, color.b)));
	}
	ALBEDO = color;
}
