shader_type spatial;
// Star points: vertex colour = B-V colour, CUSTOM0.x = the star's illuminance above the atmosphere (lux, StarField).
// Above the atmosphere a star renders at COLOR x clamp(E / reference_illuminance_lux, 0.04, 1) (the look before the
// atmosphere); through it the star is attenuated by the view ray's transmittance and hidden when its illuminance at
// the eye falls below the naked-eye threshold over the background (star_visibility.gdshaderinc): the larger of the sky
// behind the star and the luminance the eye is adapted to (atmosphere_adaptation_cd_m2, final review I1) - stars fade
// out in the day and twilight sky, stay hidden beside a sunlit Earth or with the Sun on the eye in space, and appear
// above the atmosphere once the eye has dark-adapted (night side, Earth's shadow). Evaluated per star (vertex).
render_mode unshaded, cull_disabled, depth_draw_never, blend_add;

#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/star_visibility.gdshaderinc"

uniform float point_scale = 2.4;
// Illuminance rendered at full brightness (StarField.REFERENCE_MAGNITUDE).
uniform float reference_illuminance_lux = 1.0;

varying vec3 star_light;

void vertex() {
	float illuminance = CUSTOM0.x;
	float relative = min(illuminance / reference_illuminance_lux, 1.0);
	vec3 attenuation = vec3(1.0);
	if (atmosphere_enabled) {
		vec3 direction_world = normalize((MODEL_MATRIX * vec4(VERTEX, 0.0)).xyz);
		vec3 transmittance;
		vec3 sky = atm_sky(atm_camera_local(), atmosphere_world_to_earth * direction_world, transmittance);
		// Schaefer's B is the luminance the eye is adapted to; in a non-uniform field the adaptation luminance dominates
		// over a dark patch of sky (Ferwerda et al. 1996, threshold-versus-intensity of the adapted eye).
		float background_cd_m2 = max(dot(sky, LUMINANCE_WEIGHTS) / atmosphere_scene_per_cd_m2,
				atmosphere_adaptation_cd_m2);
		float at_eye_lux = illuminance * dot(transmittance, LUMINANCE_WEIGHTS);
		attenuation = transmittance * star_visibility(at_eye_lux, background_cd_m2);
	}
	star_light = COLOR.rgb * clamp(relative, 0.04, 1.0) * attenuation;
	float fade = dot(attenuation, LUMINANCE_WEIGHTS);
	POINT_SIZE = clamp(point_scale * sqrt(relative * fade) * 2.0, 1.0, 5.0);
}

void fragment() {
	ALBEDO = star_light;
}
