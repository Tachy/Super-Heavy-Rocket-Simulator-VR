shader_type spatial;
// Earth: day texture lit by the Sun transmitted through the atmosphere plus the sky light, city lights on the night
// side, Moon shadow (solar eclipses); seen through the atmosphere (aerial perspective: color x T + in-scattering,
// atmosphere.gdshaderinc). All positions are in the body's local render frame (metres, unscaled), so floating origin
// and scaled-space rendering do not affect the lighting; the camera is the true Earth-local camera of the atmosphere
// globals (not the scaled mesh's position), so the hand-over to the terrain quadtree shows the same atmosphere.
// Until the atmosphere tables are loaded (atmosphere_enabled false): the vacuum lighting albedo x cos x day_exposure.
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"

uniform sampler2D day_texture : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform sampler2D night_texture : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform vec3 sun_position_local;   // apparent, from the Earth's centre
uniform vec3 moon_position_local;  // apparent, from the Earth's centre
uniform float sun_radius_m = 695700000.0;
uniform float moon_radius_m = 1737400.0;
uniform float night_lights_intensity = 1.2;
uniform float day_exposure;  // set from Atmosphere.DAY_EXPOSURE (world.gd): the one exposure of every surface
// Ground patches store vertices relative to their centre; this restores body-local metres.
instance uniform vec3 vertex_offset = vec3(0.0);

varying vec3 local_position;
varying vec3 local_normal;
varying vec3 sky_eclipse_factor;  // atm_sky_eclipse_factor per vertex (it varies over tens of km)

void vertex() {
	local_position = VERTEX + vertex_offset;
	local_normal = NORMAL;
	sky_eclipse_factor = atmosphere_enabled ? atm_sky_eclipse_factor(local_position) : vec3(1.0);
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = sun_position_local - local_position;
	float cos_sun = dot(n, normalize(to_sun));
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, moon_position_local - local_position, moon_radius_m);
	vec3 albedo = texture(day_texture, UV).rgb;
	// City lights fade in through civil twilight (sun 0..6 deg below the horizon).
	float night_factor = smoothstep(0.0, -0.1, cos_sun);
	vec3 night = texture(night_texture, UV).rgb;
	night = night * night * night_lights_intensity * night_factor;  // suppress dark-sky noise

	if (atmosphere_enabled) {
		// The direct Sun x the local sunlit fraction; the sky light with the eclipse sky's factor (the dark sky inside
		// the Moon's shadow, the lit air around it: atm_sky_eclipse_factor).
		vec3 sky_light;
		vec3 sun_light = atm_sun_and_sky(local_position, n, sky_light);
		vec3 day = albedo * (sun_light * sunlit + sky_light * sky_eclipse_factor);
		// Camera-relative offset: the mesh is far from the camera whenever it is drawn (the terrain takes over near
		// the ground), so the difference of the absolute float32 positions is accurate enough here.
		vec3 offset = (local_position - atmosphere_camera_hi) - atmosphere_camera_lo;
		vec3 transmittance;
		vec3 in_scatter = atm_to_point_offset(offset, transmittance);
		ALBEDO = (day + night) * transmittance + in_scatter;
	} else {
		ALBEDO = albedo * max(cos_sun, 0.0) * sunlit * day_exposure + night;
	}
}
