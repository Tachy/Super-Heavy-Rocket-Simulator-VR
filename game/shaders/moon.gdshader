shader_type spatial;
// Moon: Lommel-Seeliger reflectance (dust-covered regolith: flat-looking full Moon, correct phase
// brightness), Earth shadow (lunar eclipses) and faint earthshine; seen through the Earth's atmosphere (opaque: color x
// T + the sky in front of it - the pale daytime Moon), nothing added when the camera is outside the atmosphere and the
// Moon lies before the ray's entry.
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform vec3 sun_position_local;    // from the Moon's centre
uniform vec3 earth_position_local;  // from the Moon's centre
uniform vec3 camera_position_local;
uniform float sun_radius_m = 695700000.0;
uniform float earth_radius_m = 6371000.0;
uniform float earthshine = 0.004;
uniform float exposure = 1.6;
// [E] Sunlight refracted by the Earth's atmosphere into the umbra (reddened). Approximation of a
// Danjon L~2-3 eclipse at this fixed exposure; a physical model belongs to the atmosphere work (M8).
uniform vec3 umbra_glow = vec3(0.020, 0.005, 0.0015);

varying vec3 local_position;
varying vec3 local_normal;

void vertex() {
	local_position = VERTEX;
	local_normal = NORMAL;
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = sun_position_local - local_position;
	vec3 to_earth = earth_position_local - local_position;
	float mu0 = max(dot(n, normalize(to_sun)), 0.0);
	float mu = max(dot(n, normalize(camera_position_local - local_position)), 1e-3);
	float lommel_seeliger = 2.0 * mu0 / (mu0 + mu);
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, to_earth, earth_radius_m);
	float earth_light = earthshine * max(dot(n, normalize(to_earth)), 0.0);
	vec3 albedo = texture(albedo_texture, UV).rgb;
	vec3 eclipse_light = umbra_glow * (1.0 - sunlit) * lommel_seeliger;
	vec3 color = albedo * (lommel_seeliger * sunlit * exposure * 0.5 + earth_light + eclipse_light);
	if (atmosphere_enabled) {
		// The view ray from the camera (scaled space draws the Moon closer; its true distance comes from the Moon-local
		// camera, in true metres).
		vec3 direction_world = normalize((INV_VIEW_MATRIX * vec4(VERTEX, 0.0)).xyz);
		vec3 transmittance;
		vec3 sky = atm_sky_to_far_object(atm_camera_local(), atmosphere_world_to_earth * direction_world,
				length(local_position - camera_position_local), transmittance);
		color = color * atmosphere_exposure_gain * transmittance + sky;  // the eye adaptation's exposure gain on the lit Moon
	}
	ALBEDO = color;
}
