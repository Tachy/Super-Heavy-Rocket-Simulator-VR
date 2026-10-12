shader_type spatial;
// Earth: day texture lit by the Sun, city lights on the night side, Moon shadow (solar eclipses).
// All positions are in the body's local render frame (metres, unscaled), so floating origin and
// scaled-space rendering do not affect the lighting.
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"

uniform sampler2D day_texture : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform sampler2D night_texture : source_color, filter_linear_mipmap_anisotropic, repeat_disable;
uniform vec3 sun_position_local;   // apparent, from the Earth's centre
uniform vec3 moon_position_local;  // apparent, from the Earth's centre
uniform float sun_radius_m = 695700000.0;
uniform float moon_radius_m = 1737400.0;
uniform float night_lights_intensity = 1.2;
uniform float day_exposure = 1.5;
// Ground patches store vertices relative to their centre; this restores body-local metres.
instance uniform vec3 vertex_offset = vec3(0.0);

varying vec3 local_position;
varying vec3 local_normal;

void vertex() {
	local_position = VERTEX + vertex_offset;
	local_normal = NORMAL;
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = sun_position_local - local_position;
	float cos_sun = dot(n, normalize(to_sun));
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, moon_position_local - local_position, moon_radius_m);

	vec3 day = texture(day_texture, UV).rgb * max(cos_sun, 0.0) * sunlit * day_exposure;
	// City lights fade in through civil twilight (sun 0..6 deg below the horizon).
	float night_factor = smoothstep(0.0, -0.1, cos_sun);
	vec3 night = texture(night_texture, UV).rgb;
	night = night * night * night_lights_intensity * night_factor;  // suppress dark-sky noise
	ALBEDO = day + night;
}
