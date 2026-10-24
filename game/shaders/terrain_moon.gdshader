shader_type spatial;
// Moon terrain node (TerrainQuadtree): the model of moon.gdshader (Lommel-Seeliger regolith, Earth shadow with the
// reddened umbra glow, faint earthshine) on the relief normal, with geomorphing. Albedo: the global colour map
// (moon_color, equirectangular) through CUSTOM1's global UV; the Moon has no imagery crops and no water.
// Cast shadows (crater rims and mountains shadowing the ground, the long polar shadows) are not modelled: they belong to
// a later part; a slope facing away from the Sun is dark by its own normal only.
// Positions are in the Moon's local render frame (metres, unscaled): vertex_offset restores them from the node centre.
// Seen through the Earth's atmosphere like moon.gdshader (a lunar ground view never crosses it).
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/terrain_morph.gdshaderinc"

// CUSTOM1.u runs past 1 on nodes across the antimeridian (continuous inside the node), hence repeat.
uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
global uniform vec3 moon_sun_position_local;    // apparent, from the Moon's centre (World)
global uniform vec3 moon_earth_position_local;  // from the Moon's centre (World)
// The camera's Moon-local render position, float32 hi + lo parts (TerrainQuadtree.camera_global, once per frame).
global uniform vec3 moon_terrain_camera_hi;
global uniform vec3 moon_terrain_camera_lo;
uniform float sun_radius_m = 695700000.0;
uniform float earth_radius_m = 6371000.0;
// The constants of moon.gdshader (same look as the body mesh from scaled space).
uniform float earthshine = 0.004;
uniform float exposure = 1.6;
uniform vec3 umbra_glow = vec3(0.020, 0.005, 0.0015);

varying vec3 local_position;
varying vec3 local_normal;
varying vec2 global_uv;  // equirectangular u (continuous in the node), v
varying vec3 camera_offset;  // fragment - camera (node-local render axes, metres; exact near the camera)

void vertex() {
	vec3 camera_local = terrain_camera_local(moon_terrain_camera_hi, moon_terrain_camera_lo);
	float morph = terrain_morph_factor(VERTEX, camera_local);
	VERTEX += CUSTOM0.xyz * morph;
	NORMAL = normalize(mix(NORMAL, CUSTOM3.xyz, morph));
	local_position = VERTEX + vertex_offset;
	local_normal = NORMAL;
	global_uv = CUSTOM1.xy;
	camera_offset = VERTEX - camera_local;
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = moon_sun_position_local - local_position;
	vec3 to_earth = moon_earth_position_local - local_position;
	float mu0 = max(dot(n, normalize(to_sun)), 0.0);
	float mu = max(dot(n, normalize(-camera_offset)), 1e-3);
	float lommel_seeliger = 2.0 * mu0 / (mu0 + mu);
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, to_earth, earth_radius_m);
	float earth_light = earthshine * max(dot(n, normalize(to_earth)), 0.0);
	// v stays half a texel inside the map (repeat would bleed the opposite pole into the polar rows).
	float half_texel_v = 0.5 / float(textureSize(albedo_texture, 0).y);
	vec3 albedo = texture(albedo_texture, vec2(global_uv.x, clamp(global_uv.y, half_texel_v, 1.0 - half_texel_v))).rgb;
	vec3 eclipse_light = umbra_glow * (1.0 - sunlit) * lommel_seeliger;
	vec3 color = albedo * (lommel_seeliger * sunlit * exposure * 0.5 + earth_light + eclipse_light);
	if (atmosphere_enabled) {
		// Through the Earth's atmosphere only if the view ray crosses it before the ground (a camera near the Moon:
		// never; atm_sky_to_far_object).
		vec3 direction_world = normalize((INV_VIEW_MATRIX * vec4(VERTEX, 0.0)).xyz);
		vec3 transmittance;
		vec3 sky = atm_sky_to_far_object(atm_camera_local(), atmosphere_world_to_earth * direction_world,
				length(camera_offset), transmittance);
		color = color * transmittance + sky;
	}
	ALBEDO = color;
}
