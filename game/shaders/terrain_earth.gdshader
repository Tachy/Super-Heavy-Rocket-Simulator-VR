shader_type spatial;
// Earth terrain node (TerrainQuadtree): the lighting of earth.gdshader (day texture lit by the Sun, city lights on the
// night side, Moon shadow) on the relief normal, with geomorphing and the imagery crops blended over the day texture.
// Where the node's water mask says sea, the calm sea model (water.gdshaderinc) shades it instead.
// Positions are in the body's local render frame (metres, unscaled): vertex_offset restores them from the node centre.
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/terrain_morph.gdshaderinc"
#include "res://shaders/water.gdshaderinc"

// Global maps: CUSTOM1.u runs past 1 on nodes across the antimeridian (continuous inside the node), hence repeat.
uniform sampler2D day_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D night_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform bool day_texture_bound = false;  // set with day_texture (World); false: the sea body falls back to deep_water
// Per-node imagery crops: vertices outside a crop have UVs outside [0, 1] and fade 0, hence clamp.
uniform sampler2D imagery_fine : source_color, filter_linear_mipmap_anisotropic, repeat_disable, hint_default_black;
uniform sampler2D imagery_base : source_color, filter_linear_mipmap_anisotropic, repeat_disable, hint_default_black;
// Coverage boxes of the crops' sources (west, south, east, north; deg, in the node's longitude frame, the frame of
// CUSTOM1.u) and their fade widths (m); the default box is empty (fade 0).
uniform vec4 fine_coverage_deg = vec4(0.0, 0.0, -1.0, -1.0);
uniform vec4 base_coverage_deg = vec4(0.0, 0.0, -1.0, -1.0);
uniform float fine_fade_m = 1.0;
uniform float base_fade_m = 1.0;
uniform float metres_per_deg = 111319.49;  // body radius * pi / 180, set by the quadtree
// [E] The imagery fade is at least this angle (rad) seen from the camera, so a source's coverage edge never reads as a
// line from far away (400 km: 20 km wide); near the ground the source's own fade_m rules.
uniform float imagery_fade_angle = 0.05;
// Per-node water mask (255 = water) over the node's lat/lon box; shaded by the sea model (water.gdshaderinc).
uniform sampler2D water_mask : filter_linear, repeat_disable, hint_default_black;
global uniform vec3 earth_sun_position_local;   // apparent, from the Earth's centre (World)
global uniform vec3 earth_moon_position_local;  // apparent, from the Earth's centre (World)
// The camera's Earth-local render position, float32 hi + lo parts (TerrainQuadtree.camera_global, once per frame).
global uniform vec3 earth_terrain_camera_hi;
global uniform vec3 earth_terrain_camera_lo;
uniform float sun_radius_m = 695700000.0;
uniform float moon_radius_m = 1737400.0;
uniform float night_lights_intensity = 1.2;
uniform float day_exposure = 1.5;

varying vec3 local_position;
varying vec3 local_normal;
varying vec2 global_uv;  // equirectangular u (continuous in the node), v
varying float camera_distance;
varying vec3 camera_offset;  // fragment - camera (node-local render axes, metres; exact near the camera)
varying vec2 water_uv;

void vertex() {
	vec3 camera_local = terrain_camera_local(earth_terrain_camera_hi, earth_terrain_camera_lo);
	float morph = terrain_morph_factor(VERTEX, camera_local);
	VERTEX += CUSTOM0.xyz * morph;
	NORMAL = normalize(mix(NORMAL, CUSTOM3.xyz, morph));
	local_position = VERTEX + vertex_offset;
	local_normal = NORMAL;
	global_uv = CUSTOM1.xy;
	camera_offset = VERTEX - camera_local;
	camera_distance = length(camera_offset);
	water_uv = clamp(CUSTOM2.xy, 0.0, 1.0);
}

// 1 inside a coverage box, falling linearly to 0 over width_m towards its edge (per fragment: sharp at any node size).
float coverage_fade(vec4 box_deg, float lat, float lon, float cos_lat, float width_m) {
	float inside_deg = min(min(lat - box_deg.y, box_deg.w - lat), min(lon - box_deg.x, box_deg.z - lon) * cos_lat);
	return clamp(inside_deg * metres_per_deg / width_m, 0.0, 1.0);
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = earth_sun_position_local - local_position;
	float cos_sun = dot(n, normalize(to_sun));
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, earth_moon_position_local - local_position,
			moon_radius_m);

	// v stays half a texel inside the map (repeat would bleed the opposite pole into the polar rows).
	float half_texel_v = 0.5 / float(textureSize(day_texture, 0).y);
	vec2 map_uv = vec2(global_uv.x, clamp(global_uv.y, half_texel_v, 1.0 - half_texel_v));
	float lon = global_uv.x * 360.0 - 180.0;
	float lat = 90.0 - global_uv.y * 180.0;
	float cos_lat = cos(radians(lat));
	float min_width_m = camera_distance * imagery_fade_angle;
	float base_fade = coverage_fade(base_coverage_deg, lat, lon, cos_lat, max(base_fade_m, min_width_m));
	float fine_fade = coverage_fade(fine_coverage_deg, lat, lon, cos_lat, max(fine_fade_m, min_width_m));
	vec3 base = texture(imagery_base, UV2).rgb;
	vec3 fine = texture(imagery_fine, UV).rgb;
	vec3 albedo = mix(mix(texture(day_texture, map_uv).rgb, base, base_fade), fine, fine_fade);

	vec3 day = albedo * max(cos_sun, 0.0) * sunlit * day_exposure;
	// Sea: the imagery where it covers (lagoons, shoals), else the day texture (shallow banks from orbit; deep_water
	// without one), under the calm sea model.
	float water = water_coverage(texture(water_mask, water_uv).r);
	vec3 sea_background = day_texture_bound ? texture(day_texture, map_uv).rgb : deep_water;
	vec3 water_body = mix(mix(sea_background, base, base_fade), fine, fine_fade);
	vec3 to_camera = normalize(-camera_offset);
	float resolved_variance;
	vec3 wave_n = water_wave_normal(n, camera_offset, length(camera_offset), resolved_variance);
	day = mix(day, water_radiance(water_body, n, wave_n, resolved_variance, normalize(to_sun), to_camera, sunlit,
			day_exposure), water);
	// City lights fade in through civil twilight (sun 0..6 deg below the horizon).
	float night_factor = smoothstep(0.0, -0.1, cos_sun);
	vec3 night = texture(night_texture, map_uv).rgb;
	night = night * night * night_lights_intensity * night_factor;  // suppress dark-sky noise
	ALBEDO = day + night;
}
