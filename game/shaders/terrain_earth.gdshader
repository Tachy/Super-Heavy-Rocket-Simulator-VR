shader_type spatial;
// Earth terrain node (TerrainQuadtree): the lighting of earth.gdshader (day texture lit by the Sun transmitted through
// the atmosphere and the sky light, city lights on the night side, Moon shadow, aerial perspective) on the relief
// normal, with geomorphing and the imagery crops blended over the day texture. Where the node's water mask says sea,
// the calm sea model (water.gdshaderinc) shades it instead, reflecting the atmosphere's sky.
// Clouds (Task 9, cloud_light.gdshaderinc): the direct Sun through the cloud shadow maps, the Sun's light diffused
// through the column (isotropic from above) added to the sky light, the sky light x the column's diffuse transmittance;
// the Moon likewise through its column. Only while cloud_enabled: weather off leaves every pixel as before (the
// CLOUD_LIGHT_BEGIN / _END blocks; cloud_check.gd strips them and compares bit for bit).
// Positions are in the body's local render frame (metres, unscaled): vertex_offset restores them from the node centre;
// the atmosphere takes the fragment's camera-relative offset (camera_offset, exact near the camera), never the
// difference of two absolute float32 positions.
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/terrain_morph.gdshaderinc"
#include "res://shaders/water.gdshaderinc"
// CLOUD_LIGHT_BEGIN
#include "res://shaders/clouds.gdshaderinc"
#include "res://shaders/cloud_light.gdshaderinc"
// CLOUD_LIGHT_END

// Global maps: CUSTOM1.u runs past 1 on nodes across the antimeridian (continuous inside the node), hence repeat.
uniform sampler2D day_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
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
// City lights before the atmosphere tables are loaded (display value of a white texel); with them: scene-referred.
uniform float day_exposure;  // set from Atmosphere.DAY_EXPOSURE (world.gd): the one exposure of every surface

// Launch-mount mask (TowerView._set_trench_mask, Pad B): inside the footprint of the mount and its two exit ramps the
// terrain is not drawn (the model's own pit floor lies there). Site position as hi / lo parts like the camera, so the
// fragment's offset from the site stays exact near it; only within trench_range_m of the camera (the model is drawn
// 150 m beyond that). trench_shape = (half extent along the mount-tower axis, half extent across, ramp half width, ramp
// end across); trench_range_m 0 = off (no Pad B models).
global uniform vec3 trench_site_hi;
global uniform vec3 trench_site_lo;
global uniform vec3 trench_axis_t;
global uniform vec3 trench_axis_a;
global uniform vec4 trench_shape;
global uniform float trench_range_m;

varying vec3 trench_rel;  // vertex - site (node-local axes, metres)
varying vec3 local_position;
varying vec3 local_normal;
varying vec2 global_uv;  // equirectangular u (continuous in the node), v
varying float camera_distance;
varying vec3 camera_offset;  // fragment - camera (node-local render axes, metres; exact near the camera)
varying vec2 water_uv;
// The eclipse sky's factor on the sky light (atm_sky_eclipse_factor), per vertex: it varies over tens of km, and per
// fragment it would cost ~20 table lookups during an eclipse.
varying vec3 sky_eclipse_factor;

void vertex() {
	vec3 camera_local = terrain_camera_local(earth_terrain_camera_hi, earth_terrain_camera_lo);
	float morph = terrain_morph_factor(VERTEX, camera_local);
	VERTEX += CUSTOM0.xyz * morph;
	NORMAL = normalize(mix(NORMAL, CUSTOM3.xyz, morph));
	local_position = VERTEX + vertex_offset;
	trench_rel = VERTEX - ((trench_site_hi - vertex_offset) + (trench_site_lo - vertex_offset_lo));
	local_normal = NORMAL;
	sky_eclipse_factor = atmosphere_enabled ? atm_sky_eclipse_factor(local_position) : vec3(1.0);
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
	if (trench_range_m > 0.0 && camera_distance < trench_range_m && dot(trench_rel, trench_rel) < 10000.0) {
		float along = abs(dot(trench_rel, trench_axis_t));
		float across = abs(dot(trench_rel, trench_axis_a));
		if ((along < trench_shape.x && across < trench_shape.y) || (along < trench_shape.z && across < trench_shape.w)) {
			discard;
		}
	}
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

	// Light: the Sun transmitted to the point x the local sunlit fraction, and the sky light with the eclipse sky's
	// factor (atmosphere.gdshaderinc: atm_sky_eclipse_factor, per vertex); before the tables are loaded the vacuum Sun at
	// day_exposure and no sky.
	vec3 sun_normal = vec3(day_exposure);
	vec3 sky_light = vec3(0.0);
	if (atmosphere_enabled) {
		sun_normal = atm_sun_at(local_position);
		sky_light = atm_sky_light(local_position, n) * sky_eclipse_factor;
	}
	// CLOUD_LIGHT_BEGIN
	// x direct, y the Sun's diffuse share, z the sky's, w mu0 (cloud_ground_light); the Sun's diffuse light under the
	// clouds as an isotropic downward radiance (scene units: sun_normal is the normal-incidence irradiance / pi).
	vec4 cloud = vec4(1.0, 0.0, 1.0, 0.0);
	vec3 cloud_sun_radiance = vec3(0.0);
	bool clouded = cloud_enabled && atmosphere_enabled;
	if (clouded) {
		cloud = cloud_ground_light(camera_offset, atmosphere_sun_direction, true);
		cloud_sun_radiance = sun_normal * (max(cloud.w, 0.0) * cloud.y * sunlit);
		float upper = 0.5 * (1.0 + dot(n, cloud_geodetic_up(local_position)));  // the upper sky's view factor
		sky_light = sky_light * cloud.z + cloud_sun_radiance * upper;
		sun_normal *= cloud.x;
	}
	// CLOUD_LIGHT_END
	vec3 day = albedo * (sun_normal * max(dot(n, normalize(earth_sun_position_local)), 0.0) * sunlit + sky_light);
	// Moonlight (night only): the direct Moon and the moonlit sky's light; on the sea a second pass of the sea model.
	bool moonlit = atmosphere_enabled && atm_moonlit();
	vec3 moon_normal = vec3(0.0);
	vec3 moon_sky_light = vec3(0.0);
	if (moonlit) {
		moon_normal = atm_moon_at(local_position);
		moon_sky_light = atm_moon_sky_light(local_position, n);
		// CLOUD_LIGHT_BEGIN
		if (clouded) {
			vec4 moon_cloud = cloud_ground_light(camera_offset, normalize(atmosphere_moon_direction), false);
			float upper = 0.5 * (1.0 + dot(n, cloud_geodetic_up(local_position)));
			moon_sky_light = moon_sky_light * moon_cloud.z
					+ moon_normal * (max(moon_cloud.w, 0.0) * moon_cloud.y * upper);
			moon_normal *= moon_cloud.x;
		}
		// CLOUD_LIGHT_END
		day += albedo * (moon_normal * max(dot(n, normalize(atmosphere_moon_direction)), 0.0) + moon_sky_light);
	}
	// Sea: the imagery where it covers (lagoons, shoals), else the day texture (shallow banks from orbit; deep_water
	// without one), under the calm sea model.
	float water = water_coverage(texture(water_mask, water_uv).r);
	vec3 sea_background = day_texture_bound ? texture(day_texture, map_uv).rgb : deep_water;
	vec3 water_body = mix(mix(sea_background, base, base_fade), fine, fine_fade);
	vec3 to_camera = normalize(-camera_offset);
	float resolved_variance;
	// Outside any branch: the wave octaves' alias fade takes screen-space derivatives.
	vec3 wave_n = water_wave_normal(n, camera_offset, length(camera_offset), resolved_variance);
	vec3 sky_reflected = vec3(0.0);
	if (atmosphere_enabled && water > 0.0) {  // the reflected sky only where there is sea (no derivatives inside)
		vec3 reflected_transmittance;
		sky_reflected = atm_sky_at_offset(camera_offset, water_reflected_direction(n, wave_n, to_camera),
				reflected_transmittance);
	}
	// CLOUD_LIGHT_BEGIN
	if (clouded) {
		sky_reflected = sky_reflected * cloud.z + cloud_sun_radiance;  // the sky seen through the deck + its glow
	}
	// CLOUD_LIGHT_END
	vec3 sea = water_radiance(water_body, n, wave_n, resolved_variance, normalize(to_sun), to_camera, sunlit,
			sun_normal, sky_light, sky_reflected);
	if (moonlit && water > 0.0) {
		// The same sea under the Moon: its diffuse light, the moonlit sky reflected and the Moon's glitter path.
		vec3 moon_reflected = atm_moon_sky_at_offset(camera_offset, water_reflected_direction(n, wave_n, to_camera));
		sea += water_radiance(water_body, n, wave_n, resolved_variance, normalize(atmosphere_moon_direction),
				to_camera, 1.0, moon_normal, moon_sky_light, moon_reflected);
	}
	day = mix(day, sea, water);
	// Night is the lighting alone (user 2026-10-29: no separate night texture / city lights): the Moon, the sky.
	vec3 color = day;
	if (atmosphere_enabled) {
		// Aerial perspective along the camera-relative offset (exact in float32 near the camera).
		vec3 transmittance;
		vec3 in_scatter = atm_to_point_offset(camera_offset, transmittance);
		color = day * transmittance + in_scatter;
	}
	ALBEDO = color;
	// CLOUD_LIGHT_BEGIN
	if (clouded && cloud_debug_output == 5) {
		ALBEDO = cloud.xyz;  // checks (cloud_check.gd): the cloud light factors themselves
	}
	// CLOUD_LIGHT_END
}
