shader_type spatial;
// Earth terrain node (TerrainQuadtree): the lighting of earth.gdshader (day texture lit by the Sun, city lights on the
// night side, Moon shadow) on the relief normal, with geomorphing and the imagery crops blended over the day texture.
// Positions are in the body's local render frame (metres, unscaled): vertex_offset restores them from the node centre.
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/terrain_morph.gdshaderinc"

// Global maps: CUSTOM1.u runs past 1 on nodes across the antimeridian (continuous inside the node), hence repeat.
uniform sampler2D day_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D night_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
// Per-node imagery crops: vertices outside a crop have UVs outside [0, 1] and fade 0, hence clamp.
uniform sampler2D imagery_fine : source_color, filter_linear_mipmap_anisotropic, repeat_disable, hint_default_black;
uniform sampler2D imagery_base : source_color, filter_linear_mipmap_anisotropic, repeat_disable, hint_default_black;
// Per-node water mask (255 = water) over the node's lat/lon box; shaded by the sea model (Task 7).
uniform sampler2D water_mask : filter_linear, repeat_disable, hint_default_black;
uniform vec3 sun_position_local;   // apparent, from the Earth's centre
uniform vec3 moon_position_local;  // apparent, from the Earth's centre
uniform float sun_radius_m = 695700000.0;
uniform float moon_radius_m = 1737400.0;
uniform float night_lights_intensity = 1.2;
uniform float day_exposure = 1.5;
// The node centre in body-local render metres (vertices are stored relative to it).
instance uniform vec3 vertex_offset = vec3(0.0);

varying vec3 local_position;
varying vec3 local_normal;
varying vec4 blend;      // global u, v, fade_fine, fade_base
varying vec2 water_uv;

void vertex() {
	float morph = terrain_morph_factor(VERTEX);
	VERTEX += CUSTOM0.xyz * morph;
	NORMAL = normalize(mix(NORMAL, CUSTOM3.xyz, morph));
	local_position = VERTEX + vertex_offset;
	local_normal = NORMAL;
	blend = CUSTOM1;
	water_uv = clamp(CUSTOM2.xy, 0.0, 1.0);
}

void fragment() {
	vec3 n = normalize(local_normal);
	vec3 to_sun = sun_position_local - local_position;
	float cos_sun = dot(n, normalize(to_sun));
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, moon_position_local - local_position, moon_radius_m);

	// v stays half a texel inside the map (repeat would bleed the opposite pole into the polar rows).
	float half_texel_v = 0.5 / float(textureSize(day_texture, 0).y);
	vec2 global_uv = vec2(blend.x, clamp(blend.y, half_texel_v, 1.0 - half_texel_v));
	vec3 albedo = texture(day_texture, global_uv).rgb;
	albedo = mix(albedo, texture(imagery_base, UV2).rgb, blend.w);
	albedo = mix(albedo, texture(imagery_fine, UV).rgb, blend.z);

	vec3 day = albedo * max(cos_sun, 0.0) * sunlit * day_exposure;
	// City lights fade in through civil twilight (sun 0..6 deg below the horizon).
	float night_factor = smoothstep(0.0, -0.1, cos_sun);
	vec3 night = texture(night_texture, global_uv).rgb;
	night = night * night * night_lights_intensity * night_factor;  // suppress dark-sky noise
	ALBEDO = day + night;
}
