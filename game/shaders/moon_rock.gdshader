shader_type spatial;
// Drawn lunar rocks (moon procedural surface plan Task 11; MoonRockLayer): one MultiMesh per terrain node and template
// LOD, instances = the core's rocks of the node (StarsimCore.moon_rocks_in_node). The mesh is the unit icosphere of the
// LOD (vertex = direction); each vertex takes its template's radius r(dir) and normal from rock_template_data (row
// lod x 32 + template, column VERTEX_ID), so all 32 templates share one mesh per LOD and the instance transform (the
// core's basis x semi-axes, the centre burial_m below the base point, relative to the node centre) places the rock.
//
// Drawn by projected size, from one reference point (the camera; in VR the head centre: MODELVIEW_MATRIX is the
// centre view's): d = the threshold diameter at the rock centre's distance (detail_rock_threshold_m: ROCK_PIXEL_THRESHOLD
// x rock_threshold_scale pixels x the abundance factor, never below the node's list limit); a rock with D < d is not drawn (it is in the
// ground's statistical layer), from d to 2 d it rises out of the ground (its exposed height grows as log2(D / d): the
// statistical weight 1 - log2(D / d) mirrors it, no rock counted twice), above 2 d it stands at its nominal burial.
// The LOD MultiMesh draws an instance only inside its band of projected diameter (CUSTOM0), so each rock is drawn by
// exactly one LOD. Instance data: transform; colour = (grid a, grid b: the base in the node's vertex grid, the horizon
// map's coordinate; global u, v: the albedo map's); custom = (D, nominal exposed height in template z units (top_z -
// burial / c), albedo factor, template id).
// Light (moon_rock_light.gdshaderinc): Hapke rock law, the node's horizon map at the base for the Sun and the
// earthshine, the Earth's shadow on the Sun, Smith's G1 of the node's drawn relief; the rock textures (triplanar in the
// rock's own metres) as albedo variation and mean-preserving slope modulation.
render_mode unshaded, cull_back;

#include "res://shaders/eclipse.gdshaderinc"
#include "res://shaders/moon_regolith.gdshaderinc"
#include "res://shaders/moon_detail.gdshaderinc"
#include "res://shaders/moon_horizon.gdshaderinc"
#include "res://shaders/moon_rock_light.gdshaderinc"

// MoonRockLayer.TEMPLATES (core RockTemplates::count).
const int ROCK_TEMPLATES = 32;

global uniform vec3 moon_sun_position_local;
global uniform vec3 moon_earth_position_local;
// The Moon's local render frame from the world frame (MoonRockLayer.update, per frame): normals and light directions.
global uniform mat3 moon_world_to_local;

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
// RGBA float: (normal xyz, radius r) of template t's vertex k at LOD l in texel (k, l x ROCK_TEMPLATES + t).
uniform sampler2D rock_template_data : filter_nearest, repeat_disable;
uniform float rock_top_z[32];  // per template: the highest vertex z (unit template; LOD 0)
// The baked rock textures (data/textures/moon_detail): relative albedo (value x 255 / 128, mean 1) and tangent-space
// normals, one layer per variant (template % layers), tile rock_texture_tile_m.
uniform sampler2DArray rock_albedo_maps : filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2DArray rock_normal_maps : filter_linear_mipmap_anisotropic, repeat_enable;
uniform bool rock_textures = false;
uniform float rock_texture_tile_m = 1.0;
uniform int rock_texture_layers = 4;
uniform float sun_radius_m;
uniform float earth_radius_m;
// Per node: its centre in the Moon's local render frame (float32 is enough: light directions only). Per LOD mesh,
// constant over its vertices, CUSTOM0 = (LOD row, band low px, band high px, 0): no instance uniforms (their global
// buffer would overflow with thousands of nodes x LODs).
uniform vec3 node_center_local = vec3(0.0);

varying vec3 rock_normal;  // Moon local render frame
varying vec3 rock_up;  // the rock's vertical (its c axis), local frame
varying vec3 rock_metres;  // rock-local position (template axes, metres): the triplanar coordinate
varying vec3 rock_model_normal;  // template-space normal: the triplanar weights
varying float height_above_ground_m;
varying flat vec4 rock_custom;  // INSTANCE_CUSTOM
varying flat vec4 rock_place;  // COLOR (grid a, grid b, global u, v)

void vertex() {
	rock_custom = INSTANCE_CUSTOM;
	rock_place = COLOR;
	float diameter = INSTANCE_CUSTOM.x;
	float distance_m = length(MODELVIEW_MATRIX[3].xyz);
	float pixels = diameter / max(distance_m * moon_rock_pixel_angle, 1.0e-30);
	float abundance_here = textureLod(detail_rock_abundance, COLOR.zw, 0.0).r;
	float threshold = detail_rock_threshold_m(distance_m, rock_node_min_m, abundance_here);
	float growth = clamp(log2(diameter / threshold), 0.0, 1.0);
	int rock_lod = int(CUSTOM0.x + 0.5);
	// The test override (rock patch) draws whatever LOD it is given.
	bool band = rock_geometry_override_m > 0.0 || (pixels >= CUSTOM0.y && pixels < CUSTOM0.z);
	if (growth <= 0.0 || !band) {
		VERTEX = vec3(0.0);  // not drawn by this LOD: a degenerate triangle
		rock_normal = vec3(0.0, 0.0, 1.0);
		rock_up = vec3(0.0, 0.0, 1.0);
		rock_metres = vec3(0.0);
		rock_model_normal = vec3(0.0, 0.0, 1.0);
		height_above_ground_m = 0.0;
	} else {
		int template_id = clamp(int(INSTANCE_CUSTOM.w + 0.5), 0, ROCK_TEMPLATES - 1);
		vec4 t = texelFetch(rock_template_data, ivec2(VERTEX_ID, rock_lod * ROCK_TEMPLATES + template_id), 0);
		vec3 p = VERTEX * t.w;
		float exposed = INSTANCE_CUSTOM.y;  // top_z - burial / c: the nominal exposed height (template z units)
		float ground_z = rock_top_z[template_id] - exposed;
		p.z -= (1.0 - growth) * exposed;  // fully buried at the band's start, nominal at its end
		VERTEX = p;
		// The instance basis is rotation x diag(a, b, c) (rigid node and body transforms): the normal transforms with
		// the inverse transpose = rotation x diag(1/a, 1/b, 1/c).
		mat3 m = mat3(MODEL_MATRIX);
		vec3 scale = vec3(length(m[0]), length(m[1]), length(m[2]));
		rock_normal = normalize(moon_world_to_local * (m * (t.xyz / (scale * scale))));
		rock_up = normalize(moon_world_to_local * m[2]);
		rock_metres = p * scale;
		rock_model_normal = t.xyz;
		height_above_ground_m = (p.z - ground_z) * scale.z;  // the ground plane stays at ground_z (p already sunk)
	}
}

// Triplanar albedo (relative, mean 1) and height gradient (rock-local metres) of the rock textures.
vec4 rock_texture(vec3 position_m, vec3 model_normal, int layer) {
	vec3 w = abs(normalize(model_normal));
	w = w * w * w * w;
	w /= max(w.x + w.y + w.z, 1.0e-6);
	vec3 uvx = vec3(position_m.yz / rock_texture_tile_m, float(layer));
	vec3 uvy = vec3(position_m.zx / rock_texture_tile_m, float(layer));
	vec3 uvz = vec3(position_m.xy / rock_texture_tile_m, float(layer));
	float albedo = (texture(rock_albedo_maps, uvx).r * w.x + texture(rock_albedo_maps, uvy).r * w.y
			+ texture(rock_albedo_maps, uvz).r * w.z) * (255.0 / 128.0);
	// Tangent-space normal (8-bit, 128 = 0) -> height slope along the projection plane's two axes.
	vec3 nx = texture(rock_normal_maps, uvx).rgb * (255.0 / 127.0) - 128.0 / 127.0;
	vec3 ny = texture(rock_normal_maps, uvy).rgb * (255.0 / 127.0) - 128.0 / 127.0;
	vec3 nz = texture(rock_normal_maps, uvz).rgb * (255.0 / 127.0) - 128.0 / 127.0;
	vec2 sx = -nx.xy / max(nx.z, 0.2);
	vec2 sy = -ny.xy / max(ny.z, 0.2);
	vec2 sz = -nz.xy / max(nz.z, 0.2);
	vec3 gradient = w.x * vec3(0.0, sx.x, sx.y) + w.y * vec3(sy.y, 0.0, sy.x) + w.z * vec3(sz.x, sz.y, 0.0);
	return vec4(gradient, albedo);
}

void fragment() {
	vec3 n = normalize(rock_normal);
	vec3 up = normalize(rock_up);
	vec3 to_sun = moon_sun_position_local - node_center_local;
	vec3 to_earth = moon_earth_position_local - node_center_local;
	vec3 to_eye = normalize(moon_world_to_local * (mat3(INV_VIEW_MATRIX) * normalize(-VERTEX)));
	vec2 global_uv = rock_place.zw;
	float abundance = texture(detail_rock_abundance, global_uv).r;
	float diameter = rock_custom.x;
	float fillet = clamp(height_above_ground_m / (ROCK_FILLET_PER_DIAMETER * diameter), 0.0, 1.0);
	fillet = fillet * fillet * (3.0 - 2.0 * fillet);
	// Texture: albedo variation and the slope modulation (the gradient taken to the local frame like a normal).
	vec3 gradient = vec3(0.0);
	float relative_albedo = 1.0;
	if (rock_textures) {
		int layer = int(rock_custom.w + 0.5) % max(rock_texture_layers, 1);
		vec4 tex = rock_texture(rock_metres, rock_model_normal, layer);
		relative_albedo = mix(1.0, tex.w, fillet);
		mat3 m = mat3(MODEL_MATRIX);
		vec3 g = moon_world_to_local * (m * (tex.xyz / vec3(length(m[0]), length(m[1]), length(m[2]))));
		gradient = (g - n * dot(g, n)) * fillet;
	}
	// Terrain horizon at the base (the node's map), Earth shadow, the Sun's macroscopic elevation.
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, to_earth, earth_radius_m);
	float sun_margin = asin(clamp(dot(normalize(to_sun), up), -1.0, 1.0));
	float sun_visible = 1.0;
	float earth_visible = 1.0;
	if (horizon_enabled) {
		vec3 east = vec3(up.z, 0.0, -up.x);
		east = length(east) > POLE_EAST_MIN ? normalize(east) : vec3(0.0, 0.0, 1.0);
		vec3 north = cross(up, east);
		vec2 uv = ((rock_place.xy * horizon_grid_scale + horizon_grid_offset) / horizon_stride + 0.5)
				/ float(textureSize(horizon_map, 0).x);
		sun_visible = terrain_visible_fraction(to_sun, sun_radius_m, east, north, up, uv, sun_margin);
		float earth_margin = 0.0;
		earth_visible = terrain_visible_fraction(to_earth, earth_radius_m, east, north, up, uv, earth_margin);
	}
	float ground_theta_bar = detail_theta_bar(detail_cell_m);
	float albedo_factor = rock_custom.z;
	float sun_reflectance = moon_rock_reflectance(n, up, normalize(to_sun), to_eye, albedo_factor, gradient, fillet,
			ground_theta_bar);
	float earth_reflectance = moon_rock_reflectance(n, up, normalize(to_earth), to_eye, albedo_factor, gradient, fillet,
			ground_theta_bar);
	float lit = moon_rock_lit(sun_margin, detail_cell_m, detail_horizon_spacing_m);
	float light = moon_regolith_scale * (sun_reflectance * sunlit * sun_visible * moon_sun_scene * lit
			+ earth_reflectance * earth_visible * moon_earthshine) / moon_rock_norm(abundance);
	float half_texel_v = 0.5 / float(textureSize(albedo_texture, 0).y);
	vec3 albedo = texture(albedo_texture, vec2(global_uv.x, clamp(global_uv.y, half_texel_v, 1.0 - half_texel_v))).rgb;
	ALBEDO = albedo * relative_albedo * light;
}
