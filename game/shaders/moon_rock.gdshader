shader_type spatial;
// Drawn lunar rocks (moon procedural surface plan Task 11; MoonRockLayer): one MultiMesh per terrain node and template
// LOD, instances = the core's rocks of the node (StarsimCore.moon_rocks_in_node). The mesh is the unit icosphere of the
// LOD (vertex = direction); each vertex takes its template's radius r(dir) and normal from rock_template_data (row
// lod x 32 + template, column VERTEX_ID), so all 32 templates share one mesh per LOD and the instance transform (the
// core's basis x semi-axes, the centre burial_m below the base point, relative to the node centre) places the rock.
//
// Drawn by projected size, from one reference point (the camera; in VR the head centre: MODELVIEW_MATRIX is the
// centre view's): d = the threshold diameter at the rock centre's distance (detail_rock_threshold_m: ROCK_PIXEL_THRESHOLD
// x max(rock_threshold_scale, the abundance load factor) pixels, never below the node's list limit); a rock with D < d
// is not drawn (it is in the ground's statistical layer), from d to 2 d it rises out of the ground so that its exposed
// plan area is the share x = log2(D / d) of its nominal one (the template's plan-area tables; the statistical weight
// 1 - x mirrors it in area, no rock counted twice), above 2 d it stands at its nominal burial. Normals per fragment
// from the template's convex constraints (faces and ellipsoid): hard facet edges at every LOD.
// The LOD MultiMesh draws an instance only inside its band of projected diameter (CUSTOM0), so each rock is drawn by
// exactly one LOD. Instance data: transform; colour = (grid a, grid b: the base in the node's vertex grid, the horizon
// map's coordinate; global u, v: the albedo map's); custom = (D, nominal exposed height in template z units (top_z -
// burial / c), albedo factor, template id).
// Light (moon_rock_light.gdshaderinc): Hapke rock law, the node's horizon map at the base for the Sun and the
// earthshine, the Earth's shadow on the Sun, Smith's G1 of the node's drawn relief for each light at its own elevation;
// the rock textures (triplanar in the rock's own metres, weights by the facet normal) as albedo variation and
// mean-preserving slope modulation.
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
// The eye adaptation's exposure gain (atmosphere.gd), applied as terrain_moon.gdshader applies it to the lit Moon (Task
// 11 fix C). Rocks are drawn only near a camera at the Moon, whose view ray never crosses the Earth's atmosphere: no
// transmittance or in-scatter term (terrain_moon's is identity there).
global uniform bool atmosphere_enabled;
global uniform float atmosphere_exposure_gain;

uniform sampler2D albedo_texture : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
// RGBA float: (normal xyz, radius r) of template t's vertex k at LOD l in texel (k, l x ROCK_TEMPLATES + t).
uniform sampler2D rock_template_data : filter_nearest, repeat_disable;
uniform float rock_top_z[32];  // per template: the highest vertex z (unit template; LOD 0)
// Per template row (MoonRockLayer._build_shape_tables): texels 0..23 the faces (n, h: n . p <= h), 24..26 the
// ellipsoid's quadric rows (p^T Q p <= 1), 27 (face count, bottom z, top z, 0) - unit template frame.
uniform sampler2D rock_shape_data : filter_nearest, repeat_disable;
// Per template row: R = exposed plan area A(z) / A_max at z = bottom + (top - bottom) k / 63, G = the highest z with
// A(z) >= a A_max at a = k / 63.
uniform sampler2D rock_area_data : filter_nearest, repeat_disable;
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

varying vec3 rock_up;  // the local vertical (the instance basis' third column), local frame
varying vec3 rock_template_position;  // unit-template position before the growth band's sinking: facets, triplanar
varying vec3 rock_vertex_normal;  // the template's per-vertex normal, interpolated (diagnostics: rock_facet_normals off)
// Diagnostics (moon_surface_check): false draws the interpolated vertex normals instead of the exact facet normals.
uniform bool rock_facet_normals = true;
varying vec3 rock_x;  // template axes in the Moon local render frame divided by the (isotropic) scale: template
varying vec3 rock_y;  // vectors -> local frame
varying vec3 rock_z;
varying float rock_scale_m;  // the template's metres per unit (isotropic: LunarRocks axes (a, a, a))
varying float height_above_ground_m;
varying flat vec4 rock_custom;  // INSTANCE_CUSTOM
varying flat vec4 rock_place;  // COLOR (grid a, grid b, global u, v)

// The template's plan-area tables (linear between samples).
float rock_area_forward(int template_id, float z, float bottom, float top) {
	float x = clamp((z - bottom) / max(top - bottom, 1.0e-6), 0.0, 1.0) * 63.0;
	int k = min(int(x), 62);
	return mix(texelFetch(rock_area_data, ivec2(k, template_id), 0).r,
			texelFetch(rock_area_data, ivec2(k + 1, template_id), 0).r, x - float(k));
}

float rock_area_inverse(int template_id, float a) {
	float x = clamp(a, 0.0, 1.0) * 63.0;
	int k = min(int(x), 62);
	return mix(texelFetch(rock_area_data, ivec2(k, template_id), 0).g,
			texelFetch(rock_area_data, ivec2(k + 1, template_id), 0).g, x - float(k));
}

// [E] The last 1 / ROCK_RISE_HEIGHT_TAIL of the growth band may lift a rock whose plan area is already complete (its
// nominal ground below the widest section): a height-only rise, no area change, so the rock reaches its nominal burial
// exactly at the band's end. 4: the tail rises at most 4 x the linear height rate (the popping sweep: 20 gave single-
// frame height jumps); the plan area is complete up to a quarter band early for such rocks (a small excess, band
// rocks only).
const float ROCK_RISE_HEIGHT_TAIL = 4.0;

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
		rock_up = vec3(0.0, 0.0, 1.0);
		rock_template_position = vec3(0.0);
		rock_vertex_normal = vec3(0.0, 0.0, 1.0);
		rock_x = vec3(1.0, 0.0, 0.0);
		rock_y = vec3(0.0, 1.0, 0.0);
		rock_z = vec3(0.0, 0.0, 1.0);
		rock_scale_m = 0.0;
		height_above_ground_m = 0.0;
	} else {
		int template_id = clamp(int(INSTANCE_CUSTOM.w + 0.5), 0, ROCK_TEMPLATES - 1);
		vec4 t = texelFetch(rock_template_data, ivec2(VERTEX_ID, rock_lod * ROCK_TEMPLATES + template_id), 0);
		vec3 p = VERTEX * t.w;
		rock_template_position = p;
		rock_vertex_normal = t.xyz;
		float exposed = INSTANCE_CUSTOM.y;  // top_z - burial / c: the nominal exposed height (template z units)
		float ground_z = rock_top_z[template_id] - exposed;
		// Growth band (review I2): the rock rises so that its exposed PLAN AREA is the share x = growth of its nominal
		// one - what occludes the ground, and what the statistical weight 1 - x of the ground's layer mirrors. The cut
		// z_t with A(z_t) = x A(ground_z) from the template's tables; sunk by z_t - ground_z (fully buried at x = 0).
		vec4 extent = texelFetch(rock_shape_data, ivec2(27, template_id), 0);
		float nominal_area = rock_area_forward(template_id, ground_z, extent.y, extent.z);
		float sink = max(rock_area_inverse(template_id, growth * nominal_area) - ground_z, 0.0);
		sink = min(sink, (1.0 - growth) * ROCK_RISE_HEIGHT_TAIL * exposed);
		p.z -= sink;
		VERTEX = p;
		// The instance basis is rotation x a (isotropic, rigid node and body transforms): template vectors (normals
		// included) go to the local frame by the rotation.
		mat3 m = mat3(MODEL_MATRIX);
		float scale = length(m[2]);
		mat3 to_local = moon_world_to_local * m / scale;
		rock_x = to_local[0];
		rock_y = to_local[1];
		rock_z = to_local[2];
		rock_up = normalize(to_local[2]);
		rock_scale_m = scale;
		height_above_ground_m = (p.z - ground_z) * scale;  // the ground plane stays at ground_z (p already sunk)
	}
}

// Exact outward normal of the convex template at the (interpolated) unit-template position p: the active constraint
// is the one p is closest to violating (faces: n . p - h; the ellipsoid: (sqrt(p^T Q p) - 1) over its gradient's
// length, a distance to first order). Between two faces the larger wins: a sharp crease along the projected edge, no
// smoothing across cut planes (Task 11 fix B: hard facet edges).
vec3 rock_facet_normal(int template_id, vec3 p) {
	vec4 extent = texelFetch(rock_shape_data, ivec2(27, template_id), 0);
	vec3 q0 = texelFetch(rock_shape_data, ivec2(24, template_id), 0).xyz;
	vec3 q1 = texelFetch(rock_shape_data, ivec2(25, template_id), 0).xyz;
	vec3 q2 = texelFetch(rock_shape_data, ivec2(26, template_id), 0).xyz;
	vec3 qp = vec3(dot(q0, p), dot(q1, p), dot(q2, p));
	float v = max(dot(p, qp), 1.0e-12);
	float best = (sqrt(v) - 1.0) * sqrt(v) / max(length(qp), 1.0e-12);
	vec3 normal = qp;
	int faces = int(extent.x + 0.5);
	for (int k = 0; k < faces; k++) {
		vec4 plane = texelFetch(rock_shape_data, ivec2(k, template_id), 0);
		float g = dot(plane.xyz, p) - plane.w;
		if (g > best) {
			best = g;
			normal = plane.xyz;
		}
	}
	return normalize(normal);
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
	int template_id = clamp(int(rock_custom.w + 0.5), 0, ROCK_TEMPLATES - 1);
	vec3 model_normal = rock_facet_normals ? rock_facet_normal(template_id, rock_template_position)
			: normalize(rock_vertex_normal);
	mat3 to_local = mat3(rock_x, rock_y, rock_z);
	vec3 n = normalize(to_local * model_normal);
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
		vec4 tex = rock_texture(rock_template_position * rock_scale_m, model_normal, layer);
		relative_albedo = mix(1.0, tex.w, fillet);
		vec3 g = to_local * tex.xyz;
		gradient = (g - n * dot(g, n)) * fillet;
	}
	// Terrain horizon at the base (the node's map), Earth shadow, the Sun's macroscopic elevation.
	float sunlit = 1.0 - occulted_fraction(to_sun, sun_radius_m, to_earth, earth_radius_m);
	float sun_margin = asin(clamp(dot(normalize(to_sun), up), -1.0, 1.0));
	float earth_margin = asin(clamp(dot(normalize(to_earth), up), -1.0, 1.0));
	float sun_visible = 1.0;
	float earth_visible = 1.0;
	if (horizon_enabled) {
		vec3 east = vec3(up.z, 0.0, -up.x);
		east = length(east) > POLE_EAST_MIN ? normalize(east) : vec3(0.0, 0.0, 1.0);
		vec3 north = cross(up, east);
		vec2 uv = ((rock_place.xy * horizon_grid_scale + horizon_grid_offset) / horizon_stride + 0.5)
				/ float(textureSize(horizon_map, 0).x);
		sun_visible = terrain_visible_fraction(to_sun, sun_radius_m, east, north, up, uv, sun_margin);
		earth_visible = terrain_visible_fraction(to_earth, earth_radius_m, east, north, up, uv, earth_margin);
	}
	float ground_theta_bar = detail_theta_bar(detail_cell_m);
	float albedo_factor = rock_custom.z;
	float sun_reflectance = moon_rock_reflectance(n, up, normalize(to_sun), to_eye, albedo_factor, gradient, fillet,
			ground_theta_bar);
	float earth_reflectance = moon_rock_reflectance(n, up, normalize(to_earth), to_eye, albedo_factor, gradient, fillet,
			ground_theta_bar);
	// Smith's G1 of the node's unshadowed drawn relief for each light at its macroscopic elevation (the ground's own
	// factor; review I1: the earthshine is a directional light like the Sun).
	float lit = moon_rock_lit(sun_margin, detail_cell_m, detail_horizon_spacing_m);
	float earth_lit = moon_rock_lit(earth_margin, detail_cell_m, detail_horizon_spacing_m);
	float light = moon_regolith_scale * (sun_reflectance * sunlit * sun_visible * moon_sun_scene * lit
			+ earth_reflectance * earth_visible * moon_earthshine * earth_lit) / moon_rock_norm(abundance);
	float half_texel_v = 0.5 / float(textureSize(albedo_texture, 0).y);
	vec3 albedo = texture(albedo_texture, vec2(global_uv.x, clamp(global_uv.y, half_texel_v, 1.0 - half_texel_v))).rgb;
	ALBEDO = albedo * relative_albedo * light * (atmosphere_enabled ? atmosphere_exposure_gain : 1.0);
}
