shader_type spatial;
// Hull of the detailed Starship / Super Heavy models (game/vehicle/vehicle_models.gd): bare stainless steel (weld seams
// and oil-canning dents from the baked, mipmapped steel map, per-ring panel tint, soot-free) and, on the ship's
// windward side and the flaps' outer faces, the heat-shield tiles of heat_shield_tiles.gdshaderinc. Lit like LitAerial (Godot's PBR lighting
// and the sky's reflections, seen through the atmosphere as FOG), opaque: it must stay in the opaque pass, which the
// cloud layer draws behind (a transparent draw would hide from the clouds).
//
// Coordinates are the node's LOCAL ones = the config's body frame (X station, nose +X; flaps: X hinge axis, Y outer
// face, Z span); the steel map is addressed by the hull's UV (arc metres, station metres), the tiles are analytic. Mode 0 steel only (booster,
// chine, raceway, flap edges), 1 ship hull (tiles within the coverage table around the belly), 2 flap (tiles on +Y).

#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/heat_shield_tiles.gdshaderinc"

// [E] Large-scale variation of the sheets: +-3 % (+-10 % read as blotches once the ambient was right).
const float STEEL_MOTTLE = 0.03;
const float STEEL_MOTTLE_BASE = 1.0 - STEEL_MOTTLE;
uniform int mode = 0;
// F0 of bare iron-based steel, linear (0.56, 0.57, 0.58) [S: Filament material guide, Iron], sRGB-encoded here.
uniform vec3 steel_albedo : source_color = vec3(0.773, 0.780, 0.786);
uniform float steel_metallic : hint_range(0.0, 1.0) = 1.0;  // a metal: no diffuse, sky and ground from the radiance cube
uniform float steel_roughness : hint_range(0.0, 1.0) = 0.35;  // [E] cold-rolled sheet: hazy reflections (photos)
uniform float ring_pitch_m = 1.8;
uniform float barrel_end_m = 1.0e9;      // ship: the nose starts here (below it the shield's profile is straight)
uniform float footprint_scale = 1.0;      // test hook: tile_shimmer_check's supersampled reference sees px * its scale

// Baked steel map (tools/structures/hull_textures.py): R, G slope along u / v, B slope energy, A weld band coverage.
uniform bool steel_textured = false;      // false (no map, flaps, chine, fins ...): plain steel
uniform sampler2D steel_map : filter_linear_mipmap_anisotropic, repeat_enable;
uniform vec2 steel_period_m = vec2(4.712, 1.8);   // one map repeat along u (arc) and v (station), metres
uniform float steel_slope_range = 0.6;
uniform float steel_energy_range = 0.1;
uniform float weld_first_m = 0.35;        // no weld seams on the chamfer below this station
const float WELD_DARKEN = 0.25;           // [E] heat-affected band: albedo drop and roughness gain
const float WELD_ROUGHER = 0.22;

uniform float tile_across_flats_m = 0.30;
uniform float tile_gap_m = 0.006;
uniform float belly_azimuth_rad = -1.5707963;   // -Z in the body frame
uniform int coverage_count = 5;
uniform float coverage_station_m[8];
uniform float coverage_half_angle_rad[8];
uniform vec3 tile_albedo = vec3(0.030, 0.031, 0.034);   // linear, black ceramic with its coating
uniform vec3 gap_albedo = vec3(0.055, 0.052, 0.050);      // gap filler
uniform float tile_roughness : hint_range(0.0, 1.0) = 0.55;
uniform float tile_bevel_m = 0.012;

varying vec3 world_offset;  // fragment - the render origin (= the atmosphere's camera), world axes
varying vec3 local_pos;
varying vec3 local_normal;

void vertex() {
	world_offset = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
	local_pos = VERTEX;
	local_normal = NORMAL;
}

float coverage_half_angle(float station_m) {
	if (station_m <= coverage_station_m[0]) {
		return coverage_half_angle_rad[0];
	}
	for (int i = 1; i < 8; i++) {
		if (i >= coverage_count) {
			break;
		}
		if (station_m <= coverage_station_m[i]) {
			float t = (station_m - coverage_station_m[i - 1]) / max(coverage_station_m[i] - coverage_station_m[i - 1], 1e-6);
			return mix(coverage_half_angle_rad[i - 1], coverage_half_angle_rad[i], t);
		}
	}
	return coverage_half_angle_rad[coverage_count - 1];
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;

	vec3 n = normalize(local_normal);
	// Footprint of a pixel on the surface (m): the continuous local position's derivatives.
	float px = footprint_scale * max(length(dFdx(local_pos)), length(dFdy(local_pos)));

	// Hull ring structure: station -> ring index, distance to the weld at the ring's edge.
	float ring_pos = local_pos.x / ring_pitch_m;
	float ring = floor(ring_pos);
	float panel_fade = 1.0 - smoothstep(0.06, 0.8, px);  // the per-ring tint (a hash of the ring index) fades to its mean
	float ring_hash = hull_hash3(vec3(ring, 1.7, 4.3));

	// Steel map: weld bead + dents as a tangent-space slope, the weld's heat-affected band, all mipmapped (the engine's
	// trilinear + anisotropic filtering replaces the old footprint fades). The slope energy lost on the coarser mips
	// widens the roughness (Toksvig / LEAN style), so the glints do not shimmer. A per-ring shift along u gives every
	// ring panel its own dents; the derivatives come from the unshifted coordinates.
	float weld = 0.0;
	vec3 n_steel = n;
	float slope_var = 0.0;
	if (steel_textured) {
		vec2 uv_t = UV / steel_period_m;
		float shift = hull_hash3(vec3(ring, 9.1, 2.7));
		vec4 t = textureGrad(steel_map, uv_t + vec2(shift, 0.0), dFdx(uv_t) * footprint_scale, dFdy(uv_t) * footprint_scale);
		float on = step(weld_first_m, local_pos.x);
		vec2 slope = (t.rg - 0.5) * steel_slope_range * on;
		slope_var = max(t.b * steel_energy_range - dot(slope, slope), 0.0) * on;
		weld = t.a * on;
		vec3 tangent_u = normalize(vec3(0.0, -local_pos.z, local_pos.y) + vec3(1e-6, 0.0, 0.0));
		vec3 tangent_v = cross(n, tangent_u);
		n_steel = normalize(n - tangent_u * slope.x - tangent_v * slope.y);
	}
	vec3 steel = steel_albedo * (1.0 + panel_fade * (ring_hash - 0.5) * 0.10);
	steel *= 1.0 - WELD_DARKEN * weld;
	steel *= mix(1.0, STEEL_MOTTLE_BASE + 2.0 * STEEL_MOTTLE * hull_noise3(local_pos * 0.12), panel_fade);
	float steel_rough = steel_roughness + panel_fade * (ring_hash - 0.5) * 0.08 + WELD_ROUGHER * weld;
	float alpha_steel = steel_rough * steel_rough;  // GGX alpha (Godot: roughness^2) widened by the lost slope variance
	steel_rough = sqrt(sqrt(alpha_steel * alpha_steel + slope_var));

	float r = length(local_pos.yz);
	vec3 radial = vec3(0.0, local_pos.y, local_pos.z) / max(r, 1e-4);
	float n_radial = dot(n, radial);

	vec3 n_tile_base = n;
	float tiles = 0.0;  // 1 where the shield covers
	vec2 surface_m = vec2(0.0);
	vec3 tangent_s = vec3(0.0);
	vec3 axial = vec3(1.0, 0.0, 0.0);
	if (mode == 1) {
		float theta = atan(local_pos.z, local_pos.y) - belly_azimuth_rad;
		theta = mod(theta + PI, 2.0 * PI) - PI;  // 0 on the belly line, the seam on the lee side (no tiles there)
		surface_m = vec2(theta * r, local_pos.x);
		tangent_s = normalize(vec3(0.0, -local_pos.z, local_pos.y));
		float limit_m = coverage_half_angle(local_pos.x) * r;
		tiles = clamp((limit_m - abs(surface_m.x)) / max(px, 1e-5) + 0.5, 0.0, 1.0);
		tiles *= step(0.5, n_radial);  // not on the base cap / chamfer
		float profile_slope = -n.x / max(n_radial, 0.1);
		n_tile_base = normalize(vec3(-(local_pos.x < barrel_end_m ? 0.0 : profile_slope), 0.0, 0.0) + radial);
	} else if (mode == 2) {
		surface_m = vec2(local_pos.z, local_pos.x);
		tangent_s = vec3(0.0, 0.0, 1.0);
		tiles = smoothstep(0.6, 0.9, n.y);  // the outer face (the flaps' stowed-outward side) carries the shield
	}

	vec3 albedo_out = steel;
	float rough_out = steel_rough;
	float metal_out = steel_metallic;
	float spec_out = 0.5;
	vec3 n_out = n_steel;

	if (tiles > 0.0) {
		Tile t = tile_cell(surface_m, tile_across_flats_m);
		float gap_half = 0.5 * tile_gap_m;
		float gap = stripe_cover(t.edge_m, gap_half, px);
		float h = tile_hash(t.id);
		float detail_fade = 1.0 - smoothstep(tile_across_flats_m * 0.12, tile_across_flats_m * 0.35, px);
		// Per-tile shade: a slightly different coating per tile, a few lighter (repaired) tiles; both fade to the mean.
		float shade = 0.75 + 0.5 * h;
		float repaired = step(0.994, tile_hash(t.id + 17.0));
		vec3 tile_col = tile_albedo * mix(1.0, shade, detail_fade);
		tile_col = mix(tile_col, vec3(0.075, 0.075, 0.078), repaired * detail_fade);
		float gap_mean = clamp(2.0 * tile_gap_m / tile_across_flats_m, 0.0, 1.0);
		gap = mix(gap, gap_mean, 1.0 - detail_fade);
		vec3 tile_final = mix(tile_col, gap_albedo, gap);
		// Bevel: the face rounds off towards the edge, tilting the normal outwards (fades with the footprint).
		// The bevel is 12 mm wide: it fades out with the footprint (to nothing: its brightness is a highlight, no mean).
		float bevel_fade = 1.0 - smoothstep(tile_bevel_m * 1.0, tile_bevel_m * 3.0, px);
		float bevel = (1.0 - smoothstep(gap_half, gap_half + tile_bevel_m, t.edge_m)) * bevel_fade * (1.0 - gap);
		vec3 outward = normalize(tangent_s * t.offset_m.x + axial * t.offset_m.y + 1e-6 * tangent_s);
		vec3 n_tile = normalize(n_tile_base + 0.4 * bevel * outward);
		albedo_out = mix(steel, tile_final, tiles);
		rough_out = mix(steel_rough, mix(tile_roughness, 0.75, gap), tiles);
		metal_out = steel_metallic * (1.0 - tiles);
		spec_out = mix(0.5, 0.4, tiles);
		n_out = normalize(mix(n_steel, n_tile, tiles));
	}

	ALBEDO = albedo_out * colour_factor;
	ROUGHNESS = clamp(rough_out, 0.04, 1.0);
	METALLIC = metal_out;
	SPECULAR = spec_out;
	NORMAL = normalize((VIEW_MATRIX * vec4(MODEL_NORMAL_MATRIX * n_out, 0.0)).xyz);
}
