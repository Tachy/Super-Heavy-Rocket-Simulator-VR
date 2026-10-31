shader_type spatial;
// Hull of the detailed Starship / Super Heavy models (game/vehicle/vehicle_models.gd): bare stainless steel (ring
// weld bands, per-ring panel tint, oil-canning dents in the normal, soot-free) and, on the ship's windward side and the
// flaps' outer faces, the heat-shield tiles of heat_shield_tiles.gdshaderinc. Lit like LitAerial (Godot's PBR lighting
// and the sky's reflections, seen through the atmosphere as FOG), opaque: it must stay in the opaque pass, which the
// cloud layer draws behind (a transparent draw would hide from the clouds).
//
// All coordinates are the node's LOCAL ones = the config's body frame (X station, nose +X; flaps: X hinge axis, Y outer
// face, Z span), so the pattern is a function of the surface and needs no baked texture. Mode 0 steel only (booster,
// chine, raceway, flap edges), 1 ship hull (tiles within the coverage table around the belly), 2 flap (tiles on +Y).

#include "res://shaders/atmosphere.gdshaderinc"
#include "res://shaders/heat_shield_tiles.gdshaderinc"

uniform int mode = 0;
uniform vec3 steel_albedo : source_color = vec3(0.80, 0.81, 0.83);
uniform float steel_metallic : hint_range(0.0, 1.0) = 0.8;  // < 1: a diffuse share stands in for the sky's weak reflection
uniform float steel_roughness : hint_range(0.0, 1.0) = 0.27;
uniform float ring_pitch_m = 1.8;
uniform float weld_half_width_m = 0.06;   // heat-affected band either side of a weld
uniform float barrel_end_m = 1.0e9;      // ship: the nose starts here (below it the shield's profile is straight)
uniform float weld_bead_half_m = 0.05;    // the weld bead's geometry (detail file): half width and slope = height / half width
uniform float weld_bead_slope = 0.24;
uniform float dent_gain = 0.07;           // slope amplitude of the oil-canning dents (normal tilt)

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

// Dent height in slope units: two octaves of smooth 3D noise, a different phase per ring panel.
float dent_height(vec3 p, float ring) {
	vec3 q = p * vec3(0.55, 0.85, 0.85) + vec3(ring * 7.31, ring * 3.17, ring * 1.93);
	return (hull_noise3(q) - 0.5) + 0.35 * (hull_noise3(q * 3.1 + 11.7) - 0.5);
}

void fragment() {
	vec4 fog;
	vec3 colour_factor;
	atm_fog(world_offset, fog, colour_factor);
	FOG = fog;

	vec3 n = normalize(local_normal);
	// Footprint of a pixel on the surface (m): the continuous local position's derivatives.
	float px = max(length(dFdx(local_pos)), length(dFdy(local_pos)));

	// Hull ring structure: station -> ring index, distance to the weld at the ring's edge.
	float ring_pos = local_pos.x / ring_pitch_m;
	float ring = floor(ring_pos);
	float weld_d = abs(ring_pos - round(ring_pos)) * ring_pitch_m;
	float panel_fade = 1.0 - smoothstep(0.25, 0.8, px);  // dents and per-ring variation fade to their mean
	float ring_hash = hull_hash3(vec3(ring, 1.7, 4.3));

	// Steel: albedo with a per-ring tint, heat-affected weld bands (darker, rougher), low-frequency tint drift.
	float weld = stripe_cover(weld_d, weld_half_width_m, px);
	float weld_mean = min(2.0 * weld_half_width_m / ring_pitch_m, 1.0);
	weld = mix(weld, weld_mean, smoothstep(0.1 * ring_pitch_m, 0.4 * ring_pitch_m, px));
	vec3 steel = steel_albedo * (1.0 + panel_fade * (ring_hash - 0.5) * 0.10);
	steel *= 1.0 - 0.25 * weld;
	steel *= vec3(1.0, 0.985, 0.96) * mix(1.0, 0.9 + 0.2 * hull_noise3(local_pos * 0.12), panel_fade);
	float steel_rough = steel_roughness + panel_fade * (ring_hash - 0.5) * 0.08 + 0.22 * weld;

	// Oil-canning: the normal tilted by the gradient of the dent height (finite differences, in local space).
	vec3 g = vec3(0.0);
	if (panel_fade > 0.0) {
		float e = 0.12;
		float h0 = dent_height(local_pos, ring);
		g = vec3(dent_height(local_pos + vec3(e, 0, 0), ring) - h0, dent_height(local_pos + vec3(0, e, 0), ring) - h0,
				dent_height(local_pos + vec3(0, 0, e), ring) - h0) / e;
		g -= n * dot(g, n);
	}
	vec3 n_steel = normalize(n - dent_gain * panel_fade * g);

	vec3 albedo_out = steel;
	float rough_out = steel_rough;
	float metal_out = steel_metallic;
	float spec_out = 0.5;
	vec3 n_out = n_steel;

	vec3 n_tile_base = n;
	float tiles = 0.0;  // 1 where the shield covers
	vec2 surface_m = vec2(0.0);
	vec3 tangent_s = vec3(0.0);
	vec3 axial = vec3(1.0, 0.0, 0.0);
	if (mode == 1) {
		float r = length(local_pos.yz);
		float theta = atan(local_pos.z, local_pos.y) - belly_azimuth_rad;
		theta = mod(theta + PI, 2.0 * PI) - PI;  // 0 on the belly line, the seam on the lee side (no tiles there)
		surface_m = vec2(theta * r, local_pos.x);
		tangent_s = normalize(vec3(0.0, -local_pos.z, local_pos.y));
		float limit_m = coverage_half_angle(local_pos.x) * r;
		tiles = clamp((limit_m - abs(surface_m.x)) / max(px, 1e-5) + 0.5, 0.0, 1.0);
		vec3 radial = vec3(0.0, local_pos.y, local_pos.z) / r;
		tiles *= step(0.5, dot(n, radial));  // not on the base cap / chamfer
		// The shield lies over the hull: no weld beads under it. The bead's slope (a triangle of known size at every
		// ring) is taken out of the mesh normal's profile slope.
		float to_weld_m = (ring_pos - round(ring_pos)) * ring_pitch_m;
		float bead_slope = abs(to_weld_m) < weld_bead_half_m ? -sign(to_weld_m) * weld_bead_slope : 0.0;
		float profile_slope = -n.x / max(dot(n, radial), 0.1) - bead_slope;
		n_tile_base = normalize(vec3(-(local_pos.x < barrel_end_m ? 0.0 : profile_slope), 0.0, 0.0) + radial);
	} else if (mode == 2) {
		surface_m = vec2(local_pos.z, local_pos.x);
		tangent_s = vec3(0.0, 0.0, 1.0);
		tiles = smoothstep(0.6, 0.9, n.y);  // the outer face (the flaps' stowed-outward side) carries the shield
	}

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
		tile_col = mix(tile_col, vec3(0.12, 0.12, 0.125), repaired * detail_fade);
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
