// The march's reads of the volume levels: the sets' textures, the air shadow from S, the point's extinction and light,
// the march's step constants and its ray in grid coordinates (cloud_baked.gdshaderinc).

#ifdef CLOUD_BAKED_MARCH
// ---- The march -----------------------------------------------------------------------------------------------------
layout(set = 1, binding = 50) uniform sampler3D baked_a0;  // linear
layout(set = 1, binding = 51) uniform sampler3D baked_b0;  // linear
layout(set = 1, binding = 52) uniform sampler3D baked_o0;  // nearest
#ifdef CLOUD_BAKED_FADE
layout(set = 1, binding = 54) uniform sampler3D baked_a1;
layout(set = 1, binding = 55) uniform sampler3D baked_b1;
layout(set = 1, binding = 56) uniform sampler3D baked_o1;
#endif

// The set's volumes (fetch helpers instead of sampler parameters: si is a constant at each call).
vec2 cloud_baked_a(int si, vec3 uvw) {
#ifdef CLOUD_BAKED_FADE
	if (si == 1) {
		return texture(baked_a1, uvw).rg;
	}
#endif
	return texture(baked_a0, uvw).rg;
}

vec4 cloud_baked_b(int si, vec3 uvw) {
#ifdef CLOUD_BAKED_FADE
	if (si == 1) {
		return texture(baked_b1, uvw);
	}
#endif
	return texture(baked_b0, uvw);
}

float cloud_baked_o(int si, ivec3 texel) {
#ifdef CLOUD_BAKED_FADE
	if (si == 1) {
		return texelFetch(baked_o1, texel, 0).r;
	}
#endif
	return texelFetch(baked_o0, texel, 0).r;
}

// The air shadow's T_c at camera + offset (height m): the line toward the Sun through the point read from S (both sets
// by the cross-fade's weight while a new set fades in). False beyond the volume levels (the caller's column).
bool cloud_baked_air_sun(vec3 offset_local, float height_m, vec3 sun_dir, out float t) {
	vec4 line;
	bool inside = cloud_baked_shadow_line(0, offset_local, sun_dir, cloud_baked_air_footprint_m, line);
	t = cloud_baked_shadow_at_height(line, height_m);
#ifdef CLOUD_BAKED_FADE
	vec4 line1;
	if (cloud_baked_shadow_line(1, offset_local, sun_dir, cloud_baked_air_footprint_m, line1)) {
		float t1 = cloud_baked_shadow_at_height(line1, height_m);
		t = inside ? mix(t, t1, baked.sets[0].misc.y) : t1;
		inside = true;
	}
#endif
	return inside;
}

float cloud_baked_flat_weight_at(int si, float l);

// A march point in one set's levels (cloud_baked_lod): its level k (blend toward k + 1, texture coordinates in both)
// and keep = 1 - the flat levels' weight (the volume's share).
struct CloudBakedPoint {
	int k;
	float blend;
	float keep;
	vec3 uvw;
	vec3 g;  // the grid point and height (the next level's coordinates only while blending)
};

CloudBakedPoint cloud_baked_point(int si, float gx, float gy, float h, float footprint_m, int k_min) {
	CloudBakedPoint q;
	float l = cloud_baked_lod(si, max(abs(gx), abs(gy)), footprint_m);
	q.keep = 1.0 - cloud_baked_flat_weight_at(si, l);  // (the takeover: region by region, cloud_baked_set)
	q.k = cloud_baked_level_of(si, l, k_min, q.blend);
	int levels = int(baked.sets[si].axis_y.w);
	q.uvw = cloud_baked_uvw(si, min(q.k, levels - 1), gx, gy, h);
	q.g = vec3(gx, gy, h);
	return q;
}

// The light (B, still sqrt-coded) at a march point, blended as its extinction.
vec4 cloud_baked_b2(int si, CloudBakedPoint q) {
	vec4 b = cloud_baked_b(si, q.uvw);
	return q.blend > 0.0 ? mix(b, cloud_baked_b(si, cloud_baked_uvw(si, q.k + 1, q.g.x, q.g.y, q.g.z)), q.blend) : b;
}

// The extinction (1/m) at a march point (its level and blend, camera offset, pixel footprint): the baked field plus, with
// CLOUD_BAKED_DETAIL in level 0, one fetch of the detail's level 1 (62.5 m cells; the bake holds level 0) at the
// pixel's band limit near the edge - the old march's erosion term (cloud_detail_perturbation) on the edge coordinate.
// gap_x: how far the point lies outside any cloud the per-sample detail could add, in edge widths (the signed edge
// coordinate below the detail's reach; <= 0 within it) - the clear march's distance estimate.
float cloud_baked_beta_gap(int si, CloudBakedPoint q, vec3 offset, float footprint_m, out float gap_x) {
	int k = q.k;
	vec3 uvw = q.uvw;
	vec2 a = cloud_baked_a(si, uvw);
	if (q.blend > 0.0) {
		a = mix(a, cloud_baked_a(si, cloud_baked_uvw(si, k + 1, q.g.x, q.g.y, q.g.z)), q.blend);  // the level rule's blend
	}
	gap_x = a.y > 0.0 ? -a.x : CLOUD_BAKED_X_MAX;
#ifdef CLOUD_BAKED_DETAIL
	// The detail's reach in edge widths: down (erosion) its full amplitude, up only its lifted part at the detail's
	// largest value 1 (cloud_detail_lift_bound's term: ~1.8 widths against ~14 down).
	float reach = abs(cloud_level_amplitude[1]) / max(cloud_edge_width.x, 1.0e-6);
	float lift_reach = cloud_level_lift[1] * max(cloud_level_amplitude[1], 0.0) * (1.0 - cloud_detail_mean)
			/ max(cloud_edge_width.x, 1.0e-6);
	if (k == 0) {
		gap_x -= lift_reach;
	}
	if (k == 0 && a.y > 0.0 && a.x > -lift_reach && a.x < 1.0 + reach) {
		float w1 = clamp(log2(CLOUD_DETAIL_TILE_M / 16.0 / max(footprint_m, 1.0e-3)) - 1.0, 0.0, 1.0);
		if (w1 > 0.0 && cloud_detail_levels > 1) {
			float width_noise = cloud_baked_b2(si, q).w;
			float width = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, width_noise);
			vec3 q = fract(cloud_detail_origin * 4.0) + offset * (4.0 / CLOUD_DETAIL_TILE_M);
			float d = dot(texture(cloud_noise_detail, q).rgb, cloud_detail_weights);
			float term = w1 * cloud_level_amplitude[1] * (d - cloud_detail_mean);
			a.x += (term < 0.0 ? term : cloud_level_lift[1] * term) / max(width, 1.0e-6);
		}
	}
#endif
	return cloud_baked_extinction(a) * q.keep;
}

float cloud_baked_beta(int si, CloudBakedPoint q, vec3 offset, float footprint_m) {
	float gap_x;
	return cloud_baked_beta_gap(si, q, offset, footprint_m, gap_x);
}

// [E] Steps per ray at most; the shortest step (m); the bisections of a cloud entry; the in-cloud step weight limit
// (the old march's CLOUD_MAX_STEP_WEIGHT: a step's share of the pixel at most 0.15 / T).
const int CLOUD_BAKED_MAX_STEPS = 768;
const float CLOUD_BAKED_MIN_STEP_M = 4.0;
const int CLOUD_BAKED_BISECTIONS = 3;
const float CLOUD_BAKED_STEP_WEIGHT = 0.15;
// [E] The clear march near an edge (fix 1): metres per edge width of the distance estimate (the edge width is the
// shape value's median increment over 20-40 m, constants.hpp cloud_edge_width_min_m) and the shortest clear step;
// the in-cloud step's limit as a fraction of the cell (half a cell left wood-grain contours and facets in the near
// rims: the trapezoid over a thin lens changes with the step's phase); both only while the ray's scaled transmittance
// is above CLOUD_BAKED_FINE_TRANSMITTANCE (behind a dense part the coarse steps' errors are not seen). (Measured:
// 10 m / 8 m and 1/8 cell, the finest tried, looked the same as these.)
const float CLOUD_BAKED_EDGE_STEP_M = 20.0;
const float CLOUD_BAKED_MIN_CLEAR_STEP_M = 16.0;
const float CLOUD_BAKED_FINE_STEP_CELLS = 0.25;
const float CLOUD_BAKED_COARSE_STEP_CELLS = 0.5;
const float CLOUD_BAKED_FINE_TRANSMITTANCE = 0.3;
// [E] How far past a block's exit a skip lands (m, relative to the cell): the next sample lies inside the next block.
const float CLOUD_BAKED_EXIT_NUDGE = 0.01;
// [E] The band's height tolerance (m) for the float32 heights at its crossings.
const float CLOUD_BAKED_BAND_TOLERANCE_M = 2.0;

// The ray's state in one set's grid: its grid coordinates are linear in the distance.
struct CloudBakedRay {
	float gx0;
	float gy0;
	float gdx;
	float gdy;
};

CloudBakedRay cloud_baked_ray(int si, vec3 eye, vec3 dir) {
	CloudBakedRay r;
	vec3 axis_x = baked.sets[si].axis_x.xyz * ATM_STRETCH;
	vec3 axis_y = baked.sets[si].axis_y.xyz * ATM_STRETCH;
	r.gx0 = baked.sets[si].band.z + dot(eye, axis_x);
	r.gy0 = baked.sets[si].band.w + dot(eye, axis_y);
	r.gdx = dot(dir, axis_x);
	r.gdy = dot(dir, axis_y);
	return r;
}

#endif
