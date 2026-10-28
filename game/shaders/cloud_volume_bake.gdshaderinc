// The cloud world volume's upkeep kernels (world-volume plan Task 5; CloudWorldVolume): GLSL 450 only, compiled after
// the march's files and cloud_volume.gdshaderinc (set 0 is the march's: the snapshots' field textures, the noise, the
// tables). Kernels:
//   CLOUD_VOLUME_BAKE       the density of a box of cells of one level (one invocation per cell): per snapshot of the
//                           cross-fade pair rho and sigma_c, the mean of the point model (no detail) over the cell's
//                           taps - the GPU mirror of core cloud_volume_bake.hpp (bake_cloud_cell, the tap rule
//                           cloud_cell_bake_taps; change together). The cell is the parallelepiped centre + u e_x +
//                           v e_y + w e_z (u, v, w in [-1/2, 1/2]) with the grid's own axes at its centre: e_x / e_y
//                           the differences of the grid points half a cell east / west and north / south, e_z the
//                           geodetic normal times the cell's height. A snapshot whose field has no cloud in any texel the cell's
//                           taps can read (bilinear support in latitude / longitude, the two layer centres around the
//                           cell's heights) is 0 there exactly - without its taps.
//   CLOUD_VOLUME_OCCUPANCY  pass 0: the max-extinction mip of a box of blocks (CLOUD_VOLUME_MIP_DIV^3 cells): the
//                           largest sigma_c over the block's cells and the one-cell ring around them (the trilinear
//                           filter's support: the exact upper bound of the filtered field over the block) where any of
//                           them has rho > 0 in either snapshot, else 0 - the bound of every sample the sub-cell field
//                           can make (it redistributes the cell's water, its peak is sigma_c: ramp <= 1); with
//                           CLOUD_VOLUME_DISTANCE passes 1-3: the toroidal Chebyshev distance (blocks, capped) to the
//                           nearest block with a bound > 0, separable along x, y, z
//   CLOUD_VOLUME_PROBE      diagnostics (cloud_check.gd section 16): cloud_volume_grid of camera offsets, a texture's
//                           raw density at grid points

#ifdef CLOUD_VOLUME_OCCUPANCY
layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;
#endif

// ---- The bake's point model ------------------------------------------------------------------------------------
#ifdef CLOUD_VOLUME_BAKE
// The model height |M p| - a of camera + offset (clouds.gdshaderinc cloud_height_along without a ray: never the
// difference of two radii).
float cloud_volume_model_height(vec3 offset_m) {
	vec3 vm = offset_m * ATM_STRETCH;
	vec3 up_c = normalize((atmosphere_camera_hi + atmosphere_camera_lo) * ATM_STRETCH);
	float r_c = CLOUD_A_M + cloud_camera_height_m;
	float t = 2.0 * r_c * dot(up_c, vm) + dot(vm, vm);
	return cloud_camera_height_m + t / (sqrt(max(r_c * r_c + t, 0.0)) + r_c);
}

// The field's texel value at (latitude, longitude) of a layer centre, filtered bilinearly with exact weights (the CPU
// reference's; the hardware's ~8-bit sub-texel weights shift a 1-degree field by up to ~400 m): column c's centre at
// longitude c x 360 / columns (wrapping), row r at latitude 90 - r x 180 / (rows - 1), the second row at the last one
// weighted 0.
vec4 cloud_volume_field_texel(sampler3D field, vec3 size, vec2 lat_lon, int layer) {
	float x = lat_lon.y / (2.0 * CLOUD_PI) * size.x;
	float y = clamp((0.5 * CLOUD_PI - lat_lon.x) / (CLOUD_PI / (size.y - 1.0)), 0.0, size.y - 1.0);
	float x0 = floor(x);
	float y0 = floor(y);
	int columns = int(size.x);
	int c0 = cloud_volume_wrap1(int(x0), columns);
	int c1 = c0 + 1 < columns ? c0 + 1 : 0;
	int r0 = int(y0);
	int r1 = min(r0 + 1, int(size.y) - 1);
	vec2 f = vec2(x - x0, y - y0);
	vec4 t00 = texelFetch(field, ivec3(c0, r0, layer), 0);
	vec4 t10 = texelFetch(field, ivec3(c1, r0, layer), 0);
	vec4 t01 = texelFetch(field, ivec3(c0, r1, layer), 0);
	vec4 t11 = texelFetch(field, ivec3(c1, r1, layer), 0);
	return mix(mix(t00, t10, f.x), mix(t01, t11, f.x), f.y);
}

// One snapshot at a point (clouds.gdshaderinc cloud_snapshot_cell, the terms the bake needs: fraction, grid mean,
// ice share, stratiform weight) with the exact bilinear weights of cloud_volume_field_texel.
CloudCell cloud_volume_snapshot_cell(sampler3D field, sampler3D ice_share, sampler2D base, vec3 size, vec2 lat_lon,
		float height_m) {
	float s = cloud_layer_coordinate(height_m) - 0.5;
	float k = clamp(floor(s), 0.0, CLOUD_LAYERS - 2.0);
	float x = clamp(s - k, 0.0, 1.0);
	vec4 below = cloud_volume_field_texel(field, size, lat_lon, int(k));
	vec4 above = cloud_volume_field_texel(field, size, lat_lon, int(k) + 1);
	float gap = cloud_layer_centre_m(k + 1.0) - cloud_layer_centre_m(k);
	CloudCell c;
	c.fraction = cloud_profile_value(below.r, above.r, below.r, above.r, x, gap);
	c.extinction = cloud_profile_value(cloud_decode_extinction(below), cloud_decode_extinction(above), below.r,
			above.r, x, gap);
	vec2 lcl_cut = cloud_lcl_cut(base, size, lat_lon, height_m);
	c.fraction *= lcl_cut.x;
	c.extinction *= lcl_cut.y;
	float si = clamp(s, 0.0, CLOUD_LAYERS - 1.0);
	float li = min(floor(si), CLOUD_LAYERS - 1.0);
	float fi = si - li;
	c.ice = cloud_volume_field_texel(ice_share, size, lat_lon, int(li)).r;
	if (fi > 0.0) {
		c.ice = mix(c.ice, cloud_volume_field_texel(ice_share, size, lat_lon, int(li) + 1).r, fi);
	}
	float above_next = k + 2.0 < CLOUD_LAYERS ? cloud_volume_field_texel(field, size, lat_lon, int(k) + 2).r : above.r;
	float share_below = (1.0 - x) * below.r;
	float share_above = x * above.r;
	c.stratiform = share_below + share_above > 0.0
			? (share_below * cloud_stratiform_weight(below.r, above.r)
					+ share_above * cloud_stratiform_weight(above.r, above_next)) / (share_below + share_above)
			: 0.0;
	c.depth_above = 0.0;
	c.in_cloud_above = 0.0;
	return c;
}

#endif

// ---- The bake ------------------------------------------------------------------------------------------------------
#ifdef CLOUD_VOLUME_BAKE
// One dispatch's box (CloudWorldVolume._job_bytes).
layout(set = 1, binding = 1, std140) uniform CloudVolumeJob {
	ivec4 first;  // xyz: the box's first cell (global cells), w: the level
	ivec4 size;   // xyz: cells, w: channel pair (-1 both snapshots, 0 / 1 that slot's alone)
	ivec4 info;   // x: frame index, y: 1 = both slots hold one snapshot (evaluated once, stored twice)
} job;
layout(rgba16f, set = 1, binding = 2) uniform image3D volume_density;

// The tap rule (core cloud_volume_bake.hpp cloud_cell_bake_taps and its constants; change together).
const float CLOUD_BAKE_HORIZONTAL_SPACING_M = 200.0;
const float CLOUD_BAKE_MIN_HORIZONTAL_TAPS = 4.0;
const float CLOUD_BAKE_MAX_HORIZONTAL_TAPS = 8.0;
const float CLOUD_BAKE_WIDEST_CELL_M = 4800.0;
const float CLOUD_BAKE_WIDEST_CELL_HORIZONTAL_TAPS = 16.0;
const float CLOUD_BAKE_VERTICAL_SPACING_M = 5.0;
const float CLOUD_BAKE_MIN_VERTICAL_TAPS = 16.0;
const float CLOUD_BAKE_TAP_COUNT_SLACK = 0.01;
// [S] The edge mean factor's floor (core cloud_render_field.cpp edge_mean_floor; clouds.gdshaderinc max(m, 1e-4)).
const float CLOUD_BAKE_EDGE_MEAN_FLOOR = 1.0e-4;
// [E] The clear test's vertical margin: the cell's model heights from its corners plus this share of its height (the
// model height differs from the grid's geodetic one by <= 0.34 %, the box's curvature by < 1 m).
const float CLOUD_BAKE_HEIGHT_MARGIN = 0.01;
const float CLOUD_BAKE_HEIGHT_MARGIN_M = 2.0;
// [E] Beyond this many field columns in a cell's support (near the poles) the clear test gives up (taps evaluated).
const int CLOUD_BAKE_MAX_SUPPORT_COLUMNS = 6;

// True when no texel of the snapshot's field the cell's taps can read has cloud (R = 0: a point's fraction and
// extinction are then 0 - the profile rule and the bilinear filter of zeros). corners_ll: (latitude, longitude) of the
// cell's eight corners; h_range: its model heights.
bool cloud_volume_snapshot_clear(sampler3D field, vec3 size, vec2 corners_ll[8], vec2 centre_ll, vec2 h_range) {
	if (h_range.x > CLOUD_TOP_M) {
		return true;
	}
	vec2 x_range = vec2(1.0e30, -1.0e30);
	vec2 y_range = vec2(1.0e30, -1.0e30);
	for (int i = 0; i < 8; i++) {
		// the longitude unwrapped about the centre's
		float lon = centre_ll.y + mod(corners_ll[i].y - centre_ll.y + CLOUD_PI, 2.0 * CLOUD_PI) - CLOUD_PI;
		float x = lon / (2.0 * CLOUD_PI) * size.x;  // column c's centre at longitude c x 360 / columns
		float y = clamp((0.5 * CLOUD_PI - corners_ll[i].x) / (CLOUD_PI / (size.y - 1.0)), 0.0, size.y - 1.0);
		x_range = vec2(min(x_range.x, x), max(x_range.y, x));
		y_range = vec2(min(y_range.x, y), max(y_range.y, y));
	}
	int c0 = int(floor(x_range.x));
	int c1 = int(floor(x_range.y)) + 1;
	if (c1 - c0 + 1 > CLOUD_BAKE_MAX_SUPPORT_COLUMNS) {
		return false;
	}
	int rows = int(size.y);
	int columns = int(size.x);
	int r0 = int(floor(y_range.x));
	int r1 = min(int(floor(y_range.y)) + 1, rows - 1);
	float s0 = cloud_layer_coordinate(h_range.x) - 0.5;
	float s1 = cloud_layer_coordinate(h_range.y) - 0.5;
	int k0 = int(clamp(floor(s0), 0.0, CLOUD_LAYERS - 2.0));
	int k1 = int(clamp(floor(s1), 0.0, CLOUD_LAYERS - 2.0)) + 1;
	for (int k = k0; k <= k1; k++) {
		for (int r = r0; r <= r1; r++) {
			for (int c = c0; c <= c1; c++) {
				if (texelFetch(field, ivec3(cloud_volume_wrap1(c, columns), r, k), 0).r > 0.0) {
					return false;
				}
			}
		}
	}
	return true;
}

// The shape value of a cloud type at the tap (the unfiltered texture, or its prefilter level), fetched once per tap and
// type for both snapshots (have: bit per type).
vec4 cloud_volume_shape(int type, vec3 offset_m, float height_m, float level, inout vec4 shapes[3], inout int have) {
	if ((have & (1 << type)) == 0) {
		vec3 q = cloud_type_shape_coordinates(type, offset_m, height_m);
		shapes[type] = level > 0.0 ? cloud_shape_prefiltered(q, level) : texture(cloud_noise_shape, q);
		have |= 1 << type;
	}
	return shapes[type];
}

// One snapshot's point model at a tap (core cloud_point_extinction_at_level): x the extinction, y the in-cloud
// extinction grid mean / max(m, floor) [1/m]; 0 where the fraction is below CLOUD_MIN_FRACTION or the grid mean is 0.
vec2 cloud_volume_point(CloudCell cell, vec3 offset_m, float height_m, float level, inout vec4 shapes[3],
		inout int have) {
	if (height_m > CLOUD_TOP_M || cell.fraction < CLOUD_MIN_FRACTION || !(cell.extinction > 0.0)) {
		return vec2(0.0);
	}
	float threshold;
	float m;
	if (level > 0.0) {
		vec2 uv = cloud_prefilter_uv(cell.fraction, level);
		threshold = texture(cloud_prefilter_threshold, uv).r;
		m = texture(cloud_prefilter_edge_mean, uv).r;
	} else {
		vec2 uv = cloud_table_uv(cell.fraction, 0.0);
		threshold = texture(cloud_coverage_threshold, uv).r;
		m = texture(cloud_edge_mean, uv).r;
	}
	vec3 types = cloud_type_weights(cell);
	float ramp = 0.0;
	for (int type = 0; type < 3; type++) {
		if (types[type] > 0.0) {
			vec4 s = cloud_volume_shape(type, offset_m, height_m, level, shapes, have);
			ramp += types[type] * cloud_edge_ramp(s.r, threshold, s.a);
		}
	}
	float in_cloud = cell.extinction / max(m, CLOUD_BAKE_EDGE_MEAN_FLOOR);
	return vec2(ramp > 0.0 ? in_cloud * ramp : 0.0, in_cloud);
}

// Taps per horizontal axis (x), vertically (y) and the prefilter level (z) of a cell h_m across and v_m high (core
// cloud_cell_bake_taps).
vec3 cloud_volume_taps(float h_m, float v_m) {
	float nv = max(CLOUD_BAKE_MIN_VERTICAL_TAPS,
			ceil(v_m / CLOUD_BAKE_VERTICAL_SPACING_M * (1.0 - CLOUD_BAKE_TAP_COUNT_SLACK)));
	float cap = h_m > CLOUD_BAKE_WIDEST_CELL_M ? CLOUD_BAKE_WIDEST_CELL_HORIZONTAL_TAPS : CLOUD_BAKE_MAX_HORIZONTAL_TAPS;
	float nh = clamp(ceil(h_m / CLOUD_BAKE_HORIZONTAL_SPACING_M * (1.0 - CLOUD_BAKE_TAP_COUNT_SLACK)),
			CLOUD_BAKE_MIN_HORIZONTAL_TAPS, cap);
	float texel_m = CLOUD_SHAPE_TILE_H_M / CLOUD_SHAPE_TEXELS;
	float level = clamp(log2(h_m / nh / texel_m), 0.0, CLOUD_SHAPE_PREFILTER_LEVELS);
	return vec3(nh, nv, level);
}

// One workgroup per cell: its CLOUD_BAKE_GROUP threads share the taps (tap i to thread i mod the group) and sum them
// in shared memory in a fixed order (a rebake is bit-identical) - a level-7 cell's 8448 taps take one thread ~6 ms,
// so a small batch (a fill's budget, a slab) would idle the GPU and overrun the frame's budget by its latency.
const int CLOUD_BAKE_GROUP = 64;
layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
shared vec4 cloud_bake_partial[CLOUD_BAKE_GROUP];

void main() {
	ivec3 g = ivec3(gl_WorkGroupID);
	int lane = int(gl_LocalInvocationID.x);
	if (any(greaterThanEqual(g, job.size.xyz))) {
		return;  // (uniform over the group)
	}
	int k = job.first.w;
	int f = job.info.x;
	ivec3 c = job.first.xyz + g;
	float h = CLOUD_VOLUME_CELL_H_M[k];
	float v = CLOUD_VOLUME_CELL_V_M[k];
	CloudVolumeFrame fr = cloud_volume.frames[f];
	// The cell's centre relative to the camera's grid point: (c + 1/2) h is exact in float32, the camera's hi + lo.
	vec3 centre_grid = vec3((float(c.x) + 0.5) * h, (float(c.y) + 0.5) * h,
			CLOUD_VOLUME_SHELL_BOTTOM_M + (float(c.z) + 0.5) * v);
	vec3 dg = (centre_grid - fr.grid_hi.xyz) - fr.grid_lo.xyz;
	vec3 normal;
	vec3 centre = cloud_volume_offset_of(f, dg, normal);
	vec3 unused;
	vec3 ex = cloud_volume_offset_of(f, dg + vec3(0.5 * h, 0.0, 0.0), unused)
			- cloud_volume_offset_of(f, dg - vec3(0.5 * h, 0.0, 0.0), unused);
	vec3 ey = cloud_volume_offset_of(f, dg + vec3(0.0, 0.5 * h, 0.0), unused)
			- cloud_volume_offset_of(f, dg - vec3(0.0, 0.5 * h, 0.0), unused);
	vec3 ez = normalize(normal) * v;
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	// The clear tests: the eight corners' (latitude, longitude) and the cell's model heights.
	vec2 corners[8];
	vec2 h_range = vec2(1.0e30, -1.0e30);
	for (int i = 0; i < 8; i++) {
		vec3 p = centre + 0.5 * (ex * float((i & 1) * 2 - 1) + ey * float(((i >> 1) & 1) * 2 - 1)
				+ ez * float(((i >> 2) & 1) * 2 - 1));
		corners[i] = cloud_lat_lon(camera + p);
		float hm = cloud_volume_model_height(p);
		h_range = vec2(min(h_range.x, hm), max(h_range.y, hm));
	}
	float margin = CLOUD_BAKE_HEIGHT_MARGIN * v + CLOUD_BAKE_HEIGHT_MARGIN_M;
	h_range += vec2(-margin, margin);
	vec2 centre_ll = cloud_lat_lon(camera + centre);
	bool same = job.info.y != 0;
	int channel = job.size.w;
	bool clear0 = channel == 1 || cloud_volume_snapshot_clear(cloud_field_before, cloud_field_before_size, corners,
			centre_ll, h_range);
	bool clear1 = same || channel == 0 || cloud_volume_snapshot_clear(cloud_field_after, cloud_field_after_size,
			corners, centre_ll, h_range);
	vec4 sum = vec4(0.0);  // rho, sigma_c of snapshot 0, of snapshot 1
	if (!(clear0 && clear1)) {
		vec3 taps = cloud_volume_taps(max(length(ex), length(ey)), length(ez));
		int nh = int(taps.x);
		int nv = int(taps.y);
		float level = taps.z;
		int count = nh * nh * nv;
		for (int t = lane; t < count; t += CLOUD_BAKE_GROUP) {
			int i = t % nh;
			int j = (t / nh) % nh;
			int kz = t / (nh * nh);
			vec3 p = centre + ex * ((float(i) + 0.5) / float(nh) - 0.5) + ey * ((float(j) + 0.5) / float(nh) - 0.5)
					+ ez * ((float(kz) + 0.5) / float(nv) - 0.5);
			float hm = cloud_volume_model_height(p);
			vec2 lat_lon = cloud_lat_lon(camera + p);
			vec4 shapes[3] = vec4[3](vec4(0.0), vec4(0.0), vec4(0.0));
			int have = 0;
			if (!clear0) {
				CloudCell cell = cloud_volume_snapshot_cell(cloud_field_before, cloud_ice_before, cloud_base_before,
						cloud_field_before_size, lat_lon, hm);
				sum.xy += cloud_volume_point(cell, p, hm, level, shapes, have);
			}
			if (!clear1) {
				CloudCell cell = cloud_volume_snapshot_cell(cloud_field_after, cloud_ice_after, cloud_base_after,
						cloud_field_after_size, lat_lon, hm);
				sum.zw += cloud_volume_point(cell, p, hm, level, shapes, have);
			}
		}
		sum /= float(count);
	}
	// The group's sum (every thread reaches the barriers: the clear decision is the same for all).
	cloud_bake_partial[lane] = sum;
	barrier();
	for (int stride = CLOUD_BAKE_GROUP / 2; stride > 0; stride /= 2) {
		if (lane < stride) {
			cloud_bake_partial[lane] += cloud_bake_partial[lane + stride];
		}
		barrier();
	}
	if (lane != 0) {
		return;
	}
	sum = cloud_bake_partial[0];
	if (same) {
		sum.zw = sum.xy;
	}
	vec4 value = sum * CLOUD_VOLUME_PER_KM;
	ivec3 texel = cloud_volume_wrap(c, CLOUD_VOLUME_SIZE);
	if (channel >= 0) {  // one slot's channel pair: the other pair kept
		vec4 old = imageLoad(volume_density, texel);
		value = channel == 0 ? vec4(value.xy, old.zw) : vec4(old.xy, value.zw);
	}
	imageStore(volume_density, texel, value);
}
#endif

// ---- The max-extinction mip and the distance field -----------------------------------------------------------------
#ifdef CLOUD_VOLUME_OCCUPANCY
layout(set = 1, binding = 1, std140) uniform CloudVolumeOccupancy {
	ivec4 first;  // xyz: pass 0 the box's first block (global blocks); w: the pass
	ivec4 size;   // xyz: blocks of this dispatch; w: the distance's cap (blocks)
} occ;
layout(set = 1, binding = 2) uniform sampler3D occupancy_in;  // pass 0 the density, else the previous pass (nearest)
#ifdef CLOUD_VOLUME_DISTANCE
layout(r8, set = 1, binding = 3) uniform writeonly image3D occupancy_out;
#else
layout(r16f, set = 1, binding = 3) uniform writeonly image3D occupancy_out;
#endif

// [E] Upward rounding of the stored bound: a half float rounds to nearest (relative error <= 2^-11); this factor keeps
// the stored value above the largest texel and above the filter's convex combinations of them.
const float CLOUD_VOLUME_BOUND_ROUNDING = 1.0 + 1.0 / 1024.0;

void main() {
	ivec3 g = ivec3(gl_GlobalInvocationID);
	if (any(greaterThanEqual(g, occ.size.xyz))) {
		return;
	}
#ifndef CLOUD_VOLUME_DISTANCE
	ivec3 block = occ.first.xyz + g;
	ivec3 base = block * CLOUD_VOLUME_MIP_DIV;
	float rho = 0.0;
	float sigma = 0.0;
	for (int k = -1; k <= CLOUD_VOLUME_MIP_DIV; k++) {
		for (int j = -1; j <= CLOUD_VOLUME_MIP_DIV; j++) {
			for (int i = -1; i <= CLOUD_VOLUME_MIP_DIV; i++) {
				vec4 d = texelFetch(occupancy_in, cloud_volume_wrap(base + ivec3(i, j, k), CLOUD_VOLUME_SIZE), 0);
				rho = max(rho, max(d.x, d.z));
				sigma = max(sigma, max(max(d.y, d.w), max(d.x, d.z)));
			}
		}
	}
	float bound = rho > 0.0 ? sigma * CLOUD_VOLUME_BOUND_ROUNDING : 0.0;
	imageStore(occupancy_out, cloud_volume_wrap(block, CLOUD_VOLUME_MIP_SIZE), vec4(bound));
#else
	int pass = occ.first.w;
	int cap = occ.size.w;
	ivec3 axis = pass == 1 ? ivec3(1, 0, 0) : (pass == 2 ? ivec3(0, 1, 0) : ivec3(0, 0, 1));
	float distance = float(cap + 1);
	for (int j = -cap; j <= cap; j++) {
		float value = texelFetch(occupancy_in, cloud_volume_wrap(g + axis * j, CLOUD_VOLUME_MIP_SIZE), 0).r;
		float previous = pass == 1 ? (value > 0.0 ? 0.0 : float(cap + 1)) : round(value * CLOUD_VOLUME_DISTANCE_SCALE);
		distance = min(distance, max(float(abs(j)), previous));
	}
	imageStore(occupancy_out, g, vec4(distance / CLOUD_VOLUME_DISTANCE_SCALE));
#endif
}
#endif

// ---- Diagnostics: the probe ---------------------------------------------------------------------------------------
#ifdef CLOUD_VOLUME_PROBE
layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
layout(set = 1, binding = 1, std140) uniform CloudVolumeProbe {
	ivec4 info;  // x: points, y: mode (0 the grid of camera offsets, 1 a texture's raw density), z: texture, w: level
} probe;
layout(set = 1, binding = 2, std430) readonly buffer CloudVolumeProbeIn {
	vec4 probe_in[];
};
layout(set = 1, binding = 3, std430) writeonly buffer CloudVolumeProbeOut {
	vec4 probe_out[];
};

void main() {
	int i = int(gl_GlobalInvocationID.x);
	if (i >= probe.info.x) {
		return;
	}
	vec3 p = probe_in[i].xyz;
	if (probe.info.y == 0) {
		probe_out[i] = vec4(cloud_volume_grid_in(int(probe_in[i].w), p), 0.0);
	} else {
		probe_out[i] = cloud_volume_fetch_density(probe.info.z, probe.info.w, p);
	}
}
#endif
