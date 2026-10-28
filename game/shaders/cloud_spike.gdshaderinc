// World volume plan Task 2, spike S (throw-away: removed in Task 13): the far tier alone, marched over scratch volumes,
// to measure whether a baked density can make the far tier fit the VR budget. GLSL 450 only - compiled by CloudSpike
// (game/world/weather/cloud_spike.gd) after CloudEffect's march files, never included by a Godot shader. Kernels:
//   CLOUD_SPIKE_OCCUPANCY  the exact occupancy over the view: pass 0 evaluates every cell densely (taps), passes 1-3 turn
//                          it into the Chebyshev distance field (separable along x, y, z) and dilate the max extinction
//   CLOUD_SPIKE_BAKE       pass 0: the single-level bake of the meso density (the cell's mean extinction); pass 1: the
//                          dummy light volume (RGBA16F, 1.0 everywhere)
//   CLOUD_SPIKE_FAR        (with CLOUD_MARCH): the march of [R_near, end] from the cyclops point (cloud_pass.gdshaderinc's
//                          main calls cloud_spike_shade), variant (a) the per-sample procedural density
//                          (cloud_extinction_at at the pixel footprint) or (b) one fetch of the bake plus one dummy
//                          light-volume fetch per step
// The volumes share one grid: horizontal axes azimuthal-equidistant about the camera's model-space up up0 (metres on the
// ellipsoid's equatorial sphere of the stretched model space, axes axis_x / axis_y), vertical the model height
// (clouds.gdshaderinc cloud_height_of) over the band [z_lo, z_hi]. D3D12 caps a 3-D texture at 2048 texels per axis, so
// the horizontal plane is cut into tiles of 2048 texels (one texel of overlap: hardware filtering across a seam) and the
// tiles are stacked along the texture's depth, each with its nz cells between two zero guard slices.

#if defined(CLOUD_SPIKE_OCCUPANCY) || defined(CLOUD_SPIKE_BAKE) || defined(CLOUD_SPIKE_FAR)

#ifdef CLOUD_SPIKE_FAR
#define CLOUD_SPIKE_GRID_BINDING 33
#else
#define CLOUD_SPIKE_GRID_BINDING 0
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
#endif

// The grid (CloudSpike._grid_block): std140, vec4 / ivec4 only.
layout(set = 1, binding = CLOUD_SPIKE_GRID_BINDING, std140) uniform CloudSpikeGrid {
	vec4 up0;       // xyz: model-space unit up at the grid's origin; w: occupancy cell (m, horizontal and vertical)
	vec4 axis_x;    // xyz: the horizontal x axis (the view's forward azimuth); w: x of the grid's first cell edge (m)
	vec4 axis_y;    // xyz: up0 x axis_x; w: y of the first cell edge (m)
	vec4 band;      // x: z_lo, y: z_hi (model heights, m), z: bake cell (m), w: R_near (m)
	ivec4 occ;      // occupancy cells nx, ny, nz; w: tiles along x
	ivec4 occ_tex;  // its texture W, H, D; w: tiles along y
	ivec4 bake;     // bake cells nx, ny, nz; w: tiles along x
	ivec4 bake_tex; // its texture W, H, D; w: tiles along y
	ivec4 build;    // x: pass, y: first texture slice of this dispatch, z: variant (1 a, 2 b), w: distance cap (cells)
	vec4 view;      // x: the centre view's pixel angle (rad: the footprint's band limit)
} spike;

// A tile's texels per axis (D3D12_REQ_TEXTURE3D_U_V_OR_W_DIMENSION) and its stride: neighbours share one texel.
const int CLOUD_SPIKE_TILE = 2048;
const int CLOUD_SPIKE_STRIDE = CLOUD_SPIKE_TILE - 1;
// [E] Stored extinction unit: 1/km (a half float keeps 3 digits from 6e-5 up; 1/m values sat near its subnormals).
const float CLOUD_SPIKE_PER_KM = 1000.0;

// The texel of cell c (integer cells) in a tiled volume (cells, texture).
ivec3 cloud_spike_texel(ivec3 c, ivec4 cells, ivec4 tex) {
	int kx = min(c.x / CLOUD_SPIKE_STRIDE, cells.w - 1);
	int ky = min(c.y / CLOUD_SPIKE_STRIDE, tex.w - 1);
	int slot = ky * cells.w + kx;
	return ivec3(c.x - kx * CLOUD_SPIKE_STRIDE, c.y - ky * CLOUD_SPIKE_STRIDE, slot * (cells.z + 2) + 1 + c.z);
}

// The normalised texture coordinate of continuous cell coordinates g (cell i spans [i, i + 1]) for a filtered fetch:
// the tile holding g's filter footprint, z clamped to the guard slices (zero: no cloud beyond the band).
vec3 cloud_spike_uvw(vec3 g, ivec4 cells, ivec4 tex) {
	g.xy = clamp(g.xy, vec2(0.5), vec2(cells.xy) - 0.5);
	g.z = clamp(g.z, 0.0, float(cells.z));
	int kx = clamp(int(floor((g.x - 0.5) / float(CLOUD_SPIKE_STRIDE))), 0, cells.w - 1);
	int ky = clamp(int(floor((g.y - 0.5) / float(CLOUD_SPIKE_STRIDE))), 0, tex.w - 1);
	int slot = ky * cells.w + kx;
	vec3 t = vec3(g.x - float(kx * CLOUD_SPIKE_STRIDE), g.y - float(ky * CLOUD_SPIKE_STRIDE),
			float(slot * (cells.z + 2) + 1) + g.z);
	return t / vec3(tex.xyz);
}

// The grid's horizontal coordinates (m) of a model-space unit up: azimuthal-equidistant about up0.
vec2 cloud_spike_map(vec3 u) {
	float x = dot(u, spike.axis_x.xyz);
	float y = dot(u, spike.axis_y.xyz);
	float s = length(vec2(x, y));
	float theta = atan(s, dot(u, spike.up0.xyz));
	return s > 0.0 ? vec2(x, y) * (CLOUD_A_M * theta / s) : vec2(0.0);
}

// The camera offset (Earth-local, m) of grid point xy (m) at model height h_m.
vec3 cloud_spike_offset_of(vec2 xy, float h_m) {
	float rho = length(xy);
	float theta = rho / CLOUD_A_M;
	vec3 side = rho > 0.0 ? (xy.x * spike.axis_x.xyz + xy.y * spike.axis_y.xyz) * (sin(theta) / rho) : vec3(0.0);
	vec3 u = cos(theta) * spike.up0.xyz + side;
	return u * (CLOUD_A_M + h_m) / ATM_STRETCH - (atmosphere_camera_hi + atmosphere_camera_lo);
}

#endif

// ---- The occupancy: pass 0 taps, passes 1-3 the distance field ----------------------------------------------------
#ifdef CLOUD_SPIKE_OCCUPANCY
layout(rg16f, set = 1, binding = 1) uniform writeonly image3D spike_out;
layout(set = 1, binding = 2) uniform sampler3D spike_in;  // the previous pass's result (nearest)

// [E] Taps per axis of a cell (50 m apart in a 200 m cell: the shape's texels are 250 m across and 125 m high,
// trilinear; features narrower than the taps' spacing fall between them - the one-cell dilation of pass 1-3 covers a
// cloud whose neighbour cell was found).
const int CLOUD_SPIKE_OCCUPANCY_TAPS = 4;

// Max extinction (1/m) of cell c over its taps, exact for this view: each tap's extinction as the march samples it
// (cloud_extinction_at with the detail band-limited at the tap's own pixel footprint, pixel angle x its distance from
// the cyclops point). occupied: a tap has extinction, or its shape lifted by the detail's largest lift at that footprint
// (cloud_detail_lift_bound) reaches the threshold - cloud between the taps.
float cloud_spike_cell_max(ivec3 c, out bool occupied) {
	occupied = false;
	float largest = 0.0;
	float cell_m = spike.up0.w;
	vec3 base = vec3(spike.axis_x.w, spike.axis_y.w, spike.band.x) + vec3(c) * cell_m;
	for (int k = 0; k < CLOUD_SPIKE_OCCUPANCY_TAPS; k++) {
		for (int j = 0; j < CLOUD_SPIKE_OCCUPANCY_TAPS; j++) {
			for (int i = 0; i < CLOUD_SPIKE_OCCUPANCY_TAPS; i++) {
				vec3 g = base + (vec3(i, j, k) + 0.5) * (cell_m / float(CLOUD_SPIKE_OCCUPANCY_TAPS));
				vec3 offset = cloud_spike_offset_of(g.xy, g.z);
				CloudCell cell = cloud_field(cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + offset), g.z);
				if (cell.fraction < CLOUD_MIN_FRACTION || cell.extinction <= 0.0) {
					continue;
				}
				float footprint_m = spike.view.x * length(offset);
				float in_cloud;
				float beta = cloud_extinction_in_cell(cell, offset, g.z, footprint_m, 0.0, in_cloud);
				largest = max(largest, beta);
				if (beta > 0.0) {
					occupied = true;
					continue;
				}
				float state = cloud_detail_state(footprint_m);
				vec3 types = cloud_type_weights(cell);
				float shape_max = -1.0;
				for (int type = 0; type < 3; type++) {
					if (types[type] > 0.0) {
						shape_max = max(shape_max,
								texture(cloud_noise_shape, cloud_type_shape_coordinates(type, offset, g.z)).r);
					}
				}
				occupied = occupied || shape_max + cloud_detail_lift_bound(state) > cloud_threshold(cell.fraction, state);
			}
		}
	}
	return largest;
}

void main() {
	ivec3 t = ivec3(gl_GlobalInvocationID.xy, int(gl_GlobalInvocationID.z) + spike.build.y);
	if (any(greaterThanEqual(t, spike.occ_tex.xyz))) {
		return;
	}
	int slot = t.z / (spike.occ.z + 2);
	ivec3 c = ivec3((slot % spike.occ.w) * CLOUD_SPIKE_STRIDE + t.x, (slot / spike.occ.w) * CLOUD_SPIKE_STRIDE + t.y,
			t.z % (spike.occ.z + 2) - 1);
	if (c.z < 0 || c.z >= spike.occ.z || slot / spike.occ.w >= spike.occ_tex.w || c.x >= spike.occ.x
			|| c.y >= spike.occ.y) {
		// a guard slice or a texel outside the grid: no cloud (pass 0's flag 0), far from any (the passes' distance)
		imageStore(spike_out, t, vec4(0.0, spike.build.x == 0 ? 0.0 : float(spike.build.w + 1), 0.0, 0.0));
		return;
	}
	int pass = spike.build.x;
	if (pass == 0) {
		bool occupied;
		float largest = cloud_spike_cell_max(c, occupied);
		imageStore(spike_out, t, vec4(largest * CLOUD_SPIKE_PER_KM, occupied ? 1.0 : 0.0, 0.0, 0.0));
		return;
	}
	// The distance (Chebyshev, cells, capped at the cap + 1) along this pass's axis, and the max extinction dilated by
	// one cell along it: pass 1 from the taps' flags, passes 2 and 3 from the previous pass's distances (separable:
	// d(c) = min_j max(|j|, d_prev(c + j e))).
	ivec3 axis = pass == 1 ? ivec3(1, 0, 0) : (pass == 2 ? ivec3(0, 1, 0) : ivec3(0, 0, 1));
	int cap = spike.build.w;
	float distance = float(cap + 1);
	float largest = 0.0;
	for (int j = -cap; j <= cap; j++) {
		ivec3 n = c + axis * j;
		if (any(lessThan(n, ivec3(0))) || any(greaterThanEqual(n, spike.occ.xyz))) {
			continue;
		}
		vec2 v = texelFetch(spike_in, cloud_spike_texel(n, spike.occ, spike.occ_tex), 0).rg;
		float d = pass == 1 ? (v.y > 0.5 ? 0.0 : float(cap + 1)) : v.y;
		distance = min(distance, max(float(abs(j)), d));
		if (abs(j) <= 1) {
			largest = max(largest, v.x);
		}
	}
	imageStore(spike_out, t, vec4(largest, distance, 0.0, 0.0));
}
#endif

// ---- The bake: pass 0 the meso density, pass 1 the dummy light volume ----------------------------------------------
#ifdef CLOUD_SPIKE_BAKE
layout(r16f, set = 1, binding = 1) uniform writeonly image3D spike_bake_out;
layout(rgba16f, set = 1, binding = 3) uniform writeonly image3D spike_light_out;

// [E] Taps per axis of a bake cell (its mean extinction; 33 m apart in a 100 m cell).
const int CLOUD_SPIKE_BAKE_TAPS = 3;

void main() {
	if (spike.build.x == 1) {
		ivec3 t = ivec3(gl_GlobalInvocationID.xy, int(gl_GlobalInvocationID.z) + spike.build.y);
		if (all(lessThan(t, spike.occ_tex.xyz))) {
			imageStore(spike_light_out, t, vec4(1.0));
		}
		return;
	}
	ivec3 t = ivec3(gl_GlobalInvocationID.xy, int(gl_GlobalInvocationID.z) + spike.build.y);
	if (any(greaterThanEqual(t, spike.bake_tex.xyz))) {
		return;
	}
	int slot = t.z / (spike.bake.z + 2);
	ivec3 c = ivec3((slot % spike.bake.w) * CLOUD_SPIKE_STRIDE + t.x,
			(slot / spike.bake.w) * CLOUD_SPIKE_STRIDE + t.y, t.z % (spike.bake.z + 2) - 1);
	if (c.z < 0 || c.z >= spike.bake.z || slot / spike.bake.w >= spike.bake_tex.w || c.x >= spike.bake.x
			|| c.y >= spike.bake.y) {
		imageStore(spike_bake_out, t, vec4(0.0));
		return;
	}
	float cell_m = spike.band.z;
	vec3 base = vec3(spike.axis_x.w, spike.axis_y.w, spike.band.x) + vec3(c) * cell_m;
	float sum = 0.0;
	for (int k = 0; k < CLOUD_SPIKE_BAKE_TAPS; k++) {
		for (int j = 0; j < CLOUD_SPIKE_BAKE_TAPS; j++) {
			for (int i = 0; i < CLOUD_SPIKE_BAKE_TAPS; i++) {
				vec3 g = base + (vec3(i, j, k) + 0.5) * (cell_m / float(CLOUD_SPIKE_BAKE_TAPS));
				vec2 lat_lon;
				float in_cloud;
				// the far tier's density with the detail band-limited at the cell (finer octaves average out over it)
				sum += cloud_extinction_at(cloud_spike_offset_of(g.xy, g.z), g.z, cell_m, 0.0, lat_lon, in_cloud);
			}
		}
	}
	float mean = sum / float(CLOUD_SPIKE_BAKE_TAPS * CLOUD_SPIKE_BAKE_TAPS * CLOUD_SPIKE_BAKE_TAPS);
	imageStore(spike_bake_out, t, vec4(mean * CLOUD_SPIKE_PER_KM));
}
#endif

// ---- The far tier's march -----------------------------------------------------------------------------------------
#ifdef CLOUD_SPIKE_FAR
layout(set = 1, binding = 30) uniform sampler3D spike_occupancy;  // RG16F: dilated max extinction (1/km), distance
layout(set = 1, binding = 31) uniform sampler3D spike_bake;       // R16F: meso extinction (1/km), linear
layout(set = 1, binding = 32) uniform sampler3D spike_light;      // RGBA16F dummy light volume (1.0), linear

// The occupied step's limits (spec D / G): Delta tau per step from the occupancy's max extinction, the fraction of the
// occupancy cell, the floor; the structure term is CLOUD_STRUCTURE_TEXEL_FRACTION of the finest structure at the
// footprint (cloud_spike_structure_m).
const float CLOUD_SPIKE_STEP_TAU = 0.3;
const float CLOUD_SPIKE_STEP_CELL_FRACTION = 0.5;
const float CLOUD_SPIKE_MIN_STEP_M = 2.0;
// [E] Steps per ray at most (the counters show what a ray takes; far above the ~100 the rules talk about).
const int CLOUD_SPIKE_MAX_STEPS = 4096;
// [E] Safety factor on a sphere-tracing step's horizontal reach (the azimuthal-equidistant map stretches lateral
// distances by up to theta / sin(theta) - 1 = 1.1e-3 at 520 km - and the ray's horizontal travel at height h by a / (a
// - 500 m)).
const float CLOUD_SPIKE_TRACE_SAFETY = 0.995;
// [E] The constant light's source per unit Sun irradiance (isotropic: 1 / (4 pi)); colour only - the spike's alpha and
// step counts do not depend on it.
const float CLOUD_SPIKE_SOURCE = 0.0796;

#ifdef CLOUD_COUNTERS
uint cloud_spike_count_extinct = 0u;    // occupied steps whose sample has extinction > 0
uint cloud_spike_count_tau_limit = 0u;  // occupied steps limited by Delta tau
uint cloud_spike_count_structure = 0u;  // occupied steps limited by the structure term
#endif

// The finest structure the footprint f resolves along a ray climbing at climb (|dh/ds|), times
// CLOUD_STRUCTURE_TEXEL_FRACTION: the crossing of one cumulus shape texel (cloud_structure_step's geometry; the slab views
// hold cumulus) or a detail level's middle cell l_k where its weight w_k = clamp(log2(l_k / f) - 1, 0, 1) is above 0
// (cloud_detail_state: l_k > 2 f).
float cloud_spike_structure_m(float footprint_m, float climb) {
	float across = sqrt(max(1.0 - climb * climb, 0.0));
	float finest = 1.0 / max(climb / (CLOUD_SHAPE_TILE_V_M / CLOUD_SHAPE_TEXELS),
			across / (CLOUD_SHAPE_TILE_H_M / CLOUD_SHAPE_TEXELS));
	float middle_cell_m = CLOUD_DETAIL_TILE_M / 4.0;
	for (int k = 0; k < CLOUD_DETAIL_LEVELS; k++) {
		if (k >= cloud_detail_levels) {
			break;
		}
		if (middle_cell_m > 2.0 * footprint_m) {
			finest = min(finest, middle_cell_m);
		}
		middle_cell_m *= 0.25;
	}
	return CLOUD_STRUCTURE_TEXEL_FRACTION * finest;
}

// The next distance > t at which the ray crosses the sphere of model height h_m (1e30: none).
float cloud_spike_next_crossing(CloudRay ray, float t, float h_m) {
	vec2 x = cloud_sphere_crossing(ray, h_m);
	if (x.x > x.y) {
		return 1.0e30;
	}
	return x.x > t ? x.x : (x.y > t ? x.y : 1.0e30);
}

// One ray of the far tier: dir (Earth-local unit) from eye (the cyclops point's offset from the camera) over
// [R_near, min(cloud shell, scene_m)]. Empty space (the distance field's cells >= 2: one cell of dilation) by sphere
// tracing; occupied space in steps of min(Delta tau, half a cell, the structure term) >= 2 m, one sample at each step's
// middle, analytic segment integration of a constant source (Hillaire 2016), today's termination (the scaled
// transmittance, the Sun-ray exception). Out: the stored colour (cloud_stored_color) and the alpha-weighted distance.
void cloud_spike_shade(vec3 dir, vec3 eye, float scene_m, out vec4 color_t, out float cloud_m) {
	cloud_m = 0.0;
	color_t = vec4(0.0, 0.0, 0.0, 1.0);
	if (!cloud_enabled || !atmosphere_enabled) {
		return;
	}
	CloudRay ray = cloud_ray_from(dir, eye);
	vec2 top = cloud_sphere_crossing(ray, CLOUD_TOP_M);
	if (top.x > top.y || top.y <= 0.0) {
		return;
	}
	float d0 = max(top.x, spike.band.w);
	float d1 = top.y;
	vec2 bottom = cloud_sphere_crossing(ray, CLOUD_BOTTOM_M);
	if (bottom.x <= bottom.y) {
		if (ray.h_c >= CLOUD_BOTTOM_M) {
			if (bottom.x > 0.0) {
				d1 = min(d1, bottom.x);
			}
		} else {
			d0 = max(d0, bottom.y);
		}
	}
	d1 = min(d1, scene_m);
	if (d1 <= d0) {
		return;
	}
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	float k = cloud_transport_scale();
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	bool sun_ray = dot(dir, sun_dir) > cos(CLOUD_SUN_RAY_ANGLE);
	float pixel_angle = cloud_view.params.w;
	float z_lo = spike.band.x;
	float z_hi = spike.band.y;
	float cell_m = spike.up0.w;
	vec2 grid_min = vec2(spike.axis_x.w, spike.axis_y.w);
	vec3 source = vec3(-1.0);  // the constant light (set at the first occupied step: one air lookup per ray)
	float transmittance_bg = 1.0;
	float transmittance_sc = 1.0;
	vec3 radiance = vec3(0.0);
	float weighted = 0.0;
	float t = d0;
	for (int i = 0; i < CLOUD_SPIKE_MAX_STEPS && t < d1; i++) {
		float h = cloud_height_along(ray, t);
		if (h > z_hi || h < z_lo) {
			// Outside the band: on to where the ray enters it (no cloud outside: the occupancy's band holds the
			// weather's).
			CLOUD_COUNT_STEP(false)
			float enter = cloud_spike_next_crossing(ray, t, h > z_hi ? z_hi : z_lo);
			if (enter >= d1) {
				break;
			}
			t = enter + CLOUD_SPIKE_MIN_STEP_M * 0.5;
			continue;
		}
		vec3 p = camera + eye + dir * t;
		vec2 xy = cloud_spike_map(normalize(p * ATM_STRETCH));
		vec3 g = vec3((xy - grid_min) / cell_m, (h - z_lo) / cell_m);
		if (any(lessThan(g.xy, vec2(0.0))) || any(greaterThanEqual(g.xy, vec2(spike.occ.xy)))) {
			break;  // beyond the planned region (CloudSpike._plan: the rays' band segments plus a margin)
		}
		ivec3 c = min(ivec3(g), spike.occ.xyz - 1);
		vec2 occupancy = texelFetch(spike_occupancy, cloud_spike_texel(c, spike.occ, spike.occ_tex), 0).rg;
		if (occupancy.y >= 2.0) {
			// Sphere tracing: the cells within distance - 1 of this one are empty (Chebyshev); horizontally that far,
			// vertically up to the box's height bounds (exact crossings: the shells curve away from a straight ray).
			CLOUD_COUNT_STEP(false)
			float reach = occupancy.y - 1.0;
			float step_h = CLOUD_SPIKE_TRACE_SAFETY * reach * cell_m;
			float box_lo = z_lo + (float(c.z) - reach) * cell_m;
			float box_hi = z_lo + (float(c.z) + 1.0 + reach) * cell_m;
			float exit_v = min(cloud_spike_next_crossing(ray, t, box_lo), cloud_spike_next_crossing(ray, t, box_hi));
			t += max(min(step_h, exit_v - t), CLOUD_SPIKE_MIN_STEP_M);
			continue;
		}
		// Occupied space (the occupancy dilated by one cell).
		CLOUD_COUNT_STEP(true)
		float footprint_m = pixel_angle * t;
		float climb = abs(cloud_height_slope(ray, t));
		float max_extinction = occupancy.x / CLOUD_SPIKE_PER_KM;
		float step_tau = max_extinction > 0.0 ? CLOUD_SPIKE_STEP_TAU / max_extinction : 1.0e30;
		float step_structure = cloud_spike_structure_m(footprint_m, climb);
		float step_m = max(min(min(step_tau, CLOUD_SPIKE_STEP_CELL_FRACTION * cell_m), step_structure),
				CLOUD_SPIKE_MIN_STEP_M);
#ifdef CLOUD_COUNTERS
		if (step_tau <= step_m) {
			cloud_spike_count_tau_limit++;
		} else if (step_structure <= step_m) {
			cloud_spike_count_structure++;
		}
#endif
		step_m = min(step_m, d1 - t);
		float d = t + 0.5 * step_m;
		float height = cloud_height_along(ray, d);
		float beta;
#ifdef CLOUD_ABLATE_DENSITY
		beta = 0.5 * max_extinction;  // (ablation: no density fetch - a constant from the occupancy texel already read)
#else
		if (spike.build.z == 2) {
			vec3 q = camera + eye + dir * d;
			vec2 xy_d = cloud_spike_map(normalize(q * ATM_STRETCH));
			vec3 g_d = vec3((xy_d - grid_min) / spike.band.z, (height - z_lo) / spike.band.z);
			beta = texture(spike_bake, cloud_spike_uvw(g_d, spike.bake, spike.bake_tex)).r / CLOUD_SPIKE_PER_KM;
			CLOUD_COUNT_FETCHES(1)
		} else {
			vec2 lat_lon;
			float in_cloud;
			beta = cloud_extinction_at(eye + dir * d, height, footprint_m, 0.0, lat_lon, in_cloud);
		}
#endif
#ifdef CLOUD_COUNTERS
		if (beta > 0.0) {
			cloud_spike_count_extinct++;
		}
#endif
		float light = 1.0;
#ifndef CLOUD_ABLATE_LIGHT
		if (spike.build.z == 2) {
			// The dummy light-volume fetch (b): one RGBA16F texel per step (the occupancy's grid), its value 1.
			CLOUD_LIGHT_BEGIN
			light = texture(spike_light, cloud_spike_uvw(g, spike.occ, spike.occ_tex)).r;
			CLOUD_COUNT_FETCHES(1)
			CLOUD_LIGHT_END
		}
#endif
		t += step_m;
		if (beta <= 0.0) {
			continue;
		}
		if (source.x < 0.0) {
			source = vec3(ATM_PI * CLOUD_SPIKE_SOURCE) * atm_sun_at(camera + eye + dir * d);
		}
		float tau = beta * step_m;
		float scaled_step = exp(-k * tau);
		float true_step = exp(-tau);
		if (transmittance_sc >= CLOUD_MIN_SCALED_TRANSMITTANCE) {
			radiance += transmittance_sc * source * light * (1.0 - scaled_step) / k;
		}
		transmittance_sc *= scaled_step;
		float next_bg = transmittance_bg * true_step;
		weighted += d * (transmittance_bg - next_bg);
		transmittance_bg = next_bg;
		if (transmittance_sc < CLOUD_MIN_SCALED_TRANSMITTANCE
				&& !(sun_ray && transmittance_bg > CLOUD_SUN_MIN_TRANSMITTANCE)) {
			break;
		}
	}
	float alpha = 1.0 - transmittance_bg;
	cloud_m = alpha > 1.0e-6 ? weighted / alpha : 0.0;
	color_t = cloud_stored_color(radiance, transmittance_bg);
#ifdef CLOUD_COUNTERS
	// The spike's extra counters in the high 16 bits (CloudSpike / render_check unpack them): light fetches | Delta-tau
	// limited steps, occupied steps | steps with extinction > 0, empty steps | structure-limited steps.
	cloud_count_light = min(cloud_count_light, 0xffffu) | (min(cloud_spike_count_tau_limit, 0xffffu) << 16);
	cloud_count_occupied = min(cloud_count_occupied, 0xffffu) | (min(cloud_spike_count_extinct, 0xffffu) << 16);
	cloud_count_empty = min(cloud_count_empty, 0xffffu) | (min(cloud_spike_count_structure, 0xffffu) << 16);
#endif
}
#endif
