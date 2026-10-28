// The cloud world volume's light (world-volume plan Task 6; spec C; CloudWorldVolume): GLSL 450 only, compiled after the
// march's files, cloud_volume.gdshaderinc (with CLOUD_VOLUME_LOOKUP) and cloud_volume_bake.gdshaderinc (set 0 is the
// march's: the snapshots' fields and columns, cloud_blend, the camera). A light texel holds x tau_sun (from the cell
// centre toward the Sun), y tau_below (vertical, down to the shell's bottom: the transfer table's b), z tau_above (up to
// the shell's top: the sky's source), w tau_moon - optical depths through the baked density rho (the cell-mean
// extinction, both snapshots blended by cloud_blend). Kernels (a light job runs COLUMN, then - after a barrier - LIGHT):
//   CLOUD_VOLUME_LIGHT_COLUMN y and z of a job box's cells, one invocation per column: the level's own column of cell
//                             centres (the trilinear field along the geodetic vertical - the grid's z - is linear between
//                             them: the trapezoid sums are its exact integral) over the level's window, beyond the
//                             window (levels 0-1) the coarser shown levels marched (steps below).
//   CLOUD_VOLUME_LIGHT        x and w per cell: from the cell centre toward the Sun / the Moon through the cell's level,
//                             then the coarser levels shown in its frame, then beyond the coarsest level the grid-mean
//                             column; Earth's shadow (the straight ray meets the ground) CLOUD_VOLUME_TAU_SHADOW; w only
//                             in jobs with the Moon on (else kept; CLOUD_VOLUME_TAU_SHADOW when written fresh).
//   CLOUD_VOLUME_PROVISIONAL  all four from the next coarser shown level's light (filtered at the cell centre), or for the
//                             coarsest level (or none shown) from the grid-mean column: tau_above the column above the
//                             centre, tau_below the column from the shell's bottom less it, tau_sun / tau_moon tau_above
//                             times the path to the shell's top over the height to the cloud top (Earth's shadow as
//                             above). Written directly.
//   CLOUD_VOLUME_LIGHT_SCAN   diagnostics (cloud_check.gd section 17): counts over a light texture's texels - any channel
//                             NaN, any channel infinite, tau_sun >= CLOUD_VOLUME_TAU_SHADOW.
// Write modes of COLUMN and LIGHT (per channel they compute): directly; rate-limited (exp(-tau) moves by at most
// CLOUD_VOLUME_LIGHT_STEP_T; a clamped cell counts into its texture's counter: the schedule then runs another pass of the
// level); blended (exp(-tau) mixed by the job's weight: the real light replacing provisional light over
// PROVISIONAL_FADE_S).
//
// Steps toward a light (Delta-tau limited on the max-extinction mip, spec C): from a clear sample the ray first jumps
// the level's free space (cloud_volume_free_m; the ray is clipped to the shell, where alone it is defined; a clear block
// without free space - cloud in its ring - is crossed in one step), else steps ds = Delta tau / the bound of the
// sample's block (and of level + 1's in the fade band) with Delta tau = max(CLOUD_LIGHT_DTAU_MIN,
// CLOUD_LIGHT_DTAU_RELATIVE x tau so far) (relative accuracy: deep in a cloud the depth grows geometrically), at most
// CLOUD_LIGHT_CELLS_PER_STEP cells along the ray. Each step is the trapezoid of rho at its ends. Out of samples, the
// rest of the path is the grid-mean column's (as beyond the coarsest level). Error bound: rho along the ray is the
// trilinear field - linear between the cells' centre planes, so the trapezoid errs only where a step crosses such a
// plane with a change of slope: by at most |the slope change| ds^2 / 8 <= (the density change from cell to cell) x the
// step / 8 per crossing (<= 2 crossings a step). A step whose ends differ by more than CLOUD_LIGHT_EDGE_RATIO of the
// larger (a cloud edge) is halved, up to CLOUD_LIGHT_EDGE_HALVINGS times. For the slab of cloud_check section 17 (rho
// constant inside, linear over one cell at each edge) only the steps over the top's two kinks err: <= 2 x Delta tau / 8
// each with Delta tau <= max(0.5, 0.05 tau), i.e. <= 0.125 absolute below tau 10 and <= 1.3 % of tau above it.

#if defined(CLOUD_VOLUME_LIGHT) || defined(CLOUD_VOLUME_PROVISIONAL) || defined(CLOUD_VOLUME_LIGHT_SCAN)
layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;
#endif
#ifdef CLOUD_VOLUME_LIGHT_COLUMN
layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;
#endif

// [E] Stored for "no light" (Earth's shadow, the Moon off): exp(-6e4) is 0, a half float holds it (max 65504) and a
// filter between it and a finite depth stays finite (an infinity times a zero filter weight would be NaN).
const float CLOUD_VOLUME_TAU_SHADOW = 6.0e4;
// CloudWorldVolumeSchedule.LIGHT_STEP_T (change together): a rate-limited refresh moves exp(-tau) by at most this.
const float CLOUD_VOLUME_LIGHT_STEP_T = 0.01;
// Write modes (CloudWorldVolumeLight.MODE_*; change together).
const int CLOUD_VOLUME_LIGHT_DIRECT = 0;
const int CLOUD_VOLUME_LIGHT_RATE = 1;
const int CLOUD_VOLUME_LIGHT_BLEND = 2;

#if defined(CLOUD_VOLUME_LIGHT) || defined(CLOUD_VOLUME_LIGHT_COLUMN) || defined(CLOUD_VOLUME_PROVISIONAL)
// [E] Samples per ray toward the Sun / Moon (spec C: 32-64) and per vertical ray beyond a level's window.
const int CLOUD_LIGHT_MAX_SAMPLES = 64;
const int CLOUD_LIGHT_VERTICAL_SAMPLES = 32;
// [E] Delta tau per step: at least this, and this share of the depth so far (see the header's bound).
const float CLOUD_LIGHT_DTAU_MIN = 0.5;
const float CLOUD_LIGHT_DTAU_RELATIVE = 0.05;
// [E] Edge refinement: ends differing by more than this share of the larger halve the step, at most this often.
const float CLOUD_LIGHT_EDGE_RATIO = 0.5;
const int CLOUD_LIGHT_EDGE_HALVINGS = 4;
// [E] The shortest step (m): below the finest cell's 25 m / 16 halvings the trapezoid's error is negligible.
const float CLOUD_LIGHT_MIN_STEP_M = 1.0;
// [E] The smallest cosine to the vertical (and across it) a step's cell / block extent is taken at.
const float CLOUD_LIGHT_MIN_COSINE = 0.01;
// [E] A step in cloud spans at most this many cells along the ray (the trilinear field's kinks: see the header).
const float CLOUD_LIGHT_CELLS_PER_STEP = 2.0;
// [E] The column term's smallest height below the cloud top (m): the mean extinction above a point is the column over
// that height.
const float CLOUD_LIGHT_MIN_COLUMN_M = 10.0;
// The shell's top (CloudWorldVolumeLayout.SHELL_TOP_M).
const float CLOUD_VOLUME_SHELL_TOP_M = 20500.0;

// One job's box (CloudWorldVolume._light_job_bytes): the batch's jobs in one buffer, the dispatch's by push constant
// (one uniform set per written texture, not per job: a batch at a check's budget has thousands of jobs).
struct CloudVolumeLightJob {
	ivec4 first;  // xyz: the box's first cell (global cells), w: the level
	ivec4 size;   // xyz: cells, w: the texture index (the level, or CLOUD_VOLUME_SPARE)
	ivec4 info;   // x: frame index, y: write mode, z: 1 = tau_moon too, w: provisional from the column (1) or level + 1
	vec4 sun;     // xyz: toward the Sun (Earth-local render axes), w: the blend weight (mode BLEND)
	vec4 moon;    // xyz: toward the Moon
	ivec4 slot;   // x: the job's clamp counter
};
layout(set = 1, binding = 1, std430) readonly buffer CloudVolumeLightJobs {
	CloudVolumeLightJob light_jobs[];
};
layout(push_constant, std430) uniform CloudVolumeLightPush {
	ivec4 job;  // x: the dispatch's job
} light_push;
#define light_job light_jobs[light_push.job.x]
layout(rgba16f, set = 1, binding = 2) uniform image3D volume_light;

// True when level j is shown in frame f (the lookup's READY flag: ready, nested in its shown coarser level).
bool cloud_light_shown(int j, int f) {
	int flags = cloud_volume.levels[j].w;
	return (flags & (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)) == (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)
			&& (flags >> CLOUD_VOLUME_FRAME_SHIFT) == f;
}

// The shell part [s_in, s_out] of a ray from a point at height h (m; sphere radius r) with cosine mu to the vertical
// (false: none) and whether it meets the ground first (Earth's shadow). The sphere is the grid's Gaussian one (the
// ellipsoid's radius of curvature differs by < 0.7 %: a tenth of a degree at the terminator).
bool cloud_light_shell(float h, float r, float mu, out float s_in, out float s_out, out bool shadow) {
	float rp = r + h;
	float top = CLOUD_VOLUME_SHELL_TOP_M;
	float lift = (top - h) * (2.0 * r + top + h);  // r_top^2 - rp^2, without the difference of two radii
	shadow = mu < 0.0 && (h <= 0.0 || rp * rp * mu * mu > h * (2.0 * r + h));
	float disc = rp * rp * mu * mu + lift;
	s_in = 0.0;
	s_out = 0.0;
	if (disc < 0.0) {
		return false;
	}
	float root = sqrt(disc);
	if (h <= top) {
		s_out = mu > 0.0 ? lift / (root + rp * mu) : root - rp * mu;
		return s_out > 0.0;
	}
	if (mu >= 0.0) {
		return false;  // above the shell, moving away
	}
	s_in = -lift / (root - rp * mu);
	s_out = root - rp * mu;
	return true;
}

// The cell's centre (grid point of frame f, Earth-local offset from the camera, geodetic up) - as the bake.
vec3 cloud_light_centre(int k, ivec3 c, int f, out vec3 offset, out vec3 up) {
	CloudVolumeFrame fr = cloud_volume.frames[f];
	vec3 centre_grid = vec3((float(c.x) + 0.5) * CLOUD_VOLUME_CELL_H_M[k], (float(c.y) + 0.5) * CLOUD_VOLUME_CELL_H_M[k],
			CLOUD_VOLUME_SHELL_BOTTOM_M + (float(c.z) + 0.5) * CLOUD_VOLUME_CELL_V_M[k]);
	vec3 dg = (centre_grid - fr.grid_hi.xyz) - fr.grid_lo.xyz;
	vec3 normal;
	offset = cloud_volume_offset_of(f, dg, normal);
	up = normalize(normal);
	return centre_grid;
}
#endif

#if defined(CLOUD_VOLUME_LIGHT) || defined(CLOUD_VOLUME_LIGHT_COLUMN)
layout(set = 1, binding = 3, std430) buffer CloudVolumeLightCounts {
	uint light_clamped[];  // per job of the batch (slot.x): cells a rate-limited write clamped
};

// The finest level the light reads at grid point g (frame f): the texture own_index (level k0) where its window holds
// g with a weight > 0 (a job's level is filled: the schedule lights only ready levels), else the coarser levels shown
// in frame f. Returns the texture index (-1: none - beyond the coarsest level), its level and weight.
int cloud_light_level(vec3 g, int own_index, int k0, int f, out int k, out float fade) {
	float w = cloud_volume_fade(own_index, k0, g);
	if (w > 0.0) {
		k = k0;
		fade = w;
		return own_index;
	}
	for (int j = k0 + 1; j < CLOUD_VOLUME_LEVELS; j++) {
		if (!cloud_light_shown(j, f)) {
			continue;
		}
		w = cloud_volume_fade(j, j, g);
		if (w > 0.0) {
			k = j;
			fade = w;
			return j;
		}
	}
	k = -1;
	fade = 0.0;
	return -1;
}

// rho [1/m] at g of texture `index` (level k, weight fade): faded toward level k + 1 where it is shown in frame f (as
// cloud_volume_density), both snapshots blended.
float cloud_light_rho(int index, int k, float fade, vec3 g, int f) {
	vec4 v = cloud_volume_fetch_density(index, k, g);
	if (fade < 1.0 && k + 1 < CLOUD_VOLUME_LEVELS && cloud_light_shown(k + 1, f)) {
		v = mix(cloud_volume_fetch_density(k + 1, k + 1, g), v, fade);
	}
	return mix(v.x, v.z, cloud_blend);
}

// The step's extinction bound [1/m] at g: the max mip of g's block of texture `index` (level k), in the fade band also
// level k + 1's block at g (the band blends it in).
float cloud_light_bound(int index, int k, float fade, vec3 g, int f) {
	ivec3 block = ivec3(floor(cloud_volume_cells(k, g) / float(CLOUD_VOLUME_MIP_DIV)));
	if (k >= CLOUD_VOLUME_WINDOWED_LEVELS) {
		block.z = clamp(block.z, 0, CLOUD_VOLUME_MIP_SIZE.z - 1);
	} else if ((cloud_volume.levels[index].w & CLOUD_VOLUME_CLAMPED_BOTTOM) != 0) {
		block.z = max(block.z, 0);
	}
	float bound = cloud_volume_fetch_max(index, cloud_volume_wrap(block, CLOUD_VOLUME_MIP_SIZE));
	if (fade < 1.0 && k + 1 < CLOUD_VOLUME_LEVELS && cloud_light_shown(k + 1, f)) {
		bound = max(bound, cloud_volume_fetch_max(k + 1, cloud_volume_wrap(cloud_volume_block(k + 1, g),
				CLOUD_VOLUME_MIP_SIZE)));
	}
	return bound;
}

// The grid point of the ray's point s (m) of frame f: vertical rays move along the grid's z (the geodetic normal keeps
// the grid's x, y), others through the camera-relative transform of the straight line.
vec3 cloud_light_point(bool vertical, vec3 g0, vec3 p0, vec3 dir, float s, int f) {
	return vertical ? g0 + vec3(0.0, 0.0, s * dir.z) : cloud_volume_grid_in(f, p0 + dir * s);
}

// The optical depth over the ray's part [s0, s1] inside the volume (header: the steps). vertical: dir.z = +-1 in grid
// space from g0; else dir the Earth-local direction from the offset p0 (grid point g0 at s = 0). exit_s: where the ray
// left the coarsest level (s1 if it did not).
float cloud_light_depth(bool vertical, vec3 g0, vec3 p0, vec3 dir, float s0, float s1, float mu, int max_samples,
		int own_index, int k0, int f, out float exit_s) {
	float tau = 0.0;
	float s = s0;
	exit_s = s1;
	vec3 g = s0 > 0.0 ? cloud_light_point(vertical, g0, p0, dir, s0, f) : g0;
	int k;
	float fade;
	int index = cloud_light_level(g, own_index, k0, f, k, fade);
	if (index < 0) {
		exit_s = s;
		return 0.0;
	}
	float rho_a = cloud_light_rho(index, k, fade, g, f);
	float across = sqrt(max(1.0 - mu * mu, 0.0));
	for (int i = 0; i < max_samples && s < s1; i++) {
		// a clear sample of a level (not a re-frame's spare): its free space first
		float free_m = (index == k && rho_a <= 0.0) ? cloud_volume_free_m(g, k) : 0.0;
		float ds;
		if (free_m > 0.0) {
			ds = max(free_m, CLOUD_LIGHT_MIN_STEP_M);
		} else {
			float cell_m = min(CLOUD_VOLUME_CELL_H_M[k] / max(across, CLOUD_LIGHT_MIN_COSINE),
					CLOUD_VOLUME_CELL_V_M[k] / max(abs(mu), CLOUD_LIGHT_MIN_COSINE));  // one cell along the ray
			float bound = cloud_light_bound(index, k, fade, g, f);
			float dtau = max(CLOUD_LIGHT_DTAU_MIN, CLOUD_LIGHT_DTAU_RELATIVE * tau);
			ds = bound > 0.0 ? max(min(dtau / bound, CLOUD_LIGHT_CELLS_PER_STEP * cell_m), CLOUD_LIGHT_MIN_STEP_M)
					: float(CLOUD_VOLUME_MIP_DIV) * cell_m;  // a clear block (no free space: cloud in its ring)
		}
		ds = min(ds, s1 - s);
		// the step's far end (halved across an edge)
		vec3 g_b;
		float rho_b = 0.0;
		int index_b;
		int k_b;
		float fade_b;
		for (int h = 0; h <= CLOUD_LIGHT_EDGE_HALVINGS; h++) {
			g_b = cloud_light_point(vertical, g0, p0, dir, s + ds, f);
			index_b = cloud_light_level(g_b, own_index, k0, f, k_b, fade_b);
			rho_b = index_b >= 0 ? cloud_light_rho(index_b, k_b, fade_b, g_b, f) : rho_a;
			if (free_m > 0.0 || h == CLOUD_LIGHT_EDGE_HALVINGS || ds <= CLOUD_LIGHT_MIN_STEP_M
					|| abs(rho_b - rho_a) <= CLOUD_LIGHT_EDGE_RATIO * max(rho_a, rho_b)) {
				break;
			}
			ds *= 0.5;
		}
		// a free jump adds nothing (no density inside the free ball; rho_b at its edge starts the next step)
		tau += free_m > 0.0 ? 0.0 : 0.5 * (rho_a + rho_b) * ds;
		s += ds;
		if (index_b < 0) {
			exit_s = s;  // beyond the coarsest level: the caller's column term
			return tau;
		}
		g = g_b;
		rho_a = rho_b;
		index = index_b;
		k = k_b;
		fade = fade_b;
	}
	if (s < s1) {
		exit_s = s;  // out of samples: the rest by the grid-mean column (the caller's)
	}
	return tau;
}

float cloud_light_transmittance(float tau) {
	return tau >= CLOUD_VOLUME_TAU_SHADOW ? 0.0 : exp(-tau);
}

float cloud_light_depth_of(float t) {
	return t > 0.0 ? min(-log(t), CLOUD_VOLUME_TAU_SHADOW) : CLOUD_VOLUME_TAU_SHADOW;
}

// The texel written over `old` by the job's mode for the channels in `which` (the others kept).
vec4 cloud_light_written(vec4 old, vec4 fresh, bvec4 which, int mode, float weight, out bool clamped) {
	vec4 written = old;
	clamped = false;
	for (int i = 0; i < 4; i++) {
		if (!which[i]) {
			continue;
		}
		if (mode == CLOUD_VOLUME_LIGHT_DIRECT) {
			written[i] = fresh[i];
			continue;
		}
		float t_old = cloud_light_transmittance(old[i]);
		float t_new = cloud_light_transmittance(fresh[i]);
		if (mode == CLOUD_VOLUME_LIGHT_BLEND) {
			written[i] = cloud_light_depth_of(mix(t_old, t_new, clamp(weight, 0.0, 1.0)));
		} else if (abs(t_new - t_old) > CLOUD_VOLUME_LIGHT_STEP_T) {
			written[i] = cloud_light_depth_of(t_old + sign(t_new - t_old) * CLOUD_VOLUME_LIGHT_STEP_T);
			clamped = true;
		} else {
			written[i] = fresh[i];
		}
	}
	return written;
}
#endif

#ifdef CLOUD_VOLUME_LIGHT
// The grid-mean column's depth along the rest of a ray from s_x to the shell's top (beyond the coarsest level): the mean
// extinction above the ray's lowest point there (cloud_column_mean_depth over the height to the cloud top) times the
// path. p_x the Earth-local offset at s_x, h_x its height, mu_x its cosine to the vertical, r the sphere's radius.
float cloud_light_column_rest(vec3 p_x, vec3 dir, float h_x, float mu_x, float r, float rest_m) {
	if (rest_m <= 0.0) {
		return 0.0;
	}
	float s_low = 0.0;
	float h_low = h_x;
	if (mu_x < 0.0) {  // descending: the perigee
		float rx = r + h_x;
		s_low = min(-rx * mu_x, rest_m);
		h_low = sqrt(rx * rx * (1.0 - mu_x * mu_x) + (s_low + rx * mu_x) * (s_low + rx * mu_x)) - r;
	}
	vec2 ll = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + p_x + dir * s_low);
	float depth = cloud_column_mean_depth(ll, h_low).x;
	return depth * rest_m / max(CLOUD_TOP_M - h_low, CLOUD_LIGHT_MIN_COLUMN_M);
}

// tau toward a light (unit, Earth-local) from the cell centre: Earth's shadow, the volume, the column beyond it.
float cloud_light_toward(vec3 light, vec3 g0, vec3 p0, vec3 up, int own_index, int k0, int f) {
	float r = cloud_volume.frames[f].grid_hi.w;
	float mu = dot(up, light);
	float s_in;
	float s_out;
	bool shadow;
	bool inside = cloud_light_shell(g0.z, r, mu, s_in, s_out, shadow);
	if (shadow) {
		return CLOUD_VOLUME_TAU_SHADOW;
	}
	if (!inside) {
		return 0.0;
	}
	float exit_s;
	float tau = cloud_light_depth(false, g0, p0, light, s_in, s_out, mu, CLOUD_LIGHT_MAX_SAMPLES, own_index, k0, f,
			exit_s);
	if (exit_s < s_out) {
		vec3 p_x = p0 + light * exit_s;
		vec3 g_x = cloud_volume_grid_in(f, p_x);
		float mu_x = ((r + g0.z) * mu + exit_s) / (r + g_x.z);
		tau += cloud_light_column_rest(p_x, light, g_x.z, mu_x, r, s_out - exit_s);
	}
	return min(tau, CLOUD_VOLUME_TAU_SHADOW);
}

void main() {
	ivec3 gi = ivec3(gl_GlobalInvocationID);
	if (any(greaterThanEqual(gi, light_job.size.xyz))) {
		return;
	}
	int k = light_job.first.w;
	int index = light_job.size.w;
	int f = light_job.info.x;
	int mode = light_job.info.y;
	bool moon = light_job.info.z != 0;
	ivec3 c = light_job.first.xyz + gi;
	vec3 p0;
	vec3 up;
	vec3 g0 = cloud_light_centre(k, c, f, p0, up);
	ivec3 texel = cloud_volume_wrap(c, CLOUD_VOLUME_SIZE);
	vec4 old = imageLoad(volume_light, texel);  // y, z: the column pass's (a barrier before)
	vec4 fresh = old;
	fresh.x = cloud_light_toward(normalize(light_job.sun.xyz), g0, p0, up, index, k, f);
	fresh.w = moon ? cloud_light_toward(normalize(light_job.moon.xyz), g0, p0, up, index, k, f)
			: CLOUD_VOLUME_TAU_SHADOW;
	bool clamped;
	vec4 written = cloud_light_written(old, fresh, bvec4(true, false, false, moon || mode == CLOUD_VOLUME_LIGHT_DIRECT),
			mode, light_job.sun.w, clamped);
	if (clamped) {
		atomicAdd(light_clamped[light_job.slot.x], 1u);
	}
	imageStore(volume_light, texel, written);
}
#endif

#ifdef CLOUD_VOLUME_LIGHT_COLUMN
// rho [1/m] of the cell c (global) of texture `index`: its texel, both snapshots blended.
float cloud_light_cell_rho(int index, ivec3 c) {
	ivec3 t = cloud_volume_wrap(c, CLOUD_VOLUME_SIZE);
#define CLOUD_VOLUME_SAMPLE(tex) texelFetch(tex, t, 0) / CLOUD_VOLUME_PER_KM
	vec4 v;
	switch (index) {
		case 0: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex0); break;
		case 1: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex1); break;
		case 2: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex2); break;
		case 3: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex3); break;
		case 4: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex4); break;
		case 5: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex5); break;
		case 6: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex6); break;
		case 7: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex7); break;
		default: v = CLOUD_VOLUME_SAMPLE(cloud_volume_density_tex8); break;
	}
#undef CLOUD_VOLUME_SAMPLE
	return mix(v.x, v.z, cloud_blend);
}

void main() {
	ivec2 gi = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(gi, light_job.size.xy))) {
		return;
	}
	int k = light_job.first.w;
	int index = light_job.size.w;
	int f = light_job.info.x;
	int mode = light_job.info.y;
	ivec2 xy = light_job.first.xy + gi;
	float v = CLOUD_VOLUME_CELL_V_M[k];
	// The window's cells of the column (levels 2-7: the whole shell), the column's grid point.
	int z0 = cloud_volume.levels[index].z;
	int z1 = z0 + int(cloud_volume_window(k).z);
	vec2 xy_m = (vec2(xy) + 0.5) * CLOUD_VOLUME_CELL_H_M[k];
	float h_top = CLOUD_VOLUME_SHELL_BOTTOM_M + (float(z1) - 0.5) * v;
	float h_bottom = CLOUD_VOLUME_SHELL_BOTTOM_M + (float(z0) + 0.5) * v;
	float rho_top = cloud_light_cell_rho(index, ivec3(xy, z1 - 1));
	float rho_bottom = cloud_light_cell_rho(index, ivec3(xy, z0));
	// Beyond the outer centres: up to the shell's top / down to its bottom - the coarser levels where the window ends
	// inside the shell (levels 0-1), else the outer cell's value held over the last half cell (the reads clamp there).
	float above_out = rho_top * max(CLOUD_VOLUME_SHELL_TOP_M - h_top, 0.0);
	float below_out = rho_bottom * max(h_bottom - CLOUD_VOLUME_SHELL_BOTTOM_M, 0.0);
	bool up_shown = k + 1 < CLOUD_VOLUME_LEVELS && cloud_light_shown(k + 1, f);
	float exit_s;
	if (up_shown && h_top + 0.5 * v < CLOUD_VOLUME_SHELL_TOP_M) {
		above_out = cloud_light_depth(true, vec3(xy_m, h_top), vec3(0.0), vec3(0.0, 0.0, 1.0), 0.0,
				CLOUD_VOLUME_SHELL_TOP_M - h_top, 1.0, CLOUD_LIGHT_VERTICAL_SAMPLES, k + 1, k + 1, f, exit_s);
	}
	if (up_shown && (cloud_volume.levels[index].w & CLOUD_VOLUME_CLAMPED_BOTTOM) == 0
			&& h_bottom - 0.5 * v > CLOUD_VOLUME_SHELL_BOTTOM_M) {
		below_out = cloud_light_depth(true, vec3(xy_m, h_bottom), vec3(0.0), vec3(0.0, 0.0, -1.0), 0.0,
				h_bottom - CLOUD_VOLUME_SHELL_BOTTOM_M, -1.0, CLOUD_LIGHT_VERTICAL_SAMPLES, k + 1, k + 1, f, exit_s);
	}
	// The column's trapezoid sum over the window, then the box's cells (outside the window: the nearest window cell's).
	float total = 0.0;
	float rho_a = rho_bottom;
	for (int z = z0 + 1; z < z1; z++) {
		float rho_b = cloud_light_cell_rho(index, ivec3(xy, z));
		total += 0.5 * (rho_a + rho_b) * v;
		rho_a = rho_b;
	}
	int box0 = light_job.first.z;
	int box1 = box0 + light_job.size.z;
	float below = 0.0;
	rho_a = rho_bottom;
	int z = min(box0, z0);
	bool any_clamped = false;
	for (; z < box1; z++) {
		int zc = clamp(z, z0, z1 - 1);
		if (z > z0 && z < z1) {
			float rho_b = cloud_light_cell_rho(index, ivec3(xy, z));
			below += 0.5 * (rho_a + rho_b) * v;
			rho_a = rho_b;
		}
		if (z < box0) {
			continue;
		}
		ivec3 texel = cloud_volume_wrap(ivec3(xy, z), CLOUD_VOLUME_SIZE);
		vec4 old = imageLoad(volume_light, texel);
		vec4 fresh = vec4(old.x, min(below_out + below, CLOUD_VOLUME_TAU_SHADOW),
				min(above_out + (total - below), CLOUD_VOLUME_TAU_SHADOW), old.w);
		bool clamped;
		imageStore(volume_light, texel, cloud_light_written(old, fresh, bvec4(false, true, true, false), mode,
				light_job.sun.w, clamped));
		any_clamped = any_clamped || clamped;
	}
	if (any_clamped) {
		atomicAdd(light_clamped[light_job.slot.x], 1u);
	}
}
#endif

#ifdef CLOUD_VOLUME_PROVISIONAL
// The grid-mean column's tau toward a light from the centre (height h): Earth's shadow, else tau_above times the path
// to the shell's top over the height to the cloud top.
float cloud_provisional_toward(vec3 light, vec3 up, float h, float r, float tau_above) {
	float mu = dot(up, light);
	float s_in;
	float s_out;
	bool shadow;
	bool inside = cloud_light_shell(h, r, mu, s_in, s_out, shadow);
	if (shadow) {
		return CLOUD_VOLUME_TAU_SHADOW;
	}
	if (!inside) {
		return 0.0;
	}
	return min(tau_above * (s_out - s_in) / max(CLOUD_TOP_M - h, CLOUD_LIGHT_MIN_COLUMN_M), CLOUD_VOLUME_TAU_SHADOW);
}

void main() {
	ivec3 gi = ivec3(gl_GlobalInvocationID);
	if (any(greaterThanEqual(gi, light_job.size.xyz))) {
		return;
	}
	int k = light_job.first.w;
	int f = light_job.info.x;
	bool moon = light_job.info.z != 0;
	ivec3 c = light_job.first.xyz + gi;
	vec3 p0;
	vec3 up;
	vec3 g0 = cloud_light_centre(k, c, f, p0, up);
	vec4 light;
	int up_index = -1;  // the finest coarser level shown in frame f whose window holds the centre
	for (int j = k + 1; j < CLOUD_VOLUME_LEVELS && light_job.info.w == 0 && up_index < 0; j++) {
		if (cloud_light_shown(j, f) && cloud_volume_fade(j, j, g0) > 0.0) {
			up_index = j;
		}
	}
	if (up_index >= 0) {
		light = cloud_volume_fetch_light(up_index, up_index, g0);
		if (!moon) {
			light.w = CLOUD_VOLUME_TAU_SHADOW;
		}
	} else {
		float r = cloud_volume.frames[f].grid_hi.w;
		vec2 ll = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + p0);
		float above = cloud_column_mean_depth(ll, g0.z).x;
		float total = cloud_column_mean_depth(ll, CLOUD_VOLUME_SHELL_BOTTOM_M).x;
		light.x = cloud_provisional_toward(normalize(light_job.sun.xyz), up, g0.z, r, above);
		light.y = max(total - above, 0.0);
		light.z = above;
		light.w = moon ? cloud_provisional_toward(normalize(light_job.moon.xyz), up, g0.z, r, above)
				: CLOUD_VOLUME_TAU_SHADOW;
	}
	imageStore(volume_light, cloud_volume_wrap(c, CLOUD_VOLUME_SIZE), light);
}
#endif

#ifdef CLOUD_VOLUME_LIGHT_SCAN
layout(set = 1, binding = 1) uniform sampler3D scan_light;  // nearest
layout(set = 1, binding = 2, std430) buffer CloudVolumeLightScan {
	uint scan_counts[4];  // NaN texels, infinite texels, tau_sun >= CLOUD_VOLUME_TAU_SHADOW, texels
};

void main() {
	ivec3 t = ivec3(gl_GlobalInvocationID);
	if (any(greaterThanEqual(t, CLOUD_VOLUME_SIZE))) {
		return;
	}
	vec4 v = texelFetch(scan_light, t, 0);
	if (any(isnan(v))) {
		atomicAdd(scan_counts[0], 1u);
	}
	if (any(isinf(v))) {
		atomicAdd(scan_counts[1], 1u);
	}
	if (v.x >= CLOUD_VOLUME_TAU_SHADOW) {
		atomicAdd(scan_counts[2], 1u);
	}
	atomicAdd(scan_counts[3], 1u);
}
#endif
