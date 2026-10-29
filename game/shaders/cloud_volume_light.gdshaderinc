// The cloud world volume's light (world-volume plan Task 6; spec C; CloudWorldVolume): GLSL 450 only, compiled after the
// march's files, cloud_volume.gdshaderinc (with CLOUD_VOLUME_LOOKUP) and cloud_volume_bake.gdshaderinc (set 0 is the
// march's: the snapshots' fields and columns, cloud_blend, the camera). A light texel holds x tau_sun (from the cell
// centre toward the Sun), y tau_below (vertical, down to the shell's bottom: the transfer table's b), z tau_above (up to
// the shell's top: the sky's source), w tau_moon - optical depths through the baked density rho (the cell-mean
// extinction, both snapshots blended by cloud_blend). Kernels (CloudWorldVolume runs a level's COLUMN jobs, then its
// LIGHT jobs, each level in its own timed lists; neither reads any level's light - only baked density):
//   CLOUD_VOLUME_LIGHT_COLUMN  y and z of a job box's cells, one workgroup per column: the trapezoid sums of the level's
//                              own column of cell centres over its window (the trilinear field along the geodetic
//                              vertical - the grid's z - is linear between them: the sums are its exact integral); beyond
//                              the window (levels 0-1) the coarser shown levels' density marched along the vertical, one
//                              midpoint sample per invocation (else the grid-mean column).
//   CLOUD_VOLUME_LIGHT         x and w per cell: the distance-stepped march (below); Earth's shadow (the straight ray
//                              meets the ground) CLOUD_VOLUME_TAU_SHADOW; w only in jobs with the Moon on (else kept;
//                              CLOUD_VOLUME_TAU_SHADOW when written fresh).
//   CLOUD_VOLUME_PROVISIONAL   all four from the next coarser shown level's light (filtered at the cell centre), or for the
//                              coarsest level (or none shown) from the grid-mean column: tau_above the column above the
//                              centre, tau_below the column from the shell's bottom less it, tau_sun / tau_moon tau_above
//                              times the path below the cloud top over the height to it (Earth's shadow as above).
//                              Written directly.
//   CLOUD_VOLUME_LIGHT_SCAN    diagnostics (cloud_check.gd section 17): counts over a light texture's texels - any channel
//                              NaN, any channel infinite, tau_sun >= CLOUD_VOLUME_TAU_SHADOW.
//   CLOUD_VOLUME_LIGHT_REFERENCE diagnostics (section 17): the converged tau toward a direction from given cells (fine
//                              midpoint steps through the finest level at each point, as the march's density) next to
//                              the stored light texel.
// Write modes of COLUMN and LIGHT (per channel they compute): directly; rate-limited (exp(-tau) moves by at most
// CLOUD_VOLUME_LIGHT_STEP_T; a clamped cell counts into its job's counter: the schedule then serves the box again);
// blended (exp(-tau) mixed by the job's weight: the real light replacing provisional light over PROVISIONAL_FADE_S).
// A rate-limited channel whose old or new value is "no light" (Earth's shadow, the Moon off: CLOUD_VOLUME_TAU_SHADOW) is
// written directly: a cell leaves or enters Earth's shadow only with its Sun ray grazing the ground, where the air's
// transmittance toward the Sun (the march's factor) is ~0 - the true terminator, no front; the Moon's channel is read by
// the march only with the level's Moon flag on, which turns on with the jobs that write it (Task 7 gates and ramps it).
//
// The distance-stepped march toward a light (constant cost per cell; Task 6 review round 1, the controller's ruling -
// a clipmap cone march: the resolution falls with the distance along the ray): from the cell centre p of level k the ray
// is marched through BAKED DENSITY in segments - CLOUD_LIGHT_LEVEL_CELLS cells of level k along the ray (one cell along
// the ray: the shorter of its horizontal and vertical crossing), then as many cells of each next coarser level (the step
// grows with the level), each with CLOUD_LIGHT_SAMPLES_PER_CELL midpoint samples per cell - until the ray leaves the
// field: the shell's top or, lower, the field's cloud top (environment::cloud_render_top_m of both snapshots, plus the
// trilinear support of the coarsest level's cells above it: no density there, so the exit only shortens the march).
// A sample of segment j reads the finest level >= j whose window holds it (level k itself in segment k; a coarser one
// shown, in either frame), faded toward the next coarser level as the march's density. Beyond level 7's segment, and
// from a sample no window holds (a ray leaving level 7's window), the grid-mean column takes the rest of the path. No
// level's light is read: no interpolation of other rays' depth across cloud gaps, no dependence on the order the
// levels are lit in. Each segment's grid path is the quadratic through its
// exact grid points at its ends and middle (the curvature is second order: the cubic rest is < 15 m over level 7's
// longest segment, < 0.1 cell): no geodetic transform per sample. Error: each sample errs by the midpoint rule on the
// trilinear field (where its segment crosses a centre plane with a change of slope: <= the density change between
// neighbouring cells x ds / 4); segment j carries level j's resolution - the trade of the spec's "step into coarser
// levels", measured in cloud_check.gd section 18 against the converged integral (exp(-tau) p99 <= 0.02 at levels >= 1
// on the broken cf 0.4 field; level 0's residual is distant cloud-edge grazing at 50 m vs 100+ m resolution).

// One dispatch per level for LIGHT and LIGHT_COLUMN (Task 6 review round 1: a dispatch per job serialised the jobs -
// they write one image - each at the latency of its longest ray): LIGHT a workgroup per 4^3 block of the level's jobs
// (neighbouring rays share the texture cache), LIGHT_COLUMN a workgroup per column (CLOUD_VOLUME_CELLS_V).
#if defined(CLOUD_VOLUME_LIGHT) || defined(CLOUD_VOLUME_PROVISIONAL) || defined(CLOUD_VOLUME_LIGHT_SCAN)
layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;
#endif
#ifdef CLOUD_VOLUME_LIGHT_COLUMN
layout(local_size_x = 128, local_size_y = 1, local_size_z = 1) in;
#endif
#ifdef CLOUD_VOLUME_LIGHT_REFERENCE
layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;
#endif

// [E] Stored for "no light" (Earth's shadow, the Moon off): exp(-6e4) is 0, a half float holds it (max 65504) and a
// filter between it and a finite depth stays finite (an infinity times a zero filter weight would be NaN).
const float CLOUD_VOLUME_TAU_SHADOW = 6.0e4;
// CloudWorldVolumeSchedule.LIGHT_STEP_T (change together): a rate-limited refresh moves exp(-tau) by at most this.
const float CLOUD_VOLUME_LIGHT_STEP_T = 0.01;
// The half float holding tau keeps 11 significant bits: stored, tau is off by at most 2^-10 of it (truncated; 2^-11
// rounded), exp(-tau) by T tau 2^-10 (<= 3.6e-4 at tau 1) - a rate-limited step is shortened by that much.
const float CLOUD_VOLUME_HALF_EPSILON = 9.765625e-4;
// Write modes (CloudWorldVolumeLight.MODE_*; change together).
const int CLOUD_VOLUME_LIGHT_DIRECT = 0;
const int CLOUD_VOLUME_LIGHT_RATE = 1;
const int CLOUD_VOLUME_LIGHT_BLEND = 2;

#if defined(CLOUD_VOLUME_LIGHT) || defined(CLOUD_VOLUME_LIGHT_COLUMN) || defined(CLOUD_VOLUME_PROVISIONAL) \
		|| defined(CLOUD_VOLUME_LIGHT_REFERENCE)
// [E] Cells of each level a light ray is marched through before the next coarser level takes over (Task 6 review round
// 1: on the broken cf 0.4 field 16 gives exp(-tau) p99 0.016 / 0.008 at levels 2 / 5, 8 gives 0.22 / 0.31; round 1c:
// at a 15 deg Sun 16 left level 2 at p99 0.18 - its rays read 400 m cells for the cumulus 1.35-2 km up - 32 gives
// 0.006).
const int CLOUD_LIGHT_LEVEL_CELLS = 32;
// Midpoint samples per cell along the ray.
const int CLOUD_LIGHT_SAMPLES_PER_CELL = 2;
// The trilinear field reaches up to 1.5 cells above the highest cell holding cloud (its centre, the next centre): the
// march ends this many of the coarsest level's vertical cells above the field's cloud top.
const float CLOUD_LIGHT_TOP_SUPPORT_CELLS = 1.5;
// [E] The smallest cosine to the vertical (and across it) a cell's extent along the ray is taken at.
const float CLOUD_LIGHT_MIN_COSINE = 0.01;
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
	vec4 moon;    // xyz: toward the Moon, w: the field's cloud top (m; environment::cloud_render_top_m of both snapshots)
	ivec4 slot;   // x: its counters (CloudVolumeLightCounts); y, z: its first column, 4^3 block in its level's dispatch
};
#ifndef CLOUD_VOLUME_LIGHT_REFERENCE
layout(set = 1, binding = 1, std430) readonly buffer CloudVolumeLightJobs {
	CloudVolumeLightJob light_jobs[];
};
layout(push_constant, std430) uniform CloudVolumeLightPush {
	ivec4 job;  // x: the dispatch's (first) job; LIGHT / LIGHT_COLUMN: y its jobs, z their columns, w their 4^3 blocks
} light_push;
int light_job_index;  // the invocation's job (set first in main)
#define light_job light_jobs[light_job_index]
layout(rgba16f, set = 1, binding = 2) uniform image3D volume_light;
#endif

#if defined(CLOUD_VOLUME_LIGHT) || defined(CLOUD_VOLUME_LIGHT_COLUMN)
// The workgroup's job (light_job_index) and its unit inside the job in a level's dispatch: jobs light_push.job.x ..
// + y - 1, each holding its first unit in slot[which] (ascending; which 1: columns, 2: 4^3 blocks) of `count` units in
// all; -1 past the last unit (the whole workgroup).
int cloud_light_dispatch_unit(int which, int count) {
	int g = int(gl_WorkGroupID.x + gl_WorkGroupID.y * gl_NumWorkGroups.x);
	light_job_index = light_push.job.x;
	if (g >= count) {
		return -1;
	}
	int lo = light_push.job.x;
	int hi = lo + light_push.job.y - 1;
	while (lo < hi) {
		int mid = (lo + hi + 1) / 2;
		if (light_jobs[mid].slot[which] <= g) {
			lo = mid;
		} else {
			hi = mid - 1;
		}
	}
	light_job_index = lo;
	return g - light_jobs[lo].slot[which];
}
#endif

// True when level j is shown in frame f (the lookup's READY flag: ready, nested in its shown coarser level).
bool cloud_light_shown(int j, int f) {
	int flags = cloud_volume.levels[j].w;
	return (flags & (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)) == (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)
			&& (flags >> CLOUD_VOLUME_FRAME_SHIFT) == f;
}

// The part [s_in, s_out] of a ray from a point at height h (m; sphere radius r) with cosine mu to the vertical below the
// height `top` (false: none) and whether it meets the ground first (Earth's shadow). The sphere is the grid's Gaussian
// one (the ellipsoid's radius of curvature differs by < 0.7 %: a tenth of a degree at the terminator).
bool cloud_light_shell(float h, float r, float mu, float top, out float s_in, out float s_out, out bool shadow) {
	float rp = r + h;
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
		return false;  // above the top, moving away
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

// One cell of level k along a ray with cosine mu to the vertical (m): the shorter of its horizontal and vertical
// crossing.
float cloud_light_cell_along(int k, float mu) {
	float across = sqrt(max(1.0 - mu * mu, 0.0));
	return min(CLOUD_VOLUME_CELL_H_M[k] / max(across, CLOUD_LIGHT_MIN_COSINE),
			CLOUD_VOLUME_CELL_V_M[k] / max(abs(mu), CLOUD_LIGHT_MIN_COSINE));
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

// rho [1/m] at a point of a light ray from the finest level >= j whose window holds it: level k itself (texture
// own_index, frame f), else a coarser level shown in either frame (a re-frame in progress: read at its own grid point of
// p). g: the point's grid point in frame f, p: its Earth-local offset from the camera. found: false when none holds it.
float cloud_light_rho_at(vec3 g, vec3 p, int j, int k, int own_index, int f, out bool found) {
	for (int l = j; l < CLOUD_VOLUME_LEVELS; l++) {
		int index = l;
		int fl = f;
		if (l == k) {
			index = own_index;
		} else {
			int flags = cloud_volume.levels[l].w;
			if ((flags & (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)) != (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)) {
				continue;
			}
			fl = flags >> CLOUD_VOLUME_FRAME_SHIFT;
		}
		vec3 gl = fl == f ? g : cloud_volume_grid_in(fl, p);
		float w = cloud_volume_fade(index, l, gl);
		if (w > 0.0) {
			found = true;
			return cloud_light_rho(index, l, w, gl, fl);
		}
	}
	found = false;
	return 0.0;
}

// The height (m) above which the baked field holds no density: the field's cloud top plus the coarsest level's
// trilinear support, at most the shell's top.
float cloud_light_field_top(float cloud_top_m) {
	return min(cloud_top_m + CLOUD_LIGHT_TOP_SUPPORT_CELLS * CLOUD_VOLUME_CELL_V_M[CLOUD_VOLUME_LEVELS - 1],
			CLOUD_VOLUME_SHELL_TOP_M);
}

// The grid-mean column's depth along a ray from the point p_x (Earth-local offset, height h_x, cosine mu_x to the
// vertical; r the sphere's radius) for rest_m (beyond the coarsest level): the mean extinction above the ray's lowest
// point there (cloud_column_mean_depth over the height to the cloud top) times the part of the path below the cloud top
// (above it the column holds nothing).
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
	float s_in;
	float s_out;
	bool shadow;
	if (!cloud_light_shell(h_x, r, mu_x, CLOUD_TOP_M, s_in, s_out, shadow)) {
		return 0.0;
	}
	float path = max(min(s_out, rest_m) - s_in, 0.0);
	vec2 ll = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + p_x + dir * s_low);
	float depth = cloud_column_mean_depth(ll, h_low).x;
	return depth * path / max(CLOUD_TOP_M - h_low, CLOUD_LIGHT_MIN_COLUMN_M);
}
#endif

#if defined(CLOUD_VOLUME_LIGHT) || defined(CLOUD_VOLUME_LIGHT_COLUMN)
layout(set = 1, binding = 3, std430) buffer CloudVolumeLightCounts {
	// per job of the batch (slot.x): [2 slot] cells a rate-limited write clamped, [2 slot + 1] cells with a sample
	// inside the levels' part of a ray (or of a column beyond the window) that no level held (the nesting failed)
	uint light_counts[];
};

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
		if (mode == CLOUD_VOLUME_LIGHT_DIRECT
				|| (mode == CLOUD_VOLUME_LIGHT_RATE && max(old[i], fresh[i]) >= CLOUD_VOLUME_TAU_SHADOW)) {
			written[i] = fresh[i];  // (rate-limited: into or out of "no light" - the header)
			continue;
		}
		float t_old = cloud_light_transmittance(old[i]);
		float t_new = cloud_light_transmittance(fresh[i]);
		if (mode == CLOUD_VOLUME_LIGHT_BLEND) {
			written[i] = cloud_light_depth_of(mix(t_old, t_new, clamp(weight, 0.0, 1.0)));
			continue;
		}
		// the step less what the half float's storage of the written tau may add (header): <= STEP_T stored
		float t = t_old + sign(t_new - t_old) * CLOUD_VOLUME_LIGHT_STEP_T;
		float margin = max(t * cloud_light_depth_of(t), t_new * fresh[i]) * CLOUD_VOLUME_HALF_EPSILON;
		if (abs(t_new - t_old) > CLOUD_VOLUME_LIGHT_STEP_T - margin) {
			written[i] = cloud_light_depth_of(t_old + sign(t_new - t_old) * (CLOUD_VOLUME_LIGHT_STEP_T - margin));
			clamped = true;
		} else {
			written[i] = fresh[i];
		}
	}
	return written;
}
#endif

#ifdef CLOUD_VOLUME_LIGHT
// tau toward a light (unit, Earth-local) from the cell centre of texture own_index (level k, frame f; g0 its grid point,
// p0 its offset, up its geodetic up): Earth's shadow, else the distance-stepped march (header) up to the field's top
// (top_m: cloud_light_field_top). missing: set when a sample of a level below the coarsest found no level holding it
// (the nesting failed).
float cloud_light_toward(vec3 light, vec3 g0, vec3 p0, vec3 up, int own_index, int k, int f, float top_m,
		out bool missing) {
	missing = false;
	float r = cloud_volume.frames[f].grid_hi.w;
	float mu = dot(up, light);
	float s_in;
	float s_out;
	bool shadow;
	bool ignored;
	cloud_light_shell(g0.z, r, mu, CLOUD_VOLUME_SHELL_TOP_M, s_in, s_out, shadow);  // Earth's shadow: the whole ray
	if (shadow) {
		return CLOUD_VOLUME_TAU_SHADOW;
	}
	if (!cloud_light_shell(g0.z, r, mu, top_m, s_in, s_out, ignored)) {
		return 0.0;  // above the field, moving away
	}
	float tau = 0.0;
	float s0 = 0.0;  // the segment's start along the ray
	vec3 g_s0 = g0;  // its grid point (valid while s0_known)
	bool s0_known = true;
	bool ended = false;  // a sample found no level (beyond the coarsest window)
	for (int j = k; j < CLOUD_VOLUME_LEVELS && s0 < s_out; j++) {
		float s1 = s0 + float(CLOUD_LIGHT_LEVEL_CELLS) * cloud_light_cell_along(j, mu);
		float a = max(s0, s_in);
		float b = min(s1, s_out);
		bool known = s0_known && a == s0;
		s0_known = false;
		if (b > a) {
			// The segment's grid path: the quadratic through its exact grid points at a, (a + b) / 2, b.
			vec3 g_a = known ? g_s0 : cloud_volume_grid_in(f, p0 + light * a);
			vec3 g_m = cloud_volume_grid_in(f, p0 + light * (0.5 * (a + b)));
			vec3 g_b = cloud_volume_grid_in(f, p0 + light * b);
			float len = b - a;
			vec3 b1 = (4.0 * g_m - 3.0 * g_a - g_b) / len;
			vec3 b2 = 2.0 * (g_b + g_a - 2.0 * g_m) / (len * len);
			const int samples = CLOUD_LIGHT_LEVEL_CELLS * CLOUD_LIGHT_SAMPLES_PER_CELL;
			float ds = len / float(samples);
			for (int i = 0; i < samples && !ended; i++) {
				float t = (float(i) + 0.5) * ds;
				bool found;
				float rho = cloud_light_rho_at(g_a + t * (b1 + t * b2), p0 + light * (a + t), j, k, own_index, f, found);
				if (!found) {
					ended = true;  // out of every window: the column from this sample's start
					s1 = a + float(i) * ds;
				}
				tau += rho * ds;
			}
			g_s0 = g_b;
			s0_known = !ended && b == s1;
		}
		s0 = s1;
		if (ended) {
			break;
		}
	}
	// A ray of a level below the coarsest leaves level 7's window only beyond its segments: unheld, the nesting failed.
	missing = ended && k + 1 < CLOUD_VOLUME_LEVELS;
	if (s0 < s_out) {
		// beyond the coarsest level's segment: the grid-mean column
		vec3 g_q = s0_known ? g_s0 : cloud_volume_grid_in(f, p0 + light * s0);
		float mu_q = ((r + g0.z) * mu + s0) / (r + g_q.z);
		tau += cloud_light_column_rest(p0 + light * s0, light, g_q.z, mu_q, r, s_out - s0);
	}
	return min(tau, CLOUD_VOLUME_TAU_SHADOW);
}

void main() {
	int block = cloud_light_dispatch_unit(2, light_push.job.w);
	if (block < 0) {
		return;
	}
	ivec3 blocks = (light_job.size.xyz + 3) / 4;
	ivec3 gi = ivec3(block % blocks.x, (block / blocks.x) % blocks.y, block / (blocks.x * blocks.y)) * 4
			+ ivec3(gl_LocalInvocationID);
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
	bool fallback;
	bool fallback_moon = false;
	float top_m = cloud_light_field_top(light_job.moon.w);
	fresh.x = cloud_light_toward(normalize(light_job.sun.xyz), g0, p0, up, index, k, f, top_m, fallback);
	fresh.w = moon ? cloud_light_toward(normalize(light_job.moon.xyz), g0, p0, up, index, k, f, top_m, fallback_moon)
			: CLOUD_VOLUME_TAU_SHADOW;
	bool clamped;
	vec4 written = cloud_light_written(old, fresh, bvec4(true, false, false, moon || mode == CLOUD_VOLUME_LIGHT_DIRECT),
			mode, light_job.sun.w, clamped);
	if (clamped) {
		atomicAdd(light_counts[2 * light_job.slot.x], 1u);
	}
	if (fallback || fallback_moon) {
		atomicAdd(light_counts[2 * light_job.slot.x + 1], 1u);
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

// The grid-mean column's depth beyond a windowed level's vertical window from its outer cell centre (height h_m, Earth-
// local offset p): above it (`above`) or below it down to the shell's bottom.
float cloud_light_beyond_column(vec3 p, float h_m, bool above) {
	vec2 ll = cloud_lat_lon(atmosphere_camera_hi + atmosphere_camera_lo + p);
	float depth_above = cloud_column_mean_depth(ll, h_m).x;
	return above ? depth_above
			: max(cloud_column_mean_depth(ll, CLOUD_VOLUME_SHELL_BOTTOM_M).x - depth_above, 0.0);
}

shared float column_sum[CLOUD_VOLUME_CELLS_V];  // the window's trapezoid prefix sums (cell i: centres 0 .. i)
shared float beyond_sum[2 * CLOUD_VOLUME_CELLS_V];  // the samples beyond the window: above, then below
shared float column_out[2];  // above_out, below_out
shared vec3 column_p;  // the outer top centre's Earth-local offset
shared vec3 column_n;  // the column's (unnormalised) geodetic normal: the offset is linear in the height along it
shared bool column_clamped;
shared bool beyond_missing;

void main() {
	// One workgroup per column of the level's jobs, one invocation per window cell / box cell (Task 6 review round 1: a
	// column walked by one invocation is latency-bound - ~0.33 ms per dispatch however few columns).
	int i = int(gl_LocalInvocationID.x);
	int column = cloud_light_dispatch_unit(1, light_push.job.z);
	if (column < 0) {
		return;  // the whole workgroup: no barrier is left unmatched
	}
	ivec2 xy = light_job.first.xy + ivec2(column % light_job.size.x, column / light_job.size.x);
	int k = light_job.first.w;
	int index = light_job.size.w;
	int f = light_job.info.x;
	int mode = light_job.info.y;
	float v = CLOUD_VOLUME_CELL_V_M[k];
	// The window's cells of the column (levels 2-7: the whole shell).
	int z0 = cloud_volume.levels[index].z;
	int n = int(cloud_volume_window(k).z);
	float rho_i = i < n ? cloud_light_cell_rho(index, ivec3(xy, z0 + i)) : 0.0;
	column_sum[i] = rho_i;
	if (i == 0) {
		column_clamped = false;
	}
	barrier();
	// The trapezoid between centre i - 1 and centre i, then its inclusive prefix sum (Hillis-Steele) over the window.
	float value = i > 0 && i < n ? 0.5 * (column_sum[i - 1] + rho_i) * v : 0.0;
	barrier();
	column_sum[i] = value;
	barrier();
	for (int step = 1; step < CLOUD_VOLUME_CELLS_V; step *= 2) {
		float add = i >= step ? column_sum[i - step] : 0.0;
		barrier();
		column_sum[i] += add;
		barrier();
	}
	// Beyond the outer centres: up to the shell's top / down to its bottom. Where the window ends inside the shell
	// (levels 0-1) the coarser shown levels' density along the vertical (cloud_light_rho_at from level k + 1), one
	// midpoint sample per invocation above (up to the field's top) and one below (down to the shell's bottom); else the
	// outer cell's value held over the last half cell (the reads clamp there).
	vec2 xy_m = (vec2(xy) + 0.5) * CLOUD_VOLUME_CELL_H_M[k];
	float h_top = CLOUD_VOLUME_SHELL_BOTTOM_M + (float(z0 + n) - 0.5) * v;
	float h_bottom = CLOUD_VOLUME_SHELL_BOTTOM_M + (float(z0) + 0.5) * v;
	bool open_top = h_top + 0.5 * v < CLOUD_VOLUME_SHELL_TOP_M;
	bool open_bottom = (cloud_volume.levels[index].w & CLOUD_VOLUME_CLAMPED_BOTTOM) == 0
			&& h_bottom - 0.5 * v > CLOUD_VOLUME_SHELL_BOTTOM_M;
	if (i == 0) {
		CloudVolumeFrame fr = cloud_volume.frames[f];
		vec3 normal;
		column_p = cloud_volume_offset_of(f, (vec3(xy_m, h_top) - fr.grid_hi.xyz) - fr.grid_lo.xyz, normal);
		column_n = normal;
		beyond_missing = false;
	}
	barrier();
	float top_m = cloud_light_field_top(light_job.moon.w);
	float above_i = 0.0;
	float below_i = 0.0;
	if (open_top && top_m > h_top) {
		float dz = (top_m - h_top) / float(CLOUD_VOLUME_CELLS_V);
		float h = h_top + (float(i) + 0.5) * dz;
		bool found;
		above_i = cloud_light_rho_at(vec3(xy_m, h), column_p + column_n * (h - h_top), k + 1, k, index, f, found) * dz;
		if (!found) {
			beyond_missing = true;
		}
	}
	if (open_bottom) {
		float dz = (h_bottom - CLOUD_VOLUME_SHELL_BOTTOM_M) / float(CLOUD_VOLUME_CELLS_V);
		float h = CLOUD_VOLUME_SHELL_BOTTOM_M + (float(i) + 0.5) * dz;
		bool found;
		below_i = cloud_light_rho_at(vec3(xy_m, h), column_p + column_n * (h - h_top), k + 1, k, index, f, found) * dz;
		if (!found) {
			beyond_missing = true;
		}
	}
	beyond_sum[i] = above_i;
	beyond_sum[CLOUD_VOLUME_CELLS_V + i] = below_i;
	barrier();
	for (int stride = CLOUD_VOLUME_CELLS_V / 2; stride > 0; stride /= 2) {
		if (i < stride) {
			beyond_sum[i] += beyond_sum[i + stride];
			beyond_sum[CLOUD_VOLUME_CELLS_V + i] += beyond_sum[CLOUD_VOLUME_CELLS_V + i + stride];
		}
		barrier();
	}
	if (i == 0) {
		float above_out = cloud_light_cell_rho(index, ivec3(xy, z0 + n - 1)) * max(CLOUD_VOLUME_SHELL_TOP_M - h_top, 0.0);
		float below_out = rho_i * max(h_bottom - CLOUD_VOLUME_SHELL_BOTTOM_M, 0.0);
		if (open_top) {  // no coarser level shown: the grid-mean column
			above_out = beyond_missing ? cloud_light_beyond_column(column_p, h_top, true) : beyond_sum[0];
		}
		if (open_bottom) {
			below_out = beyond_missing ? cloud_light_beyond_column(column_p + column_n * (h_bottom - h_top), h_bottom,
					false) : beyond_sum[CLOUD_VOLUME_CELLS_V];
		}
		if (beyond_missing) {
			atomicAdd(light_counts[2 * light_job.slot.x + 1], 1u);
		}
		column_out[0] = above_out;
		column_out[1] = below_out;
	}
	barrier();
	// The box's cell i (boxes lie inside the storage, whose cells outside the window - the guard - take the nearest
	// window cell's sums).
	if (i < light_job.size.z) {
		int z = light_job.first.z + i;
		float below = column_sum[clamp(z - z0, 0, n - 1)];
		float total = column_sum[n - 1];
		ivec3 texel = cloud_volume_wrap(ivec3(xy, z), CLOUD_VOLUME_SIZE);
		vec4 old = imageLoad(volume_light, texel);
		vec4 fresh = vec4(old.x, min(column_out[1] + below, CLOUD_VOLUME_TAU_SHADOW),
				min(column_out[0] + (total - below), CLOUD_VOLUME_TAU_SHADOW), old.w);
		bool clamped;
		imageStore(volume_light, texel, cloud_light_written(old, fresh, bvec4(false, true, true, false), mode,
				light_job.sun.w, clamped));
		if (clamped) {
			column_clamped = true;
		}
	}
	barrier();
	if (i == 0 && column_clamped) {
		atomicAdd(light_counts[2 * light_job.slot.x], 1u);
	}
}
#endif

#ifdef CLOUD_VOLUME_PROVISIONAL
// The grid-mean column's tau toward a light from the centre (height h): Earth's shadow, else tau_above times the path
// below the cloud top over the height to it.
float cloud_provisional_toward(vec3 light, vec3 up, float h, float r, float tau_above) {
	float mu = dot(up, light);
	float s_in;
	float s_out;
	bool shadow;
	bool inside = cloud_light_shell(h, r, mu, CLOUD_TOP_M, s_in, s_out, shadow);
	if (shadow) {
		return CLOUD_VOLUME_TAU_SHADOW;
	}
	if (!inside) {
		return 0.0;
	}
	return min(tau_above * (s_out - s_in) / max(CLOUD_TOP_M - h, CLOUD_LIGHT_MIN_COLUMN_M), CLOUD_VOLUME_TAU_SHADOW);
}

void main() {
	light_job_index = light_push.job.x;  // one dispatch per job
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

#ifdef CLOUD_VOLUME_LIGHT_REFERENCE
// [E] The reference's step: this share of one cell along the ray of the level at the point (converged: halving it moves
// the slab's tau by < 0.1 %).
const float CLOUD_REFERENCE_STEP_CELLS = 0.1;
// [E] At most this many steps per ray (a 100 km path at level 0's 5 m steps fits).
const int CLOUD_REFERENCE_MAX_STEPS = 40000;
layout(set = 1, binding = 1, std140) uniform CloudVolumeReference {
	vec4 dir;   // xyz: toward the light (Earth-local render axes), w: the height (m) above which the field is clear
	ivec4 info; // x: cells, y: the channel of the stored texel compared (0 tau_sun, 3 tau_moon)
} reference;
layout(set = 1, binding = 2, std430) readonly buffer CloudVolumeReferenceIn {
	ivec4 reference_cells[];  // xyz: the cell (global), w: its level (frame 0)
};
layout(set = 1, binding = 3, std430) writeonly buffer CloudVolumeReferenceOut {
	vec4 reference_out[];  // x: the converged tau, y: the stored texel's channel, z: steps, w: 1 = the column took a rest
};

void main() {
	int i = int(gl_GlobalInvocationID.x);
	if (i >= reference.info.x) {
		return;
	}
	ivec4 cell = reference_cells[i];
	int k = cell.w;
	vec3 p0;
	vec3 up;
	vec3 g0 = cloud_light_centre(k, cell.xyz, 0, p0, up);
	vec4 stored = cloud_volume_fetch_light(k, k, g0);
	vec3 dir = normalize(reference.dir.xyz);
	float r = cloud_volume.frames[0].grid_hi.w;
	float mu = dot(up, dir);
	float s_in;
	float s_out;
	bool shadow;
	bool inside = cloud_light_shell(g0.z, r, mu, CLOUD_VOLUME_SHELL_TOP_M, s_in, s_out, shadow);
	float tau = 0.0;
	int steps = 0;
	float rest = 0.0;
	if (shadow) {
		tau = CLOUD_VOLUME_TAU_SHADOW;
	} else if (inside) {
		// midpoint steps through the finest level >= k holding the point (shown in frame 0), faded as the density
		float s = 0.0;
		int j = k;
		for (; steps < CLOUD_REFERENCE_MAX_STEPS && s < s_out; steps++) {
			vec3 g_s = cloud_volume_grid_in(0, p0 + dir * s);
			float mu_s = ((r + g0.z) * mu + s) / (r + g_s.z);
			if (g_s.z > reference.dir.w && mu_s > 0.0) {
				break;  // above the clear height and climbing: nothing more
			}
			float ds = min(CLOUD_REFERENCE_STEP_CELLS * cloud_light_cell_along(j, mu_s), s_out - s);
			vec3 g = cloud_volume_grid_in(0, p0 + dir * (s + 0.5 * ds));
			int level = -1;
			float fade = 0.0;
			for (int l = k; l < CLOUD_VOLUME_LEVELS && level < 0; l++) {
				if (cloud_light_shown(l, 0) || l == k) {
					float w = cloud_volume_fade(l, l, g);
					if (w > 0.0) {
						level = l;
						fade = w;
					}
				}
			}
			if (level < 0) {
				tau += cloud_light_column_rest(p0 + dir * s, dir, g_s.z, mu_s, r, s_out - s);
				rest = 1.0;
				break;
			}
			j = level;
			tau += cloud_light_rho(level, level, fade, g, 0) * ds;
			s += ds;
		}
	}
	reference_out[i] = vec4(min(tau, CLOUD_VOLUME_TAU_SHADOW), stored[reference.info.y], float(steps), rest);
}
#endif
