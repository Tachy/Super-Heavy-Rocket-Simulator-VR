// The hybrid cloud march (live VR prototype, 2026-10-29; docs/subsystems/weather-clouds.md "Hybrid march"): today's
// march (cloud_pass.gdshaderinc cloud_march - the shapes, the detail and the weather evaluated per sample, its fixed
// step schedule: the look reference) with two parts of the cloud world volume (cloud_volume.gdshaderinc, its baked
// clipmap) - GLSL 450 only, compiled by CloudEffect with CLOUD_MARCH + CLOUD_HYBRID + CLOUD_VOLUME_LOOKUP after
// cloud_volume.gdshaderinc and before cloud_pass.gdshaderinc, never included by a Godot shader.
//
// 1. Empty-space skipping: where the march's last sample was clear (outside a silhouette, nothing carried), the
//    volume's distance field at the sample's point - the finest frame-0 level whose window holds it, one texel fetch -
//    gives a ball (true metres) that holds no density of the point model in either snapshot (the occupancy's bound
//    covers the trilinear support and one cell around it). Only that proven-empty ball is skipped (less the grid
//    track's error); inside occupied space the march steps exactly as today. A sample whose block is occupied mutes
//    the queries until the ray leaves that block (no fetch per fine step next to a cloud). The ball alone, never the
//    free box's exit: the box's vertical exit carried near-vertical rays through cloud bases (Task 7 cost diagnosis,
//    section 5: the ball alone gave the core's zenith tau within 4 %).
// 2. The light (CLOUD_HYBRID_VOLUME_LIGHT): at the march's light point one trilinear fetch of the volume's baked
//    (tau_sun, tau_below, tau_above, tau_moon) - faded toward level + 1 in the fade band (two fetches) - instead of
//    the light cache's cascades and the per-sample light march; the column terms (the sky's and the transfer table's)
//    from the same fetch. Beyond the volume (no level holds the point) the far tier's light path. Without the define
//    the march's own light (the comparison option).
//
// The grid track: the volume's grid (camera-relative geodetic solve, cloud_volume_grid_in) is solved at three points
// of the ray, s, s + P, s + 2 P, and carried as the quadratic through them - no solve per step. A straight ray's grid
// coordinates are smooth (height: the curvature term s^2 (1 - mu^2) / 2 R; horizontally the azimuthal map), so the
// interpolant's error is the cubic term: |c3| ~ 0.19 / R^2 (the height's third derivative), at 2.5 P = 40 km
// 1.9 x P^3 x 4.7e-15 ~ 0.04 m; the float32 rounding of the grid (~0.1 m at 1e6 m) dominates. Margin per skip:
// CLOUD_HYBRID_MARGIN_M + CLOUD_HYBRID_MARGIN_PER_M x the distance from the anchor.

#ifdef CLOUD_HYBRID

// [E] The grid track's sample spacing P (m along the ray) and its valid range around the anchor (in P).
const float CLOUD_HYBRID_ANCHOR_M = 16000.0;
const float CLOUD_HYBRID_REACH = 2.5;
const float CLOUD_HYBRID_BEHIND = 0.25;
// [E] Taken off every skipped ball: the grid track's error and the float32 rounding of the grid (see the header).
const float CLOUD_HYBRID_MARGIN_M = 2.0;
const float CLOUD_HYBRID_MARGIN_PER_M = 1.0e-4;
// [E] After a ball no longer than a fine step the queries pause for this many fine steps.
const float CLOUD_HYBRID_QUIET_STEPS = 8.0;
// A point counts as held by a level from this many cells inside its window (the free box is clipped to it).
const float CLOUD_HYBRID_LEVEL_MARGIN_CELLS = 1.0;
// The light volume's optical depth from this on stands for "no light" (cloud_volume_march.gdshaderinc
// CLOUD_VOLUME_UNLIT_TAU: half of the light volume's no-light value, the fade band mixes the two).
const float CLOUD_HYBRID_UNLIT_TAU = 3.0e4;
const float CLOUD_HYBRID_NO_LIGHT_TAU = 6.0e4;

// The grid track of one ray (see the header) and the distance before which no free-space query is made.
struct CloudHybridTrack {
	float anchor_m;       // the track's anchor distance (-1e30: none yet)
	vec3 g0;              // the grid point at the anchor (frame 0)
	vec3 g1;              // d grid / ds there
	vec3 g2;              // half the second derivative
	float quiet_until_m;  // no free-space query before this distance (an occupied block, a small ball, no level)
};

CloudHybridTrack cloud_hybrid_track() {
	CloudHybridTrack t;
	t.anchor_m = -1.0e30;
	t.g0 = vec3(0.0);
	t.g1 = vec3(0.0);
	t.g2 = vec3(0.0);
	t.quiet_until_m = -1.0e30;
	return t;
}

// Anchors the track at distance s of the ray eye + dir s (three camera-relative solves; their differences taken from
// the camera-relative parts, never of two absolute grid values).
void cloud_hybrid_anchor(inout CloudHybridTrack t, vec3 eye, vec3 dir, float s) {
	float p = CLOUD_HYBRID_ANCHOR_M;
	vec3 q0 = cloud_volume_grid_delta(0, eye + dir * s);
	vec3 d1 = cloud_volume_grid_delta(0, eye + dir * (s + p)) - q0;
	vec3 d2 = cloud_volume_grid_delta(0, eye + dir * (s + 2.0 * p)) - q0;
	CloudVolumeFrame fr = cloud_volume.frames[0];
	t.anchor_m = s;
	t.g0 = fr.grid_hi.xyz + (fr.grid_lo.xyz + q0);
	t.g1 = (4.0 * d1 - d2) / (2.0 * p);
	t.g2 = (d2 - 2.0 * d1) / (2.0 * p * p);
}

// The grid point (frame 0) of the ray at distance s (re-anchored outside the track's range); x: s - the anchor.
vec3 cloud_hybrid_grid(inout CloudHybridTrack t, vec3 eye, vec3 dir, float s, out float x) {
	x = s - t.anchor_m;
	if (x < -CLOUD_HYBRID_BEHIND * CLOUD_HYBRID_ANCHOR_M || x > CLOUD_HYBRID_REACH * CLOUD_HYBRID_ANCHOR_M) {
		cloud_hybrid_anchor(t, eye, dir, s);
		x = 0.0;
	}
	return t.g0 + x * (t.g1 + x * t.g2);
}

// The finest shown level of frame 0 whose window holds the grid point at least margin_cells inside (-1: none); fade:
// the level's weight there (cloud_volume_fade's).
int cloud_hybrid_level(vec3 grid_m, float margin_cells, out float fade) {
	fade = 0.0;
	for (int k = 0; k < CLOUD_VOLUME_LEVELS; k++) {
		ivec4 lv = cloud_volume.levels[k];
		if ((lv.w & (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)) != (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)
				|| (lv.w >> CLOUD_VOLUME_FRAME_SHIFT) != 0) {
			continue;
		}
		vec3 u = cloud_volume_cells(k, grid_m) - vec3(lv.xyz);
		vec3 size = cloud_volume_window(k);
		float d = min(min(u.x, size.x - u.x), min(u.y, size.y - u.y));
		if ((lv.w & CLOUD_VOLUME_VERTICAL_FADE) != 0) {
			d = min(d, size.z - u.z);
			if ((lv.w & CLOUD_VOLUME_CLAMPED_BOTTOM) == 0) {
				d = min(d, u.z);
			}
		}
		if (d >= margin_cells) {
			fade = clamp((d - 0.5) / CLOUD_VOLUME_FADE_CELLS, 0.0, 1.0);
			return k;
		}
	}
	return -1;
}

// The empty stretch (true metres) the march may skip from distance a (its last sample clear): the volume's free ball
// at the point less the track's margin when it is longer than min_skip_m, else 0 (and the queries muted: inside an
// occupied block until the ray leaves the block, past a small ball until its end, beyond every level for good).
float cloud_hybrid_skip(inout CloudHybridTrack t, vec3 eye, vec3 dir, float a, float min_skip_m) {
	float x;
	vec3 g = cloud_hybrid_grid(t, eye, dir, a, x);
	float fade;
	int k = cloud_hybrid_level(g, CLOUD_HYBRID_LEVEL_MARGIN_CELLS, fade);
	if (k < 0) {
		t.quiet_until_m = 1.0e30;  // a ray leaves the levels' windows once (they are boxes around the camera)
		return 0.0;
	}
	vec3 lo;
	vec3 hi;
	float free_m = cloud_volume_level_free_m(g, k, lo, hi);
	CLOUD_COUNT_FETCHES(1)
	if (free_m <= 0.0) {
		// An occupied block: no query until the ray leaves it (linear over one block: ~mm of curvature).
		vec3 c = cloud_volume_cells(k, g);
		vec3 dc = (t.g1 + 2.0 * x * t.g2)
				/ vec3(CLOUD_VOLUME_CELL_H_M[k], CLOUD_VOLUME_CELL_H_M[k], CLOUD_VOLUME_CELL_V_M[k]);
		vec3 b0 = floor(c / float(CLOUD_VOLUME_MIP_DIV)) * float(CLOUD_VOLUME_MIP_DIV);
		vec3 b1 = b0 + float(CLOUD_VOLUME_MIP_DIV);
		float exit_m = 1.0e30;
		for (int axis = 0; axis < 3; axis++) {
			if (dc[axis] > 1.0e-12) {
				exit_m = min(exit_m, (b1[axis] - c[axis]) / dc[axis]);
			} else if (dc[axis] < -1.0e-12) {
				exit_m = min(exit_m, (b0[axis] - c[axis]) / dc[axis]);
			}
		}
		t.quiet_until_m = a + clamp(exit_m, 0.0, 1.0e6);
		return 0.0;
	}
	float skip_m = free_m - CLOUD_HYBRID_MARGIN_M - CLOUD_HYBRID_MARGIN_PER_M * abs(x);
	if (skip_m <= min_skip_m) {
		// a small ball (near cloud, partial cover): the march's own steps for a while (measured: a query per fine
		// step there cost more than the skips saved)
		t.quiet_until_m = a + max(skip_m, CLOUD_HYBRID_QUIET_STEPS * min_skip_m);
		return 0.0;
	}
	return skip_m;
}

#ifdef CLOUD_HYBRID_VOLUME_LIGHT
// The volume's light terms (tau_sun, tau_below, tau_above, tau_moon) at distance d of the ray, faded toward level + 1
// in the fade band (not across a re-frame's frames); moon: the levels' Moon weight. False where no level holds the
// point (the caller takes the far tier's light path there).
bool cloud_hybrid_light_terms(inout CloudHybridTrack t, vec3 eye, vec3 dir, float d, out vec4 tau, out float moon) {
	float x;
	vec3 g = cloud_hybrid_grid(t, eye, dir, d, x);
	float fade;
	int k = cloud_hybrid_level(g, 0.5, fade);
	tau = vec4(0.0);
	moon = 0.0;
	if (k < 0) {
		return false;
	}
	if (k + 1 >= CLOUD_VOLUME_LEVELS || ((cloud_volume.levels[k + 1].w >> CLOUD_VOLUME_FRAME_SHIFT) & 1) != 0) {
		fade = 1.0;
	}
	CLOUD_LIGHT_BEGIN
	tau = cloud_volume_fetch_light(k, k, g);
	CLOUD_COUNT_FETCHES(1)
	moon = cloud_volume.moon_weight[k / 4][k % 4];
	if (fade < 1.0) {
		tau = mix(cloud_volume_fetch_light(k + 1, k + 1, g), tau, fade);
		CLOUD_COUNT_FETCHES(1)
		moon = mix(cloud_volume.moon_weight[(k + 1) / 4][(k + 1) % 4], moon, fade);
	}
	CLOUD_LIGHT_END
	return true;
}
#endif

#endif
