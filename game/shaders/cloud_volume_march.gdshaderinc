// The cloud world volume's march (world-volume plan Task 7; spec D, light per spec C): one ray through the baked
// density, its occupancy and light (cloud_volume.gdshaderinc) - GLSL 450 only, compiled by CloudEffect after the
// march's files (clouds.gdshaderinc, cloud_pass.gdshaderinc with CLOUD_VOLUME_FULL: its machinery without its images
// and main) and cloud_volume.gdshaderinc (CLOUD_VOLUME_LOOKUP). Kernel CLOUD_VOLUME_FULL: one full-resolution ray per
// eye pixel over [0, scene] - the reference the reduced tiers of Task 8 must match.
//
// Steps (STOP-1 ruling, spec D amended):
// - Empty space: sphere tracing on the distance field of the finest shown level holding the point
//   (cloud_volume_free_m, true metres; the ray is clipped to the cloud shell first - the field is 0 outside it).
// - Occupied space: one step to the exit of the finest level's cell (exact: the next segment starts at that exit),
//   capped by CLOUD_STRUCTURE_TEXEL_FRACTION of the finest sub-cell structure the footprint resolves (the shape's
//   texel where the shape residual is on, the detail levels' middle cells where their weight is > 0), >= 2 m. The
//   density is sampled once at the step's middle (the trilinear bake there, plus the sub-cell field): Beer-Lambert over
//   the step is exact for that constant. Error bound of the mid-step sample: inside one cell the trilinear field along
//   a straight ray is a cubic in s (a product of three linear factors), not linear between texel centres; the midpoint
//   rule's error on the step's optical depth is L^3 / 24 max|f''| and f'' <= 6 Delta / L^2 over a cell crossing
//   (Delta: the spread of the cell's 8 corners), so |tau_mid - tau| <= Delta L / 4 for a full-cell step.
// - The source varies inside a step (the light volume is trilinear, the air light changes over kilometres): it is
//   integrated as LINEAR between the step's ends in closed form: with tau' = s tau (s the scale of the attenuation:
//   the transport scale k for the sky, w_D for the table's lights) the step adds T (S0 A + (S1 - S0) B) / s,
//   A = 1 - e^-tau', B = A / tau' - e^-tau' (Hillaire 2016's segment integral with a linear source). The ends' sources
//   come from light NODES (ruling R3: the light evaluated once per node, not per step): the source evaluated at
//   nodes CLOUD_VOLUME_LIGHT_NODE_CELLS of the level's cell apart along the ray (or at a step's end where a step is
//   longer), linear between them - the light volume is trilinear on the cell's scale, the structure-capped steps
//   through the sub-cell field are far shorter (V1: light fetches per occupied step 9.6 -> 0.46).
// - The silhouette entry is bisected (CLOUD_SILHOUETTE_BISECTIONS, kept): where a mid-step density crosses
//   CLOUD_SILHOUETTE_EXTINCTION_PER_M from outside, the step restarts at the entry found between its start and middle.
// - Termination as today (the scaled transmittance CLOUD_MIN_SCALED_TRANSMITTANCE; rays toward the Sun's disk marched
//   on to CLOUD_SUN_MIN_TRANSMITTANCE without light samples).
//
// Sub-cell field (spec D, decision 6; controller ruling R1: the threshold form): where the pixel footprint f is finer
// than the level's cell, the trilinear (rho, sigma_c) at the sample give the cell's cover c = rho / sigma_c, which the
// field redistributes as a cover (not as haze): density = sigma_c Phi((v - t) / s_e), t = -sqrt(s_e^2 + sigma_v^2)
// Phi^-1(c) (cloud_volume_density_at; core cloud_noise.hpp "Sub-cell field": the cell's mean stays rho). v: the shape's
// residual above the cell's scale w_s (s_f - s_cell) (s_cell at the cell's shape level - 0 below the shape's 250 m
// texel; through a fade band the cell size mixed as the levels' densities, ruling R5) plus the detail levels'
// perturbation (band limit w_k = clamp(log2(l_k / f) - 1, 0, 1)) Gaussianised by the core's table. Early-outs: rho = 0
// (clear), all weights 0 (the bake), c >= CLOUD_VOLUME_SOLID_COVER (solid: no detail fetch). The curl distortion
// (edge_curl, Nubis): the detail is read at the sample moved by edge_curl x the level-0 detail's middle cell x its
// weight w_0 x the tileable curl field (divergence-free: the detail's statistics stay).
// Approximations (documented, measured by Task 7's checks): the residual's variance is exact for the footprint's shape
// level 0 (the volume's range), the residual is used un-Gaussianised; the shape is read with the cumulus coordinates
// (the volume does not know the cell's type mix).
//
// Light per sample (spec C): the light volume's (tau_sun, tau_below, tau_above, tau_moon) at the point (faded as the
// density): the transfer table (cloud_scattering_source_mc; CLOSED_FORM: _3d) with a = mu_e tau_sun, b = tau_below,
// the sky with cloud_sky_scattering_source(tau_above, tau_above + tau_below) x the air's sky irradiance (refreshed
// every CLOUD_LIGHT_REFRESH_M / CLOUD_LIGHT_REFRESH_FRACTION as today), the Moon's J x the level's Moon weight.
// Surface correction (spec C): at each light node where the step weight T (1 - exp(-Delta tau)) >
// CLOUD_VOLUME_SURFACE_WEIGHT and the level's cell is coarser than the footprint, tau_sun = light_entry_samples
// midpoint samples of the per-sample density (baked + sub-cell) over the first CLOUD_VOLUME_ENTRY_CELLS cells of the
// level toward the Sun plus the volume's tau_sun from there. (Ruling R3 asked for the correction on the ray's first 4
// steps only: measured, that broke cloud_check section 19's slab at Sun 20 deg - worst 11.8 % against the converged
// light, bound 5 % - while the correction at the nodes costs one per half cell and passes it at 3.3 %.)
//
// Debug outputs (the checks' physical quantities): 1 / 4 the background transmittance, 2 / 7 the Sun's scattered
// radiance per unit irradiance, 12 (cloud distance, alpha); 1 and 2 without the sub-cell field (the bake alone: the
// volume's "grid mean"), 4 and 7 with it. The others are today's march (CloudEffect runs it for them).

// Globals (clouds.gdshaderinc: one set 0 for every kernel of the march's files): cloud_noise_curl, cloud_edge_curl
// (CloudLayer.edge_curl), cloud_light_entry_samples (CloudLayer.light_entry_samples).

// [E] Shortest step (spec D: >= 2 m).
const float CLOUD_VOLUME_MIN_STEP_M = 2.0;
// [E] Steps per ray at most (the counters show what rays take; far above the spec's ~100).
const int CLOUD_VOLUME_MAX_STEPS = 4096;
// Surface correction (spec C): the step weight above which a light node gets it, the cells of the level it covers.
const float CLOUD_VOLUME_SURFACE_WEIGHT = 1.0e-3;
const float CLOUD_VOLUME_ENTRY_CELLS = 2.0;
// [E] A cover this close to 1 is the solid interior (the half float's resolution of rho / sigma_c near 1: 2^-10).
const float CLOUD_VOLUME_SOLID_COVER = 1.0 - 1.0 / 1024.0;
// [E] The curl field's tile: the level-0 detail tile (cloud_noise.hpp: 31 m texels, ~250 m features); its unit
// displacement: the level-0 detail's middle cell (the brief's "1 x the detail cell size").
const float CLOUD_VOLUME_CURL_TILE_M = CLOUD_DETAIL_TILE_M;
const float CLOUD_VOLUME_CURL_UNIT_M = CLOUD_DETAIL_TILE_M / 4.0;
// [E] The shape's mips (cloud_noise_shape_mips: level 0 = the shape's mip 1) read for a cell up to this level.
const float CLOUD_VOLUME_SHAPE_MAX_LOD = 6.0;
// The rays' series of e^-x terms below this tau' (the closed form B = A / tau' - e^-tau' cancels).
const float CLOUD_VOLUME_SERIES_TAU = 1.0e-3;
// [E] The free box's vertical margin as a share of its heights (the model height |M p| - a against the grid's height
// above WGS84 differ by up to e^2 / 2 ~ 0.34 % of the height).
const float CLOUD_VOLUME_BOX_MARGIN = 0.004;
// A light-volume optical depth from this on stands for "no light" (the fade band mixes the no-light value with a
// finite one: half of it).
const float CLOUD_VOLUME_UNLIT_TAU = 3.0e4;
// The light volume's "no light" (Earth's shadow, the Moon off; cloud_volume_light.gdshaderinc CLOUD_VOLUME_TAU_SHADOW).
const float CLOUD_VOLUME_NO_LIGHT_TAU = 6.0e4;
// [E] Cells a step's start is moved along the ray to pick the cell it leaves (an exit's float32 rounding on the
// boundary would otherwise read the cell left behind: a zero-length step).
const float CLOUD_VOLUME_EXIT_NUDGE_CELLS = 1.0e-4;
// The occupied steps where the sub-cell field acts (the structure term) and the silhouette's bisections (today's
// march's CLOUD_STRUCTURE_TEXEL_FRACTION / CLOUD_SILHOUETTE_BISECTIONS).
const float CLOUD_VOLUME_STRUCTURE_FRACTION = CLOUD_STRUCTURE_TEXEL_FRACTION;
const int CLOUD_VOLUME_BISECTIONS = CLOUD_SILHOUETTE_BISECTIONS;
// [E] The light nodes' spacing (ruling R3: the source evaluated once per node, not per step): this share of the level's
// cell along the ray (its fastest axis) - the light volume is trilinear per cell, so the linear source between nodes
// half a cell apart misses only the trilinear's curvature (Delta / 16 of a cell's corner spread at most).
const float CLOUD_VOLUME_LIGHT_NODE_CELLS = 0.5;

// Checks only (cloud_light_entry_samples < 0, cloud_check section 19, ruling C12): the converged per-sample light
// march toward the Sun through the per-sample density instead of the light volume - steps growing from
// CLOUD_VOLUME_CONVERGED_FIRST_M by CLOUD_VOLUME_CONVERGED_GROWTH of the distance, to the shell's top or out of the
// coarsest level (the light volume's own grid-mean tail beyond the levels is not needed over a check's few km).
const float CLOUD_VOLUME_CONVERGED_FIRST_M = 2.0;
const float CLOUD_VOLUME_CONVERGED_GROWTH = 0.01;
const int CLOUD_VOLUME_CONVERGED_STEPS = 2048;
// The march's density mode for its light samples (set per ray: the debug outputs 1 / 2 read the bake alone).
bool cloud_volume_sub_cell = true;

// The volume's sample at a point: its level, the level's grid (its frame's) and the next coarser level's, the fade.
struct CloudVolumeAt {
	int level;      // -1: no shown level holds the point
	float fade;     // the level's weight (1 - the next coarser level's)
	vec3 grid;      // the point in the level's frame
	vec3 grid_up;   // the point in level + 1's frame
};

// The frame index (0 / 1: a re-frame in progress) of texture index k.
int cloud_volume_frame_of(int k) {
	return (cloud_volume.levels[k].w >> CLOUD_VOLUME_FRAME_SHIFT) & 1;
}

bool cloud_volume_shown(int k) {
	int flags = cloud_volume.levels[k].w;
	return (flags & (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY)) == (CLOUD_VOLUME_ACTIVE | CLOUD_VOLUME_READY);
}

// The finest shown level holding the point camera + offset (both frames of a re-frame: each level read in its own
// frame's grid; T5 carry; cross-frame nesting is Task 9's).
CloudVolumeAt cloud_volume_at(vec3 offset) {
	CloudVolumeAt at;
	at.level = -1;
	at.fade = 0.0;
	vec3 g0 = cloud_volume_grid_in(0, offset);
	bool g1_done = false;
	vec3 g1 = g0;
	for (int k = 0; k < CLOUD_VOLUME_LEVELS; k++) {
		if (!cloud_volume_shown(k)) {
			continue;
		}
		int f = cloud_volume_frame_of(k);
		if (f == 1 && !g1_done) {
			g1 = cloud_volume_grid_in(1, offset);
			g1_done = true;
		}
		vec3 g = f == 0 ? g0 : g1;
		float w = cloud_volume_fade(k, k, g);
		if (w > 0.0) {
			at.level = k;
			at.fade = w;
			at.grid = g;
			at.grid_up = g;
			if (k + 1 < CLOUD_VOLUME_LEVELS && cloud_volume_frame_of(k + 1) != f) {
				at.grid_up = cloud_volume_frame_of(k + 1) == 0 ? g0 : cloud_volume_grid_in(1, offset);
			}
			return at;
		}
	}
	return at;
}

// (rho, sigma_c) [1/m] of the sample (both snapshots by cloud_blend, faded toward level + 1).
vec2 cloud_volume_rho(CloudVolumeAt at) {
	vec4 v = cloud_volume_fetch_density(at.level, at.level, at.grid);
	CLOUD_COUNT_FETCHES(1)
	if (at.fade < 1.0 && at.level + 1 < CLOUD_VOLUME_LEVELS) {
		v = mix(cloud_volume_fetch_density(at.level + 1, at.level + 1, at.grid_up), v, at.fade);
		CLOUD_COUNT_FETCHES(1)
	}
	return vec2(mix(v.x, v.z, cloud_blend), mix(v.y, v.w, cloud_blend));
}

// The light terms (tau_sun, tau_below, tau_above, tau_moon) of the sample, faded; moon: the levels' Moon weight (the
// march reads tau_moon only with it: rate-limited jobs leave it stale once the Moon is off).
vec4 cloud_volume_light_at(CloudVolumeAt at, out float moon) {
	vec4 v = cloud_volume_fetch_light(at.level, at.level, at.grid);
	CLOUD_COUNT_FETCHES(1)
	moon = cloud_volume.moon_weight[at.level / 4][at.level % 4];
	if (at.fade < 1.0 && at.level + 1 < CLOUD_VOLUME_LEVELS) {
		vec4 up = cloud_volume_fetch_light(at.level + 1, at.level + 1, at.grid_up);
		CLOUD_COUNT_FETCHES(1)
		v = mix(up, v, at.fade);
		moon = mix(cloud_volume.moon_weight[(at.level + 1) / 4][(at.level + 1) % 4], moon, at.fade);
	}
	return v;
}

// The sample at distance s along the ray from `at` (dg: grid metres per metre along the ray; offset_s: the point's
// offset from the camera): the same level, its grid moved linearly (within one cell: ~1 mm of curvature), the fade
// there, level + 1's grid its own frame's during a re-frame.
CloudVolumeAt cloud_volume_along(CloudVolumeAt at, vec3 dg, float s, vec3 offset_s) {
	CloudVolumeAt r = at;
	r.grid = at.grid + dg * s;
	r.fade = cloud_volume_fade(at.level, at.level, r.grid);
	r.grid_up = r.grid;
	int f_up = at.level + 1 < CLOUD_VOLUME_LEVELS ? cloud_volume_frame_of(at.level + 1) : 0;
	if (at.level + 1 < CLOUD_VOLUME_LEVELS && f_up != cloud_volume_frame_of(at.level)) {
		r.grid_up = cloud_volume_grid_in(f_up, offset_s);
	}
	return r;
}

// Level k's free ball (true metres) around the grid point WITHOUT the window clip (cloud_volume_level_free_m's box
// over the toroidal storage: free of every occupied block the storage holds - inside the window the truth, beyond it
// other cells, so only valid together with level k + 1 there); 0 in an occupied block.
float cloud_volume_unclipped_free_m(vec3 grid_m, int k) {
	vec3 cells = cloud_volume_cells(k, grid_m);
	ivec3 block = cloud_volume_block(k, grid_m);
	float d = cloud_volume_fetch_distance(k, cloud_volume_wrap(block, CLOUD_VOLUME_MIP_SIZE));
	if (d < 1.0) {
		return 0.0;
	}
	vec3 lo = (vec3(block) - (d - 1.0)) * float(CLOUD_VOLUME_MIP_DIV);
	vec3 hi = (vec3(block) + d) * float(CLOUD_VOLUME_MIP_DIV);
	vec3 m = max(min(cells - lo, hi - cells), vec3(0.0));
	return min(min(m.x, m.y) * CLOUD_VOLUME_CELL_H_M[k], m.z * CLOUD_VOLUME_CELL_V_M[k]) / CLOUD_VOLUME_MAX_SCALE;
}

// Free space (true metres) around the sample: level k's distance field, and level k + 1's where k's free ball reaches
// its fade band (cloud_volume_free_m with each level's own grid). There the ball is min(k's ball without the window
// clip, k + 1's): inside the window k's storage is the truth (in the band k + 1 is free too), beyond it k + 1 alone
// holds the density - so the ball crosses the window's edge in one step (the clipped ball only converged on the edge
// geometrically: ~100 steps of a ray grazing a level 0-1 window top).
float cloud_volume_free_at(CloudVolumeAt at) {
	vec3 lo;
	vec3 hi;
	float free_m = cloud_volume_level_free_m(at.grid, at.level, lo, hi);
	if (free_m > 0.0 && cloud_volume_reaches_band(at.level, lo, hi)) {
		vec3 lo1;
		vec3 hi1;
		free_m = min(cloud_volume_unclipped_free_m(at.grid, at.level),
				cloud_volume_level_free_m(at.grid_up, at.level + 1, lo1, hi1));
	}
	return free_m;
}

// Level k's free box (cells; the blocks closer than its distance field's value, as cloud_volume_level_free_m), clipped
// to the window when `clip`; false in an occupied block.
bool cloud_volume_free_box(vec3 grid_m, int k, bool clip, out vec3 lo, out vec3 hi) {
	ivec3 block = cloud_volume_block(k, grid_m);
	float d = cloud_volume_fetch_distance(k, cloud_volume_wrap(block, CLOUD_VOLUME_MIP_SIZE));
	lo = vec3(0.0);
	hi = vec3(0.0);
	if (d < 1.0) {
		return false;
	}
	lo = (vec3(block) - (d - 1.0)) * float(CLOUD_VOLUME_MIP_DIV);
	hi = (vec3(block) + d) * float(CLOUD_VOLUME_MIP_DIV);
	if (clip) {
		vec3 o = vec3(cloud_volume.levels[k].xyz);
		lo = max(lo, o);
		hi = min(hi, o + cloud_volume_window(k));
	}
	return true;
}

// The next distance > t at which the ray crosses the sphere of model height h_m (1e30: none).
float cloud_volume_next_crossing(CloudRay ray, float t, float h_m) {
	vec2 x = cloud_sphere_crossing(ray, h_m);
	if (x.x > x.y) {
		return 1.0e30;
	}
	return x.x > t ? x.x : (x.y > t ? x.y : 1.0e30);
}

// Distance (m) from t to where the ray leaves level k's cell box [lo, hi] (grid g at t, dg grid metres per metre):
// horizontally linear (the azimuthal map's scale: / CLOUD_VOLUME_MAX_SCALE), vertically the exact crossings of the
// box's heights (the ray's height curves with the Earth: 49 m over 25 km), less CLOUD_VOLUME_BOX_MARGIN of the height
// (the model height vs the height above WGS84: <= e^2 / 2) and a metre.
float cloud_volume_box_exit_m(CloudRay ray, float t, int k, vec3 g, vec3 dg, vec3 lo, vec3 hi) {
	vec3 c = cloud_volume_cells(k, g);
	float e = 1.0e30;
	for (int a = 0; a < 2; a++) {
		float dc = dg[a] / CLOUD_VOLUME_CELL_H_M[k];
		if (dc > 0.0) {
			e = min(e, (hi[a] - c[a]) / dc);
		} else if (dc < 0.0) {
			e = min(e, (lo[a] - c[a]) / dc);
		}
	}
	e /= CLOUD_VOLUME_MAX_SCALE;
	float h_lo = CLOUD_VOLUME_SHELL_BOTTOM_M + lo.z * CLOUD_VOLUME_CELL_V_M[k];
	float h_hi = CLOUD_VOLUME_SHELL_BOTTOM_M + hi.z * CLOUD_VOLUME_CELL_V_M[k];
	h_lo += CLOUD_VOLUME_BOX_MARGIN * abs(h_lo) + 1.0;
	h_hi -= CLOUD_VOLUME_BOX_MARGIN * abs(h_hi) + 1.0;
	float h = cloud_height_along(ray, t);
	if (h <= h_lo || h >= h_hi) {
		return 0.0;
	}
	e = min(e, min(cloud_volume_next_crossing(ray, t, h_lo), cloud_volume_next_crossing(ray, t, h_hi)) - t);
	return max(e, 0.0);
}

// The empty step (true metres) from the sample at t: the free ball (cloud_volume_free_at: safe in any direction) or,
// further, the exit of the free box along the ray - a ray grazing a cloud base or a window's top leaves the box
// through its far side (the ball alone converged on the base geometrically: hundreds of steps per horizon ray).
// 0 in an occupied block.
float cloud_volume_empty_step(CloudVolumeAt at, CloudRay ray, float t, vec3 offset, vec3 dir) {
	float ball = cloud_volume_free_at(at);
	if (ball <= 0.0) {
		return 0.0;
	}
	int k = at.level;
	float probe_m = CLOUD_VOLUME_CELL_H_M[k];
	vec3 dg = (cloud_volume_grid_in(cloud_volume_frame_of(k), offset + dir * probe_m) - at.grid) / probe_m;
	vec3 lo;
	vec3 hi;
	bool clip = true;
	float exit_m = 1.0e30;
	if (k + 1 < CLOUD_VOLUME_LEVELS) {
		cloud_volume_free_box(at.grid, k, true, lo, hi);
		if (cloud_volume_reaches_band(k, lo, hi)) {
			// beyond k's window (and in its band) level k + 1 holds the density: k's box unclipped and k + 1's
			clip = false;
			vec3 lo1;
			vec3 hi1;
			if (!cloud_volume_free_box(at.grid_up, k + 1, true, lo1, hi1)) {
				return ball;
			}
			vec3 dg1 = dg;
			if (cloud_volume_frame_of(k + 1) != cloud_volume_frame_of(k)) {
				dg1 = (cloud_volume_grid_in(cloud_volume_frame_of(k + 1), offset + dir * probe_m) - at.grid_up)
						/ probe_m;
			}
			exit_m = cloud_volume_box_exit_m(ray, t, k + 1, at.grid_up, dg1, lo1, hi1);
		}
	}
	cloud_volume_free_box(at.grid, k, clip, lo, hi);
	exit_m = min(exit_m, cloud_volume_box_exit_m(ray, t, k, at.grid, dg, lo, hi));
	return max(ball, exit_m);
}

// The standard normal CDF Phi (erf by Abramowitz & Stegun 7.1.26: |error| <= 1.5e-7) and its inverse (Acklam's
// rational approximation, relative error 1.15e-9 - float's own); core cloud_normal_cdf / cloud_normal_quantile.
float cloud_volume_normal_cdf(float x) {
	float z = abs(x) * 0.70710678;
	float k = 1.0 / (1.0 + 0.3275911 * z);
	float erf_z = 1.0 - (((((1.061405429 * k - 1.453152027) * k + 1.421413741) * k - 0.284496736) * k + 0.254829592)
			* k) * exp(-z * z);
	return 0.5 + 0.5 * sign(x) * erf_z;
}

float cloud_volume_normal_quantile(float p) {
	p = clamp(p, 1.0e-7, 1.0 - 1.0e-7);
	if (p < 0.02425 || p > 0.97575) {
		float q = sqrt(-2.0 * log(min(p, 1.0 - p)));
		float x = (((((-7.784894002430293e-03 * q - 3.223964580411365e-01) * q - 2.400758277161838e+00) * q
				- 2.549732539343734e+00) * q + 4.374664141464968e+00) * q + 2.938163982698783e+00)
				/ ((((7.784695709041462e-03 * q + 3.224671290700398e-01) * q + 2.445134137142996e+00) * q
				+ 3.754408661907416e+00) * q + 1.0);
		return p < 0.5 ? x : -x;
	}
	float q = p - 0.5;
	float r = q * q;
	return (((((-3.969683028665376e+01 * r + 2.209460984245205e+02) * r - 2.759285104469687e+02) * r
			+ 1.383577518672690e+02) * r - 3.066479806614716e+01) * r + 2.506628277459239e+00) * q
			/ (((((-5.447609879822406e+01 * r + 1.615858368580409e+02) * r - 1.556989798598866e+02) * r
			+ 6.680131188771972e+01) * r - 1.328068155288572e+01) * r + 1.0);
}

// The shape residual's variance R(l) at shape level l (core shape_residual_variance: of s_0 - s_l; linear in l).
float cloud_volume_residual_variance(float level) {
	float l = clamp(level, 0.0, 7.0);
	int i = min(int(l), 6);
	float lo = i < 4 ? cloud_shape_residual_variance_lo[i] : cloud_shape_residual_variance_hi[i - 4];
	float hi = i + 1 < 4 ? cloud_shape_residual_variance_lo[i + 1] : cloud_shape_residual_variance_hi[i - 3];
	return mix(lo, hi, l - float(i));
}

// The sub-cell field's cell scale at the sample (ruling R5: continuous through a fade band - the levels' cell sizes
// mixed geometrically by the fade, as their densities are mixed - so the shape residual grows with the cell size instead
// of switching on with a level): x the cell's horizontal size [m], y its shape level (0 below the shape's own texel), z
// the residual's weight w_s at the footprint (header; 0 where the cell's shape level is not above the footprint's).
vec3 cloud_volume_cell_scale(CloudVolumeAt at, float footprint_m) {
	float cell_m = CLOUD_VOLUME_CELL_H_M[at.level];
	if (at.fade < 1.0 && at.level + 1 < CLOUD_VOLUME_LEVELS) {
		cell_m *= exp2(1.0 - at.fade);  // (each level's cell twice the finer one's)
	}
	float lod = clamp(log2(cell_m * CLOUD_SHAPE_TEXELS / CLOUD_SHAPE_TILE_H_M), 0.0, CLOUD_VOLUME_SHAPE_MAX_LOD);
	float w_s = lod > cloud_shape_prefilter_level(footprint_m)
			? clamp(log2(cell_m / max(footprint_m, 1.0e-3)) - 1.0, 0.0, 1.0) : 0.0;
	return vec3(cell_m, lod, w_s);
}

// The per-sample density [1/m] at camera + offset (height_m its height; header): the bake (rho), with sub_cell the
// sub-cell threshold field at the footprint (ruling R1, core cloud_noise.hpp "Sub-cell field"):
//   density = sigma_c Phi((v - t) / s_e),  t = -sqrt(s_e^2 + sigma_v^2) Phi^-1(c),  c = rho / sigma_c,
// v = w_s (s_f - s_cell) + P_g (the shape's residual above the cell's scale; the detail perturbation Gaussianised by
// its own distribution: sigma_P G((P - mu_P) / sigma_P)), sigma_v^2 = w_s^2 (R(l_cell) - R(l_f)) + sigma_P^2, s_e the
// edge's spread (CLOUD_SUB_CELL_EDGE_SPREAD x the edge width of the shape's width noise). The cell's mean stays rho
// (the core's test: within 1e-3 of c for a Gaussian v; 2.4e-3 / 3.6e-3 with the real residual), graded where sigma_v
// -> 0 (the bake: no aliasing where the footprint reaches the cell), a sharp cover c where sigma_v >> s_e, 0 where
// rho = 0 and <= sigma_c (the max mip's bound and the distance field hold). edge: the field acts here (partial cover,
// a weight > 0) - the structure term caps the steps there.
// (rs: the sample's (rho, sigma_c), cloud_volume_rho - the march reads it first to decide a re-take without the rest)
float cloud_volume_field(CloudVolumeAt at, vec2 rs, vec3 offset, float height_m, float footprint_m, bool sub_cell,
		out bool edge) {
	edge = false;
	float rho = rs.x;
	if (!sub_cell || rho <= 0.0) {
		return max(rho, 0.0);
	}
	float c = clamp(rho / max(rs.y, 1.0e-12), 0.0, 1.0);
	if (c >= CLOUD_VOLUME_SOLID_COVER) {
		return rho;
	}
	vec3 scale = cloud_volume_cell_scale(at, footprint_m);
	float w_s = scale.z;
	float state = cloud_detail_state(footprint_m);
	if (w_s <= 0.0 && state <= 0.0) {
		return rho;
	}
	edge = true;
	// the shape at the footprint (its width noise; the residual's fine part)
	float lod_f = cloud_shape_prefilter_level(footprint_m);
	vec3 q = cloud_shape_coordinates(offset, height_m);
	// (one fetch where the footprint reads the unfiltered shape: the prefilter's level-0 blend reads two textures)
	vec4 s_f = lod_f > 0.0 ? cloud_shape_prefiltered(q, lod_f) : texture(cloud_noise_shape, q);
	CLOUD_COUNT_FETCHES(lod_f > 0.0 && lod_f < 1.0 ? 2 : 1)
	float width = cloud_edge_width.x * pow(cloud_edge_width.y / cloud_edge_width.x, clamp(s_f.a, 0.0, 1.0));
	float v = 0.0;
	float variance = 0.0;
	if (w_s > 0.0) {
		// (below the first mip the prefilter's blend of the shape and its first mip: the shape is s_f where the
		// footprint reads it unfiltered - one fetch)
		float s_cell = scale.y < 1.0 && lod_f <= 0.0
				? mix(s_f.r, textureLod(cloud_noise_shape_mips, q, 0.0).r, scale.y) : cloud_shape_prefiltered(q, scale.y).r;
		CLOUD_COUNT_FETCHES(scale.y < 1.0 && lod_f > 0.0 ? 2 : 1)
		v += w_s * (s_f.r - s_cell);
		variance += w_s * w_s
				* max(cloud_volume_residual_variance(scale.y) - cloud_volume_residual_variance(lod_f), 0.0);
	}
	if (state > 0.0) {
		vec3 moved = offset;
		if (cloud_edge_curl > 0.0) {
			vec3 qc = fract(cloud_detail_origin) + offset / CLOUD_VOLUME_CURL_TILE_M;
			vec3 curl = texture(cloud_noise_curl, qc).rgb * 2.0 - 1.0;
			CLOUD_COUNT_FETCHES(1)
			moved += curl * (cloud_edge_curl * clamp(state, 0.0, 1.0) * CLOUD_VOLUME_CURL_UNIT_M);
		}
		vec3 w = clamp(vec3(state) - vec3(0.0, 1.0, 2.0), 0.0, 1.0);
		float mu = dot(w, cloud_detail_level_mean);
		float var_p = dot(w * w, cloud_detail_level_variance);
		float sigma_p = sqrt(max(var_p, 1.0e-20));
		float x = (cloud_detail_perturbation(moved, state) - mu) / sigma_p;
		vec2 uv = vec2((0.5 + 0.5 * x / CLOUD_SUB_CELL_STANDARD_RANGE) * (255.0 / 256.0) + 0.5 / 256.0,
				(state * CLOUD_SUB_CELL_ROWS_PER_STATE + 0.5) / CLOUD_SUB_CELL_ROWS);
		v += sigma_p * texture(cloud_detail_gaussian, uv).r;
		CLOUD_COUNT_FETCHES(1)
		variance += var_p;
	}
	float spread = CLOUD_SUB_CELL_EDGE_SPREAD * max(width, 1.0e-6);
	float t = -sqrt(spread * spread + variance) * cloud_volume_normal_quantile(c);
	return rs.y * cloud_volume_normal_cdf((v - t) / spread);
}

float cloud_volume_density_at(CloudVolumeAt at, vec3 offset, float height_m, float footprint_m, bool sub_cell,
		out bool edge) {
	return cloud_volume_field(at, cloud_volume_rho(at), offset, height_m, footprint_m, sub_cell, edge);
}


// The finest structure (times CLOUD_STRUCTURE_TEXEL_FRACTION) the sub-cell field resolves at the footprint along a ray
// climbing at climb: the shape's texel crossing where its residual is on, the detail levels' middle cells where their
// weight is > 0 (1e30: none - the step is the cell's).
float cloud_volume_structure_m(CloudVolumeAt at, float footprint_m, float climb) {
	float finest = 1.0e30;
	if (cloud_volume_cell_scale(at, footprint_m).z > 0.0) {
		float across = sqrt(max(1.0 - climb * climb, 0.0));
		finest = 1.0 / max(climb / (CLOUD_SHAPE_TILE_V_M / CLOUD_SHAPE_TEXELS),
				across / (CLOUD_SHAPE_TILE_H_M / CLOUD_SHAPE_TEXELS));
	}
	float middle_cell_m = CLOUD_DETAIL_TILE_M / 4.0;
	for (int j = 0; j < CLOUD_DETAIL_LEVELS; j++) {
		if (j >= cloud_detail_levels) {
			break;
		}
		if (middle_cell_m > 2.0 * footprint_m) {
			finest = min(finest, middle_cell_m);
		}
		middle_cell_m *= 0.25;
	}
	return finest < 1.0e29 ? CLOUD_VOLUME_STRUCTURE_FRACTION * finest : 1.0e30;
}

// Distance (m) along the ray from continuous cells c (dc: cells per metre) to the exit of the cell box [lo, hi).
float cloud_volume_box_exit(vec3 c, vec3 dc, vec3 lo, vec3 hi) {
	float exit_m = 1.0e30;
	for (int a = 0; a < 3; a++) {
		if (dc[a] > 0.0) {
			exit_m = min(exit_m, (hi[a] - c[a]) / dc[a]);
		} else if (dc[a] < 0.0) {
			exit_m = min(exit_m, (lo[a] - c[a]) / dc[a]);
		}
	}
	return max(exit_m, 0.0);
}

// The step's share e^-tau' terms of the linear-source segment integral: (A, B) at tau' (header).
vec2 cloud_volume_segment(float tau) {
	if (tau < CLOUD_VOLUME_SERIES_TAU) {
		return vec2(tau * (1.0 - 0.5 * tau), tau * (0.5 - tau / 3.0));
	}
	float e = exp(-tau);
	float a = 1.0 - e;
	return vec2(a, a / tau - e);
}

// A sample's sources at camera + offset (dir: the view ray): x the lights' (Sun, Moon: J x irradiance; the debug
// outputs 2 / 7: the Sun's J alone), y the sky's (scaled problem). Surface correction with step_weight (header).
struct CloudVolumeSource {
	vec3 light;
	vec3 sky;
};

// The air's light at the cloud, refreshed along the ray (cloud_march's CLOUD_LIGHT_REFRESH rule).
struct CloudVolumeAir {
	float at_m;
	vec3 sun;
	vec3 sky;
	vec3 moon;
	vec3 moon_sky;
};

void cloud_volume_air_refresh(inout CloudVolumeAir air, vec3 p, vec3 normal, float d, vec3 eclipse_near,
		vec3 eclipse_far, bool moonlit) {
	if (abs(d - air.at_m) <= max(CLOUD_LIGHT_REFRESH_M, CLOUD_LIGHT_REFRESH_FRACTION * d)) {
		return;
	}
	air.at_m = d;
#ifdef CLOUD_ABLATE_AIR
	air.sun = vec3(1.0);  // (ablation: constant irradiances, none of the atmosphere tables' fetches)
	air.sky = vec3(0.2);
#else
	air.sun = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
	float eclipse_x = clamp((d - CLOUD_ECLIPSE_NEAR_M) / (CLOUD_ECLIPSE_FAR_M - CLOUD_ECLIPSE_NEAR_M), 0.0, 1.0);
	air.sky = ATM_PI * atm_sky_light(p, normal) * mix(eclipse_near, eclipse_far, eclipse_x);
	if (moonlit) {
		air.moon = ATM_PI * atm_moon_at(p);
		air.moon_sky = ATM_PI * atm_moon_sky_light(p, normal);
	}
#endif
}

// tau_sun through the per-sample density over the first CLOUD_VOLUME_ENTRY_CELLS cells of the level toward the Sun
// (cloud_light_entry_samples midpoints) plus the volume's from there (spec C surface correction).
// tau_sun at the sample through the per-sample density (one call site of the density: the kernel's size):
// converged (checks: cloud_light_entry_samples < 0, header) or the surface correction - cloud_light_entry_samples
// midpoints over the level's first CLOUD_VOLUME_ENTRY_CELLS cells toward the Sun (on the level's grid moved linearly:
// over two cells the shells' curvature is ~mm), plus the volume's tau_sun from there.
float cloud_volume_sun_tau(CloudVolumeAt at, vec3 offset, float height_m, vec3 sun_dir, vec3 normal,
		float footprint_m, bool converged) {
	int k = at.level;
	float mu = dot(sun_dir, normal);
	float tau = 0.0;
	int n = converged ? CLOUD_VOLUME_CONVERGED_STEPS : cloud_light_entry_samples;
	float piece = 0.0;
	vec3 dg = vec3(0.0);
	if (!converged) {
		// (the cells of the coarsest level whose light the sample reads: in the fade band level k + 1's)
		int kl = at.fade < 1.0 && k + 1 < CLOUD_VOLUME_LEVELS ? k + 1 : k;
		float across = CLOUD_VOLUME_CELL_H_M[kl] / max(sqrt(max(1.0 - mu * mu, 0.0)), 1.0e-6);
		float up = CLOUD_VOLUME_CELL_V_M[kl] / max(abs(mu), 1.0e-6);
		float span_m = CLOUD_VOLUME_ENTRY_CELLS * min(across, up);
		dg = (cloud_volume_grid_in(cloud_volume_frame_of(k), offset + sun_dir * span_m) - at.grid) / span_m;
		float moon_unused;
		tau = cloud_volume_light_at(cloud_volume_along(at, dg, span_m, offset + sun_dir * span_m), moon_unused).x;
		piece = span_m / float(n);
	}
	float s = 0.0;
	for (int j = 0; j < n; j++) {
		float step_m = converged ? max(CLOUD_VOLUME_CONVERGED_FIRST_M, CLOUD_VOLUME_CONVERGED_GROWTH * s) : piece;
		float m = s + 0.5 * step_m;
		float h = height_m + mu * m;  // (the curvature over the few km of a check's march: metres)
		vec3 o = offset + sun_dir * m;
		CloudVolumeAt sample_at = converged ? cloud_volume_at(o) : cloud_volume_along(at, dg, m, o);
		if (converged && (h > CLOUD_TOP_M || h < CLOUD_BOTTOM_M || sample_at.level < 0)) {
			break;
		}
		bool edge;
		tau += step_m * cloud_volume_density_at(sample_at, o, h, footprint_m, cloud_volume_sub_cell, edge);
		s += step_m;
	}
	return tau;
}

CloudVolumeSource cloud_volume_source(CloudVolumeAt at, vec3 offset, float height_m, vec3 dir, float d,
		float footprint_m, float step_weight, bool sun_only, inout CloudVolumeAir air, vec3 eclipse_near,
		vec3 eclipse_far) {
	CLOUD_LIGHT_BEGIN
	CloudVolumeSource src;
	src.light = vec3(0.0);
	src.sky = vec3(0.0);
#ifdef CLOUD_ABLATE_LIGHT
	src.light = vec3(1.0);  // (ablation: a constant source, none of the light's fetches)
	CLOUD_LIGHT_END
	return src;
#endif
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	vec3 p = camera + offset;
	vec3 normal = normalize(p * ATM_STRETCH * ATM_STRETCH);  // the ellipsoid's normal (geodetic up)
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	float moon_weight;
	vec4 tau = cloud_volume_light_at(at, moon_weight);
	vec2 column = vec2(tau.z, tau.z + tau.y);  // depth from the top, the column (the table's a and a + b)
	float tau_sun = tau.x;
	bool converged = cloud_light_entry_samples < 0;
	if (tau_sun < CLOUD_VOLUME_NO_LIGHT_TAU && (converged || (cloud_light_entry_samples > 0
			&& step_weight > CLOUD_VOLUME_SURFACE_WEIGHT && CLOUD_VOLUME_CELL_H_M[at.level] > footprint_m))) {
		tau_sun = cloud_volume_sun_tau(at, offset, height_m, sun_dir, normal, footprint_m, converged);
	}
	bool moonlit = atm_moonlit();
	if (!sun_only) {
		cloud_volume_air_refresh(air, p, normal, d, eclipse_near, eclipse_far, moonlit);
		float sky = cloud_sky_scattering_source(-dot(dir, normal), column.x, column.y);
		src.sky = air.sky * sky + (moonlit ? air.moon_sky * sky : vec3(0.0));
	}
	// The lights (Sun, then the Moon while it lights the level), one call site of the source (the kernel's size).
	int lights = !sun_only && moonlit && moon_weight > 0.0 && tau.w < CLOUD_VOLUME_NO_LIGHT_TAU ? 2 : 1;
	for (int l = 0; l < lights; l++) {
		vec3 light_dir = l == 0 ? sun_dir : normalize(atmosphere_moon_direction);
		float tau_l = l == 0 ? tau_sun : tau.w;
		// (no light reaches a sample the volume marks unlit - Earth's shadow, the Moon off; also not the table's
		// diffuse part, whose 3D entry cosine can be > 0 below the horizon)
		float j = tau_l >= CLOUD_VOLUME_UNLIT_TAU ? 0.0 : cloud_scattering_model == 1
				? cloud_scattering_source_mc(light_dir, -dir, normal, tau_l, column.x, column.y)
				: cloud_scattering_source_3d(light_dir, -dir, normal, tau_l, column.x, column.y);
		src.light += sun_only ? vec3(j) : (l == 0 ? air.sun : moon_weight * air.moon) * j;
	}
	CLOUD_LIGHT_END
	return src;
}

// The clouds of one ray (cloud_volume_march without the air shadow's loss; header).
void cloud_volume_march_clouds(vec3 dir_earth, vec3 origin_offset, float from_m, float to_m, float pixel_rad,
		vec3 eclipse_near, vec3 eclipse_far, out vec3 color, out float transmittance, out float cloud_m) {
	color = vec3(0.0);
	transmittance = 1.0;
	cloud_m = 0.0;
	vec3 dir = dir_earth;
	vec3 eye = origin_offset;
	CloudRay ray = cloud_ray_from(dir, eye);
	vec2 top = cloud_sphere_crossing(ray, CLOUD_TOP_M);
	if (top.x > top.y || top.y <= 0.0) {
		return;
	}
	float d0 = max(top.x, 0.0);
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
	d0 = max(d0, from_m);
	d1 = min(d1, to_m);
	if (d1 <= d0) {
		return;
	}
	int debug = cloud_debug_output;
	bool sub_cell = debug != 1 && debug != 2;
	cloud_volume_sub_cell = sub_cell;
	bool sun_only = debug == 2 || debug == 7;
	bool coverage = debug == 1 || debug == 4;
	float k_scale = cloud_transport_scale();
	float w_tab = cloud_phase.w;
	bool table = cloud_scattering_model == 1;
	float light_scale = table ? w_tab : k_scale;
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	bool sun_ray = dot(dir, sun_dir) > cos(CLOUD_SUN_RAY_ANGLE);
	CloudVolumeAir air;
	air.at_m = -1.0e30;
	air.sun = vec3(0.0);
	air.sky = vec3(0.0);
	air.moon = vec3(0.0);
	air.moon_sky = vec3(0.0);
	float transmittance_bg = 1.0;
	float transmittance_sc = 1.0;   // exp(-k tau): the sky's scattered light (and the lights' without the table)
	float transmittance_tab = 1.0;  // exp(-w_D tau): the lights' with the table
	vec3 radiance = vec3(0.0);
	float weighted = 0.0;
	bool outside = true;
	// the last sample lay where the sub-cell field acts (partial cover; its density may be 0 there): the structure
	// term caps the next step; after a clear, solid or bake-only sample the step is the cell's (an entry is found by
	// the bisection)
	bool in_edge = false;
	// the source at the current step's start (valid: the last occupied step ended here with its end source)
	bool start_valid = false;
	CloudVolumeSource start_src;
	start_src.light = vec3(0.0);
	start_src.sky = vec3(0.0);
	// the light nodes (the source evaluated at node_a_t and node_b_t, linear between; valid until the next empty step)
	bool node_valid = false;
	float node_a_t = 0.0;
	float node_b_t = 0.0;
	CloudVolumeSource node_a = start_src;
	CloudVolumeSource node_b = start_src;
	float t = d0;
	for (int i = 0; i < CLOUD_VOLUME_MAX_STEPS && t < d1; i++) {
		vec3 offset = eye + dir * t;
		CloudVolumeAt at = cloud_volume_at(offset);
		if (at.level < 0) {
			break;  // beyond the coarsest shown level (Task 8 / 10: the far field takes it)
		}
		float free_m = cloud_volume_empty_step(at, ray, t, offset, dir);
		if (free_m > 0.0) {
			CLOUD_COUNT_STEP(false)
			t += max(free_m, CLOUD_VOLUME_MIN_STEP_M);
			start_valid = false;
			node_valid = false;
			outside = true;
			in_edge = true;  // (the first occupied step is the structure's: no cell-sized sample to re-take, ruling R3)
			continue;
		}
		// Occupied space: to the exit of the level's cell (cells per metre along the ray: a finite difference over
		// the cell's shorter side - the shells' curvature over a cell is ~1 mm), capped by the structure term.
		int k = at.level;
		float probe_m = min(CLOUD_VOLUME_CELL_H_M[k], CLOUD_VOLUME_CELL_V_M[k]);
		vec3 g_probe = cloud_volume_grid_in(cloud_volume_frame_of(k), offset + dir * probe_m);
		vec3 dg = (g_probe - at.grid) / probe_m;
		vec3 c = cloud_volume_cells(k, at.grid);
		vec3 dc = cloud_volume_cells(k, at.grid + dg) - c;
		vec3 cell = floor(c + sign(dc) * CLOUD_VOLUME_EXIT_NUDGE_CELLS);  // the cell the ray is in (leaving)
		float exit_m = cloud_volume_box_exit(c, dc, cell, cell + 1.0);
		float footprint_m = pixel_rad * t;
		float climb = abs(cloud_height_slope(ray, t));
		float structure_m = sub_cell && in_edge ? cloud_volume_structure_m(at, footprint_m, climb) : 1.0e30;
		float step_m = min(max(min(exit_m, structure_m), CLOUD_VOLUME_MIN_STEP_M), d1 - t);
		float d = t + 0.5 * step_m;
		CloudVolumeAt mid = cloud_volume_along(at, dg, 0.5 * step_m, eye + dir * d);
		float height = cloud_height_along(ray, d);
		// The step's sample, and where it enters cloud from outside the silhouette between the step's start and its
		// middle, bisected (one call site of the density: the kernel's size).
		bool edge = false;
		float beta = 0.0;
		bool bisect = false;
		float lo = t;
		float hi = d;
		for (int b = -1; b < CLOUD_VOLUME_BISECTIONS; b++) {
			float m = b < 0 ? d : 0.5 * (lo + hi);
			CloudVolumeAt am = b < 0 ? mid : cloud_volume_along(at, dg, m - t, eye + dir * m);
			bool s_m;
			float beta_m = cloud_volume_density_at(am, eye + dir * m, b < 0 ? height : cloud_height_along(ray, m),
					pixel_rad * m, sub_cell, s_m);
			if (b < 0) {
				beta = beta_m;
				edge = s_m;
				bisect = outside && beta >= CLOUD_SILHOUETTE_EXTINCTION_PER_M && step_m > 2.0 * CLOUD_VOLUME_MIN_STEP_M;
				if (!bisect) {
					break;
				}
			} else if (beta_m >= CLOUD_SILHOUETTE_EXTINCTION_PER_M) {
				hi = m;
			} else {
				lo = m;
			}
		}
		CLOUD_COUNT_STEP(true)
		if (bisect) {
			outside = false;
			if (lo - t >= CLOUD_VOLUME_MIN_STEP_M) {
				t = lo;
				start_valid = false;
				in_edge = true;  // the steps from the entry resolve the edge
				continue;
			}
		}
		if (edge && !in_edge && sub_cell && step_m > cloud_volume_structure_m(at, footprint_m, climb)) {
			// The sub-cell field acts in this cell but the step was the cell's (its start was clear or solid): taken
			// again from its start with the structure term (one sample of a sharp field per cell drew the cells' faces
			// as blocks).
			in_edge = true;
			continue;
		}
		outside = beta < CLOUD_SILHOUETTE_EXTINCTION_PER_M;
		in_edge = edge;
		float t_end = t + step_m;
		if (beta <= 0.0) {
			t = t_end;
			start_valid = false;
			continue;
		}
		float tau = beta * step_m;
		float true_step = exp(-tau);
		float scaled_step = exp(-k_scale * tau);
		if (!coverage && transmittance_sc >= CLOUD_MIN_SCALED_TRANSMITTANCE) {
			// The source at both ends (the start's shared with the step before), linear between them.
			float weight = transmittance_bg * (1.0 - true_step);
			// The source at the step's ends from the light nodes (header; ruling R3): linear between two evaluations
			// CLOUD_VOLUME_LIGHT_NODE_CELLS of the level's cell apart - the light volume is trilinear on the cell's
			// scale, the steps through the sub-cell field are far shorter. One call site of the source (the kernel's
			// size).
			float spacing_m = CLOUD_VOLUME_LIGHT_NODE_CELLS / max(max(abs(dc.x), abs(dc.y)), max(abs(dc.z), 1.0e-9));
			CloudVolumeSource end_src = start_src;
			for (int e = start_valid ? 1 : 0; e < 2; e++) {
				float t_e = e == 0 ? t : t_end;
				if (!node_valid || t_e < node_a_t || t_e > node_b_t) {
					bool advance = node_valid && t_e > node_b_t;
					float t_n = advance ? max(node_b_t + spacing_m, t_e) : t_e;
					CloudVolumeSource s_n = cloud_volume_source(cloud_volume_along(at, dg, t_n - t, eye + dir * t_n),
							eye + dir * t_n, cloud_height_along(ray, t_n), dir, t_n, pixel_rad * t_n, weight, sun_only,
							air, eclipse_near, eclipse_far);
					if (advance) {
						node_a = node_b;
						node_a_t = node_b_t;
					} else {
						node_a = s_n;
						node_a_t = t_n;
					}
					node_b = s_n;
					node_b_t = t_n;
					node_valid = true;
				}
				float f = node_b_t > node_a_t ? clamp((t_e - node_a_t) / (node_b_t - node_a_t), 0.0, 1.0) : 0.0;
				end_src.light = mix(node_a.light, node_b.light, f);
				end_src.sky = mix(node_a.sky, node_b.sky, f);
				if (e == 0) {
					start_src = end_src;
				}
			}
			vec2 seg_sky = cloud_volume_segment(k_scale * tau);
			vec2 seg_light = cloud_volume_segment(light_scale * tau);
			vec3 sky = (start_src.sky * seg_sky.x + (end_src.sky - start_src.sky) * seg_sky.y) / k_scale;
			vec3 light = (start_src.light * seg_light.x + (end_src.light - start_src.light) * seg_light.y)
					/ light_scale;
			radiance += transmittance_sc * sky + (table ? transmittance_tab : transmittance_sc) * light;
			start_src = end_src;
			start_valid = true;
		} else {
			start_valid = false;
		}
		transmittance_sc *= scaled_step;
		transmittance_tab *= exp(-w_tab * tau);
		float next_bg = transmittance_bg * true_step;
		weighted += d * (transmittance_bg - next_bg);
		transmittance_bg = next_bg;
		t = t_end;
		if ((transmittance_sc < CLOUD_MIN_SCALED_TRANSMITTANCE
				&& !(sun_ray && transmittance_bg > CLOUD_SUN_MIN_TRANSMITTANCE))
				|| (coverage && !sun_ray && transmittance_bg < CLOUD_SUN_MIN_TRANSMITTANCE)) {
			break;
		}
	}
	float alpha = 1.0 - transmittance_bg;
	cloud_m = alpha > 1.0e-6 ? weighted / alpha : 0.0;
	transmittance = transmittance_bg;
	if (coverage) {
		color = vec3(transmittance_bg);
		transmittance = 0.0;
		return;
	}
	if (sun_only) {
		color = radiance;
		transmittance = 0.0;
		return;
	}
	if (debug == 12) {
		color = vec3(cloud_m, alpha, 0.0);
		transmittance = 0.0;
		return;
	}
	if (alpha <= 1.0e-6 && dot(radiance, radiance) <= 0.0) {
		transmittance = 1.0;
		cloud_m = 0.0;
		return;
	}
	vec3 air_transmittance = vec3(1.0);
	vec3 air_light = vec3(0.0);
#ifndef CLOUD_ABLATE_AIR  // (ablation: no air in front of the clouds)
	if (alpha > 1.0e-6) {
		air_light = atm_to_point_offset(eye + dir * cloud_m, air_transmittance);
	}
#endif
	color = air_transmittance * radiance + alpha * air_light;
}

// One ray (header): dir_earth (Earth-local unit) from origin_offset (the eye's offset from the camera) over [from_m,
// to_m] (to_m: the scene's distance; the cloud shell clips it), pixel_rad the pixel's angle (footprint = pixel_rad x
// distance). air_segments: the air shadow's segments (Task 10 sets them per tier; today's CLOUD_AIR_SHADOW_SEGMENTS).
// Out: the colour (premultiplied: the clouds' light seen through the air in front of them, plus the air in front at
// the clouds' alpha, less the air shadow's loss), the background's transmittance T and the transmittance-weighted
// cloud distance (alpha-weighted, as cloud_march's; 0 without cloud). The air shadow's loss is composited on EVERY ray
// (debug output 0), as cloud_shade_ray does: a clear ray darkens where clouds elsewhere shade the air along it (R6: the
// loss skipped on rays without cloud was V6_backlit's error in the clear sky - median 34 / 20 %).
void cloud_volume_march(vec3 dir_earth, vec3 origin_offset, float from_m, float to_m, float pixel_rad,
		vec3 eclipse_near, vec3 eclipse_far, int air_segments, out vec3 color, out float transmittance,
		out float cloud_m) {
	cloud_volume_march_clouds(dir_earth, origin_offset, from_m, to_m, pixel_rad, eclipse_near, eclipse_far, color,
			transmittance, cloud_m);
#ifndef CLOUD_ABLATE_AIR_SHADOW
	// The air shadow's loss as cloud_shade_ray composites it (air_segments: Task 10; today's constant count).
	if (cloud_debug_output == 0) {
		color -= cloud_air_shadow_loss(cloud_ray_from(dir_earth, origin_offset), dir_earth, origin_offset, to_m,
				1.0 - transmittance > 1.0e-6 ? cloud_m : 1.0e30, transmittance);
	}
#endif
}

#ifdef CLOUD_VOLUME_FULL
// The kernel (header): one ray per eye pixel, written into the eye's layer of the result (premultiplied colour + T).
layout(set = 1, binding = 1) uniform sampler2D scene_depth;  // the eye's, full resolution, nearest
layout(rgba16f, set = 1, binding = 2) uniform writeonly image2D out_color;
#ifdef CLOUD_COUNTERS
layout(rgba32ui, set = 1, binding = 24) uniform writeonly uimage2D out_counters;
#endif

void main() {
	ivec2 pixel = ivec2(gl_GlobalInvocationID.xy);
	ivec2 size = ivec2(cloud_view.sizes.xy);
	if (gl_LocalInvocationIndex == 0u) {
		vec2 group_centre = vec2(gl_WorkGroupID.xy * gl_WorkGroupSize.xy) + 0.5 * vec2(gl_WorkGroupSize.xy);
		vec3 group_dir = cloud_ray_earth(min(group_centre, cloud_view.sizes.xy - 0.5));
		cloud_eclipse_factors(group_dir, cloud_view.eye_earth.xyz, cloud_group_eclipse_near, cloud_group_eclipse_far);
	}
	barrier();
	if (pixel.x >= size.x || pixel.y >= size.y) {
		return;
	}
	vec2 centre = vec2(pixel) + 0.5;
	float scene_m = cloud_scene_distance(centre, texelFetch(scene_depth, pixel, 0).r);
	vec3 dir = cloud_ray_earth(centre);
	vec3 color = vec3(0.0);
	float transmittance = 1.0;
	float cloud_m = 0.0;
	if (cloud_enabled && atmosphere_enabled) {
		cloud_volume_march(dir, cloud_view.eye_earth.xyz, 0.0, scene_m, cloud_view.params.w, cloud_group_eclipse_near,
				cloud_group_eclipse_far, CLOUD_AIR_SHADOW_SEGMENTS, color, transmittance, cloud_m);
	}
	imageStore(out_color, pixel, cloud_stored_color(color, transmittance));
#ifdef CLOUD_COUNTERS
	imageStore(out_counters, pixel,
			uvec4(cloud_count_density, cloud_count_light, cloud_count_occupied, cloud_count_empty));
#endif
}
#endif
