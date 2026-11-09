// The far field of the clouds (M8 weather and clouds, Task 10): the cloud layer of the scaled-space Earth
// (earth_clouds.gdshader, drawn while the camera is beyond CelestialBody.SCALED_SPACE_DISTANCE_M, where the cloud pass
// is off) and, since static clouds Task 8, the column part of the cloud pass's far tier at large footprints
// (cloud_pass.gdshaderinc cloud_far_column: the same slab, seen from above or from below, with a sub-grid cover).
// Include atmosphere.gdshaderinc and clouds.gdshaderinc first.
//
// The same field and the same light transport as the cloud pass, in its plane-parallel limit: a pixel there covers
// >= 7 km (the shapes' detail levels have faded to their mean; the shape noise itself is resolved by a few pixels at
// most), so the column replaces the march:
// - The column above the ground point G (the column texels' B-spline, cloud_column_mean_depth_smooth): the grid-mean
//   optical depth tau and the largest cloud fraction f. The view ray's cover is the beam cover of f at the view's
//   cosine mu (cloud_beam_cover_of: the rule fitted to the shapes' shadows, for any line through the layer), C_v;
//   independent columns: clear on 1 - C_v, the in-cloud depth tau / C_v on C_v.
// - The cloud's position: where the view ray crosses the height at which the column's depth from its top reaches the
//   transmittance-weighted mean depth of the visible light (the cloud pass's weighted cloud distance for a slab); the
//   column is taken again there (the parallax of high cloud near the limb), the air's light at that point.
// - The radiance of the cloudy part: a conservative plane-parallel slab of that depth, lit by the Sun (and at night the
//   Moon) transmitted to the cloud, through the Monte Carlo transfer table - the cloud pass's source for a slab
//   (cloud_scattering_source_mc with mu_e = mu0 exactly) integrated along the view ray with its attenuation
//   exp(-w_D tau_v) (single scattering in closed form, the table by 8-point Gauss-Legendre in u = 1 - exp(-w_D tau_v /
//   mu)); the closed form likewise when the fidelity flag says so; plus the sky's source (cloud_sky_scattering_source,
//   linear in depth: integrated in closed form) lit by the sky irradiance at the cloud.
// - Composite as the cloud pass: alpha = C_v (1 - exp(-tau / (C_v mu))), colour = T_air L + alpha S_air over
//   (1 - alpha) x the ground; the ground under the clouds lit through the column (cloud_light.gdshaderinc,
//   cloud_ground_light's column tier: the terrain's light beyond 140 km from the camera).
// Approximations (documented in docs/subsystems/weather-clouds.md): no light between columns (no lit cloud sides, no
// shadows cast on neighbouring columns), the cloud lit at one point, a Sun below the cloud's local horizon at the
// entry floor CLOUD_ENTRY_COSINE_MIN.

// [E] The smallest view cosine taken (a grazing view at the limb: the in-cloud path of a plane-parallel slab grows as
// 1 / mu without limit; the beam cover's own floor).
const float CLOUD_FAR_MU_MIN = 0.02;
// [E] The shapes' 3D Sun light, which the plane-parallel slab lacks (static clouds Task 8 fix round 1: the column model
// against the converged volume with the shapes, cloud_far_check sections 3 and 6): the volume lights a sample whose
// Sun path is shorter than its depth below the column's top as through a surface turned toward the Sun
// (cloud_entry_cosine) - the domes of a deck's top under a low Sun (+12-14 % at 30-36 deg) and the sunlit flanks of
// broken cumulus (+15-20 % at cf 0.4 under a high Sun, seen from above and at 45 deg). The column model takes them as
// an entry cosine turned this fraction of the way from mu0 to 1 (the slab's normal for the Sun's light turned toward
// it, cloud_entry_normal) and a gain of the Sun's light of 1 + this x (1 - f) for the column's largest fraction f
// (none for a deck; bounded: at most 1.4, for f -> 0). Fitted in fix round 1 to those checks (0.3 / 0.15 left broken
// cumulus 10-11 % dark) - they match the game's volume renderer, not a radiative-transfer reference. Held out in fix
// round 2 (unchanged constants, cloud_far_check section 6, three sites each): scattered cumulus cf 0.12 under the high
// Sun and cf 0.4 under a 24 deg Sun, from above and at 45 deg, within 6 % of the converged volume (the plane-parallel
// slab alone 17-31 % dark there: the volume's 3D gain grows toward a low Sun, 1.2x at 73 deg to 1.46x at 24 deg).
const float CLOUD_FAR_ENTRY_TURN = 0.15;
const float CLOUD_FAR_FLANK_GAIN = 0.4;
// [E] The height of a fine column's cloud where the coarse layers hold none (the fine grid's cloud next to a clear
// 1-deg cell: rare, the B-spline spreads the coarse column wider; the flat bake's CLOUD_BAKED_FLAT_DEFAULT_HEIGHT_M).
const float CLOUD_FAR_DEFAULT_HEIGHT_M = 2000.0;
// Fidelity flag (CloudLayer.far_3d_light): false lights the slab as plane-parallel (no entry turn, no gain).
global uniform bool cloud_far_3d_light;
// 8-point Gauss-Legendre nodes and weights on [0, 1].
const int CLOUD_FAR_NODES = 8;
const float CLOUD_FAR_GL_X[8] = float[](0.0198550717512319, 0.1016667612931866, 0.2372337950418355,
		0.4082826787521751, 0.5917173212478249, 0.7627662049581645, 0.8983332387068134, 0.9801449282487681);
const float CLOUD_FAR_GL_W[8] = float[](0.0506142681451881, 0.1111905172266872, 0.1568533229389436,
		0.1813418916891810, 0.1813418916891810, 0.1568533229389436, 0.1111905172266872, 0.0506142681451881);

// Sub-grid structure of the column model (look fixes T7 B, user 2026-11-08): the far field and the flat levels' even
// cells (where the shapes no longer place the cover) drew a cell as a uniform mix - a 0.5 cell a grey veil of half a
// white cloud - and as a homogeneous plane-parallel slab (brighter than the same water spread unevenly: the
// plane-parallel albedo bias, Cahalan et al. 1994, J. Atmos. Sci. 51, 2434).
// - In-cloud optical-depth spread: the in-cloud depth of a cell is gamma distributed with the fractional standard
//   deviation 0.75 (shape nu = 1 / FSD^2 = 1.778; the gamma form Barker 1996, J. Atmos. Sci. 53, 2289; the global FSD
//   0.75 +- 0.18 from observations, Shonk et al. 2010, QJRMS 136, 1191). The slab is drawn at the effective thickness
//   tau_eff = exp(<ln tau>) = chi tau, chi = exp(psi(nu)) / nu (Cahalan et al. 1994's effective-thickness
//   approximation, here the gamma distribution's geometric mean): psi(1.778) = 0.2687 -> chi 0.736.
// - Cover pattern: the cell's cover C as a cloud texture (white cells, dark gaps) - f(x) = Phi(a + k z(x)), z the
//   shapes' noise read CLOUD_FAR_MESO_SCALE times enlarged (a fractal field from ~8 km texels up; prefiltered at the
//   footprint, cloud_shape_prefiltered) taken to a standard normal z by its exact rank at that level (the prefilter
//   tables' inverse, cloud_prefilter_exceedance; read at most at their last level; look fixes T7c - the earlier median /
//   +-1 sigma normalisation left the tails ~uniform), a = Phi^-1(C) sqrt(1 + k^2) so that the mean over z is C; 0 and 1 kept; k fades to 0 over footprints of 8 - 16 texels (f -> C).
//   [E: scale 32 and contrast 1.5 - looked at against DSCOVR EPIC 2025-03-20 (trade cumulus 5-20 km cells); a value
//   noise on a 25-50 km lattice drew square blocks. The pattern is not the data's, only its statistics are; mesoscale
//   cellular convection is 10-50 km across, Wood & Hartmann 2006, J. Climate 19, 1748.]
const float CLOUD_FAR_TAU_SPREAD_CHI = 0.736;
// Fidelity flag (CloudLayer.far_subgrid): false draws the homogeneous slab over the even cover.
global uniform bool cloud_far_subgrid;
const float CLOUD_FAR_MESO_SCALE = 32.0;
const float CLOUD_FAR_MESO_CONTRAST = 1.5;
// [E] The prefilter levels (log2 of the footprint in the pattern's texels) over which the contrast fades to 0.
const vec2 CLOUD_FAR_MESO_FADE_LEVELS = vec2(3.0, 4.0);
// The pattern's reading turned against the shapes' axes: a rotation by 37 deg about (1, 2, 3) / |(1, 2, 3)|.
const mat3 CLOUD_FAR_MESO_TURN = mat3(vec3(0.8130, 0.5113, -0.2785), vec3(-0.4538, 0.8562, 0.2471),
		vec3(0.3648, -0.0745, 0.9281));

// The standard normal distribution function (Abramowitz & Stegun 7.1.26 for erf, |error| < 1.5e-7).
float cloud_far_normal_cdf(float x) {
	float y = abs(x) * 0.70710678;
	float t = 1.0 / (1.0 + 0.3275911 * y);
	float e = 1.0 - (((((1.061405429 * t - 1.453152027) * t) + 1.421413741) * t - 0.284496736) * t + 0.254829592) * t
			* exp(-y * y);
	return x >= 0.0 ? 0.5 * (1.0 + e) : 0.5 * (1.0 - e);
}

// The tail part of Acklam's quantile approximation (q = sqrt(-2 ln p) of the smaller tail probability).
float cloud_far_normal_tail(float q) {
	return (((((-7.784894002430293e-03 * q - 3.223964580411365e-01) * q - 2.400758277161838e+00) * q
			- 2.549732539343734e+00) * q + 4.374664141464968e+00) * q + 2.938163982698783e+00)
			/ ((((7.784695709041462e-03 * q + 3.224671290700398e-01) * q + 2.445134137142996e+00) * q
			+ 3.754408661907416e+00) * q + 1.0);
}

// The inverse of the standard normal distribution function (Acklam's rational approximation, relative error
// < 1.2e-9; p in (0, 1)).
float cloud_far_normal_quantile(float p) {
	if (p < 0.02425) {
		return cloud_far_normal_tail(sqrt(-2.0 * log(p)));
	}
	if (p > 0.97575) {
		return -cloud_far_normal_tail(sqrt(-2.0 * log(1.0 - p)));
	}
	float q = p - 0.5;
	float r = q * q;
	return (((((-3.969683028665376e+01 * r + 2.209460984245205e+02) * r - 2.759285104469687e+02) * r
			+ 1.383577518672690e+02) * r - 3.066479806614716e+01) * r + 2.506628277459239e+00) * q
			/ (((((-5.447609879822406e+01 * r + 1.615858368580409e+02) * r - 1.556989798598866e+02) * r
			+ 6.680131188771972e+01) * r - 1.328068155288572e+01) * r + 1.0);
}

// The in-cloud optical-depth spread resolved from cloud to cloud (look fixes T7b, user 2026-11-08: the capped cumulus
// water kept as the cell mean, thick clouds again - dark bases, relief - and thin ones beside them): a gamma
// distribution (CLOUD_SPREAD_NU; the far field's effective thickness CLOUD_FAR_TAU_SPREAD_CHI keeps FSD 0.75) as a field on
// the sphere (a whole column thick or thin: a tower is thick through its depth) - the shapes' noise read
// CLOUD_SPREAD_SCALE times enlarged (features of a few km, the size of single cumulus [E]) and turned against the shapes'
// axes (the cover pattern's turn inverted), its exact rank u (the noise's own value distribution, cloud_prefilter_exceedance) taken to a normal z and the
// gamma quantile by Wilson & Hilferty (1931), m = (1 - 1/(9 nu) + z / sqrt(9 nu))^3 (mean 0.9998). Independent of the
// shapes, so the mean over any cloud field is 1 (the cell's water kept) and the outlines are the shapes' as before. (A
// first try placed the factor by the shape value's rank inside each cloud: the margins thinned to wisps, the cover
// fell by 12 % and the cores stayed small - not the look.)
// The cumulus type's spread (look fixes T7c, user 2026-11-08): FSD 1 (nu = 1, the exponential) - broken boundary-layer
// cumulus are the most inhomogeneous clouds, nu ~1 and below for broken fields [S: Barker, Wielicki & Parker 1996,
// J. Atmos. Sci. 53, 2304 (Landsat); Oreopoulos & Cahalan 2005, J. Climate 18, 5110 (MODIS: the lowest nu over the
// trade-cumulus regions)]; the far field's and the other types' 0.75 (Shonk et al. 2010's global mean) unchanged.
const float CLOUD_SPREAD_NU = 1.0;  // 1 / FSD^2
const float CLOUD_SPREAD_SCALE = 2.0;
float cloud_spread_factor(vec3 p) {
	vec3 q = transpose(CLOUD_FAR_MESO_TURN) * normalize(p) * (CLOUD_A_M / (CLOUD_SHAPE_TILE_H_M * CLOUD_SPREAD_SCALE));
	float v = texture(cloud_noise_shape, q).r;
	float u = clamp(texture(cloud_prefilter_exceedance, cloud_prefilter_uv(clamp(v, 0.0, 1.0), 0.0)).r, 1.0e-4,
			1.0 - 1.0e-4);
	float z = cloud_far_normal_quantile(1.0 - u);
	float b = 1.0 / (9.0 * CLOUD_SPREAD_NU);
	float m = max(1.0 - b + z * sqrt(b), 0.0);
	return m * m * m;
}

// The cover pattern's local cover at the Earth-local point p (the pattern lies on the sphere: no shift with the
// cloud's placement height) for the cell's cover, seen with the footprint (m).
float cloud_far_meso_cover(vec3 p, float cover, float footprint_m) {
	if (!cloud_far_subgrid || !(cover > 1.0e-4) || cover >= 1.0 - 1.0e-4) {
		return cover;
	}
	float texel = CLOUD_SHAPE_TILE_H_M * CLOUD_FAR_MESO_SCALE / CLOUD_SHAPE_TEXELS;
	float level = log2(max(footprint_m, 1.0) / texel);
	// the contrast fades only far beyond the tables' last level (a footprint of 8 - 16 texels: no sparkle from the
	// Moon's distance), not with the renderer's resolution (the flat levels' march and the far field meet at the
	// scaled-space switch with different pixel footprints)
	float k = CLOUD_FAR_MESO_CONTRAST * (1.0 - smoothstep(CLOUD_FAR_MESO_FADE_LEVELS.x, CLOUD_FAR_MESO_FADE_LEVELS.y,
			level));
	if (!(k > 0.0)) {
		return cover;
	}
	level = clamp(level, 0.0, CLOUD_SHAPE_PREFILTER_LEVELS);
	vec3 q = CLOUD_FAR_MESO_TURN * normalize(p) * (CLOUD_A_M / (CLOUD_SHAPE_TILE_H_M * CLOUD_FAR_MESO_SCALE));
	float v = cloud_shape_prefiltered(q, level).r;
	// the value's exact rank at its level (cloud_prefilter_exceedance, the threshold table's inverse) taken to a normal z
	// (look fixes T7c: the median / +-1 sigma normalisation left the noise ~uniform in its tails - C 0.1 drew 0.08)
	float u = clamp(texture(cloud_prefilter_exceedance, cloud_prefilter_uv(clamp(v, 0.0, 1.0), level)).r, 1.0e-4,
			1.0 - 1.0e-4);
	float z = cloud_far_normal_quantile(1.0 - u);
	float a = cloud_far_normal_quantile(cover) * sqrt(1.0 + k * k);
	return cloud_far_normal_cdf(a + k * z);
}

// The air above a bright cloud top (look fixes T7 step 3, T5 review I2: the clouds read pink). The cloud is lit by the
// direct Sun and the sky irradiance table, seen through the direct Beer transmittance plus the table in-scatter; the
// tables are built over a ground of albedo CLOUD_FAR_TABLE_GROUND_ALBEDO, so two terms of a bright top are missing, both
// bluer than the cloud: (1) the cloud's own reflected light scattered back down by the air above it (the multiple
// reflection between the top and the air, 1 / (1 - A s) instead of 1 / (1 - A_t s)), (2) the cloud light scattered
// toward the camera by the air above it (its diffuse transmission; the direct Beer term alone is a dark surface's).
// Conservative two-stream (Eddington, g = 0: Rayleigh) for the air above the cloud of vertical depth tau (from the
// view's Beer transmittance T and cosine mu, tau = -mu ln T; absorption by ozone counted as scattering: Chappuis tau
// ~0.03): spherical albedo s = tau / (4/3 + tau), total transmission toward mu of an extended Lambertian source (by
// reciprocity a beam's) T_tot = ((2/3 + mu) + (2/3 - mu) T) / (4/3 + tau) [S: the Eddington solution, e.g. Liou 2002,
// An Introduction to Atmospheric Radiation, ch. 6; Kokhanovsky 2006, Cloud Optics]. A the albedo the air sees: the
// cloudy part's (Eddington slab of the in-cloud depth, g of the phase function) over the clear part's table ground.
// Expected (review I2, tau_R 0.15 blue / 0.04 red above a 2 km top, A 0.6-0.75): cloud B/R +8-10 %.
const float CLOUD_FAR_TABLE_GROUND_ALBEDO = 0.1;  // config/atmosphere/earth_optics.toml ground_albedo (the tables')

// The factor for the cloud's radiance seen through the air (replaces the Beer transmittance t_air): mu the view's
// cosine at the cloud, cover its cover, tau_cloud the in-cloud depth.
vec3 cloud_far_bright_top_transmittance(vec3 t_air, float mu, float cover, float tau_cloud) {
	vec3 t = clamp(t_air, vec3(1.0e-6), vec3(1.0));
	vec3 tau = -log(t) * mu;
	vec3 s = tau / (4.0 / 3.0 + tau);
	float x = 0.75 * (1.0 - cloud_asymmetry_g) * tau_cloud;
	float albedo = mix(CLOUD_FAR_TABLE_GROUND_ALBEDO, x / (1.0 + x), clamp(cover, 0.0, 1.0));
	vec3 down = (1.0 - CLOUD_FAR_TABLE_GROUND_ALBEDO * s) / (1.0 - albedo * s);
	vec3 total = ((2.0 / 3.0 + mu) + (2.0 / 3.0 - mu) * t) / (4.0 / 3.0 + tau);
	return max(total, t_air) * down;
}

// The scattering angle (x), its folded azimuth about the light's propagation from the vertical axis (y) and
// 1 - cos(theta) (z) of a view (core cloud_sun_frame_angles; cloud_scattering_source_mc's geometry with the axis up).
vec3 cloud_far_angles(vec3 light_dir, vec3 view_prop, vec3 up) {
	vec3 chord = view_prop + light_dir;
	float one_minus_cos = 0.5 * dot(chord, chord);
	float theta = 2.0 * asin(min(1.0, 0.5 * length(chord)));
	vec3 s = -light_dir;
	vec3 e1 = up - s * dot(up, s);
	float psi = 0.0;
	float e1_norm = length(e1);
	if (e1_norm > 1.0e-6) {
		e1 /= e1_norm;
		vec3 across = view_prop - s * dot(view_prop, s);
		float c1 = dot(across, e1);
		psi = atan(length(across - e1 * c1), c1);
	}
	return vec3(theta, psi, one_minus_cos);
}

// The light's radiance leaving a slab of vertical optical depth tau toward view_prop, per unit irradiance normal to
// the light (1/sr): seen from above (view_prop up, dot(view_prop, up) >= 0) the reflected light (core
// slab_reflected_radiance_mc with the table, the closed form's integral otherwise), from below the diffusely
// transmitted light - the same source J(t) integrated from the slab's base, attenuated by exp(-w (tau - t) / |mu|)
// (static clouds Task 8: a camera under a far layer sees its base). light_dir toward the light, view_prop the
// propagation toward the camera, up the slab's normal; all unit.
// below: the camera's side of the slab, from the layer's geometric up (cloud_far_light_radiance_side): with the light's
// entry normal turned toward a low Sun (cloud_far_cloudy_radiance) a view from above at less than the turn's angle
// lies below the turned plane - deciding the side by it drew a cloud deck seen toward the Sun near the limb as its dark
// base (bugs 2026-11-09: a second, inner horizon line from orbit). The view's cosine against the turned normal is then
// held at CLOUD_FAR_MU_MIN (continuous through the turned plane).
float cloud_far_light_radiance_side(float tau, vec3 light_dir, vec3 view_prop, vec3 up, bool below) {
	if (!(tau > 0.0)) {
		return 0.0;
	}
	float mu_signed = dot(view_prop, up);
	float mu = max(below ? -mu_signed : mu_signed, CLOUD_FAR_MU_MIN);
	float mu0 = max(dot(light_dir, up), CLOUD_ENTRY_COSINE_MIN);
	vec3 angles = cloud_far_angles(light_dir, view_prop, up);
	float phase = cloud_phase_function(1.0 - angles.z, angles.z);
	if (cloud_scattering_model == 1) {
		float w = cloud_phase.w;
		float r = w / mu;  // the ray's attenuation exp(-w s / mu) per unit vertical depth s from its exit
		float single;
		if (below) {
			// phase / mu int_0^tau exp(-t / mu0) exp(-r (tau - t)) dt
			float a = 1.0 / mu0 - r;
			single = abs(a * tau) < 1.0e-4 ? phase / mu * tau * exp(-r * tau)
					: phase / (mu * a) * (exp(-r * tau) - exp(-tau / mu0));
		} else {
			float a = 1.0 / mu0 + r;
			single = phase / (mu * a) * (1.0 - exp(-tau * a));
		}
		// Orders >= 2: (1 / w) int_0^U J(t(u)) du, u = 1 - exp(-r s), s the depth from the exit (top or base).
		float u_end = 1.0 - exp(-r * tau);
		float multiple = 0.0;
		for (int i = 0; i < CLOUD_FAR_NODES; i++) {
			float s = min(-log(1.0 - u_end * CLOUD_FAR_GL_X[i]) / r, tau);
			float t = below ? tau - s : s;
			multiple += CLOUD_FAR_GL_W[i] * cloud_transfer_lookup(t, tau - t, mu0, angles.x, angles.y);
		}
		return single + multiple * u_end / w;
	}
	// The closed form (cloud_scattering_source: its own single scattering inside), attenuated by exp(-k s / mu).
	float k = cloud_transport_scale();
	float r = k / mu;
	float u_end = 1.0 - exp(-r * tau);
	float sum = 0.0;
	for (int i = 0; i < CLOUD_FAR_NODES; i++) {
		float s = min(-log(1.0 - u_end * CLOUD_FAR_GL_X[i]) / r, tau);
		float t = below ? tau - s : s;
		sum += CLOUD_FAR_GL_W[i]
				* cloud_scattering_source(1.0 - angles.z, angles.z, below ? -mu : mu, mu0, t / mu0, t, tau);
	}
	return sum * u_end / k;
}

// The same with the side taken from up itself (the slab's normal is its geometric up).
float cloud_far_light_radiance(float tau, vec3 light_dir, vec3 view_prop, vec3 up) {
	return cloud_far_light_radiance_side(tau, light_dir, view_prop, up, dot(view_prop, up) < 0.0);
}

// The sky's radiance leaving the slab toward a view of signed cosine mu_view (> 0 from above, reflected; < 0 from
// below, transmitted), per unit horizontal sky irradiance: the sky source S(t) = P + Q t (cloud_sky_scattering_source,
// linear in the vertical depth t) along the ray with the scaled attenuation exp(-k s / |mu|) from its exit, in closed
// form. Q from the source at the slab's base (the source's depth is clamped to the slab: a unit depth beyond a slab
// thinner than 1 underestimated the slope).
float cloud_far_sky_radiance(float tau, float mu_view) {
	if (!(tau > 0.0)) {
		return 0.0;
	}
	float mu = max(abs(mu_view), CLOUD_FAR_MU_MIN);
	float mu_s = mu_view < 0.0 ? -mu : mu;
	float p = cloud_sky_scattering_source(mu_s, 0.0, tau);
	float q = (cloud_sky_scattering_source(mu_s, tau, tau) - p) / tau;
	float b = cloud_transport_scale() / mu;
	float e = exp(-b * tau);
	if (mu_view < 0.0) {
		// from the base: S(tau - s) = (P + Q tau) - Q s
		return ((p + q * tau) * (1.0 - e) / b - q * (1.0 - e * (1.0 + b * tau)) / (b * b)) / mu;
	}
	return (p * (1.0 - e) / b + q * (1.0 - e * (1.0 + b * tau)) / (b * b)) / mu;
}

// The blended grid-mean depth from 20 km down to layer centre `layer` (the column texels, bilinear).
float cloud_far_depth_at_centre(vec2 lat_lon, float layer) {
	float depth = 0.0;
	if (cloud_blend < 1.0) {
		depth += (1.0 - cloud_blend)
				* cloud_column_texel(cloud_column_before, cloud_field_before_size, lat_lon, layer).x;
	}
	if (cloud_blend > 0.0) {
		depth += cloud_blend * cloud_column_texel(cloud_column_after, cloud_field_after_size, lat_lon, layer).x;
	}
	return depth;
}

// The height at which the grid-mean depth from the column's top reaches target (> 0): bisection over the layer
// centres (depth non-increasing upward; 0 at 20 km), linear between the two centres around it.
float cloud_far_depth_height(vec2 lat_lon, float target) {
	float lo = 0.0;
	float hi = CLOUD_LAYERS;  // a virtual centre at 20 km with depth 0
	float d_lo = cloud_far_depth_at_centre(lat_lon, 0.0);
	if (d_lo <= target) {
		return cloud_layer_centre_m(0.0);
	}
	float d_hi = 0.0;
	for (int i = 0; i < 5; i++) {
		float mid = floor(0.5 * (lo + hi));
		float d = cloud_far_depth_at_centre(lat_lon, mid);
		if (d >= target) {
			lo = mid;
			d_lo = d;
		} else {
			hi = mid;
			d_hi = d;
		}
	}
	float h_lo = cloud_layer_centre_m(lo);
	float h_hi = hi >= CLOUD_LAYERS ? CLOUD_TOP_M : cloud_layer_centre_m(hi);
	return mix(h_lo, h_hi, clamp((d_lo - target) / max(d_lo - d_hi, 1.0e-9), 0.0, 1.0));
}

// The point where the ray from the camera through the ground point g (Earth-local, view unit along the ray) crosses
// the height h on the way down (model space: the ellipsoid as the sphere of radius a).
vec3 cloud_far_crossing(vec3 g, vec3 view, float h) {
	vec3 gm = g * ATM_STRETCH;
	vec3 um = view * ATM_STRETCH;
	float r = CLOUD_A_M + h;
	float g_len = length(gm);
	float delta = (r - g_len) * (r + g_len);  // r^2 - |g|^2, > 0 below the height
	float uu = dot(um, um);
	float b = max(-dot(gm, um), 0.0);
	float s = max(delta, 0.0) / (b + sqrt(b * b + uu * max(delta, 0.0)));
	return g - view * s;
}

// The grid-mean vertical depth from the side the light is seen from at which the slab is placed: the
// transmittance-weighted mean in-cloud ray depth of the cloudy part, x (1 - (1 + X) e^-X) / (1 - e^-X), as a grid-mean
// vertical depth (column: x the grid-mean depth, y the largest fraction; mu the view's |cosine|).
float cloud_far_seen_depth(vec2 column, float mu) {
	float cover = max(cloud_column_beam_cover(column, mu), CLOUD_MIN_FRACTION);
	float x = min(column.x / (cover * mu), 50.0);
	float mean_ray = x < 1.0e-3 ? 0.5 * x : (1.0 - (1.0 + x) * exp(-x)) / (1.0 - exp(-x));
	return cover * mu * mean_ray;
}

// [E] Low Sun: the shapes' sunlit faces (baked step 7). The slab's normal turned 0.15 of the way toward the Sun
// (CLOUD_FAR_ENTRY_TURN, fitted at Sun 24-73 deg) lights it at mu_e 0.27 under an 8 deg Sun; the volume's shapes take
// the low Sun through faces turned toward it (cloud_entry_cosine -> 1), and a view from the Sun's side sees those lit
// faces: at 8 deg the volume's Sun light was 3.2 x the slab's (50 km, ERA5 cumulus; baked-lowsun-report.md). A share w
// of the Sun's slab radiance is replaced by that of a slab whose normal is the bisector of the Sun and the view (the
// faces seen lit face both on average), w = CLOUD_FAR_FACET_SHARE x s / (s + CLOUD_FAR_FACET_ONSET) x (1 + cos alpha)
// / 2: s the beam cover's slant term (cloud_beam_cover_of: CLOUD_OVERLAP_SLANT max(cot - CLOUD_OVERLAP_ONSET_COT, 0),
// the beam meeting the shapes through their sides; 0 above a 29 deg Sun - the high-Sun fit bit for bit), (1 + cos
// alpha) / 2 the lit share of a convex shape's visible area at the phase angle alpha (Sun - cloud - camera). The share
// and the onset were fitted to the volume levels at 8 deg (from the Sun's side), held out at 2, 5 and 15 deg and
// toward the Sun. The flat levels and the scaled-space far field only (the old march's far tier unchanged).
const float CLOUD_FAR_FACET_SHARE = 0.75;
const float CLOUD_FAR_FACET_ONSET = 0.1;
// The facets' weight w (0: none) and their normal (the bisector of the Sun and the view).
float cloud_far_facet_weight(vec3 sun_dir, vec3 view_prop, float mu0, out vec3 normal) {
	normal = vec3(0.0, 0.0, 1.0);
	float m = max(mu0, CLOUD_ENTRY_COSINE_MIN);
	float cot = sqrt(max(1.0 - m * m, 0.0)) / m;
	float s = CLOUD_OVERLAP_SLANT * max(cot - CLOUD_OVERLAP_ONSET_COT, 0.0);
	float lit = 0.5 * (1.0 + dot(sun_dir, view_prop));
	vec3 h = sun_dir + view_prop;
	float h_norm = length(h);
	if (!(s > 0.0) || !(h_norm > 1.0e-4) || !(lit > 0.0)) {
		return 0.0;
	}
	normal = h / h_norm;
	return CLOUD_FAR_FACET_SHARE * s / (s + CLOUD_FAR_FACET_ONSET) * lit;
}

// The radiance of a cloudy slab of in-cloud vertical depth tau at the cloud point p (Earth-local, up its geodetic up)
// toward the camera (view_prop: the propagation toward it; from above or from below by its sign against up): the
// lights at the cloud as the cloud pass takes them (irradiance normal to the light; the sky's horizontal), not yet
// attenuated by the air; the Sun's with the shapes' 3D light for the column's largest fraction (CLOUD_FAR_ENTRY_TURN,
// CLOUD_FAR_FLANK_GAIN). sky_eclipse: the eclipse sky's factor there.
vec3 cloud_far_cloudy_radiance(vec3 p, vec3 up, vec3 view_prop, float tau, float fraction, vec3 sky_eclipse) {
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	// The shapes' 3D Sun light (CLOUD_FAR_ENTRY_TURN, CLOUD_FAR_FLANK_GAIN).
	float mu0 = dot(sun_dir, up);
	vec3 entry = up;
	float gain = 1.0;
	if (cloud_far_3d_light) {
		// continuous through the local horizon (baked step 6; the switch to the bare up at mu0 = 0 dropped the light
		// ~7 x there: a hard day / night edge on the clouds): a Sun below it still lights the tops turned toward it
		// until the Earth's horizon seen from the cloud hides it - the Sun's grazing-path transmittance and its disc's
		// visible fraction (atm_sun_at) end the light, the twilight sky's (atm_sky_light) goes on
		entry = cloud_entry_normal(sun_dir, up, mu0, max(mu0 + (1.0 - mu0) * CLOUD_FAR_ENTRY_TURN,
				CLOUD_ENTRY_COSINE_MIN));
		gain = 1.0 + CLOUD_FAR_FLANK_GAIN * (1.0 - clamp(fraction, 0.0, 1.0));
	}
	vec3 sun_irradiance = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
	vec3 sky_irradiance = ATM_PI * atm_sky_light(p, up) * sky_eclipse;
	float sky_radiance = cloud_far_sky_radiance(tau, dot(view_prop, up));
	vec3 radiance = sky_irradiance * sky_radiance;
// (diagnostics, STARSIM_BAKED_DEFINES: the Sun's or the sky's light alone; CLOUD_DIAG_NO_FACETS: the slab alone)
#ifdef CLOUD_DIAG_SUN_ONLY
	radiance = vec3(0.0);
#endif
#ifdef CLOUD_DIAG_SKY_ONLY
	sun_irradiance = vec3(0.0);
#endif
	if (dot(sun_irradiance, sun_irradiance) > 0.0) {
// (in the baked march only its kernels above 40 km camera height - the flat levels from the ray's start, orbit - where
// CLOUD_BAKED_FAR_FALLBACK is defined: in the one kernel the code raised the march's register peak, ground and deck
// +0.15 ms even with w 0; elsewhere the code as before step 7)
#if !defined(CLOUD_DIAG_NO_FACETS) && (!defined(CLOUD_BAKED_MARCH) || defined(CLOUD_BAKED_FAR_FALLBACK))
		vec3 facet = up;
		float w_facet = cloud_far_3d_light ? cloud_far_facet_weight(sun_dir, view_prop, mu0, facet) : 0.0;
		// the slab and (w_facet > 0) the facets through one copy of the light function (a loop: a second inlined call
		// raised the register peak)
		float sun_radiance = 0.0;
		int normals = w_facet > 0.0 ? 2 : 1;
		// (the slab's side from its geometric up, cloud_far_light_radiance_side; the facets' from their own normal)
		bool below = dot(view_prop, up) < 0.0;
		for (int k = 0; k < normals; k++) {
			vec3 normal = k == 0 ? entry : facet;
			sun_radiance += (k == 0 ? 1.0 - w_facet : w_facet) * cloud_far_light_radiance_side(tau, sun_dir, view_prop,
					normal, k == 0 ? below : dot(view_prop, normal) < 0.0);
		}
#else
		float sun_radiance = cloud_far_light_radiance_side(tau, sun_dir, view_prop, entry, dot(view_prop, up) < 0.0);
#endif
		radiance += sun_irradiance * gain * sun_radiance;
	}
	if (atm_moonlit()) {
		vec3 moon_dir = normalize(atmosphere_moon_direction);
		radiance += ATM_PI * atm_moon_at(p) * cloud_far_light_radiance(tau, moon_dir, view_prop, up)
				+ ATM_PI * atm_moon_sky_light(p, up) * sky_radiance;
	}
	return radiance;
}

// The cloud layer over the ground point g (Earth-local) of the scaled-space Earth: premultiplied colour (rgb) and
// alpha (a), to blend over the ground's colour as rgb + (1 - a) x ground. sky_eclipse: the eclipse sky's factor there.
vec4 cloud_far_layer(vec3 g, vec3 sky_eclipse, float footprint_m) {
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	vec3 view = normalize(g - camera);
	// The column texels' B-spline (cloud_column_mean_depth_smooth), as the cloud pass's far tier and the baked flat levels:
	// the bilinear's creases along the cell edges, sharpened by the cover's non-linear alpha, drew the field's grid as
	// rectangular blocks (the 1 deg analog from the start view, ~5 pixels a cell; look fixes T2). The placement's
	// bisection below stays on the bilinear layer centres (as the flat levels' bake).
	// Depth and cover from the fine column (look fixes T7 C: the 1-deg analog set's 0.25 deg column; the snapshot's own
	// grid otherwise); the height from the layers (the coarse column's B-spline: the seen depth as the same share).
	vec2 column_g = cloud_column_total_fine(cloud_lat_lon(g));
	if (!(column_g.x > 0.0)) {
		return vec4(0.0);
	}
	// The height to place the cloud at (cloud_far_seen_depth).
	float mu_g = max(-dot(view, cloud_geodetic_up(g)), CLOUD_FAR_MU_MIN);
	float coarse_g = cloud_column_mean_depth_smooth(cloud_lat_lon(g), 0.0).x;
	float h = coarse_g > 0.0
			? cloud_far_depth_height(cloud_lat_lon(g), cloud_far_seen_depth(column_g, mu_g) * coarse_g / column_g.x)
			: CLOUD_FAR_DEFAULT_HEIGHT_M;
	vec3 p = cloud_far_crossing(g, view, h);
	vec2 column = cloud_column_total_fine(cloud_lat_lon(p));
	if (!(column.x > 0.0)) {
		return vec4(0.0);
	}
	vec3 up = cloud_geodetic_up(p);
	vec3 view_prop = -view;
	float mu = max(dot(view_prop, up), CLOUD_FAR_MU_MIN);
	// the cover the data's total cover from straight down (cloud_column_total_cover, look fixes T7c), slanted grown by
	// the beam rule's slant term (the largest fraction with that nadir cover)
	float cover = max(cloud_beam_cover_of(cloud_beam_fraction_of(cloud_column_total_cover(cloud_lat_lon(p), column.y)),
			mu), CLOUD_MIN_FRACTION);
	// the cloudy part's vertical depth at the effective thickness of the in-cloud spread (CLOUD_FAR_TAU_SPREAD_CHI)
	float tau = (cloud_far_subgrid ? CLOUD_FAR_TAU_SPREAD_CHI : 1.0) * column.x / cover;
	// the cover as mesoscale cells (cloud_far_meso_cover; the in-cloud depth kept: the water's mean kept)
	cover = cloud_far_meso_cover(p, cover, footprint_m);
	float alpha = cover * (1.0 - exp(-tau / mu));
	vec3 radiance = cloud_far_cloudy_radiance(p, up, view_prop, tau, column.y, sky_eclipse);
	vec3 air_transmittance;
	vec3 air_light = atm_to_point_offset(p - camera, air_transmittance);
	vec3 seen = cloud_far_bright_top_transmittance(air_transmittance, mu, cover, tau);  // (the air above: step 3)
	return vec4(seen * radiance * cover + alpha * air_light, alpha);
}
