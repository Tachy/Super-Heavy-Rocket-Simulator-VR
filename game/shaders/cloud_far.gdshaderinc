// The far field of the clouds (M8 weather and clouds, Task 10): the cloud layer of the scaled-space Earth
// (earth_clouds.gdshader, drawn while the camera is beyond CelestialBody.SCALED_SPACE_DISTANCE_M, where the cloud pass
// is off) and, since static clouds Task 8, the column part of the cloud pass's far tier at large footprints
// (cloud_pass.gdshaderinc cloud_far_column: the same slab, seen from above or from below, with a sub-grid cover).
// Include atmosphere.gdshaderinc and clouds.gdshaderinc first.
//
// The same field and the same light transport as the cloud pass, in its plane-parallel limit: a pixel there covers
// >= 7 km (the shapes' detail levels have faded to their mean; the shape noise itself is resolved by a few pixels at
// most), so the column replaces the march:
// - The column above the ground point G (the column texels, cloud_column_mean_depth): the grid-mean optical depth tau
//   and the largest cloud fraction f. The view ray's cover is the beam cover of f at the view's cosine mu
//   (cloud_beam_cover_of: the rule fitted to the shapes' shadows, for any line through the layer), C_v; independent
//   columns: clear on 1 - C_v, the in-cloud depth tau / C_v on C_v.
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
// Fidelity flag (CloudLayer.far_3d_light): false lights the slab as plane-parallel (no entry turn, no gain).
global uniform bool cloud_far_3d_light;
// 8-point Gauss-Legendre nodes and weights on [0, 1].
const int CLOUD_FAR_NODES = 8;
const float CLOUD_FAR_GL_X[8] = float[](0.0198550717512319, 0.1016667612931866, 0.2372337950418355,
		0.4082826787521751, 0.5917173212478249, 0.7627662049581645, 0.8983332387068134, 0.9801449282487681);
const float CLOUD_FAR_GL_W[8] = float[](0.0506142681451881, 0.1111905172266872, 0.1568533229389436,
		0.1813418916891810, 0.1813418916891810, 0.1568533229389436, 0.1111905172266872, 0.0506142681451881);

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
float cloud_far_light_radiance(float tau, vec3 light_dir, vec3 view_prop, vec3 up) {
	if (!(tau > 0.0)) {
		return 0.0;
	}
	float mu_signed = dot(view_prop, up);
	bool below = mu_signed < 0.0;
	float mu = max(abs(mu_signed), CLOUD_FAR_MU_MIN);
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
				* cloud_scattering_source(1.0 - angles.z, angles.z, mu_signed < 0.0 ? -mu : mu, mu0, t / mu0, t, tau);
	}
	return sum * u_end / k;
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
#if defined(CLOUD_MARCH) && !defined(CLOUD_BAKED_MARCH)
		// (the old march's far tier: unchanged - the turn ends at the local horizon, a step of the light there)
		entry = mu0 > 0.0 ? cloud_entry_normal(sun_dir, up, mu0, mu0 + (1.0 - mu0) * CLOUD_FAR_ENTRY_TURN) : up;
#else
		// continuous through the local horizon (baked step 6; the switch to the bare up at mu0 = 0 dropped the light
		// ~7 x there: a hard day / night edge on the clouds): a Sun below it still lights the tops turned toward it
		// until the Earth's horizon seen from the cloud hides it - the Sun's grazing-path transmittance and its disc's
		// visible fraction (atm_sun_at) end the light, the twilight sky's (atm_sky_light) goes on
		entry = cloud_entry_normal(sun_dir, up, mu0, max(mu0 + (1.0 - mu0) * CLOUD_FAR_ENTRY_TURN,
				CLOUD_ENTRY_COSINE_MIN));
#endif
		gain = 1.0 + CLOUD_FAR_FLANK_GAIN * (1.0 - clamp(fraction, 0.0, 1.0));
	}
	vec3 sun_irradiance = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
	vec3 sky_irradiance = ATM_PI * atm_sky_light(p, up) * sky_eclipse;
	float sky_radiance = cloud_far_sky_radiance(tau, dot(view_prop, up));
	vec3 radiance = sky_irradiance * sky_radiance;
	if (dot(sun_irradiance, sun_irradiance) > 0.0) {
		radiance += sun_irradiance * gain * cloud_far_light_radiance(tau, sun_dir, view_prop, entry);
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
vec4 cloud_far_layer(vec3 g, vec3 sky_eclipse) {
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	vec3 view = normalize(g - camera);
	vec2 column_g = cloud_column_mean_depth(cloud_lat_lon(g), 0.0);
	if (!(column_g.x > 0.0)) {
		return vec4(0.0);
	}
	// The height to place the cloud at (cloud_far_seen_depth).
	float mu_g = max(-dot(view, cloud_geodetic_up(g)), CLOUD_FAR_MU_MIN);
	float h = cloud_far_depth_height(cloud_lat_lon(g), cloud_far_seen_depth(column_g, mu_g));
	vec3 p = cloud_far_crossing(g, view, h);
	vec2 column = cloud_column_mean_depth(cloud_lat_lon(p), 0.0);
	if (!(column.x > 0.0)) {
		return vec4(0.0);
	}
	vec3 up = cloud_geodetic_up(p);
	vec3 view_prop = -view;
	float mu = max(dot(view_prop, up), CLOUD_FAR_MU_MIN);
	float cover = max(cloud_column_beam_cover(column, mu), CLOUD_MIN_FRACTION);
	float tau = column.x / cover;  // the cloudy part's vertical depth
	float alpha = cover * (1.0 - exp(-tau / mu));
	vec3 radiance = cloud_far_cloudy_radiance(p, up, view_prop, tau, column.y, sky_eclipse);
	vec3 air_transmittance;
	vec3 air_light = atm_to_point_offset(p - camera, air_transmittance);
	return vec4(air_transmittance * radiance * cover + alpha * air_light, alpha);
}
