// The far field of the clouds (M8 weather and clouds, Task 10): the cloud layer of the scaled-space Earth
// (earth_clouds.gdshader, drawn while the camera is beyond CelestialBody.SCALED_SPACE_DISTANCE_M, where the cloud pass
// is off). Include atmosphere.gdshaderinc and clouds.gdshaderinc first.
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

// The light's radiance reflected by a slab of vertical optical depth tau toward view_prop, per unit irradiance normal
// to the light (1/sr; core slab_reflected_radiance_mc with the table, the closed form's integral otherwise). light_dir
// toward the light, view_prop the propagation toward the camera, up the slab's normal; all unit.
float cloud_far_light_radiance(float tau, vec3 light_dir, vec3 view_prop, vec3 up) {
	if (!(tau > 0.0)) {
		return 0.0;
	}
	float mu = max(dot(view_prop, up), CLOUD_FAR_MU_MIN);
	float mu0 = max(dot(light_dir, up), CLOUD_ENTRY_COSINE_MIN);
	vec3 angles = cloud_far_angles(light_dir, view_prop, up);
	float phase = cloud_phase_function(1.0 - angles.z, angles.z);
	if (cloud_scattering_model == 1) {
		float w = cloud_phase.w;
		float r = w / mu;  // the ray's attenuation exp(-w t / mu) per unit vertical depth
		float a = 1.0 / mu0 + r;
		float single = phase / (mu * a) * (1.0 - exp(-tau * a));
		// Orders >= 2: (1 / w) int_0^U J(t(u)) du, u = 1 - exp(-r t).
		float u_end = 1.0 - exp(-r * tau);
		float multiple = 0.0;
		for (int i = 0; i < CLOUD_FAR_NODES; i++) {
			float t = min(-log(1.0 - u_end * CLOUD_FAR_GL_X[i]) / r, tau);
			multiple += CLOUD_FAR_GL_W[i] * cloud_transfer_lookup(t, tau - t, mu0, angles.x, angles.y);
		}
		return single + multiple * u_end / w;
	}
	// The closed form (cloud_scattering_source: its own single scattering inside), attenuated by exp(-k t / mu).
	float k = cloud_transport_scale();
	float r = k / mu;
	float u_end = 1.0 - exp(-r * tau);
	float sum = 0.0;
	for (int i = 0; i < CLOUD_FAR_NODES; i++) {
		float t = min(-log(1.0 - u_end * CLOUD_FAR_GL_X[i]) / r, tau);
		sum += CLOUD_FAR_GL_W[i]
				* cloud_scattering_source(1.0 - angles.z, angles.z, mu, mu0, t / mu0, t, tau);
	}
	return sum * u_end / k;
}

// The sky's radiance reflected by the slab toward a view of cosine mu, per unit horizontal sky irradiance: the sky
// source S(t) = P + Q t (cloud_sky_scattering_source, linear in the vertical depth t) along the ray with the scaled
// attenuation exp(-k t / mu), in closed form.
float cloud_far_sky_radiance(float tau, float mu_view) {
	if (!(tau > 0.0)) {
		return 0.0;
	}
	float mu = max(mu_view, CLOUD_FAR_MU_MIN);
	float p = cloud_sky_scattering_source(mu, 0.0, tau);
	float q = cloud_sky_scattering_source(mu, 1.0, tau) - p;
	float b = cloud_transport_scale() / mu;
	float e = exp(-b * tau);
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

// The cloud layer over the ground point g (Earth-local) of the scaled-space Earth: premultiplied colour (rgb) and
// alpha (a), to blend over the ground's colour as rgb + (1 - a) x ground. sky_eclipse: the eclipse sky's factor there.
vec4 cloud_far_layer(vec3 g, vec3 sky_eclipse) {
	vec3 camera = atmosphere_camera_hi + atmosphere_camera_lo;
	vec3 view = normalize(g - camera);
	vec2 column_g = cloud_column_mean_depth(cloud_lat_lon(g), 0.0);
	if (!(column_g.x > 0.0)) {
		return vec4(0.0);
	}
	// The transmittance-weighted mean in-cloud ray depth of the cloudy part, x (1 - (1 + X) e^-X) / (1 - e^-X), as a
	// grid-mean vertical depth: the height to place the cloud at.
	float mu_g = max(-dot(view, cloud_geodetic_up(g)), CLOUD_FAR_MU_MIN);
	float cover_g = max(cloud_column_beam_cover(column_g, mu_g), CLOUD_MIN_FRACTION);
	float x = min(column_g.x / (cover_g * mu_g), 50.0);
	float mean_ray = x < 1.0e-3 ? 0.5 * x : (1.0 - (1.0 + x) * exp(-x)) / (1.0 - exp(-x));
	float h = cloud_far_depth_height(cloud_lat_lon(g), cover_g * mu_g * mean_ray);
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
	// The lights at the cloud, as the cloud pass takes them (irradiance normal to the light; the sky's horizontal).
	vec3 sun_dir = normalize(atmosphere_sun_direction);
	vec3 sun_irradiance = ATM_PI * atm_sun_at(p) * atm_sunlit(p);
	vec3 sky_irradiance = ATM_PI * atm_sky_light(p, up) * sky_eclipse;
	float sky_radiance = cloud_far_sky_radiance(tau, mu);
	vec3 radiance = sky_irradiance * sky_radiance;
	if (dot(sun_irradiance, sun_irradiance) > 0.0) {
		radiance += sun_irradiance * cloud_far_light_radiance(tau, sun_dir, view_prop, up);
	}
	if (atm_moonlit()) {
		vec3 moon_dir = normalize(atmosphere_moon_direction);
		radiance += ATM_PI * atm_moon_at(p) * cloud_far_light_radiance(tau, moon_dir, view_prop, up)
				+ ATM_PI * atm_moon_sky_light(p, up) * sky_radiance;
	}
	vec3 air_transmittance;
	vec3 air_light = atm_to_point_offset(p - camera, air_transmittance);
	return vec4(air_transmittance * radiance * cover + alpha * air_light, alpha);
}
