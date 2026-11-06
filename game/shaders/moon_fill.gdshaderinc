// Terrain inter-reflection fill light on the Moon (moon procedural surface plan Task 12; review T11-C5 b): the light the
// sunlit ground and terrain around a receiver scatter onto it. Hapke's model holds the multiple scattering inside the
// regolith, not the macroscopic view factor between terrain facets: a crater wall or a rock face in shadow is lit by the
// sunlit ground it sees. Needs moon_regolith.gdshaderinc, moon_detail.gdshaderinc and moon_horizon.gdshaderinc first.
//
// Radiosity with Lambertian-equivalent emitters (the view-factor form of the rendering
// equation, Goral et al. 1984, SIGGRAPH; terrain view factors after Dozier & Frew 1990, IEEE TGRS 28, 963): the sunlit ground around exits
// M = E mu0_g A_h(mu0_g) f_lit (A_h the Hapke directional-hemispherical albedo, core hapke_hemispherical_albedo), its
// radiance M / pi; a receiver of normal n sees it over the view factor F, and scatters the irradiance pi L F back as
// L_r = (E / pi) A_h(mu_r) x [mu0_g A_h(mu0_g) f_lit F] (reciprocity: the diffuse-irradiance response at the view's
// cosine mu_r is A_h(mu_r)). F: the ground below the receiver's horizontal plane, (1 - n.up) / 2 (exact for a flat
// surround), with the flat ground's exitance (f_lit: moon_fill_emitter, the surround's statistical lit share at the
// Sun's macroscopic elevation, times the receiver's own horizon visibility V standing for its near surround's [E]);
// plus the terrain above the horizon from the node's horizon map: for a horizontal receiver the view factor of terrain
// up to elevation h(phi) is the azimuth mean of sin^2 h, each sector's wall lit at its own incidence (normal tilted by h
// toward the receiver, [E] the wall's slope taken as its horizon elevation), weighted (1 + n.up) / 2 for a tilted
// receiver ([E]).
// Each wall's own Sun visibility (Task 12 fix 2, review N2; not the receiver's: a floor in terrain shadow is lit by the
// sunlit far wall, which lies in another azimuth than the occluder that shadows the floor): the occluder toward the Sun
// is the receiver's horizon H_s in the Sun's azimuth, a ridge across the Sun's direction at horizontal distance x_s;
// the wall point P in azimuth phi lies at distance d, height d tan h. The ray from P to the Sun (elevation e) meets the
// ridge line after x_s - d cos(dphi) and sees it at tan H_P = (x_s tan H_s - d tan h) / (x_s - d cos(dphi)); [E] d = x_s
// (the wall and the occluder equally far, a crater floor near its middle): tan H_P = (tan H_s - tan h) / (1 - cos dphi),
// the disc's visible fraction above H_P, never below V (for walls toward the Sun the expression tends to the receiver's
// own occluder). It over-estimates the far wall's light for a receiver close to that wall and under-estimates it close
// to the occluder.
// Each wall also carries Smith's G1 of the regolith relief at its own incidence over the band the ground's statistics
// use (drawn(node spacing) - drawn(horizon spacing), review n1: the finer relief is Hapke's theta_bar inside A_h, the
// coarser the wall's own tilt h).
// The emitter is the surround's hemispherical lit share, not the eye's view of it (review I2: the exitance must not turn
// with the eye), and the same for every receiver at a place (review N3: moon_fill_emitter, all rocks statistical - the
// plan-area mean of the ground and its rocks, independent of what the eye draws).
// In the materials' units (radiance = albedo x moon_regolith_scale x E_scene x R, R = moon_hapke_norm x I/F): the
// render's own I/F of the ground is K x I/F_Hapke(w), K = albedo x scale x norm, so the fill R term is
// norm x A_h(mu_r) x K_ground x E_fraction (the receiver's own K enters through the material's albedo factor).
// The earthshine's fill is left out (its exitance ~1e-4 of the Sun's on the lunar day; earthshine-only scenes keep their
// light unchanged). Vehicles take their ground fill from the sky's radiance cube, which holds the lunar ground.

// [S] The Sun's angular radius at the Moon (IAU 2015 B3 nominal solar radius 695 700 km over 1 au; +-1.7 % over the
// year: irrelevant for the walls' [E] visibility).
const float FILL_SUN_RADIUS_RAD = 0.0046505;

// A_h at the cosine mu (the mu grid, linear between).
float moon_fill_albedo(float mu) {
	float x = clamp(mu, 0.0, 1.0) * float(DETAIL_ALBEDO_MU_STEPS);
	int k = min(int(x), DETAIL_ALBEDO_MU_STEPS - 1);
	return mix(detail_hemispherical_albedo_mu[k], detail_hemispherical_albedo_mu[k + 1], x - float(k));
}

// The fill's emitter (review N3): the surround's hemispherical lit share at the Sun's macroscopic elevation (rad) - the
// zenith-view statistical factor with every rock statistical (Smith's G1 of the band, the rocks' statistical shadows and
// lit faces). Terrain and rocks at one place pass it (times the horizon visibility V) as ground_lit.
float moon_fill_emitter(float abundance, float spacing_m, float horizon_spacing_m, float sun_elevation) {
	return detail_sun_factor_nadir(abundance, 1.0e30, spacing_m, horizon_spacing_m, sun_elevation);
}

// The visible fraction of the Sun's disc whose centre stands margin_rad over a straight horizon (terrain_visible_fraction).
float moon_fill_disc_visible(float margin_rad) {
	float d = clamp(margin_rad / FILL_SUN_RADIUS_RAD, -1.0, 1.0);
	return clamp(0.5 + (d * sqrt(1.0 - d * d) + asin(d)) / PI, 0.0, 1.0);
}

// The fill's R term (moon_regolith_scale x E_scene x albedo x it = the radiance). n: the receiver's normal, up: the
// local vertical, to_sun: unit, mu_view: the receiver's view cosine, ground_lit: the surround's lit share
// (moon_fill_emitter x V), receiver_visible: the receiver's own Sun visibility V (horizon map), ground_k: the ground's
// albedo x scale x norm, horizon: read the node's map at uv (east, north: the local axes) for the walls.
float moon_fill_reflectance(vec3 n, vec3 up, vec3 to_sun, float mu_view, float ground_lit, float receiver_visible,
		float ground_k, bool horizon, vec3 east, vec3 north, vec2 uv) {
	float sun_up = dot(to_sun, up);
	if (moon_near_shadow.w < 0.5 || sun_up <= 0.0 || !detail_shading) {
		return 0.0;
	}
	float n_up = clamp(dot(n, up), -1.0, 1.0);
	float exitance = 0.5 * (1.0 - n_up) * sun_up * moon_fill_albedo(sun_up) * ground_lit;
	if (horizon) {
		float elevations[16];
		for (int c = 0; c < 4; c++) {
			vec4 block = textureLod(horizon_map, vec3(uv, float(c)), 0.0);
			for (int j = 0; j < 4; j++) {
				elevations[4 * c + j] = block[j];
			}
		}
		// The receiver's horizon in the Sun's azimuth (between its two sectors, as terrain_visible_fraction).
		vec3 sun_flat = to_sun - up * sun_up;
		float sun_flat_length = length(sun_flat);
		sun_flat = sun_flat_length > 1.0e-6 ? sun_flat / sun_flat_length : north;
		float sun_azimuth = atan(dot(sun_flat, east), dot(sun_flat, north));
		float sector = (sun_azimuth < 0.0 ? sun_azimuth + TAU : sun_azimuth) * (16.0 / TAU);
		int k0 = int(floor(sector)) & 15;
		float tan_occluder = tan(mix(elevations[k0], elevations[(k0 + 1) & 15], fract(sector)));
		float sun_elevation = asin(clamp(sun_up, 0.0, 1.0));
		float relief_variance = max(detail_drawn(detail_cell_m) - detail_drawn(detail_horizon_spacing_m), 0.0);
		float walls = 0.0;
		for (int k = 0; k < 16; k++) {
			float h = elevations[k];
			if (h <= 0.0) {
				continue;
			}
			float azimuth = float(k) * (TAU / 16.0);
			vec3 toward = north * cos(azimuth) + east * sin(azimuth);
			vec3 wall = up * cos(h) - toward * sin(h);
			float mu0 = dot(wall, to_sun);
			if (mu0 <= 0.0) {
				continue;
			}
			// The wall's own Sun visibility (N2): the occluder seen from the wall point, never below the receiver's. The
			// estimate is ill-conditioned for walls toward the Sun's azimuth (1 - cos dphi -> 0: tan_seen jumps on a tiny
			// tan_occluder - tan h, flickering between fragments); [E] floor 0.05 (dphi ~18 deg; Task 12 re-review 2 m1).
			float tan_seen = (tan_occluder - tan(h)) / max(1.0 - dot(toward, sun_flat), 0.05);
			float visible = max(receiver_visible, moon_fill_disc_visible(sun_elevation - atan(tan_seen)));
			// The wall's own relief shadows (Smith's G1 at its incidence over the ground's band, n1).
			float tan_wall = mu0 / sqrt(max(1.0 - mu0 * mu0, 1.0e-6));
			walls += sin(h) * sin(h) * mu0 * moon_fill_albedo(mu0) * detail_smith(tan_wall, relief_variance) * visible;
		}
		exitance += 0.5 * (1.0 + n_up) * walls / 16.0;
	}
	return moon_hapke_norm * moon_fill_albedo(mu_view) * ground_k * exitance;
}
