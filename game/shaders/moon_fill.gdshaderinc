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
// surround), with the flat ground's exitance (the statistical lit share f_lit at the Sun's macroscopic elevation, the
// receiver's own horizon visibility standing for its surround's); plus the terrain above the horizon from the node's
// horizon map: for a horizontal receiver the view factor of terrain up to elevation h(phi) is the azimuth mean of
// sin^2 h, each sector's wall lit at its own incidence (normal tilted by h toward the receiver, [E] the wall's slope
// taken as its horizon elevation), weighted (1 + n.up) / 2 for a tilted receiver ([E]).
// In the materials' units (radiance = albedo x moon_regolith_scale x E_scene x R, R = moon_hapke_norm x I/F): the
// render's own I/F of the ground is K x I/F_Hapke(w), K = albedo x scale x norm, so the fill R term is
// norm x A_h(mu_r) x K_ground x E_fraction (the receiver's own K enters through the material's albedo factor).
// The earthshine's fill is left out (its exitance ~1e-4 of the Sun's on the lunar day; earthshine-only scenes keep their
// light unchanged). Vehicles take their ground fill from the sky's radiance cube, which holds the lunar ground.

// A_h at elevation e (the gain grid, linear between).
float moon_fill_albedo(float elevation) {
	float x = clamp(elevation / detail_rock_shape.z, 0.0, float(DETAIL_GAIN_SAMPLES - 1));
	int k = min(int(x), DETAIL_GAIN_SAMPLES - 2);
	return mix(detail_hemispherical_albedo[k], detail_hemispherical_albedo[k + 1], x - float(k));
}

// The fill's R term (moon_regolith_scale x E_scene x albedo x it = the radiance). n: the receiver's normal, up: the
// local vertical, to_sun: unit, mu_view: the receiver's view cosine, ground_lit: the surround's lit share (statistical
// factor x horizon visibility), ground_k: the ground's albedo x scale x norm, horizon: read the node's map at uv (east,
// north: the local axes) for the walls.
float moon_fill_reflectance(vec3 n, vec3 up, vec3 to_sun, float mu_view, float ground_lit, float ground_k, bool horizon,
		vec3 east, vec3 north, vec2 uv) {
	float sun_up = dot(to_sun, up);
	if (moon_near_shadow.w < 0.5 || sun_up <= 0.0 || !detail_shading) {
		return 0.0;
	}
	float n_up = clamp(dot(n, up), -1.0, 1.0);
	float exitance = 0.5 * (1.0 - n_up) * sun_up * moon_fill_albedo(asin(sun_up)) * ground_lit;
	if (horizon) {
		float walls = 0.0;
		for (int c = 0; c < 4; c++) {
			vec4 block = textureLod(horizon_map, vec3(uv, float(c)), 0.0);
			for (int j = 0; j < 4; j++) {
				float h = block[j];
				if (h > 0.0) {
					float azimuth = float(4 * c + j) * (TAU / 16.0);
					vec3 toward = north * cos(azimuth) + east * sin(azimuth);
					vec3 wall = up * cos(h) - toward * sin(h);
					float mu0 = dot(wall, to_sun);
					if (mu0 > 0.0) {
						walls += sin(h) * sin(h) * mu0 * moon_fill_albedo(asin(min(mu0, 1.0)));
					}
				}
			}
		}
		exitance += 0.5 * (1.0 + n_up) * walls / 16.0;
	}
	return moon_hapke_norm * moon_fill_albedo(asin(clamp(mu_view, 0.0, 1.0))) * ground_k * exitance;
}
