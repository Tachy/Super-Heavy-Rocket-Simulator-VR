// Reflectance of a drawn lunar rock (moon procedural surface plan Task 11): moon_rock.gdshader and the test patch
// game/tests/moon_rock_patch.gdshader. Needs moon_regolith.gdshaderinc and moon_detail.gdshaderinc first.
//
// The rock is the statistical layer's rock drawn (moon_detail.gdshaderinc, core unresolved_light_factors): Hapke with
// the single-scattering albedo albedo_factor x w (config rocks.albedo_to_regolith; the statistical layer's reflectance
// ratio rho is this law's ratio at the mosaics' standard geometry) and the globe's mean slope (rho's definition), on
// the albedo map's value divided by the mosaic norm N = exp(-C_all) + (1 - exp(-C_all)) rho (the map holds every rock,
// like the ground's factors), each light's term (Sun and earthshine) times Smith's G1 of the node's unshadowed drawn
// relief at that light's elevation (the ground's own factor: the model's drawn rock radiance is rho G1(e) G(e) / N per
// plan area). Lit faces and the terminator come from the
// drawn shape itself. The rock texture's slopes modulate each light like the regolith texture (mean-preserving
// 1 + tanh(k . grad h)). At the base the rock blends into the regolith over ROCK_FILLET_PER_DIAMETER x D (dust fillet,
// burial blend): the regolith's reflectance on the local vertical with the node's residual mean slope.

// [E] Height of the dust fillet / burial blend at a rock's base per rock diameter (look; no lunar measurement).
const float ROCK_FILLET_PER_DIAMETER = 0.05;
// [D] Single-scattering albedo cap of the rock law (Hapke's H function needs w < 1).
const float ROCK_W_MAX = 0.999;

// The mosaic norm N at the abundance factor.
float moon_rock_norm(float abundance) {
	float c_all = abundance * detail_rock_cover(detail_rock_law.z, detail_rock_law.w);
	return exp(-c_all) + (1.0 - exp(-c_all)) * detail_rock_shape.y;
}

// Smith's G1 of the node's drawn relief below the horizon grid (the ground's own factor), Sun at macroscopic elevation.
float moon_rock_lit(float elevation, float spacing_m, float horizon_spacing_m) {
	if (elevation <= 0.0) {
		return 0.0;
	}
	return detail_smith(tan(elevation), max(detail_drawn(spacing_m) - detail_drawn(horizon_spacing_m), 0.0));
}

// Reflectance of the rock surface (unit normal n) for one light, the fillet blend toward the regolith (up: the local
// vertical, ground_theta_bar: the node's residual mean slope; fillet 1: bare rock, 0: regolith), the texture gradient's
// modulation included. Units of regolith_reflectance_rough.
float moon_rock_reflectance(vec3 n, vec3 up, vec3 to_light, vec3 to_eye, float albedo_factor, vec3 gradient,
		float fillet, float ground_theta_bar) {
	float rock = 0.0;
	if (moon_brdf == 1) {
		float mu = dot(n, to_eye);
		if (mu < REGOLITH_MU_MIN) {  // the silhouette's interpolated normal (regolith_reflectance_rough's rule)
			n = normalize(n + to_eye * (REGOLITH_MU_MIN - mu));
			mu = REGOLITH_MU_MIN;
		}
		vec4 albedo_phase = moon_hapke_albedo_phase;
		albedo_phase.x = min(albedo_phase.x * albedo_factor, ROCK_W_MAX);
		rock = moon_hapke_norm * hapke_reflectance(albedo_phase, moon_hapke_opposition, dot(n, to_light), mu,
				dot(to_light, to_eye), moon_hapke_theta_bar_rad);
	} else {  // Lommel-Seeliger: the regolith law times the statistical layer's ratio
		rock = regolith_reflectance_rough(n, to_light, to_eye, moon_hapke_theta_bar_rad) * detail_rock_shape.y;
	}
	rock *= detail_modulation(gradient, n, to_light, to_eye);
	if (fillet >= 1.0) {
		return rock;
	}
	float ground = regolith_reflectance_rough(up, to_light, to_eye, ground_theta_bar);
	return mix(ground, rock, fillet);
}
