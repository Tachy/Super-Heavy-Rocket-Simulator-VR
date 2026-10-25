// The lunar regolith's reflectance, one BRDF for every light on it (the Sun, the earthshine) - moon.gdshader,
// terrain_moon.gdshader; mirrored by Atmosphere.regolith_reflectance (the eye adaptation's Moon in view): keep in sync.
// Scene value = albedo x moon_regolith_scale x E x LS(mu0, mu) x f(g), E the light's scene value on a white Lambertian
// at normal incidence (moon_sun_scene for the Sun, moon_earthshine for the earthshine), LS = 2 mu0 / (mu0 + mu)
// Lommel-Seeliger (1 at mu0 = mu) and f(g) the phase correction at the local phase angle g (light - surface - eye) that
// makes the disc-integrated Moon follow Krisciunas & Schaefer 1991 (PASP 103, 1033) Eq. 9 with the opposition surge:
//     f(g) = O(g) x 10^(-0.4 (0.026 g + 4e-9 g^4)) / Phi_LS(g),  O(g) = 1.35 - 0.05 g below 7 deg, else 1,
// Phi_LS(g) = 1 - sin(g/2) tan(g/2) ln(cot(g/4)) the disc-integrated phase function of a Lommel-Seeliger sphere
// (checked numerically by tools/validation/moon_disc_calibration.py). moon_regolith_scale: the full-phase disc of the
// albedo texture's near side gives K&S's 0.313 lux (Atmosphere.regolith_scale; night-vision fix round I3, I4).

// [S: K&S 1991 Eq. 9] magnitude per degree of phase and per degree^4.
const float REGOLITH_MAG_PER_DEG = 0.026;
const float REGOLITH_MAG_PER_DEG4 = 4.0e-9;
// [S: K&S 1991 sec. 2, as Schaefer's skycalc applies it] opposition surge below 7 deg.
const float REGOLITH_OPPOSITION_PEAK = 1.35;
const float REGOLITH_OPPOSITION_PER_DEG = 0.05;
const float REGOLITH_OPPOSITION_END_DEG = 7.0;
// [call] The phase law is held beyond this angle (K&S's quartic turns up past ~150 deg, outside its data).
const float REGOLITH_PHASE_MAX_DEG = 150.0;

global uniform float moon_regolith_scale;
// A white Lambertian at normal incidence under the Sun at its distance (DAY_EXPOSURE / d_AU^2), without the gain.
global uniform float moon_sun_scene;
// The same under the earthshine (Atmosphere.earthshine_scene), without the gain.
global uniform float moon_earthshine;

float regolith_phase_factor(float cos_g) {
	float g_deg = min(degrees(acos(clamp(cos_g, -1.0, 1.0))), REGOLITH_PHASE_MAX_DEG);
	float g = radians(g_deg);
	float phi = g_deg < 0.01 ? 1.0 : 1.0 - sin(0.5 * g) * tan(0.5 * g) * log(1.0 / tan(0.25 * g));
	float ks = pow(10.0, -0.4 * (REGOLITH_MAG_PER_DEG * g_deg + REGOLITH_MAG_PER_DEG4 * g_deg * g_deg * g_deg * g_deg));
	float opposition = g_deg < REGOLITH_OPPOSITION_END_DEG
			? REGOLITH_OPPOSITION_PEAK - REGOLITH_OPPOSITION_PER_DEG * g_deg : 1.0;
	return opposition * ks / max(phi, 1.0e-6);
}

// LS x f for unit normal n, unit directions to the light and to the eye (0 where the light is below the horizon).
float regolith_reflectance(vec3 n, vec3 to_light, vec3 to_eye) {
	float mu0 = dot(n, to_light);
	if (mu0 <= 0.0) {
		return 0.0;
	}
	float mu = max(dot(n, to_eye), 1.0e-3);
	return 2.0 * mu0 / (mu0 + mu) * regolith_phase_factor(dot(to_light, to_eye));
}
