// The lunar regolith's reflectance, one BRDF for every light on it (the Sun, the earthshine) - moon.gdshader,
// terrain_moon.gdshader, sky.gdshader (the radiance map's lunar ground); mirrored by Atmosphere.regolith_reflectance (the
// eye adaptation's Moon in view): keep in sync. Two laws, chosen by the global moon_brdf (World.moon_brdf, set by
// Atmosphere.configure_moon_brdf):
// 1 "hapke" (default): moon_hapke_norm x Hapke's I/F (moon_hapke.gdshaderinc, the core's hapke_reflectance with
//   config/moon_surface.toml [hapke]); moon_hapke_norm = 1.35 / the uniform Hapke sphere's disc-integrated I/F at
//   phase 0, so the full-phase disc equals the Lommel-Seeliger law's (K&S's full Moon, the regolith scale's
//   calibration point) and the albedo map's mean brightness stays.
// 0 "lommel_seeliger" (the former law, bit-identical):
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
// The colour of that light (look fixes T5 fix round 1, review I3): the Sun above the atmosphere in linear Rec.709 at
// luminance 1 (Atmosphere.update: the white balance's own solar_illuminance_rgb x sun_radiance_to_luminance), so the
// post step's white balance maps a grey regolith to grey. The earthshine takes it too [call: the Earth's spectral
// albedo (bluer than the Sun) is not modelled]; the umbra glow keeps its own colour. (1, 1, 1) before the tables load.
global uniform vec3 moon_sun_color;
// The same under the earthshine (Atmosphere.earthshine_scene), without the gain.
global uniform float moon_earthshine;
// The BRDF (above): 0 Lommel-Seeliger x K&S, 1 Hapke.
global uniform int moon_brdf;
// Hapke parameters (Atmosphere.configure_moon_brdf from the core): (w, b, c, K) and (B_S0, h_S, B_C0, h_C).
global uniform vec4 moon_hapke_albedo_phase;
global uniform vec4 moon_hapke_opposition;
// The mean slope of the globe's unresolved roughness (rad; the fit's sub-pixel value). Terrain materials pass their own
// (scale-dependent) through regolith_reflectance_rough.
global uniform float moon_hapke_theta_bar_rad;
global uniform float moon_hapke_norm;

#include "res://shaders/moon_hapke.gdshaderinc"

float regolith_phase_factor(float cos_g) {
	float g_deg = min(degrees(acos(clamp(cos_g, -1.0, 1.0))), REGOLITH_PHASE_MAX_DEG);
	float g = radians(g_deg);
	float phi = g_deg < 0.01 ? 1.0 : 1.0 - sin(0.5 * g) * tan(0.5 * g) * log(1.0 / tan(0.25 * g));
	float ks = pow(10.0, -0.4 * (REGOLITH_MAG_PER_DEG * g_deg + REGOLITH_MAG_PER_DEG4 * g_deg * g_deg * g_deg * g_deg));
	float opposition = g_deg < REGOLITH_OPPOSITION_END_DEG
			? REGOLITH_OPPOSITION_PEAK - REGOLITH_OPPOSITION_PER_DEG * g_deg : 1.0;
	return opposition * ks / max(phi, 1.0e-6);
}

// The least cosine to the eye; a normal turned further away is taken back onto it.
const float REGOLITH_MU_MIN = 1.0e-3;

// LS x f for unit normal n, unit directions to the light and to the eye (0 where the light is below the horizon).
// A normal facing away from the eye (mu < REGOLITH_MU_MIN) occurs only at the silhouette - the interpolated vertex
// normals of the faceted sphere, an MSAA sample beyond its triangle: it is replaced by the limb's normal (the
// component toward the eye raised to REGOLITH_MU_MIN) before the light's cosine is taken. Otherwise a light just
// behind the limb lit the rim: in a total solar eclipse (the Sun behind the Moon, phase ~180 deg, f held at 150 deg)
// rim fragments with mu0 > 0 and mu clamped got the full sunlit Lommel-Seeliger value - white pixels around the
// eclipsed Moon (brightness-term task 2, 2026-10-25).
// theta_bar: Hapke's mean slope of the roughness not drawn as geometry (unused by Lommel-Seeliger).
float regolith_reflectance_rough(vec3 n, vec3 to_light, vec3 to_eye, float theta_bar) {
	float mu = dot(n, to_eye);
	if (mu < REGOLITH_MU_MIN) {
		n = normalize(n + to_eye * (REGOLITH_MU_MIN - mu));
		mu = REGOLITH_MU_MIN;
	}
	float mu0 = dot(n, to_light);
	if (mu0 <= 0.0) {
		return 0.0;
	}
	if (moon_brdf == 1) {
		return moon_hapke_norm * hapke_reflectance(moon_hapke_albedo_phase, moon_hapke_opposition, mu0, mu,
				dot(to_light, to_eye), theta_bar);
	}
	return 2.0 * mu0 / (mu0 + mu) * regolith_phase_factor(dot(to_light, to_eye));
}

// The same with the globe's mean slope.
float regolith_reflectance(vec3 n, vec3 to_light, vec3 to_eye) {
	return regolith_reflectance_rough(n, to_light, to_eye, moon_hapke_theta_bar_rad);
}
