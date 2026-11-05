// Procedural cm-scale surface of a drawn lunar rock and its dust (moon procedural surface plan Task 17; user
// 2026-11-05: the rocks looked like crystals with flat, evenly lit faces). Used by moon_rock.gdshader.
//
// Rock texture: a height field of value noise (with analytic gradient) in the rock's own metres - the unit-template
// position x the rock's scale, so it moves with the rock and is stable under the floating origin - at five octaves
// from 25 cm to ~1 cm: its gradient tilts the shading normal (faces no longer evenly lit: granular, pitted,
// crystalline-rough surface), a crack octave darkens thin fracture lines, the finest octaves vary the albedo (grains).
// Octaves finer than ~4 pixels fade out (no aliasing). The slopes are zero-mean (texture slopes stay in the rock law's
// theta_bar, ruling T10-C2: they cast no shadows) and the albedo variation mean-preserving to first order.
//
// Dust: regolith settles on up-facing parts and in concavities (Apollo 17 station 6 / 7 boulders: lighter dusty tops,
// filled cracks and hollows). Coverage = ROCK_DUST_MAX x patchiness x clamp(up-facing term + cavity term); dusted
// surface reflects like the regolith (single-scattering albedo factor 1, no rock texture or slopes). The rock law's
// albedo factor (config rocks.albedo_to_regolith: boulders as seen from orbit, dust included) stays the rock's MEAN
// factor: the bare rock is brighter by 1 / (1 - ROCK_DUST_MEAN) on the excess, so the drawn rock's mean - and the
// statistical layer's ratio rho that mirrors it - are unchanged.

// [E] Coarsest octave's wavelength (m) and the ratio between octaves: 25, 11, 5.2, 2.4, 1.1 cm.
const float ROCK_DETAIL_WAVELENGTH_M = 0.25;
const float ROCK_DETAIL_LACUNARITY = 2.2;
const int ROCK_DETAIL_OCTAVES = 5;
// [E] Slope of the coarsest octave's height field (rms of the noise gradient x this); finer octaves steeper as a
// self-affine surface with H = 0.8 (slope ~ wavelength^(H - 1): x1.87 at 1.1 cm), ~0.25 rms over the five octaves, a
// rough fractured surface at cm scale (Apollo close-up photos of boulders, e.g. AS17-140-21497; no measured cm slope of lunar rock
// faces found).
const float ROCK_DETAIL_SLOPE = 0.08;
const float ROCK_DETAIL_HURST = 0.8;
// [D] rms of the value noise's gradient in noise units (|grad| of the quintic value noise of uniform [-1, 1] corners,
// sampled): turns the per-octave slope into a gradient scale.
const float ROCK_NOISE_GRADIENT_RMS = 1.1;
// [E] Grain albedo variation (relative, per fine octave) and the crack lines: wavelength, half width (noise units),
// darkening.
const float ROCK_GRAIN_ALBEDO = 0.08;
const float ROCK_CRACK_WAVELENGTH_M = 0.35;
const float ROCK_CRACK_WIDTH = 0.05;
const float ROCK_CRACK_DARKENING = 0.45;
// [E] An octave fades out below this many pixels per wavelength.
const float ROCK_DETAIL_MIN_PIXELS = 4.0;
// [E] Dust: up-facing term from the template normal's vertical component (smoothstep between the two), cavity term
// (core TemplateMesh::cavity), patchiness wavelength (coverage 0.5-1 of the maximum: tops dusted throughout, no
// sharp-edged caps), the maximum coverage.
const float ROCK_DUST_UP_LOW = 0.6;
const float ROCK_DUST_UP_HIGH = 0.95;
const float ROCK_DUST_CAVITY = 1.0;
const float ROCK_DUST_PATCH_M = 0.4;
const float ROCK_DUST_MAX = 0.8;
// [D] Mean dust coverage of the templates' exposed plan area (core test "Dust share of the rock templates": the
// formula above with the patchiness mean 0.75, plan-area weighted over every template at the mean burial).
const float ROCK_DUST_MEAN = 0.343;

float rock_hash(vec3 p) {
	p = fract(p * 0.3183099 + vec3(0.71, 0.113, 0.419));
	p *= 17.0;
	return fract(p.x * p.y * p.z * (p.x + p.y + p.z)) * 2.0 - 1.0;
}

// Value noise in [-1, 1] and its gradient (quintic interpolation; I. Quilez, "value noise derivatives").
vec4 rock_noise(vec3 x) {
	vec3 i = floor(x);
	vec3 f = x - i;
	vec3 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
	vec3 du = 30.0 * f * f * (f * (f - 2.0) + 1.0);
	float a = rock_hash(i);
	float b = rock_hash(i + vec3(1.0, 0.0, 0.0));
	float c = rock_hash(i + vec3(0.0, 1.0, 0.0));
	float d = rock_hash(i + vec3(1.0, 1.0, 0.0));
	float e = rock_hash(i + vec3(0.0, 0.0, 1.0));
	float g = rock_hash(i + vec3(1.0, 0.0, 1.0));
	float h = rock_hash(i + vec3(0.0, 1.0, 1.0));
	float k = rock_hash(i + vec3(1.0, 1.0, 1.0));
	float k1 = b - a;
	float k2 = c - a;
	float k3 = e - a;
	float k4 = a - b - c + d;
	float k5 = a - c - e + h;
	float k6 = a - b - e + g;
	float k7 = -a + b + c - d + e - g - h + k;
	float value = a + k1 * u.x + k2 * u.y + k3 * u.z + k4 * u.x * u.y + k5 * u.y * u.z + k6 * u.z * u.x
			+ k7 * u.x * u.y * u.z;
	vec3 gradient = du * vec3(k1 + k4 * u.y + k6 * u.z + k7 * u.y * u.z,
			k2 + k5 * u.z + k4 * u.x + k7 * u.z * u.x,
			k3 + k6 * u.x + k5 * u.y + k7 * u.x * u.y);
	return vec4(gradient, value);
}

// The rock's surface detail at p_m (rock-local metres; pixel_m: the pixel's footprint there): xyz = height gradient
// (per metre, the rock's template frame), w = relative albedo (mean ~1).
vec4 rock_surface_detail(vec3 p_m, float pixel_m) {
	vec3 gradient = vec3(0.0);
	float albedo = 1.0;
	float wavelength = ROCK_DETAIL_WAVELENGTH_M;
	for (int o = 0; o < ROCK_DETAIL_OCTAVES; o++) {
		float fade = clamp(wavelength / (ROCK_DETAIL_MIN_PIXELS * pixel_m) - 1.0, 0.0, 1.0);
		if (fade <= 0.0) {
			break;
		}
		vec4 n = rock_noise(p_m / wavelength + float(o) * vec3(17.13, 31.71, 7.37));
		float slope = ROCK_DETAIL_SLOPE * pow(ROCK_DETAIL_WAVELENGTH_M / wavelength, 1.0 - ROCK_DETAIL_HURST);
		gradient += fade * (slope / ROCK_NOISE_GRADIENT_RMS) * n.xyz;
		if (o >= 2) {
			albedo += fade * ROCK_GRAIN_ALBEDO * n.w;
		}
		wavelength /= ROCK_DETAIL_LACUNARITY;
	}
	// Crack lines: the zero set of a noise field, ROCK_CRACK_WIDTH wide in noise units, darker (shadowed fissures).
	float crack_fade = clamp(ROCK_CRACK_WAVELENGTH_M * ROCK_CRACK_WIDTH / (pixel_m) - 1.0, 0.0, 1.0);
	if (crack_fade > 0.0) {
		vec4 c = rock_noise(p_m / ROCK_CRACK_WAVELENGTH_M + vec3(3.7, 11.3, 5.9));
		float line = 1.0 - smoothstep(0.0, ROCK_CRACK_WIDTH * max(length(c.xyz), 0.2), abs(c.w));
		albedo *= 1.0 - crack_fade * ROCK_CRACK_DARKENING * line;
	}
	return vec4(gradient, albedo);
}

// Dust coverage 0..1 from the template normal's vertical component, the cavity term and the patchiness at p_m.
float rock_dust(float normal_z, float cavity, vec3 p_m) {
	float settle = clamp(smoothstep(ROCK_DUST_UP_LOW, ROCK_DUST_UP_HIGH, normal_z) + ROCK_DUST_CAVITY * cavity, 0.0, 1.0);
	float patch = 0.5 + 0.5 * smoothstep(-1.0, 1.0, rock_noise(p_m / ROCK_DUST_PATCH_M + vec3(23.1, 2.9, 13.7)).w);
	return ROCK_DUST_MAX * patch * settle;
}

// The bare rock's albedo factor for the mean factor `mean_factor` (the rock law's): its excess over the regolith
// raised so that the dust-covered mean stays the law's.
float rock_bare_albedo_factor(float mean_factor) {
	return 1.0 + (mean_factor - 1.0) / (1.0 - ROCK_DUST_MEAN);
}
