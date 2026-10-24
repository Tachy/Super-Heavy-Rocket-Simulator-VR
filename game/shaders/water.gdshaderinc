// Calm sea shading for the Earth terrain (terrain_earth.gdshader): the geometry stays at the geoid (the physics' sea,
// no displacement); only the shading normal carries waves. Radiance in the land's units (albedo * cos_sun *
// day_exposure, i.e. a solar irradiance of pi * day_exposure): body colour lit like a diffuse surface, plus the sky
// reflected by Fresnel, plus the sun glint of a Cox-Munk slope distribution around the wave-perturbed normal.
// The wave set (vectors, phases at the camera) comes from WaterWaves (world/terrain/water_waves.gd) via the globals.

// Columns: wave vector (rad/m, body-local render axes) in xyz, slope k A in w. Phases: each wave's phase at the camera.
global uniform mat4 water_wave_vectors;
global uniform vec4 water_wave_phases;

// [E] Open-ocean body colour; open-ocean albedo ~0.06 [S: Payne 1972, J. Atmos. Sci. 29].
uniform vec3 deep_water = vec3(0.02, 0.05, 0.08);
// [E] Placeholder sky radiance reflected by the sea until the atmosphere part of M8 replaces it (clear-sky blue).
uniform vec3 sky_radiance = vec3(0.25, 0.35, 0.5);
// [E] Wind speed of the glint's slope distribution (m/s at 12.5 m).
uniform float glint_wind_mps = 5.0;

const float WATER_REFRACTIVE_INDEX = 1.333;  // [S] sea water, visible
// [S: Cox & Munk 1954, J. Opt. Soc. Am. 44] mean square slope sigma^2 = 0.003 + 0.00512 U (clean surface).
const float COX_MUNK_SIGMA2_0 = 0.003;
const float COX_MUNK_SIGMA2_PER_MPS = 0.00512;
// [E] The wave normals fade to the flat normal between these camera distances (no aliasing far away).
const float WAVE_FADE_START_M = 10000.0;
const float WAVE_FADE_END_M = 20000.0;
// [E] An octave also fades out where its phase changes by a quarter to half a cycle per pixel.
const float WAVE_ALIAS_START_RAD = 1.5707963;
const float WAVE_ALIAS_END_RAD = 3.1415927;
// [E] Gerstner steepness Q (GPU Gems 1, ch. 1), shared by the octaves: Q_i k_i A_i = Q / octaves.
const float GERSTNER_STEEPNESS = 0.5;
const int WAVE_OCTAVES = 4;
// [E] Mask threshold and half width of the coast transition (mask units: one texel across with linear filtering).
const float WATER_MASK_THRESHOLD = 0.5;
const float WATER_MASK_HALF_WIDTH = 0.5;
// [E] Lower bound of the viewing / sun cosines in the glint (numerical guard only: the Smith term already makes the
// grazing limit finite).
const float GLINT_MIN_COS = 1.0e-3;
// [E] Sun elevation (sine) over which glint and sky fade in at sunrise.
const float GLINT_SUN_FADE = 0.02;
const float SKY_SUN_FADE = 0.1;

// Water fraction of a fragment from the linearly filtered per-node mask (255 = water).
float water_coverage(float mask) {
	return smoothstep(WATER_MASK_THRESHOLD - WATER_MASK_HALF_WIDTH, WATER_MASK_THRESHOLD + WATER_MASK_HALF_WIDTH, mask);
}

float water_fresnel(float cos_theta) {
	float r0 = (WATER_REFRACTIVE_INDEX - 1.0) / (WATER_REFRACTIVE_INDEX + 1.0);
	r0 *= r0;
	return r0 + (1.0 - r0) * pow(1.0 - clamp(cos_theta, 0.0, 1.0), 5.0);
}

// Gerstner normal of the wave octaves at a fragment offset from the camera (body-local render metres), around the flat
// sea normal n. The phase field is k . x: its gradient along the surface is k's horizontal part.
vec3 water_wave_normal(vec3 n, vec3 offset_from_camera, float camera_distance) {
	float distance_fade = 1.0 - smoothstep(WAVE_FADE_START_M, WAVE_FADE_END_M, camera_distance);
	vec3 tilt = vec3(0.0);
	float lift = 0.0;
	for (int i = 0; i < WAVE_OCTAVES; i++) {
		vec4 wave = water_wave_vectors[i];
		float phase = water_wave_phases[i] + dot(wave.xyz, offset_from_camera);
		float fade = distance_fade * (1.0 - smoothstep(WAVE_ALIAS_START_RAD, WAVE_ALIAS_END_RAD, fwidth(phase)));
		vec3 horizontal = wave.xyz - dot(wave.xyz, n) * n;
		tilt += horizontal / length(wave.xyz) * (wave.w * cos(phase) * fade);
		lift += GERSTNER_STEEPNESS / float(WAVE_OCTAVES) * sin(phase) * fade;
	}
	return normalize(n * (1.0 - lift) - tilt);
}

// Smith Lambda of the Cox-Munk (Beckmann, roughness^2 = sigma2) slopes seen at cosine cos_theta from the mean normal:
// rational approximation of Walter et al. 2007 (EGSR, eq. 27). Masks and shadows the glint at grazing angles.
float cox_munk_smith_lambda(float cos_theta, float sigma2) {
	float c = max(cos_theta, GLINT_MIN_COS);
	float a = c / sqrt(sigma2 * (1.0 - c * c) + 1.0e-12);
	if (a >= 1.6) {
		return 0.0;
	}
	return (1.0 - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a);
}

// Radiance of the sea. body: body colour; n: flat sea normal; wave_n: water_wave_normal; to_sun, to_camera: unit
// vectors; sunlit: unocculted fraction of the Sun (eclipse); exposure: the land's day_exposure.
vec3 water_radiance(vec3 body, vec3 n, vec3 wave_n, vec3 to_sun, vec3 to_camera, float sunlit, float exposure) {
	float sun_up = dot(n, to_sun);
	float fresnel_view = water_fresnel(dot(wave_n, to_camera));
	vec3 diffuse = body * max(sun_up, 0.0) * sunlit * exposure * (1.0 - fresnel_view);
	vec3 sky = sky_radiance * fresnel_view * smoothstep(-SKY_SUN_FADE, SKY_SUN_FADE, sun_up) * sunlit;

	// Cox-Munk glint: BRDF F p(tan beta) G / (4 cos_s cos_v cos^4 beta), p = exp(-tan^2 beta / s2) / (pi s2), G the
	// Smith masking-shadowing; radiance = BRDF * pi * exposure * cos_s.
	float sigma2 = COX_MUNK_SIGMA2_0 + COX_MUNK_SIGMA2_PER_MPS * glint_wind_mps;
	vec3 half_vector = normalize(to_sun + to_camera);
	float cos_beta = max(dot(half_vector, wave_n), GLINT_MIN_COS);
	float cos2_beta = cos_beta * cos_beta;
	float tan2_beta = (1.0 - cos2_beta) / cos2_beta;
	float slope_pdf = exp(-tan2_beta / sigma2) / (PI * sigma2);
	float cos_view = dot(wave_n, to_camera);
	float cos_sun = dot(wave_n, to_sun);
	float smith = 1.0 / (1.0 + cox_munk_smith_lambda(cos_view, sigma2) + cox_munk_smith_lambda(cos_sun, sigma2));
	float glint = PI * exposure * water_fresnel(dot(to_camera, half_vector)) * slope_pdf * smith
			/ (4.0 * max(cos_view, GLINT_MIN_COS) * cos2_beta * cos2_beta);
	glint *= smoothstep(0.0, GLINT_SUN_FADE, sun_up) * sunlit;  // none at night or in the Moon's shadow
	return diffuse + sky + vec3(glint);
}
