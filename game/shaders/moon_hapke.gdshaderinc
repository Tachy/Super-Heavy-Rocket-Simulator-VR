// Hapke (2012) radiance factor I/F of the lunar regolith: mirror of core/src/environment/hapke.cpp (hapke_reflectance
// with the Hapke 2002 H function; keep in sync - moon_surface_check.gd renders it against the core within 2 %).
// r = K w/4 mu0e/(mu0e + mue) [p(g) (1 + B_S0 B_S(g)) + H(mu0e/K) H(mue/K) - 1] (1 + B_C0 B_C(g)) S(i, e, g), the
// isotropic multiple-scattering approximation with porosity K, shadow-hiding and coherent-backscatter opposition
// effects, the double Henyey-Greenstein phase function (c > 0 backscattering) and Hapke's (1984) macroscopic roughness
// (mean slope theta_bar). Parameters: config/moon_surface.toml [hapke] (Sato et al. 2014, LROC WAC 643 nm).
// Self-contained (no globals): moon_regolith.gdshaderinc and test shaders include it.

const float HAPKE_PI = 3.14159265358979;

// Chandrasekhar's H for isotropic scatterers, Hapke 2002 approximation (< 1 % for all w); 1 at x <= 0.
float hapke_h(float x, float w) {
	if (x <= 0.0) {
		return 1.0;
	}
	float gamma = sqrt(1.0 - w);
	float r0 = (1.0 - gamma) / (1.0 + gamma);
	return 1.0 / (1.0 - w * x * (r0 + 0.5 * (1.0 - 2.0 * r0 * x) * log((1.0 + x) / x)));
}

// Double Henyey-Greenstein p(g) from cos g.
float hapke_phase(float b, float c, float cos_g) {
	float b2 = b * b;
	float back = (1.0 - b2) / pow(1.0 - 2.0 * b * cos_g + b2, 1.5);
	float forward = (1.0 - b2) / pow(1.0 + 2.0 * b * cos_g + b2, 1.5);
	return 0.5 * (1.0 + c) * back + 0.5 * (1.0 - c) * forward;
}

// Roughness exponents E1(x) = exp(-2 / (pi tan(theta) tan x)), E2(x) = exp(-1 / (pi tan^2(theta) tan^2 x)) from sin x,
// cos x (0 at x = 0).
float hapke_e1(float tan_t, float sin_x, float cos_x) {
	return sin_x < 1.0e-6 ? 0.0 : exp(-2.0 * cos_x / (HAPKE_PI * tan_t * sin_x));
}

float hapke_e2(float tan_t, float sin_x, float cos_x) {
	float r = cos_x / (tan_t * sin_x);
	return sin_x < 1.0e-6 ? 0.0 : exp(-r * r / HAPKE_PI);
}

// I/F. albedo_phase = (w, b, c, K), opposition = (B_S0, h_S, B_C0, h_C); mu0, mu: cosines of incidence and emission
// (0 unless both > 0), cos_g: cosine of the phase angle, theta_bar: the mean slope (rad; 0: smooth).
float hapke_reflectance(vec4 albedo_phase, vec4 opposition, float mu0, float mu, float cos_g, float theta_bar) {
	if (mu0 <= 0.0 || mu <= 0.0) {
		return 0.0;
	}
	mu0 = min(mu0, 1.0);
	mu = min(mu, 1.0);
	cos_g = clamp(cos_g, -1.0, 1.0);
	float mu0e = mu0;
	float mue = mu;
	float shadowing = 1.0;
	if (theta_bar > 0.0) {
		float tan_t = tan(theta_bar);
		float chi = 1.0 / sqrt(1.0 + HAPKE_PI * tan_t * tan_t);
		float sin_i = sqrt(max(0.0, 1.0 - mu0 * mu0));
		float sin_e = sqrt(max(0.0, 1.0 - mu * mu));
		float cos_psi = 1.0;
		if (sin_i * sin_e > 1.0e-6) {
			cos_psi = clamp((cos_g - mu0 * mu) / (sin_i * sin_e), -1.0, 1.0);
		}
		float psi_fraction = acos(cos_psi) / HAPKE_PI;
		// f(psi) = exp(-2 tan(psi/2)), tan(psi/2) = sqrt((1 - cos psi) / (1 + cos psi)); 0 at psi = pi.
		float f = 1.0 + cos_psi < 1.0e-6 ? 0.0 : exp(-2.0 * sqrt((1.0 - cos_psi) / (1.0 + cos_psi)));
		float sin2_half = 0.5 * (1.0 - cos_psi);
		float e1_i = hapke_e1(tan_t, sin_i, mu0);
		float e2_i = hapke_e2(tan_t, sin_i, mu0);
		float e1_e = hapke_e1(tan_t, sin_e, mu);
		float e2_e = hapke_e2(tan_t, sin_e, mu);
		float eta_i = chi * (mu0 + sin_i * tan_t * e2_i / (2.0 - e1_i));
		float eta_e = chi * (mu + sin_e * tan_t * e2_e / (2.0 - e1_e));
		if (mu0 >= mu) {  // i <= e
			float den = 2.0 - e1_e - psi_fraction * e1_i;
			mu0e = chi * (mu0 + sin_i * tan_t * (cos_psi * e2_e + sin2_half * e2_i) / den);
			mue = chi * (mu + sin_e * tan_t * (e2_e - sin2_half * e2_i) / den);
			shadowing = mue / eta_e * mu0 / eta_i * chi / (1.0 - f + f * chi * mu0 / eta_i);
		} else {
			float den = 2.0 - e1_i - psi_fraction * e1_e;
			mu0e = chi * (mu0 + sin_i * tan_t * (e2_i - sin2_half * e2_e) / den);
			mue = chi * (mu + sin_e * tan_t * (cos_psi * e2_i + sin2_half * e2_e) / den);
			shadowing = mue / eta_e * mu0 / eta_i * chi / (1.0 - f + f * chi * mu / eta_e);
		}
	}
	float w = albedo_phase.x;
	float k = albedo_phase.w;
	// tan(g/2) = sqrt((1 - cos g) / (1 + cos g)).
	float tan_half_g = sqrt((1.0 - cos_g) / max(1.0 + cos_g, 1.0e-12));
	float shoe = opposition.y > 0.0 ? 1.0 + opposition.x / (1.0 + tan_half_g / opposition.y) : 1.0;
	float cboe = 1.0;
	if (opposition.w > 0.0) {
		float z = tan_half_g / opposition.w;
		float ratio = z < 1.0e-4 ? 1.0 - 0.5 * z : (1.0 - exp(-z)) / z;
		cboe = 1.0 + opposition.z * (1.0 + ratio) / (2.0 * (1.0 + z) * (1.0 + z));
	}
	float multiple = hapke_h(mu0e / k, w) * hapke_h(mue / k, w) - 1.0;
	return k * 0.25 * w * mu0e / (mu0e + mue) * (hapke_phase(albedo_phase.y, albedo_phase.z, cos_g) * shoe + multiple)
			* cboe * shadowing;
}
