#include "starsim/environment/hapke.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

using constants::pi;

// [call] tan x below which a roughness exponent is taken as exp(-infinity) = 0 (x = 0: normal incidence / emission).
constexpr double tan_min = 1e-12;
// [call] sin i sin e below which the azimuth between the incidence and emission planes is undefined (taken 0).
constexpr double azimuth_sin_min = 1e-12;

// Gauss-Legendre nodes and weights on [-1, 1] (Newton on P_n; Numerical Recipes gauleg).
void gauss_legendre(int n, std::vector<double>& x, std::vector<double>& w) {
    x.assign(static_cast<std::size_t>(n), 0.0);
    w.assign(static_cast<std::size_t>(n), 0.0);
    const int m = (n + 1) / 2;
    for (int i = 0; i < m; ++i) {
        double z = std::cos(pi * (i + 0.75) / (n + 0.5));
        double pp = 0.0;
        for (int iteration = 0; iteration < 100; ++iteration) {
            double p1 = 1.0, p2 = 0.0;
            for (int j = 0; j < n; ++j) {
                const double p3 = p2;
                p2 = p1;
                p1 = ((2.0 * j + 1.0) * z * p2 - j * p3) / (j + 1.0);
            }
            pp = n * (z * p1 - p2) / (z * z - 1.0);
            const double dz = p1 / pp;
            z -= dz;
            if (std::abs(dz) < 1e-15) break;
        }
        x[static_cast<std::size_t>(i)] = -z;
        x[static_cast<std::size_t>(n - 1 - i)] = z;
        const double weight = 2.0 / ((1.0 - z * z) * pp * pp);
        w[static_cast<std::size_t>(i)] = weight;
        w[static_cast<std::size_t>(n - 1 - i)] = weight;
    }
}

// SHOE B_S(g) = 1 / (1 + tan(g/2) / h_S) (Hapke 2012); h_S = 0: no surge.
double shadow_hiding(double phase_rad, double hs) {
    if (hs <= 0.0) return 0.0;
    return 1.0 / (1.0 + std::tan(0.5 * phase_rad) / hs);
}

// CBOE B_C(g) = (1 + (1 - exp(-z)) / z) / (2 (1 + z)^2), z = tan(g/2) / h_C (Hapke 2012); 1 at g = 0.
double coherent_backscatter(double phase_rad, double hc) {
    if (hc <= 0.0) return 0.0;
    const double z = std::tan(0.5 * phase_rad) / hc;
    const double ratio = z < 1e-8 ? 1.0 - 0.5 * z : -std::expm1(-z) / z;
    return (1.0 + ratio) / (2.0 * (1.0 + z) * (1.0 + z));
}

}  // namespace

double hapke_h_function(double x, double w, HapkeHFunction form) {
    if (x <= 0.0) return 1.0;
    const double gamma = std::sqrt(1.0 - w);
    if (form == HapkeHFunction::hapke_1981) return (1.0 + 2.0 * x) / (1.0 + 2.0 * gamma * x);
    const double r0 = (1.0 - gamma) / (1.0 + gamma);
    return 1.0 / (1.0 - w * x * (r0 + 0.5 * (1.0 - 2.0 * r0 * x) * std::log((1.0 + x) / x)));
}

double hapke_phase_function(double b, double c, double phase_rad) {
    const double cos_g = std::cos(phase_rad);
    const double b2 = b * b;
    const double back = (1.0 - b2) / std::pow(1.0 - 2.0 * b * cos_g + b2, 1.5);
    const double forward = (1.0 - b2) / std::pow(1.0 + 2.0 * b * cos_g + b2, 1.5);
    return 0.5 * (1.0 + c) * back + 0.5 * (1.0 - c) * forward;
}

HapkeRoughness hapke_roughness(double mu0, double mu, double phase_rad, double theta_bar_rad) {
    HapkeRoughness out{mu0, mu, 1.0};
    if (theta_bar_rad <= 0.0) return out;
    const double tan_t = std::tan(theta_bar_rad);
    const double chi = 1.0 / std::sqrt(1.0 + pi * tan_t * tan_t);
    const double sin_i = std::sqrt(std::max(0.0, 1.0 - mu0 * mu0));
    const double sin_e = std::sqrt(std::max(0.0, 1.0 - mu * mu));
    // Azimuth psi between the planes of incidence and emission.
    double cos_psi = 1.0;
    if (sin_i * sin_e > azimuth_sin_min) {
        cos_psi = std::clamp((std::cos(phase_rad) - mu0 * mu) / (sin_i * sin_e), -1.0, 1.0);
    }
    const double psi = std::acos(cos_psi);
    const double half = 0.5 * psi;
    const double f = half >= 0.5 * pi - 1e-12 ? 0.0 : std::exp(-2.0 * std::tan(half));
    const double sin2_half = std::sin(half) * std::sin(half);
    // E1(x) = exp(-2 / (pi tan(theta) tan x)), E2(x) = exp(-1 / (pi tan^2(theta) tan^2 x)); tan x = sin x / cos x.
    auto e1 = [&](double sin_x, double cos_x) {
        const double tan_x = sin_x / std::max(cos_x, 1e-300);
        return tan_x < tan_min ? 0.0 : std::exp(-2.0 / (pi * tan_t * tan_x));
    };
    auto e2 = [&](double sin_x, double cos_x) {
        const double tan_x = sin_x / std::max(cos_x, 1e-300);
        return tan_x < tan_min ? 0.0 : std::exp(-1.0 / (pi * tan_t * tan_t * tan_x * tan_x));
    };
    const double e1_i = e1(sin_i, mu0), e2_i = e2(sin_i, mu0);
    const double e1_e = e1(sin_e, mu), e2_e = e2(sin_e, mu);
    // eta(x): the effective cosine at psi = 0.
    const double eta_i = chi * (mu0 + sin_i * tan_t * e2_i / (2.0 - e1_i));
    const double eta_e = chi * (mu + sin_e * tan_t * e2_e / (2.0 - e1_e));
    const double psi_fraction = psi / pi;
    if (mu0 >= mu) {  // i <= e
        const double den = 2.0 - e1_e - psi_fraction * e1_i;
        out.mu0_effective = chi * (mu0 + sin_i * tan_t * (cos_psi * e2_e + sin2_half * e2_i) / den);
        out.mu_effective = chi * (mu + sin_e * tan_t * (e2_e - sin2_half * e2_i) / den);
        out.shadowing = out.mu_effective / eta_e * mu0 / eta_i * chi / (1.0 - f + f * chi * mu0 / eta_i);
    } else {  // e < i
        const double den = 2.0 - e1_i - psi_fraction * e1_e;
        out.mu0_effective = chi * (mu0 + sin_i * tan_t * (e2_i - sin2_half * e2_e) / den);
        out.mu_effective = chi * (mu + sin_e * tan_t * (cos_psi * e2_i + sin2_half * e2_e) / den);
        out.shadowing = out.mu_effective / eta_e * mu0 / eta_i * chi / (1.0 - f + f * chi * mu / eta_e);
    }
    return out;
}

double hapke_reflectance(const config::HapkeParameters& p, double mu0, double mu, double phase_rad,
                         double theta_bar_rad, HapkeHFunction form) {
    if (mu0 <= 0.0 || mu <= 0.0) return 0.0;
    mu0 = std::min(mu0, 1.0);
    mu = std::min(mu, 1.0);
    const HapkeRoughness r = hapke_roughness(mu0, mu, phase_rad, theta_bar_rad);
    const double k = p.porosity_k;
    const double shoe = 1.0 + p.bs0 * shadow_hiding(phase_rad, p.hs);
    const double cboe = 1.0 + p.bc0 * coherent_backscatter(phase_rad, p.hc);
    const double multiple = hapke_h_function(r.mu0_effective / k, p.w, form) *
                                hapke_h_function(r.mu_effective / k, p.w, form) -
                            1.0;
    return k * 0.25 * p.w * r.mu0_effective / (r.mu0_effective + r.mu_effective) *
           (hapke_phase_function(p.b, p.c, phase_rad) * shoe + multiple) * cboe * r.shadowing;
}

double hapke_disc_reflectance(const config::HapkeParameters& p, double phase_rad, double theta_bar_rad,
                              int quadrature_order) {
    return disc_integrated_reflectance(
        [&](double mu0, double mu) { return hapke_reflectance(p, mu0, mu, phase_rad, theta_bar_rad); }, phase_rad,
        quadrature_order);
}

double disc_integrated_reflectance(const std::function<double(double mu0, double mu)>& radiance_factor,
                                   double phase_rad, int quadrature_order) {
    if (phase_rad >= pi) return 0.0;
    // Photometric coordinates: latitude beta (from the intensity equator, the plane of Sun, Moon and eye) and
    // longitude lambda from the sub-observer point toward the Sun. mu = cos(beta) cos(lambda),
    // mu0 = cos(beta) cos(lambda - g); the lit, visible lune is lambda in [g - pi/2, pi/2]. The projected area element
    // is mu cos(beta) d(beta) d(lambda); the disc's projected area is pi.
    std::vector<double> x, w;
    gauss_legendre(quadrature_order, x, w);
    const double lambda_lo = phase_rad - 0.5 * pi, lambda_hi = 0.5 * pi;
    const double lambda_half = 0.5 * (lambda_hi - lambda_lo), lambda_mid = 0.5 * (lambda_hi + lambda_lo);
    double sum = 0.0;
    for (std::size_t a = 0; a < x.size(); ++a) {
        const double lambda = lambda_mid + lambda_half * x[a];
        for (std::size_t b = 0; b < x.size(); ++b) {
            const double beta = 0.5 * pi * x[b];
            const double cos_beta = std::cos(beta);
            const double mu = cos_beta * std::cos(lambda);
            const double mu0 = cos_beta * std::cos(lambda - phase_rad);
            if (mu0 <= 0.0 || mu <= 0.0) continue;
            sum += w[a] * w[b] * radiance_factor(mu0, mu) * mu * cos_beta;
        }
    }
    return sum * lambda_half * 0.5;  // x (pi / 2 for beta) / pi (the disc's area)
}

}  // namespace starsim::environment
