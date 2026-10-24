#pragma once

// Independent brute-force integrator of the atmosphere model (M8 atmosphere tests). It shares nothing with the table
// code (atmosphere_tables / atmosphere_precompute): own 3D ray geometry, the Task 1 optics functions called directly
// at every sample (no altitude table), fine composite-Simpson steps. The tables are checked against it.

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "starsim/config/atmosphere_optics_config.hpp"
#include "starsim/environment/atmosphere_optics.hpp"
#include "starsim/environment/atmosphere_spectra.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::test::atmosphere_reference {

using math::Vec3;

inline constexpr double nm = 1e-9;

struct Model {
    config::AtmosphereOpticsConfig optics;
    double sphere_radius_m = constants::wgs84_semi_major_axis_m;  // altitudes are |p| - a
    double ground_radius_m() const { return sphere_radius_m + optics.ground_altitude_m; }
    double top_radius_m() const { return sphere_radius_m + optics.top_altitude_m; }
};

// Roots t of |o + t d| = R for a unit d; false if the line misses the sphere.
inline bool sphere_roots(const Vec3& o, const Vec3& d, double radius_m, double& t_near, double& t_far) {
    const double b = math::dot(o, d);
    const double c = math::dot(o, o) - radius_m * radius_m;
    const double disc = b * b - c;
    if (disc < 0.0) return false;
    const double s = std::sqrt(disc);
    t_near = -b - s;
    t_far = -b + s;
    return true;
}

// Distance from o along d to the top sphere (o inside it).
inline double distance_to_top_m(const Model& m, const Vec3& o, const Vec3& d) {
    double t0 = 0.0, t1 = 0.0;
    sphere_roots(o, d, m.top_radius_m(), t0, t1);
    return std::max(t1, 0.0);
}

// Distance from o along d to the ground sphere, or a negative value if the ray does not hit it.
inline double distance_to_ground_m(const Model& m, const Vec3& o, const Vec3& d) {
    double t0 = 0.0, t1 = 0.0;
    if (!sphere_roots(o, d, m.ground_radius_m(), t0, t1)) return -1.0;
    return t0 >= 0.0 ? t0 : -1.0;
}

// Extinction coefficients (1/m) at the point p for the given wavelengths, straight from the Task 1 functions.
inline std::vector<double> extinction_per_m(const Model& m, const std::vector<double>& wavelengths_m, const Vec3& p) {
    const double h = math::norm(p) - m.sphere_radius_m;
    const double n_air = environment::air_number_density_per_m3(h);
    const double n_o3 = environment::ozone_number_density_per_m3(h, m.optics.ozone_column_du);
    std::vector<double> k(wavelengths_m.size());
    for (std::size_t i = 0; i < wavelengths_m.size(); ++i) {
        const double l = wavelengths_m[i];
        k[i] = environment::rayleigh_cross_section_m2(l) * n_air + environment::aerosol_extinction_per_m(l, h, m.optics) +
               environment::ozone_cross_section_m2(l) * n_o3;
    }
    return k;
}

// Optical depth along o + s d, s in [0, length], composite Simpson with `steps` (even) intervals.
inline std::vector<double> optical_depth(const Model& m, const std::vector<double>& wavelengths_m, const Vec3& o,
                                         const Vec3& d, double length_m, int steps) {
    std::vector<double> tau(wavelengths_m.size(), 0.0);
    const double h = length_m / steps;
    for (int i = 0; i <= steps; ++i) {
        const double w = (i == 0 || i == steps) ? 1.0 : (i % 2 ? 4.0 : 2.0);
        const auto k = extinction_per_m(m, wavelengths_m, o + d * (i * h));
        for (std::size_t j = 0; j < tau.size(); ++j) tau[j] += w * k[j];
    }
    for (double& t : tau) t *= h / 3.0;
    return tau;
}

// Transmittance from radius r along a ray with view-zenith cosine mu to the top of the atmosphere.
inline std::vector<double> transmittance_to_top(const Model& m, const std::vector<double>& wavelengths_m, double r_m,
                                                double mu, int steps = 100000) {
    const Vec3 o{0.0, 0.0, r_m};
    const Vec3 d{std::sqrt(std::max(0.0, 1.0 - mu * mu)), 0.0, mu};
    auto tau = optical_depth(m, wavelengths_m, o, d, distance_to_top_m(m, o, d), steps);
    for (double& t : tau) t = std::exp(-t);
    return tau;
}

// Bruneton's precomputed-luminance wavelengths (model.cc Init: 15 wavelengths in 5 passes of 3) and the weights that
// fold a spectral radiance (per m) into the linear sRGB "luminance" of the tables (without the 683 lm/W).
struct SpectralFold {
    std::vector<double> wavelengths_m;
    std::vector<std::array<double, 3>> weights;  // per wavelength: (XYZ->sRGB) CMF x dlambda (m)
};
inline SpectralFold luminance_fold() {
    SpectralFold f;
    const int count = 15;
    const double dl_nm = (environment::spectra::lambda_max_nm - environment::spectra::lambda_min_nm) / count;
    for (int i = 0; i < count; ++i) {
        const double l_nm = environment::spectra::lambda_min_nm + (i + 0.5) * dl_nm;
        const Vec3 xyz = environment::cie_1931_cmf(l_nm * nm);
        std::array<double, 3> w{};
        for (int c = 0; c < 3; ++c) {
            const double* row = environment::spectra::xyz_to_linear_srgb + 3 * c;
            w[c] = (row[0] * xyz.x + row[1] * xyz.y + row[2] * xyz.z) * dl_nm * nm;
        }
        f.wavelengths_m.push_back(l_nm * nm);
        f.weights.push_back(w);
    }
    return f;
}

struct SingleScattering {
    std::array<double, 3> rayleigh{};  // folded luminance, no phase function (Bruneton's reduced single scattering)
    std::array<double, 3> mie{};
    int sun_near_horizon_samples = 0;  // samples whose Sun lies within one solar radius of the horizon (0: clean cell)
};

// Single scattering at (r, mu, mu_s, nu) by a brute-force raymarch: the view ray to the nearest boundary (ground or
// top) with `view_steps` Simpson intervals (cumulative view optical depth by Simpson over each interval), at each
// sample the Sun's optical depth to the top with `sun_steps` Simpson intervals; a Sun below the horizon gives zero.
inline SingleScattering single_scattering(const Model& m, double r_m, double mu, double mu_s, double nu,
                                          int view_steps = 1000, int sun_steps = 1000) {
    const SpectralFold fold = luminance_fold();
    const auto& lambdas = fold.wavelengths_m;
    const std::size_t n = lambdas.size();

    const Vec3 o{0.0, 0.0, r_m};
    const double sin_mu = std::sqrt(std::max(0.0, 1.0 - mu * mu));
    const Vec3 view{sin_mu, 0.0, mu};
    const double sx = sin_mu > 0.0 ? (nu - mu * mu_s) / sin_mu : 0.0;
    const Vec3 sun{sx, std::sqrt(std::max(0.0, 1.0 - sx * sx - mu_s * mu_s)), mu_s};

    const double ground = distance_to_ground_m(m, o, view);
    const double length = ground >= 0.0 ? ground : distance_to_top_m(m, o, view);
    const double h = length / view_steps;
    const double sun_radius_rad = std::asin(constants::sun_radius_m / constants::astronomical_unit_m);

    SingleScattering out;
    std::vector<double> tau_view(n, 0.0);
    std::vector<double> sum_r(n, 0.0), sum_m(n, 0.0);
    std::vector<double> k_prev;
    for (int i = 0; i <= view_steps; ++i) {
        const Vec3 p = o + view * (i * h);
        const auto k = extinction_per_m(m, lambdas, p);
        if (i > 0) {
            const auto k_mid = extinction_per_m(m, lambdas, o + view * ((i - 0.5) * h));
            for (std::size_t j = 0; j < n; ++j) tau_view[j] += h / 6.0 * (k_prev[j] + 4.0 * k_mid[j] + k[j]);
        }
        k_prev = k;

        // Sun visibility: the geometric horizon of the ground sphere at p.
        const double rp = math::norm(p);
        const double mu_s_p = math::dot(p, sun) / rp;
        const double mu_horizon = -std::sqrt(std::max(0.0, 1.0 - std::pow(m.ground_radius_m() / rp, 2)));
        if (std::abs(std::acos(std::clamp(mu_s_p, -1.0, 1.0)) - std::acos(mu_horizon)) < 2.0 * sun_radius_rad) {
            ++out.sun_near_horizon_samples;
        }
        std::vector<double> t_sun(n, 0.0);
        if (distance_to_ground_m(m, p, sun) < 0.0) {
            const auto tau_sun = optical_depth(m, lambdas, p, sun, distance_to_top_m(m, p, sun), sun_steps);
            for (std::size_t j = 0; j < n; ++j) t_sun[j] = std::exp(-tau_sun[j]);
        }
        const double alt = rp - m.sphere_radius_m;
        const double n_air = environment::air_number_density_per_m3(alt);
        const double w = (i == 0 || i == view_steps) ? 1.0 : (i % 2 ? 4.0 : 2.0);
        for (std::size_t j = 0; j < n; ++j) {
            const double t = std::exp(-tau_view[j]) * t_sun[j];
            sum_r[j] += w * t * environment::rayleigh_cross_section_m2(lambdas[j]) * n_air;
            sum_m[j] += w * t * m.optics.aerosol_single_scattering_albedo *
                        environment::aerosol_extinction_per_m(lambdas[j], alt, m.optics);
        }
    }
    for (std::size_t j = 0; j < n; ++j) {
        const double e = environment::solar_irradiance_w_m2_per_m(lambdas[j]);
        for (int c = 0; c < 3; ++c) {
            out.rayleigh[c] += fold.weights[j][c] * e * sum_r[j] * h / 3.0;
            out.mie[c] += fold.weights[j][c] * e * sum_m[j] * h / 3.0;
        }
    }
    return out;
}

}  // namespace starsim::test::atmosphere_reference
