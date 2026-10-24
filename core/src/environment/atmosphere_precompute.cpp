// Precomputation of the atmosphere tables (M8 atmosphere): transmittance and single scattering in Bruneton's
// precomputed-luminance mode, per wavelength in double.
//
// Ported from Eric Bruneton's "Precomputed Atmospheric Scattering: a New Implementation" (2017,
// atmosphere/functions.glsl and atmosphere/model.cc, https://github.com/ebruneton/precomputed_atmospheric_scattering),
// distributed under the BSD 3-clause licence; his notice follows.
//
// Copyright (c) 2017 Eric Bruneton
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "atmosphere_precompute.hpp"

#include <atomic>
#include <stdexcept>
#include <thread>

#include "starsim/environment/atmosphere_optics.hpp"
#include "starsim/environment/atmosphere_spectra.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment::detail {

namespace {

constexpr double nm = 1e-9;

// Altitude step of the optical-input table: linear interpolation of the 1.2 km aerosol exponential errs by
// step^2 / (8 H^2) ~ 1e-5; the AFGL ozone rows (whole km) fall on table rows (-150 m + k x 10 m).
constexpr double profile_step_m = 10.0;

// Integration sample counts [S: Bruneton 2017 functions.glsl, SAMPLE_COUNT of
// ComputeOpticalLengthToTopAtmosphereBoundary (500) and ComputeSingleScattering (50)]. The single-scattering samples
// are the breakpoints of the adaptive quadrature (ray_breakpoints_m, integrate_along_ray): Bruneton's 50 uniform
// trapezoid samples err by up to 25-75 % near the horizon with the 1.2 km aerosol layer and the 100 km top.
constexpr int optical_length_samples = 500;
constexpr int single_scattering_samples = 50;
// Error target of the adaptive single-scattering quadrature, relative to each cell's integral [call: 1e-2 and 1e-3
// gave the same port-check errors (~1e-3, set by the transmittance table); 3e-3 keeps a margin].
constexpr double single_scattering_tolerance = 3e-3;

// Bruneton's precomputed-luminance spectrum (model.cc Init / demo.cc): 15 wavelengths in passes of 3.
constexpr int num_precomputed_wavelengths = 15;

double clamp_cosine(double mu) { return std::clamp(mu, -1.0, 1.0); }
double clamp_radius(const AtmosphereParameters& a, double r) { return std::clamp(r, a.bottom_radius_m, a.top_radius_m); }

double smoothstep(double edge0, double edge1, double x) {
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double altitude_m(double r_m) { return r_m - constants::wgs84_semi_major_axis_m; }

}  // namespace

// ---- Optical inputs.

DensityProfileTable::Row DensityProfileTable::at(double altitude_m) const {
    const double position = std::clamp((altitude_m - altitude_min_m) / step_m, 0.0, static_cast<double>(rows.size() - 1));
    const std::size_t i = std::min(static_cast<std::size_t>(position), rows.size() - 2);
    const double u = position - static_cast<double>(i);
    const Row& a = rows[i];
    const Row& b = rows[i + 1];
    return {a.air_per_m3 + u * (b.air_per_m3 - a.air_per_m3), a.ozone_per_m3 + u * (b.ozone_per_m3 - a.ozone_per_m3),
            a.aerosol_550nm_per_m + u * (b.aerosol_550nm_per_m - a.aerosol_550nm_per_m)};
}

DensityProfileTable make_density_profile_table(const config::AtmosphereOpticsConfig& optics) {
    DensityProfileTable table;
    table.altitude_min_m = optics.ground_altitude_m;
    table.step_m = profile_step_m;
    const int count =
        static_cast<int>(std::ceil((optics.top_altitude_m - optics.ground_altitude_m) / profile_step_m)) + 1;
    table.rows.resize(std::max(count, 2));
    for (int i = 0; i < static_cast<int>(table.rows.size()); ++i) {
        const double h = optics.ground_altitude_m + i * profile_step_m;
        table.rows[i] = {air_number_density_per_m3(h), ozone_number_density_per_m3(h, optics.ozone_column_du),
                         aerosol_extinction_per_m(550.0 * nm, h, optics)};
    }
    return table;
}

SpectralCoefficients spectral_coefficients(const config::AtmosphereOpticsConfig& optics,
                                           const std::array<double, 3>& wavelengths_m) {
    SpectralCoefficients c;
    c.wavelengths_m = wavelengths_m;
    const double aerosol_550 = aerosol_extinction_per_m(550.0 * nm, 0.0, optics);
    for (int k = 0; k < 3; ++k) {
        const double l = wavelengths_m[k];
        c.solar_irradiance_w_m2_per_m[k] = solar_irradiance_w_m2_per_m(l);
        c.rayleigh_cross_section_m2[k] = rayleigh_cross_section_m2(l);
        c.mie_extinction_factor[k] = aerosol_550 > 0.0 ? aerosol_extinction_per_m(l, 0.0, optics) / aerosol_550 : 0.0;
        c.mie_scattering_factor[k] = optics.aerosol_single_scattering_albedo * c.mie_extinction_factor[k];
        c.ozone_cross_section_m2[k] = ozone_cross_section_m2(l);
    }
    return c;
}

Mat3 luminance_from_radiance(const std::array<double, 3>& wavelengths_m, double dlambda_m) {
    Mat3 m{};
    for (int k = 0; k < 3; ++k) {
        const math::Vec3 xyz = cie_1931_cmf(wavelengths_m[k]);
        for (int c = 0; c < 3; ++c) {
            const double* row = spectra::xyz_to_linear_srgb + 3 * c;
            m[c][k] = (row[0] * xyz.x + row[1] * xyz.y + row[2] * xyz.z) * dlambda_m;
        }
    }
    return m;
}

// ---- Transmittance.

OpticalLengths optical_lengths_to_top(const AtmosphereParameters& atmosphere, const DensityProfileTable& profile,
                                      double r_m, double mu) {
    const double dx = distance_to_top_atmosphere_boundary_m(atmosphere, r_m, mu) / optical_length_samples;
    OpticalLengths result;
    for (int i = 0; i <= optical_length_samples; ++i) {
        const double d_i = i * dx;
        const double r_i = std::sqrt(d_i * d_i + 2.0 * r_m * mu * d_i + r_m * r_m);
        const auto row = profile.at(altitude_m(r_i));
        const double weight_i = (i == 0 || i == optical_length_samples) ? 0.5 : 1.0;
        result.air_per_m2 += row.air_per_m3 * weight_i * dx;
        result.ozone_per_m2 += row.ozone_per_m3 * weight_i * dx;
        result.aerosol_550nm += row.aerosol_550nm_per_m * weight_i * dx;
    }
    return result;
}

std::array<double, 3> compute_transmittance_to_top(const PassModel& model, double r_m, double mu) {
    return transmittance_from_optical_lengths(model.coefficients,
                                              optical_lengths_to_top(model.atmosphere, model.profile, r_m, mu));
}

std::array<double, 3> transmittance_from_optical_lengths(const SpectralCoefficients& c, const OpticalLengths& l) {
    std::array<double, 3> t{};
    for (int k = 0; k < 3; ++k) {
        t[k] = std::exp(-(c.rayleigh_cross_section_m2[k] * l.air_per_m2 + c.mie_extinction_factor[k] * l.aerosol_550nm +
                          c.ozone_cross_section_m2[k] * l.ozone_per_m2));
    }
    return t;
}

std::array<double, 3> transmittance_to_top(const PassModel& model, const std::vector<double>& transmittance,
                                           double r_m, double mu) {
    const TextureUv uv = transmittance_uv_from_r_mu(model.atmosphere, model.sizes, r_m, mu);
    return sample_2d<3>(transmittance, model.sizes.transmittance_mu, model.sizes.transmittance_r, uv.u, uv.v);
}

std::array<double, 3> transmittance_along(const PassModel& model, const std::vector<double>& transmittance, double r_m,
                                          double mu, double d_m, bool ray_r_mu_intersects_ground) {
    const double r_d = clamp_radius(model.atmosphere, std::sqrt(d_m * d_m + 2.0 * r_m * mu * d_m + r_m * r_m));
    const double mu_d = clamp_cosine((r_m * mu + d_m) / r_d);
    std::array<double, 3> num{}, den{};
    if (ray_r_mu_intersects_ground) {
        num = transmittance_to_top(model, transmittance, r_d, -mu_d);
        den = transmittance_to_top(model, transmittance, r_m, -mu);
    } else {
        num = transmittance_to_top(model, transmittance, r_m, mu);
        den = transmittance_to_top(model, transmittance, r_d, mu_d);
    }
    std::array<double, 3> t{};
    for (int k = 0; k < 3; ++k) t[k] = std::min(num[k] / den[k], 1.0);
    return t;
}

std::array<double, 3> transmittance_to_sun(const PassModel& model, const std::vector<double>& transmittance,
                                           double r_m, double mu_s) {
    const double sin_theta_h = model.atmosphere.bottom_radius_m / r_m;
    const double cos_theta_h = -std::sqrt(std::max(1.0 - sin_theta_h * sin_theta_h, 0.0));
    const double alpha = model.atmosphere.sun_angular_radius_rad;
    const double visible = smoothstep(-sin_theta_h * alpha, sin_theta_h * alpha, mu_s - cos_theta_h);
    auto t = transmittance_to_top(model, transmittance, r_m, mu_s);
    for (double& v : t) v *= visible;
    return t;
}

std::vector<OpticalLengths> compute_optical_length_table(const AtmosphereParameters& atmosphere,
                                                         const AtmosphereTableSizes& sizes,
                                                         const DensityProfileTable& profile) {
    const int w = sizes.transmittance_mu, h = sizes.transmittance_r;
    std::vector<OpticalLengths> table(static_cast<std::size_t>(w) * h);
    parallel_for_rows(h, [&](int j) {
        for (int i = 0; i < w; ++i) {
            const RMu rm = r_mu_from_transmittance_uv(atmosphere, sizes, {(i + 0.5) / w, (j + 0.5) / h});
            table[static_cast<std::size_t>(j) * w + i] = optical_lengths_to_top(atmosphere, profile, rm.r_m, rm.mu);
        }
    });
    return table;
}

std::vector<double> compute_transmittance_table(const PassModel& model, const std::vector<OpticalLengths>& lengths) {
    std::vector<double> table(lengths.size() * 3);
    for (std::size_t i = 0; i < lengths.size(); ++i) {
        const auto t = transmittance_from_optical_lengths(model.coefficients, lengths[i]);
        for (int k = 0; k < 3; ++k) table[3 * i + k] = t[k];
    }
    return table;
}

std::vector<double> compute_transmittance_table(const PassModel& model) {
    return compute_transmittance_table(model,
                                       compute_optical_length_table(model.atmosphere, model.sizes, model.profile));
}

// ---- Direct irradiance.

std::array<double, 3> compute_direct_irradiance(const PassModel& model, const std::vector<double>& transmittance,
                                                double r_m, double mu_s) {
    const double alpha_s = model.atmosphere.sun_angular_radius_rad;
    // Approximate average of the cosine factor mu_s over the visible fraction of the Sun disc.
    const double average_cosine_factor =
        mu_s < -alpha_s ? 0.0 : (mu_s > alpha_s ? mu_s : (mu_s + alpha_s) * (mu_s + alpha_s) / (4.0 * alpha_s));
    const auto t = transmittance_to_top(model, transmittance, r_m, mu_s);
    std::array<double, 3> e{};
    for (int k = 0; k < 3; ++k) e[k] = model.coefficients.solar_irradiance_w_m2_per_m[k] * t[k] * average_cosine_factor;
    return e;
}

std::vector<double> compute_direct_irradiance_table(const PassModel& model, const std::vector<double>& transmittance) {
    const int w = model.sizes.irradiance_mu_s, h = model.sizes.irradiance_r;
    std::vector<double> table(static_cast<std::size_t>(w) * h * 3);
    parallel_for_rows(h, [&](int j) {
        for (int i = 0; i < w; ++i) {
            const RMu rm = r_mu_s_from_irradiance_uv(model.atmosphere, model.sizes, {(i + 0.5) / w, (j + 0.5) / h});
            const auto e = compute_direct_irradiance(model, transmittance, rm.r_m, rm.mu);
            const std::size_t base = (static_cast<std::size_t>(j) * w + i) * 3;
            for (int k = 0; k < 3; ++k) table[base + k] = e[k];
        }
    });
    return table;
}

// ---- Single scattering.

std::vector<double> ray_breakpoints_m(double r_m, double mu, double distance_m, bool ray_r_mu_intersects_ground,
                                      int samples) {
    std::vector<double> d;
    if (distance_m <= 0.0) return d;
    // The lowest point of the ray: the ground for a ground-hitting ray, else the tangent point (the start for mu >= 0).
    const double d_low = ray_r_mu_intersects_ground ? distance_m : std::clamp(-r_m * mu, 0.0, distance_m);
    // d = d_low -/+ length u^2 at uniform u: dense where the air is.
    const auto piece = [&](double length_m, double sign) {
        if (length_m <= 0.0) return;
        const int n = std::max(2, static_cast<int>(std::lround(samples * length_m / distance_m)));
        for (int i = 0; i <= n; ++i) {
            const double u = static_cast<double>(i) / n;
            d.push_back(std::clamp(d_low + sign * length_m * u * u, 0.0, distance_m));
        }
    };
    piece(d_low, -1.0);
    piece(distance_m - d_low, +1.0);
    std::sort(d.begin(), d.end());
    d.erase(std::unique(d.begin(), d.end()), d.end());
    return d;
}

void compute_single_scattering(const PassModel& model, const std::vector<double>& transmittance,
                               const ScatteringCell& cell, std::array<double, 3>& rayleigh, std::array<double, 3>& mie,
                               int samples) {
    const AtmosphereParameters& a = model.atmosphere;
    const double r = cell.r_m, mu = cell.mu, mu_s = cell.mu_s, nu = cell.nu;
    const bool ground = cell.ray_r_mu_intersects_ground;
    const double distance = ground ? distance_to_bottom_atmosphere_boundary_m(a, r, mu)
                                   : distance_to_top_atmosphere_boundary_m(a, r, mu);
    // ComputeSingleScatteringIntegrand: Rayleigh RGB, Mie RGB (densities, no coefficients).
    const auto integrand = [&](double d) {
        const double r_d = clamp_radius(a, std::sqrt(d * d + 2.0 * r * mu * d + r * r));
        const double mu_s_d = clamp_cosine((r * mu_s + d * nu) / r_d);
        const auto t_view = transmittance_along(model, transmittance, r, mu, d, ground);
        const auto t_sun = transmittance_to_sun(model, transmittance, r_d, mu_s_d);
        const auto row = model.profile.at(altitude_m(r_d));
        std::array<double, 6> f{};
        for (int k = 0; k < 3; ++k) {
            const double t = t_view[k] * t_sun[k];
            f[k] = t * row.air_per_m3;
            f[3 + k] = t * row.aerosol_550nm_per_m;
        }
        return f;
    };
    const auto sum = integrate_along_ray<6>(integrand, ray_breakpoints_m(r, mu, distance, ground, samples),
                                            single_scattering_tolerance);
    const SpectralCoefficients& c = model.coefficients;
    for (int k = 0; k < 3; ++k) {
        rayleigh[k] = sum[k] * c.solar_irradiance_w_m2_per_m[k] * c.rayleigh_cross_section_m2[k];
        mie[k] = sum[3 + k] * c.solar_irradiance_w_m2_per_m[k] * c.mie_scattering_factor[k];
    }
}

void compute_single_scattering_tables(const PassModel& model, const std::vector<double>& transmittance,
                                      std::vector<double>& delta_rayleigh, std::vector<double>& delta_mie) {
    const AtmosphereTableSizes& s = model.sizes;
    const int w = s.scattering_width(), h = s.scattering_height(), d = s.scattering_depth();
    const std::size_t texels = static_cast<std::size_t>(w) * h * d;
    delta_rayleigh.assign(texels * 3, 0.0);
    delta_mie.assign(texels * 3, 0.0);
    parallel_for_rows(h * d, [&](int row) {
        const int z = row / h, y = row % h;
        for (int x = 0; x < w; ++x) {
            const ScatteringCell cell = scattering_cell_at_texel(model.atmosphere, s, x, y, z);
            std::array<double, 3> rayleigh{}, mie{};
            compute_single_scattering(model, transmittance, cell, rayleigh, mie, single_scattering_samples);
            const std::size_t base = ((static_cast<std::size_t>(z) * h + y) * w + x) * 3;
            for (int k = 0; k < 3; ++k) {
                delta_rayleigh[base + k] = rayleigh[k];
                delta_mie[base + k] = mie[k];
            }
        }
    });
}

// ---- Threads.

void parallel_for_rows(int rows, const std::function<void(int)>& body) {
    const int threads = std::max(1, std::min(rows, static_cast<int>(std::thread::hardware_concurrency())));
    std::atomic<int> next{0};
    const auto worker = [&] {
        for (int row = next++; row < rows; row = next++) body(row);
    };
    std::vector<std::thread> pool;
    pool.reserve(threads - 1);
    for (int i = 1; i < threads; ++i) pool.emplace_back(worker);
    worker();
    for (auto& t : pool) t.join();
}

namespace {

// Folds the pass's single scattering into the luminance tables (kComputeSingleScatteringShader with blending):
// scattering.rgb += M x delta_rayleigh, scattering.a += (M x delta_mie).r, single_mie.rgb += M x delta_mie.
void accumulate_single_scattering(const SpectralPassTables& pass, const Mat3& m, AtmosphereTables& tables) {
    const std::size_t texels = pass.delta_rayleigh.size() / 3;
    for (std::size_t i = 0; i < texels; ++i) {
        std::array<double, 3> lr{}, lm{};
        for (int c = 0; c < 3; ++c) {
            for (int k = 0; k < 3; ++k) {
                lr[c] += m[c][k] * pass.delta_rayleigh[3 * i + k];
                lm[c] += m[c][k] * pass.delta_mie[3 * i + k];
            }
        }
        for (int c = 0; c < 3; ++c) {
            tables.scattering[4 * i + c] += static_cast<float>(lr[c]);
            tables.single_mie[4 * i + c] += static_cast<float>(lm[c]);
        }
        tables.scattering[4 * i + 3] += static_cast<float>(lm[0]);
    }
}

// One pass of Model::Precompute at three wavelengths: transmittance, direct irradiance and single scattering into
// the pass tables, single scattering folded into the luminance tables. Task 3 continues here with the scattering
// orders 2 .. optics.scattering_orders (scattering density, indirect irradiance, multiple scattering), reading the
// pass tables and accumulating into tables.scattering and tables.irradiance through the same matrix m.
// The optical lengths are the same for every wavelength: computed once, not per pass as on Bruneton's GPU.
SpectralPassTables run_spectral_pass(const PassModel& model, const std::vector<OpticalLengths>& optical_lengths,
                                     const Mat3& m, AtmosphereTables& tables) {
    SpectralPassTables pass;
    pass.transmittance = compute_transmittance_table(model, optical_lengths);
    pass.delta_irradiance = compute_direct_irradiance_table(model, pass.transmittance);
    compute_single_scattering_tables(model, pass.transmittance, pass.delta_rayleigh, pass.delta_mie);
    accumulate_single_scattering(pass, m, tables);
    return pass;
}

void validate(const AtmosphereTableSizes& s) {
    for (const int v : {s.transmittance_mu, s.transmittance_r, s.scattering_r, s.scattering_mu, s.scattering_mu_s,
                        s.scattering_nu, s.irradiance_mu_s, s.irradiance_r}) {
        if (v < 2) throw std::invalid_argument("atmosphere table size below 2");
    }
    if (s.scattering_mu % 2 != 0) throw std::invalid_argument("atmosphere scattering_mu must be even");
}

}  // namespace

}  // namespace starsim::environment::detail

namespace starsim::environment {

AtmosphereTables precompute_atmosphere(const config::AtmosphereOpticsConfig& optics, const AtmosphereTableSizes& sizes) {
    using namespace detail;
    validate(sizes);
    AtmosphereTables tables;
    tables.sizes = sizes;
    tables.parameters = atmosphere_parameters(optics);
    tables.input_hash = atmosphere_input_hash(optics, sizes);
    const std::size_t scattering_texels =
        static_cast<std::size_t>(sizes.scattering_width()) * sizes.scattering_height() * sizes.scattering_depth();
    tables.scattering.assign(scattering_texels * 4, 0.0f);
    tables.single_mie.assign(scattering_texels * 4, 0.0f);
    tables.irradiance.assign(static_cast<std::size_t>(sizes.irradiance_mu_s) * sizes.irradiance_r * 4, 0.0f);

    const DensityProfileTable profile = make_density_profile_table(optics);
    const std::vector<OpticalLengths> optical_lengths = compute_optical_length_table(tables.parameters, sizes, profile);

    // Model::Init, precomputed luminance: passes of three wavelengths over 360-830 nm.
    const int num_iterations = (num_precomputed_wavelengths + 2) / 3;
    const double dlambda_nm = (spectra::lambda_max_nm - spectra::lambda_min_nm) / (3 * num_iterations);
    for (int i = 0; i < num_iterations; ++i) {
        std::array<double, 3> lambdas{};
        for (int k = 0; k < 3; ++k) lambdas[k] = (spectra::lambda_min_nm + (3 * i + k + 0.5) * dlambda_nm) * nm;
        const SpectralCoefficients coefficients = spectral_coefficients(optics, lambdas);
        const PassModel model{tables.parameters, sizes, profile, coefficients};
        run_spectral_pass(model, optical_lengths, luminance_from_radiance(lambdas, dlambda_nm * nm), tables);
    }

    // The transmittance table at kLambdaR / G / B (Init recomputes it after the passes).
    const SpectralCoefficients rgb = spectral_coefficients(
        optics, {spectra::lambda_r_nm * nm, spectra::lambda_g_nm * nm, spectra::lambda_b_nm * nm});
    const PassModel rgb_model{tables.parameters, sizes, profile, rgb};
    const std::vector<double> transmittance = compute_transmittance_table(rgb_model, optical_lengths);
    const std::size_t texels = transmittance.size() / 3;
    tables.transmittance.assign(texels * 4, 0.0f);
    for (std::size_t i = 0; i < texels; ++i) {
        for (int k = 0; k < 3; ++k) tables.transmittance[4 * i + k] = static_cast<float>(transmittance[3 * i + k]);
    }
    return tables;
}

}  // namespace starsim::environment
