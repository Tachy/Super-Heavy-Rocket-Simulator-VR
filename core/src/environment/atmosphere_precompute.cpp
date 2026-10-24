// Precomputation of the atmosphere tables (M8 atmosphere): transmittance, single and multiple scattering and the sky
// irradiance in Bruneton's precomputed-luminance mode, per wavelength in double.
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
#include <exception>
#include <mutex>
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
// The multiple-scattering ray integral (Bruneton: 50 uniform trapezoid samples) uses the same quadrature (controller
// ruling): breakpoints clustered toward the ray's lowest point, adaptive Simpson [call: 20 breakpoints; measured
// against 100 breakpoints at tolerance 3e-4: zenith luminance, sky illuminance and the twilight zenith at -18 deg
// within 3e-4, 6 s less precompute than 50 breakpoints].
constexpr int multiple_scattering_samples = 20;
constexpr double multiple_scattering_tolerance = 3e-3;

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
        c.ground_albedo[k] = optics.ground_albedo;
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

RayTransmittance::RayTransmittance(const PassModel& model, const std::vector<double>& transmittance, double r_m,
                                   double mu, bool ray_r_mu_intersects_ground)
    : model_(model), transmittance_(transmittance), r_m_(r_m), mu_(mu), ground_(ray_r_mu_intersects_ground),
      start_(transmittance_to_top(model, transmittance, r_m, ray_r_mu_intersects_ground ? -mu : mu)) {}

std::array<double, 3> RayTransmittance::at(double d_m) const {
    // transmittance_along with the start lookup reused (the same arithmetic).
    const double r_d = clamp_radius(model_.atmosphere, std::sqrt(d_m * d_m + 2.0 * r_m_ * mu_ * d_m + r_m_ * r_m_));
    const double mu_d = clamp_cosine((r_m_ * mu_ + d_m) / r_d);
    const auto other = transmittance_to_top(model_, transmittance_, r_d, ground_ ? -mu_d : mu_d);
    std::array<double, 3> t{};
    for (int k = 0; k < 3; ++k) t[k] = std::min(ground_ ? other[k] / start_[k] : start_[k] / other[k], 1.0);
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

namespace {

// ComputeSingleScatteringIntegrand: Rayleigh RGB, Mie RGB (densities, no coefficients) from the view and Sun
// transmittances and the densities at the sample point.
std::array<double, 6> single_scattering_terms(const std::array<double, 3>& t_view, const std::array<double, 3>& t_sun,
                                              const DensityProfileTable::Row& row) {
    std::array<double, 6> f{};
    for (int k = 0; k < 3; ++k) {
        const double t = t_view[k] * t_sun[k];
        f[k] = t * row.air_per_m3;
        f[3 + k] = t * row.aerosol_550nm_per_m;
    }
    return f;
}

auto single_scattering_integrand(const PassModel& model, const std::vector<double>& transmittance,
                                 const RayTransmittance& view_transmittance, const ScatteringCell& cell) {
    return [&model, &transmittance, &view_transmittance, cell](double d) {
        const double r = cell.r_m, mu = cell.mu, mu_s = cell.mu_s, nu = cell.nu;
        const double r_d = clamp_radius(model.atmosphere, std::sqrt(d * d + 2.0 * r * mu * d + r * r));
        const double mu_s_d = clamp_cosine((r * mu_s + d * nu) / r_d);
        return single_scattering_terms(view_transmittance.at(d),
                                       transmittance_to_sun(model, transmittance, r_d, mu_s_d),
                                       model.profile.at(altitude_m(r_d)));
    };
}

void single_scattering_result(const SpectralCoefficients& c, const std::array<double, 6>& sum,
                              std::array<double, 3>& rayleigh, std::array<double, 3>& mie) {
    for (int k = 0; k < 3; ++k) {
        rayleigh[k] = sum[k] * c.solar_irradiance_w_m2_per_m[k] * c.rayleigh_cross_section_m2[k];
        mie[k] = sum[3 + k] * c.solar_irradiance_w_m2_per_m[k] * c.mie_scattering_factor[k];
    }
}

}  // namespace

void compute_single_scattering(const PassModel& model, const std::vector<double>& transmittance,
                               const ScatteringCell& cell, std::array<double, 3>& rayleigh, std::array<double, 3>& mie,
                               int samples) {
    const AtmosphereParameters& a = model.atmosphere;
    const double r = cell.r_m, mu = cell.mu;
    const bool ground = cell.ray_r_mu_intersects_ground;
    const double distance = ground ? distance_to_bottom_atmosphere_boundary_m(a, r, mu)
                                   : distance_to_top_atmosphere_boundary_m(a, r, mu);
    const RayTransmittance view_transmittance(model, transmittance, r, mu, ground);
    const auto sum = integrate_along_ray<6>(single_scattering_integrand(model, transmittance, view_transmittance, cell),
                                            ray_breakpoints_m(r, mu, distance, ground, samples),
                                            single_scattering_tolerance);
    single_scattering_result(model.coefficients, sum, rayleigh, mie);
}

void compute_single_scattering_tables(const PassModel& model, const std::vector<double>& transmittance,
                                      std::vector<double>& delta_rayleigh, std::vector<double>& delta_mie) {
    const AtmosphereParameters& a = model.atmosphere;
    const AtmosphereTableSizes& s = model.sizes;
    const int w = s.scattering_width(), h = s.scattering_height(), depth = s.scattering_depth();
    const std::size_t texels = static_cast<std::size_t>(w) * h * depth;
    delta_rayleigh.assign(texels * 3, 0.0);
    delta_mie.assign(texels * 3, 0.0);
    // compute_single_scattering per texel, with what a row (fixed r, mu) shares computed once: the breakpoints and, at
    // them and their first-level midpoints, the sample radius, the view transmittance and the densities.
    parallel_for_rows(h * depth, [&](int row) {
        const int z = row / h, y = row % h;
        const ScatteringCell first = scattering_cell_at_texel(a, s, 0, y, z);
        const double r = first.r_m, mu = first.mu;
        const bool ground = first.ray_r_mu_intersects_ground;
        const double distance = ground ? distance_to_bottom_atmosphere_boundary_m(a, r, mu)
                                       : distance_to_top_atmosphere_boundary_m(a, r, mu);
        const RayTransmittance view_transmittance(model, transmittance, r, mu, ground);
        const std::vector<double> breakpoints = ray_breakpoints_m(r, mu, distance, ground, single_scattering_samples);
        struct Node {
            double d_m = 0.0, r_d_m = 0.0;
            std::array<double, 3> transmittance{};
            DensityProfileTable::Row densities;
        };
        std::vector<Node> nodes(breakpoints.empty() ? 0 : 2 * breakpoints.size() - 1);
        for (std::size_t k = 0; k < nodes.size(); ++k) {
            Node& n = nodes[k];
            n.d_m = k % 2 == 0 ? breakpoints[k / 2] : 0.5 * (breakpoints[k / 2] + breakpoints[k / 2 + 1]);
            n.r_d_m = clamp_radius(a, std::sqrt(n.d_m * n.d_m + 2.0 * r * mu * n.d_m + r * r));
            n.transmittance = view_transmittance.at(n.d_m);
            n.densities = model.profile.at(altitude_m(n.r_d_m));
        }
        for (int x = 0; x < w; ++x) {
            const ScatteringCell cell = scattering_cell_at_texel(a, s, x, y, z);
            const auto at_node = [&](std::size_t k) {
                const Node& n = nodes[k];
                const double mu_s_d = clamp_cosine((r * cell.mu_s + n.d_m * cell.nu) / n.r_d_m);
                return single_scattering_terms(n.transmittance, transmittance_to_sun(model, transmittance, n.r_d_m, mu_s_d),
                                               n.densities);
            };
            const auto sum = integrate_along_ray_with_nodes<6>(
                at_node, single_scattering_integrand(model, transmittance, view_transmittance, cell), breakpoints,
                single_scattering_tolerance);
            std::array<double, 3> rayleigh{}, mie{};
            single_scattering_result(model.coefficients, sum, rayleigh, mie);
            const std::size_t base = ((static_cast<std::size_t>(z) * h + y) * w + x) * 3;
            for (int k = 0; k < 3; ++k) {
                delta_rayleigh[base + k] = rayleigh[k];
                delta_mie[base + k] = mie[k];
            }
        }
    });
}


// ---- Multiple scattering.

namespace {

// Angular sample counts [S: Bruneton 2017 functions.glsl, SAMPLE_COUNT of ComputeScatteringDensity (16) and
// ComputeIndirectIrradiance (32)]: midpoint rule in theta and phi.
constexpr int density_samples = 16;
constexpr int indirect_irradiance_samples = 32;

struct AngleTable {
    std::vector<double> cos_v, sin_v;
    AngleTable(int count, double step) {
        for (int i = 0; i < count; ++i) {
            cos_v.push_back(std::cos((i + 0.5) * step));
            sin_v.push_back(std::sin((i + 0.5) * step));
        }
    }
};

// Largest scattering_nu of the ring lookups' fixed-size slice arrays (checked by validate()).
constexpr int max_nu_slices = 64;
using SliceValues = std::array<std::array<double, 3>, max_nu_slices>;

// The lookups of a ring of directions that share (r, mu, mu_s) and differ only in nu (the azimuth loops of
// ComputeScatteringDensity and ComputeIndirectIrradiance): the 8 taps in mu_s, mu and r are summed once per nu slice,
// then each direction only lerps between two slices - the same arithmetic as scattering_taps, regrouped.
class NuRing {
public:
    NuRing(const AtmosphereTableSizes& s, const ScatteringUvwz& uvwz) : slices_(s.scattering_nu), stride_(s.scattering_mu_s) {
        const int w = s.scattering_width(), h = s.scattering_height(), d = s.scattering_depth();
        // Within a slice, x = u_mu_s x mu_s_size - 0.5 (the slice offset tex_x x mu_s_size is added per slice); u_mu_s
        // lies in [0.5, size - 0.5] / size, so no tap leaves its slice.
        const LinearTap tx = linear_tap(uvwz.u_mu_s, stride_), ty = linear_tap(uvwz.u_mu, h), tz = linear_tap(uvwz.u_r, d);
        int k = 0;
        for (int iz = 0; iz < 2; ++iz) {
            for (int iy = 0; iy < 2; ++iy) {
                const std::size_t row = (static_cast<std::size_t>(iz ? tz.i1 : tz.i0) * h + (iy ? ty.i1 : ty.i0)) * w;
                const double wzy = (iz ? tz.f : 1.0 - tz.f) * (iy ? ty.f : 1.0 - ty.f);
                texel_[k] = row + tx.i0;
                weight_[k++] = wzy * (1.0 - tx.f);
                texel_[k] = row + tx.i1;
                weight_[k++] = wzy * tx.f;
            }
        }
    }

    // The table (3 channels per texel) per nu slice.
    SliceValues slice_values(const std::vector<double>& table) const {
        SliceValues out;
        for (int sl = 0; sl < slices_; ++sl) {
            std::array<double, 3> v{};
            const std::size_t offset = static_cast<std::size_t>(sl) * stride_;
            for (int k = 0; k < 8; ++k) {
                const double* texel = table.data() + (texel_[k] + offset) * 3;
                for (int c = 0; c < 3; ++c) v[c] += weight_[k] * texel[c];
            }
            out[sl] = v;
        }
        return out;
    }

    // GetScattering's nu lerp between the slices.
    std::array<double, 3> at(const SliceValues& slice_values, double nu) const {
        const double tex_coord_x = (nu + 1.0) / 2.0 * (slices_ - 1);
        const int s0 = std::clamp(static_cast<int>(std::floor(tex_coord_x)), 0, slices_ - 1);
        const int s1 = std::min(s0 + 1, slices_ - 1);
        const double lerp = tex_coord_x - s0;
        const auto& a = slice_values[s0];
        const auto& b = slice_values[s1];
        return {a[0] * (1.0 - lerp) + b[0] * lerp, a[1] * (1.0 - lerp) + b[1] * lerp, a[2] * (1.0 - lerp) + b[2] * lerp};
    }

private:
    int slices_, stride_;
    std::array<std::size_t, 8> texel_{};
    std::array<double, 8> weight_{};
};

// GetScattering of order n (functions.glsl) along one ring: the single scattering with its phase functions for n = 1,
// else the delta multiple-scattering table (phase functions included).
class RingRadiance {
public:
    RingRadiance(const PassModel& model, const SpectralPassTables& pass, const ScatteringUvwz& uvwz,
                 int scattering_order)
        : ring_(model.sizes, uvwz), g_(model.atmosphere.mie_phase_g), single_(scattering_order == 1) {
        if (single_) {
            rayleigh_ = ring_.slice_values(pass.delta_rayleigh);
            mie_ = ring_.slice_values(pass.delta_mie);
        } else {
            multiple_ = ring_.slice_values(pass.delta_multiple);
        }
    }
    std::array<double, 3> at(double nu) const {
        if (!single_) return ring_.at(multiple_, nu);
        const auto r = ring_.at(rayleigh_, nu);
        const auto m = ring_.at(mie_, nu);
        const double pr = rayleigh_phase_function(nu), pm = mie_phase_function(g_, nu);
        return {r[0] * pr + m[0] * pm, r[1] * pr + m[1] * pm, r[2] * pr + m[2] * pm};
    }

private:
    NuRing ring_;
    double g_;
    bool single_;
    SliceValues rayleigh_, mie_, multiple_;
};

}  // namespace

DensityGeometry::DensityGeometry(const PassModel& model, const SpectralPassTables& pass, double r_m, double mu)
    : r_m(r_m), mu(mu), omega_x(std::sqrt(std::max(1.0 - mu * mu, 0.0))) {
    const AtmosphereParameters& a = model.atmosphere;
    const AtmosphereTableSizes& s = model.sizes;
    const SpectralCoefficients& c = model.coefficients;
    const auto row = model.profile.at(altitude_m(r_m));
    std::array<double, 3> beta_rayleigh{}, beta_mie{};
    for (int k = 0; k < 3; ++k) {
        beta_rayleigh[k] = c.rayleigh_cross_section_m2[k] * row.air_per_m3;
        beta_mie[k] = c.mie_scattering_factor[k] * row.aerosol_550nm_per_m;
    }
    static const AngleTable theta_table(density_samples, constants::pi / density_samples);
    static const AngleTable phi_table(2 * density_samples, constants::pi / density_samples);
    const double dtheta = constants::pi / density_samples, dphi = constants::pi / density_samples;
    rings.resize(density_samples);
    weight.resize(static_cast<std::size_t>(density_samples) * 2 * density_samples);
    direction.resize(weight.size());
    for (int l = 0; l < density_samples; ++l) {
        Ring& ring = rings[l];
        const double cos_theta = theta_table.cos_v[l], sin_theta = theta_table.sin_v[l];
        ring.ground = ray_intersects_ground(a, r_m, cos_theta);
        if (ring.ground) {
            ring.distance_to_ground_m = distance_to_bottom_atmosphere_boundary_m(a, r_m, cos_theta);
            const auto t = transmittance_along(model, pass.transmittance, r_m, cos_theta, ring.distance_to_ground_m, true);
            for (int k = 0; k < 3; ++k) ring.ground_term[k] = t[k] * c.ground_albedo[k] / constants::pi;
        }
        // u_r and u_mu of the incident ray (r, mu_i): the same for every mu_s (scattering_uvwz_from_cell's r and mu
        // parts do not depend on mu_s).
        const ScatteringUvwz uvwz = scattering_uvwz_from_cell(a, s, {r_m, cos_theta, 1.0, 0.0, ring.ground});
        ring.u_r = uvwz.u_r;
        ring.u_mu = uvwz.u_mu;
        const double domega = dtheta * dphi * sin_theta;
        for (int m = 0; m < 2 * density_samples; ++m) {
            const std::size_t i = static_cast<std::size_t>(l) * 2 * density_samples + m;
            const double ix = phi_table.cos_v[m] * sin_theta, iy = phi_table.sin_v[m] * sin_theta, iz = cos_theta;
            direction[i] = {ix, iy, iz};
            // The scattering towards -omega: coefficients x phase functions of (omega, omega_i) x the solid angle.
            const double nu2 = omega_x * ix + mu * iz;
            const double pr = rayleigh_phase_function(nu2), pm = mie_phase_function(a.mie_phase_g, nu2);
            for (int k = 0; k < 3; ++k) weight[i][k] = (beta_rayleigh[k] * pr + beta_mie[k] * pm) * domega;
        }
    }
}

std::array<double, 3> compute_scattering_density(const PassModel& model, const SpectralPassTables& pass,
                                                 const DensityGeometry& geometry, double mu_s, double nu,
                                                 int scattering_order) {
    const AtmosphereParameters& a = model.atmosphere;
    const AtmosphereTableSizes& s = model.sizes;
    const double r = geometry.r_m, mu = geometry.mu, omega_x = geometry.omega_x;
    // Sun direction omega_s in the frame of the view direction omega = (omega_x, 0, mu), the zenith along z.
    const double sun_x = omega_x == 0.0 ? 0.0 : (nu - mu * mu_s) / omega_x;
    const double sun_y = std::sqrt(std::max(1.0 - sun_x * sun_x - mu_s * mu_s, 0.0));
    const double u_mu_s = scattering_uvwz_from_cell(a, s, {r, mu, mu_s, 0.0, false}).u_mu_s;
    const double inverse_bottom_radius = 1.0 / a.bottom_radius_m;
    std::array<double, 3> sum{};
    for (int l = 0; l < density_samples; ++l) {
        const DensityGeometry::Ring& g = geometry.rings[l];
        // (r, mu_i, mu_s) are shared by the azimuths of this ring; only nu varies.
        const RingRadiance ring(model, pass, {0.0, u_mu_s, g.u_mu, g.u_r}, scattering_order - 1);
        for (int m = 0; m < 2 * density_samples; ++m) {
            const std::size_t i = static_cast<std::size_t>(l) * 2 * density_samples + m;
            const auto& [ix, iy, iz] = geometry.direction[i];
            const double nu1 = std::clamp(sun_x * ix + sun_y * iy + mu_s * iz, -1.0, 1.0);
            std::array<double, 3> incident = ring.at(nu1);
            if (g.ground) {
                // The ground point's normal and the irradiance it received after n - 2 bounces. The ground point lies
                // on the bottom sphere: its normal is the point / bottom_radius (Bruneton normalises; same value).
                const double d = g.distance_to_ground_m;
                const double mu_ground = std::clamp(
                    (sun_x * ix * d + sun_y * iy * d + mu_s * (r + iz * d)) * inverse_bottom_radius, -1.0, 1.0);
                const TextureUv uv = irradiance_uv_from_r_mu_s(a, s, a.bottom_radius_m, mu_ground);
                const auto e = sample_2d<3>(pass.delta_irradiance, s.irradiance_mu_s, s.irradiance_r, uv.u, uv.v);
                for (int k = 0; k < 3; ++k) incident[k] += g.ground_term[k] * e[k];
            }
            const auto& w = geometry.weight[i];
            for (int k = 0; k < 3; ++k) sum[k] += incident[k] * w[k];
        }
    }
    return sum;
}

std::array<double, 3> compute_scattering_density(const PassModel& model, const SpectralPassTables& pass,
                                                 const ScatteringCell& cell, int scattering_order) {
    return compute_scattering_density(model, pass, DensityGeometry(model, pass, cell.r_m, cell.mu), cell.mu_s, cell.nu,
                                      scattering_order);
}

std::array<double, 3> compute_indirect_irradiance(const PassModel& model, const SpectralPassTables& pass, double r_m,
                                                  double mu_s, int scattering_order) {
    const AtmosphereParameters& a = model.atmosphere;
    const AtmosphereTableSizes& s = model.sizes;
    const double step = constants::pi / indirect_irradiance_samples;
    static const AngleTable theta_table(indirect_irradiance_samples / 2, step);
    static const AngleTable phi_table(2 * indirect_irradiance_samples, step);
    const double sun_x = std::sqrt(std::max(1.0 - mu_s * mu_s, 0.0));
    std::array<double, 3> sum{};
    for (int j = 0; j < indirect_irradiance_samples / 2; ++j) {
        const double cos_theta = theta_table.cos_v[j], sin_theta = theta_table.sin_v[j];
        const RingRadiance ring(model, pass, scattering_uvwz_from_cell(a, s, {r_m, cos_theta, mu_s, 0.0, false}),
                                scattering_order);
        const double domega = step * step * sin_theta;
        for (int i = 0; i < 2 * indirect_irradiance_samples; ++i) {
            const double nu = std::clamp(phi_table.cos_v[i] * sin_theta * sun_x + cos_theta * mu_s, -1.0, 1.0);
            const auto l = ring.at(nu);
            for (int k = 0; k < 3; ++k) sum[k] += l[k] * cos_theta * domega;
        }
    }
    return sum;
}

namespace {

// The integrand of ComputeMultipleScattering at distance d: the density looked up at the sample point times the
// view transmittance.
auto multiple_scattering_integrand(const PassModel& model, const std::vector<double>& scattering_density,
                                   const RayTransmittance& view_transmittance, const ScatteringCell& cell) {
    return [&model, &scattering_density, &view_transmittance, cell](double d) {
        const AtmosphereParameters& a = model.atmosphere;
        const double r = cell.r_m, mu = cell.mu, mu_s = cell.mu_s, nu = cell.nu;
        const double r_d = clamp_radius(a, std::sqrt(d * d + 2.0 * r * mu * d + r * r));
        const double mu_d = clamp_cosine((r * mu + d) / r_d);
        const double mu_s_d = clamp_cosine((r * mu_s + d * nu) / r_d);
        const auto density = sample_scattering_layout<3>(
            scattering_density, model.sizes,
            scattering_uvwz_from_cell(a, model.sizes, {r_d, mu_d, mu_s_d, nu, cell.ray_r_mu_intersects_ground}));
        const auto t = view_transmittance.at(d);
        return std::array<double, 3>{density[0] * t[0], density[1] * t[1], density[2] * t[2]};
    };
}

}  // namespace

std::array<double, 3> compute_multiple_scattering(const PassModel& model, const std::vector<double>& transmittance,
                                                  const std::vector<double>& scattering_density,
                                                  const ScatteringCell& cell) {
    const AtmosphereParameters& a = model.atmosphere;
    const double r = cell.r_m, mu = cell.mu;
    const bool ground = cell.ray_r_mu_intersects_ground;
    const double distance = ground ? distance_to_bottom_atmosphere_boundary_m(a, r, mu)
                                   : distance_to_top_atmosphere_boundary_m(a, r, mu);
    const RayTransmittance view_transmittance(model, transmittance, r, mu, ground);
    return integrate_along_ray<3>(multiple_scattering_integrand(model, scattering_density, view_transmittance, cell),
                                  ray_breakpoints_m(r, mu, distance, ground, multiple_scattering_samples),
                                  multiple_scattering_tolerance);
}

std::vector<double> compute_scattering_density_table(const PassModel& model, const SpectralPassTables& pass,
                                                     int scattering_order) {
    const AtmosphereTableSizes& s = model.sizes;
    const int w = s.scattering_width(), h = s.scattering_height(), d = s.scattering_depth();
    std::vector<double> table(static_cast<std::size_t>(w) * h * d * 3);
    // A row (fixed z, y) shares r and mu: its DensityGeometry serves every (mu_s, nu) texel of the row.
    parallel_for_rows(h * d, [&](int row) {
        const int z = row / h, y = row % h;
        const ScatteringCell first = scattering_cell_at_texel(model.atmosphere, s, 0, y, z);
        const DensityGeometry geometry(model, pass, first.r_m, first.mu);
        for (int x = 0; x < w; ++x) {
            const ScatteringCell cell = scattering_cell_at_texel(model.atmosphere, s, x, y, z);
            const auto v = compute_scattering_density(model, pass, geometry, cell.mu_s, cell.nu, scattering_order);
            const std::size_t base = ((static_cast<std::size_t>(z) * h + y) * w + x) * 3;
            for (int k = 0; k < 3; ++k) table[base + k] = v[k];
        }
    });
    return table;
}

std::vector<double> compute_indirect_irradiance_table(const PassModel& model, const SpectralPassTables& pass,
                                                      int scattering_order) {
    const int w = model.sizes.irradiance_mu_s, h = model.sizes.irradiance_r;
    std::vector<double> table(static_cast<std::size_t>(w) * h * 3);
    parallel_for_rows(h, [&](int j) {
        for (int i = 0; i < w; ++i) {
            const RMu rm = r_mu_s_from_irradiance_uv(model.atmosphere, model.sizes, {(i + 0.5) / w, (j + 0.5) / h});
            const auto e = compute_indirect_irradiance(model, pass, rm.r_m, rm.mu, scattering_order);
            const std::size_t base = (static_cast<std::size_t>(j) * w + i) * 3;
            for (int k = 0; k < 3; ++k) table[base + k] = e[k];
        }
    });
    return table;
}

std::vector<double> compute_multiple_scattering_table(const PassModel& model, const std::vector<double>& transmittance,
                                                      const std::vector<double>& scattering_density) {
    const AtmosphereParameters& a = model.atmosphere;
    const AtmosphereTableSizes& s = model.sizes;
    const int w = s.scattering_width(), h = s.scattering_height(), depth = s.scattering_depth();
    std::vector<double> table(static_cast<std::size_t>(w) * h * depth * 3);
    // compute_multiple_scattering per texel, with what a row (fixed r, mu) shares computed once: the breakpoints, the
    // view transmittance and (u_r, u_mu) at the breakpoints and their first-level midpoints.
    parallel_for_rows(h * depth, [&](int row) {
        const int z = row / h, y = row % h;
        const ScatteringCell first = scattering_cell_at_texel(a, s, 0, y, z);
        const double r = first.r_m, mu = first.mu;
        const bool ground = first.ray_r_mu_intersects_ground;
        const double distance = ground ? distance_to_bottom_atmosphere_boundary_m(a, r, mu)
                                       : distance_to_top_atmosphere_boundary_m(a, r, mu);
        const RayTransmittance view_transmittance(model, transmittance, r, mu, ground);
        const std::vector<double> breakpoints = ray_breakpoints_m(r, mu, distance, ground, multiple_scattering_samples);
        struct Node {
            double d_m = 0.0, r_d_m = 0.0, u_mu = 0.0, u_r = 0.0;
            std::array<double, 3> transmittance{};
        };
        std::vector<Node> nodes(breakpoints.empty() ? 0 : 2 * breakpoints.size() - 1);
        for (std::size_t k = 0; k < nodes.size(); ++k) {
            Node& n = nodes[k];
            n.d_m = k % 2 == 0 ? breakpoints[k / 2] : 0.5 * (breakpoints[k / 2] + breakpoints[k / 2 + 1]);
            n.r_d_m = clamp_radius(a, std::sqrt(n.d_m * n.d_m + 2.0 * r * mu * n.d_m + r * r));
            const double mu_d = clamp_cosine((r * mu + n.d_m) / n.r_d_m);
            const ScatteringUvwz uvwz = scattering_uvwz_from_cell(a, s, {n.r_d_m, mu_d, 1.0, 0.0, ground});
            n.u_mu = uvwz.u_mu;
            n.u_r = uvwz.u_r;
            n.transmittance = view_transmittance.at(n.d_m);
        }
        for (int x = 0; x < w; ++x) {
            const ScatteringCell cell = scattering_cell_at_texel(a, s, x, y, z);
            const double mu_s = cell.mu_s, nu = cell.nu;
            const auto at_node = [&](std::size_t k) {
                const Node& n = nodes[k];
                const double mu_s_d = clamp_cosine((r * mu_s + n.d_m * nu) / n.r_d_m);
                const auto density = sample_scattering_layout<3>(
                    scattering_density, s, {(nu + 1.0) / 2.0, scattering_u_mu_s(a, s, mu_s_d), n.u_mu, n.u_r});
                return std::array<double, 3>{density[0] * n.transmittance[0], density[1] * n.transmittance[1],
                                             density[2] * n.transmittance[2]};
            };
            const auto v = integrate_along_ray_with_nodes<3>(
                at_node, multiple_scattering_integrand(model, scattering_density, view_transmittance, cell),
                breakpoints, multiple_scattering_tolerance);
            const std::size_t base = ((static_cast<std::size_t>(z) * h + y) * w + x) * 3;
            for (int k = 0; k < 3; ++k) table[base + k] = v[k];
        }
    });
    return table;
}

// ---- Threads.

void parallel_for_rows(int rows, const std::function<void(int)>& body) {
    const int threads = std::max(1, std::min(rows, static_cast<int>(std::thread::hardware_concurrency())));
    std::atomic<int> next{0};
    std::atomic<bool> failed{false};
    std::exception_ptr error;
    std::mutex error_mutex;
    const auto worker = [&] {
        try {
            for (int row = next++; row < rows && !failed; row = next++) body(row);
        } catch (...) {
            const std::lock_guard<std::mutex> lock(error_mutex);
            if (!error) error = std::current_exception();
            failed = true;
        }
    };
    std::vector<std::thread> pool;
    pool.reserve(threads - 1);
    try {
        for (int i = 1; i < threads; ++i) pool.emplace_back(worker);
    } catch (...) {
        failed = true;
        for (auto& t : pool) t.join();
        throw;
    }
    worker();
    for (auto& t : pool) t.join();
    if (error) std::rethrow_exception(error);
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

// The indirect irradiance of one order into the luminance table (kComputeIndirectIrradianceShader with blending):
// irradiance.rgb += M x delta_irradiance.
void accumulate_irradiance(const std::vector<double>& delta_irradiance, const Mat3& m, AtmosphereTables& tables) {
    const std::size_t texels = delta_irradiance.size() / 3;
    for (std::size_t i = 0; i < texels; ++i) {
        for (int c = 0; c < 3; ++c) {
            double v = 0.0;
            for (int k = 0; k < 3; ++k) v += m[c][k] * delta_irradiance[3 * i + k];
            tables.irradiance[4 * i + c] += static_cast<float>(v);
        }
    }
}

// One order of multiple scattering into the luminance table (kComputeMultipleScatteringShader with blending):
// scattering.rgb += M x delta_multiple / RayleighPhaseFunction(nu), nu of the texel (clamped as in the precomputation).
void accumulate_multiple_scattering(const std::vector<double>& delta_multiple, const std::vector<double>& texel_nu,
                                    const Mat3& m, AtmosphereTables& tables) {
    for (std::size_t i = 0; i < texel_nu.size(); ++i) {
        const double inverse_phase = 1.0 / rayleigh_phase_function(texel_nu[i]);
        for (int c = 0; c < 3; ++c) {
            double v = 0.0;
            for (int k = 0; k < 3; ++k) v += m[c][k] * delta_multiple[3 * i + k];
            tables.scattering[4 * i + c] += static_cast<float>(v * inverse_phase);
        }
    }
}

// The nu of every scattering texel (scattering_cell_at_texel).
std::vector<double> scattering_texel_nu(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& s) {
    const int w = s.scattering_width(), h = s.scattering_height(), d = s.scattering_depth();
    std::vector<double> nu(static_cast<std::size_t>(w) * h * d);
    parallel_for_rows(h * d, [&](int row) {
        const int z = row / h, y = row % h;
        for (int x = 0; x < w; ++x) {
            nu[(static_cast<std::size_t>(z) * h + y) * w + x] = scattering_cell_at_texel(atmosphere, s, x, y, z).nu;
        }
    });
    return nu;
}

// The orders above the last computed one as a geometric series (an addition to Bruneton, who truncates): from order 3
// on, each order is a near-constant fraction q of the previous one per texel and wavelength (measured: q = 0.39-0.44
// at the zenith and 2 deg above the horizon, Sun 30 deg; the series to 14 orders follows it within 0.2 %). The tail
// sum_{k>=1} q^k x last = last x q / (1 - q), with q = last / previous per value [S: Hillaire 2020, "A Scalable and
// Production Ready Sky and Atmosphere Rendering Technique", EGSR, Eq. 9: all orders >= 2 as the geometric series
// 1 / (1 - f_ms)]. Without it 4 orders fall short by 7 % (blue near the horizon), 3 % at the zenith, 2 % in the sky
// illuminance. q is capped (a texel whose last orders are tiny or noisy gets at most 3 x its last order).
constexpr double max_geometric_tail_ratio = 0.75;  // [call] tail <= 3 x the last order; measured q < 0.6

std::vector<double> geometric_tail(const std::vector<double>& previous, const std::vector<double>& last) {
    std::vector<double> tail(last.size(), 0.0);
    for (std::size_t i = 0; i < last.size(); ++i) {
        if (previous[i] <= 0.0 || last[i] <= 0.0) continue;
        const double q = std::min(last[i] / previous[i], max_geometric_tail_ratio);
        tail[i] = last[i] * q / (1.0 - q);
    }
    return tail;
}

// One pass of Model::Precompute at three wavelengths: transmittance, direct irradiance and single scattering into
// the pass tables, single scattering folded into the luminance tables; then the orders 2 .. scattering_orders
// (scattering density, indirect irradiance of the previous order, multiple scattering), each folded into
// tables.irradiance and tables.scattering through the same matrix m; for scattering_orders >= 3 the geometric tail of
// the higher orders of both.
// The optical lengths are the same for every wavelength: computed once, not per pass as on Bruneton's GPU.
void run_spectral_pass(const PassModel& model, const std::vector<OpticalLengths>& optical_lengths,
                       const std::vector<double>& texel_nu, int scattering_orders, const Mat3& m,
                       AtmosphereTables& tables) {
    SpectralPassTables pass;
    pass.transmittance = compute_transmittance_table(model, optical_lengths);
    pass.delta_irradiance = compute_direct_irradiance_table(model, pass.transmittance);
    compute_single_scattering_tables(model, pass.transmittance, pass.delta_rayleigh, pass.delta_mie);
    accumulate_single_scattering(pass, m, tables);
    std::vector<double> previous_irradiance, previous_multiple;  // the orders before the last (irradiance: sky order)
    for (int order = 2; order <= scattering_orders; ++order) {
        pass.scattering_density = compute_scattering_density_table(model, pass, order);
        previous_irradiance = std::move(pass.delta_irradiance);
        pass.delta_irradiance = compute_indirect_irradiance_table(model, pass, order - 1);
        accumulate_irradiance(pass.delta_irradiance, m, tables);
        previous_multiple = std::move(pass.delta_multiple);
        pass.delta_multiple = compute_multiple_scattering_table(model, pass.transmittance, pass.scattering_density);
        accumulate_multiple_scattering(pass.delta_multiple, texel_nu, m, tables);
    }
    if (scattering_orders >= 3) {
        // previous_multiple / delta_multiple: orders N - 1 and N; previous_irradiance / delta_irradiance: the sky
        // irradiance of orders N - 2 and N - 1 (the irradiance table holds orders 1 .. N - 1).
        accumulate_multiple_scattering(geometric_tail(previous_multiple, pass.delta_multiple), texel_nu, m, tables);
        accumulate_irradiance(geometric_tail(previous_irradiance, pass.delta_irradiance), m, tables);
    }
}

void validate(const AtmosphereTableSizes& s) {
    for (const int v : {s.transmittance_mu, s.transmittance_r, s.scattering_r, s.scattering_mu, s.scattering_mu_s,
                        s.scattering_nu, s.irradiance_mu_s, s.irradiance_r}) {
        if (v < 2) throw std::invalid_argument("atmosphere table size below 2");
    }
    if (s.scattering_mu % 2 != 0) throw std::invalid_argument("atmosphere scattering_mu must be even");
    if (s.scattering_nu > max_nu_slices) throw std::invalid_argument("atmosphere scattering_nu above 64");
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
    const std::vector<double> texel_nu =
        optics.scattering_orders > 1 ? scattering_texel_nu(tables.parameters, sizes) : std::vector<double>{};

    // Model::Init, precomputed luminance: passes of three wavelengths over 360-830 nm.
    const int num_iterations = (num_precomputed_wavelengths + 2) / 3;
    const double dlambda_nm = (spectra::lambda_max_nm - spectra::lambda_min_nm) / (3 * num_iterations);
    for (int i = 0; i < num_iterations; ++i) {
        std::array<double, 3> lambdas{};
        for (int k = 0; k < 3; ++k) lambdas[k] = (spectra::lambda_min_nm + (3 * i + k + 0.5) * dlambda_nm) * nm;
        const SpectralCoefficients coefficients = spectral_coefficients(optics, lambdas);
        const PassModel model{tables.parameters, sizes, profile, coefficients};
        run_spectral_pass(model, optical_lengths, texel_nu, optics.scattering_orders,
                          luminance_from_radiance(lambdas, dlambda_nm * nm), tables);
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
