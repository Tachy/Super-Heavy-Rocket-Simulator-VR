#pragma once

// Internal building blocks of the atmosphere precomputation (M8 atmosphere): altitude tables of the optical inputs,
// the spectral coefficients of one pass of three wavelengths, the per-pass delta tables, the texture filtering and the
// passes themselves. Task 3 adds the scattering orders >= 2 and the indirect irradiance on top of these (see
// run_spectral_pass in atmosphere_precompute.cpp).
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

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <vector>

#include "starsim/config/atmosphere_optics_config.hpp"
#include "starsim/environment/atmosphere_tables.hpp"

namespace starsim::environment::detail {

// ---- GPU-style texture filtering: linear, texel centres at (i + 0.5) / size, clamp to edge.
struct LinearTap {
    int i0 = 0, i1 = 0;
    double f = 0.0;  // weight of i1
};
inline LinearTap linear_tap(double u, int size) {
    const double x = u * size - 0.5;
    const double x0 = std::floor(x);
    const int i = static_cast<int>(x0);
    return {std::clamp(i, 0, size - 1), std::clamp(i + 1, 0, size - 1), x - x0};
}

// Bilinear sample of a 2D table with C channels per texel (x fastest).
template <int C, class T>
std::array<double, C> sample_2d(const std::vector<T>& data, int width, int height, double u, double v) {
    const LinearTap tx = linear_tap(u, width), ty = linear_tap(v, height);
    std::array<double, C> out{};
    const auto add = [&](int x, int y, double w) {
        const std::size_t base = (static_cast<std::size_t>(y) * width + x) * C;
        for (int c = 0; c < C; ++c) out[c] += w * data[base + c];
    };
    add(tx.i0, ty.i0, (1.0 - tx.f) * (1.0 - ty.f));
    add(tx.i1, ty.i0, tx.f * (1.0 - ty.f));
    add(tx.i0, ty.i1, (1.0 - tx.f) * ty.f);
    add(tx.i1, ty.i1, tx.f * ty.f);
    return out;
}

// Trilinear sample of a 3D table with C channels per texel (x fastest, then y, then z).
template <int C, class T>
std::array<double, C> sample_3d(const std::vector<T>& data, int width, int height, int depth, double u, double v,
                                double w) {
    const LinearTap tx = linear_tap(u, width), ty = linear_tap(v, height), tz = linear_tap(w, depth);
    std::array<double, C> out{};
    const auto add = [&](int x, int y, int z, double weight) {
        const std::size_t base = ((static_cast<std::size_t>(z) * height + y) * width + x) * C;
        for (int c = 0; c < C; ++c) out[c] += weight * data[base + c];
    };
    for (int k = 0; k < 2; ++k) {
        const int z = k ? tz.i1 : tz.i0;
        const double wz = k ? tz.f : 1.0 - tz.f;
        add(tx.i0, ty.i0, z, (1.0 - tx.f) * (1.0 - ty.f) * wz);
        add(tx.i1, ty.i0, z, tx.f * (1.0 - ty.f) * wz);
        add(tx.i0, ty.i1, z, (1.0 - tx.f) * ty.f * wz);
        add(tx.i1, ty.i1, z, tx.f * ty.f * wz);
    }
    return out;
}

// GetScattering's lookup into a scattering-layout table: interpolation between the two nu slices around u_nu.
template <int C, class T>
std::array<double, C> sample_scattering_layout(const std::vector<T>& data, const AtmosphereTableSizes& sizes,
                                               const ScatteringUvwz& uvwz) {
    const double tex_coord_x = uvwz.u_nu * (sizes.scattering_nu - 1);
    const double tex_x = std::floor(tex_coord_x);
    const double lerp = tex_coord_x - tex_x;
    const int w = sizes.scattering_width(), h = sizes.scattering_height(), d = sizes.scattering_depth();
    const auto s0 = sample_3d<C>(data, w, h, d, (tex_x + uvwz.u_mu_s) / sizes.scattering_nu, uvwz.u_mu, uvwz.u_r);
    const auto s1 = sample_3d<C>(data, w, h, d, (tex_x + 1.0 + uvwz.u_mu_s) / sizes.scattering_nu, uvwz.u_mu, uvwz.u_r);
    std::array<double, C> out{};
    for (int c = 0; c < C; ++c) out[c] = s0[c] * (1.0 - lerp) + s1[c] * lerp;
    return out;
}

// ---- Optical inputs on an altitude table (true altitude above the ellipsoid / sphere of radius a), linear between
// rows. Replaces Bruneton's exponential density layers; the spectral dependence stays separable as in his model:
// coefficient(lambda, h) = cross section(lambda) x density(h).
struct DensityProfileTable {
    double altitude_min_m = 0.0;
    double step_m = 0.0;
    struct Row {
        double air_per_m3 = 0.0;              // molecule number density (Rayleigh)
        double ozone_per_m3 = 0.0;            // ozone number density (absorption)
        double aerosol_550nm_per_m = 0.0;     // aerosol extinction at 550 nm (Mie)
    };
    std::vector<Row> rows;

    Row at(double altitude_m) const;
};
DensityProfileTable make_density_profile_table(const config::AtmosphereOpticsConfig& optics);

// Bruneton's per-wavelength atmosphere coefficients of one pass (his AtmosphereParameters at three wavelengths).
struct SpectralCoefficients {
    std::array<double, 3> wavelengths_m{};
    std::array<double, 3> solar_irradiance_w_m2_per_m{};
    std::array<double, 3> rayleigh_cross_section_m2{};   // x air density = Rayleigh scattering
    std::array<double, 3> mie_extinction_factor{};       // x aerosol 550 nm extinction = Mie extinction
    std::array<double, 3> mie_scattering_factor{};       // single-scattering albedo x mie_extinction_factor
    std::array<double, 3> ozone_cross_section_m2{};      // x ozone density = absorption extinction
};
SpectralCoefficients spectral_coefficients(const config::AtmosphereOpticsConfig& optics,
                                           const std::array<double, 3>& wavelengths_m);

// Bruneton's luminance_from_radiance of one pass (model.cc Init): row c, column k = (XYZ->sRGB row c . CMF(lambda_k))
// x dlambda, without the 683 lm/W.
using Mat3 = std::array<std::array<double, 3>, 3>;
Mat3 luminance_from_radiance(const std::array<double, 3>& wavelengths_m, double dlambda_m);

// Delta tables of one pass, per wavelength in double (3 channels per texel): Bruneton's transmittance_texture,
// delta_irradiance_texture, delta_rayleigh_scattering_texture and delta_mie_scattering_texture. Task 3 reads them for
// the scattering density of order 2 and the ground bounce.
struct SpectralPassTables {
    std::vector<double> transmittance;       // transmittance_mu x transmittance_r
    std::vector<double> delta_irradiance;    // irradiance_mu_s x irradiance_r: the direct irradiance (W m^-2 m^-1)
    std::vector<double> delta_rayleigh;      // scattering layout: single Rayleigh, no phase function
    std::vector<double> delta_mie;           // scattering layout: single Mie, no phase function
};

// The model of one pass: geometry + profiles + the pass's coefficients.
struct PassModel {
    const AtmosphereParameters& atmosphere;
    const AtmosphereTableSizes& sizes;
    const DensityProfileTable& profile;
    const SpectralCoefficients& coefficients;
};

// Optical lengths (integrals of the three densities) to the top boundary: ComputeOpticalLengthToTopAtmosphereBoundary.
struct OpticalLengths {
    double air_per_m2 = 0.0, ozone_per_m2 = 0.0, aerosol_550nm = 0.0;
};
OpticalLengths optical_lengths_to_top(const AtmosphereParameters& atmosphere, const DensityProfileTable& profile,
                                      double r_m, double mu);
// ComputeTransmittanceToTopAtmosphereBoundary at the pass's wavelengths.
std::array<double, 3> compute_transmittance_to_top(const PassModel& model, double r_m, double mu);
std::array<double, 3> transmittance_from_optical_lengths(const SpectralCoefficients& coefficients,
                                                         const OpticalLengths& lengths);

// Lookups into the pass's own transmittance table (GetTransmittanceToTopAtmosphereBoundary, GetTransmittance,
// GetTransmittanceToSun).
std::array<double, 3> transmittance_to_top(const PassModel& model, const std::vector<double>& transmittance,
                                           double r_m, double mu);
std::array<double, 3> transmittance_along(const PassModel& model, const std::vector<double>& transmittance, double r_m,
                                          double mu, double d_m, bool ray_r_mu_intersects_ground);
std::array<double, 3> transmittance_to_sun(const PassModel& model, const std::vector<double>& transmittance,
                                           double r_m, double mu_s);

// Quadrature of an integral along a view ray (replaces Bruneton's 50 uniform trapezoid samples, which err by up to
// 25-75 % near the horizon with the 1.2 km aerosol layer and the 100 km top; Task 2 report). Task 3 reuses it for the
// multiple-scattering integral.
//
// Breakpoints: about `samples` intervals over [0, distance], clustered quadratically toward the ray's lowest point
// (the ground for a ground-hitting ray, else the tangent point - the start for mu >= 0), where the air is densest.
std::vector<double> ray_breakpoints_m(double r_m, double mu, double distance_m, bool ray_r_mu_intersects_ground,
                                      int samples);

// Adaptive Simpson over the breakpoint intervals: an interval is halved (at most adaptive_max_depth times) until the
// trapezoid-vs-Simpson error estimate of every channel is below tolerance x that channel's trapezoid integral over
// all breakpoints, shared out by interval length. f(d) returns C channels; the result is the integral of each.
inline constexpr int adaptive_max_depth = 7;

template <int C, class F>
void refine_interval(const F& f, double d0, const std::array<double, C>& f0, double d1, const std::array<double, C>& f1,
                     const std::array<double, C>& eps, double share, int depth, std::array<double, C>& total) {
    const double h = d1 - d0;
    const double dm = 0.5 * (d0 + d1);
    const std::array<double, C> fm = f(dm);
    bool converged = true;
    std::array<double, C> simpson{};
    for (int c = 0; c < C; ++c) {
        const double coarse = 0.5 * h * (f0[c] + f1[c]);
        const double fine = 0.25 * h * (f0[c] + 2.0 * fm[c] + f1[c]);
        simpson[c] = fine + (fine - coarse) / 3.0;
        if (std::abs(fine - coarse) / 3.0 > eps[c] * share) converged = false;
    }
    if (converged || depth >= adaptive_max_depth) {
        for (int c = 0; c < C; ++c) total[c] += simpson[c];
        return;
    }
    refine_interval<C>(f, d0, f0, dm, fm, eps, 0.5 * share, depth + 1, total);
    refine_interval<C>(f, dm, fm, d1, f1, eps, 0.5 * share, depth + 1, total);
}

template <int C, class F>
std::array<double, C> integrate_along_ray(const F& f, const std::vector<double>& breakpoints_m, double tolerance) {
    std::array<double, C> total{};
    if (breakpoints_m.size() < 2) return total;
    const double length = breakpoints_m.back() - breakpoints_m.front();
    std::vector<std::array<double, C>> values;
    values.reserve(breakpoints_m.size());
    for (const double d : breakpoints_m) values.push_back(f(d));
    std::array<double, C> eps{};
    for (std::size_t i = 0; i + 1 < breakpoints_m.size(); ++i) {
        const double h = breakpoints_m[i + 1] - breakpoints_m[i];
        for (int c = 0; c < C; ++c) eps[c] += 0.5 * h * std::abs(values[i][c] + values[i + 1][c]);
    }
    for (double& e : eps) e *= tolerance;
    for (std::size_t i = 0; i + 1 < breakpoints_m.size(); ++i) {
        const double share = length > 0.0 ? (breakpoints_m[i + 1] - breakpoints_m[i]) / length : 0.0;
        refine_interval<C>(f, breakpoints_m[i], values[i], breakpoints_m[i + 1], values[i + 1], eps, share, 0, total);
    }
    return total;
}

// ComputeDirectIrradiance and ComputeSingleScattering (Rayleigh, Mie) of one cell, the latter with `samples`
// quadrature intervals.
std::array<double, 3> compute_direct_irradiance(const PassModel& model, const std::vector<double>& transmittance,
                                                double r_m, double mu_s);
void compute_single_scattering(const PassModel& model, const std::vector<double>& transmittance,
                               const ScatteringCell& cell, std::array<double, 3>& rayleigh, std::array<double, 3>& mie,
                               int samples);

// The passes over whole tables (parallel over rows). The optical lengths of the transmittance texels do not depend on
// the wavelength: one table serves every pass.
std::vector<OpticalLengths> compute_optical_length_table(const AtmosphereParameters& atmosphere,
                                                         const AtmosphereTableSizes& sizes,
                                                         const DensityProfileTable& profile);
std::vector<double> compute_transmittance_table(const PassModel& model, const std::vector<OpticalLengths>& lengths);
std::vector<double> compute_transmittance_table(const PassModel& model);
std::vector<double> compute_direct_irradiance_table(const PassModel& model, const std::vector<double>& transmittance);
void compute_single_scattering_tables(const PassModel& model, const std::vector<double>& transmittance,
                                      std::vector<double>& delta_rayleigh, std::vector<double>& delta_mie);

// Runs body(row) for row in [0, rows) on hardware_concurrency threads.
void parallel_for_rows(int rows, const std::function<void(int)>& body);

}  // namespace starsim::environment::detail
