#pragma once

// Internal building blocks of the atmosphere precomputation (M8 atmosphere): altitude tables of the optical inputs,
// the spectral coefficients of one pass of three wavelengths, the per-pass delta tables, the texture filtering and the
// passes themselves: transmittance, direct irradiance, single scattering, then the scattering orders >= 2 with the
// indirect irradiance (run_spectral_pass in atmosphere_precompute.cpp).
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

// Bilinear sample of a 2D table with C channels per texel (x fastest); the first N <= C channels.
template <int N, int C = N, class T>
std::array<double, N> sample_2d(const std::vector<T>& data, int width, int height, double u, double v) {
    const LinearTap tx = linear_tap(u, width), ty = linear_tap(v, height);
    std::array<double, N> out{};
    const auto add = [&](int x, int y, double w) {
        const std::size_t base = (static_cast<std::size_t>(y) * width + x) * C;
        for (int c = 0; c < N; ++c) out[c] += w * data[base + c];
    };
    add(tx.i0, ty.i0, (1.0 - tx.f) * (1.0 - ty.f));
    add(tx.i1, ty.i0, tx.f * (1.0 - ty.f));
    add(tx.i0, ty.i1, (1.0 - tx.f) * ty.f);
    add(tx.i1, ty.i1, tx.f * ty.f);
    return out;
}

// GetScattering's lookup into a scattering-layout table: two trilinear samples (the nu slices around u_nu, each
// filtered like a GPU 3D texture over the packed (nu, mu_s) x mu x r layout), lerped in nu. As 16 texels and weights,
// so that several tables of the same layout share one coordinate computation.
struct ScatteringTaps {
    std::array<std::size_t, 16> texel{};
    std::array<double, 16> weight{};
};
inline ScatteringTaps scattering_taps(const AtmosphereTableSizes& sizes, const ScatteringUvwz& uvwz) {
    const double tex_coord_x = uvwz.u_nu * (sizes.scattering_nu - 1);
    const double tex_x = std::floor(tex_coord_x);
    const double lerp = tex_coord_x - tex_x;
    const int w = sizes.scattering_width(), h = sizes.scattering_height(), d = sizes.scattering_depth();
    const LinearTap ty = linear_tap(uvwz.u_mu, h), tz = linear_tap(uvwz.u_r, d);
    ScatteringTaps taps;
    int k = 0;
    for (int slice = 0; slice < 2; ++slice) {
        const LinearTap tx = linear_tap((tex_x + slice + uvwz.u_mu_s) / sizes.scattering_nu, w);
        const double ws = slice ? lerp : 1.0 - lerp;
        for (int iz = 0; iz < 2; ++iz) {
            const int z = iz ? tz.i1 : tz.i0;
            const double wz = ws * (iz ? tz.f : 1.0 - tz.f);
            for (int iy = 0; iy < 2; ++iy) {
                const int y = iy ? ty.i1 : ty.i0;
                const double wy = wz * (iy ? ty.f : 1.0 - ty.f);
                const std::size_t row = (static_cast<std::size_t>(z) * h + y) * w;
                taps.texel[k] = row + tx.i0;
                taps.weight[k++] = wy * (1.0 - tx.f);
                taps.texel[k] = row + tx.i1;
                taps.weight[k++] = wy * tx.f;
            }
        }
    }
    return taps;
}

// The table (C channels per texel) at the taps; the first N <= C channels.
template <int N, int C = N, class T>
std::array<double, N> sample_taps(const std::vector<T>& data, const ScatteringTaps& taps) {
    std::array<double, N> out{};
    for (int k = 0; k < 16; ++k) {
        const double w = taps.weight[k];
        const T* texel = data.data() + taps.texel[k] * C;
        for (int c = 0; c < N; ++c) out[c] += w * texel[c];
    }
    return out;
}

template <int C, class T>
std::array<double, C> sample_scattering_layout(const std::vector<T>& data, const AtmosphereTableSizes& sizes,
                                               const ScatteringUvwz& uvwz) {
    return sample_taps<C>(data, scattering_taps(sizes, uvwz));
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
    std::array<double, 3> ground_albedo{};               // Lambertian ground of the multiple-scattering bounce
};
SpectralCoefficients spectral_coefficients(const config::AtmosphereOpticsConfig& optics,
                                           const std::array<double, 3>& wavelengths_m);

// Bruneton's luminance_from_radiance of one pass (model.cc Init): row c, column k = (XYZ->sRGB row c . CMF(lambda_k))
// x dlambda, without the 683 lm/W.
using Mat3 = std::array<std::array<double, 3>, 3>;
Mat3 luminance_from_radiance(const std::array<double, 3>& wavelengths_m, double dlambda_m);

// Delta tables of one pass, per wavelength in double (3 channels per texel): Bruneton's transmittance_texture,
// delta_irradiance_texture, delta_rayleigh_scattering_texture, delta_mie_scattering_texture,
// delta_multiple_scattering_texture and delta_scattering_density_texture.
struct SpectralPassTables {
    std::vector<double> transmittance;       // transmittance_mu x transmittance_r
    // irradiance_mu_s x irradiance_r (W m^-2 m^-1): the direct irradiance, then the indirect irradiance of the last
    // computed order (the ground irradiance after n - 2 bounces when order n's density is computed).
    std::vector<double> delta_irradiance;
    std::vector<double> delta_rayleigh;      // scattering layout: single Rayleigh, no phase function
    std::vector<double> delta_mie;           // scattering layout: single Mie, no phase function
    std::vector<double> delta_multiple;      // scattering layout: the last order >= 2, phase functions included
    std::vector<double> scattering_density;  // scattering layout: the density of the order being computed
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

// GetTransmittance along one ray for many distances: the lookup at the ray's start (the denominator for a ray to the
// top, the numerator's partner for a ray to the ground) is the same for every d and is done once.
class RayTransmittance {
public:
    RayTransmittance(const PassModel& model, const std::vector<double>& transmittance, double r_m, double mu,
                     bool ray_r_mu_intersects_ground);
    std::array<double, 3> at(double d_m) const;

private:
    const PassModel& model_;
    const std::vector<double>& transmittance_;
    double r_m_, mu_;
    bool ground_;
    std::array<double, 3> start_{};  // transmittance_to_top at (r, mu), or at (r, -mu) for a ray to the ground
};

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
                     const std::array<double, C>& eps, double share, int depth, std::array<double, C>& total);

// refine_interval with the midpoint value fm = f((d0 + d1) / 2) already known.
template <int C, class F>
void refine_interval_with_midpoint(const F& f, double d0, const std::array<double, C>& f0, double d1,
                                   const std::array<double, C>& f1, const std::array<double, C>& fm,
                                   const std::array<double, C>& eps, double share, int depth,
                                   std::array<double, C>& total) {
    const double h = d1 - d0;
    const double dm = 0.5 * (d0 + d1);
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
void refine_interval(const F& f, double d0, const std::array<double, C>& f0, double d1, const std::array<double, C>& f1,
                     const std::array<double, C>& eps, double share, int depth, std::array<double, C>& total) {
    refine_interval_with_midpoint<C>(f, d0, f0, d1, f1, f(0.5 * (d0 + d1)), eps, share, depth, total);
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

// integrate_along_ray with the integrand at the breakpoints and at their first-level midpoints supplied by node(k)
// (k = 2 i: breakpoint i; k = 2 i + 1: the midpoint of interval i, at exactly (d_i + d_i+1) / 2): the same
// arithmetic, for callers that share those nodes between many integrals. Deeper refinements call f(d).
template <int C, class Node, class F>
std::array<double, C> integrate_along_ray_with_nodes(const Node& node, const F& f,
                                                     const std::vector<double>& breakpoints_m, double tolerance) {
    std::array<double, C> total{};
    if (breakpoints_m.size() < 2) return total;
    const double length = breakpoints_m.back() - breakpoints_m.front();
    std::vector<std::array<double, C>> values;
    values.reserve(breakpoints_m.size());
    for (std::size_t i = 0; i < breakpoints_m.size(); ++i) values.push_back(node(2 * i));
    std::array<double, C> eps{};
    for (std::size_t i = 0; i + 1 < breakpoints_m.size(); ++i) {
        const double h = breakpoints_m[i + 1] - breakpoints_m[i];
        for (int c = 0; c < C; ++c) eps[c] += 0.5 * h * std::abs(values[i][c] + values[i + 1][c]);
    }
    for (double& e : eps) e *= tolerance;
    for (std::size_t i = 0; i + 1 < breakpoints_m.size(); ++i) {
        const double share = length > 0.0 ? (breakpoints_m[i + 1] - breakpoints_m[i]) / length : 0.0;
        refine_interval_with_midpoint<C>(f, breakpoints_m[i], values[i], breakpoints_m[i + 1], values[i + 1],
                                         node(2 * i + 1), eps, share, 0, total);
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
std::vector<double> compute_direct_irradiance_table(const PassModel& model, const std::vector<double>& transmittance);
void compute_single_scattering_tables(const PassModel& model, const std::vector<double>& transmittance,
                                      std::vector<double>& delta_rayleigh, std::vector<double>& delta_mie);

// ---- Multiple scattering (orders >= 2), Bruneton functions.glsl.
// ComputeScatteringDensity of order n at a cell: the light scattered at (r, mu, mu_s, nu) towards -view after n - 1
// bounces (pass.delta_rayleigh / delta_mie for n = 2, else pass.delta_multiple) plus the ground bounce of
// pass.delta_irradiance; 16 x 32 incident directions.
std::array<double, 3> compute_scattering_density(const PassModel& model, const SpectralPassTables& pass,
                                                 const ScatteringCell& cell, int scattering_order);
// The part of ComputeScatteringDensity that depends on (r, mu) only - a whole texel row: per incident direction the
// scattering coefficients x phase functions x solid angle, per theta ring the ground distance and transmittance and
// the incident ray's table coordinates u_r, u_mu.
struct DensityGeometry {
    DensityGeometry(const PassModel& model, const SpectralPassTables& pass, double r_m, double mu);
    struct Ring {
        bool ground = false;
        double distance_to_ground_m = 0.0;
        std::array<double, 3> ground_term{};  // transmittance to the ground x albedo / pi
        double u_r = 0.0, u_mu = 0.0;
    };
    double r_m, mu, omega_x;
    std::vector<Ring> rings;
    std::vector<std::array<double, 3>> direction, weight;  // per (theta, phi): omega_i and its scattering weight
};
std::array<double, 3> compute_scattering_density(const PassModel& model, const SpectralPassTables& pass,
                                                 const DensityGeometry& geometry, double mu_s, double nu,
                                                 int scattering_order);
// ComputeIndirectIrradiance of a horizontal surface at (r, mu_s) from the order-n sky (n >= 1); 16 x 64 directions.
std::array<double, 3> compute_indirect_irradiance(const PassModel& model, const SpectralPassTables& pass, double r_m,
                                                  double mu_s, int scattering_order);
// ComputeMultipleScattering: the density integrated along the view ray with its transmittance (the adaptive
// quadrature of the single scattering).
std::array<double, 3> compute_multiple_scattering(const PassModel& model, const std::vector<double>& transmittance,
                                                  const std::vector<double>& scattering_density,
                                                  const ScatteringCell& cell);
std::vector<double> compute_scattering_density_table(const PassModel& model, const SpectralPassTables& pass,
                                                     int scattering_order);
std::vector<double> compute_indirect_irradiance_table(const PassModel& model, const SpectralPassTables& pass,
                                                      int scattering_order);
std::vector<double> compute_multiple_scattering_table(const PassModel& model, const std::vector<double>& transmittance,
                                                      const std::vector<double>& scattering_density);

// Runs body(row) for row in [0, rows) on hardware_concurrency threads. The first exception thrown by a body is
// rethrown after all threads have joined (the remaining rows are skipped).
void parallel_for_rows(int rows, const std::function<void(int)>& body);

}  // namespace starsim::environment::detail
