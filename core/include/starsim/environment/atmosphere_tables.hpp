#pragma once

// Precomputed atmospheric scattering tables of the Earth (M8 atmosphere): table layout, Bruneton's parameterisation
// and the sampling helpers shared by the precomputation, the CPU lookups and (mirrored) the shaders.
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

#include <array>
#include <cstdint>
#include <vector>

#include "starsim/config/atmosphere_optics_config.hpp"

namespace starsim::environment {

// Table sizes: Bruneton's defaults (constants.h) except the transmittance table, 1024 x 1024 instead of 256 x 64: the
// table is linear in T, and near the ground horizon T falls by orders of magnitude between Bruneton's texels (sea-level
// horizon ray: 82 % error in blue at 256 x 64, 0.3 % at 1024 x 1024; 512 rows still 1.1 %); the single scattering
// reads T through it and inherits the error (up to 20 % at 256 x 64, 0.12 % at 1024 x 1024; Task 2 report).
// 16 MB as float RGBA.
struct AtmosphereTableSizes {
    int transmittance_mu = 1024, transmittance_r = 1024;
    int scattering_r = 32, scattering_mu = 128, scattering_mu_s = 32, scattering_nu = 8;
    int irradiance_mu_s = 64, irradiance_r = 16;

    int scattering_width() const { return scattering_nu * scattering_mu_s; }
    int scattering_height() const { return scattering_mu; }
    int scattering_depth() const { return scattering_r; }
};

// Everything the runtime lookups need (CPU and shader). Radii in metres from the Earth's centre in the spherical model
// space (the WGS84 ellipsoid mapped onto the sphere of radius a). Spectral quantities are per metre of wavelength.
struct AtmosphereParameters {
    double bottom_radius_m = 0.0;         // a + ground_altitude_m
    double top_radius_m = 0.0;            // a + top_altitude_m
    double mu_s_min = 0.0;                // cosine of the largest precomputed Sun zenith angle (Bruneton's demo: 102 deg)
    double mie_phase_g = 0.0;             // Cornette-Shanks asymmetry
    double sun_angular_radius_rad = 0.0;  // at 1 AU
    // Bruneton's solar_irradiance: TOA spectral irradiance at 1 AU at kLambdaR / G / B (680 / 550 / 440 nm), in
    // W m^-2 m^-1 (Bruneton: per nm). The name follows the interface; it is not an illuminance.
    std::array<double, 3> solar_illuminance_rgb{};
    // Bruneton's SKY_SPECTRAL_RADIANCE_TO_LUMINANCE (precomputed-luminance mode: 683 lm/W, the tables hold radiance
    // already folded over the spectrum) and SUN_SPECTRAL_RADIANCE_TO_LUMINANCE (lm m / W: applied to the spectral
    // solar radiance at kLambdaR / G / B, ComputeSpectralRadianceToLuminanceFactors with lambda_power 0).
    std::array<double, 3> sky_radiance_to_luminance{}, sun_radiance_to_luminance{};
    double toa_solar_illuminance_lux = 0.0;  // E_v at the top of the atmosphere at 1 AU (683 lm/W x int ybar E dlambda)
};

// The precomputed tables, float32 RGBA texels (row-major, x fastest).
//  transmittance: transmittance_mu x transmittance_r; RGB = transmittance to the top boundary at kLambdaR / G / B,
//    A = 0 (unused).
//  scattering: width = scattering_nu x scattering_mu_s ((nu, mu_s) packed in x, mu_s fastest within each nu slice),
//    height = scattering_mu, depth = scattering_r. RGB = Rayleigh (and, from Task 3, multiple) scattering without the
//    phase function, A = the red channel of the single Mie scattering (Bruneton's combined layout). Luminance units:
//    radiance folded over the spectrum (W m^-2 sr^-1 per unit of the sRGB CMFs); x sky_radiance_to_luminance -> cd/m^2.
//  single_mie: same layout, RGB = single Mie scattering without the phase function, A = 0.
//  irradiance: irradiance_mu_s x irradiance_r. Bruneton's irradiance texture holds only the sky (indirect) irradiance,
//    which starts at scattering order 2: after this task's single-scattering precomputation (orders = 1) it is sized
//    but zero; Task 3 accumulates the indirect irradiance into it. The direct irradiance is a pass-internal delta table
//    (see atmosphere_precompute.hpp), never stored here - as in Bruneton.
struct AtmosphereTables {
    AtmosphereTableSizes sizes;
    AtmosphereParameters parameters;
    std::uint64_t input_hash = 0;
    std::vector<float> transmittance;
    std::vector<float> scattering;
    std::vector<float> single_mie;
    std::vector<float> irradiance;
};

// Version of the precomputation code and its tabulated spectra; part of the input hash. Bump on every change that
// alters table contents (code, spectra in atmosphere_spectra.hpp, Task 1 optics).
inline constexpr std::uint64_t atmosphere_tables_code_version = 1;

// The parameters of a configuration (radii, mu_s_min, luminance factors, solar irradiance).
AtmosphereParameters atmosphere_parameters(const config::AtmosphereOpticsConfig& optics);

// Precomputes the tables. This task fills transmittance and single scattering (orders = 1); the multiple scattering
// orders and the irradiance of config::AtmosphereOpticsConfig::scattering_orders follow in Task 3. Throws
// std::invalid_argument on invalid sizes (any < 2, odd scattering_mu).
AtmosphereTables precompute_atmosphere(const config::AtmosphereOpticsConfig& optics,
                                       const AtmosphereTableSizes& sizes = {});

// FNV-1a 64 over the code version, every config field and every table size.
std::uint64_t atmosphere_input_hash(const config::AtmosphereOpticsConfig& optics, const AtmosphereTableSizes& sizes);

// ---- Ray geometry (Bruneton functions.glsl). r: distance from the centre (m); mu: cosine of the view zenith angle.
double distance_to_top_atmosphere_boundary_m(const AtmosphereParameters& atmosphere, double r_m, double mu);
double distance_to_bottom_atmosphere_boundary_m(const AtmosphereParameters& atmosphere, double r_m, double mu);
bool ray_intersects_ground(const AtmosphereParameters& atmosphere, double r_m, double mu);

// ---- Texture parameterisation (Bruneton functions.glsl).
double texture_coord_from_unit_range(double x, int texture_size);
double unit_range_from_texture_coord(double u, int texture_size);

struct TextureUv {
    double u = 0.0, v = 0.0;
};
struct RMu {
    double r_m = 0.0, mu = 0.0;
};
TextureUv transmittance_uv_from_r_mu(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                     double r_m, double mu);
RMu r_mu_from_transmittance_uv(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                               TextureUv uv);

// GetScatteringTextureUvwzFromRMuMuSNu: x = u_nu, y = u_mu_s, z = u_mu, w = u_r.
struct ScatteringUvwz {
    double u_nu = 0.0, u_mu_s = 0.0, u_mu = 0.0, u_r = 0.0;
};
struct ScatteringCell {
    double r_m = 0.0, mu = 0.0, mu_s = 0.0, nu = 0.0;
    bool ray_r_mu_intersects_ground = false;
};
ScatteringUvwz scattering_uvwz_from_cell(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                         const ScatteringCell& cell);
ScatteringCell scattering_cell_from_uvwz(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                         const ScatteringUvwz& uvwz);
// The cell at the centre of texel (x, y, z) of the scattering table, nu clamped to its range given mu and mu_s
// (GetRMuMuSNuFromScatteringTextureFragCoord).
ScatteringCell scattering_cell_at_texel(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                        int x, int y, int z);

TextureUv irradiance_uv_from_r_mu_s(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                    double r_m, double mu_s);
RMu r_mu_s_from_irradiance_uv(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes, TextureUv uv);

// ---- Table lookups with GPU linear filtering (texel centres at (i + 0.5) / size, clamp to edge).
// GetTransmittanceToTopAtmosphereBoundary: RGBA of the transmittance table.
std::array<double, 4> lookup_transmittance_to_top(const AtmosphereTables& tables, double r_m, double mu);
// GetScattering: a scattering-layout table (scattering or single_mie) at (r, mu, mu_s, nu), interpolated in nu
// between the two nu slices as in Bruneton.
std::array<double, 4> lookup_scattering(const AtmosphereTables& tables, const std::vector<float>& table,
                                        const ScatteringCell& cell);

}  // namespace starsim::environment
