// Table layout, Bruneton's parameterisation and the table lookups of the precomputed atmosphere (M8 atmosphere).
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

#include "starsim/environment/atmosphere_tables.hpp"

#include <algorithm>
#include <cmath>

#include "atmosphere_precompute.hpp"
#include "starsim/environment/atmosphere_optics.hpp"
#include "starsim/environment/atmosphere_spectra.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr double nm = 1e-9;

// Largest precomputed Sun zenith angle [S: Bruneton 2017 demo.cc, 120 deg for float textures; controller ruling]: the
// Sun up to 30 deg below the horizon, so the whole twilight down to -18 deg (astronomical, star visibility) and the
// horizon dip seen from the 100 km top (10 deg) lie inside the tables.
constexpr double max_sun_zenith_angle_rad = 120.0 * constants::deg_to_rad;

double clamp_cosine(double mu) { return std::clamp(mu, -1.0, 1.0); }
double clamp_distance(double d) { return std::max(d, 0.0); }
double safe_sqrt(double a) { return std::sqrt(std::max(a, 0.0)); }

// CieColorMatchingFunctionTableValue of Bruneton's model.cc: zero at and outside 360 / 830 nm.
math::Vec3 cmf_bruneton(double wavelength_nm) {
    if (wavelength_nm <= spectra::lambda_min_nm || wavelength_nm >= spectra::lambda_max_nm) return {};
    return cie_1931_cmf(wavelength_nm * nm);
}

// ComputeSpectralRadianceToLuminanceFactors (model.cc) with lambda_power 0, in lm m / W (Bruneton: lm nm / W).
std::array<double, 3> sun_radiance_to_luminance_factors() {
    const std::array<double, 3> lambda_c{spectra::lambda_r_nm, spectra::lambda_g_nm, spectra::lambda_b_nm};
    std::array<double, 3> solar_c{};
    for (int c = 0; c < 3; ++c) solar_c[c] = solar_irradiance_w_m2_per_m(lambda_c[c] * nm);
    std::array<double, 3> k{};
    const int dlambda_nm = 1;
    for (int lambda = static_cast<int>(spectra::lambda_min_nm); lambda < static_cast<int>(spectra::lambda_max_nm);
         lambda += dlambda_nm) {
        const math::Vec3 xyz = cmf_bruneton(lambda);
        const double irradiance = solar_irradiance_w_m2_per_m(lambda * nm);
        for (int c = 0; c < 3; ++c) {
            const double* row = spectra::xyz_to_linear_srgb + 3 * c;
            k[c] += (row[0] * xyz.x + row[1] * xyz.y + row[2] * xyz.z) * irradiance / solar_c[c];
        }
    }
    for (double& v : k) v *= constants::max_luminous_efficacy_lm_per_w * dlambda_nm * nm;
    return k;
}

// E_v = 683 lm/W x integral of ybar E over 360-830 nm (1 nm steps, as the luminance factors).
double toa_solar_illuminance_lux() {
    double sum = 0.0;
    for (int lambda = static_cast<int>(spectra::lambda_min_nm); lambda < static_cast<int>(spectra::lambda_max_nm);
         ++lambda) {
        sum += cmf_bruneton(lambda).y * solar_irradiance_w_m2_per_m(lambda * nm);
    }
    return constants::max_luminous_efficacy_lm_per_w * sum * nm;
}

void hash_bytes(std::uint64_t& h, const void* data, std::size_t size) {
    const auto* p = static_cast<const unsigned char*>(data);
    for (std::size_t i = 0; i < size; ++i) {
        h ^= p[i];
        h *= 1099511628211ull;  // FNV-1a 64 prime
    }
}
template <class T>
void hash_value(std::uint64_t& h, T value) {
    hash_bytes(h, &value, sizeof(value));
}

}  // namespace

AtmosphereParameters atmosphere_parameters(const config::AtmosphereOpticsConfig& optics) {
    AtmosphereParameters p;
    const double a = constants::wgs84_semi_major_axis_m;
    p.bottom_radius_m = a + optics.ground_altitude_m;
    p.top_radius_m = a + optics.top_altitude_m;
    p.mu_s_min = std::cos(max_sun_zenith_angle_rad);
    p.mie_phase_g = optics.mie_phase_g;
    p.sun_angular_radius_rad = std::asin(constants::sun_radius_m / constants::astronomical_unit_m);
    p.solar_illuminance_rgb = {solar_irradiance_w_m2_per_m(spectra::lambda_r_nm * nm),
                               solar_irradiance_w_m2_per_m(spectra::lambda_g_nm * nm),
                               solar_irradiance_w_m2_per_m(spectra::lambda_b_nm * nm)};
    // Precomputed-luminance mode: the tables are already folded over the spectrum (Model constructor).
    p.sky_radiance_to_luminance.fill(constants::max_luminous_efficacy_lm_per_w);
    p.sun_radiance_to_luminance = sun_radiance_to_luminance_factors();
    p.toa_solar_illuminance_lux = toa_solar_illuminance_lux();
    return p;
}

std::uint64_t atmosphere_spectral_data_checksum() {
    std::uint64_t h = 14695981039346656037ull;  // FNV-1a 64 offset basis
    // The tabulated data as compiled - raw inputs only. Function outputs (std::exp / std::pow of the optics, the US76
    // profile) are not hashed: they come from the OS runtime (ucrtbase) and may differ in the last bit between
    // machines, which would reject a shipped cache. Code changes are covered by atmosphere_tables_code_version.
    hash_bytes(h, spectra::solar_irradiance_w_m2_nm, sizeof(spectra::solar_irradiance_w_m2_nm));
    hash_bytes(h, spectra::ozone_cross_section_m2, sizeof(spectra::ozone_cross_section_m2));
    hash_bytes(h, spectra::cie_1931_2deg_cmf, sizeof(spectra::cie_1931_2deg_cmf));
    hash_bytes(h, spectra::xyz_to_linear_srgb, sizeof(spectra::xyz_to_linear_srgb));
    for (const double v : {spectra::bin_first_wavelength_nm, spectra::bin_width_nm, spectra::lambda_r_nm,
                           spectra::lambda_g_nm, spectra::lambda_b_nm, spectra::lambda_min_nm, spectra::lambda_max_nm}) {
        hash_value(h, v);
    }
    hash_value(h, atmosphere_optics_data_checksum());  // depolarisation table, AFGL ozone profile
    return h;
}

std::uint64_t atmosphere_input_hash(const config::AtmosphereOpticsConfig& optics, const AtmosphereTableSizes& sizes) {
    std::uint64_t h = 14695981039346656037ull;  // FNV-1a 64 offset basis
    hash_value(h, atmosphere_tables_code_version);
    hash_value(h, atmosphere_spectral_data_checksum());
    for (const double v : {optics.top_altitude_m, optics.ground_altitude_m, optics.aerosol_optical_depth_550nm,
                           optics.aerosol_angstrom_exponent, optics.aerosol_single_scattering_albedo,
                           optics.aerosol_scale_height_m, optics.mie_phase_g, optics.ozone_column_du,
                           optics.ground_albedo}) {
        hash_value(h, v);
    }
    hash_value(h, optics.scattering_orders);
    hash_value(h, optics.multiple_scattering_tail);
    for (const int v : {sizes.transmittance_mu, sizes.transmittance_r, sizes.scattering_r, sizes.scattering_mu,
                        sizes.scattering_mu_s, sizes.scattering_nu, sizes.irradiance_mu_s, sizes.irradiance_r}) {
        hash_value(h, v);
    }
    return h;
}

// ---- Phase functions.

double rayleigh_phase_function(double nu) {
    const double k = 3.0 / (16.0 * constants::pi);
    return k * (1.0 + nu * nu);
}

double mie_phase_function(double g, double nu) {
    const double k = 3.0 / (8.0 * constants::pi) * (1.0 - g * g) / (2.0 + g * g);
    const double x = 1.0 + g * g - 2.0 * g * nu;
    return k * (1.0 + nu * nu) / (x * std::sqrt(x));  // x^1.5
}

// ---- Ray geometry.

double distance_to_top_atmosphere_boundary_m(const AtmosphereParameters& atmosphere, double r_m, double mu) {
    const double discriminant = r_m * r_m * (mu * mu - 1.0) + atmosphere.top_radius_m * atmosphere.top_radius_m;
    return clamp_distance(-r_m * mu + safe_sqrt(discriminant));
}

double distance_to_bottom_atmosphere_boundary_m(const AtmosphereParameters& atmosphere, double r_m, double mu) {
    const double discriminant = r_m * r_m * (mu * mu - 1.0) + atmosphere.bottom_radius_m * atmosphere.bottom_radius_m;
    return clamp_distance(-r_m * mu - safe_sqrt(discriminant));
}

bool ray_intersects_ground(const AtmosphereParameters& atmosphere, double r_m, double mu) {
    return mu < 0.0 && r_m * r_m * (mu * mu - 1.0) + atmosphere.bottom_radius_m * atmosphere.bottom_radius_m >= 0.0;
}

// ---- Parameterisation.

double texture_coord_from_unit_range(double x, int texture_size) {
    return 0.5 / texture_size + x * (1.0 - 1.0 / texture_size);
}

double unit_range_from_texture_coord(double u, int texture_size) {
    return (u - 0.5 / texture_size) / (1.0 - 1.0 / texture_size);
}

namespace {
double horizontal_distance_to_top_m(const AtmosphereParameters& a) {
    return std::sqrt(a.top_radius_m * a.top_radius_m - a.bottom_radius_m * a.bottom_radius_m);
}
}  // namespace

TextureUv transmittance_uv_from_r_mu(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                     double r_m, double mu) {
    const double H = horizontal_distance_to_top_m(atmosphere);
    const double rho = safe_sqrt(r_m * r_m - atmosphere.bottom_radius_m * atmosphere.bottom_radius_m);
    const double d = distance_to_top_atmosphere_boundary_m(atmosphere, r_m, mu);
    const double d_min = atmosphere.top_radius_m - r_m;
    const double d_max = rho + H;
    const double x_mu = (d - d_min) / (d_max - d_min);
    const double x_r = rho / H;
    return {texture_coord_from_unit_range(x_mu, sizes.transmittance_mu),
            texture_coord_from_unit_range(x_r, sizes.transmittance_r)};
}

RMu r_mu_from_transmittance_uv(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                               TextureUv uv) {
    const double x_mu = unit_range_from_texture_coord(uv.u, sizes.transmittance_mu);
    const double x_r = unit_range_from_texture_coord(uv.v, sizes.transmittance_r);
    const double H = horizontal_distance_to_top_m(atmosphere);
    const double rho = H * x_r;
    RMu out;
    out.r_m = std::sqrt(rho * rho + atmosphere.bottom_radius_m * atmosphere.bottom_radius_m);
    const double d_min = atmosphere.top_radius_m - out.r_m;
    const double d_max = rho + H;
    const double d = d_min + x_mu * (d_max - d_min);
    out.mu = d == 0.0 ? 1.0 : (H * H - rho * rho - d * d) / (2.0 * out.r_m * d);
    out.mu = clamp_cosine(out.mu);
    return out;
}

ScatteringUvwz scattering_uvwz_from_cell(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                         const ScatteringCell& cell) {
    const double r = cell.r_m, mu = cell.mu;
    const double bottom = atmosphere.bottom_radius_m;
    const double H = horizontal_distance_to_top_m(atmosphere);
    const double rho = safe_sqrt(r * r - bottom * bottom);
    ScatteringUvwz uvwz;
    uvwz.u_r = texture_coord_from_unit_range(rho / H, sizes.scattering_r);

    const double r_mu = r * mu;
    const double discriminant = r_mu * r_mu - r * r + bottom * bottom;
    const int half_mu = sizes.scattering_mu / 2;
    if (cell.ray_r_mu_intersects_ground) {
        const double d = -r_mu - safe_sqrt(discriminant);
        const double d_min = r - bottom;
        const double d_max = rho;
        uvwz.u_mu = 0.5 - 0.5 * texture_coord_from_unit_range(
                                    d_max == d_min ? 0.0 : (d - d_min) / (d_max - d_min), half_mu);
    } else {
        const double d = -r_mu + safe_sqrt(discriminant + H * H);
        const double d_min = atmosphere.top_radius_m - r;
        const double d_max = rho + H;
        uvwz.u_mu = 0.5 + 0.5 * texture_coord_from_unit_range((d - d_min) / (d_max - d_min), half_mu);
    }

    uvwz.u_mu_s = scattering_u_mu_s(atmosphere, sizes, cell.mu_s);
    uvwz.u_nu = (cell.nu + 1.0) / 2.0;
    return uvwz;
}

double scattering_u_mu_s(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes, double mu_s) {
    const double bottom = atmosphere.bottom_radius_m;
    const double d = distance_to_top_atmosphere_boundary_m(atmosphere, bottom, mu_s);
    const double d_min = atmosphere.top_radius_m - bottom;
    const double d_max = horizontal_distance_to_top_m(atmosphere);
    const double a = (d - d_min) / (d_max - d_min);
    const double D = distance_to_top_atmosphere_boundary_m(atmosphere, bottom, atmosphere.mu_s_min);
    const double A = (D - d_min) / (d_max - d_min);
    // Bruneton's ad-hoc mapping: 0 at mu_s_min, 1 at mu_s = 1, steep around mu_s = 0.
    return texture_coord_from_unit_range(std::max(1.0 - a / A, 0.0) / (1.0 + a), sizes.scattering_mu_s);
}

ScatteringCell scattering_cell_from_uvwz(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                         const ScatteringUvwz& uvwz) {
    const double bottom = atmosphere.bottom_radius_m;
    const double H = horizontal_distance_to_top_m(atmosphere);
    const double rho = H * unit_range_from_texture_coord(uvwz.u_r, sizes.scattering_r);
    ScatteringCell cell;
    cell.r_m = std::sqrt(rho * rho + bottom * bottom);
    const double r = cell.r_m;
    const int half_mu = sizes.scattering_mu / 2;
    if (uvwz.u_mu < 0.5) {
        const double d_min = r - bottom;
        const double d_max = rho;
        const double d = d_min + (d_max - d_min) * unit_range_from_texture_coord(1.0 - 2.0 * uvwz.u_mu, half_mu);
        cell.mu = d == 0.0 ? -1.0 : clamp_cosine(-(rho * rho + d * d) / (2.0 * r * d));
        cell.ray_r_mu_intersects_ground = true;
    } else {
        const double d_min = atmosphere.top_radius_m - r;
        const double d_max = rho + H;
        const double d = d_min + (d_max - d_min) * unit_range_from_texture_coord(2.0 * uvwz.u_mu - 1.0, half_mu);
        cell.mu = d == 0.0 ? 1.0 : clamp_cosine((H * H - rho * rho - d * d) / (2.0 * r * d));
        cell.ray_r_mu_intersects_ground = false;
    }

    const double x_mu_s = unit_range_from_texture_coord(uvwz.u_mu_s, sizes.scattering_mu_s);
    const double d_min = atmosphere.top_radius_m - bottom;
    const double d_max = H;
    const double D = distance_to_top_atmosphere_boundary_m(atmosphere, bottom, atmosphere.mu_s_min);
    const double A = (D - d_min) / (d_max - d_min);
    const double a = (A - x_mu_s * A) / (1.0 + x_mu_s * A);
    const double d = d_min + std::min(a, A) * (d_max - d_min);
    cell.mu_s = d == 0.0 ? 1.0 : clamp_cosine((H * H - d * d) / (2.0 * bottom * d));
    cell.nu = clamp_cosine(uvwz.u_nu * 2.0 - 1.0);
    return cell;
}

ScatteringCell scattering_cell_at_texel(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                        int x, int y, int z) {
    // Fragment coordinates of the texel centre; nu has no half-texel offset (Bruneton: size nu - 1).
    const double frag_x = x + 0.5, frag_y = y + 0.5, frag_z = z + 0.5;
    const double frag_nu = std::floor(frag_x / sizes.scattering_mu_s);
    const double frag_mu_s = std::fmod(frag_x, static_cast<double>(sizes.scattering_mu_s));
    const ScatteringUvwz uvwz{frag_nu / (sizes.scattering_nu - 1), frag_mu_s / sizes.scattering_mu_s,
                              frag_y / sizes.scattering_mu, frag_z / sizes.scattering_r};
    ScatteringCell cell = scattering_cell_from_uvwz(atmosphere, sizes, uvwz);
    const double spread = std::sqrt((1.0 - cell.mu * cell.mu) * (1.0 - cell.mu_s * cell.mu_s));
    cell.nu = std::clamp(cell.nu, cell.mu * cell.mu_s - spread, cell.mu * cell.mu_s + spread);
    return cell;
}

TextureUv irradiance_uv_from_r_mu_s(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes,
                                    double r_m, double mu_s) {
    const double x_r = (r_m - atmosphere.bottom_radius_m) / (atmosphere.top_radius_m - atmosphere.bottom_radius_m);
    const double x_mu_s = mu_s * 0.5 + 0.5;
    return {texture_coord_from_unit_range(x_mu_s, sizes.irradiance_mu_s),
            texture_coord_from_unit_range(x_r, sizes.irradiance_r)};
}

RMu r_mu_s_from_irradiance_uv(const AtmosphereParameters& atmosphere, const AtmosphereTableSizes& sizes, TextureUv uv) {
    const double x_mu_s = unit_range_from_texture_coord(uv.u, sizes.irradiance_mu_s);
    const double x_r = unit_range_from_texture_coord(uv.v, sizes.irradiance_r);
    return {atmosphere.bottom_radius_m + x_r * (atmosphere.top_radius_m - atmosphere.bottom_radius_m),
            clamp_cosine(2.0 * x_mu_s - 1.0)};
}

// ---- Lookups.

std::array<double, 4> lookup_transmittance_to_top(const AtmosphereTables& tables, double r_m, double mu) {
    const TextureUv uv = transmittance_uv_from_r_mu(tables.parameters, tables.sizes, r_m, mu);
    return detail::sample_2d<4>(tables.transmittance, tables.sizes.transmittance_mu, tables.sizes.transmittance_r, uv.u,
                                uv.v);
}

std::array<double, 4> lookup_scattering(const AtmosphereTables& tables, const std::vector<float>& table,
                                        const ScatteringCell& cell) {
    return detail::sample_scattering_layout<4>(table, tables.sizes,
                                               scattering_uvwz_from_cell(tables.parameters, tables.sizes, cell));
}

}  // namespace starsim::environment
