// CPU evaluation of the precomputed atmosphere (M8 atmosphere): Bruneton's rendering functions in luminance mode.
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

#include "starsim/environment/atmosphere_scattering.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include "atmosphere_precompute.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

using math::Vec3;
using Rgb = std::array<double, 3>;

// a / b of WGS84: the polar stretch of the model space.
constexpr double polar_stretch = 1.0 / (1.0 - constants::wgs84_flattening);

double clamp_cosine(double mu) { return std::clamp(mu, -1.0, 1.0); }
double clamp_radius(const AtmosphereParameters& a, double r) { return std::clamp(r, a.bottom_radius_m, a.top_radius_m); }
double smoothstep(double edge0, double edge1, double x) {
    const double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
Vec3 to_vec(const Rgb& c) { return {c[0], c[1], c[2]}; }

// A point in model space, clamped onto the model ground from below.
Vec3 clamp_to_ground(const AtmosphereParameters& a, const Vec3& p) {
    const double r = math::norm(p);
    if (r >= a.bottom_radius_m) return p;
    if (r == 0.0) return {a.bottom_radius_m, 0.0, 0.0};
    return p * (a.bottom_radius_m / r);
}

// GetTransmittanceToTopAtmosphereBoundary (RGB of the table).
Rgb transmittance_to_top(const AtmosphereTables& t, double r, double mu) {
    const TextureUv uv = transmittance_uv_from_r_mu(t.parameters, t.sizes, r, mu);
    return detail::sample_2d<3, 4>(t.transmittance, t.sizes.transmittance_mu, t.sizes.transmittance_r, uv.u, uv.v);
}

// GetTransmittance: between the point at (r, mu) and the point at distance d along the ray.
Rgb transmittance(const AtmosphereTables& t, double r, double mu, double d, bool ray_r_mu_intersects_ground) {
    const double r_d = clamp_radius(t.parameters, std::sqrt(d * d + 2.0 * r * mu * d + r * r));
    const double mu_d = clamp_cosine((r * mu + d) / r_d);
    Rgb num{}, den{};
    if (ray_r_mu_intersects_ground) {
        num = transmittance_to_top(t, r_d, -mu_d);
        den = transmittance_to_top(t, r, -mu);
    } else {
        num = transmittance_to_top(t, r, mu);
        den = transmittance_to_top(t, r_d, mu_d);
    }
    Rgb out{};
    for (int k = 0; k < 3; ++k) out[k] = den[k] > 0.0 ? std::min(num[k] / den[k], 1.0) : 0.0;
    return out;
}

// GetTransmittanceToSun: the transmittance times the fraction of the Sun disc above the model horizon.
Rgb transmittance_to_sun(const AtmosphereTables& t, double r, double mu_s) {
    const double sin_theta_h = t.parameters.bottom_radius_m / r;
    const double cos_theta_h = -std::sqrt(std::max(1.0 - sin_theta_h * sin_theta_h, 0.0));
    const double alpha = t.parameters.sun_angular_radius_rad;
    const double visible = smoothstep(-sin_theta_h * alpha, sin_theta_h * alpha, mu_s - cos_theta_h);
    Rgb out = transmittance_to_top(t, r, mu_s);
    for (double& v : out) v *= visible;
    return out;
}

// GetCombinedScattering with separate textures: the scattering (Rayleigh + multiple) and the single Mie scattering,
// sharing one coordinate computation.
struct Combined {
    Rgb scattering{}, single_mie{};
};
Combined combined_scattering(const AtmosphereTables& t, double r, double mu, double mu_s, double nu,
                             bool ray_r_mu_intersects_ground) {
    const ScatteringUvwz uvwz =
        scattering_uvwz_from_cell(t.parameters, t.sizes, {r, mu, mu_s, nu, ray_r_mu_intersects_ground});
    const detail::ScatteringTaps taps = detail::scattering_taps(t.sizes, uvwz);
    return {detail::sample_taps<3, 4>(t.scattering, taps), detail::sample_taps<3, 4>(t.single_mie, taps)};
}

// GetIrradiance (the sky irradiance table).
Rgb irradiance(const AtmosphereTables& t, double r, double mu_s) {
    const TextureUv uv = irradiance_uv_from_r_mu_s(t.parameters, t.sizes, r, mu_s);
    return detail::sample_2d<3, 4>(t.irradiance, t.sizes.irradiance_mu_s, t.sizes.irradiance_r, uv.u, uv.v);
}

Vec3 sky_luminance_rgb(const AtmosphereParameters& a, const Rgb& radiance) {
    return {radiance[0] * a.sky_radiance_to_luminance[0], radiance[1] * a.sky_radiance_to_luminance[1],
            radiance[2] * a.sky_radiance_to_luminance[2]};
}

}  // namespace

Vec3 ellipsoid_to_atmosphere_space(const Vec3& position_itrf_m) {
    return {position_itrf_m.x, position_itrf_m.y, position_itrf_m.z * polar_stretch};
}

Vec3 ellipsoid_direction_to_atmosphere_space(const Vec3& direction_itrf) {
    return math::normalized(Vec3{direction_itrf.x, direction_itrf.y, direction_itrf.z * polar_stretch});
}

Vec3 ellipsoid_normal_to_atmosphere_space(const Vec3& normal_itrf) {
    return math::normalized(Vec3{normal_itrf.x, normal_itrf.y, normal_itrf.z / polar_stretch});
}

AtmosphereScattering::AtmosphereScattering(std::shared_ptr<const AtmosphereTables> tables)
    : tables_(std::move(tables)) {
    if (!tables_) throw std::invalid_argument("AtmosphereScattering: no tables");
}

Vec3 AtmosphereScattering::sun_transmittance(const Vec3& position_itrf_m, const Vec3& sun_direction_itrf) const {
    const AtmosphereTables& t = *tables_;
    const Vec3 p = clamp_to_ground(t.parameters, ellipsoid_to_atmosphere_space(position_itrf_m));
    const Vec3 sun = ellipsoid_direction_to_atmosphere_space(sun_direction_itrf);
    const double r = math::norm(p);
    if (r > t.parameters.top_radius_m) {
        // Outside the atmosphere: the Sun is seen through it only if the ray to it crosses it.
        const double rmu = math::dot(p, sun);
        const double discriminant = rmu * rmu - r * r + t.parameters.top_radius_m * t.parameters.top_radius_m;
        if (rmu >= 0.0 || discriminant < 0.0) return {1.0, 1.0, 1.0};
        const Vec3 entry = p + sun * (-rmu - std::sqrt(discriminant));
        return to_vec(transmittance_to_sun(t, t.parameters.top_radius_m,
                                           clamp_cosine(math::dot(entry, sun) / t.parameters.top_radius_m)));
    }
    return to_vec(transmittance_to_sun(t, r, clamp_cosine(math::dot(p, sun) / r)));
}

Vec3 AtmosphereScattering::sky_luminance(const Vec3& camera_itrf_m, const Vec3& view_direction_itrf,
                                         const Vec3& sun_direction_itrf, Vec3* transmittance_out) const {
    const AtmosphereTables& t = *tables_;
    const AtmosphereParameters& a = t.parameters;
    Vec3 camera = clamp_to_ground(a, ellipsoid_to_atmosphere_space(camera_itrf_m));
    const Vec3 view_ray = ellipsoid_direction_to_atmosphere_space(view_direction_itrf);
    const Vec3 sun_direction = ellipsoid_direction_to_atmosphere_space(sun_direction_itrf);

    // GetSkyRadiance (shadow_length 0). A viewer in space whose ray meets the atmosphere moves to its top boundary.
    double r = math::norm(camera);
    double rmu = math::dot(camera, view_ray);
    const double discriminant = rmu * rmu - r * r + a.top_radius_m * a.top_radius_m;
    const double distance_to_top = discriminant >= 0.0 ? -rmu - std::sqrt(discriminant) : -1.0;
    if (r > a.top_radius_m) {
        if (distance_to_top <= 0.0) {
            if (transmittance_out) *transmittance_out = {1.0, 1.0, 1.0};
            return {};
        }
        camera = camera + view_ray * distance_to_top;
        r = a.top_radius_m;
        rmu += distance_to_top;
    }
    const double mu = clamp_cosine(rmu / r);
    const double mu_s = clamp_cosine(math::dot(camera, sun_direction) / r);
    const double nu = clamp_cosine(math::dot(view_ray, sun_direction));
    const bool ground = ray_intersects_ground(a, r, mu);
    if (transmittance_out) *transmittance_out = ground ? Vec3{} : to_vec(transmittance_to_top(t, r, mu));
    const Combined s = combined_scattering(t, r, mu, mu_s, nu, ground);
    const double pr = rayleigh_phase_function(nu), pm = mie_phase_function(a.mie_phase_g, nu);
    Rgb radiance{};
    for (int k = 0; k < 3; ++k) radiance[k] = s.scattering[k] * pr + s.single_mie[k] * pm;
    return sky_luminance_rgb(a, radiance);
}

Vec3 AtmosphereScattering::sky_luminance_to_point(const Vec3& camera_itrf_m, const Vec3& point_itrf_m,
                                                  const Vec3& sun_direction_itrf, Vec3* transmittance_out) const {
    const AtmosphereTables& t = *tables_;
    const AtmosphereParameters& a = t.parameters;
    Vec3 camera = clamp_to_ground(a, ellipsoid_to_atmosphere_space(camera_itrf_m));
    const Vec3 point = clamp_to_ground(a, ellipsoid_to_atmosphere_space(point_itrf_m));
    const Vec3 sun_direction = ellipsoid_direction_to_atmosphere_space(sun_direction_itrf);
    const double length_m = math::norm(point - camera);
    if (length_m == 0.0) {
        if (transmittance_out) *transmittance_out = {1.0, 1.0, 1.0};
        return {};
    }
    const Vec3 view_ray = (point - camera) / length_m;

    // GetSkyRadianceToPoint (shadow_length 0).
    double r = math::norm(camera);
    double rmu = math::dot(camera, view_ray);
    const double discriminant = rmu * rmu - r * r + a.top_radius_m * a.top_radius_m;
    const double distance_to_top = discriminant >= 0.0 ? -rmu - std::sqrt(discriminant) : -1.0;
    if (distance_to_top > 0.0) {
        camera = camera + view_ray * distance_to_top;
        r = a.top_radius_m;
        rmu += distance_to_top;
    }
    if (r > a.top_radius_m) {
        // The segment never enters the atmosphere.
        if (transmittance_out) *transmittance_out = {1.0, 1.0, 1.0};
        return {};
    }
    const double mu = clamp_cosine(rmu / r);
    const double mu_s = clamp_cosine(math::dot(camera, sun_direction) / r);
    const double nu = clamp_cosine(math::dot(view_ray, sun_direction));
    const double d = math::norm(point - camera);
    const bool ground = ray_intersects_ground(a, r, mu);
    const Rgb tr = transmittance(t, r, mu, d, ground);
    if (transmittance_out) *transmittance_out = to_vec(tr);
    const Combined s = combined_scattering(t, r, mu, mu_s, nu, ground);

    const double r_p = clamp_radius(a, std::sqrt(d * d + 2.0 * r * mu * d + r * r));
    const double mu_p = clamp_cosine((r * mu + d) / r_p);
    const double mu_s_p = clamp_cosine((r * mu_s + d * nu) / r_p);
    const Combined s_p = combined_scattering(t, r_p, mu_p, mu_s_p, nu, ground);

    // Bruneton's hack against artefacts with the Sun below the horizon: fade the single Mie term out.
    const double mie_fade = smoothstep(0.0, 0.01, mu_s);
    const double pr = rayleigh_phase_function(nu), pm = mie_phase_function(a.mie_phase_g, nu);
    Rgb radiance{};
    for (int k = 0; k < 3; ++k) {
        const double scattering = s.scattering[k] - tr[k] * s_p.scattering[k];
        const double single_mie = (s.single_mie[k] - tr[k] * s_p.single_mie[k]) * mie_fade;
        radiance[k] = scattering * pr + single_mie * pm;
    }
    return sky_luminance_rgb(a, radiance);
}

AtmosphereScattering::Illuminance AtmosphereScattering::sun_and_sky_illuminance(const Vec3& point_itrf_m,
                                                                               const Vec3& normal_itrf,
                                                                               const Vec3& sun_direction_itrf) const {
    const AtmosphereTables& t = *tables_;
    const AtmosphereParameters& a = t.parameters;
    // GetSunAndSkyIrradiance at a point inside the atmosphere (above the top: the top boundary's values).
    Vec3 p = clamp_to_ground(a, ellipsoid_to_atmosphere_space(point_itrf_m));
    double r = math::norm(p);
    if (r > a.top_radius_m) {
        p = p * (a.top_radius_m / r);
        r = a.top_radius_m;
    }
    const Vec3 sun = ellipsoid_direction_to_atmosphere_space(sun_direction_itrf);
    const Vec3 normal = ellipsoid_normal_to_atmosphere_space(normal_itrf);
    const double mu_s = clamp_cosine(math::dot(p, sun) / r);

    Illuminance e;
    // Sky: the horizontal irradiance, tilted by (1 + n.up) / 2.
    const Rgb sky = irradiance(t, r, mu_s);
    const double tilt = (1.0 + math::dot(normal, p) / r) * 0.5;
    e.sky_lux = {sky[0] * tilt * a.sky_radiance_to_luminance[0], sky[1] * tilt * a.sky_radiance_to_luminance[1],
                 sky[2] * tilt * a.sky_radiance_to_luminance[2]};
    // Sun: the cosine factor in the true (ITRF) geometry - the mapping preserves its sign but not its magnitude.
    const double cosine = std::max(math::dot(math::normalized(normal_itrf), math::normalized(sun_direction_itrf)), 0.0);
    const Rgb tr = transmittance_to_sun(t, r, mu_s);
    e.sun_lux = {a.solar_illuminance_rgb[0] * tr[0] * cosine * a.sun_radiance_to_luminance[0],
                 a.solar_illuminance_rgb[1] * tr[1] * cosine * a.sun_radiance_to_luminance[1],
                 a.solar_illuminance_rgb[2] * tr[2] * cosine * a.sun_radiance_to_luminance[2]};
    return e;
}

}  // namespace starsim::environment
