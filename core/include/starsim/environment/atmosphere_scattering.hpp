#pragma once

// CPU evaluation of the precomputed atmosphere (M8 atmosphere): Bruneton's runtime functions GetSkyRadiance,
// GetSkyRadianceToPoint, GetSunAndSkyIrradiance and the transmittance to the Sun, in his precomputed-luminance mode
// (outputs in cd/m^2 and lux, linear sRGB), on Earth-fixed positions. The shader include of Task 5 mirrors this code
// and is checked against it.
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

#include <memory>

#include "starsim/environment/atmosphere_tables.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// The atmosphere's model space: the WGS84 ellipsoid mapped onto the sphere of radius a by scaling z by a/b (a linear
// map M = diag(1, 1, a/b): straight rays stay straight, the ellipsoid surface lands on radius a at every latitude).
// Positions (ITRF metres) map by M. Below the model ground the AtmosphereScattering lookups clamp a point onto it.
math::Vec3 ellipsoid_to_atmosphere_space(const math::Vec3& position_itrf_m);
// Directions (view rays, the Sun) map by M and are renormalised.
math::Vec3 ellipsoid_direction_to_atmosphere_space(const math::Vec3& direction_itrf);
// Surface normals map by M^-T = diag(1, 1, b/a) and are renormalised: the geodetic up becomes the model's radial.
math::Vec3 ellipsoid_normal_to_atmosphere_space(const math::Vec3& normal_itrf);

// Convention for the shader port: the Sun direction maps by M like every ray, normals by M^-T, so in model space the
// Sun's elevation differs from the true one by up to ~0.19 deg at mid-latitudes (zero at the horizon and the poles /
// equator); the lookups (transmittance, scattering, sky irradiance) use the mapped geometry, the direct-sun cosine of
// sun_and_sky_illuminance the true ITRF one. The model ground (-150 m) lies below every drawn surface, so the model
// horizon (where the sky radiance jumps physically, up to ~6 % at 2 m) is always hidden behind terrain or sea.
//
// Runtime lookups into the precomputed tables. Inputs are Earth-fixed (ITRF): positions in metres, directions unit
// vectors. The tables are for the Sun at 1 AU: callers scale the results by 1 / d_AU^2. Thread-safe (const).
class AtmosphereScattering {
public:
    explicit AtmosphereScattering(std::shared_ptr<const AtmosphereTables> tables);

    const AtmosphereTables& tables() const { return *tables_; }

    // Transmittance from the point to the Sun (680 / 550 / 440 nm), with the Sun disc's fraction above the model
    // horizon (GetTransmittanceToSun).
    math::Vec3 sun_transmittance(const math::Vec3& position_itrf_m, const math::Vec3& sun_direction_itrf) const;

    // Sky luminance in cd/m^2 (RGB) seen from the camera along the view direction (GetSkyRadiance x
    // SKY_SPECTRAL_RADIANCE_TO_LUMINANCE); the transmittance of the view ray to the atmosphere's edge (0 for a ray into
    // the ground) in *transmittance. Cameras in space are moved to the top boundary along the ray.
    math::Vec3 sky_luminance(const math::Vec3& camera_itrf_m, const math::Vec3& view_direction_itrf,
                             const math::Vec3& sun_direction_itrf, math::Vec3* transmittance) const;
    // In-scattered luminance between the camera and a point (aerial perspective, GetSkyRadianceToPoint); the
    // transmittance between them in *transmittance.
    math::Vec3 sky_luminance_to_point(const math::Vec3& camera_itrf_m, const math::Vec3& point_itrf_m,
                                      const math::Vec3& sun_direction_itrf, math::Vec3* transmittance) const;

    // Illuminance of a surface element (GetSunAndSkyIrradiance x the luminance factors): the direct Sun through the
    // atmosphere and the sky (the horizontal sky irradiance with Bruneton's (1 + n.up) / 2 tilt approximation), lux RGB.
    struct Illuminance {
        math::Vec3 sun_lux, sky_lux;
    };
    Illuminance sun_and_sky_illuminance(const math::Vec3& point_itrf_m, const math::Vec3& normal_itrf,
                                        const math::Vec3& sun_direction_itrf) const;

private:
    std::shared_ptr<const AtmosphereTables> tables_;
};

}  // namespace starsim::environment
