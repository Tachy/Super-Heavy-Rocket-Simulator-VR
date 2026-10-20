#pragma once

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// What a spacecraft surface element sees in Earth orbit (refuelling r2): the Sun, sunlight reflected by the Earth
// (albedo) and the Earth's own infrared. Unit directions in the frame the caller works in.
struct RadiationEnvironment {
    math::Vec3 sun_direction;           // towards the Sun
    double solar_flux_w_m2 = 0.0;       // at the vehicle's distance from the Sun, unshadowed
    double sunlit_fraction = 0.0;       // 0 (umbra) .. 1: the part of the Sun's disk Earth and Moon leave visible
    math::Vec3 nadir_direction;         // towards the Earth's centre
    double distance_over_radius = 0.0;  // distance from the Earth's centre over its mean radius (> 1)
    // Albedo illumination [E]: cosine of the Sun's zenith angle at the point below the vehicle, >= 0 (the usual
    // engineering approximation: the whole visible cap lit like its centre).
    double albedo_phase = 0.0;
    // The Moon (refuelling r5), when given: direction and distance, the albedo phase at the sub-vehicle point (as
    // albedo_phase) and the infrared emitted there [E]: the regolith in radiative equilibrium, max((1 - A) S cos z,
    // sigma T_night^4) (no conduction: the dayside within a few K of Diviner), seen as the whole cap like its centre.
    math::Vec3 moon_direction;               // towards the Moon's centre
    double moon_distance_over_radius = 0.0;  // 0: no Moon in the environment
    double moon_albedo_phase = 0.0;
    double moon_infrared_w_m2 = 0.0;
};

// View factor from a flat element to a sphere, the element's normal at `angle_from_nadir_rad` from the direction to
// the sphere's centre, `distance_over_radius` (> 1) radii from it [S: Howell, Menguc & Siegel, catalog of
// configuration factors, "planar element to sphere, tilted"]: cos(angle) / H^2 while the whole sphere is above the
// element's horizon, the partial-view expression across the horizon, 0 beyond.
double plate_view_factor_to_sphere(double angle_from_nadir_rad, double distance_over_radius);

// The radiation environment of a vehicle at `position_gcrf_m` with the Sun at `sun_from_vehicle_m` and the Moon at
// `*moon_from_vehicle_m` (from the vehicle, geometric as for solar radiation pressure; nullptr: no lunar shadow).
// Spherical Earth of mean radius. A given Moon shadows and adds its albedo and infrared (r5; r2 had shadow only).
RadiationEnvironment earth_orbit_radiation(const math::Vec3& position_gcrf_m, const math::Vec3& sun_from_vehicle_m,
                                           const math::Vec3* moon_from_vehicle_m);

// Flux absorbed by a flat element with outward unit normal `normal` (the environment's frame): absorptivity x (direct
// sun + Earth and Moon albedo) + emissivity x their infrared (Kirchhoff: the infrared absorptivity is the emissivity).
double absorbed_flux_w_m2(const math::Vec3& normal, const RadiationEnvironment& env, double absorptivity,
                          double emissivity);

}  // namespace starsim::environment
