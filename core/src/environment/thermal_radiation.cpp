#include "starsim/environment/thermal_radiation.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/environment/eclipse.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Vec3;

double plate_view_factor_to_sphere(double angle_from_nadir_rad, double distance_over_radius) {
    const double h = std::max(distance_over_radius, 1.0 + 1e-12);
    const double full_view = std::acos(1.0 / h);  // up to here the whole sphere is above the horizon
    const double cos_a = std::cos(angle_from_nadir_rad);
    if (angle_from_nadir_rad <= full_view) return cos_a / (h * h);
    if (angle_from_nadir_rad >= constants::pi - full_view) return 0.0;
    const double s = std::sqrt(h * h - 1.0);
    const double sin_a = std::sin(angle_from_nadir_rad);
    const double rim = std::sqrt(std::max(0.0, 1.0 - h * h * cos_a * cos_a));
    return 0.5 - std::asin(std::min(1.0, s / (h * sin_a))) / constants::pi +
           (cos_a * std::acos(std::clamp(-s * cos_a / sin_a, -1.0, 1.0)) - s * rim) / (constants::pi * h * h);
}

RadiationEnvironment earth_orbit_radiation(const Vec3& position_gcrf_m, const Vec3& sun_from_vehicle_m,
                                           const Vec3* moon_from_vehicle_m) {
    RadiationEnvironment env;
    const double sun_distance = norm(sun_from_vehicle_m);
    env.sun_direction = sun_from_vehicle_m / sun_distance;
    const double au_ratio = constants::astronomical_unit_m / sun_distance;
    env.solar_flux_w_m2 = constants::solar_constant_w_m2 * au_ratio * au_ratio;
    const double earth_shadow = occulted_fraction(sun_from_vehicle_m, constants::sun_radius_m, -position_gcrf_m,
                                                  constants::earth_mean_radius_m);
    const double moon_shadow = moon_from_vehicle_m == nullptr
                                   ? 0.0
                                   : occulted_fraction(sun_from_vehicle_m, constants::sun_radius_m,
                                                       *moon_from_vehicle_m, constants::moon_mean_radius_m);
    env.sunlit_fraction = (1.0 - earth_shadow) * (1.0 - moon_shadow);
    const double r = norm(position_gcrf_m);
    env.nadir_direction = -position_gcrf_m / r;
    env.distance_over_radius = r / constants::earth_mean_radius_m;
    env.albedo_phase = std::max(0.0, -dot(env.nadir_direction, env.sun_direction));
    if (moon_from_vehicle_m != nullptr) {
        const double d = norm(*moon_from_vehicle_m);
        env.moon_direction = *moon_from_vehicle_m / d;
        env.moon_distance_over_radius = d / constants::moon_mean_radius_m;
        // the sub-vehicle point's solar zenith: its outward normal is -moon_direction
        const double cos_z = std::max(0.0, -dot(env.moon_direction, env.sun_direction));
        env.moon_albedo_phase = cos_z;
        const double night = constants::stefan_boltzmann_w_m2k4 * std::pow(constants::moon_night_temperature_k, 4);
        env.moon_infrared_w_m2 = std::max((1.0 - constants::moon_bond_albedo) * env.solar_flux_w_m2 * cos_z, night);
    }
    return env;
}

double absorbed_flux_w_m2(const Vec3& normal, const RadiationEnvironment& env, double absorptivity,
                          double emissivity) {
    const double direct = env.solar_flux_w_m2 * env.sunlit_fraction * std::max(0.0, dot(normal, env.sun_direction));
    const double view = plate_view_factor_to_sphere(std::acos(std::clamp(dot(normal, env.nadir_direction), -1.0, 1.0)),
                                                    env.distance_over_radius);
    const double albedo = constants::earth_bond_albedo * env.solar_flux_w_m2 * env.albedo_phase * view;
    double moon_albedo = 0.0, moon_infrared = 0.0;
    if (env.moon_distance_over_radius > 0.0) {
        const double view_moon = plate_view_factor_to_sphere(
            std::acos(std::clamp(dot(normal, env.moon_direction), -1.0, 1.0)), env.moon_distance_over_radius);
        moon_albedo = constants::moon_bond_albedo * env.solar_flux_w_m2 * env.moon_albedo_phase * view_moon;
        moon_infrared = env.moon_infrared_w_m2 * view_moon;
    }
    return absorptivity * (direct + albedo + moon_albedo) +
           emissivity * (constants::earth_outgoing_longwave_w_m2 * view + moon_infrared);
}

}  // namespace starsim::environment
