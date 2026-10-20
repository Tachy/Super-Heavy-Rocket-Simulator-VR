#include "starsim/vehicle/boil_off.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::vehicle {

using math::Vec3;

double strip_heat_leak_w_m2(double absorbed_w_m2, const config::ThermalSurfaceConfig& surface,
                            double boiling_point_k) {
    const double es = surface.emissivity * constants::stefan_boltzmann_w_m2k4;
    const double r = surface.wall_resistance_m2k_w;
    const double tb = boiling_point_k;
    const double a = std::max(0.0, absorbed_w_m2);
    // f(T) = es T^4 + (T - tb) / r - a is convex and increasing: Newton from a point with f >= 0 converges
    // monotonically from the right. Both candidates have f >= 0; take the nearer one.
    double t = std::min(tb + a * r, std::pow(a / es, 0.25) + tb);
    for (int k = 0; k < 100; ++k) {
        const double f = es * t * t * t * t + (t - tb) / r - a;
        const double step = f / (4.0 * es * t * t * t + 1.0 / r);
        t -= step;
        if (std::abs(step) <= 1e-12 * t) break;
    }
    return (t - tb) / r;
}

double section_heat_w(const HullSection& section, const config::ThermalConfig& thermal, double belly_azimuth_rad,
                      double boiling_point_k, const environment::RadiationEnvironment& env,
                      const math::Quat& body_to_frame) {
    const double strip_area_m2 = constants::two_pi * section.radius_m * section.length_m / boil_off_strips;
    double heat_w = 0.0;
    for (int k = 0; k < boil_off_strips; ++k) {
        const double azimuth = constants::two_pi * (k + 0.5) / boil_off_strips;
        const Vec3 normal = body_to_frame.rotate(Vec3{0.0, std::cos(azimuth), std::sin(azimuth)});
        const bool on_shield = thermal.shield.has_value() &&
                               std::abs(std::remainder(azimuth - belly_azimuth_rad, constants::two_pi)) <=
                                   thermal.shield_half_width_rad;
        const auto& surface = on_shield ? *thermal.shield : thermal.hull;
        const double absorbed = environment::absorbed_flux_w_m2(normal, env, surface.absorptivity, surface.emissivity);
        heat_w += strip_area_m2 * strip_heat_leak_w_m2(absorbed, surface, boiling_point_k);
    }
    return heat_w;
}

std::vector<TankBoilOff> boil_off_rates(const Vehicle& vehicle, const environment::RadiationEnvironment& env,
                                        const math::Quat& body_to_frame) {
    std::vector<TankBoilOff> out;
    const auto& stages = vehicle.stages();
    for (size_t s = 0; s < stages.size(); ++s) {
        const auto& stage = stages[s];
        if (!stage.config.thermal) continue;
        for (size_t k = 0; k < stage.tanks.size(); ++k) {
            const auto& tank = stage.tanks[k];
            const bool hull_tank = tank.config.role == config::TankRole::Main && tank.config.diameter_m == 0.0 &&
                                   tank.config.contents != config::TankContents::Rcs;
            if (!hull_tank || tank.latent_heat_j_kg <= 0.0 || tank.mass_kg <= 0.0 || tank.length_m <= 0.0) continue;
            const double heat_w = std::max(0.0, section_heat_w({tank.radius_m, tank.length_m}, *stage.config.thermal,
                                                               stage.config.aero.belly_azimuth_rad,
                                                               tank.boiling_point_k, env, body_to_frame));
            out.push_back({s, k, heat_w, heat_w / tank.latent_heat_j_kg});
        }
    }
    return out;
}

BoilOffRate total_boil_off(const Vehicle& vehicle, const std::vector<TankBoilOff>& tanks) {
    BoilOffRate rate;
    for (const auto& t : tanks) {
        const auto contents = vehicle.stages()[t.stage_index].tanks[t.tank_index].config.contents;
        (contents == config::TankContents::Oxidizer ? rate.oxidizer_kgps : rate.fuel_kgps) += t.mass_flow_kgps;
    }
    return rate;
}

}  // namespace starsim::vehicle
