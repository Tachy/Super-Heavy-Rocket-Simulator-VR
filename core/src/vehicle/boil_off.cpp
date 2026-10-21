#include "starsim/vehicle/boil_off.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/frames/lvlh.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

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
                      const math::Quat& body_to_frame, const std::vector<double>* wetted) {
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
        const double wet = wetted ? (*wetted)[static_cast<size_t>(k)] : 1.0;
        double leak_w_m2 = wet * strip_heat_leak_w_m2(absorbed, surface, boiling_point_k);
        if (wet < 1.0) {
            auto dry = surface;
            dry.wall_resistance_m2k_w += thermal.ullage_resistance_m2k_w;
            leak_w_m2 += (1.0 - wet) * strip_heat_leak_w_m2(absorbed, dry, boiling_point_k);
        }
        heat_w += strip_area_m2 * leak_w_m2;
    }
    return heat_w;
}

Vec3 apparent_gravity_body_mps2(const LiquidEnvironment& liquid, const Vec3& offset_body_m) {
    const Vec3 r = liquid.position_gcrf_m - liquid.centre_gcrf_m;
    const double distance = norm(r);
    const Vec3 r_hat = r / distance;
    const Vec3 offset = liquid.cg_offset_body_m + offset_body_m;  // from the reference point
    const Vec3 d = liquid.body_to_gcrf.rotate(offset);
    const Vec3 tide_gcrf = (r_hat * (3.0 * dot(r_hat, d)) - d) * (liquid.gm_m3ps2 / (distance * distance * distance));
    const Vec3& w = liquid.angular_velocity_body_radps;
    return liquid.body_to_gcrf.inverse_rotate(tide_gcrf) - liquid.specific_force_body_mps2 -
           cross(w, cross(w, offset));
}

double settled_weight(double bond_number) {
    constexpr double wetted_below = 1.0, settled_above = 10.0;  // [E]
    if (!(bond_number > wetted_below)) return 0.0;
    if (bond_number >= settled_above) return 1.0;
    return std::log(bond_number / wetted_below) / std::log(settled_above / wetted_below);
}

std::vector<double> settled_wetted_fractions(double fill, const Vec3& down_body, double aspect) {
    std::vector<double> wetted(boil_off_strips, fill >= 1.0 ? 1.0 : 0.0);
    if (fill <= 0.0 || fill >= 1.0) return wetted;
    const Vec3 down = math::normalized(down_body);
    // The free surface: the plane across `down` below which `fill` of the volume lies (midpoint samples, area weight r).
    constexpr int axial = 24, radial = 8, around = 36;
    std::vector<std::pair<double, double>> samples;  // projection on down, weight
    samples.reserve(axial * radial * around);
    double total = 0.0;
    for (int i = 0; i < axial; ++i) {
        const double x = aspect * ((i + 0.5) / axial - 0.5);
        for (int j = 0; j < radial; ++j) {
            const double r = (j + 0.5) / radial;
            for (int k = 0; k < around; ++k) {
                const double a = constants::two_pi * (k + 0.5) / around;
                samples.emplace_back(dot(down, Vec3{x, r * std::cos(a), r * std::sin(a)}), r);
                total += r;
            }
        }
    }
    std::sort(samples.begin(), samples.end(), [](const auto& p, const auto& q) { return p.first > q.first; });
    double level = samples.back().first, sum = 0.0;
    for (const auto& [projection, weight] : samples) {
        sum += weight;
        level = projection;
        if (sum >= fill * total) break;
    }
    // Each strip's wall line: the part at or beyond the surface is wet.
    constexpr int wall = 48;
    for (int k = 0; k < boil_off_strips; ++k) {
        const double a = constants::two_pi * (k + 0.5) / boil_off_strips;
        int wet = 0;
        for (int i = 0; i < wall; ++i) {
            const double x = aspect * ((i + 0.5) / wall - 0.5);
            if (dot(down, Vec3{x, std::cos(a), std::sin(a)}) >= level) ++wet;
        }
        wetted[static_cast<size_t>(k)] = static_cast<double>(wet) / wall;
    }
    return wetted;
}

std::vector<TankBoilOff> boil_off_rates(const Vehicle& vehicle, const environment::RadiationEnvironment& env,
                                        const math::Quat& body_to_frame, const LiquidEnvironment* liquid) {
    std::vector<TankBoilOff> out;
    const auto& stages = vehicle.stages();
    const double cg_station_m = liquid ? vehicle.mass_properties().cg_station_m : 0.0;
    for (size_t s = 0; s < stages.size(); ++s) {
        const auto& stage = stages[s];
        if (!stage.config.thermal) continue;
        for (size_t k = 0; k < stage.tanks.size(); ++k) {
            const auto& tank = stage.tanks[k];
            const bool hull_tank = tank.config.role == config::TankRole::Main && tank.config.diameter_m == 0.0 &&
                                   tank.config.contents != config::TankContents::Rcs;
            if (!hull_tank || tank.latent_heat_j_kg <= 0.0 || tank.mass_kg <= 0.0 || tank.length_m <= 0.0) continue;
            std::vector<double> wetted;
            if (liquid != nullptr) {
                const double centre_station_m =
                    stage.base_station_m + tank.config.base_station_m + 0.5 * tank.length_m;
                const Vec3 g = apparent_gravity_body_mps2(*liquid, {centre_station_m - cg_station_m, 0.0, 0.0});
                const double a = norm(g);
                const double bond = tank.surface_tension_n_m > 0.0
                                        ? tank.density_kg_m3 * a * tank.radius_m * tank.radius_m / tank.surface_tension_n_m
                                        : 0.0;
                const double settled = settled_weight(bond);
                const double fill = std::clamp(tank.mass_kg / tank.config.capacity_kg, 0.0, 1.0);
                wetted.assign(boil_off_strips, 1.0);
                if (settled > 0.0) {
                    const auto lying = settled_wetted_fractions(fill, g / a, tank.length_m / tank.radius_m);
                    for (size_t i = 0; i < wetted.size(); ++i) wetted[i] = (1.0 - settled) + settled * lying[i];
                }
            }
            const double heat_w = std::max(
                0.0, section_heat_w({tank.radius_m, tank.length_m}, *stage.config.thermal,
                                    stage.config.aero.belly_azimuth_rad, tank.boiling_point_k, env, body_to_frame,
                                    liquid != nullptr ? &wetted : nullptr));
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

BoilOffRate orbit_average_boil_off(const Vehicle& vehicle, const math::StateVector& orbit_gcrf,
                                   const math::Quat& body_in_lvlh, const Vec3& sun_position_gcrf_m, int samples) {
    const double gm = constants::earth_gm_m3ps2;
    const double period_s = orbit::orbital_period_s(orbit::elements_from_state(orbit_gcrf, gm).semi_major_axis_m, gm);
    BoilOffRate mean;
    for (int k = 0; k < samples; ++k) {
        const auto state = orbit::propagate_two_body(orbit_gcrf, gm, period_s * k / samples);
        const auto env = environment::earth_orbit_radiation(state.position_m, sun_position_gcrf_m - state.position_m,
                                                            nullptr);
        const math::Quat attitude = frames::lvlh_to_inertial(state) * body_in_lvlh;
        // The liquid under the tide and the LVLH rotation (no drag: two-body orbit).
        const LiquidEnvironment liquid{{}, attitude.inverse_rotate(frames::lvlh_rate_inertial_radps(state)),
                                       state.position_m, attitude};
        const auto rate = total_boil_off(vehicle, boil_off_rates(vehicle, env, attitude, &liquid));
        mean.oxidizer_kgps += rate.oxidizer_kgps / samples;
        mean.fuel_kgps += rate.fuel_kgps / samples;
    }
    return mean;
}

}  // namespace starsim::vehicle
