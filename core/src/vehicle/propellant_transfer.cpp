#include "starsim/vehicle/propellant_transfer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "starsim/vehicle/vehicle.hpp"

namespace starsim::vehicle {

using math::Vec3;

double line_mass_flow_kgps(double discharge_area_m2, double density_kg_m3, double pressure_difference_pa) {
    if (!(pressure_difference_pa > 0.0)) return 0.0;
    return discharge_area_m2 * std::sqrt(2.0 * density_kg_m3 * pressure_difference_pa);
}

double pair_discharge_area_m2(const config::TransferConnectorConfig& a, const config::TransferConnectorConfig& b) {
    const auto cda = [](const config::TransferConnectorConfig& c) {
        return c.discharge_coefficient * std::numbers::pi / 4.0 * c.line_diameter_m * c.line_diameter_m;
    };
    return std::min(cda(a), cda(b));
}

OutletCheck outlet_check(const Vehicle& vehicle, size_t stage_index, size_t tank_index, const LiquidEnvironment& liquid,
                         double max_tilt_rad) {
    const auto& stage = vehicle.stages().at(stage_index);
    const auto& tank = stage.tanks.at(tank_index);
    OutletCheck out;
    const double centre_station_m = stage.base_station_m + tank.config.base_station_m + 0.5 * tank.length_m;
    const Vec3 offset = Vec3{centre_station_m, 0.0, 0.0} - vehicle.mass_properties().cg_point_m();
    const Vec3 g = apparent_gravity_body_mps2(liquid, offset);
    const double a = norm(g);
    if (!(a > 0.0) || !(tank.mass_kg > 0.0)) return out;
    out.tilt_rad = std::acos(std::clamp(-g.x / a, -1.0, 1.0));
    out.bond_number = tank.surface_tension_n_m > 0.0
                          ? tank.density_kg_m3 * a * tank.radius_m * tank.radius_m / tank.surface_tension_n_m
                          : std::numeric_limits<double>::infinity();
    out.covered = settled_weight(out.bond_number) >= 1.0 && out.tilt_rad <= max_tilt_rad;
    return out;
}

SettlingPush settling_push(const Vehicle& vehicle, size_t stage_index) {
    SettlingPush push;
    if (!vehicle.needs_settling(stage_index)) return push;
    const auto capability = vehicle.rcs_pure_force_capability({1.0, 0.0, 0.0});
    if (!(capability.force_n > 0.0)) return push;
    push.acceleration_mps2 = capability.force_n / vehicle.mass_properties().mass_kg;
    push.lead_s = settling_lead_factor * settling_time_s(vehicle.settling_travel_m(stage_index), push.acceleration_mps2);
    push.gas_kg = capability.mass_flow_kgps * push.lead_s;
    return push;
}

double settling_gas_bound_kg(const Vehicle& vehicle, size_t stage_index) {
    double longest_m = 0.0, held_kg = 0.0, capacity_kg = 0.0;
    for (const auto& t : vehicle.stages().at(stage_index).tanks) {
        if (!settling_tank_kind(t)) continue;
        longest_m = std::max(longest_m, t.length_m);
        held_kg += t.mass_kg;
        capacity_kg += t.config.capacity_kg;
    }
    if (!(capacity_kg > 0.0)) return 0.0;
    const auto capability = vehicle.rcs_pure_force_capability({1.0, 0.0, 0.0});
    if (!(capability.force_n > 0.0)) return 0.0;
    const double without_kg = vehicle.mass_properties().mass_kg - held_kg;
    constexpr int samples = 1000;  // the fill in 0.1 % steps (the maximum is smooth)
    double largest = 0.0;
    for (int i = 0; i <= samples; ++i) {
        const double fill = static_cast<double>(i) / samples;
        const double travel_m = std::max(longest_m * (1.0 - fill), settling_min_travel_m);
        const double acceleration = capability.force_n / (without_kg + fill * capacity_kg);
        largest = std::max(largest, settling_time_s(travel_m, acceleration));
    }
    return capability.mass_flow_kgps * settling_lead_factor * largest;
}

double settling_time_s(double length_m, double acceleration_mps2) {
    if (!(acceleration_mps2 > 0.0)) return std::numeric_limits<double>::infinity();
    return std::sqrt(2.0 * length_m / acceleration_mps2);
}

}  // namespace starsim::vehicle
