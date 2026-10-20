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

double settling_time_s(double length_m, double acceleration_mps2) {
    if (!(acceleration_mps2 > 0.0)) return std::numeric_limits<double>::infinity();
    return std::sqrt(2.0 * length_m / acceleration_mps2);
}

}  // namespace starsim::vehicle
