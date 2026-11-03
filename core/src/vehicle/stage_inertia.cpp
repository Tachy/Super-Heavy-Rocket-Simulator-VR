#include "starsim/vehicle/stage_inertia.hpp"

#include <stdexcept>
#include <vector>

#include "starsim/math/constants.hpp"

namespace starsim::vehicle {

namespace {

bool main_propellant_tank(const config::TankConfig& t) {
    return t.role == config::TankRole::Main &&
           (t.contents == config::TankContents::Oxidizer || t.contents == config::TankContents::Fuel);
}

}  // namespace

math::Vec3 cylinder_inertia_kg_m2(double mass_kg, double diameter_m, double length_m) {
    const double r = 0.5 * diameter_m;
    const double transverse = mass_kg * (0.5 * r * r + length_m * length_m / 12.0);
    return {mass_kg * r * r, transverse, transverse};
}

math::Vec3 dry_stage_inertia_kg_m2(const config::StageConfig& stage) {
    if (stage.inertia_kg_m2) return *stage.inertia_kg_m2;
    return cylinder_inertia_kg_m2(stage.dry_mass_kg, stage.diameter_m, stage.length_m);
}

double tank_density_kg_m3(const config::TankConfig& tank, const config::PropellantConfig& propellant) {
    return tank.contents == config::TankContents::Oxidizer ? propellant.oxidizer_density_kg_m3
           : tank.contents == config::TankContents::Fuel   ? propellant.fuel_density_kg_m3
                                                           : tank.density_kg_m3;
}

SettledColumn settled_column(double mass_kg, double density_kg_m3, double radius_m, double base_station_m) {
    const double h = mass_kg / (density_kg_m3 * constants::pi * radius_m * radius_m);
    return {base_station_m + 0.5 * h, mass_kg * (0.25 * radius_m * radius_m + h * h / 12.0)};
}

StageMassProperties settled_stage_mass_properties(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                                  double main_propellant_kg) {
    struct Part {
        double mass, station, iyy;
    };
    std::vector<Part> parts;
    parts.push_back({stage.dry_mass_kg, stage.dry_cg_station_m, dry_stage_inertia_kg_m2(stage).y});
    if (stage.payload_mass_kg > 0.0) {
        parts.push_back({stage.payload_mass_kg,
                         stage.payload_station_m.value_or(extra_mass_station_fraction * stage.length_m), 0.0});
    }
    if (stage.fairing_mass_kg > 0.0) {
        parts.push_back({stage.fairing_mass_kg,
                         stage.fairing_station_m.value_or(extra_mass_station_fraction * stage.length_m), 0.0});
    }
    double main_capacity_kg = 0.0;
    for (const auto& t : stage.tanks) {
        if (main_propellant_tank(t)) main_capacity_kg += t.capacity_kg;
    }
    if (main_propellant_kg > 0.0 && main_capacity_kg <= 0.0) {
        throw std::invalid_argument("settled_stage_mass_properties: stage '" + stage.id + "' has no main propellant tank");
    }
    for (const auto& t : stage.tanks) {
        const double mass = main_propellant_tank(t) ? main_propellant_kg * t.capacity_kg / main_capacity_kg : t.mass_kg;
        if (mass <= 0.0) continue;
        const double radius = 0.5 * (t.diameter_m > 0.0 ? t.diameter_m : stage.diameter_m);
        const auto column = settled_column(mass, tank_density_kg_m3(t, vehicle.propellant), radius, t.base_station_m);
        parts.push_back({mass, column.station_m, column.pitch_inertia_kg_m2});
    }
    StageMassProperties mp;
    for (const auto& p : parts) {
        mp.mass_kg += p.mass;
        mp.cg_station_m += p.mass * p.station;
    }
    mp.cg_station_m /= mp.mass_kg;
    for (const auto& p : parts) {
        const double d = p.station - mp.cg_station_m;
        mp.pitch_inertia_kg_m2 += p.iyy + p.mass * d * d;
    }
    return mp;
}

}  // namespace starsim::vehicle
