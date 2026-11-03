#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::vehicle {

// Payload and fairing sit in the upper part of their stage [E], unless the stage's config gives their stations.
inline constexpr double extra_mass_station_fraction = 0.75;

// Principal moments (roll, pitch, yaw) of a thin-walled cylinder about its centre [E]: the vehicle's model of a stage's
// structure when its config gives no inertia (I_xx = m r^2, I_yy = I_zz = m (r^2 / 2 + l^2 / 12)).
math::Vec3 cylinder_inertia_kg_m2(double mass_kg, double diameter_m, double length_m);

// The dry stage about its own CG: [stage] inertia_kg_m2 if set, else the cylinder of its dry mass and hull.
math::Vec3 dry_stage_inertia_kg_m2(const config::StageConfig& stage);

// Density of a tank's contents: the vehicle's oxidizer / fuel density for main propellant, the tank's own otherwise.
double tank_density_kg_m3(const config::TankConfig& tank, const config::PropellantConfig& propellant);

// Liquid settled aft: a solid column of `mass_kg` standing on the tank bottom (base_station_m). station_m: its centre;
// pitch_inertia_kg_m2: about that centre, m (r^2 / 4 + h^2 / 12) (the fluid does not spin with the tank: no roll term).
struct SettledColumn {
    double station_m;
    double pitch_inertia_kg_m2;
};
SettledColumn settled_column(double mass_kg, double density_kg_m3, double radius_m, double base_station_m);

// Mass, CG station (from the stage bottom) and pitch inertia about that CG of one stage flying alone with its
// propellant settled aft: the same parts as Vehicle::mass_properties (dry stage about its own CG, payload / fairing,
// every tank's liquid as a column on its bottom), each moved to the common CG (parallel axis).
// main_propellant_kg: the load of the stage's main oxidizer and fuel tanks, split by their capacity (the configured
// mixture); the other tanks (headers, RCS) carry their configured mass_kg.
struct StageMassProperties {
    double mass_kg = 0.0;
    double cg_station_m = 0.0;
    double pitch_inertia_kg_m2 = 0.0;
};
StageMassProperties settled_stage_mass_properties(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                                  double main_propellant_kg);

}  // namespace starsim::vehicle
