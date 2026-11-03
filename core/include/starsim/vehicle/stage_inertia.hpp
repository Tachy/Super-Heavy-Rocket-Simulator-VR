#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::vehicle {

// Principal moments (roll, pitch, yaw) of a thin-walled cylinder about its centre [E]: the vehicle's model of a stage's
// structure when its config gives no inertia (I_xx = m r^2, I_yy = I_zz = m (r^2 / 2 + l^2 / 12)).
math::Vec3 cylinder_inertia_kg_m2(double mass_kg, double diameter_m, double length_m);

// The dry stage about its own CG: [stage] inertia_kg_m2 if set, else the cylinder of its dry mass and hull.
math::Vec3 dry_stage_inertia_kg_m2(const config::StageConfig& stage);

}  // namespace starsim::vehicle
