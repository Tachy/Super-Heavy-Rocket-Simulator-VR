#include "starsim/vehicle/stage_inertia.hpp"

namespace starsim::vehicle {

math::Vec3 cylinder_inertia_kg_m2(double mass_kg, double diameter_m, double length_m) {
    const double r = 0.5 * diameter_m;
    const double transverse = mass_kg * (0.5 * r * r + length_m * length_m / 12.0);
    return {mass_kg * r * r, transverse, transverse};
}

math::Vec3 dry_stage_inertia_kg_m2(const config::StageConfig& stage) {
    if (stage.inertia_kg_m2) return *stage.inertia_kg_m2;
    return cylinder_inertia_kg_m2(stage.dry_mass_kg, stage.diameter_m, stage.length_m);
}

}  // namespace starsim::vehicle
