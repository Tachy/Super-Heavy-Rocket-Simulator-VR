#pragma once

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::vehicle {

// Mass properties of two rigidly joined bodies (a docked pair, ISS i5), all in one frame: summed mass, the compound CG
// and the inertia tensor about it (each body's tensor about its own CG plus the parallel-axis terms).
struct CompoundMass {
    double mass_kg = 0.0;
    math::Vec3 cg_m;
    math::Mat3 inertia_kg_m2;
};

CompoundMass combine_mass(double mass1_kg, const math::Vec3& cg1_m, const math::Mat3& inertia1_kg_m2, double mass2_kg,
                          const math::Vec3& cg2_m, const math::Mat3& inertia2_kg_m2);

}  // namespace starsim::vehicle
