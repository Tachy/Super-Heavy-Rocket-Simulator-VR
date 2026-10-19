#pragma once

#include "starsim/aero/slender_body_aero.hpp"

namespace starsim::aero {

// A body whose drag is a fixed C_D A along the oncoming flow, independent of its attitude: a space station flown in a
// fixed attitude, its drag area and coefficient from its operator (ISS: the OEM header). No lift, no moment.
// Free-molecular flow is implied by the coefficient (orbit operators fit C_D ~ 2 for the free-molecular regime).
struct FixedAreaAero {
    double drag_area_m2 = 0.0;
    double drag_coefficient = 0.0;

    AeroResult compute(const math::Vec3& air_velocity_body_mps, double density_kg_m3, double speed_of_sound_mps) const;
};

}  // namespace starsim::aero
