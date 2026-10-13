#pragma once

#include <string>
#include <vector>

#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// Flip and landing of the ship at the tower (b3 of docs/superpowers/specs/2026-10-13-ship-return-design.md).
struct ShipLandingParameters {
    std::vector<std::string> flip_groups{"ship/sea_level_1", "ship/sea_level_2", "ship/sea_level_3"};
    double flip_throttle = 0.5;                                  // [E]
    double flip_tolerance_rad = 15.0 * constants::deg_to_rad;    // [E] axis from the vertical: the burn plans start
    double first_plan_retry_s = 1.0;                             // a failed first plan is tried again this often
    ScvxLandingParameters scvx;                                  // ship preset: ship_landing_parameters()
};

// The ship's preset: engine phases 3 -> 2 -> 1 (user 2026-10-13) and planning limits for a ship whose last engine
// barely holds it (one Raptor 3 2.75 MN at sea level vs ~265 t: thrust/weight ~1.04).
ShipLandingParameters ship_landing_parameters();

}  // namespace starsim::gnc
