#pragma once

#include <string>
#include <vector>

#include "starsim/vehicle/vehicle.hpp"

namespace starsim::gnc {

struct CatchFeasibility {
    bool feasible = true;
    std::string reason;  // German, empty when feasible
};

// Crew escape: can the onboard computer still try the catch? Not when an engine of the flip groups has failed (the
// catch campaign's engine-failure case ends in the upright abort) or when the propellant (oxidizer + fuel, headers
// included) is below `min_catch_propellant_kg` (measured: the lowest nominal-gate load still caught + margin).
CatchFeasibility catch_feasibility(const vehicle::Vehicle& ship, const std::vector<std::string>& flip_groups,
                                   double min_catch_propellant_kg);

}  // namespace starsim::gnc
