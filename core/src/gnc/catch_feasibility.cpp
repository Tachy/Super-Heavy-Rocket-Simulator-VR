#include "starsim/gnc/catch_feasibility.hpp"

#include <algorithm>
#include <cstdio>

namespace starsim::gnc {

CatchFeasibility catch_feasibility(const vehicle::Vehicle& ship, const std::vector<std::string>& flip_groups,
                                   double min_catch_propellant_kg) {
    for (const auto& stage : ship.stages()) {
        for (const auto& e : stage.engines) {
            if (e.failed && std::find(flip_groups.begin(), flip_groups.end(), e.group_id) != flip_groups.end()) {
                return {false, "Triebwerk ausgefallen (" + e.group_id + "): kein Fang möglich"};
            }
        }
    }
    const double propellant_kg = ship.propellant_kg();
    if (propellant_kg < min_catch_propellant_kg) {
        char text[128];
        std::snprintf(text, sizeof text, "Treibstoff zu knapp (%.1f t, Fang braucht %.1f t)", propellant_kg / 1000.0,
                      min_catch_propellant_kg / 1000.0);
        return {false, text};
    }
    return {};
}

}  // namespace starsim::gnc
