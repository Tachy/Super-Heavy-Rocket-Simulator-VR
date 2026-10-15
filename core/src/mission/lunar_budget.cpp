#include "starsim/mission/lunar_budget.hpp"

#include <cmath>
#include <cstdio>

#include "starsim/math/constants.hpp"

namespace starsim::mission {

BudgetResult lunar_budget(double initial_mass_kg, double dry_mass_kg, double reserve_kg,
                          const std::vector<BudgetLeg>& legs) {
    BudgetResult result;
    const double floor_kg = dry_mass_kg + reserve_kg;
    double mass_kg = initial_mass_kg;
    for (const auto& leg : legs) {
        mass_kg *= std::exp(-leg.delta_v_mps / (leg.isp_s * constants::standard_gravity_mps2));
        if (mass_kg < floor_kg && result.reason.empty()) {
            char text[160];
            std::snprintf(text, sizeof(text), "Treibstoff reicht nicht: nach %s fehlen %.1f t", leg.name.c_str(),
                          (floor_kg - mass_kg) / 1000.0);
            result.reason = text;
        }
    }
    result.final_mass_kg = mass_kg;
    result.margin_kg = mass_kg - floor_kg;
    result.closes = result.margin_kg >= 0.0;
    return result;
}

}  // namespace starsim::mission
