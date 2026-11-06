#include "starsim/mission/lunar_budget.hpp"

#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::mission {

BudgetResult lunar_budget(double initial_mass_kg, double dry_mass_kg, double reserve_kg,
                          const std::vector<BudgetLeg>& legs) {
    BudgetResult result;
    const double floor_kg = dry_mass_kg + reserve_kg;
    double mass_kg = initial_mass_kg;
    for (const auto& leg : legs) {
        mass_kg *= std::exp(-leg.delta_v_mps / (leg.isp_s * constants::standard_gravity_mps2));
        if (mass_kg < floor_kg && result.reason.ok()) {
            result.reason = report::make(report::Code::lunar_budget_short,
                                         {report::ref("leg", "phase.lunar_leg." + leg.id), report::mass("missing", floor_kg - mass_kg)});
        }
    }
    result.final_mass_kg = mass_kg;
    result.margin_kg = mass_kg - floor_kg;
    result.closes = result.margin_kg >= 0.0;
    return result;
}

double lunar_budget_required_mass(double dry_mass_kg, double reserve_kg, const std::vector<BudgetLeg>& legs) {
    double m = dry_mass_kg + reserve_kg;
    for (auto it = legs.rbegin(); it != legs.rend(); ++it) {
        m *= std::exp(it->delta_v_mps / (it->isp_s * constants::standard_gravity_mps2));
    }
    return m;
}

}  // namespace starsim::mission
