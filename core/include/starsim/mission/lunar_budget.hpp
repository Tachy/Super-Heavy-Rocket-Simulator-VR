#pragma once

#include <string>
#include <vector>

namespace starsim::mission {

// One propulsive leg of a lunar mission budget.
struct BudgetLeg {
    std::string name;  // German (refusal text), e.g. "Abstieg"
    double delta_v_mps = 0.0;
    double isp_s = 0.0;  // vacuum
};

struct BudgetResult {
    bool closes = false;
    double final_mass_kg = 0.0;
    double margin_kg = 0.0;  // final - (dry + reserve); < 0 when it does not close
    std::string reason;      // German when it does not close: the first leg after which the mass is short
};

// Rocket equation over the legs in order; closes when the final mass >= dry + reserve.
BudgetResult lunar_budget(double initial_mass_kg, double dry_mass_kg, double reserve_kg,
                          const std::vector<BudgetLeg>& legs);

}  // namespace starsim::mission
