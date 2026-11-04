#include "starsim/gnc/entry_residue_limit.hpp"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace starsim::gnc {

namespace {

double main_kg(const vehicle::Vehicle& ship) {
    const auto& stage = ship.stages().front();
    return stage.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) +
           stage.propellant_kg(config::TankContents::Fuel, config::TankRole::Main);
}

// The main tanks scaled to `fraction` of their contents in `base`.
void scale_main(vehicle::Vehicle& ship, const vehicle::Vehicle& base, double fraction) {
    const auto& tanks = base.stages().front().tanks;
    for (const auto& t : tanks) {
        if (t.config.role == config::TankRole::Main) ship.set_tank_mass(0, t.config.id, t.mass_kg * fraction);
    }
}

bool trims(const vehicle::Vehicle& ship, const AlphaSchedule& schedule) {
    return EntryAeroTable(ship, schedule).schedule_trim_margin_rad() > 0.0;
}

}  // namespace

void draw_main_tanks(vehicle::Vehicle& ship, double kg) {
    const double total = main_kg(ship);
    if (!(total > 0.0) || !(kg > 0.0)) return;
    const double keep = std::max(0.0, 1.0 - kg / total);
    std::vector<std::pair<std::string, double>> masses;
    for (const auto& t : ship.stages().front().tanks) {
        if (t.config.role == config::TankRole::Main) masses.emplace_back(t.config.id, t.mass_kg * keep);
    }
    for (const auto& [id, mass] : masses) ship.set_tank_mass(0, id, mass);
}

EntryResidueLimit entry_residue_limit(const vehicle::Vehicle& ship, const AlphaSchedule& schedule, double tolerance_kg) {
    EntryResidueLimit limit;
    limit.main_propellant_kg = main_kg(ship);
    limit.max_main_propellant_kg = limit.main_propellant_kg;
    if (!(limit.main_propellant_kg > 0.0) || trims(ship, schedule)) return limit;
    auto copy = ship.copy_upper(0);
    scale_main(*copy, ship, 0.0);
    if (!trims(*copy, schedule)) {
        limit.max_main_propellant_kg = 0.0;
        return limit;
    }
    double lo = 0.0, hi = 1.0;  // fractions of the main tanks' contents: lo trims, hi does not
    while ((hi - lo) * limit.main_propellant_kg > tolerance_kg) {
        const double mid = 0.5 * (lo + hi);
        scale_main(*copy, ship, mid);
        (trims(*copy, schedule) ? lo : hi) = mid;
    }
    limit.max_main_propellant_kg = lo * limit.main_propellant_kg;
    return limit;
}

}  // namespace starsim::gnc
