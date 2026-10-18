#include "starsim/config/propellant_loading.hpp"

#include <algorithm>

namespace starsim::config {
namespace {

bool feeds(const TankConfig& tank) { return tank.contents != TankContents::Rcs; }

double capacity(const StageConfig& stage, TankRole role, bool (*selected)(const TankConfig&)) {
    double kg = 0.0;
    for (const auto& t : stage.tanks) kg += selected(t) && t.role == role ? t.capacity_kg : 0.0;
    return kg;
}

// Fills the selected tanks with `kg`: headers first, then the main tanks, each group in its capacity ratio.
void fill(StageConfig& stage, double kg, bool (*selected)(const TankConfig&)) {
    const double header = capacity(stage, TankRole::Header, selected);
    const double main = capacity(stage, TankRole::Main, selected);
    const double in_header = std::clamp(kg, 0.0, header);
    const double in_main = std::max(0.0, kg - in_header);
    for (auto& t : stage.tanks) {
        if (!selected(t)) continue;
        if (t.role == TankRole::Header) {
            t.mass_kg = t.capacity_kg * in_header / header;
        } else {
            t.mass_kg = main > 0.0 ? t.capacity_kg * in_main / main : 0.0;
        }
    }
}

}  // namespace

void fill_tanks(StageConfig& stage, double total_kg) { fill(stage, total_kg, feeds); }

void fill_tanks(StageConfig& stage, double oxidizer_kg, double fuel_kg) {
    fill(stage, oxidizer_kg, [](const TankConfig& t) { return t.contents == TankContents::Oxidizer; });
    fill(stage, fuel_kg, [](const TankConfig& t) { return t.contents == TankContents::Fuel; });
}

}  // namespace starsim::config
