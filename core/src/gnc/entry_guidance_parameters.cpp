// The ship's entry guidance parameters from its config: [stage.entry] limits and caps, the hypersonic bank rate and the
// terminal nose turn's angular acceleration derived from the RCS at the entry mass (plan 2026-11-03
// planner-vehicle-parameters Task 4).

#include <algorithm>
#include <cmath>

#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/gnc/vehicle_capability.hpp"

namespace starsim::gnc {

EntryGuidanceParameters entry_guidance_parameters(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                                  EntryKind kind) {
    return entry_guidance_parameters(vehicle, stage, kind,
                                     kind == EntryKind::LunarReturn ? EntryGuidanceParameters::lunar_return()
                                                                    : EntryGuidanceParameters{});
}

EntryGuidanceParameters entry_guidance_parameters(const config::VehicleConfig& vehicle, const config::StageConfig& stage,
                                                  EntryKind kind, EntryGuidanceParameters p) {
    if (!stage.entry) return p;
    const auto& e = *stage.entry;
    p.max_dynamic_pressure_pa = kind == EntryKind::LunarReturn ? e.lunar_max_dynamic_pressure_pa : e.max_dynamic_pressure_pa;
    p.terminal_mach = e.terminal_mach;
    p.terminal_mach_latest = e.terminal_mach_latest;
    p.terminal_min_alpha_rad = e.terminal_min_alpha_rad;
    p.terminal_alpha_authority_rad = e.terminal_alpha_authority_rad;
    p.fast_bank_rate_radps = e.fast_bank_rate_radps;
    p.terminal_slew_rate_radps = e.alpha_slew_rate_radps;
    // The RCS's angular acceleration at the entry mass, the weaker transverse axis (no RCS: the caps stand).
    const auto masses = ship_landing_masses(vehicle, stage);
    const double a = rcs_pitch_acceleration_radps2(stage, masses.heavy_pitch_inertia_kg_m2, masses.heavy_cg_station_m);
    p.bank_rate_radps = e.bank_rate_max_radps;
    p.terminal_nose_slew_acceleration_radps2 = e.nose_slew_acceleration_max_radps2;
    if (a > 0.0) {
        // the hypersonic reversal's own angle: the RCS stops the roll within what counts as a settled bank
        p.bank_rate_radps = std::min(e.bank_rate_max_radps, std::sqrt(2.0 * a * entry_settled_bank_rad));
        p.terminal_nose_slew_acceleration_radps2 = std::min(e.nose_slew_acceleration_max_radps2, a);
    }
    return p;
}

}  // namespace starsim::gnc
