#pragma once

#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::gnc {

// Main-tank residue a belly-first ship may carry into the entry (planner plan Task 12b). The residue lies in the aft
// main tanks and moves the CG aft; from some load on the flaps alone can no longer hold the alpha schedule in pitch
// (the 177 t ship: one pair on its stop from Mach ~2 down with ~45 t on board). The RCS is no trim source: the entry
// guidance's bank rate is derived from the RCS alone (entry_guidance_parameters), so the trim must come from the flaps.
// The limit is the largest main-tank propellant (both main tanks drawn in proportion to their contents, as a vent
// draws them; header tanks untouched) whose EntryAeroTable trim along the schedule keeps every flap inside its stops
// (EntryAeroTable::schedule_trim_margin_rad > 0). Bisection on the main tanks' fill to `tolerance_kg`.
// Returns the ship's main-tank propellant itself when that already trims (nothing to vent), 0 when even empty main
// tanks do not (the headers alone: everything in the main tanks goes). `ship`: a single ship (its first stage).
struct EntryResidueLimit {
    double main_propellant_kg = 0.0;      // on board now
    double max_main_propellant_kg = 0.0;  // the most the flaps can trim
    double excess_kg() const { return main_propellant_kg > max_main_propellant_kg ? main_propellant_kg - max_main_propellant_kg : 0.0; }
};

EntryResidueLimit entry_residue_limit(const vehicle::Vehicle& ship, const AlphaSchedule& schedule,
                                      double tolerance_kg = 100.0);

// The ship's main tanks drawn by `kg` in proportion to their contents (a vent; header tanks untouched).
void draw_main_tanks(vehicle::Vehicle& ship, double kg);

}  // namespace starsim::gnc
