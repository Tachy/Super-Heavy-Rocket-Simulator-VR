#pragma once

namespace starsim::mission {

// What a tanker keeps for its return (refuelling r4).
struct TankerReserveParameters {
    double deorbit_delta_v_mps = 0.0;   // the previewed deorbit plan
    double exhaust_velocity_mps = 0.0;  // of the deorbit engine set
    double mixture_ratio = 3.6;         // oxidizer / fuel of the deorbit set
    double burn_margin = 1.10;          // [E] plan_ship_return's commit margin (r2)
    double gas_kg = 1500.0;             // [E] cold gas after the transfer: departure, turn to the burn, entry
    double boil_off_kg = 2000.0;        // [E] the residue boiled off until the burn (r2: ~1.4 t of 10 t in 22 h)
    double unusable_kg = 1000.0;        // [E] trapped in sumps and lines
};

struct TankerReserve {
    double burn_kg = 0.0;
    double oxidizer_kg = 0.0;
    double fuel_kg = 0.0;
    double total_kg = 0.0;
};

// The main-tank residue a tanker keeps for its return (spec r4 "End"): the deorbit burn by the rocket equation from
// the mass after it (`dry_with_headers_kg`: the ship without main propellant, header tanks full - the landing reserve -
// plus the gas and the unusable residue), times the margin; plus the gas, the boil-off until the burn and the unusable
// residue. Split at the mixture ratio (the gas draws the main tanks in proportion, the burn at the engines' ratio;
// boil-off ratios differ - the margin covers it, r5 flies it).
TankerReserve tanker_reserve(double dry_with_headers_kg, const TankerReserveParameters& p);

}  // namespace starsim::mission
