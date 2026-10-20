#pragma once

namespace starsim::mission {

// What a tanker keeps for its return (refuelling r4).
struct TankerReserveParameters {
    double deorbit_delta_v_mps = 0.0;   // the previewed deorbit plan
    double exhaust_velocity_mps = 0.0;  // of the deorbit engine set
    double mixture_ratio = 3.6;         // oxidizer / fuel of the deorbit set
    double burn_margin = 1.10;          // [E] plan_ship_return's commit margin (r2)
    // [D] r5 tanker_flight_orbit (was 1.5 t [E], which left the caught tanker with 0 t): cold gas after the transfer -
    // departure ~0.3 t, turn to the burn, entry ~3 t
    double gas_kg = 3500.0;
    double boil_off_kg = 2000.0;        // [E] until the burn (r5 measured 1.6 t in 19 h; r2: ~1.4 t of 10 t in 22 h)
    double boil_off_fuel_share = 0.56;  // [D] r5: CH4 0.9 of 1.6 t boiled before the burn (CH4 ran short at O/F 3.6)
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
// residue. Split at the mixture ratio (the gas draws the main tanks in proportion, the burn at the engines' ratio), the
// boil-off at its own measured share (r5).
TankerReserve tanker_reserve(double dry_with_headers_kg, const TankerReserveParameters& p);

}  // namespace starsim::mission
