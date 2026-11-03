#pragma once

namespace starsim::mission {

// What a tanker keeps for its return (refuelling r4). FlightSession::tanker_reserve fills every value from the tanker's
// config and physics (planner plan Task 8); the zero defaults are only unit-test fixtures.
struct TankerReserveParameters {
    double deorbit_delta_v_mps = 0.0;   // the previewed deorbit plan
    double exhaust_velocity_mps = 0.0;  // of the deorbit engine set
    double mixture_ratio = 0.0;         // oxidizer / fuel of the deorbit set
    double burn_margin = 1.0;           // the commit margin the return is held to (FlightSession::commit_burn_margin)
    double gas_kg = 0.0;                // cold gas after the transfer ([stage.recovery] return_gas_kg)
    // Boil-off until the burn, per propellant (the thermal model: LOX and CH4 boil off in their own ratio, r5)
    double boil_off_oxidizer_kg = 0.0;
    double boil_off_fuel_kg = 0.0;
    double unusable_kg = 0.0;           // trapped in sumps and lines ([stage.tanks] unusable_propellant_kg.main)
};

struct TankerReserve {
    double burn_kg = 0.0;
    double oxidizer_kg = 0.0;
    double fuel_kg = 0.0;
    double total_kg = 0.0;
    TankerReserveParameters inputs;  // what it was computed from
    double wait_s = 0.0;             // until the planned deorbit burn (FlightSession::tanker_reserve; 0: not planned)
};

// The main-tank residue a tanker keeps for its return (spec r4 "End"): the deorbit burn by the rocket equation from
// the mass after it (`dry_with_headers_kg`: the ship without main propellant, header tanks full - the landing reserve -
// plus the gas and the unusable residue), times the margin; plus the gas, the boil-off until the burn and the unusable
// residue. Split at the mixture ratio (the gas draws the main tanks in proportion, the burn at the engines' ratio), the
// boil-off as it boils.
TankerReserve tanker_reserve(double dry_with_headers_kg, const TankerReserveParameters& p);

}  // namespace starsim::mission
