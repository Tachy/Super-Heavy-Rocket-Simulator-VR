#include "starsim/mission/tanker_reserve.hpp"

#include <cmath>

namespace starsim::mission {

TankerReserve tanker_reserve(double dry_with_headers_kg, const TankerReserveParameters& p) {
    TankerReserve r;
    const double after_burn_kg = dry_with_headers_kg + p.gas_kg + p.unusable_kg;
    r.burn_kg = after_burn_kg * std::expm1(p.deorbit_delta_v_mps / p.exhaust_velocity_mps) * p.burn_margin;
    r.total_kg = r.burn_kg + p.gas_kg + p.boil_off_kg + p.unusable_kg;
    const double at_ratio_kg = r.burn_kg + p.gas_kg + p.unusable_kg;
    r.oxidizer_kg = at_ratio_kg * p.mixture_ratio / (1.0 + p.mixture_ratio) + p.boil_off_kg * (1.0 - p.boil_off_fuel_share);
    r.fuel_kg = at_ratio_kg / (1.0 + p.mixture_ratio) + p.boil_off_kg * p.boil_off_fuel_share;
    return r;
}

}  // namespace starsim::mission
