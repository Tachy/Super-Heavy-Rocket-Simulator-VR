#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/departure_guidance.hpp"
#include "starsim/gnc/proximity_geometry.hpp"
#include "starsim/mission/phasing_schedule.hpp"

namespace starsim::sim {

// The target block's approach limits (refuelling r3) over the code defaults: only the keys the target sets replace
// the caller's values (the ISS sets none: its values are the defaults).
gnc::ProximityParameters with_station_limits(gnc::ProximityParameters p, const config::StationProximityConfig& s);
gnc::DepartureParameters with_station_limits(gnc::DepartureParameters d, const config::StationProximityConfig& s);
mission::PhasingParameters with_station_limits(mission::PhasingParameters p, const config::StationProximityConfig& s);

}  // namespace starsim::sim
