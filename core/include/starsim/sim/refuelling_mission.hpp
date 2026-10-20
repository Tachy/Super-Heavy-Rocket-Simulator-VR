#pragma once

// Refuelling r5: the reference flights' start functions (dispatched by sim::start_mission). Each mission starts at
// its config's epoch (the ISS missions' convention), not at start_mission's `requested`.

#include <filesystem>

#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

// depot_launch: the depot stack from its pad; at orbit insertion the ship becomes the station (sequence
// ascent_to_station), the booster flies on to the catch.
MissionStart start_depot_launch_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::DepotLaunchMission& mission, const time::TimeScaleConverter& time);

}  // namespace starsim::sim
