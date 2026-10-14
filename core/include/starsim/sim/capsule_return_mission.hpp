#pragma once

#include <filesystem>

#include "starsim/config/mission_config.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// Places the mission's capsule (on its trunk) in its orbit at `epoch` with the RCS tank at rcs_propellant_kg, the
// nose cone open and the splashdown zone set; plan_capsule_return(capsule_return_parameters(m)) then flies it home.
void place_capsule_return_mission(FlightSession& session, const config::CapsuleReturnMission& mission,
                                  const std::filesystem::path& config_dir, time::Epoch epoch);

gnc::DeorbitParameters capsule_return_parameters(const config::CapsuleReturnMission& mission);

}  // namespace starsim::sim
