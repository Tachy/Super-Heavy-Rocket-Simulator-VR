#pragma once

#include <filesystem>

#include "starsim/config/mission_config.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// Sets up `session` for an ascent mission at `epoch`: the vehicle with the mission's payload (and fairing) on the pad,
// the landing zone or drone ship of a returning first stage, booster return on, a payload vehicle stacked on top and
// released after cutoff ([payload]), and the ascent autopilot engaged with the mission's target and pitch program.
void start_ascent_mission(FlightSession& session, const config::AscentMission& mission,
                          const std::filesystem::path& config_dir, time::Epoch epoch);

}  // namespace starsim::sim
