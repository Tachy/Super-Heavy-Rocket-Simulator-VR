#pragma once

#include <filesystem>
#include <optional>

#include "starsim/config/mission_config.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// Sets up `session` for an ascent mission at `epoch`: the vehicle with the mission's payload (and fairing) on the pad,
// the landing zone or drone ship of a returning first stage, booster return on, a payload vehicle stacked on top and
// released after cutoff ([payload]), and the ascent autopilot engaged with the mission's target and pitch program.
// `vehicle`: a changed configuration of the mission's vehicle instead of its file (studies, refuelling r5 probes).
// `keep_station`: beside the session's station (refuelling f2, FlightSession::launch_from_pad).
void start_ascent_mission(FlightSession& session, const config::AscentMission& mission,
                          const std::filesystem::path& config_dir, time::Epoch epoch,
                          const config::VehicleConfig* vehicle = nullptr, bool keep_station = false);

// start_ascent_mission for a liftoff at `liftoff`. With `wait_from` (StartOptions::wait_on_pad) the vehicle stands on
// the pad from then (from the liftoff when wait_from is not earlier) and the liftoff is scheduled (FlightSession::
// schedule_liftoff); without, it is placed at the liftoff and lifts off as soon as its engines run. Returns the epoch
// the vehicle was placed at (the session epoch).
time::Epoch start_ascent_mission_for_liftoff(FlightSession& session, const config::AscentMission& mission,
                                             const std::filesystem::path& config_dir, time::Epoch liftoff,
                                             std::optional<time::Epoch> wait_from);

}  // namespace starsim::sim
