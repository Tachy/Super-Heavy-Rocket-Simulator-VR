#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>

#include "starsim/config/mission_config.hpp"
#include "starsim/sim/ascent_profile_planner.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// What goes into a launch stack besides the vehicle file.
struct AscentStackSpec {
    double payload_kg = 0.0;  // on the top stage
    bool fairing = true;      // false: the top stage's fairing dropped
    std::optional<config::MissionPayload> payload;     // a payload vehicle stacked on top (config::stack_payload)
    std::map<std::string, double> propellant_load_kg;  // stage id -> propellant (the others stay full)
};

// The launch stack of an ascent, the one place that builds it: `vehicle` with the spec's payload mass on its top stage,
// the fairing dropped, the named stages' tanks filled (std::invalid_argument for an unknown stage or a load above its
// capacity) and the payload vehicle stacked on top.
config::VehicleConfig build_ascent_stack(config::VehicleConfig vehicle, const std::filesystem::path& config_dir,
                                         const AscentStackSpec& spec);

// Sets up `session` for an ascent mission at `epoch`: the vehicle with the mission's payload (and fairing) on the pad, a
// payload vehicle stacked on top and released after cutoff ([payload]), and the launch planned and engaged by the onboard
// computer (engage_planned_ascent: the recovery - tower, landing zone or drone ship -, the reachable target and the
// kick). Returns the plan; a refused one leaves the stack on the pad with the autopilot off (its German reason).
// `vehicle`: a changed configuration of the mission's vehicle instead of its file (studies, refuelling r5 probes).
// `keep_station`: beside the session's station (refuelling f2, FlightSession::launch_from_pad).
AscentProfile start_ascent_mission(FlightSession& session, const config::AscentMission& mission,
                                   const std::filesystem::path& config_dir, time::Epoch epoch,
                                   const config::VehicleConfig* vehicle = nullptr, bool keep_station = false);

// start_ascent_mission_for_liftoff: where the vehicle was placed (the session epoch) and the launch's plan.
struct AscentLaunch {
    time::Epoch placed;
    AscentProfile profile;
};

// start_ascent_mission for a liftoff at `liftoff`. With `wait_from` (StartOptions::wait_on_pad) the vehicle stands on
// the pad from then (from the liftoff when wait_from is not earlier) and the liftoff is scheduled (FlightSession::
// schedule_liftoff), never less than liftoff_ignition_lead_s() after the placement (the engines must reach full thrust
// by T-0): a `fixed_liftoff` (a launch window, a configured epoch) is kept and the vehicle placed that lead before it,
// otherwise (a countdown) T-0 moves to the placement + lead. Without `wait_from` the vehicle is placed at the liftoff
// and lifts off as soon as its engines run. A refused plan schedules nothing.
AscentLaunch start_ascent_mission_for_liftoff(FlightSession& session, const config::AscentMission& mission,
                                              const std::filesystem::path& config_dir, time::Epoch liftoff,
                                              std::optional<time::Epoch> wait_from, bool fixed_liftoff = true);

}  // namespace starsim::sim
