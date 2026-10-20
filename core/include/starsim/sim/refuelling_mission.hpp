#pragma once

// Refuelling r5: the reference flights' start functions (dispatched by sim::start_mission). Each mission starts at
// its config's epoch (the ISS missions' convention), not at start_mission's `requested`.

#include <filesystem>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

// depot_launch: the depot stack from its pad; at orbit insertion the ship becomes the station (sequence
// ascent_to_station), the booster flies on to the catch.
MissionStart start_depot_launch_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::DepotLaunchMission& mission, const time::TimeScaleConverter& time);

// The depot of a refuelling flight at any epoch: its [depot] mean elements at `reference` advanced by the J2 rates
// (true equator frame, as the synthetic ISS). `earth` must outlive the returned function.
mission::StationStateAt depot_orbit(const config::DepotOrbitConfig& depot, time::Epoch reference,
                                    const frames::BodyOrientation& earth);
// `behind_rad` of argument of latitude behind `station`, in its plane, on a periapsis_height_m x apoapsis_height_m
// orbit with the periapsis at the start (r3's phasing start).
math::StateVector phasing_start_state(const math::StateVector& station, double behind_rad, double periapsis_height_m,
                                      double apoapsis_height_m);
// The stage's main oxidizer and fuel tanks filled with `kg` in the ratio of their capacities (capped at capacity).
void fill_main_tanks(config::StageConfig& stage, double kg);
// The launch window parameters of a pad-start refuelling flight (as start_refuelling_flight_mission flies them; r6:
// the campaign's windows).
mission::LaunchWindowParameters refuelling_window_parameters(const config::RefuellingFlightMission& mission,
                                                             const config::AscentMission& ascent);
// A ship return mission's [deorbit] block as planner parameters; the pass search spans the tanker reserve's preview
// (FlightSession::tanker_reserve_search_s: from the 26.5 deg depot orbit no pass came within reach in 48 h, r4).
gnc::DeorbitParameters ship_deorbit_parameters(const config::ShipReturnDeorbit& deorbit);
// tanker_flight / hls_refuel: the depot placed from [depot]; the chaser launched in its window (Pad) or placed behind
// the depot (PhasingOrbit); the chain to the transfer and on (tanker: ship return to the catch).
MissionStart start_refuelling_flight_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                             const config::RefuellingFlightMission& mission,
                                             const time::TimeScaleConverter& time);

}  // namespace starsim::sim
