#pragma once

// Refuelling r5: the reference flights' start functions (dispatched by sim::start_mission). Each mission starts at
// its config's epoch (the ISS missions' convention), not at start_mission's `requested`.

#include <filesystem>
#include <optional>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

// depot_launch: the depot stack from its pad; at orbit insertion the ship becomes the station (sequence
// ascent_to_station), the booster flies on to the catch.
// `wait_from`: on the pad from then, the liftoff scheduled (StartOptions::wait_on_pad, start_ascent_mission_for_liftoff).
MissionStart start_depot_launch_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::DepotLaunchMission& mission, const time::TimeScaleConverter& time,
                                        std::optional<time::Epoch> wait_from = std::nullopt);

// The depot of a refuelling flight at any epoch: its [depot] mean elements at `reference` advanced by the J2 rates
// (true equator frame, as the synthetic ISS). `earth` must outlive the returned function.
mission::StationStateAt depot_orbit(const config::DepotOrbitConfig& depot, time::Epoch reference,
                                    const frames::BodyOrientation& earth);
// The [depot] mean elements at `reference` (circular; the depot_orbit's start).
orbit::StationMeanElements depot_mean_elements(const config::DepotOrbitConfig& depot, time::Epoch reference);
// A station orbit from mean elements, advanced by the J2 rates (true equator frame; refuelling f2: the flown depot).
// `earth` must outlive the returned function.
mission::StationStateAt station_orbit(const orbit::StationMeanElements& mean, const frames::BodyOrientation& earth);
// `behind_rad` of argument of latitude behind `station`, in its plane, on a periapsis_height_m x apoapsis_height_m
// orbit with the periapsis at the start (r3's phasing start).
math::StateVector phasing_start_state(const math::StateVector& station, double behind_rad, double periapsis_height_m,
                                      double apoapsis_height_m);
// The stage's main oxidizer and fuel tanks filled with `kg` in the ratio of their capacities (capped at capacity).
void fill_main_tanks(config::StageConfig& stage, double kg);
// The launch window parameters of a pad-start refuelling flight (as start_refuelling_flight_mission flies them; r6:
// the campaign's windows). f1: the separation on the ascent's insertion orbit, the phasing orbit chosen from the
// mission's candidates (`reference_phasing`: only the [phasing] periapsis_km orbit, r5) by their cost with the
// mission's planning values, `exhaust_velocity_mps` (phasing_exhaust_velocity_mps) and the settling gas per burn
// (refuelling f3: phasing_settling_gas_kg).
mission::LaunchWindowParameters refuelling_window_parameters(const config::RefuellingFlightMission& mission,
                                                             const config::AscentMission& ascent,
                                                             double exhaust_velocity_mps, double gas_per_burn_kg,
                                                             bool reference_phasing = false);
// The vacuum exhaust velocity of the single belly Raptor that flies the raise and the phasing burns (deorbit set).
double phasing_exhaust_velocity_mps(const vehicle::Vehicle& vehicle);
// Refuelling f3: the settling gas a phasing burn of the ship reserves (its bound over the fill,
// vehicle::settling_gas_bound_kg); 0 with the fidelity flag `ignition_settling` off.
double phasing_settling_gas_kg(const vehicle::Vehicle& vehicle, bool ignition_settling);
// A ship return mission's [deorbit] block as planner parameters; the pass search spans the tanker reserve's preview
// (FlightSession::tanker_reserve_search_s: from the 26.5 deg depot orbit no pass came within reach in 48 h, r4).
gnc::DeorbitParameters ship_deorbit_parameters(const config::ShipReturnDeorbit& deorbit);
// The chain of a pad-start refuelling flight (r5) in `window` with `parameters` (its phasing limits): Ascent -> Raise
// (the window's phasing orbit) -> Rendezvous -> Proximity -> Docking -> Transfer (tanker: down to its reserve; HLS:
// filled from the station) -> Departure. No ship return, no continuation (the callers add them).
IssMissionSequence pad_flight_sequence(const config::RefuellingFlightMission& mission,
                                       const mission::LaunchWindowParameters& parameters,
                                       const mission::LaunchWindow& window);
// Refuelling f2: a flight of the flown campaign - `mission`'s ascent launched at window.liftoff beside the session's
// station (keep_station; the session must be at that epoch), the chain of pad_flight_sequence. MissionStart.refusal
// (German) when the window is not feasible or the session is not at its liftoff (nothing changed then).
MissionStart launch_campaign_flight(FlightSession& session, const std::filesystem::path& config_dir,
                                    const config::RefuellingFlightMission& mission,
                                    const mission::LaunchWindow& window,
                                    const mission::LaunchWindowParameters& parameters);
// tanker_flight / hls_refuel: the depot placed from [depot]; the chaser launched in its window (Pad) or placed behind
// the depot (PhasingOrbit); the chain to the transfer and on (tanker: ship return to the catch).
// `wait_from` (a Pad start): as start_depot_launch_mission; the depot is placed at the session epoch.
MissionStart start_refuelling_flight_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                             const config::RefuellingFlightMission& mission,
                                             const time::TimeScaleConverter& time,
                                             std::optional<time::Epoch> wait_from = std::nullopt);

}  // namespace starsim::sim
