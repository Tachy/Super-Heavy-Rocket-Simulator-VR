#pragma once

#include <filesystem>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/mission/lunar_budget.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"

namespace starsim::sim {

// Places the mission's HLS (`vehicle`: an override of the mission's vehicle) in its circular parking orbit at `epoch`,
// the plane through the Moon at arrival (epoch + flight_time_s; LunarTransferTargeter::parking_orbit_raan), and gives
// the session the Moon's terrain (data/ next to `config_dir`) and the mission's landing site. Throws std::runtime_error
// (German) when no such plane exists (inclination below the Moon's declination), the session has no Moon orientation
// or a terrain file is missing (the message names tools/data_pipeline/fetch.py).
void place_lunar_landing_mission(FlightSession& session, const config::LunarLandingMission& mission,
                                 const std::filesystem::path& config_dir, time::Epoch epoch,
                                 const config::VehicleConfig* vehicle = nullptr);

// The landing, surface stay and ascent of the mission ([descent], [surface], [ascent], [budget]).
LunarLandingSetup lunar_landing_setup(const config::LunarLandingMission& mission);

// Rocket-equation budget of the whole mission from the planned outbound flight (TLI, swing-by, insertion / LOI) and the
// mission's nominal legs (NRHO -> LLO, descent, ascent, LLO -> NRHO). `vacuum_isp_s`: the vacuum Raptors (outbound,
// transfers, braking phase, ascent); `approach_isp_s`: the rest of the descent (the last braking set's Isp - the
// landing thrusters since the l4 mass correction).
mission::BudgetResult lunar_mission_budget(const config::LunarLandingMission& mission, const mission::OutboundPlan& plan,
                                           double initial_mass_kg, double dry_mass_kg, double vacuum_isp_s,
                                           double approach_isp_s);

// start_mission's lunar branch (`vehicle`: an override, e.g. the 204 t HLS): place, check a touchdown point within the
// slope limit at the site (approach of a polar pass, both directions), plan the outbound flight, check the budget, then
// chain the legs (FlightSession::set_lunar_mission). Refused (return_planned false, German refusal, autopilot off):
// placement or terrain failed, no touchdown point, outbound plan refused, "Mission nicht durchführbar: " + budget.
// The pass over the site is checked in the LLO when the descent is planned (the LLO plane is known only then).
MissionStart start_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                         const config::LunarLandingMission& mission, time::Epoch epoch,
                                         const config::VehicleConfig* vehicle = nullptr);

// Outbound parameters of the mission started at `epoch` (FlightSession::plan_lunar_outbound); the NRHO variant builds
// its reference (multiple shooting, ~15 s) with the first perilune at the arrival.
mission::LunarOutboundParameters lunar_outbound_parameters(const config::LunarLandingMission& mission,
                                                           const environment::GravityModel& gravity,
                                                           const ephemeris::Ephemeris& ephemeris, time::Epoch epoch);

}  // namespace starsim::sim
