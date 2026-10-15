#pragma once

#include <filesystem>

#include "starsim/config/mission_config.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// Places the mission's HLS in its circular parking orbit at `epoch`, the plane through the Moon at arrival
// (epoch + flight_time_s; LunarTransferTargeter::parking_orbit_raan). Throws std::runtime_error (German) when no such
// plane exists (inclination below the Moon's declination) or the session has no Moon orientation.
void place_lunar_landing_mission(FlightSession& session, const config::LunarLandingMission& mission,
                                 const std::filesystem::path& config_dir, time::Epoch epoch);

// Outbound parameters of the mission started at `epoch` (FlightSession::plan_lunar_outbound); the NRHO variant builds
// its reference (multiple shooting, ~15 s) with the first perilune at the arrival.
mission::LunarOutboundParameters lunar_outbound_parameters(const config::LunarLandingMission& mission,
                                                           const environment::GravityModel& gravity,
                                                           const ephemeris::Ephemeris& ephemeris, time::Epoch epoch);

}  // namespace starsim::sim
