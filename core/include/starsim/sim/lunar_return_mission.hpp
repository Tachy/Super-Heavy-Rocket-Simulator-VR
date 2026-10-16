#pragma once

#include <filesystem>
#include <string>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_corridor.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"

namespace starsim::sim {

// What the return needs (lunar l5): the entry speed from the corridor and the propellant from the planned burns.
struct LunarReturnBudget {
    bool closes = false;
    double propellant_kg = 0.0;  // to load in lunar orbit
    double tei_delta_v_mps = 0.0;  // NRHO: departure burn + powered flyby
    double braking_delta_v_mps = 0.0;
    double entry_speed_mps = 0.0;  // relative, at the interface (the highest with a corridor of the minimum width)
    gnc::EntryCorridor corridor;   // at that speed; the planned angle is its middle
    std::string reason;            // German when refused
};

// The mission's vehicle (`vehicle`: an override of the mission's) cut to its stage (the ship alone).
config::VehicleConfig lunar_return_ship(const config::LunarReturnMission& mission, const std::filesystem::path& config_dir,
                                        const config::VehicleConfig* vehicle = nullptr);

// The return settings of the mission ([return], [deorbit], [budget]); the NRHO variant builds its reference (multiple
// shooting, the first perilune two days after `epoch`). The entry speed, angle and model are set by
// lunar_return_budget (they come from the vehicle's heat shield).
LunarReturnSetup lunar_return_setup(const config::LunarReturnMission& mission, const math::Vec3& site_itrf_m,
                                    const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                                    time::Epoch epoch);

// The start state (Earth-centred GCRF): LLO - circular, of the mission's inclination (Moon equator), the plane
// containing the return asymptote at epoch + 1 h (the departure point a quarter revolution ahead); throws
// std::runtime_error (German) when the inclination is below the asymptote's declination. NRHO - the reference at `epoch`.
math::StateVector lunar_return_start_state(const FlightSession& session, const config::LunarReturnMission& mission,
                                           time::Epoch epoch, const mission::EarthReturnParameters& ret);

// Places `ship` (lunar_return_ship) with `propellant_kg` (its tanks scaled) at the start state, nose prograde, belly to
// the Moon. Throws std::runtime_error (German) without the session's Moon orientation.
void place_lunar_return_ship(FlightSession& session, const config::LunarReturnMission& mission,
                             const config::VehicleConfig& ship, time::Epoch epoch, double propellant_kg,
                             const mission::EarthReturnParameters& ret);

// Entry speed and propellant of the return of `ship` from the start state: the highest interface speed whose corridor
// is at least min_corridor_width wide (onboard predictor at the entry mass, radiative heating, the site's latitude,
// eastward), the corridor's middle as the planned angle (both set into `setup`, with the entry model for a direct
// entry), then the propellant fixed point: TEI (finite on `setup.burn_groups`), MCC allowance and braking burn on the
// vacuum Isp down to dry + landing propellant (aerocapture: + perigee raise and deorbit on the deorbit engines), coast
// attitude gas and the reserve. Refused (German): "Kein Eintrittskorridor ...", the planner's refusal, or the tanks.
// The session's launch site must be the mission's (the gate is the site's tower).
LunarReturnBudget lunar_return_budget(const FlightSession& session, const std::filesystem::path& config_dir,
                                      const config::LunarReturnMission& mission, const config::VehicleConfig& ship,
                                      time::Epoch epoch, LunarReturnSetup& setup);

// start_mission's lunar return branch: budget, placement with the budget's propellant, then the return autopilot.
// Refused (return_planned false, German refusal, autopilot off): no Moon orientation, no corridor, the plan or the
// tanks. `budget`: receives the budget when set.
MissionStart start_lunar_return_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::LunarReturnMission& mission, time::Epoch epoch,
                                        const config::VehicleConfig* vehicle = nullptr,
                                        LunarReturnBudget* budget = nullptr);

}  // namespace starsim::sim
