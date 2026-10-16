#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "starsim/config/mission_config.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

struct MissionStart {
    config::MissionKind kind = config::MissionKind::Ascent;
    std::string site_id;          // launch site of an ascent; empty for a return from orbit
    time::Epoch epoch;            // where the flight begins: `requested` for an ascent, the mission's own for a return
    // capsule return: the deorbit plan was feasible (else ship_return_plan()->reason); lunar landing: the mission was
    // planned (outbound flight, budget) and its legs chained
    bool return_planned = false;
    std::string refusal;  // lunar landing: German reason when refused (start_lunar_landing_mission)
};

// Starts mission `mission_id` of config/missions on `session` by its [mission] kind:
// - ascent: start_ascent_mission at `requested`;
// - capsule_return: place_capsule_return_mission at the mission's [orbit] epoch_utc (its RAAN belongs to that epoch),
//   then plan_capsule_return - the autopilot flies all steps to the water;
// - lunar_landing: start_lunar_landing_mission at the mission's [orbit] epoch_utc (needs a session with the Moon
//   orientation) - the autopilot flies the outbound flight, the landing, the surface stay and the ascent (mission 1);
// - ship_return: refused (std::invalid_argument): the ship returns from the orbit it flew to (plan_ship_return).
MissionStart start_mission(FlightSession& session, const std::filesystem::path& config_dir, std::string_view mission_id,
                           time::Epoch requested, const time::TimeScaleConverter& time);

}  // namespace starsim::sim
