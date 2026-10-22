#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "starsim/config/mission_config.hpp"
#include "starsim/mission/iss_launch_window.hpp"
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
    std::vector<std::string> warnings;  // ISS missions: the ISS source note, station reboosts in the mission window
    std::optional<mission::LaunchWindow> launch_window;  // ISS launch / rendezvous from a window's separation
};

struct StartOptions {
    // Launch missions (ascent, iss_launch, depot_launch, and tanker_flight / hls_refuel from the pad) stand on the pad
    // from `requested` and lift off at their scheduled T-0 (FlightSession::schedule_liftoff): the window's liftoff, the
    // config's epoch (depot_launch) or, for an ascent, `requested` + countdown ([mission] countdown_s, else
    // `countdown_s` below). T-0 always lies at least FlightSession::liftoff_ignition_lead_s() after the placement: a
    // window's liftoff or the configured epoch is kept and the vehicle placed earlier when `requested` is later than
    // T-0 minus the lead (or than T-0); an ascent's countdown T-0 moves to `requested` + the lead. iss_launch: the
    // placement is moved up to where the ISS orbit source is the one of the liftoff (iss_mission.hpp). Stations of these
    // missions are placed at the session epoch. false: placed at the liftoff, lifting off at once (the default).
    bool wait_on_pad = false;
    double countdown_s = 0.0;
};

// Starts mission `mission_id` of config/missions on `session` by its [mission] kind:
// - ascent: start_ascent_mission at `requested`;
// - capsule_return: place_capsule_return_mission at the mission's [orbit] epoch_utc (its RAAN belongs to that epoch),
//   then plan_capsule_return - the autopilot flies all steps to the water;
// - lunar_landing: start_lunar_landing_mission at the mission's [orbit] epoch_utc (needs a session with the Moon
//   orientation) - the autopilot flies the outbound flight, the landing, the surface stay and the ascent (mission 1);
// - lunar_return: start_lunar_return_mission at the mission's [orbit] epoch_utc (Moon orientation needed) - the ship
//   is tanked for its return in the LLO / NRHO (mission 2, l5);
// - iss_rendezvous / iss_launch / iss_departure: sim/iss_mission.hpp (ISS i6; the ISS from data/iss_oem);
// - depot_launch / tanker_flight / hls_refuel: sim/refuelling_mission.hpp (refuelling r5);
// - refuelling_campaign_flown: sim/refuelling_campaign_flight.hpp (refuelling f2: the whole campaign in this
//   session, driven by its supervisor);
// - ship_return: refused (std::invalid_argument): the ship returns from the orbit it flew to (plan_ship_return).
// Every start removes a previous campaign's supervisor first.
MissionStart start_mission(FlightSession& session, const std::filesystem::path& config_dir, std::string_view mission_id,
                           time::Epoch requested, const time::TimeScaleConverter& time,
                           const StartOptions& options = {});

}  // namespace starsim::sim
