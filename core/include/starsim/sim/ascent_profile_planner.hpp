#pragma once

#include <optional>
#include <string>

#include "starsim/config/mission_config.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::sim {

// A launch from a pad the onboard computer plans before liftoff (ascent trajectory and staging, Task 3).
struct AscentProfileRequest {
    std::string vehicle_id;
    std::string site_id;
    double heading_rad = 0.0;  // body +y on the pad
    time::Epoch liftoff;
    gnc::AscentTarget target;
    std::optional<config::MissionPayload> payload;  // a payload vehicle on top of the stack (exclusive with payload_kg)
    double payload_kg = 0.0;
    bool fairing = true;
    std::optional<LaunchSite> landing_zone;  // the first stage's landing zone or drone ship (none: the site's tower)
};

// The first-stage pitch program the planner chose: the kick that leaves the upper stage the most propellant at SECO
// with the booster's return planned at MECO (meco_propellant_kg) and the target orbit reached.
struct AscentProfile {
    bool feasible = false;
    std::string reason;  // German, shown to the player when not feasible
    double kick_angle_rad = 0.0;
    double upper_propellant_at_seco_kg = 0.0;
    int trial_flights = 0;  // flown for this answer (0: from the memo)
};

// [E] The kick range searched: wide around the optima (Starship to 200 km / 26.5 deg: 14.9 deg; Falcon 9 RTLS to the ISS
// plane: 11.6 deg); the Starship sweep (test "[ascent_planner]") reaches the orbit only between 7 and 21 deg.
inline constexpr double min_kick_angle_rad = 4.0 * constants::deg_to_rad;
inline constexpr double max_kick_angle_rad = 25.0 * constants::deg_to_rad;
// [E] The search stops when the optimum is known to this (spec: ~0.2 deg of the true optimum).
inline constexpr double kick_tolerance_rad = 0.1 * constants::deg_to_rad;

// One trial ascent: a fresh session with `like`'s ephemeris, Earth orientation, gravity model and config dir, the
// request's stack on its pad, booster return on (MECO at the return's planned propellant), the ascent autopilot with
// the kick `kick_angle_rad`, flown at the flight's tick to the upper stage's cutoff (the booster's return itself is not
// flown: the MECO plan settles it). Returns the propellant (oxidizer + fuel) left in the upper stage once the engines'
// tail-off has ended; NaN when the flight is infeasible (no cutoff before the stage runs dry, the orbit flown off the
// target, the booster's MECO plan unusable).
double fly_ascent_trial(const FlightSession& like, const AscentProfileRequest& request, double kick_angle_rad);

// The kick of the request's ascent: searched on trial flights (fly_ascent_trial) in [min_kick_angle_rad,
// max_kick_angle_rad] to kick_tolerance_rad - Newton steps on a parabola through the best three trials, golden-section
// steps as the safeguard. Memoised per process by the request (its liftoff epoch excluded: the optimum does not depend
// on it), so a repeated mission start flies no trial.
AscentProfile plan_ascent_profile(const FlightSession& like, const AscentProfileRequest& request);

}  // namespace starsim::sim
