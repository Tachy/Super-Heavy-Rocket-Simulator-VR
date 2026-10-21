#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// How a body's flight ended (German UI text via to_string).
// TippedOver: a stage on its legs fell over after touching down (judged by the leg landing).
// SplashedDown: a splashdown vehicle (capsule) rests on the water - a planned recovery ("gewassert"), unlike Splashdown,
// a booster lost at sea ("Wasserung").
enum class LandingOutcome { Flying, Caught, Landed, Splashdown, Impact, BrokenUp, TippedOver, SplashedDown };

const char* to_string(LandingOutcome outcome);

// How a ship return ended (ship catch robustness, 2026-10-21): caught; an upright, controlled abort (water or ground
// reached upright and slow - the ship may be lost, the abort did its job); lost (tumbled, hard impact, broken up).
enum class ReturnResult { Flying, Caught, UprightAbort, Lost };
const char* to_string(ReturnResult result);
inline constexpr double upright_abort_max_tilt_rad = 20.0 * constants::deg_to_rad;  // [E] spec
inline constexpr double upright_abort_max_sink_mps = 5.0;  // [E] the upright descent brakes to <= 3 m/s; margin
ReturnResult return_result(LandingOutcome outcome, double contact_tilt_rad, double contact_sink_mps);

// Ground at (east, north) of the pad centre lies beyond the shoreline.
bool over_water(const config::SeaConfig& sea, double east_m, double north_m);

// Site frame (east, north, up from the pad centre on the ground) of an Earth-fixed position.
math::Vec3 site_enu_m(const LaunchSite& site, const math::Vec3& itrf_m);

// Broken up in flight (heat shield, structural load): BrokenUp. Caught on the tower; resting or impacted beyond the
// shoreline: splashdown (booster lost, tower undamaged);
// otherwise landed (resting) or impact.
LandingOutcome landing_outcome(const vehicle::FlightBody& body, const LaunchSite& site, const math::Vec3& position_itrf_m);

}  // namespace starsim::sim
