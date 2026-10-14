#pragma once

#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

// How a body's flight ended (German UI text via to_string).
// TippedOver: a stage on its legs fell over after touching down (judged by the leg landing).
// SplashedDown: a splashdown vehicle (capsule) rests on the water - a planned recovery ("gewassert"), unlike Splashdown,
// a booster lost at sea ("Wasserung").
enum class LandingOutcome { Flying, Caught, Landed, Splashdown, Impact, BrokenUp, TippedOver, SplashedDown };

const char* to_string(LandingOutcome outcome);

// Ground at (east, north) of the pad centre lies beyond the shoreline.
bool over_water(const config::SeaConfig& sea, double east_m, double north_m);

// Site frame (east, north, up from the pad centre on the ground) of an Earth-fixed position.
math::Vec3 site_enu_m(const LaunchSite& site, const math::Vec3& itrf_m);

// Broken up in flight (heat shield, structural load): BrokenUp. Caught on the tower; resting or impacted beyond the
// shoreline: splashdown (booster lost, tower undamaged);
// otherwise landed (resting) or impact.
LandingOutcome landing_outcome(const vehicle::FlightBody& body, const LaunchSite& site, const math::Vec3& position_itrf_m);

}  // namespace starsim::sim
