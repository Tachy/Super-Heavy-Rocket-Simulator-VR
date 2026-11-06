#pragma once

#include <vector>

#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// The Sun seen from a point on the lunar reference sphere (R = 1737.4 km, MOON_ME latitude / longitude, east
// positive). The local horizon is the sphere's tangent plane (terrain is not considered). The direction is the
// Sun's centre, apparent (light time + stellar aberration, as the renderer draws it) for an observer at the site.
struct SiteSun {
    double elevation_rad = 0.0;  // above the geometric horizon
    double azimuth_rad = 0.0;    // from local north, east positive, [-pi, pi]
};

SiteSun site_sun(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& moon, double latitude_rad,
                 double longitude_rad, time::Epoch epoch);

// One daylight period: the Sun's centre above the geometric horizon from `sunrise` to `sunset`.
struct DaylightInterval {
    time::Epoch sunrise;
    time::Epoch sunset;
};

// Daylight intervals inside [from, from + span_s], in time order. Crossings are bracketed on a coarse grid (2 h) and
// bisected to 1 s. An interval that is already running at `from` starts at `from`; one still running at the end is
// closed at `from + span_s` (polar sites with no sunrise / sunset in the span: one interval clipped to the span, or
// none when dark). Assumes the Sun does not rise and set again within one grid step (true at every lunar site).
std::vector<DaylightInterval> daylight_intervals(const ephemeris::Ephemeris& ephemeris,
                                                 const frames::BodyOrientation& moon, double latitude_rad,
                                                 double longitude_rad, time::Epoch from, double span_s);

}  // namespace starsim::mission
