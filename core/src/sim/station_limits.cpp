#include "starsim/sim/station_limits.hpp"

namespace starsim::sim {

gnc::ProximityParameters with_station_limits(gnc::ProximityParameters p, const config::StationProximityConfig& s) {
    if (s.keep_out_radius_m) p.keep_out_radius_m = *s.keep_out_radius_m;
    if (s.waypoint0_below_m) p.waypoint0_below_m = *s.waypoint0_below_m;
    if (s.waypoint1_range_m) p.waypoint1_range_m = *s.waypoint1_range_m;
    if (s.waypoint2_range_m) p.waypoint2_range_m = *s.waypoint2_range_m;
    if (s.flyaround_radius_m) p.flyaround_radius_m = *s.flyaround_radius_m;
    return p;
}

gnc::DepartureParameters with_station_limits(gnc::DepartureParameters d, const config::StationProximityConfig& s) {
    if (s.approach_ellipsoid_m) d.ellipsoid_semi_axes_m = *s.approach_ellipsoid_m;
    return d;
}

mission::PhasingParameters with_station_limits(mission::PhasingParameters p, const config::StationProximityConfig& s) {
    if (s.waypoint0_below_m) p.waypoint0_below_m = *s.waypoint0_below_m;
    return p;
}

}  // namespace starsim::sim
