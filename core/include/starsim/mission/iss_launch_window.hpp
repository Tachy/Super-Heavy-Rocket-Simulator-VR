#pragma once

#include <functional>
#include <string>

#include "starsim/frames/body_orientation.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/phasing_schedule.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// RAAN shift (about the Earth's pole) of a separation orbit so that its plane lies on the station's after `match_s`:
// the J2 node regression is faster at the lower orbit (~0.58 deg/day at 205 vs 415 km, i3 correction 4). > 0: east.
double plane_bias_rad(double chaser_semi_major_axis_m, double station_semi_major_axis_m, double inclination_rad,
                      double match_s);

struct LaunchWindowParameters {
    double inclination_rad = 51.64 * constants::deg_to_rad;  // the ascent target (the ISS's)
    bool northerly = true;
    double insertion_s = 0.0;        // [D] liftoff -> payload separation (calibrated from the ascent)
    double insertion_arc_rad = 0.0;  // [D] inertial in-plane angle from the pad at liftoff to the separation point
    // [S] the separation orbit 200 x 210 km (falcon9_crew_dragon); the separation at its periapsis [E]
    double separation_periapsis_radius_m = constants::wgs84_semi_major_axis_m + 200e3;
    double separation_apoapsis_radius_m = constants::wgs84_semi_major_axis_m + 210e3;
    double search_s = 3.0 * 86400.0;  // [E] from the earliest liftoff (i6 correction 1)
    double sample_s = 600.0;          // [E] node-difference sampling (the Earth turns 2.5 deg in it)
    int bias_iterations = 2;          // [E] plane bias re-aimed at the schedule's boost epoch (i6 correction 2)
    PhasingParameters phasing;        // its mission limit counts from the separation
};

struct LaunchWindow {
    bool feasible = false;
    std::string reason;  // German when refused
    time::Epoch liftoff, separation;
    math::StateVector separation_state_gcrf;  // predicted
    double plane_bias_rad = 0.0;
    PhasingSchedule schedule;  // from the predicted separation state
    int rejected = 0;          // earlier crossings whose phasing did not close
};

// The station's GCRF state at an epoch (OEM or synthetic orbit, sim::iss_start).
using StationStateAt = std::function<math::StateVector(time::Epoch)>;

// The separation state: on the plane `plane_normal_gcrf` (the ascent's, fixed at liftoff), the pad's direction
// projected into it and turned by insertion_arc_rad prograde, at the separation periapsis with the periapsis speed.
math::StateVector separation_state_estimate(const math::Vec3& pad_gcrf_m, const math::Vec3& plane_normal_gcrf,
                                            const LaunchWindowParameters& p, double gm_m3ps2);

// The first liftoff at or after `earliest` (within search_s) at which the ascent guidance's target plane
// (gnc::ascent_plane_normal through the pad) is the station's plane turned by plane_bias_rad (match at the schedule's
// boost), and whose predicted separation state gives a feasible phasing_schedule. Refused: "Kein Startfenster in
// <n> Tagen: <reason of the last schedule>".
LaunchWindow plan_launch_window(const math::Vec3& pad_itrf_m, const frames::BodyOrientation& earth,
                                const StationStateAt& station, time::Epoch earliest, const LaunchWindowParameters& p,
                                double gm_m3ps2);

}  // namespace starsim::mission
