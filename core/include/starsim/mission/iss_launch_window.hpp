#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

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

// Refuelling f1: what a candidate phasing orbit costs - the propellant of its raise and boost (the later coelliptic
// and approach burns are the same for every candidate and left out) and the boil-off until Waypoint 0.
struct PhasingCostModel {
    double mass_kg = 0.0;               // after the ascent
    double exhaust_velocity_mps = 0.0;  // the phasing engine set
    double boil_off_kgps = 0.0;
    double gas_per_burn_kg = 0.0;  // refuelling f3: the settling push before each ignition
};

struct PhasingCost {
    double propellant_kg = 0.0;
    double boil_off_kg = 0.0;
    double total_kg() const { return propellant_kg + boil_off_kg; }
};

// [E] f1: the raise is done when both apsides are within this of the phasing orbit (planner and flown raise)
constexpr double raise_tolerance_m = 1e3;

// The raise from the separation to a phasing orbit, two-body with impulsive burns (gnc::next_raise_node).
struct RaiseEstimate {
    bool feasible = false;
    math::StateVector state;  // after the last burn
    double elapsed_s = 0.0;   // separation -> last burn
    double delta_v_mps = 0.0;
    int burns = 0;
};

RaiseEstimate raise_estimate(const math::StateVector& separation, const PhasingOrbit& orbit, double gm_m3ps2);

// The cost of a raised chaser's schedule towards a target on a circular orbit of target_radius_m: raise delta-v +
// the Hohmann boost to the coelliptic orbit (+ the phase burn and its return) by the rocket equation, plus the
// settling gas of each of those burns (f3) and boil-off over separation_to_waypoint0_s.
PhasingCost phasing_cost(const RaiseEstimate& raise, const PhasingSchedule& schedule,
                         double separation_to_waypoint0_s, const PhasingParameters& parameters, double target_radius_m,
                         const PhasingCostModel& model, double gm_m3ps2);

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
    // Refuelling f1: phasing orbits raised to after the separation; at each crossing the cheapest feasible one (by
    // `cost`) is taken. Empty: the separation orbit is the phasing orbit (the ISS missions).
    std::vector<PhasingOrbit> phasing_orbits;
    PhasingCostModel cost;
};

struct LaunchWindow {
    bool feasible = false;
    std::string reason;  // German when refused
    time::Epoch liftoff, separation;
    math::StateVector separation_state_gcrf;  // predicted
    double plane_bias_rad = 0.0;
    PhasingSchedule schedule;  // from the predicted separation state (with a phasing orbit: from the raised state)
    int rejected = 0;          // earlier crossings whose phasing did not close
    std::optional<PhasingOrbit> phasing_orbit;  // the chosen candidate (none without candidates)
    PhasingCost cost;
    time::Epoch raised;                     // the raise's last burn (= separation without a phasing orbit)
    math::StateVector raised_state_gcrf;    // predicted, after it
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
