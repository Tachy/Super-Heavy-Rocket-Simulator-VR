#pragma once

#include <string>
#include <vector>

#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/descent_problem.hpp"
#include "starsim/gnc/landing_site.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

// Plan frame of a landing site: origin at the pad centre on the ground, x up (geodetic vertical), y east, z north.
struct PlanFrame {
    math::Mat3 plan_to_itrf;   // columns: up, east, north
    math::Vec3 origin_itrf_m;

    math::Vec3 to_plan(const math::Vec3& itrf_m) const { return transpose(plan_to_itrf) * (itrf_m - origin_itrf_m); }
    math::Vec3 direction_to_plan(const math::Vec3& itrf) const { return transpose(plan_to_itrf) * itrf; }
    math::Vec3 direction_to_itrf(const math::Vec3& plan) const { return plan_to_itrf * plan; }
};

PlanFrame plan_frame(double latitude_rad, double longitude_rad, double height_m);

// Current state of the body in the plan frame: velocity relative to the ground, attitude body -> plan, body rate
// relative to the (rotating) plan frame. `earth`: the Earth-fixed -> GCRF transform at the current epoch.
DescentState plan_state(const vehicle::FlightBody& body, const frames::FixedToInertial& earth, const PlanFrame& frame);

struct SnapshotSettings {
    std::vector<std::string> ignition_groups;  // phase 0 engine set
    std::vector<std::string> final_groups;     // phase 1 engine set
    double reserve_kg = 5000.0;        // [E] propellant the plan must leave (residuals, unusable)
    double catch_clearance_m = 1.0;    // pins above the arm tops at the end of the plan (spec)
    double catch_sink_mps = 1.0;       // sink rate at the end of the plan (spec)
    double sea_margin_m = 150.0;       // [E] abort target beyond the shoreline
    double mount_margin_m = 5.0;       // [E] abort: the hull bottom stays this far above the mount top near it
    // Vertical corridor from where the hull bottom is corridor_margin_m above the arm tops: within corridor_radius_m of
    // the catch axis (the arms wait half open with ~3 m clearance per side and close only at the end).
    double corridor_margin_m = 5.0;    // [E]
    double corridor_radius_m = 1.5;    // [E] half the standby clearance
    double final_set_radius_m = 3.0;   // [E] the final set (slow to turn) flies above the catch point
};

// Everything the planner needs, taken on the simulation thread (the worker must not touch SPICE, the environment
// or the vehicle): the model with its own tables, the start state and both targets.
struct DescentSnapshot {
    DescentModel model;
    DescentState initial;
    DescentTarget catch_target;   // goal Catch (meaningless without a catch target at the site)
    DescentTarget abort_target;   // goal Abort
    double min_mass_kg = 0.0;
    double phase0_spool = 1.0;    // mean spool of the ignition engines now
    double startup_s = 0.0;       // their start-up time
    time::Epoch epoch;
};

DescentSnapshot take_descent_snapshot(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                      time::Epoch epoch, const LandingSite& site, const PlanFrame& frame,
                                      const SnapshotSettings& settings);

}  // namespace starsim::gnc
