#pragma once

#include <array>
#include <string>
#include <vector>

#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/frames/geodetic.hpp"
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

PlanFrame plan_frame(double latitude_rad, double longitude_rad, double height_m,
                     const frames::Ellipsoid& shape = frames::wgs84);
// On the site body's shape (Moon: the sphere; its x is the sphere's radial).
PlanFrame plan_frame(const LandingSite& site);

// The site body's frame at `epoch` (Earth: ITRF93 about the origin; Moon: env.moon, std::logic_error without it).
environment::BodyFrame site_body_frame(const LandingSite& site, const vehicle::FlightEnvironment& env, time::Epoch epoch);

// Direction of the net thrust in body axes when it exerts no pitch/yaw moment: from the thrust centroid of the
// running engines through the CG. An asymmetric set (two of the ship's three centre Raptors, or the last one) has its
// centroid off the axis, so the thrust is tilted against the body axis (~1.5 / 3 deg on the ship). Body x without
// thrust.
math::Vec3 thrust_axis_body(const vehicle::Vehicle& vehicle, double cg_station_m);
// The same for the engines of `groups` at full thrust (an engine set before it runs).
math::Vec3 thrust_axis_body(const vehicle::Vehicle& vehicle, double cg_station_m, const std::vector<std::string>& groups);

// Current state of the body in the plan frame: velocity relative to the ground, attitude body -> plan, body rate
// relative to the (rotating) plan frame. `frame`: the site body's frame at the current epoch (site_body_frame).
// The planning model's thrust acts along its body x through the CG, so the attitude is that of the thrust axis
// (thrust_axis_body), not of the hull: plans and the lateral control then steer the thrust, and an asymmetric engine
// set does not push sideways against them.
DescentState plan_state(const vehicle::FlightBody& body, const environment::BodyFrame& frame, const PlanFrame& plan);

struct SnapshotSettings {
    std::array<std::vector<std::string>, descent_phases> phase_groups;  // engine groups per phase
    double reserve_kg = 5000.0;        // [E] propellant the plan must leave (residuals, unusable)
    double catch_clearance_m = 1.0;    // pins above the arm tops at the end of the plan (spec)
    double catch_sink_mps = 1.0;       // sink rate at the end of the plan (spec)
    double sea_margin_m = 150.0;       // [E] abort target beyond the shoreline
    double sea_clearance_m = 30.0;     // [E] an abort must end at least this far beyond the shoreline (hard, while
                                       // reachable: the hull lies in the water, not on the beach)
    double mount_margin_m = 5.0;       // [E] abort: the hull bottom stays this far above the mount top near it
    // Vertical corridor from where the hull bottom is corridor_margin_m above the arm tops: within corridor_radius_m of
    // the catch axis (the arms wait half open with ~3 m clearance per side and close only at the end).
    double corridor_margin_m = 5.0;    // [E]
    double corridor_radius_m = 1.5;    // [E] half the standby clearance
    double final_set_radius_m = 3.0;   // [E] the final set (slow to turn) flies above the catch point
    // Pad landing: every final-set node this close to the touchdown point (soft; 0: no limit). A final set that can
    // hardly steer sideways (the HLS's landing thrusters on the RCS) must take over above the point (lunar l4).
    double pad_final_set_radius_m = 0.0;
};

// Everything the planner needs, taken on the simulation thread (the worker must not touch SPICE, the environment
// or the vehicle): the model with its own tables, the start state and both targets.
struct DescentSnapshot {
    DescentModel model;
    DescentState initial;
    DescentTarget catch_target;   // goal Catch, or Pad at a site with a pad target (meaningless without either)
    DescentTarget abort_target;   // goal Abort
    double min_mass_kg = 0.0;
    double phase0_spool = 1.0;    // mean spool of the ignition (phase 0) engines now
    double startup_s = 0.0;       // their start-up time
    double thrust_n = 0.0;        // thrust of all engines now (starting, running, running down)
    time::Epoch epoch;
};

DescentSnapshot take_descent_snapshot(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                      time::Epoch epoch, const LandingSite& site, const PlanFrame& frame,
                                      const SnapshotSettings& settings);

}  // namespace starsim::gnc
