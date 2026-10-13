#pragma once

#include <array>
#include <string>
#include <vector>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/math/vec_n.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::gnc {

// Planning frame of the landing guidance: origin at the pad centre on the ground, x up (geodetic vertical),
// y east, z north. Upright (body x along +x) is the identity attitude, so the tilt limit is a cone in the
// quaternion's (y, z) part.
inline math::Vec3 site_to_plan(const math::Vec3& enu) { return {enu.z, enu.x, enu.y}; }
inline math::Vec3 plan_to_site(const math::Vec3& p) { return {p.y, p.z, p.x}; }

// mass, position (3), velocity (3, relative to the ground), attitude quaternion (w, x, y, z; body -> plan),
// body angular rate (3, relative to the plan frame).
using DescentState = math::VecN<14>;

// Engine phases of the landing burn (V3 baseline): ignition set (13 engines), divert set (5), final set (3 centre
// engines, ending in a hover).
constexpr int descent_phases = 3;
using PhaseDurations = std::array<double, descent_phases>;
using Mat14x14 = std::array<double, 196>;  // row-major
using Mat14x3 = std::array<double, 42>;    // row-major

namespace descent_index {
constexpr int mass = 0;
constexpr int position = 1;
constexpr int velocity = 4;
constexpr int attitude = 7;
constexpr int rate = 11;
}  // namespace descent_index

DescentState make_state(double mass_kg, const math::Vec3& position_m, const math::Vec3& velocity_mps,
                        const math::Quat& attitude, const math::Vec3& rate_radps);
math::Vec3 state_position(const DescentState& x);
math::Vec3 state_velocity(const DescentState& x);
math::Quat state_attitude(const DescentState& x);
math::Vec3 state_rate(const DescentState& x);

// One engine set at sea-level pressure.
struct EngineSetModel {
    double thrust_max_n = 0.0;               // all engines at full throttle
    double thrust_min_n = 0.0;               // all engines at minimum throttle
    double mass_flow_per_thrust_s_m = 0.0;   // kg/s per N of vacuum-equivalent thrust (max flow / vacuum thrust)
    double gimbal_max_rad = 0.0;
    // Thrust lost to the ambient pressure (all engines, sea level): mass flow = k (|u| + this), since the flow
    // scales with throttle while the pressure loss does not.
    double pressure_thrust_loss_n = 0.0;
    double throttle_rate_per_s = 0.0;        // throttle change per second (fraction of full thrust; 0: unlimited)
};

// Planning model of the landing burn (documented simplifications): rigid body with constant inertia and CG,
// thrust at a gimbal point on the axis, mass flow proportional to thrust, tail-first axial drag along the
// motion through still air, gravity constant over the pad, Coriolis and centrifugal terms of the rotating
// plan frame; no grid fins, no wind. Air data and drag are tabulated so the model owns all its data (the
// planner can run on a worker thread).
struct DescentModel {
    std::array<EngineSetModel, descent_phases> engine_sets;  // per phase: ignition, divert, final set
    double gimbal_lever_m = 0.0;                // CG above the gimbal point
    math::Vec3 inertia_kg_m2;                   // principal, body axes
    double reference_area_m2 = 0.0;
    std::vector<double> cd_by_mach;             // every mach_step from Mach 0
    double mach_step = 0.05;
    std::vector<double> density_kg_m3;          // every height_step_m above the pad
    std::vector<double> speed_of_sound_mps;
    double height_step_m = 50.0;
    math::Vec3 gravity_mps2;                    // gravitation at the pad, plan frame
    math::Vec3 earth_rate_radps;                // Earth rotation, plan frame
    math::Vec3 earth_center_m;                  // Earth centre in the plan frame

    DescentState derivative(const DescentState& x, const math::Vec3& thrust_body_n, int phase) const;
    // Central differences; a = d derivative / d x, b = d derivative / d thrust.
    void jacobians(const DescentState& x, const math::Vec3& thrust_body_n, int phase, Mat14x14& a, Mat14x3& b) const;
};

// Snapshot inputs for the model (all evaluated on the simulation thread).
struct DescentModelInputs {
    const vehicle::Vehicle* vehicle = nullptr;    // the booster (single stage)
    std::array<std::vector<std::string>, descent_phases> phase_groups;  // engine groups per phase
    const environment::Atmosphere* atmosphere = nullptr;
    double site_height_m = 0.0;                   // pad ground above WGS84
    math::Vec3 gravity_mps2;                      // plan frame
    math::Vec3 earth_rate_radps;                  // plan frame
    math::Vec3 earth_center_m;                    // plan frame
};

DescentModel make_descent_model(const DescentModelInputs& inputs);

}  // namespace starsim::gnc
