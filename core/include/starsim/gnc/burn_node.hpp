#pragma once

#include <optional>

#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

// Impulsive manoeuvre: inertial delta-v (GCRF) centred on `epoch`.
struct BurnNode {
    time::Epoch epoch;
    math::Vec3 delta_v_gcrf_mps;
};

// Orbit-relative components at a state: prograde (along v), normal (along r x v), radial (completes).
math::Vec3 delta_v_from_components(const math::StateVector& state, double prograde_mps, double normal_mps,
                                   double radial_mps);

// Two-body planning helpers (J2 and third bodies are corrected by the closed-loop burn execution).
std::optional<BurnNode> circularize_at_apoapsis(const math::StateVector& state, time::Epoch now, double gm_m3ps2);
std::optional<BurnNode> circularize_at_periapsis(const math::StateVector& state, time::Epoch now, double gm_m3ps2);

// Prograde burn at the next apoapsis raising the periapsis to `periapsis_radius_m` (two-body); empty if the orbit is
// not closed or the periapsis is already higher.
std::optional<BurnNode> raise_periapsis_at_apoapsis(const math::StateVector& state, time::Epoch now, double gm_m3ps2,
                                                    double periapsis_radius_m);
// Burn at the next apoapsis (prograde or retrograde) that moves the periapsis to `periapsis_radius_m` (two-body); empty
// if the orbit is not closed or the target lies above the apoapsis.
std::optional<BurnNode> set_periapsis_at_apoapsis(const math::StateVector& state, time::Epoch now, double gm_m3ps2,
                                                  double periapsis_radius_m);

// Burn duration for a delta-v from the rocket equation (constant thrust and mass flow).
double burn_duration_s(double delta_v_mps, double mass_kg, double thrust_n, double mass_flow_kgps);

// Executes a node closed-loop: ignition half the burn duration before the node, thrust along the
// remaining delta-v (tracked by integrating the sensed thrust acceleration), cutoff when the engine
// shutdown transient supplies the rest.
class BurnExecutor {
public:
    struct Status {
        double time_to_ignition_s = 0.0;
        double burn_duration_s = 0.0;
        double remaining_dv_mps = 0.0;
        math::Vec3 direction_gcrf;
        bool ignite = false;   // engines should be running
        bool complete = false;
    };

    explicit BurnExecutor(BurnNode node) : node_(node), remaining_(node.delta_v_gcrf_mps) {}

    const BurnNode& node() const { return node_; }

    // Once per tick, before integration. `thrust_n` / `mass_flow_kgps`: full-thrust performance of the
    // engines used for the burn (vacuum).
    // `rcs_acceleration_gcrf`: RCS translation acceleration applied during the last tick (RCS burns).
    // `thrust_axis_body`: direction of the engines' thrust in body axes (an asymmetric set gimballed through the CG
    // pushes along an axis tilted against the hull, gnc::thrust_axis_body).
    Status update(const vehicle::FlightBody& body, time::Epoch now, double dt_s, double thrust_n,
                  double mass_flow_kgps, double shutdown_time_s, const math::Vec3& rcs_acceleration_gcrf = {},
                  const math::Vec3& thrust_axis_body = {1.0, 0.0, 0.0});

private:
    BurnNode node_;
    math::Vec3 remaining_;
    bool started_ = false;
    bool complete_ = false;
};

}  // namespace starsim::gnc
