#pragma once

#include "starsim/math/vec3.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::gnc {

// State relative to the central body's centre, inertial axes (GCRF axes).
struct PegState {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
};

// The running engines now.
struct PegVehicle {
    double thrust_n = 0.0;
    double mass_flow_kgps = 0.0;
    double mass_kg = 0.0;
    double shutdown_s = 0.0;  // thrust tail-off time (cutoff lead)
};

// Insertion target: at radius_m with the given radial and horizontal speed, in the plane with unit normal plane_normal.
struct PegTarget {
    double radius_m = 0.0;
    double radial_speed_mps = 0.0;
    double horizontal_speed_mps = 0.0;
    math::Vec3 plane_normal;
    // Terminal phase (last seconds, velocity-to-be-gained steering): > 0 - horizontal at the vis-viva speed for this
    // semi-major axis at the current radius (Kepler target); 0 - radial_speed_mps / horizontal_speed_mps (solved
    // insertion, e.g. J2-aware).
    double terminal_semi_major_axis_m = 0.0;
};

struct PegCommand {
    math::Vec3 thrust_direction;  // inertial axes
    double time_to_go_s = 0.0;
    bool cutoff = false;
};

// Powered Explicit Guidance (linear-tangent steering, Space Shuttle PEG as formulated on OrbiterWiki "Powered Explicit
// Guidance") with zero-effort out-of-plane steering and terminal velocity-to-be-gained steering; about any central body
// (gm). Extracted unchanged from AscentGuidance (l4) so the lunar ascent uses the same law.
class PegSteering {
public:
    explicit PegSteering(double gm_m3ps2) : gm_(gm_m3ps2) {}
    PegCommand update(const PegState& state, const PegVehicle& vehicle, const PegTarget& target, time::Epoch epoch);
    bool converged() const { return converged_; }
    double time_to_go_s() const { return t_; }
    void reset() { converged_ = false; }

private:
    double gm_;
    double a_ = 0.0;
    double b_ = 0.0;
    double t_ = 0.0;  // time to go
    bool converged_ = false;
    time::Epoch last_;
};

}  // namespace starsim::gnc
