#pragma once

#include <optional>

#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Guidance frame of the lunar descent (Apollo): origin at the touchdown point, x up (local vertical), z downrange (the
// approach azimuth), y crossrange completing the right-handed set (y = z x x). Positions and velocities are relative to
// the rotating Moon.
// Target of the braking phase (the "high gate"): position, velocity and acceleration, and the downrange jerk that fixes
// the time to go [S: Klumpp, "Apollo lunar descent guidance", Automatica 10 (1974) 133-146].
struct BrakingTarget {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
    math::Vec3 acceleration_mps2;
    double jerk_downrange_mps3 = 0.0;
};

// Klumpp's time to go: the positive root of J t^3 - 6 A t^2 + (18 V_T + 6 V) t - 24 (R_T - R) = 0 in the downrange
// components (J, A, V_T, R_T of the target; V, R now), found by Newton from `guess_s`; empty if Newton does not
// converge to a positive root within 30 iterations.
std::optional<double> braking_time_to_go_s(const BrakingTarget& target, double position_downrange_m,
                                           double velocity_downrange_mps, double guess_s);

// Quadratic-acceleration law: the acceleration now of the unique quadratic acceleration profile that reaches the target
// position, velocity and acceleration after t_go: a = a_T + 12 (r_T - r) / t_go^2 - 6 (v_T + v) / t_go.
math::Vec3 braking_acceleration(const BrakingTarget& target, const math::Vec3& position_m, const math::Vec3& velocity_mps,
                                double time_to_go_s);

// The braking phase: every update solves t_go (warm-started) and returns the thrust acceleration = law - gravity.
class ApolloBrakingGuidance {
public:
    explicit ApolloBrakingGuidance(BrakingTarget target, double time_to_go_guess_s = 400.0)
        : target_(target), t_go_(time_to_go_guess_s) {}
    struct Command {
        math::Vec3 thrust_acceleration_mps2;  // guidance frame
        double time_to_go_s = 0.0;
        bool valid = false;  // false: no time to go (past the target) - the caller ends the phase
    };
    // `gravity_mps2`: the effective gravity in the guidance frame (gravitation, the frame's centrifugal and Coriolis
    // terms).
    Command update(const math::Vec3& position_m, const math::Vec3& velocity_mps, const math::Vec3& gravity_mps2);
    const BrakingTarget& target() const { return target_; }

private:
    BrakingTarget target_;
    double t_go_;
};

}  // namespace starsim::gnc
