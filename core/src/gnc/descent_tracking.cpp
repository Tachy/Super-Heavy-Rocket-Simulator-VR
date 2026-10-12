#include "starsim/gnc/descent_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double rate_difference_step_s = 0.05;  // central difference of the planned rate

}  // namespace

// The plan's state_at interpolates linearly, so the planned rate is piecewise linear and its central difference is
// the planned angular acceleration.
TrackingReference sample_plan(const DescentPlan& plan, const DescentModel& model, double t, double terminal_sink_mps) {
    TrackingReference r;
    const double end = plan.duration_s();
    if (t >= end) {
        const auto& x = plan.reference.x.back();
        r.phase = 1;
        r.finished = true;
        r.velocity_mps = {-terminal_sink_mps, 0.0, 0.0};
        r.position_m = state_position(x) + r.velocity_mps * (t - end);
        r.thrust_acceleration_mps2 = -model.gravity_mps2;
        r.axis = {1.0, 0.0, 0.0};
        return r;
    }
    const auto x = plan.state_at(t);
    const auto q = state_attitude(x);
    r.phase = plan.phase_at(t);
    r.position_m = state_position(x);
    r.velocity_mps = state_velocity(x);
    r.thrust_acceleration_mps2 = q.rotate(plan.thrust_at(t)) / x[descent_index::mass];
    r.axis = q.rotate({1.0, 0.0, 0.0});
    r.rate_body_radps = state_rate(x);
    const double a = std::max(t - rate_difference_step_s, 0.0);
    const double b = std::min(t + rate_difference_step_s, end);
    if (b > a) r.angular_acceleration_body_radps2 = (state_rate(plan.state_at(b)) - state_rate(plan.state_at(a))) / (b - a);
    return r;
}

Vec3 commanded_acceleration(const TrackingReference& ref, const Vec3& position_m, const Vec3& velocity_mps,
                            const TrackingGains& g) {
    const double w = g.frequency_rad_s;
    Vec3 correction = (ref.position_m - position_m) * (w * w) + (ref.velocity_mps - velocity_mps) * (2.0 * g.damping * w);
    if (const double n = norm(correction); n > g.max_correction_mps2) correction = correction * (g.max_correction_mps2 / n);
    Vec3 a = ref.thrust_acceleration_mps2 + correction;
    const double up = std::max(a.x, 0.0);
    const double side = std::hypot(a.y, a.z);
    const double side_max = up * std::tan(g.max_tilt_rad);
    if (side > side_max && side > 0.0) {
        a.y *= side_max / side;
        a.z *= side_max / side;
    }
    return a;
}

}  // namespace starsim::gnc
