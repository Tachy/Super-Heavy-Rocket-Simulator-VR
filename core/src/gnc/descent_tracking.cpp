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
        r.phase = descent_phases - 1;
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
    r.rate_plan_radps = q.rotate(state_rate(x));
    const double a = std::max(t - rate_difference_step_s, 0.0);
    const double b = std::min(t + rate_difference_step_s, end);
    // d/dt (R w_body) = R (dw_body/dt + w x w) = R dw_body/dt.
    if (b > a) r.angular_acceleration_plan_radps2 = q.rotate((state_rate(plan.state_at(b)) - state_rate(plan.state_at(a))) / (b - a));
    return r;
}

TrackingCommand tracking_command(const TrackingReference& ref, const Vec3& position_m, const Vec3& velocity_mps,
                                 const TrackingGains& g) {
    const double w = g.frequency_rad_s;
    Vec3 correction = (ref.position_m - position_m) * (w * w) + (ref.velocity_mps - velocity_mps) * (2.0 * g.damping * w);
    if (const double n = norm(correction); n > g.max_correction_mps2) correction = correction * (g.max_correction_mps2 / n);
    const double planned = norm(ref.thrust_acceleration_mps2);
    const Vec3 thrust_dir = planned > 0.0 ? ref.thrust_acceleration_mps2 / planned : ref.axis;
    const double along = dot(correction, thrust_dir);
    TrackingCommand cmd;
    cmd.thrust_acceleration_mps2 = std::max(planned + along, 0.0);
    Vec3 tilt = (correction - thrust_dir * along) / std::max(planned, constants::standard_gravity_mps2);
    if (const double n = norm(tilt); n > g.max_tilt_correction_rad) tilt = tilt * (g.max_tilt_correction_rad / n);
    cmd.axis = math::normalized(ref.axis + tilt);
    return cmd;
}

}  // namespace starsim::gnc
