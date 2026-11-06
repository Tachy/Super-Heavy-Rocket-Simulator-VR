#include "starsim/gnc/descent_tracking.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double rate_difference_step_s = 0.05;  // central difference of the planned rate

DescentState with_normalized_attitude(DescentState x) {
    const auto q = state_attitude(x).normalized();
    x[descent_index::attitude] = q.w;
    x[descent_index::attitude + 1] = q.x;
    x[descent_index::attitude + 2] = q.y;
    x[descent_index::attitude + 3] = q.z;
    return x;
}

}  // namespace

PlanPath plan_path(const DescentPlan& plan, const DescentModel& model) {
    PlanPath path;
    const int n = plan.nodes_per_phase;
    const int last = static_cast<int>(plan.reference.x.size()) - 1;
    if (n < 2 || last < 1 || static_cast<int>(plan.reference.u.size()) != last + 1) return path;
    size_t samples = 1;
    for (int k = 0; k < last; ++k) {
        const double dt = plan.node_time_s(k + 1) - plan.node_time_s(k);
        if (dt > 0.0) samples += static_cast<size_t>(std::max(1.0, std::ceil(dt / plan_path_step_s)));
    }
    path.t_s.reserve(samples);
    path.x.reserve(samples);
    for (int k = 0; k < last; ++k) {
        const int phase = descent_node_phase(k, n);
        const double t0 = plan.node_time_s(k);
        const double dt = plan.node_time_s(k + 1) - t0;
        if (!(dt > 0.0)) continue;  // phases before the plan's start phase
        const int steps = std::max(1, static_cast<int>(std::ceil(dt / plan_path_step_s)));
        const double h = dt / steps;
        const Vec3& u0 = plan.reference.u[static_cast<size_t>(k)];
        const Vec3& u1 = plan.reference.u[static_cast<size_t>(k + 1)];
        const auto f = [&](const DescentState& y, double s) { return model.derivative(y, u0 + (u1 - u0) * (s / dt), phase); };
        DescentState x = plan.reference.x[static_cast<size_t>(k)];
        for (int i = 0; i < steps; ++i) {
            const double s = i * h;
            path.t_s.push_back(t0 + s);
            path.x.push_back(x);
            // RK4 over the first-order-hold thrust like linearize_interval, but with a finer step and a normalised
            // quaternion: the interval's end differs from its node by the planner's RK4 truncation plus its defect.
            const auto k1 = f(x, s);
            const auto k2 = f(x + k1 * (0.5 * h), s + 0.5 * h);
            const auto k3 = f(x + k2 * (0.5 * h), s + 0.5 * h);
            const auto k4 = f(x + k3 * h, s + h);
            x = with_normalized_attitude(x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (h / 6.0));
        }
    }
    if (path.t_s.empty()) return path;
    path.t_s.push_back(plan.node_time_s(last));  // the next interval (here: the end) starts from its node
    path.x.push_back(plan.reference.x.back());
    return path;
}

DescentState path_state(const PlanPath& path, double t) {
    if (path.t_s.empty()) return DescentState{};
    if (t <= path.t_s.front()) return path.x.front();
    if (t >= path.t_s.back()) return path.x.back();
    const size_t i = static_cast<size_t>(std::upper_bound(path.t_s.begin(), path.t_s.end(), t) - path.t_s.begin()) - 1;
    const double f = (t - path.t_s[i]) / (path.t_s[i + 1] - path.t_s[i]);
    const auto& a = path.x[i];
    const auto& b = path.x[i + 1];
    DescentState x = a * (1.0 - f) + b * f;
    const math::Quat qb = nearest_sign(state_attitude(b), state_attitude(a));
    const math::Quat qa = state_attitude(a);
    x[descent_index::attitude] = qa.w + (qb.w - qa.w) * f;
    x[descent_index::attitude + 1] = qa.x + (qb.x - qa.x) * f;
    x[descent_index::attitude + 2] = qa.y + (qb.y - qa.y) * f;
    x[descent_index::attitude + 3] = qa.z + (qb.z - qa.z) * f;
    return with_normalized_attitude(x);
}

// Between the samples of the path (or the nodes, without one) the planned rate is piecewise linear and its central
// difference is the planned angular acceleration.
TrackingReference sample_plan(const DescentPlan& plan, const DescentModel& model, double t, double terminal_sink_mps,
                              const PlanPath& path) {
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
    const bool on_path = !path.t_s.empty();
    const auto state = [&](double s) { return on_path ? path_state(path, s) : plan.state_at(s); };
    const auto x = state(t);
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
    if (b > a) r.angular_acceleration_plan_radps2 = q.rotate((state_rate(state(b)) - state_rate(state(a))) / (b - a));
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
