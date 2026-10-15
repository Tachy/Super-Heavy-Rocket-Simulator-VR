#include "starsim/mission/finite_burn.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"

namespace starsim::mission {

namespace {
constexpr double powered_step_s = 0.5;  // RK4 on the powered arc (accelerations < 3 g, gravity smooth)
}

math::StateVector apply_finite_burn(const environment::GravityModel& gravity, const math::StateVector& at_node,
                                    time::Epoch node_epoch, const math::Vec3& delta_v_mps, const FiniteBurnModel& model) {
    const double dv = norm(delta_v_mps);
    if (!(dv > 0.0) || !(model.thrust_n > 0.0) || !(model.mass_flow_kgps > 0.0)) {
        return {at_node.position_m, at_node.velocity_mps + delta_v_mps};
    }
    const double duration = gnc::burn_duration_s(dv, model.mass_kg, model.thrust_n, model.mass_flow_kgps);
    const math::Vec3 u = delta_v_mps / dv;
    const time::Epoch ignition = node_epoch - 0.5 * duration;
    math::StateVector s = orbit::propagate_coast(gravity, at_node, node_epoch, ignition).state;

    const auto derivative = [&](double t, const math::StateVector& y) {
        const double mass = model.mass_kg - model.mass_flow_kgps * t;
        return math::StateVector{y.velocity_mps,
                                 gravity.acceleration_mps2(y.position_m, ignition + t) + u * (model.thrust_n / mass)};
    };
    const int steps = std::max(1, static_cast<int>(std::ceil(duration / powered_step_s)));
    const double h = duration / steps;
    for (int i = 0; i < steps; ++i) {
        const double t = i * h;
        const auto k1 = derivative(t, s);
        const auto k2 = derivative(t + 0.5 * h, {s.position_m + k1.position_m * (0.5 * h), s.velocity_mps + k1.velocity_mps * (0.5 * h)});
        const auto k3 = derivative(t + 0.5 * h, {s.position_m + k2.position_m * (0.5 * h), s.velocity_mps + k2.velocity_mps * (0.5 * h)});
        const auto k4 = derivative(t + h, {s.position_m + k3.position_m * h, s.velocity_mps + k3.velocity_mps * h});
        s.position_m += (k1.position_m + k2.position_m * 2.0 + k3.position_m * 2.0 + k4.position_m) * (h / 6.0);
        s.velocity_mps += (k1.velocity_mps + k2.velocity_mps * 2.0 + k3.velocity_mps * 2.0 + k4.velocity_mps) * (h / 6.0);
    }
    return orbit::propagate_coast(gravity, s, ignition + duration, node_epoch).state;
}

}  // namespace starsim::mission
