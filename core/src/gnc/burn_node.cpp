#include "starsim/gnc/burn_node.hpp"

#include <cmath>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

// Time from now until the true anomaly reaches `target_anomaly` (elliptic orbits).
double time_to_anomaly(const orbit::KeplerianElements& el, double target_anomaly, double gm) {
    const double e = el.eccentricity;
    auto mean_anomaly = [e](double nu) {
        const double big_e = 2.0 * std::atan(std::sqrt((1.0 - e) / (1.0 + e)) * std::tan(nu / 2.0));
        return big_e - e * std::sin(big_e);
    };
    const double n = std::sqrt(gm / std::pow(el.semi_major_axis_m, 3));
    double dm = mean_anomaly(target_anomaly) - mean_anomaly(el.true_anomaly_rad);
    dm = std::fmod(dm + 2.0 * constants::two_pi, constants::two_pi);
    return dm / n;
}

std::optional<BurnNode> circularize_at(const math::StateVector& state, time::Epoch now, double gm, double anomaly) {
    const auto el = orbit::elements_from_state(state, gm);
    if (el.eccentricity >= 1.0 || el.semi_major_axis_m <= 0.0) {
        return std::nullopt;
    }
    const double dt = time_to_anomaly(el, anomaly, gm);
    const auto at_node = orbit::propagate_two_body(state, gm, dt);
    const double r = norm(at_node.position_m);
    const Vec3 h = cross(at_node.position_m, at_node.velocity_mps);
    const Vec3 horizontal = math::normalized(cross(h, at_node.position_m));
    const Vec3 circular = std::sqrt(gm / r) * horizontal;
    return BurnNode{now + dt, circular - at_node.velocity_mps};
}

}  // namespace

Vec3 delta_v_from_components(const math::StateVector& state, double prograde, double normal, double radial) {
    const Vec3 t = math::normalized(state.velocity_mps);
    const Vec3 n = math::normalized(cross(state.position_m, state.velocity_mps));
    const Vec3 r = cross(t, n);  // radial-out, perpendicular to v in the orbital plane
    return prograde * t + normal * n + radial * r;
}

std::optional<BurnNode> circularize_at_apoapsis(const math::StateVector& state, time::Epoch now, double gm) {
    return circularize_at(state, now, gm, constants::pi);
}

std::optional<BurnNode> circularize_at_periapsis(const math::StateVector& state, time::Epoch now, double gm) {
    return circularize_at(state, now, gm, 0.0);
}

double burn_duration_s(double delta_v, double mass_kg, double thrust_n, double mass_flow_kgps) {
    if (thrust_n <= 0.0 || mass_flow_kgps <= 0.0) return 0.0;
    const double ve = thrust_n / mass_flow_kgps;
    return mass_kg * (1.0 - std::exp(-delta_v / ve)) / mass_flow_kgps;
}

BurnExecutor::Status BurnExecutor::update(const vehicle::FlightBody& body, time::Epoch now, double dt_s,
                                          double thrust_n, double mass_flow_kgps, double shutdown_time_s) {
    Status st;
    const double mass = body.vehicle().mass_properties().mass_kg;

    // Integrate the thrust acceleration actually delivered since the last tick (sensed delta-v).
    double delivered = 0.0;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) {
            if (e.producing_thrust()) delivered += e.spool * e.performance->thrust_n(e.throttle, 0.0);
        }
    }
    if (delivered > 0.0) {
        started_ = true;
        remaining_ -= body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0}) * (delivered / mass * dt_s);
    }

    st.remaining_dv_mps = norm(remaining_);
    st.burn_duration_s = burn_duration_s(st.remaining_dv_mps, mass, thrust_n, mass_flow_kgps);
    st.direction_gcrf = st.remaining_dv_mps > 1e-9 ? remaining_ / st.remaining_dv_mps
                                                   : math::normalized(node_.delta_v_gcrf_mps);
    st.time_to_ignition_s = (node_.epoch - 0.5 * burn_duration_s(norm(node_.delta_v_gcrf_mps), mass, thrust_n,
                                                                   mass_flow_kgps)) - now;
    const double tail_off_dv = 0.5 * (thrust_n / mass) * shutdown_time_s;
    if (started_ && (st.remaining_dv_mps <= tail_off_dv || dot(remaining_, node_.delta_v_gcrf_mps) <= 0.0)) {
        complete_ = true;
    }
    st.complete = complete_;
    st.ignite = !complete_ && (started_ || st.time_to_ignition_s <= 0.0);
    return st;
}

}  // namespace starsim::gnc
