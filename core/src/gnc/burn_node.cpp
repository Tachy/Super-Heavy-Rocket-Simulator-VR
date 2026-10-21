#include "starsim/gnc/burn_node.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

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

std::optional<BurnNode> set_periapsis_at_apoapsis(const math::StateVector& state, time::Epoch now, double gm,
                                                  double periapsis_radius_m) {
    const auto el = orbit::elements_from_state(state, gm);
    if (el.eccentricity >= 1.0 || el.semi_major_axis_m <= 0.0) return std::nullopt;
    const double r_a = el.semi_major_axis_m * (1.0 + el.eccentricity);
    if (periapsis_radius_m > r_a) return std::nullopt;  // would make the apoapsis the periapsis
    const double a_new = 0.5 * (r_a + periapsis_radius_m);
    const double v_a = std::sqrt(gm * (2.0 / r_a - 1.0 / el.semi_major_axis_m));
    const double v_new = std::sqrt(gm * (2.0 / r_a - 1.0 / a_new));
    const double dt = time_to_anomaly(el, constants::pi, gm);
    const auto at_node = orbit::propagate_two_body(state, gm, dt);
    return BurnNode{now + dt, (v_new - v_a) * math::normalized(at_node.velocity_mps)};
}

std::optional<BurnNode> raise_periapsis_at_apoapsis(const math::StateVector& state, time::Epoch now, double gm,
                                                    double periapsis_radius_m) {
    const auto node = set_periapsis_at_apoapsis(state, now, gm, periapsis_radius_m);
    if (!node) return std::nullopt;
    const auto el = orbit::elements_from_state(state, gm);
    if (el.semi_major_axis_m * (1.0 - el.eccentricity) >= periapsis_radius_m) return std::nullopt;  // already higher
    return node;
}

std::optional<BurnNode> next_raise_node(const math::StateVector& state, time::Epoch now, double gm,
                                        double periapsis_radius_m, double apoapsis_radius_m, double tolerance_m) {
    const auto el = orbit::elements_from_state(state, gm);
    if (el.eccentricity >= 1.0 || el.semi_major_axis_m <= 0.0) return std::nullopt;
    const double r_p = el.semi_major_axis_m * (1.0 - el.eccentricity);
    const double r_a = el.semi_major_axis_m * (1.0 + el.eccentricity);
    // 1. a higher apoapsis: burn at the current apoapsis so that the opposite apsis becomes the target apoapsis - or,
    // for a target periapsis below the current apoapsis, at the current periapsis (f2)
    if (apoapsis_radius_m > r_a + tolerance_m) {
        const bool at_periapsis = periapsis_radius_m < r_a - tolerance_m;
        const double dt = time_to_anomaly(el, at_periapsis ? 0.0 : constants::pi, gm);
        const auto at_node = orbit::propagate_two_body(state, gm, dt);
        const double r = norm(at_node.position_m);
        const double v_new = std::sqrt(gm * (2.0 / r - 2.0 / (r + apoapsis_radius_m)));
        return BurnNode{now + dt, (v_new - norm(at_node.velocity_mps)) * math::normalized(at_node.velocity_mps)};
    }
    // 2. the periapsis (raised only: a higher one is left alone)
    if (r_p < periapsis_radius_m - tolerance_m) return set_periapsis_at_apoapsis(state, now, gm, periapsis_radius_m);
    return std::nullopt;
}

std::optional<BurnNode> next_raise_node_mean(const math::StateVector& state, time::Epoch now, double gm,
                                             double mean_periapsis_m, double mean_apoapsis_m, double periapsis_radius_m,
                                             double apoapsis_radius_m, double tolerance_m, bool periapsis_only) {
    const auto el = orbit::elements_from_state(state, gm);
    if (el.eccentricity >= 1.0 || el.semi_major_axis_m <= 0.0) return std::nullopt;
    const double a = 0.5 * (mean_periapsis_m + mean_apoapsis_m);
    // the prograde burn at apsis radius r of the mean orbit that moves the opposite apsis to `opposite_m`
    const auto burn = [&](double anomaly, double r, double opposite_m) -> std::optional<BurnNode> {
        const double dv = std::sqrt(gm * (2.0 / r - 2.0 / (r + opposite_m))) - std::sqrt(gm * (2.0 / r - 1.0 / a));
        if (!(dv > 0.0)) return std::nullopt;  // raised only
        const double dt = time_to_anomaly(el, anomaly, gm);
        const auto at_node = orbit::propagate_two_body(state, gm, dt);
        return BurnNode{now + dt, dv * math::normalized(at_node.velocity_mps)};
    };
    if (!periapsis_only && apoapsis_radius_m > mean_apoapsis_m + tolerance_m) {
        if (periapsis_radius_m < mean_apoapsis_m - tolerance_m) return burn(0.0, mean_periapsis_m, apoapsis_radius_m);
        return burn(constants::pi, mean_apoapsis_m, apoapsis_radius_m);
    }
    if (mean_periapsis_m < periapsis_radius_m - tolerance_m) {
        return burn(constants::pi, mean_apoapsis_m, periapsis_radius_m);
    }
    return std::nullopt;
}

double burn_duration_s(double delta_v, double mass_kg, double thrust_n, double mass_flow_kgps) {
    if (thrust_n <= 0.0 || mass_flow_kgps <= 0.0) return 0.0;
    const double ve = thrust_n / mass_flow_kgps;
    return mass_kg * (1.0 - std::exp(-delta_v / ve)) / mass_flow_kgps;
}

BurnExecutor::Status BurnExecutor::update(const vehicle::FlightBody& body, time::Epoch now, double dt_s,
                                          double thrust_n, double mass_flow_kgps, double shutdown_time_s,
                                          const Vec3& rcs_acceleration_gcrf, const Vec3& thrust_axis_body) {
    Status st;
    const double mass = body.vehicle().mass_properties().mass_kg;

    // Integrate the thrust acceleration actually delivered since the last tick (sensed delta-v: an accelerometer measures
    // the true thrust, so the truth thrust dispersion is included - l5 Task 10: with the nominal thrust a +1.7 % engine
    // overran the 1,033 m/s braking burn by ~17 m/s).
    double delivered = 0.0;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) {
            if (e.producing_thrust()) delivered += e.spool * e.performance->thrust_n(e.throttle, 0.0);
        }
    }
    delivered *= body.vehicle().thrust_factor();
    if (delivered > 0.0) {
        if (!started_ && std::getenv("STARSIM_DEBUG_BURN")) {
            std::printf("[burn] ignition %.1f s from the node (planned %.1f s), dv %.3f m/s\n", now - node_.epoch,
                        -0.5 * burn_duration_s(norm(node_.delta_v_gcrf_mps), mass, thrust_n, mass_flow_kgps),
                        norm(node_.delta_v_gcrf_mps));
        }
        started_ = true;
        remaining_ -= body.state().attitude.rotate(thrust_axis_body) * (delivered / mass * dt_s);
    }
    if (norm(rcs_acceleration_gcrf) > 0.0) {
        started_ = true;
        remaining_ -= rcs_acceleration_gcrf * dt_s;
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
        if (std::getenv("STARSIM_DEBUG_BURN")) {
            std::printf("[burn] cutoff %.1f s from the node, remaining %.3f m/s (along %.3f)\n", now - node_.epoch,
                        st.remaining_dv_mps, dot(remaining_, math::normalized(node_.delta_v_gcrf_mps)));
        }
    }
    st.complete = complete_;
    st.ignite = !complete_ && (started_ || st.time_to_ignition_s <= 0.0);
    return st;
}

}  // namespace starsim::gnc
