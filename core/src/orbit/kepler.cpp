#include "starsim/orbit/kepler.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "starsim/math/constants.hpp"

namespace starsim::orbit {

using math::StateVector;
using math::Vec3;

namespace {

constexpr double small = 1e-11;  // threshold for "circular" / "equatorial"

double wrap_two_pi(double angle) {
    angle = std::fmod(angle, constants::two_pi);
    return angle < 0.0 ? angle + constants::two_pi : angle;
}

double safe_acos(double x) { return std::acos(std::clamp(x, -1.0, 1.0)); }

// Stumpff functions C(z), S(z) with series near zero to avoid cancellation.
double stumpff_c(double z) {
    if (z > 1e-3) return (1.0 - std::cos(std::sqrt(z))) / z;
    if (z < -1e-3) return (std::cosh(std::sqrt(-z)) - 1.0) / (-z);
    return 1.0 / 2.0 - z / 24.0 + z * z / 720.0 - z * z * z / 40320.0;
}

double stumpff_s(double z) {
    if (z > 1e-3) {
        const double s = std::sqrt(z);
        return (s - std::sin(s)) / (s * s * s);
    }
    if (z < -1e-3) {
        const double s = std::sqrt(-z);
        return (std::sinh(s) - s) / (s * s * s);
    }
    return 1.0 / 6.0 - z / 120.0 + z * z / 5040.0 - z * z * z / 362880.0;
}

}  // namespace

double specific_energy(const StateVector& state, double gm) {
    return 0.5 * dot(state.velocity_mps, state.velocity_mps) - gm / norm(state.position_m);
}

double orbital_period_s(double semi_major_axis_m, double gm) {
    return constants::two_pi * std::sqrt(semi_major_axis_m * semi_major_axis_m * semi_major_axis_m / gm);
}

KeplerianElements elements_from_state(const StateVector& state, double gm) {
    const Vec3& r = state.position_m;
    const Vec3& v = state.velocity_mps;
    const double r_norm = norm(r);
    const Vec3 h = cross(r, v);
    const double h_norm = norm(h);
    const Vec3 node{-h.y, h.x, 0.0};  // k x h
    const double node_norm = norm(node);
    const Vec3 e_vec = ((dot(v, v) - gm / r_norm) * r - dot(r, v) * v) / gm;

    KeplerianElements el;
    el.eccentricity = norm(e_vec);
    el.semi_major_axis_m = -gm / (2.0 * specific_energy(state, gm));
    el.inclination_rad = safe_acos(h.z / h_norm);

    const bool equatorial = node_norm < small * h_norm;
    const bool circular = el.eccentricity < small;

    el.raan_rad = equatorial ? 0.0 : wrap_two_pi(std::atan2(node.y, node.x));

    if (!circular) {
        if (!equatorial) {
            double w = safe_acos(dot(node, e_vec) / (node_norm * el.eccentricity));
            el.argument_of_periapsis_rad = e_vec.z < 0.0 ? constants::two_pi - w : w;
        } else {
            const double w = std::atan2(e_vec.y, e_vec.x);
            el.argument_of_periapsis_rad = wrap_two_pi(h.z < 0.0 ? -w : w);
        }
        const double nu = safe_acos(dot(e_vec, r) / (el.eccentricity * r_norm));
        el.true_anomaly_rad = dot(r, v) < 0.0 ? constants::two_pi - nu : nu;
    } else if (!equatorial) {
        // Argument of latitude in place of the true anomaly.
        const double u = safe_acos(dot(node, r) / (node_norm * r_norm));
        el.true_anomaly_rad = r.z < 0.0 ? constants::two_pi - u : u;
    } else {
        // True longitude.
        const double l = std::atan2(r.y, r.x);
        el.true_anomaly_rad = wrap_two_pi(h.z < 0.0 ? -l : l);
    }
    return el;
}

StateVector state_from_elements(const KeplerianElements& el, double gm) {
    const double e = el.eccentricity;
    const double p = el.semi_major_axis_m * (1.0 - e * e);
    if (p <= 0.0) {
        throw std::invalid_argument("state_from_elements: invalid semi-latus rectum");
    }
    const double cos_nu = std::cos(el.true_anomaly_rad);
    const double sin_nu = std::sin(el.true_anomaly_rad);
    const double r = p / (1.0 + e * cos_nu);
    const double vf = std::sqrt(gm / p);
    // Perifocal frame
    const Vec3 r_pf{r * cos_nu, r * sin_nu, 0.0};
    const Vec3 v_pf{-vf * sin_nu, vf * (e + cos_nu), 0.0};

    const double co = std::cos(el.raan_rad), so = std::sin(el.raan_rad);
    const double cw = std::cos(el.argument_of_periapsis_rad), sw = std::sin(el.argument_of_periapsis_rad);
    const double ci = std::cos(el.inclination_rad), si = std::sin(el.inclination_rad);
    // Columns of R3(-raan) R1(-i) R3(-argp)
    const Vec3 p_hat{co * cw - so * sw * ci, so * cw + co * sw * ci, sw * si};
    const Vec3 q_hat{-co * sw - so * cw * ci, -so * sw + co * cw * ci, cw * si};

    return {r_pf.x * p_hat + r_pf.y * q_hat, v_pf.x * p_hat + v_pf.y * q_hat};
}

// Universal-variable Kepler propagation (Curtis, Orbital Mechanics for Engineering Students, Alg. 3.3/3.4).
StateVector propagate_two_body(const StateVector& state, double gm, double dt_s) {
    const Vec3& r0 = state.position_m;
    const Vec3& v0 = state.velocity_mps;
    const double r0_norm = norm(r0);
    const double vr0 = dot(r0, v0) / r0_norm;
    const double sqrt_mu = std::sqrt(gm);
    const double alpha = 2.0 / r0_norm - dot(v0, v0) / gm;  // 1/a

    double chi = sqrt_mu * std::abs(alpha) * dt_s;
    if (std::abs(alpha) < 1e-12 || chi == 0.0) {
        chi = sqrt_mu * dt_s / r0_norm;
    }
    for (int iteration = 0; iteration < 100; ++iteration) {
        const double z = alpha * chi * chi;
        const double c = stumpff_c(z);
        const double s = stumpff_s(z);
        const double f = r0_norm * vr0 / sqrt_mu * chi * chi * c + (1.0 - alpha * r0_norm) * chi * chi * chi * s +
                         r0_norm * chi - sqrt_mu * dt_s;
        const double df = r0_norm * vr0 / sqrt_mu * chi * (1.0 - z * s) + (1.0 - alpha * r0_norm) * chi * chi * c +
                          r0_norm;
        const double step = f / df;
        chi -= step;
        if (std::abs(step) <= 1e-13 * std::max(1.0, std::abs(chi))) {
            break;
        }
    }

    const double z = alpha * chi * chi;
    const double c = stumpff_c(z);
    const double s = stumpff_s(z);
    const double f = 1.0 - chi * chi / r0_norm * c;
    const double g = dt_s - chi * chi * chi * s / sqrt_mu;
    const Vec3 r = f * r0 + g * v0;
    const double r_norm = norm(r);
    const double f_dot = sqrt_mu / (r_norm * r0_norm) * (z * s - 1.0) * chi;
    const double g_dot = 1.0 - chi * chi / r_norm * c;
    return {r, f_dot * r0 + g_dot * v0};
}

}  // namespace starsim::orbit
