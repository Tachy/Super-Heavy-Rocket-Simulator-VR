#include "starsim/gnc/capsule_entry_control.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Quat;
using math::Vec3;

namespace {

struct Flow {
    Vec3 air_gcrf;   // velocity relative to the air
    Vec3 up_gcrf;    // radial (the geodetic vertical differs by < 0.2 deg: enough for a bank reference)
    environment::AtmosphereState air;
};

Flow flow_of(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch) {
    const auto t = env.earth_orientation.fixed_to_gcrf(epoch);
    const auto& s = body.state();
    Flow f;
    f.air_gcrf = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
    f.up_gcrf = math::normalized(s.position_gcrf_m);
    f.air = env.atmosphere.at(frames::fixed_to_geodetic(transpose(t.rotation) * s.position_gcrf_m).height_m);
    return f;
}

const aero::CapsuleAero& capsule_aero_of(const vehicle::Vehicle& v) {
    if (!v.capsule_aero()) throw std::logic_error("capsule entry control: vehicle has no capsule aerodynamics");
    return *v.capsule_aero();
}

math::Mat3 triad(const Vec3& a, const Vec3& b) {
    const Vec3 e1 = math::normalized(a);
    const Vec3 e2 = math::normalized(b - e1 * dot(b, e1));
    const Vec3 e3 = cross(e1, e2);
    math::Mat3 m;
    const Vec3 cols[3] = {e1, e2, e3};
    for (int c = 0; c < 3; ++c) {
        m(0, c) = cols[c].x;
        m(1, c) = cols[c].y;
        m(2, c) = cols[c].z;
    }
    return m;
}

// Rotation vector of q (shortest way).
Vec3 rotation_vector(const Quat& q) {
    const Quat p = q.w < 0.0 ? Quat{-q.w, -q.x, -q.y, -q.z} : q;
    const Vec3 v{p.x, p.y, p.z};
    const double s = norm(v);
    if (s < 1e-12) return {};
    return v * (2.0 * std::atan2(s, p.w) / s);
}

Vec3 hadamard(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }

Vec3 fractions(const Vec3& torque_nm, const Vec3& capability_nm) {
    const auto f = [](double t, double c) { return c > 0.0 ? std::clamp(t / c, -1.0, 1.0) : 0.0; };
    return {f(torque_nm.x, capability_nm.x), f(torque_nm.y, capability_nm.y), f(torque_nm.z, capability_nm.z)};
}

}  // namespace

double capsule_bank_rad(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch) {
    const auto& vehicle = body.vehicle();
    const auto& aero = capsule_aero_of(vehicle);
    const Flow f = flow_of(body, env, epoch);
    const auto& q = body.state().attitude;
    const Vec3 u_body = q.inverse_rotate(f.air_gcrf);
    const Vec3 u = math::normalized(f.air_gcrf);
    Vec3 lift = q.rotate(vehicle.hull_aero(u_body, f.air, false).force_body_n);
    lift = lift - u * dot(lift, u);
    if (norm(lift) < 1e-9) {  // vacuum: the trim's lift direction in the body
        lift = q.rotate(aero.trim(20.0, vehicle.mass_properties().cg_point_m()).lift_body_unit);
        lift = lift - u * dot(lift, u);
    }
    Vec3 e1, e2;
    steering_basis(f.up_gcrf, u, e1, e2);
    return std::atan2(-dot(lift, e2), dot(lift, e1));
}

Quat capsule_trim_attitude(const vehicle::Vehicle& capsule, const Vec3& air_velocity_gcrf, const Vec3& up_gcrf,
                           double bank_rad, double mach) {
    const auto t = capsule_aero_of(capsule).trim(mach, capsule.mass_properties().cg_point_m());
    const Vec3 u = math::normalized(air_velocity_gcrf);
    Vec3 e1, e2;
    steering_basis(up_gcrf, u, e1, e2);
    const Vec3 lift = e1 * std::cos(bank_rad) - e2 * std::sin(bank_rad);
    return Quat::from_matrix(triad(u, lift) * transpose(triad(t.air_velocity_body_unit, t.lift_body_unit))).normalized();
}

Vec3 capsule_entry_torque_fraction(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                   time::Epoch epoch, double bank_command_rad, const CapsuleControlGains& gains) {
    const auto& vehicle = body.vehicle();
    const Flow f = flow_of(body, env, epoch);
    const auto& s = body.state();
    const Vec3 inertia = vehicle.mass_properties().inertia_kg_m2;
    const Vec3& w = s.angular_velocity_body_radps;
    const double tau = gains.rate_time_constant_s;
    const double speed = norm(f.air_gcrf);
    const double q = 0.5 * f.air.density_kg_m3 * speed * speed;
    Vec3 dw;  // wanted change of the body rate

    if (q < gains.aero_hold_dynamic_pressure_pa) {
        // Thin air: hold the full trim attitude with the commanded bank.
        const Quat target = capsule_trim_attitude(vehicle, f.air_gcrf, f.up_gcrf, bank_command_rad);
        const Vec3 error = rotation_vector(s.attitude.conjugate() * target);  // body axes
        Vec3 rate = error * gains.hold_gain_per_s;
        if (norm(rate) > gains.hold_rate_max_radps) rate = rate * (gains.hold_rate_max_radps / norm(rate));
        if (norm(error) < gains.hold_deadband_rad && norm(w) < gains.roll_rate_deadband_radps) return {};
        dw = rate - w;
    } else {
        // Roll about the air velocity to the bank; damp the rest.
        const Vec3 u_body = math::normalized(s.attitude.inverse_rotate(f.air_gcrf));
        const double bank = capsule_bank_rad(body, env, epoch);
        const double error = bank_command_rad - bank;  // not wrapped: through lift up
        const double w_roll = dot(w, u_body);
        const Vec3 w_perp = w - u_body * w_roll;
        // d(bank)/dt = -w . u (entry_attitude: the lift turns left-handed about the velocity as the bank grows)
        const double w_roll_command =
            -std::clamp(gains.bank_gain_per_s * error, -gains.bank_rate_max_radps, gains.bank_rate_max_radps);
        if (std::abs(error) > gains.bank_deadband_rad || std::abs(w_roll) > gains.roll_rate_deadband_radps) {
            dw = u_body * (w_roll_command - w_roll);
        }
        if (norm(w_perp) > gains.damping_deadband_radps) dw = dw - w_perp;
    }
    return fractions(hadamard(inertia, dw) / tau, vehicle.max_rcs_torque_nm());
}

}  // namespace starsim::gnc
