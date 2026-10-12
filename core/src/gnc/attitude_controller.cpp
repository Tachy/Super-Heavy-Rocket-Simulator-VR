#include "starsim/gnc/attitude_controller.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

namespace {

struct Authority {
    double lever_moment = 0.0;  // sum of thrust * lever arm of the gimbaled engines (pitch/yaw)
    double roll_moment = 0.0;   // sum of thrust * ring radius
    double gimbal_max_rad = 0.0;  // smallest deflection limit of the running gimbaled engines
};

Authority gimbal_authority(const vehicle::FlightBody& body, double cg_station_m, double ambient_pressure_pa) {
    Authority a;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) {
            if (!e.gimbaled || !e.producing_thrust()) continue;
            const double thrust = e.spool * e.performance->thrust_n(e.throttle, ambient_pressure_pa);
            a.lever_moment += thrust * (cg_station_m - e.gimbal_point_m.x);
            a.roll_moment += thrust * std::hypot(e.gimbal_point_m.y, e.gimbal_point_m.z);
            const double limit = e.performance->config().gimbal_max_rad;
            a.gimbal_max_rad = a.gimbal_max_rad > 0.0 ? std::min(a.gimbal_max_rad, limit) : limit;
        }
    }
    return a;
}

double fraction(double torque, double max) { return max > 0.0 ? torque / max : 0.0; }

// Fraction of the maximum angular deceleration planned for stopping a rotation (margin for lag and
// thrust changes).
constexpr double braking_margin = 0.5;

// Rate command for an attitude error: linear (natural frequency / damping) for small errors, limited for
// large ones to the rate that can still be stopped with the available angular acceleration, so a
// saturated actuator does not overshoot.
double rate_command(double error_rad, double linear_gain, double max_angular_acceleration) {
    const double linear = linear_gain * error_rad;
    if (max_angular_acceleration <= 0.0) return linear;
    const double braking = std::sqrt(2.0 * braking_margin * max_angular_acceleration * std::abs(error_rad));
    return std::copysign(std::min(std::abs(linear), braking), error_rad);
}

}  // namespace

ControlOutput AttitudeController::allocate(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                           const Vec3& torque) {
    const auto mass = body.vehicle().mass_properties();
    const Authority a = gimbal_authority(body, mass.cg_station_m, ambient_pressure_pa);
    const Vec3 rcs_max = body.vehicle().max_rcs_torque_nm();
    ControlOutput out;
    // Moments of a deflection (see Vehicle::update_engines): pitch -> +tau_y, yaw -> -tau_z, roll -> +tau_x.
    if (a.lever_moment > 0.0) {
        out.gimbal.pitch_rad = torque.y / a.lever_moment;
        out.gimbal.yaw_rad = -torque.z / a.lever_moment;
    } else {
        out.rcs_torque_fraction.y = fraction(torque.y, rcs_max.y);
        out.rcs_torque_fraction.z = fraction(torque.z, rcs_max.z);
    }
    if (a.roll_moment > 0.0) {
        out.gimbal.roll_rad = torque.x / a.roll_moment;
    } else {
        out.rcs_torque_fraction.x = fraction(torque.x, rcs_max.x);
    }
    return out;
}

ControlOutput AttitudeController::steer_rates(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                              const Vec3& rates_body_radps) const {
    const auto mass = body.vehicle().mass_properties();
    const Vec3& w = body.state().angular_velocity_body_radps;
    const Vec3& inertia = mass.inertia_kg_m2;
    const double k = 2.0 * gains_.damping_ratio * gains_.natural_frequency_rad_s;
    const Vec3 torque{inertia.x * k * (rates_body_radps.x - w.x), inertia.y * k * (rates_body_radps.y - w.y),
                      inertia.z * k * (rates_body_radps.z - w.z)};
    return allocate(body, ambient_pressure_pa, torque);
}

ControlOutput AttitudeController::steer(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                        const Vec3& axis_gcrf, const Vec3& roll_reference_gcrf) const {
    const auto& state = body.state();
    const auto mass = body.vehicle().mass_properties();

    // Pointing error as a rotation vector in body axes.
    const Vec3 target_body = state.attitude.inverse_rotate(math::normalized(axis_gcrf));
    const Vec3 axis_error = cross(Vec3{1.0, 0.0, 0.0}, target_body);
    const double sin_error = norm(axis_error);
    const double angle = std::atan2(sin_error, target_body.x);
    const Vec3 rotation = sin_error > 1e-12 ? axis_error * (angle / sin_error) : Vec3{};

    double roll_error = 0.0;
    if (norm(roll_reference_gcrf) > 0.0) {
        const Vec3 ref_body = state.attitude.inverse_rotate(roll_reference_gcrf);
        roll_error = std::atan2(ref_body.z, ref_body.y);  // rotate +y towards the reference about +x
    }

    // Torque = I * kd * (rate_command - rate); for small errors this is I * (wn^2 e - kd w).
    const double wn = gains_.natural_frequency_rad_s;
    const double kd = 2.0 * gains_.damping_ratio * wn;
    const Vec3& w = state.angular_velocity_body_radps;
    const Vec3& inertia = mass.inertia_kg_m2;
    const Authority a = gimbal_authority(body, mass.cg_station_m, ambient_pressure_pa);
    const Vec3 rcs_max = body.vehicle().max_rcs_torque_nm();
    auto max_acceleration = [&](double gimbal_moment, double rcs_torque, double moment_of_inertia) {
        const double torque = gimbal_moment > 0.0 ? gimbal_moment * std::sin(a.gimbal_max_rad) : rcs_torque;
        return moment_of_inertia > 0.0 ? torque / moment_of_inertia : 0.0;
    };
    const double gain = wn * wn / kd;
    const Vec3 rate{rate_command(roll_error, gain, max_acceleration(a.roll_moment, rcs_max.x, inertia.x)),
                    rate_command(rotation.y, gain, max_acceleration(a.lever_moment, rcs_max.y, inertia.y)),
                    rate_command(rotation.z, gain, max_acceleration(a.lever_moment, rcs_max.z, inertia.z))};
    const Vec3 torque{inertia.x * kd * (rate.x - w.x), inertia.y * kd * (rate.y - w.y), inertia.z * kd * (rate.z - w.z)};
    return allocate(body, ambient_pressure_pa, torque);
}

}  // namespace starsim::gnc
