#include "starsim/gnc/attitude_controller.hpp"

#include <cmath>

namespace starsim::gnc {

using math::Vec3;

GimbalSteering AttitudeController::steer(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                         const Vec3& axis_gcrf, const Vec3& roll_reference_gcrf) const {
    const auto& state = body.state();
    const auto mass = body.vehicle().mass_properties();

    // Control authority of the gimbaled engines that are producing thrust.
    double gimbal_thrust = 0.0, lever_moment = 0.0, roll_moment = 0.0;
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) {
            if (!e.gimbaled || !e.producing_thrust()) continue;
            const double thrust = e.spool * e.performance->thrust_n(e.throttle, ambient_pressure_pa);
            gimbal_thrust += thrust;
            lever_moment += thrust * (mass.cg_station_m - e.gimbal_point_m.x);
            roll_moment += thrust * std::hypot(e.gimbal_point_m.y, e.gimbal_point_m.z);
        }
    }
    if (gimbal_thrust <= 0.0 || lever_moment <= 0.0) {
        return {};
    }

    // Pointing error as a rotation vector in body axes (small-angle about x-axis cross target).
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

    const double wn = gains_.natural_frequency_rad_s;
    const double kd = 2.0 * gains_.damping_ratio * wn;
    const Vec3& w = state.angular_velocity_body_radps;
    const Vec3& inertia = mass.inertia_kg_m2;
    const double torque_x = inertia.x * (wn * wn * roll_error - kd * w.x);
    const double torque_y = inertia.y * (wn * wn * rotation.y - kd * w.y);
    const double torque_z = inertia.z * (wn * wn * rotation.z - kd * w.z);

    // Moments of a deflection (see Vehicle::update_engines): pitch -> +tau_y, yaw -> -tau_z, roll -> +tau_x.
    GimbalSteering steering;
    steering.pitch_rad = torque_y / lever_moment;
    steering.yaw_rad = -torque_z / lever_moment;
    steering.roll_rad = roll_moment > 0.0 ? torque_x / roll_moment : 0.0;
    return steering;
}

}  // namespace starsim::gnc
