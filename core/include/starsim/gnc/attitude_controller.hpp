#pragma once

#include <vector>

#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

struct GimbalSteering {
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    double roll_rad = 0.0;
};

// Actuator commands that realise a desired body torque: grid fins first (if enabled), the rest by engine
// gimbals where running gimbaled engines give authority on an axis, reaction control otherwise.
struct ControlOutput {
    GimbalSteering gimbal;
    math::Vec3 rcs_torque_fraction;
    std::vector<double> grid_fin_rad;  // empty: fins neutral

    void apply_to(vehicle::VehicleCommand& command) const {
        command.gimbal_pitch_rad = gimbal.pitch_rad;
        command.gimbal_yaw_rad = gimbal.yaw_rad;
        command.gimbal_roll_rad = gimbal.roll_rad;
        command.rcs_torque_fraction = rcs_torque_fraction;
        command.grid_fin_rad = grid_fin_rad;
    }
};

// Attitude controller: second-order error dynamics per axis (natural frequency, damping) turned into a
// torque from the current inertia, then allocated to gimbals or RCS.
class AttitudeController {
public:
    struct Gains {
        double natural_frequency_rad_s = 0.8;
        double damping_ratio = 0.9;
        bool grid_fins = false;  // use the grid fins (descent); otherwise they stay neutral
    };

    AttitudeController() = default;
    explicit AttitudeController(Gains gains) : gains_(gains) {}

    // `axis_gcrf`: desired direction of the body x-axis (unit). `roll_reference_gcrf` (optional, non-zero):
    // direction the body +y axis should point to, projected perpendicular to the axis.
    ControlOutput steer(const vehicle::FlightBody& body, double ambient_pressure_pa, const math::Vec3& axis_gcrf,
                        const math::Vec3& roll_reference_gcrf = {}) const;

    // Rate command (stability augmentation for manual flight): tracks body angular rates.
    ControlOutput steer_rates(const vehicle::FlightBody& body, double ambient_pressure_pa,
                              const math::Vec3& rates_body_radps) const;

    // Allocates a desired body torque to grid fins (optional) and gimbals / RCS.
    static ControlOutput allocate(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                  const math::Vec3& torque_body_nm, bool grid_fins = false);

private:
    Gains gains_;
};

}  // namespace starsim::gnc
