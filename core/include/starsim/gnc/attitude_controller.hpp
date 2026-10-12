#pragma once

#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

struct GimbalSteering {
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    double roll_rad = 0.0;
};

// Thrust-vector attitude controller: points the body x-axis along a commanded direction.
// Second-order error dynamics per axis (natural frequency, damping), converted into gimbal angles
// from the current inertia, thrust and lever arm. Roll is only damped unless a roll reference is set.
class AttitudeController {
public:
    struct Gains {
        double natural_frequency_rad_s = 0.8;
        double damping_ratio = 0.9;
    };

    AttitudeController() = default;
    explicit AttitudeController(Gains gains) : gains_(gains) {}

    // `axis_gcrf`: desired direction of the body x-axis (unit). `roll_reference_gcrf` (optional, non-zero):
    // direction the body +y axis should point to, projected perpendicular to the axis.
    GimbalSteering steer(const vehicle::FlightBody& body, double ambient_pressure_pa, const math::Vec3& axis_gcrf,
                         const math::Vec3& roll_reference_gcrf = {}) const;

    // Rate command (stability augmentation for manual flight): tracks body angular rates.
    GimbalSteering steer_rates(const vehicle::FlightBody& body, double ambient_pressure_pa,
                               const math::Vec3& rates_body_radps) const;

private:
    Gains gains_;
};

}  // namespace starsim::gnc
