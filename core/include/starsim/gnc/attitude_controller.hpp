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
    std::vector<double> flap_rad;      // empty: flaps hold their deflection
    std::vector<double> throttle_trim;  // differential throttling per engine (stage order); empty: none

    void apply_to(vehicle::VehicleCommand& command) const {
        command.engine_throttle_trim = throttle_trim;
        command.gimbal_pitch_rad = gimbal.pitch_rad;
        command.gimbal_yaw_rad = gimbal.yaw_rad;
        command.gimbal_roll_rad = gimbal.roll_rad;
        command.rcs_torque_fraction = rcs_torque_fraction;
        command.grid_fin_rad = grid_fin_rad;
        command.flap_rad = flap_rad;
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
        // Use the flaps (ship entry): they trim the modelled aerodynamic moment and steer; otherwise they hold.
        bool flaps = false;
        // Pitch/yaw braking starts this much later than commanded (gimbals slewing to the opposite stop): large turns
        // are slowed earlier so they end without overshoot (0: instantaneous actuators).
        double actuator_delay_s = 0.0;
        // Slew-rate limit of pitch and yaw when the RCS turns them (no gimbal authority), rad/s; 0: none (roll is
        // free: a roll moves little mass far from the axis). A full-authority slew spends ~10x
        // the propellant of a slow one (start and stop impulse ~ rate), and the RCS supply is finite.
        double rcs_max_rate_radps = 0.0;
        // Attitude hold on pulsed RCS (orbit): per axis no torque while the error is within hold_deadband_rad and the
        // rate error within hold_rate_deadband_radps - the pulses then form a slow limit cycle instead of chattering
        // (0: off).
        double hold_deadband_rad = 0.0;
        double hold_rate_deadband_radps = 0.0;
        // Roll-rate limit when the RCS rolls (no gimbal roll authority), rad/s; 0: none. A blunt capsule's roll inertia
        // is close to its pitch inertia: an unlimited roll there costs as much as an unlimited pitch slew.
        double rcs_max_roll_rate_radps = 0.0;
    };

    AttitudeController() = default;
    explicit AttitudeController(Gains gains) : gains_(gains) {}

    // Planned motion the controller should not fight: body rate (relative to inertial space) and angular
    // acceleration, body axes. The roll components are ignored (roll follows the roll reference).
    struct Feedforward {
        math::Vec3 rate_body_radps;
        math::Vec3 acceleration_body_radps2;
    };

    // `axis_gcrf`: desired direction of the body x-axis (unit). `roll_reference_gcrf` (optional, non-zero):
    // direction the body +y axis should point to, projected perpendicular to the axis.
    ControlOutput steer(const vehicle::FlightBody& body, double ambient_pressure_pa, const math::Vec3& axis_gcrf,
                        const math::Vec3& roll_reference_gcrf = {}, const Feedforward* feedforward = nullptr) const;

    // Rate command (stability augmentation for manual flight): tracks body angular rates.
    ControlOutput steer_rates(const vehicle::FlightBody& body, double ambient_pressure_pa,
                              const math::Vec3& rates_body_radps) const;

    // Allocates a desired body torque to grid fins (optional) and gimbals / RCS. With `flaps` the torque is the net
    // torque wanted: the flaps cancel the modelled aerodynamic moment (hull + flaps, the trim) and add the rest by a
    // Newton step on their numerically linearised moments (minimum-norm, each flap clamped to its range, the null
    // space drawn towards mid-range); what they cannot give goes to gimbals / RCS.
    static ControlOutput allocate(const vehicle::FlightBody& body, double ambient_pressure_pa,
                                  const math::Vec3& torque_body_nm, bool grid_fins = false, bool flaps = false);

    // Grid fin deflections that turn each fin's plane into its local flow (no normal force, within its deflection
    // limit), from the latest air data: fins that carry no load (a planning model without fins stays true).
    static std::vector<double> feathered_grid_fins(const vehicle::FlightBody& body);

private:
    Gains gains_;
};

}  // namespace starsim::gnc
