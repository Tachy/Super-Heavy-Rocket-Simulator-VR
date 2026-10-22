// FlightSession: the per-tick vehicle command of each body.

#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/gnc/capsule_entry_control.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/docking_port.hpp"

namespace starsim::sim {

vehicle::VehicleCommand FlightSession::command_for(size_t index, double ambient_pressure_pa) const {
    auto command = steering_command_for(index, ambient_pressure_pa);
    apply_staging_throttle(index, command);  // a staging sequence sets its run-on and upper-stage throttles
    return command;
}

vehicle::VehicleCommand FlightSession::steering_command_for(size_t index, double ambient_pressure_pa) const {
    vehicle::VehicleCommand command;
    const auto& flying = bodies_[index];
    for (const auto& [group, run] : flying.running_groups) {
        command.groups.push_back({group, run, index == controlled_ ? input_.throttle : 1.0});
    }
    if (flying.parachutes && flying.parachutes->phase() != gnc::ParachutePhase::Armed) {
        return command;  // no steering under the parachutes (user 2026-10-14): the Dracos stay off
    }
    if (index == controlled_ && docking_free_drift()) {
        return command;  // ISS i5: free drift from the first contact to the hooks (the mechanism aligns the vehicle)
    }
    if (flying.ship_landing) {
        if (!flying.body->impacted()) apply_return_command(flying, ship_landing_controller_, ambient_pressure_pa, command);
        return command;
    }
    if (flying.entry_guidance && flying.entry_guidance->capsule()) {
        // The Dracos fly the bank; the aerodynamic trim holds the angle of attack. After the gate: lift up, damped.
        if (!flying.body->impacted() && !flying.body->resting()) {
            const auto& e = flying.entry_command;
            const double bank = e.phase == gnc::EntryPhase::Gate ? 0.0 : e.bank_rad;
            command.rcs_torque_fraction = gnc::capsule_entry_torque_fraction(*flying.body, env_, epoch_, bank);
        }
        return command;
    }
    if (flying.entry_guidance) {
        if (!flying.body->impacted()) {
            const auto& e = flying.entry_command;
            const gnc::AttitudeController::Feedforward feedforward{
                flying.body->state().attitude.inverse_rotate(e.rate_gcrf_radps), {},
                flying.entry_attitude_bias_nm, e.phase != gnc::EntryPhase::Gate, e.flap_rest_rad};
            const auto out =
                entry_controller_.steer(*flying.body, ambient_pressure_pa, e.axis_gcrf, e.roll_reference_gcrf, &feedforward);
            out.apply_to(command);
            static const bool debug = std::getenv("STARSIM_DEBUG_ENTRY_ATTITUDE") != nullptr;
            if (debug && std::fmod(epoch_.tdb_seconds_past_j2000(), 1.0) < tick_s) {  // diagnostics: once per second
                const auto& s = flying.body->state();
                const math::Vec3 axis = s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0});
                const double axis_error = std::acos(std::clamp(dot(axis, e.axis_gcrf), -1.0, 1.0));
                std::printf("[entry attitude] axis error %.1f deg, rates %+.3f/%+.3f/%+.3f rad/s, rcs %+.2f/%+.2f/%+.2f, flaps",
                            axis_error / constants::deg_to_rad, s.angular_velocity_body_radps.x,
                            s.angular_velocity_body_radps.y, s.angular_velocity_body_radps.z, out.rcs_torque_fraction.x,
                            out.rcs_torque_fraction.y, out.rcs_torque_fraction.z);
                for (const double f : out.flap_rad) std::printf(" %.0f", f / constants::deg_to_rad);
                // the hull moment correction as the pointing error a P term would need for it (entry wn 0.5 rad/s)
                const auto inertia = flying.body->vehicle().mass_properties().inertia_kg_m2;
                const auto& b = flying.entry_attitude_bias_nm;
                const auto err = gnc::AttitudeController::pointing_error(*flying.body, e.axis_gcrf, e.roll_reference_gcrf);
                std::printf(" | error %+.2f/%+.2f/%+.2f deg bias %+.2f/%+.2f/%+.2f deg k %.3f phase %s\n",
                            err.x / constants::deg_to_rad, err.y / constants::deg_to_rad, err.z / constants::deg_to_rad,
                            b.x / (inertia.x * 0.25) / constants::deg_to_rad, b.y / (inertia.y * 0.25) / constants::deg_to_rad,
                            b.z / (inertia.z * 0.25) / constants::deg_to_rad, flying.entry_hull_moment_scale, gnc::to_string(e.phase));
            }
        }
        return command;
    }
    const bool lunar_descent = lunar_step_ == LunarStep::PoweredDescent && lunar_landing_;
    const bool lunar_ascent =
        (lunar_step_ == LunarStep::Ascent || lunar_step_ == LunarStep::AbortAscent) && lunar_ascent_;
    if (index == controlled_ && autopilot_.state == AutopilotState::Lunar && (lunar_descent || lunar_ascent)) {
        // Powered lunar descent and ascent: flown as the ship's landing burn (gimbals + RCS; the fixed vacuum Raptors:
        // RCS only; the canted landing thrusters: differential throttling).
        if (!flying.body->impacted()) apply_return_command(flying, ship_landing_controller_, ambient_pressure_pa, command);
        return command;
    }
    if (index != controlled_ || flying.return_guidance) {
        if (flying.return_guidance && !flying.body->impacted()) {
            const bool coasting = flying.return_guidance->phase() == gnc::ReturnPhase::Coast;
            apply_return_command(flying, coasting ? return_coast_controller_ : return_controller_, ambient_pressure_pa,
                                 command);
        }
        return command;
    }
    // Staging: the stack (then the upper stage until its ignition) holds the guidance's last command
    const bool guided = autopilot_.state == AutopilotState::Booster || autopilot_.state == AutopilotState::Staging ||
                        autopilot_.state == AutopilotState::UpperStage;
    if (guided || autopilot_.state == AutopilotState::Burn) {
        const double throttle = guided ? autopilot_.guidance.throttle : burn_config_.throttle;
        for (auto& group : command.groups) group.throttle = throttle;
        const auto& direction = guided ? autopilot_.guidance.thrust_direction_gcrf : autopilot_.burn.direction_gcrf;
        const auto& vehicle = flying.body->vehicle();
        const math::Vec3 thrust_axis = guided            ? math::Vec3{1.0, 0.0, 0.0}
                                       : burn_config_.rcs ? burn_config_.thrust_axis_body
                                                          : gnc::thrust_axis_body(vehicle, vehicle.mass_properties().cg_station_m,
                                                                                  burn_config_.groups);
        // RCS burns turn and hold on pulses: the orbit controller (rate-limited slews, hold deadband); the ascent
        // controller chattered on the Dracos' pulses (~3 kg of a 1 m/s burn's 7 kg).
        if (burn_drifting()) return command;  // free drift: no RCS until the turn to the burn attitude
        // Turning to the burn and holding it until the preparation: the coast controller's slow slews and deadbands
        // (l5 aircatch: the ascent controller's tight hold spent 7.8 t of RCS gas in b4's 108 h wait; the burn 5.5 t).
        const bool waiting = !guided && !autopilot_.burn.ignite &&
                             autopilot_.burn.time_to_ignition_s > burn_preparation_s + settling_lead_s_;
        const auto& steering = waiting                          ? lunar_coast_controller_
                               : !guided && burn_config_.rcs ? orbit_controller_
                                                             : controller_;
        if (thrust_axis.x < 1.0 - 1e-9) {
            // Asymmetric set (the ship's deorbit Raptor): its thrust goes through the CG along an axis tilted against the
            // hull; the hull turns so that this axis points along the burn, its roll kept where it is.
            const auto& attitude = flying.body->state().attitude;
            const math::Vec3 x0 = math::normalized(direction);
            const math::Vec3 y_now = attitude.rotate(math::Vec3{0.0, 1.0, 0.0});
            const math::Vec3 y0 = math::normalized(y_now - x0 * dot(y_now, x0));
            const math::Vec3 z0 = cross(x0, y0);
            math::Mat3 m;
            const math::Vec3 columns[3] = {x0, y0, z0};
            for (int c = 0; c < 3; ++c) {
                m(0, c) = columns[c].x;
                m(1, c) = columns[c].y;
                m(2, c) = columns[c].z;
            }
            const math::Vec3 x{1.0, 0.0, 0.0};
            const math::Vec3 turn_axis = cross(thrust_axis, x);
            // Anti-parallel (an RCS burn along -x): half a turn about body y - it keeps +y, so the roll reference below
            // keeps the roll where it is (a half turn about z flipped +y: a 180 deg roll error chased every tick).
            const math::Quat to_x =
                norm(turn_axis) > 1e-9
                    ? math::Quat::from_axis_angle(turn_axis, std::acos(std::clamp(thrust_axis.x, -1.0, 1.0)))
                    : math::Quat::from_axis_angle({0.0, 1.0, 0.0}, constants::pi);
            const math::Quat hull = math::Quat::from_matrix(m) * to_x;  // hull.rotate(thrust_axis) = x0
            steering.steer(*flying.body, ambient_pressure_pa, hull.rotate(x), hull.rotate(math::Vec3{0.0, 1.0, 0.0}))
                .apply_to(command);
        } else {
            steering.steer(*flying.body, ambient_pressure_pa, direction).apply_to(command);
        }
        // Orbit manoeuvre: turning towards the burn must not push the vehicle (an unrequested push moves the orbit).
        command.rcs_keep_force = !guided;
        if (!guided && burn_config_.rcs && autopilot_.burn.ignite && !autopilot_.burn.complete) {
            // Translation along the remaining delta-v, independent of the attitude.
            command.rcs_force_fraction = flying.body->state().attitude.inverse_rotate(autopilot_.burn.direction_gcrf);
        }
        return command;
    }
    if (autopilot_.state == AutopilotState::Return && capsule_return_step_ != CapsuleReturnStep::None) {
        // After the deorbit burn: hold its attitude while the trunk is on, then turn to the trim attitude for the entry
        // (heat shield to the flow, lift up).
        const auto& s = flying.body->state();
        const auto& vehicle = flying.body->vehicle();
        math::Quat target = capsule_hold_attitude_;
        if (vehicle.capsule_aero()) {
            const auto t = earth_.fixed_to_gcrf(epoch_);
            const math::Vec3 air = s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(t), s.position_gcrf_m);
            target = gnc::capsule_trim_attitude(vehicle, air, math::normalized(s.position_gcrf_m), 0.0);
        }
        orbit_controller_.steer(*flying.body, ambient_pressure_pa, target.rotate(math::Vec3{1.0, 0.0, 0.0}),
                                target.rotate(math::Vec3{0.0, 1.0, 0.0}))
            .apply_to(command);
        command.rcs_keep_force = true;
        return command;
    }
    if (attitude_mode_active()) {
        const auto& s = flying.body->state();
        gnc::AttitudeMode mode = attitude_mode_;
        if (mode.kind == gnc::AttitudeModeKind::Sun && env_.ephemeris) {
            mode.direction_gcrf = env_.ephemeris->state_gcrf(ephemeris::Body::Sun, epoch_).position_m - s.position_gcrf_m;
        }
        // LVLH about the reference body (lunar l1); Inertial / Direction do not depend on the centre.
        const auto ref = vehicle::reference_body_of(env_, flying.body->reference_body(env_, epoch_));
        const auto target = gnc::attitude_target(mode, ref.frame(epoch_).centred({s.position_gcrf_m, s.velocity_gcrf_mps}));
        const gnc::AttitudeController::Feedforward feedforward{s.attitude.inverse_rotate(target.rate_gcrf_radps), {}};
        const auto& hold = autopilot_.state == AutopilotState::Lunar ? lunar_coast_controller_ : orbit_controller_;
        hold.steer(*flying.body, ambient_pressure_pa, target.axis_gcrf, target.roll_reference_gcrf,
                                &feedforward)
            .apply_to(command);
        command.rcs_keep_force = true;  // an orbit: turns must not push the vehicle
        if (index == controlled_) apply_translation(flying, command);
        return command;
    }
    // Axis convention: body +y points downrange (launch heading), z = x cross y. Pitch input > 0 tips
    // the nose towards +y (rotation about +z); yaw input > 0 turns it to the right of the heading, i.e.
    // towards -z (rotation about +y; z = up x heading points to the left).
    if (mode_ == ControlMode::RateCommand) {
        const math::Vec3 rates{input_.roll * max_roll_rate_radps, input_.yaw * max_pitch_yaw_rate_radps,
                               input_.pitch * max_pitch_yaw_rate_radps};
        controller_.steer_rates(*flying.body, ambient_pressure_pa, rates).apply_to(command);
        if (norm(input_.translation) > 0.0) {
            command.rcs_force_fraction = input_.translation;  // RCS translation (the pilot's, body axes)
        } else if (index == controlled_) {
            apply_translation(flying, command);
        }
    } else {
        double max_gimbal = 0.0;
        for (const auto& stage : flying.body->vehicle().stages()) {
            for (const auto& e : stage.engines) max_gimbal = std::max(max_gimbal, e.performance->config().gimbal_max_rad);
        }
        // Engines sit below the CG: a deflection towards +y turns the nose towards -y, towards +z the
        // nose towards -z (right).
        command.gimbal_yaw_rad = -input_.pitch * max_gimbal;
        command.gimbal_pitch_rad = input_.yaw * max_gimbal;
        command.rcs_torque_fraction = {input_.roll, input_.yaw, input_.pitch};  // same sense as the gimbals
        command.gimbal_roll_rad = input_.roll * max_gimbal;
    }
    return command;
}

bool FlightSession::translation_flown() const {
    if (!rendezvous_ || norm(translation_port_) <= 0.0) return false;
    if (attitude_mode_active()) return true;
    return mode_ == ControlMode::RateCommand &&
           (autopilot_.state == AutopilotState::Off || autopilot_.state == AutopilotState::Orbit ||
            autopilot_.state == AutopilotState::Lunar);
}

void FlightSession::apply_translation(const Flying& flying, vehicle::VehicleCommand& command) const {
    if (!translation_flown()) return;
    const auto target = vehicle::port_kinematics(*bodies_.at(rendezvous_->target_index).body, rendezvous_->target_port);
    command.rcs_force_fraction = flying.body->state().attitude.inverse_rotate(target.axes_gcrf * translation_port_);
    // The hold's torque comes first: a full pure translation saturates the Dracos, and with the force kept the hold
    // lost up to 14 deg in 20 s; torque first it stays below 1 deg (test_rendezvous_session).
    command.rcs_keep_force = false;
    command.rcs_force_pure = true;  // a pure translation: no torque, no cross push (docking)
}

}  // namespace starsim::sim
