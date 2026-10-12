#include "starsim/sim/flight_session.hpp"

#include <algorithm>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/frames/geodetic.hpp"

namespace starsim::sim {

FlightSession::FlightSession(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& earth,
                             std::filesystem::path config_dir)
    : config_dir_(std::move(config_dir)),
      earth_(earth),
      gravity_(ephemeris, earth),
      env_{gravity_, atmosphere_, earth, &ephemeris, true},
      gm_earth_(ephemeris.gm_m3ps2(ephemeris::Body::Earth)) {}

const char* to_string(AutopilotState state) {
    switch (state) {
        case AutopilotState::Off: return "aus";
        case AutopilotState::PreLaunch: return "Startvorbereitung";
        case AutopilotState::Booster: return "Booster";
        case AutopilotState::UpperStage: return "Oberstufe";
        case AutopilotState::Orbit: return "Orbit erreicht";
    }
    return "";
}

void FlightSession::engage_ascent_autopilot(const gnc::AscentTarget& target, const gnc::PitchProgram& program) {
    guidance_ = std::make_unique<gnc::AscentGuidance>(target, program, gm_earth_);
    autopilot_ = {AutopilotState::PreLaunch, {}};
    if (!bodies_.empty() && !bodies_[controlled_].body->clamped()) {
        guidance_->start(*bodies_[controlled_].body, earth_, epoch_);
        autopilot_.state = bodies_[controlled_].body->vehicle().stages().size() > 1 ? AutopilotState::Booster
                                                                                    : AutopilotState::UpperStage;
    }
}

void FlightSession::fail_engine(size_t body_index, size_t stage_index, size_t engine_index) {
    bodies_.at(body_index).body->vehicle().fail_engine(stage_index, engine_index);
}

// Discrete autopilot decisions, once per tick before integration.
void FlightSession::run_autopilot() {
    auto& flying = bodies_[controlled_];
    auto& body = *flying.body;
    switch (autopilot_.state) {
        case AutopilotState::PreLaunch: {
            ignite();
            bool all_running = true;
            for (const auto& e : body.vehicle().stages().front().engines) {
                all_running = all_running && (e.mode == vehicle::EngineMode::Running || e.starved);
            }
            if (all_running) {
                release_clamps();
                guidance_->start(body, earth_, epoch_);
                autopilot_.state = AutopilotState::Booster;
            }
            break;
        }
        case AutopilotState::Booster: {
            const auto& booster = body.vehicle().stages().front();
            const double propellant = booster.propellant_kg(config::TankContents::Oxidizer) +
                                      booster.propellant_kg(config::TankContents::Fuel);
            if (propellant <= booster.config.reserve_propellant_kg) {
                stage();  // MECO + hot staging; the upper stage becomes the controlled body
                autopilot_.state = AutopilotState::UpperStage;
            }
            break;
        }
        case AutopilotState::UpperStage:
            if (autopilot_.guidance.cutoff) {
                cutoff();
                autopilot_.state = AutopilotState::Orbit;
            }
            break;
        default:
            break;
    }
}

void FlightSession::launch_from_pad(const std::string& vehicle_id, const LaunchSite& site, double heading_rad,
                                    time::Epoch epoch) {
    auto vehicle = std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir_, vehicle_id), config_dir_);
    bodies_.clear();
    bodies_.push_back({vehicle::FlightBody::on_pad(std::move(vehicle), site.latitude_rad, site.longitude_rad,
                                                   site.height_m, heading_rad, earth_, epoch),
                       {}});
    controlled_ = 0;
    epoch_ = epoch;
    launch_epoch_ = epoch;
}

vehicle::VehicleCommand FlightSession::command_for(size_t index, double ambient_pressure_pa) const {
    vehicle::VehicleCommand command;
    const auto& flying = bodies_[index];
    for (const auto& [group, run] : flying.running_groups) {
        command.groups.push_back({group, run, index == controlled_ ? input_.throttle : 1.0});
    }
    if (index != controlled_) {
        return command;
    }
    const bool guided = autopilot_.state == AutopilotState::Booster || autopilot_.state == AutopilotState::UpperStage;
    if (guided) {
        for (auto& group : command.groups) group.throttle = 1.0;
        controller_.steer(*flying.body, ambient_pressure_pa, autopilot_.guidance.thrust_direction_gcrf)
            .apply_to(command);
        return command;
    }
    // Axis convention: body +y points downrange (launch heading), z = x cross y. Pitch input > 0 tips
    // the nose towards +y (rotation about +z); yaw input > 0 turns it to the right of the heading, i.e.
    // towards -z (rotation about +y; z = up x heading points to the left).
    if (mode_ == ControlMode::RateCommand) {
        const math::Vec3 rates{input_.roll * max_roll_rate_radps, input_.yaw * max_pitch_yaw_rate_radps,
                               input_.pitch * max_pitch_yaw_rate_radps};
        controller_.steer_rates(*flying.body, ambient_pressure_pa, rates).apply_to(command);
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

time::Epoch FlightSession::advance_to(time::Epoch target, int max_ticks) {
    for (int tick = 0; tick < max_ticks && target - epoch_ >= tick_s; ++tick) {
        if (autopilot_.state != AutopilotState::Off && !bodies_.empty()) {
            run_autopilot();
            const bool closed_loop = autopilot_.state == AutopilotState::UpperStage;
            if (autopilot_.state == AutopilotState::Booster || closed_loop) {
                autopilot_.guidance = guidance_->update(*bodies_[controlled_].body, env_, epoch_, closed_loop);
            }
        }
        for (size_t i = 0; i < bodies_.size(); ++i) {
            auto& body = *bodies_[i].body;
            const auto t = earth_.fixed_to_gcrf(epoch_);
            const double altitude =
                frames::fixed_to_geodetic(transpose(t.rotation) * body.state().position_gcrf_m).height_m;
            body.step(env_, epoch_, tick_s, command_for(i, atmosphere_.at(altitude).pressure_pa));
        }
        epoch_ += tick_s;
    }
    if (bodies_.empty()) {
        epoch_ = target;
    }
    return epoch_;
}

void FlightSession::ignite() {
    if (bodies_.empty()) return;
    auto& flying = bodies_[controlled_];
    for (const auto& stage : flying.body->vehicle().stages()) {
        if (stage.engines.empty()) continue;
        for (const auto& e : stage.engines) flying.running_groups[e.group_id] = true;
        break;  // lowest stage with engines only
    }
}

void FlightSession::cutoff() {
    if (bodies_.empty()) return;
    for (auto& [group, run] : bodies_[controlled_].running_groups) run = false;
}

void FlightSession::release_clamps() {
    if (bodies_.empty() || !bodies_[controlled_].body->clamped()) return;
    bodies_[controlled_].body->release_clamps();
    launch_epoch_ = epoch_;  // T+0 = liftoff
}

void FlightSession::stage() {
    if (bodies_.empty() || bodies_[controlled_].body->vehicle().stages().size() < 2) return;
    auto& lower = bodies_[controlled_];
    auto upper = lower.body->separate(1);
    for (auto& [group, run] : lower.running_groups) run = false;  // booster engines off after separation
    Flying flying{std::move(upper), {}};
    bodies_.push_back(std::move(flying));
    controlled_ = bodies_.size() - 1;
    ignite();  // hot staging: the upper stage lights immediately
}

vehicle::FlightTelemetry FlightSession::telemetry(size_t index) const {
    return bodies_.at(index).body->telemetry(env_, epoch_);
}

}  // namespace starsim::sim
