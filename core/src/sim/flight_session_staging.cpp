// FlightSession: the staging sequence (ascent trajectory and staging, Task 1). Cold staging (Falcon 9): MECO, the stack
// coasts through the tail-off, separation with the pushers' push, the upper stage lights after its delay. Hot staging
// (Super Heavy): MECO with the centre engines at minimum throttle, the ship lights while joined, separation once its
// engines are at thrust. Momentum is conserved throughout: the tail-off and the ship's start-up act on the stack.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>

#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

namespace {

bool stage_producing_thrust(const vehicle::Stage& stage) {
    return std::any_of(stage.engines.begin(), stage.engines.end(),
                       [](const vehicle::Engine& e) { return e.producing_thrust(); });
}

}  // namespace

std::optional<size_t> FlightSession::index_of_serial(uint32_t serial) const {
    for (size_t i = 0; i < bodies_.size(); ++i) {
        if (bodies_[i].serial == serial) return i;
    }
    return std::nullopt;
}

void FlightSession::stage() {
    if (bodies_.empty() || bodies_[controlled_].body->vehicle().stages().size() < 2) return;
    if (staging_) return;  // a sequence is under way: it separates once
    if (docking_busy()) {  // the stack is held by the docking mechanism (ISS i5)
        docking_status_ = "Stufentrennung nicht möglich: angedockt";
        return;
    }
    auto& lower = bodies_[controlled_];
    const auto& stages = lower.body->vehicle().stages();
    const auto& staging = stages.front().config.staging;
    const bool thrusting = stage_producing_thrust(stages.front());
    const bool first_cutoff = has_recorded(timeline_, MissionEventKind::Liftoff, lower.serial) &&
                              !has_recorded(timeline_, MissionEventKind::Meco, lower.serial);
    if (!staging || !thrusting) {
        // Nothing to wait for (a trunk, a payload, a stack whose lower stage is not burning): separate now - a stage
        // without a sequence under thrust cuts off in this tick (the stack lifted off with this serial: MECO).
        if (thrusting && first_cutoff) record_event(MissionEventKind::Meco, controlled_);
        controlled_ = separate_lowest_stage(controlled_, staging ? staging->separation_dv_mps : 0.0);
        ignite();
        if (autopilot_.state == AutopilotState::Booster) autopilot_.state = AutopilotState::UpperStage;
        return;
    }
    staging_ = StagingSequence{*staging, lower.serial, epoch_, std::nullopt, std::nullopt, 0};
    if (autopilot_.state == AutopilotState::Booster) autopilot_.state = AutopilotState::Staging;
    // MECO: the lowest stage's groups off, but the run-on set (hot staging)
    const std::string prefix = stages.front().config.id + "/";
    for (auto& [group, run] : lower.running_groups) {
        const bool run_on = std::any_of(staging->run_on_engine_groups.begin(), staging->run_on_engine_groups.end(),
                                        [&](const std::string& id) { return group == prefix + id; });
        run = run && run_on;
    }
    if (staging->kind == config::StagingConfig::Kind::Cold) return;  // note_engine_commands records the MECO
    // Hot: the run-on engines keep the command alive, so the MECO is recorded here ...
    if (first_cutoff) record_event(MissionEventKind::Meco, controlled_);
    // ... and the stage above lights while joined: its thrust acts on the stack. The ignition belongs to the upper part
    // as the body it becomes at the separation (the next serial; no other body is made before it).
    for (const auto& e : stages[1].engines) {
        if (!e.auxiliary()) lower.running_groups[e.group_id] = true;
    }
    auto ignition = mission_event(*this, controlled_, MissionEventKind::Ignition);
    ignition.body_serial = next_serial_;
    ignition.body_name = stages[1].config.name;
    timeline_.record(std::move(ignition));
    staging_->upper_ignition_epoch = epoch_;
}

void FlightSession::run_staging() {
    auto& seq = *staging_;
    const double half_tick_s = 0.5 * tick_s;
    const bool hot = seq.config.kind == config::StagingConfig::Kind::Hot;
    if (!seq.separation_epoch) {
        const auto stack = index_of_serial(seq.stack_serial);
        if (!stack || bodies_[*stack].body->vehicle().stages().size() < 2) {
            staging_.reset();  // the stack is gone (a new flight's bodies replaced it)
            return;
        }
        bool due = false;
        if (hot) {
            // at thrust: no engine of the stage above still starting (one that cannot start - starved, failed, no air
            // pressure limit met - does not hold the separation), at the earliest the tick after the ignition
            const auto& engines = bodies_[*stack].body->vehicle().stages()[1].engines;
            due = epoch_ > *seq.upper_ignition_epoch + half_tick_s &&
                  std::none_of(engines.begin(), engines.end(), [](const vehicle::Engine& e) {
                      return !e.auxiliary() && e.mode == vehicle::EngineMode::Starting;
                  });
        } else {
            due = !(epoch_ - seq.meco_epoch < seq.config.separation_delay_s - half_tick_s);
        }
        if (!due) return;
        const size_t upper = separate_lowest_stage(*stack, seq.config.separation_dv_mps);
        if (*stack == controlled_) controlled_ = upper;
        seq.separation_epoch = epoch_;
        seq.upper_serial = bodies_[upper].serial;
        if (!hot) return;  // cold: the upper stage lights after its delay
    } else {
        if (epoch_ - *seq.separation_epoch < seq.config.upper_ignition_delay_s - half_tick_s) return;
        const auto upper = index_of_serial(seq.upper_serial);
        if (!upper) {
            staging_.reset();
            return;
        }
        light_lowest_engines(bodies_[*upper]);
        record_event(MissionEventKind::Ignition, *upper);  // the observer sees the start too: it skips this one
        seq.upper_ignition_epoch = epoch_;
    }
    if (autopilot_.state == AutopilotState::Staging) autopilot_.state = AutopilotState::UpperStage;
    staging_.reset();
}

size_t FlightSession::separate_lowest_stage(size_t index, double dv_mps) {
    auto& lower = bodies_[index];
    const auto& stages = lower.body->vehicle().stages();
    std::set<std::string> upper_groups;  // the engine groups that leave with the upper part
    for (size_t i = 1; i < stages.size(); ++i) {
        for (const auto& e : stages[i].engines) upper_groups.insert(e.group_id);
    }
    auto upper = lower.body->separate(1, dv_mps);
    if (std::getenv("STARSIM_DEBUG_STAGING")) {  // diagnostics: the parts at the split
        const auto& l = lower.body->state();
        const auto& u = upper->state();
        const math::Vec3 dv = l.velocity_gcrf_mps - u.velocity_gcrf_mps;
        std::printf("[staging] separation T+%.2f s: lower rate %+.4f/%+.4f/%+.4f rad/s, upper rate %+.4f/%+.4f/%+.4f "
                    "rad/s, lower - upper %+.2f m/s along, %.2f m/s across\n",
                    mission_time_s(), l.angular_velocity_body_radps.x, l.angular_velocity_body_radps.y,
                    l.angular_velocity_body_radps.z, u.angular_velocity_body_radps.x, u.angular_velocity_body_radps.y,
                    u.angular_velocity_body_radps.z, dot(dv, math::normalized(u.velocity_gcrf_mps)),
                    norm(dv - math::normalized(u.velocity_gcrf_mps) * dot(dv, math::normalized(u.velocity_gcrf_mps))));
    }
    std::map<std::string, bool> upper_running;
    for (auto it = lower.running_groups.begin(); it != lower.running_groups.end();) {
        if (upper_groups.count(it->first) != 0) {
            upper_running.insert(*it);
            it = lower.running_groups.erase(it);
        } else {
            ++it;
        }
    }
    const bool returns = returns_after_staging(lower.body->vehicle());
    // A returning stage keeps its run-on engines into the flip (its guidance commands them from now on); any other
    // lower part shuts down.
    if (!returns) {
        for (auto& [group, run] : lower.running_groups) run = false;
    }
    lower.commanded = false;  // the MECO is recorded: no further cutoff event for this command
    lower.commanded_with_upper = false;
    lower.commanded_ascent_upper = false;
    record_event(MissionEventKind::StageSeparation, index);  // the lower part, before the upper is a body
    lower.body->set_ground_height(launch_site_.height_m);  // separated stages: the real ground, not the mount
    upper->set_ground_height(launch_site_.height_m);
    if (returns) {
        const auto landing = return_site(lower.body->vehicle());
        lower.body->set_ground_height(landing.height_m);
        lower.body->set_accelerometer(true);  // the return guidance estimates the drag online
        lower.return_guidance = std::make_unique<gnc::BoosterReturnGuidance>(
            landing, return_parameters(lower.body->vehicle().stages().front().config));
        lower.return_guidance->set_realtime(realtime_planning_);
    }
    const bool was_controlled = index == controlled_;
    Flying flying = new_flying(std::move(upper));
    flying.running_groups = std::move(upper_running);
    bodies_.push_back(std::move(flying));
    if (was_controlled) {
        clear_rendezvous_target();  // the target belongs to the controlled body's port
        reset_rendezvous();
    }
    return bodies_.size() - 1;
}

void FlightSession::apply_staging_throttle(size_t index, vehicle::VehicleCommand& command) const {
    if (!staging_ || staging_->separation_epoch || bodies_[index].serial != staging_->stack_serial) return;
    const std::string lower_prefix = bodies_[index].body->vehicle().stages().front().config.id + "/";
    for (auto& g : command.groups) {
        if (g.group.rfind(lower_prefix, 0) == 0) {
            g.throttle = 0.0;  // run-on engines: minimum throttle (the engines limit it)
        } else if (autopilot_.state == AutopilotState::Staging) {
            g.throttle = 1.0;  // the stage above lights at full thrust; its guidance takes over at the separation
        }
    }
}

}  // namespace starsim::sim
