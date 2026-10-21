// FlightSession: lunar coast - Sun hold, mid-course corrections, pilot-flown corrections.

#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "flight_session_lunar_detail.hpp"

namespace starsim::sim {

using detail::far_future_s;
using detail::german;

time::Epoch FlightSession::lunar_next_mcc_epoch() const {
    const auto& p = *lunar_parameters_;
    const auto& plan = *lunar_plan_;
    // MCCs before the next big burn only (NRHO: the swing-by; then the insertion)
    const time::Epoch big = plan.powered_flyby && !lunar_flyby_done_ ? plan.powered_flyby->epoch : plan.insertion.epoch;
    if (lunar_mcc_index_ < p.mcc_after_tli_s.size()) {
        const time::Epoch t = lunar_tli_end_ + p.mcc_after_tli_s[lunar_mcc_index_];
        const double lead = p.pilot_corrections ? p.pilot_call_lead_s : 0.0;
        if (big - t > burn_lead_s() + lead) return t;
    }
    return epoch_ + far_future_s;
}

void FlightSession::lunar_enter_coast() {
    const auto& p = *lunar_parameters_;
    const auto& plan = *lunar_plan_;
    lunar_step_ = LunarStep::Coast;
    attitude_mode_ = {gnc::AttitudeModeKind::Sun, {}, {}, {}};
    lunar_sun_hold_.start(epoch_);
    lunar_next_repoint_ = epoch_ + sun_repoint_interval_s;
    const time::Epoch mcc = lunar_next_mcc_epoch();
    if (mcc - epoch_ < 0.5 * far_future_s) {
        lunar_step_epoch_ = mcc - (p.pilot_corrections ? p.pilot_call_lead_s : 0.0);
        lunar_status_ = "Freiflug: Kurskorrektur " + std::to_string(lunar_mcc_index_ + 1) + " in " +
                        german((mcc - epoch_) / 3600.0, 1) + " h";
    } else if (plan.powered_flyby && !lunar_flyby_done_) {
        lunar_step_epoch_ = lunar_ignition_lead(*plan.powered_flyby);
        lunar_status_ = "Freiflug: Swing-by in " + german((plan.powered_flyby->epoch - epoch_) / 3600.0, 1) + " h";
    } else {
        // LLO: the MCCs target the B-plane with the arrival time free - the LOI is timed on the periselene predicted
        // from the state now (the plan's epoch can be minutes off). NRHO: the insertion epoch is fixed by the plan.
        auto insertion = plan.insertion;
        if (p.orbit == mission::LunarOrbitKind::Llo) {
            const mission::LunarOutboundPlanner planner(gravity_, *env_.ephemeris, *moon_orientation_);
            if (const auto predicted = planner.insertion(controlled_state(), epoch_, p, plan)) insertion = *predicted;
        }
        lunar_step_epoch_ = lunar_ignition_lead(insertion);
        lunar_status_ = "Freiflug: Einschuss in " + german((insertion.epoch - epoch_) / 3600.0, 1) + " h";
    }
}

void FlightSession::lunar_coast_event() {
    const auto& p = *lunar_parameters_;
    const auto& plan = *lunar_plan_;
    const mission::LunarOutboundPlanner planner(gravity_, *env_.ephemeris, *moon_orientation_);
    const auto state = controlled_state();
    if (lunar_next_mcc_epoch() - epoch_ < 0.5 * far_future_s) {
        // Planned burn_preparation_s ahead without small_burn_margin_s: an MCC is < 240 s of burn, the preparation
        // covers it.
        const double lead = p.pilot_corrections ? p.pilot_call_lead_s : burn_preparation_s;
        const auto mcc = planner.mid_course(state, epoch_, epoch_ + lead, p, plan);
        if (!mcc) {  // below the threshold (or not converged): the next one grows
            ++lunar_mcc_index_;
            lunar_enter_coast();
            return;
        }
        if (p.pilot_corrections) {
            // Step 3b: the crew flies it; the computer announces it and gives the cue.
            lunar_pilot_ = LunarPilotCall{*mcc, mcc->epoch + p.pilot_window_s, state, epoch_};
            lunar_burns_.push_back({LunarStep::PilotCorrection, *mcc,
                                    bodies_[controlled_].body->vehicle().mass_properties().mass_kg});
            lunar_step_ = LunarStep::PilotCorrection;
            attitude_mode_ = {};
            lunar_status_ = "Kurskorrektur in " + german(p.pilot_call_lead_s / 60.0, 0) + " min: " +
                            german(norm(mcc->delta_v_gcrf_mps), 1) + " m/s";
            return;
        }
        if (lunar_burn(*mcc, false, LunarStep::MidCourse)) {
            lunar_step_ = LunarStep::MidCourse;
            lunar_status_ = "Kurskorrektur " + std::to_string(lunar_mcc_index_ + 1) + ": " +
                            german(norm(mcc->delta_v_gcrf_mps), 2) + " m/s";
        }
        return;
    }
    if (plan.powered_flyby && !lunar_flyby_done_) {
        const auto swing = planner.powered_flyby(state, epoch_, p, plan);
        if (!swing) {
            lunar_abort("Mondflug abgebrochen: Swing-by nicht planbar");
            return;
        }
        if (lunar_burn(*swing, true, LunarStep::PoweredFlyby)) {
            lunar_step_ = LunarStep::PoweredFlyby;
            lunar_status_ = "Swing-by: " + german(norm(swing->delta_v_gcrf_mps), 0) + " m/s";
        }
        return;
    }
    const auto insertion = planner.insertion(state, epoch_, p, plan);
    if (!insertion) {
        lunar_abort("Mondflug abgebrochen: Einschuss nicht planbar");
        return;
    }
    if (lunar_burn(*insertion, true, LunarStep::Insertion)) {
        lunar_step_ = LunarStep::Insertion;
        lunar_status_ = "Einschuss: " + german(norm(insertion->delta_v_gcrf_mps), 0) + " m/s";
    }
}

void FlightSession::lunar_sun_coast(Flying& flying) {
    if (attitude_mode_.kind == gnc::AttitudeModeKind::Sun && lunar_sun_hold_.active()) {
        const auto& s = flying.body->state();
        const math::Vec3 sun =
            env_.ephemeris->state_gcrf(ephemeris::Body::Sun, epoch_).position_m - s.position_gcrf_m;
        const double error = std::acos(
            std::clamp(dot(s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0}), math::normalized(sun)), -1.0, 1.0));
        const auto held = lunar_sun_hold_.update(error, norm(s.angular_velocity_body_radps), epoch_);
        if (held != gnc::SunHoldResult::Holding) {
            attitude_mode_ = {};  // pointed (or stuck: coast freely instead of spending cold gas)
            if (lunar_debug_) {
                std::printf("[lunar] Sun %s: %.2f deg, %.4f deg/s, %.0f kg\n",
                            held == gnc::SunHoldResult::Settled ? "pointed" : "hold timed out",
                            error / constants::deg_to_rad,
                            norm(s.angular_velocity_body_radps) / constants::deg_to_rad,
                            flying.body->vehicle().mass_properties().mass_kg);
            }
        }
    }
    if (epoch_ - lunar_next_repoint_ >= 0.0) {
        attitude_mode_ = {gnc::AttitudeModeKind::Sun, {}, {}, {}};
        lunar_sun_hold_.start(epoch_);
        lunar_next_repoint_ = epoch_ + sun_repoint_interval_s;
        if (lunar_debug_) {
            const auto& s = flying.body->state();
            const math::Vec3 sun =
                env_.ephemeris->state_gcrf(ephemeris::Body::Sun, epoch_).position_m - s.position_gcrf_m;
            std::printf("[lunar] Sun re-point: %.2f deg off, %.4f deg/s, %.0f kg\n",
                        std::acos(std::clamp(dot(s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0}),
                                             math::normalized(sun)), -1.0, 1.0)) / constants::deg_to_rad,
                        norm(s.angular_velocity_body_radps) / constants::deg_to_rad,
                        flying.body->vehicle().mass_properties().mass_kg);
        }
    }
}

std::optional<PilotBurnCue> FlightSession::lunar_pilot_cue() const {
    if (!lunar_pilot_ || lunar_step_ != LunarStep::PilotCorrection || bodies_.empty()) return std::nullopt;
    const auto& call = *lunar_pilot_;
    // at most one tick from the reference kept by run_lunar
    const auto reference = orbit::propagate_coast(gravity_, call.reference, call.reference_epoch, epoch_).state;
    const auto& s = bodies_[controlled_].body->state();
    PilotBurnCue cue;
    cue.planned = call.planned;
    cue.remaining_delta_v_gcrf_mps = call.planned.delta_v_gcrf_mps - (s.velocity_gcrf_mps - reference.velocity_mps);
    const double remaining = norm(cue.remaining_delta_v_gcrf_mps);
    cue.attitude_error_rad =
        remaining > 0.0 ? std::acos(std::clamp(dot(s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0}),
                                                    cue.remaining_delta_v_gcrf_mps / remaining),
                                               -1.0, 1.0))
                        : 0.0;
    cue.window_end_s = call.window_end - epoch_;
    return cue;
}

void FlightSession::complete_pilot_correction() {
    if (lunar_step_ != LunarStep::PilotCorrection) return;
    lunar_pilot_.reset();
    ++lunar_mcc_index_;
    lunar_enter_coast();  // the next correction (or the insertion) is planned from the real state
}

}  // namespace starsim::sim
