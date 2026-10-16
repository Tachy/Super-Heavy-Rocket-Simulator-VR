// FlightSession: lunar l5 - the ship's return from lunar orbit (TEI, corrections, braking burn, entry or aerocapture).
//
// Design decisions:
// - Every burn is planned from the real state when its step is due (planning nodes): the TEI (and the NRHO powered
//   flyby) re-solved for the finite burn at the plan's node epoch, the MCCs at their slot, the braking burn at the
//   predicted braking-height crossing. A braking burn that does not converge flies the plan's vis-viva impulse
//   (retrograde at the crossing) - never skipped silently.
// - Big burns (TEI, NRHO departure, powered flyby, braking) on the vacuum Raptors, the MCCs as lunar_small_burn.
// - The coast is Sun-pointed as the outbound coast (lunar_sun_coast); its timed event is the next MCC or the braking
//   burn's ignition lead.
// - After the braking burn the direct return hands over to the ship return (state Return: entry_start_epoch() at the
//   lunar_return() entry guidance, b3's flip and catch); the lunar step stays EarthEntry.

#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <exception>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/earth_return.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "flight_session_lunar_detail.hpp"

namespace starsim::sim {

using detail::far_future_s;
using detail::german;
using detail::vacuum_groups;

bool FlightSession::plan_lunar_return(const LunarReturnSetup& setup) {
    if (bodies_.empty() || lunar_step_ != LunarStep::None ||
        (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit)) {
        return false;  // a flight under way keeps its plan
    }
    if (!moon_orientation_) {
        lunar_status_ = "Kein Rückflug: keine Mondorientierung";
        return false;
    }
    if (reference_body(controlled_) != ephemeris::Body::Moon) {
        lunar_status_ = "Kein Rückflug: nicht beim Mond";
        return false;
    }
    const auto configurations = burn_configurations(&vacuum_groups);
    if (configurations.empty()) {
        lunar_status_ = "Kein Rückflug: keine Vakuum-Triebwerke";
        return false;
    }
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double isp_s = configurations.front().thrust_n / configurations.front().mass_flow_kgps /
                         constants::standard_gravity_mps2;
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    planner.set_finite_burn(mission::FiniteBurnModel{vehicle.mass_properties().mass_kg, configurations.front().thrust_n,
                                                     configurations.front().mass_flow_kgps});
    auto planned = setup;
    planned.ret.earliest_departure_s = std::max(planned.ret.earliest_departure_s, min_departure_lead_s);
    try {
        lunar_return_plan_ = planner.plan(controlled_state(), epoch_, planned.ret, vehicle.mass_properties().mass_kg,
                                          isp_s, vehicle.propellant_kg());
    } catch (const std::exception&) {  // integrator step underflow on a trial arc, ephemeris range
        lunar_return_plan_.reset();
        lunar_status_ = "Kein Rückflug: Planung fehlgeschlagen";
        return false;
    }
    if (!lunar_return_plan_->feasible) {
        lunar_status_ = lunar_return_plan_->reason;
        return false;
    }
    lunar_return_ = planned;
    lunar_return_departed_ = !lunar_return_plan_->nrho_departure;
    lunar_return_braking_epoch_.reset();
    lunar_mcc_index_ = 0;
    lunar_burns_.clear();
    lunar_step_ = LunarStep::ReturnDeparture;
    lunar_step_epoch_ = lunar_ignition_lead(lunar_return_departed_ ? lunar_return_plan_->tei
                                                                   : *lunar_return_plan_->nrho_departure);
    autopilot_.state = AutopilotState::Lunar;
    attitude_mode_ = {};
    lunar_status_ = "Rückflug: TEI " + german(lunar_return_plan_->tei_delta_v_mps, 0) + " m/s, Bremsen " +
                    german(lunar_return_plan_->braking_delta_v_mps, 0) + " m/s geplant";
    return true;
}

void FlightSession::run_lunar_return_steps(Flying& flying) {
    if (flying.body->impacted()) {
        const auto& cause = flying.body->breakup_reason();
        lunar_abort("Rückflug beendet: " + (cause.empty() ? std::string("Aufschlag") : cause));
        return;
    }
    switch (lunar_step_) {
        case LunarStep::ReturnDeparture: {
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            const auto configurations = burn_configurations(&vacuum_groups);
            if (configurations.empty()) {  // engines lost since the planning
                lunar_abort("Rückflug abgebrochen: keine Vakuum-Triebwerke");
                break;
            }
            if (!lunar_return_departed_) {  // NRHO: the departure burn down to the return flyby, as planned
                const auto departure = *lunar_return_plan_->nrho_departure;
                if (lunar_burn(departure, true, LunarStep::ReturnDeparture)) {
                    lunar_return_departed_ = true;
                    lunar_step_epoch_ = lunar_ignition_lead(lunar_return_plan_->tei);
                    lunar_status_ = "Abflug aus der NRHO: " + german(norm(departure.delta_v_gcrf_mps), 0) + " m/s";
                }
                break;
            }
            // The plan's TEI (NRHO: the powered flyby) re-solved for the finite burn from the state now at its node
            // epoch (as the TLI); not converged: the plan's burn.
            mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
            planner.set_finite_burn(mission::FiniteBurnModel{flying.body->vehicle().mass_properties().mass_kg,
                                                             configurations.front().thrust_n,
                                                             configurations.front().mass_flow_kgps});
            auto tei = lunar_return_plan_->tei;
            const auto solved = planner.solve(controlled_state(), epoch_, tei.epoch, lunar_return_->ret.target,
                                              tei.delta_v_gcrf_mps);
            if (solved.converged) tei = solved.burn;
            if (lunar_burn(tei, true, LunarStep::Tei)) {
                lunar_step_ = LunarStep::Tei;
                lunar_status_ = "TEI: " + german(norm(tei.delta_v_gcrf_mps), 0) + " m/s";
            }
            break;
        }
        case LunarStep::Tei:
            lunar_tli_end_ = epoch_;  // the MCC clock (shared with the outbound flight)
            lunar_return_enter_coast();
            break;
        case LunarStep::ReturnCorrection:
            ++lunar_mcc_index_;
            lunar_return_enter_coast();
            break;
        case LunarStep::ReturnCoast:
            lunar_sun_coast(flying);
            if (epoch_ - lunar_step_epoch_ >= 0.0) lunar_return_coast_event();
            break;
        case LunarStep::Braking:
            lunar_return_after_braking();
            break;
        default:
            break;  // EarthEntry: the autopilot state is Return (b2 / b3); Task 9: the aerocapture steps
    }
}

void FlightSession::lunar_return_enter_coast() {
    const auto& p = lunar_return_->ret;
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    lunar_return_braking_epoch_ = planner.braking_epoch(controlled_state(), epoch_, p.target);
    lunar_step_ = LunarStep::ReturnCoast;
    attitude_mode_ = {gnc::AttitudeModeKind::Sun, {}, {}, {}};
    lunar_sun_hold_.start(epoch_);
    lunar_next_repoint_ = epoch_ + sun_repoint_interval_s;
    if (!lunar_return_braking_epoch_) {  // no Earth approach within 6 days: an MCC can still bring it back
        lunar_status_ = "Rückflug: keine Annäherung an die Erde";
    }
    while (lunar_mcc_index_ < p.mcc_after_tei_s.size()) {
        const time::Epoch mcc = lunar_tli_end_ + p.mcc_after_tei_s[lunar_mcc_index_];
        const bool before_braking =
            !lunar_return_braking_epoch_ || *lunar_return_braking_epoch_ - mcc >= p.mcc_min_before_braking_s;
        if (before_braking && mcc - epoch_ > 0.0) {
            lunar_step_epoch_ = mcc - burn_preparation_s;
            lunar_status_ = "Rückflug: Kurskorrektur " + std::to_string(lunar_mcc_index_ + 1) + " in " +
                            german((mcc - epoch_) / 3600.0, 1) + " h";
            return;
        }
        ++lunar_mcc_index_;  // too close to the braking burn (or already past)
    }
    if (!lunar_return_braking_epoch_) {
        lunar_abort("Rückflug abgebrochen: keine Annäherung an die Erde");
        return;
    }
    const auto configurations = burn_configurations(&vacuum_groups);
    double half_burn_s = 0.0;
    if (!configurations.empty()) {
        half_burn_s = 0.5 * gnc::burn_duration_s(lunar_return_plan_->braking_delta_v_mps,
                                                 bodies_[controlled_].body->vehicle().mass_properties().mass_kg,
                                                 configurations.front().thrust_n, configurations.front().mass_flow_kgps);
    }
    lunar_step_epoch_ = *lunar_return_braking_epoch_ - half_burn_s - burn_preparation_s;
    lunar_status_ = "Rückflug: Bremsbrennen in " + german((*lunar_return_braking_epoch_ - epoch_) / 3600.0, 1) + " h";
}

void FlightSession::lunar_return_coast_event() {
    const auto& p = lunar_return_->ret;
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    const auto state = controlled_state();
    if (lunar_mcc_index_ < p.mcc_after_tei_s.size()) {
        // MCC slot: planned burn_preparation_s ahead (an MCC is a short burn, the preparation covers it)
        const auto mcc = planner.mid_course(state, epoch_, epoch_ + burn_preparation_s, p);
        if (!mcc) {  // below the threshold (or not converged): the next one grows
            ++lunar_mcc_index_;
            lunar_return_enter_coast();
            return;
        }
        if (lunar_burn(*mcc, false, LunarStep::ReturnCorrection)) {
            lunar_step_ = LunarStep::ReturnCorrection;
            lunar_status_ = "Kurskorrektur " + std::to_string(lunar_mcc_index_ + 1) + ": " +
                            german(norm(mcc->delta_v_gcrf_mps), 2) + " m/s";
        }
        return;
    }
    const auto configurations = burn_configurations(&vacuum_groups);
    if (configurations.empty()) {
        lunar_abort("Rückflug abgebrochen: keine Vakuum-Triebwerke");
        return;
    }
    planner.set_finite_burn(mission::FiniteBurnModel{bodies_[controlled_].body->vehicle().mass_properties().mass_kg,
                                                     configurations.front().thrust_n,
                                                     configurations.front().mass_flow_kgps});
    auto braking = planner.braking(state, epoch_, p.target);
    std::string status = "Bremsbrennen: ";
    if (!braking) {
        // the plan's vis-viva impulse, retrograde at the predicted crossing
        const auto crossing = lunar_return_braking_epoch_ ? *lunar_return_braking_epoch_ : epoch_ + burn_preparation_s;
        const auto at = orbit::propagate_coast(gravity_, state, epoch_, crossing).state;
        braking = gnc::BurnNode{crossing,
                                -math::normalized(at.velocity_mps) * lunar_return_plan_->braking_delta_v_mps};
        status = "Bremsbrennen ungelenkt: ";
    }
    if (lunar_burn(*braking, true, LunarStep::Braking)) {
        lunar_step_ = LunarStep::Braking;
        lunar_status_ = status + german(norm(braking->delta_v_gcrf_mps), 0) + " m/s";
    }
}

double FlightSession::lunar_return_arrival_speed() const {
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    auto target = lunar_return_->ret.target;
    target.entry_model.reset();
    target.speed_mps = 0.0;  // coasting: no modelled braking
    return planner.evaluate(controlled_state(), epoch_, epoch_, math::Vec3{}, target).speed_mps;
}

void FlightSession::lunar_return_after_braking() {
    if (lunar_return_->aerocapture) {
        lunar_abort("Rückflug abgebrochen: Aircatch-Autopilot fehlt");  // Task 9
        return;
    }
    const auto& target = lunar_return_->ret.target;
    lunar_step_ = LunarStep::EarthEntry;
    deorbit_parameters_.entry_interface_height_m = target.interface_height_m;  // entry_start_epoch() uses it
    return_entry_parameters_ = lunar_return_->entry;                            // lunar_return(): skips carried
    ship_return_ = true;
    autopilot_.state = AutopilotState::Return;
    entry_start_ = entry_start_epoch();
    attitude_mode_ = {};
    lunar_status_ = "Wiedereintritt: " + german(lunar_return_arrival_speed(), 0) + " m/s";
}

}  // namespace starsim::sim
