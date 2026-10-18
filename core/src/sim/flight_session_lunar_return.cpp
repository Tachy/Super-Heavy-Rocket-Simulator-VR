// FlightSession: lunar l5 - the ship's return from lunar orbit (TEI, corrections, braking burn, entry or aerocapture).
//
// Design decisions:
// - Every burn is planned from the real state when its step is due (planning nodes): the TEI (and the NRHO powered
//   flyby) re-solved for the finite burn at the plan's node epoch, the MCCs at their slot, the braking burn at the
//   predicted braking-height crossing (speed and angle met even when the downrange cannot be closed). A braking burn
//   that does not converge flies the plan's vis-viva impulse (retrograde at the crossing) - never skipped silently.
// - Entry trim (Task 10): 20 s after the braking burn its execution error (~0.1 deg of entry angle per m/s; corridor
//   +-0.5 deg) is measured and trimmed as a small burn 60 s later, if it ends before the entry guidance starts.
// - Big burns (TEI, NRHO departure, powered flyby, braking) on the vacuum Raptors, the MCCs as lunar_small_burn.
// - The coast is Sun-pointed as the outbound coast (lunar_sun_coast); its timed event is the next MCC or the braking
//   burn's ignition lead.
// - After the braking burn the direct return hands over to the ship return (state Return: entry_start_epoch() at the
//   lunar_return() entry guidance, b3's flip and catch); the lunar step stays EarthEntry.
// - Aerocapture ("Aircatch"): the lunar steps fly the pass (Aerocapture: aerocapture guidance to the exit apoapsis, one
//   bank sign), plan the perigee raise from the real exit orbit (CaptureCoast, Sun-pointed; PerigeeRaise on the
//   deorbit engines), lower the apoapsis at the perigee to the circular orbit b4 starts from (CaptureCircularize: b4's
//   deorbit planner diverged on the 200 x 400 km ellipse) and hand over to b4 (plan_ship_return). A pass that stays in
//   the air, an open exit orbit or a perigee left too low end the autopilot with their cause ("Aircatch: ...").

#include "starsim/sim/flight_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <limits>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/earth_return.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"
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
    // the aimed bank sign's track offset: the re-plans (TEI re-solve, corrections) start from it
    lunar_return_->ret.target.lateral_offset_m = lunar_return_plan_->arrival.lateral_offset_m;
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
        const bool aerocapture = lunar_step_ == LunarStep::Aerocapture || lunar_step_ == LunarStep::CaptureCoast ||
                                 lunar_step_ == LunarStep::PerigeeRaise || lunar_step_ == LunarStep::RepassTrim ||
                                 lunar_step_ == LunarStep::CaptureCircularize;
        if (aerocapture) {
            flying.entry_guidance.reset();
            lunar_abort(cause.empty() ? std::string("Aircatch: Aufschlag") : "Aircatch: zerbrochen (" + cause + ")");
            return;
        }
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
            if (solved.converged) {
                tei = solved.burn;
                lunar_return_->ret.target.lateral_offset_m = solved.arrival.lateral_offset_m;
            }
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
            if (lunar_replan_burn_) {
                lunar_replan_burn_ = false;  // a re-plan's correction: the MCC slots stay as they are
            } else {
                ++lunar_mcc_index_;
            }
            lunar_return_enter_coast();
            break;
        case LunarStep::ReturnCoast:
            lunar_sun_coast(flying);
            if (epoch_ - lunar_step_epoch_ >= 0.0) lunar_return_coast_event();
            break;
        case LunarStep::Braking:
            lunar_return_after_braking();
            break;
        case LunarStep::EntryTrim:
            if (lunar_entry_trim_commanded_) {  // the trim burn is flown
                lunar_return_enter_entry();
            } else if (epoch_ - lunar_step_epoch_ >= 0.0) {
                lunar_return_entry_trim();
            }
            break;
        case LunarStep::Aerocapture: {
            if (!flying.entry_guidance) {
                if (epoch_ - lunar_step_epoch_ < 0.0) {
                    if (lunar_aerocapture_passes_ > 1) lunar_sun_coast(flying);  // the orbit to the next pass
                    break;
                }
                attitude_mode_ = {};
                // The pass from the entry lead on, as planned: one bank sign, no plane steering (the start's
                // aerocapture(apoapsis, -1)). Task 9: steering on the arrival inclination reversed nearly every cycle
                // (the target lies between the two signs' exits, 29.2 / 32.4 deg) and each roll through lift up lofted
                // the pass out to 17,800 km; b4 only needs an orbit inclined beyond the site's latitude.
                start_entry_guidance(flying, lunar_return_->entry);
                lunar_step_epoch_ = epoch_ + far_future_s;  // the guidance flies (ticked) until the exit
                break;
            }
            if (flying.entry_command.phase == gnc::EntryPhase::Exit) {
                flying.entry_guidance.reset();  // out of the atmosphere: the orbit is planned from the real state
                lunar_return_captured(flying);
            } else if (telemetry(controlled_).altitude_m < aerocapture_floor_height_m) {
                // deep in the air: the pass will not climb out again - the ship falls on (honest outcome)
                flying.entry_guidance.reset();
                lunar_abort("Aircatch: in der Atmosphäre geblieben");
            }
            break;
        }
        case LunarStep::CaptureCoast: {
            lunar_sun_coast(flying);
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            const auto& groups = flying.body->vehicle().stages().front().config.deorbit_engine_groups;
            const std::string name = to_string(lunar_capture_burn_step_);
            if (!execute_burn(lunar_capture_burn_, groups) && !lunar_small_burn(lunar_capture_burn_)) {
                lunar_abort("Aircatch: " + name + " nicht möglich");
                break;
            }
            const auto centre = orbit::propagate_coast(gravity_, controlled_state(), epoch_, lunar_capture_burn_.epoch);
            lunar_burns_.push_back({lunar_capture_burn_step_, lunar_capture_burn_,
                                    flying.body->vehicle().mass_properties().mass_kg, centre.state.position_m});
            lunar_step_ = lunar_capture_burn_step_;
            attitude_mode_ = {};
            lunar_status_ = "Aircatch: " + name + " " + german(norm(lunar_capture_burn_.delta_v_gcrf_mps), 1) + " m/s";
            break;
        }
        case LunarStep::RepassTrim:  // the perigee correction is flown: the next perigee is another pass
            lunar_return_next_pass();
            break;
        case LunarStep::PerigeeRaise:  // the burn is flown (it hands back to the lunar steps)
            lunar_return_raised(flying);
            break;
        case LunarStep::CaptureCircularize: {
            // The burn ends at its tail-off remainder: an engine still spooling down would mark b4's new burn as
            // started and fire it at once (Task 9: the deorbit burned 111 h early). Hand over once all are quiet.
            bool thrusting = false;
            for (const auto& stage : flying.body->vehicle().stages()) {
                for (const auto& e : stage.engines) thrusting = thrusting || e.producing_thrust();
            }
            if (thrusting) break;
            // b4 waits (up to days) for its deorbit pass in state Burn, uncontrolled: a ship still turning after the
            // circularisation is never coasted (ticked at 200 Hz). Damp the rates on an inertial hold first.
            if (norm(flying.body->state().angular_velocity_body_radps) > capture_settled_rate_radps) {
                if (attitude_mode_.kind == gnc::AttitudeModeKind::Free) {
                    attitude_mode_ = gnc::inertial_hold(flying.body->state().attitude);
                }
                break;
            }
            lunar_return_to_b4();
            break;
        }
        default:
            break;  // EarthEntry: the autopilot state is Return (b2 / b3)
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
        bool converged = true;
        const auto mcc = planner.mid_course(state, epoch_, epoch_ + burn_preparation_s, p, &converged);
        // The correction back onto the planned arrival failed (not converged: out of reach) or costs more than the
        // budget's allowance: re-planned from the flight data to the site's cheapest reachable pass - taken when it
        // is cheaper than flying back (this slot's burn).
        const bool expensive = mcc && norm(mcc->delta_v_gcrf_mps) > p.mcc_allowance_mps;
        if (!converged || expensive) {
            const double back_mps = mcc ? norm(mcc->delta_v_gcrf_mps) + lunar_return_plan_->braking_delta_v_mps
                                        : std::numeric_limits<double>::infinity();
            ++lunar_mcc_index_;
            const bool replanned = replan_lunar_return(back_mps);
            if (lunar_debug_) {
                std::printf("[lunar] MCC %s: %s\n", converged ? "over the allowance" : "not converged",
                            lunar_status_.c_str());
            }
            if (replanned) return;
            if (mcc) --lunar_mcc_index_;  // the correction back flies in this slot (ReturnCorrection counts it)
        }
        if (!mcc) {  // below the threshold (the next one grows) or not converged (the slot is used up above)
            if (converged) ++lunar_mcc_index_;
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

bool FlightSession::replan_lunar_return(double cheaper_than_mps) {
    if (!lunar_return_ || !lunar_return_plan_ || lunar_step_ != LunarStep::ReturnCoast || bodies_.empty() ||
        bodies_[controlled_].body->impacted()) {
        lunar_status_ = "Nachplanung: nur im Rückflug zur Erde (nach TEI, vor dem Bremsbrennen)";
        return false;
    }
    const auto& p = lunar_return_->ret;
    const auto configurations = burn_configurations(&vacuum_groups);
    if (configurations.empty()) {
        lunar_status_ = "Nachplanung: keine Vakuum-Triebwerke";
        return false;
    }
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    planner.set_finite_burn(mission::FiniteBurnModel{bodies_[controlled_].body->vehicle().mass_properties().mass_kg,
                                                     configurations.front().thrust_n,
                                                     configurations.front().mass_flow_kgps});
    const auto state = controlled_state();
    const time::Epoch burn_epoch = epoch_ + burn_preparation_s;
    const auto brake = planner.braking_epoch(state, epoch_, p.target);
    // no predicted approach (a TEI far off): the recovery is tried all the same - that is when it matters most
    if (brake && burn_epoch > *brake - p.mcc_min_before_braking_s) {
        lunar_status_ = "Nachplanung: zu nah am Bremsbrennen";
        return false;
    }
    mission::ReturnBurn r;
    try {
        r = planner.recovery(state, epoch_, burn_epoch, p.target);
    } catch (const std::exception&) {  // integrator step underflow on a trial arc
        r.converged = false;
    }
    if (!r.converged) {
        lunar_status_ = brake ? "Nachplanung: kein Weg nach Starbase"
                              : "Nachplanung: keine Annäherung an die Erde, kein Weg nach Starbase";
        return false;
    }
    // propellant: the correction and the new braking burn above the landing propellant
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double ve = configurations.front().thrust_n / configurations.front().mass_flow_kgps;
    const double dv = norm(r.burn.delta_v_gcrf_mps);
    const double needed =
        vehicle.mass_properties().mass_kg * (1.0 - std::exp(-(dv + r.arrival.braking_delta_v_mps) / ve));
    const double available = vehicle.main_propellant_kg();  // the headers hold the landing propellant
    if (needed > available) {
        lunar_status_ = "Nachplanung: Treibstoff reicht nicht (" + german((needed - available) / 1000.0, 1) + " t fehlen)";
        return false;
    }
    if (dv + r.arrival.braking_delta_v_mps >= cheaper_than_mps) {
        lunar_status_ = "Nachplanung: nicht günstiger (" + german(dv + r.arrival.braking_delta_v_mps, 0) + " m/s)";
        return false;
    }
    lunar_return_->ret.target.lateral_offset_m = r.arrival.lateral_offset_m;
    lunar_return_plan_->arrival = r.arrival;
    lunar_return_plan_->braking_delta_v_mps = r.arrival.braking_delta_v_mps;
    ++lunar_replans_;
    if (dv < p.mcc_threshold_mps) {
        lunar_status_ = "Nachplanung: keine Korrektur nötig";
        lunar_return_enter_coast();
        return true;
    }
    // on the vacuum Raptors (a re-plan may move the arrival by hours: tens to hundreds of m/s); small: lunar_small_burn
    if (!lunar_burn(r.burn, true, LunarStep::ReturnCorrection)) return false;
    lunar_replan_burn_ = true;
    lunar_step_ = LunarStep::ReturnCorrection;
    lunar_status_ = "Nachplanung: Korrektur " + german(dv, 2) + " m/s, Bremsen " +
                    german(r.arrival.braking_delta_v_mps, 0) + " m/s";
    return true;
}

double FlightSession::lunar_return_arrival_speed() const {
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    auto target = lunar_return_->ret.target;
    target.entry_model.reset();
    target.speed_mps = 0.0;  // coasting: no modelled braking
    return planner.evaluate(controlled_state(), epoch_, epoch_, math::Vec3{}, target).speed_mps;
}

void FlightSession::lunar_return_after_braking() {
    // l5 Task 10: ~0.1 deg of entry angle per m/s of the braking burn's execution error, a corridor of +-0.5 deg - the
    // error is measured after the burn and trimmed (as Apollo's and Orion's last corrections before the interface).
    lunar_step_ = LunarStep::EntryTrim;
    lunar_entry_trim_commanded_ = false;
    lunar_step_epoch_ = epoch_ + entry_trim_settle_s;
    attitude_mode_ = {};
    lunar_status_ = "Eintrittskorrektur: Navigation";
}

void FlightSession::lunar_return_entry_trim() {
    const auto& target = lunar_return_->ret.target;
    mission::EarthReturnPlanner planner(gravity_, *env_.ephemeris, earth_);
    deorbit_parameters_.entry_interface_height_m = target.interface_height_m;  // entry_start_epoch() uses it
    const auto trim_epoch = epoch_ + entry_trim_lead_s;
    const auto trim = planner.entry_trim(controlled_state(), epoch_, trim_epoch, target);
    if (!trim || entry_start_epoch() - trim_epoch < entry_trim_margin_s) {
        // none needed (below the threshold), not converged, or no time before the entry guidance: enter as braked
        lunar_return_enter_entry();
        return;
    }
    // as lunar_burn, but a trim that cannot be flown enters as braked instead of ending the flight
    if (!lunar_small_burn(*trim)) {
        lunar_return_enter_entry();
        return;
    }
    const auto centre = orbit::propagate_coast(gravity_, controlled_state(), epoch_, trim->epoch).state;
    lunar_burns_.push_back({LunarStep::EntryTrim, *trim, bodies_[controlled_].body->vehicle().mass_properties().mass_kg,
                            centre.position_m});
    lunar_entry_trim_commanded_ = true;
    lunar_status_ = "Eintrittskorrektur: " + german(norm(trim->delta_v_gcrf_mps), 1) + " m/s";
}

void FlightSession::lunar_return_enter_entry() {
    const auto& target = lunar_return_->ret.target;
    if (lunar_return_->aerocapture) {
        lunar_aerocapture_passes_ = 1;
        lunar_step_ = LunarStep::Aerocapture;
        deorbit_parameters_.entry_interface_height_m = target.interface_height_m;  // entry_start_epoch() uses it
        lunar_step_epoch_ = entry_start_epoch();  // the guidance starts at the entry lead
        attitude_mode_ = {};
        lunar_status_ = "Aircatch: Eintritt mit " + german(lunar_return_arrival_speed(), 0) + " m/s";
        return;
    }
    lunar_step_ = LunarStep::EarthEntry;
    deorbit_parameters_.entry_interface_height_m = target.interface_height_m;  // entry_start_epoch() uses it
    return_entry_parameters_ = lunar_return_->entry;                            // lunar_return(): skips carried
    ship_return_ = true;
    autopilot_.state = AutopilotState::Return;
    entry_start_ = entry_start_epoch();
    attitude_mode_ = {};
    lunar_status_ = "Wiedereintritt: " + german(lunar_return_arrival_speed(), 0) + " m/s";
}

void FlightSession::lunar_return_captured(Flying& flying) {
    const auto state = controlled_state();
    const auto e = orbit::elements_from_state(state, constants::earth_gm_m3ps2);
    if (e.semi_major_axis_m <= 0.0 || e.eccentricity >= 1.0) {
        lunar_abort("Aircatch: Bahn nicht geschlossen (Abprall)");
        return;
    }
    const double gm = constants::earth_gm_m3ps2;
    const double raised_radius_m = constants::wgs84_semi_major_axis_m + lunar_return_->perigee_raise_height_m;
    const double apoapsis_radius_m = e.semi_major_axis_m * (1.0 + e.eccentricity);
    const auto node = gnc::raise_periapsis_at_apoapsis(state, epoch_, gm, raised_radius_m);
    if (lunar_aerocapture_passes_ < aerocapture_max_passes && apoapsis_radius_m > raised_radius_m) {
        // capture cost: the raise, then the circularisation at the raised perigee (vis-viva)
        const double v_perigee = std::sqrt(gm * (2.0 / raised_radius_m - 2.0 / (raised_radius_m + apoapsis_radius_m)));
        const double cost_mps = (node ? norm(node->delta_v_gcrf_mps) : 0.0) + v_perigee - std::sqrt(gm / raised_radius_m);
        if (cost_mps > aerocapture_repass_dv_mps) {
            lunar_return_repass(flying, apoapsis_radius_m - constants::wgs84_semi_major_axis_m);
            return;
        }
    }
    if (!node) {  // the perigee is already high enough
        lunar_return_raised(flying);
        return;
    }
    if (!lunar_capture_coast(flying, *node, LunarStep::PerigeeRaise)) return;
    const double apoapsis_m = e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m;
    lunar_status_ = "Aircatch: Apogäum " + german(apoapsis_m / 1000.0, 0) + " km, Perigäumsanhebung " +
                    german(norm(node->delta_v_gcrf_mps), 1) + " m/s in " + german((node->epoch - epoch_) / 60.0, 0) +
                    " min";
}

bool FlightSession::lunar_capture_coast(Flying& flying, const gnc::BurnNode& node, LunarStep burn) {
    const auto configurations = burn_configurations(&flying.body->vehicle().stages().front().config.deorbit_engine_groups);
    const double mass_kg = flying.body->vehicle().mass_properties().mass_kg;
    const double dv_mps = norm(node.delta_v_gcrf_mps);
    double half_burn_s = 0.0;
    if (!configurations.empty()) {
        const auto& c = configurations.front();
        half_burn_s = 0.5 * gnc::burn_duration_s(dv_mps, mass_kg, c.thrust_n, c.mass_flow_kgps);
        // l5 Task 10: a pass that exits far too high (campaign seed 2: apogee 9,344 km) asked for a 1,458 m/s
        // circularisation with 51 t on board - the burn never ended. Refuse a burn the tanks cannot fly.
        const double needed_kg = mass_kg * (1.0 - std::exp(-dv_mps * c.mass_flow_kgps / c.thrust_n));
        if (needed_kg > flying.body->vehicle().propellant_kg()) {
            lunar_abort("Aircatch: " + std::string(to_string(burn)) + " nicht möglich (" + german(dv_mps, 0) +
                        " m/s, " + german(needed_kg / 1000.0, 1) + " t nötig, " +
                        german(flying.body->vehicle().propellant_kg() / 1000.0, 1) + " t an Bord)");
            return false;
        }
    }
    lunar_capture_burn_ = node;
    lunar_capture_burn_step_ = burn;
    lunar_step_ = LunarStep::CaptureCoast;
    lunar_step_epoch_ = node.epoch - half_burn_s - burn_preparation_s;
    attitude_mode_ = {gnc::AttitudeModeKind::Sun, {}, {}, {}};
    lunar_sun_hold_.start(epoch_);
    lunar_next_repoint_ = epoch_ + sun_repoint_interval_s;
    return true;
}

void FlightSession::lunar_return_repass(Flying& flying, double apoapsis_m) {
    const auto node = gnc::set_periapsis_at_apoapsis(
        controlled_state(), epoch_, constants::earth_gm_m3ps2,
        constants::wgs84_semi_major_axis_m + aerocapture_repass_perigee_height_m);
    if (!node || norm(node->delta_v_gcrf_mps) < aerocapture_repass_trim_min_mps) {
        lunar_return_next_pass();
        return;
    }
    if (!lunar_capture_coast(flying, *node, LunarStep::RepassTrim)) return;
    lunar_status_ = "Aircatch: Apogäum " + german(apoapsis_m / 1000.0, 0) + " km, Durchgang " +
                    std::to_string(lunar_aerocapture_passes_ + 1) + " - Perigäumskorrektur " +
                    german(norm(node->delta_v_gcrf_mps), 1) + " m/s in " + german((node->epoch - epoch_) / 60.0, 0) +
                    " min";
}

void FlightSession::lunar_return_next_pass() {
    ++lunar_aerocapture_passes_;
    const auto e = orbit::elements_from_state(controlled_state(), constants::earth_gm_m3ps2);
    const double period_s = constants::two_pi * std::sqrt(std::pow(e.semi_major_axis_m, 3) / constants::earth_gm_m3ps2);
    lunar_step_ = LunarStep::Aerocapture;
    lunar_step_epoch_ = entry_start_epoch(1.1 * period_s);  // the guidance starts at the entry lead of the next perigee
    attitude_mode_ = {gnc::AttitudeModeKind::Sun, {}, {}, {}};  // Sun-pointed as the capture coast
    lunar_sun_hold_.start(epoch_);
    lunar_next_repoint_ = epoch_ + sun_repoint_interval_s;
    lunar_status_ = "Aircatch: Durchgang " + std::to_string(lunar_aerocapture_passes_) + " in " +
                    german((lunar_step_epoch_ - epoch_) / 3600.0, 1) + " h";
}

void FlightSession::lunar_return_raised(Flying& flying) {
    const auto state = controlled_state();
    const auto e = orbit::elements_from_state(state, constants::earth_gm_m3ps2);
    const double periapsis_m = e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m;
    const double apoapsis_m = e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m;
    if (e.semi_major_axis_m <= 0.0 || e.eccentricity >= 1.0 || periapsis_m < capture_min_apsis_height_m) {
        lunar_abort("Aircatch: Perigäum zu tief");
        return;
    }
    const auto node = gnc::circularize_at_periapsis(state, epoch_, constants::earth_gm_m3ps2);
    if (!node || apoapsis_m - periapsis_m < capture_circular_tolerance_m) {
        lunar_return_to_b4();
        return;
    }
    if (!lunar_capture_coast(flying, *node, LunarStep::CaptureCircularize)) return;
    lunar_status_ = "Aircatch: Perigäum " + german(periapsis_m / 1000.0, 0) + " km, Zirkularisierung " +
                    german(norm(node->delta_v_gcrf_mps), 1) + " m/s in " + german((node->epoch - epoch_) / 60.0, 0) +
                    " min";
}

void FlightSession::lunar_return_to_b4() {
    const auto e = orbit::elements_from_state(controlled_state(), constants::earth_gm_m3ps2);
    const double periapsis_m = e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::wgs84_semi_major_axis_m;
    if (e.semi_major_axis_m <= 0.0 || e.eccentricity >= 1.0 || periapsis_m < capture_min_apsis_height_m) {
        lunar_abort("Aircatch: Perigäum zu tief");
        return;
    }
    const auto deorbit = lunar_return_->deorbit;
    lunar_abort("Rückkehr nach Starbase (b4)");  // the lunar flight ends here (autopilot Off): b4 takes over
    if (!plan_ship_return(deorbit)) {
        lunar_status_ = deorbit_plan_ ? deorbit_plan_->reason : std::string("Keine Rückkehr möglich");
    }
}

}  // namespace starsim::sim
