// Lunar flight of the controlled vehicle (lunar l3): the outbound autopilot (TLI, mid-course corrections, powered
// swing-by, LOI / NRHO insertion, LLO trims, NRHO station keeping) and the transfers between the LLO and the NRHO.
//
// Design decisions:
// - The vehicle coasts Sun-pointed: at the start of each coast it turns to AttitudeModeKind::Sun (ticked until the error
//   is below 1 deg and the rate below 0.005 deg/s), then coasts freely; every sun_repoint_interval_s it re-aims (the Sun
//   moves ~1 deg/day; no disturbance torques are modelled, so the attitude only drifts with the hold's residual rate).
//   Ticking a 4-day coast at 200 Hz would cost ~70 M ticks.
// - Burns: TLI, swing-by, insertion and transfer burns on the vacuum Raptors; MCCs, trims and station keeping by
//   lunar_small_burn (the single sea-level Raptor when its start/shutdown impulse fits, else the session's choice -
//   the RCS for tiny ones). A main-engine burn too small for the vacuum set falls back to lunar_small_burn as well.
// - After each burn the autopilot returns to the state Lunar (as a ship return returns to Return); the steps Tli,
//   MidCourse, PoweredFlyby, Insertion and StationKeeping mean "that burn is under way" and are resolved on return.
// - The coast allowance stops at the next step epoch (lunar_step_epoch_); the ignition preparation comes from Burn.
// - disengage_autopilot() ends the lunar flight (step None).

#include "starsim/sim/flight_session.hpp"

#include <exception>
#include <algorithm>
#include <cstdio>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"
#include "flight_session_lunar_detail.hpp"

namespace starsim::sim {

using detail::far_future_s;
using detail::vacuum_groups;
using detail::first_after;
using detail::german;

const char* to_string(LunarStep step) {
    switch (step) {
        case LunarStep::None: return "aus";
        case LunarStep::Departure: return "Abflug";
        case LunarStep::Tli: return "TLI";
        case LunarStep::Coast: return "Freiflug";
        case LunarStep::MidCourse: return "Kurskorrektur";
        case LunarStep::PilotCorrection: return "Kurskorrektur (Pilot)";
        case LunarStep::PoweredFlyby: return "Swing-by";
        case LunarStep::Insertion: return "Einschuss";
        case LunarStep::Trim: return "Bahnkorrektur";
        case LunarStep::InLlo: return "in LLO";
        case LunarStep::InNrho: return "in NRHO";
        case LunarStep::StationKeeping: return "Bahnhaltung";
        case LunarStep::TransferOut: return "Transfer";
        case LunarStep::TransferIn: return "Ankunft";
        case LunarStep::DescentWait: return "Warten auf Überflug";
        case LunarStep::PlaneTrim: return "Bahnebenenkorrektur";
        case LunarStep::Doi: return "Abstiegsbahn (DOI)";
        case LunarStep::DescentCoast: return "Freiflug zum Abstieg";
        case LunarStep::PoweredDescent: return "Landeanflug";
        case LunarStep::Landed: return "gelandet";
        case LunarStep::SurfaceStay: return "Oberflächenaufenthalt";
        case LunarStep::Ascent: return "Aufstieg";
        case LunarStep::AscentCoast: return "Freiflug zur Aposelene";
        case LunarStep::Circularize: return "Zirkularisierung";
        case LunarStep::AbortAscent: return "Abbruch zum Orbit";
        case LunarStep::Lost: return "verloren";
        case LunarStep::ReturnDeparture: return "Abflug zur Erde";
        case LunarStep::Tei: return "TEI";
        case LunarStep::ReturnCoast: return "Rückflug";
        case LunarStep::ReturnCorrection: return "Kurskorrektur";
        case LunarStep::Braking: return "Bremsbrennen";
        case LunarStep::EarthEntry: return "Wiedereintritt";
        case LunarStep::Aerocapture: return "Aircatch";
        case LunarStep::CaptureCoast: return "Freiflug zum Apogäum";
        case LunarStep::PerigeeRaise: return "Perigäumsanhebung";
        case LunarStep::CaptureCircularize: return "Zirkularisierung";
        case LunarStep::EntryTrim: return "Eintrittskorrektur";
        case LunarStep::RepassTrim: return "Perigäumskorrektur";
    }
    return "";
}

math::StateVector FlightSession::controlled_state() const {
    const auto& s = bodies_[controlled_].body->state();
    return {s.position_gcrf_m, s.velocity_gcrf_mps};
}

void FlightSession::reset_lunar() {
    lunar_step_ = LunarStep::None;
    lunar_parameters_.reset();
    lunar_plan_.reset();
    lunar_transfer_.reset();
    lunar_nrho_.reset();
    lunar_pilot_.reset();
    lunar_mcc_index_ = 0;
    lunar_flyby_done_ = false;
    lunar_trims_ = 0;
    lunar_status_.clear();
    lunar_burns_.clear();
    lunar_landing_.reset();
    lunar_touchdown_.reset();
    lunar_descent_plan_.reset();
    lunar_landing_setup_.reset();
    lunar_ascent_.reset();
    lunar_ascent_parameters_.reset();
    lunar_mission_.reset();
    lunar_mission_leg_ = LunarMissionLeg::None;
    lunar_return_.reset();
    lunar_return_plan_.reset();
    lunar_return_departed_ = false;
    lunar_return_braking_epoch_.reset();
    return_entry_parameters_.reset();
}

void FlightSession::set_lunar_mission(std::optional<LunarMissionSequence> sequence) {
    lunar_mission_ = std::move(sequence);
    lunar_mission_leg_ = lunar_mission_ ? LunarMissionLeg::Outbound : LunarMissionLeg::None;
}

void FlightSession::lunar_mission_next() {
    const auto& m = *lunar_mission_;
    const auto end = [&](LunarMissionLeg leg, const std::string& prefix) {
        lunar_mission_leg_ = leg;
        lunar_status_ = prefix + lunar_status_;
    };
    const bool in_llo = lunar_step_ == LunarStep::InLlo;
    switch (lunar_mission_leg_) {
        case LunarMissionLeg::Outbound:
            if (!in_llo) {  // NRHO variant: down to the LLO first
                if (!plan_lunar_orbit_transfer(mission::LunarOrbitKind::Llo, m.nrho, m.llo)) {
                    end(LunarMissionLeg::Ended, "Mission beendet: ");
                }
            } else if (plan_lunar_landing(m.landing)) {
                lunar_mission_leg_ = LunarMissionLeg::Surface;
            } else {
                end(LunarMissionLeg::Ended, "Mission beendet: ");
            }
            break;
        case LunarMissionLeg::Surface:  // back in the LLO after the ascent (or an abort to orbit)
            if (!lunar_touchdown_ || lunar_touchdown_->outcome != LegTouchdown::Landed) {
                end(LunarMissionLeg::Ended, "Mission abgebrochen: ");
            } else if (!m.via_nrho) {
                end(LunarMissionLeg::Complete, "Mission abgeschlossen: ");
            } else if (plan_lunar_orbit_transfer(mission::LunarOrbitKind::Nrho, m.nrho, m.llo)) {
                lunar_mission_leg_ = LunarMissionLeg::Return;
            } else {
                end(LunarMissionLeg::Ended, "Mission beendet: ");
            }
            break;
        case LunarMissionLeg::Return:
            if (!in_llo) end(LunarMissionLeg::Complete, "Mission abgeschlossen: ");
            break;
        case LunarMissionLeg::None:
        case LunarMissionLeg::Complete:
        case LunarMissionLeg::Ended:
            break;
    }
}

bool FlightSession::plan_lunar_outbound(const mission::LunarOutboundParameters& parameters) {
    if (bodies_.empty() || lunar_step_ != LunarStep::None ||
        (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit)) {
        return false;  // a flight under way keeps its plan
    }
    if (!moon_orientation_) {
        lunar_status_ = "Kein Mondflug: keine Mondorientierung";
        return false;
    }
    if (reference_body(controlled_) == ephemeris::Body::Moon) {
        lunar_status_ = "Kein Mondflug: schon beim Mond";
        return false;
    }
    auto planned = parameters;  // the TLI needs its preparation and half its burn before the centre
    planned.earliest_departure_s = std::max(planned.earliest_departure_s, min_departure_lead_s);
    const auto configurations = burn_configurations(&vacuum_groups);
    if (configurations.empty()) {
        lunar_status_ = "Kein Mondflug: keine Vakuum-Triebwerke";
        return false;
    }
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double isp_s =
        configurations.front().thrust_n / configurations.front().mass_flow_kgps / constants::standard_gravity_mps2;
    const mission::LunarOutboundPlanner planner(gravity_, *env_.ephemeris, *moon_orientation_);
    try {
        lunar_plan_ = planner.plan(controlled_state(), epoch_, planned, vehicle.mass_properties().mass_kg, isp_s,
                                   vehicle.propellant_kg());
    } catch (const std::exception&) {  // integrator step underflow on a trial arc, ephemeris range
        lunar_plan_.reset();
        lunar_status_ = "Kein Mondflug: Planung fehlgeschlagen";
        return false;
    }
    if (!lunar_plan_->feasible) {
        lunar_status_ = lunar_plan_->reason;
        return false;
    }
    lunar_parameters_ = planned;
    lunar_nrho_ = planned.nrho;
    lunar_llo_ = planned.llo;
    lunar_mcc_index_ = 0;
    lunar_flyby_done_ = false;
    lunar_trims_ = 0;
    lunar_pilot_.reset();
    lunar_burns_.clear();
    lunar_step_ = LunarStep::Departure;
    lunar_step_epoch_ = lunar_ignition_lead(lunar_plan_->tli);
    autopilot_.state = AutopilotState::Lunar;
    attitude_mode_ = {};
    lunar_status_ = "Abflug: TLI " + german(norm(lunar_plan_->tli.delta_v_gcrf_mps), 0) + " m/s geplant";
    return true;
}

bool FlightSession::plan_lunar_orbit_transfer(mission::LunarOrbitKind target,
                                              std::shared_ptr<const mission::NrhoReference> nrho, mission::LloTarget llo) {
    if (bodies_.empty() || autopilot_.state != AutopilotState::Lunar ||
        (lunar_step_ != LunarStep::InLlo && lunar_step_ != LunarStep::InNrho) || !nrho || !moon_orientation_) {
        return false;
    }
    const bool down = lunar_step_ == LunarStep::InNrho && target == mission::LunarOrbitKind::Llo;
    const bool up = lunar_step_ == LunarStep::InLlo && target == mission::LunarOrbitKind::Nrho;
    if (!down && !up) {
        lunar_status_ = "Kein Transfer: schon in dieser Bahn";
        return false;
    }
    const mission::LunarOrbitTransfer transfer(gravity_, *env_.ephemeris, *moon_orientation_);
    mission::TwoBurnTransfer plan;
    try {
        plan = down ? transfer.nrho_to_llo(controlled_state(), epoch_, *nrho, llo)
                    : transfer.llo_to_nrho(controlled_state(), epoch_, *nrho);
    } catch (const std::exception&) {  // integrator step underflow on a trial arc, ephemeris range
        lunar_status_ = "Kein Transfer: Planung fehlgeschlagen";
        return false;
    }
    if (!plan.feasible) {
        lunar_status_ = plan.reason;
        return false;
    }
    lunar_transfer_ = plan;
    lunar_transfer_target_ = target;
    lunar_nrho_ = std::move(nrho);
    lunar_llo_ = llo;
    if (lunar_parameters_) lunar_parameters_->llo = llo;  // the trims after the arrival aim at it
    lunar_step_ = LunarStep::TransferOut;
    lunar_step_epoch_ = lunar_ignition_lead(plan.first);
    lunar_status_ = "Transfer: " + german(plan.total_delta_v_mps, 0) + " m/s geplant";
    return true;
}

time::Epoch FlightSession::lunar_ignition_lead(const gnc::BurnNode& node) const {
    const auto configurations = burn_configurations(&vacuum_groups);
    double duration_s = 0.0;
    if (!configurations.empty()) {
        duration_s = gnc::burn_duration_s(norm(node.delta_v_gcrf_mps),
                                          bodies_[controlled_].body->vehicle().mass_properties().mass_kg,
                                          configurations.front().thrust_n, configurations.front().mass_flow_kgps);
    }
    return node.epoch - 0.5 * duration_s - burn_preparation_s;
}

bool FlightSession::lunar_burn(const gnc::BurnNode& node, bool main_engines, LunarStep step) {
    // A main-engine burn too small for the vacuum set (start/shutdown impulse) and the small burns (trims, station
    // keeping) fly as lunar_small_burn: the single sea-level Raptor if it fits, else the session's choice.
    const bool ok = (main_engines && execute_burn(node, vacuum_groups)) || lunar_small_burn(node);
    if (!ok) {
        lunar_abort("Mondflug abgebrochen: Brennvorgang nicht möglich");
        return false;
    }
    const auto now = controlled_state();
    const auto centre = node.epoch - epoch_ > 0.0 ? orbit::propagate_coast(gravity_, now, epoch_, node.epoch).state : now;
    lunar_burns_.push_back({step, node, bodies_[controlled_].body->vehicle().mass_properties().mass_kg, centre.position_m});
    return ok;
}

void FlightSession::lunar_abort(const std::string& status) {
    lunar_status_ = status;
    lunar_step_ = LunarStep::None;
    lunar_pilot_.reset();
    autopilot_.state = AutopilotState::Off;
    attitude_mode_ = {};
}


void FlightSession::run_lunar(Flying& flying) {
    try {
        run_lunar_steps(flying);
        // The sequence plans its next leg in the tick the previous one settles (an orbit step waits far ahead: the
        // coast allowance would skip it); a burn still flying (state Burn) settles later.
        if (lunar_mission_ && autopilot_.state == AutopilotState::Lunar &&
            (lunar_step_ == LunarStep::InLlo || lunar_step_ == LunarStep::InNrho)) {
            lunar_mission_next();
        }
    } catch (const std::exception& e) {
        if (lunar_debug_) std::printf("[lunar] planning threw: %s\n", e.what());
        // A planner that throws (integrator step underflow on a trial arc, ephemeris range) would leave the step due
        // and throw again on every tick: the flight ends. Engines of a powered phase are cut (review l4 M1: they stayed
        // lit without steering); the vehicle then coasts or falls.
        const bool powered = lunar_step_ == LunarStep::PoweredDescent || lunar_step_ == LunarStep::Ascent ||
                             lunar_step_ == LunarStep::AbortAscent || lunar_step_ == LunarStep::Circularize;
        if (powered) cutoff();
        lunar_abort("Mondflug abgebrochen: Planung fehlgeschlagen");
    }
    if (lunar_debug_ && lunar_status_ != lunar_debug_last_) {  // diagnostics: every status change
        lunar_debug_last_ = lunar_status_;
        const auto rel = mission::moon_relative(*env_.ephemeris, controlled_state(), epoch_);
        std::printf("[lunar] %.3f d  %-14s  Moon %.0f km  %.1f m/s  %.1f t  %s\n",
                    (epoch_ - launch_epoch_) / 86400.0, to_string(lunar_step_), norm(rel.position_m) / 1000.0,
                    norm(rel.velocity_mps), flying.body->vehicle().mass_properties().mass_kg / 1000.0,
                    lunar_status_.c_str());
    }
}

void FlightSession::run_lunar_steps(Flying& flying) {
    if (lunar_step_ == LunarStep::PoweredDescent || lunar_step_ == LunarStep::Landed ||
        lunar_step_ == LunarStep::SurfaceStay || lunar_step_ == LunarStep::Lost) {
        run_lunar_descent_steps(flying);  // judges an impact itself
        return;
    }
    if (lunar_step_ == LunarStep::Ascent || lunar_step_ == LunarStep::AbortAscent ||
        lunar_step_ == LunarStep::AscentCoast || lunar_step_ == LunarStep::Circularize) {
        run_lunar_ascent_steps(flying);  // judges an impact itself
        return;
    }
    if (lunar_step_ >= LunarStep::ReturnDeparture) {
        run_lunar_return_steps(flying);  // judges an impact itself
        return;
    }
    if (flying.body->impacted()) {
        lunar_abort("Mondflug beendet: Aufschlag");
        return;
    }
    const auto next_apolune = [&] {
        const auto t = lunar_nrho_ ? first_after(lunar_nrho_->apolune_epochs(), epoch_) : std::nullopt;
        return t ? *t : epoch_ + far_future_s;
    };
    const auto enter_trim = [&](time::Epoch check) {
        lunar_step_ = LunarStep::Trim;
        lunar_trims_ = 0;
        lunar_step_epoch_ = check;
    };
    switch (lunar_step_) {
        case LunarStep::Departure: {
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            // The plan's impulsive TLI re-solved for the finite burn (~7 min, ~28 deg of the parking orbit: flown as an
            // impulse it missed the Moon and MCC 1 took 333 m/s) from the state now; not converged: the impulse.
            auto tli = lunar_plan_->tli;
            const auto configurations = burn_configurations(&vacuum_groups);
            if (configurations.empty()) {  // engines lost since the planning
                lunar_abort("Mondflug abgebrochen: keine Vakuum-Triebwerke");
                break;
            }
            const mission::FiniteBurnModel engines{flying.body->vehicle().mass_properties().mass_kg,
                                                   configurations.front().thrust_n,
                                                   configurations.front().mass_flow_kgps};
            const mission::LunarOutboundPlanner planner(gravity_, *env_.ephemeris, *moon_orientation_);
            if (const auto finite = planner.finite_tli(controlled_state(), epoch_, *lunar_parameters_, *lunar_plan_, engines)) {
                tli = *finite;
            }
            if (lunar_burn(tli, true, LunarStep::Tli)) {
                lunar_step_ = LunarStep::Tli;
                lunar_status_ = "TLI: " + german(norm(tli.delta_v_gcrf_mps), 0) + " m/s";
            }
            break;
        }
        case LunarStep::Tli:
            lunar_tli_end_ = epoch_;
            lunar_enter_coast();
            break;
        case LunarStep::MidCourse:
            ++lunar_mcc_index_;
            lunar_enter_coast();
            break;
        case LunarStep::PoweredFlyby:
            lunar_flyby_done_ = true;
            // MCCs belong to the leg to the swing-by; slots after it are dropped (the insertion is re-planned anyway).
            lunar_mcc_index_ = lunar_parameters_->mcc_after_tli_s.size();
            lunar_enter_coast();
            break;
        case LunarStep::Coast:
            lunar_sun_coast(flying);
            if (epoch_ - lunar_step_epoch_ >= 0.0) lunar_coast_event();
            break;
        case LunarStep::PilotCorrection: {
            auto& call = *lunar_pilot_;
            call.reference = orbit::propagate_coast(gravity_, call.reference, call.reference_epoch, epoch_).state;
            call.reference_epoch = epoch_;
            if (epoch_ - call.window_end >= 0.0) {
                lunar_status_ = "Kurskorrektur-Fenster verstrichen";
                lunar_pilot_.reset();
                ++lunar_mcc_index_;
                lunar_enter_coast();
                break;
            }
            const auto cue = lunar_pilot_cue();
            const double to_go = call.planned.epoch - epoch_;
            lunar_status_ = to_go > 0.0 ? "Kurskorrektur in " + german(to_go / 60.0, 0) + " min: " +
                                              german(norm(cue->remaining_delta_v_gcrf_mps), 1) + " m/s"
                                        : "Kurskorrektur jetzt: noch " + german(norm(cue->remaining_delta_v_gcrf_mps), 1) +
                                              " m/s";
            break;
        }
        case LunarStep::Insertion:
            if (lunar_parameters_->orbit == mission::LunarOrbitKind::Llo) {
                enter_trim(epoch_ + llo_trim_interval_s);
                lunar_status_ = "Bahnkorrektur";
            } else {
                lunar_step_ = LunarStep::InNrho;
                lunar_step_epoch_ = next_apolune();
                lunar_status_ = "in NRHO";
            }
            break;
        case LunarStep::Trim: {
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            const mission::LunarOutboundPlanner planner(gravity_, *env_.ephemeris, *moon_orientation_);
            const auto trim = lunar_trims_ < max_llo_trims ? planner.llo_trim(controlled_state(), epoch_, *lunar_parameters_,
                                                                            burn_preparation_s + small_burn_margin_s)
                                                           : std::nullopt;
            if (trim) {
                ++lunar_trims_;
                lunar_step_epoch_ = trim->epoch + llo_trim_interval_s;  // the next check, after this trim
                lunar_burn(*trim, false, LunarStep::Trim);
                break;
            }
            const auto e = orbit::elements_from_state(mission::moon_relative(*env_.ephemeris, controlled_state(), epoch_),
                                                      env_.ephemeris->gm_m3ps2(ephemeris::Body::Moon));
            const double low = e.semi_major_axis_m * (1.0 - e.eccentricity) - constants::moon_mean_radius_m;
            const double high = e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::moon_mean_radius_m;
            lunar_step_ = LunarStep::InLlo;
            lunar_step_epoch_ = epoch_ + far_future_s;
            lunar_status_ = "in LLO: " + german(low / 1000.0, 1) + " x " + german(high / 1000.0, 1) + " km";
            break;
        }
        case LunarStep::InNrho: {
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            const auto perilune = first_after(lunar_nrho_->perilune_epochs(), epoch_);
            if (!perilune || !lunar_nrho_->covers(*perilune)) {
                lunar_status_ = "NRHO-Referenz endet: keine Bahnhaltung mehr";
                lunar_step_epoch_ = epoch_ + far_future_s;
                break;
            }
            const mission::LunarOrbitTransfer transfer(gravity_, *env_.ephemeris, *moon_orientation_);
            if (const auto sk = transfer.station_keeping(controlled_state(), epoch_, *lunar_nrho_, 1000.0,
                                                          burn_preparation_s + small_burn_margin_s)) {
                if (lunar_burn(*sk, false, LunarStep::StationKeeping)) {
                    lunar_step_ = LunarStep::StationKeeping;
                    lunar_status_ = "Bahnhaltung: " + german(norm(sk->delta_v_gcrf_mps), 2) + " m/s";
                }
                break;
            }
            lunar_step_epoch_ = next_apolune();
            break;
        }
        case LunarStep::StationKeeping:
            lunar_step_ = LunarStep::InNrho;
            lunar_step_epoch_ = next_apolune();
            lunar_status_ = "in NRHO";
            break;
        case LunarStep::TransferOut:
            if (epoch_ - lunar_step_epoch_ >= 0.0 && lunar_burn(lunar_transfer_->first, true, LunarStep::TransferOut)) {
                lunar_step_ = LunarStep::TransferIn;  // waits for the arrival burn (re-planned then)
                lunar_step_epoch_ = lunar_ignition_lead(lunar_transfer_->second);
                lunar_status_ = "Transfer: Ankunft in " + german((lunar_transfer_->second.epoch - epoch_) / 3600.0, 1) + " h";
            }
            break;
        case LunarStep::TransferIn: {
            if (epoch_ - lunar_step_epoch_ < 0.0) break;
            const mission::LunarOrbitTransfer transfer(gravity_, *env_.ephemeris, *moon_orientation_);
            const auto arrival = transfer.arrival_burn(controlled_state(), epoch_, *lunar_transfer_, lunar_transfer_target_,
                                                       lunar_nrho_.get(), lunar_llo_);
            if (!arrival) {
                lunar_abort("Mondflug abgebrochen: Ankunft nicht planbar");
                break;
            }
            if (!lunar_burn(*arrival, true, LunarStep::TransferIn)) break;
            // The step after the arrival, waiting while the burn flies.
            if (lunar_transfer_target_ == mission::LunarOrbitKind::Llo) {
                if (!lunar_parameters_) lunar_parameters_.emplace();
                lunar_parameters_->orbit = mission::LunarOrbitKind::Llo;
                lunar_parameters_->llo = lunar_llo_;
                enter_trim(arrival->epoch + llo_trim_interval_s);
            } else {
                lunar_step_ = LunarStep::InNrho;
                const auto t = first_after(lunar_nrho_->apolune_epochs(), arrival->epoch);
                lunar_step_epoch_ = t ? *t : epoch_ + far_future_s;
            }
            lunar_status_ = "Ankunft: " + german(norm(arrival->delta_v_gcrf_mps), 0) + " m/s";
            break;
        }
        case LunarStep::DescentWait:
        case LunarStep::PlaneTrim:
        case LunarStep::Doi:
        case LunarStep::DescentCoast:
            run_lunar_descent_steps(flying);
            break;
        case LunarStep::InLlo:
        case LunarStep::None:
        case LunarStep::PoweredDescent:
        case LunarStep::Landed:
        case LunarStep::SurfaceStay:
        case LunarStep::Ascent:
        case LunarStep::AscentCoast:
        case LunarStep::Circularize:
        case LunarStep::AbortAscent:
        case LunarStep::Lost:
        case LunarStep::ReturnDeparture:
        case LunarStep::Tei:
        case LunarStep::ReturnCoast:
        case LunarStep::ReturnCorrection:
        case LunarStep::Braking:
        case LunarStep::EarthEntry:
        case LunarStep::Aerocapture:
        case LunarStep::CaptureCoast:
        case LunarStep::PerigeeRaise:
        case LunarStep::CaptureCircularize:
        case LunarStep::EntryTrim:
        case LunarStep::RepassTrim:
            break;
    }
}


}  // namespace starsim::sim
