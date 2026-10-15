// Lunar flight of the controlled vehicle (lunar l3): the outbound autopilot (TLI, mid-course corrections, powered
// swing-by, LOI / NRHO insertion, LLO trims, NRHO station keeping) and the transfers between the LLO and the NRHO.
//
// Design decisions:
// - The vehicle coasts Sun-pointed: at the start of each coast it turns to AttitudeModeKind::Sun (ticked until the error
//   is below 1 deg and the rate below 0.005 deg/s), then coasts freely; every sun_repoint_interval_s it re-aims (the Sun
//   moves ~1 deg/day; no disturbance torques are modelled, so the attitude only drifts with the hold's residual rate).
//   Ticking a 4-day coast at 200 Hz would cost ~70 M ticks.
// - Burns: TLI, swing-by, insertion and transfer burns on the vacuum Raptors; MCCs, trims and station keeping by
//   execute_burn(node) (the session picks the RCS for small ones). A main-engine burn too small for the vacuum set
//   falls back to execute_burn(node) as well.
// - After each burn the autopilot returns to the state Lunar (as a ship return returns to Return); the steps Tli,
//   MidCourse, PoweredFlyby, Insertion and StationKeeping mean "that burn is under way" and are resolved on return.
// - The coast allowance stops at the next step epoch (lunar_step_epoch_); the ignition preparation comes from Burn.
// - disengage_autopilot() ends the lunar flight (step None).

#include "starsim/sim/flight_session.hpp"

#include <exception>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/orbit/kepler.hpp"

namespace starsim::sim {

namespace {

const std::vector<std::string> vacuum_groups{"vacuum"};
constexpr double far_future_s = 1e9;                               // "nothing timed"
constexpr double sun_settled_error_rad = 1.0 * constants::deg_to_rad;   // [E]
constexpr double sun_settled_rate_radps = 0.005 * constants::deg_to_rad;  // [E] ~100 deg drift in 6 h at most

std::optional<time::Epoch> first_after(const std::vector<time::Epoch>& epochs, time::Epoch t) {
    for (const auto& e : epochs) {
        if (e - t > 0.0) return e;
    }
    return std::nullopt;
}

// German number with a decimal comma.
std::string german(double value, int decimals) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    std::string s = text;
    std::replace(s.begin(), s.end(), '.', ',');
    return s;
}

}  // namespace

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
        lunar_plan_ = planner.plan(controlled_state(), epoch_, parameters, vehicle.mass_properties().mass_kg, isp_s,
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
    lunar_parameters_ = parameters;
    lunar_nrho_ = parameters.nrho;
    lunar_llo_ = parameters.llo;
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
    // A main-engine burn too small for the vacuum set (start/shutdown impulse) flies on the session's choice.
    const bool ok = (main_engines && execute_burn(node, vacuum_groups)) || execute_burn(node);
    if (!ok) {
        lunar_abort("Mondflug abgebrochen: Brennvorgang nicht möglich");
        return false;
    }
    lunar_burns_.push_back({step, node, bodies_[controlled_].body->vehicle().mass_properties().mass_kg});
    return ok;
}

void FlightSession::lunar_abort(const std::string& status) {
    lunar_status_ = status;
    lunar_step_ = LunarStep::None;
    lunar_pilot_.reset();
    autopilot_.state = AutopilotState::Off;
    attitude_mode_ = {};
}

time::Epoch FlightSession::lunar_next_mcc_epoch() const {
    const auto& p = *lunar_parameters_;
    const auto& plan = *lunar_plan_;
    // MCCs before the next big burn only (NRHO: the swing-by; then the insertion)
    const time::Epoch big = plan.powered_flyby && !lunar_flyby_done_ ? plan.powered_flyby->epoch : plan.insertion.epoch;
    if (lunar_mcc_index_ < p.mcc_after_tli_s.size()) {
        const time::Epoch t = lunar_tli_end_ + p.mcc_after_tli_s[lunar_mcc_index_];
        const double lead = p.pilot_corrections ? p.pilot_call_lead_s : 0.0;
        if (big - t > burn_preparation_s + lead) return t;
    }
    return epoch_ + far_future_s;
}

void FlightSession::lunar_enter_coast() {
    const auto& p = *lunar_parameters_;
    const auto& plan = *lunar_plan_;
    lunar_step_ = LunarStep::Coast;
    attitude_mode_ = {gnc::AttitudeModeKind::Sun, {}, {}, {}};
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

void FlightSession::run_lunar(Flying& flying) {
    try {
        run_lunar_steps(flying);
    } catch (const std::exception&) {
        // A planner that throws (integrator step underflow on a trial arc, ephemeris range) would leave the step due
        // and throw again on every tick: the flight ends, the vehicle coasts.
        lunar_abort("Mondflug abgebrochen: Planung fehlgeschlagen");
    }
    if (std::getenv("STARSIM_DEBUG_LUNAR_SESSION")) {  // diagnostics: every status change
        static std::string last;
        if (lunar_status_ != last) {
            last = lunar_status_;
            const auto rel = mission::moon_relative(*env_.ephemeris, controlled_state(), epoch_);
            std::printf("[lunar] %.3f d  %-14s  Moon %.0f km  %.1f m/s  %.1f t  %s\n",
                        (epoch_ - launch_epoch_) / 86400.0, to_string(lunar_step_), norm(rel.position_m) / 1000.0,
                        norm(rel.velocity_mps), flying.body->vehicle().mass_properties().mass_kg / 1000.0,
                        lunar_status_.c_str());
        }
    }
}

void FlightSession::run_lunar_steps(Flying& flying) {
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
        case LunarStep::Coast: {
            if (attitude_mode_.kind == gnc::AttitudeModeKind::Sun) {
                const auto& s = flying.body->state();
                const math::Vec3 sun =
                    env_.ephemeris->state_gcrf(ephemeris::Body::Sun, epoch_).position_m - s.position_gcrf_m;
                const double error = std::acos(
                    std::clamp(dot(s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0}), math::normalized(sun)), -1.0, 1.0));
                if (error < sun_settled_error_rad && norm(s.angular_velocity_body_radps) < sun_settled_rate_radps) {
                    attitude_mode_ = {};  // pointed: coast freely
                    if (std::getenv("STARSIM_DEBUG_LUNAR_SESSION")) {
                        std::printf("[lunar] Sun pointed: %.2f deg, %.4f deg/s, %.0f kg\n", error / constants::deg_to_rad,
                                    norm(s.angular_velocity_body_radps) / constants::deg_to_rad,
                                    flying.body->vehicle().mass_properties().mass_kg);
                    }
                }
            }
            if (epoch_ - lunar_next_repoint_ >= 0.0) {
                attitude_mode_ = {gnc::AttitudeModeKind::Sun, {}, {}, {}};
                lunar_next_repoint_ = epoch_ + sun_repoint_interval_s;
                if (std::getenv("STARSIM_DEBUG_LUNAR_SESSION")) {
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
            if (epoch_ - lunar_step_epoch_ >= 0.0) lunar_coast_event();
            break;
        }
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
            const auto trim = lunar_trims_ < max_llo_trims ? planner.llo_trim(controlled_state(), epoch_, *lunar_parameters_)
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
            if (const auto sk = transfer.station_keeping(controlled_state(), epoch_, *lunar_nrho_)) {
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
        case LunarStep::InLlo:
        case LunarStep::None:
            break;
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
