// Rendezvous phasing of the controlled vehicle with the station (ISS i3): the coelliptic profile from the separation
// orbit to Waypoint 0, 400 m below the station (mission::PhasingPlanner).
//
// Design decisions:
// - Every burn is re-targeted from the live states at its command epoch (the plan only fixes the epochs); the burns fly
//   through execute_burn (Crew Dragon: the Dracos along +x). The big burns are commanded burn_drift_lead_s before their
//   ignition (the vehicle turns in the drift lead, as the capsule deorbit); the mid-course correction and the braking
//   burn are small and follow each other within minutes: commanded small_burn_margin_s before the preparation.
// - Drag tracking: the controlled body's C_D A / m from its arc since the last burn, the station's from its arc since
//   the plan or its last impulse (gnc::estimate_drag_area_per_mass), each once the arc is rendezvous_track_min_arc_s
//   long; the estimates replace the plan's drag fields before every burn.
// - A waiting plan (no phase burn) checks the boost geometry rendezvous_check_lead_s before the boost with the tracked
//   drag and re-plans from the live states beyond rendezvous_replan_tolerance_m (memory feedback-replan-from-flight-data);
//   a burn that cannot be targeted before the boost re-plans once, later it aborts.
// - The coast allowance stops at the next step epoch (the command or the boost check); between the burns Crew Dragon
//   drifts freely (no attitude mode), a Starship holds its docking orientation in LVLH.
// - Refuelling r3: a main-engine chaser (the tanker, the HLS) burns on the belly Raptor (its deorbit set), below that
//   engine's minimum impulse on the cold-gas RCS; its budget is the main tanks (rcs_supply_kg).

#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/station_limits.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/math/constants.hpp"
#include "flight_session_lunar_detail.hpp"

namespace starsim::sim {

using detail::german;
using mission::PhasingBurnKind;

const char* to_string(RendezvousStep step) {
    switch (step) {
        case RendezvousStep::None: return "aus";
        case RendezvousStep::Coast: return "Phasing";
        case RendezvousStep::Burn: return "Manöver";
        case RendezvousStep::Waypoint0: return "Wegpunkt 0";
    }
    return "";
}

math::StateVector FlightSession::station_state() const {
    const auto& s = bodies_.at(*station_index_).body->state();
    return {s.position_gcrf_m, s.velocity_gcrf_mps};
}

const config::StationProximityConfig& FlightSession::station_limits() const {
    return bodies_.at(*station_index_).body->vehicle().stages().back().config.station_proximity;
}

bool FlightSession::rendezvous_main_engines() const { return !burn_configurations().empty(); }

// A main-engine chaser (refuelling r3: Starship) burns on its deorbit set - the single belly Raptor - with transients
// up to small_burn_transient_share of the burn; below that minimum on the cold-gas RCS along +x (nose along the
// delta-v), as start_burn chooses.
double FlightSession::rendezvous_rcs_below_mps() const {
    const auto& groups = bodies_[controlled_].body->vehicle().stages().front().config.deorbit_engine_groups;
    const auto set = groups.empty() ? burn_configurations() : burn_configurations(&groups);
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto& c : set) minimum = std::min(minimum, c.minimum_dv_mps);
    return minimum / small_burn_transient_share;
}

double FlightSession::rendezvous_transient_share(double delta_v_mps) const {
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    auto capability = vehicle.rcs_force_capability({1.0, 0.0, 0.0});
    if (!(capability.force_n > 0.0)) capability = vehicle.rcs_force_capability({-1.0, 0.0, 0.0});
    const double cold_gas_s = capability.force_n > 0.0 && std::isfinite(delta_v_mps)
                                  ? delta_v_mps * vehicle.mass_properties().mass_kg / capability.force_n
                                  : 0.0;
    return cold_gas_s > rendezvous_max_cold_gas_burn_s ? 1.0 : small_burn_transient_share;
}

mission::FiniteBurnModel FlightSession::rendezvous_engines(double delta_v_mps) const {
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    const double mass = vehicle.mass_properties().mass_kg;
    const double share = rendezvous_transient_share(delta_v_mps);
    if (rendezvous_main_engines() && delta_v_mps >= rendezvous_rcs_below_mps() * small_burn_transient_share / share) {
        const auto& groups = vehicle.stages().front().config.deorbit_engine_groups;
        const auto set = groups.empty() ? burn_configurations() : burn_configurations(&groups);
        // the configuration start_burn will fly (r3 minor M1: a burn between the two minimum impulses flies at the
        // minimum throttle); a failed belly Raptor leaves the set empty: the cold gas flies every burn
        for (const auto& c : set) {
            if (c.minimum_dv_mps <= share * delta_v_mps) return {mass, c.thrust_n, c.mass_flow_kgps};
        }
    }
    // RCS translation as start_burn picks it: along +x, else along -x
    auto capability = vehicle.rcs_force_capability({1.0, 0.0, 0.0});
    if (!(capability.force_n > 0.0)) capability = vehicle.rcs_force_capability({-1.0, 0.0, 0.0});
    return {mass, capability.force_n, capability.mass_flow_kgps};
}

double FlightSession::rendezvous_budget_kg() const {
    const auto& vehicle = bodies_[controlled_].body->vehicle();
    if (!rendezvous_main_engines()) return vehicle.rcs_propellant_kg();  // Crew Dragon: the Draco tanks
    return vehicle.rcs_supply_kg();  // the main tanks (engines and cold gas); the headers are the landing reserve
}

bool FlightSession::execute_rendezvous_burn(const gnc::BurnNode& node) {
    if (!rendezvous_main_engines()) return execute_burn(node);
    const auto& groups = bodies_[controlled_].body->vehicle().stages().front().config.deorbit_engine_groups;
    return start_burn(node, groups.empty() ? burn_configurations() : burn_configurations(&groups), true,
                      rendezvous_transient_share(norm(node.delta_v_gcrf_mps)));
}

void FlightSession::reset_rendezvous() {
    rendezvous_step_ = RendezvousStep::None;
    rendezvous_plan_.reset();
    rendezvous_next_ = 0;
    rendezvous_mid_course_ = false;
    rendezvous_retarget_replanned_ = false;
    rendezvous_boost_check_.reset();
    rendezvous_status_.clear();
    rendezvous_burns_.clear();
    rendezvous_warnings_.clear();
    rendezvous_replans_ = 0;
}

void FlightSession::rendezvous_abort(const std::string& status) {
    rendezvous_status_ = status;
    rendezvous_step_ = RendezvousStep::None;
    rendezvous_boost_check_.reset();
    autopilot_.state = AutopilotState::Off;
    attitude_mode_ = {};
}

bool FlightSession::plan_rendezvous(mission::PhasingParameters p) {
    const auto refuse = [&](const std::string& reason) {
        rendezvous_status_ = reason;
        return false;
    };
    if (bodies_.empty() || !station_index_) return refuse("Kein Rendezvous: keine Station");
    if (docking_busy()) return refuse("Rendezvous nicht möglich: angedockt");
    if (controlled_ == *station_index_) return refuse("Kein Rendezvous: die Station selbst wird gesteuert");
    if (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit) {
        return refuse("Kein Rendezvous: Autopilot aktiv");
    }
    if ((rendezvous_step_ != RendezvousStep::None && rendezvous_step_ != RendezvousStep::Waypoint0) ||
        lunar_step_ != LunarStep::None) {
        return refuse("Kein Rendezvous: ein Flug läuft bereits");
    }
    p = with_station_limits(p, station_limits());
    // Crew Dragon flies the phasing with its nose cone open (it opens after the separation): only then its Dracos turn
    // it without a push - closed, a turn to a burn pushed it up to 0.28 m/s (i3: Waypoint 0 missed by 130-390 m).
    auto& vehicle = bodies_[controlled_].body->vehicle();
    const bool cone_was_open = vehicle.nose_cone_open();
    if (vehicle.has_nose_cone()) vehicle.set_nose_cone_open(true);
    const auto refuse_closed = [&](const std::string& reason) {  // a refused plan leaves the cone as it was
        if (vehicle.has_nose_cone()) vehicle.set_nose_cone_open(cone_was_open);
        return refuse(reason);
    };
    const double mass = vehicle.mass_properties().mass_kg;
    if (!(p.chaser_drag_area_per_mass_m2pkg > 0.0)) {
        const double radius = 0.5 * vehicle.diameter_m();
        p.chaser_drag_area_per_mass_m2pkg = dragon_free_molecular_cd * constants::pi * radius * radius / mass;
    }
    if (!(p.target_drag_area_per_mass_m2pkg > 0.0)) {
        const auto& station = bodies_[*station_index_].body->vehicle();
        const auto& aero = station.stages().front().config.aero;
        const double radius = 0.5 * station.diameter_m();
        const double cd_area = aero.fixed_drag_area_m2 > 0.0 ? aero.fixed_drag_area_m2 * aero.fixed_drag_coefficient
                                                             : dragon_free_molecular_cd * constants::pi * radius * radius;
        p.target_drag_area_per_mass_m2pkg = cd_area / station.mass_properties().mass_kg;  // the depot flies nose first
    }
    // f3: each phasing burn settles first (the phasing drains the tanks little: the gas with the tanks as now)
    p.reserve_per_burn_kg = settling_gas_kg();
    if (!(rendezvous_budget_kg() > 0.0)) return refuse_closed("Kein Rendezvous: kein Treibstoff");
    const auto engines = rendezvous_engines();
    if (!(engines.thrust_n > 0.0)) return refuse_closed("Kein Rendezvous: keine Triebwerke für die Manöver");
    mission::PhasingPlan plan;
    try {
        plan = mission::PhasingPlanner(gravity_).plan(controlled_state(), station_state(), epoch_, p, engines,
                                                      rendezvous_budget_kg());
    } catch (const std::exception& e) {
        if (rendezvous_debug_) std::printf("[rendezvous] planning threw: %s\n", e.what());
        return refuse_closed("Kein Rendezvous: Planung fehlgeschlagen");
    }
    if (!plan.feasible) return refuse_closed(plan.reason);
    reset_rendezvous();
    rendezvous_deadline_ = epoch_ + p.mission_limit_s;
    rendezvous_earliest_ = epoch_ + p.earliest_arrival_s;
    rendezvous_chaser_ref_ = controlled_state();
    rendezvous_chaser_ref_epoch_ = epoch_;
    rendezvous_target_ref_ = station_state();
    rendezvous_target_ref_epoch_ = epoch_;
    rendezvous_target_ref_impulses_ = station_impulses_.size();
    for (const auto& impulse : station_impulses_) {
        if (impulse.epoch >= epoch_ && impulse.epoch <= plan.schedule.waypoint0_epoch) {
            rendezvous_warnings_.push_back("Achtung: Bahnanhebung der Station um " + german(impulse.delta_v_mps, 2) +
                                           " m/s während des Rendezvous");
        }
    }
    autopilot_.state = AutopilotState::Rendezvous;
    attitude_mode_ = {};
    rendezvous_start(std::move(plan));
    return true;
}

void FlightSession::rendezvous_start(mission::PhasingPlan plan) {
    rendezvous_plan_ = std::move(plan);
    rendezvous_next_ = 0;
    rendezvous_mid_course_ = false;
    rendezvous_boost_check_.reset();
    const auto& s = rendezvous_plan_->schedule;
    // Only a waiting plan checks the boost geometry: a phase plan's re-plan from the eccentric phase orbit boosts off its
    // perigee and cost more (i3 review: crew day, air x1.5, 635 kg against 603 kg when the coelliptic burn absorbs the
    // -34 km phase error; a phase trim burn after the drag tracking would be the fix - open).
    if (s.phasing_revolutions == 0) {
        const time::Epoch check = s.boost_epoch - rendezvous_check_lead_s;
        if (check > epoch_) rendezvous_boost_check_ = check;
    }
    rendezvous_enter_coast();
}

PhasingBurnKind FlightSession::rendezvous_next_kind() const {
    return rendezvous_mid_course_ ? PhasingBurnKind::MidCourse : rendezvous_plan_->burns.at(rendezvous_next_).kind;
}

time::Epoch FlightSession::rendezvous_preparation_epoch(const gnc::BurnNode& node) const {
    const auto engines = rendezvous_engines(norm(node.delta_v_gcrf_mps));
    const double duration_s =
        gnc::burn_duration_s(norm(node.delta_v_gcrf_mps), engines.mass_kg, engines.thrust_n, engines.mass_flow_kgps);
    return node.epoch - 0.5 * duration_s - burn_lead_s();
}

double FlightSession::rendezvous_turn_lead_s(PhasingBurnKind kind) {
    const bool small = kind == PhasingBurnKind::MidCourse || kind == PhasingBurnKind::Braking;
    return small ? rendezvous_small_turn_lead_s : burn_drift_lead_s;
}

void FlightSession::rendezvous_enter_coast() {
    rendezvous_step_ = RendezvousStep::Coast;
    // Crew Dragon drifts between the burns; a Starship holds nose along +V, belly to nadir (the docking attitude's
    // orientation, rolled 180 deg from LVLH) - its drag and boil-off stay defined (refuelling r3)
    attitude_mode_ = rendezvous_main_engines()
                         ? gnc::lvlh_hold(math::Quat::from_axis_angle({1.0, 0.0, 0.0}, constants::pi))
                         : gnc::AttitudeMode{};
    const auto& plan = *rendezvous_plan_;
    const auto kind = rendezvous_next_kind();
    // the planned node (the mid-course correction has none: its epoch, no delta-v)
    const gnc::BurnNode node = kind == PhasingBurnKind::MidCourse
                                   ? gnc::BurnNode{plan.schedule.approach_initiation_epoch +
                                                       plan.parameters.mid_course_after_s,
                                                   {}}
                                   : plan.burns.at(rendezvous_next_).node;
    const time::Epoch turn = rendezvous_preparation_epoch(node) - rendezvous_turn_lead_s(kind);
    rendezvous_step_epoch_ = std::max(turn, epoch_);
    if (rendezvous_boost_check_) rendezvous_step_epoch_ = std::min(rendezvous_step_epoch_, *rendezvous_boost_check_);
}

void FlightSession::rendezvous_track_drag() {
    auto& p = rendezvous_plan_->parameters;
    if (epoch_ - rendezvous_chaser_ref_epoch_ >= rendezvous_track_min_arc_s) {
        p.chaser_drag_area_per_mass_m2pkg = gnc::estimate_drag_area_per_mass(
            gravity_, rendezvous_chaser_ref_, rendezvous_chaser_ref_epoch_, controlled_state(), epoch_,
            p.chaser_drag_area_per_mass_m2pkg);
    }
    if (station_impulses_.size() != rendezvous_target_ref_impulses_) {  // a reboost: a new arc after it
        rendezvous_target_ref_ = station_state();
        rendezvous_target_ref_epoch_ = epoch_;
        rendezvous_target_ref_impulses_ = station_impulses_.size();
    } else if (epoch_ - rendezvous_target_ref_epoch_ >= rendezvous_track_min_arc_s) {
        p.target_drag_area_per_mass_m2pkg = gnc::estimate_drag_area_per_mass(
            gravity_, rendezvous_target_ref_, rendezvous_target_ref_epoch_, station_state(), epoch_,
            p.target_drag_area_per_mass_m2pkg);
    }
}

bool FlightSession::rendezvous_replan() {
    auto p = rendezvous_plan_->parameters;  // with the tracked drag
    p.mission_limit_s = rendezvous_deadline_ - epoch_;
    p.earliest_arrival_s = std::max(0.0, rendezvous_earliest_ - epoch_);
    auto plan = mission::PhasingPlanner(gravity_).plan(controlled_state(), station_state(), epoch_, p,
                                                       rendezvous_engines(), rendezvous_budget_kg());
    if (!plan.feasible) {
        rendezvous_abort(plan.reason);
        return false;
    }
    ++rendezvous_replans_;
    rendezvous_start(std::move(plan));
    const auto& first = rendezvous_plan_->burns.front();
    rendezvous_status_ = std::string("Neu geplant: ") + mission::german_name(first.kind) + " in " +
                         german((first.node.epoch - epoch_) / 3600.0, 1) + " h";
    return true;
}

void FlightSession::rendezvous_fly_due_burn() {
    const auto kind = rendezvous_next_kind();
    const mission::PhasingPlanner planner(gravity_);
    auto node = planner.retarget(kind, controlled_state(), station_state(), epoch_, *rendezvous_plan_, rendezvous_engines());
    if (node) {
        // a smaller burn flies weaker (the Raptor at its minimum throttle, or the long cold-gas burn below its minimum
        // impulse): targeted again with the model it flies
        const auto flown = rendezvous_engines(norm(node->delta_v_gcrf_mps));
        if (flown.thrust_n != rendezvous_engines().thrust_n) {
            if (auto fine = planner.retarget(kind, controlled_state(), station_state(), epoch_, *rendezvous_plan_, flown)) {
                node = fine;
            }
        }
    }
    if (kind == PhasingBurnKind::MidCourse &&
        (!node || norm(node->delta_v_gcrf_mps) < rendezvous_plan_->parameters.mid_course_threshold_mps)) {
        // below the threshold, or not converged (then the braking burn still stops the Dragon near Waypoint 0)
        rendezvous_mid_course_ = false;
        rendezvous_status_ = node ? "Kurskorrektur nicht nötig" : "Kurskorrektur nicht berechenbar";
        rendezvous_enter_coast();
        return;
    }
    if (!node) {
        const bool before_boost = kind == PhasingBurnKind::Phase || kind == PhasingBurnKind::Boost;
        if (before_boost && !rendezvous_retarget_replanned_) {
            rendezvous_retarget_replanned_ = true;
            rendezvous_replan();
            return;
        }
        rendezvous_abort(std::string("Rendezvous abgebrochen: ") + mission::german_name(kind) + " nicht berechenbar");
        return;
    }
    const double mass = bodies_[controlled_].body->vehicle().mass_properties().mass_kg;
    if (!execute_rendezvous_burn(*node)) {
        rendezvous_abort("Rendezvous abgebrochen: Brennvorgang nicht möglich");
        return;
    }
    rendezvous_burns_.push_back({kind, *node, mass});
    if (rendezvous_debug_) {  // diagnostics: the planner's prediction of the burn, compared after it
        rendezvous_debug_command_ = {controlled_state(), epoch_, rendezvous_engines(norm(node->delta_v_gcrf_mps))};
        std::printf("[rendezvous] %s on %s: %.0f N, cold gas below %.2f m/s, in %.0f s\n", mission::german_name(kind),
                    burn_config_.rcs ? "cold gas" : "engines", burn_config_.thrust_n, rendezvous_rcs_below_mps(),
                    node->epoch - epoch_);
    }
    if (kind == PhasingBurnKind::MidCourse) {
        rendezvous_mid_course_ = false;
    } else {
        ++rendezvous_next_;
    }
    rendezvous_retarget_replanned_ = false;
    rendezvous_step_ = RendezvousStep::Burn;
    rendezvous_status_ = std::string(mission::german_name(kind)) + ": " + german(norm(node->delta_v_gcrf_mps), 2) + " m/s";
}

void FlightSession::rendezvous_after_burn() {
    rendezvous_chaser_ref_ = controlled_state();  // the drag arc starts after the burn
    rendezvous_chaser_ref_epoch_ = epoch_;
    const auto last = rendezvous_burns_.back().kind;
    if (rendezvous_debug_ && rendezvous_debug_command_) {  // execution error against the planner's finite-burn model
        const auto& c = *rendezvous_debug_command_;
        const auto& node = rendezvous_burns_.back().node;
        const double beta = rendezvous_plan_->parameters.chaser_drag_area_per_mass_m2pkg;
        auto predicted = gnc::coast_with_drag(gravity_, c.state, c.epoch, node.epoch, beta);
        predicted = mission::apply_finite_burn(gravity_, predicted, node.epoch, node.delta_v_gcrf_mps, c.engines);
        predicted = gnc::coast_with_drag(gravity_, predicted, node.epoch, epoch_, beta);
        const auto now = controlled_state();
        const math::Vec3 dv_error = now.velocity_mps - predicted.velocity_mps;
        const math::Vec3 along = normalized(now.velocity_mps);
        const math::Vec3 radial = normalized(now.position_m);
        std::printf("[rendezvous] %s executed: dv error %.3f m/s (along %.3f, radial %.3f), position error %.1f m\n",
                    mission::german_name(last), norm(dv_error), dot(dv_error, along), dot(dv_error, radial),
                    norm(now.position_m - predicted.position_m));
        rendezvous_debug_command_.reset();
    }
    if (last == PhasingBurnKind::Braking) {
        rendezvous_step_ = RendezvousStep::Waypoint0;
        autopilot_.state = AutopilotState::Off;
        rendezvous_status_ = "Wegpunkt 0 erreicht";
        return;
    }
    if (last == PhasingBurnKind::ApproachInitiation) rendezvous_mid_course_ = true;
    rendezvous_enter_coast();
}

void FlightSession::run_rendezvous_steps() {
    if (!station_index_) {
        rendezvous_abort("Rendezvous abgebrochen: keine Station");
        return;
    }
    if (bodies_[controlled_].body->impacted()) {
        rendezvous_abort("Rendezvous abgebrochen: Fahrzeug verloren");
        return;
    }
    switch (rendezvous_step_) {
        case RendezvousStep::None:
        case RendezvousStep::Waypoint0:
            autopilot_.state = AutopilotState::Off;  // e.g. back from a burn flown by hand at Waypoint 0
            return;
        case RendezvousStep::Burn:
            rendezvous_after_burn();  // the state is Rendezvous again: the burn is complete
            return;
        case RendezvousStep::Coast:
            break;
    }
    if (epoch_ < rendezvous_step_epoch_) {
        const auto kind = rendezvous_next_kind();
        const time::Epoch due = kind == PhasingBurnKind::MidCourse
                                    ? rendezvous_plan_->schedule.approach_initiation_epoch +
                                          rendezvous_plan_->parameters.mid_course_after_s
                                    : rendezvous_plan_->burns.at(rendezvous_next_).node.epoch;
        rendezvous_status_ = std::string("Phasing: ") + mission::german_name(kind) + " in " +
                             german((due - epoch_) / 3600.0, 1) + " h";
        return;
    }
    rendezvous_track_drag();
    if (rendezvous_boost_check_ && epoch_ >= *rendezvous_boost_check_) {
        rendezvous_boost_check_.reset();
        const double error_m = mission::PhasingPlanner(gravity_).boost_phase_error_m(controlled_state(), station_state(),
                                                                                    epoch_, *rendezvous_plan_);
        if (rendezvous_debug_) std::printf("[rendezvous] boost phase error %.0f m\n", error_m);
        if (std::abs(error_m) > rendezvous_replan_tolerance_m) {
            rendezvous_replan();
        } else {
            rendezvous_enter_coast();
        }
        return;
    }
    rendezvous_fly_due_burn();
}

void FlightSession::run_rendezvous(Flying& flying) {
    try {
        run_rendezvous_steps();
    } catch (const std::exception& e) {
        if (rendezvous_debug_) std::printf("[rendezvous] planning threw: %s\n", e.what());
        // a throwing planner would throw again on every tick: the rendezvous ends, the vehicle drifts
        rendezvous_abort("Rendezvous abgebrochen: Planung fehlgeschlagen");
    }
    if (rendezvous_debug_ && rendezvous_status_ != rendezvous_debug_last_) {  // diagnostics: every status change
        rendezvous_debug_last_ = rendezvous_status_;
        const double range_m = station_index_ ? norm(station_state().position_m - controlled_state().position_m) : 0.0;
        std::printf("[rendezvous] %7.3f h  %-10s  range %.1f km  %.3f t  %s\n", mission_time_s() / 3600.0,
                    to_string(rendezvous_step_), range_m / 1000.0,
                    flying.body->vehicle().mass_properties().mass_kg / 1000.0, rendezvous_status_.c_str());
    }
}

}  // namespace starsim::sim
