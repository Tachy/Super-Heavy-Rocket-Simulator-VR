// ISS missions (i6): the legs of a mission chained on the session tick - ascent -> phasing -> proximity -> docking,
// or docked -> undocking -> departure -> capsule return. Refuelling r5: docked -> transfer -> undocking -> departure ->
// ship return to the catch or a continuation (the lunar mission); the depot's ascent ends as the station.

#include <algorithm>
#include <string>

#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

const char* id_of(IssMissionLeg leg) {  // i18n-ns: iss_mission_leg
    switch (leg) {
        case IssMissionLeg::None: return "none";
        case IssMissionLeg::Ascent: return "ascent";
        case IssMissionLeg::Raise: return "raise";
        case IssMissionLeg::Rendezvous: return "rendezvous";
        case IssMissionLeg::Proximity: return "proximity";
        case IssMissionLeg::Docking: return "docking";
        case IssMissionLeg::Docked: return "docked";
        case IssMissionLeg::Transfer: return "transfer";
        case IssMissionLeg::WaitUndock: return "wait_undock";
        case IssMissionLeg::Departure: return "departure";
        case IssMissionLeg::Return: return "return";
        case IssMissionLeg::WaitContinue: return "wait_continue";
        case IssMissionLeg::Complete: return "complete";
        case IssMissionLeg::Ended: return "ended";
    }
    return "";
}

namespace {

bool has_docking_port(const vehicle::Vehicle& v) {
    return std::any_of(v.stages().begin(), v.stages().end(),
                       [](const auto& stage) { return !stage.config.docking_ports.empty(); });
}

// The free volume of the main tanks of one propellant (headers excluded: the landing reserve), in kg.
double main_free_kg(const vehicle::Vehicle& v, config::TankContents contents) {
    double free = 0.0;
    for (const auto& stage : v.stages()) {
        for (const auto& tank : stage.tanks) {
            if (tank.config.role == config::TankRole::Main && tank.config.contents == contents) {
                free += tank.config.capacity_kg - tank.mass_kg;
            }
        }
    }
    return free;
}

// The status reports of the legs: "<leg>" alone, "<leg>: <cause>", and the closing ones.
report::Report leg_ref_report(report::Code code, IssMissionLeg leg) {
    return report::make(code, {report::ref("leg", std::string("phase.iss_mission_leg.") + id_of(leg))});
}

report::Report leg_report(IssMissionLeg leg) { return leg_ref_report(report::Code::iss_leg, leg); }

report::Report leg_status(IssMissionLeg leg, report::Report cause) {
    return report::with_cause(leg_ref_report(report::Code::iss_leg_status, leg), std::move(cause));
}

report::Report completed(report::Code reason, std::initializer_list<report::Param> params = {}) {
    return report::with_cause(report::make(report::Code::iss_completed), report::make(reason, params));
}

report::Report simple(report::Code code) { return report::make(code); }

}  // namespace

void FlightSession::set_iss_mission(std::optional<IssMissionSequence> sequence) {
    // a new sequence is a new owner: a reserve job of the old one is never handed over, also when the new one enters
    // the Docked leg before a tick (fix round 3)
    if (tanker_reserve_task_.busy()) tanker_reserve_orphan_ = true;
    iss_mission_ = std::move(sequence);
    ++iss_generation_;  // a continuation that sets a new sequence must not be ended as the old one (r5 minor)
    raise_start_epoch_.reset();
    iss_status_ = report::Report{};
    iss_marks_.clear();
    iss_capture_retries_ = 0;
    if (!iss_mission_) {
        iss_leg_ = IssMissionLeg::None;
        return;
    }
    if (iss_mission_->from_launch) {
        iss_enter(IssMissionLeg::Ascent, leg_report(IssMissionLeg::Ascent));
    } else if (iss_mission_->transfer && docked()) {
        // through the Docked gate: an active docking autopilot hands over first (r5 minor: start_transfer refused it)
        iss_enter(IssMissionLeg::Docked, leg_report(IssMissionLeg::Docked));
    } else if (iss_mission_->undock_epoch) {
        iss_enter(IssMissionLeg::WaitUndock, simple(report::Code::iss_undock_planned));
    } else {
        iss_enter(IssMissionLeg::Rendezvous, leg_report(IssMissionLeg::Rendezvous));
    }
}

void FlightSession::mark_rendezvous_at_waypoint0_for_test() {
    rendezvous_step_ = RendezvousStep::Waypoint0;
    rendezvous_status_ = report::make(report::Code::rendezvous_waypoint0_reached);
}

void FlightSession::iss_enter(IssMissionLeg leg, report::Report status) {
    iss_leg_ = leg;
    release_background_tasks();  // leaving Docked orphans a running reserve
    iss_status_ = std::move(status);
    if (leg == IssMissionLeg::Departure) iss_return_requested_ = false;
    if (bodies_.empty() || controlled_ >= bodies_.size()) return;
    const auto& v = bodies_[controlled_].body->vehicle();
    MissionMark mark{leg, epoch_, v.mass_properties().mass_kg, v.main_propellant_kg(), v.rcs_supply_kg(), 0.0};
    if (station_index_) mark.station_main_propellant_kg = bodies_[*station_index_].body->vehicle().main_propellant_kg();
    iss_marks_.push_back(mark);
}

void FlightSession::iss_end(report::Report reason) {
    iss_enter(IssMissionLeg::Ended, report::with_cause(report::make(report::Code::iss_ended), std::move(reason)));
}

void FlightSession::iss_start_transfer() {
    const auto& q = *iss_mission_;
    std::optional<TransferRequest> request;
    std::string reason;
    if (*q.transfer == MissionTransfer::TankerReserve) {
        // real-time play (fix round 1, I3): the reserve's two return plans and the boil-off flight on a worker while the
        // pair stays docked; else at once
        const auto reserve = realtime_planning_ || tanker_reserve_task_.busy() ? poll_tanker_reserve(&reason)
                                                                               : tanker_reserve(&reason);
        if (!reserve && reason.empty()) return;  // still planning: the Docked leg asks again next tick
        if (reserve) request = tanker_transfer_request(*reserve);
    } else {  // the chaser's main tanks up to full; the station down to empty
        TransferRequest r;
        r.direction = TransferDirection::StationToChaser;
        const auto& v = bodies_[controlled_].body->vehicle();
        r.limit_kg = {main_free_kg(v, config::TankContents::Oxidizer), main_free_kg(v, config::TankContents::Fuel)};
        request = r;
    }
    if (!request) return iss_end(report::legacy(reason));
    if (!start_transfer(*request)) {
        return iss_end(transfer_ && transfer_->active() ? docking_status_ : report::legacy(transfer_state().status));
    }
    iss_enter(IssMissionLeg::Transfer, leg_report(IssMissionLeg::Transfer));
}

void FlightSession::iss_start_raise(bool first) {
    const auto& s = bodies_[controlled_].body->state();
    const math::StateVector state{s.position_gcrf_m, s.velocity_gcrf_mps};
    const auto& to = *iss_mission_->raise_to;
    // The burn sequence is fixed at the start (refuelling f1): the osculating apsides swing by several km with J2, so a
    // re-check after each burn would keep adding burns. First: the apoapsis raise if needed (then the periapsis raise
    // at the new apoapsis follows), else the periapsis raise alone.
    // f2: the burns sized from the J2-mean apsides (the osculating ones swing +-10 km at 200 km; an apoapsis raise
    // planned from them overshot by several km and the below-depot orbits then crossed the depot's)
    const auto mean = orbit::j2_mean_orbit(state, gm_earth_, constants::earth_j2,
                                           environment::earth_j2_reference_radius_m, math::Vec3{0.0, 0.0, 1.0});
    const double mean_rp = mean.semi_major_axis_m * (1.0 - mean.eccentricity);
    const double mean_ra = mean.semi_major_axis_m * (1.0 + mean.eccentricity);
    std::optional<gnc::BurnNode> node;
    if (first) {
        raise_start_epoch_ = epoch_;  // the separation: the window's phasing limit counts from here (f1 minor M2)
        node = gnc::next_raise_node_mean(state, epoch_, gm_earth_, mean_rp, mean_ra, to.periapsis_radius_m,
                                         to.apoapsis_radius_m, mission::raise_tolerance_m);
        raise_periapsis_pending_ = node && to.apoapsis_radius_m > mean_ra + mission::raise_tolerance_m;
    } else if (raise_periapsis_pending_) {
        raise_periapsis_pending_ = false;
        // raised only, as the window's estimate (f1 minor M1: a candidate below the new periapsis lowered it)
        node = gnc::next_raise_node_mean(state, epoch_, gm_earth_, mean_rp, mean_ra, to.periapsis_radius_m,
                                         to.apoapsis_radius_m, mission::raise_tolerance_m, true);
    }
    if (!node) {  // at (or above) the phasing orbit
        auto phasing = iss_mission_->phasing;
        if (raise_start_epoch_) {  // the raise burns took their time out of the limit (f1 minor M2)
            const double used_s = epoch_ - *raise_start_epoch_;
            phasing.mission_limit_s -= used_s;
            phasing.earliest_arrival_s = std::max(0.0, phasing.earliest_arrival_s - used_s);
        }
        if (plan_rendezvous(phasing)) {
            iss_enter(IssMissionLeg::Rendezvous, leg_status(IssMissionLeg::Rendezvous, rendezvous_status_));
        } else {
            iss_end(rendezvous_status_);
        }
        return;
    }
    // as a phasing burn: the deorbit set (the belly Raptor) with transients up to small_burn_transient_share, else
    // the cold gas
    const auto& groups = bodies_[controlled_].body->vehicle().stages().front().config.deorbit_engine_groups;
    const bool started = start_burn(*node, groups.empty() ? burn_configurations() : burn_configurations(&groups), true,
                                    small_burn_transient_share);
    if (!started) return iss_end(simple(report::Code::iss_raise_not_possible));
    iss_enter(IssMissionLeg::Raise, leg_report(IssMissionLeg::Raise));
}

bool FlightSession::iss_mission_pending() const {
    if (!iss_mission_ || bodies_.empty() || controlled_ >= bodies_.size()) return false;
    const auto& body = *bodies_[controlled_].body;
    const bool autopilot_idle = autopilot_.state == AutopilotState::Off || autopilot_.state == AutopilotState::Orbit;
    switch (iss_leg_) {
        case IssMissionLeg::Ascent:  // after the engines' tail-off (r5: ~12 m/s at 5 g left after the cutoff command)
            return autopilot_idle && !body.vehicle().any_engine_producing_thrust() && has_docking_port(body.vehicle()) &&
                   !body.vehicle().payload_stacked() &&
                   (!iss_mission_->ascent_to_station || body.vehicle().stages().size() == 1);
        case IssMissionLeg::Raise:  // the burn is flown (the autopilot leaves Burn) or the vehicle is lost
            return autopilot_idle || body.impacted();
        case IssMissionLeg::Rendezvous:
            return rendezvous_step_ == RendezvousStep::Waypoint0 || rendezvous_step_ == RendezvousStep::None ||
                   body.impacted();
        case IssMissionLeg::Proximity:
            return docking_busy() || collision_impulse_ns().has_value() || body.impacted() ||
                   (iss_mission_->max_capture_retries > 0 && proximity_waiting_after_capture_failure());
        case IssMissionLeg::Docking:
            return docked() || !docking_busy() || collision_impulse_ns().has_value();
        case IssMissionLeg::Docked:  // the docking autopilot hands over first (start_transfer refuses it active)
            return iss_mission_->transfer.has_value() && docked() && autopilot_idle;
        case IssMissionLeg::Transfer:
            return transfer_state().phase == TransferPhase::Complete || transfer_state().phase == TransferPhase::Stopped;
        case IssMissionLeg::WaitUndock:  // the epoch, or undocked by hand before it (review i6)
            return epoch_ >= *iss_mission_->undock_epoch || !docked();
        case IssMissionLeg::Departure:
            return departure_phase() == gnc::DeparturePhase::Complete || collision_impulse_ns().has_value() ||
                   body.impacted();
        case IssMissionLeg::Return:
            if (iss_mission_->ship_return) {  // caught or lost - or called off in orbit (review r5)
                return body.caught() || body.impacted() || body.resting() || (!ship_return_ && autopilot_idle);
            }
            return capsule_return_step_ == CapsuleReturnStep::SplashedDown ||
                   capsule_return_step_ == CapsuleReturnStep::Lost;
        case IssMissionLeg::WaitContinue:
            return epoch_ >= *iss_mission_->continue_epoch - 0.5 * tick_s;
        case IssMissionLeg::None:
        case IssMissionLeg::Complete:
        case IssMissionLeg::Ended: return false;
    }
    return false;
}

void FlightSession::run_iss_mission() {
    if (!iss_mission_ || bodies_.empty()) return;
    const auto& q = *iss_mission_;
    // the running leg's status line
    switch (iss_leg_) {
        case IssMissionLeg::Rendezvous:
            iss_status_ = leg_status(iss_leg_, rendezvous_status_);
            break;
        case IssMissionLeg::Proximity: iss_status_ = leg_status(iss_leg_, proximity_status_); break;
        case IssMissionLeg::Docking: iss_status_ = leg_status(iss_leg_, docking_status_); break;
        case IssMissionLeg::Transfer:
            iss_status_ = leg_status(iss_leg_, report::legacy(transfer_state().status));
            break;
        case IssMissionLeg::Departure: iss_status_ = leg_status(iss_leg_, docking_status_); break;
        default: break;
    }
    // Refuelling f3: an unsettled in-orbit ignition lost the engines - whatever leg flew it, the mission ends.
    if (iss_leg_ != IssMissionLeg::None && iss_leg_ != IssMissionLeg::Complete && iss_leg_ != IssMissionLeg::Ended &&
        burn_abort_reason_.rfind("Gasansaugung", 0) == 0) {
        return iss_end(report::legacy(burn_abort_reason_));
    }
    if (!iss_mission_pending()) return;
    const bool lost = bodies_[controlled_].body->impacted();
    switch (iss_leg_) {
        case IssMissionLeg::Ascent:
            if (q.ascent_to_station) {
                make_station(controlled_);
                iss_enter(IssMissionLeg::Complete, completed(report::Code::iss_reason_depot_in_orbit));
            } else if (q.raise_to) {
                iss_start_raise(true);
            } else if (plan_rendezvous(q.phasing)) {
                iss_enter(IssMissionLeg::Rendezvous, leg_status(IssMissionLeg::Rendezvous, rendezvous_status_));
            } else {
                iss_end(rendezvous_status_);
            }
            return;
        case IssMissionLeg::Raise:
            if (lost) {
                iss_end(simple(report::Code::iss_vehicle_lost));
            } else if (!burn_abort_reason_.empty()) {  // a raise burn that ran dry is not flown (r5 minor)
                iss_end(report::legacy(burn_abort_reason_));
            } else if (bodies_[controlled_].body->vehicle().any_engine_producing_thrust()) {
                // f1: wait for the tail-off - a next burn started during it counted the residual thrust as its
                // ignition (the leg stays pending, so the session keeps ticking instead of coasting)
            } else {
                iss_start_raise(false);  // the next apsis burn, or the phasing plan once at the phasing orbit
            }
            return;
        case IssMissionLeg::Rendezvous:
            if (lost) {
                iss_end(simple(report::Code::iss_vehicle_lost));
            } else if (rendezvous_step_ == RendezvousStep::None) {
                iss_end(rendezvous_status_);
            } else if (start_proximity(q.station_port, q.proximity)) {
                iss_enter(IssMissionLeg::Proximity, leg_status(IssMissionLeg::Proximity, proximity_status_));
            } else {
                iss_end(proximity_status_);
            }
            return;
        case IssMissionLeg::Proximity:
        case IssMissionLeg::Docking:
            if (iss_leg_ == IssMissionLeg::Proximity && q.max_capture_retries > 0 &&
                proximity_waiting_after_capture_failure() && !docking_busy() && !lost &&
                !collision_impulse_ns()) {  // waiting at Waypoint 2 after a failed capture: resume or give up (r5)
                if (iss_capture_retries_ >= q.max_capture_retries) {
                    iss_end(report::make(report::Code::iss_docking_failed_after_retries,
                                         {report::count("retries", iss_capture_retries_)}));
                } else if (proximity_resume()) {
                    ++iss_capture_retries_;
                    iss_enter(IssMissionLeg::Proximity,
                              report::make(report::Code::iss_proximity_retry, {report::count("retry", iss_capture_retries_)}));
                } else {
                    iss_end(proximity_status_);
                }
                return;
            }
            if (collision_impulse_ns()) {
                iss_end(simple(report::Code::iss_collision));
            } else if (lost) {
                iss_end(simple(report::Code::iss_vehicle_lost));
            } else if (docked()) {
                iss_enter(IssMissionLeg::Docked,
                          completed(report::Code::iss_reason_docked,
                                    {report::ref("port", "place.port." + q.station_port)}));
            } else if (docking_busy()) {
                iss_enter(IssMissionLeg::Docking, leg_status(IssMissionLeg::Docking, docking_status_));
            } else {  // a failed capture: the proximity flight retreats to Waypoint 2 and waits for resume
                iss_enter(IssMissionLeg::Proximity, leg_status(IssMissionLeg::Proximity, proximity_status_));
            }
            return;
        case IssMissionLeg::Docked:
            iss_start_transfer();
            return;
        case IssMissionLeg::Transfer:
            if (collision_impulse_ns()) {  // r5 minor: a collision stops the transfer; report the collision
                iss_end(simple(report::Code::iss_collision));
            } else if (transfer_state().phase == TransferPhase::Stopped) {
                iss_end(report::legacy(transfer_state().status));
            } else if (undock()) {
                iss_enter(IssMissionLeg::Departure, leg_status(IssMissionLeg::Departure, docking_status_));
            } else {
                iss_end(docking_status_);
            }
            return;
        case IssMissionLeg::WaitUndock:
            if (!docked()) {  // the hooks are opening already: the pilot undocked early
                iss_enter(IssMissionLeg::Departure, leg_status(IssMissionLeg::Departure, docking_status_));
            } else if (undock()) {
                iss_enter(IssMissionLeg::Departure, leg_status(IssMissionLeg::Departure, docking_status_));
            } else {
                iss_end(docking_status_);
            }
            return;
        case IssMissionLeg::Departure: {
            if (collision_impulse_ns()) {
                iss_end(simple(report::Code::iss_collision));
                return;
            }
            if (lost) {
                iss_end(simple(report::Code::iss_vehicle_lost));
                return;
            }
            if (q.ship_return) {
                if (!iss_return_requested_) {
                    release_tower();  // the booster has been lifted off since the launch (no-op without one)
                    // real-time play (fix round 1, I3): planned on a worker while the ship drifts on; else at once
                    begin_ship_return(*q.ship_return, realtime_planning_);
                    iss_return_requested_ = true;
                }
                // the latest request's outcome (the pilot's key R may have replaced this one: its plan is flown)
                const auto outcome = ship_return_outcome();
                if (!outcome) {
                    iss_status_ = simple(report::Code::iss_departure_planning_return);
                    return;
                }
                iss_return_requested_ = false;
                if (*outcome) {
                    iss_enter(IssMissionLeg::Return, simple(report::Code::iss_return_planned));
                } else {
                    iss_end(ship_return_plan() ? report::legacy(ship_return_plan()->reason)
                                               : simple(report::Code::iss_return_not_possible));
                }
                return;
            }
            if (q.continuation) {
                iss_enter(IssMissionLeg::WaitContinue, leg_report(IssMissionLeg::WaitContinue));
                return;
            }
            if (!q.capsule_return) {  // a departure without a return: complete at the exit of the approach ellipsoid
                iss_enter(IssMissionLeg::Complete, completed(report::Code::iss_reason_departed));
                return;
            }
            auto p = *q.capsule_return;
            if (q.deorbit_epoch) p.lead_s = std::max(p.lead_s, *q.deorbit_epoch - epoch_ - iss_return_lead_margin_s);
            if (plan_capsule_return(p)) {
                iss_enter(IssMissionLeg::Return, simple(report::Code::iss_return_planned));
            } else {
                iss_end(deorbit_plan_ ? report::legacy(deorbit_plan_->reason) : simple(report::Code::iss_return_not_possible));
            }
            return;
        }
        case IssMissionLeg::Return:
            if (q.ship_return) {
                if (bodies_[controlled_].body->caught()) {
                    iss_enter(IssMissionLeg::Complete, completed(report::Code::iss_reason_caught));
                } else if (!ship_return_ && !lost && !bodies_[controlled_].body->resting()) {
                    iss_end(burn_abort_reason_.empty() ? simple(report::Code::iss_return_aborted)
                                                   : report::legacy(burn_abort_reason_));
                } else {
                    iss_end(simple(lost ? report::Code::iss_ship_lost : report::Code::iss_ship_not_caught));
                }
                return;
            }
            if (capsule_return_step_ == CapsuleReturnStep::SplashedDown) {
                iss_enter(IssMissionLeg::Complete, completed(report::Code::iss_reason_splashed_down));
            } else {
                iss_end(simple(report::Code::iss_capsule_lost));
            }
            return;
        case IssMissionLeg::WaitContinue: {
            const auto continuation = q.continuation;  // the continuation may replace the sequence
            const auto generation = iss_generation_;
            const std::string refusal = continuation(*this);
            if (!iss_mission_ || iss_generation_ != generation) return;  // ended, or a new sequence runs
            if (refusal.empty()) {
                iss_enter(IssMissionLeg::Complete, simple(report::Code::iss_continued));
            } else {
                iss_end(report::legacy(refusal));
            }
            return;
        }
        default: return;
    }
}

}  // namespace starsim::sim
