// ISS missions (i6): the legs of a mission chained on the session tick - ascent -> phasing -> proximity -> docking,
// or docked -> undocking -> departure -> capsule return. Refuelling r5: docked -> transfer -> undocking -> departure ->
// ship return to the catch or a continuation (the lunar mission); the depot's ascent ends as the station.

#include <algorithm>
#include <string>

#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

const char* to_string(IssMissionLeg leg) {
    switch (leg) {
        case IssMissionLeg::None: return "keine";
        case IssMissionLeg::Ascent: return "Aufstieg";
        case IssMissionLeg::Raise: return "Phasing-Bahn anheben";
        case IssMissionLeg::Rendezvous: return "Phasenflug";
        case IssMissionLeg::Proximity: return "Annäherung";
        case IssMissionLeg::Docking: return "Andocken";
        case IssMissionLeg::Docked: return "Angedockt";
        case IssMissionLeg::Transfer: return "Treibstofftransfer";
        case IssMissionLeg::WaitUndock: return "Warten auf Abkoppeln";
        case IssMissionLeg::Departure: return "Abflug";
        case IssMissionLeg::Return: return "Rückkehr";
        case IssMissionLeg::WaitContinue: return "Warten auf Fortsetzung";
        case IssMissionLeg::Complete: return "Abgeschlossen";
        case IssMissionLeg::Ended: return "Beendet";
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

}  // namespace

void FlightSession::set_iss_mission(std::optional<IssMissionSequence> sequence) {
    iss_mission_ = std::move(sequence);
    iss_status_.clear();
    iss_marks_.clear();
    iss_capture_retries_ = 0;
    if (!iss_mission_) {
        iss_leg_ = IssMissionLeg::None;
        return;
    }
    if (iss_mission_->from_launch) {
        iss_enter(IssMissionLeg::Ascent, "Aufstieg");
    } else if (iss_mission_->transfer && docked()) {
        iss_start_transfer();
    } else if (iss_mission_->undock_epoch) {
        iss_enter(IssMissionLeg::WaitUndock, "Angedockt: Abkoppeln geplant");
    } else {
        iss_enter(IssMissionLeg::Rendezvous, "Phasenflug");
    }
}

void FlightSession::mark_rendezvous_at_waypoint0_for_test() {
    rendezvous_step_ = RendezvousStep::Waypoint0;
    rendezvous_status_ = "Wegpunkt 0 erreicht";
}

void FlightSession::iss_enter(IssMissionLeg leg, std::string status) {
    iss_leg_ = leg;
    iss_status_ = std::move(status);
    if (bodies_.empty() || controlled_ >= bodies_.size()) return;
    const auto& v = bodies_[controlled_].body->vehicle();
    MissionMark mark{leg, epoch_, v.mass_properties().mass_kg, v.main_propellant_kg(), v.rcs_supply_kg(), 0.0};
    if (station_index_) mark.station_main_propellant_kg = bodies_[*station_index_].body->vehicle().main_propellant_kg();
    iss_marks_.push_back(mark);
}

void FlightSession::iss_end(const std::string& reason) { iss_enter(IssMissionLeg::Ended, "Mission beendet: " + reason); }

void FlightSession::iss_start_transfer() {
    const auto& q = *iss_mission_;
    std::optional<TransferRequest> request;
    std::string reason;
    if (*q.transfer == MissionTransfer::TankerReserve) {
        request = tanker_transfer_request({}, &reason);
    } else {  // the chaser's main tanks up to full; the station down to empty
        TransferRequest r;
        r.direction = TransferDirection::StationToChaser;
        const auto& v = bodies_[controlled_].body->vehicle();
        r.limit_kg = {main_free_kg(v, config::TankContents::Oxidizer), main_free_kg(v, config::TankContents::Fuel)};
        request = r;
    }
    if (!request) return iss_end(reason);
    if (!start_transfer(*request)) {
        return iss_end(transfer_ && transfer_->active() ? docking_status_ : transfer_state().status);
    }
    iss_enter(IssMissionLeg::Transfer, "Treibstofftransfer");
}

void FlightSession::iss_start_raise(bool first) {
    const auto& s = bodies_[controlled_].body->state();
    const math::StateVector state{s.position_gcrf_m, s.velocity_gcrf_mps};
    const auto& to = *iss_mission_->raise_to;
    // The burn sequence is fixed at the start (refuelling f1): the osculating apsides swing by several km with J2, so a
    // re-check after each burn would keep adding burns. First: the apoapsis raise if needed (then the periapsis raise
    // at the new apoapsis follows), else the periapsis raise alone.
    std::optional<gnc::BurnNode> node;
    if (first) {
        node = gnc::next_raise_node(state, epoch_, gm_earth_, to.periapsis_radius_m, to.apoapsis_radius_m,
                                    mission::raise_tolerance_m);
        const auto el = orbit::elements_from_state(state, gm_earth_);
        raise_periapsis_pending_ =
            node && to.apoapsis_radius_m > el.semi_major_axis_m * (1.0 + el.eccentricity) + mission::raise_tolerance_m;
    } else if (raise_periapsis_pending_) {
        raise_periapsis_pending_ = false;
        node = gnc::set_periapsis_at_apoapsis(state, epoch_, gm_earth_, to.periapsis_radius_m);
    }
    if (!node) {  // at (or above) the phasing orbit
        if (plan_rendezvous(iss_mission_->phasing)) {
            iss_enter(IssMissionLeg::Rendezvous, "Phasenflug: " + rendezvous_status_);
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
    if (!started) return iss_end("Phasing-Bahn anheben nicht möglich");
    iss_enter(IssMissionLeg::Raise, "Phasing-Bahn anheben");
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
        case IssMissionLeg::Rendezvous: iss_status_ = "Phasenflug: " + rendezvous_status_; break;
        case IssMissionLeg::Proximity: iss_status_ = "Annäherung: " + proximity_status_; break;
        case IssMissionLeg::Docking: iss_status_ = "Andocken: " + docking_status_; break;
        case IssMissionLeg::Transfer: iss_status_ = "Treibstofftransfer: " + transfer_state().status; break;
        case IssMissionLeg::Departure: iss_status_ = "Abflug: " + docking_status_; break;
        default: break;
    }
    if (!iss_mission_pending()) return;
    const bool lost = bodies_[controlled_].body->impacted();
    switch (iss_leg_) {
        case IssMissionLeg::Ascent:
            if (q.ascent_to_station) {
                make_station(controlled_);
                iss_enter(IssMissionLeg::Complete, "Mission abgeschlossen: Depot im Orbit");
            } else if (q.raise_to) {
                iss_start_raise(true);
            } else if (plan_rendezvous(q.phasing)) {
                iss_enter(IssMissionLeg::Rendezvous, "Phasenflug: " + rendezvous_status_);
            } else {
                iss_end(rendezvous_status_);
            }
            return;
        case IssMissionLeg::Raise:
            if (lost) {
                iss_end("Fahrzeug verloren");
            } else if (!burn_abort_reason_.empty()) {  // a raise burn that ran dry is not flown (r5 minor)
                iss_end(burn_abort_reason_);
            } else if (bodies_[controlled_].body->vehicle().any_engine_producing_thrust()) {
                // f1: wait for the tail-off - a next burn started during it counted the residual thrust as its
                // ignition (the leg stays pending, so the session keeps ticking instead of coasting)
            } else {
                iss_start_raise(false);  // the next apsis burn, or the phasing plan once at the phasing orbit
            }
            return;
        case IssMissionLeg::Rendezvous:
            if (lost) {
                iss_end("Fahrzeug verloren");
            } else if (rendezvous_step_ == RendezvousStep::None) {
                iss_end(rendezvous_status_);
            } else if (start_proximity(q.station_port, q.proximity)) {
                iss_enter(IssMissionLeg::Proximity, "Annäherung: " + proximity_status_);
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
                    iss_end("Andocken fehlgeschlagen nach " + std::to_string(iss_capture_retries_) + " Neuversuchen");
                } else if (proximity_resume()) {
                    ++iss_capture_retries_;
                    iss_enter(IssMissionLeg::Proximity, "Annäherung: neuer Anflug " + std::to_string(iss_capture_retries_));
                } else {
                    iss_end(proximity_status_);
                }
                return;
            }
            if (collision_impulse_ns()) {
                iss_end("Kollision");
            } else if (lost) {
                iss_end("Fahrzeug verloren");
            } else if (docked()) {
                iss_enter(IssMissionLeg::Docked, "Mission abgeschlossen: angedockt an " + q.station_port);
            } else if (docking_busy()) {
                iss_enter(IssMissionLeg::Docking, "Andocken: " + docking_status_);
            } else {  // a failed capture: the proximity flight retreats to Waypoint 2 and waits for resume
                iss_enter(IssMissionLeg::Proximity, "Annäherung: " + proximity_status_);
            }
            return;
        case IssMissionLeg::Docked:
            iss_start_transfer();
            return;
        case IssMissionLeg::Transfer:
            if (transfer_state().phase == TransferPhase::Stopped) {
                iss_end(transfer_state().status);
            } else if (undock()) {
                iss_enter(IssMissionLeg::Departure, "Abflug: " + docking_status_);
            } else {
                iss_end(docking_status_);
            }
            return;
        case IssMissionLeg::WaitUndock:
            if (!docked()) {  // the hooks are opening already: the pilot undocked early
                iss_enter(IssMissionLeg::Departure, "Abflug: " + docking_status_);
            } else if (undock()) {
                iss_enter(IssMissionLeg::Departure, "Abflug: " + docking_status_);
            } else {
                iss_end(docking_status_);
            }
            return;
        case IssMissionLeg::Departure: {
            if (collision_impulse_ns()) {
                iss_end("Kollision");
                return;
            }
            if (lost) {
                iss_end("Fahrzeug verloren");
                return;
            }
            if (q.ship_return) {
                release_tower();  // the booster has been lifted off since the launch (no-op without one)
                if (plan_ship_return(*q.ship_return)) {
                    iss_enter(IssMissionLeg::Return, "Rückkehr geplant");
                } else {
                    iss_end(ship_return_plan() ? ship_return_plan()->reason : std::string("Rückkehr nicht möglich"));
                }
                return;
            }
            if (q.continuation) {
                iss_enter(IssMissionLeg::WaitContinue, "Warten auf Fortsetzung");
                return;
            }
            if (!q.capsule_return) {  // a departure without a return: complete at the exit of the approach ellipsoid
                iss_enter(IssMissionLeg::Complete, "Mission abgeschlossen: abgeflogen");
                return;
            }
            auto p = *q.capsule_return;
            if (q.deorbit_epoch) p.lead_s = std::max(p.lead_s, *q.deorbit_epoch - epoch_ - iss_return_lead_margin_s);
            if (plan_capsule_return(p)) {
                iss_enter(IssMissionLeg::Return, "Rückkehr geplant");
            } else {
                iss_end(deorbit_plan_ ? deorbit_plan_->reason : std::string("Rückkehr nicht möglich"));
            }
            return;
        }
        case IssMissionLeg::Return:
            if (q.ship_return) {
                if (bodies_[controlled_].body->caught()) {
                    iss_enter(IssMissionLeg::Complete, "Mission abgeschlossen: gefangen");
                } else if (!ship_return_ && !lost && !bodies_[controlled_].body->resting()) {
                    iss_end(burn_abort_reason_.empty() ? std::string("Rückkehr abgebrochen") : burn_abort_reason_);
                } else {
                    iss_end(lost ? "Schiff verloren" : "Schiff nicht gefangen");
                }
                return;
            }
            if (capsule_return_step_ == CapsuleReturnStep::SplashedDown) {
                iss_enter(IssMissionLeg::Complete, "Mission abgeschlossen: gewassert");
            } else {
                iss_end("Kapsel verloren");
            }
            return;
        case IssMissionLeg::WaitContinue: {
            const auto continuation = q.continuation;  // the continuation may replace the sequence
            const std::string refusal = continuation(*this);
            if (!iss_mission_) return;
            if (refusal.empty()) {
                iss_enter(IssMissionLeg::Complete, "Mission fortgesetzt");
            } else {
                iss_end(refusal);
            }
            return;
        }
        default: return;
    }
}

}  // namespace starsim::sim
