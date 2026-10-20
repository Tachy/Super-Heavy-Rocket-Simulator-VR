// ISS missions (i6): the legs of a mission chained on the session tick - ascent -> phasing -> proximity -> docking,
// or docked -> undocking -> departure -> capsule return.

#include <algorithm>

#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

const char* to_string(IssMissionLeg leg) {
    switch (leg) {
        case IssMissionLeg::None: return "keine";
        case IssMissionLeg::Ascent: return "Aufstieg";
        case IssMissionLeg::Rendezvous: return "Phasenflug";
        case IssMissionLeg::Proximity: return "Annäherung";
        case IssMissionLeg::Docking: return "Andocken";
        case IssMissionLeg::Docked: return "Angedockt";
        case IssMissionLeg::WaitUndock: return "Warten auf Abkoppeln";
        case IssMissionLeg::Departure: return "Abflug";
        case IssMissionLeg::Return: return "Rückkehr";
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

}  // namespace

void FlightSession::set_iss_mission(std::optional<IssMissionSequence> sequence) {
    iss_mission_ = std::move(sequence);
    iss_status_.clear();
    iss_marks_.clear();
    if (!iss_mission_) {
        iss_leg_ = IssMissionLeg::None;
        return;
    }
    if (iss_mission_->from_launch) {
        iss_enter(IssMissionLeg::Ascent, "Aufstieg");
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

bool FlightSession::iss_mission_pending() const {
    if (!iss_mission_ || bodies_.empty() || controlled_ >= bodies_.size()) return false;
    const auto& body = *bodies_[controlled_].body;
    switch (iss_leg_) {
        case IssMissionLeg::Ascent:
            return (autopilot_.state == AutopilotState::Off || autopilot_.state == AutopilotState::Orbit) &&
                   has_docking_port(body.vehicle()) && !body.vehicle().payload_stacked();
        case IssMissionLeg::Rendezvous:
            return rendezvous_step_ == RendezvousStep::Waypoint0 || rendezvous_step_ == RendezvousStep::None ||
                   body.impacted();
        case IssMissionLeg::Proximity:
            return docking_busy() || collision_impulse_ns().has_value() || body.impacted();
        case IssMissionLeg::Docking:
            return docked() || !docking_busy() || collision_impulse_ns().has_value();
        case IssMissionLeg::WaitUndock:  // the epoch, or undocked by hand before it (review i6)
            return epoch_ >= *iss_mission_->undock_epoch || !docked();
        case IssMissionLeg::Departure:
            return departure_phase() == gnc::DeparturePhase::Complete || collision_impulse_ns().has_value() ||
                   body.impacted();
        case IssMissionLeg::Return:
            return capsule_return_step_ == CapsuleReturnStep::SplashedDown ||
                   capsule_return_step_ == CapsuleReturnStep::Lost;
        case IssMissionLeg::None:
        case IssMissionLeg::Docked:
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
        case IssMissionLeg::Departure: iss_status_ = "Abflug: " + docking_status_; break;
        default: break;
    }
    if (!iss_mission_pending()) return;
    const bool lost = bodies_[controlled_].body->impacted();
    switch (iss_leg_) {
        case IssMissionLeg::Ascent:
            if (plan_rendezvous(q.phasing)) {
                iss_enter(IssMissionLeg::Rendezvous, "Phasenflug: " + rendezvous_status_);
            } else {
                iss_end(rendezvous_status_);
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
            if (capsule_return_step_ == CapsuleReturnStep::SplashedDown) {
                iss_enter(IssMissionLeg::Complete, "Mission abgeschlossen: gewassert");
            } else {
                iss_end("Kapsel verloren");
            }
            return;
        default: return;
    }
}

}  // namespace starsim::sim
