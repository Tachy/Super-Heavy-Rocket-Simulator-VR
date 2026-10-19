// Proximity operations of the controlled vehicle at the station (ISS i4): from Waypoint 0 along the waypoint pattern
// to contact at the station port (gnc::ProximityGuidance) on the RCS translation in the target port frame.
//
// Design decisions:
// - The docking attitude (chaser port axes opposed to the target port's, ups equal) is held as an LVLH hold of the
//   controlled body, re-derived every tick from the live port kinematics: the station holds its ideal LVLH attitude,
//   so the hold's body_in_lvlh stays nearly constant, and the re-derivation absorbs the small LVLH difference between
//   the two bodies and any station attitude change.
// - The translation authority along the target port axes is measured once at the start, in the docking attitude: that
//   attitude is held in the port frame, so the body directions of the port axes stay fixed (the falling mass of the
//   RCS propellant changes it by < 1 % over the flight).
// - The guidance runs in the autopilot dispatch (run_autopilot), which FlightSession::advance_to calls before the
//   vehicle command of the same tick is built (vehicle_command -> apply_translation): its translation fraction and
//   attitude hold are flown in the tick that computed them.
// - After a completed breakout the guidance holds no attitude: the session drops the attitude mode and may coast (the
//   abort is passive until resume, which sets the hold again at once so that the next ticks run the guidance).

#include "starsim/sim/flight_session.hpp"

#include <cmath>
#include <stdexcept>

#include "starsim/frames/lvlh.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "flight_session_lunar_detail.hpp"

namespace starsim::sim {

using detail::german;

namespace {

// The target port frame axes in the station LVLH frame (columns).
math::Mat3 port_axes_lvlh(const math::StateVector& station, const vehicle::PortKinematics& target) {
    return transpose(frames::lvlh_to_inertial(station).to_matrix()) * target.axes_gcrf;
}

}  // namespace

bool FlightSession::start_proximity(const std::string& station_port, gnc::ProximityParameters p) {
    const auto refuse = [&](const std::string& reason) {
        proximity_status_ = reason;
        return false;
    };
    if (bodies_.empty() || !station_index_) return refuse("Kein Anflug: keine Station");
    if (controlled_ == *station_index_) return refuse("Kein Anflug: die Station selbst wird gesteuert");
    if (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit) {
        return refuse("Kein Anflug: Autopilot aktiv");
    }
    if ((rendezvous_step_ != RendezvousStep::None && rendezvous_step_ != RendezvousStep::Waypoint0) ||
        lunar_step_ != LunarStep::None) {
        return refuse("Kein Anflug: ein Flug läuft bereits");
    }
    auto& chaser = *bodies_[controlled_].body;
    auto& vehicle = chaser.vehicle();
    // chaser port: the vehicle's first docking port (Crew Dragon: idss)
    std::string chaser_port;
    for (const auto& stage : vehicle.stages()) {
        if (!stage.config.docking_ports.empty()) {
            chaser_port = stage.config.docking_ports.front().id;
            break;
        }
    }
    if (chaser_port.empty()) return refuse("Kein Anflug: kein Andockstutzen");
    const auto& station = *bodies_[*station_index_].body;
    vehicle::PortKinematics target;
    try {
        target = vehicle::port_kinematics(station, station_port);
    } catch (const std::invalid_argument&) {
        return refuse("Kein Anflug: unbekannter Port " + station_port);
    }
    const auto st = station_state();
    const math::Mat3 port_to_lvlh = port_axes_lvlh(st, target);
    gnc::DockingApproach approach;
    try {
        approach = gnc::docking_approach(port_to_lvlh * math::Vec3{1.0, 0.0, 0.0});
    } catch (const std::invalid_argument& e) {
        return refuse(std::string("Kein Anflug: ") + e.what());
    }
    const auto cg = orbit::lvlh_relative_state(st, controlled_state());
    const double range = norm(cg.position_m);
    if (range > p.max_start_range_m) return refuse("Kein Anflug: zu weit von der Station");
    if (range < p.keep_out_radius_m) {
        // only an on-axis start inside the corridor (tests: Waypoint 2)
        const auto port = gnc::relative_port_state(vehicle::port_kinematics(chaser, chaser_port), target);
        const auto path = gnc::proximity_path(approach, p);
        if (p.start_waypoint >= path.size() || !path[p.start_waypoint].on_axis ||
            gnc::corridor_status(port.position_m, range, p) != gnc::CorridorStatus::Inside) {
            return refuse("Kein Anflug: innerhalb der Sicherheitszone");
        }
    }
    if (vehicle.has_nose_cone()) vehicle.set_nose_cone_open(true);
    set_rendezvous_target(*station_index_, chaser_port, station_port);
    proximity_station_port_ = station_port;
    proximity_chaser_port_ = chaser_port;
    // authority along the target port axes in the docking attitude (constant: the attitude is held in the port frame)
    const math::Quat docking = proximity_docking_attitude();
    const double mass = vehicle.mass_properties().mass_kg;
    for (int axis = 0; axis < 3; ++axis) {
        for (const double sign : {1.0, -1.0}) {
            math::Vec3 u{};
            (axis == 0 ? u.x : axis == 1 ? u.y : u.z) = sign;
            const auto c = vehicle.rcs_pure_force_capability(docking.inverse_rotate(target.axes_gcrf * u));
            proximity_authority_.mps2[2 * axis + (sign < 0.0 ? 1 : 0)] = c.force_n / mass;
        }
    }
    proximity_warnings_.clear();
    for (const auto& impulse : station_impulses_) {
        if (impulse.epoch >= epoch_ && impulse.epoch <= epoch_ + proximity_warning_span_s) {
            proximity_warnings_.push_back("Achtung: Bahnanhebung der ISS um " + german(impulse.delta_v_mps, 2) +
                                          " m/s während des Anflugs");
        }
    }
    rendezvous_step_ = RendezvousStep::None;  // the phasing (i3) is complete
    proximity_.emplace(approach, p);
    autopilot_.state = AutopilotState::Proximity;
    attitude_mode_ = proximity_attitude_mode();
    proximity_status_ = "Anflug gestartet";
    return true;
}

math::Quat FlightSession::proximity_docking_attitude() const {
    const auto& chaser = *bodies_[controlled_].body;
    const auto chaser_kin = vehicle::port_kinematics(chaser, proximity_chaser_port_);
    const math::Mat3 chaser_axes_body = transpose(chaser.state().attitude.to_matrix()) * chaser_kin.axes_gcrf;
    const auto target = vehicle::port_kinematics(*bodies_[*station_index_].body, proximity_station_port_);
    return gnc::docking_attitude_gcrf(chaser_axes_body, target.axes_gcrf);
}

gnc::AttitudeMode FlightSession::proximity_attitude_mode() const {
    const auto& s = bodies_[controlled_].body->state();
    const math::Quat lvlh = gnc::lvlh_attitude_gcrf({s.position_gcrf_m, s.velocity_gcrf_mps});
    return gnc::lvlh_hold(lvlh.conjugate() * proximity_docking_attitude());
}

gnc::ProximityInput FlightSession::proximity_input() const {
    gnc::ProximityInput in;
    const auto st = station_state();
    in.epoch = epoch_;
    in.cg_lvlh = orbit::lvlh_relative_state(st, controlled_state());
    in.port = *relative_port_state();
    in.port_to_lvlh = port_axes_lvlh(st, vehicle::port_kinematics(*bodies_[*station_index_].body, proximity_station_port_));
    const double r = norm(st.position_m);
    in.mean_motion_radps = std::sqrt(constants::earth_gm_m3ps2 / (r * r * r));
    in.authority_port = proximity_authority_;
    in.propellant_kg = bodies_[controlled_].body->vehicle().rcs_propellant_kg();
    return in;
}

void FlightSession::run_proximity(Flying& flying) {
    if (!proximity_) {
        autopilot_.state = AutopilotState::Off;
        return;
    }
    const auto end = [&](const std::string& status) {
        reset_proximity();
        attitude_mode_ = {};
        proximity_status_ = status;
    };
    if (!station_index_ || !rendezvous_) return end("Anflug abgebrochen: keine Station");
    if (flying.body->impacted()) return end("Anflug abgebrochen: Fahrzeug verloren");
    const auto out = proximity_->update(proximity_input());
    proximity_status_ = proximity_->status();
    if (proximity_->phase() == gnc::ProximityPhase::Contact) {
        translation_port_ = {};
        attitude_mode_ = {};
        autopilot_.state = AutopilotState::Off;  // the flight stays readable until cleared
        return;
    }
    translation_port_ = out.translation_fraction_port;
    attitude_mode_ = out.attitude_hold ? proximity_attitude_mode() : gnc::AttitudeMode{};
}

void FlightSession::reset_proximity() {
    if (!proximity_) return;
    proximity_.reset();
    translation_port_ = {};
    if (autopilot_.state == AutopilotState::Proximity) {
        autopilot_.state = AutopilotState::Off;
        attitude_mode_ = {};
    }
}

std::optional<gnc::ProximityPhase> FlightSession::proximity_phase() const {
    if (!proximity_) return std::nullopt;
    return proximity_->phase();
}

const std::vector<gnc::ProximityEvent>* FlightSession::proximity_events() const {
    return proximity_ ? &proximity_->events() : nullptr;
}

std::optional<gnc::ProximityContact> FlightSession::proximity_contact() const {
    if (!proximity_) return std::nullopt;
    return proximity_->contact();
}

bool FlightSession::proximity_hold() {
    if (!proximity_) {
        proximity_status_ = "Kein Anflug aktiv";
        return false;
    }
    const bool accepted = proximity_->hold();
    proximity_status_ = proximity_->status();
    return accepted;
}

bool FlightSession::proximity_retreat() {
    if (!proximity_) {
        proximity_status_ = "Kein Anflug aktiv";
        return false;
    }
    const bool accepted = proximity_->retreat();
    proximity_status_ = proximity_->status();
    return accepted;
}

bool FlightSession::proximity_abort() {
    if (!proximity_) {
        proximity_status_ = "Kein Anflug aktiv";
        return false;
    }
    const bool accepted = proximity_->abort();
    proximity_status_ = proximity_->status();
    return accepted;
}

bool FlightSession::proximity_resume() {
    if (!proximity_) {
        proximity_status_ = "Kein Anflug aktiv";
        return false;
    }
    const bool accepted = proximity_->resume();
    proximity_status_ = proximity_->status();
    if (accepted) {
        if (autopilot_.state == AutopilotState::Off) autopilot_.state = AutopilotState::Proximity;
        attitude_mode_ = proximity_attitude_mode();  // the passive abort coasts: the hold makes the next ticks run
    }
    return accepted;
}

}  // namespace starsim::sim
