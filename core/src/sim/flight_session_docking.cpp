// Docking of the controlled vehicle at the station (ISS i5): the docking mechanism (sim::DockingSim: IDSS, or the four
// probes into four drogues of the refuelling ships, refuelling r3) on the session tick,
// the merge into the compound at hard capture, the docked hold, and the failure paths (failed capture, collision).
//
// Design decisions:
// - The mechanism is armed with the rendezvous target (set_rendezvous_target) when the chaser port is active and the
//   station port passive; it runs every tick from the first contact (and whenever the chaser is near enough for a hull
//   contact), before the bodies step - its wrench acts in the step that follows.
// - The compound (spec correction 4): the station body carries the summed mass (its stage's extra mass) and the
//   momentum-weighted velocity; the chaser is held on it at the mated pose (the snap is below the guide-pin tolerance)
//   and follows the station after every tick and coast. The compound CG offset from the station CG (~1 m for a Dragon)
//   and the Dragon's drag area are not modelled.
// - A failed capture hands the proximity flight a retreat to Waypoint 2 (it waits there for resume); the mechanism
//   re-arms once the chaser is back beyond the petal tips. A hull contact ends the flight: "Kollision".

#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/station_limits.hpp"

#include <cstdio>
#include <limits>
#include <stdexcept>

#include "starsim/contact/probe_drogue_mechanism.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/docking_port.hpp"

namespace starsim::sim {

namespace {

const config::DockingPortConfig* find_port(const vehicle::Vehicle& v, const std::string& id) {
    for (const auto& stage : v.stages()) {
        for (const auto& port : stage.config.docking_ports) {
            if (port.id == id) return &port;
        }
    }
    return nullptr;
}

config::VehicleConfig stages_of(const vehicle::Vehicle& v) {  // the stage geometry the docking sim needs
    config::VehicleConfig c;
    for (const auto& stage : v.stages()) c.stages.push_back(stage.config);
    return c;
}

bool capturing(contact::DockingPhase p) {
    using P = contact::DockingPhase;
    return p == P::Contact || p == P::SoftCapture || p == P::Retraction || p == P::HooksClosing ||
           p == P::HooksOpening;
}

math::Mat3 diagonal(const math::Vec3& d) { return math::Mat3::from_columns({d.x, 0, 0}, {0, d.y, 0}, {0, 0, d.z}); }

}  // namespace

void FlightSession::arm_docking() {
    if (!rendezvous_ || !station_index_ || rendezvous_->target_index != *station_index_) return;
    const auto& chaser = bodies_[controlled_].body->vehicle();
    const auto& station = bodies_[*station_index_].body->vehicle();
    const auto* cp = find_port(chaser, rendezvous_->chaser_port);
    const auto* sp = find_port(station, rendezvous_->target_port);
    // Active chaser port against a passive station port of the same mechanism (vehicle::port_mismatch): IDSS (ISS i5)
    // or the four probes into four drogues (refuelling r3).
    if (!cp || !sp || cp->mechanism != sp->mechanism || cp->soft_capture != config::SoftCaptureMode::Active ||
        sp->soft_capture != config::SoftCaptureMode::Passive) {
        return;
    }
    const auto parameters = cp->mechanism == config::DockingMechanism::ProbeDrogue
                                ? contact::probe_drogue_mechanism(*cp, *sp)
                                : contact::DockingMechanismParameters{};
    docking_.emplace(rendezvous_->chaser_port, rendezvous_->target_port, stages_of(chaser), stages_of(station),
                     parameters);
    docking_status_ = docking_->mechanism().status();
}

void FlightSession::reset_docking() {
    if (docking_ && station_index_ && controlled_ < bodies_.size() && *station_index_ < bodies_.size() &&
        bodies_[controlled_].body->docked()) {
        bodies_[controlled_].body->release_docked();  // released in place, no separation push
        detach_compound();
    }
    end_transfer();  // review r4: also when the bodies were cleared first (a new scenario) - no stale step epoch
    reset_docked_update();
    docking_.reset();
    compound_mass_.reset();
    docked_mass_kg_ = 0.0;
    docking_seen_phase_.reset();
    docking_collision_reported_ = false;
    docking_debug_events_ = 0;
}

std::optional<contact::DockingPhase> FlightSession::docking_phase() const {
    if (!docking_) return std::nullopt;
    return docking_->mechanism().phase();
}

const std::vector<contact::DockingEvent>* FlightSession::docking_events() const {
    return docking_ ? &docking_->mechanism().events() : nullptr;
}

std::optional<contact::LoadPeaks> FlightSession::docking_loads() const {
    if (!docking_) return std::nullopt;
    return docking_->mechanism().peaks();
}

bool FlightSession::docked() const {
    return docking_ && controlled_ < bodies_.size() && bodies_[controlled_].body->docked();
}

bool FlightSession::docking_busy() const {
    if (!docking_) return false;
    const auto p = docking_->mechanism().phase();
    return docked() || capturing(p) || p == contact::DockingPhase::Docked;
}

std::optional<double> FlightSession::collision_impulse_ns() const {
    if (!docking_) return std::nullopt;
    return docking_->collision_impulse_ns();
}

math::Vec3 FlightSession::docking_wrench_balance_n() const {
    return docking_ ? docking_->wrench_balance_n() : math::Vec3{};
}

bool FlightSession::docking_free_drift() const {
    if (!docking_) return false;
    using P = contact::DockingPhase;
    const auto p = docking_->mechanism().phase();
    return p == P::Contact || p == P::SoftCapture || p == P::Retraction || p == P::HooksClosing;
}

bool FlightSession::docking_ticked() const {
    if (!docking_ || !station_index_ || controlled_ >= bodies_.size() || *station_index_ >= bodies_.size()) return false;
    return capturing(docking_->mechanism().phase()) || docking_->hull_contact();
}

double FlightSession::docking_coast_limit_s() const {
    if (!docking_ || !station_index_ || controlled_ >= bodies_.size() || *station_index_ >= bodies_.size()) {
        return std::numeric_limits<double>::infinity();
    }
    return docking_->coast_limit_s(*bodies_[controlled_].body, *bodies_[*station_index_].body);
}

void FlightSession::docking_before_tick() {
    if (!station_index_ || controlled_ >= bodies_.size() || controlled_ == *station_index_) return;
    docking_->before_tick(*bodies_[controlled_].body, *bodies_[*station_index_].body, epoch_, tick_s);
}

void FlightSession::docking_after_tick() {
    if (!station_index_ || controlled_ >= bodies_.size()) return;
    auto& m = docking_->mechanism();
    const auto phase = m.phase();
    if (!docking_collision_reported_ && docking_->hull_contact()) {
        docking_collision_reported_ = true;
        docking_status_ = "Kollision";
        if (proximity_) {
            proximity_->collided(epoch_);
            proximity_status_ = "Kollision";
        }
        if (autopilot_.state == AutopilotState::Proximity || autopilot_.state == AutopilotState::Departure) {
            autopilot_.state = AutopilotState::Off;  // the flight ends (review i5: also the departure)
        }
        attitude_mode_ = {};
        translation_port_ = {};
    }
    if (phase != docking_seen_phase_) {
        docking_seen_phase_ = phase;
        if (!docking_collision_reported_) docking_status_ = m.status();
        if (phase == contact::DockingPhase::Docked) merge_docked();
        if (phase == contact::DockingPhase::Separated) separate_docked();
        if (phase == contact::DockingPhase::Failed && proximity_ && !docking_collision_reported_) {
            proximity_->capture_failed(proximity_input(), m.failure());
            proximity_status_ = proximity_->status();
            autopilot_.state = AutopilotState::Proximity;
            attitude_mode_ = proximity_attitude_mode();
        }
    }
    if (phase == contact::DockingPhase::Failed) {  // re-arm once the chaser is back beyond the first-contact plane
        const auto pose = docking_->pose(*bodies_[controlled_].body, *bodies_[*station_index_].body);
        if (pose.position_m.x > m.first_contact_m() + m.parameters().rearm_distance_m) m.rearm();
    }
    sync_docked();
    if (docking_debug_) docking_diagnostics();
}

FlightSession::MatedPose FlightSession::mated_pose(const vehicle::FlightBody& chaser,
                                                  const vehicle::FlightBody& station) const {
    // chaser port on the station port, its axes (-x, -y, +z) of the station port's
    const auto& cs = chaser.state();
    const auto ck = vehicle::port_kinematics(chaser, docking_->chaser_port());
    const auto sk = vehicle::port_kinematics(station, docking_->station_port());
    const math::Mat3 rc = cs.attitude.to_matrix();
    const math::Mat3 port_axes_body = transpose(rc) * ck.axes_gcrf;
    const math::Vec3 lever_body = transpose(rc) * (ck.position_gcrf_m - cs.position_gcrf_m);
    const math::Mat3 mated = sk.axes_gcrf * math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});
    const math::Mat3 rc_mated = mated * transpose(port_axes_body);
    return {math::Quat::from_matrix(rc_mated), sk.position_gcrf_m - rc_mated * lever_body};
}

void FlightSession::dock_in_place() {
    if (!docking_ || !station_index_) throw std::logic_error("dock_in_place: no armed docking mechanism");
    if (docking_->mechanism().phase() != contact::DockingPhase::Ready) {
        throw std::logic_error("dock_in_place: the mechanism is not ready");
    }
    auto& chaser = *bodies_[controlled_].body;
    const auto& station = *bodies_[*station_index_].body;
    const auto pose = mated_pose(chaser, station);
    const auto& ss = station.state();
    const math::Vec3 w = ss.attitude.rotate(ss.angular_velocity_body_radps);
    auto cs = chaser.state();
    cs.attitude = pose.attitude;
    cs.position_gcrf_m = pose.cg_gcrf_m;
    cs.velocity_gcrf_mps = ss.velocity_gcrf_mps + cross(w, pose.cg_gcrf_m - ss.position_gcrf_m);
    cs.angular_velocity_body_radps = pose.attitude.conjugate().rotate(w);
    chaser.set_state(cs);
    docking_->start_docked(epoch_);
    docking_seen_phase_ = contact::DockingPhase::Docked;
    docking_status_ = docking_->mechanism().status();
    merge_docked();
}

void FlightSession::merge_docked() {
    reset_docked_update();
    auto& chaser = *bodies_[controlled_].body;
    auto& station = *bodies_[*station_index_].body;
    const double mc = chaser.vehicle().mass_properties().mass_kg;
    const auto mps = station.vehicle().mass_properties();  // the station alone
    const double ms = mps.mass_kg;
    auto cs = chaser.state();
    auto ss = station.state();
    docking_merge_.momentum_before = mc * cs.velocity_gcrf_mps + ms * ss.velocity_gcrf_mps;
    const auto pose = mated_pose(chaser, station);
    const math::Mat3 rc_mated = pose.attitude.to_matrix();
    const math::Vec3 cg_mated = pose.cg_gcrf_m;
    const math::Mat3 rs = ss.attitude.to_matrix();
    docked_attitude_station_ = ss.attitude.conjugate() * pose.attitude;
    // the compound: the chaser's mass attached at its CG (station vehicle frame), the body state moved to the compound
    // CG, the momentum-weighted velocity
    const math::Vec3 cg_old = station.vehicle().mass_properties().cg_point_m();
    station.vehicle().set_attached_mass(
        vehicle::Vehicle::AttachedMass{mc, cg_old + transpose(rs) * (cg_mated - ss.position_gcrf_m)});
    const math::Vec3 cg_new = station.vehicle().mass_properties().cg_point_m();
    ss.position_gcrf_m += rs * (cg_new - cg_old);
    ss.velocity_gcrf_mps = docking_merge_.momentum_before / (mc + ms);
    station.set_state(ss);
    docked_position_station_m_ = transpose(rs) * (cg_mated - ss.position_gcrf_m);
    docked_mass_kg_ = mc;
    docking_merge_.momentum_after = (mc + ms) * ss.velocity_gcrf_mps;
    compound_cg_point_m_ = cg_new;
    docked_chaser_cg_point_m_ = chaser.vehicle().mass_properties().cg_point_m();
    update_compound_mass();
    chaser.hold_docked();
    sync_docked();
    // the flight ends docked
    reset_proximity();
    proximity_status_ = "angedockt";
    if (autopilot_.state == AutopilotState::Proximity) autopilot_.state = AutopilotState::Off;
    attitude_mode_ = {};
    translation_port_ = {};
}

void FlightSession::update_compound_mass() {
    const auto& chaser = bodies_[controlled_].body->vehicle();
    auto& station = bodies_[*station_index_].body->vehicle();
    const auto attached = station.attached_mass();
    station.set_attached_mass(std::nullopt);
    const auto mps = station.mass_properties();  // the station alone
    station.set_attached_mass(attached);
    const auto mpc = chaser.mass_properties();
    const math::Mat3 rel = docked_attitude_station_.to_matrix();
    // about the compound CG (station body frame): the station CG sits at its own CG point minus the compound's
    compound_mass_ = vehicle::combine_mass(mps.mass_kg, mps.cg_point_m() - compound_cg_point_m_,
                                           diagonal(mps.inertia_kg_m2) /* principal axes [E] */, mpc.mass_kg,
                                           docked_position_station_m_,
                                           rel * diagonal(mpc.inertia_kg_m2) * transpose(rel));
}

void FlightSession::refresh_compound() {
    auto& chaser = *bodies_[controlled_].body;
    auto& station = *bodies_[*station_index_].body;
    auto attached = *station.vehicle().attached_mass();
    // the chaser's CG moves in its own hull: carry that shift into the station frame (the pair's relative attitude)
    const math::Vec3 chaser_cg = chaser.vehicle().mass_properties().cg_point_m();
    attached.position_m += docked_attitude_station_.rotate(chaser_cg - docked_chaser_cg_point_m_);
    attached.mass_kg = chaser.vehicle().mass_properties().mass_kg;
    station.vehicle().set_attached_mass(attached);
    // the station body's state is the compound CG: move it with the new CG point, hulls fixed, velocity kept
    const math::Vec3 cg_new = station.vehicle().mass_properties().cg_point_m();
    auto ss = station.state();
    ss.position_gcrf_m += ss.attitude.rotate(cg_new - compound_cg_point_m_);
    station.set_state(ss);
    docked_position_station_m_ = attached.position_m - cg_new;
    docked_mass_kg_ = attached.mass_kg;
    compound_cg_point_m_ = cg_new;
    docked_chaser_cg_point_m_ = chaser_cg;
    update_compound_mass();
    sync_docked();
}

void FlightSession::detach_compound() {
    reset_docked_update();
    end_transfer();
    auto& station = *bodies_[*station_index_].body;
    auto s = station.state();
    const math::Vec3 cg_with = station.vehicle().mass_properties().cg_point_m();
    station.vehicle().set_attached_mass(std::nullopt);
    const math::Vec3 cg_without = station.vehicle().mass_properties().cg_point_m();
    const math::Vec3 shift = s.attitude.rotate(cg_without - cg_with);
    s.position_gcrf_m += shift;  // the station's own CG again, moving with the turning compound
    s.velocity_gcrf_mps += cross(s.attitude.rotate(s.angular_velocity_body_radps), shift);
    station.set_state(s);
    compound_mass_.reset();
    docked_mass_kg_ = 0.0;
}

bool FlightSession::undock() {
    if (!docking_) {
        docking_status_ = "Abkoppeln nicht möglich: nicht fest angedockt";
        return false;
    }
    if (transfer_ && transfer_->active()) {
        docking_status_ = "Abkoppeln nicht möglich: Treibstofftransfer läuft";
        return false;
    }
    if (docked() && autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit) {
        docking_status_ = "Abkoppeln nicht möglich: Autopilot aktiv";
        return false;
    }
    if (!docking_->undock(epoch_)) {
        docking_status_ = docking_->mechanism().refusal();
        return false;
    }
    docking_status_ = docking_->mechanism().status();
    return true;
}

void FlightSession::separate_docked() {
    auto& chaser = *bodies_[controlled_].body;
    auto& station = *bodies_[*station_index_].body;
    sync_docked();  // the chaser on its port at this tick
    chaser.release_docked();
    detach_compound();
    const double mc = chaser.vehicle().mass_properties().mass_kg;
    const double ms = station.vehicle().mass_properties().mass_kg;
    auto cs = chaser.state();
    auto ss = station.state();
    docking_separation_.momentum_before = mc * cs.velocity_gcrf_mps + ms * ss.velocity_gcrf_mps;
    // the springs along the station port axis (outward), split by mass: the pair's momentum is kept
    const math::Vec3 axis = column(vehicle::port_kinematics(station, docking_->station_port()).axes_gcrf, 0);
    const double v_rel = contact::separation_relative_speed_mps(docking_->mechanism().parameters(), mc, ms);
    cs.velocity_gcrf_mps += (v_rel * ms / (mc + ms)) * axis;
    ss.velocity_gcrf_mps -= (v_rel * mc / (mc + ms)) * axis;
    chaser.set_state(cs);
    station.set_state(ss);
    docking_separation_.momentum_after = mc * cs.velocity_gcrf_mps + ms * ss.velocity_gcrf_mps;
    docking_separation_.relative_speed_mps = v_rel;
    // the departure on the docking ports
    proximity_station_port_ = docking_->station_port();
    proximity_chaser_port_ = docking_->chaser_port();
    measure_proximity_authority();
    departure_.emplace(with_station_limits(gnc::DepartureParameters{}, station_limits()),
                       with_station_limits(gnc::ProximityParameters{}, station_limits()));
    autopilot_.state = AutopilotState::Departure;
    attitude_mode_ = proximity_attitude_mode();
}

void FlightSession::run_departure(Flying& flying) {
    if (!departure_ || !station_index_ || !rendezvous_) {
        autopilot_.state = AutopilotState::Off;
        return;
    }
    if (flying.body->impacted()) {
        autopilot_.state = AutopilotState::Off;
        attitude_mode_ = {};
        translation_port_ = {};
        return;
    }
    if (docking_collision_reported_) {  // a hull contact ended the departure (status stays "Kollision")
        autopilot_.state = AutopilotState::Off;
        attitude_mode_ = {};
        translation_port_ = {};
        return;
    }
    const size_t events_before = departure_->events().size();
    const auto in = proximity_input();
    const auto out = departure_->update(in);
    docking_status_ = departure_->status();
    if (docking_debug_) {  // STARSIM_DEBUG_DOCKING: the departure events with the relative state
        for (size_t i = events_before; i < departure_->events().size(); ++i) {
            std::printf("[docking] %8.1f s  %s  port x %.2f m, CG range %.1f m, %.1f kg\n", mission_time_s(),
                        departure_->events()[i].status.c_str(), in.port.position_m.x, norm(in.cg_lvlh.position_m),
                        in.propellant_kg);
        }
    }
    if (departure_->phase() == gnc::DeparturePhase::Complete) {
        autopilot_.state = AutopilotState::Off;
        attitude_mode_ = {};
        clear_rendezvous_target();  // also ends the docking mechanism; the departure stays readable
        return;
    }
    translation_port_ = out.translation_fraction_port;
    attitude_mode_ = out.attitude_hold ? proximity_attitude_mode() : gnc::AttitudeMode{};
    departure_next_check_ = epoch_ + departure_check_s;  // the drift coasts in between
}

std::optional<gnc::DeparturePhase> FlightSession::departure_phase() const {
    if (!departure_) return std::nullopt;
    return departure_->phase();
}

const std::vector<gnc::DepartureEvent>* FlightSession::departure_events() const {
    return departure_ ? &departure_->events() : nullptr;
}

void FlightSession::sync_docked() {
    if (!docking_ || !station_index_ || controlled_ >= bodies_.size()) return;
    auto& chaser = *bodies_[controlled_].body;
    if (!chaser.docked()) return;
    const auto& ss = bodies_[*station_index_].body->state();
    const math::Vec3 r = ss.attitude.rotate(docked_position_station_m_);
    const math::Vec3 omega = ss.attitude.rotate(ss.angular_velocity_body_radps);
    vehicle::RigidBodyState s;
    s.attitude = ss.attitude * docked_attitude_station_;
    s.position_gcrf_m = ss.position_gcrf_m + r;
    s.velocity_gcrf_mps = ss.velocity_gcrf_mps + cross(omega, r);
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(omega);
    chaser.set_state(s);
}

// STARSIM_DEBUG_DOCKING: every mechanism event, and every second while it is ticked the pose, the force and the loads.
void FlightSession::docking_diagnostics() {
    const auto& m = docking_->mechanism();
    const auto& events = m.events();
    for (; docking_debug_events_ < events.size(); ++docking_debug_events_) {
        std::printf("[docking] %8.1f s  %s\n", mission_time_s(), events[docking_debug_events_].status.c_str());
    }
    if (epoch_ < docking_debug_next_ || bodies_[controlled_].body->docked()) return;
    const auto p = docking_->pose(*bodies_[controlled_].body, *bodies_[*station_index_].body);
    if (!docking_ticked() && norm(p.position_m) > 2.0) return;  // also the last two metres before the contact
    docking_debug_next_ = epoch_ + 1.0;
    const auto f = docking_->force_on_chaser_n();
    std::printf("[docking] %8.1f s  %-16s port (%.3f %.3f %.3f) m v (%.4f %.4f %.4f) m/s  F %.0f N  peak comp %.0f N\n",
                mission_time_s(), contact::to_string(m.phase()), p.position_m.x, p.position_m.y, p.position_m.z,
                p.velocity_mps.x, p.velocity_mps.y, p.velocity_mps.z, norm(f), m.peaks().compression_n);
}

}  // namespace starsim::sim
