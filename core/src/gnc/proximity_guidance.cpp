#include "starsim/gnc/proximity_guidance.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "starsim/gnc/proximity_transfer.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

bool is_flyaround_point(const ProximityWaypoint& w) { return w.name.rfind("Umflug", 0) == 0; }

double lateral_m(const Vec3& position_port_m) { return std::hypot(position_port_m.y, position_port_m.z); }

double max_abs(const Vec3& v) { return std::max({std::abs(v.x), std::abs(v.y), std::abs(v.z)}); }

}  // namespace

const char* to_string(ProximityPhase phase) {
    switch (phase) {
        case ProximityPhase::Waypoint: return "Wegpunkt";
        case ProximityPhase::Transfer: return "Annäherung";
        case ProximityPhase::Flyaround: return "Umflug";
        case ProximityPhase::Approach: return "Anflug";
        case ProximityPhase::FinalApproach: return "Endanflug";
        case ProximityPhase::Hold: return "Halt";
        case ProximityPhase::Retreat: return "Rückzug";
        case ProximityPhase::Abort: return "Abbruch";
        case ProximityPhase::Contact: return "Kontakt";
    }
    return "";
}

ProximityGuidance::ProximityGuidance(DockingApproach approach, ProximityParameters parameters)
    : approach_(approach), p_(std::move(parameters)), path_(proximity_path(approach, p_)) {}

void ProximityGuidance::enter(ProximityPhase phase, std::string status, time::Epoch epoch) {
    phase_ = phase;
    status_ = std::move(status);
    loop_.reset();
    events_.push_back({epoch, phase_, status_});
}

Vec3 ProximityGuidance::target_cg(const ProximityInput& in, size_t index) const {
    const ProximityWaypoint& w = path_[index];
    if (w.frame == WaypointFrame::LvlhFromCg) return w.position_m;
    // the port offset (station port lever and chaser port lever in the held attitude) is constant
    return in.cg_lvlh.position_m + in.port_to_lvlh * (w.position_m - in.port.position_m);
}

bool ProximityGuidance::inside_sphere(const ProximityInput& in) const {
    return norm(in.cg_lvlh.position_m) < p_.keep_out_radius_m;
}

double ProximityGuidance::corridor_bound_m(double x_m) const {
    return std::max(x_m * std::tan(p_.corridor_half_angle_rad), p_.corridor_min_half_width_m);
}

void ProximityGuidance::start(const ProximityInput& in) {
    started_ = true;
    target_ = reached_ = std::min(p_.start_waypoint, path_.size() - 1);
    if (norm(target_cg(in, target_) - in.cg_lvlh.position_m) < p_.start_tolerance_m) {
        arrive(in, false);
    } else {
        start_segment(in);
    }
}

void ProximityGuidance::arrive(const ProximityInput& in, bool waiting_for_go) {
    reached_ = target_;
    waiting_for_go_ = waiting_for_go;
    go_ = false;
    waypoint_since_ = in.epoch;
    const std::string& name = path_[target_].name;
    enter(ProximityPhase::Waypoint, waiting_for_go ? name + " - warte auf Freigabe" : name, in.epoch);
}

bool ProximityGuidance::plan_transfer(const ProximityInput& in, ProximityPhase phase, double nominal_s,
                                      std::string status) {
    const ProximityTransfer t =
        plan_proximity_transfer(in.cg_lvlh, target_cg(in, target_), in.mean_motion_radps, nominal_s, p_);
    if (!t.feasible) {
        begin_abort("Abbruch: kein sicherer Transferweg", in.epoch);
        return false;
    }
    arrival_epoch_ = in.epoch + t.duration_s;
    last_transfer_velocity_lvlh_ = t.departure_velocity_lvlh_mps;
    enter(phase, std::move(status), in.epoch);
    return true;
}

void ProximityGuidance::start_segment(const ProximityInput& in) {
    const ProximityWaypoint& w = path_[target_];
    if (w.on_axis && norm(target_cg(in, target_)) < p_.keep_out_radius_m) {
        enter(ProximityPhase::Approach, "Anflug auf " + w.name, in.epoch);
        return;
    }
    const bool flyaround = is_flyaround_point(w) || (target_ > 0 && is_flyaround_point(path_[target_ - 1]));
    // Waypoint 0 -> next and a (re)start use the nominal transfer time; legs between fly-around points the leg time
    const bool leg = flyaround && reached_ != 0 && reached_ + 1 == target_;
    plan_transfer(in, flyaround ? ProximityPhase::Flyaround : ProximityPhase::Transfer,
                  leg ? p_.flyaround_leg_s : p_.transfer_s,
                  (flyaround ? "Umflug zu " : "Annäherung an ") + w.name);
}

void ProximityGuidance::start_retreat(const ProximityInput& in, size_t index) {
    target_ = index;
    const std::string status = "Rückzug zu " + path_[index].name;
    // on the axis (inside the sphere, or between the sphere and the on-axis waypoint) the axial law flies back; a CW
    // transfer from there would start inside the transfer clearance
    retreat_axial_ = path_[index].on_axis &&
                     (inside_sphere(in) || in.port.position_m.x < path_[index].position_m.x);
    if (retreat_axial_) {
        enter(ProximityPhase::Retreat, status, in.epoch);
    } else {
        plan_transfer(in, ProximityPhase::Retreat, p_.transfer_s, status);
    }
}

void ProximityGuidance::capture_failed(const ProximityInput& in, const std::string& reason) {
    if (phase_ != ProximityPhase::Contact || collided_) return;
    size_t index = path_.size() - 1;  // the last on-axis waypoint: Waypoint 2
    while (index > 0 && !path_[index].on_axis) --index;
    request_ = Request::None;
    loop_.reset();
    start_retreat(in, index);
    status_ = "Rückzug: Fangen fehlgeschlagen (" + reason + ")";
    events_.back().status = status_;
}

void ProximityGuidance::collided(time::Epoch epoch) {
    collided_ = true;
    request_ = Request::None;
    enter(ProximityPhase::Contact, "Kollision", epoch);
}

void ProximityGuidance::begin_abort(std::string status, time::Epoch epoch) {
    breakout_planned_ = breakout_done_ = breakout_safe_ = false;
    auto_hold_ = false;
    enter(ProximityPhase::Abort, std::move(status), epoch);
}

void ProximityGuidance::auto_hold(const ProximityInput& in, std::string status) {
    interrupted_ = phase_;
    interrupted_status_ = status_;
    auto_hold_ = true;
    hold_in_port_frame_ = true;
    hold_point_ = {in.port.position_m.x, 0.0, 0.0};
    enter(ProximityPhase::Hold, std::move(status), in.epoch);
}

bool ProximityGuidance::refuse(std::string reason) {
    refusal_ = std::move(reason);
    return false;
}

bool ProximityGuidance::command_open() {
    if (!started_) return refuse("Befehl nicht möglich: Anflug nicht gestartet");
    if (request_ != Request::None) return refuse("Befehl nicht möglich: vorheriger Befehl noch offen");
    return true;
}

void ProximityGuidance::observe(const ProximityInput& in) {
    last_cg_range_m_ = norm(in.cg_lvlh.position_m);
    last_propellant_kg_ = in.propellant_kg;
}

bool ProximityGuidance::hold() {
    if (!command_open()) return false;
    if (phase_ == ProximityPhase::Abort) return refuse("Halt nicht möglich: Abbruch läuft");
    if (phase_ == ProximityPhase::Contact) return refuse("Halt nicht möglich: Kontakt");
    if (phase_ == ProximityPhase::Hold) return refuse("Halt nicht möglich: bereits im Halt");
    request_ = Request::Hold;
    return true;
}

bool ProximityGuidance::retreat() {
    if (!command_open()) return false;
    if (phase_ == ProximityPhase::Abort) return refuse("Rückzug nicht möglich: Abbruch läuft");
    if (phase_ == ProximityPhase::Contact) return refuse("Rückzug nicht möglich: Kontakt");
    if (phase_ == ProximityPhase::Retreat) return refuse("Rückzug nicht möglich: Rückzug läuft bereits");
    const bool at_waypoint = phase_ == ProximityPhase::Waypoint ||
                             (phase_ == ProximityPhase::Hold && interrupted_ == ProximityPhase::Waypoint);
    if (target_ == 0) return refuse("Rückzug nicht möglich: kein Wegpunkt dahinter");  // at or before Waypoint 0
    retreat_index_ = at_waypoint ? target_ - 1 : reached_;
    request_ = Request::Retreat;
    return true;
}

bool ProximityGuidance::abort() {
    if (!command_open()) return false;
    if (phase_ == ProximityPhase::Abort) return refuse("Abbruch läuft bereits");
    if (phase_ == ProximityPhase::Contact) return refuse("Abbruch nicht möglich: Kontakt");
    request_ = Request::Abort;
    return true;
}

bool ProximityGuidance::resume() {
    if (!command_open()) return false;
    const bool from_hold = phase_ == ProximityPhase::Hold && !auto_hold_;
    const bool from_waiting = phase_ == ProximityPhase::Waypoint && waiting_for_go_;
    const bool from_abort = phase_ == ProximityPhase::Abort;
    if (!from_hold && !from_waiting && !from_abort) {
        return refuse("Wiederaufnahme nicht möglich: kein Halt und keine Freigabe offen");
    }
    if (from_abort) {
        // the resume flies back to Waypoint 0: only after the breakout, from a range CW still covers, and with the
        // reserve (below it, or inside the sphere, the next update would abort again and fire a new breakout)
        if (!breakout_done_) return refuse("Wiederaufnahme nicht möglich: Ausweichmanöver läuft");
        if (last_cg_range_m_ > p_.resume_max_range_m) return refuse("Wiederaufnahme nicht möglich: zu weit entfernt");
        if (last_cg_range_m_ < p_.keep_out_radius_m) {
            return refuse("Wiederaufnahme nicht möglich: innerhalb der Sicherheitszone");
        }
        if (last_propellant_kg_ < p_.deorbit_reserve_kg) {
            return refuse("Wiederaufnahme nicht möglich: Treibstoffreserve");
        }
    }
    request_ = Request::Resume;
    return true;
}

void ProximityGuidance::resolve_request(const ProximityInput& in) {
    const Request r = std::exchange(request_, Request::None);
    switch (r) {
        case Request::None: return;
        case Request::Hold:
            interrupted_ = phase_;
            interrupted_status_ = status_;
            auto_hold_ = false;
            hold_in_port_frame_ = inside_sphere(in);
            hold_point_ = hold_in_port_frame_ ? in.port.position_m : in.cg_lvlh.position_m;
            enter(ProximityPhase::Hold, "Halt", in.epoch);
            return;
        case Request::Retreat: start_retreat(in, retreat_index_); return;
        case Request::Abort: begin_abort("Abbruch", in.epoch); return;
        case Request::Resume:
            if (phase_ == ProximityPhase::Waypoint) {
                waiting_for_go_ = false;
                go_ = true;
            } else if (phase_ == ProximityPhase::Abort) {
                target_ = reached_ = 0;
                start_segment(in);
            } else if (interrupted_ == ProximityPhase::Transfer || interrupted_ == ProximityPhase::Flyaround) {
                start_segment(in);
            } else if (interrupted_ == ProximityPhase::Retreat) {
                start_retreat(in, target_);
            } else {
                enter(interrupted_, interrupted_status_, in.epoch);
            }
            return;
    }
}

void ProximityGuidance::transitions(const ProximityInput& in) {
    switch (phase_) {
        case ProximityPhase::Transfer:
        case ProximityPhase::Flyaround:
            if (norm(target_cg(in, target_) - in.cg_lvlh.position_m) < p_.arrival_tolerance_m &&
                norm(in.cg_lvlh.velocity_mps) < p_.arrival_speed_mps) {
                arrive(in, false);
            }
            return;
        case ProximityPhase::Retreat:
            if (retreat_axial_ ? in.port.position_m.x >= path_[target_].position_m.x
                               : norm(target_cg(in, target_) - in.cg_lvlh.position_m) < p_.arrival_tolerance_m &&
                                     norm(in.cg_lvlh.velocity_mps) < p_.arrival_speed_mps) {
                arrive(in, true);
            }
            return;
        case ProximityPhase::Approach:
            if (in.port.position_m.x <= path_[target_].position_m.x + p_.axis_arrival_tolerance_m) arrive(in, false);
            return;
        case ProximityPhase::FinalApproach:
            if (in.port.position_m.x <= p_.contact_plane_m) {
                contact_ = ProximityContact{in.epoch, in.port, capture_envelope_check(in.port)};
                std::string status = "Kontakt";
                if (!contact_->envelope.inside) status += " außerhalb der Fangtoleranz: " + contact_->envelope.reason;
                enter(ProximityPhase::Contact, status, in.epoch);
            }
            return;
        case ProximityPhase::Waypoint:
            if (go_ || (!waiting_for_go_ && in.epoch - *waypoint_since_ >= path_[target_].hold_s)) {
                go_ = false;
                if (target_ + 1 < path_.size()) {
                    ++target_;
                    start_segment(in);
                } else {
                    enter(ProximityPhase::FinalApproach, "Endanflug", in.epoch);
                }
            }
            return;
        case ProximityPhase::Hold:
            if (auto_hold_ && lateral_m(in.port.position_m) < 0.5 * corridor_bound_m(in.port.position_m.x) &&
                norm(in.port.velocity_mps) < p_.arrival_speed_mps) {
                auto_hold_ = false;
                enter(interrupted_, interrupted_status_, in.epoch);
            }
            return;
        case ProximityPhase::Abort:
        case ProximityPhase::Contact: return;
    }
}

void ProximityGuidance::monitor(const ProximityInput& in) {
    if (phase_ == ProximityPhase::Abort || phase_ == ProximityPhase::Contact) return;
    if (in.propellant_kg < p_.deorbit_reserve_kg) {
        begin_abort("Abbruch: Treibstoffreserve", in.epoch);
        return;
    }
    const double cg_range_m = norm(in.cg_lvlh.position_m);
    const CorridorStatus corridor = corridor_status(in.port.position_m, cg_range_m, p_);
    const bool watched = phase_ == ProximityPhase::Approach || phase_ == ProximityPhase::FinalApproach ||
                         (phase_ == ProximityPhase::Waypoint && inside_sphere(in)) ||
                         (phase_ == ProximityPhase::Hold && auto_hold_);
    if (watched) {
        if (corridor == CorridorStatus::Retreat) {
            auto_hold_ = false;
            // retreating to an on-axis waypoint the chaser is already at or beyond would arrive at once and retreat
            // again every tick: go one waypoint further back
            size_t index = reached_;
            if (index > 0 && path_[index].on_axis && in.port.position_m.x >= path_[index].position_m.x) --index;
            start_retreat(in, index);
            return;
        }
        if (phase_ == ProximityPhase::Hold) return;
        if (corridor == CorridorStatus::Hold) {
            auto_hold(in, "Halt: Korridor verlassen");
            return;
        }
        if (in.port.closing_speed_mps >
            p_.overspeed_factor * axial_closing_speed_mps(in.port.position_m.x, p_) + p_.overspeed_margin_mps) {
            auto_hold(in, "Halt: zu schnell");
        }
        return;
    }
    if ((phase_ == ProximityPhase::Transfer || phase_ == ProximityPhase::Flyaround) &&
        corridor != CorridorStatus::Inside && corridor != CorridorStatus::OutsideKos) {
        begin_abort("Abbruch: Sicherheitszone verletzt", in.epoch);
    }
}

Vec3 ProximityGuidance::loop_lvlh(const ProximityInput& in, const Vec3& desired_lvlh,
                                  const TranslationLoopParameters& lp) {
    const math::Mat3 lvlh_to_port = transpose(in.port_to_lvlh);
    return loop_.command(lvlh_to_port * desired_lvlh, lvlh_to_port * in.cg_lvlh.velocity_mps, in.authority_port, lp);
}

Vec3 ProximityGuidance::loop_port(const ProximityInput& in, const Vec3& desired_port,
                                  const TranslationLoopParameters& lp) {
    return loop_.command(desired_port, in.port.velocity_mps, in.authority_port, lp);
}

ProximityOutput ProximityGuidance::fly(const ProximityInput& in) {
    ProximityOutput out;
    const TranslationLoopParameters& near_or_far = inside_sphere(in) ? p_.loop_near : p_.loop_far;
    switch (phase_) {
        case ProximityPhase::Contact: return out;
        case ProximityPhase::Abort: {
            if (!breakout_planned_) {
                const Breakout b = breakout_delta_v(in.cg_lvlh, in.mean_motion_radps, p_);
                abort_target_lvlh_ = in.cg_lvlh.velocity_mps + b.delta_v_lvlh_mps;
                breakout_safe_ = b.safe;
                breakout_planned_ = true;
            }
            if (!breakout_done_ && max_abs(abort_target_lvlh_ - in.cg_lvlh.velocity_mps) < p_.loop_far.stop_error_mps) {
                breakout_done_ = true;
                status_ = breakout_safe_ ? "Abbruch: Ausweichmanöver ausgeführt"
                                         : "Abbruch: Ausweichmanöver ausgeführt, nicht passiv sicher";
            }
            if (breakout_done_) {
                out.attitude_hold = false;
                return out;
            }
            TranslationLoopParameters fire_now = p_.loop_far;
            fire_now.start_error_mps = 0.0;
            out.translation_fraction_port = loop_lvlh(in, abort_target_lvlh_, fire_now);
            return out;
        }
        case ProximityPhase::Transfer:
        case ProximityPhase::Flyaround:
        case ProximityPhase::Retreat: {
            if (phase_ == ProximityPhase::Retreat && retreat_axial_) {
                const Vec3 lateral = approach_velocity_port(in.port.position_m, p_);
                out.translation_fraction_port =
                    loop_port(in, {p_.retreat_speed_mps, lateral.y, lateral.z}, p_.loop_near);
                return out;
            }
            const Vec3 to = target_cg(in, target_);
            const double to_go_s = arrival_epoch_ - in.epoch;
            Vec3 desired;
            if (to_go_s > p_.transfer_final_s) {
                if (const auto v = orbit::cw_transfer_velocity(in.cg_lvlh.position_m, to, in.mean_motion_radps, to_go_s)) {
                    last_transfer_velocity_lvlh_ = *v;
                }
                desired = last_transfer_velocity_lvlh_;
            } else {
                desired = hold_velocity(in.cg_lvlh.position_m, to, p_);
            }
            out.translation_fraction_port = loop_lvlh(in, desired, p_.loop_far);
            return out;
        }
        case ProximityPhase::Approach:
        case ProximityPhase::FinalApproach:
            out.translation_fraction_port = loop_port(in, approach_velocity_port(in.port.position_m, p_), p_.loop_near);
            return out;
        case ProximityPhase::Waypoint: {
            const ProximityWaypoint& w = path_[target_];
            out.translation_fraction_port =
                w.frame == WaypointFrame::TargetPort
                    ? loop_port(in, hold_velocity(in.port.position_m, w.position_m, p_), near_or_far)
                    : loop_lvlh(in, hold_velocity(in.cg_lvlh.position_m, w.position_m, p_), near_or_far);
            return out;
        }
        case ProximityPhase::Hold:
            out.translation_fraction_port =
                hold_in_port_frame_ ? loop_port(in, hold_velocity(in.port.position_m, hold_point_, p_), near_or_far)
                                    : loop_lvlh(in, hold_velocity(in.cg_lvlh.position_m, hold_point_, p_), near_or_far);
            return out;
    }
    return out;
}

ProximityOutput ProximityGuidance::update(const ProximityInput& in) {
    last_epoch_ = in.epoch;
    observe(in);
    if (!started_) start(in);
    resolve_request(in);
    transitions(in);
    monitor(in);
    return fly(in);
}

}  // namespace starsim::gnc
