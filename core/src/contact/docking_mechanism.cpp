#include "starsim/contact/docking_mechanism.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace starsim::contact {

namespace {
// Rotation vector (axis * angle) of the rotation matrix r.
math::Vec3 rotation_vector(const math::Mat3& r) {
    const double c = std::clamp(0.5 * (r(0, 0) + r(1, 1) + r(2, 2) - 1.0), -1.0, 1.0);
    const double angle = std::acos(c);
    const math::Vec3 v{r(2, 1) - r(1, 2), r(0, 2) - r(2, 0), r(1, 0) - r(0, 1)};
    if (angle < 1e-9) return 0.5 * v;
    return (angle / (2.0 * std::sin(angle))) * v;
}
const math::Mat3 mated_axes = math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});

report::Report not_possible(report::Code outer, report::Code cause) {
    return report::with_cause(report::make(outer), report::make(cause));
}

report::Report docked_status(int hook_count) {
    return report::make(report::Code::docking_docked, {report::count("hooks", hook_count)});
}

report::Report limit_warning(report::Code code, const DockingMechanismParameters& p) {
    return report::make(code, {report::ref("limit", p.limit_ref)});
}
}  // namespace

const char* id_of(DockingPhase phase) {  // i18n-ns: docking_phase
    switch (phase) {
        case DockingPhase::Ready: return "ready";
        case DockingPhase::Contact: return "contact";
        case DockingPhase::SoftCapture: return "soft_capture";
        case DockingPhase::Retraction: return "retraction";
        case DockingPhase::HooksClosing: return "hooks_closing";
        case DockingPhase::Docked: return "docked";
        case DockingPhase::HooksOpening: return "hooks_opening";
        case DockingPhase::Separated: return "separated";
        case DockingPhase::Failed: return "failed";
    }
    return "";
}

double separation_relative_speed_mps(const DockingMechanismParameters& p, double m1, double m2) {
    const double mu = m1 * m2 / (m1 + m2);
    return std::sqrt(2.0 * p.separation_energy_j / mu);
}

DockingMechanism::DockingMechanism(DockingMechanismParameters p) : p_(std::move(p)) { extension_m_ = full_extension_m(); }

double DockingMechanism::full_extension_m() const {
    return p_.probe_drogue ? p_.probe_drogue->extension_m : p_.interface.ring_extension_m;
}

double DockingMechanism::first_contact_m() const {
    if (p_.probe_drogue) return p_.probe_drogue->cone_depth_m + p_.probe_drogue->extension_m;
    return p_.interface.ring_extension_m + 2.0 * p_.interface.petal_height_m;
}

double DockingMechanism::reach_m() const {
    if (!p_.probe_drogue) return first_contact_m() + p_.interface.interface_radius_m;
    double lateral = 0.0;
    for (const auto& q : p_.probe_drogue->probes_m) lateral = std::max(lateral, std::hypot(q.y, q.z));
    return first_contact_m() + lateral + p_.probe_drogue->mouth_radius_m;
}

InterfaceContact DockingMechanism::contact(const DockingPose& pose, double extension_m) const {
    return p_.probe_drogue ? probe_drogue_contact(pose, extension_m, *p_.probe_drogue)
                           : docking_interface_contact(pose, extension_m, p_.interface);
}

void DockingMechanism::enter(DockingPhase phase, report::Report status, double t_s) {
    phase_ = phase;
    phase_since_s_ = t_s;
    status_ = status;
    events_.push_back({t_s, phase, std::move(status)});
}

void DockingMechanism::fail(report::Report reason, double t_s) {
    failure_ = reason;
    extension_m_ = full_extension_m();
    enter(DockingPhase::Failed, report::with_cause(report::make(report::Code::docking_capture_failed), reason), t_s);
}

void DockingMechanism::warn(report::Report warning) {
    for (const auto& w : warnings_) {
        if (report::same(w, warning)) return;
    }
    warnings_.push_back(std::move(warning));
}

void DockingMechanism::rearm() {
    if (phase_ != DockingPhase::Failed) return;
    ring_touched_ = ring_blocked_ = false;
    tips_only_ = true;
    static_over_s_ = 0.0;
    phase_ = DockingPhase::Ready;
    status_ = report::make(report::Code::docking_ready);
}

bool DockingMechanism::start_docked(double t_s) {
    if (phase_ != DockingPhase::Ready) {
        refusal_ = not_possible(report::Code::docking_dock_not_possible, report::Code::docking_cause_mechanism_not_ready);
        return false;
    }
    extension_m_ = 0.0;
    kb_ = p_.joint_bending_stiffness_nm_per_rad;  // no joint runs while docked; defined all the same
    kt_ = p_.joint_torsion_stiffness_nm_per_rad;
    enter(DockingPhase::Docked, docked_status(p_.hook_count), t_s);
    return true;
}

bool DockingMechanism::undock(double t_s) {
    if (phase_ == DockingPhase::HooksOpening || phase_ == DockingPhase::Separated) {
        refusal_ = not_possible(report::Code::docking_undock_not_possible, report::Code::docking_cause_undock_running);
        return false;
    }
    if (phase_ != DockingPhase::Docked) {
        refusal_ = not_possible(report::Code::docking_undock_not_possible, report::Code::docking_cause_not_hard_docked);
        return false;
    }
    enter(DockingPhase::HooksOpening, report::make(report::Code::docking_hooks_opening), t_s);
    return true;
}

bool DockingMechanism::separation_due() {
    const bool due = separation_pending_;
    separation_pending_ = false;
    return due;
}

DockingWrench DockingMechanism::joint(const DockingPose& pose) const {
    // rest: the chaser port extension_m_ out of the passive port (its ring face on the passive ring face), centred
    const math::Vec3 rest{extension_m_, 0.0, 0.0};
    math::Vec3 force = -k_ * (pose.position_m - rest) - c_ * pose.velocity_mps;
    const double over_stroke = extension_m_ - pose.position_m.x - p_.stroke_m;
    if (over_stroke > 0.0) force.x += p_.end_stop_stiffness_n_per_m * over_stroke;
    const math::Vec3 theta = rotation_vector(pose.chaser_axes * transpose(mated_axes));  // x: roll (torsion)
    const math::Vec3& w = pose.angular_rate_radps;
    const double kb = kb_, kt = kt_;
    const math::Vec3 torque{-kt * theta.x - ct_ * w.x, -kb * theta.y - cb_ * w.y, -kb * theta.z - cb_ * w.z};
    return {force, pose.position_m, torque};
}

DockingWrench DockingMechanism::before_capture(double t, const DockingPose& pose, double m, double inertia) {
    extension_m_ = full_extension_m();  // before capture the ring / the probes are fully extended
    const auto c = contact(pose, extension_m_);
    DockingWrench w{c.force_port_n, c.ring_centre_m, c.torque_port_nm};
    if (phase_ == DockingPhase::Failed) return w;  // rebounding; rearm() starts a new approach
    if (c.any_contact) {
        last_contact_s_ = t;
        if (phase_ == DockingPhase::Ready) enter(DockingPhase::Contact, report::make(report::Code::docking_first_contact), t);
        ring_touched_ = ring_touched_ || c.ring_contact;
        tips_only_ = tips_only_ && c.tip_strike;
        ring_blocked_ = ring_blocked_ || (c.ring_contact && !c.latch_ready);
    }
    if (phase_ != DockingPhase::Contact) return w;
    if (c.latch_ready) {
        k_ = m * p_.joint_frequency_radps * p_.joint_frequency_radps;
        c_ = 2.0 * p_.joint_damping_ratio * m * p_.joint_frequency_radps;
        const double wr = p_.joint_rotation_frequency_radps;
        kb_ = wr > 0.0 ? inertia * wr * wr : p_.joint_bending_stiffness_nm_per_rad;
        kt_ = wr > 0.0 ? inertia * wr * wr : p_.joint_torsion_stiffness_nm_per_rad;
        cb_ = 2.0 * p_.joint_damping_ratio * std::sqrt(kb_ * inertia);
        ct_ = 2.0 * p_.joint_damping_ratio * std::sqrt(kt_ * inertia);
        enter(DockingPhase::SoftCapture, report::make(report::Code::docking_soft_capture), t);
        return joint(pose);
    }
    if (t - last_contact_s_ > p_.bounce_confirm_s && c.gap_m > p_.bounce_gap_m) {
        fail(report::make(tips_only_ ? report::Code::docking_fail_offset
                                     : (ring_blocked_ ? report::Code::docking_fail_angle : report::Code::docking_fail_bounced)),
             t);
    }
    return w;
}

DockingWrench DockingMechanism::update(double t, const DockingPose& pose, double m, double inertia, double dt) {
    DockingWrench w{{}, pose.position_m, {}};
    switch (phase_) {
        case DockingPhase::Ready:
        case DockingPhase::Contact:
        case DockingPhase::Failed:
            w = before_capture(t, pose, m, inertia);
            break;
        case DockingPhase::SoftCapture: {
            w = joint(pose);
            const bool settled = norm(pose.velocity_mps) < p_.settled_speed_mps &&
                                 norm(pose.angular_rate_radps) < p_.settled_rate_radps;
            if (t - phase_since_s_ >= p_.attenuation_min_s && settled) enter(DockingPhase::Retraction, report::make(report::Code::docking_retraction), t);
            break;
        }
        case DockingPhase::Retraction: {
            extension_m_ = std::max(0.0, extension_m_ - p_.retraction_speed_mps * dt);
            w = joint(pose);
            const double angle = norm(rotation_vector(pose.chaser_axes * transpose(mated_axes)));
            if (extension_m_ == 0.0 && norm(pose.position_m) <= p_.guide_pin_tolerance_m &&
                angle <= p_.guide_pin_angle_rad) {
                enter(DockingPhase::HooksClosing, report::make(report::Code::docking_hooks_closing), t);
            } else if (t - phase_since_s_ > p_.retraction_timeout_s) {
                fail(report::make(report::Code::docking_fail_retraction), t);
                return before_capture(t, pose, m, inertia);
            }
            break;
        }
        case DockingPhase::HooksClosing:
            w = joint(pose);
            if (t - phase_since_s_ >= p_.hooks_close_s) {
                enter(DockingPhase::Docked, docked_status(p_.hook_count), t);
                return {};
            }
            break;
        case DockingPhase::Docked: return {};  // the session holds the chaser on the station
        case DockingPhase::HooksOpening:
            if (t - phase_since_s_ >= p_.hooks_open_s) {
                enter(DockingPhase::Separated, report::make(report::Code::docking_separated), t);
                separation_pending_ = true;
            }
            return {};
        case DockingPhase::Separated: return {};
    }
    if (phase_ != DockingPhase::Failed && phase_ != DockingPhase::Ready) {
        record_loads(w, t, dt);
        if (phase_ == DockingPhase::Failed) return before_capture(t, pose, m, inertia);  // Überlast: latches released
    }
    return w;
}

void DockingMechanism::record_loads(const DockingWrench& w, double t, double dt) {
    const math::Vec3& f = w.force_port_n;
    // moment about the SCS mating plane centre (the passive ring face centre: the passive port origin [E])
    const math::Vec3 moment = w.torque_port_nm + cross(w.point_port_m, f);
    const double compression = std::max(0.0, f.x);
    peaks_.compression_n = std::max(peaks_.compression_n, compression);
    peaks_.tension_n = std::max(peaks_.tension_n, std::max(0.0, -f.x));
    peaks_.shear_n = std::max(peaks_.shear_n, std::hypot(f.y, f.z));
    peaks_.bending_nm = std::max(peaks_.bending_nm, std::hypot(moment.y, moment.z));
    peaks_.torsion_nm = std::max(peaks_.torsion_nm, std::abs(moment.x));
    if (peaks_.tension_n > p_.tension_limit_n) warn(limit_warning(report::Code::docking_warn_tension, p_));
    if (peaks_.shear_n > p_.shear_limit_n) warn(limit_warning(report::Code::docking_warn_shear, p_));
    if (peaks_.bending_nm > p_.bending_limit_nm) warn(limit_warning(report::Code::docking_warn_bending, p_));
    if (peaks_.torsion_nm > p_.torsion_limit_nm) warn(limit_warning(report::Code::docking_warn_torsion, p_));
    static_over_s_ = compression > p_.static_compression_limit_n ? static_over_s_ + dt : 0.0;
    if (compression > p_.dynamic_compression_limit_n || static_over_s_ > p_.static_after_s) {
        fail(report::make(report::Code::docking_fail_overload), t);
    }
}

}  // namespace starsim::contact
