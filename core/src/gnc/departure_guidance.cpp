#include "starsim/gnc/departure_guidance.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

const char* to_string(DeparturePhase phase) {
    switch (phase) {
        case DeparturePhase::UndockBurn: return "Abkoppelimpuls";
        case DeparturePhase::BackOut: return "Rückzug vom Port";
        case DeparturePhase::DepartureBurn: return "Abflugmanöver";
        case DeparturePhase::Drift: return "Abflug";
        case DeparturePhase::Complete: return "Abflug abgeschlossen";
    }
    return "";
}

DepartureGuidance::DepartureGuidance(DepartureParameters departure, ProximityParameters proximity)
    : d_(departure), p_(std::move(proximity)) {
    p_.breakout.ellipsoid_semi_axes_m = d_.ellipsoid_semi_axes_m;
    p_.breakout.ellipsoid_exit_limit_s = d_.ellipsoid_exit_limit_s;
}

void DepartureGuidance::enter(DeparturePhase phase, std::string status, time::Epoch epoch) {
    phase_ = phase;
    status_ = status;
    events_.push_back({epoch, phase, std::move(status)});
    loop_.reset();
}

math::Vec3 DepartureGuidance::axial_velocity_port(const ProximityInput& in) const {
    math::Vec3 v = approach_velocity_port(in.port.position_m, p_);  // lateral convergence onto the axis
    v.x = d_.undock_speed_mps;
    return v;
}

ProximityOutput DepartureGuidance::update(const ProximityInput& in) {
    ProximityOutput out;
    if (!started_) {
        started_ = true;
        enter(DeparturePhase::UndockBurn, to_string(DeparturePhase::UndockBurn), in.epoch);
    }
    switch (phase_) {
        case DeparturePhase::UndockBurn:
        case DeparturePhase::BackOut: {
            if (phase_ == DeparturePhase::UndockBurn &&
                in.port.velocity_mps.x >= d_.undock_speed_mps - d_.undock_tolerance_mps) {
                enter(DeparturePhase::BackOut, to_string(DeparturePhase::BackOut), in.epoch);
            }
            if (phase_ == DeparturePhase::BackOut && in.port.position_m.x >= d_.burn_range_m) {
                burn_ = breakout_delta_v(in.cg_lvlh, in.mean_motion_radps, p_);
                burn_start_lvlh_ = {in.cg_lvlh.position_m, in.cg_lvlh.velocity_mps + burn_->delta_v_lvlh_mps};
                burn_epoch_ = in.epoch;
                enter(DeparturePhase::DepartureBurn,
                      burn_->safe ? "Abflugmanöver" : "Abflugmanöver: nicht passiv sicher", in.epoch);
                break;
            }
            out.translation_fraction_port =
                loop_.command(axial_velocity_port(in), in.port.velocity_mps, in.authority_port, p_.loop_near);
            return out;
        }
        default:
            break;
    }
    if (phase_ == DeparturePhase::DepartureBurn) {
        const math::Mat3 to_port = transpose(in.port_to_lvlh);
        const math::Vec3 target =
            orbit::cw_propagate(burn_start_lvlh_, in.mean_motion_radps, in.epoch - burn_epoch_).velocity_mps;
        const math::Vec3 error = target - in.cg_lvlh.velocity_mps;
        if (norm(error) < d_.burn_tolerance_mps && !loop_.firing()) {
            enter(DeparturePhase::Drift, burn_->safe ? "Abflug" : "Abflug: nicht passiv sicher", in.epoch);
        } else {
            out.translation_fraction_port = loop_.command(to_port * target, to_port * in.cg_lvlh.velocity_mps,
                                                          in.authority_port, p_.loop_far);
            return out;
        }
    }
    if (phase_ == DeparturePhase::Drift) {
        const math::Vec3 r = in.cg_lvlh.position_m;
        const math::Vec3 a = d_.ellipsoid_semi_axes_m;
        if ((r.x / a.x) * (r.x / a.x) + (r.y / a.y) * (r.y / a.y) + (r.z / a.z) * (r.z / a.z) > 1.0) {
            enter(DeparturePhase::Complete, to_string(DeparturePhase::Complete), in.epoch);
        }
    }
    out.attitude_hold = false;  // drifting: the session may coast
    return out;
}

}  // namespace starsim::gnc
