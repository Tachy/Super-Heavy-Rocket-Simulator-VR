#include "starsim/gnc/orbital_attitude.hpp"

#include "starsim/frames/lvlh.hpp"

namespace starsim::gnc {

using math::Quat;
using math::Vec3;

Quat lvlh_attitude_gcrf(const math::StateVector& state) { return frames::lvlh_to_inertial(state); }

AttitudeTarget attitude_target(const AttitudeMode& mode, const math::StateVector& state) {
    const Vec3 x{1.0, 0.0, 0.0}, y{0.0, 1.0, 0.0};
    switch (mode.kind) {
        case AttitudeModeKind::Inertial:
            return {mode.attitude_gcrf.rotate(x), mode.attitude_gcrf.rotate(y), {}};
        case AttitudeModeKind::Lvlh: {
            const Quat q = lvlh_attitude_gcrf(state) * mode.body_in_lvlh;
            // The radius vector turns at (r x v) / r^2: the orbit rate on a circular orbit.
            const double r2 = dot(state.position_m, state.position_m);
            return {q.rotate(x), q.rotate(y), cross(state.position_m, state.velocity_mps) / r2};
        }
        case AttitudeModeKind::Direction:
        case AttitudeModeKind::Sun:
            return {math::normalized(mode.direction_gcrf), math::normalized(cross(state.position_m, state.velocity_mps)),
                    {}};
        case AttitudeModeKind::Free:
            break;
    }
    return {};
}

AttitudeMode inertial_hold(const Quat& attitude_gcrf) {
    AttitudeMode m;
    m.kind = AttitudeModeKind::Inertial;
    m.attitude_gcrf = attitude_gcrf;
    return m;
}

AttitudeMode lvlh_hold(const Quat& body_in_lvlh) {
    AttitudeMode m;
    m.kind = AttitudeModeKind::Lvlh;
    m.body_in_lvlh = body_in_lvlh;
    return m;
}

}  // namespace starsim::gnc
