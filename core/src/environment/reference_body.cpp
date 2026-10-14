#include "starsim/environment/reference_body.hpp"

#include "starsim/math/constants.hpp"

namespace starsim::environment {

ReferenceBody ReferenceBody::earth(const frames::BodyOrientation& itrf, const Atmosphere& atmosphere, double gm_m3ps2) {
    return {ephemeris::Body::Earth, gm_m3ps2, frames::wgs84, itrf, &atmosphere, nullptr};
}

ReferenceBody ReferenceBody::moon(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& moon_me) {
    return {ephemeris::Body::Moon, ephemeris.gm_m3ps2(ephemeris::Body::Moon),
            frames::Ellipsoid{constants::moon_mean_radius_m, 0.0}, moon_me, nullptr, &ephemeris};
}

math::StateVector ReferenceBody::centre_gcrf(time::Epoch epoch) const {
    return ephemeris_ ? ephemeris_->state_gcrf(id_, epoch) : math::StateVector{};
}

BodyFrame ReferenceBody::frame(time::Epoch epoch) const {
    const auto transform = orientation_->fixed_to_gcrf(epoch);
    const auto centre = centre_gcrf(epoch);
    return {transform, centre.position_m, centre.velocity_mps, frames::rotation_vector(transform)};
}

}  // namespace starsim::environment
