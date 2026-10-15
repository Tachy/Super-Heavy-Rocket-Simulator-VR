#include "starsim/environment/reference_body.hpp"

#include <cmath>

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

math::Vec3 ReferenceBody::centre_acceleration_gcrf(time::Epoch epoch) const {
    constexpr double step_s = 60.0;  // [E] central difference over +-60 s (the Moon's acceleration turns 2.7e-6 rad/s)
    if (!ephemeris_) return {};
    const auto after = ephemeris_->state_gcrf(id_, epoch + step_s);
    const auto before = ephemeris_->state_gcrf(id_, epoch - step_s);
    return (after.velocity_mps - before.velocity_mps) / (2.0 * step_s);
}

double ReferenceBody::surface_height_m(const math::Vec3& fixed_m) const {
    return terrain_ ? terrain_->height_at_fixed_m(fixed_m) : 0.0;
}

math::Vec3 ReferenceBody::surface_normal_fixed(const math::Vec3& fixed_m) const {
    if (terrain_) return terrain_->normal_fixed(fixed_m);
    if (shape_.flattening == 0.0) return math::normalized(fixed_m);
    const auto g = frames::fixed_to_geodetic(fixed_m, shape_);
    return {std::cos(g.latitude_rad) * std::cos(g.longitude_rad), std::cos(g.latitude_rad) * std::sin(g.longitude_rad),
            std::sin(g.latitude_rad)};
}

BodyFrame ReferenceBody::frame(time::Epoch epoch) const {
    const auto transform = orientation_->fixed_to_gcrf(epoch);
    const auto centre = centre_gcrf(epoch);
    return {transform, centre.position_m, centre.velocity_mps, frames::rotation_vector(transform)};
}

}  // namespace starsim::environment
