#include <cmath>
#include <stdexcept>

#include "starsim/environment/gravity.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

using ephemeris::Body;
using math::Vec3;

namespace {

Vec3 point_mass(double gm, const Vec3& r) {
    const double r2 = dot(r, r);
    return r * (-gm / (r2 * std::sqrt(r2)));
}

}  // namespace

PointMassGravity::PointMassGravity(const ephemeris::Ephemeris& ephemeris,
                                   const frames::BodyOrientation& earth_orientation, PointMassGravityConfig config)
    : ephemeris_(ephemeris),
      earth_orientation_(earth_orientation),
      config_(config),
      gm_earth_(ephemeris.gm_m3ps2(Body::Earth)),
      gm_moon_(ephemeris.gm_m3ps2(Body::Moon)),
      gm_sun_(ephemeris.gm_m3ps2(Body::Sun)) {
    if (config_.central_body == Body::Earth) {
        gm_central_ = gm_earth_;
    } else if (config_.central_body == Body::Moon) {
        gm_central_ = gm_moon_;
    } else {
        throw std::invalid_argument("PointMassGravity: central body must be Earth or Moon");
    }
}

Vec3 PointMassGravity::acceleration_mps2(const Vec3& position_m, time::Epoch epoch) const {
    Vec3 acceleration = point_mass(gm_central_, position_m);
    if (config_.central_body == Body::Earth) {
        if (config_.earth_j2) {
            acceleration += earth_j2(position_m, epoch);
        }
        if (config_.moon) {
            acceleration += third_body(Body::Moon, gm_moon_, position_m, epoch);
        }
    } else if (config_.earth) {
        acceleration += third_body(Body::Earth, gm_earth_, position_m, epoch);
    }
    if (config_.sun) {
        acceleration += third_body(Body::Sun, gm_sun_, position_m, epoch);
    }
    return acceleration;
}

// Direct attraction of the third body minus its attraction on the central body.
Vec3 PointMassGravity::third_body(Body body, double gm, const Vec3& position_m, time::Epoch epoch) const {
    Vec3 body_m = ephemeris_.state_gcrf(body, epoch).position_m;  // relative to Earth
    if (config_.central_body == Body::Moon) {
        body_m -= ephemeris_.state_gcrf(Body::Moon, epoch).position_m;
    }
    const Vec3 to_body = body_m - position_m;
    const double d2 = dot(to_body, to_body);
    const double b2 = dot(body_m, body_m);
    return gm * (to_body / (d2 * std::sqrt(d2)) - body_m / (b2 * std::sqrt(b2)));
}

// a = -3/2 J2 mu R^2 / r^5 * [ (1 - 5 z^2/r^2) r + 2 z p ],  z = r . p, p = Earth pole (unit).
Vec3 PointMassGravity::earth_j2(const Vec3& position_m, time::Epoch epoch) const {
    const auto rotation = earth_orientation_.fixed_to_gcrf(epoch).rotation;
    const Vec3 pole{rotation(0, 2), rotation(1, 2), rotation(2, 2)};
    const double r2 = dot(position_m, position_m);
    const double r = std::sqrt(r2);
    const double z = dot(position_m, pole);
    const double factor = -1.5 * constants::earth_j2 * gm_earth_ * earth_j2_reference_radius_m *
                          earth_j2_reference_radius_m / (r2 * r2 * r);
    return factor * ((1.0 - 5.0 * z * z / r2) * position_m + 2.0 * z * pole);
}

}  // namespace starsim::environment
