#include "starsim/mission/lunar_site_sun.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::mission {

namespace {

constexpr double scan_step_s = 7200.0;  // the Sun moves ~1 deg per 2 h; one crossing per step at most
constexpr double bisect_tolerance_s = 1.0;

}  // namespace

SiteSun site_sun(const ephemeris::Ephemeris& ephemeris, const frames::BodyOrientation& moon, double latitude_rad,
                 double longitude_rad, time::Epoch epoch) {
    using math::Vec3;
    const double sin_lat = std::sin(latitude_rad), cos_lat = std::cos(latitude_rad);
    const double sin_lon = std::sin(longitude_rad), cos_lon = std::cos(longitude_rad);
    const Vec3 up_fixed{cos_lat * cos_lon, cos_lat * sin_lon, sin_lat};
    const Vec3 east_fixed{-sin_lon, cos_lon, 0.0};
    const Vec3 north_fixed{-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat};

    const frames::FixedToInertial to_gcrf = moon.fixed_to_gcrf(epoch);
    const math::StateVector moon_centre = ephemeris.state_gcrf(ephemeris::Body::Moon, epoch);
    const Vec3 site_fixed_m = up_fixed * constants::moon_mean_radius_m;
    const math::StateVector site{moon_centre.position_m + to_gcrf.position(site_fixed_m),
                                 moon_centre.velocity_mps + to_gcrf.velocity(site_fixed_m, Vec3{})};

    const Vec3 sun_gcrf = ephemeris.apparent_position_gcrf(ephemeris::Body::Sun, epoch, site);
    const Vec3 sun_fixed = transpose(to_gcrf.rotation) * sun_gcrf;
    const Vec3 direction = math::normalized(sun_fixed);
    SiteSun result;
    result.elevation_rad = std::asin(std::clamp(dot(direction, up_fixed), -1.0, 1.0));
    result.azimuth_rad = std::atan2(dot(direction, east_fixed), dot(direction, north_fixed));
    return result;
}

std::vector<DaylightInterval> daylight_intervals(const ephemeris::Ephemeris& ephemeris,
                                                 const frames::BodyOrientation& moon, double latitude_rad,
                                                 double longitude_rad, time::Epoch from, double span_s) {
    const auto elevation = [&](time::Epoch t) {
        return site_sun(ephemeris, moon, latitude_rad, longitude_rad, t).elevation_rad;
    };
    const auto crossing = [&](time::Epoch lo, time::Epoch hi) {  // sign of the elevation differs at lo and hi
        const bool lo_up = elevation(lo) > 0.0;
        while (hi - lo > bisect_tolerance_s) {
            const time::Epoch mid = lo + 0.5 * (hi - lo);
            ((elevation(mid) > 0.0) == lo_up ? lo : hi) = mid;
        }
        return lo + 0.5 * (hi - lo);
    };

    std::vector<DaylightInterval> result;
    if (span_s <= 0.0) return result;
    const time::Epoch end = from + span_s;
    bool up = elevation(from) > 0.0;
    time::Epoch rise = from;
    time::Epoch previous = from;
    while (previous < end) {
        const time::Epoch next = (end - previous > scan_step_s) ? previous + scan_step_s : end;
        const bool next_up = elevation(next) > 0.0;
        if (next_up != up) {
            const time::Epoch t = crossing(previous, next);
            if (next_up) {
                rise = t;
            } else {
                result.push_back({rise, t});
            }
            up = next_up;
        }
        previous = next;
    }
    if (up) result.push_back({rise, end});
    return result;
}

}  // namespace starsim::mission
