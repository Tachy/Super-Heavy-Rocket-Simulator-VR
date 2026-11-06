#include "starsim/mission/lunar_site_sun.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::mission {

namespace {

// The Sun moves ~1 deg per 2 h. A grid step holds at most one crossing; shorter grazing intervals near the poles
// (peak or trough of the elevation curve between two grid samples) are caught by the extremum refinement.
constexpr double scan_step_s = 7200.0;
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
    // Bisection between a sample below and one above the horizon (or vice versa), `lo_up` = state at lo.
    const auto crossing = [&](time::Epoch lo, time::Epoch hi, bool lo_up) {
        while (hi - lo > bisect_tolerance_s) {
            const time::Epoch mid = lo + 0.5 * (hi - lo);
            ((elevation(mid) > 0.0) == lo_up ? lo : hi) = mid;
        }
        return lo + 0.5 * (hi - lo);
    };
    // Extremum of the elevation inside (lo, hi) by ternary search (unimodal over two grid steps); `sign` = +1 for a
    // maximum, -1 for a minimum. Returns the time and the elevation there.
    const auto extremum = [&](time::Epoch lo, time::Epoch hi, double sign) {
        while (hi - lo > bisect_tolerance_s) {
            const time::Epoch a = lo + (hi - lo) / 3.0, b = hi - (hi - lo) / 3.0;
            if (sign * elevation(a) < sign * elevation(b)) {
                lo = a;
            } else {
                hi = b;
            }
        }
        const time::Epoch t = lo + 0.5 * (hi - lo);
        return std::pair<time::Epoch, double>{t, elevation(t)};
    };

    std::vector<DaylightInterval> result;
    if (span_s <= 0.0) return result;
    const time::Epoch end = from + span_s;

    // Coarse grid of (time, elevation).
    std::vector<std::pair<time::Epoch, double>> grid;
    for (time::Epoch t = from; t < end; t = (end - t > scan_step_s) ? t + scan_step_s : end) {
        grid.emplace_back(t, elevation(t));
    }
    grid.emplace_back(end, elevation(end));

    // Near the poles the elevation curve only grazes the horizon: a short day (or short night) is a peak (or trough)
    // between two grid samples of the same sign. Insert the extremum as a sample when it lies on the other side.
    std::vector<std::pair<time::Epoch, double>> samples;
    samples.push_back(grid.front());
    for (std::size_t i = 1; i + 1 < grid.size(); ++i) {
        const double before = grid[i - 1].second, here = grid[i].second, after = grid[i + 1].second;
        const bool same_side = (before > 0.0) == (here > 0.0) && (here > 0.0) == (after > 0.0);
        if (same_side && (here - before) * (after - here) < 0.0) {
            const double sign = here > 0.0 ? -1.0 : 1.0;  // trough inside a day, peak inside a night
            const auto [t, value] = extremum(grid[i - 1].first, grid[i + 1].first, sign);
            if ((value > 0.0) != (here > 0.0)) {
                if (t < grid[i].first) {
                    samples.emplace_back(t, value);
                    samples.push_back(grid[i]);
                } else {
                    samples.push_back(grid[i]);
                    samples.emplace_back(t, value);
                }
                continue;
            }
        }
        samples.push_back(grid[i]);
    }
    if (grid.size() > 1) samples.push_back(grid.back());

    bool up = samples.front().second > 0.0;
    time::Epoch rise = from;
    for (std::size_t i = 1; i < samples.size(); ++i) {
        const bool next_up = samples[i].second > 0.0;
        if (next_up == up) continue;
        const time::Epoch t = crossing(samples[i - 1].first, samples[i].first, up);
        if (next_up) {
            rise = t;
        } else {
            result.push_back({rise, t});
        }
        up = next_up;
    }
    if (up) result.push_back({rise, end});
    return result;
}

}  // namespace starsim::mission
