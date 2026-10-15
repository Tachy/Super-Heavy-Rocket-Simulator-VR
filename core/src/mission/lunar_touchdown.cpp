// Touchdown point selection for the lunar descent (declared in lunar_descent.hpp).

#include <algorithm>
#include <cmath>
#include <vector>

#include "starsim/mission/lunar_descent.hpp"

namespace starsim::mission {

namespace {
constexpr int footprint_samples = 12;  // points on the leg circle (independent of the vehicle's leg count)

struct LatLon {
    double latitude_rad;
    double longitude_rad;
};

// Point `east_m` / `north_m` away from (lat, lon) in the local horizontal plane on the sphere of `radius_m`
// (local ENU basis, back to latitude/longitude with atan2).
LatLon offset_position(const LatLon& at, double east_m, double north_m, double radius_m) {
    const double sl = std::sin(at.latitude_rad), cl = std::cos(at.latitude_rad);
    const double so = std::sin(at.longitude_rad), co = std::cos(at.longitude_rad);
    const math::Vec3 up{cl * co, cl * so, sl};
    const math::Vec3 east{-so, co, 0.0};
    const math::Vec3 north{-sl * co, -sl * so, cl};
    const math::Vec3 p = up * radius_m + east * east_m + north * north_m;
    return {std::atan2(p.z, std::hypot(p.x, p.y)), std::atan2(p.y, p.x)};
}

struct FootprintFit {
    double slope_rad;
    double roughness_m;
    double centre_height_m;
};

// Least-squares plane z = a e + b n + c through the centre and the circle samples. The layout is symmetric
// (sum e = sum n = sum e n = 0, sum e^2 = sum n^2), so the normal equations decouple.
FootprintFit fit_footprint(const environment::LunarTerrain& terrain, const LatLon& centre, double radius_m,
                           double sphere_m) {
    std::vector<math::Vec3> points;  // (east, north, height)
    points.push_back({0.0, 0.0, terrain.height_m(centre.latitude_rad, centre.longitude_rad)});
    for (int k = 0; k < footprint_samples; ++k) {
        const double az = constants::two_pi * k / footprint_samples;
        const double e = radius_m * std::sin(az), n = radius_m * std::cos(az);
        const auto q = offset_position(centre, e, n, sphere_m + points[0].z);
        points.push_back({e, n, terrain.height_m(q.latitude_rad, q.longitude_rad)});
    }
    double sez = 0.0, snz = 0.0, see = 0.0, snn = 0.0, sz = 0.0;
    for (const auto& p : points) {
        sez += p.x * p.z;
        snz += p.y * p.z;
        see += p.x * p.x;
        snn += p.y * p.y;
        sz += p.z;
    }
    const double a = sez / see, b = snz / snn, c = sz / static_cast<double>(points.size());
    double roughness = 0.0;
    for (const auto& p : points) roughness = std::max(roughness, std::abs(p.z - (a * p.x + b * p.y + c)));
    return {std::atan(std::hypot(a, b)), roughness, points[0].z};
}

// Terrain uprange of the point (against the approach azimuth) stays below the glide cone by the margin.
bool glide_cone_clear(const environment::LunarTerrain& terrain, const LatLon& centre, double centre_height_m,
                      double approach_azimuth_rad, double sphere_m, const LunarDescentParameters& p) {
    constexpr double first_m = 100.0, step_m = 20.0;  // [E] from beyond the footprint, one pixel apart
    const double tan_slope = std::tan(p.glide_slope_rad);
    for (double d = first_m; d <= p.glide_check_range_m; d += step_m) {
        const auto q = offset_position(centre, -d * std::sin(approach_azimuth_rad), -d * std::cos(approach_azimuth_rad),
                                       sphere_m + centre_height_m);
        if (terrain.height_m(q.latitude_rad, q.longitude_rad) - centre_height_m > d * tan_slope - p.glide_margin_m) {
            return false;
        }
    }
    return true;
}
}  // namespace

std::optional<TouchdownPoint> select_touchdown_point(const environment::LunarTerrain& terrain, double latitude_rad,
                                                     double longitude_rad, double approach_azimuth_rad,
                                                     double footprint_radius_m, const LunarDescentParameters& p) {
    struct Candidate {
        double offset_m;
        double east_m;
        double north_m;
    };
    std::vector<Candidate> candidates;
    const int n = static_cast<int>(std::floor(p.touchdown_search_radius_m / p.touchdown_grid_m));
    for (int i = -n; i <= n; ++i) {
        for (int j = -n; j <= n; ++j) {
            const double e = i * p.touchdown_grid_m, nn = j * p.touchdown_grid_m;
            const double d = std::hypot(e, nn);
            if (d <= p.touchdown_search_radius_m) candidates.push_back({d, e, nn});
        }
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const Candidate& a, const Candidate& b) { return a.offset_m < b.offset_m; });

    const LatLon site{latitude_rad, longitude_rad};
    const double sphere_m = terrain.sphere_radius_m();
    const double site_height_m = terrain.height_m(latitude_rad, longitude_rad);
    for (const auto& c : candidates) {
        const auto centre = offset_position(site, c.east_m, c.north_m, sphere_m + site_height_m);
        const auto fit = fit_footprint(terrain, centre, footprint_radius_m, sphere_m);
        if (fit.slope_rad > p.slope_limit_rad || fit.roughness_m > p.roughness_limit_m) continue;
        if (!glide_cone_clear(terrain, centre, fit.centre_height_m, approach_azimuth_rad, sphere_m, p)) continue;
        return TouchdownPoint{centre.latitude_rad, centre.longitude_rad, fit.centre_height_m, fit.slope_rad,
                              c.offset_m};
    }
    return std::nullopt;
}

}  // namespace starsim::mission
