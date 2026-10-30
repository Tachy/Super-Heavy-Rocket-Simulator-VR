#include "starsim/environment/terrain_horizon.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

namespace {

using math::Vec3;

// [E] Growth of the march step per sample (2 % of the distance). Chosen so the march stays within 0.1 deg of a
// brute-force march with half-pixel steps on the LDEM_16 and the 20 m site tile (test_terrain_horizon.cpp).
constexpr double step_growth = 1.02;
// [E] Largest step: half the 1.9 km spacing of LDEM_16, so that far ridges are not stepped over.
constexpr double max_step_m = 1000.0;

// Local frame of a point on the unit sphere: up, east, north.
struct LocalFrame {
    Vec3 up, east, north;
};

LocalFrame local_frame(double latitude_rad, double longitude_rad) {
    const double cl = std::cos(latitude_rad), sl = std::sin(latitude_rad);
    const double co = std::cos(longitude_rad), so = std::sin(longitude_rad);
    return {Vec3{cl * co, cl * so, sl}, Vec3{-so, co, 0.0}, Vec3{-sl * co, -sl * so, cl}};
}

// Highest elevation any terrain at angular distance >= t can reach, seen from radius r0, when it is no higher than
// radius r1 (elevation of a point on that sphere: atan2(r1 cos t - r0, r1 sin t); falling with t, except for an
// observer above r1, where it first rises to the tangent point).
double elevation_bound_rad(double r0, double r1, double t) {
    const double k = r0 / r1;
    if (k > 1.0 && std::cos(t) > 1.0 / k) return -std::acos(1.0 / k);
    return std::atan2(r1 * std::cos(t) - r0, r1 * std::sin(t));
}

double march_ray(const Terrain& terrain, double sphere_radius_m, const LocalFrame& frame, double height_m,
                 double azimuth_rad, const HorizonMarch& march) {
    const double r0 = sphere_radius_m + height_m;
    const double r1_max = sphere_radius_m + std::max(terrain.max_height_m(), 0.0);
    const Vec3 direction = frame.north * std::cos(azimuth_rad) + frame.east * std::sin(azimuth_rad);
    double best = -std::numeric_limits<double>::infinity();
    double distance_m = march.min_distance_m > 0.0 ? march.min_distance_m : march.first_step_m;
    double step_m = march.first_step_m;
    for (;;) {
        const double t = distance_m / sphere_radius_m;
        if (t >= constants::pi) break;
        const double ct = std::cos(t), st = std::sin(t);
        const Vec3 target = frame.up * ct + direction * st;
        const double latitude = std::asin(std::clamp(target.z, -1.0, 1.0));
        const double longitude = std::atan2(target.y, target.x);
        const double r1 = sphere_radius_m + terrain.height_m(latitude, longitude);
        best = std::max(best, std::atan2(r1 * ct - r0, r1 * st));
        distance_m += step_m;
        step_m = std::min(step_m * step_growth, max_step_m);
        const double t_next = distance_m / sphere_radius_m;
        if (t_next >= constants::pi || elevation_bound_rad(r0, r1_max, t_next) <= best) break;
    }
    return best;
}

}  // namespace

double horizon_elevation_rad(const Terrain& terrain, double sphere_radius_m, double latitude_rad, double longitude_rad,
                             double height_m, double azimuth_rad, const HorizonMarch& march) {
    return march_ray(terrain, sphere_radius_m, local_frame(latitude_rad, longitude_rad), height_m, azimuth_rad, march);
}

std::vector<float> horizon_elevations(const Terrain& terrain, double sphere_radius_m,
                                      const std::vector<double>& latitude_rad, const std::vector<double>& longitude_rad,
                                      const std::vector<double>& height_m, int azimuth_count, const HorizonMarch& march) {
    if (latitude_rad.size() != longitude_rad.size() || latitude_rad.size() != height_m.size() || azimuth_count < 1) {
        throw std::invalid_argument("horizon_elevations: point arrays differ in length or azimuth_count < 1");
    }
    std::vector<float> result(latitude_rad.size() * static_cast<size_t>(azimuth_count));
    for (size_t i = 0; i < latitude_rad.size(); ++i) {
        const LocalFrame frame = local_frame(latitude_rad[i], longitude_rad[i]);
        for (int k = 0; k < azimuth_count; ++k) {
            const double azimuth = k * 2.0 * constants::pi / azimuth_count;
            result[i * static_cast<size_t>(azimuth_count) + static_cast<size_t>(k)] =
                static_cast<float>(march_ray(terrain, sphere_radius_m, frame, height_m[i], azimuth, march));
        }
    }
    return result;
}

}  // namespace starsim::environment
