#include "starsim/environment/earth_terrain.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Vec3;

namespace {

// Weight of a finer grid at `edge_distance_m` inside its edge: 0 at the edge, 1 from blend_m inward (smoothstep, so
// the blend adds no kink to the normals at either end).
double fine_weight(double edge_distance_m) {
    const double x = std::clamp(edge_distance_m / EarthTerrain::blend_m, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

// The first grid of `grids` that covers the point, blended into `coarser` towards its edge; `coarser` alone elsewhere.
template <typename Coarser>
double blended(const std::vector<GeographicGrid>& grids, double latitude_rad, double longitude_rad, Coarser coarser) {
    for (const auto& g : grids) {
        if (!g.covers(latitude_rad, longitude_rad)) continue;
        const double w = fine_weight(g.edge_distance_m(latitude_rad, longitude_rad, frames::wgs84.semi_major_axis_m));
        const double fine = g.value(latitude_rad, longitude_rad);
        return w == 1.0 ? fine : w * fine + (1.0 - w) * coarser();
    }
    return coarser();
}

}  // namespace

EarthTerrain::EarthTerrain(Geoid geoid, GeographicGrid global_relief, std::vector<GeographicGrid> regional,
                           std::vector<GeographicGrid> local, std::vector<Plateau> plateaus)
    : geoid_(std::move(geoid)), global_(std::move(global_relief)), regional_(std::move(regional)), local_(std::move(local)),
      plateaus_(std::move(plateaus)) {
    plateau_centres_fixed_m_.reserve(plateaus_.size());
    for (const auto& p : plateaus_) {
        plateau_centres_fixed_m_.push_back(frames::geodetic_to_fixed({p.latitude_rad, p.longitude_rad, 0.0}));
    }
}

double EarthTerrain::terrain_m(double latitude_rad, double longitude_rad) const {
    const auto global = [&] { return global_.value(latitude_rad, longitude_rad); };
    const auto regional = [&] { return blended(regional_, latitude_rad, longitude_rad, global); };
    return blended(local_, latitude_rad, longitude_rad, regional);
}

EarthTerrain::Ground EarthTerrain::ground(double latitude_rad, double longitude_rad) const {
    const double terrain = terrain_m(latitude_rad, longitude_rad);
    const double n = geoid_.undulation_m(latitude_rad, longitude_rad);
    if (terrain - n <= water_threshold_m) return {n, true};
    return {terrain, false};
}

EarthTerrain::PlateauHit EarthTerrain::plateau_at(double latitude_rad, double longitude_rad) const {
    PlateauHit hit;
    if (plateaus_.empty()) return hit;
    const Vec3 p = frames::geodetic_to_fixed({latitude_rad, longitude_rad, 0.0});
    for (size_t i = 0; i < plateaus_.size(); ++i) {
        const auto& plateau = plateaus_[i];
        const double d = norm(p - plateau_centres_fixed_m_[i]);  // chord: the arc within 1e-8 m at 200 m
        if (d >= plateau.radius_m + plateau.apron_m) continue;
        const double fraction = d <= plateau.radius_m ? 0.0 : (d - plateau.radius_m) / plateau.apron_m;
        if (!hit.plateau || fraction < hit.apron_fraction) hit = {&plateau, fraction};  // overlap: the nearer pad
    }
    return hit;
}

double EarthTerrain::height_ellipsoid_m(double latitude_rad, double longitude_rad) const {
    const auto hit = plateau_at(latitude_rad, longitude_rad);
    if (hit.plateau && hit.apron_fraction == 0.0) return hit.plateau->height_ellipsoid_m;
    const double below = ground(latitude_rad, longitude_rad).height_ellipsoid_m;
    if (!hit.plateau) return below;
    return hit.plateau->height_ellipsoid_m + hit.apron_fraction * (below - hit.plateau->height_ellipsoid_m);
}

bool EarthTerrain::water(double latitude_rad, double longitude_rad) const {
    if (plateau_at(latitude_rad, longitude_rad).plateau) return false;
    return ground(latitude_rad, longitude_rad).water;
}

double EarthTerrain::height_at_fixed_m(const Vec3& fixed_m) const {
    const auto g = frames::fixed_to_geodetic(fixed_m);
    return height_ellipsoid_m(g.latitude_rad, g.longitude_rad);
}

bool EarthTerrain::water_at_fixed(const Vec3& fixed_m) const {
    const auto g = frames::fixed_to_geodetic(fixed_m);
    return water(g.latitude_rad, g.longitude_rad);
}

Vec3 EarthTerrain::surface_point(double latitude_rad, double longitude_rad) const {
    return frames::geodetic_to_fixed({latitude_rad, longitude_rad, height_ellipsoid_m(latitude_rad, longitude_rad)});
}

Vec3 EarthTerrain::normal_fixed(const Vec3& fixed_m) const {
    const auto g = frames::fixed_to_geodetic(fixed_m);
    const double lat = g.latitude_rad, lon = g.longitude_rad;
    const double d_lat = normal_step_m / frames::wgs84.semi_major_axis_m;
    const double d_lon = d_lat / std::max(std::cos(lat), 1e-6);
    const Vec3 east = surface_point(lat, lon + d_lon) - surface_point(lat, lon - d_lon);
    const Vec3 north = surface_point(lat + d_lat, lon) - surface_point(lat - d_lat, lon);
    const Vec3 n = math::normalized(cross(east, north));
    const Vec3 up{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    return dot(n, up) >= 0.0 ? n : -n;
}

}  // namespace starsim::environment
