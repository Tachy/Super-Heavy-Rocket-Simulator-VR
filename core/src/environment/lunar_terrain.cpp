#include "starsim/environment/lunar_terrain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Vec3;

LunarTerrain::LunarTerrain(CylindricalGrid global, std::vector<std::shared_ptr<const LunarTile>> tiles,
                           double sphere_radius_m)
    : LunarTerrain(std::make_shared<const CylindricalGrid>(std::move(global)), std::move(tiles), sphere_radius_m) {}

LunarTerrain::LunarTerrain(std::shared_ptr<const CylindricalGrid> global,
                           std::vector<std::shared_ptr<const LunarTile>> tiles, double sphere_radius_m)
    : global_(std::move(global)), tiles_(std::move(tiles)), sphere_radius_m_(sphere_radius_m) {
    max_height_m_ = global_->grid.dn.empty() ? 0.0
                                             : global_->grid.dn_scale_m *
                                                   *std::max_element(global_->grid.dn.begin(), global_->grid.dn.end());
    for (const auto& tile : tiles_) max_height_m_ = std::max(max_height_m_, tile->max_height_m());
}

LunarTerrain::LunarTerrain(CylindricalGrid global, std::vector<PolarStereographicTile> south_tiles,
                           double sphere_radius_m)
    : LunarTerrain(std::move(global),
                   [&] {
                       std::vector<std::shared_ptr<const LunarTile>> tiles;
                       for (auto& t : south_tiles) tiles.push_back(make_polar_tile(std::move(t), false));
                       return tiles;
                   }(),
                   sphere_radius_m) {}

namespace {
Vec3 direction_of(double latitude_rad, double longitude_rad) {
    const double c = std::cos(latitude_rad);
    return {c * std::cos(longitude_rad), c * std::sin(longitude_rad), std::sin(latitude_rad)};
}
double smoothstep(double x) { return x >= 1.0 ? 1.0 : x * x * (3.0 - 2.0 * x); }
}  // namespace

double LunarTerrain::height_m(double latitude_rad, double longitude_rad) const {
    const Vec3 unit = direction_of(latitude_rad, longitude_rad);
    for (const auto& tile : tiles_) {
        const double edge_m = tile->edge_distance_m(unit);
        if (!(edge_m >= 0.0)) continue;
        const double tile_h = tile->height_m(unit);
        const double w = smoothstep(edge_m / blend_m);  // the tile's core exactly, the grid at its edge
        return w == 1.0 ? tile_h : w * tile_h + (1.0 - w) * global_->height_m(latitude_rad, longitude_rad);
    }
    return global_->height_m(latitude_rad, longitude_rad);
}

double LunarTerrain::height_at_direction_m(const Vec3& unit) const {
    for (const auto& tile : tiles_) {
        const double edge_m = tile->edge_distance_m(unit);
        if (!(edge_m >= 0.0)) continue;
        const double tile_h = tile->height_m(unit);
        const double x = edge_m / blend_m;
        if (x >= 1.0) return tile_h;  // the tile's core: no global lookup
        const double w = smoothstep(x);
        return w * tile_h + (1.0 - w) * global_->height_at_direction(unit);
    }
    return global_->height_at_direction(unit);
}

double LunarTerrain::resolution_m(double latitude_rad, double longitude_rad) const {
    const Vec3 unit = direction_of(latitude_rad, longitude_rad);
    for (const auto& tile : tiles_) {
        if (tile->contains(unit)) return tile->scale_m();
    }
    return global_resolution_m();
}

double LunarTerrain::finest_resolution_within_m(double latitude_rad, double longitude_rad, double radius_m) const {
    double finest = resolution_m(latitude_rad, longitude_rad);
    const Vec3 unit = direction_of(latitude_rad, longitude_rad);
    for (const auto& tile : tiles_) {
        if (tile->outside_distance_m(unit) <= radius_m) finest = std::min(finest, tile->scale_m());
    }
    return finest;
}

bool LunarTerrain::in_tile(double latitude_rad, double longitude_rad) const {
    const Vec3 unit = direction_of(latitude_rad, longitude_rad);
    for (const auto& tile : tiles_) {
        if (tile->contains(unit)) return true;
    }
    return false;
}

LunarTerrain::TileBlend LunarTerrain::tile_blend(const Vec3& unit) const {
    for (const auto& tile : tiles_) {
        const double edge_m = tile->edge_distance_m(unit);
        if (!(edge_m >= 0.0)) continue;
        return {smoothstep(edge_m / blend_m), tile->scale_m()};
    }
    return {};
}

double LunarTerrain::global_resolution_m() const {
    return sphere_radius_m_ * constants::deg_to_rad / global_->pixels_per_degree;
}

double LunarTerrain::finest_resolution_m() const {
    double finest = global_resolution_m();
    for (const auto& tile : tiles_) finest = std::min(finest, tile->scale_m());
    return finest;
}

double LunarTerrain::height_at_fixed_m(const Vec3& fixed_m) const {
    const double r = norm(fixed_m);
    return height_m(std::asin(fixed_m.z / r), std::atan2(fixed_m.y, fixed_m.x));
}

Vec3 LunarTerrain::surface_point(double latitude_rad, double longitude_rad) const {
    const double r = sphere_radius_m_ + height_m(latitude_rad, longitude_rad);
    return Vec3{std::cos(latitude_rad) * std::cos(longitude_rad), std::cos(latitude_rad) * std::sin(longitude_rad),
                std::sin(latitude_rad)} *
           r;
}

Vec3 LunarTerrain::normal_fixed(const Vec3& fixed_m) const {
    const double lat = std::asin(fixed_m.z / norm(fixed_m));
    const double lon = std::atan2(fixed_m.y, fixed_m.x);
    const double d_lat = normal_step_m / sphere_radius_m_;
    const double d_lon = d_lat / std::max(std::cos(lat), 1e-6);
    const Vec3 east = surface_point(lat, lon + d_lon) - surface_point(lat, lon - d_lon);
    const Vec3 north = surface_point(lat + d_lat, lon) - surface_point(lat - d_lat, lon);
    const Vec3 n = math::normalized(cross(east, north));
    return dot(n, fixed_m) >= 0.0 ? n : -n;
}

}  // namespace starsim::environment
