#include "starsim/environment/lunar_terrain.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Vec3;

double HeightGrid::bilinear(double line, double sample) const {
    const int l0 = std::clamp(static_cast<int>(std::floor(line)), 0, lines - 2);
    const int s0 = std::clamp(static_cast<int>(std::floor(sample)), 0, samples - 2);
    const double fl = line - l0, fs = sample - s0;
    return (1.0 - fl) * ((1.0 - fs) * at(l0, s0) + fs * at(l0, s0 + 1)) +
           fl * ((1.0 - fs) * at(l0 + 1, s0) + fs * at(l0 + 1, s0 + 1));
}

std::pair<double, double> PolarStereographicTile::position(double latitude_rad, double longitude_rad) const {
    const double rho = 2.0 * sphere_radius_m * std::tan(0.25 * constants::pi + 0.5 * latitude_rad);
    const double x = rho * std::sin(longitude_rad);
    const double y = -rho * std::cos(longitude_rad);
    return {pole_line + y / scale_m - first_line, pole_sample + x / scale_m - first_sample};
}

bool PolarStereographicTile::inside(double line, double sample) const {
    return line >= 0.0 && sample >= 0.0 && line <= grid.lines - 1.0 && sample <= grid.samples - 1.0;
}

double PolarStereographicTile::edge_distance_px(double line, double sample) const {
    return std::min({line, grid.lines - 1.0 - line, sample, grid.samples - 1.0 - sample});
}

double CylindricalGrid::height_m(double latitude_rad, double longitude_rad) const {
    const double lon_deg = std::fmod(std::fmod(longitude_rad / constants::deg_to_rad, 360.0) + 360.0, 360.0);
    const double line = std::clamp(zero_line - pixels_per_degree * latitude_rad / constants::deg_to_rad, 0.0,
                                   grid.lines - 1.0);
    const double sample = zero_sample + pixels_per_degree * lon_deg;
    const int l0 = std::min(static_cast<int>(std::floor(line)), grid.lines - 2);
    const double fl = line - l0;
    const double s_floor = std::floor(sample);
    const double fs = sample - s_floor;
    const int n = grid.samples;
    const int s0 = ((static_cast<int>(s_floor) % n) + n) % n;
    const int s1 = (s0 + 1) % n;
    return (1.0 - fl) * ((1.0 - fs) * grid.at(l0, s0) + fs * grid.at(l0, s1)) +
           fl * ((1.0 - fs) * grid.at(l0 + 1, s0) + fs * grid.at(l0 + 1, s1));
}

LunarTerrain::LunarTerrain(CylindricalGrid global, std::vector<PolarStereographicTile> tiles, double sphere_radius_m)
    : global_(std::move(global)), tiles_(std::move(tiles)), sphere_radius_m_(sphere_radius_m) {
    const auto grid_max = [](const HeightGrid& g) {
        return g.dn.empty() ? 0.0 : g.dn_scale_m * *std::max_element(g.dn.begin(), g.dn.end());
    };
    max_height_m_ = grid_max(global_.grid);
    for (const auto& tile : tiles_) max_height_m_ = std::max(max_height_m_, grid_max(tile.grid));
}

double LunarTerrain::height_m(double latitude_rad, double longitude_rad) const {
    for (const auto& tile : tiles_) {
        const auto [line, sample] = tile.position(latitude_rad, longitude_rad);
        if (!tile.inside(line, sample)) continue;
        const double tile_h = tile.grid.bilinear(line, sample);
        const double x = std::clamp(tile.edge_distance_px(line, sample) * tile.scale_m / blend_m, 0.0, 1.0);
        const double w = x * x * (3.0 - 2.0 * x);  // smoothstep: the tile's core exactly, the grid at its edge
        return w == 1.0 ? tile_h : w * tile_h + (1.0 - w) * global_.height_m(latitude_rad, longitude_rad);
    }
    return global_.height_m(latitude_rad, longitude_rad);
}

double LunarTerrain::resolution_m(double latitude_rad, double longitude_rad) const {
    for (const auto& tile : tiles_) {
        const auto [line, sample] = tile.position(latitude_rad, longitude_rad);
        if (tile.inside(line, sample)) return tile.scale_m;
    }
    return sphere_radius_m_ * constants::deg_to_rad / global_.pixels_per_degree;
}

bool LunarTerrain::in_tile(double latitude_rad, double longitude_rad) const {
    for (const auto& tile : tiles_) {
        const auto [line, sample] = tile.position(latitude_rad, longitude_rad);
        if (tile.inside(line, sample)) return true;
    }
    return false;
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
