#include "starsim/environment/lunar_tile.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "starsim/math/constants.hpp"
#include "starsim/math/fast_trig.hpp"

namespace starsim::environment {

using math::Vec3;

double HeightGrid::bilinear(double line, double sample) const {
    // Truncation is the floor for positions inside the grid (>= 0); below 0 the clamp gives cell 0 either way.
    const int l0 = std::clamp(static_cast<int>(line), 0, lines - 2);
    const int s0 = std::clamp(static_cast<int>(sample), 0, samples - 2);
    const double fl = line - l0, fs = sample - s0;
    const std::int16_t* row0 = &dn[static_cast<size_t>(l0) * static_cast<size_t>(samples) + static_cast<size_t>(s0)];
    const std::int16_t* row1 = row0 + samples;
    return dn_scale_m * ((1.0 - fl) * ((1.0 - fs) * row0[0] + fs * row0[1]) +
                         fl * ((1.0 - fs) * row1[0] + fs * row1[1]));
}

std::pair<double, double> PolarStereographicTile::position(double latitude_rad, double longitude_rad) const {
    const double sign = north ? 1.0 : -1.0;
    const double rho = 2.0 * sphere_radius_m * std::tan(0.25 * constants::pi - sign * 0.5 * latitude_rad);
    const double x = rho * std::sin(longitude_rad);
    const double y = sign * rho * std::cos(longitude_rad);
    return {pole_line + y / scale_m - first_line, pole_sample + x / scale_m - first_sample};
}

std::pair<double, double> PolarStereographicTile::position_of_direction(const Vec3& unit) const {
    const double sign = north ? 1.0 : -1.0;
    const double denominator = 1.0 + sign * unit.z;  // south 1 - z, north 1 + z
    if (denominator <= 0.0) return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    const double k = 2.0 * sphere_radius_m / denominator;
    return {pole_line + sign * k * unit.x / scale_m - first_line, pole_sample + k * unit.y / scale_m - first_sample};
}

bool PolarStereographicTile::inside(double line, double sample) const {
    return line >= 0.0 && sample >= 0.0 && line <= grid.lines - 1.0 && sample <= grid.samples - 1.0;
}

double PolarStereographicTile::edge_distance_px(double line, double sample) const {
    return std::min(std::min(line, grid.lines - 1.0 - line), std::min(sample, grid.samples - 1.0 - sample));
}

double CylindricalGrid::height_at_grid(double line, double sample) const {
    line = std::clamp(line, 0.0, grid.lines - 1.0);  // >= 0: truncation is the floor
    const int l0 = std::min(static_cast<int>(line), grid.lines - 2);
    const double fl = line - l0;
    const double s_floor = std::floor(sample);
    const double fs = sample - s_floor;
    const int n = grid.samples;
    int s0 = static_cast<int>(s_floor);
    while (s0 >= n) s0 -= n;  // the longitude wraps
    while (s0 < 0) s0 += n;
    const int s1 = s0 + 1 == n ? 0 : s0 + 1;
    return (1.0 - fl) * ((1.0 - fs) * grid.at(l0, s0) + fs * grid.at(l0, s1)) +
           fl * ((1.0 - fs) * grid.at(l0 + 1, s0) + fs * grid.at(l0 + 1, s1));
}

double CylindricalGrid::height_m(double latitude_rad, double longitude_rad) const {
    const double lon_deg = std::fmod(std::fmod(longitude_rad / constants::deg_to_rad, 360.0) + 360.0, 360.0);
    return height_at_grid(zero_line - pixels_per_degree * latitude_rad / constants::deg_to_rad,
                          zero_sample + pixels_per_degree * lon_deg);
}

double CylindricalGrid::height_at_direction(const Vec3& unit) const {
    const double lon_deg = math::fast_atan2(unit.y, unit.x) * constants::rad_to_deg;
    return height_at_grid(zero_line - pixels_per_degree * math::fast_asin(unit.z) * constants::rad_to_deg,
                          zero_sample + pixels_per_degree * (lon_deg < 0.0 ? lon_deg + 360.0 : lon_deg));
}

namespace {

double grid_max(const HeightGrid& g) {
    return g.dn.empty() ? 0.0 : g.dn_scale_m * *std::max_element(g.dn.begin(), g.dn.end());
}

class PolarTile final : public LunarTile {
public:
    PolarTile(PolarStereographicTile tile, bool north) : t_(std::move(tile)) {
        t_.north = north;
        max_height_m_ = grid_max(t_.grid);
    }

    bool contains(const Vec3& unit) const override {
        const auto [line, sample] = t_.position_of_direction(unit);
        return t_.inside(line, sample);
    }
    double height_m(const Vec3& unit) const override {
        const auto [line, sample] = t_.position_of_direction(unit);
        return t_.grid.bilinear(line, sample);
    }
    double edge_distance_m(const Vec3& unit) const override {
        const auto [line, sample] = t_.position_of_direction(unit);
        return t_.edge_distance_px(line, sample) * t_.scale_m;
    }
    double scale_m() const override { return t_.scale_m; }
    double outside_distance_m(const Vec3& unit) const override {
        // The projection is conformal with the scale sec^2(colatitude / 2) = 2 / (1 + cos colatitude) at the point, so
        // a true distance is the projected one times (1 + cos colatitude) / 2 = the position's denominator / 2. Past
        // the equator (the opposite hemisphere) a projected distance says nothing.
        const double sign = t_.north ? 1.0 : -1.0;
        if (sign * unit.z < 0.0) return std::numeric_limits<double>::infinity();
        const auto [line, sample] = t_.position_of_direction(unit);
        const double outside_lines = std::max({0.0, -line, line - (t_.grid.lines - 1.0)});
        const double outside_samples = std::max({0.0, -sample, sample - (t_.grid.samples - 1.0)});
        return std::hypot(outside_lines, outside_samples) * t_.scale_m * 0.5 * (1.0 + sign * unit.z);
    }
    double max_height_m() const override { return max_height_m_; }

private:
    PolarStereographicTile t_;
    double max_height_m_ = 0.0;
};

class CylindricalTile final : public LunarTile {
public:
    CylindricalTile(CylindricalGrid grid, double first_latitude_rad, double first_longitude_rad, double radius_m)
        : g_(std::move(grid)),
          first_latitude_rad_(first_latitude_rad),
          first_longitude_rad_(first_longitude_rad),
          // metres per pixel along a meridian; along a parallel it is this times cos(latitude)
          scale_ns_m_(radius_m * constants::deg_to_rad / g_.pixels_per_degree),
          max_height_m_(grid_max(g_.grid)) {}

    bool contains(const Vec3& unit) const override {
        const auto p = locate(unit);
        return p.line >= 0.0 && p.sample >= 0.0 && p.line <= g_.grid.lines - 1.0 && p.sample <= g_.grid.samples - 1.0;
    }
    double height_m(const Vec3& unit) const override {
        const auto p = locate(unit);
        return g_.grid.bilinear(p.line, p.sample);
    }
    double edge_distance_m(const Vec3& unit) const override {
        const auto p = locate(unit);
        return std::min(std::min(p.line, g_.grid.lines - 1.0 - p.line) * scale_ns_m_,
                        std::min(p.sample, g_.grid.samples - 1.0 - p.sample) * scale_ns_m_ * p.cos_latitude);
    }
    double scale_m() const override { return scale_ns_m_; }
    double outside_distance_m(const Vec3& unit) const override {
        const auto p = locate(unit);
        const double outside_lines = std::max({0.0, -p.line, p.line - (g_.grid.lines - 1.0)});
        const double outside_samples = std::max({0.0, -p.sample, p.sample - (g_.grid.samples - 1.0)});
        return std::hypot(outside_lines * scale_ns_m_, outside_samples * scale_ns_m_ * p.cos_latitude);
    }
    double max_height_m() const override { return max_height_m_; }

private:
    struct Position {
        double line, sample, cos_latitude;
    };
    Position locate(const Vec3& unit) const {
        const double lat = math::fast_asin(unit.z);
        double dlon = math::fast_atan2(unit.y, unit.x) - first_longitude_rad_;
        dlon -= constants::two_pi * std::floor(dlon / constants::two_pi + 0.5);  // the signed offset in [-pi, pi)
        const double ppd_per_rad = g_.pixels_per_degree * constants::rad_to_deg;
        return {(first_latitude_rad_ - lat) * ppd_per_rad, dlon * ppd_per_rad,
                std::sqrt(std::max(0.0, (1.0 - unit.z) * (1.0 + unit.z)))};
    }

    CylindricalGrid g_;
    double first_latitude_rad_, first_longitude_rad_, scale_ns_m_, max_height_m_;
};

}  // namespace

std::shared_ptr<const LunarTile> make_polar_tile(PolarStereographicTile tile, bool north) {
    return std::make_shared<const PolarTile>(std::move(tile), north);
}

std::shared_ptr<const LunarTile> make_cylindrical_tile(CylindricalGrid grid, double first_latitude_rad,
                                                       double first_longitude_rad, double sphere_radius_m) {
    return std::make_shared<const CylindricalTile>(std::move(grid), first_latitude_rad, first_longitude_rad,
                                                   sphere_radius_m);
}

}  // namespace starsim::environment
