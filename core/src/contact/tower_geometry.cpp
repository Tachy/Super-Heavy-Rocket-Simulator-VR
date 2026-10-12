#include "starsim/contact/tower_geometry.hpp"

#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::contact {

using math::Vec3;

namespace {
constexpr Vec3 up{0.0, 0.0, 1.0};
}

TowerGeometry::TowerGeometry(const config::TowerConfig& config, double latitude_rad, double longitude_rad,
                             double height_m)
    : config_(config) {
    const double sl = std::sin(latitude_rad), cl = std::cos(latitude_rad);
    const double so = std::sin(longitude_rad), co = std::cos(longitude_rad);
    site_to_itrf_ = from_columns({-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl});
    origin_itrf_m_ = frames::geodetic_to_fixed({latitude_rad, longitude_rad, height_m});
    toward_tower_ = {std::sin(config.azimuth_rad), std::cos(config.azimuth_rad), 0.0};
    across_ = cross(up, toward_tower_);
}

OrientedBox TowerGeometry::tower_box() const {
    return {toward_tower_ * config_.distance_m + up * (0.5 * config_.height_m), from_columns(toward_tower_, across_, up),
            {0.5 * config_.width_m, 0.5 * config_.width_m, 0.5 * config_.height_m}};
}

Annulus TowerGeometry::mount() const {
    return {config_.mount_inner_radius_m, config_.mount_outer_radius_m, config_.mount_height_m};
}

Vec3 TowerGeometry::hinge(int side) const {
    const double lateral = 0.5 * config_.arm_closed_gap_m + 0.5 * config_.arm_width_m;
    return toward_tower_ * (config_.distance_m - 0.5 * config_.width_m) + across_ * (side * lateral) +
           up * config_.catch_height_m;
}

Vec3 TowerGeometry::arm_direction(int side, double opening_rad) const {
    return -toward_tower_ * std::cos(opening_rad) + across_ * (side * std::sin(opening_rad));
}

OrientedBox TowerGeometry::arm_box(int side, double opening_rad, double carriage_offset_m) const {
    const Vec3 u = arm_direction(side, opening_rad);
    const Vec3 center = hinge(side) + u * (0.5 * config_.arm_length_m) +
                        up * (carriage_offset_m - 0.5 * config_.arm_thickness_m);
    return {center, from_columns(u, cross(up, u), up),
            {0.5 * config_.arm_length_m, 0.5 * config_.arm_width_m, 0.5 * config_.arm_thickness_m}};
}

}  // namespace starsim::contact
