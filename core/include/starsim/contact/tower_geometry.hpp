#pragma once

#include "starsim/config/launch_site_config.hpp"
#include "starsim/contact/primitives.hpp"

namespace starsim::contact {

// Catch tower in the site frame: origin at the pad centre on the ground, x east, y north, z up (geodetic
// vertical); the catch axis is the vertical through the origin. Closed arms are parallel, pointing from the
// tower over the pad, their inner edges `arm_closed_gap_m / 2` from the catch axis; an arm swings outward
// about a vertical hinge on the tower face.
class TowerGeometry {
public:
    TowerGeometry(const config::TowerConfig& config, double latitude_rad, double longitude_rad, double height_m);

    const config::TowerConfig& config() const { return config_; }
    math::Vec3 to_itrf(const math::Vec3& site_m) const { return origin_itrf_m_ + site_to_itrf_ * site_m; }
    math::Vec3 to_site(const math::Vec3& itrf_m) const { return transpose(site_to_itrf_) * (itrf_m - origin_itrf_m_); }
    const math::Mat3& site_to_itrf() const { return site_to_itrf_; }  // columns: east, north, up
    const math::Vec3& origin_itrf_m() const { return origin_itrf_m_; }
    const math::Vec3& tower_direction() const { return toward_tower_; }  // pad -> tower, horizontal unit
    const math::Vec3& across() const { return across_; }                 // up x tower_direction

    OrientedBox tower_box() const;
    Annulus mount() const;
    // Hinge of arm `side` (+1: on the +across side, -1: opposite) at the arm top height, carriage at rest.
    math::Vec3 hinge(int side) const;
    double arm_top_height_m(double carriage_offset_m) const { return config_.catch_height_m + carriage_offset_m; }
    // Horizontal unit from the hinge towards the tip; opening 0 = closed.
    math::Vec3 arm_direction(int side, double opening_rad) const;
    OrientedBox arm_box(int side, double opening_rad, double carriage_offset_m) const;

private:
    config::TowerConfig config_;
    math::Mat3 site_to_itrf_;
    math::Vec3 origin_itrf_m_;
    math::Vec3 toward_tower_;
    math::Vec3 across_;
};

}  // namespace starsim::contact
