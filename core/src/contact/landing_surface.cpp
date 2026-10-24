#include "starsim/contact/landing_surface.hpp"

#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::contact {

using math::Vec3;

PlaneField::PlaneField(double slope_rad, double uphill_azimuth_rad)
    : tan_slope_(std::tan(slope_rad)),
      uphill_{std::sin(uphill_azimuth_rad), std::cos(uphill_azimuth_rad), 0.0},
      normal_{-std::sin(slope_rad) * std::sin(uphill_azimuth_rad), -std::sin(slope_rad) * std::cos(uphill_azimuth_rad),
              std::cos(slope_rad)} {}

SurfaceSupport PlaneField::below(double east_m, double north_m) const {
    return {tan_slope_ * (east_m * uphill_.x + north_m * uphill_.y), false, normal_};
}

TerrainField::TerrainField(const environment::ReferenceBody& body, const Vec3& origin_fixed_m, const math::Mat3& site_to_fixed,
                           std::shared_ptr<const environment::Terrain> terrain)
    : terrain_(std::move(terrain)), body_(body), origin_fixed_m_(origin_fixed_m), site_to_fixed_(site_to_fixed) {
    if (terrain_) body_.set_terrain(terrain_.get());
}

SurfaceSupport TerrainField::below(double east_m, double north_m) const {
    const Vec3 q = origin_fixed_m_ + site_to_fixed_ * Vec3{east_m, north_m, 0.0};
    const auto& shape = body_.shape();
    Vec3 surface;
    if (shape.flattening == 0.0) {  // a sphere: on the radial (kept bit-identical for the Moon)
        surface = math::normalized(q) * (shape.semi_major_axis_m + body_.surface_height_m(q));
    } else {
        const auto g = frames::fixed_to_geodetic(q, shape);
        surface = frames::geodetic_to_fixed({g.latitude_rad, g.longitude_rad, body_.surface_height_m(q)}, shape);
    }
    const Vec3 up = site_to_fixed_ * Vec3{0.0, 0.0, 1.0};
    return {dot(surface - origin_fixed_m_, up), body_.surface_water(q),
            transpose(site_to_fixed_) * body_.surface_normal_fixed(surface)};
}

LandingSurface::Support LandingSurface::below(double east_m, double north_m) const {
    if (field) return field->below(east_m, north_m);
    if (deck) {
        // Long axis: unit (sin h, cos h) in (east, north).
        const double along = east_m * std::sin(deck->heading_rad) + north_m * std::cos(deck->heading_rad);
        const double across = east_m * std::cos(deck->heading_rad) - north_m * std::sin(deck->heading_rad);
        if (std::abs(along) <= deck->half_length_m && std::abs(across) <= deck->half_width_m) return {0.0, false};
        return {-deck->freeboard_m, true};
    }
    if (sea && east_m * std::sin(sea->direction_rad) + north_m * std::cos(sea->direction_rad) >= sea->shoreline_distance_m) {
        return {0.0, true};
    }
    return {0.0, false};
}

DeckSeaField::DeckSeaField(const LandingSurface::Deck& deck, const math::Mat3& carrier_to_site,
                           const Vec3& carrier_origin_site_m)
    : deck_(deck), carrier_to_site_(carrier_to_site), carrier_origin_site_m_(carrier_origin_site_m) {}

SurfaceSupport DeckSeaField::below(double east_m, double north_m) const {
    LandingSurface level;
    level.deck = deck_;
    const auto on_deck = level.below(east_m, north_m);
    if (!on_deck.water) return on_deck;
    // site z of the carrier point (x, y, z): o_z + M_20 x + M_21 y + M_22 z; the sea where it is -freeboard
    const auto& m = carrier_to_site_.m;
    const double height = (-deck_.freeboard_m - carrier_origin_site_m_.z - m[2][0] * east_m - m[2][1] * north_m) / m[2][2];
    return {height, true, Vec3{m[2][0], m[2][1], m[2][2]}};
}

}  // namespace starsim::contact
