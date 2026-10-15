#include "starsim/contact/landing_surface.hpp"

#include <cmath>

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

TerrainField::TerrainField(const environment::ReferenceBody& body, const Vec3& origin_fixed_m, const math::Mat3& site_to_fixed)
    : body_(body), origin_fixed_m_(origin_fixed_m), site_to_fixed_(site_to_fixed) {}

SurfaceSupport TerrainField::below(double east_m, double north_m) const {
    const Vec3 q = origin_fixed_m_ + site_to_fixed_ * Vec3{east_m, north_m, 0.0};
    const Vec3 surface = math::normalized(q) * (body_.shape().semi_major_axis_m + body_.surface_height_m(q));
    const Vec3 up = site_to_fixed_ * Vec3{0.0, 0.0, 1.0};
    return {dot(surface - origin_fixed_m_, up), false, transpose(site_to_fixed_) * body_.surface_normal_fixed(surface)};
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

}  // namespace starsim::contact
