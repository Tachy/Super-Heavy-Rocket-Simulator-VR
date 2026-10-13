#include "starsim/contact/landing_surface.hpp"

#include <cmath>

namespace starsim::contact {

LandingSurface::Support LandingSurface::below(double east_m, double north_m) const {
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
