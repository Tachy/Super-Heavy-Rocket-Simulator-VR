#pragma once

#include <optional>

#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

// Coastline of the landing site: ground beyond the shoreline (perpendicular to direction_rad) is water.
struct SeaSide {
    double direction_rad = 0.0;          // pad centre -> sea, clockwise from north
    double shoreline_distance_m = 0.0;
};

// Catch on the tower arms instead of a touchdown on the pad.
struct CatchTarget {
    math::Vec3 pin_point_itrf_m;          // pins' midpoint on a centred catch: on the catch axis at the arm tops
    math::Vec3 across_itrf;               // horizontal unit; the pins must point along +-this direction
    double pin_station_m = 0.0;           // vehicle frame
    double pin_azimuth_rad = 0.0;         // first pin, from body +y towards +z
    math::Vec3 tower_center_itrf_m;       // tower axis on the ground (abort keep-out)
    double tower_keep_out_radius_m = 0.0; // around the tower axis, incl. the hull radius and a margin
};

struct LandingSite {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;  // ground (pad) height above WGS84
    std::optional<CatchTarget> catch_target;
    std::optional<SeaSide> sea;
};

// Body +y direction (same frame as the inputs) that turns the first pin along +-across, upright body (x = up);
// of the two solutions the one nearer `current_body_y` (the pins are symmetric).
math::Vec3 catch_roll_reference(const math::Vec3& across, const math::Vec3& up, double pin_azimuth_rad,
                                const math::Vec3& current_body_y);

}  // namespace starsim::gnc
