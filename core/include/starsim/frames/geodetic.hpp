#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::frames {

// Geodetic coordinates on a reference ellipsoid. Longitude positive east.
struct GeodeticPosition {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
};

struct Ellipsoid {
    double semi_major_axis_m;
    double flattening;

    double semi_minor_axis_m() const { return semi_major_axis_m * (1.0 - flattening); }
    double first_eccentricity_squared() const { return flattening * (2.0 - flattening); }
};

inline constexpr Ellipsoid wgs84{constants::wgs84_semi_major_axis_m, constants::wgs84_flattening};

// Geodetic -> body-fixed Cartesian (for the Earth: ITRF).
math::Vec3 geodetic_to_fixed(const GeodeticPosition& geodetic, const Ellipsoid& ellipsoid = wgs84);

// Body-fixed Cartesian -> geodetic. Closed-form (Heikkinen 1982), exact to rounding for
// points farther than ~43 km from the Earth's centre.
GeodeticPosition fixed_to_geodetic(const math::Vec3& fixed_m, const Ellipsoid& ellipsoid = wgs84);

}  // namespace starsim::frames
