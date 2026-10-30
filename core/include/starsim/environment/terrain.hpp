#pragma once

#include <algorithm>
#include <cmath>

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// A body's surface over its shape (Moon: the sphere, Earth: the WGS84 ellipsoid) in body-fixed coordinates.
class Terrain {
public:
    virtual ~Terrain() = default;
    // Surface height above the shape below a body-fixed point (along the shape's normal through it).
    virtual double height_at_fixed_m(const math::Vec3& fixed_m) const = 0;
    // Outward unit normal of the surface below the point.
    virtual math::Vec3 normal_fixed(const math::Vec3& fixed_m) const = 0;
    // Whether the surface below the point is water (the height is then the water surface).
    virtual bool water_at_fixed(const math::Vec3& /*fixed_m*/) const { return false; }

    // The same by latitude / longitude (Earth: WGS84 geodetic): the surface height above the shape.
    virtual double height_m(double latitude_rad, double longitude_rad) const = 0;
    virtual bool water(double /*latitude_rad*/, double /*longitude_rad*/) const { return false; }
    // The same height by the unit direction of the point from the body centre (the terrain horizon march's sampler;
    // a terrain may skip the latitude / longitude round trip). Default: latitude = asin(z), longitude = atan2(y, x).
    virtual double height_at_direction_m(const math::Vec3& unit) const;
    // Spacing in metres of the finest data grid covering the point (the level of detail the terrain really holds).
    virtual double resolution_m(double latitude_rad, double longitude_rad) const = 0;
    // Upper bound of height_m over the whole body (the relief bound of the terrain horizon march).
    virtual double max_height_m() const = 0;
};

inline double Terrain::height_at_direction_m(const math::Vec3& unit) const {
    return height_m(std::asin(std::clamp(unit.z, -1.0, 1.0)), std::atan2(unit.y, unit.x));
}

}  // namespace starsim::environment
