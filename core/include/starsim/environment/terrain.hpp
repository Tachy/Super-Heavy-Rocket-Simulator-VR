#pragma once

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
};

}  // namespace starsim::environment
