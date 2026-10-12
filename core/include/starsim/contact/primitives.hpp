#pragma once

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::contact {

inline math::Mat3 from_columns(const math::Vec3& a, const math::Vec3& b, const math::Vec3& c) {
    math::Mat3 m = math::Mat3::identity();
    const math::Vec3 cols[3] = {a, b, c};
    for (int j = 0; j < 3; ++j) {
        m(0, j) = cols[j].x;
        m(1, j) = cols[j].y;
        m(2, j) = cols[j].z;
    }
    return m;
}

inline math::Vec3 column(const math::Mat3& m, int c) { return {m(0, c), m(1, c), m(2, c)}; }

// Box with centre, orthonormal axes (columns: local x, y, z in the frame) and half extents.
struct OrientedBox {
    math::Vec3 center;
    math::Mat3 axes = math::Mat3::identity();
    math::Vec3 half_extents;
};

struct Penetration {
    bool inside = false;
    double depth_m = 0.0;  // distance to the nearest face
    math::Vec3 normal;     // outward normal of that face
};

Penetration point_in_box(const OrientedBox& box, const math::Vec3& point);
double distance_to_box(const OrientedBox& box, const math::Vec3& point);  // 0 inside

// Solid ring around the vertical axis through the origin: inner..outer radius, from z = 0 to height.
struct Annulus {
    double inner_radius_m = 0.0;
    double outer_radius_m = 0.0;
    double height_m = 0.0;
};

double distance_to_annulus(const Annulus& ring, const math::Vec3& point);  // 0 inside

// Capsule = segment a-b swept by a sphere of `radius_m`; tested by sampling the segment (see the .cpp).
bool capsule_overlaps_box(const math::Vec3& a, const math::Vec3& b, double radius_m, const OrientedBox& box);
bool capsule_overlaps_annulus(const math::Vec3& a, const math::Vec3& b, double radius_m, const Annulus& ring);

}  // namespace starsim::contact
