#include "starsim/contact/primitives.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace starsim::contact {

using math::Vec3;

namespace {

// Segment samples every half radius: the sampled clearance overestimates the true one by at most
// r * (1 - sqrt(1 - 1/16)) = 3 % of the radius.
constexpr double sample_spacing_per_radius = 0.5;

// Smallest distance of the sampled capsule axis.
template <class Distance>
double min_axis_distance(const Vec3& a, const Vec3& b, double radius_m, Distance distance) {
    const double length = norm(b - a);
    const int samples = std::max(2, static_cast<int>(std::ceil(length / (sample_spacing_per_radius * radius_m))) + 1);
    double smallest = std::numeric_limits<double>::infinity();
    for (int i = 0; i < samples; ++i) {
        smallest = std::min(smallest, distance(a + (b - a) * (static_cast<double>(i) / (samples - 1))));
    }
    return smallest;
}

}  // namespace

Penetration point_in_box(const OrientedBox& box, const Vec3& point) {
    const Vec3 local = transpose(box.axes) * (point - box.center);
    const double l[3] = {local.x, local.y, local.z};
    const double h[3] = {box.half_extents.x, box.half_extents.y, box.half_extents.z};
    double best = std::numeric_limits<double>::infinity();
    int axis = 0;
    for (int i = 0; i < 3; ++i) {
        const double depth = h[i] - std::abs(l[i]);
        if (depth < 0.0) return {};
        if (depth < best) {
            best = depth;
            axis = i;
        }
    }
    return {true, best, column(box.axes, axis) * (l[axis] >= 0.0 ? 1.0 : -1.0)};
}

Penetration point_in_capsule(const Vec3& a, const Vec3& b, double radius_m, const Vec3& point) {
    const Vec3 ab = b - a;
    const double len2 = dot(ab, ab);
    const double t = len2 > 0.0 ? std::clamp(dot(point - a, ab) / len2, 0.0, 1.0) : 0.0;
    const Vec3 d = point - (a + t * ab);
    const double dist = norm(d);
    if (dist >= radius_m) return {};
    Vec3 normal = dist > 0.0 ? d / dist : Vec3{};
    if (dist == 0.0) {  // on the axis: any perpendicular
        normal = normalized_or_zero(cross(ab, std::abs(ab.x) < 0.9 * std::sqrt(len2) ? Vec3{1, 0, 0} : Vec3{0, 1, 0}));
    }
    return {true, radius_m - dist, normal};
}

double distance_to_box(const OrientedBox& box, const Vec3& point) {
    const Vec3 local = transpose(box.axes) * (point - box.center);
    const Vec3 outside{std::max(std::abs(local.x) - box.half_extents.x, 0.0),
                       std::max(std::abs(local.y) - box.half_extents.y, 0.0),
                       std::max(std::abs(local.z) - box.half_extents.z, 0.0)};
    return norm(outside);
}

double distance_to_annulus(const Annulus& ring, const Vec3& point) {
    const double rho = std::hypot(point.x, point.y);
    const double dr = std::max({ring.inner_radius_m - rho, rho - ring.outer_radius_m, 0.0});
    const double dz = std::max({point.z - ring.height_m, -point.z, 0.0});
    return std::hypot(dr, dz);
}

bool capsule_overlaps_box(const Vec3& a, const Vec3& b, double radius_m, const OrientedBox& box) {
    return capsule_penetration_m(a, b, radius_m, box) > 0.0;
}

double capsule_penetration_m(const Vec3& a, const Vec3& b, double radius_m, const OrientedBox& box) {
    return radius_m - min_axis_distance(a, b, radius_m, [&](const Vec3& p) { return distance_to_box(box, p); });
}

bool capsule_overlaps_annulus(const Vec3& a, const Vec3& b, double radius_m, const Annulus& ring) {
    return min_axis_distance(a, b, radius_m, [&](const Vec3& p) { return distance_to_annulus(ring, p); }) < radius_m;
}

}  // namespace starsim::contact
