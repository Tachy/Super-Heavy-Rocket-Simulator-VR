#include "starsim/gnc/descent_keep_out.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;

namespace {
constexpr double hull_sample_m = 1.0;  // along the hull axis, as the tower physics samples its capsule
}

HullBoxClearance hull_box_clearance(const Vec3& cg_m, const Vec3& axis, const KeepOutBox& box, const HullExtent& hull, double margin_m) {
    HullBoxClearance best;
    best.clearance_m = 1.0e12;
    best.overlap_m = -1.0e12;
    const double wide_y = -box.long_z, wide_z = box.long_y;  // horizontal, perpendicular to the long axis
    const double length = hull.below_cg_m + hull.above_cg_m;
    const int samples = static_cast<int>(std::ceil(length / hull_sample_m));
    for (int i = 0; i <= samples; ++i) {
        const double lever = -hull.below_cg_m + length * i / samples;
        const Vec3 p = cg_m + axis * lever;
        const double dy = p.y - box.center_m.y, dz = p.z - box.center_m.z;
        const double along = dy * box.long_y + dz * box.long_z;
        const double across = dy * wide_y + dz * wide_z;
        const double out_along = std::abs(along) - box.half_long_m;
        const double out_across = std::abs(across) - box.half_wide_m;
        const double out_height = std::max(std::abs(p.x - box.center_m.x) - box.half_height_m, 0.0);
        // Distance to the footprint: outside its nearest point, inside (negative) the nearest edge.
        double distance, near_along = along, near_across = across, ny, nz;
        if (out_along <= 0.0 && out_across <= 0.0) {
            const bool by_width = out_across >= out_along;
            distance = std::max(out_along, out_across);
            if (by_width) near_across = std::copysign(box.half_wide_m, across);
            else near_along = std::copysign(box.half_long_m, along);
            const double sign = (by_width ? across : along) < 0.0 ? -1.0 : 1.0;
            ny = sign * (by_width ? wide_y : box.long_y);
            nz = sign * (by_width ? wide_z : box.long_z);
        } else {
            near_along = std::clamp(along, -box.half_long_m, box.half_long_m);
            near_across = std::clamp(across, -box.half_wide_m, box.half_wide_m);
            const double ra = along - near_along, rc = across - near_across;
            distance = std::hypot(ra, rc);
            ny = (ra * box.long_y + rc * wide_y) / distance;
            nz = (ra * box.long_z + rc * wide_z) / distance;
        }
        const double footprint_only = std::max(distance, 0.0);
        best.overlap_m = std::max(best.overlap_m, hull.radius_m - std::hypot(footprint_only, out_height));
        if (out_height > hull.radius_m + margin_m) continue;
        const double clearance = distance - hull.radius_m;
        if (best.in_height_band && clearance >= best.clearance_m) continue;
        best.in_height_band = true;
        best.clearance_m = clearance;
        best.lever_m = lever;
        best.normal_y = ny;
        best.normal_z = nz;
        best.plane_m = ny * (box.center_m.y + near_along * box.long_y + near_across * wide_y) +
                       nz * (box.center_m.z + near_along * box.long_z + near_across * wide_z);
    }
    return best;
}

double keep_out_overlap_m(const DescentReference& reference, const DescentTarget& target, int start_phase, int n) {
    double worst = -1.0e12;
    if (target.keep_out_box_count == 0) return 0.0;
    const int nodes = static_cast<int>(reference.x.size());
    for (int k = start_phase * (n - 1); k + 1 < nodes; ++k) {
        const auto& a = reference.x[static_cast<size_t>(k)];
        const auto& b = reference.x[static_cast<size_t>(k + 1)];
        const Vec3 axis_a = state_attitude(a).rotate({1.0, 0.0, 0.0});
        const Vec3 axis_b = state_attitude(b).rotate({1.0, 0.0, 0.0});
        for (int j = 1; j <= keep_out_samples_per_interval; ++j) {
            const double f = static_cast<double>(j) / keep_out_samples_per_interval;
            const Vec3 cg = state_position(a) * (1.0 - f) + state_position(b) * f;
            const Vec3 axis = math::normalized(axis_a * (1.0 - f) + axis_b * f);
            for (int i = 0; i < target.keep_out_box_count; ++i) {
                worst = std::max(worst, hull_box_clearance(cg, axis, target.keep_out_boxes[static_cast<size_t>(i)], target.hull, 0.0).overlap_m);
            }
        }
    }
    return worst;
}

}  // namespace starsim::gnc
