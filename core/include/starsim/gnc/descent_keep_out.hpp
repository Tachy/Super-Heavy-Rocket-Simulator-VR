#pragma once

#include "starsim/gnc/descent_problem.hpp"

namespace starsim::gnc {

// The hull against one keep-out box at one instant, with the capsule the tower physics collides (axis segment swept by
// the hull radius). The horizontal measure is what a plan row can move; the height band only decides whether it counts.
struct HullBoxClearance {
    bool in_height_band = false;  // some part of the capsule (+ margin) lies within the box's height
    double clearance_m = 0.0;     // horizontal, hull surface to the box footprint, in the band (negative: over it)
    double overlap_m = 0.0;       // how deep the box reaches into the capsule, in 3D (<= 0: clear)
    double lever_m = 0.0;         // the axis point that decides, from the CG along the axis
    double normal_y = 1.0;        // horizontal unit away from the footprint, plan frame
    double normal_z = 0.0;
    double plane_m = 0.0;         // n . (the footprint's nearest point): keep n . (axis point) >= plane + radius + margin
};

HullBoxClearance hull_box_clearance(const math::Vec3& cg_m, const math::Vec3& axis, const KeepOutBox& box, const HullExtent& hull,
                                    double margin_m);

// Deepest overlap of the planned hull with the target's boxes, sampled between the nodes after the measured start state
// (<= 0: clear; 0 without boxes). The hull is a rigid line along the plan's thrust axis.
double keep_out_overlap_m(const DescentReference& reference, const DescentTarget& target, int start_phase, int nodes_per_phase);

}  // namespace starsim::gnc
