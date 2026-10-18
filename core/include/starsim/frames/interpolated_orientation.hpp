#pragma once

#include <cstdint>
#include <unordered_map>

#include "starsim/frames/body_orientation.hpp"

namespace starsim::frames {

// A body orientation that samples another one on a fixed grid of epochs: the rotation at an epoch is the nearest
// node's, turned on by that node's angular velocity (body frame) for the time from it. l6 Task 1: the Earth's
// orientation (sxform per J2 evaluation) was a quarter of a lunar return's plan time. The error is the change of the
// angular velocity over half a spacing (precession / nutation rates turning with the body: ~1e-11 rad at 10 min, sub-mm
// at the surface; test_interpolated_ephemeris checks 1 mm over a month).
// Nodes are filled on demand and kept: not thread-safe (like the CSPICE source).
class InterpolatedOrientation final : public BodyOrientation {
public:
    // [E] node spacing: 10 min (see the test's measured error).
    static constexpr double default_node_spacing_s = 600.0;

    // `node_spacing_s` <= 0: every call passes through to `source` (the fidelity flag's off state).
    explicit InterpolatedOrientation(const BodyOrientation& source, double node_spacing_s = default_node_spacing_s);

    FixedToInertial fixed_to_gcrf(time::Epoch epoch) const override;

private:
    struct Node {
        math::Mat3 rotation;
        math::Vec3 rate_body_radps;  // angular velocity in the body frame: R^T dR/dt = [w]x
    };
    const Node& node(std::int64_t index) const;

    const BodyOrientation& source_;
    double spacing_s_;
    mutable std::unordered_map<std::int64_t, Node> nodes_;
    mutable std::int64_t last_index_ = 0;
    mutable const Node* last_ = nullptr;
};

}  // namespace starsim::frames
