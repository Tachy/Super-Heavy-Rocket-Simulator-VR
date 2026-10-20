#pragma once

#include <optional>
#include <string>
#include <string_view>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::vehicle {

class Vehicle;

// A rigid map from one stage frame into another (x station from the stage bottom, y/z body axes).
struct StageTransform {
    math::Mat3 rotation;
    math::Vec3 translation_m;
    math::Vec3 apply(const math::Vec3& point_m) const { return rotation * point_m + translation_m; }
};

// Two mated ports (ISS i5 convention: the port axes opposed, the ups equal): maps points of the chaser's stage frame
// into the target's. Refuelling r1: two dorsal ports (axis +z, up +x) give parallel ships rolled 180 deg.
StageTransform mated_stage_transform(const config::DockingPortConfig& chaser, const config::DockingPortConfig& target);

// Why two ports cannot mate (German, shown to the user), or nullopt: the mechanisms must be equal, and a probe/drogue
// pair needs the probes on the chaser (soft_capture "active") and the drogues on the target ("passive").
std::optional<std::string> port_mismatch(const config::DockingPortConfig& chaser,
                                         const config::DockingPortConfig& target);

// The port `port_id` on any stage of the vehicle. Throws std::invalid_argument (German) for an unknown id.
const config::DockingPortConfig& port_config(const Vehicle& vehicle, std::string_view port_id);

}  // namespace starsim::vehicle
