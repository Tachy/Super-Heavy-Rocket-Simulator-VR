#pragma once

#include <string_view>

#include "starsim/math/vec3.hpp"
#include "starsim/vehicle/docking_port.hpp"

namespace starsim::gnc {

// Fidelity flag relative_nav. Ideal: the true states of both bodies (the real Dragon navigates on GPS, LIDAR and
// thermal cameras; sensor models later).
enum class RelativeNavMode { Ideal };
RelativeNavMode parse_relative_nav(std::string_view text);  // "ideal"; std::invalid_argument otherwise

// The chaser's docking port against the target's, in the target port frame (x out of the target port, z its up). The
// frame turns with the target body: a chaser at rest relative to the target reads zero velocity. Docked: position 0,
// misalignment 0 (axes opposed), clocking 0 (ups equal).
struct RelativePortState {
    math::Vec3 position_m;          // chaser port relative to the target port
    math::Vec3 velocity_mps;        // its rate of change in the rotating target port frame
    math::Vec3 angular_rate_radps;  // chaser body rate relative to the target's
    double range_m = 0.0;
    double closing_speed_mps = 0.0;  // -d(range)/dt: > 0 approaching
    double misalignment_rad = 0.0;   // between the chaser's docking axis and the reversed target axis
    double clocking_rad = 0.0;       // chaser up against target up about the target axis, (-pi, pi]
    bool chaser_port_covered = false;
};

RelativePortState relative_port_state(const vehicle::PortKinematics& chaser, const vehicle::PortKinematics& target,
                                      RelativeNavMode mode = RelativeNavMode::Ideal);

}  // namespace starsim::gnc
