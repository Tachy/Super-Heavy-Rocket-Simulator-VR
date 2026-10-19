#pragma once

#include <string_view>

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::vehicle {

// Where a docking port of a flying body is and how it moves (GCRF). The port frame: x = the outward docking axis,
// z = the clocking reference "up", y = z x x (right-handed). Two ports mate with the axes opposed and the ups equal.
struct PortKinematics {
    math::Vec3 position_gcrf_m;
    math::Vec3 velocity_gcrf_mps;            // of the port point: CG velocity + omega x lever
    math::Mat3 axes_gcrf;                    // columns x, y, z of the port frame
    math::Vec3 angular_velocity_gcrf_radps;  // of the body
    bool covered = false;                    // behind a closed nose cone
};

// The port `port_id` on any stage of the body's vehicle (config [[stage.docking_port]]); the lever runs from the CG,
// so it follows the CG as propellant is used. Throws std::invalid_argument (German) for an unknown id.
PortKinematics port_kinematics(const FlightBody& body, std::string_view port_id);

}  // namespace starsim::vehicle
