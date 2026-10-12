#pragma once

#include <array>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"

namespace starsim::contact {

// Contact sample points of a catchable body in its body frame (x along the axis from the vehicle origin).
struct CatchBodyGeometry {
    std::vector<std::vector<math::Vec3>> pins;  // per pin: points along the pin axis
    std::vector<math::Vec3> hull;               // hull ring at the height of the arms' inner faces
    double pin_station_m = 0.0;                 // vehicle frame
    double max_pin_load_n = 0.0;
};

// Empty if `pins` lists none. The hull ring sits half an arm thickness below the pins.
CatchBodyGeometry make_catch_geometry(const config::CatchPinConfig& pins, double stage_base_station_m,
                                      double hull_radius_m, double arm_thickness_m);

// Body pose in the site frame, relative to the ground.
struct SitePose {
    math::Vec3 cg_m;
    math::Vec3 velocity_mps;
    math::Mat3 body_to_site = math::Mat3::identity();  // columns: body x, y, z
    math::Vec3 angular_velocity_radps;                   // site frame
    double cg_station_m = 0.0;                           // vehicle frame
};

// One arm as the contact sees it.
struct ArmContactState {
    OrientedBox box;
    math::Vec3 hinge_m;
    double yaw_rate_radps = 0.0;  // rotation of the arm about the site vertical
    double carriage_velocity_mps = 0.0;
    math::Vec3 outward;           // horizontal unit away from the catch axis
};

struct CatchContact {
    math::Vec3 force_n;                       // on the body, site frame
    math::Vec3 torque_nm;                     // about the CG, site frame
    std::vector<int> pin_points_in_contact;   // per pin
    double max_pin_load_n = 0.0;
    std::array<double, 2> hull_push_n{};      // per arm: push against closing
    double vertical_load_n = 0.0;             // downward load on the arms

    bool all_pins_in_contact() const;
    bool any_contact() const;
};

// Penalty contact of the sample points with the arm boxes: normal force k d - c v_n (never pulling) and
// regularised Coulomb friction along the slip.
CatchContact catch_contact(const CatchBodyGeometry& body, const SitePose& pose, const std::array<ArmContactState, 2>& arms);

}  // namespace starsim::contact
