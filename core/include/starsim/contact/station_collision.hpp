#pragma once

#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"

namespace starsim::contact {

// Hull contact of a visiting vehicle with a station (ISS i5): sample points on the vehicle hull against the station's
// collision primitives ([[stage.collision]]: capsules and boxes), as a penalty contact. A contact is a "Kollision"
// for the session; the docking system region below the port is left to the docking interface.

struct StationCollisionParameters {
    double ring_spacing_m = 0.5;       // [E] hull sample rings along the vehicle
    int points_per_ring = 16;          // [E]
    double port_clearance_m = 1.0;     // [E] the top metre below a docking port (docking system, nose cone hinge) is
                                       // not sampled - the docking interface owns that contact
    double stiffness_n_per_m = 1.0e6;  // [E] hull penalty: omega ~9 rad/s on 12 t (stable at 200 Hz)
    double damping_ns_per_m = 1.1e5;   // [E] damping ratio ~0.5
    double check_range_m = 120.0;      // [E] CG distance below which the hull is checked (ISS half span ~55 m + margin)
};

// Vehicle body frame (x from the bottom of stage 0): rings every ring_spacing_m over each stage at half its diameter,
// tapering linearly to half its hull_top_diameter_m; the top stage ends port_clearance_m below its docking port.
std::vector<math::Vec3> hull_sample_points(const config::VehicleConfig& vehicle, const StationCollisionParameters& p);

struct StationCapsule {
    math::Vec3 a_m, b_m;
    double radius_m = 0.0;
};
struct StationShape {  // station body frame
    std::vector<OrientedBox> boxes;
    std::vector<StationCapsule> capsules;
};
StationShape station_shape(const config::VehicleConfig& station);

struct HullContact {
    math::Vec3 force_station_n;    // on the vehicle, station body frame (the station gets -force)
    math::Vec3 torque_station_nm;  // about the vehicle CG, station body frame
    bool any = false;
    double max_depth_m = 0.0;
};
// Points and their velocities relative to the station, both in the station body frame; the deepest penetration of
// each point over all primitives, normal force k d - c v_n (never pulling).
HullContact hull_contact(const StationShape& shape, const std::vector<math::Vec3>& points_station_m,
                         const std::vector<math::Vec3>& velocities_station_mps, const math::Vec3& vehicle_cg_station_m,
                         const StationCollisionParameters& p);

}  // namespace starsim::contact
