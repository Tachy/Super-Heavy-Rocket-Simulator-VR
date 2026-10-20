#pragma once

#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/docking_interface.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::contact {

// Four-probe interface of the refuelling ships (refuelling r3), all values [E] (SpaceX publishes none). The chaser's
// probes (active) reach into conical drogues on the target (passive). Passive port frame: x out of the target, the
// drogue mouths in the plane x = 0, each cone narrowing to its seat at x = -cone_depth_m. A probe stands
// cone_depth_m + extension_m on the chaser's interface plane, so with the extension retracted to 0 and the pose mated
// (0, axes (-x, -y, +z)) every head sits in its seat. Each head meets its nearest drogue: the cone wall (normal towards
// the axis and out), the seat (the probe's attenuator) and, beside the mouth, the drogue plate.
struct ProbeDrogueParameters {
    std::vector<math::Vec3> probes_m;   // probe roots, chaser port frame (x = 0)
    std::vector<math::Vec3> drogues_m;  // mouth centres, passive port frame (x = 0)
    double cone_depth_m = 0.50;         // [E] mouth to seat
    double mouth_radius_m = 0.35;       // [E]
    double seat_radius_m = 0.06;        // [E]
    double head_radius_m = 0.05;        // [E] capture radius = mouth - head = 0.30 m (3x the envelope offset)
    double extension_m = 0.30;          // [E] probe stroke beyond the seated length, retracted after soft capture
    double wall_stiffness_n_per_m = 2.0e5;   // [E] per head
    double wall_damping_ns_per_m = 4.0e4;    // [E]
    double seat_stiffness_n_per_m = 5.0e5;   // [E] per head (attenuator)
    double seat_damping_ns_per_m = 1.0e5;    // [E]
    double plate_stiffness_n_per_m = 2.0e5;  // [E] a head beside the mouth
    double plate_damping_ns_per_m = 4.0e4;   // [E]
    double friction_coefficient = 0.1;       // [E] as IDSS
    double friction_slip_mps = 1e-3;
    double latch_gap_m = 0.01;  // [E] a head this close to its seat, centred: its capture latch closes
    double latch_tilt_max_rad = 2.0 * constants::deg_to_rad;  // [E]
    double latch_roll_max_rad = 2.0 * constants::deg_to_rad;  // [E]
};

// The contact of every probe head (sums: force, torque about the mean head position = ring_centre_m, compression = the
// +x part). latch_ready: one head seated and the tilt / roll within the latch limits (each probe latches alone; the
// mechanism's joint takes over). ring_contact = a head on a seat, petal_contact = a head on a cone wall, tip_strike = a
// head on the plate beside a mouth.
InterfaceContact probe_drogue_contact(const DockingPose& pose, double extension_m, const ProbeDrogueParameters& p);

// Probes from the chaser port's elements_m, drogues from the target's, in their port frames (x = axis, z = up).
ProbeDrogueParameters probe_drogue_parameters(const config::DockingPortConfig& chaser,
                                              const config::DockingPortConfig& target);

}  // namespace starsim::contact
