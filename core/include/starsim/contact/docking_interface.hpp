#pragma once

#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::contact {

// IDSS soft-capture interface before capture (ISS i5), in the passive port frame (x out of the passive port, z its up;
// vehicle/docking_port.hpp). Reduced order (spec correction 2): the three petal pairs act as one axisymmetric cone
// between the ring centres; the active ring conforms in tilt within ring_tilt_max_rad (its actuator platform).

// The chaser port (its HCS mating plane centre) relative to the passive port, passive port frame (rotating with the
// passive body): mated = position 0, chaser_axes columns (-x, -y, +z).
struct DockingPose {
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
    math::Mat3 chaser_axes = math::Mat3::identity();  // columns: chaser port x, y, z in the passive port frame
    math::Vec3 angular_rate_radps;                     // chaser relative to the passive body, passive port frame
};

struct DockingInterfaceParameters {
    int petal_count = 3;                    // [S: IDSS IDD E 3.2.2.1] three inward-pointing guide petals
    double petal_outline_radius_m = 0.600;  // [S: IDSS IDD E 3.2] conic outline 1200 mm at the SCS mating plane
    double petal_height_m = 0.20;           // [E] above the ring face (figures only in the IDD)
    double petal_lean_rad = 30.0 * constants::deg_to_rad;  // [E] inward lean of the petal outline from the axis
    double petal_clearance_m = 0.01;        // [E] lateral play of the meshed petals at full ring contact
    double ring_extension_m = 0.30;         // [E] active ring face above the chaser HCS plane (passive ring at 0)
    double ring_inner_radius_m = 0.40;      // [E] ring face (striker band) inner radius
    double ring_outer_radius_m = 0.60;      // [S: = the outline radius]
    // [E] ring-face stiffness and damping: a 0.10 m/s contact on the Dragon-ISS pair (reduced mass ~12 t) peaks at
    // ~4.0 kN, below the IDSS dynamic compression limit 6,500 N (test); damping ratio ~0.3 at that mass.
    double ring_stiffness_n_per_m = 2.0e5;
    double ring_damping_ns_per_m = 2.9e4;
    double petal_stiffness_n_per_m = 2.0e5;  // [E] normal to the petal cone
    double petal_damping_ns_per_m = 2.9e4;   // [E]
    double friction_coefficient = 0.1;       // [E] metal on metal, lubricated (regularised Coulomb)
    double friction_slip_mps = 1e-3;         // [E] regularisation speed
    // Latch condition (soft capture, IDSS IDD E 3.1.1.1: rings in full contact, latches on the strikers) [E]:
    double latch_gap_m = 0.005;                               // ring faces within this
    double latch_offset_max_m = 0.02;                         // ring centres after the petal centring
    double ring_tilt_max_rad = 6.0 * constants::deg_to_rad;   // the active ring conforms up to this tilt
    double latch_roll_max_rad = 6.0 * constants::deg_to_rad;  // petal flanks / latch strikers in roll
};

struct InterfaceContact {
    math::Vec3 force_port_n;     // on the chaser at the active ring centre, passive port frame (the passive body: -force)
    math::Vec3 ring_centre_m;    // where the force acts: the active ring face centre, passive port frame
    bool any_contact = false;
    bool ring_contact = false;   // ring faces touching
    bool petal_contact = false;  // on the petal cone (centring)
    bool tip_strike = false;     // petal tips on the opposing petals (offset beyond the capture radius)
    bool latch_ready = false;    // latch condition met
    double compression_n = 0.0;  // axial load on the interface (+: compression)
    double capture_radius_m = 0.0;
};

// `ring_extension_m`: current extension of the active ring (contact before capture uses the full extension).
InterfaceContact docking_interface_contact(const DockingPose& pose, double ring_extension_m,
                                           const DockingInterfaceParameters& p);

}  // namespace starsim::contact
