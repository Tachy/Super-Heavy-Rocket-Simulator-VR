#pragma once

namespace starsim::gnc {

// Live state of an abort towards the water, in the vertical plane through the sea direction.
struct SeaReachInput {
    double distance_m = 0.0;            // to the water's aim (shoreline + clearance) along the sea direction
    double seaward_speed_mps = 0.0;     // along the sea direction
    double height_m = 0.0;              // of the CG above its height on the ground
    double sink_mps = 0.0;
    double mass_kg = 0.0;
    double max_accel_mps2 = 0.0;        // thrust of the running engine set at full throttle / mass
    double exhaust_velocity_mps = 0.0;
    double gravity_mps2 = 0.0;
    double tilt_max_rad = 0.0;          // the guidance's limit
    double tilt_rate_radps = 0.0;       // the guidance's limit; 0: instant
};

struct SeaReachEstimate {
    double time_s = 0.0;         // flight time of the estimate
    double delta_v_mps = 0.0;
    double propellant_kg = 0.0;  // rocket equation for delta_v_mps
};

// What the flight to the water and a stop on it costs at least. Lateral bang-bang at the tilt limit (accelerate, turn the
// tilt round, brake), the vertical thrust at the constant g + sink / T that ends the sink at T: the shortest T with
// that reach, not shorter than the fall from the height with a brake at full thrust. dv = (g T + sink) / cos(tilt): a
// longer flight costs hover propellant at any height, drag is not counted (slow flight; it would help a little).
SeaReachEstimate estimate_sea_reach(const SeaReachInput& input);

}  // namespace starsim::gnc
