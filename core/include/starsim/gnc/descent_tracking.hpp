#pragma once

#include "starsim/gnc/descent_planner.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// What the active plan asks for at one moment (plan frame; rates body axes, relative to the plan frame).
struct TrackingReference {
    int phase = 0;
    bool finished = false;                     // past the plan's end: slow sink against gravity until the catch
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
    math::Vec3 thrust_acceleration_mps2;       // planned thrust / planned mass
    math::Vec3 axis;                           // planned body x-axis
    math::Vec3 rate_body_radps;
    math::Vec3 angular_acceleration_body_radps2;
};

TrackingReference sample_plan(const DescentPlan& plan, const DescentModel& model, double t_s, double terminal_sink_mps);

struct TrackingGains {
    double frequency_rad_s = 0.6;     // [E] position loop (re-plans every second remove the rest)
    double damping = 1.0;
    double max_correction_mps2 = 2.0; // [E] keeps the correction a correction (the plan does the flying)
    double max_tilt_rad = 25.0 * constants::deg_to_rad;  // thrust from the vertical (phase 0 limit)
};

// Thrust acceleration to fly now: planned plus a PD correction of the position/velocity error (bounded), the
// direction limited to max_tilt from the vertical.
math::Vec3 commanded_acceleration(const TrackingReference& reference, const math::Vec3& position_m,
                                  const math::Vec3& velocity_mps, const TrackingGains& gains);

}  // namespace starsim::gnc
