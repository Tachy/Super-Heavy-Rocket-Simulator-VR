#pragma once

#include "starsim/gnc/descent_planner.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// What the active plan asks for at one moment (plan frame; rates relative to the plan frame, in its axes - the plan's
// roll is free, so its body axes are not the vehicle's).
struct TrackingReference {
    int phase = 0;
    bool finished = false;                     // past the plan's end: slow sink against gravity until the catch
    math::Vec3 position_m;
    math::Vec3 velocity_mps;
    math::Vec3 thrust_acceleration_mps2;       // planned thrust / planned mass
    math::Vec3 axis;                           // planned body x-axis
    math::Vec3 rate_plan_radps;
    math::Vec3 angular_acceleration_plan_radps2;
};

TrackingReference sample_plan(const DescentPlan& plan, const DescentModel& model, double t_s, double terminal_sink_mps);

struct TrackingGains {
    double frequency_rad_s = 0.6;     // [E] position loop (re-plans every second remove the rest)
    double damping = 1.0;
    double max_correction_mps2 = 2.0; // [E] keeps the correction a correction (the plan does the flying)
    // [E] largest tilt added to the planned axis: a slow-turning vehicle with gimbaled engines first pushes the
    // wrong way when it tilts (the gimbal side force), a large tilt command would saturate the gimbals.
    double max_tilt_correction_rad = 3.0 * constants::deg_to_rad;
};

// What to fly now: the planned body axis tilted by the lateral part of a position/velocity correction (relative to
// the planned thrust acceleration, at least gravity; bounded), and the planned thrust acceleration plus the
// correction's part along the planned thrust.
struct TrackingCommand {
    math::Vec3 axis;
    double thrust_acceleration_mps2 = 0.0;
};

TrackingCommand tracking_command(const TrackingReference& reference, const math::Vec3& position_m,
                                 const math::Vec3& velocity_mps, const TrackingGains& gains);

}  // namespace starsim::gnc
