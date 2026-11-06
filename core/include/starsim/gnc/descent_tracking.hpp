#pragma once

#include <vector>

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

// The plan's own trajectory between its nodes: every interval propagated through the planning model from its start
// node with the plan's first-order-hold thrust (like the discretisation, with a finer step; the converged plan meets the
// next node within the planner's defect tolerance), sampled at most plan_path_step_s apart. The nodes alone are a chord of it: on the
// ship's long pair phase (~1.9 s between nodes) the chord ran up to ~3 m above the braking path (an upward
// acceleration a bends the path below the chord by ~a dt^2 / 8) and 1.6 deg off its axis, and the tracking held the
// vehicle on that chord - higher and slower than planned, every re-plan later (r1 fix, 2026-11-06).
struct PlanPath {
    std::vector<double> t_s;         // from the plan's start, increasing
    std::vector<DescentState> x;
};
// Sample spacing: the linear interpolation between samples leaves ~a h^2 / 8 of the path: 2 mm at the ship's 7 m/s^2,
// 6 mm at the ~20 m/s^2 of a Falcon 9 hoverslam near empty.
constexpr double plan_path_step_s = 0.05;
PlanPath plan_path(const DescentPlan& plan, const DescentModel& model);
// The path's state at t_s (linear between samples, quaternion normalised); t_s clamped to the path. An empty path has
// no state: the zero state is returned (callers check PlanPath::t_s.empty(), as sample_plan does).
DescentState path_state(const PlanPath& path, double t_s);

// The reference at t_s: the path's state (empty path: the nodes' linear interpolation, DescentPlan::state_at).
TrackingReference sample_plan(const DescentPlan& plan, const DescentModel& model, double t_s, double terminal_sink_mps,
                              const PlanPath& path = {});

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
