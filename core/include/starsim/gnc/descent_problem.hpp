#pragma once

#include <array>
#include <vector>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/optim/socp_builder.hpp"

namespace starsim::gnc {

enum class DescentGoal { Catch, Abort };

// End of the landing burn, plan frame.
struct DescentTarget {
    DescentGoal goal = DescentGoal::Catch;
    math::Vec3 position_m;            // Catch: CG position at the catch
    math::Vec3 velocity_mps;          // Catch: velocity at the catch (sink 1 m/s); the catch ends upright with any
                                      // roll: the thrust has no roll authority, the tracking controller rolls the
                                      // pins towards the arms (gimbal roll, RCS)
    // Catch: vertical corridor while the hull passes the arms. Nodes whose reference CG height is below
    // corridor_top_m stay within corridor_radius_m of the catch axis (radius 0: no corridor).
    double corridor_top_m = 0.0;
    double corridor_radius_m = 0.0;
    double final_set_radius_m = 0.0;  // every final-set node this close to the catch axis (0: no limit)
    double ground_cg_height_m = 0.0;  // Abort: CG height at touchdown
    math::Vec3 sea_target_m;          // Abort: point at sea (y, z used)
    math::Vec3 keep_out_center_m;     // Abort: tower + mount, horizontal centre
    double keep_out_radius_m = 0.0;
    // Abort: the launch mount on the pad centre, kept clear by nodes whose reference CG height is below
    // mount_top_cg_height_m (radius 0: none).
    double mount_keep_out_radius_m = 0.0;
    double mount_top_cg_height_m = 0.0;
};

// Engine switch from phase p to p + 1, at their shared node.
struct EngineSwitchLimits {
    double max_tilt_rad = 0.0;    // calm switch: nearly upright (0: no limit)
    double max_rate_radps = 0.0;  // ... and hardly turning, pitch/yaw (0: no limit)
    double min_sink_mps = 0.0;    // still sinking at least this fast: the outgoing set at minimum throttle would climb
    double max_sink_mps = 0.0;
};

struct DescentLimits {
    // [E] per phase (13, 5, 3 engines): the more engines, the more gimbal authority.
    std::array<double, descent_phases> tilt_max_rad{25.0 * constants::deg_to_rad, 20.0 * constants::deg_to_rad,
                                                    10.0 * constants::deg_to_rad};
    double terminal_tilt_rad = 4.0 * constants::deg_to_rad;   // [E] at the catch / touchdown (structural)
    int tilt_taper_nodes = 4;                                 // the final phase's last nodes taper linearly to it
    // At the end of the plan the thrust is at most minimum + this fraction of the maximum: the final engines exceed
    // the weight even at minimum throttle and throttle at a limited rate, so the plan ends gently, not braking hard.
    double terminal_thrust_margin = 0.05;  // [E]
    // Engine switches (applied while still ahead of the plan's start phase). 13 -> 5: the webcasts of flights 13/14
    // show five engines at ~200 m/s, ~1.4 km [E]. 5 -> 3: calm (the three centre engines turn the booster slowly),
    // at a low sink rate; the final set then descends gently into the arms [E].
    std::array<EngineSwitchLimits, descent_phases - 1> switches{{
        {0.0, 0.0, 150.0, 250.0},
        {3.0 * constants::deg_to_rad, 0.03, 15.0, 30.0},
    }};
    double final_min_thrust_fraction = 0.0;  // [E] final-set thrust floor (fraction of its maximum) but at the end
    int final_ease_nodes = 3;                // last nodes free to ease down to the gentle end
    double throttle_margin = 0.1;  // [E] planned thrust stays this fraction below the maximum (tracking margin)
    // [E] planned gimbal deflection at most this fraction of the limit: the rest is the lateral controller's margin
    // (campaign: 0.4 lost one of 21 cases to saturation, 0.3 none).
    double gimbal_fraction = 0.3;
    // [E] pitch/yaw per phase: the 13 ignition engines turn the booster fast (alpha_max ~1.4 rad/s^2), the 3 final
    // engines slowly (~0.1-0.3 rad/s^2), the 5 divert engines in between.
    std::array<double, descent_phases> rate_max_radps{0.2, 0.15, 0.1};
    double glide_slope_rad = 30.0 * constants::deg_to_rad;    // above the horizontal, towards the catch point
    // Catch approach: sink <= catch sink + height above the catch point / this time constant (soft), so the booster
    // slows down into the arms as the V3 profile ends in a hover (0: no limit). [E]
    double approach_time_constant_s = 3.0;
    double min_mass_kg = 0.0;                                 // dry mass + reserve
    PhaseDurations min_phase_s{1.0, 1.0, 1.0};
    double max_phase_s = 120.0;
    // Start-up of the ignition engines (thrust = spool x throttled thrust, spool rising linearly over startup_s):
    // phase 0's thrust bounds scale with the spool at each node's reference time. The planning model's mass flow
    // keeps the full pressure-loss term meanwhile (a few kg too much; documented simplification).
    double phase0_initial_spool = 1.0;
    double phase0_startup_s = 0.0;
    // Re-plans: the thrust at the start (the measured thrust, > 0) can only change at the engines' throttle rate over
    // this window - the time until the plan takes effect. Without it a re-plan from a state slightly off its cones
    // brakes at once, within the latency the engines cannot follow, and the next plan finds the booster further off.
    double initial_thrust_n = 0.0;      // 0: free
    double initial_thrust_window_s = 0.5;
};

struct DescentWeights {
    double fuel = 1.0;               // per 1e4 kg
    double virtual_control = 1.0e3;  // L1, scaled units
    double trust_region = 1.0e-1;    // squared scaled step
    double sea_distance = 1.0e-2;    // per metre (abort)
    double final_lateral = 0.01;     // [E] catch: per metre from the catch axis, per final-set node
    double attitude_violation = 10.0;  // [E] per unit above the tilt (quaternion y,z norm) or rate (rad/s) limit;
                                       // higher values degrade the SOCP conditioning
    double corridor_violation = 10.0;  // [E] per metre outside the catch corridor, per node (~100 t of propellant)
    double final_lateral_time_s = 2.0;  // [E] lateral speed counts as the offset it builds up in this time
};

// Iterate of SCvx: states and thrust at P (N-1) + 1 nodes (P phases of N nodes, neighbours sharing one), phase
// durations.
struct DescentReference {
    std::vector<DescentState> x;
    std::vector<math::Vec3> u;  // body frame
    PhaseDurations duration_s{};
};

// Nodes of a plan with `nodes_per_phase` nodes per phase.
constexpr int descent_node_count(int nodes_per_phase) { return descent_phases * (nodes_per_phase - 1) + 1; }
// Phase whose bounds node k has (a shared node starts the next phase; the last node belongs to the last phase).
constexpr int descent_node_phase(int k, int nodes_per_phase) {
    return k / (nodes_per_phase - 1) < descent_phases ? k / (nodes_per_phase - 1) : descent_phases - 1;
}

// Scales of the state deviation variables (one unit = this much physical change).
std::array<double, 14> descent_state_scales();

// Time of node k from the plan start for the given phase durations (nodes equally spaced within a phase).
double reference_node_time(const PhaseDurations& duration_s, int k, int nodes_per_phase);

// Convex subproblem of one SCvx iteration in scaled deviations from the reference.
class DescentSubproblem {
public:
    optim::SocpProblem problem;

    DescentReference unpack(const std::vector<double>& solution) const;
    double virtual_control(const std::vector<double>& solution) const;  // L1, scaled
    double max_step(const std::vector<double>& solution) const;         // largest scaled node step

    // Layout (filled by build_descent_subproblem).
    DescentReference reference;
    int nodes = 0;
    int start_phase = 0;
    std::vector<optim::VariableBlock> dx, du, nu;
    optim::VariableBlock ds;
    std::array<double, 14> state_scale{};
    double thrust_scale = 1.0;
};

// Nodes 0 .. P(N-1); interval k belongs to phase k / (N-1); the shared node p (N-1) has phase p's bounds.
// `start_phase` s fixes the durations of phases 0 .. s-1 to 0 (their nodes are copies of the initial state).
DescentSubproblem build_descent_subproblem(const DescentModel& model, const DescentReference& reference,
                                           const std::vector<IntervalLinearization>& linearization,
                                           const DescentState& initial, int start_phase, const DescentTarget& target,
                                           const DescentLimits& limits, const DescentWeights& weights, int nodes_per_phase);

}  // namespace starsim::gnc
