#pragma once

#include <array>
#include <vector>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/optim/socp_builder.hpp"

namespace starsim::gnc {

enum class DescentGoal { Catch, Abort };

// End of the landing burn, plan frame.
struct DescentTarget {
    DescentGoal goal = DescentGoal::Catch;
    math::Vec3 position_m;            // Catch: CG position at the catch
    math::Vec3 velocity_mps;          // Catch: velocity at the catch (sink 1 m/s)
    math::Quat attitude;              // Catch: upright, pins across the arms (sign nearest the initial attitude)
    double ground_cg_height_m = 0.0;  // Abort: CG height at touchdown
    math::Vec3 sea_target_m;          // Abort: point at sea (y, z used)
    math::Vec3 keep_out_center_m;     // Abort: tower + mount, horizontal centre
    double keep_out_radius_m = 0.0;
};

struct DescentLimits {
    std::array<double, 2> tilt_max_rad{25.0 * 3.14159265358979 / 180.0, 10.0 * 3.14159265358979 / 180.0};  // per phase
    double rate_max_radps = 0.1;                                // [E] structural / controllability
    double glide_slope_rad = 30.0 * 3.14159265358979 / 180.0;  // above the horizontal, towards the catch point
    double min_mass_kg = 0.0;                                   // dry mass + reserve
    std::array<double, 2> min_phase_s{1.0, 1.0};
    double max_phase_s = 120.0;
};

struct DescentWeights {
    double fuel = 1.0;               // per 1e4 kg
    double virtual_control = 1.0e3;  // L1, scaled units
    double trust_region = 1.0e-1;    // squared scaled step
    double sea_distance = 1.0e-2;    // per metre (abort)
};

// Iterate of SCvx: states and thrust at 2N-1 nodes, phase durations.
struct DescentReference {
    std::vector<DescentState> x;
    std::vector<math::Vec3> u;  // body frame
    std::array<double, 2> duration_s{};
};

// Scales of the state deviation variables (one unit = this much physical change).
std::array<double, 14> descent_state_scales();

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

// Nodes 0 .. 2N-2; intervals 0 .. N-2 belong to phase 0, N-1 .. 2N-3 to phase 1; node N-1 is shared and has
// the final set's bounds. `start_phase` 1 fixes phase 0's duration to 0.
DescentSubproblem build_descent_subproblem(const DescentModel& model, const DescentReference& reference,
                                           const std::vector<IntervalLinearization>& linearization,
                                           const DescentState& initial, int start_phase, const DescentTarget& target,
                                           const DescentLimits& limits, const DescentWeights& weights, int nodes_per_phase);

}  // namespace starsim::gnc
