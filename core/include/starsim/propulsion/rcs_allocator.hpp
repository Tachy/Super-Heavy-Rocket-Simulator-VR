#pragma once

#include <vector>

#include "starsim/math/vec3.hpp"

namespace starsim::propulsion {

// One reaction-control thruster as the allocator sees it.
struct RcsThrusterGeometry {
    math::Vec3 position_m;  // body axes (station along x from the vehicle bottom)
    math::Vec3 direction;   // unit; the force on the vehicle (the plume points the other way)
    double thrust_n = 0.0;  // 0: unavailable (starved, failed)
};

// Allocation of a body torque and force to individual thrusters: duties d_i in [0, 1] (owed on-time per time) that
// minimise the weighted residual |W (B d - w)|^2 plus a small propellant tie-break, B the 6 x n effectiveness
// (torque about the CG and force per thruster at full thrust). A balanced layout (pure couples on every axis:
// thrusters fore and aft) weights the force rows over the torque rows: turns do not translate the vehicle, a saturated
// torque demand is cut instead. Other layouts (thrusters at one station) weight torque over force: they turn with a
// push and give up translation first. Solved by projected coordinate descent on the normal equations (n <= ~40).
// Pure: geometry in, duties out.
class RcsAllocator {
public:
    RcsAllocator() = default;
    RcsAllocator(std::vector<RcsThrusterGeometry> thrusters, double cg_station_m);

    size_t size() const { return thrusters_.size(); }
    double cg_station_m() const { return cg_station_m_; }
    bool balanced() const { return balanced_; }

    // Duties in [0, 1] for a torque about the CG and a force (see the class comment for the priorities); least
    // propellant among equals.
    // `warm_start` (same size) starts the descent from the previous tick's duties.
    std::vector<double> allocate(const math::Vec3& torque_nm, const math::Vec3& force_n,
                                 const std::vector<double>* warm_start = nullptr) const;
    math::Vec3 torque_nm(const std::vector<double>& duty) const;
    math::Vec3 force_n(const std::vector<double>& duty) const;

    // Per body axis: the smaller of the + and - torque capability (what a fraction of +-1 can rely on). Where the
    // layout has pure couples on the axis (>= 10 % of the coupled capability: thrusters fore and aft) only those count,
    // so turns do not translate the vehicle; otherwise (thrusters at one station) the coupled capability.
    math::Vec3 max_torque_nm() const { return max_torque_; }
    // Force along `direction_body` (unit) with no torque wanted; `duty` receives the duties that give it.
    double force_capability_n(const math::Vec3& direction_body, std::vector<double>* duty = nullptr) const;

private:
    std::vector<double> solve(const math::Vec3& torque_nm, const math::Vec3& force_n, double torque_weight,
                              double force_weight, const std::vector<double>* warm_start) const;

    struct Row {
        math::Vec3 torque;
        math::Vec3 force;
    };
    std::vector<RcsThrusterGeometry> thrusters_;
    double cg_station_m_ = 0.0;
    double torque_scale_ = 0.0;       // weight of the torque rows (priority / reference lever)
    double saturating_demand_ = 0.0;  // a demand beyond any capability (capability queries)
    std::vector<Row> rows_;
    std::vector<double> gram_torque_;  // n x n, sum of torque_i . torque_j, row-major
    std::vector<double> gram_force_;   // n x n, sum of force_i . force_j
    math::Vec3 max_torque_;
    bool balanced_ = false;
};

}  // namespace starsim::propulsion
