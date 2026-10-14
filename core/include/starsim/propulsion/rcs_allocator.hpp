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
// (torque about the CG and force per thruster at full thrust). Torque rows are scaled by the reference lever and
// weighted over force rows: a saturated set gives up translation first. Solved by projected coordinate descent on
// the normal equations (n <= ~40). Pure: geometry in, duties out.
class RcsAllocator {
public:
    RcsAllocator() = default;
    RcsAllocator(std::vector<RcsThrusterGeometry> thrusters, double cg_station_m);

    size_t size() const { return thrusters_.size(); }
    double cg_station_m() const { return cg_station_m_; }

    // Duties in [0, 1]: torque about the CG first (weighted), then force; least propellant among equals.
    // `warm_start` (same size) starts the descent from the previous tick's duties.
    std::vector<double> allocate(const math::Vec3& torque_nm, const math::Vec3& force_n,
                                 const std::vector<double>* warm_start = nullptr) const;
    math::Vec3 torque_nm(const std::vector<double>& duty) const;
    math::Vec3 force_n(const std::vector<double>& duty) const;

    // Per body axis: the smaller of the + and - torque capability (what a fraction of +-1 can rely on).
    math::Vec3 max_torque_nm() const { return max_torque_; }
    // Force along `direction_body` (unit) with no torque wanted; `duty` receives the duties that give it.
    double force_capability_n(const math::Vec3& direction_body, std::vector<double>* duty = nullptr) const;

private:
    struct Row {
        math::Vec3 torque;
        math::Vec3 force;
    };
    std::vector<RcsThrusterGeometry> thrusters_;
    double cg_station_m_ = 0.0;
    double torque_scale_ = 0.0;       // weight of the torque rows (priority / reference lever)
    double tie_break_ = 0.0;          // propellant weight per unit duty
    double saturating_demand_ = 0.0;  // a demand beyond any capability (capability queries)
    std::vector<Row> rows_;
    std::vector<double> gram_;  // n x n normal matrix B^T W^2 B, row-major
    math::Vec3 max_torque_;
};

}  // namespace starsim::propulsion
