#include "starsim/propulsion/rcs_allocator.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::propulsion {

using math::Vec3;

namespace {

// Torque rows are scaled by 1/reference lever (comparable to the force rows), then weighted by this: a saturated set
// gives up translation first.
constexpr double torque_priority = 100.0;
// Propellant tie-break per unit duty, relative to the mean diagonal of the normal matrix.
constexpr double propellant_weight = 1e-6;
constexpr int max_sweeps = 300;
constexpr double duty_tolerance = 1e-7;
// Capability demands: this many times all thrusters together, so the solution saturates.
constexpr double saturating_factor = 10.0;

double& axis(Vec3& v, int i) { return i == 0 ? v.x : (i == 1 ? v.y : v.z); }

}  // namespace

RcsAllocator::RcsAllocator(std::vector<RcsThrusterGeometry> thrusters, double cg_station_m)
    : thrusters_(std::move(thrusters)), cg_station_m_(cg_station_m) {
    const size_t n = thrusters_.size();
    const Vec3 cg{cg_station_m_, 0.0, 0.0};
    double lever = 1.0;
    for (const auto& t : thrusters_) lever = std::max(lever, norm(t.position_m - cg));
    torque_scale_ = torque_priority / lever;
    rows_.resize(n);
    double scale = 0.0;  // largest single-thruster torque / force
    for (size_t i = 0; i < n; ++i) {
        const auto& t = thrusters_[i];
        const Vec3 f = t.direction * t.thrust_n;
        rows_[i] = {cross(t.position_m - cg, f), f};
        scale = std::max({scale, norm(rows_[i].torque), norm(rows_[i].force)});
    }
    gram_.assign(n * n, 0.0);
    double diagonal = 0.0;
    const double w2 = torque_scale_ * torque_scale_;
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            gram_[i * n + j] = w2 * dot(rows_[i].torque, rows_[j].torque) + dot(rows_[i].force, rows_[j].force);
        }
        diagonal += gram_[i * n + i];
    }
    tie_break_ = n > 0 ? propellant_weight * diagonal / static_cast<double>(n) : 0.0;
    saturating_demand_ = saturating_factor * scale * static_cast<double>(std::max<size_t>(n, 1));
    for (int i = 0; i < 3; ++i) {
        Vec3 e;
        axis(e, i) = 1.0;
        const double plus = dot(torque_nm(allocate(e * saturating_demand_, {})), e);
        const double minus = -dot(torque_nm(allocate(e * -saturating_demand_, {})), e);
        axis(max_torque_, i) = std::max(0.0, std::min(plus, minus));
    }
}

std::vector<double> RcsAllocator::allocate(const Vec3& torque_nm, const Vec3& force_n,
                                           const std::vector<double>* warm_start) const {
    const size_t n = thrusters_.size();
    std::vector<double> d(n, 0.0);
    if (warm_start && warm_start->size() == n) d = *warm_start;
    const double w2 = torque_scale_ * torque_scale_;
    std::vector<double> c(n);  // B^T W^2 w
    for (size_t i = 0; i < n; ++i) c[i] = w2 * dot(rows_[i].torque, torque_nm) + dot(rows_[i].force, force_n);
    // Projected coordinate descent on 0.5 d^T G d - c^T d + tie_break * sum(d), d in [0, 1].
    for (int sweep = 0; sweep < max_sweeps; ++sweep) {
        double change = 0.0;
        for (size_t i = 0; i < n; ++i) {
            const double gii = gram_[i * n + i];
            if (gii <= 0.0) {
                d[i] = 0.0;
                continue;
            }
            double gradient = tie_break_ - c[i];
            for (size_t j = 0; j < n; ++j) gradient += gram_[i * n + j] * d[j];
            const double next = std::clamp(d[i] - gradient / gii, 0.0, 1.0);
            change = std::max(change, std::abs(next - d[i]));
            d[i] = next;
        }
        if (change < duty_tolerance) break;
    }
    return d;
}

Vec3 RcsAllocator::torque_nm(const std::vector<double>& duty) const {
    Vec3 sum;
    for (size_t i = 0; i < duty.size() && i < rows_.size(); ++i) sum += rows_[i].torque * duty[i];
    return sum;
}

Vec3 RcsAllocator::force_n(const std::vector<double>& duty) const {
    Vec3 sum;
    for (size_t i = 0; i < duty.size() && i < rows_.size(); ++i) sum += rows_[i].force * duty[i];
    return sum;
}

double RcsAllocator::force_capability_n(const Vec3& direction_body, std::vector<double>* duty) const {
    const Vec3 u = math::normalized_or_zero(direction_body);
    auto d = allocate({}, u * saturating_demand_);
    const double f = std::max(0.0, dot(force_n(d), u));
    if (duty) *duty = std::move(d);
    return f;
}

}  // namespace starsim::propulsion
