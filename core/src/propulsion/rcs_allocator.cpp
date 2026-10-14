#include "starsim/propulsion/rcs_allocator.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::propulsion {

using math::Vec3;

namespace {

// Torque rows are scaled by 1/reference lever (comparable to the force rows), then weighted by this: a saturated set
// gives up translation first.
constexpr double torque_priority = 100.0;
// Propellant tie-break per unit duty, relative to the largest demand gradient (|B^T W^2 w|): invariant to the weights.
constexpr double propellant_weight = 1e-6;
// Ridge on the normal matrix: B has rank <= 6, so G alone is singular; the ridge picks the smallest duties among
// equals. Relative to the smaller of the mean torque and force curvatures (a stiff force weight must not swamp the
// pure couples, whose curvature is torque only).
constexpr double ridge_weight = 1e-9;
// Multiplier tolerance of the active set (relative to the largest demand gradient), and its iteration limit per
// thruster.
constexpr double multiplier_tolerance = 1e-9;
constexpr int active_set_iterations_per_thruster = 4;

// Relative pivot floor of the modified Cholesky factorisation (round-off in nearly dependent columns).
constexpr double pivot_floor = 1e-13;

// Solves the symmetric positive semi-definite system a x = b in place (row-major m x m): Cholesky with pivots raised
// to a floor relative to the largest diagonal (a nearly dependent column then gets a tiny regularised value).
bool cholesky_solve(std::vector<double>& a, std::vector<double>& b, size_t m) {
    double largest = 0.0;
    for (size_t j = 0; j < m; ++j) largest = std::max(largest, a[j * m + j]);
    if (!(largest > 0.0)) return false;
    for (size_t j = 0; j < m; ++j) {
        double s = a[j * m + j];
        for (size_t k = 0; k < j; ++k) s -= a[j * m + k] * a[j * m + k];
        s = std::max(s, pivot_floor * largest);
        const double l = std::sqrt(s);
        a[j * m + j] = l;
        for (size_t i = j + 1; i < m; ++i) {
            double t = a[i * m + j];
            for (size_t k = 0; k < j; ++k) t -= a[i * m + k] * a[j * m + k];
            a[i * m + j] = t / l;
        }
    }
    for (size_t i = 0; i < m; ++i) {
        double t = b[i];
        for (size_t k = 0; k < i; ++k) t -= a[i * m + k] * b[k];
        b[i] = t / a[i * m + i];
    }
    for (size_t i = m; i-- > 0;) {
        double t = b[i];
        for (size_t k = i + 1; k < m; ++k) t -= a[k * m + i] * b[k];
        b[i] = t / a[i * m + i];
    }
    return true;
}
// Capability demands: this many times all thrusters together, so the solution saturates.
constexpr double saturating_factor = 10.0;
// Allocation of a balanced layout: force rows weighted this much over the (lever-scaled) torque rows, so the net
// force stays what was asked for (demands within the couple capability are met exactly, without a net force).
constexpr double couple_force_priority = 10.0;
// Pure-couple capability: the force rows weighted so stiffly that the net force is negligible (a single ring of
// thrusters, which has no pure pitch couple, then shows (almost) none).
constexpr double couple_capability_force_weight = 1e4;
// A layout counts as having pure couples on an axis when they reach this fraction of the coupled capability; then
// the controller relies on the couples alone (no translation while turning). Otherwise (thrusters at one station:
// Super Heavy, Falcon 9) it turns with a net push, as the real stages do.
constexpr double couple_fraction_min = 0.1;

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
    gram_torque_.assign(n * n, 0.0);
    gram_force_.assign(n * n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            gram_torque_[i * n + j] = dot(rows_[i].torque, rows_[j].torque);
            gram_force_[i * n + j] = dot(rows_[i].force, rows_[j].force);
        }
    }
    saturating_demand_ = saturating_factor * scale * static_cast<double>(std::max<size_t>(n, 1));

    bool all_couples = true;
    for (int i = 0; i < 3; ++i) {
        Vec3 e;
        axis(e, i) = 1.0;
        double coupled = saturating_demand_, couple = saturating_demand_;
        for (const double sign : {1.0, -1.0}) {
            const Vec3 u = e * sign;
            // Torque first (the balanced flag is not known yet: allocate() would depend on it).
            const double c = std::max(0.0, dot(torque_nm(solve(u * saturating_demand_, {}, torque_scale_, 1.0, nullptr)), u));
            coupled = std::min(coupled, c);
            // The largest torque along u the thrusters give with (nearly) no net force.
            const auto d = solve(u * c, {}, torque_scale_ / torque_priority, couple_capability_force_weight, nullptr);
            couple = std::min(couple, std::max(0.0, dot(torque_nm(d), u)));
        }
        const bool has_couples = coupled > 0.0 && couple >= couple_fraction_min * coupled;
        axis(max_torque_, i) = has_couples ? couple : coupled;
        all_couples = all_couples && has_couples;
    }
    balanced_ = n > 0 && all_couples;
}

std::vector<double> RcsAllocator::allocate(const Vec3& torque_nm, const Vec3& force_n,
                                           const std::vector<double>* warm_start) const {
    // Balanced layouts keep the force what was asked for (no translation from turning); others turn with a push.
    return balanced_ ? solve(torque_nm, force_n, torque_scale_ / torque_priority, couple_force_priority, warm_start)
                     : solve(torque_nm, force_n, torque_scale_, 1.0, warm_start);
}

std::vector<double> RcsAllocator::solve(const Vec3& torque_nm, const Vec3& force_n, double torque_weight,
                                        double force_weight, const std::vector<double>* warm_start) const {
    const size_t n = thrusters_.size();
    std::vector<double> d(n, 0.0);
    if (warm_start && warm_start->size() == n) d = *warm_start;
    const double wt2 = torque_weight * torque_weight, wf2 = force_weight * force_weight;
    std::vector<double> c(n);  // B^T W^2 w
    for (size_t i = 0; i < n; ++i) c[i] = wt2 * dot(rows_[i].torque, torque_nm) + wf2 * dot(rows_[i].force, force_n);
    auto gram = [&](size_t i, size_t j) { return wt2 * gram_torque_[i * n + j] + wf2 * gram_force_[i * n + j]; };
    double demand = 0.0, torque_curvature = 0.0, force_curvature = 0.0;
    for (size_t i = 0; i < n; ++i) {
        demand = std::max(demand, std::abs(c[i]));
        torque_curvature += wt2 * gram_torque_[i * n + i];
        force_curvature += wf2 * gram_force_[i * n + i];
    }
    const double curvature = torque_curvature > 0.0 && force_curvature > 0.0
                                 ? std::min(torque_curvature, force_curvature)
                                 : std::max(torque_curvature, force_curvature);
    const double tie_break = propellant_weight * demand;
    const double ridge = n > 0 ? ridge_weight * curvature / static_cast<double>(n) : 0.0;
    const double tolerance = multiplier_tolerance * demand;
    if (demand <= 0.0) return std::vector<double>(n, 0.0);  // nothing wanted
    // Primal active set on 0.5 d^T (G + ridge I) d - c^T d + tie_break * sum(d), 0 <= d <= 1. A thruster with no
    // thrust (starved) stays at 0.
    enum Bound { Free, Lower, Upper };
    std::vector<Bound> bound(n);
    for (size_t i = 0; i < n; ++i) {
        d[i] = gram(i, i) > 0.0 ? std::clamp(d[i], 0.0, 1.0) : 0.0;
        bound[i] = d[i] <= 0.0 ? Lower : (d[i] >= 1.0 ? Upper : Free);
    }
    auto gradient = [&](size_t i) {
        double g = tie_break - c[i] + ridge * d[i];
        for (size_t j = 0; j < n; ++j) g += gram(i, j) * d[j];
        return g;
    };
    const int max_iterations = active_set_iterations_per_thruster * static_cast<int>(n) + 10;
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        std::vector<size_t> free;
        for (size_t i = 0; i < n; ++i) {
            if (bound[i] == Free) free.push_back(i);
        }
        const size_t m = free.size();
        std::vector<double> a(m * m), y(m);
        for (size_t r = 0; r < m; ++r) {
            const size_t i = free[r];
            y[r] = c[i] - tie_break;
            for (size_t j = 0; j < n; ++j) {
                if (bound[j] != Free) y[r] -= gram(i, j) * d[j];
            }
            for (size_t s = 0; s < m; ++s) a[r * m + s] = gram(i, free[s]) + (r == s ? ridge : 0.0);
        }
        if (m > 0 && !cholesky_solve(a, y, m)) break;  // numerically singular: keep the feasible point
        // Step from d towards the free-set optimum y until a bound blocks.
        double step = 1.0;
        size_t blocking = n;
        for (size_t r = 0; r < m; ++r) {
            const size_t i = free[r];
            const double delta = y[r] - d[i];
            if (y[r] < 0.0 && delta < 0.0 && -d[i] / delta < step) {
                step = -d[i] / delta;
                blocking = i;
            } else if (y[r] > 1.0 && delta > 0.0 && (1.0 - d[i]) / delta < step) {
                step = (1.0 - d[i]) / delta;
                blocking = i;
            }
        }
        for (size_t r = 0; r < m; ++r) d[free[r]] += step * (y[r] - d[free[r]]);
        if (blocking < n) {
            d[blocking] = d[blocking] < 0.5 ? 0.0 : 1.0;
            bound[blocking] = d[blocking] == 0.0 ? Lower : Upper;
            continue;
        }
        // Optimal on the free set: release the bound with the most violated multiplier, if any.
        size_t release = n;
        double worst = tolerance;
        for (size_t i = 0; i < n; ++i) {
            if (bound[i] == Free || gram(i, i) <= 0.0) continue;
            const double g = gradient(i);
            const double violation = bound[i] == Lower ? -g : g;  // at 0 it wants to grow when g < 0
            if (violation > worst) {
                worst = violation;
                release = i;
            }
        }
        if (release == n) break;
        bound[release] = Free;
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
