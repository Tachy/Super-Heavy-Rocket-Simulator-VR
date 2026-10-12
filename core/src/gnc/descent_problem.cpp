#include "starsim/gnc/descent_problem.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::gnc {

using math::Vec3;
using optim::AffineExpr;

namespace {

// Scales of the deviation variables (one unit of a variable = this much physical change).
constexpr double mass_scale_kg = 1.0e4;
constexpr double position_scale_m = 10.0;
constexpr double velocity_scale_mps = 10.0;
constexpr double attitude_scale = 0.1;
constexpr double rate_scale_radps = 0.05;
constexpr double thrust_scale_fraction = 0.1;  // of phase 0's maximum thrust
constexpr double duration_scale_s = 1.0;
constexpr double min_reference_thrust_n = 1.0;  // below: no direction for the lower-bound linearisation

std::array<double, 14> state_scales() {
    std::array<double, 14> s{};
    s[0] = mass_scale_kg;
    for (size_t i = 1; i < 4; ++i) s[i] = position_scale_m;
    for (size_t i = 4; i < 7; ++i) s[i] = velocity_scale_mps;
    for (size_t i = 7; i < 11; ++i) s[i] = attitude_scale;
    for (size_t i = 11; i < 14; ++i) s[i] = rate_scale_radps;
    return s;
}

AffineExpr scaled(AffineExpr e, double factor) {
    for (auto& term : e.terms) term.second *= factor;
    e.constant *= factor;
    return e;
}

}  // namespace

DescentSubproblem build_descent_subproblem(const DescentModel& model, const DescentReference& ref,
                                           const std::vector<IntervalLinearization>& lin, const DescentState& initial,
                                           int start_phase, const DescentTarget& target, const DescentLimits& limits,
                                           const DescentWeights& weights, int n) {
    DescentSubproblem sub;
    sub.reference = ref;
    sub.start_phase = start_phase;
    sub.nodes = 2 * n - 1;
    sub.state_scale = state_scales();
    sub.thrust_scale = thrust_scale_fraction * model.engine_sets[0].thrust_max_n;
    const auto& S = sub.state_scale;
    const double su = sub.thrust_scale;
    const int K = sub.nodes;
    const int boundary = n - 1;
    const auto at = [](int k) { return static_cast<size_t>(k); };

    optim::SocpBuilder b;
    for (int k = 0; k < K; ++k) {
        sub.dx.push_back(b.add_variables(14, "dx"));
        sub.du.push_back(b.add_variables(3, "du"));
    }
    sub.ds = b.add_variables(2, "ds");
    std::vector<optim::VariableBlock> abs_nu;
    for (int k = 0; k < K - 1; ++k) {
        sub.nu.push_back(b.add_variables(14, "nu"));
        abs_nu.push_back(b.add_variables(14, "|nu|"));
    }
    const auto eta = b.add_variables(K, "eta");
    const auto eta_s = b.add_variables(1, "eta_s");
    const auto distance = b.add_variables(1, "sea_distance");

    // Physical values as affine expressions of the deviations.
    auto X = [&](int k, int i) { return AffineExpr{ref.x[at(k)][i]}.add(sub.dx[at(k)][i], S[static_cast<size_t>(i)]); };
    auto U = [&](int k, int j) {
        const Vec3& u = ref.u[at(k)];
        const double v[3] = {u.x, u.y, u.z};
        return AffineExpr{v[j]}.add(sub.du[at(k)][j], su);
    };

    // Dynamics, each row divided by its state scale.
    for (int k = 0; k < K - 1; ++k) {
        const auto& L = lin[at(k)];
        const int p = k < boundary ? 0 : 1;
        for (int i = 0; i < 14; ++i) {
            const double inv = 1.0 / S[static_cast<size_t>(i)];
            AffineExpr row{(ref.x[at(k + 1)][i] - L.propagated[i]) * inv};
            row.add(sub.dx[at(k + 1)][i], 1.0);
            for (int j = 0; j < 14; ++j) {
                row.add(sub.dx[at(k)][j], -L.phi[static_cast<size_t>(i * 14 + j)] * S[static_cast<size_t>(j)] * inv);
            }
            for (int j = 0; j < 3; ++j) {
                row.add(sub.du[at(k)][j], -L.gamma_minus[static_cast<size_t>(i * 3 + j)] * su * inv);
                row.add(sub.du[at(k + 1)][j], -L.gamma_plus[static_cast<size_t>(i * 3 + j)] * su * inv);
            }
            row.add(sub.ds[p], -L.sigma[static_cast<size_t>(i)] * duration_scale_s * inv);
            row.add(sub.nu[at(k)][i], -1.0);
            b.add_equality(row);
            b.add_nonnegative(AffineExpr{}.add(abs_nu[at(k)][i], 1.0).add(sub.nu[at(k)][i], -1.0));
            b.add_nonnegative(AffineExpr{}.add(abs_nu[at(k)][i], 1.0).add(sub.nu[at(k)][i], 1.0));
        }
    }
    // Initial state.
    for (int i = 0; i < 14; ++i) {
        b.add_equality(AffineExpr{(ref.x[0][i] - initial[i]) / S[static_cast<size_t>(i)]}.add(sub.dx[0][i], 1.0));
    }
    // Terminal state.
    const int last = K - 1;
    if (target.goal == DescentGoal::Catch) {
        const double t[13] = {target.position_m.x,   target.position_m.y,   target.position_m.z,
                              target.velocity_mps.x, target.velocity_mps.y, target.velocity_mps.z,
                              target.attitude.w,     target.attitude.x,     target.attitude.y,
                              target.attitude.z,     0.0,                   0.0,
                              0.0};
        for (int i = 1; i < 14; ++i) b.add_equality(X(last, i).add_constant(-t[i - 1]));
    } else {
        b.add_equality(X(last, descent_index::position).add_constant(-target.ground_cg_height_m));
        for (int i = descent_index::velocity; i < descent_index::velocity + 3; ++i) b.add_equality(X(last, i));
        for (int i = descent_index::rate; i < 14; ++i) b.add_equality(X(last, i));
        b.add_second_order_cone({AffineExpr{}.add(distance[0], 1.0), X(last, 2).add_constant(-target.sea_target_m.y),
                                 X(last, 3).add_constant(-target.sea_target_m.z)});
    }
    // Path constraints.
    const auto& e0 = model.engine_sets[0];
    const auto& e1 = model.engine_sets[1];
    for (int k = 0; k < K; ++k) {
        // The shared node starts the final set: 13 engines at minimum throttle give more thrust than 3 at full,
        // so the thrust steps down over phase 0's last interval (engines running down at the switch).
        const int p = k < boundary ? 0 : 1;
        const auto& set = p == 0 ? e0 : e1;
        const double t_max = set.thrust_max_n;
        const double t_min = set.thrust_min_n;
        const double gimbal = set.gimbal_max_rad;
        const double tilt = limits.tilt_max_rad[static_cast<size_t>(p)];
        b.add_second_order_cone({AffineExpr{t_max}, U(k, 0), U(k, 1), U(k, 2)});
        // Lower thrust bound, linearised: (ubar / |ubar|) . u >= t_min.
        const Vec3& ub = ref.u[at(k)];
        const Vec3 dir = norm(ub) >= min_reference_thrust_n ? ub / norm(ub) : Vec3{1.0, 0.0, 0.0};
        const double d[3] = {dir.x, dir.y, dir.z};
        AffineExpr lower{-t_min};
        for (int j = 0; j < 3; ++j) {
            const AffineExpr uj = U(k, j);
            lower.add_constant(d[j] * uj.constant);
            for (const auto& [index, c] : uj.terms) lower.add(index, d[j] * c);
        }
        b.add_nonnegative(lower);
        b.add_second_order_cone({scaled(U(k, 0), std::tan(gimbal)), U(k, 1), U(k, 2)});
        b.add_second_order_cone({AffineExpr{std::sqrt(0.5 * (1.0 - std::cos(tilt)))}, X(k, descent_index::attitude + 2),
                                 X(k, descent_index::attitude + 3)});
        b.add_second_order_cone({AffineExpr{limits.rate_max_radps}, X(k, descent_index::rate),
                                 X(k, descent_index::rate + 1), X(k, descent_index::rate + 2)});
        b.add_nonnegative(X(k, descent_index::mass).add_constant(-limits.min_mass_kg));
        if (target.goal == DescentGoal::Catch) {
            const double tg = std::tan(limits.glide_slope_rad);
            b.add_second_order_cone({X(k, 1).add_constant(-target.position_m.x),
                                     scaled(X(k, 2).add_constant(-target.position_m.y), tg),
                                     scaled(X(k, 3).add_constant(-target.position_m.z), tg)});
        } else {
            // Keep-out: n . (r - c) >= R with n the horizontal direction from the centre to the reference.
            const Vec3 rb = state_position(ref.x[at(k)]) - target.keep_out_center_m;
            const double h = std::hypot(rb.y, rb.z);
            const double ny = h > 1e-6 ? rb.y / h : 1.0;
            const double nz = h > 1e-6 ? rb.z / h : 0.0;
            const AffineExpr ry = X(k, 2), rz = X(k, 3);
            AffineExpr keep{-target.keep_out_radius_m - ny * target.keep_out_center_m.y - nz * target.keep_out_center_m.z};
            for (const auto& [index, c] : ry.terms) keep.add(index, ny * c);
            for (const auto& [index, c] : rz.terms) keep.add(index, nz * c);
            keep.add_constant(ny * ry.constant + nz * rz.constant);
            b.add_nonnegative(keep);
        }
        // Trust region: eta_k >= ||(dx_k, du_k)||^2 as a rotated cone ||(2 v, eta - 1)|| <= eta + 1.
        std::vector<AffineExpr> cone{AffineExpr{1.0}.add(eta[k], 1.0)};
        for (int i = 0; i < 14; ++i) cone.push_back(AffineExpr{}.add(sub.dx[at(k)][i], 2.0));
        for (int j = 0; j < 3; ++j) cone.push_back(AffineExpr{}.add(sub.du[at(k)][j], 2.0));
        cone.push_back(AffineExpr{-1.0}.add(eta[k], 1.0));
        b.add_second_order_cone(cone);
    }
    b.add_second_order_cone({AffineExpr{1.0}.add(eta_s[0], 1.0), AffineExpr{}.add(sub.ds[0], 2.0),
                             AffineExpr{}.add(sub.ds[1], 2.0), AffineExpr{-1.0}.add(eta_s[0], 1.0)});
    // Phase durations.
    for (int p = 0; p < 2; ++p) {
        const AffineExpr s = AffineExpr{ref.duration_s[static_cast<size_t>(p)]}.add(sub.ds[p], duration_scale_s);
        if (p == 0 && start_phase == 1) {
            b.add_equality(s);
            continue;
        }
        b.add_nonnegative(AffineExpr{s}.add_constant(-limits.min_phase_s[static_cast<size_t>(p)]));
        b.add_nonnegative(AffineExpr{limits.max_phase_s - ref.duration_s[static_cast<size_t>(p)]}.add(sub.ds[p], -duration_scale_s));
    }
    // Objective: maximum final mass, small virtual control, small steps (and close to the sea on an abort).
    AffineExpr objective;
    objective.add(sub.dx[at(last)][descent_index::mass], -weights.fuel * S[0] / mass_scale_kg);
    for (const auto& block : abs_nu) {
        for (int i = 0; i < 14; ++i) objective.add(block[i], weights.virtual_control);
    }
    for (int k = 0; k < K; ++k) objective.add(eta[k], weights.trust_region);
    objective.add(eta_s[0], weights.trust_region);
    if (target.goal == DescentGoal::Abort) {
        objective.add(distance[0], weights.sea_distance);
    } else {
        b.add_nonnegative(AffineExpr{}.add(distance[0], 1.0));  // unused: keep the variable bounded (zero cost)
    }
    b.set_objective(objective);
    sub.problem = b.build();
    return sub;
}

DescentReference DescentSubproblem::unpack(const std::vector<double>& s) const {
    const auto at = [](int i) { return static_cast<size_t>(i); };
    DescentReference out = reference;
    for (int k = 0; k < nodes; ++k) {
        for (int i = 0; i < 14; ++i) out.x[at(k)][i] += state_scale[at(i)] * s[at(dx[at(k)][i])];
        out.u[at(k)] += Vec3{s[at(du[at(k)][0])], s[at(du[at(k)][1])], s[at(du[at(k)][2])]} * thrust_scale;
    }
    for (int p = 0; p < 2; ++p) out.duration_s[at(p)] += duration_scale_s * s[at(ds[p])];
    if (start_phase == 1) out.duration_s[0] = 0.0;
    return out;
}

double DescentSubproblem::virtual_control(const std::vector<double>& s) const {
    double sum = 0.0;
    for (const auto& block : nu) {
        for (int i = 0; i < 14; ++i) sum += std::abs(s[static_cast<size_t>(block[i])]);
    }
    return sum;
}

double DescentSubproblem::max_step(const std::vector<double>& s) const {
    double largest = 0.0;
    for (int k = 0; k < nodes; ++k) {
        double sq = 0.0;
        for (int i = 0; i < 14; ++i) sq += std::pow(s[static_cast<size_t>(dx[static_cast<size_t>(k)][i])], 2);
        for (int j = 0; j < 3; ++j) sq += std::pow(s[static_cast<size_t>(du[static_cast<size_t>(k)][j])], 2);
        largest = std::max(largest, std::sqrt(sq));
    }
    return largest;
}

}  // namespace starsim::gnc
