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
constexpr double corridor_inactive_radius_m = 1.0e3;  // nodes above the corridor: idle (a landing burn diverts < 1 km)

}  // namespace

std::array<double, 14> descent_state_scales() {
    std::array<double, 14> s{};
    s[0] = mass_scale_kg;
    for (size_t i = 1; i < 4; ++i) s[i] = position_scale_m;
    for (size_t i = 4; i < 7; ++i) s[i] = velocity_scale_mps;
    for (size_t i = 7; i < 11; ++i) s[i] = attitude_scale;
    for (size_t i = 11; i < 14; ++i) s[i] = rate_scale_radps;
    return s;
}

namespace {

AffineExpr scaled(AffineExpr e, double factor) {
    for (auto& term : e.terms) term.second *= factor;
    e.constant *= factor;
    return e;
}

}  // namespace

double reference_node_time(const PhaseDurations& duration, int k, int n) {
    const int p = descent_node_phase(k, n);
    double t = 0.0;
    for (int q = 0; q < p; ++q) t += duration[static_cast<size_t>(q)];
    return t + duration[static_cast<size_t>(p)] * (k - p * (n - 1)) / (n - 1);
}

DescentSubproblem build_descent_subproblem(const DescentModel& model, const DescentReference& ref,
                                           const std::vector<IntervalLinearization>& lin, const DescentState& initial,
                                           int start_phase, const DescentTarget& target, const DescentLimits& limits,
                                           const DescentWeights& weights, int n) {
    DescentSubproblem sub;
    sub.reference = ref;
    sub.start_phase = start_phase;
    sub.nodes = descent_node_count(n);
    sub.state_scale = descent_state_scales();
    sub.thrust_scale = thrust_scale_fraction * model.engine_sets[0].thrust_max_n;
    const auto& S = sub.state_scale;
    const double su = sub.thrust_scale;
    const int K = sub.nodes;
    const int final_phase = descent_phases - 1;
    const int fixed_nodes_end = start_phase * (n - 1);  // nodes 0 .. this: the measured start state
    const auto at = [](int k) { return static_cast<size_t>(k); };

    optim::SocpBuilder b;
    for (int k = 0; k < K; ++k) {
        sub.dx.push_back(b.add_variables(14, "dx"));
        sub.du.push_back(b.add_variables(3, "du"));
    }
    sub.ds = b.add_variables(descent_phases, "ds");
    std::vector<optim::VariableBlock> abs_nu;
    for (int k = 0; k < K - 1; ++k) {
        sub.nu.push_back(b.add_variables(14, "nu"));
        abs_nu.push_back(b.add_variables(14, "|nu|"));
    }
    const auto eta = b.add_variables(K, "eta");
    const auto eta_s = b.add_variables(1, "eta_s");
    // Penalised auxiliary variables (soft constraints, epigraphs), created only where their constraint exists: a
    // free zero-cost variable would leave the interior-point solve ill-conditioned.
    std::vector<std::pair<int, double>> penalties;  // (variable, objective weight)
    auto penalised = [&](double weight, const char* name) {
        const int v = b.add_variables(1, name)[0];
        b.add_nonnegative(AffineExpr{}.add(v, 1.0));
        penalties.emplace_back(v, weight);
        return v;
    };

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
        const int p = k / (n - 1);
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
    if (target.goal != DescentGoal::Abort) {
        // Position, velocity, upright (quaternion y = z = 0, any roll), no pitch/yaw rotation (the thrust has no roll
        // authority: a measured roll rate stays in the model, the tracking controller handles roll).
        const double t[6] = {target.position_m.x,   target.position_m.y,   target.position_m.z,
                             target.velocity_mps.x, target.velocity_mps.y, target.velocity_mps.z};
        for (int i = 0; i < 6; ++i) b.add_equality(X(last, descent_index::position + i).add_constant(-t[i]));
        b.add_equality(X(last, descent_index::attitude + 2));
        b.add_equality(X(last, descent_index::attitude + 3));
        for (int i = descent_index::rate + 1; i < 14; ++i) b.add_equality(X(last, i));  // pitch/yaw; roll is free
    } else {
        b.add_equality(X(last, descent_index::position).add_constant(-target.ground_cg_height_m));
        for (int i = descent_index::velocity; i < descent_index::velocity + 3; ++i) b.add_equality(X(last, i));
        for (int i = descent_index::rate + 1; i < 14; ++i) b.add_equality(X(last, i));  // pitch/yaw; roll is free
        const int distance = penalised(weights.sea_distance, "sea_distance");
        b.add_second_order_cone({AffineExpr{}.add(distance, 1.0), X(last, 2).add_constant(-target.sea_target_m.y),
                                 X(last, 3).add_constant(-target.sea_target_m.z)});
        const double sea_norm = std::hypot(target.sea_target_m.y, target.sea_target_m.z);
        if (target.sea_min_distance_m > 0.0 && sea_norm > 0.0) {
            const double ny = target.sea_target_m.y / sea_norm, nz = target.sea_target_m.z / sea_norm;
            const auto& xl = ref.x[at(last)];
            b.add_nonnegative(AffineExpr{ny * xl[2] + nz * xl[3] - target.sea_min_distance_m}
                                  .add(sub.dx[at(last)][2], ny * S[2])
                                  .add(sub.dx[at(last)][3], nz * S[3]));
        }
    }
    // Path constraints.
    for (int k = 0; k < K; ++k) {
        // A shared node starts the next set: 13 engines at minimum throttle give more thrust than 5 at full, so the
        // thrust steps down over a phase's last interval (engines running down at the switch).
        const int p = descent_node_phase(k, n);
        const auto& set = model.engine_sets[at(p)];
        double spool = 1.0;
        if (p == 0 && limits.phase0_startup_s > 0.0) {
            spool = std::min(1.0, limits.phase0_initial_spool +
                                      reference_node_time(ref.duration_s, k, n) / limits.phase0_startup_s);
        }
        // Tracking margin below the maximum (none above the minimum: the final set at minimum throttle barely exceeds
        // the weight, a margin there would forbid descending).
        const double t_max = (1.0 - limits.throttle_margin) * set.thrust_max_n * spool;
        // Final set: a short, firm segment (control authority grows with thrust; a slow float at minimum throttle lets
        // gimbal saturation push the booster sideways); only the last nodes may ease down for the gentle end.
        const bool firm = p == final_phase && last - k >= limits.final_ease_nodes;
        const double t_min = (firm ? std::max(set.thrust_min_n, limits.final_min_thrust_fraction * set.thrust_max_n)
                                   : set.thrust_min_n) * spool;
        const double fraction =
            p == final_phase && limits.final_gimbal_fraction > 0.0 ? limits.final_gimbal_fraction : limits.gimbal_fraction;
        const double gimbal = fraction * set.gimbal_max_rad;
        double tilt = limits.tilt_max_rad[static_cast<size_t>(p)];
        const double rate_max = limits.rate_max_radps[static_cast<size_t>(p)];
        if (p > start_phase && k == p * (n - 1)) {
            // Engine switch ahead, hard (the outgoing set can always settle the booster before it shuts down).
            const auto& sw = limits.switches[at(p - 1)];
            if (sw.max_tilt_rad > 0.0) {
                b.add_second_order_cone({AffineExpr{std::sqrt(0.5 * (1.0 - std::cos(sw.max_tilt_rad)))},
                                         X(k, descent_index::attitude + 2), X(k, descent_index::attitude + 3)});
            }
            if (sw.max_rate_radps > 0.0) {
                b.add_second_order_cone({AffineExpr{sw.max_rate_radps}, X(k, descent_index::rate + 1),
                                         X(k, descent_index::rate + 2)});
            }
            b.add_nonnegative(scaled(X(k, descent_index::velocity), -1.0).add_constant(-sw.min_sink_mps));
            b.add_nonnegative(X(k, descent_index::velocity).add_constant(sw.max_sink_mps));
            if (sw.min_cg_height_m > 0.0) {  // e.g. the Raptors off well above the ground (plume-surface interaction)
                b.add_nonnegative(X(k, descent_index::position).add_constant(-sw.min_cg_height_m));
            }
        }
        if (const int from_end = last - k; p == final_phase && from_end < limits.tilt_taper_nodes) {
            tilt = limits.terminal_tilt_rad + (tilt - limits.terminal_tilt_rad) * from_end / limits.tilt_taper_nodes;
        }
        if (target.goal == DescentGoal::Catch && target.corridor_radius_m > 0.0 && limits.corridor_tilt_rad > 0.0 &&
            ref.x[at(k)][descent_index::position] < target.corridor_top_m) {
            tilt = std::min(tilt, limits.corridor_tilt_rad);  // the hull passes the arms nearly upright
        }
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
        // The measured start state (and, starting in a later phase, its zero-duration copies up to that phase's first
        // node) is fixed: state limits there could only make a re-plan from a slightly violating snapshot infeasible.
        if (k > fixed_nodes_end) {
        // Tilt and rate limits are soft (exact L1 penalty): right after the engine switch the booster may still turn
        // faster than the final set allows, and a re-plan must stay feasible while that dies out.
        const int tilt_slack = penalised(weights.attitude_violation, "tilt_slack");  // quaternion (y, z) norm over
        const int rate_slack = penalised(weights.attitude_violation, "rate_slack");  // rad/s over
        b.add_second_order_cone({AffineExpr{std::sqrt(0.5 * (1.0 - std::cos(tilt)))}.add(tilt_slack, 1.0),
                                 X(k, descent_index::attitude + 2),
                                 X(k, descent_index::attitude + 3)});
        // Pitch/yaw rate only: the model has no roll authority, a measured roll rate just persists.
        b.add_second_order_cone({AffineExpr{rate_max}.add(rate_slack, 1.0),
                                 X(k, descent_index::rate + 1),
                                 X(k, descent_index::rate + 2)});
        b.add_nonnegative(X(k, descent_index::mass).add_constant(-limits.min_mass_kg));
        if (target.goal != DescentGoal::Abort) {
            const double tg = std::tan(limits.glide_slope_rad);
            b.add_second_order_cone({X(k, 1).add_constant(-target.position_m.x),
                                     scaled(X(k, 2).add_constant(-target.position_m.y), tg),
                                     scaled(X(k, 3).add_constant(-target.position_m.z), tg)});
            if (limits.approach_time_constant_s > 0.0) {
                // v_x + sink_end + (x - x_catch) / tau >= -slack (soft: a re-plan from a fast state stays feasible).
                const int approach_slack = penalised(weights.corridor_violation, "approach_slack");
                AffineExpr approach = X(k, descent_index::velocity).add_constant(-target.velocity_mps.x).add(approach_slack, 1.0);
                const AffineExpr height =
                    scaled(X(k, descent_index::position).add_constant(-target.position_m.x), 1.0 / limits.approach_time_constant_s);
                approach.add_constant(height.constant);
                for (const auto& [index, c] : height.terms) approach.add(index, c);
                b.add_nonnegative(approach);
            }
            // Corridor: same structure at every node (the reference height only selects the radius).
            const bool in_corridor = target.corridor_radius_m > 0.0 && ref.x[at(k)][descent_index::position] < target.corridor_top_m;
            const double radius = in_corridor                                  ? target.corridor_radius_m
                                  : p == final_phase && target.final_set_radius_m > 0.0 ? target.final_set_radius_m
                                                                              : corridor_inactive_radius_m;
            // Soft: a re-plan from outside the corridor stays feasible (exact L1 penalty, zero slack when reachable).
            const int corridor_slack = penalised(weights.corridor_violation, "corridor_slack");
            b.add_second_order_cone({AffineExpr{radius}.add(corridor_slack, 1.0),
                                     X(k, 2).add_constant(-target.position_m.y), X(k, 3).add_constant(-target.position_m.z)});
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
            b.add_nonnegative(X(k, descent_index::position).add_constant(-target.ground_cg_height_m));  // above ground
            if (target.mount_keep_out_radius_m > 0.0) {
                // Launch mount: n . r >= R near the ground (n from the pad centre to the reference); far above it the
                // row stays with a radius that never binds, so the structure is the same at every node.
                const Vec3 rr = state_position(ref.x[at(k)]);
                const double hr = std::hypot(rr.y, rr.z);
                const double my = hr > 1e-6 ? rr.y / hr : 1.0;
                const double mz = hr > 1e-6 ? rr.z / hr : 0.0;
                const bool near_ground = rr.x < target.mount_top_cg_height_m;
                AffineExpr mount{near_ground ? -target.mount_keep_out_radius_m : corridor_inactive_radius_m};
                const AffineExpr my_x = X(k, 2), mz_x = X(k, 3);
                for (const auto& [index, c] : my_x.terms) mount.add(index, my * c);
                for (const auto& [index, c] : mz_x.terms) mount.add(index, mz * c);
                mount.add_constant(my * my_x.constant + mz * mz_x.constant);
                b.add_nonnegative(mount);
            }
        }
        }
        // Trust region: eta_k >= ||(dx_k, du_k)||^2 as a rotated cone ||(2 v, eta - 1)|| <= eta + 1.
        std::vector<AffineExpr> cone{AffineExpr{1.0}.add(eta[k], 1.0)};
        for (int i = 0; i < 14; ++i) cone.push_back(AffineExpr{}.add(sub.dx[at(k)][i], 2.0));
        for (int j = 0; j < 3; ++j) cone.push_back(AffineExpr{}.add(sub.du[at(k)][j], 2.0));
        cone.push_back(AffineExpr{-1.0}.add(eta[k], 1.0));
        b.add_second_order_cone(cone);
    }
    {
        std::vector<AffineExpr> cone{AffineExpr{1.0}.add(eta_s[0], 1.0)};
        for (int p = 0; p < descent_phases; ++p) cone.push_back(AffineExpr{}.add(sub.ds[p], 2.0));
        cone.push_back(AffineExpr{-1.0}.add(eta_s[0], 1.0));
        b.add_second_order_cone(cone);
    }
    // Phase durations.
    for (int p = 0; p < descent_phases; ++p) {
        const AffineExpr s = AffineExpr{ref.duration_s[static_cast<size_t>(p)]}.add(sub.ds[p], duration_scale_s);
        if (p < start_phase) {
            b.add_equality(s);
            continue;
        }
        b.add_nonnegative(AffineExpr{s}.add_constant(-limits.min_phase_s[static_cast<size_t>(p)]));
        b.add_nonnegative(AffineExpr{limits.max_phase_s - ref.duration_s[static_cast<size_t>(p)]}.add(sub.ds[p], -duration_scale_s));
    }
    // Gentle end (see DescentLimits::terminal_thrust_margin): a cone on the last node's thrust.
    {
        const auto& set = model.engine_sets[at(final_phase)];
        b.add_second_order_cone({AffineExpr{set.thrust_min_n + limits.terminal_thrust_margin * set.thrust_max_n},
                                 U(last, 0), U(last, 1), U(last, 2)});
    }
    // Throttle rate: the thrust magnitude (linearised along the reference direction) changes at most the engines'
    // throttle rate over each interval (reference interval lengths). Not over an engine switch, where the outgoing
    // set runs down, nor over the zero-length intervals before the start phase.
    auto magnitude = [&](int k) {
        const Vec3& ub = ref.u[at(k)];
        const Vec3 dir = norm(ub) >= min_reference_thrust_n ? ub / norm(ub) : Vec3{1.0, 0.0, 0.0};
        const double d[3] = {dir.x, dir.y, dir.z};
        AffineExpr m;
        for (int j = 0; j < 3; ++j) {
            const AffineExpr uj = U(k, j);
            m.add_constant(d[j] * uj.constant);
            for (const auto& [index, c] : uj.terms) m.add(index, d[j] * c);
        }
        return m;
    };
    if (const auto& set = model.engine_sets[at(start_phase)]; limits.initial_thrust_n > 0.0 && set.throttle_rate_per_s > 0.0) {
        // |u| at the start phase's first node within the throttle-rate reach of the measured thrust.
        // Engines of an earlier set still running down count in the measurement: clamp it to this set's range.
        const double measured = std::clamp(limits.initial_thrust_n, set.thrust_min_n, (1.0 - limits.throttle_margin) * set.thrust_max_n);
        const double reach = set.throttle_rate_per_s * set.thrust_max_n * limits.initial_thrust_window_s;
        const AffineExpr m = magnitude(fixed_nodes_end);
        AffineExpr up{measured + reach - m.constant}, down{m.constant - measured + reach};
        for (const auto& [index, c] : m.terms) {
            up.add(index, -c);
            down.add(index, c);
        }
        b.add_nonnegative(up);
        b.add_nonnegative(down);
    }
    for (int k = 0; k + 1 < K; ++k) {
        if (((k + 1) % (n - 1) == 0 && k + 1 < last) || k < fixed_nodes_end) continue;
        const auto& set = model.engine_sets[at(k / (n - 1))];
        if (set.throttle_rate_per_s <= 0.0) continue;
        const double dt = reference_node_time(ref.duration_s, k + 1, n) - reference_node_time(ref.duration_s, k, n);
        const double max_change = set.throttle_rate_per_s * set.thrust_max_n * dt;
        AffineExpr change = magnitude(k + 1);
        const AffineExpr previous = magnitude(k);
        change.add_constant(-previous.constant);
        for (const auto& [index, c] : previous.terms) change.add(index, -c);
        AffineExpr up{max_change}, down{max_change};
        up.add_constant(-change.constant);
        down.add_constant(change.constant);
        for (const auto& [index, c] : change.terms) {
            up.add(index, -c);
            down.add(index, c);
        }
        b.add_nonnegative(up);
        b.add_nonnegative(down);
    }
    // Objective: maximum final mass, small virtual control, small steps (and close to the sea on an abort).
    AffineExpr objective;
    objective.add(sub.dx[at(last)][descent_index::mass], -weights.fuel * S[0] / mass_scale_kg);
    for (const auto& block : abs_nu) {
        for (int i = 0; i < 14; ++i) objective.add(block[i], weights.virtual_control);
    }
    for (int k = 0; k < K; ++k) objective.add(eta[k], weights.trust_region);
    objective.add(eta_s[0], weights.trust_region);
    // Catch: the final (3-engine) set turns the booster slowly - it should descend straight above the catch point
    // (the hull slides down between the arms), so the lateral divert happens on the earlier sets. A soft penalty on
    // the final set's distance from the catch axis keeps every re-plan feasible.
    for (int k = 0; k < K; ++k) {
        if (target.goal != DescentGoal::Abort && k >= final_phase * (n - 1) && k > 0) {
            const double tau = weights.final_lateral_time_s;  // lateral speed counts as the offset it makes in tau
            const int lateral = b.add_variables(1, "lateral")[0];
            b.add_second_order_cone({AffineExpr{}.add(lateral, 1.0), X(k, 2).add_constant(-target.position_m.y),
                                     X(k, 3).add_constant(-target.position_m.z), scaled(X(k, 5), tau),
                                     scaled(X(k, 6), tau)});
            penalties.emplace_back(lateral, weights.final_lateral);
        }
    }
    for (const auto& [variable, weight] : penalties) objective.add(variable, weight);
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
    for (int p = 0; p < descent_phases; ++p) {
        out.duration_s[at(p)] = p < start_phase ? 0.0 : out.duration_s[at(p)] + duration_scale_s * s[at(ds[p])];
    }
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
