#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/gnc/descent_problem.hpp"
#include "starsim/optim/socp_solver.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Quat;
using math::Vec3;

namespace {

constexpr int N = 6;

DescentModel model() {
    DescentModel m;
    m.engine_sets[0] = {3.0e7, 1.2e7, 1.0 / (330.0 * 9.80665), 0.26};
    m.engine_sets[1] = {1.2e7, 4.8e6, 1.0 / (330.0 * 9.80665), 0.26};
    m.engine_sets[2] = {7.0e6, 2.8e6, 1.0 / (330.0 * 9.80665), 0.26};
    m.gimbal_lever_m = 30.0;
    m.inertia_kg_m2 = {1.0e7, 2.0e8, 2.0e8};
    m.reference_area_m2 = 63.6;
    m.cd_by_mach = {1.0};
    m.density_kg_m3 = {1.0};
    m.speed_of_sound_mps = {340.0};
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

DescentReference straight_reference(double a_scale) {
    DescentReference ref;
    ref.duration_s = {6.0, 4.0, 8.0};
    const int nodes = descent_node_count(N);
    for (int k = 0; k < nodes; ++k) {
        const double f = static_cast<double>(k) / (nodes - 1);
        ref.x.push_back(make_state(4.0e5 - 5.0e4 * f, {600.0 * (1.0 - f) + 30.0, 10.0 * (1.0 - f), 0.0},
                                   {-100.0 * (1.0 - f), 0.0, 0.0}, Quat{1.0, 0.0, 0.0, 0.0}, {}));
        ref.u.push_back({a_scale * (k < N - 1 ? 2.0e7 : k < 2 * (N - 1) ? 8.0e6 : 5.0e6), 0.0, 0.0});
    }
    return ref;
}

std::vector<IntervalLinearization> linearize(const DescentModel& m, const DescentReference& ref) {
    std::vector<IntervalLinearization> lin;
    for (int k = 0; k < descent_node_count(N) - 1; ++k) {
        const int phase = k / (N - 1);
        lin.push_back(linearize_interval(m, phase, ref.x[static_cast<size_t>(k)], ref.u[static_cast<size_t>(k)],
                                         ref.u[static_cast<size_t>(k + 1)], ref.duration_s[static_cast<size_t>(phase)],
                                         1.0 / (N - 1), 2));
    }
    return lin;
}

// Limits for the synthetic reference (100 m/s at the start, ~67 m/s at the first switch): the flight switch windows
// (13 -> 5 at ~200 m/s) lie beyond its speeds.
DescentLimits limits() {
    DescentLimits l;
    l.switches[0].min_sink_mps = 50.0;
    l.switches[0].max_sink_mps = 90.0;
    return l;
}

DescentTarget catch_target() {
    DescentTarget t;
    t.goal = DescentGoal::Catch;
    t.position_m = {30.0, 0.0, 0.0};
    t.velocity_mps = {-1.0, 0.0, 0.0};
    return t;
}

}  // namespace

TEST_CASE("The descent subproblem keeps its structure across references", "[gnc][descent]") {
    const auto m = model();
    const auto limits = ::limits();
    const DescentWeights weights;
    const auto ref_a = straight_reference(1.0);
    const auto ref_b = straight_reference(0.9);
    const auto a = build_descent_subproblem(m, ref_a, linearize(m, ref_a), ref_a.x[0], 0, catch_target(), limits, weights, N);
    const auto b = build_descent_subproblem(m, ref_b, linearize(m, ref_b), ref_b.x[0], 0, catch_target(), limits, weights, N);
    CHECK(a.problem.A.col_start() == b.problem.A.col_start());
    CHECK(a.problem.A.row_index() == b.problem.A.row_index());
    CHECK(a.problem.G.col_start() == b.problem.G.col_start());
    CHECK(a.problem.G.row_index() == b.problem.G.row_index());
    optim::SocpSolver solver(a.problem);
    CHECK_NOTHROW(solver.solve(b.problem));
}

TEST_CASE("One SCvx subproblem is solvable and its solution unpacks", "[gnc][descent]") {
    const auto m = model();
    const auto ref = straight_reference(1.0);
    const auto sub = build_descent_subproblem(m, ref, linearize(m, ref), ref.x[0], 0, catch_target(), limits(),
                                              DescentWeights{}, N);
    const auto result = optim::solve_socp(sub.problem);
    REQUIRE(result.status == optim::SocpStatus::Optimal);
    const auto next = sub.unpack(result.x);
    REQUIRE(next.x.size() == ref.x.size());
    CHECK(std::abs(next.x.back()[descent_index::position] - 30.0) < 1e-6);  // terminal height (equality)
    CHECK(next.duration_s[0] >= 1.0 - 1e-6);  // interior-point accuracy
    CHECK(sub.max_step(result.x) >= 0.0);
}

TEST_CASE("Starting in a later phase fixes the earlier phases to zero length", "[gnc][descent]") {
    const auto m = model();
    for (int start_phase = 1; start_phase < descent_phases; ++start_phase) {
        CAPTURE(start_phase);
        auto ref = straight_reference(1.0);
        for (int p = 0; p < start_phase; ++p) ref.duration_s[static_cast<size_t>(p)] = 0.0;
        for (int k = 1; k <= start_phase * (N - 1); ++k) ref.x[static_cast<size_t>(k)] = ref.x[0];
        const auto sub = build_descent_subproblem(m, ref, linearize(m, ref), ref.x[0], start_phase, catch_target(),
                                                  limits(), DescentWeights{}, N);
        const auto result = optim::solve_socp(sub.problem);
        REQUIRE(result.status == optim::SocpStatus::Optimal);
        const auto next = sub.unpack(result.x);
        for (int p = 0; p < descent_phases; ++p) {
            if (p < start_phase) CHECK(next.duration_s[static_cast<size_t>(p)] == 0.0);
            else CHECK(next.duration_s[static_cast<size_t>(p)] > 0.0);
        }
    }
}

TEST_CASE("Node times and phases of a three-phase plan", "[gnc][descent]") {
    const PhaseDurations d{6.0, 4.0, 8.0};
    CHECK(descent_node_count(N) == 16);
    CHECK(reference_node_time(d, 0, N) == 0.0);
    CHECK(reference_node_time(d, N - 1, N) == 6.0);
    CHECK(reference_node_time(d, 2 * (N - 1), N) == 10.0);
    CHECK(reference_node_time(d, 15, N) == 18.0);
    CHECK(descent_node_phase(N - 2, N) == 0);
    CHECK(descent_node_phase(N - 1, N) == 1);
    CHECK(descent_node_phase(2 * (N - 1), N) == 2);
    CHECK(descent_node_phase(15, N) == 2);
}
