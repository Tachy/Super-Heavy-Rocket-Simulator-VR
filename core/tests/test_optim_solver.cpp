#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <vector>

#include "socp_test_support.hpp"
#include "starsim/optim/socp_builder.hpp"
#include "starsim/optim/socp_solver.hpp"

using namespace starsim::optim;
using Catch::Matchers::WithinAbs;
using starsim::test::constraint_violation;

namespace {

void require_optimal(const SocpProblem& p, const SocpResult& r) {
    INFO("status " << to_string(r.status) << ", iterations " << r.iterations);
    REQUIRE(r.status == SocpStatus::Optimal);
    CHECK(constraint_violation(p, r.x) <= 1e-7);
}

// max x0 + x1  s.t.  x0 + 2 x1 <= 4, 3 x0 + x1 <= 6, x >= 0  ->  vertex (1.6, 1.2).
SocpProblem vertex_lp(double row_scale_1, double row_scale_2, double x0_unit) {
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");  // x[0] holds x0 / x0_unit
    b.set_objective(AffineExpr{}.add(x[0], -x0_unit).add(x[1], -1.0));
    b.add_nonnegative(AffineExpr{4.0 * row_scale_1}.add(x[0], -x0_unit * row_scale_1).add(x[1], -2.0 * row_scale_1));
    b.add_nonnegative(AffineExpr{6.0 * row_scale_2}.add(x[0], -3.0 * x0_unit * row_scale_2).add(x[1], -row_scale_2));
    b.add_nonnegative(AffineExpr{}.add(x[0], 1.0));
    b.add_nonnegative(AffineExpr{}.add(x[1], 1.0));
    return b.build();
}

// min ||x|| (as t) s.t. x0 + 2 x1 + 2 x2 = 9  ->  x = (1, 2, 2), t = 3. Optionally with unused variables.
SocpProblem plane_distance(bool unused_block) {
    SocpBuilder b;
    const auto x = b.add_variables(3, "x");
    const auto t = b.add_variables(1, "t");
    if (unused_block) b.add_variables(2, "unused");
    b.set_objective(AffineExpr{}.add(t[0], 1.0));
    b.add_second_order_cone({AffineExpr{}.add(t[0], 1.0), AffineExpr{}.add(x[0], 1.0), AffineExpr{}.add(x[1], 1.0),
                             AffineExpr{}.add(x[2], 1.0)});
    b.add_equality(AffineExpr{-9.0}.add(x[0], 1.0).add(x[1], 2.0).add(x[2], 2.0));
    return b.build();
}

// min ||x|| s.t. ||x - (3, 0)|| <= 1, x1 >= 0.5  ->  x = (3 - sqrt(0.75), 0.5).
SocpProblem ball_and_halfspace() {
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    const auto t = b.add_variables(1, "t");
    b.set_objective(AffineExpr{}.add(t[0], 1.0));
    b.add_second_order_cone({AffineExpr{}.add(t[0], 1.0), AffineExpr{}.add(x[0], 1.0), AffineExpr{}.add(x[1], 1.0)});
    b.add_second_order_cone({AffineExpr{1.0}, AffineExpr{-3.0}.add(x[0], 1.0), AffineExpr{}.add(x[1], 1.0)});
    b.add_nonnegative(AffineExpr{-0.5}.add(x[1], 1.0));
    return b.build();
}

}  // namespace

TEST_CASE("SOCP solver finds the vertex of an LP", "[optim]") {
    const auto p = vertex_lp(1.0, 1.0, 1.0);
    const auto r = solve_socp(p);
    require_optimal(p, r);
    CHECK_THAT(r.x[0], WithinAbs(1.6, 1e-6));
    CHECK_THAT(r.x[1], WithinAbs(1.2, 1e-6));
    CHECK_THAT(r.objective, WithinAbs(-2.8, 1e-6));
}

TEST_CASE("SOCP solver handles badly scaled data", "[optim]") {
    // Rows scaled by 1e6 and 1e-3, x0 measured in units of 1e3.
    const auto p = vertex_lp(1e6, 1e-3, 1e3);
    const auto r = solve_socp(p);
    REQUIRE(r.status == SocpStatus::Optimal);
    CHECK_THAT(r.x[0], WithinAbs(1.6e-3, 1e-9));
    CHECK_THAT(r.x[1], WithinAbs(1.2, 1e-6));
    CHECK_THAT(r.objective, WithinAbs(-2.8, 1e-6));
}

TEST_CASE("SOCP solver: minimum distance to a plane", "[optim]") {
    const auto p = plane_distance(false);
    const auto r = solve_socp(p);
    require_optimal(p, r);
    CHECK_THAT(r.x[0], WithinAbs(1.0, 1e-6));
    CHECK_THAT(r.x[1], WithinAbs(2.0, 1e-6));
    CHECK_THAT(r.x[2], WithinAbs(2.0, 1e-6));
    CHECK_THAT(r.objective, WithinAbs(3.0, 1e-6));
}

TEST_CASE("SOCP solver tolerates variables in no constraint", "[optim]") {
    const auto p = plane_distance(true);
    const auto r = solve_socp(p);
    require_optimal(p, r);
    CHECK_THAT(r.objective, WithinAbs(3.0, 1e-6));
    CHECK_THAT(r.x[4], WithinAbs(0.0, 1e-6));
    CHECK_THAT(r.x[5], WithinAbs(0.0, 1e-6));
}

TEST_CASE("SOCP solver: minimum-norm point of a ball cut by a half-space", "[optim]") {
    const auto p = ball_and_halfspace();
    const auto r = solve_socp(p);
    require_optimal(p, r);
    const double x0 = 3.0 - std::sqrt(0.75);
    CHECK_THAT(r.x[0], WithinAbs(x0, 1e-6));
    CHECK_THAT(r.x[1], WithinAbs(0.5, 1e-6));
    CHECK_THAT(r.objective, WithinAbs(std::sqrt(x0 * x0 + 0.25), 1e-6));
}

TEST_CASE("SOCP solver detects primal infeasibility", "[optim]") {
    SocpBuilder b;
    const auto x = b.add_variables(1, "x");
    b.set_objective(AffineExpr{}.add(x[0], 1.0));
    b.add_nonnegative(AffineExpr{-1.0}.add(x[0], 1.0));  // x >= 1
    b.add_nonnegative(AffineExpr{}.add(x[0], -1.0));     // x <= 0
    CHECK(solve_socp(b.build()).status == SocpStatus::PrimalInfeasible);
}

TEST_CASE("SOCP solver detects an unbounded problem", "[optim]") {
    SocpBuilder b;
    const auto x = b.add_variables(1, "x");
    b.set_objective(AffineExpr{}.add(x[0], -1.0));  // min -x
    b.add_nonnegative(AffineExpr{}.add(x[0], 1.0));  // x >= 0
    CHECK(solve_socp(b.build()).status == SocpStatus::DualInfeasible);
}

TEST_CASE("SOCP solver handles a problem without inequalities", "[optim]") {
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    b.set_objective(AffineExpr{}.add(x[0], 1.0).add(x[1], 1.0));
    b.add_equality(AffineExpr{-2.0}.add(x[0], 1.0).add(x[1], 1.0));
    const auto p = b.build();
    const auto r = solve_socp(p);
    require_optimal(p, r);
    CHECK_THAT(r.objective, WithinAbs(2.0, 1e-8));
}

TEST_CASE("SOCP solver handles duplicated equality rows", "[optim]") {
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    b.set_objective(AffineExpr{}.add(x[0], 1.0).add(x[1], 2.0));
    b.add_equality(AffineExpr{-1.0}.add(x[0], 1.0).add(x[1], 1.0));
    b.add_equality(AffineExpr{-1.0}.add(x[0], 1.0).add(x[1], 1.0));
    b.add_nonnegative(AffineExpr{}.add(x[0], 1.0));
    b.add_nonnegative(AffineExpr{}.add(x[1], 1.0));
    const auto p = b.build();
    const auto r = solve_socp(p);
    require_optimal(p, r);
    CHECK_THAT(r.x[0], WithinAbs(1.0, 1e-6));
    CHECK_THAT(r.objective, WithinAbs(1.0, 1e-6));
}

TEST_CASE("SOCP solver handles cones of dimension 1 and 2", "[optim]") {
    SocpBuilder b;
    const auto t = b.add_variables(1, "t");
    const auto x = b.add_variables(1, "x");
    b.set_objective(AffineExpr{}.add(t[0], 1.0));
    b.add_second_order_cone({AffineExpr{-1.0}.add(t[0], 1.0)});                         // t >= 1
    b.add_second_order_cone({AffineExpr{}.add(t[0], 1.0), AffineExpr{}.add(x[0], 1.0)});  // t >= |x|
    b.add_equality(AffineExpr{3.0}.add(x[0], 1.0));                                       // x = -3
    const auto p = b.build();
    const auto r = solve_socp(p);
    require_optimal(p, r);
    CHECK_THAT(r.objective, WithinAbs(3.0, 1e-6));
}

TEST_CASE("SOCP solver is deterministic", "[optim]") {
    const auto p = ball_and_halfspace();
    const auto a = solve_socp(p);
    const auto b = solve_socp(p);
    CHECK(a.x == b.x);
    CHECK(a.iterations == b.iterations);
}
