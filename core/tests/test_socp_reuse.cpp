#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "starsim/optim/socp_builder.hpp"
#include "starsim/optim/socp_solver.hpp"

using namespace starsim::optim;

namespace {

// min ||x - p|| over the half-space x0 + x1 >= 1 (p as a parameter of h only).
SocpProblem projection(double px, double py) {
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    const auto t = b.add_variables(1, "t");
    b.set_objective(AffineExpr{}.add(t[0], 1.0));
    b.add_second_order_cone({AffineExpr{}.add(t[0], 1.0), AffineExpr{-px}.add(x[0], 1.0), AffineExpr{-py}.add(x[1], 1.0)});
    b.add_nonnegative(AffineExpr{-1.0}.add(x[0], 1.0).add(x[1], 1.0));
    return b.build();
}

}  // namespace

TEST_CASE("A reusable solver gives the same results as solve_socp", "[optim]") {
    SocpSolver solver(projection(0.0, 0.0));
    for (const auto& p : {std::pair{0.0, 0.0}, std::pair{-2.0, 0.5}, std::pair{3.0, 3.0}}) {
        const auto problem = projection(p.first, p.second);
        const auto a = solver.solve(problem);
        const auto b = solve_socp(problem);
        REQUIRE(a.status == SocpStatus::Optimal);
        CHECK(a.x == b.x);
        CHECK(a.iterations == b.iterations);
    }
}

TEST_CASE("A reusable solver rejects a different structure", "[optim]") {
    SocpSolver solver(projection(0.0, 0.0));
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    b.set_objective(AffineExpr{}.add(x[0], 1.0));
    b.add_nonnegative(AffineExpr{}.add(x[0], 1.0));
    CHECK_THROWS_AS(solver.solve(b.build()), std::invalid_argument);
}

TEST_CASE("A reusable solver recovers after a failed solve", "[optim]") {
    auto problem = projection(0.0, 0.0);
    SocpSolver solver(problem);
    auto broken = problem;
    broken.h[0] = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(solver.solve(broken), std::invalid_argument);  // validate rejects NaN data
    const auto r = solver.solve(problem);
    CHECK(r.status == SocpStatus::Optimal);
    CHECK(r.x == solve_socp(problem).x);
}

TEST_CASE("Variable blocks are bounds-checked", "[optim]") {
    SocpBuilder b;
    const auto x = b.add_variables(2, "x");
    CHECK(x[1] == 1);
    CHECK_THROWS_AS(x[2], std::out_of_range);
}
