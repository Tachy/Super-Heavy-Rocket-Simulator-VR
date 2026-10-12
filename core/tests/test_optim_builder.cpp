#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <string>
#include <vector>

#include "starsim/optim/socp_builder.hpp"

using namespace starsim::optim;

TEST_CASE("SocpBuilder assembles the standard form in cone order", "[optim]") {
    SocpBuilder builder;
    const auto x = builder.add_variables(2, "x");
    const auto t = builder.add_variables(1, "t");
    CHECK(x[1] == 1);
    CHECK(t[0] == 2);
    CHECK(builder.variable_count() == 3);
    builder.set_objective(AffineExpr{4.0}.add(t[0], 1.0));
    // t >= ||(x0 - 1, 2 x1)||, added before the orthant row on purpose.
    builder.add_second_order_cone({AffineExpr{}.add(t[0], 1.0), AffineExpr{-1.0}.add(x[0], 1.0), AffineExpr{}.add(x[1], 2.0)});
    builder.add_nonnegative(AffineExpr{3.0}.add(x[0], -1.0));               // 3 - x0 >= 0
    builder.add_equality(AffineExpr{-1.0}.add(x[0], 1.0).add(x[1], 1.0));   // x0 + x1 - 1 = 0
    const SocpProblem p = builder.build();
    CHECK_NOTHROW(p.validate());
    CHECK(builder.objective_constant() == 4.0);
    CHECK(p.variables == 3);
    CHECK(p.c == std::vector<double>{0.0, 0.0, 1.0});
    CHECK(p.cones.nonnegative == 1);
    CHECK(p.cones.second_order == std::vector<int>{3});
    // A x = b
    CHECK(p.A.rows() == 1);
    CHECK(p.A.at(0, 0) == 1.0);
    CHECK(p.A.at(0, 1) == 1.0);
    CHECK(p.b == std::vector<double>{1.0});
    // G x + s = h with s = expression: row 0 the orthant row, rows 1-3 the cone.
    CHECK(p.G.rows() == 4);
    CHECK(p.G.at(0, 0) == 1.0);
    CHECK(p.G.at(1, 2) == -1.0);
    CHECK(p.G.at(2, 0) == -1.0);
    CHECK(p.G.at(3, 1) == -2.0);
    CHECK(p.h == std::vector<double>{3.0, 0.0, -1.0, 0.0});
}

TEST_CASE("SocpBuilder rejects unknown variables and empty cones", "[optim]") {
    SocpBuilder builder;
    builder.add_variables(1, "x");
    CHECK_THROWS_AS(builder.add_nonnegative(AffineExpr{}.add(5, 1.0)), std::out_of_range);
    CHECK_THROWS_AS(builder.add_second_order_cone({}), std::invalid_argument);
}

TEST_CASE("SocpProblem validation catches inconsistent sizes", "[optim]") {
    SocpProblem p;
    p.variables = 2;
    p.c = {1.0};
    p.A = CscMatrix(0, 2);
    p.G = CscMatrix(0, 2);
    CHECK_THROWS_AS(p.validate(), std::invalid_argument);
}

TEST_CASE("Status names", "[optim]") {
    CHECK(std::string(to_string(SocpStatus::Optimal)) == "optimal");
    CHECK(std::string(to_string(SocpStatus::PrimalInfeasible)) == "primal infeasible");
}
