#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

#include "starsim/optim/equilibration.hpp"

using namespace starsim::optim;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

SocpProblem badly_scaled() {
    SocpProblem p;
    p.variables = 3;
    p.c = {1.0, 2.0, 3.0};
    p.A = CscMatrix(1, 3, {{0, 0, 1e6}, {0, 2, 2e6}});
    p.b = {5e6};
    p.G = CscMatrix(4, 3, {{0, 0, 1e-3}, {1, 0, 2.0}, {1, 1, 1.0}, {2, 1, 1e3}, {3, 2, 1.0}});
    p.h = {1e-3, 1.0, 2.0, 3.0};
    p.cones.nonnegative = 1;
    p.cones.second_order = {3};
    return p;
}

double row_norm(const CscMatrix& m, int row) {
    double r = 0.0;
    for (int j = 0; j < m.cols(); ++j) r = std::max(r, std::abs(m.at(row, j)));
    return r;
}

double column_norm(const SocpProblem& p, int col) {
    double r = 0.0;
    for (int i = 0; i < p.A.rows(); ++i) r = std::max(r, std::abs(p.A.at(i, col)));
    for (int i = 0; i < p.G.rows(); ++i) r = std::max(r, std::abs(p.G.at(i, col)));
    return r;
}

}  // namespace

TEST_CASE("Ruiz equilibration balances rows and columns and keeps cones", "[optim]") {
    SocpProblem p = badly_scaled();
    const SocpProblem original = p;
    const Equilibration e = equilibrate(p, 10);

    CHECK(row_norm(p.A, 0) > 0.5);
    CHECK(row_norm(p.A, 0) < 2.0);
    CHECK(row_norm(p.G, 0) > 0.5);
    CHECK(row_norm(p.G, 0) < 2.0);
    const double cone_rows = std::max({row_norm(p.G, 1), row_norm(p.G, 2), row_norm(p.G, 3)});
    CHECK(cone_rows > 0.5);
    CHECK(cone_rows < 2.0);
    for (int j = 0; j < 3; ++j) {
        CHECK(column_norm(p, j) > 0.5);
        CHECK(column_norm(p, j) < 2.0);
    }
    // One factor per second-order cone.
    CHECK(e.cone_row[1] == e.cone_row[2]);
    CHECK(e.cone_row[2] == e.cone_row[3]);
    // Scaled data: G^ = E_G G D, h^ = E_G h, c^ = D c, b^ = E_A b.
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 3; ++j) {
            CHECK_THAT(p.G.at(i, j), WithinRel(e.cone_row[i] * original.G.at(i, j) * e.column[j], 1e-12));
        }
        CHECK_THAT(p.h[i], WithinRel(e.cone_row[i] * original.h[i], 1e-12));
    }
    for (int j = 0; j < 3; ++j) CHECK_THAT(p.c[j], WithinRel(e.column[j] * original.c[j], 1e-12));
    CHECK_THAT(p.b[0], WithinRel(e.equality_row[0] * original.b[0], 1e-12));
}

TEST_CASE("Unscaling maps scaled residuals to the original ones", "[optim]") {
    SocpProblem p = badly_scaled();
    const SocpProblem original = p;
    const Equilibration e = equilibrate(p, 10);
    std::vector<double> x{0.3, -0.2, 0.7}, y{0.4}, z{0.5, 1.0, 0.1, 0.2}, s{0.6, 2.0, 0.3, 0.1};
    // Scaled residual G^ x^ + s^ - h^ and dual residual A^T y^ + G^T z^ + c^.
    std::vector<double> primal_scaled(4), dual_scaled(3);
    for (int i = 0; i < 4; ++i) primal_scaled[i] = s[i] - p.h[i];
    p.G.multiply_add(x, primal_scaled);
    dual_scaled = p.c;
    p.A.multiply_transpose_add(y, dual_scaled);
    p.G.multiply_transpose_add(z, dual_scaled);

    unscale(e, x, y, z, s);
    std::vector<double> primal(4), dual(3);
    for (int i = 0; i < 4; ++i) primal[i] = s[i] - original.h[i];
    original.G.multiply_add(x, primal);
    dual = original.c;
    original.A.multiply_transpose_add(y, dual);
    original.G.multiply_transpose_add(z, dual);
    for (int i = 0; i < 4; ++i) CHECK_THAT(primal[i], WithinAbs(primal_scaled[i] / e.cone_row[i], 1e-9));
    for (int j = 0; j < 3; ++j) CHECK_THAT(dual[j], WithinAbs(dual_scaled[j] / e.column[j], 1e-9));
}

TEST_CASE("Equilibration leaves an empty column alone", "[optim]") {
    SocpProblem p;
    p.variables = 2;
    p.c = {1.0, 0.0};
    p.A = CscMatrix(0, 2);
    p.G = CscMatrix(1, 2, {{0, 0, 4.0}});
    p.h = {1.0};
    p.cones.nonnegative = 1;
    const Equilibration e = equilibrate(p, 10);
    CHECK(e.column[1] == 1.0);
}
