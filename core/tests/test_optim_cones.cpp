#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <limits>
#include <vector>

#include "starsim/optim/cones.hpp"

using namespace starsim::optim;
using Catch::Matchers::WithinAbs;

namespace {

// l = 3, second-order cones of dimension 3, 5, 1 and 2 (dimensions 1 and 2 are degenerate but legal).
ConeLayout test_layout() {
    ConeLayout layout;
    layout.nonnegative = 3;
    layout.second_order = {3, 5, 1, 2};
    return layout;
}

// A deterministic interior point: orthant entries > 0, each cone u0 > ||u1||.
std::vector<double> interior_point(const ConeLayout& layout, double phase) {
    std::vector<double> u(static_cast<std::size_t>(layout.size()));
    int i = 0;
    for (; i < layout.nonnegative; ++i) u[i] = 1.0 + 0.5 * std::sin(phase + i);
    for (const int q : layout.second_order) {
        double norm = 0.0;
        for (int k = 1; k < q; ++k) {
            u[i + k] = std::cos(phase * (k + 1) + i);
            norm += u[i + k] * u[i + k];
        }
        u[i] = std::sqrt(norm) + 0.3 + 0.1 * std::sin(phase + q);
        i += q;
    }
    return u;
}

}  // namespace

TEST_CASE("Cone layout size and degree", "[optim]") {
    const auto layout = test_layout();
    CHECK(layout.size() == 3 + 3 + 5 + 1 + 2);
    CHECK(layout.degree() == 3 + 4);
}

TEST_CASE("Cone division inverts the Jordan product", "[optim]") {
    const auto layout = test_layout();
    const auto lambda = interior_point(layout, 0.4);
    const auto v = interior_point(layout, 1.7);
    std::vector<double> x(v.size()), back(v.size());
    cone_division(layout, lambda, v, x);
    cone_product(layout, lambda, x, back);
    for (std::size_t i = 0; i < v.size(); ++i) CHECK_THAT(back[i], WithinAbs(v[i], 1e-12));
}

TEST_CASE("Identity element and eigenvalues", "[optim]") {
    ConeLayout layout;
    layout.nonnegative = 1;
    layout.second_order = {3};
    std::vector<double> u{-0.5, 1.0, 0.6, 0.8};
    CHECK_THAT(min_eigenvalue(layout, u), WithinAbs(-0.5, 1e-15));
    u[0] = 2.0;
    CHECK_THAT(min_eigenvalue(layout, u), WithinAbs(0.0, 1e-15));  // cone part on the boundary
    add_identity(layout, 1.0, u);
    CHECK(u == std::vector<double>{3.0, 2.0, 0.6, 0.8});
    CHECK(min_eigenvalue(ConeLayout{}, std::vector<double>{}) == std::numeric_limits<double>::infinity());
}

TEST_CASE("Max step reaches the cone boundary", "[optim]") {
    ConeLayout soc;
    soc.second_order = {3};
    const std::vector<double> u{1.0, 0.0, 0.0};
    CHECK_THAT(max_step(soc, u, std::vector<double>{0.0, 1.0, 0.0}), WithinAbs(1.0, 1e-14));
    CHECK(max_step(soc, u, std::vector<double>{1.0, 0.0, 0.0}) == std::numeric_limits<double>::infinity());
    ConeLayout pair;
    pair.second_order = {2};
    CHECK_THAT(max_step(pair, std::vector<double>{2.0, 0.0}, std::vector<double>{-1.0, 0.0}), WithinAbs(2.0, 1e-14));
    ConeLayout orthant;
    orthant.nonnegative = 2;
    CHECK_THAT(max_step(orthant, std::vector<double>{1.0, 2.0}, std::vector<double>{-0.5, -4.0}), WithinAbs(0.5, 1e-15));

    // General direction: just inside before the step, outside after it.
    const auto layout = test_layout();
    const auto p = interior_point(layout, 0.9);
    const auto d = interior_point(layout, 2.3);
    std::vector<double> dir(d.size());
    for (std::size_t i = 0; i < d.size(); ++i) dir[i] = -d[i] + 0.3 * std::sin(3.0 * i);
    const double alpha = max_step(layout, p, dir);
    REQUIRE(std::isfinite(alpha));
    std::vector<double> inside(p.size()), outside(p.size());
    for (std::size_t i = 0; i < p.size(); ++i) {
        inside[i] = p[i] + 0.999 * alpha * dir[i];
        outside[i] = p[i] + 1.001 * alpha * dir[i];
    }
    CHECK(min_eigenvalue(layout, inside) > 0.0);
    CHECK(min_eigenvalue(layout, outside) < 0.0);
}

TEST_CASE("Nesterov-Todd scaling maps s and z to the same point", "[optim]") {
    const auto layout = test_layout();
    const auto s = interior_point(layout, 0.2);
    const auto z = interior_point(layout, 1.1);
    NtScaling scaling(layout);
    REQUIRE(scaling.update(s, z));
    std::vector<double> wz(s.size()), winv_s(s.size()), back(s.size());
    scaling.apply(z, wz);
    scaling.apply_inverse(s, winv_s);
    for (std::size_t i = 0; i < s.size(); ++i) {
        CHECK_THAT(wz[i], WithinAbs(winv_s[i], 1e-12));
        CHECK_THAT(scaling.lambda()[i], WithinAbs(wz[i], 1e-12));
    }
    scaling.apply_inverse(wz, back);  // W^{-1} W z = z
    for (std::size_t i = 0; i < s.size(); ++i) CHECK_THAT(back[i], WithinAbs(z[i], 1e-12));

    // W^T W blocks agree with applying W twice.
    std::vector<double> x(s.size()), wx(s.size()), wwx(s.size());
    for (std::size_t i = 0; i < x.size(); ++i) x[i] = std::cos(0.7 * i);
    scaling.apply(x, wx);
    scaling.apply(wx, wwx);
    for (int i = 0; i < layout.nonnegative; ++i) CHECK_THAT(scaling.orthant_squared(i) * x[i], WithinAbs(wwx[i], 1e-12));
    int offset = layout.nonnegative;
    for (int k = 0; k < static_cast<int>(layout.second_order.size()); ++k) {
        const int q = layout.second_order[k];
        const auto block = scaling.cone_squared(k);
        for (int r = 0; r < q; ++r) {
            double sum = 0.0;
            for (int c = 0; c < q; ++c) sum += block[r * q + c] * x[offset + c];
            CHECK_THAT(sum, WithinAbs(wwx[offset + r], 1e-12));
        }
        offset += q;
    }
}

TEST_CASE("Nesterov-Todd scaling rejects points outside the interior", "[optim]") {
    ConeLayout layout;
    layout.second_order = {3};
    NtScaling scaling(layout);
    CHECK_FALSE(scaling.update(std::vector<double>{1.0, 1.0, 0.0}, std::vector<double>{1.0, 0.0, 0.0}));
}
