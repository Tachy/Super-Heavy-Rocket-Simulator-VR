#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <random>
#include <vector>

#include "starsim/optim/ldl.hpp"
#include "starsim/optim/minimum_degree.hpp"

using namespace starsim::optim;
using Catch::Matchers::WithinAbs;

namespace {

// [[P, B^T], [B, -Q]] with diagonal P, Q > 0 and a random sparse coupling B (fixed seed).
CscMatrix random_quasi_definite(int n1, int n2, std::uint32_t seed, std::vector<int>& sign) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> value(-1.0, 1.0);
    std::uniform_int_distribution<int> column(0, n1 - 1);
    std::vector<Triplet> t;
    for (int i = 0; i < n1; ++i) t.push_back({i, i, 2.0 + std::abs(value(rng))});
    for (int i = 0; i < n2; ++i) t.push_back({n1 + i, n1 + i, -(1.0 + std::abs(value(rng)))});
    for (int i = 0; i < n2; ++i) {
        for (int k = 0; k < 3; ++k) {
            const int j = column(rng);
            const double v = value(rng);
            t.push_back({n1 + i, j, v});
            t.push_back({j, n1 + i, v});
        }
    }
    sign.assign(static_cast<std::size_t>(n1 + n2), -1);
    std::fill(sign.begin(), sign.begin() + n1, 1);
    return CscMatrix(n1 + n2, n1 + n2, t);
}

CscMatrix grid_laplacian(int k) {
    std::vector<Triplet> t;
    auto id = [k](int r, int c) { return r * k + c; };
    for (int r = 0; r < k; ++r) {
        for (int c = 0; c < k; ++c) {
            t.push_back({id(r, c), id(r, c), 4.0});
            if (r + 1 < k) {
                t.push_back({id(r, c), id(r + 1, c), -1.0});
                t.push_back({id(r + 1, c), id(r, c), -1.0});
            }
            if (c + 1 < k) {
                t.push_back({id(r, c), id(r, c + 1), -1.0});
                t.push_back({id(r, c + 1), id(r, c), -1.0});
            }
        }
    }
    return CscMatrix(k * k, k * k, t);
}

// Trajectory-like pattern: `stages` dense blocks of `block` variables, consecutive blocks coupled densely,
// node indices scrambled (fixed seed) so the natural ordering is bad.
CscMatrix scrambled_block_tridiagonal(int stages, int block, std::uint32_t seed) {
    const int n = stages * block;
    std::vector<int> label(static_cast<std::size_t>(n));
    std::iota(label.begin(), label.end(), 0);
    std::shuffle(label.begin(), label.end(), std::mt19937(seed));
    std::vector<Triplet> t;
    for (int s = 0; s < stages; ++s) {
        for (int a = 0; a < block; ++a) {
            for (int b = 0; b < block * (s + 1 < stages ? 2 : 1); ++b) {
                const int i = label[s * block + a];
                const int j = label[s * block + b];
                t.push_back({i, j, 1.0});
                t.push_back({j, i, 1.0});
            }
        }
    }
    return CscMatrix(n, n, t);
}

int fill(const CscMatrix& a, std::vector<int> perm) {
    SparseLdl ldl;
    ldl.analyze(a, std::move(perm));
    return ldl.factor_nonzeros();
}

std::vector<int> natural(int n) {
    std::vector<int> p(static_cast<std::size_t>(n));
    std::iota(p.begin(), p.end(), 0);
    return p;
}

}  // namespace

TEST_CASE("Sparse LDL solves a small quasi-definite system in any ordering", "[optim]") {
    const CscMatrix k(3, 3, {{0, 0, 4.0}, {0, 1, 1.0}, {1, 0, 1.0}, {0, 2, 2.0}, {2, 0, 2.0}, {1, 1, -3.0}, {2, 2, -5.0}});
    const std::vector<int> sign{1, -1, -1};
    for (const auto& perm : {std::vector<int>{0, 1, 2}, std::vector<int>{2, 1, 0}, std::vector<int>{1, 2, 0}}) {
        SparseLdl ldl;
        ldl.analyze(k, perm);
        REQUIRE(ldl.factor(k, sign));
        std::vector<double> x{12.0, -5.0, -13.0};  // K (1, 2, 3)
        ldl.solve(x);
        CHECK_THAT(x[0], WithinAbs(1.0, 1e-14));
        CHECK_THAT(x[1], WithinAbs(2.0, 1e-14));
        CHECK_THAT(x[2], WithinAbs(3.0, 1e-14));
    }
}

TEST_CASE("Sparse LDL solves random quasi-definite systems", "[optim]") {
    std::vector<int> sign;
    const CscMatrix k = random_quasi_definite(40, 25, 7u, sign);
    SparseLdl ldl;
    ldl.analyze(k, minimum_degree_ordering(k));
    REQUIRE(ldl.factor(k, sign));
    CHECK(ldl.regularized_pivots() == 0);
    const int n = k.cols();
    std::vector<double> x_true(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) x_true[i] = std::sin(i + 1.0);
    std::vector<double> b(static_cast<std::size_t>(n), 0.0);
    k.multiply_add(x_true, b);
    std::vector<double> x = b;
    ldl.solve(x);
    std::vector<double> r = b;
    k.multiply_add(x, r, -1.0);
    CHECK(norm2(r) <= 1e-12 * norm2(b));
    for (int i = 0; i < n; ++i) CHECK_THAT(x[i], WithinAbs(x_true[i], 1e-9));

    // Refactorisation with new values on the same pattern.
    CscMatrix k2 = k;
    for (auto& v : k2.values()) v *= 2.0;
    REQUIRE(ldl.factor(k2, sign));
    std::vector<double> x2 = b;
    ldl.solve(x2);
    for (int i = 0; i < n; ++i) CHECK_THAT(x2[i], WithinAbs(0.5 * x_true[i], 1e-9));
}

TEST_CASE("Sparse LDL regularises a pivot with the wrong sign", "[optim]") {
    const CscMatrix singular(2, 2, {{0, 0, 1.0}, {0, 1, 1.0}, {1, 0, 1.0}, {1, 1, 1.0}});
    const std::vector<int> sign{1, 1};
    SparseLdl ldl;
    ldl.analyze(singular, {0, 1});
    REQUIRE(ldl.factor(singular, sign));
    CHECK(ldl.regularized_pivots() == 1);
}

TEST_CASE("Minimum degree reduces the fill of the LDL factor", "[optim]") {
    const CscMatrix grid = grid_laplacian(12);
    CHECK(fill(grid, minimum_degree_ordering(grid)) < fill(grid, natural(grid.cols())));
    const CscMatrix band = scrambled_block_tridiagonal(30, 6, 11u);
    CHECK(fill(band, minimum_degree_ordering(band)) < fill(band, natural(band.cols())));
}
