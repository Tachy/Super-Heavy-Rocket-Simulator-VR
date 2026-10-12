#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <vector>

#include "starsim/optim/sparse_matrix.hpp"

using namespace starsim::optim;

TEST_CASE("CSC assembly sums duplicates and sorts rows", "[optim]") {
    const CscMatrix a(3, 2, {{2, 0, 1.0}, {0, 0, 2.0}, {2, 0, 0.5}, {1, 1, -3.0}});
    CHECK(a.nonzeros() == 3);
    CHECK(a.col_start() == std::vector<int>{0, 2, 3});
    CHECK(a.row_index() == std::vector<int>{0, 2, 1});
    CHECK(a.at(2, 0) == 1.5);
    CHECK(a.at(0, 0) == 2.0);
    CHECK(a.at(1, 1) == -3.0);
    CHECK(a.at(1, 0) == 0.0);
}

TEST_CASE("CSC keeps explicit zeros as structural entries", "[optim]") {
    const CscMatrix a(2, 2, {{0, 0, 0.0}, {1, 1, 1.0}});
    CHECK(a.nonzeros() == 2);
}

TEST_CASE("CSC products agree with dense computation", "[optim]") {
    const CscMatrix a(2, 3, {{0, 0, 1.0}, {0, 2, 2.0}, {1, 1, 3.0}, {1, 2, -1.0}});
    const std::vector<double> x{1.0, 2.0, 3.0};
    std::vector<double> y{10.0, 20.0};
    a.multiply_add(x, y, 2.0);  // A x = (7, 3)
    CHECK(y == std::vector<double>{24.0, 26.0});
    const std::vector<double> u{1.0, -1.0};
    std::vector<double> w(3, 0.0);
    a.multiply_transpose_add(u, w);  // A^T u = (1, -3, 3)
    CHECK(w == std::vector<double>{1.0, -3.0, 3.0});
    const CscMatrix t = a.transposed();
    CHECK(t.rows() == 3);
    CHECK(t.cols() == 2);
    CHECK(t.at(2, 0) == 2.0);
    CHECK(t.at(2, 1) == -1.0);
    CHECK(t.at(1, 1) == 3.0);
}

TEST_CASE("CSC rejects triplets outside the matrix", "[optim]") {
    CHECK_THROWS_AS(CscMatrix(2, 2, {{2, 0, 1.0}}), std::out_of_range);
}

TEST_CASE("Vector norms and dot product", "[optim]") {
    const std::vector<double> v{3.0, -4.0};
    CHECK(norm2(v) == 5.0);
    CHECK(norm_inf(v) == 4.0);
    CHECK(dot(v, v) == 25.0);
    CHECK(norm2(std::vector<double>{}) == 0.0);
}
