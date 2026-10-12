#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <numeric>
#include <vector>

#include "starsim/optim/minimum_degree.hpp"

using namespace starsim::optim;

namespace {

bool is_permutation_of_n(std::vector<int> perm) {
    std::sort(perm.begin(), perm.end());
    for (int i = 0; i < static_cast<int>(perm.size()); ++i) {
        if (perm[i] != i) return false;
    }
    return true;
}

}  // namespace

TEST_CASE("Minimum degree eliminates the hub of an arrow matrix last", "[optim]") {
    std::vector<Triplet> t;
    const int n = 6;
    for (int i = 0; i < n; ++i) t.push_back({i, i, 1.0});
    for (int i = 1; i < n; ++i) {
        t.push_back({0, i, 1.0});
        t.push_back({i, 0, 1.0});
    }
    const auto perm = minimum_degree_ordering(CscMatrix(n, n, t));
    REQUIRE(static_cast<int>(perm.size()) == n);
    CHECK(is_permutation_of_n(perm));
    // The hub is among the last two pivots (with two nodes left both have degree 1).
    const auto hub_position = std::find(perm.begin(), perm.end(), 0) - perm.begin();
    CHECK(hub_position >= n - 2);
}

TEST_CASE("Minimum degree breaks ties by the lowest index", "[optim]") {
    // Path 0-1-2-3 (one triangle given only: the ordering symmetrises the pattern).
    const CscMatrix path(4, 4, {{0, 1, 1.0}, {1, 2, 1.0}, {2, 3, 1.0}});
    CHECK(minimum_degree_ordering(path) == std::vector<int>{0, 1, 2, 3});
}

TEST_CASE("Minimum degree handles an empty and a diagonal matrix", "[optim]") {
    CHECK(minimum_degree_ordering(CscMatrix(0, 0)).empty());
    const CscMatrix diagonal(3, 3, {{0, 0, 1.0}, {1, 1, 1.0}, {2, 2, 1.0}});
    CHECK(minimum_degree_ordering(diagonal) == std::vector<int>{0, 1, 2});
}
