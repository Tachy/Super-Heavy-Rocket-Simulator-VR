# SOCP Solver (`optim`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A dependency-free primal-dual interior-point solver for second-order cone programs in the C++ core, validated against Clarabel.

**Architecture:** New core module `optim` (namespace `starsim::optim`) below `gnc`. Small units: CSC sparse matrix -> minimum-degree ordering -> sparse LDL^T -> cone algebra / Nesterov-Todd scaling -> problem types + builder -> Ruiz equilibration -> KKT system -> homogeneous self-dual embedding (HSDE) with Mehrotra predictor-corrector. Reference problems are generated offline by a Python script (CVXPY + Clarabel) and committed as text fixtures.

**Tech Stack:** C++20 (MSVC 2022, CMake, Ninja), Catch2 v3.8.1, Python 3.13 with numpy/scipy/cvxpy/clarabel (offline fixture generation only).

**Spec:** `docs/superpowers/specs/2026-10-12-socp-solver-design.md`

## Global Constraints

- `optim` depends only on the C++ standard library; nothing outside `optim` changes except CMake lists, tests, `CLAUDE.md`.
- SI/physics conventions do not apply inside `optim` (pure numerics), but no magic numbers without a named constant and a comment.
- Code, identifiers, comments, commit messages: English. Chat with the user: German.
- Single-threaded, no randomness in the solver, no allocation inside the iteration loop: identical input -> bit-identical output.
- Problem form: `min c^T x  s.t.  A x = b,  G x + s = h,  s in K = R+^l x Q^{q1} x ... x Q^{qm}`; rows of `G`/`h` ordered orthant first, then each second-order cone; `Q^q = { s : ||s[1..]|| <= s[0] }`.
- Defaults: `tolerance = 1e-8`, `max_iterations = 50`, `static_regularization = 7e-8`, dynamic regularisation `epsilon = 1e-13`, `delta = 2e-7`, `refinement_steps = 8`, `equilibration_passes = 10`, step fraction `0.99`.
- Fixture tolerances: objective relative error <= 1e-6; unique-optimum solutions `||x - x_ref|| / max(1, ||x_ref||) <= 1e-5`; the landing-size problem converges in < 30 iterations.
- Small files, one responsibility each. MSVC builds with `/W4`: cast `size()` to `int` explicitly (`static_cast<int>`).
- Commit messages end with the session's attribution lines (Co-Authored-By / Claude-Session) as given by the harness.

Build and run commands used by every task (PowerShell, repository root):
```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests
build/core/tests/starsim_core_tests.exe "[optim]"
```
The full suite (`tools/build.ps1` without flags) must stay green at the end.

## Review Focus

- Rank-deficient equality constraints (a builder emits the same row twice) -> still `Optimal` thanks to the regularisation; tested in Task 8.
- A variable that appears in no constraint and has zero cost (unused block) -> `Optimal`, value ~0, no factorisation failure; tested in Task 8.
- Badly scaled SI data (row coefficients 1e6 next to 1e-3, variables of very different magnitude) -> same optimum as the well-scaled problem; tested in Task 8.
- No inequality rows at all (`m = 0`, pure equality LP) -> `Optimal` without dividing by a zero cone degree; tested in Task 8.
- Second-order cones of dimension 1 and 2 (degenerate cones a builder may produce) -> handled like `s0 >= 0` and `s0 >= |s1|`; tested in Task 4 (cone algebra) and Task 8 (solve).

---

## File Structure

| File | Responsibility |
|------|----------------|
| `core/include/starsim/optim/sparse_matrix.hpp`, `core/src/optim/sparse_matrix.cpp` | `Triplet`, `CscMatrix` (assembly, transpose, products), vector norms |
| `core/include/starsim/optim/minimum_degree.hpp`, `core/src/optim/minimum_degree.cpp` | fill-reducing minimum-degree ordering |
| `core/include/starsim/optim/ldl.hpp`, `core/src/optim/ldl.cpp` | `SparseLdl`: symbolic + numeric LDL^T, dynamic regularisation, solve |
| `core/include/starsim/optim/cones.hpp`, `core/src/optim/cones.cpp` | `ConeLayout`, Jordan algebra, max step, `NtScaling` |
| `core/include/starsim/optim/socp_problem.hpp`, `core/src/optim/socp_problem.cpp` | `SocpProblem`, `SocpSettings`, `SocpStatus`, `SocpResult`, validation |
| `core/include/starsim/optim/socp_builder.hpp`, `core/src/optim/socp_builder.cpp` | `AffineExpr`, `VariableBlock`, `SocpBuilder` |
| `core/include/starsim/optim/equilibration.hpp`, `core/src/optim/equilibration.cpp` | Ruiz scaling, unscaling |
| `core/include/starsim/optim/kkt_system.hpp`, `core/src/optim/kkt_system.cpp` | KKT assembly, factorisation, refined solves |
| `core/include/starsim/optim/socp_solver.hpp`, `core/src/optim/socp_solver.cpp` | `solve_socp`: HSDE predictor-corrector loop |
| `core/tests/test_optim_*.cpp`, `core/tests/socp_test_support.hpp` | unit, analytic and validation tests |
| `tools/validation/gen_socp_fixtures.py` | reference problem generator (Clarabel) |
| `tests/validation/socp/*.txt` | committed fixtures |

Spec deviations decided while planning (Task 10 records them in the spec): minimum degree on the explicit elimination graph instead of full AMD (same purpose, far simpler, adequate for banded trajectory problems); text fixtures `.txt` instead of `.json` (no JSON parser in the core); regularisation constants from ECOS (7e-8 static, 2e-7 dynamic) and up to 8 refinement steps; `add_equality(expr)` means `expr == 0`.

---

### Task 1: Sparse matrix (CSC) and module scaffolding

**Files:**
- Create: `core/include/starsim/optim/sparse_matrix.hpp`, `core/src/optim/sparse_matrix.cpp`
- Create: `core/tests/test_optim_sparse.cpp`
- Modify: `core/CMakeLists.txt` (add `src/optim/sparse_matrix.cpp` to `starsim_core`), `core/tests/CMakeLists.txt` (add `test_optim_sparse.cpp`)

**Interfaces:**
- Produces: `struct Triplet { int row; int col; double value; }`; `class CscMatrix` with `CscMatrix()`, `CscMatrix(int rows, int cols)`, `CscMatrix(int rows, int cols, const std::vector<Triplet>&)`, `rows()`, `cols()`, `nonzeros()`, `col_start()`, `row_index()`, `values()` (const and mutable), `at(row, col)`, `transposed()`, `multiply_add(x, y, alpha = 1)` (y += alpha A x), `multiply_transpose_add(x, y, alpha = 1)` (y += alpha A^T x); free functions `norm2`, `norm_inf`, `dot` on `std::span<const double>`.

- [ ] **Step 1: Write the failing test** - `core/tests/test_optim_sparse.cpp`

```cpp
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
```

Add `test_optim_sparse.cpp` to the `add_executable(starsim_core_tests ...)` list in `core/tests/CMakeLists.txt`.

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `starsim/optim/sparse_matrix.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/optim/sparse_matrix.hpp`

```cpp
#pragma once

#include <span>
#include <vector>

namespace starsim::optim {

struct Triplet {
    int row = 0;
    int col = 0;
    double value = 0.0;
};

// Compressed sparse column matrix. Row indices within a column are sorted and unique; explicit zeros are
// kept, they reserve a structural position (e.g. for values that change between factorisations).
class CscMatrix {
public:
    CscMatrix() = default;
    CscMatrix(int rows, int cols);
    // Duplicates are summed in input order (deterministic).
    CscMatrix(int rows, int cols, const std::vector<Triplet>& triplets);

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    int nonzeros() const { return static_cast<int>(row_index_.size()); }
    const std::vector<int>& col_start() const { return col_start_; }  // size cols + 1
    const std::vector<int>& row_index() const { return row_index_; }
    const std::vector<double>& values() const { return values_; }
    std::vector<double>& values() { return values_; }

    double at(int row, int col) const;  // 0 if not stored
    CscMatrix transposed() const;
    // y += alpha * A x
    void multiply_add(std::span<const double> x, std::span<double> y, double alpha = 1.0) const;
    // y += alpha * A^T x
    void multiply_transpose_add(std::span<const double> x, std::span<double> y, double alpha = 1.0) const;

private:
    int rows_ = 0;
    int cols_ = 0;
    std::vector<int> col_start_{0};
    std::vector<int> row_index_;
    std::vector<double> values_;
};

double norm2(std::span<const double> v);
double norm_inf(std::span<const double> v);
double dot(std::span<const double> a, std::span<const double> b);

}  // namespace starsim::optim
```

`core/src/optim/sparse_matrix.cpp`:

```cpp
#include "starsim/optim/sparse_matrix.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace starsim::optim {

CscMatrix::CscMatrix(int rows, int cols)
    : rows_(rows), cols_(cols), col_start_(static_cast<std::size_t>(cols) + 1, 0) {}

CscMatrix::CscMatrix(int rows, int cols, const std::vector<Triplet>& triplets) : CscMatrix(rows, cols) {
    for (const auto& t : triplets) {
        if (t.row < 0 || t.row >= rows || t.col < 0 || t.col >= cols) {
            throw std::out_of_range("CscMatrix: triplet outside the matrix");
        }
    }
    std::vector<Triplet> sorted = triplets;
    std::stable_sort(sorted.begin(), sorted.end(), [](const Triplet& a, const Triplet& b) {
        return a.col != b.col ? a.col < b.col : a.row < b.row;
    });
    int last_row = -1;
    int last_col = -1;
    for (const auto& t : sorted) {
        if (t.row == last_row && t.col == last_col) {
            values_.back() += t.value;
            continue;
        }
        row_index_.push_back(t.row);
        values_.push_back(t.value);
        ++col_start_[static_cast<std::size_t>(t.col) + 1];
        last_row = t.row;
        last_col = t.col;
    }
    for (int j = 0; j < cols; ++j) col_start_[j + 1] += col_start_[j];
}

double CscMatrix::at(int row, int col) const {
    const auto begin = row_index_.begin() + col_start_[col];
    const auto end = row_index_.begin() + col_start_[col + 1];
    const auto it = std::lower_bound(begin, end, row);
    return it != end && *it == row ? values_[static_cast<std::size_t>(it - row_index_.begin())] : 0.0;
}

CscMatrix CscMatrix::transposed() const {
    std::vector<Triplet> t;
    t.reserve(values_.size());
    for (int j = 0; j < cols_; ++j) {
        for (int p = col_start_[j]; p < col_start_[j + 1]; ++p) t.push_back({j, row_index_[p], values_[p]});
    }
    return CscMatrix(cols_, rows_, t);
}

void CscMatrix::multiply_add(std::span<const double> x, std::span<double> y, double alpha) const {
    for (int j = 0; j < cols_; ++j) {
        const double xj = alpha * x[j];
        if (xj == 0.0) continue;
        for (int p = col_start_[j]; p < col_start_[j + 1]; ++p) y[row_index_[p]] += values_[p] * xj;
    }
}

void CscMatrix::multiply_transpose_add(std::span<const double> x, std::span<double> y, double alpha) const {
    for (int j = 0; j < cols_; ++j) {
        double sum = 0.0;
        for (int p = col_start_[j]; p < col_start_[j + 1]; ++p) sum += values_[p] * x[row_index_[p]];
        y[j] += alpha * sum;
    }
}

double norm2(std::span<const double> v) { return std::sqrt(dot(v, v)); }

double norm_inf(std::span<const double> v) {
    double m = 0.0;
    for (const double e : v) m = std::max(m, std::abs(e));
    return m;
}

double dot(std::span<const double> a, std::span<const double> b) {
    double s = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
    return s;
}

}  // namespace starsim::optim
```

Add `src/optim/sparse_matrix.cpp` to the `add_library(starsim_core STATIC ...)` list in `core/CMakeLists.txt` (after `src/mission/lunar_transfer.cpp`).

- [ ] **Step 4: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: all `[optim]` tests pass.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/optim/sparse_matrix.hpp core/src/optim/sparse_matrix.cpp core/tests/test_optim_sparse.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "Add CSC sparse matrix for the optim module"
```

---

### Task 2: Minimum-degree ordering

**Files:**
- Create: `core/include/starsim/optim/minimum_degree.hpp`, `core/src/optim/minimum_degree.cpp`
- Create: `core/tests/test_optim_ordering.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `CscMatrix` (Task 1).
- Produces: `std::vector<int> minimum_degree_ordering(const CscMatrix& pattern)`; `perm[k]` = original index of the k-th pivot.

- [ ] **Step 1: Write the failing test** - `core/tests/test_optim_ordering.cpp`

```cpp
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `minimum_degree.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/optim/minimum_degree.hpp`

```cpp
#pragma once

#include <vector>

#include "starsim/optim/sparse_matrix.hpp"

namespace starsim::optim {

// Fill-reducing ordering of a symmetric sparsity pattern: minimum degree on the explicit elimination
// graph (eliminating a node joins its neighbours into a clique), ties broken by the lowest index so the
// result is deterministic. The pattern may hold one or both triangles; values and the diagonal are ignored.
// Returns perm with perm[k] = original index of the k-th pivot.
std::vector<int> minimum_degree_ordering(const CscMatrix& pattern);

}  // namespace starsim::optim
```

`core/src/optim/minimum_degree.cpp`:

```cpp
#include "starsim/optim/minimum_degree.hpp"

#include <algorithm>
#include <iterator>
#include <set>
#include <stdexcept>
#include <utility>

namespace starsim::optim {

std::vector<int> minimum_degree_ordering(const CscMatrix& pattern) {
    const int n = pattern.cols();
    if (pattern.rows() != n) throw std::invalid_argument("minimum_degree_ordering: matrix must be square");

    std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(n));
    for (int j = 0; j < n; ++j) {
        for (int p = pattern.col_start()[j]; p < pattern.col_start()[j + 1]; ++p) {
            const int i = pattern.row_index()[p];
            if (i == j) continue;
            adjacency[i].push_back(j);
            adjacency[j].push_back(i);
        }
    }
    std::set<std::pair<int, int>> queue;  // (degree, node): begin() is the next pivot
    for (int i = 0; i < n; ++i) {
        auto& list = adjacency[i];
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end());
        queue.insert({static_cast<int>(list.size()), i});
    }

    std::vector<int> perm;
    perm.reserve(static_cast<std::size_t>(n));
    std::vector<int> merged;
    while (!queue.empty()) {
        const int pivot = queue.begin()->second;
        queue.erase(queue.begin());
        perm.push_back(pivot);
        const std::vector<int> neighbours = std::move(adjacency[pivot]);
        adjacency[pivot].clear();
        // The neighbours become a clique; the pivot leaves the graph.
        for (const int u : neighbours) {
            auto& list = adjacency[u];
            queue.erase({static_cast<int>(list.size()), u});
            merged.clear();
            std::set_union(list.begin(), list.end(), neighbours.begin(), neighbours.end(), std::back_inserter(merged));
            list.clear();
            for (const int v : merged) {
                if (v != u && v != pivot) list.push_back(v);
            }
            queue.insert({static_cast<int>(list.size()), u});
        }
    }
    return perm;
}

}  // namespace starsim::optim
```

Add `src/optim/minimum_degree.cpp` to `core/CMakeLists.txt` and `test_optim_ordering.cpp` to `core/tests/CMakeLists.txt`.

- [ ] **Step 4: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/optim/minimum_degree.hpp core/src/optim/minimum_degree.cpp core/tests/test_optim_ordering.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "Add minimum-degree fill-reducing ordering"
```

---

### Task 3: Sparse LDL^T factorisation

**Files:**
- Create: `core/include/starsim/optim/ldl.hpp`, `core/src/optim/ldl.cpp`
- Create: `core/tests/test_optim_ldl.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `CscMatrix`, `minimum_degree_ordering` (Tasks 1-2).
- Produces: `class SparseLdl` with `struct Regularization { double epsilon = 1e-13; double delta = 2e-7; }`, `void analyze(const CscMatrix& matrix, std::vector<int> permutation)`, `bool factor(const CscMatrix& matrix, std::span<const int> sign, Regularization = {})`, `void solve(std::span<double> rhs_in_solution_out) const`, `int size() const`, `int factor_nonzeros() const`, `int regularized_pivots() const`.

- [ ] **Step 1: Write the failing test** - `core/tests/test_optim_ldl.cpp`

```cpp
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `ldl.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/optim/ldl.hpp`

```cpp
#pragma once

#include <span>
#include <vector>

#include "starsim/optim/sparse_matrix.hpp"

namespace starsim::optim {

// Sparse LDL^T factorisation of a symmetric quasi-definite matrix without pivoting (up-looking, after
// T. Davis' LDL package), with a fixed fill-reducing permutation. The symbolic analysis is done once per
// sparsity pattern; the numeric factorisation can be repeated for new values on the same pattern.
// Dynamic regularisation: a pivot d with sign[k] * d < epsilon is replaced by sign[k] * delta and counted.
// solve() uses an internal work array: one SparseLdl must not be used from two threads.
class SparseLdl {
public:
    struct Regularization {
        double epsilon = 1e-13;  // ECOS defaults
        double delta = 2e-7;
    };

    // `matrix` in full symmetric storage (both triangles); `permutation[k]` = original index of pivot k.
    void analyze(const CscMatrix& matrix, std::vector<int> permutation);
    // `matrix` must have the pattern given to analyze(). `sign` (original ordering): expected pivot signs
    // (+1 / -1). Returns false if a pivot is not finite.
    bool factor(const CscMatrix& matrix, std::span<const int> sign, Regularization regularization = {});
    // In place: right-hand side (original ordering) -> solution. Valid after a successful factor().
    void solve(std::span<double> rhs) const;

    int size() const { return n_; }
    int factor_nonzeros() const { return lp_.empty() ? 0 : lp_.back(); }  // strictly lower part of L
    int regularized_pivots() const { return regularized_; }

private:
    int n_ = 0;
    std::vector<int> perm_, inverse_perm_;
    std::vector<int> parent_, lnz_, lp_, flag_, pattern_, li_;
    std::vector<double> lx_, d_, y_;
    mutable std::vector<double> work_;
    int regularized_ = 0;
};

}  // namespace starsim::optim
```

`core/src/optim/ldl.cpp`:

```cpp
#include "starsim/optim/ldl.hpp"

#include <cmath>
#include <stdexcept>

namespace starsim::optim {

void SparseLdl::analyze(const CscMatrix& a, std::vector<int> permutation) {
    n_ = a.cols();
    if (a.rows() != n_ || static_cast<int>(permutation.size()) != n_) {
        throw std::invalid_argument("SparseLdl::analyze: size mismatch");
    }
    perm_ = std::move(permutation);
    inverse_perm_.assign(static_cast<std::size_t>(n_), -1);
    for (int k = 0; k < n_; ++k) {
        if (perm_[k] < 0 || perm_[k] >= n_ || inverse_perm_[perm_[k]] != -1) {
            throw std::invalid_argument("SparseLdl::analyze: not a permutation");
        }
        inverse_perm_[perm_[k]] = k;
    }
    const auto n = static_cast<std::size_t>(n_);
    parent_.assign(n, -1);
    lnz_.assign(n, 0);
    flag_.assign(n, 0);
    lp_.assign(n + 1, 0);
    const auto& ap = a.col_start();
    const auto& ai = a.row_index();
    // Elimination tree and column counts of L (entries of the permuted upper triangle).
    for (int k = 0; k < n_; ++k) {
        flag_[k] = k;
        const int kk = perm_[k];
        for (int p = ap[kk]; p < ap[kk + 1]; ++p) {
            int i = inverse_perm_[ai[p]];
            if (i >= k) continue;
            for (; flag_[i] != k; i = parent_[i]) {
                if (parent_[i] == -1) parent_[i] = k;
                ++lnz_[i];
                flag_[i] = k;
            }
        }
    }
    for (int k = 0; k < n_; ++k) lp_[k + 1] = lp_[k] + lnz_[k];
    li_.assign(static_cast<std::size_t>(lp_[n_]), 0);
    lx_.assign(static_cast<std::size_t>(lp_[n_]), 0.0);
    d_.assign(n, 0.0);
    y_.assign(n, 0.0);
    pattern_.assign(n, 0);
    work_.assign(n, 0.0);
}

bool SparseLdl::factor(const CscMatrix& a, std::span<const int> sign, Regularization regularization) {
    const auto& ap = a.col_start();
    const auto& ai = a.row_index();
    const auto& ax = a.values();
    regularized_ = 0;
    for (int k = 0; k < n_; ++k) {
        // Nonzero pattern of row k of L (reach in the elimination tree) and scatter of column k.
        y_[k] = 0.0;
        int top = n_;
        flag_[k] = k;
        lnz_[k] = 0;
        const int kk = perm_[k];
        for (int p = ap[kk]; p < ap[kk + 1]; ++p) {
            int i = inverse_perm_[ai[p]];
            if (i > k) continue;
            y_[i] += ax[p];
            int len = 0;
            for (; flag_[i] != k; i = parent_[i]) {
                pattern_[len++] = i;
                flag_[i] = k;
            }
            while (len > 0) pattern_[--top] = pattern_[--len];
        }
        d_[k] = y_[k];
        y_[k] = 0.0;
        for (; top < n_; ++top) {
            const int i = pattern_[top];
            const double yi = y_[i];
            y_[i] = 0.0;
            const int p2 = lp_[i] + lnz_[i];
            for (int p = lp_[i]; p < p2; ++p) y_[li_[p]] -= lx_[p] * yi;
            const double l_ki = yi / d_[i];
            d_[k] -= l_ki * yi;
            li_[p2] = k;
            lx_[p2] = l_ki;
            ++lnz_[i];
        }
        if (!std::isfinite(d_[k])) return false;
        const double s = sign[perm_[k]];
        if (s * d_[k] < regularization.epsilon) {
            d_[k] = s * regularization.delta;
            ++regularized_;
        }
    }
    return true;
}

void SparseLdl::solve(std::span<double> b) const {
    for (int k = 0; k < n_; ++k) work_[k] = b[perm_[k]];
    for (int j = 0; j < n_; ++j) {
        for (int p = lp_[j]; p < lp_[j + 1]; ++p) work_[li_[p]] -= lx_[p] * work_[j];
    }
    for (int j = 0; j < n_; ++j) work_[j] /= d_[j];
    for (int j = n_ - 1; j >= 0; --j) {
        for (int p = lp_[j]; p < lp_[j + 1]; ++p) work_[j] -= lx_[p] * work_[li_[p]];
    }
    for (int k = 0; k < n_; ++k) b[perm_[k]] = work_[k];
}

}  // namespace starsim::optim
```

Add `src/optim/ldl.cpp` and `test_optim_ldl.cpp` to the CMake lists.

- [ ] **Step 4: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/optim/ldl.hpp core/src/optim/ldl.cpp core/tests/test_optim_ldl.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "Add sparse LDL^T factorisation with dynamic regularisation"
```

---

### Task 4: Cone algebra and Nesterov-Todd scaling

**Files:**
- Create: `core/include/starsim/optim/cones.hpp`, `core/src/optim/cones.cpp`
- Create: `core/tests/test_optim_cones.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Produces: `struct ConeLayout { int nonnegative; std::vector<int> second_order; int size() const; int degree() const; }`; free functions `cone_product(layout, u, v, out)`, `cone_division(layout, lambda, v, out)` (out = lambda \ v, out must not alias v), `add_identity(layout, alpha, u)`, `min_eigenvalue(layout, u)` (+infinity for an empty cone), `max_step(layout, u, du)` (+infinity if unbounded); `class NtScaling` with `explicit NtScaling(const ConeLayout&)`, `bool update(s, z)`, `apply(x, y)` (y = W x), `apply_inverse(x, y)`, `lambda()`, `orthant_squared(i)`, `cone_squared(k)` (row-major q*q block of W^T W). W is symmetric, so W^T = W.

- [ ] **Step 1: Write the failing test** - `core/tests/test_optim_cones.cpp`

```cpp
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `cones.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/optim/cones.hpp`

```cpp
#pragma once

#include <span>
#include <vector>

namespace starsim::optim {

// K = R+^l x Q^{q1} x ... x Q^{qm}, Q^q = { u in R^q : ||(u1, ..., u_{q-1})|| <= u0 }. Vectors on K are laid
// out orthant first, then each second-order cone. Dimension 1 means u0 >= 0, dimension 2 means u0 >= |u1|.
struct ConeLayout {
    int nonnegative = 0;
    std::vector<int> second_order;  // dimensions, each >= 1

    int size() const;
    int degree() const;  // l + number of second-order cones
};

// Jordan algebra (Vandenberghe, "The CVXOPT linear and quadratic cone program solvers", 2010):
// u o v: orthant elementwise; cone (u.v, u0 v1 + v0 u1).
void cone_product(const ConeLayout& layout, std::span<const double> u, std::span<const double> v, std::span<double> out);
// out = lambda \ v, i.e. lambda o out = v; lambda in the interior. `out` must not alias `v`.
void cone_division(const ConeLayout& layout, std::span<const double> lambda, std::span<const double> v,
                   std::span<double> out);
// u += alpha e (e: identity element, 1 on the orthant, (1, 0, ..., 0) per cone).
void add_identity(const ConeLayout& layout, double alpha, std::span<double> u);
// Smallest eigenvalue (orthant: u_i; cone: u0 - ||u1||); +infinity for an empty cone.
double min_eigenvalue(const ConeLayout& layout, std::span<const double> u);
// Largest alpha with u + alpha du in K (u in the interior); +infinity if the ray stays inside.
double max_step(const ConeLayout& layout, std::span<const double> u, std::span<const double> du);

// Nesterov-Todd scaling W of an interior pair (s, z): W z = W^{-T} s = lambda. W is symmetric.
// Orthant: W = diag(sqrt(s / z)). Cone: W = eta [a q^T; q I + q q^T / (1 + a)] with (a, q) = wbar,
// wbar = (sbar + J zbar) / (2 gamma), gamma = sqrt((1 + sbar.zbar) / 2), eta = (det s / det z)^(1/4).
class NtScaling {
public:
    explicit NtScaling(const ConeLayout& layout);

    // False if s or z is not in the interior of K.
    bool update(std::span<const double> s, std::span<const double> z);
    void apply(std::span<const double> x, std::span<double> y) const;          // y = W x
    void apply_inverse(std::span<const double> x, std::span<double> y) const;  // y = W^{-1} x
    std::span<const double> lambda() const { return lambda_; }
    double orthant_squared(int i) const { return orthant_w_[i] * orthant_w_[i]; }
    std::span<const double> cone_squared(int k) const { return w_squared_[k]; }  // row-major q x q

private:
    void multiply(std::span<const double> x, std::span<double> y, bool inverse) const;

    ConeLayout layout_;
    std::vector<double> orthant_w_;
    std::vector<std::vector<double>> w_, w_inverse_, w_squared_;  // per cone, row-major q x q
    std::vector<double> lambda_;
};

}  // namespace starsim::optim
```

`core/src/optim/cones.cpp`:

```cpp
#include "starsim/optim/cones.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace starsim::optim {

namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();

double tail_norm(std::span<const double> u) {
    double s = 0.0;
    for (std::size_t i = 1; i < u.size(); ++i) s += u[i] * u[i];
    return std::sqrt(s);
}

// u0^2 - ||u1||^2 in the cancellation-free product form.
double cone_det(std::span<const double> u) {
    const double t = tail_norm(u);
    return (u[0] - t) * (u[0] + t);
}

// Smallest positive root of f(alpha) = |u0 + alpha d0|^2 - ||u1 + alpha d1||^2 = a alpha^2 + 2 b alpha + c
// (c > 0 for an interior u); the ray leaves the cone there.
double cone_max_step(std::span<const double> u, std::span<const double> d) {
    double a = d[0] * d[0];
    double b = u[0] * d[0];
    for (std::size_t i = 1; i < u.size(); ++i) {
        a -= d[i] * d[i];
        b -= u[i] * d[i];
    }
    const double c = cone_det(u);
    if (a == 0.0) return b < 0.0 ? -c / (2.0 * b) : infinity;
    const double discriminant = b * b - a * c;
    if (discriminant < 0.0) return infinity;
    const double t = -(b + std::copysign(std::sqrt(discriminant), b));
    double best = infinity;
    for (const double root : {t / a, t != 0.0 ? c / t : infinity}) {
        if (root > 0.0 && root < best) best = root;
    }
    return best;
}

}  // namespace

int ConeLayout::size() const {
    int n = nonnegative;
    for (const int q : second_order) n += q;
    return n;
}

int ConeLayout::degree() const { return nonnegative + static_cast<int>(second_order.size()); }

void cone_product(const ConeLayout& layout, std::span<const double> u, std::span<const double> v, std::span<double> out) {
    int i = 0;
    for (; i < layout.nonnegative; ++i) out[i] = u[i] * v[i];
    for (const int q : layout.second_order) {
        double d = 0.0;
        for (int k = 0; k < q; ++k) d += u[i + k] * v[i + k];
        for (int k = 1; k < q; ++k) out[i + k] = u[i] * v[i + k] + v[i] * u[i + k];
        out[i] = d;
        i += q;
    }
}

void cone_division(const ConeLayout& layout, std::span<const double> lambda, std::span<const double> v,
                   std::span<double> out) {
    int i = 0;
    for (; i < layout.nonnegative; ++i) out[i] = v[i] / lambda[i];
    for (const int q : layout.second_order) {
        const auto l = lambda.subspan(i, q);
        double tail = 0.0;
        for (int k = 1; k < q; ++k) tail += l[k] * v[i + k];
        const double x0 = (l[0] * v[i] - tail) / cone_det(l);
        out[i] = x0;
        for (int k = 1; k < q; ++k) out[i + k] = (v[i + k] - x0 * l[k]) / l[0];
        i += q;
    }
}

void add_identity(const ConeLayout& layout, double alpha, std::span<double> u) {
    int i = 0;
    for (; i < layout.nonnegative; ++i) u[i] += alpha;
    for (const int q : layout.second_order) {
        u[i] += alpha;
        i += q;
    }
}

double min_eigenvalue(const ConeLayout& layout, std::span<const double> u) {
    double m = infinity;
    int i = 0;
    for (; i < layout.nonnegative; ++i) m = std::min(m, u[i]);
    for (const int q : layout.second_order) {
        m = std::min(m, u[i] - tail_norm(u.subspan(i, q)));
        i += q;
    }
    return m;
}

double max_step(const ConeLayout& layout, std::span<const double> u, std::span<const double> du) {
    double alpha = infinity;
    int i = 0;
    for (; i < layout.nonnegative; ++i) {
        if (du[i] < 0.0) alpha = std::min(alpha, -u[i] / du[i]);
    }
    for (const int q : layout.second_order) {
        alpha = std::min(alpha, cone_max_step(u.subspan(i, q), du.subspan(i, q)));
        i += q;
    }
    return alpha;
}

NtScaling::NtScaling(const ConeLayout& layout)
    : layout_(layout),
      orthant_w_(static_cast<std::size_t>(layout.nonnegative), 1.0),
      lambda_(static_cast<std::size_t>(layout.size()), 0.0) {
    for (const int q : layout.second_order) {
        const auto block = static_cast<std::size_t>(q) * static_cast<std::size_t>(q);
        w_.emplace_back(block, 0.0);
        w_inverse_.emplace_back(block, 0.0);
        w_squared_.emplace_back(block, 0.0);
    }
}

bool NtScaling::update(std::span<const double> s, std::span<const double> z) {
    int i = 0;
    for (; i < layout_.nonnegative; ++i) {
        if (!(s[i] > 0.0 && z[i] > 0.0)) return false;
        orthant_w_[i] = std::sqrt(s[i] / z[i]);
        lambda_[i] = std::sqrt(s[i] * z[i]);
    }
    for (std::size_t k = 0; k < layout_.second_order.size(); ++k) {
        const int q = layout_.second_order[k];
        const auto sk = s.subspan(i, q);
        const auto zk = z.subspan(i, q);
        const double s_det = cone_det(sk);
        const double z_det = cone_det(zk);
        if (!(sk[0] > 0.0 && zk[0] > 0.0 && s_det > 0.0 && z_det > 0.0)) return false;
        const double s_norm = std::sqrt(s_det);
        const double z_norm = std::sqrt(z_det);
        double sz = 0.0;
        for (int r = 0; r < q; ++r) sz += sk[r] * zk[r];
        const double gamma = std::sqrt(0.5 * (1.0 + sz / (s_norm * z_norm)));
        const double eta = std::sqrt(s_norm / z_norm);
        const double a = (sk[0] / s_norm + zk[0] / z_norm) / (2.0 * gamma);
        auto qv = [&](int r) { return (sk[r] / s_norm - zk[r] / z_norm) / (2.0 * gamma); };  // r >= 1
        auto& w = w_[k];
        auto& wi = w_inverse_[k];
        w[0] = eta * a;
        wi[0] = a / eta;
        for (int r = 1; r < q; ++r) {
            w[r] = w[static_cast<std::size_t>(r) * q] = eta * qv(r);
            wi[r] = wi[static_cast<std::size_t>(r) * q] = -qv(r) / eta;
            for (int c = 1; c < q; ++c) {
                const double inner = (r == c ? 1.0 : 0.0) + qv(r) * qv(c) / (1.0 + a);
                w[static_cast<std::size_t>(r) * q + c] = eta * inner;
                wi[static_cast<std::size_t>(r) * q + c] = inner / eta;
            }
        }
        auto& w2 = w_squared_[k];
        for (int r = 0; r < q; ++r) {
            for (int c = 0; c < q; ++c) {
                double sum = 0.0;
                for (int m = 0; m < q; ++m) sum += w[static_cast<std::size_t>(r) * q + m] * w[static_cast<std::size_t>(m) * q + c];
                w2[static_cast<std::size_t>(r) * q + c] = sum;
            }
        }
        for (int r = 0; r < q; ++r) {
            double sum = 0.0;
            for (int c = 0; c < q; ++c) sum += w[static_cast<std::size_t>(r) * q + c] * zk[c];
            lambda_[i + r] = sum;
        }
        i += q;
    }
    return true;
}

void NtScaling::apply(std::span<const double> x, std::span<double> y) const { multiply(x, y, false); }

void NtScaling::apply_inverse(std::span<const double> x, std::span<double> y) const { multiply(x, y, true); }

void NtScaling::multiply(std::span<const double> x, std::span<double> y, bool inverse) const {
    int i = 0;
    for (; i < layout_.nonnegative; ++i) y[i] = inverse ? x[i] / orthant_w_[i] : x[i] * orthant_w_[i];
    for (std::size_t k = 0; k < layout_.second_order.size(); ++k) {
        const int q = layout_.second_order[k];
        const auto& m = inverse ? w_inverse_[k] : w_[k];
        for (int r = 0; r < q; ++r) {
            double sum = 0.0;
            for (int c = 0; c < q; ++c) sum += m[static_cast<std::size_t>(r) * q + c] * x[i + c];
            y[i + r] = sum;
        }
        i += q;
    }
}

}  // namespace starsim::optim
```

Add `src/optim/cones.cpp` and `test_optim_cones.cpp` to the CMake lists.

- [ ] **Step 4: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/optim/cones.hpp core/src/optim/cones.cpp core/tests/test_optim_cones.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "Add cone algebra and Nesterov-Todd scaling"
```

---

### Task 5: Problem types and `SocpBuilder`

**Files:**
- Create: `core/include/starsim/optim/socp_problem.hpp`, `core/src/optim/socp_problem.cpp`
- Create: `core/include/starsim/optim/socp_builder.hpp`, `core/src/optim/socp_builder.cpp`
- Create: `core/tests/test_optim_builder.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `CscMatrix`, `ConeLayout`.
- Produces:
  - `struct SocpProblem { int variables; std::vector<double> c; CscMatrix A; std::vector<double> b; CscMatrix G; std::vector<double> h; ConeLayout cones; void validate() const; }`
  - `struct SocpSettings { double tolerance = 1e-8; int max_iterations = 50; double static_regularization = 7e-8; int refinement_steps = 8; int equilibration_passes = 10; bool verbose = false; }`
  - `enum class SocpStatus { Optimal, PrimalInfeasible, DualInfeasible, MaxIterations, NumericalError }`, `const char* to_string(SocpStatus)`
  - `struct SocpResult { SocpStatus status; std::vector<double> x, y, z, s; double objective; int iterations; double primal_residual, dual_residual, gap; }`
  - `struct AffineExpr { std::vector<std::pair<int, double>> terms; double constant; AffineExpr(); explicit AffineExpr(double); AffineExpr& add(int variable, double coefficient); }`
  - `struct VariableBlock { int offset; int size; std::string name; int operator[](int i) const; }`
  - `class SocpBuilder { VariableBlock add_variables(int count, std::string name); int variable_count() const; void set_objective(const AffineExpr&); double objective_constant() const; void add_equality(const AffineExpr&); void add_nonnegative(const AffineExpr&); void add_second_order_cone(const std::vector<AffineExpr>&); SocpProblem build() const; }`

- [ ] **Step 1: Write the failing test** - `core/tests/test_optim_builder.cpp`

```cpp
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `socp_builder.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/optim/socp_problem.hpp`

```cpp
#pragma once

#include <vector>

#include "starsim/optim/cones.hpp"
#include "starsim/optim/sparse_matrix.hpp"

namespace starsim::optim {

// minimise c^T x  subject to  A x = b,  G x + s = h,  s in K (rows of G: orthant first, then each cone).
struct SocpProblem {
    int variables = 0;
    std::vector<double> c;
    CscMatrix A;  // p x n (may have 0 rows, but n columns)
    std::vector<double> b;
    CscMatrix G;  // m x n, m = cones.size()
    std::vector<double> h;
    ConeLayout cones;

    void validate() const;  // throws std::invalid_argument on inconsistent sizes
};

struct SocpSettings {
    double tolerance = 1e-8;              // relative residuals and duality gap
    int max_iterations = 50;
    double static_regularization = 7e-8;  // KKT diagonal (ECOS default)
    int refinement_steps = 8;             // iterative refinement per KKT solve
    int equilibration_passes = 10;        // Ruiz passes
    bool verbose = false;                 // per-iteration table on stderr
};

enum class SocpStatus { Optimal, PrimalInfeasible, DualInfeasible, MaxIterations, NumericalError };

const char* to_string(SocpStatus status);

struct SocpResult {
    SocpStatus status = SocpStatus::NumericalError;
    // Optimal / MaxIterations / NumericalError: last iterate. PrimalInfeasible: (y, z) is a certificate with
    // b^T y + h^T z = -1. DualInfeasible: (x, s) is a certificate with c^T x = -1.
    std::vector<double> x, y, z, s;
    double objective = 0.0;  // c^T x (+inf primal infeasible, -inf dual infeasible)
    int iterations = 0;
    double primal_residual = 0.0;
    double dual_residual = 0.0;
    double gap = 0.0;
};

}  // namespace starsim::optim
```

`core/src/optim/socp_problem.cpp`:

```cpp
#include "starsim/optim/socp_problem.hpp"

#include <stdexcept>
#include <string>

namespace starsim::optim {

void SocpProblem::validate() const {
    const auto fail = [](const char* what) { throw std::invalid_argument(std::string("SocpProblem: ") + what); };
    if (variables < 0) fail("negative variable count");
    if (static_cast<int>(c.size()) != variables) fail("c has the wrong size");
    if (A.cols() != variables || static_cast<int>(b.size()) != A.rows()) fail("A or b has the wrong size");
    if (G.cols() != variables || static_cast<int>(h.size()) != G.rows()) fail("G or h has the wrong size");
    if (cones.nonnegative < 0 || cones.size() != G.rows()) fail("cone layout does not match G");
    for (const int q : cones.second_order) {
        if (q < 1) fail("second-order cone of dimension < 1");
    }
}

const char* to_string(SocpStatus status) {
    switch (status) {
        case SocpStatus::Optimal: return "optimal";
        case SocpStatus::PrimalInfeasible: return "primal infeasible";
        case SocpStatus::DualInfeasible: return "dual infeasible";
        case SocpStatus::MaxIterations: return "maximum iterations";
        case SocpStatus::NumericalError: return "numerical error";
    }
    return "";
}

}  // namespace starsim::optim
```

`core/include/starsim/optim/socp_builder.hpp`:

```cpp
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "starsim/optim/socp_problem.hpp"

namespace starsim::optim {

// Affine expression sum(coefficient * x[variable]) + constant.
struct AffineExpr {
    std::vector<std::pair<int, double>> terms;
    double constant = 0.0;

    AffineExpr() = default;
    explicit AffineExpr(double value) : constant(value) {}
    AffineExpr& add(int variable, double coefficient) {
        terms.emplace_back(variable, coefficient);
        return *this;
    }
};

// Contiguous block of variables; block[i] is the global index of its i-th variable.
struct VariableBlock {
    int offset = 0;
    int size = 0;
    std::string name;

    int operator[](int i) const { return offset + i; }
};

// Assembles a SocpProblem from named variable blocks and affine constraints, so callers never compute row
// or column indices by hand. Inequality rows are reordered into the cone order (orthant, then cones).
class SocpBuilder {
public:
    VariableBlock add_variables(int count, std::string name);
    int variable_count() const { return variables_; }

    void set_objective(const AffineExpr& expr);  // minimised; the constant is kept separately
    double objective_constant() const { return objective_.constant; }
    void add_equality(const AffineExpr& expr);                        // expr == 0
    void add_nonnegative(const AffineExpr& expr);                     // expr >= 0
    void add_second_order_cone(const std::vector<AffineExpr>& rows);  // rows[0] >= ||rows[1..]||

    SocpProblem build() const;

private:
    void check(const AffineExpr& expr) const;

    int variables_ = 0;
    AffineExpr objective_;
    std::vector<AffineExpr> equalities_;
    std::vector<AffineExpr> nonnegatives_;
    std::vector<std::vector<AffineExpr>> cones_;
};

}  // namespace starsim::optim
```

`core/src/optim/socp_builder.cpp`:

```cpp
#include "starsim/optim/socp_builder.hpp"

#include <stdexcept>

namespace starsim::optim {

VariableBlock SocpBuilder::add_variables(int count, std::string name) {
    if (count < 0) throw std::invalid_argument("SocpBuilder: negative variable count");
    VariableBlock block{variables_, count, std::move(name)};
    variables_ += count;
    return block;
}

void SocpBuilder::check(const AffineExpr& expr) const {
    for (const auto& [index, coefficient] : expr.terms) {
        if (index < 0 || index >= variables_) throw std::out_of_range("SocpBuilder: unknown variable index");
    }
}

void SocpBuilder::set_objective(const AffineExpr& expr) {
    check(expr);
    objective_ = expr;
}

void SocpBuilder::add_equality(const AffineExpr& expr) {
    check(expr);
    equalities_.push_back(expr);
}

void SocpBuilder::add_nonnegative(const AffineExpr& expr) {
    check(expr);
    nonnegatives_.push_back(expr);
}

void SocpBuilder::add_second_order_cone(const std::vector<AffineExpr>& rows) {
    if (rows.empty()) throw std::invalid_argument("SocpBuilder: empty second-order cone");
    for (const auto& row : rows) check(row);
    cones_.push_back(rows);
}

SocpProblem SocpBuilder::build() const {
    SocpProblem p;
    p.variables = variables_;
    p.c.assign(static_cast<std::size_t>(variables_), 0.0);
    for (const auto& [index, coefficient] : objective_.terms) p.c[index] += coefficient;

    std::vector<Triplet> a;
    for (int r = 0; r < static_cast<int>(equalities_.size()); ++r) {
        for (const auto& [index, coefficient] : equalities_[r].terms) a.push_back({r, index, coefficient});
        p.b.push_back(-equalities_[r].constant);
    }
    p.A = CscMatrix(static_cast<int>(equalities_.size()), variables_, a);

    // s = expr = coefficients x + constant  <=>  G x + s = h with G = -coefficients, h = constant.
    std::vector<Triplet> g;
    int row = 0;
    auto add_row = [&](const AffineExpr& expr) {
        for (const auto& [index, coefficient] : expr.terms) g.push_back({row, index, -coefficient});
        p.h.push_back(expr.constant);
        ++row;
    };
    for (const auto& expr : nonnegatives_) add_row(expr);
    for (const auto& cone : cones_) {
        for (const auto& expr : cone) add_row(expr);
        p.cones.second_order.push_back(static_cast<int>(cone.size()));
    }
    p.cones.nonnegative = static_cast<int>(nonnegatives_.size());
    p.G = CscMatrix(row, variables_, g);
    return p;
}

}  // namespace starsim::optim
```

Add `src/optim/socp_problem.cpp`, `src/optim/socp_builder.cpp` and `test_optim_builder.cpp` to the CMake lists.

- [ ] **Step 4: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/optim/socp_problem.hpp core/src/optim/socp_problem.cpp core/include/starsim/optim/socp_builder.hpp core/src/optim/socp_builder.cpp core/tests/test_optim_builder.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "Add SOCP problem types and builder"
```

---

### Task 6: Ruiz equilibration

**Files:**
- Create: `core/include/starsim/optim/equilibration.hpp`, `core/src/optim/equilibration.cpp`
- Create: `core/tests/test_optim_equilibration.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `SocpProblem` (Task 5).
- Produces: `struct Equilibration { std::vector<double> column, equality_row, cone_row; }`; `Equilibration equilibrate(SocpProblem& problem, int passes)` (scales in place: `A <- E_A A D`, `G <- E_G G D`, `c <- D c`, `b <- E_A b`, `h <- E_G h`); `void unscale(const Equilibration&, std::span<double> x, std::span<double> y, std::span<double> z, std::span<double> s)` (`x = D x^`, `y = E_A y^`, `z = E_G z^`, `s = s^ / E_G`).

- [ ] **Step 1: Write the failing test** - `core/tests/test_optim_equilibration.cpp`

```cpp
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `equilibration.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/optim/equilibration.hpp`

```cpp
#pragma once

#include <span>
#include <vector>

#include "starsim/optim/socp_problem.hpp"

namespace starsim::optim {

// Diagonal scaling factors of a Ruiz equilibration: x = D x^, rows of A scaled by E_A, rows of G by E_G.
struct Equilibration {
    std::vector<double> column;        // D
    std::vector<double> equality_row;  // E_A
    std::vector<double> cone_row;      // E_G (equal within each second-order cone)
};

// Ruiz equilibration (infinity norms) of [A; G], applied in place: A <- E_A A D, G <- E_G G D, c <- D c,
// b <- E_A b, h <- E_G h. The rows of one second-order cone share a factor so the cone is preserved.
Equilibration equilibrate(SocpProblem& problem, int passes);

// Maps a solution of the scaled problem back: x = D x^, y = E_A y^, z = E_G z^, s = s^ / E_G.
void unscale(const Equilibration& e, std::span<double> x, std::span<double> y, std::span<double> z,
             std::span<double> s);

}  // namespace starsim::optim
```

`core/src/optim/equilibration.cpp`:

```cpp
#include "starsim/optim/equilibration.hpp"

#include <algorithm>
#include <cmath>

namespace starsim::optim {

namespace {

// Norms outside this range are clamped per pass so one pass never scales by more than 1e2.
constexpr double min_norm = 1e-4;
constexpr double max_norm = 1e4;

double factor(double norm) { return norm > 0.0 ? 1.0 / std::sqrt(std::clamp(norm, min_norm, max_norm)) : 1.0; }

void accumulate(const CscMatrix& m, std::vector<double>& column, std::vector<double>& row) {
    for (int j = 0; j < m.cols(); ++j) {
        for (int p = m.col_start()[j]; p < m.col_start()[j + 1]; ++p) {
            const double v = std::abs(m.values()[p]);
            column[j] = std::max(column[j], v);
            row[m.row_index()[p]] = std::max(row[m.row_index()[p]], v);
        }
    }
}

void scale(CscMatrix& m, const std::vector<double>& row, const std::vector<double>& column) {
    auto& values = m.values();
    for (int j = 0; j < m.cols(); ++j) {
        for (int p = m.col_start()[j]; p < m.col_start()[j + 1]; ++p) values[p] *= row[m.row_index()[p]] * column[j];
    }
}

}  // namespace

Equilibration equilibrate(SocpProblem& problem, int passes) {
    const auto n = static_cast<std::size_t>(problem.variables);
    const auto p = static_cast<std::size_t>(problem.A.rows());
    const auto m = static_cast<std::size_t>(problem.G.rows());
    Equilibration e{std::vector<double>(n, 1.0), std::vector<double>(p, 1.0), std::vector<double>(m, 1.0)};
    std::vector<double> column(n), row_a(p), row_g(m);
    for (int pass = 0; pass < passes; ++pass) {
        std::fill(column.begin(), column.end(), 0.0);
        std::fill(row_a.begin(), row_a.end(), 0.0);
        std::fill(row_g.begin(), row_g.end(), 0.0);
        accumulate(problem.A, column, row_a);
        accumulate(problem.G, column, row_g);
        int offset = problem.cones.nonnegative;
        for (const int q : problem.cones.second_order) {
            const auto begin = row_g.begin() + offset;
            std::fill(begin, begin + q, *std::max_element(begin, begin + q));
            offset += q;
        }
        for (auto& v : column) v = factor(v);
        for (auto& v : row_a) v = factor(v);
        for (auto& v : row_g) v = factor(v);
        scale(problem.A, row_a, column);
        scale(problem.G, row_g, column);
        for (std::size_t j = 0; j < n; ++j) e.column[j] *= column[j];
        for (std::size_t i = 0; i < p; ++i) e.equality_row[i] *= row_a[i];
        for (std::size_t i = 0; i < m; ++i) e.cone_row[i] *= row_g[i];
    }
    for (std::size_t j = 0; j < n; ++j) problem.c[j] *= e.column[j];
    for (std::size_t i = 0; i < p; ++i) problem.b[i] *= e.equality_row[i];
    for (std::size_t i = 0; i < m; ++i) problem.h[i] *= e.cone_row[i];
    return e;
}

void unscale(const Equilibration& e, std::span<double> x, std::span<double> y, std::span<double> z,
             std::span<double> s) {
    for (std::size_t j = 0; j < x.size(); ++j) x[j] *= e.column[j];
    for (std::size_t i = 0; i < y.size(); ++i) y[i] *= e.equality_row[i];
    for (std::size_t i = 0; i < z.size(); ++i) {
        z[i] *= e.cone_row[i];
        s[i] /= e.cone_row[i];
    }
}

}  // namespace starsim::optim
```

Add `src/optim/equilibration.cpp` and `test_optim_equilibration.cpp` to the CMake lists.

- [ ] **Step 4: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/optim/equilibration.hpp core/src/optim/equilibration.cpp core/tests/test_optim_equilibration.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "Add Ruiz equilibration for SOCP data"
```

---

### Task 7: KKT system

**Files:**
- Create: `core/include/starsim/optim/kkt_system.hpp`, `core/src/optim/kkt_system.cpp`
- Create: `core/tests/test_optim_kkt.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `CscMatrix`, `minimum_degree_ordering`, `SparseLdl`, `ConeLayout`, `NtScaling`.
- Produces: `class KktSystem { KktSystem(const CscMatrix& a, const CscMatrix& g, const ConeLayout& cones, double static_regularization, int refinement_steps); bool factor(const NtScaling& scaling); void solve(std::span<const double> rhs, std::span<double> solution); int size() const; int regularized_pivots() const; }` - unknown ordering `(x, y, z)`, size `n + p + m`.

- [ ] **Step 1: Write the failing test** - `core/tests/test_optim_kkt.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <vector>

#include "starsim/optim/kkt_system.hpp"

using namespace starsim::optim;

TEST_CASE("KKT solve satisfies the unregularised system", "[optim]") {
    const CscMatrix a(1, 3, {{0, 0, 1.0}, {0, 1, 1.0}, {0, 2, 1.0}});
    const CscMatrix g(4, 3, {{0, 0, -1.0}, {1, 2, -1.0}, {1, 1, 0.5}, {2, 0, -1.0}, {3, 1, -1.0}});
    ConeLayout cones;
    cones.nonnegative = 1;
    cones.second_order = {3};
    KktSystem kkt(a, g, cones, 7e-8, 8);
    REQUIRE(kkt.size() == 8);
    NtScaling scaling(cones);
    REQUIRE(scaling.update(std::vector<double>{0.5, 2.0, 0.3, -0.4}, std::vector<double>{1.5, 1.0, 0.2, 0.1}));
    REQUIRE(kkt.factor(scaling));

    std::vector<double> rhs(8), u(8);
    for (int i = 0; i < 8; ++i) rhs[i] = std::cos(i + 1.0);
    kkt.solve(rhs, u);

    const std::span<const double> ux(u.data(), 3), uy(u.data() + 3, 1), uz(u.data() + 4, 4);
    std::vector<double> r(8, 0.0);
    // Row block x: A^T uy + G^T uz
    a.multiply_transpose_add(uy, std::span<double>(r.data(), 3));
    g.multiply_transpose_add(uz, std::span<double>(r.data(), 3));
    // Row block y: A ux
    a.multiply_add(ux, std::span<double>(r.data() + 3, 1));
    // Row block z: G ux - W^T W uz
    g.multiply_add(ux, std::span<double>(r.data() + 4, 4));
    std::vector<double> wz(4), wwz(4);
    scaling.apply(uz, wz);
    scaling.apply(wz, wwz);
    for (int i = 0; i < 4; ++i) r[4 + i] -= wwz[i];
    for (int i = 0; i < 8; ++i) r[i] -= rhs[i];
    CHECK(norm_inf(r) <= 1e-10);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `kkt_system.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/optim/kkt_system.hpp`

```cpp
#pragma once

#include <span>
#include <vector>

#include "starsim/optim/cones.hpp"
#include "starsim/optim/ldl.hpp"
#include "starsim/optim/sparse_matrix.hpp"

namespace starsim::optim {

// KKT matrix of the interior-point iterations in full symmetric storage, unknowns (x, y, z):
//   [ delta I   A^T        G^T    ]
//   [ A        -delta I    0      ]
//   [ G         0         -W^T W  ]
// The sparsity pattern (dense W^T W block per second-order cone) is fixed: the minimum-degree ordering and
// the symbolic LDL^T analysis run once in the constructor. Solves are refined against the unregularised
// matrix (delta = 0).
class KktSystem {
public:
    KktSystem(const CscMatrix& a, const CscMatrix& g, const ConeLayout& cones, double static_regularization,
              int refinement_steps);

    // Writes -W^T W into the matrix and factorises. False on a numerical failure.
    bool factor(const NtScaling& scaling);
    // Solves K u = rhs (both of size n + p + m) after a successful factor().
    void solve(std::span<const double> rhs, std::span<double> solution);

    int size() const { return n_ + p_ + m_; }
    int regularized_pivots() const { return ldl_.regularized_pivots(); }

private:
    int n_ = 0, p_ = 0, m_ = 0;
    ConeLayout cones_;
    double delta_ = 0.0;
    int refinement_steps_ = 0;
    CscMatrix matrix_;
    std::vector<int> scaling_positions_;  // value index per W^T W entry: orthant diagonal, then cone blocks row-major
    std::vector<int> sign_;
    SparseLdl ldl_;
    std::vector<double> residual_;
};

}  // namespace starsim::optim
```

`core/src/optim/kkt_system.cpp`:

```cpp
#include "starsim/optim/kkt_system.hpp"

#include <algorithm>
#include <utility>

#include "starsim/optim/minimum_degree.hpp"

namespace starsim::optim {

namespace {

// Relative residual below which iterative refinement stops early.
constexpr double refinement_tolerance = 1e-14;

int value_position(const CscMatrix& m, int row, int col) {
    const auto begin = m.row_index().begin() + m.col_start()[col];
    const auto end = m.row_index().begin() + m.col_start()[col + 1];
    return static_cast<int>(std::lower_bound(begin, end, row) - m.row_index().begin());
}

}  // namespace

KktSystem::KktSystem(const CscMatrix& a, const CscMatrix& g, const ConeLayout& cones, double static_regularization,
                     int refinement_steps)
    : n_(a.cols()), p_(a.rows()), m_(g.rows()), cones_(cones), delta_(static_regularization),
      refinement_steps_(refinement_steps) {
    const int z0 = n_ + p_;
    std::vector<Triplet> t;
    for (int j = 0; j < n_; ++j) t.push_back({j, j, delta_});
    for (int i = 0; i < p_; ++i) t.push_back({n_ + i, n_ + i, -delta_});
    for (int j = 0; j < n_; ++j) {
        for (int q = a.col_start()[j]; q < a.col_start()[j + 1]; ++q) {
            t.push_back({n_ + a.row_index()[q], j, a.values()[q]});
            t.push_back({j, n_ + a.row_index()[q], a.values()[q]});
        }
        for (int q = g.col_start()[j]; q < g.col_start()[j + 1]; ++q) {
            t.push_back({z0 + g.row_index()[q], j, g.values()[q]});
            t.push_back({j, z0 + g.row_index()[q], g.values()[q]});
        }
    }
    // Placeholders for -W^T W (identity until the first factor()).
    std::vector<std::pair<int, int>> scaling_entries;
    int offset = z0;
    for (int i = 0; i < cones.nonnegative; ++i, ++offset) scaling_entries.push_back({offset, offset});
    for (const int q : cones.second_order) {
        for (int r = 0; r < q; ++r) {
            for (int c = 0; c < q; ++c) scaling_entries.push_back({offset + r, offset + c});
        }
        offset += q;
    }
    for (const auto& [r, c] : scaling_entries) t.push_back({r, c, r == c ? -1.0 : 0.0});
    matrix_ = CscMatrix(size(), size(), t);
    for (const auto& [r, c] : scaling_entries) scaling_positions_.push_back(value_position(matrix_, r, c));

    sign_.assign(static_cast<std::size_t>(size()), -1);
    std::fill(sign_.begin(), sign_.begin() + n_, 1);
    ldl_.analyze(matrix_, minimum_degree_ordering(matrix_));
    residual_.assign(static_cast<std::size_t>(size()), 0.0);
}

bool KktSystem::factor(const NtScaling& scaling) {
    auto& values = matrix_.values();
    std::size_t k = 0;
    for (int i = 0; i < cones_.nonnegative; ++i) values[scaling_positions_[k++]] = -scaling.orthant_squared(i);
    for (int c = 0; c < static_cast<int>(cones_.second_order.size()); ++c) {
        for (const double v : scaling.cone_squared(c)) values[scaling_positions_[k++]] = -v;
    }
    return ldl_.factor(matrix_, sign_);
}

void KktSystem::solve(std::span<const double> rhs, std::span<double> solution) {
    std::copy(rhs.begin(), rhs.end(), solution.begin());
    ldl_.solve(solution);
    const double tolerance = refinement_tolerance * (1.0 + norm_inf(rhs));
    for (int step = 0; step < refinement_steps_; ++step) {
        // residual = rhs - K0 u, K0 = matrix_ without the static regularisation.
        std::copy(rhs.begin(), rhs.end(), residual_.begin());
        matrix_.multiply_add(solution, residual_, -1.0);
        for (int j = 0; j < n_; ++j) residual_[j] += delta_ * solution[j];
        for (int i = 0; i < p_; ++i) residual_[n_ + i] -= delta_ * solution[n_ + i];
        if (norm_inf(residual_) <= tolerance) break;
        ldl_.solve(residual_);
        for (std::size_t i = 0; i < residual_.size(); ++i) solution[i] += residual_[i];
    }
}

}  // namespace starsim::optim
```

Add `src/optim/kkt_system.cpp` and `test_optim_kkt.cpp` to the CMake lists.

- [ ] **Step 4: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/optim/kkt_system.hpp core/src/optim/kkt_system.cpp core/tests/test_optim_kkt.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "Add KKT system with iterative refinement"
```

---

### Task 8: Interior-point solver (HSDE, Mehrotra predictor-corrector)

**Files:**
- Create: `core/include/starsim/optim/socp_solver.hpp`, `core/src/optim/socp_solver.cpp`
- Create: `core/tests/socp_test_support.hpp`, `core/tests/test_optim_solver.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`

**Interfaces:**
- Consumes: everything from Tasks 1-7.
- Produces: `SocpResult solve_socp(const SocpProblem& problem, const SocpSettings& settings = {})`; test helper `double constraint_violation(const SocpProblem&, const std::vector<double>& x)` in `core/tests/socp_test_support.hpp`.

Algorithm summary (implemented below; signs derived for this code, keep them together):
- Embedding residuals: `rx = A^T y + G^T z + c tau`, `ry = -A x + b tau`, `rz = s + G x - h tau`, `rtau = kappa + c^T x + b^T y + h^T z`; `mu = (s^T z + tau kappa) / (degree + 1)`.
- Newton system for weight `w` (1 predictor, `1 - sigma` corrector), cone right-hand side `d_s`, scalar `d_kappa`:
  `K [x2; y2; z2] = [-w rx; w ry; -w rz - W (lambda \ d_s)]`, `K [x1; y1; z1] = [-c; b; h]`,
  `dtau = (-w rtau - d_kappa / tau - (c^T x2 + b^T y2 + h^T z2)) / (c^T x1 + b^T y1 + h^T z1 - kappa / tau)`,
  `(dx, dy, dz) = (x2, y2, z2) + dtau (x1, y1, z1)`, `ds = W (lambda \ d_s - W dz)`, `dkappa = (d_kappa - kappa dtau) / tau`.
- Predictor: `d_s = -lambda o lambda`, `d_kappa = -tau kappa`. `sigma = (1 - alpha_aff)^3`.
- Corrector: `d_s = -lambda o lambda - (W^{-1} ds_a) o (W dz_a) + sigma mu e`, `d_kappa = -tau kappa - dtau_a dkappa_a + sigma mu`. Step `min(1, 0.99 alpha_max)`.
- Termination on the unscaled iterate: optimal if `primal_residual <= tol`, `dual_residual <= tol`, `gap <= tol * max(1, |c^T x|)`; infeasibility certificates only while `tau < kappa`.

- [ ] **Step 1: Write the test helper and the failing tests**

`core/tests/socp_test_support.hpp`:

```cpp
#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "starsim/optim/socp_problem.hpp"

namespace starsim::test {

// Largest constraint violation of x: equality residual and distance of h - G x outside the cone
// (minus its smallest eigenvalue).
inline double constraint_violation(const optim::SocpProblem& p, const std::vector<double>& x) {
    std::vector<double> ax(static_cast<std::size_t>(p.A.rows()), 0.0);
    p.A.multiply_add(x, ax);
    double v = 0.0;
    for (std::size_t i = 0; i < ax.size(); ++i) v = std::max(v, std::abs(ax[i] - p.b[i]));
    std::vector<double> s = p.h;
    p.G.multiply_add(x, s, -1.0);
    if (!s.empty()) v = std::max(v, -optim::min_eigenvalue(p.cones, s));
    return v;
}

}  // namespace starsim::test
```

`core/tests/test_optim_solver.cpp`:

```cpp
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
```

- [ ] **Step 2: Run to verify it fails**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests`
Expected: compile error, `socp_solver.hpp` not found.

- [ ] **Step 3: Implement** - `core/include/starsim/optim/socp_solver.hpp`

```cpp
#pragma once

#include "starsim/optim/socp_problem.hpp"

namespace starsim::optim {

// Primal-dual interior-point method on the homogeneous self-dual embedding with Mehrotra predictor-corrector
// steps and Nesterov-Todd scaling (after ECOS: Domahidi, Chu, Boyd 2013; Vandenberghe 2010). The data is
// Ruiz-equilibrated internally; residuals and the result refer to the original problem. Deterministic and
// single-threaded.
SocpResult solve_socp(const SocpProblem& problem, const SocpSettings& settings = {});

}  // namespace starsim::optim
```

`core/src/optim/socp_solver.cpp`:

```cpp
#include "starsim/optim/socp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "starsim/optim/equilibration.hpp"
#include "starsim/optim/kkt_system.hpp"

namespace starsim::optim {

namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double step_fraction = 0.99;  // of the distance to the cone boundary
constexpr double minimum_step = 1e-10;  // below: the iteration has stalled

struct Direction {
    std::vector<double> x, y, z, s;
    double tau = 0.0;
    double kappa = 0.0;
};

class InteriorPointSolver {
public:
    InteriorPointSolver(const SocpProblem& problem, const SocpSettings& settings)
        : original_(problem),
          settings_(settings),
          scaled_(problem),
          equilibration_(equilibrate(scaled_, settings.equilibration_passes)),
          n_(scaled_.variables),
          p_(scaled_.A.rows()),
          m_(scaled_.G.rows()),
          kkt_(scaled_.A, scaled_.G, scaled_.cones, settings.static_regularization, settings.refinement_steps),
          scaling_(scaled_.cones) {
        const auto n = static_cast<std::size_t>(n_);
        const auto p = static_cast<std::size_t>(p_);
        const auto m = static_cast<std::size_t>(m_);
        const auto total = n + p + m;
        x_.assign(n, 0.0);
        y_.assign(p, 0.0);
        z_.assign(m, 0.0);
        s_.assign(m, 0.0);
        rx_.assign(n, 0.0);
        ry_.assign(p, 0.0);
        rz_.assign(m, 0.0);
        rhs_.assign(total, 0.0);
        solution_.assign(total, 0.0);
        tau_column_.assign(total, 0.0);
        identity_.assign(m, 0.0);
        add_identity(scaled_.cones, 1.0, identity_);
        lambda_square_.assign(m, 0.0);
        d_s_.assign(m, 0.0);
        division_.assign(m, 0.0);
        work_a_.assign(m, 0.0);
        work_b_.assign(m, 0.0);
        for (Direction* d : {&affine_, &combined_}) {
            d->x.assign(n, 0.0);
            d->y.assign(p, 0.0);
            d->z.assign(m, 0.0);
            d->s.assign(m, 0.0);
        }
        xo_.assign(n, 0.0);
        yo_.assign(p, 0.0);
        zo_.assign(m, 0.0);
        so_.assign(m, 0.0);
        ax_.assign(p, 0.0);
        gx_.assign(m, 0.0);
        aty_.assign(n, 0.0);
    }

    SocpResult run() {
        SocpResult result;
        if (!initialize()) return finish(result, SocpStatus::NumericalError);
        const double degree = scaled_.cones.degree();
        for (int iteration = 0;; ++iteration) {
            result.iterations = iteration;
            compute_residuals();
            if (check_termination(result)) return result;
            if (iteration >= settings_.max_iterations) return finish(result, SocpStatus::MaxIterations);
            if (!scaling_.update(s_, z_) || !kkt_.factor(scaling_)) return finish(result, SocpStatus::NumericalError);

            // Column of tau: K [x1; y1; z1] = [-c; b; h].
            fill_rhs([&](int j) { return -scaled_.c[j]; }, [&](int i) { return scaled_.b[i]; },
                     [&](int i) { return scaled_.h[i]; });
            kkt_.solve(rhs_, tau_column_);
            const double mu = (dot(s_, z_) + tau_ * kappa_) / (degree + 1.0);

            // Predictor (affine scaling) direction.
            cone_product(scaled_.cones, scaling_.lambda(), scaling_.lambda(), lambda_square_);
            for (int i = 0; i < m_; ++i) d_s_[i] = -lambda_square_[i];
            solve_direction(1.0, -tau_ * kappa_, affine_);
            const double alpha_affine = std::min(1.0, max_step_length(affine_));
            const double sigma = std::pow(1.0 - alpha_affine, 3);

            // Corrector: centring plus the second-order term of the affine direction.
            scaling_.apply_inverse(affine_.s, work_a_);
            scaling_.apply(affine_.z, work_b_);
            cone_product(scaled_.cones, work_a_, work_b_, d_s_);
            for (int i = 0; i < m_; ++i) d_s_[i] = -lambda_square_[i] - d_s_[i] + sigma * mu * identity_[i];
            solve_direction(1.0 - sigma, -tau_ * kappa_ - affine_.tau * affine_.kappa + sigma * mu, combined_);
            const double alpha = std::min(1.0, step_fraction * max_step_length(combined_));
            if (settings_.verbose) {
                std::fprintf(stderr, "%3d  pres %9.2e  dres %9.2e  gap %9.2e  sigma %5.3f  alpha %5.3f\n", iteration,
                             result.primal_residual, result.dual_residual, result.gap, sigma, alpha);
            }
            if (alpha < minimum_step) return finish(result, SocpStatus::NumericalError);
            take_step(alpha, combined_);
        }
    }

private:
    template <class FX, class FY, class FZ>
    void fill_rhs(FX fx, FY fy, FZ fz) {
        for (int j = 0; j < n_; ++j) rhs_[j] = fx(j);
        for (int i = 0; i < p_; ++i) rhs_[n_ + i] = fy(i);
        for (int i = 0; i < m_; ++i) rhs_[n_ + p_ + i] = fz(i);
    }

    std::span<const double> part(const std::vector<double>& v, int offset, int size) const {
        return std::span<const double>(v).subspan(static_cast<std::size_t>(offset), static_cast<std::size_t>(size));
    }

    // Shifts u into the interior: unchanged if already inside, else u + (1 - min eigenvalue) e.
    void shift_into_cone(std::vector<double>& u) const {
        const double alpha = -min_eigenvalue(scaled_.cones, u);
        if (alpha >= 0.0) add_identity(scaled_.cones, 1.0 + alpha, u);
    }

    // Least-squares starting point (as ECOS/CVXOPT) with W = I.
    bool initialize() {
        if (!scaling_.update(identity_, identity_) || !kkt_.factor(scaling_)) return false;
        fill_rhs([](int) { return 0.0; }, [&](int i) { return scaled_.b[i]; }, [&](int i) { return scaled_.h[i]; });
        kkt_.solve(rhs_, solution_);
        for (int j = 0; j < n_; ++j) x_[j] = solution_[j];
        for (int i = 0; i < m_; ++i) s_[i] = -solution_[n_ + p_ + i];
        shift_into_cone(s_);
        fill_rhs([&](int j) { return -scaled_.c[j]; }, [](int) { return 0.0; }, [](int) { return 0.0; });
        kkt_.solve(rhs_, solution_);
        for (int i = 0; i < p_; ++i) y_[i] = solution_[n_ + i];
        for (int i = 0; i < m_; ++i) z_[i] = solution_[n_ + p_ + i];
        shift_into_cone(z_);
        tau_ = 1.0;
        kappa_ = 1.0;
        return true;
    }

    void compute_residuals() {
        for (int j = 0; j < n_; ++j) rx_[j] = scaled_.c[j] * tau_;
        scaled_.A.multiply_transpose_add(y_, rx_);
        scaled_.G.multiply_transpose_add(z_, rx_);
        for (int i = 0; i < p_; ++i) ry_[i] = scaled_.b[i] * tau_;
        scaled_.A.multiply_add(x_, ry_, -1.0);
        for (int i = 0; i < m_; ++i) rz_[i] = s_[i] - scaled_.h[i] * tau_;
        scaled_.G.multiply_add(x_, rz_);
        rtau_ = kappa_ + dot(scaled_.c, x_) + dot(scaled_.b, y_) + dot(scaled_.h, z_);
    }

    // Newton direction of the embedding (see the plan / file header for the derivation).
    void solve_direction(double weight, double d_kappa, Direction& d) {
        cone_division(scaled_.cones, scaling_.lambda(), d_s_, division_);  // lambda \ d_s
        scaling_.apply(division_, work_a_);                                 // W (lambda \ d_s)
        fill_rhs([&](int j) { return -weight * rx_[j]; }, [&](int i) { return weight * ry_[i]; },
                 [&](int i) { return -weight * rz_[i] - work_a_[i]; });
        kkt_.solve(rhs_, solution_);
        const auto x2 = part(solution_, 0, n_), y2 = part(solution_, n_, p_), z2 = part(solution_, n_ + p_, m_);
        const auto x1 = part(tau_column_, 0, n_), y1 = part(tau_column_, n_, p_), z1 = part(tau_column_, n_ + p_, m_);
        const double numerator = -weight * rtau_ - d_kappa / tau_ -
                                 (dot(scaled_.c, x2) + dot(scaled_.b, y2) + dot(scaled_.h, z2));
        const double denominator = dot(scaled_.c, x1) + dot(scaled_.b, y1) + dot(scaled_.h, z1) - kappa_ / tau_;
        d.tau = numerator / denominator;
        for (int j = 0; j < n_; ++j) d.x[j] = x2[j] + d.tau * x1[j];
        for (int i = 0; i < p_; ++i) d.y[i] = y2[i] + d.tau * y1[i];
        for (int i = 0; i < m_; ++i) d.z[i] = z2[i] + d.tau * z1[i];
        d.kappa = (d_kappa - kappa_ * d.tau) / tau_;
        scaling_.apply(d.z, work_b_);                                // W dz
        for (int i = 0; i < m_; ++i) work_b_[i] = division_[i] - work_b_[i];
        scaling_.apply(work_b_, d.s);                                // ds = W (lambda \ d_s - W dz)
    }

    double max_step_length(const Direction& d) const {
        double alpha = std::min(max_step(scaled_.cones, s_, d.s), max_step(scaled_.cones, z_, d.z));
        if (d.tau < 0.0) alpha = std::min(alpha, -tau_ / d.tau);
        if (d.kappa < 0.0) alpha = std::min(alpha, -kappa_ / d.kappa);
        return alpha;
    }

    void take_step(double alpha, const Direction& d) {
        for (int j = 0; j < n_; ++j) x_[j] += alpha * d.x[j];
        for (int i = 0; i < p_; ++i) y_[i] += alpha * d.y[i];
        for (int i = 0; i < m_; ++i) {
            z_[i] += alpha * d.z[i];
            s_[i] += alpha * d.s[i];
        }
        tau_ += alpha * d.tau;
        kappa_ += alpha * d.kappa;
    }

    // Unscaled copy of the iterate (still multiplied by tau).
    void unscaled_iterate() {
        std::copy(x_.begin(), x_.end(), xo_.begin());
        std::copy(y_.begin(), y_.end(), yo_.begin());
        std::copy(z_.begin(), z_.end(), zo_.begin());
        std::copy(s_.begin(), s_.end(), so_.begin());
        unscale(equilibration_, xo_, yo_, zo_, so_);
    }

    bool check_termination(SocpResult& result) {
        unscaled_iterate();
        const auto& o = original_;
        std::fill(ax_.begin(), ax_.end(), 0.0);
        o.A.multiply_add(xo_, ax_);
        std::copy(so_.begin(), so_.end(), gx_.begin());
        o.G.multiply_add(xo_, gx_);  // G x + s
        std::fill(aty_.begin(), aty_.end(), 0.0);
        o.A.multiply_transpose_add(yo_, aty_);
        o.G.multiply_transpose_add(zo_, aty_);  // A^T y + G^T z

        double primal_a = 0.0, primal_g = 0.0, dual = 0.0;
        for (int i = 0; i < p_; ++i) primal_a += std::pow(ax_[i] - o.b[i] * tau_, 2);
        for (int i = 0; i < m_; ++i) primal_g += std::pow(gx_[i] - o.h[i] * tau_, 2);
        for (int j = 0; j < n_; ++j) dual += std::pow(aty_[j] + o.c[j] * tau_, 2);
        result.primal_residual =
            std::max(std::sqrt(primal_a) / (1.0 + norm2(o.b)), std::sqrt(primal_g) / (1.0 + norm2(o.h))) / tau_;
        result.dual_residual = std::sqrt(dual) / (1.0 + norm2(o.c)) / tau_;
        result.gap = dot(so_, zo_) / (tau_ * tau_);
        const double cx = dot(o.c, xo_);
        const double primal_cost = cx / tau_;
        const double tol = settings_.tolerance;

        if (result.primal_residual <= tol && result.dual_residual <= tol &&
            result.gap <= tol * std::max(1.0, std::abs(primal_cost))) {
            store(result, 1.0 / tau_);
            result.objective = primal_cost;
            result.status = SocpStatus::Optimal;
            return true;
        }
        if (tau_ < kappa_) {
            const double by_hz = dot(o.b, yo_) + dot(o.h, zo_);
            if (by_hz < 0.0 && norm2(aty_) / -by_hz <= tol) {
                store(result, 1.0 / -by_hz);
                result.objective = infinity;
                result.status = SocpStatus::PrimalInfeasible;
                return true;
            }
            if (cx < 0.0 && std::max(norm2(ax_), norm2(gx_)) / -cx <= tol) {
                store(result, 1.0 / -cx);
                result.objective = -infinity;
                result.status = SocpStatus::DualInfeasible;
                return true;
            }
        }
        return false;
    }

    void store(SocpResult& result, double factor) const {
        result.x.resize(xo_.size());
        result.y.resize(yo_.size());
        result.z.resize(zo_.size());
        result.s.resize(so_.size());
        for (std::size_t j = 0; j < xo_.size(); ++j) result.x[j] = xo_[j] * factor;
        for (std::size_t i = 0; i < yo_.size(); ++i) result.y[i] = yo_[i] * factor;
        for (std::size_t i = 0; i < zo_.size(); ++i) {
            result.z[i] = zo_[i] * factor;
            result.s[i] = so_[i] * factor;
        }
    }

    SocpResult& finish(SocpResult& result, SocpStatus status) {
        unscaled_iterate();
        store(result, 1.0 / tau_);
        result.objective = dot(original_.c, result.x);
        result.status = status;
        return result;
    }

    const SocpProblem& original_;
    SocpSettings settings_;
    SocpProblem scaled_;
    Equilibration equilibration_;
    int n_, p_, m_;
    KktSystem kkt_;
    NtScaling scaling_;

    std::vector<double> x_, y_, z_, s_;
    double tau_ = 1.0;
    double kappa_ = 1.0;
    std::vector<double> rx_, ry_, rz_;
    double rtau_ = 0.0;
    std::vector<double> rhs_, solution_, tau_column_;
    std::vector<double> identity_, lambda_square_, d_s_, division_, work_a_, work_b_;
    Direction affine_, combined_;
    std::vector<double> xo_, yo_, zo_, so_, ax_, gx_, aty_;
};

}  // namespace

SocpResult solve_socp(const SocpProblem& problem, const SocpSettings& settings) {
    problem.validate();
    InteriorPointSolver solver(problem, settings);
    return solver.run();
}

}  // namespace starsim::optim
```

Add `src/optim/socp_solver.cpp` and `test_optim_solver.cpp` to the CMake lists.

- [ ] **Step 4: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: PASS. If a case fails, rerun it with `SocpSettings{.verbose = true}` (temporarily in the test) to see the iteration table; do not loosen tolerances - find the defect (typical: a sign in `solve_direction` or the residual definitions, which must match the summary above).

- [ ] **Step 5: Commit**

```bash
git add core/include/starsim/optim/socp_solver.hpp core/src/optim/socp_solver.cpp core/tests/socp_test_support.hpp core/tests/test_optim_solver.cpp core/CMakeLists.txt core/tests/CMakeLists.txt
git commit -m "Add homogeneous self-dual interior-point SOCP solver"
```

---

### Task 9: Reference fixtures (Clarabel) and validation tests

**Files:**
- Create: `tools/validation/gen_socp_fixtures.py`
- Create (generated, committed): `tests/validation/socp/{lp_small,socp_small,socp_medium,socp_large,portfolio,landing_gfold}.txt`
- Create: `core/tests/test_optim_validation.cpp`
- Modify: `core/tests/CMakeLists.txt` (test file + `STARSIM_SOCP_FIXTURE_DIR="${CMAKE_SOURCE_DIR}/tests/validation/socp"` in `target_compile_definitions`)

**Interfaces:**
- Consumes: `solve_socp`, `SocpProblem`, `constraint_violation`.
- Produces: fixture text format (one record per key, whitespace separated):
  ```
  # comment line
  variables <n>
  nonnegative <l>
  second_order <count> <q1> ... <qk>
  c <n> <values...>
  A <rows> <cols> <nnz>  followed by nnz lines "<row> <col> <value>"
  b <rows> <values...>
  G <rows> <cols> <nnz>  followed by nnz lines
  h <rows> <values...>
  objective <value>
  x <n> <values...>
  unique <0|1>
  ```

- [ ] **Step 1: Install the offline tools**

Run: `python -m pip install cvxpy clarabel numpy scipy`
Expected: installs without error (Apache-2.0 / BSD; used only to generate fixtures, not part of the build).

- [ ] **Step 2: Write the generator** - `tools/validation/gen_socp_fixtures.py`

```python
"""Generates reference SOCP problems for the optim module (core/src/optim) and solves them with Clarabel.

Writes tests/validation/socp/<name>.txt: the problem in our standard form
    min c'x  s.t.  A x = b,  G x + s = h,  s in R+^l x Q^{q1} x ...
plus Clarabel's solution. Offline tool only, not part of the build: CVXPY (Apache-2.0) builds the
landing and portfolio problems, Clarabel (Apache-2.0) solves all of them.

    python -m pip install cvxpy clarabel numpy scipy
    python tools/validation/gen_socp_fixtures.py
"""
import pathlib

import clarabel
import cvxpy as cp
import numpy as np
import scipy.sparse as sp

OUT = pathlib.Path(__file__).resolve().parents[2] / "tests" / "validation" / "socp"


class Problem:
    def __init__(self, c, A, b, G, h, l, socs):
        self.c, self.A, self.b, self.G, self.h = np.asarray(c, float), sp.csc_matrix(A), np.asarray(b, float), sp.csc_matrix(G), np.asarray(h, float)
        self.l, self.socs = int(l), [int(q) for q in socs]


def cone_point(rng, l, socs):
    """Strictly interior point of R+^l x Q^{q1} x ..."""
    parts = [rng.uniform(0.5, 2.0, l)]
    for q in socs:
        tail = rng.standard_normal(q - 1)
        parts.append(np.concatenate([[np.linalg.norm(tail) + rng.uniform(0.5, 2.0)], tail]))
    return np.concatenate(parts)


def sparse_with_full_rows(rng, rows, cols, density):
    m = sp.random(rows, cols, density=density, random_state=rng, format="lil")
    for i in range(rows):
        m[i, i % cols] = rng.standard_normal() + 2.0
    return m.tocsc()


def random_problem(seed, n, p, l, socs):
    """Primal and dual strictly feasible by construction, hence a finite optimum."""
    rng = np.random.default_rng(seed)
    m = l + sum(socs)
    A = sparse_with_full_rows(rng, p, n, 0.3)
    G = sparse_with_full_rows(rng, m, n, 0.3)
    x0, y0 = rng.standard_normal(n), rng.standard_normal(p)
    s0, z0 = cone_point(rng, l, socs), cone_point(rng, l, socs)
    return Problem(-(A.T @ y0 + G.T @ z0), A, A @ x0, G, G @ x0 + s0, l, socs)


def from_cvxpy(problem):
    data, _, _ = problem.get_problem_data(cp.CLARABEL)
    dims = data["dims"]
    if dims.exp or dims.psd or getattr(dims, "p3d", []):
        raise RuntimeError("only zero, nonnegative and second-order cones are supported")
    A, b, z = sp.csc_matrix(data["A"]), np.asarray(data["b"], float), dims.zero
    return Problem(data["c"], A[:z], b[:z], A[z:], b[z:], dims.nonneg, list(dims.soc))


def portfolio_problem():
    rng = np.random.default_rng(5)
    n, k = 50, 5
    mu, F = rng.uniform(0.0, 0.2, n), rng.standard_normal((n, k)) * 0.1
    d = rng.uniform(0.01, 0.05, n)
    w = cp.Variable(n)
    risk = cp.norm(cp.hstack([F.T @ w, cp.multiply(d, w)]))
    return from_cvxpy(cp.Problem(cp.Maximize(mu @ w), [cp.sum(w) == 1, w >= 0, risk <= 0.05]))


def landing_problem():
    """3-DOF powered descent, lossless convexification (Acikmese & Ploen 2007), size of the in-flight
    guidance problem: 40 nodes, thrust-rate limit, tilt and glide-slope cones. Numbers: Super Heavy on its
    three centre engines (estimates)."""
    N, tf = 40, 18.0
    dt = tf / (N - 1)
    g = np.array([0.0, 0.0, -9.80665])
    m0, t_max = 2.6e5, 6.9e6
    t_min = 0.4 * t_max
    alpha = 1.0 / (330.0 * 9.80665)
    r0, v0 = np.array([200.0, -100.0, 500.0]), np.array([-10.0, 5.0, -60.0])
    tilt, glide, du_max = np.radians(15.0), np.radians(20.0), 3.0
    r, v, u = cp.Variable((3, N)), cp.Variable((3, N)), cp.Variable((3, N))
    z, s = cp.Variable(N), cp.Variable(N)
    cons = [r[:, 0] == r0, v[:, 0] == v0, z[0] == np.log(m0), r[:, -1] == 0, v[:, -1] == 0]
    for k in range(N - 1):
        cons += [
            v[:, k + 1] == v[:, k] + dt / 2 * (u[:, k] + u[:, k + 1]) + dt * g,
            r[:, k + 1] == r[:, k] + dt / 2 * (v[:, k] + v[:, k + 1]) + dt**2 / 12 * (u[:, k + 1] - u[:, k]),
            z[k + 1] == z[k] - alpha * dt / 2 * (s[k] + s[k + 1]),
            cp.norm(u[:, k + 1] - u[:, k]) <= du_max * dt,
        ]
    for k in range(N):
        t = k * dt
        z_ref = np.log(m0 - alpha * t_max * t)
        mu1, mu2 = t_min * np.exp(-z_ref), t_max * np.exp(-z_ref)
        cons += [
            cp.norm(u[:, k]) <= s[k],
            s[k] >= mu1 * (1 - (z[k] - z_ref) + cp.square(z[k] - z_ref) / 2),
            s[k] <= mu2 * (1 - (z[k] - z_ref)),
            z[k] >= z_ref,
            z[k] <= np.log(m0 - alpha * t_min * t),
            u[2, k] >= s[k] * np.cos(tilt),
            cp.norm(r[0:2, k]) * np.tan(glide) <= r[2, k],
        ]
    return from_cvxpy(cp.Problem(cp.Maximize(z[-1]), cons))


def solve_clarabel(prob):
    n, p = len(prob.c), prob.A.shape[0]
    cones = []
    if p:
        cones.append(clarabel.ZeroConeT(p))
    if prob.l:
        cones.append(clarabel.NonnegativeConeT(prob.l))
    cones += [clarabel.SecondOrderConeT(q) for q in prob.socs]
    settings = clarabel.DefaultSettings()
    settings.verbose = False
    settings.tol_gap_abs = settings.tol_gap_rel = settings.tol_feas = 1e-10
    solver = clarabel.DefaultSolver(sp.csc_matrix((n, n)), prob.c, sp.vstack([prob.A, prob.G]).tocsc(),
                                    np.concatenate([prob.b, prob.h]), cones, settings)
    solution = solver.solve()
    if solution.status != clarabel.SolverStatus.Solved:
        raise RuntimeError(f"Clarabel: {solution.status}")
    return np.array(solution.x), float(prob.c @ np.array(solution.x))


def write(name, prob, unique):
    x, objective = solve_clarabel(prob)
    OUT.mkdir(parents=True, exist_ok=True)

    def vec(key, values):
        return f"{key} {len(values)} " + " ".join(repr(float(v)) for v in values) + "\n"

    def mat(key, m):
        coo = m.tocoo()
        lines = [f"{key} {m.shape[0]} {m.shape[1]} {coo.nnz}\n"]
        lines += [f"{i} {j} {float(v)!r}\n" for i, j, v in zip(coo.row, coo.col, coo.data)]
        return "".join(lines)

    with open(OUT / f"{name}.txt", "w", newline="\n") as f:
        f.write(f"# {name}: generated by tools/validation/gen_socp_fixtures.py, reference Clarabel {clarabel.__version__}\n")
        f.write(f"variables {len(prob.c)}\n")
        f.write(f"nonnegative {prob.l}\n")
        f.write(f"second_order {len(prob.socs)}" + "".join(f" {q}" for q in prob.socs) + "\n")
        f.write(vec("c", prob.c) + mat("A", prob.A) + vec("b", prob.b) + mat("G", prob.G) + vec("h", prob.h))
        f.write(f"objective {objective!r}\n")
        f.write(vec("x", x))
        f.write(f"unique {int(unique)}\n")
    print(f"{name}: n={len(prob.c)} p={prob.A.shape[0]} m={prob.G.shape[0]} objective={objective:.10g}")


def main():
    write("lp_small", random_problem(1, n=10, p=3, l=20, socs=[]), unique=True)
    write("socp_small", random_problem(2, n=10, p=2, l=5, socs=[3, 4, 5]), unique=True)
    write("socp_medium", random_problem(3, n=100, p=20, l=60, socs=[3] * 20 + [5] * 10), unique=True)
    write("socp_large", random_problem(4, n=200, p=30, l=150, socs=[4] * 40), unique=True)
    write("portfolio", portfolio_problem(), unique=False)
    write("landing_gfold", landing_problem(), unique=False)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: Generate the fixtures**

Run: `python tools/validation/gen_socp_fixtures.py`
Expected: six lines `name: n=... objective=...`; files in `tests/validation/socp/`. If Clarabel reports the landing problem infeasible, lengthen `tf` (e.g. 20 s) and regenerate - the problem must be feasible, not changed otherwise.

- [ ] **Step 4: Write the validation tests** - `core/tests/test_optim_validation.cpp`

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "socp_test_support.hpp"
#include "starsim/optim/socp_solver.hpp"

using namespace starsim::optim;
using starsim::test::constraint_violation;

namespace {

struct SocpFixture {
    SocpProblem problem;
    double objective = 0.0;
    std::vector<double> x;
    bool unique = false;
};

SocpFixture read_socp_fixture(const std::string& name) {
    std::ifstream in(std::string(STARSIM_SOCP_FIXTURE_DIR) + "/" + name + ".txt");
    if (!in) throw std::runtime_error("missing fixture " + name + " (run tools/validation/gen_socp_fixtures.py)");
    SocpFixture f;
    auto read_vector = [&](std::vector<double>& v) {
        int size = 0;
        in >> size;
        v.resize(static_cast<std::size_t>(size));
        for (auto& e : v) in >> e;
    };
    auto read_matrix = [&](CscMatrix& m) {
        int rows = 0, cols = 0, nnz = 0;
        in >> rows >> cols >> nnz;
        std::vector<Triplet> t(static_cast<std::size_t>(nnz));
        for (auto& e : t) in >> e.row >> e.col >> e.value;
        m = CscMatrix(rows, cols, t);
    };
    std::string key;
    while (in >> key) {
        if (key[0] == '#') {
            std::getline(in, key);
        } else if (key == "variables") {
            in >> f.problem.variables;
        } else if (key == "nonnegative") {
            in >> f.problem.cones.nonnegative;
        } else if (key == "second_order") {
            int count = 0;
            in >> count;
            f.problem.cones.second_order.resize(static_cast<std::size_t>(count));
            for (auto& q : f.problem.cones.second_order) in >> q;
        } else if (key == "c") {
            read_vector(f.problem.c);
        } else if (key == "A") {
            read_matrix(f.problem.A);
        } else if (key == "b") {
            read_vector(f.problem.b);
        } else if (key == "G") {
            read_matrix(f.problem.G);
        } else if (key == "h") {
            read_vector(f.problem.h);
        } else if (key == "objective") {
            in >> f.objective;
        } else if (key == "x") {
            read_vector(f.x);
        } else if (key == "unique") {
            int u = 0;
            in >> u;
            f.unique = u != 0;
        } else {
            throw std::runtime_error("unknown fixture key " + key);
        }
    }
    return f;
}

double data_scale(const SocpProblem& p) { return 1.0 + std::max(norm_inf(p.b), norm_inf(p.h)); }

}  // namespace

TEST_CASE("SOCP solver matches Clarabel on reference problems", "[optim][validation]") {
    const std::string name = GENERATE(as<std::string>{}, "lp_small", "socp_small", "socp_medium", "socp_large",
                                      "portfolio", "landing_gfold");
    INFO(name);
    const auto f = read_socp_fixture(name);
    const auto r = solve_socp(f.problem);
    INFO("status " << to_string(r.status) << ", iterations " << r.iterations);
    REQUIRE(r.status == SocpStatus::Optimal);
    CHECK(std::abs(r.objective - f.objective) <= 1e-6 * std::max(1.0, std::abs(f.objective)));
    CHECK(constraint_violation(f.problem, r.x) <= 1e-6 * data_scale(f.problem));
    if (f.unique) {
        double diff = 0.0;
        for (std::size_t i = 0; i < r.x.size(); ++i) diff += std::pow(r.x[i] - f.x[i], 2);
        CHECK(std::sqrt(diff) / std::max(1.0, norm2(f.x)) <= 1e-5);
    }
}

TEST_CASE("SOCP solver is deterministic on the landing problem", "[optim][validation]") {
    const auto f = read_socp_fixture("landing_gfold");
    const auto a = solve_socp(f.problem);
    const auto b = solve_socp(f.problem);
    CHECK(a.x == b.x);
    CHECK(a.iterations == b.iterations);
}

TEST_CASE("Landing-size SOCP converges in few iterations", "[optim][validation]") {
    const auto f = read_socp_fixture("landing_gfold");
    const auto start = std::chrono::steady_clock::now();
    const auto r = solve_socp(f.problem);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("landing_gfold: n=%d m=%d, %d iterations, %.1f ms\n", f.problem.variables, f.problem.G.rows(),
                    r.iterations, ms);
    }
    REQUIRE(r.status == SocpStatus::Optimal);
    CHECK(r.iterations < 30);
}
```

Add `test_optim_validation.cpp` and the `STARSIM_SOCP_FIXTURE_DIR` definition to `core/tests/CMakeLists.txt`.

- [ ] **Step 5: Run tests**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -NoTests; $env:STARSIM_VERBOSE=1; build/core/tests/starsim_core_tests.exe "[optim]"`
Expected: all `[optim]` tests pass, landing timing printed (target < 50 ms in Release; reported, not asserted). If a random fixture with `unique=True` fails only the solution-distance check while objective and feasibility pass, the optimum is degenerate: set `unique=False` for that case in the generator, regenerate, and note it in the commit message.

- [ ] **Step 6: Commit**

```bash
git add tools/validation/gen_socp_fixtures.py tests/validation/socp core/tests/test_optim_validation.cpp core/tests/CMakeLists.txt
git commit -m "Validate the SOCP solver against Clarabel reference problems"
```

---

### Task 10: Documentation and full verification

**Files:**
- Modify: `CLAUDE.md` (module status, fixture regeneration)
- Modify: `docs/superpowers/specs/2026-10-12-socp-solver-design.md` (record the planning deviations)

- [ ] **Step 1: Update `CLAUDE.md`**

In the architecture section, extend the "Implemented so far" sentence with `optim (sparse SOCP interior-point solver)`, and add `optim` to the module list:
```
- `core` is split into small modules with clear interfaces: `math`, `time`, `frames`, `ephemeris`,
  `environment`, `integrators`, `optim`, `orbit`, `vehicle`, `propulsion`, `aero`, `contact`, `gnc`, `mission`, `sim`.
```
In "Build & test", below the Horizons line, add:
```
- SOCP reference fixtures (`tests/validation/socp/`) are regenerated with `python tools/validation/gen_socp_fixtures.py`
  (needs cvxpy + clarabel; offline tool only, committed output).
```
Add to "Known issues":
```
- `optim` (SOCP solver, ECOS-style HSDE): dense Nesterov-Todd blocks per cone, so cones should stay small
  (dimension <= ~20); ordering is minimum degree on the explicit elimination graph (not full AMD).
```

- [ ] **Step 2: Record the deviations in the spec**

In `docs/superpowers/specs/2026-10-12-socp-solver-design.md`:
- "Ordering": replace the AMD sentence with "minimum degree on the explicit elimination graph (ties: lowest index), computed once in the symbolic phase; full AMD is not needed for banded trajectory problems." Rename `optim/amd.hpp/.cpp` to `optim/minimum_degree.hpp/.cpp` in the file table; add `optim/socp_problem.cpp`.
- "Linear algebra": static regularisation 7e-8, dynamic regularisation epsilon 1e-13 / delta 2e-7 (ECOS values), up to 8 refinement steps.
- `SocpBuilder`: `add_equality(expr)` means `expr == 0`.
- "Reference fixtures": `tests/validation/socp/*.txt` in the key/value text format of `gen_socp_fixtures.py` (no JSON parser in the core).

- [ ] **Step 3: Full verification**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1`
Expected: core + GDExtension build, ctest `100% tests passed` (72 existing + the new `[optim]` cases).

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md docs/superpowers/specs/2026-10-12-socp-solver-design.md
git commit -m "Document the optim module"
```
