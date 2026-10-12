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

constexpr double reference_tolerance = 1e-10;  // Clarabel's tolerances in gen_socp_fixtures.py

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
    // The references were solved to 1e-10 (gen_socp_fixtures.py). Near cone boundaries the solution vector is
    // only accurate to about the square root of the stopping tolerance, so compare at the same tolerance.
    SocpSettings settings;
    settings.tolerance = reference_tolerance;
    const auto r = solve_socp(f.problem, settings);
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
