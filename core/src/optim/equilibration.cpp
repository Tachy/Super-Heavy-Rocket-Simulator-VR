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
