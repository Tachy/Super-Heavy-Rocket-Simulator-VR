#include "starsim/optim/kkt_system.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "starsim/optim/minimum_degree.hpp"

namespace starsim::optim {

namespace {

// Relative residual below which iterative refinement stops early.
constexpr double refinement_tolerance = 1e-14;
// Refinement stops once a step no longer shrinks the residual. (Clarabel's ratio 5 stopped after one step here,
// where the residual shrinks only ~0.5 per step: the landing fixture then needed 50 instead of 17 iterations.)
constexpr double refinement_stop_ratio = 1.0;

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
    for (int j = 0; j < n_; ++j) {
        for (int q = a.col_start()[j]; q < a.col_start()[j + 1]; ++q) {
            a_positions_.push_back(value_position(matrix_, n_ + a.row_index()[q], j));
            a_positions_.push_back(value_position(matrix_, j, n_ + a.row_index()[q]));
        }
        for (int q = g.col_start()[j]; q < g.col_start()[j + 1]; ++q) {
            g_positions_.push_back(value_position(matrix_, z0 + g.row_index()[q], j));
            g_positions_.push_back(value_position(matrix_, j, z0 + g.row_index()[q]));
        }
    }

    sign_.assign(static_cast<std::size_t>(size()), -1);
    std::fill(sign_.begin(), sign_.begin() + n_, 1);
    ldl_.analyze(matrix_, minimum_degree_ordering(matrix_));
    residual_.assign(static_cast<std::size_t>(size()), 0.0);
}

void KktSystem::set_values(const CscMatrix& a, const CscMatrix& g) {
    auto& values = matrix_.values();
    for (size_t k = 0; k < a.values().size(); ++k) values[a_positions_[2 * k]] = values[a_positions_[2 * k + 1]] = a.values()[k];
    for (size_t k = 0; k < g.values().size(); ++k) values[g_positions_[2 * k]] = values[g_positions_[2 * k + 1]] = g.values()[k];
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
    double previous = std::numeric_limits<double>::infinity();
    for (int step = 0; step < refinement_steps_; ++step) {
        // residual = rhs - K0 u, K0 = matrix_ without the static regularisation.
        std::copy(rhs.begin(), rhs.end(), residual_.begin());
        matrix_.multiply_add(solution, residual_, -1.0);
        for (int j = 0; j < n_; ++j) residual_[j] += delta_ * solution[j];
        for (int i = 0; i < p_; ++i) residual_[n_ + i] -= delta_ * solution[n_ + i];
        const double size = norm_inf(residual_);
        if (size <= tolerance || size > previous / refinement_stop_ratio) break;
        previous = size;
        ldl_.solve(residual_);
        for (std::size_t i = 0; i < residual_.size(); ++i) solution[i] += residual_[i];
    }
}

}  // namespace starsim::optim
