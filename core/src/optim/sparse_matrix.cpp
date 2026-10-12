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
