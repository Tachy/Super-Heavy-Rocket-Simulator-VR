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
