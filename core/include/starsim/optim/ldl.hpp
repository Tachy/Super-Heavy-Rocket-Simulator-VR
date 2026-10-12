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
