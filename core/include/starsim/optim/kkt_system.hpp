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
    // New values of A and G with the patterns given to the constructor.
    void set_values(const CscMatrix& a, const CscMatrix& g);
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
    std::vector<int> a_positions_, g_positions_;  // value index of each A / G entry, then of its transpose
    SparseLdl ldl_;
    std::vector<double> residual_;
};

}  // namespace starsim::optim
