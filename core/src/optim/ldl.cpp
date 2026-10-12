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
