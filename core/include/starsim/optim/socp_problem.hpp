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
