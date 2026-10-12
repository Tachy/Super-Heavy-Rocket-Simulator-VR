#pragma once

#include <memory>

#include "starsim/optim/socp_problem.hpp"

namespace starsim::optim {

// Primal-dual interior-point method on the homogeneous self-dual embedding with Mehrotra predictor-corrector
// steps and Nesterov-Todd scaling (after ECOS: Domahidi, Chu, Boyd 2013; Vandenberghe 2010). The data is
// Ruiz-equilibrated internally; residuals and the result refer to the original problem. Deterministic and
// single-threaded.
SocpResult solve_socp(const SocpProblem& problem, const SocpSettings& settings = {});

// Interior-point solver for a fixed problem structure: the ordering, the symbolic LDL^T factorisation and all
// work arrays are built once; solve() accepts any problem with the same variable count, cone layout and
// sparsity patterns of A and G (values, c, b, h may change; it throws std::invalid_argument otherwise).
// Not thread-safe; deterministic.
class SocpSolver {
public:
    explicit SocpSolver(const SocpProblem& structure, SocpSettings settings = {});
    ~SocpSolver();
    SocpSolver(SocpSolver&&) noexcept;
    SocpSolver& operator=(SocpSolver&&) noexcept;
    SocpResult solve(const SocpProblem& problem);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace starsim::optim
