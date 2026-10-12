#pragma once

#include "starsim/optim/socp_problem.hpp"

namespace starsim::optim {

// Primal-dual interior-point method on the homogeneous self-dual embedding with Mehrotra predictor-corrector
// steps and Nesterov-Todd scaling (after ECOS: Domahidi, Chu, Boyd 2013; Vandenberghe 2010). The data is
// Ruiz-equilibrated internally; residuals and the result refer to the original problem. Deterministic and
// single-threaded.
SocpResult solve_socp(const SocpProblem& problem, const SocpSettings& settings = {});

}  // namespace starsim::optim
