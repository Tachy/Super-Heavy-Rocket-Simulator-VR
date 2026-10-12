#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "starsim/optim/socp_problem.hpp"

namespace starsim::test {

// Largest constraint violation of x: equality residual and distance of h - G x outside the cone
// (minus its smallest eigenvalue).
inline double constraint_violation(const optim::SocpProblem& p, const std::vector<double>& x) {
    std::vector<double> ax(static_cast<std::size_t>(p.A.rows()), 0.0);
    p.A.multiply_add(x, ax);
    double v = 0.0;
    for (std::size_t i = 0; i < ax.size(); ++i) v = std::max(v, std::abs(ax[i] - p.b[i]));
    std::vector<double> s = p.h;
    p.G.multiply_add(x, s, -1.0);
    if (!s.empty()) v = std::max(v, -optim::min_eigenvalue(p.cones, s));
    return v;
}

}  // namespace starsim::test
