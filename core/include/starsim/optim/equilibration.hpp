#pragma once

#include <span>
#include <vector>

#include "starsim/optim/socp_problem.hpp"

namespace starsim::optim {

// Diagonal scaling factors of a Ruiz equilibration: x = D x^, rows of A scaled by E_A, rows of G by E_G.
struct Equilibration {
    std::vector<double> column;        // D
    std::vector<double> equality_row;  // E_A
    std::vector<double> cone_row;      // E_G (equal within each second-order cone)
};

// Ruiz equilibration (infinity norms) of [A; G], applied in place: A <- E_A A D, G <- E_G G D, c <- D c,
// b <- E_A b, h <- E_G h. The rows of one second-order cone share a factor so the cone is preserved.
Equilibration equilibrate(SocpProblem& problem, int passes);

// Maps a solution of the scaled problem back: x = D x^, y = E_A y^, z = E_G z^, s = s^ / E_G.
void unscale(const Equilibration& e, std::span<double> x, std::span<double> y, std::span<double> z,
             std::span<double> s);

}  // namespace starsim::optim
