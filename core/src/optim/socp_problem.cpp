#include "starsim/optim/socp_problem.hpp"

#include <stdexcept>
#include <string>

namespace starsim::optim {

void SocpProblem::validate() const {
    const auto fail = [](const char* what) { throw std::invalid_argument(std::string("SocpProblem: ") + what); };
    if (variables < 0) fail("negative variable count");
    if (static_cast<int>(c.size()) != variables) fail("c has the wrong size");
    if (A.cols() != variables || static_cast<int>(b.size()) != A.rows()) fail("A or b has the wrong size");
    if (G.cols() != variables || static_cast<int>(h.size()) != G.rows()) fail("G or h has the wrong size");
    if (cones.nonnegative < 0 || cones.size() != G.rows()) fail("cone layout does not match G");
    for (const int q : cones.second_order) {
        if (q < 1) fail("second-order cone of dimension < 1");
    }
}

const char* to_string(SocpStatus status) {
    switch (status) {
        case SocpStatus::Optimal: return "optimal";
        case SocpStatus::PrimalInfeasible: return "primal infeasible";
        case SocpStatus::DualInfeasible: return "dual infeasible";
        case SocpStatus::MaxIterations: return "maximum iterations";
        case SocpStatus::NumericalError: return "numerical error";
    }
    return "";
}

}  // namespace starsim::optim
