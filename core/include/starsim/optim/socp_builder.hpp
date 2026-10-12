#pragma once

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "starsim/optim/socp_problem.hpp"

namespace starsim::optim {

// Affine expression sum(coefficient * x[variable]) + constant.
struct AffineExpr {
    std::vector<std::pair<int, double>> terms;
    double constant = 0.0;

    AffineExpr() = default;
    explicit AffineExpr(double value) : constant(value) {}
    AffineExpr& add(int variable, double coefficient) {
        terms.emplace_back(variable, coefficient);
        return *this;
    }
    AffineExpr& add_constant(double value) {
        constant += value;
        return *this;
    }
};

// Contiguous block of variables; block[i] is the global index of its i-th variable.
struct VariableBlock {
    int offset = 0;
    int size = 0;
    std::string name;

    int operator[](int i) const {
        if (i < 0 || i >= size) throw std::out_of_range("VariableBlock: index outside the block");
        return offset + i;
    }
};

// Assembles a SocpProblem from named variable blocks and affine constraints, so callers never compute row
// or column indices by hand. Inequality rows are reordered into the cone order (orthant, then cones).
class SocpBuilder {
public:
    VariableBlock add_variables(int count, std::string name);
    int variable_count() const { return variables_; }

    void set_objective(const AffineExpr& expr);  // minimised; the constant is kept separately
    double objective_constant() const { return objective_.constant; }
    void add_equality(const AffineExpr& expr);                        // expr == 0
    void add_nonnegative(const AffineExpr& expr);                     // expr >= 0
    void add_second_order_cone(const std::vector<AffineExpr>& rows);  // rows[0] >= ||rows[1..]||

    SocpProblem build() const;

private:
    void check(const AffineExpr& expr) const;

    int variables_ = 0;
    AffineExpr objective_;
    std::vector<AffineExpr> equalities_;
    std::vector<AffineExpr> nonnegatives_;
    std::vector<std::vector<AffineExpr>> cones_;
};

}  // namespace starsim::optim
