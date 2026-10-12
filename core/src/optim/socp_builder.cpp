#include "starsim/optim/socp_builder.hpp"

#include <stdexcept>

namespace starsim::optim {

VariableBlock SocpBuilder::add_variables(int count, std::string name) {
    if (count < 0) throw std::invalid_argument("SocpBuilder: negative variable count");
    VariableBlock block{variables_, count, std::move(name)};
    variables_ += count;
    return block;
}

void SocpBuilder::check(const AffineExpr& expr) const {
    for (const auto& [index, coefficient] : expr.terms) {
        if (index < 0 || index >= variables_) throw std::out_of_range("SocpBuilder: unknown variable index");
    }
}

void SocpBuilder::set_objective(const AffineExpr& expr) {
    check(expr);
    objective_ = expr;
}

void SocpBuilder::add_equality(const AffineExpr& expr) {
    check(expr);
    equalities_.push_back(expr);
}

void SocpBuilder::add_nonnegative(const AffineExpr& expr) {
    check(expr);
    nonnegatives_.push_back(expr);
}

void SocpBuilder::add_second_order_cone(const std::vector<AffineExpr>& rows) {
    if (rows.empty()) throw std::invalid_argument("SocpBuilder: empty second-order cone");
    for (const auto& row : rows) check(row);
    cones_.push_back(rows);
}

SocpProblem SocpBuilder::build() const {
    SocpProblem p;
    p.variables = variables_;
    p.c.assign(static_cast<std::size_t>(variables_), 0.0);
    for (const auto& [index, coefficient] : objective_.terms) p.c[index] += coefficient;

    std::vector<Triplet> a;
    for (int r = 0; r < static_cast<int>(equalities_.size()); ++r) {
        for (const auto& [index, coefficient] : equalities_[r].terms) a.push_back({r, index, coefficient});
        p.b.push_back(-equalities_[r].constant);
    }
    p.A = CscMatrix(static_cast<int>(equalities_.size()), variables_, a);

    // s = expr = coefficients x + constant  <=>  G x + s = h with G = -coefficients, h = constant.
    std::vector<Triplet> g;
    int row = 0;
    auto add_row = [&](const AffineExpr& expr) {
        for (const auto& [index, coefficient] : expr.terms) g.push_back({row, index, -coefficient});
        p.h.push_back(expr.constant);
        ++row;
    };
    for (const auto& expr : nonnegatives_) add_row(expr);
    for (const auto& cone : cones_) {
        for (const auto& expr : cone) add_row(expr);
        p.cones.second_order.push_back(static_cast<int>(cone.size()));
    }
    p.cones.nonnegative = static_cast<int>(nonnegatives_.size());
    p.G = CscMatrix(row, variables_, g);
    return p;
}

}  // namespace starsim::optim
