#pragma once

#include <span>
#include <vector>

namespace starsim::optim {

// K = R+^l x Q^{q1} x ... x Q^{qm}, Q^q = { u in R^q : ||(u1, ..., u_{q-1})|| <= u0 }. Vectors on K are laid
// out orthant first, then each second-order cone. Dimension 1 means u0 >= 0, dimension 2 means u0 >= |u1|.
struct ConeLayout {
    int nonnegative = 0;
    std::vector<int> second_order;  // dimensions, each >= 1

    int size() const;
    int degree() const;  // l + number of second-order cones
};

// Jordan algebra (Vandenberghe, "The CVXOPT linear and quadratic cone program solvers", 2010):
// u o v: orthant elementwise; cone (u.v, u0 v1 + v0 u1).
void cone_product(const ConeLayout& layout, std::span<const double> u, std::span<const double> v, std::span<double> out);
// out = lambda \ v, i.e. lambda o out = v; lambda in the interior. `out` must not alias `v`.
void cone_division(const ConeLayout& layout, std::span<const double> lambda, std::span<const double> v,
                   std::span<double> out);
// u += alpha e (e: identity element, 1 on the orthant, (1, 0, ..., 0) per cone).
void add_identity(const ConeLayout& layout, double alpha, std::span<double> u);
// Smallest eigenvalue (orthant: u_i; cone: u0 - ||u1||); +infinity for an empty cone.
double min_eigenvalue(const ConeLayout& layout, std::span<const double> u);
// Largest alpha with u + alpha du in K (u in the interior); +infinity if the ray stays inside.
double max_step(const ConeLayout& layout, std::span<const double> u, std::span<const double> du);

// Nesterov-Todd scaling W of an interior pair (s, z): W z = W^{-T} s = lambda. W is symmetric.
// Orthant: W = diag(sqrt(s / z)). Cone: W = eta [a q^T; q I + q q^T / (1 + a)] with (a, q) = wbar,
// wbar = (sbar + J zbar) / (2 gamma), gamma = sqrt((1 + sbar.zbar) / 2), eta = (det s / det z)^(1/4).
class NtScaling {
public:
    explicit NtScaling(const ConeLayout& layout);

    // False if s or z is not in the interior of K.
    bool update(std::span<const double> s, std::span<const double> z);
    void apply(std::span<const double> x, std::span<double> y) const;          // y = W x
    void apply_inverse(std::span<const double> x, std::span<double> y) const;  // y = W^{-1} x
    std::span<const double> lambda() const { return lambda_; }
    double orthant_squared(int i) const { return orthant_w_[i] * orthant_w_[i]; }
    std::span<const double> cone_squared(int k) const { return w_squared_[k]; }  // row-major q x q

private:
    void multiply(std::span<const double> x, std::span<double> y, bool inverse) const;

    ConeLayout layout_;
    std::vector<double> orthant_w_;
    std::vector<std::vector<double>> w_, w_inverse_, w_squared_;  // per cone, row-major q x q
    std::vector<double> lambda_;
};

}  // namespace starsim::optim
