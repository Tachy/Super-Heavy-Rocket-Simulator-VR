#include "starsim/optim/cones.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace starsim::optim {

namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();

double tail_norm(std::span<const double> u) {
    double s = 0.0;
    for (std::size_t i = 1; i < u.size(); ++i) s += u[i] * u[i];
    return std::sqrt(s);
}

// u0^2 - ||u1||^2 in the cancellation-free product form.
double cone_det(std::span<const double> u) {
    const double t = tail_norm(u);
    return (u[0] - t) * (u[0] + t);
}

// Smallest positive root of f(alpha) = |u0 + alpha d0|^2 - ||u1 + alpha d1||^2 = a alpha^2 + 2 b alpha + c
// (c > 0 for an interior u); the ray leaves the cone there.
double cone_max_step(std::span<const double> u, std::span<const double> d) {
    double a = d[0] * d[0];
    double b = u[0] * d[0];
    for (std::size_t i = 1; i < u.size(); ++i) {
        a -= d[i] * d[i];
        b -= u[i] * d[i];
    }
    const double c = cone_det(u);
    if (a == 0.0) return b < 0.0 ? -c / (2.0 * b) : infinity;
    const double discriminant = b * b - a * c;
    if (discriminant < 0.0) return infinity;
    const double t = -(b + std::copysign(std::sqrt(discriminant), b));
    double best = infinity;
    for (const double root : {t / a, t != 0.0 ? c / t : infinity}) {
        if (root > 0.0 && root < best) best = root;
    }
    return best;
}

std::size_t at(int row, int col, int q) { return static_cast<std::size_t>(row) * q + col; }

}  // namespace

int ConeLayout::size() const {
    int n = nonnegative;
    for (const int q : second_order) n += q;
    return n;
}

int ConeLayout::degree() const { return nonnegative + static_cast<int>(second_order.size()); }

void cone_product(const ConeLayout& layout, std::span<const double> u, std::span<const double> v, std::span<double> out) {
    int i = 0;
    for (; i < layout.nonnegative; ++i) out[i] = u[i] * v[i];
    for (const int q : layout.second_order) {
        double d = 0.0;
        for (int k = 0; k < q; ++k) d += u[i + k] * v[i + k];
        for (int k = 1; k < q; ++k) out[i + k] = u[i] * v[i + k] + v[i] * u[i + k];
        out[i] = d;
        i += q;
    }
}

void cone_division(const ConeLayout& layout, std::span<const double> lambda, std::span<const double> v,
                   std::span<double> out) {
    int i = 0;
    for (; i < layout.nonnegative; ++i) out[i] = v[i] / lambda[i];
    for (const int q : layout.second_order) {
        const auto l = lambda.subspan(static_cast<std::size_t>(i), static_cast<std::size_t>(q));
        double tail = 0.0;
        for (int k = 1; k < q; ++k) tail += l[k] * v[i + k];
        const double x0 = (l[0] * v[i] - tail) / cone_det(l);
        out[i] = x0;
        for (int k = 1; k < q; ++k) out[i + k] = (v[i + k] - x0 * l[k]) / l[0];
        i += q;
    }
}

void add_identity(const ConeLayout& layout, double alpha, std::span<double> u) {
    int i = 0;
    for (; i < layout.nonnegative; ++i) u[i] += alpha;
    for (const int q : layout.second_order) {
        u[i] += alpha;
        i += q;
    }
}

double min_eigenvalue(const ConeLayout& layout, std::span<const double> u) {
    double m = infinity;
    int i = 0;
    for (; i < layout.nonnegative; ++i) m = std::min(m, u[i]);
    for (const int q : layout.second_order) {
        m = std::min(m, u[i] - tail_norm(u.subspan(static_cast<std::size_t>(i), static_cast<std::size_t>(q))));
        i += q;
    }
    return m;
}

double max_step(const ConeLayout& layout, std::span<const double> u, std::span<const double> du) {
    double alpha = infinity;
    int i = 0;
    for (; i < layout.nonnegative; ++i) {
        if (du[i] < 0.0) alpha = std::min(alpha, -u[i] / du[i]);
    }
    for (const int q : layout.second_order) {
        const auto offset = static_cast<std::size_t>(i);
        const auto size = static_cast<std::size_t>(q);
        alpha = std::min(alpha, cone_max_step(u.subspan(offset, size), du.subspan(offset, size)));
        i += q;
    }
    return alpha;
}

NtScaling::NtScaling(const ConeLayout& layout)
    : layout_(layout),
      orthant_w_(static_cast<std::size_t>(layout.nonnegative), 1.0),
      lambda_(static_cast<std::size_t>(layout.size()), 0.0) {
    for (const int q : layout.second_order) {
        const auto block = static_cast<std::size_t>(q) * static_cast<std::size_t>(q);
        w_.emplace_back(block, 0.0);
        w_inverse_.emplace_back(block, 0.0);
        w_squared_.emplace_back(block, 0.0);
    }
}

bool NtScaling::update(std::span<const double> s, std::span<const double> z) {
    int i = 0;
    for (; i < layout_.nonnegative; ++i) {
        if (!(s[i] > 0.0 && z[i] > 0.0)) return false;
        orthant_w_[i] = std::sqrt(s[i] / z[i]);
        lambda_[i] = std::sqrt(s[i] * z[i]);
    }
    for (std::size_t k = 0; k < layout_.second_order.size(); ++k) {
        const int q = layout_.second_order[k];
        const auto sk = s.subspan(static_cast<std::size_t>(i), static_cast<std::size_t>(q));
        const auto zk = z.subspan(static_cast<std::size_t>(i), static_cast<std::size_t>(q));
        const double s_det = cone_det(sk);
        const double z_det = cone_det(zk);
        if (!(sk[0] > 0.0 && zk[0] > 0.0 && s_det > 0.0 && z_det > 0.0)) return false;
        const double s_norm = std::sqrt(s_det);
        const double z_norm = std::sqrt(z_det);
        double sz = 0.0;
        for (int r = 0; r < q; ++r) sz += sk[r] * zk[r];
        const double gamma = std::sqrt(0.5 * (1.0 + sz / (s_norm * z_norm)));
        const double eta = std::sqrt(s_norm / z_norm);
        const double a = (sk[0] / s_norm + zk[0] / z_norm) / (2.0 * gamma);
        auto qv = [&](int r) { return (sk[r] / s_norm - zk[r] / z_norm) / (2.0 * gamma); };  // r >= 1
        auto& w = w_[k];
        auto& wi = w_inverse_[k];
        w[0] = eta * a;
        wi[0] = a / eta;
        for (int r = 1; r < q; ++r) {
            w[at(0, r, q)] = w[at(r, 0, q)] = eta * qv(r);
            wi[at(0, r, q)] = wi[at(r, 0, q)] = -qv(r) / eta;
            for (int c = 1; c < q; ++c) {
                const double inner = (r == c ? 1.0 : 0.0) + qv(r) * qv(c) / (1.0 + a);
                w[at(r, c, q)] = eta * inner;
                wi[at(r, c, q)] = inner / eta;
            }
        }
        auto& w2 = w_squared_[k];
        for (int r = 0; r < q; ++r) {
            for (int c = 0; c < q; ++c) {
                double sum = 0.0;
                for (int m = 0; m < q; ++m) sum += w[at(r, m, q)] * w[at(m, c, q)];
                w2[at(r, c, q)] = sum;
            }
        }
        for (int r = 0; r < q; ++r) {
            double sum = 0.0;
            for (int c = 0; c < q; ++c) sum += w[at(r, c, q)] * zk[c];
            lambda_[i + r] = sum;
        }
        i += q;
    }
    return true;
}

void NtScaling::apply(std::span<const double> x, std::span<double> y) const { multiply(x, y, false); }

void NtScaling::apply_inverse(std::span<const double> x, std::span<double> y) const { multiply(x, y, true); }

void NtScaling::multiply(std::span<const double> x, std::span<double> y, bool inverse) const {
    int i = 0;
    for (; i < layout_.nonnegative; ++i) y[i] = inverse ? x[i] / orthant_w_[i] : x[i] * orthant_w_[i];
    for (std::size_t k = 0; k < layout_.second_order.size(); ++k) {
        const int q = layout_.second_order[k];
        const auto& m = inverse ? w_inverse_[k] : w_[k];
        for (int r = 0; r < q; ++r) {
            double sum = 0.0;
            for (int c = 0; c < q; ++c) sum += m[at(r, c, q)] * x[i + c];
            y[i + r] = sum;
        }
        i += q;
    }
}

}  // namespace starsim::optim
