#include "starsim/optim/socp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <span>

#include "starsim/optim/equilibration.hpp"
#include "starsim/optim/kkt_system.hpp"

namespace starsim::optim {

namespace {

constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double step_fraction = 0.99;  // of the distance to the cone boundary
constexpr double minimum_step = 1e-10;  // below: the iteration has stalled

struct Direction {
    std::vector<double> x, y, z, s;
    double tau = 0.0;
    double kappa = 0.0;
};

// Homogeneous self-dual embedding, unknowns (x, y, z, s, tau, kappa):
//   residuals  rx = A^T y + G^T z + c tau,  ry = -A x + b tau,  rz = s + G x - h tau,
//              rtau = kappa + c^T x + b^T y + h^T z.
// Newton direction for residual weight w, cone right-hand side d_s and scalar d_kappa:
//   K [x2; y2; z2] = [-w rx; w ry; -w rz - W (lambda \ d_s)],  K [x1; y1; z1] = [-c; b; h],
//   dtau = (-w rtau - d_kappa / tau - (c^T x2 + b^T y2 + h^T z2)) / (c^T x1 + b^T y1 + h^T z1 - kappa / tau),
//   (dx, dy, dz) = (x2, y2, z2) + dtau (x1, y1, z1),  ds = W (lambda \ d_s - W dz),
//   dkappa = (d_kappa - kappa dtau) / tau.
class InteriorPointSolver {
public:
    InteriorPointSolver(const SocpProblem& problem, const SocpSettings& settings)
        : original_(problem),
          settings_(settings),
          scaled_(problem),
          equilibration_(equilibrate(scaled_, settings.equilibration_passes)),
          n_(scaled_.variables),
          p_(scaled_.A.rows()),
          m_(scaled_.G.rows()),
          kkt_(scaled_.A, scaled_.G, scaled_.cones, settings.static_regularization, settings.refinement_steps),
          scaling_(scaled_.cones) {
        const auto n = static_cast<std::size_t>(n_);
        const auto p = static_cast<std::size_t>(p_);
        const auto m = static_cast<std::size_t>(m_);
        const auto total = n + p + m;
        x_.assign(n, 0.0);
        y_.assign(p, 0.0);
        z_.assign(m, 0.0);
        s_.assign(m, 0.0);
        rx_.assign(n, 0.0);
        ry_.assign(p, 0.0);
        rz_.assign(m, 0.0);
        rhs_.assign(total, 0.0);
        solution_.assign(total, 0.0);
        tau_column_.assign(total, 0.0);
        identity_.assign(m, 0.0);
        add_identity(scaled_.cones, 1.0, identity_);
        lambda_square_.assign(m, 0.0);
        d_s_.assign(m, 0.0);
        division_.assign(m, 0.0);
        work_a_.assign(m, 0.0);
        work_b_.assign(m, 0.0);
        for (Direction* d : {&affine_, &combined_}) {
            d->x.assign(n, 0.0);
            d->y.assign(p, 0.0);
            d->z.assign(m, 0.0);
            d->s.assign(m, 0.0);
        }
        xo_.assign(n, 0.0);
        yo_.assign(p, 0.0);
        zo_.assign(m, 0.0);
        so_.assign(m, 0.0);
        ax_.assign(p, 0.0);
        gx_.assign(m, 0.0);
        aty_.assign(n, 0.0);
    }

    SocpResult run() {
        SocpResult result;
        if (!initialize()) return finish(result, SocpStatus::NumericalError);
        const double degree = scaled_.cones.degree();
        for (int iteration = 0;; ++iteration) {
            result.iterations = iteration;
            compute_residuals();
            if (check_termination(result)) return result;
            if (iteration >= settings_.max_iterations) return finish(result, SocpStatus::MaxIterations);
            if (!scaling_.update(s_, z_) || !kkt_.factor(scaling_)) return finish(result, SocpStatus::NumericalError);

            // Column of tau: K [x1; y1; z1] = [-c; b; h].
            fill_rhs([&](int j) { return -scaled_.c[j]; }, [&](int i) { return scaled_.b[i]; },
                     [&](int i) { return scaled_.h[i]; });
            kkt_.solve(rhs_, tau_column_);
            const double mu = (dot(s_, z_) + tau_ * kappa_) / (degree + 1.0);

            // Predictor (affine scaling) direction.
            cone_product(scaled_.cones, scaling_.lambda(), scaling_.lambda(), lambda_square_);
            for (int i = 0; i < m_; ++i) d_s_[i] = -lambda_square_[i];
            solve_direction(1.0, -tau_ * kappa_, affine_);
            const double alpha_affine = std::min(1.0, max_step_length(affine_));
            const double sigma = std::pow(1.0 - alpha_affine, 3);

            // Corrector: centring plus the second-order term of the affine direction.
            scaling_.apply_inverse(affine_.s, work_a_);
            scaling_.apply(affine_.z, work_b_);
            cone_product(scaled_.cones, work_a_, work_b_, d_s_);
            for (int i = 0; i < m_; ++i) d_s_[i] = -lambda_square_[i] - d_s_[i] + sigma * mu * identity_[i];
            solve_direction(1.0 - sigma, -tau_ * kappa_ - affine_.tau * affine_.kappa + sigma * mu, combined_);
            const double alpha = std::min(1.0, step_fraction * max_step_length(combined_));
            if (settings_.verbose) {
                std::fprintf(stderr, "%3d  pres %9.2e  dres %9.2e  gap %9.2e  sigma %5.3f  alpha %5.3f\n", iteration,
                             result.primal_residual, result.dual_residual, result.gap, sigma, alpha);
            }
            if (alpha < minimum_step) return finish(result, SocpStatus::NumericalError);
            take_step(alpha, combined_);
        }
    }

private:
    template <class FX, class FY, class FZ>
    void fill_rhs(FX fx, FY fy, FZ fz) {
        for (int j = 0; j < n_; ++j) rhs_[j] = fx(j);
        for (int i = 0; i < p_; ++i) rhs_[n_ + i] = fy(i);
        for (int i = 0; i < m_; ++i) rhs_[n_ + p_ + i] = fz(i);
    }

    static std::span<const double> part(const std::vector<double>& v, int offset, int size) {
        return std::span<const double>(v).subspan(static_cast<std::size_t>(offset), static_cast<std::size_t>(size));
    }

    // Shifts u into the interior: unchanged if already inside, else u + (1 - min eigenvalue) e.
    void shift_into_cone(std::vector<double>& u) const {
        const double alpha = -min_eigenvalue(scaled_.cones, u);
        if (alpha >= 0.0) add_identity(scaled_.cones, 1.0 + alpha, u);
    }

    // Least-squares starting point (as ECOS/CVXOPT) with W = I.
    bool initialize() {
        if (!scaling_.update(identity_, identity_) || !kkt_.factor(scaling_)) return false;
        fill_rhs([](int) { return 0.0; }, [&](int i) { return scaled_.b[i]; }, [&](int i) { return scaled_.h[i]; });
        kkt_.solve(rhs_, solution_);
        for (int j = 0; j < n_; ++j) x_[j] = solution_[j];
        for (int i = 0; i < m_; ++i) s_[i] = -solution_[n_ + p_ + i];
        shift_into_cone(s_);
        fill_rhs([&](int j) { return -scaled_.c[j]; }, [](int) { return 0.0; }, [](int) { return 0.0; });
        kkt_.solve(rhs_, solution_);
        for (int i = 0; i < p_; ++i) y_[i] = solution_[n_ + i];
        for (int i = 0; i < m_; ++i) z_[i] = solution_[n_ + p_ + i];
        shift_into_cone(z_);
        tau_ = 1.0;
        kappa_ = 1.0;
        return true;
    }

    void compute_residuals() {
        for (int j = 0; j < n_; ++j) rx_[j] = scaled_.c[j] * tau_;
        scaled_.A.multiply_transpose_add(y_, rx_);
        scaled_.G.multiply_transpose_add(z_, rx_);
        for (int i = 0; i < p_; ++i) ry_[i] = scaled_.b[i] * tau_;
        scaled_.A.multiply_add(x_, ry_, -1.0);
        for (int i = 0; i < m_; ++i) rz_[i] = s_[i] - scaled_.h[i] * tau_;
        scaled_.G.multiply_add(x_, rz_);
        rtau_ = kappa_ + dot(scaled_.c, x_) + dot(scaled_.b, y_) + dot(scaled_.h, z_);
    }

    // Newton direction of the embedding for d_s_ (see the class comment).
    void solve_direction(double weight, double d_kappa, Direction& d) {
        cone_division(scaled_.cones, scaling_.lambda(), d_s_, division_);  // lambda \ d_s
        scaling_.apply(division_, work_a_);                                 // W (lambda \ d_s)
        fill_rhs([&](int j) { return -weight * rx_[j]; }, [&](int i) { return weight * ry_[i]; },
                 [&](int i) { return -weight * rz_[i] - work_a_[i]; });
        kkt_.solve(rhs_, solution_);
        const auto x2 = part(solution_, 0, n_), y2 = part(solution_, n_, p_), z2 = part(solution_, n_ + p_, m_);
        const auto x1 = part(tau_column_, 0, n_), y1 = part(tau_column_, n_, p_), z1 = part(tau_column_, n_ + p_, m_);
        const double numerator = -weight * rtau_ - d_kappa / tau_ -
                                 (dot(scaled_.c, x2) + dot(scaled_.b, y2) + dot(scaled_.h, z2));
        const double denominator = dot(scaled_.c, x1) + dot(scaled_.b, y1) + dot(scaled_.h, z1) - kappa_ / tau_;
        d.tau = numerator / denominator;
        for (int j = 0; j < n_; ++j) d.x[j] = x2[j] + d.tau * x1[j];
        for (int i = 0; i < p_; ++i) d.y[i] = y2[i] + d.tau * y1[i];
        for (int i = 0; i < m_; ++i) d.z[i] = z2[i] + d.tau * z1[i];
        d.kappa = (d_kappa - kappa_ * d.tau) / tau_;
        scaling_.apply(d.z, work_b_);  // W dz
        for (int i = 0; i < m_; ++i) work_b_[i] = division_[i] - work_b_[i];
        scaling_.apply(work_b_, d.s);  // ds = W (lambda \ d_s - W dz)
    }

    double max_step_length(const Direction& d) const {
        double alpha = std::min(max_step(scaled_.cones, s_, d.s), max_step(scaled_.cones, z_, d.z));
        if (d.tau < 0.0) alpha = std::min(alpha, -tau_ / d.tau);
        if (d.kappa < 0.0) alpha = std::min(alpha, -kappa_ / d.kappa);
        return alpha;
    }

    void take_step(double alpha, const Direction& d) {
        for (int j = 0; j < n_; ++j) x_[j] += alpha * d.x[j];
        for (int i = 0; i < p_; ++i) y_[i] += alpha * d.y[i];
        for (int i = 0; i < m_; ++i) {
            z_[i] += alpha * d.z[i];
            s_[i] += alpha * d.s[i];
        }
        tau_ += alpha * d.tau;
        kappa_ += alpha * d.kappa;
    }

    // Unscaled copy of the iterate (still multiplied by tau).
    void unscaled_iterate() {
        std::copy(x_.begin(), x_.end(), xo_.begin());
        std::copy(y_.begin(), y_.end(), yo_.begin());
        std::copy(z_.begin(), z_.end(), zo_.begin());
        std::copy(s_.begin(), s_.end(), so_.begin());
        unscale(equilibration_, xo_, yo_, zo_, so_);
    }

    bool check_termination(SocpResult& result) {
        unscaled_iterate();
        const auto& o = original_;
        std::fill(ax_.begin(), ax_.end(), 0.0);
        o.A.multiply_add(xo_, ax_);
        std::copy(so_.begin(), so_.end(), gx_.begin());
        o.G.multiply_add(xo_, gx_);  // G x + s
        std::fill(aty_.begin(), aty_.end(), 0.0);
        o.A.multiply_transpose_add(yo_, aty_);
        o.G.multiply_transpose_add(zo_, aty_);  // A^T y + G^T z

        double primal_a = 0.0, primal_g = 0.0, dual = 0.0;
        for (int i = 0; i < p_; ++i) primal_a += std::pow(ax_[i] - o.b[i] * tau_, 2);
        for (int i = 0; i < m_; ++i) primal_g += std::pow(gx_[i] - o.h[i] * tau_, 2);
        for (int j = 0; j < n_; ++j) dual += std::pow(aty_[j] + o.c[j] * tau_, 2);
        result.primal_residual =
            std::max(std::sqrt(primal_a) / (1.0 + norm2(o.b)), std::sqrt(primal_g) / (1.0 + norm2(o.h))) / tau_;
        result.dual_residual = std::sqrt(dual) / (1.0 + norm2(o.c)) / tau_;
        result.gap = dot(so_, zo_) / (tau_ * tau_);
        const double cx = dot(o.c, xo_);
        const double primal_cost = cx / tau_;
        const double tol = settings_.tolerance;

        if (result.primal_residual <= tol && result.dual_residual <= tol &&
            result.gap <= tol * std::max(1.0, std::abs(primal_cost))) {
            store(result, 1.0 / tau_);
            result.objective = primal_cost;
            result.status = SocpStatus::Optimal;
            return true;
        }
        if (tau_ < kappa_) {
            const double by_hz = dot(o.b, yo_) + dot(o.h, zo_);
            if (by_hz < 0.0 && norm2(aty_) / -by_hz <= tol) {
                store(result, 1.0 / -by_hz);
                result.objective = infinity;
                result.status = SocpStatus::PrimalInfeasible;
                return true;
            }
            if (cx < 0.0 && std::max(norm2(ax_), norm2(gx_)) / -cx <= tol) {
                store(result, 1.0 / -cx);
                result.objective = -infinity;
                result.status = SocpStatus::DualInfeasible;
                return true;
            }
        }
        return false;
    }

    void store(SocpResult& result, double factor) const {
        result.x.resize(xo_.size());
        result.y.resize(yo_.size());
        result.z.resize(zo_.size());
        result.s.resize(so_.size());
        for (std::size_t j = 0; j < xo_.size(); ++j) result.x[j] = xo_[j] * factor;
        for (std::size_t i = 0; i < yo_.size(); ++i) result.y[i] = yo_[i] * factor;
        for (std::size_t i = 0; i < zo_.size(); ++i) {
            result.z[i] = zo_[i] * factor;
            result.s[i] = so_[i] * factor;
        }
    }

    SocpResult& finish(SocpResult& result, SocpStatus status) {
        unscaled_iterate();
        store(result, 1.0 / tau_);
        result.objective = dot(original_.c, result.x);
        result.status = status;
        return result;
    }

    const SocpProblem& original_;
    SocpSettings settings_;
    SocpProblem scaled_;
    Equilibration equilibration_;
    int n_, p_, m_;
    KktSystem kkt_;
    NtScaling scaling_;

    std::vector<double> x_, y_, z_, s_;
    double tau_ = 1.0;
    double kappa_ = 1.0;
    std::vector<double> rx_, ry_, rz_;
    double rtau_ = 0.0;
    std::vector<double> rhs_, solution_, tau_column_;
    std::vector<double> identity_, lambda_square_, d_s_, division_, work_a_, work_b_;
    Direction affine_, combined_;
    std::vector<double> xo_, yo_, zo_, so_, ax_, gx_, aty_;
};

}  // namespace

SocpResult solve_socp(const SocpProblem& problem, const SocpSettings& settings) {
    problem.validate();
    InteriorPointSolver solver(problem, settings);
    return solver.run();
}

}  // namespace starsim::optim
