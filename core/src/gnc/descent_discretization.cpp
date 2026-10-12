#include "starsim/gnc/descent_discretization.hpp"

#include "starsim/integrators/rk4.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

// Packed state: x (14), phi (196), gamma_minus (42), gamma_plus (42), sigma (14).
constexpr size_t n_x = 14;
constexpr size_t o_phi = n_x;
constexpr size_t o_gm = o_phi + 196;
constexpr size_t o_gp = o_gm + 42;
constexpr size_t o_sigma = o_gp + 42;
constexpr size_t n_total = o_sigma + 14;
using Packed = integrators::State<n_total>;

}  // namespace

// Normalised time tau over [0, dtau], phase duration s, FOH thrust u(tau) = u0 (1 - tau/dtau) + u1 tau/dtau:
//   x' = s f,  phi' = s A phi,  gm' = s A gm + s B (1 - tau/dtau),  gp' = s A gp + s B tau/dtau,  sigma' = s A sigma + f.
IntervalLinearization linearize_interval(const DescentModel& model, int phase, const DescentState& x0, const Vec3& u0,
                                         const Vec3& u1, double s, double dtau, int substeps) {
    Packed y;
    for (size_t i = 0; i < n_x; ++i) y[i] = x0[static_cast<int>(i)];
    for (size_t i = 0; i < 14; ++i) y[o_phi + i * 14 + i] = 1.0;
    const integrators::OdeFunction<n_total> rhs = [&](double tau, const Packed& p) {
        const double lambda_plus = tau / dtau;
        const double lambda_minus = 1.0 - lambda_plus;
        const Vec3 u = u0 * lambda_minus + u1 * lambda_plus;
        DescentState x;
        for (int i = 0; i < 14; ++i) x[i] = p[static_cast<size_t>(i)];
        const DescentState f = model.derivative(x, u, phase);
        Mat14x14 a;
        Mat14x3 b;
        model.jacobians(x, u, phase, a, b);
        Packed d;
        for (int i = 0; i < 14; ++i) d[static_cast<size_t>(i)] = s * f[i];
        for (size_t i = 0; i < 14; ++i) {
            for (size_t c = 0; c < 14; ++c) {
                double sum = 0.0;
                for (size_t k = 0; k < 14; ++k) sum += a[i * 14 + k] * p[o_phi + k * 14 + c];
                d[o_phi + i * 14 + c] = s * sum;
            }
            for (size_t c = 0; c < 3; ++c) {
                double gm = 0.0, gp = 0.0;
                for (size_t k = 0; k < 14; ++k) {
                    gm += a[i * 14 + k] * p[o_gm + k * 3 + c];
                    gp += a[i * 14 + k] * p[o_gp + k * 3 + c];
                }
                d[o_gm + i * 3 + c] = s * (gm + b[i * 3 + c] * lambda_minus);
                d[o_gp + i * 3 + c] = s * (gp + b[i * 3 + c] * lambda_plus);
            }
            double sg = 0.0;
            for (size_t k = 0; k < 14; ++k) sg += a[i * 14 + k] * p[o_sigma + k];
            d[o_sigma + i] = s * sg + f[static_cast<int>(i)];
        }
        return d;
    };
    const double h = dtau / substeps;
    for (int k = 0; k < substeps; ++k) y = integrators::rk4_step<n_total>(rhs, k * h, y, h);

    IntervalLinearization out;
    for (int i = 0; i < 14; ++i) out.propagated[i] = y[static_cast<size_t>(i)];
    for (size_t i = 0; i < 196; ++i) out.phi[i] = y[o_phi + i];
    for (size_t i = 0; i < 42; ++i) {
        out.gamma_minus[i] = y[o_gm + i];
        out.gamma_plus[i] = y[o_gp + i];
    }
    for (size_t i = 0; i < 14; ++i) out.sigma[i] = y[o_sigma + i];
    return out;
}

}  // namespace starsim::gnc
