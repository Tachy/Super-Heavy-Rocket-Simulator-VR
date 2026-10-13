#include "starsim/gnc/tvc_lateral.hpp"

namespace starsim::gnc {

namespace {

constexpr double design_step_s = 0.05;   // discrete design step (the gains are applied every tick)
constexpr int riccati_iterations = 600;  // converged for the slow modes (time constant ~ seconds)

using Mat4 = std::array<std::array<double, 4>, 4>;
using Vec4 = std::array<double, 4>;

Mat4 multiply(const Mat4& x, const Mat4& y) {
    Mat4 r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            for (int k = 0; k < 4; ++k) r[i][j] += x[i][k] * y[k][j];
    return r;
}

Mat4 transpose(const Mat4& x) {
    Mat4 r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) r[i][j] = x[j][i];
    return r;
}

}  // namespace

TvcLateralGains tvc_lateral_gains(double a, double c, const TvcLateralWeights& w) {
    const double dt = design_step_s;
    // Continuous model and its exact discretisation (A_c is nilpotent: A_c^4 = 0, so the series ends at the cube).
    Mat4 ac{};
    ac[0][1] = 1.0;
    ac[1][2] = a;
    ac[2][3] = 1.0;
    const Vec4 bc{0.0, -c, 0.0, 1.0};
    const Mat4 ac2 = multiply(ac, ac);
    const Mat4 ac3 = multiply(ac2, ac);
    Mat4 ad{};
    Vec4 bd{};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            ad[i][j] = (i == j ? 1.0 : 0.0) + ac[i][j] * dt + ac2[i][j] * dt * dt / 2.0 + ac3[i][j] * dt * dt * dt / 6.0;
        }
    }
    for (int i = 0; i < 4; ++i) {  // B = (I dt + A dt^2/2 + A^2 dt^3/6) b
        double s = bc[i] * dt;
        for (int j = 0; j < 4; ++j) s += (ac[i][j] * dt * dt / 2.0 + ac2[i][j] * dt * dt * dt / 6.0) * bc[j];
        bd[i] = s;
    }
    const Vec4 q{1.0 / (w.position_m * w.position_m), 1.0 / (w.velocity_mps * w.velocity_mps),
                 1.0 / (w.tilt_rad * w.tilt_rad), 1.0 / (w.tilt_rate_radps * w.tilt_rate_radps)};
    const double r = 1.0 / (w.angular_accel_radps2 * w.angular_accel_radps2);

    Mat4 p{};
    for (int i = 0; i < 4; ++i) p[i][i] = q[i];
    const Mat4 adt = transpose(ad);
    Vec4 k{};
    for (int it = 0; it < riccati_iterations; ++it) {
        // K = (r + b' P b)^-1 b' P A
        Vec4 pb{};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) pb[i] += p[i][j] * bd[j];
        double bpb = r;
        for (int i = 0; i < 4; ++i) bpb += bd[i] * pb[i];
        const Mat4 pa = multiply(p, ad);
        for (int j = 0; j < 4; ++j) {
            double s = 0.0;
            for (int i = 0; i < 4; ++i) s += bd[i] * pa[i][j];
            k[j] = s / bpb;
        }
        // P = Q + A' P A - A' P b K
        Mat4 next = multiply(adt, pa);
        for (int i = 0; i < 4; ++i) {
            double apb = 0.0;
            for (int m = 0; m < 4; ++m) apb += adt[i][m] * pb[m];
            for (int j = 0; j < 4; ++j) next[i][j] -= apb * k[j];
            next[i][i] += q[i];
        }
        p = next;
    }
    return {k};
}

}  // namespace starsim::gnc
