#include "starsim/gnc/j2_insertion.hpp"

#include <array>
#include <cmath>

#include "starsim/math/vec3.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr int steps_per_sample = 4;   // RK4 steps between radius samples
constexpr int fit_samples = 72;       // radius samples over the revolution in the fit
constexpr int fit_iterations = 6;     // Gauss-Newton
// Finite-difference steps of the unknowns (radius offset, radial speed, horizontal speed offset).
constexpr std::array<double, 3> fd_step = {10.0, 0.01, 0.01};

struct Field {
    double gm = 0.0, j2 = 0.0, re = 0.0;
    Vec3 acceleration(const Vec3& r) const {
        const double r2 = dot(r, r);
        const double rn = std::sqrt(r2);
        Vec3 a = r * (-gm / (r2 * rn));
        if (j2 != 0.0) {
            const double k = -1.5 * j2 * gm * re * re / (r2 * r2 * rn);
            const double z2 = r.z * r.z / r2;
            a += Vec3{k * r.x * (1.0 - 5.0 * z2), k * r.y * (1.0 - 5.0 * z2), k * r.z * (3.0 - 5.0 * z2)};
        }
        return a;
    }
};

std::vector<double> profile(const Field& field, const InsertionState& s, double inclination, double u, int samples,
                            double target_radius) {
    // Equatorial frame, pole +z, ascending node along +x.
    const Vec3 x_hat{1.0, 0.0, 0.0};
    const Vec3 y_in_plane{0.0, std::cos(inclination), std::sin(inclination)};
    const Vec3 r_hat = x_hat * std::cos(u) + y_in_plane * std::sin(u);
    const Vec3 u_hat = x_hat * -std::sin(u) + y_in_plane * std::cos(u);
    Vec3 r = r_hat * s.radius_m;
    Vec3 v = r_hat * s.radial_speed_mps + u_hat * s.horizontal_speed_mps;
    const double period = 2.0 * constants::pi * std::sqrt(target_radius * target_radius * target_radius / field.gm);
    const double dt = period / (samples * steps_per_sample);
    std::vector<double> radii;
    radii.reserve(static_cast<size_t>(samples));
    for (int k = 0; k < samples; ++k) {
        for (int j = 0; j < steps_per_sample; ++j) {
            const Vec3 k1v = field.acceleration(r), k1r = v;
            const Vec3 k2v = field.acceleration(r + k1r * (0.5 * dt)), k2r = v + k1v * (0.5 * dt);
            const Vec3 k3v = field.acceleration(r + k2r * (0.5 * dt)), k3r = v + k2v * (0.5 * dt);
            const Vec3 k4v = field.acceleration(r + k3r * dt), k4r = v + k3v * dt;
            r += (k1r + k2r * 2.0 + k3r * 2.0 + k4r) * (dt / 6.0);
            v += (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
        }
        radii.push_back(norm(r));
    }
    return radii;
}

InsertionState offset_state(const InsertionState& base, const std::array<double, 3>& p) {
    return {base.radius_m + p[0], base.radial_speed_mps + p[1], base.horizontal_speed_mps + p[2]};
}

}  // namespace

std::vector<double> j2_radius_profile(double gm_m3ps2, const InsertionState& state, double inclination_rad,
                                      double arg_latitude_rad, int samples, double j2, double equatorial_radius_m) {
    return profile({gm_m3ps2, j2, equatorial_radius_m}, state, inclination_rad, arg_latitude_rad, samples, state.radius_m);
}

InsertionState j2_circular_insertion(double gm_m3ps2, double radius_m, double inclination_rad, double arg_latitude_rad,
                                     double j2, double equatorial_radius_m) {
    const InsertionState kepler{radius_m, 0.0, std::sqrt(gm_m3ps2 / radius_m)};
    if (j2 == 0.0) return kepler;
    const Field field{gm_m3ps2, j2, equatorial_radius_m};
    auto residuals = [&](const std::array<double, 3>& p) {
        auto r = profile(field, offset_state(kepler, p), inclination_rad, arg_latitude_rad, fit_samples, radius_m);
        for (double& x : r) x -= radius_m;
        return r;
    };
    std::array<double, 3> p{0.0, 0.0, 0.0};
    for (int it = 0; it < fit_iterations; ++it) {
        const auto f0 = residuals(p);
        std::array<std::vector<double>, 3> jac;
        for (int j = 0; j < 3; ++j) {
            auto q = p;
            q[static_cast<size_t>(j)] += fd_step[static_cast<size_t>(j)];
            auto f = residuals(q);
            for (size_t k = 0; k < f.size(); ++k) f[k] = (f[k] - f0[k]) / fd_step[static_cast<size_t>(j)];
            jac[static_cast<size_t>(j)] = std::move(f);
        }
        // Normal equations (3 x 3), Cramer's rule.
        double a[3][3] = {}, b[3] = {};
        for (size_t k = 0; k < f0.size(); ++k) {
            for (int i = 0; i < 3; ++i) {
                b[i] -= jac[static_cast<size_t>(i)][k] * f0[k];
                for (int j = 0; j < 3; ++j) a[i][j] += jac[static_cast<size_t>(i)][k] * jac[static_cast<size_t>(j)][k];
            }
        }
        const auto det3 = [](const double m[3][3]) {
            return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                   m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        };
        const double d = det3(a);
        if (std::abs(d) < 1e-300) break;
        for (int c = 0; c < 3; ++c) {
            double m[3][3];
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) m[i][j] = j == c ? b[i] : a[i][j];
            }
            p[static_cast<size_t>(c)] += det3(m) / d;
        }
    }
    return offset_state(kepler, p);
}

}  // namespace starsim::gnc
