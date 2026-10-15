#include "starsim/orbit/cr3bp.hpp"

#include <array>
#include <cmath>

#include "starsim/integrators/dop853.hpp"
#include "starsim/math/mat3.hpp"

namespace starsim::orbit {

using math::Vec3;

namespace {
constexpr double integration_tolerance = 1e-13;  // nondimensional, absolute and relative
constexpr double correction_tolerance = 1e-11;   // residual norm of the half-period crossing
constexpr double jacobian_step = 1e-7;           // central-difference step of (x0, z0, vy0)
constexpr int max_corrections = 30;

Cr3bpState symmetric_state(const Vec3& u) { return {{u.x, 0.0, u.y}, {0.0, u.z, 0.0}}; }
}  // namespace

integrators::State<6> cr3bp_derivative(double mu, const integrators::State<6>& y) {
    const double x = y[0], yy = y[1], z = y[2], vx = y[3], vy = y[4], vz = y[5];
    const double r1 = std::sqrt((x + mu) * (x + mu) + yy * yy + z * z);              // from the Earth
    const double r2 = std::sqrt((x - 1.0 + mu) * (x - 1.0 + mu) + yy * yy + z * z);  // from the Moon
    const double r1c = r1 * r1 * r1, r2c = r2 * r2 * r2;
    integrators::State<6> d;
    d[0] = vx;
    d[1] = vy;
    d[2] = vz;
    d[3] = 2.0 * vy + x - (1.0 - mu) * (x + mu) / r1c - mu * (x - 1.0 + mu) / r2c;
    d[4] = -2.0 * vx + yy - (1.0 - mu) * yy / r1c - mu * yy / r2c;
    d[5] = -(1.0 - mu) * z / r1c - mu * z / r2c;
    return d;
}

Cr3bpState cr3bp_propagate(double mu, const Cr3bpState& s, double duration) {
    integrators::State<6> y{{s.position.x, s.position.y, s.position.z, s.velocity.x, s.velocity.y, s.velocity.z}};
    if (duration != 0.0) {
        integrators::Dop853Options<6> options;
        options.absolute_tolerance = integrators::State<6>::filled(integration_tolerance);
        options.relative_tolerance = integration_tolerance;
        y = integrators::integrate<6>([mu](double, const integrators::State<6>& v) { return cr3bp_derivative(mu, v); },
                                      0.0, y, duration, options)
                .y;
    }
    return {{y[0], y[1], y[2]}, {y[3], y[4], y[5]}};
}

double jacobi_constant(double mu, const Cr3bpState& s) {
    const Vec3& p = s.position;
    const double r1 = norm(p - Vec3{-mu, 0.0, 0.0});
    const double r2 = norm(p - Vec3{1.0 - mu, 0.0, 0.0});
    return p.x * p.x + p.y * p.y + 2.0 * (1.0 - mu) / r1 + 2.0 * mu / r2 - dot(s.velocity, s.velocity);
}

std::optional<Cr3bpState> correct_periodic_orbit(double mu, const Cr3bpState& guess, double period) {
    // Unknowns u = (x0, z0, vy0); residual g = (y, vx, vz) at half the period. Newton, central-difference Jacobian.
    auto residual = [&](const Vec3& u) {
        const auto h = cr3bp_propagate(mu, symmetric_state(u), period / 2.0);
        return Vec3{h.position.y, h.velocity.x, h.velocity.z};
    };
    Vec3 u{guess.position.x, guess.position.z, guess.velocity.y};
    for (int it = 0; it < max_corrections; ++it) {
        const Vec3 g = residual(u);
        if (!std::isfinite(norm(g))) return std::nullopt;
        if (norm(g) < correction_tolerance) return symmetric_state(u);
        std::array<Vec3, 3> column;
        const std::array<Vec3, 3> unit{Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}};
        for (int k = 0; k < 3; ++k) {
            const Vec3 e = jacobian_step * unit[k];
            column[k] = (residual(u + e) - residual(u - e)) / (2.0 * jacobian_step);
        }
        const auto step = math::solve(math::Mat3::from_columns(column[0], column[1], column[2]), -g);
        if (!step) return std::nullopt;
        u += *step;
    }
    return std::nullopt;
}

EarthMoonRotatingFrame::EarthMoonRotatingFrame(const ephemeris::Ephemeris& ephemeris, time::Epoch epoch) {
    const double gm_earth = ephemeris.gm_m3ps2(ephemeris::Body::Earth);
    const double gm_moon = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    mu_ = gm_moon / (gm_earth + gm_moon);
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, epoch);
    const Vec3 r = moon.position_m, v = moon.velocity_mps;
    length_m_ = norm(r);
    const Vec3 h = cross(r, v);
    omega_radps_ = norm(h) / (length_m_ * length_m_);
    radial_rate_mps_ = dot(r, v) / length_m_;
    x_ = r / length_m_;
    z_ = h / norm(h);
    y_ = cross(z_, x_);
    barycentre_m_ = mu_ * r;
}

double EarthMoonRotatingFrame::mu() const { return mu_; }
double EarthMoonRotatingFrame::time_unit_s() const { return 1.0 / omega_radps_; }
double EarthMoonRotatingFrame::length_unit_m() const { return length_m_; }

// mu * v_moon, with v_moon = L' x + L omega y (exact: h = r x v defines omega and z).
Vec3 EarthMoonRotatingFrame::barycentre_velocity() const {
    return mu_ * (radial_rate_mps_ * x_ + length_m_ * omega_radps_ * y_);
}

math::StateVector EarthMoonRotatingFrame::to_gcrf(const Cr3bpState& s) const {
    // r = b + L R p  =>  v = b' + L omega R p' + L omega x (R p) + L' R p   (p' is per nondimensional time 1/omega).
    const Vec3 p = s.position.x * x_ + s.position.y * y_ + s.position.z * z_;
    const Vec3 pdot = s.velocity.x * x_ + s.velocity.y * y_ + s.velocity.z * z_;
    const Vec3 omega = omega_radps_ * z_;
    return {barycentre_m_ + length_m_ * p,
            barycentre_velocity() + length_m_ * omega_radps_ * pdot + length_m_ * cross(omega, p) + radial_rate_mps_ * p};
}

Cr3bpState EarthMoonRotatingFrame::from_gcrf(const math::StateVector& gcrf) const {
    const Vec3 omega = omega_radps_ * z_;
    const Vec3 p = (gcrf.position_m - barycentre_m_) / length_m_;
    const Vec3 pdot = (gcrf.velocity_mps - barycentre_velocity() - length_m_ * cross(omega, p) - radial_rate_mps_ * p) /
                      (length_m_ * omega_radps_);
    return {{dot(p, x_), dot(p, y_), dot(p, z_)}, {dot(pdot, x_), dot(pdot, y_), dot(pdot, z_)}};
}

}  // namespace starsim::orbit
