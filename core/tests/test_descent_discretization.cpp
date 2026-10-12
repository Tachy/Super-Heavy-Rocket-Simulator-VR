#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/gnc/descent_discretization.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Quat;
using math::Vec3;

namespace {

DescentModel model() {
    DescentModel m;
    for (auto& set : m.engine_sets) set = {1.0e7, 4.0e6, 1.0 / (330.0 * 9.80665), 0.26};
    m.gimbal_lever_m = 30.0;
    m.inertia_kg_m2 = {1.0e7, 2.0e8, 2.0e8};
    m.reference_area_m2 = 63.6;
    m.cd_by_mach = {0.9, 1.1, 1.3};
    m.mach_step = 0.5;
    m.density_kg_m3 = {1.2, 1.0, 0.8};
    m.speed_of_sound_mps = {340.0, 335.0, 330.0};
    m.height_step_m = 500.0;
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

const DescentState x0 = make_state(5.0e5, {900.0, 20.0, -10.0}, {-150.0, 3.0, -2.0},
                                   Quat::from_axis_angle({0.0, 0.3, 1.0}, 0.05), {0.0, 0.01, -0.02});
const Vec3 u0{8.0e6, 1.0e5, 0.0};
const Vec3 u1{7.0e6, 0.0, -1.0e5};

}  // namespace

TEST_CASE("FOH propagation with zero perturbation reproduces the nonlinear state", "[gnc][descent]") {
    const auto m = model();
    const auto coarse = linearize_interval(m, 0, x0, u0, u1, 20.0, 0.1, 5);
    const auto fine = linearize_interval(m, 0, x0, u0, u1, 20.0, 0.1, 40);
    // RK4 truncation at the kinks of the linearly interpolated air/drag tables: ~3e-6 relative with 5 steps.
    for (int i = 0; i < 14; ++i) CHECK(std::abs(coarse.propagated[i] - fine.propagated[i]) <= 1e-5 * (1.0 + std::abs(fine.propagated[i])));
}

TEST_CASE("FOH sensitivities predict a perturbed propagation", "[gnc][descent]") {
    const auto m = model();
    const double s = 20.0, dtau = 0.1;
    const auto ref = linearize_interval(m, 0, x0, u0, u1, s, dtau, 10);
    DescentState dx;
    for (int i = 0; i < 14; ++i) dx[i] = 1e-4 * (1.0 + std::abs(x0[i])) * ((i % 2) ? 1.0 : -1.0);
    const Vec3 du0{2.0e4, -1.0e4, 5.0e3}, du1{-1.0e4, 2.0e4, 0.0};
    const double ds = 0.02;
    const auto pert = linearize_interval(m, 0, x0 + dx, u0 + du0, u1 + du1, s + ds, dtau, 10);
    for (int i = 0; i < 14; ++i) {
        double predicted = ref.sigma[static_cast<size_t>(i)] * ds;
        for (int j = 0; j < 14; ++j) predicted += ref.phi[static_cast<size_t>(i * 14 + j)] * dx[j];
        const double dm[3] = {du0.x, du0.y, du0.z}, dp[3] = {du1.x, du1.y, du1.z};
        for (int j = 0; j < 3; ++j) {
            predicted += ref.gamma_minus[static_cast<size_t>(i * 3 + j)] * dm[j] + ref.gamma_plus[static_cast<size_t>(i * 3 + j)] * dp[j];
        }
        const double actual = pert.propagated[i] - ref.propagated[i];
        CHECK(std::abs(predicted - actual) <= 1e-2 * std::abs(actual) + 1e-8);
    }
}

TEST_CASE("A zero-length interval is the identity", "[gnc][descent]") {
    const auto lin = linearize_interval(model(), 0, x0, u0, u1, 0.0, 0.1, 5);
    for (int i = 0; i < 14; ++i) {
        CHECK(lin.propagated[i] == x0[i]);
        for (int j = 0; j < 14; ++j) CHECK(lin.phi[static_cast<size_t>(i * 14 + j)] == (i == j ? 1.0 : 0.0));
    }
}
