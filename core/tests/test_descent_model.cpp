#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/gnc/descent_model.hpp"

using namespace starsim;
using namespace starsim::gnc;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using math::Quat;
using math::Vec3;

namespace {

// Simple model: one set of 10 MN (min 4 MN), Isp 330 s, lever 30 m, no air, no Earth rotation.
DescentModel simple_model() {
    DescentModel m;
    for (auto& set : m.engine_sets) set = {1.0e7, 4.0e6, 1.0 / (330.0 * 9.80665), 15.0 * 3.14159265358979 / 180.0};
    m.gimbal_lever_m = 30.0;
    m.inertia_kg_m2 = {1.0e7, 2.0e8, 2.0e8};
    m.reference_area_m2 = 63.6;
    m.cd_by_mach = {1.0, 1.0};
    m.density_kg_m3 = {0.0, 0.0};
    m.speed_of_sound_mps = {340.0, 340.0};
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

DescentState upright(double mass, const Vec3& r, const Vec3& v) { return make_state(mass, r, v, Quat{1.0, 0.0, 0.0, 0.0}, {}); }

}  // namespace

TEST_CASE("Planning frame maps site east-north-up to up-east-north", "[gnc][descent]") {
    const Vec3 enu{1.0, 2.0, 3.0};
    CHECK(site_to_plan(enu) == Vec3{3.0, 1.0, 2.0});
    CHECK(plan_to_site(site_to_plan(enu)) == enu);
}

TEST_CASE("Planning model: thrust, gravity, mass flow and gimbal torque", "[gnc][descent]") {
    const auto model = simple_model();
    const auto x = upright(5.0e5, {1000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0});
    const auto d = model.derivative(x, {8.0e6, 0.0, 0.0}, 0);
    CHECK_THAT(d[descent_index::mass], WithinRel(-8.0e6 / (330.0 * 9.80665), 1e-12));
    CHECK_THAT(d[descent_index::position], WithinAbs(-100.0, 1e-12));
    CHECK_THAT(d[descent_index::velocity], WithinRel(8.0e6 / 5.0e5 - 9.8, 1e-12));
    // Thrust tilted towards body +z (gimbal "pitch"): torque about +y = lever * u_z.
    const auto t = model.derivative(x, {8.0e6, 0.0, 1.0e5}, 0);
    CHECK_THAT(t[descent_index::rate + 1], WithinRel(30.0 * 1.0e5 / 2.0e8, 1e-12));
    CHECK_THAT(t[descent_index::rate + 2], WithinAbs(0.0, 1e-15));
}

TEST_CASE("Planning model: drag opposes the motion through the air", "[gnc][descent]") {
    auto model = simple_model();
    model.density_kg_m3 = {1.2, 1.2};
    const auto x = upright(5.0e5, {1000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0});
    const auto d = model.derivative(x, {0.0, 0.0, 0.0}, 0);
    // a = -9.8 + 0.5 rho v^2 A cd / m (upwards for a falling body).
    CHECK_THAT(d[descent_index::velocity], WithinRel(-9.8 + 0.5 * 1.2 * 1.0e4 * 63.6 / 5.0e5, 1e-9));
}

TEST_CASE("Planning model Jacobians predict small changes", "[gnc][descent]") {
    auto model = simple_model();
    model.density_kg_m3 = {1.2, 0.8};
    model.earth_rate_radps = {3.2e-5, 0.0, 6.5e-5};
    model.earth_center_m = {-6.37e6, 0.0, 0.0};
    const Quat q = Quat::from_axis_angle({0.0, 1.0, 1.0}, 0.1);
    const auto x = make_state(5.0e5, {800.0, 20.0, -30.0}, {-120.0, 5.0, 3.0}, q, {0.01, -0.02, 0.03});
    const Vec3 u{8.0e6, 2.0e5, -1.0e5};
    Mat14x14 a;
    Mat14x3 b;
    model.jacobians(x, u, 0, a, b);
    DescentState dx;
    // Small enough that second-order terms (e.g. the bilinear q-omega product) stay below 1e-3 of the change.
    for (int i = 0; i < 14; ++i) dx[i] = 1e-6 * (1.0 + std::abs(x[i])) * ((i % 3) - 1.0);
    const Vec3 du{10.0, -20.0, 5.0};
    const auto f0 = model.derivative(x, u, 0);
    const auto f1 = model.derivative(x + dx, u + du, 0);
    for (int i = 0; i < 14; ++i) {
        double predicted = 0.0;
        for (int j = 0; j < 14; ++j) predicted += a[static_cast<size_t>(i * 14 + j)] * dx[j];
        predicted += b[static_cast<size_t>(i * 3)] * du.x + b[static_cast<size_t>(i * 3 + 1)] * du.y + b[static_cast<size_t>(i * 3 + 2)] * du.z;
        const double actual = f1[i] - f0[i];
        CHECK(std::abs(predicted - actual) <= 1e-3 * std::abs(actual) + 1e-9);
    }
}
