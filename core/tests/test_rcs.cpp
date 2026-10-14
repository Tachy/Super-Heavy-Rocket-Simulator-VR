// Individual RCS thrusters: allocation, modulation, vehicle integration.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/math/constants.hpp"
#include "starsim/propulsion/rcs_allocator.hpp"
#include "starsim/propulsion/rcs_modulator.hpp"

using namespace starsim;
using math::Vec3;
using propulsion::RcsAllocator;
using propulsion::RcsThrusterGeometry;

namespace {

constexpr double pi = constants::pi;

// A pod ring: at each azimuth one thruster each tangential +, tangential - and inward.
std::vector<RcsThrusterGeometry> ring(double station, double radius, double thrust, std::initializer_list<double> az_deg) {
    std::vector<RcsThrusterGeometry> out;
    for (double a_deg : az_deg) {
        const double a = a_deg * pi / 180.0;
        const Vec3 r{0.0, std::cos(a), std::sin(a)}, t{0.0, -std::sin(a), std::cos(a)};
        const Vec3 p = Vec3{station, 0.0, 0.0} + r * radius;
        out.push_back({p, t, thrust});
        out.push_back({p, t * -1.0, thrust});
        out.push_back({p, r * -1.0, thrust});
    }
    return out;
}

}  // namespace

TEST_CASE("RCS allocator: roll capability of a four-pod ring is four tangential thrusters", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    CHECK(a.max_torque_nm().x == Catch::Approx(4 * 3000.0 * 4.5).epsilon(0.02));
    // Pitch at 38 m: the two inward thrusters on one side plus one tangential per pod whose rolls cancel - six
    // thrusters at cos 45 deg each.
    CHECK(a.max_torque_nm().y == Catch::Approx(6 * 3000.0 * std::cos(pi / 4) * 38.0).epsilon(0.05));
}

TEST_CASE("RCS allocator: a pure torque demand gives that torque and little force", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const Vec3 want{20000.0, -50000.0, 30000.0};
    const auto d = a.allocate(want, {});
    for (double x : d) CHECK((x >= 0.0 && x <= 1.0));
    const Vec3 got = a.torque_nm(d);
    CHECK(norm(got - want) < 0.01 * norm(want));
}

TEST_CASE("RCS allocator: torque has priority over force when saturated", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const Vec3 roll{0.9 * a.max_torque_nm().x, 0.0, 0.0};
    const auto d = a.allocate(roll, Vec3{0.0, 1e5, 0.0});  // force far beyond reach
    CHECK(a.torque_nm(d).x == Catch::Approx(roll.x).epsilon(0.03));
}

TEST_CASE("RCS allocator: no thruster along an axis gives zero capability, not NaN", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const double fx = a.force_capability_n({1.0, 0.0, 0.0});
    CHECK(std::isfinite(fx));
    CHECK(fx < 1.0);
    const RcsAllocator empty({}, 0.0);
    CHECK(norm(empty.max_torque_nm()) == 0.0);
    CHECK(empty.allocate({1.0, 0.0, 0.0}, {}).empty());
    const std::vector<RcsThrusterGeometry> starved = ring(68.0, 4.5, 0.0, {45, 135, 225, 315});
    CHECK(norm(RcsAllocator(starved, 30.0).max_torque_nm()) == 0.0);
}

TEST_CASE("RCS allocator: lateral force without torque from a symmetric pair of rings", "[rcs]") {
    auto t = ring(68.0, 4.5, 3000.0, {45, 135, 225, 315});
    const auto aft = ring(8.0, 4.5, 3000.0, {45, 135, 225, 315});
    t.insert(t.end(), aft.begin(), aft.end());
    const RcsAllocator a(t, 38.0);
    std::vector<double> d;
    const double f = a.force_capability_n({0.0, 1.0, 0.0}, &d);
    CHECK(f > 3000.0);
    CHECK(norm(a.torque_nm(d)) < 0.02 * f * 30.0);
}

TEST_CASE("RCS modulator: the mean on-time follows the duty, pulses are never shorter than the minimum", "[rcs]") {
    constexpr double dt = 0.005, min_on = 0.02;
    for (const double duty : {0.0, 0.03, 0.1, 0.37, 0.8, 1.0}) {
        CAPTURE(duty);
        propulsion::RcsModulator m(1, min_on);
        int on = 0, run = 0, shortest = 1 << 30;
        bool previous = false;
        const int ticks = 2000;  // 10 s
        for (int k = 0; k < ticks; ++k) {
            const bool f = m.update({duty}, dt)[0];
            on += f ? 1 : 0;
            if (f) ++run;
            if (!f && previous) {
                shortest = std::min(shortest, run);
                run = 0;
            }
            previous = f;
        }
        CHECK(on * dt == Catch::Approx(duty * ticks * dt).margin(min_on + dt));
        if (duty > 0.0 && duty < 1.0) CHECK(shortest >= static_cast<int>(std::lround(min_on / dt)));
        if (duty == 1.0) CHECK(on == ticks);
    }
}

TEST_CASE("RCS modulator: a duty that drops to zero leaves nothing owed", "[rcs]") {
    propulsion::RcsModulator m(1, 0.02);
    for (int k = 0; k < 3; ++k) m.update({0.5}, 0.005);  // owes 7.5 ms, not yet a pulse
    m.update({0.0}, 0.005);
    for (int k = 0; k < 2; ++k) CHECK_FALSE(m.update({0.3}, 0.005)[0]);  // starts from zero again
}
