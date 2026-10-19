// ISS i3: Clohessy-Wiltshire relative motion in the target's LVLH frame (x V-bar, y -orbit normal, z nadir).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/orbit/kepler.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
const double radius_m = constants::wgs84_semi_major_axis_m + 415e3;
const double n = std::sqrt(gm / (radius_m * radius_m * radius_m));

math::StateVector circular_target() { return {{radius_m, 0.0, 0.0}, {0.0, std::sqrt(gm / radius_m), 0.0}}; }
}  // namespace

TEST_CASE("CW: a coelliptic chaser below drifts forward at 1.5 n dh", "[rendezvous][cw]") {
    const orbit::RelativeState below{{0.0, 0.0, 2900.0}, {1.5 * n * 2900.0, 0.0, 0.0}};
    const auto later = orbit::cw_propagate(below, n, 3600.0);
    CHECK(later.position_m.z == Catch::Approx(2900.0).margin(1e-6));
    CHECK(later.position_m.x == Catch::Approx(1.5 * n * 2900.0 * 3600.0).epsilon(1e-12));
    CHECK(later.velocity_mps.z == Catch::Approx(0.0).margin(1e-9));
}

TEST_CASE("CW: V-bar points are equilibria, out-of-plane motion oscillates", "[rendezvous][cw]") {
    const auto held = orbit::cw_propagate({{-400.0, 0.0, 0.0}, {}}, n, 5000.0);
    CHECK(norm(held.position_m - Vec3{-400.0, 0.0, 0.0}) < 1e-9);
    const double period = 2.0 * constants::pi / n;
    const auto lateral = orbit::cw_propagate({{0.0, 100.0, 0.0}, {}}, n, 0.5 * period);
    CHECK(lateral.position_m.y == Catch::Approx(-100.0).margin(1e-6));
}

TEST_CASE("CW: the transfer velocity reaches the aim point", "[rendezvous][cw]") {
    const Vec3 from{-6900.0, 30.0, 2900.0}, to{0.0, 0.0, 400.0};
    for (const double t : {600.0, 1500.0, 2700.0, 4000.0}) {
        CAPTURE(t);
        const auto v = orbit::cw_transfer_velocity(from, to, n, t);
        REQUIRE(v);
        const auto arrival = orbit::cw_propagate({from, *v}, n, t);
        CHECK(norm(arrival.position_m - to) < 1e-6);
    }
    const double period = 2.0 * constants::pi / n;
    CHECK_FALSE(orbit::cw_transfer_velocity(from, to, n, period));  // in-plane singular after a full revolution
}

TEST_CASE("CW agrees with two-body motion near the target", "[rendezvous][cw]") {
    // A chaser 2.9 km below and 6.9 km behind; both coasted two-body for 45 min against the CW prediction.
    const auto target = circular_target();
    const orbit::RelativeState start{{-6900.0, 50.0, 2900.0}, {1.5 * n * 2900.0, 0.02, 0.1}};
    const auto chaser = orbit::chaser_from_lvlh(target, start);
    const auto back = orbit::lvlh_relative_state(target, chaser);
    CHECK(norm(back.position_m - start.position_m) < 1e-6);
    CHECK(norm(back.velocity_mps - start.velocity_mps) < 1e-9);
    const double t = 2700.0;
    const auto truth = orbit::lvlh_relative_state(orbit::propagate_two_body(target, gm, t),
                                                  orbit::propagate_two_body(chaser, gm, t));
    const auto cw = orbit::cw_propagate(start, n, t);
    // [D] rectilinear CW: 6.9 km behind on the straight V-bar sits x^2 / 2r = 3.5 m above the curved orbit, which the
    // 6 (tau - sin tau) z term turns into ~40 m along track after 45 min (measured 42.7 m); plus ~7 m second order in dh
    CHECK(norm(truth.position_m - cw.position_m) < 60.0);
}
