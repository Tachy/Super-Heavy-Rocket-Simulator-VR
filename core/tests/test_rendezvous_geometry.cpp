// ISS i3: target-relative aim points, approach ellipsoid and the drag term of the position targeter.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/position_targeter.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
const double radius_m = constants::wgs84_semi_major_axis_m + 415e3;
// A circular target in an inclined plane (51.6 deg about x).
math::StateVector target_state() {
    const double v = std::sqrt(gm / radius_m), i = 51.6 * constants::deg_to_rad;
    return {{radius_m, 0.0, 0.0}, {0.0, v * std::cos(i), v * std::sin(i)}};
}
}  // namespace

TEST_CASE("Aim point behind and below the target", "[rendezvous]") {
    const auto target = target_state();
    const Vec3 p = mission::point_behind_below(target, 6900.0, 2900.0);
    CHECK(norm(p) == Catch::Approx(radius_m - 2900.0).margin(1e-6));
    const auto rel = orbit::lvlh_relative_state(target, {p, target.velocity_mps});
    CHECK(rel.position_m.x == Catch::Approx(-6900.0).margin(5.0));  // curvilinear: arc length 6.9 km behind
    CHECK(rel.position_m.z == Catch::Approx(2900.0).margin(5.0));   // nadir is +z
    CHECK(std::abs(rel.position_m.y) < 1e-6);
    CHECK(mission::in_plane_angle_rad(target, p) == Catch::Approx(-6900.0 / radius_m).epsilon(1e-9));
    const Vec3 wp0 = mission::point_behind_below(target, 0.0, 400.0);
    CHECK(norm(wp0 - target.position_m * ((radius_m - 400.0) / radius_m)) < 1e-6);
}

TEST_CASE("Approach ellipsoid metric", "[rendezvous]") {
    const auto target = target_state();
    const auto at = [&](Vec3 lvlh) { return orbit::chaser_from_lvlh(target, {lvlh, {}}).position_m; };
    CHECK(mission::approach_ellipsoid_metric(target, at({2000.0, 0.0, 0.0})) == Catch::Approx(1.0).epsilon(1e-6));
    CHECK(mission::approach_ellipsoid_metric(target, at({0.0, 0.0, 1000.0})) == Catch::Approx(1.0).epsilon(1e-6));
    CHECK(mission::approach_ellipsoid_metric(target, at({0.0, 0.0, 400.0})) < 1.0);  // Waypoint 0 lies inside
    CHECK(mission::approach_ellipsoid_metric(target, at({-6900.0, 0.0, 2900.0})) > 1.0);
}

TEST_CASE("Position targeter with orbital drag", "[rendezvous][phasing]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const environment::PointMassGravity gravity{ephemeris, earth};
    const time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 10, 16, 0, 0, 0.0});
    const double low = constants::wgs84_semi_major_axis_m + 205e3;
    const double i = 51.6 * constants::deg_to_rad, v = std::sqrt(gm / low);
    const math::StateVector chaser{{low, 0.0, 0.0}, {0.0, v * std::cos(i), v * std::sin(i)}};
    constexpr double beta = 2.2e-3;  // [E] Dragon-like C_D A / m
    mission::PositionTargeter targeter{gravity};
    // Without drag nothing changes: the coast equals the drag-free propagator.
    CHECK(norm(targeter.coast(chaser, t0, t0 + 3600.0).position_m -
               gnc::coast_with_drag(gravity, chaser, t0, t0 + 3600.0, 0.0).position_m) < 0.05);
    targeter.set_drag_area_per_mass(beta);
    const auto dragged = targeter.coast(chaser, t0, t0 + 6.0 * 3600.0);
    CHECK(norm(dragged.position_m - gnc::coast_with_drag(gravity, chaser, t0, t0 + 6.0 * 3600.0, beta).position_m) < 0.05);
    // A 3 h aim point taken from a drag-free coast with a 1 m/s prograde burn: the drag-aware solution hits it.
    const auto at_burn = targeter.coast(chaser, t0, t0 + 600.0);
    math::StateVector free = gnc::coast_with_drag(gravity, chaser, t0, t0 + 600.0, 0.0);
    free.velocity_mps += normalized(free.velocity_mps);
    const mission::PositionTarget aim{gnc::coast_with_drag(gravity, free, t0 + 600.0, t0 + 3.0 * 3600.0, 0.0).position_m,
                                      t0 + 3.0 * 3600.0};
    const auto solution = targeter.solve(chaser, t0, t0 + 600.0, aim, normalized(at_burn.velocity_mps), 1.0);
    CHECK(solution.converged);
    CHECK(solution.miss_m < 1.0);
}
