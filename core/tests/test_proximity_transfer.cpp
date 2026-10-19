// ISS i4: CW proximity transfers with keep-out clearance, the breakout burn, and CW against the full model.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "rendezvous_test_support.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/gnc/departure_guidance.hpp"
#include "starsim/gnc/proximity_transfer.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;
constexpr double n_iss = 1.128e-3;  // [D] mean motion at ~420 km

TEST_CASE("Waypoint 0 to Waypoint 1 (IDA-2) clears the keep-out sphere", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const orbit::RelativeState wp0{{0.0, 0.0, 400.0}, {}};
    const Vec3 wp1_cg{35.0 + 220.0, 0.0, 0.0};  // IDA-2 35 m ahead of the CG [E config], 220 m out
    const auto t = gnc::plan_proximity_transfer(wp0, wp1_cg, n_iss, p.transfer_s, p);
    REQUIRE(t.feasible);
    CHECK(t.min_cg_range_m >= 210.0);
    const auto end = orbit::cw_propagate({wp0.position_m, t.departure_velocity_lvlh_mps}, n_iss, t.duration_s);
    CHECK(norm(end.position_m - wp1_cg) < 1e-6);
    std::printf("[validation] WP0->WP1 CW transfer %.0f s, dv %.3f m/s, closest %.1f m\n", t.duration_s,
                norm(t.departure_velocity_lvlh_mps), t.min_cg_range_m);
}

TEST_CASE("A transfer through the sphere is refused", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    // straight through the station: from 400 m below to 400 m above in a short time
    const auto t = gnc::plan_proximity_transfer({{0.0, 0.0, 400.0}, {}}, {0.0, 0.0, -400.0}, n_iss, 300.0, p);
    CHECK_FALSE(t.feasible);
}

TEST_CASE("Breakout from the final approach is passively safe for 24 h (CW)", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    for (const Vec3 at : {Vec3{45.0, 0.0, 0.0}, Vec3{28.0, 0.0, -15.0}}) {  // 10 m out on IDA-2 / IDA-3 (CG-relative)
        CAPTURE(at.x, at.z);
        const orbit::RelativeState from{at, {-0.08 * at.x / norm(at), 0.0, -0.08 * at.z / norm(at)}};
        const auto b = gnc::breakout_delta_v(from, n_iss, p);
        REQUIRE(b.safe);
        CHECK(norm(b.delta_v_lvlh_mps) <= 1.5);
        CHECK(b.exit_s <= p.breakout.exit_limit_s);
        std::printf("[validation] breakout from (%.0f,%.0f,%.0f): dv %.2f m/s (%.2f,%.2f,%.2f), exit %.0f s, "
                    "closest after exit %.0f m\n", at.x, at.y, at.z, norm(b.delta_v_lvlh_mps), b.delta_v_lvlh_mps.x,
                    b.delta_v_lvlh_mps.y, b.delta_v_lvlh_mps.z, b.exit_s, b.min_range_after_exit_m);
    }
}

TEST_CASE("CW against the full model over a proximity transfer and a breakout", "[iss][proximity][validation]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    const auto s = rendezvous_test::start(clock, earth, 0.0, 0.0);  // ISS from the OEM fixture
    const environment::PointMassGravity gravity{ephemeris, earth};  // as test_phasing_planner (Earth J2 + Moon + Sun)
    const auto& iss = s.iss.state_gcrf;
    const double n = std::sqrt(constants::earth_gm_m3ps2 / std::pow(norm(iss.position_m), 3));
    const double beta_iss = 1177.76 * 1.5 / 460320.0, beta_dragon = 2.2 * constants::pi * 2.0 * 2.0 / 12500.0;  // [D]
    const gnc::ProximityParameters p;
    // 1) the WP0 -> WP1 transfer: CW arrival against the full model
    const orbit::RelativeState wp0{{0.0, 0.0, 400.0}, {}};
    const auto t = gnc::plan_proximity_transfer(wp0, {255.0, 0.0, 0.0}, n, p.transfer_s, p);
    REQUIRE(t.feasible);
    const auto chaser0 = orbit::chaser_from_lvlh(iss, {wp0.position_m, t.departure_velocity_lvlh_mps});
    const auto t1 = s.epoch + t.duration_s;
    const auto iss1 = gnc::coast_with_drag(gravity, iss, s.epoch, t1, beta_iss);
    const auto chaser1 = gnc::coast_with_drag(gravity, chaser0, s.epoch, t1, beta_dragon);
    const auto rel = orbit::lvlh_relative_state(iss1, chaser1);
    const double miss_m = norm(rel.position_m - Vec3{255.0, 0.0, 0.0});
    std::printf("[validation] CW vs full model, WP0->WP1 %.0f s: miss %.3f m\n", t.duration_s, miss_m);
    CHECK(miss_m < 2.0);  // spec correction 1: the loop's arrival tolerance; expected centimetres to decimetres
    // 2) a breakout from 10 m out on IDA-2: never back inside the sphere for 24 h in the full model
    const orbit::RelativeState from{{45.0, 0.0, 0.0}, {-0.08, 0.0, 0.0}};
    const auto b = gnc::breakout_delta_v(from, n, p);
    REQUIRE(b.safe);
    auto c = orbit::chaser_from_lvlh(iss, {from.position_m, from.velocity_mps + b.delta_v_lvlh_mps});
    auto st = iss;
    double closest_after_exit = 1e9;
    for (double k = 60.0; k <= 86400.0; k += 60.0) {
        st = gnc::coast_with_drag(gravity, st, s.epoch + (k - 60.0), s.epoch + k, beta_iss);
        c = gnc::coast_with_drag(gravity, c, s.epoch + (k - 60.0), s.epoch + k, beta_dragon);
        if (k > b.exit_s) closest_after_exit = std::min(closest_after_exit, norm(c.position_m - st.position_m));
    }
    std::printf("[validation] breakout full model 24 h: closest after the exit %.0f m\n", closest_after_exit);
    CHECK(closest_after_exit > p.keep_out_radius_m);
}

TEST_CASE("Breakout with the approach-ellipsoid exit: departure from IDA-2", "[iss][proximity][docking]") {
    gnc::ProximityParameters p;
    p.breakout.ellipsoid_semi_axes_m = Vec3{2000.0, 1000.0, 1000.0};
    // 10 m in front of IDA-2 (station CG to port 35 m on V-bar, Dragon CG 4 m behind its port), opening at 0.15 m/s
    const orbit::RelativeState from{{49.0, 0.0, 0.0}, {0.15, 0.0, 0.0}};
    const auto b = gnc::breakout_delta_v(from, 1.128e-3, p);
    REQUIRE(b.safe);
    CHECK(b.ellipsoid_exit_s > 0.0);
    CHECK(b.ellipsoid_exit_s <= p.breakout.ellipsoid_exit_limit_s);
    CHECK(b.min_range_after_exit_m >= p.keep_out_radius_m + p.kos_margin_m);
    // without the ellipsoid requirement the smallest safe magnitude may stay inside the ellipsoid longer
    gnc::ProximityParameters q;
    const auto plain = gnc::breakout_delta_v(from, 1.128e-3, q);
    REQUIRE(plain.safe);
    CHECK(norm(plain.delta_v_lvlh_mps) <= norm(b.delta_v_lvlh_mps) + 1e-12);
}

TEST_CASE("The departure ellipsoid equals the mission's approach ellipsoid", "[iss][docking]") {
    const gnc::DepartureParameters d;
    CHECK(d.ellipsoid_semi_axes_m.x == mission::approach_ellipsoid_semi_axes_m[0]);
    CHECK(d.ellipsoid_semi_axes_m.y == mission::approach_ellipsoid_semi_axes_m[1]);
    CHECK(d.ellipsoid_semi_axes_m.z == mission::approach_ellipsoid_semi_axes_m[2]);
}
