// ISS i3: the phasing planner in the full force model with drag - burn sizes, arrival at Waypoint 0, approach
// ellipsoid kept clear before the approach initiation, budget, refusals; campaign over the phase angle.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "rendezvous_test_support.hpp"
#include "starsim/ephemeris/interpolated_ephemeris.hpp"
#include "starsim/frames/interpolated_orientation.hpp"
#include "starsim/gnc/deorbit_planner.hpp"
#include "starsim/mission/phasing_planner.hpp"
#include "starsim/mission/rendezvous_geometry.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
// [D] ISS: C_D A / m from the OEM header (1.5 x 1177.76 m^2 / 460,320 kg); Dragon: [E] C_D 2.2 x pi (2 m)^2 / 12.5 t
constexpr double iss_beta = 1.5 * 1177.76 / 460320.0;
constexpr double dragon_beta = 2.2 * 3.14159265358979 * 4.0 / 12500.0;
// Crew Dragon on its four side Dracos along +x (i2: 1556 N, 4.25 kg per m/s at 12.5 t -> mass flow 0.53 kg/s)
const mission::FiniteBurnModel dracos{12500.0, 1556.0, 1556.0 / (300.0 * constants::standard_gravity_mps2)};
constexpr double dragon_propellant_kg = 2562.0;

struct Fixture {
    spice::SpiceEphemeris spice_ephemeris;
    spice::SpiceOrientation spice_earth = spice::SpiceOrientation::earth();
    spice::SpiceTimeConverter clock;
    ephemeris::InterpolatedEphemeris ephemeris{spice_ephemeris};
    frames::InterpolatedOrientation earth{spice_earth};
    environment::PointMassGravity gravity{ephemeris, earth};
    mission::PhasingPlanner planner{gravity};

    mission::PhasingParameters parameters() const {
        mission::PhasingParameters p;
        p.chaser_drag_area_per_mass_m2pkg = dragon_beta;
        p.target_drag_area_per_mass_m2pkg = iss_beta;
        return p;
    }
    // A plan whose start plane meets the ISS plane at the boost (two passes, as a launch planner would).
    mission::PhasingPlan plan(double lead_deg, const mission::PhasingParameters& p, rendezvous_test::Start& start) {
        start = rendezvous_test::start(clock, spice_earth, lead_deg, 0.0);
        const auto first = planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, p, dracos, dragon_propellant_kg);
        if (!first.feasible) return first;
        start = rendezvous_test::start(clock, spice_earth, lead_deg, first.schedule.boost_epoch - start.epoch);
        return planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, p, dracos, dragon_propellant_kg);
    }
};

// Flies the plan impulsively-finite through the planner's own dynamics: the arrival at Waypoint 0.
orbit::RelativeState fly_plan(const Fixture& fx, const mission::PhasingPlan& plan, const rendezvous_test::Start& start) {
    const auto& p = plan.parameters;
    math::StateVector dragon = start.dragon, iss = start.iss.state_gcrf;
    time::Epoch t = start.epoch;
    for (const auto& b : plan.burns) {
        const auto node = fx.planner.retarget(b.kind, dragon, iss, t, plan, dracos);
        REQUIRE(node);
        dragon = gnc::coast_with_drag(fx.gravity, dragon, t, node->epoch, p.chaser_drag_area_per_mass_m2pkg);
        iss = gnc::coast_with_drag(fx.gravity, iss, t, node->epoch, p.target_drag_area_per_mass_m2pkg);
        dragon = mission::apply_finite_burn(fx.gravity, dragon, node->epoch, node->delta_v_gcrf_mps, dracos);
        t = node->epoch;
    }
    return orbit::lvlh_relative_state(iss, dragon);  // at the braking burn = the Waypoint 0 epoch
}
}  // namespace

TEST_CASE("Phasing plan from 120 deg: burns, Waypoint 0, budget", "[rendezvous][phasing][validation]") {
    test::spice_session();
    Fixture fx;
    rendezvous_test::Start start;
    const auto plan = fx.plan(120.0, fx.parameters(), start);
    REQUIRE(plan.feasible);
    REQUIRE(plan.burns.size() == 4);  // boost, coelliptic, approach initiation, braking (no phase burn)
    CHECK(plan.burns[0].kind == mission::PhasingBurnKind::Boost);
    CHECK(plan.burns[3].kind == mission::PhasingBurnKind::Braking);
    for (const auto& b : plan.burns) {
        std::printf("[validation] %-22s at +%.2f h: %.2f m/s\n", mission::german_name(b.kind),
                    (b.node.epoch - start.epoch) / 3600.0, norm(b.node.delta_v_gcrf_mps));
    }
    std::printf("[validation] total %.1f m/s, %.0f kg, Waypoint 0 at +%.2f h, AE metric before AI %.2f\n",
                plan.total_delta_v_mps, plan.propellant_kg, (plan.schedule.waypoint0_epoch - start.epoch) / 3600.0,
                plan.min_ellipsoid_metric_before_ai);
    // Hohmann 205 km -> 418 km (OEM ISS 420.7 km - 2.9 km) is ~123 m/s [D]; CW approach initiation ~0.8 m/s, braking
    // ~0.1 m/s [D]; Demo-2 boost + circularisation 102 m/s plus a 16 m/s phase burn [S]
    CHECK(plan.total_delta_v_mps > 115.0);
    CHECK(plan.total_delta_v_mps < 140.0);  // [E] Hohmann + approach + braking + residual plane error
    CHECK(plan.propellant_kg + plan.parameters.reserve_kg < dragon_propellant_kg);
    CHECK(plan.min_ellipsoid_metric_before_ai > 1.0);
    const auto arrival = fly_plan(fx, plan, start);
    std::printf("[validation] Waypoint 0 arrival (LVLH): (%.1f, %.1f, %.1f) m, (%.3f, %.3f, %.3f) m/s\n",
                arrival.position_m.x, arrival.position_m.y, arrival.position_m.z, arrival.velocity_mps.x,
                arrival.velocity_mps.y, arrival.velocity_mps.z);
    CHECK(norm(arrival.position_m - Vec3{0.0, 0.0, plan.parameters.waypoint0_below_m}) < 100.0);  // [E] spec
}

TEST_CASE("Mid-course retarget returns a small correction (the caller skips it), never hides a failure",
          "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    rendezvous_test::Start start;
    const auto plan = fx.plan(120.0, fx.parameters(), start);
    REQUIRE(plan.feasible);
    const auto& p = plan.parameters;
    math::StateVector dragon = start.dragon, iss = start.iss.state_gcrf;
    time::Epoch t = start.epoch;
    for (const auto& b : plan.burns) {  // fly up to and including the approach initiation
        if (b.kind == mission::PhasingBurnKind::Braking) break;
        const auto node = fx.planner.retarget(b.kind, dragon, iss, t, plan, dracos);
        REQUIRE(node);
        dragon = gnc::coast_with_drag(fx.gravity, dragon, t, node->epoch, p.chaser_drag_area_per_mass_m2pkg);
        iss = gnc::coast_with_drag(fx.gravity, iss, t, node->epoch, p.target_drag_area_per_mass_m2pkg);
        dragon = mission::apply_finite_burn(fx.gravity, dragon, node->epoch, node->delta_v_gcrf_mps, dracos);
        t = node->epoch;
    }
    const auto mid = fx.planner.retarget(mission::PhasingBurnKind::MidCourse, dragon, iss, t, plan, dracos);
    REQUIRE(mid);  // converged: the correction is (almost) zero after a perfect approach initiation
    CHECK(norm(mid->delta_v_gcrf_mps) < p.mid_course_threshold_mps);
    CHECK(mid->epoch == plan.schedule.approach_initiation_epoch + p.mid_course_after_s);
}

TEST_CASE("Phasing plan uses a phase burn for a crew-day arrival", "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    auto p = fx.parameters();
    p.earliest_arrival_s = 20.0 * 3600.0;
    rendezvous_test::Start start;
    // 120 deg: a smaller lead cannot be stretched to 20 h (the phase orbit at most halves the catch-up rate)
    const auto plan = fx.plan(120.0, p, start);
    REQUIRE(plan.feasible);
    REQUIRE(plan.burns.front().kind == mission::PhasingBurnKind::Phase);
    CHECK(plan.schedule.waypoint0_epoch - start.epoch >= p.earliest_arrival_s);
    std::printf("[validation] crew day: phase burn %.2f m/s, %d revolutions, total %.1f m/s\n",
                norm(plan.burns.front().node.delta_v_gcrf_mps), plan.schedule.phasing_revolutions,
                plan.total_delta_v_mps);
    CHECK(plan.total_delta_v_mps < 140.0);  // the phase burn is part of the same Hohmann transfer
}

TEST_CASE("Phasing plan refuses a phase beyond the mission limit", "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    const auto start = rendezvous_test::start(fx.clock, fx.spice_earth, 330.0, 0.0);
    const auto plan = fx.planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, fx.parameters(), dracos,
                                      dragon_propellant_kg);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason.find("nicht innerhalb von 28 h") != std::string::npos);
}

TEST_CASE("Phasing plan refuses without the reserve", "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    const auto start = rendezvous_test::start(fx.clock, fx.spice_earth, 90.0, 0.0);
    const auto plan = fx.planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, fx.parameters(), dracos, 1200.0);
    CHECK_FALSE(plan.feasible);
    CHECK(plan.reason.rfind("Kein Rendezvous: Treibstoff", 0) == 0);
}

TEST_CASE("Phasing budget: the settling gas per burn is reserved", "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    const auto start = rendezvous_test::start(fx.clock, fx.spice_earth, 90.0, 0.0);
    const auto plan_with = [&](double reserve_per_burn_kg, double on_board_kg) {
        auto p = fx.parameters();
        p.reserve_per_burn_kg = reserve_per_burn_kg;
        return fx.planner.plan(start.dragon, start.iss.state_gcrf, start.epoch, p, dracos, on_board_kg);
    };
    const auto free = plan_with(0.0, dragon_propellant_kg);
    REQUIRE(free.feasible);
    const double needed_kg =
        free.propellant_kg + free.parameters.reserve_kg + 400.0 * static_cast<double>(free.burns.size());
    const auto short_by_one = plan_with(400.0, needed_kg - 1.0);
    CHECK_FALSE(short_by_one.feasible);
    CHECK(short_by_one.reason.rfind("Kein Rendezvous: Treibstoff reicht nicht", 0) == 0);
    CHECK(plan_with(400.0, needed_kg + 1.0).feasible);
}

TEST_CASE("Phasing plan from above the target: a retrograde boost descends to Waypoint 0", "[rendezvous][phasing]") {
    test::spice_session();
    Fixture fx;
    const auto start = rendezvous_test::start(fx.clock, fx.spice_earth, 90.0, 0.0);
    // the chaser 60 km above the ISS, 40 deg ahead of it, circular in its plane (refuelling f1)
    const auto& iss = start.iss.state_gcrf;
    const Vec3 n = math::normalized(math::cross(iss.position_m, iss.velocity_mps));
    const Vec3 r_hat = math::Quat::from_axis_angle(n, 40.0 * constants::deg_to_rad).rotate(
        math::normalized(iss.position_m));
    const double r = math::norm(iss.position_m) + 60e3;
    const math::StateVector chaser{r * r_hat, std::sqrt(gm / r) * math::cross(n, r_hat)};
    auto p = fx.parameters();
    p.mission_limit_s = 48.0 * 3600.0;
    const auto plan = fx.planner.plan(chaser, iss, start.epoch, p, dracos, dragon_propellant_kg);
    REQUIRE(plan.feasible);
    CHECK(plan.schedule.above);
    REQUIRE(plan.burns.front().kind == mission::PhasingBurnKind::Boost);
    const auto boost = fx.planner.retarget(mission::PhasingBurnKind::Boost, chaser, iss, start.epoch, plan, dracos);
    REQUIRE(boost);
    const auto at_boost = gnc::coast_with_drag(fx.gravity, chaser, start.epoch, boost->epoch,
                                               p.chaser_drag_area_per_mass_m2pkg);
    CHECK(math::dot(boost->delta_v_gcrf_mps, at_boost.velocity_mps) < 0.0);  // retrograde
    rendezvous_test::Start flown = start;
    flown.dragon = chaser;
    const auto rel = fly_plan(fx, plan, flown);
    CHECK(math::norm(rel.position_m - Vec3{0.0, 0.0, p.waypoint0_below_m}) < 100.0);
}

TEST_CASE("Phasing campaign over the phase angle", "[rendezvous][phasing][validation]") {
    test::spice_session();
    Fixture fx;
    auto p = fx.parameters();
    p.mission_limit_s = 48.0 * 3600.0;  // every phase closes within two days
    for (double lead = 0.0; lead < 360.0; lead += 30.0) {
        CAPTURE(lead);
        rendezvous_test::Start start;
        const auto plan = fx.plan(lead, p, start);
        REQUIRE(plan.feasible);
        const auto arrival = fly_plan(fx, plan, start);
        const double miss = norm(arrival.position_m - Vec3{0.0, 0.0, p.waypoint0_below_m});
        std::printf("[validation] lead %3.0f deg: %d lap(s), Waypoint 0 +%5.2f h, %.1f m/s, AE %.2f, miss %.1f m\n",
                    lead, plan.schedule.extra_laps, (plan.schedule.waypoint0_epoch - start.epoch) / 3600.0,
                    plan.total_delta_v_mps, plan.min_ellipsoid_metric_before_ai, miss);
        CHECK(plan.total_delta_v_mps < 140.0);
        CHECK(plan.min_ellipsoid_metric_before_ai > 1.0);
        CHECK(miss < 100.0);
    }
}
