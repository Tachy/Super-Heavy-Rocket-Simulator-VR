// ISS i3: analytic phasing schedule (wait on the separation orbit, or slow down with a phase burn).

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/math/constants.hpp"
#include "starsim/mission/phasing_schedule.hpp"
#include "starsim/orbit/kepler.hpp"

using namespace starsim;

namespace {
constexpr double gm = constants::earth_gm_m3ps2;
constexpr double deg = constants::deg_to_rad;
const double re = constants::wgs84_semi_major_axis_m;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

math::StateVector circular(double height_m, double argument_of_latitude_rad) {
    orbit::KeplerianElements e;
    e.semi_major_axis_m = re + height_m;
    e.inclination_rad = 51.64 * deg;
    e.raan_rad = 1.0;
    e.true_anomaly_rad = argument_of_latitude_rad;
    return orbit::state_from_elements(e, gm);
}
}  // namespace

TEST_CASE("Schedule: the Dragon waits on the separation orbit and arrives in time", "[rendezvous][phasing]") {
    const auto target = circular(415e3, 1.0);
    for (const double lead_deg : {30.0, 120.0, 240.0}) {
        CAPTURE(lead_deg);
        const auto chaser = circular(205e3, 1.0 - lead_deg * deg);  // the ISS is lead_deg ahead
        const auto s = mission::phasing_schedule(chaser, target, t0, {}, gm);
        REQUIRE(s.feasible);
        CHECK(s.phasing_revolutions == 0);  // no phase burn: the earliest window
        CHECK(s.extra_laps == 0);
        CHECK(s.boost_epoch - t0 >= mission::PhasingParameters{}.first_burn_lead_s);
        CHECK(s.waypoint0_epoch - t0 <= mission::PhasingParameters{}.mission_limit_s);
        // ~11 deg/h catch-up at 205 km against 415 km: the wait grows with the lead angle
        CHECK((s.boost_epoch - t0) / 3600.0 == Catch::Approx(lead_deg / 11.2).margin(1.0));
        CHECK(s.coelliptic_epoch - s.boost_epoch > 2700.0);  // half a transfer orbit (~46 min)
        CHECK(s.approach_initiation_epoch - s.coelliptic_epoch == Catch::Approx(10080.0));
        CHECK(s.waypoint0_epoch - s.approach_initiation_epoch == Catch::Approx(2700.0));
    }
}

TEST_CASE("Schedule: an earliest arrival slows the catch-up with a phase burn", "[rendezvous][phasing]") {
    const auto target = circular(415e3, 1.0);
    // 120 deg lead: the earliest window arrives after ~14 h. An apoapsis raise capped at the coelliptic height at most
    // halves the catch-up rate (11.2 -> 5.7 deg/h), so a much smaller lead cannot be stretched to 20 h at all.
    const auto chaser = circular(205e3, 1.0 - 120.0 * deg);
    mission::PhasingParameters p;
    p.earliest_arrival_s = 20.0 * 3600.0;  // a crew day: not before 20 h
    const auto s = mission::phasing_schedule(chaser, target, t0, p, gm);
    REQUIRE(s.feasible);
    CHECK(s.phasing_revolutions >= 1);
    CHECK(s.phase_epoch - t0 == Catch::Approx(p.first_burn_lead_s));
    CHECK(s.phase_orbit_apoapsis_radius_m > re + 205e3);
    CHECK(s.phase_orbit_apoapsis_radius_m < re + 415e3 - p.coelliptic_height_m);
    CHECK(s.waypoint0_epoch - t0 >= p.earliest_arrival_s);
    CHECK(s.waypoint0_epoch - t0 <= p.mission_limit_s);
}

TEST_CASE("Schedule: a phase beyond the mission limit is refused, a longer limit takes an extra lap", "[rendezvous][phasing]") {
    const auto target = circular(415e3, 1.0);
    const auto chaser = circular(205e3, 1.0 - 330.0 * deg);
    const auto refused = mission::phasing_schedule(chaser, target, t0, {}, gm);
    CHECK_FALSE(refused.feasible);
    CHECK(refused.reason.rfind("Kein Rendezvous", 0) == 0);
    // The ISS just ahead (0.05 deg): the coelliptic geometry needs more lead - one more lap.
    mission::PhasingParameters two_days;
    two_days.mission_limit_s = 48.0 * 3600.0;
    const auto lap = mission::phasing_schedule(circular(205e3, 1.0 - 0.05 * deg), target, t0, two_days, gm);
    REQUIRE(lap.feasible);
    CHECK(lap.extra_laps == 1);
}

TEST_CASE("Schedule refuses a chaser between the coelliptic height and the target", "[rendezvous][phasing]") {
    const auto target = circular(415e3, 1.0);
    const auto s = mission::phasing_schedule(circular(414e3, 0.5), target, t0, {}, gm);
    CHECK_FALSE(s.feasible);
    CHECK(s.reason.rfind("Kein Rendezvous", 0) == 0);
    orbit::KeplerianElements hyperbolic;
    hyperbolic.semi_major_axis_m = -2.0e7;
    hyperbolic.eccentricity = 1.5;
    CHECK_FALSE(mission::phasing_schedule(orbit::state_from_elements(hyperbolic, gm), target, t0, {}, gm).feasible);
    // 200 x 500 km: the semi-major axis is below the coelliptic height, the apoapsis crosses the station's (i3 review)
    orbit::KeplerianElements eccentric;
    eccentric.semi_major_axis_m = re + 350e3;
    eccentric.eccentricity = 150e3 / (re + 350e3);
    eccentric.inclination_rad = 51.64 * deg;
    eccentric.raan_rad = 1.0;
    const auto e = mission::phasing_schedule(orbit::state_from_elements(eccentric, gm), target, t0, {}, gm);
    CHECK_FALSE(e.feasible);
    CHECK(e.reason == "Kein Rendezvous: Bahn kreuzt die Zielbahn");
}

TEST_CASE("Schedule: a chaser above the target waits for the target to catch up, then descends",
          "[rendezvous][phasing]") {
    mission::PhasingParameters p;
    p.mission_limit_s = 48.0 * 3600.0;
    const auto target = circular(200e3, 1.0);
    for (const double ahead_deg : {10.0, 40.0, 80.0}) {  // the chaser ahead of the target
        CAPTURE(ahead_deg);
        const auto chaser = circular(260e3, 1.0 + ahead_deg * deg);
        const auto s = mission::phasing_schedule(chaser, target, t0, p, gm);
        REQUIRE(s.feasible);
        CHECK(s.above);
        CHECK(s.phasing_revolutions == 0);
        CHECK(s.boost_epoch - t0 >= p.first_burn_lead_s);
        CHECK(s.waypoint0_epoch - t0 <= p.mission_limit_s);
    }
    // the further ahead, the longer the wait (the target catches up at a constant rate)
    const auto near = mission::phasing_schedule(circular(260e3, 1.0 + 10.0 * deg), target, t0, p, gm);
    const auto far = mission::phasing_schedule(circular(260e3, 1.0 + 80.0 * deg), target, t0, p, gm);
    CHECK(far.boost_epoch - near.boost_epoch > 0.0);
}

TEST_CASE("Schedule: an above orbit whose periapsis crosses the target's is refused", "[rendezvous][phasing]") {
    orbit::KeplerianElements e;  // 195 x 300 km around a 200 km target: the periapsis dips below
    e.semi_major_axis_m = re + 247.5e3;
    e.eccentricity = 52.5e3 / (re + 247.5e3);
    e.inclination_rad = 51.64 * deg;
    e.raan_rad = 1.0;
    const auto s = mission::phasing_schedule(orbit::state_from_elements(e, gm), circular(200e3, 1.0), t0, {}, gm);
    CHECK_FALSE(s.feasible);
    CHECK(s.reason == "Kein Rendezvous: Bahn kreuzt die Zielbahn");
}

TEST_CASE("Schedule: the catch-up rate matches dn = -1.5 n da / a", "[rendezvous][phasing]") {
    // the boosts of two leads 20 deg apart differ by 20 deg / |dn|
    mission::PhasingParameters p;
    p.mission_limit_s = 96.0 * 3600.0;
    const auto target = circular(200e3, 1.0);
    const auto a = mission::phasing_schedule(circular(170e3, 1.0 - 30.0 * deg), target, t0, p, gm);
    const auto b = mission::phasing_schedule(circular(170e3, 1.0 - 50.0 * deg), target, t0, p, gm);
    REQUIRE(a.feasible);
    REQUIRE(b.feasible);
    const double r = re + 185e3, n = std::sqrt(gm / (r * r * r));
    const double dn = 1.5 * n * 30e3 / r;  // rad/s, two-body at the mean radius (J2 changes it by < 1 %)
    CHECK(b.boost_epoch - a.boost_epoch == Catch::Approx(20.0 * deg / dn).epsilon(0.02));
}
