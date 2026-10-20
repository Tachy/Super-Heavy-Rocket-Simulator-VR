// ISS i6: the launch-window planner - plane bias, separation-state estimate, the liftoff at which the ascent plane is
// the biased ISS plane, phase feasibility and the refusal.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "rendezvous_test_support.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/refuelling_mission.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
constexpr double gm = constants::earth_gm_m3ps2;

struct Fixture {
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    const orbit::OemArchive archive = orbit::OemArchive::scan(rendezvous_test::fixture_dir, clock);
    const config::VehicleConfig iss = config::load_vehicle(rendezvous_test::config_dir, "iss");
    const config::LaunchSiteConfig site = config::load_launch_site(rendezvous_test::config_dir, "ccsfs_slc40");
    Vec3 pad_itrf() const { return frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m}); }
    mission::StationStateAt station() const {
        return [this](time::Epoch t) {
            return sim::iss_start(iss, archive, clock, earth, t, sim::IssOrbitSource::Oem).state_gcrf;
        };
    }
    mission::LaunchWindowParameters parameters() const {
        mission::LaunchWindowParameters p;
        p.inclination_rad = 51.64 * deg;
        p.insertion_s = 720.0;
        p.insertion_arc_rad = 30.0 * deg;
        return p;
    }
};
}  // namespace

TEST_CASE("Launch window: the plane bias makes both nodes meet after the match time", "[iss][launch_window]") {
    const double a_c = constants::wgs84_semi_major_axis_m + 205e3, a_s = constants::wgs84_semi_major_axis_m + 415e3;
    const double i = 51.64 * deg;
    const double bias = mission::plane_bias_rad(a_c, a_s, i, 86400.0);
    // ~0.58 deg/day faster regression at 205 km (i3 correction 4): the chaser starts east of the station's node
    CHECK(bias / deg == Catch::Approx(0.58).margin(0.05));
    CHECK(mission::plane_bias_rad(a_c, a_s, i, 0.0) == 0.0);
}

TEST_CASE("Launch window: separation state in the plane, at the arc, on the 200 x 210 km orbit",
          "[iss][launch_window]") {
    mission::LaunchWindowParameters p;
    p.insertion_arc_rad = 30.0 * deg;
    const Vec3 pad{6.0e6, 0.0, 0.0};
    const Vec3 n = math::normalized(Vec3{0.0, -std::sin(51.6 * deg), std::cos(51.6 * deg)});
    const auto s = mission::separation_state_estimate(pad, n, p, gm);
    CHECK(std::abs(dot(s.position_m, n)) < 1e-6);
    CHECK(std::abs(dot(s.velocity_mps, n)) < 1e-9);
    CHECK(std::acos(dot(math::normalized(s.position_m), math::normalized(pad))) / deg == Catch::Approx(30.0));
    CHECK(norm(s.position_m) == Catch::Approx(p.separation_periapsis_radius_m));
    CHECK(dot(cross(s.position_m, s.velocity_mps), n) > 0.0);  // prograde about the plane normal
    const auto e = orbit::elements_from_state(s, gm);
    CHECK((e.semi_major_axis_m * (1.0 + e.eccentricity) - constants::wgs84_semi_major_axis_m) / 1000.0 ==
          Catch::Approx(210.0).margin(0.01));
}

TEST_CASE("Launch window: at liftoff the ascent plane is the biased ISS plane", "[iss][launch_window]") {
    test::spice_session();
    const Fixture f;
    const time::Epoch earliest = f.clock.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.station(), earliest, f.parameters(), gm);
    INFO(w.reason);
    REQUIRE(w.feasible);
    CHECK(w.liftoff - earliest >= 0.0);
    // one northerly crossing per sidereal day; a rejected one (phase out of reach in 28 h) moves it a day on
    CHECK(w.liftoff - earliest < 86164.1 * (w.rejected + 1));
    CHECK(w.liftoff - earliest > 86164.1 * w.rejected - 3600.0);
    const Vec3 pad = f.earth.fixed_to_gcrf(w.liftoff).position(f.pad_itrf());
    const Vec3 n_ascent = gnc::ascent_plane_normal(pad, f.earth, w.liftoff, 51.64 * deg, true);
    // the target plane: the ISS plane at liftoff turned by the bias about the Earth's pole
    const auto iss = f.station()(w.liftoff);
    const Vec3 pole = f.earth.fixed_to_gcrf(w.liftoff).rotation * Vec3{0.0, 0.0, 1.0};
    const Vec3 n_iss = math::normalized(cross(iss.position_m, iss.velocity_mps));
    const Vec3 n_target = math::Quat::from_axis_angle(pole, w.plane_bias_rad).rotate(n_iss);
    CHECK(std::acos(std::clamp(dot(n_ascent, n_target), -1.0, 1.0)) / deg < 0.01);
    CHECK(w.plane_bias_rad > 0.0);
    CHECK(w.separation - w.liftoff == Catch::Approx(720.0));
    CHECK(w.schedule.feasible);
    CHECK(w.schedule.waypoint0_epoch - w.liftoff < 28.0 * 3600.0);
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("[launch_window] liftoff +%.3f h, bias %.3f deg, Waypoint 0 %.1f h after liftoff, rejected %d\n",
                    (w.liftoff - earliest) / 3600.0, w.plane_bias_rad / deg,
                    (w.schedule.waypoint0_epoch - w.liftoff) / 3600.0, w.rejected);
    }
}

TEST_CASE("Launch window: a phase that never closes in a short limit is refused in German", "[iss][launch_window]") {
    test::spice_session();
    const Fixture f;
    auto p = f.parameters();
    p.phasing.mission_limit_s = 2.0 * 3600.0;  // only a phase of a few degrees would close: no window in a day
    p.search_s = 86400.0;
    const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.station(),
                                               f.clock.from_utc({2026, 10, 16, 0, 0, 0.0}), p, gm);
    CHECK_FALSE(w.feasible);
    CHECK(w.reason.find("Kein Startfenster") != std::string::npos);
    CHECK(w.rejected >= 1);
}

TEST_CASE("Launch window: never before the earliest liftoff, also when the bias moves the crossing",
          "[iss][launch_window]") {
    test::spice_session();
    const Fixture f;
    const auto p = f.parameters();
    const time::Epoch e0 = f.clock.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.station(), e0, p, gm);
    REQUIRE(w.feasible);
    // just after the window: the first-guess bias puts this crossing after `earliest`, the boost bias before it
    for (const double after_s : {5.0, 30.0, 60.0}) {
        const time::Epoch earliest = w.liftoff + after_s;
        const auto later = mission::plan_launch_window(f.pad_itrf(), f.earth, f.station(), earliest, p, gm);
        INFO(after_s << " s: " << later.reason);
        if (later.feasible) CHECK(later.liftoff - earliest >= 0.0);
    }
}

namespace {
const double re = constants::wgs84_semi_major_axis_m;

// Refuelling f1: a tanker from Starbase into a 200 km / 26.5 deg depot, inserted at 120 x 190 km.
struct TankerFixture {
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;
    const config::LaunchSiteConfig site = config::load_launch_site(rendezvous_test::config_dir, "starbase_olp1");
    const time::Epoch start = clock.from_utc({2026, 10, 20, 0, 0, 0.0});
    config::DepotOrbitConfig depot_config() const {
        config::DepotOrbitConfig d;
        d.height_m = 200e3;
        d.inclination_rad = 26.5 * deg;
        return d;
    }
    const mission::StationStateAt depot = sim::depot_orbit(depot_config(), start, earth);
    Vec3 pad_itrf() const { return frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m}); }
    mission::LaunchWindowParameters parameters() const {
        mission::LaunchWindowParameters p;
        p.inclination_rad = 26.5 * deg;
        p.insertion_s = 445.5;
        p.insertion_arc_rad = 12.2 * deg;
        p.separation_periapsis_radius_m = re + 120e3;
        p.separation_apoapsis_radius_m = re + 190e3;
        p.search_s = 20.0 * 86400.0;
        p.phasing.mission_limit_s = 48.0 * 3600.0 - p.insertion_s;
        p.cost = {308e3, 3.4e3, 12e3 / 86400.0};
        return p;
    }
};
}  // namespace

TEST_CASE("Launch window: candidates - the cheapest feasible orbit, below or above the station",
          "[iss][launch_window]") {
    test::spice_session();
    const TankerFixture f;
    auto p = f.parameters();
    p.phasing_orbits = {{re + 150e3, re + 190e3}, {re + 170e3, re + 190e3}, {re + 190e3, re + 190e3},
                        {re + 220e3, re + 240e3}, {re + 250e3, re + 270e3}};
    int below = 0, above = 0;
    time::Epoch earliest = f.start;
    for (int i = 0; i < 12; ++i) {  // twelve consecutive windows
        const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.depot, earliest, p, gm);
        INFO(i << ": " << w.reason);
        REQUIRE(w.feasible);
        REQUIRE(w.phasing_orbit);
        (w.phasing_orbit->periapsis_radius_m > re + 200e3 ? above : below)++;
        CHECK(w.schedule.above == (w.phasing_orbit->periapsis_radius_m > re + 200e3));
        CHECK(w.raised - w.separation > 0.0);
        CHECK(w.schedule.waypoint0_epoch - w.separation <= p.phasing.mission_limit_s);
        // the choice is the cheapest of the feasible candidates at that crossing
        for (const auto& c : p.phasing_orbits) {
            auto single = p;
            single.phasing_orbits = {c};
            single.search_s = 1800.0;  // only this crossing
            const auto other = mission::plan_launch_window(f.pad_itrf(), f.earth, f.depot, w.liftoff - 600.0, single, gm);
            if (other.feasible && std::abs(other.liftoff - w.liftoff) < 120.0) {
                CHECK(w.cost.total_kg() <= other.cost.total_kg() + 1.0);
            }
        }
        if (std::getenv("STARSIM_VERBOSE")) {
            std::printf("[launch_window f1] liftoff +%.2f d, orbit %.0f x %.0f km, Waypoint 0 +%.1f h, cost %.0f kg\n",
                        (w.liftoff - f.start) / 86400.0, (w.phasing_orbit->periapsis_radius_m - re) / 1e3,
                        (w.phasing_orbit->apoapsis_radius_m - re) / 1e3,
                        (w.schedule.waypoint0_epoch - w.separation) / 3600.0, w.cost.total_kg());
        }
        earliest = w.liftoff + 3600.0;
    }
    CHECK(below > 0);
    CHECK(above > 0);
}

TEST_CASE("Launch window: empty candidates keep the separation orbit (ISS unchanged)", "[iss][launch_window]") {
    test::spice_session();
    const Fixture f;
    const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.station(),
                                               f.clock.from_utc({2026, 10, 16, 0, 0, 0.0}), f.parameters(), gm);
    REQUIRE(w.feasible);
    CHECK_FALSE(w.phasing_orbit);
    // recorded before f1 (empty candidates must not move it)
    CHECK(w.liftoff.tdb_seconds_past_j2000() == Catch::Approx(845499835.167408943).epsilon(0.0).margin(1e-6));
    CHECK(w.schedule.waypoint0_epoch.tdb_seconds_past_j2000() == Catch::Approx(845519535.204464555).epsilon(0.0).margin(1e-6));
}

TEST_CASE("Launch window: a raise that eats the mission limit is infeasible", "[iss][launch_window]") {
    test::spice_session();
    const TankerFixture f;
    auto p = f.parameters();
    p.phasing_orbits = {{re + 250e3, re + 270e3}};
    p.phasing.mission_limit_s = 2.0 * 3600.0;  // two burns half an orbit apart + checkout leave no phasing
    p.search_s = 3.0 * 86400.0;
    const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.depot, f.start, p, gm);
    CHECK_FALSE(w.feasible);
    CHECK(w.reason.rfind("Kein Startfenster", 0) == 0);
}

TEST_CASE("Launch window: a candidate too close to the coelliptic height is refused", "[iss][launch_window]") {
    // review I1: flown, the osculating apoapsis of a 190 x 190 km orbit swings over the coelliptic height 197.1 km
    // (J2) and the phasing plan refuses it - the window must not choose it
    test::spice_session();
    const TankerFixture f;
    auto p = f.parameters();
    p.phasing_orbits = {{re + 190e3, re + 190e3}};
    p.search_s = 3.0 * 86400.0;
    const auto w = mission::plan_launch_window(f.pad_itrf(), f.earth, f.depot, f.start, p, gm);
    CHECK_FALSE(w.feasible);
    CHECK(w.reason.find("zu nah an der Zielbahn") != std::string::npos);
    p.phasing_orbits = {{re + 180e3, re + 190e3}, {re + 190e3, re + 190e3}};  // 180 x 190 km keeps its clearance
    p.search_s = 20.0 * 86400.0;
    const auto ok = mission::plan_launch_window(f.pad_itrf(), f.earth, f.depot, f.start, p, gm);
    REQUIRE(ok.feasible);
    CHECK(ok.phasing_orbit->periapsis_radius_m == Catch::Approx(re + 180e3));
}

TEST_CASE("Phasing cost against a hand computation", "[iss][launch_window]") {
    const double r1 = re + 170e3, r_t = re + 200e3;
    mission::PhasingParameters p;
    const double r_c = r_t - p.coelliptic_height_m;
    orbit::KeplerianElements e;
    e.semi_major_axis_m = r1;
    e.inclination_rad = 26.5 * deg;
    mission::RaiseEstimate raise;
    raise.feasible = true;
    raise.state = orbit::state_from_elements(e, gm);
    raise.delta_v_mps = 20.0;
    mission::PhasingSchedule schedule;
    schedule.feasible = true;
    const mission::PhasingCostModel model{308e3, 3.4e3, 12e3 / 86400.0};
    const double t_s = 30.0 * 3600.0;
    const auto cost = mission::phasing_cost(raise, schedule, t_s, p, r_t, model, gm);
    // Hohmann 170 km -> r_c: departure + arrival (vis-viva)
    const double dv_boost = std::sqrt(gm / r1) * (std::sqrt(2.0 * r_c / (r1 + r_c)) - 1.0) +
                            std::sqrt(gm / r_c) * (1.0 - std::sqrt(2.0 * r1 / (r1 + r_c)));
    CHECK(cost.propellant_kg == Catch::Approx(308e3 * (1.0 - std::exp(-(20.0 + dv_boost) / 3.4e3))).epsilon(1e-9));
    CHECK(cost.boil_off_kg == Catch::Approx(12e3 * 30.0 / 24.0).epsilon(1e-12));
    CHECK(cost.total_kg() == Catch::Approx(cost.propellant_kg + cost.boil_off_kg));
}
