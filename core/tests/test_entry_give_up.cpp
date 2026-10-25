// Crew escape (docs/superpowers/archive/specs/2026-10-22-crew-escape-design.md), Task 4: the entry guidance gives a ship up -
// it aims beyond the coastline and flies the belly flop down to the impact (no gate, no flip).

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "ship_entry_scenario.hpp"
#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/landing_site.hpp"
#include "starsim/gnc/sea_target.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

// A site `east_m` east of the scenario start's ground point, the sea to the east beyond 500 m.
gnc::LandingSite site_east_of(const test::EntryStart& start, double east_m) {
    gnc::LandingSite site;
    site.latitude_rad = start.latitude_rad;
    site.longitude_rad = start.longitude_rad + east_m / (constants::wgs84_semi_major_axis_m * std::cos(start.latitude_rad));
    site.height_m = 0.0;
    site.sea = gnc::SeaSide{90.0 * deg, 500.0};
    return site;
}

// East / north of a ground position from the site's pad.
Vec3 enu_from_pad(const gnc::LandingSite& site, const Vec3& itrf_m) {
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const Vec3 east{-so, co, 0.0}, north{-sl * co, -sl * so, cl}, up{cl * co, cl * so, sl};
    const Vec3 d = itrf_m - frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    return {dot(d, east), dot(d, north), dot(d, up)};
}

struct GiveUpRun {
    bool gate = false, impacted = false;
    Vec3 impact_enu_m;
};

GiveUpRun fly_given_up(double height_m, double east_of_start_m) {
    test::EntryStart start;
    start.height_m = height_m;
    start.speed_relative_mps = 120.0;
    start.flight_path_angle_rad = -85.0 * deg;
    start.heading_rad = 90.0 * deg;  // moving east, towards the pad and the sea
    test::ShipEntryScenario scenario(start);
    const auto site = site_east_of(start, east_of_start_m);
    gnc::EntryGuidanceParameters p;
    p.predictor_j2 = false;  // the scenario's Earth has central gravity only
    const gnc::EntryTarget pad_target{frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, 0.0}), 0.0};
    gnc::EntryGuidance guidance(scenario.body(), gnc::load_alpha_schedule(STARSIM_CONFIG_DIR, "aero/ship_entry_alpha.csv"),
                                pad_target, p);
    guidance.skip_to(gnc::EntryPhase::Terminal);
    guidance.give_up(gnc::sea_give_up_target(site, 2000.0));
    CHECK(guidance.given_up());
    scenario.run_guided(guidance, gnc::EntryPhase::Gate, 900.0);
    GiveUpRun r;
    r.gate = guidance.phase() == gnc::EntryPhase::Gate;
    r.impacted = scenario.body().impacted() || scenario.body().resting();
    r.impact_enu_m = enu_from_pad(site, scenario.position_itrf_m());
    std::printf("give-up from %.0f m, pad %.0f m east: impact E %.0f N %.0f m (%s)\n", height_m, east_of_start_m,
                r.impact_enu_m.x, r.impact_enu_m.y, r.impacted ? "down" : "flying");
    return r;
}

}  // namespace

TEST_CASE("Sea give-up target lies beyond the shoreline", "[crew_escape][entry]") {
    test::EntryStart start;
    const auto site = site_east_of(start, 0.0);
    const auto t = gnc::sea_give_up_target(site, 2000.0);
    const Vec3 e = enu_from_pad(site, t.position_itrf_m);
    CHECK(std::abs(e.x - 2500.0) < 1.0);
    CHECK(std::abs(e.y) < 1.0);
    CHECK(t.nose_direction_itrf.x == 0.0);
    CHECK(t.ground_height_m == 0.0);
}

TEST_CASE("A given-up ship flies the belly flop into the sea", "[crew_escape][entry]") {
    const auto r = fly_given_up(12000.0, 2000.0);
    CHECK_FALSE(r.gate);
    CHECK(r.impacted);
    CHECK(r.impact_enu_m.x > 500.0);                       // beyond the shoreline
    CHECK(std::hypot(r.impact_enu_m.x, r.impact_enu_m.y) > 300.0);  // away from the tower
}

TEST_CASE("A ship given up at the gate height comes down where it falls", "[crew_escape][entry]") {
    const auto r = fly_given_up(800.0, 200.0);
    CHECK_FALSE(r.gate);
    CHECK(r.impacted);
    CHECK(std::isfinite(r.impact_enu_m.x));
    CHECK(std::isfinite(r.impact_enu_m.y));
}
