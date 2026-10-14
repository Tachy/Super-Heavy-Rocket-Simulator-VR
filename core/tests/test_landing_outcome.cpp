#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("Ground beyond the shoreline is water", "[sim][landing]") {
    const config::SeaConfig east{90.0 * constants::deg_to_rad, 500.0};
    CHECK(sim::over_water(east, 501.0, 0.0));
    CHECK_FALSE(sim::over_water(east, 499.0, 300.0));
    CHECK_FALSE(sim::over_water(east, -2000.0, 0.0));
}

TEST_CASE("Site frame positions are east, north, up from the pad", "[sim][landing]") {
    const sim::LaunchSite site{0.4, -1.7, 10.0, {}, {}};
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const Vec3 origin = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const Vec3 p = origin + Vec3{-so, co, 0.0} * 3.0 + Vec3{-sl * co, -sl * so, cl} * 4.0 + Vec3{cl * co, cl * so, sl} * 5.0;
    const Vec3 enu = sim::site_enu_m(site, p);
    CHECK(std::abs(enu.x - 3.0) < 1e-6);
    CHECK(std::abs(enu.y - 4.0) < 1e-6);
    CHECK(std::abs(enu.z - 5.0) < 1e-6);
}

TEST_CASE("A booster coming down beyond the shoreline splashes down, on land it impacts", "[sim][landing]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
    REQUIRE(site.sea.has_value());
    const auto cfg = test::booster_config("starship_block3", 5000.0);
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const Vec3 seaward{std::sin(site.sea->direction_rad), std::cos(site.sea->direction_rad), 0.0};
    for (const double distance : {site.sea->shoreline_distance_m + 100.0, site.sea->shoreline_distance_m - 100.0}) {
        CAPTURE(distance);
        sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
        session.set_launch_site(site);
        // Engines off, 3 m above the ground: touches down at ~8 m/s (an impact, not a landing).
        session.place_body(cfg, test::state_over_site(site, earth, t0, seaward * distance + Vec3{0.0, 0.0, cg + 3.0},
                                                      {0.0, 0.0, -1.0}),
                           t0);
        session.set_control_mode(sim::ControlMode::DirectGimbal);
        for (double t = 0.1; t < 5.0 && session.landing_outcome(0) == sim::LandingOutcome::Flying; t += 0.1) {
            session.advance_to(t0 + t, 1000);
        }
        const auto outcome = session.landing_outcome(0);
        CHECK(outcome == (distance > site.sea->shoreline_distance_m ? sim::LandingOutcome::Splashdown
                                                                    : sim::LandingOutcome::Impact));
    }
}

TEST_CASE("A capsule resting on the water is gewassert", "[sim][landing]") {
    CHECK(std::string(sim::to_string(sim::LandingOutcome::SplashedDown)) == "gewassert");
}
