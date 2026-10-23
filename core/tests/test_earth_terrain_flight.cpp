// M8 Plan A, Task 7: the reference recoveries flown on the real Earth terrain (data/terrain/earth + the fine EGM2008
// geoid, sim::load_earth_terrain) instead of the flat ground at the site height. The guidance keeps aiming at the baked
// site height; the plateau under every pad is that height, so the catch and the landing must end exactly as on the
// flat ground (same limits as the flat-ground cases in test_booster_return.cpp and test_falcon9_rtls.cpp).

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/earth_terrain_loader.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "return_scenario.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

struct Env {
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
};

const Env& env() {
    test::spice_session();
    static const Env e;
    return e;
}

std::shared_ptr<const environment::EarthTerrain> real_terrain() {
    const std::filesystem::path data_dir = STARSIM_DATA_DIR;
    REQUIRE(sim::earth_terrain_available(data_dir));
    static const auto terrain = sim::load_earth_terrain(STARSIM_CONFIG_DIR, data_dir);
    return terrain;
}

frames::GeodeticPosition geodetic(const sim::FlightSession& session, const Vec3& position_gcrf_m) {
    const auto t = env().earth.fixed_to_gcrf(session.epoch());
    return frames::fixed_to_geodetic(transpose(t.rotation) * position_gcrf_m);
}

}  // namespace

TEST_CASE("starbase catch on the real terrain", "[data][slow][earth_terrain]") {
    test::spice_session();
    const auto terrain = real_terrain();
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    session.set_earth_terrain(terrain);
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::start_ascent_mission(session, config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo"), STARSIM_CONFIG_DIR,
                              t0);
    const auto& site = session.launch_site();
    // The plateau under the pad and the tower is the baked site height.
    CHECK(std::abs(terrain->height_ellipsoid_m(site.latitude_rad, site.longitude_rad) - site.height_m) <= 0.01);

    bool landing = false;
    for (double t = 0.0; t < 1200.0;) {
        t += landing ? sim::FlightSession::tick_s : 0.5;
        session.advance_to(t0 + t, 1000);
        CHECK_FALSE(session.body(session.controlled_index()).impacted());  // clears the tower on the way up
        if (session.body_count() < 2) continue;
        const auto& booster = session.body(0);
        const auto* status = session.return_status(0);
        landing = status && status->phase >= gnc::ReturnPhase::LandingBurn;
        if (booster.caught() || booster.impacted() || booster.resting()) break;
    }
    // Same limits as the flat-ground case ("Booster return to the catch tower ends on the arms").
    const auto* tower = session.tower();
    REQUIRE(tower != nullptr);
    CHECK(tower->phase() == sim::CatchPhase::Caught);
    CHECK_FALSE(tower->tower_struck());
    const auto& booster = session.body(0);
    CHECK_FALSE(booster.impacted());
    const auto t_end = env().earth.fixed_to_gcrf(session.epoch());
    const Vec3 p = tower->geometry().to_site(transpose(t_end.rotation) * booster.state().position_gcrf_m);
    // The ground under the hanging booster is the plateau (the tower stands on it): its height above the ground equals
    // its height above the tower's base.
    const auto g = geodetic(session, booster.state().position_gcrf_m);
    const double ground_m = terrain->height_ellipsoid_m(g.latitude_rad, g.longitude_rad);
    std::printf("[earth_terrain] Starbase catch on the terrain: tower %s, booster %.2f m from the catch axis, %.2f m above "
                "the tower base, ground %.3f m (site %.3f m), max pin load %.2f MN\n",
                sim::to_string(tower->phase()), std::hypot(p.x, p.y), p.z, ground_m, site.height_m,
                tower->max_pin_load_n() / 1e6);
    CHECK(std::abs(ground_m - site.height_m) <= 0.01);
    CHECK(std::abs(session.telemetry(0).altitude_terrain_m - (g.height_m - site.height_m)) <= 0.01);
}

TEST_CASE("Falcon 9 RTLS to LZ-40 on the real terrain", "[data][slow][earth_terrain]") {
    test::spice_session();
    const auto terrain = real_terrain();
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_rtls");
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 10, 40, 0.0});
    sim::FlightSession session(env().ephemeris, env().earth, STARSIM_CONFIG_DIR);
    session.set_earth_terrain(terrain);
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
    const auto zone = *session.landing_zone();
    CHECK(std::abs(terrain->height_ellipsoid_m(zone.latitude_rad, zone.longitude_rad) - zone.height_m) <= 0.01);

    for (double t = 0.0; t < 900.0; t += 0.05) {
        session.advance_to(t0 + t, 100);
        if (session.body_count() < 2) continue;
        if (!session.return_status(0)) break;
        if (session.leg_touchdown(0) != sim::LegTouchdown::None || session.body(0).impacted()) break;
    }
    // Let the stage settle on its legs.
    session.advance_to(session.epoch() + 5.0, 100);
    const auto& stage = session.body(0);
    const Vec3 p = sim::site_enu_m(zone, transpose(env().earth.fixed_to_gcrf(session.epoch()).rotation) *
                                             stage.state().position_gcrf_m);
    // The feet's height: the axis point at the foot station (the stage stands upright on the flat plateau; the feet
    // sit ~5 m off the axis, which changes the geodetic height by micrometres).
    const double cg = stage.vehicle().mass_properties().cg_station_m;
    const double foot_station =
        config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id).stages.front().landing_legs.foot_station_m;
    const Vec3 foot_gcrf = stage.state().position_gcrf_m - stage.state().attitude.rotate(Vec3{cg - foot_station, 0.0, 0.0});
    const double foot_m = geodetic(session, foot_gcrf).height_m;
    std::printf("[earth_terrain] Falcon 9 RTLS on the terrain: %s, %.2f m from the centre, feet at %.4f m (plateau %.4f m), "
                "%.1f t left\n",
                sim::to_string(session.landing_outcome(0)), std::hypot(p.x, p.y), foot_m, zone.height_m,
                stage.vehicle().propellant_kg() / 1e3);
    // Same limits as the flat-ground case ("Falcon 9 RTLS mission: liftoff to orbit and the first stage landed on LZ-40").
    CHECK(session.landing_outcome(0) == sim::LandingOutcome::Landed);
    CHECK(std::hypot(p.x, p.y) < 5.0);
    CHECK(stage.resting());
    CHECK_FALSE(stage.impacted());
    CHECK(std::abs(foot_m - zone.height_m) <= 0.01);
}
