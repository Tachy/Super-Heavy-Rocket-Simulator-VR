// Falcon 9 landing on the drone ship (milestone c4): the deck as a landing zone, the drone-ship return (no boostback,
// three-engine entry burn, one-engine landing burn) and the Starlink-class mission from SLC-40.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "return_scenario.hpp"
#include "starsim/config/drone_ship_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

const spice::SpiceEphemeris& de440() {
    test::spice_session();
    static const spice::SpiceEphemeris e;
    return e;
}

const frames::BodyOrientation& earth() {
    test::spice_session();
    static const auto e = spice::SpiceOrientation::earth();
    return e;
}

time::Epoch start() {
    test::spice_session();
    return spice::SpiceTimeConverter{}.from_utc({2026, 10, 13, 10, 40, 0.0});
}

// The drone ship ~600 km north-east of the Cape, long axis north (scenarios).
sim::LaunchSite ship() {
    return sim::drone_ship_site(config::load_drone_ship(STARSIM_CONFIG_DIR, "asds_atlantic"), 32.0 * deg, -76.0 * deg,
                                -35.0, 0.0);
}

// Falcon 9 first stage (dry + 1 t), legs out, its feet `feet_height_m` above the deck at `east_m` from the deck centre,
// sinking at `sink_mps`.
void drop_on_ship(sim::FlightSession& session, double east_m, double feet_height_m, double sink_mps) {
    const auto cfg = test::booster_config("falcon9_block5", 1000.0);
    const auto t0 = start();
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    const vehicle::Vehicle probe(cfg, STARSIM_CONFIG_DIR);
    const double cg = probe.mass_properties().cg_station_m;
    const double foot = cfg.stages.front().landing_legs.foot_station_m;
    session.place_body(cfg, test::state_over_site(session.launch_site(), earth(), t0, {east_m, 0.0, feet_height_m + cg - foot},
                                                  {0.0, 0.0, -sink_mps}),
                       t0);
    session.set_leg_deployment(0, 1.0);
}

sim::LegTouchdown run_drop(sim::FlightSession& session) {
    const auto t0 = session.epoch();
    while (session.leg_touchdown(0) == sim::LegTouchdown::None && session.epoch() - t0 < 20.0) {
        session.advance_to(session.epoch() + 0.1);
    }
    return session.leg_touchdown(0);
}

}  // namespace

TEST_CASE("The drone ship's deck loads and becomes a landing zone", "[config][falcon9][asds]") {
    const auto c = config::load_drone_ship(STARSIM_CONFIG_DIR, "asds_atlantic");
    CHECK(c.name == "Drohnenschiff");
    CHECK(c.deck_length_m == 91.0);
    CHECK(c.deck_width_m == 52.0);
    CHECK(c.freeboard_m == 3.0);
    const auto site = ship();
    REQUIRE(site.deck);
    CHECK(site.height_m == -32.0);  // sea -35 m + 3 m freeboard
    CHECK(site.deck->half_length_m == 45.5);
    CHECK(site.deck->half_width_m == 26.0);
    REQUIRE(site.sea);
    CHECK(std::abs(site.sea->direction_rad - 90.0 * deg) < 1e-12);  // beside the long edge (east)
    CHECK(site.sea->shoreline_distance_m == 26.0);
    CHECK_THROWS(config::load_drone_ship(STARSIM_CONFIG_DIR, "nowhere"));
}

TEST_CASE("A Falcon 9 stage returning to a drone ship aims at its deck", "[gnc][falcon9][asds]") {
    sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
    session.set_launch_site(ship());
    session.set_landing_zone(ship());
    const vehicle::Vehicle stage(test::booster_config("falcon9_block5", 11300.0), STARSIM_CONFIG_DIR);
    const auto target = session.return_site(stage);
    CHECK(target.height_m == -32.0);
    REQUIRE(target.pad);
    CHECK(target.pad->radius_m == 26.0);
    REQUIRE(target.sea);
    CHECK(target.sea->shoreline_distance_m == 26.0);
}

TEST_CASE("A stage set down on the deck lands, one beside it splashes down", "[legs][falcon9][asds]") {
    {
        sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
        drop_on_ship(session, 0.0, 0.5, 2.0);
        CHECK(run_drop(session) == sim::LegTouchdown::Landed);
        CHECK(session.landing_outcome(0) == sim::LandingOutcome::Landed);
    }
    {
        // Feet (9 m from the axis) 10 m beyond the western long edge (26 m from the centre line): water on every side of
        // the ship, not only beside the edge the abort aims at.
        sim::FlightSession session(de440(), earth(), STARSIM_CONFIG_DIR);
        drop_on_ship(session, -45.0, 0.5, 2.0);
        CHECK(run_drop(session) == sim::LegTouchdown::Splashdown);
        CHECK(session.landing_outcome(0) == sim::LandingOutcome::Splashdown);
    }
}
