#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("Flight session: pad, ignition, liftoff, staging", "[sim]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);

    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const double deg = constants::deg_to_rad;
    session.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, t0);
    REQUIRE(session.body_count() == 1);

    session.ignite();
    session.advance_to(t0 + 4.0);
    CHECK(session.telemetry(0).engines_running == 33);
    CHECK(session.telemetry(0).altitude_m < 100.0);  // still clamped

    session.release_clamps();
    session.advance_to(t0 + 24.0);
    const auto climbing = session.telemetry(0);
    CHECK(climbing.altitude_m > 150.0);
    CHECK(climbing.vertical_speed_mps > 20.0);
    CHECK(norm(session.body(0).state().angular_velocity_body_radps) < 0.01);  // rate command holds attitude
    CHECK_FALSE(session.body(0).impacted());

    session.stage();
    REQUIRE(session.body_count() == 2);
    CHECK(session.controlled_index() == 1);
    session.advance_to(t0 + 28.0);
    CHECK(session.telemetry(0).engines_running == 0);  // booster shut down
    CHECK(session.telemetry(1).engines_running == 3);  // ship: sea-level Raptors only (RVacs blocked at low altitude)
}
