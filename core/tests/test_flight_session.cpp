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
    CHECK(session.body_count() == 1);  // hot staging: the ship's engines start while joined (staging sequence)
    session.advance_to(t0 + 28.0);     // separated once they are at thrust (start-up 3 s), the booster shut down
    REQUIRE(session.body_count() == 2);
    CHECK(session.controlled_index() == 1);
    CHECK(session.telemetry(0).engines_running == 0);  // booster shut down
    CHECK(session.telemetry(1).engines_running == 3);  // ship: sea-level Raptors only (RVacs blocked at low altitude)
}

TEST_CASE("Coast segments agree with fixed ticks", "[sim]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const math::StateVector leo{{6778137.0, 0.0, 0.0}, {0.0, 7668.6 * 0.9, 7668.6 * 0.436}};

    // Reference: fixed 200 Hz RK4 ticks (pilot input keeps the session out of coast mode).
    sim::FlightSession ticked(ephemeris, earth, STARSIM_CONFIG_DIR);
    ticked.place_in_orbit("starship_block3", leo, t0, 1);
    ticked.set_input({0.0, 0.0, 1e-9, 1.0});  // tiny roll input: forces ticks, negligible torque
    ticked.advance_to(t0 + 3600.0, 1000000);

    sim::FlightSession coasting(ephemeris, earth, STARSIM_CONFIG_DIR);
    coasting.place_in_orbit("starship_block3", leo, t0, 1);
    coasting.advance_to(t0 + 3600.0, 1000000);

    const double dr = norm(ticked.body(0).state().position_gcrf_m - coasting.body(0).state().position_gcrf_m);
    std::printf("[validation] 1 h LEO coast vs 200 Hz ticks: |dr| = %.3f m\n", dr);
    CHECK(dr < 1.0);
}

// External view (user 2026-10-22): a vehicle flying with the autopilot off is handed to the state Orbit; the clamped
// stack on the pad is left to the ascent autopilot, an engaged autopilot is not touched.
TEST_CASE("Orbit autopilot engaged for a free-flying vehicle only", "[sim]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const double deg = constants::deg_to_rad;

    sim::FlightSession pad(ephemeris, earth, STARSIM_CONFIG_DIR);
    CHECK_FALSE(pad.engage_orbit_autopilot());  // no body
    pad.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, t0);
    CHECK_FALSE(pad.engage_orbit_autopilot());  // clamped: the ascent autopilot's job
    CHECK(pad.autopilot().state == sim::AutopilotState::Off);

    sim::FlightSession orbiting(ephemeris, earth, STARSIM_CONFIG_DIR);
    const math::StateVector leo{{6778137.0, 0.0, 0.0}, {0.0, 7668.6 * 0.9, 7668.6 * 0.436}};
    orbiting.place_in_orbit("starship_block3", leo, t0, 1);
    REQUIRE(orbiting.autopilot().state == sim::AutopilotState::Off);
    CHECK(orbiting.engage_orbit_autopilot());
    CHECK(orbiting.autopilot().state == sim::AutopilotState::Orbit);
    CHECK_FALSE(orbiting.engage_orbit_autopilot());  // already engaged
    orbiting.advance_to(t0 + 60.0);
    CHECK(orbiting.autopilot().state == sim::AutopilotState::Orbit);  // it coasts on in the state Orbit
}
