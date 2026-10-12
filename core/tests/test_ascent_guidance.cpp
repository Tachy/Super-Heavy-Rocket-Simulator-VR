#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double equatorial_radius_m = 6378137.0;

struct OrbitResult {
    double periapsis_km = 0.0;
    double apoapsis_km = 0.0;
    double inclination_deg = 0.0;
    double propellant_t = 0.0;
    double flight_time_s = 0.0;
    sim::AutopilotState state = sim::AutopilotState::Off;
};

// Flies the autopilot from the pad until it reports the orbit (or a timeout); optional failures.
OrbitResult fly_to_orbit(const gnc::AscentTarget& target, bool booster_engine_out) {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    session.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, t0);
    session.engage_ascent_autopilot(target);

    bool failed = false;
    for (double t = 0.0; t < 900.0 && session.autopilot().state != sim::AutopilotState::Orbit; t += 1.0) {
        session.advance_to(t0 + t, 1000);
        if (booster_engine_out && !failed && session.mission_time_s() > 60.0 &&
            session.autopilot().state == sim::AutopilotState::Booster) {
            for (size_t e : {20u, 27u, 33u - 1u}) session.fail_engine(0, 0, e);  // three outer-ring Raptors
            failed = true;
        }
        REQUIRE_FALSE(session.body(session.controlled_index()).impacted());
        if (std::getenv("STARSIM_VERBOSE") && session.autopilot().state == sim::AutopilotState::UpperStage) {
            const auto& b = session.body(session.controlled_index());
            const auto* g = session.ascent_guidance();
            const auto& st = b.state();
            const double r = norm(st.position_gcrf_m);
            const auto n = g->target_plane_normal_gcrf();
            const auto th = math::normalized(cross(n, st.position_gcrf_m / r));
            std::printf("  t %5.0f  tgo %7.2f  alt %7.2f km  vr %8.2f  vt %8.2f / %8.2f  thrust %5.1f MN\n",
                        session.mission_time_s(), session.autopilot().guidance.time_to_go_s,
                        (r - 6378137.0) / 1000.0, dot(st.velocity_gcrf_mps, st.position_gcrf_m / r),
                        dot(st.velocity_gcrf_mps, th), g->target_speed_mps(),
                        session.telemetry(session.controlled_index()).thrust_n / 1e6);
        }
    }
    auto trace_state = [&](const char* label) {
        if (!std::getenv("STARSIM_VERBOSE")) return;
        const auto& b = session.body(session.controlled_index());
        const double r = norm(b.state().position_gcrf_m);
        const auto el = orbit::elements_from_state({b.state().position_gcrf_m, b.state().velocity_gcrf_mps},
                                                   session.gm_earth_m3ps2());
        std::printf("  %s: alt %.3f km  vr %.2f m/s  |v| %.2f m/s  orbit %.1f x %.1f km  engines %d\n", label,
                    (r - equatorial_radius_m) / 1000.0,
                    dot(b.state().velocity_gcrf_mps, b.state().position_gcrf_m / r), norm(b.state().velocity_gcrf_mps),
                    (el.semi_major_axis_m * (1 - el.eccentricity) - equatorial_radius_m) / 1000.0,
                    (el.semi_major_axis_m * (1 + el.eccentricity) - equatorial_radius_m) / 1000.0,
                    session.telemetry(session.controlled_index()).engines_running);
    };
    trace_state("at cutoff command");
    for (int i = 0; i < 4; ++i) {
        session.advance_to(session.epoch() + 0.5);  // engine shutdown transient
        trace_state("  +0.5 s");
    }

    const auto& ship = session.body(session.controlled_index());
    const auto elements = orbit::elements_from_state(
        {ship.state().position_gcrf_m, ship.state().velocity_gcrf_mps}, session.gm_earth_m3ps2());
    OrbitResult r;
    r.periapsis_km = (elements.semi_major_axis_m * (1.0 - elements.eccentricity) - equatorial_radius_m) / 1000.0;
    r.apoapsis_km = (elements.semi_major_axis_m * (1.0 + elements.eccentricity) - equatorial_radius_m) / 1000.0;
    r.inclination_deg = elements.inclination_rad / deg;
    r.propellant_t = ship.vehicle().propellant_kg() / 1000.0;
    r.flight_time_s = session.mission_time_s();
    r.state = session.autopilot().state;
    std::printf("[validation] ascent%s: orbit %.1f x %.1f km (target %.0f x %.0f), i = %.3f deg (target %.3f), "
                "T+%.0f s, %.0f t propellant left\n", booster_engine_out ? " with 3 booster engines out" : "",
                r.periapsis_km, r.apoapsis_km, target.periapsis_altitude_m / 1000.0, target.apoapsis_altitude_m / 1000.0,
                r.inclination_deg, target.inclination_rad / deg, r.flight_time_s, r.propellant_t);
    return r;
}

}  // namespace

TEST_CASE("Launch azimuth", "[gnc]") {
    // Inclination equal to the latitude: due east.
    CHECK(gnc::launch_azimuth_rad(28.5 * deg, 28.5 * deg, 7800.0, true) == Catch::Approx(90.0 * deg).margin(1e-6));
    // Polar orbit northwards: slightly west of north (Earth rotation correction).
    const double polar = gnc::launch_azimuth_rad(28.5 * deg, 90.0 * deg, 7800.0, true);
    CHECK(polar < 0.0);
    CHECK(polar > -5.0 * deg);
    // Not reachable directly (i < latitude): clamps to due east.
    CHECK(gnc::launch_azimuth_rad(28.5 * deg, 10.0 * deg, 7800.0, true) == Catch::Approx(90.0 * deg).margin(1e-6));
}

TEST_CASE("Autopilot flies Starship V3 from Starbase into a 200 km orbit", "[gnc][validation][ascent]") {
    gnc::AscentTarget target{200e3, 200e3, 26.5 * deg, true};
    const auto r = fly_to_orbit(target, false);
    REQUIRE(r.state == sim::AutopilotState::Orbit);
    CHECK(std::abs(r.periapsis_km - 200.0) < 5.0);
    CHECK(std::abs(r.apoapsis_km - 200.0) < 5.0);
    CHECK(std::abs(r.inclination_deg - 26.5) < 0.1);
    CHECK(r.propellant_t > 0.0);
}

TEST_CASE("Closed-loop guidance recovers from a booster engine-out", "[gnc][validation][ascent]") {
    gnc::AscentTarget target{200e3, 200e3, 26.5 * deg, true};
    const auto r = fly_to_orbit(target, true);
    REQUIRE(r.state == sim::AutopilotState::Orbit);
    CHECK(std::abs(r.periapsis_km - 200.0) < 5.0);
    CHECK(std::abs(r.apoapsis_km - 200.0) < 5.0);
    CHECK(std::abs(r.inclination_deg - 26.5) < 0.1);
}
