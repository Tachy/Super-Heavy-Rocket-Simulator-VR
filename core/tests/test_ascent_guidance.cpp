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

// Flown orbit: minimum and maximum radius over one revolution in the full force model, as altitude
// above the equatorial radius. Osculating Kepler elements are not used: in the J2 field they oscillate
// by several km even for a perfectly circular orbit.
struct FlownOrbit {
    double low_km = 0.0;
    double high_km = 0.0;
};

FlownOrbit flown_orbit(const sim::FlightSession& session, size_t index) {
    FlownOrbit o{1e9, -1e9};
    for (const auto& p : session.predict_trajectory(index, 5600.0, 5.0)) {
        const double h = (norm(p) - equatorial_radius_m) / 1000.0;
        o.low_km = std::min(o.low_km, h);
        o.high_km = std::max(o.high_km, h);
    }
    return o;
}

struct OrbitResult {
    double periapsis_km = 0.0;
    double apoapsis_km = 0.0;
    double inclination_deg = 0.0;
    double propellant_t = 0.0;
    double flight_time_s = 0.0;
    double flown_low_km = 0.0, flown_high_km = 0.0;  // radius over the next revolution, as altitude
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
    // Insertion accuracy is judged on osculating elements at cutoff (how target orbits are specified);
    // the flown radius range additionally shows the ~10 km J2 oscillation of a Kepler-circular orbit.
    r.periapsis_km = (elements.semi_major_axis_m * (1.0 - elements.eccentricity) - equatorial_radius_m) / 1000.0;
    r.apoapsis_km = (elements.semi_major_axis_m * (1.0 + elements.eccentricity) - equatorial_radius_m) / 1000.0;
    const auto flown = flown_orbit(session, session.controlled_index());
    std::printf("[validation]   flown radius range over one revolution (J2 field): %.1f .. %.1f km\n", flown.low_km,
                flown.high_km);
    r.flown_low_km = flown.low_km;
    r.flown_high_km = flown.high_km;
    r.inclination_deg = elements.inclination_rad / deg;
    r.propellant_t = ship.vehicle().propellant_kg() / 1000.0;
    r.flight_time_s = session.mission_time_s();
    r.state = session.autopilot().state;
    std::printf("[validation] ascent%s: osculating orbit at cutoff %.1f x %.1f km (target %.0f x %.0f), i = %.3f deg (target %.3f), "
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
    // J2-aware insertion: the flown revolution stays at 200 km within the J2 short-period swing (~0.7 km at 26.5 deg
    // in the J2-only field; the Moon and Sun add a little); the osculating elements at cutoff differ by design.
    CHECK(r.flown_low_km > 198.0);
    CHECK(r.flown_high_km < 203.0);
    CHECK(r.flown_high_km - r.flown_low_km < 3.0);
    CHECK(std::abs(r.inclination_deg - 26.5) < 0.1);
    CHECK(r.propellant_t > 0.0);
}

TEST_CASE("Closed-loop guidance recovers from a booster engine-out", "[gnc][validation][ascent]") {
    gnc::AscentTarget target{200e3, 200e3, 26.5 * deg, true};
    const auto r = fly_to_orbit(target, true);
    REQUIRE(r.state == sim::AutopilotState::Orbit);
    CHECK(r.flown_low_km > 198.0);  // the flown revolution (J2-aware insertion), as the nominal ascent
    CHECK(r.flown_high_km < 203.0);
    CHECK(std::abs(r.inclination_deg - 26.5) < 0.1);
}

TEST_CASE("Circularization burn at apoapsis (burn execution, two-body field)", "[gnc][validation][burn]") {
    // Two-body gravity: the Kepler plan is then exact, so this checks the executor alone (ignition timing,
    // direction tracking, cutoff with the shutdown transient). J2 effects are covered separately.
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    environment::PointMassGravityConfig two_body;
    two_body.moon = two_body.sun = two_body.earth_j2 = false;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR, two_body);
    const double gm = session.gm_earth_m3ps2();
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});

    // Ship alone on a 200 x 400 km orbit, at periapsis.
    const orbit::KeplerianElements start{equatorial_radius_m + 300e3, 200e3 / (2.0 * equatorial_radius_m + 600e3),
                                         26.5 * deg, 0.0, 0.0, 0.0};
    session.place_in_orbit("starship_block3", orbit::state_from_elements(start, gm), t0, 1);
    REQUIRE(session.plan_circularization_at_apoapsis());
    const double planned_dv = norm(session.burn()->node().delta_v_gcrf_mps);
    const double propellant_before = session.body(0).vehicle().propellant_kg();

    for (double t = 0.0; t < 4000.0 && session.autopilot().state == sim::AutopilotState::Burn; t += 1.0) {
        session.advance_to(t0 + t, 1000);
    }
    session.advance_to(session.epoch() + 3.0);
    REQUIRE(session.autopilot().state == sim::AutopilotState::Off);

    const auto& ship = session.body(0);
    const auto flown = flown_orbit(session, 0);
    const double peri = flown.low_km;
    const double apo = flown.high_km;
    const double used_t = (propellant_before - ship.vehicle().propellant_kg()) / 1000.0;
    std::printf("[validation] circularization: planned dv %.1f m/s, flown orbit after burn %.1f x %.1f km, %.1f t used\n",
                planned_dv, peri, apo, used_t);
    CHECK(std::abs(peri - 400.0) < 5.0);
    CHECK(std::abs(apo - 400.0) < 5.0);
}

// Tuning aid (hidden, run with "[.tuning]"): flight-path angle profile of the booster for several kicks
// (reference: IFT-3 webcast 2024, an earlier vehicle; to be replaced by flights 12-14).
TEST_CASE("Pitch kick tuning against a flight-path angle profile", "[.tuning]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    std::printf("IFT-3 reference gamma: 60 s 72.2, 90 s 58.8, 120 s 43.7, 150 s 32.5 deg\n");
    for (double kick_deg : {3.4, 5.0, 6.0, 7.0, 8.0, 10.0}) {
        sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
        const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
        session.launch_from_pad("starship_block3", {25.99686 * deg, -97.15394 * deg, 10.0}, 90.0 * deg, t0);
        gnc::PitchProgram program;
        program.kick_angle_rad = kick_deg * deg;
        session.engage_ascent_autopilot({200e3, 200e3, 26.5 * deg, true}, program);
        std::printf("kick %4.1f deg:", kick_deg);
        double next = 60.0;
        for (double t = 0.0; t < 400.0 && next <= 150.0; t += 1.0) {
            session.advance_to(t0 + t, 1000);
            if (session.autopilot().state == sim::AutopilotState::Booster && session.mission_time_s() >= next) {
                const auto tm = session.telemetry(session.controlled_index());
                std::printf("  %3.0f s %5.1f deg (%4.1f km)", next, tm.flight_path_angle_rad / deg, tm.altitude_m / 1000.0);
                next += 30.0;
            }
        }
        std::printf("\n");
    }
}

TEST_CASE("Burns below the main-engine minimum impulse use RCS translation", "[gnc][validation][burn]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravityConfig two_body;
    two_body.moon = two_body.sun = two_body.earth_j2 = false;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR, two_body);
    const double gm = session.gm_earth_m3ps2();
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const orbit::KeplerianElements circular{equatorial_radius_m + 300e3, 0.0, 26.5 * deg, 0.0, 0.0, 0.0};
    session.place_in_orbit("starship_block3", orbit::state_from_elements(circular, gm), t0, 1);
    const double minimum = session.minimum_burn_dv_mps();
    CHECK(minimum > 0.6);  // one engine (a single-engine group since b3) at minimum throttle, ~1.75 s of transients

    const double dv = 0.5;  // m/s, far below the Raptor minimum
    const auto s = session.body(0).state();
    const auto at_node = orbit::propagate_two_body({s.position_gcrf_m, s.velocity_gcrf_mps}, gm, 600.0);
    REQUIRE(session.execute_burn({t0 + 600.0, math::normalized(at_node.velocity_mps) * dv}));
    CHECK(session.burn_configuration().rcs);
    for (double t = 0.0; t < 3000.0 && session.autopilot().state == sim::AutopilotState::Burn; t += 1.0) {
        session.advance_to(t0 + t, 100000);
    }
    REQUIRE(session.autopilot().state == sim::AutopilotState::Off);
    const auto& after = session.body(0).state();
    const auto el = orbit::elements_from_state({after.position_gcrf_m, after.velocity_gcrf_mps}, gm);
    const double expected_a = 1.0 / (2.0 / circular.semi_major_axis_m -
                                     std::pow(std::sqrt(gm / circular.semi_major_axis_m) + dv, 2) / gm);
    std::printf("[validation] 0.5 m/s RCS translation burn: semi-major axis change %.1f m, expected %.1f m\n",
                el.semi_major_axis_m - circular.semi_major_axis_m, expected_a - circular.semi_major_axis_m);
    CHECK(std::abs(el.semi_major_axis_m - expected_a) < 30.0);  // ~0.01 m/s
}

TEST_CASE("Small burn uses a reduced engine configuration and stays accurate", "[gnc][validation][burn]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravityConfig two_body;
    two_body.moon = two_body.sun = two_body.earth_j2 = false;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR, two_body);
    const double gm = session.gm_earth_m3ps2();
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const orbit::KeplerianElements circular{equatorial_radius_m + 300e3, 0.0, 26.5 * deg, 0.0, 0.0, 0.0};
    session.place_in_orbit("starship_block3", orbit::state_from_elements(circular, gm), t0, 1);

    const double dv = 20.0;
    const auto s = session.body(0).state();
    REQUIRE(session.execute_burn({t0 + 60.0, math::normalized(s.velocity_gcrf_mps) * dv}));
    const auto& config = session.burn_configuration();
    CHECK(config.groups.size() == 1);  // not all six engines for 20 m/s

    for (double t = 0.0; t < 300.0 && session.autopilot().state == sim::AutopilotState::Burn; t += 1.0) {
        session.advance_to(t0 + t, 1000);
    }
    session.advance_to(session.epoch() + 3.0);
    const auto& after = session.body(0).state();
    const double v_after = norm(after.velocity_gcrf_mps);
    // Prograde burn on a circular orbit: speed after = v_circ + dv (gravity does no work in-plane at r const).
    const auto el = orbit::elements_from_state({after.position_gcrf_m, after.velocity_gcrf_mps}, gm);
    const double expected_a = 1.0 / (2.0 / circular.semi_major_axis_m -
                                     std::pow(std::sqrt(gm / circular.semi_major_axis_m) + dv, 2) / gm);
    std::printf("[validation] 20 m/s burn with %s at %.0f %%: semi-major axis %.2f km, expected %.2f km\n",
                config.groups.front().c_str(), config.throttle * 100.0, el.semi_major_axis_m / 1000.0,
                expected_a / 1000.0);
    CHECK(std::abs(el.semi_major_axis_m - expected_a) < 1500.0);  // ~0.4 m/s
    (void)v_after;
}

TEST_CASE("An RCS burn uses the thrusters' force and mass flow along +x", "[rcs][burn]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravityConfig two_body;
    two_body.moon = two_body.sun = two_body.earth_j2 = false;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR, two_body);
    const double gm = session.gm_earth_m3ps2();
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const orbit::KeplerianElements circular{equatorial_radius_m + 300e3, 0.0, 26.5 * deg, 0.0, 0.0, 0.0};
    session.place_in_orbit("starship_block3", orbit::state_from_elements(circular, gm), t0, 1);
    const auto s = session.body(0).state();
    const auto at_node = orbit::propagate_two_body({s.position_gcrf_m, s.velocity_gcrf_mps}, gm, 600.0);
    REQUIRE(session.execute_burn({t0 + 600.0, math::normalized(at_node.velocity_mps) * 0.5}));
    const auto& c = session.burn_configuration();
    CHECK(c.rcs);
    const auto capability = session.body(0).vehicle().rcs_force_capability({1.0, 0.0, 0.0});
    CHECK(c.thrust_n == Catch::Approx(capability.force_n));
    // The ship's two aft forward thrusters at Isp 70 s: 2 x 1500 / (70 x 9.81) = 4.4 kg/s at full output.
    CHECK(c.mass_flow_kgps > 0.1);
    CHECK(c.mass_flow_kgps == Catch::Approx(capability.mass_flow_kgps));
}

TEST_CASE("A stage without forward thrusters refuses an RCS burn", "[rcs][burn]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    environment::PointMassGravityConfig two_body;
    two_body.moon = two_body.sun = two_body.earth_j2 = false;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR, two_body);
    const double gm = session.gm_earth_m3ps2();
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    auto booster = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    booster.stages.pop_back();  // Super Heavy alone: no thruster points along +x
    const orbit::KeplerianElements circular{equatorial_radius_m + 300e3, 0.0, 26.5 * deg, 0.0, 0.0, 0.0};
    const auto sv = orbit::state_from_elements(circular, gm);
    session.place_body(booster, {sv.position_m, sv.velocity_mps, {}, {}}, t0);
    CHECK(session.body(0).vehicle().rcs_force_capability({1.0, 0.0, 0.0}).force_n < 1.0);
    const auto at_node = orbit::propagate_two_body(sv, gm, 600.0);
    CHECK_FALSE(session.execute_burn({t0 + 600.0, math::normalized(at_node.velocity_mps) * 0.5}));
    CHECK(std::isfinite(session.body(0).state().position_gcrf_m.x));
}
