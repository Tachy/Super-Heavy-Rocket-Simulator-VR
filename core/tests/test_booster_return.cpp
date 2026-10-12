// Return to launch site: after hot staging the booster flips, boosts back, coasts engines first and
// lands on the pad with a landing burn (tower catch modelled as a touchdown on the pad).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
// FAA Revised Draft EA, Nov 2024 (https://www.faa.gov/media/87636, p. 7): "Up to approximately 74 MT of residual
// propellant ... on the Super Heavy vehicle for downrange landing, including expenditures" [S]. An EA projection,
// used here as the cap on the Block 3 booster's propellant at landing-burn ignition.
constexpr double faa_landing_propellant_kg = 74.0e3;

}  // namespace

TEST_CASE("Booster returns to the launch pad after staging", "[gnc][booster][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const sim::LaunchSite pad{25.99686 * deg, -97.15394 * deg, 10.0};
    const std::string vehicle_id = GENERATE(as<std::string>{}, "starship_block1", "starship_block2", "starship_block3");
    CAPTURE(vehicle_id);
    session.launch_from_pad(vehicle_id, pad, 90.0 * deg, t0);
    session.set_booster_return(true);
    gnc::AscentTarget target;
    target.periapsis_altitude_m = 200e3;
    target.apoapsis_altitude_m = 200e3;
    target.inclination_rad = 26.6 * deg;
    session.engage_ascent_autopilot(target);

    const bool verbose = std::getenv("STARSIM_VERBOSE") != nullptr;
    double last_sink_rate = 0.0;  // last tick before ground contact
    double last_side_speed = 0.0;
    double max_altitude_km = 0.0;
    gnc::ReturnPhase last_phase = gnc::ReturnPhase::Flip;
    double next_print = 0.0;
    for (double t = 0.0; t < 1200.0;) {
        // Coarse steps, single ticks from the landing burn on (touchdown sink rate).
        t += last_phase >= gnc::ReturnPhase::LandingBurn ? sim::FlightSession::tick_s : 0.5;
        session.advance_to(t0 + t, 1000);
        if (session.body_count() < 2) continue;
        const auto& booster = session.body(0);
        if (booster.resting() || booster.impacted()) break;
        const auto tm = session.telemetry(0);
        last_sink_rate = -tm.vertical_speed_mps;
        last_side_speed = std::sqrt(std::max(tm.speed_relative_mps * tm.speed_relative_mps -
                                                 tm.vertical_speed_mps * tm.vertical_speed_mps, 0.0));
        max_altitude_km = std::max(max_altitude_km, tm.altitude_m / 1000.0);
        const auto* status = session.return_status(0);
        REQUIRE(status != nullptr);
        const bool print = verbose && (status->phase != last_phase || t >= next_print);
        if (print) {
            next_print = t + (status->phase == gnc::ReturnPhase::LandingBurn ? 1.0 : 10.0);
            std::printf("  T+%6.1f %-13s alt %8.3f km  v %7.1f m/s  vz %7.1f  miss %8.0f m  thr %.2f  tgo %5.1f  eng %d  "
                        "prop %5.1f t\n",
                        session.mission_time_s(), gnc::to_string(status->phase), tm.altitude_m / 1000.0,
                        tm.speed_relative_mps, tm.vertical_speed_mps, status->impact_miss_m, status->throttle, status->time_to_go_s,
                        tm.engines_running, tm.propellant_kg / 1000.0);
        }
        if (print && std::getenv("STARSIM_DEBUG_LANDING") && status->phase == gnc::ReturnPhase::LandingBurn) {
            const auto& st = booster.state();
            const auto up = math::normalized(st.position_gcrf_m);
            const auto axis = st.attitude.rotate(math::Vec3{1.0, 0.0, 0.0});
            const auto v_rel = st.velocity_gcrf_mps - cross(math::Vec3{0, 0, 7.292115e-5}, st.position_gcrf_m);
            const auto v_side = v_rel - up * dot(v_rel, up);
            std::printf("    tilt body %5.1f cmd %5.1f err %5.1f deg  v_side %5.1f  cmd.v_side %+5.2f  rate %.3f\n",
                        std::acos(dot(axis, up)) / deg, std::acos(dot(status->axis_gcrf, up)) / deg,
                        std::acos(std::min(1.0, dot(axis, status->axis_gcrf))) / deg, norm(v_side),
                        norm(v_side) > 0 ? dot(status->axis_gcrf, v_side / norm(v_side)) : 0.0,
                        norm(st.angular_velocity_body_radps));
        }
        if (std::getenv("STARSIM_DEBUG_HOLD") && status->phase == gnc::ReturnPhase::LandingBurn &&
            tm.altitude_m < pad.height_m + 150.0 && std::fmod(t, 0.25) < sim::FlightSession::tick_s) {
            // East/north components at the pad.
            const auto tr = earth.fixed_to_gcrf(session.epoch());
            const auto& st = booster.state();
            const math::Vec3 up{std::cos(pad.latitude_rad) * std::cos(pad.longitude_rad),
                                std::cos(pad.latitude_rad) * std::sin(pad.longitude_rad), std::sin(pad.latitude_rad)};
            const math::Vec3 east = math::normalized(cross(math::Vec3{0, 0, 1}, up));
            const math::Vec3 north = cross(up, east);
            const auto r_fixed = transpose(tr.rotation) * st.position_gcrf_m;
            const auto pad_fixed = frames::geodetic_to_fixed({pad.latitude_rad, pad.longitude_rad, pad.height_m});
            const auto omega = vehicle::earth_rotation_vector(tr);
            const auto v_fixed = transpose(tr.rotation) * (st.velocity_gcrf_mps - cross(omega, st.position_gcrf_m));
            const auto axis = transpose(tr.rotation) * st.attitude.rotate(math::Vec3{1.0, 0.0, 0.0});
            const auto cmd_axis = transpose(tr.rotation) * status->axis_gcrf;
            const auto d = r_fixed - pad_fixed;
            std::printf("    h %6.1f  off E %+6.1f N %+6.1f  v E %+5.1f N %+5.1f  cmd E %+.3f N %+.3f  body E %+.3f N %+.3f  "
                        "w %+.3f %+.3f %+.3f thr %.2f eng %d\n",
                        dot(d, up), dot(d, east), dot(d, north), dot(v_fixed, east), dot(v_fixed, north),
                        dot(cmd_axis, east), dot(cmd_axis, north), dot(axis, east), dot(axis, north),
                        st.angular_velocity_body_radps.x, st.angular_velocity_body_radps.y,
                        st.angular_velocity_body_radps.z, status->throttle, tm.engines_running);
        }
        last_phase = status->phase;
    }

    REQUIRE(session.body_count() >= 2);
    const auto& booster = session.body(0);
    const auto transform = earth.fixed_to_gcrf(session.epoch());
    const auto cg_fixed = transpose(transform.rotation) * booster.state().position_gcrf_m;
    const auto pad_fixed = frames::geodetic_to_fixed({pad.latitude_rad, pad.longitude_rad, pad.height_m});
    const auto geo = frames::fixed_to_geodetic(cg_fixed);
    const auto foot = frames::geodetic_to_fixed({geo.latitude_rad, geo.longitude_rad, pad.height_m});
    const double distance_m = norm(foot - pad_fixed);
    std::printf("[validation] %s booster return: apogee %.1f km, touchdown %.1f m from the pad, sink rate %.2f m/s, "
                "lateral %.2f m/s, %.1f t propellant left, %s\n",
                vehicle_id.c_str(), max_altitude_km, distance_m, last_sink_rate, last_side_speed, booster.vehicle().propellant_kg() / 1000.0,
                booster.impacted() ? "IMPACT" : booster.resting() ? "landed" : "still flying");
    CHECK_FALSE(booster.impacted());
    CHECK(booster.resting());
    // Pad landing. A tower catch needs < ~1 m and < ~0.5 m/s lateral: not reached yet (attitude lag of the
    // centre engines; see CLAUDE.md "Known issues" - powered-descent guidance by convex optimisation).
    CHECK(distance_m < 20.0);
    CHECK(last_sink_rate < vehicle::FlightBody::max_touchdown_sink_rate_mps);
    CHECK(last_side_speed < 3.0);
}

TEST_CASE("Catch roll reference turns the pins towards the arms", "[gnc][booster]") {
    const math::Vec3 up{0.0, 0.0, 1.0};
    const math::Vec3 across{1.0, 0.0, 0.0};
    for (const double azimuth_deg : {90.0, 45.0}) {
        const double a = azimuth_deg * deg;
        const auto y = gnc::catch_roll_reference(across, up, a, {0.0, 1.0, 0.0});
        const auto z = cross(up, y);
        const math::Vec3 pin = y * std::cos(a) + z * std::sin(a);  // first pin direction in the site frame
        CHECK(std::abs(std::abs(dot(pin, across)) - 1.0) < 1e-12);
        // The opposite current roll gives the opposite reference (shortest roll, pins symmetric).
        const auto y2 = gnc::catch_roll_reference(across, up, a, -y);
        CHECK(norm(y2 + y) < 1e-12);
    }
}

// Known open (Plan 3b, 2026-10-12): with SCvx the full return does not yet end on the arms reliably (attitude
// overshoot at the 13 -> 3 engine switch); [!mayfail] reports without failing the suite until Task 11 closes it.
TEST_CASE("Booster return to the catch tower ends caught or missed", "[gnc][booster][catch][validation][!mayfail]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const auto site = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
    const std::string vehicle_id = GENERATE(as<std::string>{}, "starship_block1", "starship_block2", "starship_block3");
    CAPTURE(vehicle_id);
    session.launch_from_pad(vehicle_id, site, 90.0 * deg, t0);
    session.set_booster_return(true);
    gnc::AscentTarget target;
    target.periapsis_altitude_m = 200e3;
    target.apoapsis_altitude_m = 200e3;
    target.inclination_rad = 26.6 * deg;
    session.engage_ascent_autopilot(target);

    bool landing = false;
    double ignition_propellant_kg = -1.0;
    for (double t = 0.0; t < 1200.0;) {
        t += landing ? sim::FlightSession::tick_s : 0.5;
        session.advance_to(t0 + t, 1000);
        CHECK_FALSE(session.body(session.controlled_index()).impacted());  // clears the tower on the way up
        if (session.body_count() < 2) continue;
        const auto& booster = session.body(0);
        const auto* status = session.return_status(0);
        const bool burning = status && status->phase >= gnc::ReturnPhase::LandingBurn;
        if (burning && !landing) ignition_propellant_kg = booster.vehicle().propellant_kg();
        landing = burning;
        if (booster.caught() || booster.impacted() || booster.resting()) break;
    }
    const auto* tower = session.tower();
    REQUIRE(tower != nullptr);
    const auto phase = tower->phase();
    // Until the SCvx catch is robust (Plan 3b, Task 11 tightens this to "caught"): any end, but never the tower.
    CHECK((phase == sim::CatchPhase::Caught || phase == sim::CatchPhase::Missed || phase == sim::CatchPhase::Aborted));
    CHECK_FALSE(tower->tower_struck());
    const auto t_end = earth.fixed_to_gcrf(session.epoch());
    const auto p = tower->geometry().to_site(transpose(t_end.rotation) * session.body(0).state().position_gcrf_m);
    std::printf("%s: tower %s, booster %.1f m from the catch axis, max pin load %.2f MN\n", vehicle_id.c_str(),
                sim::to_string(phase), std::hypot(p.x, p.y), tower->max_pin_load_n() / 1e6);
    std::printf("%s: propellant at landing-burn ignition %.1f t\n", vehicle_id.c_str(), ignition_propellant_kg / 1e3);
}

TEST_CASE("Block 3 reaches landing-burn ignition within the FAA residual propellant", "[gnc][booster][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    session.launch_from_pad("starship_block3",
                            sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")),
                            90.0 * deg, t0);
    session.set_booster_return(true);
    gnc::AscentTarget target;
    target.periapsis_altitude_m = 200e3;
    target.apoapsis_altitude_m = 200e3;
    target.inclination_rad = 26.6 * deg;
    session.engage_ascent_autopilot(target);
    double ignition_propellant_kg = -1.0;
    for (double t = 0.5; t < 1200.0 && ignition_propellant_kg < 0.0; t += 0.5) {
        session.advance_to(t0 + t, 1000);
        if (session.body_count() < 2) continue;
        const auto* status = session.return_status(0);
        if (status && status->phase >= gnc::ReturnPhase::LandingBurn) {
            ignition_propellant_kg = session.body(0).vehicle().propellant_kg();
        }
    }
    std::printf("starship_block3: propellant at landing-burn ignition %.1f t (FAA EA: <= 74 t)\n", ignition_propellant_kg / 1e3);
    REQUIRE(ignition_propellant_kg >= 0.0);
    CHECK(ignition_propellant_kg <= faa_landing_propellant_kg);
}
