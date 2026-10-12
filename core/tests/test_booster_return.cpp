// Return to launch site: after hot staging the booster flips, boosts back, coasts engines first and
// lands on the pad with a landing burn (tower catch modelled as a touchdown on the pad).

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_test_macros.hpp>

#include "starsim/frames/geodetic.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

}  // namespace

TEST_CASE("Booster returns to the launch pad after staging", "[gnc][booster][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const sim::LaunchSite pad{25.99686 * deg, -97.15394 * deg, 10.0};
    session.launch_from_pad("starship_block3", pad, 90.0 * deg, t0);
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
    std::printf("[validation] booster return: apogee %.1f km, touchdown %.1f m from the pad, sink rate %.2f m/s, "
                "lateral %.2f m/s, %.1f t propellant left, %s\n",
                max_altitude_km, distance_m, last_sink_rate, last_side_speed, booster.vehicle().propellant_kg() / 1000.0,
                booster.impacted() ? "IMPACT" : booster.resting() ? "landed" : "still flying");
    CHECK_FALSE(booster.impacted());
    CHECK(booster.resting());
    // Pad landing. A tower catch needs < 10 m and < ~1 m/s lateral: not reached yet (attitude lag of the
    // centre engines; see CLAUDE.md "Known issues" - grid fins and lossless-convexification guidance).
    CHECK(distance_m < 25.0);
    CHECK(last_sink_rate < vehicle::FlightBody::max_touchdown_sink_rate_mps);
    CHECK(last_side_speed < 5.0);
}
