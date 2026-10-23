// Return to launch site: after hot staging the booster flips, boosts back, coasts engines first and is caught by
// the tower after its landing burn (Super Heavy has no legs: always caught, project decision 2026-10-13).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "return_scenario.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
// FAA Revised Draft EA, Nov 2024 (https://www.faa.gov/media/87636, p. 7): "Up to approximately 74 MT of residual
// propellant ... on the Super Heavy vehicle for downrange landing, including expenditures" [S]. An EA projection,
// used here as the cap on the Block 3 booster's propellant at landing-burn ignition.
constexpr double faa_landing_propellant_kg = 74.0e3;

// The flight-like ascent: mission starship_leo (the operational throttle and the max-Q limit of the vehicle file, the
// kick planned before the launch - 14.85 deg -, MECO planned for the return to the Starbase tower). The default pitch program at full thrust cuts off at ~97 km climbing at 1.7 km/s,
// which leaves the boostback little to turn around (~1.5 km/s instead of ~2.2).
void start_flight_like_ascent(sim::FlightSession& session, time::Epoch t0) {
    sim::start_ascent_mission(session, config::load_ascent_mission(STARSIM_CONFIG_DIR, "starship_leo"), STARSIM_CONFIG_DIR,
                              t0);
}

}  // namespace

// The explicit laws (fallback when the first SCvx plan fails) are not accurate enough for the arms (~1-3 m, the arms
// leave 0.3 m per side): the booster ditches beyond the shoreline with the arms open, gently, clear of the tower.
TEST_CASE("The explicit fallback guidance ditches the booster at sea next to the tower", "[gnc][booster][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const auto pad = sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1"));
    const std::string vehicle_id = GENERATE(as<std::string>{}, "starship_block3");
    CAPTURE(vehicle_id);
    session.launch_from_pad(vehicle_id, pad, 90.0 * deg, t0);
    gnc::BoosterReturnParameters explicit_only;
    explicit_only.scvx_landing = false;
    session.set_booster_return(true, explicit_only);
    gnc::AscentTarget target;
    target.periapsis_altitude_m = 200e3;
    target.apoapsis_altitude_m = 200e3;
    target.inclination_rad = 26.6 * deg;
    session.engage_ascent_autopilot(target, gnc::PitchProgram{});  // the guidance as given, no pre-launch plan

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
        if (booster.caught() || booster.resting() || booster.impacted()) break;
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
    const auto* tower = session.tower();
    REQUIRE(tower != nullptr);
    const auto t_end = earth.fixed_to_gcrf(session.epoch());
    const auto p = tower->geometry().to_site(transpose(t_end.rotation) * booster.state().position_gcrf_m);
    std::printf("[validation] %s explicit return: apogee %.1f km, tower %s, %.1f m from the catch axis, sink %.2f m/s, "
                "lateral %.2f m/s, %.1f t propellant left\n",
                vehicle_id.c_str(), max_altitude_km, sim::to_string(tower->phase()), std::hypot(p.x, p.y), last_sink_rate,
                last_side_speed, booster.vehicle().propellant_kg() / 1000.0);
    CHECK_FALSE(tower->tower_struck());
    CHECK(tower->phase() == sim::CatchPhase::Aborted);
    CHECK(session.landing_outcome(0) == sim::LandingOutcome::Splashdown);
    CHECK(last_sink_rate < vehicle::FlightBody::max_touchdown_sink_rate_mps);
}

TEST_CASE("A booster recovered by tower catch gets no return guidance at a site without a tower", "[gnc][booster]") {
    // Super Heavy has no legs: without a tower there is nothing to land on (project decision 2026-10-13).
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const sim::LaunchSite pad{25.99686 * deg, -97.15394 * deg, 10.0};  // no tower
    session.launch_from_pad("starship_block3", pad, 90.0 * deg, t0);
    session.set_booster_return(true);
    gnc::AscentTarget target;
    target.periapsis_altitude_m = 200e3;
    target.apoapsis_altitude_m = 200e3;
    target.inclination_rad = 26.6 * deg;
    session.engage_ascent_autopilot(target, gnc::PitchProgram{});  // the guidance as given, no pre-launch plan
    for (double t = 0.0; t < 400.0 && session.body_count() < 2; t += 0.5) session.advance_to(t0 + t, 1000);
    REQUIRE(session.body_count() >= 2);
    CHECK(session.return_status(0) == nullptr);
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

TEST_CASE("Booster return to the catch tower ends on the arms", "[gnc][booster][catch][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    const std::string vehicle_id = "starship_block3";
    start_flight_like_ascent(session, t0);

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
    CHECK(phase == sim::CatchPhase::Caught);
    CHECK_FALSE(tower->tower_struck());
    const auto t_end = earth.fixed_to_gcrf(session.epoch());
    const auto p = tower->geometry().to_site(transpose(t_end.rotation) * session.body(0).state().position_gcrf_m);
    std::printf("%s: tower %s, booster %.1f m from the catch axis, max pin load %.2f MN\n", vehicle_id.c_str(),
                sim::to_string(phase), std::hypot(p.x, p.y), tower->max_pin_load_n() / 1e6);
    std::printf("%s: propellant at landing-burn ignition %.1f t\n", vehicle_id.c_str(), ignition_propellant_kg / 1e3);
}

// The ascent autopilot plans MECO from what the return needs (flip, boostback, the propellant kept for the landing
// burn), not from a fixed reserve: the booster lights its landing burn with the planned propellant. On the flight-like
// ascent the return keeps flight 14's timeline (our webcast table and video readings, tests/validation/telemetry):
// boostback ~T+152-187 s (31 -> 11 -> 3 engines), apogee 98.9 km at T+240 s, landing-burn ignition ~T+398 s. The
// booster's gauge is not followed after staging (user 2026-10-14: physics first): it shows 11.8 % (431 t) at the
// boostback's start, falls faster than the engines can burn (13 engines: ~15 t/s against ~10.5 t/s at full thrust) and
// 77 t in the last 12 s on 3 engines (~30 t burnt); our return needs ~300 t there.
TEST_CASE("Block 3 plans MECO to reach landing-burn ignition with the planned propellant, within the FAA residual",
          "[gnc][booster][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 12, 12, 0, 0.0});
    start_flight_like_ascent(session, t0);
    double ignition_propellant_kg = -1.0, ignition_s = -1.0, boostback_start_kg = -1.0;
    double boostback_start_s = -1.0, boostback_end_s = -1.0, apogee_km = 0.0, apogee_s = -1.0;
    for (double t = 0.25; t < 1200.0 && ignition_propellant_kg < 0.0; t += 0.25) {
        session.advance_to(t0 + t, 1000);
        if (session.body_count() < 2) continue;
        const auto* status = session.return_status(0);
        if (!status) continue;
        const auto tm = session.telemetry(0);
        const double t_mission = session.mission_time_s();
        if (status->phase == gnc::ReturnPhase::Boostback && boostback_start_s < 0.0) {
            boostback_start_s = t_mission;
            boostback_start_kg = tm.propellant_kg;
        }
        if (status->phase > gnc::ReturnPhase::Boostback && boostback_start_s > 0.0 && boostback_end_s < 0.0) {
            boostback_end_s = t_mission;
        }
        if (tm.altitude_m / 1e3 > apogee_km) {
            apogee_km = tm.altitude_m / 1e3;
            apogee_s = t_mission;
        }
        if (status->phase >= gnc::ReturnPhase::LandingBurn) {
            ignition_propellant_kg = tm.propellant_kg;
            ignition_s = t_mission;
        }
    }
    const double planned_kg =
        config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3").stages.front().landing_propellant_kg;
    std::printf("[validation] starship_block3 return: boostback T+%.0f-%.0f s from %.0f t (flight 14: ~T+152-187 s, gauge "
                "431 t), apogee %.1f km at T+%.0f s (98.9 km at T+240 s), landing-burn ignition T+%.0f s (~T+398 s) with "
                "%.1f t, planned %.1f t (FAA EA: <= 74 t)\n",
                boostback_start_s, boostback_end_s, boostback_start_kg / 1e3, apogee_km, apogee_s, ignition_s,
                ignition_propellant_kg / 1e3, planned_kg / 1e3);
    CHECK(std::abs(boostback_start_s - 152.0) < 5.0);
    // ascent Task 2 (operational throttle 0.88 + q limit instead of the speed-fitted schedule; the stack is faster than
    // flight 14, MECO earlier): apogee T+221 s, ignition T+383 s (bounds 15 -> 25 s)
    CHECK(std::abs(apogee_s - 240.0) < 25.0);
    CHECK(std::abs(apogee_km - 98.9) < 10.0);
    CHECK(std::abs(ignition_s - 398.0) < 25.0);
    REQUIRE(ignition_propellant_kg >= 0.0);
    CHECK(ignition_propellant_kg <= faa_landing_propellant_kg);
    REQUIRE(planned_kg > 0.0);
    CHECK(std::abs(ignition_propellant_kg - planned_kg) < 3.0e3);  // model: two-body flip coast, no drag/gravity loss
    // The MECO plan budgets the RCS gas from the main tanks (d1 review: without it 1.7 t short).
    CHECK(ignition_propellant_kg > planned_kg - 1.0e3);
}
