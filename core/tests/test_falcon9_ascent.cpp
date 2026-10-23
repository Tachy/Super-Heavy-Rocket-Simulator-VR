// Falcon 9 Block 5 from SLC-40 on the ascent autopilot (milestone c1): the generic vehicle model and PEG guidance fly
// the second vehicle configuration into orbit.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "ascent_validation.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double equatorial_radius_m = 6378137.0;
constexpr double payload_kg = 10000.0;  // [E] CRS class (Dragon cargo with its trunk load)

sim::LaunchSite slc40() {
    return sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_slc40"));
}

struct Ascent {
    sim::AutopilotState state = sim::AutopilotState::Off;
    double meco_s = -1.0, meco_altitude_km = 0.0, meco_speed_mps = 0.0, meco_gamma_deg = 0.0;
    double peri_km = 0.0, apo_km = 0.0, inclination_deg = 0.0, propellant_t = 0.0, time_s = 0.0;
    bool impacted = false;
};

// Flies the Falcon 9 stack (payload_kg) from SLC-40 towards a 200 km orbit in the ISS plane (CRS / crew flights) on
// `program`, the first stage to its configured reserve (no return).
Ascent fly(const gnc::PitchProgram& program) {
    const double payload = payload_kg;
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 13, 10, 33, 0.0});
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    cfg.stages.back().payload_mass_kg = payload;
    session.launch_from_pad(cfg, slc40(), 45.0 * deg, t0);
    session.engage_ascent_autopilot({200e3, 200e3, 51.6 * deg, true}, program);

    Ascent a;
    for (double t = 0.0; t < 900.0 && session.autopilot().state != sim::AutopilotState::Orbit; t += 1.0) {
        session.advance_to(t0 + t, 1000);
        const auto& b = session.body(session.controlled_index());
        if (b.impacted()) {
            a.impacted = true;
            return a;
        }
        const auto& st = b.state();
        const double r = norm(st.position_gcrf_m);
        if (std::getenv("STARSIM_DEBUG_ASCENT_G") && static_cast<int>(t) % 5 == 0 && t <= 140.0) {  // vs the webcast dials
            const auto tr = earth.fixed_to_gcrf(session.epoch());
            const auto tel = session.telemetry(session.controlled_index());
            std::printf("  [g] T+%3.0f  %6.2f km  %6.0f km/h  %4.2f g  m %6.1f t  prop %6.1f t  thrust %5.2f MN  q %5.1f kPa\n",
                        session.mission_time_s(), tel.altitude_m / 1000.0,
                        norm(st.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), st.position_gcrf_m)) * 3.6,
                        tel.sensed_acceleration_g, tel.mass_kg / 1000.0, tel.propellant_kg / 1000.0, tel.thrust_n / 1e6,
                        tel.dynamic_pressure_pa / 1000.0);
        }
        if (std::getenv("STARSIM_VERBOSE") && static_cast<int>(t) % 20 == 0) {
            std::printf("  t %4.0f  state %d  alt %7.2f km  vr %8.1f  |v| %7.1f  m %7.1f t  prop %6.1f t  thrust %5.2f MN"
                        "  tgo %6.1f  vh %7.1f  dh %7.1f km\n",
                        session.mission_time_s(), static_cast<int>(session.autopilot().state),
                        (r - equatorial_radius_m) / 1000.0, dot(st.velocity_gcrf_mps, st.position_gcrf_m / r),
                        norm(st.velocity_gcrf_mps), b.vehicle().mass_properties().mass_kg / 1000.0,
                        b.vehicle().propellant_kg() / 1000.0,
                        session.telemetry(session.controlled_index()).thrust_n / 1e6,
                        session.autopilot().guidance.time_to_go_s,
                        dot(st.velocity_gcrf_mps, session.ascent_guidance()->target_plane_normal_gcrf()),
                        dot(st.position_gcrf_m, session.ascent_guidance()->target_plane_normal_gcrf()) / 1000.0);
        }
        // MECO: the booster phase ends (the staging sequence follows)
        if (a.meco_s < 0.0 && session.autopilot().state != sim::AutopilotState::Booster &&
            session.autopilot().state != sim::AutopilotState::PreLaunch) {
            a.meco_s = session.mission_time_s();
            const auto tr = earth.fixed_to_gcrf(session.epoch());
            a.meco_speed_mps = norm(st.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), st.position_gcrf_m));
            a.meco_altitude_km = (frames::fixed_to_geodetic(transpose(tr.rotation) * st.position_gcrf_m).height_m -
                                  slc40().height_m) / 1000.0;
            a.meco_gamma_deg = std::asin(dot(st.velocity_gcrf_mps, st.position_gcrf_m / r) / a.meco_speed_mps) / deg;
        }
    }
    for (int i = 0; i < 4; ++i) session.advance_to(session.epoch() + 0.5);  // shutdown transient

    const auto& stage2 = session.body(session.controlled_index());
    const auto el = orbit::elements_from_state({stage2.state().position_gcrf_m, stage2.state().velocity_gcrf_mps},
                                               session.gm_earth_m3ps2());
    a.state = session.autopilot().state;
    a.peri_km = (el.semi_major_axis_m * (1.0 - el.eccentricity) - equatorial_radius_m) / 1000.0;
    a.apo_km = (el.semi_major_axis_m * (1.0 + el.eccentricity) - equatorial_radius_m) / 1000.0;
    // Inclination to the true equator of date (the guidance's target plane is built in the Earth-fixed frame): GCRF
    // elements carry the precession of the pole since J2000 (~0.15 deg by 2026).
    const auto pole_gcrf = earth.fixed_to_gcrf(session.epoch()).rotation * math::Vec3{0.0, 0.0, 1.0};
    const auto h = cross(stage2.state().position_gcrf_m, stage2.state().velocity_gcrf_mps);
    a.inclination_deg = std::acos(dot(math::normalized(h), pole_gcrf)) / deg;
    a.propellant_t = stage2.vehicle().propellant_kg() / 1000.0;
    a.time_s = session.mission_time_s();
    std::printf("[validation] Falcon 9 ascent (%.0f t payload, kick %.1f deg): staging T+%.0f s at %.1f km, %.0f m/s "
                "inertial, gamma %.1f deg; orbit %.1f x %.1f km, i = %.3f deg, T+%.0f s, %.1f t left in stage 2\n",
                payload / 1000.0, program.kick_angle_rad / deg, a.meco_s, a.meco_altitude_km, a.meco_speed_mps,
                a.meco_gamma_deg, a.peri_km, a.apo_km, a.inclination_deg, a.time_s, a.propellant_t);
    return a;
}

// CRS-35 (ascent trajectory and staging, Task 6): the validation mission config/missions/crs35_ascent.toml (its orbit
// and payload) flown through the onboard computer's plan - kick, recovery (LZ-40) and MECO from the return planned, not
// fitted - against tests/validation/telemetry/f9_crs35_2026-10-13.csv.
const char* const crs35_table = "f9_crs35_2026-10-13.csv";
// [D] The table shows the stack's state 3.42 s late against the release:
//   python tools/telemetry/liftoff_time.py tests/validation/telemetry/f9_crs35_2026-10-13.csv
//   -> quadratic: speed zero at T+3.42 s, acceleration 4.42 m/s^2 at T+9.2 s
constexpr double crs35_time_base_s = 3.42;
// [D] The altitude dial lags the speed dial by another 2.45 s (same command, mean error 0 m over T+12-32 s): T+5.87 s.
constexpr double crs35_altitude_time_base_s = 5.87;
// Recorded MECO (display time, the speed graphic's time base): the acceleration falls below half its value before the
// cutoff (~108 km/h/s) between T+137.75 and T+138.25 s - MECO at T+138.0 +- 0.5 s (the engine icon goes dark at
// T+137.0 s: it is not delayed like the dials).
constexpr double recorded_meco_display_s = 138.0;
constexpr double recorded_meco_uncertainty_s = 0.5;
// Recorded SECO (display time): no readings T+496-563 s; the rocket-equation fit of the second stage's speed
// (T+430-496 s, rms 0.9 m/s) reaches the final 27,012 km/h at T+515 s: SECO T+515 +- 3 s.
constexpr double recorded_seco_display_s = 515.0;
constexpr double recorded_seco_uncertainty_s = 3.0;
// Open discrepancies (docs/subsystems.md "Ascent trajectory and staging", Task 6), per recorded sample as fractions of
// the recorded value (test::allowance: the measured gap + test::allowance_margin). Before max-Q the recording
// accelerates ~0.5 m/s^2 harder (speed slope 4.42 against 3.95 m/s^2 at 8-40 m/s; G dial 1.5 against 1.40-1.48 g): with
// the gauge's masses its thrust comes out 3-7 % above nine Merlins at 845 kN while after max-Q it agrees within the G
// dial's 0.05 g (0.98-1.00) - no single thrust, Isp or mass error explains both, not calibrated. After max-Q CRS-35
// throttles earlier and deeper (G 1.4-1.6 at T+50-60 s, gauge flow ~72 %; ours holds the transonic q from Mach 0.9 at
// >= 0.80).
constexpr test::SampleGap crs35_sample_gaps[] = {
    {30.0, 0.095, 0.085},   // -9.5 % speed, -8.5 % altitude
    {60.0, 0.023, 0.082},   // +2.3 % fast, -8.2 % low
    {90.0, 0.046, 0.021},   // +4.6 %, -2.1 %
    {120.0, 0.042, 0.026},  // +4.2 %, +2.6 %
};
// MECO / SECO: T+135.5 against 134.6 s (+0.7 %), 6985 against 6490 km/h (+7.6 %), 59.7 against 57.4 km (+4.0 %); the
// planned kick (12.3 deg) flies a steeper arc (path 38.2 against 36.6 deg); SECO T+507.7 against ~511.6 s (-0.7 %).
constexpr double meco_time_allowance = 0.03;
// INTERIM (follow-ups Task 7, user 2026-10-23): the measured 13.6 s flip exposed that our boostback costs too little
// (53.6 t kept at MECO against ~66 t on the CRS-35 gauge; boostback 43 s against ~55 s): 7.6 % gap + 2 % = 0.096,
// to be tightened back to 0.03 once Task 7b finds the cause (docs/subsystems.md "Falcon 9 flip time").
constexpr double meco_speed_allowance = 0.096;
constexpr double meco_altitude_allowance = 0.04;
constexpr double meco_gamma_allowance_deg = 6.0;
constexpr double seco_time_allowance = 0.02;

const test::FlownAscent& crs35() {
    static const test::FlownAscent flown = [] {
        test::spice_session();
        const spice::SpiceTimeConverter time;
        return test::fly_validation_ascent("crs35_ascent", time.from_utc({2026, 10, 13, 10, 33, 0.0}));  // [S:flights.toml]
    }();
    return flown;
}

// Falcon 9 ascent profile: the throttle from the first stage's max-Q limit ([stage] max_dynamic_pressure_pa;
// STARSIM_DEBUG_ASCENT_G=1 prints the simulated G against the CRS-35 G-force dial). Pitch kick 12 deg [E] (a fixed
// program for the guidance check; missions plan theirs, the CRS-35 validation below).
gnc::PitchProgram falcon9_program() {
    gnc::PitchProgram program;
    program.kick_angle_rad = 12.0 * deg;
    return program;
}

}  // namespace

TEST_CASE("Falcon 9 payload override changes the liftoff mass", "[sim][falcon9]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 13, 10, 33, 0.0});
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    session.launch_from_pad(cfg, slc40(), 45.0 * deg, t0);
    const double m0 = session.body(0).vehicle().mass_properties().mass_kg;
    cfg.stages.back().payload_mass_kg += payload_kg;
    session.launch_from_pad(cfg, slc40(), 45.0 * deg, t0);
    CHECK(session.body(0).vehicle().mass_properties().mass_kg - m0 == Catch::Approx(payload_kg).margin(1.0));
}

TEST_CASE("Autopilot flies Falcon 9 from SLC-40 into a 200 km orbit", "[gnc][validation][ascent][falcon9]") {
    const auto a = fly(falcon9_program());
    REQUIRE_FALSE(a.impacted);  // review focus: the stack stands on the ground (no mount at SLC-40)
    REQUIRE(a.state == sim::AutopilotState::Orbit);
    CHECK(std::abs(a.peri_km - 200.0) < 5.0);
    CHECK(std::abs(a.apo_km - 200.0) < 5.0);
    CHECK(std::abs(a.inclination_deg - 51.6) < 0.1);
    CHECK(a.propellant_t > 0.5);
    CHECK(a.propellant_t < 30.0);  // review focus: an untested drag table would show as an implausibly easy ascent
}

TEST_CASE("Falcon 9 ascent to CRS-35's orbit against its webcast telemetry", "[gnc][validation][ascent][falcon9]") {
    const auto& a = crs35();
    REQUIRE(a.plan.feasible);
    REQUIRE(a.seco_s > 0.0);
    CHECK(a.plan.recovery.kind == sim::RecoveryKind::LandingZone);  // CRS-35 returned to LZ-40
    std::printf("[validation] CRS-35: kick %.2f deg, MECO T+%.2f s with %.1f t in the first stage, SECO T+%.1f s, "
                "%.1f x %.1f km, i %.2f deg, %.1f t left in the second stage\n",
                a.plan.kick_angle_rad / deg, a.meco.t_s, a.booster_meco_propellant_t, a.seco_s, a.periapsis_km,
                a.apoapsis_km, a.inclination_deg, a.upper_propellant_at_seco_t);

    // every recorded sample of the first-stage burn (nine engines lit throughout)
    for (const auto& gap : crs35_sample_gaps) {
        const auto speed = test::compare_speed(crs35_table, a, gap.t_s, crs35_time_base_s, test::allowance(gap.speed));
        const auto altitude = test::compare_altitude(crs35_table, a, gap.t_s, crs35_altitude_time_base_s,
                                                     test::allowance(gap.altitude));
        test::print("CRS-35", speed);
        test::print("CRS-35", altitude);
        CHECK(speed.within());
        CHECK(altitude.within());
    }

    // MECO: the recorded state at the recorded MECO against the simulated state at its MECO; the altitude dial shows the
    // state `lag` earlier, compared with the simulation that much before its MECO.
    // Reading tolerance at MECO: rate x 1 x uncertainty - the reading is taken at the centre of the MECO's interval
    // (T+138.0 +- 0.5 s; flight 14 reads at its lower end: 2 x).
    const double lag = crs35_altitude_time_base_s - crs35_time_base_s;
    const auto r = test::recorded_stack(crs35_table, recorded_meco_display_s);
    const auto r_lag = test::recorded_stack(crs35_table, recorded_meco_display_s - lag);
    const auto s_lag = a.at(a.meco.t_s - lag);
    const double recorded_gamma_deg = std::asin(r.altitude_rate_km_s * 3600.0 / r_lag.speed_kmh) / deg;
    const double gamma_reading_deg = test::altitude_resolution_km / 2.0 * 3600.0 / r_lag.speed_kmh / deg;
    const test::Comparison checks[] = {
        {"MECO time [s]", recorded_meco_display_s - crs35_time_base_s, a.meco.t_s,
         recorded_meco_uncertainty_s + test::time_base_uncertainty_s, meco_time_allowance},
        {"MECO speed [km/h]", r.speed_kmh, a.meco.speed_kmh,
         test::speed_resolution_kmh + std::abs(r.speed_rate_kmh_s) * recorded_meco_uncertainty_s, meco_speed_allowance},
        {"MECO altitude [km]", r.altitude_km, s_lag.altitude_km,
         test::altitude_resolution_km + std::abs(r.altitude_rate_km_s) * recorded_meco_uncertainty_s,
         meco_altitude_allowance},
        {"MECO path angle [deg]", recorded_gamma_deg, s_lag.gamma_deg, gamma_reading_deg + meco_gamma_allowance_deg, 0.0},
        {"SECO time [s]", recorded_seco_display_s - crs35_time_base_s, a.seco_s,
         recorded_seco_uncertainty_s + test::time_base_uncertainty_s, seco_time_allowance},
    };
    for (const auto& c : checks) {
        test::print("CRS-35", c);
        CHECK(c.within());
    }
}
