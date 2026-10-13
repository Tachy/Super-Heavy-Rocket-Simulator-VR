// Falcon 9 Block 5 from SLC-40 on the ascent autopilot (milestone c1): the generic vehicle model and PEG guidance fly
// the second vehicle configuration into orbit.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
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

// Flies the Falcon 9 stack (payload_kg) from SLC-40 towards a 200 km orbit in the ISS plane (CRS / crew flights).
Ascent fly(const gnc::PitchProgram& program) {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 13, 10, 33, 0.0});
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "falcon9_block5");
    cfg.stages.back().payload_mass_kg = payload_kg;
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
        if (a.meco_s < 0.0 && session.autopilot().state == sim::AutopilotState::UpperStage) {
            a.meco_s = session.mission_time_s();
            a.meco_altitude_km = (r - equatorial_radius_m) / 1000.0;
            a.meco_speed_mps = norm(st.velocity_gcrf_mps);
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
                payload_kg / 1000.0, program.kick_angle_rad / deg, a.meco_s, a.meco_altitude_km, a.meco_speed_mps,
                a.meco_gamma_deg, a.peri_km, a.apo_km, a.inclination_deg, a.time_s, a.propellant_t);
    return a;
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

TEST_CASE("Falcon 9 pitch kick sweep", "[.tuning]") {
    for (double kick_deg : {10.0, 12.0, 14.0, 16.0, 18.0, 20.0}) {
        gnc::PitchProgram program;
        program.kick_angle_rad = kick_deg * deg;
        fly(program);
    }
}

TEST_CASE("Autopilot flies Falcon 9 from SLC-40 into a 200 km orbit", "[gnc][validation][ascent][falcon9]") {
    // [E] Falcon 9 pitch program from the sweep above: the Starship kick (10 deg) lofts the lower-thrust stack
    // (staging at 83 km, gamma 52 deg) and the second stage runs dry short of orbit; 18 deg stages at ~68 km, gamma
    // ~33 deg (Falcon 9 webcasts: MECO at ~65-70 km).
    gnc::PitchProgram program;
    program.kick_angle_rad = 18.0 * deg;
    const auto a = fly(program);
    REQUIRE_FALSE(a.impacted);  // review focus: the stack stands on the ground (no mount at SLC-40)
    REQUIRE(a.state == sim::AutopilotState::Orbit);
    CHECK(std::abs(a.peri_km - 200.0) < 5.0);
    CHECK(std::abs(a.apo_km - 200.0) < 5.0);
    CHECK(std::abs(a.inclination_deg - 51.6) < 0.1);
    CHECK(a.propellant_t > 0.5);
    CHECK(a.propellant_t < 30.0);  // review focus: an untested drag table would show as an implausibly easy ascent
}
