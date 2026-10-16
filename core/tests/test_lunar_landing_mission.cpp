// Mission config lunar_landing: loading, placement in the parking orbit through the Moon's plane, start_mission
// dispatch, and refusal without a plane.

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;

TEST_CASE("Lunar landing missions load", "[config][lunar]") {
    constexpr double deg = constants::deg_to_rad;
    CHECK(config::mission_kind(STARSIM_CONFIG_DIR, "hls_lunar_llo") == config::MissionKind::LunarLanding);
    const auto llo = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    CHECK(llo.vehicle_id == "starship_hls");
    CHECK(llo.site_id == "mons_mouton_im2");
    CHECK(llo.lunar_orbit == "llo");
    CHECK_THAT(llo.parking_height_m, WithinAbs(200e3, 1e-6));
    CHECK_THAT(llo.parking_inclination_rad, WithinAbs(28.5 * deg, 1e-12));
    CHECK_THAT(llo.flight_time_s, WithinAbs(4.0 * 86400.0, 1e-6));
    CHECK_THAT(llo.earliest_departure_s, WithinAbs(1800.0, 1e-9));
    REQUIRE(llo.mcc_after_tli_s.size() == 2);
    CHECK_THAT(llo.mcc_after_tli_s[1], WithinAbs(2.5 * 86400.0, 1e-6));
    CHECK_THAT(llo.llo_altitude_m, WithinAbs(100e3, 1e-6));
    CHECK_THAT(llo.llo_inclination_rad, WithinAbs(90.0 * deg, 1e-12));
    const auto nrho = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_nrho");
    CHECK(nrho.lunar_orbit == "nrho");
    CHECK(nrho.nrho_revolutions == 9);
    CHECK(nrho.nrho_synodic_months == 2);
    CHECK(nrho.nrho_reference_revolutions == 6);
    CHECK_THAT(nrho.nrho_apolune_position.z, WithinAbs(-0.1821, 1e-12));
    CHECK_THAT(nrho.nrho_apolune_velocity.y, WithinAbs(-0.1033, 1e-12));
    CHECK_THROWS_AS(config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "crew_dragon_return"), config::ConfigError);
}

TEST_CASE("start_mission plans the HLS outbound flight", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    const auto start = sim::start_mission(session, STARSIM_CONFIG_DIR, "hls_lunar_llo", {}, time);
    INFO(start.refusal);
    CHECK(start.kind == config::MissionKind::LunarLanding);
    CHECK(start.return_planned);
    CHECK(start.refusal.empty());
    CHECK(session.lunar_step() == sim::LunarStep::Departure);
    CHECK(session.epoch().tdb_seconds_past_j2000() == time.from_utc({2026, 11, 2, 0, 0, 0.0}).tdb_seconds_past_j2000());
    // circular 200 km parking orbit
    const auto& s = session.body(session.controlled_index()).state();
    CHECK_THAT(norm(s.position_gcrf_m) - constants::wgs84_semi_major_axis_m, WithinAbs(200e3, 1.0));
}

TEST_CASE("A parking orbit below the Moon's declination is refused", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon};
    auto mission = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    mission.parking_inclination_rad = 2.0 * constants::deg_to_rad;  // the Moon is at ~20 deg declination then
    const auto epoch = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    CHECK_THROWS_WITH(sim::place_lunar_landing_mission(session, mission, STARSIM_CONFIG_DIR, epoch),
                      ContainsSubstring("Deklination"));
}

TEST_CASE("A lunar landing mission needs a session with the Moon as a reference body", "[sim][lunar][mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR};
    const auto mission = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    const auto epoch = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    CHECK_THROWS_WITH(sim::place_lunar_landing_mission(session, mission, STARSIM_CONFIG_DIR, epoch),
                      ContainsSubstring("Mondorientierung"));
}

TEST_CASE("Lunar landing missions load their descent, surface, ascent and budget sections", "[config][lunar]") {
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, "hls_lunar_llo");
    CHECK(m.pdi_altitude_above_site_m == 15e3);
    CHECK(m.braking_range_m == 450e3);
    CHECK(m.surface_stay_s > 0.0);
    CHECK(m.insertion_altitude_above_site_m == 15e3);
    CHECK(m.descent_delta_v_mps == 2100.0);
    CHECK(m.reserve_kg == 10000.0);
}

// The LLO variant closes with the 120 t HLS; the NRHO variant does not (user 2026-10-16, l4 Task 9: its flight ran dry
// on the LLO -> NRHO leg - 986 m/s after the ~4 d site wait, the cheap window closes ~2.5 d after the LLO arrival); the
// 204 t HLS closes neither.
TEST_CASE("Mission 1 closes in the LLO with the 120 t HLS; NRHO and 204 t are refused", "[sim][lunar][lunar_budget_mission]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    for (const char* id : {"hls_lunar_llo", "hls_lunar_nrho"}) {
        const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, id);
        const bool closes = std::string(id) == "hls_lunar_llo";
        {
            sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
            const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, time.from_utc(m.epoch_utc));
            CAPTURE(id, s.refusal);
            CHECK(s.return_planned == closes);
            if (!closes) CHECK(s.refusal.find("Mission nicht durchführbar") != std::string::npos);
        }
        {
            auto heavy = config::load_vehicle(STARSIM_CONFIG_DIR, m.vehicle_id);
            heavy.stages.front().dry_mass_kg = 204e3;   // the independent bottom-up estimate (spec)
            sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
            const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, time.from_utc(m.epoch_utc), &heavy);
            CAPTURE(id, s.refusal);
            CHECK_FALSE(s.return_planned);
            CHECK(s.refusal.find("Mission nicht durchführbar") != std::string::npos);
            CHECK(session.lunar_step() == sim::LunarStep::None);
        }
    }
}

namespace {
// Mission 1 flown from the parking orbit to its end (one time-warp call) and checks how it ended.
void fly_mission_1(const char* id, double span_days, sim::LunarStep end_step) {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    const auto s = sim::start_mission(session, STARSIM_CONFIG_DIR, id, {}, time);
    INFO(s.refusal);
    REQUIRE(s.return_planned);
    const double m0 = session.body(0).vehicle().mass_properties().mass_kg;
    session.advance_to(s.epoch + span_days * 86400.0, 200'000'000);
    const auto& td = session.lunar_touchdown();
    INFO(session.lunar_status());
    REQUIRE(td);
    std::printf("[lunar_mission] %s: %s, touchdown %.2f m / %.2f m/s, %.1f t left of %.0f t (propellant %.1f t), %s\n",
                id, sim::to_string(td->outcome), td->offset_m, td->sink_mps,
                session.body(0).vehicle().mass_properties().mass_kg / 1000.0, m0 / 1000.0,
                session.body(0).vehicle().propellant_kg() / 1000.0, session.lunar_status().c_str());
    CHECK(td->outcome == sim::LegTouchdown::Landed);
    CHECK(td->offset_m < 1.0);
    CHECK(session.lunar_mission_complete());
    CHECK(session.lunar_step() == end_step);
}
}  // namespace

TEST_CASE("Mission 1, LLO variant: LEO to the lunar surface and back to the LLO", "[sim][lunar][lunar_mission_flight][terrain][data][slow]") {
    fly_mission_1("hls_lunar_llo", 40.0, sim::LunarStep::InLlo);
}

// Probe for the mission epochs (plan l4 Task 9): STARSIM_LUNAR_MISSION=<id>, STARSIM_LUNAR_EPOCH_SHIFT_D=<days added
// to the TOML epoch>, STARSIM_LUNAR_SPAN_D=<days flown>; run with STARSIM_DEBUG_LUNAR_SESSION=1 and read the "descent
// plan" line. Finding: the descent wait hardly depends on the epoch (LLO 5.4-5.9 d for shifts -3..+6 d) - the LLO plane
// follows the arrival asymptote, fixed in the tidally locked Moon's frame. The NRHO variant is refused at the start.
TEST_CASE("Lunar mission epoch probe", "[.][lunar_epoch_probe]") {
    const char* id = std::getenv("STARSIM_LUNAR_MISSION");
    const char* shift = std::getenv("STARSIM_LUNAR_EPOCH_SHIFT_D");
    const char* span = std::getenv("STARSIM_LUNAR_SPAN_D");
    REQUIRE(id);
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    auto earth = spice::SpiceOrientation::earth();
    auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
    const auto m = config::load_lunar_landing_mission(STARSIM_CONFIG_DIR, id);
    const auto epoch = time.from_utc(m.epoch_utc) + (shift ? std::atof(shift) : 0.0) * 86400.0;
    sim::FlightSession session{ephemeris, earth, STARSIM_CONFIG_DIR, {}, &moon_me};
    const auto s = sim::start_lunar_landing_mission(session, STARSIM_CONFIG_DIR, m, epoch);
    INFO(s.refusal);
    REQUIRE(s.return_planned);
    session.advance_to(epoch + (span ? std::atof(span) : 10.0) * 86400.0, 200'000'000);
    std::printf("[epoch_probe] %s shift %s d: %s\n", id, shift ? shift : "0", session.lunar_status().c_str());
}
