// Mission config lunar_landing: loading, placement in the parking orbit through the Moon's plane, start_mission
// dispatch, and refusal without a plane.

#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/mission_config.hpp"
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
