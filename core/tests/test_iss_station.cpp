#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/station_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
const std::filesystem::path fixture_dir = std::filesystem::path(STARSIM_FIXTURE_DIR) / "iss_oem";

// Radial / along-track / cross-track components of `d` in the frame of state `s`.
math::Vec3 rtn(const math::StateVector& s, const math::Vec3& d) {
    const auto r = math::normalized(s.position_m);
    const auto n = math::normalized(cross(s.position_m, s.velocity_mps));
    const auto t = cross(n, r);
    return {dot(d, r), dot(d, t), dot(d, n)};
}

// A two-body OEM text (2026-10-14T12:00-20:00 UTC, day 287, 4 min rows) with one manoeuvre of `delta_v_mps` along
// the velocity at `tig_s` after the start; the event table lists the magnitude only, as NASA's does.
std::string manoeuvre_oem(double tig_s, double delta_v_mps, const std::string& event_name) {
    constexpr double span_s = 8.0 * 3600.0, step_s = 240.0;
    const double gm = constants::earth_gm_m3ps2, a = constants::wgs84_semi_major_axis_m + 420.0e3;
    const double inc = 51.6 * constants::deg_to_rad;
    math::StateVector s{{a, 0.0, 0.0}, {0.0, std::sqrt(gm / a) * std::cos(inc), std::sqrt(gm / a) * std::sin(inc)}};
    const auto hms = [](double t_s, int& h, int& m, double& sec) {
        h = 12 + static_cast<int>(t_s / 3600.0);
        m = static_cast<int>(std::fmod(t_s, 3600.0) / 60.0);
        sec = std::fmod(t_s, 60.0);
    };
    char line[256];
    int h, m;
    double sec;
    std::string text = "CCSDS_OEM_VERS = 2.0\nCREATION_DATE = 2026-10-14T00:00:00.000\nORIGINATOR = test\n"
                       "META_START\nOBJECT_NAME = ISS\nOBJECT_ID = 1998-067-A\nCENTER_NAME = Earth\n"
                       "REF_FRAME = EME2000\nTIME_SYSTEM = UTC\nSTART_TIME = 2026-10-14T12:00:00.000\n"
                       "USEABLE_START_TIME = 2026-10-14T12:00:00.000\nUSEABLE_STOP_TIME = 2026-10-14T20:00:00.000\n"
                       "STOP_TIME = 2026-10-14T20:00:00.000\nMETA_STOP\nCOMMENT TRAJECTORY EVENT SUMMARY:\n"
                       "COMMENT |  EVENT  |  TIG  | ORB |  DV  |  HA  |  HP  |\n";
    hms(tig_s, h, m, sec);
    std::snprintf(line, sizeof line, "COMMENT  %-20s 287:%02d:%02d:%06.3f       %.1f     421.0     419.0\n",
                  event_name.c_str(), h, m, sec, std::abs(delta_v_mps));
    text += line;
    text += "COMMENT                                     (3.3)   (227.3)   (226.2)\nCOMMENT End sequence of events\n";
    double t = 0.0;
    for (double row = 0.0; row <= span_s; row += step_s) {
        if (t < tig_s && row > tig_s) {
            s = orbit::propagate_two_body(s, gm, tig_s - t);
            s.velocity_mps += delta_v_mps * math::normalized(s.velocity_mps);
            t = tig_s;
        }
        s = orbit::propagate_two_body(s, gm, row - t);
        t = row;
        hms(row, h, m, sec);
        std::snprintf(line, sizeof line, "2026-10-14T%02d:%02d:%06.3f %.9f %.9f %.9f %.12f %.12f %.12f\n", h, m, sec,
                      s.position_m.x / 1e3, s.position_m.y / 1e3, s.position_m.z / 1e3, s.velocity_mps.x / 1e3,
                      s.velocity_mps.y / 1e3, s.velocity_mps.z / 1e3);
        text += line;
    }
    return text;
}

// An archive holding just `text` (its own directory per test case: ctest runs cases in parallel).
orbit::OemArchive archive_of(const std::string& text, const std::string& case_name,
                             const time::TimeScaleConverter& time) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_iss_" + case_name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    std::ofstream(dir / "ISS.OEM_J2K_EPH_test.txt") << text;
    return orbit::OemArchive::scan(dir, time);
}
}  // namespace

TEST_CASE("Impulse signs from a manoeuvre OEM: retrograde, posigrade, near the window end", "[iss][station]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto earth = spice::SpiceOrientation::earth();
    const auto iss = config::load_vehicle(config_dir, "iss");
    const auto t0 = time.from_utc({2026, 10, 14, 12, 1, 0.0});
    struct Case {
        double tig_s, delta_v_mps;
        std::string name;
        double expected_mps;
        bool assumed;
    };
    for (const auto& c : {Case{4.0 * 3600.0 + 120.0, -1.0, "Debris Avoidance", -1.0, false},
                          Case{4.0 * 3600.0 + 120.0, 0.5, "Reboost (test)", 0.5, false},  // '(' in the name
                          Case{7.5 * 3600.0, -1.0, "Debris Avoidance", 1.0, true}}) {   // < 1 rev before the end
        const auto archive = archive_of(manoeuvre_oem(c.tig_s, c.delta_v_mps, c.name),
                                        "sign_" + std::to_string(static_cast<int>(c.tig_s)) +
                                            (c.delta_v_mps > 0.0 ? "p" : "m"),
                                        time);
        REQUIRE(archive.size() == 1);
        const auto s = sim::iss_start(iss, archive, time, earth, t0, sim::IssOrbitSource::Oem);
        REQUIRE(s.impulses.size() == 1);
        CHECK(s.impulses.front().name == c.name);
        CHECK(s.impulses.front().delta_v_mps == Catch::Approx(c.expected_mps));
        // an undeterminable sign is assumed posigrade (reboosts are) and the note says so
        CHECK((s.note.find("posigrad") != std::string::npos) == c.assumed);
    }
}

TEST_CASE("add_station takes the station's epoch", "[iss][station]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, earth, t0,
                                      sim::IssOrbitSource::Oem);
    CHECK(start.epoch == t0);
    sim::FlightSession alone(ephemeris, earth, config_dir);
    alone.add_station(start.vehicle, start.state_gcrf, start.epoch, start.impulses);
    CHECK(alone.epoch() == t0);  // a session without bodies starts at the station's epoch
    sim::FlightSession with_vehicle(ephemeris, earth, config_dir);
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    with_vehicle.place_in_orbit(dragon, {start.state_gcrf.position_m * 0.99, start.state_gcrf.velocity_mps},
                                t0 + 60.0);
    CHECK_THROWS_AS(with_vehicle.add_station(start.vehicle, start.state_gcrf, start.epoch), std::logic_error);
}

TEST_CASE("ISS start: OEM, synthetic and auto", "[iss][station]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto earth = spice::SpiceOrientation::earth();
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    REQUIRE(archive.size() == 1);
    const auto iss = config::load_vehicle(config_dir, "iss");
    const auto inside = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto outside = time.from_utc({2027, 3, 1, 0, 0, 0.0});

    const auto a = sim::iss_start(iss, archive, time, earth, inside, sim::IssOrbitSource::Oem);
    CHECK(a.from_oem);
    CHECK(norm(a.state_gcrf.position_m - archive.for_epoch(inside)->state_gcrf(inside).position_m) < 1e-6);
    CHECK(a.vehicle.stages.front().dry_mass_kg == Catch::Approx(460320.0));

    CHECK_THROWS_AS(sim::iss_start(iss, archive, time, earth, outside, sim::IssOrbitSource::Oem), std::runtime_error);

    const auto b = sim::iss_start(iss, archive, time, earth, outside, sim::IssOrbitSource::Auto);
    CHECK_FALSE(b.from_oem);
    CHECK_FALSE(b.note.empty());
    const double h = norm(b.state_gcrf.position_m) - constants::wgs84_semi_major_axis_m;
    CHECK(h / 1e3 == Catch::Approx(413.0).margin(15.0));

    const auto c = sim::iss_start(iss, archive, time, earth, inside, sim::IssOrbitSource::Synthetic, 1.0);
    CHECK_FALSE(c.from_oem);
    const auto d = sim::iss_start(iss, orbit::OemArchive{}, time, earth, outside, sim::IssOrbitSource::Synthetic);
    CHECK_FALSE(d.from_oem);  // the config's reference orbit
    CHECK(sim::parse_iss_orbit_source("auto") == sim::IssOrbitSource::Auto);
    CHECK_THROWS_AS(sim::parse_iss_orbit_source("tle"), std::invalid_argument);
}

TEST_CASE("OEM events after the start epoch become signed station impulses", "[iss][station]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto earth = spice::SpiceOrientation::earth();
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto& oem = *archive.latest();
    const auto start = oem.row_epoch(0) + 60.0;
    const auto s = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, earth, start,
                                  sim::IssOrbitSource::Oem);
    size_t expected = 0;
    for (const auto& e : oem.events()) expected += e.delta_v_mps > 0.0 && e.tig > start ? 1 : 0;
    CHECK(s.impulses.size() == expected);
    for (size_t k = 1; k < s.impulses.size(); ++k) CHECK(s.impulses[k - 1].epoch <= s.impulses[k].epoch);
    for (const auto& i : s.impulses) {
        if (i.name.find("Reboost") != std::string::npos) CHECK(i.delta_v_mps > 0.0);
    }
}

TEST_CASE("A station alone in a session coasts in ideal attitude", "[iss][station]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, earth, t0,
                                      sim::IssOrbitSource::Oem);
    sim::FlightSession session(ephemeris, earth, config_dir);
    session.set_epoch(t0);  // a session that starts with a station only
    const size_t index = session.add_station(start.vehicle, start.state_gcrf, start.epoch, start.impulses);
    CHECK(session.station_index() == index);
    session.advance_to(t0 + 3600.0, 50);  // 50 calls at most: coasting, not 720,000 ticks
    CHECK(session.epoch() - t0 == Catch::Approx(3600.0).margin(1e-6));
    CHECK(session.body(index).ideal_attitude());
}

TEST_CASE("Placing a vehicle clears the station", "[iss][station]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, earth, t0,
                                      sim::IssOrbitSource::Oem);
    sim::FlightSession session(ephemeris, earth, config_dir);
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    session.place_in_orbit(dragon, {start.state_gcrf.position_m * 0.99, start.state_gcrf.velocity_mps}, t0);
    session.add_station(start.vehicle, start.state_gcrf, start.epoch, start.impulses);
    REQUIRE(session.station_index());
    CHECK(session.body_count() == 2);
    CHECK_THROWS_AS(session.add_station(start.vehicle, start.state_gcrf, start.epoch), std::logic_error);  // one station
    CHECK_THROWS_AS(session.set_epoch(t0), std::logic_error);                                // bodies exist
    session.place_in_orbit(dragon, {start.state_gcrf.position_m * 0.99, start.state_gcrf.velocity_mps}, t0);
    CHECK_FALSE(session.station_index());
    CHECK(session.body_count() == 1);
    CHECK_THROWS_AS(session.add_station(dragon, start.state_gcrf, start.epoch), std::logic_error);  // no [stage.station]
}

TEST_CASE("A station impulse changes the velocity at its epoch", "[iss][station]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, earth, t0,
                                      sim::IssOrbitSource::Oem);
    sim::FlightSession with(ephemeris, earth, config_dir), without(ephemeris, earth, config_dir);
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    for (auto* s : {&with, &without}) {
        s->place_in_orbit(dragon, {start.state_gcrf.position_m * 0.98, start.state_gcrf.velocity_mps}, t0);
    }
    const auto i = with.add_station(start.vehicle, start.state_gcrf, start.epoch, {{t0 + 1000.0, 1.0, "Reboost"}});
    const auto j = without.add_station(start.vehicle, start.state_gcrf, start.epoch);
    with.advance_to(t0 + 2000.0, 100000);
    without.advance_to(t0 + 2000.0, 100000);
    const auto& a = with.body(i).state();
    const auto& b = without.body(j).state();
    // +1 m/s along the velocity raises the orbit: after 1000 s the bodies are ~kilometres apart, mostly along track
    CHECK(norm(a.velocity_gcrf_mps - b.velocity_gcrf_mps) > 0.5);
    CHECK(norm(a.position_gcrf_m - b.position_gcrf_m) > 500.0);
}

TEST_CASE("ISS propagated with the session's force model against the OEM", "[iss][station][validation]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    const auto archive = orbit::OemArchive::scan(fixture_dir, time);
    const auto& oem = *archive.latest();
    // after the SPX-35 docking (2026-10-15T15:00), no event inside the arc
    const auto t0 = time.from_utc({2026, 10, 16, 0, 0, 0.0});
    const auto start = sim::iss_start(config::load_vehicle(config_dir, "iss"), archive, time, earth, t0,
                                      sim::IssOrbitSource::Oem);
    sim::FlightSession session(ephemeris, earth, config_dir);
    session.set_epoch(t0);
    session.set_solar_radiation_pressure(false);  // the OEM's SOLAR_RAD_AREA = 0
    const size_t index = session.add_station(start.vehicle, start.state_gcrf, start.epoch, start.impulses);
    for (const double hours : {6.0, 24.0}) {
        session.advance_to(t0 + hours * 3600.0, 100000);
        const auto& s = session.body(index).state();
        const auto truth = oem.state_gcrf(t0 + hours * 3600.0);
        const auto e = rtn(truth, s.position_gcrf_m - truth.position_m);
        if (std::getenv("STARSIM_VERBOSE")) {
            std::printf("ISS vs OEM after %4.1f h: R %8.1f T %9.1f N %7.1f m\n", hours, e.x, e.y, e.z);
        }
        // [E] 1.5 x measured (2026-10-19), rounded up to 100 m: 6 h R 21.8 T -894.5 N -13.3 m; 24 h R -11.7
        // T -3228.7 N -43.0 m - along track: US76 at ~420 km vs NASA's density model, J2-only gravity
        CHECK(norm(e) < (hours < 12.0 ? 1400.0 : 4900.0));
    }
}
