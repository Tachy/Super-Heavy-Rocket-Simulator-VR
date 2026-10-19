#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/orbit/oem.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
const std::string fixture = std::string(STARSIM_FIXTURE_DIR) + "/iss_oem/ISS.OEM_J2K_EPH_2026-10-14T18-30-01.txt";

std::string read_all(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream s;
    s << in.rdbuf();
    return s.str();
}
}  // namespace

TEST_CASE("OEM header, window and rows", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    CHECK(oem.header().object_name == "ISS");
    CHECK(oem.header().creation_date == "2026-10-14T18:30:01.420");
    REQUIRE(oem.header().mass_kg);
    CHECK(*oem.header().mass_kg == Catch::Approx(460320.0));
    CHECK(*oem.header().drag_area_m2 == Catch::Approx(1177.76));
    CHECK(*oem.header().drag_coefficient == Catch::Approx(1.5));
    CHECK(*oem.header().solar_radiation_area_m2 == 0.0);
    CHECK(oem.row_count() == 5403);
    const auto start = time.from_utc({2026, 10, 14, 12, 0, 0.0});
    CHECK(oem.header().usable_start == start);
    CHECK(oem.row_epoch(0) == start);
    // first row in metres: -2107.105516188220 km
    CHECK(oem.row_state(0).position_m.x == Catch::Approx(-2107105.516188220).margin(1e-6));
    CHECK(oem.row_state(0).velocity_mps.z == Catch::Approx(-6003.96973354998).margin(1e-9));
    CHECK(oem.covers(start + 86400.0));
    CHECK_FALSE(oem.covers(start - 1.0));
    CHECK_THROWS_AS(oem.state_gcrf(start - 1.0), orbit::OemError);
}

TEST_CASE("OEM event table", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    REQUIRE(oem.events().size() == 1);
    const auto& e = oem.events().front();
    CHECK(e.name == "SPX-35 Docking");
    CHECK(e.tig == time.from_utc({2026, 10, 15, 15, 0, 0.0}));  // day 288 of 2026
    CHECK(e.delta_v_mps == 0.0);
    CHECK(e.apoapsis_height_m == Catch::Approx(421.1e3));
    CHECK(e.periapsis_height_m == Catch::Approx(418.6e3));
}

TEST_CASE("OEM parses LF and CRLF alike", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    std::string text = read_all(fixture);
    std::string lf;
    for (char c : text) if (c != '\r') lf.push_back(c);
    std::string crlf;
    for (char c : lf) { if (c == '\n') crlf += "\r\n"; else crlf.push_back(c); }
    std::istringstream a(lf), b(crlf);
    const auto oem_lf = orbit::Oem::parse(a, time);
    const auto oem_crlf = orbit::Oem::parse(b, time);
    CHECK(oem_lf.row_count() == oem_crlf.row_count());
    CHECK(oem_lf.events().size() == oem_crlf.events().size());
    CHECK(oem_lf.header().creation_date == oem_crlf.header().creation_date);
}

TEST_CASE("OEM rejects unsupported frames and unordered rows", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const std::string meta = "CCSDS_OEM_VERS = 2.0\nMETA_START\nOBJECT_NAME = X\nCENTER_NAME = Earth\n"
                             "TIME_SYSTEM = UTC\nUSEABLE_START_TIME = 2026-10-14T12:00:00.000\n"
                             "USEABLE_STOP_TIME = 2026-10-14T12:08:00.000\n";
    std::istringstream bad_frame(meta + "REF_FRAME = TOD\nMETA_STOP\n");
    CHECK_THROWS_AS(orbit::Oem::parse(bad_frame, time), orbit::OemError);
    std::istringstream unordered(meta + "REF_FRAME = EME2000\nMETA_STOP\n"
                                        "2026-10-14T12:04:00.000 7000 0 0 0 7.5 0\n"
                                        "2026-10-14T12:00:00.000 7000 0 0 0 7.5 0\n");
    CHECK_THROWS_AS(orbit::Oem::parse(unordered, time), orbit::OemError);
    CHECK_THROWS_AS(orbit::Oem::load("no/such/file.oem", time), orbit::OemError);
}

TEST_CASE("OEM interpolation reproduces left-out rows", "[iss][oem][validation]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    // every other row removed: 8 min steps; the removed rows are the truth. A 9-point Lagrange window spans
    // 64 min (0.7 orbit) there: ~20 m typical, ~35 m worst away from the ends, up to ~530 m where the window is
    // one-sided (file start, right after a grid seam). The error scales with h^9, so the published 4 min grid is
    // good to ~4 cm (~1 m one-sided).
    const auto thin = oem.filtered([](size_t i) { return i % 2 == 0; });
    double worst_m = 0.0, worst_mps = 0.0;
    size_t worst_row = 0;
    std::vector<double> errors_m;
    for (size_t i = 1; i + 1 < oem.row_count(); i += 2) {
        const auto s = thin.state_gcrf(oem.row_epoch(i));
        const double e = norm(s.position_m - oem.row_state(i).position_m);
        errors_m.push_back(e);
        if (e > worst_m) worst_m = e, worst_row = i;
        worst_mps = std::max(worst_mps, norm(s.velocity_mps - oem.row_state(i).velocity_mps));
    }
    std::nth_element(errors_m.begin(), errors_m.begin() + errors_m.size() / 2, errors_m.end());
    const double median_m = errors_m[errors_m.size() / 2];
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("OEM leave-one-out (8 min): median %.3f m, worst %.3f m (row %zu), %.6f m/s\n", median_m,
                    worst_m, worst_row, worst_mps);
    }
    CHECK(median_m < 30.0);
    CHECK(worst_m < 800.0);
    CHECK(worst_mps < 2.0);
    // on a row the interpolation returns the row
    CHECK(norm(oem.state_gcrf(oem.row_epoch(100)).position_m - oem.row_state(100).position_m) < 1e-6);
}

TEST_CASE("OEM interpolation does not straddle a step change", "[iss][oem]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const auto oem = orbit::Oem::load(fixture, time);
    // the grid restarts at 2026-10-17T17:57:30 (row index 1170, a 90 s step): a window must lie on one side
    const auto seam = time.from_utc({2026, 10, 17, 17, 57, 30.0});
    size_t k = 0;
    while (oem.row_epoch(k) < seam) ++k;
    REQUIRE(k == 1170);
    REQUIRE(oem.row_epoch(k) == seam);
    // just before the seam: equal to an interpolation over the rows up to the seam row only
    const auto before_only = oem.filtered([k](size_t i) { return i <= k; });
    const auto t = oem.row_epoch(k - 1) + 30.0;
    CHECK(norm(oem.state_gcrf(t).position_m - before_only.state_gcrf(t).position_m) < 1e-6);
    // just after the seam: equal to an interpolation over the rows from the seam row on
    const auto after_only = oem.filtered([k](size_t i) { return i >= k; });
    const auto u = oem.row_epoch(k) + 30.0;
    CHECK(norm(oem.state_gcrf(u).position_m - after_only.state_gcrf(u).position_m) < 1e-6);
}
