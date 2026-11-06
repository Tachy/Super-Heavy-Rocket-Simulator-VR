// Sun at a lunar site: elevation / azimuth on the 1737.4 km sphere (MOON_ME, geometric horizon, Sun centre, apparent)
// and the daylight intervals. Apollo touchdown elevations come from the ALSJ table "Sun and Earth: Altitudes and
// Azimuths at the Apollo Landing Sites" (Fjeld / Scotti ephemeris, Scotti_AltAz.txt): the same definition as ours
// (geometric elevation of the Sun's centre above the local horizontal at the landing coordinates); the mission reports
// give only planning values (Apollo 11: 10.8 deg planned, SP-4029) at a different time. Tolerance 1.0 deg.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>

#include "starsim/mission/lunar_site_sun.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double day_s = constants::seconds_per_day;

struct Fixture {
    bool spice = (test::spice_session(), true);
    spice::SpiceEphemeris ephemeris;
    spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    spice::SpiceTimeConverter time;
};

struct Touchdown {
    const char* name;
    double lat_deg, lon_deg;
    time::UtcDateTime utc;
    double reference_elevation_deg;  // ALSJ Scotti_AltAz.txt, landing row
};

}  // namespace

TEST_CASE("Sun elevation at the Apollo 11 / 15 / 17 touchdowns matches the mission reports within 1.0 deg",
          "[mission][lunar][lunar_sun]") {
    Fixture f;
    // Landing coordinates of config/lunar_sites.toml (LROC NAC).
    const Touchdown cases[] = {
        {"apollo_11", 0.67416, 23.47314, {1969, 7, 20, 20, 17, 40.0}, 10.65},
        {"apollo_15", 26.13222, 3.63386, {1971, 7, 30, 22, 16, 29.0}, 12.05},
        {"apollo_17", 20.19089, 30.77168, {1972, 12, 11, 19, 54, 57.0}, 13.33},
    };
    for (const auto& c : cases) {
        const auto sun = mission::site_sun(f.ephemeris, f.moon, c.lat_deg * deg, c.lon_deg * deg, f.time.from_utc(c.utc));
        INFO(c.name << " elevation " << sun.elevation_rad / deg << " azimuth " << sun.azimuth_rad / deg);
        CHECK(std::abs(sun.elevation_rad / deg - c.reference_elevation_deg) < 1.0);
        CHECK(sun.azimuth_rad > 0.0);  // landings in the lunar morning: Sun in the east
    }
}

TEST_CASE("Equatorial daylight lasts 14.77 +- 0.3 d", "[mission][lunar][lunar_sun]") {
    Fixture f;
    const time::Epoch from = f.time.from_utc({2026, 11, 1, 0, 0, 0.0});
    const auto intervals = mission::daylight_intervals(f.ephemeris, f.moon, 0.0, 0.0, from, 60.0 * day_s);
    const mission::DaylightInterval* full = nullptr;
    for (const auto& i : intervals) {
        if (i.sunrise > from && i.sunset < from + 60.0 * day_s) full = &i;
    }
    REQUIRE(full != nullptr);
    CHECK(((full->sunset - full->sunrise) / day_s) == Catch::Approx(14.77).margin(0.3));
    // Sun at the horizon at both ends (bisection to 1 s, ~1 deg per 2 h).
    const auto rise = mission::site_sun(f.ephemeris, f.moon, 0.0, 0.0, full->sunrise);
    const auto set = mission::site_sun(f.ephemeris, f.moon, 0.0, 0.0, full->sunset);
    CHECK(std::abs(rise.elevation_rad / deg) < 0.01);
    CHECK(std::abs(set.elevation_rad / deg) < 0.01);
    // Azimuth convention (from north, east positive, [-180, 180]): at the equator the Sun rises in the east (+90) and
    // sets in the west (-90). On the horizon cos(A) = sin(dec) / cos(lat), so A = 90 deg -+ dec; the Sun's declination
    // in the Moon frame is bounded by the 1.54 deg tilt of the lunar equator to the ecliptic (+ < 0.1 deg of libration
    // and aberration): tolerance 1.7 deg.
    CHECK(std::abs(rise.azimuth_rad / deg - 90.0) < 1.7);
    CHECK(std::abs(set.azimuth_rad / deg + 90.0) < 1.7);
}

TEST_CASE("Sun culminates on the equator side of the site", "[mission][lunar][lunar_sun]") {
    Fixture f;
    const time::Epoch from = f.time.from_utc({2026, 11, 1, 0, 0, 0.0});
    for (const double lat_deg : {60.0, -60.0}) {
        const auto intervals = mission::daylight_intervals(f.ephemeris, f.moon, lat_deg * deg, 0.0, from, 60.0 * day_s);
        const mission::DaylightInterval* full = nullptr;
        for (const auto& i : intervals) {
            if (i.sunrise > from && i.sunset < from + 60.0 * day_s) full = &i;
        }
        REQUIRE(full != nullptr);
        const auto noon = mission::site_sun(f.ephemeris, f.moon, lat_deg * deg, 0.0, full->sunrise + 0.5 * (full->sunset - full->sunrise));
        INFO("lat " << lat_deg << " azimuth " << noon.azimuth_rad / deg << " elevation " << noon.elevation_rad / deg);
        // Northern site: Sun in the south (|A| near 180); southern site: Sun in the north (|A| near 0). The daylight
        // midpoint is within ~0.3 d of the transit (3.7 deg of hour angle, < 8 deg of azimuth): margin to 30 deg.
        if (lat_deg > 0.0) {
            CHECK(std::abs(noon.azimuth_rad / deg) > 150.0);
        } else {
            CHECK(std::abs(noon.azimuth_rad / deg) < 30.0);
        }
        CHECK(noon.elevation_rad / deg == Catch::Approx(90.0 - 60.0).margin(2.0));
    }
}

TEST_CASE("Mons Mouton Sun never exceeds 7 deg over 90 d", "[mission][lunar][lunar_sun]") {
    Fixture f;
    const double lat = -84.7906 * deg, lon = 29.1957 * deg;  // config/lunar_sites.toml mons_mouton_im2
    const time::Epoch from = f.time.from_utc({2026, 11, 1, 0, 0, 0.0});
    // 90 d = 3.05 synodic months (3 lunations); hourly samples, 2161 evaluations (~0.1 s).
    double max_deg = -90.0;
    for (double t = 0.0; t <= 90.0 * day_s; t += 3600.0) {
        max_deg = std::max(max_deg, mission::site_sun(f.ephemeris, f.moon, lat, lon, from + t).elevation_rad / deg);
    }
    INFO("max elevation " << max_deg);
    CHECK(max_deg < 7.0);  // 90 - 84.79 + 1.54 (declination bound) = 6.75 deg
    CHECK(max_deg > 0.0);  // it does rise
}

TEST_CASE("Daylight intervals are clipped to the span", "[mission][lunar][lunar_sun]") {
    Fixture f;
    const time::Epoch from = f.time.from_utc({2026, 11, 1, 0, 0, 0.0});
    const auto intervals = mission::daylight_intervals(f.ephemeris, f.moon, 0.0, 0.0, from, 60.0 * day_s);
    const mission::DaylightInterval* full = nullptr;
    for (const auto& i : intervals) {
        if (i.sunrise > from && i.sunset < from + 60.0 * day_s) full = &i;
    }
    REQUIRE(full != nullptr);

    // Whole span inside the day: one interval, clipped at both ends.
    const time::Epoch day_from = full->sunrise + 2.0 * day_s;
    const auto inside = mission::daylight_intervals(f.ephemeris, f.moon, 0.0, 0.0, day_from, 3.0 * day_s);
    REQUIRE(inside.size() == 1);
    CHECK(inside[0].sunrise == day_from);
    CHECK(inside[0].sunset == day_from + 3.0 * day_s);

    // Span starts in the day and ends in the night: clipped at the start, the true sunset at the end.
    const time::Epoch late_from = full->sunset - 1.0 * day_s;
    const auto start_clipped = mission::daylight_intervals(f.ephemeris, f.moon, 0.0, 0.0, late_from, 3.0 * day_s);
    REQUIRE(start_clipped.size() == 1);
    CHECK(start_clipped[0].sunrise == late_from);
    CHECK(std::abs(start_clipped[0].sunset - full->sunset) < 3.0);  // two bisections to 1 s

    // Span inside the night (the night lasts ~14.8 d): none.
    CHECK(mission::daylight_intervals(f.ephemeris, f.moon, 0.0, 0.0, full->sunset + 2.0 * day_s, 5.0 * day_s).empty());
    CHECK(mission::daylight_intervals(f.ephemeris, f.moon, 0.0, 0.0, from, 0.0).empty());
}

TEST_CASE("A polar day shorter than the 2 h grid step is not skipped", "[mission][lunar][lunar_sun]") {
    Fixture f;
    // South polar site (lon 0) whose lowest daily Sun peak of 2026-11-01 + 200.87 d is 1e-5 deg above the horizon
    // (found by scanning the elevation maxima over 400 d and shifting the latitude by the lowest peak; the peaks near
    // the 346.6 d cycle of the Sun's declination in the Moon frame dip from +3.0 to -0.02 deg at lat -88.5). The
    // 45 min day falls between two grid samples (rise at 0.2175 of a 2 h step, set at 0.5955), so a sign-only scan
    // misses it; the extremum refinement must find it.
    const double lat = -88.476739128 * deg;
    const time::Epoch from = f.time.from_utc({2026, 11, 1, 0, 0, 0.0});
    const double window_from = 199.5 * day_s, window_span = 3.0 * day_s;
    const auto intervals = mission::daylight_intervals(f.ephemeris, f.moon, lat, 0.0, from + window_from, window_span);
    REQUIRE(intervals.size() == 1);
    const double duration_s = intervals[0].sunset - intervals[0].sunrise;
    INFO("short day " << duration_s << " s");
    CHECK(duration_s < 7200.0);
    CHECK(duration_s > 600.0);
    // Independent reference: 60 s brute-force sampling of the same window.
    double first_up = -1.0, last_up = -1.0;
    for (double t = window_from; t <= window_from + window_span; t += 60.0) {
        if (mission::site_sun(f.ephemeris, f.moon, lat, 0.0, from + t).elevation_rad > 0.0) {
            if (first_up < 0.0) first_up = t;
            last_up = t;
        }
    }
    REQUIRE(first_up > 0.0);
    CHECK(std::abs(intervals[0].sunrise - from - first_up) <= 61.0);
    CHECK(std::abs(intervals[0].sunset - from - last_up) <= 61.0);
    // The Sun at the middle of the short day is above the horizon, at both neighbouring grid samples below it.
    const time::Epoch mid = intervals[0].sunrise + 0.5 * duration_s;
    CHECK(mission::site_sun(f.ephemeris, f.moon, lat, 0.0, mid).elevation_rad > 0.0);
}
