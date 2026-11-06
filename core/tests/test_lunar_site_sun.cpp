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
}

TEST_CASE("Mons Mouton Sun never exceeds 7 deg over 60 d", "[mission][lunar][lunar_sun]") {
    Fixture f;
    const double lat = -84.7906 * deg, lon = 29.1957 * deg;  // config/lunar_sites.toml mons_mouton_im2
    const time::Epoch from = f.time.from_utc({2026, 11, 1, 0, 0, 0.0});
    double max_deg = -90.0;
    for (double t = 0.0; t <= 60.0 * day_s; t += 3600.0) {
        max_deg = std::max(max_deg, mission::site_sun(f.ephemeris, f.moon, lat, lon, from + t).elevation_rad / deg);
    }
    INFO("max elevation " << max_deg);
    CHECK(max_deg < 7.0);
    CHECK(max_deg > 0.0);  // it does rise
}

TEST_CASE("Polar daylight intervals are clipped to the span", "[mission][lunar][lunar_sun]") {
    Fixture f;
    const time::Epoch from = f.time.from_utc({2026, 11, 1, 0, 0, 0.0});
    // Span shorter than a day at the equator: one clipped interval or none, never an interval outside the span.
    const auto intervals = mission::daylight_intervals(f.ephemeris, f.moon, 0.0, 0.0, from, 0.5 * day_s);
    for (const auto& i : intervals) {
        CHECK(i.sunrise >= from);
        CHECK(i.sunset <= from + 0.5 * day_s);
    }
    CHECK(intervals.size() <= 1);
    CHECK(mission::daylight_intervals(f.ephemeris, f.moon, 0.0, 0.0, from, 0.0).empty());
}
