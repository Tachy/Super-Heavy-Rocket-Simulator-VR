#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/eclipse.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using environment::occulted_fraction;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

// State of an Earth-fixed site in GCRF (relative to the Earth's centre).
math::StateVector site_state(double lat_deg, double lon_deg, double height_m, time::Epoch epoch) {
    const auto transform = spice::SpiceOrientation::earth().fixed_to_gcrf(epoch);
    const Vec3 fixed = frames::geodetic_to_fixed({lat_deg * deg, lon_deg * deg, height_m});
    return {transform.position(fixed), transform.velocity(fixed, {})};
}

double solar_eclipse_fraction(double lat_deg, double lon_deg, double height_m, time::Epoch epoch) {
    const spice::SpiceEphemeris ephemeris;
    const auto observer = site_state(lat_deg, lon_deg, height_m, epoch);
    return occulted_fraction(ephemeris.apparent_position_gcrf(ephemeris::Body::Sun, epoch, observer),
                             constants::sun_radius_m,
                             ephemeris.apparent_position_gcrf(ephemeris::Body::Moon, epoch, observer),
                             constants::moon_mean_radius_m);
}

}  // namespace

TEST_CASE("Disk occultation geometry", "[environment]") {
    const Vec3 source{1e11, 0, 0};
    const double source_radius = 1e9;  // apparent radius ~0.01 rad
    // Same apparent size, centred: total.
    CHECK(occulted_fraction(source, source_radius, {1e8, 0, 0}, 1e6) == Catch::Approx(1.0));
    // Far off-axis: nothing.
    CHECK(occulted_fraction(source, source_radius, {1e8, 1e7, 0}, 1e6) == 0.0);
    // Smaller occluder centred: annular, area ratio.
    CHECK(occulted_fraction(source, source_radius, {1e8, 0, 0}, 5e5) == Catch::Approx(0.25).margin(1e-4));  // (asin ratio)^2
    // Equal disks offset by one radius: overlap 2/3 - sqrt(3)/(2 pi) = 0.391.
    const double r_ang = std::asin(1e-2);
    const Vec3 shifted{1e8 * std::cos(r_ang), 1e8 * std::sin(r_ang), 0};
    CHECK(occulted_fraction(source, source_radius, shifted, 1e6) == Catch::Approx(0.391).margin(2e-3));
    // Occluder behind the source.
    CHECK(occulted_fraction(source, source_radius, {2e11, 0, 0}, 1e9) == 0.0);
}

TEST_CASE("Total solar eclipse 2024-04-08 over Dallas: contact times", "[environment][validation]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    // Dallas, TX. Published totality (NASA/timeanddate): 18:40:43 - 18:44:35 UTC.
    const double lat = 32.7767, lon = -96.7970, height = 140.0;
    const auto start = time.from_utc({2024, 4, 8, 18, 38, 0.0});

    double second_contact = -1.0, third_contact = -1.0, max_fraction = 0.0;
    for (int s = 0; s <= 9 * 60; ++s) {
        const double f = solar_eclipse_fraction(lat, lon, height, start + s);
        max_fraction = std::max(max_fraction, f);
        if (f >= 1.0 && second_contact < 0.0) second_contact = s;
        if (f >= 1.0) third_contact = s;
    }
    const double published_c2 = 2 * 60 + 43.0, published_c3 = 6 * 60 + 35.0;
    std::printf("[validation] Dallas totality: C2 %+.0f s, C3 %+.0f s vs published, duration %.0f s (published 232 s)\n",
                second_contact - published_c2, third_contact - published_c3, third_contact - second_contact);
    REQUIRE(max_fraction == 1.0);
    CHECK(std::abs(second_contact - published_c2) <= 5.0);
    CHECK(std::abs(third_contact - published_c3) <= 5.0);

    // Same instant, no eclipse in the morning.
    CHECK(solar_eclipse_fraction(lat, lon, height, time.from_utc({2024, 4, 8, 15, 0, 0.0})) == 0.0);
}

TEST_CASE("Full Moon 2024-04-23 23:49 UTC: ecliptic longitudes differ by 180 deg", "[ephemeris][validation]") {
    test::spice_session();
    const spice::SpiceTimeConverter time;
    const spice::SpiceEphemeris ephemeris;
    const auto epoch = time.from_utc({2024, 4, 23, 23, 49, 0.0});
    const math::StateVector earth_centre{};
    auto ecliptic_longitude = [&](ephemeris::Body body) {
        const Vec3 p = ephemeris.apparent_position_gcrf(body, epoch, earth_centre);
        const double eps = constants::obliquity_j2000_rad;
        const double y_ecl = std::cos(eps) * p.y + std::sin(eps) * p.z;
        return std::atan2(y_ecl, p.x);
    };
    double difference = (ecliptic_longitude(ephemeris::Body::Moon) - ecliptic_longitude(ephemeris::Body::Sun)) *
                        constants::rad_to_deg;
    difference = std::fmod(difference + 720.0, 360.0);
    std::printf("[validation] full Moon: longitude difference %.4f deg\n", difference);
    CHECK(difference == Catch::Approx(180.0).margin(0.02));  // Moon moves 0.008 deg/min relative to Sun
}
