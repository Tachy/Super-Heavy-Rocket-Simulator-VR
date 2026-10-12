#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "starsim/frames/geodetic.hpp"

using namespace starsim;
using frames::GeodeticPosition;

namespace {
constexpr double deg = constants::deg_to_rad;
}

TEST_CASE("Geodetic -> ITRF reference points on WGS84", "[frames]") {
    const auto equator = frames::geodetic_to_fixed({0.0, 0.0, 0.0});
    CHECK(equator.x == Catch::Approx(6378137.0).margin(1e-6));
    CHECK(equator.y == Catch::Approx(0.0).margin(1e-6));

    const auto pole = frames::geodetic_to_fixed({90.0 * deg, 0.0, 0.0});
    CHECK(pole.z == Catch::Approx(frames::wgs84.semi_minor_axis_m()).margin(1e-6));
    CHECK(frames::wgs84.semi_minor_axis_m() == Catch::Approx(6356752.314245).margin(1e-6));
}

TEST_CASE("Geodetic round trip is exact to well below a millimetre", "[frames]") {
    const double lat = GENERATE(-89.999, -60.0, -12.5, 0.0, 25.99686, 28.60822, 51.4779, 89.9999);
    const double lon = GENERATE(-179.9, -97.15394, -80.6041, 0.0, 13.4);
    const double height = GENERATE(-400.0, 0.0, 10.0, 8848.0, 4.0e5, 3.6e7);

    const GeodeticPosition original{lat * deg, lon * deg, height};
    const auto back = frames::fixed_to_geodetic(frames::geodetic_to_fixed(original));
    // 1e-11 rad ~ 0.06 mm on the surface
    CHECK(back.latitude_rad == Catch::Approx(original.latitude_rad).margin(1e-11));
    CHECK(back.longitude_rad == Catch::Approx(original.longitude_rad).margin(1e-11));
    CHECK(back.height_m == Catch::Approx(original.height_m).margin(1e-4));
}

TEST_CASE("Geodetic conversion on the rotation axis", "[frames]") {
    const auto north = frames::fixed_to_geodetic({0.0, 0.0, 7.0e6});
    CHECK(north.latitude_rad == Catch::Approx(constants::pi / 2.0));
    CHECK(north.height_m == Catch::Approx(7.0e6 - frames::wgs84.semi_minor_axis_m()).margin(1e-6));
}
