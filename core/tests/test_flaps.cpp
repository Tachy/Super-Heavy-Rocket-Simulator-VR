#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/aero/flap.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Approx;

namespace {

constexpr double pi = constants::pi;

aero::FlapTable table() {
    config::CsvTable t;
    t.columns = {"mach", "cn_max"};
    t.rows = {{0.0, 1.2}, {5.0, 1.84}};
    return aero::FlapTable(t);
}

// Belly towards body -z (azimuth 270 deg), flap hinged on the +y side.
aero::FlapGeometry flap() {
    aero::FlapGeometry f;
    f.station_fwd_m = 14.0;
    f.station_aft_m = 2.0;
    f.hinge_azimuth_rad = 0.0;
    f.hull_radius_m = 4.5;
    f.span_m = 4.0;
    f.area_m2 = 48.0;
    f.belly_azimuth_rad = 1.5 * pi;
    return f;
}

const aero::FinAirData air{1e-3, 300.0, 0.0};

}  // namespace

TEST_CASE("Flap geometry: stowed edge-on to a belly-first flow, extended broadside", "[aero][flap]") {
    const auto f = flap();
    const auto s0 = aero::flap_span_direction(f, 0.0);  // folded leeward: +z, away from the belly at -z
    CHECK(s0.z == Approx(1.0).margin(1e-12));
    const auto s90 = aero::flap_span_direction(f, pi / 2);
    CHECK(s90.y == Approx(1.0).margin(1e-12));  // radial at the +y hinge
    CHECK(std::abs(aero::flap_normal(f, pi / 2).z) == Approx(1.0).margin(1e-12));
    const auto c = aero::flap_centroid(f, pi / 2);
    CHECK(c.x == Approx(8.0));
    CHECK(c.y == Approx(6.5));
    CHECK(c.z == Approx(0.0).margin(1e-12));
}

TEST_CASE("Extended flap in a belly-first flow carries Newtonian drag", "[aero][flap]") {
    const math::Vec3 v{0.0, 0.0, -1500.0};  // moving towards the belly (-z), Mach 5
    const double q = 0.5 * 1e-3 * 1500.0 * 1500.0;
    const auto r = aero::flap_force(flap(), pi / 2, v, air, table());
    CHECK(r.force_body_n.z == Approx(1.84 * 48.0 * q));  // opposes the motion
    CHECK(std::abs(r.force_body_n.x) < 1e-6);
    const auto stowed = aero::flap_force(flap(), 0.0, v, air, table());
    CHECK(norm(stowed.force_body_n) < 1e-6 * q);
    const auto half = aero::flap_force(flap(), pi / 4, v, air, table());
    CHECK(norm(half.force_body_n) == Approx(1.84 * 48.0 * q * 0.5));
}

TEST_CASE("Flaps in the hull's lee or in vacuum carry no force", "[aero][flap]") {
    const math::Vec3 back_first{0.0, 0.0, 1500.0};  // moving leeward
    CHECK(norm(aero::flap_force(flap(), pi / 2, back_first, air, table()).force_body_n) == 0.0);
    const aero::FinAirData vacuum{0.0, 0.0, 1.0};
    const auto r = aero::flap_force(flap(), pi / 2, math::Vec3{0.0, 0.0, -7000.0}, vacuum, table());
    CHECK(norm(r.force_body_n) == 0.0);
    CHECK(std::isfinite(r.application_point_m.x));
}

TEST_CASE("Stowed flaps in nose-first flow give no force", "[aero][flap]") {
    const math::Vec3 nose_first{-2000.0, 0.0, 0.0};  // flow along the axis
    CHECK(norm(aero::flap_force(flap(), 0.0, nose_first, air, table()).force_body_n) == 0.0);
}
