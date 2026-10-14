// Reference bodies (lunar l1): body-neutral frames of the Earth and the Moon.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
constexpr double deg = constants::deg_to_rad;
}

TEST_CASE("Spherical shapes convert geodetic coordinates exactly", "[environment][reference_body]") {
    const frames::Ellipsoid sphere{constants::moon_mean_radius_m, 0.0};
    for (const double lat : {-89.5, -45.0, 0.0, 30.0, 89.9}) {
        for (const double lon : {-170.0, 0.0, 95.0}) {
            const frames::GeodeticPosition g{lat * deg, lon * deg, 1234.5};
            const Vec3 fixed = frames::geodetic_to_fixed(g, sphere);
            CHECK(norm(fixed) == Catch::Approx(constants::moon_mean_radius_m + 1234.5).epsilon(1e-14));
            const auto back = frames::fixed_to_geodetic(fixed, sphere);
            CHECK(back.latitude_rad == Catch::Approx(g.latitude_rad).margin(1e-12));
            CHECK(back.longitude_rad == Catch::Approx(g.longitude_rad).margin(1e-12));
            CHECK(back.height_m == Catch::Approx(1234.5).margin(1e-6));
        }
    }
}

TEST_CASE("The Earth reference body is centred at the GCRF origin and reproduces ITRF", "[environment][reference_body]") {
    test::spice_session();
    const spice::SpiceOrientation itrf = spice::SpiceOrientation::earth();
    const environment::Us76Atmosphere atmosphere;
    const auto earth = environment::ReferenceBody::earth(itrf, atmosphere, constants::earth_gm_m3ps2);
    const auto t = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto frame = earth.frame(t);
    const auto direct = itrf.fixed_to_gcrf(t);
    CHECK(earth.id() == ephemeris::Body::Earth);
    CHECK(earth.has_atmosphere());
    CHECK(frame.centre_gcrf_m == Vec3{});
    CHECK(frame.centre_velocity_gcrf_mps == Vec3{});
    const Vec3 r{6.5e6, 1.0e5, 2.0e5}, v{100.0, 7500.0, 30.0};
    // Bit-identical to the ITRF arithmetic used so far.
    CHECK(frame.to_fixed(r) == transpose(direct.rotation) * r);
    CHECK(frame.relative_velocity(r, v) == v - cross(frames::rotation_vector(direct), r));
    CHECK(norm(frame.rotation_gcrf_radps) == Catch::Approx(7.2921e-5).epsilon(1e-4));
    CHECK(earth.shape().semi_major_axis_m == constants::wgs84_semi_major_axis_m);
    CHECK(earth.air(0.0).density_kg_m3 == Catch::Approx(1.225).epsilon(1e-3));
}

TEST_CASE("The Moon reference body follows DE440 and MOON_ME", "[environment][reference_body]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation moon_me = spice::SpiceOrientation::moon_mean_earth();
    const auto moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    const auto t = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    const auto frame = moon.frame(t);
    const auto centre = ephemeris.state_gcrf(ephemeris::Body::Moon, t);
    CHECK(moon.id() == ephemeris::Body::Moon);
    CHECK_FALSE(moon.has_atmosphere());
    CHECK(moon.gm_m3ps2() == ephemeris.gm_m3ps2(ephemeris::Body::Moon));
    CHECK(moon.shape().semi_major_axis_m == constants::moon_mean_radius_m);
    CHECK(moon.shape().flattening == 0.0);
    CHECK(frame.centre_gcrf_m == centre.position_m);
    CHECK(frame.centre_velocity_gcrf_mps == centre.velocity_mps);
    CHECK(moon.centre_gcrf(t).position_m == centre.position_m);
    // Sidereal rotation 27.32 d.
    CHECK(norm(frame.rotation_gcrf_radps) == Catch::Approx(2.0 * constants::pi / (27.321661 * 86400.0)).epsilon(0.02));
    // A point fixed on the surface moves with the Moon: its GCRF velocity is the centre's plus omega x r.
    const Vec3 fixed = frames::geodetic_to_fixed({-89.0 * deg, 0.0, 0.0}, moon.shape());
    const Vec3 r = frame.position_gcrf(fixed);
    CHECK(norm(r - centre.position_m) == Catch::Approx(constants::moon_mean_radius_m).epsilon(1e-12));
    const Vec3 v = frame.velocity_gcrf(fixed, {});
    CHECK(norm(frame.relative_velocity(r, v)) < 1e-9);
    CHECK(norm(frame.to_fixed(r) - fixed) < 1e-6);
    // Vacuum: no air, engines see no back-pressure, the free path is infinite.
    const auto air = moon.air(100.0);
    CHECK(air.density_kg_m3 == 0.0);
    CHECK(air.pressure_pa == 0.0);
    CHECK(std::isinf(air.mean_free_path_m));
    CHECK(air.speed_of_sound_mps > 0.0);  // Mach stays finite (zero dynamic pressure)
}
