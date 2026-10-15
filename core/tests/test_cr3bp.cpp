// CR3BP: Jacobi constant conserved, the 9:2 NRHO seed corrected to a periodic orbit of the resonant period, and the
// rotating <-> GCRF conversion round trip.

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/cr3bp.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
// 9:2 southern L2 NRHO, state at apolune [S: Zimovan, Howell & Davis 2017, "Near rectilinear halo orbits and their
// application in cis-lunar space", Table 1 (Earth-Moon mu = 0.01215)].
const orbit::Cr3bpState nrho_seed{{1.0221, 0.0, -0.1821}, {0.0, -0.1033, 0.0}};
}  // namespace

TEST_CASE("CR3BP conserves the Jacobi constant", "[orbit][cr3bp]") {
    const double mu = 0.01215;
    const double c0 = orbit::jacobi_constant(mu, nrho_seed);
    const auto later = orbit::cr3bp_propagate(mu, nrho_seed, 1.5);
    CHECK(orbit::jacobi_constant(mu, later) == Catch::Approx(c0).epsilon(1e-10));
}

TEST_CASE("CR3BP corrects the 9:2 NRHO seed to the resonant period", "[orbit][cr3bp][validation]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    const orbit::EarthMoonRotatingFrame frame{ephemeris, t0};
    const double mu = frame.mu();
    // 9 revolutions in 2 synodic months [S: Zimovan et al. 2017].
    const double period_s = 2.0 * constants::synodic_month_s / 9.0;
    // Mean time unit: sidereal month / 2 pi (the CR3BP's rotating rate), not the instantaneous one.
    const double period = period_s / (constants::sidereal_month_s / (2.0 * constants::pi));
    const auto orbit = orbit::correct_periodic_orbit(mu, nrho_seed, period);
    REQUIRE(orbit);
    const auto half = orbit::cr3bp_propagate(mu, *orbit, period / 2.0);
    CHECK(std::abs(half.position.y) < 1e-9);
    CHECK(std::abs(half.velocity.x) < 1e-9);
    CHECK(std::abs(half.velocity.z) < 1e-9);
    // Perilune / apolune radii about the Moon (nondimensional -> km with the mean distance 384,400 km).
    double rp = 1e9, ra = 0.0;
    for (int i = 0; i <= 2000; ++i) {
        const auto s = orbit::cr3bp_propagate(mu, *orbit, period * i / 2000.0);
        const double r = norm(s.position - math::Vec3{1.0 - mu, 0.0, 0.0}) * 384400.0;
        rp = std::min(rp, r);
        ra = std::max(ra, r);
    }
    std::printf("[validation] CR3BP 9:2 NRHO: perilune %.0f km, apolune %.0f km, z0 %.5f\n", rp, ra, orbit->position.z);
    // [S: Zimovan et al. 2017; Lee 2019 (NASA Gateway NRHO): perilune radius ~3,200-3,400 km, apolune ~70,000 km]
    CHECK(rp > 3000.0);
    CHECK(rp < 3700.0);
    CHECK(ra > 68000.0);
    CHECK(ra < 72500.0);
    CHECK(orbit->position.z < 0.0);  // southern: apolune below the Earth-Moon plane
}

TEST_CASE("Earth-Moon rotating frame round trip", "[orbit][cr3bp]") {
    test::spice_session();
    spice::SpiceEphemeris ephemeris;
    const time::Epoch t0 = spice::SpiceTimeConverter().from_utc({2026, 11, 2, 0, 0, 0.0});
    const orbit::EarthMoonRotatingFrame frame{ephemeris, t0};
    // The Moon itself maps to (1 - mu, 0, 0) at rest in the rotating frame (up to the eccentric radial rate).
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, t0);
    const auto moon_rot = frame.from_gcrf({moon.position_m, moon.velocity_mps});
    CHECK(moon_rot.position.x == Catch::Approx(1.0 - frame.mu()).margin(1e-12));
    CHECK(std::abs(moon_rot.position.y) < 1e-12);
    CHECK(std::abs(moon_rot.position.z) < 1e-12);
    const auto back = frame.to_gcrf(frame.from_gcrf({moon.position_m, moon.velocity_mps}));
    CHECK(norm(back.position_m - moon.position_m) < 1e-3);
    CHECK(norm(back.velocity_mps - moon.velocity_mps) < 1e-6);
}
