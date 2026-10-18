// Interpolating ephemeris and orientation caches (l6 Task 1) against the SPICE sources they sample.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include <catch2/catch_test_macros.hpp>

#include "starsim/ephemeris/interpolated_ephemeris.hpp"
#include "starsim/frames/interpolated_orientation.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {

// The l5 lunar return's epoch (2026-11-15) and a month after it: every Moon phase, both directions of travel.
const time::Epoch kStart = time::Epoch::from_tdb_seconds_past_j2000(8.475e8);
constexpr double kSpanS = 30.0 * 86400.0;
constexpr int kSamples = 20000;

// Sample epochs over the span in an order that jumps back and forth (the planners do).
time::Epoch sample(int i) {
    const double golden = 0.6180339887498949;
    return kStart + kSpanS * std::fmod(i * golden, 1.0);
}

// Rotation angle of R_a^T R_b.
double angle_between(const math::Mat3& a, const math::Mat3& b) {
    const math::Mat3 d = transpose(a) * b;
    const double c = std::clamp(0.5 * (d(0, 0) + d(1, 1) + d(2, 2) - 1.0), -1.0, 1.0);
    // acos loses digits near 0: the skew part's size is the sine
    const double s = 0.5 * std::sqrt((d(2, 1) - d(1, 2)) * (d(2, 1) - d(1, 2)) + (d(0, 2) - d(2, 0)) * (d(0, 2) - d(2, 0)) +
                                     (d(1, 0) - d(0, 1)) * (d(1, 0) - d(0, 1)));
    return std::atan2(s, c);
}

double max_abs_difference(const math::Mat3& a, const math::Mat3& b) {
    double worst = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) worst = std::max(worst, std::abs(a(i, j) - b(i, j)));
    }
    return worst;
}

}  // namespace

TEST_CASE("interpolated ephemeris follows SPICE to the millimetre", "[ephemeris][interpolated]") {
    test::spice_session();
    const spice::SpiceEphemeris spice;
    const ephemeris::InterpolatedEphemeris cached(spice);
    for (const auto body : {ephemeris::Body::Moon, ephemeris::Body::Sun}) {
        double worst_position_m = 0.0, worst_velocity_mps = 0.0;
        for (int i = 0; i < kSamples; ++i) {
            const auto t = sample(i);
            const auto truth = spice.state_gcrf(body, t);
            const auto interpolated = cached.state_gcrf(body, t);
            worst_position_m = std::max(worst_position_m, norm(interpolated.position_m - truth.position_m));
            worst_velocity_mps = std::max(worst_velocity_mps, norm(interpolated.velocity_mps - truth.velocity_mps));
        }
        std::printf("[interpolated] body %d: max |dr| %.3e m, max |dv| %.3e m/s\n", static_cast<int>(body),
                    worst_position_m, worst_velocity_mps);
        // [E] mm in position (DE440's own Moon differs from DE441 by up to 3.3 m), 1e-6 m/s in velocity
        CHECK(worst_position_m < 1e-3);
        CHECK(worst_velocity_mps < 1e-6);
    }
    // the Earth is the origin; the rest passes through
    CHECK(norm(cached.state_gcrf(ephemeris::Body::Earth, kStart).position_m) == 0.0);
    CHECK(cached.gm_m3ps2(ephemeris::Body::Moon) == spice.gm_m3ps2(ephemeris::Body::Moon));
    const math::StateVector observer{{7e6, 0.0, 0.0}, {0.0, 7.5e3, 0.0}};
    CHECK(norm(cached.apparent_position_gcrf(ephemeris::Body::Moon, kStart, observer) -
               spice.apparent_position_gcrf(ephemeris::Body::Moon, kStart, observer)) == 0.0);
}

TEST_CASE("interpolated ephemeris with no node spacing passes every call through", "[ephemeris][interpolated]") {
    test::spice_session();
    const spice::SpiceEphemeris spice;
    const ephemeris::InterpolatedEphemeris direct(spice, 0.0);
    const auto t = sample(7);
    CHECK(norm(direct.state_gcrf(ephemeris::Body::Moon, t).position_m -
               spice.state_gcrf(ephemeris::Body::Moon, t).position_m) == 0.0);
}

TEST_CASE("interpolated orientation follows SPICE's Earth and Moon frames", "[frames][interpolated]") {
    test::spice_session();
    const auto earth = spice::SpiceOrientation::earth();
    const auto moon = spice::SpiceOrientation::moon_mean_earth();
    struct Case {
        const char* name;
        const frames::BodyOrientation& source;
        double surface_radius_m;
    };
    for (const Case& c : {Case{"earth", earth, constants::wgs84_semi_major_axis_m},
                          Case{"moon", moon, constants::moon_mean_radius_m}}) {
        const frames::InterpolatedOrientation cached(c.source);
        double worst_angle_rad = 0.0, worst_rate = 0.0;
        for (int i = 0; i < kSamples; ++i) {
            const auto t = sample(i);
            const auto truth = c.source.fixed_to_gcrf(t);
            const auto interpolated = cached.fixed_to_gcrf(t);
            worst_angle_rad = std::max(worst_angle_rad, angle_between(truth.rotation, interpolated.rotation));
            worst_rate = std::max(worst_rate, max_abs_difference(truth.rotation_rate_per_s, interpolated.rotation_rate_per_s));
        }
        std::printf("[interpolated] %s: max angle %.3e rad (%.3e m at the surface), max rate error %.3e 1/s\n", c.name,
                    worst_angle_rad, worst_angle_rad * c.surface_radius_m, worst_rate);
        // [E] mm at the surface; the rate to 1e-12 1/s (1e-12 * 6378 km = 6 um/s of a ground point's velocity)
        CHECK(worst_angle_rad * c.surface_radius_m < 1e-3);
        CHECK(worst_rate < 1e-12);
    }
}

TEST_CASE("interpolated orientation with no node spacing passes every call through", "[frames][interpolated]") {
    test::spice_session();
    const auto earth = spice::SpiceOrientation::earth();
    const frames::InterpolatedOrientation direct(earth, 0.0);
    const auto t = sample(11);
    CHECK(max_abs_difference(direct.fixed_to_gcrf(t).rotation, earth.fixed_to_gcrf(t).rotation) == 0.0);
}
