// ISS i4: translation loop (deadband) and the approach laws.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/translation_loop.hpp"

using namespace starsim;
using math::Vec3;

namespace {
gnc::AxisAcceleration dragon() { return {{0.1245, 0.1245, 0.0453, 0.0453, 0.0453, 0.0453}}; }  // i2 measured
}

TEST_CASE("Deadband: no pulse below start, fire until below stop", "[iss][proximity]") {
    gnc::TranslationLoop loop;
    const gnc::TranslationLoopParameters p;  // start 0.02, stop 0.005
    CHECK(norm(loop.command({0.015, 0.0, 0.0}, {}, dragon(), p)) == 0.0);
    const Vec3 f = loop.command({0.03, 0.0, 0.0}, {}, dragon(), p);
    CHECK(f.x == Catch::Approx(0.03 / 0.1245));
    CHECK(loop.firing());
    CHECK(loop.command({0.03, 0.0, 0.0}, {0.02, 0.0, 0.0}, dragon(), p).x > 0.0);  // 0.01 > stop: still firing
    CHECK(loop.command({0.03, 0.0, 0.0}, {0.027, 0.0, 0.0}, dragon(), p).x == 0.0);  // 0.003 < stop
    CHECK_FALSE(loop.firing());
}

TEST_CASE("Loop uses the authority of the commanded sign and saturates", "[iss][proximity]") {
    gnc::TranslationLoop loop;
    const gnc::TranslationLoopParameters p;
    const Vec3 f = loop.command({0.0, -1.0, 0.5}, {}, dragon(), p);
    CHECK(norm(f) == Catch::Approx(1.0));
    CHECK(f.y < 0.0);
    CHECK(f.z > 0.0);
}

TEST_CASE("Simulated 1D closing converges to the desired speed", "[iss][proximity]") {
    gnc::TranslationLoop loop;
    const gnc::TranslationLoopParameters p{1.0, 0.01, 0.002};
    double v = 0.0;
    for (int k = 0; k < 2000; ++k) {  // 10 s at 200 Hz
        const Vec3 f = loop.command({-0.1, 0.0, 0.0}, {v, 0.0, 0.0}, dragon(), p);
        v += f.x * dragon().along(0, f.x) * 0.005;
    }
    CHECK(v == Catch::Approx(-0.1).margin(0.003));
}

TEST_CASE("Axial speed schedule", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    CHECK(gnc::axial_closing_speed_mps(220.0, p) == Catch::Approx(0.30));
    CHECK(gnc::axial_closing_speed_mps(120.0, p) == Catch::Approx(0.20));
    CHECK(gnc::axial_closing_speed_mps(40.0, p) == Catch::Approx(0.10));
    CHECK(gnc::axial_closing_speed_mps(20.0, p) == Catch::Approx(0.10));
    CHECK(gnc::axial_closing_speed_mps(10.0, p) == Catch::Approx(0.0875));
    CHECK(gnc::axial_closing_speed_mps(0.0, p) == Catch::Approx(0.075));
}

TEST_CASE("Approach and hold velocities", "[iss][proximity]") {
    const gnc::ProximityParameters p;
    const Vec3 v = gnc::approach_velocity_port({20.0, 0.5, 0.0}, p);
    CHECK(v.x == Catch::Approx(-0.10));
    CHECK(v.y == Catch::Approx(-0.01));
    CHECK(norm(Vec3{0, gnc::approach_velocity_port({20.0, 5.0, 5.0}, p).y,
                    gnc::approach_velocity_port({20.0, 5.0, 5.0}, p).z}) == Catch::Approx(0.03));
    CHECK(norm(gnc::hold_velocity({10.0, 0, 0}, {0, 0, 0}, p)) == Catch::Approx(0.05));
    CHECK(gnc::hold_velocity({1.2, 0, 0}, {0, 0, 0}, p).x == Catch::Approx(-0.02));
}
