#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "starsim/gnc/tvc_lateral.hpp"

using namespace starsim::gnc;

namespace {

// Linear lateral TVC model: p' = v, v' = a theta - c u, theta' = w, w' = u (u: angular acceleration command).
struct LinearState {
    double p = 0.0, v = 0.0, theta = 0.0, w = 0.0;
};

LinearState step(const LinearState& s, double a, double c, double u, double dt) {
    LinearState n = s;
    n.p += s.v * dt;
    n.v += (a * s.theta - c * u) * dt;
    n.theta += s.w * dt;
    n.w += u * dt;
    return n;
}

}  // namespace

TEST_CASE("LQR lateral control of a gimbaled booster settles an offset despite the wrong-way gimbal force", "[gnc][tvc]") {
    // Three engines near minimum thrust: thrust acceleration 10 m/s^2, c = I / (m L) ~ 23 m.
    const double a = 10.0, c = 23.0;
    const TvcLateralWeights weights;
    const auto gains = tvc_lateral_gains(a, c, weights);
    LinearState s;
    s.p = 5.0;  // 5 m off the plan
    double max_theta = 0.0;
    for (int i = 0; i < 4000; ++i) {  // 20 s at 200 Hz
        const double u = -(gains.k[0] * s.p + gains.k[1] * s.v + gains.k[2] * s.theta + gains.k[3] * s.w);
        s = step(s, a, c, u, 0.005);
        max_theta = std::max(max_theta, std::abs(s.theta));
    }
    CHECK(std::abs(s.p) < 0.1);
    CHECK(std::abs(s.v) < 0.05);
    CHECK(max_theta < 5.0 * 3.14159265358979 / 180.0);  // a gentle correction
}

TEST_CASE("LQR lateral gains also hold the agile ignition set", "[gnc][tvc]") {
    const double a = 80.0, c = 23.0;
    const auto gains = tvc_lateral_gains(a, c, TvcLateralWeights{});
    LinearState s;
    s.v = 3.0;  // 3 m/s sideways
    for (int i = 0; i < 2000; ++i) {
        const double u = -(gains.k[0] * s.p + gains.k[1] * s.v + gains.k[2] * s.theta + gains.k[3] * s.w);
        s = step(s, a, c, u, 0.005);
    }
    CHECK(std::abs(s.v) < 0.1);
    CHECK(std::isfinite(s.p));
}
