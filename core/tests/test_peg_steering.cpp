// PEG about the Moon (l4): a point mass with constant thrust flown on PegSteering from a vertical rise reaches the
// insertion point of a 15 x 100 km orbit.

#include <algorithm>
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/peg_steering.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("PEG steers a lunar ascent into a 15 x 100 km insertion", "[gnc][peg]") {
    const double gm = constants::moon_gm_m3ps2;
    const double R = constants::moon_mean_radius_m;
    const double rp = R + 15e3, ra = R + 100e3;
    gnc::PegTarget target;
    target.radius_m = rp;
    target.horizontal_speed_mps = std::sqrt(gm * (2.0 / rp - 2.0 / (rp + ra)));
    target.plane_normal = {0.0, 0.0, 1.0};
    target.terminal_semi_major_axis_m = 0.5 * (rp + ra);
    gnc::PegSteering peg(gm);

    // 600 t on three sea-level Raptors in vacuum (2.912 MN, 350 s each), 300 m above the sphere, rising 20 m/s.
    double m = 600e3;
    const double thrust = 3.0 * 2.912e6, flow = thrust / (350.0 * constants::standard_gravity_mps2);
    Vec3 r{R + 300.0, 0.0, 0.0}, v{20.0, 0.0, 0.0};
    time::Epoch t = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    bool cut = false;
    const double shutdown_s = 0.5;
    for (int i = 0; i < 40000 && !cut; ++i) {
        const auto cmd = peg.update({r, v}, {thrust, flow, m, shutdown_s}, target, t);
        if (cmd.cutoff) {
            // The tail-off PEG leaves to the shutdown transient (half thrust over the shutdown time), as an impulse.
            v += cmd.thrust_direction * (0.5 * thrust / m * shutdown_s);
            cut = true;
            break;
        }
        const double dt = 0.005;  // the session's 200 Hz: at ~24 m/s^2 a 0.1 s step alone moves the aposelene ~10 km
        auto accel = [&](const Vec3& rr, double mm) {
            return cmd.thrust_direction * (thrust / mm) - rr * (gm / std::pow(norm(rr), 3));
        };
        // RK4 on (r, v), mass linear
        const Vec3 k1v = accel(r, m), k1r = v;
        const Vec3 k2v = accel(r + k1r * (dt / 2), m - flow * dt / 2), k2r = v + k1v * (dt / 2);
        const Vec3 k3v = accel(r + k2r * (dt / 2), m - flow * dt / 2), k3r = v + k2v * (dt / 2);
        const Vec3 k4v = accel(r + k3r * dt, m - flow * dt), k4r = v + k3v * dt;
        r += (k1r + k2r * 2.0 + k3r * 2.0 + k4r) * (dt / 6.0);
        v += (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
        m -= flow * dt;
        t = t + dt;
    }
    REQUIRE(cut);
    // Orbit after cutoff: periapsis ~15 km, apoapsis ~100 km within 1 km.
    const double rn = norm(r), vn2 = dot(v, v);
    const double a = 1.0 / (2.0 / rn - vn2 / gm);
    const Vec3 h = cross(r, v);
    const double e = std::sqrt(std::max(0.0, 1.0 - dot(h, h) / (gm * a)));
    CHECK(a * (1.0 - e) - R == Catch::Approx(15e3).margin(1e3));
    CHECK(a * (1.0 + e) - R == Catch::Approx(100e3).margin(1e3));
    CHECK(std::abs(h.z) / norm(h) > 0.9999);  // stays in the target plane
}
