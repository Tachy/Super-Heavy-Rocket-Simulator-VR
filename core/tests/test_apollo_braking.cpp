// Apollo braking-phase guidance (Klumpp 1974): the cubic for t_go and the quadratic law reach the target exactly in
// uniform gravity; an Apollo-11-like LM from PDI reaches Klumpp's high gate in about the flown braking time.
// References:
// [S] Klumpp, "Apollo lunar-descent guidance", MIT CSDL R-695 (1971) (PDF pages):
//     p. 10: P63 ignites at typically 492 km slant range, the transfer takes typically 514 s;
//     p. 13: P64 initial state (high gate) typically 2.2 km altitude, 7.5 km ground range, -44 m/s vertical and
//            129 m/s forward velocity;
//     p. 15: DPS rated thrust 46 706 N; p. 34: 26 s at minimum (trim) thrust after ignition;
//     p. 25: P63 terminal thrust 57 % of rated, terminal Z jerk from zero rate of change of thrust;
//     p. 60: terminal pitch angle (thrust from the vertical) typically around 60 deg;
//     p. 62: terminal jerk Z = KJ x (thrust-acceleration rate from the mass flow), KJ typically 1.2.
//     Klumpp's targets themselves (Luminary pad loads) are not tabulated there: the test builds them with that recipe.
// [S] Bennett, "Apollo lunar descent and ascent trajectories", NASA TM X-58040 (1970) (PDF pages):
//     p. 7: DPS maximum (fixed) thrust 92.5-95.5 % of the 10 500 lbf design thrust (~94 % used here);
//     p. 10: PDI ~260 nmi from the landing site at ~50 000 ft, orbit velocity ~5560 ft/s; 10 % throttle for 26 s;
//     p. 12: braking phase ends at TGO 60 s, ~7000 ft, 8 min 25 s (505 s) after ignition (premission).
// [S] Apollo 11 mission report MSC-00171 (1969): PDI 102:33:05, P64 102:41:32 GET -> 507 s flown braking phase.
// [S] Apollo 11 LM mass 15 103 kg at launch (Wikipedia "Apollo 11", spacecraft data); DOI (~75 ft/s on the DPS,
//     TM X-58040 p. 7) leaves ~14 990 kg at PDI [D]. DPS vacuum Isp 311 s [S: Wikipedia "Descent propulsion system"].
// [D] P63 terminal mass estimate 8800 kg (14 990 kg minus ~6200 kg of the ~6600 kg descent propellant, R-695 p. 25).

#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/gnc/apollo_braking.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

TEST_CASE("Klumpp's time-to-go cubic has the profile's duration as its root", "[gnc][braking]") {
    gnc::BrakingTarget t;
    t.position_m = {0.0, 0.0, 50.0};
    t.velocity_mps = {0.0, 0.0, 10.0};
    t.acceleration_mps2 = {0.0, 0.0, 1.0};
    t.jerk_downrange_mps3 = 0.0;
    const auto t_go = gnc::braking_time_to_go_s(t, 0.0, 0.0, 7.0);
    REQUIRE(t_go);
    CHECK(*t_go == Catch::Approx(10.0).margin(1e-9));
    CHECK(gnc::braking_acceleration(t, {}, {}, 10.0).z == Catch::Approx(1.0).margin(1e-12));
}

TEST_CASE("The quadratic law reaches position, velocity and acceleration in uniform gravity", "[gnc][braking]") {
    gnc::BrakingTarget t;
    t.position_m = {2000.0, 0.0, -3000.0};
    t.velocity_mps = {-35.0, 0.0, 90.0};
    t.acceleration_mps2 = {0.5, 0.0, -1.0};
    t.jerk_downrange_mps3 = 0.01;
    gnc::ApolloBrakingGuidance guidance(t, 300.0);
    const Vec3 g{-1.62, 0.0, 0.0};
    Vec3 r{15000.0, 200.0, -450000.0}, v{0.0, 0.0, 1690.0};
    double t_go = 1e9;
    for (int i = 0; i < 100000; ++i) {
        const auto cmd = guidance.update(r, v, g);
        if (!cmd.valid || cmd.time_to_go_s < 2.0) { t_go = cmd.time_to_go_s; break; }
        const Vec3 a = cmd.thrust_acceleration_mps2 + g;  // ideal thrust: the law is flown exactly
        const double dt = 0.01;
        r += v * dt + a * (0.5 * dt * dt);
        v += a * dt;
    }
    // Stopped 2 s before the target: the remaining quadratic profile ends on it.
    CHECK(t_go < 2.0);
    CHECK(norm(r + v * t_go - t.position_m) < 400.0);  // coarse: 2 s of flight left
    CHECK(norm(v - t.velocity_mps) < 10.0);
}

TEST_CASE("An Apollo 11 LM from PDI reaches the high gate in about the flown braking time",
          "[gnc][braking][validation]") {
    // Point-mass LM over a flat Moon with constant gravity (the braking phase covers ~450 km: the curvature term
    // v^2/r ~ 1.6 m/s^2 is included as an upward acceleration v_z^2 / (R + h), as in Klumpp's guidance frame).
    constexpr double ft = 0.3048;
    const double g0 = constants::standard_gravity_mps2;
    const double g_moon = constants::moon_gm_m3ps2 / (constants::moon_mean_radius_m * constants::moon_mean_radius_m);
    const double rated_n = 46706.0;
    const double max_thrust_n = 0.94 * rated_n;
    const double trim_thrust_n = 0.10 * rated_n;
    const double trim_s = 26.0;
    const double isp_s = 311.0;
    const double flown_braking_s = 507.0;
    double m = 14990.0;
    const auto gravity = [&](const Vec3& r, const Vec3& v) {
        return Vec3{-g_moon + v.z * v.z / (constants::moon_mean_radius_m + r.x), 0.0, 0.0};
    };

    // Klumpp's P63 targeting recipe (R-695 pp. 25, 60, 62): terminal state = the P64 initial state, terminal thrust
    // 57 % of rated along a 60 deg pitch from the vertical (backwards), terminal Z jerk = KJ x the thrust-acceleration
    // rate from the mass flow at that thrust.
    const double terminal_mass_kg = 8800.0;
    const double terminal_thrust_n = 0.57 * rated_n;
    const double pitch_rad = 60.0 * constants::deg_to_rad;
    const double kj = 1.2;
    const Vec3 thrust_dir{std::cos(pitch_rad), 0.0, -std::sin(pitch_rad)};
    gnc::BrakingTarget target;
    target.position_m = {2200.0, 0.0, -7500.0};
    target.velocity_mps = {-44.0, 0.0, 129.0};
    target.acceleration_mps2 =
        thrust_dir * (terminal_thrust_n / terminal_mass_kg) + gravity(target.position_m, target.velocity_mps);
    const double mass_flow_kgps = terminal_thrust_n / (isp_s * g0);
    target.jerk_downrange_mps3 =
        kj * thrust_dir.z * terminal_thrust_n * mass_flow_kgps / (terminal_mass_kg * terminal_mass_kg);

    gnc::ApolloBrakingGuidance guidance(target, 500.0);
    Vec3 r{50000.0 * ft, 0.0, -260.0 * 1852.0}, v{0.0, 0.0, 5560.0 * ft};
    const double m0 = m;
    const double dt = 0.01;
    double t = 0.0, t_go = 1e9;
    Vec3 a_end;
    for (int i = 0; i < 200000; ++i) {
        const Vec3 g = gravity(r, v);
        const auto cmd = guidance.update(r, v, g);
        if (!cmd.valid || cmd.time_to_go_s < 2.0) { t_go = cmd.time_to_go_s; a_end = cmd.thrust_acceleration_mps2 + g; break; }
        const double wanted_n = norm(cmd.thrust_acceleration_mps2) * m;
        const double thrust_n = t < trim_s ? trim_thrust_n : std::min(wanted_n, max_thrust_n);
        const Vec3 a = math::normalized(cmd.thrust_acceleration_mps2) * (thrust_n / m) + g;
        r += v * dt + a * (0.5 * dt * dt);
        v += a * dt;
        m -= thrust_n / (isp_s * g0) * dt;
        t += dt;
    }
    const Vec3 r_end = r + v * t_go + a_end * (0.5 * t_go * t_go);
    const Vec3 v_end = v + a_end * t_go;
    const double duration_s = t + t_go;
    std::printf("Apollo braking: %.1f s (flown %.0f s), propellant %.0f kg, end mass %.0f kg, "
                "position error %.1f m, velocity error %.2f m/s\n",
                duration_s, flown_braking_s, m0 - m, m, norm(r_end - target.position_m),
                norm(v_end - target.velocity_mps));
    CHECK(std::abs(duration_s - flown_braking_s) < 0.1 * flown_braking_s);
    CHECK(norm(r_end - target.position_m) < 100.0);
    CHECK(norm(v_end - target.velocity_mps) < 2.0);
}
