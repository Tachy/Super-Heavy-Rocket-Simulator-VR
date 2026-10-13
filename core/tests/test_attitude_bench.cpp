// Bench (hidden, [.bench]): how closely the landing attitude controller follows a turning reference.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Quat;
using math::Vec3;

TEST_CASE("Attitude bench: 13 engines following a turning axis", "[.bench]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(test::booster_config("starship_block3", 70000.0), STARSIM_CONFIG_DIR),
                             test::state_over_site(session.launch_site(), earth, t0, {0.0, 0.0, 900.0}, {0.0, 0.0, -200.0}));
    const auto& env = session.environment();
    const double wn = std::getenv("B_WN") ? std::atof(std::getenv("B_WN")) : 1.5;
    const bool fins = !std::getenv("B_NOFINS");
    const gnc::AttitudeController controller{gnc::AttitudeController::Gains{wn, 0.9, fins}};
    const double rate = 0.1;  // rad/s about site north (tilts the axis towards east)
    vehicle::VehicleCommand cmd;
    cmd.groups = {{"booster/center", true, 0.7}, {"booster/middle", true, 0.7}};
    double max_err = 0.0;
    for (int i = 0; i < 1400; ++i) {  // 7 s: 3 s spool-up, then the turn
        const double t = i * 0.005;
        const auto epoch = t0 + t;
        const auto tr = earth.fixed_to_gcrf(epoch);
        const double sl = std::sin(session.launch_site().latitude_rad), cl = std::cos(session.launch_site().latitude_rad);
        const double so = std::sin(session.launch_site().longitude_rad), co = std::cos(session.launch_site().longitude_rad);
        const Vec3 east{-so, co, 0.0}, up{cl * co, cl * so, sl};
        const double angle = t < 3.0 ? 0.0 : std::min(rate * (t - 3.0), 0.25);
        const Vec3 axis_itrf = up * std::cos(angle) + east * std::sin(angle);
        const Vec3 axis = tr.rotation * axis_itrf;
        gnc::AttitudeController::Feedforward ff;
        const Vec3 north_axis = cross(up, east);  // rotation axis
        const Vec3 omega_gcrf = tr.rotation * (north_axis * (t >= 3.0 && angle < 0.25 ? rate : 0.0));
        ff.rate_body_radps = body.state().attitude.inverse_rotate(omega_gcrf + vehicle::earth_rotation_vector(tr));
        const double pressure = env.atmosphere.at(900.0).pressure_pa;
        controller.steer(body, pressure, axis, {}, std::getenv("B_NOFF") ? nullptr : &ff).apply_to(cmd);
        body.step(env, epoch, 0.005, cmd);
        const Vec3 actual = transpose(tr.rotation) * body.state().attitude.rotate({1.0, 0.0, 0.0});
        const double err = std::acos(std::min(1.0, dot(actual, axis_itrf)));
        if (t > 3.5) max_err = std::max(max_err, err);
        if (i % 40 == 0 && t >= 3.0)
            std::printf("t %.2f ref %.3f act %.3f err %.2f deg gimbal %.2f deg\n", t, angle,
                        std::atan2(dot(actual, east), dot(actual, up)), err * 57.3, cmd.gimbal_pitch_rad * 57.3 + cmd.gimbal_yaw_rad * 0.0);
    }
    std::printf("max error after 3.5 s: %.2f deg\n", max_err * 57.3);
}
