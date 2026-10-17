// AttitudeController integral action (l5 Task 10): pointing error, windowed integration, integral torque.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>

#include "return_scenario.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using Catch::Approx;
using math::Quat;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

}  // namespace

TEST_CASE("Attitude controller integral: pointing error, windowed integration and torque", "[attitude][integral]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const auto t0 = spice::SpiceTimeConverter{}.from_utc({2026, 10, 12, 12, 0, 0.0});
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    session.set_launch_site(sim::launch_site_from_config(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1")));
    vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(test::booster_config("starship_block3", 70000.0), STARSIM_CONFIG_DIR),
                             test::state_over_site(session.launch_site(), earth, t0, {0.0, 0.0, 900.0}, {0.0, 0.0, -200.0}));
    const Quat& q = body.state().attitude;

    // the wanted axis 3 deg from the body x axis, turned about body y; the roll reference on body +y
    const Vec3 axis_body{std::cos(3.0 * deg), 0.0, -std::sin(3.0 * deg)};  // a +y rotation turns x towards -z
    const Vec3 roll_ref_body{0.0, std::cos(1.0 * deg), std::sin(1.0 * deg)};  // 1 deg about +x
    const Vec3 error = gnc::AttitudeController::pointing_error(body, q.rotate(axis_body), q.rotate(roll_ref_body));
    CHECK(error.x == Approx(1.0 * deg).margin(1e-9));
    CHECK(error.y == Approx(3.0 * deg).margin(1e-9));
    CHECK(error.z == Approx(0.0).margin(1e-9));
    CHECK(gnc::AttitudeController::pointing_error(body, q.rotate(axis_body)).x == 0.0);  // no roll reference

    gnc::AttitudeController::Gains gains;
    gains.integral_gain = 0.05;
    gains.integral_window_rad = 2.0 * deg;
    const gnc::AttitudeController controller(gains);
    // inside the window (1 deg) integrated, outside (3 deg) not
    const Vec3 integral = controller.integrate_error({0.1, 0.2, 0.3}, error, 0.5);
    CHECK(integral.x == Approx(0.1 + 0.5 * 1.0 * deg));
    CHECK(integral.y == Approx(0.2));
    CHECK(integral.z == Approx(0.3));
    const Vec3 inertia = body.vehicle().mass_properties().inertia_kg_m2;
    const Vec3 torque = controller.integral_torque(body, integral);
    CHECK(torque.x == Approx(inertia.x * 0.05 * integral.x));
    CHECK(torque.y == Approx(inertia.y * 0.05 * integral.y));
    CHECK(torque.z == Approx(inertia.z * 0.05 * integral.z));

    // without integral gain the integral stays as it is
    const gnc::AttitudeController plain;
    const Vec3 unchanged = plain.integrate_error({0.1, 0.2, 0.3}, {0.0, 0.01, 0.0}, 1.0);
    CHECK(unchanged.y == 0.2);
}
