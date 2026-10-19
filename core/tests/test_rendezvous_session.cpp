// ISS i2: the flight session's rendezvous target, relative navigation and translation command in the target port frame.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/station_start.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "proximity_test_support.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {
using proximity_test::config_dir;
constexpr double deg = constants::deg_to_rad;

// A Dragon (cone open, LVLH hold) `range_m` in front of the ISS port IDA-2, co-moving with it; the ISS as body 1.
struct DragonAtIss : proximity_test::DragonNearIss {
    explicit DragonAtIss(double range_m) : DragonNearIss(proximity_test::in_front_of_ida2(range_m)) {}
};

// The full rotation from the LVLH hold attitude (body axes = LVLH axes), roll about the nose included.
double lvlh_attitude_error_rad(const vehicle::FlightBody& b) {
    const auto& s = b.state();
    const math::Quat error = frames::lvlh_to_inertial({s.position_gcrf_m, s.velocity_gcrf_mps}).conjugate() * s.attitude;
    return 2.0 * std::acos(std::clamp(std::abs(error.w), 0.0, 1.0));
}
}  // namespace

TEST_CASE("Rendezvous target: relative state of the Dragon port against IDA-2", "[iss][rendezvous][session]") {
    test::spice_session();
    DragonAtIss d(50.0);
    auto& s = d.session;
    CHECK_FALSE(s.relative_port_state());
    CHECK_THROWS_AS(s.set_translation_command({0.0, 1.0, 0.0}), std::logic_error);  // no target yet
    CHECK_THROWS_AS(s.set_rendezvous_target(1, "idss", "ida1"), std::invalid_argument);  // unknown port
    CHECK_THROWS_AS(s.set_rendezvous_target(0, "idss", "idss"), std::out_of_range);     // the chaser itself
    CHECK_THROWS_AS(s.set_rendezvous_target(5, "idss", "ida2"), std::out_of_range);
    s.set_rendezvous_target(1, "idss", "ida2");
    const auto r = s.relative_port_state();
    REQUIRE(r);
    // the Dragon's CG was placed 50 m out; its port sits on its nose (body +x along the LVLH velocity, like IDA-2)
    CHECK(r->range_m == Catch::Approx(50.0).margin(6.0));
    CHECK(norm(r->velocity_mps) < 0.01);  // co-moving: no omega x r phantom from the rotating port frame
    CHECK_FALSE(r->chaser_port_covered);
}

TEST_CASE("Placing a vehicle clears the rendezvous target", "[iss][rendezvous][session]") {
    test::spice_session();
    DragonAtIss d(50.0);
    auto& s = d.session;
    s.set_rendezvous_target(1, "idss", "ida2");
    s.set_translation_command({0.0, 0.5, 0.0});
    const auto dragon_state = s.body(0).state();
    s.place_in_orbit(config::load_vehicle(config_dir, "crew_dragon"),
                     {dragon_state.position_gcrf_m, dragon_state.velocity_gcrf_mps}, s.epoch());
    CHECK_FALSE(s.relative_port_state());
    CHECK(norm(s.translation_command()) == 0.0);
    CHECK_THROWS_AS(s.set_translation_command({0.0, 1.0, 0.0}), std::logic_error);
}

TEST_CASE("Dragon translates along each target port axis under an LVLH hold", "[iss][rendezvous][session][validation]") {
    test::spice_session();
    DragonAtIss d(50.0);
    auto& s = d.session;
    s.set_rendezvous_target(1, "idss", "ida2");
    constexpr double burn_s = 20.0, settle_s = 5.0;
    // the hold first rolls the placed Dragon into the LVLH attitude (~1 deg/s): the campaign starts once it is there
    double t = 0.0;
    do {
        t += settle_s;
        s.advance_to(d.t0 + t, 100000);
    } while (lvlh_attitude_error_rad(s.body(0)) > 0.5 * deg && t < 600.0);
    std::printf("[validation] LVLH hold settled after %.0f s (%.2f deg)\n", t, lvlh_attitude_error_rad(s.body(0)) / deg);
    REQUIRE(lvlh_attitude_error_rad(s.body(0)) < 0.5 * deg);
    for (const Vec3 u : {Vec3{1.0, 0.0, 0.0}, Vec3{-1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, -1.0, 0.0},
                         Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, -1.0}}) {
        CAPTURE(u.x, u.y, u.z);
        const auto& dragon = s.body(0);
        const auto before = *s.relative_port_state();
        const auto target = vehicle::port_kinematics(s.body(1), "ida2");
        // the translation command is a fraction of the pure capability (no torque, no cross push: Task 1 ruling)
        const auto capability = dragon.vehicle().rcs_pure_force_capability(
            dragon.state().attitude.inverse_rotate(target.axes_gcrf * u));
        const double mass = dragon.vehicle().mass_properties().mass_kg;
        const double kg0 = dragon.vehicle().rcs_propellant_kg();
        s.set_translation_command(u);
        double worst_attitude = 0.0;
        for (double k = 1.0; k <= burn_s; k += 1.0) {
            s.advance_to(d.t0 + t + k, 100000);
            worst_attitude = std::max(worst_attitude, lvlh_attitude_error_rad(s.body(0)));
        }
        s.set_translation_command({});
        t += burn_s;
        const auto after = *s.relative_port_state();
        const Vec3 dv = after.velocity_mps - before.velocity_mps;
        const double expected_mps = capability.force_n / mass * burn_s;
        const double used_kg = kg0 - s.body(0).vehicle().rcs_propellant_kg();
        std::printf("[validation] port-frame translation (%+.0f,%+.0f,%+.0f) for %.0f s: dv %.3f m/s (expected %.3f), "
                    "cross %.3f m/s, %.2f kg (expected %.2f), worst attitude %.2f deg\n",
                    u.x, u.y, u.z, burn_s, dot(dv, u), expected_mps, norm(dv - dot(dv, u) * u), used_kg,
                    capability.mass_flow_kgps * burn_s, worst_attitude / deg);
        CHECK(dot(dv, u) == Catch::Approx(expected_mps).epsilon(0.15));  // [E] pulses, CW drift over 20 s
        CHECK(norm(dv - dot(dv, u) * u) < 0.1 * expected_mps + 0.01);
        CHECK(used_kg == Catch::Approx(capability.mass_flow_kgps * burn_s).epsilon(0.25));  // + the hold's pulses
        CHECK(worst_attitude < 1.0 * deg);  // the hold keeps steering while the Dracos translate
        t += settle_s;
        s.advance_to(d.t0 + t, 100000);
    }
}

TEST_CASE("A translation command blocks coasting only where it is flown", "[iss][rendezvous][session]") {
    test::spice_session();
    DragonAtIss d(50.0);
    auto& s = d.session;
    s.set_rendezvous_target(1, "idss", "ida2");
    s.set_attitude_mode({});  // free drift: no hold ticks the session
    s.set_translation_command({0.0, 0.0, 0.0});
    CHECK(s.advance_to(d.t0 + 600.0, 100) - d.t0 == Catch::Approx(600.0));  // nothing commanded: coasts
    s.set_translation_command({0.0, 0.5, 0.0});
    const time::Epoch flown = s.advance_to(d.t0 + 1200.0, 100);  // rate mode flies it: ticked
    CHECK(flown - (d.t0 + 600.0) < 1.0);
    s.set_control_mode(sim::ControlMode::DirectGimbal);  // the command is not applied here: it must not stall the warp
    CHECK(s.advance_to(flown + 600.0, 100) - flown == Catch::Approx(600.0));
}

TEST_CASE("Separating the controlled body drops the rendezvous target", "[iss][rendezvous][session]") {
    test::spice_session();
    DragonAtIss d(50.0);
    auto& s = d.session;
    s.set_rendezvous_target(1, "idss", "ida2");
    s.set_translation_command({0.0, 0.5, 0.0});
    s.stage();  // the trunk goes: the capsule becomes the controlled body (index 2)
    REQUIRE(s.controlled_index() == 2);
    CHECK_FALSE(s.relative_port_state());
    CHECK(norm(s.translation_command()) == 0.0);
}
