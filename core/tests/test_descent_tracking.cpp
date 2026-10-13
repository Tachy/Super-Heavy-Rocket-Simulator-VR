#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <memory>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/descent_tracking.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Quat;
using math::Vec3;

namespace {

// Two phases of 2 s, two nodes each (3 nodes): straight down, the rate about body z rising 0 -> 0.2 rad/s.
DescentPlan synthetic_plan() {
    DescentPlan p;
    p.nodes_per_phase = 2;
    p.converged = true;
    p.reference.duration_s = {2.0, 2.0};
    p.reference.x = {make_state(1.0e5, {100.0, 0.0, 0.0}, {-20.0, 0.0, 0.0}, Quat{}, {0.0, 0.0, 0.0}),
                     make_state(0.99e5, {70.0, 0.0, 0.0}, {-10.0, 0.0, 0.0}, Quat{}, {0.0, 0.0, 0.2}),
                     make_state(0.98e5, {60.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, Quat{}, {0.0, 0.0, 0.2})};
    p.reference.u = {{2.0e6, 0.0, 0.0}, {1.5e6, 0.0, 0.0}, {1.0e6, 0.0, 0.0}};
    return p;
}

DescentModel flat_model() {
    DescentModel m;
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

}  // namespace

TEST_CASE("Plan samples interpolate between nodes", "[gnc][tracking]") {
    const auto ref = sample_plan(synthetic_plan(), flat_model(), 1.0, 1.0);
    CHECK(ref.phase == 0);
    CHECK_FALSE(ref.finished);
    CHECK(std::abs(ref.position_m.x - 85.0) < 1e-9);
    CHECK(std::abs(ref.velocity_mps.x + 15.0) < 1e-9);
    CHECK(std::abs(ref.thrust_acceleration_mps2.x - 1.75e6 / 0.995e5) < 1e-6);
    CHECK(std::abs(ref.axis.x - 1.0) < 1e-12);
    CHECK(std::abs(ref.rate_plan_radps.z - 0.1) < 1e-9);
    CHECK(std::abs(ref.angular_acceleration_plan_radps2.z - 0.1) < 1e-6);
    CHECK(sample_plan(synthetic_plan(), flat_model(), 3.0, 1.0).phase == 1);
}

TEST_CASE("After the plan's end the reference keeps sinking slowly against gravity", "[gnc][tracking]") {
    const auto ref = sample_plan(synthetic_plan(), flat_model(), 6.0, 1.0);
    CHECK(ref.finished);
    CHECK(ref.phase == 1);
    CHECK(std::abs(ref.velocity_mps.x + 1.0) < 1e-12);
    CHECK(std::abs(ref.position_m.x - 58.0) < 1e-9);  // 60 m - 1 m/s x 2 s
    CHECK(std::abs(ref.thrust_acceleration_mps2.x - 9.8) < 1e-12);
    CHECK(norm(ref.rate_plan_radps) == 0.0);
}

TEST_CASE("Planned rates are given in the plan frame, whatever the planned roll", "[gnc][tracking]") {
    auto plan = synthetic_plan();
    const Quat rolled = Quat::from_axis_angle({1.0, 0.0, 0.0}, 0.5 * 3.14159265358979);  // body z -> plan -y
    for (auto& x : plan.reference.x) x = make_state(x[0], state_position(x), state_velocity(x), rolled, state_rate(x));
    const auto ref = sample_plan(plan, flat_model(), 1.0, 1.0);
    CHECK(std::abs(ref.rate_plan_radps.y + 0.1) < 1e-9);
    CHECK(std::abs(ref.rate_plan_radps.z) < 1e-9);
    CHECK(std::abs(ref.angular_acceleration_plan_radps2.y + 0.1) < 1e-6);
}

TEST_CASE("Rate feed-forward removes the damping torque for a planned rotation", "[gnc][tracking]") {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.resize(1);
    vehicle::RigidBodyState s;
    s.position_gcrf_m = {6.4e6, 0.0, 0.0};
    s.angular_velocity_body_radps = {0.0, 0.02, 0.0};  // engines off: RCS
    const vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR), s);
    const AttitudeController controller;
    const Vec3 axis = s.attitude.rotate({1.0, 0.0, 0.0});
    const auto plain = controller.steer(body, 0.0, axis);
    AttitudeController::Feedforward ff;
    ff.rate_body_radps = {0.0, 0.02, 0.0};
    const auto fed = controller.steer(body, 0.0, axis, {}, &ff);
    CHECK(plain.rcs_torque_fraction.y < -1e-3);          // damps the rate
    CHECK(std::abs(fed.rcs_torque_fraction.y) < 1e-9);   // the rate is planned: nothing to damp
}

TEST_CASE("Tracking turns lateral errors into small, bounded tilts even at low thrust", "[gnc][tracking]") {
    // Final engines near minimum thrust: thrust acceleration ~8 m/s^2. A 50 m lateral error must not command a
    // large tilt (the slow final set would saturate its gimbals and push the booster sideways).
    auto plan = synthetic_plan();
    for (auto& u : plan.reference.u) u = {8.0e5, 0.0, 0.0};  // 8 m/s^2 at 1e5 kg
    const auto ref = sample_plan(plan, flat_model(), 1.0, 1.0);
    const TrackingGains gains;
    const auto cmd = tracking_command(ref, ref.position_m + Vec3{0.0, 50.0, 0.0}, ref.velocity_mps, gains);
    const double tilt = std::acos(std::min(1.0, dot(cmd.axis, ref.axis)));
    CHECK(tilt <= gains.max_tilt_correction_rad + 1e-9);
    CHECK(cmd.axis.y < 0.0);  // towards the plan
    // A vertical error changes the thrust, not the axis.
    const auto low = tracking_command(ref, ref.position_m - Vec3{5.0, 0.0, 0.0}, ref.velocity_mps, gains);
    CHECK(std::acos(std::min(1.0, dot(low.axis, ref.axis))) < 1e-9);
    CHECK(low.thrust_acceleration_mps2 > norm(ref.thrust_acceleration_mps2));
    const auto exact = tracking_command(ref, ref.position_m, ref.velocity_mps, gains);
    CHECK(std::abs(exact.thrust_acceleration_mps2 - norm(ref.thrust_acceleration_mps2)) < 1e-9);
}
