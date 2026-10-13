#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/flight_body.hpp"

using namespace starsim;
using math::Vec3;
using vehicle::FlightBody;
using vehicle::RigidBodyState;
using vehicle::Vehicle;
using vehicle::VehicleCommand;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

class ZeroGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3&, time::Epoch) const override { return {}; }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

// Rotating Earth about z with the sidereal rate (no precession) for pad tests.
class RotatingEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch epoch) const override {
        const double w = 7.2921150e-5;
        const double a = w * (epoch - t0);
        math::Mat3 r = math::Mat3::identity(), dr;
        r(0, 0) = std::cos(a); r(0, 1) = -std::sin(a); r(1, 0) = std::sin(a); r(1, 1) = std::cos(a);
        dr(0, 0) = -w * std::sin(a); dr(0, 1) = -w * std::cos(a); dr(1, 0) = w * std::cos(a); dr(1, 1) = -w * std::sin(a);
        return {r, dr};
    }
};

struct DeepSpace {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;  // zero above 1000 km
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};

// Ship only, instantaneous engine start/stop, far from everything.
std::unique_ptr<FlightBody> ship_in_deep_space(const RigidBodyState& state) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    for (auto& [id, engine] : cfg.engines) {
        engine.startup_s = 0.0;
        engine.shutdown_s = 0.0;
    }
    return std::make_unique<FlightBody>(std::make_unique<Vehicle>(cfg, config_dir), state);
}

RigidBodyState far_away() { return {{1e10, 0.0, 0.0}, {}, {}, {}}; }

VehicleCommand ship_full_thrust() {
    VehicleCommand c;
    c.groups = {{"ship/sea_level_1", true, 1.0}, {"ship/sea_level_2", true, 1.0}, {"ship/sea_level_3", true, 1.0}, {"ship/vacuum", true, 1.0}};
    return c;
}

}  // namespace

TEST_CASE("Vacuum burn reproduces the rocket equation", "[vehicle][validation]") {
    DeepSpace space;
    auto ship = ship_in_deep_space(far_away());
    const double m0 = ship->vehicle().mass_properties().mass_kg;
    const double dt = 0.005;
    for (int i = 0; i < 20000; ++i) {  // 100 s
        ship->step(space.env, t0 + i * dt, dt, ship_full_thrust());
    }
    const double m1 = ship->vehicle().mass_properties().mass_kg;
    const auto& stage = ship->vehicle().stages().front();
    double thrust = 0.0, mdot = 0.0;
    for (const auto& e : stage.engines) {
        thrust += e.performance->thrust_n(1.0, 0.0);
        mdot += e.performance->mass_flow_kgps(1.0);
    }
    const double expected = thrust / mdot * std::log(m0 / m1);
    const double dv = norm(ship->state().velocity_gcrf_mps);
    std::printf("[validation] vacuum burn 100 s: dv %.6f m/s, Tsiolkovsky %.6f m/s, burned %.1f t\n", dv, expected,
                (m0 - m1) / 1000.0);
    CHECK(m0 - m1 == Catch::Approx(mdot * 100.0).epsilon(1e-9));
    CHECK(dv == Catch::Approx(expected).epsilon(1e-9));
}

TEST_CASE("Torque-free rotation conserves angular momentum", "[vehicle]") {
    DeepSpace space;
    RigidBodyState state = far_away();
    state.angular_velocity_body_radps = {0.3, 0.02, -0.01};
    auto ship = ship_in_deep_space(state);
    auto momentum = [&] {
        const auto inertia = ship->vehicle().mass_properties().inertia_kg_m2;
        const Vec3& w = ship->state().angular_velocity_body_radps;
        return ship->state().attitude.rotate({inertia.x * w.x, inertia.y * w.y, inertia.z * w.z});
    };
    const Vec3 h0 = momentum();
    for (int i = 0; i < 12000; ++i) ship->step(space.env, t0 + i * 0.005, 0.005, {});
    CHECK(norm(momentum() - h0) / norm(h0) < 1e-8);
}

TEST_CASE("Thrust vector control produces pitch and roll torques", "[vehicle]") {
    DeepSpace space;
    auto ship = ship_in_deep_space(far_away());
    VehicleCommand command = ship_full_thrust();
    command.gimbal_pitch_rad = 0.05;
    command.gimbal_roll_rad = 0.05;
    for (int i = 0; i < 400; ++i) ship->step(space.env, t0 + i * 0.005, 0.005, command);
    const Vec3 w = ship->state().angular_velocity_body_radps;
    // Engines below the CG deflected towards +z push the tail to +z: nose turns towards -z (+y rotation).
    CHECK(w.y > 0.0);
    CHECK(std::abs(w.x) > 1e-4);  // roll from tangential deflection of the off-axis sea-level engines
}

TEST_CASE("Clamped vehicle rides on the rotating Earth", "[vehicle]") {
    environment::Us76Atmosphere atmosphere;
    ZeroGravity gravity;
    RotatingEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    auto body = FlightBody::on_pad(std::make_unique<Vehicle>(cfg, config_dir), 25.99686 * constants::deg_to_rad,
                                   -97.15394 * constants::deg_to_rad, 10.0, 0.0, earth, t0);
    const Vec3 start_fixed = transpose(earth.fixed_to_gcrf(t0).rotation) * body->state().position_gcrf_m;
    for (int i = 0; i < 2000; ++i) body->step(env, t0 + i * 0.005, 0.005, {});
    const auto end = earth.fixed_to_gcrf(t0 + 10.0);
    CHECK(norm(transpose(end.rotation) * body->state().position_gcrf_m - start_fixed) < 1e-6);
    CHECK(norm(body->state().velocity_gcrf_mps) == Catch::Approx(7.2921150e-5 * std::hypot(
        start_fixed.x, start_fixed.y)).epsilon(1e-9));
    CHECK_FALSE(body->impacted());
}

TEST_CASE("Stage separation conserves momentum", "[vehicle]") {
    DeepSpace space;
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    RigidBodyState state = far_away();
    state.velocity_gcrf_mps = {100.0, 2000.0, -50.0};
    state.angular_velocity_body_radps = {0.0, 0.01, 0.0};
    FlightBody stack(std::make_unique<Vehicle>(cfg, config_dir), state);
    const double m = stack.vehicle().mass_properties().mass_kg;
    const Vec3 p0 = m * stack.state().velocity_gcrf_mps;
    auto ship = stack.separate(1, 2.0);
    const double mb = stack.vehicle().mass_properties().mass_kg;
    const double ms = ship->vehicle().mass_properties().mass_kg;
    CHECK(mb + ms == Catch::Approx(m));
    CHECK(norm(mb * stack.state().velocity_gcrf_mps + ms * ship->state().velocity_gcrf_mps - p0) / norm(p0) < 1e-12);
    // Ship ahead of the booster along the body axis.
    const Vec3 axis = state.attitude.rotate({1.0, 0.0, 0.0});
    CHECK(dot(ship->state().position_gcrf_m - stack.state().position_gcrf_m, axis) > 30.0);
    CHECK(ship->vehicle().stages().front().base_station_m == 0.0);
}

TEST_CASE("Engines obey relight rules and flame out when dry", "[vehicle]") {
    DeepSpace space;
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    for (auto& [id, engine] : cfg.engines) engine.startup_s = engine.shutdown_s = 0.0;
    for (auto& group : cfg.stages[0].engine_groups) {
        if (group.id == "outer") group.relight = false;  // test the rule (Raptor 3 itself relights)
    }
    cfg.stages.pop_back();
    cfg.stages[0].tanks[0].mass_kg = 20000.0;  // little LOX
    FlightBody booster(std::make_unique<Vehicle>(cfg, config_dir), far_away());
    VehicleCommand run;
    run.groups = {{"booster/outer", true, 1.0}};
    booster.step(space.env, t0, 0.005, run);
    CHECK(booster.telemetry(space.env, t0).engines_running == 20);
    booster.step(space.env, t0, 0.005, {});
    booster.step(space.env, t0, 0.005, run);
    CHECK(booster.telemetry(space.env, t0).engines_running == 0);  // no relight

    VehicleCommand center;
    center.groups = {{"booster/center", true, 1.0}};
    for (int i = 0; i < 4000 && booster.vehicle().stages()[0].propellant_kg(config::TankContents::Oxidizer) > 0.0; ++i) {
        booster.step(space.env, t0 + i * 0.005, 0.005, center);
    }
    CHECK(booster.telemetry(space.env, t0).engines_running == 0);  // flame-out when LOX is gone
}

namespace {

class CentralGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3& r, time::Epoch) const override {
        const double d = norm(r);
        return r * (-3.986004418e14 / (d * d * d));
    }
};

struct PadWorld {
    CentralGravity gravity;
    environment::Us76Atmosphere atmosphere;
    RotatingEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};

std::unique_ptr<FlightBody> stack_on_pad(const PadWorld& world) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    return FlightBody::on_pad(std::make_unique<Vehicle>(cfg, config_dir), 25.99686 * constants::deg_to_rad,
                              -97.15394 * constants::deg_to_rad, 10.0, 0.0, world.earth, t0);
}

VehicleCommand booster_throttle(double throttle) {
    VehicleCommand c;
    c.groups = {{"booster/center", true, throttle}, {"booster/divert", true, throttle}, {"booster/middle", true, throttle},
                {"booster/outer", true, throttle}};
    return c;
}

}  // namespace

TEST_CASE("Released with too little thrust, the vehicle stays on the pad; with enough it lifts off", "[vehicle]") {
    PadWorld world;
    auto body = stack_on_pad(world);
    double t = 0.0;
    auto run = [&](double seconds, const VehicleCommand& c) {
        for (int i = 0; i < static_cast<int>(seconds / 0.005); ++i, t += 0.005) body->step(world.env, t0 + t, 0.005, c);
    };
    run(4.0, booster_throttle(0.4));  // spool up while clamped
    body->release_clamps();
    run(5.0, booster_throttle(0.4));  // T/W ~ 0.6
    const auto resting = body->telemetry(world.env, t0 + t);
    CHECK_FALSE(body->impacted());
    CHECK(std::abs(resting.vertical_speed_mps) < 0.01);
    CHECK(resting.altitude_m > 10.0);  // CG above the 10 m pad

    run(10.0, booster_throttle(1.0));
    const auto climbing = body->telemetry(world.env, t0 + t);
    CHECK_FALSE(body->impacted());
    CHECK(climbing.altitude_m > resting.altitude_m + 30.0);
    CHECK(climbing.vertical_speed_mps > 5.0);
}

TEST_CASE("A hard touchdown is an impact", "[vehicle]") {
    PadWorld world;
    auto body = stack_on_pad(world);
    body->release_clamps();
    // Lift the vehicle 200 m and let it fall with engines off.
    auto state = body->state();
    state.position_gcrf_m += normalized(state.position_gcrf_m) * 200.0;
    body->set_state(state);
    double t = 0.0;
    for (int i = 0; i < 4000 && !body->impacted(); ++i, t += 0.005) body->step(world.env, t0 + t, 0.005, {});
    CHECK(body->impacted());
}

TEST_CASE("An external wrench acts for one step", "[vehicle]") {
    DeepSpace space;
    auto body = ship_in_deep_space(far_away());
    const double m = body->vehicle().mass_properties().mass_kg;
    const double iyy = body->vehicle().mass_properties().inertia_kg_m2.y;
    body->apply_external_wrench({1.0e6, 0.0, 0.0}, {0.0, 1.0e6, 0.0});
    body->step(space.env, t0, 0.1, {});
    CHECK(body->state().velocity_gcrf_mps.x == Catch::Approx(1.0e6 / m * 0.1).epsilon(1e-6));
    CHECK(body->state().angular_velocity_body_radps.y == Catch::Approx(1.0e6 / iyy * 0.1).epsilon(1e-6));
    const Vec3 v = body->state().velocity_gcrf_mps;
    body->step(space.env, t0 + 0.1, 0.1, {});
    CHECK(body->state().velocity_gcrf_mps.x == Catch::Approx(v.x).epsilon(1e-12));  // cleared after use
}

TEST_CASE("A caught body co-rotates with the Earth", "[vehicle]") {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;
    RotatingEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
    RigidBodyState s;
    s.position_gcrf_m = {6.45e6, 0.0, 0.0};
    s.velocity_gcrf_mps = {0.0, 3.0, -2.0};
    auto body = ship_in_deep_space(s);
    body->hold_fixed_to_earth(earth, t0);
    CHECK(body->caught());
    for (int i = 0; i < 100; ++i) body->step(env, t0 + i, 1.0, {});
    const auto r = earth.fixed_to_gcrf(t0 + 100.0).rotation * Vec3{6.45e6, 0.0, 0.0};
    CHECK(norm(body->state().position_gcrf_m - r) < 1e-6);
    body->mark_impacted();
    CHECK(body->impacted());
}

TEST_CASE("The Block 3 booster has a divert pair on the middle ring", "[vehicle]") {
    // Landing sequence 13 -> 5 -> 3 (V3 baseline): the five divert engines are the three centre engines and a pair
    // of the middle ring. Engine groups may place their engines at explicit ring angles.
    const Vehicle v(config::load_vehicle(config_dir, "starship_block3"), config_dir);
    const auto& engines = v.stages().front().engines;
    REQUIRE(engines.size() == 33);
    int center = 0, divert = 0, middle = 0, outer = 0;
    for (const auto& e : engines) {
        center += e.group_id == "booster/center";
        divert += e.group_id == "booster/divert";
        middle += e.group_id == "booster/middle";
        outer += e.group_id == "booster/outer";
    }
    CHECK(center == 3);
    CHECK(divert == 2);
    CHECK(middle == 8);
    CHECK(outer == 20);
    for (size_t i = 0; i < engines.size(); ++i) {
        for (size_t j = i + 1; j < engines.size(); ++j) {
            CHECK(norm(engines[i].gimbal_point_m - engines[j].gimbal_point_m) > 0.5);  // no two engines in one place
        }
    }
    for (const auto& e : engines) {
        if (e.group_id != "booster/divert") continue;
        CHECK(std::hypot(e.gimbal_point_m.y, e.gimbal_point_m.z) == Catch::Approx(2.6));
        CHECK(std::abs(std::sin(e.ring_angle_rad)) < 1e-12);  // at 0 and 180 deg: a symmetric pair
    }
}
