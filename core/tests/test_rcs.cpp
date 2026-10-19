// Individual RCS thrusters: allocation, modulation, vehicle integration.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/propulsion/rcs_allocator.hpp"
#include "starsim/propulsion/rcs_modulator.hpp"
#include "starsim/vehicle/flight_body.hpp"

using namespace starsim;
using math::Vec3;
using propulsion::RcsAllocator;
using propulsion::RcsThrusterGeometry;

namespace {

constexpr double pi = constants::pi;
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
constexpr double tick_s = 0.005;

class ZeroGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3&, time::Epoch) const override { return {}; }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

struct DeepSpace {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;  // zero above 1000 km
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};

// One stage of a vehicle alone (the others removed from the configuration), far from everything.
std::unique_ptr<vehicle::FlightBody> stage_alone(const std::string& vehicle_id, size_t stage) {
    auto cfg = config::load_vehicle(config_dir, vehicle_id);
    const auto keep = cfg.stages[stage];
    cfg.stages = {keep};
    return std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir),
                                                 vehicle::RigidBodyState{{1e10, 0.0, 0.0}, {}, {}, {}});
}

size_t rcs_tank_index(const vehicle::Stage& stage) {
    for (size_t i = 0; i < stage.tanks.size(); ++i) {
        if (stage.tanks[i].config.contents == config::TankContents::Rcs) return i;
    }
    FAIL("no RCS tank");
    return 0;
}

// A pod ring: at each azimuth one thruster each tangential +, tangential - and inward.
std::vector<RcsThrusterGeometry> ring(double station, double radius, double thrust, std::initializer_list<double> az_deg) {
    std::vector<RcsThrusterGeometry> out;
    for (double a_deg : az_deg) {
        const double a = a_deg * pi / 180.0;
        const Vec3 r{0.0, std::cos(a), std::sin(a)}, t{0.0, -std::sin(a), std::cos(a)};
        const Vec3 p = Vec3{station, 0.0, 0.0} + r * radius;
        out.push_back({p, t, thrust});
        out.push_back({p, t * -1.0, thrust});
        out.push_back({p, r * -1.0, thrust});
    }
    return out;
}

}  // namespace

TEST_CASE("RCS allocator: roll capability of a four-pod ring is four tangential thrusters", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    CHECK(a.max_torque_nm().x == Catch::Approx(4 * 3000.0 * 4.5).epsilon(0.02));
    // Pitch at 38 m: the two inward thrusters on one side plus one tangential per pod whose rolls cancel - six
    // thrusters at cos 45 deg each.
    CHECK(a.max_torque_nm().y == Catch::Approx(6 * 3000.0 * std::cos(pi / 4) * 38.0).epsilon(0.05));
}

TEST_CASE("RCS allocator: a pure torque demand gives that torque and little force", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const Vec3 want{20000.0, -50000.0, 30000.0};
    const auto d = a.allocate(want, {});
    for (double x : d) CHECK((x >= 0.0 && x <= 1.0));
    const Vec3 got = a.torque_nm(d);
    CHECK(norm(got - want) < 0.01 * norm(want));
}

TEST_CASE("RCS allocator: torque has priority over force when saturated", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const Vec3 roll{0.9 * a.max_torque_nm().x, 0.0, 0.0};
    const auto d = a.allocate(roll, Vec3{0.0, 1e5, 0.0});  // force far beyond reach
    CHECK(a.torque_nm(d).x == Catch::Approx(roll.x).epsilon(0.03));
}

TEST_CASE("RCS allocator: no thruster along an axis gives zero capability, not NaN", "[rcs]") {
    const RcsAllocator a(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0);
    const double fx = a.force_capability_n({1.0, 0.0, 0.0});
    CHECK(std::isfinite(fx));
    CHECK(fx < 1.0);
    const RcsAllocator empty({}, 0.0);
    CHECK(norm(empty.max_torque_nm()) == 0.0);
    CHECK(empty.allocate({1.0, 0.0, 0.0}, {}).empty());
    const std::vector<RcsThrusterGeometry> starved = ring(68.0, 4.5, 0.0, {45, 135, 225, 315});
    CHECK(norm(RcsAllocator(starved, 30.0).max_torque_nm()) == 0.0);
}

TEST_CASE("RCS allocator: lateral force without torque from a symmetric pair of rings", "[rcs]") {
    auto t = ring(68.0, 4.5, 3000.0, {45, 135, 225, 315});
    const auto aft = ring(8.0, 4.5, 3000.0, {45, 135, 225, 315});
    t.insert(t.end(), aft.begin(), aft.end());
    const RcsAllocator a(t, 38.0);
    std::vector<double> d;
    const double f = a.force_capability_n({0.0, 1.0, 0.0}, &d);
    CHECK(f > 3000.0);
    CHECK(norm(a.torque_nm(d)) < 0.02 * f * 30.0);
}

TEST_CASE("RCS allocator: pure translation capability is the largest force without torque or cross-push", "[rcs]") {
    // two +y thrusters 1 m fore and aft of the CG, 100 N and 50 N: only 50 N + 50 N balance about z
    const RcsAllocator a({{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 100.0}, {{-1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, 50.0}}, 0.0);
    std::vector<double> d;
    const double pure = a.pure_force_capability_n({0.0, 1.0, 0.0}, &d);
    CHECK(pure == Catch::Approx(100.0).epsilon(0.01));
    CHECK(norm(a.force_n(d) - Vec3{0.0, pure, 0.0}) < 0.01 * pure);
    CHECK(norm(a.torque_nm(d)) < 0.01 * 100.0 * 1.0);
    CHECK(a.pure_force_capability_n({1.0, 0.0, 0.0}) == 0.0);    // no thruster along x
}

TEST_CASE("RCS allocator: a balanced layout turns at full capability without a net force", "[rcs]") {
    // Rings fore and aft plus two forward (ullage) thrusters aft: pure couples on every axis. A saturated turn must not
    // borrow the forward thrusters (a push the attitude controller did not ask for).
    auto t = ring(46.0, 4.5, 1500.0, {0, 180});
    const auto aft = ring(6.0, 4.5, 1500.0, {0, 180});
    t.insert(t.end(), aft.begin(), aft.end());
    t.push_back({{6.0, 4.5, 0.0}, {1.0, 0.0, 0.0}, 1500.0});
    t.push_back({{6.0, -4.5, 0.0}, {1.0, 0.0, 0.0}, 1500.0});
    const RcsAllocator a(t, 20.0);
    CHECK(a.balanced());
    const Vec3 cap = a.max_torque_nm();
    for (const Vec3 want : {Vec3{cap.x, 0.0, 0.0}, Vec3{0.0, -cap.y, 0.0}, Vec3{0.0, 0.0, cap.z}}) {
        CAPTURE(want.x, want.y, want.z);
        const auto d = a.allocate(want, {}, nullptr, true);  // in vacuum
        CHECK(norm(a.force_n(d)) < 1.0);
        CHECK(norm(a.torque_nm(d) - want) < 0.01 * norm(want));
    }
    // Beyond the capability (all axes at once) the demand is cut and the force stays small.
    CHECK(norm(a.force_n(a.allocate(cap * 2.0, {}, nullptr, true))) < 0.01 * 1500.0);  // finite force weight: a small leak
    // A single ring has no pure pitch couple: it turns with a push (torque first).
    CHECK_FALSE(RcsAllocator(ring(68.0, 4.5, 3000.0, {45, 135, 225, 315}), 30.0).balanced());
}

TEST_CASE("RCS modulator: the mean on-time follows the duty, pulses are never shorter than the minimum", "[rcs]") {
    constexpr double dt = 0.005, min_on = 0.02;
    for (const double duty : {0.0, 0.03, 0.1, 0.37, 0.8, 1.0}) {
        CAPTURE(duty);
        propulsion::RcsModulator m(1, min_on);
        int on = 0, run = 0, shortest = 1 << 30;
        bool previous = false;
        const int ticks = 2000;  // 10 s
        for (int k = 0; k < ticks; ++k) {
            const bool f = m.update({duty}, dt)[0];
            on += f ? 1 : 0;
            if (f) ++run;
            if (!f && previous) {
                shortest = std::min(shortest, run);
                run = 0;
            }
            previous = f;
        }
        CHECK(on * dt == Catch::Approx(duty * ticks * dt).margin(min_on + dt));
        if (duty > 0.0 && duty < 1.0) CHECK(shortest >= static_cast<int>(std::lround(min_on / dt)));
        if (duty == 1.0) CHECK(on == ticks);
    }
}

TEST_CASE("RCS modulator: a duty that drops to zero leaves nothing owed", "[rcs]") {
    propulsion::RcsModulator m(1, 0.02);
    for (int k = 0; k < 3; ++k) m.update({0.5}, 0.005);  // owes 7.5 ms, not yet a pulse
    m.update({0.0}, 0.005);
    for (int k = 0; k < 2; ++k) CHECK_FALSE(m.update({0.3}, 0.005)[0]);  // starts from zero again
}

TEST_CASE("RCS thrusters are loaded for every stage", "[rcs][config]") {
    const auto ss = config::load_vehicle(config_dir, "starship_block3");
    const auto f9 = config::load_vehicle(config_dir, "falcon9_block5");
    CHECK(ss.stages[0].rcs.thrusters.size() == 12);
    CHECK(ss.stages[1].rcs.thrusters.size() == 26);
    CHECK(f9.stages[0].rcs.thrusters.size() == 8);
    CHECK(f9.stages[1].rcs.thrusters.size() == 8);
    // Pod expansion: Super Heavy pod 1 at 45 deg, radius 4.5, station 68 m; first thruster tangential +.
    const auto& t = ss.stages[0].rcs.thrusters[0];
    CHECK(t.position_m.x == Catch::Approx(68.0));
    CHECK(t.position_m.y == Catch::Approx(4.5 * std::cos(pi / 4)));
    CHECK(t.direction.y == Catch::Approx(-std::sin(pi / 4)));
    CHECK(f9.stages[0].rcs.propellant == "rcs_n2");
    CHECK(ss.stages[0].rcs.propellant == "main");
}

TEST_CASE("RCS tanks do not change the liftoff mass and stay out of the propellant gauge", "[rcs][config]") {
    const auto cfg = config::load_vehicle(config_dir, "falcon9_block5");
    const vehicle::Vehicle v(cfg, config_dir);
    CHECK(v.rcs_propellant_kg() == Catch::Approx(200.0 + 80.0));
    double main = 0.0;
    for (const auto& s : cfg.stages) {
        for (const auto& t : s.tanks) {
            if (t.contents != config::TankContents::Rcs) main += t.mass_kg;
        }
    }
    CHECK(v.propellant_kg() == Catch::Approx(main));
    // 25.6 t + 3.9 t dry before d1: the N2 was carved out of them.
    CHECK(cfg.stages[0].dry_mass_kg + 200.0 == Catch::Approx(25600.0));
    CHECK(cfg.stages[1].dry_mass_kg + 80.0 == Catch::Approx(3900.0));
}

TEST_CASE("A roll command spins the Falcon 9 second stage and draws its nitrogen", "[rcs][vehicle]") {
    // At a full roll command the tangential thrusters fire every tick: roll acceleration = capability / Ixx, N2 drawn
    // at sum(firing * F) / (Isp g0).
    DeepSpace space;
    auto body = stage_alone("falcon9_block5", 1);
    const auto& v = body->vehicle();
    const double capability = v.max_rcs_torque_nm().x;
    const double ixx = v.mass_properties().inertia_kg_m2.x;
    const size_t tank = rcs_tank_index(v.stages()[0]);
    const double n2_before = v.stages()[0].tanks[tank].mass_kg;
    const double isp_g0 = v.stages()[0].config.rcs.isp_s * constants::standard_gravity_mps2;
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {1.0, 0.0, 0.0};
    double expected_n2 = 0.0;
    const int ticks = 400;  // 2 s
    for (int k = 0; k < ticks; ++k) {
        body->step(space.env, t0 + k * tick_s, tick_s, roll);
        for (const auto& t : v.stages()[0].rcs) expected_n2 += t.output * t.thrust_n / isp_g0 * tick_s;
    }
    const Vec3 w = body->state().angular_velocity_body_radps;
    CHECK(capability > 0.0);
    CHECK(w.x == Catch::Approx(ticks * tick_s * capability / ixx).epsilon(0.03));
    CHECK(std::abs(w.y) < 0.01 * w.x);
    CHECK(std::abs(w.z) < 0.01 * w.x);
    CHECK(norm(body->state().velocity_gcrf_mps) < 0.01);
    CHECK(expected_n2 > 0.0);
    CHECK(n2_before - v.stages()[0].tanks[tank].mass_kg == Catch::Approx(expected_n2).epsilon(0.02));
}

TEST_CASE("A small roll command gives its mean torque in pulses", "[rcs][vehicle]") {
    DeepSpace space;
    auto body = stage_alone("falcon9_block5", 1);
    const auto& v = body->vehicle();
    const double capability = v.max_rcs_torque_nm().x;
    const double ixx = v.mass_properties().inertia_kg_m2.x;
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {0.13, 0.0, 0.0};
    const size_t n = v.stages()[0].rcs.size();
    std::vector<int> run(n, 0);
    int shortest = 1 << 30;
    bool on_off_only = true;
    const int ticks = 800;  // 4 s
    for (int k = 0; k < ticks; ++k) {
        body->step(space.env, t0 + k * tick_s, tick_s, roll);
        for (size_t i = 0; i < n; ++i) {
            const double out = v.stages()[0].rcs[i].output;
            on_off_only = on_off_only && (out == 0.0 || out == 1.0);
            if (out > 0.0) {
                ++run[i];
            } else if (run[i] > 0) {
                shortest = std::min(shortest, run[i]);
                run[i] = 0;
            }
        }
    }
    CHECK(on_off_only);
    CHECK(body->state().angular_velocity_body_radps.x ==
          Catch::Approx(ticks * tick_s * 0.13 * capability / ixx).epsilon(0.05));
    CHECK(shortest >= 4);  // 20 ms at 200 Hz
}

TEST_CASE("An empty RCS tank starves its thrusters", "[rcs][vehicle]") {
    DeepSpace space;
    auto body = stage_alone("falcon9_block5", 1);
    auto& stage = body->vehicle().stages()[0];
    stage.tanks[rcs_tank_index(stage)].mass_kg = 0.0;
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {1.0, 0.0, 0.0};
    for (int k = 0; k < 200; ++k) body->step(space.env, t0 + k * tick_s, tick_s, roll);
    CHECK(norm(body->vehicle().max_rcs_torque_nm()) == 0.0);
    CHECK(norm(body->state().angular_velocity_body_radps) == 0.0);
    CHECK(std::isfinite(body->state().position_gcrf_m.x));
}

TEST_CASE("Separation re-bases the thrusters and gives each part its own", "[rcs][vehicle]") {
    const auto cfg = config::load_vehicle(config_dir, "falcon9_block5");
    vehicle::Vehicle stack(cfg, config_dir);
    const double local = cfg.stages[1].rcs.thrusters[0].position_m.x;
    CHECK(stack.stages()[1].rcs[0].position_m.x == Catch::Approx(cfg.stages[0].length_m + local));
    const Vec3 stack_torque = stack.max_rcs_torque_nm();
    auto upper = stack.separate(1);
    CHECK(upper->stages()[0].rcs[0].position_m.x == Catch::Approx(local));
    CHECK(stack.stages()[0].rcs.size() == 8);
    CHECK(upper->stages()[0].rcs.size() == 8);
    CHECK(stack.max_rcs_torque_nm().x > 0.0);
    CHECK(upper->max_rcs_torque_nm().x > 0.0);
    CHECK(stack.max_rcs_torque_nm().y != Catch::Approx(stack_torque.y));  // rebuilt for the new CG and set
}

TEST_CASE("The RCS allocation follows the CG", "[rcs][vehicle]") {
    DeepSpace space;
    auto body = stage_alone("starship_block3", 0);
    auto& v = body->vehicle();
    const double full = v.max_rcs_torque_nm().y;
    for (auto& t : v.stages()[0].tanks) t.mass_kg *= 0.05;
    const double drained = v.max_rcs_torque_nm().y;
    CHECK(drained != Catch::Approx(full).epsilon(0.01));
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {1.0, 0.0, 0.0};
    body->step(space.env, t0, tick_s, roll);
    const Vec3 tau = v.rcs_torque_nm();
    CHECK(tau.x > 0.0);
    CHECK(std::hypot(tau.y, tau.z) < 0.01 * tau.x);
}

TEST_CASE("RCS capability of every stage (report)", "[rcs][vehicle]") {
    struct Case {
        const char* vehicle;
        size_t stage;
        int old_count;
        double old_thrust_n;
    };
    for (const Case c : {Case{"starship_block3", 0, 8, 3000.0}, Case{"starship_block3", 1, 6, 1500.0},
                         Case{"falcon9_block5", 0, 8, 450.0}, Case{"falcon9_block5", 1, 0, 0.0}}) {
        auto body = stage_alone(c.vehicle, c.stage);
        const auto& v = body->vehicle();
        const double force = 0.5 * c.old_count * c.old_thrust_n;
        const Vec3 old{force * 0.5 * v.diameter_m(), force * 0.5 * v.length_m(), force * 0.5 * v.length_m()};
        const Vec3 now = v.max_rcs_torque_nm();
        const auto forward = v.rcs_force_capability({1.0, 0.0, 0.0});
        std::printf("[rcs] %s/%s: torque roll/pitch/yaw %.0f / %.0f / %.0f kNm (pre-d1 %.0f / %.0f / %.0f), "
                    "+x force %.0f N (%.2f kg/s; pre-d1 %.0f N)\n",
                    c.vehicle, v.stages()[0].config.id.c_str(), now.x / 1e3, now.y / 1e3, now.z / 1e3, old.x / 1e3,
                    old.y / 1e3, old.z / 1e3, forward.force_n, forward.mass_flow_kgps, force);
        CHECK(now.x > 0.0);
        CHECK(now.y > 0.0);
        CHECK(now.z > 0.0);
    }
}

TEST_CASE("A coast segment starts with the thrusters off", "[rcs][vehicle]") {
    // Review finding: a coast after a tick that ended mid-pulse integrated the pulse's mass flow over the whole segment.
    DeepSpace space;
    auto body = stage_alone("falcon9_block5", 1);
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {1.0, 0.0, 0.0};
    body->step(space.env, t0, tick_s, roll);
    double firing = 0.0;
    for (const auto& t : body->vehicle().stages()[0].rcs) firing += t.output;
    REQUIRE(firing > 0.0);
    body->coast(space.env, t0 + tick_s, 100.0);
    for (const auto& t : body->vehicle().stages()[0].rcs) CHECK(t.output == 0.0);
    CHECK(norm(body->vehicle().rcs_force_n()) == 0.0);
    CHECK(norm(body->vehicle().rcs_torque_nm()) == 0.0);
}

TEST_CASE("The RCS duty is reported per thruster and empty when stopped", "[rcs][vehicle]") {
    DeepSpace space;
    auto body = stage_alone("falcon9_block5", 1);
    auto& v = body->vehicle();
    CHECK(v.rcs_duty().empty());  // nothing allocated yet
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {0.3, 0.0, 0.0};
    body->step(space.env, t0, tick_s, roll);
    REQUIRE(v.rcs_duty().size() == v.stages()[0].rcs.size());
    double sum = 0.0;
    for (const double d : v.rcs_duty()) {
        CHECK(d >= 0.0);
        CHECK(d <= 1.0);
        sum += d;
    }
    CHECK(sum > 0.0);
    v.stop_rcs();
    CHECK(v.rcs_duty().empty());
}
