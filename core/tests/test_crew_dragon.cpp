// Crew Dragon (d2 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): vehicle data, payload stacking,
// thruster availability, orbital attitude modes, Draco translation burns.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

}  // namespace

TEST_CASE("Crew Dragon loads as capsule and trunk", "[dragon][config]") {
    const auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    REQUIRE(cfg.stages.size() == 2);
    CHECK(cfg.stages[0].id == "trunk");
    CHECK(cfg.stages[1].id == "capsule");
    const auto& capsule = cfg.stages[1];
    CHECK(capsule.rcs.thrusters.size() == 16);
    int behind_cone = 0;
    for (const auto& t : capsule.rcs.thrusters) {
        CHECK(t.thrust_n == Catch::Approx(400.0));
        behind_cone += t.behind_nose_cone ? 1 : 0;
    }
    CHECK(behind_cone == 4);
    CHECK(capsule.nose_cone);
    REQUIRE(capsule.engine_groups.size() == 1);
    CHECK(capsule.engine_groups[0].count == 8);
    CHECK(capsule.engine_groups[0].abort_only);
    // 12.5 t at launch [S], 2,562 kg MMH/NTO [S] in the RCS tank (out of the propellant gauge).
    const vehicle::Vehicle v(cfg, config_dir);
    CHECK(v.mass_properties().mass_kg == Catch::Approx(12500.0));
    CHECK(v.rcs_propellant_kg() == Catch::Approx(2562.0));
    CHECK(v.propellant_kg() == 0.0);
}

TEST_CASE("A payload vehicle stacks on top of its launcher", "[dragon][config]") {
    const auto f9 = config::load_vehicle(config_dir, "falcon9_block5");
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    const auto stack = config::stack_payload(f9, dragon);
    REQUIRE(stack.stages.size() == 4);
    CHECK(stack.stages[2].id == "trunk");
    CHECK(stack.stages[3].id == "capsule");
    CHECK_FALSE(stack.stages[1].payload);
    CHECK(stack.stages[2].payload);
    CHECK(stack.stages[3].payload);
    CHECK(stack.stages[1].payload_mass_kg == 0.0);  // the payload is the stages now, not a mass
    CHECK(stack.engines.contains("superdraco"));
    CHECK(stack.engines.contains("merlin1d"));
    auto bare = f9;
    bare.stages.back().payload_mass_kg = 0.0;
    const vehicle::Vehicle launcher(bare, config_dir), stacked(stack, config_dir);
    CHECK(stacked.mass_properties().mass_kg == Catch::Approx(launcher.mass_properties().mass_kg + 12500.0));
    // Stage ids must stay unique (engine groups are addressed "<stage>/<group>").
    CHECK_THROWS_AS(config::stack_payload(f9, f9), config::ConfigError);
}

namespace {

constexpr double tick_s = 0.005;
const time::Epoch t_deep = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);

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

std::unique_ptr<vehicle::FlightBody> deep_space_body(const config::VehicleConfig& cfg) {
    return std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir),
                                                 vehicle::RigidBodyState{{1e10, 0.0, 0.0}, {}, {}, {}});
}

double rcs_kg(const vehicle::Vehicle& v, const std::string& stage_id) {
    for (const auto& s : v.stages()) {
        if (s.config.id != stage_id) continue;
        double kg = 0.0;
        for (const auto& t : s.tanks) kg += t.config.contents == config::TankContents::Rcs ? t.mass_kg : 0.0;
        return kg;
    }
    return 0.0;
}

}  // namespace

TEST_CASE("A stacked payload's thrusters stay inhibited until it flies alone", "[dragon][rcs]") {
    DeepSpace space;
    const auto stack = config::stack_payload(config::load_vehicle(config_dir, "falcon9_block5"),
                                             config::load_vehicle(config_dir, "crew_dragon"));
    auto body = deep_space_body(stack);
    auto upper = body->separate(1);  // second stage + trunk + capsule, as after staging
    vehicle::VehicleCommand roll;
    roll.rcs_torque_fraction = {1.0, 0.0, 0.0};
    const double dragon_before = rcs_kg(upper->vehicle(), "capsule");
    for (int k = 0; k < 200; ++k) upper->step(space.env, t_deep + k * tick_s, tick_s, roll);
    CHECK(upper->state().angular_velocity_body_radps.x > 0.0);   // the second stage's nitrogen pods roll it
    CHECK(rcs_kg(upper->vehicle(), "capsule") == dragon_before);  // no Draco fired
    for (const auto& t : upper->vehicle().stages().back().rcs) CHECK(t.blocked);
    // Alone (trunk + capsule) its Dracos fire.
    auto dragon = upper->separate(1);
    for (const auto& t : dragon->vehicle().stages().back().rcs) CHECK(t.blocked == t.behind_nose_cone);
    for (int k = 0; k < 200; ++k) dragon->step(space.env, t_deep + k * tick_s, tick_s, roll);
    CHECK(rcs_kg(dragon->vehicle(), "capsule") < dragon_before);
}

TEST_CASE("The Dracos behind the nose cone fire only with the cone open", "[dragon][rcs]") {
    const auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    vehicle::Vehicle v(cfg, config_dir);
    REQUIRE(v.has_nose_cone());
    CHECK_FALSE(v.nose_cone_open());
    CHECK(v.rcs_force_capability({-1.0, 0.0, 0.0}).force_n == Catch::Approx(0.0).margin(1e-6));
    CHECK(v.rcs_force_capability({1.0, 0.0, 0.0}).force_n == Catch::Approx(1600.0).epsilon(0.01));
    v.set_nose_cone_open(true);
    CHECK(v.rcs_force_capability({-1.0, 0.0, 0.0}).force_n == Catch::Approx(1600.0).epsilon(0.01));
}

TEST_CASE("Dragon turns on pure couples: a full torque on any axis does not push it", "[dragon][rcs]") {
    // Balanced layout (pure couples on every axis): with rcs_keep_force a saturated turn leaves the capsule where it is.
    DeepSpace space;
    for (const Vec3 axis : {Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, -1.0}}) {
        CAPTURE(axis.x, axis.y, axis.z);
        auto body = deep_space_body(config::load_vehicle(config_dir, "crew_dragon"));
        body->vehicle().set_nose_cone_open(true);
        vehicle::VehicleCommand turn;
        turn.rcs_torque_fraction = axis;
        turn.rcs_keep_force = true;
        for (int k = 0; k < 400; ++k) body->step(space.env, t_deep + k * tick_s, tick_s, turn);
        const Vec3 w = body->state().angular_velocity_body_radps;
        CHECK(dot(w, axis) > 0.005);                           // it turns (> 0.3 deg/s after 2 s)
        CHECK(norm(body->state().velocity_gcrf_mps) < 2e-3);  // < 2 mm/s from leaks
    }
}

TEST_CASE("Abort-only engines are never lit by the session", "[dragon][sim]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const double gm = session.gm_earth_m3ps2();
    const orbit::KeplerianElements circular{6378137.0 + 200e3, 0.0, 51.6 * constants::deg_to_rad, 0.0, 0.0, 0.0};
    session.place_in_orbit("crew_dragon", orbit::state_from_elements(circular, gm), t_deep);
    session.ignite();
    session.advance_to(t_deep + 1.0);
    CHECK_FALSE(session.body(0).vehicle().any_engine_producing_thrust());
    CHECK(session.minimum_burn_dv_mps() == 0.0);  // no burn configuration on main engines
}

TEST_CASE("The LVLH frame: x along the velocity, z to nadir, rotating at the orbit rate", "[dragon][attitude]") {
    const double gm = 3.986004418e14;
    const orbit::KeplerianElements circular{6578137.0, 0.0, 0.9, 0.3, 0.0, 1.0};
    const auto s = orbit::state_from_elements(circular, gm);
    const auto q = gnc::lvlh_attitude_gcrf(s);
    CHECK(norm(q.rotate({1.0, 0.0, 0.0}) - math::normalized(s.velocity_mps)) < 1e-9);  // circular: v is horizontal
    CHECK(norm(q.rotate({0.0, 0.0, 1.0}) + math::normalized(s.position_m)) < 1e-12);
    const auto target = gnc::attitude_target(gnc::lvlh_hold(), s);
    const double n = std::sqrt(gm / std::pow(circular.semi_major_axis_m, 3));
    CHECK(norm(target.rate_gcrf_radps) == Catch::Approx(n).epsilon(1e-9));
    CHECK(dot(target.rate_gcrf_radps, cross(s.position_m, s.velocity_mps)) > 0.0);
    // "Trunk forward": the body turned 180 deg about z in LVLH points its nose against the velocity.
    const auto back =
        gnc::attitude_target(gnc::lvlh_hold(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi)), s);
    CHECK(dot(back.axis_gcrf, math::normalized(s.velocity_mps)) == Catch::Approx(-1.0));
}

namespace {

// Crew Dragon (trunk + capsule) alone on a 200 km circular orbit at 51.6 deg, nose cone open.
struct DragonInOrbit {
    const spice::SpiceEphemeris* ephemeris;
    const frames::BodyOrientation* earth;
    sim::FlightSession session;
    time::Epoch t0;
    double gm;
    explicit DragonInOrbit(const spice::SpiceEphemeris& eph, const frames::BodyOrientation& orientation)
        : ephemeris(&eph), earth(&orientation), session(eph, orientation, STARSIM_CONFIG_DIR), t0(t_deep),
          gm(session.gm_earth_m3ps2()) {
        const orbit::KeplerianElements circular{6378137.0 + 200e3, 0.0, 51.6 * constants::deg_to_rad, 0.0, 0.0, 0.0};
        session.place_in_orbit("crew_dragon", orbit::state_from_elements(circular, gm), t0);
        session.set_nose_cone(0, true);
    }
};

double angle_between(const Vec3& a, const Vec3& b) {
    return std::acos(std::clamp(dot(math::normalized(a), math::normalized(b)), -1.0, 1.0));
}

}  // namespace

TEST_CASE("Dragon captures an inertial attitude from a tumble and holds it on pulses", "[dragon][attitude]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);
    auto& s = o.session;
    const auto hold = s.body(0).state().attitude;
    s.set_attitude_mode(gnc::inertial_hold(hold));
    s.apply_angular_rate(0, {0.5 * constants::deg_to_rad, -1.0 * constants::deg_to_rad, 0.7 * constants::deg_to_rad});
    const double kg0 = s.body(0).vehicle().rcs_propellant_kg();
    double worst = 0.0, kg_after_capture = 0.0;
    for (double t = 1.0; t <= 1800.0; t += 1.0) {
        s.advance_to(o.t0 + t, 1000);
        if (t == 120.0) kg_after_capture = s.body(0).vehicle().rcs_propellant_kg();
        if (t > 120.0) {
            worst = std::max(worst, angle_between(s.body(0).state().attitude.rotate({1.0, 0.0, 0.0}),
                                                  hold.rotate({1.0, 0.0, 0.0})));
        }
    }
    const double used = kg0 - s.body(0).vehicle().rcs_propellant_kg();
    const double hold_kg_per_h = (kg_after_capture - s.body(0).vehicle().rcs_propellant_kg()) / (1680.0 / 3600.0);
    std::printf("[validation] Dragon inertial hold 30 min after a 1.3 deg/s tumble: worst %.2f deg after 2 min, %.2f kg "
                "(capture %.2f kg, hold %.2f kg/h)\n",
                worst / constants::deg_to_rad, used, kg0 - kg_after_capture, hold_kg_per_h);
    CHECK(worst < 1.0 * constants::deg_to_rad);
    CHECK(used > 0.0);
    CHECK(used < 5.0);  // [E] ruling 4: ~1 kg/h hold + the capture
}

TEST_CASE("Dragon holds LVLH with its trunk forward for a quarter orbit", "[dragon][attitude]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);
    auto& s = o.session;
    const auto mode = gnc::lvlh_hold(math::Quat::from_axis_angle({0.0, 0.0, 1.0}, constants::pi));
    s.set_attitude_mode(mode);
    const double kg0 = s.body(0).vehicle().rcs_propellant_kg();
    double worst = 0.0, kg_after_turn = 0.0;
    for (double t = 1.0; t <= 1330.0; t += 1.0) {
        s.advance_to(o.t0 + t, 1000);
        const auto& b = s.body(0).state();
        if (t == 300.0) kg_after_turn = s.body(0).vehicle().rcs_propellant_kg();
        if (std::getenv("STARSIM_VERBOSE") && static_cast<int>(t) % 20 == 0) {
            const auto target = gnc::attitude_target(mode, {b.position_gcrf_m, b.velocity_gcrf_mps});
            const Vec3 w = b.angular_velocity_body_radps;
            const Vec3 wt = b.attitude.inverse_rotate(target.rate_gcrf_radps);
            std::printf("t %4.0f err %.3f deg w %.5f %.5f %.5f target %.5f %.5f %.5f kg %.3f\n", t,
                        angle_between(b.attitude.rotate({1.0, 0.0, 0.0}), target.axis_gcrf) / constants::deg_to_rad, w.x,
                        w.y, w.z, wt.x, wt.y, wt.z, kg0 - s.body(0).vehicle().rcs_propellant_kg());
        }
        if (t > 300.0) {  // after the 180 deg turn
            const auto target = gnc::attitude_target(mode, {b.position_gcrf_m, b.velocity_gcrf_mps});
            worst = std::max(worst, angle_between(b.attitude.rotate({1.0, 0.0, 0.0}), target.axis_gcrf));
        }
    }
    std::printf("[validation] Dragon LVLH trunk-forward hold: worst %.2f deg, turn %.2f kg, hold %.2f kg/h\n",
                worst / constants::deg_to_rad, kg0 - kg_after_turn,
                (kg_after_turn - s.body(0).vehicle().rcs_propellant_kg()) / (1030.0 / 3600.0));
    CHECK(worst < 1.5 * constants::deg_to_rad);
    CHECK(kg0 - s.body(0).vehicle().rcs_propellant_kg() < 4.0);  // [E] 180 deg turn at 1 deg/s + ~3 kg/h hold
}

TEST_CASE("Dragon points its nose at the Sun", "[dragon][attitude]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);
    auto& s = o.session;
    gnc::AttitudeMode sun;
    sun.kind = gnc::AttitudeModeKind::Sun;
    s.set_attitude_mode(sun);
    s.advance_to(o.t0 + 600.0, 200000);
    const auto& b = s.body(0).state();
    const Vec3 to_sun = ephemeris.state_gcrf(ephemeris::Body::Sun, s.epoch()).position_m - b.position_gcrf_m;
    CHECK(angle_between(b.attitude.rotate({1.0, 0.0, 0.0}), to_sun) < 1.0 * constants::deg_to_rad);
}
