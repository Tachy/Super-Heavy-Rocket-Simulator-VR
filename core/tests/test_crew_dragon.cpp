// Crew Dragon (d2 of docs/superpowers/archive/specs/2026-10-14-rcs-crew-dragon-design.md): vehicle data, payload stacking,
// thruster availability, orbital attitude modes, Draco translation burns.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
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

namespace {

// Flies an RCS burn of `dv` m/s prograde from the 200 km orbit (node 300 s ahead) and compares the final velocity with
// the same orbit coasted without the burn in an identical session: the difference is the burn.
struct BurnResult {
    Vec3 error_mps;
    double propellant_kg = 0.0;
    Vec3 axis_body;
    double misalignment_at_ignition_rad = -1.0;  // thrust axis vs burn direction when the translation starts
};

BurnResult fly_prograde_burn(DragonInOrbit& o, double dv, bool drop_forward_thrusters = false) {
    auto& s = o.session;
    const auto& b0 = s.body(0).state();
    const math::StateVector start{b0.position_gcrf_m, b0.velocity_gcrf_mps};
    const double node_s = 300.0;
    const auto at_node = orbit::propagate_two_body(start, o.gm, node_s);
    const Vec3 planned = math::normalized(at_node.velocity_mps) * dv;
    if (drop_forward_thrusters) {
        // The side pods' forward thrusters lost: the strongest axis is -x (the four Dracos under the nose cone). Such a
        // layout turns only with a push and a short lever, so the capsule starts with its nose against the burn: the hull
        // attitude for a -x burn (half a turn about body z of the burn frame) is then already held - a wrong anti-parallel
        // attitude would turn it, spend propellant and push it off the planned delta-v.
        auto cfg = config::load_vehicle(config_dir, "crew_dragon");
        std::erase_if(cfg.stages[1].rcs.thrusters, [](const config::RcsThrusterConfig& t) { return t.direction.x > 0.5; });
        auto state = s.body(0).state();
        const Vec3 x = -math::normalized(planned);
        const Vec3 y_now = state.attitude.rotate({0.0, 1.0, 0.0});
        const Vec3 y = math::normalized(y_now - x * dot(y_now, x));
        const Vec3 z = cross(x, y);
        math::Mat3 m;
        const Vec3 columns[3] = {x, y, z};
        for (int c = 0; c < 3; ++c) {
            m(0, c) = columns[c].x;
            m(1, c) = columns[c].y;
            m(2, c) = columns[c].z;
        }
        state.attitude = math::Quat::from_matrix(m);
        s.place_body(cfg, state, o.t0);
        s.set_nose_cone(0, true);
    }
    const double kg0 = s.body(0).vehicle().rcs_propellant_kg();
    REQUIRE(s.execute_burn({o.t0 + node_s, planned}));
    BurnResult r;
    r.axis_body = s.burn_configuration().thrust_axis_body;
    REQUIRE(s.burn_configuration().rcs);
    for (double t = 1.0; t < 1200.0 && s.autopilot().state == sim::AutopilotState::Burn; t += 1.0) {
        s.advance_to(o.t0 + t, 1000);
        if (r.misalignment_at_ignition_rad < 0.0 && s.autopilot().burn.ignite) {
            r.misalignment_at_ignition_rad =
                angle_between(s.body(0).state().attitude.rotate(r.axis_body), s.autopilot().burn.direction_gcrf);
        }
        if (std::getenv("STARSIM_VERBOSE") && static_cast<int>(t) % 10 == 0) {
            const auto& w = s.body(0).state().angular_velocity_body_radps;
            std::printf("t %4.0f kg %.2f ignite %d remaining %.3f w %.5f %.5f %.5f\n", t,
                        kg0 - s.body(0).vehicle().rcs_propellant_kg(), s.autopilot().burn.ignite ? 1 : 0,
                        s.autopilot().burn.remaining_dv_mps, w.x, w.y, w.z);
        }
    }
    REQUIRE(s.autopilot().state == sim::AutopilotState::Off);
    r.propellant_kg = kg0 - s.body(0).vehicle().rcs_propellant_kg();
    sim::FlightSession reference(*o.ephemeris, *o.earth, STARSIM_CONFIG_DIR);
    reference.place_in_orbit("crew_dragon", start, o.t0);
    reference.advance_to(s.epoch(), 1000000);
    r.error_mps = (s.body(0).state().velocity_gcrf_mps - reference.body(0).state().velocity_gcrf_mps) - planned;
    return r;
}

}  // namespace

TEST_CASE("Dragon flies 1 m/s and 10 m/s burns on its Dracos", "[dragon][burn][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    for (const double dv : {1.0, 10.0}) {
        CAPTURE(dv);
        DragonInOrbit o(ephemeris, earth);
        const auto r = fly_prograde_burn(o, dv);
        std::printf("[validation] Dragon %.0f m/s Draco burn: error %.4f m/s, %.2f kg, axis %.0f/%.0f/%.0f\n", dv,
                    norm(r.error_mps), r.propellant_kg, r.axis_body.x, r.axis_body.y, r.axis_body.z);
        CHECK(r.axis_body.x == Catch::Approx(1.0));  // the side pods' forward thrusters (+x)
        CHECK(norm(r.error_mps) < 0.01 + 0.01 * dv);  // [E] 1 % + the pulse quantum
        const double ideal_kg = 12500.0 * (1.0 - std::exp(-dv / (300.0 * constants::standard_gravity_mps2)));
        CHECK(r.propellant_kg > ideal_kg);
        CHECK(r.propellant_kg < 1.3 * ideal_kg + 0.5);  // turning and holding during the burn
    }
}

TEST_CASE("An RCS burn along -x turns the hull round and keeps the delta-v", "[dragon][burn]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);
    const auto r = fly_prograde_burn(o, 2.0, true);
    std::printf("[validation] Dragon 2 m/s burn on the nose Dracos (-x): error %.4f m/s, %.2f kg\n", norm(r.error_mps),
                r.propellant_kg);
    CHECK(r.axis_body.x == Catch::Approx(-1.0));
    CHECK(norm(r.error_mps) < 0.05);
    const double ideal_kg = 12500.0 * (1.0 - std::exp(-2.0 / (300.0 * constants::standard_gravity_mps2)));
    CHECK(r.propellant_kg < 1.5 * ideal_kg + 1.0);  // no half turn flown
}

TEST_CASE("The crew mission loads its payload vehicle", "[dragon][config]") {
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_crew_dragon");
    REQUIRE(m.payload);
    CHECK(m.payload->vehicle_id == "crew_dragon");
    CHECK(m.payload->separation_delay_s == Catch::Approx(30.0));
    CHECK(m.payload->separation_dv_mps == Catch::Approx(0.3));
    CHECK(m.payload->nose_cone_open_s == Catch::Approx(600.0));
    CHECK(m.payload_kg == 0.0);
    CHECK_FALSE(config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_rtls").payload);
}

TEST_CASE("A mission without a payload vehicle ends in orbit as before", "[dragon][sim]") {
    // Review focus 5: no payload separation without a payload. The second stage flies the upper-stage phase from a placed
    // state to cutoff and must end in state Orbit, not PayloadSeparation.
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const double gm = session.gm_earth_m3ps2();
    const orbit::KeplerianElements start{6378137.0 + 195e3, 0.0005, 51.6 * constants::deg_to_rad, 0.0, 0.0, 0.5};
    session.place_in_orbit("falcon9_block5", orbit::state_from_elements(start, gm), t_deep, 1);
    sim::engage_pad_ascent(session, {200e3, 200e3, 51.6 * constants::deg_to_rad, true});
    session.ignite();
    bool separation_state = false;
    for (double t = 1.0; t < 600.0 && session.autopilot().state != sim::AutopilotState::Orbit; t += 1.0) {
        session.advance_to(t_deep + t, 1000);
        separation_state = separation_state || session.autopilot().state == sim::AutopilotState::PayloadSeparation;
    }
    CHECK_FALSE(separation_state);
    CHECK(session.autopilot().state == sim::AutopilotState::Orbit);
    CHECK(session.body_count() == 1);
}

TEST_CASE("Falcon 9 lifts Crew Dragon into its insertion orbit and the capsule separates",
          "[dragon][mission][validation]") {
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 13, 10, 33, 0.0});
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_crew_dragon");
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
    double seco_s = -1.0, separation_s = -1.0;
    size_t stage2 = 0, capsule = 0;
    for (double t = 1.0; t < 1200.0; t += 1.0) {
        session.advance_to(t0 + t, 1000);
        const auto state = session.autopilot().state;
        if (seco_s < 0.0 && state == sim::AutopilotState::PayloadSeparation) seco_s = session.mission_time_s();
        if (separation_s < 0.0 && seco_s > 0.0 && state != sim::AutopilotState::PayloadSeparation) {
            separation_s = session.mission_time_s();
            capsule = session.controlled_index();
            stage2 = capsule - 1;
            break;
        }
    }
    REQUIRE(seco_s > 0.0);
    REQUIRE(separation_s > 0.0);
    CHECK(separation_s - seco_s == Catch::Approx(30.0).margin(1.0));
    CHECK(session.body(capsule).vehicle().stages().back().config.id == "capsule");
    CHECK(session.body(stage2).vehicle().stages().back().config.id == "second_stage");
    CHECK(session.attitude_mode().kind == gnc::AttitudeModeKind::Inertial);
    // Insertion orbit of the capsule.
    const auto& s = session.body(capsule).state();
    const auto el = orbit::elements_from_state({s.position_gcrf_m, s.velocity_gcrf_mps}, session.gm_earth_m3ps2());
    const double peri_km = (el.semi_major_axis_m * (1.0 - el.eccentricity) - 6378137.0) / 1000.0;
    const double apo_km = (el.semi_major_axis_m * (1.0 + el.eccentricity) - 6378137.0) / 1000.0;
    std::printf("[validation] crew mission: SECO T+%.0f s, separation T+%.0f s, orbit %.1f x %.1f km, i %.2f deg\n",
                seco_s, separation_s, peri_km, apo_km, el.inclination_rad / constants::deg_to_rad);
    CHECK(peri_km == Catch::Approx(200.0).margin(5.0));  // [E] osculating elements carry the J2 swing
    CHECK(apo_km == Catch::Approx(210.0).margin(5.0));
    CHECK(el.inclination_rad / constants::deg_to_rad == Catch::Approx(51.6).margin(0.2));
    // Clearance: the distance (between the CGs, ~12 m apart at the release) grows for 60 s - attitude pulses must not push
    // the capsule back - by about the spring push x 60 s (0.3 m/s: 18 m).
    const double d0 = norm(session.body(capsule).state().position_gcrf_m - session.body(stage2).state().position_gcrf_m);
    double last = d0;
    const double kg0 = session.body(capsule).vehicle().rcs_propellant_kg();
    const auto t_sep = session.epoch();
    bool growing = true;
    for (double t = 1.0; t <= 60.0; t += 1.0) {
        session.advance_to(t_sep + t, 1000);
        const double d =
            norm(session.body(capsule).state().position_gcrf_m - session.body(stage2).state().position_gcrf_m);
        growing = growing && d > last;
        last = d;
    }
    std::printf("[validation] capsule 60 s after the separation: %.1f m from the second stage (+%.1f m)\n", last,
                last - d0);
    CHECK(growing);
    CHECK(last - d0 > 15.0);
    CHECK(last - d0 < 25.0);  // no extra push from the hold
    CHECK_FALSE(session.body(stage2).vehicle().any_engine_producing_thrust());
    // The nose cone opens 600 s after the separation; the capsule holds its attitude on the Dracos meanwhile.
    CHECK_FALSE(session.body(capsule).vehicle().nose_cone_open());
    session.advance_to(t_sep + 601.0, 200000);
    CHECK(session.body(capsule).vehicle().nose_cone_open());
    const double used = kg0 - session.body(capsule).vehicle().rcs_propellant_kg();
    std::printf("[validation] capsule: %.2f kg of MMH/NTO in the first 10 min\n", used);
    CHECK(used < 5.0);
    // The first stage landed on LZ-40 meanwhile (RTLS as in falcon9_rtls).
    CHECK(session.landing_outcome(0) == sim::LandingOutcome::Landed);
}

TEST_CASE("A retrograde Draco burn waits for its half turn", "[dragon][burn]") {
    // Review (d2): the orbit controller slews at 1 deg/s, so a burn whose axis points away from the hold needs more than
    // the 120 s burn preparation: the turn must be done when the translation starts.
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    DragonInOrbit o(ephemeris, earth);  // nose along the velocity
    const auto r = fly_prograde_burn(o, -2.0);  // retrograde: +x (the side pods) turns half round
    const double ideal_kg = 12500.0 * (1.0 - std::exp(-2.0 / (300.0 * constants::standard_gravity_mps2)));
    std::printf("[validation] Dragon retrograde 2 m/s burn: misaligned %.1f deg at ignition, error %.4f m/s, %.2f kg\n",
                r.misalignment_at_ignition_rad / constants::deg_to_rad, norm(r.error_mps), r.propellant_kg);
    CHECK(r.axis_body.x == Catch::Approx(1.0));
    CHECK(r.misalignment_at_ignition_rad >= 0.0);
    CHECK(r.misalignment_at_ignition_rad < 5.0 * constants::deg_to_rad);
    CHECK(norm(r.error_mps) < 0.03);
    CHECK(r.propellant_kg < 1.3 * ideal_kg + 2.5);  // the half turn at 1 deg/s costs ~1.5 kg
}

TEST_CASE("Re-engaging the ascent autopilot after staging keeps a stacked payload on the upper stage",
          "[dragon][sim]") {
    // Review (d2): [second stage, trunk, capsule] has three stages but one launcher stage with engines: the autopilot must
    // resume as the upper stage (cutoff, payload release), not as a booster burning the stage dry.
    test::spice_session();
    static const spice::SpiceEphemeris ephemeris;
    static const auto earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter time;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR);
    const auto t0 = time.from_utc({2026, 10, 13, 10, 33, 0.0});
    const auto m = config::load_ascent_mission(STARSIM_CONFIG_DIR, "falcon9_crew_dragon");
    sim::start_ascent_mission(session, m, STARSIM_CONFIG_DIR, t0);
    for (double t = 1.0; t < 400.0 && session.autopilot().state != sim::AutopilotState::UpperStage; t += 1.0) {
        session.advance_to(t0 + t, 1000);
    }
    REQUIRE(session.autopilot().state == sim::AutopilotState::UpperStage);
    session.advance_to(session.epoch() + 10.0, 10000);
    session.disengage_autopilot();
    sim::engage_pad_ascent(session, {m.periapsis_height_m, m.apoapsis_height_m, m.inclination_rad, m.northerly});
    CHECK(session.autopilot().state == sim::AutopilotState::UpperStage);
}

TEST_CASE("A payload is stacked only while a launcher stage is below it", "[dragon][config]") {
    const auto stack = config::stack_payload(config::load_vehicle(config_dir, "falcon9_block5"),
                                             config::load_vehicle(config_dir, "crew_dragon"));
    vehicle::Vehicle v(stack, config_dir);
    CHECK(v.payload_stacked());
    auto upper = v.separate(1);  // second stage + Dragon
    CHECK(upper->payload_stacked());
    CHECK_FALSE(v.payload_stacked());  // the first stage alone
    auto dragon = upper->separate(1);
    CHECK_FALSE(dragon->payload_stacked());
    CHECK_FALSE(upper->payload_stacked());
    CHECK_FALSE(vehicle::Vehicle(config::load_vehicle(config_dir, "crew_dragon"), config_dir).payload_stacked());
}
