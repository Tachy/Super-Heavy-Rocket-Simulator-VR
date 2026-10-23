#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <utility>

#include "return_scenario.hpp"
#include "starsim/aero/slender_body_aero.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/gnc/descent_model.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::gnc;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using math::Quat;
using math::Vec3;

namespace {

// Simple model: one set of 10 MN (min 4 MN), Isp 330 s, lever 30 m, no air, no Earth rotation.
DescentModel simple_model() {
    DescentModel m;
    for (auto& set : m.engine_sets) set = {1.0e7, 4.0e6, 1.0 / (330.0 * 9.80665), 15.0 * 3.14159265358979 / 180.0};
    m.gimbal_lever_m = 30.0;
    m.inertia_kg_m2 = {1.0e7, 2.0e8, 2.0e8};
    m.reference_area_m2 = 63.6;
    m.cd_by_mach = {1.0, 1.0};
    m.density_kg_m3 = {0.0, 0.0};
    m.speed_of_sound_mps = {340.0, 340.0};
    m.gravity_mps2 = {-9.8, 0.0, 0.0};
    return m;
}

DescentState upright(double mass, const Vec3& r, const Vec3& v) { return make_state(mass, r, v, Quat{1.0, 0.0, 0.0, 0.0}, {}); }

}  // namespace

TEST_CASE("Planning frame maps site east-north-up to up-east-north", "[gnc][descent]") {
    const Vec3 enu{1.0, 2.0, 3.0};
    CHECK(site_to_plan(enu) == Vec3{3.0, 1.0, 2.0});
    CHECK(plan_to_site(site_to_plan(enu)) == enu);
}

TEST_CASE("Planning model: thrust, gravity, mass flow and gimbal torque", "[gnc][descent]") {
    const auto model = simple_model();
    const auto x = upright(5.0e5, {1000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0});
    const auto d = model.derivative(x, {8.0e6, 0.0, 0.0}, 0);
    CHECK_THAT(d[descent_index::mass], WithinRel(-8.0e6 / (330.0 * 9.80665), 1e-12));
    CHECK_THAT(d[descent_index::position], WithinAbs(-100.0, 1e-12));
    CHECK_THAT(d[descent_index::velocity], WithinRel(8.0e6 / 5.0e5 - 9.8, 1e-12));
    // Thrust tilted towards body +z (gimbal "pitch"): torque about +y = lever * u_z.
    const auto t = model.derivative(x, {8.0e6, 0.0, 1.0e5}, 0);
    CHECK_THAT(t[descent_index::rate + 1], WithinRel(30.0 * 1.0e5 / 2.0e8, 1e-12));
    CHECK_THAT(t[descent_index::rate + 2], WithinAbs(0.0, 1e-15));
}

TEST_CASE("Planning model: drag opposes the motion through the air", "[gnc][descent]") {
    auto model = simple_model();
    model.density_kg_m3 = {1.2, 1.2};
    const auto x = upright(5.0e5, {1000.0, 0.0, 0.0}, {-100.0, 0.0, 0.0});
    const auto d = model.derivative(x, {0.0, 0.0, 0.0}, 0);
    // a = -9.8 + 0.5 rho v^2 A cd / m (upwards for a falling body).
    CHECK_THAT(d[descent_index::velocity], WithinRel(-9.8 + 0.5 * 1.2 * 1.0e4 * 63.6 / 5.0e5, 1e-9));
}

TEST_CASE("Planning model Jacobians predict small changes", "[gnc][descent]") {
    auto model = simple_model();
    model.density_kg_m3 = {1.2, 0.8};
    model.earth_rate_radps = {3.2e-5, 0.0, 6.5e-5};
    model.earth_center_m = {-6.37e6, 0.0, 0.0};
    const Quat q = Quat::from_axis_angle({0.0, 1.0, 1.0}, 0.1);
    const auto x = make_state(5.0e5, {800.0, 20.0, -30.0}, {-120.0, 5.0, 3.0}, q, {0.01, -0.02, 0.03});
    const Vec3 u{8.0e6, 2.0e5, -1.0e5};
    Mat14x14 a;
    Mat14x3 b;
    model.jacobians(x, u, 0, a, b);
    DescentState dx;
    // Small enough that second-order terms (e.g. the bilinear q-omega product) stay below 1e-3 of the change.
    for (int i = 0; i < 14; ++i) dx[i] = 1e-6 * (1.0 + std::abs(x[i])) * ((i % 3) - 1.0);
    const Vec3 du{10.0, -20.0, 5.0};
    const auto f0 = model.derivative(x, u, 0);
    const auto f1 = model.derivative(x + dx, u + du, 0);
    for (int i = 0; i < 14; ++i) {
        double predicted = 0.0;
        for (int j = 0; j < 14; ++j) predicted += a[static_cast<size_t>(i * 14 + j)] * dx[j];
        predicted += b[static_cast<size_t>(i * 3)] * du.x + b[static_cast<size_t>(i * 3 + 1)] * du.y + b[static_cast<size_t>(i * 3 + 2)] * du.z;
        const double actual = f1[i] - f0[i];
        CHECK(std::abs(predicted - actual) <= 1e-3 * std::abs(actual) + 1e-9);
    }
}

namespace {

// A descending stage at `height_m` over a sea-level pad, `speed_mps` tail first along the vertical, its body axis
// tilted `alpha_rad` from the vertical towards plan +y (east): the planning model next to the flight model's hull aero.
struct HullCase {
    DescentModel model;
    DescentState state;
    aero::AeroResult flight;  // aero::SlenderBodyAero::compute at the same state
    double cg_station_m = 0.0;
    Vec3 inertia_kg_m2;
};

HullCase hull_case(const config::VehicleConfig& cfg, double height_m, double speed_mps, double alpha_rad) {
    static const environment::Us76Atmosphere atmosphere;
    const vehicle::Vehicle vehicle(cfg, STARSIM_CONFIG_DIR);
    DescentModelInputs in;
    in.vehicle = &vehicle;
    in.atmosphere = &atmosphere;
    in.gravity_mps2 = {-9.8, 0.0, 0.0};
    HullCase c;
    c.model = make_descent_model(in);
    const auto mass = vehicle.mass_properties();
    c.cg_station_m = mass.cg_station_m;
    c.inertia_kg_m2 = mass.inertia_kg_m2;
    const Quat q = Quat::from_axis_angle({0.0, 0.0, -1.0}, alpha_rad);  // body x (up) towards plan +y
    const Vec3 v{-speed_mps, 0.0, 0.0};
    c.state = make_state(mass.mass_kg, {height_m, 0.0, 0.0}, v, q, {});
    const auto air = atmosphere.at(height_m);
    c.flight = vehicle.aero().compute(q.inverse_rotate(v), air.density_kg_m3, air.speed_of_sound_mps, false);
    return c;
}

// The Block 3 ship flying alone with its main tanks scaled to `propellant_kg` (RCS supplies full).
config::VehicleConfig ship_alone(double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    double total = 0.0;
    for (const auto& tank : cfg.stages[0].tanks) total += tank.contents == config::TankContents::Rcs ? 0.0 : tank.mass_kg;
    for (auto& tank : cfg.stages[0].tanks) {
        if (tank.contents != config::TankContents::Rcs) tank.mass_kg *= propellant_kg / total;
    }
    return cfg;
}

// Acceleration and angular acceleration the normal force adds: the derivative with it minus the one without.
std::pair<Vec3, Vec3> normal_force_effect(DescentModel model, const DescentState& x) {
    model.crossflow_enabled = true;
    const auto with = model.derivative(x, {}, 0);
    model.crossflow_enabled = false;
    const auto without = model.derivative(x, {}, 0);
    const auto d = with - without;
    return {{d[descent_index::velocity], d[descent_index::velocity + 1], d[descent_index::velocity + 2]},
            {d[descent_index::rate], d[descent_index::rate + 1], d[descent_index::rate + 2]}};
}

}  // namespace

TEST_CASE("cross-flow normal force matches the flight model", "[gnc][descent]") {
    // Falcon 9 first stage, 1.2 km, 250 m/s tail first, 10 deg angle of attack. A flat top: the flight model has no
    // potential term here, its normal force is the cross-flow term alone (generic eta * C_dc). Same density, Mach and
    // coefficient (the air data sit on a 50 m table point; the generic coefficient is constant): exact to rounding,
    // far inside the brief's 5 %.
    const auto c = hull_case(test::booster_config("falcon9_block5", 12000.0), 1200.0, 250.0, 10.0 * constants::deg_to_rad);
    REQUIRE(!c.model.potential_term);
    REQUIRE(!c.model.crossflow_cd_by_mach.empty());
    REQUIRE(c.model.crossflow_cd_by_mach.front() == aero::generic_crossflow_efficiency * aero::generic_crossflow_drag_coefficient);
    CHECK_THAT(c.flight.angle_of_attack_rad, WithinAbs(constants::pi - 10.0 * constants::deg_to_rad, 1e-12));
    const auto [accel, angular] = normal_force_effect(c.model, c.state);
    const Quat q = state_attitude(c.state);
    const Vec3 flight_normal_body{0.0, c.flight.force_body_n.y, c.flight.force_body_n.z};
    const Vec3 expected = q.rotate(flight_normal_body) / c.state[descent_index::mass];
    REQUIRE(norm(expected) > 1.0);  // m/s^2: the effect the planner lacked
    CHECK(norm(accel - expected) < 1e-6 * norm(expected));
    // Perpendicular to the body axis.
    CHECK(std::abs(dot(accel, q.rotate({1.0, 0.0, 0.0}))) < 1e-9 * norm(accel));
}

TEST_CASE("no cross-flow at zero angle of attack", "[gnc][descent]") {
    const auto c = hull_case(test::booster_config("falcon9_block5", 12000.0), 1200.0, 250.0, 0.0);
    const auto [accel, angular] = normal_force_effect(c.model, c.state);
    CHECK(norm(accel) < 1e-9);
    CHECK(norm(angular) < 1e-9);
}

TEST_CASE("the measured aerodynamics scale the normal force", "[gnc][descent]") {
    // The return guidance's online drag estimate (normal_force_scale) scales force and moment alike; the moment the
    // landing LQR cancels (hull_normal_force) is the one in the derivative.
    auto c = hull_case(test::booster_config("falcon9_block5", 12000.0), 1200.0, 250.0, 10.0 * constants::deg_to_rad);
    const auto [accel, angular] = normal_force_effect(c.model, c.state);
    const auto hull = c.model.hull_normal_force(c.state);
    CHECK_THAT(angular.z, WithinRel(hull.moment_body_nm.z / c.inertia_kg_m2.z, 1e-9));
    c.model.normal_force_scale = 0.9;
    const auto [accel_scaled, angular_scaled] = normal_force_effect(c.model, c.state);
    CHECK_THAT(norm(accel_scaled), WithinRel(0.9 * norm(accel), 1e-9));
    CHECK_THAT(angular_scaled.z, WithinRel(0.9 * angular.z, 1e-9));
}

TEST_CASE("cross-flow moment sign", "[gnc][descent]") {
    const auto c = hull_case(test::booster_config("falcon9_block5", 12000.0), 1200.0, 250.0, 10.0 * constants::deg_to_rad);
    const auto [accel, angular] = normal_force_effect(c.model, c.state);
    const Vec3 moment = cross(Vec3{c.flight.normal_force_station_m - c.cg_station_m, 0.0, 0.0},
                              Vec3{0.0, c.flight.force_body_n.y, c.flight.force_body_n.z});
    REQUIRE(std::abs(moment.z) > 0.0);
    CHECK(angular.z * moment.z > 0.0);
    CHECK_THAT(angular.z, WithinRel(moment.z / c.inertia_kg_m2.z, 1e-6));
    CHECK(std::abs(angular.x) < 1e-12);
    CHECK(std::abs(angular.y) < 1e-12);
}

TEST_CASE("ship normal force with the nose term matches the flight model", "[gnc][descent]") {
    // The ship has an ogive nose and an identified cross-flow curve. At 10 deg tail first the potential term
    // |sin 2a| cos(a/2) = 0.030 stands against the cross-flow's ~1.0 x 7.4 x sin^2 a = 0.22 (~14 % of the force), but
    // it acts at the nose, far beyond the cross-flow's mid-length arm: material for the moment, so the model carries it.
    const auto c = hull_case(ship_alone(30000.0), 1000.0, 150.0, 10.0 * constants::deg_to_rad);
    REQUIRE(c.model.potential_term);
    const auto [accel, angular] = normal_force_effect(c.model, c.state);
    const Quat q = state_attitude(c.state);
    const Vec3 normal_body{0.0, c.flight.force_body_n.y, c.flight.force_body_n.z};
    const Vec3 expected = q.rotate(normal_body) / c.state[descent_index::mass];
    // The cross-flow curve sampled every 0.05 in Mach (linear in between, as the curve is between its points) and the
    // air data on a 50 m table point: exact to rounding.
    CHECK(norm(accel - expected) < 1e-6 * norm(expected));
    const Vec3 moment = cross(Vec3{c.flight.normal_force_station_m - c.cg_station_m, 0.0, 0.0}, normal_body);
    CHECK_THAT(angular.z, WithinRel(moment.z / c.inertia_kg_m2.z, 1e-6));
}

TEST_CASE("Planning model Jacobians follow the normal force", "[gnc][descent]") {
    const auto c = hull_case(ship_alone(30000.0), 1000.0, 150.0, 8.0 * constants::deg_to_rad);
    const Quat q = state_attitude(c.state) * Quat::from_axis_angle({0.0, 1.0, 0.0}, 0.05);
    const auto x = make_state(c.state[descent_index::mass], {1000.0, 20.0, -30.0}, {-150.0, 5.0, 3.0}, q, {0.01, -0.02, 0.03});
    const Vec3 u{0.0, 0.0, 0.0};
    Mat14x14 a;
    Mat14x3 b;
    c.model.jacobians(x, u, 0, a, b);
    DescentState dx;
    for (int i = 0; i < 14; ++i) dx[i] = 1e-6 * (1.0 + std::abs(x[i])) * ((i % 3) - 1.0);
    const auto f0 = c.model.derivative(x, u, 0);
    const auto f1 = c.model.derivative(x + dx, u, 0);
    for (int i = 0; i < 14; ++i) {
        double predicted = 0.0;
        for (int j = 0; j < 14; ++j) predicted += a[static_cast<size_t>(i * 14 + j)] * dx[j];
        const double actual = f1[i] - f0[i];
        CHECK(std::abs(predicted - actual) <= 1e-3 * std::abs(actual) + 1e-9);
    }
}
