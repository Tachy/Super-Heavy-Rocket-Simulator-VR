// Unit cases for the booster return's predictions (gnc/booster_return_prediction.cpp, booster_return_burns.cpp): the
// boostback delta-v of the MECO plan, the propellant the plan sums up (rocket equation, flip, RCS gas, entry burn) and
// the one-dimensional landing-burn prediction. Every expected value is a hand solution (closed form or a construction
// whose answer is known by design) from the vehicle's config numbers, never a number read from a simulation run.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"
#include "return_scenario.hpp"

using namespace starsim;
using math::Vec3;

namespace {

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

// Uniform field towards -x (the landing hand case falls along x).
class UniformGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3&, time::Epoch) const override { return {-constants::standard_gravity_mps2, 0.0, 0.0}; }
};

class Vacuum final : public environment::Atmosphere {
public:
    environment::AtmosphereState at(double) const override { return environment::vacuum_state(); }
};

const gnc::LandingSite site_on_equator{ephemeris::Body::Earth, 0.0, 0.0, 0.0, {}, {}, {}};

// The engines of the booster's group `group` (config id without the stage prefix) from the config numbers.
struct GroupFigures {
    int count = 0;
    double vacuum_thrust_n = 0.0;      // one engine, full throttle
    double mass_flow_kgps = 0.0;       // one engine, full throttle: thrust_ref / (isp_ref g0)
    double throttle_min = 0.0;
};
GroupFigures figures(const config::VehicleConfig& cfg, const std::string& group) {
    for (const auto& g : cfg.stages.front().engine_groups) {
        if (g.id != group) continue;
        const auto& e = cfg.engines.at(g.engine_id);
        return {g.count, e.thrust_ref_n + e.reference_pressure_pa * e.nozzle_exit_area_m2,
                e.thrust_ref_n / (e.isp_ref_s * constants::standard_gravity_mps2), e.throttle_min};
    }
    FAIL("no engine group " << group);
    return {};
}

// Boostback hand case: after the flip (flip_duration_s) the stage is on state `s_f`; the site is where the two-body arc
// of s_f arrives at the end of flight_time_s. The stack at MECO is s_f flown back by the flip, with the velocity
// after the flip changed by `delta_mps` - the Lambert arc then has to undo exactly that change.
struct BoostbackCase {
    math::StateVector meco;
    gnc::LandingSite site;
    gnc::BoosterReturnParameters parameters;
};
BoostbackCase boostback_case(const Vec3& delta_mps, double flip_s) {
    BoostbackCase c;
    c.parameters.flip_duration_s = flip_s;
    c.parameters.flight_time_s = 230.0;
    c.parameters.boostback_loss_fraction = 0.1;
    const math::StateVector s_true{{6.45e6, 0.0, 0.0}, {300.0, 1500.0, 200.0}};  // on the arc to the site
    const auto arrival = orbit::propagate_two_body(s_true, constants::earth_gm_m3ps2, c.parameters.flight_time_s - flip_s);
    const auto geo = frames::fixed_to_geodetic(arrival.position_m);  // identity Earth: GCRF = ITRF
    c.site = {ephemeris::Body::Earth, geo.latitude_rad, geo.longitude_rad, geo.height_m, {}, {}, {}};
    math::StateVector s_f = s_true;
    s_f.velocity_mps += delta_mps;
    c.meco = orbit::propagate_two_body(s_f, constants::earth_gm_m3ps2, -flip_s);
    return c;
}

}  // namespace

// Construction: the arc that reaches the site needs exactly the velocity s_true has; the stage has `delta` more, so the
// boostback has to remove |delta| (plus the planned loss fraction). Tolerance 0.01 m/s: the Lambert bisection ends at
// 1e-12 in its universal variable and the two ~1.5 km/s velocities cancel to about 1e-13 of themselves; the 10 mm/s margin
// covers the position round trip through geodetic coordinates (mm).
TEST_CASE("Boostback delta-v is the velocity change onto the arc to the site, plus the loss fraction",
          "[booster_return][gnc]") {
    const FixedEarth earth;
    const time::Epoch epoch = time::Epoch::from_tdb_seconds_past_j2000(0.0);
    const Vec3 delta{40.0, -25.0, 30.0};  // |delta| = sqrt(1600 + 625 + 900) = sqrt(3125) = 55.9017 m/s
    for (const double flip_s : {0.0, 10.0}) {
        CAPTURE(flip_s);
        const auto c = boostback_case(delta, flip_s);
        const auto dv = gnc::BoosterReturnGuidance::boostback_delta_v_mps(c.meco, earth, epoch, c.site, c.parameters);
        REQUIRE(dv.has_value());
        CHECK(*dv == Catch::Approx(1.1 * std::sqrt(3125.0)).margin(0.01));
    }
    // the stage already on the arc: nothing to burn
    const auto on_arc = boostback_case({}, 10.0);
    const auto none = gnc::BoosterReturnGuidance::boostback_delta_v_mps(on_arc.meco, earth, epoch, on_arc.site, on_arc.parameters);
    REQUIRE(none.has_value());
    CHECK(*none < 0.01);
}

TEST_CASE("Without a boostback there is no boostback delta-v", "[booster_return][gnc]") {
    const FixedEarth earth;
    auto c = boostback_case({10.0, 0.0, 0.0}, 10.0);
    c.parameters.boostback = false;
    CHECK(!gnc::BoosterReturnGuidance::boostback_delta_v_mps(c.meco, earth, time::Epoch::from_tdb_seconds_past_j2000(0.0),
                                                              c.site, c.parameters));
}

// The plan's propellant against the rocket equation. Hand check by running it forward: the stage starts the boostback
// with Q (the plan, less the flip), its mass (dry + Q) falls by exp(-dv/c) and what is left of the propellant is the
// landing propellant. Relative tolerance 1e-9: closed forms in double precision.
TEST_CASE("MECO propellant: boostback by the rocket equation, flip at minimum throttle", "[booster_return][gnc]") {
    const FixedEarth earth;
    const time::Epoch epoch = time::Epoch::from_tdb_seconds_past_j2000(0.0);
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const vehicle::Vehicle stack(cfg, STARSIM_CONFIG_DIR);
    const double dry_kg = stack.stages().front().config.dry_mass_kg + stack.stages().front().extra_mass_kg;
    const double landing_kg = 90.0e3;

    auto c = boostback_case({60.0, 0.0, 0.0}, 10.0);
    c.parameters.rcs_propellant_kg = 0.0;
    c.parameters.entry_altitude_m = 0.0;
    c.parameters.flip_groups = {"booster/center"};
    c.parameters.boostback_groups = {"booster/center", "booster/divert", "booster/middle"};
    const double dv = 1.1 * 60.0;  // by construction (previous case)

    // exhaust velocity of the boostback set: all its engines are Raptors of one kind, c = F_vac / mdot
    const auto center = figures(cfg, "center");
    const double exhaust_mps = center.vacuum_thrust_n / center.mass_flow_kgps;
    const double e = std::exp(-dv / exhaust_mps);
    const double flip_kg = c.parameters.flip_duration_s * center.count * center.throttle_min * center.mass_flow_kgps;

    const double plan_kg =
        gnc::BoosterReturnGuidance::meco_propellant_kg(stack, c.meco, earth, epoch, c.site, c.parameters, landing_kg);
    const double boostback_start_kg = plan_kg - flip_kg;  // engines off at the plan: no run-down
    const double left_kg = (dry_kg + boostback_start_kg) * e - dry_kg;
    CHECK(left_kg == Catch::Approx(landing_kg).epsilon(1e-5));  // 1e-5 of 90 t = 0.9 kg = the effect of 8 mm/s on dv ((dry + Q)/c ~ 115 kg per m/s)
    CHECK(flip_kg > 0.0);

    // the flip on the RCS (Falcon 9: no engine group) costs the main tanks nothing
    auto rcs = c.parameters;
    rcs.flip_groups.clear();
    const double rcs_plan_kg = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, c.meco, earth, epoch, c.site, rcs, landing_kg);
    CHECK(plan_kg - rcs_plan_kg == Catch::Approx(flip_kg).epsilon(1e-9));
}

TEST_CASE("MECO propellant: RCS gas and the entry burn are added by the rocket equation", "[booster_return][gnc]") {
    const FixedEarth earth;
    const time::Epoch epoch = time::Epoch::from_tdb_seconds_past_j2000(0.0);
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const vehicle::Vehicle stack(cfg, STARSIM_CONFIG_DIR);
    const double dry_kg = stack.stages().front().config.dry_mass_kg + stack.stages().front().extra_mass_kg;
    const double landing_kg = 90.0e3;
    const auto c = boostback_case({}, 10.0);

    gnc::BoosterReturnParameters p;
    p.boostback = false;  // the run-down and the entry burn only, the site does not matter
    const double base_kg = gnc::BoosterReturnGuidance::meco_propellant_kg(stack, c.meco, earth, epoch, c.site, p, landing_kg);
    CHECK(base_kg == Catch::Approx(landing_kg).epsilon(1e-12));  // engines off: nothing runs down

    p.rcs_propellant_kg = 4.0e3;  // without a boostback and entry burn the gas is added one to one
    CHECK(gnc::BoosterReturnGuidance::meco_propellant_kg(stack, c.meco, earth, epoch, c.site, p, landing_kg) ==
          Catch::Approx(landing_kg + 4.0e3).epsilon(1e-12));

    p.rcs_propellant_kg = 0.0;
    p.entry_altitude_m = 60.0e3;
    p.entry_delta_v_mps = 125.0;
    p.entry_groups = {"booster/center"};
    const auto center = figures(cfg, "center");
    const double exhaust_mps = center.vacuum_thrust_n / center.mass_flow_kgps;
    const double expected_kg = (landing_kg + dry_kg) * std::exp(125.0 / exhaust_mps) - dry_kg;
    CHECK(gnc::BoosterReturnGuidance::meco_propellant_kg(stack, c.meco, earth, epoch, c.site, p, landing_kg) ==
          Catch::Approx(expected_kg).epsilon(1e-9));
}

// Landing burn straight down in vacuum with uniform gravity and constant thrust (13 Raptors at the design throttle, no
// start-up, no drag). Closed form with the mass flow: v(t) = v0 - c ln(m0 / m(t)) + g t, m = m0 - mdot t, and
// the way x(t) = v0 t - c I(t) + g t^2 / 2 with I(t) = [(1 - k t) ln(1 - k t) + k t] / k, the integral of ln(m0 / m) (k = mdot / m0).
// The prediction is explicit Euler with dt = 0.05 s (speed first, then the height with the new speed): its distance is
// short by about a dt T / 2 (T the burn time, a the deceleration), its end time quantised to dt. Tolerances: duration
// 2 dt; height a_max dt T (twice the Euler bias).
TEST_CASE("Landing-burn prediction matches the constant-thrust hand solution", "[booster_return][gnc]") {
    const double h0_m = GENERATE(1000.0, 100.0);  // 100 m: the burn cannot stop above the ground (negative stop height)
    CAPTURE(h0_m);
    auto cfg = test::booster_config("starship_block3", 120.0e3);
    for (auto& [id, engine] : cfg.engines) engine.startup_s = 0.0;
    const auto center = figures(cfg, "center");
    const auto divert = figures(cfg, "divert");
    const auto middle = figures(cfg, "middle");
    const double throttle = 0.8;
    const int n = center.count + divert.count + middle.count;
    const double thrust_n = throttle * (center.count * center.vacuum_thrust_n + divert.count * divert.vacuum_thrust_n +
                                        middle.count * middle.vacuum_thrust_n);
    const double mdot = throttle * (center.count * center.mass_flow_kgps + divert.count * divert.mass_flow_kgps +
                                    middle.count * middle.mass_flow_kgps);
    REQUIRE(n == 13);

    gnc::BoosterReturnParameters p;
    p.scvx_landing = false;
    p.landing_ignition_groups = {"booster/center", "booster/divert", "booster/middle"};
    p.landing_groups = p.landing_ignition_groups;
    p.landing_throttle = throttle;
    p.final_throttle = throttle;
    const gnc::BoosterReturnGuidance guidance(site_on_equator, p);

    auto vehicle = std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR);
    const double m0 = vehicle->mass_properties().mass_kg;
    const double cg_m = vehicle->mass_properties().cg_station_m;
    const double v0 = 150.0;
    const vehicle::RigidBodyState state{{constants::wgs84_semi_major_axis_m + cg_m + h0_m, 0.0, 0.0}, {-v0, 0.0, 0.0}, {}, {}};
    const vehicle::FlightBody body(std::move(vehicle), state);
    const FixedEarth earth;
    const UniformGravity gravity;
    const Vacuum vacuum;
    const vehicle::FlightEnvironment env{gravity, vacuum, earth, nullptr, false};

    const double g = constants::standard_gravity_mps2;
    const double c = thrust_n / mdot;
    const double k = mdot / m0;
    const auto speed = [&](double t) { return v0 - c * std::log(m0 / (m0 - mdot * t)) + g * t; };
    const auto way = [&](double t) {
        const double u = 1.0 - k * t;
        return v0 * t - c * (u * std::log(u) + k * t) / k + 0.5 * g * t * t;
    };
    REQUIRE(thrust_n / m0 > 2.0 * g);  // decelerates strongly: the root below is unique
    double lo = 0.0, hi = 60.0;
    for (int i = 0; i < 100; ++i) (speed(0.5 * (lo + hi)) > 0.0 ? lo : hi) = 0.5 * (lo + hi);
    const double t_stop = 0.5 * (lo + hi);
    const double stop_height_m = h0_m - way(t_stop);
    const double a_max = thrust_n / (m0 - mdot * t_stop) - g;

    const auto burn = guidance.predict_landing_burn(body, env, time::Epoch::from_tdb_seconds_past_j2000(0.0));
    CHECK(burn.duration_s == Catch::Approx(t_stop).margin(0.1));
    CHECK(burn.stop_height_m == Catch::Approx(stop_height_m).margin(a_max * 0.05 * t_stop));
    CHECK((burn.stop_height_m > 0.0) == (stop_height_m > 0.0));
}
