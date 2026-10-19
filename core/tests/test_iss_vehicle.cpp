#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/aero/fixed_area_aero.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/flight_body.hpp"

using namespace starsim;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

math::StateVector iss_like_state() {
    const double r = constants::wgs84_semi_major_axis_m + 415.0e3;
    const double v = std::sqrt(constants::earth_gm_m3ps2 / r);
    const double i = 51.64 * constants::deg_to_rad;
    return {{r, 0.0, 0.0}, {0.0, v * std::cos(i), v * std::sin(i)}};
}

class EarthPointMass final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    math::Vec3 acceleration_mps2(const math::Vec3& r, time::Epoch) const override {
        const double d = norm(r);
        return r * (-constants::earth_gm_m3ps2 / (d * d * d));
    }
};
class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};
// Same rotation <=> |q1 . q2| = 1 (q and -q are the same rotation).
double dot4(const math::Quat& a, const math::Quat& b) { return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z; }
}  // namespace

TEST_CASE("Fixed-area drag opposes the motion with C_D A q", "[iss][aero]") {
    const aero::FixedAreaAero aero{1177.76, 1.5};
    // the hull-aero convention (SlenderBodyAero, FlightBody): the body's velocity relative to the air, body axes
    const Vec3 motion{7660.0, 30.0, -12.0};
    const double rho = 3.0e-12;
    const auto result = aero.compute(motion, rho, 900.0);
    const double q = 0.5 * rho * dot(motion, motion);
    CHECK(norm(result.force_body_n) == Catch::Approx(q * 1.5 * 1177.76).epsilon(1e-12));
    CHECK(norm(cross(result.force_body_n, motion)) < 1e-12 * norm(result.force_body_n) * norm(motion));
    CHECK(dot(result.force_body_n, motion) < 0.0);  // drag
    CHECK(norm(result.moment_body_nm) == 0.0);
    CHECK(result.dynamic_pressure_pa == Catch::Approx(q));
}

TEST_CASE("ISS config: mass, drag, inertia, ports and primitives", "[iss][config]") {
    const auto iss = config::load_vehicle(config_dir, "iss");
    REQUIRE(iss.stages.size() == 1);
    const auto& s = iss.stages.front();
    CHECK(s.dry_mass_kg == Catch::Approx(460320.0));
    CHECK(s.aero.fixed_drag_area_m2 == Catch::Approx(1177.76));
    CHECK(s.aero.fixed_drag_coefficient == Catch::Approx(1.5));
    REQUIRE(s.inertia_kg_m2.has_value());
    REQUIRE(s.body_in_lvlh_rad.has_value());
    REQUIRE(s.docking_ports.size() == 2);
    CHECK(s.docking_ports[0].id == "ida2");
    CHECK(s.docking_ports[1].id == "ida3");
    for (const auto& p : s.docking_ports) {
        CHECK(norm(p.axis) == Catch::Approx(1.0));
        CHECK(std::abs(dot(p.axis, p.up)) < 1e-12);
    }
    CHECK(s.docking_ports[0].axis.x == Catch::Approx(1.0));   // forward: V-bar
    CHECK(s.docking_ports[1].axis.z == Catch::Approx(-1.0));  // zenith (body +z = nadir): R-bar
    CHECK(s.collision.size() >= 6);
}

TEST_CASE("Vehicle uses the fixed area and the explicit inertia", "[iss][vehicle]") {
    const auto cfg = config::load_vehicle(config_dir, "iss");
    const vehicle::Vehicle iss(cfg, config_dir);
    REQUIRE(iss.fixed_area_aero() != nullptr);
    const auto mp = iss.mass_properties();
    CHECK(mp.mass_kg == Catch::Approx(460320.0));
    const auto& inertia = *cfg.stages.front().inertia_kg_m2;
    CHECK(mp.inertia_kg_m2.x == Catch::Approx(inertia.x));
    CHECK(mp.inertia_kg_m2.y == Catch::Approx(inertia.y));
    CHECK(mp.inertia_kg_m2.z == Catch::Approx(inertia.z));
    const auto air = environment::AtmosphereState{3.0e-12, 0.0, 1000.0, 900.0};
    const auto drag = iss.hull_aero({7660.0, 0.0, 0.0}, air, false);  // flying +x through the air
    CHECK(drag.force_body_n.x == Catch::Approx(-0.5 * 3.0e-12 * 7660.0 * 7660.0 * 1.5 * 1177.76));
}

TEST_CASE("Rocket inertia is unchanged by the explicit-inertia path", "[iss][vehicle]") {
    const auto cfg = config::load_vehicle(config_dir, "crew_dragon");
    const vehicle::Vehicle dragon(cfg, config_dir);
    const auto mp = dragon.mass_properties();
    CHECK(mp.inertia_kg_m2.y == mp.inertia_kg_m2.z);  // axisymmetric as before
}

TEST_CASE("LVLH axes: x along the velocity, z to nadir, rate = orbit rate", "[iss][frames]") {
    const auto s = iss_like_state();
    const auto q = frames::lvlh_to_inertial(s);
    CHECK(dot(q.rotate({1.0, 0.0, 0.0}), math::normalized(s.velocity_mps)) == Catch::Approx(1.0).margin(1e-12));
    CHECK(dot(q.rotate({0.0, 0.0, 1.0}), -math::normalized(s.position_m)) == Catch::Approx(1.0).margin(1e-12));
    const Vec3 w = frames::lvlh_rate_inertial_radps(s);
    CHECK(norm(w) == Catch::Approx(norm(s.velocity_mps) / norm(s.position_m)).epsilon(1e-12));
}

TEST_CASE("Ideal LVLH attitude follows the orbit through ticks and coasts", "[iss][vehicle]") {
    const auto cfg = config::load_vehicle(config_dir, "iss");
    const auto s = iss_like_state();
    vehicle::RigidBodyState state{s.position_m, s.velocity_mps, frames::lvlh_to_inertial(s), {}};
    vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
    const math::Quat tea = math::Quat::from_axis_angle({0.0, 0.0, 1.0}, -3.0 * constants::deg_to_rad);
    body.set_ideal_lvlh_attitude(tea);
    CHECK(body.ideal_attitude());

    environment::Us76Atmosphere atmosphere;
    EarthPointMass gravity;
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
    const time::Epoch t0 = time::Epoch::from_tdb_seconds_past_j2000(8.5e8);
    for (int k = 0; k < 200; ++k) body.step(env, t0 + k * 0.005, 0.005, {});
    body.coast(env, t0 + 1.0, 1500.0);
    const auto& end = body.state();
    const math::StateVector end_sv{end.position_gcrf_m, end.velocity_gcrf_mps};
    const math::Quat expected = frames::lvlh_to_inertial(end_sv) * tea;
    CHECK(std::abs(std::abs(dot4(end.attitude, expected)) - 1.0) < 1e-12);
    const Vec3 w_body = end.attitude.inverse_rotate(frames::lvlh_rate_inertial_radps(end_sv));
    CHECK(norm(end.angular_velocity_body_radps - w_body) < 1e-15);
}
