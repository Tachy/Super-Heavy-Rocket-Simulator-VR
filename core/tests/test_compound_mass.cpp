// ISS i5: compound mass of a docked pair and the docked hold of a body.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <memory>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/vehicle/compound_mass.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Mat3;
using math::Vec3;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

Mat3 diag(double a, double b, double c) { return Mat3::from_columns({a, 0, 0}, {0, b, 0}, {0, 0, c}); }

class ZeroGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3&, time::Epoch) const override { return {}; }
};
class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {Mat3::identity(), {}}; }
};
struct DeepSpace {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;  // zero above 1000 km
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};
}  // namespace

TEST_CASE("Compound mass: sum, CG and parallel-axis inertia", "[iss][docking]") {
    const auto c = vehicle::combine_mass(4.0, {0, 0, 0}, diag(1, 1, 1), 1.0, {5, 0, 0}, diag(2, 2, 2));
    CHECK(c.mass_kg == 5.0);
    CHECK(c.cg_m.x == Catch::Approx(1.0));
    // about x: 1 + 2; about y and z: 1 + 4*1^2 + 2 + 1*4^2 = 23
    CHECK(c.inertia_kg_m2(0, 0) == Catch::Approx(3.0));
    CHECK(c.inertia_kg_m2(1, 1) == Catch::Approx(23.0));
    CHECK(c.inertia_kg_m2(2, 2) == Catch::Approx(23.0));
    CHECK(c.inertia_kg_m2(0, 1) == Catch::Approx(0.0).margin(1e-12));
}

TEST_CASE("Compound mass: an off-axis part gives products of inertia", "[iss][docking]") {
    const auto c = vehicle::combine_mass(1.0, {0, 0, 0}, diag(0, 0, 0), 1.0, {2, 2, 0}, diag(0, 0, 0));
    // about the compound CG (1, 1, 0): each part at (+-1, +-1, 0): I_xy = -sum m x y = -2
    CHECK(c.inertia_kg_m2(0, 1) == Catch::Approx(-2.0));
    CHECK(c.inertia_kg_m2(0, 1) == c.inertia_kg_m2(1, 0));
    CHECK(c.inertia_kg_m2(2, 2) == Catch::Approx(4.0));
}

TEST_CASE("Docked hold: the body is not integrated and burns nothing", "[iss][docking]") {
    DeepSpace space;
    vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir, "crew_dragon"), config_dir),
                             vehicle::RigidBodyState{{1e10, 0.0, 0.0}, {1.0, 2.0, 3.0}, {}, {0.0, 0.01, 0.0}});
    body.vehicle().set_nose_cone_open(true);
    const auto before = body.state();
    const double prop = body.vehicle().rcs_propellant_kg();
    body.hold_docked();
    REQUIRE(body.docked());
    vehicle::VehicleCommand cmd;
    cmd.rcs_force_fraction = {1.0, 0.0, 0.0};
    const auto t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
    for (int i = 0; i < 100; ++i) body.step(space.env, t0 + 0.005 * i, 0.005, cmd);
    CHECK(body.state().position_gcrf_m.x == before.position_gcrf_m.x);
    CHECK(body.state().angular_velocity_body_radps.y == before.angular_velocity_body_radps.y);
    CHECK(body.vehicle().rcs_propellant_kg() == prop);
    CHECK(norm(body.rcs_force_body_n()) == 0.0);
    CHECK(body.can_coast());
    body.coast(space.env, t0 + 1.0, 600.0);
    CHECK(body.state().position_gcrf_m.x == before.position_gcrf_m.x);
    body.release_docked();
    CHECK_FALSE(body.docked());
    body.step(space.env, t0 + 700.0, 0.005, {});
    CHECK(body.state().position_gcrf_m.x != before.position_gcrf_m.x);  // integrated again
}
