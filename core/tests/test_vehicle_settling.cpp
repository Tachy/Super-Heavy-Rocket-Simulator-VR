#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/propellant_loading.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using Catch::Approx;
using math::Vec3;
using vehicle::Vehicle;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// The ship alone with `propellant_kg` aboard (headers first).
Vehicle ship_with(double propellant_kg) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], propellant_kg);
    return Vehicle(cfg, config_dir);
}

double angle_between(const Vec3& a, const Vec3& b) { return std::acos(std::clamp(math::dot(a, b), -1.0, 1.0)); }

}  // namespace

TEST_CASE("Propellant starts settled aft and the tilted path meets the column", "[settling]") {
    auto ship = ship_with(64000.0);
    CHECK(ship.liquid_down_body().x == -1.0);
    const auto aft = ship.mass_properties();
    ship.set_liquid_down_body({-1.0, 1e-7, 0.0});  // table path, a hair off the axis
    const auto tilted = ship.mass_properties();
    CHECK(tilted.cg_station_m == Approx(aft.cg_station_m).margin(1e-4));
    CHECK(tilted.inertia_kg_m2.y == Approx(aft.inertia_kg_m2.y).epsilon(1e-5));
    CHECK(tilted.mass_kg == aft.mass_kg);
}

TEST_CASE("In a belly-first entry the residue moves forward and to the belly", "[settling]") {
    // Entry at alpha ~60 deg: the flow on the belly (-z) pushes the hull to +z and slightly aft (axial / normal ~0.05),
    // the liquid goes the other way.
    auto ship = ship_with(64000.0);
    const auto aft = ship.mass_properties();
    ship.set_liquid_down_body({0.05, 0.0, -1.0});
    const auto entry = ship.mass_properties();
    INFO("CG aft " << aft.cg_station_m << " entry " << entry.cg_station_m << " z " << entry.cg_offset_m.z);
    CHECK(entry.cg_station_m > aft.cg_station_m + 0.5);  // 64 t: 35 t full headers, 29 t main-tank residue (+0.95 m)
    CHECK(entry.cg_offset_m.z < -0.1);
    CHECK(entry.cg_offset_m.y == Approx(0.0).margin(1e-9));
}

TEST_CASE("Settling follows the specific force with its time constant and holds in a coast", "[settling]") {
    auto ship = ship_with(64000.0);
    ship.settle_propellant({0.0, 0.0, 0.005}, 100.0);  // below the threshold: no change
    CHECK(ship.liquid_down_body().x == -1.0);

    // One g towards +z: down turns to -z with tau = sqrt(2 D / a) (D = 9 m).
    const double a = constants::standard_gravity_mps2;
    const double tau = std::sqrt(2.0 * 9.0 / a);
    for (int i = 0; i < 1000; ++i) ship.settle_propellant({0.0, 0.0, a}, tau / 1000.0);
    const Vec3 target{0.0, 0.0, -1.0};
    CHECK(angle_between(ship.liquid_down_body(), target) == Approx(0.5 * constants::pi * std::exp(-1.0)).epsilon(1e-6));
    CHECK(math::norm(ship.liquid_down_body()) == Approx(1.0).epsilon(1e-12));
}

TEST_CASE("A reversal through 180 deg stays a unit vector and ends opposite", "[settling]") {
    auto ship = ship_with(64000.0);
    ship.set_liquid_down_body({1.0, 0.0, 0.0});  // at the forward domes
    for (int i = 0; i < 2000; ++i) {
        ship.settle_propellant({20.0, 0.0, 0.0}, 0.01);  // thrust: back aft
        REQUIRE(math::norm(ship.liquid_down_body()) == Approx(1.0).epsilon(1e-9));
    }
    CHECK(ship.liquid_down_body().x < -0.999999);
}

TEST_CASE("Separation and copies keep the liquid direction", "[settling]") {
    auto stack = Vehicle(config::load_vehicle(config_dir, "starship_block3"), config_dir);
    stack.set_liquid_down_body({0.2, 0.0, -1.0});
    const Vec3 down = stack.liquid_down_body();
    const auto copy = stack.copy_upper(1);
    const auto ship = stack.separate(1);
    for (const Vec3& d : {copy->liquid_down_body(), ship->liquid_down_body(), stack.liquid_down_body()}) {
        CHECK(d.x == Approx(down.x));
        CHECK(d.z == Approx(down.z));
    }
}

namespace {

class ZeroGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    Vec3 acceleration_mps2(const Vec3&, time::Epoch) const override { return {}; }
};

class FixedEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch) const override { return {math::Mat3::identity(), {}}; }
};

// Roll rate after one tick of a 40 t ship (5 t main-tank residue) at 60 km over the equator, nose up, moving belly
// first at 1,500 m/s with the flaps stowed: the hull's normal force passes through the axis.
double roll_rate_after_tick(const Vec3& liquid_down_body) {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    const vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    config::fill_tanks(cfg.stages[0], 40000.0);
    for (auto& f : cfg.stages[0].flaps) f.initial_deflection_rad = 0.0;
    auto vehicle = std::make_unique<Vehicle>(cfg, config_dir);
    vehicle->set_liquid_down_body(liquid_down_body);
    const vehicle::RigidBodyState state{{6378137.0 + 60000.0, 0.0, 0.0}, {0.0, 0.0, -1500.0}, {}, {}};
    vehicle::FlightBody body(std::move(vehicle), state);
    body.step(env, time::Epoch::from_tdb_seconds_past_j2000(8.4e8), 0.005, {});
    return body.state().angular_velocity_body_radps.x;
}

}  // namespace

// The liquid does not turn with a round tank (no roll inertia): the wall pressure passes through the axis, so a
// sideways residue gives no roll moment - the hull's normal force acts about the axis in roll, not about the offset CG.
TEST_CASE("Liquid off the axis gives no roll moment", "[settling]") {
    const auto ship = ship_with(40000.0);
    auto side = ship_with(40000.0);
    side.set_liquid_down_body({0.0, 1.0, 0.0});
    REQUIRE(std::abs(side.mass_properties().cg_offset_m.y) > 0.01);  // the residue lies on the +y side
    CHECK(side.mass_properties().roll_offset_m.y == Approx(ship.mass_properties().roll_offset_m.y).margin(1e-12));

    const double column = roll_rate_after_tick({-1.0, 0.0, 0.0});
    const double sideways = roll_rate_after_tick({0.0, 1.0, 0.0});
    INFO("roll rate column " << column << " sideways " << sideways);
    CHECK(sideways == Approx(column).margin(1e-9));
}
