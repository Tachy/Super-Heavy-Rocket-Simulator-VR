// Entry heating (Sutton-Graves) and the ship's heat-shield / structural limits.

#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/aero/entry_heating.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/vehicle/flight_body.hpp"

using namespace starsim;
using Catch::Approx;
using math::Vec3;

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

struct World {
    ZeroGravity gravity;
    environment::Us76Atmosphere atmosphere;
    FixedEarth earth;
    vehicle::FlightEnvironment env{gravity, atmosphere, earth, nullptr, false};
};

// Ship alone at `height_m` over the equator (on GCRF +y), body axes = GCRF axes; `velocity` horizontal.
std::unique_ptr<vehicle::FlightBody> ship(double height_m, const Vec3& velocity, double heat_rate_limit = 0.0) {
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    cfg.stages.erase(cfg.stages.begin());
    if (heat_rate_limit > 0.0) cfg.stages[0].heat_shield.max_heat_rate_w_m2 = heat_rate_limit;
    // Entry mass: only the landing propellant left (split like the full tanks).
    double full = 0.0;
    for (const auto& tank : cfg.stages[0].tanks) full += tank.mass_kg;
    for (auto& tank : cfg.stages[0].tanks) tank.mass_kg *= cfg.stages[0].landing_propellant_kg / full;
    const vehicle::RigidBodyState state{{0.0, 6378137.0 + height_m, 0.0}, velocity, {}, {}};  // horizontal flight
    return std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
}

void fly(vehicle::FlightBody& body, World& world, double duration_s) {
    const double dt = 0.005;
    for (double t = 0.0; t < duration_s - 1e-9; t += dt) body.step(world.env, t0 + t, dt, {});
}

}  // namespace

TEST_CASE("Sutton-Graves stagnation heat rate", "[heating]") {
    // q = k sqrt(rho / R_n) v^3: 1.7415e-4 * sqrt(1e-4) * 7000^3 = 597.3 kW/m^2
    CHECK(aero::stagnation_heat_rate_w_m2(1e-4, 7000.0, 1.0) == Approx(5.9733e5).epsilon(1e-4));
    CHECK(aero::stagnation_heat_rate_w_m2(0.0, 7000.0, 1.0) == 0.0);
    CHECK(aero::stagnation_heat_rate_w_m2(1e-4, 7000.0, 0.0) == 0.0);  // no heat shield: not modelled
}

TEST_CASE("The ship's heat shield fails beyond its heat-rate limit", "[heating][vehicle]") {
    World world;
    auto body = ship(40000.0, {7000.0, 0.0, 0.0});  // nose first: little drag, ~1.5 MW/m^2
    fly(*body, world, 0.1);
    CHECK(body->impacted());
    CHECK(body->breakup_reason() == "Hitzeschild versagt");
}

TEST_CASE("Heat load integrates the heat rate; a warning comes before the limit", "[heating][vehicle]") {
    World world;
    auto body = ship(40000.0, {7000.0, 0.0, 0.0}, 1.0e7);
    fly(*body, world, 1.0);
    REQUIRE_FALSE(body->impacted());
    CHECK(body->breakup_reason().empty());
    CHECK(body->peak_heat_rate_w_m2() > 1.4e6);
    CHECK(body->heat_load_j_m2() == Approx(body->peak_heat_rate_w_m2() * 1.0).epsilon(0.02));
    CHECK_FALSE(body->heat_warning());  // 15 % of the raised limit
    auto warm = ship(40000.0, {7000.0, 0.0, 0.0}, 1.8e6);
    fly(*warm, world, 0.1);
    CHECK(warm->heat_warning());
    CHECK_FALSE(warm->impacted());
}

TEST_CASE("A belly-first ship breaks up beyond its load limit", "[heating][vehicle]") {
    World world;
    auto body = ship(40000.0, {0.0, 0.0, -7000.0}, 1.0e8);  // broadside at 40 km: ~17 g
    fly(*body, world, 0.1);
    CHECK(body->impacted());
    CHECK(body->breakup_reason() == "Strukturüberlast");
}

TEST_CASE("Vehicles without a heat shield and stacks are not judged", "[heating][vehicle]") {
    World world;
    auto cfg = config::load_vehicle(config_dir, "starship_block3");
    const vehicle::RigidBodyState state{{6378137.0 + 40000.0, 0.0, 0.0}, {7000.0, 0.0, 0.0}, {}, {}};
    vehicle::FlightBody stack(std::make_unique<vehicle::Vehicle>(cfg, config_dir), state);
    fly(stack, world, 0.1);
    CHECK_FALSE(stack.impacted());
    CHECK(stack.heat_load_j_m2() == 0.0);
}

#include "starsim/sim/landing_outcome.hpp"

TEST_CASE("A broken-up ship's outcome is 'zerbrochen'", "[heating][vehicle]") {
    World world;
    auto body = ship(40000.0, {7000.0, 0.0, 0.0});
    fly(*body, world, 0.1);
    REQUIRE(body->impacted());
    sim::LaunchSite site;
    CHECK(sim::landing_outcome(*body, site, {}) == sim::LandingOutcome::BrokenUp);
    CHECK(std::string(sim::to_string(sim::LandingOutcome::BrokenUp)) == "zerbrochen");
}
