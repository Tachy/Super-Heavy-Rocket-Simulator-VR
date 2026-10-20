// Refuelling r1: tanker, depot and HLS vehicles, run-time tank masses, probe/drogue mating geometry.

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

// The top stage of a vehicle flying alone (the ship after staging).
std::unique_ptr<vehicle::Vehicle> top_stage_alone(const std::string& vehicle_id) {
    auto cfg = config::load_vehicle(config_dir, vehicle_id);
    cfg.stages.erase(cfg.stages.begin(), cfg.stages.end() - 1);
    return std::make_unique<vehicle::Vehicle>(cfg, config_dir);
}

}  // namespace

TEST_CASE("set_tank_mass sets one tank at run time, clamped to its capacity", "[refuelling][vehicle]") {
    auto ship = top_stage_alone("starship_block3");
    const double full_kg = ship->mass_properties().mass_kg;
    const double lox_capacity_kg = 1224808.7;

    ship->set_tank_mass(0, "lox_main", 100000.0);
    CHECK(ship->mass_properties().mass_kg == Catch::Approx(full_kg - (lox_capacity_kg - 100000.0)));

    ship->set_tank_mass(0, "lox_main", 5.0e6);  // more than fits
    CHECK(ship->stages()[0].propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) ==
          Catch::Approx(lox_capacity_kg));

    ship->set_tank_mass(0, "ch4_main", -1.0);
    CHECK(ship->stages()[0].propellant_kg(config::TankContents::Fuel, config::TankRole::Main) == 0.0);

    CHECK_THROWS_AS(ship->set_tank_mass(0, "no_such_tank", 1.0), std::invalid_argument);
    CHECK_THROWS_AS(ship->set_tank_mass(3, "lox_main", 1.0), std::invalid_argument);
}

TEST_CASE("A refilled dry ship has its engines and cold-gas RCS back", "[refuelling][vehicle]") {
    auto ship = top_stage_alone("starship_block3");
    for (const char* id : {"lox_main", "ch4_main", "lox_header", "ch4_header"}) ship->set_tank_mass(0, id, 0.0);
    ship->consume_propellant(0.0);  // the main feed is dry: the engines starve
    REQUIRE(ship->stages()[0].engines.front().starved);
    CHECK(ship->rcs_pure_force_capability({0.0, 0.0, 1.0}).force_n == 0.0);  // ullage gas comes from the tanks

    ship->set_tank_mass(0, "lox_main", 300000.0);
    ship->set_tank_mass(0, "ch4_main", 80000.0);
    CHECK_FALSE(ship->stages()[0].engines.front().starved);
    CHECK(ship->rcs_pure_force_capability({0.0, 0.0, 1.0}).force_n > 0.0);
}
