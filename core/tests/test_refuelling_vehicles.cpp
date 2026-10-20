// Refuelling r1: tanker, depot and HLS vehicles, run-time tank masses, probe/drogue mating geometry.

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/port_mating.hpp"
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

namespace {

// A config directory of its own (ctest runs cases in parallel): engines, propellants, Block 3 and one test vehicle.
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_refuelling_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml",
                               dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string vehicle_with_stage(const std::string& stage_body) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"ship\"]\n\n[[stage]]\n" +
           stage_body;
}

}  // namespace

TEST_CASE("A stage taken over with `from` is the source vehicle's stage", "[refuelling][config]") {
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    REQUIRE(tanker.stages.size() == block3.stages.size());
    for (size_t i = 0; i < block3.stages.size(); ++i) {
        const auto& a = block3.stages[i];
        const auto& b = tanker.stages[i];
        CAPTURE(a.id);
        CHECK(b.id == a.id);
        CHECK(b.dry_mass_kg == a.dry_mass_kg);
        CHECK(b.dry_cg_station_m == a.dry_cg_station_m);
        REQUIRE(b.tanks.size() == a.tanks.size());
        for (size_t k = 0; k < a.tanks.size(); ++k) {
            CHECK(b.tanks[k].id == a.tanks[k].id);
            CHECK(b.tanks[k].mass_kg == a.tanks[k].mass_kg);
            CHECK(b.tanks[k].capacity_kg == a.tanks[k].capacity_kg);
        }
        CHECK(b.engine_groups.size() == a.engine_groups.size());
        CHECK(b.rcs.thrusters.size() == a.rcs.thrusters.size());
        CHECK(b.flaps.size() == a.flaps.size());
    }
    CHECK(tanker.engines.size() == block3.engines.size());
}

TEST_CASE("Bad `from` stages are config errors", "[refuelling][config]") {
    struct Case {
        const char* name;
        const char* body;
        const char* message;
    };
    const Case cases[] = {
        {"from_override", "from = \"starship_block3:ship\"\ndry_mass_kg = 1.0\n", "cannot be set on a reused stage"},
        {"from_no_stage", "from = \"starship_block3:nose\"\n", "no such stage"},
        {"from_no_vehicle", "from = \"no_such_vehicle:ship\"\n", "no_such_vehicle.toml"},
        {"from_syntax", "from = \"starship_block3\"\n", "must be \"<vehicle>:<stage>\""},
        {"from_cycle", "from = \"test_vehicle:ship\"\n", "chain too long"},
    };
    for (const auto& c : cases) {
        CAPTURE(c.name);
        const auto dir = config_with(c.name, vehicle_with_stage(c.body));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"), Catch::Matchers::ContainsSubstring(c.message));
    }
}

namespace {

config::DockingPortConfig dorsal_port(config::SoftCaptureMode side) {
    config::DockingPortConfig p;
    p.id = "transfer_dock";
    p.mechanism = config::DockingMechanism::ProbeDrogue;
    p.position_m = {20.0, 0.0, 5.0};
    p.axis = {0.0, 0.0, 1.0};
    p.up = {1.0, 0.0, 0.0};
    p.soft_capture = side;
    p.elements_m = {{12.0, -1.5, 5.0}, {12.0, 1.5, 5.0}, {28.0, -1.5, 5.0}, {28.0, 1.5, 5.0}};
    return p;
}

}  // namespace

TEST_CASE("Back-to-back mating puts the ships parallel, 10 m apart, rolled 180 deg", "[refuelling][docking]") {
    const auto t = vehicle::mated_stage_transform(dorsal_port(config::SoftCaptureMode::Active),
                                                  dorsal_port(config::SoftCaptureMode::Passive));
    const math::Vec3 nose = t.rotation * math::Vec3{1.0, 0.0, 0.0};
    CHECK(nose.x == Catch::Approx(1.0));  // both noses the same way
    const math::Vec3 dorsal = t.rotation * math::Vec3{0.0, 0.0, 1.0};
    CHECK(dorsal.z == Catch::Approx(-1.0));  // backs face each other
    const math::Vec3 axis_point = t.apply({20.0, 0.0, 0.0});
    CHECK(axis_point.x == Catch::Approx(20.0));
    CHECK(axis_point.y == Catch::Approx(0.0).margin(1e-12));
    CHECK(axis_point.z == Catch::Approx(10.0));  // centrelines 10 m apart
}

TEST_CASE("Ports mate only with the same mechanism and probes against drogues", "[refuelling][docking]") {
    const auto probes = dorsal_port(config::SoftCaptureMode::Active);
    const auto drogues = dorsal_port(config::SoftCaptureMode::Passive);
    CHECK_FALSE(vehicle::port_mismatch(probes, drogues).has_value());
    CHECK(vehicle::port_mismatch(probes, probes).has_value());
    CHECK(vehicle::port_mismatch(drogues, probes).has_value());

    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    const auto iss = config::load_vehicle(config_dir, "iss");
    REQUIRE(dragon.stages.back().docking_ports.size() == 1);  // the capsule (stages: trunk, capsule)
    const auto& idss = dragon.stages.back().docking_ports.front();
    CHECK(idss.mechanism == config::DockingMechanism::Idss);  // existing ports default to IDSS
    CHECK(idss.elements_m.empty());
    CHECK_FALSE(vehicle::port_mismatch(idss, iss.stages.front().docking_ports.front()).has_value());
    CHECK(vehicle::port_mismatch(idss, drogues).has_value());  // a Dragon cannot dock to a depot drogue
}

TEST_CASE("Bad probe/drogue ports and transfer connectors are config errors", "[refuelling][config]") {
    const std::string ship = "from = \"starship_block3:ship\"\n";
    const std::string port_head = "[[stage.docking_port]]\nid = \"p\"\nname = \"P\"\nposition_m = [20.0, 0.0, 5.0]\n"
                                  "axis = [0.0, 0.0, 1.0]\nup = [1.0, 0.0, 0.0]\n";
    const std::string connector_head = "[[stage.transfer_connector]]\nid = \"c\"\nposition_m = [8.0, 0.0, 5.0]\n";
    struct Case {
        const char* name;
        std::string body;
        const char* message;
    };
    const Case cases[] = {
        {"pd_no_elements", ship + port_head + "mechanism = \"probe_drogue\"\n", "at least 3 elements_m"},
        {"pd_two_elements",
         ship + port_head + "mechanism = \"probe_drogue\"\nelements_m = [[12.0, 0.0, 5.0], [28.0, 0.0, 5.0]]\n",
         "at least 3 elements_m"},
        {"idss_elements", ship + port_head + "elements_m = [[12.0, 0.0, 5.0], [28.0, 0.0, 5.0], [20.0, 1.0, 5.0]]\n",
         "belong to a probe_drogue port"},
        {"pd_mechanism", ship + port_head + "mechanism = \"magnet\"\n", "mechanism must be"},
        {"tc_no_tank", ship + connector_head + "tank = \"nope\"\n", "main propellant tank"},
        {"tc_header", ship + connector_head + "tank = \"lox_header\"\n", "main propellant tank"},
    };
    for (const auto& c : cases) {
        CAPTURE(c.name);
        const auto dir = config_with(c.name, vehicle_with_stage(c.body));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"), Catch::Matchers::ContainsSubstring(c.message));
    }
}
