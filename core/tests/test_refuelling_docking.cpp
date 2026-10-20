// Refuelling r3: Starship rendezvous and probe/drogue docking at the depot.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using math::Vec3;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

std::unique_ptr<vehicle::Vehicle> ship_alone(const std::string& vehicle_id) {
    return std::make_unique<vehicle::Vehicle>(config::top_stage_config(config::load_vehicle(config_dir, vehicle_id)),
                                              config_dir);
}

}  // namespace

TEST_CASE("Tanker and HLS translate on all six axes; the Block 3 ship still has no -x", "[refuelling][rcs]") {
    for (const std::string id : {"starship_tanker", "starship_hls"}) {
        auto ship = ship_alone(id);
        ship->set_tank_mass(0, "lox_main", 7826.0);  // 10 t main propellant at O/F 3.6 (r1 translation case)
        ship->set_tank_mass(0, "ch4_main", 2174.0);
        INFO(id);
        for (const Vec3 u : {Vec3{1, 0, 0}, Vec3{-1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, -1, 0}, Vec3{0, 0, 1}, Vec3{0, 0, -1}}) {
            CHECK(ship->rcs_pure_force_capability(u).force_n > 1000.0);
        }
        CHECK(ship->rcs_pure_force_capability({-1, 0, 0}).force_n ==
              Catch::Approx(ship->rcs_pure_force_capability({1, 0, 0}).force_n).epsilon(0.01));
    }
    auto block3 = ship_alone("starship_block3");
    CHECK(block3->rcs_pure_force_capability({-1, 0, 0}).force_n == 0.0);
}

TEST_CASE("RCS supply: main-tank cold gas counts the main tanks, Dragon its RCS tank", "[refuelling][rcs]") {
    auto tanker = ship_alone("starship_tanker");
    tanker->set_tank_mass(0, "lox_main", 20000.0);
    tanker->set_tank_mass(0, "ch4_main", 6000.0);
    CHECK(tanker->rcs_supply_kg() == Catch::Approx(26000.0));  // the headers are not RCS supply (landing reserve)
    CHECK(tanker->rcs_propellant_kg() == 0.0);
    const vehicle::Vehicle dragon(config::load_vehicle(config_dir, "crew_dragon"), config_dir);
    CHECK(dragon.rcs_supply_kg() == Catch::Approx(dragon.rcs_propellant_kg()));
    CHECK(dragon.rcs_supply_kg() > 0.0);
}

namespace {

// A config dir with Block 3, the engines and propellants and one test vehicle (as test_refuelling_vehicles.cpp; own
// temp prefix - ctest runs cases in parallel).
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r3_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml", dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string reused_stage(const std::string& from, const std::string& additions) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"booster\"]\n\n[[stage]]\nfrom = \"" + from + "\"\n" + additions;
}

const std::string docking_pod = "[[stage.rcs_pod]]\nid = \"dock\"\nstation_m = 46.0\nradius_m = 4.5\n"
                                "azimuths_deg = [0.0, 180.0]\nthrust_n = 1500.0\ndirections = [\"aft\"]\n";

size_t aft_thrusters(const config::StageConfig& s) {
    return static_cast<size_t>(std::count_if(s.rcs.thrusters.begin(), s.rcs.thrusters.end(),
                                             [](const config::RcsThrusterConfig& t) { return t.direction.x < -0.99; }));
}

}  // namespace

TEST_CASE("A reused stage may add RCS pods, not other RCS keys", "[refuelling][config]") {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    CHECK(aft_thrusters(tanker.stages.back()) == 2);
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");

    // the booster has [stage.rcs] (its forward pod): pods may be added
    auto dir = config_with("rcs_pod_ok", reused_stage("starship_block3:booster", docking_pod));
    const auto added = config::load_vehicle(dir, "test_vehicle");
    CHECK(added.stages.back().rcs.thrusters.size() == block3.stages.front().rcs.thrusters.size() + 2);
    CHECK(aft_thrusters(added.stages.back()) == 2);
    std::filesystem::remove_all(dir);

    dir = config_with("rcs_table", reused_stage("starship_block3:booster", "[stage.rcs]\nisp_s = 70.0\n"));
    CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                      Catch::Matchers::ContainsSubstring("cannot be set on a reused stage"));
    std::filesystem::remove_all(dir);
}

TEST_CASE("An rcs_pod on a reused stage without RCS is an error", "[refuelling][config]") {
    const auto dir = config_with("rcs_none", reused_stage("bare:booster", docking_pod));
    std::ofstream(dir / "vehicles" / "bare.toml")
        << "[vehicle]\nid = \"bare\"\nname = \"Bare\"\npropellant = \"methalox\"\nstages_bottom_to_top = [\"booster\"]\n\n"
           "[[stage]]\nid = \"booster\"\nname = \"Bare\"\nlength_m = 10.0\ndiameter_m = 9.0\ndry_mass_kg = 1000.0\n"
           "dry_cg_station_m = 5.0\n";
    CHECK(config::load_vehicle(dir, "bare").stages.front().rcs.thrusters.empty());
    CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"), Catch::Matchers::ContainsSubstring("rcs_pod needs"));
    std::filesystem::remove_all(dir);
}
