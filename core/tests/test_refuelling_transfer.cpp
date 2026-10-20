// Refuelling r4: propellant transfer between docked ships.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/vehicle_config.hpp"

using namespace starsim;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

const config::StageConfig& ship_stage(const std::string& id) {
    static std::map<std::string, config::VehicleConfig> cache;
    auto it = cache.find(id);
    if (it == cache.end()) it = cache.emplace(id, config::load_vehicle(config_dir, id)).first;
    return it->second.stages.back();
}

// A config dir with Block 3, the engines and propellants and one test vehicle (as test_refuelling_docking.cpp; own
// temp prefix - ctest runs cases in parallel).
std::filesystem::path config_with(const std::string& name, const std::string& vehicle_toml) {
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_r4_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::filesystem::copy_file(config_dir / "vehicles" / "starship_block3.toml", dir / "vehicles" / "starship_block3.toml");
    std::ofstream(dir / "vehicles" / "test_vehicle.toml") << vehicle_toml;
    return dir;
}

std::string tanker_with(const std::string& ship_additions) {
    return "[vehicle]\nid = \"test_vehicle\"\nname = \"Test\"\npropellant = \"methalox\"\n"
           "stages_bottom_to_top = [\"ship\"]\n\n[[stage]]\nfrom = \"starship_block3:ship\"\n" + ship_additions;
}

}  // namespace

TEST_CASE("Tanker, depot and HLS carry the transfer hardware", "[refuelling][prop_transfer][config]") {
    for (const std::string id : {"starship_tanker", "starship_depot", "starship_hls"}) {
        INFO(id);
        const auto& s = ship_stage(id);
        REQUIRE(s.propellant_transfer);
        CHECK(s.propellant_transfer->delivery_pressure_pa == 3.5e5);
        CHECK(s.propellant_transfer->receive_pressure_pa == 1.5e5);
        REQUIRE(s.transfer_connectors.size() == 2);
        for (const auto& c : s.transfer_connectors) {
            CHECK(c.line_diameter_m == 0.15);
            CHECK(c.discharge_coefficient == 0.6);
            CHECK(c.chilldown_kg == (c.contents == config::TankContents::Oxidizer ? 150.0 : 60.0));
        }
    }
    CHECK_FALSE(ship_stage("starship_block3").propellant_transfer);
}

TEST_CASE("Transfer hardware errors are reported", "[refuelling][prop_transfer][config]") {
    const std::string connector =
        "[[stage.transfer_connector]]\nid = \"lox\"\ntank = \"lox_main\"\nposition_m = [8.0, 0.0, 5.0]\n";
    SECTION("a connector without its line") {
        const auto dir = config_with("no_line", tanker_with(connector));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                          Catch::Matchers::ContainsSubstring("line_diameter_m"));
    }
    SECTION("receive pressure not below delivery") {
        const auto dir = config_with(
            "pressure", tanker_with(connector + "line_diameter_m = 0.15\ndischarge_coefficient = 0.6\n"
                                                "chilldown_kg = 150.0\n\n[stage.propellant_transfer]\n"
                                                "delivery_pressure_pa = 2.0e5\nreceive_pressure_pa = 2.0e5\n"));
        CHECK_THROWS_WITH(config::load_vehicle(dir, "test_vehicle"),
                          Catch::Matchers::ContainsSubstring("receive_pressure_pa"));
    }
}
