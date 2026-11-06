// Display names live in the text tables (config/i18n/de.toml, en.toml), not in the config files: every shown id has its
// name key in both tables, the loaders no longer read `name` / `display_name` (an old file that still has them loads),
// and a body's shown name is a table key (vehicle::Vehicle::shown_name_key). Also the main-engine propellant load of a
// body (its propellant gauge).

#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <toml++/toml.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;
using Catch::Approx;

namespace {

const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

const config::StageConfig& stage(const config::VehicleConfig& v, const std::string& id) {
    for (const auto& s : v.stages)
        if (s.id == id) return s;
    FAIL("no stage " << id);
    return v.stages.front();
}

// Flattened leaf keys of a text table ("[vehicle]" + "starship_block3.name" -> "vehicle.starship_block3.name") with
// their text.
void flatten(const toml::table& t, const std::string& prefix, std::map<std::string, std::string>& out) {
    for (const auto& [key, node] : t) {
        const std::string name = prefix.empty() ? std::string(key.str()) : prefix + "." + std::string(key.str());
        if (const auto* sub = node.as_table()) {
            flatten(*sub, name, out);
        } else if (const auto s = node.value<std::string>()) {
            out[name] = *s;
        }
    }
}

std::map<std::string, std::string> load_table(const char* language) {
    std::map<std::string, std::string> out;
    flatten(toml::parse_file((config_dir / "i18n" / (std::string(language) + ".toml")).string()), "", out);
    return out;
}

// Every id of config/ whose name is shown, as its table key.
std::set<std::string> shown_name_keys() {
    std::set<std::string> keys;
    for (const auto& entry : std::filesystem::directory_iterator(config_dir / "vehicles")) {
        if (entry.path().extension() != ".toml") continue;
        const std::string id = entry.path().stem().string();
        const auto v = config::load_vehicle(config_dir, id);
        keys.insert(v.name_key);
        keys.insert(v.display_name_key);
        for (const auto& s : v.stages) {
            keys.insert(s.name_key);
            for (const auto& p : s.docking_ports) keys.insert(p.name_key);
        }
    }
    for (const auto& entry : std::filesystem::directory_iterator(config_dir / "missions")) {
        if (entry.path().extension() != ".toml") continue;
        const std::string id = entry.path().stem().string();
        const toml::table doc = toml::parse_file(entry.path().string());
        if (!doc.contains("mission")) continue;  // a target file
        keys.insert("mission." + id + ".name");
        keys.insert(config::mission_display_name_key(config_dir, id));
    }
    const auto ids_of = [](const char* file, const char* array) {
        std::vector<std::string> ids;
        const toml::table doc = toml::parse_file((config_dir / file).string());
        if (const auto* list = doc[array].as_array()) {
            for (const auto& node : *list) ids.push_back(node.as_table()->at("id").value<std::string>().value());
        }
        return ids;
    };
    for (const auto& id : ids_of("launch_sites.toml", "site")) {
        keys.insert("site." + id + ".name");
        const toml::table doc = toml::parse_file((config_dir / "launch_sites.toml").string());
        for (const auto& node : *doc["site"].as_array()) {
            const auto& t = *node.as_table();
            if (t.at("id").value<std::string>().value() == id && t.contains("viewpoint")) {
                keys.insert("site." + id + ".viewpoint.name");
            }
        }
    }
    for (const auto& id : ids_of("start_places.toml", "place")) keys.insert("place." + id + ".name");
    for (const auto& id : ids_of("lunar_sites.toml", "site")) keys.insert("lunar_site." + id + ".name");
    for (const auto& id : ids_of("splashdown_zones.toml", "zone")) keys.insert("zone." + id + ".name");
    for (const auto& id : ids_of("drone_ships.toml", "drone_ship")) keys.insert("drone_ship." + id + ".name");
    const toml::table gauges = toml::parse_file((config_dir / "ui" / "webcast_gauges.toml").string());
    for (const auto& [id, node] : *gauges["gauge"].as_table()) keys.insert("gauge." + std::string(id.str()) + ".label");
    return keys;
}

}  // namespace

TEST_CASE("Every shown name has its key in both text tables", "[config][display]") {
    const auto de = load_table("de");
    const auto en = load_table("en");
    const auto keys = shown_name_keys();
    CHECK(keys.size() > 100);
    for (const auto& key : keys) {
        INFO(key);
        CHECK(de.count(key) == 1);
        CHECK(en.count(key) == 1);
    }
}

TEST_CASE("No trademark in the shown names", "[config][display]") {
    for (const char* language : {"de", "en"}) {
        for (const auto& [key, text] : load_table(language)) {
            const bool name_section = key.rfind("vehicle.", 0) == 0 || key.rfind("mission.", 0) == 0 ||
                                      key.rfind("site.", 0) == 0 || key.rfind("place.", 0) == 0 ||
                                      key.rfind("lunar_site.", 0) == 0 || key.rfind("zone.", 0) == 0 ||
                                      key.rfind("drone_ship.", 0) == 0 || key.rfind("gauge.", 0) == 0;
            if (!name_section) continue;
            INFO(language << " " << key << " = " << text);
            CHECK(text.find("Starship") == std::string::npos);
            CHECK(text.find("SpaceX") == std::string::npos);
        }
    }
}

TEST_CASE("The loaders do not read names; an old file that still has them loads", "[config][display]") {
    const auto dir = std::filesystem::temp_directory_path() / "starsim_display_names_old_file";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir / "vehicles");
    std::filesystem::copy(config_dir / "engines", dir / "engines", std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(config_dir / "propellants.toml", dir / "propellants.toml");
    std::ifstream in(config_dir / "vehicles" / "starship_block3.toml", std::ios::binary);
    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string text = buffer.str();
    const auto replace_first = [&](const std::string& what, const std::string& with) {
        const auto at = text.find(what);
        REQUIRE(at != std::string::npos);
        text.replace(at, what.size(), with);
    };
    replace_first("[vehicle]", "[vehicle]\nname = \"Old name\"\ndisplay_name = \"Old display name\"");
    replace_first("id = \"booster\"", "id = \"booster\"\nname = \"Old booster\"\ndisplay_name = \"Old booster display\"");
    std::ofstream(dir / "vehicles" / "starship_block3.toml", std::ios::binary) << text;

    const auto v = config::load_vehicle(dir, "starship_block3");
    CHECK(v.name_key == "vehicle.starship_block3.name");
    CHECK(v.display_name_key == "vehicle.starship_block3.display_name");
    CHECK(stage(v, "booster").name_key == "vehicle.starship_block3.stage.booster.display_name");
    std::filesystem::remove_all(dir);
}

TEST_CASE("Stages that fly as separate bodies carry a name key", "[config][display]") {
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    CHECK(stage(block3, "booster").name_key == "vehicle.starship_block3.stage.booster.display_name");
    CHECK(stage(block3, "ship").name_key == "vehicle.starship_block3.stage.ship.display_name");
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    CHECK(stage(dragon, "capsule").name_key == "vehicle.crew_dragon.stage.capsule.display_name");
    CHECK(stage(dragon, "trunk").name_key == "vehicle.crew_dragon.stage.trunk.display_name");
    const auto f9 = config::load_vehicle(config_dir, "falcon9_block5");
    CHECK(stage(f9, "first_stage").name_key == "vehicle.falcon9_block5.stage.first_stage.display_name");
    CHECK(stage(f9, "second_stage").name_key == "vehicle.falcon9_block5.stage.second_stage.display_name");
    CHECK(stage(config::load_vehicle(config_dir, "starship_depot"), "depot").name_key ==
          "vehicle.starship_depot.stage.depot.display_name");
}

TEST_CASE("A reused stage has its own key under the vehicle that lists it", "[config][display]") {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    CHECK(stage(tanker, "booster").name_key == "vehicle.starship_tanker.stage.booster.display_name");
    CHECK(stage(tanker, "ship").name_key == "vehicle.starship_tanker.stage.ship.display_name");
    const auto hls_launch = config::load_vehicle(config_dir, "starship_hls_launch");
    CHECK(stage(hls_launch, "hls").name_key == "vehicle.starship_hls_launch.stage.hls.display_name");
}

TEST_CASE("Missions may carry a display name key", "[config][display]") {
    CHECK(config::mission_display_name_key(config_dir, "starship_leo") == "mission.starship_leo.display_name");
    CHECK(config::mission_display_name_key(config_dir, "leo_ascent").empty());  // a target file without [mission]
}

TEST_CASE("A body's shown name key: the vehicle's, else its parts'", "[vehicle][display]") {
    vehicle::Vehicle stack(config::load_vehicle(config_dir, "starship_block3"), config_dir);
    CHECK(stack.shown_name_key() == "vehicle.starship_block3.display_name");
    auto ship = stack.separate(1);
    CHECK(stack.shown_name_key() == "vehicle.starship_block3.stage.booster.display_name");
    CHECK(ship->shown_name_key() == "vehicle.starship_block3.stage.ship.display_name");

    // Falcon 9 with Crew Dragon: the second stage carries the capsule; capsule + trunk (no main engines) is the
    // capsule, the topmost named stage; split up, each part by its own stage.
    const auto launch = config::stack_payload(config::load_vehicle(config_dir, "falcon9_block5"),
                                              config::load_vehicle(config_dir, "crew_dragon"));
    vehicle::Vehicle f9(launch, config_dir);
    CHECK(f9.shown_name_key() == "vehicle.falcon9_block5.display_name");
    auto upper = f9.separate(1);
    CHECK(f9.shown_name_key() == "vehicle.falcon9_block5.stage.first_stage.display_name");
    CHECK(upper->shown_name_key() == "vehicle.falcon9_block5.stage.second_stage.display_name");
    auto dragon = upper->separate(1);
    CHECK(upper->shown_name_key() == "vehicle.falcon9_block5.stage.second_stage.display_name");
    CHECK(dragon->shown_name_key() == "vehicle.crew_dragon.stage.capsule.display_name");
    auto capsule = dragon->separate(1);
    CHECK(capsule->shown_name_key() == "vehicle.crew_dragon.stage.capsule.display_name");
    CHECK(dragon->shown_name_key() == "vehicle.crew_dragon.stage.trunk.display_name");
    CHECK(vehicle::Vehicle(config::load_vehicle(config_dir, "crew_dragon"), config_dir).shown_name_key() ==
          "vehicle.crew_dragon.display_name");
}

TEST_CASE("The top stage flying alone is shown by its stage, not by the stack's name", "[vehicle][display]") {
    // a ship placed in orbit or on its way back from the Moon (config::top_stage_config): not "Heavy Launcher"
    const vehicle::Vehicle ship(config::top_stage_config(config::load_vehicle(config_dir, "starship_block3")), config_dir);
    CHECK(ship.shown_name_key() == "vehicle.starship_block3.stage.ship.display_name");
    const vehicle::Vehicle tanker(config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker")), config_dir);
    CHECK(tanker.shown_name_key() == "vehicle.starship_tanker.stage.ship.display_name");
    const vehicle::Vehicle hls(config::top_stage_config(config::load_vehicle(config_dir, "starship_hls")), config_dir);
    CHECK(hls.shown_name_key() == "vehicle.starship_hls.display_name");  // a one-stage vehicle keeps its own name
}

TEST_CASE("The propellant gauge's load counts main and header tanks", "[vehicle][display]") {
    vehicle::Vehicle stack(config::load_vehicle(config_dir, "starship_block3"), config_dir);
    auto ship = stack.separate(1);
    double capacity = 0.0;
    for (const auto& t : ship->stages().front().tanks)
        if (t.config.contents != config::TankContents::Rcs) capacity += t.config.capacity_kg;
    CHECK(ship->stages().front().has_header_tanks());
    CHECK(ship->propellant_capacity_kg() == Approx(capacity));
    CHECK(ship->propellant_capacity_kg() > ship->main_propellant_kg());
    CHECK(ship->propellant_kg() <= ship->propellant_capacity_kg() + 1e-6);
}
