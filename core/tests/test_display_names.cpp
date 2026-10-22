// External view (webcast overlay): display names of vehicles, separated parts and missions (no trademarks), and the
// main-engine propellant load of a body (its propellant gauge).

#include <filesystem>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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

}  // namespace

TEST_CASE("Stages that fly as separate bodies carry a display name", "[config][display]") {
    const auto block3 = config::load_vehicle(config_dir, "starship_block3");
    CHECK(stage(block3, "booster").display_name == "Booster Mk3");
    CHECK(stage(block3, "ship").display_name == "Raumschiff Mk3");
    const auto dragon = config::load_vehicle(config_dir, "crew_dragon");
    CHECK(stage(dragon, "capsule").display_name == "Crew Capsule");
    CHECK(stage(dragon, "trunk").display_name == "Trunk");
    const auto f9 = config::load_vehicle(config_dir, "falcon9_block5");
    CHECK(stage(f9, "first_stage").display_name == "Medium Launcher Stufe 1");
    CHECK(stage(f9, "second_stage").display_name == "Medium Launcher Stufe 2");
    CHECK(stage(config::load_vehicle(config_dir, "starship_depot"), "depot").display_name == "Depot Mk3");
}

TEST_CASE("A reused stage keeps the source's display name unless it sets its own", "[config][display]") {
    const auto tanker = config::load_vehicle(config_dir, "starship_tanker");
    CHECK(stage(tanker, "booster").display_name == "Booster Mk3");
    CHECK(stage(tanker, "ship").display_name == "Tanker Mk3");
    const auto hls_launch = config::load_vehicle(config_dir, "starship_hls_launch");
    CHECK(stage(hls_launch, "hls").display_name == "Lunar Lander Mk3");
}

TEST_CASE("Missions may carry a display name", "[config][display]") {
    CHECK(config::mission_display_name(config_dir, "starship_leo") == "Testflug in den Erdorbit");
    CHECK(config::mission_display_name(config_dir, "leo_ascent").empty());  // a target file without [mission]
}

TEST_CASE("A body's shown name: the vehicle's display name, else its parts'", "[vehicle][display]") {
    vehicle::Vehicle stack(config::load_vehicle(config_dir, "starship_block3"), config_dir);
    CHECK(stack.shown_name() == "Heavy Launcher Mk3");
    auto ship = stack.separate(1);
    CHECK(stack.shown_name() == "Booster Mk3");
    CHECK(ship->shown_name() == "Raumschiff Mk3");

    // Falcon 9 with Crew Dragon: the second stage carries the capsule; capsule + trunk (no main engines) is the
    // capsule, the topmost named stage; split up, each part by its own stage.
    const auto launch = config::stack_payload(config::load_vehicle(config_dir, "falcon9_block5"),
                                              config::load_vehicle(config_dir, "crew_dragon"));
    vehicle::Vehicle f9(launch, config_dir);
    CHECK(f9.shown_name() == "Medium Launcher");
    auto upper = f9.separate(1);
    CHECK(f9.shown_name() == "Medium Launcher Stufe 1");
    CHECK(upper->shown_name() == "Medium Launcher Stufe 2");
    auto dragon = upper->separate(1);
    CHECK(upper->shown_name() == "Medium Launcher Stufe 2");
    CHECK(dragon->shown_name() == "Crew Capsule");
    auto capsule = dragon->separate(1);
    CHECK(capsule->shown_name() == "Crew Capsule");
    CHECK(dragon->shown_name() == "Trunk");
    CHECK(vehicle::Vehicle(config::load_vehicle(config_dir, "crew_dragon"), config_dir).shown_name() == "Crew Capsule");
}

TEST_CASE("The top stage flying alone is shown by its stage, not by the stack's name", "[vehicle][display]") {
    // a ship placed in orbit or on its way back from the Moon (config::top_stage_config): not "Heavy Launcher"
    const vehicle::Vehicle ship(config::top_stage_config(config::load_vehicle(config_dir, "starship_block3")), config_dir);
    CHECK(ship.shown_name() == "Raumschiff Mk3");
    const vehicle::Vehicle tanker(config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker")), config_dir);
    CHECK(tanker.shown_name() == "Tanker Mk3");
    const vehicle::Vehicle hls(config::top_stage_config(config::load_vehicle(config_dir, "starship_hls")), config_dir);
    CHECK(hls.shown_name() == "Lunar Lander Mk3");  // a one-stage vehicle keeps its own name
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
