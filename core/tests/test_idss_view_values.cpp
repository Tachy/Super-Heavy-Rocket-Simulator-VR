// Task 6 of the Falcon / Dragon / ISS models: the docking mechanism's view numbers are copies of the physics' C++ defaults (the core has no
// config path for them), kept honest here; the sim's port lies on the drawn mating plane.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <toml++/toml.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/docking_mechanism.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;

toml::table detail() { return toml::parse_file((config_dir / "vehicles" / "detail" / "crew_dragon.toml").string()); }
}  // namespace

TEST_CASE("The view's mechanism numbers equal the physics defaults", "[docking][dragon][idss_view]") {
    const auto file = detail();
    const contact::DockingMechanismParameters p;
    const auto* idss = file["idss"].as_table();
    REQUIRE(idss != nullptr);
    CHECK((*idss)["ring_extension_m"].value_or(-1.0) == Catch::Approx(p.interface.ring_extension_m));
    CHECK((*idss)["ring_tilt_max_deg"].value_or(-1.0) * constants::deg_to_rad == Catch::Approx(p.interface.ring_tilt_max_rad));
    CHECK((*idss)["petal_count"].value_or(-1) == p.interface.petal_count);
    CHECK((*idss)["petal_height_m"].value_or(-1.0) == Catch::Approx(p.interface.petal_height_m));
    CHECK((*idss)["petal_base_diameter_m"].value_or(-1.0) == Catch::Approx(2.0 * p.interface.petal_outline_radius_m));
    CHECK((*idss)["hook_count"].value_or(-1) == p.hook_count);
}

TEST_CASE("Dragon's port lies on the drawn mating plane", "[docking][dragon][idss_view]") {
    const auto file = detail();
    const auto* neck = file["capsule"]["neck_m"].as_array();
    REQUIRE(neck != nullptr);
    const double top_of_flange_m = neck->at(0).as_array()->at(0).value_or(-1.0);
    const auto vehicle = config::load_vehicle(config_dir, "crew_dragon");
    const auto& port = vehicle.stages.back().docking_ports.front();
    CHECK(port.position_m.x == Catch::Approx(top_of_flange_m).margin(1e-9));
}
