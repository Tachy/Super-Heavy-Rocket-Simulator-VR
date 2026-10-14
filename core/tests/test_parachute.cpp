// Parachutes (d4 of docs/superpowers/specs/2026-10-14-rcs-crew-dragon-design.md): configuration, drag area over the
// reefing stages, riser-point force; vehicle state, body physics, opening loads; the deployment sequencer.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "starsim/aero/parachute.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using math::Vec3;

namespace {

config::ParachuteConfig two_stage_chute() {
    config::ParachuteConfig p;
    p.id = "test";
    p.role = "main";
    p.count = 1;
    p.drag_area_m2 = 100.0;
    p.fill_time_s = 2.0;
    p.reefing = {{0.1, 3.0}, {0.4, 3.0}};
    p.max_load_n = 1e6;
    p.deploy_height_m = 1000.0;
    return p;
}

}  // namespace

TEST_CASE("Parachute drag area grows through its reefing stages", "[parachute]") {
    const auto p = two_stage_chute();
    CHECK(aero::parachute_drag_area_m2(p, -1.0) == 0.0);
    CHECK(aero::parachute_drag_area_m2(p, 0.0) == 0.0);
    CHECK(aero::parachute_drag_area_m2(p, 1.0) == Catch::Approx(100.0 * 0.1 * 0.25));  // t^2 fill: half time, quarter
    CHECK(aero::parachute_drag_area_m2(p, 2.0) == Catch::Approx(10.0));                // first stage reached
    CHECK(aero::parachute_drag_area_m2(p, 4.9) == Catch::Approx(10.0));                // held 3 s
    CHECK(aero::parachute_drag_area_m2(p, 6.0) == Catch::Approx(10.0 + 30.0 * 0.25));  // filling to stage 2
    CHECK(aero::parachute_drag_area_m2(p, 7.0) == Catch::Approx(40.0));
    CHECK(aero::parachute_drag_area_m2(p, 11.0) == Catch::Approx(40.0 + 60.0 * 0.25)); // full opening
    CHECK(aero::parachute_drag_area_m2(p, 12.0) == Catch::Approx(100.0));
    CHECK(aero::parachute_drag_area_m2(p, 1000.0) == Catch::Approx(100.0));
    auto unreefed = p;
    unreefed.reefing.clear();
    CHECK(aero::parachute_drag_area_m2(unreefed, 1.0) == Catch::Approx(25.0));
    CHECK(aero::parachute_drag_area_m2(unreefed, 2.0) == Catch::Approx(100.0));
}

TEST_CASE("Parachute force opposes the riser point's air velocity with q C_D S", "[parachute]") {
    const Vec3 v{-50.0, 3.0, -4.0};
    const auto f = aero::parachute_force_body_n(20.0, v, 1.2);
    const double q = 0.5 * 1.2 * dot(v, v);
    CHECK(norm(f) == Catch::Approx(q * 20.0));
    CHECK(dot(math::normalized(f), math::normalized(v)) == Catch::Approx(-1.0));
    CHECK(norm(aero::parachute_force_body_n(20.0, {}, 1.2)) == 0.0);
}

TEST_CASE("Crew Dragon carries two drogues and four mains", "[parachute][config]") {
    const auto c = config::load_vehicle(STARSIM_CONFIG_DIR, "crew_dragon");
    const auto& capsule = c.stages.back();
    REQUIRE(capsule.parachutes.size() == 2);
    const auto& drogue = capsule.parachutes[0];
    const auto& main = capsule.parachutes[1];
    CHECK(drogue.role == "drogue");
    CHECK(drogue.count == 2);
    CHECK(drogue.deploy_height_m == Catch::Approx(5500.0));
    CHECK(main.role == "main");
    CHECK(main.count == 4);
    CHECK(main.deploy_height_m == Catch::Approx(1800.0));
    CHECK(main.deploy_height_m < drogue.deploy_height_m);
    CHECK(capsule.max_splashdown_speed_mps == Catch::Approx(9.0));
    CHECK(c.stages.front().parachutes.empty());  // the trunk
}

namespace {

// A copy of the config dir with `from` replaced by `to` in crew_dragon.toml.
std::filesystem::path patched_config(const std::string& from, const std::string& to) {
    namespace fs = std::filesystem;
    const fs::path src = STARSIM_CONFIG_DIR;
    const fs::path dir = fs::temp_directory_path() / "starsim_parachute_config";
    fs::remove_all(dir);
    fs::create_directories(dir);
    fs::copy(src, dir, fs::copy_options::recursive);
    std::ifstream in(src / "vehicles" / "crew_dragon.toml");
    std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    std::ofstream(dir / "vehicles" / "crew_dragon.toml") << text;
    return dir;
}

}  // namespace

TEST_CASE("Parachute config is validated", "[parachute][config]") {
    CHECK_THROWS_AS(config::load_vehicle(patched_config("role = \"drogue\"", "role = \"ballute\""), "crew_dragon"),
                    config::ConfigError);
    CHECK_THROWS_AS(config::load_vehicle(patched_config("reefing = [[0.35, 4.0]]", "reefing = [[1.2, 4.0]]"), "crew_dragon"),
                    config::ConfigError);
    CHECK_THROWS_AS(config::load_vehicle(patched_config("max_splashdown_speed_mps = 9.0", ""), "crew_dragon"),
                    config::ConfigError);
    CHECK_THROWS_AS(config::load_vehicle(patched_config("deploy_height_m = 1800.0", "deploy_height_m = 6000.0"), "crew_dragon"),
                    config::ConfigError);
}
