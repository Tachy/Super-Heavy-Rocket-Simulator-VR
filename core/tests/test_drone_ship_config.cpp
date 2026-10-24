// Drone ships chosen from the list in config/drone_ships.toml by the first stage's impact.

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;

namespace {
constexpr double deg = constants::deg_to_rad;

// A copy of config/drone_ships.toml with every occurrence of one text replaced (unique directory per test).
std::filesystem::path patched(const std::string& name, const std::string& from, const std::string& to) {
    std::ifstream in(std::filesystem::path(STARSIM_CONFIG_DIR) / "drone_ships.toml");
    std::string text{std::istreambuf_iterator<char>(in), {}};
    REQUIRE(text.find(from) != std::string::npos);
    for (auto at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size()))
        text.replace(at, from.size(), to);
    const auto dir = std::filesystem::temp_directory_path() / ("starsim_drone_ship_" + name);
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    std::ofstream(dir / "drone_ships.toml") << text;
    return dir;
}
}  // namespace

TEST_CASE("The drone ship list is loaded in file order", "[config][drone_ship]") {
    const auto ships = config::load_drone_ships(STARSIM_CONFIG_DIR);
    REQUIRE(ships.size() == 3);
    CHECK(ships[0].id == "asds_atlantic");
    CHECK(ships[0].basin == "atlantic");
    CHECK(ships[2].id == "asds_pacific");
    CHECK(ships[2].basin == "pacific");
    for (const auto& s : ships) CHECK(s.name == "Drohnenschiff");
}

TEST_CASE("The drone ship for a point is the one whose service area holds it", "[config][drone_ship]") {
    const auto ships = config::load_drone_ships(STARSIM_CONFIG_DIR);
    // two Atlantic ships share a home port: the file order breaks the tie
    const auto atlantic = config::drone_ship_for(ships, 30.5 * deg, -74.0 * deg);
    REQUIRE(atlantic);
    CHECK(atlantic->id == "asds_atlantic");
    const auto pacific = config::drone_ship_for(ships, 30.0 * deg, -125.0 * deg);
    REQUIRE(pacific);
    CHECK(pacific->id == "asds_pacific");
    CHECK_FALSE(config::drone_ship_for(ships, -10.0 * deg, 20.0 * deg));
    // a longitude beyond pi wraps (235 deg east = 125 deg west)
    const auto wrapped = config::drone_ship_for(ships, 30.0 * deg, 235.0 * deg);
    REQUIRE(wrapped);
    CHECK(wrapped->id == "asds_pacific");
}

TEST_CASE("The drone ship hull and deck limits are loaded", "[config][drone_ship]") {
    using Catch::Approx;
    for (const auto& s : config::load_drone_ships(STARSIM_CONFIG_DIR)) {
        INFO(s.id);
        CHECK(s.hull.length_m == Approx(91.4));
        CHECK(s.hull.beam_m == Approx(30.5));
        CHECK(s.hull.draft_m == Approx(3.0));  // depth 6.0 m - freeboard 3.0 m [D]
        CHECK(s.hull.roll_gyration_m == Approx(0.35 * 30.5));
        CHECK(s.hull.pitch_gyration_m == Approx(0.25 * 91.4));
        CHECK(s.hull.heave_added_mass_factor > 1.0);  // a wide shallow barge: several displaced masses
        CHECK(s.hull.roll_added_inertia_factor > 0.0);
        CHECK(s.hull.pitch_added_inertia_factor > 0.0);
        for (double z : {s.hull.heave_damping_ratio, s.hull.roll_damping_ratio, s.hull.pitch_damping_ratio}) {
            CHECK(z > 0.0);
            CHECK(z < 1.0);
        }
        CHECK(s.max_deck_tilt_rad == Approx(3.0 * deg));
        CHECK(s.max_heave_rate_mps == Approx(1.0));
    }
}

TEST_CASE("A hull that does not reach below the waterline is refused", "[config][drone_ship]") {
    const auto dir = patched("draft", "hull_depth_m = 6.0", "hull_depth_m = 3.0");
    CHECK_THROWS(config::load_drone_ships(dir));
    std::filesystem::remove_all(dir);
}
