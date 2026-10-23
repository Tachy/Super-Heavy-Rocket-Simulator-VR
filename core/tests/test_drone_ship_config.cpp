// Drone ships chosen from the list in config/drone_ships.toml by the first stage's impact.

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;

namespace {
constexpr double deg = constants::deg_to_rad;
}

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
