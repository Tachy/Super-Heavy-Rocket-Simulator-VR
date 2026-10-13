// Ship flip and catch (b3): engines 3 -> 2 -> 1, catch pins, SCvx landing plans, flip and catch in flight.

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <set>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/vehicle.hpp"

using namespace starsim;

namespace {

constexpr double deg = constants::deg_to_rad;

}  // namespace

TEST_CASE("Ship has three single sea-level engine groups and catch pins", "[ship][landing]") {
    const auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
    const auto& ship = cfg.stages.at(1);
    std::set<std::string> sea_level;
    for (const auto& g : ship.engine_groups) {
        if (g.engine_id != "raptor3") continue;
        sea_level.insert(g.id);
        CHECK(g.count == 1);
        CHECK(g.gimbal);
        CHECK(g.ring_radius_m == 1.3);
    }
    CHECK(sea_level == std::set<std::string>{"sea_level_1", "sea_level_2", "sea_level_3"});

    const vehicle::Vehicle vehicle(cfg, STARSIM_CONFIG_DIR);
    int sea_level_engines = 0;
    std::set<long> angles;
    for (const auto& e : vehicle.stages().at(1).engines) {
        if (e.group_id.rfind("ship/sea_level_", 0) != 0) continue;
        ++sea_level_engines;
        angles.insert(std::lround(std::atan2(e.gimbal_point_m.z, e.gimbal_point_m.y) / deg));
    }
    CHECK(sea_level_engines == 3);
    CHECK(angles == std::set<long>{0, 120, -120});

    REQUIRE(ship.catch_pins.azimuths_rad.size() == 2);
    CHECK(std::abs(ship.catch_pins.azimuths_rad[0] - 90.0 * deg) < 1e-9);
    CHECK(std::abs(ship.catch_pins.azimuths_rad[1] - 270.0 * deg) < 1e-9);
    CHECK(ship.catch_pins.station_m > 30.0);
    CHECK(ship.catch_pins.station_m < 38.0);  // below the forward flaps (38-46 m)
}
