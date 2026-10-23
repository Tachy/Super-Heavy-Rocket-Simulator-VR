#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using Catch::Matchers::WithinAbs;

TEST_CASE("Launch sites load with their catch tower", "[config]") {
    const auto site = config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1");
    CHECK(site.name == "Starbase OLP-1");
    CHECK_THAT(site.latitude_rad, WithinAbs(25.99686 * constants::deg_to_rad, 1e-12));
    CHECK(site.height_m == 10.0);
    REQUIRE(site.tower.has_value());
    const auto& t = *site.tower;
    CHECK(t.catch_height_m > t.mount_height_m);
    CHECK(t.arm_closed_gap_m > 9.0);  // wider than the 9 m booster hull
    CHECK(t.mount_inner_radius_m < t.mount_outer_radius_m);
    CHECK_THAT(t.arm_open_rad, WithinAbs(40.0 * constants::deg_to_rad, 1e-12));
    REQUIRE(site.sea.has_value());
    CHECK_THAT(site.sea->direction_rad, WithinAbs(90.0 * constants::deg_to_rad, 1e-12));
    CHECK(site.sea->shoreline_distance_m > 0.0);
    CHECK(config::load_launch_site(STARSIM_CONFIG_DIR, "ksc_lc39a").tower.has_value());
    CHECK_THROWS_AS(config::load_launch_site(STARSIM_CONFIG_DIR, "nowhere"), config::ConfigError);
}

TEST_CASE("Boosters define two opposite catch pins", "[config]") {
    for (const char* id : {"starship_block3"}) {
        CAPTURE(id);
        const auto v = config::load_vehicle(STARSIM_CONFIG_DIR, id);
        const auto& booster = v.stages.front();
        const auto& pins = booster.catch_pins;
        REQUIRE(pins.azimuths_rad.size() == 2);
        CHECK_THAT(std::abs(pins.azimuths_rad[1] - pins.azimuths_rad[0]), WithinAbs(constants::pi, 1e-12));
        CHECK(pins.station_m > booster.grid_fins.station_m);
        CHECK(pins.station_m < booster.length_m);
        CHECK(pins.protrusion_m > 0.0);
        CHECK(pins.max_load_n > 0.0);
        CHECK(v.stages.back().catch_pins.azimuths_rad.size() == 2);  // the ship is caught too (b3)
    }
}

TEST_CASE("SLC-40 and its landing zone LZ-40 load", "[config][falcon9]") {
    const auto pad = config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_slc40");
    CHECK(pad.kind == "launch_pad");
    CHECK_FALSE(pad.tower.has_value());
    CHECK(pad.pad_radius_m == 0.0);
    CHECK(pad.landing_zone_id == "ccsfs_lz40");
    CHECK(config::load_launch_site(STARSIM_CONFIG_DIR, "ksc_lc39a").landing_zone_id.empty());
    const auto lz = config::load_launch_site(STARSIM_CONFIG_DIR, "ccsfs_lz40");
    CHECK(lz.kind == "landing_zone");
    CHECK(lz.pad_radius_m > 20.0);
    REQUIRE(lz.sea.has_value());
    // LZ-40 lies inside the SLC-40 complex: less than 1.5 km from the pad.
    const auto a = frames::geodetic_to_fixed({pad.latitude_rad, pad.longitude_rad, pad.height_m});
    const auto b = frames::geodetic_to_fixed({lz.latitude_rad, lz.longitude_rad, lz.height_m});
    CHECK(norm(a - b) < 1500.0);
    CHECK(config::load_launch_site(STARSIM_CONFIG_DIR, "starbase_olp1").kind == "launch_pad");
}
