// The process-wide terrain cache (M8 Plan B, Task 1): one instance while held, loader errors rethrown and not cached.

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/data_missing.hpp"
#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/moon_surface_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/rock_abundance_map.hpp"
#include "starsim/sim/earth_terrain_loader.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/terrain_cache.hpp"
#include "test_support.hpp"

using namespace starsim;

TEST_CASE("cache returns the same instance while held", "[terrain_cache][data]") {
    const std::filesystem::path data_dir = STARSIM_DATA_DIR;
    REQUIRE(sim::earth_terrain_available(data_dir));
    const environment::EarthTerrain* first_address = nullptr;
    {
        const auto a = sim::shared_earth_terrain(STARSIM_CONFIG_DIR, data_dir);
        const auto b = sim::shared_earth_terrain(STARSIM_CONFIG_DIR, data_dir);
        REQUIRE(a);
        CHECK(a.get() == b.get());
        first_address = a.get();
    }
    // Released by both: the weak entry is expired, a third call loads again (a new instance; the address may repeat).
    const auto c = sim::shared_earth_terrain(STARSIM_CONFIG_DIR, data_dir);
    REQUIRE(c);
    CHECK(first_address != nullptr);
    const auto d = sim::shared_earth_terrain(STARSIM_CONFIG_DIR, data_dir);
    CHECK(c.get() == d.get());
}

TEST_CASE("cache rethrows a missing file", "[terrain_cache]") {
    const auto dir = std::filesystem::temp_directory_path() / "starsim_terrain_cache_empty_data";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    CHECK_THROWS_AS(sim::shared_earth_terrain(STARSIM_CONFIG_DIR, dir), config::ConfigError);
    CHECK_THROWS_AS(sim::shared_earth_terrain(STARSIM_CONFIG_DIR, dir), config::ConfigError);
    CHECK_THROWS_AS(sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, dir), config::ConfigError);
    CHECK_THROWS_AS(sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, dir), config::ConfigError);
    std::filesystem::remove_all(dir);
}

TEST_CASE("two sites give two terrains, the cache keeps both alive while held", "[sim][lunar][terrain][data]") {
    const std::filesystem::path data_dir = STARSIM_DATA_DIR;
    const auto a11 = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_11");
    const auto a15 = config::load_lunar_site(STARSIM_CONFIG_DIR, "apollo_15");
    if (!sim::lunar_site_tiles_present(STARSIM_CONFIG_DIR, data_dir, a11) ||
        !sim::lunar_site_tiles_present(STARSIM_CONFIG_DIR, data_dir, a15)) {
        SKIP("Apollo 11 / 15 tiles not installed");
    }
    const auto t11 = sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data_dir, a11);
    const auto t15 = sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data_dir, a15);
    CHECK(t11.get() != t15.get());
    CHECK(sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data_dir, a11).get() == t11.get());
    CHECK(sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data_dir, a15).get() == t15.get());
    // Each terrain carries its own tile only.
    CHECK(t11->finest_resolution_within_m(a11.latitude_rad, a11.longitude_rad, 100.0) <= 61.0);
    CHECK(t11->finest_resolution_within_m(a15.latitude_rad, a15.longitude_rad, 100.0) > 400.0);
    CHECK(t15->finest_resolution_within_m(a15.latitude_rad, a15.longitude_rad, 100.0) <= 61.0);
    CHECK(t15->finest_resolution_within_m(a11.latitude_rad, a11.longitude_rad, 100.0) > 400.0);
    // The global-only terrain is a third instance.
    CHECK(sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data_dir).get() != t11.get());
    // A site without a tile throws and caches nothing.
    const auto none = config::lunar_site_at(STARSIM_CONFIG_DIR, 10.37 * constants::deg_to_rad, -45.13 * constants::deg_to_rad);
    CHECK_THROWS_AS(sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data_dir, none), config::DataMissing);
    CHECK_THROWS_AS(sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data_dir, none), config::DataMissing);
}

TEST_CASE("The composed Moon surface is shared per options and built from shared parts",
          "[terrain_cache][moon_surface][data][slow]") {
    const std::filesystem::path data_dir = STARSIM_DATA_DIR;
    if (!std::filesystem::exists(data_dir / "terrain" / "ldem_64.i16") ||
        !std::filesystem::exists(data_dir / "terrain" / "diviner_rock_abundance.i16")) {
        SKIP("LOLA or Diviner data not installed (fetch.py)");
    }
    // The Diviner map alone (read + DN conversion + gap fill), measured.
    const auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
    const auto t0 = std::chrono::steady_clock::now();
    const auto map = environment::load_rock_abundance_map(cfg.rocks, data_dir / "terrain");
    const double map_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    REQUIRE(map);
    std::printf("Diviner rock abundance map: %d x %d, load + gap fill %.2f s\n", map->lines, map->samples, map_s);

    const auto t1 = std::chrono::steady_clock::now();
    const auto full = sim::shared_moon_surface(STARSIM_CONFIG_DIR, data_dir, {true, true});
    const double full_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t1).count();
    std::printf("shared_moon_surface (detail + rocks, first call): %.2f s\n", full_s);
    const auto again = sim::shared_moon_surface(STARSIM_CONFIG_DIR, data_dir, {true, true});
    const auto no_rocks = sim::shared_moon_surface(STARSIM_CONFIG_DIR, data_dir, {true, false});
    const auto off = sim::shared_moon_surface(STARSIM_CONFIG_DIR, data_dir, {false, false});
    CHECK(full.get() == again.get());
    CHECK(no_rocks.get() != full.get());
    CHECK(full->rocks_on());
    CHECK(!no_rocks->rocks_on());
    CHECK(!off->detail_on());
    CHECK(no_rocks->detail_shared() == full->detail_shared());  // one detail generator
    CHECK(full->lola_shared() == sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data_dir));
    CHECK(off->lola_shared() == full->lola_shared());
}
