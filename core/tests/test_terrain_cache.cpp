// The process-wide terrain cache (M8 Plan B, Task 1): one instance while held, loader errors rethrown and not cached.

#include <filesystem>
#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/sim/earth_terrain_loader.hpp"
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
