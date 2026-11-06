#include "starsim/sim/terrain_cache.hpp"

#include <map>
#include <mutex>
#include <string>
#include <tuple>
#include <utility>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/environment/rock_abundance_map.hpp"
#include "starsim/sim/earth_terrain_loader.hpp"
#include "starsim/sim/lunar_site.hpp"

namespace starsim::sim {

namespace {

// (config_dir, data_dir, height tile key of the active site; empty = no site, global grid only)
using Key = std::tuple<std::filesystem::path, std::filesystem::path, std::string>;

Key key_of(const std::filesystem::path& config_dir, const std::filesystem::path& data_dir, std::string tile_key = {}) {
    return {std::filesystem::weakly_canonical(config_dir), std::filesystem::weakly_canonical(data_dir),
            std::move(tile_key)};
}

// Weak cache of one terrain type. The lock is held while loading: concurrent callers of one key wait for the first
// load and then share it (a second load of the same data would cost the memory this cache saves).
template <typename T, typename Loader>
std::shared_ptr<const T> cached(std::map<Key, std::weak_ptr<const T>>& cache, std::mutex& mutex,
                                const std::filesystem::path& config_dir, const std::filesystem::path& data_dir,
                                Loader load) {
    const Key key = key_of(config_dir, data_dir);
    std::lock_guard lock(mutex);
    if (auto held = cache[key].lock()) return held;
    auto loaded = load(config_dir, data_dir);  // throws: nothing cached
    cache[key] = loaded;
    return loaded;
}

}  // namespace

std::shared_ptr<const environment::EarthTerrain> shared_earth_terrain(const std::filesystem::path& config_dir,
                                                                      const std::filesystem::path& data_dir) {
    static std::mutex mutex;
    static std::map<Key, std::weak_ptr<const environment::EarthTerrain>> cache;
    return cached(cache, mutex, config_dir, data_dir, load_earth_terrain);
}

std::shared_ptr<const environment::LunarTerrain> shared_lunar_terrain(const std::filesystem::path& config_dir,
                                                                      const std::filesystem::path& data_dir) {
    static std::mutex mutex;
    static std::map<Key, std::weak_ptr<const environment::LunarTerrain>> cache;
    return cached(cache, mutex, config_dir, data_dir,
                  [](const std::filesystem::path& c, const std::filesystem::path& d) { return load_lunar_terrain(c, d); });
}

std::shared_ptr<const environment::LunarTerrain> shared_lunar_terrain(const std::filesystem::path& config_dir,
                                                                      const std::filesystem::path& data_dir,
                                                                      const config::LunarSiteConfig& site) {
    static std::mutex mutex;
    static std::map<Key, std::weak_ptr<const environment::LunarTerrain>> cache;
    const Key key = key_of(config_dir, data_dir, site.height_tile_key);
    std::lock_guard lock(mutex);
    if (auto held = cache[key].lock()) return held;
    auto loaded = load_lunar_terrain(config_dir, data_dir, site);  // throws: nothing cached
    cache[key] = loaded;
    return loaded;
}

namespace {

// The shared body of both shared_moon_surface overloads: `lola` is the (shared) LOLA terrain of the key.
std::shared_ptr<const environment::DetailedLunarTerrain> compose_surface(
    const std::filesystem::path& config_dir, const std::filesystem::path& data_dir,
    environment::SurfaceDetailOptions options, const Key& key, std::shared_ptr<const environment::LunarTerrain> lola) {
    static std::mutex mutex;
    static std::map<Key, std::weak_ptr<const environment::LunarSurfaceDetail>> details;
    static std::map<Key, std::weak_ptr<const environment::LunarRocks>> rocks;
    static std::map<Key, std::weak_ptr<const environment::RockAbundanceMap>> abundance_maps;  // site-independent
    static std::map<std::pair<Key, std::pair<bool, bool>>, std::weak_ptr<const environment::DetailedLunarTerrain>> surfaces;
    std::lock_guard lock(mutex);
    auto& surface_slot = surfaces[{key, {options.detail, options.rocks}}];
    if (auto held = surface_slot.lock()) return held;
    std::shared_ptr<const environment::LunarSurfaceDetail> detail;
    std::shared_ptr<const environment::LunarRocks> rock_layer;
    if (options.detail) {
        detail = details[key].lock();
        if (!detail) {
            detail = std::make_shared<const environment::LunarSurfaceDetail>(
                config::load_moon_surface_config(config_dir / "moon_surface.toml"), lola);
            details[key] = detail;
        }
        if (options.rocks) {
            rock_layer = rocks[key].lock();
            if (!rock_layer) {
                // The Diviner map does not depend on the site: one copy for every site's rock layer.
                auto& map_slot = abundance_maps[key_of(config_dir, data_dir)];
                auto map = map_slot.lock();
                if (!map) {
                    map = environment::load_rock_abundance_map(detail->config().rocks, data_dir / "terrain");
                    map_slot = map;
                }
                rock_layer = std::make_shared<const environment::LunarRocks>(detail, std::move(map));
                rocks[key] = rock_layer;
            }
        }
    }
    auto surface = std::make_shared<const environment::DetailedLunarTerrain>(lola, detail, rock_layer, options);
    surface_slot = surface;
    return surface;
}

}  // namespace

std::shared_ptr<const environment::DetailedLunarTerrain> shared_moon_surface(const std::filesystem::path& config_dir,
                                                                            const std::filesystem::path& data_dir,
                                                                            environment::SurfaceDetailOptions options) {
    auto lola = shared_lunar_terrain(config_dir, data_dir);  // own lock; throws: nothing cached
    return compose_surface(config_dir, data_dir, options, key_of(config_dir, data_dir), std::move(lola));
}

std::shared_ptr<const environment::DetailedLunarTerrain> shared_moon_surface(const std::filesystem::path& config_dir,
                                                                            const std::filesystem::path& data_dir,
                                                                            environment::SurfaceDetailOptions options,
                                                                            const config::LunarSiteConfig& site) {
    auto lola = shared_lunar_terrain(config_dir, data_dir, site);
    return compose_surface(config_dir, data_dir, options, key_of(config_dir, data_dir, site.height_tile_key),
                           std::move(lola));
}

}  // namespace starsim::sim
