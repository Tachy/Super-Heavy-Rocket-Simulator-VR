#include "starsim/sim/terrain_cache.hpp"

#include <map>
#include <mutex>
#include <utility>

#include "starsim/sim/earth_terrain_loader.hpp"
#include "starsim/sim/lunar_site.hpp"

namespace starsim::sim {

namespace {

using Key = std::pair<std::filesystem::path, std::filesystem::path>;

Key key_of(const std::filesystem::path& config_dir, const std::filesystem::path& data_dir) {
    return {std::filesystem::weakly_canonical(config_dir), std::filesystem::weakly_canonical(data_dir)};
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
    return cached(cache, mutex, config_dir, data_dir, load_lunar_terrain);
}

}  // namespace starsim::sim
