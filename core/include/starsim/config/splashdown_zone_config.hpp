#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace starsim::config {

// A capsule's recovery area at sea (config/splashdown_zones.toml).
struct SplashdownZoneConfig {
    std::string id;
    std::string name;  // German (UI)
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double radius_m = 0.0;
};

SplashdownZoneConfig load_splashdown_zone(const std::filesystem::path& config_dir, std::string_view zone_id);

}  // namespace starsim::config
