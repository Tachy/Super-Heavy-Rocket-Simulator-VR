#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace starsim::config {

// A drone ship's landing deck (config/drone_ships.toml). Its position is set by the mission.
struct DroneShipConfig {
    std::string id;
    std::string name;
    double deck_length_m = 0.0;
    double deck_width_m = 0.0;
    double freeboard_m = 0.0;  // deck above the sea
};

DroneShipConfig load_drone_ship(const std::filesystem::path& config_dir, std::string_view ship_id);

}  // namespace starsim::config
