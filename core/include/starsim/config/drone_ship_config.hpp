#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace starsim::config {

// A drone ship's landing deck (config/drone_ships.toml). Its position is set by the mission.
struct DroneShipConfig {
    std::string id;
    std::string name;
    double deck_length_m = 0.0;
    double deck_width_m = 0.0;
    double freeboard_m = 0.0;  // deck above the sea
    std::string basin;         // "atlantic" / "pacific"
    double home_port_latitude_rad = 0.0;
    double home_port_longitude_rad = 0.0;
    // Service area: the ship is sent where a stage's predicted impact lies inside this box (longitudes in (-pi, pi]).
    double area_min_latitude_rad = 0.0;
    double area_max_latitude_rad = 0.0;
    double area_min_longitude_rad = 0.0;
    double area_max_longitude_rad = 0.0;
};

DroneShipConfig load_drone_ship(const std::filesystem::path& config_dir, std::string_view ship_id);

// All drone ships of config/drone_ships.toml, in file order.
std::vector<DroneShipConfig> load_drone_ships(const std::filesystem::path& config_dir);

// The ship whose service area contains the point; several: the nearest home port (great circle, file order on a tie);
// none: nullopt.
std::optional<DroneShipConfig> drone_ship_for(const std::vector<DroneShipConfig>& ships, double latitude_rad,
                                              double longitude_rad);

}  // namespace starsim::config
