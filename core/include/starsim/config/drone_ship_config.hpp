#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace starsim::config {

// The barge hull as a box for the deck motion (contact::DeckMotion): waterplane rectangle, draft, radii of gyration
// about the long axis (roll) and the beam (pitch), added mass / inertia as factors of the rigid-body value, linear
// damping ratios of the three modes.
struct DroneShipHull {
    double length_m = 0.0;
    double beam_m = 0.0;
    double draft_m = 0.0;  // hull depth - freeboard [D]
    double roll_gyration_m = 0.0;
    double pitch_gyration_m = 0.0;
    double heave_added_mass_factor = 0.0;
    double roll_added_inertia_factor = 0.0;
    double pitch_added_inertia_factor = 0.0;
    double heave_damping_ratio = 0.0;
    double roll_damping_ratio = 0.0;
    double pitch_damping_ratio = 0.0;
};

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
    DroneShipHull hull;
    // Sea-state limits of a landing: significant (2 sigma) deck tilt and heave rate.
    double max_deck_tilt_rad = 0.0;
    double max_heave_rate_mps = 0.0;
};

DroneShipConfig load_drone_ship(const std::filesystem::path& config_dir, std::string_view ship_id);

// All drone ships of config/drone_ships.toml, in file order.
std::vector<DroneShipConfig> load_drone_ships(const std::filesystem::path& config_dir);

// The ship whose service area contains the point; several: the nearest home port (great circle, file order on a tie);
// none: nullopt.
std::optional<DroneShipConfig> drone_ship_for(const std::vector<DroneShipConfig>& ships, double latitude_rad,
                                              double longitude_rad);

}  // namespace starsim::config
