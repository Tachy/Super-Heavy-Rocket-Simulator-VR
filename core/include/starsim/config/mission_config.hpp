#pragma once

#include <filesystem>
#include <string>
#include <string_view>

#include "starsim/time/utc_date_time.hpp"

namespace starsim::config {

// Ship return from orbit (config/missions/<id>.toml): the scenario's start orbit and the deorbit planning settings.
struct ShipReturnMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string stage_id;  // the ship's stage (stages below it are left out)
    std::string site_id;
    time::UtcDateTime epoch_utc;
    double periapsis_height_m = 0.0;  // above the equatorial radius
    double apoapsis_height_m = 0.0;
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    double argument_of_latitude_rad = 0.0;
    double propellant_kg = 0.0;  // in orbit
    double entry_interface_height_m = 0.0;
    double entry_flight_path_rad = 0.0;
    double nominal_bank_rad = 0.0;
    double max_crossrange_m = 0.0;
    double search_window_s = 0.0;
};

ShipReturnMission load_ship_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

}  // namespace starsim::config
