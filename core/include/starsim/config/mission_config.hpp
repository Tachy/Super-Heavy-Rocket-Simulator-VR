#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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

// Capsule return from orbit (config/missions/<id>.toml, d4): the start orbit, the splashdown zone and the deorbit
// planning settings (FlightSession::plan_capsule_return).
struct CapsuleReturnMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string zone_id;  // config/splashdown_zones.toml
    time::UtcDateTime epoch_utc;
    double periapsis_height_m = 0.0;  // above the equatorial radius
    double apoapsis_height_m = 0.0;
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    double argument_of_latitude_rad = 0.0;
    double rcs_propellant_kg = 0.0;  // in the capsule's RCS tank at the start
    double entry_interface_height_m = 0.0;
    double entry_flight_path_rad = 0.0;
    double nominal_bank_rad = 0.0;
    double max_crossrange_m = 0.0;
    double search_window_s = 0.0;
};

CapsuleReturnMission load_capsule_return_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

// Where a mission's drone ship waits ([drone_ship] of an ascent mission): the nominal impact point of its first stage.
struct MissionDroneShip {
    std::string id;  // config/drone_ships.toml
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double heading_rad = 0.0;   // deck long axis, clockwise from north
    double sea_height_m = 0.0;  // sea surface above WGS84 (geoid)
};

// A payload vehicle on top of an ascent mission's stack ([payload]; config::stack_payload), released after the upper
// stage's cutoff.
struct MissionPayload {
    std::string vehicle_id;
    double separation_delay_s = 30.0;  // after the upper stage's cutoff
    double separation_dv_mps = 0.3;    // spring push (relative speed)
    double nose_cone_open_s = 600.0;   // after the separation; < 0: stays closed
};

// Launch to orbit on the ascent autopilot (config/missions/<id>.toml), optionally with a first-stage return to a landing
// zone (Falcon 9 RTLS) or a drone ship (ASDS).
struct AscentMission {
    std::string id;
    std::string name;
    std::string vehicle_id;
    std::string site_id;
    std::string landing_zone_id;  // empty: no landing zone
    std::optional<MissionDroneShip> drone_ship;  // exclusive with landing_zone_id
    double heading_rad = 0.0;     // body +y on the pad
    double payload_kg = 0.0;
    bool fairing = true;          // false: the payload flies without the upper stage's fairing (Dragon)
    std::optional<MissionPayload> payload;  // a payload vehicle on top of the stack (exclusive with payload_kg)
    double periapsis_height_m = 0.0;
    double apoapsis_height_m = 0.0;
    double inclination_rad = 0.0;
    bool northerly = true;
    double kick_angle_rad = 0.0;
    std::vector<std::pair<double, double>> booster_throttle;  // {mission time s, fraction}
};

AscentMission load_ascent_mission(const std::filesystem::path& config_dir, std::string_view mission_id);

}  // namespace starsim::config
