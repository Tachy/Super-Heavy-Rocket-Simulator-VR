#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "starsim/config/vehicle_config.hpp"

namespace starsim::config {

// Catch tower of a launch site ([site.tower] in config/launch_sites.toml). Lengths in the site frame: origin
// at the pad centre on the ground, east/north/up.
struct TowerConfig {
    double azimuth_rad = 0.0;           // pad centre -> tower centre, clockwise from north
    double distance_m = 0.0;            // pad centre -> tower centre, horizontal
    double width_m = 0.0;               // square cross-section
    double height_m = 0.0;
    double mount_height_m = 0.0;        // top of the launch mount: a stack on the pad stands here
    double mount_outer_radius_m = 0.0;
    double mount_inner_radius_m = 0.0;
    double arm_length_m = 0.0;          // hinge -> tip
    double arm_width_m = 0.0;
    double arm_thickness_m = 0.0;
    double arm_open_rad = 0.0;          // each arm swung outward from closed (parallel arms)
    double arm_standby_rad = 0.0;       // waiting opening before the catch (arms close from here at the end)
    double arm_closed_gap_m = 0.0;      // clear gap between the closed arms
    double arm_rate_rad_s = 0.0;
    double arm_force_limit_n = 0.0;     // closing stalls above this push of the hull
    double catch_height_m = 0.0;        // arm top surface (carriage at rest) above the pad ground
    double carriage_mass_kg = 0.0;
    double carriage_stiffness_n_m = 0.0;
    double carriage_damping_n_s_m = 0.0;
    double carriage_stroke_m = 0.0;
};

// Coastline of a launch site ([site.sea]): a straight shoreline perpendicular to `direction_rad`; the ground beyond
// it is water (aborted landings end there as a splashdown).
struct SeaConfig {
    double direction_rad = 0.0;          // pad centre -> sea, clockwise from north
    double shoreline_distance_m = 0.0;   // pad centre -> shoreline along that direction
};

struct LaunchSiteConfig {
    std::string id;
    std::string name;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;  // ground (pad) height above WGS84
    std::optional<TowerConfig> tower;
    std::optional<SeaConfig> sea;
};

// Reads site `site_id` from config/launch_sites.toml.
LaunchSiteConfig load_launch_site(const std::filesystem::path& config_dir, std::string_view site_id);

}  // namespace starsim::config
