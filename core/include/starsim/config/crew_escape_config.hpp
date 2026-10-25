#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "starsim/math/vec3.hpp"

namespace starsim::config {

// One sample of a thrust-time table (piecewise linear between samples, 0 outside the table).
struct ThrustSample {
    double time_s = 0.0;
    double thrust_n = 0.0;
};

// A seat in the ship's stage frame (x station from the stage bottom, y/z body axes).
struct SeatPosition {
    math::Vec3 position_stage_m;
};

// Crew escape of a crewed ship return (config/crew_escape/<id>.toml; docs/superpowers/archive/specs/2026-10-22-crew-escape-
// design.md): the seats, their rails, envelope, deployment timing, give-up rules and the judging limits. The ship's own
// data stay untouched (user 2026-10-22).
struct CrewEscapeConfig {
    std::string id;
    std::string seat_vehicle_id;         // seat + occupant until the seat-man separation (config/vehicles/)
    std::string parachutist_vehicle_id;  // occupant under the main canopy
    int crew_count = 0;                  // seats.size()
    std::vector<SeatPosition> seats;     // firing order
    math::Vec3 rail_direction_body;      // unit, ship body axes
    double rail_length_m = 0.0;          // the seat clears the hatch after this travel
    std::vector<ThrustSample> catapult;  // along the rail, time from the seat's firing
    std::vector<ThrustSample> rocket;
    double stagger_s = 0.0;              // between consecutive seats
    double envelope_max_height_m = 0.0;  // above the ground
    double envelope_max_eas_mps = 0.0;   // equivalent airspeed
    double mode1_max_eas_mps = 0.0;      // mode 1 below this and below mode_low_height_m
    double mode_low_height_m = 0.0;      // above: mode 3 (drogue held until here)
    double mode1_main_delay_s = 0.0;
    double mode2_drogue_delay_s = 0.0;
    double mode2_separation_delay_s = 0.0;  // the main follows on the separation
    double min_firing_height_m = 0.0;    // an early give-up fires over water, at the latest here
    double sea_target_margin_m = 0.0;    // a given-up ship aims this far beyond the shoreline
    double min_catch_propellant_kg = 0.0;
    double max_load_g = 0.0;             // sustained longer than max_load_duration_s: lost
    double max_load_duration_s = 0.0;
    double min_clearance_m = 0.0;        // to the ship hull, its plume and the tower
    double plume_half_angle_rad = 0.0;
    double plume_length_m = 0.0;
    double max_touchdown_sink_mps = 0.0;
};

// Throws ConfigError on a missing / non-finite / negative value, an empty seat list, an unknown vehicle id or a thrust
// table whose times do not ascend.
CrewEscapeConfig load_crew_escape(const std::filesystem::path& config_dir, std::string_view id);

// Thrust of a table at `t_s` (linear between samples, 0 before the first and after the last).
double thrust_at(const std::vector<ThrustSample>& table, double t_s);

}  // namespace starsim::config
