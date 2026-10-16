#pragma once

#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::gnc {

// Entry conditions at the interface for a corridor query (relative to the rotating Earth).
struct CorridorQuery {
    double speed_mps = 0.0;
    double interface_height_m = 120e3;
    double latitude_rad = 0.0;
    double heading_rad = 0.5 * constants::pi;  // clockwise from north
    double gate_height_m = 800.0;
    config::HeatShieldConfig limits;           // a limit of 0 is not judged
    double constraint_fraction = 0.9;          // of the heat-rate, heat-load and load limits (as the guidance)
    double aerocapture_apoapsis_m = 0.0;       // > 0: aerocapture corridor to this apoapsis height
};

struct EntryCorridor {
    bool exists = false;
    double shallow_rad = 0.0;  // overshoot boundary: full lift down still captures (direct) / reaches the apoapsis
    double steep_rad = 0.0;    // undershoot boundary: full lift up stays within the limits (and, aerocapture, exits high enough)
    double middle_rad = 0.0;
    double width_rad = 0.0;
    std::string limiting;      // German: what bounds the steep side ("Hitzestrom", "Wärmelast", "Last", "Austritt")
};

// Corridor in flight-path angle at the interface by bisection on the predictor (constant full lift down / up), in
// [-15, -0.5] deg, on a sphere of the model's radius (a 2-D question). Direct: shallow = the shallowest angle at which
// lift down does not exit; steep = the steepest at which lift up keeps the peaks within fraction x limits.
// Aerocapture: shallow = the shallowest at which lift down exits at or below the apoapsis (or stays in); steep = the
// steepest at which lift up exits at or above it within the limits. Skips are never carried (first pass only).
EntryCorridor entry_corridor(EntryPredictorModel model, const CorridorQuery& query);

// Window of the initial bank of a profiled entry (EntryPredictorModel::final_bank_rad, l5 user 2026-10-16) at one
// interface angle: the initial bank magnitudes whose prediction (the model as the guidance flies it) reaches the
// gate without a skip, within fraction x limits. A constant-bank corridor's middle can have none: lift
// down enough to stay in dives once below circular speed.
struct BankWindow {
    bool exists = false;
    double flight_path_rad = 0.0;
    double low_rad = 0.0;   // the smallest initial bank that captures (less: skips out / exits)
    double high_rad = 0.0;  // the largest within the limits (more: heat rate or load)
    double middle_rad = 0.0;
    double width_rad = 0.0;
};
BankWindow entry_bank_window(const EntryPredictorModel& model, const CorridorQuery& query, double flight_path_rad,
                             double max_bank_rad);
// The angle within `corridor` (steps of 0.1 deg) with the widest bank window; exists false if none has one.
BankWindow widest_bank_window(const EntryPredictorModel& model, const CorridorQuery& query, const EntryCorridor& corridor,
                              double max_bank_rad);

// The highest interface speed in [low, high] whose corridor is at least min_width wide (bisection, 25 m/s).
// Returns low - 1 when even `low` has none.
double max_entry_speed(const EntryPredictorModel& model, CorridorQuery query, double min_width_rad, double low_mps,
                       double high_mps);

}  // namespace starsim::gnc
