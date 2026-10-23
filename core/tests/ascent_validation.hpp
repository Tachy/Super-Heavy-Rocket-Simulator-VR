#pragma once

// Ascent validation against recorded flights (ascent trajectory and staging, Task 6): a mission flown through the
// onboard computer's plan (sim::start_ascent_mission) and the webcast telemetry tables (tests/validation/telemetry)
// it is compared with.

#include <cmath>
#include <string>
#include <vector>

#include "starsim/sim/ascent_profile_planner.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::test {

// The simulated flight, sampled on mission time (T+0 = the clamps' release).
struct FlownAscent {
    struct Sample {
        double t_s = 0.0, altitude_km = 0.0, speed_kmh = 0.0, gamma_deg = 0.0, mass_t = 0.0, thrust_mn = 0.0;
        double sensed_g = 0.0, q_kpa = 0.0, mach = 0.0;
    };
    sim::AscentProfile plan;
    std::vector<Sample> samples;   // the controlled stack (after the separation: the upper stage), every sample_step_s
    Sample meco;                   // the last tick of the booster phase (state Booster)
    double seco_s = -1.0;          // the upper stage's cutoff (UpperStage -> Orbit)
    Sample seco;
    double booster_meco_propellant_t = 0.0;
    double upper_propellant_at_seco_t = 0.0;
    double periapsis_km = 0.0, apoapsis_km = 0.0, inclination_deg = 0.0;  // the orbit after the tail-off

    // Linear interpolation of the samples at mission time t (speed, altitude, flight-path angle).
    Sample at(double t_s) const;
};

// Flies the ascent mission `mission_id` (config/missions) from `liftoff`: planned, then to the upper stage's cutoff
// and its tail-off. The booster's return is ended at the separation (it does not act on the upper stage).
// STARSIM_ASCENT_DUMP=<dir>: writes the samples to <dir>/<mission_id>.csv.
FlownAscent fly_validation_ascent(const std::string& mission_id, time::Epoch liftoff);

// One recorded reading of the stack (booster columns of a telemetry table; the stack's until the separation):
// speed (km/h) from a least-squares line over +-1 s of readings, altitude (km) over +-2 s.
struct RecordedReading {
    double t_s = 0.0, speed_kmh = 0.0, altitude_km = 0.0;
    double speed_rate_kmh_s = 0.0, altitude_rate_km_s = 0.0;  // the local slopes (time-base tolerance)
};
RecordedReading recorded_stack(const std::string& table, double t_s, const std::string& vehicle = "booster");

// Reading accuracy of the tables (README of tests/validation/telemetry): speed shown to 1 km/h, altitude to 0.1 km, each
// reading stamped within +-0.25 s; the time base (liftoff_time.py) known to +-0.25 s (four tables: 3.24-3.50 s).
inline constexpr double speed_resolution_kmh = 1.0;
inline constexpr double altitude_resolution_km = 0.05;
inline constexpr double reading_time_s = 0.25;
inline constexpr double time_base_uncertainty_s = 0.25;

// One compared quantity: |simulated - recorded| <= reading_tolerance + allowance * |recorded|. The allowance is a
// documented open model discrepancy (docs/subsystems.md "Ascent trajectory and staging"), 0 where the model follows the
// flight within the readings.
struct Comparison {
    std::string what;
    double recorded = 0.0, simulated = 0.0, reading_tolerance = 0.0, allowance = 0.0;
    double bound() const { return reading_tolerance + allowance * std::abs(recorded); }
    bool within() const { return std::abs(simulated - recorded) <= bound(); }
};
void print(const std::string& flight, const Comparison& c);

// The stack's speed (altitude) at display time `t_s` of `table` against the simulation at t_s - time_base_s (the dial
// shows the state time_base_s late: the speed and the altitude dial have their own, liftoff_time.py), with the reading
// tolerances from the local rates.
Comparison compare_speed(const std::string& table, const FlownAscent& flown, double t_s, double time_base_s,
                         double allowance);
Comparison compare_altitude(const std::string& table, const FlownAscent& flown, double t_s, double time_base_s,
                            double allowance);

}  // namespace starsim::test
