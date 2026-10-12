#pragma once

#include <filesystem>
#include <fstream>

#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

// Writes flight telemetry as CSV (SI units, angles in degrees) for plots and comparisons
// (tools/plot_telemetry.py).
class TelemetryRecorder {
public:
    explicit TelemetryRecorder(const std::filesystem::path& path);

    void record(double mission_time_s, const vehicle::FlightTelemetry& t);

private:
    std::ofstream file_;
};

}  // namespace starsim::sim
