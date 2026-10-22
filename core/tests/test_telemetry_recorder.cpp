// TelemetryRecorder: one CSV row per sample, with the Earth / Moon distances and speeds (external view Task 4c).

#include <filesystem>
#include <fstream>
#include <string>

#include <catch2/catch_test_macros.hpp>

#include "starsim/sim/telemetry_recorder.hpp"

using namespace starsim;

TEST_CASE("The telemetry CSV carries the Earth and Moon distances and speeds", "[telemetry_recorder]") {
    const auto path = std::filesystem::temp_directory_path() / "starsim_telemetry_recorder_earth_moon.csv";
    vehicle::FlightTelemetry t;
    t.speed_inertial_mps = 1633.5;
    t.speed_earth_mps = 1950.25;
    t.distance_earth_m = 4.0e8;
    t.distance_moon_m = 1.8374e6;
    t.speed_moon_mps = 1633.5;
    {
        sim::TelemetryRecorder recorder(path);
        recorder.record(12.5, t);
    }
    std::ifstream in(path);
    std::string header, row;
    std::getline(in, header);
    std::getline(in, row);
    in.close();
    std::filesystem::remove(path);
    CHECK(header.ends_with(",engines_running,speed_earth_mps,distance_earth_m,distance_moon_m,speed_moon_mps"));
    CHECK(row.ends_with(",0,1950.25,4e+08,1.8374e+06,1633.5"));
}
