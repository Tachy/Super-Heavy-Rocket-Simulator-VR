#include "starsim/sim/telemetry_recorder.hpp"

#include <stdexcept>

#include "starsim/math/constants.hpp"

namespace starsim::sim {

TelemetryRecorder::TelemetryRecorder(const std::filesystem::path& path) : file_(path) {
    if (!file_) {
        throw std::runtime_error("cannot write telemetry to " + path.string());
    }
    file_ << "time_s,altitude_m,latitude_deg,longitude_deg,speed_inertial_mps,speed_relative_mps,"
             "vertical_speed_mps,flight_path_angle_deg,mach,dynamic_pressure_pa,angle_of_attack_deg,mass_kg,"
             "propellant_kg,thrust_n,sensed_acceleration_g,engines_running,speed_earth_mps,distance_earth_m,"
             "distance_moon_m,speed_moon_mps\n";  // speed_inertial_mps: about the reference body; Moon columns nan
                                                  // without a Moon in the session
}

void TelemetryRecorder::record(double mission_time_s, const vehicle::FlightTelemetry& t) {
    const double deg = constants::rad_to_deg;
    file_ << mission_time_s << ',' << t.altitude_m << ',' << t.latitude_rad * deg << ',' << t.longitude_rad * deg << ','
          << t.speed_inertial_mps << ',' << t.speed_relative_mps << ',' << t.vertical_speed_mps << ','
          << t.flight_path_angle_rad * deg << ',' << t.mach << ',' << t.dynamic_pressure_pa << ','
          << t.angle_of_attack_rad * deg << ',' << t.mass_kg << ',' << t.propellant_kg << ',' << t.thrust_n << ','
          << t.sensed_acceleration_g << ',' << t.engines_running << ',' << t.speed_earth_mps << ',' << t.distance_earth_m
          << ',' << t.distance_moon_m << ',' << t.speed_moon_mps << '\n';
}

}  // namespace starsim::sim
