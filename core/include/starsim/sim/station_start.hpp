#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

enum class IssOrbitSource { Oem, Synthetic, Auto };  // fidelity flag iss_orbit
IssOrbitSource parse_iss_orbit_source(std::string_view text);  // "oem" | "synthetic" | "auto"; std::invalid_argument

// An impulsive velocity change of a station (an OEM reboost / debris avoidance) along its velocity.
struct StationImpulse {
    time::Epoch epoch;
    double delta_v_mps = 0.0;  // signed: + along the velocity
    std::string name;
};

struct StationStart {
    config::VehicleConfig vehicle;         // OEM header values applied (mass, drag area, C_D) when from the OEM
    math::StateVector state_gcrf;
    time::Epoch epoch;                     // of state_gcrf
    std::vector<StationImpulse> impulses;  // events with DV > 0 after the start epoch, sorted by epoch
    bool from_oem = false;
    std::string note;                      // German: which source was used (and why), shown by the mission
};

// The ISS at `epoch`. Oem: the archive file covering the epoch, else std::runtime_error (German message). Synthetic:
// mean elements of archive.latest() (or the config's reference orbit without any file), advanced by J2 in the true
// equator frame (`earth`: the Earth's orientation, see orbit/synthetic_station_orbit.hpp); the argument of latitude
// from the scenario (default: the advanced one). Auto: Oem where covered, else Synthetic (note says so).
StationStart iss_start(const config::VehicleConfig& iss, const orbit::OemArchive& archive,
                       const time::TimeScaleConverter& time, const frames::BodyOrientation& earth, time::Epoch epoch,
                       IssOrbitSource source, std::optional<double> argument_of_latitude_rad = {});

}  // namespace starsim::sim
