#pragma once
// Refuelling r3 tests: the depot as a station in its 200 km / 26.5 deg orbit and a tanker (or HLS) near it.
#include <algorithm>
#include <filesystem>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::refuelling_test {

inline const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
constexpr double oxidizer_fuel_ratio = 3.6;  // Block 3 main tanks [D]

// The depot orbit of the lunar missions' start (spec r3): 200 km circular, 26.5 deg, RAAN 0 (true equator frame).
inline math::StateVector depot_state(const frames::BodyOrientation& earth, time::Epoch epoch,
                                     double argument_of_latitude_rad = 0.0) {
    orbit::StationMeanElements m;
    m.epoch = epoch;
    m.semi_major_axis_m = constants::wgs84_semi_major_axis_m + 200.0e3;
    m.inclination_rad = 26.5 * constants::deg_to_rad;
    m.argument_of_latitude_rad = argument_of_latitude_rad;
    return orbit::synthetic_station_state(m, constants::earth_gm_m3ps2, earth);
}

// A ship alone with `main_propellant_kg` in its main tanks (O/F 3.6); headers as configured.
inline config::VehicleConfig ship_with(const std::string& vehicle_id, double main_propellant_kg) {
    auto cfg = config::top_stage_config(config::load_vehicle(config_dir, vehicle_id));
    for (auto& tank : cfg.stages.back().tanks) {
        if (tank.role != config::TankRole::Main) continue;
        const double share = tank.contents == config::TankContents::Oxidizer
                                 ? oxidizer_fuel_ratio / (1.0 + oxidizer_fuel_ratio)
                                 : 1.0 / (1.0 + oxidizer_fuel_ratio);
        tank.mass_kg = std::min(tank.capacity_kg, share * main_propellant_kg);
    }
    return cfg;
}
inline config::VehicleConfig depot_config(double main_propellant_kg) {
    return ship_with("starship_depot", main_propellant_kg);
}
inline config::VehicleConfig tanker_config(double main_propellant_kg) {
    return ship_with("starship_tanker", main_propellant_kg);
}

// A session with the depot (station) at t0 = 2026-10-20 00:00 UTC; the chaser is placed by the test.
struct DepotSession {
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceTimeConverter clock;  // not "time": that would hide the namespace for time::Epoch below
    sim::FlightSession session{ephemeris, earth, config_dir};
    time::Epoch t0 = clock.from_utc({2026, 10, 20, 0, 0, 0.0});
    math::StateVector depot = depot_state(earth, t0);
    double depot_propellant_kg = 600.0e3;  // [test] a depot after a few tankers
};

}  // namespace starsim::refuelling_test
