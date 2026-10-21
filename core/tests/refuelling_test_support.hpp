#pragma once
// Refuelling r3 tests: the depot as a station in its 200 km / 26.5 deg orbit and a tanker (or HLS) near it.
#include <algorithm>
#include <filesystem>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/frames/lvlh.hpp"
#include "starsim/gnc/orbital_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/docking_port.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::refuelling_test {

using math::Vec3;

inline const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
constexpr double oxidizer_fuel_ratio = 3.6;  // Block 3 main tanks [D]

// The depot orbit of the lunar missions' start (spec r3): 200 km circular, 26.5 deg, RAAN 0 (true equator frame);
// `height_m` other than 200 km for the reboost tests (f2).
inline math::StateVector depot_state(const frames::BodyOrientation& earth, time::Epoch epoch,
                                     double argument_of_latitude_rad = 0.0, double height_m = 200.0e3) {
    orbit::StationMeanElements m;
    m.epoch = epoch;
    m.semi_major_axis_m = constants::wgs84_semi_major_axis_m + height_m;
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

// The chaser (ship alone) with its transfer_dock at `position_port_m` in the depot's transfer_dock frame, in the
// docking attitude (axes (-x, -y, +z) of the depot port's: rolled 180 deg, dorsal up), moving with the depot's
// rotating frame plus `velocity_port_mps`. Body 0 = chaser, body 1 = depot.
struct TankerAtPort : DepotSession {
    TankerAtPort(const Vec3& position_port_m, const Vec3& velocity_port_mps = {},
                 const config::VehicleConfig& chaser = tanker_config(200.0e3), double depot_kg = 600.0e3) {
        depot_propellant_kg = depot_kg;
        sim::FlightSession probe(ephemeris, earth, config_dir);  // the ideal-attitude depot's port
        probe.add_station(depot_config(depot_propellant_kg), depot, t0);
        const auto sk = vehicle::port_kinematics(probe.body(0), "transfer_dock");
        const vehicle::Vehicle v(chaser, config_dir);
        const auto& port = chaser.stages.back().docking_ports.front();
        const math::Mat3 port_axes_body = math::Mat3::from_columns(port.axis, cross(port.up, port.axis), port.up);
        const math::Mat3 desired = sk.axes_gcrf * math::Mat3::from_columns({-1, 0, 0}, {0, -1, 0}, {0, 0, 1});
        const math::Mat3 r = desired * transpose(port_axes_body);
        const Vec3 lever_body = port.position_m - v.mass_properties().cg_point_m();
        const Vec3 port_gcrf = sk.position_gcrf_m + sk.axes_gcrf * position_port_m;
        const Vec3 cg = port_gcrf - r * lever_body;
        const Vec3 omega = sk.angular_velocity_gcrf_radps;
        const Vec3 velocity =
            sk.velocity_gcrf_mps + sk.axes_gcrf * velocity_port_mps + cross(omega, cg - sk.position_gcrf_m);
        const math::Quat q = math::Quat::from_matrix(r);
        session.place_body(chaser, {cg, velocity, q, q.inverse_rotate(omega)}, t0);
        session.add_station(depot_config(depot_propellant_kg), depot, t0);
        session.set_attitude_mode(gnc::lvlh_hold(gnc::lvlh_attitude_gcrf({cg, velocity}).conjugate() * q));
    }
    // Advances until `done` or `limit_s`; returns the elapsed seconds.
    template <class Done>
    double fly(Done done, double limit_s, double step_s = 1.0) {
        double t = 0.0;
        while (t < limit_s && !done()) {
            session.advance_to(session.epoch() + step_s);
            t += step_s;
        }
        return t;
    }
};

}  // namespace starsim::refuelling_test
