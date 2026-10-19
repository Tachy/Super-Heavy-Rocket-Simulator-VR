#pragma once

#include <optional>
#include <string>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/docking_mechanism.hpp"
#include "starsim/contact/station_collision.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

// Docking of a visiting vehicle at a station port (ISS i5) on the session tick, in the pattern of CatchTowerSim:
// before_tick runs the mechanism and the hull contact and applies their wrenches to both bodies - equal and opposite
// forces; the station's torque is dropped (it flies an ideal attitude, documented simplification).
class DockingSim {
public:
    DockingSim(std::string chaser_port, std::string station_port, const config::VehicleConfig& chaser_config,
               const config::VehicleConfig& station_config, contact::DockingMechanismParameters p = {});

    void before_tick(vehicle::FlightBody& chaser, vehicle::FlightBody& station, time::Epoch epoch, double dt_s);

    // How long the chaser may coast without the hull or the docking interface reaching the station: the clearance (CG
    // to the nearest primitive minus the hull's reach; port to port minus the petal reach) over the relative speed in
    // the station's rotating frame (plus the relative turn times the reach). Infinity when docked or out of range.
    double coast_limit_s(const vehicle::FlightBody& chaser, const vehicle::FlightBody& station) const;

    // The relative pose the mechanism sees (chaser port in the passive port frame).
    contact::DockingPose pose(const vehicle::FlightBody& chaser, const vehicle::FlightBody& station) const;

    contact::DockingMechanism& mechanism() { return mechanism_; }
    const contact::DockingMechanism& mechanism() const { return mechanism_; }
    bool hull_contact() const { return hull_contact_; }  // this tick
    std::optional<double> collision_impulse_ns() const { return collision_impulse_ns_; }  // from the first hull contact
    math::Vec3 wrench_balance_n() const { return balance_n_; }  // sum of the forces applied this tick (GCRF)
    math::Vec3 force_on_chaser_n() const { return last_force_n_; }  // this tick, GCRF
    const std::string& chaser_port() const { return chaser_port_; }
    const std::string& station_port() const { return station_port_; }
    const contact::StationCollisionParameters& collision_parameters() const { return collision_p_; }

private:
    std::string chaser_port_, station_port_;
    contact::DockingMechanism mechanism_;
    contact::StationCollisionParameters collision_p_;  // before hull_body_m_: it is built from them
    contact::StationShape shape_;
    std::vector<math::Vec3> hull_body_m_;  // chaser vehicle frame
    std::optional<double> collision_impulse_ns_;
    bool hull_contact_ = false;
    math::Vec3 balance_n_, last_force_n_;
    std::optional<time::Epoch> t0_;
};

}  // namespace starsim::sim
