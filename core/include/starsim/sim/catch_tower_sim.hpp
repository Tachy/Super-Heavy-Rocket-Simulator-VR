#pragma once

#include <array>
#include <map>
#include <optional>
#include <vector>

#include "starsim/contact/catch_arms.hpp"
#include "starsim/contact/catch_trigger.hpp"
#include "starsim/contact/pin_contact.hpp"
#include "starsim/contact/tower_geometry.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

enum class CatchPhase { Open, Standby, Closing, Caught, Missed, Aborted };

const char* to_string(CatchPhase phase);  // German UI text

// The catch tower of the launch site during a flight: automatic arm control, contact forces on catchable
// bodies (single stages with catch pins), tower collisions of all bodies, and the catch itself.
class CatchTowerSim {
public:
    CatchTowerSim(const config::TowerConfig& config, double latitude_rad, double longitude_rad, double height_m);

    const contact::TowerGeometry& geometry() const { return geometry_; }
    const contact::CatchArms& arms() const { return arms_; }
    CatchPhase phase() const { return phase_; }
    void close_arms(bool instantly);
    void standby_arms(bool instantly);  // half open, waiting for the booster
    // The landing guidance gave up the catch: the arms stay (or swing back) open for the rest of the flight.
    void abort_catch();
    // A body struck the tower, the mount or an arm (tower damage).
    bool tower_struck() const { return tower_struck_; }

    // Contact wrenches for the coming tick (from the current states) and the automatic tower.
    void before_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth, time::Epoch epoch);
    // Moves the arms over the tick, then judges collisions, pin overloads and the catch at its end.
    void after_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                    time::Epoch epoch_end, double dt_s);
    bool in_contact(const vehicle::FlightBody& body) const;  // pins on the arms in the last tick
    // The first tick with the pins on the arms (the catch moment, as the webcast calls it); none before.
    std::optional<time::Epoch> first_contact_epoch(const vehicle::FlightBody& body) const;
    void forget(const vehicle::FlightBody* body) { tracks_.erase(body); }  // a body leaving the session
    // Longest coast segment before a freely moving body could enter the tower zone (0: tick now).
    double coast_limit_s(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                         time::Epoch epoch) const;
    double max_pin_load_n() const { return max_pin_load_n_; }  // largest so far (diagnostics)

private:
    struct Track {
        contact::CatchBodyGeometry geometry;
        contact::CatchContact contact;
        double at_rest_s = 0.0;
        std::optional<time::Epoch> first_contact;
    };

    static bool catchable(const vehicle::FlightBody& body);
    contact::SitePose site_pose(const vehicle::FlightBody& body, const frames::FixedToInertial& t) const;
    std::array<contact::ArmContactState, 2> arm_states() const;
    Track& track(const vehicle::FlightBody& body);

    contact::TowerGeometry geometry_;
    contact::CatchArms arms_;
    contact::CatchTrigger trigger_;
    CatchPhase phase_ = CatchPhase::Open;
    std::map<const vehicle::FlightBody*, Track> tracks_;
    double hull_push_n_ = 0.0;
    double vertical_load_n_ = 0.0;
    double max_pin_load_n_ = 0.0;
    double caught_load_n_ = 0.0;
    bool tower_struck_ = false;  // weight of the bodies hanging on the arms
};

}  // namespace starsim::sim
