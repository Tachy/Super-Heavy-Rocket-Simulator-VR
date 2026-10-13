#pragma once

#include <map>
#include <vector>

#include "starsim/contact/landing_legs.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

// How a stage's touchdown on its legs ended.
enum class LegTouchdown { None, Landed, TippedOver, Impact, Splashdown };

const char* to_string(LegTouchdown touchdown);  // German UI text ("gelandet", "umgekippt", "Aufprall", "Wasserung")

// Landing on legs at one site (a landing zone's pad, or a drone ship's deck): contact forces of the deployed feet on
// every body with legs, and the judgement at the end of each tick (the pattern of CatchTowerSim). Bodies whose legs
// are moving or out leave their ground contact to this simulation (FlightBody::resolve_ground_contact).
class LegLandingSim {
public:
    // Site frame: origin on the support surface at (latitude, longitude, height above WGS84), x east, y north, z up.
    // `tick_s`: the session's tick (friction is limited to what stops a foot's slip within one tick).
    LegLandingSim(double latitude_rad, double longitude_rad, double height_m, contact::LandingSurface surface,
                  double tick_s);

    const contact::LandingSurface& surface() const { return surface_; }
    void clear() { tracks_.clear(); }  // the session replaced its bodies
    // Contact wrenches for the coming tick from the current states.
    void before_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth, time::Epoch epoch);
    // Judges each body with legs out at the end of the tick.
    void after_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                    time::Epoch epoch_end, double dt_s);

    LegTouchdown touchdown(const vehicle::FlightBody& body) const;
    bool in_contact(const vehicle::FlightBody& body) const;          // any foot in the last tick
    std::vector<double> crushed_m(const vehicle::FlightBody& body) const;  // crush stroke used per foot

    // Site frame of an Earth-fixed position, and the pose of a body in it.
    math::Vec3 to_site(const math::Vec3& itrf_m) const;
    contact::SitePose site_pose(const vehicle::FlightBody& body, const frames::FixedToInertial& t) const;

private:
    struct Track {
        contact::LegGeometry geometry;
        std::vector<double> crushed_m;
        contact::LegContact contact;
        double at_rest_s = 0.0;
        LegTouchdown touchdown = LegTouchdown::None;
    };
    Track& track(const vehicle::FlightBody& body);

    math::Vec3 origin_itrf_m_;
    math::Mat3 site_to_itrf_ = math::Mat3::identity();  // columns: east, north, up
    contact::LandingSurface surface_;
    std::map<const vehicle::FlightBody*, Track> tracks_;
    double tick_s_ = 0.0;
};

}  // namespace starsim::sim
