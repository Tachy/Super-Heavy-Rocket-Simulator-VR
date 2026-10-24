#pragma once

#include <limits>
#include <map>
#include <vector>

#include "starsim/contact/landing_legs.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::sim {

// How a stage's touchdown on its legs ended.
enum class LegTouchdown { None, Landed, TippedOver, Impact, Splashdown };

const char* to_string(LegTouchdown touchdown);  // German UI text ("gelandet", "umgekippt", "Aufprall", "Wasserung")

// Where a leg landing simulation sits (lunar l2: any reference body): a point on the body's shape (Earth: WGS84 geodetic
// height; Moon: the height above the sphere, i.e. the terrain height there) with the site frame east / north / up.
struct LegSite {
    ephemeris::Body body = ephemeris::Body::Earth;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    // Bodies farther from the site (horizontally) keep their own ground contact (FlightBody). A lunar site and an Earth
    // site on terrain set it; an Earth site's flat plane or deck reaches everywhere, as before l2.
    double tracking_radius_m = std::numeric_limits<double>::infinity();
};

// Landing on legs at one site (a landing zone's pad, a drone ship's deck, a lunar site): contact forces of the deployed
// feet on every body with legs, and the judgement at the end of each tick (the pattern of CatchTowerSim). Bodies whose
// legs are moving or out leave their ground contact to this simulation while they are on its site
// (FlightBody::set_legs_on_site).
class LegLandingSim {
public:
    struct SiteFrame {
        math::Vec3 origin_fixed_m;
        math::Mat3 site_to_fixed = math::Mat3::identity();  // columns: east, north, up
    };
    static SiteFrame site_frame(const LegSite& site, const frames::Ellipsoid& shape);

    // `shape`: the site body's shape. `tick_s`: the session's tick (friction is limited to what stops a foot's slip
    // within one tick).
    LegLandingSim(const LegSite& site, const frames::Ellipsoid& shape, contact::LandingSurface surface, double tick_s);
    // An Earth site on WGS84 (latitude, longitude, height above the ellipsoid).
    LegLandingSim(double latitude_rad, double longitude_rad, double height_m, contact::LandingSurface surface,
                  double tick_s);

    const LegSite& site() const { return site_; }
    const contact::LandingSurface& surface() const { return surface_; }
    const math::Vec3& origin_fixed_m() const { return frame_.origin_fixed_m; }
    const math::Mat3& site_to_fixed() const { return frame_.site_to_fixed; }
    void clear() { tracks_.clear(); }  // the session replaced its bodies
    void forget(const vehicle::FlightBody* body) { tracks_.erase(body); }  // the session removed this body
    // Contact wrenches for the coming tick from the current states.
    void before_tick(const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env,
                     time::Epoch epoch);
    // Judges each body with legs out on the site at the end of the tick.
    void after_tick(const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env,
                    time::Epoch epoch_end, double dt_s);

    LegTouchdown touchdown(const vehicle::FlightBody& body) const;
    // The body lifts off: its judgement starts over (the crush strokes stay).
    void release(const vehicle::FlightBody& body);
    bool in_contact(const vehicle::FlightBody& body) const;          // any foot in the last tick
    std::vector<double> crushed_m(const vehicle::FlightBody& body) const;  // crush stroke used per foot

    // Site frame of a body-fixed position, and the pose of a body in it.
    math::Vec3 to_site(const math::Vec3& fixed_m) const;
    contact::SitePose site_pose(const vehicle::FlightBody& body, const environment::BodyFrame& frame) const;

private:
    struct Track {
        contact::LegGeometry geometry;
        std::vector<double> crushed_m;
        contact::LegContact contact;
        double at_rest_s = 0.0;
        LegTouchdown touchdown = LegTouchdown::None;
        bool on_site = true;
    };
    Track& track(const vehicle::FlightBody& body);

    LegSite site_;
    SiteFrame frame_;
    contact::LandingSurface surface_;
    std::map<const vehicle::FlightBody*, Track> tracks_;
    double tick_s_ = 0.0;
};

}  // namespace starsim::sim
