#pragma once

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "starsim/contact/deck_motion.hpp"
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

    // The deck of a drone-ship site moves with the sea (sea state, Task 3): contact and judgement are evaluated in the
    // moving deck frame (the deck rectangle of LandingSurface::Deck; the water beside it at the mean sea level,
    // contact::DeckSeaField), and a landed stage rides the deck (FlightBody::move_landed_hold). `sea_origin`: the
    // sea's time origin (DeckMotion time = epoch - sea_origin). nullptr: the fixed deck (exactly as without). Needs a
    // deck surface. For a site without landed stages; change_deck_motion with stages landed.
    void set_deck_motion(std::shared_ptr<const contact::DeckMotion> motion, time::Epoch sea_origin);
    // Changes the motion under stages already landed on the deck, without a jump: each is first held fixed where it is
    // at `epoch` (its state then), then a new motion takes it along from its pose relative to the deck at `epoch`
    // (nullptr: it stays fixed to the Earth there).
    void change_deck_motion(std::shared_ptr<const contact::DeckMotion> motion, time::Epoch sea_origin,
                            const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env,
                            time::Epoch epoch);
    const contact::DeckMotion* deck_motion() const { return deck_motion_.get(); }
    std::optional<contact::DeckPose> deck_pose(time::Epoch epoch) const;  // nullopt: no deck motion
    // Moves every stage landed on the moving deck to the deck's pose at `epoch` (after_tick does it each tick; the
    // session calls it after a coast). Nothing without deck motion.
    void follow_deck(const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env,
                     time::Epoch epoch);

private:
    struct Track {
        contact::LegGeometry geometry;
        std::vector<double> crushed_m;
        contact::LegContact contact;
        double at_rest_s = 0.0;
        LegTouchdown touchdown = LegTouchdown::None;
        bool on_site = true;
        // Landed on the moving deck: bottom point and attitude in the deck-attached site frame (Carrier), fixed.
        std::optional<math::Vec3> held_bottom_m;
        math::Mat3 held_body_to_deck = math::Mat3::identity();
    };
    Track& track(const vehicle::FlightBody& body);

    // The deck-attached site frame at an epoch: the site frame carried along by the deck's heave, roll and pitch
    // (identity and zero on a level deck at rest, so the deck stays the rectangle at z = 0 with its heading).
    struct Carrier {
        math::Mat3 to_site = math::Mat3::identity();  // carrier axes -> site axes
        math::Vec3 origin_site_m;
        math::Vec3 velocity_site_mps;
        math::Vec3 angular_velocity_site_radps;
    };
    Carrier carrier(time::Epoch epoch) const;
    static contact::SitePose relative_pose(const contact::SitePose& site_pose, const Carrier& carrier);

    LegSite site_;
    SiteFrame frame_;
    contact::LandingSurface surface_;
    std::map<const vehicle::FlightBody*, Track> tracks_;
    double tick_s_ = 0.0;
    std::shared_ptr<const contact::DeckMotion> deck_motion_;
    time::Epoch sea_origin_;
    // The carrier of the last epoch asked for: after_tick(t), follow_deck(t) and before_tick(t) of the next tick share it.
    // Cleared whenever the motion changes. The sim runs on one thread (the session's).
    mutable std::optional<std::pair<double, Carrier>> carrier_cache_;
};

}  // namespace starsim::sim
