#pragma once

#include <limits>
#include <memory>
#include <optional>

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/environment/detailed_lunar_terrain.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::contact {

// What a surface offers below a point of the site frame (x east, y north, z up).
struct SurfaceSupport {
    double height_m = 0.0;  // surface below the point
    bool water = false;
    math::Vec3 normal{0.0, 0.0, 1.0};  // outward unit normal of the surface there, site frame
};

// Ground other than a flat plane (sloped planes in tests, lunar and Earth terrain): height and normal in a site frame.
class SurfaceField {
public:
    virtual ~SurfaceField() = default;
    virtual SurfaceSupport below(double east_m, double north_m) const = 0;
    // Support of a footpad disc of radius_m centred at (east, north) and the normal there (a field with structure under
    // the pad decides how the pad sits on it). Default: the point below the centre.
    virtual SurfaceSupport below_disc(double east_m, double north_m, double /*radius_m*/) const {
        return below(east_m, north_m);
    }
    // Height and water only (the normal is left at its default): for points that only ask whether they are below the
    // surface. Default: below.
    virtual SurfaceSupport height_below(double east_m, double north_m) const { return below(east_m, north_m); }
    // Upper bounds of below_disc(...).height_m and height_below(...).height_m (a point above them touches nothing:
    // the legs skip the query). Default: +infinity (always queried).
    virtual double below_disc_upper_bound(double /*east_m*/, double /*north_m*/, double /*radius_m*/) const {
        return std::numeric_limits<double>::infinity();
    }
    virtual double height_below_upper_bound(double /*east_m*/, double /*north_m*/) const {
        return std::numeric_limits<double>::infinity();
    }
};

// A plane through the site origin rising towards `uphill_azimuth_rad` (clockwise from north) at `slope_rad`.
class PlaneField final : public SurfaceField {
public:
    PlaneField(double slope_rad, double uphill_azimuth_rad);
    SurfaceSupport below(double east_m, double north_m) const override;

private:
    double tan_slope_;
    math::Vec3 uphill_;  // horizontal unit (east, north, 0)
    math::Vec3 normal_;
};

// A reference body's surface (shape + terrain) seen from a site frame: origin `origin_fixed_m` on the surface, columns of
// `site_to_fixed` east, north, up (body-fixed axes). The height is that of the surface point on the shape's normal
// through the site-plane point (east, north, 0) (a sphere: its radial) - it includes the drop of the curved surface
// below the plane; that point lies |(east, north)| x height / R off the vertical (< 1 mm within 100 m of the site).
// Water where the terrain says so (Earth: the sea at the geoid).
// On a DetailedLunarTerrain with detail on, below_disc (radius > 0) is its support_under_disc (the pad pivoting on the
// contact surface) and height_below its contact surface at the point (ground and rocks, contact_height_m), with the
// terrain's support_upper_bound_m as the bounds; otherwise both are below (bit-for-bit the point query of a LOLA-only
// surface) and the bounds infinite.
class TerrainField final : public SurfaceField {
public:
    // `terrain` (optional): the body's terrain, shared so the field keeps it alive (the ReferenceBody only points to it).
    TerrainField(const environment::ReferenceBody& body, const math::Vec3& origin_fixed_m, const math::Mat3& site_to_fixed,
                 std::shared_ptr<const environment::Terrain> terrain = nullptr);
    SurfaceSupport below(double east_m, double north_m) const override;
    SurfaceSupport below_disc(double east_m, double north_m, double radius_m) const override;
    SurfaceSupport height_below(double east_m, double north_m) const override;
    double below_disc_upper_bound(double east_m, double north_m, double radius_m) const override;
    double height_below_upper_bound(double east_m, double north_m) const override;

private:
    SurfaceSupport detailed_support(double east_m, double north_m, double radius_m) const;
    double site_height(double east_m, double north_m, double height_m) const;  // terrain height -> site z

    std::shared_ptr<const environment::Terrain> terrain_;
    const environment::DetailedLunarTerrain* detailed_ = nullptr;  // terrain_ when it is one with detail on
    environment::ReferenceBody body_;
    math::Vec3 origin_fixed_m_;
    math::Mat3 site_to_fixed_;
};

// What carries a landing stage, in its site frame: origin on the support surface at the site (pad centre or deck
// centre), x east, y north, z up. Flat unless a field is given.
struct LandingSurface {
    // A drone ship's deck: a rectangle at z = 0 over the sea at z = -freeboard_m. Held on station, level here; with a
    // sea state the leg landing evaluates it in the moving deck frame with the sea at the mean level (DeckSeaField).
    struct Deck {
        double half_length_m = 0.0;  // along the long axis
        double half_width_m = 0.0;
        double heading_rad = 0.0;    // long axis, clockwise from north
        double freeboard_m = 0.0;    // deck above the sea
        config::DroneShipHull hull;  // its hull's motion data (contact::DeckMotion)
    };
    std::optional<Deck> deck;              // none: a ground plane at z = 0
    std::optional<config::SeaConfig> sea;  // ground plane only: water beyond the shoreline (still at z = 0)
    // Sloped or terrain ground (lunar l2): replaces the plane, deck and sea where given.
    std::shared_ptr<const SurfaceField> field;

    using Support = SurfaceSupport;
    Support below(double east_m, double north_m) const;
    // A footpad disc: the field's below_disc; the plane, deck and sea are level, the point below the centre.
    Support below_disc(double east_m, double north_m, double radius_m) const;
    // Height and water only: the field's height_below, else below.
    Support height_below(double east_m, double north_m) const;
    // The field's upper bounds of the two (+infinity without a field: the plane, deck and sea are always queried).
    double below_disc_upper_bound(double east_m, double north_m, double radius_m) const;
    double height_below_upper_bound(double east_m, double north_m) const;
};

// A moving drone-ship deck seen from its deck-attached frame (the site frame carried along by the deck's heave, roll
// and pitch: `carrier_to_site`, origin `carrier_origin_site_m` = the deck centre): inside the deck rectangle the deck
// at z = 0 (as LandingSurface::Deck), beside it the sea at the MEAN sea level - the site-frame plane z = -freeboard,
// which does not move with the deck (spec goal 3). That plane, expressed in the carrier frame, is the height returned
// over water: (-freeboard - o_z - M_20 x - M_21 y) / M_22, normal M^T e_z (a point is below it exactly when its site z
// is below -freeboard).
class DeckSeaField final : public SurfaceField {
public:
    DeckSeaField(const LandingSurface::Deck& deck, const math::Mat3& carrier_to_site, const math::Vec3& carrier_origin_site_m);
    SurfaceSupport below(double east_m, double north_m) const override;

private:
    LandingSurface::Deck deck_;
    math::Mat3 carrier_to_site_;
    math::Vec3 carrier_origin_site_m_;
};

}  // namespace starsim::contact
