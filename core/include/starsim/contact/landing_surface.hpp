#pragma once

#include <memory>
#include <optional>

#include "starsim/config/launch_site_config.hpp"
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

// Ground other than a flat plane (sloped planes in tests, lunar terrain): height and normal in a site frame.
class SurfaceField {
public:
    virtual ~SurfaceField() = default;
    virtual SurfaceSupport below(double east_m, double north_m) const = 0;
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
// `site_to_fixed` east, north, up (body-fixed axes). The height is that of the surface point on the radial through the
// site-plane point (east, north, 0) - it includes the drop of the curved surface below the plane; that point lies
// |(east, north)| x height / R off the vertical (< 1 mm within 100 m of the site).
class TerrainField final : public SurfaceField {
public:
    TerrainField(const environment::ReferenceBody& body, const math::Vec3& origin_fixed_m, const math::Mat3& site_to_fixed);
    SurfaceSupport below(double east_m, double north_m) const override;

private:
    environment::ReferenceBody body_;
    math::Vec3 origin_fixed_m_;
    math::Mat3 site_to_fixed_;
};

// What carries a landing stage, in its site frame: origin on the support surface at the site (pad centre or deck
// centre), x east, y north, z up. Flat unless a field is given.
struct LandingSurface {
    // A drone ship's deck: a rectangle at z = 0 over the sea at z = -freeboard_m. Held on station, level (no sea state).
    struct Deck {
        double half_length_m = 0.0;  // along the long axis
        double half_width_m = 0.0;
        double heading_rad = 0.0;    // long axis, clockwise from north
        double freeboard_m = 0.0;    // deck above the sea
    };
    std::optional<Deck> deck;              // none: a ground plane at z = 0
    std::optional<config::SeaConfig> sea;  // ground plane only: water beyond the shoreline (still at z = 0)
    // Sloped or terrain ground (lunar l2): replaces the plane, deck and sea where given.
    std::shared_ptr<const SurfaceField> field;

    using Support = SurfaceSupport;
    Support below(double east_m, double north_m) const;
};

}  // namespace starsim::contact
