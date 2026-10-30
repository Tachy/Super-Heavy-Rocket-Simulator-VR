#pragma once

#include <string>
#include <vector>

#include "starsim/environment/geographic_grid.hpp"
#include "starsim/environment/geoid.hpp"
#include "starsim/environment/terrain.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// A pad, tower footprint or landing zone (config/launch_sites.toml): flat at height_ellipsoid_m inside radius_m, a
// linear apron of apron_m out to the terrain around it. Never water.
struct Plateau {
    std::string site_id;
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_ellipsoid_m = 0.0;
    double radius_m = 0.0;
    double apron_m = 0.0;
};

// The Earth's surface (M8 Plan A) above the WGS84 ellipsoid: the global relief (ETOPO 2022) everywhere, a regional
// (GLO-30) and a local (3DEP 1 m) grid around the launch areas, each blended into the next coarser one over blend_m
// from its edge, and the site plateaus on top. Where the terrain lies at or below mean sea level (orthometric height
// <= water_threshold_m) the surface is water at the geoid. All grids hold WGS84 ellipsoidal heights.
class EarthTerrain final : public Terrain {
public:
    static constexpr double blend_m = 500.0;           // [E] local -> regional and regional -> global, at the edge
    // [E] orthometric <= this: water. GLO-30 sets the sea to 0 m orthometric; the regional grids store it as the
    // ellipsoidal N(EGM2008) rounded to their 0.1 m DN quantum, so a sea node reads within +-0.05 m (half the quantum)
    // orthometric. Measured 2026-10-23 on the real tiles (200k random nodes each, the same EGM2008 grid as the core):
    // 97 % / 99 % of the near-sea nodes of starbase / cape regional lie within +-0.05 m, with no pile-up just above it
    // (the 0.05-0.055 m bin holds ~0.03 % - low land, as dense as its neighbours). Bilinear interpolation between sea
    // nodes stays inside that band (the geoid is near-linear over a 1" cell). Raising it would only turn land a few
    // centimetres above mean sea level into water.
    static constexpr double water_threshold_m = 0.05;
    static constexpr double normal_step_m = 0.5;       // [E] central differences for the normal: half the 1 m spacing

    EarthTerrain(Geoid geoid, GeographicGrid global_relief, std::vector<GeographicGrid> regional,
                 std::vector<GeographicGrid> local, std::vector<Plateau> plateaus);

    // The surface (sea: the geoid) above the WGS84 ellipsoid at a geodetic latitude / longitude.
    double height_ellipsoid_m(double latitude_rad, double longitude_rad) const;
    double height_m(double latitude_rad, double longitude_rad) const override { return height_ellipsoid_m(latitude_rad, longitude_rad); }
    bool water(double latitude_rad, double longitude_rad) const override;
    // Latitude step of the finest grid covering the point (local, regional, global relief) in metres.
    double resolution_m(double latitude_rad, double longitude_rad) const override;
    // An ellipsoidal-height bound without scanning the grids (source: constants::earth_terrain_max_height_m).
    double max_height_m() const override { return constants::earth_terrain_max_height_m; }
    const Geoid& geoid() const { return geoid_; }
    const std::vector<Plateau>& plateaus() const { return plateaus_; }

    // Terrain: the WGS84 geodetic latitude / longitude of the point.
    double height_at_fixed_m(const math::Vec3& fixed_m) const override;
    math::Vec3 normal_fixed(const math::Vec3& fixed_m) const override;
    bool water_at_fixed(const math::Vec3& fixed_m) const override;

private:
    struct PlateauHit {
        const Plateau* plateau = nullptr;  // nullptr: no plateau there
        double apron_fraction = 0.0;       // 0 inside the radius .. 1 at the apron's outer edge
    };
    struct Ground {
        double height_ellipsoid_m = 0.0;  // the surface without plateaus (sea: the geoid)
        bool water = false;
    };

    double terrain_m(double latitude_rad, double longitude_rad) const;  // grids only (sea floor included)
    Ground ground(double latitude_rad, double longitude_rad) const;
    PlateauHit plateau_at(double latitude_rad, double longitude_rad) const;
    math::Vec3 surface_point(double latitude_rad, double longitude_rad) const;

    Geoid geoid_;
    GeographicGrid global_;
    std::vector<GeographicGrid> regional_;
    std::vector<GeographicGrid> local_;
    std::vector<Plateau> plateaus_;
    std::vector<math::Vec3> plateau_centres_fixed_m_;  // on the ellipsoid (height 0)
};

}  // namespace starsim::environment
