#pragma once

#include <utility>
#include <vector>

#include "starsim/environment/height_grid.hpp"
#include "starsim/environment/terrain.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// A crop of a south polar stereographic product (LOLA LDEM_80S_*; dsmap_polar.cat [S]): rho = 2 R tan(pi/4 + lat/2),
// X = rho sin(lon), Y = -rho cos(lon), line = pole_line + Y / scale, sample = pole_sample + X / scale (0-based, full
// product); the crop starts at (first_line, first_sample).
struct PolarStereographicTile {
    HeightGrid grid;
    double sphere_radius_m = 0.0;
    double scale_m = 0.0;  // metres per pixel at the pole
    double pole_line = 0.0;
    double pole_sample = 0.0;
    int first_line = 0;
    int first_sample = 0;

    // Grid position (line, sample) in the crop of a latitude / longitude.
    std::pair<double, double> position(double latitude_rad, double longitude_rad) const;
    // The same by the unit direction (no trigonometry: X = 2 R y / (1 - z), Y = -2 R x / (1 - z)); (inf, inf) at the
    // north pole, which no south polar crop contains.
    std::pair<double, double> position_of_direction(const math::Vec3& unit) const;
    bool inside(double line, double sample) const;
    double edge_distance_px(double line, double sample) const;  // to the crop's nearest edge
};

// A global simple-cylindrical grid (LOLA LDEM_16): line = zero_line - ppd lat_deg, sample = zero_sample + ppd lon_deg
// with the longitude east in [0, 360).
struct CylindricalGrid {
    HeightGrid grid;
    double pixels_per_degree = 0.0;
    double zero_line = 0.0;
    double zero_sample = 0.0;

    double height_m(double latitude_rad, double longitude_rad) const;  // bilinear; longitude wraps, poles clamped
    // The same by the unit direction (fast_atan2 / fast_asin instead of the library's: 2e-8 rad, 3 cm).
    double height_at_direction(const math::Vec3& unit) const;
    // Bilinear at grid position (line clamped, sample wrapping).
    double height_at_grid(double line, double sample) const;
};

// Terrain of the Moon (lunar l2): the global grid everywhere, the site tiles where present, blended over blend_m at a
// tile's edge (no step where the tile ends). Heights above the sphere; positions in body-fixed (MOON_ME) coordinates.
class LunarTerrain final : public Terrain {
public:
    static constexpr double blend_m = 1000.0;      // [E] tile -> global grid
    static constexpr double normal_step_m = 10.0;  // central differences for the normal (half a 20 m pixel)

    LunarTerrain(CylindricalGrid global, std::vector<PolarStereographicTile> tiles, double sphere_radius_m);

    double height_m(double latitude_rad, double longitude_rad) const override;
    // Same value as height_m; the tiles are located without trigonometry, the global grid only when needed.
    double height_at_direction_m(const math::Vec3& unit) const override;
    // A site tile's pixel scale inside a tile, the global grid's spacing elsewhere.
    double resolution_m(double latitude_rad, double longitude_rad) const override;
    // A site tile's pixel scale when the tile (its rectangle) lies within radius_m of the point, else resolution_m.
    double finest_resolution_within_m(double latitude_rad, double longitude_rad, double radius_m) const override;
    // The highest sample of the global grid and the tiles (the blend never exceeds its two inputs).
    double max_height_m() const override { return max_height_m_; }
    double height_at_fixed_m(const math::Vec3& fixed_m) const override;  // below the point's direction
    math::Vec3 normal_fixed(const math::Vec3& fixed_m) const override;  // outward unit normal of the surface there
    bool in_tile(double latitude_rad, double longitude_rad) const;
    double sphere_radius_m() const { return sphere_radius_m_; }

private:
    math::Vec3 surface_point(double latitude_rad, double longitude_rad) const;

    CylindricalGrid global_;
    std::vector<PolarStereographicTile> tiles_;
    double sphere_radius_m_;
    double max_height_m_ = 0.0;
};

}  // namespace starsim::environment
