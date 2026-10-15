#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Heights above a reference sphere on a regular grid, stored as the LOLA products store them (int16 x scale).
struct HeightGrid {
    int lines = 0;
    int samples = 0;
    double dn_scale_m = 0.0;
    std::vector<std::int16_t> dn;  // row-major, lines x samples

    double at(int line, int sample) const {
        return dn_scale_m * dn[static_cast<size_t>(line) * static_cast<size_t>(samples) + static_cast<size_t>(sample)];
    }
    // Bilinear inside [0, lines-1] x [0, samples-1] (nodes exactly).
    double bilinear(double line, double sample) const;
};

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
};

// Terrain of the Moon (lunar l2): the global grid everywhere, the site tiles where present, blended over blend_m at a
// tile's edge (no step where the tile ends). Heights above the sphere; positions in body-fixed (MOON_ME) coordinates.
class LunarTerrain {
public:
    static constexpr double blend_m = 1000.0;      // [E] tile -> global grid
    static constexpr double normal_step_m = 10.0;  // central differences for the normal (half a 20 m pixel)

    LunarTerrain(CylindricalGrid global, std::vector<PolarStereographicTile> tiles, double sphere_radius_m);

    double height_m(double latitude_rad, double longitude_rad) const;
    double height_at_fixed_m(const math::Vec3& fixed_m) const;  // below the point's direction
    math::Vec3 normal_fixed(const math::Vec3& fixed_m) const;   // outward unit normal of the surface there
    bool in_tile(double latitude_rad, double longitude_rad) const;
    double sphere_radius_m() const { return sphere_radius_m_; }

private:
    math::Vec3 surface_point(double latitude_rad, double longitude_rad) const;

    CylindricalGrid global_;
    std::vector<PolarStereographicTile> tiles_;
    double sphere_radius_m_;
};

}  // namespace starsim::environment
