#pragma once

#include <memory>
#include <utility>

#include "starsim/environment/height_grid.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// A crop of a polar stereographic product (LOLA LDEM_*; dsmap_polar.cat [S]), rho = 2 R tan(pi/4 -+ lat/2).
// South: X = rho sin(lon), Y = -rho cos(lon): line = pole_line + Y / scale, sample = pole_sample + X / scale.
// North (PDS label convention, checked against LDEM_16 for LDEM_60N_60M): line = pole_line + rho cos(lon) / scale,
// sample = pole_sample + rho sin(lon) / scale. 0-based, full product; the crop starts at (first_line, first_sample).
struct PolarStereographicTile {
    HeightGrid grid;
    double sphere_radius_m = 0.0;
    double scale_m = 0.0;  // metres per pixel at the pole
    double pole_line = 0.0;
    double pole_sample = 0.0;
    int first_line = 0;
    int first_sample = 0;
    bool north = false;  // the hemisphere (the projection's pole): false = south

    // Grid position (line, sample) in the crop of a latitude / longitude.
    std::pair<double, double> position(double latitude_rad, double longitude_rad) const;
    // The same by the unit direction (no trigonometry: south X = 2 R y / (1 - z), Y = -2 R x / (1 - z)); (inf, inf) at
    // the projection's opposite pole.
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

// One high-resolution patch on the sphere (immutable). Directions are unit vectors in the body-fixed frame (MOON_ME).
class LunarTile {
public:
    virtual ~LunarTile() = default;
    virtual bool contains(const math::Vec3& unit_moon_me) const = 0;
    virtual double height_m(const math::Vec3& unit_moon_me) const = 0;         // bilinear, above the sphere
    virtual double edge_distance_m(const math::Vec3& unit_moon_me) const = 0;  // to the nearest edge; > 0 inside
    virtual double scale_m() const = 0;                                        // pixel size (the finer axis)
    // Approximate true distance from the direction to the tile (0 inside, +inf where the projection is meaningless).
    virtual double outside_distance_m(const math::Vec3& unit_moon_me) const = 0;
    virtual double max_height_m() const = 0;  // the highest sample
};

std::shared_ptr<const LunarTile> make_polar_tile(PolarStereographicTile tile, bool north);
// A simple-cylindrical patch: pixel (0, 0) is centred at (first_latitude_rad, first_longitude_rad), lines run south,
// samples east (the longitude wraps over 0/360); the grid's zero_line / zero_sample are not used.
std::shared_ptr<const LunarTile> make_cylindrical_tile(CylindricalGrid grid, double first_latitude_rad,
                                                       double first_longitude_rad, double sphere_radius_m);

}  // namespace starsim::environment
