#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "starsim/environment/height_grid.hpp"
#include "starsim/environment/lunar_tile.hpp"
#include "starsim/environment/terrain.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Terrain of the Moon (lunar l2): the global grid everywhere, the site tiles where present, blended over blend_m at a
// tile's edge (no step where the tile ends). Heights above the sphere; positions in body-fixed (MOON_ME) coordinates.
class LunarTerrain final : public Terrain {
public:
    static constexpr double blend_m = 1000.0;      // [E] tile -> global grid
    static constexpr double normal_step_m = 10.0;  // central differences for the normal (half a 20 m pixel)

    LunarTerrain(CylindricalGrid global, std::vector<std::shared_ptr<const LunarTile>> tiles, double sphere_radius_m);
    // South polar crops (the site tiles of lunar_sites.toml).
    LunarTerrain(CylindricalGrid global, std::vector<PolarStereographicTile> south_tiles, double sphere_radius_m);

    double height_m(double latitude_rad, double longitude_rad) const override;
    // Same value as height_m; the tiles are located without trigonometry, the global grid only when needed.
    double height_at_direction_m(const math::Vec3& unit) const override;
    // A site tile's pixel scale inside a tile, the global grid's spacing elsewhere.
    double resolution_m(double latitude_rad, double longitude_rad) const override;
    // A site tile's pixel scale when the tile lies within radius_m of the point, else resolution_m.
    double finest_resolution_within_m(double latitude_rad, double longitude_rad, double radius_m) const override;
    // The highest sample of the global grid and the tiles (the blend never exceeds its two inputs).
    double max_height_m() const override { return max_height_m_; }
    double height_at_fixed_m(const math::Vec3& fixed_m) const override;  // below the point's direction
    math::Vec3 normal_fixed(const math::Vec3& fixed_m) const override;  // outward unit normal of the surface there
    bool in_tile(double latitude_rad, double longitude_rad) const;
    // The tile blend at a direction, as used inside height_at_direction_m: weight 1 in a tile's core, falling to 0 at
    // its edge (smoothstep over blend_m), 0 on the global grid; scale_m is that tile's pixel scale (0 outside tiles).
    struct TileBlend {
        double weight = 0.0;
        double scale_m = 0.0;
    };
    TileBlend tile_blend(const math::Vec3& unit) const;
    double tile_weight(const math::Vec3& unit) const { return tile_blend(unit).weight; }
    double global_resolution_m() const;  // the global grid's spacing at the equator
    double finest_resolution_m() const;  // the smallest of the tiles' pixel scales and the global spacing
    double sphere_radius_m() const { return sphere_radius_m_; }

private:
    math::Vec3 surface_point(double latitude_rad, double longitude_rad) const;

    CylindricalGrid global_;
    std::vector<std::shared_ptr<const LunarTile>> tiles_;
    double sphere_radius_m_;
    double max_height_m_ = 0.0;
};

}  // namespace starsim::environment
