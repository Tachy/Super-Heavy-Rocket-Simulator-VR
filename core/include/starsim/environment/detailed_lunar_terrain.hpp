#pragma once

#include <memory>

#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/terrain.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Fidelity flags moon_surface_detail / moon_rocks. Rocks need the detail (they stand on its relief): rocks without
// detail are off.
struct SurfaceDetailOptions {
    bool detail = true;
    bool rocks = true;
    friend constexpr bool operator==(const SurfaceDetailOptions&, const SurfaceDetailOptions&) = default;
};

// The Moon's physics surface: LOLA + the procedural craters and roughness (LunarSurfaceDetail) + the rock tops
// (LunarRocks), max(ground, rock tops) - rocks are star-shaped, so there are no overhangs.
//
// Detail off: every Terrain override forwards to LunarTerrain (bit-for-bit today's surface, incl. the 10 m normal
// step); support_under_disc is then the point query at the disc centre (point footpads).
//
// Detail on:
// - Terrain point queries are the full detail: LOLA (height_at_direction_m) + relief_m(unit, 0) and every rock whose
//   top covers the point; normal_fixed by central differences over normal_step_m (the footpad scale);
//   resolution_m / finest_resolution_within_m report the finest generated wavelength.
// - The contact surface (support_under_disc): the ground the pads feel is LOLA + the full relief on a cube-face lattice
//   of contact_lattice_spacing (4 cm nominal), bilinear between the nodes, and the rocks of at least
//   contact_min_rock_diameter_m. Both scales lie below the sinkage of a loaded pad in regolith (a few cm), so the
//   pad does not feel what lies below them. The lattice nodes and the rocks near the pads are cached per face cell of
//   contact_patch_level (~10 m): the values are pure functions of the cell, so a cached query is bit-identical to a
//   fresh one.
// Thread-safe (const; the contact cache has its own lock).
class DetailedLunarTerrain final : public Terrain {
public:
    // Normal of the detailed surface by central differences over +- this distance: the HLS footpad radius, the slope
    // across a pad ([E]; LunarTerrain uses 10 m, half a 20 m pixel).
    static constexpr double detail_normal_step_m = 1.0;
    // Rocks smaller than this do not carry a pad: a 5 cm rock buried by 0.3 of its height stands ~2-3 cm proud, inside
    // the few-cm sinkage of a loaded footpad in regolith ([E]; Apollo LM footpads sank a few cm).
    static constexpr double contact_min_rock_diameter_m = 0.05;
    // Contact lattice: 2^26 nodes per face edge = R pi/2 / 2^26 = 4.07 cm at the face centre (3.4-4.1 cm elsewhere)
    // ([E]: below the pad sinkage; the omitted sub-lattice relief has an RMS of ~2 mm).
    static constexpr int contact_lattice_level = 26;
    // Cache cells: face cells of level 18 (10.4 m nominal), 256 x 256 lattice nodes each ([E]: a pad and its rock reach
    // touch a few cells).
    static constexpr int contact_patch_level = 18;
    static constexpr int max_cached_patches = 64;  // [E] ~32 MB of lattice nodes; the oldest cell is dropped beyond
    // Disc sample pattern: the centre, a ring of 8 at half the radius and a ring of 16 on the rim ([E]: the rim ring
    // finds the rim of a bowl the pad sits in; finer relief is what the max filter is meant to smooth).
    static constexpr int disc_inner_ring = 8;
    static constexpr int disc_rim_ring = 16;

    // Throws std::invalid_argument for a null lola, for detail on without a detail generator, or for rocks on (with
    // detail) without rocks. Detail / rocks may be null where their flag is off.
    DetailedLunarTerrain(std::shared_ptr<const LunarTerrain> lola, std::shared_ptr<const LunarSurfaceDetail> detail,
                         std::shared_ptr<const LunarRocks> rocks, SurfaceDetailOptions options);
    ~DetailedLunarTerrain() override;

    double height_at_fixed_m(const math::Vec3& fixed_m) const override;
    math::Vec3 normal_fixed(const math::Vec3& fixed_m) const override;
    double height_m(double latitude_rad, double longitude_rad) const override;
    double height_at_direction_m(const math::Vec3& unit) const override;
    double resolution_m(double latitude_rad, double longitude_rad) const override;
    double finest_resolution_within_m(double latitude_rad, double longitude_rad, double radius_m) const override;
    double max_height_m() const override { return max_height_m_; }

    // Central-difference step of normal_fixed: detail_normal_step_m with detail, LunarTerrain::normal_step_m without.
    double normal_step_m(const math::Vec3& fixed_m) const;

    // Height with the relief averaged over min_wavelength_m (LunarSurfaceDetail::relief_m) and only the rocks of at
    // least min_rock_diameter_m: max(LOLA + relief, their tops). (0, 0) is height_at_direction_m. Detail off: LOLA.
    double height_limited_m(const math::Vec3& unit, double min_wavelength_m, double min_rock_diameter_m) const;

    // Support of a flat footpad disc of radius_m centred below centre_unit, the disc normal to up_fixed: the highest
    // point of the contact surface under the disc (heights above the sphere, referred to the disc centre's vertical:
    // a point d off the centre counts d^2 / 2R lower) and the normal there - a rock's own normal when a rock is the
    // highest point, else the normal of the least-squares plane through the disc's ground samples (the pad-scale
    // slope; in a bowl the pad sits on the rim all round and that normal is the bowl's axis).
    // Ground: the max over the sample pattern; a rock whose peak lies under the disc counts with its peak, a rock cut
    // by the rim with the highest point of the rim arc over it (search along the arc). radius_m = 0: the contact surface
    // at the centre. Detail off: height_at_fixed_m and normal_fixed at the centre.
    struct DiscSupport {
        double height_m;
        math::Vec3 normal_fixed;
    };
    DiscSupport support_under_disc(const math::Vec3& centre_unit, const math::Vec3& up_fixed, double radius_m) const;
    // The contact surface's ground at a direction: LOLA + the relief interpolated on the contact lattice.
    double contact_ground_m(const math::Vec3& unit) const;

    const LunarTerrain& lola() const { return *lola_; }
    const LunarSurfaceDetail& detail() const { return *detail_; }  // detail on only
    const LunarRocks& rocks() const { return *rocks_; }            // rocks on only
    std::shared_ptr<const LunarTerrain> lola_shared() const { return lola_; }
    std::shared_ptr<const LunarSurfaceDetail> detail_shared() const { return detail_; }
    std::shared_ptr<const LunarRocks> rocks_shared() const { return rocks_; }
    const SurfaceDetailOptions& options() const { return options_; }
    bool detail_on() const { return options_.detail; }
    bool rocks_on() const { return options_.detail && options_.rocks; }

private:
    struct ContactCache;
    double full_height_m(const math::Vec3& unit) const;  // detail on

    std::shared_ptr<const LunarTerrain> lola_;
    std::shared_ptr<const LunarSurfaceDetail> detail_;
    std::shared_ptr<const LunarRocks> rocks_;
    SurfaceDetailOptions options_;
    double sphere_radius_m_ = 0.0;
    double max_height_m_ = 0.0;
    std::unique_ptr<ContactCache> cache_;
};

}  // namespace starsim::environment
