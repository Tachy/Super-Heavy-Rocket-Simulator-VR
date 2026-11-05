#pragma once

#include <memory>
#include <optional>
#include <utility>
#include <vector>

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
// - The contact surface (support_under_disc, contact_height_m): the ground the pads feel is LOLA + the full relief on a
//   cube-face lattice of contact_lattice_level (8 cm nominal), bilinear between the nodes, and the rocks of at least
//   contact_min_rock_diameter_m with their contact_rock_top (the template's column tops interpolated). Both scales lie
//   below the sinkage of a loaded pad in regolith (a few cm), so the pad does not feel what lies below them. The
//   lattice nodes and the rocks near the pads are cached per face cell of contact_patch_level (~10 m): the values are
//   pure functions of the cell, so a cached query is bit-identical to a fresh one.
// Thread-safe (const; the contact cache has its own lock).
class DetailedLunarTerrain final : public Terrain {
public:
    // Normal of the detailed surface by central differences over +- this distance: the HLS footpad radius, the slope
    // across a pad ([E]; LunarTerrain uses 10 m, half a 20 m pixel).
    static constexpr double detail_normal_step_m = 1.0;
    // Rocks smaller than this do not carry a pad: a 5 cm rock buried by 0.3 of its height stands ~2-3 cm proud, inside
    // the few-cm sinkage of a loaded footpad in regolith ([E]; Apollo LM footpads sank a few cm).
    static constexpr double contact_min_rock_diameter_m = 0.05;
    // Contact lattice: 2^25 nodes per face edge = R pi/2 / 2^25 = 8.1 cm at the face centre (6.8-8.1 cm elsewhere)
    // ([E]: below the few-cm sinkage of a loaded pad; the omitted sub-lattice relief has an RMS of 4.4 mm, measured at
    // the site - 2.5 mm at 4 cm, whose twice as many new nodes per slid millimetre put six sliding pads over the 50 us
    // per tick: 48-51 us against 34-36 us here).
    static constexpr int contact_lattice_level = 25;
    // Cache cells: face cells of level 18 (10.4 m nominal), 128 x 128 lattice nodes each ([E]: a pad and its rock reach
    // touch a few cells).
    static constexpr int contact_patch_level = 18;
    // [E] ~32 MB of lattice nodes; the oldest cell is dropped beyond. Sized for the parallel touchdown search (Task 5
    // fix round 2, IM-2, 1 km refused site on 22 threads): 64 cells thrashed (full sweep 100 s), 256 -> 29.5 s,
    // 1024 -> 27.5 s. The leg contact needs a few cells.
    static constexpr int max_cached_patches = 256;
    // Disc sample pattern: the centre, a ring of 8 at half the radius and a ring of 16 on the rim ([E]: the rim ring
    // finds the rim of a bowl the pad sits in; finer relief is what the max filter is meant to smooth).
    static constexpr int disc_inner_ring = 8;
    static constexpr int disc_rim_ring = 16;
    // Upper-bound slope of the contact ground over a pad radius from the pad centre ([E] 2 = 63 deg: a fresh crater wall
    // reaches 0.52 (d/D 0.13 at the rim), its rim outside 0.15, the roughness ~0.1 at the pad scale; regolith does not
    // stand steeper than ~40 deg over a metre). Only used to skip queries of feet clearly above the ground.
    static constexpr double bound_slope = 2.0;
    // |fitted plane slope| x radius <= this x (max - min of the ground samples): sqrt 2 / 2 x sum|x| / sum x^2 of the
    // pattern in units of the radius (centre, 8 at 1/2, 16 at 1: 12.46 / 9) ([D] from the pattern).
    static constexpr double bound_tilt_gain = 0.98;

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
    // height_limited_m for every direction of `units` into heights_m (same size, else std::invalid_argument), in
    // parallel over the cores left free by other running batches (detail::for_each_index_budgeted): bit for bit the
    // serial loop at any thread count. Terrain node builds call it from several worker threads at once.
    void heights_limited_m(const std::vector<math::Vec3>& units, double min_wavelength_m, double min_rock_diameter_m,
                           std::vector<double>& heights_m) const;

    // Support of a footpad disc of radius_m centred below centre_unit on its ball joint (the pad pivots onto the ground,
    // HLS detail file): the pad's plane is the least-squares plane through the contact ground at the disc's sample
    // pattern (heights above the sphere referred to the disc centre's vertical: a point d off the centre counts
    // d^2 / 2R lower; up_fixed is the vertical of the pattern), and the support is that plane's height at the centre
    // plus the highest protrusion of the contact surface above the plane within the disc (a ground sample or a rock).
    // On a plane the support is the plane's height under the centre (a point foot); a rock lifts the pad by its
    // protrusion above the fitted plane. Normal: the fitted plane's, or the carrying rock's normal where a rock protrudes
    // most. Rocks: evaluated at their peak, along the plane's downhill line from the peak inside the disc and along the
    // rim arc they cross (golden-section searches of the protrusion).
    // radius_m = 0: the contact surface at the centre - the ground and every contact-size rock covering it - and its
    // normal (the ground's over +- detail_normal_step_m, or the rock's). Detail off: height_at_fixed_m and normal_fixed
    // at the centre.
    struct DiscSupport {
        double height_m;
        math::Vec3 normal_fixed;
    };
    DiscSupport support_under_disc(const math::Vec3& centre_unit, const math::Vec3& up_fixed, double radius_m) const;
    // support_under_disc(unit, *, 0).height_m without the normal (the base ring's height). Detail off: LOLA.
    double contact_height_m(const math::Vec3& unit) const;
    // An upper bound of support_under_disc(centre_unit, up_fixed, radius_m).height_m, cheap (the contact ground at the
    // centre and the highest rock peak of the cached cells around): legs skip the disc query for a foot above it.
    // radius_m = 0: rigorous (max of the ground at the point and the rock peaks). radius_m > 0: the ground within the
    // disc is taken within bound_slope x radius_m of the centre's, which also bounds the fitted plane's tilt
    // (bound_tilt_gain). Detail off: +infinity (no skip).
    double support_upper_bound_m(const math::Vec3& centre_unit, const math::Vec3& up_fixed, double radius_m) const;
    // The contact surface's ground at a direction: LOLA + the relief interpolated on the contact lattice.
    double contact_ground_m(const math::Vec3& unit) const;
    // The contact surface's top of a rock along `unit` and its normal: LunarRocks::rock_top (the template's exact
    // column top, RockTemplates::column_top). nullopt where the vertical misses the rock above its base.
    std::optional<std::pair<double, math::Vec3>> contact_rock_top(const Rock& rock, const math::Vec3& unit) const;

    const LunarTerrain& lola() const { return *lola_; }
    // The parts; detail() / rocks() throw std::logic_error where the part was not given (null with its flag off).
    const LunarSurfaceDetail& detail() const;
    const LunarRocks& rocks() const;
    std::shared_ptr<const LunarTerrain> lola_shared() const { return lola_; }
    std::shared_ptr<const LunarSurfaceDetail> detail_shared() const { return detail_; }
    std::shared_ptr<const LunarRocks> rocks_shared() const { return rocks_; }
    const SurfaceDetailOptions& options() const { return options_; }
    bool detail_on() const { return options_.detail; }
    bool rocks_on() const { return options_.detail && options_.rocks; }

private:
    struct ContactCache;
    struct RockQuery;
    double full_height_m(const math::Vec3& unit) const;  // detail on
    // Contact ground at count directions (one pass over the cached lattice nodes; missing nodes computed after it).
    void contact_ground_batch(const math::Vec3* units, int count, double* out) const;
    // The cached rocks that may reach the disc (base within radius_m + a) and the highest peak of the cells searched.
    void gather_rocks(const math::Vec3& centre_unit, double radius_m, RockQuery& out, bool with_candidates = true) const;
    DiscSupport point_contact(const math::Vec3& unit, const math::Vec3& up_fixed, bool with_normal) const;

    std::shared_ptr<const LunarTerrain> lola_;
    std::shared_ptr<const LunarSurfaceDetail> detail_;
    std::shared_ptr<const LunarRocks> rocks_;
    SurfaceDetailOptions options_;
    double sphere_radius_m_ = 0.0;
    double max_height_m_ = 0.0;
    std::unique_ptr<ContactCache> cache_;
};

}  // namespace starsim::environment
