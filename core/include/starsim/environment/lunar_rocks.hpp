#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/rock_abundance_map.hpp"
#include "starsim/environment/rock_templates.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// One generated rock. Frame: local z = the local vertical, x and y horizontal, yawed at random about the vertical. The
// rock's surface is centre + local_to_fixed * (a x, b y, c z) with (x, y, z) = r(dir) dir of its template (r <= 1:
// a, b, c bound the rock). The template carries the fragment's shape (axis ratios ~2 : sqrt2 : 1, facets) and its tilt,
// so a = b = c: the rock's isotropic size.
//
// Size convention: diameter_m is the plan-view-area-equivalent diameter, the diameter of the disc with the area the
// rock covers as seen from above (the part above the ground: a b x RockTemplates::exposed_area at the burial cut).
// a follows from it per rock, so the area of every generated rock is exactly pi D^2 / 4 (the area definition of the
// Diviner rock abundance; the size-frequency counts' own convention is an open point, docs/subsystems/moon-surface.md).
struct Rock {
    std::uint64_t id = 0;
    math::Vec3 base_unit;       // where the rock stands (the vertical axis), body-fixed unit direction
    double diameter_m = 0.0;    // plan-view-area-equivalent diameter D (size-frequency diameter)
    math::Vec3 axes_m;          // a, b, c semi-axes after the template's scale (bounding; the template fills them partly)
    math::Mat3 local_to_fixed;  // orientation (body-fixed): columns a-axis, b-axis, up
    double base_height_m = 0.0; // ground (LOLA + relief) at base_unit, above the reference sphere
    double burial_m = 0.0;      // depth of the rock's centre below base_height_m: c (bottom_z + f (top_z - bottom_z)) for
                                // the burial share f of the template's height (negative: the centre stands above it)
    int template_id = 0;
    double albedo_factor = 1.0;  // relative to the regolith
};

// Deterministic rock population on top of LunarSurfaceDetail: a pure function of the body-fixed direction (integer
// hash, no state). Rocks of one diameter octave (0.01 m, doubling, to 10 m) live in the face cells of one level (centre
// inside the cell: every rock belongs to exactly one cell, found from both sides of a face edge); count Poisson from
// the hash, position uniform in the cell, D by the inverse cumulative SFD, burial, template and yaw from
// the hash. Local abundance multiplies the density: abundance_scale x (the map blended to 1 over a band at the map
// edge), evaluated at the cell centre. Thread-safe (const).
class LunarRocks {
public:
    struct Octave {
        double min_diameter_m = 0.0, max_diameter_m = 0.0;
        int level = 0;                // face-cell level (cells >= 4 x max diameter nominal)
        double density_per_m2 = 0.0;  // N(min) - N(max) at abundance 1
    };

    // abundance_source "uniform": the configured law everywhere (a passed map is ignored). "diviner": the map is
    // required (load_rock_abundance_map); a null map throws std::invalid_argument (German, names fetch.py).
    explicit LunarRocks(std::shared_ptr<const LunarSurfaceDetail> detail,
                        std::shared_ptr<const RockAbundanceMap> abundance = nullptr);

    const std::vector<Octave>& octaves() const { return octaves_; }
    // Population factor at a direction (1 = the configured law x abundance_scale outside the map or on a no-data
    // pixel; inside the map the Diviner area fraction / abundance_reference_area_fraction, smoothly blended to the
    // uniform law over abundance_blend_band_rad below the map's latitude limit).
    double abundance_factor(const math::Vec3& unit) const;
    // abundance_factor on an equirectangular grid (row 0 at +90 deg latitude, column 0 at -180 deg longitude, cell
    // centres), row-major: the renderer's abundance map.
    std::vector<float> abundance_grid(int width, int height) const;

    // Rocks standing in the cell (octaves whose level is the cell's level; empty otherwise), diameter >= min.
    std::vector<Rock> rocks_in_cell(const FaceCell& cell, double min_diameter_m) const;
    // Rocks of every octave whose base lies in the face cell `region` (any level; a terrain node of the same key),
    // diameter >= min: octaves on finer levels from all descendant cells, octaves on coarser levels from the
    // ancestor cell filtered by face_cell_of(base, region.level). The four children of a region hold exactly its
    // rocks. Sorted largest first (ties by id): the renderer draws a prefix of each list (MoonRockLayer visible counts).
    // Throws std::invalid_argument when an octave would need more than 4^max_region_descent cells (min too small for
    // the region).
    std::vector<Rock> rocks_in_region(const FaceCell& region, double min_diameter_m) const;
    static constexpr int max_region_descent = 10;  // [D] at most ~1e6 cells per octave in one query
    // Rocks whose base lies within radius_m (chord on the reference sphere) of `unit`.
    std::vector<Rock> rocks_near(const math::Vec3& unit, double radius_m, double min_diameter_m) const;
    // Rocks that may touch the disc: base within radius_m + the rock's long semi-axis a.
    std::vector<Rock> rocks_touching(const math::Vec3& unit, double radius_m, double min_diameter_m) const;

    // Top of the rock above the reference sphere along `unit` (nullopt: the vertical line through `unit` misses the
    // part of the rock above its base height) and the rock's outward unit normal there (body-fixed). The rock stands
    // on the plane tangent at base_unit; the sphere's curvature enters as d^2 / 2R. "Above" is against the rock's
    // base_height_m only, not the local ground at `unit`: on sloped or rough ground a returned top can lie below the
    // local ground, and a nullopt can hide rock above it (take max(ground, top) for heights; do not read nullopt as
    // "no rock" in a contact or hazard query).
    std::optional<std::pair<double, math::Vec3>> rock_top(const Rock& rock, const math::Vec3& unit) const;
    // rock_top with the contact normal of a flat body (a pad) with normal reference_fixed pressing on the rock there:
    // RockTemplates::normal_toward within contact_edge_tolerance_m (on a face its normal; on an edge or a vertex the
    // pad's own normal if it lies in the normal cone, else the nearest edge / face normal).
    std::optional<std::pair<double, math::Vec3>> rock_top(const Rock& rock, const math::Vec3& unit,
                                                          const math::Vec3& reference_fixed) const;
    // Faces within this distance of a contact point count as touching it ([E]: well below the pad's sinkage).
    static constexpr double contact_edge_tolerance_m = 1e-3;
    // Share of the rock height below the ground is limited to [0, 0.6] ([E]): keeps every rock visible above the
    // ground. The burial clamp is symmetric about the configured mean 0.3, so the mean share stays the configured one.
    static constexpr double max_burial_fraction = 0.6;

    // Area share of the configured rocks with diameter in [min, max] (abundance 1 x abundance_scale), clipped to the
    // law's validity range: the integral of pi D^2 / 4 over the law, which is the plan-view area of the generated
    // rocks (D is their area-equivalent diameter). For the shading.
    double cover_fraction(double min_diameter_m, double max_diameter_m) const;
    // cover_fraction(D, D_max) sampled at cover_curve_samples log-spaced diameters from the law's D_min to D_max: the
    // shader's share of the rocks smaller than the ones drawn as meshes (the renderer interpolates it).
    struct CoverCurve {
        std::vector<double> diameters_m;
        std::vector<double> cover_above;
    };
    static constexpr int cover_curve_samples = 33;  // [E] log-spaced diameters over the rock law's range (3.3 per octave)
    CoverCurve cover_curve() const;

    // Largest semi-axis a per diameter D of any rock (the deepest burial of the template with the least exposed
    // area): the reach of rocks_touching.
    double max_semi_axis_per_diameter() const { return max_semi_axis_per_diameter_; }

    // Half width of the map -> uniform blend below the map's latitude limit: 5 deg = 150 km on the Moon, several
    // map pixels, narrow against the map ([D]).
    static constexpr double abundance_blend_band_rad = 5.0 * constants::deg_to_rad;
    // Cells are at least this many maximum diameters wide ([E]); a rock reaches max_semi_axis_per_diameter x D from
    // its centre, and the queries search the cells within that reach.
    static constexpr double cell_per_diameter = 4.0;

private:
    struct Filter {
        math::Vec3 unit;
        double radius_m = 0.0;
        bool add_extent = false;  // radius_m + a
    };
    void collect(int octave, const FaceCell& cell, double min_diameter_m, const Filter* filter,
                 std::vector<Rock>& out) const;
    double cell_factor(const FaceCell& cell) const;
    double burial_fraction(std::uint64_t id) const;

    std::shared_ptr<const LunarSurfaceDetail> detail_;
    std::shared_ptr<const RockAbundanceMap> abundance_;
    std::vector<Octave> octaves_;
    double max_semi_axis_per_diameter_ = 0.0;
};

}  // namespace starsim::environment
