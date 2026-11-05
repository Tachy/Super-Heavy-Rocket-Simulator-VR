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

// One generated rock. Frame: local x = longest axis (a), y = b, z = c = the local vertical (the rock rests on its
// flattest side, [E]), yawed at random about the vertical. The rock's surface is centre + local_to_fixed *
// (a x, b y, c z) with (x, y, z) = r(dir) dir of its template (r <= 1: a, b, c bound the rock).
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
// the hash, position uniform in the cell, D by the inverse cumulative SFD, shape ratios, burial, template and yaw from
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

    // Area share of the configured rocks with diameter in [min, max] (abundance 1 x abundance_scale), clipped to the
    // law's validity range: the integral of pi D^2 / 4 over the law, which is the plan-view area of the generated
    // rocks (D is their area-equivalent diameter). For the shading.
    double cover_fraction(double min_diameter_m, double max_diameter_m) const;

    // Largest long semi-axis a per diameter D of any rock (the thinnest b / a and the deepest burial of the least
    // filled template): the reach of rocks_touching.
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
    double b_to_a(std::uint64_t id) const;

    std::shared_ptr<const LunarSurfaceDetail> detail_;
    std::shared_ptr<const RockAbundanceMap> abundance_;
    std::vector<Octave> octaves_;
    double max_semi_axis_per_diameter_ = 0.0;
};

}  // namespace starsim::environment
