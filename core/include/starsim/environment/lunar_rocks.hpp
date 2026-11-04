#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/rock_templates.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// One generated rock. Frame: local x = longest axis (a), y = b, z = c = the local vertical (the rock rests on its
// flattest side, [E]), yawed at random about the vertical. The rock's surface is centre + local_to_fixed *
// (a x, b y, c z) with (x, y, z) = r(dir) dir of its template (r <= 1: a, b, c bound the rock).
struct Rock {
    std::uint64_t id = 0;
    math::Vec3 base_unit;       // where the rock stands (the vertical axis), body-fixed unit direction
    double diameter_m = 0.0;    // size-frequency diameter D = 2 a (bounding long axis)
    math::Vec3 axes_m;          // a, b, c semi-axes after the template's scale
    math::Mat3 local_to_fixed;  // orientation (body-fixed): columns a-axis, b-axis, up
    double base_height_m = 0.0; // ground (LOLA + relief) at base_unit, above the reference sphere
    double burial_m = 0.0;      // depth of the rock's centre below base_height_m ((2 f - 1) c for burial share f of the
                                // height 2 c; negative: the centre stands above the ground)
    int template_id = 0;
    double albedo_factor = 1.0;  // relative to the regolith
};

// Local rock abundance map: area fraction of the rocks Diviner sees (diameter >= abundance_reference_min_diameter_m)
// on a simple-cylindrical grid over |latitude| <= lat_limit_rad (the Diviner GDR L3 rock abundance product covers
// 60 S - 60 N). Line 0 is the northern edge, sample 0 longitude 0 (east positive); pixel centres at half steps.
struct RockAbundanceMap {
    int lines = 0, samples = 0;
    double lat_limit_rad = 0.0;
    std::vector<float> area_fraction;  // row-major, lines x samples
    double at(double latitude_rad, double longitude_rad) const;  // bilinear, longitude wraps
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

    // The map is optional (null: the uniform law everywhere, e.g. while the Diviner product is not installed).
    explicit LunarRocks(std::shared_ptr<const LunarSurfaceDetail> detail,
                        std::shared_ptr<const RockAbundanceMap> abundance = nullptr);

    const std::vector<Octave>& octaves() const { return octaves_; }
    // Population factor at a direction (1 = the configured law x abundance_scale outside the map; inside the map
    // the Diviner area fraction / abundance_reference_area_fraction, smoothly blended to the uniform law over
    // abundance_blend_band_rad below the map's latitude limit).
    double abundance_factor(const math::Vec3& unit) const;

    // Rocks standing in the cell (octaves whose level is the cell's level; empty otherwise), diameter >= min.
    std::vector<Rock> rocks_in_cell(const FaceCell& cell, double min_diameter_m) const;
    // Rocks whose base lies within radius_m (chord on the reference sphere) of `unit`.
    std::vector<Rock> rocks_near(const math::Vec3& unit, double radius_m, double min_diameter_m) const;
    // Rocks that may touch the disc: base within radius_m + the rock's long semi-axis a.
    std::vector<Rock> rocks_touching(const math::Vec3& unit, double radius_m, double min_diameter_m) const;

    // Top of the rock above the reference sphere along `unit` (nullopt: the vertical line through `unit` misses the
    // part of the rock above its base height) and the rock's outward unit normal there (body-fixed). The rock stands
    // on the plane tangent at base_unit; the sphere's curvature enters as d^2 / 2R.
    std::optional<std::pair<double, math::Vec3>> rock_top(const Rock& rock, const math::Vec3& unit) const;

    // Area share of the configured rocks with diameter in [min, max] (disc of diameter D each, the population's own
    // definition; abundance 1 x abundance_scale), clipped to the law's validity range. For the shading.
    double cover_fraction(double min_diameter_m, double max_diameter_m) const;

    // Half width of the map -> uniform blend below the map's latitude limit: 5 deg = 150 km on the Moon, several
    // map pixels, narrow against the map ([D]).
    static constexpr double abundance_blend_band_rad = 5.0 * constants::deg_to_rad;
    // Cells are at least this many maximum diameters wide: a rock reaches at most D/2 from its centre and cells are
    // >= 0.7 nominal wide everywhere, so 4 keeps one octave's rocks within the cells next to a point ([E]).
    static constexpr double cell_per_diameter = 4.0;
    // Steps over the rock height when scanning for its top from above ([E]: the templates are smooth, no thin spikes).
    static constexpr int top_scan_steps = 48;

private:
    struct Filter {
        math::Vec3 unit;
        double radius_m = 0.0;
        bool add_extent = false;  // radius_m + a
    };
    void collect(int octave, const FaceCell& cell, double min_diameter_m, const Filter* filter,
                 std::vector<Rock>& out) const;
    double cell_factor(const FaceCell& cell) const;

    std::shared_ptr<const LunarSurfaceDetail> detail_;
    std::shared_ptr<const RockAbundanceMap> abundance_;
    std::vector<Octave> octaves_;
};

}  // namespace starsim::environment
