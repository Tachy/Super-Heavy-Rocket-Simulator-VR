#pragma once

#include <array>
#include <optional>
#include <vector>

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Triangle mesh of a unit-size rock template (vertices on the template's surface, outward unit normals).
struct TemplateMesh {
    std::vector<math::Vec3> vertices, normals;
    std::vector<int> indices;  // counter-clockwise triangles seen from outside
};

// Star-shaped rock shapes: 16 seeded radial functions r(dir) > 0 on the unit sphere, built from a few random plane
// waves (r = exp(sum a_k cos(f_k dir.d_k + p_k)), a_k ~ 1/f_k), scaled so that max r = 1 (a rock's semi-axes a, b, c
// then bound it; the shape fills them only partly: about 0.66 along an axis on average, so a rock's size follows from
// its measured plan-view area, see exposed_area and LunarRocks). Meshes: icosphere subdivision 4, 3, 2, 1 (2562, 642, 162, 42 vertices) with vertex = dir r(dir).
// Everything derives from the integer hash: identical on every machine.
class RockTemplates {
public:
    static constexpr int count = 16;  // [E] enough that neighbours do not look alike
    static constexpr int lods = 4;    // [E] near, mid, far, speck
    static const RockTemplates& instance();

    // Unit-size radial function (dir_local need not be normalised).
    double radius(int id, const math::Vec3& dir_local) const;
    // Outward unit normal of the unit-size surface in direction dir_local.
    math::Vec3 normal(int id, const math::Vec3& dir_local) const;
    const TemplateMesh& mesh(int id, int lod) const;

    // Top of the unit-size surface on the vertical line (x, y) (unit coordinates, |(x, y)| < 1): highest z inside the
    // rock, found by a scan from z = 1 down in top_scan_steps steps and bisection; nullopt when the line misses the rock.
    std::optional<double> column_top(int id, double x, double y) const;
    // column_top interpolated bilinearly between the footprint columns (the footprint_columns^2 grid of column centres,
    // computed once): ~20 ns instead of the scan's ~4 us, within ~1e-4 of the unit size of column_top (curvature x
    // cell^2 / 8). nullopt where a corner column misses the rock or (x, y) lies outside the grid of column centres
    // (a band of half a column at the outline).
    std::optional<double> column_top_interpolated(int id, double x, double y) const;
    // The footprint column (i, j)'s top (column centre x = -1 + (2 i + 1) / footprint_columns, y likewise): NaN where
    // the column misses the rock.
    double column_grid_node(int id, int i, int j) const {
        return shapes_[static_cast<size_t>(id)].column_grid[static_cast<size_t>(i * footprint_columns + j)];
    }
    // Highest footprint column top (>= every column_top_interpolated value).
    double column_grid_max(int id) const { return shapes_[static_cast<size_t>(id)].column_grid_max; }
    // Lowest and highest point of the unit-size surface (sampled; the vertical extent of the template).
    double bottom_z(int id) const { return shapes_[static_cast<size_t>(id)].bottom_z; }
    double top_z(int id) const { return shapes_[static_cast<size_t>(id)].top_z; }
    // Plan-view area of the part of the unit-size rock above the plane z = cut_z (projection onto x-y; unit disc = pi):
    // the area seen from above when the rock is buried to cut_z. From column_top on a footprint_columns^2 grid of
    // column centres over [-1, 1]^2, so it is the same criterion LunarRocks::rock_top uses.
    double exposed_area(int id, double cut_z) const;

    // Scan steps over the height when searching a column's top ([E]: the templates are smooth, no thin spikes).
    static constexpr int top_scan_steps = 48;
    // Columns per side of the footprint grid ([E]: ~7200 columns inside the unit disc; area error well below 1 %).
    static constexpr int footprint_columns = 96;

private:
    RockTemplates();

    static constexpr int waves = 8;  // [E]
    struct Wave {
        math::Vec3 direction;
        double frequency = 0.0, amplitude = 0.0, phase = 0.0;
    };
    struct Shape {
        std::array<Wave, waves> wave;
        double scale = 1.0;  // 1 / sampled maximum
        double bottom_z = -1.0, top_z = 1.0;
        std::vector<double> column_tops;  // sorted tops of the footprint columns that hit the rock
        std::vector<double> column_grid;  // footprint_columns^2 column tops by (i, j), NaN where the column misses
        double column_grid_max = -1.0;
    };
    double raw_radius(const Shape& shape, const math::Vec3& dir_unit) const;

    std::array<Shape, count> shapes_;
    std::array<std::array<TemplateMesh, lods>, count> meshes_;
};

}  // namespace starsim::environment
