#pragma once

#include <array>
#include <optional>
#include <utility>
#include <vector>

#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Triangle mesh of a unit-size rock template (vertices on the template's surface, outward unit normals).
struct TemplateMesh {
    std::vector<math::Vec3> vertices, normals;
    // Per vertex 0..1: how deep the vertex sits in a concavity (a step's inner corner, a pit, a crack): the share of its
    // neighbourhood that rises above its tangent plane (the renderer settles dust there).
    std::vector<double> cavity;
    std::vector<int> indices;  // counter-clockwise triangles seen from outside
};

// Lunar rock shapes (moon procedural surface plan Task 3, angular fragments Task 11 fix A, fractured eroded lumps
// Task 17). Each template starts as an impact fragment: an ellipsoid with semi-axes 1 : b/a : c/a (the fragment axis
// law, Fujiwara et al. 1978 / Michikami et al. 2016: mean 2 : sqrt2 : 1, with scatter) cut by K hash-seeded primary
// fracture planes. Multi-scale relief then breaks it up: secondary and tertiary chips (shallow planar cuts at the
// protruding parts: smaller facets on edges and corners), steps (a chip on only one side of a leaning wall: a face
// broken into terraces, a concave inner corner), shallow pits, and erosion (every convex edge and corner rounded by a
// smooth minimum of the constraints, radius growing with the fragment's age class). The resting pose turns the
// fragment (c axis near the vertical, rms tilt 25 deg). The surface is star-shaped about the origin: a radial function
// r(dir), sampled on icospheres (LOD subdivisions 5, 4, 3, 2, 1: 10242 ... 42 vertices; the coarser vertex sets are
// prefixes of the finer ones).
// The rock IS its LOD-0 mesh: radius / normal / column tops / support / contact normals are exact queries on that
// polyhedron (same shape for the renderer, the contact physics and the statistics). Scaled so that the farthest vertex
// is at r = 1. Template frame: z = the local vertical; a rock scales it isotropically by its size (LunarRocks).
// Everything derives from the integer hash: identical on every machine.
class RockTemplates {
public:
    static constexpr int count = 32;  // [E] enough that neighbours do not look alike and the axis law is sampled
    static constexpr int lods = 5;    // [E] near, mid, far, small, speck
    static const RockTemplates& instance();

    // Distance from the origin to the LOD-0 surface along dir_local (need not be normalised): exact.
    double radius(int id, const math::Vec3& dir_local) const;
    // Outward unit normal of the LOD-0 triangle the ray along dir_local leaves through.
    math::Vec3 normal(int id, const math::Vec3& dir_local) const;
    // Contact normal at the unit-size surface point p for a contact whose own normal is `reference` (unit, template
    // frame): the unit vector of the normal cone at p closest to `reference`. The cone is spanned by the normals of every
    // LOD-0 triangle within `tolerance` (unit size) of p: inside a triangle its normal; on an edge or a vertex a flat body
    // pressing on it - a pad - keeps its own normal if that lies in the cone, else the nearest edge / face normal.
    math::Vec3 normal_toward(int id, const math::Vec3& p, const math::Vec3& reference, double tolerance) const;
    const TemplateMesh& mesh(int id, int lod) const;

    // Top of the unit-size surface on the vertical line (x, y) (unit coordinates): the highest z of the LOD-0 mesh on
    // the line, exact; nullopt when the line misses the rock.
    std::optional<double> column_top(int id, double x, double y) const;
    // column_top and the outward normal of the triangle it lies on.
    std::optional<std::pair<double, math::Vec3>> column_hit(int id, double x, double y) const;
    // The footprint column (i, j)'s top (column centre x = -1 + (2 i + 1) / footprint_columns, y likewise): NaN where
    // the column misses the rock.
    double column_grid_node(int id, int i, int j) const {
        return shapes_[static_cast<size_t>(id)].column_grid[static_cast<size_t>(i * footprint_columns + j)];
    }
    // The point of the unit-size body farthest along u (template frame, any length): max u . p, exact (a vertex of the
    // LOD-0 mesh).
    math::Vec3 support_point(int id, const math::Vec3& u) const;
    // Lowest and highest point of the unit-size surface (exact: the lowest / highest LOD-0 vertex).
    double bottom_z(int id) const { return shapes_[static_cast<size_t>(id)].bottom_z; }
    double top_z(int id) const { return shapes_[static_cast<size_t>(id)].top_z; }
    // Plan-view area of the part of the unit-size rock above the plane z = cut_z (projection onto x-y; unit disc = pi):
    // the area seen from above when the rock is buried to cut_z. From column_top on a footprint_columns^2 grid of
    // column centres over [-1, 1]^2, so it is the same criterion LunarRocks::rock_top uses.
    double exposed_area(int id, double cut_z) const;

    // The fragment's own axis ratios (b/a, c/a of the ellipsoid before the cuts) and the tilt of its c axis from the
    // vertical (rad): for the shape statistics.
    double fragment_b_to_a(int id) const { return shapes_[static_cast<size_t>(id)].b_to_a; }
    double fragment_c_to_a(int id) const { return shapes_[static_cast<size_t>(id)].c_to_a; }
    double c_axis_tilt_rad(int id) const { return shapes_[static_cast<size_t>(id)].tilt_rad; }
    bool eroded(int id) const { return shapes_[static_cast<size_t>(id)].eroded; }
    int primary_planes(int id) const { return shapes_[static_cast<size_t>(id)].primary_planes; }
    // Relief features: chips (secondary + tertiary), steps, pits; the erosion radius (unit size, before the scale).
    int chip_count(int id) const { return shapes_[static_cast<size_t>(id)].chips; }
    int step_count(int id) const { return shapes_[static_cast<size_t>(id)].steps; }
    int pit_count(int id) const { return shapes_[static_cast<size_t>(id)].pits; }
    double erosion_radius(int id) const { return shapes_[static_cast<size_t>(id)].erosion_radius; }

    // Shape statistics of the relief against the same fragment without relief (primary cuts only, sharp; the Task 11
    // templates): volume per D^3 (D: the plan-area diameter of the whole silhouette seen from above), volume when the
    // farthest point is at r = 1 (the templates' normalisation) and the extent ratios along the fragment's own axes
    // (b/a, c/a of the body, not of the ellipsoid).
    struct ShapeStats {
        double volume_per_d3 = 0.0, unit_volume = 0.0;
        double extent_b_to_a = 0.0, extent_c_to_a = 0.0;
    };
    const ShapeStats& relief_stats(int id) const { return shapes_[static_cast<size_t>(id)].relief; }
    const ShapeStats& base_stats(int id) const { return shapes_[static_cast<size_t>(id)].base; }
    // Enclosed volume of the unit-size LOD-0 mesh.
    double volume(int id) const { return shapes_[static_cast<size_t>(id)].volume; }

    // Columns per side of the footprint grid ([E]: ~7200 columns inside the unit disc; area error well below 1 %).
    static constexpr int footprint_columns = 96;
    // Burial share of the height for the dust statistics (LunarRocks' configured mean, 0.3).
    static constexpr double stats_burial_fraction = 0.3;

    // An up-facing LOD-0 triangle for the column queries: edge functions e = ex x + ey y + ec (>= 0 inside), its plane
    // z = z0 + zx x + zy y, its z and xy extent.
    struct ColumnTri {
        std::array<double, 3> ex{}, ey{}, ec{};
        double z0 = 0.0, zx = 0.0, zy = 0.0, zmin = 0.0, zmax = 0.0, x0 = 0.0, x1 = 0.0, y0 = 0.0, y1 = 0.0;
        int tri = -1;
    };

private:
    RockTemplates();

    // CSR lists: entries of bin k are items[offsets[k] .. offsets[k + 1]).
    struct Bins {
        std::vector<int> offsets, items;
    };
    struct Shape {
        double b_to_a = 1.0, c_to_a = 1.0, tilt_rad = 0.0;
        bool eroded = false;
        int primary_planes = 0, chips = 0, steps = 0, pits = 0;
        double erosion_radius = 0.0;
        double bottom_z = -1.0, top_z = 1.0, volume = 0.0;
        ShapeStats relief, base;
        // LOD-0 polyhedron (template frame, unit size).
        std::vector<math::Vec3> vertex;
        std::vector<std::array<int, 3>> triangle;
        std::vector<math::Vec3> triangle_normal;
        std::vector<ColumnTri> column_tris;
        Bins column_bins;   // up-facing triangles (column_tris) by xy cell, highest first
        Bins space_bins;    // all triangles by 3-D cell (space_bins_per_side^3 over [-1, 1]^3)
        Bins support_bins;  // vertex candidates per cube-map direction cell
        std::vector<double> column_tops;  // sorted tops of the footprint columns that hit the rock
        std::vector<double> column_grid;  // footprint_columns^2 column tops by (i, j), NaN where the column misses
    };
    int find_triangle(const math::Vec3& d) const;  // LOD-0 triangle whose cone holds d (icosphere hierarchy)

    std::array<Shape, count> shapes_;
    std::array<std::array<TemplateMesh, lods>, count> meshes_;
    // The icosphere hierarchy shared by all templates: unit directions (level 5) and the triangles of every level
    // (triangle k of level l has the children 4 k .. 4 k + 3 at level l + 1).
    std::vector<math::Vec3> directions_;
    std::vector<std::vector<int>> level_indices_;
    std::vector<std::vector<std::array<math::Vec3, 3>>> edge_normals_;  // per level and triangle: unit edge-plane normals
};

}  // namespace starsim::environment
