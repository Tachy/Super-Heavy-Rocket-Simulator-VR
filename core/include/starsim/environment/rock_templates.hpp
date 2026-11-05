#pragma once

#include <array>
#include <optional>
#include <vector>

#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Triangle mesh of a unit-size rock template (vertices on the template's surface, outward unit normals).
struct TemplateMesh {
    std::vector<math::Vec3> vertices, normals;
    std::vector<int> indices;  // counter-clockwise triangles seen from outside
};

// Angular impact-fragment rock shapes: each template is a convex body, the intersection of an ellipsoid with semi-axes
// 1 : b/a : c/a (the impact-fragment axis law, Fujiwara et al. 1978 / Michikami et al. 2016: mean 2 : sqrt2 : 1, with
// scatter) and K hash-seeded half-spaces (planar fracture faces: flat facets meeting in sharp edges). Fresh fragments
// have many deep cuts (almost no ellipsoid surface left); eroded ones a few shallow cuts (mostly the rounded ellipsoid).
// The fragment is turned by a hash-seeded rotation (the c axis tilted: resting or embedded at random) and scaled so
// that max r = 1. Template frame: z = the local vertical; a rock scales it isotropically by its size (LunarRocks).
// Being convex and containing the origin, every template is star-shaped: r(dir) is the exact ray exit distance, and the
// column top on a vertical line is exact and cheap (min / max over the constraints). Meshes: icosphere subdivision 4,
// 3, 2, 1 (2562, 642, 162, 42 vertices) with vertex = dir r(dir) and the exact facet / ellipsoid normal.
// Everything derives from the integer hash: identical on every machine.
class RockTemplates {
public:
    static constexpr int count = 32;  // [E] enough that neighbours do not look alike and the axis law is sampled
    static constexpr int lods = 4;    // [E] near, mid, far, speck
    static constexpr int max_planes = 24;
    static const RockTemplates& instance();

    // Unit-size radial function (dir_local need not be normalised).
    double radius(int id, const math::Vec3& dir_local) const;
    // Outward unit normal of the unit-size surface in direction dir_local (the active facet's normal, or the
    // ellipsoid's).
    math::Vec3 normal(int id, const math::Vec3& dir_local) const;
    // Contact normal at the unit-size surface point p for a contact whose own normal is `reference` (unit, template
    // frame): the unit vector of the normal cone at p closest to `reference`. The cone is spanned by the normals of every
    // face (and the ellipsoid) within `tolerance` (unit size) of p: on a face it is the face's normal; on an edge or a
    // vertex (where the normal is not defined) a flat body pressing on it - a pad - keeps its own normal if that lies in
    // the cone, else the nearest edge / face normal.
    math::Vec3 normal_toward(int id, const math::Vec3& p, const math::Vec3& reference, double tolerance) const;
    const TemplateMesh& mesh(int id, int lod) const;

    // Top of the unit-size surface on the vertical line (x, y) (unit coordinates): the highest z inside the rock,
    // exact (the least upper bound of the constraints on the line); nullopt when the line misses the rock.
    std::optional<double> column_top(int id, double x, double y) const;
    // The footprint column (i, j)'s top (column centre x = -1 + (2 i + 1) / footprint_columns, y likewise): NaN where
    // the column misses the rock.
    double column_grid_node(int id, int i, int j) const {
        return shapes_[static_cast<size_t>(id)].column_grid[static_cast<size_t>(i * footprint_columns + j)];
    }
    // The point of the unit-size body farthest along u (template frame, any length): max u . p, exact (the ellipsoid's
    // own maximum, the maximum on each face's ellipse, or a fixed extreme point - an edge end or a vertex).
    math::Vec3 support_point(int id, const math::Vec3& u) const;
    // Lowest and highest point of the unit-size surface (exact: the support of the convex body along -z / +z).
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
    int plane_count(int id) const { return shapes_[static_cast<size_t>(id)].planes; }
    // The constraints themselves (template frame, unit size): face k is n . p <= h (|n| = 1), the ellipsoid
    // p^T Q p <= 1 - for the renderer's exact per-fragment facet normals (moon_rock.gdshader).
    math::Vec3 plane_normal(int id, int k) const { return shapes_[static_cast<size_t>(id)].normal[static_cast<size_t>(k)]; }
    double plane_distance(int id, int k) const { return shapes_[static_cast<size_t>(id)].distance[static_cast<size_t>(k)]; }
    const math::Mat3& quadric(int id) const { return shapes_[static_cast<size_t>(id)].quadric; }
    bool eroded(int id) const { return shapes_[static_cast<size_t>(id)].eroded; }

    // Columns per side of the footprint grid ([E]: ~7200 columns inside the unit disc; area error well below 1 %).
    static constexpr int footprint_columns = 96;

private:
    RockTemplates();

    struct Shape {
        // Constraints in the template frame, after the rotation and the scale: n . p <= h (|n| = 1) and p^T Q p <= 1.
        std::array<math::Vec3, max_planes> normal{};
        std::array<double, max_planes> distance{};
        int planes = 0;
        math::Mat3 quadric;
        // Ball coordinates: p = ball_to_template q (|q| <= 1: the ellipsoid), planes ball_normal . q <= distance;
        // fixed_points: the direction-independent extreme points (edge ends on the ellipsoid, vertices), template frame.
        math::Mat3 ball_to_template;
        std::array<math::Vec3, max_planes> ball_normal{};
        std::vector<math::Vec3> fixed_points;
        double b_to_a = 1.0, c_to_a = 1.0, tilt_rad = 0.0;
        bool eroded = false;
        double bottom_z = -1.0, top_z = 1.0;
        std::vector<double> column_tops;  // sorted tops of the footprint columns that hit the rock
        std::vector<double> column_grid;  // footprint_columns^2 column tops by (i, j), NaN where the column misses
    };
    // Ray exit distance along the unit direction d and the index of the active constraint (-1: the ellipsoid).
    static double exit_distance(const Shape& shape, const math::Vec3& d, int* active);
    static std::optional<double> column_top(const Shape& shape, double x, double y);
    static math::Vec3 support_point(const Shape& shape, const math::Vec3& u);

    std::array<Shape, count> shapes_;
    std::array<std::array<TemplateMesh, lods>, count> meshes_;
};

}  // namespace starsim::environment
