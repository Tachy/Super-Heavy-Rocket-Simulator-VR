#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "starsim/math/vec3.hpp"

// Cube-sphere face cells for the procedural surface detail, with exactly the mapping of game/world/terrain/cube_sphere.gd
// (so a cell at a level is the terrain quadtree node of the same key): faces 0-5 are +X, -X, +Y, -Y, +Z, -Z of the
// body-fixed frame; a face point (s, t) in [-1, 1]^2 maps to the unit direction of
// normal + tan(pi/4 s) u + tan(pi/4 t) v (tangent-adjusted gnomonic cube); cell (i, j) at `level` covers
// s in [-1 + 2 i / 2^level, -1 + 2 (i + 1) / 2^level] (t likewise with j). Beyond +-1 the face's projection continues
// ("continued coordinates"), used only next to a face edge.

namespace starsim::environment {

struct FaceCell {
    int face = 0;
    int level = 0;
    std::int64_t i = 0, j = 0;
    friend constexpr bool operator==(const FaceCell&, const FaceCell&) = default;
};

// Continuous face coordinates of a direction.
struct FaceCoord {
    int face = 0;
    double s = 0.0, t = 0.0;
};

// Local geometry of a face point on a sphere of radius R: unit tangents along increasing s and t (not orthogonal near
// face corners), metres per unit s and t, and the local linear size relative to the face centre (sqrt of the area
// element ratio: 1 at the face centre, 0.84 at an edge midpoint, 0.88 at a corner).
struct FaceFrame {
    math::Vec3 e_s, e_t;
    double m_s = 0.0, m_t = 0.0;
    double scale = 1.0;
};

const math::Vec3& face_normal(int face);
const math::Vec3& face_u(int face);
const math::Vec3& face_v(int face);

double face_warp(double s);                                     // tan(pi/4 s), exactly +-1 at +-1, odd
math::Vec3 face_direction(int face, double s, double t);        // CubeSphere.direction (continues beyond +-1)
FaceCoord face_coord_of(const math::Vec3& unit);                // the face CubeSphere.key_at picks (same tie rule)
FaceCoord face_coord_on(int face, const math::Vec3& unit);      // continued coordinates; needs dot(unit, normal) > 0
FaceFrame face_frame(int face, double s, double t, double sphere_radius_m);

FaceCell face_cell_of(const math::Vec3& unit, int level);              // same mapping as cube_sphere.gd key_at
math::Vec3 face_cell_point(const FaceCell& c, double s01, double t01);  // unit direction inside the cell
// The 3x3 block around the cell; cells beyond a face edge are the cells of the neighbouring face that hold the
// continued block's cell centres. At a cube corner (three faces) the block beyond the corner repeats a neighbour:
// the array then holds duplicates, which consumers skip.
std::array<FaceCell, 9> face_cell_neighbourhood(const FaceCell& c);
// Every cell at `level` that can hold a point within radius_m of `unit` (all faces, no duplicates). A 10 % margin on
// the local cell width; for radius_m <= 0.45 x the nominal cell size at most the 2 x 2 cells nearest the point on
// each face the disc reaches.
std::vector<FaceCell> face_cells_within(const math::Vec3& unit, int level, double radius_m, double sphere_radius_m);

// Margin on the local cell width when collecting the cells around a point (metric change over the range, [E]).
inline constexpr double face_cell_range_margin = 1.1;

// Smooth partition of unity over the faces around a direction, for fields defined per face lattice that must not
// step at face edges: face f weighs h(s_f) h(t_f) (continued coordinates, h = smoothstep from 1 at |x| = 1 - band
// to 0 at 1 + band), normalised to sum 1. Inside a face farther than `band` from its edges only that face (weight 1);
// across an edge two faces, at a corner three. Depends only on the direction (no face is preferred).
struct FaceBlend {
    int count = 0;
    std::array<FaceCoord, 3> coord;
    std::array<double, 3> weight{};
};
FaceBlend face_blend(const math::Vec3& unit, double band);
// Default blend half-width in face coordinates: 0.01 = +-13.6 km on the Moon at an edge midpoint, wide against every
// band-limited footprint the terrain asks for (<= a few km), narrow against the face ([D]).
inline constexpr double face_blend_band = 0.01;

// Nominal cell size (edge length at the face centre): R pi/2 / 2^level. Cells are 0.7-1.0 times as wide elsewhere
// (narrowest across an edge midpoint, [D] from the warp's metric).
double face_cell_nominal_size_m(int level, double sphere_radius_m);
// The finest level whose nominal cell size is at least cell_size_m (level 0 for anything larger than a face; at most
// max_face_cell_level).
int face_cell_level_for_size(double cell_size_m, double sphere_radius_m);
inline constexpr int max_face_cell_level = 30;
// Cell area on the sphere (area element at the cell centre times the cell's (s, t) extent; relative error ~ the
// cell's angular size squared).
double face_cell_area_m2(const FaceCell& c, double sphere_radius_m);

}  // namespace starsim::environment
