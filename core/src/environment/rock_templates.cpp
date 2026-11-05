#include "starsim/environment/rock_templates.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <thread>
#include <utility>

#include "starsim/environment/surface_hash.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"

namespace starsim::environment {

using math::Mat3;
using math::Vec3;

using ColumnTri = RockTemplates::ColumnTri;

namespace {

// ---- The fragment (Task 11 fix A / fix C; unchanged by the Task 17 relief) ----
// Fragment axis law: a : b : c = 2 : sqrt2 : 1 on average (b/a 0.71, c/b 0.71, c/a 0.50) [S] Fujiwara et al. 1977/1978 laboratory
// impact fragments; confirmed for catastrophic-disruption fragments by Michikami et al. 2016 (Icarus 264, peaks near
// b/a 0.7, c/a 0.5; cratering fragments flatter, c/a down to ~0.2) and for Itokawa boulders (Michikami et al. 2010) and
// LRO NAC lunar boulders (b/a mostly ~0.7, 0.3-0.9; Krishna & Kumar 2016).
// c is drawn as c/b (same law, 1 / sqrt2) so that c <= b needs no clamp that would pile templates up at c = b.
constexpr double fragment_b_to_a_mean = 0.71, fragment_c_to_b_mean = 0.71;  // [S] see above
constexpr double fragment_ratio_sd = 0.15;                                   // [E] spread of the NAC b/a range 0.3-0.9
constexpr double min_b_to_a = 0.3, min_c_to_a = 0.2;  // [S] NAC b/a range; Michikami 2016 flattest (cratering) fragments
// Fresh vs eroded: lunar boulders are worn down by micrometeorite abrasion and thermal fatigue and finally destroyed
// (Basilevsky et al. 2013, PSS 89; 2015, PSS 117: survival times of metre boulders ~40-80 Myr), so a surface population
// mixes angular fresh fragments with rounded old ones. No measured share found: [E] from the Apollo photo comparison.
constexpr double eroded_share = 0.3;                                    // [E]
constexpr int fresh_planes_min = 16, fresh_planes_span = 9;             // [E] 16-24 primary fracture faces
constexpr double fresh_chip_min = 0.15, fresh_chip_span = 0.25;         // [E] cut depth 15-40 % of the ellipsoid support
constexpr int eroded_planes_min = 5, eroded_planes_span = 4;            // [E] 5-8 shallow faces
constexpr double eroded_chip_min = 0.03, eroded_chip_span = 0.09;       // [E] 3-12 %: mostly the rounded ellipsoid
constexpr double plane_jitter = 0.4;                                    // [E] of the Fibonacci spacing: irregular faces
// Orientation: a fragment rests gravitationally stable on (near) its broadest face, so its shortest (c) axis stands near
// the vertical; local slopes, clods under the rock and partial burial scatter it (controller ruling R4, 2026-11-05: no
// upright balanced slabs, which the Apollo panoramas do not show). No measured tilt distribution found ([E]; the
// review's lead, Demidov & Basilevsky's Lunokhod/Apollo H/D measurements, could not be retrieved by two web searches):
// the tilt is a 2-D Gaussian on the tangent plane (Rayleigh-distributed angle) with an rms tilt of 25 deg, capped at
// 60 deg (a slab tipped further would fall onto a face).
constexpr double rms_tilt_rad = 25.0 * constants::deg_to_rad;  // [E] ruling R4
constexpr double max_tilt_rad = 60.0 * constants::deg_to_rad;  // [E] stability cap

// ---- Multi-scale relief (Task 17; user 2026-11-05: the Task 11 templates looked like crystals) ----
// Lengths in the fragment frame (long semi-axis 1, before the final scale to max r = 1).
// Erosion: micrometeorite abrasion wears lunar rocks down by ~1 mm/Myr (Hoerz et al. 1975, The Moon 13) and metre
// boulders survive ~40-150 Myr (Basilevsky et al. 2013 / 2015), so an old boulder's edges are rounded over cm-dm
// while a fresh fragment keeps nearly sharp, chipped edges. The rounding radius is a share of the rock size [E]
// (survival time and so age grow with size): fresh 0.6-1.4 %, eroded 3.5-7 % of the long semi-axis. Implemented as a
// compact smooth minimum (length rho, reach erosion_reach rho) of the exit distances of every convex constraint: a face
// stays exactly planar away from its edges; an edge of two faces sinks by ~0.7 rho, a corner of three by ~1.1 rho.
constexpr double fresh_erosion_min = 0.006, fresh_erosion_span = 0.008;    // [E]
constexpr double eroded_erosion_min = 0.035, eroded_erosion_span = 0.035;  // [E]
constexpr double erosion_reach = 3.0;  // [D] support of the smooth minimum in units of rho (h(x) < 3 % beyond 2 rho)
// Chips: shallow planar cuts (random direction, depth below the body's support along it) remove protruding edges and
// corners as smaller facets: the 2nd and 3rd fracture scale of a broken rock (the A17 station 6 / 7 boulders show
// faces broken into facets of ~1/3 and ~1/10 of the face size). No measured facet statistics: counts and depths [E]
// from the Apollo photo comparison (screenshots/moon_surface/compare/).
struct FeatureClass {
    int min, span;                     // count min + [0, span)
    double depth_min, depth_span;      // fragment units
};
constexpr FeatureClass fresh_chips_secondary{10, 7, 0.04, 0.06};    // [E]
constexpr FeatureClass fresh_chips_tertiary{20, 11, 0.008, 0.017};  // [E]
constexpr FeatureClass eroded_chips_secondary{4, 5, 0.02, 0.03};    // [E]
constexpr FeatureClass eroded_chips_tertiary{8, 5, 0.005, 0.01};    // [E]
// Steps: a chip that removes material only beyond a wall plane leaning out of the surface: the face drops by the depth
// beyond a line (a terrace, a concave inner corner). Directions: most near a primary face's normal (steps on the large
// fracture faces), the rest anywhere. [E]
constexpr FeatureClass fresh_steps_major{5, 4, 0.04, 0.04};   // [E]
constexpr FeatureClass fresh_steps_minor{8, 5, 0.015, 0.02};    // [E]
constexpr FeatureClass eroded_steps_major{2, 2, 0.025, 0.03};   // [E]
constexpr FeatureClass eroded_steps_minor{3, 3, 0.01, 0.015};   // [E]
constexpr double step_on_face_share = 0.6;                      // [E]
constexpr double step_face_tilt_rad = 0.04;                     // [E] scar plane against the face (a terrace across it)
constexpr double step_wall_lean_min_rad = 0.6, step_wall_lean_span_rad = 0.5;  // [E] wall from the tangent plane: a riser of 35-63 deg
constexpr double step_wall_offset = 0.5;                        // [E] of the removed cap's extent, either side
constexpr double step_wall_min_distance = 0.02;                 // [D] the wall half-space excludes the origin (continuous r)
// Pits: shallow concavities (spalled or impact-pitted spots): the radius sinks by depth x (1 - (angle / reach)^2)^2.
struct PitClass {
    int min, span;
    double depth_min, depth_span, reach_min_rad, reach_span_rad;
};
constexpr PitClass fresh_pits{1, 2, 0.02, 0.03, 0.15, 0.25};    // [E]
constexpr PitClass eroded_pits{2, 3, 0.02, 0.04, 0.2, 0.3};     // [E]
// Undulation: fracture faces are not flat (conchoidal, hackled) and old surfaces lumpy: radial kernels of either sign
// (amplitude up to the given share, the same profile as a pit) over the whole body. [E]
constexpr PitClass fresh_lumps{8, 6, 0.0, 0.012, 0.4, 0.4};   // [E] |amplitude| 0-1.2 %, reach 0.4-0.8 rad: gently warped faces
constexpr PitClass eroded_lumps{8, 6, 0.0, 0.025, 0.35, 0.4}; // [E] 0-2.5 %
// Cavity per vertex (renderer's dust): the mean sine of the elevation of a ring of neighbours above the vertex's
// tangent plane, full at this value (a 90 deg inner corner gives ~0.35). [E]
constexpr double cavity_full_sine = 0.3;
constexpr int cavity_ring = 8;
constexpr double cavity_min_angle_rad = 0.05;   // [E] ring radius at the finest LOD (~2.5 vertex spacings)
// Normals per LOD: central differences of the surface over this share of the LOD's edge angle (the coarse LODs get the
// relief's normal smoothed over their own vertex spacing: no aliasing). [E]
constexpr double normal_step_per_edge = 0.2;

constexpr int normal_draws = 6;            // normal-like draws: sum of six uniforms (no library distributions)
constexpr int placement_samples = 6000;    // Fibonacci directions on which features find the support
constexpr double parallel_limit = 1e-15;   // |n . d| below this: the constraint does not bound the ray
constexpr int max_terms = 160;
constexpr int finest_level = 5;
constexpr std::array<int, RockTemplates::lods> lod_levels = {5, 4, 3, 2, 1};  // icosphere subdivision per LOD [E]
constexpr double icosahedron_edge_rad = 1.1071487177940904;  // atan(2): angle between neighbouring icosahedron vertices
// Query structures.
constexpr int column_bins_per_side = 96;   // [D] a few up-facing triangles per cell
constexpr int space_bins_per_side = 32;    // [D] ~20 triangles per cell
constexpr int support_cells_per_side = 16; // [D] cube-map cells: ~50 vertex candidates each
constexpr double point_in_triangle_tolerance = 1e-12;

double unit(std::uint64_t seed, std::uint64_t a, std::uint64_t b) {
    return hash_unit(surface_hash(surface_stream::rocks << 8, 0x7E4, seed * 256 + a, b));
}

double normal_draw(std::uint64_t seed, std::uint64_t field) {
    double sum = 0.0;
    for (int k = 0; k < normal_draws; ++k) sum += unit(seed, field, static_cast<std::uint64_t>(k));
    return (sum - 0.5 * normal_draws) * std::sqrt(2.0);
}

Vec3 any_perpendicular(const Vec3& d) {
    return math::normalized(cross(std::abs(d.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, d));
}

Vec3 fibonacci(int n, int count) {
    const double golden = constants::pi * (3.0 - std::sqrt(5.0));
    const double z = 1.0 - 2.0 * (n + 0.5) / count, rho = std::sqrt(std::max(0.0, 1.0 - z * z));
    return {rho * std::cos(golden * n), rho * std::sin(golden * n), z};
}

Vec3 unit_vector(double u1, double u2) {
    const double z = 2.0 * u1 - 1.0, rho = std::sqrt(std::max(0.0, 1.0 - z * z));
    return {rho * std::cos(constants::two_pi * u2), rho * std::sin(constants::two_pi * u2), z};
}

// Unit tangent of n at the angle phi from a fixed perpendicular.
Vec3 tangent(const Vec3& n, double phi) {
    const Vec3 t1 = any_perpendicular(n), t2 = cross(n, t1);
    return t1 * std::cos(phi) + t2 * std::sin(phi);
}

// Uniform random rotation from three uniforms (Shoemake 1992, unit quaternion).
Mat3 uniform_rotation(double u1, double u2, double u3) {
    const double a = std::sqrt(1.0 - u1), b = std::sqrt(u1);
    const double x = a * std::sin(constants::two_pi * u2), y = a * std::cos(constants::two_pi * u2);
    const double z = b * std::sin(constants::two_pi * u3), w = b * std::cos(constants::two_pi * u3);
    Mat3 r;
    r.m[0] = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)};
    r.m[1] = {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)};
    r.m[2] = {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)};
    return r;
}

Mat3 rotation_z(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return Mat3::from_columns({c, s, 0}, {-s, c, 0}, {0, 0, 1});
}

Mat3 rotation_y(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return Mat3::from_columns({c, 0, -s}, {0, 1, 0}, {s, 0, c});
}

int count_of(const FeatureClass& c, double u) { return c.min + std::min(static_cast<int>(u * c.span), c.span - 1); }

// ---- The radial field of a fragment (fragment frame) ----
struct Plane {
    Vec3 n;  // unit, outward
    double h = 0.0;  // n . p <= h inside
};
struct Wedge {  // removed: n_floor . p > h_floor and n_wall . p > h_wall (both h > 0)
    Plane floor, wall;
};
struct Pit {
    Vec3 axis;
    double depth = 0.0, reach = 0.0, cos_reach = 1.0;
};

struct Field {
    Vec3 semi;
    std::vector<Plane> planes;  // primary first, then chips
    int primary = 0;
    std::vector<Wedge> wedges;
    std::vector<Pit> pits;
    double rho = 0.0;

    // Radius along the unit direction d: relief false = the sharp primary body (ellipsoid and primary planes).
    double radius(const Vec3& d, bool relief) const {
        std::array<double, max_terms> t{};
        int n = 0;
        const Vec3 q{d.x / semi.x, d.y / semi.y, d.z / semi.z};
        t[n++] = 1.0 / std::sqrt(dot(q, q));
        const int planes_used = relief ? static_cast<int>(planes.size()) : primary;
        for (int k = 0; k < planes_used; ++k) {
            const Plane& p = planes[static_cast<size_t>(k)];
            const double nd = dot(p.n, d);
            if (nd > parallel_limit) t[n++] = p.h / nd;
        }
        if (relief) {
            for (const Wedge& w : wedges) {
                const double a = dot(w.floor.n, d), b = dot(w.wall.n, d);
                if (a > parallel_limit && b > parallel_limit && n < max_terms) {
                    t[n++] = std::max(w.floor.h / a, w.wall.h / b);
                }
            }
        }
        double t_min = t[0];
        for (int k = 1; k < n; ++k) t_min = std::min(t_min, t[k]);
        double r = t_min;
        if (relief && rho > 0.0) {
            // Compact smooth minimum: t_min - rho ln(1 + sum of h(x_k)) over the other constraints, x = (t - t_min) /
            // rho, h(x) = (1 - x / reach)^3 below the reach: an edge of two equal constraints sinks by rho ln 2 like a
            // log-sum-exp, a face farther than reach x rho from every edge stays exactly planar. Continuous where the
            // minimum changes hands (the other one's h(0) = 1 either way).
            double s = 1.0;
            bool skipped_min = false;
            for (int k = 0; k < n; ++k) {
                if (!skipped_min && t[k] == t_min) {
                    skipped_min = true;
                    continue;
                }
                const double x = (t[k] - t_min) / (rho * erosion_reach);
                if (x < 1.0) s += (1.0 - x) * (1.0 - x) * (1.0 - x);
            }
            r = t_min - rho * std::log(s);
        }
        if (relief) {
            for (const Pit& p : pits) {
                const double c = dot(d, p.axis);
                if (c > p.cos_reach) {
                    const double x = std::acos(std::min(1.0, c)) / p.reach, w = 1.0 - x * x;
                    r *= 1.0 - p.depth * w * w;
                }
            }
        }
        return r;
    }
};

// Builds the fragment's field: the primary body, then the relief placed against it.
Field make_field(std::uint64_t seed, bool eroded, const Vec3& semi, int planes, double chip_min, double chip_span,
                 int* chips_out, int* steps_out, int* pits_out) {
    Field f;
    f.semi = semi;
    f.primary = planes;
    // Faces: tangent planes of the ellipsoid at a jittered Fibonacci set of points s nu (nu on the unit sphere, so
    // the faces are spread over the surface, rims of flat fragments included), turned at random and pushed in.
    const Mat3 face_turn = uniform_rotation(unit(seed, 5, 0), unit(seed, 5, 1), unit(seed, 5, 2));
    for (int k = 0; k < planes; ++k) {
        const auto ku = static_cast<std::uint64_t>(k);
        const double z = std::clamp(1.0 - 2.0 * (k + 0.5 + plane_jitter * (2.0 * unit(seed, 6, ku) - 1.0)) / planes,
                                    -1.0, 1.0);
        const double rho = std::sqrt(1.0 - z * z);
        const double phi = constants::pi * (3.0 - std::sqrt(5.0)) * k +
                           plane_jitter * constants::two_pi / std::sqrt(static_cast<double>(planes)) *
                               (2.0 * unit(seed, 7, ku) - 1.0);
        const Vec3 nu = face_turn * Vec3{rho * std::cos(phi), rho * std::sin(phi), z};
        const Vec3 n = math::normalized(Vec3{nu.x / semi.x, nu.y / semi.y, nu.z / semi.z});
        const double support_e = norm(Vec3{n.x * semi.x, n.y * semi.y, n.z * semi.z});
        f.planes.push_back({n, support_e * (1.0 - (chip_min + chip_span * unit(seed, 8, ku)))});
    }
    // The primary body's surface on the placement directions: supports and caps of the features.
    std::vector<Vec3> surface(placement_samples);
    for (int k = 0; k < placement_samples; ++k) {
        const Vec3 d = fibonacci(k, placement_samples);
        surface[static_cast<size_t>(k)] = d * f.radius(d, false);
    }
    const auto support = [&](const Vec3& n) {
        double best = -1e300;
        for (const Vec3& p : surface) best = std::max(best, dot(n, p));
        return best;
    };

    f.rho = eroded ? eroded_erosion_min + eroded_erosion_span * unit(seed, 20, 0)
                   : fresh_erosion_min + fresh_erosion_span * unit(seed, 20, 0);
    // Chips.
    int chips = 0;
    std::uint64_t feature = 0;
    for (const FeatureClass& c : eroded ? std::array{eroded_chips_secondary, eroded_chips_tertiary}
                                        : std::array{fresh_chips_secondary, fresh_chips_tertiary}) {
        const int n = count_of(c, unit(seed, 21, feature));
        for (int k = 0; k < n; ++k, ++feature) {
            const Vec3 dir = unit_vector(unit(seed, 22, feature), unit(seed, 23, feature));
            const double depth = c.depth_min + c.depth_span * unit(seed, 24, feature);
            f.planes.push_back({dir, support(dir) - depth});
            ++chips;
        }
        ++feature;
    }
    // Steps.
    int steps = 0;
    for (const FeatureClass& c : eroded ? std::array{eroded_steps_major, eroded_steps_minor}
                                        : std::array{fresh_steps_major, fresh_steps_minor}) {
        const int n = count_of(c, unit(seed, 30, feature));
        for (int k = 0; k < n; ++k, ++feature) {
            Vec3 dir;
            if (unit(seed, 31, feature) < step_on_face_share) {
                const int face = std::min(static_cast<int>(unit(seed, 32, feature) * planes), planes - 1);
                const Vec3 nf = f.planes[static_cast<size_t>(face)].n;
                dir = math::normalized(nf + tangent(nf, constants::two_pi * unit(seed, 33, feature)) *
                                                (step_face_tilt_rad * unit(seed, 34, feature)));
            } else {
                dir = unit_vector(unit(seed, 32, feature), unit(seed, 33, feature));
            }
            const double depth = c.depth_min + c.depth_span * unit(seed, 35, feature);
            const double top = support(dir), floor_h = top - depth;
            // The removed cap of the primary body: its centroid and extent.
            Vec3 centre{0, 0, 0};
            int in_cap = 0;
            for (const Vec3& p : surface) {
                if (dot(dir, p) > floor_h) centre = centre + p, ++in_cap;
            }
            if (in_cap == 0) continue;
            centre = centre / static_cast<double>(in_cap);
            double extent = 0.0;
            for (const Vec3& p : surface) {
                if (dot(dir, p) > floor_h) extent = std::max(extent, norm(p - centre));
            }
            const double lean = step_wall_lean_min_rad + step_wall_lean_span_rad * unit(seed, 36, feature);
            const double offset = step_wall_offset * extent * (2.0 * unit(seed, 37, feature) - 1.0);
            bool placed = false;
            for (const double flip : {1.0, -1.0}) {
                const Vec3 t = tangent(dir, constants::two_pi * unit(seed, 38, feature)) * flip;
                const Vec3 m = math::normalized(t * std::cos(lean) + dir * std::sin(lean));
                const double g = dot(m, centre) + offset * std::cos(lean);
                if (g > step_wall_min_distance && floor_h > step_wall_min_distance) {
                    f.wedges.push_back({{dir, floor_h}, {m, g}});
                    placed = true;
                    break;
                }
            }
            if (placed) ++steps;
        }
        ++feature;
    }
    // Undulation (signed kernels; stored with the pits, which only sink).
    const PitClass& lc = eroded ? eroded_lumps : fresh_lumps;
    const int lumps = lc.min + std::min(static_cast<int>(unit(seed, 50, 0) * lc.span), lc.span - 1);
    for (int k = 0; k < lumps; ++k) {
        const auto ku = static_cast<std::uint64_t>(k);
        Pit p;
        p.axis = unit_vector(unit(seed, 51, ku), unit(seed, 52, ku));
        p.depth = (lc.depth_min + lc.depth_span * unit(seed, 53, ku)) * (unit(seed, 54, ku) < 0.5 ? -1.0 : 1.0);
        p.reach = lc.reach_min_rad + lc.reach_span_rad * unit(seed, 55, ku);
        p.cos_reach = std::cos(p.reach);
        f.pits.push_back(p);
    }
    // Pits.
    const PitClass& pc = eroded ? eroded_pits : fresh_pits;
    const int pits = pc.min + std::min(static_cast<int>(unit(seed, 40, 0) * pc.span), pc.span - 1);
    for (int k = 0; k < pits; ++k) {
        const auto ku = static_cast<std::uint64_t>(k);
        Pit p;
        p.axis = unit_vector(unit(seed, 41, ku), unit(seed, 42, ku));
        p.depth = pc.depth_min + pc.depth_span * unit(seed, 43, ku);
        p.reach = pc.reach_min_rad + pc.reach_span_rad * unit(seed, 44, ku);
        p.cos_reach = std::cos(p.reach);
        f.pits.push_back(p);
    }
    *chips_out = chips;
    *steps_out = steps;
    *pits_out = pits;
    return f;
}

// ---- Icosphere hierarchy ----
struct Icospheres {
    std::vector<Vec3> vertices;              // unit directions, finest level (coarser levels: prefixes)
    std::vector<std::vector<int>> indices;   // per level 0..finest_level
};

Icospheres make_icospheres() {
    const double t = 0.5 * (1.0 + std::sqrt(5.0));
    Icospheres s;
    for (const Vec3& v : {Vec3{-1, t, 0}, Vec3{1, t, 0}, Vec3{-1, -t, 0}, Vec3{1, -t, 0}, Vec3{0, -1, t}, Vec3{0, 1, t},
                          Vec3{0, -1, -t}, Vec3{0, 1, -t}, Vec3{t, 0, -1}, Vec3{t, 0, 1}, Vec3{-t, 0, -1},
                          Vec3{-t, 0, 1}}) {
        s.vertices.push_back(math::normalized(v));
    }
    s.indices.push_back({0, 11, 5, 0, 5, 1,  0, 1,  7,  0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11, 4,  11, 10, 2,  10, 7, 6, 7, 1, 8,
                         3, 9,  4, 3, 4, 2,  3, 2,  6,  3, 6,  8,  3, 8,  9,  4, 9, 5, 2, 4,  11, 6,  2,  10, 8,  6, 7, 9, 8, 1});
    for (int level = 0; level < finest_level; ++level) {
        std::map<std::pair<int, int>, int> midpoint;
        const auto middle = [&](int a, int b) {
            const std::pair<int, int> key{std::min(a, b), std::max(a, b)};
            const auto it = midpoint.find(key);
            if (it != midpoint.end()) return it->second;
            s.vertices.push_back(math::normalized(s.vertices[static_cast<size_t>(a)] + s.vertices[static_cast<size_t>(b)]));
            return midpoint[key] = static_cast<int>(s.vertices.size()) - 1;
        };
        const std::vector<int>& cur = s.indices.back();
        std::vector<int> next;
        for (size_t k = 0; k < cur.size(); k += 3) {
            const int a = cur[k], b = cur[k + 1], c = cur[k + 2];
            const int ab = middle(a, b), bc = middle(b, c), ca = middle(c, a);
            // Children of parent triangle k / 3: triangles 4 (k / 3) .. 4 (k / 3) + 3 of the next level.
            for (const int tri : {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca}) next.push_back(tri);
        }
        s.indices.push_back(std::move(next));
    }
    return s;
}

const Icospheres& icospheres() {
    static const Icospheres s = make_icospheres();
    return s;
}

int vertices_at_level(int level) { return 10 * (1 << (2 * level)) + 2; }

// ---- Polyhedron queries (shared by the relief body and its relief-free reference) ----
struct Poly {
    std::vector<Vec3> vertex;
    std::vector<std::array<int, 3>> triangle;
    std::vector<Vec3> normal;
};

Poly make_poly(std::vector<Vec3> vertex, const std::vector<int>& indices) {
    Poly p;
    p.vertex = std::move(vertex);
    for (size_t k = 0; k < indices.size(); k += 3) {
        const std::array<int, 3> t{indices[k], indices[k + 1], indices[k + 2]};
        p.triangle.push_back(t);
        const Vec3 &a = p.vertex[static_cast<size_t>(t[0])], &b = p.vertex[static_cast<size_t>(t[1])],
                   &c = p.vertex[static_cast<size_t>(t[2])];
        p.normal.push_back(math::normalized(cross(b - a, c - a)));
    }
    return p;
}

int cell_of(double x, int cells) {  // [-1, 1] -> 0 .. cells - 1
    return std::clamp(static_cast<int>(std::floor((x + 1.0) * 0.5 * cells)), 0, cells - 1);
}

template <class Range>
void fill_bins(int bin_count, const Range& for_each_item, std::vector<int>& offsets, std::vector<int>& items) {
    offsets.assign(static_cast<size_t>(bin_count) + 1, 0);
    for_each_item([&](int bin, int) { ++offsets[static_cast<size_t>(bin) + 1]; });
    for (int k = 0; k < bin_count; ++k) offsets[static_cast<size_t>(k) + 1] += offsets[static_cast<size_t>(k)];
    items.assign(static_cast<size_t>(offsets.back()), 0);
    std::vector<int> fill(offsets.begin(), offsets.end() - 1);
    for_each_item([&](int bin, int item) { items[static_cast<size_t>(fill[static_cast<size_t>(bin)]++)] = item; });
}

// Up-facing triangles by xy cell, as precomputed edge functions and planes (column_top's hot loop: a few per query);
// each cell's list sorted by the triangles' highest z, so a query stops once no later triangle can be higher.
void column_bins(const Poly& p, std::vector<ColumnTri>& tris, std::vector<int>& offsets, std::vector<int>& items) {
    constexpr int n = column_bins_per_side;
    tris.clear();
    for (size_t k = 0; k < p.triangle.size(); ++k) {
        if (!(p.normal[k].z > 0.0)) continue;
        ColumnTri c;
        c.tri = static_cast<int>(k);
        const auto& t = p.triangle[k];
        for (size_t e = 0; e < 3; ++e) {
            const Vec3& a = p.vertex[static_cast<size_t>(t[e])];
            const Vec3& b = p.vertex[static_cast<size_t>(t[(e + 1) % 3])];
            c.ex[e] = -(b.y - a.y), c.ey[e] = b.x - a.x, c.ec[e] = (b.y - a.y) * a.x - (b.x - a.x) * a.y;
        }
        const Vec3& a = p.vertex[static_cast<size_t>(t[0])];
        const Vec3& nk = p.normal[k];
        c.zx = -nk.x / nk.z, c.zy = -nk.y / nk.z, c.z0 = a.z - c.zx * a.x - c.zy * a.y;
        c.zmin = 1e300, c.zmax = -1e300;
        c.x0 = 1e300, c.x1 = -1e300, c.y0 = 1e300, c.y1 = -1e300;
        for (const int v : t) {
            const Vec3& q = p.vertex[static_cast<size_t>(v)];
            c.zmin = std::min(c.zmin, q.z), c.zmax = std::max(c.zmax, q.z);
            c.x0 = std::min(c.x0, q.x), c.x1 = std::max(c.x1, q.x), c.y0 = std::min(c.y0, q.y), c.y1 = std::max(c.y1, q.y);
        }
        tris.push_back(c);
    }
    fill_bins(n * n, [&](const auto& emit) {
        for (size_t k = 0; k < tris.size(); ++k) {
            const ColumnTri& c = tris[k];
            for (int i = cell_of(c.x0, n); i <= cell_of(c.x1, n); ++i) {
                for (int j = cell_of(c.y0, n); j <= cell_of(c.y1, n); ++j) emit(i * n + j, static_cast<int>(k));
            }
        }
    }, offsets, items);
    for (size_t b = 0; b + 1 < offsets.size(); ++b) {
        std::sort(items.begin() + offsets[b], items.begin() + offsets[b + 1], [&](int u, int v) {
            const double zu = tris[static_cast<size_t>(u)].zmax, zv = tris[static_cast<size_t>(v)].zmax;
            return zu != zv ? zu > zv : u < v;
        });
    }
}

// Highest crossing of the vertical line (x, y) with the up-facing triangles; *tri its triangle (-1: none).
std::optional<double> column_top_on(const std::vector<ColumnTri>& tris, const std::vector<int>& offsets,
                                    const std::vector<int>& items, double x, double y, int* tri = nullptr) {
    if (tri) *tri = -1;
    if (!(x >= -1.0 && x <= 1.0 && y >= -1.0 && y <= 1.0)) return std::nullopt;
    constexpr int n = column_bins_per_side;
    const auto bin = static_cast<size_t>(cell_of(x, n) * n + cell_of(y, n));
    double best = -std::numeric_limits<double>::infinity();
    for (int s = offsets[bin]; s < offsets[bin + 1]; ++s) {
        const ColumnTri& c = tris[static_cast<size_t>(items[static_cast<size_t>(s)])];
        if (c.zmax <= best) break;  // sorted by zmax: nothing later can be higher
        // Edge functions (counter-clockwise from above for an up-facing triangle).
        if (c.ex[0] * x + c.ey[0] * y + c.ec[0] < -point_in_triangle_tolerance ||
            c.ex[1] * x + c.ey[1] * y + c.ec[1] < -point_in_triangle_tolerance ||
            c.ex[2] * x + c.ey[2] * y + c.ec[2] < -point_in_triangle_tolerance) {
            continue;
        }
        const double z = std::clamp(c.z0 + c.zx * x + c.zy * y, c.zmin, c.zmax);
        if (z > best) {
            best = z;
            if (tri) *tri = c.tri;
        }
    }
    if (!std::isfinite(best)) return std::nullopt;
    return best;
}

double poly_volume(const Poly& p) {
    double v = 0.0;
    for (const auto& t : p.triangle) {
        v += dot(p.vertex[static_cast<size_t>(t[0])],
                 cross(p.vertex[static_cast<size_t>(t[1])], p.vertex[static_cast<size_t>(t[2])]));
    }
    return v / 6.0;
}

// Volume per D^3 (D: the diameter of the whole plan-view silhouette's area), volume at a farthest point of 1 and the
// extent ratios along the fragment axes.
RockTemplates::ShapeStats poly_stats(const Poly& p, const Mat3& turn) {
    std::vector<ColumnTri> tris;
    std::vector<int> offsets, items;
    column_bins(p, tris, offsets, items);
    constexpr int columns = RockTemplates::footprint_columns;
    int above = 0;
    for (int i = 0; i < columns; ++i) {
        for (int j = 0; j < columns; ++j) {
            const auto z = column_top_on(tris, offsets, items, -1.0 + (2.0 * i + 1.0) / columns, -1.0 + (2.0 * j + 1.0) / columns);
            if (z) ++above;
        }
    }
    const double cell = 2.0 / columns;
    const double area = above * cell * cell;
    const double d = 2.0 * std::sqrt(area / constants::pi);
    RockTemplates::ShapeStats s;
    double r_max = 0.0;
    for (const Vec3& v : p.vertex) r_max = std::max(r_max, norm(v));
    s.volume_per_d3 = poly_volume(p) / (d * d * d);
    s.unit_volume = poly_volume(p) / (r_max * r_max * r_max);
    std::array<double, 3> width{};
    for (int axis = 0; axis < 3; ++axis) {
        const Vec3 u = math::column(turn, axis);
        double lo = 1e300, hi = -1e300;
        for (const Vec3& v : p.vertex) lo = std::min(lo, dot(u, v)), hi = std::max(hi, dot(u, v));
        width[static_cast<size_t>(axis)] = hi - lo;
    }
    s.extent_b_to_a = width[1] / width[0];
    s.extent_c_to_a = width[2] / width[0];
    return s;
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
Vec3 closest_on_triangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c) {
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) return a;
    const Vec3 bp = p - b;
    const double d3 = dot(ab, bp), d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) return b;
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) return a + ab * (d1 / (d1 - d3));
    const Vec3 cp = p - c;
    const double d5 = dot(ab, cp), d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) return c;
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) return a + ac * (d2 / (d2 - d6));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) return b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
    const double denom = 1.0 / (va + vb + vc);
    return a + ab * (vb * denom) + ac * (vc * denom);
}

// Cube-map cell of a direction: face (0..5: +x, -x, +y, -y, +z, -z) x cells^2.
int cube_cell(const Vec3& u) {
    constexpr int n = support_cells_per_side;
    const double ax = std::abs(u.x), ay = std::abs(u.y), az = std::abs(u.z);
    int face;
    double s, t, m;
    if (ax >= ay && ax >= az) {
        face = u.x >= 0.0 ? 0 : 1, m = ax, s = u.y, t = u.z;
    } else if (ay >= az) {
        face = u.y >= 0.0 ? 2 : 3, m = ay, s = u.z, t = u.x;
    } else {
        face = u.z >= 0.0 ? 4 : 5, m = az, s = u.x, t = u.y;
    }
    return (face * n + cell_of(s / m, n)) * n + cell_of(t / m, n);
}

Vec3 cube_direction(int face, double s, double t) {
    const double sign = face % 2 == 0 ? 1.0 : -1.0;
    switch (face / 2) {
        case 0: return math::normalized(Vec3{sign, s, t});
        case 1: return math::normalized(Vec3{t, sign, s});
        default: return math::normalized(Vec3{s, t, sign});
    }
}

}  // namespace

// ---- Construction ----
RockTemplates::RockTemplates() {
    const Icospheres& ico = icospheres();
    directions_ = ico.vertices;
    level_indices_ = ico.indices;
    for (const auto& idx : level_indices_) {
        auto& level = edge_normals_.emplace_back();
        for (size_t k = 0; k < idx.size(); k += 3) {
            const Vec3& a = directions_[static_cast<size_t>(idx[k])];
            const Vec3& b = directions_[static_cast<size_t>(idx[k + 1])];
            const Vec3& c = directions_[static_cast<size_t>(idx[k + 2])];
            level.push_back({math::normalized(cross(a, b)), math::normalized(cross(b, c)), math::normalized(cross(c, a))});
        }
    }
    const auto build = [&](int id) {
        Shape& shape = shapes_[static_cast<size_t>(id)];
        const auto seed = static_cast<std::uint64_t>(id);
        // Axis ratios, fresh / eroded, primary faces (the Task 11 draws: the same hash fields).
        shape.b_to_a = std::clamp(fragment_b_to_a_mean + fragment_ratio_sd * normal_draw(seed, 1), min_b_to_a, 1.0);
        shape.c_to_a = std::max(
            min_c_to_a, shape.b_to_a * std::clamp(fragment_c_to_b_mean + fragment_ratio_sd * normal_draw(seed, 2), 0.0, 1.0));
        shape.eroded = unit(seed, 3, 0) < eroded_share;
        const int planes = shape.eroded
                               ? eroded_planes_min + std::min(static_cast<int>(unit(seed, 4, 0) * eroded_planes_span),
                                                              eroded_planes_span - 1)
                               : fresh_planes_min + std::min(static_cast<int>(unit(seed, 4, 0) * fresh_planes_span),
                                                             fresh_planes_span - 1);
        shape.primary_planes = planes;
        const Field field = make_field(seed, shape.eroded, Vec3{1.0, shape.b_to_a, shape.c_to_a}, planes,
                                       shape.eroded ? eroded_chip_min : fresh_chip_min,
                                       shape.eroded ? eroded_chip_span : fresh_chip_span, &shape.chips, &shape.steps,
                                       &shape.pits);
        shape.erosion_radius = field.rho;
        // Orientation: fragment frame -> template frame.
        const double tilt_x = rms_tilt_rad / std::sqrt(2.0) * normal_draw(seed, 9);
        const double tilt_y = rms_tilt_rad / std::sqrt(2.0) * normal_draw(seed, 10);
        const double tilt = std::min(std::hypot(tilt_x, tilt_y), max_tilt_rad);
        const Mat3 turn = rotation_z(std::atan2(tilt_y, tilt_x)) * rotation_y(tilt) *
                          rotation_z(constants::two_pi * unit(seed, 11, 0));
        const Mat3 back = math::transpose(turn);
        shape.tilt_rad = std::acos(std::min(1.0, std::abs(turn.m[2][2])));

        // The relief removes material unevenly along the fragment axes; a stretch S along them (fragment frame)
        // restores the relief-free fragment's extent ratios, so the axis law holds for the drawn shape: the body S B
        // has the radius r_B(S^-1 d / |S^-1 d|) / |S^-1 d|.
        const int n0 = vertices_at_level(finest_level);
        std::vector<double> r(static_cast<size_t>(n0)), r_base(static_cast<size_t>(n0));
        Vec3 lo_relief{1e300, 1e300, 1e300}, hi_relief{-1e300, -1e300, -1e300}, lo_base = lo_relief, hi_base = hi_relief;
        for (int k = 0; k < n0; ++k) {
            const Vec3 d = back * directions_[static_cast<size_t>(k)];
            const Vec3 p = d * field.radius(d, true), q = d * (r_base[static_cast<size_t>(k)] = field.radius(d, false));
            lo_relief = {std::min(lo_relief.x, p.x), std::min(lo_relief.y, p.y), std::min(lo_relief.z, p.z)};
            hi_relief = {std::max(hi_relief.x, p.x), std::max(hi_relief.y, p.y), std::max(hi_relief.z, p.z)};
            lo_base = {std::min(lo_base.x, q.x), std::min(lo_base.y, q.y), std::min(lo_base.z, q.z)};
            hi_base = {std::max(hi_base.x, q.x), std::max(hi_base.y, q.y), std::max(hi_base.z, q.z)};
        }
        const Vec3 stretch{(hi_base.x - lo_base.x) / (hi_relief.x - lo_relief.x),
                           (hi_base.y - lo_base.y) / (hi_relief.y - lo_relief.y),
                           (hi_base.z - lo_base.z) / (hi_relief.z - lo_relief.z)};
        const auto relief_radius = [&](const Vec3& d) {  // d: unit, fragment frame
            const Vec3 q{d.x / stretch.x, d.y / stretch.y, d.z / stretch.z};
            const double len = norm(q);
            return field.radius(q / len, true) / len;
        };
        double r_max = 0.0;
        for (int k = 0; k < n0; ++k) {
            r[static_cast<size_t>(k)] = relief_radius(back * directions_[static_cast<size_t>(k)]);
            r_max = std::max(r_max, r[static_cast<size_t>(k)]);
        }
        // Scale so that the farthest vertex is at r = 1.
        const double scale = 1.0 / r_max;
        std::vector<Vec3> vertex(static_cast<size_t>(n0)), base_vertex(static_cast<size_t>(n0));
        for (int k = 0; k < n0; ++k) {
            vertex[static_cast<size_t>(k)] = directions_[static_cast<size_t>(k)] * (r[static_cast<size_t>(k)] * scale);
            base_vertex[static_cast<size_t>(k)] = directions_[static_cast<size_t>(k)] * r_base[static_cast<size_t>(k)];
        }
        const Poly poly = make_poly(vertex, level_indices_[finest_level]);
        shape.relief = poly_stats(poly, turn);
        shape.base = poly_stats(make_poly(base_vertex, level_indices_[finest_level]), turn);
        shape.volume = poly_volume(poly);
        shape.vertex = poly.vertex;
        shape.triangle = poly.triangle;
        shape.triangle_normal = poly.normal;
        shape.bottom_z = 1e300, shape.top_z = -1e300;
        for (const Vec3& v : shape.vertex) shape.bottom_z = std::min(shape.bottom_z, v.z), shape.top_z = std::max(shape.top_z, v.z);

        // Query structures.
        column_bins(poly, shape.column_tris, shape.column_bins.offsets, shape.column_bins.items);
        {
            constexpr int n = space_bins_per_side;
            fill_bins(n * n * n, [&](const auto& emit) {
                for (size_t k = 0; k < shape.triangle.size(); ++k) {
                    Vec3 lo{1e300, 1e300, 1e300}, hi{-1e300, -1e300, -1e300};
                    for (const int v : shape.triangle[k]) {
                        const Vec3& q = shape.vertex[static_cast<size_t>(v)];
                        lo = {std::min(lo.x, q.x), std::min(lo.y, q.y), std::min(lo.z, q.z)};
                        hi = {std::max(hi.x, q.x), std::max(hi.y, q.y), std::max(hi.z, q.z)};
                    }
                    for (int i = cell_of(lo.x, n); i <= cell_of(hi.x, n); ++i) {
                        for (int j = cell_of(lo.y, n); j <= cell_of(hi.y, n); ++j) {
                            for (int l = cell_of(lo.z, n); l <= cell_of(hi.z, n); ++l) {
                                emit((i * n + j) * n + l, static_cast<int>(k));
                            }
                        }
                    }
                }
            }, shape.space_bins.offsets, shape.space_bins.items);
        }
        {
            // Support candidates per cube-map cell: for u within the cell (angle <= alpha from the centre u0) the
            // maximum of u . v can only be at a vertex v with u0 . (v - w0) >= -sin(alpha) |v - w0| (w0 the maximum at
            // u0): otherwise u . (v - w0) < 0 for every such u. A vertex inside a planar region (every incident
            // triangle with one normal) lies in the convex hull of its ring: some ring vertex reaches at least its
            // value for every u, so it is left out (the flat faces' interiors would fill the lists).
            std::vector<Vec3> first_normal(static_cast<size_t>(n0));
            std::vector<char> seen(static_cast<size_t>(n0), 0), flat(static_cast<size_t>(n0), 1);
            for (size_t k = 0; k < shape.triangle.size(); ++k) {
                for (const int v : shape.triangle[k]) {
                    const auto vi = static_cast<size_t>(v);
                    if (!seen[vi]) {
                        seen[vi] = 1, first_normal[vi] = shape.triangle_normal[k];
                    } else if (norm(first_normal[vi] - shape.triangle_normal[k]) > 1e-12) {
                        flat[vi] = 0;
                    }
                }
            }
            constexpr int n = support_cells_per_side;
            std::vector<std::vector<int>> lists(static_cast<size_t>(6 * n * n));
            for (int face = 0; face < 6; ++face) {
                for (int i = 0; i < n; ++i) {
                    for (int j = 0; j < n; ++j) {
                        const double s0 = -1.0 + 2.0 * i / n, s1 = -1.0 + 2.0 * (i + 1) / n;
                        const double t0 = -1.0 + 2.0 * j / n, t1 = -1.0 + 2.0 * (j + 1) / n;
                        const Vec3 u0 = cube_direction(face, 0.5 * (s0 + s1), 0.5 * (t0 + t1));
                        double alpha = 0.0;
                        for (const auto& [s, t] : {std::pair{s0, t0}, std::pair{s0, t1}, std::pair{s1, t0}, std::pair{s1, t1}}) {
                            alpha = std::max(alpha, std::acos(std::clamp(dot(u0, cube_direction(face, s, t)), -1.0, 1.0)));
                        }
                        const double sin_alpha = std::sin(std::min(alpha * 1.01 + 1e-9, 0.5 * constants::pi));
                        int w0 = 0;
                        for (int k = 1; k < n0; ++k) {
                            if (dot(u0, shape.vertex[static_cast<size_t>(k)]) > dot(u0, shape.vertex[static_cast<size_t>(w0)])) w0 = k;
                        }
                        auto& list = lists[static_cast<size_t>((face * n + i) * n + j)];
                        const Vec3 w = shape.vertex[static_cast<size_t>(w0)];
                        for (int k = 0; k < n0; ++k) {
                            const Vec3 x = shape.vertex[static_cast<size_t>(k)] - w;
                            if (k != w0 && flat[static_cast<size_t>(k)]) continue;
                            if (dot(u0, x) >= -sin_alpha * norm(x) - 1e-12) list.push_back(k);
                        }
                    }
                }
            }
            fill_bins(6 * n * n, [&](const auto& emit) {
                for (size_t b = 0; b < lists.size(); ++b) {
                    for (const int k : lists[b]) emit(static_cast<int>(b), k);
                }
            }, shape.support_bins.offsets, shape.support_bins.items);
        }
        // Footprint grid: the top of every column, for the exposed plan-view area at any cut height.
        shape.column_grid.assign(static_cast<size_t>(footprint_columns * footprint_columns),
                                 std::numeric_limits<double>::quiet_NaN());
        for (int i = 0; i < footprint_columns; ++i) {
            for (int j = 0; j < footprint_columns; ++j) {
                const double x = -1.0 + (2.0 * i + 1.0) / footprint_columns;
                const double y = -1.0 + (2.0 * j + 1.0) / footprint_columns;
                if (const auto top = column_top_on(shape.column_tris, shape.column_bins.offsets, shape.column_bins.items, x, y)) {
                    shape.column_tops.push_back(*top);
                    shape.column_grid[static_cast<size_t>(i * footprint_columns + j)] = *top;
                }
            }
        }
        std::sort(shape.column_tops.begin(), shape.column_tops.end());

        // Render meshes: the LOD's prefix of the vertices, normals and cavities from the field smoothed over the
        // LOD's own vertex spacing.
        const auto surface_point = [&](const Vec3& d) {
            const Vec3 u = math::normalized(d);
            return u * (relief_radius(back * u) * scale);
        };
        for (int lod = 0; lod < lods; ++lod) {
            const int level = lod_levels[static_cast<size_t>(lod)];
            const int nv = vertices_at_level(level);
            const double step = normal_step_per_edge * icosahedron_edge_rad / (1 << level);
            const double ring = std::max(2.0 * step, cavity_min_angle_rad);
            TemplateMesh& mesh = meshes_[static_cast<size_t>(id)][static_cast<size_t>(lod)];
            mesh.indices = level_indices_[static_cast<size_t>(level)];
            mesh.vertices.assign(vertex.begin(), vertex.begin() + nv);
            for (int k = 0; k < nv; ++k) {
                const Vec3 d = directions_[static_cast<size_t>(k)];
                const Vec3 e1 = any_perpendicular(d), e2 = cross(d, e1);
                const Vec3 n = math::normalized(cross(surface_point(d + e1 * step) - surface_point(d - e1 * step),
                                                      surface_point(d + e2 * step) - surface_point(d - e2 * step)));
                const Vec3 outward = dot(n, d) > 0.0 ? n : n * -1.0;
                mesh.normals.push_back(outward);
                const Vec3 p = vertex[static_cast<size_t>(k)];
                double elevation = 0.0;
                for (int q = 0; q < cavity_ring; ++q) {
                    const double phi = constants::two_pi * q / cavity_ring;
                    const Vec3 x = surface_point(d + (e1 * std::cos(phi) + e2 * std::sin(phi)) * ring) - p;
                    elevation += dot(x, outward) / std::max(norm(x), 1e-12);
                }
                mesh.cavity.push_back(std::clamp(elevation / cavity_ring / cavity_full_sine, 0.0, 1.0));
            }
        }
    };
    std::vector<std::thread> workers;
    for (int id = 0; id < count; ++id) workers.emplace_back(build, id);
    for (std::thread& w : workers) w.join();
}

const RockTemplates& RockTemplates::instance() {
    static const RockTemplates templates;
    return templates;
}

// ---- Queries on the LOD-0 polyhedron ----
int RockTemplates::find_triangle(const Vec3& d) const {
    // Descend the icosphere hierarchy: at each level the child whose cone holds d (the most interior one: robust on
    // the shared edges).
    const auto score = [&](int level, int tri) {
        const auto& e = edge_normals_[static_cast<size_t>(level)][static_cast<size_t>(tri)];
        return std::min({dot(d, e[0]), dot(d, e[1]), dot(d, e[2])});
    };
    int best = 0;
    double best_score = -1e300;
    for (int k = 0; k < 20; ++k) {
        if (const double s = score(0, k); s > best_score) best_score = s, best = k;
    }
    for (int level = 1; level <= finest_level; ++level) {
        const int parent = best;
        best_score = -1e300;
        for (int c = 0; c < 4; ++c) {
            if (const double s = score(level, 4 * parent + c); s > best_score) {
                best_score = s, best = 4 * parent + c;
            }
        }
    }
    return best;
}

double RockTemplates::radius(int id, const Vec3& dir_local) const {
    const Shape& shape = shapes_[static_cast<size_t>(id)];
    const Vec3 d = math::normalized(dir_local);
    const int t = find_triangle(d);
    const Vec3& n = shape.triangle_normal[static_cast<size_t>(t)];
    return dot(n, shape.vertex[static_cast<size_t>(shape.triangle[static_cast<size_t>(t)][0])]) / dot(n, d);
}

Vec3 RockTemplates::normal(int id, const Vec3& dir_local) const {
    return shapes_[static_cast<size_t>(id)].triangle_normal[static_cast<size_t>(find_triangle(math::normalized(dir_local)))];
}

Vec3 RockTemplates::normal_toward(int id, const Vec3& p, const Vec3& reference, double tolerance) const {
    const Shape& shape = shapes_[static_cast<size_t>(id)];
    constexpr int n = space_bins_per_side;
    std::vector<int> found;
    std::vector<Vec3> cone;
    for (int i = cell_of(p.x - tolerance, n); i <= cell_of(p.x + tolerance, n); ++i) {
        for (int j = cell_of(p.y - tolerance, n); j <= cell_of(p.y + tolerance, n); ++j) {
            for (int l = cell_of(p.z - tolerance, n); l <= cell_of(p.z + tolerance, n); ++l) {
                const auto bin = static_cast<size_t>((i * n + j) * n + l);
                for (int s = shape.space_bins.offsets[bin]; s < shape.space_bins.offsets[bin + 1]; ++s) {
                    const int k = shape.space_bins.items[static_cast<size_t>(s)];
                    if (std::find(found.begin(), found.end(), k) != found.end()) continue;
                    const auto& t = shape.triangle[static_cast<size_t>(k)];
                    // Farther than the tolerance from the triangle's plane: farther from the triangle (cheap reject).
                    if (std::abs(dot(shape.triangle_normal[static_cast<size_t>(k)],
                                     p - shape.vertex[static_cast<size_t>(t[0])])) > tolerance) {
                        continue;
                    }
                    const Vec3 q = closest_on_triangle(p, shape.vertex[static_cast<size_t>(t[0])],
                                                       shape.vertex[static_cast<size_t>(t[1])],
                                                       shape.vertex[static_cast<size_t>(t[2])]);
                    if (norm(q - p) <= tolerance) {
                        found.push_back(k);
                        const Vec3& nk = shape.triangle_normal[static_cast<size_t>(k)];
                        bool known = false;
                        for (const Vec3& c : cone) known = known || norm(c - nk) < 1e-12;
                        if (!known) cone.push_back(nk);
                    }
                }
            }
        }
    }
    if (cone.empty()) return normal(id, p);
    const Vec3 r = math::normalized(reference);
    // The unit vector of cone{n_i} closest to r: r itself when inside, else the best of the generators and of r's
    // projections onto the planes of generator pairs that fall between the pair.
    Vec3 best = cone.front();
    double best_dot = -2.0;
    const auto offer = [&](const Vec3& c) {
        if (const double d = dot(c, r); d > best_dot) best_dot = d, best = c;
    };
    for (const Vec3& c : cone) offer(c);
    for (size_t i = 0; i < cone.size(); ++i) {
        for (size_t j = i + 1; j < cone.size(); ++j) {
            const Vec3 w = cross(cone[i], cone[j]);
            if (dot(w, w) < parallel_limit) continue;
            const Vec3 in_plane = r - w * (dot(r, w) / dot(w, w));
            if (norm(in_plane) < parallel_limit) continue;
            const Vec3 c = math::normalized(in_plane);
            // Between n_i and n_j: c = a n_i + b n_j with a, b >= 0.
            if (dot(cross(cone[i], c), w) >= 0.0 && dot(cross(c, cone[j]), w) >= 0.0) offer(c);
        }
    }
    for (size_t i = 0; i < cone.size(); ++i) {
        for (size_t j = i + 1; j < cone.size(); ++j) {
            for (size_t k = j + 1; k < cone.size(); ++k) {
                const auto lambda = math::solve(Mat3::from_columns(cone[i], cone[j], cone[k]), r);
                if (lambda && lambda->x >= 0.0 && lambda->y >= 0.0 && lambda->z >= 0.0) return r;
            }
        }
    }
    return best;
}

std::optional<double> RockTemplates::column_top(int id, double x, double y) const {
    const Shape& shape = shapes_[static_cast<size_t>(id)];
    return column_top_on(shape.column_tris, shape.column_bins.offsets, shape.column_bins.items, x, y);
}

std::optional<std::pair<double, Vec3>> RockTemplates::column_hit(int id, double x, double y) const {
    const Shape& shape = shapes_[static_cast<size_t>(id)];
    int tri = -1;
    const auto top = column_top_on(shape.column_tris, shape.column_bins.offsets, shape.column_bins.items, x, y, &tri);
    if (!top) return std::nullopt;
    return std::pair{*top, shape.triangle_normal[static_cast<size_t>(tri)]};
}

Vec3 RockTemplates::support_point(int id, const Vec3& u) const {
    const Shape& shape = shapes_[static_cast<size_t>(id)];
    const auto bin = static_cast<size_t>(cube_cell(u));
    const auto& offsets = shape.support_bins.offsets;
    int best = -1;
    double best_value = -std::numeric_limits<double>::infinity();
    for (int s = offsets[bin]; s < offsets[bin + 1]; ++s) {
        const int k = shape.support_bins.items[static_cast<size_t>(s)];
        if (const double v = dot(u, shape.vertex[static_cast<size_t>(k)]); v > best_value) best_value = v, best = k;
    }
    if (best < 0) throw std::logic_error("RockTemplates: no support candidate");
    return shape.vertex[static_cast<size_t>(best)];
}

double RockTemplates::exposed_area(int id, double cut_z) const {
    const auto& tops = shapes_[static_cast<size_t>(id)].column_tops;
    const auto above = tops.end() - std::upper_bound(tops.begin(), tops.end(), cut_z);
    const double cell = 2.0 / footprint_columns;
    return static_cast<double>(above) * cell * cell;
}

const TemplateMesh& RockTemplates::mesh(int id, int lod) const {
    return meshes_[static_cast<size_t>(id)][static_cast<size_t>(lod)];
}

}  // namespace starsim::environment
