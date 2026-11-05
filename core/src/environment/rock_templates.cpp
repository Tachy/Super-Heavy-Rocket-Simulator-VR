#include "starsim/environment/rock_templates.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

#include "starsim/environment/surface_hash.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Mat3;
using math::Vec3;

namespace {

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
constexpr int fresh_planes_min = 16, fresh_planes_span = 9;             // [E] 16-24 fracture faces: polyhedral
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

constexpr int normal_draws = 6;           // normal-like draws: sum of six uniforms (no library distributions)
constexpr int sample_points = 20000;      // Fibonacci directions that find the maximum radius
constexpr int refined_candidates = 32;    // best samples refined by a pattern search on the sphere
constexpr double refine_start_step = 0.02, refine_end_step = 1e-10;  // rad
// The refined maximum is a local maximum of the best samples; a margin keeps r <= 1 against a missed corner.
constexpr double scale_margin = 1.005;
constexpr double parallel_limit = 1e-15;  // |n_z| or |n . d| below this: the constraint does not bound the line
constexpr double feasible_tolerance = 1e-9;
constexpr std::array<int, RockTemplates::lods> subdivisions = {4, 3, 2, 1};  // icosphere levels per LOD [E]

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

// Pattern search for a maximum of f over the unit sphere from d (8 tangent directions, halving steps).
template <class F>
Vec3 refine_on_sphere(const F& f, Vec3 d) {
    double best = f(d), step = refine_start_step;
    while (step > refine_end_step) {
        const Vec3 t1 = any_perpendicular(d), t2 = cross(d, t1);
        bool moved = false;
        for (int k = 0; k < 8; ++k) {
            const double angle = constants::two_pi * k / 8.0;
            const Vec3 trial = math::normalized(d + (t1 * std::cos(angle) + t2 * std::sin(angle)) * step);
            const double value = f(trial);
            if (value > best) {
                best = value, d = trial, moved = true;
                break;
            }
        }
        if (!moved) step *= 0.5;
    }
    return d;
}

// Icosphere: subdivided icosahedron, shared vertices (watertight), vertices on the unit sphere.
struct Icosphere {
    std::vector<Vec3> vertices;
    std::vector<int> indices;
};

Icosphere make_icosphere(int subdivision) {
    const double t = 0.5 * (1.0 + std::sqrt(5.0));
    Icosphere s;
    for (const Vec3& v : {Vec3{-1, t, 0}, Vec3{1, t, 0}, Vec3{-1, -t, 0}, Vec3{1, -t, 0}, Vec3{0, -1, t}, Vec3{0, 1, t},
                          Vec3{0, -1, -t}, Vec3{0, 1, -t}, Vec3{t, 0, -1}, Vec3{t, 0, 1}, Vec3{-t, 0, -1},
                          Vec3{-t, 0, 1}}) {
        s.vertices.push_back(math::normalized(v));
    }
    s.indices = {0, 11, 5, 0, 5, 1,  0, 1,  7,  0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11, 4,  11, 10, 2,  10, 7, 6, 7, 1, 8,
                 3, 9,  4, 3, 4, 2,  3, 2,  6,  3, 6,  8,  3, 8,  9,  4, 9, 5, 2, 4,  11, 6,  2,  10, 8,  6, 7, 9, 8, 1};
    for (int level = 0; level < subdivision; ++level) {
        std::map<std::pair<int, int>, int> midpoint;
        const auto middle = [&](int a, int b) {
            const std::pair<int, int> key{std::min(a, b), std::max(a, b)};
            const auto it = midpoint.find(key);
            if (it != midpoint.end()) return it->second;
            s.vertices.push_back(math::normalized(s.vertices[static_cast<size_t>(a)] + s.vertices[static_cast<size_t>(b)]));
            return midpoint[key] = static_cast<int>(s.vertices.size()) - 1;
        };
        std::vector<int> next;
        for (size_t k = 0; k < s.indices.size(); k += 3) {
            const int a = s.indices[k], b = s.indices[k + 1], c = s.indices[k + 2];
            const int ab = middle(a, b), bc = middle(b, c), ca = middle(c, a);
            for (const int tri : {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca}) next.push_back(tri);
        }
        s.indices = std::move(next);
    }
    return s;
}

// Fragment-frame description used during construction: unit-ball coordinates q = p / s (s the semi-axes), planes
// m . q <= h (m = n * s componentwise).
struct BallBody {
    std::vector<Vec3> m;
    std::vector<double> h;
};

bool feasible(const BallBody& body, const Vec3& q, int skip_a = -1, int skip_b = -1, int skip_c = -1) {
    for (size_t k = 0; k < body.m.size(); ++k) {
        const int ki = static_cast<int>(k);
        if (ki == skip_a || ki == skip_b || ki == skip_c) continue;
        if (dot(body.m[k], q) > body.h[k] + feasible_tolerance) return false;
    }
    return true;
}

// The extreme points of the unit ball cut by the planes that do not depend on the direction: the ends of every two-plane
// edge on the sphere and every vertex of three planes (feasible ones). With the ball's own maximum and the maximum on
// each plane's circle (direction-dependent, RockTemplates::support_point) they hold the maximum of any linear function.
std::vector<Vec3> fixed_extreme_points(const BallBody& body) {
    std::vector<Vec3> out;
    const size_t n = body.m.size();
    for (size_t k = 0; k < n; ++k) {
        for (size_t l = k + 1; l < n; ++l) {
            const Vec3 &a = body.m[k], &b = body.m[l];
            const Vec3 w = cross(a, b);
            const double w2 = dot(w, w);
            if (w2 < parallel_limit) continue;
            const double ha = body.h[k], hb = body.h[l], ab = dot(a, b);
            const Vec3 q0 = (a * (ha * dot(b, b) - hb * ab) + b * (hb * dot(a, a) - ha * ab)) / w2;
            const Vec3 wh = w / std::sqrt(w2);
            const double p = dot(q0, wh), c = dot(q0, q0) - 1.0, disc = p * p - c;
            if (disc < 0.0) continue;
            for (const double t : {-p - std::sqrt(disc), -p + std::sqrt(disc)}) {
                const Vec3 q = q0 + wh * t;
                if (feasible(body, q, static_cast<int>(k), static_cast<int>(l))) out.push_back(q);
            }
        }
    }
    for (size_t k = 0; k < n; ++k) {
        for (size_t l = k + 1; l < n; ++l) {
            for (size_t j = l + 1; j < n; ++j) {
                const auto q = math::solve(math::transpose(Mat3::from_columns(body.m[k], body.m[l], body.m[j])),
                                           Vec3{body.h[k], body.h[l], body.h[j]});
                if (!q || dot(*q, *q) > 1.0 + feasible_tolerance) continue;
                if (feasible(body, *q, static_cast<int>(k), static_cast<int>(l), static_cast<int>(j))) out.push_back(*q);
            }
        }
    }
    return out;
}

}  // namespace

double RockTemplates::exit_distance(const Shape& shape, const Vec3& d, int* active) {
    double t = 1.0 / std::sqrt(dot(d, shape.quadric * d));
    int which = -1;
    for (int k = 0; k < shape.planes; ++k) {
        const double nd = dot(shape.normal[static_cast<size_t>(k)], d);
        if (nd > parallel_limit) {
            const double tk = shape.distance[static_cast<size_t>(k)] / nd;
            if (tk < t) t = tk, which = k;
        }
    }
    if (active) *active = which;
    return t;
}

Vec3 RockTemplates::support_point(const Shape& shape, const Vec3& u) {
    const auto inside = [&](const Vec3& p, int skip) {
        for (int k = 0; k < shape.planes; ++k) {
            if (k != skip && dot(shape.normal[static_cast<size_t>(k)], p) >
                                 shape.distance[static_cast<size_t>(k)] + feasible_tolerance) {
                return false;
            }
        }
        return true;
    };
    const Vec3 v = math::transpose(shape.ball_to_template) * u;
    if (const Vec3 p = shape.ball_to_template * math::normalized(v); inside(p, -1)) return p;  // the ellipsoid's own
    Vec3 best_p;
    double best = -std::numeric_limits<double>::infinity();
    const auto consider = [&](const Vec3& p) {
        if (const double value = dot(u, p); value > best) best = value, best_p = p;
    };
    for (const Vec3& p : shape.fixed_points) consider(p);
    for (int k = 0; k < shape.planes; ++k) {  // the maximum on each plane's circle
        const Vec3& m = shape.ball_normal[static_cast<size_t>(k)];
        const double len = norm(m), d = shape.distance[static_cast<size_t>(k)] / len;
        if (d >= 1.0) continue;
        const Vec3 mh = m / len;
        const Vec3 perp = v - mh * dot(v, mh);
        const double perp_len = norm(perp);
        const Vec3 q = mh * d + (perp_len > parallel_limit ? perp / perp_len : any_perpendicular(mh)) *
                                    std::sqrt(1.0 - d * d);
        const Vec3 p = shape.ball_to_template * q;
        if (dot(u, p) > best && inside(p, k)) consider(p);
    }
    if (!std::isfinite(best)) throw std::logic_error("RockTemplates: no extreme point found");
    return best_p;
}

Vec3 RockTemplates::support_point(int id, const Vec3& u) const {
    return support_point(shapes_[static_cast<size_t>(id)], u);
}

std::optional<double> RockTemplates::column_top(const Shape& shape, double x, double y) {
    double lo = -std::numeric_limits<double>::infinity(), hi = std::numeric_limits<double>::infinity();
    const Mat3& q = shape.quadric;
    const double a = q.m[2][2], b = 2.0 * (q.m[0][2] * x + q.m[1][2] * y);
    const double c = q.m[0][0] * x * x + 2.0 * q.m[0][1] * x * y + q.m[1][1] * y * y - 1.0;
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return std::nullopt;
    lo = (-b - std::sqrt(disc)) / (2.0 * a);
    hi = (-b + std::sqrt(disc)) / (2.0 * a);
    for (int k = 0; k < shape.planes; ++k) {
        const Vec3& n = shape.normal[static_cast<size_t>(k)];
        const double rhs = shape.distance[static_cast<size_t>(k)] - n.x * x - n.y * y;
        if (n.z > parallel_limit) {
            hi = std::min(hi, rhs / n.z);
        } else if (n.z < -parallel_limit) {
            lo = std::max(lo, rhs / n.z);
        } else if (rhs < 0.0) {
            return std::nullopt;
        }
    }
    if (hi < lo) return std::nullopt;
    return hi;
}

RockTemplates::RockTemplates() {
    for (int id = 0; id < count; ++id) {
        Shape& shape = shapes_[static_cast<size_t>(id)];
        const auto seed = static_cast<std::uint64_t>(id);
        // Axis ratios, fresh / eroded, faces.
        shape.b_to_a = std::clamp(fragment_b_to_a_mean + fragment_ratio_sd * normal_draw(seed, 1), min_b_to_a, 1.0);
        shape.c_to_a = std::max(
            min_c_to_a, shape.b_to_a * std::clamp(fragment_c_to_b_mean + fragment_ratio_sd * normal_draw(seed, 2), 0.0, 1.0));
        shape.eroded = unit(seed, 3, 0) < eroded_share;
        const int planes = shape.eroded
                               ? eroded_planes_min + std::min(static_cast<int>(unit(seed, 4, 0) * eroded_planes_span),
                                                              eroded_planes_span - 1)
                               : fresh_planes_min + std::min(static_cast<int>(unit(seed, 4, 0) * fresh_planes_span),
                                                             fresh_planes_span - 1);
        const double chip_min = shape.eroded ? eroded_chip_min : fresh_chip_min;
        const double chip_span = shape.eroded ? eroded_chip_span : fresh_chip_span;
        shape.planes = planes;
        const Vec3 semi{1.0, shape.b_to_a, shape.c_to_a};
        // Faces: tangent planes of the ellipsoid at a jittered Fibonacci set of points s nu (nu on the unit sphere, so
        // the faces are spread over the surface, rims of flat fragments included), turned at random and pushed in.
        const Mat3 face_turn = uniform_rotation(unit(seed, 5, 0), unit(seed, 5, 1), unit(seed, 5, 2));
        BallBody ball;
        std::array<Vec3, max_planes> fragment_normal{};
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
            const double h = support_e * (1.0 - (chip_min + chip_span * unit(seed, 8, ku)));
            fragment_normal[static_cast<size_t>(k)] = n;
            shape.distance[static_cast<size_t>(k)] = h;
            ball.m.push_back(Vec3{n.x * semi.x, n.y * semi.y, n.z * semi.z});
            ball.h.push_back(h);
        }
        // Orientation: fragment frame -> template frame.
        const double tilt_x = rms_tilt_rad / std::sqrt(2.0) * normal_draw(seed, 9);
        const double tilt_y = rms_tilt_rad / std::sqrt(2.0) * normal_draw(seed, 10);
        const double tilt = std::min(std::hypot(tilt_x, tilt_y), max_tilt_rad);
        const Mat3 turn = rotation_z(std::atan2(tilt_y, tilt_x)) * rotation_y(tilt) *
                          rotation_z(constants::two_pi * unit(seed, 11, 0));
        shape.tilt_rad = std::acos(std::min(1.0, std::abs(turn.m[2][2])));
        for (int k = 0; k < planes; ++k) {
            shape.normal[static_cast<size_t>(k)] = turn * fragment_normal[static_cast<size_t>(k)];
        }
        const Mat3 inverse_s2 = Mat3::from_columns({1.0 / (semi.x * semi.x), 0, 0}, {0, 1.0 / (semi.y * semi.y), 0},
                                                   {0, 0, 1.0 / (semi.z * semi.z)});
        shape.quadric = turn * inverse_s2 * math::transpose(turn);

        // Scale to max r = 1: the best Fibonacci samples refined on the sphere.
        std::vector<std::pair<double, int>> samples;
        samples.reserve(sample_points);
        for (int n = 0; n < sample_points; ++n) {
            samples.emplace_back(exit_distance(shape, fibonacci(n, sample_points), nullptr), n);
        }
        std::partial_sort(samples.begin(), samples.begin() + refined_candidates, samples.end(),
                          [](const auto& a, const auto& b) { return a.first > b.first; });
        double max_r = 0.0;
        const auto r_of = [&](const Vec3& d) { return exit_distance(shape, d, nullptr); };
        for (int c = 0; c < refined_candidates; ++c) {
            max_r = std::max(max_r, r_of(refine_on_sphere(r_of, fibonacci(samples[static_cast<size_t>(c)].second,
                                                                          sample_points))));
        }
        const double scale = 1.0 / (max_r * scale_margin);
        for (int k = 0; k < planes; ++k) {
            shape.distance[static_cast<size_t>(k)] *= scale;
            ball.h[static_cast<size_t>(k)] *= scale;
        }
        for (auto& row : shape.quadric.m) {
            for (double& v : row) v /= scale * scale;
        }
        // Ball coordinates of the scaled body (p = L q, |q| <= 1 the ellipsoid; planes m . q <= h with m = L^T n) and
        // its fixed extreme points, for support_point; the vertical extent is the support along +-z.
        shape.ball_to_template = turn * Mat3::from_columns({semi.x * scale, 0, 0}, {0, semi.y * scale, 0},
                                                           {0, 0, semi.z * scale});
        for (int k = 0; k < planes; ++k) {
            ball.m[static_cast<size_t>(k)] = math::transpose(shape.ball_to_template) * shape.normal[static_cast<size_t>(k)];
            shape.ball_normal[static_cast<size_t>(k)] = ball.m[static_cast<size_t>(k)];
        }
        for (const Vec3& q : fixed_extreme_points(ball)) shape.fixed_points.push_back(shape.ball_to_template * q);
        shape.top_z = support_point(shape, {0, 0, 1}).z;
        shape.bottom_z = support_point(shape, {0, 0, -1}).z;

        // Footprint grid: the top of every column, for the exposed plan-view area at any cut height.
        shape.column_grid.assign(static_cast<size_t>(footprint_columns * footprint_columns),
                                 std::numeric_limits<double>::quiet_NaN());
        for (int i = 0; i < footprint_columns; ++i) {
            for (int j = 0; j < footprint_columns; ++j) {
                const double x = -1.0 + (2.0 * i + 1.0) / footprint_columns;
                const double y = -1.0 + (2.0 * j + 1.0) / footprint_columns;
                if (const auto top = column_top(shape, x, y)) {
                    shape.column_tops.push_back(*top);
                    shape.column_grid[static_cast<size_t>(i * footprint_columns + j)] = *top;
                }
            }
        }
        std::sort(shape.column_tops.begin(), shape.column_tops.end());

        for (int lod = 0; lod < lods; ++lod) {
            const Icosphere ico = make_icosphere(subdivisions[static_cast<size_t>(lod)]);
            TemplateMesh& mesh = meshes_[static_cast<size_t>(id)][static_cast<size_t>(lod)];
            mesh.indices = ico.indices;
            for (const Vec3& d : ico.vertices) {
                mesh.vertices.push_back(d * radius(id, d));
                mesh.normals.push_back(normal(id, d));
            }
        }
    }
}

const RockTemplates& RockTemplates::instance() {
    static const RockTemplates templates;
    return templates;
}

double RockTemplates::radius(int id, const Vec3& dir_local) const {
    return exit_distance(shapes_[static_cast<size_t>(id)], math::normalized(dir_local), nullptr);
}

Vec3 RockTemplates::normal(int id, const Vec3& dir_local) const {
    const Shape& shape = shapes_[static_cast<size_t>(id)];
    const Vec3 d = math::normalized(dir_local);
    int active = -1;
    const double t = exit_distance(shape, d, &active);
    if (active >= 0) return shape.normal[static_cast<size_t>(active)];
    return math::normalized(shape.quadric * (d * t));
}

Vec3 RockTemplates::normal_toward(int id, const Vec3& p, const Vec3& reference, double tolerance) const {
    const Shape& shape = shapes_[static_cast<size_t>(id)];
    std::vector<Vec3> cone;
    for (int k = 0; k < shape.planes; ++k) {
        const Vec3& n = shape.normal[static_cast<size_t>(k)];
        if (shape.distance[static_cast<size_t>(k)] - dot(n, p) <= tolerance) cone.push_back(n);
    }
    const Vec3 qp = shape.quadric * p;
    const double s = std::sqrt(std::max(0.0, dot(p, qp)));
    if (s > 0.0 && (1.0 - s) * s / norm(qp) <= tolerance) cone.push_back(math::normalized(qp));
    if (cone.empty()) return normal(id, p);
    const Vec3 r = math::normalized(reference);
    // The unit vector of cone{n_i} closest to r: r itself when inside, else the best of the generators and of r's
    // projections onto the planes of generator pairs that fall between the pair.
    Vec3 best = cone.front();
    double best_dot = -2.0;
    const auto offer = [&](const Vec3& c) {
        if (const double d = dot(c, r); d > best_dot) best_dot = d, best = c;
    };
    for (const Vec3& n : cone) offer(n);
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
    return column_top(shapes_[static_cast<size_t>(id)], x, y);
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
