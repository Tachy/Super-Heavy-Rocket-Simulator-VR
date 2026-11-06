#include "starsim/environment/lunar_surface_detail.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <memory>
#include <tuple>
#include <utility>

#include "starsim/environment/surface_hash.hpp"
#include "surface_sampling.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

using math::Vec3;

namespace {

// Crater reach: the profile is zero from 3 R = 1.5 D ([D] profile definition).
constexpr double crater_reach_per_diameter = 1.5;
// Crater cells are at least reach / 0.45 wide (nominal), so a crater reaches at most 0.45 of a nominal cell; cells
// are >= 0.7 nominal wide everywhere ([D] warp metric), so a query only meets craters of the cells nearest it.
constexpr double crater_cell_per_diameter = crater_reach_per_diameter / 0.45;
// LOLA resolves craters wider than 5 pixels ([E], plan Task 2): larger ones are not generated again.
constexpr double lola_crater_pixels = 5.0;
// Band-limited queries: features of size >= limit / 8 exactly, fading out to limit / 16 ([D] the dropped share of
// the square mean is ~(1/8)^(1 + H) of the octave at the limit, ~2 %).
constexpr double box_exact_fraction = 1.0 / 8.0;
// Diameter sub-intervals per octave for the expected crater mean.
constexpr int mean_field_steps = 32;
// Roughness normalisation: windows of the baseline size with 25 x 25 samples (Guo et al. 2021: 5 mm DTM, 125 mm
// windows), octaves up to 1024 x the window (the rest adds < 0.03 % after the plane fit), 1536 windows ([E], median
// uncertainty ~1 %).
constexpr int window_samples = 25;
constexpr double normalisation_top_octave = 1024.0;
constexpr int normalisation_windows = 1536;
// Crater search: faces whose normal is within ~90 deg of the query hold candidate cells (the reach is << R), and a
// 5 % margin on the (s, t) cull distance for the metric change between the query and the crater ([E]).
constexpr double min_face_view_cos = 1e-3;
constexpr double crater_cull_margin = 1.05;
// Roughness footprint means over another blend face's (sheared) parallelogram, only inside the face blend band:
// composite midpoint rule with 3 samples per lattice cell across, 4 to 96 per side ([E], checked by the band-limit
// test in the blend band).
constexpr double cross_samples_per_cell = 3.0;
constexpr int min_cross_samples = 4;
constexpr int max_cross_samples = 96;
// Value-noise node values are uniform in [-1, 1]: sqrt(3) gives unit variance.
const double unit_variance_gain = std::sqrt(3.0);

double smooth01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

double fade(double x) { return x * x * x * (x * (6.0 * x - 15.0) + 10.0); }
double fade_integral(double x) { return x * x * x * x * (x * (x - 3.0) + 2.5); }  // integral of fade from 0

// Integral over [a, b] of the fade "hat" basis of node j (support j - 1 .. j + 1, 1 at j).
double basis_integral(std::int64_t j, double a, double b) {
    const double jd = static_cast<double>(j);
    double sum = 0.0;
    double lo = std::max(a, jd - 1.0), hi = std::min(b, jd);
    if (hi > lo) sum += fade_integral(hi - jd + 1.0) - fade_integral(lo - jd + 1.0);
    lo = std::max(a, jd);
    hi = std::min(b, jd + 1.0);
    if (hi > lo) sum += (hi - lo) - (fade_integral(hi - jd) - fade_integral(lo - jd));
    return sum;
}

using sampling::poisson;

// Miller-Rabin, deterministic for n < 2^32 with bases 2, 7, 61 (Jaeschke 1993).
bool is_prime(std::uint64_t n) {
    if (n < 2) return false;
    for (std::uint64_t p : {2ull, 3ull, 5ull, 7ull, 11ull, 13ull}) {
        if (n % p == 0) return n == p;
    }
    std::uint64_t d = n - 1;
    int r = 0;
    while ((d & 1) == 0) {
        d >>= 1;
        ++r;
    }
    const auto pow_mod = [n](std::uint64_t b, std::uint64_t e) {
        std::uint64_t result = 1;
        b %= n;
        while (e > 0) {
            if (e & 1) result = result * b % n;
            b = b * b % n;
            e >>= 1;
        }
        return result;
    };
    for (std::uint64_t a : {2ull, 7ull, 61ull}) {
        if (a % n == 0) continue;
        std::uint64_t x = pow_mod(a, d);
        if (x == 1 || x == n - 1) continue;
        bool composite = true;
        for (int i = 1; i < r && composite; ++i) {
            x = x * x % n;
            if (x == n - 1) composite = false;
        }
        if (composite) return false;
    }
    return true;
}

std::uint64_t next_prime(std::uint64_t n) {
    while (!is_prime(n)) ++n;
    return n;
}

using sampling::inverse_cumulative;

// Exact area integral of a crater over the rectangle [0, X] x [0, Y] (crater-centred, units of the crater radius,
// X, Y > 0). In polar coordinates the rectangle is rho < min(X sec theta, Y csc theta), so with the radial integral
// F(rho) = integral_0^rho f(s) s ds the corner integral is P(X, atan(Y/X)) + P(Y, atan(X/Y)),
// P(a, Theta) = integral_0^Theta F(a sec theta) d theta. f = depth g1 + rim g2 is piecewise in rho (breaks at 1, 3):
//   rho < 1:      F = depth (rho^4/4 - rho^2/2) + rim rho^2/2
//   1 <= rho < 3: F = depth (-1/4) + rim (1/2 + c (1 - 1/rho) - c (rho^2 - 1)/54),   c = 27/26
//   rho >= 3:     F = -depth/4 + rim 27/26
// and integral of rho^4, rho^2, 1/rho, 1 over theta (rho = a sec theta) are a^4 (tan + tan^3/3), a^2 tan, sin / a,
// theta. Written with u = a tan theta (bounded: the other rectangle side at Theta) so a -> 0 stays finite.
struct CraterShape {
    double depth = 0.0, rim = 0.0;
};

constexpr double rim_c = 27.0 / 26.0;

// Antiderivative in theta of F(a sec theta) on one rho piece, at (theta, u = a tan theta, s = sin theta).
double piece_antiderivative(const CraterShape& f, int piece, double a, double theta, double u, double s) {
    switch (piece) {
        case 0:  // rho < 1: a^4 (t + t^3/3) = a^3 u + a u^3 / 3, a^2 t = a u
            return f.depth * (0.25 * (a * a * a * u + a * u * u * u / 3.0) - 0.5 * a * u) + f.rim * 0.5 * a * u;
        case 1:  // 1 <= rho < 3
            return (-0.25 * f.depth + f.rim * (0.5 + rim_c + rim_c / 54.0)) * theta +
                   f.rim * (-rim_c * s / a - rim_c * a * u / 54.0);
        default:  // rho >= 3
            return (-0.25 * f.depth + f.rim * rim_c) * theta;
    }
}

// P(a, Theta) with tan Theta = other / a (other = the rectangle's other side).
double polar_wedge(const CraterShape& f, double a, double other) {
    struct Point {
        double theta, u, s;
    };
    const double hyp = std::hypot(a, other);
    const Point end{std::atan2(other, a), other, other / hyp};
    // Break angles where rho = a sec theta reaches 1 and 3 (only below the end angle).
    std::array<Point, 2> breaks{};
    int nb = 0;
    for (const double radius : {1.0, 3.0}) {
        if (a >= radius) continue;
        const double u = std::sqrt(radius * radius - a * a);  // a tan theta at rho = radius
        if (u >= other) break;
        breaks[static_cast<size_t>(nb++)] = {std::acos(a / radius), u, u / radius};
    }
    int piece = a < 1.0 ? 0 : (a < 3.0 ? 1 : 2);
    Point from{0.0, 0.0, 0.0};
    double sum = 0.0;
    for (int k = 0; k <= nb; ++k) {
        const Point to = k < nb ? breaks[static_cast<size_t>(k)] : end;
        sum += piece_antiderivative(f, piece, a, to.theta, to.u, to.s) -
               piece_antiderivative(f, piece, a, from.theta, from.u, from.s);
        from = to;
        ++piece;
    }
    return sum;
}

// An edge line closer than this to the crater centre (crater radii) adds a zero-width wedge: skipped (error O(1e-12)).
constexpr double min_wedge_distance = 1e-12;

// P(a, Theta) for a signed along-edge coordinate t = a tan Theta (P is odd in Theta).
double signed_wedge(const CraterShape& f, double a, double t) {
    return t >= 0.0 ? polar_wedge(f, a, t) : -polar_wedge(f, a, -t);
}

// Signed integral of the crater over the triangle (crater centre, P, Q) (positive when counter-clockwise): the wedge
// between the polar angles of P and Q out to the line PQ at distance a = |P x u| (u the unit vector P -> Q), whose
// points sit at a sec theta: P(a, t_Q) - P(a, t_P) with t the along-line coordinate from the foot of the
// perpendicular. A rectangle's four triangles reproduce the corner integrals.
double crater_triangle(const CraterShape& f, double px, double py, double qx, double qy) {
    const double dx = qx - px, dy = qy - py;
    const double len = std::hypot(dx, dy);
    if (len == 0.0) return 0.0;
    const double ux = dx / len, uy = dy / len;
    const double signed_a = px * uy - py * ux;
    const double a = std::abs(signed_a);
    if (a < min_wedge_distance) return 0.0;
    const double tp = px * ux + py * uy;
    const double wedge = signed_wedge(f, a, tp + len) - signed_wedge(f, a, tp);
    return signed_a > 0.0 ? wedge : -wedge;
}

// Distance from the origin to the segment P Q.
double segment_distance(double px, double py, double qx, double qy) {
    const double dx = qx - px, dy = qy - py;
    const double l2 = dx * dx + dy * dy;
    const double t = l2 > 0.0 ? std::clamp(-(px * dx + py * dy) / l2, 0.0, 1.0) : 0.0;
    return std::hypot(px + t * dx, py + t * dy);
}

// Below this footprint size / crater radius a footprint lying wholly inside one smooth part of the profile uses the
// closed-form mean (bowl: exact; rim flank: second-order Taylor, error ~0.2 (L/R)^4 rim < 3e-6 rim) instead of the
// triangle sums, which lose ~1e-16 (R/L)^2 of the depth to cancellation ([D]).
constexpr double small_square_ratio = 1.0 / 16.0;

// Mean of a crater over the footprint p + a e_s + b e_t, |a|, |b| <= L/2 (the face lattice's (s, t) parallelogram,
// the same footprint the roughness means use). Tangent-plane coordinates: e1 = e_s, e2 = e_t made orthogonal, so
// e_s = (1, 0), e_t = (cos_st, sin_st) with sin_st > 0; the crater centre at (x, y) metres from the query point.
double crater_box_mean_m(const Crater& c, double x, double y, double cos_st, double sin_st, double limit_m) {
    const double radius = 0.5 * c.diameter_m;
    const double eps = limit_m / radius;
    const double cx = -x / radius, cy = -y / radius;  // footprint centre in crater units, crater at the origin
    const double ax = 0.5 * eps, bx = 0.5 * eps * cos_st, by = 0.5 * eps * sin_st;  // half sides A = (ax, 0), B
    // Corners counter-clockwise: c - A - B, c + A - B, c + A + B, c - A + B.
    const std::array<double, 4> vx = {cx - ax - bx, cx + ax - bx, cx + ax + bx, cx - ax + bx};
    const std::array<double, 4> vy = {cy - by, cy - by, cy + by, cy + by};
    // Origin inside: its parallelogram coordinates (alpha along A, beta along B) within [-1, 1].
    const double beta = -cy / by, alpha = (-cx - beta * bx) / ax;
    double r_min = 0.0, r_max = 0.0;
    if (std::abs(alpha) > 1.0 || std::abs(beta) > 1.0) {
        r_min = std::numeric_limits<double>::max();
        for (size_t k = 0; k < 4; ++k) {
            r_min = std::min(r_min, segment_distance(vx[k], vy[k], vx[(k + 1) % 4], vy[(k + 1) % 4]));
        }
    }
    if (r_min >= 3.0) return 0.0;
    for (size_t k = 0; k < 4; ++k) r_max = std::max(r_max, std::hypot(vx[k], vy[k]));
    if (eps <= small_square_ratio) {
        if (r_max <= 1.0) {  // the bowl is quadratic: the footprint mean of x^2 + y^2 adds (|A|^2 + |B|^2) / 3 = eps^2/6
            return c.rim_height_m - c.depth_m + c.depth_m * (cx * cx + cy * cy + eps * eps / 6.0);
        }
        if (r_min >= 1.0 && r_max <= 3.0) {
            // Second order: mean = g(c) + (A^T H A + B^T H B) / 6, H of rho^-3: -3 rho^-5 I + 15 rho^-7 c c^T.
            const double rho = std::hypot(cx, cy);
            const double r3 = rho * rho * rho, r5 = r3 * rho * rho;
            const double ca = cx, cb = cx * cos_st + cy * sin_st;  // c . e_s, c . e_t
            const double curvature = -6.0 / r5 + 15.0 * (ca * ca + cb * cb) / (r5 * rho * rho);
            return c.rim_height_m * rim_c * (1.0 / r3 + eps * eps / 24.0 * curvature - 1.0 / 27.0);
        }
    }
    const CraterShape f{c.depth_m, c.rim_height_m};
    double integral = 0.0;
    for (size_t k = 0; k < 4; ++k) integral += crater_triangle(f, vx[k], vy[k], vx[(k + 1) % 4], vy[(k + 1) % 4]);
    return integral / (eps * eps * sin_st);
}

// Per-face view of a query point for the cell searches (continued coordinates and metric at the clamped point).
struct FaceView {
    bool used = false;
    FaceCoord c;
    double m_s = 0.0, m_t = 0.0;
    double sin_angle = 1.0;
    // 1 / sqrt(1 - |cos angle(e_s, e_t)|): |a e_s + b e_t| >= sqrt(1 - |cos|) |(a, b)| makes the (s, t) offset times
    // the metric a safe cull against the reach.
    double cull_factor = 1.0;
};

std::uint64_t crater_cell_hash(const FaceCell& cell) {
    return surface_hash((surface_stream::craters << 8) | static_cast<std::uint64_t>(cell.face),
                        static_cast<std::uint64_t>(cell.level), static_cast<std::uint64_t>(cell.i),
                        static_cast<std::uint64_t>(cell.j));
}

std::pair<double, double> crater_position_draw(std::uint64_t cell_hash, int m) {
    const auto mu = static_cast<std::uint64_t>(m);
    return {hash_unit(surface_hash(cell_hash, mu, 0, 0)), hash_unit(surface_hash(cell_hash, mu, 1, 0))};
}

// Cell index range on a face view within reach_m (the cell search of face_cells_within, without allocation).
struct CellRange {
    std::int64_t i0 = 0, i1 = -1, j0 = 0, j1 = -1;
};

CellRange cell_range(const FaceView& v, int level, double reach_m) {
    const std::int64_t count = std::int64_t{1} << level;
    const double n = static_cast<double>(count);
    const double cells_s = face_cell_range_margin * reach_m / (v.m_s * 2.0 / n * v.sin_angle);
    const double cells_t = face_cell_range_margin * reach_m / (v.m_t * 2.0 / n * v.sin_angle);
    const double u = (v.c.s + 1.0) * 0.5 * n, w = (v.c.t + 1.0) * 0.5 * n;
    if (u + cells_s < 0.0 || w + cells_t < 0.0 || u - cells_s >= n || w - cells_t >= n) return {};
    const auto clamp_index = [count](double x) {
        return std::clamp(static_cast<std::int64_t>(std::floor(x)), std::int64_t{0}, count - 1);
    };
    return {clamp_index(u - cells_s), clamp_index(u + cells_s), clamp_index(w - cells_t), clamp_index(w + cells_t)};
}

}  // namespace

double crater_profile_m(const Crater& c, double distance_m) {
    const double rho = distance_m / (0.5 * c.diameter_m);
    if (rho >= 3.0) return 0.0;
    if (rho < 1.0) return c.rim_height_m - c.depth_m + c.depth_m * rho * rho;
    return c.rim_height_m * (1.0 / (rho * rho * rho) - 1.0 / 27.0) * (27.0 / 26.0);
}

struct LunarSurfaceDetail::Query {
    Vec3 p;
    double band_start_m = 0.0;
    // The faces whose lattices define the point (face_blend: one inside a face, two across an edge, three at a corner).
    struct BlendFace {
        FaceCoord c;       // continued coordinates
        FaceFrame frame;   // at the continued coordinates
        Vec3 e1, e2;       // orthonormal: e1 = e_s, e2 = e_t made orthogonal
        double cos_st = 0.0, sin_st = 1.0;  // e_t = cos_st e1 + sin_st e2
        double w_crater = 0.0;  // partition of unity (sum 1): footprint weights of the crater means
        double w_rough = 0.0;   // w / sqrt(sum w^2): blends independent face fields with unchanged variance
    };
    int blend_count = 0;
    std::array<BlendFace, 3> blend;
    std::array<FaceView, 6> views;  // crater cell search
};

LunarSurfaceDetail::LunarSurfaceDetail(config::MoonSurfaceConfig config, std::shared_ptr<const LunarTerrain> lola)
    : config_(std::move(config)), lola_(std::move(lola)) {
    if (!lola_) throw std::invalid_argument("LunarSurfaceDetail: no lunar terrain (LOLA) given");
    static std::atomic<std::uint64_t> next_instance_id{1};
    instance_id_ = next_instance_id.fetch_add(1);
    const auto& sfd = config_.craters.sfd;
    if (sfd.form != "power" && sfd.form != "exponential") {
        throw std::invalid_argument("LunarSurfaceDetail: unknown size distribution '" + sfd.form + "'");
    }
    sphere_radius_m_ = lola_->sphere_radius_m();
    finest_band_start_m_ = lola_->finest_resolution_m();
    rim_to_depth_ = config_.craters.rim_height_to_diameter / config_.craters.depth_to_diameter_fresh;

    // Crater octaves from the smallest diameter up.
    for (double d_lo = sfd.min_diameter_m; d_lo < sfd.max_diameter_m;) {
        const double d_hi = std::min(2.0 * d_lo, sfd.max_diameter_m);
        CraterOctave o;
        o.min_diameter_m = d_lo;
        o.max_diameter_m = d_hi;
        o.level = face_cell_level_for_size(crater_cell_per_diameter * d_hi, sphere_radius_m_);
        o.cumulative_min_per_m2 = sfd.cumulative_density_per_m2(d_lo);
        o.cumulative_max_per_m2 = sfd.cumulative_density_per_m2(d_hi);
        o.density_per_m2 = o.cumulative_min_per_m2 - o.cumulative_max_per_m2;
        crater_octaves_.push_back(o);
        d_lo = d_hi;
    }
    for (size_t k = 0; k < crater_octaves_.size(); ++k) {
        crater_octaves_[k].mean_relief_m = crater_mean_field_m(static_cast<int>(k), std::numeric_limits<double>::max());
    }

    // Roughness octaves: spacing 1/2 of the finest wavelength upward in factors of ~2 to the coarsest band start.
    const double face_edge_m = 0.5 * constants::pi * sphere_radius_m_;
    const double top_m = 2.0 * std::max(lola_->global_resolution_m(), finest_band_start_m_);
    for (double target = 0.5 * finest_wavelength_m(); target <= top_m; target *= 2.0) {
        const auto n = next_prime(static_cast<std::uint64_t>(std::llround(face_edge_m / target)));
        RoughnessOctave o{static_cast<std::int64_t>(n), face_edge_m / static_cast<double>(n)};
        o.relative_amplitude = unit_variance_gain * std::pow(o.spacing_m / config_.roughness.baseline_m,
                                                             config_.roughness.hurst_exponent);
        roughness_octaves_.push_back(o);
    }
    roughness_scale_m_ = derive_roughness_scale();
}

double LunarSurfaceDetail::finest_wavelength_m() const { return config_.craters.sfd.min_diameter_m; }

double LunarSurfaceDetail::band_start_m(const Vec3& unit) const {
    const auto blend = lola_->tile_blend(unit);
    return blend.weight * blend.scale_m + (1.0 - blend.weight) * lola_->global_resolution_m();
}

LunarSurfaceDetail::Query LunarSurfaceDetail::make_query(const Vec3& unit, bool for_craters) const {
    Query q;
    q.p = unit;
    const FaceBlend fb = face_blend(unit, face_blend_band);
    double sum_w2 = 0.0;
    for (int k = 0; k < fb.count; ++k) sum_w2 += fb.weight[static_cast<size_t>(k)] * fb.weight[static_cast<size_t>(k)];
    const double rough_norm = 1.0 / std::sqrt(sum_w2);
    q.blend_count = fb.count;
    for (int k = 0; k < fb.count; ++k) {
        Query::BlendFace& b = q.blend[static_cast<size_t>(k)];
        b.c = fb.coord[static_cast<size_t>(k)];
        b.frame = face_frame(b.c.face, b.c.s, b.c.t, sphere_radius_m_);
        b.e1 = b.frame.e_s;
        b.cos_st = dot(b.frame.e_t, b.e1);
        b.e2 = math::normalized(b.frame.e_t - b.e1 * b.cos_st);
        b.sin_st = dot(b.frame.e_t, b.e2);
        b.w_crater = fb.weight[static_cast<size_t>(k)];
        b.w_rough = fb.weight[static_cast<size_t>(k)] * rough_norm;
    }
    if (!for_craters) return q;
    q.band_start_m = band_start_m(unit);
    for (int face = 0; face < 6; ++face) {
        if (dot(unit, face_normal(face)) <= min_face_view_cos) continue;
        FaceView& v = q.views[static_cast<size_t>(face)];
        v.used = true;
        v.c = face_coord_on(face, unit);
        const FaceFrame f = face_frame(face, std::clamp(v.c.s, -1.0, 1.0), std::clamp(v.c.t, -1.0, 1.0),
                                       sphere_radius_m_);
        v.m_s = f.m_s;
        v.m_t = f.m_t;
        v.sin_angle = std::max(norm(cross(f.e_s, f.e_t)), 0.5);
        v.cull_factor = 1.0 / std::sqrt(1.0 - std::abs(dot(f.e_s, f.e_t)));
    }
    return q;
}

double LunarSurfaceDetail::crater_mean_field_m(int octave, double band_start_m) const {
    const CraterOctave& o = crater_octaves_[static_cast<size_t>(octave)];
    const double cut = std::min(o.max_diameter_m, lola_crater_pixels * band_start_m);
    if (cut <= o.min_diameter_m) return 0.0;
    if (cut >= o.max_diameter_m && o.mean_relief_m != 0.0) return o.mean_relief_m;
    // Expected volume per crater: depth R^2 integral(g1) + rim R^2 integral(g2) = depth R^2 (-pi/2 + kappa 27 pi / 13).
    const auto& c = config_.craters;
    const double mean_depth_ratio =
        c.degraded_fraction * c.depth_to_diameter_degraded + (1.0 - c.degraded_fraction) * c.depth_to_diameter_fresh;
    const double unit_volume = -0.5 * constants::pi + rim_to_depth_ * 27.0 * constants::pi / 13.0;
    const auto& sfd = c.sfd;
    double sum = 0.0;
    const double ratio = std::pow(cut / o.min_diameter_m, 1.0 / mean_field_steps);
    double d0 = o.min_diameter_m;
    for (int k = 0; k < mean_field_steps; ++k) {
        const double d1 = k + 1 == mean_field_steps ? cut : d0 * ratio;
        const double d = std::sqrt(d0 * d1);
        const double volume = mean_depth_ratio * d * 0.25 * d * d * unit_volume;
        sum += (sfd.cumulative_density_per_m2(d0) - sfd.cumulative_density_per_m2(d1)) * volume;
        d0 = d1;
    }
    return sum;
}

int LunarSurfaceDetail::crater_count(int octave, const FaceCell& cell, std::uint64_t cell_hash) const {
    const CraterOctave& o = crater_octaves_[static_cast<size_t>(octave)];
    return poisson(hash_unit(cell_hash), o.density_per_m2 * face_cell_area_m2(cell, sphere_radius_m_));
}

bool LunarSurfaceDetail::complete_crater(int octave, const FaceCell& cell, std::uint64_t cell_hash, int m, double s01,
                                         double t01, Crater& out) const {
    const CraterOctave& o = crater_octaves_[static_cast<size_t>(octave)];
    const auto& c = config_.craters;
    const auto mu = static_cast<std::uint64_t>(m);
    const double n_lo = o.cumulative_min_per_m2;
    const double n_hi = o.cumulative_max_per_m2;
    const double d = inverse_cumulative(c.sfd, n_lo - hash_unit(surface_hash(cell_hash, mu, 2, 0)) * (n_lo - n_hi));
    const Vec3 centre = face_cell_point(cell, s01, t01);
    if (d > lola_crater_pixels * finest_band_start_m_ && d > lola_crater_pixels * band_start_m(centre)) return false;
    out.centre_unit = centre;
    out.diameter_m = d;
    out.fresh = hash_unit(surface_hash(cell_hash, mu, 3, 0)) >= c.degraded_fraction;
    out.depth_m = d * (out.fresh ? c.depth_to_diameter_fresh : c.depth_to_diameter_degraded);
    out.rim_height_m = rim_to_depth_ * out.depth_m;
    return true;
}

void LunarSurfaceDetail::craters_in_cell_of(int octave, const FaceCell& cell, std::vector<Crater>& out) const {
    const std::uint64_t h = crater_cell_hash(cell);
    const int count = crater_count(octave, cell, h);
    for (int m = 0; m < count; ++m) {
        const auto [s01, t01] = crater_position_draw(h, m);
        Crater cr;
        if (complete_crater(octave, cell, h, m, s01, t01, cr)) out.push_back(cr);
    }
}

std::vector<Crater> LunarSurfaceDetail::craters_in_cell(const FaceCell& cell) const {
    std::vector<Crater> out;
    for (size_t k = 0; k < crater_octaves_.size(); ++k) {
        if (crater_octaves_[k].level == cell.level) craters_in_cell_of(static_cast<int>(k), cell, out);
    }
    return out;
}

std::vector<Crater> LunarSurfaceDetail::craters_within(const Vec3& unit, double radius_m, double min_diameter_m,
                                                       double max_diameter_m) const {
    std::vector<Crater> out, cell_craters;
    for (size_t k = 0; k < crater_octaves_.size(); ++k) {
        const CraterOctave& o = crater_octaves_[k];
        if (o.max_diameter_m <= min_diameter_m || o.min_diameter_m >= max_diameter_m) continue;
        for (const FaceCell& cell : face_cells_within(unit, o.level, radius_m, sphere_radius_m_)) {
            cell_craters.clear();
            craters_in_cell_of(static_cast<int>(k), cell, cell_craters);
            for (const Crater& c : cell_craters) {
                if (c.diameter_m < min_diameter_m || c.diameter_m >= max_diameter_m) continue;
                if (sphere_radius_m_ * norm(c.centre_unit - unit) <= radius_m) out.push_back(c);
            }
        }
    }
    return out;
}

const double* LunarSurfaceDetail::crater_mean_fields(double band_start_m) const {
    struct Memo {
        std::uint64_t instance = 0;
        double band_start_m = 0.0;
        std::vector<double> values;
    };
    thread_local Memo memo;
    if (memo.instance != instance_id_ || memo.band_start_m != band_start_m) {
        memo.values.resize(crater_octaves_.size());
        for (size_t k = 0; k < crater_octaves_.size(); ++k) {
            memo.values[k] = crater_mean_field_m(static_cast<int>(k), band_start_m);
        }
        memo.instance = instance_id_;
        memo.band_start_m = band_start_m;
    }
    return memo.values.data();
}

const LunarSurfaceDetail::CraterCellMemo::Slot* LunarSurfaceDetail::memo_cell(int octave, const FaceCell& cell,
                                                                               std::uint64_t cell_hash) const {
    thread_local std::unique_ptr<CraterCellMemo> memo;
    if (!memo) memo = std::make_unique<CraterCellMemo>();
    CraterCellMemo::Slot& slot = memo->table[static_cast<size_t>(cell_hash & (CraterCellMemo::slots - 1))];
    if (slot.instance == instance_id_ && slot.octave == octave && slot.face == cell.face && slot.i == cell.i &&
        slot.j == cell.j) {
        return &slot;
    }
    const int count = crater_count(octave, cell, cell_hash);
    if (count > CraterCellMemo::max_draws) return nullptr;  // rare: generated in place
    slot.instance = instance_id_;
    slot.octave = octave;
    slot.face = cell.face;
    slot.i = cell.i;
    slot.j = cell.j;
    slot.count = count;
    for (int m = 0; m < count; ++m) {
        const auto k = static_cast<size_t>(m);
        std::tie(slot.s01[k], slot.t01[k]) = crater_position_draw(cell_hash, m);
        slot.complete[k] = complete_crater(octave, cell, cell_hash, m, slot.s01[k], slot.t01[k], slot.crater[k]);
    }
    return &slot;
}

double LunarSurfaceDetail::craters_at(const Query& q, double limit_m) const {
    double sum = 0.0;
    const double* mean_field = crater_mean_fields(q.band_start_m);
    for (size_t k = 0; k < crater_octaves_.size(); ++k) {
        const CraterOctave& o = crater_octaves_[k];
        const double w = limit_m > 0.0
                             ? smooth01(std::log2(std::sqrt(o.min_diameter_m * o.max_diameter_m) /
                                                  (box_exact_fraction * limit_m)) + 1.0)
                             : 1.0;
        if (w <= 0.0) continue;
        // Zero mean: LOLA heights are area means that already hold the craters' mean, so each octave enters as its
        // deviation from the population mean (an octave below the footprint, w = 0, then adds exactly nothing).
        sum -= w * mean_field[k];
        // The footprint's half diagonal is at most L (|e_s +- e_t| <= 2).
        const double reach = crater_reach_per_diameter * o.max_diameter_m + (limit_m > 0.0 ? limit_m : 0.0);
        const double n = static_cast<double>(std::int64_t{1} << o.level);
        for (int face = 0; face < 6; ++face) {
            const FaceView& v = q.views[static_cast<size_t>(face)];
            if (!v.used) continue;
            const CellRange r = cell_range(v, o.level, reach);
            const double cull = crater_cull_margin * reach * v.cull_factor;
            const double cull_squared = cull * cull;  // squared compare: hypot is slow, and a crater at the cull
                                                      // distance lies beyond its reach (adds exactly 0) either way
            for (std::int64_t j = r.j0; j <= r.j1; ++j) {
                for (std::int64_t i = r.i0; i <= r.i1; ++i) {
                    const FaceCell cell{face, o.level, i, j};
                    const std::uint64_t h = crater_cell_hash(cell);
                    const CraterCellMemo::Slot* memo = memo_cell(static_cast<int>(k), cell, h);
                    const int count = memo ? memo->count : crater_count(static_cast<int>(k), cell, h);
                    for (int m = 0; m < count; ++m) {
                        const auto [s01, t01] = memo ? std::pair{memo->s01[static_cast<size_t>(m)],
                                                                 memo->t01[static_cast<size_t>(m)]}
                                                     : crater_position_draw(h, m);
                        const double ds = (-1.0 + 2.0 * (static_cast<double>(i) + s01) / n) - v.c.s;
                        const double dt = (-1.0 + 2.0 * (static_cast<double>(j) + t01) / n) - v.c.t;
                        const double es = ds * v.m_s, et = dt * v.m_t;
                        if (es * es + et * et > cull_squared) continue;
                        Crater cr;
                        if (memo) {
                            if (!memo->complete[static_cast<size_t>(m)]) continue;
                            cr = memo->crater[static_cast<size_t>(m)];
                        } else if (!complete_crater(static_cast<int>(k), cell, h, m, s01, t01, cr)) {
                            continue;
                        }
                        if (limit_m > 0.0) {
                            const Vec3 offset = (cr.centre_unit - q.p) * sphere_radius_m_;
                            for (int b = 0; b < q.blend_count; ++b) {
                                const Query::BlendFace& bf = q.blend[static_cast<size_t>(b)];
                                sum += w * bf.w_crater *
                                       crater_box_mean_m(cr, dot(offset, bf.e1), dot(offset, bf.e2), bf.cos_st,
                                                         bf.sin_st, limit_m);
                            }
                        } else {
                            sum += crater_profile_m(cr, sphere_radius_m_ * norm(q.p - cr.centre_unit));
                        }
                    }
                }
            }
        }
    }
    return sum;
}

double LunarSurfaceDetail::roughness_node(int octave, int face, std::int64_t j, std::int64_t k) const {
    // Every face has its own lattice, continued beyond its edges (indices < 0 or > cells_per_edge); faces are joined
    // by the face blend, not by shared nodes.
    const std::uint64_t h = surface_hash((surface_stream::roughness << 8) | static_cast<std::uint64_t>(face),
                                         static_cast<std::uint64_t>(octave), static_cast<std::uint64_t>(j),
                                         static_cast<std::uint64_t>(k));
    return 2.0 * hash_unit(h) - 1.0;
}

const double* LunarSurfaceDetail::roughness_top_weights(double band_start_m) const {
    struct Memo {
        std::uint64_t instance = 0;
        double band_start_m = 0.0;
        std::vector<double> values;
    };
    thread_local Memo memo;
    if (memo.instance != instance_id_ || memo.band_start_m != band_start_m) {
        memo.values.resize(roughness_octaves_.size());
        for (size_t m = 0; m < roughness_octaves_.size(); ++m) {
            // Top: full up to half the band start, none from the band start (LOLA resolves 2-pixel wavelengths, [E]).
            memo.values[m] = smooth01(std::log2(band_start_m / roughness_octaves_[m].spacing_m));
        }
        memo.instance = instance_id_;
        memo.band_start_m = band_start_m;
    }
    return memo.values.data();
}

double LunarSurfaceDetail::roughness_at(const Query& q, double limit_m, double scale_m) const {
    const auto& r = config_.roughness;
    double sum = 0.0;
    const double* top_weights = roughness_top_weights(q.band_start_m);
    // The amplitude's metric factor per blend face: (local scale)^H (the octave's part is precomputed).
    std::array<double, 3> metric{};
    for (int bi = 0; bi < q.blend_count; ++bi) {
        metric[static_cast<size_t>(bi)] = std::pow(q.blend[static_cast<size_t>(bi)].frame.scale, r.hurst_exponent);
    }
    for (size_t m = 0; m < roughness_octaves_.size(); ++m) {
        const RoughnessOctave& o = roughness_octaves_[m];
        const double w_top = top_weights[m];
        if (w_top <= 0.0) continue;
        const double w_box = limit_m > 0.0 ? smooth01(std::log2(o.spacing_m / (box_exact_fraction * limit_m)) + 1.0) : 1.0;
        if (w_box <= 0.0) continue;
        const std::int64_t n = o.cells_per_edge;
        const double nd = static_cast<double>(n);
        const int oct = static_cast<int>(m);
        for (int bi = 0; bi < q.blend_count; ++bi) {
            const Query::BlendFace& b = q.blend[static_cast<size_t>(bi)];
            const int face = b.c.face;
            const double amplitude = scale_m * o.relative_amplitude * metric[static_cast<size_t>(bi)];
            const double u = (b.c.s + 1.0) * 0.5 * nd, v = (b.c.t + 1.0) * 0.5 * nd;
            // This face's lattice field at lattice coordinates (x, y).
            const auto field = [&](double x, double y) {
                const auto j0 = static_cast<std::int64_t>(std::floor(x));
                const auto k0 = static_cast<std::int64_t>(std::floor(y));
                const double fu = fade(x - static_cast<double>(j0));
                const double fv = fade(y - static_cast<double>(k0));
                return (1.0 - fv) * ((1.0 - fu) * roughness_node(oct, face, j0, k0) +
                                     fu * roughness_node(oct, face, j0 + 1, k0)) +
                       fv * ((1.0 - fu) * roughness_node(oct, face, j0, k0 + 1) +
                             fu * roughness_node(oct, face, j0 + 1, k0 + 1));
            };
            double value = 0.0;
            if (limit_m <= 0.0) {
                value = field(u, v);
            } else {
                // The footprint is the crater-weighted set of the blend faces' (s, t) parallelograms (one face away
                // from edges). Over this face's own parallelogram the mean is exact: the basis is a product of 1-D
                // fade hats, so it separates.
                const double du = 0.5 * limit_m / b.frame.m_s * 0.5 * nd, dv = 0.5 * limit_m / b.frame.m_t * 0.5 * nd;
                const double u0 = u - du, u1 = u + du, v0 = v - dv, v1 = v + dv;
                const auto ja = static_cast<std::int64_t>(std::floor(u0)), jb = static_cast<std::int64_t>(std::floor(u1)) + 1;
                const auto ka = static_cast<std::int64_t>(std::floor(v0)), kb = static_cast<std::int64_t>(std::floor(v1)) + 1;
                constexpr std::int64_t max_nodes = 64;
                if (jb - ja + 1 > max_nodes || kb - ka + 1 > max_nodes) {
                    throw std::logic_error("LunarSurfaceDetail: square mean spans too many lattice nodes");
                }
                std::array<double, max_nodes> wy{};
                for (std::int64_t k = ka; k <= kb; ++k) wy[static_cast<size_t>(k - ka)] = basis_integral(k, v0, v1);
                double own = 0.0;
                for (std::int64_t j = ja; j <= jb; ++j) {
                    const double wx = basis_integral(j, u0, u1);
                    if (wx == 0.0) continue;
                    double row = 0.0;
                    for (std::int64_t k = ka; k <= kb; ++k) {
                        const double wk = wy[static_cast<size_t>(k - ka)];
                        if (wk != 0.0) row += wk * roughness_node(oct, face, j, k);
                    }
                    own += wx * row;
                }
                const double own_mean = own / ((u1 - u0) * (v1 - v0));
                // Another blend face's parallelogram is sheared in this lattice: composite midpoint rule
                // (cross_samples_per_cell per lattice cell). Octaves finer than L/8 (and, ramping in, up to L/4) use
                // this face's own parallelogram instead: their footprint means are small, and the quadrature cost
                // grows with (L / spacing)^2.
                const double cross_weight = smooth01(std::log2(o.spacing_m / (box_exact_fraction * limit_m)));
                value = own_mean * (b.w_crater + (1.0 - cross_weight) * (1.0 - b.w_crater));
                for (int gi = 0; gi < q.blend_count && cross_weight > 0.0; ++gi) {
                    if (gi == bi) continue;
                    const Query::BlendFace& g = q.blend[static_cast<size_t>(gi)];
                    const auto to_lattice = [&](const Vec3& metres) {
                        const double x2 = dot(metres, b.e2) / b.sin_st;  // along e_t (m)
                        const double x1 = dot(metres, b.e1) - b.cos_st * x2;  // along e_s (m)
                        return std::pair{x1 / b.frame.m_s * 0.5 * nd, x2 / b.frame.m_t * 0.5 * nd};
                    };
                    const auto [au, av] = to_lattice(g.frame.e_s * (0.5 * limit_m));
                    const auto [bu, bv] = to_lattice(g.frame.e_t * (0.5 * limit_m));
                    const double cells = 2.0 * std::max(std::abs(au) + std::abs(bu), std::abs(av) + std::abs(bv));
                    const int count = std::clamp(static_cast<int>(std::ceil(cross_samples_per_cell * cells)),
                                                 min_cross_samples, max_cross_samples);
                    double cross = 0.0;
                    for (int i = 0; i < count; ++i) {
                        const double alpha = -1.0 + (2.0 * i + 1.0) / count;
                        for (int k = 0; k < count; ++k) {
                            const double beta = -1.0 + (2.0 * k + 1.0) / count;
                            cross += field(u + alpha * au + beta * bu, v + alpha * av + beta * bv);
                        }
                    }
                    value += cross_weight * g.w_crater * cross / (static_cast<double>(count) * count);
                }
            }
            sum += w_top * w_box * b.w_rough * amplitude * value;
        }
    }
    return sum;
}


double LunarSurfaceDetail::derive_roughness_scale() const {
    const double window = config_.roughness.baseline_m;
    const double spacing = window / window_samples;
    const double half = 0.5 * (window_samples - 1);
    std::vector<double> stds;
    stds.reserve(normalisation_windows);
    std::vector<double> z(window_samples * window_samples);
    for (int w = 0; w < normalisation_windows; ++w) {
        const auto wu = static_cast<std::uint64_t>(w);
        const double zc = 2.0 * hash_unit(surface_hash(0xA11CE, wu, 0, 0)) - 1.0;
        const double phi = constants::two_pi * hash_unit(surface_hash(0xA11CE, wu, 1, 0));
        const double rc = std::sqrt(std::max(0.0, 1.0 - zc * zc));
        const Vec3 centre{rc * std::cos(phi), rc * std::sin(phi), zc};
        const Vec3 e1 = math::normalized(cross(std::abs(centre.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, centre));
        const Vec3 e2 = cross(centre, e1);
        for (int a = 0; a < window_samples; ++a) {
            for (int b = 0; b < window_samples; ++b) {
                const Vec3 p = math::normalized(
                    centre + (e1 * ((a - half) * spacing) + e2 * ((b - half) * spacing)) / sphere_radius_m_);
                Query q = make_query(p, false);
                q.band_start_m = 2.0 * normalisation_top_octave * window;
                z[static_cast<size_t>(a * window_samples + b)] = roughness_at(q, 0.0, 1.0);
            }
        }
        // Plane fit on the symmetric grid: the mean and the two slopes are independent.
        double mean = 0.0, sx = 0.0, sy = 0.0, xx = 0.0;
        for (int a = 0; a < window_samples; ++a) {
            for (int b = 0; b < window_samples; ++b) {
                const double v = z[static_cast<size_t>(a * window_samples + b)];
                mean += v;
                sx += v * (a - half);
                sy += v * (b - half);
                xx += (a - half) * (a - half);
            }
        }
        const double count = window_samples * window_samples;
        mean /= count;
        const double gx = sx / xx, gy = sy / xx;
        double var = 0.0;
        for (int a = 0; a < window_samples; ++a) {
            for (int b = 0; b < window_samples; ++b) {
                const double res = z[static_cast<size_t>(a * window_samples + b)] - mean - gx * (a - half) - gy * (b - half);
                var += res * res;
            }
        }
        stds.push_back(std::sqrt(var / count));
    }
    auto mid = stds.begin() + static_cast<std::ptrdiff_t>(stds.size() / 2);
    std::nth_element(stds.begin(), mid, stds.end());
    return config_.roughness.rms_height_at_baseline_m / *mid;
}

double LunarSurfaceDetail::crater_relief_m(const Vec3& unit, double min_wavelength_m) const {
    return craters_at(make_query(unit), min_wavelength_m);
}

double LunarSurfaceDetail::roughness_relief_m(const Vec3& unit, double min_wavelength_m) const {
    return roughness_at(make_query(unit), min_wavelength_m, roughness_scale_m_);
}

double LunarSurfaceDetail::relief_m(const Vec3& unit, double min_wavelength_m) const {
    const Query q = make_query(unit);
    return craters_at(q, min_wavelength_m) + roughness_at(q, min_wavelength_m, roughness_scale_m_);
}

}  // namespace starsim::environment
