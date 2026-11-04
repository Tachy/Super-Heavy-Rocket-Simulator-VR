#include "starsim/environment/lunar_surface_detail.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "starsim/environment/surface_hash.hpp"
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

int poisson(double u, double lambda) {
    double p = std::exp(-lambda), cdf = p;
    int n = 0;
    while (u > cdf) {
        ++n;
        p *= lambda / n;
        if (p <= cdf * 1e-17) break;  // no further mass (u within rounding of 1)
        cdf += p;
    }
    return n;
}

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

double inverse_cumulative(const config::SizeFrequency& sfd, double cumulative_per_m2) {
    if (sfd.form == "power") {
        return sfd.reference_diameter_m * std::pow(cumulative_per_m2 / sfd.density_per_m2, -1.0 / sfd.exponent);
    }
    return -std::log(cumulative_per_m2 / sfd.density_per_m2) / sfd.exponent;  // "exponential"
}

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

// Signed corner integral C(X, Y) = sgn X sgn Y C(|X|, |Y|) (the shape is even in x and y); 0 on the axes.
double crater_corner(const CraterShape& f, double x, double y) {
    const double ax = std::abs(x), ay = std::abs(y);
    if (ax < 1e-12 || ay < 1e-12) return 0.0;  // the corner on an axis: a zero-width wedge (error O(1e-12))
    const double sign = (x < 0.0) != (y < 0.0) ? -1.0 : 1.0;
    return sign * (polar_wedge(f, ax, ay) + polar_wedge(f, ay, ax));
}

// Below this square size / crater radius a square lying wholly inside one smooth part of the profile uses the
// closed-form mean (bowl: exact; rim flank: second-order Taylor, error ~0.2 (L/R)^4 rim < 3e-6 rim) instead of the
// corner differences, which lose ~1e-16 (R/L)^2 of the depth to cancellation ([D]).
constexpr double small_square_ratio = 1.0 / 16.0;

// Mean of a crater over the L x L square centred at the query, the crater centre at (x, y) metres from it.
double crater_box_mean_m(const Crater& c, double x, double y, double limit_m) {
    const double radius = 0.5 * c.diameter_m;
    const double eps = limit_m / radius;
    const double cx = -x / radius, cy = -y / radius;  // square centre in crater units
    const double x0 = cx - 0.5 * eps, x1 = cx + 0.5 * eps, y0 = cy - 0.5 * eps, y1 = cy + 0.5 * eps;
    const double near_x = std::max({x0, -x1, 0.0}), near_y = std::max({y0, -y1, 0.0});
    const double r_min = std::hypot(near_x, near_y);
    if (r_min >= 3.0) return 0.0;
    if (eps <= small_square_ratio) {
        const double r_max = std::hypot(std::max(std::abs(x0), std::abs(x1)), std::max(std::abs(y0), std::abs(y1)));
        if (r_max <= 1.0) {  // the bowl is quadratic: the square mean of x^2 + y^2 adds eps^2 / 6
            return c.rim_height_m - c.depth_m + c.depth_m * (cx * cx + cy * cy + eps * eps / 6.0);
        }
        if (r_min >= 1.0 && r_max <= 3.0) {  // laplacian of rho^-3 in the plane: 9 rho^-5
            const double rho = std::hypot(cx, cy);
            const double r3 = rho * rho * rho;
            return c.rim_height_m * rim_c * (1.0 / r3 + eps * eps / 24.0 * 9.0 / (r3 * rho * rho) - 1.0 / 27.0);
        }
    }
    const CraterShape f{c.depth_m, c.rim_height_m};
    const double integral = crater_corner(f, x1, y1) - crater_corner(f, x0, y1) - crater_corner(f, x1, y0) +
                            crater_corner(f, x0, y0);
    return integral / (eps * eps);
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
    const double cells_s = 1.1 * reach_m / (v.m_s * 2.0 / n * v.sin_angle);
    const double cells_t = 1.1 * reach_m / (v.m_t * 2.0 / n * v.sin_angle);
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
    FaceCoord fc;
    FaceFrame frame;
    Vec3 e1, e2;  // orthonormal: e1 = e_s, e2 = e_t made orthogonal
    double band_start_m = 0.0;
    std::array<FaceView, 6> views;
};

LunarSurfaceDetail::LunarSurfaceDetail(config::MoonSurfaceConfig config, std::shared_ptr<const LunarTerrain> lola)
    : config_(std::move(config)), lola_(std::move(lola)) {
    if (!lola_) throw std::invalid_argument("LunarSurfaceDetail: kein Mondgelaende (LOLA) uebergeben");
    const auto& sfd = config_.craters.sfd;
    if (sfd.form != "power" && sfd.form != "exponential") {
        throw std::invalid_argument("LunarSurfaceDetail: unbekannte Groessenverteilung '" + sfd.form + "'");
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
        o.density_per_m2 = sfd.cumulative_density_per_m2(d_lo) - sfd.cumulative_density_per_m2(d_hi);
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
        roughness_octaves_.push_back({static_cast<std::int64_t>(n), face_edge_m / static_cast<double>(n)});
    }
    roughness_scale_m_ = derive_roughness_scale();
}

double LunarSurfaceDetail::finest_wavelength_m() const { return config_.craters.sfd.min_diameter_m; }

double LunarSurfaceDetail::band_start_m(const Vec3& unit) const {
    const auto blend = lola_->tile_blend(unit);
    return blend.weight * blend.scale_m + (1.0 - blend.weight) * lola_->global_resolution_m();
}

LunarSurfaceDetail::Query LunarSurfaceDetail::make_query(const Vec3& unit) const {
    Query q;
    q.p = unit;
    q.fc = face_coord_of(unit);
    q.frame = face_frame(q.fc.face, q.fc.s, q.fc.t, sphere_radius_m_);
    q.e1 = q.frame.e_s;
    q.e2 = math::normalized(q.frame.e_t - q.e1 * dot(q.frame.e_t, q.e1));
    q.band_start_m = band_start_m(unit);
    for (int face = 0; face < 6; ++face) {
        if (dot(unit, face_normal(face)) <= 1e-3) continue;
        FaceView& v = q.views[static_cast<size_t>(face)];
        v.used = true;
        v.c = face == q.fc.face ? q.fc : face_coord_on(face, unit);
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
    const double n_lo = c.sfd.cumulative_density_per_m2(o.min_diameter_m);
    const double n_hi = c.sfd.cumulative_density_per_m2(o.max_diameter_m);
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

double LunarSurfaceDetail::craters_at(const Query& q, double limit_m) const {
    double sum = 0.0;
    for (size_t k = 0; k < crater_octaves_.size(); ++k) {
        const CraterOctave& o = crater_octaves_[k];
        const double w = limit_m > 0.0
                             ? smooth01(std::log2(std::sqrt(o.min_diameter_m * o.max_diameter_m) /
                                                  (box_exact_fraction * limit_m)) + 1.0)
                             : 1.0;
        if (w < 1.0) sum += (1.0 - w) * crater_mean_field_m(static_cast<int>(k), q.band_start_m);
        if (w <= 0.0) continue;
        const double reach =
            crater_reach_per_diameter * o.max_diameter_m + (limit_m > 0.0 ? limit_m * std::sqrt(0.5) : 0.0);
        const double n = static_cast<double>(std::int64_t{1} << o.level);
        for (int face = 0; face < 6; ++face) {
            const FaceView& v = q.views[static_cast<size_t>(face)];
            if (!v.used) continue;
            const CellRange r = cell_range(v, o.level, reach);
            const double cull = 1.05 * reach * v.cull_factor;
            for (std::int64_t j = r.j0; j <= r.j1; ++j) {
                for (std::int64_t i = r.i0; i <= r.i1; ++i) {
                    const FaceCell cell{face, o.level, i, j};
                    const std::uint64_t h = crater_cell_hash(cell);
                    const int count = crater_count(static_cast<int>(k), cell, h);
                    for (int m = 0; m < count; ++m) {
                        const auto [s01, t01] = crater_position_draw(h, m);
                        const double ds = (-1.0 + 2.0 * (static_cast<double>(i) + s01) / n) - v.c.s;
                        const double dt = (-1.0 + 2.0 * (static_cast<double>(j) + t01) / n) - v.c.t;
                        if (std::hypot(ds * v.m_s, dt * v.m_t) > cull) continue;
                        Crater cr;
                        if (!complete_crater(static_cast<int>(k), cell, h, m, s01, t01, cr)) continue;
                        if (limit_m > 0.0) {
                            const Vec3 offset = (cr.centre_unit - q.p) * sphere_radius_m_;
                            sum += w * crater_box_mean_m(cr, dot(offset, q.e1), dot(offset, q.e2), limit_m);
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
    const std::int64_t n = roughness_octaves_[static_cast<size_t>(octave)].cells_per_edge;
    if (j <= 0 || j >= n || k <= 0 || k >= n) {
        // On a face edge or beyond it: the node of the face that owns the direction, the lowest face index where two
        // or three faces share it (edge / corner nodes hash identically from every side); beyond an edge (square
        // means only) the nearest node of the neighbouring face.
        const double nd = static_cast<double>(n);
        const Vec3 d = face_direction(face, -1.0 + 2.0 * static_cast<double>(j) / nd,
                                      -1.0 + 2.0 * static_cast<double>(k) / nd);
        const double comps[3] = {d.x, d.y, d.z};
        const double top = std::max({std::abs(d.x), std::abs(d.y), std::abs(d.z)});
        int owner = 6;
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(comps[axis]) >= top - 1e-12) owner = std::min(owner, 2 * axis + (comps[axis] < 0.0 ? 1 : 0));
        }
        const FaceCoord c = face_coord_on(owner, d);
        face = owner;
        j = std::clamp(static_cast<std::int64_t>(std::llround((c.s + 1.0) * 0.5 * nd)), std::int64_t{0}, n);
        k = std::clamp(static_cast<std::int64_t>(std::llround((c.t + 1.0) * 0.5 * nd)), std::int64_t{0}, n);
    }
    const std::uint64_t h = surface_hash((surface_stream::roughness << 8) | static_cast<std::uint64_t>(face),
                                         static_cast<std::uint64_t>(octave), static_cast<std::uint64_t>(j),
                                         static_cast<std::uint64_t>(k));
    return 2.0 * hash_unit(h) - 1.0;
}

double LunarSurfaceDetail::roughness_at(const Query& q, double limit_m, double scale_m) const {
    const auto& r = config_.roughness;
    double sum = 0.0;
    for (size_t m = 0; m < roughness_octaves_.size(); ++m) {
        const RoughnessOctave& o = roughness_octaves_[m];
        // Top: full up to half the band start, none from the band start (LOLA resolves 2-pixel wavelengths, [E]).
        const double w_top = smooth01(std::log2(q.band_start_m / o.spacing_m));
        if (w_top <= 0.0) continue;
        const double w_box = limit_m > 0.0 ? smooth01(std::log2(o.spacing_m / (box_exact_fraction * limit_m)) + 1.0) : 1.0;
        if (w_box <= 0.0) continue;
        const double amplitude =
            scale_m * unit_variance_gain * std::pow(q.frame.scale * o.spacing_m / r.baseline_m, r.hurst_exponent);
        const std::int64_t n = o.cells_per_edge;
        const double nd = static_cast<double>(n);
        const double u = (q.fc.s + 1.0) * 0.5 * nd, v = (q.fc.t + 1.0) * 0.5 * nd;
        const int oct = static_cast<int>(m);
        double value = 0.0;
        if (limit_m <= 0.0) {
            const std::int64_t j0 = std::clamp(static_cast<std::int64_t>(std::floor(u)), std::int64_t{0}, n - 1);
            const std::int64_t k0 = std::clamp(static_cast<std::int64_t>(std::floor(v)), std::int64_t{0}, n - 1);
            const double fu = fade(std::clamp(u - static_cast<double>(j0), 0.0, 1.0));
            const double fv = fade(std::clamp(v - static_cast<double>(k0), 0.0, 1.0));
            const int face = q.fc.face;
            value = (1.0 - fv) * ((1.0 - fu) * roughness_node(oct, face, j0, k0) + fu * roughness_node(oct, face, j0 + 1, k0)) +
                    fv * ((1.0 - fu) * roughness_node(oct, face, j0, k0 + 1) + fu * roughness_node(oct, face, j0 + 1, k0 + 1));
        } else {
            // Exact mean over the square: the basis is a product of 1-D fade hats, so the mean separates.
            const double du = 0.5 * limit_m / q.frame.m_s * 0.5 * nd, dv = 0.5 * limit_m / q.frame.m_t * 0.5 * nd;
            const double u0 = u - du, u1 = u + du, v0 = v - dv, v1 = v + dv;
            const std::int64_t ja = static_cast<std::int64_t>(std::floor(u0)), jb = static_cast<std::int64_t>(std::floor(u1)) + 1;
            const std::int64_t ka = static_cast<std::int64_t>(std::floor(v0)), kb = static_cast<std::int64_t>(std::floor(v1)) + 1;
            constexpr std::int64_t max_nodes = 64;
            if (jb - ja + 1 > max_nodes || kb - ka + 1 > max_nodes) {
                throw std::logic_error("LunarSurfaceDetail: square mean spans too many lattice nodes");
            }
            std::array<double, max_nodes> wy{};
            for (std::int64_t k = ka; k <= kb; ++k) wy[static_cast<size_t>(k - ka)] = basis_integral(k, v0, v1);
            for (std::int64_t j = ja; j <= jb; ++j) {
                const double wx = basis_integral(j, u0, u1);
                if (wx == 0.0) continue;
                double row = 0.0;
                for (std::int64_t k = ka; k <= kb; ++k) {
                    const double wk = wy[static_cast<size_t>(k - ka)];
                    if (wk != 0.0) row += wk * roughness_node(oct, q.fc.face, j, k);
                }
                value += wx * row;
            }
            value /= (u1 - u0) * (v1 - v0);
        }
        sum += w_top * w_box * amplitude * value;
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
                Query q;
                q.p = p;
                q.fc = face_coord_of(p);
                q.frame = face_frame(q.fc.face, q.fc.s, q.fc.t, sphere_radius_m_);
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
