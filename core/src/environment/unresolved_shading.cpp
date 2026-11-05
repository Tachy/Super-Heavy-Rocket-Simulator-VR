#include "starsim/environment/unresolved_shading.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <mutex>
#include <utility>
#include <stdexcept>
#include <thread>
#include <vector>

#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/hapke.hpp"
#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/rock_templates.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

namespace {

constexpr double pi = constants::pi;

// The generator's quintic fade F(t) = 6 t^5 - 15 t^4 + 10 t^3 and its derivative.
double fade(double t) { return t * t * t * (t * (6.0 * t - 15.0) + 10.0); }
double fade_derivative(double t) { return 30.0 * t * t * (t - 1.0) * (t - 1.0); }

// Mean square gradient (magnitude) of one value-noise octave per (node variance / spacing^2): the field is
// sum v_jk (1 - F)(1 - F) ... with iid node values, so d f / d x = F'(u) / s [(1 - F(v)) (v10 - v00) + F(v) (v11 - v01)]
// has variance 2 sigma^2 / s^2 <F'^2> <(1 - F)^2 + F^2> per axis (u, v uniform over the cell); two axes.
double value_noise_gradient_factor() {
    constexpr int n = 4096;  // midpoint rule; the integrands are polynomials (error ~1e-8)
    double f2 = 0.0, mix = 0.0;
    for (int k = 0; k < n; ++k) {
        const double t = (k + 0.5) / n;
        f2 += fade_derivative(t) * fade_derivative(t);
        mix += (1.0 - fade(t)) * (1.0 - fade(t)) + fade(t) * fade(t);
    }
    return 2.0 * (2.0 * (f2 / n) * (mix / n));
}

// |Phi(q)|^2 of the value-noise kernel phi(t) = 1 - F(|t|) (|t| < 1), Phi(q) = 2 int_0^1 (1 - F(t)) cos(q t) dt, on a q
// grid, with the two moments int q^2 |Phi|^2 dq and int |Phi|^2 dq ( = the r -> 0 limits of the transfer integrals).
struct KernelSpectrum {
    static constexpr int n = 16000;
    static constexpr double dq = 0.005;  // q to 80: |Phi|^2 ~ q^-8 is < 1e-14 there
    std::vector<double> power;
    double a0 = 0.0, b0 = 0.0;
    KernelSpectrum() : power(n) {
        constexpr int nt = 256;
        for (int k = 0; k < n; ++k) {
            const double q = (k + 0.5) * dq;
            double phi = 0.0;
            for (int i = 0; i < nt; ++i) {
                const double t = (i + 0.5) / nt;
                phi += (1.0 - fade(t)) * std::cos(q * t);
            }
            phi *= 2.0 / nt;
            power[static_cast<size_t>(k)] = phi * phi;
            a0 += q * q * phi * phi * dq;
            b0 += phi * phi * dq;
        }
    }
};

const KernelSpectrum& kernel_spectrum() {
    static const KernelSpectrum s;
    return s;
}

double sinc(double u) { return std::abs(u) < 1e-9 ? 1.0 : std::sin(u) / u; }

// regolith_drawn_fraction tabulated in log2 r (1/16 octave) over [2^-8, 2^6]; outside 1 / the last.
struct RegolithFractionTable {
    static constexpr int per_octave = 16, lo = -8, hi = 6;
    std::vector<double> value;
    RegolithFractionTable() {
        const auto& s = kernel_spectrum();
        value.resize((hi - lo) * per_octave + 1);
        for (size_t j = 0; j < value.size(); ++j) {
            const double r = std::exp2(lo + static_cast<double>(j) / per_octave);
            double a = 0.0, b = 0.0;
            for (int k = 0; k < KernelSpectrum::n; ++k) {
                const double q = (k + 0.5) * KernelSpectrum::dq;
                const double box = sinc(0.5 * q * r);
                const double p = s.power[static_cast<size_t>(k)] * box * box;
                const double d = std::sin(q * r);
                a += p * d * d;
                b += p;
            }
            a *= KernelSpectrum::dq / (r * r);
            b *= KernelSpectrum::dq;
            value[j] = a * b / (s.a0 * s.b0);
        }
    }
};

// Integral of the crater profile's squared gradient over the plane, per D^2 (crater_profile_m: parabolic bowl to R,
// rim (r/R)^-3 law to 3 R): 2 pi (depth/D)^2 + 2 pi (81/26)^2 (rim/D)^2 (1 - 3^-6) / 6.
double crater_gradient_integral_per_d2(double depth_to_diameter, double rim_to_diameter) {
    const double rim_slope = 81.0 / 26.0;
    return 2.0 * pi * depth_to_diameter * depth_to_diameter +
           2.0 * pi * rim_slope * rim_slope * rim_to_diameter * rim_to_diameter * (1.0 - 1.0 / 729.0) / 6.0;
}

const config::SizeFrequency& require_power(const config::SizeFrequency& sfd, const char* what) {
    if (sfd.form != "power") {
        throw std::invalid_argument(std::string("Unaufgeloeste Schattierung: ") + what +
                                    " braucht ein Potenzgesetz (form = \"power\")");
    }
    return sfd;
}

// Mean exposed height / D of generated rocks: per octave the cells around a fixed direction (face 0's centre) until
// enough rocks, height of the rock top above the base on the rock's axis; octaves weighted by their cover.
double measured_eta(const LunarRocks& rocks) {
    constexpr int wanted = 300;   // [D] rocks per octave (eta's spread ~0.1: mean to ~1 %)
    constexpr int max_ring = 40;  // up to 81 x 81 cells per octave
    const math::Vec3 probe{1.0, 0.0, 0.0};
    double weighted = 0.0, weights = 0.0;
    for (const auto& o : rocks.octaves()) {
        const FaceCell centre = face_cell_of(probe, o.level);
        const std::int64_t n = std::int64_t{1} << o.level;
        double sum = 0.0;
        int count = 0;
        for (int ring = 0; ring <= max_ring && count < wanted; ++ring) {
            for (int di = -ring; di <= ring; ++di) {
                for (int dj = -ring; dj <= ring; ++dj) {
                    if (std::max(std::abs(di), std::abs(dj)) != ring) continue;
                    FaceCell c = centre;
                    c.i += di;
                    c.j += dj;
                    if (c.i < 0 || c.j < 0 || c.i >= n || c.j >= n) continue;
                    for (const Rock& r : rocks.rocks_in_cell(c, o.min_diameter_m)) {
                        if (r.diameter_m >= o.max_diameter_m) continue;
                        const auto top = rocks.rock_top(r, r.base_unit);
                        if (!top) continue;
                        sum += (top->first - r.base_height_m) / r.diameter_m;
                        ++count;
                    }
                }
            }
        }
        if (count == 0) continue;
        const double cover = rocks.cover_fraction(o.min_diameter_m, o.max_diameter_m);
        weighted += cover * sum / count;
        weights += cover;
    }
    if (weights <= 0.0) throw std::runtime_error("Unaufgeloeste Schattierung: keine Steine zum Vermessen");
    return weighted / weights;
}

// Mesh-drawn share of one crater profile (D = 1): lattice of spacing L at 3 x 3 offsets, vertex = L x L box mean
// (midpoint rule, sub-steps <= 0.1 D), slope = central differences over 2 L; drawn integral = sum |grad|^2 L^2.
double crater_profile_drawn_share(double depth, double rim, double spacing) {
    Crater c;
    c.diameter_m = 1.0;
    c.depth_m = depth;
    c.rim_height_m = rim;
    const double full = crater_gradient_integral_per_d2(depth, rim);
    const double reach = 1.5 + 2.0 * spacing;  // profile radius 3 R plus two vertices
    const int sub = std::max(2, static_cast<int>(std::ceil(spacing / 0.1)));
    constexpr int offsets = 3;
    double drawn = 0.0;
    for (int oy = 0; oy < offsets; ++oy) {
        for (int ox = 0; ox < offsets; ++ox) {
            const double x0 = (ox + 0.5) / offsets * spacing, y0 = (oy + 0.5) / offsets * spacing;
            const int i_lo = static_cast<int>(std::floor((-reach - x0) / spacing));
            const int i_hi = static_cast<int>(std::ceil((reach - x0) / spacing));
            const int j_lo = static_cast<int>(std::floor((-reach - y0) / spacing));
            const int j_hi = static_cast<int>(std::ceil((reach - y0) / spacing));
            const int nx = i_hi - i_lo + 1, ny = j_hi - j_lo + 1;
            std::vector<double> z(static_cast<size_t>(nx) * static_cast<size_t>(ny));
            for (int j = 0; j < ny; ++j) {
                for (int i = 0; i < nx; ++i) {
                    const double cx = x0 + (i_lo + i) * spacing, cy = y0 + (j_lo + j) * spacing;
                    double sum = 0.0;
                    for (int b = 0; b < sub; ++b) {
                        const double y = cy + ((b + 0.5) / sub - 0.5) * spacing;
                        for (int a = 0; a < sub; ++a) {
                            const double x = cx + ((a + 0.5) / sub - 0.5) * spacing;
                            sum += crater_profile_m(c, std::sqrt(x * x + y * y));
                        }
                    }
                    z[static_cast<size_t>(j) * static_cast<size_t>(nx) + static_cast<size_t>(i)] = sum / (sub * sub);
                }
            }
            for (int j = 1; j < ny - 1; ++j) {
                for (int i = 1; i < nx - 1; ++i) {
                    const auto at = [&](int ii, int jj) {
                        return z[static_cast<size_t>(jj) * static_cast<size_t>(nx) + static_cast<size_t>(ii)];
                    };
                    const double gx = (at(i + 1, j) - at(i - 1, j)) / (2.0 * spacing);
                    const double gy = (at(i, j + 1) - at(i, j - 1)) / (2.0 * spacing);
                    drawn += (gx * gx + gy * gy) * spacing * spacing;
                }
            }
        }
    }
    return drawn / (offsets * offsets) / full;
}

double table_lookup(const std::vector<double>& table, double x) {
    // Linear in the table index x (clamped).
    if (table.empty()) return 0.0;
    const double xc = std::clamp(x, 0.0, static_cast<double>(table.size() - 1));
    const size_t k = std::min(static_cast<size_t>(xc), table.size() - 2);
    const double f = xc - static_cast<double>(k);
    return table[k] * (1.0 - f) + table[k + 1] * f;
}

double crater_fraction_from_table(const UnresolvedShadingModel& m, double diameter_over_spacing) {
    if (diameter_over_spacing <= 0.0) return 0.0;
    const double x = 2.0 * (std::log2(diameter_over_spacing) + UnresolvedShadingModel::crater_fraction_lowest_octaves);
    if (x < 0.0) return 0.0;
    return table_lookup(m.crater_fraction, x);
}

double erf_lambda(double a) {
    // Smith's Lambda for a Gaussian surface (Beckmann), a = tan(elevation) / rms slope magnitude.
    return 0.5 * (std::erf(a) - 1.0) + std::exp(-a * a) / (2.0 * a * std::sqrt(pi));
}

}  // namespace

UnresolvedShadingModel make_unresolved_shading_model(const LunarSurfaceDetail& detail, const LunarRocks* rocks) {
    const auto& cfg = detail.config();
    UnresolvedShadingModel m;
    m.globe_slope_variance = slope_variance_from_theta_bar(cfg.hapke.theta_bar_rad);
    m.total_slope_variance = m.globe_slope_variance;  // the rocks' share comes out below (with the rocks)

    // Regolith: octave k's nodes have variance (scale x relative_amplitude)^2 / 3 = scale^2 (s / baseline)^(2 H) (node
    // values uniform in [-1, 1]); its mean square slope is g x that / s^2 (value_noise_gradient_factor).
    const auto& octaves = detail.roughness_octaves();
    if (octaves.empty()) throw std::invalid_argument("Unaufgeloeste Schattierung: keine Rauheitsoktaven");
    const double g_noise = value_noise_gradient_factor();
    const double scale = detail.roughness_scale_m();
    for (const auto& o : octaves) {
        const double amplitude = scale * o.relative_amplitude / std::sqrt(3.0);
        m.regolith_octave_spacing_m.push_back(o.spacing_m);
        m.regolith_octave_variance.push_back(g_noise * amplitude * amplitude / (o.spacing_m * o.spacing_m));
    }

    // Craters: n(D) = b rho ref^b D^(-b-1); each crater adds I D^2 (fresh / degraded mix).
    const auto& craters = cfg.craters;
    const auto& csfd = require_power(craters.sfd, "craters.sfd");
    const double rim_per_depth = craters.rim_height_to_diameter / craters.depth_to_diameter_fresh;
    const double i_fresh = crater_gradient_integral_per_d2(craters.depth_to_diameter_fresh, craters.rim_height_to_diameter);
    const double i_degraded = crater_gradient_integral_per_d2(craters.depth_to_diameter_degraded,
                                                              rim_per_depth * craters.depth_to_diameter_degraded);
    const double i_mix = craters.degraded_fraction * i_degraded + (1.0 - craters.degraded_fraction) * i_fresh;
    m.crater_exponent = 2.0 - csfd.exponent;
    m.crater_variance_coefficient =
        csfd.exponent * csfd.density_per_m2 * std::pow(csfd.reference_diameter_m, csfd.exponent) * i_mix;
    m.crater_min_diameter_m = csfd.min_diameter_m;
    m.crater_max_diameter_m = csfd.max_diameter_m;
    m.crater_fraction.resize(UnresolvedShadingModel::crater_fraction_samples);
    for (int k = 0; k < UnresolvedShadingModel::crater_fraction_samples; ++k) {
        const double d_over_l = std::exp2(0.5 * k - UnresolvedShadingModel::crater_fraction_lowest_octaves);
        m.crater_fraction[static_cast<size_t>(k)] = crater_drawn_fraction(craters, 1.0 / d_over_l);
    }

    // The fill light's hemispherical albedo of the regolith (Task 12): the globe's parameters and mean slope.
    m.hemispherical_albedo.resize(UnresolvedShadingModel::lit_gain_samples);
    for (int k = 0; k < UnresolvedShadingModel::lit_gain_samples; ++k) {
        const double e = std::max(k * UnresolvedShadingModel::lit_gain_step_rad, lit_gain_floor_rad);
        m.hemispherical_albedo[static_cast<size_t>(k)] =
            hapke_hemispherical_albedo(cfg.hapke, std::sin(e), cfg.hapke.theta_bar_rad);
    }

    m.drawn_table.resize(UnresolvedShadingModel::drawn_table_samples);
    for (int k = 0; k < UnresolvedShadingModel::drawn_table_samples; ++k) {
        const double spacing = UnresolvedShadingModel::drawn_table_min_spacing_m *
                               std::exp2(static_cast<double>(k) / UnresolvedShadingModel::drawn_table_per_octave);
        m.drawn_table[static_cast<size_t>(k)] = drawn_slope_variance_direct(
            m, spacing, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    }

    if (rocks != nullptr) {
        const auto& rsfd = require_power(cfg.rocks.sfd, "rocks.sfd");
        m.rock_cover_exponent = 2.0 - rsfd.exponent;
        // cover = (pi / 4) b rho ref^b (D1^q - D0^q) / q (LunarRocks::cover_fraction), x abundance_scale.
        m.rock_cover_coefficient = cfg.rocks.abundance_scale * 0.25 * pi * rsfd.exponent * rsfd.density_per_m2 *
                                   std::pow(rsfd.reference_diameter_m, rsfd.exponent) / m.rock_cover_exponent;
        m.rock_min_diameter_m = rsfd.min_diameter_m;
        m.rock_max_diameter_m = rsfd.max_diameter_m;
        m.rock_exposed_height_per_diameter = measured_eta(*rocks);
        // Standard geometry of the albedo mosaics (i 30, e 0, g 30 deg) with the globe's theta_bar.
        config::HapkeParameters rock = cfg.hapke;
        rock.w = std::min(cfg.hapke.w * cfg.rocks.albedo_to_regolith, 0.999);
        const double mu0 = std::cos(30.0 * constants::deg_to_rad), g = 30.0 * constants::deg_to_rad;
        m.rock_reflectance_ratio = hapke_reflectance(rock, mu0, 1.0, g, cfg.hapke.theta_bar_rad) /
                                   hapke_reflectance(cfg.hapke, mu0, 1.0, g, cfg.hapke.theta_bar_rad);
        // G(e) and sigma(e) of the generated angular rocks (Task 11 fix C, R1; the half-spheroid laws rock_lit_gain /
        // rock_shadow_per_cover stay as the reference shapes of the tests).
        const RockLightTables tables =
            template_rock_light_tables(cfg.rocks.burial_fraction_mean, cfg.rocks.burial_fraction_sd);
        m.rock_lit_gain = tables.gain;
        m.rock_shadow = tables.shadow;
        // Oblique views (Task 12): the ground the rocks hide from the eye and their seen sides.
        const RockViewModel view = template_rock_view_tables(cfg.rocks.burial_fraction_mean, cfg.rocks.burial_fraction_sd);
        m.rock_view_excess = view.excess;
        m.rock_view_gain_ratio = view.gain_ratio;
        // The rocks' share of the globe's mean slope (header comment): out of the regolith's budget.
        m.rock_area_share = 1.0 - std::exp(-rock_cover(m, m.rock_min_diameter_m, m.rock_max_diameter_m));
        // The templates' own mean |grad h| per plan area (review n1 of the Task 11 fix C re-review: the half-spheroid's
        // pi eta before), the shape the light tables describe.
        m.rock_mean_tan = m.rock_area_share * tables.mean_tan;
        const double globe_mean_tan = 0.5 * pi * std::tan(cfg.hapke.theta_bar_rad);
        const double regolith_mean_tan = (globe_mean_tan - m.rock_mean_tan) / (1.0 - m.rock_area_share);
        if (!(regolith_mean_tan > 0.0)) {
            throw std::invalid_argument("Unaufgeloeste Schattierung: die Steine tragen mehr Hangneigung als der Hapke-Fit");
        }
        const double t = 2.0 / pi * regolith_mean_tan;
        m.total_slope_variance = pi * t * t;
    }
    return m;
}

double slope_variance_from_theta_bar(double theta_bar_rad) {
    const double t = std::tan(theta_bar_rad);
    return pi * t * t;
}

double theta_bar_from_slope_variance(double slope_variance) {
    return std::atan(std::sqrt(std::max(slope_variance, 0.0) / pi));
}

double regolith_drawn_fraction(double spacing_over_lattice) {
    static const RegolithFractionTable table;
    if (spacing_over_lattice <= 0.0) return 1.0;
    const double x = (std::log2(spacing_over_lattice) - RegolithFractionTable::lo) * RegolithFractionTable::per_octave;
    if (x <= 0.0) return table.value.front();
    return table_lookup(table.value, x);
}

double crater_drawn_fraction(const config::CraterPopulation& craters, double spacing_over_diameter) {
    const double rim_per_depth = craters.rim_height_to_diameter / craters.depth_to_diameter_fresh;
    const double d_f = craters.depth_to_diameter_fresh, r_f = craters.rim_height_to_diameter;
    const double d_d = craters.depth_to_diameter_degraded, r_d = rim_per_depth * d_d;
    const double w_f = (1.0 - craters.degraded_fraction) * crater_gradient_integral_per_d2(d_f, r_f);
    const double w_d = craters.degraded_fraction * crater_gradient_integral_per_d2(d_d, r_d);
    return (w_f * crater_profile_drawn_share(d_f, r_f, spacing_over_diameter) +
            w_d * crater_profile_drawn_share(d_d, r_d, spacing_over_diameter)) /
           (w_f + w_d);
}

double crater_drawn_fraction_table(const UnresolvedShadingModel& m, double diameter_over_spacing) {
    return crater_fraction_from_table(m, diameter_over_spacing);
}

double drawn_slope_variance_direct(const UnresolvedShadingModel& m, double spacing_m, double regolith_top_spacing_m,
                                   double crater_top_diameter_m) {
    double v = 0.0;
    for (size_t k = 0; k < m.regolith_octave_spacing_m.size(); ++k) {
        const double s = m.regolith_octave_spacing_m[k];
        if (s > regolith_top_spacing_m) break;
        v += m.regolith_octave_variance[k] * regolith_drawn_fraction(spacing_m / s);
    }
    // Craters: int coef D^(q - 1) g(D / L) dD, midpoint rule in ln D (16 per octave).
    const double d_hi = std::min(crater_top_diameter_m, m.crater_max_diameter_m);
    if (d_hi > m.crater_min_diameter_m) {
        const double octaves = std::log2(d_hi / m.crater_min_diameter_m);
        const int n = std::max(1, static_cast<int>(std::ceil(16.0 * octaves)));
        const double step = octaves / n;
        for (int k = 0; k < n; ++k) {
            const double d = m.crater_min_diameter_m * std::exp2((k + 0.5) * step);
            v += m.crater_variance_coefficient * std::pow(d, m.crater_exponent) * step * std::log(2.0) *
                 crater_fraction_from_table(m, d / spacing_m);
        }
    }
    return v;
}

double drawn_slope_variance(const UnresolvedShadingModel& m, double spacing_m) {
    if (!(spacing_m > 0.0) || std::isinf(spacing_m)) return std::isinf(spacing_m) ? 0.0 : m.drawn_table.front();
    const double x = std::log2(spacing_m / UnresolvedShadingModel::drawn_table_min_spacing_m) *
                     UnresolvedShadingModel::drawn_table_per_octave;
    if (x >= static_cast<double>(m.drawn_table.size() - 1)) return 0.0;
    return table_lookup(m.drawn_table, x);
}

double residual_theta_bar(const UnresolvedShadingModel& m, double spacing_m) {
    return theta_bar_from_slope_variance(m.total_slope_variance - drawn_slope_variance(m, spacing_m));
}

double unshadowed_slope_variance(const UnresolvedShadingModel& m, double spacing_m, double horizon_spacing_m) {
    return std::max(drawn_slope_variance(m, spacing_m) - drawn_slope_variance(m, horizon_spacing_m), 0.0);
}

double smith_lit_fraction(double tan_elevation, double slope_variance) {
    if (tan_elevation <= 0.0) return 0.0;
    if (slope_variance <= 0.0) return 1.0;
    const double a = tan_elevation / std::sqrt(slope_variance);
    return 1.0 / (1.0 + erf_lambda(a));
}

double smith_visible_lit_fraction(double tan_light, double tan_view, double azimuth_difference_rad,
                                  double slope_variance) {
    if (tan_light <= 0.0) return 0.0;
    if (slope_variance <= 0.0) return 1.0;
    const double s = std::sqrt(slope_variance);
    const double lambda_l = erf_lambda(tan_light / s);
    const double lambda_v = (std::isinf(tan_view) || !(tan_view > 0.0)) ? 0.0 : erf_lambda(tan_view / s);
    double phi = std::fmod(std::abs(azimuth_difference_rad), 2.0 * pi);
    if (phi > pi) phi = 2.0 * pi - phi;
    const double correlation = 4.41 * phi / (4.41 * phi + 1.0);
    return (1.0 + lambda_v) /
           (1.0 + std::max(lambda_l, lambda_v) + correlation * std::min(lambda_l, lambda_v));
}

double rock_cover(const UnresolvedShadingModel& m, double d_lo_m, double d_hi_m) {
    const double lo = std::max(d_lo_m, m.rock_min_diameter_m), hi = std::min(d_hi_m, m.rock_max_diameter_m);
    if (hi <= lo || m.rock_cover_coefficient == 0.0) return 0.0;
    return m.rock_cover_coefficient * (std::pow(hi, m.rock_cover_exponent) - std::pow(lo, m.rock_cover_exponent));
}

double rock_shadow_per_cover(double eta, double sin_elevation) {
    if (sin_elevation <= 0.0) return std::numeric_limits<double>::infinity();
    const double s = std::min(sin_elevation, 1.0);
    const double cot = std::sqrt(1.0 - s * s) / s;
    return 0.5 * (std::sqrt(1.0 + 4.0 * eta * eta * cot * cot) - 1.0);
}

double rock_lit_gain(double eta, double elevation_rad) {
    const double e = std::clamp(elevation_rad, lit_gain_floor_rad, 0.5 * pi);
    const double se = std::sin(e), ce = std::cos(e);
    const double h = 2.0 * eta;  // height / plan radius
    // Plan disc rho = sin u (dense at the steep rim), dA = sin u cos u du dphi; midpoint rule in both.
    constexpr int nu = 400, nphi = 400;  // [D] < 1e-4 against 4x finer
    double sum = 0.0;
    for (int iu = 0; iu < nu; ++iu) {
        const double u = (iu + 0.5) / nu * 0.5 * pi;
        const double su = std::sin(u), cu = std::cos(u);
        const double norm = std::sqrt(h * h * su * su + cu * cu);
        const double nz = cu / norm, nr = h * su / norm;
        double ring = 0.0;
        for (int ip = 0; ip < nphi; ++ip) {
            const double phi = (ip + 0.5) / nphi * 2.0 * pi;
            const double mu0 = nr * std::cos(phi) * ce + nz * se;
            if (mu0 > 0.0) ring += mu0 / (mu0 + nz);
        }
        sum += ring / nphi * 2.0 * pi * su * cu;
    }
    const double mean = sum * (0.5 * pi / nu) / pi;
    return mean / (se / (se + 1.0));
}

double rock_lit_gain_table(const UnresolvedShadingModel& m, double elevation_rad) {
    if (m.rock_lit_gain.empty()) return 1.0;
    const double x = std::clamp(elevation_rad / UnresolvedShadingModel::lit_gain_step_rad, 0.0,
                                static_cast<double>(UnresolvedShadingModel::lit_gain_samples - 1));
    const int k = std::min(static_cast<int>(x), UnresolvedShadingModel::lit_gain_samples - 2);
    const double f = x - k;
    return m.rock_lit_gain[static_cast<size_t>(k)] * (1.0 - f) + m.rock_lit_gain[static_cast<size_t>(k) + 1] * f;
}

double rock_shadow_table(const UnresolvedShadingModel& m, double elevation_rad) {
    if (m.rock_shadow.empty()) return 0.0;
    const double x = std::clamp(elevation_rad / UnresolvedShadingModel::lit_gain_step_rad, 0.0,
                                static_cast<double>(UnresolvedShadingModel::lit_gain_samples - 1));
    const int k = std::min(static_cast<int>(x), UnresolvedShadingModel::lit_gain_samples - 2);
    const double f = x - k;
    return m.rock_shadow[static_cast<size_t>(k)] * (1.0 - f) + m.rock_shadow[static_cast<size_t>(k) + 1] * f;
}

RockLightTables rock_light_tables(const std::vector<RockColumn>& columns, double cell) {
    constexpr int n = UnresolvedShadingModel::lit_gain_samples;
    RockLightTables out;
    out.gain.assign(n, 0.0);
    out.shadow.assign(n, 0.0);
    if (columns.empty()) return out;
    std::array<double, n> se{}, ce{}, cot{};
    for (int k = 0; k < n; ++k) {
        const double e = std::clamp(k * UnresolvedShadingModel::lit_gain_step_rad, lit_gain_floor_rad, 0.5 * pi);
        se[static_cast<size_t>(k)] = std::sin(e);
        ce[static_cast<size_t>(k)] = std::cos(e);
        cot[static_cast<size_t>(k)] = ce[static_cast<size_t>(k)] / se[static_cast<size_t>(k)];
    }
    double reach = 0.0;
    for (const RockColumn& c : columns) reach = std::max(reach, std::hypot(c.x, c.y));
    const int bins = 2 * static_cast<int>(std::ceil(reach / cell)) + 3;
    const double plan_area = static_cast<double>(columns.size()) * cell * cell;
    std::vector<double> footprint_start(static_cast<size_t>(bins));
    std::vector<double> shadow_start(static_cast<size_t>(bins) * n);
    for (int a = 0; a < rock_light_azimuths; ++a) {
        const double psi = 2.0 * pi * (a + 0.5) / rock_light_azimuths;
        const double cp = std::cos(psi), sp = std::sin(psi);
        // Mean half-width along s of a column square cut by a line along s: cell / 2 / max(|cos|, |sin|) on average
        // over the cut position is the square's area over its width across s; [D] the area-preserving choice.
        const double half_extent = 0.5 * cell * cell / (cell * (std::abs(cp) + std::abs(sp)));
        std::fill(footprint_start.begin(), footprint_start.end(), std::numeric_limits<double>::infinity());
        std::fill(shadow_start.begin(), shadow_start.end(), std::numeric_limits<double>::infinity());
        for (const RockColumn& c : columns) {
            // s along the light's azimuth (the light stands toward +s), w across; the shadow of the top falls at
            // s - height cot e on the ground.
            const double s = c.x * cp + c.y * sp, w = -c.x * sp + c.y * cp;
            const auto bin = static_cast<size_t>(std::floor(w / cell + 0.5) + (bins - 1) / 2);
            // The footprint is the union of the column squares (its area is the plan area below); the shadow comes
            // from the top surface's points.
            footprint_start[bin] = std::min(footprint_start[bin], s - half_extent);
            const double nh = c.normal.x * cp + c.normal.y * sp, nz = c.normal.z;
            for (int k = 0; k < n; ++k) {
                const auto ku = static_cast<size_t>(k);
                double& start = shadow_start[bin * n + ku];
                start = std::min(start, s - c.height * cot[ku]);
                const double mu0 = nh * ce[ku] + nz * se[ku];
                if (mu0 > 0.0) out.gain[ku] += mu0 / (mu0 + nz);
            }
        }
        for (int b = 0; b < bins; ++b) {
            const double f1 = footprint_start[static_cast<size_t>(b)];
            if (!std::isfinite(f1)) continue;
            for (int k = 0; k < n; ++k) {
                out.shadow[static_cast<size_t>(k)] +=
                    std::max(0.0, f1 - shadow_start[static_cast<size_t>(b) * n + static_cast<size_t>(k)]) * cell;
            }
        }
    }
    for (int k = 0; k < n; ++k) {
        const auto ku = static_cast<size_t>(k);
        out.gain[ku] /= static_cast<double>(columns.size()) * rock_light_azimuths * (se[ku] / (se[ku] + 1.0));
        out.shadow[ku] /= plan_area * rock_light_azimuths;
    }
    // Mean |grad h| of the top per plan area (each column's facet slope; steep rim columns clamped at [D] n_z 1e-3).
    double tan_sum = 0.0;
    for (const RockColumn& c : columns) {
        const double nz = std::max(c.normal.z, 1e-3);
        tan_sum += std::sqrt(std::max(0.0, 1.0 - nz * nz)) / nz;
    }
    out.mean_tan = tan_sum / static_cast<double>(columns.size());
    return out;
}

RockLightTables template_rock_light_tables(double burial_mean, double burial_sd) {
    static std::mutex mutex;
    static std::map<std::pair<double, double>, RockLightTables> cache;
    const std::lock_guard<std::mutex> lock(mutex);
    if (const auto it = cache.find({burial_mean, burial_sd}); it != cache.end()) return it->second;
    constexpr int n = UnresolvedShadingModel::lit_gain_samples;
    constexpr int grid = 64;  // [D] columns per side: half-spheroid G within 0.7 %, sigma 1.1 % of (1 + sigma) (test)
    constexpr int burial_nodes = 9;  // [D] N in [-4, 4] step 1, Gaussian weights (the clamp is in the nodes)
    const RockTemplates& templates = RockTemplates::instance();
    const double cell = 2.0 / grid;
    // Every template's top columns once (the cut only selects them).
    struct Top {
        double x, y, z;
        math::Vec3 normal;
    };
    std::vector<std::vector<Top>> tops(RockTemplates::count);
    for (int id = 0; id < RockTemplates::count; ++id) {
        for (int i = 0; i < grid; ++i) {
            for (int j = 0; j < grid; ++j) {
                const double x = -1.0 + (i + 0.5) * cell, y = -1.0 + (j + 0.5) * cell;
                if (const auto z = templates.column_top(id, x, y)) {
                    tops[static_cast<size_t>(id)].push_back({x, y, *z, templates.normal(id, math::Vec3{x, y, *z})});
                }
            }
        }
    }
    RockLightTables out;
    out.gain.assign(n, 0.0);
    out.shadow.assign(n, 0.0);
    double weights = 0.0;
    for (int q = 0; q < burial_nodes; ++q) {
        const double z = q - 0.5 * (burial_nodes - 1);
        const double weight = std::exp(-0.5 * z * z);
        const double f = std::clamp(burial_mean + burial_sd * z, 0.0, LunarRocks::max_burial_fraction);
        for (int id = 0; id < RockTemplates::count; ++id) {
            const double cut = templates.bottom_z(id) + f * (templates.top_z(id) - templates.bottom_z(id));
            std::vector<RockColumn> columns;
            for (const Top& t : tops[static_cast<size_t>(id)]) {
                if (t.z > cut) columns.push_back({t.x, t.y, t.z - cut, t.normal});
            }
            const RockLightTables one = rock_light_tables(columns, cell);
            for (int k = 0; k < n; ++k) {
                out.gain[static_cast<size_t>(k)] += weight * one.gain[static_cast<size_t>(k)];
                out.shadow[static_cast<size_t>(k)] += weight * one.shadow[static_cast<size_t>(k)];
            }
            out.mean_tan += weight * one.mean_tan;
            weights += weight;
        }
    }
    for (int k = 0; k < n; ++k) {
        out.gain[static_cast<size_t>(k)] /= weights;
        out.shadow[static_cast<size_t>(k)] /= weights;
    }
    out.mean_tan /= weights;
    cache.emplace(std::make_pair(burial_mean, burial_sd), out);
    return out;
}

double rock_statistical_cover(const UnresolvedShadingModel& m, double geometry_min_diameter_m) {
    const double d = geometry_min_diameter_m;
    double cover = rock_cover(m, m.rock_min_diameter_m, d);
    if (!(d < m.rock_max_diameter_m) || m.rock_cover_coefficient == 0.0 || !(d > 0.0)) return cover;
    // Band [d, 2 d] clipped to the law, in x = log2(D / d): dC/dx = coef q ln2 d^q e^(k x), k = q ln2, weight 1 - x.
    const double x0 = std::max(0.0, std::log2(m.rock_min_diameter_m / d));
    const double x1 = std::min(1.0, std::log2(m.rock_max_diameter_m / d));
    if (x1 <= x0) return cover;
    const double q = m.rock_cover_exponent;
    const double ln2 = std::log(2.0);
    const double k = q * ln2;
    // Integral of (1 - x) e^(k x) over [x0, x1].
    double integral;
    if (std::abs(k) < 1e-9) {
        integral = (x1 - 0.5 * x1 * x1) - (x0 - 0.5 * x0 * x0);
    } else {
        const auto antiderivative = [k](double x) { return std::exp(k * x) * ((1.0 - x) / k + 1.0 / (k * k)); };
        integral = antiderivative(x1) - antiderivative(x0);
    }
    cover += m.rock_cover_coefficient * q * ln2 * std::pow(d, q) * integral;
    return cover;
}

namespace {

struct Point2 {
    double x, y;
};

double cross2(const Point2& o, const Point2& a, const Point2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

// Convex hull (Andrew's monotone chain), counter-clockwise, no collinear points.
std::vector<Point2> convex_hull(std::vector<Point2> p) {
    std::sort(p.begin(), p.end(), [](const Point2& a, const Point2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    if (p.size() < 3) return p;
    std::vector<Point2> h(2 * p.size());
    size_t k = 0;
    for (size_t i = 0; i < p.size(); ++i) {
        while (k >= 2 && cross2(h[k - 2], h[k - 1], p[i]) <= 0.0) --k;
        h[k++] = p[i];
    }
    for (size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && cross2(h[k - 2], h[k - 1], p[i - 1]) <= 0.0) --k;
        h[k++] = p[i - 1];
    }
    h.resize(k - 1);
    return h;
}

double polygon_area(const std::vector<Point2>& p) {
    double a = 0.0;
    for (size_t i = 0; i < p.size(); ++i) {
        const Point2& u = p[i];
        const Point2& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return 0.5 * std::abs(a);
}

// Area of the intersection of two convex counter-clockwise polygons (Sutherland-Hodgman).
double intersection_area(const std::vector<Point2>& subject, const std::vector<Point2>& clip) {
    std::vector<Point2> out = subject, in;
    for (size_t i = 0; i < clip.size() && !out.empty(); ++i) {
        const Point2& a = clip[i];
        const Point2& b = clip[(i + 1) % clip.size()];
        in.swap(out);
        out.clear();
        for (size_t j = 0; j < in.size(); ++j) {
            const Point2& p = in[j];
            const Point2& q = in[(j + 1) % in.size()];
            const double dp = cross2(a, b, p), dq = cross2(a, b, q);
            if (dp >= 0.0) out.push_back(p);
            if ((dp >= 0.0) != (dq >= 0.0)) {
                const double t = dp / (dp - dq);
                out.push_back({p.x + t * (q.x - p.x), p.y + t * (q.y - p.y)});
            }
        }
    }
    return out.size() < 3 ? 0.0 : polygon_area(out);
}

struct ExposedFace {
    math::Vec3 normal;
    double area;
};

// The template mesh's part above z = cut: its points (heights above the cut) and faces.
void exposed_part(const TemplateMesh& mesh, double cut, std::vector<math::Vec3>& points,
                  std::vector<ExposedFace>& faces) {
    for (const math::Vec3& v : mesh.vertices) {
        if (v.z > cut) points.push_back({v.x, v.y, v.z - cut});
    }
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const math::Vec3 tri[3] = {mesh.vertices[static_cast<size_t>(mesh.indices[t])],
                                   mesh.vertices[static_cast<size_t>(mesh.indices[t + 1])],
                                   mesh.vertices[static_cast<size_t>(mesh.indices[t + 2])]};
        // Clip the triangle to z >= cut (polygon of up to 4 vertices).
        std::vector<math::Vec3> poly;
        for (int i = 0; i < 3; ++i) {
            const math::Vec3& p = tri[i];
            const math::Vec3& q = tri[(i + 1) % 3];
            const double dp = p.z - cut, dq = q.z - cut;
            if (dp >= 0.0) poly.push_back(p);
            if ((dp >= 0.0) != (dq >= 0.0)) {
                const double s = dp / (dp - dq);
                const math::Vec3 x = p + (q - p) * s;
                poly.push_back(x);
                points.push_back({x.x, x.y, 0.0});
            }
        }
        if (poly.size() < 3) continue;
        math::Vec3 n{0.0, 0.0, 0.0};
        for (size_t i = 1; i + 1 < poly.size(); ++i) n = n + math::cross(poly[i] - poly[0], poly[i + 1] - poly[0]);
        const double twice = std::sqrt(math::dot(n, n));
        if (twice <= 0.0) continue;
        faces.push_back({n / twice, 0.5 * twice});
    }
}

// The ground region a convex body hides from direction (elevation e, azimuth psi of the horizontal direction toward
// the light / eye): its points pushed away from that direction by height x cot e, the convex hull.
std::vector<Point2> projection(const std::vector<math::Vec3>& points, double cot_e, double cos_psi, double sin_psi) {
    std::vector<Point2> p;
    p.reserve(points.size());
    for (const math::Vec3& v : points) p.push_back({v.x - v.z * cot_e * cos_psi, v.y - v.z * cot_e * sin_psi});
    return convex_hull(std::move(p));
}

double view_elevation_at(int kv) {
    return std::clamp(kv * UnresolvedShadingModel::view_elevation_step_rad, lit_gain_floor_rad, 0.5 * pi);
}
double light_elevation_at(int k) {
    return std::clamp(k * UnresolvedShadingModel::lit_gain_step_rad, lit_gain_floor_rad, 0.5 * pi);
}

constexpr size_t view_table_size = static_cast<size_t>(UnresolvedShadingModel::view_elevation_samples) *
                                   UnresolvedShadingModel::view_azimuth_samples * UnresolvedShadingModel::lit_gain_samples;

size_t view_index(int kv, int ka, int k) {
    return (static_cast<size_t>(kv) * UnresolvedShadingModel::view_azimuth_samples + static_cast<size_t>(ka)) *
               UnresolvedShadingModel::lit_gain_samples + static_cast<size_t>(k);
}

double trilinear_view(const std::vector<double>& t, double light_elevation_rad, double view_elevation_rad,
                      double azimuth_difference_rad, double nadir_value) {
    if (t.empty()) return nadir_value;
    using M = UnresolvedShadingModel;
    const double xl = std::clamp(light_elevation_rad / M::lit_gain_step_rad, 0.0, M::lit_gain_samples - 1.0);
    const double xv = std::clamp(view_elevation_rad / M::view_elevation_step_rad, 0.0, M::view_elevation_samples - 1.0);
    double a = std::fmod(std::abs(azimuth_difference_rad), 2.0 * pi);
    if (a > pi) a = 2.0 * pi - a;
    const double xa = std::clamp(a / M::view_azimuth_step_rad, 0.0, M::view_azimuth_samples - 1.0);
    const int kl = std::min(static_cast<int>(xl), M::lit_gain_samples - 2);
    const int kv = std::min(static_cast<int>(xv), M::view_elevation_samples - 2);
    const int ka = std::min(static_cast<int>(xa), M::view_azimuth_samples - 2);
    const double fl = xl - kl, fv = xv - kv, fa = xa - ka;
    double r = 0.0;
    for (int dv = 0; dv < 2; ++dv) {
        for (int da = 0; da < 2; ++da) {
            const double w = (dv ? fv : 1.0 - fv) * (da ? fa : 1.0 - fa);
            r += w * ((1.0 - fl) * t[view_index(kv + dv, ka + da, kl)] + fl * t[view_index(kv + dv, ka + da, kl + 1)]);
        }
    }
    return r;
}

}  // namespace

RockViewTables rock_view_tables(const TemplateMesh& mesh, double cut) {
    using M = UnresolvedShadingModel;
    RockViewTables out;
    out.excess.assign(view_table_size, 0.0);
    out.flux.assign(view_table_size, 0.0);
    out.hidden.assign(view_table_size, 0.0);
    out.flux_nadir.assign(M::lit_gain_samples, 0.0);
    std::vector<math::Vec3> points;
    std::vector<ExposedFace> faces;
    exposed_part(mesh, cut, points, faces);
    if (points.size() < 3) return out;
    const double plan_area = polygon_area(projection(points, 0.0, 1.0, 0.0));
    if (!(plan_area > 0.0)) return out;
    struct SeenFace {
        double weight, mu, a, b;
    };
    std::vector<SeenFace> seen;
    for (int yaw = 0; yaw < rock_view_yaws; ++yaw) {
        const double psi = 2.0 * pi * (yaw + 0.5) / rock_view_yaws;  // the light's azimuth
        std::vector<std::vector<Point2>> light_regions(M::lit_gain_samples);
        for (int k = 0; k < M::lit_gain_samples; ++k) {
            const double e = light_elevation_at(k);
            light_regions[static_cast<size_t>(k)] = projection(points, std::cos(e) / std::sin(e), std::cos(psi), std::sin(psi));
        }
        for (int kv = 0; kv < M::view_elevation_samples; ++kv) {
            const double ev = view_elevation_at(kv);
            const double sev = std::sin(ev), cev = std::cos(ev);
            for (int ka = 0; ka < M::view_azimuth_samples; ++ka) {
                const double phi = psi + ka * M::view_azimuth_step_rad;  // the eye's azimuth
                const std::vector<Point2> view_region = projection(points, cev / sev, std::cos(phi), std::sin(phi));
                const double view_area = polygon_area(view_region);
                const math::Vec3 v{cev * std::cos(phi), cev * std::sin(phi), sev};
                // The faces the eye sees: projected area, mu, and the light's cosine as ce x a + se x b.
                seen.clear();
                double den = 0.0;
                for (const ExposedFace& f : faces) {
                    const double mu = math::dot(f.normal, v);
                    if (mu <= 0.0) continue;
                    den += f.area * mu;
                    seen.push_back({f.area * mu, mu, f.normal.x * std::cos(psi) + f.normal.y * std::sin(psi), f.normal.z});
                }
                for (int k = 0; k < M::lit_gain_samples; ++k) {
                    const double e = light_elevation_at(k);
                    const double se = std::sin(e), ce = std::cos(e);
                    const size_t i = view_index(kv, ka, k);
                    out.excess[i] += (view_area - intersection_area(view_region, light_regions[static_cast<size_t>(k)])) /
                                     plan_area;
                    out.hidden[i] += view_area / plan_area - 1.0;
                    // Seen radiance: Lommel-Seeliger per seen face, projected-area weighted, over the flat ground's.
                    double num = 0.0;
                    for (const SeenFace& f : seen) {
                        const double mu0 = ce * f.a + se * f.b;
                        if (mu0 > 0.0) num += f.weight * mu0 / (mu0 + f.mu);
                    }
                    const double flat = se / (se + sev);
                    if (den > 0.0) out.flux[i] += num / den / flat * (view_area / plan_area);
                }
            }
        }
    }
    for (size_t i = 0; i < view_table_size; ++i) {
        out.excess[i] /= rock_view_yaws;
        out.hidden[i] /= rock_view_yaws;
        out.flux[i] /= rock_view_yaws;
    }
    for (int k = 0; k < M::lit_gain_samples; ++k) {
        out.flux_nadir[static_cast<size_t>(k)] = out.flux[view_index(M::view_elevation_samples - 1, 0, k)];
    }
    return out;
}

RockViewModel template_rock_view_tables(double burial_mean, double burial_sd) {
    static std::mutex mutex;
    static std::map<std::pair<double, double>, RockViewModel> cache;
    const std::lock_guard<std::mutex> lock(mutex);
    if (const auto it = cache.find({burial_mean, burial_sd}); it != cache.end()) return it->second;
    using M = UnresolvedShadingModel;
    constexpr int burial_nodes = 5;  // [D] N in [-2, 2] step 1 (the tails beyond weigh < 3 %: the view terms vary slowly with the burial)
    constexpr int lod = 2;           // [D] 162 vertices: the polygons' areas within ~1 % of LOD 1 (test)
    const RockTemplates& templates = RockTemplates::instance();
    struct Sum {
        std::vector<double> excess, flux, hidden_plus_one, flux_nadir;
        double weights = 0.0;
    };
    std::vector<Sum> sums(RockTemplates::count);
    std::vector<std::thread> workers;
    for (int id = 0; id < RockTemplates::count; ++id) {
        workers.emplace_back([&, id] {
            Sum& s = sums[static_cast<size_t>(id)];
            s.excess.assign(view_table_size, 0.0);
            s.flux.assign(view_table_size, 0.0);
            s.hidden_plus_one.assign(view_table_size, 0.0);
            s.flux_nadir.assign(M::lit_gain_samples, 0.0);
            for (int q = 0; q < burial_nodes; ++q) {
                const double z = q - 0.5 * (burial_nodes - 1);
                const double weight = std::exp(-0.5 * z * z);
                const double f = std::clamp(burial_mean + burial_sd * z, 0.0, LunarRocks::max_burial_fraction);
                const double cut = templates.bottom_z(id) + f * (templates.top_z(id) - templates.bottom_z(id));
                const RockViewTables one = rock_view_tables(templates.mesh(id, lod), cut);
                for (size_t i = 0; i < view_table_size; ++i) {
                    s.excess[i] += weight * one.excess[i];
                    s.flux[i] += weight * one.flux[i];
                    s.hidden_plus_one[i] += weight * (1.0 + one.hidden[i]);
                }
                for (int k = 0; k < M::lit_gain_samples; ++k) {
                    s.flux_nadir[static_cast<size_t>(k)] += weight * one.flux_nadir[static_cast<size_t>(k)];
                }
                s.weights += weight;
            }
        });
    }
    for (std::thread& w : workers) w.join();
    std::vector<double> excess(view_table_size, 0.0), flux(view_table_size, 0.0), hidden(view_table_size, 0.0),
        flux_nadir(M::lit_gain_samples, 0.0);
    double weights = 0.0;
    for (const Sum& s : sums) {
        for (size_t i = 0; i < view_table_size; ++i) {
            excess[i] += s.excess[i];
            flux[i] += s.flux[i];
            hidden[i] += s.hidden_plus_one[i];
        }
        for (size_t k = 0; k < flux_nadir.size(); ++k) flux_nadir[k] += s.flux_nadir[k];
        weights += s.weights;
    }
    RockViewModel out;
    out.excess.resize(view_table_size);
    out.gain_ratio.resize(view_table_size);
    for (int kv = 0; kv < M::view_elevation_samples; ++kv) {
        for (int ka = 0; ka < M::view_azimuth_samples; ++ka) {
            for (int k = 0; k < M::lit_gain_samples; ++k) {
                const size_t i = view_index(kv, ka, k);
                const bool zenith = kv == M::view_elevation_samples - 1;  // exact: no excess, the zenith gain
                out.excess[i] = zenith ? 0.0 : excess[i] / weights;
                const double nadir = flux_nadir[static_cast<size_t>(k)];
                out.gain_ratio[i] = zenith || !(nadir > 0.0) ? 1.0 : (flux[i] / hidden[i]) / (nadir / weights);
            }
        }
    }
    cache.emplace(std::make_pair(burial_mean, burial_sd), out);
    return out;
}

double rock_view_excess_table(const UnresolvedShadingModel& m, double light_elevation_rad, double view_elevation_rad,
                              double azimuth_difference_rad) {
    return trilinear_view(m.rock_view_excess, light_elevation_rad, view_elevation_rad, azimuth_difference_rad, 0.0);
}

double rock_view_gain_ratio_table(const UnresolvedShadingModel& m, double light_elevation_rad,
                                  double view_elevation_rad, double azimuth_difference_rad) {
    return trilinear_view(m.rock_view_gain_ratio, light_elevation_rad, view_elevation_rad, azimuth_difference_rad, 1.0);
}

double hapke_hemispherical_albedo(const config::HapkeParameters& p, double mu0, double theta_bar_rad) {
    if (!(mu0 > 0.0)) return 0.0;
    mu0 = std::min(mu0, 1.0);
    // [D] 48-node Gauss-Legendre in mu (on [0, 1]) x 96 azimuths: < 1e-4 against twice the nodes.
    constexpr int n_mu = 48, n_phi = 96;
    static const std::pair<std::vector<double>, std::vector<double>> gl = [] {
        std::vector<double> x(n_mu), w(n_mu);
        for (int i = 0; i < n_mu; ++i) {
            double z = std::cos(pi * (i + 0.75) / (n_mu + 0.5));
            for (int it = 0; it < 100; ++it) {
                double p0 = 1.0, p1 = z;
                for (int k = 2; k <= n_mu; ++k) {
                    const double p2 = ((2.0 * k - 1.0) * z * p1 - (k - 1.0) * p0) / k;
                    p0 = p1;
                    p1 = p2;
                }
                const double dp = n_mu * (z * p1 - p0) / (z * z - 1.0);
                const double dz = p1 / dp;
                z -= dz;
                if (std::abs(dz) < 1e-15) break;
            }
            double p0 = 1.0, p1 = z;
            for (int k = 2; k <= n_mu; ++k) {
                const double p2 = ((2.0 * k - 1.0) * z * p1 - (k - 1.0) * p0) / k;
                p0 = p1;
                p1 = p2;
            }
            const double dp = n_mu * (z * p1 - p0) / (z * z - 1.0);
            x[static_cast<size_t>(i)] = 0.5 * (1.0 + z);
            w[static_cast<size_t>(i)] = 1.0 / ((1.0 - z * z) * dp * dp);  // 2 / (...) on [-1, 1], halved on [0, 1]
        }
        return std::make_pair(x, w);
    }();
    const double sin_i = std::sqrt(std::max(0.0, 1.0 - mu0 * mu0));
    double sum = 0.0;
    for (int i = 0; i < n_mu; ++i) {
        const double mu = gl.first[static_cast<size_t>(i)];
        const double sin_e = std::sqrt(std::max(0.0, 1.0 - mu * mu));
        double ring = 0.0;
        for (int j = 0; j < n_phi; ++j) {
            const double phi = (j + 0.5) / n_phi * 2.0 * pi;
            const double cos_g = std::clamp(mu0 * mu + sin_i * sin_e * std::cos(phi), -1.0, 1.0);
            ring += hapke_reflectance(p, mu0, mu, std::acos(cos_g), theta_bar_rad);
        }
        sum += gl.second[static_cast<size_t>(i)] * mu * ring * (2.0 * pi / n_phi);
    }
    return sum / (pi * mu0);
}

double hemispherical_albedo_table(const UnresolvedShadingModel& m, double elevation_rad) {
    if (m.hemispherical_albedo.empty()) return 0.0;
    const double x = std::clamp(elevation_rad / UnresolvedShadingModel::lit_gain_step_rad, 0.0,
                                static_cast<double>(UnresolvedShadingModel::lit_gain_samples - 1));
    const int k = std::min(static_cast<int>(x), UnresolvedShadingModel::lit_gain_samples - 2);
    const double f = x - k;
    return m.hemispherical_albedo[static_cast<size_t>(k)] * (1.0 - f) +
           m.hemispherical_albedo[static_cast<size_t>(k) + 1] * f;
}

UnresolvedLightFactors unresolved_light_factors(const UnresolvedShadingModel& m, double abundance,
                                                double geometry_min_diameter_m, double spacing_m,
                                                double horizon_spacing_m, double sun_elevation_rad,
                                                double earth_elevation_rad, bool geometry_shadows_statistical) {
    return unresolved_light_factors(m, abundance, geometry_min_diameter_m, spacing_m, horizon_spacing_m,
                                    sun_elevation_rad, earth_elevation_rad, geometry_shadows_statistical,
                                    UnresolvedView{});
}

UnresolvedLightFactors unresolved_light_factors(const UnresolvedShadingModel& m, double abundance,
                                                double geometry_min_diameter_m, double spacing_m,
                                                double horizon_spacing_m, double sun_elevation_rad,
                                                double earth_elevation_rad, bool geometry_shadows_statistical,
                                                const UnresolvedView& view) {
    UnresolvedLightFactors out;
    const double rho = m.rock_reflectance_ratio;
    const double c_all = abundance * rock_cover(m, m.rock_min_diameter_m, m.rock_max_diameter_m);
    const double c = abundance * rock_statistical_cover(m, geometry_min_diameter_m);
    const double c_drawn = geometry_shadows_statistical ? std::max(c_all - c, 0.0) : 0.0;
    const double norm = std::exp(-c_all) + (1.0 - std::exp(-c_all)) * rho;
    const bool zenith = !(view.elevation_rad < 0.5 * pi);
    const double ev = std::max(view.elevation_rad, lit_gain_floor_rad);
    const double sigma_view = zenith ? 0.0 : rock_shadow_table(m, ev);
    const double rock_seen = 1.0 - std::exp(-c * (1.0 + sigma_view));
    const double unshadowed = unshadowed_slope_variance(m, spacing_m, horizon_spacing_m);
    const double unresolved_by_map = unshadowed_slope_variance(m, spacing_m, view.shadow_map_resolved_m);
    // With the near-field map the drawn rocks below shadow_caster_factor x d cast no map shadow: theirs stay statistical.
    const double c_uncast =
        geometry_shadows_statistical
            ? std::max(abundance * rock_statistical_cover(m, shadow_caster_factor * geometry_min_diameter_m) - c, 0.0)
            : 0.0;
    // One directional light (the Sun, or the Earth's disc as the earthshine's source) at macroscopic elevation e.
    const auto light = [&](double elevation_rad, double azimuth_difference_rad, bool shadow_map) {
        const double se = std::sin(elevation_rad);
        if (se <= 0.0) return 0.0;
        const double variance = shadow_map ? unresolved_by_map : unshadowed;
        const double lit = zenith ? smith_lit_fraction(std::tan(elevation_rad), variance)
                                  : smith_visible_lit_fraction(std::tan(elevation_rad), std::tan(ev),
                                                               azimuth_difference_rad, variance);
        const double sigma = rock_shadow_table(m, elevation_rad);
        const double excess = zenith ? 0.0 : rock_view_excess_table(m, elevation_rad, ev, azimuth_difference_rad);
        const double ratio = zenith ? 1.0 : rock_view_gain_ratio_table(m, elevation_rad, ev, azimuth_difference_rad);
        const double drawn_visible_shadow = std::max(sigma - sigma_view + excess, 0.0);
        return lit * (std::exp(-c * (1.0 + sigma + excess) - (shadow_map ? c_uncast : c_drawn) * drawn_visible_shadow) +
                      rock_seen * rho * rock_lit_gain_table(m, elevation_rad) * ratio) / norm;
    };
    out.sun = light(sun_elevation_rad, view.sun_azimuth_difference_rad, view.sun_shadow_map);
    out.earth = light(earth_elevation_rad, view.earth_azimuth_difference_rad, false);
    return out;
}

}  // namespace starsim::environment
