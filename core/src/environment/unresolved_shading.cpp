#include "starsim/environment/unresolved_shading.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/hapke.hpp"
#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
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
        m.rock_lit_gain.resize(UnresolvedShadingModel::lit_gain_samples);
        for (int k = 0; k < UnresolvedShadingModel::lit_gain_samples; ++k) {
            m.rock_lit_gain[static_cast<size_t>(k)] =
                rock_lit_gain(m.rock_exposed_height_per_diameter, k * UnresolvedShadingModel::lit_gain_step_rad);
        }
        // The rocks' share of the globe's mean slope (header comment): out of the regolith's budget.
        m.rock_area_share = 1.0 - std::exp(-rock_cover(m, m.rock_min_diameter_m, m.rock_max_diameter_m));
        m.rock_mean_tan = m.rock_area_share * pi * m.rock_exposed_height_per_diameter;
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

UnresolvedLightFactors unresolved_light_factors(const UnresolvedShadingModel& m, double abundance,
                                                double geometry_min_diameter_m, double spacing_m,
                                                double horizon_spacing_m, double sun_elevation_rad,
                                                bool geometry_shadows_statistical) {
    UnresolvedLightFactors out;
    const double rho = m.rock_reflectance_ratio;
    const double c_all = abundance * rock_cover(m, m.rock_min_diameter_m, m.rock_max_diameter_m);
    const double c = abundance * rock_statistical_cover(m, geometry_min_diameter_m);
    const double c_drawn = geometry_shadows_statistical ? std::max(c_all - c, 0.0) : 0.0;
    const double norm = std::exp(-c_all) + (1.0 - std::exp(-c_all)) * rho;
    const double rock_share = 1.0 - std::exp(-c);
    out.earth = (std::exp(-c) + rock_share * rho) / norm;
    const double se = std::sin(sun_elevation_rad);
    if (se <= 0.0) {
        out.sun = 0.0;
        return out;
    }
    const double lit = smith_lit_fraction(std::tan(sun_elevation_rad),
                                          unshadowed_slope_variance(m, spacing_m, horizon_spacing_m));
    const double sigma = rock_shadow_per_cover(m.rock_exposed_height_per_diameter, se);
    out.sun = lit * (std::exp(-c * (1.0 + sigma) - c_drawn * sigma) + rock_share * rho * rock_lit_gain_table(m, sun_elevation_rad)) / norm;
    return out;
}

}  // namespace starsim::environment
