// Unresolved-surface shading statistics (Task 10 of the Moon procedural surface plan): the Hapke slope budget, the
// regolith and crater slope variances against the generator, Smith's shadowing function, the rock shadow / lit-face
// terms and the light factors the Moon terrain shader mirrors.

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <thread>
#include <functional>
#include <fstream>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <memory>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/environment/hapke.hpp"
#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/surface_hash.hpp"
#include "starsim/environment/unresolved_shading.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using environment::LunarRocks;
using environment::LunarSurfaceDetail;
using environment::UnresolvedShadingModel;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double pi = constants::pi;
constexpr double R = 1737400.0;
constexpr double site_lat = -84.79 * deg, site_lon = 29.2 * deg;
const double inf = std::numeric_limits<double>::infinity();

Vec3 direction(double lat, double lon) {
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

// Flat global grid and one 20 m tile centred on the site (the fixture of the surface-detail and rock tests).
environment::PolarStereographicTile site_tile() {
    environment::PolarStereographicTile t;
    t.sphere_radius_m = R;
    t.scale_m = 20.0;
    t.pole_line = t.pole_sample = 15199.5;
    const auto [line, sample] = t.position(site_lat, site_lon);
    t.first_line = static_cast<int>(std::lround(line)) - 200;
    t.first_sample = static_cast<int>(std::lround(sample)) - 200;
    t.grid = {401, 401, 0.5, std::vector<std::int16_t>(401 * 401, 0)};
    return t;
}

std::shared_ptr<const LunarSurfaceDetail> shared_detail() {
    static const auto d = [] {
        environment::CylindricalGrid g;
        g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, 0)};
        g.pixels_per_degree = 16.0;
        g.zero_line = 1439.5;
        g.zero_sample = -0.5;
        auto terrain = std::make_shared<const environment::LunarTerrain>(std::move(g), std::vector{site_tile()}, R);
        auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
        cfg.rocks.abundance_source = "uniform";  // the Diviner map is not installed in the test environment
        return std::make_shared<const LunarSurfaceDetail>(cfg, terrain);
    }();
    return d;
}

const LunarRocks& shared_rocks() {
    static const LunarRocks r(shared_detail());
    return r;
}

const UnresolvedShadingModel& shared_model() {
    static const UnresolvedShadingModel m = environment::make_unresolved_shading_model(*shared_detail(), &shared_rocks());
    return m;
}

struct Tangent {
    Vec3 c, e1, e2;
};
Tangent tangent(const Vec3& c) {
    const Vec3 e1 = math::normalized(cross(std::abs(c.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, c));
    return {c, e1, cross(c, e1)};
}
Vec3 at(const Tangent& t, double x, double y) { return math::normalized(t.c + (t.e1 * x + t.e2 * y) / R); }

double unit_hash(std::uint64_t a, std::uint64_t b) {
    return environment::hash_unit(environment::surface_hash(0x7A10, a, b, 0));
}

}  // namespace

TEST_CASE("Hapke slope budget: theta_bar <-> mean square slope, residual by mesh spacing",
          "[moon_surface][unresolved_shading]") {
    const auto& m = shared_model();
    const double theta = shared_detail()->config().hapke.theta_bar_rad;
    CHECK(std::abs(environment::theta_bar_from_slope_variance(environment::slope_variance_from_theta_bar(0.3)) - 0.3) <
          1e-14);
    CHECK(std::abs(m.globe_slope_variance - pi * std::tan(theta) * std::tan(theta)) < 1e-14);
    // Beyond every generated band nothing is drawn: the regolith's whole budget (the globe's without the rocks).
    const double theta_regolith = environment::theta_bar_from_slope_variance(m.total_slope_variance);
    CHECK(std::abs(environment::residual_theta_bar(m, 1.0e7) - theta_regolith) < 1e-14);
    CHECK(theta_regolith < theta);
    // Monotonic and continuous in the spacing (fade continuity: no step when a node splits, d -> d / 2 continuously
    // through the morph): the largest change per 1 % spacing step.
    double previous = environment::residual_theta_bar(m, 0.01);
    double worst_step = 0.0;
    bool monotonic = true;
    for (double d = 0.01 * 1.01; d < 1.0e5; d *= 1.01) {
        const double t = environment::residual_theta_bar(m, d);
        monotonic = monotonic && t >= previous - 1e-15;
        worst_step = std::max(worst_step, std::abs(t - previous));
        previous = t;
    }
    std::printf("  residual theta_bar: 1 cm %.3f deg, 10 cm %.3f, 1 m %.3f, 10 m %.3f, 100 m %.3f, 1 km %.3f, globe %.3f;"
                " largest change per 1 %% spacing step %.2e rad\n",
                environment::residual_theta_bar(m, 0.01) / deg, environment::residual_theta_bar(m, 0.1) / deg,
                environment::residual_theta_bar(m, 1.0) / deg, environment::residual_theta_bar(m, 10.0) / deg,
                environment::residual_theta_bar(m, 100.0) / deg, environment::residual_theta_bar(m, 1000.0) / deg,
                theta / deg, worst_step);
    std::printf("  drawn slope variance of %.4f: 1 cm %.4f (regolith %.4f), 10 cm %.4f, 1 m %.4f, 10 m %.4f\n",
                m.total_slope_variance, environment::drawn_slope_variance(m, 0.01),
                environment::drawn_slope_variance_direct(m, 0.01, inf, 0.0),
                environment::drawn_slope_variance(m, 0.1), environment::drawn_slope_variance(m, 1.0),
                environment::drawn_slope_variance(m, 10.0));
    double full_regolith = 0.0;
    for (const double v : m.regolith_octave_variance) full_regolith += v;
    std::printf("  generated mean square slope: regolith octaves %.4f, craters 1 cm - 2 km %.4f; drawn share at 1 cm %.3f;"
                " Hapke residual below the generator %.4f\n", full_regolith,
                environment::drawn_slope_variance_direct(m, 0.0, 0.0, inf),
                environment::drawn_slope_variance(m, 0.01) /
                    environment::drawn_slope_variance_direct(m, 0.0, inf, inf),
                m.total_slope_variance - environment::drawn_slope_variance_direct(m, 0.0, inf, inf));
    CHECK(monotonic);
    CHECK(worst_step < 1.0e-3);
    CHECK(environment::residual_theta_bar(m, 0.01) > 0.0);  // the drawn relief never exceeds the budget
    CHECK(environment::residual_theta_bar(m, 0.01) < theta);
}

TEST_CASE("Rocks out of the slope budget: regolith and rocks add up to the globe's Hapke mean slope (no double shadow)",
          "[moon_surface][unresolved_shading]") {
    const auto& m = shared_model();
    const double theta = shared_detail()->config().hapke.theta_bar_rad;
    // Hapke's mean-tan form: (1 - A) <tan>_regolith + A <tan>_rock = <tan>_globe = (pi / 2) tan theta_bar, <tan>_rock the
    // generated templates' mean |grad h| per plan area (review n1 of the Task 11 fix C re-review; pi eta of the
    // half-spheroid before).
    const auto& cfg = shared_detail()->config();
    const double rocks_tan =
        environment::template_rock_light_tables(cfg.rocks.burial_fraction_mean, cfg.rocks.burial_fraction_sd).mean_tan;
    std::printf("  templates' mean |grad h| %.4f against the half-spheroid's pi eta %.4f\n", rocks_tan,
                pi * m.rock_exposed_height_per_diameter);
    CHECK(rocks_tan > 0.3);
    CHECK(rocks_tan < 3.0);
    const double regolith_tan = 0.5 * pi * std::tan(environment::theta_bar_from_slope_variance(m.total_slope_variance));
    CHECK(std::abs((1.0 - m.rock_area_share) * regolith_tan + m.rock_area_share * rocks_tan -
                   0.5 * pi * std::tan(theta)) < 1e-12);
    CHECK(std::abs(m.rock_area_share - (1.0 - std::exp(-environment::rock_cover(m, 0.0, inf)))) < 1e-15);
    CHECK(m.rock_area_share > 0.0);
    // Without the rocks nothing comes out: the regolith keeps the globe's budget.
    const auto bare = environment::make_unresolved_shading_model(*shared_detail(), nullptr);
    CHECK(bare.total_slope_variance == bare.globe_slope_variance);
    CHECK(bare.rock_mean_tan == 0.0);
    // The half-spheroid's mean |grad h| over its plan area is pi eta (closed form against a midpoint sum).
    const double eta = m.rock_exposed_height_per_diameter, a = 0.5, height = eta;  // D = 1
    constexpr int n = 200000;
    double sum = 0.0;
    for (int k = 0; k < n; ++k) {
        const double r = (k + 0.5) / n * a;
        sum += height * r / (a * a * std::sqrt(1.0 - r * r / (a * a))) * 2.0 * pi * r * (a / n);
    }
    CHECK(std::abs(sum / (pi * a * a) / (pi * eta) - 1.0) < 2e-3);  // the rim's integrable singularity limits the sum
    std::printf("  rocks out of the budget: area share %.4f, mean tan %.4f of the globe's %.4f; theta_bar globe %.3f deg,"
                " regolith %.3f deg; S^2 %.4f -> %.4f\n", m.rock_area_share, m.rock_mean_tan,
                0.5 * pi * std::tan(theta), theta / deg,
                environment::theta_bar_from_slope_variance(m.total_slope_variance) / deg, m.globe_slope_variance,
                m.total_slope_variance);
}

TEST_CASE("Crater slope variance is the profile's gradient integral over the size law",
          "[moon_surface][unresolved_shading]") {
    const auto& cfg = shared_detail()->config().craters;
    // Independent: numeric integral of (dh/dr)^2 2 pi r dr over crater_profile_m (central differences), D = 1 m.
    const auto integral = [](double depth, double rim) {
        environment::Crater c;
        c.diameter_m = 1.0;
        c.depth_m = depth;
        c.rim_height_m = rim;
        constexpr int n = 200000;
        const double r_max = 1.5, dr = r_max / n, h = 1e-7;
        double sum = 0.0;
        for (int k = 0; k < n; ++k) {
            const double r = (k + 0.5) * dr;
            const double g = (environment::crater_profile_m(c, r + h) - environment::crater_profile_m(c, r - h)) / (2 * h);
            sum += g * g * 2.0 * pi * r * dr;
        }
        return sum;
    };
    const double rim_per_depth = cfg.rim_height_to_diameter / cfg.depth_to_diameter_fresh;
    const double fresh = integral(cfg.depth_to_diameter_fresh, cfg.rim_height_to_diameter);
    const double degraded = integral(cfg.depth_to_diameter_degraded, rim_per_depth * cfg.depth_to_diameter_degraded);
    const double mix = cfg.degraded_fraction * degraded + (1.0 - cfg.degraded_fraction) * fresh;
    const double expected = cfg.sfd.exponent * cfg.sfd.density_per_m2 * std::pow(cfg.sfd.reference_diameter_m, cfg.sfd.exponent) * mix;
    const auto& m = shared_model();
    std::printf("  crater gradient integral / D^2: fresh %.5f degraded %.5f; variance per ln D %.5f (model %.5f)\n", fresh,
                degraded, expected, m.crater_variance_coefficient);
    CHECK(std::abs(m.crater_variance_coefficient / expected - 1.0) < 1e-4);
}

TEST_CASE("Regolith slope variance matches the generator's roughness field", "[moon_surface][unresolved_shading]") {
    const auto& d = *shared_detail();
    const auto& m = shared_model();
    // Gradient of the full-detail roughness by central differences (0.2 mm, well below the 5 mm finest octave) at
    // hashed points of the site's tile core (band start 20 m, octaves fade out above band_start / 2).
    const Tangent t = tangent(direction(site_lat, site_lon));
    constexpr int points = 4000;
    constexpr double h = 2.0e-4;
    double sum = 0.0;
    for (int k = 0; k < points; ++k) {
        const double x = (unit_hash(k, 1) - 0.5) * 2000.0, y = (unit_hash(k, 2) - 0.5) * 2000.0;
        const double gx = (d.roughness_relief_m(at(t, x + h, y), 0.0) - d.roughness_relief_m(at(t, x - h, y), 0.0)) / (2 * h);
        const double gy = (d.roughness_relief_m(at(t, x, y + h), 0.0) - d.roughness_relief_m(at(t, x, y - h), 0.0)) / (2 * h);
        sum += gx * gx + gy * gy;
    }
    const double measured = sum / points;
    // Model: the octaves up to the site's band start (the faded top octaves carry < 1 % of the slope variance).
    const double model = environment::drawn_slope_variance_direct(m, 0.0, d.band_start_m(t.c), 0.0);
    // Sampling error of the mean of |grad h|^2 over 4000 points (chi-square with 2 dof per point: ~1.6 %, the octave
    // fields add more correlation): 4 sigma ~ 7 %.
    std::printf("  regolith mean square slope at the site: generator %.4f, model %.4f (%+.1f %%)\n", measured, model,
                100.0 * (model / measured - 1.0));
    CHECK(std::abs(model / measured - 1.0) < 0.07);
}

TEST_CASE("Slope variance the mesh draws: model against the band-limited relief", "[moon_surface][unresolved_shading]") {
    const auto& d = *shared_detail();
    const auto& m = shared_model();
    // A node of vertex spacing L draws relief_m(unit, L) at its vertices; its normals are central differences over 2 L.
    // Independent points scattered over 4 km x 4 km of the tile core (each a vertex and its four neighbours), so the
    // coarse octaves and the rare large craters enter with their share; the roughness (octaves up to the band start,
    // 20 m) and the craters (up to 5 x the band start) separately.
    const Tangent t = tangent(direction(site_lat, site_lon));
    const double band = d.band_start_m(t.c);
    const auto mesh_slope = [&](double spacing, bool craters) {
        const auto relief = [&](double x, double y) {
            const Vec3 u = at(t, x, y);
            return craters ? d.crater_relief_m(u, spacing) : d.roughness_relief_m(u, spacing);
        };
        constexpr int points = 20000;
        double sum = 0.0;
        for (int k = 0; k < points; ++k) {
            const double x = (unit_hash(k, 21) - 0.5) * 4000.0, y = (unit_hash(k, 22) - 0.5) * 4000.0;
            const double gx = (relief(x + spacing, y) - relief(x - spacing, y)) / (2 * spacing);
            const double gy = (relief(x, y + spacing) - relief(x, y - spacing)) / (2 * spacing);
            sum += gx * gx + gy * gy;
        }
        return sum / points;
    };
    for (const double spacing : {0.02, 0.08, 0.32, 1.28}) {
        const double rough = mesh_slope(spacing, false);
        const double crater = mesh_slope(spacing, true);
        const double model_rough = environment::drawn_slope_variance_direct(m, spacing, band, 0.0);
        const double model_crater = environment::drawn_slope_variance_direct(m, spacing, 0.0, 5.0 * band);
        std::printf("  spacing %.2f m: mesh normals' mean square slope roughness %.5f (model %.5f, ratio %.2f), craters "
                    "%.5f (model %.5f, ratio %.2f)\n", spacing, rough, model_rough, model_rough / rough, crater,
                    model_crater, model_crater / crater);
        // [D] Roughness: the spectral transfer is exact for the octave spectrum; the generator's top octaves fade out
        // over band_start / 2 .. band_start (the model counts them fully: a few % at the coarse spacings); 20000 points
        // (chi-square, 2 dof each) ~1 %. Craters: heavy-tailed (slopes concentrate in the bowls and rims), 20000
        // points ~10 %, plus the midpoint box mean and the D / L table. Gates 10 % and 20 %.
        CHECK(std::abs(model_rough / rough - 1.0) < 0.10);
        CHECK(std::abs(model_crater / crater - 1.0) < 0.20);
    }
}

TEST_CASE("Smith shadowing function against its defining integral", "[moon_surface][unresolved_shading]") {
    // Lambda = (1 / nu) int_nu^inf (q - nu) p(q) dq, p Gaussian with the per-axis variance S^2 / 2, nu = tan e.
    for (const double variance : {0.05, 0.2, 0.6}) {
        for (const double e_deg : {2.0, 5.0, 15.0, 45.0, 80.0}) {
            const double nu = std::tan(e_deg * deg), sigma = std::sqrt(0.5 * variance);
            constexpr int n = 200000;
            const double q_max = nu + 12.0 * sigma, dq = (q_max - nu) / n;
            double integral = 0.0;
            for (int k = 0; k < n; ++k) {
                const double q = nu + (k + 0.5) * dq;
                integral += (q - nu) * std::exp(-q * q / (2 * sigma * sigma)) / (sigma * std::sqrt(2 * pi)) * dq;
            }
            const double g1 = 1.0 / (1.0 + integral / nu);
            CHECK(std::abs(environment::smith_lit_fraction(nu, variance) - g1) < 1e-6);
        }
    }
    CHECK(environment::smith_lit_fraction(0.0, 0.2) == 0.0);
    CHECK(environment::smith_lit_fraction(-0.1, 0.2) == 0.0);
    CHECK(environment::smith_lit_fraction(0.1, 0.0) == 1.0);
}

TEST_CASE("Rock cast shadow: closed form against a ray-cast projection", "[moon_surface][unresolved_shading]") {
    for (const double eta : {0.15, 0.35, 0.6}) {
        for (const double e_deg : {3.0, 10.0, 30.0, 70.0}) {
            const double e = e_deg * deg, h = 2.0 * eta;  // plan radius 1
            // Ground points (x, y) outside the unit disc whose ray to the Sun (+x, elevation e) meets the upper
            // half-spheroid x^2 + y^2 + (z / h)^2 <= 1, z >= 0: the ray (x + t cos e, y, t sin e), t >= 0.
            const double extent = std::sqrt(1.0 + h * h / std::tan(e) / std::tan(e)) + 0.01;
            constexpr int n = 1500;
            int shadow = 0;
            for (int iy = 0; iy < n; ++iy) {
                const double y = -1.0 + (iy + 0.5) * 2.0 / n;
                for (int ix = 0; ix < n; ++ix) {
                    const double x = -extent + (ix + 0.5) * extent / n;  // the anti-sun side only
                    if (x * x + y * y <= 1.0) continue;
                    // (x + t c)^2 + y^2 + (t s / h)^2 <= 1 for some t >= 0: quadratic in t, min over t >= 0.
                    const double c = std::cos(e), s = std::sin(e) / h;
                    const double tmin = std::max(0.0, -x * c / (c * c + s * s));
                    const double v = (x + tmin * c) * (x + tmin * c) + y * y + (tmin * s) * (tmin * s);
                    if (v <= 1.0) ++shadow;
                }
            }
            const double area = shadow * (2.0 / n) * (extent / n);
            const double sigma = area / pi;
            const double formula = environment::rock_shadow_per_cover(eta, std::sin(e));
            CHECK(std::abs(sigma / formula - 1.0) < 0.01);
        }
    }
}

TEST_CASE("Rock lit-face gain: limits and growth toward a low Sun", "[moon_surface][unresolved_shading]") {
    // Sun and eye at the zenith: every lit facet has mu0 = mu, the Lommel-Seeliger value 1/2 like the flat ground.
    CHECK(std::abs(environment::rock_lit_gain(0.35, 0.5 * pi) - 1.0) < 2e-3);
    // A flat disc (eta 0) is the ground at every elevation.
    CHECK(std::abs(environment::rock_lit_gain(0.0, 5.0 * deg) - 1.0) < 1e-5);  // midpoint rule over the disc
    const double low = environment::rock_lit_gain(0.35, 5.0 * deg), mid = environment::rock_lit_gain(0.35, 20.0 * deg);
    std::printf("  rock lit gain (eta 0.35): 2 deg %.2f, 5 deg %.2f, 10 deg %.2f, 20 deg %.2f, 45 deg %.2f\n",
                environment::rock_lit_gain(0.35, 2.0 * deg), low, environment::rock_lit_gain(0.35, 10.0 * deg), mid,
                environment::rock_lit_gain(0.35, 45.0 * deg));
    CHECK(low > mid);
    CHECK(mid > 1.0);
    // Below the floor the floor's value.
    CHECK(environment::rock_lit_gain(0.35, 0.1 * deg) == environment::rock_lit_gain(0.35, environment::lit_gain_floor_rad));
}

TEST_CASE("Rock light tables from columns: the half-spheroid's exact laws", "[moon_surface][unresolved_shading]") {
    // A half-spheroid of plan radius 1 and height h = 2 eta on the column grid: G(e) and sigma(e) of rock_light_tables
    // against rock_lit_gain / rock_shadow_per_cover (independent integrals of the same quantities).
    for (const double eta : {0.2, 0.35, 0.5}) {
        const double h = 2.0 * eta;
        for (const int grid : {64, 128}) {
            const double cell = 2.0 / grid;
            std::vector<environment::RockColumn> columns;
            for (int i = 0; i < grid; ++i) {
                for (int j = 0; j < grid; ++j) {
                    const double x = -1.0 + (i + 0.5) * cell, y = -1.0 + (j + 0.5) * cell, r2 = x * x + y * y;
                    if (r2 >= 1.0) continue;
                    const double z = h * std::sqrt(1.0 - r2);
                    columns.push_back({x, y, z, math::normalized(math::Vec3{x, y, z / (h * h)})});
                }
            }
            const auto t = environment::rock_light_tables(columns, cell);
            double worst_g = 0.0, worst_s = 0.0;
            for (int k = 2; k < UnresolvedShadingModel::lit_gain_samples - 1; ++k) {  // 4 .. 88 deg
                const double e = k * UnresolvedShadingModel::lit_gain_step_rad;
                worst_g = std::max(worst_g, std::abs(t.gain[static_cast<size_t>(k)] / environment::rock_lit_gain(eta, e) - 1.0));
                const double s = environment::rock_shadow_per_cover(eta, std::sin(e));
                // The rock darkens its cover share by 1 + sigma (exp(-C (1 + sigma))): the error relative to that. The
                // grid's error is first order (~cell / 3 of shadow length per line, the rim's discretisation).
                worst_s = std::max(worst_s, std::abs(t.shadow[static_cast<size_t>(k)] - s) / (1.0 + s));
            }
            std::printf("  half-spheroid eta %.2f grid %3d: worst G error %.4f, worst sigma error / (1 + sigma) %.4f (4-88 deg)\n",
                        eta, grid, worst_g, worst_s);
            CHECK(worst_g < (grid == 64 ? 0.02 : 0.01));
            CHECK(worst_s < (grid == 64 ? 0.015 : 0.008));
            // The mean |grad h| per plan area (the rocks' share of Hapke's mean slope): pi eta for the half-spheroid; the
            // rim's integrable singularity converges slowly on the grid.
            std::printf("    mean |grad h| %.4f against pi eta %.4f\n", t.mean_tan, pi * eta);
            CHECK(std::abs(t.mean_tan / (pi * eta) - 1.0) < (grid == 64 ? 0.06 : 0.04));
        }
    }
}

TEST_CASE("Light factors: the Sun and the earthshine follow one directional-light law, each at its own elevation",
          "[moon_surface][unresolved_shading]") {
    const auto& m = shared_model();
    // The earthshine is a directional light like the Sun (Earth disc ~1.9 deg): its factor at elevation e is the Sun's
    // at e, independently of the other light's elevation; below the horizon both are 0.
    for (const double e_deg : {-5.0, 2.0, 7.0, 40.0}) {
        for (const double spacing : {0.01, 1.0, 100.0}) {
            for (const double d : {inf, 0.3}) {
                const auto f = environment::unresolved_light_factors(m, 1.0, d, spacing, 20.0, e_deg * deg, 20.0 * deg);
                const auto swapped =
                    environment::unresolved_light_factors(m, 1.0, d, spacing, 20.0, 20.0 * deg, e_deg * deg);
                CHECK(swapped.earth == f.sun);
                CHECK(swapped.sun == f.earth);
                if (e_deg < 0.0) CHECK(f.sun == 0.0);
            }
        }
    }
    // A low Earth: the rocks' cast shadows dim the earthlit ground, their lit faces brighten it; with every rock
    // statistical and no unshadowed drawn relief the factor is [exp(-C (1 + sigma)) + (1 - exp(-C)) rho G] / N.
    {
        const double e = 7.0 * deg;
        const auto f = environment::unresolved_light_factors(m, 1.0, inf, 1.0e4, 1.0e4, 40.0 * deg, e);
        const double c = environment::rock_cover(m, 0.0, inf);
        const double rho = m.rock_reflectance_ratio;
        const double norm = std::exp(-c) + (1.0 - std::exp(-c)) * rho;
        const double sigma = environment::rock_shadow_table(m, e);
        const double expected =
            (std::exp(-c * (1.0 + sigma)) + (1.0 - std::exp(-c)) * rho * environment::rock_lit_gain_table(m, e)) / norm;
        CHECK(std::abs(f.earth - expected) < 1e-12);
    }
    // No rocks at all (abundance 0) and no drawn relief below the horizon spacing: both factors 1.
    const auto bare = environment::unresolved_light_factors(m, 0.0, inf, 1.0e4, 1.0e4, 7.0 * deg, 3.0 * deg);
    CHECK(std::abs(bare.sun - 1.0) < 1e-12);
    CHECK(std::abs(bare.earth - 1.0) < 1e-12);
    for (const double e_deg : {3.0, 7.0, 15.0, 30.0, 60.0}) {
        const auto near = environment::unresolved_light_factors(m, 1.0, inf, 0.01, 20.0, e_deg * deg, e_deg * deg);
        const auto rocks = environment::unresolved_light_factors(m, 1.0, inf, 1.0e4, 1.0e4, e_deg * deg, e_deg * deg);
        std::printf("  light %4.1f deg: factor 1 cm node %.3f (Smith G1 %.3f), rocks alone %.3f\n", e_deg, near.sun,
                    environment::smith_lit_fraction(std::tan(e_deg * deg), environment::unshadowed_slope_variance(m, 0.01, 20.0)),
                    rocks.sun);
    }
}

TEST_CASE("Rock growth band: statistical cover, no rock counted twice, the drawn share closes the radiance",
          "[moon_surface][unresolved_shading]") {
    const auto& m = shared_model();
    const double c_all = environment::rock_cover(m, 0.0, inf);
    CHECK(environment::rock_statistical_cover(m, inf) == c_all);
    CHECK(environment::rock_statistical_cover(m, 0.0) == 0.0);
    // Closed form against a midpoint integral of the weight over the law (dC/dD = coef q D^(q-1)).
    for (const double d : {0.003, 0.007, 0.01, 0.0137, 0.05, 1.0, 5.0, 6.0, 9.9, 20.0}) {
        double numeric = 0.0;
        const int n = 200000;
        const double lo = std::log(m.rock_min_diameter_m), hi = std::log(m.rock_max_diameter_m);
        for (int i = 0; i < n; ++i) {
            const double diameter = std::exp(lo + (i + 0.5) / n * (hi - lo));
            const double x = std::log2(diameter / d);
            const double w = x <= 0.0 ? 1.0 : (x >= 1.0 ? 0.0 : 1.0 - x);
            numeric += w * m.rock_cover_coefficient * m.rock_cover_exponent *
                       std::pow(diameter, m.rock_cover_exponent) * (hi - lo) / n;
        }
        const double closed = environment::rock_statistical_cover(m, d);
        CHECK(std::abs(closed - numeric) < 1e-6 * c_all);
        // Between the hard cut at d and at 2 d (the band's two ends).
        CHECK(closed >= environment::rock_cover(m, 0.0, d) - 1e-15);
        CHECK(closed <= environment::rock_cover(m, 0.0, 2.0 * d) + 1e-15);
    }
    // Continuous and monotone in the threshold (1 % steps from 5 mm to 30 m).
    double previous = environment::rock_statistical_cover(m, 0.005);
    for (double d = 0.005 * 1.01; d < 30.0; d *= 1.01) {
        const double c = environment::rock_statistical_cover(m, d);
        CHECK(c >= previous);
        CHECK(c - previous < 0.003 * c_all);
        previous = c;
    }
    // Drawn rocks (mean radiance rho G1(e) G(e) / N per plan area for either light) on the visible ground exp(-C_g)
    // plus that ground's factors give the all-statistical mean exactly: Sun and earthshine (shadows statistical).
    for (const double abundance : {1.0, 6.0}) {
        for (const double d : {0.01, 0.04, 0.3, 3.0}) {
            for (const double e_deg : {3.0, 7.0, 30.0}) {
                const double e = e_deg * deg;
                const double e_earth = 0.5 * e + 2.0 * deg;  // a different elevation for the Earth
                const auto all = environment::unresolved_light_factors(m, abundance, inf, 0.02, 20.0, e, e_earth);
                const auto ground = environment::unresolved_light_factors(m, abundance, d, 0.02, 20.0, e, e_earth);
                const double c_g = abundance * (c_all - environment::rock_statistical_cover(m, d));
                const double norm = std::exp(-abundance * c_all) + (1.0 - std::exp(-abundance * c_all)) * m.rock_reflectance_ratio;
                const double unshadowed = environment::unshadowed_slope_variance(m, 0.02, 20.0);
                const double lit = environment::smith_lit_fraction(std::tan(e), unshadowed);
                const double lit_earth = environment::smith_lit_fraction(std::tan(e_earth), unshadowed);
                const double rock_sun = lit * m.rock_reflectance_ratio * environment::rock_lit_gain_table(m, e) / norm;
                const double rock_earth =
                    lit_earth * m.rock_reflectance_ratio * environment::rock_lit_gain_table(m, e_earth) / norm;
                CHECK(std::abs(std::exp(-c_g) * ground.sun + (1.0 - std::exp(-c_g)) * rock_sun - all.sun) < 1e-12);
                CHECK(std::abs(std::exp(-c_g) * ground.earth + (1.0 - std::exp(-c_g)) * rock_earth - all.earth) < 1e-12);
            }
        }
    }
    // With the drawn rocks' own shadows (Task 12's shadow map) the ground loses only the statistical rocks' shadows.
    const auto own = environment::unresolved_light_factors(m, 1.0, 0.04, 0.02, 20.0, 7.0 * deg, 5.0 * deg, false);
    const auto stat = environment::unresolved_light_factors(m, 1.0, 0.04, 0.02, 20.0, 7.0 * deg, 5.0 * deg, true);
    CHECK(own.sun > stat.sun);
    CHECK(own.earth > stat.earth);
}

TEST_CASE("Shipped model: rock shape and reflectance ratio", "[moon_surface][unresolved_shading]") {
    const auto& m = shared_model();
    const auto& cfg = shared_detail()->config();
    std::printf("  eta %.3f, rho %.3f, cover 0.01-10 m %.4f (LunarRocks %.4f), drawn 1 cm %.4f 1 m %.4f, craters coef %.4e\n",
                m.rock_exposed_height_per_diameter, m.rock_reflectance_ratio,
                environment::rock_cover(m, 0.01, 10.0), shared_rocks().cover_fraction(0.01, 10.0),
                environment::drawn_slope_variance(m, 0.01), environment::drawn_slope_variance(m, 1.0),
                m.crater_variance_coefficient);
    CHECK(std::abs(environment::rock_cover(m, 0.01, 10.0) / shared_rocks().cover_fraction(0.01, 10.0) - 1.0) < 1e-12);
    CHECK(std::abs(environment::rock_cover(m, 0.5, 2.0) / shared_rocks().cover_fraction(0.5, 2.0) - 1.0) < 1e-12);
    CHECK(m.rock_exposed_height_per_diameter > 0.1);
    CHECK(m.rock_exposed_height_per_diameter < 0.8);
    // rho is taken at the standard geometry; its spread over other geometries (single-scattering-dominated: close to
    // the albedo ratio) is printed for the docs.
    config::HapkeParameters rock = cfg.hapke;
    rock.w = cfg.hapke.w * cfg.rocks.albedo_to_regolith;
    for (const auto& [i, e, g] : {std::array<double, 3>{30, 0, 30}, {60, 30, 50}, {85, 20, 80}, {87, 75, 12},
                                  {87, 75, 160}, {10, 8, 3}}) {
        const double ratio = environment::hapke_reflectance(rock, std::cos(i * deg), std::cos(e * deg), g * deg,
                                                            cfg.hapke.theta_bar_rad) /
                             environment::hapke_reflectance(cfg.hapke, std::cos(i * deg), std::cos(e * deg), g * deg,
                                                            cfg.hapke.theta_bar_rad);
        std::printf("    rock / regolith I/F at (i %.0f, e %.0f, g %.0f): %.3f\n", i, e, g, ratio);
    }
    CHECK(m.rock_reflectance_ratio > cfg.rocks.albedo_to_regolith - 0.05);
    REQUIRE(m.rock_lit_gain.size() == static_cast<size_t>(UnresolvedShadingModel::lit_gain_samples));
    REQUIRE(m.rock_shadow.size() == static_cast<size_t>(UnresolvedShadingModel::lit_gain_samples));
    const auto tables = environment::template_rock_light_tables(cfg.rocks.burial_fraction_mean, cfg.rocks.burial_fraction_sd);
    CHECK(std::abs(environment::rock_lit_gain_table(m, 20.0 * deg) - tables.gain[10]) < 1e-12);
    CHECK(std::abs(environment::rock_shadow_table(m, 20.0 * deg) - tables.shadow[10]) < 1e-12);
    // The generated angular rocks against the half-spheroid of the same eta (printed for the docs, R1).
    for (const double e_deg : {2.0, 4.0, 8.0, 14.0, 20.0, 30.0, 50.0, 80.0}) {
        const double e = e_deg * deg, eta = m.rock_exposed_height_per_diameter;
        std::printf("    e %4.1f deg: templates G %.3f sigma %.3f | half-spheroid G %.3f sigma %.3f\n", e_deg,
                    environment::rock_lit_gain_table(m, e), environment::rock_shadow_table(m, e),
                    environment::rock_lit_gain(eta, e), environment::rock_shadow_per_cover(eta, std::sin(e)));
    }
    // Deterministic: a second build gives the same numbers bit for bit.
    const auto again = environment::make_unresolved_shading_model(*shared_detail(), &shared_rocks());
    CHECK(again.rock_exposed_height_per_diameter == m.rock_exposed_height_per_diameter);
    CHECK(again.drawn_table == m.drawn_table);
    CHECK(again.crater_fraction == m.crater_fraction);
}

namespace {

// A half-spheroid of plan radius 1 and height h as a closed convex mesh (latitude-longitude rings, cut at z = 0 by the
// caller's cut 0): the reference shape of the view tables.
environment::TemplateMesh half_spheroid_mesh(double h, int rings, int sectors) {
    environment::TemplateMesh mesh;
    for (int i = 0; i <= rings; ++i) {
        const double u = 0.5 * pi * i / rings;  // 0 at the top
        for (int j = 0; j < sectors; ++j) {
            const double phi = 2.0 * pi * j / sectors;
            mesh.vertices.push_back({std::sin(u) * std::cos(phi), std::sin(u) * std::sin(phi), h * std::cos(u)});
        }
    }
    // A bottom ring slightly below the cut closes the body (the cut removes it).
    for (int j = 0; j < sectors; ++j) {
        const double phi = 2.0 * pi * j / sectors;
        mesh.vertices.push_back({std::cos(phi), std::sin(phi), -0.01});
    }
    for (int i = 0; i <= rings; ++i) {
        for (int j = 0; j < sectors; ++j) {
            const int a = i * sectors + j, b = i * sectors + (j + 1) % sectors;
            const int c = (i + 1) * sectors + j, d = (i + 1) * sectors + (j + 1) % sectors;
            mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
        }
    }
    return mesh;
}

size_t view_entry(int kv, int ka, int k) {
    return (static_cast<size_t>(kv) * UnresolvedShadingModel::view_azimuth_samples + static_cast<size_t>(ka)) *
               UnresolvedShadingModel::lit_gain_samples + static_cast<size_t>(k);
}

}  // namespace

TEST_CASE("Rock view tables: a half-spheroid's hidden ground, overlap limits and seen radiance",
          "[moon_surface][unresolved_shading]") {
    // Task 12 (review T11-C5 R1). The ground a convex body hides from the eye is the shadow of a light at the eye's
    // place: the polygon's hidden area against the closed form rock_shadow_per_cover; from the light's own direction
    // (dphi 0, same elevation) the eye sees no ground outside the shadow (X = 0) and every seen face is lit at
    // mu0 = mu (Lommel-Seeliger 1/2 = the flat ground's at that geometry: gain x |P_v| / A = 1 + hidden); from the
    // opposite azimuth the two regions share only the footprint (X = hidden).
    using M = UnresolvedShadingModel;
    for (const double eta : {0.25, 0.5}) {
        const auto mesh = half_spheroid_mesh(2.0 * eta, 48, 192);
        const auto t = environment::rock_view_tables(mesh, 0.0);
        double worst_hidden = 0.0, worst_same = 0.0, worst_opposite = 0.0, worst_flux = 0.0;
        for (int kv = 1; kv < M::view_elevation_samples - 1; ++kv) {  // 2 .. 88 deg
            const double ev = kv * M::view_elevation_step_rad;
            const int k_same = static_cast<int>(std::lround(ev / M::lit_gain_step_rad));
            const double closed = environment::rock_shadow_per_cover(eta, std::sin(ev));
            worst_hidden = std::max(worst_hidden, std::abs(t.hidden[view_entry(kv, 0, 0)] - closed) / (1.0 + closed));
            worst_same = std::max(worst_same, std::abs(t.excess[view_entry(kv, 0, k_same)]) / (1.0 + closed));
            worst_opposite = std::max(worst_opposite, std::abs(t.excess[view_entry(kv, M::view_azimuth_samples - 1, k_same)] -
                                                               t.hidden[view_entry(kv, 0, 0)]) / (1.0 + closed));
            worst_flux = std::max(worst_flux, std::abs(t.flux[view_entry(kv, 0, k_same)] /
                                                       (1.0 + t.hidden[view_entry(kv, 0, k_same)]) - 1.0));
        }
        std::printf("  view tables half-spheroid eta %.2f: hidden %.4f, X same-direction %.4f, X opposite %.4f, "
                    "flux at phase 0 %.4f (worst, 2-88 deg)\n", eta, worst_hidden, worst_same, worst_opposite, worst_flux);
        CHECK(worst_hidden < 0.01);
        CHECK(worst_same < 0.01);
        CHECK(worst_opposite < 0.01);
        CHECK(worst_flux < 0.01);
        // From the zenith nothing outside the footprint is hidden.
        CHECK(std::abs(t.hidden[view_entry(M::view_elevation_samples - 1, 0, 10)]) < 1e-9);
    }
}

TEST_CASE("Light factors from an oblique eye: the zenith view is the nadir law, the excess darkens the ground",
          "[moon_surface][unresolved_shading]") {
    const auto& m = shared_model();
    using M = UnresolvedShadingModel;
    REQUIRE(m.rock_view_excess.size() == static_cast<size_t>(M::view_elevation_samples * M::view_azimuth_samples *
                                                             M::lit_gain_samples));
    for (const double abundance : {1.0, 10.0}) {
        for (const double d : {0.1, inf}) {
            const auto nadir = environment::unresolved_light_factors(m, abundance, d, 0.3264, 20.0, 7 * deg, 12 * deg);
            const auto zenith = environment::unresolved_light_factors(m, abundance, d, 0.3264, 20.0, 7 * deg, 12 * deg,
                                                                      true, environment::UnresolvedView{});
            CHECK(zenith.sun == nadir.sun);
            CHECK(zenith.earth == nadir.earth);
        }
    }
    // The excess and the seen gain at the Task 11 rock gate's geometry (Sun 7 deg, eye 60 deg, 37 deg apart) and a
    // grazing eye; printed for the docs.
    for (const auto& [es, ev, dphi] : {std::array<double, 3>{7, 60, 37}, {7, 60, 180}, {7, 10, 0}, {7, 10, 90},
                                       {30, 20, 90}, {30, 20, 180}}) {
        std::printf("    Sun %2.0f deg, eye %2.0f deg, dphi %3.0f deg: X %.3f, gain ratio %.3f, sigma(eye) %.3f\n", es, ev,
                    dphi, environment::rock_view_excess_table(m, es * deg, ev * deg, dphi * deg),
                    environment::rock_view_gain_ratio_table(m, es * deg, ev * deg, dphi * deg),
                    environment::rock_shadow_table(m, ev * deg));
        CHECK(environment::rock_view_excess_table(m, es * deg, ev * deg, dphi * deg) >= 0.0);
    }
    // Down-Sun the eye sees the rocks' lit sides: per cover more light than from the zenith (the image share
    // 1 + sigma(e_v) times the gain ratio; Task 13: the ratio is per that share, which holds the plan view, so a tilted
    // rock's lit ground under its near bulge makes the ratio itself dip below 1 at phase 0), and more than up-Sun, where
    // the eye sees their dark sides.
    const double down = environment::rock_view_gain_ratio_table(m, 20 * deg, 20 * deg, 0.0);
    CHECK((1.0 + environment::rock_shadow_table(m, 20 * deg)) * down > 1.0);
    CHECK(down > environment::rock_view_gain_ratio_table(m, 20 * deg, 20 * deg, pi));
    CHECK(environment::rock_view_gain_ratio_table(m, 20 * deg, 20 * deg, pi) < 1.0);
    // The near-field shadow map takes the Sun's drawn-rock shadows and the drawn relief's G1 off the statistics; the
    // earthshine keeps them.
    environment::UnresolvedView mapped;
    mapped.sun_shadow_map = true;
    const auto with_map = environment::unresolved_light_factors(m, 1.0, 0.1, 0.0102, 20.0, 7 * deg, 7 * deg, false, mapped);
    const auto without = environment::unresolved_light_factors(m, 1.0, 0.1, 0.0102, 20.0, 7 * deg, 7 * deg, true,
                                                               environment::UnresolvedView{});
    const auto no_drawn = environment::unresolved_light_factors(m, 1.0, 0.1, 0.0102, 20.0, 7 * deg, 7 * deg, false,
                                                                environment::UnresolvedView{});
    const double g1 = environment::smith_lit_fraction(std::tan(7 * deg),
                                                      environment::unshadowed_slope_variance(m, 0.0102, 20.0));
    CHECK(std::abs(with_map.sun - no_drawn.sun / g1) < 1e-12);
    // The drawn rocks below shadow_caster_factor x d keep their Sun shadows statistical: between no drawn shadow and
    // all of them statistical, and exactly the latter with every drawn rock below the casting threshold.
    const auto mapped_drawn = environment::unresolved_light_factors(m, 1.0, 0.1, 0.0102, 20.0, 7 * deg, 7 * deg, true,
                                                                    mapped);
    CHECK(mapped_drawn.sun < with_map.sun);
    CHECK(mapped_drawn.sun > without.sun / g1);
    const auto all_uncast = environment::unresolved_light_factors(m, 1.0, 4.0, 0.0102, 20.0, 7 * deg, 7 * deg, true,
                                                                  mapped);
    const auto all_stat = environment::unresolved_light_factors(m, 1.0, 4.0, 0.0102, 20.0, 7 * deg, 7 * deg, true,
                                                                environment::UnresolvedView{});
    CHECK(std::abs(all_uncast.sun - all_stat.sun / g1) < 1e-12);
    CHECK(with_map.earth == no_drawn.earth);
    CHECK(mapped_drawn.earth == without.earth);
    // A map that resolves the relief only from 0.5 m: the drawn relief finer than that keeps its Smith factor.
    environment::UnresolvedView coarse = mapped;
    coarse.shadow_map_resolved_m = 0.5;
    const auto with_coarse = environment::unresolved_light_factors(m, 1.0, 0.1, 0.0102, 20.0, 7 * deg, 7 * deg, false, coarse);
    const double g1_coarse = environment::smith_lit_fraction(std::tan(7 * deg),
                                                             environment::unshadowed_slope_variance(m, 0.0102, 0.5));
    CHECK(std::abs(with_coarse.sun - with_map.sun * g1_coarse) < 1e-12);
    CHECK(g1_coarse < 1.0);
    CHECK(g1_coarse > g1);
}

TEST_CASE("Smith visible-lit fraction: zenith limit, shadow hiding, monotone in the azimuth difference",
          "[moon_surface][unresolved_shading]") {
    const double v = 0.3;
    for (const double el : {3.0, 7.0, 20.0}) {
        const double tl = std::tan(el * deg);
        CHECK(std::abs(environment::smith_visible_lit_fraction(tl, inf, 1.0, v) -
                       environment::smith_lit_fraction(tl, v)) < 1e-15);
        // The eye at the light's place sees no shadow.
        CHECK(std::abs(environment::smith_visible_lit_fraction(tl, tl, 0.0, v) - 1.0) < 1e-15);
        double previous = 2.0;
        for (const double phi : {0.0, 0.5, 1.0, 2.0, pi}) {
            const double f = environment::smith_visible_lit_fraction(tl, std::tan(30 * deg), phi, v);
            CHECK(f <= previous + 1e-15);
            CHECK(f > 0.0);
            previous = f;
        }
        // Opposite azimuths at a high eye: close to the uncorrelated product's (1 + Lv) / (1 + Ll + Lv).
        std::printf("    Sun %4.1f deg: G1 %.4f, eye 30 deg same azimuth %.4f, opposite %.4f\n", el,
                    environment::smith_lit_fraction(tl, v),
                    environment::smith_visible_lit_fraction(tl, std::tan(30 * deg), 0.0, v),
                    environment::smith_visible_lit_fraction(tl, std::tan(30 * deg), pi, v));
    }
}

TEST_CASE("Hapke hemispherical albedo: isotropic scatterers 1 - gamma H(mu0), the shipped regolith's range",
          "[moon_surface][unresolved_shading]") {
    // Isotropic single scattering, no opposition effect, smooth, K = 1: A_h(mu0) = 1 - sqrt(1 - w) H(mu0) (Hapke 2012,
    // eq. 10.33 with the exact H; the 2002 approximation of H is within 1 %: at w 0.9, 1 - gamma H ~0.5, the identity holds to ~2 %).
    config::HapkeParameters iso;
    iso.w = 0.9;
    iso.b = 0.0;
    iso.c = 0.0;
    iso.porosity_k = 1.0;
    double worst = 0.0;
    for (const double mu0 : {0.05, 0.2, 0.5, 0.8, 1.0}) {
        const double expected = 1.0 - std::sqrt(1.0 - iso.w) *
                                          environment::hapke_h_function(mu0, iso.w, environment::HapkeHFunction::hapke_2002);
        worst = std::max(worst, std::abs(environment::hapke_hemispherical_albedo(iso, mu0, 0.0) / expected - 1.0));
    }
    std::printf("  isotropic A_h against 1 - gamma H: worst %.4f\n", worst);
    CHECK(worst < 0.03);  // [D] the H approximation's error, amplified by 1 - gamma H ~ 0.5
    // The integration itself, exactly: at w -> 0 single scattering dominates, A_h = (w / 2) (1 - mu0 ln(1 + 1 / mu0))
    // (Lommel-Seeliger's hemispherical integral; multiple scattering adds O(w) relative).
    config::HapkeParameters thin = iso;
    thin.w = 1e-4;
    double worst_ls = 0.0;
    for (const double mu0 : {0.05, 0.2, 0.5, 0.8, 1.0}) {
        const double expected = 0.5 * thin.w * (1.0 - mu0 * std::log(1.0 + 1.0 / mu0));
        worst_ls = std::max(worst_ls, std::abs(environment::hapke_hemispherical_albedo(thin, mu0, 0.0) / expected - 1.0));
    }
    std::printf("  single-scattering limit: worst %.5f\n", worst_ls);
    CHECK(worst_ls < 1e-3);
    const auto& m = shared_model();
    REQUIRE(m.hemispherical_albedo.size() == static_cast<size_t>(UnresolvedShadingModel::lit_gain_samples));
    for (const double e : {2.0, 7.0, 15.0, 30.0, 60.0, 90.0}) {
        std::printf("    regolith A_h at Sun %2.0f deg: %.4f\n", e, environment::hemispherical_albedo_table(m, e * deg));
        CHECK(environment::hemispherical_albedo_table(m, e * deg) > 0.02);
        CHECK(environment::hemispherical_albedo_table(m, e * deg) < 0.4);
    }
}

TEST_CASE("Rock view tables' file cache: bit-identical round trip, the key guards inputs and format",
          "[moon_surface][unresolved_shading]") {
    // Review I5 of Task 12: the game reads the tables from data/cache instead of building them (~8 s) at each start.
    const auto& m = shared_model();
    const auto& cfg = shared_detail()->config();
    const double mean = cfg.rocks.burial_fraction_mean, sd = cfg.rocks.burial_fraction_sd;
    const std::uint64_t key = environment::rock_view_cache_key(mean, sd);
    CHECK(key == environment::rock_view_cache_key(mean, sd));
    CHECK(key != environment::rock_view_cache_key(mean, sd + 0.01));
    CHECK(key != environment::rock_view_cache_key(mean + 0.01, sd));
    // A directory of its own (ctest runs cases as parallel processes; Task 13, review n4 of Task 12).
    const auto dir = std::filesystem::temp_directory_path() /
                     ("starsim_rock_view_cache_test_" +
                      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "_" +
                      std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())));
    std::filesystem::remove_all(dir);
    const auto file = dir / environment::rock_view_cache_name;
    environment::RockViewModel model{m.rock_view_excess, m.rock_view_gain_ratio};
    environment::write_rock_view_cache(file, key, model);
    REQUIRE(std::filesystem::exists(file));
    environment::RockViewModel back;
    REQUIRE(environment::read_rock_view_cache(file, key, back));
    CHECK(back.excess == model.excess);  // bit for bit
    CHECK(back.gain_ratio == model.gain_ratio);
    environment::RockViewModel other;
    CHECK_FALSE(environment::read_rock_view_cache(file, key + 1, other));
    CHECK(other.excess.empty());
    // A truncated file is refused.
    std::filesystem::resize_file(file, std::filesystem::file_size(file) - 8);
    CHECK_FALSE(environment::read_rock_view_cache(file, key, other));
    // template_rock_view_tables takes a matching file instead of building (a burial law nothing else uses: no
    // in-process entry; the file holds the shipped tables, so a build would differ).
    const double mean2 = 0.123, sd2 = 0.0456;
    environment::write_rock_view_cache(file, environment::rock_view_cache_key(mean2, sd2), model);
    const auto read = environment::template_rock_view_tables(mean2, sd2, file);
    CHECK(read.excess == model.excess);
    CHECK(read.gain_ratio == model.gain_ratio);
    std::filesystem::remove_all(dir);
    // A failed write reports false (a cache path under a plain file cannot be created).
    {
        const auto blocker = std::filesystem::temp_directory_path() /
                             ("starsim_rock_view_cache_blocker_" +
                              std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::ofstream(blocker) << "x";
        CHECK_FALSE(environment::write_rock_view_cache(blocker / "sub" / environment::rock_view_cache_name, key, model));
        std::filesystem::remove(blocker);
    }
    // The version guard (review n4): a checksum of the shipped tables (rounded to 1e-6, so last-bit differences between
    // CPUs do not matter) pinned per rock_view_cache_version. When it fails, the tables changed: if the templates, the
    // burial law and the grids did not change, the algorithm did - bump rock_view_cache_version; then pin the printed sum.
    std::uint64_t sum = 1469598103934665603ULL;
    for (const std::vector<double>* t : {&m.rock_view_excess, &m.rock_view_gain_ratio}) {
        for (const double v : *t) {
            const auto r = static_cast<std::int64_t>(std::llround(v * 1e6));
            for (int b = 0; b < 8; ++b) {
                sum ^= static_cast<std::uint64_t>((r >> (8 * b)) & 0xFF);
                sum *= 1099511628211ULL;
            }
        }
    }
    std::printf("  rock view tables: version %llu, checksum %llu\n",
                static_cast<unsigned long long>(environment::rock_view_cache_version), static_cast<unsigned long long>(sum));
    constexpr std::uint64_t pinned_version = 2, pinned_checksum = 5551542726429447617ULL;  // update together
    CHECK(environment::rock_view_cache_version == pinned_version);
    CHECK(sum == pinned_checksum);
}
