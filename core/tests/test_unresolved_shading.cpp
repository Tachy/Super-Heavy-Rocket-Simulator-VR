// Unresolved-surface shading statistics (Task 10 of the Moon procedural surface plan): the Hapke slope budget, the
// regolith and crater slope variances against the generator, Smith's shadowing function, the rock shadow / lit-face
// terms and the light factors the Moon terrain shader mirrors.

#include <algorithm>
#include <array>
#include <cmath>
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
    // Hapke's mean-tan form: (1 - A) <tan>_regolith + A pi eta = <tan>_globe = (pi / 2) tan theta_bar.
    const double regolith_tan = 0.5 * pi * std::tan(environment::theta_bar_from_slope_variance(m.total_slope_variance));
    const double rocks_tan = pi * m.rock_exposed_height_per_diameter;
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
    CHECK(std::abs(sum / (pi * a * a) / rocks_tan - 1.0) < 2e-3);  // the rim's integrable singularity limits the sum
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

TEST_CASE("Light factors: albedo share keeps the mosaic mean, the earthshine sees no shadow share",
          "[moon_surface][unresolved_shading]") {
    const auto& m = shared_model();
    // Every rock statistical: the earthshine factor is the mosaic's mean exactly, at any Sun elevation and spacing.
    for (const double e_deg : {-5.0, 2.0, 7.0, 40.0}) {
        for (const double spacing : {0.01, 1.0, 100.0}) {
            const auto f = environment::unresolved_light_factors(m, 1.0, inf, spacing, 20.0, e_deg * deg);
            CHECK(std::abs(f.earth - 1.0) < 1e-12);
            if (e_deg < 0.0) CHECK(f.sun == 0.0);
        }
    }
    // Rocks above the geometry threshold drawn elsewhere: the statistical share drops their albedo share.
    const auto drawn = environment::unresolved_light_factors(m, 1.0, 0.5, 1.0, 20.0, 30.0 * deg);
    CHECK(drawn.earth < 1.0);
    // No rocks at all (abundance 0) and no drawn relief below the horizon spacing: both factors 1.
    const auto bare = environment::unresolved_light_factors(m, 0.0, inf, 1.0e4, 1.0e4, 7.0 * deg);
    CHECK(std::abs(bare.sun - 1.0) < 1e-12);
    CHECK(std::abs(bare.earth - 1.0) < 1e-12);
    for (const double e_deg : {3.0, 7.0, 15.0, 30.0, 60.0}) {
        const auto near = environment::unresolved_light_factors(m, 1.0, inf, 0.01, 20.0, e_deg * deg);
        const auto rocks = environment::unresolved_light_factors(m, 1.0, inf, 1.0e4, 1.0e4, e_deg * deg);
        std::printf("  Sun %4.1f deg: sun factor 1 cm node %.3f (Smith G1 %.3f), rocks alone %.3f; earth %.3f\n", e_deg,
                    near.sun,
                    environment::smith_lit_fraction(std::tan(e_deg * deg), environment::unshadowed_slope_variance(m, 0.01, 20.0)),
                    rocks.sun, near.earth);
    }
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
    CHECK(std::abs(environment::rock_lit_gain_table(m, 20.0 * deg) -
                   environment::rock_lit_gain(m.rock_exposed_height_per_diameter, 20.0 * deg)) < 1e-12);
    // Deterministic: a second build gives the same numbers bit for bit.
    const auto again = environment::make_unresolved_shading_model(*shared_detail(), &shared_rocks());
    CHECK(again.rock_exposed_height_per_diameter == m.rock_exposed_height_per_diameter);
    CHECK(again.drawn_table == m.drawn_table);
    CHECK(again.crater_fraction == m.crater_fraction);
}
