// Terrain horizon (terrain shadows): the exact smooth-sphere dip, an analytic Gaussian mountain, the real LDEM_16 /
// IM-2 tile against a brute-force march, poles and the antimeridian, and the march's timing.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <random>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/terrain_horizon.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/fast_trig.hpp"
#include "starsim/sim/lunar_site.hpp"

using namespace starsim;
using environment::HorizonMarch;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double R = constants::moon_mean_radius_m;
constexpr double site_lat = -84.7906 * deg, site_lon = 29.1957 * deg;
constexpr double tolerance_rad = 0.1 * deg;

Vec3 unit(double lat, double lon) {
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

// Great-circle destination (vector form) at angular distance t along azimuth az (clockwise from north).
std::pair<double, double> destination(double lat, double lon, double az, double t) {
    const Vec3 up = unit(lat, lon);
    const Vec3 east{-std::sin(lon), std::cos(lon), 0.0};
    const Vec3 north{-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat)};
    const Vec3 p = up * std::cos(t) + (north * std::cos(az) + east * std::sin(az)) * std::sin(t);
    return {std::asin(p.z), std::atan2(p.y, p.x)};
}

// A terrain of constant height (a smooth sphere when 0).
class FlatTerrain : public environment::Terrain {
public:
    explicit FlatTerrain(double height_m) : height_(height_m) {}
    double height_at_fixed_m(const Vec3&) const override { return height_; }
    Vec3 normal_fixed(const Vec3& p) const override { return math::normalized(p); }
    double height_m(double, double) const override { return height_; }
    double resolution_m(double, double) const override { return 100.0; }
    double max_height_m() const override { return height_; }

private:
    double height_;
};

// One Gaussian mountain: amplitude_m * exp(-d^2 / (2 sigma^2)), d the great-circle distance to its centre.
class GaussianMountain : public environment::Terrain {
public:
    GaussianMountain(double lat, double lon, double amplitude_m, double sigma_m)
        : centre_(unit(lat, lon)), amplitude_m_(amplitude_m), sigma_m_(sigma_m) {}
    double height_at_fixed_m(const Vec3& p) const override {
        const Vec3 u = math::normalized(p);
        return height_m(std::asin(u.z), std::atan2(u.y, u.x));
    }
    Vec3 normal_fixed(const Vec3& p) const override { return math::normalized(p); }
    double height_m(double lat, double lon) const override {
        const double d = R * std::acos(std::clamp(dot(centre_, unit(lat, lon)), -1.0, 1.0));
        return amplitude_m_ * std::exp(-d * d / (2.0 * sigma_m_ * sigma_m_));
    }
    double resolution_m(double, double) const override { return 100.0; }
    double max_height_m() const override { return amplitude_m_; }

private:
    Vec3 centre_;
    double amplitude_m_, sigma_m_;
};

// Brute force: samples every half terrain pixel (finer near the observer) out to 400 km, no early-out.
double brute_force_elevation_rad(const environment::Terrain& terrain, double lat, double lon, double h, double az) {
    constexpr double range_m = 400e3;
    const double r0 = R + h;
    double best = -1e9, d = 10.0;
    while (d < range_m) {
        const double t = d / R;
        const auto [la, lo] = destination(lat, lon, az, t);
        const double r1 = R + terrain.height_m(la, lo);
        best = std::max(best, std::atan2(r1 * std::cos(t) - r0, r1 * std::sin(t)));
        // Half a pixel, but near the observer finer too: a bilinear cell can rise 15 deg within a few hundred metres.
        d += std::min(0.5 * terrain.resolution_m(la, lo), 10.0 + 0.0005 * d);
    }
    return best;
}

std::shared_ptr<const environment::LunarTerrain> lunar_terrain_or_skip() {
    if (!std::filesystem::exists(std::filesystem::path(STARSIM_DATA_DIR) / "terrain" / "ldem_16.i16")) {
        SKIP("data/terrain missing (tools/data_pipeline/fetch.py)");
    }
    return sim::load_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
}

constexpr HorizonMarch leaf_march{10.0, 10.0};

}  // namespace

TEST_CASE("A smooth sphere's horizon is the exact dip -acos(R / (R + h))", "[terrain_horizon]") {
    const FlatTerrain flat(0.0);
    for (const double h : {10.0, 2000.0, 20000.0}) {
        for (const double az : {0.0, 1.0, 4.0}) {
            const double e = environment::horizon_elevation_rad(flat, R, 0.3, 1.0, h, az, leaf_march);
            CHECK(e == Catch::Approx(-std::acos(R / (R + h))).margin(1e-4));
        }
    }
}

TEST_CASE("A Gaussian mountain's horizon matches the analytic maximum of the profile", "[terrain_horizon]") {
    constexpr double lat = 0.2, lon = -2.0, az = 0.7, h = 10.0;
    constexpr double distance_m = 40e3, amplitude_m = 3000.0, sigma_m = 5000.0;
    const auto [mlat, mlon] = destination(lat, lon, az, distance_m / R);
    const GaussianMountain mountain(mlat, mlon, amplitude_m, sigma_m);
    double analytic = -1e9;  // along the radial line: the profile at angular distance t is H(distance - t R)
    for (double t = 1e-6; t < 0.05; t += 1e-6) {
        const double off_m = distance_m - t * R;
        const double r1 = R + amplitude_m * std::exp(-off_m * off_m / (2.0 * sigma_m * sigma_m));
        analytic = std::max(analytic, std::atan2(r1 * std::cos(t) - (R + h), r1 * std::sin(t)));
    }
    REQUIRE(analytic > 0.0);
    const double e = environment::horizon_elevation_rad(mountain, R, lat, lon, h, az, leaf_march);
    CHECK(std::abs(e - analytic) <= tolerance_rad);
    // Away from the mountain the horizon is the plain sphere's.
    const double away = environment::horizon_elevation_rad(mountain, R, lat, lon, h, az + constants::pi, leaf_march);
    CHECK(away == Catch::Approx(-std::acos(R / (R + h))).margin(1e-4));
}

TEST_CASE("A batch is point-major, azimuth-minor and equals the single calls", "[terrain_horizon]") {
    const GaussianMountain mountain(0.1, 0.1, 2000.0, 8000.0);
    const std::vector<double> lat{0.09, 0.0}, lon{0.1, 0.2}, h{5.0, 50.0};
    const auto many = environment::horizon_elevations(mountain, R, lat, lon, h, 4, leaf_march);
    REQUIRE(many.size() == 8);
    for (size_t i = 0; i < 2; ++i) {
        for (int k = 0; k < 4; ++k) {
            const double e = environment::horizon_elevation_rad(mountain, R, lat[i], lon[i], h[i],
                                                                k * 2.0 * constants::pi / 4.0, leaf_march);
            CHECK(many[i * 4 + static_cast<size_t>(k)] == Catch::Approx(e).margin(1e-6));
        }
    }
    CHECK_THROWS(environment::horizon_elevations(mountain, R, lat, lon, {1.0}, 4, leaf_march));
}

TEST_CASE("LOLA data: the march agrees with a brute-force march and its early-out is conservative",
          "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    std::mt19937 rng(20261030);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    int compared = 0;
    double worst_rad = 0.0;
    for (int i = 0; i < 64; ++i) {
        // Half the points over the whole Moon (LDEM_16), half within 30 km of the IM-2 site (20 m tile).
        double lat, lon;
        if (i % 2 == 0) {
            lat = std::asin(2.0 * u01(rng) - 1.0);
            lon = (2.0 * u01(rng) - 1.0) * constants::pi;
        } else {
            std::tie(lat, lon) = destination(site_lat, site_lon, 2.0 * constants::pi * u01(rng), 30e3 * u01(rng) / R);
        }
        const double h = terrain->height_m(lat, lon) + 2.0;
        for (int k = 0; k < 16; ++k) {
            const double az = k * 2.0 * constants::pi / 16.0;
            const double marched = environment::horizon_elevation_rad(*terrain, R, lat, lon, h, az, leaf_march);
            const double brute = brute_force_elevation_rad(*terrain, lat, lon, h, az);
            INFO("point " << i << " lat " << lat / deg << " lon " << lon / deg << " h " << h << " az " << k);
            CHECK(marched - brute <= tolerance_rad);  // conservative: never higher
            if (brute < 5.0 * deg) {
                CHECK(std::abs(marched - brute) <= tolerance_rad);
                worst_rad = std::max(worst_rad, std::abs(marched - brute));
                ++compared;
            }
        }
    }
    std::printf("[terrain_horizon] %d rays compared, worst |march - brute| = %.4f deg\n", compared, worst_rad / deg);
    CHECK(compared > 500);
}

TEST_CASE("LOLA data: poles and the antimeridian give finite horizons", "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    for (const double lat : {-89.9 * deg, 89.9 * deg, -90.0 * deg, 90.0 * deg}) {
        for (const double lon : {0.0, 90.0 * deg, constants::pi, -constants::pi}) {
            const double h = terrain->height_m(lat, lon) + 2.0;
            for (int k = 0; k < 16; ++k) {
                const double e = environment::horizon_elevation_rad(*terrain, R, lat, lon, h,
                                                                    k * 2.0 * constants::pi / 16.0, leaf_march);
                REQUIRE(std::isfinite(e));
                CHECK(std::abs(e) < 0.5 * constants::pi);
            }
        }
    }
    // Across lon +-180: the two sides of the antimeridian see the same ridge line, so the horizon is continuous.
    const double east = environment::horizon_elevation_rad(*terrain, R, 0.1, constants::pi - 1e-6, 2000.0, 1.0,
                                                           leaf_march);
    const double west = environment::horizon_elevation_rad(*terrain, R, 0.1, -constants::pi + 1e-6, 2000.0, 1.0,
                                                           leaf_march);
    CHECK(std::abs(east - west) < 0.01 * deg);
}

namespace {

// Times a 65 x 65 grid of `spacing_m` around the IM-2 site, 16 azimuths each.
void print_timing(const environment::LunarTerrain& terrain, double spacing_m, const HorizonMarch& march,
                  const char* label) {
    std::vector<double> lat, lon, h;
    for (int i = 0; i < 65; ++i) {
        for (int j = 0; j < 65; ++j) {
            const double north_m = (i - 32) * spacing_m, east_m = (j - 32) * spacing_m;
            const double la = site_lat + north_m / R;
            const double lo = site_lon + east_m / (R * std::cos(site_lat));
            lat.push_back(la);
            lon.push_back(lo);
            h.push_back(terrain.height_m(la, lo));
        }
    }
    const auto t0 = std::chrono::steady_clock::now();
    const auto out = environment::horizon_elevations(terrain, R, lat, lon, h, 16, march);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[terrain_horizon] %s: 65 x 65 x 16 at %.1f m spacing: %.1f ms (%.2f ms per 1000 rays)\n", label,
                spacing_m, ms, ms / (static_cast<double>(out.size()) / 1000.0));
    CHECK(out.size() == 65u * 65u * 16u);
}

}  // namespace

TEST_CASE("LOLA data: horizon march timing (printed, not asserted)", "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    print_timing(*terrain, 10.4, {10.4, 10.4}, "leaf (level 12)");
    print_timing(*terrain, 333.0, {333.0, 333.0}, "level 7");
}

// ---- Banded node lattices (horizon_elevations_lattice) ----

namespace {

// A terrain forwarding to another and counting the height samples of the march.
class CountingTerrain : public environment::Terrain {
public:
    explicit CountingTerrain(const environment::Terrain& inner) : inner_(inner) {}
    double height_at_fixed_m(const Vec3& p) const override { return inner_.height_at_fixed_m(p); }
    Vec3 normal_fixed(const Vec3& p) const override { return inner_.normal_fixed(p); }
    double height_m(double lat, double lon) const override { return inner_.height_m(lat, lon); }
    double height_at_direction_m(const Vec3& u) const override {
        ++samples;
        return inner_.height_at_direction_m(u);
    }
    double resolution_m(double lat, double lon) const override { return inner_.resolution_m(lat, lon); }
    double max_height_m() const override { return inner_.max_height_m(); }
    mutable size_t samples = 0;

private:
    const environment::Terrain& inner_;
};

struct NodeLattice {
    int n = 65;
    std::vector<double> lat, lon, h;
};

// An n x n lattice of ~spacing_m, row i going north, column j east, around the centre (column 0 lies first_column
// spacings east of it, row 0 first_row spacings north), on the sphere by the vector form (poles, antimeridian).
NodeLattice make_lattice(const environment::Terrain& terrain, double lat0, double lon0, double spacing_m,
                         int first_column = -32, int first_row = -32) {
    NodeLattice node;
    const Vec3 up = unit(lat0, lon0);
    const Vec3 east{-std::sin(lon0), std::cos(lon0), 0.0};
    const Vec3 north{-std::sin(lat0) * std::cos(lon0), -std::sin(lat0) * std::sin(lon0), std::cos(lat0)};
    for (int i = 0; i < node.n; ++i) {
        for (int j = 0; j < node.n; ++j) {
            const Vec3 p = math::normalized(up + north * ((first_row + i) * spacing_m / R) +
                                            east * ((first_column + j) * spacing_m / R));
            node.lat.push_back(std::asin(p.z));
            node.lon.push_back(std::atan2(p.y, p.x));
            node.h.push_back(terrain.height_m(node.lat.back(), node.lon.back()));
        }
    }
    return node;
}

// Compares the banded lattice with the full per-vertex march (the same samples, no bands); prints
// worst / 99th percentile; checks the tolerance where the reference is below 5 deg.
// The limits are the measured values with a margin (see the cases): where the reference is below 5 deg the worst and
// the 99th percentile of |banded - full|, in degrees.
void compare_banded(const environment::Terrain& terrain, double lat0, double lon0, double spacing_m,
                    const char* label, double worst_limit_deg, double p99_limit_deg) {
    const NodeLattice node = make_lattice(terrain, lat0, lon0, spacing_m);
    const auto banded = environment::horizon_elevations_lattice(terrain, R, node.n, node.lat, node.lon, node.h,
                                                                spacing_m, 16);
    std::vector<float> full(banded.size());  // the vertex's own march: from one spacing, the lattice's first step
    for (size_t v = 0; v < node.lat.size(); ++v) {
        const environment::HorizonMarch march{spacing_m, environment::lattice_first_step_m(
                                                             terrain, node.lat[v], node.lon[v], spacing_m)};
        for (size_t k = 0; k < 16; ++k) {
            full[v * 16 + k] = static_cast<float>(environment::horizon_elevation_rad(
                terrain, R, node.lat[v], node.lon[v], node.h[v], static_cast<double>(k) * 2.0 * constants::pi / 16.0,
                march));
        }
    }
    REQUIRE(banded.size() == full.size());
    std::vector<double> all, low;
    for (size_t i = 0; i < full.size(); ++i) {
        const double d = std::abs(static_cast<double>(banded[i]) - full[i]) / deg;
        all.push_back(d);
        if (full[i] < 5.0 * deg) low.push_back(d);
    }
    const auto percentile = [](std::vector<double> v, double p) {
        std::sort(v.begin(), v.end());
        return v[static_cast<size_t>(p * static_cast<double>(v.size() - 1))];
    };
    REQUIRE(!low.empty());
    const double worst_low = *std::max_element(low.begin(), low.end());
    std::printf("[terrain_horizon] banded %s: reference < 5 deg: %zu values, worst %.4f deg, p99 %.4f deg; "
                "all %zu: worst %.4f deg, p99 %.4f deg\n", label, low.size(), worst_low, percentile(low, 0.99),
                all.size(), *std::max_element(all.begin(), all.end()), percentile(all, 0.99));
    CHECK(worst_low <= worst_limit_deg);
    CHECK(percentile(low, 0.99) <= p99_limit_deg);
}

}  // namespace

TEST_CASE("A banded lattice over a smooth sphere is the exact dip everywhere", "[terrain_horizon]") {
    const FlatTerrain flat(0.0);
    const NodeLattice node = make_lattice(flat, 0.4, -1.0, 50.0);
    const auto out = environment::horizon_elevations_lattice(flat, R, node.n, node.lat, node.lon, node.h, 50.0, 8);
    REQUIRE(out.size() == 65u * 65u * 8u);
    for (const float e : out) CHECK(e == Catch::Approx(0.0).margin(1e-4));  // at h = 0 the dip is 0 (grazing)
    CHECK_THROWS(environment::horizon_elevations_lattice(flat, R, 60, node.lat, node.lon, node.h, 50.0, 8));
}

TEST_CASE("Adjacent banded lattices give identical values along their shared edge", "[terrain_horizon]") {
    // A mountain 4 km away so that the near, mid and far bands all matter at the edge.
    const auto [mlat, mlon] = destination(0.2, 0.3, 1.2, 4000.0 / R);
    const GaussianMountain mountain(mlat, mlon, 1500.0, 700.0);
    constexpr double spacing_m = 40.0;
    // Lattice A covers columns -64..0 around the centre, lattice B columns 0..64 (A's column 64 is B's column 0).
    const NodeLattice a = make_lattice(mountain, 0.2, 0.3, spacing_m, -64, -32);
    const NodeLattice b = make_lattice(mountain, 0.2, 0.3, spacing_m, 0, -32);
    const auto ha = environment::horizon_elevations_lattice(mountain, R, 65, a.lat, a.lon, a.h, spacing_m, 16);
    const auto hb = environment::horizon_elevations_lattice(mountain, R, 65, b.lat, b.lon, b.h, spacing_m, 16);
    int differing = 0;
    for (int i = 0; i < 65; ++i) {
        for (int k = 0; k < 16; ++k) {
            const size_t ia = (static_cast<size_t>(i) * 65 + 64) * 16 + static_cast<size_t>(k);
            const size_t ib = (static_cast<size_t>(i) * 65 + 0) * 16 + static_cast<size_t>(k);
            if (ha[ia] != hb[ib]) ++differing;
        }
    }
    CHECK(differing == 0);
}

TEST_CASE("A banded lattice stays within 0.45 deg of the full march on a Gaussian mountain", "[terrain_horizon]") {
    const auto [mlat, mlon] = destination(0.2, 0.3, 1.2, 6000.0 / R);
    const GaussianMountain mountain(mlat, mlon, 1800.0, 1500.0);
    compare_banded(mountain, 0.2, 0.3, 40.0, "gaussian 40 m", 0.45, 0.3);
}

TEST_CASE("A coarse-level lattice (3 km spacing, step cap spacing / 2) matches the full march", "[terrain_horizon]") {
    const auto [mlat, mlon] = destination(0.2, 0.3, 1.2, 60000.0 / R);
    const GaussianMountain mountain(mlat, mlon, 1800.0, 1500.0);  // a 1.5 km wide ridge: found at the 1.5 km step cap
    // worst 1.28 deg (a 1.8 km peak seen at an interpolated azimuth), p99 0
    compare_banded(mountain, 0.2, 0.3, 3000.0, "gaussian 3 km", 1.4, 0.05);
}

TEST_CASE("LOLA data: banded lattice vs the full march, IM-2 leaf and level 7", "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    // Rough relief next to the vertices: a ridge that is the horizon at one vertex and not at the next switches the
    // occluder, and the linear interpolation over a stride errs by up to half the difference of their slopes (worst
    // value, rare); the typical error (p99) is a fifth to a tenth of it.
    compare_banded(*terrain, site_lat, site_lon, 10.4, "IM-2 leaf 10.4 m", 0.4, 0.1);
    compare_banded(*terrain, site_lat, site_lon, 333.0, "IM-2 level 7 333 m", 0.65, 0.3);
}

TEST_CASE("LOLA data: banded lattice vs the full march near the pole and across lon 180", "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    // At the pole the north of neighbouring vertices turns by degrees: the bake falls back to smaller strides (exact
    // inside; the edge vertices interpolate over an edge stride decided from the edge alone, for the seams: worst 0.30).
    compare_banded(*terrain, -89.5 * deg, 20.0 * deg, 333.0, "lat -89.5 level 7", 0.35, 0.1);
    compare_banded(*terrain, 5.0 * deg, constants::pi - 0.02 * deg, 333.0, "lon 180 level 7", 1.1, 0.65);
    compare_banded(*terrain, 5.0 * deg, 100.0 * deg, 333.0, "lon 100 level 7 (control)", 0.5, 0.2);
}

TEST_CASE("LOLA data: banded lattice timing (printed, not asserted)", "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    struct Case {
        const char* label;
        double lat, lon, spacing_m;
    };
    for (const Case& c : {Case{"IM-2 leaf 10.4 m", site_lat, site_lon, 10.4},
                          Case{"IM-2 level 7 333 m", site_lat, site_lon, 333.0},
                          Case{"far-side plain level 7 333 m", 0.0, 165.0 * deg, 333.0},
                          Case{"IM-2 level 5 1.5 km", site_lat, site_lon, 1492.0},
                          Case{"IM-2 level 4 3 km", site_lat, site_lon, 2984.0},
                          Case{"IM-2 level 2 12 km", site_lat, site_lon, 11936.0},
                          Case{"IM-2 level 1 24 km", site_lat, site_lon, 23872.0}}) {
        const NodeLattice node = make_lattice(*terrain, c.lat, c.lon, c.spacing_m);
        const auto t0 = std::chrono::steady_clock::now();
        const auto out = environment::horizon_elevations_lattice(*terrain, R, node.n, node.lat, node.lon, node.h,
                                                                 c.spacing_m, 16);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        const CountingTerrain counting(*terrain);
        environment::horizon_elevations_lattice(counting, R, node.n, node.lat, node.lon, node.h, c.spacing_m, 16);
        std::printf("[terrain_horizon] banded timing %s: 65 x 65 x 16: %.1f ms, %zu samples, %.1f ns per sample\n",
                    c.label, ms, counting.samples, ms * 1e6 / static_cast<double>(counting.samples));
        CHECK(out.size() == 65u * 65u * 16u);
    }
}

TEST_CASE("fast_atan, fast_atan2 and fast_asin agree with the library to 3e-8 rad", "[terrain_horizon]") {
    double worst = 0.0;
    for (int i = -4000; i <= 4000; ++i) {
        const double x = i / 400.0;  // |x| up to 10, across both argument reductions
        worst = std::max(worst, std::abs(math::fast_atan(x) - std::atan(x)));
    }
    for (int i = 0; i < 3600; ++i) {
        const double a = (i + 0.5) * constants::pi / 1800.0 - constants::pi;  // all quadrants, incl. near the axes
        worst = std::max(worst, std::abs(math::fast_atan2(std::sin(a), std::cos(a)) - a));
    }
    for (int i = -1000; i <= 1000; ++i) {
        worst = std::max(worst, std::abs(math::fast_asin(i / 1000.0) - std::asin(i / 1000.0)));
    }
    CHECK(worst < 3e-8);
    CHECK(math::fast_atan2(0.0, 0.0) == 0.0);
    CHECK(math::fast_atan2(0.0, -1.0) == Catch::Approx(constants::pi));
    CHECK(math::fast_asin(2.0) == Catch::Approx(0.5 * constants::pi));
}

TEST_CASE("LOLA data: the lunar height by direction equals the height by latitude / longitude",
          "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    double worst = 0.0;
    for (int i = 0; i < 20000; ++i) {
        double lat, lon;
        if (i % 2 == 0) {  // everywhere, the poles included
            lat = std::asin(2.0 * u01(rng) - 1.0);
            lon = (2.0 * u01(rng) - 1.0) * constants::pi;
        } else {  // in and around the IM-2 tile (20 km square, 1 km blend at its edge)
            std::tie(lat, lon) = destination(site_lat, site_lon, 2.0 * constants::pi * u01(rng), 14e3 * u01(rng) / R);
        }
        worst = std::max(worst, std::abs(terrain->height_at_direction_m(unit(lat, lon)) - terrain->height_m(lat, lon)));
    }
    CHECK(worst < 0.1);  // 2e-8 rad of fast_atan2 is 3.5 cm of position
}

// ---- Seams at polar strides, first step at a site tile's edge ----

namespace {

// Transposed copy of a lattice (vertex (i, j) -> (j, i)); reversed: the new rows run backwards (i -> 64 - i).
NodeLattice transposed(const NodeLattice& a, bool reversed) {
    NodeLattice t;
    for (int i = 0; i < t.n; ++i) {
        for (int j = 0; j < t.n; ++j) {
            const size_t from = static_cast<size_t>(j) * 65 + static_cast<size_t>(reversed ? 64 - i : i);
            t.lat.push_back(a.lat[from]);
            t.lon.push_back(a.lon[from]);
            t.h.push_back(a.h[from]);
        }
    }
    return t;
}

}  // namespace

TEST_CASE("Banded lattices straddling a polar stride threshold agree along the shared edge", "[terrain_horizon]") {
    // 333 m spacing at -86 deg: the local north turns 0.18 deg per vertex. The lattice poleward of the shared row (A)
    // needs smaller strides than the one equatorward (B), but the shared row must come out identical: as a row edge,
    // as a column edge (transposed lattice) and in reversed orientation.
    constexpr double spacing_m = 333.0;
    const auto [mlat, mlon] = destination(-86.0 * deg, 0.3, 1.0, 60e3 / R);
    const GaussianMountain mountain(mlat, mlon, 3000.0, 10e3);
    const NodeLattice a = make_lattice(mountain, -86.0 * deg, 0.3, spacing_m, -32, -64);  // rows poleward of the centre
    const NodeLattice b = make_lattice(mountain, -86.0 * deg, 0.3, spacing_m, -32, 0);    // B's row 0 is A's row 64
    const auto ha = environment::horizon_elevations_lattice(mountain, R, 65, a.lat, a.lon, a.h, spacing_m, 16);
    const auto hb = environment::horizon_elevations_lattice(mountain, R, 65, b.lat, b.lon, b.h, spacing_m, 16);
    int differing = 0;
    for (size_t j = 0; j < 65; ++j) {
        for (size_t k = 0; k < 16; ++k) differing += ha[(64 * 65 + j) * 16 + k] != hb[j * 16 + k] ? 1 : 0;
    }
    CHECK(differing == 0);
    for (const bool reversed : {false, true}) {
        const NodeLattice bt = transposed(b, reversed);  // the shared row is now column 0
        const auto hbt = environment::horizon_elevations_lattice(mountain, R, 65, bt.lat, bt.lon, bt.h, spacing_m, 16);
        int differing_t = 0;
        for (size_t j = 0; j < 65; ++j) {
            const size_t i = reversed ? 64 - j : j;
            for (size_t k = 0; k < 16; ++k) differing_t += ha[(64 * 65 + j) * 16 + k] != hbt[i * 65 * 16 + k] ? 1 : 0;
        }
        INFO("reversed " << reversed);
        CHECK(differing_t == 0);
    }
}

TEST_CASE("The first step reads the finest data nearby; a march needs a positive first step", "[terrain_horizon]") {
    const FlatTerrain flat(0.0);
    CHECK(flat.finest_resolution_within_m(0.3, 0.4, 1e6) == 100.0);  // the default: resolution_m at the point
    CHECK(environment::lattice_first_step_m(flat, 0.3, 0.4, 10.0) == 50.0);
    CHECK_THROWS_AS(environment::horizon_elevation_rad(flat, R, 0.0, 0.0, 1.0, 0.0, HorizonMarch{0.0, 0.0}),
                    std::invalid_argument);
    CHECK_THROWS_AS(environment::horizon_elevations(flat, R, {0.0}, {0.0}, {1.0}, 4, HorizonMarch{10.0, -1.0}),
                    std::invalid_argument);
}

namespace {

using Point = std::pair<double, double>;  // latitude, longitude (rad)

// Where the IM-2 tile ends along `azimuth` from the site: the distance from the site (m), bisected to 1 m.
double tile_edge_distance_m(const environment::LunarTerrain& terrain, double azimuth) {
    double lo = 0.0, hi = 100e3;
    while (hi - lo > 1.0) {
        const double mid = 0.5 * (lo + hi);
        const auto [la, lon] = destination(site_lat, site_lon, azimuth, mid / R);
        (terrain.in_tile(la, lon) ? lo : hi) = mid;
    }
    return lo;
}

// The point `metres` outside (negative: inside) the tile's edge along `azimuth` from the site.
Point at_edge(const environment::LunarTerrain& terrain, double azimuth, double metres) {
    return destination(site_lat, site_lon, azimuth, (tile_edge_distance_m(terrain, azimuth) + metres) / R);
}

}  // namespace

TEST_CASE("LOLA data: terrain queries find the tile within a radius", "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    const double global_m = terrain->resolution_m(0.0, 1.0);
    for (const double azimuth : {0.3, 2.0, 3.5, 5.0}) {
        const auto inside = at_edge(*terrain, azimuth, -5.0);
        CHECK(terrain->finest_resolution_within_m(inside.first, inside.second, 1.0) == Catch::Approx(20.0));
        for (const double d : {10.0, 500.0, 5000.0, 40e3}) {
            const auto p = at_edge(*terrain, azimuth, d);
            INFO("azimuth " << azimuth << " distance outside " << d);
            CHECK(terrain->finest_resolution_within_m(p.first, p.second, environment::first_step_data_range_m) ==
                  Catch::Approx(20.0));
            CHECK(terrain->finest_resolution_within_m(p.first, p.second, 0.5 * d) == Catch::Approx(global_m));
            CHECK(terrain->finest_resolution_within_m(p.first, p.second, 1.5 * d) == Catch::Approx(20.0));
        }
        const auto far = at_edge(*terrain, azimuth, 80e3);
        CHECK(terrain->finest_resolution_within_m(far.first, far.second, environment::first_step_data_range_m) ==
              Catch::Approx(global_m));
    }
}

TEST_CASE("LOLA data: horizons just outside and inside the tile edge match the brute force",
          "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    int compared = 0;
    double worst_rad = 0.0;
    for (const double azimuth : {0.3, 2.0, 3.5, 5.0}) {
        for (const double d : {-200.0, -20.0, 20.0, 200.0}) {  // metres from the edge, + outside
            const auto [la, lo] = at_edge(*terrain, azimuth, d);
            const double h = terrain->height_m(la, lo) + 2.0;
            const HorizonMarch march{10.0, environment::lattice_first_step_m(*terrain, la, lo, 10.0)};
            for (int k = 0; k < 16; ++k) {
                const double az = k * 2.0 * constants::pi / 16.0;
                const double marched = environment::horizon_elevation_rad(*terrain, R, la, lo, h, az, march);
                const double brute = brute_force_elevation_rad(*terrain, la, lo, h, az);
                INFO("edge azimuth " << azimuth << " d " << d << " ray " << k << " marched " << marched / deg
                                     << " brute " << brute / deg);
                CHECK(marched - brute <= tolerance_rad);
                if (brute < 5.0 * deg) {
                    CHECK(std::abs(marched - brute) <= tolerance_rad);
                    worst_rad = std::max(worst_rad, std::abs(marched - brute));
                    ++compared;
                }
            }
        }
    }
    std::printf("[terrain_horizon] tile edge: %d rays compared, worst |march - brute| = %.4f deg\n", compared,
                worst_rad / deg);
    CHECK(compared > 100);
}

TEST_CASE("LOLA data: the horizon is continuous across the tile edge", "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    constexpr double spacing_m = 10.0;
    const auto horizon = [&](Point p, int k) {
        const double h = terrain->height_m(p.first, p.second) + 2.0;
        const HorizonMarch march{spacing_m, environment::lattice_first_step_m(*terrain, p.first, p.second, spacing_m)};
        return environment::horizon_elevation_rad(*terrain, R, p.first, p.second, h, k * 2.0 * constants::pi / 16.0,
                                                  march);
    };
    for (const double azimuth : {0.3, 2.0, 3.5, 5.0}) {
        double across = 0.0, within = 0.0;
        for (int k = 0; k < 16; ++k) {
            // Two points one spacing apart: straddling the edge, and (as the reference) well inside the tile.
            across = std::max(across, std::abs(horizon(at_edge(*terrain, azimuth, 0.5 * spacing_m), k) -
                                               horizon(at_edge(*terrain, azimuth, -0.5 * spacing_m), k)));
            for (const double d : {-3000.0, -5000.0}) {
                within = std::max(within, std::abs(horizon(at_edge(*terrain, azimuth, d), k) -
                                                   horizon(at_edge(*terrain, azimuth, d - spacing_m), k)));
            }
        }
        std::printf("[terrain_horizon] tile edge azimuth %.1f: across %.4f deg, inside %.4f deg\n", azimuth,
                    across / deg, within / deg);
        CHECK(across <= within + 0.02 * deg);
    }
}

TEST_CASE("LOLA data: banded lattice timing of a leaf node straddling the tile edge (printed)",
          "[terrain_horizon][data]") {
    const auto terrain = lunar_terrain_or_skip();
    constexpr double spacing_m = 10.4;
    const auto [lat, lon] = at_edge(*terrain, 0.3, 0.0);
    const NodeLattice node = make_lattice(*terrain, lat, lon, spacing_m);
    const auto t0 = std::chrono::steady_clock::now();
    const auto out = environment::horizon_elevations_lattice(*terrain, R, node.n, node.lat, node.lon, node.h,
                                                             spacing_m, 16);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::printf("[terrain_horizon] banded timing leaf node at the tile edge: 65 x 65 x 16: %.1f ms\n", ms);
    CHECK(out.size() == 65u * 65u * 16u);
}
