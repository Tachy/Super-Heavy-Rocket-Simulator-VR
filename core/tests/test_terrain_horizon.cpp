// Terrain horizon (terrain shadows): the exact smooth-sphere dip, an analytic Gaussian mountain, the real LDEM_16 /
// IM-2 tile against a brute-force march, poles and the antimeridian, and the march's timing.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <random>
#include <tuple>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/terrain_horizon.hpp"
#include "starsim/math/constants.hpp"
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
    print_timing(*terrain, 373.0, {373.0, 373.0}, "level 7");
}
