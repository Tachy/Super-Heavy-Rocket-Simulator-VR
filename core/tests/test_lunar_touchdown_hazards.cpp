// Touchdown selection on the composed Moon surface (Task 5 of the Moon procedural surface plan): the footpads' step
// spread on the pivoting-pad support, the slope over the feet and the rock tops under the bells.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/config/moon_surface_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/detailed_lunar_terrain.hpp"
#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/sim/terrain_cache.hpp"

using namespace starsim;
using environment::DetailedLunarTerrain;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double R = 1737400.0;
constexpr double site_lat = -84.79 * deg, site_lon = 29.2 * deg;
constexpr double footprint_m = 12.0;  // the HLS legs (config: footprint_radius_m)

Vec3 direction(double lat, double lon) {
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

// Flat LOLA (global grid and a 20 m tile around the site, all zero) with the procedural detail and rocks on it: the
// relief and the rocks are the only structure (as test_detailed_lunar_terrain.cpp).
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

std::shared_ptr<const DetailedLunarTerrain> flat_surface() {
    static const auto s = [] {
        environment::CylindricalGrid g;
        g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, 0)};
        g.pixels_per_degree = 16.0;
        g.zero_line = 1439.5;
        g.zero_sample = -0.5;
        auto lola = std::make_shared<const environment::LunarTerrain>(std::move(g), std::vector{site_tile()}, R);
        auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
        cfg.rocks.abundance_source = "uniform";
        auto detail = std::make_shared<const environment::LunarSurfaceDetail>(cfg, lola);
        auto rocks = std::make_shared<const environment::LunarRocks>(detail);
        return std::make_shared<const DetailedLunarTerrain>(lola, detail, rocks, environment::SurfaceDetailOptions{});
    }();
    return s;
}

const config::StageConfig& hls_stage() {
    static const auto v = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_hls");
    return v.stages.front();
}

// East / north offset (m) of `unit` from (lat, lon) on the tangent plane.
std::pair<double, double> east_north(double lat, double lon, const Vec3& unit) {
    const Vec3 up = direction(lat, lon);
    const Vec3 east{-std::sin(lon), std::cos(lon), 0.0};
    const Vec3 north = cross(up, east);
    const Vec3 d = unit * R - up * R;
    return {dot(d, east), dot(d, north)};
}

// Independent check of a selected point at one roll: the six feet on support_under_disc, their plane's slope and
// peak-to-peak spread (heights along the point's vertical).
struct FeetCheck {
    double slope_rad, spread_m;
};
FeetCheck feet_at_roll(const DetailedLunarTerrain& s, const mission::TouchdownPoint& p, double roll_rad,
                       const mission::TouchdownHazardLimits& h) {
    const Vec3 up = direction(p.latitude_rad, p.longitude_rad);
    const Vec3 east{-std::sin(p.longitude_rad), std::cos(p.longitude_rad), 0.0};
    const Vec3 north = cross(up, east);
    const Vec3 origin = up * (R + p.height_m);
    std::vector<Vec3> pts;
    for (const double az : h.leg_azimuths_rad) {
        const double e = footprint_m * std::sin(az + roll_rad), n = footprint_m * std::cos(az + roll_rad);
        const Vec3 u = math::normalized(origin + east * e + north * n);
        pts.push_back({e, n, dot(u * (R + s.support_under_disc(u, up, h.foot_pad_radius_m).height_m) - origin, up)});
    }
    // Evenly spaced legs: the plane's normal equations decouple.
    double sez = 0, snz = 0, see = 0, snn = 0, sz = 0;
    for (const auto& q : pts) sez += q.x * q.z, snz += q.y * q.z, see += q.x * q.x, snn += q.y * q.y, sz += q.z;
    const double a = sez / see, b = snz / snn, c = sz / pts.size();
    double lo = 1e9, hi = -1e9;
    for (const auto& q : pts) {
        lo = std::min(lo, q.z - (a * q.x + b * q.y + c));
        hi = std::max(hi, q.z - (a * q.x + b * q.y + c));
    }
    return {std::atan(std::hypot(a, b)), hi - lo};
}

}  // namespace

TEST_CASE("The hazard limits follow from the HLS config", "[moon_surface][lunar][mission]") {
    const auto& stage = hls_stage();
    const auto h = mission::touchdown_hazard_limits(stage);
    CHECK(h.foot_pad_radius_m == stage.landing_legs.foot_pad_radius_m);
    CHECK(h.foot_pad_radius_m == 1.0);
    CHECK(h.max_foot_step_m == stage.landing_legs.stroke_m);                       // 0.6 m
    CHECK(h.bell_clearance_m == Catch::Approx(0.0 - (-1.5) - 0.6).margin(1e-12));  // exits flush at station 0
    CHECK(h.base_radius_m == 4.5);
    CHECK(h.leg_azimuths_rad.size() == 6);
    config::StageConfig no_legs = stage;
    no_legs.landing_legs.foot_pad_radius_m = 0.0;
    CHECK_THROWS_AS(mission::touchdown_hazard_limits(no_legs), std::invalid_argument);
}

TEST_CASE("A rock under a planned footpad moves the target to the nearest safe spot", "[moon_surface][lunar][mission]") {
    const auto s = flat_surface();
    const auto h = mission::touchdown_hazard_limits(hls_stage());
    const mission::LunarDescentParameters p;
    // The rock standing highest above its base within 400 m of the site (deterministic): its proud height bound.
    const Vec3 site = direction(site_lat, site_lon);
    const auto rocks = s->rocks().rocks_near(site, 400.0, 1.0);
    REQUIRE_FALSE(rocks.empty());
    const auto proud = [](const environment::Rock& r) { return r.axes_m.z - r.burial_m; };
    const auto rock = *std::max_element(rocks.begin(), rocks.end(),
                                        [&](const auto& a, const auto& b) { return proud(a) < proud(b); });
    const double rock_lat = std::asin(rock.base_unit.z), rock_lon = std::atan2(rock.base_unit.y, rock.base_unit.x);
    // The target 12 m east of the rock: one foot of the roll with a leg towards the west stands on it.
    const double target_lat = rock_lat, target_lon = rock_lon + footprint_m / (R * std::cos(rock_lat));
    const auto top = s->support_under_disc(rock.base_unit, rock.base_unit, h.foot_pad_radius_m).height_m;
    const double ground = s->contact_ground_m(rock.base_unit);
    std::printf("[touchdown_hazards] rock D %.2f m, proud bound %.2f m, pad support %.2f m above the ground\n",
                rock.diameter_m, proud(rock), top - ground);
    REQUIRE(top - ground > h.max_foot_step_m);  // the rock alone exceeds the step spread

    // LOLA (flat) alone keeps the target.
    const auto lola_only = mission::select_touchdown_point(s->lola(), target_lat, target_lon, 0.0, footprint_m, p);
    REQUIRE(lola_only);
    CHECK(lola_only->offset_m == 0.0);

    const auto t0 = std::chrono::steady_clock::now();
    const auto point = mission::select_touchdown_point(*s, target_lat, target_lon, 0.0, footprint_m, p, h);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    REQUIRE(point);
    std::printf("[touchdown_hazards] synthetic: offset %.1f m, slope %.2f deg, step %.3f m, bell clearance %.3f m, "
                "%.1f ms\n",
                point->offset_m, point->slope_rad / deg, point->max_foot_step_m, point->min_bell_clearance_m, ms);
    CHECK(point->offset_m > 0.0);
    CHECK(point->offset_m <= p.touchdown_search_radius_m);
    CHECK(point->slope_rad <= p.slope_limit_rad);
    CHECK(point->max_foot_step_m <= h.max_foot_step_m);
    CHECK(point->min_bell_clearance_m >= 0.0);
    // The nearest safe spot: the grid's next ring (20 m) or the one after it, not farther, on a flat field.
    CHECK(point->offset_m <= 2.0 * std::sqrt(2.0) * p.touchdown_grid_m + 1e-9);
    // Independent check at rolls between the sampled ones (the feet on the pad support, as the leg contact).
    for (const double roll : {0.0, 7.0 * deg, 23.0 * deg, 41.0 * deg}) {
        const auto c = feet_at_roll(*s, *point, roll, h);
        CAPTURE(roll);
        CHECK(c.slope_rad <= p.slope_limit_rad);
        CHECK(c.spread_m <= h.max_foot_step_m);
    }
}

TEST_CASE("Detail off selects the same point as before", "[moon_surface][lunar][mission][terrain][data]") {
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2");
    const auto lola = sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR);
    const auto off = sim::shared_moon_surface(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, {false, false});
    const auto h = mission::touchdown_hazard_limits(hls_stage());
    const mission::LunarDescentParameters p;
    for (const double az : {0.0, 180.0 * deg}) {
        const auto before = mission::select_touchdown_point(*lola, site.latitude_rad, site.longitude_rad, az,
                                                            footprint_m, p);
        const auto now = mission::select_touchdown_point(*off, site.latitude_rad, site.longitude_rad, az, footprint_m,
                                                         p, h);
        REQUIRE(before);
        REQUIRE(now);
        CHECK(now->latitude_rad == before->latitude_rad);
        CHECK(now->longitude_rad == before->longitude_rad);
        CHECK(now->height_m == before->height_m);
        CHECK(now->slope_rad == before->slope_rad);
        CHECK(now->offset_m == before->offset_m);
        CHECK(std::isnan(now->max_foot_step_m));
        CHECK(std::isnan(now->min_bell_clearance_m));
    }
}

TEST_CASE("The IM-2 site with full detail has a safe point within the search radius",
          "[moon_surface][lunar][mission][terrain][data]") {
    const std::filesystem::path data = STARSIM_DATA_DIR;
    if (!std::filesystem::exists(data / "terrain" / "diviner_rock_abundance.i16")) SKIP("Diviner data not installed");
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2");
    const auto surface = sim::shared_moon_surface(STARSIM_CONFIG_DIR, data, {true, true});
    const auto h = mission::touchdown_hazard_limits(hls_stage());
    const mission::LunarDescentParameters p;
    for (const double az : {180.0 * deg, 0.0}) {
        const auto lola = mission::select_touchdown_point(surface->lola(), site.latitude_rad, site.longitude_rad, az,
                                                          footprint_m, p);
        const auto t0 = std::chrono::steady_clock::now();
        const auto point = mission::select_touchdown_point(*surface, site.latitude_rad, site.longitude_rad, az,
                                                           footprint_m, p, h);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        REQUIRE(lola);
        REQUIRE(point);
        const auto [e, n] = east_north(lola->latitude_rad, lola->longitude_rad,
                                       direction(point->latitude_rad, point->longitude_rad));
        std::printf("[touchdown_hazards] IM-2 approach %.0f deg: offset %.1f m from the site (LOLA only %.1f m, %.1f m "
                    "apart), slope %.2f deg (LOLA %.2f), height %.3f m (LOLA %.3f), step %.3f m, bell clearance %.3f m, "
                    "%.1f ms\n",
                    az / deg, point->offset_m, lola->offset_m, std::hypot(e, n), point->slope_rad / deg,
                    lola->slope_rad / deg, point->height_m, lola->height_m, point->max_foot_step_m,
                    point->min_bell_clearance_m, ms);
        CHECK(point->offset_m <= p.touchdown_search_radius_m);
        CHECK(point->offset_m >= lola->offset_m);
        CHECK(point->slope_rad <= p.slope_limit_rad);
        CHECK(point->max_foot_step_m <= h.max_foot_step_m);
        CHECK(point->min_bell_clearance_m >= 0.0);
    }
}
