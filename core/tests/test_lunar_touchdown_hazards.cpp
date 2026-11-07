// Touchdown selection on the composed Moon surface (Task 5 of the Moon procedural surface plan): the footpads' step
// spread on the pivoting-pad support, the slope over the feet and the rock tops under the bells.

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
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
#include "starsim/environment/lunar_tile.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/touchdown_search.hpp"
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

// Independent check of a selected point at one roll: the six feet on support_under_disc (heights along the point's
// vertical) and the plane the hull rests on, found as the supporting plane (every foot on or below it) of a foot
// triangle containing the centre: its slope and the deepest foot below it.
struct FeetCheck {
    double slope_rad, hang_m;
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
    struct Supporting {
        double c, slope, hang;
    };
    std::vector<Supporting> supporting;
    for (size_t i = 0; i < pts.size(); ++i) {
        for (size_t j = i + 1; j < pts.size(); ++j) {
            for (size_t k = j + 1; k < pts.size(); ++k) {
                const Vec3 nrm = cross(pts[j] - pts[i], pts[k] - pts[i]);
                if (std::abs(nrm.z) < 1e-9) continue;
                const double a = -nrm.x / nrm.z, b = -nrm.y / nrm.z, c = pts[i].z - a * pts[i].x - b * pts[i].y;
                double hang = 0.0;
                bool all_below = true;
                for (const auto& q : pts) {
                    all_below = all_below && q.z <= a * q.x + b * q.y + c + 1e-9;
                    hang = std::max(hang, a * q.x + b * q.y + c - q.z);
                }
                const auto side = [](const Vec3& p0, const Vec3& p1) { return p0.x * p1.y - p0.y * p1.x; };
                const double s1 = side(pts[i], pts[j]), s2 = side(pts[j], pts[k]), s3 = side(pts[k], pts[i]);
                const bool inside = (s1 >= 0 && s2 >= 0 && s3 >= 0) || (s1 <= 0 && s2 <= 0 && s3 <= 0);
                if (all_below && inside) supporting.push_back({c, std::atan(std::hypot(a, b)), hang});
            }
        }
    }
    // the highest supporting planes at the centre (two when it balances on opposite feet): the worst of them
    double top = -1e9;
    for (const auto& q : supporting) top = std::max(top, q.c);
    FeetCheck worst{0.0, 0.0};
    for (const auto& q : supporting) {
        if (q.c < top - 1e-9) continue;
        worst.slope_rad = std::max(worst.slope_rad, q.slope);
        worst.hang_m = std::max(worst.hang_m, q.hang);
    }
    return worst;
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

// Fix round 1 (review I2): the hull rests on the facet of the feet's upper convex hull under its axis, not on their
// least-squares plane. Six HLS feet on a 7 deg plane, the foot farthest uphill raised by 0.5 m (within the 0.6 m step):
// the least-squares plane stays below the 8 deg limit, the rest plane through the raised foot exceeds it.
TEST_CASE("The rest plane is the feet's upper hull facet under the axis", "[moon_surface][lunar][mission]") {
    const auto h = mission::touchdown_hazard_limits(hls_stage());
    const double tan7 = std::tan(7.0 * deg);
    std::vector<Vec3> pts;
    for (const double az : h.leg_azimuths_rad) {
        const double e = footprint_m * std::sin(az), n = footprint_m * std::cos(az);
        pts.push_back({e, n, tan7 * n});
    }
    const auto uphill = std::max_element(pts.begin(), pts.end(), [](const Vec3& a, const Vec3& b) { return a.y < b.y; });
    uphill->z += 0.5;
    const auto facets = mission::touchdown_rest_planes(pts);
    REQUIRE_FALSE(facets.empty());
    // Independent: the supporting planes (every foot on or below) of the foot triangles that contain the centre, the
    // highest at the centre.
    struct Supporting {
        double a, b, c;
    };
    std::vector<Supporting> supporting;
    for (size_t i = 0; i < pts.size(); ++i) {
        for (size_t j = i + 1; j < pts.size(); ++j) {
            for (size_t k = j + 1; k < pts.size(); ++k) {
                const Vec3 nrm = cross(pts[j] - pts[i], pts[k] - pts[i]);
                if (std::abs(nrm.z) < 1e-9) continue;
                const double a = -nrm.x / nrm.z, b = -nrm.y / nrm.z, c = pts[i].z - a * pts[i].x - b * pts[i].y;
                bool all_below = true;
                for (const auto& q : pts) all_below = all_below && q.z <= a * q.x + b * q.y + c + 1e-9;
                // the centre inside the triangle (or on an edge): on the same side of all three edges
                const auto side = [](const Vec3& p0, const Vec3& p1) { return p0.x * p1.y - p0.y * p1.x; };
                const double s1 = side(pts[i], pts[j]), s2 = side(pts[j], pts[k]), s3 = side(pts[k], pts[i]);
                const double eps = 1e-9;
                const bool inside = (s1 >= -eps && s2 >= -eps && s3 >= -eps) || (s1 <= eps && s2 <= eps && s3 <= eps);
                if (all_below && inside) supporting.push_back({a, b, c});
            }
        }
    }
    double top = -1e9;
    for (const auto& q : supporting) top = std::max(top, q.c);
    std::vector<Supporting> expected;
    for (const auto& q : supporting) {
        if (q.c < top - 1e-9) continue;
        const bool dup = std::any_of(expected.begin(), expected.end(), [&](const Supporting& e) {
            return std::abs(e.a - q.a) < 1e-9 && std::abs(e.b - q.b) < 1e-9;
        });
        if (!dup) expected.push_back(q);
    }
    REQUIRE(facets.size() == expected.size());
    double max_slope = 0.0, max_hang = 0.0;
    for (const auto& f : facets) {
        const bool match = std::any_of(expected.begin(), expected.end(), [&](const Supporting& e) {
            return std::abs(e.a - f.a) < 1e-12 && std::abs(e.b - f.b) < 1e-12 && std::abs(e.c - f.c) < 1e-12;
        });
        CHECK(match);
        double hang = 0.0;
        for (const auto& q : pts) hang = std::max(hang, f.at(q.x, q.y) - q.z);
        CHECK(f.hang_m == Catch::Approx(hang).margin(1e-12));
        max_slope = std::max(max_slope, f.slope_rad());
        max_hang = std::max(max_hang, f.hang_m);
    }
    double sez = 0, snz = 0, see = 0, snn = 0;
    for (const auto& q : pts) sez += q.x * q.z, snz += q.y * q.z, see += q.x * q.x, snn += q.y * q.y;
    const double lsq_slope = std::atan(std::hypot(sez / see, snz / snn));
    const mission::LunarDescentParameters p;
    std::printf("[touchdown_hazards] %zu rest facet(s): steepest %.2f deg, least-squares %.2f deg, deepest hang %.3f m\n",
                facets.size(), max_slope / deg, lsq_slope / deg, max_hang);
    CHECK(lsq_slope < p.slope_limit_rad);
    CHECK(max_slope > p.slope_limit_rad);
    CHECK(max_hang > 0.0);
    CHECK(max_hang < h.max_foot_step_m);

    // On a plane the rest plane is that plane (one facet, however many triangles span it); nothing hangs.
    std::vector<Vec3> flat;
    for (const auto& q : pts) flat.push_back({q.x, q.y, 0.1 * q.x + 0.05 * q.y + 1.0});
    const auto on_plane = mission::touchdown_rest_planes(flat);
    REQUIRE(on_plane.size() == 1);
    CHECK(on_plane[0].a == Catch::Approx(0.1).margin(1e-12));
    CHECK(on_plane[0].b == Catch::Approx(0.05).margin(1e-12));
    CHECK(on_plane[0].c == Catch::Approx(1.0).margin(1e-12));
    CHECK(on_plane[0].hang_m < 1e-12);
    // Feet all on one side of the axis: no rest plane (the hull tips).
    CHECK(mission::touchdown_rest_planes({{3.0, 1.0, 0.0}, {4.0, -1.0, 0.0}, {5.0, 0.0, 0.0}}).empty());
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
    // The nearest safe spot (review M2): no grid candidate nearer than the result passes - the search limited to just
    // inside its offset finds nothing.
    auto nearer = p;
    nearer.touchdown_search_radius_m = point->offset_m - 1e-6;
    CHECK_FALSE(mission::select_touchdown_point(*s, target_lat, target_lon, 0.0, footprint_m, nearer, h));
    // Independent check at rolls between the sampled ones (the feet on the pad support, as the leg contact).
    for (const double roll : {0.0, 7.0 * deg, 23.0 * deg, 41.0 * deg}) {
        const auto c = feet_at_roll(*s, *point, roll, h);
        CAPTURE(roll);
        CHECK(c.slope_rad <= p.slope_limit_rad);
        CHECK(c.hang_m <= h.max_foot_step_m);
    }
}

// Review I3': the candidates are checked ring by ring in parallel. The chosen point must be the serial one bit for bit
// and the concurrent checks (cold surface cells built on several threads at once) must give the serial values. Each
// search runs on a fresh memo (a search with other limits replaces the memo's context first), so every candidate is
// really checked on the threads.
TEST_CASE("The parallel touchdown search picks the serial point bit for bit", "[moon_surface][lunar][mission]") {
    const auto s = flat_surface();
    const auto base = mission::touchdown_hazard_limits(hls_stage());
    const auto rocks = s->rocks().rocks_near(direction(site_lat, site_lon), 400.0, 1.0);
    REQUIRE_FALSE(rocks.empty());
    const auto proud = [](const environment::Rock& r) { return r.axes_m.z - r.burial_m; };
    const auto rock = *std::max_element(rocks.begin(), rocks.end(),
                                        [&](const auto& a, const auto& b) { return proud(a) < proud(b); });
    const double rock_lat = std::asin(rock.base_unit.z), rock_lon = std::atan2(rock.base_unit.y, rock.base_unit.x);
    const double target_lat = rock_lat, target_lon = rock_lon + footprint_m / (R * std::cos(rock_lat));

    const auto fresh_search = [&](const mission::TouchdownHazardLimits& h, mission::LunarDescentParameters p,
                                  int threads) {
        auto other = h;
        other.base_radius_m += 1e-3;  // another memo context: the next search checks every candidate again
        auto tiny = p;
        tiny.touchdown_search_radius_m = 0.0;
        (void)mission::select_touchdown_point(*s, target_lat, target_lon, 0.0, footprint_m, tiny, other);
        p.touchdown_search_threads = threads;
        return mission::select_touchdown_point(*s, target_lat, target_lon, 0.0, footprint_m, p, h);
    };
    struct Variant {
        const char* name;
        double max_step_m, bell_clearance_m, radius_m;
    };
    for (const Variant v : {Variant{"HLS limits (point 60 m off)", base.max_foot_step_m, base.bell_clearance_m, 200.0},
                            Variant{"step 0.25 m", 0.25, base.bell_clearance_m, 200.0},
                            Variant{"refused (bell clearance -1 m)", base.max_foot_step_m, -1.0, 100.0}}) {
        auto h = base;
        h.max_foot_step_m = v.max_step_m;
        h.bell_clearance_m = v.bell_clearance_m;
        mission::LunarDescentParameters p;
        p.touchdown_search_radius_m = v.radius_m;
        const auto serial = fresh_search(h, p, 1);
        for (const int threads : {3, 0}) {
            const auto parallel = fresh_search(h, p, threads);
            CAPTURE(v.name, threads);
            std::printf("[touchdown_hazards] parallel search (%s, %d threads): %s\n", v.name, threads,
                        parallel ? "found" : "refused");
            REQUIRE(serial.has_value() == parallel.has_value());
            if (!serial) continue;
            CHECK(std::bit_cast<std::uint64_t>(parallel->latitude_rad) == std::bit_cast<std::uint64_t>(serial->latitude_rad));
            CHECK(std::bit_cast<std::uint64_t>(parallel->longitude_rad) ==
                  std::bit_cast<std::uint64_t>(serial->longitude_rad));
            CHECK(std::bit_cast<std::uint64_t>(parallel->height_m) == std::bit_cast<std::uint64_t>(serial->height_m));
            CHECK(std::bit_cast<std::uint64_t>(parallel->slope_rad) == std::bit_cast<std::uint64_t>(serial->slope_rad));
            CHECK(parallel->offset_m == serial->offset_m);
            CHECK(std::bit_cast<std::uint64_t>(parallel->max_foot_step_m) ==
                  std::bit_cast<std::uint64_t>(serial->max_foot_step_m));
            CHECK(std::bit_cast<std::uint64_t>(parallel->min_bell_clearance_m) ==
                  std::bit_cast<std::uint64_t>(serial->min_bell_clearance_m));
        }
    }
}

// Follow-up U-7: the search on a background thread gives the synchronous point bit for bit (each run on a fresh memo,
// so both really check the candidates), the first azimuth's point when it has one, and std::nullopt for a refused site.
TEST_CASE("The asynchronous touchdown search gives the synchronous point", "[moon_surface][lunar][mission]") {
    const auto s = flat_surface();
    const auto base = mission::touchdown_hazard_limits(hls_stage());
    const auto rocks = s->rocks().rocks_near(direction(site_lat, site_lon), 400.0, 1.0);
    REQUIRE_FALSE(rocks.empty());
    const auto proud = [](const environment::Rock& r) { return r.axes_m.z - r.burial_m; };
    const auto rock = *std::max_element(rocks.begin(), rocks.end(),
                                        [&](const auto& a, const auto& b) { return proud(a) < proud(b); });
    const double target_lat = std::asin(rock.base_unit.z);
    const double target_lon = std::atan2(rock.base_unit.y, rock.base_unit.x) + footprint_m / (R * std::cos(target_lat));
    const auto fresh_memo = [&](const mission::TouchdownHazardLimits& h) {
        auto other = h;
        other.base_radius_m += 1e-3;  // another memo context: the next search checks every candidate again
        mission::LunarDescentParameters tiny;
        tiny.touchdown_search_radius_m = 0.0;
        (void)mission::select_touchdown_point(*s, target_lat, target_lon, 0.0, footprint_m, tiny, other);
    };
    for (const bool refused : {false, true}) {
        auto h = base;
        if (refused) h.bell_clearance_m = -1.0;  // every candidate fails after its full roll sweep
        mission::LunarDescentParameters p;
        p.touchdown_search_radius_m = refused ? 100.0 : 200.0;
        fresh_memo(h);
        auto future = mission::select_touchdown_point_async(s, target_lat, target_lon, {0.0, 180.0 * deg}, footprint_m,
                                                            p, h);
        const auto async = future.get();
        fresh_memo(h);
        const auto sync = mission::select_touchdown_point(*s, target_lat, target_lon, 0.0, footprint_m, p, h);
        CAPTURE(refused);
        std::printf("[touchdown_hazards] asynchronous search (%s): %s\n", refused ? "refused site" : "rock",
                    async ? "found" : "refused");
        REQUIRE(async.has_value() == sync.has_value());
        CHECK(async.has_value() == !refused);
        if (!sync) continue;
        CHECK(std::bit_cast<std::uint64_t>(async->latitude_rad) == std::bit_cast<std::uint64_t>(sync->latitude_rad));
        CHECK(std::bit_cast<std::uint64_t>(async->longitude_rad) == std::bit_cast<std::uint64_t>(sync->longitude_rad));
        CHECK(std::bit_cast<std::uint64_t>(async->height_m) == std::bit_cast<std::uint64_t>(sync->height_m));
        CHECK(std::bit_cast<std::uint64_t>(async->slope_rad) == std::bit_cast<std::uint64_t>(sync->slope_rad));
        CHECK(async->offset_m == sync->offset_m);
        CHECK(std::bit_cast<std::uint64_t>(async->max_foot_step_m) == std::bit_cast<std::uint64_t>(sync->max_foot_step_m));
        CHECK(std::bit_cast<std::uint64_t>(async->min_bell_clearance_m) ==
              std::bit_cast<std::uint64_t>(sync->min_bell_clearance_m));
    }
    CHECK_THROWS_AS(mission::select_touchdown_point_async(nullptr, target_lat, target_lon, {0.0}, footprint_m,
                                                          mission::LunarDescentParameters{}, base),
                    std::invalid_argument);
}

TEST_CASE("Detail off selects the same point as before", "[moon_surface][lunar][mission][terrain][data]") {
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2");
    const auto lola = sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"));
    const auto off = sim::shared_moon_surface(STARSIM_CONFIG_DIR, STARSIM_DATA_DIR, {false, false}, config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"));
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
    const auto surface = sim::shared_moon_surface(STARSIM_CONFIG_DIR, data, {true, true}, config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"));
    const auto h = mission::touchdown_hazard_limits(hls_stage());
    mission::LunarDescentParameters p;
    if (const char* t = std::getenv("STARSIM_TOUCHDOWN_THREADS")) p.touchdown_search_threads = std::atoi(t);  // profiling
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

// Review I3: the cost of a refused site. Hidden (minutes): "[.touchdown_worst]". A bell clearance below zero refuses
// every candidate only after its full roll sweep and the base check - the most work a candidate can cost - so this is
// the worst case over the whole 1 km search at IM-2 (every LOLA-admissible candidate checked). A step limit of 1 cm
// is the cheap refusal (the bound or the first roll).
TEST_CASE("Worst-case time of a refused site at IM-2", "[.touchdown_worst]") {
    const std::filesystem::path data = STARSIM_DATA_DIR;
    const auto site = config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2");
    const auto surface = sim::shared_moon_surface(STARSIM_CONFIG_DIR, data, {true, true}, config::load_lunar_site(STARSIM_CONFIG_DIR, "mons_mouton_im2"));
    mission::LunarDescentParameters p;
    if (const char* r = std::getenv("STARSIM_TOUCHDOWN_RADIUS_M")) p.touchdown_search_radius_m = std::atof(r);  // profiling
    if (const char* t = std::getenv("STARSIM_TOUCHDOWN_THREADS")) p.touchdown_search_threads = std::atoi(t);  // profiling
    for (const int variant : {0, 1}) {
        auto h = mission::touchdown_hazard_limits(hls_stage());
        if (variant == 0) h.max_foot_step_m = 0.01;
        else h.bell_clearance_m = -1.0;
        for (int call = 0; call < 2; ++call) {
            const auto t0 = std::chrono::steady_clock::now();
            const auto point = mission::select_touchdown_point(*surface, site.latitude_rad, site.longitude_rad,
                                                               call == 0 ? 180.0 * deg : 0.0, footprint_m, p, h);
            const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            std::printf("[touchdown_hazards] refused site (%s), call %d: %s, %.2f s\n",
                        variant == 0 ? "step 1 cm" : "bell clearance -1 m", call + 1, point ? "found" : "refused", s);
            std::fflush(stdout);
            CHECK_FALSE(point);
        }
    }
}

// Flat touchdown point plan, Task 1: the search prefers a point within preferred_slope_rad anywhere in the radius.
namespace {

constexpr double tilt_lat = 0.7 * deg, tilt_lon = 23.5 * deg;  // an equatorial mare-like site

// An analytic LOLA tile around (tilt_lat, tilt_lon): a plane rising east at `slope_deg` up to `terrace_m` east of the
// site, level beyond it (the terrace). Constant along north (the approach's uprange stays level).
class TerraceTile final : public environment::LunarTile {
public:
    TerraceTile(double slope_deg, double terrace_m) : grade_(std::tan(slope_deg * deg)), terrace_m_(terrace_m) {}
    bool contains(const Vec3& u) const override { return distance_m(u) < radius_m; }
    double height_m(const Vec3& u) const override {
        const auto [e, n] = east_north(tilt_lat, tilt_lon, u);
        (void)n;
        return grade_ * std::min(e, terrace_m_);
    }
    double edge_distance_m(const Vec3& u) const override { return radius_m - distance_m(u); }
    double scale_m() const override { return 20.0; }
    double outside_distance_m(const Vec3& u) const override { return std::max(distance_m(u) - radius_m, 0.0); }
    double max_height_m() const override { return grade_ * std::min(radius_m, terrace_m_); }

private:
    static constexpr double radius_m = 6000.0;
    static double distance_m(const Vec3& u) {
        const auto [e, n] = east_north(tilt_lat, tilt_lon, u);
        return std::hypot(e, n);
    }
    double grade_, terrace_m_;
};

std::shared_ptr<const environment::LunarTerrain> terrace_lola(double slope_deg, double terrace_m) {
    environment::CylindricalGrid g;  // a coarse flat global grid (the tile carries the structure)
    g.grid = {180, 360, 0.5, std::vector<std::int16_t>(180 * 360, 0)};
    g.pixels_per_degree = 1.0;
    g.zero_line = 89.5;
    g.zero_sample = -0.5;
    return std::make_shared<const environment::LunarTerrain>(
        std::move(g), std::vector<std::shared_ptr<const environment::LunarTile>>{
                          std::make_shared<const TerraceTile>(slope_deg, terrace_m)}, R);
}

std::shared_ptr<const DetailedLunarTerrain> terrace_surface(double slope_deg, double terrace_m) {
    auto lola = terrace_lola(slope_deg, terrace_m);
    auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
    cfg.rocks.abundance_source = "uniform";
    auto detail = std::make_shared<const environment::LunarSurfaceDetail>(cfg, lola);
    auto rocks = std::make_shared<const environment::LunarRocks>(detail);
    return std::make_shared<const DetailedLunarTerrain>(lola, detail, rocks, environment::SurfaceDetailOptions{});
}

constexpr double terrace_east_m = 300.0;

}  // namespace

TEST_CASE("A flatter point farther out beats a steeper near one", "[moon_surface][lunar][mission]") {
    mission::LunarDescentParameters p;
    mission::LunarDescentParameters nearest = p;
    nearest.preferred_slope_rad = p.slope_limit_rad;  // every admissible point preferred: the old choice
    SECTION("LOLA only") {
        const auto lola = terrace_lola(7.0, terrace_east_m);
        const auto point = mission::select_touchdown_point(*lola, tilt_lat, tilt_lon, 0.0, footprint_m, p);
        const auto near = mission::select_touchdown_point(*lola, tilt_lat, tilt_lon, 0.0, footprint_m, nearest);
        REQUIRE(point);
        REQUIRE(near);
        std::printf("[touchdown_hazards] terrace LOLA: preferred %.0f m off at %.2f deg, nearest %.0f m off at %.2f deg\n",
                    point->offset_m, point->slope_rad / deg, near->offset_m, near->slope_rad / deg);
        CHECK(point->within_preferred_slope);
        CHECK(point->slope_rad <= p.preferred_slope_rad);
        CHECK(point->offset_m >= terrace_east_m - footprint_m - p.touchdown_grid_m);
        CHECK(near->offset_m == 0.0);
        CHECK(near->slope_rad > p.preferred_slope_rad);
        CHECK(near->within_preferred_slope);  // within its own (8 deg) preferred slope
        // No preferred candidate nearer than the result: the search limited to just inside its offset falls back.
        auto inside = p;
        inside.touchdown_search_radius_m = point->offset_m - 1e-6;
        const auto fallback = mission::select_touchdown_point(*lola, tilt_lat, tilt_lon, 0.0, footprint_m, inside);
        REQUIRE(fallback);
        CHECK_FALSE(fallback->within_preferred_slope);
        CHECK(fallback->offset_m == 0.0);
    }
    SECTION("composed surface with the hazard check") {
        const auto s = terrace_surface(6.5, terrace_east_m);
        const auto h = mission::touchdown_hazard_limits(hls_stage());
        const auto point = mission::select_touchdown_point(*s, tilt_lat, tilt_lon, 0.0, footprint_m, p, h);
        const auto near = mission::select_touchdown_point(*s, tilt_lat, tilt_lon, 0.0, footprint_m, nearest, h);
        REQUIRE(point);
        REQUIRE(near);
        std::printf("[touchdown_hazards] terrace detail: preferred %.0f m off at %.2f deg, nearest %.0f m off at %.2f deg\n",
                    point->offset_m, point->slope_rad / deg, near->offset_m, near->slope_rad / deg);
        CHECK(point->within_preferred_slope);
        CHECK(point->slope_rad <= p.preferred_slope_rad);
        CHECK(point->offset_m >= terrace_east_m - footprint_m - p.touchdown_grid_m);
        CHECK(near->offset_m < terrace_east_m - footprint_m - p.touchdown_grid_m);
        CHECK(near->slope_rad > p.preferred_slope_rad);
        // Independent check of the preferred point at rolls between the sampled ones.
        for (const double roll : {0.0, 7.0 * deg, 23.0 * deg, 41.0 * deg}) {
            CAPTURE(roll);
            CHECK(feet_at_roll(*s, *point, roll, h).slope_rad <= p.preferred_slope_rad + 0.5 * deg);
        }
    }
}

TEST_CASE("No point within the preferred slope falls back to the nearest admissible one",
          "[moon_surface][lunar][mission]") {
    const auto lola = terrace_lola(7.0, 1.0e9);  // 7 deg everywhere
    mission::LunarDescentParameters p;
    mission::LunarDescentParameters nearest = p;
    nearest.preferred_slope_rad = p.slope_limit_rad;
    const auto point = mission::select_touchdown_point(*lola, tilt_lat, tilt_lon, 0.0, footprint_m, p);
    const auto near = mission::select_touchdown_point(*lola, tilt_lat, tilt_lon, 0.0, footprint_m, nearest);
    REQUIRE(point);
    REQUIRE(near);
    CHECK_FALSE(point->within_preferred_slope);
    CHECK(point->slope_rad > p.preferred_slope_rad);
    CHECK(point->slope_rad <= p.slope_limit_rad);
    CHECK(point->latitude_rad == near->latitude_rad);
    CHECK(point->longitude_rad == near->longitude_rad);
    CHECK(point->offset_m == near->offset_m);
    // Steeper than the hard limit everywhere: refused as before.
    const auto steep = terrace_lola(9.0, 1.0e9);
    CHECK_FALSE(mission::select_touchdown_point(*steep, tilt_lat, tilt_lon, 0.0, footprint_m, p));
}

// Review Focus 4: the preferred point lies rings beyond the fallback; the parallel search must take the serial one.
TEST_CASE("The parallel search takes the serial preferred point on a terrace", "[moon_surface][lunar][mission]") {
    const auto s = terrace_surface(6.5, terrace_east_m);
    const auto h = mission::touchdown_hazard_limits(hls_stage());
    const auto run = [&](int threads) {
        auto other = h;
        other.base_radius_m += 1e-3;  // a fresh memo context: every candidate is checked on the threads
        mission::LunarDescentParameters tiny;
        tiny.touchdown_search_radius_m = 0.0;
        (void)mission::select_touchdown_point(*s, tilt_lat, tilt_lon, 0.0, footprint_m, tiny, other);
        mission::LunarDescentParameters p;
        p.touchdown_search_radius_m = 500.0;
        p.touchdown_search_threads = threads;
        return mission::select_touchdown_point(*s, tilt_lat, tilt_lon, 0.0, footprint_m, p, h);
    };
    const auto serial = run(1);
    REQUIRE(serial);
    CHECK(serial->within_preferred_slope);
    for (const int threads : {3, 0}) {
        CAPTURE(threads);
        const auto parallel = run(threads);
        REQUIRE(parallel);
        CHECK(std::bit_cast<std::uint64_t>(parallel->latitude_rad) == std::bit_cast<std::uint64_t>(serial->latitude_rad));
        CHECK(std::bit_cast<std::uint64_t>(parallel->longitude_rad) == std::bit_cast<std::uint64_t>(serial->longitude_rad));
        CHECK(std::bit_cast<std::uint64_t>(parallel->height_m) == std::bit_cast<std::uint64_t>(serial->height_m));
        CHECK(std::bit_cast<std::uint64_t>(parallel->slope_rad) == std::bit_cast<std::uint64_t>(serial->slope_rad));
        CHECK(parallel->offset_m == serial->offset_m);
        CHECK(parallel->within_preferred_slope == serial->within_preferred_slope);
    }
}
