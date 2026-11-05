// Composed physics surface of the Moon (Task 4 of the Moon procedural surface plan): LOLA + procedural craters,
// roughness and rocks, the footpad disc support and the fidelity flags.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <limits>
#include <memory>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/contact/landing_legs.hpp"
#include "starsim/contact/landing_surface.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/detailed_lunar_terrain.hpp"
#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/surface_hash.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/terrain_cache.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;
using environment::DetailedLunarTerrain;
using environment::Rock;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double R = 1737400.0;
constexpr double site_lat = -84.79 * deg, site_lon = 29.2 * deg;

Vec3 direction(double lat, double lon) {
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

double unit_hash(std::uint64_t a, std::uint64_t b) {
    return environment::hash_unit(environment::surface_hash(0x7A54, a, b, 0));
}

struct Tangent {
    Vec3 c, e1, e2;
};
Tangent tangent(const Vec3& c) {
    const Vec3 e1 = math::normalized(cross(std::abs(c.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, c));
    return {c, e1, cross(c, e1)};
}
Vec3 at(const Tangent& t, double x, double y) { return math::normalized(t.c + (t.e1 * x + t.e2 * y) / R); }

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

// Flat LOLA (global grid and a 20 m site tile, all zero): the relief and the rocks are the only structure.
std::shared_ptr<const environment::LunarTerrain> flat_lola() {
    static const auto t = [] {
        environment::CylindricalGrid g;
        g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, 0)};
        g.pixels_per_degree = 16.0;
        g.zero_line = 1439.5;
        g.zero_sample = -0.5;
        return std::make_shared<const environment::LunarTerrain>(std::move(g), std::vector{site_tile()}, R);
    }();
    return t;
}

std::shared_ptr<const environment::LunarSurfaceDetail> detail() {
    static const auto d = [] {
        auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
        cfg.rocks.abundance_source = "uniform";  // the site lies outside the Diviner map anyway
        return std::make_shared<const environment::LunarSurfaceDetail>(cfg, flat_lola());
    }();
    return d;
}

std::shared_ptr<const environment::LunarRocks> rocks() {
    static const auto r = std::make_shared<const environment::LunarRocks>(detail());
    return r;
}

std::shared_ptr<const DetailedLunarTerrain> surface() {
    static const auto s = std::make_shared<const DetailedLunarTerrain>(flat_lola(), detail(), rocks(),
                                                                       environment::SurfaceDetailOptions{});
    return s;
}

double angle_between(const Vec3& a, const Vec3& b) {
    return std::acos(std::clamp(dot(math::normalized(a), math::normalized(b)), -1.0, 1.0));
}

// LOLA with the site tile rising along its lines at slope_rad (50 counts per pixel: exactly linear in the tile; the
// pole-scale slope, 0.2 % steeper than the true one at the site's latitude).
std::shared_ptr<const environment::LunarTerrain> sloped_lola(double slope_rad) {
    environment::CylindricalGrid g;
    g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, 0)};
    g.pixels_per_degree = 16.0;
    g.zero_line = 1439.5;
    g.zero_sample = -0.5;
    auto tile = site_tile();
    tile.grid.dn_scale_m = std::tan(slope_rad) * tile.scale_m / 50.0;
    for (int l = 0; l < 401; ++l) {
        for (int s = 0; s < 401; ++s) tile.grid.dn[static_cast<size_t>(l * 401 + s)] = static_cast<std::int16_t>((l - 200) * 50);
    }
    return std::make_shared<const environment::LunarTerrain>(std::move(g), std::vector{tile}, R);
}

// The configured generator with neither craters nor roughness (the relief is exactly 0): LOLA and the rocks only.
std::shared_ptr<const environment::LunarSurfaceDetail> bare_detail(std::shared_ptr<const environment::LunarTerrain> lola) {
    auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
    cfg.rocks.abundance_source = "uniform";
    cfg.craters.sfd.density_per_m2 = 0.0;
    cfg.roughness.rms_height_at_baseline_m = 0.0;
    return std::make_shared<const environment::LunarSurfaceDetail>(cfg, std::move(lola));
}

// A plane over the disc in its tangent frame (heights referred to the disc centre's vertical).
struct Plane {
    double mean = 0.0, gx = 0.0, gy = 0.0;
    double at(double x, double y) const { return mean + gx * x + gy * y; }
};

// The pad's plane as specified: least squares through the contact ground at the centre, 8 points at half the radius
// and 16 on the rim, less the sphere's drop d^2 / 2R.
Plane pad_plane(const DetailedLunarTerrain& s, const Tangent& disc, double radius) {
    double sum = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0;
    int n = 0;
    const auto add = [&](double x, double y) {
        const double g = s.contact_ground_m(at(disc, x, y)) - (x * x + y * y) / (2.0 * R);
        sum += g, sx += g * x, sy += g * y, sxx += x * x, syy += y * y;
        ++n;
    };
    add(0.0, 0.0);
    for (const auto& [ring, rho] : {std::pair{8, 0.5 * radius}, std::pair{16, radius}}) {
        for (int k = 0; k < ring; ++k) {
            const double phi = constants::two_pi * k / ring;
            add(rho * std::cos(phi), rho * std::sin(phi));
        }
    }
    return {sum / n, sx / sxx, sy / syy};
}

// Highest protrusion of the contact ground above `plane` over the disc on a dense polar pattern.
double dense_ground_protrusion(const DetailedLunarTerrain& s, const Tangent& disc, double radius, const Plane& plane) {
    double best = -std::numeric_limits<double>::infinity();
    for (int i = 0; i <= 20; ++i) {
        const double rho = radius * i / 20.0;
        for (int k = 0; k < 40; ++k) {
            const double phi = constants::two_pi * k / 40.0, x = rho * std::cos(phi), y = rho * std::sin(phi);
            best = std::max(best, s.contact_ground_m(at(disc, x, y)) - (x * x + y * y) / (2.0 * R) - plane.at(x, y));
        }
    }
    return best;
}

// Brute-force highest protrusion of `rock`'s top above `plane` over the disc (centre, radius) in the tangent frame of
// the disc centre: a polar grid plus a dense rim. The default plane (0) gives the highest top.
struct Highest {
    double height_m = -std::numeric_limits<double>::infinity();
    Vec3 normal;
    Vec3 unit;  // where
};
Highest brute_rock_max(const environment::LunarRocks& lr, const Rock& rock, const Tangent& disc, double radius,
                       const Plane& plane = {}) {
    Highest best;
    const auto try_point = [&](double x, double y) {
        if (const auto top = lr.rock_top(rock, at(disc, x, y))) {
            const double h = top->first - (x * x + y * y) / (2.0 * R) - plane.at(x, y);
            if (h > best.height_m) best = {h, top->second, at(disc, x, y)};
        }
    };
    constexpr int rings = 100, per_ring = 240, rim = 8000;
    for (int i = 1; i <= rings; ++i) {
        const double rho = radius * i / rings;
        for (int k = 0; k < per_ring; ++k) {
            const double phi = constants::two_pi * (k + 0.5 * (i % 2)) / per_ring;
            try_point(rho * std::cos(phi), rho * std::sin(phi));
        }
    }
    for (int k = 0; k < rim; ++k) {
        const double phi = constants::two_pi * k / rim;
        try_point(radius * std::cos(phi), radius * std::sin(phi));
    }
    try_point(0.0, 0.0);
    return best;
}

// Rocks near `where` that stand well proud of their base (precondition of the disc tests).
std::vector<Rock> prominent_rocks(const environment::LunarRocks& lr, const Vec3& where) {
    std::vector<Rock> out;
    for (const Rock& r : lr.rocks_near(where, 60.0, 0.4)) {
        if (r.diameter_m > 1.5) continue;
        const double top = r.base_height_m + r.axes_m.z * environment::RockTemplates::instance().top_z(r.template_id) -
                           r.burial_m;
        if (top - r.base_height_m > 0.15) out.push_back(r);
    }
    return out;
}

// The rock's highest point (brute force over its footprint).
Vec3 rock_peak(const environment::LunarRocks& lr, const Rock& rock) {
    return brute_rock_max(lr, rock, tangent(rock.base_unit), rock.axes_m.x).unit;
}

// Unit tangent vector at `p` (perpendicular to p) in the direction of `v`.
Vec3 tangential(const Vec3& p, const Vec3& v) { return math::normalized(v - p * dot(v, p)); }

bool same(const Vec3& a, const Vec3& b) { return std::memcmp(&a, &b, sizeof(Vec3)) == 0; }

}  // namespace

TEST_CASE("Detail off reproduces LunarTerrain bit-for-bit", "[moon_surface][terrain]") {
    const std::filesystem::path data = STARSIM_DATA_DIR;
    if (!std::filesystem::exists(data / "terrain" / "ldem_16.i16")) SKIP("LOLA data not installed (fetch.py)");
    const auto lola = sim::shared_lunar_terrain(STARSIM_CONFIG_DIR, data);
    auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
    cfg.rocks.abundance_source = "uniform";
    const auto real_detail = std::make_shared<const environment::LunarSurfaceDetail>(cfg, lola);
    const DetailedLunarTerrain with_parts(lola, real_detail, std::make_shared<const environment::LunarRocks>(real_detail),
                                          {false, true});
    const DetailedLunarTerrain bare(lola, nullptr, nullptr, {false, false});
    CHECK(with_parts.max_height_m() == lola->max_height_m());
    CHECK(bare.max_height_m() == lola->max_height_m());
    int mismatches = 0;
    for (int k = 0; k < 10000; ++k) {
        Vec3 unit;
        if (k % 2 == 0) {  // within 15 km of the shipped site (the 20 m tile and its blend)
            const auto t = tangent(direction(-84.79 * deg, 29.2 * deg));
            unit = at(t, (unit_hash(1, k) - 0.5) * 30000.0, (unit_hash(2, k) - 0.5) * 30000.0);
        } else {  // anywhere
            const double z = 2.0 * unit_hash(3, k) - 1.0, phi = constants::two_pi * unit_hash(4, k);
            unit = {std::sqrt(1.0 - z * z) * std::cos(phi), std::sqrt(1.0 - z * z) * std::sin(phi), z};
        }
        const double lat = std::asin(unit.z), lon = std::atan2(unit.y, unit.x);
        const Vec3 fixed = unit * (R + 1234.5);
        for (const DetailedLunarTerrain* s : {&with_parts, &bare}) {
            const bool ok = s->height_m(lat, lon) == lola->height_m(lat, lon) &&
                            s->height_at_direction_m(unit) == lola->height_at_direction_m(unit) &&
                            s->height_at_fixed_m(fixed) == lola->height_at_fixed_m(fixed) &&
                            same(s->normal_fixed(fixed), lola->normal_fixed(fixed)) &&
                            s->resolution_m(lat, lon) == lola->resolution_m(lat, lon) &&
                            s->finest_resolution_within_m(lat, lon, 700.0) ==
                                lola->finest_resolution_within_m(lat, lon, 700.0) &&
                            s->normal_step_m(fixed) == environment::LunarTerrain::normal_step_m &&
                            s->height_limited_m(unit, 1.0, 0.1) == lola->height_at_direction_m(unit);
            // Point footpads: the disc is the point query at its centre.
            const auto disc = s->support_under_disc(unit, unit, 1.0);
            const bool disc_ok = disc.height_m == lola->height_at_fixed_m(unit * R) &&
                                 same(disc.normal_fixed, lola->normal_fixed(unit * R));
            if (!ok || !disc_ok) ++mismatches;
        }
    }
    CHECK(mismatches == 0);
}

TEST_CASE("A pivoting footpad on a 9 deg plane stands at the plane's height under its centre",
          "[moon_surface][terrain][contact]") {
    const auto lola = sloped_lola(9.0 * deg);
    const DetailedLunarTerrain ground(lola, bare_detail(lola), nullptr, {true, false});
    const auto t = tangent(direction(site_lat, site_lon));
    constexpr double pad = 1.0;
    double worst_h = 0.0, worst_n = 0.0, slope = 0.0, level_excess = 0.0;
    for (int k = 0; k < 20; ++k) {
        const Vec3 c = at(t, (unit_hash(30, k) - 0.5) * 300.0, (unit_hash(31, k) - 0.5) * 300.0);
        const auto support = ground.support_under_disc(c, c, pad);
        const double plane_height = lola->height_at_direction_m(c);
        const Vec3 plane_normal = lola->normal_fixed(c * (R + plane_height));
        worst_h = std::max(worst_h, std::abs(support.height_m - plane_height));
        worst_n = std::max(worst_n, angle_between(support.normal_fixed, plane_normal));
        slope = angle_between(plane_normal, c);
        // A level pad (not pivoting) would rest on the uphill rim: r tan(slope) higher.
        const Tangent disc = tangent(c);
        level_excess = dense_ground_protrusion(ground, disc, pad, Plane{plane_height, 0.0, 0.0});
        CHECK(std::abs(support.height_m - plane_height) <= 1e-3);
        CHECK(angle_between(support.normal_fixed, plane_normal) <= 1e-3);
    }
    std::printf("9 deg plane (true slope %.3f deg): |support - plane height| <= %.2e m, normal off <= %.2e rad; a level "
                "pad would stand %.4f m higher\n",
                slope / deg, worst_h, worst_n, level_excess);
    CHECK(level_excess > 0.1);  // the case tells the pivoting pad from a level one
}

TEST_CASE("A rock at the rim lifts the pivoting pad by its protrusion above the fitted plane",
          "[moon_surface][terrain][contact]") {
    const auto lola = sloped_lola(9.0 * deg);
    const auto bare = bare_detail(lola);
    const auto lr = std::make_shared<const environment::LunarRocks>(bare);
    const DetailedLunarTerrain s(lola, bare, lr, {});
    constexpr double pad = 1.0;
    int edge_cases = 0, rim_cases = 0;
    for (const Rock& rock : prominent_rocks(*lr, direction(site_lat, site_lon))) {
        if (edge_cases >= 3 && rim_cases >= 3) break;
        const Vec3 peak = rock_peak(*lr, rock);
        // Uphill at the peak: the pad centre lies uphill, the rock at the pad's downhill rim.
        const Vec3 uphill = tangential(peak, -lola->normal_fixed(peak * R));
        for (const bool rim_cut : {false, true}) {
            const double offset = rim_cut ? pad + 0.3 * rock.axes_m.x : 0.999 * pad;
            const Vec3 centre = math::normalized(peak + uphill * (offset / R));
            const Tangent disc = tangent(centre);
            const Plane plane = pad_plane(s, disc, pad);
            const Highest rock_max = brute_rock_max(*lr, rock, disc, pad, plane);
            if (!(rock_max.height_m > dense_ground_protrusion(s, disc, pad, plane) + 0.03)) continue;
            bool other = false;
            for (const Rock& r : lr->rocks_touching(centre, pad, DetailedLunarTerrain::contact_min_rock_diameter_m)) {
                if (r.id != rock.id && brute_rock_max(*lr, r, disc, pad, plane).height_m > rock_max.height_m - 0.01) other = true;
            }
            if (other) continue;
            const auto support = s.support_under_disc(centre, centre, pad);
            const double expected = plane.mean + rock_max.height_m;
            // A level pad on the same rock: lifted by the rock's full height above the plane's centre height.
            const double level = brute_rock_max(*lr, rock, disc, pad).height_m;
            std::printf("9 deg plane, rock D %.2f m, %s: support %.4f m, plane + protrusion %.4f m (diff %.2f mm), level pad on "
                        "the rock %.4f m, normal off %.4f rad\n",
                        rock.diameter_m, rim_cut ? "rim cut" : "peak under edge", support.height_m, expected,
                        1e3 * (support.height_m - expected), level, angle_between(support.normal_fixed, rock_max.normal));
            CHECK(std::abs(support.height_m - expected) <= 1e-3);
            CHECK(angle_between(support.normal_fixed, rock_max.normal) <= 0.05);
            CHECK(std::abs(expected - level) > 0.1);  // the plane drops ~r tan(9 deg) = 0.16 m to the downhill rim
            (rim_cut ? rim_cases : edge_cases) += 1;
        }
    }
    CHECK(edge_cases >= 2);
    CHECK(rim_cases >= 2);
}

TEST_CASE("A pivoting footpad on the procedural ground stands on its plane and the rock protruding most",
          "[moon_surface][terrain][contact]") {
    const auto& s = *surface();
    constexpr double pad = 1.0;
    int edge_cases = 0, rim_cases = 0;
    for (const Rock& rock : prominent_rocks(*rocks(), direction(site_lat, site_lon))) {
        if (edge_cases >= 3 && rim_cases >= 3) break;
        const Vec3 peak_unit = rock_peak(*rocks(), rock);
        // Away from the rock: the direction from its base to its peak, continued (base at the peak itself: any).
        const Tangent at_peak = tangent(peak_unit);
        Vec3 away = (peak_unit - rock.base_unit) * R;
        away = away - at_peak.c * dot(away, at_peak.c);
        if (norm(away) < 1e-3) away = at_peak.e1;
        away = math::normalized(away);
        for (const bool rim_cut : {false, true}) {
            const double offset = rim_cut ? pad + 0.3 * rock.axes_m.x : 0.999 * pad;
            const Vec3 centre = math::normalized(peak_unit + away * (offset / R));
            const Tangent disc = tangent(centre);
            const Plane plane = pad_plane(s, disc, pad);
            const Highest rock_max = brute_rock_max(*rocks(), rock, disc, pad, plane);
            if (!(rock_max.height_m > dense_ground_protrusion(s, disc, pad, plane) + 0.03)) continue;
            bool other = false;
            for (const Rock& r : rocks()->rocks_touching(centre, pad, DetailedLunarTerrain::contact_min_rock_diameter_m)) {
                if (r.id != rock.id && brute_rock_max(*rocks(), r, disc, pad, plane).height_m > rock_max.height_m - 0.01) {
                    other = true;
                }
            }
            if (other) continue;
            const auto support = s.support_under_disc(centre, centre, pad);
            const double expected = plane.mean + rock_max.height_m;
            std::printf("rock D %.2f m, %s: support %.4f m, brute force %.4f m (diff %.2f mm), normal off %.4f rad\n",
                        rock.diameter_m, rim_cut ? "rim cut" : "peak under edge", support.height_m, expected,
                        1e3 * (support.height_m - expected), angle_between(support.normal_fixed, rock_max.normal));
            CHECK(std::abs(support.height_m - expected) <= 1e-3);
            CHECK(angle_between(support.normal_fixed, rock_max.normal) <= 0.05);
            (rim_cut ? rim_cases : edge_cases) += 1;
        }
    }
    CHECK(edge_cases >= 3);
    CHECK(rim_cases >= 3);
}

TEST_CASE("Flat ground: the pivoting pad is the point query; in a crater it rests on the rim",
          "[moon_surface][terrain][contact]") {
    // Flat: no relief, no rocks - the pad's support is the ground at its centre.
    const auto flat = bare_detail(flat_lola());
    const DetailedLunarTerrain level(flat_lola(), flat, nullptr, {true, false});
    const auto t = tangent(direction(site_lat, site_lon));
    for (int k = 0; k < 10; ++k) {
        const Vec3 c = at(t, (unit_hash(40, k) - 0.5) * 200.0, (unit_hash(41, k) - 0.5) * 200.0);
        const auto support = level.support_under_disc(c, c, 1.0);
        CHECK(std::abs(support.height_m - level.contact_ground_m(c)) <= 1e-9);
        CHECK(angle_between(support.normal_fixed, c) <= 1e-9);
    }
    // Craters of 4 m and more only, no roughness: a 1 m pad centred in an isolated crater rests on its bowl. The bowl is
    // rotationally symmetric, so the fitted plane is level and the rim samples (rho = 1 m) protrude most: the support
    // is the bowl's height at 1 m from the centre (crater_profile_m, plus the octaves' constant mean-field shift).
    auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
    cfg.rocks.abundance_source = "uniform";
    cfg.roughness.rms_height_at_baseline_m = 0.0;
    cfg.craters.sfd.min_diameter_m = 4.0;
    const auto craters = std::make_shared<const environment::LunarSurfaceDetail>(cfg, flat_lola());
    const DetailedLunarTerrain with_craters(flat_lola(), craters, nullptr, {true, false});
    int found = 0;
    for (const auto& crater : craters->craters_within(direction(site_lat, site_lon), 400.0, 4.0, 16.0)) {
        if (found >= 3) break;
        bool isolated = true;  // no other crater reaches the pad (1.5 D from its centre) or its own reach
        for (const auto& other : craters->craters_within(crater.centre_unit, 200.0, 4.0, 2000.0)) {
            const double d = R * norm(other.centre_unit - crater.centre_unit);
            if (d > 1e-6 && d < 1.5 * other.diameter_m + 1.5 * crater.diameter_m) isolated = false;
        }
        if (!isolated) continue;
        const Vec3 c = crater.centre_unit;
        const auto support = with_craters.support_under_disc(c, c, 1.0);
        const double shift = with_craters.contact_ground_m(c) - environment::crater_profile_m(crater, 0.0);
        const double expected = shift + environment::crater_profile_m(crater, 1.0) - 1.0 / (2.0 * R);
        std::printf("crater D %.2f m (depth %.3f m): pad support %.5f m, bowl at 1 m %.5f m (diff %.3f mm), point %.4f m\n",
                    crater.diameter_m, crater.depth_m, support.height_m, expected, 1e3 * (support.height_m - expected),
                    with_craters.contact_ground_m(c));
        CHECK(std::abs(support.height_m - expected) <= 1e-3);
        CHECK(angle_between(support.normal_fixed, c) <= 1e-3);
        ++found;
    }
    CHECK(found >= 2);
}

TEST_CASE("The point contact surface includes the rocks under it", "[moon_surface][terrain][contact]") {
    const auto& s = *surface();
    int found = 0;
    for (const Rock& rock : prominent_rocks(*rocks(), direction(site_lat, site_lon))) {
        if (found >= 4) break;
        const Vec3 peak = rock_peak(*rocks(), rock);
        const double ground = s.contact_ground_m(peak);
        const auto top = s.contact_rock_top(rock, peak);
        if (!top || top->first < ground + 0.1) continue;
        // The contact top is the rock's top (LunarRocks::rock_top) to well below a millimetre.
        const auto exact = rocks()->rock_top(rock, peak);
        REQUIRE(exact);
        CHECK(std::abs(top->first - exact->first) <= 1e-4);
        CHECK(angle_between(top->second, exact->second) <= 0.02);
        // Expected: the ground and every contact-size rock's contact top at the point.
        double expected = ground;
        Vec3 expected_normal;
        for (const Rock& r : rocks()->rocks_touching(peak, 0.0, DetailedLunarTerrain::contact_min_rock_diameter_m)) {
            if (const auto rt = s.contact_rock_top(r, peak); rt && rt->first > expected) {
                expected = rt->first;
                expected_normal = rt->second;
            }
        }
        const auto point = s.support_under_disc(peak, peak, 0.0);
        CHECK(point.height_m == expected);
        CHECK(same(point.normal_fixed, expected_normal));
        CHECK(s.contact_height_m(peak) == expected);
        CHECK(s.support_upper_bound_m(peak, peak, 0.0) >= expected);
        ++found;
    }
    CHECK(found >= 3);
}

TEST_CASE("A base-ring point over a rock touches it", "[moon_surface][terrain][contact][legs]") {
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    auto moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    const auto& s = *surface();
    int found = 0;
    for (const Rock& rock : prominent_rocks(*rocks(), direction(site_lat, site_lon))) {
        if (found >= 2) break;
        const Vec3 peak = rock_peak(*rocks(), rock);
        const double ground = s.contact_ground_m(peak), top = s.contact_height_m(peak);
        if (top < ground + 0.1) continue;
        // Site frame at the rock's peak; one base-ring point there, between the ground and the rock top.
        const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, peak)), north = cross(peak, east);
        const auto field = std::make_shared<contact::TerrainField>(
            moon, peak * moon.shape().semi_major_axis_m, contact::from_columns(east, north, peak), surface());
        CHECK(std::abs(field->height_below(0.0, 0.0).height_m - top) <= 1e-9);
        contact::LegGeometry geometry;
        geometry.base_ring = {Vec3{0.0, 0.0, 0.0}};
        contact::SitePose pose;
        pose.cg_station_m = 20.0;
        pose.body_to_site = contact::from_columns({0, 0, 1}, {0, 1, 0}, {-1, 0, 0});
        std::vector<double> crushed;
        contact::LandingSurface surface_on_rock;
        surface_on_rock.field = field;
        pose.cg_m = {0.0, 0.0, 20.0 + 0.5 * (ground + top)};
        CHECK(contact::leg_contact(geometry, crushed, pose, surface_on_rock).base_contact);
        pose.cg_m = {0.0, 0.0, 20.0 + top + 1e-3};
        CHECK(!contact::leg_contact(geometry, crushed, pose, surface_on_rock).base_contact);
        ++found;
    }
    CHECK(found >= 2);
}

TEST_CASE("A cached disc query equals a fresh one bit for bit", "[moon_surface][terrain][contact]") {
    const auto& s = *surface();
    constexpr double pad = 1.0;
    {
        const DetailedLunarTerrain fresh(flat_lola(), detail(), rocks(), {});
        const auto t = tangent(direction(site_lat, site_lon));
        for (int k = 0; k < 40; ++k) {
            const Vec3 c = at(t, (unit_hash(10, k) - 0.5) * 40.0, (unit_hash(11, k) - 0.5) * 40.0);
            s.support_under_disc(c, c, pad);
            const auto a = s.support_under_disc(c, c, pad);      // warm cache
            const auto b = fresh.support_under_disc(c, c, pad);  // cold cache
            CHECK(a.height_m == b.height_m);
            CHECK(same(a.normal_fixed, b.normal_fixed));
            const auto p = s.support_under_disc(c, c, 0.0);  // the contact surface at a point
            const auto q = fresh.support_under_disc(c, c, 0.0);
            CHECK(p.height_m == q.height_m);
            CHECK(same(p.normal_fixed, q.normal_fixed));
            // The pivoting pad never stands below the ground at its centre (the centre sample's protrusion).
            CHECK(a.height_m >= s.contact_ground_m(c));
        }
    }
}

TEST_CASE("A footpad disc on the procedural ground stands on its plane and highest protrusion",
          "[moon_surface][terrain][contact]") {
    // Pads centred in craters of 6-16 m (rocks off: the ground alone). The pattern's highest protrusion above the pad's
    // plane agrees with a dense one of the same ground to 3 cm ([E]: what the 25-sample pattern misses of the
    // sub-pattern roughness). A single-crater geometry check is in "Flat ground: ..." (craters >= 4 m only).
    static const DetailedLunarTerrain ground_only(flat_lola(), detail(), nullptr, {true, false});
    constexpr double pad = 1.0;
    int found = 0;
    double worst = 0.0;
    for (const auto& crater : detail()->craters_within(direction(site_lat, site_lon), 300.0, 6.0, 16.0)) {
        if (found >= 8) break;
        const Tangent disc = tangent(crater.centre_unit);
        const auto support = ground_only.support_under_disc(crater.centre_unit, crater.centre_unit, pad);
        const Plane plane = pad_plane(ground_only, disc, pad);
        const double dense = plane.mean + dense_ground_protrusion(ground_only, disc, pad, plane);
        worst = std::max(worst, std::abs(support.height_m - dense));
        CHECK(std::abs(support.height_m - dense) <= 0.03);
        CHECK(angle_between(support.normal_fixed, math::normalized(crater.centre_unit - disc.e1 * plane.gx -
                                                                   disc.e2 * plane.gy)) <= 1e-9);
        ++found;
    }
    std::printf("pads in %d craters: |pattern - dense protrusion| <= %.4f m\n", found, worst);
    CHECK(found >= 3);
}

TEST_CASE("Full-detail height agrees with the band-limited height at its limit plus the removed band mean",
          "[moon_surface][terrain]") {
    const auto& s = *surface();
    const auto t = tangent(direction(site_lat, site_lon));
    // At its limit (no averaging, every rock) the band-limited height is the full-detail height.
    for (int k = 0; k < 200; ++k) {
        const Vec3 p = at(t, (unit_hash(20, k) - 0.5) * 200.0, (unit_hash(21, k) - 0.5) * 200.0);
        CHECK(s.height_limited_m(p, 0.0, 0.0) == s.height_at_direction_m(p));
        CHECK(s.height_at_direction_m(p) >= s.lola().height_at_direction_m(p) + s.detail().relief_m(p, 0.0));
    }
    // Without rocks: the band-limited height at L = the full-detail ground averaged over the footprint (the removed
    // band's mean is what the limit drops), to 10 % of the removed band's RMS (LunarSurfaceDetail's tolerance).
    constexpr double no_rocks = 1e9;
    for (const double limit : {4.0, 1.0}) {
        double rms_removed = 0.0, max_diff = 0.0;
        constexpr int points = 30, samples = 24;
        for (int k = 0; k < points; ++k) {
            const Vec3 p = at(t, (unit_hash(22, k) - 0.5) * 2000.0, (unit_hash(23, k) - 0.5) * 2000.0);
            const auto fc = environment::face_coord_of(p);
            const auto frame = environment::face_frame(fc.face, fc.s, fc.t, R);
            const double ds = 0.5 * limit / frame.m_s, dt = 0.5 * limit / frame.m_t;
            double mean = 0.0;
            for (int a = 0; a < samples; ++a) {
                for (int b = 0; b < samples; ++b) {
                    const double ss = fc.s + ds * (2.0 * (a + 0.5) / samples - 1.0);
                    const double tt = fc.t + dt * (2.0 * (b + 0.5) / samples - 1.0);
                    mean += s.height_limited_m(environment::face_direction(fc.face, ss, tt), 0.0, no_rocks);
                }
            }
            mean /= samples * samples;
            const double coarse = s.height_limited_m(p, limit, no_rocks);
            const double removed = s.height_limited_m(p, 0.0, no_rocks) - coarse;
            rms_removed += removed * removed;
            max_diff = std::max(max_diff, std::abs(coarse - mean));
        }
        rms_removed = std::sqrt(rms_removed / points);
        std::printf("limit %.0f m: removed band RMS %.4f m, |limited - footprint mean| max %.4f m\n", limit, rms_removed,
                    max_diff);
        CHECK(max_diff <= 0.1 * rms_removed);
    }
}

TEST_CASE("Physics query cost", "[moon_surface][terrain][contact][legs][contact_speed]") {
    // Runs alone (CTest test contact_speed, RUN_SERIAL): a wall-time gate.
    // The whole leg contact of the HLS (six 1 m pads on a 12 m footprint and the 12 base-ring points) on the composed
    // surface at the site, through the TerrainField the landing uses; a fresh surface (cold caches).
    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto moon_me = spice::SpiceOrientation::moon_mean_earth();
    auto moon = environment::ReferenceBody::moon(ephemeris, moon_me);
    const auto fresh = std::make_shared<const DetailedLunarTerrain>(flat_lola(), detail(), rocks(),
                                                                    environment::SurfaceDetailOptions{});
    const Vec3 site = direction(site_lat, site_lon);
    const Vec3 east = math::normalized(cross(Vec3{0.0, 0.0, 1.0}, site)), north = cross(site, east);
    contact::LandingSurface surface_at_site;
    surface_at_site.field = std::make_shared<contact::TerrainField>(moon, site * moon.shape().semi_major_axis_m,
                                                                    contact::from_columns(east, north, site), fresh);
    config::LandingLegConfig legs;
    legs.azimuths_rad = {0.0, 60 * deg, 120 * deg, 180 * deg, 240 * deg, 300 * deg};
    legs.footprint_radius_m = 12.0;
    legs.foot_station_m = -1.5;
    legs.foot_pad_radius_m = 1.0;
    legs.stroke_m = 0.6;
    legs.crush_force_n = 5e5;
    legs.stiffness_n_m = 4e7;
    legs.damping_n_s_m = 8e5;
    legs.friction = 0.6;
    legs.fixed = true;
    const auto geometry = contact::make_leg_geometry(legs, 0.0, 4.5);
    contact::SitePose pose;
    pose.cg_station_m = 20.0;
    pose.body_to_site = contact::from_columns({0, 0, 1}, {0, 1, 0}, {-1, 0, 0});
    std::vector<double> crushed;
    using clock = std::chrono::steady_clock;
    volatile double sink = 0.0;
    const auto tick = [&](double x, double y, double z) {
        pose.cg_m = {x, y, z};
        sink = sink + contact::leg_contact(geometry, crushed, pose, surface_at_site).force_n.z;
    };
    // Resting height: the highest pad support 2 cm into its foot (the others may hang free).
    double rest_z = -1e9;
    for (const Vec3& foot : geometry.feet) {
        const Vec3 p = pose.body_to_site * (foot - Vec3{pose.cg_station_m, 0.0, 0.0});
        rest_z = std::max(rest_z, surface_at_site.below_disc(3.0 + p.x, -5.0 + p.y, 1.0).height_m - p.z - 0.02);
    }
    // Cold: a fresh surface (cleared caches) on the first tick there; the rest from the same surface.
    const auto t0 = clock::now();
    tick(3.0, -5.0, rest_z);
    const double cold_us = std::chrono::duration<double, std::micro>(clock::now() - t0).count();
    // Blocks of ticks; the median block (robust against machine load in the parallel test run).
    constexpr int blocks = 7, per_block = 200;
    const auto median_us = [&](auto&& pose_of_tick) {
        std::vector<double> block_us;
        int k = 0;
        for (int b = 0; b < blocks; ++b) {
            const auto start = clock::now();
            for (int i = 0; i < per_block; ++i, ++k) pose_of_tick(k);
            block_us.push_back(std::chrono::duration<double, std::micro>(clock::now() - start).count() / per_block);
        }
        std::sort(block_us.begin(), block_us.end());
        return block_us[blocks / 2];
    };
    const double resting_us = median_us([&](int) { tick(3.0, -5.0, rest_z); });
    // Sliding: 1 mm per tick (0.2 m/s at 200 Hz, a touchdown's slip), on ground not yet visited.
    const double sliding_us = median_us([&](int k) { tick(3.0 + 1e-3 * (k + 1), -5.0, rest_z); });
    // Descending from 50 m (CG above the rest height) at 2 m/s with 0.2 m/s drift, 5,000 ticks to the ground: the mean.
    double descending_us = 0.0;
    {
        constexpr int ticks = 5000;
        const auto start = clock::now();
        for (int k = 0; k < ticks; ++k) tick(-20.0 + 1e-3 * k, 10.0, rest_z + 50.0 * (1.0 - static_cast<double>(k) / ticks));
        descending_us = std::chrono::duration<double, std::micro>(clock::now() - start).count() / ticks;
    }
    std::printf("leg contact, six 1 m pads + 12 base-ring points per tick: first tick %.0f us (cold), resting %.1f us, "
                "sliding 1 mm/tick %.1f us, descending from 50 m %.1f us (medians of %d x %d ticks; descent mean)\n",
                cold_us, resting_us, sliding_us, descending_us, blocks, per_block);
    CHECK(resting_us < 50.0);
    CHECK(sliding_us < 50.0);
    CHECK(descending_us < 50.0);
}
