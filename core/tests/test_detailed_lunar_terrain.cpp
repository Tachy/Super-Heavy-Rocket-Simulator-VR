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
#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/detailed_lunar_terrain.hpp"
#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/surface_hash.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/terrain_cache.hpp"

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

// Brute-force highest rock top of `rock` over the disc (centre, radius) in the tangent frame of the disc centre: a
// polar grid plus a dense rim (the disc's highest point of a rock cut by the rim lies on the rim).
struct Highest {
    double height_m = -std::numeric_limits<double>::infinity();
    Vec3 normal;
    Vec3 unit;  // where
};
Highest brute_rock_max(const Rock& rock, const Tangent& disc, double radius) {
    const auto& lr = *rocks();
    Highest best;
    const auto try_point = [&](double x, double y) {
        if (const auto top = lr.rock_top(rock, at(disc, x, y))) {
            const double h = top->first - (x * x + y * y) / (2.0 * R);
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

// Highest contact ground over the disc on a dense pattern (precondition checks: a rock must stand clear of it).
double dense_ground_max(const Tangent& disc, double radius) {
    double best = -std::numeric_limits<double>::infinity();
    for (int i = 0; i <= 20; ++i) {
        const double rho = radius * i / 20.0;
        for (int k = 0; k < 40; ++k) {
            const double phi = constants::two_pi * k / 40.0;
            best = std::max(best, surface()->contact_ground_m(at(disc, rho * std::cos(phi), rho * std::sin(phi))));
        }
    }
    return best;
}

// Rocks near the site that stand well proud of the ground (precondition of the disc tests).
std::vector<Rock> prominent_rocks() {
    std::vector<Rock> out;
    for (const Rock& r : rocks()->rocks_near(direction(site_lat, site_lon), 60.0, 0.4)) {
        if (r.diameter_m > 1.5) continue;
        const double top = r.base_height_m + r.axes_m.z * environment::RockTemplates::instance().top_z(r.template_id) -
                           r.burial_m;
        if (top - r.base_height_m > 0.15) out.push_back(r);
    }
    return out;
}

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

TEST_CASE("A footpad disc stands on the highest support under it", "[moon_surface][terrain][contact]") {
    const auto& s = *surface();
    constexpr double pad = 1.0;
    int edge_cases = 0, rim_cases = 0;
    for (const Rock& rock : prominent_rocks()) {
        if (edge_cases >= 3 && rim_cases >= 3) break;
        // The rock's peak by brute force over its whole footprint (a disc around its base covering its reach).
        const Vec3 peak_unit = brute_rock_max(rock, tangent(rock.base_unit), rock.axes_m.x).unit;
        // Away from the rock: the direction from its base to its peak, continued (base at the peak itself: any).
        const Tangent at_peak = tangent(peak_unit);
        Vec3 away = (peak_unit - rock.base_unit) * R;
        away = away - at_peak.c * dot(away, at_peak.c);
        if (norm(away) < 1e-3) away = at_peak.e1;
        away = math::normalized(away);

        for (const bool rim_cut : {false, true}) {
            // Peak under the pad edge (1 mm inside), or the rim cutting the rock's flank (peak 0.3 a outside).
            const double offset = rim_cut ? pad + 0.3 * rock.axes_m.x : 0.999 * pad;
            const Vec3 centre = math::normalized(peak_unit + away * (offset / R));
            const Tangent disc = tangent(centre);
            const Highest expected = brute_rock_max(rock, disc, pad);
            if (!(expected.height_m > dense_ground_max(disc, pad) + 0.03)) continue;  // the rock must be the support
            // No other rock of contact size may reach higher under this disc.
            bool other = false;
            for (const Rock& r : rocks()->rocks_touching(centre, pad, DetailedLunarTerrain::contact_min_rock_diameter_m)) {
                if (r.id != rock.id && brute_rock_max(r, disc, pad).height_m > expected.height_m - 0.01) other = true;
            }
            if (other) continue;
            const auto support = s.support_under_disc(centre, centre, pad);
            std::printf("rock D %.2f m, %s: support %.4f m, brute force %.4f m (diff %.2f mm), normal off %.4f rad\n",
                        rock.diameter_m, rim_cut ? "rim cut" : "peak under edge", support.height_m, expected.height_m,
                        1e3 * (support.height_m - expected.height_m), angle_between(support.normal_fixed, expected.normal));
            CHECK(std::abs(support.height_m - expected.height_m) <= 1e-3);
            CHECK(angle_between(support.normal_fixed, expected.normal) <= 0.05);
            (rim_cut ? rim_cases : edge_cases) += 1;
        }
    }
    CHECK(edge_cases >= 3);
    CHECK(rim_cases >= 3);
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
            CHECK(a.height_m >= p.height_m);  // the disc holds its centre
        }
    }
}

TEST_CASE("A footpad disc on the procedural ground stands on its highest points", "[moon_surface][terrain][contact]") {
    // Pads centred in craters of 6-16 m (rocks off: the ground alone). The pattern max agrees with a dense max of the
    // same ground to 3 cm ([E]: what the 25-sample pattern misses of the sub-pattern roughness). The real relief at the
    // crater scale (roughness RMS 0.19 m at 20 m, overlapping craters) is too strong for a single-crater geometry check
    // here; the leg's response to a crater geometry is tested on an analytic field ([contact][legs]).
    static const DetailedLunarTerrain ground_only(flat_lola(), detail(), nullptr, {true, false});
    constexpr double pad = 1.0;
    int found = 0;
    double worst = 0.0;
    for (const auto& crater : detail()->craters_within(direction(site_lat, site_lon), 300.0, 6.0, 16.0)) {
        if (found >= 8) break;
        const Tangent disc = tangent(crater.centre_unit);
        const auto support = ground_only.support_under_disc(crater.centre_unit, crater.centre_unit, pad);
        double dense = -1e9;
        for (int i = 0; i <= 40; ++i) {
            for (int k = 0; k < 80; ++k) {
                const double rho = pad * i / 40.0, phi = constants::two_pi * k / 80.0;
                dense = std::max(dense, ground_only.contact_ground_m(at(disc, rho * std::cos(phi), rho * std::sin(phi))));
            }
        }
        worst = std::max(worst, std::abs(support.height_m - dense));
        CHECK(std::abs(support.height_m - dense) <= 0.03);
        ++found;
    }
    std::printf("pads in %d craters: |pattern max - dense max| <= %.4f m\n", found, worst);
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

TEST_CASE("Physics query cost", "[moon_surface][terrain][contact]") {
    const auto& s = *surface();
    const auto t = tangent(direction(site_lat, site_lon));
    constexpr double footprint = 12.0, pad = 1.0;
    const auto pads = [&](int tick) {
        double sum = 0.0;
        for (int k = 0; k < 6; ++k) {
            const double az = constants::two_pi * k / 6.0;
            const double drift = 1e-3 * tick;  // the pads slide 1 mm per tick (a touchdown's slip)
            const Vec3 c = at(t, footprint * std::cos(az) + 3.0 + drift, footprint * std::sin(az) - 5.0);
            sum += s.support_under_disc(c, c, pad).height_m;
        }
        return sum;
    };
    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    volatile double sink = pads(0);
    const double cold_us = std::chrono::duration<double, std::micro>(clock::now() - t0).count();
    constexpr int ticks = 400;
    t0 = clock::now();
    for (int k = 1; k <= ticks; ++k) sink = pads(k);
    const double sliding_us = std::chrono::duration<double, std::micro>(clock::now() - t0).count() / ticks;
    t0 = clock::now();
    for (int k = 0; k < ticks; ++k) sink = pads(ticks);
    const double resting_us = std::chrono::duration<double, std::micro>(clock::now() - t0).count() / ticks;
    (void)sink;
    std::printf("six footpad discs: first tick %.0f us (cold cache), sliding 1 mm/tick %.1f us/tick, resting %.1f us/tick\n",
                cold_us, sliding_us, resting_us);
    CHECK(resting_us < 50.0);
}
