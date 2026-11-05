// Lunar rocks (Task 3 of the Moon procedural surface plan): shapes, population per cube-face cell, rock tops, cover
// fraction, determinism, the abundance map and the summed surface roughness.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <memory>
#include <set>
#include <thread>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/config/rock_abundance_file.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/lunar_surface_detail.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/rock_templates.hpp"
#include "starsim/environment/surface_hash.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using environment::FaceCell;
using environment::LunarRocks;
using environment::LunarSurfaceDetail;
using environment::Rock;
using environment::RockTemplates;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double R = 1737400.0;
constexpr double site_lat = -84.79 * deg, site_lon = 29.2 * deg;
constexpr double global_lat = -60.0 * deg, global_lon = 100.0 * deg;

Vec3 direction(double lat, double lon) {
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

struct Tangent {
    Vec3 c, e1, e2;
};
Tangent tangent(const Vec3& c) {
    const Vec3 e1 = math::normalized(cross(std::abs(c.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, c));
    return {c, e1, cross(c, e1)};
}
Vec3 at(const Tangent& t, double x, double y) { return math::normalized(t.c + (t.e1 * x + t.e2 * y) / R); }

double unit_hash(std::uint64_t a, std::uint64_t b, std::uint64_t c = 0) {
    return environment::hash_unit(environment::surface_hash(0x40C5, a, b, c));
}

// Flat global grid and one 20 m tile centred on the site (the same flat terrain as the surface-detail tests).
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

std::shared_ptr<const environment::LunarTerrain> make_terrain() {
    environment::CylindricalGrid g;
    g.grid = {2880, 5760, 0.5, std::vector<std::int16_t>(2880 * 5760, 0)};
    g.pixels_per_degree = 16.0;
    g.zero_line = 1439.5;
    g.zero_sample = -0.5;
    return std::make_shared<const environment::LunarTerrain>(std::move(g), std::vector{site_tile()}, R);
}

config::MoonSurfaceConfig load_config(const char* abundance_source) {
    auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
    cfg.rocks.abundance_source = abundance_source;
    return cfg;
}

// The shipped configuration with the uniform law (the Diviner product is not installed in the test environment; the
// site at -84.8 deg lies outside the map anyway).
std::shared_ptr<const LunarSurfaceDetail> shared_detail() {
    static const auto d = std::make_shared<const LunarSurfaceDetail>(load_config("uniform"), make_terrain());
    return d;
}

// The shipped configuration as it is ("diviner"): needs a map.
std::shared_ptr<const LunarSurfaceDetail> diviner_detail() {
    static const auto d = std::make_shared<const LunarSurfaceDetail>(load_config("diviner"), make_terrain());
    return d;
}

// Plan-view area of the part of the rock above its base height, measured with rock_top on a sunflower pattern of
// `points` columns over the rock's bounding ellipse (pi a b).
double measured_footprint_m2(const LunarRocks& lr, const Rock& r, int points) {
    const Vec3 e_a = math::column(r.local_to_fixed, 0), e_b = math::column(r.local_to_fixed, 1);
    const double golden = constants::pi * (3.0 - std::sqrt(5.0));
    int hits = 0;
    for (int k = 0; k < points; ++k) {
        const double rho = std::sqrt((k + 0.5) / points), phi = golden * k;
        const double x = r.axes_m.x * rho * std::cos(phi), y = r.axes_m.y * rho * std::sin(phi);
        if (lr.rock_top(r, math::normalized(r.base_unit + (e_a * x + e_b * y) / R))) ++hits;
    }
    return constants::pi * r.axes_m.x * r.axes_m.y * hits / points;
}

// Lowest and highest vertex of a template's finest mesh (independent of the sampled extents in RockTemplates).
// Maximum of f(d) over the unit sphere, independent of the templates' own construction: 20000 hashed directions, the
// best 16 refined by a pattern search (8 tangent directions, halving steps down to 1e-11 rad).
template <class F>
double sphere_max(const F& f, std::uint64_t salt) {
    std::vector<std::pair<double, Vec3>> samples;
    for (int k = 0; k < 20000; ++k) {
        const double z = 2.0 * unit_hash(salt, static_cast<std::uint64_t>(k), 1) - 1.0, phi = constants::two_pi * unit_hash(salt, static_cast<std::uint64_t>(k), 2);
        const double rho = std::sqrt(1.0 - z * z);
        const Vec3 d{rho * std::cos(phi), rho * std::sin(phi), z};
        samples.emplace_back(f(d), d);
    }
    std::partial_sort(samples.begin(), samples.begin() + 64, samples.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
    double best = -std::numeric_limits<double>::infinity();
    for (int c = 0; c < 64; ++c) {
        Vec3 d = samples[static_cast<size_t>(c)].second;
        double value = samples[static_cast<size_t>(c)].first, step = 0.02;
        while (step > 1e-11) {
            const Vec3 t1 = math::normalized(cross(std::abs(d.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, d));
            const Vec3 t2 = cross(d, t1);
            bool moved = false;
            for (int k = 0; k < 32 && !moved; ++k) {
                const double angle = constants::two_pi * k / 32.0;
                const Vec3 trial = math::normalized(d + (t1 * std::cos(angle) + t2 * std::sin(angle)) * step);
                if (const double v = f(trial); v > value) value = v, d = trial, moved = true;
            }
            if (!moved) step *= 0.5;
        }
        best = std::max(best, value);
    }
    return best;
}

// Vertical extent of a template (lowest, highest z of its surface), measured by sphere_max on z r(d).
std::pair<double, double> refined_z_extent(int template_id) {
    const RockTemplates& t = RockTemplates::instance();
    const auto id = static_cast<std::uint64_t>(template_id);
    return {-sphere_max([&](const Vec3& d) { return -d.z * t.radius(template_id, d); }, 2 * id + 100),
            sphere_max([&](const Vec3& d) { return d.z * t.radius(template_id, d); }, 2 * id + 101)};
}

const LunarRocks& rocks() {
    static const LunarRocks r(shared_detail());
    return r;
}

// Synthetic Diviner-like map: constant area fraction over +-60 deg (the loader is tested on a file in the real format).
std::shared_ptr<const environment::RockAbundanceMap> constant_map(double fraction) {
    auto m = std::make_shared<environment::RockAbundanceMap>();
    m->lines = 120;
    m->samples = 240;
    m->lat_limit_rad = 60.0 * deg;
    m->area_fraction.assign(static_cast<size_t>(m->lines * m->samples), static_cast<float>(fraction));
    return m;
}

bool same_bits(const Rock& a, const Rock& b) {
    return a.id == b.id && std::memcmp(&a.base_unit, &b.base_unit, sizeof(Vec3)) == 0 &&
           std::memcmp(&a.axes_m, &b.axes_m, sizeof(Vec3)) == 0 &&
           std::memcmp(&a.local_to_fixed, &b.local_to_fixed, sizeof(math::Mat3)) == 0 &&
           std::memcmp(&a.diameter_m, &b.diameter_m, sizeof(double)) == 0 &&
           std::memcmp(&a.base_height_m, &b.base_height_m, sizeof(double)) == 0 &&
           std::memcmp(&a.burial_m, &b.burial_m, sizeof(double)) == 0 && a.template_id == b.template_id;
}

// Guo et al. 2021's statistic: plane-detrended standard deviation of a W x W window sampled 25 x 25.
template <typename F>
double window_rms(const Tangent& t, double window_m, F&& height) {
    constexpr int n = 25;
    const double half = 0.5 * (n - 1), step = window_m / n;
    std::array<double, n * n> z{};
    double mean = 0.0, sx = 0.0, sy = 0.0, xx = 0.0;
    for (int a = 0; a < n; ++a) {
        for (int b = 0; b < n; ++b) {
            const double v = height(at(t, (a - half) * step, (b - half) * step));
            z[a * n + b] = v;
            mean += v;
            sx += v * (a - half);
            sy += v * (b - half);
            xx += (a - half) * (a - half);
        }
    }
    mean /= n * n;
    const double gx = sx / xx, gy = sy / xx;
    double var = 0.0;
    for (int a = 0; a < n; ++a) {
        for (int b = 0; b < n; ++b) {
            const double r = z[a * n + b] - mean - gx * (a - half) - gy * (b - half);
            var += r * r;
        }
    }
    return std::sqrt(var / (n * n));
}

// Median over `count` windows of the summed surface (craters + roughness, with the rock tops when `with_rocks`).
double median_total_rms(const Vec3& centre, double patch_m, double window_m, int count, bool with_rocks) {
    const auto& d = *shared_detail();
    const Tangent base = tangent(centre);
    std::vector<double> v;
    for (int k = 0; k < count; ++k) {
        const Vec3 c = at(base, (unit_hash(1, k) - 0.5) * patch_m, (unit_hash(2, k) - 0.5) * patch_m);
        const std::vector<Rock> near = with_rocks ? rocks().rocks_touching(c, 0.75 * window_m, 0.0)
                                                  : std::vector<Rock>{};
        v.push_back(window_rms(tangent(c), window_m, [&](const Vec3& p) {
            const double ground = d.relief_m(p, 0.0);
            if (near.empty()) return ground;
            const double lola = d.lola().height_at_direction_m(p);
            double h = ground;
            for (const Rock& r : near) {
                if (const auto top = rocks().rock_top(r, p)) h = std::max(h, top->first - lola);
            }
            return h;
        }));
    }
    std::nth_element(v.begin(), v.begin() + count / 2, v.end());
    return v[static_cast<size_t>(count / 2)];
}

}  // namespace

TEST_CASE("Rock counts per size bin match the configured SFD", "[moon_surface][environment]") {
    const auto& cfg = shared_detail()->config().rocks;
    const Vec3 centre = direction(global_lat, global_lon);
    const std::array<std::pair<double, double>, 7> bins = {
        {{0.01, 0.02}, {0.03, 0.07}, {0.1, 0.2}, {0.3, 0.6}, {1.0, 2.0}, {3.0, 6.0}, {6.0, 10.0}}};
    for (const auto& [lo, hi] : bins) {
        const double per_m2 = cfg.sfd.cumulative_density_per_m2(lo) - cfg.sfd.cumulative_density_per_m2(hi);
        const double radius = std::sqrt(1500.0 / per_m2 / constants::pi);  // a spherical cap's area is pi chord^2
        const double expected = per_m2 * constants::pi * radius * radius;
        const auto found = rocks().rocks_near(centre, radius, lo);
        double n = 0.0;
        for (const Rock& r : found) n += (r.diameter_m >= lo && r.diameter_m < hi) ? 1.0 : 0.0;
        std::printf("rocks %5.2f-%5.2f m: %6.0f counted, %6.0f expected (3 sigma %4.0f), disc radius %6.0f m\n", lo, hi,
                    n, expected, 3.0 * std::sqrt(expected), radius);
        CHECK(expected >= 1000.0);
        CHECK(std::abs(n - expected) <= 3.0 * std::sqrt(expected));
    }
}

TEST_CASE("Every rock belongs to exactly one cell", "[moon_surface][environment]") {
    const auto& octaves = rocks().octaves();
    const auto it = std::find_if(octaves.begin(), octaves.end(), [](const auto& o) { return o.min_diameter_m >= 1.0; });
    REQUIRE(it != octaves.end());
    const int level = it->level;
    const std::int64_t n = std::int64_t{1} << level;
    const double cell_m = environment::face_cell_nominal_size_m(level, R);
    // A cell on a face edge (s = -1 side of +Z) and one at a cube corner (3 faces meet).
    std::vector<FaceCell> centres = {FaceCell{4, level, 0, 0}, FaceCell{5, level, n - 1, n - 1}};  // cube corners
    for (std::int64_t j = n / 3; j < n / 3 + 80; ++j) centres.push_back(FaceCell{4, level, 0, j});  // along an edge
    size_t total = 0;
    for (const FaceCell& c : centres) {
        std::set<FaceCell, decltype([](const FaceCell& a, const FaceCell& b) {
                     return std::tie(a.face, a.level, a.i, a.j) < std::tie(b.face, b.level, b.i, b.j);
                 })>
            block;
        for (const FaceCell& nb : environment::face_cell_neighbourhood(c)) block.insert(nb);
        std::map<std::uint64_t, FaceCell> owner;
        for (const FaceCell& cell : block) {
            for (const Rock& r : rocks().rocks_in_cell(cell, 0.0)) {
                CHECK(owner.emplace(r.id, cell).second);  // no duplicate id
                CHECK(environment::face_cell_of(r.base_unit, level) == cell);
            }
        }
        // The same rocks found by position from a disc covering the block (same octave, standing in the block).
        const Vec3 middle = environment::face_cell_point(c, 0.5, 0.5);
        std::set<std::uint64_t> by_position;
        for (const Rock& r : rocks().rocks_near(middle, 2.5 * cell_m, 1.0)) {
            if (r.diameter_m >= it->max_diameter_m || r.diameter_m < it->min_diameter_m) continue;
            if (block.count(environment::face_cell_of(r.base_unit, level)) != 0) by_position.insert(r.id);
        }
        std::set<std::uint64_t> by_cell;
        for (const auto& [id, cell] : owner) {
            (void)cell;
            by_cell.insert(id);
        }
        CHECK(by_position == by_cell);
        total += by_cell.size();
    }
    CHECK(total > 20);
}

TEST_CASE("Rocks in a region: the children partition it, octaves above and below its level", "[moon_surface][environment]") {
    // A level-17 region (~20 m) at the site: octaves from 4 cm live on finer levels (descendant cells), the metre
    // octaves on coarser ones (the ancestor cell, filtered); one at a face corner too.
    for (const FaceCell& region : {environment::face_cell_of(direction(site_lat, site_lon), 17), FaceCell{4, 17, 0, 0}}) {
        const double min_d = 0.04;
        const auto parent = rocks().rocks_in_region(region, min_d);
        std::set<std::uint64_t> parent_ids;
        for (const Rock& r : parent) {
            CHECK(parent_ids.insert(r.id).second);  // no duplicate
            CHECK(environment::face_cell_of(r.base_unit, region.level) == region);
            CHECK(r.diameter_m >= min_d);
        }
        std::set<std::uint64_t> child_ids;
        const FaceCell children[4] = {{region.face, region.level + 1, 2 * region.i, 2 * region.j},
                                      {region.face, region.level + 1, 2 * region.i + 1, 2 * region.j},
                                      {region.face, region.level + 1, 2 * region.i, 2 * region.j + 1},
                                      {region.face, region.level + 1, 2 * region.i + 1, 2 * region.j + 1}};
        for (const FaceCell& child : children) {
            for (const Rock& r : rocks().rocks_in_region(child, min_d)) CHECK(child_ids.insert(r.id).second);
        }
        CHECK(child_ids == parent_ids);
        // The same set by position: every rock near the region whose base lies in it.
        const Vec3 middle = environment::face_cell_point(region, 0.5, 0.5);
        std::set<std::uint64_t> by_position;
        for (const Rock& r : rocks().rocks_near(middle, environment::face_cell_nominal_size_m(region.level, R), min_d)) {
            if (environment::face_cell_of(r.base_unit, region.level) == region) by_position.insert(r.id);
        }
        CHECK(by_position == parent_ids);
        std::printf("  region %d/%d/%lld/%lld: %zu rocks >= %.2f m\n", region.face, region.level,
                    static_cast<long long>(region.i), static_cast<long long>(region.j), parent_ids.size(), min_d);
        CHECK(parent_ids.size() > 100);
    }
    // A coarse region with the finest octave asked for: refused, not a hang.
    CHECK_THROWS_AS(rocks().rocks_in_region(FaceCell{0, 8, 0, 0}, 0.0), std::invalid_argument);
}

TEST_CASE("A rock top lies above the ground and below its full height", "[moon_surface][environment]") {
    const Vec3 centre = direction(site_lat, site_lon);
    const auto found = rocks().rocks_near(centre, 200.0, 0.5);
    REQUIRE(found.size() > 100);
    const RockTemplates& templates = RockTemplates::instance();
    int hits = 0, misses = 0;
    double worst_surface = 0.0;
    for (const Rock& r : found) {
        const Tangent t = tangent(r.base_unit);
        const double a = r.axes_m.x, b = r.axes_m.y, c = r.axes_m.z;
        const Vec3 e_a = math::column(r.local_to_fixed, 0), e_b = math::column(r.local_to_fixed, 1);
        for (int k = 0; k < 40; ++k) {
            const double x = (2.0 * unit_hash(r.id % 9973, k, 1) - 1.0) * 1.2 * a;
            const double y = (2.0 * unit_hash(r.id % 9973, k, 2) - 1.0) * 1.2 * a;
            const Vec3 p = at(t, x, y);
            const auto top = rocks().rock_top(r, p);
            if (!top) {
                ++misses;
                continue;
            }
            ++hits;
            const Vec3 offset = (p - r.base_unit) * R;
            const double lx = dot(offset, e_a), ly = dot(offset, e_b);
            const double curvature = (lx * lx + ly * ly) / (2.0 * R);
            CHECK(top->first > r.base_height_m);
            // Below the template's real top (its highest point c top_z above the centre), not only the bounding c.
            CHECK(top->first <= r.base_height_m - r.burial_m + c * templates.top_z(r.template_id) + curvature + 1e-9);
            // On the rock's surface: in unit coordinates |u| = r(u / |u|).
            const double lz = top->first - curvature - r.base_height_m + r.burial_m;
            const Vec3 u{lx / a, ly / b, lz / c};
            worst_surface = std::max(worst_surface, std::abs(norm(u) - templates.radius(r.template_id, u)));
            CHECK(std::abs(norm(top->second) - 1.0) < 1e-9);
            CHECK(dot(top->second, r.base_unit) > -1e-6);  // the top surface faces up
        }
        // The vertical axis through the base stays a hit unless the rock is buried below its own top.
        const auto apex = rocks().rock_top(r, r.base_unit);
        if (apex) CHECK(apex->first > r.base_height_m);
    }
    std::printf("rock tops: %d hits, %d misses, worst |u| - r(u) %.2e\n", hits, misses, worst_surface);
    CHECK(worst_surface < 1e-6);
    CHECK(hits > 1000);
    CHECK(misses > 1000);
}

TEST_CASE("The buried share of the rock height is the configured one", "[moon_surface][environment]") {
    const auto& cfg = shared_detail()->config().rocks;
    const auto found = rocks().rocks_near(direction(site_lat, site_lon), 60.0, 0.1);
    REQUIRE(found.size() > 2000);
    // Share by height (the config's definition: share of the rock height below the regolith surface), measured on the
    // template's exact vertical extent (bottom_z / top_z, checked against an independent search in "Rock templates are
    // angular impact fragments"): ground at -burial_m from the centre, rock from c z_lo to c z_hi.
    const RockTemplates& t = RockTemplates::instance();
    double sum = 0.0, lowest = 1.0, highest = 0.0;
    for (const Rock& r : found) {
        const double z_lo = t.bottom_z(r.template_id), z_hi = t.top_z(r.template_id);
        const double share = (r.burial_m / r.axes_m.z - z_lo) / (z_hi - z_lo);
        sum += share;
        lowest = std::min(lowest, share);
        highest = std::max(highest, share);
    }
    const double mean = sum / static_cast<double>(found.size());
    const double sigma_mean = cfg.burial_fraction_sd / std::sqrt(static_cast<double>(found.size()));
    std::printf("buried share: mean %.4f (configured %.2f, 3 sigma %.4f), range %.4f-%.4f over %zu rocks\n", mean,
                cfg.burial_fraction_mean, 3.0 * sigma_mean, lowest, highest, found.size());
    constexpr double extent_tolerance = 1e-9;  // rounding
    CHECK(std::abs(mean - cfg.burial_fraction_mean) <= 3.0 * sigma_mean + extent_tolerance);
    CHECK(lowest >= -extent_tolerance);
    CHECK(highest <= 0.6 + extent_tolerance);
}

TEST_CASE("Rock shapes are star-shaped and closed", "[moon_surface][environment]") {
    const RockTemplates& t = RockTemplates::instance();
    for (int id = 0; id < RockTemplates::count; ++id) {
        double r_min = 1e9, r_max = 0.0;
        for (int k = 0; k < 10000; ++k) {
            const double z = 2.0 * unit_hash(id, k, 1) - 1.0, phi = constants::two_pi * unit_hash(id, k, 2);
            const double rho = std::sqrt(1.0 - z * z);
            const double r = t.radius(id, {rho * std::cos(phi), rho * std::sin(phi), z});
            r_min = std::min(r_min, r);
            r_max = std::max(r_max, r);
        }
        // Positive, and no needle: the thinnest fragment (c/a 0.2) cut by 35 % keeps ~0.13 of its long semi-axis.
        CHECK(r_min > 0.1);
        CHECK(r_max <= 1.0);
        // Normalised: the rock reaches its bounding axes (the corners found by the refined search; margin 0.5 %).
        const double r_peak = sphere_max([&](const Vec3& d) { return t.radius(id, d); }, static_cast<std::uint64_t>(id) + 300);
        CHECK(r_peak <= 1.0);
        CHECK(r_peak > 0.99);
        // Exposed plan-view area: the whole projection below the rock, nothing above it, shrinking in between.
        const double full = t.exposed_area(id, t.bottom_z(id) - 1e-3);
        CHECK(full > 0.2 * constants::pi);
        CHECK(full < constants::pi);
        CHECK(t.exposed_area(id, t.top_z(id) + 1e-3) == 0.0);
        CHECK(t.bottom_z(id) < 0.0);
        CHECK(t.top_z(id) > 0.0);
        double previous = full;
        for (int k = 0; k <= 20; ++k) {
            const double area = t.exposed_area(id, t.bottom_z(id) + (t.top_z(id) - t.bottom_z(id)) * k / 20.0);
            CHECK(area <= previous);
            previous = area;
        }
        for (int lod = 0; lod < RockTemplates::lods; ++lod) {
            const auto& m = t.mesh(id, lod);
            const size_t nv = m.vertices.size(), nf = m.indices.size() / 3;
            REQUIRE(m.normals.size() == nv);
            std::map<std::pair<int, int>, int> edges;  // directed edge count
            for (size_t f = 0; f < nf; ++f) {
                const int i0 = m.indices[3 * f], i1 = m.indices[3 * f + 1], i2 = m.indices[3 * f + 2];
                for (const auto& e : {std::pair{i0, i1}, std::pair{i1, i2}, std::pair{i2, i0}}) ++edges[e];
                // Star-shaped about the origin: every triangle faces outward.
                const Vec3 a = m.vertices[static_cast<size_t>(i0)], b = m.vertices[static_cast<size_t>(i1)],
                           c = m.vertices[static_cast<size_t>(i2)];
                CHECK(dot(cross(b - a, c - a), a + b + c) > 0.0);
            }
            // Watertight and oriented: each directed edge once, its reverse once (V - E + F = 2 on the sphere).
            bool closed = true;
            for (const auto& [e, count] : edges) {
                closed = closed && count == 1 && edges.count({e.second, e.first}) == 1;
            }
            CHECK(closed);
            CHECK(static_cast<int>(nv) - static_cast<int>(edges.size() / 2) + static_cast<int>(nf) == 2);
            if (lod == 0) {
                double worst = 0.0;
                for (size_t v = 0; v < nv; ++v) {
                    const Vec3 p = m.vertices[v];
                    worst = std::max(worst, std::abs(norm(p) - t.radius(id, p)) / t.radius(id, p));
                    CHECK(std::abs(norm(m.normals[v]) - 1.0) < 1e-9);
                    CHECK(dot(m.normals[v], p) > 0.0);
                }
                CHECK(worst < 0.01);
            }
        }
    }
}

TEST_CASE("Rock templates are angular impact fragments", "[moon_surface][environment]") {
    const RockTemplates& t = RockTemplates::instance();
    // Axis law (Fujiwara et al. / Michikami et al. 2016): mean b/a 0.71, c/a 0.50, spread 0.15 (clamped), so the
    // template means lie within 3 sigma of the mean (sigma = 0.15 / sqrt(count)).
    double sum_b = 0.0, sum_c = 0.0;
    int eroded = 0, tilted = 0;
    for (int id = 0; id < RockTemplates::count; ++id) {
        CHECK(t.fragment_c_to_a(id) <= t.fragment_b_to_a(id));
        CHECK(t.fragment_b_to_a(id) <= 1.0);
        sum_b += t.fragment_b_to_a(id);
        sum_c += t.fragment_c_to_a(id);
        eroded += t.eroded(id) ? 1 : 0;
        tilted += t.c_axis_tilt_rad(id) > 30.0 * deg ? 1 : 0;
    }
    const double sigma = 0.15 / std::sqrt(static_cast<double>(RockTemplates::count));
    const double mean_b = sum_b / RockTemplates::count, mean_c = sum_c / RockTemplates::count;
    std::printf("templates: mean b/a %.3f, c/a %.3f (law 0.71, 0.50, 3 sigma %.3f), eroded %d, c axis tilted > 30 deg %d of %d\n",
                mean_b, mean_c, 3.0 * sigma, eroded, tilted, RockTemplates::count);
    CHECK(std::abs(mean_b - 0.71) <= 3.0 * sigma);
    CHECK(std::abs(mean_c - 0.50) <= 3.0 * sigma);
    CHECK(eroded >= 1);                           // both fresh and eroded fragments
    CHECK(eroded < RockTemplates::count);
    // Resting fragments (ruling R4): c axis near the vertical, rms tilt 25 deg, none beyond the 60 deg cap; the scatter
    // still tilts some by more than 30 deg (Rayleigh: ~24 % expected).
    double tilt_square = 0.0, max_tilt = 0.0;
    for (int id = 0; id < RockTemplates::count; ++id) {
        tilt_square += t.c_axis_tilt_rad(id) * t.c_axis_tilt_rad(id);
        max_tilt = std::max(max_tilt, t.c_axis_tilt_rad(id));
    }
    const double rms_tilt = std::sqrt(tilt_square / RockTemplates::count);
    std::printf("templates: rms c-axis tilt %.1f deg, max %.1f deg\n", rms_tilt / deg, max_tilt / deg);
    CHECK(tilted >= 1);
    CHECK(tilted <= RockTemplates::count / 2);
    CHECK(rms_tilt > 15.0 * deg);
    CHECK(rms_tilt < 35.0 * deg);
    CHECK(max_tilt <= 60.0 * deg + 1e-9);

    double worst_extent_gap = 0.0;
    for (int id = 0; id < RockTemplates::count; ++id) {
        // Faceted: on a fresh fragment most of the surface is planar faces. A plane's normal is constant over its face:
        // neighbouring directions with the same normal; across an edge the normal jumps (sharp edge, no blend).
        int planar = 0, samples = 0, sharp = 0;
        for (int k = 0; k < 2000; ++k) {
            const double z = 2.0 * unit_hash(id, k, 7) - 1.0, phi = constants::two_pi * unit_hash(id, k, 8);
            const double rho = std::sqrt(1.0 - z * z);
            const Vec3 d{rho * std::cos(phi), rho * std::sin(phi), z};
            const Vec3 n0 = t.normal(id, d);
            const Vec3 e = math::normalized(cross(std::abs(d.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, d));
            const Vec3 n1 = t.normal(id, d + e * 1e-3);
            ++samples;
            const bool on_face = norm(n1 - n0) < 1e-12;  // same face (an ellipsoid normal turns by ~1e-3)
            if (on_face) ++planar;
            // An edge: 0.05 rad on, still on a face (its normal constant), but a different face's normal.
            const Vec3 far = d + e * 0.05;
            const Vec3 n2 = t.normal(id, far), n3 = t.normal(id, far + e * 1e-3);
            if (on_face && norm(n3 - n2) < 1e-12 && norm(n2 - n0) > 0.1) ++sharp;
        }
        const double planar_share = static_cast<double>(planar) / samples;
        std::printf("template %2d: %s, %2d faces, b/a %.2f c/a %.2f, tilt %4.1f deg, planar share %.2f, edge pairs %d\n", id,
                    t.eroded(id) ? "eroded" : "fresh ", t.plane_count(id), t.fragment_b_to_a(id), t.fragment_c_to_a(id),
                    t.c_axis_tilt_rad(id) / deg, planar_share, sharp);
        if (!t.eroded(id)) {
            CHECK(planar_share > 0.8);
            CHECK(sharp > 0);
        }
        // Exact column tops: the point (x, y, top) lies on the surface (radial function), and no column rises above
        // top_z; top_z is reached (within a grid cell's slope).
        double highest = -1.0, lowest = 1.0;
        for (int i = 0; i < 64; ++i) {
            for (int j = 0; j < 64; ++j) {
                const double x = -1.0 + (2.0 * i + 1.0) / 64.0, y = -1.0 + (2.0 * j + 1.0) / 64.0;
                if (const auto top = t.column_top(id, x, y)) {
                    const Vec3 p{x, y, *top};
                    CHECK(std::abs(norm(p) - t.radius(id, p)) < 1e-9);
                    CHECK(*top <= t.top_z(id) + 1e-12);
                    highest = std::max(highest, *top);
                }
                // The bottom: the lowest point inside the column (mirror: the column top of -z is not exposed, so scan).
                for (int s = 0; s <= 200; ++s) {
                    const Vec3 p{x, y, -1.0 + 2.0 * s / 200.0};
                    if (norm(p) > 1e-9 && norm(p) <= t.radius(id, p)) {
                        lowest = std::min(lowest, p.z);
                        break;
                    }
                }
            }
        }
        CHECK(highest > t.top_z(id) - 0.05);
        CHECK(lowest >= t.bottom_z(id) - 1e-9);
        CHECK(lowest < t.bottom_z(id) + 0.05);
        CHECK(highest <= t.top_z(id) + 1e-12);
        // top_z / bottom_z (the exact support: an enumeration of the extreme points) against an independent search on
        // the surface: no surface point lies beyond them, and the search (which can stall on a ridge of the
        // polyhedron) gets close to them.
        const auto [z_lo, z_hi] = refined_z_extent(id);
        CHECK(z_hi <= t.top_z(id) + 1e-9);
        CHECK(z_lo >= t.bottom_z(id) - 1e-9);
        CHECK(z_hi > t.top_z(id) - 2e-3);
        CHECK(z_lo < t.bottom_z(id) + 2e-3);
        worst_extent_gap = std::max({worst_extent_gap, t.top_z(id) - z_hi, z_lo - t.bottom_z(id)});
        // support_point along tilted directions: on the surface, no surface point beyond it, and the contact normal of
        // a pad with normal u pressing there is u (u lies in the normal cone at the support point: face, edge or vertex).
        for (int k = 0; k < 3; ++k) {
            const auto ku = static_cast<std::uint64_t>(k);
            const double z = 0.2 + 0.8 * unit_hash(id, ku, 11), phi = constants::two_pi * unit_hash(id, ku, 12);
            const Vec3 u{std::sqrt(1.0 - z * z) * std::cos(phi), std::sqrt(1.0 - z * z) * std::sin(phi), z};
            const Vec3 p = t.support_point(id, u);
            CHECK(std::abs(norm(p) - t.radius(id, p)) < 1e-9);
            const double searched =
                sphere_max([&](const Vec3& d) { return dot(u, d) * t.radius(id, d); }, 1000 + 8 * static_cast<std::uint64_t>(id) + ku);
            CHECK(searched <= dot(u, p) + 1e-9);
            CHECK(searched > dot(u, p) - 2e-3);
            CHECK(norm(t.normal_toward(id, p, u, 1e-9) - u) < 1e-6);
        }
    }
    std::printf("vertical extent: the independent search stays within %.2e of the exact support\n", worst_extent_gap);
}

TEST_CASE("Pad contact normal on hand-built vertices and faces", "[moon_surface][environment][contact]") {
    // Review m1: expectations independent of the code under test. A vertex is solved here from three of the template's
    // planes (Cramer's rule) and checked inside every other plane and the ellipsoid; a pad whose normal lies inside the
    // three faces' cone (their normalised sum) rests on that vertex with its own normal. A face point is the foot of the
    // origin on a plane, strictly inside the rest; a pad tilted 0.3 rad from that face gets the face normal.
    const RockTemplates& t = RockTemplates::instance();
    const auto inside = [&](int id, const Vec3& p, int a, int b, int c) {
        for (int k = 0; k < t.plane_count(id); ++k) {
            if (k == a || k == b || k == c) continue;
            if (dot(t.plane_normal(id, k), p) > t.plane_distance(id, k) - 1e-6) return false;
        }
        return dot(p, t.quadric(id) * p) < 1.0 - 1e-6;
    };
    const auto det = [](const Vec3& a, const Vec3& b, const Vec3& c) { return dot(a, cross(b, c)); };
    int vertices = 0, faces = 0;
    for (int id = 0; id < RockTemplates::count; ++id) {
        if (t.eroded(id)) continue;
        const int n = t.plane_count(id);
        for (int a = 0; a < n && vertices < 4 * (id + 1); ++a) {
            const Vec3 na = t.plane_normal(id, a);
            const Vec3 face_point = na * t.plane_distance(id, a);
            if (inside(id, face_point, a, -1, -1)) {
                const Vec3 side = math::normalized(cross(na, std::abs(na.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}));
                const Vec3 pad = math::normalized(na * std::cos(0.3) + side * std::sin(0.3));
                CHECK(norm(t.normal_toward(id, face_point, pad, 1e-9) - na) < 1e-12);
                ++faces;
            }
            for (int b = a + 1; b < n; ++b) {
                for (int c = b + 1; c < n; ++c) {
                    const Vec3 nb = t.plane_normal(id, b), nc = t.plane_normal(id, c);
                    // Rows na, nb, nc; right-hand side the distances.
                    const double d = det(na, nb, nc);
                    if (std::abs(d) < 1e-3) continue;
                    const Vec3 h{t.plane_distance(id, a), t.plane_distance(id, b), t.plane_distance(id, c)};
                    const Vec3 cx{na.x, nb.x, nc.x}, cy{na.y, nb.y, nc.y}, cz{na.z, nb.z, nc.z};
                    const Vec3 p{det(h, cy, cz) / d, det(cx, h, cz) / d, det(cx, cy, h) / d};
                    if (!inside(id, p, a, b, c)) continue;
                    const Vec3 pad = math::normalized(na + nb + nc);
                    CHECK(norm(t.normal_toward(id, p, pad, 1e-9) - pad) < 1e-9);
                    ++vertices;
                }
            }
        }
    }
    std::printf("hand-built contact normals: %d vertices, %d faces\n", vertices, faces);
    CHECK(vertices >= 20);
    CHECK(faces >= 10);
}

TEST_CASE("Rocks are deterministic and pole-regular", "[moon_surface][environment]") {
    const Vec3 pole = direction(-90.0 * deg, 0.0);
    const std::array<Vec3, 3> centres = {direction(site_lat, site_lon), pole, direction(30.0 * deg, 10.0 * deg)};
    for (const Vec3& c : centres) {
        const auto reference = rocks().rocks_near(c, 40.0, 0.05);
        REQUIRE(reference.size() > 100);
        std::array<std::vector<Rock>, 4> results;
        std::vector<std::thread> threads;
        for (size_t k = 0; k < results.size(); ++k) {
            threads.emplace_back([&, k] {
                const LunarRocks own(shared_detail());  // another instance: no shared state
                results[k] = (k % 2 == 0 ? own : rocks()).rocks_near(c, 40.0, 0.05);
            });
        }
        for (auto& th : threads) th.join();
        for (const auto& res : results) {
            REQUIRE(res.size() == reference.size());
            for (size_t i = 0; i < res.size(); ++i) CHECK(same_bits(res[i], reference[i]));
        }
        for (const Rock& r : reference) {
            CHECK(std::isfinite(r.base_height_m));
            CHECK(std::isfinite(r.burial_m));
            CHECK(std::isfinite(norm(r.base_unit)));
            CHECK(r.axes_m.x >= r.axes_m.y);
            CHECK(r.axes_m.y >= r.axes_m.z);
            CHECK(r.axes_m.z > 0.0);
            CHECK(r.axes_m.x <= rocks().max_semi_axis_per_diameter() * r.diameter_m);
            const Vec3 e_a = math::column(r.local_to_fixed, 0), e_b = math::column(r.local_to_fixed, 1),
                       up = math::column(r.local_to_fixed, 2);
            CHECK(std::abs(norm(e_a) - 1.0) < 1e-12);
            CHECK(std::abs(dot(e_a, e_b)) < 1e-12);
            CHECK(norm(cross(e_a, e_b) - up) < 1e-12);
        }
    }
    // Rock tops at the pole are finite too.
    for (const Rock& r : rocks().rocks_near(pole, 20.0, 0.2)) {
        if (const auto top = rocks().rock_top(r, r.base_unit)) {
            CHECK(std::isfinite(top->first));
            CHECK(std::isfinite(top->second.x + top->second.y + top->second.z));
        }
    }
}

TEST_CASE("Cover fraction equals the area of the generated rocks", "[moon_surface][environment]") {
    const auto& cfg = shared_detail()->config().rocks;
    // The configured range: 3.1 % of the area for 0.01-10 m, 0.68 % for 0.5-10 m (the Diviner reference).
    CHECK(std::abs(rocks().cover_fraction(0.01, 10.0) - 0.0311) < 0.0005);
    CHECK(std::abs(rocks().cover_fraction(cfg.abundance_reference_min_diameter_m, 10.0) -
                   cfg.abundance_reference_area_fraction) < 0.02 * cfg.abundance_reference_area_fraction);
    CHECK(rocks().cover_fraction(1.0, 0.5) == 0.0);
    CHECK(rocks().cover_fraction(20.0, 30.0) == 0.0);  // outside the law's range

    // The generated rocks' real plan-view area (part above the ground, measured with rock_top on the template
    // geometry) in a patch of ~8000 rocks of 0.05-0.2 m (population scatter of the area ~1.5 %, 1 sigma).
    const double lo = 0.05, hi = 0.2, patch_radius = 43.0;
    const Vec3 centre = direction(site_lat, site_lon);
    double footprint = 0.0, discs = 0.0;
    size_t count = 0;
    for (const Rock& r : rocks().rocks_near(centre, patch_radius, lo)) {
        if (r.diameter_m >= hi) continue;
        footprint += measured_footprint_m2(rocks(), r, 64);
        discs += 0.25 * constants::pi * r.diameter_m * r.diameter_m;
        ++count;
    }
    const double patch = constants::pi * patch_radius * patch_radius;
    const double measured = footprint / patch;
    const double expected = rocks().cover_fraction(lo, hi);
    std::printf("cover fraction %.2f-%.2f m: %zu rocks, measured footprint %.5f, formula %.5f (%+.2f %%); "
                "footprint / D^2 discs %.4f\n",
                lo, hi, count, measured, expected, 100.0 * (measured / expected - 1.0), footprint / discs);
    std::printf("largest semi-axis a per diameter D: %.3f\n", rocks().max_semi_axis_per_diameter());
    CHECK(count > 7000);
    CHECK(std::abs(measured - expected) <= 0.05 * expected);
    // Geometry alone (no population scatter): each rock covers pi D^2 / 4, D being its area-equivalent diameter.
    CHECK(std::abs(footprint / discs - 1.0) < 0.02);
}

TEST_CASE("The abundance map scales the population and blends into the uniform law", "[moon_surface][environment]") {
    const auto& cfg = shared_detail()->config().rocks;
    const double ref = cfg.abundance_reference_area_fraction;
    const LunarRocks mapped(diviner_detail(), constant_map(3.0 * ref));
    CHECK(rocks().abundance_factor(direction(0.0, 0.3)) == cfg.abundance_scale);  // no map: the uniform law
    CHECK(std::abs(mapped.abundance_factor(direction(0.0, 0.3)) - 3.0 * cfg.abundance_scale) < 1e-6);  // the map stores floats
    CHECK(std::abs(mapped.abundance_factor(direction(-54.0 * deg, 2.0)) - 3.0 * cfg.abundance_scale) < 1e-6);
    CHECK(mapped.abundance_factor(direction(-60.5 * deg, 2.0)) == cfg.abundance_scale);
    CHECK(mapped.abundance_factor(direction(-84.8 * deg, 0.5)) == cfg.abundance_scale);
    // No step at the map edge: the factor is monotone and moves by < 0.4 % of the jump per 0.01 deg.
    double previous = mapped.abundance_factor(direction(-54.0 * deg, 2.0));
    double max_step = 0.0;
    for (int k = 1; k <= 800; ++k) {
        const double f = mapped.abundance_factor(direction((-54.0 - 0.01 * k) * deg, 2.0));
        CHECK(f <= previous + 1e-12);
        max_step = std::max(max_step, previous - f);
        previous = f;
    }
    CHECK(previous == cfg.abundance_scale);
    CHECK(max_step < 0.004 * 2.0 * cfg.abundance_scale);

    // Counts follow the factor: 3 x the uniform population on the map, 1 x outside.
    const double lo = 0.01, hi = 0.02;
    const double per_m2 = cfg.sfd.cumulative_density_per_m2(lo) - cfg.sfd.cumulative_density_per_m2(hi);
    const double radius = std::sqrt(500.0 / per_m2 / constants::pi);
    for (const auto& [lat, factor] : {std::pair{10.0 * deg, 3.0}, std::pair{-70.0 * deg, 1.0}}) {
        const double expected = factor * per_m2 * constants::pi * radius * radius;
        double n = 0.0;
        for (const Rock& r : mapped.rocks_near(direction(lat, 1.0), radius, lo)) n += r.diameter_m < hi ? 1.0 : 0.0;
        CHECK(std::abs(n - expected) <= 3.0 * std::sqrt(expected));
    }
}

TEST_CASE("abundance_source selects the abundance law", "[moon_surface][environment]") {
    const auto& cfg = shared_detail()->config().rocks;
    // "uniform": a passed map is ignored.
    const LunarRocks uniform(shared_detail(), constant_map(3.0 * cfg.abundance_reference_area_fraction));
    CHECK(uniform.abundance_factor(direction(0.0, 0.3)) == cfg.abundance_scale);
    // "diviner" without a map: German error naming the fetch target.
    try {
        const LunarRocks missing(diviner_detail());
        FAIL("no exception");
    } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).find("fetch.py diviner_rock_abundance") != std::string::npos);
    }
    // A map that is entirely no data has nothing to fill from: the uniform law.
    auto empty = std::make_shared<environment::RockAbundanceMap>(*constant_map(1.0));
    std::fill(empty->area_fraction.begin(), empty->area_fraction.end(), std::numeric_limits<float>::quiet_NaN());
    const LunarRocks none(diviner_detail(), empty);
    CHECK(none.abundance_factor(direction(30.0 * deg, 1.0)) == cfg.abundance_scale);
}

TEST_CASE("Diviner no-data gaps are filled: the abundance factor has no step", "[moon_surface][environment]") {
    const auto& cfg = shared_detail()->config().rocks;
    // A smooth synthetic map (0.5 .. 1.5 x reference, periodic in longitude) with a hole of 10 x 20
    // pixels and a single missing pixel.
    auto map = std::make_shared<environment::RockAbundanceMap>(*constant_map(1.0));
    const double ref = cfg.abundance_reference_area_fraction;
    const auto at = [&](int l, int s) -> float& { return map->area_fraction[static_cast<size_t>(l * map->samples + s)]; };
    for (int l = 0; l < map->lines; ++l) {
        for (int s = 0; s < map->samples; ++s) {
            at(l, s) = static_cast<float>(ref * (1.0 + 0.5 * std::sin(0.05 * l) * std::cos(constants::two_pi * 3.0 * s / map->samples)));
        }
    }
    float lo = std::numeric_limits<float>::max(), hi = 0.0f;
    double max_pixel_step = 0.0;  // the map's own pixel-to-pixel variation
    for (int l = 0; l < map->lines; ++l) {
        for (int s = 0; s < map->samples; ++s) {
            lo = std::min(lo, at(l, s));
            hi = std::max(hi, at(l, s));
            if (s + 1 < map->samples) max_pixel_step = std::max(max_pixel_step, double(std::abs(at(l, s + 1) - at(l, s))));
        }
    }
    const auto nan = std::numeric_limits<float>::quiet_NaN();
    for (int l = 40; l < 50; ++l) {
        for (int s = 100; s < 120; ++s) at(l, s) = nan;
    }
    at(80, 30) = nan;
    environment::fill_rock_abundance_gaps(*map);
    for (const float v : map->area_fraction) {
        REQUIRE(!std::isnan(v));
        CHECK(v >= lo);
        CHECK(v <= hi);
    }
    const LunarRocks with_map(diviner_detail(), map);
    // Sweep along longitude through the hole (line 45) and the single pixel (line 80) in 1/8-pixel steps; the factor's
    // change between neighbours stays within the map's own pixel step (as a factor).
    const double bound = cfg.abundance_scale * max_pixel_step / ref;
    const double lat_hole = (60.0 - 45.5 * 120.0 / map->lines) * deg;
    const double lat_single = (60.0 - 80.5 * 120.0 / map->lines) * deg;
    for (const double lat : {lat_hole, lat_single}) {
        double previous = with_map.abundance_factor(direction(lat, 0.0));
        for (int k = 1; k <= 8 * map->samples; ++k) {
            const double f = with_map.abundance_factor(direction(lat, constants::two_pi * k / (8.0 * map->samples)));
            CHECK(std::abs(f - previous) <= bound);
            previous = f;
        }
    }
}

TEST_CASE("The Diviner rock abundance file loads into the map", "[moon_surface][environment]") {
    // A small file in the real format (tools/data_pipeline/rock_abundance.py): 0.1 px/deg, 12 x 36, +-60 deg.
    const auto dir = std::filesystem::temp_directory_path() / "starsim_rock_abundance_load_test";
    std::filesystem::remove_all(dir);
    std::filesystem::create_directories(dir);
    constexpr int lines = 12, samples = 36, missing_dn = -32768;
    std::vector<std::int16_t> dn(lines * samples);
    for (int l = 0; l < lines; ++l) {
        for (int s = 0; s < samples; ++s) dn[l * samples + s] = static_cast<std::int16_t>(10 * l + s % 3);
    }
    dn[5 * samples + 7] = missing_dn;
    {
        std::ofstream values(dir / "diviner_rock_abundance.i16", std::ios::binary);
        values.write(reinterpret_cast<const char*>(dn.data()), static_cast<std::streamsize>(2 * dn.size()));
        std::ofstream header(dir / config::rock_abundance_file_name);
        header << "[grid]\nproduct = \"TEST\"\nprojection = \"simple_cylindrical\"\nlines = 12\nsamples = 36\n"
                  "pixels_per_degree = 0.1\nzero_line = 5.5\nzero_sample = -0.5\nmax_latitude_deg = 60.0\n"
                  "dn_scale = 0.001\nmissing_dn = -32768\nvalues = \"diviner_rock_abundance.i16\"\n";
    }
    auto diviner_cfg = load_config("diviner").rocks;
    const auto map = environment::load_rock_abundance_map(diviner_cfg, dir);
    REQUIRE(map);
    CHECK(map->lines == lines);
    CHECK(map->samples == samples);
    CHECK(std::abs(map->lat_limit_rad - 60.0 * deg) < 1e-12);
    // Pixel centre (line 2, sample 4): latitude (5.5 - 2) / 0.1 = 35 deg, longitude (4 + 0.5) / 0.1 = 45 deg.
    CHECK(std::abs(map->at(35.0 * deg, 45.0 * deg) - 0.001 * (10 * 2 + 4 % 3)) < 1e-6);
    // Longitude wraps: between sample 35 (355 deg) and sample 0 (5 deg).
    CHECK(std::abs(map->at(-5.0 * deg, 0.0) - 0.001 * 0.5 * ((10 * 6 + 35 % 3) + (10 * 6 + 0))) < 1e-6);
    // The missing pixel (line 5, sample 7) is filled at load.
    CHECK(!std::isnan(map->area_fraction[5 * samples + 7]));
    // The loader fills it with the mean of the 8 pixels around it.
    double ring = 0.0;
    for (int dl = -1; dl <= 1; ++dl) {
        for (int ds = -1; ds <= 1; ++ds) {
            if (dl != 0 || ds != 0) ring += 0.001 * (10 * (5 + dl) + (7 + ds) % 3);
        }
    }
    CHECK(std::abs(map->at(5.0 * deg, 75.0 * deg) - ring / 8.0) < 1e-6);  // its centre (5 deg, 75 deg)
    // "uniform" reads nothing; "diviner" with no file names fetch.py.
    diviner_cfg.abundance_source = "uniform";
    CHECK(environment::load_rock_abundance_map(diviner_cfg, dir / "absent") == nullptr);
    diviner_cfg.abundance_source = "diviner";
    try {
        (void)environment::load_rock_abundance_map(diviner_cfg, dir / "absent");
        FAIL("no exception");
    } catch (const config::ConfigError& e) {
        CHECK(std::string(e.what()).find("fetch.py diviner_rock_abundance") != std::string::npos);
    }
    std::filesystem::remove_all(dir);
}

TEST_CASE("Craters, roughness and rock tops sum to the measured total roughness", "[moon_surface][environment][slow]") {
    const double target = shared_detail()->config().roughness.rms_height_measured_total_m;
    constexpr double window = 0.125;
    // The site, and a cube corner (three faces meet): the +6 % roughness of the skewed lattice found in Task 2.
    const Vec3 corner = math::normalized(Vec3{1.0, 1.0, 1.0});
    const struct {
        const char* name;
        Vec3 centre;
        double patch_m;
    } places[] = {{"site", direction(site_lat, site_lon), 2000.0}, {"cube corner", corner, 2000.0}};
    for (const auto& p : places) {
        const double without = median_total_rms(p.centre, p.patch_m, window, 600, false);
        const double with = median_total_rms(p.centre, p.patch_m, window, 600, true);
        std::printf("%-11s craters + roughness %.3f mm, + rock tops %.3f mm, measured total %.3f mm (%.1f %%)\n", p.name,
                    1e3 * without, 1e3 * with, 1e3 * target, 100.0 * (with / target - 1.0));
        CHECK(std::abs(with - target) <= 0.10 * target);
    }
}
