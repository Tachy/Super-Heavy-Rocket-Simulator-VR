#include "starsim/environment/detailed_lunar_terrain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/rock_templates.hpp"
#include "starsim/math/constants.hpp"
#include "parallel_batch.hpp"

namespace starsim::environment {

using math::Vec3;

namespace {

constexpr double nan = std::numeric_limits<double>::quiet_NaN();
constexpr double lattice_nodes_per_edge = static_cast<double>(std::int64_t{1} << DetailedLunarTerrain::contact_lattice_level);
constexpr int patch_shift = DetailedLunarTerrain::contact_lattice_level - DetailedLunarTerrain::contact_patch_level;
constexpr std::int64_t patch_nodes = std::int64_t{1} << patch_shift;  // per side
constexpr std::int64_t patch_mask = patch_nodes - 1;
// A cell's points lie within its largest centre-to-corner chord; 1 % margin for the edges' curvature ([E]).
constexpr double patch_radius_margin = 1.01;
// Rim arc search for a rock cut by the disc rim: coarse samples over the arc the rock can cover, then golden-section
// steps around the best ([E]; Task 17: the fractured templates' top is piecewise planar with many small facets, so the
// arc is sampled finer (9 coarse samples, 12 golden steps: bracket ~0.1 % of the span): the 5 / 8 of the smooth convex templates left the
// maximum up to 0.1 mm low on a neighbouring facet, whose normal differed by up to 0.19 rad).
constexpr int arc_coarse_samples = 9;
constexpr int arc_golden_steps = 12;
// Node climb over a rock's footprint columns: at most this many steps ([E]: 96 columns across, a climb from the peak
// node takes a few).
constexpr int max_climb_steps = 200;
// Template peak search: grid over the unit disc, then a pattern search to this step ([E]).
constexpr int peak_grid = 41;
constexpr double peak_min_step = 1e-6;
constexpr int peak_max_iterations = 400;

Vec3 direction_of(double latitude_rad, double longitude_rad) {
    return {std::cos(latitude_rad) * std::cos(longitude_rad), std::cos(latitude_rad) * std::sin(longitude_rad),
            std::sin(latitude_rad)};
}

struct Tangent {
    Vec3 e1, e2;
};
Tangent tangent_of(const Vec3& up) {
    const Vec3 e1 = math::normalized(cross(std::abs(up.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, up));
    return {e1, cross(up, e1)};
}

// Position (x, y) of each template's highest column top in its unit coordinates.
struct TemplatePeak {
    double x = 0.0, y = 0.0;
};
const std::array<TemplatePeak, RockTemplates::count>& template_peaks() {
    static const auto peaks = [] {
        const RockTemplates& templates = RockTemplates::instance();
        std::array<TemplatePeak, RockTemplates::count> out{};
        for (int id = 0; id < RockTemplates::count; ++id) {
            const auto top = [&](double x, double y) {
                if (x * x + y * y >= 1.0) return -std::numeric_limits<double>::infinity();
                const auto z = templates.column_top(id, x, y);
                return z ? *z : -std::numeric_limits<double>::infinity();
            };
            double bx = 0.0, by = 0.0, bz = top(0.0, 0.0);
            for (int a = 0; a < peak_grid; ++a) {
                for (int b = 0; b < peak_grid; ++b) {
                    const double x = -1.0 + 2.0 * a / (peak_grid - 1), y = -1.0 + 2.0 * b / (peak_grid - 1);
                    const double z = top(x, y);
                    if (z > bz) bx = x, by = y, bz = z;
                }
            }
            double step = 2.0 / (peak_grid - 1);
            for (int it = 0; it < peak_max_iterations && step > peak_min_step; ++it) {
                bool moved = false;
                for (const auto& [dx, dy] : {std::pair{1.0, 0.0}, std::pair{-1.0, 0.0}, std::pair{0.0, 1.0}, std::pair{0.0, -1.0}}) {
                    const double z = top(bx + dx * step, by + dy * step);
                    if (z > bz) {
                        bx += dx * step, by += dy * step, bz = z;
                        moved = true;
                        break;
                    }
                }
                if (!moved) step *= 0.5;
            }
            out[static_cast<size_t>(id)] = {bx, by};
        }
        return out;
    }();
    return peaks;
}

// LunarRocks::rock_top's height (the template's exact column top, RockTemplates::column_top): the rock top of the
// contact surface. Height only.
std::optional<double> contact_top(const Rock& rock, const Vec3& unit, double sphere_radius, double* ux_out = nullptr,
                                  double* uy_out = nullptr, double* top_out = nullptr) {
    const Vec3 offset = (unit - rock.base_unit) * sphere_radius;
    const double x = dot(offset, math::column(rock.local_to_fixed, 0)), y = dot(offset, math::column(rock.local_to_fixed, 1));
    const double ux = x / rock.axes_m.x, uy = y / rock.axes_m.y;
    if (ux * ux + uy * uy >= 1.0) return std::nullopt;
    const auto top = RockTemplates::instance().column_top(rock.template_id, ux, uy);
    if (!top) return std::nullopt;
    const double above_base = rock.axes_m.z * *top - rock.burial_m;
    if (above_base <= 0.0) return std::nullopt;
    if (ux_out) *ux_out = ux, *uy_out = uy, *top_out = *top;
    return rock.base_height_m + above_base + (x * x + y * y) / (2.0 * sphere_radius);
}

// The rock's outward unit normal (body-fixed) at the unit-size surface point (ux, uy, top), as LunarRocks::rock_top.
Vec3 contact_top_normal(const Rock& rock, double ux, double uy, double top) {
    const Vec3 n_unit = RockTemplates::instance().normal(rock.template_id, Vec3{ux, uy, top});
    const Vec3 n_local = math::normalized({n_unit.x / rock.axes_m.x, n_unit.y / rock.axes_m.y, n_unit.z / rock.axes_m.z});
    return rock.local_to_fixed * n_local;
}

struct PatchKey {
    int face = 0;
    std::int64_t i = 0, j = 0;
    friend bool operator==(const PatchKey&, const PatchKey&) = default;
};
struct PatchKeyHash {
    size_t operator()(const PatchKey& k) const {
        auto h = static_cast<std::uint64_t>(k.i) * 0x9E3779B97F4A7C15ULL;
        h ^= static_cast<std::uint64_t>(k.j) + 0x632BE59BD9B4E019ULL + (h << 6) + (h >> 2);
        h ^= static_cast<std::uint64_t>(k.face) + 0x94D049BB133111EBULL + (h << 6) + (h >> 2);
        return static_cast<size_t>(h);
    }
};

// Drops the least recently used entry beyond `limit`.
template <typename Map>
void evict(Map& map, size_t limit) {
    while (map.size() > limit) {
        auto oldest = map.begin();
        for (auto it = map.begin(); it != map.end(); ++it) {
            if (it->second.used < oldest->second.used) oldest = it;
        }
        map.erase(oldest);
    }
}

}  // namespace

struct DetailedLunarTerrain::ContactCache {
    struct GroundPatch {
        std::vector<double> nodes = std::vector<double>(static_cast<size_t>(patch_nodes * patch_nodes), nan);
        std::uint64_t used = 0;
    };
    struct RockEntry {
        Rock rock;
        bool peak_valid = false;
        Vec3 peak_unit;
        double peak_height_m = 0.0;
        Vec3 peak_normal;
        double top_bound_m = 0.0;  // no point of the rock's top lies higher (peak + a^2 / 2R)
    };
    struct RockPatch {
        std::vector<RockEntry> rocks;
        double max_peak_m = -std::numeric_limits<double>::infinity();  // max top_bound_m
    };
    struct RockSlot {
        std::shared_ptr<const RockPatch> patch;
        std::uint64_t used = 0;
    };

    std::mutex mutex;
    std::uint64_t clock = 0;
    std::unordered_map<PatchKey, GroundPatch, PatchKeyHash> ground;
    std::unordered_map<PatchKey, RockSlot, PatchKeyHash> rocks;
};

DetailedLunarTerrain::DetailedLunarTerrain(std::shared_ptr<const LunarTerrain> lola,
                                           std::shared_ptr<const LunarSurfaceDetail> detail,
                                           std::shared_ptr<const LunarRocks> rocks, SurfaceDetailOptions options)
    : lola_(std::move(lola)), detail_(std::move(detail)), rocks_(std::move(rocks)), options_(options),
      cache_(std::make_unique<ContactCache>()) {
    if (!lola_) throw std::invalid_argument("DetailedLunarTerrain: kein Mondgelaende (LOLA) uebergeben");
    if (options_.detail && !detail_) throw std::invalid_argument("DetailedLunarTerrain: Oberflaechendetail fehlt");
    if (rocks_on() && !rocks_) throw std::invalid_argument("DetailedLunarTerrain: Felsbrocken fehlen");
    sphere_radius_m_ = lola_->sphere_radius_m();
    max_height_m_ = lola_->max_height_m();
    if (options_.detail) {
        // Relief bound (practical, for the terrain horizon march): two fresh rims per crater octave on top of each
        // other, every octave's mean-field shift, and the largest value-noise sum (|node| <= 1, unit_variance_gain
        // sqrt 3, face blend sum of weights <= sqrt 3).
        double bound = 0.0;
        const auto& cfg = detail_->config();
        for (const auto& o : detail_->crater_octaves()) {
            bound += 2.0 * cfg.craters.rim_height_to_diameter * o.max_diameter_m + std::abs(o.mean_relief_m);
        }
        for (const auto& o : detail_->roughness_octaves()) {
            bound += 3.0 * detail_->roughness_scale_m() *
                     std::pow(o.spacing_m / cfg.roughness.baseline_m, cfg.roughness.hurst_exponent);
        }
        max_height_m_ += bound;
        if (rocks_on()) {
            // A rock stands at most its full height c (top_z - bottom_z <= 2) above its base, c <= a.
            max_height_m_ += 2.0 * rocks_->max_semi_axis_per_diameter() * cfg.rocks.sfd.max_diameter_m;
        }
    }
}

DetailedLunarTerrain::~DetailedLunarTerrain() = default;

double DetailedLunarTerrain::height_limited_m(const Vec3& unit, double min_wavelength_m,
                                              double min_rock_diameter_m) const {
    if (!options_.detail) return lola_->height_at_direction_m(unit);
    double h = lola_->height_at_direction_m(unit) + detail_->relief_m(unit, min_wavelength_m);
    if (rocks_on()) {
        for (const Rock& rock : rocks_->rocks_touching(unit, 0.0, min_rock_diameter_m)) {
            if (const auto top = rocks_->rock_top(rock, unit)) h = std::max(h, top->first);
        }
    }
    return h;
}

void DetailedLunarTerrain::heights_limited_m(const std::vector<Vec3>& units, double min_wavelength_m,
                                             double min_rock_diameter_m, std::vector<double>& heights_m) const {
    if (heights_m.size() != units.size()) {
        throw std::invalid_argument("DetailedLunarTerrain::heights_limited_m: Richtungen und Hoehen verschieden lang");
    }
    starsim::environment::detail::for_each_index_budgeted(static_cast<int>(units.size()), [&](int k) {
        const auto i = static_cast<size_t>(k);
        heights_m[i] = height_limited_m(units[i], min_wavelength_m, min_rock_diameter_m);
    });
}

double DetailedLunarTerrain::full_height_m(const Vec3& unit) const { return height_limited_m(unit, 0.0, 0.0); }

double DetailedLunarTerrain::height_at_direction_m(const Vec3& unit) const {
    return options_.detail ? full_height_m(unit) : lola_->height_at_direction_m(unit);
}

double DetailedLunarTerrain::height_m(double latitude_rad, double longitude_rad) const {
    return options_.detail ? full_height_m(direction_of(latitude_rad, longitude_rad))
                           : lola_->height_m(latitude_rad, longitude_rad);
}

double DetailedLunarTerrain::height_at_fixed_m(const Vec3& fixed_m) const {
    return options_.detail ? full_height_m(math::normalized(fixed_m)) : lola_->height_at_fixed_m(fixed_m);
}

double DetailedLunarTerrain::normal_step_m(const Vec3& /*fixed_m*/) const {
    return options_.detail ? detail_normal_step_m : LunarTerrain::normal_step_m;
}

Vec3 DetailedLunarTerrain::normal_fixed(const Vec3& fixed_m) const {
    if (!options_.detail) return lola_->normal_fixed(fixed_m);
    const Vec3 unit = math::normalized(fixed_m);
    const Tangent t = tangent_of(unit);
    const double step = normal_step_m(fixed_m);
    const auto point = [&](const Vec3& offset_m) {
        const Vec3 dir = math::normalized(unit + offset_m / sphere_radius_m_);
        return dir * (sphere_radius_m_ + full_height_m(dir));
    };
    const Vec3 along1 = point(t.e1 * step) - point(t.e1 * -step);
    const Vec3 along2 = point(t.e2 * step) - point(t.e2 * -step);
    const Vec3 n = math::normalized(cross(along1, along2));
    return dot(n, unit) >= 0.0 ? n : -n;
}

double DetailedLunarTerrain::resolution_m(double latitude_rad, double longitude_rad) const {
    return options_.detail ? detail_->finest_wavelength_m() : lola_->resolution_m(latitude_rad, longitude_rad);
}

double DetailedLunarTerrain::finest_resolution_within_m(double latitude_rad, double longitude_rad,
                                                        double radius_m) const {
    return options_.detail ? detail_->finest_wavelength_m()
                           : lola_->finest_resolution_within_m(latitude_rad, longitude_rad, radius_m);
}

std::optional<std::pair<double, Vec3>> DetailedLunarTerrain::contact_rock_top(const Rock& rock, const Vec3& unit) const {
    double ux = 0.0, uy = 0.0, top = 0.0;
    const auto h = contact_top(rock, unit, sphere_radius_m_, &ux, &uy, &top);
    if (!h) return std::nullopt;
    return std::pair{*h, contact_top_normal(rock, ux, uy, top)};
}

const LunarSurfaceDetail& DetailedLunarTerrain::detail() const {
    if (!detail_) throw std::logic_error("DetailedLunarTerrain: kein Oberflaechendetail geladen");
    return *detail_;
}

const LunarRocks& DetailedLunarTerrain::rocks() const {
    if (!rocks_) throw std::logic_error("DetailedLunarTerrain: keine Felsbrocken geladen");
    return *rocks_;
}

double DetailedLunarTerrain::contact_ground_m(const Vec3& unit) const {
    double out = 0.0;
    contact_ground_batch(&unit, 1, &out);
    return out;
}

void DetailedLunarTerrain::contact_ground_batch(const Vec3* units, int count, double* out) const {
    if (!options_.detail) {
        for (int i = 0; i < count; ++i) out[i] = lola_->height_at_direction_m(units[i]);
        return;
    }
    constexpr int chunk = 32;
    struct Sample {
        int face = 0;
        std::int64_t j0 = 0, k0 = 0;
        double fu = 0.0, fv = 0.0;
        std::array<double, 4> node{};  // (j0, k0), (j0 + 1, k0), (j0, k0 + 1), (j0 + 1, k0 + 1)
    };
    std::array<Sample, chunk> samples;
    ContactCache& c = *cache_;
    const auto node_jk = [](const Sample& s, int n) { return std::pair{s.j0 + (n & 1), s.k0 + (n >> 1)}; };
    for (int first = 0; first < count; first += chunk) {
        const int n_samples = std::min(chunk, count - first);
        for (int i = 0; i < n_samples; ++i) {
            const FaceCoord fc = face_coord_of(units[first + i]);
            const double u = (fc.s + 1.0) * 0.5 * lattice_nodes_per_edge, v = (fc.t + 1.0) * 0.5 * lattice_nodes_per_edge;
            Sample& s = samples[static_cast<size_t>(i)];
            s.face = fc.face;
            s.j0 = static_cast<std::int64_t>(std::floor(u));
            s.k0 = static_cast<std::int64_t>(std::floor(v));
            s.fu = u - static_cast<double>(s.j0);
            s.fv = v - static_cast<double>(s.k0);
        }
        bool missing = false;
        {
            std::lock_guard lock(c.mutex);
            PatchKey last_key{-1, 0, 0};
            ContactCache::GroundPatch* patch = nullptr;
            for (int i = 0; i < n_samples; ++i) {
                Sample& s = samples[static_cast<size_t>(i)];
                for (int n = 0; n < 4; ++n) {
                    const auto [j, k] = node_jk(s, n);
                    const PatchKey key{s.face, j >> patch_shift, k >> patch_shift};
                    if (!patch || !(key == last_key)) {
                        patch = &c.ground[key];
                        patch->used = ++c.clock;
                        last_key = key;
                    }
                    const double value = patch->nodes[static_cast<size_t>((j & patch_mask) * patch_nodes + (k & patch_mask))];
                    s.node[static_cast<size_t>(n)] = value;
                    missing = missing || std::isnan(value);
                }
            }
        }
        if (missing) {
            // Computed outside the lock (a pure function of the node: whoever stores it stores the same value).
            for (int i = 0; i < n_samples; ++i) {
                Sample& s = samples[static_cast<size_t>(i)];
                for (int n = 0; n < 4; ++n) {
                    if (!std::isnan(s.node[static_cast<size_t>(n)])) continue;
                    const auto [j, k] = node_jk(s, n);
                    const double ss = -1.0 + 2.0 * static_cast<double>(j) / lattice_nodes_per_edge;
                    const double tt = -1.0 + 2.0 * static_cast<double>(k) / lattice_nodes_per_edge;
                    s.node[static_cast<size_t>(n)] = detail_->relief_m(face_direction(s.face, ss, tt), 0.0);
                }
            }
            std::lock_guard lock(c.mutex);
            for (int i = 0; i < n_samples; ++i) {
                const Sample& s = samples[static_cast<size_t>(i)];
                for (int n = 0; n < 4; ++n) {
                    const auto [j, k] = node_jk(s, n);
                    auto& patch = c.ground[{s.face, j >> patch_shift, k >> patch_shift}];
                    patch.used = ++c.clock;
                    patch.nodes[static_cast<size_t>((j & patch_mask) * patch_nodes + (k & patch_mask))] =
                        s.node[static_cast<size_t>(n)];
                }
            }
            evict(c.ground, static_cast<size_t>(max_cached_patches));
        }
        for (int i = 0; i < n_samples; ++i) {
            const Sample& s = samples[static_cast<size_t>(i)];
            const auto& node = s.node;
            const double relief = (1.0 - s.fv) * ((1.0 - s.fu) * node[0] + s.fu * node[1]) +
                                  s.fv * ((1.0 - s.fu) * node[2] + s.fu * node[3]);
            out[first + i] = lola_->height_at_direction_m(units[first + i]) + relief;
        }
    }
}

struct DetailedLunarTerrain::RockQuery {
    std::vector<const ContactCache::RockEntry*> candidates;           // base within radius + a (unsorted, unique)
    std::vector<std::shared_ptr<const ContactCache::RockPatch>> held;  // keeps the entries alive
    double max_peak_m = -std::numeric_limits<double>::infinity();      // of the cells searched (incl. a^2 / 2R)
};

void DetailedLunarTerrain::gather_rocks(const Vec3& centre_unit, double radius_m, RockQuery& out,
                                        bool with_candidates) const {
    ContactCache& c = *cache_;
    // Each cell holds every rock that may touch it: a point needs only its own cell.
    const auto visit = [&](const FaceCell& cell) {
        const PatchKey key{cell.face, cell.i, cell.j};
        std::shared_ptr<const ContactCache::RockPatch> patch;
        {
            std::lock_guard lock(c.mutex);
            if (auto it = c.rocks.find(key); it != c.rocks.end()) {
                it->second.used = ++c.clock;
                patch = it->second.patch;
            }
        }
        if (!patch) {
            const Vec3 centre = face_cell_point(cell, 0.5, 0.5);
            double reach = 0.0;
            for (const auto& [a, b] : {std::pair{0.0, 0.0}, std::pair{1.0, 0.0}, std::pair{0.0, 1.0}, std::pair{1.0, 1.0}}) {
                reach = std::max(reach, sphere_radius_m_ * norm(face_cell_point(cell, a, b) - centre));
            }
            auto built = std::make_shared<ContactCache::RockPatch>();
            const auto& peaks = template_peaks();
            for (const Rock& rock : rocks_->rocks_touching(centre, patch_radius_margin * reach, contact_min_rock_diameter_m)) {
                ContactCache::RockEntry e;
                e.rock = rock;
                const TemplatePeak& p = peaks[static_cast<size_t>(rock.template_id)];
                const Vec3 offset = math::column(rock.local_to_fixed, 0) * (rock.axes_m.x * p.x) +
                                    math::column(rock.local_to_fixed, 1) * (rock.axes_m.y * p.y);
                e.peak_unit = math::normalized(rock.base_unit + offset / sphere_radius_m_);
                // No contact top lies above the template's highest point (+ the curvature term a^2 / 2R).
                const double highest = rock.axes_m.z * RockTemplates::instance().top_z(rock.template_id) -
                                       rock.burial_m;
                if (highest > 0.0) {
                    e.peak_valid = true;
                    e.top_bound_m = rock.base_height_m + highest + rock.axes_m.x * rock.axes_m.x / (2.0 * sphere_radius_m_);
                    built->max_peak_m = std::max(built->max_peak_m, e.top_bound_m);
                    double ux = 0.0, uy = 0.0, top = 0.0;
                    if (const auto h = contact_top(rock, e.peak_unit, sphere_radius_m_, &ux, &uy, &top)) {
                        e.peak_height_m = *h;
                        e.peak_normal = contact_top_normal(rock, ux, uy, top);
                    } else {
                        e.peak_height_m = -std::numeric_limits<double>::infinity();
                    }
                }
                built->rocks.push_back(e);
            }
            std::lock_guard lock(c.mutex);
            auto& slot = c.rocks[key];
            if (!slot.patch) slot.patch = std::move(built);
            slot.used = ++c.clock;
            patch = slot.patch;
            evict(c.rocks, static_cast<size_t>(max_cached_patches));
        }
        out.max_peak_m = std::max(out.max_peak_m, patch->max_peak_m);
        if (!with_candidates) return;
        for (const auto& e : patch->rocks) {
            if (e.peak_valid && sphere_radius_m_ * norm(e.rock.base_unit - centre_unit) <= radius_m + e.rock.axes_m.x) {
                out.candidates.push_back(&e);
            }
        }
        out.held.push_back(std::move(patch));
    };
    if (radius_m > 0.0) {
        for (const FaceCell& cell : face_cells_within(centre_unit, contact_patch_level, radius_m, sphere_radius_m_)) visit(cell);
    } else {
        visit(face_cell_of(centre_unit, contact_patch_level));
    }
    // A rock touching several cells is listed by each: once (a fixed order by id).
    std::sort(out.candidates.begin(), out.candidates.end(),
              [](const auto* a, const auto* b) { return a->rock.id < b->rock.id; });
    out.candidates.erase(std::unique(out.candidates.begin(), out.candidates.end(),
                                     [](const auto* a, const auto* b) { return a->rock.id == b->rock.id; }),
                         out.candidates.end());
}

DetailedLunarTerrain::DiscSupport DetailedLunarTerrain::point_contact(const Vec3& unit, const Vec3& up_fixed,
                                                                      bool with_normal) const {
    double best = contact_ground_m(unit);
    Vec3 normal;
    if (with_normal) {
        const Vec3 up = math::normalized(up_fixed);
        const Tangent t = tangent_of(up);
        const auto at = [&](double x, double y) { return math::normalized(unit + (t.e1 * x + t.e2 * y) / sphere_radius_m_); };
        const double s = detail_normal_step_m;
        const double gx = (contact_ground_m(at(s, 0.0)) - contact_ground_m(at(-s, 0.0))) / (2.0 * s);
        const double gy = (contact_ground_m(at(0.0, s)) - contact_ground_m(at(0.0, -s))) / (2.0 * s);
        normal = math::normalized(up - t.e1 * gx - t.e2 * gy);
    }
    if (!rocks_on()) return {best, normal};
    // Every contact-size rock whose footprint covers the point: the highest top there (highest bound first).
    RockQuery q;
    gather_rocks(unit, 0.0, q);
    std::sort(q.candidates.begin(), q.candidates.end(), [](const auto* a, const auto* b) {
        return a->top_bound_m != b->top_bound_m ? a->top_bound_m > b->top_bound_m : a->rock.id < b->rock.id;
    });
    for (const auto* e : q.candidates) {
        if (e->top_bound_m <= best) break;
        double ux = 0.0, uy = 0.0, top = 0.0;
        if (const auto h = contact_top(e->rock, unit, sphere_radius_m_, &ux, &uy, &top); h && *h > best) {
            best = *h;
            if (with_normal) normal = contact_top_normal(e->rock, ux, uy, top);
        }
    }
    return {best, normal};
}

double DetailedLunarTerrain::contact_height_m(const Vec3& unit) const {
    if (!options_.detail) return lola_->height_at_direction_m(unit);
    return point_contact(unit, unit, false).height_m;
}

double DetailedLunarTerrain::support_upper_bound_m(const Vec3& centre_unit, const Vec3& /*up_fixed*/,
                                                   double radius_m) const {
    if (!options_.detail) return std::numeric_limits<double>::infinity();
    const double r = std::max(radius_m, 0.0);
    const double ground = contact_ground_m(centre_unit);
    double peaks = -std::numeric_limits<double>::infinity();
    if (rocks_on()) {
        RockQuery q;
        gather_rocks(centre_unit, r, q, false);
        peaks = q.max_peak_m;
    }
    if (r == 0.0) return std::max(ground, peaks);
    // support = max over the disc of (surface - plane slope . offset) <= max(surface) + |slope| r, where the ground
    // stays within bound_slope r of the centre's and |slope| r <= bound_tilt_gain (max - min of the ground samples).
    return std::max(ground + bound_slope * r, peaks) + bound_tilt_gain * 2.0 * bound_slope * r;
}

DetailedLunarTerrain::DiscSupport DetailedLunarTerrain::support_under_disc(const Vec3& centre_unit, const Vec3& up_fixed,
                                                                           double radius_m) const {
    if (!options_.detail) {
        const Vec3 fixed = centre_unit * sphere_radius_m_;
        return {lola_->height_at_fixed_m(fixed), lola_->normal_fixed(fixed)};
    }
    const double r = std::max(radius_m, 0.0);
    if (r == 0.0) return point_contact(centre_unit, up_fixed, true);
    const double two_r_sphere = 2.0 * sphere_radius_m_;
    const Vec3 up = math::normalized(up_fixed);
    const Tangent t = tangent_of(up);
    const auto at = [&](double x, double y) {
        return math::normalized(centre_unit + (t.e1 * x + t.e2 * y) / sphere_radius_m_);
    };

    // The pad's plane: least-squares through the ground at the sample pattern (symmetric: the slopes are independent
    // of each other and of the mean, which is the plane's height at the centre).
    constexpr int samples = 1 + disc_inner_ring + disc_rim_ring;
    std::array<double, samples> xs{}, ys{}, g{};
    std::array<Vec3, samples> units;
    {
        int i = 1;
        for (const auto& [ring, rho] : {std::pair{disc_inner_ring, 0.5 * r}, std::pair{disc_rim_ring, r}}) {
            for (int k = 0; k < ring; ++k, ++i) {
                const double phi = constants::two_pi * k / ring;
                xs[static_cast<size_t>(i)] = rho * std::cos(phi);
                ys[static_cast<size_t>(i)] = rho * std::sin(phi);
            }
        }
        for (int k = 0; k < samples; ++k) units[static_cast<size_t>(k)] = at(xs[static_cast<size_t>(k)], ys[static_cast<size_t>(k)]);
    }
    contact_ground_batch(units.data(), samples, g.data());
    double sum = 0.0, sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_yy = 0.0;
    for (size_t k = 0; k < samples; ++k) {
        g[k] -= (xs[k] * xs[k] + ys[k] * ys[k]) / two_r_sphere;
        sum += g[k], sum_x += g[k] * xs[k], sum_y += g[k] * ys[k], sum_xx += xs[k] * xs[k], sum_yy += ys[k] * ys[k];
    }
    const double mean = sum / samples, gx = sum_x / sum_xx, gy = sum_y / sum_yy;
    const auto plane = [&](double x, double y) { return mean + gx * x + gy * y; };
    // The highest protrusion above the plane, and the pad's normal.
    double best = -std::numeric_limits<double>::infinity();
    for (size_t k = 0; k < samples; ++k) best = std::max(best, g[k] - plane(xs[k], ys[k]));
    Vec3 normal = math::normalized(up - t.e1 * gx - t.e2 * gy);
    if (!rocks_on()) return {mean + best, normal};

    // Rocks, by an upper bound of their protrusion: top <= top_bound, the plane over the footprint (base +- a) >= its
    // height at the base - |slope| a. First every peak under the disc (cached, free), then the searches only for the
    // rocks whose bound still exceeds the best protrusion, highest bound first.
    //
    // Inside the disc the protrusion is approached by climbing over the rock's footprint column nodes from the peak (or
    // from the best rim point); on the rim (a rock the rim cuts) it is searched along the arc. The winning rock's point
    // is then refined to its support point along the pad normal (its exact top peaks at a vertex or an edge).
    // Bound of that choice (review m2): only the winner is refined, so a losing rock whose true peak lies between its
    // footprint nodes can out-protrude the winner by at most its node-grid sag (the polyhedron's top between nodes of
    // spacing 2 a / footprint_columns); measured against the brute-force maximum: <= 0.15 mm (test_detailed_lunar_terrain
    // "pivoting footpad" cases), below the contact tolerance of 1 mm.
    const double slope = std::hypot(gx, gy);
    RockQuery q;
    gather_rocks(centre_unit, r, q);
    struct Candidate {
        const ContactCache::RockEntry* e;
        double bx, by, px, py, bound;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(q.candidates.size());
    const ContactCache::RockEntry* winner = nullptr;  // the rock carrying the pad, and where
    double winner_x = 0.0, winner_y = 0.0;
    for (const auto* e : q.candidates) {
        const Vec3 b = (e->rock.base_unit - centre_unit) * sphere_radius_m_;
        const Vec3 d = (e->peak_unit - centre_unit) * sphere_radius_m_;
        const double bx = dot(b, t.e1), by = dot(b, t.e2), px = dot(d, t.e1), py = dot(d, t.e2);
        const double bound = e->top_bound_m - plane(bx, by) + slope * e->rock.axes_m.x;
        if (bound <= best) continue;
        candidates.push_back({e, bx, by, px, py, bound});
        if (px * px + py * py <= r * r) {  // the peak under the disc: its protrusion
            const double p = e->peak_height_m - (px * px + py * py) / two_r_sphere - plane(px, py);
            if (p > best) best = p, winner = e, winner_x = px, winner_y = py;
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.bound != b.bound ? a.bound > b.bound : a.e->rock.id < b.e->rock.id;
    });
    const RockTemplates& templates = RockTemplates::instance();
    constexpr int columns = RockTemplates::footprint_columns;
    constexpr double golden = 0.6180339887498949;  // (sqrt 5 - 1) / 2
    for (const Candidate& cand : candidates) {
        if (cand.bound <= best) break;
        const Rock& rock = cand.e->rock;
        const double a = rock.axes_m.x;
        double found = -std::numeric_limits<double>::infinity(), found_x = 0.0, found_y = 0.0;
        // Protrusion of the rock's contact top at disc point (x, y) above the plane (-inf where it misses).
        const auto protrusion = [&](double x, double y) {
            const auto top = contact_top(rock, at(x, y), sphere_radius_m_);
            if (!top) return -std::numeric_limits<double>::infinity();
            const double p = *top - (x * x + y * y) / two_r_sphere - plane(x, y);
            if (p > found) found = p, found_x = x, found_y = y;
            return p;
        };
        // The footprint column node (i, j): its disc point and protrusion (-inf outside the disc or where it misses).
        const Vec3 ea = math::column(rock.local_to_fixed, 0), eb = math::column(rock.local_to_fixed, 1);
        const double ea1 = dot(ea, t.e1), ea2 = dot(ea, t.e2), eb1 = dot(eb, t.e1), eb2 = dot(eb, t.e2);
        const auto node = [&](int i, int j) {
            if (i < 0 || j < 0 || i >= columns || j >= columns) return -std::numeric_limits<double>::infinity();
            const double z = templates.column_grid_node(rock.template_id, i, j);
            const double above_base = rock.axes_m.z * z - rock.burial_m;
            if (std::isnan(z) || above_base <= 0.0) return -std::numeric_limits<double>::infinity();
            const double xr = a * (-1.0 + (2.0 * i + 1.0) / columns), yr = rock.axes_m.y * (-1.0 + (2.0 * j + 1.0) / columns);
            const double x = cand.bx + ea1 * xr + eb1 * yr, y = cand.by + ea2 * xr + eb2 * yr;
            if (x * x + y * y > r * r) return -std::numeric_limits<double>::infinity();
            const double height = rock.base_height_m + above_base + (xr * xr + yr * yr) / two_r_sphere;
            const double p = height - (x * x + y * y) / two_r_sphere - plane(x, y);
            if (p > found) found = p, found_x = x, found_y = y;
            return p;
        };
        // The node nearest the disc point (x, y).
        const auto nearest_node = [&](double x, double y) {
            const double dx = x - cand.bx, dy = y - cand.by;
            const double ux = (ea1 * dx + ea2 * dy) / a, uy = (eb1 * dx + eb2 * dy) / rock.axes_m.y;
            return std::pair{static_cast<int>(std::lround((ux + 1.0) * 0.5 * columns - 0.5)),
                             static_cast<int>(std::lround((uy + 1.0) * 0.5 * columns - 0.5))};
        };
        // Climb over the nodes to a local maximum of the protrusion.
        const auto climb = [&](int i, int j) {
            double value = node(i, j);
            for (int step = 0; step < max_climb_steps; ++step) {
                int bi = i, bj = j;
                double bv = value;
                for (int di = -1; di <= 1; ++di) {
                    for (int dj = -1; dj <= 1; ++dj) {
                        if (di == 0 && dj == 0) continue;
                        const double v = node(i + di, j + dj);
                        if (v > bv) bv = v, bi = i + di, bj = j + dj;
                    }
                }
                if (bi == i && bj == j) return;
                i = bi, j = bj, value = bv;
            }
        };
        const bool peak_inside = cand.px * cand.px + cand.py * cand.py <= r * r;
        if (peak_inside) {
            const auto [i, j] = nearest_node(cand.px, cand.py);
            climb(i, j);
        }
        // Along the rim arc the rock covers (a rock the rim cuts), then inward from the best rim point.
        const double db = std::hypot(cand.bx, cand.by);
        if (db + a > r && cand.bound > std::max(best, found)) {
            double half_span = constants::pi;
            bool crosses = true;
            if (db > 0.0) {
                const double cos_span = (r * r + db * db - a * a) / (2.0 * r * db);
                if (cos_span >= 1.0) crosses = false;
                else if (cos_span > -1.0) half_span = std::acos(cos_span);
            }
            if (crosses) {
                const double theta_b = std::atan2(cand.by, cand.bx);
                const auto rim = [&](double theta) { return protrusion(r * std::cos(theta), r * std::sin(theta)); };
                const double step = 2.0 * half_span / arc_coarse_samples;
                double best_theta = theta_b, best_value = -std::numeric_limits<double>::infinity();
                for (int k = 0; k < arc_coarse_samples; ++k) {
                    const double theta = theta_b - half_span + (k + 0.5) * step;
                    const double v = rim(theta);
                    if (v > best_value) best_value = v, best_theta = theta;
                }
                if (best_value > -std::numeric_limits<double>::infinity()) {
                    double lo = best_theta - step, hi = best_theta + step;
                    double x1 = hi - golden * (hi - lo), x2 = lo + golden * (hi - lo);
                    double f1 = rim(x1), f2 = rim(x2);
                    for (int k = 0; k < arc_golden_steps; ++k) {
                        if (f1 >= f2) {
                            hi = x2, x2 = x1, f2 = f1;
                            x1 = hi - golden * (hi - lo);
                            f1 = rim(x1);
                        } else {
                            lo = x1, x1 = x2, f1 = f2;
                            x2 = lo + golden * (hi - lo);
                            f2 = rim(x2);
                        }
                    }
                    if (!peak_inside) {  // the protrusion may grow inward on a tilted plane
                        const double rim_x = found_x, rim_y = found_y;
                        const auto [i, j] = nearest_node(rim_x, rim_y);
                        climb(i, j);
                    }
                }
            }
        }
        if (found > best) best = found, winner = cand.e, winner_x = found_x, winner_y = found_y;
    }
    if (winner) {
        // The rock's contact top is exact (its LOD-0 polyhedron, Task 17: not convex): its protrusion above the plane
        // peaks at a vertex or on the rim, generally between the footprint nodes the climb visits. Inside the disc that peak is the
        // rock's support point along the pad's normal (RockTemplates::support_point; the sphere's curvature over the
        // pad, ~1e-7 m, aside); a rock the rim cuts keeps the rim search's point.
        const Rock& rock = winner->rock;
        const Vec3 u = math::transpose(rock.local_to_fixed) * normal;  // a = b = c: the template frame is the rock's
        const Vec3 p = RockTemplates::instance().support_point(rock.template_id, u);
        const Vec3 point = math::normalized(rock.base_unit + (math::column(rock.local_to_fixed, 0) * (rock.axes_m.x * p.x) +
                                                              math::column(rock.local_to_fixed, 1) * (rock.axes_m.y * p.y)) /
                                                                 sphere_radius_m_);
        const double px = dot(point - centre_unit, t.e1) * sphere_radius_m_;
        const double py = dot(point - centre_unit, t.e2) * sphere_radius_m_;
        if (px * px + py * py <= r * r) {
            if (const auto top = contact_top(rock, at(px, py), sphere_radius_m_)) {
                const double value = *top - (px * px + py * py) / two_r_sphere - plane(px, py);
                if (value > best) best = value, winner_x = px, winner_y = py;
            }
        }
        // The carrying rock's contact normal there for the pad (its own normal where the rock meets it with an edge or
        // a vertex).
        if (const auto top = rocks_->rock_top(rock, at(winner_x, winner_y), normal)) normal = top->second;
        else normal = winner->peak_normal;
    }
    return {mean + best, normal};
}

}  // namespace starsim::environment
