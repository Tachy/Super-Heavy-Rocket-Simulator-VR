#include "starsim/environment/detailed_lunar_terrain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "starsim/environment/cube_face_cell.hpp"
#include "starsim/environment/rock_templates.hpp"
#include "starsim/math/constants.hpp"

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
// steps around the best ([E]: brackets the arc to ~5 % of its span; the rock top is smooth there).
constexpr int arc_coarse_samples = 5;
constexpr int arc_golden_steps = 8;
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
    };
    struct RockPatch {
        std::vector<RockEntry> rocks;
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

double DetailedLunarTerrain::contact_ground_m(const Vec3& unit) const {
    const double lola = lola_->height_at_direction_m(unit);
    if (!options_.detail) return lola;
    const FaceCoord fc = face_coord_of(unit);
    const double u = (fc.s + 1.0) * 0.5 * lattice_nodes_per_edge, v = (fc.t + 1.0) * 0.5 * lattice_nodes_per_edge;
    const auto j0 = static_cast<std::int64_t>(std::floor(u)), k0 = static_cast<std::int64_t>(std::floor(v));
    const double fu = u - static_cast<double>(j0), fv = v - static_cast<double>(k0);
    std::array<double, 4> node{};  // (j0, k0), (j0 + 1, k0), (j0, k0 + 1), (j0 + 1, k0 + 1)
    const auto slot = [&](int n) { return std::pair{j0 + (n & 1), k0 + (n >> 1)}; };
    ContactCache& c = *cache_;
    bool missing = false;
    {
        std::lock_guard lock(c.mutex);
        for (int n = 0; n < 4; ++n) {
            const auto [j, k] = slot(n);
            auto& patch = c.ground[{fc.face, j >> patch_shift, k >> patch_shift}];
            patch.used = ++c.clock;
            node[static_cast<size_t>(n)] = patch.nodes[static_cast<size_t>((j & patch_mask) * patch_nodes + (k & patch_mask))];
            missing = missing || std::isnan(node[static_cast<size_t>(n)]);
        }
    }
    if (missing) {
        // Computed outside the lock (pure function of the node: whoever stores it stores the same value).
        for (int n = 0; n < 4; ++n) {
            if (!std::isnan(node[static_cast<size_t>(n)])) continue;
            const auto [j, k] = slot(n);
            const double s = -1.0 + 2.0 * static_cast<double>(j) / lattice_nodes_per_edge;
            const double t = -1.0 + 2.0 * static_cast<double>(k) / lattice_nodes_per_edge;
            node[static_cast<size_t>(n)] = detail_->relief_m(face_direction(fc.face, s, t), 0.0);
        }
        std::lock_guard lock(c.mutex);
        for (int n = 0; n < 4; ++n) {
            const auto [j, k] = slot(n);
            auto& patch = c.ground[{fc.face, j >> patch_shift, k >> patch_shift}];
            patch.used = ++c.clock;
            patch.nodes[static_cast<size_t>((j & patch_mask) * patch_nodes + (k & patch_mask))] = node[static_cast<size_t>(n)];
        }
        evict(c.ground, static_cast<size_t>(max_cached_patches));
    }
    const double relief = (1.0 - fv) * ((1.0 - fu) * node[0] + fu * node[1]) + fv * ((1.0 - fu) * node[2] + fu * node[3]);
    return lola + relief;
}

DetailedLunarTerrain::DiscSupport DetailedLunarTerrain::support_under_disc(const Vec3& centre_unit, const Vec3& up_fixed,
                                                                           double radius_m) const {
    if (!options_.detail) {
        const Vec3 fixed = centre_unit * sphere_radius_m_;
        return {lola_->height_at_fixed_m(fixed), lola_->normal_fixed(fixed)};
    }
    const double r = std::max(radius_m, 0.0);
    const double two_r_sphere = 2.0 * sphere_radius_m_;
    const Vec3 up = math::normalized(up_fixed);
    const Tangent t = tangent_of(up);
    const auto at = [&](double x, double y) {
        return math::normalized(centre_unit + (t.e1 * x + t.e2 * y) / sphere_radius_m_);
    };

    // Ground: the max over the sample pattern; the normal of the plane fitted through it.
    double best = -std::numeric_limits<double>::infinity();
    Vec3 normal;
    if (r > 0.0) {
        double sum = 0.0, sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_yy = 0.0;
        int count = 0;
        const auto sample = [&](double x, double y) {
            const double g = contact_ground_m(at(x, y)) - (x * x + y * y) / two_r_sphere;
            best = std::max(best, g);
            sum += g, sum_x += g * x, sum_y += g * y, sum_xx += x * x, sum_yy += y * y;
            ++count;
        };
        sample(0.0, 0.0);
        for (const auto& [ring, rho] : {std::pair{disc_inner_ring, 0.5 * r}, std::pair{disc_rim_ring, r}}) {
            for (int k = 0; k < ring; ++k) {
                const double phi = constants::two_pi * k / ring;
                sample(rho * std::cos(phi), rho * std::sin(phi));
            }
        }
        // The pattern is symmetric: the plane's slopes are independent of each other and of the mean.
        const double gx = sum_x / sum_xx, gy = sum_y / sum_yy;
        normal = math::normalized(up - t.e1 * gx - t.e2 * gy);
    } else {
        best = contact_ground_m(centre_unit);
        const double s = detail_normal_step_m;
        const double gx = (contact_ground_m(at(s, 0.0)) - contact_ground_m(at(-s, 0.0))) / (2.0 * s);
        const double gy = (contact_ground_m(at(0.0, s)) - contact_ground_m(at(0.0, -s))) / (2.0 * s);
        normal = math::normalized(up - t.e1 * gx - t.e2 * gy);
    }
    if (!rocks_on()) return {best, normal};

    // Rocks that may reach the disc, from the cached cells it touches (each cell holds every rock that may touch it).
    ContactCache& c = *cache_;
    std::vector<const ContactCache::RockEntry*> candidates;
    std::vector<std::shared_ptr<const ContactCache::RockPatch>> held;
    for (const FaceCell& cell : face_cells_within(centre_unit, contact_patch_level, r, sphere_radius_m_)) {
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
                if (const auto top = rocks_->rock_top(rock, e.peak_unit)) {
                    e.peak_valid = true;
                    e.peak_height_m = top->first;
                    e.peak_normal = top->second;
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
        for (const auto& e : patch->rocks) {
            if (e.peak_valid && sphere_radius_m_ * norm(e.rock.base_unit - centre_unit) <= r + e.rock.axes_m.x) {
                candidates.push_back(&e);
            }
        }
        held.push_back(std::move(patch));
    }
    // Highest peaks first (ties by id: a fixed order); a rock whose peak is not above the best so far cannot win.
    std::sort(candidates.begin(), candidates.end(), [](const auto* a, const auto* b) {
        return a->peak_height_m != b->peak_height_m ? a->peak_height_m > b->peak_height_m : a->rock.id < b->rock.id;
    });
    candidates.erase(std::unique(candidates.begin(), candidates.end(),
                                 [](const auto* a, const auto* b) { return a->rock.id == b->rock.id; }),
                     candidates.end());
    for (const auto* e : candidates) {
        if (e->peak_height_m <= best) break;
        const Vec3 d = (e->peak_unit - centre_unit) * sphere_radius_m_;
        const double px = dot(d, t.e1), py = dot(d, t.e2);
        if (px * px + py * py <= r * r) {  // the peak lies under the disc: the rock's highest point there
            const double h = e->peak_height_m - (px * px + py * py) / two_r_sphere;
            if (h > best) best = h, normal = e->peak_normal;
            continue;
        }
        if (r <= 0.0) continue;
        // The rock is cut by the rim: the highest point of the rim arc within the rock's reach.
        const Vec3 b = (e->rock.base_unit - centre_unit) * sphere_radius_m_;
        const double bx = dot(b, t.e1), by = dot(b, t.e2), db = std::hypot(bx, by), a = e->rock.axes_m.x;
        double half_span = constants::pi;
        if (db > 0.0) {
            const double cos_span = (r * r + db * db - a * a) / (2.0 * r * db);
            if (cos_span >= 1.0) continue;
            if (cos_span > -1.0) half_span = std::acos(cos_span);
        }
        const double theta_b = std::atan2(by, bx);
        const double rim_drop = r * r / two_r_sphere;
        std::pair<double, Vec3> found{-std::numeric_limits<double>::infinity(), Vec3{}};
        const auto value = [&](double theta) {
            const auto top = rocks_->rock_top(e->rock, at(r * std::cos(theta), r * std::sin(theta)));
            if (!top) return -std::numeric_limits<double>::infinity();
            const double h = top->first - rim_drop;
            if (h > found.first) found = {h, top->second};
            return h;
        };
        const double step = 2.0 * half_span / arc_coarse_samples;
        double best_theta = theta_b, best_value = -std::numeric_limits<double>::infinity();
        for (int k = 0; k < arc_coarse_samples; ++k) {
            const double theta = theta_b - half_span + (k + 0.5) * step;
            const double v = value(theta);
            if (v > best_value) best_value = v, best_theta = theta;
        }
        if (best_value == -std::numeric_limits<double>::infinity()) continue;
        constexpr double golden = 0.6180339887498949;  // (sqrt 5 - 1) / 2
        double lo = best_theta - step, hi = best_theta + step;
        double x1 = hi - golden * (hi - lo), x2 = lo + golden * (hi - lo);
        double f1 = value(x1), f2 = value(x2);
        for (int k = 0; k < arc_golden_steps; ++k) {
            if (f1 >= f2) {
                hi = x2, x2 = x1, f2 = f1;
                x1 = hi - golden * (hi - lo);
                f1 = value(x1);
            } else {
                lo = x1, x1 = x2, f1 = f2;
                x2 = lo + golden * (hi - lo);
                f2 = value(x2);
            }
        }
        if (found.first > best) best = found.first, normal = found.second;
    }
    return {best, normal};
}

}  // namespace starsim::environment
