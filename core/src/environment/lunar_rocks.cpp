#include "starsim/environment/lunar_rocks.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "starsim/environment/surface_hash.hpp"
#include "starsim/math/constants.hpp"
#include "surface_sampling.hpp"

namespace starsim::environment {

using math::Vec3;

namespace {

// Share of the rock height below the ground is limited to [0, 0.6] and the axis ratios to the NAC range ([E]): keeps
// every rock a rock (visible above the ground, c <= b <= a, b/a >= 0.3 as measured on NAC boulders).
constexpr double max_burial_fraction = 0.6;
constexpr double min_axis_ratio = 0.3, max_axis_ratio = 1.0, min_c_to_a = 0.2;
// Normal-ish draws: sum of six uniforms (variance 1/2) scaled to unit variance, truncated at +-4.2 sigma [E].
constexpr int normal_draws = 6;
// Bisection refinement of the rock top (bracket c/24 wide): 40 halvings are far below a micrometre.
constexpr int top_bisections = 40;
constexpr double min_inside_norm = 1e-12;

double unit_of(std::uint64_t id, std::uint64_t field) { return hash_unit(surface_hash(id, field, 0x5EED, 0)); }

double normal_draw(std::uint64_t id, std::uint64_t field) {
    double sum = 0.0;
    for (int k = 0; k < normal_draws; ++k) sum += unit_of(id, field * 16 + static_cast<std::uint64_t>(k));
    return (sum - 0.5 * normal_draws) * std::sqrt(2.0);
}

std::uint64_t rock_cell_hash(const FaceCell& cell) {
    return surface_hash((surface_stream::rocks << 8) | static_cast<std::uint64_t>(cell.face),
                        static_cast<std::uint64_t>(cell.level), static_cast<std::uint64_t>(cell.i),
                        static_cast<std::uint64_t>(cell.j));
}

double smooth01(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

}  // namespace

double RockAbundanceMap::at(double latitude_rad, double longitude_rad) const {
    const double line = std::clamp((lat_limit_rad - latitude_rad) / (2.0 * lat_limit_rad) * lines - 0.5, 0.0,
                                   static_cast<double>(lines - 1));
    double lon = std::fmod(longitude_rad, constants::two_pi);
    if (lon < 0.0) lon += constants::two_pi;
    const double sample = lon / constants::two_pi * samples - 0.5;
    const int l0 = std::min(static_cast<int>(std::floor(line)), lines - 1);
    const int l1 = std::min(l0 + 1, lines - 1);
    const double fl = line - l0;
    const int s0 = static_cast<int>(std::floor(sample));
    const double fs = sample - s0;
    const auto px = [&](int l, int s) {
        const int w = ((s % samples) + samples) % samples;
        return static_cast<double>(area_fraction[static_cast<size_t>(l) * static_cast<size_t>(samples) +
                                                 static_cast<size_t>(w)]);
    };
    return (1.0 - fl) * ((1.0 - fs) * px(l0, s0) + fs * px(l0, s0 + 1)) +
           fl * ((1.0 - fs) * px(l1, s0) + fs * px(l1, s0 + 1));
}

LunarRocks::LunarRocks(std::shared_ptr<const LunarSurfaceDetail> detail,
                       std::shared_ptr<const RockAbundanceMap> abundance)
    : detail_(std::move(detail)), abundance_(std::move(abundance)) {
    if (!detail_) throw std::invalid_argument("LunarRocks: keine Oberflaechendetails uebergeben");
    const auto& sfd = detail_->config().rocks.sfd;
    if (sfd.form != "power" && sfd.form != "exponential") {
        throw std::invalid_argument("LunarRocks: unbekannte Groessenverteilung '" + sfd.form + "'");
    }
    if (abundance_ && (abundance_->lines < 2 || abundance_->samples < 2 || abundance_->lat_limit_rad <= 0.0 ||
                       abundance_->area_fraction.size() !=
                           static_cast<size_t>(abundance_->lines) * static_cast<size_t>(abundance_->samples))) {
        throw std::invalid_argument("LunarRocks: ungueltige Felsenhaeufigkeitskarte");
    }
    for (double d_lo = sfd.min_diameter_m; d_lo < sfd.max_diameter_m;) {
        const double d_hi = std::min(2.0 * d_lo, sfd.max_diameter_m);
        Octave o;
        o.min_diameter_m = d_lo;
        o.max_diameter_m = d_hi;
        o.level = face_cell_level_for_size(cell_per_diameter * d_hi, detail_->sphere_radius_m());
        o.density_per_m2 = sfd.cumulative_density_per_m2(d_lo) - sfd.cumulative_density_per_m2(d_hi);
        octaves_.push_back(o);
        d_lo = d_hi;
    }
}

double LunarRocks::abundance_factor(const Vec3& unit) const {
    const auto& rocks = detail_->config().rocks;
    if (!abundance_ || rocks.abundance_reference_area_fraction <= 0.0) return rocks.abundance_scale;
    const double lat = std::asin(std::clamp(unit.z, -1.0, 1.0));
    const double w = smooth01((abundance_->lat_limit_rad - std::abs(lat)) / abundance_blend_band_rad);
    if (w <= 0.0) return rocks.abundance_scale;
    const double local = abundance_->at(lat, std::atan2(unit.y, unit.x)) / rocks.abundance_reference_area_fraction;
    return rocks.abundance_scale * ((1.0 - w) + w * local);
}

double LunarRocks::cell_factor(const FaceCell& cell) const { return abundance_factor(face_cell_point(cell, 0.5, 0.5)); }

void LunarRocks::collect(int octave, const FaceCell& cell, double min_diameter_m, const Filter* filter,
                         std::vector<Rock>& out) const {
    const Octave& o = octaves_[static_cast<size_t>(octave)];
    const auto& cfg = detail_->config().rocks;
    const double radius = detail_->sphere_radius_m();
    const std::uint64_t h = rock_cell_hash(cell);
    const auto ou = static_cast<std::uint64_t>(octave);
    const int count = sampling::poisson(hash_unit(surface_hash(h, ou, 0xC0, 0)),
                                        cell_factor(cell) * o.density_per_m2 * face_cell_area_m2(cell, radius));
    const double n_lo = cfg.sfd.cumulative_density_per_m2(o.min_diameter_m);
    const double n_hi = cfg.sfd.cumulative_density_per_m2(o.max_diameter_m);
    for (int m = 0; m < count; ++m) {
        const std::uint64_t id = surface_hash(h, (ou << 32) | static_cast<std::uint64_t>(m), 0x1D, 0);
        const double d = sampling::inverse_cumulative(cfg.sfd, n_lo - unit_of(id, 2) * (n_lo - n_hi));
        if (d < min_diameter_m) continue;
        const Vec3 base = face_cell_point(cell, unit_of(id, 0), unit_of(id, 1));
        if (filter) {
            const double limit = filter->radius_m + (filter->add_extent ? 0.5 * d : 0.0);
            if (radius * norm(base - filter->unit) > limit) continue;
        }
        Rock r;
        r.id = id;
        r.base_unit = base;
        r.diameter_m = d;
        const double a = 0.5 * d;
        const double b_to_a =
            std::clamp(cfg.axis_b_to_a_mean + cfg.axis_ratio_sd * normal_draw(id, 3), min_axis_ratio, max_axis_ratio);
        const double c_to_a =
            std::clamp(cfg.axis_c_to_a_mean + cfg.axis_ratio_sd * normal_draw(id, 4), min_c_to_a, b_to_a);
        r.axes_m = {a, a * b_to_a, a * c_to_a};
        const double burial_fraction = std::clamp(
            cfg.burial_fraction_mean + cfg.burial_fraction_sd * normal_draw(id, 5), 0.0, max_burial_fraction);
        r.burial_m = (2.0 * burial_fraction - 1.0) * r.axes_m.z;
        r.template_id = std::min(static_cast<int>(unit_of(id, 6) * RockTemplates::count), RockTemplates::count - 1);
        r.albedo_factor = cfg.albedo_to_regolith;
        // Frame: yaw about the vertical (the rock rests on its flattest side).
        const Vec3 t1 = math::normalized(cross(std::abs(base.z) < 0.9 ? Vec3{0, 0, 1} : Vec3{1, 0, 0}, base));
        const Vec3 t2 = cross(base, t1);
        const double yaw = constants::two_pi * unit_of(id, 7);
        const Vec3 e_a = t1 * std::cos(yaw) + t2 * std::sin(yaw);
        r.local_to_fixed = math::Mat3::from_columns(e_a, cross(base, e_a), base);
        r.base_height_m = detail_->lola().height_at_direction_m(base) + detail_->relief_m(base, 0.0);
        out.push_back(r);
    }
}

std::vector<Rock> LunarRocks::rocks_in_cell(const FaceCell& cell, double min_diameter_m) const {
    std::vector<Rock> out;
    for (size_t k = 0; k < octaves_.size(); ++k) {
        if (octaves_[k].level == cell.level && octaves_[k].max_diameter_m > min_diameter_m) {
            collect(static_cast<int>(k), cell, min_diameter_m, nullptr, out);
        }
    }
    return out;
}

std::vector<Rock> LunarRocks::rocks_near(const Vec3& unit, double radius_m, double min_diameter_m) const {
    std::vector<Rock> out;
    const Filter filter{unit, radius_m, false};
    for (size_t k = 0; k < octaves_.size(); ++k) {
        if (octaves_[k].max_diameter_m <= min_diameter_m) continue;
        for (const FaceCell& cell : face_cells_within(unit, octaves_[k].level, radius_m, detail_->sphere_radius_m())) {
            collect(static_cast<int>(k), cell, min_diameter_m, &filter, out);
        }
    }
    return out;
}

std::vector<Rock> LunarRocks::rocks_touching(const Vec3& unit, double radius_m, double min_diameter_m) const {
    std::vector<Rock> out;
    const Filter filter{unit, radius_m, true};
    for (size_t k = 0; k < octaves_.size(); ++k) {
        if (octaves_[k].max_diameter_m <= min_diameter_m) continue;
        const double reach = radius_m + 0.5 * octaves_[k].max_diameter_m;
        for (const FaceCell& cell : face_cells_within(unit, octaves_[k].level, reach, detail_->sphere_radius_m())) {
            collect(static_cast<int>(k), cell, min_diameter_m, &filter, out);
        }
    }
    return out;
}

std::optional<std::pair<double, Vec3>> LunarRocks::rock_top(const Rock& rock, const Vec3& unit) const {
    const double sphere_radius = detail_->sphere_radius_m();
    const Vec3 e_a = math::column(rock.local_to_fixed, 0), e_b = math::column(rock.local_to_fixed, 1);
    const Vec3 offset = (unit - rock.base_unit) * sphere_radius;
    const double x = dot(offset, e_a), y = dot(offset, e_b);
    const double a = rock.axes_m.x, b = rock.axes_m.y, c = rock.axes_m.z;
    const double ux = x / a, uy = y / b;
    if (ux * ux + uy * uy >= 1.0) return std::nullopt;

    const RockTemplates& templates = RockTemplates::instance();
    // f < 0 inside the star-shaped rock: |u| - r(u / |u|) in the unit-size coordinates u = (x / a, y / b, z / c).
    const auto f = [&](double z) {
        const Vec3 u{ux, uy, z / c};
        const double n = norm(u);
        return n < min_inside_norm ? -1.0 : n - templates.radius(rock.template_id, u / n);
    };
    double z_out = 0.0, z_in = 0.0;
    bool found = false;
    double previous = c;
    for (int i = 0; i <= top_scan_steps && !found; ++i) {
        const double z = c * (1.0 - 2.0 * i / top_scan_steps);
        if (f(z) <= 0.0) {
            z_in = z;
            z_out = i == 0 ? z : previous;
            found = true;
        }
        previous = z;
    }
    if (!found) return std::nullopt;
    for (int k = 0; k < top_bisections && z_out != z_in; ++k) {
        const double mid = 0.5 * (z_out + z_in);
        (f(mid) <= 0.0 ? z_in : z_out) = mid;
    }
    const double above_base = z_in - rock.burial_m;
    if (above_base <= 0.0) return std::nullopt;

    const Vec3 u{ux, uy, z_in / c};
    const Vec3 n_unit = templates.normal(rock.template_id, u);
    const Vec3 n_local = math::normalized({n_unit.x / a, n_unit.y / b, n_unit.z / c});
    const double height = rock.base_height_m + above_base + (x * x + y * y) / (2.0 * sphere_radius);
    return std::pair{height, rock.local_to_fixed * n_local};
}

double LunarRocks::cover_fraction(double min_diameter_m, double max_diameter_m) const {
    const auto& cfg = detail_->config().rocks;
    const double lo = std::max(min_diameter_m, cfg.sfd.min_diameter_m);
    const double hi = std::min(max_diameter_m, cfg.sfd.max_diameter_m);
    if (hi <= lo) return 0.0;
    // Area = integral of (pi / 4) D^2 (-dN/dD) dD.
    double area = 0.0;
    if (cfg.sfd.form == "power") {
        const double b = cfg.sfd.exponent;
        const double ref = cfg.sfd.reference_diameter_m;
        area = 0.25 * constants::pi * cfg.sfd.density_per_m2 * b * std::pow(ref, b) *
               (std::pow(hi, 2.0 - b) - std::pow(lo, 2.0 - b)) / (2.0 - b);
    } else {
        constexpr int steps = 4000;  // log-spaced midpoint rule for the exponential law
        const double ratio = std::pow(hi / lo, 1.0 / steps);
        double d0 = lo;
        for (int k = 0; k < steps; ++k) {
            const double d1 = d0 * ratio, d = std::sqrt(d0 * d1);
            area += 0.25 * constants::pi * d * d *
                    (cfg.sfd.cumulative_density_per_m2(d0) - cfg.sfd.cumulative_density_per_m2(d1));
            d0 = d1;
        }
    }
    return cfg.abundance_scale * area;
}

}  // namespace starsim::environment
