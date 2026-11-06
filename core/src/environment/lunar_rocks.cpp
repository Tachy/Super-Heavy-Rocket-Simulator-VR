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

// Normal-ish draws: sum of six uniforms (variance 1/2) scaled to unit variance, truncated at +-4.2 sigma [E].
constexpr int normal_draws = 6;

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

// Height of the ground in the template's unit coordinates for burial share f of the template's height.
double burial_cut(const RockTemplates& templates, int id, double f) {
    return templates.bottom_z(id) + f * (templates.top_z(id) - templates.bottom_z(id));
}

}  // namespace

LunarRocks::LunarRocks(std::shared_ptr<const LunarSurfaceDetail> detail,
                       std::shared_ptr<const RockAbundanceMap> abundance)
    : detail_(std::move(detail)), abundance_(std::move(abundance)) {
    if (!detail_) throw std::invalid_argument("LunarRocks: keine Oberflaechendetails uebergeben");
    const auto& rocks = detail_->config().rocks;
    const auto& sfd = rocks.sfd;
    if (sfd.form != "power" && sfd.form != "exponential") {
        throw std::invalid_argument("LunarRocks: unbekannte Groessenverteilung '" + sfd.form + "'");
    }
    if (rocks.abundance_source == "uniform") {
        abundance_ = nullptr;
    } else if (rocks.abundance_source == "diviner") {
        if (!abundance_) {
            throw std::invalid_argument(
                "LunarRocks: abundance_source = \"diviner\" braucht die Diviner-Felsenhäufigkeitskarte - bitte python "
                "tools/data_pipeline/fetch.py diviner_rock_abundance ausführen");
        }
    } else {
        throw std::invalid_argument("LunarRocks: unbekannte abundance_source '" + rocks.abundance_source + "'");
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
    const RockTemplates& templates = RockTemplates::instance();
    double least_area = constants::pi;
    for (int t = 0; t < RockTemplates::count; ++t) {
        least_area = std::min(least_area, templates.exposed_area(t, burial_cut(templates, t, max_burial_fraction)));
    }
    max_semi_axis_per_diameter_ = 0.5 * std::sqrt(constants::pi / least_area);
}

std::vector<float> LunarRocks::abundance_grid(int width, int height) const {
    std::vector<float> out;
    if (width <= 0 || height <= 0) return out;
    out.resize(static_cast<size_t>(width) * static_cast<size_t>(height));
    for (int j = 0; j < height; ++j) {
        const double lat = (90.0 - (j + 0.5) * 180.0 / height) * constants::deg_to_rad;
        for (int i = 0; i < width; ++i) {
            const double lon = (-180.0 + (i + 0.5) * 360.0 / width) * constants::deg_to_rad;
            const Vec3 unit{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
            out[static_cast<size_t>(j) * static_cast<size_t>(width) + static_cast<size_t>(i)] =
                static_cast<float>(abundance_factor(unit));
        }
    }
    return out;
}

double LunarRocks::abundance_factor(const Vec3& unit) const {
    const auto& rocks = detail_->config().rocks;
    if (!abundance_ || rocks.abundance_reference_area_fraction <= 0.0) return rocks.abundance_scale;
    const double lat = std::asin(std::clamp(unit.z, -1.0, 1.0));
    const double w = smooth01((abundance_->lat_limit_rad - std::abs(lat)) / abundance_blend_band_rad);
    if (w <= 0.0) return rocks.abundance_scale;
    const double value = abundance_->at(lat, std::atan2(unit.y, unit.x));
    if (std::isnan(value)) return rocks.abundance_scale;  // no data: the uniform law
    const double local = value / rocks.abundance_reference_area_fraction;
    return rocks.abundance_scale * ((1.0 - w) + w * local);
}

double LunarRocks::cell_factor(const FaceCell& cell) const { return abundance_factor(face_cell_point(cell, 0.5, 0.5)); }

double LunarRocks::burial_fraction(std::uint64_t id) const {
    const auto& cfg = detail_->config().rocks;
    return std::clamp(cfg.burial_fraction_mean + cfg.burial_fraction_sd * normal_draw(id, 5), 0.0,
                      max_burial_fraction);
}

void LunarRocks::collect(int octave, const FaceCell& cell, double min_diameter_m, const Filter* filter,
                         std::vector<Rock>& out) const {
    const Octave& o = octaves_[static_cast<size_t>(octave)];
    const auto& cfg = detail_->config().rocks;
    const RockTemplates& templates = RockTemplates::instance();
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
        // Shape: template (its fragment axis ratios and tilt) and burial share first; the size a then makes the
        // plan-view area above the ground pi D^2 / 4.
        const int template_id =
            std::min(static_cast<int>(unit_of(id, 6) * RockTemplates::count), RockTemplates::count - 1);
        const double cut = burial_cut(templates, template_id, burial_fraction(id));
        const double a = 0.5 * d * std::sqrt(constants::pi / templates.exposed_area(template_id, cut));
        if (filter) {
            const double limit = filter->radius_m + (filter->add_extent ? a : 0.0);
            if (radius * norm(base - filter->unit) > limit) continue;
        }
        Rock r;
        r.id = id;
        r.base_unit = base;
        r.diameter_m = d;
        r.axes_m = {a, a, a};  // isotropic: the template carries the fragment's axis ratios and tilt
        r.burial_m = cut * r.axes_m.z;
        r.template_id = template_id;
        r.albedo_factor = cfg.albedo_to_regolith;
        // Frame: yaw about the vertical (the template's own rotation tilts the fragment).
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

std::vector<Rock> LunarRocks::rocks_in_region(const FaceCell& region, double min_diameter_m) const {
    std::vector<Rock> out;
    for (size_t k = 0; k < octaves_.size(); ++k) {
        const Octave& o = octaves_[k];
        if (o.max_diameter_m <= min_diameter_m) continue;
        if (o.level >= region.level) {
            const int shift = o.level - region.level;
            if (shift > max_region_descent) {
                throw std::invalid_argument("LunarRocks::rocks_in_region: zu viele Zellen (Mindestdurchmesser zu klein)");
            }
            const std::int64_t n = std::int64_t{1} << shift;
            for (std::int64_t j = region.j * n; j < (region.j + 1) * n; ++j) {
                for (std::int64_t i = region.i * n; i < (region.i + 1) * n; ++i) {
                    collect(static_cast<int>(k), FaceCell{region.face, o.level, i, j}, min_diameter_m, nullptr, out);
                }
            }
        } else {
            const int shift = region.level - o.level;
            std::vector<Rock> coarse;
            collect(static_cast<int>(k), FaceCell{region.face, o.level, region.i >> shift, region.j >> shift},
                    min_diameter_m, nullptr, coarse);
            for (const Rock& r : coarse) {
                if (face_cell_of(r.base_unit, region.level) == region) out.push_back(r);
            }
        }
    }
    std::sort(out.begin(), out.end(), [](const Rock& a, const Rock& b) {
        return a.diameter_m != b.diameter_m ? a.diameter_m > b.diameter_m : a.id < b.id;
    });
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
        const double reach = radius_m + max_semi_axis_per_diameter_ * octaves_[k].max_diameter_m;
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
    const auto hit = templates.column_hit(rock.template_id, ux, uy);
    if (!hit) return std::nullopt;
    const double above_base = c * hit->first - rock.burial_m;
    if (above_base <= 0.0) return std::nullopt;

    const Vec3 n_unit = hit->second;
    const Vec3 n_local = math::normalized({n_unit.x / a, n_unit.y / b, n_unit.z / c});
    const double height = rock.base_height_m + above_base + (x * x + y * y) / (2.0 * sphere_radius);
    return std::pair{height, rock.local_to_fixed * n_local};
}

std::optional<std::pair<double, Vec3>> LunarRocks::rock_top(const Rock& rock, const Vec3& unit,
                                                           const Vec3& reference_fixed) const {
    auto top = rock_top(rock, unit);
    if (!top) return top;
    // a = b = c (isotropic rocks): the template frame maps to the fixed frame by local_to_fixed alone.
    const double sphere_radius = detail_->sphere_radius_m();
    const Vec3 offset = (unit - rock.base_unit) * sphere_radius;
    const double a = rock.axes_m.x;
    const double ux = dot(offset, math::column(rock.local_to_fixed, 0)) / a;
    const double uy = dot(offset, math::column(rock.local_to_fixed, 1)) / a;
    const double uz = (top->first - rock.base_height_m - (ux * ux + uy * uy) * a * a / (2.0 * sphere_radius) + rock.burial_m) / a;
    const Vec3 n_unit = RockTemplates::instance().normal_toward(
        rock.template_id, Vec3{ux, uy, uz}, math::transpose(rock.local_to_fixed) * reference_fixed,
        contact_edge_tolerance_m / a);
    top->second = rock.local_to_fixed * n_unit;
    return top;
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

LunarRocks::CoverCurve LunarRocks::cover_curve() const {
    const auto& sfd = detail_->config().rocks.sfd;
    const double d_min = sfd.min_diameter_m, d_max = sfd.max_diameter_m;
    CoverCurve curve;
    curve.diameters_m.resize(cover_curve_samples);
    curve.cover_above.resize(cover_curve_samples);
    for (int k = 0; k < cover_curve_samples; ++k) {
        const double d = d_min * std::pow(d_max / d_min, static_cast<double>(k) / (cover_curve_samples - 1));
        curve.diameters_m[static_cast<size_t>(k)] = d;
        curve.cover_above[static_cast<size_t>(k)] = cover_fraction(d, d_max);
    }
    return curve;
}

}  // namespace starsim::environment
