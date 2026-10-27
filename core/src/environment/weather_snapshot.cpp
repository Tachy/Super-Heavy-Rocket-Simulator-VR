// Weather snapshots on ellipsoidal height layers (M8 weather and clouds); physics and encoding in weather_snapshot.hpp.

#include "starsim/environment/weather_snapshot.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

#include "starsim/environment/cloud_noise.hpp"
#include "starsim/environment/cloud_optics.hpp"
#include "starsim/math/constants.hpp"
#include "parallel_rows.hpp"

namespace starsim::environment {

namespace {

using weather_decode::FloatGrid;
namespace c = constants;

constexpr int channels = 4;  // RGBA8
constexpr double nan = std::numeric_limits<double>::quiet_NaN();
constexpr double water_code_ratio = weather_water_code_max_kg_m3 / weather_water_code_min_kg_m3;

const std::array<double, 256>& water_decode_table() {
    static const std::array<double, 256> table = [] {
        std::array<double, 256> t{};
        for (int code = 1; code < 256; ++code) {
            t[code] = weather_water_code_min_kg_m3 * std::pow(water_code_ratio, (code - 1) / 254.0);
        }
        return t;
    }();
    return table;
}

// Lambert W, branch -1, for x in [-1/e, 0): the solution w <= -1 of w e^w = x. Start from the branch-point series near
// -1/e (Corless et al. 1996, eq. 4.22) and the asymptotic form near 0, then Halley's iteration.
double lambert_w_minus1(double x) {
    constexpr double inv_e = 0.36787944117144233;
    if (x <= -inv_e) return -1.0;
    double w;
    if (x < -0.25) {
        const double p = -std::sqrt(2.0 * (std::exp(1.0) * x + 1.0));
        w = -1.0 + p - p * p / 3.0 + 11.0 / 72.0 * p * p * p;
    } else {
        const double l1 = std::log(-x), l2 = std::log(-l1);
        w = l1 - l2 + l2 / l1;
    }
    for (int i = 0; i < 40; ++i) {
        const double ew = std::exp(w);
        const double f = w * ew - x;
        const double step = f / (ew * (w + 1.0) - (w + 2.0) * f / (2.0 * w + 2.0));
        w -= step;
        if (std::abs(step) <= 1e-14 * std::abs(w)) break;
    }
    return w;
}

// Romps' saturation vapour pressure over liquid water (Pa), his eq. for constant heat capacities.
double romps_saturation_pressure_pa(double temperature_k) {
    const double rv = c::romps_vapour_gas_constant_j_kgk, cvl = c::romps_liquid_c_j_kgk, cvv = c::romps_vapour_cv_j_kgk;
    const double t0 = c::romps_triple_point_temperature_k;
    return c::romps_triple_point_pressure_pa * std::pow(temperature_k / t0, (cvv + rv - cvl) / rv) *
           std::exp((c::romps_vapour_internal_energy_at_triple_j_kg - (cvv - cvl) * t0) / rv *
                    (1.0 / t0 - 1.0 / temperature_k));
}

double finite_or_zero(double v) { return std::isfinite(v) ? v : 0.0; }

// Value of a grid cell without the bounds check of FloatGrid::at (shapes are validated once).
double cell(const FloatGrid& g, int level, int row, int column) {
    return g.values[(static_cast<std::size_t>(level) * g.rows + row) * g.columns + column];
}

void require_shape(const FloatGrid& g, int levels, int rows, int columns, const char* name) {
    if (g.levels != levels || g.rows != rows || g.columns != columns ||
        g.values.size() != static_cast<std::size_t>(levels) * rows * columns) {
        throw std::invalid_argument(std::string("weather snapshot: grid ") + name + " has the wrong shape");
    }
}

void require_grid(int rows, int columns) {
    if (rows < 2 || columns < 2) throw std::invalid_argument("weather snapshot: grid needs >= 2 rows and columns");
}

// The 2 m temperature and dew point are optional but come as a pair; true when present (shape-checked).
bool require_optional_2m(const FloatGrid& t2m, const FloatGrid& td2m, int rows, int columns) {
    if (t2m.values.empty() && td2m.values.empty()) return false;
    require_shape(t2m, 1, rows, columns, "temperature_2m_k");
    require_shape(td2m, 1, rows, columns, "dewpoint_2m_k");
    return true;
}

// Lifting condensation level of one column from the 2 m fields: weather_lcl_unknown_m where the data is missing, 0 where
// the surface air is saturated within the fog spread (fog / low stratus may reach the ground).
float column_lcl_m(const FloatGrid& t2m, const FloatGrid& td2m, int row, int column, double pressure_pa) {
    const double t = cell(t2m, 0, row, column), td = cell(td2m, 0, row, column);
    const double lcl = lifting_condensation_level_m(t, td, pressure_pa);
    if (!std::isfinite(lcl)) return weather_lcl_unknown_m;
    if (t - td < c::cloud_fog_spread_k) return 0.0f;
    return static_cast<float>(std::min(lcl, weather_top_m));
}

// The lifting condensation level of a box of columns (a downsampled cell, a pole row): the LOWEST known base, so the
// flat base drawn there never cuts the cloud of the box's lowest-based column (a mean would). Unknown and invalid
// (NaN) levels do not count; a saturated column (0: fog) makes the box saturated (no cut). value(): the lowest
// ground + level less the box's ground height, at least 0.
struct LclMin {
    double base_m = std::numeric_limits<double>::infinity();
    bool saturated = false;
    void add(double lcl_m, double ground_m) {
        if (!(lcl_m >= 0.0)) return;
        saturated = saturated || lcl_m == 0.0;
        base_m = std::min(base_m, ground_m + lcl_m);
    }
    float value(double box_ground_m) const {
        if (saturated) return 0.0f;
        return std::isfinite(base_m) ? static_cast<float>(std::max(0.0, base_m - box_ground_m)) : weather_lcl_unknown_m;
    }
};

// A layer that can hold the base of a cloud for the LCL rule: liquid of at least the smallest non-zero water code and
// a cloud fraction of at least cloud_min_cover (a trace below either is no cloud base; it is still moved up).
bool significant_liquid(double fraction, double liquid_kg_m3) {
    return liquid_kg_m3 >= weather_water_code_min_kg_m3 && fraction >= c::cloud_min_cover;
}

// Condensate the snapshot stores (a non-zero water code of either phase): a layer that shows.
bool stores_condensate(double liquid_kg_m3, double ice_kg_m3) {
    return encode_weather_water(liquid_kg_m3) > 0 || encode_weather_water(ice_kg_m3) > 0;
}

double row_latitude_rad(int row, int rows) { return c::pi / 2.0 - row * c::pi / (rows - 1); }
double column_longitude_rad(int column, int columns) { return column * c::two_pi / columns; }

using detail::for_each_row;

// Heights (gpm) and virtual temperatures of the levels visited in `order` (descending pressure), NaN at or below the
// surface. The humidity of the virtual temperature is the column water vapour spread with the exponential profile
// (weather_snapshot.hpp).
template <class Temperature>
void integrate_heights(const std::vector<int>& order, std::span<const double> levels_pa, Temperature temperature_k,
                       double total_column_water_vapour_kg_m2, double surface_pressure_pa,
                       double surface_geopotential_m2_s2, std::span<double> out_gpm, std::span<double> out_tv_k) {
    std::fill(out_gpm.begin(), out_gpm.end(), nan);
    std::fill(out_tv_k.begin(), out_tv_k.end(), nan);
    if (!std::isfinite(surface_pressure_pa) || !std::isfinite(surface_geopotential_m2_s2)) return;
    const double rd_g = c::dry_air_gas_constant_j_kgk / c::standard_gravity_mps2;
    const double vapour_factor = c::water_vapour_gas_constant_j_kgk / c::dry_air_gas_constant_j_kgk - 1.0;
    const double column_vapour = std::max(0.0, finite_or_zero(total_column_water_vapour_kg_m2));
    double previous_p = surface_pressure_pa, previous_tv = 0.0, previous_z = 0.0;
    double surface_q = 0.0, profile_exponent = 0.0;  // q = surface_q (p / p_s)^profile_exponent
    bool started = false;
    for (int k : order) {
        const double p = levels_pa[k];
        if (!(p < surface_pressure_pa) || !(p > 0.0)) continue;
        const double t = temperature_k(k);
        if (!std::isfinite(t) || t <= 0.0) continue;
        if (!started) {
            // a = H_p / H_v from the first level's temperature; q_s = W g0 a / p_s.
            const double a = std::max(1.0, rd_g * t / c::water_vapour_scale_height_m);
            surface_q = column_vapour * c::standard_gravity_mps2 * a / surface_pressure_pa;
            profile_exponent = a - 1.0;
        }
        const double q = surface_q * std::pow(p / surface_pressure_pa, profile_exponent);
        const double tv = t * (1.0 + vapour_factor * q);
        double z;
        if (!started) {
            // From the surface to the first level above it: the standard lapse rate below that level's temperature.
            const double exponent = c::dry_air_gas_constant_j_kgk * c::standard_lapse_rate_k_per_m / c::standard_gravity_mps2;
            z = surface_geopotential_m2_s2 / c::standard_gravity_mps2 +
                tv / c::standard_lapse_rate_k_per_m * (std::pow(surface_pressure_pa / p, exponent) - 1.0);
            started = true;
        } else {
            z = previous_z + rd_g * 0.5 * (tv + previous_tv) * std::log(previous_p / p);  // T linear in ln p
        }
        out_gpm[static_cast<std::size_t>(k)] = z;
        out_tv_k[static_cast<std::size_t>(k)] = tv;
        previous_p = p;
        previous_tv = tv;
        previous_z = z;
    }
}

std::vector<int> descending_pressure_order(std::span<const double> levels_pa) {
    std::vector<int> order(levels_pa.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return levels_pa[a] > levels_pa[b]; });
    return order;
}

// One level of a column above the ground: ellipsoidal height and the fields (cloud fraction, liquid, ice water content,
// temperature, pressure).
constexpr int sample_fields = 5;
struct LevelSample {
    double h, values[sample_fields];
};

// The total cloud cover of a column from the data (0..1; unknown where missing or not finite).
float column_total_cloud_cover(const FloatGrid& g, int row, int column, double scale) {
    if (g.values.empty()) return weather_total_cloud_cover_unknown;
    const double v = cell(g, 0, row, column) * scale;
    return std::isfinite(v) ? static_cast<float>(std::clamp(v, 0.0, 1.0)) : weather_total_cloud_cover_unknown;
}

// Box mean of the known total cloud covers (unknown when none is known).
struct CoverMean {
    double sum = 0.0, weight = 0.0;
    void add(float cover, double w) {
        if (!(cover >= 0.0f)) return;
        sum += w * cover;
        weight += w;
    }
    float value() const { return weight > 0.0 ? static_cast<float>(sum / weight) : weather_total_cloud_cover_unknown; }
};

// The layer means of one column into the texels. The profile is piecewise linear in height between the samples, held
// below the lowest and above the highest; layer parts below the ground do not count.
class ColumnLayers {
public:
    // samples sorted by ascending height; empty = a clear column.
    // lcl_m: the column's lifting condensation level above the ground (raise_liquid_cloud_to_lcl; <= 0 = none);
    // total_cloud_cover: the data's (scale_cloud_cover_to_total; negative = unknown). Returns the liquid water path the
    // cumulus got from the adiabatic parcel (add_cumulus_adiabatic_water) and the condensate path dropped above the
    // in-cloud bound (bound_incloud_condensate).
    struct Result {
        double added_liquid_path_kg_m2 = 0.0, dropped_condensate_path_kg_m2 = 0.0;
    };
    Result build(std::vector<LevelSample>& samples, double ground_m, double lcl_m, double total_cloud_cover,
                 std::uint8_t* texels, std::size_t layer_stride) {
        std::sort(samples.begin(), samples.end(), [](const LevelSample& a, const LevelSample& b) { return a.h < b.h; });
        samples_ = &samples;
        cumulative_.assign(samples.size(), {});
        for (std::size_t i = 1; i < samples.size(); ++i) {
            const double dh = samples[i].h - samples[i - 1].h;
            for (int f = 0; f < sample_fields; ++f) {
                cumulative_[i][f] = cumulative_[i - 1][f] + 0.5 * dh * (samples[i].values[f] + samples[i - 1].values[f]);
            }
        }
        cursor_ = 0;
        std::array<double, weather_layer_count> fraction{}, liquid{}, ice{}, temperature{}, pressure{};
        for (int layer = 0; layer < weather_layer_count; ++layer) {
            const double top = weather_layer_bottom_m(layer + 1);
            const double bottom = std::max(weather_layer_bottom_m(layer), ground_m);
            if (samples.empty() || !(top > bottom)) continue;
            const auto lo = integral_to(bottom), hi = integral_to(top);
            const double thickness = top - bottom;
            const auto i = static_cast<std::size_t>(layer);
            fraction[i] = (hi[0] - lo[0]) / thickness;
            liquid[i] = (hi[1] - lo[1]) / thickness;
            ice[i] = (hi[2] - lo[2]) / thickness;
            temperature[i] = (hi[3] - lo[3]) / thickness;
            pressure[i] = (hi[4] - lo[4]) / thickness;
        }
        // Bases at the LCL, then the cover from the data's total (the cloud types follow the final fractions), the water
        // that the scaled fractions concentrate beyond the bound dropped, then the cumulus water on those fractions.
        raise_liquid_cloud_to_lcl(fraction, liquid, ice, ground_m, lcl_m);
        scale_cloud_cover_to_total(fraction, liquid, ice, ground_m, total_cloud_cover);
        const double dropped = bound_incloud_condensate(fraction, liquid, ice, ground_m);
        const double added = add_cumulus_adiabatic_water(fraction, liquid, ice, temperature, pressure, ground_m, lcl_m);
        for (int layer = 0; layer < weather_layer_count; ++layer) {
            std::uint8_t* t = texels + static_cast<std::size_t>(layer) * layer_stride;
            const auto i = static_cast<std::size_t>(layer);
            t[0] = encode_weather_cloud_fraction(fraction[i]);
            t[1] = encode_weather_water(liquid[i]);
            t[2] = encode_weather_water(ice[i]);
            t[3] = 0;
        }
        return {added, dropped};
    }

private:
    // Integral of the profile from the lowest sample to x (negative below it); x must not decrease between calls.
    std::array<double, sample_fields> integral_to(double x) {
        const auto& s = *samples_;
        std::array<double, sample_fields> out{};
        if (x <= s.front().h) {
            for (int f = 0; f < sample_fields; ++f) out[f] = (x - s.front().h) * s.front().values[f];
            return out;
        }
        while (cursor_ + 1 < s.size() && s[cursor_ + 1].h <= x) ++cursor_;
        const LevelSample& a = s[cursor_];
        const double dx = x - a.h;
        if (cursor_ + 1 == s.size()) {
            for (int f = 0; f < sample_fields; ++f) out[f] = cumulative_[cursor_][f] + dx * a.values[f];
            return out;
        }
        const LevelSample& b = s[cursor_ + 1];
        const double w = dx / (b.h - a.h);  // b.h > x >= a.h
        for (int f = 0; f < sample_fields; ++f) {
            const double vx = a.values[f] + w * (b.values[f] - a.values[f]);
            out[f] = cumulative_[cursor_][f] + 0.5 * dx * (a.values[f] + vx);
        }
        return out;
    }

    const std::vector<LevelSample>* samples_ = nullptr;
    std::vector<std::array<double, sample_fields>> cumulative_;
    std::size_t cursor_ = 0;
};

}  // namespace

double weather_layer_bottom_m(int layer) {
    const int l = std::clamp(layer, 0, weather_layer_count);
    if (l <= weather_fine_layer_count) return l * weather_fine_layer_thickness_m;
    return weather_fine_layer_count * weather_fine_layer_thickness_m +
           (l - weather_fine_layer_count) * weather_coarse_layer_thickness_m;
}

double weather_layer_coordinate(double height_ellipsoid_m) {
    const double fine_top = weather_fine_layer_count * weather_fine_layer_thickness_m;
    double s;
    if (height_ellipsoid_m < fine_top) {
        s = height_ellipsoid_m / weather_fine_layer_thickness_m;
    } else {
        s = weather_fine_layer_count + (height_ellipsoid_m - fine_top) / weather_coarse_layer_thickness_m;
    }
    return std::clamp(s, 0.0, static_cast<double>(weather_layer_count));
}

std::uint8_t encode_weather_water(double kg_m3) {
    if (!(kg_m3 > 0.0)) return 0;
    const double code = std::log(kg_m3 / weather_water_code_min_kg_m3) / std::log(water_code_ratio) * 254.0 + 1.0;
    if (code < 0.5) return 0;
    return static_cast<std::uint8_t>(std::min(255.0, std::round(code)));
}

double decode_weather_water(std::uint8_t code) { return water_decode_table()[code]; }

std::uint8_t encode_weather_cloud_fraction(double fraction) {
    return static_cast<std::uint8_t>(std::round(std::clamp(finite_or_zero(fraction), 0.0, 1.0) * 255.0));
}

double decode_weather_cloud_fraction(std::uint8_t code) { return code / 255.0; }

std::vector<double> hypsometric_level_heights_gpm(std::span<const double> levels_pa,
                                                  std::span<const double> temperature_k,
                                                  double total_column_water_vapour_kg_m2, double surface_pressure_pa,
                                                  double surface_geopotential_m2_s2) {
    if (temperature_k.size() != levels_pa.size()) {
        throw std::invalid_argument("hypsometric_level_heights_gpm: spans differ in length");
    }
    std::vector<double> out(levels_pa.size()), tv(levels_pa.size());
    integrate_heights(descending_pressure_order(levels_pa), levels_pa, [&](int k) { return temperature_k[k]; },
                      total_column_water_vapour_kg_m2, surface_pressure_pa, surface_geopotential_m2_s2, out, tv);
    return out;
}

std::vector<double> level_virtual_temperatures_k(std::span<const double> levels_pa,
                                                 std::span<const double> temperature_k,
                                                 double total_column_water_vapour_kg_m2, double surface_pressure_pa) {
    if (temperature_k.size() != levels_pa.size()) {
        throw std::invalid_argument("level_virtual_temperatures_k: spans differ in length");
    }
    std::vector<double> z(levels_pa.size()), out(levels_pa.size());
    integrate_heights(descending_pressure_order(levels_pa), levels_pa, [&](int k) { return temperature_k[k]; },
                      total_column_water_vapour_kg_m2, surface_pressure_pa, 0.0, z, out);
    return out;
}

double geometric_height_m(double geopotential_height_m, double latitude_rad) {
    const double s2 = std::sin(latitude_rad) * std::sin(latitude_rad);
    const double gamma = c::wgs84_equatorial_gravity_mps2 * (1.0 + c::wgs84_somigliana_k * s2) /
                         std::sqrt(1.0 - c::wgs84_first_eccentricity_squared * s2);
    const double radius = c::wgs84_semi_major_axis_m /
                          (1.0 + c::wgs84_flattening + c::wgs84_gravity_ratio_m - 2.0 * c::wgs84_flattening * s2);
    return radius * geopotential_height_m / (gamma / c::standard_gravity_mps2 * radius - geopotential_height_m);
}

double cloud_water_density_kg_m3(double specific_kg_kg, double pressure_pa, double temperature_k) {
    return specific_kg_kg * pressure_pa / (c::dry_air_gas_constant_j_kgk * temperature_k);
}

double lifting_condensation_level_m(double t2m_k, double td2m_k, double pressure_pa) {
    if (!(t2m_k > 0.0) || !(td2m_k > 0.0) || !(pressure_pa > 0.0) || !std::isfinite(t2m_k) || !std::isfinite(td2m_k) ||
        !std::isfinite(pressure_pa)) {
        return nan;
    }
    if (td2m_k >= t2m_k) return 0.0;
    const double ra = c::romps_dry_air_gas_constant_j_kgk, rv = c::romps_vapour_gas_constant_j_kgk;
    const double cva = c::romps_dry_air_cv_j_kgk, cvv = c::romps_vapour_cv_j_kgk, cvl = c::romps_liquid_c_j_kgk;
    const double t0 = c::romps_triple_point_temperature_k;
    // Vapour pressure of the dew point; the parcel's mass fraction of vapour, gas constant and heat capacity.
    const double pv = romps_saturation_pressure_pa(td2m_k);
    const double relative_humidity = pv / romps_saturation_pressure_pa(t2m_k);
    const double qv = ra * pv / (rv * pressure_pa + (ra - rv) * pv);
    const double rm = (1.0 - qv) * ra + qv * rv;
    const double cpm = (1.0 - qv) * (cva + ra) + qv * (cvv + rv);
    // Romps 2017: z = cpm T / g (1 - b / (a W_-1((b / a) c^(1/a)))), with a, b, c from his eqs. 22-24.
    const double a = -(cvv + rv - cvl) / rv + cpm / rm;
    const double b = -(c::romps_vapour_internal_energy_at_triple_j_kg - (cvv - cvl) * t0) / (rv * t2m_k);
    const double cc = relative_humidity * std::exp(b);
    const double w = lambert_w_minus1(b / a * std::pow(cc, 1.0 / a));
    return std::max(0.0, cpm * t2m_k / c::romps_gravity_mps2 * (1.0 - b / (a * w)));
}

double standard_pressure_at_height_pa(double height_m) {
    const double exponent = c::standard_gravity_mps2 / (c::dry_air_gas_constant_j_kgk * c::standard_lapse_rate_k_per_m);
    return c::standard_pressure_pa *
           std::pow(1.0 - c::standard_lapse_rate_k_per_m * height_m / c::standard_temperature_k, exponent);
}

void raise_liquid_cloud_to_lcl(std::span<double> cloud_fraction, std::span<double> liquid_kg_m3,
                               std::span<const double> ice_kg_m3, double ground_m, double lcl_m) {
    const int n = static_cast<int>(liquid_kg_m3.size());
    if (!(lcl_m > 0.0) || !(lcl_m < c::cloud_boundary_layer_top_m) || cloud_fraction.size() != liquid_kg_m3.size() ||
        ice_kg_m3.size() != liquid_kg_m3.size()) {
        return;
    }
    const auto i = [](int k) { return static_cast<std::size_t>(k); };
    const auto bottom_of = [&](int k) { return std::max(weather_layer_bottom_m(k), ground_m); };  // above the ground
    const auto top_of = [](int k) { return weather_layer_bottom_m(k + 1); };
    const auto significant = [&](int k) {
        return top_of(k) > ground_m && significant_liquid(cloud_fraction[i(k)], liquid_kg_m3[i(k)]);
    };
    // The lowest significant liquid cloud and its run of significant layers: a boundary-layer cloud when its base lies
    // less than cloud_boundary_layer_top_m above the ground.
    int first = 0;
    while (first < n && !significant(first)) ++first;
    if (first == n || bottom_of(first) - ground_m >= c::cloud_boundary_layer_top_m) return;
    int first_end = first;
    while (first_end < n && significant(first_end)) ++first_end;
    const double lcl_height_m = ground_m + lcl_m;
    // All liquid below the LCL (every run, traces included): its mass and mass-weighted cloud fraction.
    double mass_below = 0.0, fraction_mass = 0.0;
    for (int k = 0; k < n && bottom_of(k) < lcl_height_m; ++k) {
        if (!(top_of(k) > bottom_of(k))) continue;  // under the ground
        const double cut = std::min(lcl_height_m, top_of(k)) - bottom_of(k);
        mass_below += liquid_kg_m3[i(k)] * cut;
        fraction_mass += cloud_fraction[i(k)] * liquid_kg_m3[i(k)] * cut;
    }
    if (!(mass_below > 0.0)) return;
    const double fraction_below = fraction_mass / mass_below;
    // The cloud it goes to: the lowest significant run with a part above the LCL, its base (the LCL at the lowest) in
    // the boundary layer and at most the depth of the lowest run above the LCL (it is then the continuation of the
    // same cloud; a deck further up is another cloud and keeps its water); none: the lowest run is lifted.
    const double lowest_depth_m = top_of(first_end - 1) - bottom_of(first);
    int target = 0;
    while (target < n && !(significant(target) && top_of(target) > lcl_height_m)) ++target;
    if (target < n && (std::max(bottom_of(target), lcl_height_m) - ground_m >= c::cloud_boundary_layer_top_m ||
                       bottom_of(target) > lcl_height_m + lowest_depth_m)) {
        target = n;
    }
    int target_end = target;
    while (target_end < n && significant(target_end)) ++target_end;
    // The layers under the LCL keep their part above it (a layer mean over the whole layer); a wholly cut layer loses
    // its cloud fraction unless ice holds it.
    for (int k = 0; k < n && bottom_of(k) < lcl_height_m; ++k) {
        if (!(top_of(k) > bottom_of(k))) continue;
        const double kept = std::max(0.0, top_of(k) - std::max(lcl_height_m, bottom_of(k)));
        liquid_kg_m3[i(k)] *= kept / (top_of(k) - bottom_of(k));
        if (!(kept > 0.0) && !(ice_kg_m3[i(k)] > 0.0)) cloud_fraction[i(k)] = 0.0;
    }
    if (target < n) {
        // Spread over the target run (its part above the LCL) in proportion to its liquid: the profile above is kept,
        // the column's liquid water path too; the cloud fraction there is at least the moved water's mass-weighted one.
        double mass_target = 0.0;
        for (int k = target; k < target_end; ++k) mass_target += liquid_kg_m3[i(k)] * (top_of(k) - bottom_of(k));
        const double scale = (mass_target + mass_below) / mass_target;  // mass_target > 0: significant above the LCL
        for (int k = target; k < target_end; ++k) {
            liquid_kg_m3[i(k)] *= scale;
            cloud_fraction[i(k)] = std::max(cloud_fraction[i(k)], fraction_below);
        }
        return;
    }
    // No significant cloud above the LCL in the boundary layer: the lowest run (wholly under the LCL) is lifted to
    // start there, keeping its depth; it carries all the liquid that was under the level (its mean density for one run).
    const double depth_m = lowest_depth_m, density = mass_below / depth_m;
    for (int k = 0; k < n; ++k) {
        const double bottom = weather_layer_bottom_m(k), top = top_of(k);
        const double overlap = std::min(top, lcl_height_m + depth_m) - std::max(bottom, lcl_height_m);
        if (!(overlap > 0.0)) continue;
        liquid_kg_m3[i(k)] += density * overlap / (top - bottom);
        cloud_fraction[i(k)] = std::max(cloud_fraction[i(k)], fraction_below);
    }
}

double cloud_column_cover_max_random(std::span<const double> cloud_fraction, std::span<const double> liquid_kg_m3,
                                     std::span<const double> ice_kg_m3, double ground_m) {
    const int n = static_cast<int>(cloud_fraction.size());
    if (liquid_kg_m3.size() != cloud_fraction.size() || ice_kg_m3.size() != cloud_fraction.size()) return 0.0;
    double clear = 1.0, above = 0.0;
    for (int k = n - 1; k >= 0; --k) {
        const auto i = static_cast<std::size_t>(k);
        const bool cloudy =
            weather_layer_bottom_m(k + 1) > ground_m && stores_condensate(liquid_kg_m3[i], ice_kg_m3[i]);
        const double f = cloudy ? std::clamp(cloud_fraction[i], 0.0, 1.0) : 0.0;
        if (above < 1.0) clear *= (1.0 - std::max(f, above)) / (1.0 - above);
        above = f;
    }
    return 1.0 - clear;
}

void scale_cloud_cover_to_total(std::span<double> cloud_fraction, std::span<const double> liquid_kg_m3,
                                std::span<const double> ice_kg_m3, double ground_m, double total_cloud_cover) {
    const std::size_t n = cloud_fraction.size();
    if (!(total_cloud_cover >= 0.0) || liquid_kg_m3.size() != n || ice_kg_m3.size() != n) return;
    const double target = std::min(total_cloud_cover, 1.0);
    // The layers that count for the cover (condensate above the ground) and the largest of the fractions that are scaled:
    // a trace (at or below ERA5's cloud-presence threshold) keeps its fraction, it must not become a cloud (and a cloud
    // base) under the data's.
    std::array<bool, weather_layer_count> cloudy{};
    double largest = 0.0;
    for (std::size_t k = 0; k < n && k < cloudy.size(); ++k) {
        cloudy[k] = weather_layer_bottom_m(static_cast<int>(k) + 1) > ground_m &&
                    stores_condensate(liquid_kg_m3[k], ice_kg_m3[k]);
        if (cloudy[k] && cloud_fraction[k] > c::cloud_trace_fraction) largest = std::max(largest, cloud_fraction[k]);
    }
    if (!(largest > 0.0)) return;
    const std::vector<double> original(cloud_fraction.begin(), cloud_fraction.end());
    const auto apply = [&](double s) {
        for (std::size_t k = 0; k < n && k < cloudy.size(); ++k) {
            // Only the cloud's layers: a fraction without condensate (nothing shows) or a trace keeps its value.
            if (!(cloudy[k] && original[k] > c::cloud_trace_fraction)) continue;
            cloud_fraction[k] = std::max(std::min(1.0, s * original[k]), c::cloud_min_cover);
        }
        return cloud_column_cover_max_random(cloud_fraction, liquid_kg_m3, ice_kg_m3, ground_m);
    };
    // The cover rises monotonically with the factor; at 1 / largest the largest fraction is 1 (cover 1). Bisection to
    // round-off: the result is the upper end (its cover >= the target).
    double lo = 0.0, hi = 1.0 / largest;
    for (int it = 0; it < 64 && hi - lo > 1e-15 * hi; ++it) {
        const double mid = 0.5 * (lo + hi);
        (apply(mid) >= target ? hi : lo) = mid;
    }
    apply(hi);
}

double bound_incloud_condensate(std::span<const double> cloud_fraction, std::span<double> liquid_kg_m3,
                                std::span<double> ice_kg_m3, double ground_m) {
    const std::size_t n = cloud_fraction.size();
    if (liquid_kg_m3.size() != n || ice_kg_m3.size() != n) return 0.0;
    double dropped_kg_m2 = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
        const double top = weather_layer_bottom_m(static_cast<int>(k) + 1);
        if (!(top > ground_m)) continue;
        const double thickness = top - std::max(weather_layer_bottom_m(static_cast<int>(k)), ground_m);
        // The renderer shows a layer's water on at least cloud_min_cover.
        const double fraction = std::max(cloud_fraction[k], c::cloud_min_cover);
        const double liquid_max = c::cloud_liquid_water_max_kg_m3 * fraction, ice_max = c::cloud_ice_water_max_kg_m3 * fraction;
        if (liquid_kg_m3[k] > liquid_max) {
            dropped_kg_m2 += (liquid_kg_m3[k] - liquid_max) * thickness;
            liquid_kg_m3[k] = liquid_max;
        }
        if (ice_kg_m3[k] > ice_max) {
            dropped_kg_m2 += (ice_kg_m3[k] - ice_max) * thickness;
            ice_kg_m3[k] = ice_max;
        }
    }
    return dropped_kg_m2;
}

double adiabatic_liquid_water_gradient_kg_m3_per_m(double temperature_k, double pressure_pa) {
    if (!(temperature_k > 0.0) || !(pressure_pa > 0.0) || !std::isfinite(temperature_k) ||
        !std::isfinite(pressure_pa)) {
        return 0.0;
    }
    const double ra = c::romps_dry_air_gas_constant_j_kgk, rv = c::romps_vapour_gas_constant_j_kgk;
    const double cpd = c::romps_dry_air_cv_j_kgk + ra, g = c::romps_gravity_mps2, t = temperature_k, p = pressure_pa;
    const double es = romps_saturation_pressure_pa(t);
    if (!(es < p)) return 0.0;
    const double latent =
        c::romps_vapour_internal_energy_at_triple_j_kg + rv * t +
        (c::romps_vapour_cv_j_kgk - c::romps_liquid_c_j_kgk) * (t - c::romps_triple_point_temperature_k);
    const double rs = ra / rv * es / (p - es);
    // Partial derivatives of r_s: d ln e_s / dT = L / (R_v T^2) holds exactly for Romps' e_s (Kirchhoff's law).
    const double drs_dt = rs * p / (p - es) * latent / (rv * t * t);
    const double drs_dp = -rs / (p - es);
    const double dp_dz = -g * ((p - es) / (ra * t) + es / (rv * t));  // hydrostatic, moist air
    const double dt_dz = -(g + latent * drs_dp * dp_dz) / (cpd + latent * drs_dt);
    const double dry_density = (p - es) / (ra * t);
    return std::max(0.0, -dry_density * (drs_dt * dt_dz + drs_dp * dp_dz));
}

double add_cumulus_adiabatic_water(std::span<const double> cloud_fraction, std::span<double> liquid_kg_m3,
                                   std::span<const double> ice_kg_m3, std::span<const double> temperature_k,
                                   std::span<const double> pressure_pa, double ground_m, double lcl_m,
                                   const AdiabaticFractionProfile& profile) {
    const int n = static_cast<int>(liquid_kg_m3.size());
    if (!(lcl_m > 0.0) || !(lcl_m < c::cloud_boundary_layer_top_m) || !(profile.at_base > 0.0) ||
        !(profile.at_reference >= 0.0) ||
        cloud_fraction.size() != liquid_kg_m3.size() || ice_kg_m3.size() != liquid_kg_m3.size() ||
        temperature_k.size() != liquid_kg_m3.size() || pressure_pa.size() != liquid_kg_m3.size()) {
        return 0.0;
    }
    const auto i = [](int k) { return static_cast<std::size_t>(k); };
    const auto bottom_of = [&](int k) { return std::max(weather_layer_bottom_m(k), ground_m); };
    const auto top_of = [](int k) { return weather_layer_bottom_m(k + 1); };
    const auto significant = [&](int k) {
        return top_of(k) > ground_m && significant_liquid(cloud_fraction[i(k)], liquid_kg_m3[i(k)]);
    };
    // The lowest significant liquid run, a boundary-layer cloud (raise_liquid_cloud_to_lcl's rule).
    int first = 0;
    while (first < n && !significant(first)) ++first;
    if (first == n || bottom_of(first) - ground_m >= c::cloud_boundary_layer_top_m) return 0.0;
    int end = first;
    while (end < n && significant(end)) ++end;
    const double base_m = std::max(ground_m + lcl_m, bottom_of(first));
    // Its path above the base: the in-cloud water path (a layer cut by the base holds its mean over the whole layer, so
    // its in-cloud water over the part d above the base is L t / (C d)) and the sub-adiabatic one of the parcel, which
    // ends at the boundary-layer top (the shallow-cumulus range of f_ad; a congestus / cumulonimbus above keeps the
    // data's water, its in-cloud path still counts); each path layer's cumulus weight.
    const double parcel_top_m = ground_m + c::cloud_boundary_layer_top_m;
    // f_ad(h) h integrated from the base to the height h (m): linear f_ad down to the reference height, constant above.
    const double reference_m = c::cumulus_adiabatic_fraction_reference_height_m;
    const double slope = (profile.at_reference - profile.at_base) / reference_m;
    const auto moment = [&](double h) {
        const double lower = std::min(h, reference_m);
        const double below = 0.5 * profile.at_base * lower * lower + slope * lower * lower * lower / 3.0;
        return below + 0.5 * profile.at_reference * (h * h - lower * lower);
    };
    double in_cloud_path = 0.0, floor_path = 0.0;  // weighted by the layers' cumulus weights
    std::array<double, weather_layer_count> cumulus{};  // 0 off the path
    for (int k = first; k < end && k < weather_layer_count; ++k) {
        const double low = std::max(bottom_of(k), base_m), part = top_of(k) - low;
        if (!(part > c::cloud_base_snap_m)) continue;  // a sliver under the base: snapped away by the renderer
        const double thickness = top_of(k) - bottom_of(k);
        const double fraction = cloud_fraction[i(k)], liquid = liquid_kg_m3[i(k)], ice = ice_kg_m3[i(k)];
        const double h_low = std::min(low, parcel_top_m) - base_m, h_high = std::min(top_of(k), parcel_top_m) - base_m;
        const double extinction = cloud_extinction_per_m(liquid, ice);
        const double ice_share = extinction > 0.0 ? cloud_extinction_per_m(0.0, ice) / extinction : 0.0;
        const double fraction_above = k + 1 < n ? cloud_fraction[i(k + 1)] : 0.0;
        const double weight = cloud_type_weights(cloud_stratiform_weight(fraction, fraction_above), ice_share).cumulus;
        cumulus[i(k)] = weight;
        // Weighted, so that the layers that get water (a deck under the cumulus gets none) meet the floor.
        in_cloud_path += weight * liquid * thickness / fraction;
        floor_path += weight * adiabatic_liquid_water_gradient_kg_m3_per_m(temperature_k[i(k)], pressure_pa[i(k)]) *
                      (moment(h_high) - moment(h_low));
    }
    if (!(in_cloud_path > 0.0) || !(floor_path > in_cloud_path)) return 0.0;
    const double ratio_to_floor = floor_path / in_cloud_path;
    double added = 0.0;
    for (int k = first; k < end && k < weather_layer_count; ++k) {
        if (!(cumulus[i(k)] > 0.0)) continue;
        const double thickness = top_of(k) - bottom_of(k);
        const double part = top_of(k) - std::max(bottom_of(k), base_m);
        // The layer's factor by its cumulus weight; its in-cloud water (over the part above the base) not raised above
        // the bound (a denser layer keeps its).
        const double factor = 1.0 + cumulus[i(k)] * (ratio_to_floor - 1.0);
        const double in_cloud = liquid_kg_m3[i(k)] * thickness / (cloud_fraction[i(k)] * part);
        const double ratio = std::max(1.0, std::min(factor, c::cumulus_liquid_water_max_kg_m3 / in_cloud));
        const double liquid = liquid_kg_m3[i(k)] * ratio;
        added += (liquid - liquid_kg_m3[i(k)]) * thickness;
        liquid_kg_m3[i(k)] = liquid;
    }
    return added;
}

WeatherSnapshot::WeatherSnapshot(const WeatherSnapshotId& id, int rows, int columns)
    : id_(id), rows_(rows), columns_(columns),
      ground_m_(static_cast<std::size_t>(rows) * columns, 0.0f),
      lcl_m_(static_cast<std::size_t>(rows) * columns, weather_lcl_unknown_m),
      total_cloud_cover_(static_cast<std::size_t>(rows) * columns, weather_total_cloud_cover_unknown),
      added_liquid_path_kg_m2_(static_cast<std::size_t>(rows) * columns, 0.0f),
      dropped_condensate_path_kg_m2_(static_cast<std::size_t>(rows) * columns, 0.0f),
      texels_(static_cast<std::size_t>(weather_layer_count) * rows * columns * channels, 0) {}

std::size_t WeatherSnapshot::texel_index(int layer, int row, int column) const {
    return ((static_cast<std::size_t>(layer) * rows_ + row) * columns_ + column) * channels;
}

WeatherSnapshot WeatherSnapshot::from_era5(const WeatherSnapshotId& id, const Era5Fields& f, const Geoid& geoid) {
    const int levels = static_cast<int>(f.levels_pa.size());
    const int rows = f.temperature_k.rows, columns = f.temperature_k.columns;
    require_grid(rows, columns);
    require_shape(f.cloud_fraction, levels, rows, columns, "cloud_fraction");
    require_shape(f.liquid_kg_kg, levels, rows, columns, "liquid_kg_kg");
    require_shape(f.ice_kg_kg, levels, rows, columns, "ice_kg_kg");
    require_shape(f.temperature_k, levels, rows, columns, "temperature_k");
    require_shape(f.surface_pressure_pa, 1, rows, columns, "surface_pressure_pa");
    require_shape(f.surface_geopotential_m2_s2, 1, rows, columns, "surface_geopotential_m2_s2");
    require_shape(f.total_column_water_vapour_kg_m2, 1, rows, columns, "total_column_water_vapour_kg_m2");
    const bool has_lcl = require_optional_2m(f.temperature_2m_k, f.dewpoint_2m_k, rows, columns);
    if (!f.total_cloud_cover.values.empty()) require_shape(f.total_cloud_cover, 1, rows, columns, "total_cloud_cover");

    WeatherSnapshot s(id, rows, columns);
    const std::vector<int> order = descending_pressure_order(f.levels_pa);
    const std::size_t layer_stride = static_cast<std::size_t>(rows) * columns * channels;
    for_each_row(rows, [&](int r) {
        const double lat = row_latitude_rad(r, rows);
        std::vector<double> z(static_cast<std::size_t>(levels)), tv(static_cast<std::size_t>(levels));
        std::vector<LevelSample> samples;
        samples.reserve(static_cast<std::size_t>(levels));
        ColumnLayers layers;
        for (int col = 0; col < columns; ++col) {
            const double n = geoid.undulation_m(lat, column_longitude_rad(col, columns));
            const double ps = cell(f.surface_pressure_pa, 0, r, col);
            const double phis = cell(f.surface_geopotential_m2_s2, 0, r, col);
            integrate_heights(order, f.levels_pa, [&](int k) { return cell(f.temperature_k, k, r, col); },
                              cell(f.total_column_water_vapour_kg_m2, 0, r, col), ps, phis, z, tv);
            samples.clear();
            for (int k = 0; k < levels; ++k) {
                if (std::isnan(z[static_cast<std::size_t>(k)])) continue;
                const double p = f.levels_pa[static_cast<std::size_t>(k)];
                const double t = tv[static_cast<std::size_t>(k)];  // ERA5 contents are per kg of moist air
                samples.push_back(
                    {geometric_height_m(z[static_cast<std::size_t>(k)], lat) + n,
                     {std::clamp(finite_or_zero(cell(f.cloud_fraction, k, r, col)), 0.0, 1.0),
                      cloud_water_density_kg_m3(std::max(0.0, finite_or_zero(cell(f.liquid_kg_kg, k, r, col))), p, t),
                      cloud_water_density_kg_m3(std::max(0.0, finite_or_zero(cell(f.ice_kg_kg, k, r, col))), p, t),
                      finite_or_zero(cell(f.temperature_k, k, r, col)), p}});
            }
            const double ground = geometric_height_m(finite_or_zero(phis) / c::standard_gravity_mps2, lat) + n;
            s.ground_m_[static_cast<std::size_t>(r) * columns + col] = static_cast<float>(ground);
            float& lcl = s.lcl_m_[static_cast<std::size_t>(r) * columns + col];
            if (has_lcl) lcl = column_lcl_m(f.temperature_2m_k, f.dewpoint_2m_k, r, col, ps);
            const std::size_t i = static_cast<std::size_t>(r) * columns + col;
            s.total_cloud_cover_[i] = column_total_cloud_cover(f.total_cloud_cover, r, col, 1.0);
            const auto built = layers.build(samples, ground, lcl, s.total_cloud_cover_[i],
                                            s.texels_.data() + s.texel_index(0, r, col), layer_stride);
            s.added_liquid_path_kg_m2_[i] = static_cast<float>(built.added_liquid_path_kg_m2);
            s.dropped_condensate_path_kg_m2_[i] = static_cast<float>(built.dropped_condensate_path_kg_m2);
        }
    });
    s.make_poles_uniform();
    return s;
}

WeatherSnapshot WeatherSnapshot::from_gfs(const WeatherSnapshotId& id, const GfsFields& f, const Geoid& geoid) {
    const int levels = static_cast<int>(f.levels_pa.size());
    const int rows = f.geopotential_height_m.rows, columns = f.geopotential_height_m.columns;
    require_grid(rows, columns);
    require_shape(f.cloud_percent, levels, rows, columns, "cloud_percent");
    require_shape(f.liquid_kg_kg, levels, rows, columns, "liquid_kg_kg");
    require_shape(f.ice_kg_kg, levels, rows, columns, "ice_kg_kg");
    require_shape(f.temperature_k, levels, rows, columns, "temperature_k");
    require_shape(f.geopotential_height_m, levels, rows, columns, "geopotential_height_m");
    require_shape(f.surface_geopotential_height_m, 1, rows, columns, "surface_geopotential_height_m");
    const bool has_lcl = require_optional_2m(f.temperature_2m_k, f.dewpoint_2m_k, rows, columns);
    if (!f.total_cloud_percent.values.empty()) {
        require_shape(f.total_cloud_percent, 1, rows, columns, "total_cloud_percent");
    }

    WeatherSnapshot s(id, rows, columns);
    const std::size_t layer_stride = static_cast<std::size_t>(rows) * columns * channels;
    for_each_row(rows, [&](int r) {
        const double lat = row_latitude_rad(r, rows);
        std::vector<LevelSample> samples;
        samples.reserve(static_cast<std::size_t>(levels));
        ColumnLayers layers;
        for (int col = 0; col < columns; ++col) {
            const double n = geoid.undulation_m(lat, column_longitude_rad(col, columns));
            const double surface_gpm = finite_or_zero(cell(f.surface_geopotential_height_m, 0, r, col));
            samples.clear();
            for (int k = 0; k < levels; ++k) {
                const double z = cell(f.geopotential_height_m, k, r, col);
                if (!std::isfinite(z) || !(z > surface_gpm)) continue;  // under the ground: extrapolated by GFS
                const double p = f.levels_pa[static_cast<std::size_t>(k)];
                const double t = cell(f.temperature_k, k, r, col);
                const bool density_known = std::isfinite(t) && t > 0.0;
                const auto water = [&](const FloatGrid& g) {
                    return density_known ? cloud_water_density_kg_m3(std::max(0.0, finite_or_zero(cell(g, k, r, col))), p, t)
                                         : 0.0;
                };
                samples.push_back({geometric_height_m(z, lat) + n,
                                   {std::clamp(finite_or_zero(cell(f.cloud_percent, k, r, col)) / 100.0, 0.0, 1.0),
                                    water(f.liquid_kg_kg), water(f.ice_kg_kg), density_known ? t : 0.0, p}});
            }
            const double ground = geometric_height_m(surface_gpm, lat) + n;
            s.ground_m_[static_cast<std::size_t>(r) * columns + col] = static_cast<float>(ground);
            // GFS 2 m fields come without a surface pressure: the standard atmosphere's at the column's ground height.
            float& lcl = s.lcl_m_[static_cast<std::size_t>(r) * columns + col];
            if (has_lcl) {
                lcl = column_lcl_m(f.temperature_2m_k, f.dewpoint_2m_k, r, col, standard_pressure_at_height_pa(surface_gpm));
            }
            const std::size_t i = static_cast<std::size_t>(r) * columns + col;
            s.total_cloud_cover_[i] = column_total_cloud_cover(f.total_cloud_percent, r, col, 0.01);  // percent
            const auto built = layers.build(samples, ground, lcl, s.total_cloud_cover_[i],
                                            s.texels_.data() + s.texel_index(0, r, col), layer_stride);
            s.added_liquid_path_kg_m2_[i] = static_cast<float>(built.added_liquid_path_kg_m2);
            s.dropped_condensate_path_kg_m2_[i] = static_cast<float>(built.dropped_condensate_path_kg_m2);
        }
    });
    s.make_poles_uniform();
    return s;
}

WeatherSnapshot WeatherSnapshot::uniform_slab(int rows, int columns, double bottom_m, double top_m,
                                              double liquid_water_kg_m3, double cloud_fraction) {
    require_grid(rows, columns);
    WeatherSnapshot s(WeatherSnapshotId{}, rows, columns);
    for (int layer = 0; layer < weather_layer_count; ++layer) {
        const double b = weather_layer_bottom_m(layer), t = weather_layer_bottom_m(layer + 1);
        const double overlap = std::max(0.0, std::min(t, top_m) - std::max(b, bottom_m)) / (t - b);
        const std::uint8_t cf = encode_weather_cloud_fraction(cloud_fraction * overlap);
        const std::uint8_t lw = encode_weather_water(liquid_water_kg_m3 * overlap);
        for (int r = 0; r < rows; ++r) {
            for (int col = 0; col < columns; ++col) {
                const std::size_t i = s.texel_index(layer, r, col);
                s.texels_[i] = cf;
                s.texels_[i + 1] = lw;
            }
        }
    }
    return s;
}

WeatherSnapshot WeatherSnapshot::gfs_test_column(int rows, int columns, double cloud_top_m, double liquid_kg_kg,
                                                 double cloud_percent, double t2m_k, double td2m_k,
                                                 double total_cloud_percent) {
    require_grid(rows, columns);
    // Level heights (geopotential m): every 100 m from 25 m to 4 km, every 500 m to the tropopause of the standard
    // atmosphere's lapse-rate formula (11 km).
    constexpr double fine_step_m = 100.0, coarse_step_m = 500.0, fine_top_m = 4000.0, tropopause_m = 11000.0;
    std::vector<double> heights;
    for (double h = 0.25 * fine_step_m; h < fine_top_m; h += fine_step_m) heights.push_back(h);
    for (double h = fine_top_m + 0.5 * coarse_step_m; h < tropopause_m; h += coarse_step_m) heights.push_back(h);
    const int levels = static_cast<int>(heights.size());
    const auto grid_of = [&](int n) {
        FloatGrid g;
        g.levels = n;
        g.rows = rows;
        g.columns = columns;
        g.values.assign(static_cast<std::size_t>(n) * rows * columns, 0.0f);
        return g;
    };
    GfsFields f;
    f.cloud_percent = f.liquid_kg_kg = f.ice_kg_kg = f.temperature_k = f.geopotential_height_m = grid_of(levels);
    f.surface_geopotential_height_m = grid_of(1);
    f.temperature_2m_k = f.dewpoint_2m_k = grid_of(1);
    const std::size_t cells = static_cast<std::size_t>(rows) * columns;
    for (int k = 0; k < levels; ++k) {
        const double h = heights[static_cast<std::size_t>(k)];
        f.levels_pa.push_back(standard_pressure_at_height_pa(h));
        const bool cloud = h < cloud_top_m;
        for (std::size_t i = 0; i < cells; ++i) {
            const std::size_t j = static_cast<std::size_t>(k) * cells + i;
            f.geopotential_height_m.values[j] = static_cast<float>(h);
            f.temperature_k.values[j] =
                static_cast<float>(c::standard_temperature_k - c::standard_lapse_rate_k_per_m * h);
            f.liquid_kg_kg.values[j] = cloud ? static_cast<float>(liquid_kg_kg) : 0.0f;
            f.cloud_percent.values[j] = cloud ? static_cast<float>(cloud_percent) : 0.0f;
        }
    }
    std::fill(f.temperature_2m_k.values.begin(), f.temperature_2m_k.values.end(), static_cast<float>(t2m_k));
    std::fill(f.dewpoint_2m_k.values.begin(), f.dewpoint_2m_k.values.end(), static_cast<float>(td2m_k));
    if (total_cloud_percent >= 0.0) {
        f.total_cloud_percent = grid_of(1);
        std::fill(f.total_cloud_percent.values.begin(), f.total_cloud_percent.values.end(),
                  static_cast<float>(total_cloud_percent));
    }
    GeographicGrid flat;  // a zero geoid: heights above the geoid are heights above the ellipsoid
    flat.grid.lines = flat.grid.samples = 2;
    flat.grid.dn_scale_m = 1.0;
    flat.grid.dn.assign(4, 0);
    flat.north_latitude_deg = 90.0;
    flat.step_latitude_deg = flat.step_longitude_deg = 180.0;
    flat.global_longitude = true;
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::gfs;
    return from_gfs(id, f, Geoid(std::move(flat)));
}

WeatherSnapshot WeatherSnapshot::from_parts(const WeatherSnapshotId& id, int rows, int columns,
                                            std::vector<float> ground_m, std::vector<std::uint8_t> texels,
                                            std::vector<float> lcl_m, std::vector<float> total_cloud_cover,
                                            std::vector<float> added_liquid_path_kg_m2,
                                            std::vector<float> dropped_condensate_path_kg_m2) {
    require_grid(rows, columns);
    const std::size_t cells = static_cast<std::size_t>(rows) * columns;
    if (lcl_m.empty()) lcl_m.assign(cells, weather_lcl_unknown_m);
    if (total_cloud_cover.empty()) total_cloud_cover.assign(cells, weather_total_cloud_cover_unknown);
    if (added_liquid_path_kg_m2.empty()) added_liquid_path_kg_m2.assign(cells, 0.0f);
    if (dropped_condensate_path_kg_m2.empty()) dropped_condensate_path_kg_m2.assign(cells, 0.0f);
    if (ground_m.size() != cells || lcl_m.size() != cells || total_cloud_cover.size() != cells ||
        added_liquid_path_kg_m2.size() != cells || dropped_condensate_path_kg_m2.size() != cells ||
        texels.size() != static_cast<std::size_t>(weather_layer_count) * rows * columns * channels) {
        throw std::invalid_argument("weather snapshot: parts do not match the grid");
    }
    WeatherSnapshot s(id, 2, 2);
    s.rows_ = rows;
    s.columns_ = columns;
    s.ground_m_ = std::move(ground_m);
    s.lcl_m_ = std::move(lcl_m);
    s.total_cloud_cover_ = std::move(total_cloud_cover);
    s.added_liquid_path_kg_m2_ = std::move(added_liquid_path_kg_m2);
    s.dropped_condensate_path_kg_m2_ = std::move(dropped_condensate_path_kg_m2);
    s.texels_ = std::move(texels);
    return s;
}

void WeatherSnapshot::make_poles_uniform() {
    // All columns of a pole row are one point: their mean (ground and every layer) so no longitude seam appears there.
    const auto& table = water_decode_table();
    for (int r : {0, rows_ - 1}) {
        double ground = 0.0, added = 0.0, dropped = 0.0;
        LclMin lcl;
        CoverMean cover;
        for (int col = 0; col < columns_; ++col) {
            const std::size_t i = static_cast<std::size_t>(r) * columns_ + col;
            ground += ground_m_[i];
            added += added_liquid_path_kg_m2_[i];
            dropped += dropped_condensate_path_kg_m2_[i];
            lcl.add(lcl_m_[i], ground_m_[i]);
            cover.add(total_cloud_cover_[i], 1.0);
        }
        ground /= columns_;
        for (int col = 0; col < columns_; ++col) {
            const std::size_t i = static_cast<std::size_t>(r) * columns_ + col;
            ground_m_[i] = static_cast<float>(ground);
            lcl_m_[i] = lcl.value(ground);
            total_cloud_cover_[i] = cover.value();
            added_liquid_path_kg_m2_[i] = static_cast<float>(added / columns_);
        }
        // The pole's mean column, its cover scaled to the mean total cloud cover (as a downsampled box).
        std::array<double, weather_layer_count> fraction{}, liquid{}, ice{};
        for (int layer = 0; layer < weather_layer_count; ++layer) {
            const auto k = static_cast<std::size_t>(layer);
            for (int col = 0; col < columns_; ++col) {
                const std::size_t i = texel_index(layer, r, col);
                fraction[k] += decode_weather_cloud_fraction(texels_[i]) / columns_;
                liquid[k] += table[texels_[i + 1]] / columns_;
                ice[k] += table[texels_[i + 2]] / columns_;
            }
        }
        scale_cloud_cover_to_total(fraction, liquid, ice, ground, cover.value());
        dropped = dropped / columns_ + bound_incloud_condensate(fraction, liquid, ice, ground);
        for (int col = 0; col < columns_; ++col) {
            dropped_condensate_path_kg_m2_[static_cast<std::size_t>(r) * columns_ + col] = static_cast<float>(dropped);
        }
        for (int layer = 0; layer < weather_layer_count; ++layer) {
            const auto k = static_cast<std::size_t>(layer);
            const std::uint8_t cf = encode_weather_cloud_fraction(fraction[k]);
            const std::uint8_t lw = encode_weather_water(liquid[k]);
            const std::uint8_t iw = encode_weather_water(ice[k]);
            for (int col = 0; col < columns_; ++col) {
                const std::size_t i = texel_index(layer, r, col);
                texels_[i] = cf;
                texels_[i + 1] = lw;
                texels_[i + 2] = iw;
            }
        }
    }
}

double WeatherSnapshot::ground_height_m(double latitude_rad, double longitude_rad) const {
    if (!std::isfinite(latitude_rad) || !std::isfinite(longitude_rad)) return nan;
    const double y = std::clamp((c::pi / 2.0 - latitude_rad) / (c::pi / (rows_ - 1)), 0.0, rows_ - 1.0);
    const int r0 = std::min(static_cast<int>(y), rows_ - 2);
    const double ty = y - r0;
    double x = longitude_rad / (c::two_pi / columns_);
    x -= std::floor(x / columns_) * columns_;
    const int c0 = std::min(static_cast<int>(x), columns_ - 1);
    const int c1 = (c0 + 1) % columns_;
    const double tx = x - c0;
    const auto g = [&](int r, int col) { return static_cast<double>(ground_m_[static_cast<std::size_t>(r) * columns_ + col]); };
    return (1.0 - ty) * ((1.0 - tx) * g(r0, c0) + tx * g(r0, c1)) + ty * ((1.0 - tx) * g(r0 + 1, c0) + tx * g(r0 + 1, c1));
}

WeatherCell WeatherSnapshot::at(double latitude_rad, double longitude_rad, double height_ellipsoid_m) const {
    if (!std::isfinite(latitude_rad) || !std::isfinite(longitude_rad) || !std::isfinite(height_ellipsoid_m)) return {};
    if (height_ellipsoid_m > weather_top_m) return {};
    if (height_ellipsoid_m < ground_height_m(latitude_rad, longitude_rad)) return {};
    const double y = std::clamp((c::pi / 2.0 - latitude_rad) / (c::pi / (rows_ - 1)), 0.0, rows_ - 1.0);
    const int r0 = std::min(static_cast<int>(y), rows_ - 2);
    const double ty = y - r0;
    double x = longitude_rad / (c::two_pi / columns_);
    x -= std::floor(x / columns_) * columns_;
    const int c0 = std::min(static_cast<int>(x), columns_ - 1);
    const int c1 = (c0 + 1) % columns_;
    const double tx = x - c0;
    const double s = std::clamp(weather_layer_coordinate(height_ellipsoid_m) - 0.5, 0.0, weather_layer_count - 1.0);
    const int l0 = std::min(static_cast<int>(s), weather_layer_count - 2);
    const double ts = s - l0;

    const auto& table = water_decode_table();
    WeatherCell out;
    for (int dl = 0; dl < 2; ++dl) {
        for (int dr = 0; dr < 2; ++dr) {
            for (int dc = 0; dc < 2; ++dc) {
                const double w = (dl ? ts : 1.0 - ts) * (dr ? ty : 1.0 - ty) * (dc ? tx : 1.0 - tx);
                if (w == 0.0) continue;
                const std::size_t i = texel_index(l0 + dl, r0 + dr, dc ? c1 : c0);
                out.cloud_fraction += w * decode_weather_cloud_fraction(texels_[i]);
                out.liquid_water_kg_m3 += w * table[texels_[i + 1]];
                out.ice_water_kg_m3 += w * table[texels_[i + 2]];
            }
        }
    }
    return out;
}

WeatherSnapshot WeatherSnapshot::downsampled(int factor) const {
    if (factor < 1 || (rows_ - 1) % factor != 0 || columns_ % factor != 0) {
        throw std::invalid_argument("weather snapshot: grid not divisible by the downsampling factor");
    }
    const int rows = (rows_ - 1) / factor + 1, columns = columns_ / factor;
    require_grid(rows, columns);
    WeatherSnapshot out(id_, rows, columns);
    // A centred box of width `factor`: offsets -h..h, the end cells half-weighted when the factor is even.
    const int h = factor / 2;
    const auto weight = [&](int d) { return (factor % 2 == 0 && std::abs(d) == h) ? 0.5 : 1.0; };
    const auto& table = water_decode_table();
    for_each_row(rows, [&](int r) {
        for (int col = 0; col < columns; ++col) {
            double weight_sum = 0.0, ground = 0.0, added = 0.0, dropped = 0.0;
            LclMin lcl;
            CoverMean cover;
            std::array<std::array<double, 3>, weather_layer_count> sums{};
            for (int dr = -h; dr <= h; ++dr) {
                const int sr = r * factor + dr;
                if (sr < 0 || sr >= rows_) continue;
                for (int dc = -h; dc <= h; ++dc) {
                    const int sc = ((col * factor + dc) % columns_ + columns_) % columns_;
                    const double w = weight(dr) * weight(dc);
                    weight_sum += w;
                    ground += w * ground_m_[static_cast<std::size_t>(sr) * columns_ + sc];
                    added += w * added_liquid_path_kg_m2_[static_cast<std::size_t>(sr) * columns_ + sc];
                    dropped += w * dropped_condensate_path_kg_m2_[static_cast<std::size_t>(sr) * columns_ + sc];
                    cover.add(total_cloud_cover_[static_cast<std::size_t>(sr) * columns_ + sc], w);
                    lcl.add(lcl_m_[static_cast<std::size_t>(sr) * columns_ + sc],
                            ground_m_[static_cast<std::size_t>(sr) * columns_ + sc]);
                    for (int layer = 0; layer < weather_layer_count; ++layer) {
                        const std::size_t i = texel_index(layer, sr, sc);
                        sums[layer][0] += w * decode_weather_cloud_fraction(texels_[i]);
                        sums[layer][1] += w * table[texels_[i + 1]];
                        sums[layer][2] += w * table[texels_[i + 2]];
                    }
                }
            }
            out.ground_m_[static_cast<std::size_t>(r) * columns + col] = static_cast<float>(ground / weight_sum);
            out.lcl_m_[static_cast<std::size_t>(r) * columns + col] = lcl.value(ground / weight_sum);
            out.total_cloud_cover_[static_cast<std::size_t>(r) * columns + col] = cover.value();
            out.added_liquid_path_kg_m2_[static_cast<std::size_t>(r) * columns + col] =
                static_cast<float>(added / weight_sum);
            // The box means of the layers no longer overlap as the cells did (a box's cloud spans its layers at
            // different places): the cover is scaled to the box's total cloud cover again, the water kept.
            std::array<double, weather_layer_count> fraction{}, liquid{}, ice{};
            for (int layer = 0; layer < weather_layer_count; ++layer) {
                const auto k = static_cast<std::size_t>(layer);
                fraction[k] = sums[layer][0] / weight_sum;
                liquid[k] = sums[layer][1] / weight_sum;
                ice[k] = sums[layer][2] / weight_sum;
            }
            const std::size_t cell = static_cast<std::size_t>(r) * columns + col;
            scale_cloud_cover_to_total(fraction, liquid, ice, out.ground_m_[cell], out.total_cloud_cover_[cell]);
            out.dropped_condensate_path_kg_m2_[cell] = static_cast<float>(
                dropped / weight_sum + bound_incloud_condensate(fraction, liquid, ice, out.ground_m_[cell]));
            for (int layer = 0; layer < weather_layer_count; ++layer) {
                const std::size_t i = out.texel_index(layer, r, col);
                const auto k = static_cast<std::size_t>(layer);
                out.texels_[i] = encode_weather_cloud_fraction(fraction[k]);
                out.texels_[i + 1] = encode_weather_water(liquid[k]);
                out.texels_[i + 2] = encode_weather_water(ice[k]);
            }
        }
    });
    out.make_poles_uniform();
    return out;
}

}  // namespace starsim::environment
