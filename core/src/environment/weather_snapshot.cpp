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

// One level of a column above the ground: ellipsoidal height and the three fields.
struct LevelSample {
    double h, values[3];
};

// The layer means of one column into the texels. The profile is piecewise linear in height between the samples, held
// below the lowest and above the highest; layer parts below the ground do not count.
class ColumnLayers {
public:
    // samples sorted by ascending height; empty = a clear column.
    void build(std::vector<LevelSample>& samples, double ground_m, std::uint8_t* texels, std::size_t layer_stride) {
        std::sort(samples.begin(), samples.end(), [](const LevelSample& a, const LevelSample& b) { return a.h < b.h; });
        samples_ = &samples;
        cumulative_.assign(samples.size(), {0.0, 0.0, 0.0});
        for (std::size_t i = 1; i < samples.size(); ++i) {
            const double dh = samples[i].h - samples[i - 1].h;
            for (int f = 0; f < 3; ++f) {
                cumulative_[i][f] = cumulative_[i - 1][f] + 0.5 * dh * (samples[i].values[f] + samples[i - 1].values[f]);
            }
        }
        cursor_ = 0;
        for (int layer = 0; layer < weather_layer_count; ++layer) {
            std::uint8_t* t = texels + static_cast<std::size_t>(layer) * layer_stride;
            t[0] = t[1] = t[2] = t[3] = 0;
            const double top = weather_layer_bottom_m(layer + 1);
            const double bottom = std::max(weather_layer_bottom_m(layer), ground_m);
            if (samples.empty() || !(top > bottom)) continue;
            const auto lo = integral_to(bottom), hi = integral_to(top);
            const double thickness = top - bottom;
            t[0] = encode_weather_cloud_fraction((hi[0] - lo[0]) / thickness);
            t[1] = encode_weather_water((hi[1] - lo[1]) / thickness);
            t[2] = encode_weather_water((hi[2] - lo[2]) / thickness);
        }
    }

private:
    // Integral of the profile from the lowest sample to x (negative below it); x must not decrease between calls.
    std::array<double, 3> integral_to(double x) {
        const auto& s = *samples_;
        std::array<double, 3> out{};
        if (x <= s.front().h) {
            for (int f = 0; f < 3; ++f) out[f] = (x - s.front().h) * s.front().values[f];
            return out;
        }
        while (cursor_ + 1 < s.size() && s[cursor_ + 1].h <= x) ++cursor_;
        const LevelSample& a = s[cursor_];
        const double dx = x - a.h;
        if (cursor_ + 1 == s.size()) {
            for (int f = 0; f < 3; ++f) out[f] = cumulative_[cursor_][f] + dx * a.values[f];
            return out;
        }
        const LevelSample& b = s[cursor_ + 1];
        const double w = dx / (b.h - a.h);  // b.h > x >= a.h
        for (int f = 0; f < 3; ++f) {
            const double vx = a.values[f] + w * (b.values[f] - a.values[f]);
            out[f] = cumulative_[cursor_][f] + 0.5 * dx * (a.values[f] + vx);
        }
        return out;
    }

    const std::vector<LevelSample>* samples_ = nullptr;
    std::vector<std::array<double, 3>> cumulative_;
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

WeatherSnapshot::WeatherSnapshot(const WeatherSnapshotId& id, int rows, int columns)
    : id_(id), rows_(rows), columns_(columns),
      ground_m_(static_cast<std::size_t>(rows) * columns, 0.0f),
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
                      cloud_water_density_kg_m3(std::max(0.0, finite_or_zero(cell(f.ice_kg_kg, k, r, col))), p, t)}});
            }
            const double ground = geometric_height_m(finite_or_zero(phis) / c::standard_gravity_mps2, lat) + n;
            s.ground_m_[static_cast<std::size_t>(r) * columns + col] = static_cast<float>(ground);
            layers.build(samples, ground, s.texels_.data() + s.texel_index(0, r, col), layer_stride);
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
                                    water(f.liquid_kg_kg), water(f.ice_kg_kg)}});
            }
            const double ground = geometric_height_m(surface_gpm, lat) + n;
            s.ground_m_[static_cast<std::size_t>(r) * columns + col] = static_cast<float>(ground);
            layers.build(samples, ground, s.texels_.data() + s.texel_index(0, r, col), layer_stride);
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

WeatherSnapshot WeatherSnapshot::from_parts(const WeatherSnapshotId& id, int rows, int columns,
                                            std::vector<float> ground_m, std::vector<std::uint8_t> texels) {
    require_grid(rows, columns);
    if (ground_m.size() != static_cast<std::size_t>(rows) * columns ||
        texels.size() != static_cast<std::size_t>(weather_layer_count) * rows * columns * channels) {
        throw std::invalid_argument("weather snapshot: parts do not match the grid");
    }
    WeatherSnapshot s(id, 2, 2);
    s.rows_ = rows;
    s.columns_ = columns;
    s.ground_m_ = std::move(ground_m);
    s.texels_ = std::move(texels);
    return s;
}

void WeatherSnapshot::make_poles_uniform() {
    // All columns of a pole row are one point: their mean (ground and every layer) so no longitude seam appears there.
    const auto& table = water_decode_table();
    for (int r : {0, rows_ - 1}) {
        double ground = 0.0;
        for (int col = 0; col < columns_; ++col) ground += ground_m_[static_cast<std::size_t>(r) * columns_ + col];
        ground /= columns_;
        for (int col = 0; col < columns_; ++col) ground_m_[static_cast<std::size_t>(r) * columns_ + col] = static_cast<float>(ground);
        for (int layer = 0; layer < weather_layer_count; ++layer) {
            double sum[3] = {0.0, 0.0, 0.0};
            for (int col = 0; col < columns_; ++col) {
                const std::size_t i = texel_index(layer, r, col);
                sum[0] += decode_weather_cloud_fraction(texels_[i]);
                sum[1] += table[texels_[i + 1]];
                sum[2] += table[texels_[i + 2]];
            }
            const std::uint8_t cf = encode_weather_cloud_fraction(sum[0] / columns_);
            const std::uint8_t lw = encode_weather_water(sum[1] / columns_);
            const std::uint8_t iw = encode_weather_water(sum[2] / columns_);
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
            double weight_sum = 0.0, ground = 0.0;
            std::array<std::array<double, 3>, weather_layer_count> sums{};
            for (int dr = -h; dr <= h; ++dr) {
                const int sr = r * factor + dr;
                if (sr < 0 || sr >= rows_) continue;
                for (int dc = -h; dc <= h; ++dc) {
                    const int sc = ((col * factor + dc) % columns_ + columns_) % columns_;
                    const double w = weight(dr) * weight(dc);
                    weight_sum += w;
                    ground += w * ground_m_[static_cast<std::size_t>(sr) * columns_ + sc];
                    for (int layer = 0; layer < weather_layer_count; ++layer) {
                        const std::size_t i = texel_index(layer, sr, sc);
                        sums[layer][0] += w * decode_weather_cloud_fraction(texels_[i]);
                        sums[layer][1] += w * table[texels_[i + 1]];
                        sums[layer][2] += w * table[texels_[i + 2]];
                    }
                }
            }
            out.ground_m_[static_cast<std::size_t>(r) * columns + col] = static_cast<float>(ground / weight_sum);
            for (int layer = 0; layer < weather_layer_count; ++layer) {
                const std::size_t i = out.texel_index(layer, r, col);
                out.texels_[i] = encode_weather_cloud_fraction(sums[layer][0] / weight_sum);
                out.texels_[i + 1] = encode_weather_water(sums[layer][1] / weight_sum);
                out.texels_[i + 2] = encode_weather_water(sums[layer][2] / weight_sum);
            }
        }
    });
    out.make_poles_uniform();
    return out;
}

}  // namespace starsim::environment
