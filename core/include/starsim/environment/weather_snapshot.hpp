#pragma once

// One weather snapshot on ellipsoidal height layers (M8 weather and clouds): cloud fraction, cloud liquid and ice water
// content as grid means on the 32 render layers, built from decoded ERA5 or GFS pressure-level fields.
//
// Grid: `rows` latitude rows from +90 deg (row 0) to -90 deg (last row) inclusive, `columns` longitude columns east from
// 0 deg (wrapping), the ERA5 / GFS 0.25 deg grid (721 x 1440) or a coarser one (downsampled analog set).
//
// Height layers (render and cache form): 32 layers above the WGS84 ellipsoid, 16 x 250 m from 0 to 4 km, then
// 16 x 1 km from 4 to 20 km. Layer i spans the layer coordinate [i, i + 1), its value is the mean over the layer's
// above-ground part and belongs to its centre (coordinate i + 0.5).
//
// Physics of the build (per column):
// - ERA5: pressure-level heights by integrating the hypsometric equation upward from the surface (surface pressure +
//   surface geopotential) with the virtual temperature Tv = T (1 + (R_v / R_d - 1) q). Dry air misses ERA5's
//   geopotential by up to 40 m in the tropics; the per-level humidity is not downloaded (103 MB per hour) but
//   distributed from the single-level total column water vapour W with an exponential vapour-density profile of scale
//   height H_v (water_vapour_scale_height_m): q = q_s (p / p_s)^(a - 1), a = H_p / H_v, H_p = R_d T_1 / g0 with T_1 the
//   temperature of the first level above the surface, q_s = W g0 a / p_s (so that W = int q dp / g0). Max error 9.5 m
//   over the six fixture columns (test "Hypsometric heights match ERA5 geopotential"). Between levels the temperature
//   is linear in ln p; from the surface to the first level above it the standard lapse rate.
// - GFS: the HGT field directly; the ground from the surface HGT record.
// - Geopotential height -> geometric height with the WGS84 normal gravity and the matching effective radius
//   (geometric_height_m), + the geoid undulation -> height above the ellipsoid.
// - kg/kg -> kg/m^3 with the air density: ERA5's specific contents are per kg of moist air, so rho = p / (R_d T_v)
//   with the virtual temperature of the vapour profile above; GFS has no humidity input and stays dry,
//   rho = p / (R_d T) (moist air is less dense: the dry form overestimates by 0.61 q, <= ~1.5 % near a tropical
//   surface, less aloft).
// - Lifting condensation level per column from the 2 m temperature and dew point (Romps 2017, exact; ERA5 with its
//   surface pressure, GFS with the standard atmosphere's at the column's ground height), m above the ground, 0 where the
//   surface air is saturated (T - Td < cloud_fog_spread_k), weather_lcl_unknown_m where unknown, capped at 20 km; the
//   downsampled set and the pole rows average the known values like the ground height. The column's boundary-layer
//   liquid below it is moved up (raise_liquid_cloud_to_lcl).
// - Levels at or below the surface are ignored; the lowest level above the ground is held down to the ground, the
//   highest one held upward; layer parts below the ground do not count, at() is zero below the ground.
//
// Render encoding (RGBA8 per cell; R cloud fraction, G liquid, B ice, A unused = 0), shared with the shader:
// - cloud fraction: code = round(255 f), f = code / 255.
// - water content: code 0 = none; code c in 1..255 = min * (max / min)^((c - 1) / 254), min 2e-7 and max 4e-3 kg/m^3
//   (weather_water_code_min/max_kg_m3), logarithmic; relative error <= 1.97 % over that range (half a code step).
//   Below about 2e-7 kg/m^3 a layer mean reads 0 (liquid: optical depth < 0.03 per km at 10 um); above 4e-3 it clamps
//   (grid means of ERA5 / GFS stay below).

#include <cstdint>
#include <span>
#include <vector>

#include "starsim/environment/geoid.hpp"
#include "starsim/environment/weather_source.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/weather_decode/weather_decode.hpp"

namespace starsim::environment {

inline constexpr int weather_layer_count = 32;
inline constexpr int weather_fine_layer_count = 16;
inline constexpr double weather_fine_layer_thickness_m = 250.0;
inline constexpr double weather_coarse_layer_thickness_m = 1000.0;
inline constexpr double weather_top_m = 20000.0;

// Bottom of layer 0..32 (32: the top, 20 km) above the ellipsoid.
double weather_layer_bottom_m(int layer);
// Continuous layer coordinate of a height (layer i spans [i, i + 1)), clamped to [0, 32]; continuous and monotone.
double weather_layer_coordinate(double height_ellipsoid_m);

inline constexpr double weather_water_code_min_kg_m3 = 2e-7;
inline constexpr double weather_water_code_max_kg_m3 = 4e-3;
std::uint8_t encode_weather_water(double kg_m3);
double decode_weather_water(std::uint8_t code);
std::uint8_t encode_weather_cloud_fraction(double fraction);
double decode_weather_cloud_fraction(std::uint8_t code);

// Geopotential heights (gpm) of pressure levels above the surface, integrated upward from the surface. levels_pa in any
// order, temperature_k parallel to it; total_column_water_vapour_kg_m2 0 = dry air (profile above). Levels at or below
// the surface pressure (or with a non-finite temperature) are NaN.
std::vector<double> hypsometric_level_heights_gpm(std::span<const double> levels_pa,
                                                  std::span<const double> temperature_k,
                                                  double total_column_water_vapour_kg_m2, double surface_pressure_pa,
                                                  double surface_geopotential_m2_s2);

// The virtual temperatures of the same integration (NaN at or below the surface): the ERA5 build's density input.
std::vector<double> level_virtual_temperatures_k(std::span<const double> levels_pa,
                                                 std::span<const double> temperature_k,
                                                 double total_column_water_vapour_kg_m2, double surface_pressure_pa);

// Geometric height above the geoid of a geopotential height: Z g0 = gamma(phi) R(phi) h / (R(phi) + h) solved for h,
// with gamma(phi) the WGS84 normal gravity (Somigliana, NIMA TR8350.2 eq. 4-1) and R(phi) = a / (1 + f + m - 2 f sin^2
// phi) the radius whose inverse-square decay has the WGS84 free-air gradient (TR8350.2 eq. 4-3; the "effective Earth
// radius" of List 1968, Smithsonian Meteorological Tables; Mahoney 2001, "A discussion of various measures of altitude").
double geometric_height_m(double geopotential_height_m, double latitude_rad);

// Cloud water content (kg/m^3) of a specific content (kg/kg): q p / (R_d T); pass the virtual temperature for moist
// air (ERA5), the temperature for dry air (GFS).
double cloud_water_density_kg_m3(double specific_kg_kg, double pressure_pa, double temperature_k);

// Lifting condensation level (m above the 2 m level) of a parcel at the temperature t2m_k and dew point td2m_k lifted
// dry-adiabatically from the pressure pressure_pa: Romps 2017's exact Lambert-W (branch -1) expression, with the
// parcel's own moist heat capacity and gas constant (the exact form costs one W evaluation per column, ~100 ns).
// Td >= T (saturated) gives 0; a non-finite or non-positive input gives NaN. The pressure matters little (84 kPa
// instead of 101 kPa moves a 10 K spread by 3 m): pass the surface pressure when known, else the default.
double lifting_condensation_level_m(double t2m_k, double td2m_k, double pressure_pa = constants::standard_pressure_pa);

// Pressure of the standard atmosphere at a height above sea level (troposphere): p0 (1 - L h / T0)^(g / (R L)).
double standard_pressure_at_height_pa(double height_m);

// Stored for a column whose lifting condensation level is not known (no 2 m fields, invalid input). 0 is a KNOWN level:
// the surface air is saturated (fog spread, cloud_fog_spread_k) and cloud may reach the ground.
inline constexpr float weather_lcl_unknown_m = -1.0f;

// Cloud bases at the lifting condensation level (static clouds, spec decisions 1-2), on the means of the 32 layers of one
// column (each over the layer's above-ground part): the lowest liquid cloud whose base lies less than
// cloud_boundary_layer_top_m above the ground is a boundary-layer cloud whose base is the LCL; condensate below it is moved
// up. The liquid water of the cut part is spread over the part of that cloud above the LCL in proportion to its liquid
// (the column's liquid water path is kept; the cloud fraction there is at least the mass-weighted fraction of the cut
// part, so the covered area does not shrink); a cloud that lies wholly under the LCL is lifted to start there, keeping
// depth and mean density. A base above the LCL, an LCL <= 0 (saturated surface: fog) or >= the boundary-layer top, and
// ice are untouched. Cloud fraction of a cut layer with ice stays. cloud_fraction, liquid_kg_m3 and ice_kg_m3 have one
// entry per layer.
void raise_liquid_cloud_to_lcl(std::span<double> cloud_fraction, std::span<double> liquid_kg_m3,
                               std::span<const double> ice_kg_m3, double ground_m, double lcl_m);

struct WeatherCell {
    double cloud_fraction = 0.0, liquid_water_kg_m3 = 0.0, ice_water_kg_m3 = 0.0;  // grid means
};

// ERA5 fields of one hour: the 4D grids have levels_pa.size() levels (any level order), the surface grids one level.
struct Era5Fields {
    weather_decode::FloatGrid cloud_fraction, liquid_kg_kg, ice_kg_kg, temperature_k, surface_pressure_pa,
        surface_geopotential_m2_s2, total_column_water_vapour_kg_m2;
    // 2 m temperature and dew point (one level each); both empty = no lifting condensation level (unknown).
    weather_decode::FloatGrid temperature_2m_k, dewpoint_2m_k;
    std::vector<double> levels_pa;
};

// GFS fields of one cycle and lead: TCDC (percent), CLMR, ICMR, TMP, HGT on levels_pa; the ground from HGT:surface.
struct GfsFields {
    weather_decode::FloatGrid cloud_percent, liquid_kg_kg, ice_kg_kg, temperature_k, geopotential_height_m,
        surface_geopotential_height_m;
    // TMP / DPT 2 m above ground (one level each); both empty = no lifting condensation level (unknown).
    weather_decode::FloatGrid temperature_2m_k, dewpoint_2m_k;
    std::vector<double> levels_pa;
};

class WeatherSnapshot {
public:
    // Throw std::invalid_argument when the grids do not share one shape.
    static WeatherSnapshot from_era5(const WeatherSnapshotId& id, const Era5Fields& fields, const Geoid& geoid);
    static WeatherSnapshot from_gfs(const WeatherSnapshotId& id, const GfsFields& fields, const Geoid& geoid);
    // Synthetic test field: one uniform cloud slab everywhere (layer means of the slab's overlap), ground at 0 m;
    // the id is the default one (kind off). lcl_m: the lifting condensation level stored for every column (the texels are
    // the slab as given, not redistributed: a check of the renderer's flat base).
    static WeatherSnapshot uniform_slab(int rows, int columns, double bottom_m, double top_m, double liquid_water_kg_m3,
                                        double cloud_fraction, float lcl_m = weather_lcl_unknown_m);
    // From the cache form (weather_snapshot_file): ground rows x columns, texels as render_texels(). Throws
    // std::invalid_argument on a size mismatch.
    // lcl_m empty = all unknown.
    static WeatherSnapshot from_parts(const WeatherSnapshotId& id, int rows, int columns, std::vector<float> ground_m,
                                      std::vector<std::uint8_t> texels, std::vector<float> lcl_m = {});

    const WeatherSnapshotId& id() const { return id_; }
    int rows() const { return rows_; }
    int columns() const { return columns_; }

    // Trilinear in (lat, lon, layer coordinate), longitude wraps, zero above 20 km and below the ground.
    WeatherCell at(double latitude_rad, double longitude_rad, double height_ellipsoid_m) const;
    // Ellipsoidal ground height, bilinear.
    double ground_height_m(double latitude_rad, double longitude_rad) const;

    // Render form: 32 layers x rows x columns RGBA8 (encoding above), layer-major so the game builds one Image per layer.
    std::vector<std::uint8_t> render_texels() const { return texels_; }
    const std::vector<std::uint8_t>& texels() const { return texels_; }
    const std::vector<float>& ground_heights_m() const { return ground_m_; }

    // Lifting condensation level of a column (m above the ground; 0 = saturated surface air, weather_lcl_unknown_m =
    // unknown), row * columns() + column. The 2 m parcel's condensation height from the data's 2 m temperature and dew
    // point; the base of convective cloud (the build has already moved the column's boundary-layer condensate above it).
    float lcl_m(int row, int column) const { return lcl_m_[static_cast<std::size_t>(row) * columns_ + column]; }
    const std::vector<float>& lcl_heights_m() const { return lcl_m_; }

    // Box mean over factor x factor cells (analog set, factor 4: 1 deg); needs (rows - 1) and columns divisible by it.
    WeatherSnapshot downsampled(int factor) const;

private:
    WeatherSnapshot(const WeatherSnapshotId& id, int rows, int columns);
    std::size_t texel_index(int layer, int row, int column) const;
    void make_poles_uniform();

    WeatherSnapshotId id_;
    int rows_ = 0, columns_ = 0;
    std::vector<float> ground_m_;        // rows x columns, ellipsoidal
    std::vector<float> lcl_m_;           // rows x columns, above the ground
    std::vector<std::uint8_t> texels_;   // layers x rows x columns x 4
};

}  // namespace starsim::environment
