#pragma once

// Weather source selection and request planning (M8 weather and clouds). Pure functions, no networking: the game
// downloads what these functions describe.
//
// Time convention: valid times are whole hours since 1900-01-01 00:00 UTC. This is the ERA5 Zarr `time` index, verified
// once (2026-10-25) against the ARCO-ERA5 store: `time/.zarray` is int64, chunk 67706, blosc lz4 clevel 5 byte shuffle;
// `time/.zattrs` says "hours since 1900-01-01 00:00:00" (proleptic Gregorian); the first chunk decodes to 0, 1, 2, ...
// 67705, i.e. index = hours since 1900-01-01 (offset 0), so the chunk key `<variable>/<t>.0.0.0` uses t directly.
//
// Source by epoch: before 1940-01-01 -> analog; up to the end of ERA5T -> ERA5; then the newest GFS cycle known to
// exist (analysis / short leads of the covering cycle for the gap, forecast leads beyond) up to +384 h; beyond that, or
// when nothing is known (offline) -> analog (the same calendar day and hour in a reference year).

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "starsim/time/utc_date_time.hpp"

namespace starsim::environment {

enum class WeatherSourceKind { era5, gfs, analog, off };

// Last available ERA5 hours, from the store's root .zattrs. A stop date means the last hour of that day (23:00).
struct Era5Availability {
    std::int64_t final_stop_hour = 0, era5t_stop_hour = 0;
};

struct GfsCycle {
    std::int64_t cycle_hour = 0;  // 00 / 06 / 12 / 18 UTC run time
};

struct WeatherSnapshotId {
    WeatherSourceKind kind = WeatherSourceKind::off;
    std::int64_t valid_hour = 0;     // the snapshot's valid time
    std::int64_t cycle_hour = 0;     // gfs
    int lead_h = 0;                  // gfs
    std::int64_t analog_hour = 0;    // analog: the reference-year hour shown (its data is the ERA5 hour of that year)
    // Unique file stem: "era5_<hour>", "gfs_<cycle>_f<lead>", an analog shares the ERA5 stem of its reference hour.
    std::string cache_key() const;
};

struct WeatherRequest {
    std::string url;
    std::int64_t first_byte = -1, last_byte = -1;  // -1: the whole object / open end
    std::string file_name;
};

struct WeatherBracket {
    WeatherSnapshotId before, after;
    double blend = 0.0;  // 0 = before, 1 = after
};

// First ERA5 hour (1940-01-01 00:00), GFS horizon, hourly / 3-hourly limits.
constexpr int gfs_hourly_limit_h = 120;
constexpr int gfs_max_lead_h = 384;

// The ARCO-ERA5 store's 37 pressure levels (hPa) in the store's order, ascending pressure: index k of a 4D chunk's
// level axis is era5_levels_hpa[k] (the store's `level` array; pinned against it by the weather snapshot test via
// tests/validation/weather/era5_columns.toml).
inline constexpr std::array<int, 37> era5_levels_hpa{1,   2,   3,   5,   7,   10,  20,  30,  50,  70,  100, 125, 150,
                                                     175, 200, 225, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700,
                                                     750, 775, 800, 825, 850, 875, 900, 925, 950, 975, 1000};
// Pressure levels (mb) of the GFS 0.25 deg pgrb2 files from 1000 to 50 mb that all five variables share (CLMR / ICMR /
// TCDC have no 70 mb level, which the cloud layers up to 20 km do not need), in request order.
inline constexpr std::array<int, 22> gfs_levels_mb{1000, 975, 950, 925, 900, 850, 800, 750, 700, 650, 600,
                                                   550,  500, 450, 400, 350, 300, 250, 200, 150, 100, 50};

// The two snapshots bracketing utc_hour_f (fractional hours since 1900) for the warp step (1, 3, 6 h), the blend factor
// between them, and the source each one comes from. latest_gfs is the newest cycle known to exist (nullopt: offline /
// unknown). Beyond the last GFS lead or offline: analog. Beyond 120 h of the latest cycle the GFS leads are 3-hourly, so
// the step is at least 3 h there.
WeatherBracket weather_bracket(double utc_hour_f, int step_h, const std::optional<Era5Availability>& era5,
                               const std::optional<GfsCycle>& latest_gfs, int analog_reference_year);

// Raw file names, shared by the request planning and the weather_snapshot CLI that reads them:
// ERA5 "<key>_<variable>.blosc"; the static orography (geopotential_at_surface) has one name for every hour, so a cache
// keyed by file name downloads it once; GFS "<key>_<VAR>_<llll>mb.grb2" and the ground "<key>_HGT_surface.grb2".
inline constexpr const char* era5_static_orography_file_name = "era5_geopotential_at_surface.blosc";
std::string era5_file_name(const std::string& cache_key, std::string_view variable);
std::string gfs_level_file_name(const std::string& cache_key, std::string_view variable, int level_mb);
std::string gfs_surface_height_file_name(const std::string& cache_key);
// "<key>_TMP_2m.grb2" / "<key>_DPT_2m.grb2": the 2 m above ground temperature and dew point.
std::string gfs_2m_file_name(const std::string& cache_key, std::string_view variable);

// Requests for one snapshot: ERA5 gives nine files - six timed (fraction_of_cloud_cover,
// specific_cloud_liquid_water_content, specific_cloud_ice_water_content, temperature, surface_pressure,
// total_column_water_vapour), the static geopotential_at_surface (URL with the same t, file name without it; the
// caller skips it if cached) and the timed 2m_temperature, 2m_dewpoint_temperature (lifting condensation level, 4.8 MB
// together); an analog snapshot requests its reference-year hour.
std::vector<WeatherRequest> era5_requests(const WeatherSnapshotId&);

std::string gfs_index_url(const WeatherSnapshotId&);
// One byte range per wanted record (TCDC, CLMR, ICMR, HGT, TMP on 1000..50 mb, then HGT surface, then TMP and DPT at
// 2 m above ground, 1.0 MB together), the last byte of a
// record is the byte before the next record's offset. Throws std::runtime_error when a wanted record is missing.
std::vector<WeatherRequest> gfs_requests(const WeatherSnapshotId&, std::string_view idx_text);

// Candidate cycles for probing which one is on the server, newest first, starting at the cycle at or before the wall clock.
std::vector<GfsCycle> gfs_cycle_candidates(double wall_clock_utc_hour_f, int count);

// Throws std::runtime_error on a missing or malformed key.
Era5Availability parse_era5_zattrs(std::string_view json);

// Warp policy: the snapshot step (1, 3, 6 h) or 0 = hold, from the warp factor and the measured wall seconds per
// snapshot download. The smallest step whose snapshot interval in wall time (step * 3600 / warp) is at least
// snapshot_interval_margin times the download time; none qualifies -> hold.
constexpr double snapshot_interval_margin = 2.0;
int weather_snapshot_step(double warp_factor, double seconds_per_snapshot_download);

// Whole hours since 1900-01-01 00:00 UTC (floor); proleptic Gregorian.
std::int64_t hours_since_1900(const time::UtcDateTime&);
time::UtcDateTime utc_from_hours_since_1900(std::int64_t hours);

}  // namespace starsim::environment
