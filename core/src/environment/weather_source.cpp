#include "starsim/environment/weather_source.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>

namespace starsim::environment {

namespace {

constexpr const char* era5_base_url =
    "https://storage.googleapis.com/gcp-public-data-arco-era5/ar/full_37-1h-0p25deg-chunk-1.zarr-v3/";
constexpr const char* gfs_base_url = "https://noaa-gfs-bdp-pds.s3.amazonaws.com/";

// ERA5 starts 1940-01-01 00:00 (the store's valid_time_start); earlier epochs have no data.
constexpr int era5_first_year = 1940;

// ERA5 files per snapshot hour: key suffix is the chunk index of the array's dimensionality (4D: t.0.0.0, 3D: t.0.0).
// The static orography is the same at every t: its file name has no hour (era5_static_orography_file_name).
struct Era5Variable {
    const char* name;
    bool four_dimensional;
    bool time_independent;
};
constexpr std::array<Era5Variable, 7> era5_variables{{
    {"fraction_of_cloud_cover", true, false},
    {"specific_cloud_liquid_water_content", true, false},
    {"specific_cloud_ice_water_content", true, false},
    {"temperature", true, false},
    {"surface_pressure", false, false},
    {"total_column_water_vapour", false, false},
    {"geopotential_at_surface", false, true},
}};

constexpr std::array<const char*, 5> gfs_variables{"TCDC", "CLMR", "ICMR", "HGT", "TMP"};
// Pressure levels (mb) of the 0.25 deg pgrb2 files from 1000 to 50 mb that all five variables share (CLMR / ICMR / TCDC
// have no 70 mb level, which the cloud layers up to 20 km do not need).
constexpr std::array<int, 22> gfs_levels_mb{1000, 975, 950, 925, 900, 850, 800, 750, 700, 650, 600,
                                            550,  500, 450, 400, 350, 300, 250, 200, 150, 100, 50};

// Howard Hinnant's days_from_civil / civil_from_days (proleptic Gregorian), days since 1970-01-01.
std::int64_t days_from_civil(std::int64_t y, int m, int d) {
    y -= m <= 2;
    const std::int64_t era = (y >= 0 ? y : y - 399) / 400;
    const auto yoe = static_cast<unsigned>(y - era * 400);
    const unsigned doy = (153u * static_cast<unsigned>(m + (m > 2 ? -3 : 9)) + 2u) / 5u + static_cast<unsigned>(d) - 1u;
    const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097 + static_cast<std::int64_t>(doe) - 719468;
}

void civil_from_days(std::int64_t z, int& year, int& month, int& day) {
    z += 719468;
    const std::int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    const auto doe = static_cast<unsigned>(z - era * 146097);
    const unsigned yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
    const std::int64_t y = static_cast<std::int64_t>(yoe) + era * 400;
    const unsigned doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
    const unsigned mp = (5u * doy + 2u) / 153u;
    day = static_cast<int>(doy - (153u * mp + 2u) / 5u + 1u);
    month = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
    year = static_cast<int>(y + (month <= 2));
}

const std::int64_t days_1970_to_1900 = -days_from_civil(1900, 1, 1);

std::int64_t hours_from_ymdh(std::int64_t y, int m, int d, int h) {
    return (days_from_civil(y, m, d) + days_1970_to_1900) * 24 + h;
}

std::int64_t floor_div(std::int64_t a, std::int64_t b) {
    std::int64_t q = a / b;
    if ((a % b != 0) && ((a < 0) != (b < 0))) --q;
    return q;
}

bool is_leap(int y) { return (y % 4 == 0 && y % 100 != 0) || y % 400 == 0; }

WeatherSnapshotId analog_snapshot(std::int64_t valid_hour, int reference_year) {
    const time::UtcDateTime u = utc_from_hours_since_1900(valid_hour);
    int day = u.day;
    if (u.month == 2 && day == 29 && !is_leap(reference_year)) day = 28;
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::analog;
    id.valid_hour = valid_hour;
    id.analog_hour = hours_from_ymdh(reference_year, u.month, day, u.hour);
    return id;
}

WeatherSnapshotId gfs_snapshot(std::int64_t valid_hour, std::int64_t cycle_hour) {
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::gfs;
    id.valid_hour = valid_hour;
    id.cycle_hour = cycle_hour;
    id.lead_h = static_cast<int>(valid_hour - cycle_hour);
    return id;
}

// The source of one snapshot hour. No gap: before 1940 analog, up to the ERA5T end ERA5, then GFS - the analysis /
// short leads of the covering cycle while the hour lies before the newest cycle, the newest cycle's forecast after.
WeatherSnapshotId snapshot_at(std::int64_t hour, const std::optional<Era5Availability>& era5,
                              const std::optional<GfsCycle>& latest_gfs, int analog_reference_year) {
    if (hour < hours_from_ymdh(era5_first_year, 1, 1, 0)) return analog_snapshot(hour, analog_reference_year);
    if (era5 && hour <= era5->era5t_stop_hour) {
        WeatherSnapshotId id;
        id.kind = WeatherSourceKind::era5;
        id.valid_hour = hour;
        return id;
    }
    if (latest_gfs) {
        const std::int64_t latest = latest_gfs->cycle_hour;
        if (hour >= latest) {
            if (hour - latest <= gfs_max_lead_h) return gfs_snapshot(hour, latest);
        } else if (era5) {
            return gfs_snapshot(hour, floor_div(hour, 6) * 6);  // the gap after the ERA5T end
        }
    }
    return analog_snapshot(hour, analog_reference_year);
}

std::string two_digits(int v) {
    char buf[8];
    std::snprintf(buf, sizeof buf, "%02d", v);
    return buf;
}

std::string gfs_file_url(const WeatherSnapshotId& id) {
    const time::UtcDateTime c = utc_from_hours_since_1900(id.cycle_hour);
    char buf[32];
    std::snprintf(buf, sizeof buf, "%04d%02d%02d", c.year, c.month, c.day);
    char lead[8];
    std::snprintf(lead, sizeof lead, "%03d", id.lead_h);
    const std::string hh = two_digits(c.hour);
    return std::string(gfs_base_url) + "gfs." + buf + "/" + hh + "/atmos/gfs.t" + hh + "z.pgrb2.0p25.f" + lead;
}

std::string gfs_cycle_stamp(std::int64_t cycle_hour) {  // "YYYYMMDDHH" as in the index's d= field
    const time::UtcDateTime c = utc_from_hours_since_1900(cycle_hour);
    char buf[16];
    std::snprintf(buf, sizeof buf, "%04d%02d%02d%02d", c.year, c.month, c.day, c.hour);
    return buf;
}

[[noreturn]] void zattrs_error(const std::string& what) { throw std::runtime_error("ERA5 .zattrs: " + what); }

// Strict extraction of one string value `"key": "value"` (no escapes); the closing quote of the key is part of the match.
std::string zattrs_string(std::string_view json, std::string_view key) {
    const std::string quoted = "\"" + std::string(key) + "\"";
    const auto at = json.find(quoted);
    if (at == std::string_view::npos) zattrs_error("missing key " + std::string(key));
    std::size_t p = at + quoted.size();
    auto skip_space = [&] {
        while (p < json.size() && (json[p] == ' ' || json[p] == '\t' || json[p] == '\n' || json[p] == '\r')) ++p;
    };
    skip_space();
    if (p >= json.size() || json[p] != ':') zattrs_error("no value for " + std::string(key));
    ++p;
    skip_space();
    if (p >= json.size() || json[p] != '"') zattrs_error("value of " + std::string(key) + " is not a string");
    ++p;
    const auto end = json.find('"', p);
    if (end == std::string_view::npos) zattrs_error("unterminated value of " + std::string(key));
    return std::string(json.substr(p, end - p));
}

// "YYYY-MM-DD" -> the last hour (23:00) of that day.
std::int64_t last_hour_of_day(const std::string& date, std::string_view key) {
    bool ok = date.size() == 10 && date[4] == '-' && date[7] == '-';
    for (std::size_t i = 0; ok && i < date.size(); ++i) {
        if (i != 4 && i != 7 && (date[i] < '0' || date[i] > '9')) ok = false;
    }
    const int y = ok ? std::stoi(date.substr(0, 4)) : 0;
    const int m = ok ? std::stoi(date.substr(5, 2)) : 0;
    const int d = ok ? std::stoi(date.substr(8, 2)) : 0;
    if (!ok || m < 1 || m > 12 || d < 1 || d > 31) zattrs_error("bad date in " + std::string(key) + ": " + date);
    return hours_from_ymdh(y, m, d, 23);
}

struct IndexRecord {
    std::int64_t offset = 0;
    std::string stamp, variable, level, forecast;
};

std::vector<IndexRecord> parse_gfs_index(std::string_view text) {
    std::vector<IndexRecord> records;
    std::size_t start = 0;
    while (start < text.size()) {
        auto end = text.find('\n', start);
        if (end == std::string_view::npos) end = text.size();
        std::string_view line = text.substr(start, end - start);
        start = end + 1;
        if (!line.empty() && line.back() == '\r') line.remove_suffix(1);
        if (line.empty()) continue;
        std::array<std::string, 6> f;
        std::size_t field = 0, from = 0;
        while (field < f.size()) {
            const auto colon = line.find(':', from);
            if (colon == std::string_view::npos) {
                f[field++] = std::string(line.substr(from));
                break;
            }
            f[field++] = std::string(line.substr(from, colon - from));
            from = colon + 1;
        }
        if (field < 6 || f[2].rfind("d=", 0) != 0) throw std::runtime_error("GFS index: malformed line: " + std::string(line));
        IndexRecord r;
        try {
            r.offset = std::stoll(f[1]);
        } catch (const std::exception&) {
            throw std::runtime_error("GFS index: bad byte offset: " + std::string(line));
        }
        r.stamp = f[2].substr(2);
        r.variable = f[3];
        r.level = f[4];
        r.forecast = f[5];
        records.push_back(std::move(r));
    }
    return records;
}

}  // namespace

std::string WeatherSnapshotId::cache_key() const {
    switch (kind) {
        case WeatherSourceKind::era5:
            return "era5_" + std::to_string(valid_hour);
        case WeatherSourceKind::analog:
            return "era5_" + std::to_string(analog_hour);  // the data is the ERA5 hour of the reference year
        case WeatherSourceKind::gfs: {
            char lead[8];
            std::snprintf(lead, sizeof lead, "%03d", lead_h);
            return "gfs_" + std::to_string(cycle_hour) + "_f" + lead;
        }
        case WeatherSourceKind::off:
            break;
    }
    return "off";
}

std::int64_t hours_since_1900(const time::UtcDateTime& t) { return hours_from_ymdh(t.year, t.month, t.day, t.hour); }

time::UtcDateTime utc_from_hours_since_1900(std::int64_t hours) {
    const std::int64_t days = floor_div(hours, 24);
    time::UtcDateTime u;
    civil_from_days(days - days_1970_to_1900, u.year, u.month, u.day);
    u.hour = static_cast<int>(hours - days * 24);
    u.minute = 0;
    u.second = 0.0;
    return u;
}

WeatherBracket weather_bracket(double utc_hour_f, int step_h, const std::optional<Era5Availability>& era5,
                               const std::optional<GfsCycle>& latest_gfs, int analog_reference_year) {
    int step = std::max(1, step_h);
    // Beyond 120 h of the newest cycle the forecast leads are 3-hourly.
    if (latest_gfs && utc_hour_f >= static_cast<double>(latest_gfs->cycle_hour + gfs_hourly_limit_h)) {
        step = std::max(step, 3);
    }
    const auto before_hour = static_cast<std::int64_t>(std::floor(utc_hour_f / step)) * step;
    WeatherBracket b;
    b.before = snapshot_at(before_hour, era5, latest_gfs, analog_reference_year);
    b.after = snapshot_at(before_hour + step, era5, latest_gfs, analog_reference_year);
    b.blend = std::clamp((utc_hour_f - static_cast<double>(before_hour)) / step, 0.0, 1.0);
    return b;
}

std::vector<WeatherRequest> era5_requests(const WeatherSnapshotId& id) {
    std::vector<WeatherRequest> out;
    if (id.kind != WeatherSourceKind::era5 && id.kind != WeatherSourceKind::analog) return out;
    const std::int64_t t = id.kind == WeatherSourceKind::era5 ? id.valid_hour : id.analog_hour;
    const std::string stem = "era5_" + std::to_string(t);
    for (const auto& v : era5_variables) {
        WeatherRequest r;
        r.url = std::string(era5_base_url) + v.name + "/" + std::to_string(t) + (v.four_dimensional ? ".0.0.0" : ".0.0");
        r.file_name = v.time_independent ? std::string(era5_static_orography_file_name) : era5_file_name(stem, v.name);
        out.push_back(std::move(r));
    }
    return out;
}

std::string era5_file_name(const std::string& cache_key, std::string_view variable) {
    return cache_key + "_" + std::string(variable) + ".blosc";
}

std::string gfs_level_file_name(const std::string& cache_key, std::string_view variable, int level_mb) {
    char lvl[8];
    std::snprintf(lvl, sizeof lvl, "%04d", level_mb);
    return cache_key + "_" + std::string(variable) + "_" + lvl + "mb.grb2";
}

std::string gfs_surface_height_file_name(const std::string& cache_key) { return cache_key + "_HGT_surface.grb2"; }

std::string gfs_index_url(const WeatherSnapshotId& id) { return gfs_file_url(id) + ".idx"; }

std::vector<WeatherRequest> gfs_requests(const WeatherSnapshotId& id, std::string_view idx_text) {
    const std::vector<IndexRecord> records = parse_gfs_index(idx_text);
    const std::string stamp = gfs_cycle_stamp(id.cycle_hour);
    const std::string forecast = id.lead_h == 0 ? "anl" : std::to_string(id.lead_h) + " hour fcst";
    const std::string url = gfs_file_url(id);

    std::map<std::pair<std::string, int>, std::size_t> wanted;  // (variable, level) -> record position
    std::optional<std::size_t> surface_height;                  // HGT surface: the ground
    for (std::size_t i = 0; i < records.size(); ++i) {
        const IndexRecord& r = records[i];
        if (r.stamp != stamp || r.forecast != forecast) continue;
        if (r.variable == "HGT" && r.level == "surface" && !surface_height) surface_height = i;
        if (std::find(gfs_variables.begin(), gfs_variables.end(), r.variable) == gfs_variables.end()) continue;
        // Pressure levels read "<N> mb"; "surface", "1 hybrid level" and the like are not wanted.
        const auto space = r.level.find(' ');
        if (space == std::string::npos || r.level.substr(space) != " mb" || space == 0 || space > 5) continue;
        int level = 0;
        for (std::size_t k = 0; k < space; ++k) {
            if (r.level[k] < '0' || r.level[k] > '9') level = -1;
            if (level >= 0) level = level * 10 + (r.level[k] - '0');
        }
        if (level <= 0) continue;
        wanted.emplace(std::make_pair(r.variable, level), i);  // first one wins
    }

    std::vector<WeatherRequest> out;
    const auto add = [&](std::size_t i, std::string file_name) {
        WeatherRequest r;
        r.url = url;
        r.first_byte = records[i].offset;
        r.last_byte = i + 1 < records.size() ? records[i + 1].offset - 1 : -1;  // the file's last record: open end
        if (r.last_byte != -1 && r.last_byte < r.first_byte) throw std::runtime_error("GFS index: offsets not increasing");
        r.file_name = std::move(file_name);
        out.push_back(std::move(r));
    };
    const std::string key = id.cache_key();
    for (const char* variable : gfs_variables) {
        for (int level : gfs_levels_mb) {
            const auto it = wanted.find({variable, level});
            if (it == wanted.end()) {
                throw std::runtime_error(std::string("GFS index: record ") + variable + " " + std::to_string(level) +
                                         " mb missing for " + url);
            }
            add(it->second, gfs_level_file_name(key, variable, level));
        }
    }
    if (!surface_height) throw std::runtime_error("GFS index: record HGT surface missing for " + url);
    add(*surface_height, gfs_surface_height_file_name(key));
    return out;
}

std::vector<GfsCycle> gfs_cycle_candidates(double wall_clock_utc_hour_f, int count) {
    std::vector<GfsCycle> out;
    const std::int64_t newest = floor_div(static_cast<std::int64_t>(std::floor(wall_clock_utc_hour_f)), 6) * 6;
    for (int i = 0; i < count; ++i) out.push_back(GfsCycle{newest - 6 * i});
    return out;
}

Era5Availability parse_era5_zattrs(std::string_view json) {
    Era5Availability a;
    a.final_stop_hour = last_hour_of_day(zattrs_string(json, "valid_time_stop"), "valid_time_stop");
    a.era5t_stop_hour = last_hour_of_day(zattrs_string(json, "valid_time_stop_era5t"), "valid_time_stop_era5t");
    return a;
}

int weather_snapshot_step(double warp_factor, double seconds_per_snapshot_download) {
    if (!(warp_factor > 0.0) || !(seconds_per_snapshot_download > 0.0)) return 1;
    for (int step : {1, 3, 6}) {
        if (step * 3600.0 / warp_factor >= snapshot_interval_margin * seconds_per_snapshot_download) return step;
    }
    return 0;
}

}  // namespace starsim::environment
