// Weather source selection and request planning (M8 weather and clouds, Task 2). Fixtures: tests/validation/weather/
// (real ERA5 root .zattrs of 2026-10-25, real GFS .idx of 2026-09-20 00Z f006).

#include <cstdint>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/environment/weather_source.hpp"

using namespace starsim;
using namespace starsim::environment;

namespace {

const std::string fixture_dir = STARSIM_WEATHER_FIXTURE_DIR;

std::string read_text(const std::string& name) {
    std::ifstream in(fixture_dir + "/" + name, std::ios::binary);
    REQUIRE(in.good());
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::int64_t hour_of(int y, int mo, int d, int h) { return hours_since_1900({y, mo, d, h, 0, 0.0}); }

const std::optional<Era5Availability> availability = Era5Availability{hour_of(2026, 6, 30, 23), hour_of(2026, 10, 19, 23)};
const std::optional<GfsCycle> latest_cycle = GfsCycle{hour_of(2026, 10, 25, 0)};

}  // namespace

TEST_CASE("Hours since 1900 convert both ways", "[weather]") {
    CHECK(hour_of(1900, 1, 1, 0) == 0);
    CHECK(hour_of(1900, 1, 2, 0) == 24);
    CHECK(hour_of(1901, 1, 1, 0) == 365 * 24);
    CHECK(hour_of(1940, 1, 1, 0) == 350616);  // 40 years, 9 leap days
    for (std::int64_t h : {0LL, 350640LL, 1088136LL + 12345, 1330000LL}) {
        CHECK(hours_since_1900(utc_from_hours_since_1900(h)) == h);
    }
    const auto u = utc_from_hours_since_1900(hour_of(2024, 2, 29, 17));
    CHECK(u.year == 2024);
    CHECK(u.month == 2);
    CHECK(u.day == 29);
    CHECK(u.hour == 17);
}

TEST_CASE("Archive epochs use ERA5", "[weather]") {
    const std::int64_t h = hour_of(2024, 4, 8, 18);
    const auto b = weather_bracket(static_cast<double>(h) + 0.5, 1, availability, latest_cycle, 2025);
    CHECK(b.before.kind == WeatherSourceKind::era5);
    CHECK(b.after.kind == WeatherSourceKind::era5);
    CHECK(b.before.valid_hour == h);
    CHECK(b.after.valid_hour == h + 1);
    CHECK(b.blend == Catch::Approx(0.5));
    CHECK(b.before.cache_key() == "era5_" + std::to_string(h));

    const std::int64_t first = hour_of(1940, 1, 1, 0);
    const auto start = weather_bracket(static_cast<double>(first), 1, availability, latest_cycle, 2025);
    CHECK(start.before.kind == WeatherSourceKind::era5);
    CHECK(start.after.kind == WeatherSourceKind::era5);
    CHECK(start.blend == Catch::Approx(0.0));

    const auto early = weather_bracket(static_cast<double>(first - 100), 1, availability, latest_cycle, 2025);
    CHECK(early.before.kind == WeatherSourceKind::analog);
    CHECK(early.after.kind == WeatherSourceKind::analog);
    CHECK(early.before.analog_hour == hour_of(2025, 12, 27, 20));  // 1939-12-27 20:00 shown as 2025

    // Straddling the 1940 start: the day before is analog, the first hour ERA5, no exception.
    const auto seam = weather_bracket(static_cast<double>(first) - 0.5, 1, availability, latest_cycle, 2025);
    CHECK(seam.before.kind == WeatherSourceKind::analog);
    CHECK(seam.after.kind == WeatherSourceKind::era5);
}

TEST_CASE("Analog hours keep the calendar day and hour", "[weather]") {
    // 2028-02-29 06:00 -> 2025-02-28 06:00 (29 Feb exists only in leap years).
    const std::int64_t h = hour_of(2028, 2, 29, 6);
    const auto b = weather_bracket(static_cast<double>(h), 1, std::nullopt, std::nullopt, 2025);
    CHECK(b.before.kind == WeatherSourceKind::analog);
    CHECK(b.before.analog_hour == hour_of(2025, 2, 28, 6));
    CHECK(b.before.valid_hour == h);
    // A normal day keeps month / day / hour.
    const std::int64_t g = hour_of(2031, 7, 14, 13);
    const auto c = weather_bracket(static_cast<double>(g), 1, std::nullopt, std::nullopt, 2025);
    CHECK(c.before.analog_hour == hour_of(2025, 7, 14, 13));
    // The analog stands for the ERA5 hour of the reference year: same cache stem.
    CHECK(c.before.cache_key() == "era5_" + std::to_string(hour_of(2025, 7, 14, 13)));
}

TEST_CASE("The ERA5T end hands over to GFS without a gap", "[weather]") {
    const std::int64_t stop = availability->era5t_stop_hour;
    const auto at = weather_bracket(static_cast<double>(stop) + 0.5, 1, availability, latest_cycle, 2025);
    CHECK(at.before.kind == WeatherSourceKind::era5);
    CHECK(at.before.valid_hour == stop);
    CHECK(at.after.kind == WeatherSourceKind::gfs);
    CHECK(at.after.valid_hour == stop + 1);
    // The gap lies before the newest cycle: a covering cycle's analysis / shortest lead.
    CHECK(at.after.lead_h >= 0);
    CHECK(at.after.lead_h < 6);
    CHECK(at.after.cycle_hour + at.after.lead_h == at.after.valid_hour);
    CHECK(at.after.cycle_hour % 6 == 0);
    CHECK(at.blend == Catch::Approx(0.5));

    const auto exact = weather_bracket(static_cast<double>(stop), 1, availability, latest_cycle, 2025);
    CHECK(exact.before.kind == WeatherSourceKind::era5);
    CHECK(exact.after.kind == WeatherSourceKind::gfs);
    CHECK(exact.blend == Catch::Approx(0.0));

    const auto past = weather_bracket(static_cast<double>(stop) + 1.0, 1, availability, latest_cycle, 2025);
    CHECK(past.before.kind == WeatherSourceKind::gfs);
    CHECK(past.after.kind == WeatherSourceKind::gfs);

    // Without any GFS known the seam falls back to the analog, not to a gap.
    const auto none = weather_bracket(static_cast<double>(stop) + 0.5, 1, availability, std::nullopt, 2025);
    CHECK(none.before.kind == WeatherSourceKind::era5);
    CHECK(none.after.kind == WeatherSourceKind::analog);
}

TEST_CASE("Future epochs use the latest forecast, then the analog", "[weather]") {
    const std::int64_t cyc = latest_cycle->cycle_hour;
    const auto b54 = weather_bracket(static_cast<double>(cyc + 54), 1, availability, latest_cycle, 2025);
    CHECK(b54.before.kind == WeatherSourceKind::gfs);
    CHECK(b54.before.cycle_hour == cyc);
    CHECK(b54.before.lead_h == 54);
    CHECK(b54.before.cache_key() == "gfs_" + std::to_string(cyc) + "_f054");
    CHECK(b54.blend == Catch::Approx(0.0));

    const auto b200 = weather_bracket(static_cast<double>(cyc) + 200.5, 1, availability, latest_cycle, 2025);
    CHECK(b200.before.kind == WeatherSourceKind::gfs);
    CHECK(b200.after.kind == WeatherSourceKind::gfs);
    CHECK(b200.before.lead_h == 198);  // 3-hourly leads
    CHECK(b200.after.lead_h == 201);
    CHECK(b200.blend == Catch::Approx(2.5 / 3.0));

    // Up to the end of the hourly range the step stays as asked.
    const auto b119 = weather_bracket(static_cast<double>(cyc) + 119.5, 1, availability, latest_cycle, 2025);
    CHECK(b119.before.lead_h == 119);
    CHECK(b119.after.lead_h == 120);

    // Warp step 6 stays 6 h, aligned to the cycle.
    const auto b6 = weather_bracket(static_cast<double>(cyc) + 130.0, 6, availability, latest_cycle, 2025);
    CHECK(b6.before.lead_h == 126);
    CHECK(b6.after.lead_h == 132);

    // The last lead is still GFS, one hour beyond is the analog.
    const auto last = weather_bracket(static_cast<double>(cyc) + 383.5, 1, availability, latest_cycle, 2025);
    CHECK(last.before.kind == WeatherSourceKind::gfs);
    CHECK(last.before.lead_h == 381);
    CHECK(last.after.kind == WeatherSourceKind::gfs);
    CHECK(last.after.lead_h == 384);
    const auto seam = weather_bracket(static_cast<double>(cyc) + 385.0, 1, availability, latest_cycle, 2025);
    CHECK(seam.before.kind == WeatherSourceKind::gfs);  // lead 384 is the last one, the next hour is the analog
    CHECK(seam.after.kind == WeatherSourceKind::analog);
    const auto beyond = weather_bracket(static_cast<double>(cyc) + 388.0, 1, availability, latest_cycle, 2025);
    CHECK(beyond.before.kind == WeatherSourceKind::analog);
    CHECK(beyond.after.kind == WeatherSourceKind::analog);

    const auto b400 = weather_bracket(static_cast<double>(cyc) + 400.0, 1, availability, latest_cycle, 2025);
    CHECK(b400.before.kind == WeatherSourceKind::analog);
    CHECK(b400.after.kind == WeatherSourceKind::analog);
    const auto u = utc_from_hours_since_1900(b400.before.valid_hour);
    const auto a = utc_from_hours_since_1900(b400.before.analog_hour);
    CHECK(a.year == 2025);
    CHECK(a.month == u.month);
    CHECK(a.day == u.day);
    CHECK(a.hour == u.hour);
}

TEST_CASE("Offline means analog", "[weather]") {
    const auto b = weather_bracket(static_cast<double>(hour_of(2024, 4, 8, 18)) + 0.25, 1, std::nullopt, std::nullopt, 2025);
    CHECK(b.before.kind == WeatherSourceKind::analog);
    CHECK(b.after.kind == WeatherSourceKind::analog);
    CHECK(b.before.valid_hour + 1 == b.after.valid_hour);
    CHECK(b.blend == Catch::Approx(0.25));
    // No ERA5 availability but a GFS cycle: past epochs older than the cycle stay analog.
    const auto g = weather_bracket(static_cast<double>(hour_of(2024, 4, 8, 18)), 1, std::nullopt, latest_cycle, 2025);
    CHECK(g.before.kind == WeatherSourceKind::analog);
}

TEST_CASE("Warp steps align to hours since 1900", "[weather]") {
    const std::int64_t h = hour_of(2024, 4, 8, 18);  // 18:00 is a multiple of 6 h
    const auto b = weather_bracket(static_cast<double>(h) + 4.0, 3, availability, latest_cycle, 2025);
    CHECK(b.before.valid_hour == h + 3);
    CHECK(b.after.valid_hour == h + 6);
    CHECK(b.blend == Catch::Approx(1.0 / 3.0));
}

TEST_CASE("ERA5 requests list the ten files of a snapshot", "[weather]") {
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::era5;
    id.valid_hour = 1088136;
    const auto reqs = era5_requests(id);
    REQUIRE(reqs.size() == 10);
    const std::string base = "https://storage.googleapis.com/gcp-public-data-arco-era5/ar/full_37-1h-0p25deg-chunk-1.zarr-v3/";
    CHECK(reqs[0].url == base + "fraction_of_cloud_cover/1088136.0.0.0");
    CHECK(reqs[1].url == base + "specific_cloud_liquid_water_content/1088136.0.0.0");
    CHECK(reqs[2].url == base + "specific_cloud_ice_water_content/1088136.0.0.0");
    CHECK(reqs[3].url == base + "temperature/1088136.0.0.0");
    CHECK(reqs[4].url == base + "surface_pressure/1088136.0.0");
    CHECK(reqs[5].url == base + "total_column_water_vapour/1088136.0.0");
    CHECK(reqs[6].url == base + "geopotential_at_surface/1088136.0.0");
    CHECK(reqs[7].url == base + "2m_temperature/1088136.0.0");
    CHECK(reqs[8].url == base + "2m_dewpoint_temperature/1088136.0.0");
    CHECK(reqs[7].file_name == "era5_1088136_2m_temperature.blosc");
    CHECK(reqs[8].file_name == "era5_1088136_2m_dewpoint_temperature.blosc");
    // The single-level total cloud cover (the cover the snapshot's layers are scaled to).
    CHECK(reqs[9].url == base + "total_cloud_cover/1088136.0.0");
    CHECK(reqs[9].file_name == "era5_1088136_total_cloud_cover.blosc");
    // File names as the weather_snapshot CLI reads them; the static orography's name has no hour (one cached copy).
    CHECK(reqs[0].file_name == "era5_1088136_fraction_of_cloud_cover.blosc");
    CHECK(reqs[5].file_name == "era5_1088136_total_column_water_vapour.blosc");
    CHECK(reqs[6].file_name == era5_static_orography_file_name);
    CHECK(std::string(era5_static_orography_file_name) == "era5_geopotential_at_surface.blosc");
    std::set<std::string> names;
    for (const auto& r : reqs) {
        CHECK(r.first_byte == -1);
        CHECK(r.last_byte == -1);
        names.insert(r.file_name);
    }
    CHECK(names.size() == 10);
    WeatherSnapshotId other = id;
    other.valid_hour += 1;
    CHECK(era5_requests(other)[6].file_name == reqs[6].file_name);
    CHECK(era5_requests(other)[6].url == base + "geopotential_at_surface/1088137.0.0");

    // An analog requests its reference-year hour.
    WeatherSnapshotId analog;
    analog.kind = WeatherSourceKind::analog;
    analog.valid_hour = 1330000;
    analog.analog_hour = 1088136;
    CHECK(era5_requests(analog)[3].url == base + "temperature/1088136.0.0.0");
    WeatherSnapshotId gfs;
    gfs.kind = WeatherSourceKind::gfs;
    CHECK(era5_requests(gfs).empty());
}

TEST_CASE("GFS byte ranges come from the index", "[weather]") {
    const std::string idx = read_text("gfs_20260920_00_f006.idx");
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::gfs;
    id.cycle_hour = hour_of(2026, 9, 20, 0);
    id.lead_h = 6;
    id.valid_hour = id.cycle_hour + 6;
    CHECK(gfs_index_url(id) ==
          "https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.20260920/00/atmos/gfs.t00z.pgrb2.0p25.f006.idx");

    const auto reqs = gfs_requests(id, idx);
    REQUIRE(reqs.size() == 5 * 22 + 1 + 2 + 1);
    // File names as the weather_snapshot CLI reads them; the ground (HGT surface) comes last.
    CHECK(reqs[0].file_name == "gfs_1110792_f006_TCDC_1000mb.grb2");
    CHECK(reqs[21].file_name == "gfs_1110792_f006_TCDC_0050mb.grb2");
    const auto& ground = reqs[5 * 22];
    CHECK(ground.file_name == "gfs_1110792_f006_HGT_surface.grb2");
    CHECK(ground.first_byte == 412883802);  // line 562 HGT:surface
    CHECK(ground.last_byte == 413376245 - 1);        // line 563 starts at 413376245
    // Then the 2 m temperature and dew point (index lines 581 and 583; 582 SPFH and 584 RH are not wanted).
    CHECK(reqs[5 * 22 + 1].file_name == "gfs_1110792_f006_TMP_2m.grb2");
    CHECK(reqs[5 * 22 + 1].first_byte == 420490760);
    CHECK(reqs[5 * 22 + 1].last_byte == 420999562 - 1);
    CHECK(reqs[5 * 22 + 2].file_name == "gfs_1110792_f006_DPT_2m.grb2");
    CHECK(reqs[5 * 22 + 2].first_byte == 422220087);
    CHECK(reqs[5 * 22 + 2].last_byte == 422752522 - 1);
    // Then the total cloud cover, the instantaneous TCDC entire atmosphere (line 636; 637 is the 0-6 hour average).
    CHECK(reqs.back().file_name == "gfs_1110792_f006_TCDC_total.grb2");
    CHECK(reqs.back().first_byte == 449322989);
    CHECK(reqs.back().last_byte == 450159083 - 1);
    const std::string grib_url = "https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.20260920/00/atmos/gfs.t00z.pgrb2.0p25.f006";
    for (const auto& r : reqs) {
        CHECK(r.url == grib_url);
        CHECK(r.first_byte >= 0);
        CHECK(r.last_byte >= r.first_byte);
    }
    // CLMR 50 mb is line 205 at 167256433; the next record (206) starts where it ends.
    bool found = false;
    for (const auto& r : reqs) {
        if (r.first_byte == 167256433) {
            found = true;
            CHECK(r.file_name.find("CLMR") != std::string::npos);
            CHECK(r.file_name.find("0050") != std::string::npos);
            CHECK(r.last_byte >= 167256433 + 100);
        }
    }
    CHECK(found);
    std::set<std::string> names;
    for (const auto& r : reqs) names.insert(r.file_name);
    CHECK(names.size() == reqs.size());

    // The range ends one byte before the next record: consecutive lines of the index.
    const std::string next_line = "\n206:";
    const auto pos = idx.find(next_line);
    REQUIRE(pos != std::string::npos);
    const auto next_offset = std::stoll(idx.substr(idx.find(':', pos + 1) + 1));
    for (const auto& r : reqs) {
        if (r.first_byte == 167256433) CHECK(r.last_byte == next_offset - 1);
    }

    // A missing record (here: no CLMR 300 mb) throws.
    std::string damaged;
    {
        std::size_t start = 0;
        while (start < idx.size()) {
            const auto end = idx.find('\n', start);
            const std::string line = idx.substr(start, end - start);
            if (line.find(":CLMR:300 mb:") == std::string::npos) damaged += line + "\n";
            start = end == std::string::npos ? idx.size() : end + 1;
        }
    }
    CHECK_THROWS_AS(gfs_requests(id, damaged), std::runtime_error);
    CHECK_THROWS_AS(gfs_requests(id, ""), std::runtime_error);
}

TEST_CASE("GFS analysis leads use anl in the index", "[weather]") {
    const char* variables[] = {"TCDC", "CLMR", "ICMR", "HGT", "TMP"};
    const int levels[] = {1000, 975, 950, 925, 900, 850, 800, 750, 700, 650, 600, 550,
                          500,  450, 400, 350, 300, 250, 200, 150, 100, 50};
    std::string idx;
    long long offset = 0;
    int n = 1;
    for (const char* v : variables) {
        for (int l : levels) {
            idx += std::to_string(n++) + ":" + std::to_string(offset) + ":d=2026092000:" + v + ":" + std::to_string(l) +
                   " mb:anl:\n";
            offset += 1000;
        }
    }
    idx += std::to_string(n++) + ":" + std::to_string(offset) + ":d=2026092000:HGT:surface:anl:\n";
    offset += 1000;
    idx += std::to_string(n++) + ":" + std::to_string(offset) + ":d=2026092000:TMP:2 m above ground:anl:\n";
    offset += 1000;
    idx += std::to_string(n++) + ":" + std::to_string(offset) + ":d=2026092000:DPT:2 m above ground:anl:\n";
    offset += 1000;
    idx += std::to_string(n++) + ":" + std::to_string(offset) + ":d=2026092000:TCDC:entire atmosphere:anl:\n";
    WeatherSnapshotId id;
    id.kind = WeatherSourceKind::gfs;
    id.cycle_hour = hour_of(2026, 9, 20, 0);
    id.lead_h = 0;
    id.valid_hour = id.cycle_hour;
    const auto reqs = gfs_requests(id, idx);
    REQUIRE(reqs.size() == 114);
    CHECK(reqs[0].first_byte == 0);
    CHECK(reqs[0].last_byte == 999);
    CHECK(reqs.back().last_byte == -1);  // the file's last record: open end
    CHECK(reqs[0].url.find(".f000") != std::string::npos);
    // Without the HGT surface record (the ground), a 2 m record or the total cloud cover the snapshot cannot be built:
    // throws.
    CHECK_THROWS_AS(gfs_requests(id, idx.substr(0, idx.rfind(std::to_string(n - 1) + ":"))), std::runtime_error);
    CHECK_THROWS_AS(gfs_requests(id, idx.substr(0, idx.rfind(std::to_string(n - 2) + ":"))), std::runtime_error);
    CHECK_THROWS_AS(gfs_requests(id, idx.substr(0, idx.rfind(std::to_string(n - 3) + ":"))), std::runtime_error);
    id.lead_h = 6;  // there are no "6 hour fcst" lines in this index
    CHECK_THROWS_AS(gfs_requests(id, idx), std::runtime_error);
}

TEST_CASE("Cycle candidates step back in 6 h", "[weather]") {
    const double wall = static_cast<double>(hour_of(2026, 10, 25, 3)) + 10.0 / 60.0;
    const auto c = gfs_cycle_candidates(wall, 4);
    REQUIRE(c.size() == 4);
    CHECK(c[0].cycle_hour == hour_of(2026, 10, 25, 0));
    CHECK(c[1].cycle_hour == hour_of(2026, 10, 24, 18));
    CHECK(c[2].cycle_hour == hour_of(2026, 10, 24, 12));
    CHECK(c[3].cycle_hour == hour_of(2026, 10, 24, 6));
    CHECK(gfs_cycle_candidates(wall, 0).empty());
}

TEST_CASE("Warp policy", "[weather]") {
    // Download 20 s per snapshot: a step qualifies when its interval in wall time is >= 2 x 20 s.
    CHECK(weather_snapshot_step(1.0, 20.0) == 1);
    CHECK(weather_snapshot_step(60.0, 20.0) == 1);    // 60 s per hour
    CHECK(weather_snapshot_step(90.0, 20.0) == 1);    // exactly 40 s per hour
    CHECK(weather_snapshot_step(200.0, 20.0) == 3);   // 18 s per hour, 54 s per 3 h
    CHECK(weather_snapshot_step(400.0, 20.0) == 6);   // 27 s per 3 h, 54 s per 6 h
    CHECK(weather_snapshot_step(1.0e5, 20.0) == 0);   // hold
    CHECK(weather_snapshot_step(100.0, 0.0) == 1);    // nothing measured yet
    // Monotone: a higher warp never gives a smaller step, hold counts as the largest.
    int previous = 1;
    for (double warp = 1.0; warp < 1.0e6; warp *= 1.5) {
        int step = weather_snapshot_step(warp, 20.0);
        int rank = step == 0 ? 7 : step;
        CHECK(rank >= previous);
        previous = rank;
    }
}

TEST_CASE("ERA5 .zattrs parse", "[weather]") {
    const auto a = parse_era5_zattrs(read_text("era5_root_zattrs.json"));
    CHECK(a.final_stop_hour == hour_of(2026, 6, 30, 23));
    CHECK(a.era5t_stop_hour == hour_of(2026, 10, 19, 23));
    CHECK_THROWS_AS(parse_era5_zattrs("{\"valid_time_stop\": \"2026-06-30\"}"), std::runtime_error);
    CHECK_THROWS_AS(parse_era5_zattrs("{\"valid_time_stop_era5t\": \"2026-10-19\"}"), std::runtime_error);
    CHECK_THROWS_AS(parse_era5_zattrs("{\"valid_time_stop\": \"soon\", \"valid_time_stop_era5t\": \"2026-10-19\"}"),
                    std::runtime_error);
    CHECK_THROWS_AS(parse_era5_zattrs(""), std::runtime_error);
}
