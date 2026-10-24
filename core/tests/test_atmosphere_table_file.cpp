// Atmosphere table cache (M8 atmosphere, Task 3): the binary file round-trips, damaged or stale files are never read
// but recomputed.

#include <atomic>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/atmosphere_optics_config.hpp"
#include "starsim/environment/atmosphere_table_file.hpp"
#include "starsim/environment/atmosphere_tables.hpp"

using namespace starsim;
namespace fs = std::filesystem;

namespace {

const fs::path config_dir = STARSIM_CONFIG_DIR;

// Small tables: the file code does not care about the resolution.
environment::AtmosphereTableSizes small_sizes() {
    environment::AtmosphereTableSizes s;
    s.transmittance_mu = 32;
    s.transmittance_r = 16;
    s.scattering_r = 4;
    s.scattering_mu = 8;
    s.scattering_mu_s = 4;
    s.scattering_nu = 2;
    s.irradiance_mu_s = 8;
    s.irradiance_r = 4;
    return s;
}

// A unique, empty temp directory per call (test cases run as parallel processes).
struct TempDir {
    fs::path path;
    TempDir() {
        std::random_device rd;
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() /
               ("starsim_atmosphere_" + std::to_string(stamp) + "_" + std::to_string(rd()) + "_" + std::to_string(rd()));
        fs::create_directories(path);
    }
    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }
};

std::vector<char> read_bytes(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void write_bytes(const fs::path& file, const std::vector<char>& bytes) {
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

bool same_tables(const environment::AtmosphereTables& a, const environment::AtmosphereTables& b) {
    const auto& p = a.parameters;
    const auto& q = b.parameters;
    return a.input_hash == b.input_hash && a.sizes.transmittance_mu == b.sizes.transmittance_mu &&
           a.sizes.transmittance_r == b.sizes.transmittance_r && a.sizes.scattering_r == b.sizes.scattering_r &&
           a.sizes.scattering_mu == b.sizes.scattering_mu && a.sizes.scattering_mu_s == b.sizes.scattering_mu_s &&
           a.sizes.scattering_nu == b.sizes.scattering_nu && a.sizes.irradiance_mu_s == b.sizes.irradiance_mu_s &&
           a.sizes.irradiance_r == b.sizes.irradiance_r && p.bottom_radius_m == q.bottom_radius_m &&
           p.top_radius_m == q.top_radius_m && p.mu_s_min == q.mu_s_min && p.mie_phase_g == q.mie_phase_g &&
           p.sun_angular_radius_rad == q.sun_angular_radius_rad && p.solar_illuminance_rgb == q.solar_illuminance_rgb &&
           p.sky_radiance_to_luminance == q.sky_radiance_to_luminance &&
           p.sun_radiance_to_luminance == q.sun_radiance_to_luminance &&
           p.toa_solar_illuminance_lux == q.toa_solar_illuminance_lux && a.transmittance == b.transmittance &&
           a.scattering == b.scattering && a.single_mie == b.single_mie && a.irradiance == b.irradiance;
}

}  // namespace

TEST_CASE("Table files round-trip and damaged files are recomputed", "[atmosphere]") {
    const auto optics = config::load_atmosphere_optics(config_dir / "atmosphere" / "earth_optics.toml");
    const auto sizes = small_sizes();
    const auto tables = environment::precompute_atmosphere(optics, sizes);
    REQUIRE(tables.input_hash == environment::atmosphere_input_hash(optics, sizes));
    TempDir dir;

    // Round trip.
    const fs::path file = dir.path / "tables.bin";
    environment::write_atmosphere_tables(file, tables);
    const auto back = environment::read_atmosphere_tables(file, tables.input_hash);
    REQUIRE(back.has_value());
    CHECK(same_tables(*back, tables));
    // No temp file left behind.
    int entries = 0;
    for (const auto& e : fs::directory_iterator(dir.path)) {
        (void)e;
        ++entries;
    }
    CHECK(entries == 1);

    // A different expected hash (stale inputs), a missing file, a truncated copy and flipped bytes: nullopt.
    CHECK_FALSE(environment::read_atmosphere_tables(file, tables.input_hash + 1).has_value());
    CHECK_FALSE(environment::read_atmosphere_tables(dir.path / "missing.bin", tables.input_hash).has_value());
    const auto bytes = read_bytes(file);
    {
        auto truncated = bytes;
        truncated.resize(bytes.size() - 100);
        write_bytes(dir.path / "truncated.bin", truncated);
        CHECK_FALSE(environment::read_atmosphere_tables(dir.path / "truncated.bin", tables.input_hash).has_value());
        write_bytes(dir.path / "header_only.bin", std::vector<char>(bytes.begin(), bytes.begin() + 20));
        CHECK_FALSE(environment::read_atmosphere_tables(dir.path / "header_only.bin", tables.input_hash).has_value());
    }
    for (const std::size_t at : {std::size_t{3}, std::size_t{40}, bytes.size() / 2, bytes.size() - 3}) {
        auto flipped = bytes;
        flipped[at] = static_cast<char>(flipped[at] ^ 0x10);
        write_bytes(dir.path / "flipped.bin", flipped);
        INFO("flipped byte " << at << " of " << bytes.size());
        CHECK_FALSE(environment::read_atmosphere_tables(dir.path / "flipped.bin", tables.input_hash).has_value());
    }

    // load_or_compute_atmosphere: an empty cache directory gets the file; a second call reads it; a damaged file is
    // recomputed and replaced; other sizes (a changed hash) recompute.
    TempDir cache;
    const fs::path cache_file = cache.path / environment::atmosphere_cache_file_name;
    const auto computed = environment::load_or_compute_atmosphere(config_dir, cache.path, sizes);
    REQUIRE(computed);
    CHECK(fs::exists(cache_file));
    CHECK(same_tables(*computed, tables));
    const auto loaded = environment::load_or_compute_atmosphere(config_dir, cache.path, sizes);
    CHECK(same_tables(*loaded, tables));

    auto damaged = read_bytes(cache_file);
    damaged[damaged.size() / 3] = static_cast<char>(damaged[damaged.size() / 3] ^ 0x01);
    write_bytes(cache_file, damaged);
    REQUIRE_FALSE(environment::read_atmosphere_tables(cache_file, tables.input_hash).has_value());
    const auto repaired = environment::load_or_compute_atmosphere(config_dir, cache.path, sizes);
    CHECK(same_tables(*repaired, tables));
    CHECK(environment::read_atmosphere_tables(cache_file, tables.input_hash).has_value());

    auto other_sizes = sizes;
    other_sizes.irradiance_r = 6;
    const auto recomputed = environment::load_or_compute_atmosphere(config_dir, cache.path, other_sizes);
    CHECK(recomputed->input_hash == environment::atmosphere_input_hash(optics, other_sizes));
    CHECK(recomputed->sizes.irradiance_r == 6);
    CHECK(environment::read_atmosphere_tables(cache_file, recomputed->input_hash).has_value());
}
