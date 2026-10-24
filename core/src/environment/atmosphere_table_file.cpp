// Binary cache of the precomputed atmosphere tables (M8 atmosphere); layout in atmosphere_table_file.hpp.

#include "starsim/environment/atmosphere_table_file.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "starsim/config/atmosphere_optics_config.hpp"

namespace starsim::environment {

namespace {

constexpr char magic[8] = {'S', 'T', 'A', 'R', 'A', 'T', 'M', 'O'};
constexpr int parameter_count = 15;

std::uint64_t fnv1a(const char* data, std::size_t size) {
    std::uint64_t h = 14695981039346656037ull;  // FNV-1a 64 offset basis
    for (std::size_t i = 0; i < size; ++i) {
        h ^= static_cast<unsigned char>(data[i]);
        h *= 1099511628211ull;  // FNV-1a 64 prime
    }
    return h;
}

std::array<int, 8> size_fields(const AtmosphereTableSizes& s) {
    return {s.transmittance_mu, s.transmittance_r, s.scattering_r,    s.scattering_mu,
            s.scattering_mu_s,  s.scattering_nu,   s.irradiance_mu_s, s.irradiance_r};
}

AtmosphereTableSizes sizes_from_fields(const std::array<int, 8>& f) {
    AtmosphereTableSizes s;
    s.transmittance_mu = f[0];
    s.transmittance_r = f[1];
    s.scattering_r = f[2];
    s.scattering_mu = f[3];
    s.scattering_mu_s = f[4];
    s.scattering_nu = f[5];
    s.irradiance_mu_s = f[6];
    s.irradiance_r = f[7];
    return s;
}

std::array<double, parameter_count> parameter_fields(const AtmosphereParameters& p) {
    return {p.bottom_radius_m,
            p.top_radius_m,
            p.mu_s_min,
            p.mie_phase_g,
            p.sun_angular_radius_rad,
            p.solar_illuminance_rgb[0],
            p.solar_illuminance_rgb[1],
            p.solar_illuminance_rgb[2],
            p.sky_radiance_to_luminance[0],
            p.sky_radiance_to_luminance[1],
            p.sky_radiance_to_luminance[2],
            p.sun_radiance_to_luminance[0],
            p.sun_radiance_to_luminance[1],
            p.sun_radiance_to_luminance[2],
            p.toa_solar_illuminance_lux};
}

AtmosphereParameters parameters_from_fields(const std::array<double, parameter_count>& f) {
    AtmosphereParameters p;
    p.bottom_radius_m = f[0];
    p.top_radius_m = f[1];
    p.mu_s_min = f[2];
    p.mie_phase_g = f[3];
    p.sun_angular_radius_rad = f[4];
    p.solar_illuminance_rgb = {f[5], f[6], f[7]};
    p.sky_radiance_to_luminance = {f[8], f[9], f[10]};
    p.sun_radiance_to_luminance = {f[11], f[12], f[13]};
    p.toa_solar_illuminance_lux = f[14];
    return p;
}

// The float counts of the four tables for the sizes (RGBA texels).
std::array<std::uint64_t, 4> expected_counts(const AtmosphereTableSizes& s) {
    const std::uint64_t scattering = static_cast<std::uint64_t>(s.scattering_width()) * s.scattering_height() *
                                     s.scattering_depth() * 4;
    return {static_cast<std::uint64_t>(s.transmittance_mu) * s.transmittance_r * 4, scattering, scattering,
            static_cast<std::uint64_t>(s.irradiance_mu_s) * s.irradiance_r * 4};
}

template <class T>
void append(std::vector<char>& out, const T& value) {
    const auto* p = reinterpret_cast<const char*>(&value);
    out.insert(out.end(), p, p + sizeof(T));
}

// Reads sequentially from a byte buffer; every read checks the remaining length.
struct Reader {
    const std::vector<char>& bytes;
    std::size_t at = 0;
    template <class T>
    bool read(T& value) {
        if (bytes.size() - at < sizeof(T)) return false;
        std::memcpy(&value, bytes.data() + at, sizeof(T));
        at += sizeof(T);
        return true;
    }
    bool read_floats(std::vector<float>& out, std::uint64_t count) {
        if ((bytes.size() - at) / sizeof(float) < count) return false;
        out.resize(static_cast<std::size_t>(count));
        std::memcpy(out.data(), bytes.data() + at, static_cast<std::size_t>(count) * sizeof(float));
        at += static_cast<std::size_t>(count) * sizeof(float);
        return true;
    }
};

// A temp file name next to `file`, unique across processes and threads.
std::filesystem::path unique_temp_path(const std::filesystem::path& file) {
    static std::atomic<unsigned> counter{0};
    std::random_device rd;
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto thread = std::hash<std::thread::id>{}(std::this_thread::get_id());
    return file.parent_path() / (file.filename().string() + ".tmp." + std::to_string(stamp) + "." +
                                 std::to_string(rd()) + "." + std::to_string(thread) + "." +
                                 std::to_string(counter++));
}

constexpr int rename_attempts = 5;  // [call] back-off 50, 100, 150, 200 ms

// Temp files of writers killed between write and rename (84 MB each): siblings "<name>.tmp.*" older than 10 minutes
// (a live writer needs well under a second between creating and renaming its file).
void remove_stale_temp_files(const std::filesystem::path& file) {
    const std::string prefix = file.filename().string() + ".tmp.";
    const auto now = std::filesystem::file_time_type::clock::now();
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(file.parent_path(), ec)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind(prefix, 0) != 0) continue;
        std::error_code entry_error;
        const auto written = entry.last_write_time(entry_error);
        if (!entry_error && now - written > std::chrono::minutes(10)) std::filesystem::remove(entry.path(), entry_error);
    }
}

}  // namespace

void write_atmosphere_tables(const std::filesystem::path& file, const AtmosphereTables& tables) {
    const auto counts = expected_counts(tables.sizes);
    const std::array<const std::vector<float>*, 4> arrays{&tables.transmittance, &tables.scattering,
                                                          &tables.single_mie, &tables.irradiance};
    for (int i = 0; i < 4; ++i) {
        if (arrays[i]->size() != counts[i]) throw std::invalid_argument("atmosphere tables do not match their sizes");
    }
    std::vector<char> bytes;
    std::size_t total = 8 + 4 + 4 + 8 + 8 * 4 + parameter_count * 8 + 4 * 8 + 8;
    for (const auto c : counts) total += static_cast<std::size_t>(c) * sizeof(float);
    bytes.reserve(total);
    bytes.insert(bytes.end(), magic, magic + sizeof(magic));
    append(bytes, atmosphere_table_file_version);
    append(bytes, std::uint32_t{0});
    append(bytes, tables.input_hash);
    for (const std::int32_t v : size_fields(tables.sizes)) append(bytes, v);
    for (const double v : parameter_fields(tables.parameters)) append(bytes, v);
    for (const auto c : counts) append(bytes, c);
    for (const auto* a : arrays) {
        const auto* p = reinterpret_cast<const char*>(a->data());
        bytes.insert(bytes.end(), p, p + a->size() * sizeof(float));
    }
    append(bytes, fnv1a(bytes.data(), bytes.size()));

    const std::filesystem::path temp = unique_temp_path(file);
    {
        std::ofstream out(temp, std::ios::binary | std::ios::trunc);
        if (!out) throw std::runtime_error("cannot write " + temp.string());
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        out.close();
        if (!out) {
            std::error_code ec;
            std::filesystem::remove(temp, ec);
            throw std::runtime_error("cannot write " + temp.string());
        }
    }
    // Retried with a short back-off: on Windows the rename fails transiently while a virus scanner holds the fresh
    // temp file or another process has the target open without delete sharing.
    std::error_code ec;
    for (int attempt = 0; attempt < rename_attempts; ++attempt) {
        if (attempt > 0) std::this_thread::sleep_for(std::chrono::milliseconds(50 * attempt));
        std::filesystem::rename(temp, file, ec);
        if (!ec) return;
    }
    std::error_code ignored;
    std::filesystem::remove(temp, ignored);
    throw std::runtime_error("cannot rename " + temp.string() + " to " + file.string() + ": " + ec.message());
}

std::optional<AtmosphereTables> read_atmosphere_tables(const std::filesystem::path& file, std::uint64_t expected_hash) {
    std::error_code size_error;
    const auto file_size = std::filesystem::file_size(file, size_error);
    if (size_error || file_size < sizeof(magic) + sizeof(std::uint64_t)) return std::nullopt;
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::nullopt;
    std::vector<char> bytes(static_cast<std::size_t>(file_size));
    if (!in.read(bytes.data(), static_cast<std::streamsize>(bytes.size())) || in.peek() != EOF) return std::nullopt;

    // The checksum first: nothing of a damaged file is interpreted.
    const std::size_t payload = bytes.size() - sizeof(std::uint64_t);
    std::uint64_t stored_checksum = 0;
    std::memcpy(&stored_checksum, bytes.data() + payload, sizeof(stored_checksum));
    if (fnv1a(bytes.data(), payload) != stored_checksum) return std::nullopt;

    Reader r{bytes};
    char file_magic[8]{};
    std::uint32_t version = 0, reserved = 0;
    std::uint64_t hash = 0;
    if (!r.read(file_magic) || std::memcmp(file_magic, magic, sizeof(magic)) != 0) return std::nullopt;
    if (!r.read(version) || version != atmosphere_table_file_version || !r.read(reserved)) return std::nullopt;
    if (!r.read(hash) || hash != expected_hash) return std::nullopt;
    std::array<std::int32_t, 8> size_values{};
    for (auto& v : size_values) {
        if (!r.read(v) || v < 2 || v > (1 << 16)) return std::nullopt;
    }
    std::array<double, parameter_count> parameter_values{};
    for (auto& v : parameter_values) {
        if (!r.read(v)) return std::nullopt;
    }
    AtmosphereTables tables;
    tables.sizes = sizes_from_fields(size_values);
    tables.parameters = parameters_from_fields(parameter_values);
    tables.input_hash = hash;
    const auto expected = expected_counts(tables.sizes);
    std::array<std::uint64_t, 4> counts{};
    for (int i = 0; i < 4; ++i) {
        if (!r.read(counts[i]) || counts[i] != expected[i]) return std::nullopt;
    }
    if (!r.read_floats(tables.transmittance, counts[0]) || !r.read_floats(tables.scattering, counts[1]) ||
        !r.read_floats(tables.single_mie, counts[2]) || !r.read_floats(tables.irradiance, counts[3])) {
        return std::nullopt;
    }
    if (r.at != payload) return std::nullopt;
    return tables;
}

std::shared_ptr<const AtmosphereTables> load_or_compute_atmosphere(const std::filesystem::path& config_dir,
                                                                   const std::filesystem::path& cache_dir,
                                                                   const AtmosphereTableSizes& sizes) {
    const auto optics = config::load_atmosphere_optics(config_dir / "atmosphere" / "earth_optics.toml");
    const std::uint64_t hash = atmosphere_input_hash(optics, sizes);
    const std::filesystem::path file = cache_dir / atmosphere_cache_file_name;
    if (auto cached = read_atmosphere_tables(file, hash)) {
        return std::make_shared<const AtmosphereTables>(std::move(*cached));
    }
    auto tables = std::make_shared<const AtmosphereTables>(precompute_atmosphere(optics, sizes));
    // The cache is only an optimisation: a failed write (read-only or invalid directory, disk full, a rename blocked by
    // another process) is reported and the freshly computed tables are returned (as sim/ascent_plan_cache.cpp).
    try {
        std::filesystem::create_directories(cache_dir);
        remove_stale_temp_files(file);
        write_atmosphere_tables(file, *tables);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "atmosphere: cache not written (%s)\n", e.what());
    }
    return tables;
}

}  // namespace starsim::environment
