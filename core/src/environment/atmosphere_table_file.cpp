// Binary cache of the precomputed atmosphere tables (M8 atmosphere); layout in atmosphere_table_file.hpp.

#include "starsim/environment/atmosphere_table_file.hpp"

#include <array>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "cache_file_io.hpp"
#include "starsim/config/atmosphere_optics_config.hpp"

namespace starsim::environment {

namespace {

constexpr char magic[8] = {'S', 'T', 'A', 'R', 'A', 'T', 'M', 'O'};
constexpr int parameter_count = 16;

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
            p.toa_solar_illuminance_lux,
            p.ground_albedo};
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
    p.ground_albedo = f[15];
    return p;
}

// The float counts of the five tables for the sizes (RGBA texels).
std::array<std::uint64_t, 5> expected_counts(const AtmosphereTableSizes& s) {
    const std::uint64_t scattering = static_cast<std::uint64_t>(s.scattering_width()) * s.scattering_height() *
                                     s.scattering_depth() * 4;
    return {static_cast<std::uint64_t>(s.transmittance_mu) * s.transmittance_r * 4, scattering, scattering, scattering,
            static_cast<std::uint64_t>(s.irradiance_mu_s) * s.irradiance_r * 4};
}

}  // namespace

void write_atmosphere_tables(const std::filesystem::path& file, const AtmosphereTables& tables) {
    const auto counts = expected_counts(tables.sizes);
    const std::array<const std::vector<float>*, 5> arrays{&tables.transmittance, &tables.scattering,
                                                          &tables.single_mie, &tables.single_rayleigh,
                                                          &tables.irradiance};
    for (int i = 0; i < 5; ++i) {
        if (arrays[i]->size() != counts[i]) throw std::invalid_argument("atmosphere tables do not match their sizes");
    }
    std::vector<char> bytes;
    std::size_t total = 8 + 4 + 4 + 8 + 8 * 4 + parameter_count * 8 + 5 * 8 + 8;
    for (const auto c : counts) total += static_cast<std::size_t>(c) * sizeof(float);
    bytes.reserve(total);
    bytes.insert(bytes.end(), magic, magic + sizeof(magic));
    detail::append_bytes(bytes, atmosphere_table_file_version);
    detail::append_bytes(bytes, std::uint32_t{0});
    detail::append_bytes(bytes, tables.input_hash);
    for (const std::int32_t v : size_fields(tables.sizes)) detail::append_bytes(bytes, v);
    for (const double v : parameter_fields(tables.parameters)) detail::append_bytes(bytes, v);
    for (const auto c : counts) detail::append_bytes(bytes, c);
    for (const auto* a : arrays) {
        const auto* p = reinterpret_cast<const char*>(a->data());
        bytes.insert(bytes.end(), p, p + a->size() * sizeof(float));
    }
    detail::append_bytes(bytes, detail::fnv1a64(bytes.data(), bytes.size()));

    detail::write_file_atomically(file, bytes);
}

std::optional<AtmosphereTables> read_atmosphere_tables(const std::filesystem::path& file, std::uint64_t expected_hash) {
    const auto read = detail::read_whole_file(file);
    if (!read || read->size() < sizeof(magic) + sizeof(std::uint64_t)) return std::nullopt;
    const std::vector<char>& bytes = *read;

    // The checksum first: nothing of a damaged file is interpreted.
    const std::size_t payload = bytes.size() - sizeof(std::uint64_t);
    std::uint64_t stored_checksum = 0;
    std::memcpy(&stored_checksum, bytes.data() + payload, sizeof(stored_checksum));
    if (detail::fnv1a64(bytes.data(), payload) != stored_checksum) return std::nullopt;

    detail::ByteReader r{bytes};
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
    std::array<std::uint64_t, 5> counts{};
    for (int i = 0; i < 5; ++i) {
        if (!r.read(counts[i]) || counts[i] != expected[i]) return std::nullopt;
    }
    if (!r.read_array(tables.transmittance, counts[0]) || !r.read_array(tables.scattering, counts[1]) ||
        !r.read_array(tables.single_mie, counts[2]) || !r.read_array(tables.single_rayleigh, counts[3]) ||
        !r.read_array(tables.irradiance, counts[4])) {
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
        // Temp files of killed writers (84 MB each): siblings "<name>.tmp.*".
        detail::remove_stale_temp_files(cache_dir, file.filename().string() + ".tmp.");
        write_atmosphere_tables(file, *tables);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "atmosphere: cache not written (%s)\n", e.what());
    }
    return tables;
}

}  // namespace starsim::environment
