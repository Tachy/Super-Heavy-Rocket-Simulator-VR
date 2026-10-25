// Cache file of one weather snapshot (M8 weather and clouds); layout in weather_snapshot_file.hpp.

#include "starsim/environment/weather_snapshot_file.hpp"

#include <cstring>
#include <utility>
#include <vector>

#include "cache_file_io.hpp"

namespace starsim::environment {

namespace {

constexpr char magic[8] = {'S', 'T', 'A', 'R', 'W', 'T', 'H', 'R'};
constexpr std::int32_t texel_channels = 4;
constexpr std::int32_t max_grid_size = 1 << 14;  // a sanity bound on rows / columns before allocating

}  // namespace

void write_weather_snapshot(const std::filesystem::path& file, const WeatherSnapshot& snapshot) {
    const auto& ground = snapshot.ground_heights_m();
    const auto& texels = snapshot.texels();
    const WeatherSnapshotId& id = snapshot.id();
    std::vector<char> bytes;
    bytes.reserve(96 + ground.size() * sizeof(float) + texels.size());
    bytes.insert(bytes.end(), magic, magic + sizeof(magic));
    detail::append_bytes(bytes, weather_snapshot_file_version);
    detail::append_bytes(bytes, std::uint32_t{0});
    detail::append_bytes(bytes, static_cast<std::int32_t>(id.kind));
    detail::append_bytes(bytes, static_cast<std::int32_t>(id.lead_h));
    detail::append_bytes(bytes, static_cast<std::int64_t>(id.valid_hour));
    detail::append_bytes(bytes, static_cast<std::int64_t>(id.cycle_hour));
    detail::append_bytes(bytes, static_cast<std::int64_t>(id.analog_hour));
    detail::append_bytes(bytes, static_cast<std::int32_t>(snapshot.rows()));
    detail::append_bytes(bytes, static_cast<std::int32_t>(snapshot.columns()));
    detail::append_bytes(bytes, static_cast<std::int32_t>(weather_layer_count));
    detail::append_bytes(bytes, texel_channels);
    const auto* g = reinterpret_cast<const char*>(ground.data());
    bytes.insert(bytes.end(), g, g + ground.size() * sizeof(float));
    const auto* t = reinterpret_cast<const char*>(texels.data());
    bytes.insert(bytes.end(), t, t + texels.size());
    detail::append_bytes(bytes, detail::fnv1a64(bytes.data(), bytes.size()));
    detail::write_file_atomically(file, bytes);
}

std::optional<WeatherSnapshot> read_weather_snapshot(const std::filesystem::path& file) {
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
    if (!r.read(file_magic) || std::memcmp(file_magic, magic, sizeof(magic)) != 0) return std::nullopt;
    if (!r.read(version) || version != weather_snapshot_file_version || !r.read(reserved)) return std::nullopt;
    std::int32_t kind = 0, lead_h = 0, rows = 0, columns = 0, layers = 0, channels = 0;
    std::int64_t valid_hour = 0, cycle_hour = 0, analog_hour = 0;
    if (!r.read(kind) || !r.read(lead_h) || !r.read(valid_hour) || !r.read(cycle_hour) || !r.read(analog_hour) ||
        !r.read(rows) || !r.read(columns) || !r.read(layers) || !r.read(channels)) {
        return std::nullopt;
    }
    if (kind < static_cast<std::int32_t>(WeatherSourceKind::era5) || kind > static_cast<std::int32_t>(WeatherSourceKind::off))
        return std::nullopt;
    if (rows < 2 || columns < 2 || rows > max_grid_size || columns > max_grid_size || layers != weather_layer_count ||
        channels != texel_channels) {
        return std::nullopt;
    }
    const std::uint64_t cells = static_cast<std::uint64_t>(rows) * static_cast<std::uint64_t>(columns);
    std::vector<float> ground;
    std::vector<std::uint8_t> texels;
    if (!r.read_array(ground, cells) || !r.read_array(texels, cells * layers * channels)) return std::nullopt;
    if (r.at != payload) return std::nullopt;

    WeatherSnapshotId id;
    id.kind = static_cast<WeatherSourceKind>(kind);
    id.lead_h = lead_h;
    id.valid_hour = valid_hour;
    id.cycle_hour = cycle_hour;
    id.analog_hour = analog_hour;
    return WeatherSnapshot::from_parts(id, rows, columns, std::move(ground), std::move(texels));
}

}  // namespace starsim::environment
