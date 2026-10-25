#pragma once

// Shared helpers of the binary cache files (atmosphere tables, weather snapshots): the FNV-1a 64 checksum, the atomic
// write (unique temp file + rename) and a whole-file read. Internal to core/src/environment.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <optional>
#include <string_view>
#include <vector>

namespace starsim::environment::detail {

std::uint64_t fnv1a64(const char* data, std::size_t size);

// A temp file name next to `file`, unique across processes and threads ("<name>.tmp.<...>").
std::filesystem::path unique_temp_path(const std::filesystem::path& file);

// Writes the bytes to a unique temp file next to `file` and renames it into place (retried with a short back-off: on
// Windows the rename fails transiently while a virus scanner holds the fresh file). Throws std::runtime_error.
void write_file_atomically(const std::filesystem::path& file, const std::vector<char>& bytes);

// Removes the files of `directory` whose name contains name_marker and that are older than stale_temp_file_age: the
// temp files of writers killed between write and rename (a live writer needs well under a second between creating
// and renaming its file). Errors are ignored.
inline constexpr std::chrono::minutes stale_temp_file_age{10};
void remove_stale_temp_files(const std::filesystem::path& directory, std::string_view name_marker);

// The whole file, or nullopt if it cannot be read.
std::optional<std::vector<char>> read_whole_file(const std::filesystem::path& file);

template <class T>
void append_bytes(std::vector<char>& out, const T& value) {
    const auto* p = reinterpret_cast<const char*>(&value);
    out.insert(out.end(), p, p + sizeof(T));
}

// Reads sequentially from a byte buffer; every read checks the remaining length.
struct ByteReader {
    const std::vector<char>& bytes;
    std::size_t at = 0;
    template <class T>
    bool read(T& value) {
        if (bytes.size() - at < sizeof(T)) return false;
        std::memcpy(&value, bytes.data() + at, sizeof(T));
        at += sizeof(T);
        return true;
    }
    template <class T>
    bool read_array(std::vector<T>& out, std::uint64_t count) {
        if ((bytes.size() - at) / sizeof(T) < count) return false;
        out.resize(static_cast<std::size_t>(count));
        std::memcpy(out.data(), bytes.data() + at, static_cast<std::size_t>(count) * sizeof(T));
        at += static_cast<std::size_t>(count) * sizeof(T);
        return true;
    }
};

}  // namespace starsim::environment::detail
