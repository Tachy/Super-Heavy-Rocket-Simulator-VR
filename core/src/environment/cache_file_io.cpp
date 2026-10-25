// Shared helpers of the binary cache files (cache_file_io.hpp).

#include "cache_file_io.hpp"

#include <atomic>
#include <chrono>
#include <fstream>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

namespace starsim::environment::detail {

namespace {
constexpr int rename_attempts = 5;  // back-off 50, 100, 150, 200 ms
}

std::uint64_t fnv1a64(const char* data, std::size_t size) {
    std::uint64_t h = 14695981039346656037ull;  // FNV-1a 64 offset basis
    for (std::size_t i = 0; i < size; ++i) {
        h ^= static_cast<unsigned char>(data[i]);
        h *= 1099511628211ull;  // FNV-1a 64 prime
    }
    return h;
}

std::filesystem::path unique_temp_path(const std::filesystem::path& file) {
    static std::atomic<unsigned> counter{0};
    std::random_device rd;
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto thread = std::hash<std::thread::id>{}(std::this_thread::get_id());
    return file.parent_path() / (file.filename().string() + ".tmp." + std::to_string(stamp) + "." +
                                 std::to_string(rd()) + "." + std::to_string(thread) + "." +
                                 std::to_string(counter++));
}

void write_file_atomically(const std::filesystem::path& file, const std::vector<char>& bytes) {
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

void remove_stale_temp_files(const std::filesystem::path& directory, std::string_view name_marker) {
    const auto now = std::filesystem::file_time_type::clock::now();
    std::error_code ec;
    std::filesystem::directory_iterator it(directory, ec), end;
    for (; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.find(name_marker) == std::string::npos) continue;
        std::error_code entry_error;
        const auto written = it->last_write_time(entry_error);
        if (!entry_error && now - written > stale_temp_file_age) std::filesystem::remove(it->path(), entry_error);
    }
}

std::optional<std::vector<char>> read_whole_file(const std::filesystem::path& file) {
    std::error_code size_error;
    const auto file_size = std::filesystem::file_size(file, size_error);
    if (size_error) return std::nullopt;
    std::ifstream in(file, std::ios::binary);
    if (!in) return std::nullopt;
    std::vector<char> bytes(static_cast<std::size_t>(file_size));
    if (!in.read(bytes.data(), static_cast<std::streamsize>(bytes.size())) || in.peek() != EOF) return std::nullopt;
    return bytes;
}

}  // namespace starsim::environment::detail
