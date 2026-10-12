#include "starsim/time/utc_date_time.hpp"

#include <charconv>
#include <cstdio>

namespace starsim::time {

namespace {

// Reads a fixed-width unsigned integer and advances `pos`.
bool read_int(std::string_view text, size_t& pos, size_t width, int& out) {
    if (pos + width > text.size()) {
        return false;
    }
    const char* first = text.data() + pos;
    const auto [ptr, ec] = std::from_chars(first, first + width, out);
    if (ec != std::errc{} || ptr != first + width) {
        return false;
    }
    pos += width;
    return true;
}

bool expect(std::string_view text, size_t& pos, char c) {
    if (pos >= text.size() || text[pos] != c) {
        return false;
    }
    ++pos;
    return true;
}

}  // namespace

std::string UtcDateTime::to_iso_string() const {
    char buffer[40];
    std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%09.6f", year, month, day, hour, minute,
                  second);
    return buffer;
}

std::optional<UtcDateTime> UtcDateTime::parse_iso(std::string_view text) {
    if (!text.empty() && (text.back() == 'Z' || text.back() == 'z')) {
        text.remove_suffix(1);
    }
    UtcDateTime result;
    size_t pos = 0;
    if (!read_int(text, pos, 4, result.year) || !expect(text, pos, '-') ||
        !read_int(text, pos, 2, result.month) || !expect(text, pos, '-') ||
        !read_int(text, pos, 2, result.day) || !(expect(text, pos, 'T') || expect(text, pos, ' ')) ||
        !read_int(text, pos, 2, result.hour) || !expect(text, pos, ':') ||
        !read_int(text, pos, 2, result.minute) || !expect(text, pos, ':')) {
        return std::nullopt;
    }
    const auto [ptr, ec] = std::from_chars(text.data() + pos, text.data() + text.size(), result.second);
    if (ec != std::errc{} || ptr != text.data() + text.size()) {
        return std::nullopt;
    }
    const bool valid = result.month >= 1 && result.month <= 12 && result.day >= 1 && result.day <= 31 &&
                       result.hour <= 23 && result.minute <= 59 && result.second >= 0.0 && result.second < 61.0;
    if (!valid) {
        return std::nullopt;
    }
    return result;
}

}  // namespace starsim::time
