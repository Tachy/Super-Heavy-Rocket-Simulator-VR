#pragma once

#include <optional>
#include <string>
#include <string_view>

namespace starsim::time {

// Calendar date and time in UTC. Only used at the user-interface boundary;
// the simulation itself works with time::Epoch (TDB).
// second may be in [60, 61) on a leap-second day.
struct UtcDateTime {
    int year = 2000;
    int month = 1;
    int day = 1;
    int hour = 0;
    int minute = 0;
    double second = 0.0;

    // ISO 8601, e.g. "2026-10-12T14:03:07.250000" (microsecond resolution).
    std::string to_iso_string() const;

    // Parses "YYYY-MM-DDTHH:MM:SS[.fff...]" (a trailing 'Z' is accepted).
    static std::optional<UtcDateTime> parse_iso(std::string_view text);

    friend bool operator==(const UtcDateTime&, const UtcDateTime&) = default;
};

}  // namespace starsim::time
