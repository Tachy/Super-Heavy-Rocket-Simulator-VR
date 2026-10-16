// Private helpers shared by the lunar FlightSession files (flight_session_lunar*.cpp). Not part of the public API.
#pragma once

#include <algorithm>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "starsim/time/epoch.hpp"

namespace starsim::sim::detail {

inline constexpr double far_future_s = 1e9;  // "nothing timed"

// First epoch of `epochs` after t.
inline std::optional<time::Epoch> first_after(const std::vector<time::Epoch>& epochs, time::Epoch t) {
    for (const auto& e : epochs) {
        if (e - t > 0.0) return e;
    }
    return std::nullopt;
}

// German number with a decimal comma.
inline std::string german(double value, int decimals) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    std::string s = text;
    std::replace(s.begin(), s.end(), '.', ',');
    return s;
}

}  // namespace starsim::sim::detail
