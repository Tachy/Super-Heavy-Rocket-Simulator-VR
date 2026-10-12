#pragma once

#include <compare>

namespace starsim::time {

// An instant in time, stored as TDB seconds past J2000.0 (= SPICE "ephemeris time").
// A double resolves ~0.1 microseconds over +-150 years, far below every modelled effect.
class Epoch {
public:
    constexpr Epoch() = default;

    static constexpr Epoch from_tdb_seconds_past_j2000(double tdb_s) { return Epoch(tdb_s); }

    constexpr double tdb_seconds_past_j2000() const { return tdb_s_; }

    constexpr Epoch operator+(double seconds) const { return Epoch(tdb_s_ + seconds); }
    constexpr Epoch operator-(double seconds) const { return Epoch(tdb_s_ - seconds); }
    constexpr Epoch& operator+=(double seconds) { tdb_s_ += seconds; return *this; }

    // Elapsed TDB seconds between two epochs.
    constexpr double operator-(const Epoch& other) const { return tdb_s_ - other.tdb_s_; }

    constexpr auto operator<=>(const Epoch&) const = default;

private:
    explicit constexpr Epoch(double tdb_s) : tdb_s_(tdb_s) {}

    double tdb_s_ = 0.0;
};

}  // namespace starsim::time
