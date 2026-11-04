#pragma once

#include <cstdint>

// Portable integer hash for the procedural surface detail (craters, roughness, rocks): only 64-bit integer operations,
// so every feature is identical across runs, machines, compilers and threads. No library random numbers.

namespace starsim::environment {

// Independent streams for the same cell or node key: the first argument of surface_hash is (stream << 8) | face, the
// others the level (or octave), i and j.
namespace surface_stream {
inline constexpr std::uint64_t craters = 1;
inline constexpr std::uint64_t roughness = 2;
inline constexpr std::uint64_t rocks = 3;
}  // namespace surface_stream

namespace detail {
// SplitMix64 finaliser (Steele, Lea, Flood 2014): a bijective 64-bit avalanche mix.
constexpr std::uint64_t splitmix64(std::uint64_t z) {
    z += 0x9E3779B97F4A7C15ull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}
}  // namespace detail

// Hash of four integers (a cell key face, level, i, j, or a feature key); every argument changes every output bit.
constexpr std::uint64_t surface_hash(std::uint64_t a, std::uint64_t b, std::uint64_t c, std::uint64_t d) {
    std::uint64_t h = detail::splitmix64(a ^ 0x5AF0C0DE5EEDull);
    h = detail::splitmix64(h ^ b);
    h = detail::splitmix64(h ^ c);
    return detail::splitmix64(h ^ d);
}

// Uniform number in [0, 1) from the top 53 bits (exact in double).
constexpr double hash_unit(std::uint64_t h) { return static_cast<double>(h >> 11) * 0x1.0p-53; }

}  // namespace starsim::environment
