#pragma once

// Cache file of one weather snapshot (M8 weather and clouds), extension .wthr. Layout (little-endian, native x86-64):
//   char[8]  magic "STARWTHR"
//   uint32   format version (weather_snapshot_file_version), uint32 0
//   int32    source kind (WeatherSourceKind), int32 lead_h
//   int64    valid_hour, cycle_hour, analog_hour
//   int32    rows, columns, layers (32), channels (4)
//   float32[rows x columns]            ground heights above the ellipsoid (m)
//   float32[rows x columns]            lifting condensation level above the ground (m; 0 = saturated, -1 = unknown)
//   uint8[layers x rows x columns x 4] render texels (weather_snapshot.hpp encoding)
//   uint64   FNV-1a 64 checksum of every byte before it
// Version 1 (no LCL block) and 2 (unknown LCL stored as 0, texels without the LCL redistribution) are refused like any
// other version: the caller rebuilds / re-downloads it.
// A file that is short, long, damaged (checksum) or of another version is never read.

#include <cstdint>
#include <filesystem>
#include <optional>

#include "starsim/environment/weather_snapshot.hpp"

namespace starsim::environment {

inline constexpr std::uint32_t weather_snapshot_file_version = 3;
inline constexpr const char* weather_snapshot_file_extension = ".wthr";

// Writes to a unique temp file next to `file` and renames it into place; first removes the stale temp files of killed
// writers in that directory ("*.wthr.tmp.*" older than 10 minutes). Throws std::runtime_error on an I/O error.
void write_weather_snapshot(const std::filesystem::path& file, const WeatherSnapshot& snapshot);

// The snapshot of the file, or nullopt if it is missing, damaged or of another format version.
std::optional<WeatherSnapshot> read_weather_snapshot(const std::filesystem::path& file);

}  // namespace starsim::environment
