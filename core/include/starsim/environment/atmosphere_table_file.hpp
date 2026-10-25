#pragma once

// Binary cache of the precomputed atmosphere tables (M8 atmosphere). Layout (little-endian, native x86-64):
//   char[8]  magic "STARATMO"
//   uint32   format version (atmosphere_table_file_version), uint32 0
//   uint64   input hash (atmosphere_input_hash)
//   int32[8] table sizes (AtmosphereTableSizes in declaration order)
//   double[16] parameters (AtmosphereParameters in declaration order)
//   uint64[4] float counts of transmittance, scattering, single_mie, irradiance
//   float32[] the four tables in that order
//   uint64   FNV-1a 64 checksum of every byte before it
// A file that is short, long, damaged (checksum), of another version or of other inputs (hash) is never read.

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>

#include "starsim/environment/atmosphere_tables.hpp"

namespace starsim::environment {

inline constexpr std::uint32_t atmosphere_table_file_version = 2;  // 2: ground_albedo among the parameters
inline constexpr const char* atmosphere_cache_file_name = "earth_atmosphere.bin";

// Writes the tables to a unique temp file next to `file` and renames it into place (parallel writers never expose a
// partial file). Throws std::runtime_error on an I/O error.
void write_atmosphere_tables(const std::filesystem::path& file, const AtmosphereTables& tables);

// The tables of the file, or nullopt if it is missing, damaged, of another format version or its input hash is not
// expected_hash.
std::optional<AtmosphereTables> read_atmosphere_tables(const std::filesystem::path& file, std::uint64_t expected_hash);

// Loads <cache_dir>/earth_atmosphere.bin if its hash matches the configuration (<config_dir>/atmosphere/
// earth_optics.toml) and the sizes, else precomputes the tables and writes the file (creating cache_dir). A failed
// write is reported on stderr and the computed tables are still returned; stale temp files are removed.
std::shared_ptr<const AtmosphereTables> load_or_compute_atmosphere(const std::filesystem::path& config_dir,
                                                                   const std::filesystem::path& cache_dir,
                                                                   const AtmosphereTableSizes& sizes = {});

}  // namespace starsim::environment
