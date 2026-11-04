#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace starsim::config {

// Diviner rock abundance map as written by tools/data_pipeline/rock_abundance.py into data/terrain/: a TOML header
// ([grid]) and the product's int16 DN (little endian, line 0 = northern edge). Area fraction = DN x dn_scale; DN ==
// missing_dn is no data. Simple cylindrical: line = zero_line - ppd lat_deg, sample = zero_sample + ppd lon_deg
// (0-based pixel centres, longitude 0..360 east).
struct RockAbundanceFile {
    std::string product;
    int lines = 0, samples = 0;
    double pixels_per_degree = 0.0, zero_line = 0.0, zero_sample = 0.0;
    double max_latitude_deg = 0.0;  // the map covers -max .. +max
    double dn_scale = 0.0;
    int missing_dn = 0;
    std::vector<std::int16_t> dn;  // lines x samples, row-major
};

// File name of the map's header inside the terrain data directory (data/terrain).
inline constexpr const char* rock_abundance_file_name = "diviner_rock_abundance.toml";

// Reads the header and its DN file. Throws ConfigError (German, names tools/data_pipeline/fetch.py) when the file or
// its data are missing, ConfigError on a bad key or a data file of the wrong size.
RockAbundanceFile load_rock_abundance_file(const std::filesystem::path& toml_path);

}  // namespace starsim::config
