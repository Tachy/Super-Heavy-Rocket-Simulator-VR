#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/config/rock_abundance_file.hpp"

namespace starsim::environment {

// Local rock abundance map: area fraction of the rocks Diviner sees (diameter >= abundance_reference_min_diameter_m)
// on a simple-cylindrical grid over |latitude| <= lat_limit_rad (the Diviner GDR L3 rock abundance product covers
// 60 S - 60 N). Line 0 is the northern edge, sample 0 longitude 0 (east positive); pixel centres at half steps.
// No-data pixels are NaN (make_rock_abundance_map fills them).
struct RockAbundanceMap {
    int lines = 0, samples = 0;
    double lat_limit_rad = 0.0;
    std::vector<float> area_fraction;  // row-major, lines x samples
    // Bilinear over the valid neighbours (longitude wraps, latitude clamped to the outer pixel centres); NaN when all
    // four are no data.
    double at(double latitude_rad, double longitude_rad) const;
};

// Replaces every NaN pixel by the mean of its valid neighbours, front by front (deterministic, order independent), so
// the abundance factor is continuous across gaps: filled values stay within the range of the valid pixels. A map
// without any valid pixel is left unchanged.
void fill_rock_abundance_gaps(RockAbundanceMap& map);

// Converts the Diviner file (DN -> area fraction, missing DN -> NaN, then gaps filled by fill_rock_abundance_gaps). Throws std::invalid_argument when its grid is
// not the layout RockAbundanceMap assumes (lines symmetric about the equator, sample 0 centred half a pixel east of
// longitude 0, max latitude = lines / 2 / ppd).
RockAbundanceMap make_rock_abundance_map(const config::RockAbundanceFile& file);

// The map the configuration asks for: null for abundance_source = "uniform" (the file is not read), the Diviner map
// from terrain_dir / config::rock_abundance_file_name for "diviner" (ConfigError naming fetch.py when it is missing).
std::shared_ptr<const RockAbundanceMap> load_rock_abundance_map(const config::RockPopulation& rocks,
                                                                const std::filesystem::path& terrain_dir);

}  // namespace starsim::environment
