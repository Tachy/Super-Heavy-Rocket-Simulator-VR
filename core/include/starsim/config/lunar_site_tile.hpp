#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "starsim/config/lunar_site_config.hpp"

namespace starsim::config {

// One file of a product (config/lunar_terrain_products.toml [[product.file]]). Polar files use pole_* / scale_m,
// cylindrical files pixels_per_degree / first_*_deg (the outer edge of pixel (0, 0)).
struct LunarTerrainFile {
    std::string url;
    int lines = 0;
    int samples = 0;
    std::uint64_t record_bytes = 0;
    std::uint64_t header_bytes = 0;
    double pole_line = 0.0;  // 0-based pixel-centre coordinates of the pole
    double pole_sample = 0.0;
    double scale_m = 0.0;
    double pixels_per_degree = 0.0;
    double first_latitude_deg = 0.0;
    double first_longitude_deg = 0.0;
};

struct LunarTerrainProduct {
    std::string id;
    std::string kind;        // "height" | "albedo"
    std::string projection;  // "polar_stereographic_south" | "_north" | "simple_cylindrical"
    double latitude_min_deg = 0.0;
    double latitude_max_deg = 0.0;
    double resolution_m = 0.0;
    std::string sample_type;  // "int16" | "float32" (little endian)
    double scale = 1.0;       // value = raw * scale + offset
    double offset = 0.0;
    std::string source;
    double invalid_below = -std::numeric_limits<double>::infinity();  // raw values below this are no data
    double tile_half_size_m = 0.0;  // the product's own, else [selection] (height_ or albedo_tile_half_size_m)
    std::vector<LunarTerrainFile> files;
};

// How neighbouring row pieces of one file merge into a request: pieces whose gap is at most max_gap_bytes join while
// the block stays within max_block_bytes. Height: gap 0 (exactly the crop rows; whole rows of a full-width crop join
// anyway), so a 150 km tile is ~2500 small ranges (the PDS Geosciences server takes 32 at a time). Albedo: the LROC
// host (CloudFront) blocks the whole host with 429 after ~1500 small ranges, so rows join into blocks of at most
// 24 MiB (~220 equatorial rows, ~7 requests per tile) at the price of the unused bytes between the crops.
struct RangePolicy {
    std::uint64_t max_gap_bytes = 0;
    std::uint64_t max_block_bytes = 24ull << 20;
};
RangePolicy default_range_policy(std::string_view kind);

struct ByteRange {
    std::size_t file_index = 0;  // in the product's file list
    std::string url;
    std::uint64_t first = 0, last = 0;  // inclusive
};

// A run of tile pixels of one tile line that lies contiguous in one file (a line across a file seam or the 0/360
// wrap has two): where it sits in the body of its range.
struct TileSegment {
    std::size_t range_index = 0;
    std::uint64_t body_offset = 0;
    int line = 0;
    int sample = 0;
    int count = 0;
};

struct SiteTileRequest {
    std::string product_id, kind;  // kind: "height" | "albedo"
    double latitude_rad = 0.0, longitude_rad = 0.0, half_size_m = 0.0;
    std::vector<ByteRange> ranges;  // sorted by (file, offset); rows of the crop, merged by the RangePolicy
    std::string cache_key;          // "<kind>_<product>_<lat 0.01 deg>_<lon 0.01 deg>_<half km>"

    // Tile geometry and decoding (everything assemble_site_tile needs; the products are not consulted again).
    std::string projection;
    int lines = 0, samples = 0;
    double sphere_radius_m = 0.0;
    // polar: pole and crop origin in the full product
    double pole_line = 0.0, pole_sample = 0.0, scale_m = 0.0;
    int first_line = 0, first_sample = 0;
    // cylindrical: pixel (0, 0) is centred at this latitude / longitude, lines run south, samples east
    double pixels_per_degree = 0.0, first_latitude_rad = 0.0, first_longitude_rad = 0.0;
    std::string sample_type;
    double value_scale = 1.0, value_offset = 0.0, invalid_below = -std::numeric_limits<double>::infinity();
    double resolution_m = 0.0;
    std::vector<TileSegment> segments;
};

// Reads config/lunar_terrain_products.toml. The result is in selection order (height_order, then albedo_order); each
// product carries its resolved tile half size. Throws ConfigError on a malformed file.
std::vector<LunarTerrainProduct> load_lunar_terrain_products(const std::filesystem::path& config_dir);

// The tile of `kind` around the site from the FIRST product (in the given order) whose coverage holds the whole tile.
// A tile is assembled from one product only: several files of it are fine (45 deg file seams, the 0/360 wrap), but a
// tile across two products of different projections (albedo near +-60 deg: equatorial against polar mosaic) is
// refused, as is any tile that no product holds, with report::Refusal lunar.site_no_product.
SiteTileRequest plan_site_tile(const std::vector<LunarTerrainProduct>& products, std::string_view kind,
                               double latitude_rad, double longitude_rad);
SiteTileRequest plan_site_tile(const std::vector<LunarTerrainProduct>& products, std::string_view kind,
                               double latitude_rad, double longitude_rad, const RangePolicy& policy);

// Decodes the downloaded range bodies (one per request.ranges entry, in order) into the tile file. Heights: int16 DN
// with dn_scale_m 0.5 and 9 check points. Albedo (kind "albedo", ledger ruling R3): the same int16 format with
// dn_scale_m 1/16384, relative albedo = tile / its low-pass at the global map's pixel footprint (2665 m, ruling R21), scaled
// to a tile mean of exactly 1 (1.0 = 16384); no-data pixels are 1.0. Geographic grids keep the source grid (no reprojection).
TerrainGridFile assemble_site_tile(const SiteTileRequest& request, const std::vector<std::vector<std::uint8_t>>& bodies);

}  // namespace starsim::config
