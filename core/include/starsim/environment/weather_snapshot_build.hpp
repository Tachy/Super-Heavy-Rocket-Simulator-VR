#pragma once

// Raw downloads -> one weather snapshot (M8 weather and clouds): the decoding half of the weather pipeline, shared by the
// weather_snapshot CLI and the GDExtension (the game builds a snapshot from the files it downloaded).
//
// Raw files, named by weather_source.hpp (era5_file_name, gfs_level_file_name, ...) as the request planning saves them:
//   ERA5 and analog (the data is the ERA5 hour of the reference year, key id.cache_key()): <raw dir>/<key>_<variable>.blosc
//         for fraction_of_cloud_cover, specific_cloud_liquid_water_content, specific_cloud_ice_water_content,
//         temperature (37 x 721 x 1440), surface_pressure, total_column_water_vapour (721 x 1440) and the static
//         era5_geopotential_at_surface.blosc.
//   GFS:  <raw dir>/<key>_<VAR>_<llll>mb.grb2 for TCDC, CLMR, ICMR, HGT, TMP on 1000..50 mb and <key>_HGT_surface.grb2.

#include <filesystem>

#include "starsim/environment/weather_snapshot.hpp"

namespace starsim::environment {

// Builds the snapshot of `id` (kind era5, gfs or analog) from the raw files in raw_dir, the geoid from config_dir;
// downsample > 1 box-averages the grid (WeatherSnapshot::downsampled). The snapshot keeps `id`. Throws
// std::runtime_error / std::invalid_argument on a missing, damaged or unexpected raw file or a kind `off`.
WeatherSnapshot build_weather_snapshot_from_raw(const WeatherSnapshotId& id, const std::filesystem::path& raw_dir,
                                                const std::filesystem::path& config_dir, int downsample = 1);

}  // namespace starsim::environment
