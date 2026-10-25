#pragma once

// Decoders of the real-weather inputs (M8 weather and clouds): ERA5 Zarr chunks (blosc1 frame, lz4 codec) and GFS
// GRIB2 messages. The third-party lz4 code is a PRIVATE detail of this module (cmake/weather_decoders.cmake).

#include <cstddef>
#include <span>
#include <stdexcept>
#include <vector>

namespace starsim::weather_decode {

struct FloatGrid {  // row-major, latitude rows north -> south, longitude columns east from 0 deg
    int levels = 1, rows = 0, columns = 0;
    std::vector<float> values;  // levels x rows x columns
    float at(int level, int row, int column) const;
};

struct DecodeError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

// A blosc1 frame (lz4 codec, byte or no shuffle) holding float32 little-endian values; shape from the caller.
// Throws DecodeError on any malformed header, unsupported codec, size mismatch.
FloatGrid decode_zarr_blosc_chunk(std::span<const std::byte> chunk, int levels, int rows, int columns);

// One GRIB2 message (sections 0-8) on a regular lat-lon grid (template 3.0, scanning north -> south, west -> east)
// without a bitmap. Packing templates: 5.0 (simple), 5.2 (complex) and 5.3 (complex with spatial differencing) - all
// that GFS 0.25 deg pgrb2 uses for its fields (5.0 only for constant fields); 5.40 (JPEG2000) and bitmaps are refused
// with DecodeError. Returns grid + metadata.
struct Grib2Field {
    FloatGrid grid;
    double first_latitude_deg, first_longitude_deg, step_deg;
};
Grib2Field decode_grib2_message(std::span<const std::byte> message);

}  // namespace starsim::weather_decode
