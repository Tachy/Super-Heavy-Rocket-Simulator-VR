# Third-party decoders of the weather_decode module (M8 weather and clouds), built from the sources fetched by
# tools/data_pipeline/fetch.py. Linked PRIVATE to starsim_core: only core/src/weather_decode/ includes their headers.
#   lz4 1.10.0 (BSD-2-Clause): the lz4 blocks inside the blosc1 frames of the ERA5 Zarr chunks.
# GRIB2 needs no library: the packing templates GFS uses (5.0, 5.2, 5.3) are decoded in core/src/weather_decode/grib2.cpp
# (NCEPLIBS-g2c is LGPL, ecCodes far too heavy for three templates).

set(LZ4_ROOT "${STARSIM_EXTERNAL_DIR}/lz4-1.10.0")
if(NOT EXISTS "${LZ4_ROOT}/lib/lz4.c")
    message(FATAL_ERROR "lz4 sources not found in ${LZ4_ROOT}. Run: python tools/data_pipeline/fetch.py lz4")
endif()

add_library(starsim_lz4 STATIC "${LZ4_ROOT}/lib/lz4.c")
target_include_directories(starsim_lz4 SYSTEM PUBLIC "${LZ4_ROOT}/lib")
if(MSVC)
    target_compile_options(starsim_lz4 PRIVATE /w)
else()
    target_compile_options(starsim_lz4 PRIVATE -w)
endif()
