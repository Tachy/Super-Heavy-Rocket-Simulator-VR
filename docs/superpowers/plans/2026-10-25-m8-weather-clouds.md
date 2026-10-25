# M8 Weather and Clouds (Part 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The real weather of the scene's epoch (ERA5 archive, GFS analysis / forecast, a typical analog day) as
volumetric, flyable clouds everywhere on Earth, lit and shadowing physically, within the 90 Hz VR budget.

**Architecture:** The core decodes raw ERA5 Zarr chunks and GFS GRIB2 records (external decoders only in the new
module `weather_decode`), turns them into an immutable `WeatherSnapshot` on fixed ellipsoidal height layers (heights
hypsometric from temperature and surface pressure, or GFS `HGT`; water content to kg/m^3) and writes it as a compact
cache file; it also picks the source per epoch, plans the HTTP requests, generates the cloud noise and holds the CPU
reference of the cloud optics. The game downloads (Godot HTTP, worker threads) into `data/weather/`, uploads two
snapshots as 3D textures and blends them; one full-screen spatial shader raymarches the clouds wherever the camera is
in true space (detail noise fades to the coverage-preserving mean with distance), `earth.gdshader` shades the same
field from its column in scaled space, and a shared include gives every lit surface its cloud shadow.

**Tech Stack:** C++20 core (CMake/Ninja, Catch2, toml++), vendored lz4 (BSD-2) + a GRIB2 decoder, godot-cpp
GDExtension, Godot 4.7 double build (Forward+, D3D12), Godot shading language, Python data pipeline (numpy, xarray,
numcodecs, cfgrib/eccodes only in offline tools), PowerShell build scripts.

**Spec:** `docs/superpowers/specs/2026-10-25-m8-weather-clouds-design.md`

## Global Constraints

- SI units in the core; names carry frame and unit (`height_ellipsoid_m`, `liquid_water_kg_m3`); angles in radians;
  weather valid times as `std::int64_t` hours since 1900-01-01 00:00 UTC (the ERA5 Zarr time index), UTC only.
- `core` has no Godot dependency and **no networking**; the external decoders (lz4, GRIB2) are linked PRIVATE to
  `weather_decode` and included by no other module (the CSPICE rule); physics / decoding never in GDScript.
- Constants with their source in `constants.hpp` (rho_w 1000 kg/m^3, rho_ice 917 kg/m^3, R_d 287.05 J/(kg K),
  g0 9.80665 m/s^2, effective radii liquid 10e-6 m [E] / ice 30e-6 m [E] with citations).
- ERA5: ARCO-ERA5 store `https://storage.googleapis.com/gcp-public-data-arco-era5/ar/full_37-1h-0p25deg-chunk-1.zarr-v3`,
  chunk key `<variable>/<t>.0.0.0` (3D: `<t>.0.0`), t = hours since 1900-01-01 (verify against the `time` array once,
  Task 2), float32 little-endian, blosc (lz4, clevel 5, byte shuffle). Variables: `fraction_of_cloud_cover`,
  `specific_cloud_liquid_water_content`, `specific_cloud_ice_water_content`, `temperature` (37 levels), `surface_pressure`,
  `geopotential_at_surface` (constant: any t). Era ends from the root `.zattrs`: `valid_time_stop`,
  `valid_time_stop_era5t`. **No geopotential, no wind** (user, ~150 MB per hour).
- GFS: `https://noaa-gfs-bdp-pds.s3.amazonaws.com/gfs.YYYYMMDD/HH/atmos/gfs.tHHz.pgrb2.0p25.fFFF` + `.idx`; records
  `TCDC`, `CLMR`, `ICMR`, `HGT`, `TMP` on the pressure levels from 1000 to 50 mb, fetched by HTTP `Range`; cycles 00/06/12/18,
  forecasts hourly to 120 h, 3-hourly to 384 h.
- Height layers (render and cache form): 32 layers above the WGS84 ellipsoid, 16 x 250 m from 0 to 4 km, then
  16 x 1 km from 4 to 20 km; layer centres at mid-layer. One mapping function in core and shader.
- Time warp (user): hourly snapshots while downloads keep up, then 3 h, then 6 h, then hold the last weather with the
  UI saying so; brought to the current time (blended) when the warp drops.
- Performance gate (user, 90 Hz, clouds <= +3 ms): `res://tests/terrain_perf.gd` 1440p desktop proxy with clouds on at
  the VR quality: p99 <= 11 ms, max <= 16 ms. Both VR eyes see the same cloud shapes.
- Anti-goals: no tuned colours / opacity sliders bypassing the field, no random weather, no pop (near / far,
  snapshot, source change), never block the flight on a download.
- UI German ("Wetter: ERA5, 08.04.2024 18:00 UTC", "Wetter: GFS-Vorhersage vom 25.10. 00 UTC, +54 h",
  "Wetter: typisch (Analogtag 08.04.2025)", "Wetter: Stand 18:00 UTC", "Wetter: aus"), code / comments / commits English,
  Conventional Commits; commit + `git push origin main` per task.
- Tests: core cases tagged `[weather]` (`[slow]` if > ~40 s); run only affected tags
  (`tools/build.ps1 -NoBindings -TestFilter "[weather]"`; with bindings once the binding changes, plus `[atmosphere]`
  when `atmosphere_*` changes); GDScript tests headless; GPU checks without `--headless`.
- Visual work: send progress PNGs to the user with SendUserFile after each rendering task (memory: send images often).

## Review Focus

1. **Network gone or slow mid-flight** (no internet, a 404 for a not-yet-published GFS cycle, a truncated download):
   the scene keeps flying with the last snapshot or the analog, says so, never stalls, and a damaged file is never
   read (tests in Task 2 for 404 / fallback planning, Task 3 for truncated files, Task 7 with a failing fake fetcher).
2. **Epoch at the source seams** - exactly at `valid_time_stop_era5t`, at the newest GFS cycle not yet on the server,
   beyond 384 h, before 1940: a defined source, no gap, no exception (Task 2 tests).
3. **Mountains and the ground below 1000 hPa levels** (Himalaya, Andes; levels under the surface in ERA5 / GFS): no
   cloud under the terrain, no NaN from extrapolated levels (Task 3 test with a high-altitude column).
4. **Poles and the date line** (lat +-90, lon 0 / 360 seam): no seam or NaN in the field, the noise or the shader
   lookup (Task 3 and Task 4 tests; visual in Task 11 over Antarctica and the Pacific date line).
5. **Camera inside a cloud and at the pad under a deck** (inside the volume, the raymarch start; the camera between
   layers; the drone ship at sea level under stratus): no black frame, no banding, continuous when climbing out
   (Task 8 GPU check; Task 4 CPU test for a point inside the slab).

---

### Task 1: The decoder module (`weather_decode`)

**Files:**
- Create: `cmake/weather_decoders.cmake` (vendored sources from `external/`, PRIVATE to `weather_decode`)
- Create: `core/include/starsim/weather_decode/weather_decode.hpp`, `core/src/weather_decode/zarr_blosc.cpp`,
  `core/src/weather_decode/grib2.cpp`
- Create: `tools/validation/gen_weather_fixtures.py`, committed output in `tests/validation/weather/`
- Modify: `CMakeLists.txt` (include the cmake file), `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`,
  `tools/data_pipeline/fetch.py` + `sources.toml` (fetch the decoder sources into `external/` like CSPICE)
- Test: `core/tests/test_weather_decode.cpp`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::weather_decode {
  struct FloatGrid {                 // row-major, latitude rows north -> south, longitude columns east from 0 deg
      int levels = 1, rows = 0, columns = 0;
      std::vector<float> values;     // levels x rows x columns
      float at(int level, int row, int column) const;
  };
  // A blosc1 frame (lz4 codec, byte or no shuffle) holding float32 little-endian values; shape from the caller.
  // Throws DecodeError on any malformed header, unsupported codec, size mismatch.
  FloatGrid decode_zarr_blosc_chunk(std::span<const std::byte> chunk, int levels, int rows, int columns);
  // One GRIB2 message (sections 0-8) on a regular lat-lon grid (template 3.0); packing templates 5.0, 5.2, 5.3 and
  // 5.40 (JPEG2000) where GFS uses them - the builder lists what the fixtures contain. Returns grid + metadata.
  struct Grib2Field { FloatGrid grid; double first_latitude_deg, first_longitude_deg, step_deg; };
  Grib2Field decode_grib2_message(std::span<const std::byte> message);
  struct DecodeError : std::runtime_error { using std::runtime_error::runtime_error; };
  }
  ```

- [ ] **Step 1: Choose the decoders.** lz4 (BSD-2, `lz4.c`/`lz4.h`) for blosc's lz4 blocks; the blosc1 frame header,
  block split and byte unshuffle are small enough to implement here (document the format with its reference,
  c-blosc `README_HEADER.rst`). GRIB2: NCEPLIBS-g2c or ecCodes - pick by licence (permissive required for the
  installer; record it) and footprint; if neither fits, implement the packing templates the fixtures need. Note the
  choice and licence in the commit body.
- [ ] **Step 2: Write `gen_weather_fixtures.py`** (offline, `-I`, needs xarray, zarr, numcodecs, cfgrib or eccodes):
  - a 4-level x 8 x 16 cut-out of a real ERA5 `fraction_of_cloud_cover` hour, re-encoded with numcodecs
    `Blosc(cname="lz4", clevel=5, shuffle=Blosc.SHUFFLE)` (the store's codec), plus the raw float values;
  - one real GFS record (`CLMR` at 300 mb, a few hundred kB; byte range from the `.idx`) and one `TCDC` record, with
    values at 20 listed grid points decoded by cfgrib;
  - the ERA5 columns for Task 3 (see there). Writes `tests/validation/weather/*.bin` + `*_reference.json`.
- [ ] **Step 3: Write the failing tests** (`[weather]`):
  - `"Zarr blosc chunk decodes bit-exact"`: the fixture decodes to the reference floats exactly.
  - `"Damaged blosc chunks are refused"`: truncated, wrong typesize, unknown codec byte -> `DecodeError`.
  - `"GRIB2 records decode to cfgrib's values"`: CLMR and TCDC at the 20 points within float32 rounding of the
    packing's precision (state it: the binary/decimal scale of the record).
  - `"GRIB2 grid metadata"`: 721 x 1440, first lat 90, lon 0, step 0.25.
- [ ] **Step 4: Run** `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[weather]"` —
  expected: FAIL (module missing).
- [ ] **Step 5: Implement** the module and the CMake wiring.
- [ ] **Step 6: Run** the same command — expected: PASS.
- [ ] **Step 7: Commit + push** `feat(weather_decode): ERA5 Zarr blosc and GFS GRIB2 decoding behind one module`.

### Task 2: Source selection and request planning

**Files:**
- Create: `core/include/starsim/environment/weather_source.hpp`, `core/src/environment/weather_source.cpp`
- Test: `core/tests/test_weather_source.cpp`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::environment {
  enum class WeatherSourceKind { era5, gfs, analog, off };
  struct Era5Availability { std::int64_t final_stop_hour, era5t_stop_hour; };   // from the store's .zattrs
  struct GfsCycle { std::int64_t cycle_hour; };                                 // 00/06/12/18 UTC run time
  struct WeatherSnapshotId {
      WeatherSourceKind kind; std::int64_t valid_hour;   // the snapshot's valid time
      std::int64_t cycle_hour = 0; int lead_h = 0;       // gfs
      std::int64_t analog_hour = 0;                      // analog: the reference-year hour shown
      std::string cache_key() const;                     // unique file stem, e.g. "era5_1088136"
  };
  struct WeatherRequest { std::string url; std::int64_t first_byte = -1, last_byte = -1; std::string file_name; };
  // The two snapshots bracketing utc_hour_f (fractional hours since 1900) for the warp step (1, 3, 6 h), the blend
  // factor between them, and the source each one comes from. latest_gfs is the newest cycle known to exist (nullopt:
  // offline / unknown). Beyond the last GFS lead (384 h) or offline: analog.
  struct WeatherBracket { WeatherSnapshotId before, after; double blend; };
  WeatherBracket weather_bracket(double utc_hour_f, int step_h, const std::optional<Era5Availability>&,
                                 const std::optional<GfsCycle>& latest_gfs, int analog_reference_year);
  // Requests for one snapshot; GFS needs the parsed .idx of its file first.
  std::vector<WeatherRequest> era5_requests(const WeatherSnapshotId&);
  std::string gfs_index_url(const WeatherSnapshotId&);
  std::vector<WeatherRequest> gfs_requests(const WeatherSnapshotId&, std::string_view idx_text);  // byte ranges
  std::vector<GfsCycle> gfs_cycle_candidates(double wall_clock_utc_hour_f, int count);  // newest first, for probing
  Era5Availability parse_era5_zattrs(std::string_view json);   // throws on missing keys
  // Warp policy: snapshot step (1, 3, 6) or 0 = hold, from the warp factor and the measured seconds per snapshot.
  int weather_snapshot_step(double warp_factor, double seconds_per_snapshot_download);
  std::int64_t hours_since_1900(const time::UtcDateTime&);
  }
  ```

- [ ] **Step 1: Verify the ERA5 time index** once (curl the `time` array's first chunk, decode with Task 1) and state
  in a comment that index = hours since 1900-01-01 (or the offset found).
- [ ] **Step 2: Write the failing tests** (`[weather]`) — Review Focus 2 lives here:
  - `"Archive epochs use ERA5"`: 2024-04-08 18:30 with ERA5T ending 2026-10-19 -> both snapshots era5, hours
    18 / 19, blend 0.5; 1940-01-01 00:00 -> era5; before 1940 -> analog.
  - `"The ERA5T end hands over to GFS without a gap"`: an epoch one hour after `era5t_stop_hour` -> before = era5
    (last hour), after = gfs (analysis or shortest lead of the covering cycle).
  - `"Future epochs use the latest forecast, then the analog"`: +54 h from the latest cycle -> gfs lead 54; +200 h ->
    3-hourly leads bracket it; +400 h -> analog of the same calendar day/hour in the reference year (29 Feb ->
    28 Feb in a non-leap year).
  - `"Offline means analog"`: no availability and no cycle -> analog both.
  - `"GFS byte ranges come from the index"`: a committed `.idx` excerpt -> one range per wanted record (5 variables x
    levels 1000..50 mb), the last record's range ends at the next offset; a missing record throws.
  - `"Cycle candidates step back in 6 h"`: wall clock 2026-10-25 03:10 -> 00Z, 18Z (24th), 12Z ... newest first.
  - `"Warp policy"`: download 20 s per snapshot: warp 1 -> 1 h; warp 360 (an hour per 10 s) -> 3; warp 2000 -> 6;
    warp 1e5 -> 0 (hold). Monotone in warp.
  - `"ERA5 .zattrs parse"`: the real root attributes (committed excerpt) give the two stop hours; missing key throws.
- [ ] **Step 3: Run** `tools/build.ps1 -NoBindings -TestFilter "[weather]"` — expected: FAIL.
- [ ] **Step 4: Implement** (pure functions; the warp policy chooses the smallest step whose snapshot interval in wall
  time exceeds ~2x the download time, the factor documented).
- [ ] **Step 5: Run** — expected: PASS.
- [ ] **Step 6: Commit + push** `feat(environment): weather source by epoch, ERA5 / GFS request planning, warp policy`.

### Task 3: The weather snapshot (heights, water content, layers, cache file)

**Files:**
- Create: `core/include/starsim/environment/weather_snapshot.hpp`, `core/src/environment/weather_snapshot.cpp`
  (build from decoded fields, layer mapping, queries)
- Create: `core/include/starsim/environment/weather_snapshot_file.hpp`, `core/src/environment/weather_snapshot_file.cpp`
- Create: `core/apps/weather_snapshot.cpp` (CLI: raw files -> cache file; used by the pipeline), CMake target
- Modify: `core/include/starsim/math/constants.hpp`
- Test: `core/tests/test_weather_snapshot.cpp`

**Interfaces:**
- Consumes: Task 1 (`FloatGrid`), Task 2 (`WeatherSnapshotId`), `config::load_coarse_geoid`.
- Produces:
  ```cpp
  namespace starsim::environment {
  inline constexpr int weather_layer_count = 32;
  double weather_layer_bottom_m(int layer);            // 16 x 250 m to 4 km, 16 x 1 km to 20 km
  double weather_layer_coordinate(double height_ellipsoid_m);   // continuous layer index for interpolation
  struct WeatherCell { double cloud_fraction, liquid_water_kg_m3, ice_water_kg_m3; };   // grid means
  struct Era5Fields { weather_decode::FloatGrid cloud_fraction, liquid_kg_kg, ice_kg_kg, temperature_k,
                      surface_pressure_pa, surface_geopotential_m2_s2; std::vector<double> levels_pa; };
  struct GfsFields  { weather_decode::FloatGrid cloud_percent, liquid_kg_kg, ice_kg_kg, temperature_k,
                      geopotential_height_m; std::vector<double> levels_pa; };
  class WeatherSnapshot {
  public:
      static WeatherSnapshot from_era5(const WeatherSnapshotId&, const Era5Fields&, const Geoid&);
      static WeatherSnapshot from_gfs(const WeatherSnapshotId&, const GfsFields&, const Geoid&);
      const WeatherSnapshotId& id() const;
      int rows() const; int columns() const;            // 721 x 1440 (or coarser for the analog set)
      // Trilinear in (lat, lon, layer coordinate), longitude wraps, zero above 20 km and below the ground.
      WeatherCell at(double latitude_rad, double longitude_rad, double height_ellipsoid_m) const;
      double ground_height_m(double latitude_rad, double longitude_rad) const;  // ellipsoidal, from the surface field
      // Render form: rows x columns x 32 layers, RGBA8 = (cloud fraction, liquid, ice, unused), encoding documented
      // (relative error <= 2 % above 1e-5 kg/m^3); layer-major so the game builds one Image per layer.
      std::vector<std::uint8_t> render_texels() const;
      WeatherSnapshot downsampled(int factor) const;   // analog set (factor 4: 1 deg)
      // Synthetic test field: one uniform cloud slab everywhere (Task 4 tests, smoke test, cloud_check.gd).
      static WeatherSnapshot uniform_slab(int rows, int columns, double bottom_m, double top_m,
                                          double liquid_water_kg_m3, double cloud_fraction);
  };
  void write_weather_snapshot(const std::filesystem::path&, const WeatherSnapshot&);          // tmp + rename
  std::optional<WeatherSnapshot> read_weather_snapshot(const std::filesystem::path&);        // nullopt if bad
  }
  ```
  Physics of the build: level heights by integrating the hypsometric equation upward from the surface (surface
  pressure + surface geopotential), dry air (R_d) unless the ERA5 fixture check fails the tolerance - then add
  `specific_humidity` (virtual temperature) and record it in Task 2's ERA5 list; geopotential height -> geometric
  height with the latitude-dependent normal gravity and radius (cite the formula), + geoid undulation -> ellipsoidal;
  kg/kg -> kg/m^3 with rho = p / (R_d T); levels below the surface ignored, layers below the ground zero (Review
  Focus 3). GFS: `HGT` directly, `TCDC` percent -> fraction.

- [ ] **Step 1: Extend `gen_weather_fixtures.py`** with six ERA5 columns (tropics, mid-latitude ocean, Starbase,
  Cape, Himalaya ~30N 85E, Antarctic plateau) at one hour: all 37 levels of T, cloud fields and the store's
  `geopotential`, plus surface pressure and surface geopotential; and one GFS column with HGT / TMP.
- [ ] **Step 2: Write the failing tests** (`[weather]`):
  - `"Hypsometric heights match ERA5 geopotential"`: per fixture column, every level above the surface up to 100 hPa
    within 30 m of the store's geopotential height (print the max; if dry air misses, the humidity branch above).
  - `"Geopotential height converts to ellipsoidal height"`: 10 000 gpm at 0 / 45 / 90 deg vs an independent
    computation within 1 m; geoid added at a known point.
  - `"Water content in kg per cubic metre"`: 1e-4 kg/kg at 850 hPa, 280 K -> 1e-4 x 85000 / (287.05 x 280).
  - `"No cloud under the ground"` (Review Focus 3): the Himalaya column: `at()` below `ground_height_m` is zero, no
    NaN anywhere in `render_texels()`.
  - `"Poles and the date line"` (Review Focus 4): `at()` at lat +-pi/2 is finite and the same for every longitude
    (within interpolation); lon 359.9 deg and -0.1 deg agree.
  - `"Layer mapping"`: bottoms 0, 250, ..., 4000, 5000, ..., 20000; coordinate is continuous and monotone.
  - `"Render encoding error"`: round trip of liquid/ice values 1e-5 .. 3e-3 kg/m^3 within 2 %.
  - `"Snapshot files round-trip; damaged ones are refused"` (Review Focus 1): write/read equal; truncated copy and a
    flipped byte -> nullopt (unique temp names).
- [ ] **Step 3: Run** `tools/build.ps1 -NoBindings -TestFilter "[weather]"` — expected: FAIL.
- [ ] **Step 4: Implement** the snapshot, the file (magic `STARWTHR`, version, id fields, sizes, texels / float
  fields as the builder chooses, FNV-1a 64 checksum as `atmosphere_table_file.hpp`) and the CLI
  (`weather_snapshot.exe --era5|--gfs <raw dir> --id <cache key> --config <dir> --out <file> [--downsample 4]`).
- [ ] **Step 5: Run** — expected: PASS.
- [ ] **Step 6: Commit + push** `feat(environment): weather snapshots on ellipsoidal height layers with a cache file`.

### Task 4: Cloud optics, noise and the CPU reference

**Files:**
- Create: `core/include/starsim/environment/cloud_optics.hpp`, `core/src/environment/cloud_optics.cpp`
- Create: `core/include/starsim/environment/cloud_noise.hpp`, `core/src/environment/cloud_noise.cpp`
- Test: `core/tests/test_cloud_optics.cpp`

**Interfaces:**
- Consumes: Task 3.
- Produces:
  ```cpp
  namespace starsim::environment {
  double cloud_extinction_per_m(double liquid_water_kg_m3, double ice_water_kg_m3);  // 3 W / (2 rho r_eff), both phases
  double cloud_phase_function(double cos_theta);           // Mie approximation for droplets, citation in the header
  double two_stream_reflectance(double optical_depth, double asymmetry_g);    // conservative scattering reference
  double two_stream_diffuse_transmittance(double optical_depth, double asymmetry_g);
  // The multiple-scattering approximation the shader uses (e.g. Wrenninge octaves), as a plane-parallel slab
  // estimate for the tests and the CPU light.
  double slab_reflectance(double optical_depth, double sun_cos_zenith);
  struct CloudNoise {                                     // tileable 3D textures, deterministic (fixed seed)
      int shape_size = 128, detail_size = 32;
      std::vector<std::uint8_t> shape_rgba, detail_rgba;  // Perlin-Worley shape, Worley detail (Schneider 2015)
      std::array<float, 256> coverage_threshold;          // threshold t(cf) with P(shape > t) = cf, cf = i/255
  };
  CloudNoise generate_cloud_noise();
  // The CPU light: transmittance of the Sun through the grid-mean clouds along the ray (documented: no detail
  // noise, so a vehicle in a gap between cumuli gets the cell's mean), and the diffuse sky factor under the column.
  double cloud_sun_transmittance(const WeatherSnapshot&, const math::Vec3& position_itrf_m,
                                 const math::Vec3& sun_direction_itrf);
  double cloud_sky_factor(const WeatherSnapshot&, const math::Vec3& position_itrf_m);  // two-stream diffuse T of the column above
  }
  ```

- [ ] **Step 1: Write the failing tests** (`[weather]`):
  - `"Extinction from water content"`: 0.3 g/m^3 liquid -> 3 x 3e-4 / (2 x 1000 x 10e-6) = 0.045 /m (exact formula).
  - `"Phase function is normalised and forward-peaked"`: integral over the sphere 1 within 1e-3; g in 0.84-0.88.
  - `"Slab reflectance agrees with two-stream"`: optical depths 1, 3, 10, 30, 50 at Sun zenith 0 and 60 deg within
    15 % (tolerance and reason - octave approximation vs exact transport - in a comment; cite a DISORT or adding-method
    table if one is used instead of two-stream).
  - `"An overcast deck dims the ground physically"`: tau 20 -> diffuse transmittance 0.1-0.3 (cite the measured
    overcast range in the test).
  - `"Noise coverage is preserved"`: for cf = 0.1, 0.5, 0.9 the fraction of shape texels above `coverage_threshold`
    is cf within 0.02; noise tiles seamlessly (opposite faces equal after wrap).
  - `"Sun transmittance through a uniform slab"` (Review Focus 5, a point inside the slab included): a synthetic
    snapshot with one 1-km layer of known extinction -> Beer-Lambert along the slant ray within 1 %; from inside, half
    the path.
- [ ] **Step 2: Run** `tools/build.ps1 -NoBindings -TestFilter "[weather]"` — expected: FAIL.
- [ ] **Step 3: Implement.** The noise generator is plain C++ (no external library), parallel over z slices.
- [ ] **Step 4: Run** — expected: PASS.
- [ ] **Step 5: Commit + push** `feat(environment): cloud optics, coverage-preserving noise and the CPU cloud light`.

### Task 5: Offline pipeline, analog set, licences, packaging

**Files:**
- Create: `tools/data_pipeline/weather.py` (kind `weather_analog`), `tools/data_pipeline/test_weather.py`
- Modify: `tools/data_pipeline/fetch.py` (dispatch), `sources.toml` (ERA5 / ARCO-ERA5 with the Copernicus licence text
  and attribution "Contains modified Copernicus Climate Change Service information", GFS public domain, lz4, the GRIB2
  library), `CREDITS.md` (regenerated), `tools/data_pipeline/requirements.txt` (if needed), `tools/package.ps1`
  (analog set into a data part), `.gitignore` (`data/weather/`)

**Interfaces:**
- Consumes: Task 3 CLI `weather_snapshot.exe`, Task 2 key format.
- Produces: `data/weather/analog/<cache key>.wthr` - reference year 2025 [call], the 15th of each month at 00/06/12/18
  UTC, downsampled 4x (1 deg), 48 files; an index `data/weather/analog/index.json` (month/hour -> file). The runtime
  analog picks the file of the epoch's month and nearest 6-hour slot (offline); online the exact day may be fetched
  (Task 7).

- [ ] **Step 1: Write the failing pytest** `test_weather.py`: the ERA5 URL for 2025-04-15 12 UTC and variable
  `temperature` equals `<store>/temperature/<t>.0.0.0` with t from the same rule as the core; the analog index lists 48
  entries with unique files.
- [ ] **Step 2: Run** `python -m pytest tools/data_pipeline/test_weather.py` — expected: FAIL.
- [ ] **Step 3: Implement** download (resumable, temp + rename) -> `weather_snapshot.exe --downsample 4` -> analog
  file; raw files under `data/raw/weather/` (deletable). Measure and note the total size (expected ~100-300 MB) and the
  per-hour download (~150 MB) in the commit body.
- [ ] **Step 4: Run** the pytest and `python tools/data_pipeline/fetch.py weather_analog` — expected: PASS, 48 files.
- [ ] **Step 5: Commit + push** `feat(data): ERA5 analog weather set, weather licences and packaging`.

### Task 6: GDExtension access

**Files:**
- Modify: `bindings/src/starsim_core.hpp`, `bindings/src/starsim_core.cpp`, `game/tests/smoke_test.gd`

**Interfaces:**
- Consumes: Tasks 2-4.
- Produces (StarsimCore; all safe on a worker thread; errors German via `push_error`, `ok = false`):
  - `weather_bracket(utc_hour_f: float, step_h: int, era5_zattrs: String, latest_gfs_cycle_hour: int,
    analog_year: int) -> Dictionary {before: Dictionary, after: Dictionary, blend: float}` (snapshot dicts: `kind`,
    `valid_hour`, `cycle_hour`, `lead_h`, `analog_hour`, `cache_key`); `latest_gfs_cycle_hour < 0` and empty zattrs
    mean unknown.
  - `weather_requests(snapshot: Dictionary, idx_text: String) -> Array[Dictionary]` ({url, first_byte, last_byte,
    file_name}); `weather_gfs_index_url(snapshot) -> String`; `weather_gfs_cycle_candidates(wall_utc_hour_f: float,
    count: int) -> PackedInt64Array`; `weather_snapshot_step(warp: float, seconds_per_download: float) -> int`;
    `weather_utc_hour(epoch_tdb_s: float) -> float` (fractional hours since 1900 UTC via the time converter).
  - `weather_build_snapshot(snapshot: Dictionary, raw_dir: String, out_file: String) -> Dictionary {ok, error}`.
  - `weather_load_snapshot(file: String) -> Dictionary {ok, error, rows, columns, layers, texels: PackedByteArray,
    layer_bottoms_m: PackedFloat64Array, label: String}` - keeps it as the active "before" or "after" via
    `weather_set_active(before_file: String, after_file: String, blend: float)` for the CPU queries below.
  - `weather_cloud_light(position_itrf_m: Vector3, sun_direction_itrf: Vector3) -> Dictionary {sun_transmittance:
    float, sky_factor: float}` (blended over the active pair; 1, 1 with weather off).
  - `cloud_noise() -> Dictionary {shape: PackedByteArray, shape_size: int, detail: PackedByteArray, detail_size: int,
    coverage_threshold: PackedFloat32Array}`.
  - `weather_write_slab_snapshot(file: String, bottom_m: float, top_m: float, liquid_water_kg_m3: float,
    cloud_fraction: float) -> bool` (writes `WeatherSnapshot::uniform_slab(181, 360, ...)`; for checks only).

- [ ] **Step 1: Extend smoke_test.gd**: `weather_bracket` for 2024-04-08 18:30 returns era5 with blend 0.5;
  `weather_load_snapshot` of a slab written by `weather_write_slab_snapshot` (unique temp file) has `texels.size() == rows * columns * layers * 4`; `cloud_noise().shape.size() == 4 * 128^3`.
- [ ] **Step 2: Run** `tools/build.ps1 -TestFilter "[weather]"`, then the smoke test — expected: smoke FAILS.
- [ ] **Step 3: Implement** the methods (thin: conversions only).
- [ ] **Step 4: Run** both — expected: PASS.
- [ ] **Step 5: Commit + push** `feat(bindings): weather sources, snapshots, cloud light and noise for the game`.

### Task 7: Weather service in the game (downloads, cache, warp, UI)

**Files:**
- Create: `game/world/weather/weather_service.gd` (`class_name WeatherService`), `game/world/weather/weather_fetcher.gd`
  (HTTPRequest wrapper with `Range`, temp + rename, injectable for tests), `game/tests/weather_service_test.gd`
- Modify: `game/world/world.gd` (create / update the service, hand textures to `CloudLayer` in Task 8),
  `game/ui/webcast/top_bar.gd` (weather line under the local time), `game/ui/time_panel.gd` (checkbox "Wetter",
  persisted in the game's existing settings mechanism - the builder finds it; `user://` if none)

**Interfaces:**
- Consumes: Task 6.
- Produces (`WeatherService`):
  - `func update(epoch_tdb_s: float, warp_factor: float) -> void` (main thread, per frame; non-blocking)
  - `signal snapshots_changed(before: Dictionary, after: Dictionary)` (loaded snapshot dicts from Task 6),
    `func blend() -> float`, `func status_text() -> String` (the German lines of Global Constraints), `var enabled: bool`
  - cache `Paths.data_dir() + "/weather/cache"`, LRU cap 4 GB [call], raw files deleted after a successful build;
    builds run on `WorkerThreadPool`; on start the latest GFS cycle is probed (`.idx` HEAD/GET of the candidates) and
    the ERA5 `.zattrs` fetched once per session. The analog: the shipped 1-deg file at once, then, online, the
    full-resolution ERA5 hour of the same calendar day in the reference year is fetched and blended in. The next
    bracket is prefetched in the background.

- [ ] **Step 1: Write the failing headless test** `weather_service_test.gd` with a fake fetcher serving files from a
  temp dir:
  - archive epoch -> requests the 7 ERA5 files for the two bracketing hours (6 timed + 1 static, once) and emits
    `snapshots_changed` after the builds;
  - the fake returns 404 / a truncated file -> status falls back to "Wetter: typisch (Analogtag ...)", no exception,
    `update()` never waits (Review Focus 1);
  - warp 1e5 -> no new requests, status "Wetter: Stand HH:00 UTC"; warp back to 1 -> the current bracket is requested
    and blended in;
  - `enabled = false` -> "Wetter: aus", textures cleared;
  - LRU: filling past the cap deletes the oldest files.
- [ ] **Step 2: Run** `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script
  res://tests/weather_service_test.gd` — expected: FAIL.
- [ ] **Step 3: Implement** the service, fetcher and UI lines.
- [ ] **Step 4: Run** the test, the smoke test and `webcast_overlay_test.gd` — expected: PASS. One real online run
  (2024-04-08 18:42 UTC at Starbase via `render_check.gd --webcast`) downloads and logs size and time.
- [ ] **Step 5: Commit + push** `feat(game): weather service - downloads, cache, warp policy, source line in the overlay`.

### Task 8: Volumetric clouds (raymarch, near field)

**Files:**
- Create: `game/shaders/clouds.gdshaderinc` (field lookup with blend, layer mapping, density with coverage-preserving
  detail, extinction, phase, multiple-scattering approximation - the port of Task 4; change both together)
- Create: `game/shaders/cloud_layer.gdshader` (full-screen quad, spatial, unshaded, reads the depth texture,
  raymarches the shell 0-20 km between the camera and the scene depth, lit by `atmosphere.gdshaderinc`, composited with
  aerial perspective; first in the transparent pass so plumes draw over it)
- Create: `game/world/weather/cloud_layer.gd` (`class_name CloudLayer`: textures from the snapshots and the noise,
  global uniforms `cloud_field_before/after` (sampler3D), `cloud_blend`, `cloud_noise_shape/detail`,
  `cloud_coverage_threshold`, `cloud_enabled`, quality uniforms; desktop / VR quality presets)
- Create: `game/tests/cloud_check.gd` (GPU: transmittance / radiance at sample pixels vs the core's CPU reference)
- Modify: `game/project.godot` (`[shader_globals]`), `game/world/world.gd`, `game/tests/render_check.gd`
  (`--weather=off|analog|online|<file>`, `--cloud_quality=desktop|vr`; default `off` [call]: existing visual checks
  stay offline and reproducible; the game defaults to weather on)

**Interfaces:**
- Consumes: Tasks 6-7, `atmosphere.gdshaderinc` (sun transmittance, sky irradiance, sky radiance to point), the
  floating-origin uniforms `atmosphere_camera_hi/lo`.
- Produces: `clouds.gdshaderinc` functions `float cloud_density(vec3 p_earth_m)`, `float cloud_sun_transmittance(vec3
  p_earth_m, vec3 sun_dir)` (few samples through the layers, detail fading with distance), used by Task 9 / 10.
  Distance LOD rule: detail noise amplitude fades to 0 by a documented distance (the field mean is coverage-weighted, so
  the faded density integrates to the same optical depth - the no-pop rule).

- [ ] **Step 1: Write `cloud_check.gd`** cases: a synthetic snapshot (uniform 1-km slab, Task 6 helper) seen from below,
  inside and above: transmittance at the centre pixel vs `cloud_sun_transmittance`-style CPU Beer-Lambert within 3 %;
  camera inside the slab (Review Focus 5) finite, non-black; climbing out in 50 m steps changes the centre pixel
  smoothly (no step > 5 % between frames).
- [ ] **Step 2: Run** `godot --path game --script res://tests/cloud_check.gd` (GPU) — expected: FAIL.
- [ ] **Step 3: Implement** the include, the shader and `CloudLayer` (blue-noise jittered start, step count from the
  quality preset and the segment length, early exit at transmittance < 0.01; lit by the transmitted Sun, the sky
  irradiance and, at night, the existing Moon light (`atmosphere_moon_light` / `_direction`); stereo: the jitter pattern is per screen
  pixel and frame, identical for both eyes; no temporal accumulation unless the gate needs it - then it must be
  per-eye reprojected, documented).
- [ ] **Step 4: Run** `cloud_check.gd` — expected: PASS. Render and send PNGs: Starbase on 2024-04-08 18:42 UTC (real
  weather), ascent `--fly=60` through the deck, a view from inside a cloud.
- [ ] **Step 5: Commit + push** `feat(game): volumetric clouds from the weather field, flyable everywhere`.

### Task 9: Clouds in the light (shadows, ambient, vehicle light, adaptation)

**Files:**
- Modify: `game/shaders/terrain_earth.gdshader`, `game/shaders/water.gdshaderinc`, `game/shaders/earth.gdshader`
  (true-space near shading), `game/shaders/lit_aerial.gdshader` - sun term x `cloud_sun_transmittance`, sky term x the
  column's diffuse factor
- Modify: `game/world/atmosphere.gd` (vehicle Sun light and the adaptation luminance x `weather_cloud_light` at the
  followed body / eye; Moon light likewise), `game/world/lit_aerial.gd` if materials need the include
- Test: extend `cloud_check.gd`; `atmosphere_check.gd` must still pass with weather off

**Interfaces:**
- Consumes: Task 8 include, Task 6 `weather_cloud_light`.

- [ ] **Step 1: Extend `cloud_check.gd`**: ground point under the synthetic slab: rendered ground luminance ratio
  (weather on / off) equals the CPU `sun_transmittance` x sun share + `sky_factor` x sky share within 5 %; with weather
  off every Earth-surface pixel is bit-identical to before (guards the atmosphere work).
- [ ] **Step 2: Run** `cloud_check.gd` and `atmosphere_check.gd` — expected: cloud case FAILS, atmosphere PASS.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** both — expected: PASS. Send PNGs: cloud shadows on the Cape coast from 5 km, the pad under an
  overcast deck (dimmer, flat light), a sunset with lit cloud bases.
- [ ] **Step 5: Commit + push** `feat(game): cloud shadows, light under the deck, vehicle light and eye adaptation through clouds`.

### Task 10: Far field (scaled space) without a seam

**Files:**
- Modify: `game/shaders/earth.gdshader` (scaled-space branch: column optical depth, top height and lit top from the
  field - same `clouds.gdshaderinc` lookups with the detail faded to the mean), `game/world/celestial_body.gd` if the
  scaled-space switch needs to tell the cloud layer to stop
- Modify: `game/world/weather/cloud_layer.gd` (raymarch off in scaled space)
- Test: extend `cloud_check.gd`

- [ ] **Step 1: Extend `cloud_check.gd`**: the same Earth point rendered just inside and just outside the scaled-space
  limit (1.5e7 m) - cloud-covered pixel luminance agrees within 5 % (the no-pop rule); a view from the Moon has cloud
  cover where the snapshot has it (sample 5 cells: cloudy vs clear).
- [ ] **Step 2: Run** — expected: FAIL.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** — expected: PASS. Send PNGs: ISS orbit over a frontal band, the Earth from the Moon with clouds,
  the limb from orbit.
- [ ] **Step 5: Commit + push** `feat(game): clouds from the Moon and deep space in the scaled-space Earth`.

### Task 11: Performance gate, visual set, docs

**Files:**
- Modify: `game/tests/terrain_perf.gd` (clouds on at the VR preset; gate p99 <= 11 ms, max <= 16 ms)
- Modify: `docs/subsystems.md` (new section "Weather and clouds": sources, seams, layer form, optics with citations,
  LOD rule, light, VRAM ~270 MB for two snapshots, download sizes, known limits: CPU light uses the grid mean,
  plumes always draw over clouds, earthshine still constant albedo), `CLAUDE.md` (architecture list: `weather_decode`
  and the weather field; Build: `fetch.py weather_analog`; known issues), `docs/ansichten.md` (weather line and
  switch), the spec's status line

- [ ] **Step 1: Run** `terrain_perf.gd` 3 times (GPU) with clouds at the VR preset — expected: p99 <= 11 ms, max
  <= 16 ms in 3 of 3; if not, tune the preset (steps, LOD distance, half-resolution option) - never the physics.
- [ ] **Step 2: Render the visual set** (Validation in the spec) and send the PNGs: overcast vs clear day at Starbase
  (historical dates), ascent below / inside / above a deck, Cape from 100 km, ISS orbit over a front, sunset cloud
  bases, cloud shadows at the Cape, the Earth from the Moon, Antarctica and the Pacific date line (Review Focus 4), the
  2024-04-08 ERA5 field next to the GOES-East image of that hour (pattern position by eye).
- [ ] **Step 3: Run** the affected tests once more: `tools/build.ps1 -TestFilter "[weather],[atmosphere]"`, smoke,
  `weather_service_test.gd`, `cloud_check.gd`, `atmosphere_check.gd`.
- [ ] **Step 4: Write the docs.**
- [ ] **Step 5: Commit + push** `docs: weather and clouds - sources, optics, light, performance, known limits`; then
  ask the user for the VR check (pad under clouds, ascent through clouds, orbit) and for `/clear`.
