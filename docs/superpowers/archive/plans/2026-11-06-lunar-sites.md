# Lunar Landing Sites Anywhere - Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Any latitude / longitude on the Moon becomes a landing site: 60 m terrain + 100 m albedo tile downloaded on
selection, a small list of landing windows with the Sun at touchdown, and an HLS mission (free LLO plane, user-chosen
stay) that flies there - Apollo 11/15/17 as validation cases.

**Architecture:** The core stays network-free: it *plans* the byte ranges of a site tile from a product description
(config) and *assembles* downloaded bytes into the existing `TerrainGridFile` format; the game (and the pipeline via a
core CLI) does the HTTP. `LunarTerrain` becomes "global grid + the active site's tile" behind a tile interface
(geographic, polar north / south). The mission side generalises the polar LLO to a plane chosen per site and window
and adds a window search backed by the real planners; the interim UI lives in the time panel's mission picker.

**Tech Stack:** C++20 core (Catch2), godot-cpp bindings, Godot 4 GDScript, Python data pipeline (unittest).

**Spec:** `docs/superpowers/archive/specs/2026-11-06-lunar-sites-design.md` (approved 2026-11-06). Code facts for the
implementer: the subsystem docs `docs/subsystems/lunar.md`, `moon-surface.md`, `terrain-rendering.md`.

## Global Constraints

- `core` has no networking; HTTP only in GDScript (`WeatherFetcher` pattern) and Python (`lunar_tile._get`).
- Frame / unit in every name; MOON_ME latitude / longitude (the LOLA frame); SI inside, degrees only in config / UI.
- Every refusal is a report code (`core/include/starsim/report/codes.def`) with a `[report]` text in **both**
  `config/i18n/de.toml` and `en.toml`; `python tools/i18n_check.py codes` stays green.
- Every new data source in `tools/data_pipeline/sources.toml` with licence; CREDITS regenerated.
- No baked shading from satellite photos in the rendered surface; albedo tiles only from illumination-normalised products.
- No tuning of mission or vehicle parameters to make a site work (user rule); a refusal with the right reason is a pass.
- Window list: 3-5 entries, horizon ~90 d, Sun above the horizon at touchdown and through the whole stay (hard),
  preferred elevation band 5-20 deg (ranking only), every listed window backed by the real planners.
- Offline + tile not cached -> the selection is refused with a report code (no landing on the 470 m fallback).
- Download and window search never block a frame (HTTPRequest nodes, `WorkerThreadPool` / `FlightSim.post_deferred`).
- Tests: per change `-Fast -TestFilter` of the touched tags, the same tags with `[slow]` before each push; data
  tests carry `[data]` and `SKIP()` when their files are missing (Diviner pattern). New cases > ~40 s cold: `[slow]`.
- Commit + `git push origin main` after every task once its tests are green (Conventional Commits). Update the touched
  `docs/subsystems/*.md` in the same task (current state only; superseded text to `docs/history/`).

## Review Focus

1. **Site near a product boundary** (|lat| ~60 deg, ~80 deg, SLDEM 45-deg file seams, lon 0/360 wrap): the tile is
   assembled from several source files or one product is chosen consistently - Task 3 test `site tile across a file seam and the 0/360 wrap`.
2. **Download interrupted / server answers 200 instead of 206 / partial cache**: no half tile is ever used; retry
   works - Task 11 test `interrupted download leaves no tile and the retry completes`.
3. **A site with no window at all** (far side with budget short, Sun never right for the stay): the answer is an
   empty list with a cause, not a hang or a silent epoch hop - Task 8 test `far-side site with a too-long stay refuses with the daylight cause`.
4. **Re-selecting a site while a search or download for another site runs**: the stale result is discarded, the
   terrain never mixes two sites - Task 11 test `switching site mid-download discards the first site's result`.
5. **Stay longer than the lunar day at the site** (e.g. 10 d at the equator) or zero stay: refused with the
   daylight cause / zero stay works - Task 8 tests `stay longer than daylight` and `zero stay`.

---

### Task 1: Data products - verify and describe (research spike, no game code)

Decides the products every later task reads. Output is config + a findings note, so later tasks never hard-code URLs.

**Files:**
- Create: `config/lunar_terrain_products.toml`, `tools/data_pipeline/probe_lunar_products.py`
- Modify: `tools/data_pipeline/sources.toml`, `config/README.md` (new file documented)
- Findings: `.superpowers/sdd/2026-11-06-lunar-sites/t1-products.md` (ledger workspace)

**Interfaces:**
- Produces `config/lunar_terrain_products.toml`, one `[[product]]` per product:
  `id`, `kind = "height" | "albedo"`, `projection = "simple_cylindrical" | "polar_stereographic_south" | "polar_stereographic_north"`,
  `latitude_min_deg`, `latitude_max_deg` (coverage), `resolution_m`, `sample_type = "int16" | "float32" | "uint16" | "uint8"`,
  `scale`, `offset` (value = raw * scale + offset; heights in m above the 1737.4 km sphere, albedo as I/F or reflectance),
  `[[product.file]]` entries with `url`, `lines`, `samples`, `record_bytes`, and the projection keys
  (`pixels_per_degree`, `first_latitude_deg`, `first_longitude_deg` for cylindrical files; `pole_line`, `pole_sample`,
  `scale_m` for polar), `source` (= a `sources.toml` id). Plus `[selection]`: `height_order` / `albedo_order`
  (product ids, first one whose coverage holds the whole tile wins) and `tile_half_size_m = 75000.0`,
  `albedo_tile_half_size_m = 75000.0`.

- [ ] **Step 1:** Write `probe_lunar_products.py`: for each candidate URL fetch the PDS label (`lunar_tile._label_url`,
  `parse_label`), print lines / samples / sample type / scale / projection, and time one ranged read of 64 rows x
  2,500 samples (sequential and 8 in parallel). Candidates: SLDEM2015 512 ppd float tiles (PDS geosciences
  `lrolol_1xxx/data/sldem2015/tiles/float_img/`), `LDEM_64` (cylindrical, `lola_gdr/cylindrical/img/ldem_64.img`),
  polar `LDEM_80S_20M` / `LDEM_80N_20M`, the polar product covering 60-80 deg at <= 60 m (search the LOLA polar
  directory; else the best available, documented), and an illumination-normalised WAC albedo mosaic at ~100 m
  (LROC RDR `WAC_HAPKE` / `WAC_EMP` normalised 643 nm mosaics - pick one whose label states photometric normalisation).
- [ ] **Step 2:** Run it; record per product: URL, layout, licence, measured throughput, and the download time of a
  150 x 150 km tile at the measured rate (rows as separate ranges, N parallel). If one tile takes > 2 min at the
  equator, propose a smaller `tile_half_size_m` in the findings (the 150 km figure is the spec's estimate).
- [ ] **Step 3:** Write `config/lunar_terrain_products.toml` and the `sources.toml` entries (licence: "NASA PDS data:
  public domain (NASA data and images are not copyrighted)" only after checking each product page). Document in
  `config/README.md`.
- [ ] **Step 4:** `python tools/data_pipeline/build_credits.py` (or the CREDITS generator named in `docs/setup.md`);
  commit `feat(data): lunar terrain and albedo product descriptions` + push.

### Task 2: Core - tile interface, geographic and north-polar tiles

**Files:**
- Create: `core/include/starsim/environment/lunar_tile.hpp`, `core/src/environment/lunar_tile.cpp`
- Modify: `core/include/starsim/environment/lunar_terrain.hpp`, `core/src/environment/lunar_terrain.cpp`,
  `core/src/config/lunar_site_config.cpp` (accept `polar_stereographic_north`, moon `simple_cylindrical` tiles),
  `core/src/sim/lunar_site.cpp`, `core/tests/test_lunar_terrain.cpp`, `docs/subsystems/lunar.md`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::environment {
  class LunarTile {  // one high-resolution patch on the sphere, immutable
  public:
      virtual ~LunarTile() = default;
      virtual bool contains(const math::Vec3& unit_moon_me) const = 0;
      virtual double height_m(const math::Vec3& unit_moon_me) const = 0;      // bilinear, above the sphere
      virtual double edge_distance_m(const math::Vec3& unit_moon_me) const = 0; // >0 inside
      virtual double scale_m() const = 0;
  };
  std::shared_ptr<const LunarTile> make_polar_tile(PolarStereographicTile tile, bool north);
  std::shared_ptr<const LunarTile> make_cylindrical_tile(CylindricalGrid grid, double first_latitude_rad,
                                                         double first_longitude_rad, double sphere_radius_m);
  }
  // LunarTerrain(CylindricalGrid global, std::vector<std::shared_ptr<const LunarTile>> tiles, double sphere_radius_m)
  ```
  `tile_blend`, `in_tile`, `finest_resolution_within_m` keep their signatures and the 1 km smoothstep (`blend_m`).

- [ ] **Step 1: Failing tests** in `test_lunar_terrain.cpp` tagged `[environment][lunar][terrain]`:
  `north polar tile mirrors the south one` (synthetic grid, height at (+85, 30) equals the south tile's at (-85, -30)
  mirror within 1e-9 m); `cylindrical tile samples bilinearly and blends to the global grid over 1 km` (synthetic
  tile at 0 N 23 E, weight 1 at 2 km inside the edge, 0 outside, 0.5 at mid-blend); `cylindrical tile across the
  0/360 seam` (tile centred on lon 0 returns the same height at lon -0.01 deg and 359.99 deg).
- [ ] **Step 2:** Run `tools/build.ps1 -NoBindings -Fast -TestFilter "[environment][lunar][terrain]"` -> FAIL (no `LunarTile`).
- [ ] **Step 3:** Implement; `PolarStereographicTile` gains a hemisphere sign; existing south tile behaviour unchanged.
- [ ] **Step 4:** Same command -> PASS; then `-Fast -TestFilter "[terrain],[lunar]"` green (existing site still loads).
- [ ] **Step 5:** Commit `refactor(environment): lunar tiles behind an interface; north polar and cylindrical tiles` + push.

### Task 3: Core - site tile request planning and assembly (+ CLI)

**Files:**
- Create: `core/include/starsim/config/lunar_site_tile.hpp`, `core/src/config/lunar_site_tile.cpp`,
  `core/apps/lunar_site_tile.cpp` (CLI, built like `weather_snapshot`), `core/tests/test_lunar_site_tile.cpp`
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`, `core/src/config/lunar_site_config.cpp` (writer)

**Interfaces:**
- Consumes: `config/lunar_terrain_products.toml` (Task 1), `TerrainGridFile`.
- Produces:
  ```cpp
  namespace starsim::config {
  struct ByteRange { std::size_t file_index; std::string url; std::uint64_t first, last; };  // inclusive
  struct SiteTileRequest {
      std::string product_id, kind;            // "height" | "albedo"
      double latitude_rad, longitude_rad, half_size_m;
      std::vector<ByteRange> ranges;           // rows of the crop, adjacent rows merged
      std::string cache_key;                   // "<kind>_<product>_<lat 0.01 deg>_<lon 0.01 deg>_<half km>"
  };
  std::vector<LunarTerrainProduct> load_lunar_terrain_products(const std::filesystem::path& config_dir);
  SiteTileRequest plan_site_tile(const std::vector<LunarTerrainProduct>&, std::string_view kind,
                                 double latitude_rad, double longitude_rad);  // throws Refusal lunar.site_no_product
  TerrainGridFile assemble_site_tile(const SiteTileRequest&, const std::vector<std::vector<std::uint8_t>>& bodies);
  void write_terrain_grid_file(const TerrainGridFile&, const std::filesystem::path& toml_path);  // + .i16 next to it
  }
  ```
  Height tiles are written as int16 DN with `dn_scale_m = 0.5` (as today) and 9 `[[check]]` points; albedo tiles as
  uint16 *relative* albedo: tile / its own 10 km low-pass, 32768 = 1.0 (keeps the global mosaic's calibration; the
  shader multiplies). Geographic tiles keep the source grid (no reprojection). CLI: `lunar_site_tile plan <config_dir>
  <kind> <lat_deg> <lon_deg>` prints the request as JSON; `lunar_site_tile assemble <config_dir> <request.json>
  <body_dir> <out.toml>`.

- [ ] **Step 1: Failing tests** `[config][lunar_site_tile]`: `plan picks SLDEM at Apollo 15 and the north polar
  product at 85 N` (product ids from a test products file); `ranges cover exactly the crop rows, merged when
  adjacent` (count = crop lines for a 512 ppd file); `site tile across a file seam and the 0/360 wrap` (two files'
  ranges, assembled heights continuous); `assemble then load_terrain_grid_file round-trips heights within 0.25 m`;
  `albedo tile is relative: mean 1.0 within 1e-3 over the tile`; `no product for the site refuses with lunar.site_no_product`.
- [ ] **Step 2:** Run `-NoBindings -Fast -TestFilter "[config][lunar_site_tile]"` -> FAIL.
- [ ] **Step 3:** Implement; new report code `lunar_site_no_product(latitude, longitude)` (de/en texts).
- [ ] **Step 4:** PASS; `python tools/i18n_check.py codes` green.
- [ ] **Step 5:** Commit `feat(config): plan and assemble lunar site tiles from product descriptions` + push.

### Task 4: Data - LDEM_64 global, Apollo presets, pipeline through the CLI

**Files:**
- Modify: `tools/data_pipeline/lunar_tile.py`, `fetch.py`, `sources.toml`, `test_lunar_tile.py`,
  `config/lunar_sites.toml`, `config/i18n/{de,en}.toml` (`[lunar_site]` names), `core/src/sim/lunar_site.cpp`
  (global grid file name from config), `docs/setup.md` (data parts), `docs/subsystems/lunar.md`

**Interfaces:**
- Consumes: CLI from Task 3. Produces: `data/terrain/ldem_64.{i16,toml}` (replaces `ldem_16`, which is removed from
  the pipeline); preset tiles `data/terrain/sites/<cache_key>.{i16,toml}` (height + albedo) for every `[[site]]`;
  `config/lunar_sites.toml` `[[site]]` keys become `id, latitude_deg, longitude_deg` (+ `[sources]` refs); product and
  half size come from `[selection]` of Task 1. Sites (coordinates from the LROC NAC landing-site reference, recorded
  in `[sources]`): `apollo_11` 0.67416 N 23.47314 E, `apollo_15` 26.13222 N 3.63386 E, `apollo_17` 20.19089 N
  30.77168 E (verify each against the cited source; correct and note if they differ), `mons_mouton_im2` unchanged.
  `fetch.py lunar_test_sites` (opt-in) also caches the two free test sites of Task 13: Tsiolkovskiy (20.4 S 129.1 E)
  and 75.0 N 30.0 E.

- [ ] **Step 1: Failing tests** in `test_lunar_tile.py`: `fetch_site_tiles builds requests via the CLI for every
  site` (CLI and `_get` mocked; four sites -> eight requests); `old ldem_16 config entry is gone`.
- [ ] **Step 2:** `python -m unittest tools/data_pipeline/test_lunar_tile.py` -> FAIL.
- [ ] **Step 3:** Implement; `LDEM_64` is a data part (setup.md), `fetch.py` downloads it whole.
- [ ] **Step 4:** Unit tests PASS; run `python tools/data_pipeline/fetch.py lola_ldem_64 lunar_sites` for real, then
  `-NoBindings -TestFilter "[terrain],[lunar]"` (with `[slow]`): re-baseline only values that move because of the
  470 m global grid, each with its reason in the commit body; any red that is not explained by resolution is a defect.
- [ ] **Step 5:** Commit `feat(data): LDEM_64 global lunar grid and Apollo 11/15/17 site tiles` + push.

### Task 5: Core - the active site: free coordinates, terrain per site

**Files:**
- Modify: `core/include/starsim/config/lunar_site_config.hpp`, `core/src/config/lunar_site_config.cpp`,
  `core/include/starsim/sim/lunar_site.hpp`, `core/src/sim/lunar_site.cpp`, `core/include/starsim/sim/terrain_cache.hpp`,
  `core/src/sim/terrain_cache.cpp`, `core/src/sim/lunar_landing_mission.cpp`, `core/tests/test_lunar_terrain.cpp`,
  `core/tests/test_terrain_cache.cpp`, `codes.def`, i18n tables

**Interfaces:**
- Produces:
  ```cpp
  // config: a site is a place, not a list entry
  LunarSiteConfig lunar_site_at(double latitude_rad, double longitude_rad);  // id "site_<lat>_<lon>" (0.01 deg)
  // sim
  std::shared_ptr<const environment::LunarTerrain> load_lunar_terrain(const fs::path& config_dir, const fs::path& data_dir,
                                                                      const config::LunarSiteConfig& site);
  std::shared_ptr<const environment::DetailedLunarTerrain> shared_moon_surface(config_dir, data_dir, options,
                                                                               const config::LunarSiteConfig& site);
  bool lunar_site_tiles_present(const fs::path& config_dir, const fs::path& data_dir, const config::LunarSiteConfig&);
  ```
  Tiles are looked up in `data/terrain/sites/` by the Task 3 `cache_key` (preset and downloaded alike). Missing
  height tile -> `DataMissing` with new code `data.lunar_site_tile_missing(latitude, longitude)`. The no-site overload
  (global only) stays for orbit-only sessions.

- [ ] **Step 1: Failing tests** `[sim][lunar][terrain][data]`: `terrain for Apollo 11 uses its 60 m tile`
  (`finest_resolution_within_m` <= 61 at the site); `two sites give two terrains, the cache keeps both alive while
  held`; `free coordinates without a cached tile raise data.lunar_site_tile_missing`.
- [ ] **Step 2:** `-NoBindings -Fast -TestFilter "[sim][lunar][terrain]"` -> FAIL.
- [ ] **Step 3:** Implement; `lunar_landing_mission.cpp` `load_site` uses the mission's site config (override in Task 9).
- [ ] **Step 4:** PASS; `-Fast -TestFilter "[lunar],[terrain]"` green.
- [ ] **Step 5:** Commit `feat(sim): lunar terrain per active site from free coordinates` + push.

### Task 6: Core - Sun at the site and daylight intervals

**Files:**
- Create: `core/include/starsim/mission/lunar_site_sun.hpp`, `core/src/mission/lunar_site_sun.cpp`,
  `core/tests/test_lunar_site_sun.cpp`
- Modify: `tools/data_pipeline/sources.toml` (Apollo mission report Sun elevations), `docs/subsystems/lunar.md`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::mission {
  struct SiteSun { double elevation_rad, azimuth_rad; };  // azimuth from north, east positive; Sun centre, apparent
  SiteSun site_sun(const ephemeris::Ephemeris&, const frames::BodyOrientation& moon, double latitude_rad,
                   double longitude_rad, time::Epoch);
  struct DaylightInterval { time::Epoch sunrise, sunset; };  // Sun centre crosses the geometric horizon
  std::vector<DaylightInterval> daylight_intervals(const ephemeris::Ephemeris&, const frames::BodyOrientation&,
                                                   double latitude_rad, double longitude_rad,
                                                   time::Epoch from, double span_s);  // root-bracketing, 10 s accuracy
  }
  ```
  Polar sites with no sunrise / sunset in the span return one interval clipped to the span (or none if dark).

- [ ] **Step 1: Failing tests** `[mission][lunar][lunar_sun]`: `Sun elevation at the Apollo 11 / 15 / 17 touchdowns
  matches the mission reports within 1.0 deg` (touchdown UTC 1969-07-20 20:17:40, 1971-07-30 22:16:29,
  1972-12-11 19:54:57; reference elevations from the mission reports, cited in `sources.toml` - fill in the values
  the reports give); `equatorial daylight lasts 14.77 +- 0.3 d`; `Mons Mouton Sun never exceeds 7 deg over 60 d`.
- [ ] **Step 2:** `-NoBindings -Fast -TestFilter "[lunar_sun]"` -> FAIL.
- [ ] **Step 3:** Implement with light time + aberration as the renderer does (`CLAUDE.md`: apparent positions).
- [ ] **Step 4:** PASS. **Step 5:** Commit `feat(mission): Sun at a lunar site and its daylight intervals` + push.

### Task 7: Core - free LLO plane: outbound, descent, ascent (physics-critical, Opus)

**Files:**
- Modify: `core/include/starsim/mission/lunar_outbound.hpp` (+ `.cpp`), `core/include/starsim/gnc/lunar_ascent_guidance.hpp`
  (+ `.cpp`), `core/src/sim/lunar_landing_mission.cpp`, `core/include/starsim/config/mission_config.hpp` (+ `.cpp`),
  `config/missions/hls_lunar_*.toml`, `core/tests/test_lunar_outbound.cpp`, `test_lunar_ascent.cpp`,
  `test_lunar_landing_mission.cpp`, `docs/subsystems/lunar.md`, `docs/history/lunar.md`

**Interfaces:**
- Produces:
  ```cpp
  // LloTarget: the plane is chosen, not polar
  struct LloTarget { double altitude_m = 100e3; std::optional<math::Vec3> plane_normal_moon_inertial; double inclination_rad = pi/2; };
  // the plane that holds the site at touchdown (and is closest to the arrival plane of least LOI cost)
  math::Vec3 llo_plane_through_site(const frames::BodyOrientation& moon, double latitude_rad, double longitude_rad,
                                    time::Epoch touchdown, const math::Vec3& arrival_asymptote_moon_inertial);
  // ascent: plane through the site at liftoff, given by its normal (replaces `northerly`)
  struct LunarAscentParameters { /* ... */ std::optional<math::Vec3> plane_normal_moon_inertial; };
  ```
  `[lunar_orbit] inclination_deg` becomes optional: absent = chosen per site / window (`hls_lunar_llo*.toml` drop it;
  keep the key working for explicit missions). The ascent plane = the plane through the site at liftoff closest to
  the landing LLO plane (least ascent plane change); the descent planner's existing site-pass search and plane trim
  stay the mechanism for the residual crossrange.

- [ ] **Step 1: Failing tests:** `[mission][lunar][outbound][validation]` `LLO for Apollo 11 at 0.7 N passes the
  site within 20 km at touchdown` and `... Apollo 15 at 26 N`; `[gnc][lunar_ascent]` `ascent from 20 N inserts
  into the requested plane within 0.05 deg`; `[sim][lunar][mission]` `mons mouton mission unchanged: inclination
  within 1 deg of polar and the same feasibility` (regression of today's case).
- [ ] **Step 2:** `-NoBindings -Fast -TestFilter "[lunar][outbound],[lunar_ascent],[sim][lunar][mission]"` -> FAIL.
- [ ] **Step 3:** Implement. The plane is the free parameter (user: "die Neigung des Orbits kann frei gewaehlt
  werden"); the outbound planner targets it at LOI; NRHO: unchanged orbit, the descent from it is allowed only when
  the budget closes, else `lunar.nrho_site_unreachable(latitude, budget_short)` (new code).
- [ ] **Step 4:** PASS, then the same tags with `[slow]` (incl. `[lunar_mission_flight]`, `[lunar_campaign]` seeds as
  today - the Mons Mouton campaign must not regress).
- [ ] **Step 5:** Commit `feat(mission): LLO plane chosen per site and window; ascent into a given plane` + push.

### Task 8: Core - landing window search (Opus)

**Files:**
- Create: `core/include/starsim/sim/lunar_windows.hpp`, `core/src/sim/lunar_windows.cpp`, `core/tests/test_lunar_windows.cpp`
- Modify: `core/include/starsim/config/mission_config.hpp` (+ `.cpp`), `config/missions/hls_lunar_*.toml` (`[windows]`),
  `codes.def`, i18n tables, `config/README.md`, `docs/subsystems/lunar.md`

**Interfaces:**
- Consumes: Tasks 5-7. Produces:
  ```cpp
  namespace starsim::sim {
  struct LunarWindowRequest {
      std::string mission_id; config::LunarSiteConfig site; double stay_s; time::Epoch start; };
  struct LunarLandingWindow {
      time::Epoch mission_start, touchdown, liftoff;
      double sun_elevation_rad, sun_azimuth_rad, daylight_after_touchdown_s;
      double propellant_reserve_kg, llo_inclination_rad; mission::LunarOrbitKind orbit; };
  struct LunarWindowResult { std::vector<LunarLandingWindow> windows; std::optional<report::Report> refusal; };
  // runs on the caller's thread (minutes allowed); progress 0..1 through the callback; cancel when it returns false
  LunarWindowResult find_lunar_landing_windows(const FlightSession& prototype, const fs::path& config_dir,
                                               const LunarWindowRequest&, const std::function<bool(double)>& progress);
  }
  ```
  `[windows]` per mission: `horizon_days = 90`, `count = 5`, `sun_preferred_min_deg = 5`, `sun_preferred_max_deg = 20`.
  Algorithm: daylight intervals (Task 6) over the horizon -> touchdown candidates every 6 h inside intervals that
  still hold `stay_s` + the ascent lead -> rank by distance to the preferred band, ties chronological -> for each in
  order run the real chain (mission start epoch = touchdown - transfer - descent lead; `plan_lunar_outbound`,
  descent plan, `lunar_mission_budget`) until `count` pass. Refusal = most frequent failure cause across candidates
  as `lunar.no_window(cause)` (causes: `lunar.no_daylight_for_stay`, `lunar_budget_short`, `lunar.no_site_pass`,
  `lunar.nrho_site_unreachable`). Results cached in memory per (mission, site key, stay, start rounded to 1 h).

- [ ] **Step 1: Failing tests** `[sim][lunar][lunar_windows]` (+ `[terrain][data]`, `[slow]` where > 40 s):
  `Apollo 11 lists 3-5 windows, each with Sun above the horizon, sorted by preference`; `every listed Apollo 15
  window flies: start the mission at window 1 and the descent plan's touchdown is within 1 h of the listed one`;
  `far-side site with a too-long stay refuses with the daylight cause` (Tsiolkovskiy, stay 16 d);
  `stay longer than daylight` (equator, 15 d -> `lunar.no_daylight_for_stay`); `zero stay` lists windows;
  `progress reaches 1 and cancel stops early`.
- [ ] **Step 2:** `-NoBindings -Fast -TestFilter "[lunar_windows]"` -> FAIL.
- [ ] **Step 3:** Implement. **Step 4:** PASS (with `[slow]`). Log the measured search time per site in the ledger.
- [ ] **Step 5:** Commit `feat(sim): lunar landing window search backed by the mission planners` + push.

### Task 9: Core - start a lunar mission at a chosen site, window and stay

**Files:**
- Modify: `core/include/starsim/sim/mission_start.hpp` (+ `.cpp`), `core/src/sim/lunar_landing_mission.cpp`,
  `core/tests/test_lunar_landing_mission.cpp`, `docs/subsystems/lunar.md`

**Interfaces:**
- Produces: `struct LunarStartOverride { config::LunarSiteConfig site; double stay_s; time::Epoch mission_start; };`
  and `StartOptions::lunar` (`std::optional<LunarStartOverride>`); `start_mission` / `start_mission_deferred` use it
  for `lunar_landing` missions instead of `[mission] site`, `[surface] stay_hours` and `[orbit] epoch_utc`. Other
  kinds ignore it.
- [ ] **Step 1: Failing test** `[sim][lunar][lunar_mission_flight][terrain][data][slow]` `Apollo 17 window 1 with a
  72 h stay flies to touchdown, stays 72 h and lifts off into the LLO`.
- [ ] **Step 2:** FAIL. **Step 3:** Implement. **Step 4:** PASS; `[sim][lunar][mission]` fast set green.
- [ ] **Step 5:** Commit `feat(sim): lunar landing missions take site, stay and window from the start options` + push.

### Task 10: Bindings

**Files:**
- Modify: `bindings/src/starsim_core.{hpp,cpp}`, `bindings/src/flight_sim.{hpp,cpp}`, `bindings/src/report_dict.cpp` (if needed)

**Interfaces:**
- Produces (GDScript):
  `StarsimCore.lunar_site_tile_requests(lat_deg, lon_deg) -> Array[Dictionary]` (one per kind: `{kind, cache_key,
  ranges: [{url, first, last}], ok, report}`); `StarsimCore.assemble_lunar_site_tile(request: Dictionary, bodies:
  Array[PackedByteArray]) -> Dictionary{ok, report}` (writes into `data/terrain/sites/`; callable from a worker);
  `StarsimCore.lunar_site_tiles_present(lat_deg, lon_deg) -> bool`; `StarsimCore.set_lunar_site(lat_deg, lon_deg)
  -> Dictionary{ok, report}` (rebuilds moon terrain + surface; worker-callable, swaps atomically);
  `FlightSim.find_lunar_windows(mission_id, lat_deg, lon_deg, stay_s, start_tdb_s) -> int` (deferred command id;
  `result(id)` -> `{done, ok, report, progress, windows: [{mission_start_tdb_s, touchdown_tdb_s, liftoff_tdb_s,
  sun_elevation_deg, sun_azimuth_deg, daylight_after_touchdown_s, propellant_reserve_kg, llo_inclination_deg, orbit}]}`);
  `FlightSim.cancel(id)`; `FlightSim.start_mission(..., options: Dictionary = {})` with `lunar = {lat_deg, lon_deg,
  stay_s, mission_start_tdb_s}`.
- [ ] **Step 1:** Extend `game/tests/smoke_test.gd` `_check_lunar` with: requests for Apollo 11 non-empty; window
  search for Apollo 11 returns `ok` and >= 3 windows (data present) else skip message.
- [ ] **Step 2:** Run the smoke test -> FAIL. **Step 3:** Implement. **Step 4:** `tools/build.ps1 -TestFilter "[lunar_windows]"` + smoke test PASS.
- [ ] **Step 5:** Commit `feat(bindings): lunar site tiles, site switch and window search` + push.

### Task 11: Game - site service (download, cache, switch) and site albedo in the shader

**Files:**
- Create: `game/world/terrain/lunar_site_service.gd`, `game/tests/lunar_site_service_test.gd`
- Modify: `game/world/weather/weather_fetcher.gd` (progress signal `progress(id, bytes)`), `game/world/world.gd`
  (`switch_lunar_site(lat_deg, lon_deg)` rebuilds `moon_terrain` quadtree, rock layer, near shadow),
  `game/shaders/terrain_moon.gdshader`, `game/shaders/moon_rock.gdshader` (uniforms `site_albedo_texture`,
  `site_albedo_bounds` (vec4 lat/lon min/max rad), `site_albedo_blend_m`), `docs/subsystems/terrain-rendering.md`

**Interfaces:**
- Produces: `class_name LunarSiteService extends Node`; `setup(core, fetcher: Object, parallel := 8)`;
  `func prepare(lat_deg: float, lon_deg: float) -> void`; signals `progress(fraction: float)`,
  `ready(lat_deg, lon_deg)`, `failed(report: Dictionary)`; `cancel()`. Uses cached tiles without network; partial
  downloads live as `.part` until assembled; a new `prepare` cancels the running one (generation counter). Offline +
  missing -> `failed` with `data.lunar_site_tile_missing`. Site albedo = relative tile multiplied into the global
  albedo, blended to 1.0 over `site_albedo_blend_m = 5000` at the tile edge.
- [ ] **Step 1: Failing tests** (`FakeFetcher` as in `weather_service_test.gd`): `cached site prepares without a
  request`; `download progresses, assembles and emits ready`; `interrupted download leaves no tile and the retry
  completes`; `switching site mid-download discards the first site's result`; `offline missing tile fails with the code`.
- [ ] **Step 2:** `tools/run_godot_tests.ps1 -Only lunar_site_service_test` -> FAIL.
- [ ] **Step 3:** Implement service + `world.gd` switch + shader blend. **Step 4:** PASS; `weather_fetcher_test` and
  `weather_service_test` still green; render check at Apollo 17 (`res://tests/render_check.gd`, GPU) shows the dark
  mantle in the valley without a seam -> PNG to `screenshots/lunar_sites/`, send to the user.
- [ ] **Step 5:** Commit `feat(game): lunar site service - download, cache, terrain switch, site albedo` + push.

### Task 12: Game - interim site / stay / window picker

**Files:**
- Modify: `game/ui/time_panel.gd`, `game/ui/mission_catalogue.gd` (`is_lunar_landing(mission_id)`),
  `game/vehicle/flight_controller.gd` + `game/world/world.gd` (`start_mission(..., options)`),
  `config/i18n/{de,en}.toml` (`[ui]` keys `ui.lunar_site.*`), `game/tests/mission_catalogue_test.gd`,
  `docs/ansichten.md`

**Interfaces:**
- Consumes: Tasks 10-11. Flow for a `lunar_landing` mission: preset list (`config/lunar_sites.toml`, names from
  `[lunar_site]`) or "Koordinaten" with latitude / longitude fields (deg, N/E positive) + stay (hours, default from
  the mission's `stay_hours`) -> "Termine suchen" -> `LunarSiteService.prepare` progress -> `find_lunar_windows`
  progress -> list (date UTC, Sun elevation / azimuth, daylight after touchdown, reserve) or the refusal text ->
  choose -> start with `options.lunar`.
- [ ] **Step 1: Failing tests:** `mission_catalogue_test` `hls_lunar_llo is a lunar landing, ascent is not`;
  new `game/tests/lunar_picker_test.gd` (headless, fake service + fake sim): `invalid latitude 95 is rejected
  before any request`; `the window list shows the refusal text when no window exists`; `start passes the chosen window`.
- [ ] **Step 2:** FAIL. **Step 3:** Implement; all texts via `I18n.text`. **Step 4:** PASS; `python tools/i18n_check.py`
  green; GPU screenshot of the picker with the Apollo 15 list -> `screenshots/lunar_sites/`, send to the user.
- [ ] **Step 5:** Commit `feat(ui): pick a lunar site, stay and landing window` + push.

### Task 13: Validation - Apollo 11/15/17, far side, high north; look against photos; docs

**Files:**
- Modify: `core/tests/test_lunar_landing_campaign.cpp` (site-parameterised campaign), `core/tests/test_lunar_windows.cpp`,
  `tools/data_pipeline/sources.toml` (photo references, not shipped), `docs/subsystems/{lunar,moon-surface,terrain-rendering}.md`,
  `docs/architecture.md`, `docs/known-issues.md`, `docs/setup.md`

- [ ] **Step 1: Tests:** campaigns `[sim][lunar_campaign][validation][terrain][data][slow]` one seed per site
  (Apollo 11, 15, 17, Tsiolkovskiy 20.4 S 129.1 E, 75 N 30 E) at the first listed window, 24 h stay - each lands
  within the existing landing gates or the window search refused it with a cause (then the test asserts the cause);
  `[nightly]` 7 seeds per Apollo site.
- [ ] **Step 2:** Run the `[slow]` set; run `-Nightly -TestFilter "[lunar_campaign]"` in the background (output to a
  file, check every 10 min). Reds that are planner defects are fixed (user rule: the planner absorbs it); none are tuned away.
- [ ] **Step 3: Look:** render each Apollo site at its historical Sun elevation / azimuth (Task 6 values) from the
  LM-window view and from ~15 km; compare side by side with AS11-40-5903, AS15-86-11603, AS17-140-21497 and the LROC
  NAC landing-site images (km-scale relief, albedo patterns); PNGs to `screenshots/lunar_sites/`, send to the user;
  findings into `moon-surface.md` Validation.
- [ ] **Step 4:** Docs: current state in the subsystem files, the data part in `setup.md`, open points in `known-issues.md`.
- [ ] **Step 5:** Commit `test(lunar): Apollo, far-side and high-north landing campaigns; docs` + push.
