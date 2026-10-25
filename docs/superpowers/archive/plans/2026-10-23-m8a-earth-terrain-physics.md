# M8 Plan A: Earth terrain, geoid and site heights (physics) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Earth gets a physical surface - geoid sea level, a global coarse relief, regional 30 m and local 1 m
terrain around the pads with flat pad plateaus - and every launch site sits at its data-derived height; the
reference flights are re-validated on it.

**Architecture:** The Python pipeline turns EGM2008, ETOPO 2022, Copernicus GLO-30 and USGS 3DEP into simple grids
(TOML header + raw int16, like the lunar tiles) with WGS84 ellipsoidal heights. The core gets an abstract
`environment::Terrain` (implemented by the existing `LunarTerrain` and a new `EarthTerrain`), a `Geoid`, and a
committed 1-degree geoid that every `FlightSession` loads, so tests stay independent of the downloaded data. The game
session loads the full `EarthTerrain` when the data are present; the interim `GroundPatch` samples it.

**Tech Stack:** C++20 core (Catch2, CMake/Ninja), Python 3 pipeline (numpy, rasterio, pyproj, requests), Godot 4 GDScript.

**Spec:** `docs/superpowers/specs/2026-10-23-m8-terrain-tiles-design.md` (this plan = "Plan A - physics first").

## Global Constraints

- Only free data; every source with its licence in `tools/data_pipeline/sources.toml`, `CREDITS.md` regenerated
  (`python tools/data_pipeline/credits.py`). Licence check before adding: EGM2008 (NGA, public), ETOPO 2022 (NOAA,
  public domain), Copernicus GLO-30 (free with attribution "produced using Copernicus WorldDEM-30 (c) DLR e.V.
  2010-2014 and (c) Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA;
  all rights reserved"), USGS 3DEP (public domain). A source whose licence forbids redistribution is not used.
- Height names carry their reference: `height_ellipsoid_m`, `height_orthometric_m`, `geoid_undulation_m`. All core
  grids hold WGS84 ellipsoidal heights except the geoid grids (undulation N). Orthometric = ellipsoidal - N.
- No magic numbers: every new constant `[S]/[E]/[D]` with source or reason, physical constants in `constants.hpp`.
- Tests must not need `data/`: tests reading downloaded files are tagged `[data]` (and `[slow]` if > ~40 s).
- Test workflow (CLAUDE.md): run only the affected tags via `tools/build.ps1 -NoBindings -TestFilter "[a],[b]"`;
  runs that may exceed 10 min go to the background with logs in `tmp/`.
- No widened tolerances. Moved numbers are re-baselined with the reason in `docs/subsystems.md`; a catch or landing
  that fails after the height change is a bug to fix (spec "Out-of-tolerance after the shift").
- C++ UTF-8, UI/exception strings German, code/comments/docs English. Conventional Commits; commit and
  `git push origin main` after every task once its tests are green.
- Progress ledger: `.superpowers/sdd/2026-10-23-m8a-terrain/progress.md` (done, open, decisions, next step).

## Review Focus

- Data missing (fresh checkout, no `fetch.py` run): game and tests run exactly as before on the flat model + coarse
  geoid, with one German log line in the game - Task 5 (binding) and Task 4 (no terrain set) pin it.
- A site moved in `launch_sites.toml` after the download: the loader must refuse stale tiles/plateaus like
  `load_lunar_terrain` does - Task 3 test `stale local tile is refused`.
- Point exactly on a grid seam / antimeridian / pole for the global grids: no NaN, wraps - Task 1 and Task 3 tests.
- Sea vs land decision at the coastline next to a pad (LZ-40 ~650 m from the shore): water must not appear on the
  plateau, a lagoon at 0 m orthometric is water - Task 3 tests `plateau is never water`, `zero orthometric is water`.
- Crew escape / impact far inland with terrain set: touchdown on the global relief, not on the pad height - Task 4
  test `impact on terrain away from the site`.

---

### Task 1: Geoid (pipeline + core + committed 1-degree grid)

**Files:**
- Create: `core/include/starsim/environment/geographic_grid.hpp`, `core/src/environment/geographic_grid.cpp`
- Create: `core/include/starsim/environment/geoid.hpp`, `core/src/environment/geoid.cpp`
- Create: `core/include/starsim/config/geoid_config.hpp`, `core/src/config/geoid_config.cpp`
- Create: `tools/data_pipeline/geoid.py`, `tools/data_pipeline/test_geoid.py`
- Create: `config/geoid/egm2008_1deg.csv` (generated, committed), `tests/validation/geoid/egm2008_points.csv`
- Modify: `tools/data_pipeline/fetch.py` (kind `geoid`), `tools/data_pipeline/sources.toml`, `config/README.md`
- Test: `core/tests/test_geoid.cpp`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::environment {
  // Node-registered latitude/longitude grid: line 0 at north_latitude_deg, sample 0 at west_longitude_deg,
  // lines go south. global_longitude: samples span 360 deg and wrap.
  struct GeographicGrid {
      HeightGrid grid;                    // from lunar_terrain.hpp (int16 x dn_scale_m)
      double north_latitude_deg = 0.0, west_longitude_deg = 0.0;
      double step_latitude_deg = 0.0, step_longitude_deg = 0.0;
      bool global_longitude = false;
      bool covers(double latitude_rad, double longitude_rad) const;
      double value(double latitude_rad, double longitude_rad) const;   // bilinear; lat clamped, lon wrapped when global
      double edge_distance_m(double latitude_rad, double longitude_rad, double radius_m) const;  // inside: >= 0
  };
  class Geoid {
  public:
      explicit Geoid(GeographicGrid undulation);
      double undulation_m(double latitude_rad, double longitude_rad) const;   // EGM2008 N above WGS84
  };
  }
  namespace starsim::config {
  environment::Geoid load_coarse_geoid(const std::filesystem::path& config_dir);   // config/geoid/egm2008_1deg.csv
  }
  ```
  Grid files (`data/geoid/egm2008_25.toml` + `.i16`, later the terrain grids): the lunar `[grid]` header with
  `projection = "geographic"`, `north_latitude_deg`, `west_longitude_deg`, `step_latitude_deg`, `step_longitude_deg`,
  `global_longitude`, `height_reference = "egm2008_undulation" | "wgs84_ellipsoid"`, `[[check]]` points; extend
  `config::TerrainGridFile` / `load_terrain_grid_file` with these fields.
- Coarse CSV: header comment (source, generation command, licence), then 181 rows (90 N .. 90 S) x 361 values
  (180 W .. 180 E inclusive), undulation in cm as integers.

- [ ] **Step 1: Pipeline tests** (`test_geoid.py`): `test_downsample_1deg_takes_node_values` (a synthetic 2.5' array:
  the 1-degree grid takes the nodes at whole degrees, not averages), `test_csv_round_trip` (write + parse gives the
  same cm values, 181 x 361), `test_grid_header_fields` (projection, steps, `global_longitude = true`).
- [ ] **Step 2: Implement `geoid.py`**: download `https://cdn.proj.org/us_nga_egm08_25.tif` (EGM2008 2.5', NGA,
  public; record in `sources.toml` with `kind = "geoid"`), write `data/geoid/egm2008_25.toml/.i16` (dn 0.01 m) with
  `[[check]]` points, and `--write-config` regenerates `config/geoid/egm2008_1deg.csv`. Run the pytest file: PASS.
- [ ] **Step 3: Core tests** (`test_geoid.cpp`, tags `[environment][geoid]`):
  - `geographic grid wraps the antimeridian` (value at lon +179.99 deg and -179.99 deg differ by less than one
    cell's slope; lon 180 = lon -180 exactly),
  - `geographic grid clamps the poles` (lat +-90 deg finite),
  - `coarse geoid matches EGM2008 reference points`: for each row of `tests/validation/geoid/egm2008_points.csv`
    (>= 8 points: Starbase, LC-39A, SLC-40, the drone-ship areas, `pacific_san_diego`, two open-ocean points, values
    from NGA's EGM2008 point calculator or the 2.5' grid with source noted) `|N - ref| <= 2.0 m` over ocean/coast
    [E: 1-degree bilinear interpolation error of EGM2008 near coasts; record the measured maximum in the file],
  - `[data]` case `fine geoid matches EGM2008 reference points` with `<= 0.05 m`.
- [ ] **Step 4: Run** `tools/build.ps1 -NoBindings -TestFilter "[geoid]"`: the new cases fail to compile/fail.
- [ ] **Step 5: Implement** `GeographicGrid`, `Geoid`, CSV loader, the `TerrainGridFile` extension.
- [ ] **Step 6: Run** `-TestFilter "[geoid],[terrain],[config]"`: PASS (lunar terrain loading unchanged).
- [ ] **Step 7: Commit + push** `feat(environment): EGM2008 geoid with a committed 1-degree grid`.

### Task 2: Earth height data pipeline (ETOPO, GLO-30, 3DEP)

**Files:**
- Create: `config/earth_terrain.toml`, `tools/data_pipeline/earth_terrain.py`, `tools/data_pipeline/test_earth_terrain.py`
- Modify: `tools/data_pipeline/fetch.py` (kind `earth_terrain`), `tools/data_pipeline/sources.toml`, `CREDITS.md`
  (regenerated), `tools/data_pipeline/requirements.txt` (or the existing dependency note: rasterio, pyproj)

**Interfaces:**
- Consumes: the grid header of Task 1 (`projection = "geographic"`), `data/geoid/egm2008_25.*`.
- Produces in `data/terrain/earth/`: `global_relief.toml/.i16` (ETOPO 2022 surface, resampled to 2' node grid,
  dn 1 m, global_longitude), `<area>_regional.toml/.i16` (GLO-30, ~1 arcsec, dn 0.1 m),
  `<area>_local.toml/.i16` (3DEP 1 m bare earth reprojected to a geographic grid of 1/30 arcsec lat x lon step for
  ~1 m, dn 0.01 m). All `height_reference = "wgs84_ellipsoid"`. Each header carries `area`, `level`, the area centre
  and half sizes it was cut for, and `[[check]]` points.
- `config/earth_terrain.toml`:
  ```toml
  [[area]]
  id = "starbase"
  latitude_deg = 25.99686        # centre: OLP-1
  longitude_deg = -97.15394
  local_half_size_m = 5000.0     # [E] level 1 (spec: ~10 x 10 km)
  regional_half_size_m = 100000.0  # [E] level 2 (spec: ~200 x 200 km)
  [[area]]
  id = "cape"                    # LC-39A, SLC-40, LZ-40 in one local tile
  latitude_deg = 28.585
  longitude_deg = -80.590
  local_half_size_m = 6000.0     # [E] covers LC-39A (28.608, -80.604) and SLC-40 (28.562, -80.577) + 1 km margin
  regional_half_size_m = 100000.0
  [global]
  relief_step_arcmin = 2.0       # [E] impacts away from the areas; 117 MB as int16
  ```

- [ ] **Step 1: Tests** (`test_earth_terrain.py`, synthetic rasters, no network):
  `test_orthometric_to_ellipsoidal_adds_geoid` (h_ell = H + N at every node),
  `test_navd88_converted_with_geoid18` (the 3DEP path goes NAVD88 -> ellipsoidal through pyproj with GEOID18
  `us_noaa_g2018u0.tif`; a fixed point's result equals pyproj's own transform within 1 mm),
  `test_area_bounds` (half sizes -> lat/lon box at the area latitude), `test_resample_global_wraps`,
  `test_local_tile_nodata_filled_from_regional` (3DEP gaps take the GLO-30 value; none left as nodata).
- [ ] **Step 2: Implement `earth_terrain.py`**: ETOPO 2022 60" surface GeoTIFF (NOAA NCEI, public domain) -> 2';
  GLO-30 1-degree COG tiles from the AWS open-data bucket `copernicus-dem-30m` covering each area box; 3DEP 1 m
  tiles via the TNM Access API (`https://tnmaccess.nationalmap.gov/api/v1/products`, dataset "Digital Elevation
  Model (DEM) 1 meter") for each local box. Downloads into `data/raw/earth/` (cached, `--force` refetches). Note in
  the header: GLO-30 is a surface model (vegetation/buildings), 3DEP is bare earth. Coverage fallback: an area
  without 3DEP 1 m uses GLO-30 for its local level, logged and written to the header (`source = ...`).
- [ ] **Step 3: Run** the pytest file: PASS. Run `python tools/data_pipeline/fetch.py earth_terrain geoid` once and
  record sizes, sources and the seam offsets (local vs regional mean difference over the overlap, per area) in the
  ledger; these numbers go into `docs/subsystems.md` in Task 7.
- [ ] **Step 4: Regenerate** `CREDITS.md`; commit + push `feat(data): Earth relief, GLO-30 and 3DEP terrain tiles`.

### Task 3: `Terrain` interface and `EarthTerrain`

**Files:**
- Create: `core/include/starsim/environment/terrain.hpp`
- Create: `core/include/starsim/environment/earth_terrain.hpp`, `core/src/environment/earth_terrain.cpp`
- Create: `core/include/starsim/sim/earth_terrain_loader.hpp`, `core/src/sim/earth_terrain_loader.cpp`
- Modify: `lunar_terrain.hpp/.cpp` (derive from `Terrain`), `reference_body.hpp/.cpp` (`set_terrain(const Terrain*)`),
  `contact/landing_surface.hpp/.cpp` (`TerrainField` holds `std::shared_ptr<const environment::Terrain>`, sets
  `SurfaceSupport::water`), `config/launch_site_config.hpp/.cpp` (plateau fields), `config/launch_sites.toml`
  (plateau values)
- Test: `core/tests/test_earth_terrain.cpp`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::environment {
  class Terrain {   // a body's surface over its shape, body-fixed coordinates
  public:
      virtual ~Terrain() = default;
      virtual double height_at_fixed_m(const math::Vec3& fixed_m) const = 0;   // above the shape, below the point
      virtual math::Vec3 normal_fixed(const math::Vec3& fixed_m) const = 0;    // outward unit normal
      virtual bool water_at_fixed(const math::Vec3& /*fixed_m*/) const { return false; }
  };
  struct Plateau {   // a pad, tower footprint or landing zone: flat at height_ellipsoid_m inside radius_m
      std::string site_id;
      double latitude_rad = 0.0, longitude_rad = 0.0, height_ellipsoid_m = 0.0, radius_m = 0.0, apron_m = 0.0;
  };
  class EarthTerrain final : public Terrain {
  public:
      static constexpr double blend_m = 500.0;            // [E] local -> regional and regional -> global, at the edge
      static constexpr double water_threshold_m = 0.05;   // [E] orthometric <= this: water (GLO-30 sets the sea to 0)
      EarthTerrain(Geoid geoid, GeographicGrid global_relief, std::vector<GeographicGrid> regional,
                   std::vector<GeographicGrid> local, std::vector<Plateau> plateaus);
      double height_ellipsoid_m(double latitude_rad, double longitude_rad) const;  // surface; sea = geoid
      bool water(double latitude_rad, double longitude_rad) const;
      const Geoid& geoid() const;
      // Terrain overrides (WGS84 geodetic of fixed_m; normal by central differences over normal_step_m)
      static constexpr double normal_step_m = 0.5;        // [E] half the 1 m local spacing
  };
  }
  namespace starsim::sim {
  // config/earth_terrain.toml + data/terrain/earth + data/geoid + plateaus from config/launch_sites.toml.
  // Throws config::ConfigError (German) when a file is missing or stale (area centre / size or plateau height
  // differs from what the tile was cut for).
  std::shared_ptr<const environment::EarthTerrain> load_earth_terrain(const std::filesystem::path& config_dir,
                                                                      const std::filesystem::path& data_dir);
  bool earth_terrain_available(const std::filesystem::path& data_dir);
  }
  ```
- `LaunchSiteConfig` gains `plateau_radius_m`, `plateau_apron_m` (TOML keys of the same names, required for every
  site). Values: `starbase_olp1` 150 / 50, `ksc_lc39a` 150 / 50, `ccsfs_slc40` 100 / 50 (all [E] from imagery:
  mount + tower + hardstand), `ccsfs_lz40` 61 / 30 (61 m = half the 400 ft gravel apron [S] FAA EA 2025; apron [E]).
- Precedence in `height_ellipsoid_m`: plateau (inside radius: its height; over the apron: linear blend to the
  terrain below) > local > regional > global relief; each finer grid blends into the next over `blend_m` from its
  edge. Water: not on a plateau, and `terrain - N <= water_threshold_m`; then the height is N.

- [ ] **Step 1: Tests** (tags `[environment][terrain][earth_terrain]`, synthetic grids):
  `local grid wins inside and blends at the edge` (value continuous across the blend band, equals local at
  `blend_m` inside the edge), `plateau is flat and blends over the apron`, `plateau is never water` (plateau
  height below N), `zero orthometric is water and sits on the geoid`, `sea floor below the geoid is water at N`,
  `normal of a tilted plane grid` (a synthetic 2 % slope gives the analytic normal within 1e-6),
  `lunar terrain through the Terrain interface is unchanged` (existing `[terrain]` cases stay green),
  `stale local tile is refused` (loader with an area centre moved by > 1 cell throws `ConfigError`),
  `[data]` `loaded Earth terrain at the pads` (each plateau height within 0.01 m of `launch_sites.toml`; the
  `[[check]]` points of every grid file reproduced within 0.01 m).
- [ ] **Step 2: Run** `-TestFilter "[earth_terrain]"`: FAIL (missing types).
- [ ] **Step 3: Implement** the interface change, `EarthTerrain`, the loader, the config fields.
- [ ] **Step 4: Run** `-TestFilter "[earth_terrain],[terrain],[legs],[lunar_descent_session],[config]"`: PASS.
- [ ] **Step 5: Commit + push** `feat(environment): Earth terrain with plateaus, water and the Terrain interface`.

### Task 4: Session and contact on the Earth terrain

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp`, `core/src/sim/flight_session.cpp` (geoid, terrain API),
  `core/include/starsim/vehicle/flight_body.hpp` (`ground_height_at`), `core/src/vehicle/flight_body.cpp` (comments),
  `core/src/sim/flight_session.cpp:487-497` and `flight_session_return.cpp:383-392` (leg sites take a
  `TerrainField` when terrain is set), `core/src/sim/flight_session_crew_escape.cpp:99,201,259` (water from the
  terrain when set, else `site.sea`), telemetry (`altitude_terrain_m` on Earth = altitude - surface height when set)
- Test: `core/tests/test_earth_terrain_session.cpp`

**Interfaces:**
- Consumes: Task 1 `Geoid`, `config::load_coarse_geoid`; Task 3 `EarthTerrain`, `Terrain`.
- Produces on `FlightSession`:
  ```cpp
  const environment::Geoid& geoid() const;   // coarse 1-degree grid, loaded from config_dir in the constructor
  void set_earth_terrain(std::shared_ptr<const environment::EarthTerrain> terrain);   // nullptr: flat model
  const environment::EarthTerrain* earth_terrain() const;
  std::shared_ptr<const environment::EarthTerrain> earth_terrain_shared() const;      // game queries
  double sea_height_m(double latitude_rad, double longitude_rad) const;  // terrain's fine geoid, else coarse
  ```
  `FlightBody::ground_height_at`: Earth with `body.terrain()` set -> `body.surface_height_m(fixed)`; otherwise the
  flat `ground_height_m_` (the old model = fidelity "flat ground", kept for comparison runs). Guidance planners keep
  `site.sea` and the flat site height (spec).

- [ ] **Step 1: Tests** (tags `[session][earth_terrain]`, a synthetic `EarthTerrain` built in the test - no data):
  `vehicle rests on the plateau height`, `impact on terrain away from the site` (a body dropped 20 km from the pad
  on a synthetic hill grid strikes at the hill height, not the pad height), `crew escape touchdown over water from
  terrain` (`touchdown_over_water` follows `EarthTerrain::water`), `leg landing on a sloped terrain field`,
  `flat model unchanged without terrain` (same `ground_height_m` behaviour as before, bit-identical state after a
  short rest), `sea height uses the coarse geoid without terrain`.
- [ ] **Step 2: Run** `-TestFilter "[earth_terrain]"`: FAIL.
- [ ] **Step 3: Implement**.
- [ ] **Step 4: Run** `-TestFilter "[earth_terrain],[session],[legs],[crew_escape],[contact]"`: PASS (crew-escape and
  legs campaigns run without terrain, so their numbers must not move here).
- [ ] **Step 5: Commit + push** `feat(sim): Earth terrain in the flight session, contact and crew escape`.

### Task 5: Sea level from the geoid; game loads the terrain

**Files:**
- Modify: `core/include/starsim/sim/ascent_profile_planner.hpp:29` (remove `drone_ship_sea_height_m`),
  `core/src/sim/ascent_profile_planner.cpp:407-416` (sea height = `session.sea_height_m` at the recovery point),
  `core/src/sim/flight_session_capsule.cpp:13,137,206` (splashdown target and ground at the geoid of the zone),
  `core/tests/test_falcon9_asds.cpp:352` and other tests that assert the old constants
- Modify: `bindings/src/flight_sim.cpp` (after creating the session: `if (sim::earth_terrain_available(data_dir))
  session.set_earth_terrain(sim::load_earth_terrain(...))`, else `UtilityFunctions::print` "Erdgelände fehlt -
  flacher Boden (python tools/data_pipeline/fetch.py)"; new getter `earth_surface(lat_deg, lon_deg)` ->
  `{present, height_m, water}` reading `earth_terrain_shared()`, like `moon_terrain`); check that
  `sim::start_mission` keeps the terrain on the session
- Modify: `game/world/ground_patch.gd` (vertex heights from `earth_surface` when present; old flat rule otherwise),
  `game/world/world.gd` (pass the FlightSim)

**Interfaces:**
- Consumes: Task 4 `FlightSession::sea_height_m`, `earth_terrain_shared`; Task 3 loader.

- [ ] **Step 1: Tests**: `drone ship deck at the geoid plus freeboard` (`[asds]`: `landing_zone()->height_m ==
  session.sea_height_m(lat, lon) + 3.0`), `capsule splashdown target on the geoid` (`[splashdown]`).
- [ ] **Step 2: Run** `-TestFilter "[asds],[splashdown]"`: FAIL.
- [ ] **Step 3: Implement** core + bindings + GroundPatch.
- [ ] **Step 4: Run** `-TestFilter "[asds],[splashdown],[dragon],[falcon9],[capsule],[parachute],[ascent_planner]"`
  in the background (log `tmp/m8a_t5.log`); re-baseline moved numbers (drone ship sea ~-27 m instead of -35 m, the
  San Diego zone N ~-36 m instead of 0) with the reason in the test comments. Full build with bindings, then the
  smoke test (`godot ... --headless --path game --script res://tests/smoke_test.gd`): PASS, and the log shows the
  terrain loaded.
- [ ] **Step 5: Render check** (GPU): `render_check.gd --lat=25.99686 --lon=-97.15394` ground view: the patch shows
  the coast relief, no hole or step at the pad; read the PNG.
- [ ] **Step 6: Commit + push** `feat(sim): sea level from the EGM2008 geoid; game loads the Earth terrain`.

### Task 6: Site heights derived from the data

**Files:**
- Create: `tools/data_pipeline/site_heights.py`, `tools/data_pipeline/test_site_heights.py`
- Modify: `config/launch_sites.toml` (heights [D], header comment rewritten: ellipsoidal from DEM + geoid, no
  longer estimates), `game/` nothing else

**Interfaces:**
- Consumes: Task 2 local grids, Task 3 plateau radii.
- Produces: `python tools/data_pipeline/site_heights.py [--write]` prints per site: median ellipsoidal height of
  the local grid within `plateau_radius_m`, its spread (max - min inside the radius), orthometric height, and the old
  value; `--write` replaces only the `height_m = ...` line of each `[[site]]` block, keeping comments, with the tag
  `# [D] median of <source> within <r> m, <date>`.

- [ ] **Step 1: Tests**: `test_median_inside_radius` (synthetic grid with a mound: median of the disc),
  `test_write_keeps_comments` (round trip on a copy of `launch_sites.toml`: only the height lines change).
- [ ] **Step 2: Implement**, run pytest: PASS.
- [ ] **Step 3: Run** `site_heights.py`, record old/new heights and spreads in the ledger. Plausibility gate before
  writing: each orthometric height between -2 m and +25 m (coastal pads; LC-39A's pad mound is ~15 m [E]); a spread
  > 5 m means the radius catches slopes - shrink that site's radius and note why. Then `--write`.
- [ ] **Step 4: Run** `-TestFilter "[earth_terrain]"` including the `[data]` case (plateau heights match the config).
- [ ] **Step 5: Commit (no push)** `fix(config): launch site heights derived from 3DEP + EGM2008` - the campaign
  tests are re-validated in Task 7; this commit is pushed together with Task 7 (exception to "push after every task").

### Task 7: Re-validation of the reference flights and docs

**Files:**
- Modify: tests whose reference numbers move (comment the old value, the new value and the reason), guidance or
  physics where a catch / landing fails, `docs/subsystems.md` (new section "Earth terrain and geoid": sources,
  levels, sizes, seam offsets, datum notes, water rule, the flat-ground comparison mode; update the ascent / catch /
  RTLS / ASDS / capsule sections with re-baselined numbers), `CLAUDE.md` (Known issues: Earth ground no longer flat
  near sites; implemented modules list; drop "Earth DEM ... M8" as open), `config/README.md`
- Create: `core/tests/test_earth_terrain_flight.cpp` (`[data][slow][earth_terrain]`)

- [ ] **Step 1: Run** in the background, logs in `tmp/m8a_t7_*.log`, checked every 10 min:
  `-TestFilter "[ascent],[ascent_planner],[catch],[return],[rtls],[asds],[falcon9],[ship],[crew_escape],[splashdown],[iss_mission],[validation],[booster],[scvx_landing],[descent],[entry],[deorbit]"`,
  then `"[refuelling],[campaign],[lunar]"` (the lunar return lands at a launch site).
- [ ] **Step 2: Triage every failure** into (a) a comparison whose expected value moved because the site moved
  (flight 14 / CRS-35 altitude: check whether the reference altitude is above the pad, MSL or the ellipsoid and
  convert consistently) -> re-baseline with reason; (b) a catch / landing / planner failure -> fix the cause
  (systematic debugging), no tolerance change. List each in the ledger.
- [ ] **Step 3: Test** `starbase catch on the real terrain` and `Falcon 9 RTLS to LZ-40 on the real terrain`
  (`[data][slow]`): session with `load_earth_terrain`, the reference mission flown to the end: caught / landed with
  the same limits as the flat-ground cases; the booster's final rest height equals the plateau height within 0.01 m.
- [ ] **Step 4: Run** Step 1's tag sets again + the two `[data]` flights: all PASS.
- [ ] **Step 5: Docs** as listed; update memory note (milestone order: Plan A done, Plan B next).
- [ ] **Step 6: Commit + push** `test: reference flights re-validated on derived site heights` and
  `docs: Earth terrain and geoid`.
