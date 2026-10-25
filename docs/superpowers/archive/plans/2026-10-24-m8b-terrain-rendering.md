# M8 Plan B: streamed terrain renderer, imagery and calm sea Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Earth and the Moon are drawn by a streamed level-of-detail terrain whose heights are the physics
heights, with 0.6 m NAIP imagery around the pads, a 10 m Sentinel-2 composite around Starbase and the Cape, a calm
procedural sea everywhere, and the Mons Mouton site relief on the Moon; the interim `GroundPatch` is gone.

**Architecture:** One GDScript quadtree per body on a cube sphere (6 faces, uniform near the poles - Mons Mouton is at
-84.8 deg). Each node is a 64 x 64 cell mesh whose vertex heights come from the core's `Terrain` in one batch call
(`StarsimCore.surface_samples`), so rendered ground = physics ground. Nodes are built on `WorkerThreadPool` and
uploaded on the main thread under a per-frame budget; geomorphing in the vertex shader (a per-vertex morph target)
and parent-kept-until-children-ready remove popping and holes. Imagery is an EPSG:4326 tile pyramid written by the
pipeline; a node's texture is the mosaic crop over its lat/lon bounding box with per-vertex UVs computed in float64.
Earth and Moon terrains are loaded once per process (cache) and shared by the renderer and every `FlightSession`.

**Tech Stack:** C++20 core (Catch2), godot-cpp bindings, Godot 4.7 GDScript + shaders, Python pipeline (numpy,
rasterio, pyproj, requests, Pillow; `pystac-client` new).

**Spec:** `docs/superpowers/specs/2026-10-23-m8-terrain-tiles-design.md` (this plan = "Plan B - rendering"; goals 5-9).
Plan A (physics, done): `docs/superpowers/plans/2026-10-23-m8a-earth-terrain-physics.md`.

## Global Constraints

- Only free data; every source with its licence in `tools/data_pipeline/sources.toml`, `CREDITS.md` regenerated
  (`python tools/data_pipeline/credits.py`). NAIP (USDA FSA, public domain); Sentinel-2 L2A (Copernicus, free and
  open, attribution "Contains modified Copernicus Sentinel data 2025"). EOX s2cloudless is NOT used (user 2026-10-24:
  own recent composite).
- Physics never runs in GDScript: the renderer only reads heights / water from the core. Rendered and physics ground
  must agree (spec anti-goal): vertex heights are core heights, never re-derived in the game.
- Floating origin: node vertices relative to the node centre (float32), node transforms in double (`precision=double`).
- VR: 90 Hz on the RTX 4090, no popping, no jitter (spec anti-goals). Per-eye identical LOD and morph (morph distance
  from a per-node `camera_local` instance uniform, not from the view matrix).
- `data/` is optional: without Earth terrain / imagery / LDEM the game renders exactly as before (flat rule, day
  texture, smooth Moon) with one German log line per missing item; no per-frame errors.
- Godot: text formats only; runtime data loaded with `Image.load_from_file` (no import step). Headless GDScript tests
  run with `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/<name>.gd`.
- C++ UTF-8, UI/log strings German, code/comments/docs English, no magic numbers (`[S]/[E]/[D]` with source/reason).
- Test workflow (CLAUDE.md): only the affected tags via `tools/build.ps1 [-NoBindings] -TestFilter "[a],[b]"`; runs
  that may exceed 10 min to the background, logs in `tmp/`. Conventional Commits; commit + `git push origin main`
  after every task once its tests are green.
- Progress ledger: `.superpowers/sdd/2026-10-24-m8b-terrain-rendering/progress.md` (done, open, decisions, numbers,
  next step).

## Review Focus

- Fresh checkout without `data/`: game starts, Earth/Moon look as before, one German log line, no errors - Task 6
  test `quadtree without terrain uses the flat rule` + smoke test; Task 1 `load_terrain` without files.
- Node across the antimeridian or containing a pole: no NaN vertex, no texture seam (global u continuous across
  +-180 deg, pole vertex longitude taken from the node centre) - Task 5 tests `antimeridian node`, `pole node`.
- Camera jump (Tab to a body 1000 km away, view switch Moon -> pad, time warp): never a hole - the root nodes are
  built at start and kept, a parent is only hidden when all 4 children are uploaded, stale requests are dropped -
  Task 6 test `jump keeps coverage`.
- Vehicle on the pad: the drawn surface at the pad centre, on the LZ-40 plateau and at a dune point equals
  `surface_samples` within 0.05 m at the finest level (morph 0) - Task 6 test `finest mesh matches the core`.
- Imagery gap (a NAIP or Sentinel tile missing, sea-only tiles skipped by the pipeline): the node falls back to the
  next coarser source / level or the day texture, never black - Task 5 test `missing tile falls back`.

---

### Task 1: Shared terrain cache and geodetic surface sampling (core + bindings)

**Files:**
- Create: `core/include/starsim/sim/terrain_cache.hpp`, `core/src/sim/terrain_cache.cpp`
- Create: `core/include/starsim/environment/surface_samples.hpp`, `core/src/environment/surface_samples.cpp`
- Modify: `core/include/starsim/environment/terrain.hpp` (geodetic virtuals), `earth_terrain.hpp/.cpp`,
  `lunar_terrain.hpp/.cpp` (overrides), `core/src/sim/lunar_landing_mission.cpp:29` (cache),
  `bindings/src/flight_sim.cpp:62-77` (cache), `bindings/src/starsim_core.hpp/.cpp` (new methods)
- Test: `core/tests/test_terrain_cache.cpp`, `core/tests/test_surface_samples.cpp`, `game/tests/smoke_test.gd`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::environment {
  class Terrain {   // additions
      virtual double height_m(double latitude_rad, double longitude_rad) const = 0;  // above the shape (Earth geodetic)
      virtual bool water(double /*latitude_rad*/, double /*longitude_rad*/) const { return false; }
      virtual double resolution_m(double latitude_rad, double longitude_rad) const = 0;  // finest grid spacing there
  };
  struct SurfaceSamples { std::vector<double> height_m; std::vector<uint8_t> water; };  // 1 = water
  SurfaceSamples sample_surface(const Terrain& terrain, std::span<const double> latitude_rad,
                                std::span<const double> longitude_rad);   // sizes equal, else std::invalid_argument
  }
  namespace starsim::sim {
  // Process-wide: one instance per (canonical config_dir, data_dir) while anyone holds it (weak_ptr cache, mutex).
  std::shared_ptr<const environment::EarthTerrain> shared_earth_terrain(const std::filesystem::path& config_dir,
                                                                        const std::filesystem::path& data_dir);
  std::shared_ptr<const environment::LunarTerrain> shared_lunar_terrain(const std::filesystem::path& config_dir,
                                                                        const std::filesystem::path& data_dir);
  }
  ```
  `EarthTerrain::height_m` = `height_ellipsoid_m`; `resolution_m` = spacing (m, latitude step x metres per degree) of
  the finest grid covering the point (local 1 m, regional ~30 m, global ~3.7 km). `LunarTerrain::resolution_m`: 20 m
  in a site tile, LDEM_16 spacing (~1.9 km) elsewhere. Loader errors propagate (German `ConfigError`), not cached.
- GDScript API on `StarsimCore` (thread-safe: only reads immutable terrains; callable from `WorkerThreadPool`):
  - `load_terrain(config_dir: String) -> Dictionary` `{earth: bool, moon: bool}`; prints one German line per missing
    or unloadable terrain ("Erdgelände fehlt - flacher Boden ...", "Mondgelände fehlt - glatte Kugel ...").
  - `surface_samples(body: String, latitudes_deg: PackedFloat64Array, longitudes_deg: PackedFloat64Array) ->
    Dictionary` `{present: bool, height_m: PackedFloat64Array, water: PackedByteArray}` (`body` "earth" / "moon").
  - `surface_resolution_m(body: String, latitude_deg: float, longitude_deg: float) -> float` (INF when absent).
  `FlightSim::setup` and `lunar_landing_mission` take their terrain from the cache.

- [ ] **Step 1: Tests** (tags `[terrain_cache]`, `[surface_samples]`, `[environment]`):
  `sample_surface matches pointwise height and water` (synthetic `EarthTerrain` from Plan A's test helpers: every
  sample equals `height_ellipsoid_m` / `water` bit-identically), `sample_surface rejects mismatched spans`,
  `resolution follows the finest grid` (synthetic local / regional / global: 1 m / 30 m / global step in m within
  1e-9 relative), `lunar resolution in and outside the tile`, `cache returns the same instance while held`
  (`[data]`: two calls -> same pointer; after both released a third call loads again), `cache rethrows a missing
  file` (empty temp dir, unique name: throws `ConfigError`, a second call throws again).
- [ ] **Step 2: Run** `tools/build.ps1 -NoBindings -TestFilter "[terrain_cache],[surface_samples]"`: FAIL (missing).
- [ ] **Step 3: Implement** core, then bindings; extend `smoke_test.gd`: `load_terrain` returns a Dictionary with
  both keys, `surface_samples("earth", ...)` at the Starbase pad equals `FlightSim.earth_surface` height when
  present, and `present == false` with empty arrays when absent.
- [ ] **Step 4: Run** `-NoBindings -TestFilter "[terrain_cache],[surface_samples],[earth_terrain],[terrain],[lunar_descent_session]"`,
  then the full build with bindings and the smoke test: PASS. Record in the ledger: process memory after starting
  the game and launching twice (`FlightSim.setup` x2) before / after (the ~770 MB copy per setup must be gone).
- [ ] **Step 5: Commit + push** `feat(sim): process-wide terrain cache and geodetic surface sampling`.

### Task 2: Earth leg sites get a tracking radius

**Files:**
- Modify: `core/src/sim/flight_session.cpp:1272-1285` (`earth_leg_site`), `core/include/starsim/sim/leg_landing_sim.hpp:24-27` (comment)
- Test: `core/tests/test_earth_terrain_session.cpp`

**Interfaces:**
- `FlightSession::earth_leg_site` with terrain sets `LegSite::tracking_radius_m = earth_leg_site_tracking_radius_m`,
  a new `static constexpr double earth_leg_site_tracking_radius_m = 5000.0;` in `flight_session.cpp` [E: a landing
  missing its zone by more than 5 km is no leg landing; the local 1 m tile reaches 5-6 km; beyond it the body's own
  ground contact (`FlightBody` on the terrain) judges]. The flat-ground and deck paths are unchanged.

- [ ] **Step 1: Tests** (`[session][earth_terrain][legs]`, synthetic terrain): `leg site tracks a body inside the
  radius` (legged body 1 km from the zone: on the leg site), `body beyond the radius keeps its own ground contact`
  (6 km away: not on the leg site; it rests / impacts on the terrain height there via `FlightBody`).
- [ ] **Step 2: Run** `-NoBindings -TestFilter "[earth_terrain]"`: the second case FAILS.
- [ ] **Step 3: Implement**.
- [ ] **Step 4: Run** `-NoBindings -TestFilter "[earth_terrain],[legs],[rtls],[asds],[falcon9]"`: PASS (no flight
  lands 5 km off; numbers must not move).
- [ ] **Step 5: Commit + push** `fix(sim): Earth leg sites track bodies within 5 km`.

### Task 3: NAD83 -> ITRF horizontal shift of the 3DEP heights

**Files:**
- Modify: `tools/data_pipeline/earth_terrain.py` (`read_3dep_local`, header field), `tools/data_pipeline/test_earth_terrain.py`
- Possibly modify: `config/launch_sites.toml` (only by the rule in Step 4)

**Interfaces:**
- `nad83_horizontal_shift_deg(latitude_deg, longitude_deg) -> tuple[float, float]` (dlat, dlon of ITRF2014 minus
  NAD83(2011) for the same physical point, from the existing `_nad83_to_itrf_transformer`, EPSG 6319 -> 7912 at
  h = 0). `read_3dep_local` reprojects the UTM tiles to a geographic grid in NAD83(2011) and then labels that grid
  in ITRF by adding the area-centre shift to `north_latitude_deg` / `west_longitude_deg` (the shift varies < 1 cm
  over 10 km - record the measured variation across the box). Header gains `horizontal_shift_m = <east, north>`.
  Task 4 reuses the same function for NAIP.

- [ ] **Step 1: Tests**: `test_nad83_shift_matches_pyproj` (a point near Starbase: shift equals pyproj's own 3D
  transform within 1 mm, magnitude 1.0-1.6 m), `test_local_grid_origin_shifted` (synthetic local tile: origin moved
  by exactly the shift, heights untouched).
- [ ] **Step 2: Implement**, run the pytest file: PASS.
- [ ] **Step 3: Run** `python tools/data_pipeline/fetch.py earth_terrain` (cached raw files, local tiles re-cut),
  then `python tools/data_pipeline/site_heights.py` (no `--write`): record old/new heights in the ledger.
- [ ] **Step 4: Rule:** every site differs by < 0.05 m [E: well inside the plateau survey spread] -> config unchanged,
  note in the ledger. Otherwise `--write`, then run Plan A Task 7's campaign tag sets in the background
  (`tmp/m8b_t3_*.log`) and re-baseline per Plan A's triage rules.
- [ ] **Step 5: Run** `-NoBindings -TestFilter "[earth_terrain]"` incl. the `[data]` case: PASS.
- [ ] **Step 6: Commit + push** `fix(data): 3DEP tiles shifted from NAD83 to ITRF`.

### Task 4: Imagery pipeline (NAIP, Sentinel-2 composite, tile pyramid)

**Files:**
- Create: `tools/data_pipeline/imagery.py`, `tools/data_pipeline/test_imagery.py`, `config/imagery.toml`
- Modify: `tools/data_pipeline/fetch.py` (kind `imagery`), `sources.toml`, `requirements.txt` (`pystac-client`),
  `CREDITS.md` (regenerated), `config/README.md`

**Interfaces:**
- Tiling (EPSG:4326 TMS): level z has 2^(z+1) x 2^z tiles of 256 x 256 px, tile size 180/2^z deg; x counts east
  from 180 W, y counts south from 90 N; pixel centres at (i + 0.5) / 256 of the tile. Path
  `data/imagery/earth/<source_id>/<z>/<x>/<y>.jpg` (JPEG quality 88 [E]).
- `config/imagery.toml` (one entry per source, boxes from `config/earth_terrain.toml`'s areas):
  ```toml
  [[source]]
  id = "naip_starbase"     # also naip_cape; s2_starbase, s2_cape
  product = "naip"         # | "sentinel2_median"
  area = "starbase"
  box = "local"            # NAIP over local_half_size_m (+ 500 m margin), Sentinel-2 over regional_half_size_m
  min_level = 14           # NAIP 14-17 (z17: 0.59 m/px at 26 deg lat), Sentinel-2 8-13 (z13: 9.5 m)
  max_level = 17
  ```
  Written index `data/imagery/earth/index.toml`: per source `id, product, year(s), min_level, max_level,
  north/south/west/east_deg` (exact coverage), `fade_m` (500 for NAIP, 5000 for Sentinel-2 [E]), `gain_rgb`,
  `offset_rgb`.
- NAIP: Microsoft Planetary Computer STAC (`https://planetarycomputer.microsoft.com/api/stac/v1`, collection `naip`,
  newest year covering the box, assets signed via `https://planetarycomputer.microsoft.com/api/sas/v1/token/naip`),
  RGB bands, NAD83 UTM -> geographic + `nad83_horizontal_shift_deg` (Task 3). Sentinel-2: Element84 Earth Search
  (`https://earth-search.aws.element84.com/v1`, `sentinel-2-l2a`, 2025, `eo:cloud_cover < 20`, up to 8 scenes per
  MGRS tile, `visual` + `scl` assets); per-pixel median of the clear pixels (SCL classes 4, 5, 6, 7 [S: ESA L2A
  scene classification]), WGS84 UTM -> geographic (no datum shift). Colour match: per-channel gain/offset by least
  squares against `data/textures/earth_day.jpg` sampled on a 1 km lattice over each box (stored in the index,
  applied when writing tiles), so the imagery meets the day texture without a colour step. Lower levels of each
  source are 2x2 box downsamples of its finest level. Tiles entirely outside the box are not written.

- [ ] **Step 1: Tests** (synthetic rasters, no network): `test_tile_bounds_level0` (z0: 2 tiles, x=1 starts at
  0 deg), `test_tile_bounds_level17` (tile size 180/2^17 deg), `test_tiles_for_box` (exactly the tiles intersecting
  a box), `test_median_ignores_clouds` (3 scenes, clouds by SCL: the clear value wins), `test_colour_match_recovers_gain`
  (synthetic gain 1.3 / offset -10 recovered within 1 %), `test_downsample_pyramid` (a level-z tile equals the 2x2
  box mean of its 4 children), `test_naip_shift_applied` (output georeference moved by the Task 3 shift).
- [ ] **Step 2: Implement**, run the pytest file: PASS.
- [ ] **Step 3: Run** `python tools/data_pipeline/fetch.py imagery` (background, `tmp/m8b_t4_fetch.log`); record per
  source: years/scenes used, tile counts, disk size, gain/offset, and a 1-tile spot check (read a z17 tile at the
  OLP-1 mount and a z13 tile over South Padre Island; describe what is visible) in the ledger.
- [ ] **Step 4: Regenerate** `CREDITS.md`; commit + push `feat(data): NAIP and Sentinel-2 imagery tile pyramid`.

### Task 5: Cube-sphere quadtree and node builder (headless GDScript)

**Files:**
- Create: `game/world/terrain/cube_sphere.gd`, `game/world/terrain/terrain_node_builder.gd`,
  `game/world/terrain/imagery_source.gd`, `game/world/terrain/height_source.gd`
- Test: `game/tests/terrain_mesh_test.gd`

**Interfaces:**
- `CubeSphere` (static): node key `Vector4i(face, level, i, j)` (face 0-5: +X,-X,+Y,-Y,+Z,-Z body-fixed; level 0 =
  whole face; i, j in [0, 2^level)); `direction(face, s, t) -> Vector3` (s, t in [-1, 1], tangent-adjusted:
  unit vector of face axes with tan(pi/4 * s), tan(pi/4 * t)); the direction is the geodetic normal (Earth: lat =
  asin(z), lon = atan2(y, x); Moon: sphere); `node_corners(key)`, `node_size_m(key, radius_m)` (arc length of the
  node's edge), `children(key)`, `parent(key)`, `lat_lon_box(key) -> Rect2` (float64, deg; antimeridian nodes get
  east > 180).
- `HeightSource` (RefCounted): `body` ("earth"/"moon"), `core` (StarsimCore), `sites` (launch sites for the flat
  rule); `sample(lats_deg: PackedFloat64Array, lons_deg: PackedFloat64Array) -> Dictionary {height_m, water}`:
  `core.surface_samples` when present, else Earth: the GroundPatch flat rule (site height within 2 deg of a site,
  else 0, water none), Moon: 0; `position_fixed(lat_deg, lon_deg, height_m) -> Vector3` (WGS84 via
  `GroundPatch.geodetic_to_fixed` moved here as `HeightSource.geodetic_to_fixed`, Moon: sphere 1737.4 km).
- `ImagerySource` (RefCounted, thread-safe reads): loads `data/imagery/earth/index.toml` via `TomlLite` (absent:
  empty); `crop(box_deg: Rect2, texel_deg: float) -> Dictionary {image: Image, box_deg: Rect2, fade_m: float}` or
  `{}`: the finest source whose coverage intersects `box_deg`, level = the finest <= max_level with tile size >=
  texel_deg * 256, mosaic of at most 3 x 3 tiles (choose a coarser level otherwise), missing tile -> next coarser
  level of the same source, then the next coarser source; RGB8 image with mipmaps; `box_deg` is the exact mosaic
  extent for UVs.
- `TerrainNodeBuilder.build(key, height_source, imagery, radius_m) -> Dictionary` (pure data, worker-thread safe):
  - samples a 67 x 67 lattice (64 cells + one ring outside for seamless normals) in one `sample` call;
  - `vertices` (65 x 65 + skirt ring, relative to `center_fixed`, render axes via `Frames.to_render`),
    `center_fixed: Vector3` (float64), `normals` (central differences on the lattice), `indices` (shared constant
    array), skirt depth = node edge / 64 x 2 [E];
  - `morph` (CUSTOM0.xyz): for odd-index vertices the parent-level position (mean of the two / four even neighbours
    along the edge / cell) minus the vertex, zero for even vertices; CUSTOM0.w = 1 on skirt vertices;
  - `uv` (fine imagery crop), `uv2` (base imagery crop: the next coarser source covering the node, else zero),
    CUSTOM1 = (global equirect u, v, fade_fine, fade_base): u continuous across +-180 within the node; fades 1
    inside the source coverage, falling linearly to 0 over `fade_m` at its edge;
  - `water_mask: Image` (L8, 128 x 128 over `lat_lon_box`, from one `sample` call), CUSTOM2.xy = mask UV;
  - `max_resolution_m`: the minimum of `surface_resolution_m` at the node centre and its 4 corners (the quadtree
    stops splitting where the data are coarser than the mesh).

- [ ] **Step 1: Tests** (`terrain_mesh_test.gd`, synthetic height functions, no data, no GPU):
  `cube sphere covers the sphere once` (sum of node solid angles at level 3 = 4 pi within 1e-9; adjacent faces share
  edge directions exactly), `children tile the parent`, `antimeridian node` (u continuous, no NaN),
  `pole node` (face +Z level 2: all vertices finite, pole vertex longitude = node-centre longitude),
  `morph target equals the parent surface` (a node of a quadratic hill: every vertex + morph lies on the parent's
  mesh within 1e-3 m), `skirts hang below the edge`, `normals of a tilted plane` (analytic within 1e-4),
  `missing tile falls back` (temp index with a gap at level 17: the crop comes from level 16; a box outside every
  source returns `{}`), `flat rule without terrain` (Starbase vertex at the site height, Atlantic vertex at 0).
- [ ] **Step 2: Run** the test headless: FAIL (missing classes).
- [ ] **Step 3: Implement**.
- [ ] **Step 4: Run**: PASS. Also measure `build()` time for one level-17 node at the pad with real data (needs the
  bindings build) and record it (target < 20 ms on a worker thread [E]).
- [ ] **Step 5: Commit + push** `feat(game): cube-sphere terrain nodes with morph targets and imagery crops`.

### Task 6: Earth terrain renderer replaces GroundPatch

**Files:**
- Create: `game/world/terrain/terrain_quadtree.gd`, `game/shaders/terrain_earth.gdshader`,
  `game/shaders/terrain_morph.gdshaderinc`
- Modify: `game/world/world.gd` (`core.load_terrain`, quadtree child of `earth`, hide the ellipsoid mesh while the
  tree draws, `_make_pad` uses `HeightSource.geodetic_to_fixed`), `game/world/celestial_body.gd` (`body_mesh_visible`)
- Delete: `game/world/ground_patch.gd` (+ `.uid`)
- Test: `game/tests/terrain_quadtree_test.gd`, `game/tests/smoke_test.gd`, `game/tests/render_check.gd`

**Interfaces:**
- `TerrainQuadtree` (Node3D, child of the `CelestialBody`): `setup(height_source, imagery, radius_m, material_template:
  ShaderMaterial)`, `update_for_camera(camera_fixed_m: Vector3)` each frame, `active` (false in scaled space: body
  centre beyond `CelestialBody.SCALED_SPACE_DISTANCE_M`; then only the ellipsoid mesh is drawn; while active the
  ellipsoid mesh is hidden), stats `{nodes_drawn, nodes_loading, uploads_last_frame}`.
- Selection: a node splits when the camera's distance to its bounding sphere is < `LOD_FACTOR * node_size_m`
  (`LOD_FACTOR = 3.0` [E], tuned in Task 9), its mesh spacing `node_size_m / 64` is still coarser than the node's
  `max_resolution_m` (no splitting beyond the data), and level < `MAX_LEVEL = 21` [E: safety cap, ~7 cm cells at
  the Earth]; merges at `1.15 * LOD_FACTOR * node_size_m` (hysteresis
  [E]). Children are requested on split; the parent stays drawn until all 4 are uploaded; requests whose node is
  no longer wanted are dropped before building. The 6 x 4 level-1 nodes are built synchronously at setup and never
  freed.
- Loading: `WorkerThreadPool.add_task(TerrainNodeBuilder.build ...)`, results queued; main thread uploads at most
  `UPLOADS_PER_FRAME = 4` [E] (ArrayMesh with CUSTOM0-2 RGBA float, ImageTexture from the crop / mask, one
  ShaderMaterial per node duplicated from the template). Freed nodes return their MeshInstance3D to a pool.
- Shader `terrain_earth.gdshader`: same lighting as `earth.gdshader` (unshaded, sun cosine on the mesh normal, night
  lights, `eclipse.gdshaderinc`, `day_exposure`), plus: morph (`terrain_morph.gdshaderinc`: `VERTEX += CUSTOM0.xyz *
  morph`, morph = clamp((length(VERTEX - camera_local) - 1.4 * K * s) / (0.55 * K * s), 0, 1) with instance uniforms
  `camera_local`, `node_size_m`, K = LOD_FACTOR), albedo = day texture (CUSTOM1.xy) blended with `imagery_base`
  (UV2, CUSTOM1.w) then `imagery_fine` (UV, CUSTOM1.z); `vertex_offset` as before for body-local lighting.

- [ ] **Step 1: Tests** (`terrain_quadtree_test.gd`, headless, fake loader that builds synchronously):
  `coverage is complete` (after any update the drawn leaves' solid angles sum to 4 pi), `refines towards the
  camera` (camera 2 m above the OLP-1 pad: the node under it has level >= 17 with real data present, <= the level
  allowed by the flat rule otherwise), `jump keeps coverage` (camera moved from the pad to 1000 km away and to the
  Moon side in one frame with uploads throttled to 1 per frame: coverage stays 4 pi every frame), `merge
  hysteresis` (camera oscillating +-5 % around a split distance: no split/merge every frame),
  `finest mesh matches the core` (needs data; skipped with a printed note otherwise: at the OLP-1 pad centre, an
  LZ-40 plateau point and a dune point 2 km west of OLP-1, the mesh surface (barycentric on the node triangle, morph
  0) equals `core.surface_samples` within 0.05 m), `quadtree without terrain uses the flat rule`.
- [ ] **Step 2: Run** headless: FAIL.
- [ ] **Step 3: Implement** quadtree, shader, world wiring; delete GroundPatch (grep `GroundPatch` / `ground_patch`:
  no references left, also not in docs except history).
- [ ] **Step 4: Run** the GDScript tests + full build + smoke test: PASS.
- [ ] **Step 5: Render checks** (GPU), PNGs to `tmp/m8b_t6_*.png`, read each: Starbase ground view at the tower
  (`--lat=25.99686 --lon=-97.15394`), LC-39A, LZ-40, a 400 km orbit view over Florida, and `--fly=40` from OLP-1
  (vehicle stands on the drawn pad, no gap); compare against Plan A's `tmp/m8a_t5_pad*.png`.
- [ ] **Step 6: Commit + push** `feat(game): streamed Earth terrain renderer replaces the ground patch`.

### Task 7: Calm sea

**Files:**
- Create: `game/shaders/water.gdshaderinc`
- Modify: `game/shaders/terrain_earth.gdshader`, `game/world/world.gd` (sun direction and time uniforms)
- Test: `game/tests/terrain_mesh_test.gd` (water mask), `game/tests/render_check.gd` (sea view option)

**Interfaces:**
- Water where the node's `water_mask` (linear filtered, threshold 0.5 with a 1-texel smoothstep) says so; there the
  surface colour is: body colour = `imagery` colour where imagery covers (keeps lagoons and shoals), else `deep_water`
  [E: Vec3(0.02, 0.05, 0.08), open-ocean albedo ~0.06 [S: Payne 1972]]; plus Fresnel (Schlick, n = 1.333 [S]) times
  `sky_radiance` [E: placeholder constant until the atmosphere part, documented]; plus sun glint from a Cox-Munk
  slope distribution with wind `glint_wind_mps = 5.0` [E] (sigma^2 = 0.003 + 0.00512 U [S: Cox & Munk 1954]) using
  the wave-perturbed normal; waves: 4 directional Gerstner normal octaves (wavelengths 60, 23, 9, 3.5 m [E], gentle
  slopes), animated by `world_time_s` (simulation time, so time warp speeds them), faded out with distance (to the
  flat-normal glint) beyond 20 km [E] to avoid aliasing. Geometry stays at the geoid (physics sea), no displacement.
- `render_check.gd --sea_lat= --sea_lon= --eye_height=10`: ground view at sea at the geoid + eye height.

- [ ] **Step 1: Tests**: `water mask follows the core` (synthetic terrain with a straight coast: mask texels on the
  water side are 255, land side 0, matching `water` at texel centres).
- [ ] **Step 2: Run**: FAIL. **Step 3: Implement.** **Step 4: Run** the GDScript tests + smoke test: PASS.
- [ ] **Step 5: Render checks**, read each: a drone-ship area at sea (`config/drone_ships.toml` first Atlantic area,
  sun 30 deg high: glint visible, horizon steady), the LZ-40 coast (shoreline at the imagery's coast, no water on the
  plateau), Starbase lagoon, a night view (no glint, no black blotches).
- [ ] **Step 6: Commit + push** `feat(game): calm sea with Fresnel, glint and gentle waves`.

### Task 8: Moon terrain (LDEM_16 relief, Mons Mouton site)

**Files:**
- Create: `game/shaders/terrain_moon.gdshader`
- Modify: `game/world/world.gd` (Moon quadtree), `game/tests/render_check.gd` (`--body=moon` for ground views)
- Test: `game/tests/terrain_quadtree_test.gd`

**Interfaces:**
- A second `TerrainQuadtree` under `moon` with `HeightSource` body "moon" (`MOON_ME`, the frame of LOLA and of
  `StarsimCore.moon_fixed_to_gcrf`), no imagery (`moon_color.png` via CUSTOM1 global UV), radius 1737.4 km.
- `terrain_moon.gdshader`: the Lommel-Seeliger + Earth shadow + earthshine model of `moon.gdshader` on the relief
  normal, morph include shared. Cast shadows: not modelled (documented: long polar shadows belong to a later part).
- The lunar landing (`lunar_landing_mission`) and the renderer share the cached `LunarTerrain` (Task 1).

- [ ] **Step 1: Tests**: `moon finest mesh matches the core` (needs data: at the IM-2 site and a tile-edge point the
  mesh surface equals `surface_samples("moon", ...)` within 0.05 m), `moon refines to the 20 m tile` (camera 10 m
  above the site: leaf spacing <= 20 m; above a far-side point: stops at the LDEM_16 spacing).
- [ ] **Step 2: Run**: FAIL. **Step 3: Implement.** **Step 4: Run** tests + smoke test (lunar mission part): PASS.
- [ ] **Step 5: Render checks**, read each: Mons Mouton ground view, a 100 km lunar orbit view (limb relief), the
  lunar landing at `--mission=hls_lunar_llo --at_event=landed,-30` and `--at_event=landed,5` (lander above the
  drawn surface, legs touching it after touchdown).
- [ ] **Step 6: Commit + push** `feat(game): lunar relief and the Mons Mouton site in the terrain renderer`.

### Task 9: Performance, docs and the VR check

**Files:**
- Create: `game/tests/terrain_perf.gd`
- Modify: `docs/subsystems.md` (new section "Terrain rendering": tiling, LOD, morph, loading budget, imagery sources
  and colour match, sea model and its placeholders, numbers; update "Earth terrain and geoid" for Task 3 and the
  cache), `CLAUDE.md` (Known issues: GroundPatch text replaced; game structure: `game/world/terrain/`; implemented
  list: leg site radius), `docs/ansichten.md` (ground detail in the external view), `config/README.md`
- Modify: the [E] constants of Task 6 / 7 if the measurement demands

**Interfaces:**
- `terrain_perf.gd` (GPU, desktop window 2560 x 1440): scripted camera descent from 400 km to 2 m above OLP-1 in
  60 s, then a 360 deg pan at the pad, then a Tab-style jump to the Cape; logs per second: frame time p50/p99/max,
  nodes drawn/loading, uploads, VRAM (`Performance.RENDER_VIDEO_MEM_USED`); summary line. Desktop proxy gate
  [E]: p99 <= 8 ms, max <= 16 ms outside the jump frame.

- [ ] **Step 1: Run** `terrain_perf.gd`, log `tmp/m8b_t9_perf.log`; tune `LOD_FACTOR`, `UPLOADS_PER_FRAME`, node
  cells until the proxy gate holds; record the numbers in the ledger and `docs/subsystems.md`.
- [ ] **Step 2: Run** all GDScript terrain tests, smoke test, and `-NoBindings -TestFilter "[terrain_cache],[surface_samples],[earth_terrain]"`: PASS.
- [ ] **Step 3: Docs** as listed; memory notes (`project-m8-terrain`, `project-milestone-order`: Plan B done, next
  the sea-state / deck-motion part).
- [ ] **Step 4: Commit + push** `docs: terrain rendering`.
- [ ] **Step 5: Ask the user** for the VR check (`Start-VR.cmd`): launch from OLP-1 and look around at the pad,
  descend view at the Cape, a drone-ship landing at sea, the Mons Mouton landing - 90 Hz held, no popping, no
  jitter, ground under the vehicles. Fix what the check finds before closing the plan.
