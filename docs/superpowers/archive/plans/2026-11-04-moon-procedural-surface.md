# Moon Procedural Near-Ground Surface Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or
> superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deterministic procedural lunar surface detail (small craters, sub-LOLA regolith roughness, rocks) in the
core, shared by the leg contact, the landing planner and the renderer; drawn seamlessly by screen size with
procedural rock/regolith textures, Hapke reflectance and near-field Sun shadows.

**Architecture:** A stateless core model `environment::LunarSurfaceDetail` (integer hash, cube-sphere cells per size
octave) is composed over `environment::LunarTerrain` into `environment::DetailedLunarTerrain`, which every consumer
queries with a detail limit (physics: full detail; a terrain node: its vertex spacing; the renderer: rocks above a
size). Footpads become discs with a max-filtered support; the existing touchdown selection gains rock/bell checks.
The game streams the detail through thin bindings, draws rocks as instanced meshes chosen by projected size, shades
with Hapke plus a statistical term for everything unresolved, and adds a Sun shadow map around the camera.

**Tech Stack:** C++20 core (Catch2), godot-cpp GDExtension, Godot 4 double (GDScript, .gdshader), Python (numpy) for
the texture bake.

**Spec:** `docs/superpowers/archive/specs/2026-11-04-moon-procedural-surface-design.md` (read it before any task).

## Global Constraints
- Architecture: generator, contact and planner in `core` (no Godot, float64); bindings thin; no physics in GDScript.
- Deterministic: integer hash only, no `<random>` distributions, no time / address seeds; identical across runs,
  machines and threads.
- Every external number in config or `constants.hpp` with source and [S]/[E]/[D]; data sources with licence in
  `tools/data_pipeline/sources.toml` (CREDITS regenerated).
- Vehicle config changes need the user's decision: the only one this plan proposes is
  `[stage.landing_legs] foot_pad_radius_m` (Task 4) - ask before committing it.
- Detail off (fidelity flags) = today's LOLA-only surface bit-for-bit, incl. the 10 m `normal_fixed` step and the
  point footpad query.
- Seamless rule: geometry from ~1-2 px projected size (`ROCK_PIXEL_THRESHOLD` = 1.5 px [E]); unresolved detail
  statistically in the shading; no popping; under load the threshold rises smoothly, never a hard count cut.
- Finest drawn scale ~1 cm; solar disc 0.53 deg; VR budget 11.1 ms frame, GPU <= 9 ms (docs/subsystems/vr-performance.md).
- Surface static (no plume interaction). Earth terrain untouched.
- New Catch2 tag `[moon_surface]` on every new case; heavy cases also `[slow]`. Test per change with
  `tools/build.ps1 -Fast -TestFilter "[moon_surface],<touched tags>"`, before push the same with `[slow]`.
- Images to `screenshots/moon_surface/`; progress PNGs to the user after each visual task.
- Docs: new `docs/subsystems/moon-surface.md` (current state), updates to `terrain-rendering.md`, `lunar.md`,
  `known-issues.md` in the task that changes them. English in repo, German UI strings.
- Commit + `git push origin main` after each green task (Conventional Commits).

## Review Focus
1. Cube face edges and corners: craters, roughness and rocks continuous across faces (no seam, no duplicate or lost
   rock at an edge) - tests in Tasks 2 and 3.
2. The 1 km tile-to-global blend (20 m LOLA tile -> 1.9 km grid): the procedural band start follows the blend, no
   height or roughness step - test in Task 2.
3. Poles (the site is at -84.8 deg; exactly -90 deg is reachable by the free camera): no degenerate cells or NaN -
   tests in Tasks 2 and 3.
4. Camera jumps (teleport to the far side, time-lapse, altitude change of km in one frame): rock generation stays on
   workers, no frame hitch above budget, no rock pile-up - measured in Task 11.
5. Sun below the local horizon (earthshine only): the near-field shadow and the statistical shadow share must not
   darken the earthshine-lit ground - render check in Tasks 10 and 12.

---

### Task 1: Measured statistics and their config (Opus)

**Files:**
- Create: `config/moon_surface.toml`, `core/include/starsim/config/moon_surface_config.hpp`,
  `core/src/config/moon_surface_config.cpp`, `core/tests/test_moon_surface_config.cpp`
- Modify: `tools/data_pipeline/sources.toml` (literature entries; Diviner rock abundance only after licence check),
  `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`
- Create: `docs/subsystems/moon-surface.md` (sources section; filled further by later tasks), row in `docs/subsystems.md`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::config {
  struct SizeFrequency {               // cumulative N(>D) per m^2; form chosen from the source (power law or exponential)
      std::string form;                // "power" | "exponential"
      double reference_diameter_m, density_per_m2, exponent;  // power: N = density (D/ref)^-exponent; exp: density exp(-exponent D)
      double min_diameter_m, max_diameter_m;
      double cumulative_density_per_m2(double diameter_m) const;
  };
  struct CraterPopulation { SizeFrequency sfd; double depth_to_diameter_fresh, depth_to_diameter_degraded;
                            double rim_height_to_diameter, degraded_fraction; };
  struct RegolithRoughness { double hurst_exponent, rms_height_at_baseline_m, baseline_m; };
  struct RockPopulation { SizeFrequency sfd; double axis_b_to_a_mean, axis_c_to_a_mean, axis_ratio_sd;
                          double burial_fraction_mean, burial_fraction_sd, albedo_to_regolith;
                          std::string abundance_source; double abundance_scale; };  // "uniform" | "diviner"
  struct HapkeParameters { double w, b, c, bs0, hs, bc0, hc, theta_bar_rad, porosity_k; };
  struct MoonSurfaceConfig { CraterPopulation craters; RegolithRoughness roughness; RockPopulation rocks;
                             HapkeParameters hapke; };
  MoonSurfaceConfig load_moon_surface_config(const std::filesystem::path& file);  // throws German message on bad keys
  }
  ```

- [ ] **Step 1:** Research and record the numbers (each with source, [S]/[E]/[D] and the scale range it covers) in
  `config/moon_surface.toml` and the sources section of `docs/subsystems/moon-surface.md`: small-crater SFD down to
  the cm scale (production / equilibrium, e.g. Neukum PF, Xiao & Werner 2015), fresh/degraded depth-to-diameter and
  rim height; LOLA roughness continuation (Hurst exponent, RMS at a baseline, e.g. Rosenburg et al. 2011); rock SFD
  (LRO NAC boulder counts, Surveyor / Apollo rock counts, e.g. Bart & Melosh 2010, Li et al.), axis ratios, burial,
  rock/regolith albedo ratio; Hapke parameters (Sato et al. 2014 / Hapke 2012). Check the Diviner rock-abundance
  licence; if free, add it to `sources.toml` and set `abundance_source = "diviner"` (download in Task 3), else
  `"uniform"` with the documented model.
- [ ] **Step 2: Failing tests** `[moon_surface][config]`: "The shipped Moon surface config loads with sources-sane
  values" (every field finite; 0.5 < hurst < 1; 0 < w < 1; `rocks.sfd.min_diameter_m <= 0.01`;
  `cumulative_density_per_m2` strictly decreasing over [min, max]); "A missing key names it in German" (loader throws,
  message contains the key).
- [ ] **Step 3:** Implement the loader (pattern of the other `config/*_config.cpp` TOML loaders).
- [ ] **Step 4:** `tools/build.ps1 -NoBindings -Fast -TestFilter "[moon_surface]"` -> PASS.
- [ ] **Step 5:** Commit `feat(config): measured lunar surface statistics (craters, roughness, rocks, Hapke)`, push.

### Task 2: Deterministic crater and roughness detail

**Files:**
- Create: `core/include/starsim/environment/surface_hash.hpp`, `core/include/starsim/environment/cube_face_cell.hpp`,
  `core/src/environment/cube_face_cell.cpp`, `core/include/starsim/environment/lunar_surface_detail.hpp`,
  `core/src/environment/lunar_surface_detail.cpp`, `core/tests/test_lunar_surface_detail.cpp`
- Modify: `core/include/starsim/environment/lunar_terrain.hpp` (add `double tile_weight(const math::Vec3& unit) const`
  = the blend weight already used inside, 1 in a tile, 0 on the global grid)

**Interfaces:**
- Consumes: Task 1 `config::MoonSurfaceConfig`.
- Produces:
  ```cpp
  namespace starsim::environment {
  std::uint64_t surface_hash(std::uint64_t a, std::uint64_t b, std::uint64_t c, std::uint64_t d);  // portable mix
  double hash_unit(std::uint64_t h);                                                                // [0, 1)
  struct FaceCell { int face; int level; std::int64_t i, j; };
  FaceCell face_cell_of(const math::Vec3& unit, int level);           // same mapping as game/world/terrain/cube_sphere.gd
  math::Vec3 face_cell_point(const FaceCell& c, double s01, double t01);  // unit direction inside the cell
  std::array<FaceCell, 9> face_cell_neighbourhood(const FaceCell& c);  // 3x3, crosses face edges
  int face_cell_level_for_size(double cell_size_m, double sphere_radius_m);
  class LunarSurfaceDetail {
  public:
      LunarSurfaceDetail(config::MoonSurfaceConfig config, std::shared_ptr<const LunarTerrain> lola);
      // Relief added to LOLA at `unit` with every feature shorter than min_wavelength_m averaged out (0: full detail).
      double relief_m(const math::Vec3& unit, double min_wavelength_m) const;
      double crater_relief_m(const math::Vec3& unit, double min_wavelength_m) const;
      double roughness_relief_m(const math::Vec3& unit, double min_wavelength_m) const;
      double band_start_m(const math::Vec3& unit) const;  // LOLA resolution blended by tile_weight
      double finest_wavelength_m() const;                  // ~0.01 m
      const config::MoonSurfaceConfig& config() const;
  };
  }
  ```

- [ ] **Step 1: Failing tests** `[moon_surface][environment]`:
  - "Detail is deterministic across calls and threads" (10^4 points, 4 threads, bitwise equal).
  - "Crater counts per size octave match the configured SFD" (count crater centres over >= 1000 expected per octave
    from 1 m to 50 m in a site patch; each within 3 sigma Poisson).
  - "Roughness continues the LOLA spectrum" (RMS height difference at lags 0.1, 1, 10 m follows
    `rms_height_at_baseline_m * (lag/baseline)^hurst` within 15 %).
  - "Band-limited relief is the average of the finer relief" (for limits 10, 1, 0.1 m:
    |relief(limit) - mean(relief(0) over a limit x limit square)| <= 0.1 x RMS of the removed band).
  - "No seam at cube face edges and corners" (points 1 mm apart across an edge / at a corner: relief difference
    <= the 1 mm slope bound of the full-detail field).
  - "The procedural band follows the tile blend" (across the 1 km blend band_start_m is continuous; relief has no
    step > the local 1 cm slope bound).
  - "Poles are regular" (lat -90, -89.9999, +90: finite, continuous).
  - "No band below the LOLA resolution is counted twice" (crater population starts above band_start_m only where
    LOLA does not already resolve the crater: craters with D > 5 x band_start_m are not generated).
- [ ] **Step 2:** Run -> FAIL (missing types).
- [ ] **Step 3:** Implement. Per size octave a cell level (`face_cell_level_for_size`) with an expected O(1-10)
  features per cell; feature parameters from `surface_hash(face, level, i, j)`; queries visit the 3x3 neighbourhood.
  Band-limiting: features smaller than the limit enter with their area-averaged contribution (craters: the mean of
  bowl + rim over the limit cell, ~0 for a full crater) and roughness octaves below the limit are dropped with a
  smooth octave weight (no hard cut), so coarse = mean of fine.
- [ ] **Step 4:** Tests PASS; also `[terrain]` and `[terrain_horizon]` unchanged.
- [ ] **Step 5:** Commit `feat(environment): deterministic lunar crater and regolith detail below the LOLA grid`, push.

### Task 3: Rocks (templates, population, rock tops)

**Files:**
- Create: `core/include/starsim/environment/rock_templates.hpp`, `core/src/environment/rock_templates.cpp`,
  `core/include/starsim/environment/lunar_rocks.hpp`, `core/src/environment/lunar_rocks.cpp`,
  `core/tests/test_lunar_rocks.cpp`
- Modify: `tools/data_pipeline/fetch.py` + `sources.toml` only if Task 1 chose `"diviner"` (fetch the rock-abundance
  map into `data/terrain/`)

**Interfaces:**
- Consumes: Task 2 hash, cells, `LunarSurfaceDetail` (relief for the rock's base height).
- Produces:
  ```cpp
  namespace starsim::environment {
  struct TemplateMesh { std::vector<math::Vec3> vertices, normals; std::vector<int> indices; };
  class RockTemplates {        // star-shaped seeded radial functions r(dir) on a unit rock
  public:
      static constexpr int count = 16;   // [E]
      static constexpr int lods = 4;     // [E]
      static const RockTemplates& instance();
      double radius(int id, const math::Vec3& dir_local) const;   // unit-size radial function
      const TemplateMesh& mesh(int id, int lod) const;
  };
  struct Rock {
      std::uint64_t id; math::Vec3 base_unit; double diameter_m;
      math::Vec3 axes_m;          // a, b, c semi-axes after the template's scale
      math::Mat3 local_to_fixed;  // orientation (body-fixed)
      double base_height_m;       // ground (LOLA + relief) at base_unit
      double burial_m;            // depth of the rock's centre below base_height_m
      int template_id; double albedo_factor;
  };
  class LunarRocks {
  public:
      explicit LunarRocks(std::shared_ptr<const LunarSurfaceDetail> detail);
      std::vector<Rock> rocks_in_cell(const FaceCell& cell, double min_diameter_m) const;
      std::vector<Rock> rocks_near(const math::Vec3& unit, double radius_m, double min_diameter_m) const;
      // Top of the rock above the sphere along `unit` (nullopt: unit misses the rock), and the rock's outward normal there.
      std::optional<std::pair<double, math::Vec3>> rock_top(const Rock& rock, const math::Vec3& unit) const;
      double cover_fraction(double min_diameter_m, double max_diameter_m) const;   // area share, for the shading
  };
  }
  ```

- [ ] **Step 1: Failing tests** `[moon_surface][environment]`:
  - "Rock counts per size bin match the configured SFD" (>= 1000 expected per bin, 3 sigma).
  - "Every rock belongs to exactly one cell" (union of `rocks_in_cell` over a 3x3 block across a face edge: no
    duplicate id, equals `rocks_near` over the same area).
  - "A rock top lies above the ground and below its full height" (`base_height_m - burial_m + c` bound).
  - "Rock shapes are star-shaped and closed" (each template mesh watertight; `radius` > 0 for 10^4 directions;
    mesh vertex radii equal `radius` within 1 % at LOD 0).
  - "Rocks are deterministic and pole-regular" (bitwise equal across threads; finite at lat -90).
  - "Cover fraction equals the area of the generated rocks" (patch measure vs `cover_fraction` within 5 %).
- [ ] **Step 2:** Run -> FAIL.
- [ ] **Step 3:** Implement (cells per size octave like Task 2; abundance scale from the config / Diviner map).
- [ ] **Step 4:** PASS.
- [ ] **Step 5:** Commit `feat(environment): deterministic lunar rocks from measured size-frequency data`, push.

### Task 4: Composed physics surface, disc footpads, fidelity flags

**Files:**
- Create: `core/include/starsim/environment/detailed_lunar_terrain.hpp`,
  `core/src/environment/detailed_lunar_terrain.cpp`, `core/tests/test_detailed_lunar_terrain.cpp`
- Modify: `core/include/starsim/contact/landing_surface.hpp` / `.cpp` (`below_disc`), `core/src/contact/landing_legs.cpp`
  (feet use `below_disc`), `core/include/starsim/sim/terrain_cache.hpp` / `.cpp`, `core/include/starsim/sim/flight_session.hpp`
  (flags), config vehicle loader for `foot_pad_radius_m`, `core/tests/test_terrain_contact.cpp`

**Interfaces:**
- Consumes: Tasks 2-3.
- Produces:
  ```cpp
  namespace starsim::environment {
  struct SurfaceDetailOptions { bool detail = true; bool rocks = true; };   // fidelity flags moon_surface_detail / moon_rocks
  class DetailedLunarTerrain final : public Terrain {
  public:
      DetailedLunarTerrain(std::shared_ptr<const LunarTerrain> lola, std::shared_ptr<const LunarSurfaceDetail> detail,
                           std::shared_ptr<const LunarRocks> rocks, SurfaceDetailOptions options);
      // Terrain overrides: full detail incl. rock tops; resolution / finest_resolution_within_m report finest_wavelength_m
      // where detail is on; normal_fixed over normal_step_m(fixed) (footpad scale with detail, LunarTerrain's 10 m without).
      double height_limited_m(const math::Vec3& unit, double min_wavelength_m, double min_rock_diameter_m) const;
      struct DiscSupport { double height_m; math::Vec3 normal_fixed; };
      DiscSupport support_under_disc(const math::Vec3& centre_unit, const math::Vec3& up_fixed, double radius_m) const;
      const LunarTerrain& lola() const; const LunarSurfaceDetail& detail() const; const LunarRocks& rocks() const;
  };
  }
  namespace starsim::contact {  // SurfaceField gains
      virtual SurfaceSupport below_disc(double east_m, double north_m, double radius_m) const { return below(east_m, north_m); }
  }
  namespace starsim::sim {
  std::shared_ptr<const environment::Terrain> shared_moon_surface(const std::filesystem::path& config_dir,
      const std::filesystem::path& data_dir, environment::SurfaceDetailOptions options);  // cached like shared_lunar_terrain
  }
  ```
  `FlightSession::set_moon_surface_detail(bool)`, `set_moon_rocks(bool)` (fidelity flags, default on).
  Vehicle config `[stage.landing_legs] foot_pad_radius_m` (0 = point foot, today's behaviour).

- [ ] **Step 0:** Ask the user (via the controller) to approve `foot_pad_radius_m = 1.0` [E] for `starship_hls.toml`
  (matches the drawn pad, detail file `pad_radius_m = 1.0`); until approved use 0 in config and a test-only value.
- [ ] **Step 1: Failing tests** `[moon_surface][terrain][contact]`:
  - "Detail off reproduces LunarTerrain bit-for-bit" (heights, normals, resolutions at 10^4 points equal with `==`).
  - "A footpad disc stands on the highest support under it" (synthetic rock under the pad edge: height = rock top,
    normal = rock normal there; on flat ground = point query).
  - "A foot on a rock compresses that leg; a foot in a small crater extends" (`landing_legs` foot compression sign
    and size vs geometry within 1 mm).
  - "Full-detail height agrees with the band-limited height at its limit plus the removed band mean" (consistency).
  - "Physics query cost" (six `support_under_disc` per tick < 50 us total, measured, `[slow]` only if it exceeds).
  - Existing `[contact][legs]`, `[terrain]`, `[lunar]` fast cases stay green with detail OFF and ON.
- [ ] **Step 2:** Run -> FAIL.
- [ ] **Step 3:** Implement; `FlightSession::set_moon_terrain` receives the composed terrain; a per-site cell cache is
  allowed for the six pads.
- [ ] **Step 4:** PASS; then `-TestFilter "[lunar],[contact],[terrain],[moon_surface]"` with `[slow]`.
- [ ] **Step 5:** Docs (`lunar.md` contact section, `moon-surface.md` physics), commit
  `feat(contact): lunar footpads stand on procedural craters and rocks (disc support, fidelity flags)`, push.

### Task 5: Touchdown selection sees rocks and craters (Opus)

**Files:**
- Modify: `core/include/starsim/mission/lunar_descent.hpp`, `core/src/mission/lunar_touchdown.cpp`,
  `core/src/mission/lunar_descent.cpp:233`, `core/src/sim/lunar_landing_mission.cpp:79`, any live re-selection caller
  (grep `select_touchdown_point`), `core/tests/test_lunar_descent.cpp` (or a new `test_lunar_touchdown_hazards.cpp`)

**Interfaces:**
- Consumes: Task 4 `DetailedLunarTerrain` (as `const environment::Terrain&` plus `support_under_disc`).
- Produces:
  ```cpp
  struct TouchdownHazardLimits {
      double foot_pad_radius_m;     // from the vehicle config
      double max_foot_step_m;       // per-foot height spread the legs absorb (from stroke_m, documented)
      double bell_clearance_m;      // lowest engine exit above the footpad plane (from vehicle geometry) minus margin
  };
  std::optional<TouchdownPoint> select_touchdown_point(const environment::Terrain& terrain, double latitude_rad,
      double longitude_rad, double approach_azimuth_rad, double footprint_radius_m, const LunarDescentParameters& p,
      const TouchdownHazardLimits& hazards);
  TouchdownHazardLimits touchdown_hazard_limits(const config::StageConfig& stage);
  ```
  `TouchdownPoint` gains `double max_foot_step_m`, `double min_bell_clearance_m` (diagnostics).

- [ ] **Step 1: Failing tests** `[moon_surface][lunar]`:
  - "A rock under a planned footpad moves the target to the nearest safe spot" (synthetic field; offset > 0,
    all limits met).
  - "Detail off selects the same point as before" (bitwise equal to today's result at the IM-2 site).
  - "The IM-2 site with full detail has a safe point within the search radius" (and its offset reported).
- [ ] **Step 2:** Run -> FAIL.
- [ ] **Step 3:** Implement: candidate grid as today, plus per foot `support_under_disc` step spread, slope over the
  footprint, rock tops under the bells vs `bell_clearance_m`. A re-selection after a dispersion uses the same
  function on the live state.
- [ ] **Step 4:** PASS; then shipped lunar missions: `-TestFilter "[lunar]"` with `[slow]` and the lunar landing
  campaign. A red is a planner defect - fix the planner, never relax the contact or remove rocks.
- [ ] **Step 5:** Docs (`lunar.md`), commit `feat(lunar): touchdown selection avoids rocks and checks bell clearance`, push.

### Task 6: Bindings for the detailed surface

**Files:**
- Modify: `bindings/src/starsim_core.cpp` / `.hpp`, `bindings/src/flight_sim.cpp` (flags), `game/tests/smoke_test.gd`

**Interfaces:**
- Consumes: Tasks 2-4.
- Produces (GDScript-visible, all float64 positions Moon-fixed):
  - `StarsimCore.surface_samples_detail(body, lats_deg, lons_deg, min_wavelength_m, min_rock_diameter_m) -> Dictionary`
    (same keys as `surface_samples`; band-limited heights; rock tops included above `min_rock_diameter_m`, INF = none).
  - `StarsimCore.moon_rocks(face, level, i, j, min_diameter_m) -> Dictionary{ids: PackedInt64Array,
    base_fixed_m: PackedFloat64Array (3 per rock), basis: PackedFloat64Array (9 per rock, local_to_fixed),
    axes_m: PackedFloat64Array (3), burial_m, base_height_m, albedo: PackedFloat64Array, template: PackedInt32Array}`.
  - `StarsimCore.rock_template_mesh(id, lod) -> Dictionary{vertices: PackedVector3Array, normals, indices: PackedInt32Array}`.
  - `StarsimCore.moon_surface_statistics() -> Dictionary` (Hapke parameters, SFD parameters, roughness, cover
    function samples) - the shader uniforms come from here, nothing duplicated in GDScript.
  - `surface_finest_resolution_m("moon", ...)` reports the procedural limit when detail is on.
  - `FlightSim.set_moon_surface_detail(bool)`, `set_moon_rocks(bool)`.
- [ ] **Step 1:** Smoke-test checks (failing first): the four calls exist and return consistent sizes; detail off ->
  `surface_samples_detail` equals `surface_samples`.
- [ ] **Step 2:** Implement (thin conversion only), `tools/build.ps1 -TestFilter "[moon_surface]"`, smoke test PASS.
- [ ] **Step 3:** Commit `feat(bindings): procedural lunar surface, rocks and statistics for the game`, push.

### Task 7: Terrain mesh down to the cm scale

**Files:**
- Modify: `game/world/terrain/height_source.gd`, `terrain_node_builder.gd`, `terrain_quadtree.gd`,
  `docs/subsystems/terrain-rendering.md`
- Test: `game/tests/moon_surface_check.gd` (new; headless consistency checks)

**Interfaces:**
- Consumes: Task 6 `surface_samples_detail` (min_wavelength = node vertex spacing, rocks excluded: INF),
  `surface_finest_resolution_m`.
- Produces: Moon nodes split to the procedural limit near the camera; horizon maps baked only for nodes at or above
  the LOLA data level - deeper nodes sample their ancestor's horizon texture (scale/offset uniforms).
- [ ] **Step 1:** `moon_surface_check.gd` (failing first): node at level 18 at the IM-2 site - mesh vertices equal
  core band-limited heights within 0.004 m; a parent/child seam morphs without gap; deeper nodes do not bake horizon
  maps (count of bakes).
- [ ] **Step 2:** Implement; run the check headless; `render_check.gd --body=moon` at the site with `--climb=2,20,200,2000`
  -> PNGs in `screenshots/moon_surface/task7/`, no cracks, no shimmer; GPU/worker time logged.
- [ ] **Step 3:** Docs, commit `feat(terrain): Moon mesh refines to the procedural detail near the camera`, push;
  send PNGs.

### Task 8: Procedural regolith and rock textures

**Files:**
- Create: `tools/moon_textures/bake_moon_detail.py`, `tools/moon_textures/test_bake_moon_detail.py`
- Modify: `tools/data_pipeline/fetch.py` (target `moon_detail`, writes `data/textures/moon_detail/`), release
  packaging list (docs/setup.md, packaging script) so the textures ship

**Interfaces:**
- Consumes: `config/moon_surface.toml` (roughness, crater SFD below the mesh scale, rock albedo).
- Produces: `data/textures/moon_detail/regolith_{height,normal}_{s0,s1,s2}.png` (tileable, mipmapped by Godot; scales
  s0..s2 cover ~1 cm .. the mesh spacing, exact sizes in the bake script header) and
  `rock_{albedo,normal}_{0..3}.png` (triplanar set). Deterministic (fixed seeds), no downloaded imagery.
- [ ] **Step 1: Failing tests** (pytest): bake twice -> byte-identical; maps tile seamlessly (edge difference <= 1 LSB);
  height-map RMS spectrum matches the config roughness over its scale band within 15 %; mean albedo variation 1.0
  within 1 % (detail never shifts the WAC mean).
- [ ] **Step 2:** Implement, run `python -m pytest tools/moon_textures`, then `python tools/data_pipeline/fetch.py moon_detail`.
- [ ] **Step 3:** Commit `feat(tools): baked procedural regolith and rock textures`, push.

### Task 9: Hapke reflectance (Opus)

**Files:**
- Create: `core/include/starsim/environment/hapke.hpp`, `core/src/environment/hapke.cpp`, `core/tests/test_hapke.cpp`,
  `game/shaders/moon_hapke.gdshaderinc`
- Modify: `game/shaders/moon_regolith.gdshaderinc` (call Hapke; keep Lommel-Seeliger behind the flag),
  `terrain_moon.gdshader`, `moon.gdshader`, game setting `moon_brdf` ("hapke" default | "lommel_seeliger")

**Interfaces:**
- Produces: `double hapke_reflectance(const config::HapkeParameters& p, double mu0, double mu, double phase_rad,
  double theta_bar_rad)` (radiance factor I/F; core reference implementation); the shader function mirrors it with
  `theta_bar` as a uniform per material (scale-dependent, Task 10).
- [ ] **Step 1: Failing tests** `[moon_surface][hapke]`: published check values of the chosen parameter set
  reproduced within 1 %; disk-integrated phase curve vs the published reference (e.g. ROLO / Lane & Irvine) within
  0.1 mag over phase 5-120 deg; full-Moon disk brightness vs today's Lommel-Seeliger globe within 3 %.
- [ ] **Step 2:** Implement core + shader; a shader-parity check in `moon_surface_check.gd` (a flat patch rendered at
  three geometries vs `hapke_reflectance` through a binding or a printed table, within 2 %).
- [ ] **Step 3:** Render far globe + site before/after (`screenshots/moon_surface/task9/`), commit
  `feat(render): Hapke reflectance for the Moon (globe, terrain)`, push; send PNGs.

### Task 10: Regolith shading with the statistical term

**Files:**
- Create: `game/shaders/moon_detail.gdshaderinc`
- Modify: `game/shaders/terrain_moon.gdshader`, `game/world/world.gd` (texture + statistics uniforms from
  `moon_surface_statistics`)

**Interfaces:**
- Consumes: Task 8 textures, Task 9 Hapke, Task 6 statistics.
- Produces: shader function `moon_detail_shade(...)` - stochastic hex tiling of the regolith maps faded in by pixel
  footprint (mip-consistent); unresolved rocks/craters below the current geometry threshold
  `d_geo = ROCK_PIXEL_THRESHOLD x pixel footprint` enter as albedo share (`cover_fraction(min, d_geo)` x
  `albedo_to_regolith`), shadow share for the Sun elevation and an increased `theta_bar`; resolved ones are excluded.
- [ ] **Step 1:** Failing check in `moon_surface_check.gd`: mean radiance of a 50 m patch rendered with and without
  geometry at the threshold differs <= 2 %; earthshine-only patch unchanged by the shadow share.
- [ ] **Step 2:** Implement; low-Sun renders at the IM-2 site next to LRO NAC / Apollo references (in
  `screenshots/moon_surface/task10/`), no visible tiling.
- [ ] **Step 3:** Commit `feat(render): procedural regolith detail with statistical unresolved rocks and craters`, push;
  send PNGs.

### Task 11: Rocks drawn by projected size

**Files:**
- Create: `game/world/terrain/moon_rock_layer.gd`, `game/shaders/moon_rock.gdshader`
- Modify: `terrain_node_builder.gd` (rock list per node on the worker), `game/world/world.gd`, `game/tests/render_check.gd`
  (`--approach=<from_m,to_m,steps>` series), `game/tests/moon_surface_check.gd`

**Interfaces:**
- Consumes: Task 6 `moon_rocks`, `rock_template_mesh`; Tasks 8-10 textures and shading.
- Produces: per node a MultiMesh per template LOD; selection per frame from one reference point between the eyes:
  geometry when `diameter / distance >= ROCK_PIXEL_THRESHOLD x pixel_angle` (pixel angle from the active render target
  and FOV); growth band to `2 x ROCK_PIXEL_THRESHOLD` (burial from full to nominal); budget control raises the
  threshold smoothly (`rock_threshold_scale`, logged); pebbles below `PEBBLE_GEOMETRY_LIMIT_M` [E, measured] go to the
  regolith height-map relief instead of instances. Rock shader: triplanar textures, Hapke, dust fillet / burial
  blend at the base.
- [ ] **Step 1:** Failing checks: drawn rock transforms equal the core's (ids, positions within 1 mm); the radiance
  check of Task 10 holds with real rocks crossing the threshold; a camera jump of 10 km keeps the main thread under
  5 ms (generation on workers).
- [ ] **Step 2:** Implement; descent and approach series (`--climb`, `--approach`) at the site, low Sun -> PNG series
  in `screenshots/moon_surface/task11/`; no popping between frames (frame-difference check on the series).
- [ ] **Step 3:** Commit `feat(render): lunar rocks drawn by projected size, seamless with the regolith`, push; send PNGs.

### Task 12: Near-field Sun shadows

**Files:**
- Create: `game/world/moon_near_shadow.gd` (+ shader include if a custom pass is chosen)
- Modify: `game/world/world.gd` (`moon_light`), `terrain_moon.gdshader`, `moon_rock.gdshader`, vehicle / structure lit
  materials (receive), `docs/subsystems/moon-surface.md`, `docs/known-issues.md` (close "vehicles cast no shadow on the
  terrain" for the Moon)

**Interfaces:**
- Consumes: Tasks 7, 11.
- Produces: Sun visibility near the camera = min(horizon-map visibility, near-field shadow), solar-disc softening
  0.53 deg; casters rocks, near terrain, vehicles, structures; receivers terrain, rocks, vehicles; beyond range the
  statistical shadow share (Task 10) blended in; both VR eyes; fidelity setting `moon_near_shadows`. Technique (Godot
  directional shadows or a custom pass) chosen by measured VR cost and documented.
- [ ] **Step 1:** Failing checks: a test rock of 2 m at Sun elevation 7 deg casts a shadow 16 m +- 1 m long; HLS on the
  pad shadows the ground; earthshine-only scene unchanged.
- [ ] **Step 2:** Implement; renders at landing and on the ground (`screenshots/moon_surface/task12/`).
- [ ] **Step 3:** Commit `feat(render): near-field Sun shadows on the Moon (rocks, terrain, vehicles)`, push; send PNGs.

### Task 13: VR budget, seamlessness acceptance, docs

**Files:**
- Modify: `docs/subsystems/moon-surface.md`, `terrain-rendering.md`, `vr-performance.md`, `known-issues.md`, `lunar.md`,
  `docs/setup.md` (measured waits)

- [ ] **Step 1:** GPU/main-thread timings at the documented VR render target (`render_check.gd --stereo` mock, `--gpu_time`)
  on the ground, at 100 m and 2 km at the site; within budget, else apply the cut order (pebbles to texture, then
  threshold scale) and re-measure.
- [ ] **Step 2:** Final series: descent from 15 km to touchdown and a ground pan, PNGs to the user; tests
  `[moon_surface],[lunar],[contact],[terrain],[terrain_horizon]` with `[slow]`; `-Nightly` in the background.
- [ ] **Step 3:** Docs current state + known limits (plume interaction later); commit `docs: Moon procedural surface
  (state, budgets, limits)`, push. Then final review (Opus) and the user's VR check.
