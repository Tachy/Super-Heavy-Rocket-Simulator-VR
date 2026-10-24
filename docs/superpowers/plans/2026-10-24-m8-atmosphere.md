# M8 Atmosphere Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A physically computed Earth atmosphere (sky, limb, aerial perspective, ground and vehicle light) that is
continuous from the pad into space and correct in colour, within the VR frame budget.

**Architecture:** The core (C++, float64) ports Bruneton 2017's precomputation: spectral optical inputs (US76 number
density, Rayleigh from the refractive index, ozone, aerosols, E-490 Sun) -> transmittance / scattering / irradiance
tables in luminance, cached in `data/atmosphere/`, plus a CPU evaluation of the same model. The GDExtension hands
the tables to Godot. One shader include ports Bruneton's runtime lookups and is used by the sky shader, the Earth
surfaces, Moon / Sun / stars and the vehicles' materials (`FOG` output). The WGS84 ellipsoid is mapped onto a sphere
by stretching the polar axis by a/b.

**Tech Stack:** C++20 core (CMake/Ninja, Catch2, toml++), godot-cpp GDExtension, Godot 4.7 double build (Forward+,
D3D12), Godot shading language, PowerShell build scripts.

**Spec:** `docs/superpowers/specs/2026-10-24-m8-atmosphere-design.md`

## Global Constraints

- SI units in the core; names carry frame and unit (`position_itrf_m`, `wavelength_m`); angles in radians.
- `core` has no Godot dependency; physics/precomputation never in GDScript; the binding stays thin.
- Constants and tabulated data carry their source (`constants.hpp` rule; data headers with the citation).
- Provenance tags [S]/[E]/[D] on every config value (`config/README.md`).
- Bruneton's code is BSD-3: every ported file carries his copyright + licence notice; credit entry in
  `tools/data_pipeline/sources.toml` (CREDITS.md regenerated).
- Reference source: clone `https://github.com/ebruneton/precomputed_atmospheric_scattering` into
  `external/precomputed_atmospheric_scattering` (git-ignored) and port from `atmosphere/functions.glsl`,
  `atmosphere/model.cc`, `atmosphere/constants.h`, `atmosphere/demo/demo.cc` (spectral tables).
- No white balance: CIE 1931 2-degree XYZ -> linear sRGB (D65).
- Atmosphere top 100 km; the model's ground radius below the lowest relevant surface (documented choice, see Task 2).
- One exposure: scene value = luminance (cd/m^2) x `day_exposure` x pi / E_v,TOA (E_v,TOA: the model's TOA solar
  illuminance at 1 AU); `day_exposure` stays 1.5.
- Performance gate: `res://tests/terrain_perf.gd` 1440p desktop proxy p99 <= 8 ms, max <= 16 ms, atmosphere on.
- Floating origin: camera vectors are differenced in float64 and passed hi/lo; no r^2 - R^2 of large float32 values.
- Tests: core cases tagged `[atmosphere]` (`[slow]` if > ~40 s); run only affected tags
  (`tools/build.ps1 -NoBindings -TestFilter "[atmosphere]"`, with bindings once the binding changes).
- UI strings German, code/comments/commits English, Conventional Commits; commit + `git push origin main` per task.

## Review Focus

1. Camera on or below the model's ground (pad at -21 m ellipsoidal, sea at the geoid down to about -106 m, the drone
   ship's deck): no NaN, no black sky, no jump - clamped onto the ground radius (tests in Task 3 and Task 5).
2. Camera far outside the atmosphere with the scaled-space Earth (Moon view, 3.8e8 m): the limb is where the Earth
   mesh's limb is, no float overflow (GPU check in Task 5, visual in Task 10).
3. Sun at and below the horizon (mu_s below Bruneton's mu_s_min, the night side): finite, non-negative, continuous
   through twilight (test in Task 3: a sweep of the Sun elevation from +10 to -18 degrees is monotone and finite).
4. View rays grazing the horizon (Bruneton's ground / non-ground seam at mu = mu_horizon): no visible line on the sea
   horizon or the limb (test in Task 3: radiance continuous across mu_horizon within 2 %; visual in Task 10).
5. Missing, stale or damaged cache, fresh checkout without `data/atmosphere/`: tables are recomputed and the game
   starts; a damaged file is never read (test in Task 3; GDScript path in Task 5).

---

### Task 1: Optical inputs of the atmosphere

**Files:**
- Create: `config/atmosphere/earth_optics.toml`
- Create: `core/include/starsim/config/atmosphere_optics_config.hpp`, `core/src/config/atmosphere_optics_config.cpp`
- Create: `core/include/starsim/environment/atmosphere_optics.hpp`, `core/src/environment/atmosphere_optics.cpp`
- Create: `core/include/starsim/environment/atmosphere_spectra.hpp` (tabulated data with citations)
- Modify: `core/CMakeLists.txt`, `core/tests/CMakeLists.txt`, `config/README.md` (new file listed)
- Test: `core/tests/test_atmosphere_optics.cpp`

**Interfaces:**
- Produces:
  ```cpp
  namespace starsim::config {
  struct AtmosphereOpticsConfig {
      double top_altitude_m;                   // 100e3 [call]
      double ground_altitude_m;                // model ground below the ellipsoid, e.g. -500 [call, Task 2 documents]
      double aerosol_optical_depth_550nm;      // 0.1 [E]
      double aerosol_angstrom_exponent;        // [E]
      double aerosol_single_scattering_albedo; // 0.9 [E]
      double aerosol_scale_height_m;           // 1200 [E]
      double mie_phase_g;                      // 0.8 [E]
      double ozone_column_du;                  // ~300 [S/E]
      double ground_albedo;                    // 0.1 [E]
      int scattering_orders;                   // 4
  };
  AtmosphereOpticsConfig load_atmosphere_optics(const std::filesystem::path& toml_path);  // throws on missing keys
  }
  namespace starsim::environment {
  double air_refractivity(double wavelength_m);                    // n - 1, Peck & Reeder 1972, standard air
  double rayleigh_cross_section_m2(double wavelength_m);           // Bucholtz 1995 (King factor)
  double air_number_density_per_m3(double altitude_m);             // p / (k T) of Us76Atmosphere, 0 above 1000 km
  double ozone_number_density_per_m3(double altitude_m, double column_du);  // US76 ozone profile scaled to column
  double ozone_cross_section_m2(double wavelength_m);              // Serdyuchenko 2014 via Bruneton, 10 nm, linear
  double aerosol_extinction_per_m(double wavelength_m, double altitude_m, const config::AtmosphereOpticsConfig&);
  double solar_irradiance_w_m2_per_m(double wavelength_m);         // ASTM E-490 AM0, 1 AU, per metre of wavelength
  math::Vec3 cie_1931_cmf(double wavelength_m);                    // xbar, ybar, zbar
  }
  ```
  (Use the project's existing vector type; check `core/include/starsim/math/` for its name.)

- [ ] **Step 1: Write the failing tests** in `test_atmosphere_optics.cpp`, tag `[atmosphere]`:
  - `"Rayleigh scattering matches Bucholtz 1995"`: `rayleigh_cross_section_m2(550e-9) * 2.54743e25` within 1 % of
    1.15e-5 /m [Bucholtz 1995 Table 3]; same for 400 nm and 700 nm with the table's values (cite the row in the test).
  - `"Vertical Rayleigh optical depth matches Bucholtz 1995"`: integral of sigma(550 nm) x `air_number_density_per_m3`
    from 0 to 100 km within 2 % of 0.0973 (Bucholtz, US standard atmosphere).
  - `"Ozone profile integrates to its column"`: integral of `ozone_number_density_per_m3(h, 300)` = 300 DU
    (2.6867e20 m^-2 per DU) within 0.5 %.
  - `"Aerosol optical depth at 550 nm is the configured one"`: vertical integral within 0.5 %.
  - `"Solar spectrum integrates to the visible E-490 band"`: integral 400-700 nm within 1 % of the E-490 value
    (state it in the test with the source); `cie_1931_cmf(555e-9).y` within 0.2 % of 1.0.
  - `"Optics config loads and refuses missing keys"`: the committed TOML loads; a copy without `ground_albedo`
    (unique temp file name) throws.
- [ ] **Step 2: Run** `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[atmosphere]"`
  — expected: build fails (headers missing).
- [ ] **Step 3: Implement** the config, the TOML (every value tagged, sources in comments), the spectra header
  (E-490, ozone cross sections and CIE CMFs as transcribed in Bruneton's `demo.cc` / `constants.h`, with his notice
  and the original sources) and `atmosphere_optics.cpp`. The US76 ozone profile is tabulated from the US Standard
  Atmosphere 1976 (cite table) and normalised to the column.
- [ ] **Step 4: Run** the same command — expected: all `[atmosphere]` cases PASS.
- [ ] **Step 5: Commit + push** `feat(environment): spectral optical inputs of the Earth atmosphere`.

### Task 2: Transmittance and single scattering tables

**Files:**
- Create: `core/include/starsim/environment/atmosphere_tables.hpp`, `core/src/environment/atmosphere_tables.cpp`
  (table layout, Bruneton parameterisation, sampling helpers)
- Create: `core/src/environment/atmosphere_precompute.cpp` (+ internal header `atmosphere_precompute.hpp` in src/)
- Test: `core/tests/test_atmosphere_tables.cpp`, `core/tests/atmosphere_reference.hpp` (independent brute-force
  integrator for the tests)

**Interfaces:**
- Consumes: Task 1.
- Produces:
  ```cpp
  namespace starsim::environment {
  struct AtmosphereTableSizes {   // Bruneton's defaults; r count may grow for the 100 km top (document)
      int transmittance_mu = 256, transmittance_r = 64;
      int scattering_r = 32, scattering_mu = 128, scattering_mu_s = 32, scattering_nu = 8;
      int irradiance_mu_s = 64, irradiance_r = 16;
  };
  struct AtmosphereParameters {   // everything the runtime lookups need (CPU and shader)
      double bottom_radius_m, top_radius_m, mu_s_min, mie_phase_g, sun_angular_radius_rad;
      std::array<double, 3> solar_illuminance_rgb;          // TOA at 1 AU, luminance mode (Bruneton's solar_irradiance)
      std::array<double, 3> sky_radiance_to_luminance, sun_radiance_to_luminance;
      double toa_solar_illuminance_lux;                     // E_v,TOA for the exposure
  };
  struct AtmosphereTables {
      AtmosphereTableSizes sizes; AtmosphereParameters parameters; std::uint64_t input_hash;
      std::vector<float> transmittance;   // RGBA, mu fastest, then r (RGB at Bruneton's kLambdaR/G/B)
      std::vector<float> scattering;      // RGBA 3D: (nu, mu_s) packed in x, mu in y, r in z - Bruneton's layout
      std::vector<float> single_mie;      // RGBA 3D, same layout
      std::vector<float> irradiance;      // RGBA 2D
  };
  AtmosphereTables precompute_atmosphere(const config::AtmosphereOpticsConfig&, const AtmosphereTableSizes& = {});
  std::uint64_t atmosphere_input_hash(const config::AtmosphereOpticsConfig&, const AtmosphereTableSizes&);
  }
  ```
  In this task `precompute_atmosphere` fills transmittance and single scattering only (orders = 1); Task 3 adds the rest.

- [ ] **Step 1: Write the failing tests** (`[atmosphere]`, use reduced sizes where a test only checks the method,
  e.g. half resolution, to stay < 40 s):
  - `"Transmittance table matches a direct optical-depth integral"`: zenith, 60 deg, 85 deg and a ray grazing the
    horizon from 0, 10, 50 km; interpolated table value vs `atmosphere_reference.hpp` integral (1e5 steps) within 0.5 %
    (document the measured max).
  - `"Single scattering matches a brute-force raymarch"`: five (r, mu, mu_s, nu) cells vs the reference integrator
    within 1 %.
  - `"The ground radius lies below every relevant surface"`: `bottom_radius_m` < a - 106 m and the choice is the
    configured `ground_altitude_m`.
  - `"Input hash changes with every input"`: changing any config field or size changes the hash.
- [ ] **Step 2: Run** `tools/build.ps1 -NoBindings -TestFilter "[atmosphere]"` — expected: FAIL (not implemented).
- [ ] **Step 3: Implement** the parameterisation and the transmittance + single scattering passes as in Bruneton's
  `functions.glsl` (ComputeTransmittanceToTopAtmosphereBoundary, ComputeSingleScattering), in double, per wavelength,
  with the density/coefficient from Task 1 sampled on an altitude table (density profile as a table, not Bruneton's
  exponential layers). Parallelise over table rows with `std::thread` (hardware_concurrency). Wavelength handling as
  Bruneton's `Model::Init` with precomputed luminance: 15 wavelengths 360-830 nm in passes of 3, accumulated through
  the luminance_from_radiance matrix; the transmittance table at kLambdaR/G/B (680/550/440 nm).
- [ ] **Step 4: Run** — expected: PASS.
- [ ] **Step 5: Commit + push** `feat(environment): atmosphere transmittance and single scattering tables`.

### Task 3: Multiple scattering, irradiance, CPU evaluation, cache

**Files:**
- Modify: `core/src/environment/atmosphere_precompute.cpp`
- Create: `core/include/starsim/environment/atmosphere_scattering.hpp`, `core/src/environment/atmosphere_scattering.cpp`
  (CPU runtime lookups + ellipsoid mapping)
- Create: `core/include/starsim/environment/atmosphere_table_file.hpp`, `core/src/environment/atmosphere_table_file.cpp`
- Create: `core/apps/atmosphere_tables.cpp` (CLI), CMake target `atmosphere_tables`
- Test: `core/tests/test_atmosphere_scattering.cpp`, `core/tests/test_atmosphere_table_file.cpp`

**Interfaces:**
- Consumes: Task 2.
- Produces:
  ```cpp
  namespace starsim::environment {
  // Earth-fixed positions; the ellipsoid is mapped onto the sphere of radius a by scaling z by a/b (directions too,
  // renormalised). Below the ground radius a point is clamped onto it.
  math::Vec3 ellipsoid_to_atmosphere_space(const math::Vec3& position_itrf_m);
  class AtmosphereScattering {
  public:
      explicit AtmosphereScattering(std::shared_ptr<const AtmosphereTables> tables);
      math::Vec3 sun_transmittance(const math::Vec3& position_itrf_m, const math::Vec3& sun_direction_itrf) const;
      // luminance cd/m^2 (RGB); transmittance to the atmosphere's edge in *transmittance
      math::Vec3 sky_luminance(const math::Vec3& camera_itrf_m, const math::Vec3& view_direction_itrf,
                               const math::Vec3& sun_direction_itrf, math::Vec3* transmittance) const;
      math::Vec3 sky_luminance_to_point(const math::Vec3& camera_itrf_m, const math::Vec3& point_itrf_m,
                                        const math::Vec3& sun_direction_itrf, math::Vec3* transmittance) const;
      struct Illuminance { math::Vec3 sun_lux, sky_lux; };
      Illuminance sun_and_sky_illuminance(const math::Vec3& point_itrf_m, const math::Vec3& normal_itrf,
                                          const math::Vec3& sun_direction_itrf) const;
  };
  void write_atmosphere_tables(const std::filesystem::path& file, const AtmosphereTables&);   // tmp + rename
  std::optional<AtmosphereTables> read_atmosphere_tables(const std::filesystem::path& file,
                                                         std::uint64_t expected_hash);      // nullopt if bad
  // Loads <cache_dir>/earth_atmosphere.bin if its hash matches, else precomputes and writes it.
  std::shared_ptr<const AtmosphereTables> load_or_compute_atmosphere(const std::filesystem::path& config_dir,
                                                                     const std::filesystem::path& cache_dir);
  }
  ```
  The Sun's distance scaling (1/d_AU^2) is applied by callers (the tables are for 1 AU).
  CLI: `atmosphere_tables.exe --config <dir> --out <dir>` prints the time and the file.

- [ ] **Step 1: Write the failing tests** (`[atmosphere]`; full-resolution precompute once per test process via a
  function-local static; tag `[slow]` if the precompute exceeds ~40 s):
  - `"Multiple scattering converges"`: orders 4 vs 6 at ten sample directions differ < 1 %.
  - `"Clear-sky luminances are physical"`: ground camera, Sun at 60 deg elevation: zenith luminance 2000-10000 cd/m^2;
    horizontal sun + sky illuminance with the Sun at zenith 90-120 klx; blue/red ratio of zenith sky > 1.5.
  - `"Direct sunlight matches ASTM G173 at air mass 1.5"`: transmitted E-490 spectrum 400-700 nm at sea level, Sun at
    48.19 deg zenith, vs G173 direct (integrated band, value in the test with its source) within 10 %, the reason
    (aerosol/water-vapour model differences) in a comment.
  - `"Sunset reddens the Sun"`: `sun_transmittance` at 2 deg elevation has R/B > 3; at 0.5 deg below the horizon 0.
  - `"Sky darkens with altitude"`: zenith luminance at 50 km < 1e-2, at 100 km < 1e-4 of the ground value (Sun 30 deg).
  - `"Twilight is finite and monotone"` (Review Focus 3): Sun from +10 to -18 deg in 0.5 deg steps at the ground,
    zenith luminance finite, >= 0, non-increasing; zenith colour bluer at -4 deg than at +60 deg.
  - `"Radiance is continuous across the horizon seam"` (Review Focus 4): mu_horizon +- 1e-4 from 2 m and 400 km,
    relative jump < 2 %.
  - `"Cameras below the ground radius are clamped"` (Review Focus 1): camera at -200 m ellipsoidal at 0 and 60 deg
    latitude gives the same finite sky as at the ground radius.
  - `"The ellipsoid maps onto the ground sphere"`: WGS84 surface points at 0/30/60/90 deg latitude map to radius a
    within 1 mm.
  - `"Table files round-trip and damaged files are recomputed"` (Review Focus 5): write/read equal; a truncated copy
    and a flipped byte give nullopt; `load_or_compute_atmosphere` on an empty unique temp dir writes the file.
- [ ] **Step 2: Run** `tools/build.ps1 -NoBindings -TestFilter "[atmosphere]"` — expected: FAIL.
- [ ] **Step 3: Implement** ComputeScatteringDensity, ComputeIndirectIrradiance, ComputeMultipleScattering and the
  direct irradiance as in Bruneton's `model.cc` / `functions.glsl` (ground albedo from the config), and the runtime
  lookups (GetSkyRadiance, GetSkyRadianceToPoint, GetSunAndSkyIrradiance) in `atmosphere_scattering.cpp`. File
  format: magic, version, hash, sizes, parameters, then the four float arrays; a CRC or the hash over the payload
  detects damage.
- [ ] **Step 4: Run** — expected: PASS; record the precompute time (target < 60 s on 24 cores) in the commit body.
- [ ] **Step 5: Commit + push** `feat(environment): multiple scattering, CPU sky evaluation and table cache`.

### Task 4: GDExtension access, packaging, credits

**Files:**
- Modify: `bindings/src/starsim_core.hpp`, `bindings/src/starsim_core.cpp`
- Modify: `tools/package.ps1` (run `build/core/atmosphere_tables.exe --config config --out <package>/data/atmosphere`
  and, for local runs, `data/atmosphere`), `tools/data_pipeline/sources.toml` (Bruneton code BSD-3, E-490, Serdyuchenko
  ozone, CIE CMF, Bucholtz/Peck-Reeder as formula sources), `CREDITS.md` (regenerated)
- Test: `game/tests/smoke_test.gd` (extend)

**Interfaces:**
- Consumes: Task 3.
- Produces (StarsimCore; Vector3 is double):
  - `load_atmosphere(config_dir: String, cache_dir: String) -> Dictionary` — `{ok, error, transmittance:
    PackedFloat32Array, transmittance_size: Vector2i, scattering, single_mie: PackedFloat32Array, scattering_size:
    Vector3i, irradiance, irradiance_size: Vector2i, parameters: Dictionary}` (parameter names as
    `AtmosphereParameters` fields). Safe to call from a worker thread; keeps the `AtmosphereScattering` for the calls
    below.
  - `atmosphere_sun_transmittance(position_itrf_m: Vector3, sun_direction_itrf: Vector3) -> Vector3`
  - `atmosphere_sky_luminance(camera_itrf_m: Vector3, view_direction_itrf: Vector3, sun_direction_itrf: Vector3)
    -> Dictionary {luminance: Vector3, transmittance: Vector3}`
  - `atmosphere_sky_luminance_to_point(camera_itrf_m, point_itrf_m, sun_direction_itrf) -> Dictionary` (same keys)
  - `atmosphere_illuminance(point_itrf_m, normal_itrf, sun_direction_itrf) -> Dictionary {sun_lux: Vector3,
    sky_lux: Vector3}`
  Errors (German) via `push_error` and `ok = false`; calls before a successful load return zero vectors.

- [ ] **Step 1: Extend smoke_test.gd**: `load_atmosphere(Paths.config_dir(), <unique temp dir>)` returns ok with
  `scattering.size() == 4 * x * y * z`, and `atmosphere_sky_luminance` at the pad looking up with the Sun at 60 deg is
  2000-10000 on the green channel.
- [ ] **Step 2: Run** `tools/build.ps1 -TestFilter "[atmosphere]"` then the smoke test
  (`external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script
  res://tests/smoke_test.gd`) — expected: smoke test FAILS on the missing method.
- [ ] **Step 3: Implement** the methods, the package step and the credit entries; regenerate CREDITS.md with the
  pipeline's generator.
- [ ] **Step 4: Run** both again — expected: PASS.
- [ ] **Step 5: Commit + push** `feat(bindings): atmosphere tables and evaluations for the game`.

### Task 5: Shader include, sky and per-frame uniforms

**Files:**
- Create: `game/shaders/atmosphere.gdshaderinc` (Bruneton runtime port: lookups, GetSkyRadiance,
  GetSkyRadianceToPoint, GetSunAndSkyIrradiance, the ellipsoid stretch, the scene-unit scale)
- Create: `game/shaders/sky.gdshader` (`shader_type sky`)
- Create: `game/world/atmosphere.gd` (`class_name Atmosphere`: loads the tables on a worker thread, builds
  ImageTexture (FORMAT_RGBAF) / ImageTexture3D, sets the global uniforms, per-frame `update(...)`)
- Modify: `game/project.godot` `[shader_globals]` (new globals below), `game/world/world.gd` (`_setup_environment`:
  BG_SKY with the sky shader, ambient + reflections from the sky; remove the constant ambient colour; create and
  update `Atmosphere`), German status text while tables compute
- Create: `game/tests/atmosphere_check.gd` (GPU: renders sample views, reads pixels, compares with the CPU)

**Interfaces:**
- Consumes: Task 4.
- Produces (global shader uniforms, all in the Earth-local render frame = ITRF in render axes, polar axis +Y):
  `atmosphere_transmittance` (sampler2D), `atmosphere_scattering`, `atmosphere_single_mie` (sampler3D),
  `atmosphere_irradiance` (sampler2D), `atmosphere_camera_hi`, `atmosphere_camera_lo` (vec3, camera position split
  from float64), `atmosphere_camera_altitude_m` (float, float64-derived height above the model ground in the
  stretched space), `atmosphere_world_to_earth` (mat3: render world axes -> Earth-local axes), `atmosphere_sun_direction`
  (vec3, Earth-local, apparent), `atmosphere_luminance_scale` (float: day_exposure x pi / E_v,TOA x 1/d_AU^2),
  `atmosphere_enabled` (bool). Include functions:
  `vec3 atm_sky(vec3 camera_local, vec3 view_dir_local, out vec3 transmittance)`,
  `vec3 atm_to_point(vec3 camera_local, vec3 point_local, out vec3 transmittance)`,
  `vec3 atm_sun_and_sky(vec3 point_local, vec3 normal_local, out vec3 sky)` (returns transmitted sun, scene units),
  `vec3 atm_camera_local()` (hi + lo). The near-ground rho uses `atmosphere_camera_altitude_m` for the camera
  (Global Constraints).
  `Atmosphere.update(camera_gcrf_m: Vector3, earth_center_gcrf_m: Vector3, earth_fixed_to_gcrf: Basis,
  sun_apparent_gcrf_m: Vector3)` once per frame from `world.gd`.

- [ ] **Step 1: Write `atmosphere_check.gd`** (GPU, run without --headless): for (pad looking up, Sun 60 deg), (pad at
  sunset looking at the Sun), (400 km looking at the limb), (Moon distance looking at the Earth's limb, Review Focus 2),
  (pad camera at -21 m ellipsoidal, Review Focus 1): render a frame with only the sky (vehicles hidden), read the
  viewport pixel at the sample direction, convert back through the exposure / tonemap off (the check renders with
  `TONE_MAPPER_LINEAR` and glow off), compare with `atmosphere_sky_luminance x atmosphere_luminance_scale` within 2 %;
  print per sample, exit code 1 on failure.
- [ ] **Step 2: Run** `godot --path game --script res://tests/atmosphere_check.gd` — expected: FAIL (black sky).
- [ ] **Step 3: Implement** the include, the sky shader (in `AT_CUBEMAP_PASS` it also draws the Earth's ground as
  the WGS84 surface with the day texture lit by `atm_sun_and_sky`, seen through `atm_to_point`; in the background pass
  ground rays return the same), `atmosphere.gd` and the world wiring (tables load on a WorkerThreadPool task; until
  ready `atmosphere_enabled = false` and the old black sky).
- [ ] **Step 4: Run** the check — expected: PASS; also `smoke_test.gd` headless still PASS and one render_check PNG
  from the pad (`--lat=25.9967 --lon=-97.1543 --azimuth=90 --elevation=10`) read to confirm a blue sky.
- [ ] **Step 5: Commit + push** `feat(game): physically computed sky from the atmosphere tables`.

### Task 6: Earth surfaces: ground light, aerial perspective, sea sky

**Files:**
- Modify: `game/shaders/earth.gdshader`, `game/shaders/terrain_earth.gdshader`, `game/shaders/water.gdshaderinc`
- Modify: `game/world/world.gd` (earth material: no extra uniforms if globals suffice)
- Modify: `game/tests/atmosphere_check.gd` (surface samples)

**Interfaces:**
- Consumes: Task 5 include + globals.
- Produces: no new names; `water.gdshaderinc` loses `sky_radiance` and takes the sky through `atm_sky` from the water
  point in the reflected direction (times the sunlit fraction).

- [ ] **Step 1: Extend the check**: a terrain pixel 20 km from a camera at 2 m (the Cape) and an Earth-disk pixel from
  400 km and from the Moon distance (the scaled-space ellipsoid): pixel vs CPU
  (`albedo x illuminance -> transmittance + atmosphere_sky_luminance_to_point`) within 3 % (albedo read from the same
  texel by the check, documented).
- [ ] **Step 2: Run** — expected: FAIL.
- [ ] **Step 3: Implement**: `day = albedo x (sun_T x max(cos,0) x sunlit + sky_irradiance) x scale` replaces
  `albedo x cos x sunlit x day_exposure`; the sea uses the transmitted sun for its diffuse and glint terms; then
  `color = color x T + inscatter` for day, sea and city lights (night lights keep their own emission and twilight fade).
  In scaled space `earth.gdshader` uses the Earth-local camera from the globals (true metres), so the hand-over to the
  terrain quadtree shows the same atmosphere.
- [ ] **Step 4: Run** the check — expected: PASS; render_check PNGs: Cape coast from 5 km
  (`--sea_lat=28.4 --sea_lon=-80.4 --eye_height=5000 --elevation=-5`), Earth from 400 km and from the Moon
  (`--focus=earth --view_from=moon`), read for haze, limb and sea horizon (no seam line).
- [ ] **Step 5: Commit + push** `feat(game): Earth surfaces lit and seen through the atmosphere`.

### Task 7: Moon, Sun disk and stars through the atmosphere

**Files:**
- Modify: `game/shaders/moon.gdshader`, `game/shaders/terrain_moon.gdshader`, `game/shaders/sun.gdshader`,
  `game/shaders/stars.gdshader`, `game/world/star_field.gd` (pass the stars' illuminance instead of a relative
  brightness)
- Create: `game/shaders/star_visibility.gdshaderinc` (threshold over the background luminance)
- Modify: `game/tests/atmosphere_check.gd`

**Interfaces:**
- Consumes: Task 5.
- Produces: `float star_visibility(float star_illuminance_lux, float background_luminance_cd_m2)` (0..1, smooth),
  from a Schaefer 1990 / Garstang-type threshold I_th = c1 (1 + sqrt(c2 B))^2 with the constants and citation in the
  include. Stars: illuminance E = 2.54e-6 x 10^(-0.4 m) lux [S: zero-point of the V magnitude]; the rendered
  brightness keeps today's look above the atmosphere (relative scale `REFERENCE_MAGNITUDE` unchanged), multiplied by
  transmittance x `star_visibility`.

- [ ] **Step 1: Extend the check**: from the pad at noon a magnitude-0 star pixel equals the sky (invisible); at 120 km
  altitude at noon it equals today's brightness within 5 %; at the pad with the Sun 18 deg below the horizon a
  magnitude-1 star is visible (> 2x sky); the Sun disk's colour at 1 deg elevation has R/B > 3; the Moon from the pad
  at noon is the Moon x T + sky within 3 %.
- [ ] **Step 2: Run** — expected: FAIL.
- [ ] **Step 3: Implement**: opaque Moon / Sun: `color x T + sky`; stars (additive): `color x T x visibility` with the
  background luminance from `atm_sky` along the star's direction.
- [ ] **Step 4: Run** — expected: PASS; render_check PNG of the daytime Moon from the pad and a twilight sky read.
- [ ] **Step 5: Commit + push** `feat(game): Moon, Sun and stars seen through the atmosphere`.

### Task 8: Vehicles and structures: haze, sunlight, exposure

**Files:**
- Create: `game/shaders/lit_aerial.gdshader` (shared spatial shader: albedo / roughness / metallic / emission
  uniforms covering what the current StandardMaterial3Ds use, plus `FOG` = aerial perspective from `atm_to_point`)
- Modify: `game/vehicle/vehicle_view.gd`, `game/world/tower_view.gd`, `game/world/drone_ship_view.gd`,
  `game/world/world.gd` (pads, sun light), plume / trail materials (`flight_trail_view.gd`, plume shader or material in
  `vehicle_view.gd`: transmittance only)
- Modify: `game/tests/atmosphere_check.gd`

**Interfaces:**
- Consumes: Task 4 (`atmosphere_sun_transmittance`), Task 5.
- Produces: `LitAerial.material(albedo: Color, roughness: float, metallic: float, emission := Color.BLACK) ->
  ShaderMaterial` (static helper in `game/world/lit_aerial.gd`) used instead of `StandardMaterial3D.new()` in those
  files. Sun light: `sun_light.light_color` = normalised transmitted sun at the followed body, `light_energy` such that a
  white Lambertian under the TOA Sun gives `day_exposure` (Global Constraints) x |T| x eclipse fraction.

- [ ] **Step 1: Extend the check**: a white test cube at the pad at noon renders within 5 % of
  `day_exposure x (sun_T x cos + sky ambient)`; the same cube 40 km away seen from the pad matches
  `cube x T + inscatter` (CPU) within 5 %; the sun light colour at 2 deg elevation has R/B > 3.
- [ ] **Step 2: Run** — expected: FAIL.
- [ ] **Step 3: Implement** the shader / helper, convert the materials (look unchanged at 0 distance), drive the sun
  light each frame from `atmosphere_sun_transmittance` (body ITRF position from the snapshot the world already uses).
- [ ] **Step 4: Run** — expected: PASS; render_check `--vehicle=starship_block3 --site=starbase_olp1 --fly=120` from the
  pad read (the rocket fades into the sky); headless `smoke_test.gd` still PASS.
- [ ] **Step 5: Commit + push** `feat(game): vehicles and structures in the atmosphere's light and haze`.

### Task 9: Solar eclipse in the sky

**Files:**
- Modify: `game/shaders/atmosphere.gdshaderinc` (segmented in-scattering), `game/world/atmosphere.gd` (enable flag
  when the penumbra touches the Earth: CPU check with `StarsimCore.solar_eclipse_fraction` or the Moon-Sun geometry),
  `game/shaders/sky.gdshader`, Earth surface shaders (aerial perspective path)
- Modify: `game/tests/atmosphere_check.gd`

**Interfaces:**
- Consumes: Task 5, `eclipse.gdshaderinc`'s `occulted_fraction`.
- Produces: global `atmosphere_eclipse_segments` (int, 0 = off; [call] 8 when active), `atmosphere_moon_position`
  (vec3, Earth-local, apparent). Inside `atm_sky` / `atm_to_point`: in-scatter = sum over segments of
  sunlit(midpoint) x [S(p_i) - T(p_i, p_i+1) S(p_i+1)].

- [ ] **Step 1: Extend the check** for 2024-04-08 18:42 UTC: from a ground point in totality (Dallas, lat 32.78 lon
  -96.80) the zenith sky is < 1 % of the same sky with segments off, the horizon away from the umbra > 10x the zenith;
  outside the eclipse (one hour later) segments on and off agree within 1 %.
- [ ] **Step 2: Run** — expected: FAIL.
- [ ] **Step 3: Implement** the segmented path and the flag; document the approximation in the include.
- [ ] **Step 4: Run** — expected: PASS; render_check of the totality from the ground read.
- [ ] **Step 5: Commit + push** `feat(game): the Moon's shadow darkens the sky`.

### Task 10: Performance gate, visual set, docs

**Files:**
- Modify: `docs/subsystems.md` (new section "Atmosphere rendering": model, inputs, mapping, exposure, eclipse
  approximation, validation numbers, known limits), `CLAUDE.md` (implemented-modules list: atmosphere optics /
  scattering; known issues: replace "Sky is black ... horizon glow until the atmosphere part"; Game structure: the
  atmosphere files), `docs/ansichten.md` (sky / haze in the views), `game/tests/terrain_perf.gd` only if it must
  enable the atmosphere explicitly
- Memory: `project-milestone-order.md`, new `project-m8-atmosphere.md`

- [ ] **Step 1: Run the perf gate** `godot --path game --script res://tests/terrain_perf.gd` — expected: p99 <= 8 ms,
  max <= 16 ms. If not: sky cubemap update mode (incremental), sampler precision, fewer eclipse segments - measure,
  never drop physics silently.
- [ ] **Step 2: Visual set** (render_check PNGs, each read): OLP-1 noon looking at / away from the Sun, OLP-1 sunset,
  ascent views at 10 / 30 / 60 / 100 km (`--fly=` with `--autopilot`), ISS orbit at the day limb and the terminator,
  Earth from the Moon, totality from Dallas, the Cape coast with the drone ship from 5 km, the rocket from the pad at
  40 km. Note anything wrong as a finding and fix in the owning task's files.
- [ ] **Step 3: Write the docs** listed above.
- [ ] **Step 4: Run** `tools/build.ps1 -TestFilter "[atmosphere]"`, `smoke_test.gd`, `atmosphere_check.gd` — expected:
  all PASS.
- [ ] **Step 5: Commit + push** `docs: atmosphere rendering`; then the final whole-plan review, its fixes, and the
  user's VR check (Start-VR.cmd: pad, ascent, orbit) named in the plan-end message.
