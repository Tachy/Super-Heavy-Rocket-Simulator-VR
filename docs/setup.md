# Setup, build, test and release

Moved verbatim from `CLAUDE.md` on 2026-10-25. The everyday commands and the test rules are repeated in short in `CLAUDE.md`.

## Build & test (Windows, MSVC 2022 Build Tools, CMake, Ninja, SCons)
One-time setup:
```powershell
# Runtime data (SPICE kernels, textures, star catalogue) and CSPICE sources; the geoid and Earth terrain sources need
# pip install -r tools/data_pipeline/requirements.txt (numpy, rasterio, pyproj); Earth terrain: several GB of raw downloads
# The imagery step inside fetch.py (NAIP + own Sentinel-2 composite -> data/imagery/earth tile pyramids) also needs
# pystac-client (in requirements.txt), ~2.5 GB raw under data/raw/imagery and network time (S2 composite ~35 min per
# area); `fetch.py --retile imagery` redoes fit + tiles offline from those caches; `--workers N` sets the process count
# The timezones step (timezone-boundary-builder with oceans, ODbL, + tzdata package) writes data/timezones/zone_grid.bin
# (0.02 deg run-length grid, ~1.6 MB) and zones.json (transitions 1990-2060): the camera's local time in the overlay
# Lunar terrain (data parts): `fetch.py lola_ldem_64` downloads the global LOLA grid LDEM_64 (64 px/deg, 473 m, 531 MB, whole,
# ~7 min) to data/terrain/ldem_64.{toml,i16}; `fetch.py lunar_sites` (needs build/core/lunar_site_tile.exe) cuts the height and
# albedo tile of every preset site of config/lunar_sites.toml through that CLI (HTTP ranges, 16 parallel connections: PDS answers
# 403 above ~300 requests at 32; HTTP 429 stops with its Retry-After) into data/terrain/sites/<cache key>.{toml,i16}: Mons Mouton
# 2 MB + 5 MB, each Apollo site ~26 MB + ~5 MB (the albedo download is ~165 MB transient per equatorial tile, only ~5 MB kept).
# `fetch.py lunar_test_sites` (opt-in, like weather_analog) also caches Tsiolkovskiy (20.4 S 129.1 E) and 75.0 N 30.0 E (the
# far-side and high-north site campaign cases and window tests SKIP without them; the Apollo cases need `lunar_sites`).
# The weather_analog step is opt-in (a bare fetch.py skips it): after building the core run
# `python tools/data_pipeline/fetch.py weather_analog` (needs build/core/weather_snapshot.exe); it downloads 48 ERA5 hours (the 15th of each
# month at 00/06/12/18 UTC of 2025, ~65 MB raw each, deleted after use) and writes data/weather/analog (48 x 8.6 MB, ~400 MB)
python tools/data_pipeline/fetch.py        # a bare fetch also bakes data/textures/moon_detail (procedural Moon regolith / rock textures, 4.4 MB,
                                           # ~10 s, no download; `fetch.py --force moon_detail` rebakes; tests: python -m pytest tools/moon_textures).
                                           # tools/package.ps1 ships them with the base data (data/textures is mirrored whole)
python tools/data_pipeline/prepare.py      # converts data/raw -> data/textures, data/stars (needs Pillow)

# Engine sources (git-ignored, pinned)
git clone --depth 1 --branch 4.7.2-stable https://github.com/godotengine/godot.git external/godot   # ed1daf0
git clone https://github.com/godotengine/godot-cpp.git external/godot-cpp                           # b051e69
# Engine patches (tools/godot_patches/): openxr-double-projection.patch builds the OpenXR eye projections in double
# (Godot's float helper loses the far plane at near 1 m / far 3e7 m: broken stereo culling, vehicles vanish in VR)
cd external/godot; git apply ../../tools/godot_patches/openxr-double-projection.patch

# Godot double-precision editor (~15 min on 24 cores; needs misc/scripts/install_accesskit.py
# and install_d3d12_sdk_windows.py run once)
cd external/godot; scons platform=windows target=editor precision=double production=yes

# Extension API of OUR engine build, used by godot-cpp
cd external/godot-cpp; ../godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --dump-extension-api
```
Every build:
```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1          # core + GDExtension + default-tier ctest (parallel, ~10 min cold)
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast                  # without [slow] campaigns (~80 s)
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Nightly               # only the nightly tier (long campaigns, webcast reports)
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[cr3bp]"  # only matching tags
# Godot smoke test (extension loads, float64 end to end)
external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd
```
- Run `smoke_test.gd` alone, not beside a parallel ctest run: its Falcon 9 ASDS planning waits on a 120 s wall-clock `wait_idle` and overruns under CPU load (the 5 ASDS checks then fail).
- Release build (sibling pattern): `tools/package.ps1 [-Version x.y.z] [-SkipTests] [-NoInstaller]` first stops when
  `python tools/structures/build_models.py --check` finds a stale or incomplete set in `data/models` (manifest input
  hash recomputed with the recorded Blender version, every output present; no Blender needed), then builds the
  double-precision release template once (`python -m SCons platform=windows target=template_release precision=double
  production=yes` in external/godot, ~5 min), core + extension, smoke test, exports `game/export_presets.cfg`, lays out
  `build/package` (exe next to `config/` + base `data/`; `Paths` uses the executable's folder in exported builds),
  self-checks the exported exe (`-- --self-check`, with and without the data parts) and writes the ZIPs (game, data
  parts terrain / imagery / LDEM_64 (531 MB: without it the Moon is the smooth sphere, the game logs it) / Diviner rock map (optional, 88 MB: without it the Moon rocks use the global statistical
  abundance), each < 2 GB; the base carries exactly the preset lunar site tiles of `config/lunar_sites.toml` (height + albedo, by their cache keys from the core CLI `plan`; the self-check asserts those files and nothing else in `data/terrain/sites`), never free-site / test-site tiles or `.work` leftovers; `package.ps1 -ListSiteTiles` lists them without a build) and the Inno Setup installer (`tools/installer.iss`, all data inside: the repo
  is private). `/build` publishes the release-please version, `/build-local` a local test installer. Version:
  `game/util/game_version.gd` (release-please `extra-files`; the action has `skip-github-release`).
- Test workflow (user 2026-10-19, tiers 2026-11-03): run only what the change affects - the tags of the touched
  modules and of the subsystems whose code paths changed. Per change `-Fast -TestFilter "<tags>"`; before a push and
  before a plan's final review the same tags with `[slow]` (no `-Fast`); `-Nightly` only at a plan's end or when asked,
  in the background; no blanket full-suite runs. A change to shared code (e.g. `FlightSession::advance_to`,
  `Vehicle::mass_properties`) whose new branch is inactive for other subsystems does not pull their campaigns in. Tag
  a new test case `[slow]` when it takes more than ~40 s cold (the plan cache is wiped after every relink). Webcast
  agreement cases are `[nightly]` reports: read them, they do not gate (the physical gates stay in the default tier).
- `-TestFilter` has Catch semantics: `,` = union of groups, `[a][b]` inside a group = intersection (`"[gnc][validation],[legs]"`).
  Matching cases run as parallel ctest processes (`[slow]` included unless `-Fast`; output in `build/ctest_filter.log`);
  a spec that is not a tag list runs the exe serially. `-ListOnly` prints the ctest command and the matched count
  without building or running (0 matches throws "TestFilter matched no test"). Three tiers: the default and every
  filter exclude the label `nightly`; `-Nightly` alone runs only the nightly cases, with a filter it adds the nightly
  cases of those tags. Check of the filter: `tools/tests/test_build_filter.ps1`.
- Tier sizes and waits (2026-11-03, 24 logical processors, cold plan cache; "wall est." = longest-first packing of the
  measured case times on 24 slots, CPU = sum of case times; before = audit run `test-audit/full_run.log`, one
  uniform pre-tier suite of 1529 cases):

  | Set | Cases (default) | CPU s | Wall est. s | Wall before s |
  |---|---|---|---|---|
  | full default tier | 1518 | 10582 | 384 (run before the split: 619) | 1941 (CPU 20382) |
  | `-Fast` (no `[slow]`) | 1396 | 1938 | 81 | - |
  | nightly only | 30 | not measured | not measured | - |
  | `[falcon9]` / `[asds]` | 52 / 36 | 1178 / 838 | 230 | 201 / 125 |
  | `[ascent_planner]` | 63 | 2815 | 384 | 365 |
  | `[lunar]` / `[return]` | 96 / 29 | 2484 / 1502 | <= 384 (see below) | 1044 |
  | `[refuelling]` | 163 | 1631 | 384 | 1941 |
  | `[sim]` / `[mission]` | 112 / 72 | 1757 / 2391 | 104 / 384 | 369 |
  | `[docking]` / `[ship]` | 96 / 66 | 1438 / 2152 | 384 | 335 |
  | `[atmosphere]` / `[weather]` | 18 / 113 | 94 / 231 | 45 / 75 | 341 / 113 |
  | `[timeline]` / `[catch]` | 29 / 33 | 168 / 726 | 30 / 177 | 96 / 145 |
  | `[dragon]` / `[iss_mission]` | 62 / 40 | 356 / 639 | 74 / 119 | 70 / 182 |
  | `[gnc]\|[validation]\|[ascent]` | 256 | 5663 | <= 384 (see below) | 1044 |

  The default run took 619 s real (target ~500 s missed): its longest cases (aerocapture seeds 3, 11, 19 in one case at
  460 s, `tanker_flight` 384 s) bounded the wall from below. Task 8b split the reduced lunar return cases into one per
  seed (7 cases, 1518 in the default tier); measured case times (ctest, 7 cases in parallel, 2026-11-03): direct seeds
  2 / 12 / 15 / 18 at 45 / 50 / 55 / 44 s, aerocapture seeds 3 / 11 / 19 at 86 / 86 / 84 s. The longest case of the
  default tier is now `tanker_flight` at 384 s; the 384 s wall is derived from the measured case times, not from a new
  full run (the 619 s run was the last one). Rows marked "<= 384" were 460 s before the split; ctest packing and 24 parallel processes stretch it. Some sets
  are slower than the audit figures although cases moved out (`[falcon9]`, `[ascent_planner]`, `[docking]`): this run
  is cold-cache and 24 processes compete for memory bandwidth (cache state of the audit run not recorded).
- Refuelling / lunar sets (2026-11-04, depot/tanker/HLS models plan Task 7, 24 logical processors, GPU work running alongside): `-TestFilter "[refuelling],[docking],[prop_transfer],[campaign],[campaign_flown],[hls],[lunar],[legs]"` with `[slow]` (no `-Fast`, no nightly):
  380 cases, 380 passed, 2404 s wall (40 min; the longest case `tanker_flight: pad -> booster caught -> docked -> transfer -> ship caught` finishes last; Mission 1 LLO 769 s, `hls_refuel_orbit` 1380 s). The `[nightly]` flown refuelling campaign case
  (`[campaign_flown]` against the calculator) is not in it and takes about 52 min wall alone (Task 6c's 28-case nightly run: 2803 s): background, output to a file.
- Moon surface sets (2026-11-06, Moon procedural surface plan Task 13, 24 logical processors, nothing else running):
  `-NoBindings -TestFilter "[moon_surface],[contact],[lunar],[terrain],[terrain_horizon]"` with `[slow]`: 315 cases,
  311 s wall incl. an incremental build (the longest case "Lunar landing campaign: dispersed descents 8-14" 206 s,
  `hls_refuel_orbit` 221 s); `-Fast -TestFilter "[moon_surface],[contact]"` 138 cases, 77 s incl. the build. The hidden
  `[.diag]` case of `test_rock_statistics_exact.cpp` (exact rasters of the templates against the rock tables, run by
  name only) takes ~120 s. GPU checks: `moon_surface_check.gd -- --rocks_only --no_jump --rock_patch_abundance=10`
  ~12 s after the rock view tables are cached (a cold cache adds ~8 s), the full `moon_surface_check.gd` (incl. both
  camera jumps and the summed roughness) about 20 min; a `render_check --stereo --gpu_time=120` view 45-60 s (the
  boulder field longer: its rock lists).
- Long test runs (user 2026-10-20): a run that may take longer than 10 minutes goes to the background with its output
  in a file; check its state every 10 minutes (failures so far, a case hanging or far beyond its usual time). When
  tests are going wrong, stop the run early and fix the cause instead of waiting for the end.
- ctest runs test cases as parallel processes: tests must not share temp files or directories (make names unique)
  (exception: the write-once plan cache `build/plan_cache`, Task 5 of the 2026-10-23 follow-ups: content-addressed,
  files renamed into place, wiped on every test relink, active only through `STARSIM_PLAN_CACHE_DIR` set by the test
  CMake; a test that asserts trial flights calls `sim::disable_plan_cache()`).
- Text tables and report codes (cheap, run after changing `codes.def`, `config/i18n/` or any user-visible string):
  `python tools/i18n_check.py` (codes, tables, phases, literals; or a subset: `codes tables phases`, `literals <paths>`)
  and `python -m pytest tools/test_i18n_check.py -q`; the game side is `game/tests/i18n_test.gd` (part of
  `tools/run_godot_tests.ps1`, or `godot --headless --path game --script res://tests/i18n_test.gd`). Core cases: `-Fast
  -TestFilter "[report]"`. Details: `docs/subsystems/i18n.md`.
- Headless Godot tests (run after changes in `game/`): `powershell -ExecutionPolicy Bypass -File tools/run_godot_tests.ps1`
  runs every `game/tests/*_test.gd` with `--headless`, time and exit code per script in `build/godot_tests.log`, exit 1 if any
  fails. Scripts needing the generated models (`docking_mechanism_test`, `tower_view_test`, `vehicle_view_test`) are skipped
  unless `-Gpu`; `-Only a,b` selects scripts. Needs the built GDExtension. Scripts not named `*_test` are GPU checks or tools. `terrain_quadtree_test` is slow with the terrain data (502 s measured, data checks alone take 132 s Earth, 193 s Moon), so the runner gives it a 900 s limit (`$slowScripts`); it prints elapsed ms per check.
- Visual check (needs GPU, no --headless): `godot --path game --script res://tests/render_check.gd --
  --utc=2024-04-08T18:42:00 --focus=earth --view_from=moon --out=<png>`; ground views with
  `--lat= --lon= --look_at_moon --fov=`; clouds with `--weather=<abs path .wthr>` or `--weather=slab:<bottom>,<top>,<liquid>,<fraction>`
  and `--cloud_quality=desktop|vr`, `--cloud_divisor=<x>` (`docs/subsystems/weather-clouds.md` "Diagnostics"). The
  game's first view: `--start_view` (implies `--weather=analog`, the game's weather being on by default; `--weather=off`
  for the cloudless Earth); `--white_balance=off` drops the eye's adaptation to the Sun's white. Read the PNG to verify rendering changes.
- Sky radiance checks (GPU, `res://tests/sky_radiance_check.gd`, `--stale` in any mode freezes the cube: must exit 1): no
  options (phases against references; `-- --only=<phase>`), `-- --steady`,
  `-- --timelapse`, `-- --flight --utc=2026-10-12T17:30:00 --expect=lit --t_plus=1`, `-- --flight
  --utc=2026-11-09T06:00:00 --expect=dark --t_plus=1`, `-- --flight --live --utc=2026-10-12T17:30:00 --t_plus=1`; headless
  rule test `res://tests/sky_radiance_rule_test.gd` (`docs/subsystems/atmosphere-scattering.md`).
- `STARSIM_VERBOSE=1` makes validation tests print per-epoch errors.
- `core/include/starsim/integrators/dop853_tableau.hpp` is generated by `tools/codegen/gen_dop853_tableau.py`
  from SciPy's coefficient file - never edit it by hand.
- Validation fixtures are regenerated with `python tools/validation/fetch_horizons.py` (committed).
- SOCP reference fixtures (`tests/validation/socp/`) are regenerated with `python tools/validation/gen_socp_fixtures.py`
  (needs cvxpy + clarabel; offline tool only, committed output).
- The GDExtension DLL is written to `game/bin/` (git-ignored).


## Vehicle configuration
`config/` holds propellants, engines, vehicles and aero tables as TOML/CSV with provenance tags
([S] sourced, [E] estimate, [D] derived) - see `config/README.md`.
`build/core/refuelling_report.exe [--height-km N ...]` prints the tanker campaign of a lunar mission (refuelling r6,
levers as flags; measured inputs in `config/refuelling/reference_flights.toml`, checked by the slow r5 flights;
`--fly` flies the whole campaign in one session and compares, ~22 min; `--fly-placed` ~5 min).
`python tools/vehicle_report.py <vehicle-id> [--payload-kg N]` prints masses, thrust, burn time
and the Tsiolkovsky delta-v budget; use it after every config change.

