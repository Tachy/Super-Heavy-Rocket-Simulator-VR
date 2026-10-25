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
python tools/data_pipeline/fetch.py
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
powershell -ExecutionPolicy Bypass -File tools/build.ps1          # core + GDExtension + full ctest (parallel, ~3 min)
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast                  # without [slow] campaigns (~1 min)
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[cr3bp]"  # only matching tags
# Godot smoke test (extension loads, float64 end to end)
external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd
```
- Release build (sibling pattern): `tools/package.ps1 [-Version x.y.z] [-SkipTests] [-NoInstaller]` builds the
  double-precision release template once (`python -m SCons platform=windows target=template_release precision=double
  production=yes` in external/godot, ~5 min), core + extension, smoke test, exports `game/export_presets.cfg`, lays out
  `build/package` (exe next to `config/` + base `data/`; `Paths` uses the executable's folder in exported builds),
  self-checks the exported exe (`-- --self-check`, with and without the data parts) and writes the ZIPs (game, data
  parts terrain / imagery, each < 2 GB) and the Inno Setup installer (`tools/installer.iss`, all data inside: the repo
  is private). `/build` publishes the release-please version, `/build-local` a local test installer. Version:
  `game/util/game_version.gd` (release-please `extra-files`; the action has `skip-github-release`).
- Test workflow (user 2026-10-19, replaces 2026-10-15): always run only what the change affects - the tags of the
  touched modules and of the subsystems whose code paths changed (`-TestFilter`, `[slow]` cases of those tags
  included), also before a push and before a plan's final review; no blanket `-Fast` or full-suite runs. A change to
  shared code (e.g. `FlightSession::advance_to`, `Vehicle::mass_properties`) whose new branch is inactive for other
  subsystems does not pull their campaigns in. Tag a new test case `[slow]` when it takes more than ~40 s.
- `-TestFilter "[a],[b]"` runs the union of the tags as parallel ctest processes (`[slow]` included unless `-Fast`;
  output in `build/ctest_filter.log`); a spec that is not a tag list runs the exe serially.
- Long test runs (user 2026-10-20): a run that may take longer than 10 minutes goes to the background with its output
  in a file; check its state every 10 minutes (failures so far, a case hanging or far beyond its usual time). When
  tests are going wrong, stop the run early and fix the cause instead of waiting for the end.
- ctest runs test cases as parallel processes: tests must not share temp files or directories (make names unique)
  (exception: the write-once plan cache `build/plan_cache`, Task 5 of the 2026-10-23 follow-ups: content-addressed,
  files renamed into place, wiped on every test relink, active only through `STARSIM_PLAN_CACHE_DIR` set by the test
  CMake; a test that asserts trial flights calls `sim::disable_plan_cache()`).
- Visual check (needs GPU, no --headless): `godot --path game --script res://tests/render_check.gd --
  --utc=2024-04-08T18:42:00 --focus=earth --view_from=moon --out=<png>`; ground views with
  `--lat= --lon= --look_at_moon --fov=`. Read the PNG to verify rendering changes.
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

