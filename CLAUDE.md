# Super Heavy / Starship Simulator

Physically accurate simulator for Super Heavy + Starship: a playable PC-VR flight simulator
(OpenXR, HOTAS, seated cockpit) and a mission/engineering simulation (planning, telemetry, validation).
Not an arcade game: physical correctness has top priority.

## Where to find what (read only what the task needs)
| Topic | File |
|---|---|
| Implemented core modules, game structure (world, terrain, atmosphere, flight, UI, XR, HOTAS) | `docs/architecture.md` |
| One-time setup, every build command, release packaging, test workflow details, visual checks | `docs/setup.md` |
| Known issues (solver limits, open stalls, data caveats) | `docs/known-issues.md` |
| A subsystem's current state: model, code, standing decisions, validation, diagnostics, open points | `docs/subsystems/<slug>.md` (index: `docs/subsystems.md`) |
| Why something is as it is, superseded numbers, earlier approaches | `docs/history/<slug>.md` |
| Views and controls of the finished game (German, living document) | `docs/ansichten.md` |
| Vehicle data provenance ([S]/[E]/[D]) | `config/README.md` |
| Active specs / plans; finished ones | `docs/superpowers/specs`, `plans`; `docs/superpowers/archive/` |

Read the subsystem file before changing a subsystem and update it with the change (current state only; move what
becomes superseded into its history file).

## Communication & language
- Answer the user **in German**. Code, identifiers, comments, commit messages and docs in the repo: **English**.
  UI text is German.
- Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`, ...): release-please builds the release PR and
  CHANGELOG.md. Releases are made by the user (merging that PR).

## Working on plans
- No feature branches (user 2026-10-15): commit directly on `main` and `git push origin main` after every completed
  task (plan task, fix pass, docs update) once its tests are green.
- Run a plan through all its tasks; ask for `/clear` only at its end (user 2026-10-15 / 10-19). Before asking, record
  the state (done, open, decisions, next step) in the plan's ledger and the memory, commit and push; after "weiter"
  resume from ledger, memory and `git log`.
- Token economy (user 2026-10-25, measured: subagents ~80 % of use): subagents on Sonnet by default, Opus only for
  judgment-heavy / physics-critical work and final reviews; fix rounds by a fresh small fixer that reads brief, report
  and findings (not a resumed implementer); briefs and rules as files, short dispatches. Quality gates stay.
- Keep the context lean: long command output to files (read the tail or grep), files in excerpts, broad searches by a
  subagent. The user still gets all results, numbers, decisions and open points in full.

## Architecture (non-negotiable)
```
core/        C++20 simulation core. NO Godot dependency. Pure float64.
bindings/    GDExtension adapter (godot-cpp, precision=double). Thin: no physics/business logic.
game/        Godot 4 project (double-precision build). Rendering, UI, VR, input. Text formats only (.tscn/.tres/.gd).
config/      Data-driven definitions: vehicles, launch sites, missions (TOML).
tools/       Data pipeline (Python): downloads, license log, tile generation, CREDITS.
data/        Downloaded runtime data (git-ignored), fetched via tools/data_pipeline/fetch.py.
external/    Third-party sources & engine build (git-ignored).
tests/       Validation fixtures (reference values from JPL Horizons etc.).
```
- `core` is split into small modules with clear interfaces (`math`, `time`, `frames`, `ephemeris`, `spice`,
  `environment`, `integrators`, `optim`, `orbit`, `vehicle`, `propulsion`, `aero`, `contact`, `gnc`, `mission`, `sim`,
  `weather_decode`; what each holds: `docs/architecture.md`). Modules depend downwards only.
- External libraries are wrapped behind interfaces inside one module; nothing else includes them. CSPICE lives only in
  `core/src/spice/` (implements `time::TimeScaleConverter`, `ephemeris::Ephemeris`, `frames::BodyOrientation`); lz4 and
  the GRIB2 decoding only in `core/src/weather_decode/`. No networking in `core`.
- The game only reads simulation state and sends commands. Physics never runs in GDScript.
- The simulation runs on its own thread with a fixed tick; rendering interpolates.
- Vehicles, launch sites and missions are configuration, not code.

## Physics & numerics conventions
- SI units internally, always. Angles in radians.
- Frame and unit go into the name: `position_gcrf_m`, `velocity_itrf_mps`, `epoch_tdb_s`, `latitude_rad`.
- Inertial frame: GCRF (Earth-centred, ICRF-aligned; SPICE calls it `J2000`). Earth-fixed: ITRF93 via NAIF binary PCK.
- Time: `Epoch` stores TDB seconds past J2000. UTC only at the user interface boundary (leap seconds via SPICE LSK).
- Earth shape: WGS84 ellipsoid. Moon: sphere R = 1737.4 km (+ LOLA terrain).
- Default gravity: point masses Earth + Moon + Sun plus Earth J2 (switchable). Earth/Moon/Sun positions
  come from DE440, only spacecraft are integrated. `FlightSession` samples them (and the Earth/Moon orientation) on
  a grid and interpolates (`InterpolatedEphemeris` 20 min, `InterpolatedOrientation` 10 min: sub-mm; fidelity flag
  `interpolated_ephemeris`) - CSPICE per force evaluation was 75 % of the planners' time.
- Simplifications are allowed only when deliberate, documented and switchable (fidelity flags).
- Rendering: floating origin at the player's head; positions are differenced in double and only then
  converted to float32. Sun and Moon are rendered at APPARENT positions (light time + aberration).

## Quality rules
- Every physical model gets unit tests and at least one reference/validation case with a documented tolerance.
- Small files with one responsibility; prefer composition over inheritance.
- No magic numbers: physical constants live in `core/include/starsim/math/constants.hpp` with their source.
- CSPICE is not thread-safe: the `spice` adapters serialise every call with a process-wide lock (`detail::lock_spice`).
- C++ sources are UTF-8 (/utf-8); UI strings in the core are German.
- Physics first (decision 2026-10-14): readings the physics cannot explain (e.g. webcast gauges) are not followed.

## Build & test (Windows, MSVC 2022 Build Tools, CMake, Ninja, SCons) - details in `docs/setup.md`
```powershell
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -TestFilter "[tag1],[tag2]"  # core + affected tags
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[tag]"                       # + GDExtension
external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd
```
- Weather data (opt-in, needs the built core): `python tools/data_pipeline/fetch.py weather_analog` writes the shipped
  typical-weather files `data/weather/analog`; live ERA5 / GFS snapshots are downloaded at run time into
  `data/weather/cache` (`docs/subsystems/weather-service.md`). Weather and cloud limits: `docs/subsystems/weather-clouds.md`.
- Test workflow (user 2026-10-19): run only what the change affects - the tags of the touched modules and of the
  subsystems whose code paths changed (`[slow]` cases of those tags included), also before a push and before a plan's
  final review; no blanket `-Fast` or full-suite runs. Tag a new case `[slow]` when it takes more than ~40 s.
- Long runs (user 2026-10-20): > 10 min to the background with output in a file, check every 10 min; stop early and
  fix the cause when tests go wrong.
- ctest runs cases as parallel processes: tests must not share temp files or directories (unique names).
- Visual checks need a GPU (no `--headless`): `res://tests/render_check.gd` (options in `docs/setup.md`); read the PNG.
  During visual work send progress PNGs to the user.
- After every vehicle config change: `python tools/vehicle_report.py <vehicle-id> [--payload-kg N]`.

## Data licensing
Only free data. Every source is recorded with its licence in `tools/data_pipeline/sources.toml`;
`CREDITS.md` is generated from it (third-party licence texts included). Check the licence when adding a source.
Trademarks ("Starship", "SpaceX") are used internally only; display names come from config.
