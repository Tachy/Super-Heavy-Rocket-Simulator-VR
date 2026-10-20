# Super Heavy / Starship Simulator

Physically accurate simulator for Super Heavy + Starship: a playable PC-VR flight simulator
(OpenXR, HOTAS, seated cockpit) and a mission/engineering simulation (planning, telemetry, validation).
Not an arcade game: physical correctness has top priority.

## Communication & language
- Answer the user **in German**.
- Code, identifiers, comments, commit messages and docs inside the repo: **English**.
- Commit messages follow Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`, ...): release-please
  builds the release PR and CHANGELOG.md from them. Releases are made by the user (merging that PR).

## Working on plans
- No feature branches (user 2026-10-15): work and commit directly on `main`, and push (`git push origin main`) after
  every completed task - a plan task, a fix pass, a docs update - once its tests are green.
- Clear the context after every completed plan (user 2026-10-15: token cost grows with the context; auto-compaction
  comes too late; user 2026-10-19: run a plan through all its tasks, ask for `/clear` only at its end, not per task). Claude cannot run `/clear` or `/compact` itself: it records the state (done, open, decisions, next
  step) in the plan's ledger/progress file and the memory, commits and pushes, then asks the user for `/clear` and
  names the next step; after "weiter" it resumes from ledger, memory and `git log`.
- Keep the context lean: long command output to files (read the tail or grep), files in excerpts, broad searches by a
  subagent. The user still gets all information: results, numbers, decisions and open points are reported in full.

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
- `core` is split into small modules with clear interfaces: `math`, `time`, `frames`, `ephemeris`,
  `environment`, `integrators`, `optim`, `orbit`, `vehicle`, `propulsion`, `aero`, `contact`, `gnc`, `mission`, `sim`.
  Implemented so far: math, time, frames, ephemeris, spice (adapter), environment (gravity, reference bodies Earth and
  Moon: body-fixed frame, shape, atmosphere or vacuum; thermal radiation environment: Sun, Earth albedo and infrared),
  integrators (RK4, DOP853 + dense output, terminal events), orbit (Kepler, coast propagation, CR3BP, Clohessy-Wiltshire relative motion,
  CCSDS OEM parser + archive with segment-wise Lagrange interpolation, synthetic station orbit by J2 rates),
  mission (position targeting, 9:2 NRHO reference, lunar outbound and lunar orbit transfers, lunar descent planning,
  the mission delta-v budget, the Earth return: TEI, mid-course corrections, braking burn, the coelliptic phasing
  to the ISS: schedule + full-model planner with drag, and the ISS launch window),
  gnc (among others: PEG steering module, Apollo braking law, lunar landing guidance with SCvx approach and
  terminal descent, lunar ascent and abort guidance, entry corridor, lunar-return entry with an energy-linear bank profile and
  aerocapture guidance, relative navigation between docking ports, proximity guidance: waypoints, corridor,
  hold / retreat / breakout, final approach, R-bar approach from below (the depot), departure after undocking),
  eclipse geometry (disk occultation), optim (sparse SOCP interior-point solver),
  contact (catch tower: geometry, arms + carriage, pin/hull penalty contact, collisions; landing legs and surfaces;
  IDSS docking interface and mechanism: soft capture, joint, retraction, hooks, separation; probe/drogue interface
  (four probes into four drogues) on the same mechanism; hull and landing legs vs station primitives),
  aero (slender body + cross-flow table, grid fins, ship flaps, Sutton-Graves entry heating plus radiative heating
  from the NASA TM-102652 equilibrium tables, blunt capsule from a
  modified-Newtonian table with offset-CG trim, reefed parachutes, fixed-area drag of a station),
  propulsion (engine performance, RCS allocator incl. pure translation capability + pulse modulator),
  vehicle (mass properties, Starship header tanks with a feed source, propellant settling with the apparent gravity,
  ideal LVLH attitude of a station, docking-port kinematics, docked hold, attached mass, compound mass, run-time tank
  masses, probe/drogue port mating of the refuelling ships, propellant boil-off by a hull heat balance in flight and
  orbit-averaged), sim (flight session incl. the ISS as a station body from
  OEM or synthetic orbit, rendezvous target with relative port state and port-frame RCS translation, rendezvous phasing flight to
  Waypoint 0 with drag tracking and re-plan, proximity flight to a station port, docking into a compound, undocking
  and departure, ISS missions chained by a mission sequence: from the separation orbit, from the pad with a launch
  window, docked -> undocking -> splashdown; the depot as a station target with its approach limits, Starship phasing
  on the belly Raptor + cold gas, the docked compound following run-time tank masses).
  The data pipeline archives every fetched ISS OEM in `data/iss_oem/` (one file per creation date).
  Modules depend downwards only (e.g. `ephemeris` may use `time` and `math`, never `gnc`).
- External libraries are wrapped behind interfaces inside one module; nothing else includes them.
  CSPICE lives only in `core/src/spice/` and implements `time::TimeScaleConverter`,
  `ephemeris::Ephemeris` and `frames::BodyOrientation`.
- The game only reads simulation state and sends commands. Physics never runs in GDScript.
- The simulation runs on its own thread with a fixed tick; rendering interpolates.
- Vehicles, launch sites and missions are configuration, not code.

## Physics & numerics conventions
- SI units internally, always. Angles in radians.
- Frame and unit go into the name: `position_gcrf_m`, `velocity_itrf_mps`, `epoch_tdb_s`, `latitude_rad`.
- Inertial frame: GCRF (Earth-centred, ICRF-aligned; SPICE calls it `J2000`). Earth-fixed: ITRF93 via NAIF binary PCK.
- Time: `Epoch` stores TDB seconds past J2000. UTC only at the user interface boundary (leap seconds via SPICE LSK).
- Earth shape: WGS84 ellipsoid. Moon: sphere R = 1737.4 km (+ LOLA terrain later).
- Default gravity: point masses Earth + Moon + Sun plus Earth J2 (switchable). Earth/Moon/Sun positions
  come from DE440, only spacecraft are integrated. `FlightSession` samples them (and the Earth/Moon orientation) on
  a grid and interpolates (`InterpolatedEphemeris` 20 min, `InterpolatedOrientation` 10 min: sub-mm; fidelity flag
  `interpolated_ephemeris`) - CSPICE per force evaluation was 75 % of the planners' time.
- Simplifications are allowed only when deliberate, documented and switchable (fidelity flags).
- Rendering: floating origin at the player's head; positions are differenced in double and only then
  converted to float32.

## Quality rules
- Every physical model gets unit tests and at least one reference/validation case with a documented tolerance.
- Small files with one responsibility; prefer composition over inheritance.
- No magic numbers: physical constants live in `core/include/starsim/math/constants.hpp` with their source.
- CSPICE is not thread-safe: the `spice` adapters serialise every call with a process-wide lock (`detail::lock_spice`).

## Build & test (Windows, MSVC 2022 Build Tools, CMake, Ninja, SCons)
One-time setup:
```powershell
# Runtime data (SPICE kernels, textures, star catalogue) and CSPICE sources
python tools/data_pipeline/fetch.py
python tools/data_pipeline/prepare.py      # converts data/raw -> data/textures, data/stars (needs Pillow)

# Engine sources (git-ignored, pinned)
git clone --depth 1 --branch 4.7.2-stable https://github.com/godotengine/godot.git external/godot   # ed1daf0
git clone https://github.com/godotengine/godot-cpp.git external/godot-cpp                           # b051e69

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
- Test workflow (user 2026-10-19, replaces 2026-10-15): always run only what the change affects - the tags of the
  touched modules and of the subsystems whose code paths changed (`-TestFilter`, `[slow]` cases of those tags
  included), also before a push and before a plan's final review; no blanket `-Fast` or full-suite runs. A change to
  shared code (e.g. `FlightSession::advance_to`, `Vehicle::mass_properties`) whose new branch is inactive for other
  subsystems does not pull their campaigns in. Tag a new test case `[slow]` when it takes more than ~40 s.
- Long test runs (user 2026-10-20): a run that may take longer than 10 minutes goes to the background with its output
  in a file; check its state every 10 minutes (failures so far, a case hanging or far beyond its usual time). When
  tests are going wrong, stop the run early and fix the cause instead of waiting for the end.
- ctest runs test cases as parallel processes: tests must not share temp files or directories (make names unique).
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

## Known issues
- `optim` (SOCP solver, ECOS-style HSDE): dense Nesterov-Todd blocks per cone, so cones should stay small
  (dimension <= ~20); ordering is minimum degree on the explicit elimination graph (not full AMD). Solution
  vectors are accurate to roughly sqrt(tolerance) near cone boundaries; objective and feasibility to the tolerance.
- The very first `--import` of a fresh `game/.godot` segfaults on editor shutdown when the
  extension is present (import itself completes; later runs are fine). Not yet investigated.
- Solar radiation pressure is modelled for vehicles (FlightBody); the Artemis I coast test still uses the
  pure gravity propagator and shows SRP as a t^2 residual (~1.9e-8 m/s^2: 39 m after 18 h).
- Detailed subsystem notes - vehicle data (Block 3, Falcon 9, Crew Dragon, HLS), ascent, booster return and catch,
  SCvx landing, ship entry / flip / catch / return, legs, RCS, capsule entry and parachutes, lunar missions - with
  their decisions, validation numbers and `STARSIM_DEBUG_*` diagnostics live in `docs/subsystems.md`. Read the
  relevant section before changing a subsystem and update it with the change.
- Standing decisions from there: only Block 3 / V3 is modelled (2026-10-13); Super Heavy is always caught, Falcon 9
  lands on legs; physics first - webcast propellant gauges that the engines cannot explain are not followed
  (2026-10-14); Starship throttle schedules are guidance choices, not vehicle data.
- Earth is a 256x128 ellipsoid mesh whose facets sit up to ~0.6 km below the true surface; below 500 km a
  2x2 deg `GroundPatch` (6 cm accuracy) is drawn under the camera, at the pad height near launch sites.
  Physics ground = flat at the pad height (FlightBody::ground_height_m): gentle contact (<= 3 m/s, <= 10 deg)
  rests, harder contact is an impact. Sky is black (no atmosphere rendering). Terrain/atmosphere: M8 (incl. Earth DEM tiles for physics, e.g. Copernicus GLO-30 - user 2026-10-15; the lunar l2 terrain path is the template).
- C++ sources are UTF-8 (/utf-8); UI strings in the core are German.
- The TWCS lever reads 0 until first moved; FlightController only takes it over after a movement.
- Interactive regression check (keys injected, logs clock/altitude/throttle): `res://tests/flight_check.gd`.
- Horizons serves DE441; our DE440 Moon differs by up to 3.3 m (1990-2040). Tolerances reflect this.

## Game (Godot) structure
- `game/world/`: `world.gd` (per-frame orchestration), `celestial_body.gd` (floating origin + scaled space:
  bodies beyond 1.5e7 m are drawn closer and smaller with exact angular size), `orbit_camera.gd`
  (camera always at the render origin, true position in float64 GCRF), `frames.gd` (GCRF -> render axes:
  (x,y,z) -> (y,z,x), north = +Y), `star_field.gd`, `orbit_line.gd`, `sim_clock.gd`.
- `game/shaders/`: Earth/Moon lighting in body-local metres; `eclipse.gdshaderinc` mirrors
  `core/src/environment/eclipse.cpp` - change both together.
- Rendering uses APPARENT positions (light time + aberration) for Sun and Moon; geometric positions
  would misplace the Moon's shadow on Earth by ~40 km.
- UI text is German, code English.
- Views and controls of the finished game are specified in `docs/ansichten.md` (German, living document):
  keep it in sync when building M6 (planner), M7 (cockpit/VR) and M8.
- Flight: `game/vehicle/flight_controller.gd` (input, pilot actions) drives the C++ `FlightSim`
  (`sim::FlightSession`, 200 Hz); `FlightSim` runs `sim::SessionRunner` (own thread), getters read a snapshot,
  commands return ids (`result(id)`, signal `command_finished`); lunar missions included (`lunar_state`, `lunar_site`,
  `moon_terrain`), ISS missions too (`iss_state`, `rendezvous_plan`, `docking_state`, `station_geometry`,
  `iss_mission`, commands `proximity_*`, `undock`). `vehicle_view.gd` draws stages and plumes; `ui/flight_hud.gd` shows telemetry.
  The world clock waits for the flight sim at high time warp.
- HOTAS: Thrustmaster T.16000M + TWCS, found by device name (`config/input_hotas.toml`). Raw axes (twist,
  throttle lever) only exist when `config/sdl_gamecontrollerdb.txt` is loaded BEFORE engine start via
  `SDL_GAMECONTROLLERCONFIG_FILE` (done by `Start-Desktop.cmd`); Godot's built-in mappings drop them and
  `Input.add_joy_mapping` at runtime has no effect. Probe axes: `res://tests/joy_probe.gd`.
- Start the game with `Start-Desktop.cmd` (repo root).
- VR (M7): reuse the VR/desktop start-mode pattern of the sibling project `../Stunt-Track-Racer-VR`
  (`scripts/autoload/xr_manager.gd`, `Start-VR.cmd` / `Start-Desktop.cmd`); details in the plan, M7.

## Vehicle configuration
`config/` holds propellants, engines, vehicles and aero tables as TOML/CSV with provenance tags
([S] sourced, [E] estimate, [D] derived) - see `config/README.md`.
`python tools/vehicle_report.py <vehicle-id> [--payload-kg N]` prints masses, thrust, burn time
and the Tsiolkovsky delta-v budget; use it after every config change.

## Data licensing
Only free data. Every source is recorded with its licence in `tools/data_pipeline/sources.toml`;
`CREDITS.md` is generated from it. Check the licence when adding a source. Trademarks
("Starship", "SpaceX") are used internally only; display names come from config.
