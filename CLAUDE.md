# Super Heavy / Starship Simulator

Physically accurate simulator for Super Heavy + Starship: a playable PC-VR flight simulator
(OpenXR, HOTAS, seated cockpit) and a mission/engineering simulation (planning, telemetry, validation).
Not an arcade game: physical correctness has top priority.

## Communication & language
- Answer the user **in German**.
- Code, identifiers, comments, commit messages and docs inside the repo: **English**.

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
  `environment`, `integrators`, `orbit`, `vehicle`, `propulsion`, `aero`, `contact`, `gnc`, `mission`, `sim`.
  Implemented so far: math, time, frames, ephemeris, spice (adapter), environment (gravity),
  integrators (RK4, DOP853 + dense output, terminal events), orbit (Kepler, coast propagation),
  eclipse geometry (disk occultation).
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
  come from DE440, only spacecraft are integrated.
- Simplifications are allowed only when deliberate, documented and switchable (fidelity flags).
- Rendering: floating origin at the player's head; positions are differenced in double and only then
  converted to float32.

## Quality rules
- Every physical model gets unit tests and at least one reference/validation case with a documented tolerance.
- Small files with one responsibility; prefer composition over inheritance.
- No magic numbers: physical constants live in `core/include/starsim/math/constants.hpp` with their source.
- CSPICE is not thread-safe: only the simulation thread calls `ephemeris`.

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
powershell -ExecutionPolicy Bypass -File tools/build.ps1          # core + GDExtension + ctest
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings
# Godot smoke test (extension loads, float64 end to end)
external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/smoke_test.gd
```
- Visual check (needs GPU, no --headless): `godot --path game --script res://tests/render_check.gd --
  --utc=2024-04-08T18:42:00 --focus=earth --view_from=moon --out=<png>`; ground views with
  `--lat= --lon= --look_at_moon --fov=`. Read the PNG to verify rendering changes.
- `STARSIM_VERBOSE=1` makes validation tests print per-epoch errors.
- `core/include/starsim/integrators/dop853_tableau.hpp` is generated by `tools/codegen/gen_dop853_tableau.py`
  from SciPy's coefficient file - never edit it by hand.
- Validation fixtures are regenerated with `python tools/validation/fetch_horizons.py` (committed).
- The GDExtension DLL is written to `game/bin/` (git-ignored).

## Known issues
- The very first `--import` of a fresh `game/.godot` segfaults on editor shutdown when the
  extension is present (import itself completes; later runs are fine). Not yet investigated.
- Solar radiation pressure is modelled for vehicles (FlightBody); the Artemis I coast test still uses the
  pure gravity propagator and shows SRP as a t^2 residual (~1.9e-8 m/s^2: 39 m after 18 h).
- Starship throttle schedules are not public: the IFT-3 test reconstructs the throttle from measured
  acceleration (mean 0.75 of nameplate) and checks consistency (speed 4.3 %, altitude 7.9 %, max-Q).
- Earth is a 256x128 ellipsoid mesh whose facets sit up to ~0.6 km below the true surface; below 500 km a
  2x2 deg `GroundPatch` (6 cm accuracy) is drawn under the camera, at the pad height near launch sites.
  Physics ground = flat at the pad height (FlightBody::ground_height_m): gentle contact (<= 3 m/s, <= 10 deg)
  rests, harder contact is an impact. Sky is black (no atmosphere rendering). Terrain/atmosphere: M8.
- Orbit targets are judged on osculating elements at cutoff. Burn planning helpers are two-body; in the J2
  field a Kepler-circular orbit oscillates ~10 km in radius (J2 raises gravity ~0.14 %). J2-aware targeting: open.
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
- Flight: `game/vehicle/flight_controller.gd` (input, pilot actions) drives the C++ `FlightSim`
  (`sim::FlightSession`, 200 Hz); `vehicle_view.gd` draws stages and plumes; `ui/flight_hud.gd` shows telemetry.
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
