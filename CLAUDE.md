# Super Heavy / Starship Simulator

Physically accurate simulator for Super Heavy + Starship: a playable PC-VR flight simulator
(OpenXR, HOTAS, seated cockpit) and a mission/engineering simulation (planning, telemetry, validation).
Not an arcade game: physical correctness has top priority.

## Communication & language
- Answer the user **in German**.
- Code, identifiers, comments, commit messages and docs inside the repo: **English**.
- Commit messages follow Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`, ...): release-please
  builds the release PR and CHANGELOG.md from them. Releases are made by the user (merging that PR).

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
  Implemented so far: math, time, frames, ephemeris, spice (adapter), environment (gravity),
  integrators (RK4, DOP853 + dense output, terminal events), orbit (Kepler, coast propagation),
  eclipse geometry (disk occultation), optim (sparse SOCP interior-point solver),
  contact (catch tower: geometry, arms + carriage, pin/hull penalty contact, collisions).
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
- Booster return: on a site without a tower blocks 1-3 land on the pad with the explicit laws (no impact, sink
  < 2 m/s, 4-11 m off, < 1.2 m/s lateral). At a catch site the landing burn flies SCvx plans (`gnc/scvx_landing`)
  re-planned every 1 s on a worker thread (effective at snapshot + 0.5 s; tests and time warp wait, real-time play
  never waits - `FlightSession::set_realtime_planning`). From an ignition state (scenario: 355 m/s at ~1.85 km,
  Flight 5 webcast) the dispersion campaign `[gnc][catch][validation]` (+-10 m, +-3 m/s, +-5 % mass, +-3 deg,
  engine failure) catches 21 of 21 (< 1 m, < 0.5 m/s). Profile close to recorded catches: 13 engines ~4.5 s,
  switch at 70-95 m/s (~750 m), 3 engines ~16 s. Lateral tracking is an LQR on the gimbaled-rocket dynamics
  (`gnc/tvc_lateral`; the gimbal's wrong-way side force makes cascaded position/attitude loops swing up on 3
  engines); plans use <= 30 % of the gimbal limit, the LQR effort is 0.2 x the gimbal authority at the current
  thrust. The coast phase already rolls the pins towards the arms (grid fins / RCS). Planning model: constant
  CG/inertia, no grid fins, no wind; no re-plans in the last 3 s. The explicit laws remain the fallback.
  Full returns: block 3 is caught; blocks 1/2 still abort (`[!mayfail]`): our ascent stages too high and fast
  (block 1: T+134 s, 83 km, 2.0 km/s; IFT-3: ~T+165 s, ~70 km, ~1.57 km/s - the autopilot flies the booster at
  full throttle, IFT-3's reconstructed throttle averages 0.75), so the booster reaches 131 km (IFT-3: 106 km)
  and arrives too fast. Next: an ascent throttle schedule from the IFT-3 reconstruction.
  Abort to the sea is weak: a late abort (< ~250 m) on 3 engines near minimum throttle (T/W > 1, cannot hover or
  descend slowly) can end hovering over land until the propellant runs out.
- Descent drag (tail first): body table `aero/super_heavy_descent.csv` plus the grid fins' axial drag
  (`aero/grid_fin.csv` column `ca_axial`, per fin count/area) [E], calibrated [D] (x1.0 to Mach 1.2, x1.3 from
  Mach 2.5) so that the unguided block 1 booster placed in IFT-3's T+330 s state matches the webcast speeds at
  31.5/19.8/10.1/4.8 km and Flight 5's at 2 km within 3 % (test "Booster descent drag reproduces the IFT-3
  telemetry"; 72 t propellant assumed, the real mass is not public).
- Booster return reserves (block 1/2: 330 t, block 3: 280 t) and the grid fin normal-force data (`aero/grid_fin.csv`,
  sizes/stations) are estimates [E]. Block 3 is checked against the FAA EA's ~74 t residual propellant at
  landing-burn ignition (Raptor 3 at 280 tf: 70.3 t at ignition).
- Catch tower dimensions and azimuths (`[site.tower]`, Starbase 315 deg, LC-39A 0 deg) and the catch pins
  (`[stage.catch_pins]`, 5 MN per pin) are estimates [E]; verify the tower azimuths against imagery.
- Wikipedia's booster load 2,700 t LOX / 700 t CH4 (O/F 3.86) does not match Raptor's 3.6: blocks 1/2 now
  split the 3,400 t total at 3.6 [D] (`vehicle_report.py` prints the loaded O/F).

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
  (`sim::FlightSession`, 200 Hz); `vehicle_view.gd` draws stages and plumes; `ui/flight_hud.gd` shows telemetry.
  The world clock waits for the flight sim at high time warp.
- HOTAS: Thrustmaster T.16000M + TWCS, found by device name (`config/input_hotas.toml`). Raw axes (twist,
  throttle lever) only exist when `config/sdl_gamecontrollerdb.txt` is loaded BEFORE engine start via
  `SDL_GAMECONTROLLERCONFIG_FILE` (done by `Start-Desktop.cmd`); Godot's built-in mappings drop them and
  `Input.add_joy_mapping` at runtime has no effect. Probe axes: `res://tests/joy_probe.gd`.
- Start the game with `Start-Desktop.cmd` (repo root).
- GNC (M5): `gnc/ascent_guidance` (pitch program + PEG + terminal v_go steering), `gnc/burn_node`
  (closed-loop burns), autopilot state machine in `sim::FlightSession`. Burns pick the strongest engine
  configuration whose start/shutdown impulse is <= 1/4 of the delta-v; smaller burns are rejected (RCS
  translation comes with docking, M9). Game keys: P ascent autopilot (config/missions/leo_ascent.toml),
  O circularize at apoapsis, X autopilot off. HUD: NavBall (`ui/nav_ball.gd`), flight director
  (`ui/flight_overlay.gd`), trajectories (`vehicle/trajectory_view.gd`: green planned, orange predicted).
- Catch tower (M6, `contact/*`, `sim/catch_tower_sim`): a site loaded with `config::load_launch_site` /
  `sim::launch_site_from_config` carries its tower; stacks then stand on the launch mount (20 m). The tower
  closes its arms automatically (time to pin contact <= closing time + 1 s, axis offset <= 3 m), applies
  penalty contact forces (pins and a hull ring vs the arm boxes, friction 0.3) through
  `FlightBody::apply_external_wrench`, and judges: caught (both pins, engines off, at rest 2 s ->
  `FlightBody::hold_fixed_to_earth`), impact (tower/mount hit, pin load > `max_load_n`, carriage hard stop
  > 1 m/s, an arm > 0.2 m deep in the hull) or missed. With a tower the booster's landing burn aims at the
  catch point, rolls its pins towards the arms and cuts off on pin contact. Fidelity flag
  `FlightSession::set_towers`; `FlightSession::place_body` places a vehicle freely (scenarios).
  Game: `game/world/tower_view.gd` draws the tower from `FlightSim.site_tower`, arms from `tower_state`
  (render_check `--tower_arms_deg=`); a new `class_name` needs one `godot --headless --import`.
- Booster return (M6, `gnc/booster_return`): `FlightSession::set_booster_return(true)` gives the separated
  booster its own guidance (flip, Lambert boostback with drag-aware aim-point shooting, coast: RCS impact
  trim above ~2 kPa dynamic pressure, below it grid-fin steering - angle of attack found by bisection on
  the aero model (for the blunt tail-first booster the tilted axial drag dominates: the force points to the
  side the top leans to) -, landing burn 13 -> 3 engines: 1-D burn simulation for ignition, ZEM/ZEV
  vertical, braking-profile divert). It uses its own faster attitude controller with grid fins enabled.
  The attitude controller limits the rate command to what the available angular acceleration (gimbal or
  RCS, plus fins) can still stop. Diagnostics: `STARSIM_VERBOSE=1`, `STARSIM_DEBUG_LANDING=1`,
  `STARSIM_DEBUG_HOLD=1` with test "[booster]".
- SCvx landing planner (M6, Plan 3a; flown since Plan 3b): `gnc/descent_*` plans the landing burn (13 -> 3 engines)
  in the plan frame (origin pad centre on the ground, x up, y east, z north; upright = identity attitude):
  `descent_model` (14-state rigid body, tabulated air/drag, Coriolis + centrifugal, numeric Jacobians;
  `make_descent_model` builds it from the booster on the simulation thread), `descent_discretization` (FOH +
  RK4 sensitivities), `descent_problem` (convex subproblem in scaled deviations: thrust/gimbal/tilt/rate/glide
  cones, catch or sea-abort goal with tower keep-out; the shared phase node has the 3-engine bounds),
  `descent_planner` (`plan_descent`: penalised trust region, warm start, converged when the virtual control is
  small, the step or final mass has settled and the nonlinear interval defects are < 5 cm). Uses the reusable
  `optim::SocpSolver`. Nominal block 3 case: 4 iterations, ~0.2 s. In flight (Plan 3b): `gnc/descent_snapshot`
  (sim-thread snapshot; the worker never calls SPICE), `gnc/descent_replanner` (worker thread, fixed sim-time
  latency), `gnc/descent_tracking` (plan reference sampling), `gnc/tvc_lateral` (LQR gains), `gnc/scvx_landing`
  (first plan, re-plans, abort to the sea after repeated unreachable catches, commands incl. a body torque);
  `sim/landing_outcome` (shoreline `[site.sea]`, "Wasserung"); tower `CatchPhase::Aborted` (arms stay open).
  Diagnostics: `STARSIM_DEBUG_SCVX=1`, `STARSIM_DEBUG_SCVX_SOLVER=1`, `STARSIM_DEBUG_CATCH=1`.
- Grid fins (`aero/grid_fin`, `[stage.grid_fins]`): flat-plate style normal force from the local flow at
  each fin (incl. omega x r: aerodynamic damping), slope/limit over Mach from a table (transonic choking).
  Actuators: deflection and rate limits (`Vehicle::update_grid_fins`). The controller allocates torque to
  the fins first (minimum-norm least squares), the remainder to gimbals/RCS, using the air data of the
  last tick (`FlightBody::air_data()`); fins are used only when `AttitudeController::Gains::grid_fins`.
- `FlightBody::resting()`: carried by the ground after a gentle touchdown; resting bodies coast analytically
  (follow the Earth), impacted/resting bodies do not block coast segments.
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
