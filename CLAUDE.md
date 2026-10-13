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
- Compact the context before each milestone of a plan (`/compact`). Before that, record the state (done, open,
  decisions, next step) in the plan's ledger/progress file so the work resumes from it. Claude cannot run
  `/compact` itself: at a milestone boundary it saves the state and asks the user to compact.

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
  contact (catch tower: geometry, arms + carriage, pin/hull penalty contact, collisions),
  aero (slender body + cross-flow table, grid fins, ship flaps, Sutton-Graves entry heating).
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
- Only the current configuration (Block 3 / V3, Raptor 3) is modelled (decision 2026-10-13); its data come from
  the flights that flew it (12-14, `tests/validation/telemetry/flights.toml`). Earlier blocks were removed.
- Starship throttle schedules are not public; they are a guidance choice, not vehicle data. Flights 12-14 fly the
  booster at ~85-90 % after max-Q with an acceleration cap of ~2.2 g; our ascent autopilot still flies full thrust.
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
- Booster recovery (decision 2026-10-13): Super Heavy is always caught (no legs; `[stage.recovery] method =
  "tower_catch"`): return guidance only at sites with a tower. Falcon 9 boosters land on the pad or the drone ship
  ("drone_ship_landing") - not modelled yet. If the first SCvx catch plan fails, an SCvx sea abort is planned at once;
  the explicit laws are the last fallback (over the arms, ~1-3 m: not accurate enough to be caught); with
  `scvx_landing = false` the whole return aims beyond the shoreline and ditches (test: 1.05 m/s, 650 m off).
- Booster return: at a catch site the landing burn flies SCvx plans (`gnc/scvx_landing`), V3 sequence 13 -> 5 (divert pair) -> 3 engines,
  re-planned every 1 s on a worker thread (effective at snapshot + 0.5 s: a re-plan starts from the snapshot
  predicted over that latency with the active plan's thrust, its start thrust within the throttle-rate reach of the
  measured thrust; tests and time warp wait, real-time play never waits - `FlightSession::set_realtime_planning`).
  The approach into the arms is slow (sink <= 1 m/s + height / 3 s, soft). From an ignition state (scenario:
  355 m/s at 2.88 km as on flights 13/14 [D]; the ignition profile flies 13 engines at 35 %) the dispersion campaign
  `[gnc][catch][validation]` (+-10 m, +-3 m/s, +-5 % mass, +-3 deg, engine failure) catches 21 of 21 (< 0.1 m, < 0.05 m/s lateral, 0.95-0.97 m/s sink at pin contact). The landing
  throttle accounts for the nozzle pressure loss (thrust = throttle x F_vac - p A_e; a throttle proportional to sea
  level thrust gave ~10 % too little thrust at low throttle). Lateral tracking is an LQR on the
  gimbaled-rocket dynamics (`gnc/tvc_lateral`); plans use <= 30 % of the gimbal limit, the LQR effort is 0.2 x the
  gimbal authority. The coast phase already rolls the pins towards the arms. Planning model: constant CG/inertia,
  no grid fins, no wind; no re-plans in the last 3 s. The explicit laws remain the fallback.
  Switch windows [E]: 13 -> 5 at 180-220 m/s sink (5 engines seen at ~200 m/s on flights 13/14), 5 -> 3 calm at
  15-30 m/s (satnews 2025-10-05: V3 ends in a hover on 3 engines).
  Full return: caught, 50.9 t at landing-burn ignition (planned 51 t, flight 14 gauge); it arrives slower than flights
  13/14 (~300 m/s at ignition, ~2.3 km). Abort to the sea is weak: a late
  abort can end hovering over land until the propellant runs out.
- Descent drag (tail first): body table `aero/super_heavy_descent.csv` [D] plus the grid fins' axial drag
  (`aero/grid_fin.csv` column `ca_axial` [E], 3 x 13.5 m^2). Identified from our webcast telemetry of flights 13 and
  14 (`tests/validation/telemetry`, `tools/telemetry/webcast_telemetry.py` + `drag_estimate.py`; below ~9 km the
  speed dial gives way to the engine icon, speed then from the 0.1 km altitude of the nearly vertical fall): the
  unguided booster (210 t dry + 51 t) follows both descents from ~27 km within 0.6 / 2.6 % rms and loses altitude to
  the landing burn within 415 / 158 m (test "Booster descent drag reproduces the webcast telemetry of flights 13
  and 14"). The webcast's speed and altitude graphics are offset by ~1.5-2 s. Subsonic drag is not measured.
- Block 3 masses [D] from flights 12-14 (`tools/telemetry/mass_estimate.py`, `ship_dv.py`): ship dry 255 t
  (rocket equation staging -> cutoff with the propellant gauge, 248-260 t), booster dry 210 t +-40 t (stack mass
  balance and descent ballistic coefficient), liftoff ~5,550 t; tank capacities 3,650 / 1,600 t [S]; Raptor 3
  280 tf consistent (peaks ~270 tf sea-level equivalent; 250 tf is contradicted), minimum throttle 30 % [D] (V3
  hovers on 3 engines). MECO of a returning booster is planned by the ascent autopilot from the state
  (`BoosterReturnGuidance::meco_propellant_kg`: run-down + flip + boostback by the rocket equation on the Lambert
  delta-v after the flip, +8.6 % [D] for the drag-shifted aim point and finite burn) so that `landing_propellant_kg`
  = 51 t [D] (flight 14 gauge) remain at landing-burn ignition (test: 50.9 t; 30 / 70 t targets within 0.4 t);
  it stages with ~207 t. `reserve_propellant_kg` (225 t) is the MECO without return guidance. Real boosters stage
  with ~530 t (~15 %) and burn ~470 t in the boostback.
- Catch tower dimensions and azimuths (`[site.tower]`, Starbase 315 deg, LC-39A 0 deg) and the catch pins
  (`[stage.catch_pins]`, 5 MN per pin) are estimates [E]; verify the tower azimuths against imagery.

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
  side the top leans to) -, landing burn (explicit fallback 13 -> 3 engines; SCvx 13 -> 5 -> 3): 1-D burn simulation
  for ignition, ZEM/ZEV vertical to the gate, then the approach profile sink = 1 m/s + height / 3 s, braking-profile
  divert). It uses its own faster attitude controller with grid fins enabled.
  The attitude controller limits the rate command to what the available angular acceleration (gimbal or
  RCS, plus fins) can still stop. Diagnostics: `STARSIM_VERBOSE=1`, `STARSIM_DEBUG_LANDING=1`,
  `STARSIM_DEBUG_HOLD=1` with test "[booster]".
- SCvx landing planner (M6, Plan 3a; flown since Plan 3b): `gnc/descent_*` plans the landing burn in
  `descent_phases` = 3 engine phases (13 -> 5 -> 3; 10 nodes per phase, neighbours share a node)
  in the plan frame (origin pad centre on the ground, x up, y east, z north; upright = identity attitude):
  `descent_model` (14-state rigid body, tabulated air/drag, Coriolis + centrifugal, numeric Jacobians;
  `make_descent_model` builds it from the booster on the simulation thread), `descent_discretization` (FOH +
  RK4 sensitivities), `descent_problem` (convex subproblem in scaled deviations: thrust/gimbal/tilt/rate/glide
  cones, catch or sea-abort goal with tower keep-out; a shared phase node has the next set's bounds, engine
  switches have sink windows, the last one is calm),
  `descent_planner` (`plan_descent`: penalised trust region, warm start, converged when the virtual control is
  small, the step or final mass has settled and the nonlinear interval defects are < 5 cm). Uses the reusable
  `optim::SocpSolver`. Nominal block 3 case: 6 iterations, ~0.5 s fresh; re-plans 1-4 iterations, < 0.5 s. In flight (Plan 3b): `gnc/descent_snapshot`
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
- Ship entry (b1 of `docs/superpowers/specs/2026-10-13-ship-return-design.md`): belly first. Hull =
  `SlenderBodyAero` with the ship's cross-flow table `aero/ship_crossflow.csv` [D] (used when the ship flies alone;
  shaped like a cylinder in cross-flow: transonic rise to 1.9, 1.5-1.6 hypersonic). Four flaps (`aero/flap`,
  `[[stage.flap]]` [E] aft 2 x 48 m^2, forward 2 x 24 m^2 moved 20 deg leeward; Newtonian flat plates,
  `aero/flap.csv`; 0 deg folded leeward, 90 deg broadside; 20 deg/s). `AttitudeController::Gains::flaps`: the flaps
  cancel the modelled aero moment (hull + flaps) and steer by a Newton step on their linearised moments
  (minimum-norm, per-flap clamp, null space drawn to mid-range), the rest goes to RCS. `gnc/entry_attitude`
  builds the setpoint from angle of attack and bank (belly at `[stage.aero] belly_azimuth_deg`). Nominal alpha
  over Mach `aero/ship_entry_alpha.csv` [D] (55 deg hypersonic, 50 near Mach 8, 60-80 towards the belly flop).
  Ship dry CG 28 m [D] (the trim requirement: at 20 m it sat ~9 m behind the hull's centre of pressure).
  Entry control: 0.5 rad/s and bank changes <= 1 deg/s (0.8 rad/s limit-cycles in roll with 20 deg/s flaps;
  faster bank changes depart in alpha). Validation: replay of flights 12-14 (bank follows the recorded altitude)
  - speed over time above Mach 3 within 1.6 / 2.3 / 4.9 % rms, below Mach 3 over altitude 6.0 / 5.2 %,
  belly-flop speed within 2-3 % (test "Ship entry reproduces the webcast telemetry of flights 12-14").
  Corridor (`[stage.heat_shield]`, judged while the ship flies alone): Sutton-Graves stagnation heat rate
  (`aero/entry_heating`, nose radius 6 m [E]), heat load, aerodynamic load (thrust not counted); beyond a limit the
  ship breaks up ("Hitzeschild versagt" / "Strukturüberlast", outcome "zerbrochen"), warning from 80 %; skip-out
  event ("Abprall": below 80 km, then above 100 km). Heating is integrated in `step()` only. Entry mass in tests:
  dry + `landing_propellant_kg` (25 t [E]) in the main tanks (no header tanks modelled).
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
