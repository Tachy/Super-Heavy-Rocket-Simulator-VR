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
  contact (catch tower: geometry, arms + carriage, pin/hull penalty contact, collisions; landing legs and surfaces),
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
- Starship throttle schedules are not public; they are a guidance choice, not vehicle data. The ascent mission
  `config/missions/starship_leo.toml` (`sim::start_ascent_mission`, `FlightSim.start_mission`) carries the booster
  schedule [D] of flights 13/14 (gauge shape, level fitted to the recorded speed: ~0.85, max-Q bucket ~0.76, ~0.83, ~0.76
  towards MECO) and kick 14 deg: test "Starship ascent follows the webcast telemetry of flights 13 and 14" (flown with the
  recorded MECO of ~500 t: speed within 3-6 %, altitude within ~1 km at T+60-120 s, MECO T+135 s / 48 km / 5840 km/h vs
  ~T+140 s / ~53 km / ~5950 km/h). The game's key P and the older ascent tests still fly full thrust with the default
  pitch program. Known gap: our return guidance plans MECO with ~207 t; the flights stage with ~500 t (as for Falcon 9,
  the real return spends far more than our Lambert boostback + 8.6 %): flight 14's gauge falls 11.8 -> 3.5 % (~300 t)
  in ~25 s on 13 engines (their full mass flow) - ~2.2 km/s by the rocket equation (dry 210 t), our boostback ~1.5 km/s;
  the real booster also arrives faster at landing-burn ignition (355 vs ~300 m/s). Next return-guidance topic.
- Earth is a 256x128 ellipsoid mesh whose facets sit up to ~0.6 km below the true surface; below 500 km a
  2x2 deg `GroundPatch` (6 cm accuracy) is drawn under the camera, at the pad height near launch sites.
  Physics ground = flat at the pad height (FlightBody::ground_height_m): gentle contact (<= 3 m/s, <= 10 deg)
  rests, harder contact is an impact. Sky is black (no atmosphere rendering). Terrain/atmosphere: M8.
- Circular ascent targets are J2-aware (`gnc/j2_insertion`): when the gravity model has J2
  (`GravityModel::central_j2`), PEG aims at the radius, radial and horizontal speed at the predicted cutoff point whose
  next revolution stays closest to the target radius (Gauss-Newton on a point-mass + J2 revolution, re-solved when the
  cutoff point moves > 2 deg). A Kepler-circular insertion swung 5-20 km in radius (Starship 26.5 deg: 190.8-200.0 km);
  now the flown revolution keeps 199.7-201.6 km - the J2 short-period swing left is 0.7 / 2.1 / 3.3 km at 26.5 / 51.6 /
  97 deg. Circular-orbit tests judge the flown revolution; elliptical targets (Starlink 200 x 280 km) are still judged
  on osculating elements at cutoff (Kepler). Burn planning helpers are two-body.
- C++ sources are UTF-8 (/utf-8); UI strings in the core are German.
- The TWCS lever reads 0 until first moved; FlightController only takes it over after a movement.
- Interactive regression check (keys injected, logs clock/altitude/throttle): `res://tests/flight_check.gd`.
- Horizons serves DE441; our DE440 Moon differs by up to 3.3 m (1990-2040). Tolerances reflect this.
- Booster recovery (decision 2026-10-13): Super Heavy is always caught (no legs; `[stage.recovery] method =
  "tower_catch"`): return guidance only at sites with a tower. Falcon 9 boosters land on the pad or the drone ship
  ("leg_landing", milestone c: `docs/superpowers/specs/2026-10-13-falcon9-landing-design.md`;
  vehicle, legs, the RTLS to LZ-40 (c3), the drone ship (c4) and the game interface (c5) done; the drawing and UI wait
  for the user's UI specification). If the first SCvx catch plan fails, an SCvx sea abort is planned at once;
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
  13/14 (~300 m/s at ignition, ~2.3 km). Abort plans must end beyond the shoreline (`DescentTarget::sea_min_distance_m`
  = shoreline + `sea_clearance_m` 30 m [E], a hard constraint) and pull towards the sea target (soft); only when the
  water is out of reach do the plans fall back to the soft pull (before: the soft pull alone, 100 kg per metre, set the
  booster down beside the tower). `FlightSession::abort_landing` / `FlightSim.abort_landing` is the pilot's abort; test
  "A commanded abort late in the landing burn ditches the booster at sea" (1500 / 800 / 400 m: water, 25.7 / 14.6 /
  5.7 t left).
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
  O circularize at apoapsis, R ship return to the site's tower (b4), X autopilot off. HUD: NavBall (`ui/nav_ball.gd`), flight director
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
  dry + `landing_propellant_kg` (35 t [E]) in the main tanks (no header tanks modelled).
  The ship alone carries the stack's axial drag table [E] (`ascent_drag_table`; before b2 a ship built alone had none).
- Ship entry guidance (b2, `gnc/entry_guidance`, `gnc/entry_predictor`, `gnc/entry_aero_table`): the onboard computer
  predicts the rest of the entry as a 3-DOF point mass (Earth-fixed, J2, US76, the ship's trimmed C_D A / C_L A over
  Mach and alpha with flaps in pitch trim, online drag / lift scale factors from the accelerometer
  `FlightBody::aero_acceleration_body_mps2()`; pure, no SPICE) every 2 s and solves the bank magnitude for zero range
  error (Illinois); heat-rate, load and dynamic-pressure peaks over 90 % of a limit cut the bank back. Bank reversals are
  timed by the predictor (hold vs reverse-now end on opposite sides). Bank rate 1 deg/s above Mach 10, 3 deg/s below
  Mach 5 [E]. From Mach 5 (latest Mach 3, once a reversal has settled) the terminal phase ("Endanflug") steers the
  lift in the velocity frame relative to the alpha schedule (Levenberg-Marquardt steps on the predicted gate miss,
  starting from the entry's bank); the gate ("Tor") is 800 m [E] above the pad, where b3 flips (flight 13 relit at
  ~0.95 km [D]; at 950 m campaign case 4 misses by 212 m). `FlightSession::place_entering_ship` (scenarios),
  `entry_status` (setpoint; bindings telemetry `entry_*`, HUD lines). Campaign `[ship][guidance][validation]`
  (12 dispersed entries to Starbase, truth aero +-10 %): 11 within 0.11 m, case 4 marginal (roundoff moves it between
  ~0.1 and ~1.2 km: open), belly <= 30 deg off down at the gate; load limit held.
  Tests use `predictor_j2 = false` (the scenario Earth has central gravity; J2 alone shifts a long glide ~2 %).
- Ship flip and catch (b3, `gnc/ship_landing`): at the gate the three sea-level Raptors relight at 80 % [E], the aft
  flaps fold and the forward flaps go broadside until the tilt is < 30 deg (the tail drops while the engines spool);
  the flip takes ~4 s (flights: 2-3 s; Raptor start-up 3 s [E]) and ~8 t, and pushes the ship ~100 m along its nose -
  the gate lies back along the pad -> tower line with the nose to the tower, so the side pins (0/180 deg) lie across the
  arms without a roll. Once upright (< 15 deg, < 0.2 rad/s) SCvx plans (`ship_landing_parameters()`) fly 3 -> 2 engines:
  the belly engine shuts down at 20-40 m/s sink [D: flight 14 video, two engines from ~200 m], the leeward pair lands
  (layout [D] from the flight 13 splashdown video; its thrust centroid lies in the symmetry plane). Thrust of an
  asymmetric set goes through the CG along an axis tilted against the hull (~1.5 deg for the pair): plan states carry
  that thrust-axis attitude (`gnc::thrust_axis_body`), else the LQR (no integrator) holds a ~2 m offset. Ship LQR
  effort 0.1 of the gimbal authority [E] (more limit-cycles after the flip), attitude controller with
  `Gains::actuator_delay_s` 0.75 s (gimbal slew in the braking; no overshoot). Flaps folded in the burn.
  `FlightSession::place_landing_ship` (scenarios); the entry hands over at the gate. Test from the gate: caught 0.01 m,
  0.94 m/s sink, 5.7 t left; campaign `[ship][catch][validation]` (+-10 m, +-10 m/s, +-5 % propellant, +-5 deg alpha /
  bank, belly engine failed in the flip): 20 of 21 caught (<= 0.02 m), the failure aborts to the sea.
  Gate lead: `gnc::ship_entry_target` puts the entry's target 220 m [E] back from the catch axis on the line tower -> pad
  with `EntryTarget::nose_direction_itrf` to the tower. Only that approach is robust: the arms stand across the line,
  a burn arriving from the side (azimuth 45/225 deg) runs into an arm at arm height; flip-push 20 deg off the line
  already ends there. The terminal phase turns the nose once the fall is steep (basis frozen, -70 deg): the steering
  turns to the nearest edge of the nose sector at fraction 0.2 (alpha ~86 deg), the lift direction turns about the air
  velocity at 3 deg/s [E] (a shortest-path attitude slew passes through low alpha: ~1 km miss) with the steering held,
  then Levenberg-Marquardt steps in sector coordinates (fraction 0.02-0.5; the sector narrows from +-30 deg 4 km above
  the gate to +-10 deg at it [E]). The predictor flies the same turn at its rate (an instant turn hid the lift swept in
  it: 0.4-0.8 km overshoots). The commanded attitude slews at 1.5x the turn rate and its rate is fed forward to the
  entry controller (`EntryCommand::rate_gcrf_radps`; without it the PD loop lagged the turn by ~18 deg of alpha), which
  has an actuator delay of 0.5 s [E] (2x slew or 5 deg/s set off a roll limit cycle on the flaps: departures). Session
  test from Mach 2.6, 28 km west: gate < 1 m off, caught 0.02 m, 0.95 m/s, 2.1 t left. Energy-short terminal phase (36 km
  from Mach 2.6; not seen from orbit, the deorbit plan arrives with energy to spare): the gate is hit (0.3 m), but the fall
  turns steep too late for the nose turn (137 deg off the tower direction): the flip pushes away from the tower, the burn
  aborts and the ship ditches at sea (1.4 t left) - safe, no catch. Ship catch propellant margin:
  5 of 21 dispersed gates arrive with <= 0.7 t (gate +-10 m/s: up to ~39 m/s across after the flip, braked high inside
  the glide cone); campaign case 12 (seed 13) reaches the catch point before the arms have closed, runs dry and drops
  ~1 m onto the arms at 4.5 m/s - judged "gefangen" correctly (carriage absorbs it, pin loads below the limit).
  Diagnostics: `STARSIM_DEBUG_FLIP=1`, `STARSIM_DEBUG_BURN=1`, `STARSIM_DEBUG_ENTRY=1` with the SCvx / catch flags;
  hidden test "Ship catch single case" with `STARSIM_SHIP_SEED=<1-20>`.
- Ship return from orbit (b4, `config/missions/ship_return_starbase.toml`, `config::load_ship_return_mission`):
  `FlightSession::plan_ship_return` (game key R) plans the deorbit burn on the onboard computer (`gnc/deorbit_planner`:
  coast samples over 48 h, passes = closest approaches of the ground track; per pass the burn epoch is bracketed on the
  entry predictor's downrange error and burn epoch + retrograde delta-v are shot (Newton) to the 120 km entry interface
  at -1 deg with the prediction at a nominal 50 deg bank (both signs averaged) ending at the gate; the first pass within
  60 km crossrange [E] is taken; refused without the landing reserve or without a pass). The burn flies on
  `[stage.recovery] deorbit_engine_groups` - the ship: the belly sea-level Raptor alone (user 2026-10-13), the hull turned
  so that its thrust axis through the CG (`gnc::thrust_axis_body`, ~3 deg off the hull) points along the burn; the burn
  executor senses along that axis (`FlightSession::execute_burn(node, groups)`). `AutopilotState::Return` ("Rückkehr")
  coasts to 300 s before the entry interface, then the entry guidance (b2) and the flip and catch (b3) take over.
  From 200 km / 26.5 deg: burn after ~21-22 h (one pass a day passes Starbase within reach), 65-78 m/s, 5.6-6.8 t.
  Campaign `[ship][mission][validation]` (orbit phase 0-315 deg, orbit 190-230 km, 42-48 t, deorbit execution
  +-1 % / 0.5 deg via `FlightSession::apply_velocity_error`): 14 of 14 caught, gate ~1 m, 8.9-13.9 t left (~40 s each).
  Diagnostics: `STARSIM_DEBUG_DEORBIT=1`; test "Ship return single case" (hidden, `STARSIM_RETURN_PHASE=<deg>` or
  `STARSIM_RETURN_DISPERSED=<i>`, with `STARSIM_VERBOSE=1`).
- `FlightBody::resting()`: carried by the ground after a gentle touchdown; resting bodies coast analytically
  (follow the Earth), impacted/resting bodies do not block coast segments. A freely falling body below 5 km above its
  ground is ticked (a coast segment would fall through the ground before its single contact check).
- Falcon 9 (milestone c, `config/vehicles/falcon9_block5.toml`, Block 5 only): octaweb groups `center` / `pair` /
  `ring`, 4 grid fins, cold-gas RCS, `[stage.landing_legs]`; sites `ccsfs_slc40` (launch, no tower) and `ccsfs_lz40`
  (`kind = "landing_zone"`, ~340 m east of the pad [E], 85 m pad [S]). Evidence: our webcast telemetry of CRS-35 (RTLS,
  LZ-40) and Starlink 15-25 (drone ship) - same round-dial layout as Starship (`webcast_telemetry.py clean --vehicle
  falcon9`: stage 1 left / stage 2 right after separation, G-FORCE and the stage-1 engine icon + propellant gauge
  before; the icon replaces the speed dial in the entry and landing burns). Derived [D]: first-stage propellant 425 t
  (decision 2026-10-14: the gauge falls 0.63 %/s at full thrust - nine Merlins at their 845 kN rating burn 2.75 t/s; the
  telemetry fixes only capacity x mass flow: the alternative, the commonly cited 395.7 t with the engines at ~93 % all
  ascent, keeps no official figure; `tools/telemetry/mass_estimate.py --area-m2 10.52`; with 425 t the simulated G
  matches the dial before max-Q), second-stage propellant 106.7 t, landing propellant 12.1 t (gauge 2.8-2.9 % at
  landing-burn ignition on both flights; touchdown 1.7 % = 7.2 t), MECO gauge 66 t RTLS / 47 t drone ship, descent drag
  table `aero/falcon9_descent.csv` (unguided replay from after the entry burn: 0.9 / 1.5 % rms, test "Falcon 9 descent
  drag reproduces ..."). Burns seen: CRS-35 entry burn on the centre engine only, landing burn 1 -> 3 -> 1; Starlink 15-25
  entry burn on 3, landing burn on 1 (~40 % throttle by the gauge). The drone-ship descent stays ~2 km higher than a
  zero-lift fall (grid-fin lift). Ascent: `PitchProgram::booster_throttle` (first-stage throttle over mission time,
  empty = full) carries the Falcon 9 schedule [D] from the CRS-35 G-force dial against the simulated stack: full thrust,
  a ~72 % max-Q bucket T+50-61 s, then ~95 % (`STARSIM_DEBUG_ASCENT_G=1` prints the simulated G); pitch kick 12 deg [E];
  CRS-35 T+60 / 120 s 1219 / 4807 km/h (recorded 1188 / 4672: the real stack starts ~2.5 s slower), MECO T+137 s /
  56.4 km / 6372 km/h vs recorded 137 / 56.6 / 6441. The upper stage's plane steering nulls out-of-plane speed and offset together
  (ZEM/ZEV) and the terminal v_go removes the rest; inclinations of steep planes are judged against the true equator of
  date (GCRF elements carry ~0.15 deg of precession by 2026). `FlightSession::launch_from_pad` also takes a loaded
  `VehicleConfig` (mission payload; Dragon flies without a fairing).
- Landing legs (c2, `contact/landing_legs`, `contact/landing_surface`, `sim/leg_landing_sim`): `Vehicle::deploy_legs`
  (sticky, `deploy_time_s`; `FlightSession::deploy_legs` / `set_leg_deployment`); once the legs move,
  `LegLandingSim` owns the body's ground contact: four feet as spring-dampers in series with plastic crush cores
  (`stroke_m`, `crush_force_n`), friction capped at what stops a foot's slip within a tick (the small roll inertia made
  plain regularised friction limit-cycle). Surface: the site's ground plane with its shoreline, or a deck over the sea
  (`FlightSession::set_landing_surface`). Judgement (`FlightSession::leg_touchdown`): "gelandet" (>= 3 feet, engines
  off, at rest 2 s -> `FlightBody::hold_landed`), "umgekippt" (> 30 deg on a foot), "Aufprall" (core used up, base on
  the surface, legs not fully out), "Wasserung". Drop tests `[legs]`: 1/3/6 m/s land (6 m/s crushes 0.36 m), 10 m/s and
  25 deg at 2 m/s are lost. Diagnostics: `STARSIM_DEBUG_LEGS=1`.
- Falcon 9 RTLS (c3, `config/missions/falcon9_rtls.toml`, `config::load_ascent_mission`): `FlightSession::set_landing_zone`
  (after `launch_from_pad`; the legs land on the zone's ground) makes a `"leg_landing"` stage return to the zone
  (`returns_after_staging`, `return_site` with a `gnc::PadTarget`). The return guidance is generic over the vehicle:
  `gnc::booster_return_parameters(stage)` takes engine sets, return flight time, boostback loss fraction, entry burn
  and the landing-burn profile from `[stage.recovery]` (Super Heavy has none of these keys: defaults unchanged).
  Phases Flip -> Boostback -> Coast -> EntryBurn ("Wiedereintritt", Falcon 9 only: retrograde on the centre engine from
  56.6 km for 125 m/s [D], leaning <= 5 deg towards the impact miss) -> Coast -> LandingBurn. The impact prediction
  carries the entry burn still ahead (a finite burn since c4). The landing burn flies SCvx plans with the goal `DescentGoal::Pad`
  (terminal state of a catch at leg-contact height, sink 1 m/s, no corridor, no exponential approach: one Merlin at
  minimum throttle exceeds the weight), 1 -> 3 -> 1 engines with switch windows 230-270 / 140-190 m/s [D]; the legs
  deploy 5 s [E] + deploy time before the planned touchdown, the engines stop on leg contact or 0.5 s after the plan's
  end; an unreachable zone aborts to the sea ("Wasserung"). `LandingOutcome::TippedOver` ("umgekippt") comes from the
  leg judgement. Validation: from ignition (3.3 km, 300 m/s, 12.1 t) lands 0.01 m off at 0.88 m/s; campaign
  `[falcon9][rtls][validation]` (+-10 m, +-3 m/s, +-5 % propellant, +-3 deg, truth hull aero +-10 % via
  `FlightSession::set_aero_dispersion`) 21 of 21 within 0.01 m, <= 0.99 m/s, >= 5.7 t left; full mission: MECO T+141 s,
  entry burn T+356 s, landing burn T+423 s at 3.0 km / 287 m/s / 11.7 t, touchdown T+459 s, 0.00 m off (CRS-35: 137 /
  355 / 420 at ~3.2 km, ~300 m/s / 442 s). `return_flight_time_s` 260 s and `boostback_loss_fraction` 0.12 [D] were
  identified on that mission with the 395.7 t stage (kept). Known gap: our MECO leaves 55 t, CRS-35's gauge ~66 t (the
  real return spends ~11 t more).
- SCvx landing aborts (all vehicles; the pilot's `abort_landing` too): a first plan that finds the target unreachable aborts at once; `failures_before_abort`
  (2) re-plans with a virtual control above the stall threshold, or `unconverged_before_abort` (4) unconverged re-plans of
  any kind (a marginal first plan the vehicle could not follow), give the target up and plan for the sea.
- Falcon 9 drone ship (c4, `config/drone_ships.toml`, `config/missions/falcon9_asds.toml`): a drone ship is a landing zone
  with a deck (`sim::drone_ship_site`: deck 91 x 52 m [S], 3 m freeboard [E], stationary and level; `sea` marks the water
  beside the long edge for the abort); the mission's `[drone_ship]` places it at the nominal impact point of our ascent
  [D] (hidden test "... nominal impact point ...": drag + finite entry burn, 10 s after MECO). `sim::start_ascent_mission`
  sets a session up from an `AscentMission` (payload, pad, landing zone or drone ship, booster return, autopilot).
  `[stage.recovery.drone_ship]` overrides the RTLS profile when the landing zone has a deck (`StageConfig::return_profile_for`,
  `FlightSession::return_parameters`): `boostback = false` (the guidance starts coasting, turns engines first on its RCS;
  MECO plans run-down + entry burn + landing propellant only: 24.7 t), entry burn on centre + pair from 66 km for
  880 m/s [D] at 0.72, landing burn on the centre engine in all three phases [D] (`landing_phase_max_tilt_deg` 20).
  What the slanted approach (~90 m/s across at ignition) needed, all generic: the impact prediction flies the pending
  entry burn as a finite burn with its lean (`PendingBurn`; the impulse model was 3.5 km off); the entry burn solves its
  lean by shooting on that prediction (`entry_max_lean_rad` 5 deg); leg landings aim the ballistic impact beyond the pad by
  the landing burn's lead (`landing_burn_lead_itrf`: the retrograde burn bends the fall to the vertical and stops short);
  SCvx landing burns feather the grid fins into their local flow (`AttitudeController::feathered_grid_fins`: neutral fins
  acted as a tail the planning model does not know - Super Heavy too, catch campaign still 21/21); the return guidance
  estimates the drag online from the accelerometer (`FlightBody::set_accelerometer`, `drag_scale()`). Validation: full
  mission (kick 13 deg: MECO T+152 s, 64.8 km, 8373 km/h, flight path 24.7 deg vs Starlink 15-25 T+149 s, 60 km,
  ~8000 km/h, ~25 deg; 16.5 t to 200 x 281 km - 17 t arrives with 0.3 t left): entry burn T+379-399 s (T+364-386), landing
  burn T+476 s at 3.0 km / 271 m/s / 12.0 t (T+476 s, ~3.0 km, ~270 m/s, 11.9 t), landed T+515 s (T+499 s) 0.01 m off;
  campaigns `[falcon9][asds][validation]` from ignition (as RTLS) and from MECO states (+-200 m, +-5 m/s, +-5 %, aero
  +-10 %; two halves) 21 of 21 each; a deck out of reach ditches beside the ship. Known gaps: our MECO leaves 25 t, the
  gauge ~47 t - and the gauge itself falls from 47 t at MECO to 25.9 t at the entry burn while the speed stays ~8000 km/h
  (no burn between; unexplained); landing burns end at the SCvx reserve (5 t [E], Super Heavy's value) after ~36 s: the
  gauge shows 7.2 t at touchdown after a ~23 s burn of ~4.9 t, and a 7.2 t planner reserve is infeasible from the recorded
  ignition state in our model (our burn decelerates harder early - unmeasured subsonic and power-on drag [E]).
  Diagnostics: hidden test "Falcon 9 ASDS single MECO case" with `STARSIM_ASDS_SEED=<101-120>`.
- Game interface for Falcon 9 (c5; the drawing and UI are specified by the user first): `FlightSim.start_mission(id, tdb)`
  (ascent missions of `config/missions`), `mission_site_id`, `landing_zone()` (pad radius or drone-ship deck, sea height,
  site frame in ITRF), `body_legs(i)` (geometry, deployment, touchdown), `body_grid_fins(i)` (geometry, deflection);
  `return_state(i)["legs"]`. Covered by the smoke test. The bindings build with C++ exceptions on
  (`GODOTCPP_DISABLE_EXCEPTIONS OFF`): godot-cpp's default turned every core config error into an engine crash.
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
