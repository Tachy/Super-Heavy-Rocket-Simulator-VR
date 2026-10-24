# Super Heavy Rocket Simulator VR

**English** · [Deutsch](README.de.md)

A physically accurate simulator for a two-stage super-heavy launch system (booster + ship) and its companions:
a PC-VR flight simulator (OpenXR, HOTAS, seated cockpit - in progress) and a mission/engineering simulation
(planning, guidance, telemetry, validation). Not an arcade game: physical correctness comes first, and every model
is checked against reference data or a dispersion campaign with a documented tolerance (~600 automated test cases).

An independent hobby project. It is not affiliated with, endorsed by or connected to SpaceX; vehicle names are
used only to identify the real-world systems the configurations approximate.

## Highlights

**Earth to Moon and back, flown by the onboard computer**
- A lunar lander (HLS variant) fuelled in a 200 km Earth orbit flies translunar injection, mid-course corrections and
  insertion into a 100 km polar lunar orbit or the 9:2 near-rectilinear halo orbit (NRHO, the Gateway orbit),
  rebuilt by multiple shooting in the full Earth-Moon-Sun force model. The NRHO is reached through a powered lunar
  swing-by (3.2 km/s TLI + 0.24 + 0.22 km/s): a direct transfer would meet the orbit head-on at 3.5 km/s.
- Powered descent to the lunar south pole on real LOLA terrain (20 m tiles around the site): braking on an
  Apollo-style guidance law (validated against Apollo 11: 508.5 s vs 507 s flown), approach planned by convex
  optimisation (SCvx) and an Apollo P66-like terminal descent on twelve canted landing thrusters steered by
  differential throttling. Campaign: 21 of 21 dispersed descents land within 0.8 m at ~1 m/s on slopes up to ~9 deg.
- Ascent back to orbit on PEG with engine-out handling; aborts from 3 km, 1 km and 0.3 km reach orbit.
- Return to Earth: trans-Earth injection, corrections, a braking burn before the entry interface (the ship's
  heat shield cannot take a full 11 km/s lunar-return entry - the simulation shows why), then either a direct entry
  to the launch site with flip and tower catch, or an **aerocapture** into Earth orbit followed by a normal deorbit and
  catch. Entry guidance with an energy-linear bank profile (FNPEG principle) and skip handling.
- Honest budgets: missions that do not close (e.g. a heavier lander, the NRHO variant at 120 t dry) are refused with
  the propellant shortfall rather than tuned until they fit.

**Booster and ship come home**
- Booster return to the tower: flip, boostback aimed by drag-aware shooting, grid-fin descent (transonic choking
  modelled), landing burn 13 -> 5 -> 3 engines on SCvx plans re-solved in flight on a worker thread, caught by the
  tower arms (contact model of arms, carriage and pins). Campaign: 21 of 21 caught (< 0.1 m, ~1 m/s sink), engine
  failure included.
- Ship return from orbit: deorbit planned on the onboard computer for the next pass within reach, belly-first entry
  with a predictor-corrector guidance (heat rate, load and dynamic-pressure limits, bank reversals), flap control,
  flip on three Raptors at 800 m and a 3 -> 2 engine catch. Campaigns: 14 of 14 orbit returns caught; 20 of 21
  dispersed flips caught (the one with a failed engine aborts safely to the sea).
- Falcon 9 for comparison: return to the landing zone (21 of 21 within 0.01 m) and drone-ship landings (21 of 21),
  with timelines matching the CRS-35 and Starlink 15-25 webcasts within seconds.
- Crew Dragon: individual Draco thrusters (RCS allocator + pulse modulation), offset-CG lifting entry from a
  modified-Newtonian aero table, reefed drogues and mains, splashdown in the recovery zone (8 of 8 within 1.1 km at
  ~7 m/s; deorbit to splashdown 50.5 min vs 54 min for Crew-10).

**Physics models**
- 6-DOF variable-mass rigid bodies: per-engine thrust and moments, start/stop transients, throttle and gimbal rate
  limits, relight rules, flame-out on depletion, moving centre of gravity and inertia, stage separation, hot staging,
  landing legs with crush and friction, splashdown.
- US Standard Atmosphere 1976 (<= 0.2 % against the tables), pressure-dependent thrust/Isp with flow-separation
  limits, Mach-dependent slender-body aerodynamics bridging to free-molecular flow, grid fins, ship flaps, blunt
  capsules, solar radiation pressure with Earth/Moon shadow.
- Entry heating: Sutton-Graves convective plus **radiative heating** from NASA's equilibrium tables (TM-102652,
  624 transcribed points), heat-shield failure and break-up as outcomes.
- Gravity: Earth, Moon and Sun point masses plus Earth J2 (each switchable); states always in the inertial frame,
  with Earth or Moon as reference body. DOP853 with dense output and event location for coasts, 200 Hz in powered
  flight; days-long coasts in large steps.
- Vehicle data of the current Starship (V3) identified from our own transcription of the flight 12-14 webcasts
  (speed, altitude, propellant gauges, engine icons). Where a gauge contradicts the physics, the physics wins.

**Own numerics, no black boxes**
- Sparse second-order-cone (SOCP) interior-point solver, checked against Clarabel reference solutions; SCvx
  successive convexification on top of it for all powered landings (booster, ship, Falcon 9, lunar lander).
- Powered Explicit Guidance (PEG), Lambert and full-force-model targeting, circular restricted three-body problem,
  entry corridors and the highest admissible entry speed for a given heat shield.

**The real sky at the true instant**
- Sun and Moon from JPL's DE440 ephemeris, Earth orientation from the NAIF ITRF93 binary PCK, time scales
  UTC -> TAI -> TT -> TDB with leap seconds - validated against JPL Horizons (Sun < 4 cm, launch sites < 8 cm).
- Apparent positions (light time + aberration) for rendering: the 2024-04-08 total eclipse appears over Texas with
  contact times within 2 s, the shadow drawn in the shaders.
- Floating origin with float64 positions end to end (Godot 4.7.2 double-precision build) and scaled space for far
  bodies with exact angular size; Blue Marble / Black Marble Earth, LROC Moon, HYG star field.

**Flying it yourself**
- Thrustmaster T.16000M stick + TWCS throttle (raw axes via an SDL mapping), keyboard fallback.
- HUD with NavBall (prograde/retrograde and target markers), flight-director crosshair, planned (green) and
  predicted (orange) trajectories, delta-v bars for burns.
- Autopilots fly every mission step; the pilot can take over steering, or steering and thrust, with the onboard
  computer re-planning from the new state.
- Playable in the game today: Starship to orbit with booster catch and ship return, Falcon 9 (RTLS, drone ship,
  Crew Dragon launch) and the Crew Dragon return. The lunar missions run in the simulation core and its tests; their
  game interface is the next step.

**Data-driven and free**
- Vehicles (Starship V3 with the HLS variant, Falcon 9, Crew Dragon), engines, aero tables, launch and landing sites
  and missions are TOML/CSV configuration with provenance tags ([S] sourced, [E] estimate, [D] derived);
  `tools/vehicle_report.py` prints mass and delta-v budgets.
- Only free data and tools; every source and its licence is recorded and `CREDITS.md` is generated.

## Status and roadmap

Done: sky, time and ephemerides (M0-M4), flight physics and ascent guidance (M5), booster return and tower catch,
ship entry / flip / catch / return from orbit, Falcon 9 landings, Crew Dragon with RCS, entry and parachutes, and the
lunar missions l1-l5 (reference bodies, lander and terrain, outbound flight to LLO / NRHO, landing and ascent, return
with direct entry or aerocapture). Next: the lunar missions in the game (l6), the in-game mission planner, the VR
cockpit (M7), terrain and atmosphere rendering with Earth elevation data (M8), tankers and orbital refuelling (M9) and
docking with the ISS. Views and controls of the finished game: [docs/ansichten.md](docs/ansichten.md) (German); model
decisions and validation numbers: [docs/subsystems.md](docs/subsystems.md).

## Architecture

```
core/      C++20 simulation core, no Godot dependency, float64 throughout
bindings/  thin GDExtension adapter (godot-cpp, precision=double)
game/      Godot 4 project: rendering, UI, VR, input - physics never runs in GDScript
config/    vehicles, engines, aero tables, launch and lunar sites, missions (see config/README.md)
tools/     data pipeline, validation fixtures, telemetry transcription, reports
tests/     validation fixtures (JPL Horizons, webcast telemetry, reference solvers)
```

## Installation (Windows)

Each GitHub release carries:
- `SuperHeavyRocketSimulatorVR-<version>-setup.exe` - the installer with everything (per user, no admin rights). The
  Earth terrain and the aerial imagery of the launch sites are optional components; without them the ground near the
  pads is flat and untextured. Start menu: the simulator (VR, OpenXR runtime running) and its desktop mode.
- `...-windows.zip` - the same game without installer, playable with the base data (`Start-VR.cmd`,
  `Start-Desktop.cmd`); `...-data-terrain.zip` and `...-data-imagery.zip` unpack into the same folder.

## Building (Windows)

Requirements: MSVC 2022 Build Tools, CMake, Ninja, SCons, Python 3. The one-time setup (runtime data, CSPICE,
the Godot double-precision editor and godot-cpp) is described in [CLAUDE.md](CLAUDE.md) (section "Build & test").

```powershell
python tools/data_pipeline/fetch.py
python tools/data_pipeline/prepare.py
powershell -ExecutionPolicy Bypass -File tools/build.ps1         # core + GDExtension + all tests
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Fast   # without the long dispersion campaigns
Start-Desktop.cmd                                                  # run the game
powershell -ExecutionPolicy Bypass -File tools/package.ps1        # release build: exe, ZIPs, installer in build/
```

## Credits

Data sources and their licences: [CREDITS.md](CREDITS.md).
