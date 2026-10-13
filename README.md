# Super Heavy Rocket Simulator VR

**English** · [Deutsch](README.de.md)

A physically accurate simulator for a two-stage super-heavy launch system (booster + ship): a PC-VR flight
simulator (OpenXR, HOTAS, seated cockpit - in progress) and a mission/engineering simulation (planning,
guidance, telemetry, validation). Not an arcade game: physical correctness comes first, and every model is
checked against reference data with a documented tolerance.

An independent hobby project. It is not affiliated with, endorsed by or connected to SpaceX; vehicle names are
used only to identify the real-world systems the configurations approximate.

## Highlights

**The real sky at the true instant**
- Sun and Moon from JPL's DE440 ephemeris, Earth orientation from the NAIF ITRF93 binary PCK, time
  scales UTC -> TAI -> TT -> TDB with leap seconds - validated against JPL Horizons (Sun < 4 cm,
  launch sites < 8 cm).
- Apparent positions (light time + aberration) for rendering: the 2024-04-08 total eclipse appears over
  Texas with contact times within 2 s, the shadow drawn in the shaders.
- Floating origin with float64 positions end to end (Godot 4.7.2 double-precision build) and scaled space
  for far bodies with exact angular size; Blue Marble / Black Marble Earth, LROC Moon, HYG star field.

**Flight physics**
- 6-DOF variable-mass rigid bodies: per-engine thrust and moments, start/stop transients, throttle and gimbal
  rate limits, relight rules, flame-out on depletion, moving centre of gravity and inertia, stage separation
  and hot staging.
- US Standard Atmosphere 1976 (<= 0.2 % against the tables), pressure-dependent engine thrust/Isp with
  flow-separation limits, Mach-dependent slender-body aerodynamics bridging to free-molecular flow by
  Knudsen number, solar radiation pressure with Earth/Moon shadow.
- Gravity: Earth, Moon and Sun point masses plus Earth J2 (every term switchable); DOP853 integrator with
  dense output and event location for coast arcs, 200 Hz RK4 ticks in powered flight.
- Vehicle data of the current Starship (V3) identified from our own transcription of the flight 12-14 webcasts
  (speed, altitude, propellant gauges, engine icons): masses, thrust, booster descent drag (within ~1-3 %).

**Guidance, navigation and control**
- Ascent autopilot: launch azimuth with Earth-rotation correction, pitch kick, gravity turn, Powered Explicit
  Guidance (PEG) on the upper stage - 200 km target orbit reached to about 1 km, also with three booster
  engines failed at T+60 s.
- Manoeuvre nodes with closed-loop burn execution; automatic choice of engine set, RCS translation for
  burns below the main-engine minimum impulse (0.5 m/s executed to ~1 mm/s).
- Lambert solver, translunar injection targeting in the full force model, mid-course corrections and lunar
  orbit insertion: a planned Earth-Moon mission arrives in a 100 km polar lunar orbit.
- Booster return to the launch site: flip, Lambert boostback with drag-aware aim-point shooting,
  atmospheric descent steered by grid fins (angle-of-attack control, transonic choking modelled), landing
  burn 13 -> 3 engines - all three booster generations land on the pad.

**Flying it yourself**
- Thrustmaster T.16000M stick + TWCS throttle (raw axes via an SDL mapping), keyboard fallback.
- HUD with NavBall (prograde/retrograde and target markers), flight-director crosshair, planned (green) and
  predicted (orange) trajectories, delta-v bars for burns.

**Data-driven and free**
- Vehicles (current Starship V3, Falcon 9 for comparison), engines, aero tables, launch sites (Starbase,
  LC-39A) and missions are TOML/CSV configuration with provenance tags ([S] sourced, [E] estimate,
  [D] derived); `tools/vehicle_report.py` prints mass and delta-v budgets.
- Only free data and tools; every source and its licence is recorded and `CREDITS.md` is generated.

## Status and roadmap

Milestones M0-M5 are done; M6 (mission planning, booster return) is in progress. Next: convex powered-descent
guidance for a precise tower catch (own SOCP solver, catch tower and arms), the in-game mission planner, the VR
cockpit (M7), high-resolution terrain, atmosphere and effects (M8), rendezvous, docking and orbital refuelling
(M9). Views and controls of the finished game: [docs/ansichten.md](docs/ansichten.md) (German).

## Architecture

```
core/      C++20 simulation core, no Godot dependency, float64 throughout
bindings/  thin GDExtension adapter (godot-cpp, precision=double)
game/      Godot 4 project: rendering, UI, VR, input - physics never runs in GDScript
config/    vehicles, engines, aero tables, launch sites, missions
tools/     data pipeline, validation fixtures, reports
tests/     validation fixtures (JPL Horizons, telemetry, reference solvers)
```

## Building (Windows)

Requirements: MSVC 2022 Build Tools, CMake, Ninja, SCons, Python 3. The one-time setup (runtime data, CSPICE,
the Godot double-precision editor and godot-cpp) is described in [CLAUDE.md](CLAUDE.md) (section "Build & test").

```powershell
python tools/data_pipeline/fetch.py
python tools/data_pipeline/prepare.py
powershell -ExecutionPolicy Bypass -File tools/build.ps1   # core + GDExtension + tests
Start-Desktop.cmd                                            # run the game
```

## Credits

Data sources and their licences: [CREDITS.md](CREDITS.md).
