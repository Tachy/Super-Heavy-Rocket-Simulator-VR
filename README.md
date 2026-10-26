# Super Heavy: Rocket Simulator VR

**English** · [Deutsch](README.de.md)

**Super Heavy: Rocket Simulator VR** is a physically accurate simulator for a two-stage super-heavy launch system (booster + ship) and its companions:
a PC-VR flight simulator (OpenXR, HOTAS, seated cockpit - in progress) and a mission/engineering simulation
(planning, guidance, telemetry, validation). Not an arcade game: physical correctness comes first, and every model
is checked against reference data or a dispersion campaign with a documented tolerance (~1,400 automated test cases
in the simulation core, plus GPU and game checks).

An independent hobby project. It is not affiliated with, endorsed by or connected to SpaceX; vehicle names are
used only to identify the real-world systems the configurations approximate.

## How it was built

This simulator was created entirely in dialogue with Claude Code, by agentic engineering: the author describes goals,
makes the decisions and checks the results - in the headset, on images and against measured data - while Claude Code
plans, writes, tests, reviews and documents. Not a single line of code was written or edited by hand. Physics core,
numerics, rendering, VR integration, tests, data pipeline and release packaging all came out of that conversation.

## Highlights

**From the pad to the Moon and back, flown by the onboard computer**
- Ascent with optimised pitch kick (trial flights at every launch), staging (hot staging for Starship), PEG guidance
  to orbit; the booster is always caught at the tower.
- Orbital refuelling: tanker flights to a propellant depot with rendezvous, phasing and propellant transfer; the
  planner answers how many tankers a lunar mission needs (13 tankers / 52.5 days for the HLS) and flies the whole
  campaign.
- A lunar lander (HLS) fuelled in Earth orbit flies translunar injection, mid-course corrections and insertion into a
  100 km polar lunar orbit or the 9:2 near-rectilinear halo orbit (NRHO, the Gateway orbit), rebuilt by multiple
  shooting in the full Earth-Moon-Sun force model; the NRHO is reached through a powered lunar swing-by.
- Powered descent to the lunar south pole on real LOLA terrain: Apollo-style braking (validated against Apollo 11:
  508.5 s vs 507 s flown), approach by convex optimisation (SCvx), terminal descent on twelve canted landing
  thrusters; 21 of 21 dispersed descents land within 0.8 m. Ascent back to orbit with engine-out handling and aborts.
- Return to Earth: trans-Earth injection, a braking burn before entry, then a direct entry to the launch site with
  flip and tower catch, or an aerocapture into Earth orbit followed by a normal deorbit and catch.
- Honest budgets: missions that do not close are refused with the propellant shortfall rather than tuned to fit.

**Booster and ship come home**
- Booster: flip, drag-aware boostback, grid-fin descent, landing burn 13 -> 5 -> 3 engines on SCvx plans re-solved in
  flight, caught by the tower arms (21 of 21, engine failure included).
- Ship: deorbit for the next pass within reach, belly-first entry with predictor-corrector guidance and heat / load /
  dynamic-pressure limits, flap control, flip and a 3 -> 2 engine catch (14 of 14 orbit returns caught).
- Crewed ship returns carry ejection seats: on any abort the onboard computer fires them, early and over water where
  possible; every crew member flies as its own body under a parachute.
- Falcon 9: return to the landing zone and drone-ship landings (21 of 21 each), drone ship moving in a real sea state
  (strip theory, deck motion limits); timelines match the CRS-35 and Starlink 15-25 webcasts within seconds.
- Crew Dragon: individual Draco thrusters, lifting entry, drogues and mains, splashdown in the recovery zone.

**Docking at the ISS**
- Crew Dragon rendezvous and docking with the ISS (real NASA ephemeris where available): phasing over days, approach
  corridors to the IDA-2 / IDA-3 ports, contact model with soft and hard capture; undocking and return to splashdown.

**Physics models**
- 6-DOF variable-mass rigid bodies: per-engine thrust and moments, transients, throttle and gimbal limits, relights,
  moving centre of gravity, stage separation, landing legs, tower arms, splashdown, RCS with individual thrusters.
- US Standard Atmosphere 1976, pressure-dependent thrust/Isp, Mach-dependent aerodynamics bridging to
  free-molecular flow, grid fins, flaps, capsules, solar radiation pressure with Earth/Moon shadow.
- Entry heating: Sutton-Graves convective plus radiative heating from NASA's equilibrium tables; heat-shield failure
  and break-up as outcomes.
- Gravity: Earth, Moon and Sun point masses plus Earth J2; DOP853 with dense output for coasts, 200 Hz in powered
  flight; the simulation runs on its own thread.
- Vehicle data of Starship V3 identified from our own transcription of the flight 12-14 webcasts; where a gauge
  contradicts the physics, the physics wins.

**Own numerics, no black boxes**
- Sparse second-order-cone (SOCP) interior-point solver checked against Clarabel; SCvx successive convexification for
  all powered landings; PEG, Lambert and full-force-model targeting, circular restricted three-body problem, entry
  corridors.

**The real Earth and sky at the true instant**
- Sun and Moon from JPL DE440, Earth orientation from the NAIF ITRF93 PCK, UTC -> TDB with leap seconds - validated
  against JPL Horizons (Sun < 4 cm, launch sites < 8 cm); apparent positions (light time + aberration); the
  2024-04-08 total eclipse appears over Texas with contact times within 2 s.
- Terrain: Copernicus elevation and Sentinel-2 imagery around the launch and landing sites (EGM2008 geoid), the whole
  Moon from LOLA, a seamless transition from the pad to orbit; floating origin with float64 positions end to end
  (Godot 4.7 double-precision build).
- Atmosphere: precomputed multiple scattering (Bruneton-style tables), refraction from the density profile, eye
  adaptation with night vision "like the eye", eclipse shadow in the sky.
- Weather and clouds: real weather for the chosen instant - ERA5 reanalysis for the past, GFS forecasts for the next
  16 days, typical weather beyond - as volumetric clouds with physically based light (Monte Carlo multiple
  scattering), cloud shadows on the ground and clouds seen from orbit. A rework to static, crisper clouds with cached
  light is in progress.

**In the game**
- Mission selection by place, vehicle and mission; real missions start at their historical liftoff time.
- Webcast-style overlay (mission clock, timeline arc, gauges), external views (vehicle, tower, drone ship, ground
  sites), HUD with NavBall.
- PC-VR (OpenXR, tested on Pimax): the overlay as a panel, one shared cloud image for both eyes, an opt-in performance
  log (`Start-VR.cmd --perf-log`) and a frame-rate readout.
- Autopilots fly every mission step; manual flying with the Thrustmaster T.16000M stick + TWCS throttle comes with the
  cockpit (M7).

**Data-driven and free**
- Vehicles, engines, aero tables, launch and landing sites and missions are TOML/CSV configuration with provenance
  tags ([S] sourced, [E] estimate, [D] derived); `tools/vehicle_report.py` prints mass and delta-v budgets.
- Only free data and tools; every source and its licence is recorded and `CREDITS.md` is generated.

## Vehicles

| Vehicle | What it is |
|---|---|
| Starship V3 (Super Heavy Block 3 / Ship Block 3) | the two-stage launch system: 33-engine booster, ship with six Raptors, flaps, heat shield |
| Starship Tanker | ship variant carrying propellant to the depot |
| Starship Depot | the propellant depot in Earth orbit |
| Starship HLS (+ its launch stack) | lunar lander: no heat shield or flaps, legs, twelve canted landing thrusters |
| Falcon 9 Block 5 | with return to the landing zone or a drone-ship landing |
| Crew Dragon | crew capsule with Draco RCS, SuperDracos, parachutes |
| ISS | the space station as a docking target |
| Ejection seat, parachutist | crew escape from a crewed ship return |

## Missions

| Group | Missions |
|---|---|
| Starship | to low Earth orbit with booster catch; test flight 14 (validation); return to Starbase (also crewed) |
| Refuelling | depot launch; tanker to the phasing orbit; tanker flight to the depot; the flown refuelling campaign |
| Moon | HLS to the phasing orbit; HLS refuelling at the depot and lunar mission; HLS landing via LLO, via LLO from the depot, via NRHO; return from the Moon (LLO direct, also crewed; LLO with aerocapture; NRHO direct) |
| Falcon 9 | landing at LZ-40; drone-ship landing (also in a sea state); CRS-35 (validation); with Crew Dragon |
| Crew Dragon / ISS | Falcon 9 with Crew Dragon to the ISS; Crew Dragon to the ISS from orbit; undocking and return; Crew Dragon return |

## Status and roadmap

Done: sky, time and ephemerides; flight physics and ascent; booster and ship return with tower catch; Falcon 9 landings
and sea state; Crew Dragon; lunar missions in the core and the game; ISS docking; orbital refuelling; crew escape;
terrain and atmosphere rendering (M8); real weather with volumetric clouds; mission selection; release packaging; VR
performance work. In progress: static clouds (cached light, crisp outlines, cloud types, cloud bases at the
condensation level). Next: rain, fog and lightning, wind, and the VR cockpit with manual flying and the in-game mission
planner (M7). Views and controls of the finished game: [docs/ansichten.md](docs/ansichten.md) (German); model
decisions and validation numbers: [docs/subsystems.md](docs/subsystems.md).

## Architecture

```
core/      C++20 simulation core, no Godot dependency, float64 throughout
bindings/  thin GDExtension adapter (godot-cpp, precision=double)
game/      Godot 4 project: rendering, UI, VR, input - physics never runs in GDScript
config/    vehicles, engines, aero tables, launch and lunar sites, missions (see config/README.md)
tools/     data pipeline, validation fixtures, telemetry transcription, reports, packaging
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
the Godot double-precision editor and godot-cpp) is described in [docs/setup.md](docs/setup.md).

```powershell
python tools/data_pipeline/fetch.py
python tools/data_pipeline/prepare.py
powershell -ExecutionPolicy Bypass -File tools/build.ps1                         # core + GDExtension + all tests
powershell -ExecutionPolicy Bypass -File tools/build.ps1 -TestFilter "[weather]" # only the tests of a tag
Start-Desktop.cmd                                                                  # run the game (Start-VR.cmd for VR)
powershell -ExecutionPolicy Bypass -File tools/package.ps1                        # release build: installer in build/
```

## Credits

Data sources and their licences: [CREDITS.md](CREDITS.md).
