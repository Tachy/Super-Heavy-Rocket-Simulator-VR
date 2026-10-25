# Architecture - modules and game structure

Moved verbatim from `CLAUDE.md` on 2026-10-25 (the per-session context is kept small). Rules stay in `CLAUDE.md`; per-subsystem detail in `docs/subsystems/`.

## Core modules (what is implemented)

- `core` is split into small modules with clear interfaces: `math`, `time`, `frames`, `ephemeris`,
  `environment`, `integrators`, `optim`, `orbit`, `vehicle`, `propulsion`, `aero`, `contact`, `gnc`, `mission`, `sim`.
  Implemented so far: math, time, frames, ephemeris, spice (adapter), environment (gravity, reference bodies Earth and
  Moon: body-fixed frame, shape, atmosphere or vacuum; thermal radiation environment: Sun, Earth and Moon albedo and infrared;
  terrain: the lunar site tile + LDEM_16, the Earth terrain (3DEP 1 m / GLO-30 / ETOPO 2022 + site plateaus) and the
  EGM2008 geoid as the sea level; one shared immutable instance per data set, `sim::shared_earth_terrain` /
  `shared_lunar_terrain`, also read by the renderer through `StarsimCore.surface_samples`; on the Earth terrain a site's
  leg landing tracks bodies within 5 km of the site; the sea state at a drone ship's station: long-crested JONSWAP sea,
  deterministic by seed; the Earth's atmosphere optics and precomputed scattering: a float64 port of Bruneton 2017 -
  transmittance / scattering / irradiance tables in luminance with 5 orders + geometric tail, the WGS84 stretch, CPU
  evaluation of sun transmittance, sky luminance and illuminance - and their cache `data/atmosphere/`, input-hashed;
  astronomical refraction from the US76 density: the lift table of Sun, Moon and stars for the camera),
  integrators (RK4, DOP853 + dense output, terminal events), orbit (Kepler, coast propagation, CR3BP, Clohessy-Wiltshire relative motion,
  CCSDS OEM parser + archive with segment-wise Lagrange interpolation, synthetic station orbit by J2 rates, energy- and J2-mean semi-major axis),
  mission (position targeting, 9:2 NRHO reference, lunar outbound and lunar orbit transfers, lunar descent planning,
  the mission delta-v budget and its required mass, the tanker's return reserve, the refuelling campaign calculator (tanker
  count from the r5 reference flights: windows with the phasing orbit chosen per window, depot boil-off, the depot's mean orbit followed: drag decay, transfer pushes, station keeping; HLS demand), the Earth return: TEI, mid-course corrections, braking burn, the coelliptic phasing (from below or above the target)
  to the ISS: schedule + full-model planner with drag, and the ISS launch window),
  gnc (among others: PEG steering module, the first stage's max-Q limiter and operational throttle, Apollo braking law, lunar landing guidance with SCvx approach and
  terminal descent, lunar ascent and abort guidance, entry corridor, lunar-return entry with an energy-linear bank profile and
  aerocapture guidance, relative navigation between docking ports, proximity guidance: waypoints, corridor,
  hold / retreat / breakout, final approach, R-bar approach from below (the depot), departure after undocking),
  eclipse geometry (disk occultation), optim (sparse SOCP interior-point solver),
  contact (catch tower: geometry, arms + carriage, pin/hull penalty contact, collisions; landing legs and surfaces;
  IDSS docking interface and mechanism: soft capture, joint, retraction, hooks, separation; probe/drogue interface
  (four probes into four drogues) on the same mechanism; hull and landing legs vs station primitives;
  drone-ship deck motion: box-barge heave / roll / pitch from the wave plane fit with strip-theory excitation, its
  significant values; the mean sea beside the moving deck),
  aero (slender body + cross-flow table, grid fins, ship flaps, Sutton-Graves entry heating plus radiative heating
  from the NASA TM-102652 equilibrium tables, blunt capsule from a
  modified-Newtonian table with offset-CG trim, reefed parachutes, fixed-area drag of a station),
  propulsion (engine performance, RCS allocator incl. pure translation capability + pulse modulator),
  vehicle (mass properties, Starship header tanks with a feed source, propellant settling with the apparent gravity,
  ideal LVLH attitude of a station, docking-port kinematics, docked hold, attached mass, compound mass, run-time tank
  masses, probe/drogue port mating of the refuelling ships, propellant boil-off by a hull heat balance in flight and
  orbit-averaged, ship-to-ship transfer physics: line flow, outlet coverage, settling time; per-stage settling progress
  before an ignition, PMD tanks, gas ingestion of an unsettled in-orbit ignition, the settling push and its gas bound), sim (flight session incl. the staging sequence (cold: MECO -> separation -> upper ignition; hot: the ship lights while joined) and the pre-launch ascent planner (`sim::plan_launch`: the first-stage kick by trial ascents for the least propellant, the recovery - tower, landing zone or drone ship - as a hard constraint, refusals; every launch from a pad, validated against flight 14 and CRS-35; a mission's sea state: the onboard refusal of a
  drone-ship landing in a rough sea, the deck turned to its least-tilt heading, legs landing on the moving deck and the
  landed stage riding it), the ISS as a station body from
  OEM or synthetic orbit, rendezvous target with relative port state and port-frame RCS translation, rendezvous phasing flight to
  Waypoint 0 with drag tracking and re-plan, proximity flight to a station port, docking into a compound, undocking
  and departure, ISS missions chained by a mission sequence: from the separation orbit, from the pad with a launch
  window, docked -> undocking -> splashdown; the depot as a station target with its approach limits, Starship phasing
  on the belly Raptor + cold gas, the docked compound following run-time tank masses; ship-to-ship propellant transfer
  with the settling push and the docked pair's boil-off, the tanker's return reserve; the autopilot's settling push
  before in-orbit ignitions (burns start turn + lead ahead; planners reserve the gas); the refuelling reference flights
  chained by the mission sequence: depot launch -> station, tanker pad -> perigee raise -> docking -> transfer ->
  departure -> ship return -> catch (the caught booster lifted off the tower), HLS refuel -> lunar mission continued
  from the live HLS); the refuelling campaign flown in one session (supervisor: live planning from the flown depot,
  station-keeping reboosts, comparison with the calculator); the crew escape of a crewed ship return (catch
  feasibility through the entry, give-up to the sea, ejection seats with their envelope and modes, each crew member a
  body to the touchdown, the empty ship to its impact; ship data untouched).
  The data pipeline archives every fetched ISS OEM in `data/iss_oem/` (one file per creation date).
  sim also holds the mission timeline (`sim::MissionTimeline`: recorded events, onboard-computer predictions, T-0, scheduled
  liftoff with pad hold via `FlightSession::schedule_liftoff` / `StartOptions::wait_on_pad`); vehicle telemetry has the
  inertial speed about the reference body and Earth / Moon distances and speeds.
  Modules depend downwards only (e.g. `ephemeris` may use `time` and `math`, never `gnc`).

## Game (Godot) structure
- `game/world/`: `world.gd` (per-frame orchestration), `celestial_body.gd` (floating origin + scaled space:
  bodies beyond 1.5e7 m are drawn closer and smaller with exact angular size), `orbit_camera.gd`
  (camera always at the render origin, true position in float64 GCRF), `frames.gd` (GCRF -> render axes:
  (x,y,z) -> (y,z,x), north = +Y), `star_field.gd`, `orbit_line.gd`, `sim_clock.gd`.
- `game/world/terrain/` (M8 Plan B): streamed terrain of the Earth and the Moon - `terrain_quadtree.gd` (cube-sphere
  quadtree: LOD, worker builds, upload / release budgets, morph ranges), `terrain_node_builder.gd` (node mesh data,
  worker-safe), `cube_sphere.gd`, `height_source.gd` (core surface or flat rule), `imagery_source.gd` (NAIP / Sentinel-2
  tile crops), `water_waves.gd`; shaders `terrain_earth/moon.gdshader`, `terrain_morph.gdshaderinc`,
  `water.gdshaderinc`. Performance run (GPU): `res://tests/terrain_perf.gd`. Details: `docs/subsystems.md` "Terrain
  rendering". A fresh checkout needs one `godot --headless --path game --import` before `--script` runs resolve new
  `class_name` scripts.
- `game/shaders/`: Earth/Moon lighting in body-local metres; `eclipse.gdshaderinc` mirrors
  `core/src/environment/eclipse.cpp` - change both together.
- Atmosphere (M8): `game/world/atmosphere.gd` (`Atmosphere`: loads the core's tables on a worker, uploads them,
  per-frame globals `atmosphere_*`, eye adaptation luminance, Sun light, eclipse segments and umbra light),
  `world/eye_adaptation.gd`, `world/lit_aerial.gd` (vehicle / tower / ship materials); shaders `atmosphere.gdshaderinc`
  (the GPU port of `core/src/environment/atmosphere_scattering.cpp` - change both together), `sky.gdshader`,
  `lit_aerial.gdshader`, `plume.gdshader`, `star_visibility.gdshaderinc`. GPU check (needs a GPU):
  `res://tests/atmosphere_check.gd [-- --f32]`; the table cache is built by `build/core/atmosphere_tables.exe`
  (~62 s) or on the first game start.
- Clouds (M8): `game/world/weather/cloud_layer.gd` (`CloudLayer`: field textures, cross-fade, presets),
  `cloud_effect.gd` (`CloudEffect`: the compute cloud pass in the world's Compositor), `cloud_field_texture.gd`,
  `cloud_shader_port.gd`; shaders `clouds.gdshaderinc` (port of the core's cloud optics / noise / texels - change both
  together), `cloud_pass.gdshaderinc` (compute kernels), `cloud_layer.gdshader` (composite). Details:
  `docs/subsystems/weather-clouds.md`.
- Global shader parameters are set only through `ShaderGlobals.set_value` (`world/shader_globals.gd`), which keeps a
  script-side copy: compute passes read the same values (RenderingServer's getter is editor-only).
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
- Mission selection: the time panel's Ort / Fahrzeug / Mission boxes come from `game/ui/mission_catalogue.gd` (pads of
  `launch_sites.toml` + `config/start_places.toml`, missions by `[mission] start_place` / `start_vehicle`).
- HOTAS: Thrustmaster T.16000M + TWCS, found by device name (`config/input_hotas.toml`). Raw axes (twist,
  throttle lever) only exist when `config/sdl_gamecontrollerdb.txt` is loaded BEFORE engine start via
  `SDL_GAMECONTROLLERCONFIG_FILE` (done by `Start-Desktop.cmd`); Godot's built-in mappings drop them and
  `Input.add_joy_mapping` at runtime has no effect. Probe axes: `res://tests/joy_probe.gd`.
- Start the game with `Start-Desktop.cmd` (repo root); `Start-VR.cmd` starts the OpenXR mode (`openxr/enabled=false` in
  project.godot: headless runs never touch the runtime).
- External view (plan 2026-10-22): `game/xr/` (`xr_manager.gd`: start modes, F12 recentre + automatic 2 s after start;
  `vr_overlay_panel.gd`: curved panel, 2 m / 64 deg, a scene mesh (mirrored on the desktop window) whose shader
  `shaders/vr_panel.gdshader` pins its clip depth so the distance-scaled near plane never clips it; it carries the
  webcast overlay and, in VR, the TimePanel top left; `vr_panel_mouse.gd`: captured mouse drives a pointer on it like the
  sibling's track editor), `game/ui/webcast/` (webcast overlay: timeline arc, round gauges
  chosen by `gauge_rules.gd` per situation, engine gauge, UTC + T-/T+ clock; the old HUD stays behind F1). Camera:
  `world/orbit_camera.gd` + `camera_math.gd` (right mouse button orbits around the local vertical, wheel zoom is
  logarithmic), `view_target.gd` + Tab / Shift+Tab switch the followed body. Spec `docs/superpowers/archive/specs/2026-10-22-external-view-design.md`.
- VR (M7): reuse the VR/desktop start-mode pattern of the sibling project `../Stunt-Track-Racer-VR`
  (`scripts/autoload/xr_manager.gd`, `Start-VR.cmd` / `Start-Desktop.cmd`); details in the plan, M7.

