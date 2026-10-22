# External view: desktop and VR (design)

Date: 2026-10-22. Status: draft for the user's review.

## Intent

The first piece of the graphics part is the **external view**: the camera looks at one simulated object from
outside, and a transparent overlay in the style of the SpaceX launch webcast shows that object's values and the
mission timeline. The same view runs in two start modes, desktop and PC-VR. Later every simulated object can be
viewed this way; the current draft (`OrbitCamera` around the controlled vehicle, text HUD) is the starting point.

Why now: the external view is what the player sees most of the time and it is the frame every later view (cockpit,
planner) builds on. Getting the VR/desktop split, the overlay and the camera right here comes before the cockpit.

## Goals

1. **Two start modes, no switching at run time** (already decided, master plan M7): `Start-Desktop.cmd` and
   `Start-VR.cmd`, pattern from `../Stunt-Track-Racer-VR/scripts/autoload/xr_manager.gd`. VR without a headset ->
   message and exit, no silent fallback to desktop. Test scripts (`--script`, headless) always run as desktop.
2. **Desktop:** full 16:9 view of the object, the overlay drawn transparently on top.
3. **VR:** the camera pose from the mouse/orbit logic is the *base pose*; the headset moves in 6DOF on top of it.
   The overlay is drawn on a **curved panel (section of a sphere centred on the eye), 2 m in front of the eye, in
   the camera's view direction**. It is fixed to the base pose and does **not** turn with the head: the player
   turns the head to look at the corner gauges. **F12 is the usual VR recentre:** it resets only the headset's
   view direction (yaw + position relative to the base pose, as in the sibling project) so that the overlay is in
   front again; it does not touch the camera control (orbit angles, distance, viewed object).
4. **Overlay content, SpaceX webcast look** (thin, light, transparent; no opaque panels):
   - **Top:** UTC and the mission clock: **T- before liftoff, then only T+ for the whole mission** (no second
     countdown to the next event; the arc shows that). T- requires a scheduled liftoff in the core (see My calls).
   - **Bottom left and bottom right:** round gauge elements for the **viewed object only**. Switching the camera to
     another object switches all gauges.
   - **Bottom centre:** the **timeline arc** with the mission events (Liftoff, Max Q, MECO, stage separation,
     boostback, entry, catch, burns, docking, ...) and a marker for "now". The arc shows the **whole mission with
     all participating objects** (booster and ship after separation, tankers and depot, ...); it does **not**
     change when the viewed object changes. Each event knows its object, so the arc can label or tint it.
5. **Gauges are chosen by relevance, live:** a gauge is shown only while its value is simulated and meaningful
   for the viewed object, and disappears otherwise. Examples: heat flux only during entry heating; Mach only in
   continuum flow (existing Knudsen rule in `flight_hud.gd`); engine layout only for an object with engines; crew
   g-load only with crew aboard. Candidate set (fuller than the webcast, but kept clear):
   - engine layout with each engine lit/unlit, **remaining propellant drawn graphically as a ring around it**
     (LOX and CH4 separately where the vehicle has them)
   - g-load (sensed acceleration)
   - speed inertial and speed relative to the surface
   - altitude (above the surface of the reference body: Earth or Moon)
   - further values in the same style (e.g. vertical speed, dynamic pressure, heat flux / heat load, apoapsis /
     periapsis in orbit, distance and closing rate to a docking target)
6. **Mouse control (this is what we start with), valid from close range out to Earth and lunar orbits:**
   - the camera always looks at the object's **centre of gravity**, never rolled (the reference body's local
     vertical is "up");
   - **right mouse button + up/down:** pitch, pivoting about the object;
   - **right mouse button + left/right:** rotate about the local vertical through the object;
   - **mouse wheel:** move toward / away from the object, logarithmic in distance (as today, ×1.15 per notch); the
     FOV stays fixed;
   - the left mouse button stays free (later: picking).
7. **Switching the viewed object:** among the simulated vehicle bodies (booster, ship, capsule, ...); the gauges
   follow.

## Non-goals (for this design)

- Cockpit view and its Dragon-style monitors. The **current text HUD, NavBall, key help and detailed readouts move
  to the cockpit monitors later**; that is discussed separately when the cockpit is built.
- Keyboard camera control, VR controller and HOTAS camera control: later.
- On-board cameras, ground observer as a separate mode beyond what exists, free 6DOF camera: later (ansichten.md §5).
- Viewing non-vehicle objects (ISS, depot, crew bodies after ejection): later, but the object list must be able to
  take them.

## Anti-goals

- Arcade look or invented values: every number on the overlay comes from the simulation (the game only reads
  state; physics never runs in GDScript). An event time on the arc is either a recorded event or a time the
  on-board planner/autopilot actually predicts.
- A cluttered overlay: if it does not stay clear, show fewer gauges.
- A VR panel that follows the head, or any VR camera motion the player did not cause (comfort).
- Breaking the existing flight/regression scripts (`flight_check.gd`, `smoke_test.gd`, `render_check.gd`).

## Constraints

- Architecture rules of CLAUDE.md: physics and event detection in `core`, thin bindings, game reads snapshots.
  The timeline needs a core-side event model (see below), exposed through `FlightSim` like the other states.
- Floating origin at the player's head; positions differenced in float64 before float32. In VR the base pose is
  the render origin and the headset offset is small (metres), so precision is unaffected.
- VR performance target from M7: stable 90 fps on the target headset (Pimax, RTX 4090, `d3d12`).
- Free assets only; a font for the webcast look must be free (e.g. OFL) and recorded in
  `tools/data_pipeline/sources.toml` / `CREDITS.md`.
- UI text German, code English. `docs/ansichten.md` is updated with this view (German, living document).

## Decided by the user

- External view first; VR and desktop as separate start modes; VR = HUD on a curved panel 2 m ahead in the camera
  view direction, not head-locked, 6DOF on top of the camera pose; F12 resets the VR view direction.
- Webcast look: round gauges bottom left/right, timeline arc bottom centre, UTC (+ countdown) on top.
- Gauges belong to the viewed object only and switch with it; only currently meaningful gauges are shown.
- The timeline arc scales sensibly, also over missions lasting days.
- Mouse control as in Goal 6; keyboard/VR controller/HOTAS later.
- Old HUD elements go to the cockpit later.
- F12 = usual VR recentre only (Goal 3).
- Timeline arc = whole mission, all objects; clock = T- before T-0, then T+ only, for the whole mission.
- T-0 = liftoff for launches; the first action (first burn ignition, or undocking) for missions starting in orbit.

## My calls (marked so the user can check them)

- **Timeline scaling:** the arc shows a window from shortly before the last event to shortly after the next two
  or three events; its time scale adapts (seconds at launch, hours/days on a lunar transfer) and eases smoothly
  when the next event approaches, so the dots stay readable.
- **Event source:** a core event log per session: past events are recorded when they happen (liftoff, max Q as the
  dynamic-pressure peak, MECO, stage separation, ignitions/cut-offs, entry interface, catch/touchdown, docking,
  ...); future events are the times the active planners and autopilots already predict (`burn_in_s`,
  `entry_in_s`, `time_to_ignition_s`, mission-sequence steps). A predicted event moves when the plan changes; no
  nominal-profile guesses beyond what a planner computes. The log is **mission-wide**: it spans all bodies and all
  steps of a chained mission sequence (e.g. the flown refuelling campaign over 52 days).
- **T-0 (decided by the user):** for a mission that starts with a launch, T-0 is **liftoff**; for a mission that
  starts in orbit, T-0 is the **first action**: the ignition of the first burn, or the undocking time for a
  mission that starts docked. Before T-0 the clock counts T-, afterwards T+ for the rest of the mission. Whenever
  the mission waits for a window before its first action (an ISS or depot launch window, a departure window in
  orbit), T- runs over the whole wait, however long it is.
- **Scheduled liftoff (new core work):** a launch gets a scheduled T-0 epoch instead of lifting off immediately.
  The pad countdown starts at a configurable lead time (mission/launch config, short default so tests stay fast);
  the autopilot's ignition sequence runs relative to T-0 (engine start before T-0, clamp release at T-0). Launch
  windows (ISS) set T-0 from the window. Time warp may run through the countdown. Missions starting in orbit
  need no new scheduling: their first burn or undocking is already planned; its epoch becomes T-0.
- **Other UI:** the `TimePanel` (UTC input, presets, Earth/Moon focus, time warp) stays on the desktop as today and
  is not part of the webcast overlay; the external view is the vehicle focus mode, the Earth/Moon focus modes
  remain. In VR the panel is not shown for now; its keys (Space, +/-) work. A VR version comes with the controllers.
- **Camera frame:** horizontal rotation is measured against local north in the reference body's horizon frame (as
  `OrbitCamera._orbit_frame` does today). On object switch, angles and distance are kept, the distance clamped to
  the new object's size. When the reference body changes (Earth <-> Moon), "up" is blended over a few seconds,
  never jumped.
- **Mouse in VR:** the mouse stays the camera control while wearing the headset until the VR controllers come.
- **Old HUD until the cockpit exists:** the text HUD, NavBall and key help stay available via a key toggle (off by
  default in the external view), so flying and debugging still work.
- **Object switching key:** Tab / Shift+Tab cycles through the vehicle bodies (WASD/QE are taken by flight control).
- **VR overlay rendering:** the overlay is one 2D control tree rendered into a transparent `SubViewport`; desktop
  shows it full-screen, VR maps the same texture onto the sphere-section mesh attached to the base pose (unshaded,
  drawn on top). One layout for both modes.
- **Gauge appearance details** (font, line widths, colours, exact ring layout of the engines, which values pair in
  which round element): left to the builder, reviewed with the user on render-check screenshots.

## Left to the builder

- Module/file split in `game/` (e.g. `game/xr/`, `game/ui/external_hud/`), gauge widget implementation, mesh
  resolution of the curved panel, angular size of the panel (it must fit the headset's field of view comfortably).
- Exact near/far handling over the zoom range (close-up metres to lunar distance), reusing the existing scaled
  space.
- How the CG is obtained per body (`body_cg_station_m` exists) and how the "local vertical" switches between Earth
  and Moon.
- Tests: core event log with unit tests; gauge-relevance rules testable headless in GDScript; render check PNGs
  for the desktop overlay; a manual VR session for the VR part (master plan: manual VR session per milestone from M7).
