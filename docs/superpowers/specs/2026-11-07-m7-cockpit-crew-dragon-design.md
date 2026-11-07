# M7 cockpit, step 1: Crew Dragon cabin (design)

State: brainstorm with the user 2026-11-07. This document covers the first cockpit (Crew Dragon), the order of the
following steps and what was already found for Starship and HLS. Step 1 is what gets built next.

## Intent

The player flies from the commander's seat. The cockpit becomes the main view in flight (`docs/ansichten.md` section 4);
the external 3D view of today stays and is one button away. The user wants the cabin **as real as possible, 1:1 to the
real Crew Dragon interior**: "Das muss möglichst 1:1 dem Innenraum entsprechen ... Das soll möglichst echt aussehen."

Order (user 2026-11-07, plan change during the brainstorm): **Crew Dragon first** because everything about it is known
(photos, the displays' designer, NASA imagery); the finished design is then ported to **Starship** and **HLS**, which get
their own cockpits (three cockpits in total). The Dragon fixes the shared parts: seat, suit, crew figure, display frame,
camera screen, mouse / VR cursor interaction, cockpit <-> external toggle.

## Goals (step 1)

1. **Cabin 1:1** to the real Crew Dragon pressure vessel interior, rebuilt by us from references (no third-party model or
   photo pixels ship; "rebuild everything real"). Every dimension carries [S] / [E] / [D] with its derivation, like the
   vehicle data (`config/README.md`). The cabin agrees with the exterior model (`config/vehicles/detail/crew_dragon.toml`:
   windows at az +-33.3 and +-90 deg, x ~2.38 m; hatch at az 0; nose / docking neck): windows are real openings with
   glass through which the rendered world is seen.
2. **Four seats in one row** as on the real Dragon. The **player is the commander, left of centre**. The **pilot** sits
   right of centre, **mission specialists** on the outer seats. How many seats are occupied comes from the mission
   (user 2026-11-07); the player is always in the commander's seat.
3. **Crew figures** in the **newer SpaceX EVA-suit look** (user 2026-11-07: "neuere Optik"; references Polaris Dawn EVA
   suit): white, slim, dark side stripes, **visor closed** (copper-tinted, no faces). Step 1: figures sit still in their
   seats (hands on armrests / thighs). The player's own body (suit, legs, harness) is visible when looking down.
4. **Display console as on the real Dragon**: three large touchscreens in a dark housing across the panel, two procedure
   tablets on mounts above the outer screens, two rows of physical buttons / covered switches below, a central handle,
   overhead lights, placards. Geometry, materials and lighting as in the references.
5. **Left screen = external cameras, always** (user 2026-11-07). A button on the screen cycles through the cameras;
   the physical buttons under the left screen are direct camera keys; a HOTAS key also cycles. **Centre and right
   screens** are the space for our own UI later; in step 1 they are switched on but empty (dark Dragon-style background).
6. **Toggle cockpit <-> external view** (today's 3D view with the webcast overlay, VR included) with **stick button 3**
   ("the left one, left of the thumb hat at the top" of the T.16000M stick; user 2026-11-07). The index is measured on
   the user's stick and written to `config/input_hotas.toml` (first button entry there). A keyboard key does the same on
   the desktop. The simulation keeps running; only the camera place changes. Choosing a Dragon mission starts the player
   **in the cockpit**.
7. **Mouse interaction until hand tracking exists** (user 2026-11-07): desktop with the normal mouse pointer; in VR a
   **virtual mouse pointer that runs exactly on the 3D surfaces of the screens and button rows** (it moves across the
   console surfaces, not through space). Left click presses the screen element / button under it.
   **The player's suited hand with extended index finger follows the pointer, and on a click the finger really touches
   the surface** (user 2026-11-07), arm driven by IK from the shoulder within the arm's reach. **Smoothed motion, no
   angular or jerky movement** (user: "geglättete Bewegungen, keine eckigen und ruckartigen"): the hand eases towards
   the pointer (e.g. critically damped), the tap is a short smooth press and release. My call: the hand of the side the
   pointer is on (left hand for the left screen, right hand otherwise); the other hand rests on its HOTAS grip pose. The
   same hand / arm rig is reused later by hand tracking.
8. **Desktop and VR**: same cabin; VR with free head motion (6DOF), F12 recentres to the commander's eye point; desktop
   looks around with the mouse (right button held, as in the external view).

## HOTAS in the cabin (deliberate deviation, user 2026-11-07)

The real Crew Dragon has no stick or throttle (manual flight by touchscreen arrows). Because the player holds a real
HOTAS in VR, the cabin gets a **stick and a throttle on the commander's armrests**, placed exactly where the real HOTAS
is (one calibration, shared with the hand poses), styled plainly in Dragon look. Marked as a deviation from the
original and switchable in the cockpit config. The suited hands hold them (grip pose) unless a hand is following the
mouse pointer. Starship and HLS get the same.

## Physical buttons, handle, tablets

User rule 2026-11-07: **buttons whose real function is known get that function; otherwise they are scenery.** For each
button / switch / the handle the builder records the evidence (source, [S] / [E]) in the cockpit config; unknown ones
are drawn but inert. Exception decided by the user: the buttons under the **left** screen become the camera direct
keys. In step 1 only the camera keys and the toggle work; the known functions of the other buttons (likely candidates
from reports: abort / escape handle, fire suppression, cabin pressure, trunk jettison, deorbit) are listed with evidence
and wired in a later step when the systems behind them exist in the game. Tablets: scenery in step 1 (my call), later
procedure lists.

## External cameras (left screen)

My proposal, accepted without changes by the user ("Kameraliste so, solange nichts ergänzt"), modelled on the SpaceX
webcast cameras:

1. **Chase camera**: today's external view with the webcast overlay, following the vehicle.
2. **First stage, looking aft**: on the Falcon 9 interstage along the stage to engines and plume; on the booster's
   return: grid fins and legs.
3. **Second stage engine**: MVac nozzle glowing with the Earth behind.
4. **Dragon nose (docking camera)**: forward under the open nose cone; the ISS approach camera.
5. **Dragon trunk**: looking aft over the trunk; second-stage separation.

Cameras are **configuration, not code** (per vehicle / stage: mount point in the body frame, direction, field of view,
name key for i18n). The list offered follows the situation: a camera whose body no longer exists is not offered; a
camera on a separated stage keeps showing that stage (as the booster view of `ansichten.md` does today).

## Constraints

- Architecture (`CLAUDE.md`): no physics in GDScript; the cockpit only reads simulation state and sends commands.
  Vehicles, cockpits and cameras are configuration (TOML), texts from `config/i18n/<de|en>.toml`.
- Model pipeline as for the exterior models (`docs/subsystems/vehicle-models.md`): typed tables in config, Blender
  generators under `tools/structures/blender/`, `build_models.py`, git-ignored output in `data/models/<id>/` with
  manifest; game falls back with a notice when the model is missing. Surface detail by texture / shader, geometry only
  for the silhouette (user rule).
- Rendering: floating origin at the player's head; positions differenced in double, then float32.
- VR budget: Pimax Dream Air, RTX 4090, 90 Hz (`docs/subsystems/vr-performance.md`). The left-screen camera is an extra
  render of the world: it must fit the frame budget (lower resolution / update rate on that screen are allowed).
- Licensing: reference photos are measuring references only; they stay in the git-ignored `screenshots/m7-cockpit-refs/`
  and are not shipped or committed.
- Trademarks only internal; display names from config.

## Decided by the user (2026-11-07)

- Three cockpits (Starship, HLS, Crew Dragon); Dragon first, then port.
- Dragon original: four seats, commander left of centre; occupancy per mission. This **replaces** two entries of
  `docs/ansichten.md` section 4 ("Spieler ... mittlerer Sitz", "zwei weitere Crewmitglieder").
- Screens: left = external cameras (switchable), centre + right = our UI (content adapted, not copied). This
  **replaces** `ansichten.md` "rechts Außenansicht Ship".
- Toggle cockpit <-> external with stick button 3.
- Mouse until hand tracking: desktop pointer; VR virtual pointer on the console surfaces.
- Buttons: known function -> wired, else scenery; left-screen buttons = camera keys.
- Suits: newer EVA look; visors closed (proposal accepted with the suit choice).

## My calls (the user can override)

- Cockpit configuration in its own file `config/cockpits/crew_dragon.toml` (cabin tables, seats, eye point, screens,
  buttons with evidence, cameras), referenced from the vehicle config; crew occupancy as a new mission key (e.g.
  `[crew] count`, default 4 for crewed Dragon missions).
- Desktop toggle key `C`; F12 keeps resetting the seat position.
- Step 1 crew figures are static; motion from the accelerations of the core (pressed into the seats, nodding on
  engine cut-offs, loose arms in weightlessness, breathing) is step 2.
- Tablets are scenery in step 1.
- New subsystem doc `docs/subsystems/cockpit.md`; `docs/ansichten.md` section 4 updated with the decisions above.
- Mouse in VR = the physical mouse beside the HOTAS (the user said "mit Maus"). The pointer stays on the console
  surfaces, starts at the centre screen and is re-placed there by F12.
- Toggle in VR: the external view uses today's VR rig and its own pose; returning puts the head back at the commander's
  eye point (the seat pose is kept across toggles). F12 recentres whichever view is active.
- Vehicles without a cockpit config (Starship, HLS until step 5, Falcon 9 without Dragon, ...): no cockpit; the toggle
  does nothing (one log line) and missions start in the external view as today.
- Crew count: mission key, validated against the cockpit's seat count (1 .. seats); default for crewed Dragon missions 4.
- Crew figures: one rigged figure with per-seat poses, suit proportions [E] from the photos (no measured body needed).
- Performance: the VR frame budget of `vr-performance.md` stays the gate; the left screen may drop resolution / update
  rate (measured) to meet it.
- Exterior / interior: the plan starts with a measuring task (cabin, windows, hatch against the exterior tables); a
  mismatch is a **proposal to the user** that gates the window / hatch modelling (project rule: config changes need a
  decision). The rest of the cabin may proceed meanwhile.
- Cameras need bodies that exist: Falcon 9 stages, Dragon and ISS have detail models today
  (`vehicle-models.md`); a camera whose mount body or view target is not drawn is left out of step 1 and listed.

## Left to the builder

- How the cabin geometry is measured from the photos (known sizes: helmet, glove, hatch diameter, window extents and
  capsule diameter from the exterior tables, seat pitch) and how far the KUBAHA / Forest Katsch models (CC BY 4.0, used
  for the exterior) help inside; record each number's provenance.
- The figure / suit rig and how the player's own body is drawn (one rigged figure reused for all seats).
- How the VR pointer maps mouse motion onto the console surfaces (a 2D layout of the panel strip mapped to the 3D
  surfaces is one way), its look and hiding when idle; desktop: picking by ray from the mouse.
- How the left screen renders (SubViewport with its own camera or another way), its resolution and update rate within
  the VR budget.
- Test split (headless config / logic tests, GPU render checks to `screenshots/m7-cockpit/`, PNGs sent to the user during
  the visual work).

## Acceptance of "1:1" (user 2026-11-07)

No dimensioned drawing exists. Acceptance is visual by the user: comparison renders from the same pilot's view as the
Discovery photos (`mielke/discovery1..16`), side by side; no fixed tolerances, but every dimension documented with its
derivation and provenance.

## Far cameras (user 2026-11-07)

Terrain LOD, clouds, atmosphere and eye adaptation are centred on the main camera (the player's head). In step 1 the
**chase camera and the Dragon cameras (nose, trunk) must look right**; cameras on far stages (Falcon 9 first stage on
its return, tens of km away) **may show coarse terrain / imperfect exposure for now**. A proper per-camera solution is
its own later step (listed in the following steps).

## Known risks for the builder
- **Interior lighting vs eye adaptation**: a bright white cabin next to sunlit windows; the existing eye adaptation
  (`docs/subsystems/eye-adaptation.md`) must give a plausible cabin exposure in VR.
- **Exterior / interior agreement**: windows and hatch must line up through the hull; the exterior tables are [E] and
  may need a joint correction (proposal to the user, not a silent change: config changes need a decision).

## References (git-ignored, `screenshots/m7-cockpit-refs/`)

- `mielke/`: 33 photos from Shane Mielke (principal UI/UX designer, SpaceX Crew Displays team 2018-2020),
  https://www.shanemielke.com/work/spacex/crew-dragon-displays/ (also 4 videos there: demo, ISS docking). Images 18-33
  (Discovery documentary) are taken **from the pilot's view** and are the main reference for the console and the view.
- `sheet_dragon.jpg`: NASA photos Crew-10 / 11 / 13 cockpit (images.nasa.gov `jsc2025e064523`, `jsc2024e079790`,
  `jsc2026e404727`, `jsc2025e064540`).
- `sheet_suit.jpg`: SpaceX EVA suit (Polaris Dawn).
- Others: AJ Fitzpatrick (same team, glove touch-target rules, no designs: ITAR),
  https://www.andrewjfitzpatrick.com/portfolio-spacex.html; Dillon Baird's fan rebuild MUTANTdragon,
  https://dillonbaird.io/articles/mutantdragon/; Wikimedia `File:Crew_Dragon_interior_configuration.jpg` (CC0, seats
  S1-S4); the real display software runs on Chromium / JavaScript (press). The user is researching the UI further.

## Following steps (not step 1)

2. Crew motion from the core's accelerations; announcements (`ansichten.md` "Crew").
3. Centre / right screen UI in Dragon style with our content (`ansichten.md` "Die drei Bildschirme" content list),
   touch targets sized for gloves.
4. Hand tracking (below) replacing the mouse where available; known button functions wired.
5. Port to **Starship** and **HLS**.
6. Far cameras rendered correctly (terrain LOD / clouds / exposure per camera).

## Hand tracking (finding of the 2026-11-07 spike)

User wish: hands and fingers tracked via OpenXR, drawn on the suit's arms, but only within the arm's freedom of
movement, limited by the pilot's body (IK from the shoulder, joint limits, stiffer EVA glove, collision with console,
seat and suit; a finger stopping on a screen surface is the touch). Near the calibrated HOTAS position the hand snaps
to a grip pose.

Spike result (throwaway Godot project, not in the repo): the active runtime is **Pimax OpenXR 0.1.0** (Pimax Play
2.0.4); it advertises `XR_EXT_hand_tracking` and `XR_EXT_hand_joints_motion_range`, Godot 4.7 enables them
(`hand_tracking_supported=true`), but **no hand data arrived** (`has_tracking_data` false for the whole run, user: no
reaction to the hands). The Dream Air (SLAM, tracking cameras) hand tracking appears to live in **Pimax EVO** (early
access, requested by the user). Re-run the spike once EVO is installed. Fallback stays the mouse / VR pointer.

## Starship and HLS (collected for step 5)

- **HLS** (SpaceX renders 2024/25): upright seats in a row along the hull wall facing a band of windows, small screens
  in front, an open central shaft with stairs, living deck above. Upright seats fit a lander that never re-enters.
- **Starship (Earth return)**: no official cockpit render exists. The only official crewed non-HLS renders (dearMoon
  2021) show a **large faceted window in the nose** on the leeward (untiled) side plus a ring of small windows further
  aft. The window always looks sideways (ascent: horizon; belly-flop entry: sky; vertical landing: horizon, not the
  ground below), so flying is by camera. Entry loads act across the axis: seats (and console) likely need to pivot by
  ~90 deg between ascent / landing and entry (Dragon seats also tilt). Open question to the user: large dearMoon nose
  window or small single windows.
- References: `screenshots/m7-cockpit-refs/sheet_starship.jpg`, `sheet_hls.jpg`.
