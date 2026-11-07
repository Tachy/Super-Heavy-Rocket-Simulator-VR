# M7 Cockpit Step 1: Crew Dragon Cabin Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The player sits in a 1:1 Crew Dragon cabin as commander (VR and desktop), sees external cameras on the left screen, toggles to the existing external view, and works the console with a mouse-driven suited hand.

**Architecture:** The cabin, the crew figure and the console are our own Blender-generated glTFs from typed tables in a new `config/cockpits/crew_dragon.toml`, built by the existing `build_models.py` pipeline into `data/models/crew_dragon/`. A new `game/cockpit/` package attaches the cabin to the capsule stage of the Dragon's `VehicleView`, adds a cockpit camera mode to `World` (eye point in the capsule frame = floating origin), renders the left screen from a `SubViewport` camera, and maps mouse input to a pointer on the console surfaces that a smoothed IK arm follows. Nothing touches the core: crew count is read in GDScript.

**Tech Stack:** Godot 4.7 double (GDScript), Blender headless generators (Python), pytest, `TomlLite`, `I18n`.

**Spec:** `docs/superpowers/specs/2026-11-07-m7-cockpit-crew-dragon-design.md` (read it; the plan argues from it).

## Global Constraints

- No physics in GDScript; the cockpit only reads `FlightSim` state and sends existing commands.
- Every cabin / console / figure dimension carries `[S]` / `[E]` / `[D]` and its derivation (photo name + known size), as `config/README.md` describes.
- Rebuild everything ourselves: no third-party mesh data, no photo pixels in repo or `data/`; reference photos stay in git-ignored `screenshots/m7-cockpit-refs/`.
- Surface detail (panel lines, fasteners, labels, placards) by texture / shader; geometry only for the silhouette.
- Exterior config (`config/vehicles/detail/crew_dragon.toml`) is NOT changed without the user's decision (Task 1 gate).
- Texts only through `I18n` with keys in both `config/i18n/de.toml` and `en.toml` (`python tools/i18n_check.py` green).
- All images to `screenshots/m7-cockpit/<topic>/`; send progress PNGs to the user during visual tasks (Tasks 4, 5, 7, 8, 9).
- VR frame budget of `docs/subsystems/vr-performance.md` is the gate; the left screen may lower resolution / update rate.
- Player seat: commander, left of centre (seat 2 of S1..S4 counted from the left as seen by the crew). Toggle: stick button 3 + key `C`. F12 recentres the active view.
- Commit on `main`, push after each task with green affected tests (`CLAUDE.md`).

## Review Focus

1. A mission whose vehicle has no cockpit (Starship, HLS, Falcon 9 cargo): no cockpit, toggle inert with one log line, start in the external view - test in Task 6.
2. Stage changes (Falcon 9 staging, trunk jettison, undocking) rebuild the views: the cabin must reattach to the capsule stage and the player must not jump - test in Task 6.
3. A camera's mount body vanishes or separates (first stage after landing, second stage after Dragon separation): the camera list shrinks and the screen falls back to the chase camera - test in Task 7.
4. Pointer at the strip's edges, in the gaps between screens or at a point out of the arm's reach: pointer clamps, hand stays within reach, no NaN pose - test in Task 8.
5. Toggling repeatedly, under time warp, and in VR after F12: exposure snaps (`World._view_jumped()`), head returns to the eye point, no drift - test in Task 6.

---

### Task 1: Measure the cabin and gate the window / hatch agreement (Opus)

Judgement-heavy measurement work; no game code.

**Files:**
- Create: `config/cockpits/crew_dragon.toml` (sections `[cabin]`, `[[seat]]`, `[eye]`, `[console]`, `[[screen]]`, `[[button]]`, `[hotas]`; numbers only in this task)
- Create: `docs/subsystems/cockpit.md` (model, provenance table, open points)
- Create: `tools/cockpit/measure_refs.md` is NOT wanted; scratch scripts stay in the session scratchpad.

**Interfaces:**
- Produces: the TOML layout below, which Tasks 2-3 consume. Frame = the capsule stage body frame of `vehicle-models.md` (X station from the shield apex, nose +X; (Y, Z) = r (cos a, sin a); hatch azimuth 0).

```toml
[cabin]           # inner pressure-vessel profile and fixtures
profile_m = [[x, r], ...]                     # inner wall of revolution
floor_x_m = ...                               # aft bulkhead / floor station
windows = [[az_deg, x_m, half_x_m, half_az_deg], ...]   # must match the exterior table or be flagged
hatch = [az_deg, x_m, half_x_m, half_az_deg]
[[seat]]          # S1..S4 from the crew's left
id = "s2"; role = "commander"; hip_m = [x, y, z]; back_angle_deg = ...; width_m = ...
[eye]             # commander's design eye point and forward axis in the capsule frame
position_m = [x, y, z]; forward = [x, y, z]; up = [x, y, z]
[console]         # housing and the three screens as rectangles (centre, normal, up, width, height)
[[screen]]        # id = "left" | "centre" | "right"; centre_m, normal, up, size_m, aspect
[[button]]        # id, row, centre_m, size_m, kind = "push" | "covered" | "handle", function = "camera" | "<known>" | "none", evidence = "..."
[hotas]           # stick_grip_m, throttle_grip_m (from the commander's eye point; calibratable later)
```

- [ ] **Step 1:** Derive each number from the references (`screenshots/m7-cockpit-refs/mielke/discovery1..16` pilot view, NASA cockpit photos, Wikimedia `Crew_Dragon_interior_configuration.jpg`) against known sizes (capsule inner radius from `crew_dragon.toml` profile minus wall, helmet ~0.30 m, glove, hatch extents). Write every value with its tag and derivation comment.
- [ ] **Step 2:** List the physical buttons per row with function and evidence; functions without evidence are `"none"`; the row under the left screen gets `function = "camera"` with `camera_slot = 1..n`.
- [ ] **Step 3:** Compare interior windows and hatch with the exterior table (`windows`, `hatch` in `config/vehicles/detail/crew_dragon.toml`). Record the differences in `cockpit.md`.
- [ ] **Step 4 (gate):** If any window / hatch differs by more than its half extent's 10 %, STOP and write the proposal (which numbers, which evidence) for the user; window and hatch geometry in Task 3 waits for the decision, the rest proceeds.
- [ ] **Step 5:** Commit `feat(cockpit): measured Crew Dragon cabin tables with provenance` and push.

### Task 2: Cockpit config loader and crew count

**Files:**
- Create: `game/cockpit/cockpit_config.gd` (class `CockpitConfig`)
- Modify: `game/ui/mission_catalogue.gd:76-93` (read `[crew] count`)
- Modify: `config/missions/falcon9_crew_dragon.toml`, `falcon9_crew_dragon_iss.toml`, `crew_dragon_iss.toml`, `crew_dragon_iss_departure.toml`, `crew_dragon_return.toml` (`[crew] count = 4`)
- Modify: `config/vehicles/crew_dragon.toml` (`cockpit = "crew_dragon"` key; also teach `core/src/config/vehicle_config.cpp` to accept and ignore it if it refuses unknown keys)
- Test: `game/tests/cockpit_config_test.gd`

**Interfaces:**
- Produces:
  - `static func CockpitConfig.for_vehicle(vehicle_id: String) -> CockpitConfig` (null when no part of a `+` stack id names a cockpit; cached; one log line).
  - Properties: `id: String`, `capsule_stage_id: String`, `eye: Transform3D` (capsule frame, body axes), `seats: Array[Dictionary]`, `screens: Dictionary` (id -> {centre, normal, up, size}), `buttons: Array[Dictionary]`, `cameras: Array[Dictionary]`, `hotas: Dictionary`, `hotas_props_enabled: bool`.
  - `static func MissionCatalogue.crew_count(mission_id: String) -> int` (missing key = seat count of the cockpit, else 0).
  - `func CockpitConfig.validate() -> PackedStringArray` (empty = valid).

- [ ] **Step 1: Write the failing test** `cockpit_config_test.gd`: `for_vehicle("crew_dragon")` and `for_vehicle("falcon9_block5+crew_dragon")` return the config; `for_vehicle("starship_block3")` returns null; four seats, exactly one `role == "commander"`, its id `"s2"`; screens `left`, `centre`, `right` present; `validate()` empty; a crafted table with crew count 5 or 0 makes `validate_crew(5)` fail; `crew_count("crew_dragon_return") == 4`; `crew_count("starship_leo") == 0`.
- [ ] **Step 2:** Run `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/cockpit_config_test.gd`; expect FAIL (class missing).
- [ ] **Step 3:** Implement with `TomlLite.load_file`; validation messages English.
- [ ] **Step 4:** Run the test (PASS) and `powershell -ExecutionPolicy Bypass -File tools/run_godot_tests.ps1 -Only mission_catalogue_test,cockpit_config_test`; core config tag if `vehicle_config.cpp` changed (`tools/build.ps1 -NoBindings -Fast -TestFilter "[config]"`).
- [ ] **Step 5:** Commit `feat(cockpit): cockpit config loader and mission crew count`; push.

### Task 3: Cabin and console generator

**Files:**
- Create: `tools/structures/blender/gen_dragon_cabin.py`
- Modify: `tools/structures/vehicle_spec.py` (`load_dragon` also loads and validates `config/cockpits/<cockpit>.toml` into `spec["cockpit"]`)
- Modify: `tools/structures/blender/generators.toml` `[crew_dragon]` (`generators += "gen_dragon_cabin.py"`, `outputs += "dragon_cabin.gltf"`)
- Modify: `tools/structures/build_models.py` `dragon_manifest` (file list, `cockpit` block: node names, screen rectangles, button ids)
- Modify: `game/vehicle/vehicle_models.gd` (`MODEL_FILES["crew_dragon"] += "dragon_cabin"`, `SOURCE_SCRIPTS += "gen_dragon_cabin.py"`, staleness also on `config/cockpits/crew_dragon.toml`)
- Test: `tools/structures/tests/test_dragon_cabin.py`

**Interfaces:**
- Consumes: Task 1 TOML.
- Produces: `dragon_cabin.gltf` in the capsule stage frame (same frame as `dragon_capsule.gltf`) with nodes `cabin_shell`, `floor`, `hatch_frame`, `window_<i>` (wells, glass separate as `window_glass_<i>`), `seat_s1..s4`, `console_housing`, `screen_left|centre|right` (one quad each, UV 0..1 over the active area, +normal toward the crew), `tablet_l`, `tablet_r`, `button_<id>` (one node per button), `handle`, `light_<i>`, `hotas_stick`, `hotas_throttle`. Manifest `cockpit.screens[id] = {centre, normal, up, size}` and `cockpit.buttons[id] = {centre, size}` in the capsule frame.

- [ ] **Step 1: Write the failing test** `test_dragon_cabin.py`: spec loads; every Task 1 value is tagged; generated node set equals the list above; each `screen_*` quad's centre / size equals the TOML within 1 mm; the cabin shell lies inside the exterior profile (`dragon_profile` inner wall) everywhere with >= 2 cm wall; window wells line up with the exterior windows within the Task 1 tolerance (skip with reason while the gate is open); closed-surface check as `test_dragon_closed.py` does for the shell.
- [ ] **Step 2:** `python -m pytest tools/structures/tests/test_dragon_cabin.py -q` -> FAIL.
- [ ] **Step 3:** Implement the generator (lathe shell, simple boxes / bevelled slabs for console, seats from a few profiles; no detail geometry).
- [ ] **Step 4:** `python tools/structures/build_models.py --vehicle crew_dragon --force`, then the test and `python -m pytest tools/structures/tests/test_dragon_spec.py tools/structures/tests/test_vehicle_models.py -q` -> PASS.
- [ ] **Step 5:** Commit `feat(cockpit): Crew Dragon cabin and console generator`; push.

### Task 4: Crew figure (EVA suit) generator

**Files:**
- Create: `tools/structures/blender/gen_crew_figure.py`, `config/cockpits/crew_figure.toml` (body and suit dimensions [E] from `screenshots/m7-cockpit-refs/sheet_suit.jpg` and the Discovery photos)
- Modify: `generators.toml`, `build_models.py`, `vehicle_models.gd` (output `crew_figure.gltf` in the crew_dragon set)
- Test: `tools/structures/tests/test_crew_figure.py`

**Interfaces:**
- Produces: `crew_figure.gltf`: one skinned mesh (suit, boots, gloves, helmet with closed copper visor as separate surface `visor`), skeleton bones `hips, spine, chest, neck, head, shoulder_l/r, upper_arm_l/r, forearm_l/r, hand_l/r, index_1..3_l/r, thumb_1..2_l/r, fingers_l/r (merged other fingers), thigh_l/r, shin_l/r, foot_l/r`; rest pose = seated in a Dragon seat. Manifest `crew_figure.bones` with rest positions.

- [ ] **Step 1: Failing test:** bone set as listed; arm length shoulder->index tip within the [E] table; figure seated on `seat_s2`'s hip point fits inside the seat envelope (no part below the seat pan or behind the back by more than 1 cm); visor is its own surface.
- [ ] **Step 2:** Run, FAIL. **Step 3:** Implement (simple capsule / lathe segments, surface stripes by texture). **Step 4:** Build + PASS; look shot of the seated figure with `vehicle_look_shots.gd` to `screenshots/m7-cockpit/figure/`, send to the user.
- [ ] **Step 5:** Commit `feat(cockpit): crew figure in the EVA suit`; push.

### Task 5: Cabin in the game and look pass (Opus review of the look)

**Files:**
- Create: `game/cockpit/cockpit_view.gd` (class `CockpitView`, child of the capsule stage node), `game/shaders/cabin.gdshader` (painted / matte cabin surfaces, panel detail map), `game/tests/cockpit_shots.gd` (pilot-view and comparison shots)
- Modify: `game/vehicle/vehicle_view.gd` (create `CockpitView` under the stage whose id is `CockpitConfig.capsule_stage_id`; rebuild on stage changes like `docking_mechanism`)
- Modify: `tools/run_godot_tests.ps1` (`$needsGpuOrModels += cockpit_view_test`)
- Test: `game/tests/cockpit_view_test.gd`

**Interfaces:**
- Consumes: `CockpitConfig`, `VehicleModels.load_for("crew_dragon")["dragon_cabin"]`, `["crew_figure"]`.
- Produces: `CockpitView.eye_node: Node3D` (at `CockpitConfig.eye`), `CockpitView.screen_surface(id: String) -> MeshInstance3D`, `CockpitView.button_node(id: String) -> Node3D`, `CockpitView.set_crew(count: int)` (figures in seats in the order s2 commander, s3 pilot, s1, s4; the commander figure is the player body), `CockpitView.set_hotas_props(enabled: bool)`.

- [ ] **Step 1: Failing test** `cockpit_view_test.gd`: a `falcon9_crew_dragon` session at T+0 has exactly one `CockpitView`; its `eye_node` global position equals the capsule stage transform * eye within 1 mm; after staging (T+170 s) and after trunk jettison in `crew_dragon_return` it still exists once; `set_crew(2)` shows the player body and one figure; exterior capsule hull casts shadows double-sided (sunlight enters only through windows).
- [ ] **Step 2:** Run, FAIL. **Step 3:** Implement; cabin lights as `OmniLight3D` / `SpotLight3D` from `light_<i>`; screens get a dark Dragon-blue emissive placeholder through the display-referred colour path (`display_colour.gdshader`).
- [ ] **Step 4:** PASS; `cockpit_shots.gd -- --mission=falcon9_crew_dragon --t_plus=-60 --out_dir=<repo>/screenshots/m7-cockpit/look` writes the pilot-view shot at the Discovery photos' framing plus a side-by-side PNG with `mielke/discovery1.jpg`; send to the user; iterate materials / lighting until the user's verdict ("1:1" acceptance is the user's).
- [ ] **Step 5:** Commit `feat(cockpit): Crew Dragon cabin in the game`; push.

### Task 6: Cockpit camera mode, toggle and start

**Files:**
- Create: `game/cockpit/cockpit_camera.gd` (class `CockpitCamera`: pose from the eye node, desktop mouse look, VR base), `game/cockpit/hotas_buttons.gd` (class `HotasButtons`: edge-detected joystick buttons by role from `config/input_hotas.toml [buttons]`)
- Modify: `game/world/world.gd` (`camera_mode` cockpit: `_update_camera` uses `CockpitCamera` for `position_gcrf_m` / basis; `toggle_cockpit()`; start in cockpit when `CockpitConfig.for_vehicle(controlled vehicle)` exists; `_view_jumped()` on every switch), `game/xr/vr_panel_mouse.gd:15` (`PASS_KEYS += KEY_C`), `game/main.gd` (`KEY_C` in `_unhandled_key_input`), `game/tests/joy_probe.gd` (`--buttons` prints pressed button indices), `config/input_hotas.toml` (`[buttons] stick_device = "T16000M"`, `cockpit_toggle = <measured>`, `camera_next = <measured>`)
- Test: `game/tests/cockpit_camera_test.gd`

**Interfaces:**
- Consumes: `CockpitView.eye_node`, `World.flight.views`, `XrManager.set_base`, `XrManager.recenter`.
- Produces: `World.cockpit_active: bool`, `World.toggle_cockpit() -> void`, signal `World.cockpit_toggled(active: bool)`, `HotasButtons.pressed(role: String) -> bool` (true on the frame of the press), `CockpitCamera.position_gcrf_m: Vector3`, `CockpitCamera.basis_gcrf: Basis`.

- [ ] **Step 1 (user):** Run `godot --path game --script res://tests/joy_probe.gd -- --buttons`, ask the user to press stick button 3 ("left of the thumb hat") and the camera key they choose; write the indices to `[buttons]`.
- [ ] **Step 2: Failing test** `cockpit_camera_test.gd`: Dragon mission starts with `cockpit_active`; `position_gcrf_m` equals body position + attitude * (eye - CG) within 1 mm over 100 ticks incl. staging; `toggle_cockpit()` twice restores the same head pose and calls `_view_jumped` twice (counter); toggling under time warp x100 is stable; a Starship mission starts external, `toggle_cockpit()` leaves it external and logs once; desktop right-drag look is clamped to +-170 deg yaw, +-80 deg pitch relative to the seat and reset by F12; `HotasButtons` edge detection from a scripted button state sequence.
- [ ] **Step 3:** Run FAIL; implement; run PASS; plus `run_godot_tests.ps1 -Only camera_math_test,cockpit_camera_test,cockpit_config_test`.
- [ ] **Step 4:** Commit `feat(cockpit): cockpit view mode, toggle on stick button 3 and C`; push.

### Task 7: Left screen external cameras

**Files:**
- Create: `game/cockpit/screen_camera.gd` (class `ScreenCamera`: `SubViewport` + `Camera3D` in the world, texture onto `screen_left`), `game/cockpit/camera_catalog.gd` (class `CameraCatalog`: available cameras from config + live bodies)
- Modify: `config/cockpits/crew_dragon.toml` (`[[camera]]` entries: `id`, `body_vehicle` / `stage` id, `mount_m`, `look`, `up`, `fov_deg`, `name_key`; ids `chase`, `stage1_aft`, `stage2_engine`, `dragon_nose`, `dragon_trunk`), `config/i18n/de.toml`, `en.toml` (`[cockpit.camera] chase = "Verfolger"`, ...)
- Test: `game/tests/camera_catalog_test.gd`, GPU check `cockpit_shots.gd --screen`

**Interfaces:**
- Consumes: `FlightSim` snapshot stage dictionaries (`vehicle_id`, stage ids), `OrbitCamera` chase pose for `chase`, `CockpitView.screen_surface("left")`.
- Produces: `CameraCatalog.available(sim) -> Array[String]`, `CameraCatalog.mount_pose_gcrf(sim, id) -> Transform3D`, `ScreenCamera.select(id: String)`, `ScreenCamera.next()`, `ScreenCamera.current_id: String`, settings `size_px` (default 1024 x 640) and `update_hz` (default 30, measured in VR).

- [ ] **Step 1: Failing test:** at T-60 `chase`, `stage1_aft`, `stage2_engine` and `dragon_trunk` are offered; `dragon_nose` only while `capsule_state.nose_cone_open`; after staging `stage1_aft` stays while the first stage body exists and vanishes after it is gone; current camera falling out -> `chase`; `next()` cycles in config order over available ids only; mount pose in GCRF equals body pose * mount within 1 mm.
- [ ] **Step 2:** FAIL; implement (screen material via display-referred path; cameras far away accepted coarse, spec "Far cameras"); PASS.
- [ ] **Step 3:** GPU: `cockpit_shots.gd --screen` for each camera to `screenshots/m7-cockpit/screen/`; VR frame time with the screen on vs off with `perf_log` (`vr-performance.md` procedure); lower `update_hz` / `size_px` until within budget; record numbers in `cockpit.md`; send PNGs.
- [ ] **Step 4:** Commit `feat(cockpit): external cameras on the left screen`; push.

### Task 8: Console pointer, smoothed hand and touch

**Files:**
- Create: `game/cockpit/console_strip.gd` (class `ConsoleStrip`: 2D strip coordinates <-> 3D points on screens and button rows), `game/cockpit/console_pointer.gd` (class `ConsolePointer`: desktop ray pick / VR mouse-delta pointer on the strip, click dispatch), `game/cockpit/arm_ik.gd` (class `ArmIk`: two-bone IK shoulder-elbow-wrist with joint limits, index pose), `game/cockpit/hand_motion.gd` (class `HandMotion`: critically damped follow + tap)
- Modify: `game/cockpit/cockpit_view.gd` (player figure's arms driven by `ArmIk`; idle hands in HOTAS grip pose), `game/cockpit/screen_camera.gd` (on-screen camera button region on `screen_left`)
- Test: `game/tests/console_pointer_test.gd`

**Interfaces:**
- Consumes: manifest `cockpit.screens` / `cockpit.buttons`, figure bones, `ScreenCamera.next()` / `select()`.
- Produces: `ConsoleStrip.to_surface(p: Vector2) -> Dictionary` ({`surface_id`, `point_m`, `normal`}), `ConsoleStrip.clamp(p: Vector2) -> Vector2`; `ConsolePointer` signal `pressed(surface_id: String, uv: Vector2)`; `HandMotion.step(target: Vector3, delta: float) -> Vector3` (half-life 0.08 s [E], no overshoot); `HandMotion.tap()` (press 0.12 s in, 0.15 s out, finger tip reaches the surface plane within 1 mm); `ArmIk.solve(shoulder: Vector3, target: Vector3, pole: Vector3) -> Dictionary` (bone rotations, target clamped to reach).
- Hand side: left screen -> left hand, otherwise right hand; the other hand stays on its HOTAS grip.

- [ ] **Step 1: Failing test:** strip edges and screen gaps clamp and never return a point off a surface; a step target 30 cm away gives a monotonic, overshoot-free path whose speed never jumps by more than the damping allows (no "eckig / ruckartig"); `tap()` touches the plane within 1 mm and returns; a target beyond reach is clamped, no NaN; click on the camera button calls `ScreenCamera.next()`; click on `button_<camera slot k>` selects camera k; click on a `function = "none"` button does nothing but animates the tap.
- [ ] **Step 2:** FAIL; implement; PASS.
- [ ] **Step 3:** GPU: hand-following shots / short frame sequence to `screenshots/m7-cockpit/hand/`; send to the user; desktop run by the user (move / click) for smoothness.
- [ ] **Step 4:** Commit `feat(cockpit): console pointer with a smoothed suited hand`; push.

### Task 9: HOTAS on the armrests, crew occupancy, VR acceptance

**Files:**
- Modify: `game/cockpit/cockpit_view.gd` (`hotas_stick` / `throttle` at `[hotas]` grips relative to the eye; stick deflection and throttle lever follow the real axes from `FlightController` input, view only; crew from `MissionCatalogue.crew_count`), `config/cockpits/crew_dragon.toml` (`[hotas] props = true`)
- Test: extend `cockpit_view_test.gd`

- [ ] **Step 1: Failing test:** props present when enabled and absent when `props = false`; stick prop angle follows a scripted axis value; crew count 1..4 shows that many figures in the order s2, s3, s1, s4.
- [ ] **Step 2:** FAIL; implement; PASS.
- [ ] **Step 3:** User VR run (`Start-VR.cmd`, Dragon mission): seat height, HOTAS prop position against the real HOTAS (adjust `[hotas]` with the user), toggle, F12, pointer, frame time; record in `cockpit.md`.
- [ ] **Step 4:** Commit `feat(cockpit): armrest HOTAS and crew occupancy`; push.

### Task 10: Docs and final review

**Files:**
- Modify: `docs/subsystems/cockpit.md` (current state, decisions, validation, open points incl. far cameras and hand tracking), `docs/subsystems.md` (index row), `docs/architecture.md` (game: `game/cockpit/`), `docs/ansichten.md` section 4 (umgesetzt), `docs/setup.md` (joy_probe `--buttons`, cockpit shots)

- [ ] **Step 1:** Update the docs; `python tools/i18n_check.py` green.
- [ ] **Step 2:** Affected tests with `[slow]` / GPU list: `tools/run_godot_tests.ps1 -Only cockpit_config_test,cockpit_view_test,cockpit_camera_test,camera_catalog_test,console_pointer_test,mission_catalogue_test,i18n_test -Gpu`, `python -m pytest tools/structures/tests -q`.
- [ ] **Step 3:** Final whole-plan review (Opus), fix round by a fresh fixer if needed.
- [ ] **Step 4:** Commit `docs(cockpit): step 1 state`; push; ledger + memory; ask the user for `/clear`.

## Ledger

| Task | State | Notes |
|---|---|---|
| 1 | open | |
| 2 | open | |
| 3 | open | |
| 4 | open | |
| 5 | open | |
| 6 | open | |
| 7 | open | |
| 8 | open | |
| 9 | open | |
| 10 | open | |
