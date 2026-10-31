# Pad B Physics Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the physics of the Starbase pad once onto the measured Pad B: position, tower direction, deck height, hinges on the tower corner, arm length, catch height and catch pins; then draw rails, bracing and hoist cables on that corner.

**Architecture:** All numbers stay configuration (`config/launch_sites.toml`, `config/vehicles/starship_block3.toml`, `config/structures/starbase_olp2.toml`). `contact::TowerGeometry` is the one place that derives hinges and the tower box; guidance and the game read it. Visual-only offsets disappear: the drawn structure and the physics use the same geometry.

**Tech Stack:** C++20 core (Catch2), GDExtension bindings, Godot 4 double, Python tools (pytest, Blender generators).

**Spec:** user rulings 2026-10-31 and the rows of `docs/subsystems/launch-structures-measurements.md` and "Physics deviations" in `docs/subsystems/launch-structures.md`. Read both.

## Global Constraints

- Reality first: the drawn geometry is the reference, the physics follows once (user 2026-10-31). Physics never runs in GDScript.
- Pad B ring centre: latitude 25.99677, longitude -97.15804 (user 2026-10-31: use these).
- Deck (ring top) 14.5 m. Arms point about 155-168 deg at the catch; the tower lies north-north-west of the ring.
- The hinge of the whole arm system sits on the tower corner toward the mount. Rail bracing goes at the top and the bottom back to the two neighbouring corners. The hoist cable pulls from above directly at that corner. The ship QD arm is always below the chopsticks.
- `arm_closed_gap_m` 9.6 and closed pose at opening 0 stay unchanged.
- Commit directly on `main`, push after each green task; test only the tags the change affects, `[slow]` of them included; long runs in the background with output in a file. English in the repo.
- Out of scope (own vehicle plan): flap areas, nose, thrust point, SL ring, grid fin shape. Pad 1 stays unmodelled (see `project-cape-multipad-idea`).

## Decided numbers

| Key (`[site.tower]` unless noted) | Old | New | Source |
|---|---|---|---|
| site `latitude_deg`, `longitude_deg` | 25.99686, -97.15394 | 25.99677, -97.15804 | above |
| `azimuth_deg` | 315 | 350 | squares: X axis bearing 26.5 (mean of trench 113 - 90 and foot normal 30) minus `plan_yaw_deg` 36.5; measured line 334-348 |
| `plan_yaw_deg` (new, moved from `[mount]`) | - | 36.5 | squares against the ring line |
| `distance_m` | 30 | 36.1 | 35 (3), 36 (4) |
| `width_m` | 12 | 13.3 | outer leg width 13.3-14; box of the lattice |
| `hinge_distance_m` (new) | 24 (derived) | 26.3 | outer corner toward the mount along the line, from the two numbers above |
| `arm_length_m` | 36 | 28.2 | hinge to axis + the tip's 1.9 m |
| `mount_height_m` | 20 | 14.5 | measured |
| `catch_height_m` | 95 | 133 | measured 133 (4) |
| `arm_standby_deg` | 6.5 | keep | about 3 m clear per side at 26.3 m |
| `[stage.catch_pins]` station, azimuths | 66.5, 45 / 225 | hinge station of the fins - 1.0 = 63.0, 0 / 180 | user 2026-10-31; fin station stays 64.0 here |

Parked height 134 m, carriage stroke and travel: Task 3 confirms the travel reaches them.

## Review Focus

1. A stack on the pad: engines' bells at the 14.5 m ring and the ridge at 6.9 m: bells must clear the deflector at liftoff and in the first second of ignition sag.
2. A booster returning with the new catch height 133 m under an opening arm: the hull clears the rotated tower box corner at every opening from `arm_open_deg` to 0.
3. Two-pad neighbourhood and terrain: the plateau at the new site reaches over the old pad's imagery; a 150 m plateau + 50 m apron must not cut the road or leave a step at the old position.
4. Saved or configured epochs in validated missions (`flight14_ascent`): 410 m shift and a 5.5 m lower start must not push a validated ascent out of its tolerance silently; the tolerance is checked, not widened.
5. Abort aim: `[site.sea]` direction and shoreline distance after the shift: the abort guidance must still land beyond the shoreline.

---

### Task 1: Site, tower box and hinge geometry in the core

**Files:**
- Modify: `config/launch_sites.toml`, `core/include/starsim/config/launch_site_config.hpp`, `core/src/config/launch_site_config.cpp`, `core/include/starsim/contact/tower_geometry.hpp`, `core/src/contact/tower_geometry.cpp`, `config/earth_terrain.toml`
- Test: `core/tests/test_tower_geometry.cpp`, `core/tests/test_launch_site_config.cpp`

**Interfaces:**
- Produces: `TowerConfig::plan_yaw_rad`, `TowerConfig::hinge_distance_m`; `TowerGeometry::hinge(int)` at `toward * hinge_distance_m`; `tower_box()` axes rotated by `plan_yaw_rad`, same formula as `TowerView._plan_axes`.

- [ ] **Step 1: Write the failing tests** in `test_tower_geometry.cpp`, tag `[contact]`: `"Tower box is parallel to the measured squares"` (box x axis = toward cos(yaw) - across sin(yaw)); `"Hinges straddle the corner toward the mount"` (hinge(side) along toward = 26.3, across = side * 5.8; the box corner nearest the axis within 2 m of the line at 26.3 +- 1); keep `"Closed arms hold the pins but not the hull"` and `"Open arms leave room"` with the new numbers.
- [ ] **Step 2: Run** `tools/build.ps1 -NoBindings -TestFilter "[contact]"`. Expected: FAIL (unknown keys).
- [ ] **Step 3: Implement** the keys of "Decided numbers"; `plan_yaw_deg` is read in `[site.tower]`, removed from `[mount]`; `tools/structures` and `tower_view.gd` read it from the geometry dictionary (add to the binding's tower dictionary).
- [ ] **Step 4: Move the site data:** `python tools/data_pipeline/site_heights.py --write`; the terrain centre in `config/earth_terrain.toml` follows, tiles regenerated only if the area no longer covers the 5.5 km.
- [ ] **Step 5: Add** to `test_launch_site_config.cpp` the position, azimuth 350 and `[site.sea]`; the sea shoreline value is re-read from the mosaic and updated (Review Focus 5). Run `[contact]`, `[config]` tags, then Commit `feat(contact): tower at the measured Pad B ...`.

### Task 2: Mount height and pad start

**Files:**
- Modify: `config/launch_sites.toml`, `game/world/structures/tower_lights.gd`, `tools/structures/check_envelope.py`, `docs/subsystems/launch-structures.md`
- Test: `tools/structures/tests/test_mount_layout.py`, `game/tests/tower_view_test.gd`, core tests of tags using `mount_height_m`

**Interfaces:**
- Consumes: Task 1 geometry.

- [ ] **Step 1: Write the failing test** `test_ring_top_is_the_physics_deck` back in `test_mount_layout.py`: `deck_height_m == physics mount_height_m` (0.01 m); new core case `"A stack stands on the 14.5 m ring"` `[contact]`.
- [ ] **Step 2: Run** them. Expected: FAIL at 20 m.
- [ ] **Step 3: Set** `mount_height_m = 14.5`; remove the "Mount deck" exception in `check_envelope.py` and its deviation row; bell clearance test for Review Focus 1 (bell at depth <= its length vs `_deflector_z`, liftoff and 1 m sag).
- [ ] **Step 4: Validate:** ascent tags with `[slow]`, `flight14_ascent` within its documented tolerance unchanged; if it leaves, stop and report instead of widening (Review Focus 4). `python tools/vehicle_report.py starship_block3`.
- [ ] **Step 5: Commit** `feat(site): stack starts from the measured 14.5 m deck`.

### Task 3: Arms, catch height, catch pins (physics-critical)

**Files:**
- Modify: `config/launch_sites.toml`, `config/vehicles/starship_block3.toml`, `core/src/contact/pin_contact.cpp` only if it assumes azimuths, `core/src/gnc/booster_return*.cpp` and `core/src/gnc/landing_site.cpp` only for hard assumptions found, `config/vehicles/detail/starship_block3.toml`
- Test: `core/tests/test_catch_arms.cpp`, `test_catch_tower.cpp`, `test_pin_contact.cpp`, `test_catch_trigger.cpp`, `test_scvx_catch.cpp`, `test_booster_return.cpp`, `test_ship_catch.cpp`

**Interfaces:**
- Produces: booster caught with pins at station 63.0, azimuths 0 / 180, arm tops at 133 m, roll set so the 90 deg fin points away from the tower.

- [ ] **Step 1: Write the failing tests:** `"Pins at 0 and 180 rest on both closed arms"` `[contact]` (both pads loaded, hull 0 contact); `"Arm sweep clears the rotated tower corner"` `[contact]` (Review Focus 2: openings 0..40 deg, carriage offsets over the stroke); `"Catch ends with the 90 deg fin away from the tower"` in `test_catch_trigger.cpp`.
- [ ] **Step 2: Run** the catch tags. Expected: FAIL.
- [ ] **Step 3: Set** arm length, catch height, pins. Find with a search every use of the old azimuths and of a fixed catch height; change guidance only where it assumes them; the booster's roll target comes from the pin azimuths and `toward_tower`, no magic number.
- [ ] **Step 4: Re-plan check:** the full booster return of `starship_leo` with its `[slow]` cases, ship catch cases too (`catch_profile` video reference: 13 engines at ~2 km, 3 engines from ~500 m). A higher catch shifts the landing burn: it is the planner's result, report the shift.
- [ ] **Step 5: Commit** `feat(catch): arms and pins as measured ...`; Opus review here.

### Task 4: Drawn arms, carriage, rails, bracing and cable on the corner

**Files:**
- Modify: `tools/structures/blender/mechazilla_layout.py`, `config/structures/starbase_olp2.toml`, `game/world/tower_view.gd`, `game/world/structures/tower_lights.gd`, `game/tests/tower_view_test.gd`
- Test: `tools/structures/tests/` mechazilla cases, `check_envelope.py`

**Interfaces:**
- Consumes: `hinge(int)`, `arm_length_m`, `plan_yaw_rad` via the geometry dictionary.

- [ ] **Step 1: Write failing tests:** `test_rail_runs_on_the_corner`, `test_bracing_top_and_bottom_reach_both_neighbour_corners` (4 members, ends within 0.3 m of the corner rail and the leg lines), `test_hoist_cable_meets_the_corner` (top anchor over the rail within 0.5 m), `test_qd_arm_is_below_the_arms` (QD deck < lowest arm point at every carriage position used in a stack).
- [ ] **Step 2: Run** them. Expected: FAIL.
- [ ] **Step 3: Implement** in the layout; lengthen the arm legs from 25.9 to 28.2 m; drawn hinge = physics hinge; lamps from the corner instead of `face_m`; remove the fixed deviation rows.
- [ ] **Step 4: Render** `render_check` top and side views into `screenshots/pad-b-plan/`, send the PNGs, compare with `DSC_6301`, `DSC05957`.
- [ ] **Step 5: Run** `tower_view_test`, `tower_lights_test`, `smoke_test`, `check_envelope`; Commit.

### Task 5: Measured hardstand and site rename

**Files:**
- Modify: `config/structures/starbase_olp2.toml`, `tools/structures/blender/mount_layout.py`, `config/launch_sites.toml` and all files holding `starbase_olp1`, `docs/ansichten.md`, `docs/subsystems/launch-structures.md`
- Test: `tools/structures/tests/test_mount_layout.py`, `game/tests/trench_mask_check.gd`, `game/tests/mission_catalogue_test.gd`

- [ ] **Step 1: Write the failing test** `test_hardstand_is_the_measured_outline`: SW edge bearing 167 (3), length 218 (10); SE edge bearing 63 (3), 203 (10), converted with azimuth and yaw; concrete up to the road.
- [ ] **Step 2: Run** it. Expected: FAIL.
- [ ] **Step 3: Replace** `hardstand_outline_m`; the convexity assumption of `_x_range` holds for the outline or the outline is cut; the terrain plateau checked against Review Focus 3 by a render.
- [ ] **Step 4: Rename** the site id to `starbase_pad2` and the display name to `Starbase Pad 2` mechanically, including `PROTOTYPE_SITE_ID` and test names; run the tags of the touched tests.
- [ ] **Step 5: Docs and Commit:** subsystem file current state, superseded numbers into `docs/history/launch-structures.md`, `docs/ansichten.md`; commit, push.

### Task 6: Final review

- [ ] **Step 1:** Opus review of the whole range against this plan.
- [ ] **Step 2:** Affected tags incl. `[slow]` once more, background with a log file. Expected: all green.
- [ ] **Step 3:** Ledger and memory updated, plan moved to `docs/superpowers/archive/plans/`, commit, push.
