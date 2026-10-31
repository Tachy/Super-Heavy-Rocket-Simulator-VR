# Mechazilla Fine Build Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The full visual model of THE Mechazilla at the pad: measured lattice, carriage, both chopsticks, QD arm, square mount, flame
trench and lights, read as the real structure from the catch outside view (50-500 m) and the pad (eye 2 m, 10-100 m).

**Architecture:** The prototype's hybrid stays: lattice instanced by `LatticeBuilder` from `config/structures/starbase_olp2.toml`, modules from
headless Blender generators into git-ignored `data/models/starbase_olp2/`, `TowerView` the assembler, motion only from `tower_state()`.

**Tech Stack:** Blender 3.4 Python, Python tools / pytest, GDScript, MultiMesh, runtime `GLTFDocument`, Godot shaders.

**Spec:** `docs/superpowers/specs/2026-10-28-pad-b-model-design.md` (A, C-G, I, measures of success). Numbers:
`docs/subsystems/launch-structures-measurements.md`.

## Scope decision (Claude, 2026-10-31; overrules the spec's order)
The user decided that physics changes come in one plan after the models are accepted, and that from the catch only geometry and look
are taken now. So spec section B (`starbase_olp2` site, square mount physics, 27 m arms, carriage travel and QD state in the core, mission
switch, Pad A removal, catch retuning) is NOT in this plan; it moves to the physics plan.

Rule while the physics stays: **a surface the vehicle touches sits on the physics number, everything else is drawn as measured.**
| Part | Drawn now | Physics stays | Why |
|---|---|---|---|
| Mount deck | 20 m (physics); plan, faces, courses as measured | 20 m, measured 14.5 +-1.2 m | a stack stands on the deck: at 14.5 m it would hover 5.5 m |
| Arm tops, carriage | at `tower_state()`, 95 m at the catch | measured 133 +-4 m at the catch, 134 +-4 m parked | pins land on them |
| Arm length | 27 m, closed tips at the catch axis | 36 m | hinge-to-axis 24.5 +-2 m: the short arm reaches it |
| Tower face, height | width 12 m, roof deck at 144 +-5 m, lattice 139 m | box 12 / 146 m | within the error |
| QD arm | static, swung clear | no state | needs the core's `qd_arm_angle_rad` |
| Azimuth, distance | physics 315 deg, 30 m | measured 345 +-25, 32 +-3 m | low confidence |

These rows go into a "Physics deviations" table in `launch-structures.md`, input to the physics plan.

## Global Constraints
- NO change of `core/`, `bindings/`, `config/launch_sites.toml`, missions. Site stays `starbase_olp1`.
- No photo pixels in assets, no photo files in the repo, no traced drawings, no logos. Every number [E] with its method, [S] with URL.
- Real frames win, the STL shapes only; surface detail in maps, geometry for the silhouette ([[feedback-surface-detail-not-mesh]]).
- CC0 materials only, recorded in `tools/data_pipeline/sources.toml`, CREDITS regenerated.
- Without models: today's boxes and one log line. Deterministic generation, manifest hash.
- Colours: from the result of item 11 of `.superpowers/sdd/2026-10-31-starship-prototype/real-frames-brief.md` (the user reads the tower as black; the
  earlier pass read bare silver legs in direct sun and black braces). No tint to fake it: albedo [E] + the physical lighting.
- Anti-goals: shimmer, LOD pop, arm motion on an own clock, night lights changing the day scene.
- Budget (VR, headset `--perf-log`): tower + mount + lights <= 2.0 ms GPU median, <= 3.0 ms p99; pad gives way first (spec decision 16).
- Images to `screenshots/pad-b/`; one GPU run at a time; lines <= 120; commit own paths on `main`, push after each green task.

## Review Focus
1. Stack on the mount: engines and skirt seen from the pad at 2 m must not pass through the frustum's deck, ring or hoods (test in Task 3).
2. Booster caught: closed arms against the hull at the pins, 27 m arm, no visible gap or penetration (test in Task 2).
3. Lattice floors changed from 4.0 to 5.3 m: the far shader and merged tier still match (hand-over check in Task 1).
4. Terrain trench mask from orbit and at the seam with the far terrain LOD: no hole visible beyond the site (check in Task 3).
5. Day scene unchanged with lights present, < 0.5 % (probe in Task 4).

---

### Task 1: Measured lattice and tower body

**Files:** Modify: `config/structures/starbase_olp2.toml`, `tools/structures/detail_file.py`, `game/world/structures/lattice_builder.gd`,
`game/world/structures/tower_material.gd`, `game/shaders/tower_far.gdshader`, `tools/structures/tests/`, `game/tests/tower_view_test.gd`, `config/README.md`.

- [ ] Step 1: Tests first: lattice panels end at 139 m, roof deck 144 m, 26 floors of 5.3 m, leg 2.2 m, brace 0.5 m, plinth top 12.2 m, cladding
  12-79 m on the pad-side face, no pyramid (flat roof block + railing); silhouette top <= 148 m. Run, expect failures.
- [ ] Step 2: Apply the numbers; new tables `[tower.plinth]`, `[tower.cladding]` (solid plates with seams, one face), `[tower.roof]` (louvred block ~7 m,
  railing, mast if frames show it), stairs and cable trays as boxes. X-brace crossings ~7-8 m in the open part per the webcast pass; state which pitch the frames decide.
- [ ] Step 3: Material per item 11 result; refit the far shader's pitch and colours to the merged tier.
- [ ] Step 4: `pytest tools/structures/tests -q`, `tower_view_test`, `tower_handover_check --focal=both` (gates as in `launch-structures.md`) pass; PNG side by side with `f14` / Ship 42 frames; commit, push.

### Task 2: Carriage and both chopsticks

**Files:** Modify: `tools/structures/blender/gen_chopstick.py`, `game/world/tower_view.gd`, `config/structures/starbase_olp2.toml`. Create: `tools/structures/blender/gen_carriage.py`. Test: `game/tests/tower_view_test.gd`, `tools/structures/tests/`.

**Interfaces:** Produces glTF `carriage.gltf` (origin at the arm top plane, tower centre plane), `chopstick.gltf` (pivot = hinge, as today).

- [ ] Step 1: Tests first: both arm boxes replaced, `-across` mirrored, arm pose equal to `TowerGeometry::arm_direction` within 0.1 deg at open / standby / closed, carriage node height = `tower_state()` height + offset within 0.01 m; closed arm reaches the catch axis; caught-booster render shows the pins on the arm tops.
- [ ] Step 2: Chopstick: 27 m, depth 8.6 m with the inclined lower chord, rounded outer end, grated deck and rack strip (maps), tan catch strip, inner pads, railing; hydraulic actuators following the arm angle by geometry. Carriage: black frame across 13.5 m x 9 m, platforms, hinge frames at the corners.
- [ ] Step 3: Tower-side parts: booster-side platform at 86.8 m (reach ~12 m), QD arm `gen_qd_arm.py` (deck 122 m, truss bottom 104 m, reach 20 m) static and swung clear.
- [ ] Step 4: Tests pass; catch-pose PNGs against `f7_3060`, `f5_2405` (old hardware, shape of the arms only); `check_envelope.py` (Create: `tools/structures/check_envelope.py`, 0.10 / 0.05 m; the table's rows above are its only allowed exceptions, listed by name); commit, push.

### Task 3: Square mount, trench and tunnels

**Files:** Create: `tools/structures/blender/gen_mount.py`, `tools/structures/blender/gen_trench.py`. Modify: `game/world/tower_view.gd`, `config/structures/starbase_olp2.toml`, `game/shaders/terrain_earth.gdshader`, terrain node setup. Test: `game/tests/tower_view_test.gd`, new GPU check `game/tests/trench_mask_check.gd`.

- [ ] Step 1: Tests first: frustum base 44.6 x 40 m, top deck 18.7 m at the physics mount height, ring outer 10.4 m / opening 9 m with ~12 clamps; two exit ramps opposite, axis perpendicular to mount-tower, 33 m from the axis, 20 m wide; stack clearance at the skirt and bells.
- [ ] Step 2: Generators: panels in horizontal courses with seams as map, door arches, hoods, diagonal stair strip, long deluge wall ~5.5 x 55 m, tunnels with their own walls.
- [ ] Step 3: Terrain mask from the footprint polygon (shader discard near the site only, distance-limited). Physics ground stays flat (documented simplification).
- [ ] Step 4: `trench_mask_check` from 2 m, 500 m, 50 km, orbit: no hole beyond the footprint; pad PNGs against `f12_1860_full`, `f14_2095`; commit, push.

### Task 4: Lights

**Files:** Create: `tools/structures/blender/gen_lights.py`, `game/world/structures/tower_lights.gd`. Modify: `tower_view.gd`, detail file. Test: `game/tests/tower_lights_test.gd`.

- [ ] Step 1: Tests first: on between civil dusk and dawn (Sun elevation from the snapshot), obstruction lights always with the FAA flash rate [S], floodlit deck illuminance within 30 % of the fixture model, day luminance change < 0.5 %.
- [ ] Step 2: Floodlight banks, tower and obstruction, deck lights in physical units through the eye adaptation; max 4 shadowed [E]; far emissive points.
- [ ] Step 3: Tests pass; night and twilight PNGs; commit, push.

### Task 5: Evidence and acceptance state

**Files:** Modify: `docs/subsystems/launch-structures.md`, `docs/history/`, `docs/ansichten.md` if views change, `structure_models.gd` (remove `PROTOTYPE_SITE_ID`: every catch site, LC-39A keeps its own mount and no trench).

- [ ] Step 1: Tower proportions within 3 % or 0.3 m against the table; silhouette IoU >= 0.90 where a camera solve exists, otherwise proportion checks and say so.
- [ ] Step 2: Motion recorded in a catch: rendered arm and carriage equal to `tower_state()`; `tower_cost_check` before / after, 1x and 2x pixels.
- [ ] Step 3: `pytest`, `tower_view_test`, `tower_handover_check`, `smoke_test`; docs with the "Physics deviations" table; commit, push.
- [ ] **STOP:** images and numbers to the user; headset run by the user. The physics plan follows after both models are accepted.
