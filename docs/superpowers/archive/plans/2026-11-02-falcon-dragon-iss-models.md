# Falcon 9, Crew Dragon, ISS Detail Models Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Own rebuilt detail models of Falcon 9 (with interstage, moving legs), Crew Dragon with trunk (hinged nose, moving docking mechanism) and the ISS (exact docking rings), in the Starship prototype's pipeline.

**Architecture:** Physics config stays the master; `config/vehicles/detail/<id>.toml` holds the rest; headless Blender generators write glTF and a manifest into git-ignored `data/models/<id>/`; `vehicle_models.gd` / `vehicle_view.gd` load and pose from the snapshot. Reference models are only measured by script. Read `docs/subsystems/vehicle-models.md` first.

**Tech Stack:** Blender Python, Python tools, GDScript, Godot shaders; C++ core only for config-driven values.

**Spec:** `docs/superpowers/archive/specs/2026-11-02-falcon-dragon-iss-models-design.md`

## Global Constraints
- Own rebuild of everything; no third-party mesh in the repo or the game. A reference's terms are checked before download; references stay in git-ignored `screenshots/falcon9_ref/`.
- Reality wins over config: a disagreeing config value is corrected, tagged [S]/[E]/[D], and goes through the before and after runs of its vehicle. Photos and real data win over reference models.
- Simple hulls, detail by texture and shader. Geometry only for the silhouette, moving parts, and docking rings and hooks.
- No logos, trademark lettering or photo pixels; display names from config.
- Motion only from the snapshot, except the docking demo. No physics in GDScript; `core` has no new Godot dependency.
- Starship models, ship aero and the reds in `docs/known-issues.md` stay untouched.
- Limits are the existing tests' tolerances. No test weakened. A validated result that breaks without a plain cause: stop and report.
- The plan does not wait for verdicts: renders to `screenshots/<topic>/` and to the user after each vehicle, remarks worked in as they arrive.
- Only affected test tags; commit by path on `main`, push when green; German to the user, English in the repo; update the subsystem file with each task; `python tools/vehicle_report.py <id>` after each vehicle config change.

## Review Focus
1. Stage separation and the stack: a Falcon 9 stage flying alone and the second stage + Dragon show the right models at the right place, with no model left on the wrong stage. Test in Task 4.
2. Trunk jettison and entry: the capsule alone with no trunk, the nose closed, the sim's state decides. Test in Task 5.
3. A missing or stale model set, or a missing Blender: one log line and today's cylinders for each new vehicle. Test in Task 2.
4. A leg at `deployment` 0, 1 and in between: stowed flush with the hull, deployed feet on the contact physics' points, no flip at mid stroke. Test in Task 3.
5. The docked pair: rings of Dragon and ISS coincide at both ports, and the mechanism's retracted pose stays inside the closed nose. Tests in Tasks 6, 8.

---

### Task 1: Measuring references and the Falcon 9 baseline

**Files:**
- Modify: `screenshots/falcon9_ref/INDEX.md` (git-ignored)
- Create: `tools/structures/measure/glb_measure.py`, `tools/structures/tests/test_glb_measure.py`
- Create: `.superpowers/sdd/2026-11-02-models/baseline.md`

**Interfaces:**
- Produces: `glb_measure.bbox(path, node_filter=None) -> dict` and `glb_measure.node_positions(path) -> dict[str, tuple]`, reading header and accessors with no execution of the file.

- [ ] **Step 1:** Search Sketchfab, Thingiverse, GrabCAD and similar for Falcon 9, Crew Dragon, trunk, IDSS / IDA. Record URL, licence and what can be measured; download only what the terms allow, each in its own new folder. None usable: say so; photos and published dimensions carry it.
- [ ] **Step 2:** Obtain the public IDSS Interface Definition Document and note ring diameters, petal and hook layout in `INDEX.md` with page numbers.
- [ ] **Step 3:** Test `test_bbox_of_a_unit_cube_glb` (a generated cube: extents 1.0 +- 1e-6), see it fail, implement, see it pass: `python -m pytest tools/structures/tests/test_glb_measure.py -v`.
- [ ] **Step 4:** Record the baseline with old values on the current tree, background, verbose files, every result read: `[falcon9]`, `[legs]`, `[asds]`, `[dragon]`, `[iss]`, `[docking]`, `[proximity]`, `[rendezvous]`, `[iss_mission]`, `[splashdown]`, each with `[slow]`.
- [ ] **Step 5:** Commit the tool by path.

### Task 2: Pipeline for several vehicles

**Files:**
- Modify: `game/vehicle/vehicle_models.gd`, `tools/structures/build_models.py`, `tools/structures/vehicle_spec.py`, `game/tests/vehicle_view_test.gd`
- Create: `tools/structures/blender/generators.toml` or an equivalent per-vehicle generator table, `config/vehicles/detail/falcon9_block5.toml`, `crew_dragon.toml`, `iss.toml` as stubs
- Test: `tools/structures/tests/`

**Interfaces:**
- Produces: `VehicleModels.MODEL_VEHICLE_IDS: PackedStringArray` replacing `PROTOTYPE_VEHICLE_ID` (ids `starship_block3`, `falcon9_block5`, `crew_dragon`, `iss`; update `tile_shimmer_check.gd`, `vehicle_cost_check.gd`); `build_models.py --vehicle <id>` for each, with its own generator list and manifest.

- [ ] **Step 1:** Failing `vehicle_view_test.gd` case: for `falcon9_block5` with no manifest `load_for` returns `{}` with exactly one log line and the view shows cylinders; a stale model set likewise.
- [ ] **Step 2:** Implement; confirm `starship_block3` output is byte-identical: manifest hash unchanged, `python tools/structures/build_models.py --vehicle starship_block3 --force`.
- [ ] **Step 3:** Run `python -m pytest tools/structures/tests -v`, Godot `vehicle_view_test`, `smoke_test`. Commit.

### Task 3: Falcon 9 first stage: length, fins, legs, model

**Files:**
- Modify: `config/vehicles/falcon9_block5.toml`, `config/vehicles/detail/falcon9_block5.toml`, `docs/subsystems/falcon9.md`, `docs/subsystems/vehicle-models.md`, `game/vehicle/vehicle_view.gd`
- Create: `tools/structures/blender/falcon9_*.py`
- Test: `game/tests/vehicle_view_test.gd`, `tools/structures/tests/test_falcon9_spec.py`

**Interfaces:**
- Consumes: Task 2's generator table; snapshot `body_legs` (`deployment` 0..1, `azimuths`, `foot_station_m`, `footprint_radius_m`).
- Produces: glTF nodes `hull`, `interstage`, `octaweb`, `merlin`, `grid_fin`, `leg` (a rig with the real folding mechanism: a function of deployment only); manifest `leg_foot_deployed_m`.

- [ ] **Step 1:** Physics: `length_m = 47.7` [S], tanks, engines, station 0 and masses stay; `[stage.grid_fins] station_m` to the measured place on the interstage [E]. Review CG and tank stations: change only with evidence, stated. Correct docs and comments that give 41.2 m as the stage. `vehicle_report.py falcon9_block5`.
- [ ] **Step 2:** Re-run Task 1's Falcon and Dragon tags. Table old, new, limit. Stop on an unexplained break. Check legs against photos too (span, foot station): reality wins, re-run `[legs]`, `[asds]`.
- [ ] **Step 3:** Failing test `test_interstage_top_at_stage_length`: drawn hull + interstage reach 47.7 m +- 0.01, with no physics key in the detail file.
- [ ] **Step 4:** Failing test in `vehicle_view_test.gd`: at `deployment` 0 the legs lie inside the hull envelope + their stowed thickness; at 1 each foot is within 0.05 m of (station `foot_station_m`, radius `footprint_radius_m`, leg azimuth); sampling 0..1 at 0.05 the foot moves at most 25 % of the stroke per sample (no flip).
- [ ] **Step 5:** Generators and view rig; tests green.
- [ ] **Step 6:** Renders in `screenshots/falcon9/` (side, base, legs at 0, 0.5, 1, fins) via `vehicle_look_shots.gd`; compare with photos in `falcon9_ref`; send to the user. Commit in logical commits, push.

### Task 4: Second stage and stack

**Files:**
- Modify: `config/vehicles/detail/falcon9_block5.toml`, `config/vehicles/falcon9_block5.toml` if references disagree, `game/vehicle/vehicle_view.gd`
- Create: `tools/structures/blender/falcon9_second_stage.py`
- Test: `game/tests/vehicle_view_test.gd`

**Interfaces:**
- Produces: nodes `hull`, `mvac`, plume re-anchor via manifest `exit_plane_x_m`.

- [ ] **Step 1:** Failing test: in a Falcon 9 stack, the second stage's base sits 47.7 m above the first stage's station 0, its MVac inside the interstage; after separation each part keeps its model: first stage alone, second stage alone, `vehicle_id` as `<vehicle>/<stage>`.
- [ ] **Step 2:** Implement; green. MVac exit diameter verified against a second source and config.
- [ ] **Step 3:** Renders of the full stack and separation against photos and webcast frames; send. Commit, push.

### Task 5: Crew Dragon with trunk and nose

**Files:**
- Modify: `config/vehicles/crew_dragon.toml` if references disagree (capsule 4.4 vs 4.5 m), `config/vehicles/detail/crew_dragon.toml`, `game/vehicle/vehicle_view.gd`, `docs/subsystems/crew-dragon.md`
- Create: `tools/structures/blender/dragon_*.py`
- Test: `game/tests/vehicle_view_test.gd`

**Interfaces:**
- Consumes: snapshot `nose_cone`, `nose_cone_open`.
- Produces: nodes `capsule`, `heat_shield`, `superdraco_pod`, `draco_pod`, `nose_cone` (hinge at the measured point), `trunk`, `trunk_array`, `trunk_radiator`; `VehicleView` animates the nose between its two states over `nose_open_time_s` in the detail file [E], a view easing of a binary state, no state of its own beyond that.

- [ ] **Step 1:** Config corrections with `vehicle_report.py crew_dragon` and Dragon tags, table old, new, limit.
- [ ] **Step 2:** Failing tests: closed nose angle 0 and open at the detail file's angle follow `nose_cone_open`; the nose never leaves its hinge; capsule alone after trunk separation shows no trunk; a Dragon on the Falcon 9 second stage stands at its config station.
- [ ] **Step 3:** Generators and view; green.
- [ ] **Step 4:** Renders in `screenshots/dragon/`, nose closed and open, side and front, against `D_*` photos; send. Commit, push.

### Task 6: Moving docking mechanism and demo

**Files:**
- Create: `tools/structures/blender/idss_mechanism.py`, `game/vehicle/docking_mechanism_view.gd`, `game/tests/docking_mechanism_demo.gd`, `game/tests/docking_mechanism_test.gd`
- Modify: `config/vehicles/detail/crew_dragon.toml`, `docs/subsystems/vehicle-models.md`

**Interfaces:**
- Consumes: IDSS dimensions from Task 1; port `position_m`, axis of `crew_dragon.toml`.
- Produces: `DockingMechanismView.set_state(ring_extension_m: float, ring_tilt_rad: Vector2, hook_closed: float) -> void`, the only entry for later docking visuals; nodes `ring`, `petal_<i>`, `guide_pin_<i>`, `hook_<i>` (12), `struts` following the ring. Extension and stroke values come from `docking_mechanism.hpp` parameters via config, not from new numbers.

- [ ] **Step 1:** Failing tests: `set_state(0, 0, 0)` keeps all parts inside the closed nose's envelope; full extension places the ring plane at the IDSS extension on the port axis within 0.005 m; 12 hooks at their documented angles; struts stay attached at the tilt limit of 4 deg.
- [ ] **Step 2:** Implement; green.
- [ ] **Step 3:** Demo scene: extend, contact, retract, close hooks; only through `set_state`.
- [ ] **Step 4:** Close renders per phase against references; send. Commit, push.

### Task 7: ISS measurement and port positions

**Files:**
- Modify: `config/vehicles/iss.toml`, `docs/subsystems/iss.md`, `config/vehicles/detail/iss.toml`
- Create: `tools/structures/measure/iss_measure.py`
- Test: `tools/structures/tests/test_iss_measure.py`

**Interfaces:**
- Consumes: `glb_measure` on `screenshots/falcon9_ref/models/iss/d_igoal/ISS_D_IGOAL.glb`.
- Produces: table of module, truss, radiator, array positions in metres, scale from the 109 m truss, written into the detail file with [D]; ports IDA-2 and IDA-3 positions.

- [ ] **Step 1:** Failing test `test_scale_from_truss`: measured truss span 109 m +- 1 after scaling.
- [ ] **Step 2:** Implement; measure; check node positions against the photos.
- [ ] **Step 3:** Port positions corrected in `iss.toml` if outside the measure's error: old and new. Note that `stations` in the config are from the aft end of the Russian segment. Run `[iss]`, `[docking]`, `[proximity]`, `[rendezvous]`, `[iss_mission]`: table old, new, limit. Commit, push.

### Task 8: ISS model and rings

**Files:**
- Create: `tools/structures/blender/iss_*.py`, `tools/structures/blender/idss_passive_ring.py`
- Modify: `game/vehicle/vehicle_view.gd`, `docs/subsystems/vehicle-models.md`
- Test: `game/tests/vehicle_view_test.gd`

**Interfaces:**
- Consumes: Task 7's table, IDSS dimensions, Task 6's ring dimensions.
- Produces: nodes `pressurised`, `truss`, `radiator_<i>`, `array_<i>` under separate fixed joint nodes `alpha_joint_<i>` / `beta_joint_<i>`, `pma_2`, `ida_2`, `pma_3`, `ida_3`.

- [ ] **Step 1:** Failing test: with Dragon's port on each ISS port (config data), the Dragon ring and passive ring planes coincide within 0.01 m, axes within 0.1 deg, clocking equal to the config's reference.
- [ ] **Step 2:** Generators, joints fixed at one pose [E] and stated; green.
- [ ] **Step 3:** Renders overall and each port close, with Dragon docked, against `I_*`, `D_Iss*` photos; send. Commit, push.

### Task 9: Cost, docs, final

**Files:**
- Modify: `game/tests/vehicle_cost_check.gd`, `docs/subsystems/vehicle-models.md`, `falcon9.md`, `crew-dragon.md`, `iss.md`, `docs/ansichten.md` if a view changes, `CREDITS` inputs only if something ships

- [ ] **Step 1:** Cost measurement with GPU: Falcon 9 on the pad, Dragon + ISS docked, within the prototype's budget of 1.0 ms desktop GPU per view [E]. Over budget: reduce geometry, no shader removal unstated.
- [ ] **Step 2:** Godot `smoke_test`, `vehicle_view_test`, `docking_mechanism_test`; all Task 1 tags once more; table old, new, limit in `final-values.md`.
- [ ] **Step 3:** Final review over the whole plan. Fix round.
- [ ] **Step 4:** Send overview renders for the user's overall verdict. Record state in ledger and memory, commit, push; ask for `/clear` only after the verdict.
