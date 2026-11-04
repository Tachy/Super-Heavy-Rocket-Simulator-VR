# Depot, Tanker, HLS Detail Models Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Own rebuilt detail models of the tanker (Block 3 ship + dorsal probes), the depot (finless insulated hull + drogues) and the HLS (legs, landing thrusters, solar band, elevator, nose port), also on their boosters, in the existing vehicle-model pipeline.

**Architecture:** Physics config stays the master; the Block 3 ship generators gain variant switches read from `config/vehicles/detail/<id>.toml`; dorsal hardware and HLS parts are new generators; headless Blender writes glTF + manifest into git-ignored `data/models/<id>/`; `vehicle_models.gd` / `vehicle_view.gd` load and pose from the snapshot. Read `docs/subsystems/vehicle-models.md` first.

**Tech Stack:** Blender Python, Python tools, GDScript, Godot shaders; C++ core / bindings only where a config value or a snapshot field is missing.

**Spec:** `docs/superpowers/specs/2026-11-04-depot-tanker-hls-models-design.md`

## Global Constraints
- Own rebuild of everything; no third-party mesh in the repo or the game. A reference's terms are checked before download; references stay in git-ignored `screenshots/depot_tanker_hls_ref/`.
- No vehicle config or physics change from references without the user's decision (user 2026-11-04): a reference that disagrees with the config becomes a proposal (config value, reference value, source, affected campaigns) for the user; until the user decides, the config values are drawn. With no photos of these vehicles, the config values are the definition (gaps marked [E]). Only a change the user approves is applied, tagged, and goes through before / after runs.
- Drawn probe tips / drogue mouths, connector faces, leg feet and thruster exits coincide with the config points the physics uses (tolerance 0.01 m unless a task says otherwise).
- Block 3 ship / booster output byte-identical (manifest hash test `test_starship_hash_is_the_one_of_the_fixed_generator_list` and the deterministic position / UV / index compare).
- Simple hulls, detail by texture and shader; geometry for the silhouette, legs, thrusters, docking hardware, elevator.
- No logos, trademark lettering, flag or agency markings; display names from config.
- Motion only from the snapshot, except the demo. No physics in GDScript; `core` gets no Godot dependency.
- Ship aero and the reds in `docs/known-issues.md` stay untouched. Limits are the existing tests' tolerances; no test weakened. A config correction that turns a campaign red: fix the planner so the flight works (user 2026-11-03); an unplain break: stop and report.
- The plan does not wait for verdicts: renders to `screenshots/depot_tanker_hls/` and to the user after each vehicle; remarks worked in as they arrive.
- Only affected test tags (`-Fast -TestFilter` per change, the same with `[slow]` before a push); commit by path on `main`, push when green; German to the user, English in the repo; update `docs/subsystems/vehicle-models.md` with each task; `python tools/vehicle_report.py <id>` after each vehicle config change.

## Review Focus
1. Taken-over stages: the tanker's and the HLS-launch's booster show the Block 3 booster, the tanker ship shows the tanker model (not Block 3), the HLS-launch's upper stage the HLS model; after separation each part keeps its own. Test in Task 2.
2. A missing or stale model set for one of the four new vehicle ids, or no Blender: one log line and the cylinders, the other vehicles unaffected. Test in Task 2.
3. Docked pair geometry: tanker or HLS under the depot in the sim's docked pose - probe tips inside the drogue mouths, connector faces meeting, no hull or leg interpenetration (the HLS legs at the depot: r3's clearance). Tests in Tasks 4 and 6.
4. HLS on a slope: each foot on its contact point at the leg's stroke, also with legs at different compression, no foot floating or sunk by more than 0.02 m. Test in Task 6.
5. The depot and the HLS show no tiles, flaps, chine, or catch pins from any angle, including the far / LOD stand-ins. Test in Task 2 (variant switches) and the renders of Tasks 4 and 5.

---

### Task 1: References and baseline

**Files:**
- Modify: `screenshots/depot_tanker_hls_ref/INDEX.md` (git-ignored)
- Create: `.superpowers/sdd/2026-11-04-depot-tanker-hls-models/baseline.md`

- [ ] **Step 1:** Flight 12-14 webcast frames (`data/webcasts/`, ffmpeg path in memory / `docs/setup.md`) and photos at the URLs in `INDEX.md` (open in a browser, measure by scaling, do not store unclear-licence images): V3 docking drogues and propellant ports (count, placement nose vs dorsal / leeward, size), payload door presence on V3; HLS renders and mockup photos for legs, solar band extent, windows, airlock / elevator track and basket, landing-thruster ports. Each finding with source, method, [S]/[E] in `INDEX.md` sections 5-6. Frame crops to `screenshots/depot_tanker_hls_ref/frames/`.
- [ ] **Step 2:** For each config value a finding contradicts (`[[stage.docking_port]]`, `[[stage.transfer_connector]]`, `[stage.landing_legs]`, landing-thruster engine group stations, RCS pods), list config value, measured value, source, affected campaigns in `baseline.md` under "Proposals for the user"; the orchestrator puts them to the user. Nothing is applied without the user's decision.
- [ ] **Step 3:** Baseline on the current tree, background, outputs to files, every result read: `[refuelling]`, `[docking]`, `[prop_transfer]`, `[campaign]`, `[campaign_flown]`, `[hls]`, `[lunar]`, `[legs]`, each also with `[slow]`. Table per case: value, limit, pass.
- [ ] **Step 4:** Commit nothing in the repo except notes the ledger needs; report.

### Task 2: Pipeline: variants and taken-over stages

**Files:**
- Modify: `tools/structures/blender/generators.toml`, `tools/structures/vehicle_spec.py`, `tools/structures/build_models.py`, `tools/structures/blender/gen_hull.py` (and `hull_textures.py` / `hull_tiles.py` only where the switches need them), `game/vehicle/vehicle_models.gd`, `game/vehicle/vehicle_view.gd`, `game/tests/vehicle_view_test.gd`, `config/README.md`
- Create: `config/vehicles/detail/starship_tanker.toml`, `starship_depot.toml`, `starship_hls.toml` (stubs with the variant table)
- Test: `tools/structures/tests/test_ship_variants.py`

**Interfaces:**
- Produces: detail-file table `[variant]` with booleans `tiles`, `flaps`, `chine`, `catch_pins`, `payload_door`, and string `skin` (`"steel"` | `"insulated"` | `"white"`); `vehicle_spec.load_starship(vehicle_id)` resolves a taken-over stage (`from = "<v>:<s>"`) to the source stage's physics plus the variant table; `generators.toml` entries `starship_tanker`, `starship_depot`, `starship_hls` (kind `starship`); `starship_hls_launch` reuses models by stage: `VehicleModels.model_set_for_stage(vehicle_id: String, stage_id: String) -> String` returns the vehicle id whose model set draws that stage (`starship_tanker`/`booster` -> `starship_block3`, `starship_hls_launch`/`booster` -> `starship_block3`, `starship_hls_launch`/`hls` -> `starship_hls`, otherwise the vehicle itself).
- [ ] **Step 1:** Failing Python tests: `test_block3_variant_defaults_reproduce_block3` (no `[variant]` = all true, skin steel, hash unchanged); `test_depot_variant_has_no_tile_flap_chine_pin_nodes` (spec for `starship_depot` lists none of `flap_*`, `chine`, `catch_pins`, tile material); `test_tanker_ship_resolves_block3_physics_with_tanker_tanks`.
- [ ] **Step 2:** Failing `vehicle_view_test.gd` cases: the stage-to-model-set mapping above for each of the four vehicles before and after separation; each new id with no manifest or a stale one gives `{}`, exactly one log line, cylinders, and leaves a loaded `starship_block3` set untouched.
- [ ] **Step 3:** Implement; `python tools/structures/build_models.py --vehicle starship_block3 --force` byte-identical; `python -m pytest tools/structures/tests -v`; Godot `vehicle_view_test`, `smoke_test`. Commit, push.

### Task 3: Tanker: dorsal probes and connectors

**Files:**
- Create: `tools/structures/blender/gen_dorsal_hardware.py`, `game/vehicle/probe_drogue_view.gd`, `game/tests/probe_drogue_test.gd`
- Modify: `config/vehicles/detail/starship_tanker.toml` (probe / connector drawing table), `config/vehicles/starship_tanker.toml` only for Task 1 corrections, `game/vehicle/vehicle_view.gd`, `generators.toml`, `vehicle_models.gd`
- Test: `tools/structures/tests/test_dorsal_hardware.py`, `game/tests/probe_drogue_test.gd`

**Interfaces:**
- Consumes: `[[stage.docking_port]]` (`mechanism = "probe_drogue"`, `position_m`, `axis`, `up`, `elements_m`, `soft_capture`), `[[stage.transfer_connector]]` (`position_m`, `line_diameter_m`); snapshot `docked` and the refuelling block (`bindings/src/flight_snapshot_refuelling.cpp`).
- Produces: `dorsal_hardware.gltf` per vehicle with nodes `probe_<i>` / `drogue_<i>` (origin at the config element, axis +interface normal), `connector_lox`, `connector_ch4`, `dock_fairing`; manifest `dorsal_elements_m`, `probe_stroke_m`; `ProbeDrogueView.set_state(extension: float, captured: bool) -> void` (extension 0..1 of `probe_stroke_m` [E]), the only entry for docking visuals. The view drives it from the snapshot: extended during the final approach / docked phase, stowed otherwise (choose the snapshot fields; if none tell the phase, add one in the bindings, no logic in GDScript beyond the mapping).
- [ ] **Step 1:** Only if the user approved a Task 1 proposal for the tanker: apply it, `vehicle_report.py starship_tanker`, re-run `[refuelling]`, `[docking]`, `[prop_transfer]` with `[slow]`; table old, new, limit.
- [ ] **Step 2:** Failing tests: `test_probe_tips_on_config_elements` (extended tip within 0.01 m of each `elements_m` point); `probe_drogue_test.gd`: `set_state(0, false)` keeps every probe inside its stowed envelope (below the hull radius + fairing height), `set_state(1, true)` puts tips on the elements; connector faces on `position_m` within 0.01 m.
- [ ] **Step 3:** Implement; green.
- [ ] **Step 4:** Renders in `screenshots/depot_tanker_hls/tanker_*` via `vehicle_look_shots.gd` (side, dorsal close, probes stowed / extended, on the booster); compare with Task 1 frames; send. Commit, push.

### Task 4: Depot and the docked pair

**Files:**
- Modify: `config/vehicles/detail/starship_depot.toml`, `config/vehicles/starship_depot.toml` only for Task 1 corrections, `tools/structures/hull_textures.py` (insulated skin), `game/shaders/vehicle_hull.gdshader` only if the skin needs a parameter, `gen_dorsal_hardware.py` (drogues)
- Create: `game/tests/refuel_dock_demo.gd`
- Test: `game/tests/probe_drogue_test.gd`, `tools/structures/tests/test_dorsal_hardware.py`

**Interfaces:**
- Consumes: Task 2 variant (`skin = "insulated"`, no tiles / flaps / chine / pins), Task 3 `dorsal_hardware.gltf` and `ProbeDrogueView`.
- Produces: depot model with drogues (`drogue_<i>`, mouth plane at the config element), RCS pods at the config stations; the demo scene places a tanker in the depot's docked pose from the configs and runs approach, extension, capture through `set_state` only.
- [ ] **Step 1:** Only user-approved Task 1 proposals for the depot (mirror an approved tanker change: the configs say "mirror images"); same re-runs as Task 3 Step 1 if anything changed.
- [ ] **Step 2:** Failing tests: `test_drogue_mouths_on_config_elements`; docked pair in `probe_drogue_test.gd`: with tanker and depot posed as the sim's docked state (port planes coincident, axes opposite, `up` aligned), each extended probe tip lies inside its drogue's mouth cone and no hull mesh vertex of one ship lies inside the other's hull radius; same for the HLS once Task 6 exists (marked pending until then).
- [ ] **Step 3:** Implement, insulated skin as a baked map (light, slightly quilted MLI look by texture, no geometry); green; Block 3 still byte-identical.
- [ ] **Step 4:** Renders `depot_*`: alone in sunlight and shade, docked pair, demo phases; send. Commit, push.

### Task 5: HLS hull: skin, solar band, windows, nose port, landing thrusters

**Files:**
- Create: `tools/structures/blender/gen_hls_parts.py`
- Modify: `config/vehicles/detail/starship_hls.toml`, `config/vehicles/starship_hls.toml` only for Task 1 corrections, `hull_textures.py` (white skin, solar band, window slots by texture where flat), `vehicle_view.gd` (thruster plumes at the drawn exits), `generators.toml`, `vehicle_models.gd`
- Test: `tools/structures/tests/test_hls_parts.py`, `game/tests/vehicle_view_test.gd`

**Interfaces:**
- Consumes: the HLS landing-thruster engine group(s) in `starship_hls.toml` (positions, directions, count), `[[stage.docking_port]]` entries (nose port if configured; otherwise the drawn nose port is marked visual-only [E] and noted as an open point, no physics added).
- Produces: `hls_parts.gltf` nodes `solar_band`, `windows`, `nose_port`, `thruster_<i>` (exit plane centre on the config engine position, axis on its direction); manifest `thruster_exit_m`.
- [ ] **Step 1:** Only user-approved Task 1 proposals (thruster stations, count): `vehicle_report.py starship_hls`, re-run `[hls]`, `[lunar]`, `[legs]` with `[slow]`; planner fixes if red; table.
- [ ] **Step 2:** Failing tests: `test_thruster_exits_on_config_positions` (0.01 m, axis within 0.5 deg); `vehicle_view_test.gd`: a firing landing thruster's plume starts at `thruster_exit_m`.
- [ ] **Step 3:** Implement; green.
- [ ] **Step 4:** Renders `hls_*` (side, nose, band, thrusters firing); send. Commit, push.

### Task 6: HLS legs, elevator, airlock; HLS on the Moon and on its booster

**Files:**
- Modify: `gen_hls_parts.py` (or a sibling `gen_hls_legs.py`), `config/vehicles/detail/starship_hls.toml`, `vehicle_view.gd`
- Test: `game/tests/vehicle_view_test.gd`, `game/tests/probe_drogue_test.gd` (HLS pair case), `tools/structures/tests/test_hls_parts.py`

**Interfaces:**
- Consumes: `[stage.landing_legs]` (`fixed`, `azimuths_deg`, `footprint_radius_m` 12.0, `foot_station_m` -1.5, `stroke_m` 0.6), snapshot `body_legs` (per-leg compression if present).
- Produces: nodes `leg_<i>` (strut set from the hull to the foot), `foot_<i>` (pad posed by the leg's compression), `airlock_hatch`, `elevator_track`, `elevator_basket` (fixed pose [E] at the top, separate nodes for a later sim value).
- [ ] **Step 1:** Failing tests: uncompressed foot centre within 0.01 m of (station -1.5, radius 12.0, leg azimuth); with compression c (0..`stroke_m`) the foot rises by c along the leg's stroke axis; on a 7 deg slope session snapshot each foot within 0.02 m of its contact point; the HLS-depot docked pair (Task 4 pending case) has no leg inside the depot's hull radius.
- [ ] **Step 2:** Implement; green.
- [ ] **Step 3:** Renders `hls_moon_*` from a lunar landing session (touchdown, standing, liftoff) and `hls_launch_*` on the booster at the pad; send. Commit, push.

### Task 6b: Deployable HLS legs (user decision 2026-11-04)

The user chose deployable legs over fixed ones: stowed flat against the hull at launch, in ascent and in space, deployed before the lunar landing (like Falcon 9's). Physics change.

**Files:**
- Modify: `config/vehicles/starship_hls.toml` (`[stage.landing_legs]`: `fixed = false` and the deployment keys the Falcon 9 legs use; stowed geometry [E]), the core's lunar landing sequence (deploy command before the terminal descent; read how `falcon9_block5` commands `deployment`), `gen_hls_parts.py` / leg generator (a rig that is a function of `deployment` only), `vehicle_view.gd`, `docs/subsystems/` of lunar landing and legs
- Test: core `[legs]`, `[hls]`, `[lunar]` cases; `game/tests/vehicle_view_test.gd`

- [ ] **Step 1:** Failing core test: an HLS lunar landing deploys the legs before touchdown (deployment 1 before the first foot contact) and the legs are stowed (deployment 0) on the pad, in ascent and in LLO; a lunar liftoff keeps them deployed. Then implement; `vehicle_report.py starship_hls`.
- [ ] **Step 2:** Re-run Task 1's `[hls]`, `[lunar]`, `[legs]` with `[slow]`: table old, new, limit; planner fixes if red (user 2026-11-03).
- [ ] **Step 3:** Failing view tests: at deployment 0 every leg part lies inside the hull envelope + its stowed thickness and clear of the booster (grid fins, hot-stage ring) on `starship_hls_launch`; at 1 the feet on (station -1.5, radius 12.0, azimuth) within 0.01 m; sampling 0..1 at 0.05 the foot moves at most 25 % of its path per sample. Implement; green.
- [ ] **Step 4:** Renders `hls_launch_*` stowed, `hls_moon_*` deploying and deployed; send. Commit, push.

### Task 6c: Lunar mission lands in daylight (user decision 2026-11-04)

The shipped `hls_lunar_llo` lands at 84.79 S in lunar night (Sun -1.45 deg against a terrain horizon of +2.85 deg; Task 6).

**Files:**
- Modify: the lunar mission file(s) under `config/missions/` (`epoch_utc` so the landing and liftoff fall in lunar daylight with the Sun at least ~5 deg above the terrain horizon at the site for the whole surface stay), core tests that pin the old epoch (their reference values re-derived, not loosened), `docs/known-issues.md` (remove the lunar-night entry)
- Create: a reproducible Sun-elevation scan script under `tools/` (site, epoch range, terrain horizon) with its output in the report

- [ ] **Step 1:** Scan with the script; choose the epoch; state Sun elevation at landing and liftoff.
- [ ] **Step 2:** Change the epoch; re-run `[lunar]`, `[hls]`, `[campaign]` with `[slow]`: table old, new, limit; a pinned value that changes is re-derived from the new epoch and stated; planner fixes if red.
- [ ] **Step 3:** Moon renders from the shipped mission (no scratch copy, no `--force_sun`); send. Commit, push.

### Task 7: Cost, docs, final

**Files:**
- Modify: `game/tests/vehicle_cost_check.gd`, `docs/subsystems/vehicle-models.md`, `docs/subsystems/` files of refuelling / lunar if a config changed, `docs/ansichten.md` if a view changes, `docs/known-issues.md` for open gaps

- [ ] **Step 1:** Cost with GPU: depot + tanker docked, HLS on the Moon, within the prototype's budget of 1.0 ms desktop GPU per view [E]; over budget: reduce geometry, stated.
- [ ] **Step 2:** Godot `smoke_test`, `vehicle_view_test`, `probe_drogue_test`; Task 1 tags once more with `[slow]`; table old, new, limit in `final-values.md`.
- [ ] **Step 3:** Final review over the whole plan; fix round.
- [ ] **Step 4:** Overview renders for the user's overall verdict. Ledger and memory, archive spec and plan, commit, push; ask for `/clear`.
