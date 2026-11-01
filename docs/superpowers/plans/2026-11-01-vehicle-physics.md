# Vehicle Physics From The Implemented Geometry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The flap, grid fin and drag physics of Starship and Super Heavy follow the drawn geometry; ascent and booster descent drag are derived and compared with the known values; ship entry drag is re-determined.

**Architecture:** A Python derivation tool (`tools/aero_derive/`) reads the built models and detail file, computes areas and drag with documented engineering methods and an error for each number. The core keeps its existing aero code paths; only config values and the flap's hinge geometry change. Campaigns run last, as the gate.

**Tech Stack:** Python + numpy (tool), C++20 core, TOML / CSV config, Catch2, Godot headless checks.

**Spec:** the user's ruling of 2026-11-01, quoted:
"Die Physik muss für Flaps und Gridfins folgen. Beim Luftwiderstand beim Aufstieg und beim Abstieg des Boosters sollte sich der hergeleitete Luftwiderstand vom implementierten Modell herleiten lassen und den bereits bekannten Werten gleichen. Der Luftwiderstand beim Wiedereintritt und in der Atmosphäre beim Starship muss anhand der neuen Geometrie und der Flaps neu festgelegt werden. Die Physik muss sich danach richten." and "nach der Luftwiderstandsberechnung für Aufstieg und Booster für Abstieg die Werte vergleichen und rückmelden, erst OK holen bevor weiter."

## Global Constraints
- **STOP after Task 2.** Report the comparison to the user. Tasks 3 onward start only after the user's OK.
- Tasks 1 and 2 change no physics value in `config/` and nothing in `core/`.
- Each derived number carries its method, source tag and error. No fit to make numbers agree; a disagreement is reported as it is.
- The ascent and booster descent tables are not re-fitted (user 2026-10-31): the derivation is a cross-check.
- No campaign before Task 6 (`feedback-no-tuning-before-vehicle-plan`). After a vehicle config change: `python tools/vehicle_report.py <vehicle-id>`.
- Run only affected test tags. Commit on `main`, push when green, by path, no `*.uid`.
- Images in `screenshots/vehicle-physics/`. Long output to files.
- Reference flight values come from `tests/validation/telemetry` and `tools/telemetry/*`; readings the physics cannot explain are not followed.

## Review Focus
1. Reference area: tables use pi/4 D^2 = 63.6 m^2; a mesh method must convert to it, and must not count hidden or internal faces (stacked ship, covers inside the hull).
2. Grid fin drag is added to the descent table by the vehicle model: the derived body value must be compared without the fins, the total with them, using 3 x 13.5 m^2 and the drawn gross area both.
3. Shadowing: tail-first Newtonian flow must not load the lee side, the interior of the engine bay or fins behind the skirt.
4. Power on / off columns: base drag differs; the descent table's columns are equal by decision and the ascent's are not.
5. Rarefaction: the core blends to 2.2 above Kn 0.01; compare continuum values only.

---

### Task 1: Secured areas and the drag derivation tool

**Files:**
- Create: `tools/aero_derive/geometry.py`, `tools/aero_derive/newtonian.py`, `tools/aero_derive/buildup.py`, `tools/aero_derive/tests/test_aero_derive.py`
- Read: `data/models/starship_block3/*.gltf` + manifest, `config/vehicles/detail/starship_block3.toml`, `tools/structures/blender/hull_profile.py`, `tools/structures/measure/gltf_flap_measure.py`

**Interfaces:**
- Produces: `geometry.load_vehicle(vehicle_id) -> Vehicle` with triangles per part in body coordinates (flaps at a given deflection, fins at a given deflection), `hull_wetted_area_m2(stage)`, `flap_area_m2(kind)`, `grid_fin_gross_area_m2()`, `grid_fin_open_fraction()`.
- Produces: `newtonian.axial_force_coefficient(tris, flow_dir_body, cp_max, a_ref_m2) -> (ca, cn)` with occlusion by ray test against all tris; `cp_max` = the normal-shock stagnation value at Mach (modified Newtonian).
- Produces: `buildup.skin_friction_cd(wetted_m2, length_m, mach, altitude_m, a_ref_m2)`, `buildup.nose_pressure_cd(mach, fineness)`, `buildup.base_cd(mach, base_area_m2, powered, a_ref_m2)`, `buildup.protuberance_cd(...)`. Each cites its source (Hoerner, Fluid-Dynamic Drag; Van Driest II or Sommer-Short for compressible friction; NASA / DATCOM where used) in its docstring, and returns its relative error too.

- [ ] Step 1: failing tests: sphere mesh `ca` at Mach 10 against the Newtonian 0.92 x cp_max / 2 relation within 2 %; a flat disc head on = cp_max; a cylinder side-on (per D x L) within 2 % of 4/3 x cp_max / 2 ... shadowed back faces carry no load; flat plate friction at Re 1e8 Mach 0 against Schlichting within 3 %.
- [ ] Step 2: run them, they fail.
- [ ] Step 3: implement.
- [ ] Step 4: run `python -m pytest tools/aero_derive -q`, all pass.
- [ ] Step 5: table of secured areas, written by the tool: aft flap, forward flap, grid fin gross and open, wetted hull booster and ship, against `area_m2` and spans in `config/vehicles/starship_block3.toml`, each with source and error. Fin outline, cell pitch: `docs/subsystems/vehicle-models.md`.
- [ ] Step 6: commit `feat(tools): aero derivation from the built vehicle models`.

### Task 2: Derive ascent and booster descent drag, compare — STOP

**Files:**
- Create: `tools/aero_derive/derive_ascent_descent.py`, `docs/subsystems/aero-derivation.md`; add to `docs/subsystems.md`
- Test: `tools/aero_derive/tests/test_derive_outputs.py`

**Interfaces:**
- Consumes: Task 1.
- Produces: `screenshots/vehicle-physics/drag_compare.csv` and a plot per case; command `python tools/aero_derive/derive_ascent_descent.py`.

- [ ] Step 1: Ascent (stack, nose first, zero alpha), Mach 0.3, 0.6, 0.8, 0.95, 1.05, 1.2, 1.5, 2, 3, 5, 8, 15, 25; power on and off: friction along the flight's altitude / Mach history (from `flight14` telemetry), nose wave drag, base, hot-stage truss, fins and flaps as protuberances, strakes. Above Mach 3 also the Newtonian mesh value. Compare with `config/aero/starship_stack_ascent.csv` and with the drag implied by flights where available: `tools/telemetry/peak_dynamic_pressure.py`, `late_ascent_fit.py` (state if the telemetry cannot resolve Cd).
- [ ] Step 2: Booster descent (tail first, alone, fins at 0): body and fins separately, same Mach list. Subsonic: blunt body, Hoerner. Transonic: from Hoerner's blunt-face trend, flagged as the least certain. Supersonic and up: Newtonian mesh + base. Compare with `config/aero/super_heavy_descent.csv`, the raw `tools/telemetry/drag_estimate.py` bins of flights 13 and 14 (bin scatter +-10-15 %), and `grid_fin.csv` `ca_axial`. Show the total ballistic coefficient beta against the telemetry's at 261 t, since this is what the flight actually measured.
- [ ] Step 3: test: the script's rows contain every Mach, every derived value has an error, and the comparison is finite.
- [ ] Step 4: `docs/subsystems/aero-derivation.md`: method, table, verdict per Mach band: within the combined error / outside, with the reason if known. Commit `docs(aero): ascent and booster descent drag derived from the implemented model`, push.
- [ ] Step 5: **STOP.** Report to the user: the full tables, verdicts, sensitivities (fin area 13.5 vs 12.63; stowed flaps on or off). Wait for the OK. Record in the ledger and memory.

---
Tasks 3-7 are the plan as of the checkpoint; refine them with the user's answer before dispatching.

### Task 3: Flaps follow the geometry
User 2026-11-01 (flight 14 frame at T+49 s): the ship starts with all flaps fully out, d = 90, which is their rest pose, standing smooth in the flight axis. Drawing check 2026-11-01: incidence 0.0000 deg for all four at d = 90. Change: rest pose d = 90 at start, on the pad, in ascent and in coast, in physics and view; pin a test (panel normal perpendicular to the body axis at d = 90, forward and aft). Check the clearance to the arms and the QD arm on the mount and at the catch at d = 90.
`config/vehicles/starship_block3.toml` (and `starship_hls.toml`, tanker by inheritance): aft 36.5 m^2 span 3.69, forward 12.37 m^2, hinge stations 42.28-47.75. Add optional `hinge_lean_deg` and `hinge_radius_aft_m` to `[[stage.flap]]`, read in `core/src/vehicle` and used in `core/src/aero/flap.cpp` for span direction, normal and application point (centroid of the real planform, not mid-span x mid-station); same function in `attitude_controller.cpp`. Unit tests in `core/tests/test_flaps.cpp` `[aero][flap]`. The game's manifest and physics now agree. Forward RCS pod station 46 stands beside the hinge: move it to its drawn-free position or report (user decision). Lean 0 reproduces existing results to 1e-12 for other vehicles.

### Task 4: Grid fins follow the geometry
`config/vehicles/starship_block3.toml`: `area_m2` 12.63 [E], `radius_m` 7.19, `grid_fin.csv` reviewed against the open fraction; `[gnc][gridfin]`, `[aero][grid_fins]`, `[gnc][booster][validation]`. Descent table's fin share re-subtracted with the new area, so the total against telemetry stays (no re-fit of the total).

### Task 5: Ship entry and atmospheric drag re-determined
Mesh-based derivation for the ship alone over Mach, alpha 40-140, flap deflections: hull + flaps, against `ship_crossflow.csv` (which includes flap roots today), new `nose_length_m` 13.4, `nose_radius_m` from the drawn nose; crossflow and alpha tables re-determined. Replay `[ship][entry][validation]` on flights 12-14 (speed rms < 0.05, low rms < 0.10) as the check; `[flap]`, `[descent]`, `entry_aero_table`.

### Task 6: Campaign gate
Run in the background, one tag at a time, stop on the first sign of systematic failure: `[gnc][booster][validation]`, `[ship][catch][validation]`, `[ship][mission][validation]`, `[ship][guidance][validation]`, `[gnc][validation][ascent][slow]`, then the red list in `docs/known-issues.md` (lunar direct, `tanker_flight`, refuelling digest, ship case 20, booster 400 m abort, ship abort tower clearance, wide run). Final arm geometry and final aero fly here for the first time. Findings go to fix tasks.

### Task 7: Final review
Opus final review on the whole diff; docs moved to history; `vehicle_report` for every changed vehicle; plan archived; memory updated. Ask at the plan's start whether depot, tanker and HLS get models.
