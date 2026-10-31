# Starship + Super Heavy Prototype Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Detail-model prototypes of Super Heavy and Starship Block 3 in the Pad B pipeline, for the user's verdict. Ends at a STOP.

**Architecture:** Per-vehicle detail file for what the physics does not need; headless Blender generates hull, flaps, grid fins and
engines as glTF into git-ignored `data/models/`, placed from the vehicle config's numbers. `vehicle_view.gd` swaps its cylinders
for the models, and moves flaps and fins from the snapshot. Tiles are an analytic hex shader, not geometry.

**Tech Stack:** Blender 3.4 Python, GDScript, Godot spatial shader via `LitAerial`.

**Spec:** none approved. User order 2026-10-31 ("nimm Dir auch die Prototypen für Starship und den Booster vor"), inputs:
`.superpowers/sdd/2026-10-28-vehicle-models/user-decisions.md` (tiles as textures), `research-models.md` section 7 (read 7.1-7.3:
facts and the tile shader design), Pad B pipeline `docs/subsystems/launch-structures.md`. Choices are the controller's, listed
in `docs/subsystems/vehicle-models.md`, for the user's review.

## Global Constraints
- NO physics change: `config/vehicles/*.toml` and `core/` stay untouched. The hull is placed and sized from the config (length,
  diameter, nose, flap hinges / spans, fin stations, engine rings, belly azimuth); the detail file never duplicates these.
- Only `starship_block3` for now; depot, tanker, HLS and Falcon 9 keep today's view. Without models: today's cylinders + one log line.
- No photo pixels, photo files, logos, or trademark lettering. Numbers [E] with method, [S] with URL. References in git-ignored
  `screenshots/mechazilla/`; web lookups only of public pages, no stream downloads.
- Motion only from the snapshot; no own clock. Cost: both vehicles in the pad view add <= 1.0 ms desktop GPU [E]; tile shader is
  stereo-correct (a function of the surface only), anti-aliased with `fwidth`, no shimmer.
- Lines <= 120, English code; images to `screenshots/vehicles/`; commit own paths on `main`, push per task.

## Review Focus
1. Stage separation and catch: each stage's node keeps its own transform; a model never stays attached to the wrong stage.
2. Belly azimuth: tiles and flaps on the side the physics calls windward (`belly_azimuth_deg` 270); a wrong sign shows the shield on
   the lee side during entry.
3. Flap / fin sign: a positive physics deflection moves the model the way the physics means; zero is the rest pose.
4. Plumes stay at the engines' exit planes after the swap; the engines' gimbal is not faked.
5. Fallback and missing snapshot fields; other vehicles render unchanged.

---

### Task 1: Detail file and Blender hull, flaps, fins, engines
**Files:** Create `config/vehicles/detail/starship_block3.toml`, `tools/structures/blender/gen_hull.py`, `gen_flap.py`,
`gen_grid_fin.py`, `gen_engine.py`, tests in `tools/structures/tests/test_vehicle_models.py`, `docs/subsystems/vehicle-models.md`;
modify `tools/structures/build_models.py`, `docs/subsystems.md`.
**Interfaces:** Produces in `data/models/starship_block3/`: `booster.gltf`, `ship.gltf` (origin at the stage's station 0 as
`vehicle_view.gd` places it, axis along +Y of the config's station direction: read the view first and state it; UVs on the hull = (arc
metres, station metres) so that the shader needs no bake), `flap_fwd.gltf`, `flap_aft.gltf`, `grid_fin.gltf` (pivot on the hinge),
`raptor.gltf`, `rvac.gltf`.
- [ ] Tests: hull bounds = config length / diameter within 0.05 m; nose matches `nose_length_m`; generator deterministic; hinge at the
  origin; exit diameter = `config/engines/` value; triangles printed.
- [ ] Implement: lathe hull with ring welds as normal detail from a ring pitch [E]; chine and raceway swept; hot-stage dome vents;
  flaps, fins as the references show; bare Raptor 3 bells. Read Block 3 references; resolve the research's catch-pin check by
  reporting it (no physics change).
- [ ] Tests green, Blender preview PNGs read; commit `feat(tools): Starship and Super Heavy models from the vehicle config`, push.

### Task 2: In the game, driven by the snapshot
**Files:** Create `game/vehicle/vehicle_models.gd`, `game/tests/vehicle_view_test.gd`; modify `game/vehicle/vehicle_view.gd`.
**Interfaces:** `VehicleModels.load_for(vehicle_id) -> Dictionary` (`{}` = old view). Engine instances from the config rings
(MultiMesh per stage).
- [ ] Headless test: engine count per stage = config (33; 6), positions equal the ring formula; flap and fin pivots follow snapshot angles
  incl. sign at 3 states; hull node per stage; other vehicle: old node set; no models: old node set.
- [ ] Implement; stainless steel `LitAerial` material; plumes re-anchored to the exit planes.
- [ ] Test + smoke_test green; render on the pad; commit `feat(game): Starship and Super Heavy detail models`, push.

### Task 3: Tile shader, look pass, evidence, STOP
**Files:** Create `game/shaders/heat_shield_tiles.gdshaderinc` or as `LitAerial` needs; modify `vehicle_models.gd`, docs.
- [ ] Shader per research 7.3: axial hex cells from (arc, station), coverage from `belly_azimuth_deg` and the detail file; gap mask, bevel
  normal, per-tile variation from a hash; `fwidth` fade. Check script `tile_shimmer_check.gd`: shimmer ratio <= 1.10 between 5 and 500 m.
- [ ] Compare with the references, one look pass. Renders to `screenshots/vehicles/`: stack on pad, booster fins close, ship belly + flaps,
  chase view at altitude, stereo. Frame times: before (models renamed) / after.
- [ ] docs, tests, STOP report, commit `feat(game): procedural heat shield tiles and vehicle prototype state`, push. **STOP.** Full build only
  on the user's go.

### Final
Opus review over the plan's range, one fix round. The user's verdict and headset run remain open.
