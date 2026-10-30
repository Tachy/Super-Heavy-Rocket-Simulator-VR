# Pad B Prototype Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The spec's section H prototype: the full-height Pad B lattice tower instanced from a draft detail file and one
Blender-made chopstick driven by `tower_state()`, rendered for the user's verdict. It ends at the spec's STOP.

**Architecture:** A site-independent detail file `config/structures/starbase_olp2.toml` drives both a headless Blender
generator (profile meshes + chopstick glTF into git-ignored `data/models/`) and a Godot `LatticeBuilder` (MultiMesh per
section and face, distance hand-over to the lattice shader). `TowerView` stays the assembler and falls back to today's boxes.

**Tech Stack:** Blender 3.4 Python (headless), Python tools, GDScript / MultiMesh / runtime `GLTFDocument`, Godot shaders.

**Spec:** `docs/superpowers/specs/2026-10-28-pad-b-model-design.md` (A, C, D, E, F, H; binding inputs:
`.superpowers/sdd/2026-10-28-vehicle-models/user-decisions.md`, `research-models.md`).

## Global Constraints
- Prototype only: NO physics or config change of `[site.tower]`, no `starbase_olp2` site, no catch or mission change, no
  lights, mount, trench, carriage or QD arm models; those are the full build, after the user's go.
- Placed at `starbase_olp1`'s tower box; the arm's visual length is Pad B's (~26 m [E]) although the physics arm stays 36 m:
  a known prototype mismatch, stated in the report. LC-39A keeps its boxes.
- No photo pixels in assets, no photo files in the repo, no traced drawings, no logos; references stay in git-ignored
  `screenshots/mechazilla/pad-b/`. Every number [E] with its method, [S] with its URL.
- `game/` text formats only; generated glTF in git-ignored `data/models/starbase_olp2/` with a manifest (input hash of config +
  scripts + Blender version); deterministic generation; without models: today's boxes and one log line.
- Blender: `C:\Program Files\Blender Foundation\Blender 3.4`, `blender -b --factory-startup -P <script> -- --config <toml> --out <dir>`.
- CC0 materials only (ambientCG / Poly Haven), recorded in `tools/data_pipeline/sources.toml`; CREDITS regenerated.
- Anti-goals: shimmer, LOD pop, a visible hand-over band; arm motion on any own clock. Starting hand-over ranges ~600 m / ~5 km
  [E]; a member never thinner than ~1.5 px before it fades; members cast no individual shadow, a per-section proxy casts.
- Lines <= 120, English code, German UI; images to `screenshots/pad-b/`; one GPU run at a time, 300 s timeout, foreign Godot may
  be stopped; commit own paths on `main`, push after each green task.

## Review Focus
1. Models missing or stale, Blender absent: the game runs with the boxes and logs once; tests do not fail on a fresh checkout.
2. Far from the origin / site jump: no jitter at 2 m eye height or 1 km (float32 relative to the tower node); nodes are freed on
   a site change and a re-entry does not duplicate them.
3. Arm at every state: open, standby, closed, with shock offset: the pivot stays on the physics hinge; the glTF's forward axis
   and mirror do not flip the truss on the other arm's side.
4. Stereo / VR: one MultiMesh path, no per-eye state; hand-over fade does not differ between the eyes.
5. A site other than `starbase_olp1` (LC-39A) renders unchanged.

---

### Task 1: Draft detail file from first measurements

**Files:**
- Create: `config/structures/starbase_olp2.toml`, `tools/structures/measure/ratio.py`, `tools/structures/detail_file.py`,
  `tools/structures/tests/test_detail_file.py`, `docs/subsystems/launch-structures.md`
- Modify: `docs/subsystems.md` (index line), `config/README.md` (detail files), `.gitignore` if `data/models` is not covered

**Interfaces:** Produces the TOML keys every later task reads: `[tower] sections` (array of `{height_m, face_width_m, pitch_m,
pattern}`), `[profiles.<name>] {kind = "h_beam"|"tube"|"angle"|"box", size_m, wall_m}`, per-section `leg_profile`,
`brace_profile`, `[chopstick] {length_m, width_m, depth_m, truss_pitch_m, rail_profile, chord_profile, brace_profile}`; each
number with a sibling comment `# [E] <method>`. `detail_file.py::load(path) -> dict` validates required keys and units.
- [ ] Step 1: tests: required keys present; section heights sum to the tower height 144.5 m [E] within 1 %; every value line
  carries a `[E]`/`[S]` tag; `load` raises on a missing key.
- [ ] Step 2: read the references named in `screenshots/mechazilla/pad-b/INDEX.md` (tower, chopstick, carriage views) and derive the
  draft numbers by `ratio.py` (points file next to the image, scale from the 9 m booster diameter [S]); a value that cannot
  be measured from any reference takes a stated estimate with its reason. Full camera solve / silhouette IoU: full build.
- [ ] Step 3: `python -m pytest tools/structures/tests -q` green; subsystem file created (state: prototype, standing decisions
  from the spec); commit `feat(tools): draft Pad B detail file from first measurements`, push.

### Task 2: Blender generator: profiles and chopstick

**Files:**
- Create: `tools/structures/blender/common.py`, `gen_profiles.py`, `gen_chopstick.py`, `tools/structures/build_models.py`,
  `tools/structures/tests/test_build_models.py`

**Interfaces:** Consumes Task 1. Produces `data/models/starbase_olp2/profiles.gltf` (named meshes `profile_<name>`, unit length
along +Z, origin at one end, beveled), `chopstick.gltf` (root node at the hinge, arm along +X, top rail plane at Z = 0, named
nodes `truss`, `rail`), `manifest.json`. `build_models.py [--force]` exit 0 and no work when the hash matches; exit 0 with a
message when Blender is missing.
- [ ] Step 1: tests (skip without Blender): two runs give byte-identical `.bin`; manifest changes when the config changes; chopstick
  bounds equal `length_m`, `width_m`, `depth_m` within 0.05 m; hinge at the origin; triangle count printed.
- [ ] Step 2: implement: chopstick from the detail numbers (triangular / box truss as the references show, top rails, tapered tip,
  bevels, UVs). Keep each profile low: a beveled member is instanced thousands of times (state triangles and the count).
- [ ] Step 3: tests green; one Blender preview PNG to `screenshots/pad-b/`, read and compare with the chopstick references; commit
  `feat(tools): headless Blender generator for tower profiles and the chopstick`, push.

### Task 3: Lattice and chopstick in the game

**Files:**
- Create: `game/world/structures/lattice_builder.gd`, `game/world/structures/structure_models.gd`,
  `game/tests/tower_view_test.gd`
- Modify: `game/world/tower_view.gd`, `game/shaders/lattice.gdshader`, `docs/subsystems/launch-structures.md`

**Interfaces:** Consumes Tasks 1-2. `StructureModels.load_for(site_id) -> Dictionary` (`{}` unless the site has a detail file and
fresh models; only `starbase_olp1` for now, a named constant with a comment), `LatticeBuilder.build(detail, profiles, material) ->
Node3D` (one `MultiMeshInstance3D` per section and face, local site frame, tower base at the box's base, face toward the pad on
the physics box face; shadow proxy per section). `TowerView.update_state` keeps driving pivots from `tower_state()`; the chopstick
replaces the box on the `+across` arm only, the other arm stays a box (spec H: one chopstick).
- [ ] Step 1: headless test: with models: the node tree has the sections' MultiMeshes, instance counts equal the pattern's
  (per section: legs + braces from pitch), total height = detail height within 1 %; the arm's pivot basis / position after
  `update_state({"opening_rad": x, "carriage_offset_m": y})` equals the box path's for 3 states. Without models or for LC-39A:
  the old node set. No duplicates after a second `build`.
- [ ] Step 2: implement; runtime load by `GLTFDocument` from `Paths.data_dir()/models/...`; material `LitAerial` from Task 4's hook
  (plain steel until then); near range only, hand-over in Task 4.
- [ ] Step 3: test + `smoke_test.gd` green; a first render from the pad; commit `feat(game): Pad B lattice and a chopstick from the detail
  file`, push.

### Task 4: Material and distance hand-over

**Files:**
- Modify: `lattice_builder.gd`, `game/shaders/lattice.gdshader`, the `LitAerial` shader / script as needed,
  `tools/data_pipeline/sources.toml`, data fetch entry for one CC0 steel set, `CREDITS.md` via its generator
- Test: `game/tests/tower_view_test.gd`, new `game/tests/tower_handover_check.gd` (GPU)

**Interfaces:** Consumes Task 3. `tower_handover_check.gd -- --out=<dir>` dollies 50 m -> 5 km and prints per-band luminance change.
- [ ] Step 1: check script first: per-frame luminance change of the tower pixels at each band <= 2 % beyond the dolly's own; shimmer
  ratio (still view, head turned) <= 1.10; no member below 1 px before its fade (spec's measures of success).
- [ ] Step 2: one CC0 steel set (licence checked and recorded, downloaded by the pipeline to `data/`; missing -> plain steel) + triplanar
  grime; near = members; mid = merged per-section mesh; far = lattice shader with the measured pitch; dithered visibility fades;
  per-section shadow proxy. Fix the ranges by the check; state them.
- [ ] Step 3: check + tests green, material licence in CREDITS; commit `feat(game): tower material and lattice distance hand-over`, push.

### Task 5: Prototype evidence and STOP

**Files:** Modify: `docs/subsystems/launch-structures.md`, `docs/ansichten.md` (only if the running prototype changes what is described)

- [ ] Step 1: renders via `render_check.gd` to `screenshots/pad-b/`: catch camera, pad at 2 m eye height, 1 km, arm open / standby /
  closed, a stereo run; read every PNG, compare with the references by eye (what reads real, what does not).
- [ ] Step 2: desktop frame times before (boxes) / after, same views: GPU median / mean, main thread; triangle and instance counts.
  The headset run (`Start-VR.cmd --perf-log`) needs the user and is listed as open.
- [ ] Step 3: `[structures]`-free: run tower_view_test, smoke_test, tower_handover_check; write the STOP report (images, numbers,
  mismatches, what the full build changes); commit `docs: Pad B prototype state`, push.
- [ ] **STOP.** No full build: images and frame times to the user; the full build starts only on the user's go.

### Final
Opus review over the plan's range, one fix round. The user's verdict and headset run remain open.
