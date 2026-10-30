# Launch structures (Pad B tower and chopstick model)

State: prototype plan `docs/superpowers/plans/2026-10-31-pad-b-prototype.md`, tasks 1-3 done (draft detail file, Blender generator, game near range). Spec:
`docs/superpowers/specs/2026-10-28-pad-b-model-design.md`. The lattice and one chopstick render at `starbase_olp1`; material and distance hand-over follow (task 4).

## Model
- `config/structures/starbase_olp2.toml`: site-independent detail of THE Mechazilla (tower sections, member profiles,
  chopstick truss). Placement and physics numbers (arm length, hinge, tower height) stay in `launch_sites.toml`; the
  file never duplicates them. Keys: `config/README.md`.
- Every value is tagged `[E]` (own estimate with its method) or `[S]` (published, URL).
- Draft numbers: 12 sections x 12.04 m = 144.5 m, face width 12 m, pitch 8 m, leg box 1.6 m, brace H 0.7 m; chopstick
  length 26 m, width 3 m, depth 8.75 m, tube chords 0.65 m, diagonals 0.5 m.

## Game side (task 3)
- `StructureModels.load_for(site_id)` (`game/world/structures/structure_models.gd`): `{}` unless the site is the
  prototype's (`PROTOTYPE_SITE_ID = "starbase_olp1"`), the detail file exists and `data/models/starbase_olp2/` is not
  older than the detail file / generator scripts (file mtimes; the manifest's Blender hash cannot be reproduced in
  GDScript). Otherwise one log line per site and `TowerView` keeps its boxes. glTF is loaded at run time (`GLTFDocument`);
  the Blender axes (arm +X, Z up) become render axes once, in `BLENDER_TO_RENDER`.
- `LatticeBuilder.build` (`lattice_builder.gd`): per section one MultiMesh of 4 corner legs and one per face of braces
  (`x_brace`: 2 diagonals + 1 horizontal per panel; `k_brace`: K from the far leg's mid-height; the top section adds a
  cap horizontal); panels = round(height / pitch). Members cast no shadow; a per-section `shadow_proxy` box with the
  lattice shader casts. Footprint on the physics box's pad-side face and base; height 144.5 m vs the box's 146 m.
- `TowerView.build` frees its children first (no duplicates on a rebuild); the chopstick replaces the `+across` arm box
  only, its hinge is the pivot, so `update_state` is unchanged. Test: `game/tests/tower_view_test.gd` (headless; the dummy
  renderer keeps no MultiMesh transforms, so layout is read from `LatticeBuilder.face_members`).

## Standing decisions (from the spec)
- Rebuild from dimensions; photos only as references in git-ignored `screenshots/mechazilla/pad-b/`, no photo pixels,
  no traced drawings, no logos.
- Measurement is scripted: `tools/structures/measure/ratio.py` reads a points file (next to the image, git-ignored)
  and scales by the 9 m booster diameter [S]. Camera solve (`solvePnP`) and silhouette IoU: full build.
- Hybrid pipeline: lattice instanced by Godot from the file, non-repeating modules by headless Blender (glTF in
  git-ignored `data/models/starbase_olp2/`), boxes as fallback. Physics numbers are the master (envelope check 0.10 m /
  0.05 m, full build).
- Prototype: placed at `starbase_olp1`'s tower box; arm visual length 26 m vs. physics 36 m is a known mismatch.

## Validation
- `game/tests/tower_view_test.gd` (104 checks), `smoke_test.gd`.
- `python -m pytest tools/structures/tests -q`: required keys, sections sum to 144.5 m within 1 %, every value line
  tagged, loader raises on missing keys / bad units.

## Open points
- First render vs references (task 3): panels are far larger than the photographed tower's rows, no platforms,
  cabin, carriage or mount; plain steel until task 4.
- Single-view ratios in oblique photos (face width, depth) carry several percent error; leg, wall, pitch and truss
  pitch are estimates (not measurable from the references). The full build re-measures with a camera solve.
