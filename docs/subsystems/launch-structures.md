# Launch structures (Pad B tower and chopstick model)

State: prototype plan `docs/superpowers/plans/2026-10-31-pad-b-prototype.md`, tasks 1-4 done (draft detail file, Blender generator, game lattice, look correction, material, distance hand-over). Spec:
`docs/superpowers/specs/2026-10-28-pad-b-model-design.md`. The lattice and one chopstick render at `starbase_olp1`.

## Model
- `config/structures/starbase_olp2.toml`: site-independent detail of THE Mechazilla (tower sections, member profiles,
  chopstick truss). Placement and physics numbers (arm length, hinge, tower height) stay in `launch_sites.toml`; the
  file never duplicates them. Keys: `config/README.md`.
- Every value is tagged `[E]` (own estimate with its method) or `[S]` (published, URL).
- Draft numbers (look correction, task 4; scale 10.6 px/m from the 144.5 m tower photo): 12 sections x 12.04 m = 144.5 m,
  face width 12 m, brace row pitch 4 m (~36 rows), leg box 2.2 m (bright closed corner column), brace H 0.4 m, deck
  slabs 0.4 m thick at every row, 2.5 m inside the leg line (inset), dark column bundle 3.6 m, roof cap 0.6 m + 5 m pyramid frame, foot ring 3 m wide;
  chopstick length 26 m, width 3 m, depth 8.75 m, truss pitch 6.5 m, tube chords 1.0 m, diagonals 0.7 m, rails 0.9 m.
  Patterns: `x_brace` for the two lowest sections, `chevron` (Lambda per row) above.

## Game side (task 3)
- `StructureModels.load_for(site_id)` (`game/world/structures/structure_models.gd`): `{}` unless the site is the
  prototype's (`PROTOTYPE_SITE_ID = "starbase_olp1"`), the detail file exists and `data/models/starbase_olp2/` is not
  older than the detail file / generator scripts (file mtimes; the manifest's Blender hash cannot be reproduced in
  GDScript). Otherwise one log line per site and `TowerView` keeps its boxes. glTF is loaded at run time (`GLTFDocument`);
  the Blender axes (arm +X, Z up) become render axes once, in `BLENDER_TO_RENDER`.
- `LatticeBuilder.build` (`lattice_builder.gd`): per section one MultiMesh of 4 corner legs, one per face of braces
  (`x_brace`: 2 diagonals + 1 horizontal per panel; `k_brace`; `chevron`: Lambda; the top section adds a cap
  horizontal), one `decks` MultiMesh of unit boxes (a deck per row, the column bundle, roof cap, foot ring) and on the
  top section a `roof` MultiMesh (pyramid frame); panels = round(height / pitch). Members cast no shadow; a per-section
  `shadow_proxy` box with the lattice shader casts. Footprint on the physics box's pad-side face and base; height
  144.5 m vs the box's 146 m.
- Distance hand-over per section (visibility ranges, no fade): near `legs`/`face_n`/`decks`/`roof` up to
  `NEAR_END_M` = 300 m; `mid` (the same geometry merged into 2 surfaces, 2 draw calls) 300 .. `MID_END_M` = 1000 m;
  `far` box with `shaders/tower_far.gdshader` beyond (analytic chevron / deck / leg coverage of the measured pitch,
  filtered by the pixel footprint, alpha-blended). Why these ranges and why no dither: `tower_handover_check.gd` below.
- Material (`tower_material.gd`, `shaders/tower_steel.gdshader`): CC0 ambientCG Metal032 colour map (data/materials,
  `python tools/data_pipeline/fetch.py ambientcg_metal032`; missing: plain steel) triplanar in the member's own scaled
  frame + procedural grime per instance; decks dark; chopstick matte black (`black_paint`).
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
- `game/tests/tower_view_test.gd` (headless), `smoke_test.gd`.
- `game/tests/tower_handover_check.gd` (GPU, ~90 s): dolly 50 m .. 5 km (ratio 1.05): second difference of the mean
  luminance of the tower's rectangle <= 2 % in the windows -20 %/+25 % around both boundaries (measured 0.40 % at
  300 m, 0.79 % at 1000 m; the median elsewhere 0.14 %, the maximum 3.6 % at 52 m where the rectangle clips the frame);
  tiers shown alone agree within 0.3 % (members / merged, 250-480 m) and <= 1.75 % (merged / shader, 900-1300 m;
  0.6 % at 1000 m); shimmer (head turn in 0.25 px steps) of the hand-over over the unreduced geometry 0.83-1.01 at
  280-4000 m (<= 1.10; 280 / 330 m straddle the near / mid swap: 1.006 / 0.999); brace 1.04 px / leg 1.71 px wide where their range ends (60 deg, 900 px = 779 px focal length).
  Findings behind the design: Godot's dither fade of two tiers (60 m each side) was not complementary (rectangle 2.6 %
  brighter before and 3.6 % darker after the boundary) and a hashed alpha in the far shader flickered 20-60 % more
  than the geometry: hard swap of matched tiers and a blended coverage instead; `blur` / colours of the far shader are
  fitted to the merged tier.
- `python -m pytest tools/structures/tests -q`: required keys, sections sum to 144.5 m within 1 %, every value line
  tagged, loader raises on missing keys / bad units.

## Prototype evidence (task 5, `screenshots/pad-b/t5_*.png`, `tools/structures/measure/render_views.sh`)
- Look pass: narrower decks (2.5 m inset), thinner chevrons (0.4 m), darker core; seen diagonally the tower now reads as
  an open lattice with bright corner columns and a dark interior (`t5_look2.png`); face-on it is still paler than the photo.
- Desktop GPU frame time (RTX 4090, 1600x900, 120 frames, median / mean, ms) boxes -> models: catch view 1.34 / 1.53 ->
  1.35 / 1.55, 1 km 0.81 / 0.94 -> 0.82 / 0.94, pad 2 m 0.33 / 0.37 -> 0.35 / 0.40: no measurable cost. Headset run open.
- Not verified: the tower's shadow on the ground (the proxy casts, but no tower shadow shows in `t5_shadow.png`: probably
  beyond the sun shadow range or too faint to read), texture pop at 300 m (only luminance checked).

## Open points
- Render vs the 2025-08 tower photo (task 4, `screenshots/pad-b/t4_*.png`): rows, chevrons, decks, bright corner
  columns, dark core, roof frame and black chopstick now read like the tower. Still different: no cabin, stairs,
  cable trays, no carriage / mount pyramid under the arm (the old mount cylinder), the -across arm is a box, the
  steel is brushed grey without the photo's weathering, the photo's tower is wider relative to its height (face width
  and tower height are physics numbers), grime is not tuned.
- The far shader's `blur`, `alpha_scale` and colours are fitted to the merged tier at 900-1300 m for one sun / sky
  condition; other light may show a small step (< 2 % luminance measured).
- Single-view ratios in oblique photos (face width, depth) carry several percent error; leg, wall, pitch and truss
  pitch are estimates (not measurable from the references). The full build re-measures with a camera solve.
