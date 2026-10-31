# Launch structures (Pad B tower and chopstick model)

State: prototype plan `docs/superpowers/plans/2026-10-31-pad-b-prototype.md`, prototype built and reviewed (draft detail file, Blender generator, game lattice, look correction, material, distance hand-over; final review fixes done). Spec:
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
  top section a `roof` MultiMesh (pyramid frame); panels = round(height / pitch). Members cast no shadow, and there is no
  shadow proxy: the game's Sun (`world.gd` DirectionalLight) has no shadow maps and `terrain_earth.gdshader` is unshaded,
  so the tower casts no shadow on the ground today (a real one is its own decision: Sun shadows or a baked / analytic
  tower shadow on the terrain shader). Footprint on the physics box's pad-side face and base; height
  144.5 m vs the box's 146 m.
- Distance hand-over (visibility ranges, hard swap, no fade): near `legs`/`face_n`/`decks`/`roof` up to `NEAR_END_M` =
  300 m; `mid` (the same geometry merged into 2 surfaces, 2 draw calls per section, ~21.5k triangles) 300 ..
  `MID_END_M` = 5000 m; `far` box with `shaders/tower_far.gdshader` beyond (analytic chevron / deck / leg coverage of
  the measured pitch, filtered by the pixel footprint; coverage resolved by MSAA alpha-to-coverage, so the box stays in
  the OPAQUE pass and clouds in front of it cover it: an alpha-blended draw lands after `CloudEffect`, which runs
  before the transparent pass). Godot measures a visibility range from the centre of the instance's bounding box:
  `LatticeBuilder._share_range_centre` gives every tier node a `custom_aabb` centred on the tower's middle, so all
  sections and faces swap at one camera distance (before: sections at different heights and the faces swapped at
  different distances, a tower half in each tier). The far tier starts where the tower is 2 px (desktop, 779 px focal
  length) to 3.6 px (VR eye, 1500 px) wide; at 1 km, where it began before, it lost the truss at VR focal lengths.
  Nothing else of this plan draws in the transparent pass (steel / dark / chopstick materials have no alpha).
  Why these ranges and why no dither: `tower_handover_check.gd` below.
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
- `game/tests/tower_handover_check.gd` (GPU, a few minutes per focal length; `--focal=desktop|vr|both`, `--bands_only`,
  `--tiers_only`): run for the desktop view (60 deg / 900 px = 779 px focal length) and a VR-like one (1500 px, 33.4 deg).
  (0) Band scan: fine camera steps through each boundary (near / mid 0.9 .. 1.13 x 300 m, mid / far 0.97 .. 1.05 x
  5000 m); the tower's structure (mean 3x3 high-pass of the luminance in the strip between the legs) is measured in 7
  vertical windows, as built and with each neighbouring tier alone; every window that tells the tiers apart (> 12 %) must
  show the same tier. Red before the fix (`f_red.log`, boundary then at 1000 m): at x1.000 windows 0-3 showed the far
  tier and 4-6 the merged one at both focal lengths; now uniform in all 36 steps. (1) Dolly 50 m .. 9 km (ratio 1.05):
  second difference of the mean luminance of the tower's rectangle <= 2 % in the windows -20 %/+25 % around both
  boundaries: desktop 0.35 % at 300 m, 0.81 % at 5 km; VR 0.15 % / 0.66 %; the median elsewhere 0.15-0.21 %, the maximum
  3.6-4.8 % at ~50 m where the rectangle clips the frame. Tiers shown alone agree within 0.19-0.30 % (members / merged,
  250-480 m) and 0.03-1.58 % (merged / shader, 4.5-6.5 km); structure of members / merged within 2 %, merged / shader not
  compared (the strip is 1-2 px: a comparison needs >= 3 px half width). (2) Shimmer (head turn in 0.25 px steps) of the
  hand-over over the unreduced geometry 0.97-1.01 at 280-4000 m at both focal lengths (<= 1.10), 0.49-0.67 at 6 and 8 km
  (the far tier is calmer than the raw members). (3) Brace width where the near range ends: 1.04 px (desktop), 2.0 px
  (VR); the leg is 0.34 / 0.66 px where the merged mesh ends (reported only: the far shader takes over where the tower is
  2-4 px wide).
  Findings behind the design: Godot's dither fade of two tiers (60 m each side) was not complementary (rectangle 2.6 %
  brighter before and 3.6 % darker after the boundary) and a hashed alpha in the far shader flickered 20-60 % more
  than the geometry: hard swap of matched tiers and a coverage (alpha-to-coverage) instead; `alpha_scale` 0.85 and the
  colours of the far shader are fitted to the merged tier at 4.5-6.5 km (alpha 1.0 / 1.15 left the far tier up to 2.3 %
  brighter at VR focal length).
- `python -m pytest tools/structures/tests -q`: required keys, sections sum to 144.5 m within 1 %, every value line
  tagged, loader raises on missing keys / bad units / unknown pattern and checks the deck, core, roof and foot tables;
  `build_models.py` generates into a sibling `.new` directory and swaps it in only when every generator succeeded (a
  failed run leaves the old models and manifest).

## Prototype evidence (task 5, `screenshots/pad-b/t5_*.png`, `tools/structures/measure/render_views.sh`)
- Look pass: narrower decks (2.5 m inset), thinner chevrons (0.4 m), darker core; seen diagonally the tower now reads as
  an open lattice with bright corner columns and a dark interior (`t5_look2.png`); face-on it is still paler than the photo.
- Cost (`game/tests/tower_cost_check.gd`, RTX 4090, 1600x900, median of 120 frames, tower node shown vs hidden, 3 repeats
  each, repeat spread <= 0.003 ms GPU): GPU render time the tower adds 0.05 ms at the frame-filling close-up (60 m;
  0.40 vs 0.35 ms) and 0.12 ms at 4x the pixels (render scale 2: 0.79 vs 0.67 ms); 0.00-0.01 ms at 300 m, 1 km and 6 km
  (also at 4x pixels). CPU render time +0.06-0.09 ms. The tower adds 100 draw calls / 24.7k triangles at the close-up (the
  near tier: ~97 tier nodes, MultiMeshes), 51 draw calls / 24.7k triangles from 300 m (merged mesh), 39 draw calls / 3.3k
  triangles in the far tier (the chopstick's 2.9k triangles and the mount stay); the lattice alone is ~21.5k triangles.
  `TowerView.build` (loads the models, builds the MultiMeshes and the merged meshes) takes 155-174 ms (median of 5) once
  per site build, on the main thread. The per-frame main-thread cost is not visible in `Performance.TIME_PROCESS` (it
  reads the 60 Hz frame, 16.8-17.3 ms, with and without the tower); the CPU render time is the measured cost. Well under
  the budget on the desktop; VR (stereo, ~20 MP) and a headset run (`--perf-log`) are open.
- Not verified: texture pop at 300 m (only luminance checked); the far shader's fit under other light (one sun / sky).
- Cloud in front of the far tier: `screenshots/pad-b/f_cloud_a2c.png` (opaque, alpha-to-coverage: the haze of a thin slab
  between the camera and the tower at 6 km dims it) against `f_cloud_blend.png` (the old blended draw: the tower stays
  crisp on top of the cloud).

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
- Brace width: the spec asks that no member is thinner than ~1.5 px before it fades; the brace (0.4 m) is 1.04 px wide
  where the near range ends on the 60 deg / 900 px desktop view (2.0 px at a 1500 px VR focal length). The hand-over is a
  hard swap to a merged mesh with the same geometry, so the 1.04 px is not visible in the luminance, structure or shimmer
  measures; recorded as a deviation from the spec's number.
- Silhouette top: the lattice's roof cap and pyramid frame reach 150.1 m (144.5 m + 0.6 m + 5 m), the physics tower box
  is 146 m high: open point for the camera solve (decide there which one the photo supports).
