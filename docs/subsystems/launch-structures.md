# Launch structures (Pad B tower and chopstick model)

State: fine build plan `docs/superpowers/plans/2026-10-31-mechazilla-fine-build.md`, task 1 done (measured lattice and tower
body); tasks 2-5 (carriage and chopsticks, mount and trench, lights, acceptance) open. The prototype plan
(`2026-10-31-pad-b-prototype.md`) built the hybrid pipeline. Spec: `docs/superpowers/specs/2026-10-28-pad-b-model-design.md`.
The lattice and one chopstick render at `starbase_olp1`; everything the physics touches stays on the physics numbers (table
"Physics deviations" below).

## Model
- `config/structures/starbase_olp2.toml`: site-independent detail of THE Mechazilla (tower sections, member profiles,
  chopstick truss). Placement and physics numbers (arm length, hinge, tower height) stay in `launch_sites.toml`; the
  file never duplicates them. Keys: `config/README.md`.
- Every value is tagged `[E]` (own estimate with its method) or `[S]` (published, URL).
- Measured numbers (task 1, `launch-structures-measurements.md`; the superseded draft numbers are in
  `docs/history/launch-structures.md`): lattice 139.1 m (9 sections: 8 x 16.05 m X-braced, 1 x 10.7 m chevron under the
  roof), 26 floor levels of 5.35 m (decks, independent of the brace pitch), X-brace panel pitch 8.025 m (crossings 7-8 m,
  webcast pass and the diamonds on the Ship 42 photo; the top section keeps 5.35 m), face width 12 m (the legs 2.2 m
  box, braces 0.5 m H), plinth 12.2 m (12.4 m wide), light-grey cladding plates on the pad-side face from 12.2 to 79 m
  (courses of 5.35 m, two plates across, 0.1 m seams), roof deck 144 m with a louvred 8 x 8 x 7 m block (139.9-146.9 m) and
  a railing (silhouette top 146.9 m, the physics box is 146 m), zig-zag stair flights on one side face and two cable trays
  on the back face, dark deck slabs 0.4 m at every floor, column bundle 3.6 m. Chopstick (task 2 replaces it): length 26 m,
  width 3 m, depth 8.75 m, truss pitch 6.5 m, tube chords 1.0 m, diagonals 0.7 m, rails 0.9 m.
- Open part of the lattice: `x_brace` on all eight lower sections, `chevron` on the top one. Which brace pitch the frames
  decide: the Ship 42 photo shows diamonds 7-8 m between crossings (6.4 px/m, ~50 px), confirming the webcast pass; the floors
  stay at 5.3 m, so a brace panel spans 1.5 floors and the sections hold 3 floors = 2 panels.

## Game side (task 3)
- `StructureModels.load_for(site_id)` (`game/world/structures/structure_models.gd`): `{}` unless the site is the
  prototype's (`PROTOTYPE_SITE_ID = "starbase_olp1"`), the detail file exists and `data/models/starbase_olp2/` is not
  older than the detail file / generator scripts (file mtimes; the manifest's Blender hash cannot be reproduced in
  GDScript). Otherwise one log line per site and `TowerView` keeps its boxes. glTF is loaded at run time (`GLTFDocument`);
  the Blender axes (arm +X, Z up) become render axes once, in `BLENDER_TO_RENDER`.
- `LatticeBuilder.build(detail, profiles, materials)` (`lattice_builder.gd`; `materials` = `TowerMaterial.lattice_set()`:
  leg / brace / dark / clad): per section one MultiMesh of the corner legs (the top section's run up to the roof deck's
  underside), one per face of braces (`x_brace`: 2 diagonals + 1 horizontal per panel; `k_brace`; `chevron`: Lambda; the top
  section adds a cap horizontal), `decks` (dark unit boxes: a deck per floor level, the column bundle, stair flights, cable
  trays, on the top section the roof deck, louvre bands and railing), `clad` (light unit boxes: the plinth on the first
  section, the cladding plates on face 0 where the section overlaps 12.2-79 m, the roof block on the top one) and on the top
  section `roof` (four frame horizontals under the roof deck). Panels = round(height / brace pitch), floors = round(height /
  floor pitch): the two are independent. The layout functions (`dark_solids`, `clad_solids`, `top_frame_members`,
  `face_members`) are public for the headless test. Members cast no shadow, and there is no
  shadow proxy: the game's Sun (`world.gd` DirectionalLight) has no shadow maps and `terrain_earth.gdshader` is unshaded,
  so the tower casts no shadow on the ground today (a real one is its own decision: Sun shadows or a baked / analytic
  tower shadow on the terrain shader). Footprint on the physics box's pad-side face and base; roof deck 144 m and
  silhouette top 146.9 m against the box's 146 m.
- Distance hand-over (visibility ranges, hard swap, no fade): near `legs`/`face_n`/`decks`/`clad`/`roof` up to
  `NEAR_END_M` = 300 m; `mid` (the same geometry merged into 4 surfaces = draw calls per section: legs, braces, dark,
  light) 300 ..
  `MID_END_M` = 5000 m; `far` box with `shaders/tower_far.gdshader` beyond (analytic chevron / deck / leg coverage of
  the measured pitches, filtered by the pixel footprint, plus the closed plinth and the cladding plates; coverage resolved by MSAA alpha-to-coverage, so the box stays in
  the OPAQUE pass and clouds in front of it cover it: an alpha-blended draw lands after `CloudEffect`, which runs
  before the transparent pass). Godot measures a visibility range from the centre of the instance's bounding box:
  `LatticeBuilder._share_range_centre` gives every tier node a `custom_aabb` centred on the tower's middle, so all
  sections and faces swap at one camera distance (before: sections at different heights and the faces swapped at
  different distances, a tower half in each tier). The far tier starts where the tower is 2 px (desktop, 779 px focal
  length) to 3.6 px (VR eye, 1500 px) wide; at 1 km, where it began before, it lost the truss at VR focal lengths.
  Nothing else of this plan draws in the transparent pass (steel / dark / chopstick materials have no alpha).
  Why these ranges and why no dither: `tower_handover_check.gd` below.
- Colours (ruling 2026-10-31 on the real frames: `vehicle-models.md` "Black parts", real-frames item 11): legs bare steel,
  linear albedo ~0.15 +-0.08 [E] (bright in direct sun, near black in shade), braces / decks / bundle dark, albedo ~0.04,
  plinth / plates / roof block light grey ~0.30. No tint to fake the look: albedo and the physical lighting. The tints in
  `tower_material.gd` are computed from the Metal032 map's linear mean (0.205, 0.248, 0.294) and the mean grime (0.91), as
  the sRGB values of a `source_color` uniform (the earlier tint 0.65 gave only ~0.09). The far shader's `leg_colour`,
  `steel_colour`, `dark_colour`, `clad_colour` are the same albedos (refit to the merged tier below).
- Material (`tower_material.gd`, `shaders/tower_steel.gdshader`): CC0 ambientCG Metal032 colour map (data/materials,
  `python tools/data_pipeline/fetch.py ambientcg_metal032`; missing: plain steel) triplanar in the member's own scaled
  frame + procedural grime per instance; decks dark; chopstick matte black (`black_paint`).
- `TowerView.build` frees its children first (no duplicates on a rebuild); the chopstick replaces the `+across` arm box
  only, its hinge is the pivot, so `update_state` is unchanged. Test: `game/tests/tower_view_test.gd` (headless; the dummy
  renderer keeps no MultiMesh transforms, so layout is read from `LatticeBuilder.face_members` and the `*_solids`
  functions). Task 1 cases: lattice top 139 m, 26 floors of 5.3 m, roof deck 144 m, plinth top 12.2 m, cladding plates
  12.2-79 m on the pad-side face only, silhouette <= 148 m (146.9 m), four frame members and no pyramid under the roof.

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
- `python -m pytest tools/structures/tests -q`: required keys, sections sum to the 139.1 m lattice (139 +-0.2), 26 floors of
  5.3 +-0.1 m, leg 2.2 / brace 0.5 m, plinth 12.2 m, cladding 12.2-79 m, X-brace pitch 7-8 m, no pyramid keys, silhouette
  <= 148 m, every value line tagged, loader raises on missing keys / bad units / unknown pattern and checks the deck, core, roof and foot tables;
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

## Fine build task 1 evidence (`screenshots/pad-b/fine/`, `.superpowers/sdd/2026-10-31-pad-b-prototype/fine1_*.txt`)
- `tower_handover_check --focal=both` passes (`fine1_handover.txt`): all 36 band steps uniform; dolly second difference
  near -> mid 0.29 % (desktop) / 0.31 % (VR), mid -> far 1.04 % / 0.80 % (gate 2 %); tiers shown alone: members / merged
  0.23-0.41 %, merged / shader 0.35-1.15 % (VR, `fine1_tiers0.txt`: 0.4-1.2 %); shimmer 0.98-1.01 at 280-4000 m, 0.29-0.40
  at 6 and 8 km (gate 1.10); brace 1.30 px (desktop) / 2.50 px (VR) where the near range ends. The far shader's colours
  were set to the new albedos from the numbers above and needed no second fit (every tier pair already inside the gate).
- Side by side with the Ship 42 frame: `c_side_by_side_ship42.png` (frame left, render right; the frame stays in the
  git-ignored folder). The render reads as the same tower: bright corner columns (in sun, `a_top135.png`), dark X diamonds,
  closed light plates on the pad-side face to 79 m, flat roof deck with the louvred block and railing. Differences left:
  the frame's plinth is a dark sloped base, ours a straight box; the frame's lattice has more members (cross braces
  inside the face, platforms with grating); legs in shade are darker than the frame's sunlit ones (the render's Sun at
  9 am is on the other side of the tower from that camera).
- X-brace pitch decided by the Ship 42 diamonds (~50 px at 6.4 px/m = 7.5-8 m) and the webcast pass: 8.025 m.

## Physics deviations (what is drawn as measured differs from the physics; input to the physics plan)
| Part | Drawn now | Physics stays | Why |
|---|---|---|---|
| Mount deck | 20 m (physics); plan, faces, courses as measured (task 3) | 20 m, measured 14.5 +-1.2 m | a stack stands on the deck |
| Arm tops, carriage | at `tower_state()`, 95 m at the catch | measured 133 +-4 m at the catch, 134 +-4 m parked | pins land on them |
| Arm length | 27 m, closed tips at the catch axis (task 2) | 36 m | hinge-to-axis 24.5 +-2 m |
| Tower face, height | width 12 m, roof deck 144 m, lattice 139.1 m, silhouette 146.9 m | box 12 / 146 m | within the error |
| Leg spacing | outer width 12 m (leg centres 9.8 m) | box 12 m | measured leg centres 11.9 +-1 m (outer 13.3-14): the legs stay inside the physics box |
| QD arm | static, swung clear (task 2) | no state | needs the core's `qd_arm_angle_rad` |
| Azimuth, distance | physics 315 deg, 30 m | measured 345 +-25, 32 +-3 m | low confidence |

## Open points
- Render vs the frames (task 1): still different: no carriage / mount frustum (tasks 2-3; the old mount cylinder), the
  -across arm is a box (task 2), no platforms with grating and no cross braces inside the face, the plinth is a plain box,
  the hoist cables and side members of the roof block are missing, grime is not tuned.
- The far shader's `blur`, `alpha_scale` and colours are fitted to the merged tier at 900-1300 m for one sun / sky
  condition; other light may show a small step (< 2 % luminance measured).
- Single-view ratios in oblique photos (face width, depth) carry several percent error; leg, wall, pitch and truss
  pitch are estimates (not measurable from the references). The full build re-measures with a camera solve.
- Brace width: the spec asks that no member is thinner than ~1.5 px before it fades; the brace (0.5 m) is 1.30 px wide
  where the near range ends on the 60 deg / 900 px desktop view (2.5 px at a 1500 px VR focal length). The hand-over is a
  hard swap to a merged mesh with the same geometry, so the 1.30 px is not visible in the luminance, structure or shimmer
  measures; recorded as a deviation from the spec's number.
