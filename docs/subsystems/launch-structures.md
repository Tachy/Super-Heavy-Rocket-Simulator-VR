# Launch structures (Pad B tower and chopstick model)

State: fine build plan `docs/superpowers/plans/2026-10-31-mechazilla-fine-build.md`, tasks 1-4 done (lattice and tower body;
carriage, both chopsticks, QD arm; square mount, trench, terrain mask; lights); task 5 (acceptance) open. The prototype plan
(`2026-10-31-pad-b-prototype.md`) built the hybrid pipeline. Spec: `docs/superpowers/specs/2026-10-28-pad-b-model-design.md`.
The lattice, carriage, both chopsticks and the QD arm render at `starbase_olp1`; everything the physics touches stays on the physics numbers (table
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
  on the back face, dark deck slabs 0.4 m at every floor, column bundle 3.6 m.
- Chopstick (user ruling 2026-10-31; member map: measurements "Arm member map"): `mechazilla_layout.chopstick`, a U of large
  round black tubes only (4.3k triangles, 24 truss members incl. 3 caps, table `ARM_MEMBERS`), 25.9 m. Plan: root out from the
  hinge to the bend (5.2 m, 3.6 m outward), leg back in 5.6 deg to its capped end (19.5 m), connector to the bearing tube
  (23 m; this is what was once read as a "28 deg tip bend"), inner root tube from the hinge to the bearing tube's start (13 m).
  The BEARING PIECE (1.3 m tube, flat 0.15 x 1.4 m plate on its inner side, tan strip on top, contact pad at the axis station) runs
  along the arm's X: at the core's closed pose (`opening_rad` 0) both faces are parallel and `arm_closed_gap_m` 9.6 apart, centred on
  the catch axis. One truss plane under the bearing line (not a V): lower chord, 4 posts, 3 diagonals; knee strut, hinge column and two
  root columns to one foot 17 m down. Thin: at most 3 supply lines; black ladders and probe brace (`fittings`), platform, mast, short
  pushers, hanging probe. No railing, no deck, no lattice.
- Carriage (`mechazilla_layout.carriage`): black frame 13.5 x 9 m, hinge frames on the physics pivot (5.8 m), walkway past the corners,
  3.8 m sheave block; four hoist cables to the roof block, stretched in `TowerView` with the carriage offset. Per arm an
  electromechanical actuator (NSF 2026-05: no longer hydraulic): barrel on the carriage, rod to a lug on the arm's inclined root column,
  both cylinders placed from the two end points each update (`TowerView.actuator_ends`), so they follow the opening angle by geometry
  (5.3 m closed, 6.5 m at 0.7 rad). Its numbers are the `actuator_*` keys of `[carriage]`.
- QD arm (`mechazilla_layout.qd_arm`): THE ship QD arm, one arm; deep root truss with light cabinets, shallow tip platform, connector
  head; tip deck 86.8 m, static, head 1 m clear of the 4.5 m hull on the tower's centre plane, fits between the closed arms.
- Mount (`mount_layout.py` -> `gen_mount.py` -> `mount.gltf`, `[mount]` of the detail file): four sloped corner piers and a deck, no stepped
  pyramid. Base 44.6 x 40 m, deck 18.7 m wide, ring top 14.5 m as measured, slopes 45 deg (measured 48 (5)). Closed plated faces only on
  +-X. Each +-Y side is one portal: 26 m wide at the ground, sides following the piers up to the 7.2 m top, a vertical spandrel, a lintel
  and a ribbed duct above it; inside, a ribbed deflector curves from a ridge (6.9 m) under the ring to the pit floor 9 (2) m below the ground. Ring: outer 10.4 m, opening 9 m, stands 1 m proud of the deck, 20 clamps; 7 m
  chamber under it. Two hoods 70 deg apart, symmetric about the line away from the tower (roof 4 m above the deck): the long one over the
  closed -X face with a box 6.5 m down the slope, the short one ending at the -Y deck edge. -X face: two rectangular grey doors, six
  vents (padb_mount_pyramid_front_person_scale_2026-05); two booster QD cabinets between the ducts (static). 5.5k triangles. Plate map
  `mount_panels.png` (`mount_textures.py`: stacked plates 6 x 2 m, rust-tan seams, grey to tan, stains; `shaders/mount_shell.gdshader`).
- Trench and ground (`trench.gltf`, `[trench]`): two flat aprons 26 m wide, floor rising from the pit to the ground, centre 33 m from the
  axis, 1 m side walls, tan-brown floor. Hardstand: a flat convex plate of light concrete all round mount and tower foot (outline +-15 m,
  low confidence: wide-angle views only), `pad_concrete.png`, satellite terrain beyond its edge; the old pad disc stays hidden, so past the
  20 km visibility range the site is terrain imagery only. Tower foot: an 18 m block, 7 m walls, chamfered roof up to the legs' plinth.
- Terrain mask: `terrain_earth.gdshader` does not draw the terrain inside the footprint (central rectangle + both ramps, 0.05 m inside
  the walls) within `mask_range_m` = 20 km of the camera; the pit model is drawn 150 m beyond. Globals `trench_*` (project.godot) set by
  `TowerView._set_trench_mask`; the site position is a hi / lo pair like the camera's, so the offset stays exact. Physics ground stays flat.
  Beyond 20 km the footprint (< 6 px at a 60 deg view) is terrain again (no hole, a step in the ground colour only).
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
- `TowerView.build` frees its children first (no duplicates on a rebuild); both arm boxes are chopsticks (the model's +Y points
  to the axis; the -across arm is mirrored by a negative scale), the pivot is the physics hinge. `update_state` also places the
  carriage (hinge height + offset) and the cables. `render_check --catch_hull=<station>` hangs a static hull with fins in the arms for
  comparisons (`game/tests/catch_pose_hull.gd`, no flight). Test: `game/tests/tower_view_test.gd` (headless; the dummy
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
- `tools/structures/tests/test_mechazilla_layout.py`: the bearing faces of both arms built at the closed pose from the PHYSICS hinge (not from the
  layout's own numbers): bearing faces parallel within 0.5 deg, the physics gap apart, centred on the axis; U plan, member table; closed
  arm's surfaces >= 0.03 m off the hull, pad 0.1 m from it and ending at the arm tip, open frame (no deck, strip <= 0.8 m, top at Z = 0).
  `tower_view_test`: the pad meshes' centres through the pivots against hinge + pad station (0.05 m), the pad centres 9.8 m apart, the
  actuators' ends on the carriage and the lug, length follows the opening. `python tools/structures/check_envelope.py`: inner face on the
  physics half gap, arm length, hinge, tower width / height, arm top plane, leg outer width, QD head against the hull, drawn structure under the
  deck against the pin-to-fin distance (vehicle config); a row is listed only when the compared numbers differ (exceptions = table rows).
  Mount deck and azimuth / distance have no drawn number in the detail file (the view reads the physics geometry).
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
| Tower, carriage and arms | lattice, foot, mount and trench parallel at `plan_yaw_deg` 36.5, tower centre 36.1 m from the axis; the corner toward the mount lies about 3.7 m beyond the physics pivot plane | physics box and pivots at 30 m, face-on | **carriage and arms stand off the corner until the physics plan moves the pivots onto it** (user 2026-10-31: the hinge sits on the corner; rail bracing at top and bottom back to the two neighbouring corners, not modelled); tower lamps still placed from the physics face |
| Mount deck | ring top 14.5 m as measured (user 2026-10-31: reality first), portal top 7.2 m, slope 45 deg | physics 20 m until the physics plan | **until then a stack on the pad hangs 5.5 m above the ring**; `check_envelope` lists the row |
| Arm tops, carriage | at `tower_state()`, 95 m at the catch | measured 133 +-4 m at the catch, 134 +-4 m parked | pins land on them |
| Arm length | 25.9 m, closed tip 1.9 m past the catch axis (physics hinge 24 m from it; the real tip is about at the axis; the pad ends at the tip and spans the axis) | 36 m | hinge-to-axis 24.5 +-2 m; kept from the earlier fit; the flat bearing piece clears the hull at any length |
| Hinge position | resolved in the fix round: drawn hinge frames on the physics pivot 5.8 m (the corner hinges read 6.75 m: not drawn); `check_envelope` compares them, no exception | 5.8 m | the inner face stays on the physics half gap 4.8 m |
| Root of the U | root 35 deg, 6.3 m from the physics hinge (5.8 m off the axis) to the bend 9.4 m off it | photos: 50 +-15 deg, about 10 m, so a real hinge about 2 +-3 m off the centre plane | hinge stays on the physics number; a physics plan moving the hinge needs `arm_closed_gap_m` 9.6 unchanged and opening 0 unchanged |
| Fins against the arm | pins 66.5 m put the deck 2-2.5 m above the fin plates, the arm's top 2.5 m and truss run through the 0 / 180 fins at a catch: visible overlap, NOT hidden. Pad at the closed pose under where the fin bars come down | pins 66.5 m, azimuths 45 / 225, fins without contact | V3 user view: deck 0-0.9 m under the fins; proposal in the measurements |
| Tower face, height | width 12 m, roof deck 144 m, lattice 139.1 m, silhouette 146.9 m | box 12 / 146 m | within the error |
| Leg spacing | outer width 12 m (leg centres 9.8 m) | box 12 m | measured leg centres 11.9 +-1 m (outer 13.3-14): the legs stay inside the physics box |
| QD arm | static, deck 86.8 m, head 1 m off the hull (not attached, not swung off); stack on the 20 m deck stands 5.5 m higher than measured | no state | needs the core's `qd_arm_angle_rad` and mount height |
| Azimuth, distance | physics 315 deg, 30 m | measured 345 +-25, 32 +-3 m | low confidence |

## Open points
- Arm: root bend and width from weak
  views (no top view of a Pad 2 arm found); deck grating and rack strip not drawn; 4.3k triangles per arm, far tier unchanged (cost:
  task 5); shock absorbers and the fin's station undecided.
- Render vs the frames (`sf_*.png`): mount taller and steeper than the photos (the 20 m deck), plates cleaner and more regular, hoods without
  the chamfered facets, vents plain wedges, no scaffolds, vehicles, recessed door bays or the deck's fine hardware; hardstand outline and portal
  width carry the largest plan errors (ship39 reads the ramp 15 m, tower foot 26 m, NOAA 20 +-4). Still different on the tower: no platforms with grating and no cross braces inside the face, the plinth is a plain box,
  the hoist cables and side members of the roof block are missing, grime is not tuned.
- The far shader's `blur`, `alpha_scale` and colours are fitted to the merged tier at 900-1300 m for one sun / sky
  condition; other light may show a small step (< 2 % luminance measured).
- Single-view ratios in oblique photos (face width, depth) carry several percent error; leg, wall, pitch and truss
  pitch are estimates (not measurable from the references). The full build re-measures with a camera solve.
- Brace width: the spec asks that no member is thinner than ~1.5 px before it fades; the brace (0.5 m) is 1.30 px wide
  where the near range ends on the 60 deg / 900 px desktop view (2.5 px at a 1500 px VR focal length). The hand-over is a
  hard swap to a merged mesh with the same geometry, so the 1.30 px is not visible in the luminance, structure or shimmer
  measures; recorded as a deviation from the spec's number.

## Lights (fine build task 4, spec G)
- Fixtures: `config/structures/starbase_olp2_lights.toml` (own file: the lamps do not feed the Blender models, so a change never stales them;
  validator `detail_file.load_lights`). Groups in the site frame (`ref` face / axis, along / across / z, count + steps, mirror): obstruction
  lamps (4 at the roof corners 146 m, 4 at 72 m), roof floods (4), the two face-edge lamps at 89 and 83 m (`spacex-starship-mdr-100`: lamps
  ~89 / ~83 m on the pad-side face's edge, 4.9 px/m from the 144 m tower), 14 lattice lamps every 9 m, tower-foot floods (2), two mount
  banks of four lamps (z 27 m, 21 m across, aimed at the deck; `padb_night_floodlights_mount`, `HN635FUWYAAzbOB`), deck lamps (4). All
  positions / lumens / CCT [E] with the photo named, FAA values [S]. 27 burning lamps + 8 obstruction lamps, 4 shadow-casting spots
  (`max_shadowed`, the banks' first two lamps each), lights are Godot spot / omni with ranges 12-150 m (no light beyond its range).
- Physics: lumens -> cd (`TowerLights.candela_cone` for the cone profile Godot's `spot_angle_attenuation` 4 makes: flux of the profile = the
  lumens; `candela_omni`), energy = cd / pi x `atmosphere.scene_per_cd_m2` (the Sun's convention; includes the eye adaptation's gain, updated
  each frame), distance attenuation 2 (inverse square, Godot's 1 - (d/r)^4 window), CCT by Kim et al. 2002 (6500 K ~ white; 5000 K orange in
  linear sRGB). Lenses: `light_lens.gdshader`, a billboard disc at L = I / A in cd/m^2 x `atmosphere_scene_per_cd_m2`; far away it stops at
  `point_angle_rad` 0.0006 and its luminance falls by the area ratio (flux kept: the far emissive point), halo cosmetic.
- Schedule: `night_factor` 0 above Sun -6 deg (civil dusk, [S]) up to 1 at -7 deg (`fade_deg`), from the Sun's elevation at the site
  (`TowerView.update_lights`, called from `World._update_sun_light`, no extra cost by day: lights hidden). Obstruction lenses flash always:
  30 / min [S: FAA AC 70/7460-1 / 150/5345-43, 20-40], burn 0.5 s [E]; red 2000 cd [S] by night (with a red omni glow on the steel), white
  20000 cd [S, L-865] by day (lens only).
- Eye adaptation (open design point, [call]): L_a had no term for artificial light, so a lamp-lit pad at the dark-adapted gain (12 400) was blown
  to white. `Atmosphere.lamp_adaptation_cd_m2` (set by `World` from `TowerView.lamp_adaptation_cd_m2`) is one more candidate of L_a's maximum
  (and its scotopic equivalent): 3 % of the lit deck's luminance (albedo 0.3 / pi x the fixture model's 176 lux = 16.8 cd/m^2), weighted by what
  the eye sees - camera within 300 m full, 0 beyond 1.5 km, site within 25 deg of the view axis full, 0 beyond 60 deg - so there is no global
  term; at a mount view L_a 0.50 cd/m^2, gain 7390 (was 12 400). 3 % is the mean-to-peak ratio of a night view [call], to be tuned in the headset.
  Other lamps (city) are still not in L_a.
- Tests: `game/tests/tower_lights_test.gd` (schedule at -5.9 / -6.5 / -7.5 deg, flux of the cone profile, FAA rate 30 / min, day: no lamp burns /
  all 8 obstruction lenses flash, night: 27 burn, 4 shadowed, rendered deck illuminance 218 lux = 1.24 x the fixture model 176 lux (limit
  30 %), exposure scaling, no lamps at box sites); `pytest tools/structures/tests/test_detail_file.py` (lamps file). Day luminance with the lamps
  against `render_check --no_tower_lights` (fixed exposure, 17:30 UTC, mount view): mean sRGB luminance changes by 0.004 % (limit 0.5 %).
- Cost (`tower_cost_check --utc=`, 1600x900, tower incl. lamps vs hidden): day close-up +0.06 ms GPU (1x), night +0.36 ms (+0.35 ms at 2x
  pixels), 300 m +0.05 ms, 1 km +0.05 ms, 6 km +0.07 ms; 22 draw calls more at night (lights, lenses, housings). Night image set
  `screenshots/pad-b/fine/cmp_*.png` (ours left, reference right), twilight at Sun -6.7 deg, night at -47 deg.
- Left out / differs: no Blender `gen_lights.py` (lens discs and plain dark boxes made in Godot, surface detail rule: the housings are not
  modelled); no lamps on the arms, carriage, QD arm or the vehicle; no cranes / scaffold lights, no booster in the lamp tests; the far
  hardstand receives lamp light only inside 150 m; the far tower tier (> ~1 km) is not lit by the lamps (its shader has its own colour), so
  at 900 m by night the tower is nearly dark where the photo shows it bright; twilight at -6 .. -8 deg is dark: the references' lamps burn
  at sunset (Sun ~ -1 deg), the spec's civil dusk threshold is kept.

## Fine build task 3 evidence (`screenshots/pad-b/fine/`, `.superpowers/sdd/2026-10-31-pad-b-prototype/fine3_*.txt`)
- `tools/structures/tests/test_mount_layout.py`: base 44.6 x 40 m, deck 18.7 m, ring top = physics height, ring 9 / 10.4 m, 20 clamps, aprons 26 m wide / 33 m
  out as wide as the portal foot, floor -9 m, portal open over its whole width on both sides, rectangular doors, hoods down the front face, hardstand round the site and off the pit, foot on the physics axis, every polygon planar. Stack clearance (Review Focus 1): sampled points of
  every mount polygon (6 per triangle) against the hull (4.5 m radius above the ring top, +0.04 m) and the 33 booster engines' bells below it
  (rings 1.0 / 2.6 / 4.1 m, exact bell contour, margin 0.04 m): none inside; a 8 m opening is caught (negative control). `tower_view_test`
  (309 cases): model dimensions, deck at 19 m, ring opening 4.5 m, mask equals the drawn footprint (+-0.06 m), no script error in the log.
- `game/tests/trench_mask_check.gd` (Review Focus 4; GPU): per distance 30 m .. 400 km three frames (terrain alone / mask on, no models / as drawn);
  the pixels the mask changes (isolated noise filtered, 150 frames wait for the node morph) lie on the projected footprint (+3 px) at every
  distance (0 px outside), 100 % of them are covered by the model (per colour channel: the hazed dark ramp matched the hole's luminance at 5-12 km, frame read), none beyond 20 km. `fine3f_trench_mask.txt`.
- Cost (`tower_cost_check`, RTX 4090, 1600x900): the whole tower incl. mount and trench adds +0.08 ms GPU at the 60 m close-up (0.68 vs 0.60), 0.01 ms at 6 km; the terrain mask itself within +-0.01 ms (noise level).
  The discard removes early-z on the terrain (not measurable here). `TowerView.build` 144 ms.
