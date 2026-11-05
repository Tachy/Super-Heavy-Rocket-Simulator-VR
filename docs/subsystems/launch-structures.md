# Launch structures (Pad B tower and chopstick model)

State: fine build plan `docs/superpowers/plans/2026-10-31-mechazilla-fine-build.md`, tasks 1-4 done (lattice and tower body;
carriage, both chopsticks, QD arm; square mount, trench, terrain mask; lights); task 5 (acceptance) open. The prototype plan
(`2026-10-31-pad-b-prototype.md`) built the hybrid pipeline. Spec: `docs/superpowers/specs/2026-10-28-pad-b-model-design.md`.
The lattice, carriage, both chopsticks and the QD arm render at `starbase_pad2`; the physics stands on the measured Pad B since the physics plan's Tasks 1-3
(`docs/superpowers/archive/plans/2026-10-31-pad-b-physics-alignment.md`): ring 25.99677 N, 97.15804 W, deck 14.5 m, tower azimuth 349, 31.7 m, box yaw 37.4, one hinge axis 2 +-1 m outside the box corner toward the mount (20.40 m along the line, 1.51 m across), arm 24.0 m (3.6 m past the axis: margin, user 2026-10-31), catch 133 m, pins 63.0 at 0 / 180. Open differences: table "Physics deviations" below.
Sea: `[site.sea]` 90 deg, 765 m [D]: first water along that bearing in the 3DEP grid, the nearest shore of bearings 60-130. An abort first checks from the live state whether the water is in reach (`gnc::estimate_sea_reach`, a lower bound, x `sea_reach_margin` 1.5). A booster reaches it from 1500 and 800 m with 5.2 and 4.1 t left; from 400 m it gives the water up and comes down soft on land (500 m was an estimate from the old position).
Booster roll at a catch is one-way (`away_from_tower_azimuth_deg` 90, `gnc::catch_roll_reference(CatchTarget, ...)`); a ship keeps the nearer of its two rolls.

## Model
- `config/structures/starbase_olp2.toml`: site-independent detail of THE Mechazilla (tower sections, member profiles,
  chopstick truss). Placement and physics numbers (arm length, hinge, tower height) stay in `launch_sites.toml`; the
  file never duplicates them. Keys: `config/README.md`.
- Every value is tagged `[E]` (own estimate with its method) or `[S]` (published, URL).
- Measured numbers (task 1, `launch-structures-measurements.md`; the superseded draft numbers are in
  `docs/history/launch-structures.md`): lattice 139.1 m (9 sections: 8 x 16.05 m X-braced, 1 x 10.7 m chevron under the
  head), 26 floor levels of 5.35 m (decks, independent of the brace pitch), X-brace panel pitch 8.025 m (crossings 7-8 m,
  webcast pass and the diamonds on the Ship 42 photo; the top section keeps 5.35 m), face width 12 m (the legs 2.2 m
  box, braces 0.5 m H), plinth 12.2 m (12.4 m wide), light-grey cladding plates on the pad-side face from 12.2 to 79 m
  (courses of 5.35 m, two plates across, 0.1 m seams), the tower head 140.5-147.9 m (silhouette top, the physics box is 146 m), zig-zag stair flights on one side face and two cable trays
  on the back face, dark deck slabs 0.4 m at every floor, column bundle 3.6 m.
- Common hinge axis (user rulings 2026-10-31 / 2026-11-01): both arms turn about ONE vertical axis `hinge_outside_corner_m` = 2 +-1 m [E] outside the physics box's corner toward the mount, `TowerGeometry::hinge_pivot` (20.396 m along the line, 1.508 m across, from `distance_m`, `width_m`, `plan_yaw_deg`, the offset; `layout.pivot_site` is the same formula). Offset: crop_rail_colleen and DSC06000 read the drum and block about 2 m off the corner edge, no pixel solve; the direction is NOT fixed by the photos (one oblique view each, height and depth mix): the corner's outward diagonal is a choice (-0.991 along, -0.132 across the line). The arm box is the closed arm turned rigidly about it: each side keeps a fixed offset to its line (`arm_line_offset_m`: 7.31 m +across, 4.29 m -across; they differ by 2 x 1.51 m), at opening 0 both faces are parallel to the line, 9.6 m apart, centred on the catch axis. Standby 7.1 deg: hull clear 2.83 / 2.81 m per side. The second tower site (`ksc_lc39a`, no photos, yaw 0) has the same offset: 22.59 m along, 7.41 m across, arm 34.59 m (tip where it was), standby 6.5 deg, hull clear by 2.5-3.2 m (test). No `hinge_distance_m`, no per-side hinge.
- Chopstick (member map: measurements "Arm member map"): `mechazilla_layout.chopstick(detail, side)` in two models (`chopstick.gltf`, `chopstick_minus.gltf`: the roots differ, the -across model is mirrored in the view), large
  round black tubes only (4.1k triangles, 23 truss members incl. 3 caps, table `ARM_MEMBERS`), 24.0 m = the physics arm: inside the closed-tip reach 24.5 (2) m and at or just below the hinge-to-tip 27 (2) m of the photos (straight lines hinge to tip 25.09 m on the +across arm, 24.38 m on the -across arm; 24.0 is the part along the arm). Plan: root out from the
  common axis to the bend (5.2 m along, 9.4 m off the catch axis: 64 / 57 deg off the arm), leg back in to its capped end (17.5 m), connector to the bearing tube
  (21 m), inner root tube from the axis to the bearing tube's start (11 m).
  The BEARING PIECE (1.3 m tube, flat 0.15 x 1.4 m plate on its inner side, tan strip on top, contact pad at the axis station) runs
  along the arm's X. One truss plane under the inner root and bearing line (not a V): lower chord from the foot rising to the tip (side view a long triangle: 15 m at the foot, 8.6 m at 11 m, 7 m at 20.6 m), 4 posts, 3 diagonals; three thick
  root columns from the top to one foot on the common axis, 15 m down (the axis itself is the carriage's hinge column). Closed 1.6 m box on the tip (`tip_box_*`), thin hanging probe. Thin: at most 3 supply lines; black ladders and probe brace (`fittings`), platform, mast. No railing, no deck, no lattice.
- Carriage (`mechazilla_layout.carriage`, `rails`, `rollers`; frame X toward the pad, Y = -across, origin on the common axis): the tower box of the physics
  (13.3 m, yaw 37.4) is the drawn lattice. One rotary hinge column (1.4 m) on the axis, no outriggers; horizontal struts at both collar levels from the axis back to the corner. Two collars along both side faces from the corner to the next corners; upper = walkway with railing, lower =
  truss 1.8 m deep, 15 m below the carriage top (+-3 m; `collar_gap_m`, at the root's foot). The whole carriage runs on 6 roller points (`rollers`, 3 per level) on three rails (`rails.gltf`, plinth top to lattice top), `rail_inboard_m` = 1 +-0.5 m inboard of the corner edge on the faces (crop_rail_colleen: both visible rails on the plated face; the third, at the other neighbour, on the other side face by symmetry). 6 rollers (3 per level), 3 corners with rails, none at the rear and the second drum behind the first: user's description, not resolved on photos.
  White actuator drums beside the axis, one per arm (`actuator_*` keys), rod to a lug at the top of the outer root column, about level. A 3.8 m sheave block on the axis, 2.0 m wide (photo); 8 hoist cables in two groups of 4 (`cable_group_spacing_m` 1.25 +-0.3 m, 0.19 m pitch inside a group = 1.2 cable widths, the most that fits the 2 m block; the worker photo shows 2-3 widths across the whole block, `mechazilla_layout.hoist_cables`), vertical, running down onto the block's top (inside its width), load centred on the axis (`TowerView.hinge_axis_site`, `_cable_*`; statics, user 2026-10-31). Their upper ends meet the tower head's underside, on its load beam over the axis.
- QD arm (`mechazilla_layout.qd_geometry`, `qd_arm`): THE ship QD arm, FIXED in height (tip deck 81.3 m = 86.8 - 5.5, the stack stands 5.5 m lower) on the tower's rear
  side: swing axis `root_x_m` / `root_beyond_m` (-3.0 / 2.0 m in the box axes, 31.9 m from the ring axis), heading to the catch axis, reach 26.5 m so that the head's face is 1 m
  off the 4.5 m hull (connected pose, default). Walkway on a deep root truss, shallow tip platform, connector head; a swing drum under the root and a fixed bracket (`bracket` node)
  to the tower's rear corner. The swing key `TowerView.qd_arm_swing_rad` (0 = connected, positive turns the head away from the catch line, tested at 90 deg: clear of both
  chopsticks in plan at every opening and 40 m below them in height); the black frame with roller-like parts at the corner of the photos is read as this swing bearing.
  No time-dependent swing: it needs the core's `qd_arm_angle_rad` (Open points).
- Mount (`mount_layout.py` -> `gen_mount.py` -> `mount.gltf`, `[mount]` of the detail file; shape ruling 2026-10-31, numbers: launch-structures-measurements.md
  "Mount shape 2026-10-31"): a BLOCK with straight vertical walls (X -10.9 .. +10.3 in the square axes, Y +-9.4, flat plain deck 13.5 m, ring top 14.5 m)
  and a pyramid ANNEX only on the side away from the tower (plated slopes, trapezoid footprint Y +-22.3 at X -10.9 narrowing to +-9.2 at X -20; two doors at
  its foot, white vent plates, dark louvre boxes along the slanted edges). Nothing of the mount lies beyond X +10.3. The block is crossed along Y by ONE
  tunnel of one width (15.2 m) from end to end: rectangular portals (top 7.2 m above the ground), vertical walls, ceiling with the 7 m chamber under the
  ring, no deflector, no ridge, no flare. Ring: outer 11.6 m, opening 10.0 m (user 2026-11-03 "at the mount everything must fit": the outer Raptor bells, ring 4.40 m: the smallest passing opening is 9.63 m from the exact bell contour at the ring's height with the 0.04 m margin - the lip itself, 4.40 + 0.66 + 0.04 = 5.10 m radius, does not reach the ring's height (it would need 10.2 m there); DSC06000's 11.6 m reading is the ring unit's outer extent, the opening is set by the bell fit; clamps and hood roots moved out with it, the QD heads stay at 4.85 m, 0.35 m off the hull, above the bell envelope), stands 1 m proud of the deck, 20 clamps (the originals show no proud ring or
  clamps: kept until the seat is decided). Two hoods 70 deg apart, symmetric about the line away from the tower (roof 4 m above the deck): the long one
  down the middle of the annex, the short one ending at the -Y deck edge; a booster QD head stands on the ring under each (0.35 m off the hull, static, above the bell envelope;
  no cabinets or booms on the deck). 1.1k triangles. Plate map `mount_panels.png` (`mount_textures.py`; `shaders/mount_shell.gdshader`).
- Trench and ground (`trench.gltf`, `[trench]`): the tunnel's floor is one closed surface: level at -9 m (`pit_depth_m`) for |Y| < 9.4 under the ring,
  straight ramps up to the ground at Y +-41.5, 15.2 m wide; the walls outside the block run from the inner face (X +-7.6) out to the deck plate's side edges (X -10.9 / +10.3); their top falls from the deck height (13.5 m) at the block to a 0.5 m small wall (`wall_height_m`) where the annex footprint ends (Y 22.3), then runs level to the ramp end. Tan-brown floor (the ribbed dark look is a surface matter, not drawn). The floor
  stays closed so that a water layer can be added later. Hardstand: ONE flat plane of light concrete at the site height + `hardstand_lift_m` (user 2026-10-31: the area is graded, no terrain following) over the outline the user drew on a Google Maps screenshot, up to the road (`[trench] hardstand_outline_m`, "east,north" pairs from the ring centre, +-15 m: 3.3 px/m, image up = bearing 33 deg; 20 points, 3.96 ha, not convex; it reaches 211 m from the ring, beyond the 150 m plateau), turned into the mount frame with the azimuth and plan yaw (`mount_layout.hardstand_outline`), ear-clipped and cut by the block with its annex and the ramps (`hardstand_pieces`); `pad_concrete.png`, base colour from the 95th luminance percentile of the NAIP imagery around the site (sRGB 0.398 / 0.377 / 0.297, `TERRAIN_BRIGHT`: no bright patch against the terrain). Ground check (`game/tests/hardstand_ground_probe.gd`, the physics ground): site -21.65 m; tip, east corner and the whole plateau within 0.05 m; the road side +0.0 to +0.4 m (mean +0.24, one mound +0.96 at E 152 / N 129, +2.05 at the SE corner E 184 / N 83), so the flat plate steps by at most 0.4 m at the road except at that mound; a tilted plane fitted to the road samples would miss the tip by 0.15 m and the east corner by 0.5 m: flat it stays. The terrain does not poke through: `apron_mask.png` (512 x 512, 1 m texel, eroded by one texel; global `apron_mask` / `apron_mask_half_m`, `terrain_earth.gdshader`) masks it inside the plate. Tower foot: a block cut vertically
  on the side toward the mount (X 17.6, 7.3 m from the block's wall), far side X 37.6, 18 m across, 7 m walls, chamfered roof up to the legs' plinth.
- Terrain mask: `terrain_earth.gdshader` does not draw the terrain inside the tunnel's plan (X +-7.55, Y out to 41.45; only the tunnel lies below the ground)
  within `mask_range_m` = 20 km of the camera; the pit model is drawn 150 m beyond. Globals `trench_*` (project.godot) set by
  `TowerView._set_trench_mask`; the site position is a hi / lo pair like the camera's, so the offset stays exact. Physics ground stays flat.
  Beyond 20 km the footprint (< 6 px at a 60 deg view) is terrain again (no hole, a step in the ground colour only).
- Open part of the lattice: `x_brace` on all eight lower sections, `chevron` on the top one. Which brace pitch the frames
  decide: the Ship 42 photo shows diamonds 7-8 m between crossings (6.4 px/m, ~50 px), confirming the webcast pass; the floors
  stay at 5.3 m, so a brace panel spans 1.5 floors and the sections hold 3 floors = 2 panels.

## Game side (task 3)
- `StructureModels.load_for(site_id)` (`game/world/structures/structure_models.gd`): `{}` unless the site is the
  prototype's (`PROTOTYPE_SITE_ID = "starbase_pad2"`), the detail file exists and `data/models/starbase_olp2/` is not
  older than the detail file / generator scripts (file mtimes; the manifest's Blender hash cannot be reproduced in
  GDScript). Otherwise one log line per site and `TowerView` keeps its boxes. glTF is loaded at run time (`GLTFDocument`);
  the Blender axes (arm +X, Z up) become render axes once, in `BLENDER_TO_RENDER`.
- `LatticeBuilder.build(detail, profiles, materials)` (`lattice_builder.gd`; `materials` = `TowerMaterial.lattice_set()`:
  leg / brace / dark / clad): per section one MultiMesh of the corner legs (the top section's run through the head up to its
  roof plate), one per face of braces (`x_brace`: 2 diagonals + 1 horizontal per panel; `k_brace`; `chevron`: Lambda; the top
  section adds a cap horizontal), `decks` (dark unit boxes: a deck per floor level, the column bundle, stair flights, cable
  trays), `clad` (light unit boxes: the plinth on the first
  section, the cladding plates on face 0 where the section overlaps 12.2-79 m) and on the top
  section `head_frame` (four frame horizontals under the head's roof plate) and, in the far tier, `far_head_legs` (the legs from the far box's top to the roof plate). Panels = round(height / brace pitch), floors = round(height /
  floor pitch): the two are independent. The layout functions (`dark_solids`, `clad_solids`, `top_frame_members`,
  `face_members`) are public for the headless test. Sun shadows (moon procedural surface plan Task 16,
  `EarthNearShadow`; terrain-rendering.md "Near-field Sun shadows (Earth)"): the near and mid tiers cast (every member
  while the near tier is drawn, the merged mesh from 300 m), the far tier (alpha-to-coverage box beyond 5 km) does not; no
  shadow proxy. Head, chopsticks, carriage, QD arm, mount and hardstand cast and receive through Godot's lighting, the
  terrain through `terrain_earth.gdshader`'s light(). Footprint on the physics box's pad-side face and base; head top 147.9 m and
  silhouette top 146.9 m against the box's 146 m.
- Tower head (`[tower.head]`, `mechazilla_layout.head_outline / head / head_faces`, static model `rails.gltf`, the same at every distance; read on `padb_tower_top_cabin_2026-04`, user 2026-11-01). Plan: the tower's square stays at the rear (rear corner and both neighbouring corners in place), only the front (hinge) corner is pulled out along the diagonal, 1.3 +-0.3 m past the hinge axis (the axis stays, about 0.6 m past the outer cable group as the photo shows it; the two groups need >= 1.2 m to lie inside). Section, top down: roof plate 0.9 m, 0.2 m proud; on the two front edges closed plate walls of 7 vertical panels in two tones with a row of brackets along the top and a 0.6 m bottom band; floor 0.4 m on 0.4 m beams, the load beam over the axis carries the cables (underside 140.5 m, 1.4 m over the lattice top). The two rear sides stay open onto the lattice, with one chevron each drawn; the legs run up to the roof plate. Small platform at the point. The sheave block's top (138.4 m at rest) clears the underside up to +2.1 m carriage offset. Not visible on any photo: the rear of the head, the inside of the walls. Weathering stains are not drawn, only the panels' tones.
- Distance hand-over (visibility ranges, hard swap, no fade): near `legs`/`face_n`/`decks`/`clad`/`head_frame` up to
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
- Colours (user 2026-11-05 "nach Webcast-Bildern"; frames and measurements: `.superpowers/sdd/2026-11-04-moon-procedural-surface/tower-steel-frames.md`,
  crops in `screenshots/tower_reference/`), all [E], tint = albedo (sRGB of a `source_color` uniform, sun cast divided out): legs weathered
  galvanised, mid-grey slightly brown-warm, linear ~(0.16, 0.14, 0.125) (f14_2095 right leg sunlit mean 122/104/90 in warm low sun, f13_2040,
  f12_1860); braces darker, linear ~0.06 (f13_2040, f14_2095 truss pixels 37-56); plinth plate-clad steel warm grey-tan ~(0.17, 0.15, 0.12)
  (f14_2095 crop_f14_plinth, sunlit 104/102/95), its own material and instance node `plinth`; cladding plates neutral mid-grey ~0.13
  (f13_2040); QD arm dark grey matte ~0.04 (`arm_grey`); head walls / bands unchanged albedo, metallic 0; chopsticks / carriage stay matte
  black. Every one of them metallic 0, roughness 0.8-0.95 (frames: satin-matte, no mirror highlight on any member). Metal032 colour map
  modulates the legs (contrast 0.25) and braces (0.5); its NormalGL map triplanar at strength 0.5 (missing file: smooth). Open: plinth
  panel seams are not drawn (the plinth is a unit box); the far shader's `leg_colour`, `steel_colour`, `clad_colour` follow the new
  albedos. Earlier values: `docs/history/launch-structures.md` "Tower colours before the webcast frames".
- Material (`tower_material.gd`, `shaders/tower_steel.gdshader`): CC0 ambientCG Metal032 colour map (data/materials,
  `python tools/data_pipeline/fetch.py ambientcg_metal032`; missing: plain steel) triplanar in the member's own scaled
  frame + procedural grime per instance; decks dark; chopstick matte black (`black_paint`). The steel is rough and matte
  (user 2026-11-05): metallic 0, roughness 0.85 clean / 0.95 grimy, so it scatters diffusely.
- `TowerView.build` frees its children first (no duplicates on a rebuild); both arm boxes are chopsticks (the models' +Y points
  to the axis; the -across arm is mirrored by a negative scale), both pivots stand on the common axis `hinge_pivot_m`. `update_state` also places the
  carriage (hinge height + offset) and the cables. `render_check --catch_hull=<station>` hangs a static hull with fins in the arms for
  comparisons (`game/tests/catch_pose_hull.gd`, no flight). Test: `game/tests/tower_view_test.gd` (headless; the dummy
  renderer keeps no MultiMesh transforms, so layout is read from `LatticeBuilder.face_members` and the `*_solids`
  functions). Task 1 cases: lattice top 139 m, 26 floors of 5.3 m, plinth top 12.2 m, cladding plates
  12.2-79 m on the pad-side face only, silhouette <= 148 m (147.9 m), four frame members under the head's roof.

## Standing decisions (from the spec)
- Member faces are wound outward by construction in `tools/structures/blender/member_geometry.py`: vertices are not
  shared between faces, so `recalc_face_normals` gives an arbitrary result, and the steel shader culls back faces.
- Tower braces stay H sections, legs closed boxes (user 2026-11-01; photo reads H, not unambiguous at its resolution).
- Rebuild from dimensions; photos only as references in git-ignored `screenshots/mechazilla/pad-b/`, no photo pixels,
  no traced drawings, no logos.
- Measurement is scripted: `tools/structures/measure/ratio.py` reads a points file (next to the image, git-ignored)
  and scales by the 9 m booster diameter [S]. Camera solve (`solvePnP`) and silhouette IoU: full build.
- Hybrid pipeline: lattice instanced by Godot from the file, non-repeating modules by headless Blender (glTF in
  git-ignored `data/models/starbase_olp2/`), boxes as fallback. Physics numbers are the master (envelope check 0.10 m /
  0.05 m, full build).
- Prototype: placed at `starbase_pad2`'s tower box (site renamed from `starbase_olp1` / "Starbase OLP-1" in Task 5, 2026-10-31; ids mechanical in config, tests and tools).

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
- `tools/structures/tests/test_mechazilla_layout.py`: the bearing faces of both arms built at the closed pose from the PHYSICS axis (not from the
  layout's own numbers): bearing faces parallel within 0.5 deg, the physics gap apart, centred on the axis; U plan, member table; closed
  arm's surfaces >= 0.03 m off the hull, pad 0.1 m from it and ending at the arm tip, open frame (no deck, strip <= 0.8 m, top at Z = 0).
  `tower_view_test`: the pad meshes' centres through the pivots against axis + line offset + pad station (0.05 m), the pad centres 9.8 m apart, the
  actuators' ends on the carriage and the lug, length follows the opening. `python tools/structures/check_envelope.py`: inner face on the
  physics half gap, arm length, tower width / height, arm top plane, leg outer width, QD head against the hull, drawn structure under the
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
  closed light plates on the pad-side face to 79 m, the tower head. Differences left:
  the frame's plinth is a dark sloped base, ours a straight box; the frame's lattice has more members (cross braces
  inside the face, platforms with grating); legs in shade are darker than the frame's sunlit ones (the render's Sun at
  9 am is on the other side of the tower from that camera).
- X-brace pitch decided by the Ship 42 diamonds (~50 px at 6.4 px/m = 7.5-8 m) and the webcast pass: 8.025 m.

## Physics deviations (what is drawn as measured differs from the physics; input to the physics plan)
| Part | Drawn now | Physics stays | Why |
|---|---|---|---|
| Hinge, root | one axis 2 +-1 m outside the box corner, as the physics; root 64 / 57 deg, 12.1 / 9.4 m to the bend | same axis | photos read the root 50 +-15 deg (within the error); the offset's direction is the outward diagonal by choice, the photos do not fix it |
| Arm length | 24.0 m, as the physics | same | follows the axis so the tip stays 3.6 m past the catch axis; inside reach 24.5 (2) m, at or just below hinge-to-tip 27 (2) m (straight lines 25.09 / 24.38 m) |
| Fins against the arm | drawn pins and deck at the fine build's place until Task 4 | pins at station 63.0 (fin hinge 64.0 - 1.0), azimuths 0 / 180 across the arms, the 90 deg fin away from the tower; fins without contact | the drawn arm's top reaches the 0 / 180 fin plates at a catch: visible, NOT hidden |
| Tower height | head top 147.9 m, underside 140.5 m, lattice 139.1 m | box 146 m | within the error |
| QD arm | static at the connected pose, deck 81.3 m, head 1 m off the hull (swing key in the view only) | no state | needs the core's `qd_arm_angle_rad` |

## Open points
- QD arm swing over time: needs the core's `qd_arm_angle_rad` state (the view has the key `qd_arm_swing_rad`, default connected). Collar distance 15 +-3 m (photo reading 13, moved to the root's foot) and the QD root position +-1.5 m are single-photo readings; photo differences of Task 4b: rollers not resolved on the photos, no stairs / grating on the collars, no tower platforms; lamps: masts of the mount banks are drawn (none in the photos), lamp aims now go to the stack.
- Arm: root bend and width from weak
  views (no top view of a Pad 2 arm found); deck grating and rack strip not drawn; about 4.1k triangles per arm, far tier unchanged (cost:
  task 5); shock absorbers and the fin's station undecided.
- Mount render vs DSC06000 / DSC06022 / CRL05690 (`screenshots/pad-b-plan/4a_*.png`, Task 4a): the annex lacks its stepped courses and recessed door bays, vent plates and louvre boxes are plain boxes, no tunnel-floor surface detail (ribbed dark steel), no inner wall panels in the round opening, the ramp walls are drawn without the photo's plating; water in the underpass: user note 2026-10-31, probably let in and evaporating at ignition, not verified, not drawn (the floor is closed for it). Hardstand edges of the annex flanks lie under the plate (Task 5).
- Earlier render vs the frames (`sf_*.png`, before Task 4a): mount taller and steeper than the photos (the 20 m deck), plates cleaner and more regular, hoods without
  the chamfered facets, vents plain wedges, no scaffolds, vehicles, recessed door bays or the deck's fine hardware; portal
  width carries the largest plan error (ship39 reads the ramp 15 m, tower foot 26 m, NOAA 20 +-4). Still different on the tower: no platforms with grating and no cross braces inside the face, the plinth is a plain box,
  grime is not tuned.
- The far shader's `blur`, `alpha_scale` and colours are fitted to the merged tier at 900-1300 m for one sun / sky
  condition; other light may show a small step (< 2 % luminance measured).
- Single-view ratios in oblique photos (face width, depth) carry several percent error; leg, wall, pitch and truss
  pitch are estimates (not measurable from the references). The full build re-measures with a camera solve.
- Brace width: the spec asks that no member is thinner than ~1.5 px before it fades; the brace (0.5 m) is 1.30 px wide
  where the near range ends on the 60 deg / 900 px desktop view (2.5 px at a 1500 px VR focal length). The hand-over is a
  hard swap to a merged mesh with the same geometry, so the 1.30 px is not visible in the luminance, structure or shimmer
  measures; recorded as a deviation from the spec's number.

## Lights (fine build task 4, spec G)
- Positions follow the tower box and the squares' axes (`ref` face = the box's pad-side face, axis = the ring axis; floating test `tower_lights_test`: every lamp within 1 m of the tower box, the foot block, the mount deck, the ground or a mast / cross arm).
- Fixtures: `config/structures/starbase_olp2_lights.toml` (own file: the lamps do not feed the Blender models, so a change never stales them;
  validator `detail_file.load_lights`). Groups in the site frame (`ref` face / axis, along / across / z, count + steps, mirror): obstruction
  lamps (4 on the head's roof corners 148.1 m, 4 at 72 m), head floods (4, on the corner legs just under its floor, 140.2 m), the two face-edge lamps at 89 and 83 m (`spacex-starship-mdr-100`: lamps
  ~89 / ~83 m on the pad-side face's edge, 4.9 px/m from the 144 m tower), 14 lattice lamps every 9 m, tower-foot floods (2), two mount
  banks of four lamps (z 27 m on masts on the ground at X 13 m, Y +-24 m of the squares' axes, aimed at the stack; `padb_night_floodlights_mount`, `HN635FUWYAAzbOB`), deck lamps (4). All
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
- `tools/structures/tests/test_mount_layout.py`: base 44.6 x 40 m, deck 18.7 m, ring top = physics height, ring 10 / 11.6 m, 20 clamps, aprons 26 m wide / 33 m
  out as wide as the portal foot, floor -9 m, portal open over its whole width on both sides, rectangular doors, hoods down the front face, hardstand round the site and off the pit, foot on the physics axis, every polygon planar. Stack clearance (Review Focus 1): sampled points of
  every mount polygon (6 per triangle) against the hull (4.5 m radius above the ring top, +0.04 m) and the 33 booster engines' bells below it
  (rings 0.88 / 2.55 / 4.40 m, bells 1.30 m inner / 1.32 m lip, exact bell contour, margin 0.04 m): `test_booster_stack_clears_deck_ring_and_qd` green with the 10.0 m opening (smallest passing 9.63 m); the 8 m and the earlier 9 m openings are caught (negative controls). The physics has no mount opening (no collision shape to follow). `tower_view_test`
  (309 cases): model dimensions, deck at 19 m, ring opening radius 5.0 m, mask equals the drawn footprint (+-0.06 m), no script error in the log.
- `game/tests/trench_mask_check.gd` (Review Focus 4; GPU): per distance 30 m .. 400 km three frames (terrain alone / mask on, no models / as drawn);
  the pixels the mask changes (isolated noise filtered, 150 frames wait for the node morph) lie on the projected footprint (+3 px) at every
  distance (0 px outside), 100 % of them are covered by the model (per colour channel: the hazed dark ramp matched the hole's luminance at 5-12 km, frame read), none beyond 20 km. `fine3f_trench_mask.txt`.
- Cost (`tower_cost_check`, RTX 4090, 1600x900): the whole tower incl. mount and trench adds +0.08 ms GPU at the 60 m close-up (0.68 vs 0.60), 0.01 ms at 6 km; the terrain mask itself within +-0.01 ms (noise level).
  The discard removes early-z on the terrain (not measurable here). `TowerView.build` 144 ms.
