# Cockpit: superseded numbers and approaches

## Shot cameras before round 4 (fix rounds 1-3)
- GoPro (discovery2 = discovery1 framing; round 3: the console block moved 0.35 m down X, so the camera moved with it: X 2.6545 -> 2.3045, same fit): least-squares fit (scipy, own equidistant fisheye r = f theta, principal point at the image centre) on the 12 active-area corners of the
  three screens (TOML positions; pixels read by eye from the 4128 x 2322 photo, +-5 px), 7 parameters. Result in the capsule frame: position (2.6545, -0.4837, -0.0251),
  forward (0.4857, 0.8742, 0.0073), image right (-0.0209, 0.0200, -0.9996), f = 2085 px/rad (113.4 x 64 deg), reprojection error rms 20 px, max 36 px of 4128 (~0.5 %).
  The fit puts the camera 0.50 m from the screen plane, 0.2 m toward the hatch of the eye points (not the review's 0.64 m estimate; position and f are correlated, the fisheye
  distortion is tied by the three coplanar screens).
- Hatch (crew1_1), round 3: pose (2.11, 1.70, 0), level, looking -Y, image up +X, 3:2, ~87 deg horizontal. Fitted by eye on four feature groups after the leg / console changes (windows, console underside slab, tunnel rim / ceiling dish, helmets / headrests; 5 points per group read on the 2576 px side-by-side): residual ~20-50 px of 1288 (< 4 %), X raised 0.06 m against the review's (2.05, 1.70). Earlier review estimate: (2.05, 1.7, 0), level, looking -Y, image up +X, 3:2,
  ~87 deg horizontal (65 deg vertical). Exterior hull is left visible (single-sided, culled from inside; hiding it would remove the window shadows and flood the floor with sun).


## Fix round 3 console block, ceiling and helmet (superseded by round 4)
- Legs (crew1_1 fit of the review): `crew_figure.toml` `thigh_raise_deg` -25 (thigh 10 deg above the hatch axis; the seat pan is built along the thigh, `build_seat(pan_raise_deg)`), `knee_angle_deg` 105
  (shin 65 deg below the hatch axis). `[[seat]]` `foot_rest_m` s2/s3 (1.615, 0.733, +-0.35), s1/s4 (1.535, 0.733, +-1.05), `foot_rest_normal` (0.9063, -0.4226, 0) = up the shin.
  Residual against the photo: rest centre X inside 1.50-1.60 +-0.1, Y 0.13 above the 0.50-0.60 band (the Drillis-Contini thigh of 0.436 m ends 0.45 m ahead of the hip joint; closing it would move the hip joint, i.e. the seat, 0.1 m back: not done).
  The foot rest is now a flat carbon tray (top face in the plane through `foot_rest_m`, heel stop and toe lip) with the two rails, not an upright frame; seen nearly edge-on from the hatch as in crew1_1.
- Console block and ceiling: the whole console group (housing, screens, tablets, lockers, buttons, lamps) and the ceiling plane moved 0.35 m down X ([E], reviewer fit on crew1_1 / discovery10: console 0.35, tunnel rim 0.5 m too high
  relative to windows and heads; ceiling taken with the console so the upper panel still ends 0.05 under it). `[cabin] profile_m` now ends the side wall at x 3.00. Eye to centre-screen distance is 0.44 m (was 0.70): the shoulder-to-index reach of 0.78 m
  is still more than the shoulder-to-screen distance (arm bent, ~75 % extended), so the screens stay reachable; the commander eye / seats were not moved. The GoPro shot camera moved with the block (overlay stays aligned).
- Helmet (user finding): opening +-58 deg x -30..+42 deg (rounded, square-to-disc map), closed copper visor seated in it (30 mm dome, roughness 0.38), one continuous 40 mm white rim bead swept around the opening (smooth, no segments),
  white brow band / cheeks / chin from the shell, dark pivot housings on the cheeks (`pivot_housing_m`, `pivot_el_deg`), 36-sided shell. Like-for-like shots `helmet_34` / `helmet_side` (38 / 62 deg, 0.75 m, 48 deg fov) with `helmet_vs_detail2`, `helmet_vs_detail1`, `helmet_vs_demo1_2`.
- Materials: shelf modules albedo 0.07 (was 0.32), lockers 0.02, tablets 0.03, speculars 0.03-0.05; `fill_ceiling` omni follows `ceiling_x_m` - 0.25; the two console lamps got shadow bias (shadow_bias 0.12, normal bias 3); ring light lifted 15 mm (was 4 mm);
  seam lines fade at grazing view. `cockpit_shots.gd` debug options `--hide=<node,..>` and `--hide_exterior`.
- Open: the radial moire stripes at the top edge of `pilot_eye` (above the dark upper panel, where the view runs up the forward tunnel) persist after the ring light lift, seam fade, lamp shadow bias and a coarse hatch-plate fan; they stay with the ring light, the
  shell and the exterior hull hidden one after the other, so the cause is not found (suspect: the world / sky render seen through the tunnel). Not blocking.


Why superseded: the 0.35 m move put the commander's eye 0.44 m from the centre screen (reach photos: 0.6-0.75 m) and rested on reading the tunnel's far rim as its
near rim; round 4 replaced it with one joint fit (docs/subsystems/cockpit.md "Fix round 4"). Earlier values: ceiling 3.35 (rounds 0-2), 3.00 (round 3);
s2 hip (1.80, -0.02, 0.35), eye (2.447, -0.351, 0.35) = hip + 0.72 up + 0.10 forward, eye forward 20 deg above the chest normal; console normal (-0.819, -0.574, 0),
centre screen centre (3.021, 0.050) rounds 0-2, (2.671, 0.050) round 3; foot rests (1.615, 0.733) / (1.535, 0.733) round 3; helmet opening +-58 x -30..+42 deg,
rim 20 mm, grey pivot housings 24 x 70 x 50 mm.

## Fix round 2 helmet (superseded by rounds 3 and 4)
- Helmet: shell half width 0.150 x depth 0.170, visor +-72 deg x -28..+58 deg with a 45 mm dome bulge, white rim ring (tube loop), collar r 0.112. Visor stays closed and copper (dark coating, metallic 0.8).
  Helmet shell albedo neutral (0.80). Materials: housing matte near black (specular 0.1; the beige sheen came from the metallic 0.1 / roughness 0.55 specular of the lamps), tablets no metal sheen.

## Rounds 4-5 helmet shell and the fix-round-5 grey parts (superseded by Task 5b)
- Round 4 / 5 helmet against eva_suit_detail2: egg-shaped ellipsoid shell (half width 0.150, half depth 0.170, above 0.20, below 0.17, `egg_taper` 0.08), cut flat at
  -0.75 of the lower half, a low crown ridge (6 mm, 1/e width 30 mm, fading before the visor), opening +-62 deg x -40..+38 deg (square-to-disc map), closed visor
  dome 35 mm, white frame bead r 16 mm with the black seal r 12 mm 4 deg inside, round pivot buttons 13 deg behind the opening at el -30, grey collar ring r 0.112 x 0.07 m.
  Why superseded: the user's clean side profile (spacex-anzug.jpg) showed the shell 0.375 m deep against 0.28 m measured at the opening-height ruler ("bulbous"),
  no crown ridge, a faceted rear, a sloped lower edge with the grey hood under it and a much larger opening (to the temple).
- Fix round 5 open point "boots, seal and pivot read ~0.5 sRGB" (AgX pivot at field key 1.2): under the unified appearance model (look fixes, field key 0.2) the
  boots read 0.15 sRGB in hatch_crew1_1 (Task 5b re-check), so the point is closed.

## Crew figure of Task 4 (superseded by Task 8b, 2026-11-09)
Proportions were Drillis and Contini 1966 fractions of a 1.78 m adult; the suit was lofted capsules per segment with rigid weights (one bone per
segment): creases at the elbow and knee, the arm root a capsule end against the torso (user 2026-11-09: "very inhuman"). Rest pose: forearms along the
armrests, palms down (it read as a shrug in the hatch shots); hands were boxes of capsules with one merged slab for the middle / ring / little fingers
(31 bones). The figure's eye was within 0.20 m of the config eye (round 4 made it exact). 13.2k triangles. Section text before Task 8b:
### Crew figure (Task 4)
- `config/cockpits/crew_figure.toml` (every value tagged; proportions = Drillis and Contini 1966 fractions of a 1.78 m adult [E]; suit look from
  `sheet_suit` / `eva_suit_detail1-4`), `tools/structures/blender/gen_crew_figure.py` (+ pure `figure_layout.py`: bones, seat basis, manifest block) writes
  `crew_figure.gltf` into the Crew Dragon set (13.2k triangles since Task 5b: faceted helmet 48 x 30, hood, thin beads). `vehicle_spec.load_dragon` loads it into `spec["crew_figure"]`; `VehicleModels` returns
  the meshes under `"crew_figure"` and counts the TOML for staleness; manifest block `crew_figure` (`bones` head / tail / parent, `eye_m`, `materials`, `frame`).
- Frame convention: glTF origin = the seat's hip reference point (`[[seat]] hip_m` = seat-back plane meets the pan surface); **x = up the seat back, y = the
  seat's chest normal, z = the crew's left**. The game places it at a seat with the basis columns `((cos a, -sin a, 0), (sin a, cos a, 0), (0, 0, 1))` in
  the capsule frame (a = `back_angle_deg`; `figure_layout.to_capsule` / `seat_basis`). The hip joint sits 0.10 above and 0.12 ahead of the origin.
- One skinned mesh node `crew_figure` under the armature `crew_figure_rig`, rigid weights (one bone per segment), surfaces suit / boots / gloves / helmet /
  `visor` (closed copper solid, own surface and material). Bones: `hips, spine, chest, neck, head, shoulder / upper_arm / forearm / hand / index_1..3 /
  thumb_1..2 / fingers (merged middle, ring, little) / thigh / shin / foot`, `_l` at +Z. Rest pose = seated (round 3: thighs 25 deg below the pan plane = 10 deg above the hatch axis, interior knee 105 deg = shins 65 deg below the hatch axis (crew1_1 recliner posture; was 15 deg / 110 deg),
  forearms along the armrests, hands palm down).
- Tests (`test_crew_figure.py`): bone set, arm reach shoulder to index tip 0.778 m against 0.44 x stature = 0.783 (+-0.05), mesh inside the s2 seat envelope
  (nothing more than 1 cm below the pan or behind the back), visor a closed own surface, glTF skeleton = layout, figure eye within 0.20 m of the config eye.
- Simplifications: rigid skinning (creases at elbow and knee until blend weights are added), suit as lofts (stripes, collar, patches, flag by texture
  later), hands are boxes of capsules, no head or face inside the helmet. The figure's own eye point (`eye_m`) is 0.06 above and ahead of the helmet centre
  and about 0.12 m from the config eye (the config eye derives from the hip point, the figure's hip joint is 0.12 ahead of it); the player's eye stays the config.
- Look shots: `tools/structures/blender/preview_crew_figure.py` (Workbench; builds the cabin, places the figure at s2): `screenshots/m7-cockpit/figure/`.

## Tasks 8 / 8b before fix round 1 (2026-11-09)
- Camera keys: five blank keys (`c_upper_1`, `c_upper_2`, `c_lower_1`, `c_lower_2` + `c_lower_round` under the centre screen, right hand) = slots 1..5 in
  catalog order (chase, stage1_aft, stage2_engine, dragon_nose, dragon_trunk). Replaced by the user's second decision: four left-hand keys, dragon_nose last
  in the catalog and only on the on-screen button.
- `r_upper_5` gap hidden as a 40 mm test tolerance (34 mm short after Task 8, 30.7 mm after 8b); now a named known gap in the test.
- Leg posture: thigh_raise_deg -25, knee 105, legs in the sagittal plane (knees together; thigh exactly on the 10 deg photo-band edge), foot rest
  hip + (-0.185, 0.753), normal (0.9063, -0.4226, 0). Hand inward on the sagittal thigh, wrist_m (0.152, 0.322, 0.115).
- Suit: torso u = column index / 32 (equal angles; the panels squeezed into the section corners), chevron in the light insert grey 18 mm, gloves light grey
  0.62, shoulder / hip shapes weighted by bend angle only (any swing direction), elbow weight blend +-0.05 m, cavity shade 0.35, one joined mesh with the 16
  morph targets on every primitive (10.5 MB .bin), the non-index fingers flat while pointing.

## Console body and struts before fidelity Task 4 (2026-11-21)
- `[console]`: centre_m (2.5226, -0.0536, 0) = centre screen centre + 0.015 up; width_m 1.32 [E] (discovery2 iPad scale); height_m 0.50 (0.265 above, 0.235 below the
  centre screen centre); depth_m 0.15 [E] (crew11 / crew13 "thin box"); strut_z_m +-0.55 [E] (round 4 crew1_1 post bases). The body was a sharp box (housing, back
  plate 3 mm, underside plate 3 mm) and the two struts were straight tubes r 0.02 from the back top edge vertically (+X) to the ceiling plane (0.66 m long).
  Lamp strips 12 mm inside the back edge. Task 4 showed the posts lean back toward the crew to the ring's rim and the body is 0.205 deep and 1.40 wide.
- `photo_cameras.toml` round-4 rows: crew1_1 (1.839, 1.454, 0.013), forward (0.14263, -0.98978, 0), fov_h 96.449 [E]; discovery2 (2.2437, -0.6322, -0.0251),
  forward (0.32649, 0.94528, 0.00729), up (0.94491, -0.32614, -0.02800), fov_h 113.412 [E] (rms 20 px).

## Console values of fidelity Task 4 before its fix round 1 (2026-11-21)
- `[console]` as Task 4 built it: centre_m (2.5083, -0.0393, 0) [D], width_m 1.40 [D] (crew1_1 back_end points, which lie on the recess contour), height_m 0.540
  [D] (u_top 0.265 prior + u_bot -0.2754), hood_depth_m 0.205 +-0.021, underside_profile [[0, 0], [0.205, 0.022]] [D], strut_z_m +-0.549, post_top_m (-0.632, 0.751)
  (r 0.981, prior-dominated: the collar poked ~2 cm through the wall at the ring's rim), post_foot_m (0.022, 0.069, 0.050, 0.319) [D]. Fix round 1 tags every
  value by `constrained_by_data` ([E] where prior-held or unseen), adds discovery10, holds the top radius at 0.945 (collar inside the ring) and refits.
- Lamp strips: in front of the back lip's fillet at lateral (0.10, 0.36), (0.42, 0.56) m: 15-18 px of 800 under and the outer ones ~60 px outside the photo's.
- `photo_cameras.toml` Task 4 rows: crew1_1 (1.8568, 1.4261, 0.0047), forward (0.17176, -0.98513, 0.00301), fov_h 96.615; discovery2 (2.2102, -0.6497, -0.0327),
  forward (0.35901, 0.93322, 0.01485), fov_h 114.972.
- Docs claim then: "render from the fitted camera lies on the photo" with 1.33 / 5.60 px rms; those were fit-model residuals, the body outline was not compared.

## Screens, shelf and keys before fidelity Task 5 (2026-11-21)
- `[[screen]]`: active area 0.382 x 0.215 [E] (training2 finger scale, 17.3 in class), aspect 1.7778, outer centres +-0.402 m (`screen_gap_m` 0.020 [D]: 2 x 0.007 bezel
  + 0.006 seam, "bezels nearly touch" in discovery2). No bezel / corner radius / recess: the screens were bare quads 2 mm in front of the housing. Task 4 found
  discovery2 / 10 wanting pitch 0.455 and glass half width 0.203 (outer corners 87-143 px off).
- Shelf rule of fix round 2 (superseded): `shelf_height_m` 0.10 and `shelf_top_gap_m` 0.0175; the buttons were grouped into separate light grey modules by
  `cabin_layout.shelf_modules` (`module_split_m` 0.012, `module_margin_m` 0.004), dark housing in the gaps; test `test_shelf_is_a_ten_centimetre_row_of_separate_modules`
  (>= 6 modules, >= 3 mm gaps); `test_screen_gap_is_one_to_two_centimetres`.
- `[[button]]` centres were read along the screens' edges in discovery10 as px fractions (linear, ignoring the fisheye) on the 0.402 pitch: rows 0.17 / 0.20 m below
  the centre screen centre, faces 0.029-0.030 m proud, keys 6 mm (`BUTTON_PROUD_M`, cover 12 mm). cam1 / cam5 / r_upper_1 / r_upper_5 were round 0.016 keys; the 2 x 3
  blocks 0.024 x 0.016; r_upper_* / r_lower_* without labels. The outermost keys sat at +-0.540 m (now +-0.605).
- `[[tablet]]` centres +-0.367 m along the row, 0.228 up, 0.060 proud (round 4).
- `photo_cameras.toml` Task 4 fix round 1 rows: crew1_1 (1.8362, 1.4265, 0.0035) fov 97.054; discovery2 (2.2152, -0.6499, -0.0313) fov 115.094; discovery10
  (2.2147, -0.6291, -0.0190) fov 118.399.
- `console_pointer_test.gd` KNOWN_REACH_GAP_M: only r_upper_5, 30.7 mm.

## Console materials before fidelity Task 6 (2026-11-21)
- Flat colours per node (CockpitView): `console_housing` linear 0.028 (roughness 0.9), `console_shelf` 0.07 with a red strip mark (`SHELF_STRIP_*`: 10 mm, 40 cd/m2)
  across the whole lip, `console_back` 0.30 sRGB silver (metallic 0.25) with the dark vent band by shader (`CONSOLE_VENT_UP_M` -0.11..+0.07 from the centre screen centre,
  +-0.58 m; it covered the silver), the posts the same silver, `console_under` 0.10, `console_lockers` 0.02. Console UVs were the local planar (x, y) in metres.
- Key labels: the text block was centred by the font box ("HG" x 1.15 line height), which put the ink 0.7-1.1 mm above the key centre.

## Console ends, lockers and tablets before fidelity Task 7 (2026-11-21)
- Console body: square ends at the full 1.40 m everywhere (`width_m` [E] "lower bound"); in crew1_1 the render's ends were 16-47 px inside the photo contour near
  the top and 30-49 px outside below the lip. No wall recess: the shell was one white surface of revolution (cabin 64,368 triangles).
- `[[locker]]` 5 / 6: centre (+-0.562 along the row, 0.216 up), 0.15 x 0.165 [E] (discovery2 120 x 125 px at 0.00132 m/px); the latch was a texture constant
  (`cabin_textures.LATCHES` (+-0.4845, 0.217)). The ends were silver behind o -0.12 (`END_SILVER_O`).
- `[[tablet]]`: no roll (fit 1.3 / 1.9 deg not built), a plain 8 mm slab with a 12 mm post 6 cm straight back, dark flat material (linear 0.03, rendered as black
  slabs in discovery2).

## Wall recess and console end fit before fidelity Task 7 fix round 1 (2026-11-21)
- In edge fitted per side (left / right averaged) from every crew1_1 `REVEAL_IN_PX` row, the console band rows 170-220 included: 123.45 +- 0.46 deg,
  -81.9 +- 3.9 deg/m, 18 +- 56 deg/m2 over x 2.824-3.139 (l/r 0.52 deg, rms 0.97 deg). The same rows were also back-projected onto the console's back face as
  equalities, so one image line stood for two features ~0.8 m apart (review Important 1).
- `end_taper` [0.6063, 0.590, 0.543] was tagged [D] from a plane fit (`recess.fit_taper`) of the L/R-averaged back rows 186-221 and the left underside rows
  230-260, +-0.015 / 0.025 / 0.057, rms 1.6 mm. Values unchanged by the refit (now [E], an upper bound).
- `RECESS_TINT_FROM` 0.15 (the colour line 2-8 px inside the photo's panel edge). The cabin glTF loaded with Godot's default mesh compression (seam sparkle).
