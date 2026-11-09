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
