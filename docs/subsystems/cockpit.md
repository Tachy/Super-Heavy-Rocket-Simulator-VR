# Cockpit (M7): Crew Dragon cabin

Spec `docs/superpowers/specs/2026-11-07-m7-cockpit-crew-dragon-design.md`, plan
`docs/superpowers/plans/2026-11-07-m7-cockpit-crew-dragon.md`. State after plan Tasks 1-9 (commits db33f654..15a1fc24; VR acceptance pending): typed tables, config loader, Blender cabin and console generator, crew body and suit, cabin in the game, camera mode, external cameras, console pointer and hand, HOTAS props. Superseded numbers: `docs/history/cockpit.md`.

## Model
- `config/cockpits/crew_dragon.toml`: `[cabin]` (inner wall of revolution, floor, ceiling, forward tunnel, windows /
  hatch in the exterior table's convention, window reveals), `[[seat]]` s1..s4, `[eye]`, `[console]` (housing slab),
  `[[screen]]` left / centre / right, `[[tablet]]`, `[[light]]`, `[[button]]` (39, incl. the centre handle), `[hotas]`.
  Every value line is tagged [S] / [E] / [D] with its derivation (`tools/structures/detail_file.value_lines_without_tag`
  finds none untagged).
- Frame: capsule stage body frame of `vehicle-models.md` "Crew Dragon": X = station above the heat shield apex (nose
  +X), (Y, Z) = r (cos az, sin az), az 0 = side hatch. The crew face the hatch side and the console: Y toward the hatch,
  **Z = az +90 = the crew's left**. Seats s1..s4 from the crew's left (s1 at +Z), s2 = commander (player), s3 = pilot.
- Buttons: `panel` = the screen above (its normal / up apply), `row` 1 upper / 2 lower, `shape` round / rect,
  `kind` push / covered / handle, `function` only with evidence, else `"none"`; the four BLANK-faced keys at the left
  screen's right edge (`c_upper_1`, `c_upper_2`, `c_lower_1`, `c_lower_2`) are `camera` with `camera_slot` 1..4 (user decisions
  2026-11-09: camera selection on unlabelled keys; second decision: four keys, left hand only; the 18 labelled keys cam1..cam18 keep their ids
  and are `none`, so is the blank `c_lower_round`).
  Known functions: `abort` (centre handle), `chute_mains_only`, `chute_drogues_and_mains`, `backup_g_meter`; nothing is
  wired in Task 1.
- `[hotas]`: deliberate deviation (real Dragon has none): T.16000M stick on the commander's right armrest, TWCS throttle
  on the left; grips as offsets from the eye point in capsule axes, `props = true`; calibrated in the VR run (Task 9).

### Provenance of the key numbers
| Quantity | Value | Tag | Derivation | Uncertainty |
|---|---|---|---|---|
| Side wall | 0.18 m normal | E | crew1_1 window wells 0.15-0.25 deep; window spacing in crew1_1 gives inner r 1.43 at x 2.38 (exterior 1.62) | +-0.05 |
| Inner profile | exterior `profile_m` offset by the wall | D | closest exterior point 0.094 m (>= 0.02 required) | as wall |
| Floor | x 1.15 | E | Wikimedia: cargo C5-C7 under the seat pans, 0.5 m below the hatch sill | +-0.15 |
| Ceiling / tunnel | x 3.48 / r 0.45 to x 3.66 | E / D | round 4 joint fit on crew1_1 (tunnel far rim, dish far edge, 2-3 px); IDSS 0.8 m passage + trim; hatch face 0.08 under the recess floor 3.74 | +-0.08 |
| Seat pitch | 0.70 m (Z +-0.35, +-1.05) | E | Wikimedia: pitch / seat-back width 1.2, back 0.58 m | +-0.05 |
| Seat hip (s2 / s1) | [1.474, -0.307, 0.35] / [1.444, 0.132, 1.05] | D / E | s2: the eye minus the figure's own eye point (0.812, -0.288); outer seats 0.439 toward the hatch and 0.03 lower (crew1_1 fit: head spacings and helmet widths; floor clearance of the foot trays) | +-0.1 |
| Back angle | 35 deg from +X toward -Y | E | crew1_1 torso foreshortening from the hatch, knees above the hips | +-10 deg |
| Eye (s2) | [2.2857, -0.5945, 0.35] | D | centre screen centre + 0.55 along its normal + 0.23 along its up (discovery2 GoPro geometry, reach photos): 0.69 m to the centre screen centre | +-0.08 per axis |
| Eye forward | (0.380, 0.925, 0): 22 deg above horizontal, 22.7 deg off the screen normal | D | straight ahead in the seat plane toward the screen plane | +-10 deg |
| Screens | 0.382 x 0.215 m (16:9) | E | training2 finger scale (index finger ~0.072 m on the glass) 0.35-0.40 wide; aspect 1.76-1.88 measured | +-15 % |
| Console placement | centre screen centre (2.512, -0.043, 0); `[console] centre_m` = that - 0.0062 along up (2.5076, -0.0386, 0) | E (block) / D (shift) | round 4 joint fit on crew1_1; the Task 4 free fit (crew1_1 + discovery2 + discovery10) reads (2.537, -0.032) +-0.029, within 1 sigma, block not moved; the 6.2 mm shift along up is the fitted u_bot (see Console body); every console item moved rigidly with round 4 | +-0.06 m, +-4 deg |
| Screen layout | centre on the centre line, gap 0.020 (fix round 2, was 0.045), the three screens coplanar (console straight, fold_deg 0, user decision 2026-11-07) | S / D | user decision 2026-11-07 after photo check (discovery10 top-down, crew2_1, inspiration4_1: one straight front); gap discovery10 (90 / 780 px) | +-0.1 m, +-5 deg |
| Console tilt | `tilt_deg` 45.0: up axis from +X toward -Y; the generator builds the body on it (`cabin_layout.console_frame`, refused if the centre screen differs > 0.5 deg) | D | Task 4 free fit 46.3 +-1.2 deg (data-constrained), built 45 within 1.1 sigma | +-1.2 deg |
| Console body | 1.40 x 0.542 m, back face 0.221 behind the screen plane, all edges filleted r 0.04, square sides, underside falls 0.059 to the back lip | E / D | Task 4 fit ("Console (fidelity Task 4)"), tags by `constrained_by_data`: hood depth 0.221 +-0.011 and u_bot -0.2772 +-0.0021 [D]; width (lower bound, the crew1_1 ends lie on the recess contour), u_top 0.265, underside drop (0.059 +-0.036, crew1_1 lip only), fillet r, draft [E] | width +-0.06, u_top +-0.05, drop +-0.04, r +-0.02 |
| Ceiling posts | bases Z +-0.555 on the body top, tops on the ceiling ring at (Y -0.579, Z +-0.747); rod r 0.017, collar r 0.039 x 0.04, sleeve r 0.022 x 0.066, funnel foot r 0.051 x 0.306 | D / E | Task 4 fit on crew1_1 (the only photo showing them): base Z +-0.008, top angle 52.2 +-0.7 deg, radii from px widths [D]; top radius 0.945 [E] (along crew1_1's ray; set so the collar stays inside the ring's outer radius 0.997), collar / sleeve / funnel lengths [E] | radii +-0.003-0.004, top radius +-0.03, lengths +-50 % |
| Tablets | 0.1348 x 0.2032 m | S / E | Apple iPad mini spec; model assumed from the shape | - |
| Button positions | rows 0.17 / 0.20 m below the screen centres | E | discovery10 x along the screens' lower edges (fisheye: fractions per screen) | +-0.02 along, +-0.03 across |
| HOTAS grips | hip + 0.32 forward + 0.28 up the back = 0.09 m forward, 0.55 m below the eye (seat axes), +-0.25 / 0.27 lateral | E | armrest, forearm reach | calibrated in Task 9 |

### Crew figure (Task 4, body and suit Task 8b)
- `config/cockpits/crew_figure.toml` (every value tagged), generator `tools/structures/blender/gen_crew_figure.py` with the pure modules `figure_layout.py`
  (bones, seat basis, joints, manifest block), `figure_body.py` (the body / suit hull), `figure_measure.py` (tape measures) and the baker
  `tools/structures/suit_textures.py`; writes `crew_figure.gltf`, `suit_albedo.png`, `suit_wrinkle.png` into the Crew Dragon set. `vehicle_spec.load_dragon`
  loads the TOML into `spec["crew_figure"]`; `VehicleModels` returns the meshes under `"crew_figure"` and counts the TOML for staleness; manifest block
  `crew_figure` (`bones`, `eye_m`, `materials`, `frame`, `joints`, `joint_order`, `suit_material`, `suit_maps`, helmet / visor entries). Skinned mesh
  nodes on one armature (fix round 1): `crew_figure` = the suit (the only mesh with morph targets) and `crew_figure_parts` = the rigid parts (
  hood, glove lock rings, gloves, boots) plus the two helmets on the head bone (Task 9 VR fix B, manifest `crew_figure.helmet_nodes`): `crew_figure_helmet` (visor closed:
  every figure but the player's) and `crew_figure_helmet_open` (the player's commander s2: see "Player's open helmet" below).
- Frame convention: glTF origin = the seat's hip reference point (`[[seat]] hip_m` = seat-back plane meets the pan surface); **x = up the seat back, y = the
  seat's chest normal, z = the crew's left**. The game places it at a seat with the basis columns `((cos a, -sin a, 0), (sin a, cos a, 0), (0, 0, 1))` in
  the capsule frame (a = `back_angle_deg`; `figure_layout.to_capsule` / `seat_basis`). The hip joint sits 0.10 above and 0.12 ahead of the origin.
- Eye contract: the figure's eye point (helmet centre + 0.03 up, 0.06 ahead) = (0.83, 0.23) in the figure frame = `[eye]` minus seat s2's `hip_m` (tested < 1 cm).
- Skeleton (35 bones): `hips, spine, chest, neck, head`, per side `shoulder / upper_arm / forearm / hand / index_1..3 / thumb_1..2 / fingers, fingers_2,
  fingers_3 (middle / ring / little, three phalanges on shared bones) / thigh / shin / foot`, `_l` at +Z. Rest pose = seated launch posture (fix round 1, crew1_1):
  thighs 21 deg below the pan plane, interior knee 102 deg, then the knee swings OUT about the hip-ankle line by `knee_out_deg` 20 (`figure_layout.leg_joints`:
  ankle and knee angle kept; knees 0.35 m apart, the boots side by side on the tray 0.18 m apart); seen from the hatch the thigh is 12.3 deg above and the shin
  62.3 deg below the hatch axis (photo bands 10-20 / 60-65.5, >= 2 deg margin; before: exactly on the 10 deg edge). The upper arms hang (12 deg from the torso
  axis), the elbows beside the torso, the hands palm down on the thighs (`[pose] wrist_m` on the sagittal thigh, riding along with the knee swing; `elbow_pole`,
  KSC-20190731-PH_SPX01_0002 / crew1_1). The seats' `foot_rest_m` / `foot_rest_normal` follow the ankle (hip + (-0.152, 0.754)).

### Player's open helmet and own head (Task 9 VR fix B)
- User VR run 2026-11-11: "the commander shall have his visor open so he can see out of the helmet". The game never hid the player's head, so the VR camera sat inside
  the closed visor (its inner surface tinted the whole view). Now: the commander (s2, the player body) wears `crew_figure_helmet_open`; s3 / s1 / s4 keep the closed copper
  visor (user decision 2026-11-07 unchanged). `CockpitView.figure_helmet_open(seat)`.
- Open helmet (`gen_crew_figure.build_open_shell`, `figure_layout.visor_opening_span` / `visor_stow_*`): the shell with the face opening cut out along `visor_outline`
  (per column the lower run ends and the upper run starts on the outline; extra columns at every outline row's half width, so the cut follows its corners), a dark
  lining (`boots` black) `helmet_liner_depth_m` 10 mm inside with a lip along the opening (under the white frame bead), and a dark hood lining (4 mm inside the hood, a floor
  facing up), so the opening shows a dark empty interior (no face is modelled). Frame bead, seal and pivot buttons as on the closed helmet.
- Raised visor: the visor grid turned up and back over the crown by the opening's height + `visor_stow_margin_deg` 5 (100.5 deg about the helmet centre's lateral axis),
  `visor_stow_gap_m` 10 mm off the shell, kept `visor_stow_lift_deg` 3 above the lower edge and out of the opening (its side parts ride along the opening's upper edge);
  jsc2024e066389 / conv_2ks5k (DM-2, visor up: the raised visor rides over the crown, its lower edge at the brow). A rotation about the real pivot buttons does not fit our
  shell (the chin edge is 4 cm from that axis), hence the centre axis. Drawn as see-through coated glass (`cabin_glass.gdshader` `tint` = the visor albedo,
  `RAISED_VISOR_OPACITY` 0.15): 066389 shows the white shell through it.
- Own head: `CockpitView._process` hides the player's head (helmet, visor, hood lining: all on the head bone, collapsed) while the active camera is within
  `HEAD_HIDE_RADIUS_M` 0.30 m of the eye point, whatever the view (VR, desktop pilot eye); `set_player_head_hidden(true)` still hides it from anywhere (shots). The figure's
  head does not follow the headset, so a helmet drawn around the camera would stay put while the view turns; nothing of the own helmet shows. Eye position unchanged
  (delta 0: the eye contract helmet centre + 0.03 up / 0.06 ahead still holds; with the head hidden no rim can cut into the view).
- Shots: `cockpit_shots.gd --visor` (`helmet_open_34 / _front / _profile / _rear34 / _hatch`, `pilot_eye`, `_up`, `_left`, `_right`, `_down`), `screenshots/m7-cockpit-vrfix/after/`.
- Tests: `test_crew_figure.py` (`test_opening_span_inverts_the_outline`, `test_open_helmet_has_the_face_opening_and_a_dark_lining`,
  `test_raised_visor_rides_over_the_crown_and_clears_the_view`; the figure tests read suit + parts + the closed helmet), `cockpit_view_test.gd` `_check_helmets`.

### Crew body and suit (Task 8b)
- Measures: ANSUR II (2012 U.S. Army survey, male public database, 4082 subjects, public domain; `sources.toml` `ansur2_male`, CSV in
  `data/anthropometry/`): the **median (50th percentile) of each measure**, `[anthropometry]` in metres (stature 1.755; per-measure medians, not one person).
  Segment lengths: upper arm = acromion-radiale 0.335 (the bone starts at the acromion), forearm = radiale-stylion 0.267, thigh = trochanter - knee
  0.408, shin = knee - malleolus 0.418, torso = acromion - trochanter 0.540. Arm reach shoulder to index tip 0.802 m.
- Topology: the edge-loop layout of Blender Studio's Human Base Meshes (CC0, reference only, `blender_human_base_meshes`; no file downloaded, no vertex
  data): one torso tube (32 columns, `[body] torso_levels_m`: superellipse sections with the exponent fitted to the ANSUR girths), its bottom ring split into two
  leg loops by a 3-vertex crotch bridge, an arm port cut out of the torso's side from the axilla to the shoulder top (loop down the anterior axillary fold,
  under the axilla, up the posterior fold, over the shoulder: 18 vertices), 3 transition rings per port (deltoid cap pushed to >= `deltoid_radius_m`
  laterally / gluteal fold), five loops at the elbow and the knee; the cage gets one Catmull-Clark step (`figure_body.subdivide`); `girth_factors` corrects
  the cage's girths so the SUBDIVIDED body has the ANSUR girths (three fixed-point passes, cached).
- Build pose -> rest: the hull is modelled in a build pose (arms straight in a 35 deg A-pose, legs straight 6 deg apart; `build_pose`: one rigid transform
  per bone), skinned with smooth weights (two bones across each joint, <= 4 influences) and posed into the seated rest by dual-quaternion skinning (no
  collapse). The exported mesh is that rest mesh.
- Suit = its own hull: the body sections plus the `[suit]` ease (torso 14 mm, upper arm 16, forearm 7, thigh 16, shank 6: loose at the thighs and upper
  arms, tight at the forearm and shank, eva_suit_newatlas / crew1_1); the body itself is not exported (covered). Geometry for the silhouette only: suit hull,
  glove lock rings, gloves (palm, index, middle / ring / little, thumb), boot shafts over the suit's lower legs and the feet, the helmet as before.
- Surface detail by texture (`suit_textures.py`, atlas 2.56 x 1.60 m of suit surface, 1.25 mm texel, `[suit_look]`): dark grey side panels (hip to axilla, up
  the front of the shoulder to the collar), the stripe down the back of the upper arm, the grey forearm panel, the chest plate's dark chevron (stripe grey,
  22 mm), seam piping,
  the front closure, thigh pockets, ribbed knee bellows (shading), grey joint inserts (inner elbow, back of the knee), the flag on the upper left arm.
  Torso UV (fix round 1): u around each ring is its ARC-LENGTH share from the back centre (front = 0.5); the torso columns sit at equal angles and crowd at the
  corners of the near-box sections, so the earlier column-index u squeezed the 75 mm panels into ~15 mm at the chest corners (the cause of the "uniform grey"
  look, not the lighting). Gloves dark grey 0.09 (KSC-20190731, jsc2024e066391, demo1_1). Under the direct sun patch of the pad shots the AgX shoulder still
  compresses white suit and grey parts (the studio photos are evenly lit).
- Dynamic folds: (a) `suit_wrinkle.png` (2.5 mm texel): compression folds per joint zone at full bend (RG = height slopes, B = cavity; pitch 28 mm, depth
  6 mm, strongest on the compressed side); every suit vertex carries its zone (UV2: slot + 1, mask). `game/shaders/crew_suit.gdshader` blends them by the
  joint's wrinkle weight (instance uniforms `wrinkle_l` / `wrinkle_r` = shoulder, elbow, hip, knee), the normal through the surface gradient of the height
  (no tangents needed), the cavity darkens the albedo (`cavity_shade` 0.25). The elbow's upper arm -> forearm weight blend spans +-0.07 m (`ELBOW_BLEND_M`,
  fix round 1: +-0.05 left a hard inner-elbow crease). (b) 16 corrective blend shapes `<joint>_<side>_flex` / `_ext`: the bind-space offsets that make
  the game's linear skinning equal the dual-quaternion pose at full bend / the extension limit, with the joint zone pushed out along its normals until the
  limb keeps its rest volume (cloth bunching instead of a collapse). No cloth simulation.
- `game/cockpit/figure_deform.gd` (`FigureDeform`, one per figure, `CockpitView.figure_deform(seat)`): theta = the angle between the reference direction
  (shoulder / hip: minus the chest / hips bone; elbow / knee: the parent bone) and the child bone, measured between bone heads; flex weight ramps from the
  rest angle to `[joints]` full, ext weight from the rest angle to the extension limit, wrinkle weight 0 at the straight joint and 1 at full bend
  (`figure_layout.bend_weights` mirrors it). Swing gate (fix round 1): the shoulder / hip flex shapes are baked for ONE swing direction (manifest joint
  `plane`, `figure_layout.bake_direction`: shoulder forward + 25 % out, hip forward), so their weight is multiplied by `swing_gate` (1 within 25 deg of
  that direction, 0 beyond 65 deg, smoothstep; `figure_layout.swing_gate` mirrors `FigureDeform.swing_gate`); the plane rides with the reference bone's
  pose. Measured: commander's right shoulder at the HOTAS grip (swung back, 39 deg) gate 0 (was a forward bulge at 0.2 weight = the crumpled shoulder),
  reaching to the centre screen gate 1. Updated after every `pose_arms` (the commander) and once at build (the others are static); only the suit mesh
  node gets a deformer.
- Volumes (part volume from the fixed rest port loop, `figure_body.part_volume`): with the shapes the arm / leg keeps its rest volume within 0.5 % at a
  90 deg elbow and knee, a straight elbow, a 140 deg elbow, a 135 deg knee and a 120 deg shoulder raise; plain LBS changes the arm by ~3-5 % at the 120 deg
  raise. The shoulder shapes are baked for a forward raise (toward the console, 25 % outward); other swing directions are gated off (plain skinning).
- Seated launch pose (all figures): hands on the thighs; the commander's idle hands on the HOTAS grips while the props are shown (`CockpitView.idle_tip`:
  `[hotas] *_grip_m` + 0.09 m along the chest normal to the index tip, `GRIP_TIP_AHEAD_M`; the fingers close by `ArmIk.FIST_CURL_DEG` x grip; Task 9
  calibrates the grips). Pointing hand (fix round 1): while the index is extended (reach / tap) the middle / ring / little fingers fold in by
  `ArmIk.POINT_CURL_DEG` 45 per joint (cap `MAX_FINGER_CURL_DEG` 65). The commander's IK elbow pole is `ELBOW_POLE` (down the seat back, a little back and out).
- Budget: 28.1k triangles per figure (suit 14.2k, helmet / visor / beads ~9k, gloves, boots), ~112k for four figures. Morph targets (fix round 1) only on
  the suit primitive (7966 vertices x 16 targets, POSITION + NORMAL); the rigid parts (15.3k vertices) are a separate mesh without targets: `crew_figure.bin`
  4.5 MB (was 10.5 MB with the targets on every primitive), morphed vertex-target pairs 127k per figure (was 373k, -66 %), 0.51M for four. VR check in Task 9.
- Tests: `tools/structures/tests/test_crew_figure.py` (ANSUR rows = the CSV medians; body measures within 1 cm: buttock / waist / chest girths, breadths
  and depths, neck base, thigh, lower thigh, calf, ankle, wrist girths, segment lengths, biacromial and bideltoid breadths; eye contract; suit = body + ease;
  closed hull with the four ports; weights; joint volumes; wrinkle weights; swing gate; torso arc-length UV; no NaN; maps; glTF shapes and mask UV; morph
  targets only on the suit, .bin < 6 MB), `test_dragon_cabin.py` (knee / shin in the photo bands with margin, knees apart, foot plate), `console_pointer_test.gd`
  (`_check_figure_deform`), `cockpit_view_test.gd` (35 bones). Shots: `cockpit_shots.gd --body` (`screenshots/m7-cockpit/body/`: `body_side`, `body_34`,
  `body_arm_bent`, `body_arm_raised`, `body_elbow_bent`, `body_elbow_straight`, `*_vs_<photo>`).
- Shoulder hole (user VR run 2026-11-11, Task 9 VR fix B): the arm port's transition rings (port loop on the torso -> first arm ring 0.10 m below the acromion, three
  rings chest -> upper arm) form a thin band almost in the pivot's lateral plane; a forward or backward swing turns its thin triangles over (inverted triangles, measured
  with the blended skinning matrices: 0 at rest, 19 / 35 at a 30 / 45 deg backward swing, 49-90 at a 60-120 deg forward raise, in DQS as well as LBS; also at the HOTAS idle
  pose). With back-face culling the inverted triangles were a see-through hole into the hollow hull. `crew_suit.gdshader` is now `cull_disabled` (Godot flips NORMAL on back
  faces), so a fold reads as a crease, no hole (`screenshots/m7-cockpit-vrfix/after/ba_idle_top_r.png`). Remapping the transition weights (linear, power curves,
  spreading arm weight into the torso) does not remove the inversions (measured, same or more); the fold itself (ruling O crumple) stays until the port transition is
  redesigned (longer transition / pivot nearer the glenohumeral centre). Diagnosis shots: `cockpit_shots.gd --shoulder` (six arm poses x eye / out / top x side).
- Open: a backward / sideways shoulder swing gets plain skinning (no corrective baked for it; a soft front-shoulder fold remains at the HOTAS grip); the reach
  to the centre screen needs a near-straight arm (round-4 fit, 0.69 of 0.80 m); the knee bellows are shading only; no face inside the helmet; the gloves'
  fingers are separate rigid capsules per phalanx; the non-player figures' hands rest on the thighs (no armrest grips modelled).

### Windows and hatch against the exterior table
- `[cabin] windows` / `hatch` are copies [D] of `config/vehicles/detail/crew_dragon.toml` (outer-surface convention);
  the cabin generator carries each aperture through the wall along the wall normal (x shifts -0.06 m on the inner face).
- Az +-33.3 windows: exterior photo `D_CCP_SpaceX_Demo_2_Dragon__3_` shows the oval glass beside the hatch, about
  0.30-0.37 x 0.48 m depending on the assumed camera distance (table 0.31 x 0.51); inside (`dragon_cockpit_crew11`,
  `11b`) a round reveal ahead of and inboard of an outer seat, below the console: same place, size not measurable.
- Az +-90 windows: `crew1_1` (camera in the hatch, helmet-scaled) puts them at az 84-90 (+-10 deg), centre at the outer
  crews' eye height (x ~2.37 vs 2.385); the dark interior opening measures about 0.46 (x) x 0.50 (tangential) m
  (+-25 %). Larger than the 0.31 m wide glass: read as a flared round reveal (`window_reveal_radius_m` 0.24), not as a
  conflict with the exterior pane.
- Hatch: interior face seen only in part under the console (discovery views), no scale; consistent with az 0, x
  1.70-2.99.
- Gate (ruling E): no measured interior number contradicts the exterior table beyond its uncertainty, so no proposal and
  the exterior file is unchanged. The photos resolve positions to about +-10 deg / +-0.1 m and sizes to +-20-25 %; the
  10 % gate (2.5 cm, 0.55 deg on the windows) is finer than the evidence: "not contradicted", not "confirmed to 10 %".

## Code
- `game/cockpit/cockpit_config.gd` (`CockpitConfig`): `for_vehicle(id)` (cached; `+` stack ids use the first part whose
  `config/vehicles/<part>.toml` `[vehicle] cockpit = "<id>"` names a cabin; the core ignores that key), properties
  `id`, `capsule_stage_id` (`[cockpit] capsule_stage`), `eye` (Transform3D, -Z = gaze), `seats`, `screens`, `buttons`,
  `cameras` (empty until the camera task), `hotas`, `hotas_props_enabled`, plus `cabin`, `console`, `tablets`, `lights`,
  `validate()` and `validate_crew(count)`. `MissionCatalogue.crew_count(mission_id)` reads `[crew] count` (4 in the five
  Crew Dragon missions; missing = the cockpit's seat count, else 0). `TomlLite` reads one-line arrays of arrays.
- Cabin model: `tools/structures/blender/gen_dragon_cabin.py` (+ pure helpers `cabin_layout.py`: apertures, node names, manifest block) writes
  `dragon_cabin.gltf` into the Crew Dragon model set (last generator; 61.6k triangles, `cabin_shell` 52.6k of them, rows 2 cm / columns 0.75 deg around
  the apertures so the cut edge hides under the bezel). `vehicle_spec.load_dragon` loads and validates `config/cockpits/<[vehicle] cockpit>.toml` into
  `spec["cockpit"]` (hashed into the manifest; `VehicleModels` staleness includes the cockpit TOML). Capsule stage frame.
  Nodes: `cabin_shell`, `floor`, `hatch_frame`, `window_<i>` / `window_glass_<i>` (i = exterior table row, +az then -az: 0 = +33.3, 1 = -33.3,
  2 = +90, 3 = -90), `seat_s1..s4`, `console_housing` (the filleted body's front and ends + the dark upper panel, `console_shape.rounded_box`; `console_back` = its back face + the two ceiling posts, `console_under` = its underside and top; `console_shelf` = the continuous sloped tray with its lip, `console_bezel` the screens' dark glass borders, `guard_<id>` / `guard_rim_<id>` the clear key guards and their red rims, `console_hit` the hidden flat key hit faces (fidelity Task 5); tested: every button stands >= 5 mm proud of the housing, the three screens are coplanar within 1 mm / 0.1 deg), `screen_left|centre|right`
  (one quad each, exactly the TOML centre / size, UV 0..1 with (0, 0) top left, +normal toward the crew), `tablet_l|r` (`[[tablet]]` ids l / r),
  `button_<id>` (one node per button; the centre handle is the node `handle`, no `button_handle`), `light_<i>` (`[[light]]` rows in file order),
  `hotas_stick`, `hotas_throttle`. Manifest `cockpit`: `nodes`, `screens[id] = {centre, normal, up, size}`, `buttons[id] = {centre, size}` (all 39 incl.
  `handle`), `button_ids`, `tablets`. No materials in the manifest yet (the game side assigns them).
- Wall: the inner surface is the exterior meridian offset by `wall_m` along its outward normal; windows / hatch are cut in that chart (`cabin_layout.at_depth`).
  Window well = funnel from the round interior reveal (`window_reveal_radius_m`) to the exterior ellipse 2 cm under the outer skin (glass there), 6 mm bezel
  ring (`window_bezel_m`). Hatch: super-ellipse opening (exponent 4 as the exterior), door inner face recessed 3 cm, white trim `hatch_trim_m`.
- Simplifications (silhouette only, texture later): the handle is a raised pad with a T grip (not a recess); buttons are boxes / cylinders 6 mm proud (cover
  12 mm) on the tray (fidelity Task 5); seats are
  boxes (back, headrest, pan, lower-leg support, armrests) at the tabled hips, pan along the chest normal, shin support not measured; the HOTAS props
  are a base, shaft and grip at the grip offsets (calibrated in Task 9). `test_dragon_cabin.py` compares the windows with the exterior table (glass
  centre carried back along the normal: 2.5 cm / 0.55 deg, the Task 1 gate).

### Cabin in the game (Task 5)
- `game/cockpit/cockpit_view.gd` (`CockpitView`): child of the capsule stage node (`VehicleView._build_cockpit`, rebuilt with the stage nodes, `VehicleView.cockpit`;
  exists in every view). Cabin meshes by node name from `VehicleModels.load_for("crew_dragon")["cabin"]`; `eye_node` at `CockpitConfig.eye`; `screen_surface(id)`,
  `button_node(id)`, `set_crew(n)` (seats s2, s3, s1, s4; s2 = player body), `set_hotas_props(b)`, `set_player_head_hidden(b)` (collapses the head bone on request, shots), `figure_skeleton(seat)` (Task 8 poses the arm bones). Figures load through `VehicleModels.scene_for` (keeps Skeleton3D / skin; `_meshes_of` does not).
  Default crew = seat count; `World._update_cabin_occupancy` sets `MissionCatalogue.crew_count(FlightController.active_mission_id)` (free launches keep the seat count).
- Exterior hull parts of the capsule stage cast shadows double-sided (sun enters only through the windows).
- Look: `shaders/cabin.gdshader` (lit_aerial look; closed cabin: `ambient_light_disabled`, no sky ambient or sky radiance, glossy parts reflect the walls
  (round 5, below); procedural panel seams on the shell),
  `cabin_glass.gdshader` (windows), screens through `display_colour.gdshader` (dark Dragon blue placeholder, display-referred). Lamps: `OmniLight3D` at the `[[light]]` rows
  plus two fill lights standing in for bounced light ([call], flux constants in the class), energy = `TowerLights.energy_of(cd, scene_per_cd_m2)` every frame. Seats: white shell + black pads (fix round 2);
  housing near black; walls white matte.
- Lighting (fix round 1): the cabin has no GI, so each cabin surface gets a flat bounced-light term in the shader (`fill` = `FILL_LUX` 450 lx / pi x
  `scene_per_cd_m2`, emission albedo x fill, set every frame; scene-referred like the lamps) on top of the lamps and the sky-AO remainder; the two ceiling lamps cast
  shadows; a ceiling ring light (8 omnis on r 0.72 m at `ceiling_x_m` - 0.10, 6000 lm, [E] crew1_1 annulus r 0.45..0.9 m) stands in for the annulus (its emissive mesh
  comes with the generator). The scene-referred eye adaptation meters the cabin to 18 % grey whatever the light level, so flux alone cannot make it white: `Atmosphere.field_key`
  (default `EyeAdaptation.FIELD_MID_GREY` 0.18) is the field's grey; the shots set 1.2 ([call], `--field_key`) and measure white wall p90 0.82-0.87 sRGB
  (pilot_eye 0.87, gopro 0.82, hatch 0.86; at key 0.18 they were 0.56-0.62, at 0.40: 0.67-0.74). Whoever puts the player camera in the cabin (Task 8/9) must set `field_key`.
  Open: a sound way to set the key (cabin-local high-key rule) is a decision for the eye-adaptation subsystem.
- Shots: `game/tests/cockpit_shots.gd` (GPU, 2560 x 1440 window) writes `pilot_eye` (our own view, no photo pair), `gopro_discovery2` (the photographers' GoPro, rendered 140 deg
  rectilinear and remapped to the equidistant fisheye, 1600 x 900), `hatch_crew1_1` (3:2) and side-by-side / overlay PNGs (`gopro_vs_discovery2`, `gopro_overlay_discovery2`,
  `gopro_vs_discovery1`, `hatch_vs_crew1_1`) with the photos to `screenshots/m7-cockpit/look/`; each shot prints its p90 / mean grey. Eye adaptation on (`--fixed_exposure` off).
  Test: `cockpit_view_test.gd` (needs models; in `$needsGpuOrModels`).

### Fix round 2 (geometry and material fidelity)
- Console: (screens, shelf and keys: see "Screens and tray (fidelity Task 5)"), housing 0.1575 above the screens plus a
  dark upper panel (`upper_top_x_m` 3.30, `upper_depth_m` 0.04) up to the ceiling, `[[locker]]` 5 / 6 (0.15 x 0.165, outboard of the tablets), the lamp panels moved to the
  upper panel's top corners, light grey `console_back` (back, underside, two posts). Nodes added: `console_back`, `console_shelf`, `console_lockers`, `ceiling_ring_light` (emissive annulus r 0.50-0.85,
  `[cabin] ring_light_radii_m`; the game's ring omnis sit on its mid radius). Buttons of the left / right panels moved 0.025 m toward the centre with their screens.
- Seats: `seat_shell_<id>` (white), `seat_pad_<id>` and `seat_rest_<id>` (black pads / carbon rails + foot frame). `[[seat]]` carries `foot_rest_m` / `foot_rest_normal` ([D], from the figure bones; tested under the ankles).
- Shots: `helmet_closeup` (pilot, other figures / seats / console hidden) and `helmet_vs_detail2` in `screenshots/m7-cockpit/look/`.
- Open: boots read grey (exposure of the high-key cabin), screens are still empty blue, visor shows the lamp blobs, the real visor opening above a dark face aperture is not modelled (closed by decision).

### Shot cameras (round 4)
- GoPro (discovery2 = discovery1 framing): the fix-round-1 least-squares fit (equidistant fisheye on the 12 screen corners, rms 20 px of 4128, f = 2085 px/rad,
  113 x 64 deg) is relative to the screens: 0.61 m along their normal, 0.23 m along their up from the centre screen centre, between the heads. It moves rigidly
  with the console block: position (2.2437, -0.6322, -0.0251), forward (0.3265, 0.9453, 0.0073), right (-0.0241, 0.0160, -0.9996).
- Hatch (crew1_1): joint fit (below): (1.839, 1.454, 0.013), pitched 8.2 deg up, f 536 px on 1200 x 800 = 96.4 x 73.4 deg (3:2). The camera sits 0.15 m inside
  the inner wall (the photographer leans in through the hatch; the near-field boots of the outer crew confirm a close camera).
- pilot_eye: the config eye, turned toward the screen row (aim 0.15 m to the commander's side of the centre screen centre), 105 deg horizontal, field key 0.5
  (`--field_key` per shot; the dark console dominates the metering). Task 5b: `helmet_profile` (pure side view from the pilot's right, 1.1 m, 40 deg) with `helmet_side_vs_spacex_anzug`, and `helmet_side_vs_jsc066389` in the composed set. Debug options of `cockpit_shots.gd`: `--only=<shots>`, `--no_lamp_shadows`, `--hide=<nodes>`.

### Fix round 3 (legs)
- Legs (crew1_1 fit of the review): `crew_figure.toml` `thigh_raise_deg` -25 (thigh 10 deg above the hatch axis; the seat pan is built along the thigh, `build_seat(pan_raise_deg)`), `knee_angle_deg` 105
  (shin 65 deg below the hatch axis). `[[seat]]` `foot_rest_m` s2/s3 (1.615, 0.733, +-0.35), s1/s4 (1.535, 0.733, +-1.05), `foot_rest_normal` (0.9063, -0.4226, 0) = up the shin.
  The foot rests move with the hips (round 4: hip + (-0.185, 0.753)).
  The foot rest is now a flat carbon tray (top face in the plane through `foot_rest_m`, heel stop and toe lip) with the two rails, not an upright frame; seen nearly edge-on from the hatch as in crew1_1.
- Materials: shelf modules albedo 0.07 (was 0.32), lockers 0.02, tablets 0.03, speculars 0.03-0.05; `fill_ceiling` omni follows `ceiling_x_m` - 0.25; the two console lamps got shadow bias (shadow_bias 0.12, normal bias 3); ring light lifted 15 mm (was 4 mm);
  seam lines fade at grazing view. `cockpit_shots.gd` debug options `--hide=<node,..>` and `--hide_exterior`.

### Fix round 4 (one joint fit of the cabin layout)
Rounds 1-3 corrected the layout piecewise per photo; round 3 moved console and ceiling 0.35 m down and left the eye 0.44 m from the centre screen. Round 4 fits
seats, eye, console, ceiling and both shot cameras together (scratch scripts `fit4.py` / `round4_toml.py` of the session; points read on 25-50 px grids).
- crew1_1 (1200 x 800), 12 points + 4 helmet widths, free: camera (X, Y, Z, yaw, pitch, f), inner / outer helmet (X, Y), console back lip (X, Y), console tilt,
  console width, ceiling X. Model: windows (2.32, 0, +-1.43); helmets at Z +-0.35 / +-1.05, width 0.28 m; back face from the lip 0.50 m up the console to the post
  bases (Z +-0.55); tunnel far rim (ceiling, -0.45, 0); dish far edge at the wall radius. Result: camera (1.839, 1.454, 0.013), 8.2 deg up, 96.4 deg; lip
  (2.450, 0.229), console up 44.7 deg from +X, width 1.23 (prior 1.25 +-0.08; the table keeps 1.32 of discovery2); ceiling 3.48; inner helmets (2.553, -0.655),
  outer (2.440, -0.216). Residuals: windows 3-4 px, helmets 0-8 px, lip 5-6 px, posts 2-5 px, rim / dish 1-3 px; helmet widths 69 / 87 px against 65 / 90-100.
  Findings: (1) round 3 read the tunnel's far rim as the near rim: the ceiling is at 3.48, not 3.00; (2) the inner seats sit 0.44 m further from the hatch than
  the outer ones (head spacing 175 px for 0.70 m against 645 px for 2.10 m, helmet widths 65 against 90-100 px).
- Reach (crew2_1, inspiration4_1, demo1_3: near-straight arms to the screens) and discovery2 (the GoPro between the heads at 0.61 / 0.23 m along the screens'
  normal / up) fix the commander's eye: centre screen centre + 0.55 normal + 0.23 up -> (2.2857, -0.5945, 0.35); eye to the centre screen centre 0.69 m, right
  shoulder to it 0.69 m and left shoulder to the left screen 0.70 m of the 0.778 m reach (88 %, near-straight arm). The right screen needs a lean (0.85 m).
- Discrepancy with crew1_1, explained: the photo's distances agree (its inner eye is 0.67-0.71 m from the centre screen), but its heads sit 0.33 m higher on the
  axis at the same Y (eye 0.46 m along the screens' up, 53 deg off their normal) and its foot rests 0.26 m higher (x 1.50-1.60 against 1.29): the whole seat sits
  ~0.3 m higher in crew1_1. Read as the hangar / ingress seat position (the crew sit up toward the camera); the game uses the launch posture under the
  reach-fitted eye. The hatch comparison therefore shows our heads ~85 px lower than the photo while windows, console and ceiling line up.
- The player figure's own eye is now the config eye (hip = eye - figure eye point (0.812, -0.288) at back 35 deg); `cockpit_view_test` checks it within 2 cm.
- Console block: rotated 10 deg about Z and moved with the centre screen centre; posts start at the housing's back top edge; the dark upper panel ends 0.47 m up the
  console (x 2.845; it covers the lamp panels; crew1_1 shows the dish over the console). Nodes `console_back` (silver back face + posts, brushed albedo 0.30,
  metallic 0.25, the dark vent band by shader: `cabin.gdshader` band, up -0.11..+0.07 from the centre screen centre, +-0.58 m), `console_under` (dark grey underside
  and top), `console_lamps` (emissive strips along the underside's back lip, crew1_1). The white slab of round 3 is gone.
- Console lamps: the console parts (console_*, tablets, buttons, handle, lamp panels) are on render layer `CONSOLE_LAYER` (1 << 11) and the console lamps'
  `light_cull_mask` excludes it (the lamps hang 8 cm in front of the console face and face the crew; ring and fill lights still light the console). The figures
  keep the lamp shadows. (Round 4 took this for the moire's cause; round 5 found the real one, below.)
- Helmet: superseded by Task 5b (below; round-4 shape in the history file). The closed visor stays two-tone by shader (`visor_tint_split` 0.55: copper top,
  dark clear glass below, manifest `crew_figure.visor_tint`), closed and copper (user decision).
- Tests: `test_layout_is_one_consistent_fit` (eye to centre screen 0.60-0.75, shoulders 0.62-0.75 within the reach, gaze < 30 deg off the screen normal, figure eye =
  config eye, upper panel under the ceiling); foot trays move with the hip and stay above the floor (the crew1_1 x band no longer applies: launch posture); helmet test
  on the round-4 ranges; cockpit_config_test eye forward; cockpit_view_test figure eye at the config eye.

### Fix round 5 (moire, black parts, helmet pivot)
- Moire (concentric wood-grain rings on the dark upper panel, fine striping on the dark panels of pilot_eye and behind the helmets): the aerial perspective.
  A/B on pilot_eye (`screenshots/m7-cockpit/moire/moire_ab_pilot_eye.png`, contrast-stretched): unchanged with every shadow off (`--no_shadows`), with the ring
  and fill omnis off, with the Sun off; gone with `atm_fog` skipped in `cabin.gdshader`. `atm_to_point_model` forms S(camera) - T S(point) from table reads at
  r ~ 6.4e6 m; over a path of metres the two reads differ by less than their float32 / filter resolution and the difference is rounding noise that changes
  with distance. Fix (shared include, `atmosphere-scattering.md`): `atm_to_point_offset` evaluates paths shorter than `ATM_SHORT_PATH_M` (200 m) once at 200 m
  along the same ray and scales (in-scatter x d / D, transmittance T^(d / D)); the fixed shot matches the fog-off shot. The round-3 seam fade at grazing view stays.
- Black parts: the cabin materials took the sky's ambient light and radiance, scaled by AO 0.03, but the sky is far brighter than the dim cabin the eye
  adaptation meters, so every reflection carried it (black seats, console, brackets and the lower visor mid / bluish grey; A/B `--no_sky_light`,
  `moire/black_ab_hatch.png`). Now `cabin.gdshader` is `ambient_light_disabled` and adds the walls' reflection as emission: a uniform surround of luminance
  `fill_surround` = fill x wall albedo (`CockpitView.WALL_ALBEDO`, set each frame) weighted with the split-sum environment BRDF (Karis 2014 fit);
  `ambient_visibility` is gone. The console albedos are linear photo measures and are now converted for the sRGB uniform (they went in as sRGB:
  0.07 became 0.006 linear, the face read pure black once the sky lift was gone).
- The round-5 "boots read grey" point is closed under the unified appearance model (Task 5b re-check below).
- Helmet: the visor pivot is a round grey button (`pivot_button_radius_m` 9.5 mm, `pivot_button_height_m` 4 mm, collar grey) at jaw height
  (`pivot_el_deg` -30) 9 deg (Task 5b; round 5: 13) behind the opening edge (jsc2024e066389: 19 mm disc ~0.09 m below eye level, ~25 mm behind the seal); `helmet_side_vs_jsc066389`.
- Debug options of `cockpit_shots.gd`: `--no_shadows`, `--lights_off=<patterns|sun>`, `--no_sky_light`, `--agx_contrast=<c>`.

### Photo fit toolkit (fidelity plan Task 3)
- `tools/structures/cabin_fit/` replaces the session scratch fits (`fit4.py`): `camera.py` (pinhole / equidistant fisheye with optional k, `Camera.from_row` / `to_row` of `config/cockpits/photo_cameras.toml`),
  `fit.py` (`fit(photos, PartModel, anchors)`: part parameters shared, per photo pose 6 + ln f (+ k1), soft-L1 on image rows only (anchors and priors stay quadratic), sigma from a rank-aware SVD of (J^T J)^-1 scaled by max(1, chi2/dof) for data-constrained parameters only (data dof); `FitResult.constrained_by_data[name]` False = only the prior fixes it (tag [E]): sigma == prior_sigma exactly, inf without a prior or for an unobserved camera),
  `annotations.py` (`annotations/<photo_id>.toml`: `[[point]] label, px`, `[[edge]] label, px = [[u,v],...]`, `sigma_px`), `anchors.py` (known lengths: IDSS passage 0.800 m [S], iPad mini 0.2032 x 0.1348 m [S], window axial extent 0.51 m [E], ANSUR II hand / head medians [S]).
  Tests: `python -m pytest tools/structures/tests -q -k cabin_fit`.

### Console (fidelity Task 4: suspension and body)
- Fit: `tools/structures/cabin_fit/parts/console.py` (`Console` PartModel; `python -m cabin_fit.parts.console [--as-built]` from `tools/structures`), annotations
  `cabin_fit/annotations/crew1_1.toml` (4 post points, 2 back-face ends, 4 edges: both rod axes, back top edge, silver lip edge; sigma 2 px), `discovery2.toml`
  (10 screen glass corners, the front bottom edge under the modules; sigma 6 px) and `discovery10.toml` (10 corners incl. the right screen's bottom right, the
  front bottom edge; another frame of discovery2's fixed GoPro mount: corners within 2-12 px, edge profiles within 2-4 px, so not an independent view).
  Cameras with the photo_cameras.toml rows as prior (0.05 m, 0.02 / 0.035 rad).
  Free fit (block free): cx 2.537 +-0.029, cy -0.032 +-0.029, tilt 46.3 +-1.2 deg, hood 0.181 +-0.030, u_bot -0.2779 +-0.0022, back drop 0.091 +-0.068, post Z
  0.547 +-0.022, top angle 50.2 +-1.8 deg [data-constrained]; width 1.344 +-0.046, draft -2 +-10 deg, u_top 0.270 +-0.046, post base depth 0.175, top radius
  0.966 +-0.027 [not data-constrained]. As built (block, width 1.40, u_top 0.265, draft 0 and post top radius 0.945 held): hood 0.221 +-0.011, u_bot -0.2774
  +-0.0021, back drop 0.059 +-0.036, post Z 0.555 +-0.008, post base depth 0.216 +-0.011 (built 0.199), top angle 52.2 +-0.7 deg; screen pitch 0.4547 +-0.0027,
  glass half width 0.2025 +-0.0013 (nuisance parameters). The three fitted cameras are written to `photo_cameras.toml` [D] (Ruling A).
- Fit-model residuals (the sharp-edged model on its own points, not the render): crew1_1 1.25 px rms of 1200 (post points <= 1.2 px, rod axes 0.2-0.5 px rms,
  back top 0.7, lip 0.5, back_end points 3.7-4.3 px in v), discovery2 5.9 px rms of 4128 (corners <= 9 px, front bottom edge 11 px rms), discovery10 4.4 px
  rms (corners <= 4.4 px, front bottom edge 8.6 px rms).
- Render vs photo per feature (fid_* at the written cameras, render scaled to photo px; fix round 1):

  | Feature | crew1_1 (px of 1200) | discovery2 / 10 (px of 4128) |
  |---|---|---|
  | Post rod axes (v 120-140) | 0-1 px (half-depth centres), widths 9 vs 8-10 px; sleeve at v 145: 1.5-2.5 px | not visible |
  | Back top edge | render 172.2-173.1 vs photo 171 (annotated; column steps 171-175): +1..+2 px | not visible |
  | Silver lip | no material step in the render (back face and fillet one grey): the built flat back face ends at v 221 (5.5 px above the photo lip 226.5), the 4 cm fillet runs down to v 244; the fit's sharp lip lies on it within 0.5 px | not visible |
  | Lamp strips | v 226-231 vs 228-232 (<= 2 px); u 400-445 / 460-545 / 640-725 / 745-780 vs 406-434 / 466-546 / 642-734 / 742-786 (<= 6 px) | not visible |
  | Body ends (outline) | v 176: -47 / -43 (render inside); v 190: -18 / -16; v 226: +30 / +31; v 240: +45 / +42; v 260: +49 / +48 (render outside; left / right, render 374-303 / 819-896 vs photo 327-352 / 862-838) | hidden by helmets |
  | Underside | its lower edge is hidden by the helmets from v ~300 (render and photo both dark down to there) | back-facing from the crew side |
  | Front bottom edge | not visible | render silhouette 28-49 px above the photo edge (u 700-3000) |
  | Screens | not visible | the table row (pitch 0.402, half width 0.198) puts the outer screens' corners 87-143 px off, the centre screen 9-21 px |

  So the posts, the back top edge and the lamps lie on the photo; the body outline does not. Open, carried to Task 7 (recess): the crew1_1 outline narrowing
  downward is the contour of the white wall's trapezoid recess around the console; the recess panels must hide the render's lower body ends (+30..+49 px). Near
  the top (v 176-190) the render is 16-47 px narrower than that contour, which an occluder cannot fix: the recess side walls must fill it, or the body is wider
  at the top. Task 7's ledger check: re-run the crew1_1 overlay with the recess, confirm the ends are occluded, else refit the width and the end shape.
  Open (edge radius, [E]): the photos' back lip and front bottom edge look sharper than the 4 cm fillet (lip: face end 5.5 px above, fillet to 18 px below;
  front bottom: the fillet's silhouette 1-2 cm higher = 28-49 px); a per-edge radius (bottom edges <= 1 cm) would close both. Open (screens task): pitch
  0.455 / half width 0.203 (discovery2 and discovery10 agree).
- Underside photos, checked one by one (fix round 1); none constrains `underside_profile`, the width or the post foot:
  - jsc2026e404725 (16 mm, f 1707 px known from EXIF): the console is 0.3-0.6 m from the lens and out of focus (edges 30-60 px wide); two screens are
    visible, the near one with only its bottom right corner, the far one nearly edge-on (bl, tl, tr): 3-5 corners, which could place a camera, but no console
    end, no post and no back edge is in the frame; the underside is seen only edge-on below the lower tray.
  - jsc2025e064539 (24 mm, cropped: f unknown): same view, Crew-11 trainer state with a protruding grey module tray and a grab bar that the flight console
    (crew1_1, discovery2/10) does not show; the underside's back edge is behind the tray, dark on dark.
  - jsc2022e068643 / 068646 (no EXIF): the same pilot-side view up past the screens, defocused near screen, the far screen edge-on; the underside again only
    beyond the tray. 068660 (50 mm): the console is a blurred band at the frame's top edge, no edge to read.
  - nasatour_0242-0330 (video frames, unknown wide lens with distortion, motion blur): the module tray from below and the screens; no cabin-fixed point
    for the lens, nothing behind the tray.
  - Geometry: the underside's normal points toward -X +Y (the hatch side); from the crew side (eye about (2.45, -0.35)) it faces away, so only the hatch
    view (crew1_1) sees it, and there its lower edge is behind the helmets. The underside shape stays [E]; a hatch-side photo below the helmets is needed.
- Findings: the posts are short (0.83 m) and lean from the body's back top edge up and back toward the crew's heads to the ceiling ring's rim (collars at the
  dish's lower edge in crew1_1), not vertical struts; a funnel foot (outboard flare in the photo, built round) and a sleeve sit on the body, a collar (joint
  head) at the ceiling.
- Generator: `blender/console_shape.py` (pure: `rounded_box(w, h, d, r, segments)` cube-sphere fillets, closed; `drop_underside`, `draft_sides`; `post(base, top, r,
  head_r, head_len, sides, foot, top_plane_normal)` lathe, head cut flush on the ceiling plane), `cabin_layout.console_frame` (axes from `tilt_deg`, checked against
  the centre screen +-0.5 deg) / `console_posts`. Body faces are split by direction into `console_housing` / `console_back` / `console_under` (same materials as
  before); the lamp strips lie on the back-bottom fillet 45 deg round from the underside (`UNDER_LAMP_FILLET_DEG`, on the fillet's chord, 1 mm sunk).
  FILLET_SEGMENTS 4, POST_SIDES 20; cabin 63,750 triangles. Tests: `test_console_shape.py` (closed, extents, volume, every vertex r from the inner box, post
  radii / flush head), `test_dragon_cabin.py` (tilt = screens +-0.5 deg, console_frame follows tilt_deg and refuses a mismatch, the built front face is one
  plane square to it, posts end on the ceiling ring within 5 mm, the collars' outer radius inside the ring (config and mesh: max 0.9926 of 0.997), generated
  posts reach the ceiling).
- Not measurable from the photos ([E], listed for the user): the body's rear mount and a straight-below view (no photo), the front width (helmets / visors hide
  the ends in discovery2/10), the fillet radius and the side draft, the post top's radius along crew1_1's ray, the collar length (seen end-on), the funnel /
  sleeve lengths (foreshortened), the underside shape (above). The Hawthorne trainer was not used for the suspension.
- Shots: `cockpit_shots.gd --console_closeup` (stereo pairs `closeup_eye_*`, `closeup_back_*` + `_stereo` sheets, IPD 64 mm); `--fidelity` now also renders
  `fid_discovery10`.
- GPU (stereo pimax, cockpit view, 120 frames): median 23.36 ms, p99 29.9 ms (fix round 1; Task 4: 24.08 / 23.58; baseline 24.43 / 23.41): no change.

### Screens and tray (fidelity Task 5)
- Measurement: `cabin_fit/parts/console.py --as-built` now also frees `screen_half_h_m` and fits the two iPad mini bodies (6 corners in
  `annotations/discovery10.toml`; body 0.1348 x 0.2032 [S] as prior = scale anchor; pose lat / u / o / roll free). Result (sigma, data-constrained
  unless noted): glass 0.400 +- 0.013 x 0.227 +- 0.008 m, pitch 0.449 +- 0.015 m, tablets l (-0.321, 0.227) / r (0.334, 0.230) +- 0.005-0.010 (o prior-dominated [E]).
  Residuals 1.18 / 6.25 / 6.23 px rms (crew1_1 / discovery2 / discovery10), chi2 77 / dof 52. The three camera rows of `photo_cameras.toml` were rewritten from
  this fit, each component tagged by `constrained_by_data` ([E] for crew1_1 x / pitch / f and discovery2 f).
- Keys: `cabin_fit/parts/keys.py` back-projects 38 key centres read in discovery10 through its fitted camera onto the key plane (o 0.030): every key lies
  1.5-6 cm further out than the old px-fraction reading (cluster A -0.592..-0.473, the pilot's module +0.473..+0.592: mirror-symmetric within 0.4 mm), rows
  at u -0.186 / -0.214 (row means; were -0.171 / -0.201). Sigma per key 3-7 mm (read 8 px + o +-5 mm) plus the camera's ~1 cm common mode.
- Screens: active 0.392 x 0.219 (diagonal 0.449 +- 0.014 m, 17.7 in, 16:9) = glass - 2 `bezel_m` 0.004 [E]; `corner_radius_m` 0.006 [E], `recess_m` 0.005 [E];
  `screen_gap_m` 0.057 [D]. Built: `console_bezel` (dark glossy ring between the square active quad and the rounded glass outline, at the screen plane), and in
  `console_housing` a face frame `recess_m` in front of the glass (`console_shape.plate_with_holes`, rounded holes, walls down to the glass and back to the body).
  Node ids and the screen quads (UV, centre, size) unchanged.
- Tray (`[tray]`, replaces the separate shelf modules; old rule in `docs/history/cockpit.md`): one prism (`cabin_layout.tray_profiles`, `console_shape.extrude` +
  `round_corners`) along +-`half_length_m` 0.625 in `console_shelf`: deck from the housing at u -0.149 (`shelf_top_gap_m` 0.0395 [D]) down `deck_drop_m` 0.019 [D] to
  the key face, which leans out toward its bottom by `slope_deg` 15 [E] (normal turned up toward the crew's eyes; `depth_m` 0.024 [D] at mid face), the lip
  (`lip_width_m` 0.010 [D], `lip_height_m` 0.006 [E]) down to u -0.237 (`shelf_height_m` 0.088 [D]); fillets `radius_m` 0.004 on the front edges only. The red strip
  mark of `console_shelf` sits on the lip.
- Keys stand `KEY_HEIGHT_M` 8 mm (cover 12 mm) proud of the tray face along its normal (`centre_m` = face centre, re-seated from lat / row whenever the tray
  changes); the four end keys (CANCEL / EXECUTE, both outer modules) are square 0.026 rect keys, the 2 x 3 blocks 0.026 x 0.024 (jsc2022e068617). Labels added on
  the pilot's module from jsc2022e068617 (trainer): CANCEL, EMERGENCY DEORBIT, BREAKOUT, EXECUTE, SUPPRESS FIRE, FIRE RESPONSE (r_upper_2 / r_lower_1 stay blank).
- Guards (`[[guard]]` l / r; `[[button]] guard`): clear cover (`guard_<id>`, filleted box 4 mm over the key faces, `GUARD_MARGIN_M` 4 mm past the keys) and red
  rim + hinge barrel on `hinge_edge` (`guard_rim_<id>`, colour `rim_colour` via the manifest `guards` block). Game: transparent StandardMaterial3D (alpha 0.04).
- Pointer hit geometry: `console_hit` (one flat quad per key face, `cabin_layout.hit_faces`; the handle square to its panel) is hidden in the game; the pointer
  itself still works on `ConsoleStrip` (config rectangles), so every key keeps a hit face at its visible face.
- Render vs photo (as built, written cameras): screen glass corners discovery2 6.2 / 6.5 / 3.1 px rms (left / centre / right; Task 4: 87-143 px at the outer
  screens), discovery10 4.5 / 3.4 / 4.3; tablets 17 / 12 px (roll not built); keys 12 px rms, max 31 (row means, slope); tray deck start -0.7 px, lip bottom +1.2 px
  (column u 1450). Front bottom edge of the body unchanged (28-49 px, the 4 cm fillet; per-edge radius still open).
- Reach: the outer keys moved out; the commander's right hand stays short of the pilot's module at the lean limit (r_upper_3 31.8 mm, r_lower_3 45.7 mm,
  r_upper_5 61.5 mm: `console_pointer_test.gd` KNOWN_REACH_GAP_M; the pointer reaches them all). Task 9 re-measures.
- Cabin 64,368 triangles; GPU (stereo pimax, cockpit view, 120 frames) median 24.07 / 23.70 ms (Task 4: 23.36): no change.
- Shots: `--console_closeup` adds `closeup_tray` / `closeup_tray_r` (the outer modules from 0.6 m on the line to the eye, stereo, IPD 64 mm).
- [E] / under-constrained (listed for the user): tray slope (no camera for jsc2022e068617), lip height, bezel, corner radius, recess, key sizes (ratios in the
  trainer photo), the hinge side, tablet depth and roll.

## Camera mode and toggle
State after plan Task 6 (code: `game/cockpit/cockpit_camera.gd`, `hotas_buttons.gd`, `World.toggle_cockpit`).
- `OrbitCamera.Mode.COCKPIT`: the pose is the head (`position_gcrf_m`, `free_basis`) that `World._update_camera` takes from `CockpitCamera`;
  near plane 0.03 m (`COCKPIT_NEAR_M`). VR: the same path sets the XR base (`XrManager.set_base`), the headset moves on top; toggling in recentres.
- `CockpitCamera` reads the simulation snapshot in float64, not the scene nodes: head = body position + attitude * ((capsule stage base - CG) along the
  station axis + `CockpitConfig.eye.origin`), axes = attitude * eye basis, turned by the look. Body found by the capsule stage id and the
  vehicle id (so the Dragon stack, the Dragon alone and the capsule after the trunk jettison all work). Test: the eye node of the scene graph sits at the
  floating origin (= the camera) within 1 mm, also through the staging at warp x100.
- Desktop look: right-drag (OrbitCamera `free_look_rad`), clamped to +-170 deg yaw, +-80 deg pitch relative to the seat, F12 resets it. VR ignores the mouse look.
- `World.cockpit_active`, `toggle_cockpit()` (signal `cockpit_toggled(active)`), keys: `C` (`main.gd`; `VrPanelMouse.PASS_KEYS`), stick button `cockpit_toggle`.
  Every switch calls `_view_jumped()` (eye adaptation snaps). A mission start whose controlled vehicle has a cabin starts in the cockpit
  (`_on_mission_started`); a vehicle without one: the toggle does nothing and logs one line. A focus / ground-site change leaves the cockpit.
  `launch_vehicle` stays external (the launch is asynchronous). If the capsule body disappears the camera returns to the external view.
- `_keep_camera_above_surface` is skipped in the cockpit (the vehicle's physics decides where the head is).
- `HotasButtons` (`config/input_hotas.toml [buttons]`): `stick_device = "T16000M"`, `cockpit_toggle = 2`, `camera_next = 3`; the indices are
  [E] unverified (T.16000M button 3, 0-based; Task 9's VR run measures). `pressed(role)` is true on the frame of the press; `camera_next` calls `ScreenCamera.next()` (Task 7).
  Measure with `godot --path game --script res://tests/joy_probe.gd -- --buttons --seconds=30`.
- Test: `game/tests/cockpit_camera_test.gd` (needs the generated Dragon models; listed in `tools/run_godot_tests.ps1` under `-Gpu`).

## External cameras (left screen, Task 7)
State after plan Task 7 (code: `game/cockpit/camera_catalog.gd`, `screen_camera.gd`, `game/shaders/screen_feed.gdshader`, `World._update_screen_camera`).
- Cameras are configuration: `[[camera]]` entries of `config/cockpits/crew_dragon.toml` (`id`, `name_key`, `stage`, optional `vehicle` fragment and
  `requires` = a `capsule_state` flag, `mount_m` / `look` / `up` in the stage's body frame, `fov_deg` = horizontal). Order = cycling order = key slot order (slot k = k-th entry). Texts:
  `cockpit.camera.<id>` in `config/i18n/{de,en}.toml`. Five cameras, all poses [E] (own choices after the spec list, none measured): `chase` (on the capsule,
  16 m to the side, looking at it), `stage1_aft` (first stage interstage, looking aft), `stage2_engine` (boom at the second stage base, toward the nozzle),
  `dragon_trunk` (capsule side, looking aft along the trunk), `dragon_nose` (docking camera, only while `capsule_state.nose_cone_open`; last, so it has no
  key and is reached only by the on-screen camera button, user decision 2026-11-09 second).
- `CameraCatalog` (static, float64 from the snapshot like `CockpitCamera`): `available(sim, config = Crew Dragon)` = the ids whose stage a live body carries (a camera
  on a separated stage stays while that body exists, then vanishes), `mount_pose_gcrf(sim, id)` = body position + attitude * (stage base - CG + mount),
  camera axes attitude * (look, up). `body_of`, `entry_of`, `all_ids`.
- `ScreenCamera` (child of `World`, `World.screen_camera`): a `SubViewport` (`size_px` 1024 x 640, `update_hz` 30, `UPDATE_ONCE` per interval) with its own
  `Camera3D` in the same World3D, own empty `Compositor` (none of the main view's cloud / meter / display-tone passes) and `CameraAttributesPractical`. The camera
  sits at mount pose - main camera (floating origin) in double. It renders only while the cockpit is the active view (`render_always` for shots).
  `select(id)`, `next()` (config order over the offered ids), `current_id`; the current camera falling out switches to `chase`. The stick button `camera_next`
  (`HotasButtons`, only in the cockpit) calls `next()`. The console camera keys are wired by `ConsolePointer` (Console pointer, Task 8).
- Screen material: `screen_feed.gdshader` on `screen_left` (`material_override`), the display-referred path (`vision_display_referred`) as the placeholder
  before it. The feed is 1.6:1, the screen 16:9: the screen shows the feed's middle 90 % of the height (`crop_y`), nothing stretched.
- Exposure [call] (user decision, fix round 1): the feed has its OWN simple auto-exposure, independent of the pilot eye's adaptation: the feed camera's
  `CameraAttributesPractical.auto_exposure_enabled` (Godot's luminance reduction = the mean of the feed's own buffer, scale 0.4, `AUTO_SPEED` 1.4: a ~1.5 s time
  constant at 60 fps, ~3 s at 90 fps because Godot steps it per feed render by frame time). The buffer still holds the scene times the eye's gain (shaders apply it, ~1 by day, up to
  several 1000 at night in the lit cabin), which the metering divides out; the clamp (`AUTO_MIN/MAX_SENSITIVITY`, 1 .. 1e9 = luminance 1.25e-3 ..) is wide and fixed: it only guards
  the division on an all-black frame. A gain-scaled clamp was tried and dropped (each attribute change makes Godot re-meter at once). Limits: a night frame that is
  black except for a few lit parts (pad floodlights, a sunlit rim) shows exactly that (the mean is dominated by the black, the lit parts clip; nothing unlit is brightened
  that the scene does not light); the sunlit day views read right. Far cameras: terrain / clouds / atmosphere are centred on the main camera (spec "Far cameras"), accepted coarse.
- No overlays in the image: guidance / HUD geometry in the world (`TrajectoryView` target orbit, `OrbitLine`, `FlightTrailView`, `MapLabel`) is moved to the
  `RenderLayers.OVERLAY` layer only (bit 12, `game/world/render_layers.gd`); the feed camera's `cull_mask` excludes it and `CockpitView.CONSOLE_LAYER`, the main camera keeps
  the default mask (sees all). New overlay geometry calls `RenderLayers.mark_overlay(node)`.
- Camera name: a `Label` (`CameraName`, top left inside the visible 90 %, white with black outline) in the feed's SubViewport shows the selected camera's `name_key`
  through `I18n` (follows the language). It is set every frame the screen updates and `show()`n every time (a Control under that SubViewport came up hidden).
- Deviation from the plan interface: `chase` is a fixed mount on the capsule (the config entry), NOT the `OrbitCamera` chase pose of the plan; a free orbit around the
  vehicle would need the main camera's controls on the feed. Deliberate.
- `stage2_engine` (rule the catalog implements: offered while ANY body carries the second stage, whatever the Dragon does; vanishes when no body does; test in
  `camera_catalog_test`). Mounted on a boom 0.5 m above the stage base, 4.5 m off the axis, looking at the nozzle with a 60 deg field: during the first stage burn the
  nozzle is inside the interstage (the view shows the interstage hull), it is visible from the staging on (T+200 shot: bell, Earth behind). Poses stay [E] in the TOML.
- Tests: `game/tests/camera_catalog_test.gd` (stub simulation: offered ids, `requires`, staging, fall-out, cycling, pose to 1 mm; real session: T-60 list, mount =
  scene node * mount within 1 mm, staging at warp x100, feed renders in / not outside the cockpit, `camera_next`). Shots: `cockpit_shots.gd --screen`
  (`screenshots/m7-cockpit/screen/`; fix round 1: `.../fix1/{night_tm60,day_tp60,day_tp200,night_tp200}`). Frame time: `game/tests/screen_perf.gd`.
- Desktop frame time (RTX 4090, root viewport 1.65 x scale as the VR pixel proxy, vsync 60 Hz, Crew Dragon at T-60 in the cockpit): frame 16.7 ms in every state;
  root GPU median 2.5 ms (feed off), 2.7 (30 Hz 1024 x 640), 2.9 (30 Hz 512 x 320), 2.4-2.9 (60 Hz). The feed's own GPU time at a render: chase 1.7 ms mean
  (3.2 max), stage1_aft 0.3 (0.9 max) at 1024 x 640. At 30 Hz a render lands on every third frame of 90 Hz. Open (Task 9): the headset measurement against the
  11.1 ms frame budget with weather and pad / ascent scenes (the cabin pad scene is GPU-light), and lowering `update_hz` / `size_px` if it does not fit.

### Task 5b (helmet shell, console modules, re-check under the appearance model)
- Helmet shell (`[helmet]` of crew_figure.toml, `figure_layout.helmet_*`, `gen_crew_figure.build_shell`): fitted to the clean side profile spacex-anzug.jpg at the
  opening-height ruler (opening top edge to chin tip 265 px = the round-4 opening height 0.232 m -> 1142 px/m; helmet centre = figure eye - EYE_UP / EYE_FORWARD).
  Star-shaped about the helmet centre: a superellipsoid (exponent 2.5; front 0.125, rear 0.155, above 0.215, below 0.15, half width 0.135 with the egg taper) cut by flat
  facets (rear face y = -0.135, the sloped upper rear facet, flat sides z = +-0.125); the open bottom follows a sloped cut (rear corner (0.036, -0.134), 36 deg down
  to the chin). Result 0.27 m deep x 0.33 m tall (round 4: 0.375 x 0.33); 12 outline points of the photo within -1.6..+1.3 cm (scratch fit). One smooth group
  (per-facet groups gave a zig-zag of shading along the facet edges). The head bone and eye are unchanged (`head_centre_*` kept, the rear 0.03 ahead of the seat back).
- Face opening: `visor_outline` rows [el, half az], bottom (chin tip, 12 deg) to top (brow, 18 deg), widest 92 deg at the temple: the photo's boundary points projected
  onto the shell (0.2-1 mm). Visor dome 12 mm (flush in its frame), thin frame bead r 9 mm, black seal r 6 mm 2.5 deg inside; pivot buttons 9 deg behind the opening
  edge at el -30. Grey knit neck hood (`collar` material): a loft from an ellipse on the shoulders (`collar_height_m` 0.18 below the centre, `collar_radius_m` 0.105)
  up to the shell's lower edge.
- Panel lines: `grooves` (planes [n, offset, limit vector, limit] in the helmet frame) -> manifest `crew_figure.helmet_grooves` (figure frame) -> `cabin.gdshader`
  groove uniforms on the helmet surface only (`CockpitView._helmet_material`): the groove over the crown down the side through the notch, the line across the rear face.
- Key labels: optional `label` in `[[button]]` (only where a photo shows it: conv_42860a, the 2020 module row with the same key layout as discovery10, and training2):
  CANCEL, WATER DEORBIT, DEORBIT NOW, BREAKOUT, EXECUTE, DEPRESS RESPONSE, SUPPRESS FIRE, FIRE RESPONSE (cam1-5, 11-13), POWER 1/2, STRING 1A-2C, RESET 1/2 (cam6-10, 14-18),
  MAINS ONLY, DROGUES & MAINS, ENABLE BACKUP G-METER; 21 keys, the others blank faces. `cabin_layout.label_cells` packs one cell per labelled key (key-face aspect,
  8 px/mm, 1024 x 512); `surface_textures.cockpit_labels` bakes `cockpit_labels.png` (linear coverage; Pillow's bundled Aileron Regular, CC0, recorded in `sources.toml` as `font_aileron_pillow`; needs Pillow >= 10.1 with FreeType for `ImageFont.load_default(size=...)`, an older Pillow would silently fall back to a bitmap font and bake different labels) in `write_dragon`; the
  manifest `cockpit.buttons[id]` carries `label`, `label_cell`, `normal` / `up` / `right`, `cockpit.labels.file`. `CockpitView` gives each labelled key its own
  material (shader label uniforms project the cell onto the key face). Button ids / node names unchanged. The camera keys keep their function (user decision)
  although the real keys carry abort / fire / power labels.
- Red light strips: `cabin.gdshader` mark uniforms on `console_shelf` (10 mm along each module's bottom lip, red, 40 cd/m2 emitted, scene-referred each frame);
  the centre T-handle housing gets a red marking at its lower right corner (15 % x 40 %). No new geometry (the handle body stays the round-3 pad + T grip).
  Deviation from the brief ("T-handle at the right end of the shelf"): the 2020 photo (conv_42860a) is cropped at the centre handle, so the existing centre handle is marked red and no second handle was added.
- Albedo literals (stated linear vs sRGB): keys linear 0.03 (was the sRGB literal 0.09 = 0.009 linear), seat pads linear 0.035 (was sRGB 0.03), HOTAS linear 0.04
  (was sRGB 0.06), all converted for the sRGB uniform; seat shells stay sRGB 0.82 (= linear 0.64, white paint like the walls).
- Re-check under the unified appearance model (`screenshots/m7-cockpit/look2/`, default field key 0.2): boots 0.15 sRGB, black pads 0.07 (round 5: ~0.5): closed.
  Whites no longer clip at the frame edges; instead they read grey where the frame is dark-dominated or wide (hatch_crew1_1 wall ~0.46 sRGB, p90 0.59; pilot_eye wall
  0.73, p90 0.77) against the near-white walls of crew1_1 (appearance model, not changed here). Shelf modules 0.20 sRGB against the housing 0.10: lighter than the
  housing as in discovery2, a little lighter than conv_42860a.

## Console pointer, hand and touch (Task 8)
State after plan Task 8 (code: `game/cockpit/console_strip.gd`, `console_pointer.gd`, `hand_motion.gd`, `arm_ik.gd`, `CockpitView.pose_arms`, `ScreenCamera.CAMERA_BUTTON_UV`).
- Standing decision (user 2026-11-09): camera selection is on keys WITHOUT a label; labelled keys (DEORBIT NOW, SUPPRESS FIRE, ...) are `function = "none"` (the tap animates, nothing
  happens) until their real function is implemented. Ids stay stable. Standing decision (user 2026-11-09, second): FOUR camera keys, all for the left hand:
  `c_upper_1`, `c_upper_2`, `c_lower_1`, `c_lower_2` (blank, under the left screen's right end) = slots 1..4 = chase, stage1_aft, stage2_engine, dragon_trunk (catalog order);
  the fifth camera `dragon_nose` (last in the catalog) only through the on-screen button; `c_lower_round` (blank, under the centre screen, right hand) is `none`. Plus the on-screen camera button on `screen_left`
  (`ScreenCamera.CAMERA_BUTTON_UV`, bottom right, ~3.8 x 4.3 cm, drawn into the feed, flashes on use): tap = `ScreenCamera.next()`.
- `ConsoleStrip`: the console as a 2D strip (u to the crew's right, v up, from the centre screen centre); surfaces = 3 screens + 39 keys, each a rectangle with its 3D face. `clamp`
  moves any point to the nearest surface (gaps, module faces, strip edges), `to_surface` gives {surface_id, point_m, normal, uv}, `pick` casts a ray on the console plane.
- `ConsolePointer` (child of `World`, `console_pointer`): desktop = mouse ray onto the strip (a click farther than 3 cm from every surface hits nothing); VR (`vr_mode`, XR active) =
  `move_by(mouse counts)` moves a strip pointer (0.6 mm/count [E]) that always lies on a surface. Hand side: left screen and keys left of the left/centre gap = left hand,
  else right (1.5 cm hysteresis); the other hand rests. `pressed(surface_id, uv)`; `activate()` dispatches (camera button, `camera` keys, else "none").
- `HandMotion`: critically damped spring (exact solution, frame-rate independent, no overshoot; half life 0.08 s [E]: 50 % of a step after 0.08 s); tap = smootherstep press 0.12 s,
  release 0.15 s, depth = `HOVER_M` 25 mm [E], so the fingertip ends on the face plane (test: within 1 mm). The hover target is the surface point + 25 mm along the normal.
- `ArmIk`: two-bone IK on the figure skeleton rest pose (`for_skeleton`), end effector = index tip with straight finger; wrist bend <= 20 deg, upper arm <= 100 deg from rest, elbow
  towards the pole (`CockpitView.ELBOW_POLE`, Task 8b), tip clamped to the reach (`reached` false), no NaN. Bones are set as rest-pose-derived global transforms (upper arm, forearm, hand, three index bones; thumb and
  fingers follow the hand). `extend` 0 = relaxed grip curl (28 deg per joint), 1 = index straight, smoothed like the hand.
- `CockpitView.pose_arms({l|r: {tip, extend}})`, `idle_tip(side)` (the rest hand), `posed_tip`, `arm_result`: the torso leans about the waist (spine bone, <= 22 deg [E], 4 passes) when the
  target is beyond 97 % of the straight reach. Measured (centre screen at 0.69 m from the eye): centre and left screen and all left keys need no lean and are touched to < 1 mm; the right
  screen centre needs 14.5 deg; with the Task 5 key positions the pilot's module (outboard of ~0.5 m) is out of the commander's reach at the lean limit: KNOWN GAPS `r_upper_3` 31.8 mm,
  `r_lower_3` 45.7 mm, `r_upper_5` 61.5 mm (tip clamped, the click still fires; sampled keys only, the other pilot-module keys are short too), stated in `console_pointer_test.gd`
  `KNOWN_REACH_GAP_M` (the test fails when a shortfall moves by more than 2 mm); Task 9 re-measures after the seat / eye calibration.
- Tap plane (Task 5 fix): the keys lean (`[tray] slope_deg`), so `ConsoleStrip` gives every key its own face frame (normal / up from `tray_axes`, the manifest `button_frame` and `hit_faces` use the
  same), its strip rectangle is the face seen along the console normal, and the pointer's hover / tap move along that face normal. `console_hit` stays hidden geometry; the pointer test checks
  every key's tap points against the manifest face frame within 0.5 mm.
- Idle hands: Task 8b, see "Crew body and suit" (HOTAS grips while the props are shown, else the hands on the thighs).
- Mouse ownership in the cockpit (user VR run 2026-11-11, `CockpitUiGate`, `game/cockpit/cockpit_ui_gate.gd`, driven from `main.gd` each frame by `World.cockpit_active`): in the cockpit
  NO 2D UI shows - VR: the curved overlay/menu panel (`VrOverlayPanel`) is hidden and `VrPanelMouse.suspended` (it takes and pushes no event); desktop: the webcast layer, the menu layer and the
  legacy HUD are hidden (F1 does nothing there). Leaving restores what each part showed before. The VR mouse mode stays CAPTURED (relative motion), so plain mouse motion (no right button) reaches
  `ConsolePointer` (`_unhandled_input` -> `move_by`) and the left button `click()`s the surface under the pointer. Desktop (not VR): the visible cursor's ray (`pick`), the same left click; relative
  motion on the strip was not chosen there because the cursor already shows where the ray lands. The pointer is visible on the strip as `ConsolePointer.marker` (orange disc 4 mm radius, 1.5 mm above
  the face, layer `CONSOLE_LAYER` so the screen feed does not see it) besides the hand's index finger.
- Open: the VR run decides the feel (0.6 mm/count); tablets and the handle are not touch targets beyond the generic key rectangle; the desktop smoothness check is by the user.
- Tests: `game/tests/console_pointer_test.gd` (config key table, strip, HandMotion, ArmIk numbers, dispatch, figure arms with models). Shots: `cockpit_shots.gd --hand --cabin_keys`
  (`screenshots/m7-cockpit/hand/`: `hand_eye_NN_*` sequences from the eye, `hand_hatch_*`, contact sheets `sheet_*.png`).

## HOTAS props and crew occupancy (Task 9)
- Props: `hotas_stick` / `hotas_throttle` meshes (baked at the `[hotas]` grips by `gen_dragon_cabin.py`) are tilted by `CockpitView.set_hotas_axes(pitch, roll, yaw, throttle)` about their
  base (stick base 0.17 m, lever base 0.15 m below the grip along the seat back): stick +-`STICK_MAX_DEG` 15 deg (pitch > 0 forward, roll > 0 to the crew's right, yaw twist
  +-10 deg), lever `THROTTLE_TRAVEL_DEG` 50 deg centred on throttle 0.5. Source: `FlightController.input_pitch / _roll / _yaw` (last manual input, 0 on the autopilot) and
  `FlightController.throttle`; `World._update_cabin_occupancy` pushes them every frame (not while a shots / test script drives the pointer). View only.
- The idle hand rides the deflected prop (`idle_tip` = prop transform x the rest tip), so the commander's arm follows stick and lever.
- Dev aid `--hotas-nudge` (`Start-VR.cmd --hotas-nudge`): Alt+1 / Alt+2 pick stick / throttle, Alt+arrows (left / right crew side, up / down forward / back), Alt+PageUp / PageDown
  along the seat back; 1 cm per press, Shift 2 mm; each press prints the new `[hotas] *_grip_m` for `config/cockpits/crew_dragon.toml`.
- Tests: `cockpit_view_test.gd` (props on / off, crew 1..4 order s2, s3, s1, s4, tilt angles, hand follows the stick); shots `cockpit_shots.gd --hotas`
  (`screenshots/m7-cockpit/hotas/`). The VR calibration of the grip offsets is the user's run (`.superpowers/sdd/.../vr-run-checklist.md`); results go here.

## Standing decisions
| Date | Who | Decision |
|---|---|---|
| 2026-11-07 | user | Cabin "as 1:1 as possible" to the real Crew Dragon; commander (player) in seat s2, left of centre; stick + throttle on the armrests are a deliberate, switchable deviation (`[hotas] props`). |
| 2026-11-07 | user | Console straight, the three screens coplanar (`fold_deg 0`), after the photo check (discovery10, crew2_1, inspiration4_1). |
| 2026-11-07 | user | Closed visor stays copper (confirmed after the smoked-visor photos); only its shape follows the photos. |
| 2026-11-07 | user | Eye adaptation: no cabin special rule; the cabin light is the illumination (rule D, game default since b2a7fe16). |
| 2026-11-07 | ruling I / K | Corrections of our own measured cockpit tables (console width, legs, console, ceiling) are not vehicle physics config: fixed without a user decision, reported. |
| 2026-11-07 | ruling G | 9.3 m3 published = habitable volume; the 15.2 m3 shell volume stays as measured. |
| 2026-11-09 | user | Camera selection on UNLABELLED keys (+ on-screen button); labelled keys have `function = "none"` until their real function exists; second decision: FOUR keys (`c_upper_1/2`, `c_lower_1/2`, left hand), `c_lower_round` is `none`. |
| 2026-11-09 | user | Anatomical crew body from ANSUR II measures, suit hull with texture detail, folds by wrinkle maps + corrective shapes (no cloth simulation). |
| 2026-11-09 | ruling O | Shoulder / upper-chest crumple on a backward arm swing (plain skinning) is a documented known limit, judged by the user in the VR run; the raised arm is near-straight (photo-fitted reach, jsc066391). |
| 2026-11-11 | user | The player's figure (commander) has the visor open (raised over the crown); the other crew keep the closed copper visor. A visible hole in the suit is not acceptable (supersedes ruling O for holes only; the crumple stays a known limit). |
| 2026-11-09 | ruling D | Default button indices `cockpit_toggle = 2`, `camera_next = 3` [E], verified in the VR run. |

## Validation
- Wall check (scratch script, Task 1): inner profile >= 0.094 m from the exterior meridian and the neck well everywhere.
- Tag check: `detail_file.value_lines_without_tag("config/cockpits/crew_dragon.toml")` empty; unit vectors normalised
  (`tools/structures/tests/test_dragon_cabin.py` `test_cockpit_tables_are_tagged`; windows against the exterior table 2.5 cm / 0.55 deg;
  buttons >= 5 mm proud, screens coplanar within 1 mm / 0.1 deg). `python -m pytest tools/structures/tests -q`.
- `cockpit_config_test` (loader, validation, crew counts), `cockpit_view_test` (cabin in the scene, eye, figures, props, crew order, tilt), `cockpit_camera_test`
  (head pose in float64, toggle, floating origin within 1 mm through staging at warp x100), `camera_catalog_test`, `console_pointer_test` (73 checks: key table, strip,
  HandMotion, ArmIk numbers, reach gap, dispatch), `mission_catalogue_test`, `i18n_test`. Run: `tools/run_godot_tests.ps1 -Only <list> -Gpu`.
- Tests state: green on 2026-11-09/10 (pytest 372). **VR acceptance (Task 9 Step 3) pending**: seat height, HOTAS prop positions, toggle, F12, pointer in VR, hand look,
  shoulder, frame times are not yet measured; no VR result is recorded here.

## Diagnostics
- Cockpit shots: `game/tests/cockpit_shots.gd` (GPU, no `--headless`) with `--mission= --t_plus= --out_dir=` writing to `screenshots/m7-cockpit/<topic>/`; default = look shots
  (pilot_eye, gopro_discovery2, hatch_crew1_1, helmet_* with reference comparison sheets via `--refs=`), `--screen`, `--hand`, `--body`, `--hotas`, `--shoulder`, `--visor` (see the script header).
- Fidelity shots (Task 1): `cockpit_shots.gd --fidelity [--photo=<ids>]` renders one shot `fid_<id>` per row of `config/cockpits/photo_cameras.toml` (crew1_1, discovery2, discovery10: the Task 4 console fit poses, tagged; fits write back here) at the photo framing, with `fid_<id>_vs` and `fid_<id>_overlay` (50 % blend + our Sobel edges in red) to `screenshots/m7-cockpit/fidelity/<task>/`. Every shot now shows the left screen feed (`render_always` and `UPDATE_ONCE` on the two frames before the capture; `cockpit_shots_feed_test.gd` measures the screen region, mean 0.64 against black). `render_check --webcast --mission=falcon9_crew_dragon --t_plus=-60 --cockpit` shows the cabin view.
- GPU baseline before the geometry tasks (2026-11-21, RTX 4090, cockpit view with the feed, `render_check ... --cockpit --stereo=res://tests/data/pimax_dream_air_projections.json --gpu_time=120`, 3534 x 2838 per eye): median 23.4 - 24.4 ms (two runs), p99 29.7 ms. Budget for the fidelity plan: > +2 ms vs this baseline is reported.
- `Start-VR.cmd --hotas-nudge`: move the HOTAS props with Alt keys, each press prints the new `[hotas] *_grip_m` lines.
- `godot --path game --script res://tests/joy_probe.gd -- --buttons --seconds=30`: prints the 0-based stick button indices for `config/input_hotas.toml [buttons]`.
- VR run checklist (German): `.superpowers/sdd/2026-11-07-m7-cockpit-crew-dragon/vr-run-checklist.md`; frame times from `data/perf/perf_*.csv`.

## Open points
- **VR acceptance (Task 9 Step 3) pending.** KNOWN GAPS for it: `r_upper_5` out of reach by 30.7 mm at the 22 deg lean limit (re-measure after seat / eye calibration);
  shoulder backward crumple (ruling O); HOTAS grip points [E]; stick / camera button indices [E]; stick axis signs (pitch > 0 forward, roll > 0 right) to confirm;
  VR pointer input path (who owns the mouse in the cockpit); frame budget of the screen feed (`update_hz` / `size_px`) and of the morphs (untimed).
- Hand tracking: the Pimax OpenXR runtime sends no hand data, so operation stays on the mouse pointer; real touch comes with a runtime that delivers it.
- Far cameras: terrain / clouds / atmosphere are centred on the main camera, so the external feed on the left screen is coarse at distance (accepted).
- Sun-patch colour: the sunlit patch in the cabin reads cream (B/R 0.60 linear) against the illuminant (0.72-0.80); to be measured pre-tonemap (colour of the material vs the pipeline).
- Deferred minors: `task-*-review.md` in `.superpowers/sdd/2026-11-07-m7-cockpit-crew-dragon/` (e.g. feed flash on pilot-eye gain jumps, `capsule_body` not tied to the controlled body,
  no VR recentre on toggle-out, panel contrast under sun + AgX, thigh 12.3 vs 12.0 bound, 61.6k cabin triangles reducible).
- Cabin volume: our trim surface above the floor encloses 15.2 m3, SpaceX gives 9.3 m3 pressurized [S:
  spacex.com/vehicles/dragon] (read as habitable volume, ruling G). Re-check against a dimensioned source if one appears.
- Screen size has no published value; 0.382 x 0.215 m rests on one finger scale (+-15 %).
- Labels exist only for the 21 keys conv_42860a / training2 show legibly (Task 5b); functions beyond the four named stay `none`.
- Seat poses: one launch / entry recline (35 deg) for all seats; the real seats reportedly tilt between phases (not measured). Outer seats may be toed in (crew1_1); not modelled.
- Console struts, lockers in the lower corners (Wikimedia "17", "18"), the small display left of the handle and the hatch's inner mechanism are not in the tables.
- Starship and HLS cockpits follow after the Crew Dragon (spec).
