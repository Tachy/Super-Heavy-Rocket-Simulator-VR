# Cockpit (M7): Crew Dragon cabin

Spec `docs/superpowers/specs/2026-11-07-m7-cockpit-crew-dragon-design.md`, plan
`docs/superpowers/plans/2026-11-07-m7-cockpit-crew-dragon.md`. State after plan Task 5 (cabin in the game): typed tables (Task 1), config loader (Task 2), Blender cabin and console generator (Task 3), crew figure (Task 4).

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
  `kind` push / covered / handle, `function` only with evidence, else `"none"`; the 18 keys under the left screen
  (clusters A, B) are `camera` with `camera_slot` 1..18 (upper row left to right, then lower row; user decision).
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
| Console placement | centre screen centre (2.512, -0.043, 0), normal (-0.707, -0.707, 0) (console up 45 deg from +X) | E | round 4 joint fit on crew1_1 (back lip (2.450, 0.229) and post bases, residual 2-7 px of 1200); every console item moved rigidly with it | +-0.06 m, +-4 deg |
| Screen layout | centre on the centre line, gap 0.020 (fix round 2, was 0.045), the three screens coplanar (console straight, fold_deg 0, user decision 2026-11-07) | S / D | user decision 2026-11-07 after photo check (discovery10 top-down, crew2_1, inspiration4_1: one straight front); gap discovery10 (90 / 780 px) | +-0.1 m, +-5 deg |
| Console housing | 1.32 x 0.54 x 0.15 m | E / D | discovery2 / discovery10 at the iPad-mini scale: screens span 1.22 m, shelf ends 0.03-0.05 m past the outer screens (corrected from 1.60 in fix round 1; the tablets stand above the outer screens) | +-0.05 m |
| Tablets | 0.1348 x 0.2032 m | S / E | Apple iPad mini spec; model assumed from the shape | - |
| Button positions | rows 0.17 / 0.20 m below the screen centres | E | discovery10 x along the screens' lower edges (fisheye: fractions per screen) | +-0.02 along, +-0.03 across |
| HOTAS grips | hip + 0.32 forward + 0.28 up the back = 0.09 m forward, 0.55 m below the eye (seat axes), +-0.25 / 0.27 lateral | E | armrest, forearm reach | calibrated in Task 9 |

### Crew figure (Task 4)
- `config/cockpits/crew_figure.toml` (every value tagged; proportions = Drillis and Contini 1966 fractions of a 1.78 m adult [E]; suit look from
  `sheet_suit` / `eva_suit_detail1-4`), `tools/structures/blender/gen_crew_figure.py` (+ pure `figure_layout.py`: bones, seat basis, manifest block) writes
  `crew_figure.gltf` into the Crew Dragon set (4.8k triangles). `vehicle_spec.load_dragon` loads it into `spec["crew_figure"]`; `VehicleModels` returns
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
  2 = +90, 3 = -90), `seat_s1..s4`, `console_housing` (one straight box and one straight lower shelf across all three screens, two struts; the shelf front plane is the lowest button face minus its height; tested: every button stands >= 5 mm proud of the housing, the three screens are coplanar within 1 mm / 0.1 deg), `screen_left|centre|right`
  (one quad each, exactly the TOML centre / size, UV 0..1 with (0, 0) top left, +normal toward the crew), `tablet_l|r` (`[[tablet]]` ids l / r),
  `button_<id>` (one node per button; the centre handle is the node `handle`, no `button_handle`), `light_<i>` (`[[light]]` rows in file order),
  `hotas_stick`, `hotas_throttle`. Manifest `cockpit`: `nodes`, `screens[id] = {centre, normal, up, size}`, `buttons[id] = {centre, size}` (all 39 incl.
  `handle`), `button_ids`, `tablets`. No materials in the manifest yet (the game side assigns them).
- Wall: the inner surface is the exterior meridian offset by `wall_m` along its outward normal; windows / hatch are cut in that chart (`cabin_layout.at_depth`).
  Window well = funnel from the round interior reveal (`window_reveal_radius_m`) to the exterior ellipse 2 cm under the outer skin (glass there), 6 mm bezel
  ring (`window_bezel_m`). Hatch: super-ellipse opening (exponent 4 as the exterior), door inner face recessed 3 cm, white trim `hatch_trim_m`.
- Simplifications (silhouette only, texture later): the handle is a raised pad with a T grip (not a recess); buttons are boxes / cylinders 6 mm proud (cover
  12 mm) on the shelf, whose front plane is derived from the buttons' own offset from the screens' plane (about 0.024 m proud), one shelf for all panels; seats are
  boxes (back, headrest, pan, lower-leg support, armrests) at the tabled hips, pan along the chest normal, shin support not measured; the HOTAS props
  are a base, shaft and grip at the grip offsets (calibrated in Task 9). `test_dragon_cabin.py` compares the windows with the exterior table (glass
  centre carried back along the normal: 2.5 cm / 0.55 deg, the Task 1 gate).

### Cabin in the game (Task 5)
- `game/cockpit/cockpit_view.gd` (`CockpitView`): child of the capsule stage node (`VehicleView._build_cockpit`, rebuilt with the stage nodes, `VehicleView.cockpit`;
  exists in every view). Cabin meshes by node name from `VehicleModels.load_for("crew_dragon")["cabin"]`; `eye_node` at `CockpitConfig.eye`; `screen_surface(id)`,
  `button_node(id)`, `set_crew(n)` (seats s2, s3, s1, s4; s2 = player body), `set_hotas_props(b)`, `set_player_head_hidden(b)` (collapses the head bone: the camera sits in
  that helmet), `figure_skeleton(seat)` (Task 8 poses the arm bones). Figures load through `VehicleModels.scene_for` (keeps Skeleton3D / skin; `_meshes_of` does not).
  Default crew = seat count; the mission's crew count is not wired to the view yet.
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
- Console: `[console]` carries `screen_gap_m` 0.020 (active-area gap, 2 x 0.007 bezel + 0.006 seam), `shelf_height_m` 0.10 (modules, top 0.0175 under the screens, proud of them by ~2.5 cm),
  `module_split_m` / `module_margin_m` (buttons are grouped into light grey modules by `cabin_layout.shelf_modules`, dark housing shows in the gaps), housing 0.1575 above the screens plus a
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
  (`--field_key` per shot; the dark console dominates the metering). Debug options of `cockpit_shots.gd`: `--only=<shots>`, `--no_lamp_shadows`, `--hide=<nodes>`.

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
- Helmet against eva_suit_detail2: egg-shaped shell (`egg_taper` 0.08, narrower at the front) with a low crown ridge (6 mm, fades before the visor); larger
  opening +-62 deg x -40..+38 deg; closed visor (35 mm dome) two-tone by shader (`visor_tint_split` 0.55: copper top, dark clear glass below, manifest
  `crew_figure.visor_tint`); white frame bead (r 16 mm) with a black seal bead (r 12 mm) just inside; round 5: round grey pivot buttons instead of the round-4 brackets (below) (round 4: 8 x 80 x 18 mm, beside the
  frame at eye height) instead of the grey blocks; grey collar (`[material.collar]` 0.30). Visor stays closed and copper (user decision).
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
- Still grey: the figures' boots, seal and pivot (albedo 0.05) read ~0.5 sRGB. Not light: they sit at the AgX pivot. At the shots' field key 1.2 a 5 %
  albedo is ~0.18 scene value, which AgX maps to mid grey; AgX contrast 2.0 (`--agx_contrast`) leaves them at ~0.53 while the console goes black. A darker
  rendering needs a cabin exposure / tone rule (eye-adaptation subsystem, user decision).
- Helmet: the visor pivot is a round grey button (`pivot_button_radius_m` 9.5 mm, `pivot_button_height_m` 4 mm, collar grey) at jaw height
  (`pivot_el_deg` -30) 13 deg behind the opening edge (jsc2024e066389: 19 mm disc ~0.09 m below eye level, ~25 mm behind the seal); `helmet_side_vs_jsc066389`.
- Debug options of `cockpit_shots.gd`: `--no_shadows`, `--lights_off=<patterns|sun>`, `--no_sky_light`, `--agx_contrast=<c>`.

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
  [E] unverified (T.16000M button 3, 0-based; Task 9's VR run measures). `pressed(role)` is true on the frame of the press; `camera_next` is read by Task 7.
  Measure with `godot --path game --script res://tests/joy_probe.gd -- --buttons --seconds=30`.
- Test: `game/tests/cockpit_camera_test.gd` (needs the generated Dragon models; listed in `tools/run_godot_tests.ps1` under `-Gpu`).

## Validation
- Wall check (scratch script, Task 1): inner profile >= 0.094 m from the exterior meridian and the neck well everywhere.
- Tag check: `detail_file.value_lines_without_tag("config/cockpits/crew_dragon.toml")` empty; unit vectors normalised.

## Open points
- Cabin volume: our trim surface above the floor encloses 15.2 m3, SpaceX gives 9.3 m3 pressurized [S:
  spacex.com/vehicles/dragon]. The photo-measured width (four seats, windows at r ~1.43) does not leave room for a
  9.3 m3 vessel of this height; either the published figure is a free / habitable volume or the floor / ceiling sit
  higher or lower than read. Re-check against a dimensioned source if one appears.
- Screen size has no published value; 0.382 x 0.215 m rests on one finger scale (+-15 %).
- Button labels on the flight panel are unreadable at video resolution; functions beyond the four named stay `none`.
  Candidates from reports (fire suppression, cabin pressure, trunk jettison, deorbit) are not tied to a button.
- Seat poses: one launch / entry recline (35 deg) for all seats; the real seats reportedly tilt between phases (not
  measured). Outer seats may be toed in (crew1_1); not modelled.
- Console struts, lockers in the lower corners (Wikimedia "17", "18"), the small display left of the handle and the
  hatch's inner mechanism are not in the tables (surface detail or later).
