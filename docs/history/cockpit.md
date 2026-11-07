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
