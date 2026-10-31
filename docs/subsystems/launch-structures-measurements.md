# Launch structures: Pad B reference measurements

State 2026-10-31 (night pass, spec `docs/superpowers/specs/2026-10-28-pad-b-model-design.md` section A). Data only: nothing
in `config/launch_sites.toml` or `config/structures/starbase_olp2.toml` was changed; the full build applies the values
after the user's go. Tools: `tools/structures/measure/` (`ratio.py`, `camera_solve.py`, `gridcrop.py`), tests
`python -m pytest tools/structures/tests -q`. References and points files stay in git-ignored
`screenshots/mechazilla/pad-b/` (no photo pixels in the repo); crops with a pixel grid in `screenshots/measure/`.

## Method and honest limits
- Scales: **V3 ship 9 m x 52.1 m** held at the tower (`padb_chopsticks_catching_ship42_test_2026-09`: 58 px width and
  333 px length give 6.44 and 6.39 px/m, a 0.8 % cross-check); a standing person (1.78 m) at the mount front
  (`padb_mount_pyramid_front_person_scale_2026-05`, 35 px/m); pickup trucks (5.9 m) in the two top-down photos
  (16.5 px/m, `padb_aerial_tower_foot_mount_2026-09`); NOAA 2026-01: 0.20 +- 0.01 m/px (pickups 6 m = 30 px; the tower
  shadow length is consistent with a mid-January noon sun of about 45 deg).
- `camera_solve.py` (OpenCV `solvePnP`, SQPNP, focal length scanned, principal point = image centre) ran on the Ship 42
  photo (5 coplanar ship points, RMS 0.55 px, f = 2925 px, camera 457 m away). With coplanar points the camera pitch is
  weakly determined: the solve puts the camera 104 m above the ship base (13 deg pitch), which a roadside photo
  cannot have, and it reads the tower 4-6 % too tall (ground at -89 m instead of -82 m). It is therefore **used only as a
  consistency check**; the adopted numbers are the zero-pitch ratios (identical to the solve for a level camera). Only
  one reference has enough known 3D points, so the spec's three-view IoU check falls back to the proportion checks for
  the other views (to be reported to the user).
- The depth offset between the measured object and the scale object adds about +-2 % where the view is perpendicular
  to the tower-to-catch-axis line (Ship 42 photo: the arms are horizontal, so likely) and up to +-6 % otherwise.
- **NAIP cannot be used**: the Starbase NAIP in the data pipeline is the 2022 flight (`terrain-rendering.md`), older than
  the tower (stacked 2024-08-21) and the mount (lifted 2025-05-12). Plan values come from the NOAA 2026-01 frame (public
  domain, a single oblique frame: towers lean, so only ground-plane features at ground scale are usable) and the
  oblique NSF photos. Everything below is therefore **photo-only [E]**.
- The tower-to-pad azimuth is low confidence: the NOAA frame is not north-aligned (Pad 1 appears 19 deg south of due
  east of Pad 2 where the repo's site coordinates put it 29 m north); the Pad1-Pad2 separation of 390 m at 0.20 m/px is
  11 % below the 436 m of the site coordinates (either the olp1 or the Pad B coordinates are off by about 45 m).

## Resolved: 144.5 m is the whole tower, not the lattice
- Ship 42 photo (ground at the foot y 1005, roof-deck top y 70, railing top y 57, last lattice panel y 100; 6.4 px/m):
  roof deck 146.3 m, railing 148 m, lattice panels end at 141 m. Full-stack photo (nose tip = roof deck level, mount deck
  14.5 m, stack 124.4 m [S:wiki_starship]): roof deck 138.4-144 m. Mean **roof deck 144 +- 5 m**, lattice panels
  **139 +- 5 m** (roof frame / top block about 5 m, the photos show a flat deck block with railing, not a pyramid).
- So 144.5 m [E] is the whole tower to the roof deck (silhouette incl. railing about 146-148), consistent with the
  physics box 146 m. The draft detail file stacks 144.5 m of lattice plus 5.6 m of roof cap and reaches 150.1 m: **5-6 m
  (4 %) too tall**; the lattice should be about 139 m. Photos cannot prove the spec's 1 % (1.4 m) on the height; a
  published figure is needed for that.

## Comparison with the current draft and `[site.tower]` (olp1 numbers)
| Value | Measured [E] (+- ) | Draft TOML / `[site.tower]` | Deviation | Method, image |
|---|---|---|---|---|
| Tower, roof deck | 144 (5) m | 144.5 lattice; box 146 | within error | ship-scale ratio, Ship 42; stack ruler, full-stack |
| Lattice panels only | 139 (5) m | 144.5 (+5.6 roof = 150.1) | **-4 % (silhouette +4 %)** | same |
| Sections, section height | not identifiable (no splice lines visible) | 12 x 12.04 m | open | tower_full, Ship 42 |
| Floor levels / pitch | 26 (2) levels, 5.3 (0.5) m | 36 rows, pitch 4.0 m | **pitch +33 %** | row count, tower_full 2025-08 + Ship 42 crops (perspective makes rows 15 % larger at the foot) |
| Face width, leg centre to centre | 11.9 (1.0) m (outer 13.3-14) | 12.0 m | -1 % | pickup scale top-down, tower_foot 2026-09; silhouette 18.8 m at ~45 deg, Ship 42 |
| Leg (closed box) | 2.2 (0.4) m | 2.2 m | 0 | QD arm photo, Ship 42 |
| Brace members | 0.5 (0.15) m | 0.4 m | +25 % | QD arm photo (diagonals 28 px) |
| Plinth height | 12.2 (1) m | foot ring 1.2 m | new part | Ship 42 (foot y 927) |
| Clad face (solid plates) | 12 to 79 m on the pad-side face | none | new part | Ship 42, full-stack |
| Chopstick length hinge to tip | 27 (2) m | 26 m (draft), 36 m (`[site.tower]`) | +4 % / **-25 % vs olp1** | 40.6 px/m from booster, `chopsticks_around_booster_top` |
| Chopstick depth | 8.6 (0.8) m | 8.75 m | -2 % | same; Ship 42 (top rail y 440, bottom y 495) |
| Chopstick width | not measurable (no plan view) | 3.0 m; 2.0 m top surface | open | needs a top view of an arm |
| Chord / diagonal / rail tubes | 0.9 / 0.7 / not resolved (0.2) m | 1.0 / 0.7 / 0.9 m | within error | booster_top |
| Carriage | spans the tower width (13.5 m outer) x about 9 m high; arm deck 122 (3) m and truss bottom 104 (3) m in that test (corrected, see "Arm research") | none | new part | Ship 42 |
| QD arm | reach face edge to ship wall 18-20 (2) m; deck 88 (3) m, truss bottom 77 (3) m above ground (corrected, the old row read the chopsticks) | none | new part | Ship 42 (6.4 px/m); QD photo gives 28 m at an unknown view angle, rejected |
| Mount shape | square frustum, front face 14.5 m high, base 44.6 (2.5) m wide, about 40 (5) m deep, top deck 18.7 (2) m wide, faces about 48 deg | olp1: annulus r 13 / 5.5, 20 m high | **height -27 %, plan 44 m vs 26 m** | person scale + Ship 42 (front face), NOAA (depth) |
| Mount height (deck) | 14.5 (1.2) m | `mount_height_m` 20 | **-27 %** | person scale 14.9 m, booster scale 13.7 m, Ship 42 14.1 m; full-stack photo gave 19.6 m but its near-field perspective inflates the front face (rejected) |
| Launch ring | clamp ring outer about 10.4 (1) m, opening about 9 m, ring of ~12 clamps | olp1 inner r 5.5 (11 m) | -5 % | pickup scale, tower_foot |
| Trench | **two** open sloped exit ramps, left and right, axis perpendicular to the mount-to-tower line; each about 33 (5) m from the mount axis, about 20 (4) m wide; two door arches in the front face | none (visual only) | corrects `INDEX.md` ("one trench") | ship39-on-mount top view, NOAA, person image |
| Catch axis to tower face plane | 24 (2) m (projected, lower bound 23.7) | olp1: 30 - 6 = 24 | 0 | Ship 42 |
| Tower centre to catch axis (`distance_m`) | 32 (3) m | 30 m | +7 %, within error | Ship 42 33.9 m (projection), tower_foot 30-33 m |
| Azimuth pad to tower | 345 (25) deg, low confidence | 315 deg | **-30 deg, unresolved** | NOAA frame direction, assumes north from the shadow |
| Arm geometry check | arms 26 m, hinges at the corners (lateral 6.75), closed tips +-4.8 m reach 25.9 m vs measured 24.5 (2) m | `arm_standby_deg`, `arm_closed_gap_m` derived for 24 m | consistent, no fudge | physics rule of spec B |

## Biggest deviations from the olp1-based numbers
1. Mount height 14.5 m instead of 20 m, and a 44 m square frustum with a top deck of 19 m instead of a 26 m annulus.
2. Floor pitch 5.3 m instead of 4.0 m; lattice 5 m shorter than the draft's 144.5 m (roof included: 144.5 m).
3. Chopstick length about 27 m against `[site.tower]` 36 m (olp1), draft 26 m is right.
4. Two flame-trench exits (not one), opposite each other.
5. Azimuth 345 deg against 315 deg (low confidence).

## Flight webcast pass (2026-10-31, local streams flight 12 / 13 / 14, Pad B)
Frames in git-ignored `screenshots/mechazilla/webcast/tower/` (contact sheets `sheet_f1x_60s.jpg`, `tail_*`, `ret_*`,
liftoff sheets `lift_f1x_*`, grid crops `g_*`). Time codes are video seconds; liftoff (T-0) at f12 1924 s, f13 2105 s,
f14 2149 s (clock in the frame).
- **Not in the streams: no booster return, no catch.** Sampled f12 / f13 / f14 every 60 s from the start and every
  30 s over the tails, and every 5 s over T+5:55 to T+10:50 of each flight: after liftoff the stream shows only ship
  cameras (plus hot-staging, boosters' shot is not shown). So **carriage heights over time in a return, its rate, the arm
  angles and timing at the catch, and the arm height at the catch are not obtainable here** (still open; they need the
  booster-catch footage of another source, or the user's clips).
- What the streams do show: three liftoff views with the stack on the mount and the tower beside it - f12 ground camera
  (T-1:04, `f12_1860_full.png`), f13 drone (T-1:05, `f13_2040_full.png`), f14 ground telephoto (T-0:55,
  `f14_2095_prelaunch_full.png`); 2 s sheets from T-54 s to T+26 s.
- **Method limits.** The ground cameras look up (vertical scale falls from about 8.6 px/m at the foot to about 6 px/m at
  the nose: booster 475 px for 72.3 m, ship 309 px for 52.1 m, f14), so a single 9 m ratio is wrong along the height. Heights
  are read with a piecewise map through the known stack features (mount deck 14.5 m, hot-stage ring 86.8 m, nose tip 139 m)
  and are good to about +-4 m; a full camera solve found no stable solution (3 unknowns, 2 axis points). Depth offset
  tower to stack (about 24-32 m) adds a few percent. All values are [E], photo/video only.

| Value | Reading [E] (+-) | Method, frame | Against current data |
|---|---|---|---|
| Arms during stacked liftoff | parked at the ship nose: top rail 134 (4) m above ground (f14 132, f13 137), carriage block about 120 to 134 m | piecewise map, f14 t=2095, f13 t=2040 | the 88 m of Ship 42 was a catch pose; **stacked parking is about 45 m higher** |
| Arms open angle in that pose | span 277 px (outer ends) = 35.6 m at 7.8 px/m, so about 21 (5) deg from the line (lower bound: view may be oblique) | f14 t=2095 (outer ends x 813 / 1090, ship axis 940) | `arm_open_deg` 40 would span 48 m: the parked pose is about half open |
| Arms and carriage during the countdown | **no motion** from T-55 s to T+3 s (2 s sheets, 3 flights) | `lift_f12_1912_2s`, `lift_f13_2093_2s`, `lift_f14_2095_2s` | carriage / arms stay parked until the stack clears the tower |
| Roof deck / top block | box top 147-149 m, box underside about 142 m, louvred panel block about 7 m high; roof 146 (5) m | f13 t=2040 (nose 139 m at y 362, box y 318-350, 4.4 px/m) | agrees with 144 (5) m, upper half; keeps the "draft 150.1 m too tall" finding open at +-5 m |
| Clad lower tower | closed plate face from the foot to about 40 % of the height on the camera side | f13 `g_f13_2040_tower_low.png` | confirms "clad face 12 to 79 m" |
| Ship QD arm at the hot-stage ring (was read as a booster-side platform; "Arm research") | black lattice platform with railing at the booster top (86.8 m), reaching about 12 m out from the leg; stays attached to T-0 | f13 y 560-600; f14 `qd_f14_2135_1s.png` 15 frames 1 s | retracts only after T-0 (not visible: smoke) - **swing timing open** |
| Face width at the leg centres | 12.9 (1.0) m after the +32 m depth correction (106 px, 8.9 px/m at the booster) | f14 t=2095, legs x 886 / 992 | within error of 11.9 / 12.0 |
| Long dark wall right of the mount | about 5.5 m high, about 55 m long (crude, oblique) | f14 mount crop | trench / deluge wall, new part |

### Look details for the model (from the three liftoff views)
- Ship: black tile skin, nose and flaps black; booster: white / satin stainless, dirty (soot) in the lower third, black grid
  fins and black hot-stage ring cut-outs.
- Tower legs: bare steel, very bright silver in direct sun and near black in shade (no paint); X-braces and platforms
  black / dark charcoal; open lattice above about 40 % of the height, closed light-grey plate cladding below on the
  pad-side face; panels about 7-8 m between X-brace crossings in the open part (not resolved to the 5.3 m floor pitch).
- Roof: a flat deck with a light-grey louvred block (about 7 m, a vented machine / hoist room) and a railing; no cabin
  glazing visible from the stream.
- Chopsticks (parked): black open trusses, top rail with a lighter tan catch strip, pivoting about a corner hinge on the
  carriage, rounded outboard end, parked about half open; carriage a black frame with platforms.
- Mount: tan / gold-beige steel panels in horizontal courses with vertical seams (square frustum), black base ring in
  shade, two large grey rounded hoods flanking the booster on the deck, a tan diagonal stair or ramp strip across the
  front face, steam venting at the foot; sand-coloured pad apron.
- Neighbours: tank farm (two grey tall cylinders, two spheres, a dome) behind the mount at about 0.8 mount lengths.

## Catch arms from the Pad A catch webcasts (2026-10-31, flights 5 / 7 / 8)
Source: local streams `data/webcasts/flight5_2024-10-13.mp4`, `flight7_2025-01-16.mp4`, `flight8_2025-03-06.mp4`
(720p, git-ignored). Frames in git-ignored `screenshots/mechazilla/webcast/catch/` (`f5_*`, `f7_*`, `f8_*` = video
second; contact sheets `*_sheet_*`, `*_catch_*`). Only geometry and look were read; the catch sequence, timing, engines
and motion are in the physics and were not measured.
- **All three catches happened** (booster standing in the arms, not on the mount, for minutes): flight 5 (T+7:00
  close-up `f5_2405`, T+8:10 `f5_2475`, crowd reaction), flight 7 (booster hung T+9:23 to T+13:23, `f7_2970`,
  `f7_3060`, `f7_3210`), flight 8 (`f8_2572` T+7:14, `f8_2580`, `f8_2588`). Liftoff (T-0) in video seconds: f5 1983,
  f7 2407, f8 2138 (clock in the frame, +-2 s).
- **Old hardware, not Block 3 / Pad B.** These are Block 1 / 2 boosters (4 grid fins, other catch fittings) at the old
  Pad A tower with longer arms. Scale: 9 m diameter and known old booster length 71 m (Block 3 / Pad B frames: 72.3 m).
  The 146 m tower height of Pad A was not assumed: in `f7_3210` the 71 m booster gives a tower of 147 m (check).
  Each value carries a transfer tag: [T] transfers as a principle, [A] assumption needed, [N] old only, do not use.

| Value | Old catch (frame, time, +-) | Block 3 / Pad B estimate, transfer | Physics now |
|---|---|---|---|
| Arm deck (top chord) height at the catch | 133 (4) m above ground; booster held with the same deck height `f7_3210` (T+13:23, tower 332 px = 147 m, deck y 210); `f8_2572` T+7:14 gives 130-136 m | [A] about 130-134 m: Pad B parks the stacked arms at 134 (4) m, so the carriage reaches this height; the catch height is set by the tower top, not by the booster | arm 95 m above ground: differs by about 38 m |
| Booster bottom above ground when held | 64 (4) m (`f7_3210`: bell at y 365, tower foot y 510, 2.25 px/m); consistent with `f7_2970` | [A] deck 132 m minus length 72.3 m plus 2 m (see next row) = about 62 m | 28.5 m (8.5 m above the 20 m mount): differs by about 35 m |
| Booster top against the arm deck | old booster top rim 1.6 (1.0) m above the deck in the one side view (`f8_2572`, 6.2 px/m); `f5_2405`, `f7_3060` are oblique top-down views: "ring rim level with the decks (0-2 m)" there is NOT reliable (corrected 2026-10-31: foreshortening of 1-2 m is not resolvable) | [N] old only. **Do not use as the Block 3 deck-to-ring relation**: no Block 3 catch exists (only Block 2 / V2 boosters were caught). The Block 3 relation comes from the fins (see "Catch pins against the grid fins" below) | pins at 66.5 m: 2.5 m ABOVE the fin hinge, collides, see below |
| Arm length, tower face to tip | about 24-27 m: `f7_2280` T-2:07, arms parked fully open, tip to tip 153 px = 59.5 m at 2.57 px/m (tower 12 m wide); `f8_2572` oblique gives 35 m (nearer tower, perspective: not used) | [N] old arms are longer; Pad B arms are shorter (about 27 m, olp1 measurements above) | `[site.tower]` 36 m (olp1) |
| Arm truss depth | tip 6.5 (1.5) m, root at the tower 12-16 m (inclined lower chord, `f8_2572`, `f5_2475`) | [A] same layout (deep Warren truss with sloped lower chord), depth scales with the arm length | not in physics |
| Arm plan: two arms, left and right of the booster | arms parallel and straight while holding, about one booster diameter plus walkway apart; inner faces at the ring (`f5_2405`, `f7_3060`) | [T] two parallel arms; opening and closing as chopsticks about a corner hinge | pins at 45 / 225 deg (see catch check) |
| Walkway and rail on the arm top | flat deck with railing; diamond mesh grating pads at the inner end of each arm (left: 3, right: 2 pads) and along the top; a long ladder-like rail strip (rack, about 1 m wide) along the inner top edge, sloping to the tower (`f7_3060`, `f5_2405`) | [T] drawn as grated deck and strip on the arm top; pad count [N] | not in physics |
| Where the booster is held | old (V2, 4 fins): `catch_IE-Photo-41-2` shows the fins edge-on as horizontal bars at the very top of the booster, 150 px (about 10 m at the booster scale there) above the arms; `catch_IE-Photo-44-1` (on board along an arm): an orange block on the arm deck and the grid fin beyond it, above the arm plane. So on V2 the arms pass BELOW the fins. Corrected 2026-10-31: the earlier text "arms above the grid fins, fins not visible behind the arms" was wrong. The `f7_3060` / `f5_2405` waffle panels at the arm ends may be fins or arm pads: not resolved | [T] principle only (arms under the fins). No Block 3 station or azimuth transfers; see the Block 3 section below | pins at 66.5 m / 45 / 225 deg: see below |
| Carriage and mount on the tower | black frame with platform at the tower face; hinge frame at the arm root; top box (hoist / winch room) above the arms; tower top box 146 m with a mast (`f5_2475`, `f7_3210`) | [A] carriage as at Pad B: static frame at the face, arms hinge at the corners | not in physics |
| Paint | arms: black / dark charcoal steel, no paint colours; tan / orange scorch on the arm inner faces and hot-stage ring after the catch (`f7_3060`); tower legs silver bare steel with white hexagon X-bracing in sun (`f8_2580`), dark in shade (`f5_2450`) | [T] as above (Pad B finding: silver legs, dark braces) | not in physics |
| Booster top look | open ring: vertical stringers (corrugated wall about 3 m tall), dome and plumbing visible inside, stainless with soot streaks (`f7_3060`) | [N] Block 3 hot-stage / top differs | not in physics |

### Catch pins against the grid fins (Block 3, measured / inferred, 2026-10-31)
No Block 3 booster has been caught: no catch height, deck-to-ring relation or pin hardware exists for it. What is
measured on Block 3 and what is inferred:

| Item | Value | Basis | Tag |
|---|---|---|---|
| Fin layout | 3 fins, 90 / 90 / 180 deg; the two outer fins of the 90 / 90 chain are the OPPOSITE pair; the middle fin points away from the tower | aerial 2026-04, `f12` 1804 s (spacing); 1:100 STL guide `ssh3-booster.webp` (fins at front and left, right side empty) | measured V3 |
| Fin station | hinge 64.0 (config) / 64.6 (1) m (NSF 2026-03); hot-stage ring base 67.9 m; so hinge 3.3 m below the ring base. In `padb_chopsticks_around_booster_top_2026-03` (40.8 px/m): near fin top 1.4, bottom 2.7 m below the ring base (view about 11 deg from below; side fin appears 2.6 m lower than the near one, only about 0.8 m explained by perspective: fin deflection or unequal stations not excluded) | NSF photo, crop `v3_padb_arms_fins_grid.png`; `f12` 1804 s gives ring base to fin 1.2-2.6 m, the third fin 8 m (strong upward perspective): +-1.5 m | measured V3 +-1 m |
| Fin shape | 1:100 STL outline (irregular octagon, mirror-symmetric, 3.53 x 4.54 m, 12.63 m^2, pitch 0.350 m, cross rib 1.75 m from the root), checked against the installed-fin image (102.9 cells both); cusps / scallops from photos, thickness 0.5 m. Earlier chamfered 9-gon (0.49 m pitch, 11.80 m^2) superseded: `vehicle-models.md` "Fin plan round" | `detail/starship_block3.toml`, STL, installed-fin image | STL +-0.01 m |
| Fin mount (V3) | B19 image (user 2026-10-31, B17 / B19 side by side): round dark mount ring on the barrel below the truss, ~1.4 m (14 px at 9.1 px/m); centre 547 px above the booster bottom of 607 px (72.3 m) = station 65.2 m uncorrected; looking up compresses the upper part, so the true station is a little LOWER: 64.7 +-1.5 m, config 64.0 agrees. V2 (B17) had a block right at the top. Drawn as a round ring (disc 1.4 m, 0.12 m) | image ratio | measured V3 +-1.5 m |
| Hardware under the fins | none visible. `f12` 1804 s cannot show it: the hull below the fins is covered by vapour. NSF 2026-03 shows the near fin directly above the arm's top tube (fin underside at the arm rail) but no pin, pad or block | frames | undecidable |
| Arm | depth 8.6-8.75 m, deck top tube round, inner faces 4.8 m from the axis (`arm_closed_gap_m` 9.6), width 3 m | NSF 2026-03 (ratio), config | V3 inferred |
| Fin catch bearing | flat bar under the fin root beside the toothed hinge ring: ~0.5 x 0.15 m (+-40 %, oblique, fin ~170 px/m), slanted ~30 deg (+-12); on one fin of the opposite pair, absent on the 90 deg fin (the two-fin image: fins +-53 deg from the view axis, nearer the 90 deg gap than a 180 deg one); the second opposite fin is unseen. Drawn on 0 / 180 (fallback). Plausible as a bearing (faces down onto the arm top, the fins sit just above the top chord in the arm photos): user ruling, not established. The cylinder and soot streak first read next to it are shadows / reflections on the steel (user) | user images 2026-10-31 (red circle, two-fin view), the stacking image, `padb_chopsticks_around_booster_top_2026-03.jpg` | open |

Clearance by numbers (fin hinge 64.0 m; fin plate span 4.26 m radial, chord +-1.9 m, axial +-0.25 m plus 0.22 m spikes):
- Fin axial band: neutral (cells axial) 63.5 to 64.25 m; at the 30 deg deflection limit 62.6 to 64.3 m (chord 1.9 x sin 30 = 0.95 m).
- A fin lying within about 18 deg of the arm axis passes between the arms (its lateral extent stays under 4.6 m); a fin more than about 18 deg off the arm axis (lateral, over an arm) has its tip at r 8.8 m over the arm deck (inner face 4.8 m, outer 7.8 m): the deck must lie below its band.
- Layout A (arm axis along the middle fin, tower on the gap side): the two OPPOSITE fins lie over the two arms (symmetrical), the middle fin passes between the arms. Layout B (arm axis along the opposite pair): the middle fin lies over one arm only, the other arm is free. Layout C (arms at 45 deg, today's pins): all three fins lie over arms.
- Highest deck top that clears every layout with 0.3 m margin: **63.2 m (fins neutral), 62.3 m (fins at 30 deg)**. Pin axis about 0.3 m above the deck top (tube radius), so the pin line may stand at 63.5 / 62.6 m.
- Today's pins at 66.5 m: deck top about 66.2 m, 2.0 m above the highest fin point and 2.5 m above the hinge. The arm (8.6 m deep) then covers 57.6 to 66.2 m and runs straight through the fin band 62.6 to 64.3 m in every layout (a fin over an arm: A 2, B 1, C 3 fins). Visually it collides; in the physics the fins have no contact with the arms (no fin contact in `contact/`), so the effect there is only on the catch pose.

Factory photo of a real Block 3 fin (`screenshots/starship_v3_ref/eiv42cz7buif1.jpg`): a chamfered square hung on a corner,
one forked hollow pylon at the root corner (hinge / actuator), lattice with scalloped rim; no pad, block or pin on the fin or
on the pylon fork. So the bearing hardware is not part of the fin; the plate extent used above (corner root, about 4.1 m
diagonal) stays within the same axial band to the photo's accuracy (chord not measured to better than +-0.3 m).

User hypothesis "bearing points directly under the two opposite fins": **not contradicted, geometrically favoured, not proven.** For: layout A is the only one in which both arms see the same station and azimuth (pins at 0 / 180 under fins 0 / 180, the arm axis along the middle fin, the fin-free 180 deg gap towards the tower, arm deck under the fin underside as in the NSF frame); the STL has two thickened "catch" fins plus one plain; the existing finding that the middle fin points away from the tower. Against / open: no hardware or reinforcement seen under any fin (f12 1804 s cannot show it); the STL does not say which two fins are the catch pair; the fin heights in the NSF frame differ by more than perspective explains.

Proposal (config left untouched): `[stage.catch_pins]` `station_m` 66.5 to 62.5 (deck 62.2, 0.4 m below the 30 deg fin edge, 1.3 m below the neutral fin edge), `azimuths_deg` [45, 225] to [0, 180] (under the opposite fin pair; unproven, the alternative [90, 270] (B) puts one pin under the middle fin). Booster bottom above the deck: station 62.2 instead of 66.2, so for the same bottom height (28.5 m at deck 95 m) `catch_height_m` 95 to 90.7 m, or with 95 m kept the bottom hangs 4.3 m higher (32.8 m). The deck height at a real Block 3 catch is unknown: the old deck readings (133 m, bottom 64 m) are V2 (deck at the top rim, a different pin station) and set no Block 3 number.

Open: the pad shapes on the inner faces, the arm width at the tip (about 8 m at the end of the booster ring, one view),
hardware under the fins (needs a view of the hull below a fin without vapour, or a Block 3 catch), the V2 waffle panels at
the arm ends in `f7_3060` (fins or arm pads), which two fins are the catch fins.
Crops with grid: `screenshots/mechazilla/webcast/catch/v3_padb_arms_fins_grid.png`, `v3_f12_1804_fins_grid.png`,
`v3_stl_guide_top.png`, `v2_f7_3060_grid.png`.

## Arm research (Pad 2 / V3, fine build task 2, 2026-10-31)
Separates what is **[S] sourced** from what is **[R] read from an image**. Pad 1 / V2 facts do not carry over: V2 had its fin block at the very top of the barrel and pins near the rim (B17), V3 has a round mount ring clearly lower on the barrel (B19, user comparison; about 7.4-8 m below the rim of the 72.3 m hull, station ~64.3-65, low camera so +-1 m). The mount moved down, the arm deck goes with it; every old catch-height reading (`f5`, `f7`, `f8`, deck at the rim) is V2 only.

### One tower QD arm, and two rows above were swapped
- **The "upper QD arm at deck 122 m" is the chopstick pair holding Ship 42, and the "carriage parked arm top 88 m" is the QD arm.** Ship 42 frame (6.4 px/m): the upper structure with the big V struts holds the ship at y 225-340 = 122-104 m and has the hoist cables and sheave block; the lower lattice at y 445-520 = 88-77 m reaches the ship's aft with a connector head. Corrected rows above.
- The lower arm / platform at 86.8 m is the **ship quick-disconnect arm (SQD)**, not a booster or hot-stage connection. Evidence: [S] NSF's file name "Pad-2-SQD-Attached-to-Ship-39" for the photo with the jointed arm and white boxes: its head meets the hull a few metres above the hot-stage ring (tiles above it); [S] NSF 2026-09: the Ship 42 test practised reconnecting the SQD arm to a caught ship, exactly this frame; [S] NSF: the Pad 2 arm is mounted lower than Pad 1's although the stack is taller; [S] the booster QDs are on the mount (two, split LOX / methane, moved to its far side), so no booster arm on the tower; [R] it stays connected to T-0 in the 1 s frames (`qd_f14_2135_1s`), it swings away after (not visible, smoke). No source names a second arm or a hot-stage connection. `padb_chopstick_arm_backlit_2026-04` shows this QD arm too, not a chopstick (tip platform, stair, head): the INDEX line is corrected.

### Sourced [S]
- Pad 2 arms about 10 m shorter than Pad 1's, two full truss sections removed; boosters 12, 14, 15 were already caught on the inner part (NSF 2025-08, 2026-05).
- Actuators electromechanical, no longer hydraulic (NSF 2026-05; one low-grade blog mentions a "hydraulic pin" of the QD arm, unresolved).
- Catch pin centring: one sled on each catch beam plus a **telescopic pusher at the end of each arm**: these are the long cylinders on the arm top at the tip. Simpler lateral pistons (NSF 2025-08).
- Revised **stabilizer arms** reaching further (to clear a ship's aft flaps); no ship lifting pins (NSF 2025-08): the hanging probes with thin braces.
- Carriage: redesigned skates; drawworks in the tower base under panels, cables through the tower (NSF 2025-08, 2025-12). No source for the sheave block or for shock absorbers on Pad 2 (the block is read from the photo; shock absorbers undecided).
- V3 grip: fins lowered "to align with the tower's catch" and "a new catch point addition" (nextbigfuture 2025-08, a repost of SpaceX statements); "the arms catch from below", catch point in the methane tank zone near the fin base (The Nov Tech 2026-09). Conflict: a wiki says the fins take the load, later that dedicated points do. **No Block 3 catch has flown; the bearing layout stays [R] / user ruling.**
- SQD arm: repackaged, swings farther off, shoulder reinforced (NSF 2025-11, 2026-04).
- Limit of this pass: nasaspaceflight.com pages answer 403 to direct fetches, so NSF text is from search excerpts; no top view or annotated analysis of a Pad 2 arm was found. New image: `teslaoracle_chopsticks_v2_full_2025-03.jpg`.

### Read from images [R]
- Section: two top chords (round, about 1 m) and a single lower chord: a V section with struts converging to lower nodes (user view; side view 2025-08). Root columns reach a foot 17 +-2 m below the deck (`padb_mount_and_chopsticks_side_2025-08`, arms spread: the best side view, 14.3 px/m), knee 11 +-1.5 m at 0.21 L, lower chord rises to 8.4-8.6 m at 0.35 L, ends at 0.87 L (6.9 +-0.7 m, user view).
- **Kink: yes, in plan, inward.** Straight lines stay straight in a photo, and the top chord breaks in `padb_chopsticks_around_booster_top_2026-03`, `padb_chopsticks_above_mount_2026-05` and the user view at 0.19 +-0.03 L and 0.92 +-0.03 L. The side view 2025-08 shows the same deck flat, so the breaks are not vertical. Tip break: image slopes 5.6 / 32 deg at ~11 deg from below give 28 +-15 deg, receding from a camera outside the arm: inward. Root break angle **not measurable** (the one high view, `images.jpeg`, 678 px, gives no consistent solve): 5 +-5 deg from the geometry (hinge 6.75 m, main beam 6.3 m). Main beams parallel when closed: `images.jpeg` shows the two outer pieces parallel at an unknown near-closed pose; the user view is consistent, it cannot prove it. Drawn so, tested to 0.1 deg.
- Tips (user view): at the hull, grey block 2.1 x 1.2 m with its top 0.6 m below the deck on the hull side, round end caps. The real block sits at the arm's end, which appears nearly at the axis station; the drawn arm runs 3 m past the axis because the physics hinge is 24 m from the axis. Open.
- Width 4 +-1 m from the deck in `images.jpeg` (weak).

### User view of a held V3 booster: deck against the fins (first V3 measurement)
Image 1179 x 1240, hull 415 px wide at the rim = 45.6 px/m; horizon y ~960, rim ellipse half axes 207 / 65 px, so the camera is ~20 deg below the rim, about 16 m lower and 45 m away (f ~2075 px). Readings: rim ellipse centre y 200, near arc y 137; left arm deck top y 517, left fin plate 412-477, near right fin underside y ~470 at the deck y ~470-480.
- Fin underside to deck top: **0.0-0.9 m** (right 0-0.3, left 0.9): the fin plate stands just above the deck. Robust: both at about the same depth.
- Rim to deck in the plane of the axis: (517 - 200) / 45.6 = **6.9 +-0.6 m, station 65.4**. The rough 8.7 m (station 63.6) measures from the near arc (y 137, 0.3 hull diameters nearer) to a deck at the hull's side: mixed depths, too long by the 1.4 m ellipse half axis.
- The same view gives the fin itself higher than the config: fin root 7.1 m below the near arc, +-0.8, station ~65.2 against the config's hinge 64.0 (NSF 2026-03: 64.6 +-1; B19 comparison 64.3-65).
- Pin proposal test: **62.5 m ties the deck to the fin correctly only with the fin at 64.0** (deck 62.2 is then 1.3 m below the neutral fin edge, the view says 0-0.9 m). Better: the deck follows the fin: deck top = hinge station - 0.5 - (0..0.9) m. With the hinge 64.0: pin line ~63.0 (deck 62.7); with the hinge at the view's 65.2: pin ~64.2. Recommended for the physics plan: first settle the fin station (fin agent), then the pin at hinge - 1.0 m +-0.5. Azimuths 0 / 180 are confirmed by the view: both arms lie under two opposite fins, the third is behind the hull.

## Open, needs more than these references
- Booster return and catch: the flight 12-14 webcasts have none (see the webcast pass); the catch arm geometry from
  the Pad A catches is above (old hardware, transfer tags).
- Section splices and section count; chopstick plan width; carriage vertical travel and rate; hinge height of the QD arm
  (a plan view or a dated survey is needed); trench depth (no side view of the ramps); azimuth and distance on a
  georeferenced image newer than 2025-05 (a NAIP flight after 2025-05 or the FAA EA site plan).
- Points files (git-ignored): `points_chopsticks_booster.toml`, `points_full_stack.toml` (ratio, from the prototype
  task) and `points_ship42_solve.toml` (camera solve).
