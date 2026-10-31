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
| Carriage | spans the tower width (13.5 m outer) x about 9 m high; parked arm top 88 m | none | new part | Ship 42 |
| QD arm | reach face edge to ship wall 20 (2) m; deck 122 (3) m, truss bottom 104 (3) m above ground | none | new part | Ship 42 (6.4 px/m); QD photo gives 28 m at an unknown view angle, rejected |
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
| Booster-side platform at the hot-stage ring | black lattice platform with railing at the booster top (86.8 m), reaching about 12 m out from the leg; stays attached to T-0 | f13 y 560-600; f14 `qd_f14_2135_1s.png` 15 frames 1 s | retracts only after T-0 (not visible: smoke) - **swing timing open** |
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

## Open, needs more than these references
- Booster return and catch (carriage heights, rate, arm angles and timing, arm height at the catch) are absent from the
  flight 12-14 webcasts (see the webcast pass); another source is needed.
- Section splices and section count; chopstick plan width; carriage vertical travel and rate; hinge height of the QD arm
  (a plan view or a dated survey is needed); trench depth (no side view of the ramps); azimuth and distance on a
  georeferenced image newer than 2025-05 (a NAIP flight after 2025-05 or the FAA EA site plan).
- Points files (git-ignored): `points_chopsticks_booster.toml`, `points_full_stack.toml` (ratio, from the prototype
  task) and `points_ship42_solve.toml` (camera solve).
