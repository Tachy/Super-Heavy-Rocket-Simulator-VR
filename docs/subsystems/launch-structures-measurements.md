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

## Open, needs more than these references
- Section splices and section count; chopstick plan width; carriage vertical travel and rate; hinge height of the QD arm
  (a plan view or a dated survey is needed); trench depth (no side view of the ramps); azimuth and distance on a
  georeferenced image newer than 2025-05 (a NAIP flight after 2025-05 or the FAA EA site plan).
- Points files (git-ignored): `points_chopsticks_booster.toml`, `points_full_stack.toml` (ratio, from the prototype
  task) and `points_ship42_solve.toml` (camera solve).
