# Launch structures: history

Superseded numbers and earlier approaches of the Pad B tower model. Current state: `docs/subsystems/launch-structures.md`.

## Prototype draft numbers (prototype plan tasks 1-5, replaced 2026-10-31 by fine build task 1)
- 12 sections x 12.04 m = 144.5 m of lattice, brace row pitch 4.0 m (~36 rows), `x_brace` for the two lowest sections and
  `chevron` above, one pitch for braces and floor decks. Measured instead: lattice 139.1 m, 26 floors of 5.35 m, X-brace
  panel 8.025 m.
- Brace H 0.4 m (look correction: thinner chevrons); measured 0.5 +-0.15 m (QD arm photo), 0.5 m again.
- Roof: 0.6 m cap with a 5 m pyramid frame on top reached 150.1 m (144.5 + 0.6 + 5), 4 % over the measured silhouette.
  Measured: flat roof deck at 144 m with a louvred block and a railing, no pyramid.
- Foot ring 3 m wide x 1.2 m high around the tower foot; measured: a closed plinth 12.2 m high; the ring is dropped
  (not resolved in the frames).
- No cladding, stairs, cable trays: the open lattice reached the ground.
- Tower steel: one material for legs and braces, sRGB tint 0.65 of the Metal032 map = linear albedo ~0.09 (comment said
  ~0.15); far tier colours (0.69 / 0.55 sRGB) never refitted after the user's darkening call of 2026-10-31. Replaced by
  bright legs (linear albedo ~0.15) and dark braces (~0.04), far tier refitted (real-frames item 11).
- Hand-over numbers at 12 sections (brace 1.04 px at 300 m desktop, 2.0 px VR): with the 0.5 m brace 1.30 px / 2.4 px.

## Chopstick before fine build task 2
Draft model: straight open truss from the hinge, length 26 m, width 3 m, depth 8.75 m, truss pitch 6.5 m, top rails 0.9 m, W diagonals, one
arm only (-across stayed a box). Replaced by the measured Pad 2 arm. The measurement rows "QD arm deck 122 m" and "carriage parked arm top 88 m"
had the two parts swapped (`launch-structures-measurements.md`, Arm research); the plan's `gen_qd_arm` at deck 122 m followed that error.

## Mount shape: pyramid replaced (2026-10-31)
The first square mount was a closed stepped pyramid (plinth 1.6 m, ledge 0.8 m, tan running-bond courses of 1.6 x 3.2 m) with 12 m tunnel mouths, two door
arches, a diagonal stair strip, hoods on the tower side, 12 clamps, a 6 m pit, 20 m ramps with a 0.15 m curb and a 55 x 5.5 m dark wall beside the ramp. Photos show
four piers with open portals, rectangular doors, hoods on the front side, large plates; the stair strip was a pier, and the dark wall is unsupported near the
mount (the only long low wall in the frames is the site's perimeter wall, person-scale photo), so it was removed.

## Physics before the Pad B alignment (2026-10-31)
Site 25.99686, -97.15394 (the imagery point, on the tank farm), height -21.11 m (orthometric +1.41, spread 3.1 m). Tower azimuth 315, 30 m, width 12,
box face-on, hinges derived 24 m, arm 36 m, mount 20 m, catch 95 m. Booster pins station 66.5, azimuths 45 / 225, either of the two rolls. Sea 90 deg,
500 m [E]. Abort test then: 1500 m 553 m / 12.3 t, 800 m 549 m / 10.0 t, 400 m 532 m / 6.0 t. Fresh SCvx stall window 4; one failed abort re-plan gave
the water up at once.

## Hardstand and site name before the Pad B alignment (2026-10-31)
Hardstand: convex 7-point plate `-48,-40 -20,-85 92,-85 112,-50 112,70 10,85 -48,35` (m, mount frame), +-15 m, from wide-angle views, low confidence.
Site id `starbase_olp1`, display name "Starbase OLP-1" (the OLP-1 imagery position); now `starbase_pad2` / "Starbase Pad 2".

## Hardstand triangle (2026-10-31, superseded the same day)
Triangle from the south tip -36 E / -74.6 N (+-8 m) and two edges (SW bearing 167 deg 218 m, SE bearing 63 deg 203 m), keys `hardstand_tip_*`, `hardstand_sw_*`, `hardstand_se_*`, cut in Y bands (`_x_range`, convex only). Replaced by the user's sketched 20-point outline `hardstand_outline_m`.

## Two hinges on outriggers (until 2026-10-31)
Physics: `hinge_distance_m` 22.3 and one hinge per arm at +-5.8 m across, each arm turning about its own hinge; arm 25.9 m. Drawn: hinge columns 3.7 / 4.3 m in front of the faces, tied to the collars by outriggers (`hinge_y_m`, `beam_m`); root 35 deg, lower chord with a knee (11 m at 21 %, 8.6 m at 35 %), knee strut, foot 17 m at the arm's line, collar distance 13 m, pushers on the tip, rails 0.2 m outside the faces, cables from the sheave block's side at 0.3 m pitch. Replaced by the user's ruling of one axis on the corner.

## Hinge axis on the box corner (physics plan Task 4d, replaced 2026-11-01 by Task 4e)
- Axis on the corner toward the mount: 22.378 m along the line, 1.244 m across; `arm_line_offset_m` 7.044 / 4.556 m; arm 25.98 m; standby 6.5 deg (hull clear 2.83 m per side); second site 24 m / 6 m, arm 36 m.
- Stations on the arm: bearing start 13 m, leg end 19.5 m, connector 23 m, plate end 25.2 m, pad 23.23 m, tip box 22.6-26.2 m, mast 13.5 m, lower chord's end at 0.87 of the arm.
- Rails on the corners themselves; sheave block 2.5 m wide with its tangent on the axis, outside the corner; 4 hoist cables in one bundle on the axis (0.16 m pitch).
- User ruling 2026-11-01 on the photo differences: take the photo for axis, rails and cables; rollers and rear corner stay the user's description.


## Leg colour (superseded 2026-11-01)
Ruling 2026-10-31 (real frames, `vehicle-models.md` "Black parts"): legs bare steel of linear albedo ~0.15 +-0.08, tint
(0.908, 0.834, 0.774) computed from the Metal032 map's linear mean (0.205, 0.248, 0.294) and the mean grime 0.91, grime strength 0.45
on legs and braces. Rendered: warm brownish-grey (lit face (123, 112, 103) sRGB, std 30) and mottled by map and grime. The photo
nsf_padb_chopsticks_2025-02 shows bright silver, clean legs; the warm tint, the full map contrast and the strong grime were the
causes. Replaced by albedo ~0.30 cool, map contrast 0.25, grime 0.08 (legs); braces 0.04 neutral.

## Tower top before the head (superseded 2026-11-01)

The tower top was drawn as a flat deck at 144 m with a centred louvred 8 x 8 x 7 m block and a railing (silhouette 146.9 m, `[tower.roof]`), and the hoist cables ended on an invented crown beam with two sheave blocks at 142.6 m (Task 6 fix 2, reasoning from the statics, no photo). `padb_tower_top_cabin_2026-04` contradicts the block and the railing and the user named the real part (the tower head with the overhanging hinge corner): all replaced by `[tower.head]`. Earlier text of the measurements:

- Roof: a flat deck with a light-grey louvred block (about 7 m, a vented machine / hoist room) and a railing; no cabin
  glazing visible from the stream.

Earlier text of the subsystem file: (courses of 5.35 m, two plates across, 0.1 m seams), roof deck 144 m with a louvred 8 x 8 x 7 m block (139.9-146.9 m) and a railing (silhouette top 146.9 m, the physics box is 146 m).

Earlier cable text: Their upper ends meet the fixed crown beam on the tower top (`[tower.roof] crown_*`, `mechazilla_layout.crown`, node `crown` of rails.gltf): a beam from inside the corner leg (1 m) along the outward diagonal to the axis, a cross beam over the axis along the groups, a 1.2 m sheave block on it above each group, underside 142.6 m (under the roof plate, top <= 146.9 m, so `check_envelope.py` adds no exception). [E] from statics only: `padb_tower_top_cabin_2026-04` shows the roof cabin overhanging the lattice with two cables hanging from its underside on the outer side, no beam or sheave resolved.
