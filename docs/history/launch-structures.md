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
