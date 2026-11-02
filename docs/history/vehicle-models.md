# Vehicle models: history

## Black shade sides (open point until 2026-10-31, fixed by the radiance refresh)

- Black shade sides in a static scene (user decision, lighting unchanged): the sky's radiance cubemap (ambient and reflections) stays
  black after the atmosphere tables load: `world.gd` uses `Sky.PROCESS_MODE_INCREMENTAL` and `sky.gdshader` reads only global uniforms,
  so Godot does not re-render it. In a real flight session a white diffuse hull's shade side reads `0 1 2`, after one sky-material write
  74-92, with REALTIME ~80. Not the hull shader, the camera or its height. An earlier probe seemed to refute this because its clock jump
  dirtied the cube; whether a running clock refreshes it in the real game is unchecked. Smallest fix: write a `radiance_serial` uniform on
  the sky material after the tables load and as the Sun moves; it brightens the shade side of every lit object. Evidence:
  `game/tests/shade_probe.gd`, `vehicle_look_shots.gd --poke / --sky_realtime`.

## Steel values until 2026-10-31

Steel: metallic 0.8, roughness 0.27 [E] (a diffuse share stands in for the sky's weak reflection); albedo sRGB 0.80 / 0.81 / 0.83, warm tint (1, 0.985, 0.96), mottle +-10 %. The diffuse share compensated the then black radiance cube.

## Analytic heat shield tiles until 2026-10-31

The tiles were computed per pixel in the shader (`heat_shield_tiles.gdshaderinc`): axial hex cells of (arc, station), the distance
to the cell edge box-filtered by the pixel footprint for the gap, a bevel normal faded out at 1-3 bevel widths, a per-tile hash for
the shade and the repaired tiles, all faded to the mean by the footprint. Shimmer: tiles ratio 1.081, error <= 0.124 (40 m), temporal
0.0001-0.010; cost vs the old steel in the belly close-up +0.024 / +0.087 ms (scale 1 / 2). Replaced by the baked maps (see the
hull shader section); the file is now `hull_noise.gdshaderinc` (hash and value noise of the steel only).

## Lowered dome and bells below the ship's base (finish pass, superseded 2026-10-31)
The ship's engines hung below its base (gimbal at station 0, exit planes 2.1 / 3.6 m below it, a closed base disc), so bells stuck into the booster
and looked like bumps in the dome. The finish pass moved the dome down instead (start 4.4 m below the top, 2.25 m high, apex 70.15 m), which only
moved the symptom. Replaced by the true arrangement: "Stage interface" in the subsystem file.

## Superseded by the real-frames round (2026-10-31)
The fins were drawn root-flat with cells along the tangent (a misreading of the core's lift normal), with catch hooks on 0 / 180 (invented). Replaced text:

    - `grid_fin.gltf`: pivot on the hull at the hinge, hinge axis +Y (radial). The core turns the lattice normal from the tangent
      dents: two noise octaves, ~1.2 m and ~0.38 m, rms slope 0.03), B slope energy sx^2 + sy^2, A heat-affected band coverage.
    | Grid fin | octagon 4.03 x 4.03 m (irregular, not regular: see subsystem "Fin plan round"), 11.5 cells across (0.35 pitch), depth 0.4 (catch version 0.7: hardware), curved catch corner, shaft 0.3 m | 13.5 m^2 at span 3.0 (chord 4.5), depth 0.35, rectangular | photos 2026-03: fins read as dark slabs with a SCALLOPED outer edge; the lattice cannot be seen | octagon lattice at the model's measured pitch 0.35 m (11 walls, wall 0.04, frame 0.07), serrated tip edge (every second cell capped at the rim, photo IE-Photo-44), 4 catch hooks only on the two catch fins (0 / 180 deg, plain fin at 90: the model has 2 catch + 1 plain; azimuths unconfirmed) |
    | Booster top / hot stage | 20 V teeth (18 deg pitch) between a top ring (y 22.98) and the hull, ~2.8 m high in the model view; no vent plates | 24 vent plates 3.2 m tall + louvres (first pass, read from photo 108) | chopsticks photo 2026-03: V truss ring with rod struts ~4.9 m high, posts + collars at the hull; the "louvres" of photo 108 are this truss | vent plates removed; V truss 20 teeth, 4.0 m + 0.4 m posts (photo height), collar ring; rods 0.3 m thick (the real ones are tapered) |
    | Shield edge | aft shell +-90 deg (to ~12 m), upper shell +-78 deg (14-38 m), nose shell closes all round (from ~38.5 m) | 75 / 65 / 55 / 30 deg at 0 / 20 / 35 / 52 m | photos 108 / 2026-05: the nose is dark all round and the barrel's sides are dark nearly to the edge: model | table replaced: 90 / 90 / 84 / 78 / 78 / 180 / 180 deg at 0 / 12 / 14 / 20 / 38 / 41 / 52.1 m; chine ends at 38.5 m |
    | Aft flap | hinge 12.0 m, span 3.69 m, swept outer edge: full chord to 7.2 m, then to 0.44 m at the forward end; area 36.5 m^2; wedge 0.45 -> 0.2 m; 4 lugs at 0.75 / 4.2 / 7.7 / 11.2 m | hinge 12 m, span 4, 48 m^2, thickness 0.35, chamfered rectangle | photo 108: the trapezoid with the cut forward corner agrees | model planform, wedge, 4 lugs, hinge fairing |
    | Catch pins | booster 45 / 225 deg at 66.5 m | model: catch hardware at the grid-fin corners (hooks) | the drawn hooks are on the fins; the pins stay as configured (a decision on the physics numbers is open) |
    | Shield coverage | heating model: full windward half | drawn +-78 .. 90 deg, nose all round | unchanged (drawing only) |
    | Grid fin | 13.5 m^2, span 3.0, chord 4.5, centre of pressure at r 6.0 | octagon 4.03 x 4.03 m (about 13.5 m^2 gross, ~85 % lattice cover), root at the hull: centre at 4.5 + 2.0 = 6.5 m | the area agrees, the lever arm is 0.5 m longer in the drawing |
    | Forward flap hinge | straight, parallel to the axis, 38-46 m | leans 19 deg along the ogive, 5.8 m | the flap normal at rest differs by ~19 deg (in the game the flap lies on the tangent plane) |
    - Interface look (verdict of the renders `screenshots/vehicles/ifc/cmp_*.png`): the arrangement agrees (open dark base, rims in the aft plane, bells inside, ring on the truss). Still different: the sea-level cluster is too wide (config ring 1.3 m, real 0.89 m); bells are smooth cones without regen ribs and with no visible chamber / pump detail; drawn throats sit 0.8 m higher than the model's (bell 2.1 / 3.6 m vs 1.3 / 2.8 m from throat to exit, the bell's `bell_length_m` also sets the plume anchor); the plate has no ribs, port rings or the two open covers of f12 138.9 s; the outer comb of stringer slots on the real aft skirt is not drawn; ship plumes pass through the booster at hot staging (no interaction, no vent effects). No real side close-up of the V3 stack exists in the references: the ring and its 0.6 m rest on the printed model.
    - Catch pins: the config has booster pins at 45 / 225 deg between the fins at station 66.5 m [E]; web reports for V3 put the
    - Coverage vs thermal: the shield is drawn from the 1:100 model (see "Shapes redrawn": +-90 / +-78 deg, nose all round); the aero heating

## Superseded 2026-10-31 (fix round after the real-frames round)
- Weld bead 12 mm x 10 cm wide (half width 5 cm), heat band 1.2 x; shader weld darkening 0.25 + roughness gain 0.22; dotted stringer lines on every line in a regular grid at 0.09 m (0.7 coverage).
- Bells: "the ribs do not show, cause not found" (the view never applied the engine material).
- Fin aliasing: "no shimmer measurement of the fin".

## Superseded 2026-10-31 (fin outline round)
- Grid fin: regular octagon on a corner (across flats 3.8 m, width 4.11 m), walls through the octagon's centre at pitch 0.47 m, a
  four-sided pyramid spike (0.22 m) under every crossing, toothed rim 0.66 m pitch / 0.25 m deep, 552 triangles, no cross rib,
  no band, no ring or struts, gross 11.9 m^2, lever arm r ~6.6 m. Replaced by the measured chamfered outline and cusped saddle walls.

## Superseded 2026-11-01 (forward flap hinge)
- Forward flap hinge drawn straight, parallel to the station axis at the hull radius R (4.5 m), the lean "stays undrawn on purpose"; flap pose basis (X station, Y e_r, stow) with the rotation about the body +X. Replaced by the leaned hinge (chord of the drawn hull).
- Measured on the 1:100 model (not used, user ruling 2026-11-01): hinge edge lean 18.9 deg, stations 42.28-47.75, azimuth 20.1 deg, aft-end radius 4.64 m.

## Superseded 2026-11-01 (forward flap from the STL)
- Forward flap placed on the chord of the drawn hull (commit 0cd9838): hinge 5.8 m centred on the physics hinge (39.1-44.9), lean 7.99 deg, aft-end radius 4.676 m, outline 5 points, 3 lugs, area "13.8 m^2" (real area of that outline 12.0; the number was wrong). Replaced by the STL face: 5.784 m at 42.282-47.75, lean 19.03 deg, 6-point outline, 12.37 m^2.
- Old text of the docs rows:
  >   against a re-implementation of it. Planform = the 1:100 model's (swept, wedge section, hinge lugs, `[flap.aft|fwd]`), the drawn hinge
  >   edge centred on the physics hinge, its axis the straight chord of the drawn hull over its stations (`vehicle_spec.drawn_hinge`,
  >   manifest `drawn_hinge_station_aft_m / _lean_deg / _radius_aft_m`: forward flap 39.1 m, lean 7.99 deg towards the axis going
  >   forward, aft-end radius 4.676 m = hull 4.49 m + the chord's sagitta 0.19 m so the line touches the shell at worst, inside the
  >   0.5 m hinge cover; aft flap: lean 0, hull radius, as before; one code path); it does NOT keep the physics area (see "Physics deviations for the user").
  > | Forward flap | hinge edge 5.8 m (the model's own reading: leans 18.9 deg at stations 42.28-47.75; NOT used, user ruling 2026-11-01), span 3.04 m, swept quad with an aft foot beyond the hinge end (-1.49 m), area 13.8 m^2, 3 lugs | hinge 8 m, span 3, 24 m^2 | photo 108: a wide swept flap, not measurable (perspective); model kept | model planform centred on the physics hinge, hinge axis the chord of the drawn hull (7.99 deg lean) |
  > | Forward flap | 8 x 3 m, 24 m^2 | 5.8 x 3.04 m swept, 13.8 m^2 (-43 %) | the forward flaps are the most overstated; hinge moment arm and pitch trim are affected |
  > | Forward flap hinge | straight, parallel to the axis, 38-46 m, azimuth belly +-110 | on the physics stations 38-46 (drawn 39.1-44.9) and azimuth; leaned 7.99 deg along the drawn ogive, the chord of the hull; the frames agree with the azimuth: the tile edge sits at ~+-105 next to it | physics hinge is straight (lean 0) at radius R; drawn lean 7.99 deg, drawn hinge centre 42.0 (= physics centre), radius 4.68 -> 3.87; the model's 18.9 deg / 42.28-47.75 is not used (user ruling) |
  > hooks; the real fins show a serrated outer edge we do not draw; the V-truss rods are cylinders, not tapered; the forward flap hinge leans
  > 7.99 deg (chord of the drawn hull); the nose is fuller near the tip in the model; the number of skirt ports and posts is a guess; the skirt has no photo.
  >   forward flap hinge leans 7.99 deg along the drawn ogive; the model's 18.9 deg is not used).
  > - Catch (frame IE-Photo-41-2, Flight 5, https://cms.interestingengineering.com/wp-content/uploads/2024/10/IE-Photo-41-2.jpg; scale: tower

## Superseded 2026-11-01 (flaps and grid fins follow the drawing, vehicle physics Tasks 3, 4)
Physics deviation rows, now none:

| Item | Physics (config) | Drawn (model) | Meaning |
|---|---|---|---|
| Aft flap | 12 x 3.69 m, 36.48 m^2, centroid (u 5.059, v 1.682) | same | none. Thin plate: the wedge and lugs carry no aero |
| Forward flap | hinge 42.282-47.75, lean 19.03 deg, 12.374 m^2, span 3.154, centroid (u 1.609, v 1.250), station 44.21 at d = 90 | same | none (the earlier "13.8 m^2" in these docs was wrong) |
| Grid fin | 13.5 m^2, span 3.0, chord 4.5, centre of pressure at r 6.0 | STL outline, gross 12.63 m^2 (cell map: 102.9 cells of 0.350 m), root at r 4.65 (hull + 0.15 m gap), tail at r 9.19, centroid at r 7.19 m; walls (0.04 m on a 0.35 m pitch, both families) block 22 % looking along the cells | the physics has 7 % more area (13.5 vs 12.63) and a 1.19 m shorter lever arm than the drawing (r 6.0 vs 7.19); rest attitude agrees (cells axial at d = 0) |
- Config oddity: the 13.5 m^2 fin at a 3.0 m span (radius_m 6.0) gave a 4.5 m chord.
- `super_heavy_descent.csv` before the re-subtraction (fins 3 x 13.5 m^2), Mach: C_D: 0: 0.900, 0.6: 0.950, 0.8: 1.000, 0.95: 1.150, 1.1: 1.450, 1.5: 1.380, 2: 1.420, 3: 1.750, 5-25: 1.880.

- Until Task 5b `nose_length_m` was 17 m against the drawn 13.4 m, a documented deviation.

## Crew Dragon, first drawing (Task 5, superseded by Task 5c, 2026-11-02)
Plain revolved wall with one bulge, an ellipse as nose on a flat seam at station 3.7 (hinge r 0.9), boxes as pods, triangular fins, capsule 4.5 m + trunk 3.6 m, SuperDraco pods at 36 / 144 / 216 / 324. User: the nose "looks like a knob". Corrected misreadings of the first model measurement: "M2 capsule 4.44" was the port's top without the nose (real 4.85, total 8.08); "fins 1.3 m" was wrong (full trunk height); the az 36 / 144 groups are the Draco ports, the SuperDraco bays lie at 63 / 117.
