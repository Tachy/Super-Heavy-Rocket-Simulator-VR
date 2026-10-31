# Vehicle detail models (Super Heavy Block 3 + Starship Block 3, prototype)

State: plan `docs/superpowers/plans/2026-10-31-starship-booster-prototype.md`, tasks 1 (models generated), 2 (game side) and 3 (hull / tile shader,
look pass, evidence) done; the prototype awaits the user's verdict and a headset run. Only `starship_block3`; depot, tanker, HLS and Falcon 9 keep today's view.

## Model
- Physics numbers are the master: `config/vehicles/starship_block3.toml` and `config/engines/*.toml` (length, diameter, nose,
  flap hinge azimuths / stations / spans / areas, grid-fin station / radius / area / azimuths, catch pins, engine rings,
  nozzle exit diameters, belly azimuth). `config/vehicles/detail/starship_block3.toml` holds what the physics does not
  need (every value `[E]` or `[S]`, keys in `config/README.md`) and repeats none of the physics keys (tested).
- `tools/structures/vehicle_spec.py` resolves vehicle config + detail file + engines into one JSON spec (Blender's Python
  3.10 has no `tomllib`); `python tools/structures/build_models.py --vehicle starship_block3 [--force]` runs the four
  generators headless and writes git-ignored `data/models/starship_block3/` with `manifest.json` (input hash, triangles,
  per-flap rotation sign, engine exit planes, frame statement). Generation goes to a `.new` sibling and is swapped in only
  when every generator succeeded; no Blender: one message, exit 0 (the game keeps the cylinders).
- Frame of every glTF (export_yup off, Blender axes kept) = the config's body frame: X station axis (the stage's station 0
  at X = 0, nose +X), (Y, Z) = r (cos a, sin a), belly azimuth 270 = -Z. The game maps body (x, y, z) to render (y, z, x)
  like `Frames.to_render`; `vehicle_view.gd` places stage geometry at station along render +Z, so the models need that one
  permutation and no other offset.
- `booster.gltf`, `ship.gltf`: nodes `hull` (lathe, ring-weld beads from the ring pitch, radius = config radius + bead only;
  ship: tangent ogive of the config's nose length with a blunt tip; booster: flat top), `hot_stage` (flange + 24 vent plates
  with louvre ribs), `fin_fairings`, `catch_pins` at the config stations / azimuths / protrusion, ship `chine` (swept strake
  along the tile boundary, azimuth = belly +- the coverage half angle of the tile table) and `raceway`. Hull UV = (arc metres
  at the barrel radius, station metres): glTF u = arc, v = station (Blender v = 1 - station).
- `flap_fwd.gltf`, `flap_aft.gltf`: hinge line = X axis from the aft end (origin), panel along +Z at rest (stowed, leeward),
  outer face +Y. The game's basis (X station, Y e_r, Z stow direction, mirrored when the stow points to the belly side) and
  the rotation about +X by `rotation_sign_about_plus_x` x deflection are derived from `core/src/aero/flap.cpp` and tested
  against a re-implementation of it. Planform rectangular so that it keeps the physics area (chamfered corners: -1.3 %).
- `grid_fin.gltf`: pivot on the hull at the hinge, hinge axis +Y (radial). The core turns the lattice normal from the tangent
  toward +X: a right-handed rotation by +d about the radial axis, zero = rest. Chord = area / span, span = 2 (radius_m - hull
  radius), both from the config. Egg-crate lattice, cells open along the tangent.
- `raptor.gltf`, `rvac.gltf`: origin = gimbal point, thrust +X, bell opens toward -X, exit plane at X = -`bell_length_m`
  (manifest `exit_plane_x_m`: the plume re-anchor of task 2), exit rim diameter = config value (tested), bare bell +
  housing above the gimbal.

## Game side (`game/vehicle/vehicle_models.gd`, `vehicle_view.gd`)
- `VehicleModels.load_for(vehicle_id)` -> `{}` (cylinders, one log line) unless the vehicle is `starship_block3`, the manifest and
  all glTFs exist and are newer than the detail file, the vehicle config and the generator scripts. Result cached per run.
  The prototype switch is a named constant (`PROTOTYPE_VEHICLE_ID`), like Pad B's.
- The snapshot's stage dictionaries carry `vehicle_id` (bindings, `body_stages`): stage ids repeat in the tanker and HLS-launch
  configs (stages taken over), so the id alone cannot tell them apart; a separated part reports `<vehicle>/<stage>` (the part
  before `/` counts). No `vehicle_id` (older snapshot) = cylinders.
- Per stage one node `stage_<id>` (origin at the stage's base station, the body -> render permutation as basis) with the hull
  nodes (`hull`, `hot_stage`, ...), one MultiMesh per engine kind (`engines_raptor`, `engines_rvac`, one instance per snapshot
  engine, posed from `position_m` / `direction` = the real gimbal), `flap_<i>`, `grid_fin_<i>` as children in stage-local body
  coordinates. Stage separation rebuilds on the new stage list: a model never stays on the wrong stage. Material: the view's
  stainless steel `LitAerial`.
- Flap pose: hinge at (aft station, hull radius at the hinge azimuth), basis (X, e_r, stow) with stow = -rotation_sign * (X x e_r)
  (mirrored when the manifest sign is +1), then rotation about +X by sign * deflection; manifest entry matched by stage id,
  hinge azimuth and aft station. Fin pose: pivot at (station, hull radius, azimuth), rotation about +X by the azimuth, then about
  the radial Y by +deflection. Both are tested against re-implementations of `flap_span_direction` and the grid-fin normal.
- Plumes start at the model's exit plane (manifest: Raptor 2.1 m, RVac 3.6 m below the gimbal point); cylinders keep 3.0 m.

## Hull shader (`game/shaders/vehicle_hull.gdshader` + `heat_shield_tiles.gdshaderinc`)
- One opaque spatial shader (LitAerial's atmosphere fog, Godot PBR lighting: the clouds draw behind it, nothing goes to the
  transparent pass), three materials made by `VehicleModels` from the manifest's `look` block (ring pitch, weld bead size, tile
  size / gap, coverage table, belly azimuth, barrel end): mode 0 bare steel (booster, chine, raceway, fins, flap edges), 1 ship hull,
  2 flap (tiles on the outer +Y face). Coordinates are the node's LOCAL body frame, no baked texture, nothing view dependent: both
  eyes of a stereo frame evaluate the same surface function.
- Steel: metallic 0.8, roughness 0.27 [E] (a diffuse share stands in for the sky's weak reflection), per-ring tint / roughness from a
  hash of the ring index, darker rougher heat-affected band either side of each weld, oil-canning dents as the gradient of two noise
  octaves tilting the normal (fade with the pixel footprint).
- Tiles: axial hex cells (0.30 m across flats, 6 mm gap, config values) of (arc length, station) with the arc taken at the local
  radius; coverage = half angle around the belly azimuth from the detail file's table (cut by the smooth boundary like the chine
  strake); gap mask, bevel normal, per-tile shade hash, 0.6 % lighter "repaired" tiles. The weld beads' slope is taken out of the
  normal under the shield (barrel: radial; ogive: the profile slope minus the bead). Anti-aliasing: the pixel footprint (derivatives
  of the local position) box-filters the gap and edge, the per-tile detail fades to the mean colour between 0.12 and 0.35 tile widths
  per pixel, the bevel (a highlight, no mean) fades out at 1-3 bevel widths. Flaps: planar (span, hinge) coordinates.
- Known limit: s = angle x local radius shears the lattice a little on the ogive; real tiles are cut there anyway.

## Standing decisions
- Detail in geometry only where it reads (beads, vents, lattice, strakes); the tiles are the shader above. Engines keep the
  view's LitAerial steel. Staying data-driven: nothing is typed into a generator that the configs carry.
- The deterministic check compares positions, UVs and indices byte-for-byte; Blender's smooth normals differ in the 7th digit
  between runs, so the `.bin` files as a whole are not byte-identical.

## Validation
- `python -m pytest tools/structures/tests/test_vehicle_models.py -q` (34 tests with the Pad B ones; Blender cases skip
  without Blender): hull bounds / diameter / nose / UV / outward winding, pins, fin fairings, chine azimuth, flap and fin
  hinge at the origin and sizes, engine exit diameter, manifest, determinism, up-to-date skip, spec validation, flap sign.
- `godot --headless --path game --script res://tests/vehicle_view_test.gd` (synthetic snapshots from the vehicle config): node
  sets of other vehicle / no models / no `vehicle_id`, 33 + 6 engines and their ring-formula poses, gimbal follow, flap poses at
  0 / 35 / 90 deg and fin poses at 0 / -20 / 30 deg against the core's definitions, separation, plume exit planes, short lists.
  (The headless renderer keeps no MultiMesh transforms: the view keeps the last engine poses for the test.)
- Preview: `blender -b --factory-startup -P tools/structures/blender/preview_vehicle.py -- --spec <json> --out
  screenshots/vehicles` (stack, ship top / base, booster base / top, fin close-up; `--flap-deg`, `--fin-deg`).
- Triangles: booster 21,252, ship 21,020, flap 84 each, grid fin 924, Raptor 2,048, RVac 2,560 (33 + 6 engines are instances).

- `godot --path game --script res://tests/tile_shimmer_check.gd` (GPU): ship return entry state at 60 km (vacuum, belly lit),
  5 to 500 m, 1500 px focal, fixed exposure: mean luminance of a 1 m patch max / min 1.065 (limit 1.10), flicker under 0.3 px
  head motion <= 0.0013 (sigma / mean). The first version (bevel 0.7, fading with the tile detail) failed at 1.111 with flicker 0.0075:
  the bevel highlights were lost in the fade.
- Cost (`vehicle_cost_check.gd`, RTX 4090, 1600x900 window, stack on the pad, vehicle view shown / hidden, 3 repeats of 120-frame
  medians, scale 1 and 2 per axis): GPU render time at 100 m 1.185 / 1.184 ms (scale 1) and 2.177 / 2.143 ms (scale 2); at 400 m
  1.243 / 1.211 and 2.216 / 2.181 ms: +0.00 to +0.04 ms, the added draw calls 17-24 and 59-127 k primitives. Close-up (10 m from the
  ship's belly, hull filling the frame): scale 2 1.848 / 2.034 ms, i.e. 0.19 ms LESS with the stack (the hull hides sky / ground pixels);
  at scale 1 the three repeats scatter (0.63 / 0.51, 0.64 / 0.76, 0.94 / 1.10 ms): no sign can be read from them. Main-thread
  process time is the 60 Hz sync (16.9 ms) with and without. Within the plan's 1.0 ms [E] on this machine; not measured in VR.
- Evidence tools: `vehicle_look_shots.gd` (real mission, orbit camera relative to a body, `--stereo=` for eye images, `--scan`
  for flap / fin deflections over time), `vehicle_cost_check.gd`, `vehicle_rig.gd` (the common mission / camera rig).

## Open points
- Free camera (`FOCUS_FREE`) and a low camera looking up at a hull's shade side: that side is black (no ambient) with the old LitAerial
  steel and the tower too; the orbit camera from above / level shows the sky's reflection. Not caused by the hull shader; unexplained.
- Catch pins: the config has booster pins at 45 / 225 deg between the fins at station 66.5 m [E]; web reports for V3 put the
  catch features at / integrated with the grid-fin mechanisms (unconfirmed, partly older designs). Modelled as in the config
  (physics and visual agree); a check against Block 3 photos and a decision on the physics numbers are open.
- Config numbers that look odd next to the references: the 13.5 m^2 fin at a 3.0 m span (radius_m 6.0) gives a 4.5 m chord;
  the 20 outer engines on the 4.1 m ring overlap by 0.16 m at the exit planes and the outer bell edge (4.82 m) passes the hull
  radius (4.5 m).
- Flaps at rest lie on the hull's tangent plane (stowed, flat): the tip stands up to 0.9 m off the curved hull.
- Raceway position, tile coverage, hot-stage and bell dimensions are estimates; no Block 3 drawing exists.
