# Vehicle detail models (Super Heavy Block 3 + Starship Block 3, prototype)

State: plan `docs/superpowers/plans/2026-10-31-starship-booster-prototype.md`, task 1 done (models generated); the game side
(task 2) and the tile shader (task 3) are open. Only `starship_block3`; depot, tanker, HLS and Falcon 9 keep today's view.

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

## Standing decisions
- Detail in geometry only where it reads (beads, vents, lattice, strakes); tiles are the task 3 shader (UV and coverage table
  are ready). Staying data-driven: nothing is typed into a generator that the configs carry.
- The deterministic check compares positions, UVs and indices byte-for-byte; Blender's smooth normals differ in the 7th digit
  between runs, so the `.bin` files as a whole are not byte-identical.

## Validation
- `python -m pytest tools/structures/tests/test_vehicle_models.py -q` (34 tests with the Pad B ones; Blender cases skip
  without Blender): hull bounds / diameter / nose / UV / outward winding, pins, fin fairings, chine azimuth, flap and fin
  hinge at the origin and sizes, engine exit diameter, manifest, determinism, up-to-date skip, spec validation, flap sign.
- Preview: `blender -b --factory-startup -P tools/structures/blender/preview_vehicle.py -- --spec <json> --out
  screenshots/vehicles` (stack, ship top / base, booster base / top, fin close-up; `--flap-deg`, `--fin-deg`).
- Triangles: booster 21,252, ship 21,020, flap 84 each, grid fin 924, Raptor 2,048, RVac 2,560 (33 + 6 engines are instances).

## Open points
- Catch pins: the config has booster pins at 45 / 225 deg between the fins at station 66.5 m [E]; web reports for V3 put the
  catch features at / integrated with the grid-fin mechanisms (unconfirmed, partly older designs). Modelled as in the config
  (physics and visual agree); a check against Block 3 photos and a decision on the physics numbers are open.
- Config numbers that look odd next to the references: the 13.5 m^2 fin at a 3.0 m span (radius_m 6.0) gives a 4.5 m chord;
  the 20 outer engines on the 4.1 m ring overlap by 0.16 m at the exit planes and the outer bell edge (4.82 m) passes the hull
  radius (4.5 m).
- Flaps at rest lie on the hull's tangent plane (stowed, flat): the tip stands up to 0.9 m off the curved hull.
- Raceway position, tile coverage, hot-stage and bell dimensions are estimates; no Block 3 drawing exists.
