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
- `booster.gltf`, `ship.gltf`: nodes `hull` (a PLAIN lathe: rings only at the section ends - base, chamfer end, barrel end - and on the nose, 128
  segments around (sagitta 1.4 mm at R = 4.5 m); the weld seams and dents are the baked steel map, not mesh; radius = config
  radius; ship: tangent ogive of the drawn length 13.4 m (`nose_length_m` the same since Task 5b) with a blunt tip; booster: flat top), `hot_stage`
  (collar), `top_truss` (20-tooth V truss), `skirt` (rings, posts, ports), booster `raceway`, `fin_fairings`, `catch_pins` at the config stations / azimuths / protrusion, ship `chine` (swept strake
  along the tile boundary, azimuth = belly +- the coverage half angle of the tile table), `raceway` and `ports`. Hull UV = (arc metres
  at the barrel radius, station metres): glTF u = arc, v = station (Blender v = 1 - station).
- `flap_fwd.gltf`, `flap_aft.gltf`: hinge line = X axis from the aft end (origin), panel along +Z at rest (stowed, leeward),
  outer face +Y. The game's basis (X the drawn hinge, Y its normal in the meridian plane, Z stow direction, mirrored when
  the stow points to the belly side) and the rotation about the hinge X by `rotation_sign_about_plus_x` x deflection are derived from `core/src/aero/flap.cpp` and tested
  Planform = the 1:100 model's (swept, wedge section, hinge lugs, `[flap.aft|fwd]`). Aft flap: drawn hinge edge centred on the physics
  hinge, on the barrel (hull radius, no lean). Forward flap: size, orientation and place from the STL's flat flap face (user 2026-11-01), typed
  into `[flap.fwd]` (`hinge_station_aft_m` 42.282, `hinge_radius_aft_m` 4.621, `hinge_lean_deg` 19.03 towards the axis going forward, hinge
  5.784 m, 6-point outline, area 12.37 m^2; azimuth stays the physics' 20 / 160); `vehicle_spec.drawn_hinge` reads them (manifest
  `drawn_hinge_station_aft_m / _station_fwd_m / _lean_deg / _radius_aft_m`). d = 90 reproduces the STL pose in the meridian plane (check:
  `tools/structures/measure/gltf_flap_measure.py`). Hinge tube, lugs and the hinge cover are unchanged in construction; the forward cover spans
  the drawn hinge +- `overhang_m` (41.48-48.55 m). The straight hinge is up to 14 mm inside the drawn convex ogive at mid length (t = 0.75) and
  0.03-0.39 m above it elsewhere (cover height 0.5 m). It does NOT keep the physics area (see "Physics deviations for the user").
- `grid_fin.gltf`: pivot on the hull at the hinge, hinge axis +Y (radial). The plate lies across the station axis and its cells open along X: at deflection 0 an axial flow passes through them (the core's "normal" in `grid_fin.cpp` is the lift normal, the tangent at d = 0; a right-handed rotation by +d about the radial axis turns it toward +X). Plan geometry from the 1:100 plain fin STL (irregular octagon, exactly mirror-symmetric about the hinge axis, square cells at 45 deg to it, a wall crossing on the axis, cross rib corner to corner 1.75 m from the root), walls cut into cusped saddles (scooped on the AFT face only, 0.30 of the 0.50 m depth, tips slightly rounded; forward face flat), a closed outer frame wall (0.07 m thick, black like the lattice, full depth) round all 8 outline edges with soft arcs on its aft edge (one per wall end, 0.22 m deep), thin light band only on the two root tapers and the root chamfer, a fan of two full-depth struts from the ring to the rib with 45 deg cells in the whole root triangle (no tray, no hole). Nodes `frame` (closed wall round the whole outline), `band`, `lattice`, `pylon` (bolted ring, housing), `bearing` (flat bar under the root, only on `catch_fin_azimuths_deg`), `plate` (far stand-in), `shaft`. 4,060 triangles. The booster carries no catch-pin geometry any more (config kept). See "Fin plan round".
  toward +X: a right-handed rotation by +d about the radial axis, zero = rest. Drawn size = the STL's outline (see "Fin plan round"), not the config area / span. Egg-crate lattice, cells open along the tangent.
- `raptor.gltf`, `rvac.gltf`: origin = gimbal point, thrust +X, bell opens toward -X, exit plane at X = -`bell_length_m`
  (manifest `exit_plane_x_m`: the plume re-anchor of task 2), exit rim diameter = config value (tested), bare bell +
  housing above the gimbal.

## Game side (`game/vehicle/vehicle_models.gd`, `vehicle_view.gd`)
- `VehicleModels.load_for(vehicle_id)` -> `{}` (cylinders, one log line) unless the vehicle is in `MODEL_VEHICLE_IDS`
  (`starship_block3`, `falcon9_block5`, `crew_dragon`, `iss`), the manifest and all glTFs of its `MODEL_FILES` entry exist and
  are newer than the detail file, the vehicle config and its `SOURCE_SCRIPTS` / `SOURCE_BAKERS`. Result cached per run. The
  Crew Dragon and ISS have empty file lists until their generators exist (plan 2026-11-02): a fresh manifest
  without files, a missing or a stale one all give `{}` + one log line + cylinders (reason "missing" / "stale": the German
  notice is shown). Tests count lines (`VehicleModels.log_count`) and run on a scratch repo folder (`root_override`).
- Pipeline for several vehicles: `tools/structures/blender/generators.toml` lists per vehicle the `kind` (`starship`: the
  full spec of `vehicle_spec.load_starship`; `generic`: vehicle config + detail file as they are), the hashed helper
  `scripts`, texture `bakers` and the Blender `generators`. `build_models.py --vehicle <id>` takes everything from it; a
  vehicle with no generators prints one line, writes nothing, exits 0. The Starship's manifest hash and output are
  byte-identical to before the table (test `test_starship_hash_is_the_one_of_the_fixed_generator_list`, forced rebuild).
  The `iss` detail file is a stub (comments only) until its task.
- The snapshot's stage dictionaries carry `vehicle_id` (bindings, `body_stages`): stage ids repeat in the tanker and HLS-launch
  configs (stages taken over), so the id alone cannot tell them apart; a separated part reports `<vehicle>/<stage>` (the part
  before `/` counts). No `vehicle_id` (older snapshot) = cylinders.
- Per stage one node `stage_<id>` (origin at the stage's base station, the body -> render permutation as basis) with the hull
  nodes (`hull`, `hot_stage`, ...), one MultiMesh per engine kind (`engines_raptor`, `engines_rvac`, one instance per snapshot
  engine, posed from `position_m` / `direction` = the real gimbal), `flap_<i>`, `grid_fin_<i>` as children in stage-local body
  coordinates. Stage separation rebuilds on the new stage list: a model never stays on the wrong stage (a hidden view still
  rebuilds, it only skips posing and plumes). Materials: the hull shader's three (below); engines keep the view's `LitAerial` steel.
- Flap pose: hinge at the manifest's drawn (aft station, radius), equal to the physics hinge, at the hinge azimuth, basis (X' = cos l X - sin l e_r,
  Y' = sin l X + cos l e_r, stow) with l the manifest lean and stow = -rotation_sign * (X x e_r) (mirrored when the manifest
  sign is +1), then rotation about X' by sign * deflection (d = 0 near the hull tangent, d = 90 in the meridian plane, span
  perpendicular to the hinge); manifest entry matched by stage id,
  hinge azimuth and aft station. Fin pose: pivot at (station, hull radius, azimuth), rotation about +X by the azimuth, then about
  the radial Y by +deflection. Both are tested against re-implementations of `flap_span_direction` and the grid-fin normal.
- Plumes start at the DRAWN exit plane (manifest: Raptor 2.1 m, RVac 3.6 m below the drawn gimbal point; ship engines drawn higher, see "Stage interface"); cylinders keep 3.0 m.

## Falcon 9 first stage (plan 2026-11-02, Task 3)
- `generators.toml` kind `falcon9` (`vehicle_spec.load_falcon9`, `validate_falcon9_detail`), key `outputs`: `falcon9_stage1.gltf` (nodes `hull`, `interstage`,
  `octaweb`), `falcon9_grid_fin.gltf` (`frame`, `lattice`, `plate`, `pylon`; same frame as the Starship's fin), `falcon9_leg.gltf` (`leg_main`, `leg_pusher`),
  `merlin.gltf` (`gen_engine.py`). Triangles 992 / 368 / 156 / 2,048. Plain paint so far (`look.paint`, `part_materials`, `no_maps`): no baked map of panel lines or soot yet.
- Leg rig `VehicleModels.leg_poses(rig, azimuth, deployment)`: a function of the snapshot's deployment only. A rigid leg turns about the hinge (M4: station 2.2, r 1.46) from the stowed
  angle to the angle that puts its tip on the config's foot; the leg's length (8.68 m) follows from hinge and foot. The pusher runs from the hull ring to 5.5 m along the leg and is scaled along
  its axis (1.4 to 8 m): the real part telescopes, ours stretches. Manifest `leg`, `leg_foot_deployed_m`.
- Fins fold by a view rule: `fin_stowed_transform` while another stage stands on the fin's stage. The physics has no fold; on separation the fin snaps out.
- Engines: manifest `look.engine_ids`, `axial_offset_m_by_stage`; the Merlin Vacuum has its own (`merlin_vacuum`, `vacuum` id).
- The manifest's age is judged against the physics config: after any edit of `falcon9_block5.toml`, even a comment, the set counts as
  stale (cylinders and the notice) until `python tools/structures/build_models.py --vehicle falcon9_block5` has run.
- Checks: `tools/structures/tests/test_falcon9_spec.py`, `vehicle_view_test.gd` `_check_falcon9` (needs the generated set). Shots: `vehicle_look_shots.gd --legs=<d>`.

### Second stage and stack (Task 4)
- `falcon9_second_stage.py` writes `falcon9_stage2.gltf` (`hull`: plain white lathe from the skirt to the flat top plane at the config length, 9.8 m, where the payload's trunk base stands (tested); RCS bumps at the config's pods; `engine_plate`: the dark cone from station 0 to the engine mount) and `mvac.gltf`
  (root `mvac`: `bell`, `housing`, `duct`). Triangles 536 / 3,840. Stage `second_stage` has its own part material (`stage_part_materials`).
- Station 0 of the second stage = the interstage top. The Merlin Vacuum's gimbal point is the physics' engine station -0.6 m (drawn there, offset 0). The nozzle hangs 3.4 m below it
  (manifest `exit_plane_x_m` -3.4, the plume starts there with the rim's radius): exit station 43.7 m of the stack, inside the interstage
  (41.2 .. 47.7 m), tested; nozzle exit to top 13.8 m, the published length. Measurement and consumers: `falcon9.md` "Stack". The adapter under the trunk is white (`D_SpaceX_Crew_9_Preflight`), no dark ring: the dark band in Crew-13 t2880 was the trunk's cells.
- Nozzle: exit diameter 3.3 m = config = Wikipedia infobox; gimbal-to-exit 3.4 m [E] (extension 2.7 m [S, the original nozzle; a shorter
  one flies now, no published length] + 0.7 m regenerative bell; photo `F_Merlin_nozzle`: cone height 1.08 x the exit diameter = 3.6 m +-0.4 m,
  perspective); top radius 0.69 m and contour exponent 0.84 from the same photo. Not drawn: thermal blankets, the turbine exhaust's real
  routing (one straight duct), the corrugated wall seams; the real stage's skirt shape at the engine section is not measured (plain).
- Separation: `<vehicle>/<stage>` ids keep both parts' models (tested, stack, first stage alone, second stage alone).

### Measuring references and provenance (plan 2026-11-02)
None ships; numbers were read by script (`tools/structures/measure/`, `reference_models/`) or by eye and drawn by us. Files live in the git-ignored `screenshots/falcon9_ref/`; credits in `tools/data_pipeline/sources.toml` / `CREDITS.md`. Real photos win over models.
- M1 "Falcon 9 - Crew Dragon", KUBAHA, CC BY 4.0: stack on one scale; stage split not usable (first stage ~6 m short), rejected for capsule shape, engine ring.
- M2 "SpaceX - Dragon 2", KUBAHA, CC BY 4.0: scaled x10 to the 4.0 m capsule; capsule 4.85 m, total 8.08 m, trunk 3.234 m, fins full trunk height with tip r 2.64, nose hinge r 1.108 at x 3.69, seam plane, Draco port groups at 36 / 144, bays at 63 / 117.
- M3 "SpaceX Dragon 2 exterior", M4 "SpaceX Falcon 9 and Dragon 2", Forest Katsch, CC BY 4.0, one Dragon, not independent. M4 scale 1.000: tank + bay 41.4 m, interstage 7.0 m (+-0.5), first stage 48.4 m (not averaged in: 47.7 m published), foot -2.1 m, foot radius 9.2 m, engine ring 1.24 m at 22.5 + 45 k deg.
- NASA ISS model D (IGOAL), NASA 3D Resources: station tables, IDA ports, clocking class. IDSS IDD Rev. E (NTRS 20170001546): docking dimensions; flange OD not dimensioned.
- Photos (NASA public domain; SpaceX CC0; `F_Merlin_nozzle` CC BY 2.0 Steve Jurvetson): `F_NHQ202210010026`, `D_SpaceX_Crew_9_Preflight`, `F_KSC_20200429`, `F_ORBCOMM_2`, `F_Falcon_9_first_stage_in_hangar`, `F_Landed_rockets_in_hangar_39A`, `F_Falcon_9_B1046`, `D_CCP_SpaceX_Demo_2_Dragon_(3)`, `D_Crew_2_..._Falcon_9_mate`, `D_Iss065e002827`, `D_Iss071e186365`, `D_Iss072e096331`. Webcast frames (`crew13_*`, `starlink15-25_*`, `crs35_*`) are for private reference only.
- Models disagree on grid fin station and azimuth, leg azimuth and the capsule's diameter; the photos decided (`falcon9.md`, `crew-dragon.md`).

## Crew Dragon (plan 2026-11-02, Task 5)
- `generators.toml` kind `dragon` (`vehicle_spec.load_dragon`), generator `gen_dragon.py`. Our own rebuild from typed tables in `config/vehicles/detail/crew_dragon.toml`, measured on the KUBAHA "Dragon 2" model scaled to 4.0 m (ray casts, scratch scripts), photos `D_*` win where they disagree; no third-party vertex data in the repo.
- `dragon_capsule.gltf` (station 0 = shield apex): `heat_shield`, `band`, `capsule` (skin r(x, az): `profile_m` of revolution + four cowl lobes `hood_outer_m`, ending on the tilted seam), `superdraco_pod` (bay floor `bay_floor_m`, walls, lip; columns follow the bay's edge `bay_half_width_deg_m`), `superdraco_nozzle` (outlines only, as the photos show them), `draco_pod` (leaning port slots at az +-36, +-144), `window`, `hatch` (panel, flanges), `seams`, `hinge_fairing`, `deck` (seat on the seam, conical white skirt to the flange, port holes and Draco slot in `seams`: photo D_Iss071e186365 over M2's flat deck), `recess` (floor, well, hatch end). Triangles 25,606.
- `dragon_nose.gltf`: `nose_cone` (skin above the seam, lip, inside, inner cone with the port recess) and `nose_seal` (seal ring, hinge arm), ORIGIN = the hinge point, closed = identity. Triangles 5,763. The seam is a plane, x = 4.0 + 0.26 Y (fit residual 1.4 mm): 4.2 m on the hatch side, 3.71 m at the hinge.
- `dragon_trunk.gltf` (station 0 = trunk base; tables count from the shield apex, the generator adds the stage length): `trunk` (shell, top ring 0.34 m above the stage length, floor, four full-height fins `fin_outline_m` at 0 / 90 / 180 / 270), `trunk_array` (cells on the hinge half az 90-270, dark base, ports), `trunk_radiator` (hatch half), `umbilical` (the fairing that leaves with the trunk). Triangles 908.
- Derived from the physics config, not typed twice: the bays sit at the middle of each consecutive pair of the SuperDraco `angles_deg` (63 / 117 / 243 / 297), the throats at the pair's spread; the profile's last station equals the capsule `length_m` (tested). The physics Draco pods (config) are an [E] control layout and are NOT what `draco_pod` draws.
- Docking mechanism (Task 6): `idss_mechanism.py` -> `idss_mechanism.gltf`, drawn from the IDSS IDD Rev E numbers in `[idss]` of the detail file (figure and pdf page per line, [S] / [E]). `DockingMechanismView` (`game/vehicle/docking_mechanism_view.gd`), a child of the capsule stage at the port, created by `VehicleView` as `docking_mechanism`, retracted with open hooks: `set_state(ring_extension_m, ring_tilt_rad, hook_closed)` is the only entry for docking visuals (nothing calls it in flight yet). Nodes `flange`, `hcs_metal`, `hook_pockets` (fixed), `ring` with `petal_0..2`, `guide_pin_0..1`, `hook_0..11` (active hooks, on theta 15 + 30 k deg), `struts` with `strut_0..5` (each barrel, middle, rod, no scaling). Triangles 3,944.
  - Port: `position_m` 4.22 m = the HCS mating plane = flange top (`neck_m[0]`), tested (`core/tests/test_idss_view_values.cpp`). Was 4.0, the recess floor region, up to 0.44 m under the drawn cone. Evidence: M2 neck top 4.22, cone top 4.44 = + the 0.2 m petal; side view D_Iss072e096331 (143 px/m from the 4.0 m shoulder): white lip ends 4.24 m (+-0.15), 4.10 m from the seam line; fan drawing 4.07-4.18. The beige blanket with the gold handrails below it in the photos is read as the IDA, not Dragon (cables, handrails; a Dragon neck that long would not fit under the nose): that reading is mine, no document states it.
  - Physics copies: ring extension 0.30 [E], tilt range 6 deg [E], counts, petal height and base are copies of the C++ defaults (no config path there), compared by the same test. Extension semantics: ring face above the HCS plane; at 0 the drawing keeps the IDD's 6 mm recess, blended out towards full extension.
  - Flange OD recheck (Task 7, `D_Iss072e096331` full resolution 3840 px, shoulder 4.0 m = about 275 px/m, hand read, blurred edge): the white lip under the open nose spans 1.6-1.8 m (review 1.64, own read about 1.7), the beige port body below it about 1.57 m. That lip is the white skirt's lower edge of the deck, not the metal HCS flange (hidden behind the skirt in this view); the head-on photo `D_Iss065e002827` is too small (the Dragon is only ~300 px wide at full resolution) to read a flange diameter. The 1.40 m therefore stays: the side photo does not measure the flange. Open instead: the skirt's end radius (0.70 drawn, 0.80-0.90 read) and the nose depth belong together (a skirt end at r 0.85 does not fit under the drawn nose, shoulder r 0.72, seal r 0.75); needs a redraw of the nose and skirt, not a number change.
  - Open dimensions [E]: flange outer diameter 1.40 m (the IDD leaves it open), capture ring section (figure not to scale), the six actuators (the IDD defines the interface, not the actuators; count and layout are ours; tube lengths tested against the stroke), separator angles other than the dimensioned ones (+-3 deg). Hook turn 27.0 deg [D]: the IDD gives no stroke; it follows from the leading edge stations 37 and 52.5 mm of Figs 3.2.3.4-1, -2 about the pivot of Fig 3.2.3.4-4, and reproduces the 17.5-17.8 mm tip heights (test).
  - The physics' `petal_lean_rad` 30 deg [E] differs from the IDD's 45 deg face, which is drawn. Not changed (capture radius is physics): open point. Hook's circumferential sense from the figure's view side: assumed.
  - Checks: `game/tests/docking_mechanism_test.gd` (headless), demo `game/tests/docking_mechanism_demo.gd -- --out_dir=<dir>` (GPU, six phases, four views; the passive port is a second mechanism). Same-camera comparison with M2: `screenshots/dragon/idss/cmp_*.png` (M2's mechanism is a simplified cone with no hooks, ring or pins: the IDD wins).
- Check: `python tools/structures/measure/dragon_profile.py` compares the generated glTF with the typed numbers (largest deviation 1.05 cm, chord near the nose tip); `tools/structures/tests/test_dragon_profile.py` pins it per table.
- Nose: hinge at x 3.69 m, r 1.108 m, azimuth 180 (opposite the hatch), tangential axis; open angle 120 deg [E: the reference's animation, photos D_Iss071e186365 / D_Iss072e096331 agree to about +-15 deg], opening time 8 s [E, view only]. `VehicleModels.nose_transform(nose, fraction)`.
  `VehicleView.sync(..., capsule)` takes `FlightSim.capsule_state` (`nose_cone_open`; `present` false on a launcher = closed): the view eases its fraction towards the binary
  state at 1 / open_time_s per second (`_process` / `advance_nose`), a Dragon first seen shows the state as it is. `VehicleView.look_nose_fraction` / `vehicle_look_shots.gd --nose` for shots.
- Stack and payload ids: the vehicle id of a stack is `launcher+payload` ("falcon9_block5+crew_dragon"), a separated part `launcher+payload/<stage>`. The view loads the model set
  of every `+` part and each stage draws from the set that holds its hull id (`_stage_models`): the Dragon on the Falcon 9 second stage is drawn, and a separated payload shows
  the Dragon, the second stage the Falcon 9 set. A set that is stale / missing leaves its stages as cylinders and the notice shows.
- Checks: `tools/structures/tests/test_dragon_spec.py`, `vehicle_view_test.gd` `_check_dragon`. Shots: `vehicle_look_shots.gd --mission=crew_dragon_return --t_plus=2400 --nose=<0|1>`
  (body 1 = capsule after the trunk jettison; `sun<deg>` turns the camera about the axis).
- Not drawn / unsure: hatch and window detail beyond flanges and panel (textures), the trunk's cell pattern (one dark paint on the hinge side, radiators on the hatch side), SuperDraco layout evidence (two models, one photo; moderate).
- The trunk's dark half is right: normals outward (`trunk_array` 108 of 108 side vertices), albedo [0.04, 0.05, 0.09] [E], and `D_Iss072e096331` and `D_SpaceX_Crew_9_Preflight` show the sunlit cells near black with a blue sheen. Against the Crew-9 photo: the real cells do reach the base, as drawn.
- Closed surfaces (`tools/structures/tests/test_dragon_closed.py`, edges welded to 0.1 mm across a file's parts): the trunk's shell, fins and base disc form one closed body; the fin roots and the umbilical's back and upper end were open and are now closed (the umbilical's stood free above the trunk's top; the fin roots lay on the hull and never showed). Edges that stay open on purpose: trunk panels (single sheets 12 mm proud of the closed hull); on the capsule the decals on its surface (hatch, windows, seams, Draco ports), the SuperDraco bells (open mouths in closed pods), and 120 / 115 edges where band and capsule meet at x 0.68 without shared vertices; the nose's seal strip; three edges of the nose cone at its hinge (x -0.03, r 0.1). Renders from behind, below, the fins' back sides and through the open port show no hole (`screenshots/final/trunk_rear.png`); a ray down the axis hits the recess floor at 3.74 m.
- The Falcon 9 leg's crush is not drawn: after a hard landing the feet sink up to 0.5 m into the deck in the view.
- The thin tan rod beside the docked Dragon is the station's Canadarm2 (`detail/iss.toml` `SSRMS_upper`, `SSRMS_lower`): its wrist ends free, as a real arm's would, without the end effector drawn. Not a fault of Dragon or the docking; ISS look stays as it is (user 2026-11-02).

## ISS (plan 2026-11-02, Task 8)
- `generators.toml` kind `iss` (`vehicle_spec.load_iss`): the ports come from the physics config, the IDSS numbers from Dragon's `[idss]` (one androgynous interface, one table), the drawing from `config/vehicles/detail/iss.toml` below the marker `---- the drawing (Task 8) ----` (typed tables: `[[lathe]]` per module, `[[box]]`, `[[plate]]`, `[[tube]]`, `[[bay]]` truss, `[[wing]]`, `[[irosa_plane]]`, `[[gamma]]`, `[ida]`, `[pose]`). Numbers read off NASA model D (radial slices, area-weighted; plane fits; rounded), no vertices copied. Stage frame of `iss.toml` as glTF frame.
- `iss_station.gltf` (`iss_station.py`): `pressurised` (Unity, Destiny, Harmony, Node 3, Cupola, Columbus, Kibo + logistics module, Quest, Bishop, PMM), `pressurised_ru` (Zvezda, Zarya, Nauka, Prichal, Rassvet, Poisk), `beam`, `pma_1`, `truss` (S0, Z1, P1 / S1, P3 / S3: longerons, rings, diagonals), `payloads` (ELC 1-4, ESP-2 / 3, AMS, JEM EF deck, Bartolomeo deck, MT, MBS: one box each), `wings_ru`, `arm` (Canadarm2 pose of the model, ERA as two tubes). `iss_rings.gltf` (`idss_passive_ring.py`): for n = 2, 3: `pma_n`, `ida_n`, `ida_n_band`, `ida_n_face`, `ida_n_floor`, `ida_n_hcs`, `ida_n_pockets`, `ida_n_hooks`, `ida_n_pins`, `ida_n_ring`, `ida_n_petals`, `ida_n_fittings`. `iss_arrays.gltf` (`iss_arrays.py`): relative to their joints: `array_<i>`, `mast_<i>`, `irosa_<i>`, `irosa_boom_<i>`, `truss_outboard_<k>`, `pv_radiator_<k>`, `radiator_<j>` (3 accordion wings).
- Triangles: `iss_station` 20,316, `iss_arrays` 3,016, `iss_rings` 14,172 (about 7,000 per port): **37,504** in all, plus Dragon docked (capsule 25,606, nose 5,763, trunk 908, mechanism 3,944 + instances). Cost: see "Cost of the Falcon 9, Dragon and ISS models" below.
- Joints: `VehicleView._build_joints` makes fixed nodes `alpha_joint_<k>`, `beta_joint_<i>` (under its alpha joint), `radiator_joint_<j>` from the manifest's `joints` (origin, axis, `pose_deg`). Stated pose [E]: alpha 0; beta -90 port / +90 starboard from the file's pose: blankets broadside (normal along z), span 108.4 m at this pose (tested against 109 m +-1), as in the station photos, iROSA on the zenith side. The simulation has no joint state: the pose never moves; the file's own pose is beta 0.
- Maps (`tools/structures/iss_textures.py`, baker, deterministic, no photo pixels): `array_cells.png` (legacy blanket bays), `irosa_cells.png`, `panel_skin.png` (module skin), `radiator_ribs.png`; albedo multiplied, UV in metres, period in the manifest `look.textured`, mips made by the game (box), shader `lit_aerial_mapped.gdshader` (lit_aerial + the map; both eyes sample with their own derivatives). No normal maps.
- Rings: the passive interface uses the Task 6 functions of `idss_mechanism.py` for seals, hooks, receptacles, separators and the capture ring; ring recessed by the IDD's 6 mm; passive hooks and active hooks both, the latter fixed open; guide pins; no actuators. `petal_frame` draws the IDD's 45 deg face (r 0.6 -> 0.4, 0.2 m) as a frame with lightening holes (rim, centre rib, cross member, two diagonals, 3 cm bars) as model D shows; the Dragon mechanism's `petal` is still a closed plate (Task 6). Face out to r 0.78 [D], chamfer to the barrel r 0.89 [D], barrel 0.64 m deep [D], PMA from Harmony to the ring axis with its 0.72 m offset ([D], at both ports the same table).
- Test (`vehicle_view_test.gd` `_check_iss`, ports read from the config text, geometry from the meshes): nodes, joints at their pose, span; at each port with Dragon mated (axes opposed, ups equal): flange plane and IDA face coincide (0.0000 m, limit 0.01), axes 0.0000 deg (limit 0.1), centres on the axis, passive petals at the IDD's angles about the config's `up` (worst 0.023 deg, limit 0.1), Dragon's petals exactly between them (60.000 deg), passive guide pins on Dragon's receptacles (0.2 mm, limit 2 mm). `tools/structures/tests/test_iss_spec.py`: ports equal the config, `up` keeps model D's petal angles within +-3 deg, tables complete, triangle cap.
- Comparison: `screenshots/iss/cmp_*.png` (ours | model D, same camera, clay colours, our Dragon on both; rasteriser in the scratchpad, not shipped), `screenshots/iss/game/g_*.png` (real session `crew_dragon_iss_departure`, station is body 1, `--t_plus=-500`: still docked).
- Not drawn / simple on purpose: outboard payload detail (ELC, ESP, JEM EF as boxes), SPDM, Strela, antennas except the two Z1 dishes, all handrails away from the IDAs, MLI wrinkles, Whipple bolts, hatches on Node ports, visiting vehicles (model D has none). Array masts are one bar, not the lattice. Nauka's three radiators `[E]` in y (read in the side view). Radiator accordion half height 0.55 m read in the view (the mesh's z range also holds the root); PV radiators flat, not 7 panels.

## Cost of the Falcon 9, Dragon and ISS models (plan 2026-11-02, Task 9)
- `vehicle_cost_check.gd --mission=... --t_plus=... --views="name,body,station_m,dx,dy,dz,fov;..."` (RTX 4090, 1600 x 900, scales 1 and 2, 3 repeats x 120-frame GPU medians, variants interleaved): `hull` (models as shipped), `cyl` (every vehicle as cylinders), `hidden`. The first run of the Task 9 session had a harness bug (`cyl` did not reload the view's sets, `_sets_signature`); the numbers below are after the fix. Raw logs: scratchpad `t9/cost_*.log`, values in the plan's `final-values.md`.
- Budget: the prototype's 1.0 ms desktop GPU per view [E], read as models vs cylinders. Falcon 9 on the pad (3 views, 7-42 k primitives more than hidden), in landing with legs (2 views): models vs cylinders -0.013 ... +0.021 ms at scale 1 and 2 (one landing run at scale 1, 45 m, was disturbed by terrain loading and is not used), +0.075 ms at scale 2 in the 17 m legs view. The frame is terrain-bound (4-7 ms, 2.9-3.4 M primitives).
- Dragon docked at the ISS (the worst view for the rings: IDA, passive ring and Dragon fill the frame, 59 k primitives more than hidden, 62 more draw calls): models vs cylinders +0.043 ms (1.4 MP), +0.335 ms (5.8 MP), +0.223 ms (15 MP window clamped to 2647 x 1421, scale 2); against nothing drawn +0.08 / +0.50 / +0.57 ms. Dragon 25 m view: +0.04 / +0.30 / +0.24 ms. Overview 40 m and the whole station at 200 m: no measurable cost (the models are cheaper than the terrain / Earth pixels they hide).
- Within budget on the desktop at every size tried. VR (5294 x 4250 per eye = 22.5 MP, 90 Hz) NOT measured: the window could not exceed the screen height, so the 15 MP run is the largest. Linear scaling of the ring close-up (+0.223 ms at 15 MP) gives about +0.33 ms per eye, +0.7 ms for both eyes against cylinders (+1.7 ms against nothing drawn), i.e. under 1.0 ms only if the cylinders count as the baseline; the desktop figure holds as a proxy for pixel-bound cost only approximately (the cost grew faster than the pixel count between 1.4 and 5.8 MP). The cost is the pixel shader work and overdraw of the rings, not the 59 k triangles; no geometry was reduced, no shader removed.

## Steel map (`tools/structures/hull_textures.py` -> `data/models/starship_block3/hull_steel.png`)
- Baked by the driver (numpy, no Blender), procedural from the detail file's `[hull]` numbers (ring pitch, weld height / half
  width, `dent_slope_rms`; no photo pixels), covered by the manifest's input hash (the script is hashed), deterministic (fixed
  lattice seed, fixed zlib level, test: byte-identical PNG). One RGBA8 PNG, 1024 x 512 texels, tiling: u = circumference / 6
  (4.71 m, 4.6 mm per texel, closes around the R = 4.5 m hull), v = the 1.8 m ring pitch (3.5 mm per texel) with the weld bead
  at v = 0 (a weld at every multiple of the pitch). Channels: R, G slope along u / v (bead: triangle 5 mm high, 4 cm wide (was 12 mm / 10 cm; fix round 2026-10-31: frames show hairlines <= 5 cm, upper bound, not resolved); heat band 1.2 x the bead;
  dents: two noise octaves, ~1.2 m and ~0.38 m, rms slope 0.012), B slope energy sx^2 + sy^2, A heat-affected band coverage + the dotted stringer lines (12 per tile = 0.39 m apart, 60 % of the lines carry dots (seeded choice, no regular grid), pitch 0.07-0.11 m per line, 20 % of the dots missing; rings shift by whole stringers). Shader: weld / dot darkening 0.25, roughness gain 0.08 (was 0.22: the dots glinted white, frames show them dark). `flap_lee.png`: panel lines of the flaps' lee face, same channels, not tiling, no ring shift.
  The manifest's `look.steel_map` holds file, periods, ranges.
- VRAM: 2.0 MiB + mips = 2.7 MiB (RGBA8, the full chain built by the game with `Image.generate_mipmaps`: box averages, which is
  what the channels are designed for); the file is 180 kB. Sampled with trilinear + anisotropic filtering (project setting
  `textures/default_filters/anisotropic_filtering_level` = 3, 8x; it holds for every texture of the project).
- Roughness carries the lost normal variance: on a mip the shader takes `B - |mean slope|^2` (zero on mip 0, the slope variance
  the averaging removed) and widens the GGX alpha (`alpha_eff^2 = alpha^2 + variance`, Godot's roughness = sqrt(alpha)), so
  the glints do not shimmer. Missing file / not the hull: `steel_textured` false, plain steel, one log line.

## Hull shader (`game/shaders/vehicle_hull.gdshader` + `hull_noise.gdshaderinc`)
- One opaque spatial shader (LitAerial's atmosphere fog, Godot PBR lighting: the clouds draw behind it, nothing goes to the
  transparent pass), four materials made by `VehicleModels` from the manifest's `look` block (ring pitch, first weld station, steel
  map, tile size / gap, coverage table, belly azimuth, barrel end): `steel` plain (chine, raceway, fins, hot stage, flap edges),
  `booster` mode 0 with the steel map, `ship` mode 1 with the steel map (tiles on the windward side), `flap` mode 2 (tiles on the
  outer +Y face). The steel map is addressed by the hull's UV, the tiles by the LOCAL body position; nothing is view dependent:
  both eyes of a stereo frame sample the same maps with their own derivatives.
- Steel: a metal lit by the sun's lobe and the sky's radiance cube: F0 linear 0.56 / 0.57 / 0.58 [S: Filament guide, Iron], metallic 1.0, roughness
  0.35 [E], large-scale mottle +-3 % [E], per-ring tint / roughness from a
  hash of the ring index (the only analytic steel detail left: it fades to its mean from 0.06 m, panel_fade), the steel map's weld
  bead + dents as a tangent-space normal (frame: circumferential tangent, cross(n, tangent), the mesh normal, so it also follows the
  ogive), its heat-affected band darkening the albedo (0.25) and roughening (0.22) [E], a per-ring shift of u so every ring panel has
  its own dents (derivatives from the unshifted UV: no mip seam). No seams below `look.weld_first_m` (the chamfer). The seams
  continue over the ogive at the 1.8 m station pitch. The old footprint fades of the dents / bead slope and the profile-radius
  workaround are gone (the tile arc uses the local radius; the mesh is a plain lathe).
- Tiles: baked maps (`tools/structures/hull_tiles.py` -> `tile_patch.png`, `tile_sel.png`, `tile_ids.png`, manifest `look.tile_maps`;
  procedural, deterministic, hashed with the generator, git-ignored with the other models). The shader still decides WHERE tiles are
  (coverage = half angle around the belly azimuth from the detail file's table, cut by the smooth boundary like the chine strake;
  flaps: the outer +Y face; planar / arc-at-local-radius coordinates (arc, station) in metres; under the shield the normal is the
  barrel's radial or the ogive's profile slope); the maps decide how they look:
  - `tile_patch` 256 x 512 RGBA8, one lattice period (0.30 x 0.52 m, two hexagon tiles; 1.17 x 1.01 mm per texel: finer than a VR
    pixel at 5 m, 1.7 mm, so the nearest view is minified, not blurred; the 6 mm gap is 5 texels): R, G bevel tilt of the normal
    (0.4 at the edge, 12 mm wide), B tilt energy, A gap-filler coverage (4 x 4 supersampled bake). 0.5 MiB + mips = 0.67 MiB. The game
    builds the mip chain (`Image.generate_mipmaps`, box averages, which the channels are designed for) and samples it with trilinear
    + 8x anisotropic filtering (`textureGrad`, derivatives of the surface coordinate, outside the branch). Lost tilt energy
    (B - |mean tilt|^2) widens the GGX alpha like the steel's: no gap / bevel fade, no hash, no analytic lattice left.
  - Per-tile variation: `tile_sel` (128 x 256, nearest, no mips: which of the period's two tiles, the second lattice's tiles
    belong to four period cells, so also the id cell offset) and `tile_ids` (128 x 64 period cells = 38 x 33 m before the shade
    pattern repeats, nearest: shade 0.75 .. 1.25 per tile, 0.6 % lighter repaired tiles) = 98 kB + 32 kB. Ids are fetched at mip 0
    only, so mips cannot bleed one tile's id into the next (the id of a texel is exact; a footprint wider than a tile just samples
    some tile's id). Ids cannot be averaged, so this one fade remains: the shade / repaired variation fades to the mean (shade 1,
    none repaired) between 0.12 and 0.35 tile widths per pixel, as before.
  - Colour [E]: tile 0.030 / 0.031 / 0.034 (linear, near-black coated ceramic), gap filler 0.055 / 0.052 / 0.050, roughness 0.55 /
    0.75, repaired 0.075. Missing maps: `tiles_textured` false, plain dark tile surface without gaps, one log line.
- Known limit: s = angle x local radius shears the lattice a little on the ogive; real tiles are cut there anyway.

## Standing decisions
- Detail in geometry only where the silhouette needs it (V truss, lattice, strakes, hot-stage collar, nose); fine structure (welds,
  dents) is texture, the tiles are baked maps too (user 2026-10-31: everything gets mipmapped textures; the shader keeps only where they are). Engines keep the
  view's LitAerial steel. Staying data-driven: nothing is typed into a generator that the configs carry.
- The deterministic check compares positions, UVs and indices byte-for-byte; Blender's smooth normals differ in the 7th digit
  between runs, so the `.bin` files as a whole are not byte-identical.

## Validation
- `python -m pytest tools/structures/tests/test_vehicle_models.py -q` (43 tests in the folder; Blender cases skip
  without Blender): hull bounds / diameter / nose / UV / outward winding / no weld rings in the barrel, steel map (deterministic,
  seamless, weld bead and band at v = 0, rms slope; no Blender needed), pins, fin fairings, chine azimuth, flap and fin
  hinge at the origin and sizes, engine exit diameter, manifest, determinism, up-to-date skip, spec validation, flap sign.
- `godot --headless --path game --script res://tests/vehicle_view_test.gd` (synthetic snapshots from the vehicle config): node
  sets of other vehicle / no models / no `vehicle_id`, 33 + 6 engines and their ring-formula poses, gimbal follow, flap poses at
  0 / 35 / 90 deg and fin poses at 0 / -20 / 30 deg against the core's definitions, separation, plume exit planes, short lists.
  (The headless renderer keeps no MultiMesh transforms: the view keeps the last engine poses for the test.)
- Preview: `blender -b --factory-startup -P tools/structures/blender/preview_vehicle.py -- --spec <json> --out
  screenshots/vehicles` (stack, ship top / base, booster base / top, fin close-up; `--flap-deg`, `--fin-deg`).
- Triangles: booster 6,528, ship 9,100 (the nose rings dominate); before the plain lathe 21,252 / 21,020 (72 segments, a bead ring every
  1.8 m). Flap 128 aft / 116 fwd, grid fin 552, Raptor 2,048, RVac 2,560 (33 + 6 engines are instances).

- `godot --path game --script res://tests/tile_shimmer_check.gd -- --surface=tiles|steel` (GPU, one surface per run): VR focal
  2900 px (FOV 17.6 deg on the 900 px window), fixed exposure, 5 to 500 m, a 2 m patch (> the ring pitch, so beads count), 8 frames of
  0.3 px head motion, each at scale 1 (A) and at scale 2 (B: the supersampled reference, `footprint_scale` 2 so it evaluates the SAME
  filtered pattern, only sampled more densely). Gated: per-pixel temporal flicker (mean |dL| between motion frames / L) of A <=
  1.5 x the reference's own + 0.004; per-pixel RMS of A - B over L <= 0.15 [E]; tiles: luminance ratio max / min <= 1.10. Tiles (baked maps, 2026-10-31): ratio
  1.059, temporal 0.0000-0.0076 (ref 0.0000-0.0075), error <= 0.044 (160 m; the analytic tiles: 1.081, error <= 0.124); steel (booster barrel, sunlit, now the mipmapped steel map):
  temporal <= 0.0007 (ref <= 0.0007), error <= 0.043 (500 m; it was 0.128 with the analytic weld / dent fades), mean 0.29 -> 0.31
  with distance; ratio 1.074 (not gated). Logs of this state: `hs_shimmer_tiles.log`, `hs_shimmer_steel.log`. Before the fix round the
  check averaged a 1 m patch (blind to moire) and gated only the ratio; with the per-pixel metrics the first shader failed (steel
  500 m error 0.131, tiles 320 m 0.375 / ratio 1.117; logs `.superpowers/sdd/2026-10-31-starship-prototype/shimmer_red_*.log`, the
  reference there still evaluated finer footprints). Logs before the steel map: `shimmer_final_tiles.log`, `shimmer_final_steel.log`.
- Cost with the plain lathe and the steel map (`hs_cost.log`; the older numbers below are the 21k-triangle meshes with analytic steel):
  close-up scale 1 0.933 / 0.909 / 0.905 / 1.088 ms (hull / steel / cyl / hidden), scale 2 1.777 / 1.689 / 1.678 / 2.033; shader vs old
  LitAerial on the same meshes +0.024 / +0.088 ms (+0.015 ms per MP, as before), models vs cylinders +0.028 / +0.099; stack 100 m
  scale 2 2.159 / 2.147 / 2.107 / 2.138; frame primitives: the models add 29 k at 100 m (were 58 k).
- Cost before (`vehicle_cost_check.gd`, RTX 4090, 1600 x 900 window, stack on the pad, terrain settled, 3 repeats of 120-frame GPU medians
  with the variants interleaved, scales 1 and 2 per axis; 2 is the engine's maximum): `hull` shader vs `steel` (old LitAerial on the
  SAME meshes) vs `cyl` (old cylinders) vs `hidden`. Frame-filling close-up (10 m from the belly): scale 1 0.929 / 0.909 / 0.903 /
  1.081 ms; scale 2 (5.8 MP) 1.764 / 1.685 / 1.674 / 2.031 ms: the shader costs +0.079 ms on the same meshes (+0.014 ms per MP), the
  models +0.090 ms over the cylinders. Stack at 100 m: scale 2 2.164 / 2.131 / 2.105 / 2.135 ms (shader +0.033, models vs cylinders
  +0.059); at 400 m 2.211 / 2.201 / 2.177 / 2.178 ms (+0.010 / +0.034). The earlier "-0.19 ms" only said the hull is cheaper than
  the sky / ground pixels it hides (here hull vs hidden -0.27 ms close-up); it is no cost figure of the shader. Draw calls 698-734
  (the models add 2-21 over the cylinders), 3.0-3.5 M primitives in the frame (the models add 20-125 k). VR extrapolation (NOT
  measured, a linear scaling of the close-up's 0.014 ms/MP to the two eyes' 45 MP of the Pimax target): up to ~0.6 ms when the hull
  fills the whole view; at the stack's usual distances 10 % of that or less. Dents are skipped under tiles. The plan's 1.0 ms [E]
  therefore holds with margin at 2 x pixels, and is an estimate at VR size.
- Cost of the tile maps (`vehicle_cost_check.gd`, same rig, 3 repeats; logs `tl_cost_before.txt` / `tl_cost_after.txt` in the plan's sdd folder):
  the shader vs the old steel on the same meshes in the frame-filling belly close-up, analytic -> texture: scale 1 +0.024 -> +0.032 ms
  (+0.017 -> +0.022 ms per MP), scale 2 (5.8 MP) +0.087 -> +0.118 ms (+0.015 -> +0.020 ms per MP): +30 % of the shader's own cost (three
  more texture reads per tiled pixel, no hex maths); triangles unchanged. The absolute frame times of the two runs differ (scale 2 hull
  2.45 vs 1.80 ms, the steel variant moved the same way), so only the within-run deltas are comparable. VR extrapolation of the
  per-MP figure to 45 MP: ~0.9 ms when a tiled hull fills both eyes (was ~0.7 ms), NOT measured.
- Repeats: the patch repeats every 0.30 x 0.52 m but its content is the same on every tile by nature (identical ceramic tiles); what
  could read as a pattern is the shade / repaired variation, which repeats after 38 x 33 m of the surface (a ship is 50 m long and
  its shield ~14 m wide at most). In the stereo close-up (`screenshots/vehicles/tl_after_st_eye0.png`) and the 5 m / 15 m / 60 m
  views no repetition, seam or mip bleed at tile edges is visible; the id map is what breaks the patch.
- Evidence tools: `vehicle_look_shots.gd` (real mission, orbit camera relative to a body, `--stereo=` for eye images, `--scan`
  for flap / fin deflections over time), `vehicle_cost_check.gd`, `vehicle_rig.gd` (the common mission / camera rig). Final fix round
  renders: `screenshots/vehicles/fin300_b0.png` (booster at T+300 s, fins -15.5 / 30.0 / -10.2 deg), `flap_real.png` (ship return,
  flaps 90 / 81 / 24.6 / 22.4 deg).

## Shapes redrawn from the 1:100 model (2026-10-31)
Reference (not shipped, nothing of it in the repo or `data/`): "SpaceX Starship V3 - 1:100 - IFT 12", Printables, author "I am Darth",
licence CC BY-NC-SA 4.0; 26 STL / STEP files in the git-ignored `screenshots/starship_v3_ref/`. Mm x 0.1 = full-scale metres. We only
MEASURE it (`tools/structures/measure/stl_measure.py`: bbox, profiles, views; `gltf_view.py` renders our glTFs the same way) and redraw
every part with our own generators; the values are in the detail file tagged `[E] model:`. It is not an asset of the game, so it is not
in CREDITS; a public release should ask the author. The print is partly adapted (thick walls, the `-w` wide fins, pegs, stands, the
engine plates): not copied. Photos in `screenshots/mechazilla/` win where they differ (stated per row).

| Part | Model (full scale) | Config / earlier detail | Photo check | Used now |
|---|---|---|---|---|
| Hull diameter | 9.0 m (tank parts r 4.50) | 9.0 | agrees | plain lathe, unchanged |
| Ship ogive | tangent ogive R 4.5, 13.4 m long from station 38.7, tip r 0.55, closes within 0.2 m | `nose_length_m` 13.4 | photo 108: curved part ~11-13 m: model | drawn 13.4 m (`nose_drawn_length_m`), tip cap 0.25 m; the model is fuller within 3 m of the tip (r 2.2 vs 1.75 at 2.7 m) |
| Aft flap | hinge 12.0 m, span 3.69 m, swept outer edge: full chord to 7.2 m, then to 0.44 m at the forward end; area 36.5 m^2; wedge 0.45 -> 0.2 m | hinge 12 m, span 4, 48 m^2 | f14 file 5099 / 4699 / 5299 s: planform agrees (fisheye, not fitted); 2 V brackets + 1 central actuator instead of the model's 4 pegs | model planform, wedge, 3 lugs, hinge cover |
| Forward flap | hinge edge 5.784 m, leaned 19.03 deg at stations 42.28-47.75 (aft end r 4.621, azimuth 20 / 160), span 3.154 m, swept 6-point quad with an aft foot beyond the hinge end (-1.45 m), area 12.37 m^2 (flat face of `ss3-ship-nose-flap-l.stl`, boundary loop), no lugs drawn (user 2026-11-01) | hinge 8 m, span 3, 24 m^2 | photo 108: a wide swept flap, not measurable (perspective); STL size, orientation and place used | the model's own place and lean (user 2026-11-01) |
| Grid fin | STL: irregular octagon 4.03 x 4.03 m over the cell axes (NOT regular: sides 0.35 / 2.37 / 2.01 / 1.10 / 1.98 m), pitch 0.350 m, lattice depth 0.4 on all variants (the catch STL's 0.7 is only the bar at the pylon; photo 0.45, `aero-derivation.md` Task 4b) | 12.63 m^2, r 7.19 (13.5 m^2 at span 3.0 until Task 4) | STL for the plan, photos for cusps / scallops, see "Fin plan round" | same irregular octagon, 3.53 m wide x 4.54 m span, pitch 0.350 m |
| Booster top / hot stage | 20 V teeth, ~2.8 m high in the model view | | NSF photo 2026-03 measured, see "Real-frames round" | 20 spindle rods pairs, fork feet, ring 0.32 m |
| Booster skirt | base ring r 4.65, four thin ring tubes at r 4.9 (stations 0.7 / 1.05 / 1.95 / 2.35), a comb of posts between them, one port 0.57 x 0.74 x 0.8 m at 2.9-3.7 m | flat skirt | no photo of the base | base ring, 4 ring flanges, 48 posts, 4 ports (count unknown) |
| Booster raceway | pill 0.7 x 5 m + a thin rail along the tank part | none | none | rail + one pill per tank section (side and spacing estimated) |
| Shield edge | aft +-90, upper +-78, nose all round | | superseded by frames, see "Tile boundary" | 90 / 112 / 105 / 180 deg |
| Transfer ports | block 0.9 x 0.45 x 0.8 m | none | no photo | two on the lee side at station 44 (position estimated) |
| Engines | plate heights 3.34 m (Raptor) / 4.6 m (RVac) = bell 2.1 + housing 1.0 / 3.6 + 1.0; RVac exit blobs 2.8-3.2 m on a 3.0 m ring | bell 2.1 / 3.6 m | n/a | unchanged (they already agree) |
| Hot-stage ring part | 9.0 m ring 0.6 m high, 3 pads | collar | n/a | collar kept |

### Physics deviations for the user
The drawn shape sits on the physics' pivots. The flaps' physics equals the drawing (config = detail outline and manifest hinge, `test_config_flaps_are_the_drawn_flaps`; vehicle physics plan Task 3, 2026-11-01, previous numbers `docs/history/guidance-missions.md`), and so do the grid fins (12.63 m^2 gross, centre of pressure at the drawn centroid r 7.19; Task 4, `aero-derivation.md`; earlier rows `docs/history/vehicle-models.md`). Where the real shape still differs from the physics numbers:

| Item | Physics (config) | Drawn (model) | Meaning |
|---|---|---|---|
| Forward RCS pod | station 46, azimuth 0 / 180, r 4.5 | not drawn; hull r 3.35 at station 46, the forward cover there reaches azimuth -2 deg | the physics pod floats 1.15 m off the hull, 1.54-1.75 m from the forward flap at every deflection; user decision (vehicle physics plan Task 3 report) |
| Fin azimuths | 0 / 90 / 180 (90 / 90 / 180) | as configured | aerial 2026-04, f12 1804 s: the spacing agrees, the middle fin points away from the tower |
| Fin station | 64 m | as configured | NSF 2026-03: 64.6 +-1 m: agrees |
| Hull ring pitch | n/a | 2.0 +-0.15 m (photo 108 Block 2: 1.8) | drawing only |
| Catch pins | booster 45 / 225 deg at 66.5 m | booster: NOT drawn (the disc on the hull above the fins was this pin as a fat stub; no frame shows a pin, the bearing is the flat bar under the fin root); ship pins unchanged. Config untouched | frames (f12 1804 s, aerial 2026-04, NSF 2026-03) show no pin standing out of the silhouette; the arms lie under the opposite fin pair. Hooks on fins are gone: none under any fin in f12 1804 s (that view cannot show it: vapour covers the hull below the fins). **Measured 2026-10-31 (`launch-structures-measurements.md`, "Catch pins against the grid fins"): 66.5 m is 2.5 m ABOVE the fin hinge (64.0): an arm deck there (8.6 m deep) runs through the fins' height band 62.6-64.3 m in every arm / fin layout. Highest clear deck 63.2 m (fins neutral) or 62.2 m (fins at 30 deg); proposal pins 62.5 m at azimuths 0 / 180 (under the opposite fin pair, hypothesis not contradicted), `catch_height_m` -4 m for the same booster bottom.** Pin azimuth, station and existence: physics decision for the user |
| Shield coverage | heating model: full windward half | drawn +-90 aft, +-112 at 13.5-21.5 m, +-105 up to 38 m, nose all round (+-10 deg) | the real shield is WIDER than the half the heating model treats above the aft part; drawing only |
| Ship gimbal station | station 0 of the ship (stack 72.3 m) | Raptor 2.1 m, RVac 3.6 m above it (exit planes at station 0) | thrust acts 2.1 / 3.6 m further aft than drawn: lever arm to the dry CG (23.5 m) 23.5 m in the physics, 21.4 m for the gimbaled Raptors as drawn, so TVC authority +10 % against the drawing; RVacs fixed. Stack length 124.4 m holds, the ship's and booster's lengths are unchanged |
| Ship tank bases | LOX main at station 3.5 m | engine plate at 3.7 m, housings up to 3.1 (Raptor) / 4.6 m (RVac) | the tank base lies 0.2 m below the plate, the RVac housing 1.1 m into it; nothing of it visible (CG / masses unchanged) |
| Booster dome top | tank volumes in the config, no dome | apex 71.7 m under the 0.6 m ring | CH4 tank top not compared: drawing only |
| Sea-level ring | 1.3 m | drawn from the config, real 0.89 +-0.04 m (fit) | the Raptor gimbal's roll / yaw lever is ~46 % longer than the real one; a config decision for the user |

### Evidence
Side by side (ours | model view | photo): `screenshots/vehicles/shapes_flaps.png`, `shapes_fins.png`, `shapes_booster_top.png`, `shapes_skirt.png`,
`shapes_nose.png`. What still differs: the model's grid fin has its catch corner (curved cells + a shaft at the corner) which we draw as four
hooks; the real fins show a serrated outer edge we do not draw; the V-truss rods are cylinders, not tapered; the forward flap is the STL's (hinge leans 19.03 deg);
the nose is fuller near the tip in the model; the number of skirt ports and posts is a guess; the skirt has no photo.
Engine look pass (2026-10-31, `vehicle_look_shots.gd` from real sessions, `screenshots/vehicles/lk_*`, `d_*`, `e_*`, `f_*`, `g_*`): the
truss, hinges, lugs, skirt rings and posts, aft / forward flaps and the all-round nose shield read as in the references. Added from the
model STLs (`tank-bottom` / `tank-center` sections) and the photo of the stack on the mount: four long line fairings `strakes` (0.82 m
high in the model, 1.2 m drawn, photo ~1.5 m; stations 3.2 .. 25.8 m on the -90 side and 3.2 .. 34.4 m on the +90 side, sloped top) and, from
the printed-model photos, the booster's forward dome (first inside the V truss, `dome_start_m` 3.6 / `dome_height_m` 3.6, since moved below it, see "Finish pass"; inside the physics
length) and forked feet of the V teeth. Not drawn yet: the grid fin's serrated outer edge, tapered truss rods, the ship's thick raceway
tubes widening at their foot, black round fittings, the ribs ring and panel lines (textures), the 1.7 m brackets on the strakes.
Catch: arm tops 95 m above the pad ground, pins at 66.5 m -> booster bottom 28.5 m above the pad ground = 8.5 m above the 20 m mount; the
photo of the stack on the mount puts the arm ledge at the pin level ~65-66.5 m above the mount top (+-1 m), so the pin station agrees with
the photo, the catch height itself is not measurable from the photos (no photo of a booster catch in `screenshots/mechazilla/`).
Triangles: booster 10,224, ship 9,100, flap 128 aft / 116 fwd, grid fin 552 (was 924).

## Finish pass (2026-10-31)
- Dome and bells: see "Stage interface" (the finish pass's lowered dome is superseded, `docs/history/vehicle-models.md`).
- Grid fins: pitch 0.35 m from the vertex lattice of `ss3-grid-fin.stl`, serrated edge, catch hooks only on `catch_fin_azimuths_deg` (detail
  file, manifest `look.catch_fin_azimuths_deg`, `vehicle_view._build_fins`). Azimuths: the opposite pair 0 / 180 (fallback; the pins in the
  config stay at 45 / 225 and so disagree with the hardware on the fins: user decision).
- Strakes: soft end, a 5 m straight ramp from 1.2 m to flush at constant width (the pointed tip was a taper of width AND height). Height 1.2 m =
  model 0.82 / photo (stack on the mount) ~1.5 m, uncertainty +-0.4 m.
- Steel: dent slope rms 0.03 -> 0.012 and the shader's large-scale mottle 3 % -> 1.5 % (bare stainless in photo 108 and the webcast is nearly
  smooth); NO tint or colour compensation: the blue cast is the reflected sky (F0 stays neutral). New faint longitudinal seams in the steel
  map (one per 4.71 m of arc, bead 0.2 x the ring weld, staggered by the per-ring shift of u).
- Booster truss rods are tapered (0.8 -> 1.3 x 0.15 m), the ship raceway has saddle feet every 2 m (0.7 x 0.4 m), the transfer ports are round
  fittings (r 0.4 m: collar, neck, cap; one steel material, not black). Skirt port / post counts: no photo exists, unchanged.
- Ribbed ring (photo 108) is the Block 2 interstage; the Block 3 booster has the open V truss (photos 116 and 2026-03), nothing drawn.
- Ship parts: flaps, hinges, nose, shield edge, chine, raceway, ports covered earlier; nothing further undrawn that a photo shows (the
  forward flap hinge leans 19.03 deg, the STL's).
- Catch (frame IE-Photo-41-2, Flight 5, https://cms.interestingengineering.com/wp-content/uploads/2024/10/IE-Photo-41-2.jpg; scale: tower
  ~146 m = 715 px, ~4.9 px/m [E]; the webcasts of flights 13 / 14 show only ship cameras):

  | Quantity | From the frame | Config |
  |---|---|---|
  | Arm truss above the ground | tower top 325 px, arm truss ~470 px: 145 px = ~30 m below the tower top: ~113 m +-10 m | 95 m |
  | Pins | just under the hot-stage ring, both sides of the hull | 66.5 m (booster top 72.3) |
  | Hardware azimuth | not decidable from this view | 45 / 225 deg |

  The booster is still ~30 m above its final position in the frame; the arm height carries +-10 m and a perspective bias. No physics change.

## Stage interface (2026-10-31)
The ship's engines hang inside the hull (user 2026-10-31: "die Triebwerke sind in der Hülle versenkt"). Every bell ends about at the hull's aft
edge; the ship's rim sits on the booster's hot-stage ring; the forward dome lies below that ring.

Exit planes, station above the ship's aft end (+ = inside the hull). The rims of both kinds lie in ONE plane, the sea-level bells are not deeper:

| Engine kind | Exit plane (drawn) | Uncertainty [E] | Real images | 1:100 `ss3-ship-engine-plate` (shape only) |
|---|---|---|---|---|
| RVac (`raptor_vacuum3`) | 0.0 m | +-0.15 m | Ship 39 photo from below, near-orthographic, 143 px/m: mean of the 3 cover planes -0.07 m against the hull rim (covers stand a little proud); camera fit below: hull rim +0.04 +-0.07 m against the RVac rims | y 0, flush with `ss3-ship-tank-bottom` y 0; rim r 1.15, ring 3.0; throat y 2.8; housing to y 3.7 / 4.6 |
| Sea level (`raptor3`) | 0.0 m | +-0.3 m | Camera fit to frame f12 138.9 s: +0.02 +-0.08 m (fit error; the lens is not modelled, hence +-0.3 m). Ship 39 photo with lifted shadows: faint rims at -0.1 m +-0.3 m. f13 2262 s: back Raptor rims, far hull edge and near rims follow one plane (y = 766 + 534 / d predicts the far edge at 826 px, seen 825 px) | y 0; rim r 0.65, ring 0.95; throat y 1.3; housing to y 2.7 |

Method of the fit (`.superpowers/sdd/2026-10-31-starship-prototype/ifc_bayfit.py`): pinhole camera, position, rotation, focal length, sea-level ring, sea-level plane and hull plane free, 5 rim ellipses and 5 aft-edge points read by eye (+-10 px), config rim diameters: rms 0.85 sigma; a mirrored assignment fails (rms 10). Forcing the sea-level plane to 0 fits about as well as the free fit (rms 0.87): the data carry no sign of an offset, the error stays what is stated. The fit also gives the real sea-level ring: 0.89 +-0.04 m (config 1.3 m, see deviations).

Stack-up (stations of the stack, booster base = 0; physics numbers unchanged):

| Station m | What | Source |
|---|---|---|
| 67.9 | truss foot (`hot_stage.height_m` 4.4) | photo estimate, earlier pass |
| 69.42 | barrel end / dome shoulder (`dome_start_m` 2.88) | section of `ss3-booster-tank-top`: shoulder y 20.7 |
| 71.7 | dome apex (`dome_height_m` 2.28) and foot of the hot-stage ring | same section: apex 22.98; `ss3-hot-stage-ring`; real frames show only that the dome lies right below the rims (f13 2253.6 s), no gap measurable |
| 72.3 | top of the ring = booster length = ship's aft rim = both exit planes | `length_m`; ring 0.6 m high: model only, no real image of a V3 stack close-up was found |
| 72.3 + 3.7 = 76.0 | engine plate; centre puck at 75.0 (r < 1.4) | model section; the puck in f13 2256.5 s |
| 72.3 + 2.1 = 74.4 / + 3.6 = 75.9 | drawn gimbal points Raptor / RVac | exit plane + `bell_length_m` |
| 124.4 | nose tip | config: the stack length holds |

Open truss above the shoulder: 2.88 m = the model's 2.8 m teeth; the rods run on 1.5 m in front of the barrel (photo height 4.4 m). Clearance rim to dome, stacked (test, config gimbal limits): Raptor 0.559 m at 15 deg, RVac 0.793 m (no gimbal).

Drawing: `[ship.engine_recess]` holds one exit station per kind, the manifest gives `ship_exit_station_m` and `ship_axial_offset_m` (= recess + bell length), `VehicleModels.axial_offset_m` applies it once for the bell pose and the plume (`vehicle_view._engine_axial_offset_m`); the direction stays the snapshot's. The booster's engines keep their stations (their bells stand out of its skirt: photos, unchanged). The ship's hull has no base disc any more; node `engine_bay` (rim, wall, 3 frames, 96 stringers, plate with centre cone, 4,100 triangles; dark own material) closes it.

## Real-frames round (2026-10-31)
Evidence `screenshots/vehicles/real/cmp_*.png` (real | engine). Frames: `data/webcasts` file seconds (`screenshots/mechazilla/webcast/real/`), NSF photos in `screenshots/mechazilla/pad-b/`. Earlier statements that the streams show no booster cameras are wrong: flights 12 / 13 / 14 show booster side and base cameras from T+3 min.

| Item | Reading (frame) | Drawn | Uncertainty |
|---|---|---|---|
| Fin attitude on the pad / in ascent | plate across the axis, cells axial (aerial 2026-04, f12 1804 s just after liftoff, stack photo 2026-05). No frame shows a folded fin | d = 0 = cells along X; before this round the drawing was turned 90 deg about the radial (cells tangential), which was wrong | certain |
| Fin mounting | regular octagon hung on a corner on a short stem, near the hull (aerial, f12 1804 s) | vertex at r = 4.65 m, shaft + small dark housing | housing not resolved |
| Fin size | near fin 168 px at 41 px/m = 4.1 m wide (NSF 2026-03) | across flats 3.8, width 4.11 m | +-0.2 m |
| Cells | SUPERSEDED: pitch 0.35 m (STL, rim teeth of the 2026-03 arm photo, 102.9 cells; `aero-derivation.md` Task 4b). This round's reading: rows 0.67 m apart = 0.47 m (f12 1804 s) | walls +-45 deg, pitch then 0.47 m, now 0.35 m | +-0.02 m |
| Spikes | a four-sided pyramid under every wall crossing, pointing aft, pale faces lit in f12 1804 s | one pyramid per crossing, 0.22 m long | length +-0.08 m (from the rim teeth, not measurable directly) |
| Rim | deep band 0.43 m, aft edge toothed 0.61 m pitch (NSF 2026-03) | depth 0.5 m incl. teeth 0.25 m, pitch 0.66 m | +-0.1 m |
| Plain / catch fins | spacing 90 / 90 / 180: the opposite pair lies at the arms, the middle fin points away from the tower (aerial 2026-04, NSF 2026-03). No catch hardware visible on any fin from below (f12 1804 s), the fallback hooks were invented: removed | no hooks; fins differ in nothing | hardware on the upper side not seen |
| V rods | spindles: 0.26 m at the middle, 0.15 m at the ring, 0.11 m at the valley (9 m = 370 px, 10.7 / 6 / 4.7 px); rods 2.72 m, fork legs 0.97 m 0.3 m wide, ring band 0.32 m | so | +-0.02 m width |
| Flaps | aft planform agrees; bare lee face with panels ~0.95 x 0.74 m, two diagonal stiffeners, six dark hexagon covers near the aft end, 2 V brackets + 1 central actuator (f14 5099, 4699, 5299 s) | map `flap_lee.png`, 6 covers, 3 lugs; forward flap uses the same map (not verified), no lugs (user 2026-11-01) | fisheye: +-0.5 m |
| Hinge heat protection | thick black tiled bulges along the hinges on the belly side (f12 1804 s, f13 5905 s, f14 4949 s) | `hinge_covers` in the ship: 1.3 m wide, 0.5 m high, 0.8 m beyond the hinge ends, tapered, tiled by the coverage; wholly on the belly side, so the flap's sweep (lee tangent to radial) is clear (test) | +-0.3 / +-0.2 m |
| Hull lines | dotted stringer lines ~0.35 m apart, dots ~0.09 m, rings 2.0 m; solid lines only at rings; a faint line beyond ~30 m, gone beyond ~100 m | steel map | +-0.1 m |
| Bells | Raptor: throat 1.35 m above the exit (1.04 x width), chamber + powerhead above; fine ribs. RVac: 80 flutes on the lower ~62 %, smooth cone above, throat 3.0 m (f14 2515, 2563 s) | throat in the mesh, gimbal / exit / rims unchanged; plume anchor unchanged; `vehicle_engine.gdshader` with a rib map (ribs show since the fix round 2026-10-31: `VehicleView._build_engines` never applied `VehicleModels.engine_material`, the bells wore the hull steel; the hot spot on the shoulder was that steel's sun glint) | +-0.3 / +-0.4 m |
| Aft skirt comb | ribbed band at about 3-4 m on the ship + a ring of arched slots below (QD photo 2026-05, f12 5674 s); also the ladders beside the name and around the payload door | not drawn (location-bound; no generic map) | n/a |

### Tile boundary
Second side (fix round 2026-10-31): f14 2100 s (ship seen belly-on at the pad, both flaps at the silhouette) shows both edges black to the silhouette; f13 1940 s shows a thin bright strip on the left edge along the barrel (~0.6 m of apparent width, which would be a boundary near 60 deg from the belly on that side, or just a sky glint on the tile: ambiguous, 1 of 3 frames). No consistent asymmetry, so the table stays symmetric (+-10 deg); the other side is read as symmetric within that tolerance, not as measured. The QD photo's lee tile patch around the raceway is not drawn.

Measured on the Ship 42 photo (2026-09, telephoto, hull 58 px x 2 scaled, stations 19.4 px/m in the crop): boundary offsets from the axis -0.17 R (22-38 m), +0.09 R (below 13.4 m), a wider patch -0.43 R at 13.4-21.6 m; anchor: the aft flap hinge lies on the boundary (f14 4949 s, f12 5674 s) and config azimuth belly +-90, and the stack photo 2026-05 shows both silhouettes black to the base. Solving gives the view 85 deg off the belly; check: the forward flap hinge at x -0.42 R gives 110 deg (config +-110).

| Station m | Half angle now | Before | +- |
|---|---|---|---|
| 0-13 | 90 | 90 | 8 |
| 13.5-21.5 | 112 | 84-78 | 10 |
| 22-38 | 105 | 78 | 10 |
| 41-52.1 | 180 | 180 | n/a |

Only one side is measured; the other side is taken symmetric (both silhouettes in the stack photo; entry frames from both aft cameras look alike, not measured). The QD photo (Ship 39, 2026-05) shows a lee-side patch of tiles around the raceway from ~6.5 m upward (herringbone + hexagons) and bare steel below it, not drawn: the table cannot hold it, and its azimuth is not determined. The chine follows the table.

### Black parts (sunlit frames)
Albedo [E] from the sRGB of sunlit parts against known surfaces:

| Part | Frames | Albedo |
|---|---|---|
| Fin rim / cells | NSF 2026-03 (sun), aerial, f12 1804 s: dark grey / black | 0.07 +-0.03 / 0.025 +-0.01 |
| Raptor bells | f14 2563 s (sky light): near black. Soot possible, no clean sunlit view | 0.05 +-0.03 |
| RVac | f14 2515 s: grey metal: **contradicts black** | 0.25 +-0.1 |
| Chopstick arms | NSF 2026-03: black paint (already) | 0.035 |
| Tower | legs bare steel, **bright in direct sun** (f8 2580 s, f12 1804 s), near black in shade (stack photo sRGB 17 18 23); braces dark. "Black" holds only in shade | 0.15 +-0.08 (mean) |

Tower material lowered accordingly (`tower_material.gd`, `tower_view.gd`, also `launch-structures.md`); the far tier's fitted colours were not refit.

### Fin aliasing
Lattice and spikes draw up to 120 m (`FIN_LATTICE_RANGE_M`: wall 4 cm = 1 Pimax pixel at ~100 m), beyond that the opaque `plate`. A hard swap, measured 2026-10-31 (`tile_shimmer_check.gd --surface=fin`, booster fin 0 from above, 20-320 m, patch on the projected fin): luminance step 115 -> 125 m 1.014 (gate 1.10), temporal <= 0.0012 (ref 0.0012), error <= 0.037 (gate 0.15). Not seen in the headset. Cell pitch: frame f12 1804 s counts ~9 +-1.5 columns over the 4.1 m width = 0.46 +-0.07 m, so the config's 0.47 stays.

## Open points
- Interface look: the arrangement agrees. Still different: the sea-level cluster is too wide (config ring 1.3 m, real 0.89 m); no powerhead detail above the throat (plain cylinder against the pump and pipe bundle of f14 2515 s); plate: dark, the real one is crumpled foil with bolt rings (f14 2515 s), not drawn; the bay wall's real corrugation and four frames are only partly drawn; ship plumes pass through the booster at hot staging.
- Steel shimmer after the look pass (2026-10-31): temporal <= 0.0008, error <= 0.044, luminance ratio 1.134 (not gated for steel); after the thinner welds / dots 1.180, exactly at the gate (the 20 m patch dips; no headroom left). The flat flap / tile patches that
  read tan under a low Sun are the black tile (albedo 0.03) in direct, reddened Sun (~14 deg): diffuse, specular only
  ~10 %, AgX lifts it (probe in `sky-fix-report.md` Fix 2). Not a bug. Steel luminance ratio gated at 1.18.
- Shade sides: lit by the sky's radiance cubemap, which follows the Sun and the place since 2026-10-31 (`atmosphere-scattering.md`, "Radiance cubemap"). A white hull's
  shade side in a real flight session on the pad: `73 89 105` by day (was `0 0 1`), `0 0 0` on a moonless night (`sky_radiance_check.gd --flight`).
- Coverage vs thermal: see the deviations table and "Tile boundary".
  model treats the full windward half (90 deg). The earlier 75 / 65 / 55 / 30 table is superseded (the user was asked; numbers in the table).
- Catch pins: see the deviations table: the pins stay as configured, frames give no sign of them.
  catch features at / integrated with the grid-fin mechanisms (unconfirmed, partly older designs). Modelled as in the config
  (physics and visual agree); a check against Block 3 photos and a decision on the physics numbers are open.
- Config numbers that look odd next to the references: the 20 outer engines on the 4.1 m ring overlap by 0.16 m at the exit planes and the outer bell edge (4.82 m) passes the hull
  radius (4.5 m).
- Flaps at d = 0 (folded, not the rest pose, which is d = 90) lie on the hull's tangent plane: the tip stands up to 0.9 m off the curved hull.
- Raceway position, tile coverage, hot-stage and bell dimensions are estimates; no Block 3 drawing exists.

## Fin outline round (2026-10-31)
Sources: the factory photo `screenshots/starship_v3_ref/eiv42cz7buif1.jpg` (a real fin on its fixture; outline, lattice), the
user's installed-fin image (V3 fin against the sky: hearts, light band, ring + struts, serrated lower edge; user ruling
2026-10-31: used as a DIRECT template for shape, no photo pixel in any asset, it carries a photographer's watermark), the
person-on-fixture factory photo (wall depth, cusps, cross rib, root box), the arm photos (`padb_chopsticks_around_booster_top_2026-03.jpg`,
the stacking image). Maps stay generated by our scripts.

| Quantity | Value [E] | Method, uncertainty |
|---|---|---|
| Lattice | square, walls at +-45 deg to the outline's axes (measured 39.0 / 127.3 deg in the photo = the fin turned 6.5 deg in its plane) | LSD segments in the fin (`measure/fin_lines.py`), 69 / 12 segments; vanishing points -> f^2 = 1.7e7 px^2 (f ~ 4100 px), the picture is nearly face-on |
| Pitch | SUPERSEDED: 0.35 m (`aero-derivation.md` Task 4b). Then 75.7 px = 0.49 m | spectrum peak of the rectified picture (`fin_rectify.py`), wrong as the "Fin plan round" below shows |
| Overall | 4.11 m wide x 3.84 m span, gross 11.80 m^2 | vertices read by eye on the rectified picture, +-8 px = +-0.06 m (`fin_outline.py`) |
| Sides / chamfers | AB 1.49, BC 0.88 (root chamfer under the pylon), CD 1.49, DE 0.67, EF 1.33, FG 0.59, GH 2.38, HI 2.00 (the long lower-left chamfer, 4.1 pitches), IA 2.00 m | same; all nine sides unequal, convex |
| Cells per side | 3.0 / 1.8 / 3.0 / 1.4 / 2.7 / 1.2 / 4.9 / 4.1 / 4.1 pitches | side / pitch |
| Wall depth | 0.50 m (cusp tip to cusp tip), valley 0.19 m below each tip (0.12 m of wall left in the middle) | person photo: valley ~0.4 of the depth; +-0.05 m |
| Cusps | one tip per crossing on both faces: sharp horns (vertical tangent), concave arcs between; they are what the earlier "spikes" were (same feature, lit pale from below in f12 1804 s) | person photo, factory photo |
| Wall thickness | 0.04 m, upper bound (not resolved) | |
| Rim scallops | one per wall end, pitch 0.35 m (= pitch / sqrt 2), depth 0.22 m | factory photo; the stacking image reads ~0.44 +-0.1 m (low resolution, 9-10 teeth over the width): kept 0.35 |
| Pylon | ring 0.32 m radius x 0.16 m, 12 bolts, tapered root block to the root chamfer, 3 triangular struts into the lattice, cross rib 1.05 m from the root | installed-fin photo, person photo, +-0.15 m |
| Light band | 0.20 m flat rim face on the forward side | installed-fin photo |
| Fin thickness | edge-on in the stacking image: 0.5 +-0.1 m cannot be separated from the plate seen 10-20 deg from above (up to 1.0 m apparent) | kept 0.5 m |

Triangles: 3,130 per fin (frame ~720, lattice ~1,900, rest pylon / band / ring); the far plate tier keeps the outline (12
triangles). Three fins add 9.4 k to a booster that had 10 k: state for the user, `vehicle_cost_check.gd` not rerun here.
The photographed tie rods at the crossings are not drawn (600 triangles).

### Catch bearing (flat bar under the root)
Seen as a bright flat bar slanting under the fin root next to the toothed hinge ring, ~0.5 x 0.15 m (+-40 %, oblique, fin
~170 px/m). Working assumption (user ruling 2026-10-31): the booster rests left and right on these bars, face down onto the arm
(the stacking image and the NSF arm photo show the fins edge-on with serrated undersides just above the arm top chord). Drawn on
`catch_fin_azimuths_deg` = 0 / 180 only; the 90 deg fin shows no bar (left / right pair in the user's two-fin image: the fins
sit +-53 deg from the view axis, nearer to the 90 deg gap of fins 0 + 90 / 90 + 180 than to a 180 deg gap, which would sit at
+-90 deg; the second opposite fin has not been seen). Whether it is a bearing is not established; the cylinder and soot streak
that first looked like part of it are shadows / reflections on the steel (user) and are not drawn.

## Fin plan round (2026-10-31, after the user's STL ruling)
User rulings: the plan geometry (outline, lattice, cross rib, mount) comes from the 1:100 plain fin STL
(`screenshots/starship_v3_ref/ss3-grid-fin.stl`, x 0.1; the `-catch` variant only adds a 0.7 m thick bar at the pylon, `-w` is
the print-wide variant: ignored; correction Task 4b: the STL's aft face has shallow scallops, 0.09 m, and its walls of 0.044 m are print widths); the fin is mirror-symmetric about the hinge axis with cells at 45 deg to it; the STL lacks the deeper
serrations, so cusps and rim scallops stay from the photos. Our own generator redraws it, no mesh data in the repo.
Scripts: `measure/fin_stl_outline.py` (outline, axis, pitch), `fin_stl_rib.py` (walls by direction), `fin_cell_map.py` (cell map),
`fin_lattice.py` / `fin_fit.py` / `fin_vertices.py` (homography from lattice points of an image).

| Quantity | Value | Check |
|---|---|---|
| Outline | irregular octagon, NOT regular: (u, v) = (4.03, 3.785), (3.785, 4.03), (1.422, 3.918), (0, 2.495), (0, 1.4), (1.4, 0), (2.495, 0), (3.918, 1.422); sides 0.35 (root chamfer), 2.37, 2.01, 1.10, 1.98 (tail chamfer) | the old line "regular octagon 4.03 x 4.03" is wrong and corrected; gross area 12.63 m^2 |
| Symmetry | exactly about the diagonal u = v (fitted axis 45.00 deg, mirror error 0.000 m) = the hinge axis through the root chamfer | in axis coordinates: width 3.53 m, span 4.54 m |
| Lattice | square walls along u and v = +-45 deg to the axis; wall centres at 0.02 + 0.35 k; a crossing lies on the axis (every 0.495 m) | pitch 0.3502 m from the wall peaks |
| Cross rib | the diagonal u + v = 5.34, corner to corner (width 3.53 m), 1.75 m from the root chamfer, 2.79 m from the tail chamfer | `fin_stl_rib.py`: s = 3.72 / 3.78 (two faces) |
| Cell map | rows along the 45 deg lines, full + partial cells: 3+2, 5+2, 7+2, 9+2, 11+2, 11+2, 11+2, 10+2, 9+2, 8+2, 7+2 (partials 0.06-0.62 of a cell), total 102.9 cells (91 full, 33 partial) | `real/rect/stl_cellmap.png` |
| Installed-fin image | the heart cells (87 found) fit a lattice homography with residual 0.017 grid steps (0.3 px) rms; the outline polygon read on it holds 102.9 cells = the STL; 3 rows of 11 cells, flank rows 9, 7, 5, 3 agree. The earlier pitch 0.49 m (8.3 pitches over 4.1 m) was wrong: 4.03 m / 0.35 = 11.5 cells. Caveat: the metric (square cells) of the rectified image was not recovered from the hearts alone (the nearest-neighbour basis is sheared), only the cell count and area are used | `real/cmp_fin_rectified.png` |
| Root zone | STL: radial walls in a fan between the rib and the root. User template `333b32ea` and right-fin image `6318470d`: a fan of struts from the toothed hinge ring to the cross rib with cells between them (open triangular frame). Drawn: two struts ring to rib (+-0.45 m at the rib) and the lattice over the whole root triangle. The factory photos' tray (open housing, floor) is read as the production fixture, not the installed fin. Rim: band 0.08 m measured against the render (~0.09 m); the wide flat look in earlier shots is the 0.5 m deep lit wall seen obliquely, no change. | |
| Pylon | round mount ring on the barrel (V3 B19 image: dark ring ~1.4 m, replaces the V2 block; hull fairing now a 1.4 m x 0.12 m disc), hinge ring r 0.40 m touching the root chamfer | `launch-structures-measurements.md` |
| Cusps / scallops (photos) | pointed tips at the crossings, valley 0.10 m, scallop 0.10 m deep at 0.2475 m, band 0.08 m | `real/cmp_fin_template.png` |

Triangles 3,448 per fin (cusp_steps 3; 4,872 at 5). Chirality: the STL is symmetric, so no chamfer "faces forward"; the band is on the
forward face of the root tapers.

### Fin frame and hearts (2026-10-31)
- Frame: the lattice walls end inside a continuous closed wall (`frame_m` 0.07 +-0.03 m: about 1.5-2 wall thicknesses in the template and
  the factory photo, not resolved; wall depth = the full fin depth, the free aft edge a row of soft arcs, one per wall end, 0.22 +-0.05 m
  deep, so the frame is no deeper than the saddle valleys at its arcs). Black (`fin_cells` material); the light band only on the root
  chamfer and the two tapers. Free wall ends / star spikes at the outline are gone; the cusps remain only inside as saddle tips
  (`cusp_soft` 0.3: sin profile blended with the round scallop).
- Hearts: view tilt of the template `333b32ea` from the lattice fit (`measure/fin_tilt.py`, weak-perspective reading of the
  nearest-neighbour basis, Jacobian of the homography at the fin centre): 37.7 deg off the cell axis (+-6 deg), tilted along the
  chord, vehicle axis vertical, from below / aft. Ray test of the openings (`measure/fin_hearts.py`): symmetric scoops on both faces
  give lenses at any tilt (the earlier look, tips at both faces cancel the notch); hearts need ONE scooped and ONE flat face
  (scoop 0.3 of 0.5 m) and a tilt of 25-30 deg (at 38 deg the hearts shrink to chevrons). Adopted: aft face scooped (the factory photo shows
  the deep scoops on the visible face), forward face flat, `cusp_dip_aft_m` 0.30 / `cusp_dip_fwd_m` 0, view 30 deg. Hypothesis, not seen:
  the forward face of a real fin.
- Look shot of the view: `vehicle_look_shots.gd` `--t_plus=54`, `fr3,0,64,-87,0,-50,3,aimy7` (booster, station 64, 100 m away from the
  fin at the 30 deg tilt, `aimy<m>` / `aimz<m>` move the aim point off the axis onto a fin); side by side `real/cmp_fin_template.png`.
