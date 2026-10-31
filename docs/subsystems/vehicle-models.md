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
  radius; ship: tangent ogive of the DRAWN length 13.4 m (not the physics 17 m) with a blunt tip; booster: flat top), `hot_stage`
  (collar), `top_truss` (20-tooth V truss), `skirt` (rings, posts, ports), booster `raceway`, `fin_fairings`, `catch_pins` at the config stations / azimuths / protrusion, ship `chine` (swept strake
  along the tile boundary, azimuth = belly +- the coverage half angle of the tile table), `raceway` and `ports`. Hull UV = (arc metres
  at the barrel radius, station metres): glTF u = arc, v = station (Blender v = 1 - station).
- `flap_fwd.gltf`, `flap_aft.gltf`: hinge line = X axis from the aft end (origin), panel along +Z at rest (stowed, leeward),
  outer face +Y. The game's basis (X station, Y e_r, Z stow direction, mirrored when the stow points to the belly side) and
  the rotation about +X by `rotation_sign_about_plus_x` x deflection are derived from `core/src/aero/flap.cpp` and tested
  against a re-implementation of it. Planform = the 1:100 model's (swept, wedge section, hinge lugs, `[flap.aft|fwd]`), the drawn hinge
  edge centred on the physics hinge; it does NOT keep the physics area (see "Physics deviations for the user").
- `grid_fin.gltf`: pivot on the hull at the hinge, hinge axis +Y (radial). The core turns the lattice normal from the tangent
  toward +X: a right-handed rotation by +d about the radial axis, zero = rest. Drawn size = the model's octagon (4.03 m across flats,
  root flat on the hull), not the config area / span. Egg-crate lattice, cells open along the tangent, `hooks` = catch hardware.
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
  coordinates. Stage separation rebuilds on the new stage list: a model never stays on the wrong stage (a hidden view still
  rebuilds, it only skips posing and plumes). Materials: the hull shader's three (below); engines keep the view's `LitAerial` steel.
- Flap pose: hinge at (aft station, hull radius at the hinge azimuth), basis (X, e_r, stow) with stow = -rotation_sign * (X x e_r)
  (mirrored when the manifest sign is +1), then rotation about +X by sign * deflection; manifest entry matched by stage id,
  hinge azimuth and aft station. Fin pose: pivot at (station, hull radius, azimuth), rotation about +X by the azimuth, then about
  the radial Y by +deflection. Both are tested against re-implementations of `flap_span_direction` and the grid-fin normal.
- Plumes start at the model's exit plane (manifest: Raptor 2.1 m, RVac 3.6 m below the gimbal point); cylinders keep 3.0 m.

## Steel map (`tools/structures/hull_textures.py` -> `data/models/starship_block3/hull_steel.png`)
- Baked by the driver (numpy, no Blender), procedural from the detail file's `[hull]` numbers (ring pitch, weld height / half
  width, `dent_slope_rms`; no photo pixels), covered by the manifest's input hash (the script is hashed), deterministic (fixed
  lattice seed, fixed zlib level, test: byte-identical PNG). One RGBA8 PNG, 1024 x 512 texels, tiling: u = circumference / 6
  (4.71 m, 4.6 mm per texel, closes around the R = 4.5 m hull), v = the 1.8 m ring pitch (3.5 mm per texel) with the weld bead
  at v = 0 (a weld at every multiple of the pitch). Channels: R, G slope along u / v (bead: triangle 12 mm high, 10 cm wide;
  dents: two noise octaves, ~1.2 m and ~0.38 m, rms slope 0.03), B slope energy sx^2 + sy^2, A heat-affected band coverage.
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
| Ship ogive | tangent ogive R 4.5, 13.4 m long from station 38.7, tip r 0.55, closes within 0.2 m | `nose_length_m` 17 (physics) | photo 108: curved part ~11-13 m: model | drawn 13.4 m (`nose_drawn_length_m`), tip cap 0.25 m; the model is fuller within 3 m of the tip (r 2.2 vs 1.75 at 2.7 m) |
| Aft flap | hinge 12.0 m, span 3.69 m, swept outer edge: full chord to 7.2 m, then to 0.44 m at the forward end; area 36.5 m^2; wedge 0.45 -> 0.2 m; 4 lugs at 0.75 / 4.2 / 7.7 / 11.2 m | hinge 12 m, span 4, 48 m^2, thickness 0.35, chamfered rectangle | photo 108: the trapezoid with the cut forward corner agrees | model planform, wedge, 4 lugs, hinge fairing |
| Forward flap | hinge edge 5.8 m (leans 19 deg along the ogive), span 3.04 m, swept quad with an aft foot beyond the hinge end (-1.49 m), area 13.8 m^2, 3 lugs | hinge 8 m, span 3, 24 m^2 | photo 108: a wide swept flap, not measurable (perspective); model kept | model planform centred on the physics hinge, hinge kept straight |
| Grid fin | regular octagon 4.03 x 4.03 m, 10 cells across (0.4 pitch), depth 0.4 (catch version 0.7: hardware), curved catch corner, shaft 0.3 m | 13.5 m^2 at span 3.0 (chord 4.5), depth 0.35, rectangular | photos 2026-03: fins read as dark slabs with a SCALLOPED outer edge; the lattice cannot be seen | octagon lattice + 4 catch hooks on the root side; the serrated edge is not drawn (open) |
| Booster top / hot stage | 20 V teeth (18 deg pitch) between a top ring (y 22.98) and the hull, ~2.8 m high in the model view; no vent plates | 24 vent plates 3.2 m tall + louvres (first pass, read from photo 108) | chopsticks photo 2026-03: V truss ring with rod struts ~4.9 m high, posts + collars at the hull; the "louvres" of photo 108 are this truss | vent plates removed; V truss 20 teeth, 4.0 m + 0.4 m posts (photo height), collar ring; rods 0.3 m thick (the real ones are tapered) |
| Booster skirt | base ring r 4.65, four thin ring tubes at r 4.9 (stations 0.7 / 1.05 / 1.95 / 2.35), a comb of posts between them, one port 0.57 x 0.74 x 0.8 m at 2.9-3.7 m | flat skirt | no photo of the base | base ring, 4 ring flanges, 48 posts, 4 ports (count unknown) |
| Booster raceway | pill 0.7 x 5 m + a thin rail along the tank part | none | none | rail + one pill per tank section (side and spacing estimated) |
| Shield edge | aft shell +-90 deg (to ~12 m), upper shell +-78 deg (14-38 m), nose shell closes all round (from ~38.5 m) | 75 / 65 / 55 / 30 deg at 0 / 20 / 35 / 52 m | photos 108 / 2026-05: the nose is dark all round and the barrel's sides are dark nearly to the edge: model | table replaced: 90 / 90 / 84 / 78 / 78 / 180 / 180 deg at 0 / 12 / 14 / 20 / 38 / 41 / 52.1 m; chine ends at 38.5 m |
| Transfer ports | block 0.9 x 0.45 x 0.8 m | none | no photo | two on the lee side at station 44 (position estimated) |
| Engines | plate heights 3.34 m (Raptor) / 4.6 m (RVac) = bell 2.1 + housing 1.0 / 3.6 + 1.0; RVac exit blobs 2.8-3.2 m on a 3.0 m ring | bell 2.1 / 3.6 m | n/a | unchanged (they already agree) |
| Hot-stage ring part | 9.0 m ring 0.6 m high, 3 pads | collar | n/a | collar kept |

### Physics deviations for the user
The drawn shape sits on the physics' pivots; nothing in `config/` was changed. Where the real shape differs from the physics numbers:

| Item | Physics (config) | Drawn (model) | Meaning |
|---|---|---|---|
| Aft flap | 12 x 4 m, 48 m^2 | 12 x 3.69 m swept, 36.5 m^2 (-24 %) | the aero model has 31 % more aft flap area (control authority, flap heating area) than the drawn flap; matching the model would mean ~36 m^2 |
| Forward flap | 8 x 3 m, 24 m^2 | 5.8 x 3.04 m swept, 13.8 m^2 (-43 %) | the forward flaps are the most overstated; hinge moment arm and pitch trim are affected |
| Grid fin | 13.5 m^2, span 3.0, chord 4.5, centre of pressure at r 6.0 | octagon 4.03 x 4.03 m (about 13.5 m^2 gross, ~85 % lattice cover), root at the hull: centre at 4.5 + 2.0 = 6.5 m | the area agrees, the lever arm is 0.5 m longer in the drawing |
| Ship nose | `nose_length_m` 17 | ogive 13.4 m | drag / nose-radius numbers use the longer, sharper nose; the drawn one is blunter |
| Forward flap hinge | straight, parallel to the axis, 38-46 m | leans 19 deg along the ogive, 5.8 m | the flap normal at rest differs by ~19 deg (in the game the flap lies on the tangent plane) |
| Catch pins | booster 45 / 225 deg at 66.5 m | model: catch hardware at the grid-fin corners (hooks) | the drawn hooks are on the fins; the pins stay as configured (a decision on the physics numbers is open) |
| Shield coverage | heating model: full windward half | drawn +-78 .. 90 deg, nose all round | unchanged (drawing only) |

### Evidence
Side by side (ours | model view | photo): `screenshots/vehicles/shapes_flaps.png`, `shapes_fins.png`, `shapes_booster_top.png`, `shapes_skirt.png`,
`shapes_nose.png`. What still differs: the model's grid fin has its catch corner (curved cells + a shaft at the corner) which we draw as four
hooks; the real fins show a serrated outer edge we do not draw; the V-truss rods are cylinders, not tapered; the forward flap hinge is
straight; the nose is fuller near the tip in the model; the number of skirt ports and posts is a guess; the skirt has no photo.
Triangles: booster 6,528, ship 9,100, flap 128 aft / 116 fwd, grid fin 552 (was 924).

## Open points
- Steel shimmer after the look pass (2026-10-31): temporal <= 0.0008, error <= 0.044, luminance ratio 1.134 (not gated for steel). The flat flap / tile patches that
  read tan under a low Sun are the black tile (albedo 0.03) in direct, reddened Sun (~14 deg): diffuse, specular only
  ~10 %, AgX lifts it (probe in `sky-fix-report.md` Fix 2). Not a bug. Steel luminance ratio gated at 1.18.
- Shade sides: lit by the sky's radiance cubemap, which follows the Sun and the place since 2026-10-31 (`atmosphere-scattering.md`, "Radiance cubemap"). A white hull's
  shade side in a real flight session on the pad: `73 89 105` by day (was `0 0 1`), `0 0 0` on a moonless night (`sky_radiance_check.gd --flight`).
- Coverage vs thermal: the shield is drawn from the 1:100 model (see "Shapes redrawn": +-90 / +-78 deg, nose all round); the aero heating
  model treats the full windward half (90 deg). The earlier 75 / 65 / 55 / 30 table is superseded (the user was asked; numbers in the table).
- Catch pins: the config has booster pins at 45 / 225 deg between the fins at station 66.5 m [E]; web reports for V3 put the
  catch features at / integrated with the grid-fin mechanisms (unconfirmed, partly older designs). Modelled as in the config
  (physics and visual agree); a check against Block 3 photos and a decision on the physics numbers are open.
- Config numbers that look odd next to the references: the 13.5 m^2 fin at a 3.0 m span (radius_m 6.0) gives a 4.5 m chord;
  the 20 outer engines on the 4.1 m ring overlap by 0.16 m at the exit planes and the outer bell edge (4.82 m) passes the hull
  radius (4.5 m).
- Flaps at rest lie on the hull's tangent plane (stowed, flat): the tip stands up to 0.9 m off the curved hull.
- Raceway position, tile coverage, hot-stage and bell dimensions are estimates; no Block 3 drawing exists.
