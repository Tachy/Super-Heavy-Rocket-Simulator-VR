# Terrain rendering (M8 Plan B)

History and the reasons behind superseded decisions: `docs/history/terrain-rendering.md`.

Plan `docs/superpowers/archive/plans/2026-10-24-m8b-terrain-rendering.md`, spec
`docs/superpowers/archive/specs/2026-10-23-m8-terrain-tiles-design.md` (Plan B). The renderer draws exactly the physics'
surface (`StarsimCore.surface_samples` on the shared terrain cache, `docs/subsystems/earth-terrain.md`) with imagery
on top; it replaced the interim `GroundPatch`.

## Model

- **Tiling.** Cube sphere (6 faces, tangent-warped), quadtree keys `Vector4i(face, level, i, j)`; the 24 level-1 nodes
  are built at setup and never freed. A node is a 64 x 64-cell grid (65 x 65 vertices) plus a skirt ring 2 cells deep
  (hides cracks between neighbours one level apart), vertices float32 relative to the node centre (float64
  body-fixed). Heights: one `surface_samples` call per node on a 67 x 67 lattice (one ring outside for seamless
  central-difference normals); without data/ the flat rule (site height near the sites, the ellipsoid / geoid
  elsewhere).
- **LOD.** A node splits when the camera is closer to its bounding sphere than `LOD_FACTOR` x its size and its mesh
  spacing is coarser than the finest data under the whole lattice (`surface_finest_resolution_m`) or, where data is
  coarser or absent, than the curvature limit sqrt(8 R x 1 cm) (714 m Earth, 373 m Moon: cells sag <= 1 cm, so
  vehicles stand on the drawn ground); it merges beyond `MERGE_FACTOR` 1.15 x that distance; `MAX_LEVEL` 21;
  `LOD_FACTOR` 2.5. A parent stays drawn until all four children are uploaded (the drawn leaves always tile the
  sphere). At OLP-1 with data the leaf under the camera reaches level 18 (1 m 3DEP cells).
- **Morph.** Odd vertices carry the offset to the parent level's surface (CUSTOM0) and the parent-level normal
  (CUSTOM3); the shader blends both by the camera distance between `morph_start_m` and `morph_end_m`, set per node from
  `LOD_FACTOR` / `MERGE_FACTOR` so that a node is unmorphed whenever its children appear or vanish and a child is
  fully morphed at that moment (start = 1.15 K s + 2 r, end = 0.97 K x parent size; start < end for every level >= 2
  on both bodies; at K 2.5 the band is ~0.55 node sizes, smallest margin of start below end 0.05 of the end; quadtree
  test). The level-1 roots never morph. A node drawn for the first time also starts fully morphed (`morph_floor` 1 ->
  0 over `MORPH_IN_S` 0.5 s [E]): children whose upload came late slide in instead of popping; a parent shown again
  after a merge gets no floor.
- **Camera precision.** The camera's body-local position reaches the shaders as one global per body and frame, float32
  hi + lo parts (`earth_terrain_camera_hi/lo`, `moon_terrain_camera_hi/lo`); each node's centre likewise per instance
  (`vertex_offset`, `vertex_offset_lo`, set once at upload). The shader forms (hi_c - hi_n) + (lo_c - lo_n): float32
  precision relative to the camera-node distance, 6400 km from the centre (test: relative error < 3e-7).
- **Loading.** Builds run on `WorkerThreadPool` (`MAX_BUILDS_IN_FLIGHT` 8 [E], ~20-40 ms of GDScript each), coarsest
  and nearest first; uploads (mesh + crop textures + water mask, ~0.3-1 ms) take `UPLOAD_BUDGET_MS` 2 ms [E] of main
  thread per frame (at least one, at most `UPLOADS_PER_FRAME` 16). Dropped subtrees leave the tree at once (hidden)
  and release their resources within `RELEASE_BUDGET_MS` 1 ms per frame (at least 16): a jump between sites drops
  ~2000 nodes. `POOL_MAX` 4096 MeshInstance3Ds are recycled. Godot's threaded culling is off
  (`rendering/limits/spatial_indexer/threaded_cull_minimum_instances` = 1000000 in project.godot): it posts one slice
  per pool thread and waits for all of them, so behind running terrain builds the render thread stalled 15-30 ms.
- **Imagery** (pipeline `tools/data_pipeline/imagery.py`, `config/imagery.toml`; tiles EPSG:4326 TMS, 256 px JPEG):
  NAIP 0.6 m (Planetary Computer; z14-17, `fade_m` 500 m) - Starbase 2022 (4 scenes), Cape 2021 (7 scenes), shifted
  NAD83 -> ITRF; own Sentinel-2 L2A 2025 clear-pixel median composite 10 m (z8-13, `fade_m` 5 km) - 72 scenes per
  area; 417 MB in all. Colour fit (no colour step between the sources and the global day texture): Sentinel-2 -> day
  texture with one luminance gain + offset (Starbase 0.796 / -26.3, Cape 0.659 / -0.75), NAIP -> its area's
  Sentinel-2 composite per channel at 10 m (Starbase gains [0.566, 0.400, 0.347], Cape [0.600, 0.628, 0.582]). No data
  inside a box (the Atlantic beyond the Cape DOQQs, Sentinel-2 composite gaps) is filled before writing, in final
  colours: NAIP from its area's fitted Sentinel-2 composite, Sentinel-2 from the day texture; every tile of a box is
  written (Cape NAIP 11306 tiles, Starbase NAIP 6379, S2 10543 / 10276), lower levels average valid pixels only; the
  run logs the no-data pixels left inside each box (0 for all four). Remaining dark sea inside the S2 boxes is data,
  not holes (the day texture's open ocean is near-black, the fit pulls the S2 sea onto it); rectangular scene / MGRS
  brightness steps over open water remain (no water mask in the pipeline). `fetch.py --retile imagery` refits and
  retiles from the cached clips without network; `--workers N` runs on all cores (bit-identical to serial). In the
  renderer each node gets a fine crop and the next coarser source as base (`ImagerySource.crop`, <= 3 x 3 tiles, cut
  to the node box + 2 px and converted to RGBA8 with mipmaps on the worker); the shader blends day texture -> base ->
  fine with per-fragment coverage fades of width max(`fade_m`, camera distance x 0.05 rad [E]): a source edge never
  reads as a line from orbit. Consequence: a fine patch smaller than about 0.1 x its viewing distance fades out
  entirely from far away (the base / day texture shows instead).
- **Sea** (`water.gdshaderinc`): where the node's 128 x 128 water mask (the core's water flag) says sea, the geoid
  surface is shaded by a calm-sea model: body colour from the imagery where it covers, else the day texture, lit
  diffusely x (1 - F); Fresnel (Schlick, n = 1.333 [S]) times the reflected sky (the atmosphere model's sky along the
  reflected ray, `docs/subsystems/atmosphere-rendering.md`); sun glint by the Cox-Munk slope pdf (sigma^2 = 0.003 +
  0.00512 U [S], `glint_wind_mps` 5 [E]) with Smith masking-shadowing [S]; four Gerstner normal octaves (60 / 23 / 9 /
  3.5 m [E], slope 0.04, Q = 0.5), phases computed in double at the camera (`WaterWaves`, global
  `water_wave_phases`: no float32 time in the shader), faded out between 10 and 20 km distance and per octave where it
  aliases; the resolved slope variance is taken out of the Cox-Munk sigma^2. The F of the sky reflection and the
  diffuse part is the mean Fresnel over the unresolved slopes (Bruneton, Neyret & Holzschuch 2010 fit,
  sigma_v = sqrt(sigma^2 / 2), Schlick's r0 kept; = Schlick at sigma 0): ~1/2 at grazing view and a sharp sea horizon
  at the dip. The wave frame re-anchors (a pattern jump) after 1 deg only while the camera is above the 20 km fade
  distance; lower down only after 10 deg [E]. Bodies without water (the Moon) build no water mask. Geometry stays on
  the geoid (a mission's sea state rescales the wave set: `docs/subsystems/sea-state.md`). The Sun light on the sea,
  the glint and the reflected sky are the transmitted ones, so the far sea fades into the horizon sky.
- **Clouds** (Task 9, `weather-clouds.md` "Light"): `terrain_earth.gdshader` includes `clouds.gdshaderinc` and
  `cloud_light.gdshaderinc` inside `CLOUD_LIGHT_BEGIN` / `_END` blocks (active only while `cloud_enabled`): the Sun
  through the cloud shadow maps, the Sun's diffuse light and the sky light through the column, the sea's reflected sky
  under the deck. Its ~15 extra global samplers per node need `rendering_device/d3d12/max_resource_descriptors`
  262144 in project.godot.
- **Moon**: the same quadtree on the lunar terrain (LDEM_16 everywhere, the 20 m site tiles), albedo from the global
  colour map, `terrain_moon.gdshader` (Lommel-Seeliger + Earth shadow + earthshine + terrain shadows). Leaf level 12, 10.4 m spacing at
  the IM-2 site and at a tile edge (mesh vs core 0.004 / 0.005 m); the far side refines to the curvature limit (373 m,
  level 7). The lander's CG is drawn 18.11 m above the drawn surface = the session's `altitude_terrain_m`, 0.8 m from
  the site.
  **Terrain shadows:** the node builder bakes the core's horizon (`StarsimCore.horizon_elevations`, banded bake, observer =
  the vertex on the terrain surface) for 16 azimuths per grid vertex into a half-float RGBAH image of 260 x 65 texels
  (texel `a * 4 + c`, row `b` = vertex (a, b), azimuths 4c .. 4c+3; the core's order reinterpreted, no copy; half floats
  0.004 deg at 5 deg, range +-pi/2 so no clamp). CUSTOM2 carries the vertex's grid position (a, b) (the Moon has no
  water mask), skirts take their edge vertex's. The shader filters bilinearly itself (`texelFetch`, so shared edges read
  identical texels) and takes the horizon in the light's azimuth (linear between the two sectors); the visible fraction
  is the exact circular segment of the disc (angular radius r / distance) above that straight horizon; it multiplies
  the Sun term, the umbra glow and (own horizon, the Earth's disc) the earthshine. Local east / north come from the
  Moon-fixed position (pole +Y, any east at a pole). Nodes with spacing > `HORIZON_MAX_SPACING_M` (1000 m, level <= 5)
  build none (`horizon_enabled` false: lit as before). Bake: 32 ms per leaf node (level 12), 43-46 ms at level 7, 0.06 s
  at 666 m, 0.2 s at 1.3 km, 0.8-1.4 s at 2.7 km and coarser (hence the cap; one worker, idle machine). Lunar site load
  (IM-2, worker threads, headless, to idle): 4.9 s without horizons, 40 s with horizons on all levels, 11.3 s with the cap.
  Upload: one 135 KB ImageTexture per node inside the existing upload budget. GPU (1600 x 900, `--gpu_time`, median):
  15 km view over rough terrain 0.724 -> 1.018 ms (+0.29 ms, fragment ALU / fetch of the horizon test; per-vertex
  pre-computation was slower), 2 m ground view 0.608 -> 0.649 ms, stereo mock 2.93 -> 3.20 ms.

## Code

- `game/world/terrain/`: `cube_sphere.gd`, `height_source.gd`, `imagery_source.gd`, `terrain_node_builder.gd`
  (worker-safe), `terrain_quadtree.gd`, `water_waves.gd`.
- Shaders: `terrain_earth.gdshader`, `terrain_moon.gdshader`, `terrain_morph.gdshaderinc`, `water.gdshaderinc`.
- Imagery pipeline: `tools/data_pipeline/imagery.py`, `config/imagery.toml`.
- Performance run: `game/tests/terrain_perf.gd` (GPU).

## Standing decisions

- The renderer draws exactly the physics' surface via `surface_samples`; cells sag <= 1 cm (curvature limit).
- `LOD_FACTOR` 2.5 (final, performance vs morph band); single-threaded culling; upload / release budgets as above.
- Desktop proxy gate [E] for 90 Hz VR: p99 <= 8 ms, max <= 16 ms outside the jump frame.
- Camera position reaches the shaders as global hi/lo float32 parts, not per-instance parameters.

## Validation

- Camera-precision test: relative error < 3e-7. Quadtree test: morph start < end for every level >= 2 on both bodies.
- Moon: mesh vs core 0.004 / 0.005 m at the IM-2 site / tile edge; lander CG 18.11 m above the drawn surface.
- Performance (`terrain_perf.gd`, RTX 4090, D3D12, 2560 x 1440, vsync off, the editor binary the game runs with, clock
  at warp 1): descent above OLP-1 400 km -> 2 m in 60 s, 360 deg pan at 2 m on the plateau 180 m east of the mount,
  360 deg pan 50 m above the pad, jump to LC-39A. Final (K 2.5), frame ms p50 / p99 / max: descent 3.3 / 6.3 / 10.9,
  pan low 4.3 / 6.9 / 8.9, pan high 4.3 / 6.0 / 9.0, Cape 4.4 / 8.1 / 11.2, all p99 7.1 / max 11.2; VRAM 2.3 GB;
  gate PASS (a repeat run: p99 7.26 / max 12.27 ms, Cape p99 8.25 ms while loading); up to 1875 drawn leaves, full
  load at the Cape ~8 s, jump frame 8.7 ms. Terrain main-thread time: worst frame per second ~2.6-2.9 ms in steady
  views with ~2500 nodes in the tree, up to ~7 ms in frames with uploads. GPU time stays below ~2 ms: CPU-bound.

## Diagnostics

- `res://tests/terrain_perf.gd` (per-second console output stalled the main thread by up to ~22 ms per print burst;
  lines are now printed at the end of the run). Sporadic 60-120 ms process spikes seen earlier were traced to that.
- A fresh checkout needs one `godot --headless --path game --import` before `--script` runs (tests) resolve the
  `class_name` terrain scripts.

## Open points and known limits

- Open for the user, unverified: NAIP suggests the real OLP-1 mount stands some tens of metres WSW of our [E] site
  coordinates (`launch_sites.toml`); it affects the physics' site position, not the renderer.
- Terrain-load draw stall (~1 run in 3 shows one 11-17 ms frame inside the RenderingServer draw; engine-internal,
  worker-pool dependent; see CLAUDE.md known issues).
- Moon shadows: shadow edges are as sharp as the grid (vertex spacing, 10 m at the sites, 333 m on the far side; the
  bake's banded interpolation is within 0.3-1 deg worst case of a single-ray march); the horizon is taken in the disc
  centre's azimuth; nodes coarser than 1 km and nodes without terrain data have none; no shadows of vehicles or of
  structures; the bake dominates a lunar site's first load (see above); the first triangles at a pole use a
  consistent but arbitrary north.
- A fine imagery patch smaller than ~0.1 x viewing distance fades out from far away; S2 brightness steps over open
  water remain.
