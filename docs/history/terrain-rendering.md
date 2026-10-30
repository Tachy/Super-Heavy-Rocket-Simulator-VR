# Terrain rendering (M8 Plan B) - history

Verbatim record of this subsystem's notes as they stood in `docs/subsystems.md` on 2026-10-25, before the split. Current state: `docs/subsystems/terrain-rendering.md`. Kept for the reasons behind decisions, superseded numbers and earlier approaches; read only when that history is needed.

Plan `docs/superpowers/archive/plans/2026-10-24-m8b-terrain-rendering.md`, spec
`docs/superpowers/archive/specs/2026-10-23-m8-terrain-tiles-design.md` (Plan B). The renderer draws exactly the physics'
surface (`StarsimCore.surface_samples` on the shared terrain cache) with imagery on top; it replaced the interim
`GroundPatch`. Code: `game/world/terrain/` (`cube_sphere.gd`, `height_source.gd`, `imagery_source.gd`,
`terrain_node_builder.gd`, `terrain_quadtree.gd`, `water_waves.gd`), shaders `terrain_earth.gdshader`,
`terrain_moon.gdshader`, `terrain_morph.gdshaderinc`, `water.gdshaderinc`.

- **Tiling.** Cube sphere (6 faces, tangent-warped), quadtree keys `Vector4i(face, level, i, j)`; the 24 level-1 nodes
  are built at setup and never freed. A node is a 64 x 64-cell grid (65 x 65 vertices) plus a skirt ring 2 cells deep
  (hides cracks between neighbours one level apart), vertices float32 relative to the node centre (float64 body-fixed).
  Heights: one `surface_samples` call per node on a 67 x 67 lattice (one ring outside for seamless central-difference
  normals); without data/ the flat rule (site height near the sites, the ellipsoid / geoid elsewhere).
- **LOD.** A node splits when the camera is closer to its bounding sphere than `LOD_FACTOR` x its size and its mesh
  spacing is coarser than the finest data under the whole lattice (`surface_finest_resolution_m`) or, where data is
  coarser or absent, than the curvature limit sqrt(8 R x 1 cm) (714 m Earth, 373 m Moon: cells sag <= 1 cm, so
  vehicles stand on the drawn ground); it merges beyond `MERGE_FACTOR` 1.15 x that distance; `MAX_LEVEL` 21. A parent
  stays drawn until all four children are uploaded (the drawn leaves always tile the sphere). At OLP-1 with data the
  leaf under the camera reaches level 18 (1 m 3DEP cells).
- **Morph.** Odd vertices carry the offset to the parent level's surface (CUSTOM0) and the parent-level normal
  (CUSTOM3); the shader blends both by the camera distance between `morph_start_m` and `morph_end_m`, set per node from
  `LOD_FACTOR` / `MERGE_FACTOR` so that a node is unmorphed whenever its children appear or vanish and a child is fully
  morphed at that moment (start = 1.15 K s + 2 r, end = 0.97 K x parent size; start < end for every level >= 2 on both
  bodies). The level-1 roots never morph. A node drawn for the first time also starts fully morphed (`morph_floor` 1 ->
  0 over `MORPH_IN_S` 0.5 s [E]): children whose upload came late (the camera already inside their morph range, e.g.
  a fast descent) slide in instead of popping; a parent shown again after a merge gets no floor.
- **Camera precision.** The camera's body-local position reaches the shaders as one global per body and frame,
  float32 hi + lo parts (`earth_terrain_camera_hi/lo`, `moon_terrain_camera_hi/lo`); each node's centre likewise per
  instance (`vertex_offset`, `vertex_offset_lo`, set once at upload). The shader forms (hi_c - hi_n) + (lo_c - lo_n):
  float32 precision relative to the camera-node distance, 6400 km from the centre (test: relative error < 3e-7). Before
  Task 9 every drawn node got its camera position by `set_instance_shader_parameter` each frame (~2400 calls, ~3.6 ms).
- **Loading.** Builds run on `WorkerThreadPool` (`MAX_BUILDS_IN_FLIGHT` 8 [E], ~20-40 ms of GDScript each), coarsest
  and nearest first; uploads (mesh + crop textures + water mask, ~0.3-1 ms) take `UPLOAD_BUDGET_MS` 2 ms [E] of main
  thread per frame (at least one, at most `UPLOADS_PER_FRAME` 16). Dropped subtrees leave the tree at once (hidden) and
  release their resources within `RELEASE_BUDGET_MS` 1 ms per frame (at least 16): a jump between sites drops ~2000
  nodes. `POOL_MAX` 4096 MeshInstance3Ds are recycled. Godot's threaded culling is off
  (`rendering/limits/spatial_indexer/threaded_cull_minimum_instances` = 1000000 in project.godot): it posts one slice
  per pool thread and waits for all of them, so behind running terrain builds the render thread stalled 15-30 ms
  (single-threaded culling of ~2000 instances costs well under a millisecond).
- **Imagery** (Task 4, pipeline `tools/data_pipeline/imagery.py`, `config/imagery.toml`; tiles EPSG:4326 TMS, 256 px
  JPEG): NAIP 0.6 m (Planetary Computer; z14-17, `fade_m` 500 m) - Starbase 2022 (4 scenes, 6375 tiles), Cape 2021
  (7 scenes, 9359 tiles), shifted NAD83 -> ITRF; own Sentinel-2 L2A 2025 clear-pixel median composite 10 m (z8-13,
  `fade_m` 5 km) - 72 scenes per area, 10276 / 10398 tiles; 417 MB in all. Colour fit (no colour step between the
  sources and the global day texture): Sentinel-2 -> day texture with one luminance gain + offset (Starbase 0.796 /
  -26.3, Cape 0.659 / -0.75; a ~7 km texel cannot support per-channel fits), NAIP -> its area's Sentinel-2 composite per
  channel at 10 m (Starbase gains [0.566, 0.400, 0.347], Cape [0.600, 0.628, 0.582]; z14 aggregate means within 1.6 DN).
  No data inside a box (the Atlantic beyond the Cape DOQQs, Sentinel-2 composite gaps such as the Cape NE corner) is
  filled before writing, in final colours: NAIP from its area's fitted Sentinel-2 composite, Sentinel-2 from the day
  texture; every tile of a box is written (Cape NAIP 11306 tiles, Starbase NAIP 6379, S2 10543 / 10276), lower levels
  average valid pixels only; the run logs the no-data pixels left inside each box (0 for all four). Remaining dark sea
  inside the S2 boxes is data, not holes: the day texture's open ocean is near-black (~(0-2, 0-5, 7-20) DN), the fit
  pulls the S2 sea onto it, and darker scenes clip to ~1 DN; rectangular scene / MGRS brightness steps over open water
  remain (no water mask in the pipeline).
  `fetch.py --retile imagery` refits and retiles from the cached clips without network; the pipeline runs on all cores
  (`--workers N`, bit-identical to serial): retile 110 s with 24 workers vs 539 s serial. In the renderer each node gets
  a fine crop and the next coarser source as base (`ImagerySource.crop`, <= 3 x 3 tiles, cut to the node box + 2 px and
  converted to RGBA8 with mipmaps on the worker); the shader blends day texture -> base -> fine with per-fragment
  coverage fades of width max(`fade_m`, camera distance x 0.05 rad [E]): a source edge never reads as a line from orbit
  (20 km wide at 400 km). Consequence: a fine patch smaller than about 0.1 x its viewing distance fades out entirely
  from far away (the base / day texture shows instead).
- **Sea** (Task 7, `water.gdshaderinc`): where the node's 128 x 128 water mask (the core's water flag) says sea, the
  geoid surface is shaded by a calm-sea model: body colour from the imagery where it covers, else the day texture
  (shallow banks visible from orbit), lit diffusely x (1 - F); Fresnel (Schlick, n = 1.333 [S]) times the reflected sky
  (since M8 atmosphere: the model's sky along the reflected ray, "Atmosphere rendering"; the former placeholder
  `sky_radiance` is gone); sun glint by the Cox-Munk slope pdf (sigma^2 = 0.003 +
  0.00512 U [S], `glint_wind_mps` 5 [E]) with Smith masking-shadowing [S]; four Gerstner normal octaves (60 / 23 / 9 /
  3.5 m [E], slope 0.04, Q = 0.5), phases computed in double at the camera (`WaterWaves`, global
  `water_wave_phases`: no float32 time in the shader), faded out between 10 and 20 km distance and per octave where it
  aliases; the resolved slope variance is taken out of the Cox-Munk sigma^2. Since 2026-10-25 the F of the sky
  reflection and the diffuse part is the mean Fresnel over those unresolved slopes (Bruneton, Neyret & Holzschuch 2010
  fit, sigma_v = sqrt(sigma^2 / 2), Schlick's r0 kept; = Schlick at sigma 0): before, the faded octaves left a flat
  mirror with F -> 1 at grazing view, the sea beyond ~2.5 km (10 m eye) had the luminance of the sky above the horizon
  and the visible horizon sat ~0.17 deg below the geometric dip (a hazy band); now ~1/2 at grazing and a sharp sea
  horizon at the dip. The wave frame re-anchors (a pattern jump)
  after 1 deg only while the camera is above the 20 km fade distance; lower down only after 10 deg [E]. Bodies without
  water (the Moon) build no water mask. Geometry stays on the geoid (a mission's sea state rescales the wave set:
  "Sea state and deck motion", Task 6). The Sun light on the sea, the glint and the reflected sky are the transmitted
  ones, so the far sea fades into the horizon sky (the old grazing "horizon glow" against the black sky is gone).
- **Moon** (Task 8): the same quadtree on the lunar terrain (LDEM_16 everywhere, the 20 m site tiles), albedo from the
  global colour map, `terrain_moon.gdshader` (Lommel-Seeliger + Earth shadow + earthshine). Leaf level 12, 10.4 m
  spacing at the IM-2 site and at a tile edge (mesh vs core 0.004 / 0.005 m); the far side refines to the curvature limit
  (373 m, level 7). The lander's CG is drawn 18.11 m above the drawn surface = the session's `altitude_terrain_m`, 0.8 m
  from the site. No cast shadows (crater rims, polar shadows): a polar site with the Sun ~1 deg high is dark by its
  normals only - a later part.
- **Performance** (Task 9, `game/tests/terrain_perf.gd`, RTX 4090, D3D12, 2560 x 1440, vsync off, the editor binary
  the game runs with, clock at warp 1): descent above OLP-1 400 km -> 2 m in 60 s, 360 deg pan at 2 m on the plateau
  180 m east of the mount, 360 deg pan 50 m above the pad, jump to LC-39A. Desktop proxy gate [E] for 90 Hz VR: p99 <=
  8 ms, max <= 16 ms outside the jump frame.

  | Step | descent p50 / p99 / max | pan low | pan high | Cape | all p99 / max | VRAM |
  |---|---|---|---|---|---|---|
  | before Task 9 (K 3) | 10.1 / 29.3 / 203 | 14.3 / 19.8 / 22.7 | 14.2 / 19.2 / 23.6 | 14.1 / 21.4 / 45.1 | 22.4 / 203 | 4.1 GB |
  | camera global, flat traversal, upload budget (K 3) | 4.7 / 12.3 / 35.6 | 5.5 / 10.2 / 13.4 | 5.6 / 8.7 / 11.8 | 5.9 / 11.7 / 36.2 | 11.3 / 36.2 | 4.0 GB |
  | + single-threaded culling, deferred release, crops cut (K 3) | 4.3 / 8.2 / 13.9 | 5.5 / 9.4 / 14.3 | 5.5 / 8.1 / 10.2 | 5.8 / 10.6 / 13.0 | 9.2 / 14.3 | 2.7 GB |
  | + `LOD_FACTOR` 2.5 (final) | 3.3 / 6.3 / 10.9 | 4.3 / 6.9 / 8.9 | 4.3 / 6.0 / 9.0 | 4.4 / 8.1 / 11.2 | 7.1 / 11.2 | 2.3 GB |

  Final: gate PASS (a repeat run: p99 7.26 / max 12.27 ms, Cape p99 8.25 ms while loading); up to 1875 drawn leaves (was ~2450 at K 3), full load at the Cape ~8 s (was ~11 s), jump frame
  8.7 ms (was 55 ms). Terrain main-thread time: worst frame per second ~2.6-2.9 ms in steady views with ~2500 nodes in
  the tree, up to ~7 ms in frames with uploads (GDScript ~1 us per
  visited node; the traversal is flat, children that stay leaves are handled inline, visibility touches only changes).
  K 2.0 measured p99 5.3 ms but narrows the morph band to ~0.16 node sizes (start 1.15 K s + 2 r against end 1.94 K s):
  morphing becomes almost a pop; K 2.5 keeps ~0.55 node sizes (smallest margin of start below end over all levels
  and both bodies 0.05 of the end, at K 3 0.11; quadtree test). GPU time stays below ~2 ms: the frame is CPU-bound.
  Single sporadic process spikes of 60-120 ms appeared in some runs (one of three K 2.5 runs failed the max gate
  with a 124 ms spike); traced in M8 atmosphere Task 10 ("Atmosphere rendering", Performance): the run's own
  per-second console output stalled the main thread in its "process" part (measured up to ~22 ms per print burst;
  the lines are now printed at the end of the run) - the likely cause of these spikes as well.
- **Fresh checkout:** the terrain scripts are `class_name` classes; one `godot --headless --path game --import` is
  needed before `--script` runs (tests) can resolve them.
- **Known issue (open for the user, unverified):** NAIP suggests the real OLP-1 mount stands some tens of metres WSW of
  our [E] site coordinates (`launch_sites.toml`); it affects the physics' site position, not the renderer.
- **Moon without cast shadows (superseded by the horizon maps, 2026-10-30):** "Moon: no cast shadows (crater rims,
  polar shadows): a polar site with the Sun ~1 deg high is dark by its normals only - a later part."
