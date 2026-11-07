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
  elsewhere). On the Moon with the procedural detail on (fidelity flag `moon_surface_detail`, `moon-surface.md`; the
  builder sees it as `surface_finest_resolution_m` < `surface_data_resolution_m`) the heights are
  `surface_samples_detail` band-limited to the level's nominal vertex spacing (`TerrainNodeBuilder.level_spacing_m`,
  R pi/2 / 2^L / 64, one value per level so neighbours of a level match on their edge; rocks left out): the area mean
  over a vertex cell, so nothing finer than the mesh aliases and a parent is the mean of its children. A second call
  samples the parent level's band (2 x the spacing) on the 35 x 35 even lattice points (one parent cell outside) for
  the morph target. Detail off: `surface_samples` exactly as before (builder output bit-identical, SHA-256 of every
  array before / after the change).
- **LOD.** A node splits when the camera is closer to its bounding sphere than `LOD_FACTOR` x its size and its mesh
  spacing is coarser than the finest data under the whole lattice (`surface_finest_resolution_m`) or, where data is
  coarser or absent, than the curvature limit sqrt(8 R x 1 cm) (714 m Earth, 373 m Moon: cells sag <= 1 cm, so
  vehicles stand on the drawn ground); it merges beyond `MERGE_FACTOR` 1.15 x that distance; `MAX_LEVEL` 22;
  `LOD_FACTOR` 2.5. A parent stays drawn until all four children are uploaded (the drawn leaves always tile the
  sphere). At Pad 2 with data the leaf under the camera reaches level 18 (1 m 3DEP cells). This is the screen-size
  (mipmap) rule: height and lateral distance enter as one distance, and a cell projects to at most 1 / (64 K) =
  6.3 mrad when its node splits, so drawn cells span ~4.5-9 px of the VR target (0.69 mrad per px at the centre,
  3534 px per eye) and ~4-8 px of a 1600 px desktop view at 70 deg: no node is split finer than that pixel footprint.
  With the Moon's procedural detail the data limit is the detail's finest wavelength (0.01 m), so the Moon refines to
  level 22 (1.02 cm spacing, `MAX_LEVEL`) under a camera ~2 m above the ground; detail off it stops at the LOLA level
  as before.
- **Morph.** Odd vertices carry the offset to the parent level's surface (CUSTOM0) and the parent-level normal
  (CUSTOM3); the shader blends both by the camera distance between `morph_start_m` and `morph_end_m`, set per node from
  `LOD_FACTOR` / `MERGE_FACTOR` so that a node is unmorphed whenever its children appear or vanish and a child is
  fully morphed at that moment (start = 1.15 K s + 2 r, end = 0.97 K x parent size; start < end for every level >= 2
  on both bodies; at K 2.5 the band is ~0.55 node sizes, smallest margin of start below end 0.05 of the end; quadtree
  test). The level-1 roots never morph. A node drawn for the first time also starts fully morphed (`morph_floor` 1 ->
  0 over `MORPH_IN_S` 0.5 s [E]): children whose upload came late slide in instead of popping; a parent shown again
  after a merge gets no floor. With the Moon's procedural detail the even vertices morph too, to the parent band's
  height at the same point (the parent's own vertex there), and the parent-level normals of the even vertices are the
  parent's central differences over that lattice: fully morphed a node is its parent's mesh (within float32 rounding),
  unmorphed it carries the band between its and its parent's spacing.
  The parent band's heights are not sampled again: the parent keeps its band-limited lattice heights
  (`band_heights`, 36 KB) while it can split and its children are not all uploaded, and each child reads its 35 x 35
  even points from them (`TerrainNodeBuilder.parent_quadrant`; `lattice_coordinate` is dyadic, so they are the same
  points bit for bit; a re-split after a merge samples them).
- **Poles.** Heights are sampled at longitude 0 for a lattice point on a pole (`_lattice`'s `sample_lons`; uv and the
  horizon bake keep the node centre's longitude): a latitude-longitude grid's polar row holds different heights per
  longitude, and with each node's own centre longitude the four nodes around a pole drew their shared corner up to
  ~110 m apart (LOLA, detail on and off; found by `moon_surface_check.gd`'s pole seam check, fix round 1).
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
  With the Moon's procedural detail (a tree that uploaded a band-limited node) builds go camera-first: ordered by the
  distance from the camera's ground point (below the camera on the parent's surface radius) to the node, less
  `CAMERA_FIRST_HALF_DIAGONAL` 0.75 [D] of its nominal size, over that size, then the level - so the chain of nodes
  under the camera builds level after level ahead of the wide rings; `MAX_BUILDS_IN_FLIGHT_DETAIL` 16 [E] at once as
  high-priority pool tasks (low-priority tasks share `low_priority_thread_ratio` 0.3 of the pool: 7 of 24 threads;
  20 measured 24.4 s against 26.4 s for a 2 m spawn, not worth the extra contention). `surface_samples_detail` runs in
  parallel in the core (`DetailedLunarTerrain::heights_limited_m`: the threads of all running batches share
  hardware threads - 2, so a lone batch spreads over ~17 threads and many concurrent ones stay near one each; bit for
  bit the serial loop). Without detail the order and the cap are as before (level first, 8 low-priority tasks).
- **Imagery** (pipeline `tools/data_pipeline/imagery.py`, `config/imagery.toml`; tiles EPSG:4326 TMS, 256 px JPEG):
  NAIP 0.6 m (Planetary Computer; z14-17, `fade_m` 500 m) - Starbase 2022 (4 scenes), Cape 2021 (7 scenes), shifted
  NAD83 -> ITRF; own Sentinel-2 L2A 2025 clear-pixel median composite 10 m (z8-13, `fade_m` 5 km) - 72 scenes per
  area; 275 MB in all (Starbase NAIP 6379 tiles 37 MB, Cape NAIP 11306 / 79 MB, S2 10276 / 76 MB and 10543 / 83 MB).
- **Surface albedo** (Task 15 of the Moon surface plan, `tools/data_pipeline/earth_albedo.py`, `config/earth_albedo.toml`):
  every Earth ground texel - the day texture and all tiles - holds the surface reflectance per channel (R, G, B ~
  MODIS bands 1 / 4 / 3 = Sentinel-2 B04 / B03 / B02), sRGB-encoded (IEC 61966-2-1); the shaders' only decode is the
  GPU's sRGB decode of their `source_color` samplers (`terrain_earth`, `earth_surface`, `sky`), the BRDF / atmosphere
  shading is unchanged. Per level:
  - Global (`data/textures/earth_day.jpg`, 5400 x 2700, written by `prepare.py`): land from a MODIS MCD43A4 v061 NBAR
    climatology (`fetch.py earth_albedo`: every land tile at the COG's 3.7 km overview, per pixel the median over the
    mid-month products Jan-Jun and Dec 2025 plus Jul-Nov 2024 - the Planetary Computer lacks 2025-07..11 -, at least 3
    valid, averaged into the texture's pixels, taken where ETOPO 2022 + EGM2008 say the pixel is at least half land
    (the core's water rule, `land_fraction`): 4.14 M of 14.58 M pixels; the 0.83 M water pixels with MCD43A4 values -
    cloud- and glint-bright offshore spots - are left out). Elsewhere (sea, sea ice, polar night) the
    Blue Marble Next Generation (Dec 2004) through per-channel monotone tables count -> reflectance, fitted to MCD43A4
    NBAR of 2004-12-16 (279 tiles, 3.0 M pixels; luminance error per pixel median 10 %, 90th percentile 59 % - snow
    and season). Inside each Sentinel-2 box the land texture pixels carry the Sentinel-2 reflectance of the land pixels
    averaged into them, and outside the box the edge ratio Sentinel-2 / MODIS of the land pixels fades out over 50 km
    on land pixels only (`EMBED_FEATHER_M` [E]): the two calibrated land products differ there (Sentinel-2 +9.5 % at
    Starbase, -10 % at the Cape against the climatology), and the embedding spreads that over the feather instead of
    drawing it as a step where the renderer fades between the levels. The sea is not embedded pixel by pixel (the land
    feather is justified by the land disagreement): sea pixels inside a box take the box's calibrated open-water
    level (next item) plus the mean brightness of their water pixels above the open-water pedestal (below). 2.5 MB
    (JPEG quality 95).
  - Calibrated sea (follow-up U-3, `earth_albedo.open_water_levels`, `SEA_FEATHER_M`): the Blue Marble sea through the
    land tables is one nearly constant colour worldwide (luminance 0.019: [0.0087, 0.0215, 0.0283]), and it is not a
    calibrated product. Inside each Sentinel-2 box the sea texels take the median (`OPEN_WATER_LEVEL_PERCENTILE` [E]) of
    the L2A reflectance over open water (renderer's water rule, > 2 km from non-water, per channel; Starbase [0.033,
    0.042, 0.049] = luminance 0.040, Cape [0.0054, 0.0133, 0.0281] = 0.0127: the turbid Gulf against the clear Atlantic),
    fading back to the global sea with a cosine over 100 km outside the box (`SEA_FEATHER_M` [E]; the Gulf east of
    -94.8 deg and the Atlantic beyond ~300 km of the Cape keep 0.019). A single global open-water value cannot be
    sourced from the two scenes (a factor 3 apart; the global 0.019 lies between them), so the calibration is
    regional: where we have L2A the sea is L2A. Open-water tile pixels read 0.0404 at the Gulf (L2A median 0.0404)
    and 0.0125 at the Cape (0.0127); the offshore z13 mean over 330 sampled pixels is 0.0404. Rendered, the sea at
    the pad view and a 5 km Gulf view changes by < 1 % in luminance (Sun 87 deg, 18:30 UTC: 0.2318 -> 0.2284 and
    0.2626 -> 0.2620 mean linear luminance of the sea patch): the sea shader's reflected sky dominates the body colour
    at those angles. L2A includes sky glint (Sen2Cor does not remove it), which the sea shader adds again - the
    calibrated body colour counts it twice at a few percent of the sea's radiance; not separated.
  - Water in the tiles (`earth_albedo.WaterMask`, the renderer's sea rule: orthometric height <= 0.05 m from the 1 m
    local grid where it covers, else the GLO-30 regional grid, minus EGM2008): a water pixel of a Sentinel-2 or NAIP
    tile takes the global level's sea (`sea_only`: the day texture with land pixels replaced by the nearest sea pixel, so
    the 7 km land pixels do not bleed into the sea) plus the imagery's brightness above the open-water pedestal
    (`open_water_pedestal`: per channel the 90th percentile of the Sentinel-2 reflectance over water > 2 km from any
    non-water pixel, measured per area, written to meta.toml: Starbase [0.045, 0.066, 0.063], Cape [0.014, 0.035,
    0.039]). The pedestal is what Sen2Cor L2A reports over the sea of the scene (turbidity, glint, scene steps), so
    open water reads the calibrated sea, while shallow bottoms, tidal flats and surf keep their brightness: the Boca
    Chica wind-tidal flats are flagged water by the rule (and drawn as water by the game) but are bright bottom
    (L2A ~0.16), so a plain sea replacement would turn them deep-sea dark.
  - Sentinel-2: the L2A surface reflectance itself, no fit: the TCI median decoded with its measured linear
    quantisation (TCI = floor(1 + DN x 254 / 2500), DN = reflectance x 1e4: fit 0.101604 DN + 0.497, rms 0.4 counts on
    S2B_14RNN_20251116), and the pixels saturated in the TCI (reflectance >= 0.25: beaches, flats, roofs; Starbase 1.54 M
    in 182 of 400 blocks, Cape 1.21 M in 195) recomputed as the median of the B04 / B03 / B02 bands with the same
    clear-pixel rule (`data/raw/imagery/<id>/reflectance.tif`, uint16).
  - NAIP (not calibrated): per channel a monotone table count -> reflectance fitted to the area's Sentinel-2
    reflectance on a 10 m lattice, then a local gain = Sentinel-2 / tabled NAIP over a Gaussian sliding window
    (sigma `NAIP_RATIO_SIGMA_M` 50 m [E], NAIP rendered at 2.5 m and tabled before the 10 m means, gain clipped to
    0.5..2 [E]; median 1.00, 5-95 % 0.78-1.43 Starbase / 0.67-1.23 Cape, clipped 7.6 % / 3.8 % of the cells - ground
    changed between NAIP 2021/22 and Sentinel-2 2025), so bright local flats keep their brightness.
  No data inside a box (the Atlantic beyond the Cape DOQQs, Sentinel-2 composite gaps) is filled before writing, in
  reflectance: NAIP from its area's Sentinel-2 reflectance, Sentinel-2 from the day texture; every tile of a box is
  written, lower levels are reflectance (linear) means of the valid pixels; the run logs the no-data pixels left inside
  each box (0 for all four). `fetch.py --retile imagery` refits and retiles from the cached clips / composites / reflectance files
  without network; `--workers N` runs on all cores (bit-identical to serial).
  Validation (luminance of the reflectance, `screenshots/earth_albedo/task15/` and the task reports): the six spots
  within +-25 % of their references at every level the game draws there - global 12 km box vs MCD43A4 NBAR
  (Jul 2024 - Jun 2025 median): Starbase pad +3 %, Boca Chica beach 0 %, Cape LC-39A +2 %, Sahara 25 N 25 E -2 %,
  Amazon 5 S 65 W +4 %, Kansas 38.5 N 98.5 W 0 %; Sentinel-2 z13 and NAIP z17 200 m boxes vs Sentinel-2 L2A band
  medians: pad -1 % / +4 %, beach +8 % / +10 % (the spot lies on the tidal flats: 92.6 % of its 460 z13 pixels are water by the renderer's rule,
  and a water pixel reads the global level there (~0.084, a 12 km mean with land and bay) plus the imagery's excess over the
  open-water pedestal (luminance 0.062): the global level is ~0.02 above the pedestal, so a spot of flat pixels reads above
  L2A, by (global - pedestal) / L2A = +8..14 %, not below it; z13 water pixels 0.180, land pixels 0.189, all 0.1805 against
  L2A 0.1666), Cape +1 % / +6 %. Level steps (mean over the transition): NAIP vs Sentinel-2 over the NAIP boxes
  +0.2 % Starbase / -2.5 % Cape (median 250 m cell 1.1 % / 1.4 %, 90th percentile 3 % / 17 % - dark Cape sea cells),
  Sentinel-2 vs the day texture in the 10 km band inside the box edge +4.6 % / +2.1 % over all cells (land +4.9 % /
  +2.4 %, sea -0.8 % / +1.0 %). The plan's limit for a level step is 5 %: the Starbase land step (+4.9 %) is 0.1 point
  under it, i.e. close to the limit (the product disagreement of 10-20 % spread over the 50 km feather), and the Cape
  NAIP 90th percentile (17 %: single dark sea cells, median 1.4 %) is beyond it.
  Which checks are independent: (a) independent of the chain - Sentinel-2 L2A (ESA Sen2Cor, own median composite) vs
  MODIS MCD43A4 NBAR (NASA, native 500 m, the same 12 dates) at 0.005 deg cells of the land pixels, 60 048 / 63 394
  cells: mean ratio +11.6 % Starbase (median cell 1.10, 10-90 % 0.97-1.26, r = 0.80), -9.3 % Cape (median 0.85, 10-90 %
  0.69-1.09, r = 0.82), per channel R / G / B Starbase 1.16 / 1.09 / 1.36, Cape 0.95 / 0.88 / 1.14 (the blue bands
  differ in width and in the aerosol correction); the Sahara / Amazon / Kansas global values against the Dec 2004
  MODIS NBAR through the tables (-2 % / +4 % / 0 % against 0.068 for Kansas: season sensitivity); the TCI decode fit
  (rms 0.4 counts). The two land products differ by 10-20 %, which is the real uncertainty of the level. (b) Self-
  consistency, not independent: the global spots vs MCD43A4 (the texture is built from the same product: checks
  binning, mask, encoding, JPEG), the Sentinel-2 spots vs band medians with the same clear-pixel rule as
  `reflectance.tif`, and every NAIP spot (the NAIP level is the Sentinel-2 reflectance by construction of the 50 m
  gain, which absorbs any NAIP error; its absolute level is that of Sentinel-2). A published surface albedo (field site
  / AERONET) is not used: no free product was found that matches the visible bands at a spot.
  In the renderer each node gets a fine crop and the next coarser source as base (`ImagerySource.crop`, <= 3 x 3
  tiles, cut to the node box + 2 px and converted to RGBA8 with mipmaps on the worker); the shader blends day texture
  -> base -> fine with per-fragment coverage fades of width max(`fade_m`, camera distance x 0.05 rad [E]): a source
  edge never reads as a line from orbit. Consequence: a fine patch smaller than about 0.1 x its viewing distance fades
  out entirely from far away (the base / day texture shows instead).
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
- **Near-field Sun shadows (Earth)** (moon procedural surface plan Task 16, `EarthNearShadow`,
  `game/world/earth_near_shadow.gd`, `earth_near_shadow.gdshaderinc`): the Moon's technique (moon-surface.md "Near-field
  shadows") on the Earth. Godot's directional shadow map of the Sun's DirectionalLight (`World.sun_light`), PSSM 4 splits
  (0.1 / 0.2 / 0.5 of the range, blended), 8192 px atlas with 16-bit depth (128 MB; Task 13b, was 32-bit / 256 MB with the same image), PCSS with the solar disc's angular diameter
  (0.533 deg at 1 AU), soft filter HIGH (LOW, the Moon's, left a grain in the wide penumbrae on the terrain), bias
  0.03 / normal bias 1.0 [E]; one shadow pass for both VR eyes (multiview).
  Range [E]: the shadow of a CASTER_HEIGHT_M 147.9 m caster (the tower head) x 1.25 at the Sun's elevation, at least
  500 m, quantised in 2^(1/4) steps (no split resize every frame), + 4 x the eye's height, at most 4 km; Godot fades the
  last 20 %. Casters: the pad structures (tower lattice near and mid tiers, head, chopsticks, carriage, QD arm, mount,
  hardstand), vehicles, the drone ship (Godot's default); the Earth terrain does not cast (`cast_shadows` false: the pad
  plateau is flat, the relief's own shading is the normal). Receivers: every lit material (structures, vehicles, incl.
  the lattice on itself and the vehicle on the mount) and `terrain_earth.gdshader`, lit only for that
  (`ambient_light_disabled`, `specular_disabled`): the direct Sun inside the map's range goes to light() and is multiplied
  by the map there (D [w + (1 - w) S], Godot's ATTENUATION), everything else (sky light, the clouds' diffuse Sun, the Moon,
  the in-scattered light) stays in EMISSION, unshadowed. The direct Sun is the transmitted one (atmosphere, cloud shadow
  maps) seen through the aerial perspective's transmittance; on the sea its diffuse Sun and glint (water_radiance is
  linear in the Sun's visible fraction: the sea minus the same sea without it). Active with the fidelity setting
  `earth_near_shadows` (`user://settings.cfg` [graphics], default on; the time panel's "Schatten Erde"; render_check
  `--earth_near_shadows=off`), the Moon's near shadows off, the Sun above -1.1 deg and the eye within 30 km of the
  Earth's ground (SurfaceClearance); the shadows keep full strength to 20 km and fade out by a smoothstep of the light's
  `shadow_opacity` between 20 and 30 km (Task 13b: no step in the vehicles' self-shadows; the terrain's 4 km range no
  longer reaches the ground from there); above 30 km the light casts nothing (no cost in orbit). Both shadow classes drive the
  one light: `MoonNearShadow.update` first, `EarthNearShadow.update` only while the Moon's is off; the directional atlas
  goes back to the project's when neither is active.
- **Moon**: the same quadtree on the lunar terrain (LDEM_64 everywhere, the 20 m site tiles), albedo from the global
  colour map, `terrain_moon.gdshader` (Hapke by default, the former Lommel-Seeliger law with the game setting `World.moon_brdf` = "lommel_seeliger" (moon-surface.md, Hapke reflectance) + Earth shadow + earthshine + terrain shadows; with the detail on the regolith detail shading of `moon_detail.gdshaderinc`: residual Hapke mean slope per node spacing, statistical shadows and unresolved rocks, the baked regolith normal maps by stochastic hex tiling - moon-surface.md, Regolith shading). The Moon's rocks are drawn per node as instanced meshes by projected size (`TerrainQuadtree.rock_layer` = `MoonRockLayer`: the rock list built with the node on its worker, MultiMeshInstance3Ds attached at the upload as children of the node's mesh instance, detached at the release; moon-surface.md, Rocks drawn). Per-node uniforms beyond the mesh come through `TerrainQuadtree.node_parameters` (the Moon: `MoonDetailShading.node_parameters`); `terrain_visible_fraction` also returns the light's margin over the horizon. Detail off: leaf level 12, 10.4 m spacing at
  the IM-2 site and at a tile edge (mesh vs core 0.004 / 0.005 m); the far side refines to the curvature limit (373 m;
  level 7, spacing 333 m). The lander's CG is drawn 18.11 m above the drawn surface = the session's `altitude_terrain_m`, 0.8 m from
  the site.
  **Terrain shadows:** the node builder bakes the core's horizon (`StarsimCore.horizon_elevations`, banded bake, observer =
  the vertex on the terrain surface) for 16 azimuths per grid vertex into 4 half-float RGBAH layers of 65 x 65 texels
  (layer c, texel (a, b) = vertex (a, b), azimuths 4c .. 4c+3; half floats 0.004 deg at 5 deg, range +-pi/2 so no clamp;
  `HORIZON_STRIDE` 2 would halve the resolution: 4x less bake time and memory but up to 1 deg shadow-edge error, kept 1).
  The layers become one `Texture2DArray` (filter_linear, texel centres = vertices: hardware bilinear between the
  vertices, shared edges of same-level neighbours read identical texels: the march's `spacing_m` is one value per
  level, R x pi / 2 / 2^level / 64 x `HORIZON_STRIDE` [E], not the node's own longest edge, which differs per node;
  measured 0.00 half-float steps off the face centre at (-60, 40) L7, (35, 45) L4 and across a cube-face edge) and material parameters `horizon_max_rad` (the node's highest horizon
  + its half-float rounding `HORIZON_MAX_REL_MARGIN` 2^-10 relative + `HORIZON_MAX_MARGIN_RAD` 1e-4, conservative at
  any elevation) and `horizon_stride` (from the builder's constant; the row length is `textureSize`). The binding returns
  the values layer-major (`layer_major` flag, a pure reorder in C++), so each layer image is a slice. A pole vertex
  (within `POLE_VERTEX_RAD` 1e-7) is baked in the frame of its neighbour's longitude toward the node centre. CUSTOM2 carries the vertex's grid position (the Moon has no water mask; the vertex stage
  turns it into the map's uv), skirts take their edge vertex's. The shader takes the horizon in the light's azimuth
  (one filtered sample, a second only after a layer's last azimuth; linear between the two sectors); a light whose disc
  lies wholly above `horizon_max_rad` skips every fetch, and a light that cannot reach the fragment (below its normal's
  horizon, the Sun with no umbra glow) skips its horizon. The visible fraction is the exact circular segment of the disc
  (angular radius r / distance) above that straight horizon; it multiplies the Sun term, the umbra glow and (own horizon,
  the Earth's disc) the earthshine. `sun_radius_m` / `earth_radius_m` are set from World (no shader defaults). Local
  east / north come from the Moon-fixed position (pole +Y, any east at a pole). **Horizon owners:** a node bakes its own
  map only down to the measured data's level - the nodes the tree had before the procedural detail: a child bakes when
  its parent owns a map and would split on the LOLA resolution alone (`surface_data_resolution_m`, the `children_bake`
  flag; at IM-2 owners stop at level 12). Deeper nodes draw with that owner's `Texture2DArray` (an ancestor, uploaded
  while any descendant is wanted) and its `horizon_max_rad`; the quadtree sets `horizon_grid_scale` 2^-d and
  `horizon_grid_offset` ((i - i_o 2^d), (j - j_o 2^d)) x 64 x 2^-d (d = level difference, exact dyadic floats), and the
  shader takes the owner texel at CUSTOM2 x scale + offset (the lattices are linear in the face coordinates, so it is
  the same point). With the detail the bake's observers are the LOLA heights (the march is over LOLA; the band-limited
  vertices differ by up to ~10^2 m at coarse levels), so the maps equal the detail-off ones. Detail off every node owns
  its map (as before). All baked levels get a horizon: the lattice
  march's step cap is max(1000 m, spacing / 2) (core), a single-point query keeps 1000 m. Bake and encode per node (one thread,
  idle machine, IM-2, stride 1, 2026-10-30): level 1 109 ms, 2 210, 3 385, 4 662, 5 220, 6 67, 7 54, 8 46, 9-12 38-39
  (the leaf 38 ms, of which the GDScript maximum loop ~5 ms). Levels 1-4 are expensive because their coarse lattice
  marches out to hundreds of km from small first steps (2 % growth: ~260 samples per ray at 3 km spacing, 17.8 M samples
  per node); the step cap bites only beyond ~50 km. Lunar site load (IM-2, 915 nodes, 8 builds in flight, to idle):
  3.7 s without horizons; with them 31.3 s headless (a bare quadtree, no World, no clouds) and ~20 s in a real window
  (render_check, 1600 x 900: loading starts at 5 s, ends at ~25 s; the cloud volume is off there: `cloud_enabled`
  false, no baked set, no bake running). The earlier 9.4 s was measured at stride 2 (4x cheaper at the coarse levels);
  16 builds in flight instead of 8 did not help at stride 2: 9.1 s. GPU (1600 x 900,
  `--gpu_time`, median; the machine's GPU state flips between runs, so before / after are interleaved): 15 km view over
  rough terrain +9 % (0.446 -> 0.488 ms), 2 m ground view +7 % (0.347 -> 0.370), stereo mock +6-8 % (2.85-2.94 ->
  3.10). The cost is the horizon fetch's cache misses (the same shader with every fetch at one fixed texel: +0 %; the
  ALU of atan / asin / segment: < 1 %); per-vertex pre-computation and skipping the earthshine's horizon where the Sun
  dominates were slower.

## Code

- `game/world/terrain/`: `cube_sphere.gd`, `height_source.gd`, `imagery_source.gd`, `terrain_node_builder.gd`
  (worker-safe), `terrain_quadtree.gd`, `water_waves.gd`.
- Shaders: `terrain_earth.gdshader`, `terrain_moon.gdshader`, `terrain_morph.gdshaderinc`, `water.gdshaderinc`,
  `earth_near_shadow.gdshaderinc` (Task 16).
- Earth near-field Sun shadows: `game/world/earth_near_shadow.gd` (`World.earth_near_shadow`); shadow-length probe
  `game/tests/earth_shadow_length.gd` (render_check `--shadow_length`).
- Imagery pipeline: `tools/data_pipeline/imagery.py`, `config/imagery.toml`; surface albedo
  `tools/data_pipeline/earth_albedo.py`, `config/earth_albedo.toml` (tests `test_imagery.py`, `test_earth_albedo.py`).
- Performance run: `game/tests/terrain_perf.gd` (GPU).
- Lunar landing sites (any latitude / longitude, plan `docs/superpowers/plans/2026-11-06-lunar-sites.md`):
  `game/world/terrain/lunar_site_service.gd` (`LunarSiteService`, node `World.lunar_sites`) and `World.switch_lunar_site`.
  - Service: `setup(core, fetcher, parallel = 8)`, `prepare(lat, lon)` (coordinates snapped to hundredths of a degree,
    `snap_deg`: the core's tile key and site id have that granularity but the tile is centred on the exact coordinate, so
    one key must come from one coordinate), signals `progress(fraction)` (bytes of finished + running ranges / bytes to
    fetch), `site_ready(lat, lon)` (the snapped site; `ready` is a Node signal), `failed(report)`; `cancel()`. Cached tiles:
    no request. Missing ranges (`StarsimCore.lunar_site_tile_requests`) download `parallel` at a time, each into its own file
    `data/terrain/sites/<key>.parts/<n>.range` (the fetcher's `.part` is renamed only for a complete answer; a complete range
    survives an interruption and is not fetched again); then `assemble_lunar_site_tile` runs on a worker thread and the
    range files are removed. A new `prepare` / `cancel` cancels the requests and bumps a generation counter: late answers and
    a late assembly are ignored. Height range failures: retried 3 x, a 403 (PDS: too many connections) halves the parallel
    requests, a 429 (LROC CloudFront, Retry-After ~ 1 h) is never retried (`data.lunar_site_rate_limited`), no answer at all
    gives `data.lunar_site_tile_missing`, other statuses `data.lunar_site_download_failed` (game-side codes with texts in
    `[report]`, not in `codes.def`). The albedo is look only: a refused, failed or unassemblable albedo is dropped, the site
    is `ready` with the global albedo (`site_albedo_toml()` = "").
  - `World.switch_lunar_site(lat, lon)` (after `site_ready`; coordinates as given, not snapped): a worker thread runs
    `core.set_lunar_site` (atomic swap), `core.moon_unresolved_shading()` (~1 s) and reads the albedo tile; the main thread
    then frees the Moon quadtree (its builds are waited for, the rock pools freed) and builds a new one with a new rock
    layer on the new site (the six roots are built synchronously, as at start); `lunar_site_switched` /
    `lunar_site_switch_failed(report)`. A newer switch discards an older one; a failed core switch keeps the old terrain.
  - Site albedo: `moon_site_albedo.gdshaderinc`, included by `terrain_moon.gdshader` and `moon_rock.gdshader`: the tile is a
    RELATIVE albedo (R3: `TerrainGridFile` int16, `kind = "albedo"`, value x `dn_scale_m` (1/16384), tile mean 1) read by
    `LunarSiteService.read_albedo` into an RF image; uniforms `site_albedo_texture`, `site_albedo_bounds` (lat / lon min /
    max, rad), `site_albedo_grid` (cylindrical: zero_line, zero_sample, pixels per degree, 1; polar: pole - first line /
    sample, scale_m, 2 north / 3 south; w = 0 off), `site_albedo_blend_m = 5000`. The factor `mix(1, relative, smoothstep(edge
    distance / blend))` multiplies the albedo input only (Hapke stays the one lighting law; no baked shading). Rocks copy
    the uniforms (`COPIED_TEMPLATE_PARAMETERS`).
  - Fetcher additions (`weather_fetcher.gd`): `progress(id, bytes)`, `cancel(id)`, `retry_after_s[id]` (Retry-After seconds
    of a 429 / 503 answer).
  - Diagnostics: `render_check --lunar_site=<lat>,<lon> [--site_albedo=off]` (tiles on disk; a 25 km view at Apollo 17 with and
    without the albedo differs in 17 % of the pixels by > 3 grey levels, max 35). Tests: `lunar_site_service_test.gd`.
- Moon procedural detail checks and cost runs: `game/tests/moon_surface_check.gd` (headless).

## Standing decisions

- The renderer draws exactly the physics' surface via `surface_samples`; cells sag <= 1 cm (curvature limit).
- `LOD_FACTOR` 2.5 (final, performance vs morph band); single-threaded culling; upload / release budgets as above.
- Desktop proxy gate [E] for 90 Hz VR: p99 <= 8 ms, max <= 16 ms outside the jump frame.
- Camera position reaches the shaders as global hi/lo float32 parts, not per-instance parameters.

## Validation

- Earth near-field Sun shadows (Task 16): the Pad B tower's shadow tip (render_check `--shadow_length`, `EarthShadowLength`:
  a nadir view 500 m over the pad, the shadowed ground pixels from the on / off frames at the direct Sun's 50 % contour,
  against the largest (P - O).a + (h - dh) / tan e over the 91 330 casting vertices: the head's roof, 147.9 m; dh the
  terrain at the tip over the site). Hard shadows (map resolution alone, tolerance 2 texels / sin e + 2 image pixels):
  Sun 39.5 deg -1.89 m (tol 6.57), 31.1 deg -2.24 m (7.75), 11.4 deg -4.23 m (9.33). With the solar-disc penumbra (+ half
  its ground length, theta h / sin^2 e): 39.5 deg -3.24 m (8.26), 31.1 deg -4.07 m (10.31), 11.4 deg -12.73 m (26.84).
  The terrain beyond the plateau lies 2.5-6 m over the site at the 11 deg tip (without dh the probe read -17 m). Off
  path: the same deterministic frame as the unshaded shader before Task 16 within 1 pixel of 1 440 000 (8-bit diff 1);
  on: 0.96 % of the pixels change (the shadows). Headless `earth_near_shadow_test.gd`: range rule, activity, setting.
  The soft rows are a plausibility check only (the hard rows are the geometric gate; the 11.4 deg soft prediction moved
  734 to 716 m between PCSS LOW and HIGH because the tip pixel, hence dh, moved). Rerun: `render_check --shadow_length`
  (options in the header of `game/tests/render_check.gd`; GPU, no `--headless`); keep the eye over the tower's centre
  (in a nadir view the shadowed upper tower's pixels project beyond their footprint and could lengthen the tip).
- Earth shadows, Task 13b (render_check `--shadow_crawl=24,0.02`: the view turns 0.02 deg in azimuth and 0.01 deg in elevation
  per frame, 24 frames saved; nadir 500 m over the pad, Sun 19.4 deg, fov 70 deg, penumbra pixels of the shadow tip, 10 922 of
  them, high-passed with a 7 x 7 box): the mean frame-to-frame change of the high-passed luminance, in units of 1e-5:
  no shadows 22 (the imagery's own resampling), PCSS HIGH 24, PCSS LOW 32, hard shadows 17. So the HIGH penumbra does not crawl
  against the ground at this head motion (the same as without shadows within the noise; LOW is 45 % above); not checked in the
  headset (the per-eye pixel grids, real head motion). Atlas: 16-bit at 8192 px equals 32-bit (Sun 39.4 deg hard shadow
  length -1.31 m, tolerance 1.61, both; 115 / 8 pixels of 1.44 M differ by > 2 at Sun 19.4 / 5.6 deg, 23 / 3 by > 8, single lattice
  edge pixels); 4096 px doubles the texel (0.22 -> 0.44 m at 500 m range) and the edge error (-2.9 m) and 2048 px -4.8 m, so
  the size stays 8192, the depth is 16-bit. Stereo mock (3534 x 2838 per eye, pad with Starship, eye 150 m, Sun 32.7 deg),
  GPU median of 120 frames, nothing else running: shadows off 7.44 ms, 32-bit 10.55, 16-bit 9.99; with the analog weather on
  14.37 off, 17.19 on (16-bit). The Falcon 9 drone ship (mission `falcon9_asds_sea`, landed + 20 s, Sun 70.9 deg): the stage and
  its legs shadow the deck, no acne, the deck's edge is lit to the rim; one render, not a test. A weather-on pad scene (analog,
  Sun 39 deg): tower, mount and vehicle shadows stay; no artefact seen.
- Camera-precision test: relative error < 3e-7. Quadtree test: morph start < end for every level >= 2 on both bodies.
- Moon: mesh vs core 0.004 / 0.005 m at the IM-2 site / tile edge; lander CG 18.11 m above the drawn surface (detail
  off; `terrain_quadtree_test.gd` runs the Moon with the detail off as the regression baseline).
- Moon with the procedural detail (`game/tests/moon_surface_check.gd`, headless, ~10 min with its two converged trees;
  `-- --measure` adds the cost runs, ~31 min in all): level 18 at IM-2 mesh = `surface_samples_detail` at the level spacing
  (worst 6e-10 m in the builder's float64 arrays, tolerance 0.004 m; the detail differs from LOLA there by up to
  0.52 m); child fully morphed = parent mesh (1.6e-9 m, normals 2e-8 rad), unmorphed 0.053 m of finer detail;
  same-level neighbours share their edge at morph 0 / 0.5 / 1 (0 m), also across a cube face edge (faces 0 / 2,
  level 16: 65 shared vertices, 0 m) and around the +z pole (the four level-16 nodes: 65 each, 7e-15 m; before the pole
  sampling fix 64 each and up to 110 m); a child built from its parent's band heights equals one that samples the
  band (vertices, normals, morph, custom3 identical); 2 m above IM-2 the leaf is level 22, 1208 of
  2700 nodes bake a horizon (the detail-off tree's 1208), 1492 draw with an owner's (owner levels 7, 8, 9, 12), a level
  22 node addresses its level-12 owner's grid exactly; detail off: leaf 12, every node bakes, even vertices still.
  Detail off is identical to before: SHA-256 of every builder array of six nodes (Moon L5 / L7 / L12 / pole, Earth
  with data and flat) before / after the change equal, and render_check images at 2 / 20 / 200 / 2000 m pixel-identical
  (`--moon_detail=off --fixed_exposure --deterministic=32`). A render with the skirts tinted showed no skirt pixel at
  2 m (no cracks).
- Moon cost, IM-2 (i7-13700K, editor binary, the shipped extension build: CMake Release /O2), fix round 1 of Task 7.
  Real window (render_check, 1600 x 900, wall clock; "deepest drawn level" = the leaf under the camera):
  2 m spawn: full view 50.5 s -> 26.4 s, level 22 under the camera 49.8 s -> 7.0 s (level 21 at 6.8 s; the first
  ~3.5 s are the root chain's whole-face horizon bakes); 2000 m -> 2 m teleport: full view 16.4 s -> 7.9 s, level 22
  15.7 s -> 1.5 s. Steps: parent band reuse + camera-first order 46.8 s / 14.4 s (level 22 6.3 s / 1.35 s); + the
  parallel core batch 36.4 s / 10.5 s; + 16 high-priority builds 26.4 s / 7.9 s (20: 24.4 s / 7.3 s). Summed worker
  time 346 s -> 361 s (more contention per build; 2700 builds, 1208 bakes). Upward climb steps need no builds.
  Synchronous tree, one GDScript thread (`--measure`), nodes uploaded / drawn / leaf / summed build, detail off ->
  on: 2 m 1208 / 912 / 12 / 223 s -> 2700 / 2031 / 22 / 177 s; 20 m 220 s -> 2284 / 1719 / 19 / 165 s; 200 m
  123 s -> 1856 / 1398 / 16 / 153 s; 2000 m 1180 / 891 / 12 / 121 s -> 1436 / 1083 / 13 / 140 s (Task 7 before the fix:
  2 m 555 s). The single-thread timings vary ~2x between runs (the same detail-off tree took 223 s and 123 s): the
  13700K's efficiency cores; the cost spread of Task 7's review M4 (10-13 vs 19-24 us per point) is the same effect.
  67 x 67 `surface_samples_detail` batch alone: 5.6-7.3 ms at levels 6-18 (1.2-1.6 us per point), 2.8-3.8 ms at
  20-22; serial before 46-58 ms (10-13 us per point); tile blend band the same, cube face blend band 3x (19.5-23 ms).
  Node build without a bake 24-34 ms (face band 44-59 ms); a bake adds 15-60 ms (more at coarse levels).
  `moon_rocks` at the Diviner map's richest cells (3 x 3 pixel means: RA 0.357 at (39.36, -0.17), 0.349 at
  (8.11, -53.98), 0.343 at (-24.83, -45.14); with Tycho's floor; 3 x 3 cells per point, D >= 0.01 m): levels 16-25
  15-28 us per cell mean, worst 196 us; at most 21 rocks in a cell (level 25), 60-360 rocks per level over the 36
  cells. GPU (median, RTX 4090, detail off -> on): 1600 x 900 at 2 / 20 / 200 / 2000 m 1.29 / 1.29 / 0.84 / 0.96 ms
  -> 1.97 / 1.27 / 1.22 / 1.03 ms; stereo mock at the VR target (3534 x 2838 per eye) 2 m 3.24 -> 4.96 ms (+1.7 ms,
  ~2.2x the drawn triangles; `vr-performance.md`).
- Performance (`terrain_perf.gd`, RTX 4090, D3D12, 2560 x 1440, vsync off, the editor binary the game runs with, clock
  at warp 1): descent above Pad 2 400 km -> 2 m in 60 s, 360 deg pan at 2 m on the plateau 180 m east of the mount,
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

- Lunar site tiles are centred on the exact coordinate while key and id round to hundredths of a degree: a tile built for
  (20.19089, 30.77168) (preset / CLI) fails `set_lunar_site(20.19, 30.77)` with "does not match the products description".
  The game avoids it by snapping before every prepare; the core should snap the site itself (`lunar_site_at`).
- A site switch rebuilds the Moon quadtree on the main thread (roots, rock layer prewarm); it is a one-off hitch, not a
  streamed change.

- Moon quadtree main thread after a camera jump (Task 13 run of `moon_surface_check`, 10 km jump at 2 m, rock layer
  attached): the update's selection phase (the recursion over the new nodes) takes up to 21 ms at IM-2 and 52 ms in the
  Diviner boulder field in one frame, p99 14.4 / 16.6 ms over 364 / 414 frames - over the 5 ms hitch gate the rock layer
  meets (its own share max 4.8 ms). Not addressed (a time-sliced selection would be the fix); not measured: frame pacing
  in the headset while the view loads, the leaf `band_heights` retention (~70 MB at a 2 m view, Task 7 review).
- Moon with the procedural detail: a fresh 2 m spawn takes 26 s until the whole view is built (the cm detail under
  the camera after 7 s), a teleport 8 s (1.5 s); CPU-bound (16 builds at once on 24 threads; ~2700 builds and 1208
  horizon bakes, GDScript mesh assembly ~20 ms each). A 10 s spawn needs less work per node (e.g. the horizon bake or
  the builder's loops in the core) - open. Not measured in the exported release template (production builds refuse
  `--path`); the extension is the same DLL there.
- Crest lines at grazing view (Task 7 review M1): fragments whose interpolated vertex normal faces away from the eye
  (central-difference vertex normals leaning beyond a crest) got `regolith_reflectance`'s limb replacement and
  Lommel-Seeliger's 2x maximum. With Hapke (default, Task 9) such fragments are shaded with the triangle's facet normal
  (`dFdx`/`dFdy` of `camera_offset`) and Hapke's grazing-view rise is smaller: at IM-2, 2 m, the pixels brighter than
  their 7 x 7 median by > 40 fell from 1463 to 134 (`screenshots/moon_surface/task9/crest_before_above_after_h2_crop.png`).
  With `World.moon_brdf` = "lommel_seeliger" the lines stay (that law is kept bit-identical).
- The Moon's detail flag is read when the world builds its Moon tree (`World.moon_surface_detail`); a running world
  does not rebuild its tree on a change.

- Earth surface albedo (Task 15, U-3): the global sea outside the Sentinel-2 boxes is still the Blue Marble through
  land-fitted tables (luminance 0.019, no calibrated global ocean product): only the sea in and around the two boxes
  is L2A (above). The cosine fade over 100 km makes the Gulf shelf east of Starbase a brighter patch against the
  0.019 Gulf beyond it (a real shelf is turbid; the rim itself is an [E] construction). The pedestal stays the 90th
  percentile, so a bright flat pixel reads median + (value - pedestal): below its L2A value by pedestal - median (0.021
  at Starbase; the beach-spot figures of the Task 15 gates were not re-measured). The water rule
  (0.05 m) flags tidal flats as water; their brightness is kept through the pedestal rule, which is an [E] construction
  (90th percentile of the open-water L2A), and the Cape NAIP sea cells step up to 17 % (90th percentile, dark cells)
  against Sentinel-2. The two calibrated land products disagree by 10-20 % (Sentinel-2 2025 median of the least cloudy
  scenes vs the MODIS NBAR monthly median), spread over the 50 km land feather. The global level's land is an annual
  median: no seasons and no snow cover outside the permanently snow-covered areas.
- Ground bounce in the radiance cube: built (follow-up U-4), see `docs/subsystems/launch-structures.md` (Validation,
  "Ground bounce"). Open: only Earth eyes below 6 km over the imagery pyramids (the two areas) get the local profile.
- The Falcon 9 drone ship's shadow on the sea, and the Sun's penumbra on the sea in general, were not examined (the deck was).
- Open for the user, unverified: NAIP suggests the real OLP-1 mount stands some tens of metres WSW of our [E] site
  coordinates (`launch_sites.toml`); it affects the physics' site position, not the renderer.
- Terrain-load draw stall (~1 run in 3 shows one 11-17 ms frame inside the RenderingServer draw; engine-internal,
  worker-pool dependent; see CLAUDE.md known issues).
- Moon shadows: shadow edges are as sharp as the grid (vertex spacing, 10 m at the sites, 333 m on the far side; the
  bake's banded interpolation is within 0.3-1 deg worst case of a single-ray march); the horizon is taken in the disc
  centre's azimuth; nodes without terrain data have none; near the eye (within 150 m-4 km, the Moon only) the Sun's
  shadow map adds the drawn relief's, rocks', vehicles' and structures' shadows (moon-surface.md, Near-field shadows;
  `terrain_moon.gdshader` is a lit material only for that, its `light()`; the Earth's terrain receives the structures'
  and vehicles' shadows the same way, see Near-field Sun shadows (Earth), but casts none itself); the bake dominates a lunar site's first load (see above); the first triangles at a pole use a
  consistent but arbitrary north (the shader's pole fallback east; the baked pole vertex follows its neighbour);
  near-pole nodes (not only the pole vertex) bilinearly mix horizons of vertices whose norths differ by degrees (the
  azimuth profile is re-sampled in the bake, not by the GPU's filter), so shadow edges there are less accurate than
  the stated bound (worst 0.30 deg at lat -89.5 in the bake itself).
- Moon shadows, nodes one level apart: the horizon map is per level (its marches depend on the level's spacing), with no
  morph of the horizon, so a split or merge swaps the horizons at once. Child edge vs parent at IM-2: 15-24 % of the
  low-horizon (< 5 deg) values differ by more than the Sun's disc radius (0.267 deg), worst 6.7-17.6 deg, medians
  0.003-0.28 deg. Seen (IM-2, 2 km up, Sun 3.6 deg above the local horizontal, 0.5 m camera steps across a node
  split, same view, `screenshots/moon_shadows/pop_*`): an L10 -> L11 split (node 9 km away) changes 0.098 % of the
  pixels by more than 2/255 (0.005 % by more than 8, max 25) and an L11 -> L12 split (4.6 km) 0.43 % (0.012 %, max
  25); the same step with no split: 0.014 % (0.002 %, max 25). No step or pop is visible at 1:1; the change sits
  along the shadow edges inside the split node. Not a visible defect at these views; a horizon morph stays open.
- A fine imagery patch smaller than ~0.1 x viewing distance fades out from far away; S2 brightness steps over open
  water remain.
