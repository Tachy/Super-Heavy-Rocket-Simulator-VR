# Moon procedural near-ground surface (design, 2026-11-04)

Status: draft for the user's approval. Slot set by the user 2026-11-04: next plan after the depot/tanker/HLS models,
before M7. Statements marked **(user)** are the user's; **(call)** marks a decision taken by the designer that the
user has not explicitly confirmed.

## Why
Up close the Moon is bare: the terrain mesh ends at the LOLA data (20 m site tiles, 10.4 m leaf vertex spacing at the
IM-2 site, 333 m on the far side), the only albedo is a 4k global WAC mosaic (~2.7 km per pixel), there is no detail
texture, no rock, no small crater, and nothing near the ground casts a shadow (vehicles neither). The shipped lunar
missions land at a polar site with the Sun at <= 7 deg, where real terrain is dominated by long rock and crater
shadows. The physics ground (`environment::LunarTerrain`, `contact::TerrainField`) is equally smooth, so a landing
today cannot meet a rock or a small crater.

## Goal (user)
- Procedural lunar surface detail near the ground: small and large rocks, small craters, regolith (dust) texture;
  rocks AND dust textured.
- **Whenever the camera is close enough, anywhere on the Moon, any camera** (cockpit, external, free camera, VR):
  no special-cased moments or sites.
- **Seamless, mipmap principle:** what is drawn follows the camera distance relative to the display resolution -
  the height/distance at which a rock appears fits the resolution; no visible popping, no visible band.
- **Rocks cast shadows** on the dust and on each other; vehicles cast shadows too (near-field shadow map).
- **Physics first:** the generator is deterministic from the coordinates and lives in the core, so ground contact
  and landing meet the same rocks and craters the image shows.
- Size-frequency distributions from measured lunar statistics (data, not free noise).
- Finest drawn scale: ~1 cm stones (proposed in chat, user "passt"); finer is texture.

## Non-goals
- **Plume-surface interaction** (dust lifting, blown stones, erosion pits): the surface stays static in this plan;
  own plan later **(call - asked in chat, user answered "leg los" without addressing it)**.
- Earth terrain (unchanged), Moon far-globe imagery beyond what the BRDF change needs, astronaut walking, footprints,
  vehicle tracks.
- No new downloaded per-site imagery packs: detail is procedural everywhere.

## Anti-goals (failure even if it technically works)
- Rocks / craters only in the image (physics on a smooth surface) or only in the physics.
- Popping, a visible ring where detail starts, a brightness change when a rock passes from texture to geometry.
- Detail that changes when revisiting the same spot (must be deterministic, identical on every run and machine).
- Free noise that looks "game-like": densities, sizes, shapes and roughness must trace to sources with [S]/[E]/[D].
- Missing the VR frame budget (11.1 ms, GPU target <= 9 ms, docs/subsystems/vr-performance.md).
- Lunar missions that start tipping over or failing because rocks appeared (see Landing).

## Current state to build on (recon 2026-11-04)
- Terrain: cube-sphere quadtree (`game/world/terrain/terrain_quadtree.gd`, `terrain_node_builder.gd`), 64x64-cell
  nodes, heights via one `StarsimCore.surface_samples` call per node, `MAX_LEVEL` 21 (level 21 = ~2 cm spacing),
  split limited by the finest data resolution (today stops at level 12 at the site), vertex morphing to the parent.
- Shading: `game/shaders/terrain_moon.gdshader` + `moon_regolith.gdshaderinc`: Lommel-Seeliger with a
  Krisciunas & Schaefer phase function and opposition surge; the far globe `moon.gdshader` uses the same function.
- Shadows: per-node horizon maps (16 azimuths, one texel per vertex, `environment/terrain_horizon.*`) with exact solar
  disc fraction; vehicles get one whole-vehicle sun fraction; no Godot shadow maps on the Moon
  (`moon_light.shadow_enabled = false`).
- Physics: `environment::LunarTerrain` (bilinear LOLA, `normal_fixed` over 10 m), `contact::TerrainField::below`
  returns height + normal for the six HLS footpads; mesh vs core agreement 0.004 m.
- Deterministic hash noise exists (`environment/cloud_noise.hpp`, integer hash, fixed seed).

## Design

### 1. One deterministic surface model in the core
A new core component (in `environment`, behind its own interface) defines the procedural detail on top of LOLA as a
pure function of the Moon-fixed position (MOON_ME), with no state and no library RNG (integer hash, portable math),
so every machine and every run gives the same surface:

- **Small craters**: a crater population from 1 cm-scale up to where LOLA resolves craters (~100 m at the 20 m tiles,
  larger on the far side where LOLA_16 is 1.9 km), with a measured size-frequency distribution (small-crater
  equilibrium / production function from the literature, e.g. Neukum production function, equilibrium densities
  such as Xiao & Werner 2015 - source and numbers documented). Profile: depth/diameter and rim shape from fresh vs
  degraded crater morphometry, degradation state drawn per crater.
- **Regolith roughness below the LOLA resolution**: band-limited fractal undulation whose amplitude and Hurst
  exponent continue the measured LOLA roughness statistics (e.g. Rosenburg et al. 2011) down to the cm scale; the
  procedural band starts exactly where the LOLA grid stops so nothing is counted twice.
- **Rocks**: discrete rocks with a cumulative size-frequency law from LRO NAC boulder counts / Apollo / Surveyor
  rock statistics (power-law or exponential form, documented [S]), scaled locally by a rock-abundance field
  (Diviner rock abundance map, PDS, if its licence checks out - otherwise a documented model, e.g. more rocks on
  fresh crater rims and steep slopes), shapes from a small set of core-defined star-shaped templates (seeded radial
  functions; axis ratios from measured lunar rock shapes), random orientation, partial burial depth drawn from a
  documented distribution, rock albedo brighter than mature regolith.
- Spatial organisation: per size octave a cell grid on the cube-sphere faces (aligned with the terrain quadtree keys)
  so a query touches only nearby cells; seamless across face edges. Exact scheme is the builder's.
- **Detail-limited queries** (the mipmap principle in the core): every height query takes a smallest-wavelength /
  smallest-feature limit. The physics asks with the full detail; a terrain node at level L asks band-limited to its
  vertex spacing; the renderer asks for the rocks above a size. Coarse = exact average of fine wherever feasible, so
  morphing between levels stays smooth and mean brightness / height do not jump.
- **Same truth for image and physics**: `LunarTerrain` (or a composed terrain over it) exposes height and normal
  including craters, roughness and rocks; `contact::TerrainField::below` and the footpad contact see the rocks (rock
  top as support surface with its local normal); `normal_fixed` uses a scale fitting the footpad, not a fixed 10 m.
  The quadtree's "finest data resolution" becomes the procedural limit, so nodes split down to the cm scale near the
  camera.

### 2. Rendering by screen size (seamless)
- **Selection by projected size**, not by altitude: a feature becomes geometry when it spans about 1-2 px on the
  target display (VR render target per eye; the desktop window otherwise). Height and lateral distance are one
  rule. Both VR eyes use the same selection (one reference point between the eyes) so the eyes never disagree.
- **Unresolved detail stays statistically in the texture**: below the geometry threshold rocks and craters are not
  dropped but represented in the shading (albedo share of rock, roughness / shadow share for the current Sun
  elevation) from the same size-frequency law, so the mean radiance of a patch does not change when a rock passes
  from texture into geometry.
- **No popping**: a rock grows out of the regolith over a short size band (burial decreases) instead of appearing;
  terrain detail morphs as today. A budget cap must not create popping: under load the pixel threshold rises
  smoothly for all features, never a hard count cut.
- **Rocks as instanced geometry** per terrain node (MultiMesh or equivalent), meshes from the core templates with
  LODs; positions, scale, orientation, burial from the core (one call per node, worker thread like the heights).
- **Pebbles 1 cm .. geometry limit**: the smallest stones can be texture-borne relief (parallax / height-map pebbles
  with self-shadowing) instead of instances when instance counts would break the budget; where the geometry limit
  sits is the builder's call from measurement, the visual rule (no visible boundary, ~1 cm finest) is fixed.
- **Textures (rock and dust)**: procedural, generated by our own tool (no downloaded textures), baked mipmapped
  height/normal/albedo-variation maps at several scales for the regolith with stochastic (hex) tiling against
  repetition; triplanar rock textures from a small baked set; dust fillets and burial blend at rock bases in the
  shader. Global brightness keeps the WAC mosaic as the large-scale albedo (normalised so detail does not shift the
  mean). Measuring references for the look (not shipped): LRO NAC 0.5 m/px images (PD) and Apollo surface photos
  (PD), especially low-Sun ones.

### 3. Reflectance: Hapke instead of Lommel-Seeliger
- Hapke model (single-scattering albedo, particle phase function, shadow-hiding and coherent-backscatter opposition
  effects, macroscopic roughness) with lunar parameters from published fits (e.g. Sato et al. 2014 LROC WAC Hapke
  parameters, Hapke 2012) - [S] with source.
- The macroscopic roughness term only covers what is NOT drawn: as rocks/craters become geometry the unresolved
  roughness decreases accordingly (scale-dependent), part of the "no brightness jump" rule.
- Terrain, rocks and the far globe use the same function so the Moon from orbit keeps its brightness.

### 4. Near-field shadows
- A shadow map around the camera for the Sun: casters = rocks, near terrain detail (small craters are strong low-Sun
  shadows), vehicles, structures; receivers = terrain, rocks, vehicles. Both VR eyes. This also closes the known
  limit "vehicles cast no shadow on the terrain" on the Moon and replaces the whole-vehicle sun fraction near the
  ground by real shadowing.
- Combined with the horizon maps (minimum of both visibilities); the horizon maps stay for km-scale terrain.
  Beyond the shadow-map range the statistical shadow share in the shading takes over, blended so no ring is visible.
- Soft edge from the solar disc (0.53 deg), consistent with the horizon-map penumbra. Range and cascade layout are
  the builder's, sized for the low polar Sun (a 2 m rock casts a ~16 m shadow at 7 deg).
- Earthshine keeps the horizon-map treatment only.

### 5. Landing with rocks (physics first)
Rocks and small craters are now real for the legs. Planners absorb this like any vehicle change (standing rule):
- **Contact model (call)**: rock templates are star-shaped, so a rock's top is single-valued; the support surface is
  max(terrain incl. craters/roughness, rock tops) - no overhangs. A footpad is a disc: the query returns the highest
  support point under the pad disc and the normal there (footprint max-filter), so cm features cannot make the
  contact stiff or jittery; the existing contact stiffness and tick stay. The six footpads remain the only ground
  contact; engine-bell and hull clearance against rocks are checked by the hazard check, not simulated as contact.
- **Hazard check (call)**: like mission planners who pick the landing spot from LRO maps, the landing planner checks
  the planned touchdown point against the core surface (footpads on rocks, slope over the 12 m footprint vs the HLS
  limits in `starship_hls.toml`, bell clearance) and moves the target to the nearest safe spot near the nominal site
  before the descent - a target change, no new guidance phase. Terminal re-targeting after a dispersion re-uses the
  same check on the live state (re-plan from flight data). The core surface stands in for the sensor; sensor
  modelling is not in scope. Rock density is NOT capped at the shipped site: the site's statistics are what they are.
- Shipped lunar missions and the lunar tests must stay green; a red is a planner defect, not a reason to remove
  rocks or fudge the leg contact.
- Leg contact validation: footpad on a rock, footpad in a small crater, tilt limits - unit tests with documented
  tolerances.

### 6. Fidelity flags
Switchable (project rule): procedural detail in the physics surface (on by default), rocks in the physics (on),
Hapke vs the current Lommel-Seeliger (Hapke default), near-field shadows (on). Turning detail off must reproduce the
current LOLA-only surface bit-for-bit (regression baseline for the existing tests) - including the 10 m
`normal_fixed` scale and the point footpad query, which are gated by the same flag **(call)**.

## Budgets and data (calls)
- VR budget measured at the documented OpenXR render target (docs/subsystems/vr-performance.md) with the existing
  Moon scene (horizon maps on). If a cut is needed, the order is: pebbles to the texture route first, then a
  smoothly higher pixel threshold; feature statistics are never thinned. Worker cost per node and node counts at
  the deepest levels are measured and documented, not pre-set.
- The physics queries the full-detail surface for six pads per tick; a per-site cell cache is allowed (builder).
- Literature numbers typed in with [S] and source (as elsewhere in the project) are acceptable; downloaded datasets
  (Diviner rock abundance, WAC Hapke maps) only after the licence check in `sources.toml`, otherwise the documented
  model.
- Shadow technique (Godot directional shadows or a custom pass) is the builder's, chosen by VR cost.

## Validation (each with a documented tolerance)
- Determinism: same queries -> identical results across runs and threads; cross-face seams continuous.
- Statistics: generated rock and crater counts per area and size bin reproduce the source distributions (Poisson
  tolerance); roughness spectrum continues LOLA's.
- Consistency: band-limited coarse heights equal the average of the fine heights within tolerance; drawn surface vs
  core height (incl. rocks) within the existing 0.004 m order at the lander; the drawn rocks are the core's rocks.
- Hapke: disk-integrated lunar phase curve against a published reference (e.g. ROLO / Lane & Irvine) and the
  current globe brightness; WAC Hapke parameter fit reproduces its source.
- Seamlessness: render series descending and approaching (render_check `--climb`, approach series) show no popping
  or brightness step; the mean patch radiance stays within a stated tolerance across the geometry threshold.
- Look: low-Sun renders at the IM-2 site compared side by side with LRO NAC / Apollo references (statistics and
  impression, not pixel matching); progress PNGs to the user.
- VR budget: GPU and main-thread timings on the Moon at the VR render target, within the documented budget.
- Landing: shipped lunar missions and `[lunar]` / `[contact][legs]` tests green with rocks on.

## Constraints
- Architecture rules: generator and contact in `core` (no Godot), bindings thin, shaders/GD in `game`; no physics in
  GDScript; constants with sources in `constants.hpp`; data sources with licences in `sources.toml` / CREDITS.
- Every external number marked [S]/[E]/[D]; the user decides config changes for vehicle parameters (legs etc.) -
  propose, do not change unasked.
- Test images in `screenshots/moon_surface/`.

## Left to the builder
Cell/hash scheme, number of rock templates and LODs, geometry-vs-texture limit for pebbles, shadow-map range and
cascades, texture resolutions and tile scales, how the core exposes rock lists to the bindings, task split and order
(physics surface first is the natural order: it is what the image must match).
