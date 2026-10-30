# Cloud world volume - design

Date: 2026-10-28. Status: draft for the user's approval. Replaces the world-fixed direction cache
(`2026-10-27-cloud-view-cache-design.md`) as the cloud rendering architecture of M8
(`docs/subsystems/weather-clouds.md`, `docs/subsystems/vr-performance.md`). Inputs: `.superpowers/sdd/2026-10-28-cloud-architecture/`
(`user-decisions.md` binding; `recon-current.md`, `research-industry.md`, `research-techniques.md`) and
`.superpowers/sdd/2026-10-27-cloud-view-cache/dots-investigation.md`.

Terms used below. **World volume**: the camera-centred, Earth-fixed clipmap of static data (light, occupancy).
**Near tier**: the per-frame, per-eye march of each eye ray from 0 to R_near. **Far tier**: the per-frame, shared
(cyclopean) march from R_near to the end of the ray. The old "far tier" (pass 1 of `cloud_march`, 20-34 km band) is
called "old pass 1".

## Why

- The per-frame march (1/3 resolution, VR preset) costs 45-50 ms in VR; it looked like a "6x6-Pixel-Rasterung" and its
  edges shimmered (head-fixed grid, binary edges). Cost by component (VR proxy, `recon-current.md` 3.3): sub-samples
  23-29 ms, old pass 1 13-21 ms, hybrid per-sample light march 12.6 / 11.2 ms, outline pass 10-12 ms, air shadow
  2-5 ms. Per sample it does ~10-20 texture fetches plus a 12-32-sample light march: about 10 x the per-sample cost of
  Nubis^3 (Guerrilla 2023), which reaches 2.2-4 ms on a PS5 with world-space voxel density, an SDF and cached lighting.
- The direction cache (converged 2.6-3.6 ms, "perfekt" at full resolution) failed after four fix rounds: tile holes,
  dotted seams, staircase edges, horizon lag, visible rebuilds. Root cause: a translucent volume with gaps at several
  depths cannot be reprojected with one depth per texel, and validity rules work per tile. At 1 km/s a cached
  direction texel of a cloud 10 km away is 1 px wrong after 5 ms (`research-industry.md` 4). The research found no
  shipped direction cache for a fast observer (Ghost of Tsushima: slow ground observer only; "Clouds in the Cloud",
  HPG 2023: the same parallax distortion).
- What the industry does (2022-2026, `research-industry.md` 3): view-independent quantities (density support,
  distance fields, light transmittance) in world space, refreshed amortised; the view-dependent integral marched per
  frame. Every engine that adds screen-space temporal reuse shows its artefacts (trails, ghosting, flicker, eye-order
  bugs in VR), which the user has rejected.

## What the user wants (their words, condensed)

- Stop the direction cache: "das muss was mit der grundsätzlichen Logik zu tun haben". Its lessons are input, its code
  is not continued.
- Quality bar: the converged full-resolution look of the cache ("visuell sehen die Wolken jetzt natürlich perfekt aus
  mit voller Auflösung") everywhere - no holes, seams, visible tile rebuilds, pops, flicker or shimmer.
- Feathery edges: "Hoffe wir bekommen so auch fedrige Ränder der Wolken hin".
- "Es zählt eigentlich alles": pad, deck passage at ~1 km/s, whole decks to the horizon from high altitude, orbit,
  re-entry, booster catch. "Wir müssen die Architektur so anpassen, dass es passt und optimiert ist."
- No foveated rendering / VRS of any kind ("unruhiges Geflimmer an den Rändern ... es muss ein optimierter
  Algorithmus sein"). No temporal jitter / screen-space temporal accumulation in VR. VR 45 Hz with weather (22.2 ms),
  90 Hz without. "Qualität zuerst", physics first, one path for desktop and VR.
- The 3D shape is static and the basis for the light ("eine 3D-Form bleibt statisch und muss nicht immer neu berechnet
  werden, sondern dient als Grundlage für das Licht"). Shape primitives ("Nebelkugeln", any free 3D form) where the
  research supports them: "halte Dich an die Recherchen ... haben andere ja auch schon gemacht". "Verlass Dich auf die
  Recherche ... nimm das fortschrittlichste Verfahren."
- Approved direction ("Also auf gehts ... setze es nach den Erkenntnissen für uns um"): world-space static data at
  several scales around the camera (fine near ~50 m voxels, coarse far) + distance field for empty-space skipping;
  world-space light volume (Sun optical depth + sky light) updated in small slices as the Sun moves; per-frame exact
  march without reprojection and temporal filtering; per-eye near tier (~1-2 km) + one shared far tier composited
  exactly (L = L_near + T_near L_far); a 2.5D column model for the far deck from altitude / orbit; prototype first on
  the RTX 4090 for the worst views; fall back to a lower far-tier resolution (still no temporal filtering) if > ~10 ms.
- Budget: ~11-14 ms for clouds in VR (scene without clouds 7.4 ms); research estimate 5-12 ms, ~340 MB.

## Goals

1. Every frame shows a fresh march of every visible cloud with the look of the full-resolution reference: nothing
   view-dependent is kept between frames, so holes, seams, tile rebuilds, ghosting and reprojection lag cannot occur
   by construction.
2. The 3D shape is static and the basis for the light: the macro + meso density (weather envelope x shape noise) is
   baked into a world volume once per weather snapshot and region, together with the Sun optical depth, the
   multiple-scattering inputs, the sky light and the occupancy / distance field derived from it.
3. The march does ~3-8 fetches per sample (baked density 1, light 1, table 1, detail 0-3, curl 0-1; today ~10-20 plus a
   12-32-sample light march), skips empty space with the distance field and stops early in dense cloud.
4. Both eyes are exact: per-eye march where disparity matters (< R_near), one shared march beyond (disparity
   <= 0.063 px at 2 km), composited by the exact split of the volume rendering equation.
5. The look stays: the same physics (weather envelope, cover / water rules, optics, phase function, transfer table),
   sunset colours and air shadow; the image is A/B-checked against the per-sample reference march. Feathery edges at
   every distance.
6. VR with weather on holds 45 Hz in every phase (cloud GPU <= 11 ms median, <= 14 ms p99); one path for desktop and VR.

## Non-goals

- Wind, advection or cloud evolution (shapes stay frozen per snapshot; standing non-goal).
- Changes to the physics model: optics, transfer table, cover / water rules, LCL, phase function, the column model's
  physics. (Wrenninge octaves are not introduced: the transfer table is the ruling and the better physics.)
- Foveated rendering / VRS, temporal accumulation, reprojection of cloud images (excluded by the user).
- The scaled-space far field beyond 1.5e7 m (unchanged) and the rest of the weather (M11).

## Anti-goals (failure even if it "works")

- Any hole, seam, tile / slab border, level boundary, refresh front or pop visible in the image.
- Blobby, soft or "cotton wool" edges instead of the feathery, crisp edges of the converged cache.
- Flicker or shimmer while nothing changes; the two eyes seeing different content (other than the true disparity).
- The cloud layer shifting against the world; trails or ghosting when moving.
- The look drifting from the reference march (light, shape, colour, evening skies).
- Missing 45 Hz in a normal flight phase with weather on.

## Constraints

- Godot 4 double build, `CompositorEffect` + RenderingDevice compute (`CloudEffect`); one graphics+compute queue (no
  async compute in this Godot build): world-volume upkeep runs inside the frame as fixed small slices.
- `core` has no Godot dependency; physics never in GDScript; GPU shaders in `game/shaders/`. Floating origin: volume
  origins in float64 Earth-fixed coordinates, differenced in double before float32.
- Kept rulings: physics first, every simplification deliberate, documented and switchable; transfer table over
  delta-Eddington; independent columns; beam-cover fit; the plane-parallel bias of the Sun disk accepted; the far
  tier marches the shapes and the column model draws only beyond the far fade's footprints (standing near / far
  ruling); the 32 km shape lattice stays unless the user decides otherwise (Open); deterministic march; weather and
  clock jumps restart without stale content and never show a frame without clouds; main-thread scheduling
  <= 1.0 ms per frame; cache contents never depend on the frame's depth buffer.
- Fidelity flags that stay: `scattering_model`, `air_shadow`, `far_3d_light`, `detail_levels`, `deterministic_march`
  (reference path), `cloud_debug_output`, `far_fade_m`, `CloudShadowEffect.grid_mean_only`, `pass_enabled`.
- Tests: only affected tags / checks; visual checks on the GPU; images to `screenshots/<topic>/`.

## The design (decided)

Chosen per the user's "nimm das fortschrittlichste Verfahren": at each point the most advanced technique the research
shows as proven in shipped or peer-reviewed systems - Nubis^3 (Guerrilla 2023): world-space voxel density at several
scales, up-rezzed per sample by noise, SDF skipping, cached light; stateless split-ray stereo (geometric bound,
`research-techniques.md` 1.9); Hillaire's analytic segment integration (2016); the project's Monte Carlo transfer
table (Bouthors-style, more physical than Wrenninge octaves).

### A. What is static (per weather snapshot, Earth-fixed, deterministic)

- **Weather envelope** (unchanged physics): the core's `ColumnLayers::build` per snapshot (LCL raise, cover scaling to
  tcc, in-cloud bound, adiabatic cumulus water) and the render texels. Built once per snapshot on the CPU, as today.
- **Shape and detail noise** (unchanged): core `cloud_noise.cpp` textures, Earth-fixed.
- **Baked density** (user decision: "world-space static density at several scales", the static 3D form as the basis
  for the light): per world-volume cell the macro + meso density - today's model (envelope x shape noise, cover / mean
  tables, LCL base ramp) band-limited to the cell size (the shape's box-filtered mips with the prefilter tables, as old
  pass 1 band-limits at a footprint; a few taps per cell). Stored per cell: the band-limited mean extinction rho and
  the envelope's in-cloud extinction sigma_c; rho / sigma_c is the cell's cloud fraction. Box filtering keeps each
  cell's water, so each weather cell's water is conserved by construction (the 0.996-1.000 test kept, now per cell).
- **Occupancy, distance field, light**: derived from the baked density (B, C).
- All built on the GPU (compute) from the snapshot textures already resident - the CPU would need ~1-2 s per full
  build and ~1 GB of uploads. A C++ reference of the per-cell bake (`core`, environment module) checks the GPU bake at
  sample points in a `[weather]` test.
- The per-sample procedural evaluation of envelope + shape stays only as the reference march (F).

### B. World volume: clipmap levels

8 levels, each 256 x 256 cells horizontally x 128 vertically, toroidally addressed, snapped to its cell size:

| Level | Cell (h x v) | Horizontal extent | Vertical |
|---|---|---|---|
| 0 | 50 m x 25 m | 12.8 km | 3.2 km window around the camera height (clamped to the shell) |
| 1 | 100 m x 50 m | 25.6 km | 6.4 km window |
| 2-7 | 200 m .. 6.4 km x 164 m | 51 .. 1638 km | whole shell -0.5 .. 20.5 km |

- Grid: horizontal axes in an Earth-fixed azimuthal-equidistant projection about a frame origin, vertical = height
  above the WGS84 ellipsoid (a flat tangent grid would miss the shell by 13 km at 410 km). Levels are centred on the
  camera's ground point. Levels 0-1 are off when their window cannot meet the shell (camera above ~22 / ~24 km;
  always in orbit).
- Per cell (16 B): **density** RGBA16 (rho, sigma_c for both snapshots of the cross-fade pair, one fetch, blended by
  w per sample) and **light** RGBA16F (tau_sun; tau_below = vertical optical depth to the cloud base, the table's b;
  tau_above = vertical depth to the top, sky source; tau_moon). Per level at 1/4 resolution: a max-extinction mip
  (R16F, the Delta-tau step bound) and a Chebyshev **distance field** (R8, in coarse cells), both from rho of both
  snapshots, dilated by the largest lift of the sub-cell noise. Memory: 134 MB per level, **~1.07 GB** for 8 levels
  (+134 MB transient during a re-frame), ~2 MB for mips and distance fields. Lean variant: below, decision 2.
- Both snapshots baked (Claude's call): the cross-fade stays exact per sample at any w (`cloud_layer_test`: never
  jumps), no density rebake fronts while w moves; a new pair's snapshot is baked into the free channel pair while its
  weight is 0, as the field slots work today.
- Unlike the rejected block-maximum skip of the old march (VRP 388-393: noise maxima above threshold almost
  everywhere), the occupancy comes from the actual baked density per level: tight where cells are small and near
  (50-200 m), and coarse cells skip whole clear layers and clear weather cells, where long rays spend their length.
- **Scroll**: each level bakes only the newly exposed slabs. Density and occupancy of a slab are baked in the frame
  it is exposed, before the march - never late (1 km/s: level 0 ~15 k cells per frame; 7.8 km/s, levels 2-7: ~0.15 M).
  Its light may first be **provisional** (Claude's call): inherited from the next coarser level, replaced within the
  upkeep budget and blended in over 0.25 s. New slabs enter at the level's outer edge inside the 8-cell fade band to
  the next coarser level, so they cannot pop. The coarsest level is always complete.
- **Re-frame** when the ground point is > 1000 km from the frame origin (as the light cache today): levels are
  rebuilt one at a time, coarsest first, each into the transient spare level, cross-faded over 0.5 s. Upkeep may spike
  to at most **3 ms** [E] in those frames (Claude's call; 22.2 - 7.4 - 11 ms leaves ~3.8 ms).
- **Shape primitives ("Nebelkugeln")**: per the research verdict (`research-techniques.md` 6.6) primitives are the
  macro shape and the empty-space structure, never rendered as sprites or per-ray Gaussians. The distance field is
  exactly that structure: every value is a sphere of guaranteed-empty space the march jumps across (sphere tracing).
  The baked macro shape is today's noise-threshold field in the first build (the look the user approved). A lobe
  generator (Bouthors-style stacked lobes per convective cell, masses normalised so each weather cell keeps its cloud
  water, splatted into the bake) is a later switchable option, `CloudLayer.shape_source` NOISE / LOBES (Open).

### C. Light volume

- tau_sun per cell = optical depth toward the Sun from the cell centre through the baked density, stepping from the
  cell's level into coarser ones, then the grid-mean column beyond the coarsest level; Earth's shadow -> infinite.
  ~32-64 samples per cell with Delta-tau-limited steps on the max-extinction mip. tau_below / tau_above along the
  vertical likewise; tau_moon only while the Moon is the dominant light (Sun below -6 deg at the level centre).
- Multiple scattering: the existing transfer table (a = mu_e tau_sun, b = tau_below, mu_e, theta, psi; one fetch per
  sample), `scattering_model` CLOSED_FORM kept. Sky light: `cloud_sky_scattering_source` with tau_above x the
  atmosphere's sky irradiance (re-evaluated every 500 m / 5 % along the ray as today).
- **Surface correction** (replaces the 12-32-sample hybrid light march): for a sample whose step weight
  T (1 - exp(-Delta tau)) > 1e-3 and whose level's cell is coarser than the footprint, a short Sun march of 4 samples
  over the first 2 cells of the active level through the per-sample density (baked + sub-cell detail, D), then the
  volume's tau from that point. It resolves the lit surface below the cell size (the error source of cloud_check 8's
  broken-field worst pixels). Flag `light_entry_samples` (0 = volume only).
- **Refresh**: continuous round-robin slices under a per-frame budget (1.0 ms of the upkeep's 1.5 ms). Triggers per
  cell: Sun angle since its last update times the level's sensitivity (shadow shift H dtheta / sin^2 e kept below 1/4
  cell), weather blend weight moved >= 0.02, provisional slab, re-frame. Priority: provisional slabs, visible and
  near, most stale. A refreshed cell moves its tau toward the new value by at most a bounded step per refresh, so time
  warp or a pair switch shows as lag, never as a front. Full cycle of 67 M cells ~60 ms GPU [E]: ~1-2 s at the budget.
- Cold start / jump: coarsest level first (the first frame shows clouds), full volume within ~1 s [E].

### D. Per-frame march

- **Resolution (Claude's call)**: the bar is the look of the full-resolution reference, not a pixel count. Start
  where the research priced it: near tier per eye at full resolution if the gate allows, else 0.7x per axis; far tier
  at 0.5x per axis. Every reduced tier is upsampled depth-aware (guided by its transmittance-weighted cloud distance
  and the scene depth) WITHOUT history. Resolution is raised only where the measured budget allows and A/B images
  against the full-resolution reference show a difference.
- **Near tier**, per eye, rays [0, min(R_near, scene depth)], R_near = 2 km. The dispatch is skipped when level 0's
  distance field at the eye exceeds R_near (pad in clear weather, high altitude, orbit: ~0 ms).
- **Far tier**, shared: the cyclops point (eyes' midpoint), rays [R_near, end]. Its sample directions are fixed in an
  Earth-fixed cube parameterisation (0.5 x the eyes' ~35 px/deg, from the renderer's projection); only texels inside
  the union of both eye frusta (+ the upsample's margin) are marched each frame. Head rotation in a still view marches
  the same rays (bit-identical) and only resamples them - no shimmer from head motion (Claude's call). Nothing is
  kept for the next frame. Rays end at the cyclopean scene depth (per frame, the nearer of both eyes' depths resampled
  to the direction; geometry beyond R_near has <= 0.06 px disparity).
- **Composite** per eye pixel: L = L_near + T_near L_far(dir), T = T_near T_far; where the eye's scene depth < R_near
  the far term is 0. The two-pass composite (`cloud_layer.gdshader`, `cloud_layer_light.gdshader`, T in .a) stays.
- **Steps**: empty space by sphere tracing on the distance field of the finest level containing the point; in occupied
  space step = min(Delta tau <= 0.3 from the max-extinction mip, 1/2 cell, 1/4 of the finest structure resolved at the
  pixel footprint), >= 2 m; samples on Earth-fixed shells (deterministic phase, kept); analytic segment integration
  (Hillaire 2016); silhouette entry bisection (4 x, kept); termination as today incl. the Sun-ray exception to T 1e-9.
  Retired: sub-samples, outline pass 1b, the head-fixed divisor-3 grid, temporal pass and Catmull-Rom upsample in the
  production path.
- **Per sample**: baked density (1 fetch, both snapshots), light volume (1), transfer table (1); where the pixel
  footprint is finer than the level's cell: the sub-cell field (below, 1-4 fetches). Level fade: 8 cells at each
  level's outer edge blend density and light linearly.
- **Sub-cell detail and feathery edges** (where footprint < cell; near tier and edges, at every distance whose
  footprint is finer than its level's cell): the density inside the cell is redistributed by the sub-cell field
  v_sub = the shape noise's residual above the cell's scale (levels >= 2, 1 fetch at the footprint's mip) + the three
  detail noise levels (1000 / 250 / 62.5 m tiles, cells to ~8 m), each band-limited by the footprint
  (w_k = clamp(log2(l_k / f) - 1, 0, 1), as today): density = sigma_c m(c) smoothstep((v_sub - t(c)) / w) with the
  existing threshold / mean tables at the cell's cover c = rho / sigma_c - the cell's mean, hence its water, stays rho.
  Where the footprint reaches the cell size the weights go to 0 and the baked rho is used directly: no aliasing, no
  shimmer. Added (Nubis 2015-2023 practice): a curl-noise distortion of the detail lookup (amplitude ~ the detail cell
  size, band-limited the same way) for wispy, torn edges; flag `edge_curl` (0 = without). Early-out: where
  |c - t| leaves no room for the sub-cell field to change the result (solid interior, clear), the detail fetches are
  skipped. The feathery edges come from this per-sample field, not from the bake, so the bake cannot make edges blobby.
- **Air shadow / crepuscular rays**: Bruneton difference form kept; T_c at each segment = exp(-tau_sun) from the light
  volume (1 fetch instead of 3 shadow-map samples), beyond the coarsest level the far map's grid mean. Far tier: 32
  segments over [R_near, end]; near tier: 4 segments over its range; composited like L. Rays now come from clouds up
  to ~800 km away instead of 9-11 km (fixes an open point).
- **Terrain / vehicle shadows**: `CloudShadowEffect`'s two maps stay as the terrain's interface but are filled from
  the light volume (one fetch per texel, ~0.05 ms instead of 0.6). The CPU light for vehicles stays (grid mean).
- **Cost estimate** [E] (2 x 10.0 Mpx eyes; far region ~11.7 Mpx at eye density -> 2.9 Mpx at 0.5x; 200-300 G
  trilinear fetches/s sustained on the RTX 4090, `research-techniques.md` 1.2, x1.5 for ALU / divergence):

| Tier, view | Mpx | Steps per px (occupied / empty) | Fetches per occupied step | ms |
|---|---|---|---|---|
| Near full res, V1 pad (40 % px with cloud < 2 km) | 20 x 0.4 | 24 / 8 | ~10 (1+1+1, detail 3, curl 1, entry ~3) | 10-15 |
| Near 0.7x, V1 | 9.8 x 0.4 | 24 / 8 | ~10 | 5-7.5 |
| Near 0.7x, V2 / V7 inside the deck (all px, T < 1e-3 after ~150 m) | 9.8 | 12 / 0 | ~10 | 6-9 |
| Far 0.5x, V3 broken deck to the horizon | 2.9 | 48 / 32 | ~5 (1+1+1, shape residual 1, entry rare) | 1.7-2.6 x1.5 = 2.6-3.9 |
| Far 0.5x, V4 80 km / V5 orbit | 2.9 | 32 / 16 | ~4 | 1.6-2.4 |
| Upkeep (bake slabs, light slices, mips) | | | | 1.5 (spike <= 3) |
| Upsample + composite + cyclopean depth; air shadow; shadow maps | | | | 0.7; 0.5; 0.05 |

  Totals: V1 ~9-12.5 ms (near 0.7x), V2 / V7 in the deck ~9-12 ms (far mostly hidden), V3 ~5.5-7 ms, orbit ~4-5 ms;
  the near tier at full resolution fits only where few of its pixels hit cloud. The gate measures the real numbers.

### E. Altitude, orbit, column model

- The far tier marches the volume as long as the lateral pixel footprint is below the far fade (1.5-3 km, kept); beyond
  the coarsest level (819 km radius) its samples take the light from the grid-mean column (today's
  `cloud_far_light_depth` path) and the density from the snapshot directly (reference model band-limited). Beyond the
  fade the existing 2.5D column model (`cloud_far_slab` / `cloud_far_column`, sub-grid cover, 3D-light gain) draws,
  composited by the ray split. This keeps the standing ruling (shapes marched, column only beyond the fade). Scaled
  space beyond 1.5e7 m unchanged.
- Retired: the 20 / 34 km two-pass hand-over (`handover_m`): one density and light model (the volume) serves all
  distances inside the levels, so the band's purpose (switching to old pass 1's cheaper light) is gone.

### F. Reuse and retirement

- **Reused**: core weather build, render texels, noise generation, optics, phase function, transfer table and its
  fixtures, threshold / mean / prefilter tables (now also for the sub-cell redistribution), the Sun-disk handling,
  atmosphere coupling, the column model and scaled-space far field, `CloudShadowEffect` interface, composite shaders,
  the cross-fade slots (`cloud_layer.gd`), `cloud_light_cache.gd` / `_schedule.gd` (their toroidal re-anchor, tangent
  frame and budgeted scheduling evolve into the world volume), `cloud_centre_view.gd`'s union-frustum math.
- **Reference path**: today's per-sample procedural march (`cloud_shade_ray`: envelope + shape + detail per sample,
  per-sample light, sub-samples) stays behind `CloudLayer.world_volume = false` / `STARSIM_MARCH_REFERENCE`, at
  divisor 1 and 8 x steps for checks only.
- **Retired - the direction view cache**: it stays the production default only until the world volume passes its
  gates; then one commit (`refactor:`) deletes `cloud_view_cache.gd`, `_layout.gd`, `_planner.gd`, `_schedule.gd`,
  `cloud_view_cache.gdshaderinc`, `cloud_view_cache_test.gd`, `cloud_check` section 15, `render_check --ab_view_cache`
  and the `view_cache` flag; its record and lessons move to `docs/history/weather-clouds.md`; `refs/wip/view-cache-r3r4`
  stays local. Also retired from the production path: the 5-cascade light cache textures, the hybrid light march,
  sub-samples, outline pass 1b, the 20 / 34 km band.

### G. Spike, prototype, gate (before the full build)

- **Counters in every gate measurement**: per pixel the density fetch count and time, the light fetch count and time,
  steps inside occupied space and empty steps (debug output images + histograms); times by shader clock where the
  Godot build exposes `VK_KHR_shader_clock`, else by ablation runs (fetches replaced by constants).
- **Step 0**: instrument today's march with the same counters (where the ~10 x gap sits).
- **Spike S** (days, throw-away allowed): the far tier alone at the real resolution (0.5x cyclopean of the real union
  region; one 1.0x run for scaling) in V3 and V1. (a) Today's per-sample procedural density (one sample per step, no
  sub-samples) with brute-force perfect occupancy (an exact occupancy volume built offline by dense evaluation) and
  constant light; (b) baked meso density from a single-level bake sized to the view (scratch memory free for the
  spike), the same occupancy, constant light plus one dummy light-volume fetch per step. Measure ms, occupied steps
  per pixel, time split density vs light fetches. Rules: (b) <= 5 ms in V3 -> build the prototype; (b) not >= 1.5 x
  cheaper per occupied step than (a) -> the density fetch is not the lever: rework the step rule / termination first
  and report; > 100 occupied steps per pixel in V3 despite perfect occupancy -> redesign the step rule before the
  build; (b) > 10 ms -> stop and report to the user.
- **Prototype**: the plan's first kept increment on `main` behind `world_volume` (default off): world volume (8 levels,
  baked density, occupancy, tau_sun + table + sky), near + far tier, composite; air shadow measured separately, column
  model as is. Weather: one analog file and one ERA5 date (2025-10-25 15:00 UTC).
- Harness: offscreen stereo at the real per-eye size 2 x 3534 x 2838 with the Pimax projections recorded from a
  headset run, `--gpu_passes` per pass, plus one headset run (`Start-VR.cmd --perf-log`).
- Gate views: V1 pad under broken cumulus (cf 0.4-0.6) up and to the horizon; V2 Starbase ascent through the
  1.5-2.5 km deck to ~1 km/s (existing `--frames_every` flight); V3 9 km above a broken deck to the horizon; V4 20 km
  and 80 km whole decks to the horizon; V5 400 km nadir and limb; V6 backlit cumulus and a 3-deg sunset (refresh
  fronts); V7 booster catch: recorded descent through a deck down to the pad. Re-entry is not a gate view: after the
  gate it is the scroll / re-frame test (7.8 km/s, re-frames, provisional light).
- Kill criteria: far tier > 10 ms p99 at 0.5x; near tier > 7.5 ms p99 at 0.7x in V1 / V2 / V7; total cloud GPU
  > 14 ms p99 after the fall-backs; light volume outside cloud_check 8's bounds with 4 entry samples; any visible
  level / slab / refresh seam (radiance step across a fade band or front > 1 %); a still frame not bit-identical.
- Fall-backs in order, each reported to the user: F1 near tier full -> 0.7x (if it started full); F2 density cost:
  level selection one level coarser at range and fewer detail octaves at range (band cut at 2 x footprint), checked
  by the edge measure; F3 R_near 2 -> 1 km (disparity 0.13 px); F4 altitude views only: lower the column model's fade
  footprint (changes the horizon look: user decides). Then stop and report; no further architecture switch and no
  far tier below 0.5x without the user.

## Decisions taken for the builder (Claude's calls - the user may overrule)

1. Both snapshots of the cross-fade pair are baked (exact per-sample cross-fade, no rebake fronts) at the price of
   4 B per cell more.
2. Volume memory ~1.07 GB (+134 MB transient at re-frames), not the research's ~340 MB: baked density for two
   snapshots, four 16-bit light terms (tau_sun, tau_below for the table, tau_above for the sky, tau_moon), 8 levels to
   the horizon from ~50 km. Cloud VRAM total ~1.8 GB, as today (the view cache's 1.13 GB and the light cache's 160 MB
   go). Lean variant: one blended density field rebaked with the light on weight steps (4 B), 8-bit log tau without
   the Moon (3 B), 7 levels: ~0.41 GB.
3. Resolution start: near full or 0.7x, far 0.5x, depth-aware upsample without history; raised only on budget and an
   A/B difference.
4. Level layout, R_near = 2 km, 8-cell fade bands, 1000 km re-frame, upkeep 1.5 ms (spike <= 3 ms at re-frames),
   light surface correction with 4 samples over 2 cells.
5. Provisional light inherited from the coarser level for newly exposed slabs; the never-late rule holds for density
   and occupancy only.
6. Sub-cell detail redistributes the baked cell density with the existing threshold / mean tables (water kept per
   cell); the shape noise's residual above the cell scale is part of the sub-cell field from level 2 up.
7. The far tier's sample grid is Earth-fixed in direction (marched every frame, not cached) so head rotation does not
   move it - the cure for the head-fixed grid's shimmer.
8. Curl-noise edge distortion added (`edge_curl`), default decided by the user from side-by-side images.
9. Air shadow and the shadow maps read the light volume (cheaper; crepuscular rays from far clouds).
10. Today's march kept as the reference path; the direction cache deleted at switch-over, not before.
11. Rate-limited light refresh (lag instead of fronts under time warp / pair switches), documented and switchable.
12. Spike rules (5 / 10 ms, 1.5 x, 100 steps), kill bounds and the fall-back order F1-F4.

## Left to the builder

- Encodings beyond the above (log vs linear tau, packing); taps per cell of the bake.
- The far tier's storage (cube texture vs pooled face tiles, cap 300 MB), the upsample kernel and the cyclopean depth
  pass.
- Slice shapes and order of the refresh; distance-field build method; the exact step constants within the bounds.
- Whether a level's light march reuses coarser levels' tau (hierarchical) or marches to the top.

## Measures of success

- VR, weather on, headset `--perf-log`: pad, deck passage, above the decks, orbit, re-entry, booster catch at 45 Hz:
  frame p99 <= 22.2 ms; cloud GPU <= 11 ms median, <= 14 ms p99 per phase; weather off 90 Hz as today.
- Against the full-resolution reference path (divisor 1, 8 x steps) in V1-V7: render_check A/B median <= 1 %, p99 <=
  the reference's own difference when turned 0.5 px; cloud_grain_check (grain / alpha grain within 25 %, cover / mean
  within 3 %); coverage within 0.02 in every frame of the V2 and V7 flights (no 0.06 in-deck exception any more).
- Feathery edges: on edge pixels (alpha 0.05-0.95) of V1 / V3 / V7 cumuli the edge-band width and the high-pass energy
  of alpha within 15 % of the reference path's [E bound, the plan fixes it from the reference's own run-to-run
  spread]; plus the user's side-by-side judgement.
- Light: cloud_check 8 (slab 5 %, broken p99 10 % / worst 20 %), slab radiance vs MC <= 3 %, backlit transmitted
  radiance within 5 %, Sun disk section 13 unchanged; cloud_far_check limb speckle 5 %, 50 km horizon <= 10 %,
  far field vs volume 10 %. Water: baked cell mean vs the model's within the existing 0.996-1.000.
- Stability: still view bit-identical (`--repeat_identical`); head turned in a still view: shimmer ratio <= 1.10;
  sunset refresh front <= 1 % radiance step; jumps: clouds in the first frame, full volume <= 1 s, coverage within
  0.02 of the reference after it.
- Stereo: `cloud_vr_stereo_test` extended - near+far composite vs an unsplit per-eye march <= 0.5 %; disparity of a
  far-tier cloud vs the per-eye march <= 0.1 px.
- Scroll / re-entry (after the gate): at 7.8 km/s no frame reads a coarser level's DENSITY because a slab was late
  (provisional light allowed, replaced within 1 s); upkeep <= 3 ms in re-frame frames; volume <= 1.1 GB (+ the spare
  level); main thread <= 1.0 ms per frame.

## Open (the user decides later)

- `shape_source` LOBES (stacked-lobe macro shapes) vs today's noise shapes, from side-by-side images.
- `edge_curl` on or off by default.
- De-tiling the 32 km lattice: the bake now evaluates the shape once per cell, so de-tiling the baked part costs
  nothing per frame; the per-sample residual stays tiled - re-measure the look, the user's call.
- F4 (earlier column hand-over at altitude) if the prototype needs it.
- The lean memory variant (decision 2).
