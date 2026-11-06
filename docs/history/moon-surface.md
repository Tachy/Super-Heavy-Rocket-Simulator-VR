# Moon surface: history

Superseded states of `docs/subsystems/moon-surface.md`.

## Generator before fix round 1 of Task 2 (5dda73a, 2026-11-04)
- Crater layer not zero-mean: smaller craters entered a band-limited query with their expected mean (stated as
  "-0.2 mm per octave", true only near D = 0.4 m; really ~-5.4e-4 x D per octave), and nothing removed the
  population mean. The detailed surface sat below LOLA: -0.055 m in the tile core, -1.06 m on the global grid
  (measured by the zero-mean test, red run); relief(L) for L far above every band returned the summed mean field.
- Roughness face lattices shared their edge / corner nodes (hashed once by the lowest face index): full detail C1
  across edges. Band-limited square means took nodes beyond an edge from the nearest node of the neighbouring face,
  and the crater square was oriented by the query face's e_s (e_t orthogonalised) while the roughness used the (s, t)
  parallelogram: relief(L) stepped at face edges (1 um pairs up to 1.07 mm apart, ~1160 x the slope bound, at
  L = 0.1-10 m). Replaced by per-face continued lattices joined by `face_blend` and one parallelogram footprint for
  both layers.
- Physics surface, first version (Task 4, 05353f7), superseded in fix round 1 (2026-11-05): the footpad disc was level in
  the site frame and stood on the highest point under it (max filter); on a planar slope s every pad stood r tan s
  higher (1 m pad, 8 deg: 0.14 m) and a rim rock lifted it by its full height - replaced by the pivoting pad (ruling I2).
  The contact lattice was 2^26 per face edge (4.07 cm, sub-lattice RMS ~2 mm): six pads sliding 1 mm per tick cost
  100-155 us per tick then (per-node relief 6 us), 48-51 us after the relief memos - replaced by 2^25 (8.1 cm, 34-36 us).
  Rocks were evaluated with `rock_top` (column scan, ~4.4 us per call; ~60 calls per tick with pivoting pads, ~250 us)
  - replaced by the interpolated footprint column tops. The point contact (r = 0, base ring) ignored rocks (review I1).
- Rock templates, first version (Task 3, superseded by fix A of Task 11, 2026-11-05): 16 smooth star-shaped radial
  functions r = exp(sum of 8 random plane waves, a_k = 0.30 / f_k, f_k 1.5-5 rad), scaled to max r = 1 (mean r ~0.66);
  per rock b/a 0.71 +- 0.15 and c/a 0.50 +- 0.15 (config keys `axis_b_to_a_mean`, `axis_c_to_a_mean`, `axis_ratio_sd`)
  stretched the template with the c axis always vertical (every rock lay on its flattest side). Against the Apollo
  photos (Task 11 compare/ images) the rocks were rounded, flattened "bread loaves"; cover-weighted exposed eta 0.383,
  `max_semi_axis_per_diameter` 1.77 D. The contact top was the template's column tops interpolated bilinearly on the
  96 x 96 footprint grid (`column_top_interpolated`, ~1e-4 of the height from the scan-based `column_top`, ~4 us).
- Near-field shadows, first version (Task 12, 62ee7474; superseded by fix 1, 2026-11-05): every attached rock node took
  its shadow-only MultiMeshes with it, whatever its distance (boulder-field camera jump 5.9-8.0 ms on the rock layer's
  main-thread share; the shadow pass drew casters Godot never sampled beyond the range). Eviction at the pool bound
  released the far node's MultiMeshes to the pool, whose trim then freed and re-allocated GPU buffers for the nearer node
  (34.7 ms spikes), so it was switched off (EVICTIONS_PER_FRAME 0). The fill light's emitter was the eye's view factor
  (G2/G1(v), the oblique-eye rock terms) instead of a hemispherical exitance, and A_h was looked up by elevation (asin per
  lookup). The rock view tables were built at every start (~8 s on 24 threads). The near-shadow map's second factor call
  computed the earthshine's factor too (unused).
- Rock templates, convex fragments (fix A of Task 11, superseded by Task 17, 2026-11-05): 32 angular impact fragments. Each is a convex body: an ellipsoid with
  semi-axes 1 : b/a : c/a (b/a 0.71 +- 0.15 clamped 0.3-1, c/b 0.71 +- 0.15, c/a >= 0.2: the 2 : sqrt2 : 1 fragment law
  of Fujiwara et al. / Michikami et al. 2016, sampled: template means b/a 0.709, c/a 0.496) cut by K hash-seeded planes
  (fracture faces: tangent planes at a jittered Fibonacci set of points on the ellipsoid, pushed in). Fresh fragments
  (70 %, [E]): 16-24 faces cut 15-40 % deep, the whole surface planar (planar share of directions 0.94-1.00), sharp
  edges; eroded ones (30 %, [E]; boulders wear down, Basilevsky et al. 2013 / 2015): 5-8 faces cut 3-12 %, mostly the
  rounded ellipsoid (planar share 0.19-0.37). The fragment is turned to rest gravitationally stable (controller ruling
  R4, Task 11 fix C: no upright balanced slabs, which the Apollo panoramas do not show): its shortest (c) axis near the
  vertical, tilted by a 2-D Gaussian on the tangent plane with 25 deg rms ([E]: slopes, clods, burial; no measured tilt
  distribution retrieved), capped at 60 deg; template rms tilt 25.2 deg, max 45.5 deg, 9 of 32 above 30 deg) and scaled
  so that max r = 1 (20000 Fibonacci directions, the best 32 refined by a pattern
  search, margin 0.5 %). r(dir) is the exact ray exit distance; `normal` the active face's (or the ellipsoid's) normal;
  `column_top(x, y)` exact (the least upper bound of the constraints on the vertical line, ~50 ns); `support_point(u)`
  exact (the ellipsoid's maximum, each face ellipse's maximum or a precomputed edge end / vertex); `bottom_z` / `top_z`
  = the support along -z / +z; `normal_toward(p, reference, tol)` the normal-cone vector nearest a contact's own normal
  (on an edge or a vertex a pad keeps its normal). Meshes = icosphere 4/3/2/1 (2562 / 642 / 162 / 42 vertices) with
  vertex dir r(dir) and the exact face normals (edges soften over one triangle). `exposed_area(cut_z)` = plan-view area
  of the part above the plane z = cut_z, from the column tops of a 96 x 96 grid over [-1, 1]^2. Template construction
  (32 templates): within a rock test case of 0.55-0.64 s in all. Everything from the integer hash.
  Drawn with per-fragment facet normals from the convex constraints (`rock_shape_data`: up to 24 planes and the
  ellipsoid's quadric per template; the active constraint at the interpolated template position): hard facet edges at
  every LOD. LODs icosphere 4 / 3 / 2 / 1 (5120 / 1280 / 320 / 80 triangles, bands >= 100 / 30-100 / 10-30 / 4-10 px)
  plus the icosahedron. User 2026-11-05 (screenshots/moon_surface/compare/user/a17_boulder_real_left_render_right.jpg):
  the rocks looked like crystals - clean convex polyhedra with flat, evenly lit faces and sharp straight edges.

## Task 17 templates and contact queries before fix round 1 (7ad10c70, 2026-11-05)
- Shape constants: erosion radius fresh 0.6-1.4 %, eroded 3.5-7 % of the long semi-axis; eroded fragments 5-8 primary
  planes cut 3-12 % (mostly the ellipsoid); undulation 0-1.2 % fresh / 0-2.5 % eroded; step risers 35-63 deg from the
  tangent plane, sharp inner corners; no major spalls, no through-cracks; vertex normals from central differences over
  0.2 of the LOD edge angle. Review (task-17-review.md, I4): mid-size rocks read as rounded river cobbles, the big A17
  boulder a smooth warped lump; fracture edges drew a staircase at the LOD-0 vertex spacing.
- Shader: flat-normal share per LOD 0 / 0.25 / 0.6 / 0.85 / 1 / 1; crack lines a pure darkening (~2.5 % mean, not
  mean-preserving); dust fillet blended on the plain local vertical below a straight 0.05 D contour (an untextured band
  at the base of large rocks); ROCK_DUST_MEAN 0.343.
- Contact: the rim of a pad over a rock searched by 9 coarse samples + 12 golden-section steps (5 + 8 before Task 17):
  on the faceted, multimodal top it missed the global maximum (pivoting-pad rim-cut case D 0.42 m: 0.02 mm low, normal
  0.115 rad off, gate 0.05); `normal_toward` over 32^3 spatial bins with heap vectors and a Mat3 solve per generator
  triple. contact_speed sliding 51.5-52.8 us alone (gate 50; the convex Task 11 templates 37-39 us).

## Regolith shading before the Task 10 fix round (13a8c9c, 2026-11-05; moved here in Task 13, review N2 of Task 10)
- The rocks were shadowed twice: in the globe's theta_bar and by their explicit terms (review I2); the fix round took the
  rocks' share out of the slope budget.
- Site brightness (IM-2, Sun 6.75 deg, mean 8-bit, Task 9 / Task 10 first version / fix round): 2 m 35.3 / 30.6 / 30.8,
  20 m 38.7 / 36.1 / 36.3, 200 m 37.7 / 36.3 / 36.4; grazing view 25.1 / 22.1 / 22.3; nadir from 1.2 km 36.7 / 35.7 / 35.2.
- The first slope-map version used 8-bit normals (zero slope on code 128 = +0.0039, Godot's rounded 8-bit mips drifting
  by ~1 code): a 4-7 % mean shift of the modulation under a low Sun; replaced by the half-float `.rgbah` maps.

## Rock statistics before Task 13 (a4154314, 2026-11-05; ruling T17-A)
- Cast shadow per cover sigma(e) from the template column tops (64 per side) with per-bin intervals along the light: on
  the non-convex Task 17 shapes ~5 % long (8 deg: 3.30 against 3.14 from an exact raster of the meshes; the binning
  took each strip's extreme column and filled the lit ground under a sun-side bulge).
- View tables from the LOD-2 meshes' convex hull: the seen faces summed every front face (hidden ones in concavities
  too) and the hull regions left the plan view out of the light region, so the excess X counted the ground under a
  bulge a second time (8 deg Sun, eye 60 deg, 30 deg apart: X 0.180 against 0.045 exact, rock flux per cover 2.09
  against 1.86).
- Lommel-Seeliger face gains (mu0 / (mu0 + mu) over the flat face's) while the renderer draws Hapke: at a 7-8 deg Sun
  the drawn rock faces are ~13 % brighter than that law said. The hull's flux excess hid most of it: the GPU rock patch
  at abundance 10 read +2.64 % (gate 2 %); with honest geometry and Lommel-Seeliger +4.25 %; with the Hapke faces +1.52 %.

## Moved from the subsystem file in the final-review fix pass (2026-11-06, final review I-1 / 13a review M-5)
Superseded numbers and per-round records as they stood in `docs/subsystems/moon-surface.md` at bc37da1f (verbatim).

### Model, Rocks (Task 3): template vertex normals
(central differences over 0.5 of the) LOD's edge angle (0.2 until fix round 1, which drew the vertex staircase of fracture edges as shading teeth)

### Model, Touchdown hazard check (Task 5)
 (A cheap pair-sum bound before the disc queries was removed in fix round 2: it never
  fired, not even on all 7,449 candidates with a 1 cm step limit.)

### Model, Baked textures (Task 8): slope-map memory
GPU memory of the four slope maps ~43 MB with mips (was ~11 MB, with the 512 px tiles before the Task 11 fix round).

### Model, Regolith shading (Task 10): residual theta_res values (13a review M-5 / ledger T10 N2)
Values (before Task 12's template mean tan, with the half-spheroid's pi eta in the slope budget): L22 (1 cm) 20.1 deg,
  10 cm 21.7, 1 m 22.3, 10 m 22.7, 100 m 22.9, 1 km 22.97, beyond the bands the regolith's 22.972.

### Model, Rocks drawn (Task 11): retry set
Before Task 12 (review N1 of the Task 11 fix C re-review) a refused node was never retried and FIFO chose.

### Model, Rocks drawn: flat-normal share per LOD
Task 17 fix round 1 raised the flat-normal share of LOD 1-2 from 0.25 / 0.6 to 0.4 / 0.75.

### Model, Near-field shadows and fill light (Task 12): wall Sun visibility
Task 12 fix 1 gated every wall by the receiver's V, which zeroed the fill exactly in terrain shadow.

### Model, Near-field shadows and fill light: the rocks' fill emitter (superseded by Task 17)
The rocks' call site (`moon_rock.gdshader`) still passes
  Smith's G1 alone (owned by the rock-look task at the time of fix 2: open point).

### Validation, environment seams (band-limited)
Before fix round 1 of Task 2 the band-limited step across face edges was up to 1.07 mm, ~1160 x the bound.

### Validation, touchdown hazards (Task 5): IM-2 figures with the Task 3 templates
IM-2 with full detail: the target moves 28.3 m (slope 3.19 deg, hang 0.461 m, clearance 0.572 m)

### Validation, touchdown hazards (Task 5): synthetic rock case with the Task 3 templates
(D 8.70 m, pad support 6.18 m above the ground) moves 60.0 m (slope 1.74 deg, hang 0.586 m, clearance 0.738 m, 337-420 ms cold)

### Validation, Regolith shading (Task 10): drawn slope variance
Before the transfer functions, with "octaves >= L drawn", the model was 1.26-2.40 x the mesh.

### Validation, Site brightness (Task 10): LROC NAC comparison
  Against the LROC NAC frame M1434867405 (90 cm pixels, a different Sun; `screenshots/moon_surface/task10/
  nadir_lroc_nac_left_task10_middle_fix1_right.png`): before the fix round the render lacked the 5-20 m craters - not a
  density error (config N(>D) = 0.0336 D^-2 is Minton et al. 2019 Fit 1, the median of the modern counts; it predicts
  336 craters > 10 m and 84 > 20 m per km^2) but a drawing gap: a level-13 leaf (5.2 m spacing) draws craters only from
  ~4 L = 20-40 m and nothing drew the smaller ones (Task 10 review C3). With the crater textures (s2 / s3) the render
  now shows the 2.5-20 m population; it stays fainter than the NAC frame's because the craters cast no shadows (open
  point). No crater count was made on the images.

### Validation, detail off identity (Task 10)
A first version that multiplied the old expression by the factors 1.0 differed in 2-7 px by 1 LSB, so the detail-off branch keeps the old expression.

### Validation, Drawn rocks (Task 11): radiance gate numbers with the Task 11 statistics
Sun 7 deg d = 0.1 / 0.2 / 0.4 / 0.8 m +1.33 / +1.12 / +1.11 / +1.14 %, Sun 15 deg d = 0.1 / 0.4 m +0.36 / +0.45 %,
  Sun 30 deg +0.10 / +0.04 %, earthshine only (Earth 7 deg) +1.33 %; against the law's all-statistical mean +2.06 /
  +1.86 / +1.79 / +1.63 %, +0.64 / +0.71 %, +0.23 / +0.15 %, earthshine +2.06 % (the patch's drawn cover 0.0150 /
  0.0120 / 0.0093 / 0.0064 against the law's 0.0119 / 0.0089 / 0.0064 / 0.0043).

### Validation, Drawn rocks (Task 11): camera jump with the five-LOD Task 11 rocks
the rock layer's main-thread share max 1.08-1.19 ms, p99 1.0-1.1 ms (gate 5 ms; largest attach 0.41-0.48 ms); the
  quadtree's own update without rocks max 13.45 / p99 12.75 ms (`--jump_baseline`). Without a drawn frame between the
  updates (the earlier harness) one call per run stalled 17 ms on the full staging pool, on whichever call hit it
  (terrain mesh, MultiMesh mesh or buffer).

### Validation, Near-field shadows (Task 12): rock gate with the Task 12 view tables
rock gate at Sun 7 deg, e 30, g 60 against the realised drawn cover: abundance 1 -0.26 .. -0.28 %, 3 -1.14 ..
  -1.30 %, 10 -0.01 .. +0.58 % (Task 11 fix C: +4.93 % at 10; gate 2 %)

### Validation, Near-field shadows (Task 12): handover at the range boundary, first Task 12 numbers
down-Sun 1.000 / 1.003 / 1.001 / 1.001, across 1.005 / 1.006 / 1.000 / 1.000, into the Sun 0.999 / 0.980 / 0.981 /
  0.998 (dark ground, 0.3 % of the down-Sun radiance; gate 2 %). Before Smith's visible-lit fraction and the 2-deg view
  grid it was +2.0 .. +4.2 % (down-Sun).

### Validation: the dated per-round blocks (Task 12 fix 1 / fix 2, Task 17, Task 17 fix round 1)
- Task 12 fix 1 (2026-11-05). Red step: `moon_shadow_check` with the game files before Task 12 fails (no shadow: "umbra"
  0.0257 against lit 0.0305, half-shadow 13.8 m); the lander check without the map fails (tip 129.4 m, halfway 0.081 of
  the lit 0.125). Green: the 2 m box 15.98 m (16.29 +- 1), umbra 0, penumbra 1.25 m; earthshine-only on/off 0 px; off
  against the Task 11 shader 0 px; the HLS model (63 meshes, 53.7 m tall, feet on flat ground, Sun 20 deg) shadow tip
  147.7 m (half-shadow point) against the geometric tip 148.0 m (window [tip - penumbra 4.3 m, tip + 1 m]), its shadow
  halfway keeps no Sun; the setting's file round trip. Relief identity (IM-2 2026-11-13T06:00, Sun 0.07 deg, the eye in
  terrain shadow, detail on): near shadows on / off with the fill 0 px; fill on / off 1.6 % of the pixels differ (the
  partly sunlit receivers). That view's near floor gets neither earthshine nor fill (99.5 % of its ground pixels are code 0
  with the fill off even at a 60 x brighter exposure; its eye adapts to 0 cd/m^2, so the camera exposure is not
  calibrated there): its ground-row ratios (x9.7 fix 1 ungated, x1.57 receiver-gated, x2.23 per-wall) divide by near-zero
  codes and are not a measure. Shadowed floors at Sun 6.75 deg (IM-2 2026-11-22, 200 m, camera exposures 8 / 0.004 s /
  ISO 160 and 4 / 0.02 s / ISO 1600, Task 12 fix 2, review N2): pixels below 2 % of the sunlit median (1192 cd/m^2)
  2.77 cd/m^2 with the fill (p10 / p50 / p90 1.67 / 2.31 / 3.46), 0.04 cd/m^2 without (no earthshine there: the Earth is
  below the horizon), 0.23 % of the sunlit median; expected [E] (E / pi) A_h(mu_r) sum sin^2 h mu0 A_h(mu0) / 16 with
  rim walls h 5-15 deg lit at mu0 0.2-0.4 over half the azimuths ~ 0.1-1 % of the sunlit floor. Sunlit ground x1.0004.
  Boulder field at the eye 2 m with the near shadows off (Task 12 fix 2, N1): no glow or blotch above the eye's
  horizontal plane (the rock view-elevation clamp; A/B without it the artifact returns at the same line). Gates
  unchanged (detail patches -1.49 .. +0.23 %, rock gate at abundance 1 / 3 / 10 as above); handover (A11) across
  1.011 / 1.002 / 1.000 / 1.000, down-Sun 1.001 / 1.002 / 1.001 / 1.001, into the Sun 0.983 / 0.990 / 0.983 / 0.995;
  sky_radiance_check PASS (IM2_READ_EXPOSURE 2.5), atmosphere_check PASS. Core: the view tables' cache round trip bit
  for bit, the key guards burial law and format, a truncated file is refused. Camera jumps (moon_surface_check
  --rocks_only, 2 runs): IM-2 max 4.52 / 3.26 ms (pass); boulder field 30.7 ms (one 29 ms pool allocation, 3 frames
  over) / 4.76 ms (pass); without any rock casters the boulder jump also had single 22-28 ms allocation stalls (A/B).
  Load: the statistics build 1311 ms without the cache file, 439 ms with it (the whole render_check process 36.5 / 35.5 s;
  the ~8 s of Task 12's first version were the core tests' process - the game shared the build with its other loading).
- Task 17 (fractured rock templates, 2026-11-05): core [slow] tags 247 / 250 (fail: lunar campaign 8-14, seed 14 at
  8 deg 1.39-1.51 m - also at the Task 12 head, 1.52 m; contact_speed sliding 51.5-52.8 us alone, 37-39 us with the
  convex templates; one rim-cut case of "A rock at the rim lifts the pivoting pad": heights within 0.02 mm, normal
  0.115 rad off on a near-tie of two facets). Campaigns 1-7 / 15-21 7 / 7 (offsets 0.04-0.34 / 0.16-0.84 m). IM-2
  hazard: offset 28.3 m, slope 3.12 deg unchanged, step 0.467 -> 0.468 m, bell clearance 0.552 -> 0.548 m. Summed
  roughness at the site + rock tops 4.003 -> 4.014 mm (residual 3.62 mm kept: the formula gives 3.610). Rock patch
  radiance gates unchanged (within -0.25..+0.06 %). GPU stereo mock: IM-2 2 m 17.7 ms, boulder field 48.2 ms (drawn
  vertices +1 %). Camera jumps: IM-2 rock share 3.6-3.9 ms; boulder field one 24-31 ms pool allocation per run (2 of 2,
  T12-C). Images: screenshots/moon_surface/task17/.
- Task 17 fix round 1 (2026-11-06): core [slow] tags 251 / 252 (fail: lunar campaign 8-14, seed 14 at 8 deg 1.52 m,
  pre-existing at the Task 12 head); pivoting-pad cases: every rim-cut normal 0.0000 rad off, heights within 1e-4 mm of
  the brute force (now refined along the rim angle; one peak-under-edge case 0.375 mm above its pattern-search
  reference, inside the 1 mm gate); new `arc_peak` test: exact minus a 1e5-sample brute force <= 4.9e-6, the nudged
  point reproduces the value to 1.3e-9. contact_speed alone 25 / 44-46 / 12 us. Campaigns 1-7 / 15-21 7 / 7 (offsets
  0.03-0.29 / 0.16-0.85 m), 8-14 offsets 0.03-0.68 m + seed 14. IM-2 hazard: offset 28.3 m, slope 3.12 deg unchanged,
  step 0.468 m, bell clearance 0.548 -> 0.560 m; synthetic step 0.525 m, bell 0.649 m; rock D 8.70 m pad support 5.19 m
  (proud bound 9.14 m). Summed roughness site + rock tops 4.005 mm (residual formula 3.614 mm, 3.62 kept); regolith
  theta_bar 23.356 deg, S^2 0.5858. Shape statistics: volume per D^3 mean +0.94 % (single templates within 11.5 %),
  extent ratios within 0.46 %; dust share 0.3403 (ROCK_DUST_MEAN 0.340); 8 of 32 templates cracked. GPU: moon_shadow_check
  PASS, smoke PASS. Images: screenshots/moon_surface/task17/fix1/.

### Open points, lunar landing campaign: Task 5 fix round 1 figures and the two-foot stance
At Task 5 fix round 1: 21 / 21, misses 0.01-0.91 m (seed 17 0.91 m and seed 5 0.89 m are the closest to the gate; seed 7, 1.12 m before
  the fix, now 0.09 m). The earlier two-foot stance (~50 s, ~2 cm/s creep) had two causes, both fixed: the guidance
  held the attitude with the RCS after the first contact (a rate damper about the line through two feet - now
  released at contact) and the regularised leg friction crept under any tangential load (now Coulomb stick-slip,
  `contact::leg_contact`).

### Diviner map required by the lunar mission (superseded 2026-11-06)
Standing decision until U-2: a missing Diviner map was an error, "not a silent fallback": `load_rock_abundance_map` (via
`load_rock_abundance_file`) threw a German `ConfigError` naming `fetch.py diviner_rock_abundance`, and `LunarRocks` with
`abundance_source = "diviner"` and no map threw `std::invalid_argument`; `tools/package.ps1` refused to build a package
without the map in the base data. Replaced because the map is an 88 MB optional data part (user 2026-11-06).

### Mission start order: search started inside begin (superseded 2026-11-07)
Until lunar sites Task 8 `sim::begin_lunar_landing_mission` placed the HLS, started the touchdown search over both polar
approach directions (Task 7: after planning the outbound flight inside the call) and only then returned its status - the
event line stayed blank while the outbound plan ran (~2-15 s). Now the status comes first and the first poll plans.
