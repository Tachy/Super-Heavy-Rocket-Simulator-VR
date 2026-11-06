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
