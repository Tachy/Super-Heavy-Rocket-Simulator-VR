# Moon procedural near-ground surface

Spec `docs/superpowers/specs/2026-11-04-moon-procedural-surface-design.md`, plan
`docs/superpowers/plans/2026-11-04-moon-procedural-surface.md`. Current state after Task 10 (craters, regolith
roughness and rocks in the core; the composed physics surface and disc footpads; the touchdown hazard check; the
bindings; the terrain mesh down to the cm detail; the baked textures; Hapke reflectance for globe and terrain; the
regolith detail shading with the statistics of the unresolved surface; rocks as geometry and near-field shadows in the
renderer follow).

## Model
Measured statistics in `config/moon_surface.toml`; the generator `environment::LunarSurfaceDetail` (craters +
roughness, Task 2) draws them as a pure function of the body-fixed direction; the rocks (`environment::LunarRocks`, Task 3) stand
on top of that relief.
- Small craters: equilibrium population, cumulative N(>D) = 0.0336 D^-2 per m^2 (2.2 % of geometric saturation),
  0.01 m - 2 km; 70 % degraded (d/D 0.07), 30 % fresh (d/D 0.13, rim height 0.024 D).
- Regolith roughness (what is not a crater or rock): self-affine, RMS height in a window W = 3.7 mm (W / 0.125 m)^0.78
  (1 m: 1.9 cm, 20 m: 0.19 m, 100 m: 0.68 m). How the config is consumed: `roughness.rms_height_at_baseline_m`
  (3.7 mm, [D]) is the RESIDUAL the roughness generator draws; `roughness.rms_height_measured_total_m` (4 mm, Guo
  et al. 2021) is the measured total of the natural surface and is used only as the target of validations of the
  summed surface (craters + rocks + roughness). Derivation in Sources, Regolith roughness.
- Rocks: N(>D) = 1.54e-3 D^-2.286 per m^2, 0.01 - 10 m (N(>1 cm) = 57 per m^2, N(>10 cm) = 0.30 per m^2,
  N(>1 m) = 1.5 per 1000 m^2, N(>10 m) = 8 per km^2; area covered 3.1 % for 0.01-10 m, 0.68 % for 0.5-10 m). Axis
  ratios b/a 0.71, c/a 0.50 (sd 0.15), burial 0.3 +- 0.15 of the height, single-scattering albedo 1.35 x regolith;
  D = plan-view-area-equivalent diameter (the generated rocks cover exactly the law's disc area).
  Local abundance: Diviner rock abundance (60 S - 60 N) divided by `rocks.abundance_reference_area_fraction` (0.0068,
  the disc area the law gives for 0.5-10 m, lower limit `rocks.abundance_reference_min_diameter_m`); uniform elsewhere.
- Hapke (643 nm, highland proxy): w 0.429, b 0.220, c 0.514, B_S0 1.61, h_S 0.054, B_C0 0, h_C 1, theta_bar
  23.657 deg, K 1.

### Generator (Task 2)
- Determinism: integer hash only (`surface_hash`, SplitMix64 chain; streams `(stream << 8) | face` for craters,
  roughness, rocks), no library random numbers, no state; bitwise identical across calls, threads and instances.
- Cells: cube-face cells with exactly the mapping of `game/world/terrain/cube_sphere.gd` (face, level, i, j = the terrain
  quadtree key). Cells beyond a face edge are found per face from the point's continued face coordinates, so features
  are continuous across edges and corners.
- Craters: one diameter octave (0.01 m, doubling, to 2 km) per cell level, cells >= 3.34 D_max nominal (reach 1.5 D <=
  0.45 cell; cells are >= 0.7 nominal wide everywhere), 1-5 expected per cell; count Poisson from the hash with the
  exact cell area (area element at the cell centre), position uniform in the cell, D by the inverse cumulative SFD,
  fresh / degraded by `degraded_fraction`. Profile: parabolic bowl rim - depth + depth (r/R)^2 inside, rim
  ((r/R)^-3 - 1/27) / (1 - 1/27) outside to 3R (zero there: the earlier (r/R)^-3 cut at 3R left a step of rim/27);
  degraded rim scaled with depth (0.0129 D). A crater is generated only where LOLA does not resolve it: D <= 5 x
  band_start at its centre.
- Zero mean: the profile's volume is negative (-0.37 depth R^2), so a crater population lowers the mean surface by
  about -5.4e-4 x D per diameter octave [D, 2D] (summed: -0.044 m for craters <= 100 m in a tile core, -1.08 m for
  craters to 2 km on the global grid). LOLA heights are area means that already hold this, so every octave enters as
  its deviation from the population mean (`CraterOctave::mean_relief_m`, density x expected volume, at the point's
  band_start). The detailed surface keeps LOLA's mean height; relief(L) is 0 for L far above every generated band.
- Roughness: value noise with quintic fade on one face lattice per octave (spacing 5 mm, doubling, to 2 x the coarsest
  band start); the number of cells per face edge is a prime, so no lattice line is shared between octaves (no grid
  pinning). Every face has its own lattice, continued beyond its edges; within 0.01 face units of an edge (+-13.6 km,
  `face_blend_band`, ~2 % of the surface) the field is the `face_blend` of the 2 (3 at a corner) face fields:
  smoothstep weights h(s) h(t), normalised by sqrt(sum w^2) so the blend of independent fields keeps the variance.
  The field depends on the direction only and is C1 across edges and corners. Octave amplitude
  A (local spacing / 0.125 m)^0.78 with the local cell size (sqrt of the area element, 0.84-1 of nominal); A is
  derived at construction so that the median plane-detrended RMS in 125 mm windows (Guo's statistic, 25 x 25 samples,
  1536 windows at hashed places) equals `rms_height_at_baseline_m` (3.7 mm). Octaves fade out between band_start / 2
  and band_start.
- band_start_m = tile pixel scale x tile_weight + global spacing (1.9 km) x (1 - tile_weight): continuous over the
  1 km tile blend (20 m in a tile core).
- Band limit L > 0 (terrain nodes, coarse queries): the mean of the full relief over the footprint p + a e_s + b e_t,
  |a|, |b| <= L/2 - the face lattice's (s, t) parallelogram (axes up to ~25 deg from orthogonal near face corners),
  one footprint for both layers; inside the face blend band the `face_blend`-weighted set of the faces'
  parallelograms. Craters with D >= L/8: exact mean (polygon integral as four centre-to-edge triangles, each a
  closed-form polar wedge of the profile; for L < R/16 inside one smooth part the bowl's exact quadratic mean or a
  second-order Taylor of the flank); smaller craters fade out over L/16 - L/8 (their mean deviation is ~0). Roughness
  octaves with spacing >= L/8: exact mean over the own face's parallelogram (separable fade integrals); over another
  blend face's parallelogram (sheared in this lattice) a composite midpoint rule with 3 samples per cell for
  octaves >= L/4, ramping to the own parallelogram at L/8; finer octaves (zero mean) fade out over L/16 - L/8.
  relief(L) is continuous across face edges and corners for every L.
- Cost (Release, this machine): full-detail query ~7 us, 1 m mean ~19 us; inside the face blend band ~59 us (half
  full detail, half 1 m: 2-3 lattices and the cross means); construction 0.7-1.0 s (the roughness normalisation).

### Rocks (Task 3)
- Templates (`RockTemplates`): 16 star-shaped radial functions r(dir) = exp(sum of 8 random plane waves, a_k = 0.30 / f_k,
  f_k 1.5-5 rad) scaled to max r = 1 (sampled on 20000 directions, margin 0.5 %), so a rock's semi-axes a, b, c bound
  it; the shape fills them only partly (mean r ~0.66). meshes = icosphere 4/3/2/1 (2562 / 642 / 162 / 42 vertices) with
  vertex dir r(dir), true surface normals. Per template: vertical extent `bottom_z` / `top_z` (same 20000 directions),
  `column_top(x, y)` (scan from z = 1 down in 48 steps + 40 bisections; the one criterion `rock_top` also uses) and
  `exposed_area(cut_z)` = plan-view area of the part above the plane z = cut_z, from the column tops of a 96 x 96 grid
  over [-1, 1]^2. All values [E] (appearance only); everything from the integer hash.
- Population (`LunarRocks`): one diameter octave (0.01 m, doubling, to 10 m: 10 octaves) per face-cell level, cells >= 4 x
  D_max nominal; count Poisson (exact cell area x abundance factor at the cell centre x octave density), position uniform
  in the cell (each rock belongs to exactly one cell: found from both sides of a face edge, no duplicates), D by the
  inverse cumulative SFD, b/a 0.71 +- 0.15 (clamped 0.3-1), c/a 0.50 +- 0.15 (0.2 - b/a), burial share f of the height
  0.3 +- 0.15 (clamped 0-0.6, symmetric about the mean, so the mean share stays 0.3), template and yaw random, the c
  axis vertical (rests on its flattest side, [E]), albedo = `albedo_to_regolith`. Normal-like draws are the sum of 6
  uniforms (no library distributions).
- Size convention: D is the plan-view-area-equivalent diameter: the rock's area seen from above (the part above the
  ground) is pi D^2 / 4. Per rock a = (D / 2) sqrt(pi / ((b/a) exposed_area(cut))), so a is ~0.84 D for a typical rock and at
  most 1.77 D (`max_semi_axis_per_diameter`: b/a 0.3, burial 0.6, least filled template); the visible long dimension is
  ~1.1-1.3 D. This is the area definition of Diviner rock abundance (the normalisation 0.0068 stays a disc integral and
  is exact); the Surveyor counts' own convention is an open point.
- Burial by height (the config's definition): ground at cut = bottom_z + f (top_z - bottom_z) of the template,
  `burial_m` = c x cut (depth of the centre below the ground; negative = centre above it).
- Ground: the rock stands on LOLA + `relief_m(base, 0)` at its base point, on the plane tangent there; no rock is tilted.
  On a slope the ground falls away on the downhill side, so part of the underside is exposed and the uphill side is
  buried deeper (Open points).
- Rock top: `rock_top(rock, unit)` takes the template's `column_top` at (x/a, y/b) and returns height above the sphere =
  base + c z_top - burial + d^2 / 2R with the outward normal (template normal through the inverse axis scaling); nullopt
  where the rock is not above its base height. "Above" is against `base_height_m` only, not the local ground at `unit`.
- Abundance: `abundance_source` selects it. "uniform": `abundance_scale` everywhere (a passed map is ignored).
  "diviner": `load_rock_abundance_map(rocks, data/terrain)` reads the Diviner map (`RockAbundanceMap`, cylindrical
  float grid, area fraction, |lat| <= 60 deg, no data = NaN); `LunarRocks` without a map then throws a German error
  naming `fetch.py diviner_rock_abundance`, like the LOLA loader (decision: error, not a silent fallback). Inside the
  map the factor is map / `abundance_reference_area_fraction` (0.0068), blended smoothstep to 1 over the last 5 deg
  below the map edge (`abundance_blend_band_rad`, 150 km: no step). No-data pixels are filled at load (`fill_rock_abundance_gaps`: mean of the valid 8-neighbours, front by
  front, deterministic), so the factor is continuous across gaps; an all-NaN map gives the uniform law.
- `cover_fraction(min, max)`: area share (pi/4) D^2 integrated over the law = the generated rocks' real plan-view area
  (0.0311 for 0.01-10 m, 0.00678 for 0.5-10 m). `rocks_touching(unit, r, min)` returns rocks with base within r + a
  (hazard / contact queries; cells searched within r + 1.77 D_max).

### Physics surface (Task 4)
- `environment::DetailedLunarTerrain(lola, detail, rocks, SurfaceDetailOptions{detail, rocks})` is the Moon's
  `Terrain` for the physics: max(LOLA + relief, rock tops). Fidelity flags `moon_surface_detail` / `moon_rocks`
  (default on; rocks need the detail). Detail off forwards every override to `LunarTerrain` (bit-for-bit, incl. the
  10 m normal step) and `support_under_disc` is the point query at the disc centre.
- Detail on, point queries (`height_m`, `height_at_direction_m`, `height_at_fixed_m`): LOLA `height_at_direction_m`
  + `relief_m(unit, 0)` + every rock (>= 0.01 m) whose top covers the point (relief ~2.4 us with the memos, 6 us before;
  `rocks_touching` at radius 0 ~6.6 us). `normal_fixed`: central differences over +-1 m (`detail_normal_step_m`, [E] the HLS pad radius;
  `normal_step_m(fixed)` reports it). `resolution_m` / `finest_resolution_within_m`: 0.01 m. `max_height_m`: LOLA's +
  a practical relief bound (two fresh rims per crater octave, mean-field shifts, the value-noise maximum) + twice the
  largest rock semi-axis. `height_limited_m(unit, L, D_min)`: relief averaged over L and only rocks >= D_min
  ((0, 0) is the full point query, bitwise).
- Contact surface (`support_under_disc(centre, up, r)`, `contact_height_m`, `contact_ground_m`, `contact_rock_top`):
  the pads feel LOLA + the full relief interpolated bilinearly on a cube-face lattice of 2^25 nodes per face edge
  (8.1 cm nominal, 6.8-8.1 cm) and the rocks >= 5 cm (`contact_min_rock_diameter_m`) with their contact top (the
  template's column tops interpolated on its 96 x 96 footprint grid, `RockTemplates::column_top_interpolated`: within
  1e-4 m of `rock_top` for the tested rocks, ~20 ns instead of ~4 us). Both scales lie inside the few-cm sinkage of a
  loaded pad in regolith ([E]; a 5 cm rock buried 0.3 of its height stands 2-3 cm proud; the omitted sub-lattice relief
  has an RMS of 4.4 mm, 2.5 mm at 4 cm - the 4 cm lattice put six sliding pads over the 50 us budget, see Cost).
- Pivoting pad (ruling I2, fix round 1 of Task 4; the HLS pad sits on a ball joint): heights are referred to the disc
  centre's vertical (a point d off the centre counts d^2 / 2R lower); the pad's plane is the least-squares plane
  through the contact ground at a pattern of 25 samples (centre, 8 at r/2, 16 on the rim, [E]); the support is the
  plane's height at the centre (the samples' mean) plus the highest protrusion of the contact surface above the plane
  within the disc - a ground sample or a rock. On a plane the support is the plane's height under the centre (a point
  foot); a rock lifts the pad by its protrusion above the fitted plane (a level pad would take the full rock height).
  Normal: the plane's, or the carrying rock's at the point where it protrudes most. Rocks are taken in the order of an
  upper bound of their protrusion (top bound - plane at the base + slope x a) with early stop; first every peak under
  the disc (cached per rock, free), then: inside the disc the protrusion of the bilinear contact top above the linear
  plane is greatest at a footprint column node - found by climbing over the nodes from the peak; a rock the rim cuts is
  searched along its rim arc (5 coarse samples, 8 golden-section steps) and climbed inward from the best rim point.
  Peaks: per template the highest column top (41 x 41 grid + pattern search, computed once). r = 0 (`contact_height_m`,
  the base ring): the ground at the point and every contact-size rock covering it (normal: the ground's over +-1 m or
  the rock's).
- Upper bound (`support_upper_bound_m`, `SurfaceField::below_disc_upper_bound` / `height_below_upper_bound`): legs skip
  the query of a foot or base-ring point above it (the same leg result: no depth, not touching). r = 0: max(ground at
  the point, highest rock top bound of its cache cell) - rigorous. r > 0: max(ground at the centre + 2 r, rock bound)
  + 0.98 x 2 x 2 r ([E] `bound_slope` 2: the contact ground stays within 2 r of the centre's over the pad - fresh crater
  walls reach 0.52; [D] `bound_tilt_gain` 0.98 bounds the fitted plane's tilt x r by the samples' spread).
- Cache: per face cell of level 18 (10.4 m nominal) the lattice nodes (128 x 128, filled lazily, computed outside the
  lock, one locked pass per disc) and the rocks >= 5 cm that may touch the cell (`rocks_touching` from the cell centre,
  radius = largest centre-to-corner chord x 1.01) with their peaks and top bounds; at most 256 cells of each kind (LRU,
  ~32 MB of nodes; sized for the parallel touchdown search: 64 cells thrashed there).
  Node values and rock lists are pure functions of the cell, so a cached query equals a fresh one bit for bit
  (tested). A point needs only its own cell. `LunarSurfaceDetail` memoises per thread the generated crater cells
  (2048 direct-mapped slots, ~1 MB) and the band-start-dependent octave terms (crater mean field, roughness top
  weights): the same values, a full-detail relief point 2.4 us instead of 6 us.
- Cost (Release, this machine, `[contact_speed]`, run alone as CTest `contact_speed`; whole `leg_contact` of the HLS:
  six 1 m pads on a 12 m footprint + 12 base-ring points through `TerrainField`, medians of 7 x 200 ticks): resting
  25 us, sliding 1 mm per tick (0.2 m/s at 200 Hz) 34-36 us, descending from 50 m at 2 m/s with 0.2 m/s drift 12 us
  (mean of 5,000 ticks; feet and ring above their bound skipped), first tick on a cold surface 0.2 ms (budget < 50 us
  per tick, gated). Under the parallel suite the same case measured 46 / 58 / 15 us, hence the serial run.
- Legs (`contact::leg_contact`): each foot's support is `LandingSurface::below_disc(e, n, foot_pad_radius_m)` (once
  per foot per tick, skipped above `below_disc_upper_bound`); the base ring asks `height_below` (height and water
  only: the contact surface at the point incl. rocks, skipped above `height_below_upper_bound`). `TerrainField` routes
  `below_disc` (r > 0) to `support_under_disc` and `height_below` to `contact_height_m` only for a
  `DetailedLunarTerrain` with detail on; otherwise both are `below` (bit-for-bit today's point feet) and the bounds
  infinite. Vehicle config `[stage.landing_legs] foot_pad_radius_m` (default 0 = point foot); `starship_hls.toml`
  1.0 m [E] (drawn pad, user-approved 2026-11-04).
- Session: `sim::shared_moon_surface(config_dir, data_dir, options)` caches the composed surface per (directories,
  options) and its parts (LOLA via `shared_lunar_terrain`, one `LunarSurfaceDetail`, one `LunarRocks` with the
  Diviner map, loaded only with rocks on). `FlightSession::set_moon_terrain(DetailedLunarTerrain)` keeps
  `moon_terrain()` = LOLA (telemetry `altitude_terrain_m`, planners, game queries) and uses
  `moon_surface()` for the ground and the legs: the given surface when its options match the session's flags, else a
  rebuild from its parts; `set_moon_surface_detail` / `set_moon_rocks` re-apply at once and rebuild a lunar landing
  site on the new surface (refused while a body rests on the Moon; the flag then keeps its old value). The
  `lunar_landing` mission loads all parts
  (`{true, true}`) and hands them to the session. `set_moon_terrain(LunarTerrain)` stays the LOLA-only surface.

### Touchdown hazard check (Task 5)
- `mission::select_touchdown_point(const DetailedLunarTerrain&, lat, lon, approach_azimuth, footprint_radius, p,
  TouchdownHazardLimits)`: the candidates of the LOLA selection (20 m grid, nearest first, within
  `touchdown_search_radius_m`) pass the LOLA checks first (footprint plane slope, roughness 0.5 m, glide cone - the
  cheap pre-check); the first that also passes the hazard check on the composed surface is the target. Detail off:
  the LOLA selection on `lola()` bit for bit (diagnostics NaN).
- Hazard check, for every roll (the roll at touchdown is not controlled): the feet stand on `support_under_disc` (the
  pivoting pad the legs feel; frame as `contact::TerrainField`: verticals through the candidate's tangent-plane
  points, heights along its up), sampled on the footprint circle at <= 0.5 pad radius ([E]
  `hazard_roll_spacing_per_pad_radius`; HLS: 156 azimuths, 26 rolls of the six legs). Per roll the hull rests on the
  facet(s) of the feet's upper convex hull above the axis (`mission::touchdown_rest_planes`: the foot triangles
  containing the centre whose plane is highest there and has every foot on or below it; two facets when the centre
  lies on the edge between two opposite feet - common for six legs: the hull balances there and may tip onto either
  side, both are checked). Each facet: slope within `slope_limit_rad`; the deepest foot hanging below it within
  `max_foot_step_m`; the contact ground under the hull base disc (0.5 m grid [E]) and the contact tops of the rocks
  >= 5 cm that can rise above it (an upper bound decides; then 13 x 13 samples over the rock's +-a square [E]) at most
  `bell_clearance_m` above it. (A cheap pair-sum bound before the disc queries was removed in fix round 2: it never
  fired, not even on all 7,449 candidates with a 1 cm step limit.)
- Memo: the hazard result of a candidate does not depend on the approach azimuth (only the glide cone does); it is
  memoised per candidate centre for the surface's parts (by ownership), options, footprint, slope limit and hazard
  limits (one context, <= 50,000 entries [E], thread-safe). The mission's reachability check (both azimuths) and the
  descent planner then pay the hazard check once.
- Parallel search (Task 5 fix round 2): with detail on, the candidates are checked ring by ring (`touchdown_grid_m`
  wide, nearest ring first) on `LunarDescentParameters::touchdown_search_threads` threads (0: hardware threads less
  two, as the weather builds; 1: serial), taken in azimuth order so that the candidates in flight share surface cells.
  The first admissible candidate of the first ring that has one, in the serial (offset, grid) order, is taken: the
  same point as the serial search bit for bit (test "The parallel touchdown search picks the serial point bit for
  bit": serial against 3 and 22 threads, each on a fresh memo, point 60 m off / refused). `check_hazards` is pure;
  the contact cache copies node values under its lock, computes missing nodes outside it and keeps rock cells alive by
  `shared_ptr`, the crater memos are per thread - no shared mutable state outside the locks.
- Limits from the vehicle config (`mission::touchdown_hazard_limits(StageConfig)`, [D]): `foot_pad_radius_m` (HLS
  1.0 m); `max_foot_step_m` = `stroke_m` (0.6 m) - a conservative bound on the hanging feet and on the tilt to a
  third foot, not crush absorption: statically no core crushes (4.5 MN per leg against ~0.5 MN lunar weight, 6-9 mm
  elastic), so the rigid hull rests on its rest facet and the other feet hang; a rocking onto another facet tilts it
  by at most ~atan(0.6 / 12) = 2.9 deg; `bell_clearance_m` = (lowest engine-group `station_m` - `foot_station_m`) -
  `stroke_m` = (0 + 1.5) - 0.6 = 0.9 m (assumes the group's thrust point `station_m` is its exit plane - the HLS's
  exits are flush with the aft end, detail file `engine_recess` 0; a lander with recessed nozzles needs its exit
  station - and the legs may stroke fully); `base_radius_m` = `diameter_m` / 2 = 4.5 m (the bells lie inside the
  hull, whose aft edge is at the same station). `TouchdownPoint` with the check: `height_m` the rest plane under the
  centre (mean over the rolls), `slope_rad` the steepest rest facet, `max_foot_step_m` the deepest hang,
  `min_bell_clearance_m` (the gap with the legs fully stroked).
- Users: `LunarDescentPlanner::plan(..., const DetailedLunarTerrain&, ..., TouchdownHazardLimits)`;
  `FlightSession::plan_lunar_landing` takes it when `moon_surface()` is a `DetailedLunarTerrain` with detail on and
  the legs have pads; the `lunar_landing` mission's reachability check the same. There is no live re-selection after
  a dispersion in the descent (the guidance flies to the planned point): an open point.

### Bindings (Task 6)
- `StarsimCore.load_terrain` also builds the composed surface (`sim::shared_moon_surface`, detail + rocks; without the
  Diviner map detail only, with a German line). `StarsimCore.set_moon_surface_detail` / `set_moon_rocks` (default on;
  call from one thread) switch the game-side queries; `FlightSim.set_moon_surface_detail` / `set_moon_rocks` post the
  session flags as commands.
- `surface_samples_detail(body, lats_deg, lons_deg, min_wavelength_m, min_rock_diameter_m)`: same keys as
  `surface_samples`; `LunarSurfaceDetail` band limit + rocks >= the diameter (INF = none). Detail off or rocks off: LOLA
  / no rock tops (detail off is identical to `surface_samples`). Batches run in parallel in the core
  (`DetailedLunarTerrain::heights_limited_m`, a thread budget shared by all running batches, bit for bit the serial
  loop; [moon_surface] test with four concurrent callers): a 67 x 67 node lattice at IM-2 takes 5.6-6.8 ms alone
  (1.3-1.5 us per point; levels 6-18) against 46-58 ms serial (10-13 us per point; 0.6 / 6.3 us at level 22).
- `moon_rocks(face, level, i, j, min_d)`: rocks of one cube-face cell (empty at a level no octave uses); per rock
  `ids`, `base_fixed_m` (3, radius R + base height), `basis` (9 = local_to_fixed as three column vectors a, b, up),
  `axes_m` (3), `diameter_m`, `burial_m`, `base_height_m`, `albedo`, `template`.
- `rock_template_mesh(id, lod)` (lod 0..3 = 2562 / 642 / 162 / 42 vertices); `moon_surface_statistics()` (Hapke, crater
  / rock SFD, roughness, rock cover samples `cover_diameters_m` / `cover_above_m`: 33 log-spaced diameters).
- `surface_finest_resolution_m("moon")` returns 0.01 m with detail on (the quadtree's data limit, Task 7);
  `surface_data_resolution_m(body, lats, lons)` (Task 7) the same over the measured data alone (LOLA), whatever the
  flag.
- Thread safety: queries are const and use per-thread memos; the flags are atomics.

### Terrain mesh (Task 7)
- The Moon's quadtree draws the band-limited surface: every node samples `surface_samples_detail` at its level's
  vertex spacing (rocks off) and morphs all vertices to its parent's band; it refines by the existing screen-size rule
  down to level 22 (1.02 cm) near the camera; horizon maps are baked only down to the LOLA level, deeper nodes use their
  owner's. Details, numbers and costs: `terrain-rendering.md`. `World.moon_surface_detail` (static, read at the Moon
  terrain's setup; render_check `--moon_detail=off`) sets the core's and the flight's flag; switching it for a running
  world needs a rebuilt Moon tree (not built). `SurfaceClearance` keeps cameras above the full-detail surface.
  Children read their parent band from the parent's kept heights, builds go camera-first with 16 at once, and pole
  lattice points are sampled at longitude 0 (fix round 1; `terrain-rendering.md` "Morph", "Poles", "Loading").

### Baked textures (Task 8)
- `python tools/data_pipeline/fetch.py moon_detail` (also part of a bare fetch; `tools/moon_textures/bake_moon_detail.py`)
  writes `data/textures/moon_detail/` (14.2 MB, shipped with the base data): per scale s0..s3
  `regolith_height_<s>.png` and `crater_height_<s>.png` (16-bit; the tests integrate them), `regolith_slope_<s>.rgbah`
  (the shader's map: raw little-endian half floats, regolith dh/du, dh/dv and craters dh/du, dh/dv, periodic central
  differences of the float heights - no 8-bit quantisation, loaded without a per-pixel decode),
  `regolith_normal_<s>.png` (8-bit preview of the sum); `rock_{albedo,normal}_{0..3}.png`; `moon_detail.toml` (tile
  sizes, band edges, height scales, calibrated amplitude, crater counts / ranges / rms).
  Procedural only, fixed numpy seeds, byte-identical rebakes.
- Band split (no double counting): the textures carry ONLY wavelengths below the mesh's vertex spacing, from the same
  law as the core roughness (octave variance a^2 (lambda / 0.125 m)^(2 H), H from `roughness.hurst_exponent`). Four
  disjoint, contiguous bands of 3 octaves, 512 px each (tile = 8 x the longest wavelength): s0 5-40 mm (tile 0.32 m),
  s1 40-320 mm (2.56 m), s2 0.32-2.56 m (20.48 m), s3 2.56-20.48 m (163.84 m; Task 10 fix round, for the craters coarse
  leaves miss). The outer edges (5 mm, 20.48 m) carry a quarter-octave raised-cosine taper inside the band; the shared
  edges (40 mm, 320 mm, 2.56 m) are complementary (cos / sin amplitude ramps over a quarter
  octave centred on the edge: the two neighbours' variance sums to the law at every wavelength - the earlier inside
  tapers on both sides lost ~0.9 mm^2 of Guo's statistic, Task 10 review I1). The mesh
  resolves wavelengths >= 2 x spacing (finest level 22: 1.02 cm -> 2 cm), so the T10 shader fades each scale in only where
  the mesh no longer resolves it (s0 below ~2 cm near the camera; s1 / s2 farther out where the node spacing grows).
  Edges (`wavelength_min_m` / `wavelength_max_m` in `moon_detail.toml`; flat part; content): s0 5-40 mm (flat 5.95-36.7
  mm, content to 43.6 mm); s1 40-320 mm (flat 43.6-293 mm, content 36.7-349 mm); s2 0.32-2.56 m (flat 0.349-2.35 m,
  content 0.293-2.79 m); s3 2.56-20.48 m (flat 2.79-17.2 m, content from 2.35 m). The shader weights each scale by the
  share of its band the mesh does not draw (Regolith shading). The s1 / s2 bands overlap what the mesh carries at level
  22 by design (ruling T8 I-1).
- Craters in the textures (Task 10 fix round): each scale also carries the small craters of the core's law (config
  [craters]: Minton et al. 2019 equilibrium N(>D) = 0.0336 D^-2, fresh / degraded d/D 0.13 / 0.07, rim, 70 % degraded)
  whose diameter lies in its band (from 1 cm; each crater in exactly one scale): the law's expected count on the tile
  (32 / 135 / 135 / 135), uniform positions, D at stratified quantiles of the law's inverse cumulative, exactly 30 %
  fresh, the core's `crater_profile_m`, zero mean, periodic. Stratified because a tile holds only ~6 craters in its top
  octave, which carry most of the height variance: independent draws left the first bake at 0.59-1.24 x the law's
  crater variance; stratified, the tiles hold 0.92 / 0.74 / 1.11 / 1.38 x (the positions' overlaps of the few large
  craters remain random). The crater relief is a separate component (its own height PNG, the slope map's blue / alpha),
  so the shader weights it with the core's crater drawn fraction, not the regolith's transfer.
- Amplitude: the octave constant a = 2.29 mm is calibrated at bake time exactly as the core does (Guo's statistic:
  median plane-detrended std of 25 x 25 windows at 5 mm spacing, 1536 windows, of a full-spectrum 10.24 m field = 3.7 mm
  `rms_height_at_baseline_m`). Heights 16-bit (`32768 + h / height_scale_m * 32767`), normals 8-bit tangent space OpenGL
  (green = up = image top) from the metric pixel size, periodic differences. Band RMS heights 1.04 / 5.4 / 26.5 mm,
  RMS slopes 0.39 / 0.25 / 0.16.
- Rock set (triplanar, tile 1 m, 512 px): relative albedo (8-bit, 128 = 1.0, decoded mean 1.0000, sd 0.10-0.12; the
  shader multiplies by `rocks.albedo_to_regolith`, so the WAC mean never shifts) and normals; variants fine grain,
  vesicular, breccia, fractured. All rock-texture amplitudes are [E] (spectrum H 0.8, 0.5 x the regolith amplitude a
  = `ROCK_AMPLITUDE_FACTOR`: bare rock has no loose grains, rms slope 0.21 instead of 0.43 with the full regolith a, which
  looked over-rough in the normal maps; albedo sd 0.10): the config holds no rock-surface data.

### Hapke reflectance (Task 9)
- Core reference `environment::hapke_reflectance(HapkeParameters, mu0, mu, g, theta_bar)`: the radiance factor I/F of
  Hapke 2012 (IMSA with porosity K, SHOE, CBOE, double Henyey-Greenstein, Hapke 1984 roughness; H function Hapke 2002),
  `hapke_disc_reflectance` (a uniform sphere's disc mean I/F = ROLO's disc-equivalent reflectance A; Gauss-Legendre over
  the lit, visible lune in photometric coordinates). Parameters from `config/moon_surface.toml [hapke]` (Model above).
- Game: `moon_regolith.gdshaderinc` chooses the law by the global `moon_brdf` (game setting `World.moon_brdf`,
  "hapke" default | "lommel_seeliger"; render_check `--moon_brdf=lommel_seeliger`); `moon_hapke.gdshaderinc` mirrors the
  core function (float32). Globe (`moon.gdshader`), terrain (`terrain_moon.gdshader`), the radiance map's lunar ground
  (`sky.gdshader`) and the eye adaptation (`Atmosphere.regolith_reflectance`, through the core binding) use the same
  function, for the Sun and the earthshine. `Atmosphere.configure_moon_brdf` loads the parameters
  (`StarsimCore.load_moon_hapke`, config only, no terrain data) and sets the globals `moon_hapke_albedo_phase` (w, b, c, K),
  `moon_hapke_opposition` (B_S0, h_S, B_C0, h_C), `moon_hapke_theta_bar_rad` (the globe's 23.657 deg) and `moon_hapke_norm`.
- Brightness: scene value = albedo map x `moon_regolith_scale` x E x `moon_hapke_norm` x I/F, with `moon_hapke_norm` =
  1.35 / A_Hapke(0) = 5.420 [D] (`Atmosphere.hapke_normalisation`): the full-phase disc equals the Lommel-Seeliger law's
  (both at K&S's full-Moon illuminance, the regolith scale's calibration point), so the albedo map's mean brightness is
  unchanged at full Moon; away from it the disc follows Hapke's own phase curve (Validation).
- Mean slope: with the detail shading a terrain fragment takes the residual of its node (Regolith shading below);
  without it (detail off) the globe's 23.657 deg. `terrain_moon.gdshader`'s `hapke_theta_bar_rad` >= 0 overrides both
  (tests, diagnostics).

### Regolith shading (Task 10)
- Core reference `environment/unresolved_shading.hpp` (`UnresolvedShadingModel`, `make_unresolved_shading_model`),
  mirrored by `game/shaders/moon_detail.gdshaderinc`; wired by `MoonDetailShading` (`game/world/terrain/
  moon_detail_shading.gd`: template uniforms from `StarsimCore.moon_unresolved_shading`, the textures, the abundance
  map; per-node uniforms through `TerrainQuadtree.node_parameters`). Detail off (or `render_check
  --moon_detail_shading=off`): none of it runs (`detail_shading` false), the shader is Task 9's.
- Slope budget (residual theta_bar): Hapke's theta_bar is the globe fit's sub-pixel roughness; for Hapke's Gaussian
  slopes the mean square slope is S^2 = pi tan^2 theta_bar = 0.603. The globe's pixels contain the rocks, which the
  renderer models explicitly (below; geometry from Task 11), so their share comes out first, in Hapke's mean-tan form
  (tan theta_bar = (2/pi) <|grad h|>): <tan>_regolith = (<tan>_globe - A pi eta) / (1 - A), A = 1 - exp(-C_all) =
  0.0306 the rocks' area share at abundance 1 (the globe's mean), pi eta = 1.20 a half-spheroid's mean |grad h| over its
  plan area (closed form, checked by a midpoint sum): <tan> 0.688 -> 0.674, theta_bar 23.657 -> 23.158 deg, S^2 0.603 ->
  0.575 (`[unresolved_shading]` test: regolith and rocks add up to the globe's mean tan to 1e-12; without rocks nothing
  comes out). Before the Task 10 fix round the rocks were shadowed twice (in theta_bar and by their explicit terms,
  review I2). A node of vertex spacing L draws drawn(L) of it:
  its vertices are the relief's L x L box mean, its normals central differences over 2 L; drawn(L) applies exactly
  that filter to the statistical surface - per roughness octave the spectral transfer of the generator's value noise
  (quintic fade kernel; `regolith_drawn_fraction`: 1 at L << s, 0.39 at L = s/2, 0.083 at L = s), per crater size the
  same filter in real space on the crater profile (`crater_drawn_fraction`, 3 x 3 lattice offsets), summed over the
  generator's octaves (5 mm .. 3.8 km) and the crater law (1 cm .. 2 km); tabulated at 4 per octave (1 mm .. 17 km).
  The fragment's residual is tan theta_res = sqrt((S^2 - drawn(L_eff)) / pi), L_eff = node spacing x 2^morph (the
  morph slides a node onto its parent's surface, so a split changes nothing). Values: L22 (1 cm) 20.2 deg, 10 cm 21.9,
  1 m 22.5, 10 m 22.8, 100 m 23.1, 1 km 23.15, beyond the bands the regolith's 23.158. The whole generated surface holds
  S^2 = 0.296 (roughness octaves 0.252, craters 0.045); the rest, 0.278, is relief below the generator (< 5 mm: the
  law continued to ~0.6 mm would hold it, Helfenstein & Shepard 1999). The texture relief (regolith and craters) is NOT
  subtracted (below).
- Statistical shadows (Sun term only; the earthshine keeps the horizon maps alone, spec section 4): the drawn mesh relief
  finer than the horizon map's grid (drawn(L_eff) - drawn(H), H = the horizon owner's nominal spacing) casts no shadow
  in the renderer, so the Sun term is multiplied by Smith's G1 = 1 / (1 + Lambda) for a Gaussian surface of that slope
  variance (Lambda(a) = (erf a - 1) / 2 + exp(-a^2) / (2 a sqrt pi), a = tan e / S). The Sun's elevation e is its
  margin over the horizon map in its azimuth (`terrain_visible_fraction`'s margin: a slope facing the Sun raises it),
  over the local horizontal where no map is read. The near-field shadow map (Task 12) takes over inside its range.
- Unresolved rocks (D below the geometry threshold; until Task 11 every rock): Boolean model of half-spheroids of plan
  radius D/2 and height eta D (eta 0.383, the cover-weighted mean of generated rocks' exposed height / D, measured with
  `rock_top` at the base): cover C = abundance x cover(D_min, d_geo) (the law's closed form), visible rock share 1 -
  exp(-C) with reflectance ratio rho = I/F(1.35 w) / I/F(w) = 1.494 (standard geometry i 30, e 0, g 30; 1.43-1.54
  over (87, 75, 12) .. (85, 20, 80) deg), lit-face gain G(e) (Lommel-Seeliger radiance of the half-spheroid seen from
  the zenith over the flat ground's, tabulated every 2 deg, floor 1 deg: 4.9 at 2 deg, 2.2 at 5, 1.4 at 10, 1.0 at
  20, 0.92 at 45 deg), cast shadows sigma = (sqrt(1 + 4 eta^2 cot^2 e) - 1) / 2 per rock area (exact projection).
  Sun factor = G1 x [exp(-C (1 + sigma)) + (1 - exp(-C)) rho G] / N, earthshine factor = [exp(-C) + (1 - exp(-C)) rho]
  / N, N = exp(-C_all) + (1 - exp(-C_all)) rho: the albedo mosaic already holds every rock, so with every rock
  statistical the earthshine factor is exactly 1 (the WAC mean never shifts). d_geo = `rock_geometry_pixels` x distance
  x pixel angle (`ROCK_PIXEL_THRESHOLD` 1.5 px when Task 11 draws rocks; 0 now = no rock geometry). The abundance
  factor comes from a 2880 x 1440 (1/8 deg) equirectangular map of `LunarRocks::abundance_factor`
  (`StarsimCore.moon_rock_abundance_grid`), sampled like the colour map; the site (-84.8 deg) is outside Diviner: 1.
  Craters have no albedo share (no data); unresolved craters enter through theta_res.
- Regolith textures (maps s0..s3, regolith + crater components): face-lattice coordinates (u, v) = the node grid position x the level's
  nominal spacing R pi / (2 x 64 x 2^level) from the node corner, plus a per-node origin computed in float64 (modulo the
  tile, and the skewed hex coordinate modulo 256 cells), so the texture is continuous across nodes of a face.
  Stochastic hex tiling: triangle grid of cell = tile / 2, a hashed offset per grid vertex (integer hash, period 256
  cells), the three samples blended variance-preserving (sum w s / sqrt(sum w^2)), `textureGrad` with the continuous
  coordinate's derivatives (anisotropic mips). The maps are loaded as zero-mean four-channel half-float slope maps
  (`MoonDetailShading.slope_image` loads the baked `.rgbah` maps and builds the mips; 8-bit normals put a zero slope on
  code 128 = +0.0039 and Godot's rounded 8-bit mips drift by ~1 code: a 4-7 % mean shift of the modulation under a low
  Sun with the first version). The regolith component's amplitude weight
  at L_eff is the share of its band's height variance the mesh does not draw (`MoonDetailShading.texture_weight`: the
  mesh keeps sinc^6(pi L / lambda) of a wavelength - L x L box mean, linear interpolation between vertices - so
  w^2 = sum V (1 - sinc^6) / sum V over the band's octave law V ~ lambda^(2H); tabulated 4 per octave from 1 mm). The
  crater component's weight is the share of its craters' slope variance the mesh does not draw: w^2 = 1 - the mean of
  the core's `crater_drawn_fraction(D / L)` (the table drawn(L) uses, `moon_unresolved_shading` "crater_fraction") over
  the band's diameters, weighted by the law's slope variance per ln D (~ D^(2 - b); `MoonDetailShading.crater_weight`).
  The mesh's craters (an independent realisation of the same law) and the texture's add up to the law's slope variance
  per band: nothing counted twice. Weights regolith / craters per scale s0 / s1 / s2 / s3: L22 0.88/0.95, 0.22/0.59,
  0.03/0.23, 0.00/0.18; L19 1.00/1.00, 0.88/0.97, 0.22/0.59, 0.03/0.23; L17 1.00/1.00, 1.00/1.00, 0.65/0.89, 0.12/0.44.
  The Task 8 handoff's log-linear rule (0 at lambda_min / 2, 1 at lambda_max / 2) left the summed roughness 25-28 %
  short at L20-L19: replaced. A scale is skipped when both weights are below 0.1 ([E], < 0.016 rms slope) and once the
  footprint's short axis exceeds its lambda_max (the mip has averaged its slopes away; s3 therefore costs nothing near
  the ground and carries the 2.5-20 m craters a level-13 leaf (5.2 m spacing, 1.2 km up) does not draw). The height gradient is mapped onto the surface through the screen-space cotangent
  frame.
- Why baked craters, not shader craters (the review's preferred route, built and measured in the fix round): a
  deterministic field evaluated per fragment (half-octave bins of the core's law, 2 x 2 hashed cells per bin, 32-bit
  integer hash, exact shader-vs-core parity) closed the roughness gate as well but cost +5.4 ms of GPU time in the stereo
  mock (13.0 against 7.6 ms; +11 ms before optimising; ~0.35 ms per bin, since divergent lanes run every candidate's
  full path) - far over the ~1.8 ms left for Tasks 11-12. The texture craters ride in the regolith maps' fetches. Costs
  of this route: the craters repeat with the tiles (hidden by the hex tiling), the texture's craters cross-fade into
  the mesh's (different) craters over ~2 octaves during a descent, and craters above 20 m are left to the mesh.
- The texture relief casts no shadows, so it does not tilt the Hapke normal (a normal-mapped field under a 3-7 deg
  Sun averages 1.2-3 x brighter than Hapke's model of the same roughness: numpy experiment in the Task 10 report). It
  modulates each light by 1 + tanh(k . grad h), k = (l + v) / (mu0 + mu) - l / mu0 (d ln R / d grad h of the
  single-scattering law at the mesh normal): odd in grad h, so the mean over the symmetric slope field is unchanged and
  the texture slopes stay in theta_res's budget; first-order exact for gentle slopes, saturating to lit / dark facets
  under a low Sun (0 .. 2 x the mean).
- Crest lines (Task 7 M1): with Hapke, a terrain fragment whose interpolated normal faces away from the eye is shaded
  with its triangle's facet normal (`dFdx`/`dFdy` of `camera_offset`); Hapke's grazing-view rise is also smaller (I/F at
  mu 1e-3 / at mu = mu0 = 0.5: 1.41 against Lommel-Seeliger's 2.0). The Lommel-Seeliger law is unchanged bit for bit
  (the facet fix and the Hapke branch only run with `moon_brdf` 1).
- Not switched: the umbra glow of a lunar eclipse stays Lommel-Seeliger-weighted ([E] approximation).

## Code
- `core/include/starsim/environment/hapke.hpp`, `core/src/environment/hapke.cpp`: `hapke_reflectance`, `hapke_h_function`
  (`HapkeHFunction` 2002 | 1981), `hapke_phase_function`, `hapke_roughness`, `hapke_disc_reflectance`,
  `disc_integrated_reflectance` (Task 9). Bindings `load_moon_hapke`, `moon_hapke_reflectance`,
  `moon_hapke_disc_reflectance`; `game/shaders/moon_hapke.gdshaderinc`, `game/tests/hapke_patch.gdshader`, `game/tests/hapke_patch_production.gdshader`.
- `tools/moon_textures/bake_moon_detail.py`, `test_bake_moon_detail.py`, `contact_sheet.py` (Task 8).
- `core/include/starsim/environment/unresolved_shading.hpp`, `core/src/environment/unresolved_shading.cpp`,
  `core/tests/test_unresolved_shading.cpp` (Task 10): `UnresolvedShadingModel`, `make_unresolved_shading_model`,
  `slope_variance_from_theta_bar` / `theta_bar_from_slope_variance`, `regolith_drawn_fraction`, `crater_drawn_fraction`,
  `drawn_slope_variance_direct` / `drawn_slope_variance`, `residual_theta_bar`, `unshadowed_slope_variance`,
  `smith_lit_fraction`, `rock_cover`, `rock_shadow_per_cover`, `rock_lit_gain` / `rock_lit_gain_table`,
  `unresolved_light_factors`, `crater_drawn_fraction_table`; `LunarRocks::abundance_grid` (the renderer's abundance
  map, sampled in the core). Bindings `moon_unresolved_shading`, `moon_unresolved_light_factors`,
  `moon_residual_theta_bar`, `moon_rock_abundance_grid`. `game/shaders/moon_detail.gdshaderinc` (the shading is split
  into `detail_light_factors`, `detail_texture_gradient`, `detail_modulation` and `detail_theta_bar` instead of the
  plan's single `moon_detail_shade(...)`: the screen derivatives must be taken in uniform control flow before the
  horizon branches, and the light factors need the horizon margin computed between them),
  `game/world/terrain/moon_detail_shading.gd` (`MoonDetailShading`: `texture_weight`, `crater_weight`, `slope_image`),
  `TerrainQuadtree.node_parameters`; tests `game/tests/moon_detail_checks.gd`, `game/tests/moon_detail_patch.gdshader`.
- `core/include/starsim/environment/surface_hash.hpp`: `surface_hash(a, b, c, d)`, `hash_unit`, `surface_stream`.
- `core/include/starsim/environment/cube_face_cell.hpp`, `core/src/environment/cube_face_cell.cpp`: `FaceCell`,
  `face_cell_of`, `face_cell_point`, `face_cell_neighbourhood` (3 x 3, duplicates at cube corners),
  `face_cells_within` (all faces), `face_cell_level_for_size` (finest level with cells >= the size),
  `face_cell_area_m2`, `face_coord_of` / `face_coord_on` / `face_direction` / `face_frame` (metric, local scale),
  `face_blend` / `face_blend_band` (partition of unity over the faces near an edge).
- `core/include/starsim/environment/lunar_surface_detail.hpp`, `core/src/environment/lunar_surface_detail.cpp`:
  `LunarSurfaceDetail(config, lola)` with `relief_m` / `crater_relief_m` / `roughness_relief_m(unit, min_wavelength_m)`,
  `band_start_m`, `finest_wavelength_m` (0.01 m), `craters_in_cell`, `craters_within`, `crater_profile_m`,
  `crater_octaves`, `roughness_octaves`, `roughness_scale_m`. It knows nothing about rocks.
- `core/include/starsim/environment/rock_templates.hpp`, `core/src/environment/rock_templates.cpp`: `RockTemplates`
  (`radius`, `normal`, `mesh(id, lod)`, `column_top`, `bottom_z`, `top_z`, `exposed_area`, 16 templates x 4 LODs),
  `TemplateMesh`.
- `core/include/starsim/environment/lunar_rocks.hpp`, `core/src/environment/lunar_rocks.cpp`: `LunarRocks(detail, map)`:
  `rocks_in_cell`, `rocks_near`, `rocks_touching`, `rock_top`, `cover_fraction`, `abundance_factor`, `octaves`,
  `max_semi_axis_per_diameter`; `Rock`. `core/src/environment/surface_sampling.hpp`: Poisson / inverse SFD shared with
  the craters.
- `core/include/starsim/environment/rock_abundance_map.hpp`, `core/src/environment/rock_abundance_map.cpp`:
  `RockAbundanceMap` (`at`: bilinear over valid neighbours), `make_rock_abundance_map(file)` (checks the grid layout),
  `load_rock_abundance_map(rocks config, terrain_dir)`. `core/include/starsim/config/rock_abundance_file.hpp`,
  `core/src/config/rock_abundance_file.cpp`: `load_rock_abundance_file` (TOML header + int16 DN; German ConfigError
  naming fetch.py when missing).
- `tools/data_pipeline/rock_abundance.py` (fetch target `diviner_rock_abundance`, kind `rock_abundance`): downloads
  DGDR_RA_AVG_CYL_032 (Diviner GDR L3 rock abundance average, 32 px/deg, 3840 x 11520 int16 LSB, 88 MB; layout from
  its PDS3 label: line 0 = 60 N, zero_line 1919.5, zero_sample -0.5, RA = DN x 0.001, missing -32768) into
  `data/terrain/diviner_rock_abundance.{i16,toml}`. The 128 px/deg version (1.9 GB) is not used.
- `LunarTerrain::tile_blend` / `tile_weight` / `global_resolution_m` / `finest_resolution_m` (`lunar.md`).
- `core/include/starsim/config/moon_surface_config.hpp`, `core/src/config/moon_surface_config.cpp`:
  `config::load_moon_surface_config(file)` -> `MoonSurfaceConfig` (craters, roughness, rocks, hapke);
  `SizeFrequency::cumulative_density_per_m2(D)` (forms `power` | `exponential`, not clipped to [min, max]).
  Missing / bad keys throw `ConfigError` with a German message naming the full key (`roughness.hurst_exponent`).
- `core/include/starsim/environment/detailed_lunar_terrain.hpp`, `core/src/environment/detailed_lunar_terrain.cpp`:
  `DetailedLunarTerrain`, `SurfaceDetailOptions`, `support_under_disc` / `DiscSupport`, `contact_height_m`,
  `support_upper_bound_m`, `contact_ground_m`, `contact_rock_top`, `height_limited_m`, `normal_step_m`, the contact
  cache; `detail()` / `rocks()` throw `std::logic_error` where the part is absent.
- `RockTemplates::column_top_interpolated` / `column_grid_node` / `column_grid_max` (the footprint grid's column tops,
  kept from construction).
- `contact::SurfaceField::below_disc` / `height_below` / `below_disc_upper_bound` / `height_below_upper_bound`, the same
  on `LandingSurface`, `TerrainField` (`core/include/starsim/contact/landing_surface.hpp`); `contact::leg_contact`
  feet on `below_disc`, the base ring on `height_below`, both skipped above the bounds.
- `sim::shared_moon_surface` (`core/include/starsim/sim/terrain_cache.hpp`); `FlightSession::set_moon_terrain`
  overloads, `set_moon_surface_detail`, `set_moon_rocks`, `moon_surface()`; `lunar_landing_mission.cpp` load_site.
- `core/include/starsim/mission/lunar_descent.hpp`, `core/src/mission/lunar_touchdown.cpp`: `TouchdownHazardLimits`,
  `touchdown_hazard_limits`, `touchdown_rest_planes`, the hazard memo, the hazard-checking `select_touchdown_point` overload, `TouchdownPoint::max_foot_step_m` /
  `min_bell_clearance_m`; `LunarDescentPlanner::plan` overload on the composed surface.

## Standing decisions
- Equilibrium, not the Neukum production function, for the small craters: every surface older than ~1 Gy is at
  equilibrium below the crossover diameter (hundreds of m on maria, km on highlands); the production function lies above.
- One power law per population (the interface holds one `SizeFrequency`): the rock law is the Surveyor cm-dm
  fit extrapolated to 10 m (see Sources for the join).
- Hapke parameters stay the self-consistent Sato et al. (2014) set (K = 1, B_C0 = 0, theta_bar fixed): mixing in a
  porosity K ~ 1.3 would need a refit of w.
- The Moon's brightness anchor stays K&S's full-Moon illuminance (the regolith scale): Hapke is normalised to the
  former globe at phase 0, not to its absolute I/F (the albedo map is a relative colour mosaic, not calibrated
  reflectance).

## Validation
- `[moon_surface][config]`: the shipped file loads, every field finite, 0.5 < H < 1, 0 < w < 1, |c| <= 1, rocks down
  to <= 0.01 m, both cumulative laws strictly decreasing over their ranges, roughness residual <= measured total, the
  abundance reference fraction equals the integral of the rock law within 2 %; a missing key is named in German. The
  loader rejects |c| > 1 and a residual above the total.
- `[moon_surface][environment]` (`core/tests/test_lunar_surface_detail.cpp`, synthetic flat LOLA with one 8 km tile at
  -84.79 deg; all fast, ~10 s together):
  - face cells: cube_sphere.gd mapping, cell centres map back, edge directions bit-identical, corner neighbourhood
    reaches three faces, `face_cells_within` covers every point of the radius;
  - determinism: 10^4 points, full and 1 m queries, 4 threads and a second instance: bitwise equal;
  - crater counts on the global grid, bins 1-2 ... 32-50 m with 1500 expected each: 1511, 1540, 1483, 1425, 1502,
    1498 (3 sigma 116); fresh share and d/D per crater; > 100 m within 20 km: 4375 vs 4212 expected;
  - roughness (Guo's statistic, ruling of 2026-11-04): roughness alone at the site 3.70 mm (configured 3.7, tolerance
    15 %); windows 0.1 / 1 / 10 m on the global grid 1.016 / 1.000 / 0.977 x the law 3.7 mm (W / 0.125)^0.78
    (tolerance 15 %); craters + roughness at the site 3.90 mm <= the measured total 4 mm (rocks: see below);
  - band limit: |relief(L) - mean of relief(0) over the footprint (24 x 24 samples per face parallelogram, blend
    weighted)| <= 0.1 x RMS of the removed band at every one of 60 points, L = 10 / 1 / 0.1 m: site max 0.028 / 0.032
    / 0.032 x (RMS removed 0.16 / 0.025 / 0.0033 m); face corner region outside the blend (s, t ~ +-0.9-0.98) 0.022 /
    0.029 / 0.022; inside the face blend band across edges and at corners 0.034 / 0.042 / 0.047;
  - seams, full detail: 1200 pairs 1 mm and 1 um apart across face edges and 480 pairs through the 8 cube corners
    within the 1 mm step bound of the field (1.8 mm per mm; 1 cm: 15.5 mm); the 1 um pairs are the continuity gate;
  - seams, band-limited (L = 0.1 / 1 / 10 m): 600 pairs across edges and 480 through corners (1 mm and 1 um) within
    the 1 mm step bound of relief(L) off the edges (0.93 / 0.76 / 0.57 mm); worst 1 um pair 0.67-0.78 of its bound,
    i.e. the field's slope, no step (before fix round 1 the step was up to 1.07 mm, ~1160 x the bound);
  - zero mean: mean of relief(L) over a grid of L x L squares tiling a patch, tolerance 4 standard errors: tile core
    5 km, 400 x 250 m: -0.0007 m (s.e. 0.0027; the population mean it removed is -0.044 m); global grid 200 km,
    1600 x 5 km: +0.019 m (s.e. 0.045; removed -1.08 m); relief(L) = 0 exactly for L = 2 km in the tile core and
    100 km on the global grid;
  - tile blend: band_start 20 m in the core, 1895 m outside, steps within the smoothstep slope; relief steps over 1 cm
    within the 1 cm bound across the blend;
  - poles (-90, -89.9999, +90 deg): finite for L = 0, 0.1, 1, 10 m, 1 mm neighbours within the bound;
  - nothing counted twice: no crater above 5 x 20 m in the tile core (craters 50-100 m exist), every crater
    D <= 5 x band_start at its centre, a 640 m square mean of the roughness is 0 in the tile core.
- Baked textures (Task 8, `python -m pytest tools/moon_textures`, 8 tests, ~6 s): bake twice -> every file byte-identical;
  every map seamless (wrap-around neighbour difference rms <= 1.1 x the interior one, seam maximum <= interior maximum + 1
  LSB: a map with slope cannot have a 1 LSB edge difference, so seamless is tested relative to the interior step);
  band variance in three log sub-bands of each scale's flat part vs the config law (measured / law amplitude 0.96-1.06,
  tolerance 15 %), Hurst exponent from the band ratio within 0.15, nothing above the band; an independent full-spectrum
  field (other seed) gives Guo's statistic 0.99 / 0.99 / 0.98 / 0.96 / 0.96 x the law at 125 mm / 0.25 / 0.5 / 1 / 2 m
  windows (tolerance 15 % at 125 mm, 0.5 m, 1 m); core cross-check without bake code: the field of
  `tools/validation/moon_roughness_residual.py` (its `fbm` + `window_rms`, 8 m patch) scaled to the config 3.7 mm Guo
  statistic has octave variances (25 mm-0.8 m) 1.01-1.02 x the law with the baked a (tolerance 10 %); rock albedo mean 1.000 within 1 %.
- Roughness stationarity (diagnostic, 3000 windows each, not a test): 3.72 mm at a face centre, 3.76 at an edge
  midpoint, 3.94 near a cube corner (the skewed lattice there, +6 %), 3.75 at the site; craters + roughness
  3.88-3.95 mm (4.09 near a corner).
- `[moon_surface][environment]` (`core/tests/test_lunar_rocks.cpp`, Task 3):
  - counts per bin on the global grid (1500 expected each, 3 sigma 116): 0.01-0.02 m 1515, 0.03-0.07 1511, 0.1-0.2 1432,
    0.3-0.6 1517, 1-2 1515, 3-6 1455, 6-10 1554; with a synthetic map (3 x reference) 3 x the uniform counts on the map,
    1 x outside; the factor is monotone and steps < 0.4 % of the jump per 0.01 deg across the blend band;
  - ownership: 82 3 x 3 blocks (2 cube corners, 80 cells along a face edge): no duplicate id, every rock's cell is the
    cell that lists it, and the rocks found by position equal the union of the block's cells;
  - tops: every top of the > 100 rocks of a 200 m disc (40 sample points each; 7307 hits) lies above base height and
    below base - burial + c top_z + d^2/2R, and on the rock's surface (|u| - r(u) worst 4.8e-14), unit normals facing up;
  - burial: buried share of the height measured on the LOD 0 mesh extents, 3350 rocks >= 0.1 m within 60 m of the
    site: mean 0.3001 (configured 0.30, 3 sigma 0.0078), range -0.0009 - 0.6005;
  - templates: r > 0.2 and <= 1 (max > 0.97) on 10^4 directions; exposed area full below the rock (0.2 pi - pi), 0 above,
    monotone; every mesh at every LOD closed
    and oriented (each directed edge once with its reverse, V - E + F = 2, every triangle faces outward), LOD 0 vertices
    within 1 % of r, unit outward normals; determinism on 4 threads (own and shared instance), also at -90 deg;
  - cover fraction: formula 0.0311 / 0.00678; real plan-view area of the 8083 rocks of 0.05-0.2 m in a 43 m disc at the
    site, measured with `rock_top` on 64 sunflower columns per rock: 0.00748 vs formula 0.00745 (+0.37 %, tolerance
    5 %); measured footprint / sum of pi D^2 / 4 = 0.9933 (tolerance 2 %); every rock's a <= 1.769 D;
  - abundance source: "uniform" ignores a map, "diviner" without a map throws (names the fetch target), an all-NaN map
    gives the uniform law; gap fill: filled values lie in the valid range and the factor swept across a 10 x 20 hole and
    a single missing pixel changes by no more than the map's own pixel step; a 12 x 36 file in the real format (TOML
    header + int16, one missing DN) loads: pixel centres, longitude wrap, the gap filled with its neighbours' mean; a missing file names fetch.py;
  - `[slow]` summed roughness (median plane-detrended 125 mm window std, 600 windows over 2 km, rock tops entered as
    max(ground, rock top)): site craters + roughness 3.845 mm, + rock tops 4.049 mm vs the measured total 4.000 mm
    (+1.2 %, tolerance 10 %); near the cube corner (1,1,1) 4.090 -> 4.232 mm (+5.8 %). The corner excess of the skewed
    lattice is unchanged after the face blend (4.090 vs 3.845 at the site = +6.4 %) and stays inside 10 % of the total, so
    the amplitude is not touched. The rock tops add 0.20 mm (site) to the median (rare rocks dominate the mean square,
    not the median).
- Physics surface (Task 4, `core/tests/test_detailed_lunar_terrain.cpp`, synthetic flat LOLA + the shipped config
  with the uniform law; `[moon_surface][terrain]` / `[contact]`):
  - detail off on the real LOLA (10^4 points, half within 15 km of the site): heights by latitude / longitude,
    direction and fixed point, normals (memcmp), resolutions, `normal_step_m`, `height_limited_m` and the disc
    support (= point query) all `==` LunarTerrain, with and without the detail parts present; `max_height_m` equal;
  - pivoting pad on a 9 deg plane (sloped synthetic site tile, craters and roughness off; true slope 9.018 deg):
    support = the plane's height under the centre within 0 m (tolerance 1 mm), normal = the plane's within 2.1e-6 rad
    (tolerance 1e-3); a level pad would stand 0.1587 m higher (checked > 0.1, so the case tells them apart);
  - a rock at the downhill rim of a pad on the 9 deg plane (3 peaks 1 mm inside the edge, 4 rocks cut by the rim, D
    0.41-0.44 m): support = plane height + the rock's protrusion above the pad's plane by brute force (polar grid
    100 x 240 + 8000 rim points of `rock_top` minus the plane) within 0.05 mm (tolerance 1 mm), normal within 0.012 rad
    (tolerance 0.05); a level pad on the same rock stands 0.158 m lower (checked > 0.1);
  - the same on the procedural ground (plane fitted to the rough ground; 5 peaks under the edge, 3 rim cuts, D
    0.41-0.64 m): within 0.10 mm, normals within 0.047 rad;
  - flat ground (no relief, no rocks): support = the ground at the centre (1e-9 m), normal radial; a 1 m pad centred
    in an isolated crater (craters >= 4 m only, D 4.6-5.8 m): the bowl's height at 1 m from the centre within
    0.041 mm (tolerance 1 mm), normal on the axis;
  - point contact (r = 0, the base ring) on 4 rock peaks: = max(ground, contact tops of every contact-size rock)
    exactly, its normal the rock's; the contact top within 1e-4 m / 0.02 rad of `rock_top`; the upper bound above it;
    a base-ring point between the ground and the rock top touches (`base_contact`), 1 mm above the top it does not
    (through `TerrainField` and `leg_contact`);
  - cached query == fresh instance bit for bit (40 discs and 40 point supports); the pad never stands below the ground
    at its centre;
  - pads centred in 8 craters of 6-16 m (rocks off): plane + pattern protrusion vs plane + a dense protrusion (21 x 40)
    of the same ground within 3 cm ([E]), normal = the fitted plane's;
  - mutation check (fix round 1): with the plane's tilt forced to 0 (a level pad) and the point contact without rocks,
    6 cases went red (9 deg plane, rim rock on the plane, procedural rock, procedural craters, point contact, base
    ring); restored, green;
  - `height_limited_m(p, 0, 0) == height_at_direction_m(p)` (200 points); without rocks the band-limited height at
    L = 4 / 1 m equals the footprint mean of the full ground to 0.0021 / 0.0008 m (removed band RMS 0.072 / 0.027 m;
    tolerance 0.1 x);
  - cost (`[contact_speed]`, CTest `contact_speed`, RUN_SERIAL): see Physics surface; resting, sliding and descending
    each < 50 us gated (medians of 7 x 200 ticks).
- `[terrain_cache][moon_surface][data][slow]`: the real Diviner map (3840 x 11520) loads with its gap fill in 0.44 s;
  the first `shared_moon_surface({true, true})` 1.65 s (detail construction ~1 s + map); one instance per options, one
  detail generator, LOLA shared with `shared_lunar_terrain`.
- `[hls][legs][lunar][terrain][moon_surface][data][slow]` (`test_hls_landing.cpp`): the HLS (1 m pads) dropped at
  1 m/s on the composed surface at the flattest LOLA spot near Mons Mouton (spot chosen with the flag off, then on)
  lands and rests, no crush.
- `[moon_surface][lunar][mission]` (`test_lunar_touchdown_hazards.cpp`): the limits from `starship_hls.toml` (pad
  1.0 m, step 0.6 m, bell clearance 0.9 m, base 4.5 m); the rest planes (six HLS feet on 7 deg, the uphill foot
  raised 0.5 m: two facets, steepest 8.20 deg against a least-squares 7.78 deg, hang 0.500 m; matched against an
  independent supporting-plane search; a plane gives one facet, feet on one side none); on flat LOLA with the
  procedural detail and rocks, a target 12 m east of the highest rock within 400 m (D 8.70 m, pad support 6.18 m above
  the ground) moves 60.0 m (slope 1.74 deg, hang 0.586 m, clearance 0.738 m, 337-420 ms cold), no nearer grid
  candidate passes (the search limited to just inside the offset finds nothing), independently re-checked at rolls
  between the samples; detail off gives the LOLA point bit for bit at IM-2 (both approach directions); IM-2 with full
  detail: the target moves 28.3 m (slope 3.19 deg, hang 0.461 m, clearance 0.572 m), 111-179 ms cold, 0.2 ms for the
  second approach direction (memo). Hidden `[.touchdown_worst]`: the refused-site times above.
- `[contact][legs]` (`test_landing_legs.cpp`): stick-slip friction - an upright HLS (translation only) on a 9 deg plane
  in lunar gravity does not creep (< 1 mm in 20 s; the regularised law crept 0.52 m at 2.6 cm/s); on 35 deg (above
  the 31 deg friction angle) it slides at g (sin - mu cos) within 10 %.
- Diviner product check (python, label + three rows by range request, not the 88 MB file): the label layout above;
  median RA 0.004 at the equator (17 % no data in that row), 0.003 at 57 N, 0.002 at 60 S; p95 0.006-0.008.
- `[moon_surface][hapke]` (`test_hapke.cpp`, Task 9):
  - published check values of the implementation: the six USGS ISIS3 photometric unit-test truths (HapkeHen defaults;
    HapkeHen w 0.52, B0 1, h 1, theta 30 deg, Hg1 0.213, Hg2 1; HapkeLeg w 0.52, B0 1, h 1, theta 30 deg; at (g, i, e) =
    (0, 0, 0) and (60, 45, 30) deg) reproduced to <= 0.0002 % with the 1981 H function ISIS uses (gate 1 %); the
    Hapke 2002 H at w -> 1 is 2.8854 against Chandrasekhar's 2.9078 (-0.77 %), and within 1 % of the exact H (integral
    equation solved in the test) at the shipped w. No publication tabulates I/F for the shipped median parameter set
    (Sato et al.'s tables are paywalled; the map itself carries no reflectance); the formula is therefore checked on
    ISIS's sets and the shipped set through its disc phase curve;
  - Helmholtz reciprocity (1e-12), the smooth limit (theta_bar 0 = closed form), 0 below the horizon;
  - the disc integrator against the analytic Lommel-Seeliger sphere (< 1e-5, 0-150 deg), order 48 vs 96 < 1e-5;
  - phase curve: the uniform Hapke sphere against ROLO 665.1 nm (Kieffer & Stone 2005, zero libration, waxing/waning
    mean) over 5-97 deg (ROLO's validity ends at 97 deg): level m_Hapke - m_ROLO = -0.377 mag (the highland-proxy w at
    643 nm is brighter than ROLO's whole near side incl. maria), shape residual about it at most 0.056 mag (12 deg; gate
    0.1 mag), Hapke slightly flatter (+0.055 at 5-10 deg, -0.054 at 97 deg). Against K&S 1991 Eq. 9 (5-120 deg) the
    shape residual reaches 0.130 mag (informative; K&S is the older V-band summary);
  - normalisation identity (Ruling T9 I1; not an independent physics check, the gain is defined at this point): Hapke x
    5.420 against the former globe at phase 0 within 3 % (1.0000, re-integrated independently: checks integrator and
    wiring); at the realistic full-Moon phases 1.5 / 2 / 3 / 5 deg within 5 % (0.96-0.995, accepted: it moves toward
    ROLO; the ROLO phase curve above carries the physics); printed: ratio 0.984 / 0.973 / 0.962 / 0.964 / 0.995 / 1.058 / 1.029 at 0.5 / 1 / 2 /
    3 / 5 / 7 / 10 deg (Hapke's surge falls off between K&S's linear surge and ROLO's steeper one: disc at 2 deg / 0 deg
    = 0.849 Hapke, 0.883 K&S, 0.834 ROLO, the last extrapolated below its 1.55 deg validity).
  - CBOE and porosity K (Fix round 1; not in the shipped set, B_C0 = 0 and K = 1, and not in the ISIS truths):
    r x (1 + B_C0 B_C) against an independently written B_C(g) at five phases (1e-12), B_C(0) = 1, the effect off at
    B_C0 = 0; K = 1.2 against the smooth closed form with H(mu/K) (1e-12), linear in K as w -> 0, reciprocity with K.
- Game (Task 9): `moon_surface_check.gd` - the eye adaptation's mirror (`Atmosphere.regolith_reflectance`, through the
  core) integrated over the disc: Hapke / Lommel-Seeliger 1.0000 at phase 0 (gate 3 %), 0.962 / 0.995 / 1.029 / 1.055 /
  1.134 / 1.188 at 2 / 5 / 10 / 30 / 60 / 90 deg (Hapke's crescent is brighter than K&S's; against ROLO it is within
  0.06 mag); with a GPU the shader parity: a flat patch through `moon_hapke.gdshaderinc` (HDR readback, calibrated to
  0.50000 for 0.5) against the core at (i, e, g) = (30, 0, 30), (60, 30, 50), (20, 75, 85), (10, 8, 3) deg: -0.03 /
  -0.05 / -0.03 / -0.08 % (gate 2 %); and the production wiring (`game/tests/hapke_patch_production.gdshader`:
  `regolith_reflectance_rough` from `moon_regolith.gdshaderinc` with the globals `configure_moon_brdf` sets, and
  terrain_moon.gdshader's theta_bar resolution) against moon_hapke_norm x core at the same four geometries: -0.02 /
  -0.04 / -0.02 / -0.09 %, and with a terrain node's override theta_bar 0.1 rad -0.03 % (gate 2 %; the terrain_moon /
  moon materials themselves are not rendered by the check, they call this function). Renders (render_check, IM-2, 2026-11-22T01:20 UTC): detail off with
  `--moon_brdf=lommel_seeliger` against the code before Task 9: globe, 2 m, 20 m 0 px different; 200 m 2440 px (max 25 /
  255) - a second run of the same new code differs from its first by 2304 px (streaming timing), from the old by 262 px
  (max 1). GPU (1600 x 900, 2 m, detail on, median of 120 frames, two rounds): Hapke 1.736 / 1.742 ms against
  Lommel-Seeliger 1.684 / 1.688 ms (+0.05 ms); stereo VR not measured (Task 13).

- Regolith shading (Task 10): `[moon_surface][unresolved_shading]` (10 cases, `core/tests/test_unresolved_shading.cpp`):
  the regolith slope variance of the model against the generator's field (central differences at 4000 points, full
  detail): 0.2456 vs 0.2464 (-0.3 %, gate 7 %); the mesh-drawn slope variance (relief_m at spacing L, central
  differences over 2 L, 20000 points over 4 km) against the model at L = 0.02 / 0.08 / 0.32 / 1.28 m: roughness 1.00 /
  1.00 / 1.00 / 1.00, craters 0.97 / 0.97 / 0.96 / 0.96 (gates 10 / 20 %; before the transfer functions, with "octaves
  >= L drawn", the model was 1.26-2.40 x the mesh); crater slope integral closed form against the profile's numeric
  integral 1e-4; Smith's G1 against its defining integral 1e-6; rock cast shadow against a ray-cast projection 1 %;
  lit-face gain limits; light factors (all rocks statistical: earthshine factor 1 exactly; Sun below the horizon: 0);
  residual theta_bar monotonic, largest change per 1 % spacing step 2.0e-4 rad; rocks out of the budget (regolith and
  rocks add up to the globe's Hapke mean tan to 1e-12, no rocks -> the globe's budget, the half-spheroid's pi eta
  against a midpoint sum 2e-3). Bake (`python -m pytest tools/moon_textures`, 12 cases): complementary shared band
  edges (squared weights sum to 1 to 1e-12), slope maps = the heights' central differences (to the 16-bit / half-float
  rounding, mean square slope within 0.5 %), crater count = the law's expectation per tile, the diameter quantiles,
  the profile, exact periodicity. Game (`moon_surface_check.gd`,
  `MoonDetailChecks`, `--detail_only` skips the quadtrees): fade continuity over 1 cm - 30 km in 1 % steps (theta
  1.9e-4 rad, Sun factor 0.10 %); with a GPU, flat patches through `moon_detail_patch.gdshader` against the core
  (moon_hapke_norm x I/F(theta_res) x factor), 512 x 512 readback: statistics only -0.05 / -0.04 / -0.03 % (Sun 7 / 15 /
  30 deg), with the texture modulation (regolith + craters) -0.55 / -0.35 % (50 m patches) and +0.06 / -0.08 % (2 m
  patches, L22), earthshine only (Sun 5 deg below) -0.04 % at abundance 1 and with no detail, -0.30 % at abundance 40
  with textures (gate 2 %). Summed roughness (Guo's statistic, 96 windows, mesh = core heights at the level's vertices
  bilinear, + each scale's regolith and crater components at their weights) at L22 .. L17: 4.05 / 3.85 / 3.65 / 3.68 /
  3.73 / 3.69 mm against the measured 4 mm (+1.3 / -3.8 / -8.6 / -8.1 / -6.8 / -7.7 %, gate 10 %, not loosened; before
  the fix round 4.02 / 3.78 / 3.50 / 3.49 / 3.49 / 3.42 mm with L20-L17 failing). No double roughness anywhere.
- Site brightness (render_check, IM-2, 2026-11-22T01:20 UTC, Sun 6.75 deg, elevation -20 deg, eye adapted; mean 8-bit of
  the image, `--moon_detail_shading=off` = Task 9's shading in the same view; re-measured in the fix round): Task 9 /
  Task 10 / fix round 2 m 35.3 / 30.6 / 30.8 (-13 %), 20 m 38.7 / 36.1 / 36.3 (-6 %), 200 m 37.7 / 36.3 / 36.4 (-3 %);
  grazing view (2 m, -6 deg) 25.1 / 22.1 / 22.3; nadir from 1.2 km 36.7 / 35.7 / 35.2 (-4 %). The
  near ground darkens most: Smith's statistical shadows of the drawn cm relief (G1 ~0.70 at 7 deg) outweigh the smaller
  residual theta_bar (x1.02-1.25); rocks x0.96. All of it is shadowing physics, none tuned. The fix round's rocks-out
  budget (theta_res -0.5 deg) and the crater textures change the means by < 1 % (the modulation is mean-preserving).
  Against the LROC NAC frame M1434867405 (90 cm pixels, a different Sun; `screenshots/moon_surface/task10/
  nadir_lroc_nac_left_task10_middle_fix1_right.png`): before the fix round the render lacked the 5-20 m craters - not a
  density error (config N(>D) = 0.0336 D^-2 is Minton et al. 2019 Fit 1, the median of the modern counts; it predicts
  336 craters > 10 m and 84 > 20 m per km^2) but a drawing gap: a level-13 leaf (5.2 m spacing) draws craters only from
  ~4 L = 20-40 m and nothing drew the smaller ones (Task 10 review C3). With the crater textures (s2 / s3) the render
  now shows the 2.5-20 m population; it stays fainter than the NAC frame's because the craters cast no shadows (open
  point). No crater count was made on the images.
- Detail off and Lommel-Seeliger with detail off pixel-identical to Task 9 (render_check `--deterministic
  --fixed_exposure`, IM-2, 2 m and 20 m, the old `terrain_moon.gdshader` swapped in: 0 px; a first version that
  multiplied the old expression by the factors 1.0 differed in 2-7 px by 1 LSB, so the detail-off branch keeps the old
  expression).

## Sources
Each value with tag, reference and the scale range it covers. Tags: [S] sourced, [E] estimate, [D] derived.

### Small craters
| Value | Tag | Source | Range |
|---|---|---|---|
| N(>D) = 0.0336 D^-2 / m^2 (2.2 % geom. saturation) | [S] | Minton et al. 2019 (Icarus; arXiv:1902.07746) sec. 1.2, Fit 1 to Fassett's Apollo 15 counts (LROC NAC 0.5 m/px): N(>r) = 0.0084 r^-2; geometric saturation 0.385 r^-2 (Gault 1970); in diameter x 4 | fitted 10-50 m |
| slope 2 | [S] | Xiao & Werner 2015 (JGR Planets 120): slope ~2 on all terrains, equilibrium 0.69-3.9 % of geometric saturation where it occurs below 500 m; Gault 1970: 1-10 %; Hartmann 1984: slope 1.83 | m - km |
| min 0.01 m | [E] | finest drawn scale; Shoemaker & Morris 1970 (Radio Science 5): steady state F = 10^10.9 c^-2 at all five Surveyor sites; Surveyor 7: 13 cm - 3 m follows it | extrapolation cm |
| max 2000 m | [E] | LOLA_16 (1.9 km) on the far side; the 20 m site tiles resolve ~100 m (generator fades by resolution) | - |
| d/D fresh 0.13 | [S] | Sun et al. 2018 median of ~850 fresh small craters (cited in arXiv:2008.10132); Stopar et al. 2017 (Icarus 298): 0.11-0.17 for fresh D < 400 m, ~0.12 for 30-100 m; Mahanti et al. 2018: freshest class 0.13-0.17 | 30-400 m measured |
| d/D degraded 0.07 | [E] | Mahanti et al. 2018 (Icarus 299): small craters (35-250 m) at Apollo 16/17 mostly degraded, mean d/D < 0.1, 99 % in 0.04-0.17; Minton et al. 2019: counting limit 0.05 | 35-250 m |
| rim height 0.024 D | [D] | Pike 1977: fresh simple craters d = 0.196 D, h_rim = 0.036 D -> rim/depth 0.184, applied to d/D 0.13 | km-scale ratio applied to small craters |
| degraded fraction 0.70 | [S] | Mahanti et al. 2018 via LROC news: ~70 % class C at Apollo 16 (highland), 88 % at Apollo 17 | 35-250 m |

Disagreement: equilibrium densities range 0.7 % (Xiao & Werner) to 5 % (Shoemaker & Morris' 10^10.9 if read as per
10^6 km^2 with D in m - units not confirmed from the original) of geometric saturation; 2.2 % (Minton, a modern
count close to the median of the Robbins et al. 2014 counter ensemble) lies in the middle.

### Regolith roughness
| Value | Tag | Source | Range |
|---|---|---|---|
| H = 0.78 | [S] | Guo et al. 2021 (GRL 48, e2021GL094931), Chang'e-4 5 mm DTM, sec. 3.2: H = 0.78, 0.85, 0.85, 0.71, 0.51 for the five natural subregions (0.61 rover tracks, excluded), median 0.78; breakover 0.5-2 m | 10 mm - 2.5 m |
| RMS 4 mm at 125 mm (measured total) | [S] | Guo et al. 2021 sec. 3.1: RMS height (std. dev. in a window) median 4 mm at 125 mm, 1-18 mm | 15 mm - 1.25 m |
| residual 3.7 mm at 125 mm | [D] | derived below (`tools/validation/moon_roughness_residual.py`) | 125 mm |

Residual (no double counting): Guo's 5 mm DTM is the whole natural surface, so its 4 mm already contains the cm-dm
craters and rocks that we also generate down to 0.01 m (336 craters and 57 rocks per m^2 above 1 cm). The roughness
generator therefore draws only the residual. Method: a Monte Carlo patch (8 m x 8 m, 5 mm pixels as in Guo, 4096
windows of 125 mm) of the configured populations: craters 0.01-4 m from the configured SFD, 70 % degraded (d/D 0.07,
rim scaled with depth to 0.013 D) / 30 % fresh (d/D 0.13, rim 0.024 D), profile assumed as a parabolic bowl
(rim-to-floor depth d) with the rim outside falling as h ((r/R)^-3 - 1/27) / (1 - 1/27) to 3R (the generator's
profile; continuous at 3R); rocks 0.01-4 m from the configured SFD as
ellipsoids a:b:c = 1:0.71:0.50 (D = long axis), 30 % of the height buried, added on top; plus a spectral fBm with
H = 0.78. Guo's statistic is the median over windows of the plane-detrended standard deviation, so the residual is
the fBm amplitude for which the summed surface has a median of 4 mm. Larger features add only curvature in a 125 mm
window (a fresh 5 m crater: about 0.1 mm) and are left out. Results, seeds 1-8 (with the continuous rim profile; the earlier (r/R)^-3 cut at 3R gave the
same residual): craters alone median 0.44-0.48 mm, rocks alone 0.24 mm, craters + rocks 0.73-0.79 mm (mean-square based 2.1-3.6 mm, dominated by the rare large rocks);
residual 3.72-3.76 mm -> 3.7 mm. Sensitivity: crater density x 0.3 / x 2.3 (0.7 % / 5 % of geometric saturation)
gives 3.85 / 3.51 mm, rock density x 2 gives 3.64 mm, crater x 2.3 with rock x 2 gives 3.41 mm. Subtracting mean
variances instead (sqrt(4^2 - c^2 - r^2)) gives 1.8-3.4 mm and is not stable, because the mean square is carried by
a few windows with large rocks while Guo reports the median. The residual is fixed only at the 125 mm baseline with
Guo's H; at larger windows the craters' share grows, which is why the summed-surface validation (Validation) checks
the total. Check at 1 m windows (seeds 1-2): craters + rocks median 4.65 mm (seed 1); the residual matching Guo extrapolated (4 mm x 8^0.78 = 20.2 mm) is 17.4-18.2 mm, the 3.7 mm law extrapolates to 18.6 mm (2-7 % apart), so one residual with Guo's H holds from 125 mm to 1 m.

Disagreement: at metre-hectometre scales H ~ 0.9 (Cai 2019 LPSC 2343 / Cai & Fa 2020, NAC DTMs: maria and highlands
median 0.9; RMS height in 100 m windows 1.28 m maria, 3.90 m highlands); LOLA 17 m - 2.7 km: highlands 0.95, maria
0.76 (Rosenburg et al. 2011). Those lie above the 1-2 m breakover, at scales where craters dominate; the cm-m
regolith term follows Guo (whose cm-m measurement also contains craters and rocks, hence the residual above).
Extrapolated to 100 m the residual term gives 0.68 m, below the NAC totals with craters (consistent).

### Rocks
| Value | Tag | Source | Range |
|---|---|---|---|
| N(>D) = 0.154 D^-2.286 per 100 m^2, D in m (= 1.54e-3 D^-2.286 per m^2) | [S] | Cintala & McBride 1995 (NASA TM-104804) Table 3: power-law fits to the Surveyor fragment counts of Shoemaker & Morris 1968; Surveyor VI (mare, mature regolith; exponent +-0.094) | mm - dm measured |
| other Surveyor sites | - | I: 0.797 D^-1.779; III: 0.049 D^-2.559; VII (Tycho ejecta, highland): 2.666 D^-1.802 | mm - dm |
| max 10 m | [E] | extrapolation | - |
| b/a 0.71, c/a 0.50 | [S] | Fujiwara et al. 1978 (Icarus 31) impact fragments a:b:c = 2:sqrt2:1; LRO NAC boulders b/a mostly ~0.7, 0.3-0.9 (Kumar et al., NGRI; Krishna & Kumar 2016) | NAC: m-dam |
| sd 0.15 | [E] | spread of the NAC range | - |
| burial 0.3 +- 0.15 | [E] | no lunar measurement found | - |
| albedo 1.35 x regolith | [E] | w_rock 0.65 / w_regolith 0.48 of the synthetic NAC boulder-field model of Marshal, Ruesch, Woehler et al. (EGU22-12217), taken there from earlier work; Ruesch et al. 2024 (JGR) found boulders near Reiner Gamma with w like the regolith | - |
| abundance reference 0.0068 for 0.5-10 m | [D] / [E] | disc area of the law, (pi/4) k b (D1^(2-b) - D0^(2-b)) / (2-b) = 0.00678 with D0 = 0.5 m [E: middle of Diviner's ~0.1-1 m sensitivity, Bandfield et al. 2011]; D0 = 1 m would give 0.0047 (local factor 1.45x higher), D0 = 0.1 m 0.0137 | - |
| abundance Diviner | [S] | Bandfield et al. 2011 (JGR 116, doi:10.1029/2011JE003866), GDR L3 rock abundance, 60 S - 60 N, sensitive to rocks > ~0.1-1 m; typical < 1 % (Hayne et al. 2017) | regional |

Join of the ranges: Surveyor frames resolve mm-dm fragments; orbital counts (Lunar Orbiter, LRO NAC) start at ~1-2 m.
Cintala & McBride (1995) found the orbital (> m) slopes steeper (-3.5 to -6) than the Surveyor ones (~-2) and could
not decide whether one law spans both; Li et al. 2017 (LRO near the Surveyor sites, exponents -1.5 to -3.6) argue a
single power law fits mm to m. We extrapolate the Surveyor VI law and check it against the orbital data: it covers
0.68 % of the area with 0.5-10 m rocks (Diviner: typically < 1 %); Surveyor I/VII laws (slope ~1.8) would give
several % (Cintala & McBride: they overestimate the large blocks), Surveyor III is a crater interior. Steeper
decametre slopes (Bart & Melosh 2010: mostly -3 to -4.5 around fresh craters; Cintala & McBride Surveyor VII
orbital -4.0) belong to ejecta fields, which the abundance field raises locally.

### Hapke
| Value | Tag | Source |
|---|---|---|
| w 0.429, b 0.220, c 0.514, B_S0 1.61, h_S 0.054 | [D] | medians over the brighter half (w above the global median, highland proxy; 24,660 1-deg cells) of the PDS map WAC_HAPKEPARAMMAP_643NM (Sato et al. 2014, JGR Planets 119, 1775), computed by us; cells with fill values excluded. Global medians: 0.395, 0.228, 0.420, 1.66, 0.051; darker half (maria proxy): 0.353, 0.237, 0.327, 1.73, 0.049 |
| B_C0 0, h_C 1, theta_bar 23.657 deg | [S] | fixed in Sato et al. 2014 (map README) |
| K 1 | [S] | Barker et al. 2016: Sato et al. used K = 1. Discrepancy recorded: the map README lists "phi - filling factor - fixed at 1.0" (band 9), while the data band as read by our script is 0; in Hapke 2012 K = 1 corresponds to phi -> 0 (phi = 1 would leave K undefined), so the README's phi either means something else or is wrong. The README is not cited as support for K = 1 |

Cross-check: Barker et al. 2016 (LOLA 1064 nm, Icarus) quote Sato's highlands b = 0.23-0.24, B_S0 1.60 at 689 nm,
h_S 0.075-0.08, maria h_S 0.05; our highland-proxy medians agree except h_S (0.054: the w-split mixes in
intermediate terrain). The map stops at 70 deg; 60-70 S cells give c 0.78, h_S 0.018 - not used, the polar opposition
geometry (g near 0) is not observed there. The published global-average table of Sato et al. was not accessible
(paywalled).

Validation references (Task 9; `tools/data_pipeline/sources.toml` `lit_hapke_validation`): USGS ISIS3
`isis/src/base/objs/Hapke` (Hapke.cpp, unitTest.cpp, Hapke.truth; public domain) - its roughness and IMSA code match
ours term by term; ROLO coefficients for 665.1 nm (a0..a3 -1.88914, -1.58096, 0.30477, -0.17908; b1..b3 0.04415,
0.00983, -0.00389; d1..d3 0.37141, -0.13514, 0.01248; p1..p4 4.06054, 12.8802, -30.5858, 16.7498) as transcribed by
J. Hollan from Kieffer & Stone 2005 Table 4 (amper.ped.muni.cz, "kindly sent by Tom Stone"; not checked against the
paper, whose PDF was not reachable); Chandrasekhar's H(1) = 2.9078 for conservative isotropic scattering.

## Open points
- Summed roughness passes at every level but L20 / L19 sit 1.4-1.9 % inside the gate (3.65 / 3.68 mm, -8.6 / -8.1 %;
  the rocks, Task 11, add ~0.8 mm^2 -> ~3.76 mm). The crater textures' realisation matters at that level: the s1 tile
  holds 0.74 x the law's crater height variance (a handful of 16-32 cm craters carry it; their overlaps are random). A
  larger tile or several crater tiles per scale would tighten it.
- The texture craters cast no shadows inside their bowls - the main cue of small craters under a low Sun (the LROC
  frame's dark crater floors); they only modulate the radiance (mean-preserving). Task 12's shadow map, or horizon maps
  of the crater textures, would add them. Craters above 20 m (beyond s3) on coarse leaves are left to the mesh (it
  draws ~30-60 % of their slope variance at D = 4-8 L).
- Texture metric: the textures (regolith and craters) live in nominal face metres; the equi-angular cube's true metric is
  0.67-1 x that (det J 0.71-1), so at a cube corner the texture features are up to 1.5 x smaller and the crater density
  up to 1.41 x higher than the law (IM-2 lies mid-face). A per-node metric (as the shader-crater route had) would
  correct it.
- Approximations of the modulation (review M6): its k is the Lommel-Seeliger derivative d ln R / d grad h while the BRDF
  is Hapke (the mean is unaffected - the modulation is odd in grad h - only the contrast's scale); Smith's elevation is
  taken over the local radial horizontal / the horizon-map margin, not over the mean plane of the H-scale relief.
- The texture weights use an approximate mesh transfer in GDScript (sinc^6 for the regolith band, the core's crater
  table for the craters; review M2): the core's exact value-noise transfer (drawn(L)) is not used for the textures,
  because they are a property of the baked bands the core does not know; the summed-roughness gate checks the result.
- Site brightness under the polar Sun (Task 9 found the terrain 20-25 % darker than with Lommel-Seeliger): with the
  residual theta_bar the near ground loses less to Hapke's shadowing (theta 20.2 deg at 1 cm: x1.02-1.25 by phase at
  i 83, e 70 deg), but the drawn cm relief now carries Smith's statistical shadows (G1 0.70 at 7 deg, 0.40 at 3 deg for
  a 1 cm node under a 20 m horizon grid) and the rocks their shadows (x0.96 at 7 deg). Net, IM-2 2026-11-22T01:20 (Sun
  6.75 deg), mean 8-bit against Task 9's shading in the same views: -13 % at 2 m, -6 % at 20 m, -3 % at 200 m
  (Validation, fix round).
  Both are physics; no term was tuned.
- Two roughness-shadowing theories meet at the fade: far away Hapke's 1984 model (theta_bar) shadows the relief, near
  the camera the drawn mesh with Smith's G1. For Gaussian slopes they agree within ~5-17 % looking down-Sun or across
  it, but looking into a low Sun the microfacet side is up to 2-6 x darker (numpy experiment, Task 10 report): a
  brightness gradient with distance in that view direction. The near-field shadow map (Task 12) replaces G1 inside its
  range with real shadows (then the same comparison against Hapke applies).
- The regolith texture relief (s0-s3) casts no shadows; it only modulates the radiance (mean-preserving). Real cast shadows of
  pebbles (horizon maps of the height textures, the spec's "height-map pebbles with self-shadowing") are not built.
- Rock lit-face gain and cast shadows assume a nadir view and half-spheroids (eta from the generated rocks); oblique
  views (rocks hiding ground, side faces seen) are not modelled. Task 11's geometry-vs-statistics check will show the
  size of that.
- `rock_geometry_pixels` is 0 until Task 11 (no rock geometry; every rock statistical).
- Texture coordinates follow the cube face lattice: across the 12 cube face edges the regolith texture is discontinuous
  (statistically the same both sides; a visible line only at cm-m footprints right at an edge; IM-2 is mid-face).
- drawn(L) counts the vertex normals' slopes; the rasterizer interpolates normals across triangles (smoother, a little
  less slope variance drawn than counted).
- The abundance map is 1/8 deg (3.8 km): finer Diviner structure (crater ejecta below ~4 km) is smoothed.
- ROLO coefficients (665.1 nm) are a third-party transcription (Hollan, from Tom Stone), unchecked against Kieffer &
  Stone 2005 Table 4 [E]; only the shape is gated (a level error would not show). The user may check the table. ROLO
  validity is 1.55-97 deg (paper sec. 3.3.1); 97-120 deg has no gated reference (K&S 1991 informative only; Lane &
  Irvine 1973 or Buratti et al. 1996 would add one).
- Future, physically better: one global w scales the albedo map only as a gain; per-texel w by inverting Hapke at the
  WAC mosaic's normalisation geometry (i 30, e 0, g 30; a monotone 1-D LUT) and anchoring the level to ROLO would drop
  `moon_hapke_norm` (maria have a smaller multiple-scattering share). The level offset m_Hapke - m_ROLO = -0.377 mag also
  shows the highland-proxy w is not the near-side mean.
- The full-Moon normalisation is the phase-0 point (Ruling T9 I1: accepted, no retuning); at 2-3 deg phase the Hapke disc is 3.6-3.8 % below the former
  globe's (Hapke's surge narrower than K&S's linear one, close to ROLO's), at 7 deg 5.8 % above.
- Diviner rock abundance does not cover the shipped site (-84.8 deg); there the uniform population applies (done). The
  full 88 MB product is loaded by `shared_moon_surface` (0.44 s incl. gap fill) and must be fetched
  (`fetch.py diviner_rock_abundance`); how it is packaged for the release is the user's decision (open).
- Contact surface: the pads feel the relief on an 8 cm lattice (sub-lattice RMS 4.4 mm) and rocks >= 5 cm (both [E],
  below the pad sinkage); the 8 cm (instead of 4 cm) lattice was chosen in fix round 1 of Task 4 for the six-pad cost
  gate (sliding 34-36 us against 48-51 us) - one constant (`contact_lattice_level`) if the controller wants 4 cm back.
  The disc pattern (25 samples) can miss up to ~1.5 cm of sub-pattern roughness. Rocks: the contact top is the
  template's column tops interpolated (<= 1e-4 m from `rock_top` for the tested rocks); the greatest protrusion is
  found at a column node by a climb from the peak (a local maximum: a rock with two separate summits under the pad may
  be read at the lower one) or along the rim arc.
- Pivoting pad: the pad stays parallel to the plane fitted to the ground and is lifted by the highest protrusion (the
  ruling); a real ball-joint pad on a rim rock would tilt onto the rock and the ground opposite, lifting its joint by
  roughly half as much. No joint range limit is modelled.
- Upper bound for skipping feet: `bound_slope` 2 is a practical bound (overlapping steep crater walls could exceed it
  in principle); a violation only delays a foot's contact query until it is below the bound.
- Physics cost: the first tick on a cold surface ~0.2 ms. The touchdown hazard check runs on LOLA-admissible
  candidates only (IM-2: 136 ms cold on 22 threads, 172-176 ms serial in the same session; 0.4-0.8 ms for the second
  approach direction from the memo). A refused site checks every candidate: the whole 1 km search at IM-2 has 7,449,
  i7-13700K, 22 threads: 15.8 s when every candidate fails at its first roll (serial 168.9 s), 29.5 s worst case
  (every candidate through the full roll sweep and the base check; serial 250-290 s), 0.1-0.2 s for the next call
  from the memo. Per check the threads take ~2x the serial time (hyper-threads and E-cores), so the speed-up is
  ~10x. Synchronous at the mission start; no budget cap (a cap could refuse a site whose safe spot lies farther out).
  The worst case is over the 20 s target: `docs/known-issues.md`.
- No terminal re-targeting: after a dispersion the descent flies to the planned point; a live re-selection would call
  the same `select_touchdown_point` on the live state (spec: re-plan from flight data) - not built.
- The roll at touchdown is free, so every roll must be safe (conservative); a roll-controlled touchdown could pick the
  roll that clears the rocks.
- Lunar landing campaign on the composed surface (`[lunar_campaign][slow]`, Task 5 fix round 1): 21 / 21 within the
  1 m gate, misses 0.01-0.91 m (seed 17 0.91 m and seed 5 0.89 m are the closest to the gate; seed 7, 1.12 m before
  the fix, now 0.09 m). The earlier two-foot stance (~50 s, ~2 cm/s creep) had two causes, both fixed: the guidance
  held the attitude with the RCS after the first contact (a rate damper about the line through two feet - now
  released at contact) and the regularised leg friction crept under any tangential load (now Coulomb stick-slip,
  `contact::leg_contact`).
  The 0.89 / 0.91 m of seeds 5 / 17 (slopes 5.1 / 5.3 deg) arise after the first contact, not in the guidance: at
  first contact the footprint centre is 0.02 m off (lateral 0.009 / 0.006 m/s; `LunarTouchdown::first_contact_offset_m`,
  printed by the campaign). Trace of seed 5 (200 Hz): the offset stays <= 0.1 m while the hull tips onto the downhill
  feet up to ~5 deg (2.3 s), grows to 0.89 m while it overshoots to 7.8 deg and swings back (2.3-7.4 s), then stays
  constant while the hull rocks 3.2-5.7 deg for ~15 s. Mechanism (foot slip during the tip vs. leg stroke) not traced:
  open point.
- Two Moon surfaces in one tick: the body's own ground contact (`FlightBody::resolve_ground_contact` via
  `ReferenceBody`) uses the full-detail point surface (relief at every point, rocks >= 1 cm, exact `rock_top`, normal
  over +-1 m, ~13 / ~50 us), the legs and base ring the contact surface (8 cm lattice, rocks >= 5 cm, interpolated
  rock tops). They differ by mm-cm; routing the body contact through the contact surface would need the body contact
  to know the site frame - left as is (a hull touching the ground is an impact either way).
- Point queries with detail (ground contact of every body at the Moon via `ReferenceBody`, two per body per tick in
  `FlightBody::resolve_ground_contact`) cost ~13 us each (LOLA alone ~0.02 us); `normal_fixed` with detail ~50 us.
  Telemetry `altitude_terrain_m` stays on LOLA.
- The real map's median (0.002-0.004) is about half the reference 0.0068: with "diviner" the mid latitudes get ~0.5 x
  the Surveyor VI population. Points at the reference's 0.5 m lower limit or at the law (open, not changed).
- Size convention of the Surveyor fragment counts (long dimension vs mean / area-equivalent) not determined from the
  original paper; D is taken as the plan-view-area-equivalent diameter (exact for Diviner's area fraction). If the
  counts used the long dimension, rocks of a given count are ~10-30 % smaller than generated.
- Rocks stand on the tangent plane at the base point, not tilted: on a slope s the ground falls away downhill, so the
  underside is exposed there and the uphill side buried deeper (ellipsoid c/a 0.5, f 0.3 at 20 deg: the downhill
  ground crossing moves from ~0.92 a to ~0.6 a). Crater walls near the site reach such slopes. Fix: burial against the
  lowest ground under the footprint, or tilt large rocks.
- Burial, rock albedo ratio and degraded d/D are estimates; no direct lunar measurements found.
- One Hurst exponent for cm to tens of m although the measured breakover is 1-2 m (H 0.78 below, ~0.9 above).
- Units of Shoemaker & Morris' 10^10.9 not confirmed from the original paper (used only as a comparison).
- The rock law is the Surveyor VI (mare) fit, used also at the polar highland site (Diviner does not reach -84.8 deg);
  highland rock populations may differ (Surveyor VII is Tycho ejecta, not typical). Same kind of caveat as Guo's
  Von Karman floor for the roughness.
- Units of the Cintala & McBride Table 3 fit (D in m, N per 100 m^2) read from a scanned table; the Diviner-scale
  area check supports metres.
- Diviner's lower rock-size limit (0.5 m chosen) is not pinned down; it sets the local abundance factor within 1.45x.
- The roughness residual (3.7 mm) rests on the generator's crater profile (re-derived, unchanged) and an assumed rock
  profile; Task 3 validates the summed surface against Guo's 4 mm (see Validation).
- band_start uses the global grid's latitude spacing (1.9 km) everywhere; near the poles LOLA_16's longitude spacing
  is finer (172 m at -84.8 deg), so outside the tiles there some 0.2-1.9 km detail is both in LOLA_16 and generated.
- Roughness is ~6 % rougher near the 8 cube corners (skewed face lattice; craters + roughness 4.09 mm there, above
  Guo's 4 mm before rocks): Task 3's summed check should include a corner window, or the amplitude could take the
  lattice skew into account. (Stationarity numbers measured before the face blend of fix round 1.)
- Queries inside the face blend band (+-13.6 km along the 12 cube edges) cost ~3-6 x more (cross footprint means);
  terrain nodes there build ~2x slower (fix round 1 of Task 7: 44-59 ms per node against 24-34 ms elsewhere).
- Terrain horizons are baked over LOLA with LOLA observer heights, while the drawn surface is band-limited (up to ~10^2 m
  apart at owner levels outside the site tiles): the procedural relief casts no terrain shadow beyond the near-field
  shadows (Task 12), and crater floors / rims read their horizon from the LOLA height (at south-pole Sun elevations of
  1-2 deg ~1 deg of horizon error over a few km). Within the plan's design (Task 7 review M2).
- Generated craters up to 2 km and roughness up to ~1 km wavelength outside the tiles are statistical, not the real
  features at those places (LOLA_16 does not resolve them).
