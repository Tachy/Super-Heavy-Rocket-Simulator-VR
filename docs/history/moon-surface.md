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
