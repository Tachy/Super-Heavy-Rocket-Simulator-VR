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
