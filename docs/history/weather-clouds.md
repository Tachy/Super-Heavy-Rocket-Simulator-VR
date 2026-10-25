# Weather clouds - history

Superseded states of `docs/subsystems/weather-clouds.md`. Read only when the reasons behind the current design are
needed.

## Task 8 first version (commit 312b7cc, 2026-10-25)

- **Raymarch in a spatial shader**: the full-screen quad itself raymarched at full resolution (first in the
  transparent pass), jitter a PCG hash per pixel and frame (white noise; interleaved gradient noise drew diagonal
  hatching), no temporal accumulation. Measured at the VR preset (terrain_perf, eclipse epoch, ERA5 18:00 pinned):
  p99 13.0 -> 17.4 ms, GPU +4-5 ms on the pad, up to +17 ms ~8 km above a full deck; the per-step fixed cost dominated
  (light samples 4 -> 1 changed little, coarse steps 64 -> 32 saved 3-5 ms). Replaced by the compute cloud pass at 1/2
  resolution with per-eye temporal accumulation (amendment, controller ruling).
- **Hard coverage edges**: in-cloud extinction mean / cf where the shape exceeded the threshold, zero elsewhere. User
  feedback on the first render (tmp/weather_t8_starbase_era5.png of that version): "cotton balls" - flat, uniform
  brightness, hard grainy edges; "Die Ränder sind viel weicher - es liegt am Alpha-Übergang" (MSFS 2024 comparison).
  Replaced by the mean-preserving soft edge ramp.
- **Plane-parallel Sun depth per column**: the TMS term took tau_s from the light march, but the diffuse field was that
  of the column top's incidence mu0 E: cumulus sides facing a low Sun too grey, nothing below the local horizon (no lit
  bases at sunset). Replaced by the entry-surface source (`cloud_scattering_source_3d`).
- **Linear vertical interpolation**: bases rounded like the tops ("cotton balls" from below). Replaced by the vertical
  profile rule (flat bases). Ice cloud had the same shapes as cumulus (the 8 km ice cloud over Starbase on 2024-04-08
  rendered as cotton balls); now thin sheets.
- **Uploads**: one `ImageTexture3D` per snapshot created on the main thread, 49 ms (a VR frame drop at each hour
  change). Replaced by `CloudFieldTexture` (one layer per frame: 5.2 ms per step; two layers took 6.6 ms).

## Task 8 amendment (commits 8161738..1c8bc71, 2026-10-25)

- **Smooth shapes at a distance, uniform edge band**: the shape noise (Perlin gain 0.5, finest octave 500 m) with a
  multiplicative detail factor 1 + a (d - mean) (Worley fBm, tile 2 km) whose amplitude faded linearly to 0 at 30 km
  (desktop) / 15 km (VR); a 100 m edge ramp at the noise's median gradient everywhere. User feedback 2 on
  tmp/weather_t8_starbase_era5.png: low clouds at the horizon had smooth, round, cut-out silhouettes and grainy rims
  ("brauchen weichere Ränder, müssen unregelmäßigere Ränder haben ... echte Alpha-Übergänge"). Replaced in fix round 1
  by the fractal value (Lovejoy's D = 1.35, detail levels band-limited by the pixel footprint) and the edge zone of
  varying width (25-200 m).
- **Temporal weight 0.1** (exponential mean, 23 % of one frame's noise left) and point samples over deep steps
  (grain at bases seen at a grazing angle): replaced by the running mean (floor 1/32) and the step refinement.
- **Sky light without the eclipse** (`atm_sky_light`): the clouds glowed against the darkened scene at 80 % partial
  phase; evaluated per sample the eclipse factor cost ~10 ms at the eclipse epoch, so it is now probed per work group.
- **Column depth conventions**: the point's in-cloud extinction mean / m(cf) against the A channel's mean / cf
  (a step of t_v at the layer centres); now mean / cf in both.
- **Resize**: the result texture was replaced inside the render callback (one renderer error, one frame without
  clouds); now shown by the main thread and freed later.
