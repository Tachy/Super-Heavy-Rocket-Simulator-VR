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

## Closed-form Sun source as the game's default (Task 8, replaced in Task 8b, 2026-10-25)

- **TMS on the delta-Eddington source function** (`cloud_scattering_source_3d`, still available as the fidelity flag
  `CloudLayer.ScatteringModel.CLOSED_FORM` / `cloud_scattering_model 0`): against the Monte Carlo fixtures forward-
  scattered radiance at a low Sun up to -56 % (Sun 60 deg, view 60 deg forward, tau 10: -44.5 %), the albedo at Sun
  60 deg up to -32 % (tau 1-3), the semi-infinite albedo 0.90 at mu0 0.5 and 1.16 at mu0 0.1; the TMS aureole through
  thick cloud too bright (no IMS). User feedback vs MSFS 2024: no backlit glow, weak silver linings, no inner glow of
  cumuli. Replaced as the default by the Monte Carlo transfer table (Bouthors et al. 2008).
- **First table form** (Task 8b, uncommitted): the table held the whole multiple-scattering source incl. the phase
  function's HG part (g 0.995, a ~0.3 deg peak) with the view ray attenuated by exp(-tau). Tallying the HG part needs
  the radiance within 0.3 deg of every view direction: relative 1-sigma median 1.9 %, p99 55 %, 25-50 % at small
  scattering angles deep in thick slabs (rings around the Sun under a deck) and zeros at tau* 256. Replaced by the
  Draine part with the view ray attenuated by exp(-w_D tau) (median 0.5 %).
- **Performance after Task 8 fix round 1** (closed form, same setup): without clouds p99 13.20 / 13.06 ms, with clouds
  15.54 ms; descent 12.1 -> 16.7 ms p99, worst GPU second 18.9 ms at 8.4 km.

## Point-sampled shapes at kilometre pixels (Task 10, replaced in its fix round 1, 2026-10-26)

- The volume read the shape texture point-sampled at any footprint; only the detail levels were band-limited. From
  the scaled-space limit (8 600 km, ~7-27 km pixels) its octaves aliased: moire and speckle around the sub-camera
  point, the background transmittance at the centre 0.85 against the column's 0.49. The hand-over against the far
  field (reported then, not asserted): deck tau 10 up to 8.2 %, tau 2 up to 6.8 %, broken cf 0.4 (21 x 21 block means)
  up to 32 % (centre: volume 0.154, far field 0.202). Ruling 2026-10-26: band-limiting the shape at footprints larger
  than its features is sampling, not a shape change; views below a footprint threshold stay bit-identical.
- First fix-round attempt: the mips up to level 4 (4 km texels) alone: the moire stayed at 27 km pixels (cf 0.4 65 %);
  with sub-pixel offsets beyond level 4: 10.7 %, rings from the coarse mips' slice cover; unfiltered shape + offsets
  only: 11.6 % (the volume brighter everywhere). Kept: mips up to level 2 + offsets (7.5-8.5 %).

## The cloud pass at long range before Task 10 (fixed 2026-10-26)

- **Step retry on the float32 distance**: a step heavier than `CLOUD_MAX_STEP_WEIGHT` was retried while
  `b - a_this > 1.001 x refined`; far from the camera the float32 difference carries the distance's ulp (1 m at
  8 600 km, 0.25 m at 2 300 km), above the 0.1 % margin of a ~60 m step, so the same step was retried until the
  256-step budget ran out: from the scaled-space limit single rays never reached a deck (speckle; counted per
  pixel: 40-170 of the 256 steps were retries). Now the intended step length is compared.
- **The half-resolution checkerboard's nearest depth on smooth ground**: the nearest of a 2 x 2 block cut the block's
  centre ray short by the block's depth extent (several km for slanted ground seen from 8 600 km): every other
  half-resolution ray lost the clouds within that distance of the ground - a deck at 1-2 km drew with T_bg 0.375 /
  0.625 instead of 0 outside the sub-camera region. Now a block within 5 % in distance takes its mean.

## Performance measurements before the Task 11 gate (superseded by the Task 11 section of the subsystem file)

Moved verbatim from `docs/subsystems/weather-clouds.md` (terrain_perf 1440p, 2024-04-08 18:42 eclipse epoch, ERA5 18:00
pinned, VR preset 64 / 128 steps; the machine state differed between these notes - see the Task 11 entry):

- Performance (terrain_perf 1440p, 2024-04-08 18:42 eclipse epoch, ERA5 18:00 pinned, VR preset, one session, Task
  8b): without clouds p99 13.15 ms, with clouds 15.50 ms with the table / 15.13 ms with the closed form (p50 8.92 ->
  9.65 / 9.60); the table costs +0.4 ms p99, the descent 11.97 -> 16.69 ms p99, worst GPU second 21.3 ms (closed form
  20.5) at 8.4 km above the decks. One run had a 48 ms frame in the descent at 19 m (GPU 8.6 ms: a draw / present stall,
  not the clouds' GPU work). The baseline itself misses the Task 11 gate (p99 <= 11 ms). Levers measured: the detail levels ~2 ms in the descent
  (`--cloud_detail_levels`), the step refinement ~0.3 ms, the eclipse probes ~0.3 ms (per sample they cost ~10 ms).
- Performance after Task 9 (terrain_perf 1440p, same epoch and ERA5 file, VR preset, fix round 1: five runs with
  clouds, four without): with clouds p50 5.44 / p99 9.87 9.97 9.79 10.03 9.95 / max 14.80 15.89 14.26 14.42 19.33 ms;
  without p50 4.53-4.56 / p99 7.65 7.58 7.62 7.59 / max 12.90 11.31 10.66 45.81 ms. All clouds cost +0.9 ms p50 and
  +2.3 ms p99 over the baseline (budget +3 ms; the Task 8b state measured p99 9.45, so the light added ~0.5 ms p99).
  VRAM +464 MB at the Cape (2884 vs 2448 MB: two snapshots of field, ice and column textures, 232 MB each). The
  39 ms frame of the first Task 9 run (t = 0 of the descent, main-thread terrain start, process 37.6 ms) did not recur in
  nine further runs, not even with a never-compiled terrain shader source (cold shader cache) - so it is not the first
  compile of the larger terrain shader, nor the transfer table's build threads (they finish at setup, before the run),
  nor the column textures (created by `show_now` before it). Sporadic draw / present stalls of 16-46 ms occur with and
  without clouds (one 45.8 ms frame in a run without clouds: "rest 42.9, draw 42.6", GPU 2.6 ms), i.e. they are the
  platform's, not the clouds'. The 11 ms gate is Task 11's.
- Performance after Task 10 (same terrain_perf setup; the machine ran a VR runtime in the background, so the baseline
  is higher than Task 9's): without clouds p99 12.65 / 12.64 ms, with clouds 16.10 / 16.33 / 16.13 / 16.08 ms (p50
  8.4 -> 9.7); the Task 9 cloud pass in the same state 15.90 / 16.09 ms: the two cloud-pass fixes cost nothing
  measurable. Scaled space (2560 x 1440, 2025-10-25 15:00 UTC ERA5): the far field adds 0.26 ms GPU from the Moon's
  direction at 330 000 km (0.51 -> 0.77 ms) and 0.30 ms just outside 1.5e7 m (0.43 -> 0.73 ms); frame p99 ~1.5 ms.
  The shape prefilter kernel (fix round 1, A/B in one session, 2560 x 1440, ERA5 2024-04-08 18:00, inside the limit at
  1.4985e7 / 1.2e7 m): GPU p50 6.09 -> 6.29 / 6.07 and 8.05 -> 8.03 / 8.08 ms: not measurable.

## Light marched per sample (until static clouds Task 1, 2026-10-26)

- **Light** (the superseded sentence of the subsystem file): "the Sun's (and at night the Moon's) optical depth is
  marched toward the light through the 3D density (`cloud_light_optical_depth`: near samples over 6 km in geometric
  intervals from 20 m through the shapes, then far samples to the cloud top through the grid mean; stratified,
  unbiased over the jitter)." Replaced for the Sun near the camera by the light cache (the light depends on the Sun,
  the field and the point, not on the camera: computed once, looked up); the march stays outside the cache and for
  the Moon. The brief's start values (128 x 128 x 64 voxels, a 24 + 8 sample fill) failed its own check (broken cloud
  at a 20-deg Sun: 78 % at a pixel; 24 samples 30 % off the converged integral) and were raised to 512 voxels up and
  96 + 32 samples.

## Light cache, one cascade refilled whole (static clouds Task 1, replaced in Task 2, 2026-10-26)

- The superseded Light bullet: "the Sun's optical depth toward the light tau_light comes from the **light cache**
  (static clouds Task 1; `CloudLightCache`, `shaders/cloud_light_cache.gdshaderinc`): one Earth-fixed cascade of 128 x
  128 x 512 voxels (R32F, 32 MB) over 8 km x 8 km x the shell (-0.5 .. 20 km; 62.5 m across, 40 m up), anchored on the
  ground below the camera (axes east / north of the anchor's tangent plane, height above the ellipsoid), filled by the
  `CLOUD_LIGHT_FILL` kernel with `cloud_light_optical_depth_with` at each voxel centre: 96 near + 32 far samples at the
  stratified midpoints (deterministic), the detail band-limited at least at the voxel's 62.5 m. The march looks it up
  trilinearly (`cloud_cached_light_depth`); outside the voxel centres' range, while not filled, with the flag off, and
  for the Moon it marches as before (`cloud_light_optical_depth`: near samples over 6 km in geometric intervals from 20
  m through the shapes, then far samples to the cloud top through the grid mean; stratified, unbiased over the jitter) -
  one call site, so outside the cascade the image is the march's bit for bit. `CloudLayer` (main thread) requests a fill
  when the camera's ground point is more than 2 km from the anchor, the Sun turned by 0.25 deg, w1 moved by 0.05, or the
  slots, the quality, the detail levels or the debug output changed; `CloudEffect` (render thread) fills the whole
  cascade before the march of that frame (Task 1: no progressive fill, no generations - Task 2) and hands the lookup its
  frame (`CloudShaderPort.pack` overrides: `cloud_light_cache_valid / _origin / _east / _north / _extent / _size`), so
  the march reads only a complete fill with the frame it was filled for. Fidelity flag `CloudLayer.light_cache_enabled`
  (global `cloud_light_cache_enabled`; off = the per-sample march everywhere, the reference)."
  Replaced by five cascades with progressive, budgeted, cross-faded refresh: a whole-cascade refill cost 5-8 ms GPU in
  one frame (against the 2.5 ms budget) and the single 8 km box ended in a hard seam.
- The superseded validation text: "Section 8, the light cache vs the per-sample march with the cache's own sample count
  (96 + 32, jittered, 8 frames, the view's jitter pinned alike) at every pixel of a 64 x 64 view from 5 km (error
  relative to max(pixel, 5 % of the view mean)): a tau-10 slab (debug 2) worst 0.83 / 1.03 % at Sun 45 / 20 deg
  (tolerance 5 %), a cf 0.4 broken layer (debug 7, the shapes) worst 3.55 / 8.93 %, mean 0.13 / 0.53 % (tolerance 10 %);
  a view whose samples all lie > 5.6 km north of the anchor bit-identical with the flag off and on; the fill 5-8 ms GPU
  (RTX 4090, 8.4 M voxels, the check's slabs). Measured on the way (broken, Sun 20 deg): 64 / 256 / 512 voxels up worst
  78 / 23 / 8 % (vs a midpoint march; 256 across changed nothing); a 24 + 8 fill matched the 24-sample march within 2 %
  but both were 30 % off the converged integral at a pixel, 48 + 16 left 10.6 %."
- The superseded limit: "- Light cache (Task 1 state): one 8 km cascade, refilled whole in one frame (5-8 ms GPU on the
  check's slabs, more in dense weather) when re-anchored, the Sun turned 0.25 deg or w1 moved 0.05 - a spike against the
  2.5 ms budget and a lag of up to 0.05 in the blend; cascades, the progressive fill and cross-faded generations are
  Task 2. A view from altitude (9 km, 10 deg down) sees no cloud inside the cascade (the image is the march's)."
- Task 2's first layout, x8 per cascade (8 / 64 / 512 km, 96 MB): the slab held everywhere, but broken cloud at its
  scale was 42 / 193 % (64 km) and 129 / 1005 % (512 km) off at a pixel; replaced by the x2 clipmap (controller
  ruling: 5 levels to 128 km, the march ends at the 30-50 km hand-over, VRAM budget 128 -> 192 MB). Its fill at
  524 288 voxels per frame cost 0.74 ms (budget lowered to 327 680).

## Light cache read at the sample in every cascade (static clouds Task 2 round 2, replaced in round 3, 2026-10-26)

- Superseded fill sentence: "The fill is progressive: the
  schedule hands the render thread at most 327 680 voxels of jobs per frame (~0.5 ms GPU), a full refresh of all 42 M
  voxels takes 128 frames (1.4 s at 90 Hz)."
- Superseded lookup sentence: "Lookup (`cloud_cached_light_depth` -> (weighted tau, weight left)): cascade k has weight
  1 within 34 voxels of the ground point (Chebyshev), falling to 0 at 42 (a band of 8 voxels: 0.5 .. 8 km), the rest
  goes to the next cascade and after the last (beyond 34-42 km) to the per-sample march"
- Superseded validation text: "Section 8, the light cache vs the per-sample march with the cache's own sample count (96
  + 32, jittered, 8 frames, the view's jitter pinned alike) at every pixel of a 64 x 64 view per cascade at its scale
  (camera 5 / 9.9 / 18.8 / 36.5 / 72 km up, 80-81 deg down; the farthest cloud samples asserted inside that cascade's
  full weight; error relative to max(pixel, 5 % of the view mean)), the fill at 1/16 of the voxels per frame (complete
  after 16 frames): a tau-10 slab (debug 2) worst 0.48-1.25 % in every cascade (tolerance 5 %); a cf 0.4 broken layer
  (debug 7, the shapes) cascade 0 worst 2.71 / 4.60 % at Sun 45 / 20 deg (tolerance 10 %) - cascades 1-4 FAIL: worst
  12.7 / 18.0, 14.2 / 26.0, 21.2 / 62.8, 44.7 / 107.7 %, mean 0.3-5.1 % (open, see Known limits); a view beyond every
  cascade (complete fill) and the frames right after a clock jump bit-identical to the march. Section 10 (no seam): the
  row-to-row step of (cache - march) / mean along a line of pixels across each edge band within the image noise: 0 -> 1
  0.36 / 0.61 %, 1 -> 2 0.18 / 0.32 %, 2 -> 3 0.11 / 0.25 %, 4 -> march 0.23 / 0.59 % - 3 -> 4 0.29 % against a noise of
  0.19 % (open); the same view before / after a re-anchor of cascade 3: 0.0000 %. Measured on the way: 64 / 256 / 512
  voxels up worst 78 / 23 / 8 % (Task 1, broken, Sun 20 deg); 24 + 8 fill samples 30 % off the converged integral at a
  pixel, 48 + 16 10.6 %; a x8 layout (8 / 64 / 512 km) broken worst 42 / 193 % (64 km) and 129 / 1005 % (512 km)."
- Superseded limit: "- Light cache: broken cloud in cascades 1-4 (125 m .. 1 km voxels) misses `cloud_check` section 8's
  10 % at single pixels (12-108 % worst, mean 0.3-5 %): a voxel coarser than the shapes' 25-200 m edges blurs a cumulus'
  self-shadowing, independent of the pixel footprint - only cascade 0 (62.5 m) holds it. The seam check 3 -> 4 shows a
  0.29 % row step against 0.19 % noise (the band or cascade 3's own interpolation: not separated yet). Beyond 34-42 km
  the light is marched. The fill's per-frame GPU time measured 0.45 and 0.80 ms (worst frame, two runs, RTX 4090) at 327
  680 voxels - timing spread not yet understood; dense real weather not measured."
- Why: cascades 1-4 read at the sample missed broken cloud by 13-108 % at a pixel; the hybrid near march (1 km / 12
  samples) brings 1-2 under 10 % and 3-4 to one 40 % edge pixel. The 0.45-0.80 ms fill spread was the GPU's clock
  state and the weather (real weather ~2x the slab's cost per voxel); the budget went 327 680 -> 98 304 voxels and
  RESET_ANGLE_RAD 15 -> 25 deg (a 4.7 s refresh under 1000x warp).

## Light cache: atomic first fill, cascade-order serving, 1 km near march everywhere (static clouds Task 2 round 3, replaced in fix round 1, 2026-10-26)

- Superseded fill / refresh text: "The fill is progressive: the
  schedule hands the render thread at most 98 304 voxels of jobs per frame (<= 0.5 ms GPU, measured below), a full
  refresh of all 42 M voxels takes 427 frames (4.7 s at 90 Hz). Refresh: the Sun turned by 0.25 deg or the blend
  moved by 0.05 -> the other slot is filled completely while the old one stays shown alone, then the lookup
  cross-fades over 0.5 s. A clock
  jump (`CloudLayer.notify_clock_jump`, `SimClock.time_jumped`), a Sun jump > 25 deg, or a change of quality / detail
  / debug output / noise / the content of a slot with weight invalidates everything: the march takes over until
  filled."
- Superseded hybrid text: "Cascade 0
  is read at the sample; cascades 1-4 are hybrid (controller ruling 2026-10-26): the sample marches the first 1 km of
  its light path through the shapes (`cloud_near_light_depth`: 12 geometric intervals from 20 m, the stratified
  midpoints) and adds the cascade's tau read at that point (the fill's tau starts there, so nothing is counted
  twice) - a far cascade's voxel cannot hold a cumulus' self-shadowing."
- Superseded validation text (section 8): "a cf 0.4 broken layer (debug
  7, the shapes), worst pixel at Sun 45 / 20 deg (tolerance 10 %): cascade 0 2.71 / 4.60 %, 1 3.38 / 9.58 %, 2 7.10 /
  9.37 %, 3 8.09 / FAIL 39.6 %, 4 5.81 / FAIL 39.8 % (mean errors 0.12-0.81 %; the two failures one pixel at the
  view's edge at a 20-deg Sun, open);"
- Superseded limit: "- Light cache: broken cloud in cascades 3-4 (hybrid) misses `cloud_check` section 8's 10 % at one
  edge pixel of the
  view at a 20-deg Sun (39.6 / 39.8 %, mean 0.5 / 0.7 %); a longer or finer near march did not change it (1 km / 16
  samples the same, 2 km / 8 worse). The hybrid near march costs the cloud pass +45-65 % in the VR proxy
  (`vr-performance.md`): over the 2.5 ms budget in VR, open. The fill's GPU time per voxel depends on the GPU's clock
  state and the weather (real weather ~2x the slab's); the budget (98 304 voxels) keeps the slow state under 0.5 ms.
  Beyond 34-42 km the light is marched (the volume march ends at <= 34 km)."
- Why: the review (headless probe of the schedule) found the far cascades starved under time warp - cascades 0-1
  started a new generation as soon as the last fade ended and took the whole budget, the reset angle was checked on
  cascade 0 only: the shown light of cascades 2-4 drifted without bound (100x: 12.5 deg and growing, 1000x: 125 deg).
  A first fill was all-or-nothing (the whole window as one box, valid only when all 512 layers were done: 85 frames
  for cascade 0) and restarted on every re-anchor and on a Sun turn > 0.25 deg, so at 100x / 1000x warp and at 2 or
  7.8 km/s no cascade ever held anything. Now: progressive strips around the ground point, kept through re-anchors and
  small turns; fair sharing of the budget; one generation at a time, the stalest cascade first; the reset angle per
  cascade and slot (the generation in progress takes over). The cascades 3-4 "edge pixel" failures were cascade 3's
  own pixels (not a blend band: their rays lie in cascade 3's full weight), the shadow of the next cumulus beyond the
  1-km point smeared over +-dx by 500-m voxels; seen from the higher cameras, cascade 2's pixels failed too (32 %,
  found with the new per-cascade worst-pixel diagnostics). Longer near marches (3 / 2 / 2 km) with 100-m intervals
  after the geometric 12 bring every cascade under 10 %.

## Static clouds Task 3: deterministic march (2026-10-26)

- Superseded (cloud pass, step 1): "fine steps inside; blue-noise start jitter,
     64 x 64 void-and-cluster mask offset per frame by the golden ratio - the same for both eyes); background through" - the start jitter is now only the reference
  path's (`CloudLayer.deterministic_march` off).
- Superseded preset: "- **Quality presets** [E]: desktop 128 coarse / 256 max steps, fine 1.2 %, light 6 near + 3 far samples; VR 48 / 96,
  2.5 %, 5 + 2 (Task 11: 64 / 128 before; see Performance); both 3 detail levels; resolution 1/2 desktop, 1/3 of the VR
  centre view (VR performance plan part 2); temporal weight floor 1/32."
- Why: the per-pixel and per-frame jitter averaged by the temporal accumulation made the clouds shimmer in VR (the
  user's flicker). Fixed sample points give a static image; the first images of the deterministic march (VR preset
  48 / 96 / 2.5 %) showed what the jitter had averaged away: cumulus bases skipped by coarse steps (cloud_check case
  9 off by 5 px; the 9 km deck view thinned) and static salt-and-pepper holes in near cumuli from detail finer than
  the step sampled at fixed points. Fixes: the cloud-ahead check of a clear coarse step, the detail band-limited at
  half the step, the VR preset at 96 / 192 / 1.2 %.

## Static clouds Task 3, fix round 1 (2026-10-26)

- Superseded (cloud pass, step 3): "a VR eye's
     tent gather has the largest radius 1 (bilinear) with the deterministic march, 2 with the jittered one
     (`CloudEffect.TENT_RADIUS_*`, the centre block's params.z)."
- Superseded preset: "VR 96 / 192,
  1.2 %, 5 + 2 (static clouds Task 3: the deterministic march lost cumulus bases with 48 coarse steps and kept only the
  large lobes with a 2.5 % fine step - its detail is band-limited at the step; the cost is Task 10's)"
- Superseded open point: "a cloud edge is binary per march pixel, so at the VR divisor 3 cumulus outlines
  show 3-px stair steps (bilinear upsample; the jittered march's mean had soft partial-alpha edges); far cumulus
  fields at 20-40 km show faint horizontal bands (the steps' iso-distance shells). Left to the headset check and
  Task 10 (tent radius / divisor / steps); the VR preset's doubled steps (96 / 192, fine 1.2 %) are not yet
  measured against the 2.5 ms budget."
- Superseded validation: section 8 ran on the jittered march (8 pinned frames per image) - it then measured the
  light over the frames' mean, not the game path's per-pixel error. Its numbers then: "a cf 0.4 broken layer (debug
  7, the shapes), worst pixel at Sun 45 / 20 deg (tolerance 10 %): cascade 0 2.71 / 4.60 %, 1 3.38 / 9.58 %, 2 7.10 /
  9.37 %, 3 5.40 / 8.99 %, 4 6.36 / 7.58 % (mean errors 0.15-0.81 %"; slab "worst 0.43-1.12 %".
- Why: review I1-I3. The 1.2 % VR fine step was a look choice at about twice the in-cloud samples (case 9 passes with
  2.5 %); bilinear showed march-pixel stairs; section 8 on the jittered path hid the game path's per-pixel cache error.
- Superseded (static clouds Task 4): soft edges of varying width, log-uniform between 25 and 200 m (`cloud_edge_width_*_m`; Wang, Geerts & French 2009 transition zone ~200 m), w_min / w_max = 0.024 / 0.072 in shape value; the rims looked frayed, now 20 .. 40 m (10 m was rejected: base-contour case 9 of cloud_check failed, desktop 3 px vs limit 2).

## Static clouds Task 5: signed detail and ice-share mixing (replaced 2026-10-26)

- Superseded text (weather-clouds.md, Model): "The thresholded value is v = s + sum_k w_k a_k (d_k - mean): the shape
  noise's R (trilinear, no mipmaps; its Perlin octaves with the fBm gain 2^-H, H = 2 - D) plus three detail levels
  [...]. A continues the shape's increment spectrum at 62.5 m, times a factor calibrated on the observable
  (`cloud_detail_amplitude_calibration` 2.5: the Worley octaves are not Gaussian, D = 2 - H alone gave D = 1.22; with
  it the box-counted D is 1.350)." and "Ice-dominated cloud (the ice share) mixes in the ramp of thin-sheet
  coordinates (vertical tile 2 km instead of 16 km [E]; no fibre direction: no wind in the snapshot), the same detail."
  Validation: "perimeter fractal dimension [...] D = 1.350".
- Why replaced: the signed detail lifted points beside a body above the threshold (beads) and cut channels into it:
  cumuli fell apart into puffs, beads and holes (user 2026-10-26: "Diese Schaefchenwolken haengen normalerweise auch
  mehr zusammen zu jeweils einer Wolke"). Measured on a 1.5 km cumulus layer at cf 0.3: 0.46 / 0.48 % of the volume
  and 1.25 / 1.89 % of the top view in fragments under 200 m (detail level 0 / all levels). Tried and rejected:
  scaling the detail by the shape's gradient (the rim displacement uniform in metres) did not reduce them (a 16 km
  plane at 8 m, all levels: 1838 -> 1963 fragments); a smaller signed amplitude (x 1.25) halved them but D fell to
  1.25-1.27. Erosion only (v = s + min(P, 0)) cut them to a sixth (x 2.5, D 1.267) but left the rims where P > 0 on the
  shape's own slope (steepness 10-90 % rise p90 41 m vs the Task-4 bound 20 m); x 3 (D 1.297) with widths from the
  eroded increments (0.0076 / 0.0137) failed the cloud_check base contour (3 / 4 px vs 2 / 3), with signed widths
  VR 6 px. Kept: a quarter of the upward detail (x 2.5, D 1.291, p90 31 m); the steepness bound became the edge-width
  maximum 40 m (ruling 2026-10-26; was "90th percentile within 20 m"). The ice-share mix needed two shape lookups per
  sample where ice and liquid met; the type rule picks one tile.

## Static clouds Task 5: one lift on the summed detail, discrete type switch (replaced in fix round 1, 2026-10-26)

Superseded text of `docs/subsystems/weather-clouds.md`, verbatim:

  The thresholded value is v = s + e(P), P = sum_k w_k a_k (d_k - mean), e(P) = P below 0 and 0.25 P above
  (`cloud_detail_lift` [E], `cloud_detail_erosion`): the shape noise's R (trilinear, no mipmaps; its Perlin octaves with
  the fBm gain 2^-H, H = 2 - D) eroded by three detail levels (isotropic tiles 1000 / 250 / 62.5 m, cells 500 m down to
  8 m; single Worley octaves with gain 2^-H, a_k = A 4^(-H k)). The detail lowers v fully and lifts it by a quarter
  (user 2026-10-26: each cumulus hangs together as ONE cloud): it erodes a body from its rim inward and lifts few beads
  beside it; the shape's low octaves decide where a cloud is (lobes, cauliflower). The quarter keeps the rims where the
  detail points up crisp (erosion only left them on the shape's slope: steepness p90 41 m). A continues the shape's
  increment spectrum at 62.5 m, times a factor calibrated on the observables (`cloud_detail_amplitude_calibration` 2.5:
  box-counted D = 1.291, test range 1.25-1.45; x 3 broke the cloud_check base contour). Measured on a 1.5 km cumulus
  layer at cf 0.3 (16 m voxels, level 0 / all levels): fragments under (200 m)^3 0.07 / 0.09 % of the volume (signed
  detail of M8: 0.46 / 0.48 %), 0.52 / 0.49 % of the top view (1.25 / 1.89 %), holes ~1 %.
...
- **Shapes by cloud type** (static clouds Task 5; `cloud_type_of` in `cloud_noise.hpp` and `clouds.gdshaderinc`):
  ERA5 / GFS carry no type, so it follows per point from the layer data: ice share > 0.5 (`cloud_cirrus_ice_share`
  [E]) -> cirrus: thin sheets (vertical tile `cloud_ice_tile_vertical_m` 2 km instead of 16 km [E]; no fibre
  direction: no wind in the snapshot); the fraction of the layer centre at or below the point >= 0.9
  (`cloud_stratiform_fraction` [E]) and not thickening toward the centre above -> stratiform: the shape tile stretched
...
  top and few, wide gaps); else cumulus (the shape tile). One shape lookup per sample at the type's coordinates (own
...
  test "Coverage and water are kept for every cloud type"). The type switches where the bracketing layer fractions
  cross the rule (no blend; formerly the ice share mixed two ramps). Shape tile 32 km across / 16 km high [E];
...
  m to 1024 m: D = 1.291, test 1.25-1.45, Lovejoy 1982; the shape alone 1.01); one connected body per cumulus
  (26-connected components of a 8.2 x 8.2 x 1.5 km layer at 16 m, cf 0.3 / 0.5, states 1 / 3: fragments under (200 m)^3
  < 0.25 % of the volume [E], measured 0.04-0.09 %; top view fragments < 1 % (0.14-0.52 %), holes < 2.5 % (0.95-1.27
  %)); cloud types (the rule on canonical columns; covered fraction within 0.003 and water / grid mean within 1.5 % for
  cumulus, stratiform and cirrus coordinates at states 0 / 3, cf 0.3 / 0.9);
...
between rows); steep edges of varying width, 10-90 % rise median 8 m (< 10 m), 90th percentile 31 m (<= 40 m, the

- Why replaced: the single lift of a quarter on the summed detail flattened the 100-500 m lobes (review I2: the
  cauliflower relief of t4_deck.png was gone in t5_deck.png); level 0 now keeps half. The discrete type switch left
  a shape seam where the bracketing fractions crossed 0.9 or the ice share 0.5, and a pop when a snapshot blend
  crossed the rule (review I1); the types are smooth weights now. A first weight version interpolated the centres'
  stratiform weights linearly, which made the interval under a deck's base centre half cumulus (cloud_far_check deck
  tau 10 with the shapes 11.1 % vs the 10 % bound); the fraction-weighted mean restored 9.1 %.


## Static clouds Task 5: covered area in a type band reported only (replaced in Task 5b, 2026-10-26)

- Superseded text (weather-clouds.md, types):
  "while the covered area grows inside a band where two patterns overlap: cf 0.3 -> 0.48-0.63)."
- Why replaced: ruling 2026-10-26 accepts the growth as a bounded transition artifact; the test now checks it against
  the union of the mixed patterns (the 0.63 was a three-type mix, not two).

## Static clouds Task 3: outlines on the march grid (replaced in Task 5b, 2026-10-26)

- Superseded text (weather-clouds.md, open points):
  > - **Deterministic march (Task 3)**: a cloud edge is binary per march pixel; the Catmull-Rom upsample rounds the
  >   stair corners at the VR divisor 3, but outlines still follow the march grid (beads of ~1 march pixel), and some
  >   mid-distance cumuli (the 20 km view's frayed ones) keep static per-march-pixel holes - the shape field's own
  >   structure at the step / pixel scale sampled at fixed points (not the scene-depth step grid: deriving the grid from
  >   the layer segment changed nothing; not the upsample). Tried in fix round 1 and dropped: a sub-pixel coverage at the
  >   first entry (2 x 2 taps over the footprint half a footprint deeper, 4 field lookups per ray) changed ~1 000 of
  >   1.44 M pixels by > 20/255 with no visible softening. Far cumulus fields at 20-40 km show faint horizontal bands (the
  >   steps' iso-distance shells). Left to Task 4 / Task 10 / the headset check (divisor 2, more steps, a shape
  >   prefilter); the VR cost is over the 2.5 ms budget (`vr-performance.md`, Task 10).
- Why replaced: the user found the outlines pixelated and stair-stepped in the t3-t5 images; the outline pass
  (Task 5b) marches outline pixels again with four rotated-grid sub-rays. Measured on cloud_check section 11 (a deck's
  limb rolled 7 deg, VR preset): the largest along-edge alpha step 0.407 before, 0.102 after.

## Static clouds Task 5b: four full sub-ray marches per outline pixel (replaced in fix round 1, 2026-10-26)

- Superseded text (weather-clouds.md, cloud pass 1b):
  > 1b. **outline pass** (static clouds Task 5b; the deterministic march only, not the debug outputs): the same march
  > kernel again with `CLOUD_EDGE_REFINE` (`CloudEffect`: its `_refine` variants, the march's image read at binding 6,
  > the result in the centre's `refined` image the temporal kernel takes instead). A pixel whose alpha differs from
  > one of its 8 neighbours' by more than `CLOUD_EDGE_ALPHA_STEP` 0.25 [E] (one sub-ray's share) is marched again with
  > 4 rays at the fixed rotated-grid offsets (+-1/8, +-3/8 march pixel) and averaged (colour and alpha; the cloud
  > distance alpha-weighted, the scene distance the block's as before); every other pixel is copied. The binary edge
  > of one centre ray per march pixel made outlines follow the march grid (stairs and beads of a march pixel that the
  > Catmull-Rom upsample only rounded); now an outline pixel holds its covered fraction in quarters - smooth along the
  > edge, still one march pixel wide across it (crisp), the same in every frame (no jitter, no noise pattern).
  > Cost: the VR proxy's cloud pass about 3x (`vr-performance.md`, open - Task 10);
- Why replaced: the VR proxy's cloud pass cost ~3x (descent 3.20 -> 11.37, pan low 6.54 -> 17.25, pan high 5.04 ->
  14.87 ms medians). Fix round 1, measured step by step (same VR-proxy run): coverage-only sub-rays with the centre's
  colour (8.02 / 11.86 / 10.18), + the coverage rays' early end and the neighbour's colour in place of a sub-ray's
  full march (6.51 / 10.06 / 8.36), + the centre as one of the four rays (5.71 / 9.16 / 7.47), + the compacted list
  (4.66 / 8.25 / 6.74). Section 11 stays at 0.103 (was 0.102).

## Snapshot file format 1 (superseded 2026-10-26, static-clouds Task 6)
- `.wthr` version 1: header, ground heights, render texels, checksum (no lifting condensation level); ERA5 hour = 6
  timed files + the static orography, GFS = 5 x 22 records + HGT surface. Replaced by version 2 (LCL block, 2 m
  temperature and dew point in the downloads); version-1 files are refused and rebuilt.

