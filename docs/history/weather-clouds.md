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

## Cloud bases at the LCL (static-clouds Task 7, 2026-10-26): the LCL stored but unused, the RG8 column texture, file format 2
- Superseded text (weather-clouds.md, Model):
  > - **Lifting condensation level in the snapshot** (`WeatherSnapshot::lcl_m(row, column)`, file format 2): per column the
  >   condensation height (m above the ground) of the 2 m parcel, Romps 2017 exact (`weather-service.md`); the downsampled set
  >   and the pole rows average it like the ground. Not yet used by the renderer (the flat cumulus base is a later task).
  > - Column per pixel: `environment::cloud_render_column_texels` (RG8 per cell and layer: R the grid-mean depth from
  >   20 km to the layer centre, the A channel's log code; G the largest cloud fraction above; +66 MB per snapshot, two bytes
  >   per texel) ... Godot gives a Texture3DRD of R8G8 the luminance-alpha swizzle (R, R, R, G): the shaders read `.ra`.
  >   The empty slot's column texture is a zero LA8 volume (an R8 one reads cover 1 through `.ra`).
- Why replaced: the LCL now moves the boundary-layer condensate and cuts a flat base in the shader; the base code needs
  two more bytes per cell, so the column texture became RGBA8 (the same channels in Godot shaders and compute kernels,
  no swizzle; +66 MB per 0.25-deg snapshot). Format 3: unknown LCL is -1 (0 is the known saturated surface air) and the
  stored texels are the redistributed ones; version-2 files are refused and rebuilt. GFS pressure for the LCL: the
  standard atmosphere at the column's ground height instead of a flat 101325 Pa (1500 m ground: 84.6 kPa, LCL ~3 m
  different at a 10 K spread).

## Cloud bases at the LCL, fix round 1 (2026-10-26): base in the RGBA8 column texture, lowest run only, mean box LCL
- Superseded text (weather-clouds.md):
  > - **Cloud bases at the lifting condensation level** (static clouds Task 7; spec decisions 1-2; `weather_snapshot.hpp`,
  >   file format 3): `WeatherSnapshot::lcl_m(row, column)` is the condensation height (m above the GROUND) of the 2 m
  >   parcel, Romps 2017 exact (`weather-service.md`): 0 = saturated surface air (T - Td < `cloud_fog_spread_k` 1 K [E]),
  >   `weather_lcl_unknown_m` (-1) = unknown (no 2 m fields, invalid input); the downsampled set and the pole rows average the
  >   known values like the ground height (a box of only unknown columns stays unknown). The build (`ColumnLayers::build` ->
  >   `raise_liquid_cloud_to_lcl`, on the 32 layer means of the column) treats the lowest liquid cloud whose base is less than
  >   `cloud_boundary_layer_top_m` 3 km [E] above the ground as a boundary-layer cloud whose base is the LCL: condensate
  >   below it is moved up. Where cloud remains above the LCL, the cut part's liquid is spread over that part in proportion
  >   to its liquid (profile kept; cloud fraction at least the cut part's mass-weighted fraction, so the covered area does not
  >   shrink); a cloud wholly under the LCL is lifted to start there with the same depth and mean density. Untouched: a base
  >   at or above the LCL, an LCL of 0 (fog / low stratus keep the ground), unknown, an LCL >= 3 km, ice (a layer's cloud
  >   fraction stays where it holds ice). The column's liquid water path is conserved (1e-6 relative on the layer means; the
  >   8-bit codes then cost <= 1.97 % per layer). Limit: a very dense result clamps at the water code's 4e-3 kg/m^3 maximum
  >   (not reached by the ERA5 / GFS data tested).
  >   GPU: the column texture's layer 0 carries each cell's cloud base as a height above the ellipsoid (ground + LCL, B / A =
  >   the 16-bit `encode_cloud_lcl_base`, 0.32 m steps, 0 = none: fog, unknown, LCL >= 3 km); `cloud_lcl_cut` (shader, core
  >   `cloud_lcl_cut_factor`) reads the four nearest columns by `texelFetch` and multiplies the cloud fraction and the
  >   extinction by a ramp from 0 at the base to 1 over `cloud_lcl_ramp_m` 30 m [E], mixed bilinearly with the columns
  >   without a base counting 1: flat sharp bases, nothing hangs below them, silhouettes above base + 30 m are untouched
  >   (heights above the ellipsoid; nothing is fetched above 9.5 km). The cut loses the water between the base and the first
  >   30 m (at most ~15 m of cloud per column).
  >   - Column per pixel: `environment::cloud_render_column_texels` (RGBA8 per cell and layer: R the grid-mean depth from
  >     20 km to the layer centre, the A channel's log code; G the largest cloud fraction above; B, A in layer 0 only: the cloud base (Task 7); +133 MB per snapshot, four bytes per texel) - the
  >     centre above (four decoded texels, bilinear: linear in tau as the CPU) + the trapezoid of the grid-mean extinction
  >     down to the point; cover = max(its G, the point's fraction). The format is RGBA8 so Godot's shaders and the compute
  >     kernels read the same channels (`.rg`; the RG8 form needed the luminance-alpha swizzle `.ra` in Godot shaders only).
  > slot's column texture is a zero RGBA8 volume (cover 0, no base).
  > - `cloud_check.gd` with `STARSIM_LCL_ONLY=1`: section 12 alone (the flat cloud base at the lifting condensation level: a slab from the ground with level 800 m, optical depth straight up 0.593 of the uncut slab's vs 0.5925 expected, no cloud below the level, none lost above it).
- Why replaced (review of 9369f5f): the shader cut condensate the core never moved (only the lowest contiguous liquid
  run was moved, with no significance threshold, but a base was written for every column with 0 < LCL < 3 km: ice
  below the LCL, a second liquid run below it and real cloud above a 1e-10 trace run were erased on screen); the cut
  lost up to layer/4 + ramp/2 (~77 m) of cloud per base, not 15 m, because the LCL layer's kept mass is a mean over the
  whole layer (cloud_check on a built column: +2.75 % with a first fixed-profile compensation, 0.07 % with the
  profile-integral scale); the RGBA8 column texture wasted 31 of 32 layers (+66 MB per snapshot, two resident +133 MB)
  for a 2-D quantity; averaged LCLs (downsampled / poles) cut the cloud of the box's lower-LCL columns; 9.5 km fetch
  limit had too little margin. Now: all liquid below the LCL is moved, a base only where nothing is left below, a
  separate RG32F base texture with the layer's extinction scale, the box's lowest base, 12 km; file format 4.
- Task 7 fix round 2 (2026-10-27): a boundary-layer cloud wholly under the LCL gave its water to ANY lowest significant
  run above it within the boundary layer (a deck at 2 km turned dense, the low cloud vanished); the cut scale of a base
  a few metres under its layer's top was unbounded. Now: merge only into a run starting within LCL + the lowest run's
  depth, otherwise lift the lowest run to the LCL; bases closer than 20 m under the layer top are dropped; file format 5.

## Cloud march sub-samples, Task 5c (replaced in fix round 1, 2026-10-27)
Superseded text of `docs/subsystems/weather-clouds.md` (march, validation, open point):

> **Sub-samples**
> (static clouds Task 5c, `CLOUD_STRUCTURE_STEP_M` 125 m [E] = the shape texture's finest texel, at most
> `CLOUD_MAX_SUBSTEPS` 4 [E]): a step longer than 125 m (the VR fine step is 2.5 % of the distance, 500 m at 20
> km) takes the mean extinction of ceil(step / 125 m) sub-samples at the midpoints of equal sub-steps, in the
> step's field cell (the field is smooth over a step), the detail band-limited at the sub-step; one light sample
> per step at the sub-samples' extinction-weighted centre (also the step's distance for the air light). A
> silhouette is bisected from the first sub-sample over the threshold (between it and the one before); the thin
> cloud of the sub-samples before it ends the step there with its optical depth kept. One point per step hit or
> skipped thin and broken cloud on long oblique paths with each ray's fixed step phase - static salt-and-pepper
> at the march pixel (the user's t7_cape view) that the jittered march's history had averaged away, gone with 8 x
> the steps; and the sub-threshold thin cloud before every entry was dropped. Not for the grid-mean debug outputs
> (1 / 2 / 6) nor the jittered march.

> - GPU `game/tests/cloud_grain_check.gd` (static clouds Task 5c; the main scene, analog weather 2024-06-15 21 UTC, VR
>   preset): the grain - the mean |Laplacian| of the cloud image's premultiplied luminance at a spacing of one march
>   pixel over the pixels with alpha > 0.1 (and their four neighbours), relative to their mean luminance - within 25 %
>   [E] of the 8 x steps reference's (768 / 1536), both ways (no salt-and-pepper, no blur): the Cape (28.57 N, 80.65 W,
>   20 deg up) 1.176, the ground at Starbase 1.096, the deck from 9 km 1.104 (before the sub-samples 1.704 - RED -, 1.196,
>   1.179; the cloud pixels then 458 k / 626 k of the reference's on the ground: thin cloud was missed); the cloud image
>   of the next frame bit-identical in each view. `render_check --repeat_identical` at the Cape: the frames differ in
>   the sea below the horizon only (y >= 825; the base too) - the check prints the differing pixels' bounding box.
>   `cloud_far_check`: the broken layer's volume-vs-far block 18.10 % (was 19.36 %; Task 8), the decks unchanged (a
>   first version that compared the float32 b - a_this in the step-weight test sprayed bright speckles near the limb from
>   8 600 km: deck tau 2 at 15 %).

> - **March sub-samples (Task 5c)**: the VR proxy's cloud pass +49 / +55 / +62 % (descent / pan low / pan high medians,
>   `vr-performance.md`) for the shapes' lookups at the sub-samples - over the budget like the base (Task 10). Left: some
>   march-pixel blocks in dark thin layers (one light sample per step; grain 1.18 x the reference's at the Cape), steps
>   beyond 500 m sampled at a quarter of the step, and cirrus, whose vertical shape texel (2 km / 128 = 16 m) is finer
>   than the 125 m interval.

- Why replaced (review of 8f85b31): the grain check asserted only the luminance grain - the deck's alpha grain was 1.30
  x the reference's, its alpha -8.5 % and cover -5.3 %, the ground -3.9 % / -3.4 %. Causes: the 192-step budget ran
  out in broken layers' cloudy regions before the far cloud (7-16 % of the deck view's rays); the step's one field
  cell for all sub-samples (cirrus sheets -4 % alpha); 125 m sub-steps (half a texel) and at most 4. The detail band
  limit jumped where a step crossed 125 m (sub-step halved). Now: max steps 4 x the coarse, a cell per sub-sample,
  a per-type quarter-texel structure step, up to 8 sub-samples, the band limit continuous, the thin cloud before an
  entry carried into the next step (also for single-sample steps).

## Far field vs the volume at the scaled-space limit (superseded by the near / far hand-over, static clouds Task 8, 2026-10-27)
Superseded text of `docs/subsystems/weather-clouds.md` (validation, open points); the march then ran to the horizon
and drew every cloud with the shapes up to 1.5e7 m:

> differing pixels' bounding box. `cloud_far_check` after fix round 1: the deck tau 10 volume-vs-far block 11.20 %
> (limit 10 %; cb65707 9.88 %, the same march at 8 x the steps 11.97 %: the converged volume is brighter than the far
> field, open for Task 8 with the broken layer's 17.50 %).

> five clear cells (no cloud within 1 deg) unchanged (ratio 1.0000). The game's volume (the shapes, prefiltered)
> against the far field, tolerance 10 % (ruling 2026-10-26: the shapes' 3D light is outside a column model): deck
> tau 10 within 6.4 %, tau 2 within 4.3 %, broken cf 0.4 (21 x 21 block means) within 7.3 % (Task 11: every placement
> resets the cloud history and the frame counter and draws exactly 64 frames, so the temporal mean is the same number of
> frames in every run; the earlier runs, timing-dependent, gave 7.3 / 5.2 / 7.5-8.5 %; before the prefilter 32 %).

> - Far field against the volume at the hand-over (Task 10, `cloud_far_check.gd`): with the game's volume (the shapes)
> a deck of tau 10 differs by up to 6.4 % (the volume brighter 25-45 deg from the sub-camera point, Sun ~35 deg up,
> view ~68 deg off the vertical), tau 2 by up to 4.3 %, a broken layer (cf 0.4) by up to 7.3 % (the volume brighter
> at the centre by 5-8 %); with the volume on the grid mean the decks agree within 2.95 % / 3.89 %: the step is the shapes' sub-grid structure (the 3D entry cosine
> under a varying Sun depth only brightens: mu_e is clamped at mu0 from below; the soft edges' thin fringes), which a
> column model cannot carry. Accepted with the tolerance 10 % (ruling 2026-10-26): a faint step when crossing
> 1.5e7 m. A thin layer seen from afar cuts many horizontal slices of the 32 km shape tile, whose cover is not exactly
> cf: faint rings around the sub-camera point remain (the shapes' own, not aliasing). The sub-pixel offsets leave a
> fine temporal grain on broken cloud beyond f_s until the temporal mean converges (still camera: frames; a moving
> one: the floor weight's ~23 % of one frame's noise) - with the jittered march only; the deterministic one reads the
> footprint's centre.

## Far tier's fade, column model, Task 2 light ruling (superseded by static clouds Task 8 fix round 1, 2026-10-27)
Superseded text of `docs/subsystems/weather-clouds.md` (model, open points). The fade then took the footprint projected
onto the vertical (x |cos|): grazing views kept the shapes at multi-km footprints (the limb from orbit speckled); the
column model drew the 0.25 deg cells bilinear (a lattice of soft squares from orbit) as an even cover, ~11-20 % darker
than the converged volume for broken cumulus; the dotted horizon row was the upsample's.

>   `cloud_handover_near_m` / `_far_m`, both presets 20 / 34 km [E]): the march ends at 34 km - inside the light cache's
>   full weight (ruling of Task 2: the light beyond is never taken from the per-sample light march) and below the
>   footprints that need the shape prefilter

>   - where the pixel footprint at the ray's crossing of 2 km (`cloud_far_reference_m`; its lowest point when it stays
>     above: continuous over the pixels), projected onto the vertical (x |cos| of the view there), exceeds 0.5 km [E],
>     the far tier fades by 1 km to the far field's column model (`cloud_far_slab`: the far field's slab placement along
>     the ray, from above or from below, a crossing in front of the band taken at its start; `cloud_far_column`: the
>     beam cover C_v, the plane-parallel slab), weighted by w at the slab: seen from above with ~1 km footprints the
>     32 km shape tile showed as a lattice over a uniform layer and the mips (1 km texels at most) beat into streaks; at
>     grazing views the overlap merges the shapes into the horizon's band, which the column model drew faint (the
>     unprojected footprint faded the band from 20 km to the column). From 420 km every pixel is the far tier's (the
>     march segment is empty): at VR resolution (2.3 km) the column model below, the shapes along the limb.

>   Approximations: no light between columns (no lit cloud sides, no shadows on neighbouring columns), the light taken
>   at one point,

>   broken cf 0.4 (21 x 21 block means) within 2.60 % at the six points off the sub-camera point; there the ground alone

> - Near / far hand-over (static clouds Task 8): the far tier's light is the column's beyond 1.5 km of the light path
>   (no light cache beyond the march: no far neighbours' shadows past 1.5 km); its shapes are band-limited at the pixel
>   footprint (no detail beyond ~125 m footprints, as the march); at large footprints (VR beyond ~90 km, low orbit) it is
>   the far field's column model (plane-parallel, the 0.25 deg cells as soft patches from orbit). In the band both passes
>   run (the cost of the overlap); in all the cost is about the march's to the horizon before (VR proxy,
>   `vr-performance.md`), as the far pass takes the same steps, with a cheaper light. The scaled-space limit: the
>   ground's sea glint steps there (terrain vs scaled-space Earth, weather off), seen through broken cloud. The dotted line at the far sea horizon in the deck and 20 km views (pre-existing,
>   unchanged): the checkerboard of the march pixels' nearest / farthest scene depth - on the horizon rows alternate
>   march pixels end at the sea (~340 km from 9 km) or run into the sky, and only the latter hold the cloud band beyond
>   the sea horizon, so the upsample alternates along the row (period 2 march pixels).

> - Far field: plane-parallel independent columns (no 3D radiative effects: no lit cloud sides at a low Sun, no shadows
>   cast on neighbouring columns or the ground beside them); the field's 0.25 deg grid shows as soft cells near the
>   hand-over (~4 pixels per cell at 1.5e7 m).

## The Sun's disk brightness (superseded by static clouds Task 9)

`sun.gdshader` drew the disk at `intensity` 30 (scene value, "the disc's display brightness"), 16 x a white Lambertian
under full Sun and 100 x the lit ground - 1500 x too dim against the physical 1.5e5 x. Seen through a cloud the volume's
own forward-scattered light (HG part g 0.995, ~200 x the disk at tau 2 at the Sun's position, in the same scene units)
drowned it: no disk through any slab of tau >= 2. The march also left exp(-tau) ~ 5e-4 of the background behind a thick
cloud (the scaled criterion's early exit), invisible next to a disk of 30, 120 x a lit ground next to the physical one.

## Far tier's sub-grid cover and the 3D Sun light's validation (superseded by static clouds Task 8 fix round 2, 2026-10-27)
The sub-grid cover scaled the even cover by the share of taps above the prefilter threshold of the column's largest
fraction over that fraction, clamped at 1; where whole footprints were covered the clamp lost 6-15 % of the mean cover
(alpha column / converged volume 0.85-0.94 at single sites), and the 3D Sun light's constants compensated part of it.
The 3D light was not switchable and validated only on the scenes it was fitted to. Text then:

  - the shapes' 3D Sun light (static clouds Task 8 fix round 1; `CLOUD_FAR_ENTRY_TURN` 0.15, `CLOUD_FAR_FLANK_GAIN`
    0.4 [E]): the volume lights a sample whose Sun path is shorter than its depth below the column's top as through a
    surface turned toward the Sun (`cloud_entry_cosine`) - a deck's domes under a low Sun were 12-14 % brighter in the
    converged volume than the slab, broken cumulus (cf 0.4) under a high Sun 15-20 % (their lit flanks); the slab's
    normal for the Sun's light is turned 0.15 of the way from mu0 to 1 and the Sun's light gains 1 + 0.4 (1 - f) for
    the column's largest fraction f (fitted to cloud_far_check sections 3 and 6: 0.3 / 0.15 left the broken layer
    10-11 % dark);
    lattice of soft squares; the slab heights stay bilinear); the even cover C_v (the beam cover of the largest
    fraction at the view's cosine) times the sub-grid cover (`cloud_far_cover_ratio`, spec decision 4's deterministic
    footprint average): the shapes' prefiltered cover - the share of 4 x 8 fixed taps over the pixel's footprint x 2
    across the view and / mu along it (at most 32 km) above the prefilter threshold of the column's largest fraction,
    at the mip of the taps' spacing - over that fraction (1 on average: the cover and the water stay the grid's; 1 for
    a deck), half of it from a second, unaligned reading of the shape (Earth-fixed, turned 37 deg about (1, 2, 3),
    x 1.37) faded in from 1 to 3 km of footprint (the 32 km tile's period showed as a regular lattice over the hundreds
    of km of an orbit view), and none beyond 8-16 km footprints (the scaled-space far field's even cover); the cover
    clamped to 1, the in-cloud depth the even cover's; the radiance the plane-parallel slab's with the shapes' 3D Sun
    light (below, far field).
Validation text then (cloud_far_check section 6, one site each):

  of the covered pixels [E]). Section 6 (`--only=column`, fix round 1): the broken layer and the deck from 420 km at
  nadir / 45 deg, mean radiance of the central square, tolerance 10 %: the column model vs the converged volume
  0.927 / 0.940 (broken; before fix round 1 0.891 / 0.799) and 0.988 / 1.008 (deck), vs the far tier's shapes (the
  brightness across the fade) 0.973 / 0.993 and 0.992 / 1.004; no speckle

## The Sun's disk through cloud, alpha in the cloud image (superseded by static clouds Task 9 fix round 1, 2026-10-27)

The bullet as it stood after Task 9 (the cloud image stored alpha; the composite blended premultiplied alpha; the disk
clamp per channel; numbers: the disk's mean 6.9e4 was given as its centre):

- **The Sun's disk through cloud** (static clouds Task 9; `sun.gdshader`, `World.sun_disc_scene_value`,
  `cloud_pass.gdshaderinc` `CLOUD_SUN_*`, `CLOUD_ALPHA_SNAP`): the disk is drawn at its physical radiance on the scene
  scale (a white Lambertian in full Sun = `Atmosphere.DAY_EXPOSURE` 1.5 = E / pi, so the disk is E / solid angle =
  1.5 x (AU / R_sun)^2 = 6.9e4 over the limb law's mean 0.8; it was 30, 100 x a lit ground instead of 1.5e5 x, under
  which the clouds' own forward peak - the phase function's HG part, g 0.995 - outshone the disk 200 x at tau 2 and the
  disk vanished under any cloud). The composite is linear in the scene behind the clouds, so the disk through cloud is
  disk x (1 - alpha) = disk x T_bg of the march. The march continues past the scaled criterion (which stops at
  exp(-tau) ~ 5e-4) on rays within `CLOUD_SUN_RAY_ANGLE` 0.014 rad of the Sun down to T_bg 1e-9 (coverage-only
  sub-rays too), and the pass snaps an alpha within 2^-12 of 1 to 1 (the half-float store of the cloud image rounds
  toward zero: 1 - 1e-5 came out as 1 - 2^-11, a leak of 29 on the scene scale behind any thick cloud). Limit: the
  image's alpha is a half float, so a direct beam below T ~ 2.4e-4 (slant tau 8.3, vertical 6.8 at mu 0.82) is
  lost (the disk vanishes there, where it would still outshine a lit ground up to slant tau ~11) and above it is
  quantised to 4.9e-4 steps (tau 5: 8 %). The shapes' transmittance along the Sun's ray is the Jensen mean of
  exp(-tau) over the slab's low shape octaves: 15 % above the grid mean's at tau 2, 40 % at tau 5.

Diagnostics bullet then:

- `cloud_check.gd` with `STARSIM_SUN_ONLY=1`: section 13 alone (the Sun's disk through slabs of tau 2 / 5 / 10 / 20 at
  55 deg: the disk's part = the render with the disk less without it, within 5 % (or 4.9e-4 of the clear disk) of the
  march's shapes transmittance, the march's slant tau within 8 % of the core's, brighter than the terrain while
  expected to, the aureole 303 x the radiance 60 deg off; near march and far tier alone). `render_check.gd
  --sun_through_deck=<tau> --lat= --lon=` renders the view (`.superpowers/sdd/.../img/t9_tau*.png`).

Measured then (fix round 1, at 11283a0): the premultiplied blend rounds its factor 1 - alpha to the half-float target
too, so even with alpha computed exactly the disk stayed at 2^-11 = 4.9e-4 of itself behind every deck of slant tau
7.6+ (disk/clear 4.89e-4 at vertical tau 7, 8, 9, 10); the per-channel clamp turned the 55 deg disk (81 500) into
(60 000, 60 000, 51 456) instead of the hue (60 000, 47 546, 37 918).

## Cloud shadows on the air, first version (superseded by static clouds Task 9b fix round 1, 2026-10-27)

The Model bullet as of 34661fa (the Mie fade per point, a clamp per segment, the ray to 20 km, the upsample's floor
from every tap, the outline pass's loss of this pixel for the source's, the grain identity with the air shadow off):

- **Cloud shadows on the air** (static clouds Task 9b, ruling 2026-10-27 "Wolkenschatten auf die Luft";
  `cloud_pass.gdshaderinc` `cloud_air_shadow_loss`, fidelity flag `CloudLayer.air_shadow` / global `cloud_air_shadow`,
  render_check `--no_air_shadow`). The tables' air light is the clear sky's; under and beside clouds the Sun's direct
  beam is cut. Of the air light only the single scattering of the direct beam is weighted by the beam's cloud
  transmittance T_c at the scattering point; the multiple scattering (the sky's diffuse light, the deck's glow) stays.
  The single Rayleigh part comes from the new atmosphere table `atmosphere_single_rayleigh` (core
  `AtmosphereTables::single_rayleigh`, `atmosphere-scattering.md`), the single Mie from `atmosphere_single_mie`. The
  march kernel integrates per pixel over the ray below 20 km (to the scene or the shell's exit) in 32 segments at
  (i / n)^2 [E], Bruneton's difference form per segment (unweighted it telescopes to the plain single-scattering
  lookup), T_c = the mean of 3 fixed points per segment [E] from the cloud shadow maps (`cloud_shadow_transmittance`:
  near map's shapes within 9-11 km, far map's grid mean to 120-140 km, slanted column beyond; inside the near range
  the near map alone) - one midpoint per segment aliased the near map's shadows into regular stripes across the sky,
  3 x 32 points look as 96 segments. The segment's part in front of the clouds' weighted distance is seen directly, the
  rest through the background's T (linear split: a step at the midpoint stair-stepped the loss along the clouds'
  distance contours). The pass's colour carries -L (the march clamps to +-65000, the upsample's undershoot floor is the
  lowest tap's colour instead of 0, the composite's additive pass adds it). Static (fixed points, `repeat_identical`),
  world space (both VR eyes), the eclipse's unocculted fraction applied. Weather off or no cloud in the Sun's way: L = 0
  exactly (cloud_check 14: 0.0000 %).
  - Backlit cumulus (cloud_check 14: cf 0.5, in-cloud tau 50, 1-2 km, Sun 15 deg up, HDR medians; before -> after):
    2-6 deg from the Sun sky 2.59 -> 1.75, body 2.27 -> 0.60 (body / sky 0.88 -> 0.34), rim p90 / body 1.77 -> 3.95;
    6-10 deg sky 1.86 -> 1.40, body 1.72 -> 0.70 (0.50), rim / body 1.63 -> 2.77; 10-15 deg sky 1.17 -> 0.80, body
    1.12 -> 0.57 (0.71), rim / body 1.87 -> 2.76. The acceptance (cloud-visual-reference.md (3)) holds. Under a cf 0.5
    deck at a 15-deg Sun the beam crosses ~3.7 km of the cloud layer: the air below it lies ~70 % in shadow (beam cover
    0.70), so the aureole of the air in front of the body nearly vanishes (the measurement round's 24-piece light-march
    diagnostic, -21..-24 %, had left the sky unshadowed and used the per-sample light march).
  - Crepuscular rays (cloud_check 14: cf 0.4 deck, Sun 4 deg, the sky's on / off ratio per column along a 12-deg band):
    toward the Sun p10 / p50 / p90 0.41 / 0.59 / 0.76 (modulation 0.58), away 0.92 / 0.93 / 0.94 (0.019: the
    anti-crepuscular side is lit mostly by single Rayleigh and the aerosol's weak backscatter).
  - Cost (VR proxy, whole frame, `vr-performance.md`): +1.6-1.7 ms backlit cumulus, +4.2-4.5 ms the 9 km deck view.
  - The bit-identity of the cloud image (cloud_grain_check) is checked with the air shadow off: the loss is integrated
    to the scene, so the sea's waves move it (~2500 Cape sea pixels by < 3e-5).

Open points then: "The measurement round's "+21..+39 % at view zenith 60 forward" was this fallback, not the table."
(the per-sample light march for thick slabs seen from below). Review r1 (task-9b-findings-r1.md): the per-point Mie fade
removed forward Mie light the scene never added at a setting Sun (segment sum / scene single scattering 1.54 at a
0.3-deg Sun); the per-segment clamp biased the sum up to +8 % at p95 near the horizon; the crepuscular metric
((p90 - p10) / p50 of the on / off ratio) passed a ray-free control (0.23); the outline pass's error up to 7.4 %
(32 % of the outline pixels over 1 %); the floor from every tap let a body pixel next to the shadowed sky go to
-0.60 (backlit) / -0.67 (rays); the 20 km clip sampled the backlit body 2 % brighter (rim / body 3.93-3.95 against
4.01 converged).

## Task 9b fix round 2 remnants (superseded 2026-10-27)
- Cost note: the backlit cumulus view cost +1.6-1.9 ms before the clip to the cloud top (+1.0-1.1 ms after).
- Ray-free control: its undetrended spread 0.23 passed the earlier (undetrended) metric.
- Rim / body 4.01 at 2-6 deg from the Sun was recorded as "just above the acceptance's 4 (cloud-visual-reference.md
  (3))"; the band's bound is now 2-5 x [E] (photographic range, not a physical bound).

## Before static clouds Task 9c (superseded 2026-10-27: cover scaled to the total cloud cover, cumulus water from an adiabatic parcel)
weather-clouds.md, Validation (real-date plausibility):

- Real-date plausibility (final review I4, `test_weather_snapshot.cpp` "Real-date columns ...", fixture
  `era5_plausibility.toml` by `tools/validation/gen_weather_fixtures.py`: ERA5 columns of 2024-04-08 18 UTC and
  2025-08-15 18 UTC at Starbase, the Cape, the North Atlantic, the warm pool and the most overcast cell near Starbase,
  with the store's own `total_cloud_cover` and `cloud_base_height`). Cloud base (lowest layer with fraction > 1 % and
  condensate, ERA5's definition) within one layer plus the spacing of the pressure levels around ERA5's cbh (unchanged by
  the bases at the LCL; the fixture now carries the 2 m temperature and dew point, and a boundary-layer base lies no more
  than one layer below the column's LCL: all ten columns pass). Cover: the
  snapshot is systematically LOWER than ERA5's tcc, by 0.00-0.36 (mean 0.14) with the same maximum-random overlap (the
  renderer's maximum overlap is lower still, 0.00-0.37): the 37 pressure levels hold the model levels' fractions
  interpolated, so clouds that overlap in the model at different horizontal places no longer add up. The test asserts
  never more than tcc + 0.05, at most 0.40 below, mean deficit < 0.2 (a limit of the data choice, `known-issues.md`).

weather-service.md:

  ERA5 hour = 8 timed files (the 6 cloud / temperature / pressure / vapour fields + `2m_temperature`,
  `2m_dewpoint_temperature`) + the static `era5_geopotential_at_surface.blosc` (downloaded once, kept in `cache/raw`).
  GFS = 113 byte ranges (5 variables x 22 levels, HGT surface, TMP and DPT "2 m above ground").
  (`raise_liquid_cloud_to_lcl`, `weather-clouds.md`). Stored in the `.wthr` file: format version 5 = ground block, then a
  float32 LCL block (+4 MB at 0.25 deg), then the texels (version 2 had the unknown level as 0 and no redistribution,
  version 3 moved only the lowest liquid run and averaged the downsampled levels).
- **Old files heal**: `read_weather_snapshot` refuses version-1, -2 and -3 files (and anything else of another version). A

known-issues.md:

- Weather and clouds (M8): the snapshot's column cloud cover is lower than ERA5's total cloud cover (0.00-0.36, mean
  0.14 in the real-date check; the 37 pressure levels lose the overlap of the model levels), so multi-layer skies are
  thinner than the reanalysis says; the limits (light between columns, shadow darkness under broken cloud, plumes always draw

## Before static clouds Task 9c fix round 1 (superseded 2026-10-27: height-dependent f_ad, in-cloud bound, trace layers kept, weighted floor)
weather-clouds.md, Cover from the total cloud cover (first 9c state, file format 6):

- f_ad = `cumulus_adiabatic_fraction` 0.4 [E], constant with height; the floor integrated f_ad Gamma_l (z - z_b)^2 / 2.
  Grid-mean liquid water path of the four analog hours 50.2 -> 140.4 g/m^2 (+180 %, added 86-111 g/m^2 per hour), thin
  area 11.8 %, Cu tau p10 / p50 / p90 6.2 / 30.6 / 144, in-cloud LWC p99 1.054 g/m^3; f_ad 0.3 / 0.5: 111 / 168 g/m^2.
- The floor ratio was computed over the whole run path (deck layers too), so a column with a deck below the cumulus
  ended below the floor (7.5 of the 11.8 points of thin area).
- `scale_cloud_cover_to_total` scaled every layer with condensate, traces too (a 0.75 % layer 830 m under the Cape 2025
  base crossed ERA5's 1 % threshold), and no in-cloud bound followed the scaling down: tcc ~ 0 columns (1.1 % at
  0.25 deg) put each cloud layer on cloud_min_cover with its water kept (dense specks). The plausibility cloud-base
  check therefore ran on the unscaled fractions.

## Before the Task 10 follow-up (superseded 2026-10-27: 6 sub-samples; the step weight 0.3 of 954a4c8 was reverted to 0.15 because it failed cloud_far_check's hand-over fade cases at 1.1085 / 1.1024 of 10 %)
weather-clouds.md:

- Reference values at Task 10 (6b9149e, RTX 4090; sdd `t10_checks_e1/`): `cloud_check.gd` PASS, 198 samples, 312 s for
  the whole run; `cloud_far_check.gd`: slab radiance vs Monte Carlo worst 0.34 % (72 geometries), deck tau 10 inside /
  outside 2.68 % (grid mean), 0.22 % (far tier), 8.16 % (converged shapes, limit 10 %), deck tau 2 3.17 / 0.60 / 1.54 %,
  broken cf 0.4 2.28 %, far field / converged volume 0.925 and 0.951 (limit 10 %), far tier / march 0.990 (both
  cases), cross-fade worst 2.48 % / 0.38 %, hand-over step 2.52 % / 0.58 % (limits 3 %), from 420 km 0 pixels differ;
  only the Moon view fails (cache file `era5_1089330.wthr` missing, known); `cloud_grain_check.gd` PASS (grain ratio
  to the 8 x reference 1.00-1.07, cover and mean alpha within 0.44 %, RMS 0.025-0.048); stereo, layer, smoke and
  repeat_identical (ground, deck) green.

- **March sub-samples (Task 5c, fix round 1)**: the VR proxy's cloud pass +97 / +75 / +72 % (descent / pan low / pan
  high medians over cb65707, `vr-performance.md`) for the shapes' and field's lookups at up to 8 sub-samples and the
  doubled step budget - over the budget like the base (Task 10); at most 4 sub-samples cost +39 / +27 / +25 % and
  still pass the grain check (cover / alpha within 1.8 %, luminance 2.2 %, Cape grain 1.10). One light sample per step.

- Quality-preset note: the march ran at up to 8 sub-samples per step (`CLOUD_MAX_SUBSTEPS` 8, longer steps 1/8 of the step) and the step-weight limit was 0.15.

Static clouds final review (2026-10-27), superseded cloud_check section 8 acceptance text (the old interior 10 % / silhouette 20 % acceptance, with the then-failing cascade 1 case):

> frames): a tau-10 slab (debug 2) worst 0.07-0.71 % in every cascade (tolerance 5 %); a cf 0.4 broken layer (debug
>   7, the shapes): interior pixels worst <= 10 % and the 99th percentile <= 10 %, silhouette pixels (alpha < 0.3 within
>   one march pixel, debug 4) worst <= 20 %. Worst pixel at Sun 45 / 20 deg: cascade 0 2.78 / 6.71 %, 1 4.81 / 16.00 %
>   (FAIL: interior, 9 interior pixels over 10 %; p99 6.81 %), 2 6.00 / 11.45 %, 3 8.13 / 9.52 %, 4 9.95 / 12.08 %
>   (p99 1.6-6.8 %, mean errors 0.16-0.95 %).

Static clouds final review (2026-10-27), superseded Open points blocks "VR (2026-10-26)" and "Performance gate (Task 11)" (they call the VR preset 48 / 96 at divisor 4; it is 96 / 384 at divisor 3 since Tasks 3 / 5c / 10):

- **VR (2026-10-26, `vr-performance.md`)**: the 1440p gate below underestimated the headset ~10x (20 MP per frame at
  the user's Pimax FSR 1.5). Measured at the VR pixel count (`terrain_perf --render_scale=1.65 --gpu_passes`): the cloud
  pass 11-13 ms at divisor 2, 1.4-3.2 ms at 4 (the VR preset now), shadow 0.55 ms, the clouds' share of the opaque pass
  0.2-1 ms. Ground / deck / 20 km views unchanged at divisor 4, orbit cumulus fields step coarser; their hard cell edges
  exist at divisor 2 too (not the upsample's depth weight: open point). VR run: GPU 26-66 -> 13-17 ms, but in the
  headset the clouds look grainy and blurred differently per eye (user 2026-10-26): each eye marches with its own
  jitter and history. Since 1ba1be7 VR marches and accumulates once in a centre view covering both eyes
  (`CloudCentreView`, divisor 3), each eye reprojects into it at the cloud's distance (exact disparity,
  `cloud_vr_stereo_test`) and gathers a 4 x 4 tent (soft alpha edges, the same in both eyes). VR run
  (`perf_20261026_085348.csv`): GPU 15.2 / 18.1 / 19.5 / 15.6 / 14.4 ms pad / climb / decks / 13-60 / > 60 km, 1-2.3 ms
  more than two eyes at divisor 4; user: kept. Open: the tent's width is fixed in screen pixels, so far clouds get the
  same blur as near ones (looks unnatural, user 2026-10-26).
- **Performance gate (Task 11)**: `terrain_perf.gd --weather=<abs path .wthr> --cloud_quality=vr` (1440p desktop proxy of
  the 90 Hz VR frame; the gate it prints: p99 <= 11 ms with clouds / 8 ms without, max <= 16 ms; 2024-04-08 18:42 eclipse
  epoch, ERA5 18:00 pinned, three runs with and three without in one session, alternating). The machine's baseline moved
  between the notes: a clean one measured without clouds p50 4.5 / p99 7.6 ms (Task 9), the session of Task 11
  p50 8.25-8.29 / p99 12.58-12.63 / max 16.3-17.1 ms (three runs; the Task 9 scripts at 21a833c on the same machine and
  the same day gave p99 12.47, so it is the machine, not the code; no VR headset runtime was running (Pimax client / runtime and DesktopXR bridge stopped), the
  Virtual Desktop service and its virtual display driver could not be stopped without administrator rights). In that
  state the absolute gate p99 <= 11 ms cannot be met even without clouds and was not met. What the clouds add, measured
  against the baseline of the same session, is the budget the gate is derived from (clouds <= +3 ms p99 over the
  baseline):
  - preset 64 / 128 steps (before Task 11): p50 +1.4 ms, p99 +3.5-3.7 ms (16.14 / 16.07 / 16.23 vs 12.51 / 12.59 /
    12.53), max 20.9-22.4 ms - over the budget.
  - tuning runs (single runs, VR preset, `terrain_perf.gd --cloud_tune=...`): steps 48 / max 96: p99 14.90 (-1.2 ms);
    light near 5 -> 4: 15.62 (-0.5); fine step 2.5 -> 3.5 %: 15.72 (-0.4); all three: 14.47 (-1.7). Not touched: the
    detail levels (they are the approved shapes' fine structure), the resolution divisor, the light march's far part.
  - **preset 48 / 96 (the VR preset now)**: with clouds p50 9.51 / 9.51 / 9.51, p99 14.96 / 15.00 / 14.86, max 18.86 /
    19.04 / 19.61 ms against 8.29 / 8.25 / 8.27, 12.63 / 12.59 / 12.58, 16.91 / 16.32 / 17.07 ms without: clouds +1.25 ms
    p50, +2.4 ms p99 (within the +3 ms budget, 3 of 3), +2-3 ms max. By phase (third run): descent p99 15.90 vs 11.91
    ms (the worst, the decks seen from 8 km: GPU 14-15 ms per frame), pan low 13.45 vs 12.62, pan high 13.27 vs 12.23,
    Cape 13.99 vs 13.29. VRAM +430 MB (2886 vs 2456 MB at the Cape: two snapshots of field, ice and column textures).
    Image A/B of 64 / 128 vs 48 / 96 (`render_check --cloud_quality=vr --fixed_exposure --deterministic=64`, 1600 x 900,
    four views: cumulus layer from 2.5 km, sunset bases, the Cape shadows from 3.5 km, 420 km orbit over a front): mean
    absolute difference 0.3-0.7 of 255, 1-2.5 % of the pixels differ by more than 8 levels (cloud edges, p99 5-13), none
    visible by eye. The checks (`cloud_check`, `cloud_far_check`) run the desktop preset and do not cover the VR one.
  - The absolute gate (p99 <= 11, max <= 16 ms with clouds) therefore needs the machine's clean state (baseline p99
    ~7.6 ms): expected ~10.0 ms p99 with the +2.4 ms of the clouds. To be re-run by the user on a quiet machine.
  - Pre-warm (Task 11): `earth_clouds.gdshader` is drawn once on a sub-pixel quad at load, and the shape-prefilter march
    kernel is dispatched once (one group) in the first cloud frame; the first switch to the scaled-space far field used
    to cost up to 5.4 ms in one frame. Sporadic draw / present stalls of 16-46 ms happen with and without clouds (one
    46 ms frame in a baseline run of Task 11), i.e. they are the platform's.
  - Levers measured earlier (superseded numbers in `docs/history/weather-clouds.md`): the detail levels ~2 ms in the
    descent, the step refinement ~0.3 ms, the eclipse probes ~0.3 ms (per sample ~10 ms).


Static clouds final review (2026-10-27), weather-service.md file-format version history (moved from the current text):

> Stored in the `.wthr` file: format version 7 = ground block, the float32 LCL block, the float32
>   tcc block (-1 unknown), the float32 added-cumulus-water block and the float32 dropped-condensate block (kg/m^2,
>   diagnostics; +16 MB at 0.25 deg for the four), then the texels (version 2 had the unknown level as 0 and no redistribution, version 3 moved only the lowest liquid
>   run and averaged the downsampled levels, 4 merged a deck far above, 5 had no tcc and no cumulus water, 6 had a constant f_ad, no in-cloud bound and no dropped-condensate block).

## Cloud view cache before its fill (superseded by cloud view cache Task 3, 2026-10-27)

> **Cloud view cache (in progress, `docs/superpowers/archive/specs/2026-10-27-cloud-view-cache-design.md`)**: pure logic so far -
>   `CloudViewCacheLayout` (cube map, tiles, 3 resolution levels, face size from the eyes' projection) and
>   `CloudViewCacheSchedule` (tile state, staleness, priority pick under a budget); `cloud_view_cache_test.gd`. Not wired
>   into the renderer yet.

## Cloud view cache before its lookup (superseded by cloud view cache Task 4, 2026-10-28)

> - **Cloud view cache (in progress, `docs/superpowers/archive/specs/2026-10-27-cloud-view-cache-design.md`)**:
>   `CloudViewCacheLayout` (cube map, tiles, 3 resolution levels, face size from the eyes' projection),
>   `CloudViewCacheSchedule` (tile state, staleness, priority pick under a budget; `cloud_view_cache_test.gd`) and the
>   fill (Task 3): `CloudViewCache` (`game/world/weather/cloud_view_cache.gd`) holds per level a cube texture of colour + T
>   (RGBA16F) and one of the alpha-weighted cloud distance (R32F) (3456 per face for the VR run: 0.86 GB full level, 1.13
>   GB all three) and fills the texels of a tile list ({tile, level}; one dispatch per level, push constants, the list
>   sorted by level) with the kernel `CLOUD_CACHE_FILL` (`shaders/cloud_view_cache.gdshaderinc`: the march's code with
>   `CLOUD_MARCH` + `CLOUD_CACHE_FILL`, its own images instead of the march's). Each texel's ray (`dir_of`'s mirror in
>   GLSL) starts at the anchor - its offset from the camera in the fill's CloudView `eye_earth` (float64 difference on the
>   CPU) - and ends at the ellipsoid / sea crossing seen from there (else 1e30), never the frame's depth; the texel angle
>   at a face centre (2 / level face texels) is the detail's band limit. The march and the fill shade a ray with the one
>   function `cloud_shade_ray` (`cloud_pass.gdshaderinc`: march, air shadow loss, debug outputs, the stored colour; with
>   `cloud_ray_jitter`, `cloud_stored_color`, `cloud_eclipse_factors`); `cloud_march` and `CloudRay` take the ray's
>   origin (`h_c`: its height). `cloud_ray_from` is origin-aware only in the fill (an anchor kilometres from the camera
>   has its own height and up); the per-frame march keeps the camera's ray at compile time - a run-time branch on the
>   offset let the GPU compiler re-associate the camera's path (4 % of the pixels changed, up to 30 % of the mean colour:
>   the deterministic march's step grain flips with ulp-level changes). CloudEffect compiles `fill` / `fill_prefilter`
>   and serves `request_view_cache_fill(layout, anchor_earth_m, tiles)` in the next frame before the march (checks only:
>   the game's frame still shows the per-frame march; the schedule is not wired yet). Per-frame bound (controller ruling, tested as a maximum, not a median): `mark_light` + `pick` <= 1.0 ms
>   on every frame once level 0 is everywhere, a cold start / jump frame <= 10 ms. So the schedule is amortised: the light
>   rescan is a pass of `LIGHT_SCAN_BLOCKS` blocks per `mark_light` call (persistent cursor, `light_scan_pending()`); an anchor
>   move starts an epoch (O(blocks)), tiles of older epochs are "off" (stale, parallax candidates) and counted exactly per
>   block; `pick` expands at most `MAX_BLOCK_EXPANSIONS` blocks (2 x 64 = 128 tiles) per call, nearest to the view's cone first.

The validation bullet of the fill ended (before Task 4):

>   p99 46 / 7.8 % and T RMS 0.021 / 0.020 printed: a ray against the bilinear mean of the four texel rays around it at
>   the deterministic march's hard edges and step grain (open for Task 4's lookup).

## Cloud view cache before Task 4 fix round 1 (superseded 2026-10-28: plan on a worker, covering march, latency cost model, relative A / B gate)

>   step grain flips with ulp-level changes). Per-frame bound of the schedule (controller ruling, tested as a maximum):
>   `mark_light` + `pick` <= 1.0 ms on every frame once level 0 is everywhere, a cold start / jump frame <= 10 ms. So the
>   schedule is amortised: the light rescan is a pass of `LIGHT_SCAN_BLOCKS` blocks per `mark_light` call (persistent
>   cursor, `light_scan_pending()`); an anchor move starts an epoch (O(blocks)), tiles of older epochs are "off" (stale,
>   parallax candidates) and counted exactly per block; `pick` expands at most `MAX_BLOCK_EXPANSIONS` blocks (2 x 64 =
>   128 tiles) per call, nearest to the view's cone first; `completed_batch` completes a pick's tiles in one call.
>
>     cloud_check (but section 15) and cloud_far_check run the march; a debug output also shows the march). Main thread:
>     `CloudViewCachePlanner` (`cloud_view_cache_planner.gd`, owned by CloudLayer, FrameProfiler section
>     `cloud_view_cache`): the layout from the eyes' finest density (CloudEffect publishes 7 x 7 sample rays per eye and
>     the face size every frame; at most `FACE_TEXELS_MAX` 4096 [E] for the 2 GB cap, re-created beyond a 10 % [E]
>     change, built on a worker), the anchor (the camera at a restart), the light state (Sun, weather weight and a
>     generation that counts the light cache's resets and its first completion after one or a clock jump - tiles filled
>     while it was still filling refresh once: they differed by whole tiles of 10-30 % in the ground view's A / B), up to
>     `MAX_PICKS` 4 picks per frame while the budget lasts (a further one only within `PICK_TIME_US` 400 us of the first),
>     and per tile the shown state the lookup reads (level, what it fades from, fade slot, the fade's start). Restarts
>     (everything back to nothing, level 0 of the view first): `World._view_jumped` (`notify_view_jump`),
>     `SimClock.time_jumped` (`notify_clock_jump`), a pinned snapshot (`show_now`), and - interim until Task 5's
>     re-anchor - the camera `RESTART_DISTANCE_M` 500 m [E] from the anchor. Render thread (`CloudEffect._run_view_cache`):
>     the batch's tiles filled (GPU timestamp `cloud_cache_fill`; a refresh at the same level first copies the tile's shown
>     texels to its fade slot; each entry's nearest cloud distance by `atomicMin`, read back asynchronously into
>     `set_tile_distance`), the shown states uploaded, then per eye `CLOUD_CACHE_LOOKUP` into the pass's result (the
>     composite unchanged; before the first batch the march shows the frame).
>
>     (a refresh: one Sun state per tile, never an abrupt switch; 1024 slots [E], a slot still in use ends that fade). A
>     tile without a level shows no cloud (T = 1; the near shell / coarse march covers it in Task 5). Scene geometry
>
>   - **Budget (`CloudLayer.cache_budget_ms`, PRESETS `cache_budget_ms` / `cache_budget_floor_ms` /
>     `frame_target_ms`)**: desktop 6.0 ms [E] fixed; VR adaptive (user-approved ruling): 22.2 ms - the rest of the
>     frame's GPU time (95th percentile of 90 frames: the root viewport's measured time minus the fill) - 2 ms, between 2
>     and 11.4 ms [E]. Tiles per frame from the budget over the cost per full-level tile (a level costs its texels: 1/16,
>     1/4, 1), the cost from the fills' timestamps paired in order with their tile equivalents - the larger of the last 4
>     and the last 30 fills' ratio; never below one tile. The cost per full tile varies 0.1-1.6 ms with the direction
>     (clear sky / a deck below), so the fill overshoots for a few frames where the picks turn from cheap to costly tiles
>     (desktop proxy: fill median 4.0 ms, p90 7.5, p99 17 ms at the 6 ms budget).
>
>   - **Open (Task 4)**: the A / B p99 bound (5 %) is not met on general views (cloud outlines: two point samplings of the
>     deterministic march's hard edges; numbers under Validation); the schedule's main-thread time during a fill is ~1 ms
>     at its median-high end (desktop: median 0.6, p90 1.0, p99 1.4 ms; the pick alone 0.5-1.3 ms in the level-0 phase);
>     in flight the interim restart every 500 m leaves ~0.2 s without clouds each time (Task 5); near objects in front of
>     shadowed air get the air loss of the whole cached ray scaled by distance only.
>
> - Cloud view cache in the frame (cloud view cache Task 4): `cloud_vr_stereo_test` mirrors the parallax lookup - a
>   cloud at 300 m / 2 km / 20 km seen from each eye (IPD 0.064 m) and from a camera 50 m off the anchor lands within
>   1e-12 eye px (35 px/deg; bound 0.25) when the cached distance is the cloud's in every direction; a flat deck through
>   the point within 1e-8 px from 2 km on (0.61 px at 300 m, printed: inside the near shell's reach). `cloud_layer_test`:
>   a view jump and a clock jump restart the cache (nothing above level 0 shown afterwards); a 1 deg Sun step starts
>   cross-fades (refreshes from their fade slot), none complete in the frame they start. `cloud_view_cache_test`:
>   `completed_batch` equals `completed` per tile over 60 frames. `render_check --shimmer_check` (deck view, cache full,
>   16 frames yawed +-0.3 px): the frame-to-frame change's p99 0.237 vs the march's at divisor 1 0.257 (ratio 0.92,
>   bound 1.10). `render_check --ab_view_cache` (cache full vs the march at divisor 1, desktop 1600 x 900, 2 x 2 blocks,
>   luminance error / max(|march|, 5 % of the mean)): median deck / ground / 20 km / dusk 0.21 / 0.07 / 0.17 / 0.12 %
>   (bound 1 %, met), p99 over the blocks without scene geometry 9.0 / 14.6 / 9.9 / 22.4 % (bound 5 %, NOT met; the
>   whole image 18.1 / 14.8 / 79.0 / 20.3 %), T RMS 0.009 / 0.007 / 0.022 / 0.023. The p99 sits on cloud outlines: two
>   point samplings of the deterministic march's hard edges (the march against itself turned by 0.5 px: p99 43 % on the
>   ground view). Images `screenshots/cloud-view-cache/t4_<view>_ab_r3_{cache,march,ab_error}.png`.

## Superseded by cloud view cache Task 4 fix round 2 (2026-10-28)

> desktop 6.0 ms [E] fixed;

> (desktop deck view: one full-level deck tile up to ~8.4 ms at the 6 ms desktop budget).

> the A / B p99 at 20 km (9.9 %) exceeds that view's calibration (8.2 %; the other three views
>     meet it - Validation); the desktop's fixed 6 ms fill budget is below the latency of a single full-level deck tile
>     (~8.4 ms on the desktop deck view: fill p99 8-9 ms there; only splitting a ray's march across invocations or frames
>     would lower it); the coarse

> against the calibration's 16.5 / 43.0 / 8.2 / 66.1 % -
>   met but at 20 km (**NOT met**: 9.9 > 8.2 %, exit code 1); T RMS 0.009 / 0.007 / 0.022 / 0.007. Images
>   `screenshots/cloud-view-cache/t4_<view>_ab_f1_{cache,march,ab_error}.png`.

## Superseded by cloud view cache Task 5 (2026-10-28: near shell, progressive re-anchor)

> - **Cloud view cache (in progress, `docs/superpowers/archive/specs/2026-10-27-cloud-view-cache-design.md`; Tasks 2-4 done,
> Task 5 near shell / progressive re-anchor and Task 6 VR acceptance open)**:

> `SimClock.time_jumped` (`notify_clock_jump`), a pinned snapshot (`show_now`), and - interim until Task 5's
> re-anchor - the camera `RESTART_DISTANCE_M` 500 m [E] from the anchor; a requested restart hands out a reset batch
> at once (the frame shows the per-frame march until the restarted run's batch: never a tile of the old place or
> instant).

> tile has nothing and fades a first fill in from it (after a start, a jump, a layout re-creation, the interim
> restart; ~20-45 frames, see `vr-performance.md`).

> eye `CLOUD_CACHE_LOOKUP` into the pass's result (the composite unchanged; before the first batch, and with the
> camera farther than the restart distance from the shown anchor, the march alone shows the frame).

> - **The lookup (`cloud_cache_lookup(eye_offset_earth, dir_earth, out cloud_m)`)**: the anchor's direction an eye ray
> reads is the root of r(q(s)) = |o + s dir| along the ray (o = eye - anchor, r the cached distance): two fixed-point
> steps, then secant steps while both ends saw cloud, `CACHE_PARALLAX_ITERATIONS` 3 [E] (fixed-point steps alone left
> 25 eye px of a deck at 300 m seen 50 m off the anchor).

> tile without a level shows the covering march's result (T = 1 when not covering; `cloud_cache_sample` returns the
> colour without the fade from nothing and its weight, `cloud_cache_lookup` mixes it with T = 1).

> - **Open (Task 4)**: the coarse levels show slightly less cloud than the full one (deck view: coverage 1.9 % below
> the march's for ~2 s after a restart, inside the 0.02 bound); in flight the interim restart every 500 m means the
> covering march (slow frames, ~45 ms in VR) for ~1 s each time (Task 5's progressive re-anchor removes it); near
> objects in front of shadowed air get the air loss of the whole cached ray scaled by distance only; the lookup's
> parallax is GPU-tested only at the anchor (the GDScript mirror covers off-anchor eyes).

## Superseded by cloud view cache Task 5 fix round 1 (2026-10-28: one tile / anchor rule, eviction by parallax, the near shell in the budget)

>     asynchronously), the shown states uploaded, while covering the march (stamped `cloud_cache_cover`), then per eye
>     `CLOUD_CACHE_LOOKUP` into the pass's result

>     anchor). Each tile is reprojected from the anchor it was rendered from (Task 5): the lookup from the current anchor
>     first (the ring's offsets from the camera, float64 differences, in a buffer); where it lands on a tile of another
>     anchor, again from that one (a deck at R_near from an anchor 200 m off: < 0.011 eye px). The colour: the tile's finest shown level, bilinear in the
>     cube,

>     (95th percentile of 90 frames; a frame's rest = the span of its captured timestamps minus its own fill and covering
>     march, from the same frame's stamps) - 2 ms, between 2 and 11.4 ms [E].

> [E], from the parallax error: a deck at R_near lands within 1e-6 eye px from the re-anchor's 100 m and within 0.011
>     px from an older anchor 200-300 m off - `cloud_vr_stereo_test`).

>     not the slots') within R_near x (1 + `ANCHOR_EVICT_FRACTION`) + `NEAR_SHELL_MARGIN_M` 100 m [E]; the render thread
>     runs the near shell when it is within R_near + the farthest shown anchor's offset + the margin
>     (`CacheMode.NEAR_SHELL`)

>     sphere of R_near around the anchor of the cache tile it meets there (`cloud_near_shell_end`: the tile states and the
>     anchor ring bound at 8 / 9, push constants; a tile without a level: the whole ray, covering)

>     schedule refills it from the new one (its parallax and staleness classes, cross-faded from the fade slot). Tiles
>     whose anchor the camera left more than `ANCHOR_EVICT_FRACTION` 0.1 [E] x R_near = 200 m behind (and those of a ring
>     entry about to be reused) are dropped (`CloudViewCacheSchedule.forget`; the covering march shows them in view). The
>     light cache's reach stays camera-centred: with the anchor within 100 m of the camera (tiles of older anchors within
>     200 m) its cascades (kilometres) hold the anchor's rays as the camera's - not tested separately.

>   - **Validation and open (Task 5)**: `cloud_view_cache_test` (the re-anchor at 5 % of R_near, not below; through
>     the planner a re-anchor restarts nothing, every tile that showed a level keeps one, refills take the new anchor,
>     an anchor left 200 m behind loses its tiles), `cloud_vr_stereo_test` (a tile reprojected from its own anchor 100 /
>     200 m off: a deck at R_near within 1.2e-6 / 3.8e-4 eye px), core `[weather]` "The render field's nearest cloud",
>     cloud_check with `STARSIM_MARCH_REFERENCE` (the per-frame march bit for bit as before Task 5). `render_check
>     --frames_every=0.5` (the brief's Starbase ascent through a slab 1.5-2.5 km, `--fly=25 --frames_for=80`, VR preset
>     at render scale 1.65): watched from the ground 2 km east PASS - the mean alpha over the near shell's boundary
>     annulus within 0.010 of the march's (31 samples), flight frames GPU median 5.3 / p99 18.8 ms; **on board FAIL**:
>     outside the deck within 0.003, converged inside it 0.0003, but while climbing through it at 140-160 m/s the cache
>     lags 0.04-0.06 (worst 0.059) and the flight frames' GPU p99 is 27.5 ms (median 8.7; the frame target 22.2 ms).
>     Cause (measured, the anchor held fixed): a cached volume seen from off its anchor loses coverage - 0.075 / 0.18 /
>     0.28 of the mean alpha 130 / 260 / 400 m off inside the broken deck (Task 4's cache without the near shell: 0.20 /
>     0.34 / 0.46); the re-anchor every 100 m cannot refill the view's tiles as fast as the camera moves (a whole view
>     refill takes seconds). Open for the user's VR check: the deck passage from on board (softer, partly thinner near
>     clouds for ~10 s - the spec's accepted "briefly softer").

## Superseded by cloud view cache Task 5 fix round 2 (2026-10-28: motion refreshes at level 0, no keep cap without a cloud in reach, the lookup under the near shell)

>     **Re-anchor** (`CloudViewCachePlanner`): once the camera is `REANCHOR_FRACTION` 0.05 [E] x R_near = 100 m from the
>     anchor (`CloudViewCacheSchedule.reanchor_needed`) it becomes the anchor, the next entry of a ring of `ANCHOR_RING`
>     128 [E]; every tile keeps what it shows from its own anchor (no restart, no tile dropped) until the schedule refills
>     it from the new one (its parallax class by error x nearness - a tile without a known cloud joins it only once its
>     anchor is `SKY_REFRESH_FRACTION` 0.5 [E] of its keep distance off, counted at the cap's distance - and the staleness
>     classes, cross-faded from the fade slot; blocks with tiles missing are expanded before those with a parallax only).
>     **Eviction by a tile's own parallax (fix round 1)**: a tile is dropped once the camera is farther from its anchor
>     than `CloudViewCacheSchedule.keep_distance_m` = `ANCHOR_EVICT_RATIO` 0.1 [E] x its nearest cached cloud distance
>     (200 m for a deck at R_near: the deck passage's measured best), at most `ANCHOR_KEEP_FRACTION` 0.5 [E] x R_near = 1
>     km (clear sky, far clouds: the camera stays inside the anchor's sphere); the planner keeps a tile list and the
>     nearest cloud per ring entry and scans an entry only when the camera may have left its nearest one's keep distance,
>     then hands the schedule the largest kept offset (`set_anchor_deviation`); the tiles of a ring entry about to be
>     reused are dropped too (`CloudViewCacheSchedule.forget`; the near shell's kernel marches them whole in view). The
>     light cache's reach stays camera-centred: with the anchor within 100 m of the camera (tiles of older anchors within
>     1 km) its cascades (kilometres) hold the anchor's rays as the camera's - not tested separately.
>   - **Validation and open (Task 5, fix round 1)**: `cloud_view_cache_test` (the re-anchor at 5 % of R_near, not
>     below; through the planner a re-anchor restarts nothing, every tile that showed a level keeps one, refills take the
>     new anchor; eviction by a tile's own parallax: a cloud at R_near / 4 / 20 R_near and clear sky kept at 5 / 15 / 45 /
>     55 % of R_near off as [all] / [not R_near] / [only 20 R_near and sky] / [none], the schedule's bound the largest
>     kept offset; clear sky joins the parallax class only past 500 m), `cloud_vr_stereo_test` (a tile reprojected from
>     its own anchor 100 / 200 m off: a deck at R_near within 1.2e-6 / 3.8e-4 eye px, 1 km off at 5 R_near 3.8e-4;
>     the pick across two / three anchors' tiles: every settled ray read from the anchor of the tile it reads within
>     1e-6 px, the unsettled ones only where no tile holds the point - the rules before disagreed on 13 / read an
>     unchecked tile on 132 of the same rays), core `[weather]` "The render field's nearest cloud", cloud_check with
>     `STARSIM_MARCH_REFERENCE` (the per-frame march bit for bit as before Task 5). `render_check --frames_every=0.5
>     --frames_for=120` (the Starbase ascent through a slab 1.5-2.5 km, `--fly=25`, VR preset at render scale 1.65;
>     the alpha bound 0.06 while the camera is within a layer's extent +- R_near - the field's nearest cloud within
>     R_near -, else 0.02; the first frame after each sample's reference march not counted: its measured GPU time is the
>     reference's): watched from the ground PASS (worst 0.012, GPU median 6.9 / p99 15.2 ms); **on board FAIL**: GPU
>     median 12.3 / p99 24.4 ms (target 22.2; before the fix 27.5) and the deck passage's alpha worst 0.29 (bound 0.06;
>     before 0.059). The frame does not hold the near shell and a fill: the rest of the frame (scene, VR proxy) p95
>     ~14 ms (median 8) plus the near shell in the deck 8-15 ms leave the adaptive budget at its 2 ms floor, so the
>     deck's tiles are refreshed far slower than the camera moves (the anchor-held loss above) - a trade-off between the
>     frame target and the deck's look for the controller. `--jump_check` fails (deck 0.23 / ground 0.17; at c625926
>     already 0.06-0.11): the near shell's first frames after a restart (frames 2-4) show up to 0.2 less coverage than
>     the march - open. The main-thread section `cloud_view_cache` (the nearest-cloud query and the plan's hand-over)
>     over the 5400 flight frames: median 0.057 / p99 0.082 / max 0.18 ms. Open for the user's VR check: the deck
>     passage from on board.

## World volume schedule before Task 5 (superseded 2026-10-28: the coarsest level budgeted, taps as the cost unit)

Text of `weather-clouds.md` (Task 3) replaced when the GPU bake measured a level-7 cell at 8448 taps (the whole level
takes the GPU seconds, not one frame):

    active / ready / frame origin; `slot_ready`; the spare while it fades). Exposed slabs are baked in the adopting
    plan (not budgeted) and lit first; fills (cold start, jump, level on) are baked under the budget coarse to fine
    (only the coarsest level whole at once); a replaced snapshot slot is rebaked alone (`slot_ready`). Light
    refresh generations (Sun / Moon / blend triggers, tiles nearest first, equal shares finest first) as in the file
    header. Spike frames (cold start, jump, re-frame, until `complete()`): 3 ms split bake / light, else 0.5 / 1.0 ms.
    Calibration: `feedback()`; start rates 1 M bake and 1.12 M light cells/ms [E]. Measured (headless): fill 41 frames
    (0.9 s at 45 Hz), `plan()` p99 0.28 ms at 7.8 km/s, orbital-speed bake 54.7 k cells/frame (= the layout formula).


## World volume window, scroll and occupancy lookups before Task 5 fix round 1 (superseded 2026-10-28: guard band, bake ahead, fade-band lookups)

Superseded by: storage box + window `GUARD_CELLS` inside it, scroll slabs baked earliest-deadline-first ahead of the
window under `SCROLL_CAP_MS`, the occupancy lookups taking level k + 1 in the fade band (review I1 / I2: unbudgeted
slabs stacked to ~3.8 / 5.4 / 12 ms upkeep frames at level 5 / 6 / 7 boundary crossings; the lookups were not
conservative in the fade band). Text as it stood:

    (polar axis +Y). A level's window is centred on the camera's ground point and snapped to the cell; levels 0-1 are
    centred vertically on the camera height and clamped at the shell BOTTOM only (no top clamp, so level 0 switches
    off at 22.1 km without a pop; `level_active`), and fade at their vertical window edges except at the clamped
    bottom (`fade_weight`, 8 cells; the GLSL mirror needs `origin`, `vertical_fade`, `clamped_bottom` from the lookup).
    active / ready / frame origin; `slot_ready`; the spare while it fades, `build_spare` while it is built). Exposed
    slabs are baked in the adopting plan (not budgeted, their cost taken from the budget) and lit first; fills (cold
    start, jump, level on) are baked under the budget strictly coarse to fine - the coarsest level too (Task 5: its
    cap 8) to a block with a bound, separable x / y / z. Every block claimed free has no rho in its support: a sample
    there reads 0 at any blend, and the sub-cell field (it redistributes rho, peak sigma_c) is 0 too.

## World volume nesting and fade-band bound before Task 5 fix round 2 (superseded 2026-10-28: nesting under lag, cross-block bound, clamped bottom)

Superseded by: windows advancing coarse to fine and a nested level stepping only onto nested positions, `shown` =
ready and nested in a shown coarser level (review round 2, N1: under a binding bake cap a finer level's fade band read
level k + 1 outside its window or while it refilled); the fade-band bound over every level k + 1 block the block
overlaps (I1 rest: level 1's blocks straddle level 2's vertically); clamped reads below a clamped bottom (N2). Text as
it stood:

    active / ready / frame origin; `storage`; `slot_ready`; the spare while it fades, `build_spare` while it is
    built).
    `cloud_volume_max_extinction` = max(level k's block bound, level k + 1's) when the block reaches the band,
    `cloud_volume_free_m` = min(level k's free ball, level k + 1's) when level k's free ball reaches the band (the free
    ball is clipped to the level's window).
    each free ball: 0 with density (blend toward level k + 1 included); 1e4 points in the fade band: 0 with density in
    their free ball, 0 above the block bound (with the band ignored, as before the fix: 7-90 per level failed).

## World volume schedule text before Task 6 (superseded 2026-10-28: light volume, split calibration)

Superseded by: `CloudWorldVolumeLight` (light only for filled levels, write modes, settle boxes from the GPU's clamp
counts), the calibration from two GPU intervals (`feedback_split`), a light start rate measured on the slab. Text as
it stood (weather-clouds.md, schedule and GPU side):
    Light refresh generations (Sun /
    Moon / blend triggers, tiles nearest first, equal shares finest first) as in the file header.
    `feedback(upkeep_ms,
    bake_taps, light_cells)` calibrates; start rates 2e7 taps/ms (half the measured, below) and 1.12 M light cells/ms
    [E];
    CloudEffect `run()`s it (with the pass's own camera) after the set 0 is built and before the march (GPU interval
    `cloud_volume_upkeep`), then

## World volume light before Task 6 review round 1 (superseded 2026-10-28: distance-stepped density march)

Superseded by: the distance-stepped march through baked density (16 cells of each level, controller ruling
alternative B), no level reading another's light, one dispatch per level, per-level light and bake rates, the
refresh one level at a time. Superseded on the way (never committed): the constant-cost cascade (16 / 8 cells of
the level, then the next coarser level's own tau read at the exit point): exp(-tau) p99 0.35 / 0.016 / 0.0077 at
levels 0 / 2 / 5 (N 16, Sun 45 deg, broken cf 0.4) but 0.99 / 0.78 / 0.24 at 15 deg (N 8) - trilinear
interpolation of other rays' tau across cloud gaps and the slab top's kink (Sun 10 deg: top cell 0.54-0.70
against 1.39). Text as it stood (calibration line, then the light bullet):

    its rate taps per ms; `feedback_split(bake_ms,
    bake_taps, light_ms, light_cells)` calibrates each rate from its own GPU interval (Task 6); start rates 2e7 taps/ms
    (half the measured, below) and 1.25e5 light cells/ms (section 17's slab, the costliest content measured) [E];

    - World volume light (world volume plan Task 6; schedule `CloudWorldVolumeLight`,
      `game/world/weather/cloud_world_volume_light.gd`, called by the schedule; kernels
      `game/shaders/cloud_volume_light.gdshaderinc`, run by `CloudWorldVolume` after the bake, GPU interval
      `cloud_volume_light`; no march reads the light before Task 7):
      - Texel (RGBA16F): tau_sun, tau_below (vertical, down to the shell's bottom: the transfer table's b), tau_above (up
        to the shell's top: the sky's source), tau_moon - optical depths through rho (cell-mean extinction, both snapshots
        blended by `cloud_blend`). Earth's shadow / Moon off: 6e4 (exp = 0; finite, so a filter across it stays finite).
      - `CLOUD_VOLUME_LIGHT_COLUMN` (per column of a job box): tau_below / tau_above as exact trapezoid sums of the level's
        own column of texels over its window (the trilinear field along the geodetic vertical is linear between centres),
        beyond the window (levels 0-1) the coarser shown levels marched, the outer half cell held as the reads clamp.
      - `CLOUD_VOLUME_LIGHT` (per cell, after a barrier): tau_sun / tau_moon from the cell centre - Earth's shadow when the
        straight ray meets the ground (Gaussian sphere of the grid), the ray clipped to the shell, then steps through the
        job's own (filled) level and the coarser levels SHOWN in its frame: free-space jumps on clear samples (distance
        field; they add nothing), else Delta tau = max(0.5, 0.05 tau) / the block's max-extinction bound (level k + 1's too
        in the fade band), at most 2 cells along the ray, trapezoid, a step whose ends differ by > 50 % halved up to 4 times;
        64 samples; beyond the coarsest level or out of samples the grid-mean column (`cloud_column_mean_depth` over the
        height to the cloud top at the ray's lowest point there, times the path). Error bound (shader header): only steps
        across the trilinear field's kinks err, <= (cell-to-cell density change) x step / 8 per crossing. A re-frame's
        spare reads its own textures without free-space jumps.
      - `CLOUD_VOLUME_PROVISIONAL`: from the next coarser shown level's light at the cell centre, level 7 (or none shown)
        from the grid-mean column (tau_sun = tau_above x path to the shell's top / height to the cloud top).
      - Write modes: DIRECT (fresh cells; in spike frames - cold start, jump, re-frame - and for cells never shown with
        provisional light), RATE (refresh generations: each channel's exp(-tau) moves <= `LIGHT_STEP_T` 0.01 per pass; the
        GPU counts clamped cells per job, read back asynchronously, and a box that clamped any is served again once the
        level's queue is through - "settle" - until it clamps none), BLEND (a box SHOWN with provisional light outside a
        spike: the real light recomputed each plan and mixed in exp(-tau) by dt / the time left, linear over
        `PROVISIONAL_FADE_S` 0.25 s).
      - Schedule: light only for filled (`ready`) levels (their rays read their own window, then coarser shown levels);
        per plan blends, fresh cells (coarse to fine), refresh generations (equal shares, finest first), provisional light
        once. A level's light reference (Sun, Moon, blend) is set at its first real light after a fill (no overwrite while
        filling: Task 3 minor). Triggers as before (shadow shift < 1/4 cell, Moon below -6 deg at the level centre, blend
        0.02). Jobs: one storage buffer per batch, the job by push constant, one uniform set per written texture (a set
        per job ran out of D3D12 descriptors at a check's 40 x budget). `light_budget_scale` (checks only).
      - Measured (cloud_check section 17, RTX 4090; section 8's thick slab 1-2 km, tau ~10, at Starbase, camera 2 m):
        tau_below / tau_above vs the CPU bake's column <= 0.2 %; tau_sun (Sun 45 deg) vs the converged ray integral through
        the level's density <= 0.64 / 0.15 / 0.14 % (levels 0, 2, 5); tau_moon (Moon 40 deg) <= 0.76 / 0.08 / 0.06 % (levels
        0, 2, 5). The analytic slab's depth above / mu0 differs by up to 4.9 % at levels 0-2: the baked slab's top undulates
        with the bake's shape noise (tau_above varies 9.6-10.3 over 1.5 km at level 0), so a slanted ray crosses another
        column's top (the uniform-slab reference holds only in the cell's own column). A 5 deg Sun step: one refresh pass
        moves exp(-tau_sun) by at most 0.0103 (612 k / 125 k texels clamped at levels 0 / 2; level 5's threshold above the
        step: no refresh), settled 0.042 / 0.042. Earth's shadow (Sun 8 deg below the site): all 24.9 M texels of levels
        0, 2, 5 with the Sun >= 6 deg below their horizon at 6e4. No NaN / infinity in any level after five fills. Sun
        10 deg up: no Moon jobs. Cold start (light at the spike share, bake at 40 x): every level completes, frames from
        density shown 16 / 22 / 26 / 30 / 34 / 38 / 43 / 51 (levels 7 .. 0; level 0 misses the 1 s / 45-frame target on
        this worst-case content: every ray through cloud, ~61 k cells/ms at level 0, ~2.9e5 cells/ms on average); light GPU
        median 2.71 ms in those frames (the spike's 3 ms less the bake), max 14.7 ms (the calibration lags the content).
      - Deferred to Task 7 (ruling C12: no march reads the light yet): the broken cf 0.4 views' radiance vs the converged
        per-sample march, radiance 0 in Earth's shadow, the cold start's first frame with clouds, `render_check
        --front_check` (C13: the front metric on the volume's change less the reference's).

## World volume light cost model and N = 16 before Task 6 review round 1c (superseded 2026-10-29: N = 32, latency-aware cost model)

Superseded by: 32 cells per level per march segment (controller ruling, round 1c: level 2 at a 15 deg Sun read 400 m
cells for the cumulus 1.35-2 km up), and a per-level cost of latency + cells / rate learned at the slow end of the
window, with growth bounds and the fill bake held while a filled level's light is due.

```
    from the cell centre CLOUD_LIGHT_LEVEL_CELLS = 16 cells of the cell's own level along the ray (a cell along the ray
    = the shorter of its horizontal and vertical crossing), then 16 cells of each coarser level, 2 midpoint samples per
```
```
    quadratic through its exact grid points at its ends and middle (no geodetic transform per sample). Cost: a
    constant per level (2 x 16 samples x the levels on the path), shortened by the top exit.
```
```
  - Cost model (`CloudWorldVolumeSchedule`): budgets in ms; a light job costs its cells at the level's light-kernel
    rate plus its column cells at the level's column rate, each level `LIGHT_LEVEL_OVERHEAD_MS` 0.05 once a plan.
    `feedback_split` calibrates every rate from its own GPU interval: an overrun adopts the measured rate at once, an
    underrun raises it by 20 % of the gap (at most x2); a light dispatch below `LIGHT_CALIBRATION_MIN_CELLS` 65536 may
    only raise a rate (its time is its longest ray's latency: adopting it spiralled the plans down to one column per
    level); an unmeasured level takes the slowest measured level's light rates. Spike frames give the light
    `LIGHT_SPIKE_SHARE` 0.65 of what the bake leaves of 3 ms (level 0's rate jitters by up to ~40 % between frames).
    An upkeep over 2 x its plan halves the next plan once.
```
```
    - Slab, Sun 45 deg: tau_sun worst 0.18 % (levels 0, 2, 5); tau_below / tau_above vs the CPU bake's column
      <= 0.17 %; tau_moon (Moon 40 deg) <= 0.14 %. Sun 10 deg: worst 1.39 % (the top cell 1.391 vs 1.393 at level 0,
      9.172 vs 9.192 at level 2; the cascade's slab-top kink error is gone).
    - Broken field, exp(-tau_sun) error (4096 random cells per level inside the window, 0-2.5 km), p90 / p99:
      Sun 45 deg level 0 0.0000 / 0.22, level 2 0.0004 / 0.0046, level 5 0.0006 / 0.0024; Sun 15 deg level 0
      0.0016 / 0.85, level 2 0.0017 / 0.18, level 5 0.0006 / 0.0044. Shadow edges (exp(-tau) = 1/2 crossings at 500 m)
      45 deg: levels 2 / 5 0.08 / 0.03 cells (level 0 none); 15 deg: level 0 5.2, level 2 8.3, level 5 0.02 cells.
      Open (controller): level 2 at 15 deg misses p99 <= 0.02 and the 2-cell edge bound - its rays' far part reads
      level 3+ (400 m and coarser cells) where the reference keeps level 2's 200 m; with 32 cells per level it passes
      (level 2 p99 0.0060, edges 0.03 cells; level 0 p99 0.50) at about half the light rate (level 0 0.62e6 cells/ms)
      and a pair switch's light median 1.06 ms (over the 1.0 ms budget).
```

Round 1b's numbers with this model (cloud_check sections 17 / 18 / full): cold start share 0.65, upkeep max 2.27 ms
alone and 3.18 ms in the full run (level 0: 3.07 ms against 1.95 planned, its rate jittering faster than the
feedback); levels 7 .. 0 complete 34 / 48 / 43 / 52 / 38 / 37 / 88 / 30 frames after their density shown (level 1 at
48-98 over four runs). At share 0.5 every level took 57-194 frames; planning at the slowest of the last 45 rates held
level 0 back 74 frames - below ~0.4 ms a level's dispatch did not shrink with its cells.

## World volume light: starvation open point (superseded 2026-10-29, Task 6 review round 2)
- "World volume light, starvation: a level whose per-plan latency alone exceeds the 1.0 ms light budget is skipped and
  would never be served in normal frames (measured latencies <= 0.4 ms, so not hit today)." Replaced by partial
  columns and the probe after `LIGHT_PROBE_FRAMES` plans (weather-clouds.md, cost model). The re-review also named a
  level-0 column costing ~1.25 ms at ~1e5 cells/ms; that mixed up cells and columns (a column is ~124 cells, ~1.2 us
  at that rate) - the live starvation path is the latency, not the column.

## World volume text before Task 7 (superseded 2026-10-29: the march reads the volume)

```
  `STARSIM_WORLD_VOLUME=1` or `--world_volume`; Task 5 runs the upkeep alone, no march reads the volume yet):
```
```
  `cloud_volume_light` with per level `cloud_volume_light_column_<k>` / `cloud_volume_light_cells_<k>`; no march reads
  the light before Task 7):
```
```
  - Deferred to Task 7 (ruling C12: no march reads the light yet): the broken cf 0.4 views' radiance vs the converged
    per-sample march, radiance 0 in Earth's shadow, the cold start's first frame with clouds, `render_check
    --front_check` (C13: the front metric on the volume's change less the reference's); the Moon's ramp-in (its
    channel is gated by the level's Moon flag, which switches with the jobs that write it).
```

## World volume march, Task 7 round 1 (2026-10-29): superseded approaches and numbers

- Sub-cell field, spreading form (superseded by the threshold form, controller ruling R1): density = sigma_c
  ramp(ramp^-1(c) + 4 c (1 - c) v / w), v = w_s (s_f - s_cell) + P_L + t(0.5, 0) - t(0.5, L) (the detail made
  zero-mean by the tables' threshold shift at the median fraction). It spread the cell's water as haze over the cell:
  V3 alpha +0.04 against the reference (mean T 0.487 / 0.527), edges 3 x wider (2.37 vs 0.79 px). Before it, an offset
  w ramp^-1(c) with the tables' shift at a fraction estimated from c eroded cells next to solid ones into dark blocks.
- The air shadow's loss was composited only on rays with cloud: V6_backlit's clear sky lost the loss the reference has
  (clouds in front of the Sun shade the air along clear rays) - A/B median 34 % (round 0) / 19.9 %; V3 9.4 %, V1 0.5 %.
  With the loss on every ray: 0.39 / 0.88 / 0.00 %.
- Surface correction on every step (round 0: ~9.6 light fetches per occupied step, ~40 % of the 95.6 ms V1 pass), then
  on the ray's first 4 light evaluations only (ruling R3's text): section 19's slab at Sun 20 deg worst 11.8 % (bound
  5 %); replaced by the correction at the light nodes (one per half cell): 3.3 %.
- `CloudShadowEffect.ran_msec` / a 250 ms validity window: a ~100 ms check frame (the full-resolution march) outlasted
  it; the shadow maps switched off and on between frames of an unchanged scene (repeat_identical DIFFER, 1.1 M pixels).

## Baked prototype step 2: flat outer levels (commit e8a0e4a, 2026-10-29), superseded by step 3

- **Flat outer levels and the level rule (step 2)**: beyond the volume, levels 6.. of 256 x 256
  2.5D columns (6.4 km doubling), azimuthal equidistant about their own anchor, over only the visible cap (camera
  horizon + the 20 km tops' horizon, baked for 1.25 x the height + 2 km, + 0.5 deg, + 2 deg drift; the far side is
  never baked or fetched): ground 2 levels, ISS 4, 8 600 km 5. Geometry (shared by the sets until the cap or the
  weather changes; `CLOUD_BAKED_FLAT`): projected cover from 4 x 4 jittered shape taps (field at 2 x 2 points, one
  sample per field layer, 8 in ice layers) as the volume's signed edge coordinate, the cover, the grid-mean vertical
  optical depth, top and base. Light per set (`CLOUD_BAKED_FLAT_LIGHT`, 4 dispatches): the old far tier's column model
  (Monte Carlo table, 3D gain) for the set's anchor view, the phase per pixel; x `CLOUD_BAKED_FLAT_SUN_GAIN` 1.4 [E]
  (the column model, within 2 % of the old far tier's, lit a deck ~15 % darker than the volume / the old march's
  shapes). Per pixel: an entry on the top (descending) / base (ascending) shell from a guess fetch (heights only) and
  a second fetch (F smoothed by four bilinear taps, L, H), the slant cover 1 - (1 - C)^(1 + depth cot / 2 km).
  Level rule (all levels): l = log2(max(r / inner_0, footprint / c_0)) per sample; level ceil(l), blended toward the
  next over a band of 0 octaves below 20 km (the approved hard switches; any band cost ~0.17 ms on the ground) growing
  to 1 octave at 60 km (from above a level's 256 columns end long before the footprint wants the next: its border
  showed as a square). The volume's last level blends into the flat levels over >= 0.25 octave. Levels finer than the
  footprint at the nearest possible cloud are neither baked nor marched (bake with 0.8 x the pixel angle, back with
  1 x): at divisor 2 in the mock (1.39 mrad) level 0 drops above 110 km (back below 92 km), 1: 200 / 164, 2: 380 /
  308, 3: 741 / 597, 4: 1 462 / 1 174, 5: 2 904 / 2 327 km; memory per set 219 MB on the ground, 72 MB at the ISS.
  Anchor move trigger max(3.2 km, 0.05 x height above 20 km). Beyond the scaled-space switch (~8 600 km) the far field
  (earth_clouds.gdshader) draws as before. Stereo mock, divisor 2 (march ms before -> after): ground 1.70 -> 2.02,
  9 km deck 2.34 -> 2.79, 100 km 0.57 -> 0.86, ISS oblique 0.31 -> 0.42, ISS nadir 0.71 -> 1.01. Bake: first 40-90 ms
  GPU (dispatches <= 2.9 ms), re-bakes with the geometry kept ~19 ms. Diagnostics: `STARSIM_BAKED_DEFINES=
  CLOUD_BAKED_FLAT_ONLY | CLOUD_BAKED_VOLUME_ONLY | CLOUD_BAKED_NO_BLEND`. Report: `.superpowers/sdd/
  2026-10-28-cloud-world-volume/baked-outer-report.md`.
- Why superseded (step 3, user findings on e8a0e4a and the ruling that the old far field's cover is correct): against
  the far field at the same camera the step 2 flat levels drew 1.09-1.14 x its mean alpha and 1.23-1.45 x its
  brightness (the slant cover 1 - (1 - C)^(1 + depth cot / 2 km) over the whole band, the Sun gain 1.4 [E] fitted on one
  deck); 256-column levels ended long before the footprint wanted the next (lattice, a square at the ISS); the crisp
  sub-cell outline of 6.4 km+ cells drew blobs. Step 2's per-tap cover itself was optically thick (98.6 % of it from taps
  of nadir opacity > 0.9 at the ISS view): not haze.

## Baked prototype step 3: flat levels' per-cell light and footprint-limited pattern (commit 2103a65, 2026-10-29), superseded by step 4
- What it was: per set an L array (RGBA16F per flat level, kernel `CLOUD_BAKED_FLAT_LIGHT`) = the column model's light
  (single scattering / P, multiple, sky) for the view from the set's anchor at the column's entry height x C_col, and
  C_col; the march divided the bilinear L by C_col and applied the phase and `atm_sun_at` / `atm_sunlit` at the pixel.
  The field's column in the flat bake was bilinear (`cloud_column_mean_depth`); F was fetched bilinear (heights) or as
  the mean of four bilinear fetches half a texel apart (where the cell spanned > 2 pixels). The per-pixel sub-cell
  pattern only where the cell was 2-4 x the footprint, faded out over 2-4 km footprints; P faded to C_col from 6.4 to
  12.8 km cells, so beyond that each cell was an even cover.
- Why superseded (step 4, the user's headset view on 2103a65, GFS 29.10. 18 UTC +4 h, low Sun, a few thousand km up):
  large square cells with staircase outlines (the field's 0.25 deg cells drawn as bilinear tents and plateaus - the far
  field reference shows the same squares - with no sub-cell shape at 3-30 km footprints), cells lit orange next to
  white ones (the light per cell for the anchor's view, the entry height jumping per cell), blur at intermediate
  distances and a dotted texture at 600-1500 km (the 32 km shape tile's period in the 3.2 km cells' pattern).

## Baked prototype step 4: entry on the top shell, domain warp, uniform shapes (commit 0bee596, 2026-10-30), superseded by step 5
- What it was: F = (P, depth, top P, base P), T = (C_col, top): top / base the highest cloudy layer's top and the lowest
  one's base (fraction >= CLOUD_MIN_FRACTION) at the cell's field points; the bake's pattern read the largest-fraction
  layer's types at its height. The march entered on the top shell (descending) or the base shell (ascending) of the
  fetched column, rejecting entries outside top / base +- (50 m + 1e-4 x distance), and lit the far field's seen depth
  below the entry. Shape coordinates domain-warped (`cloud_baked_warp`: the shape's coarsest mip at 0.19 x q, +-0.35
  tile). The per-pixel pattern was a soft step (+-0.03 of the shape value) per tap: inside the shapes the depth was
  even (in decks textured x 1 + 3 (s - s_50)).
- Why superseded (step 5, controller's review of the approach sheets): a seam of displaced shapes and pixel garbage
  along a line at 60-250 km - a latitude near 26 N in the ERA5 test field where an upper layer's fraction crossed
  CLOUD_MIN_FRACTION: the top (and the largest-fraction layer) jumped by kilometres, so the entry point, hence the
  pattern's point, jumped (diagnosed by colouring Y = 0 of the flat grid, then moving the camera 1 deg south: the seam
  stayed at the latitude); a straight "roof" edge at 1 500 km low Sun (the Earth's shadow edge on one height per column,
  kinked where the height jumped; the far-field reference shows it too); staircase outlines (binary per pixel at divisor
  2); uniform white / grey paper cut-outs; marbled streaks at 1 500-4 000 km (the warp's shear).
- Baked prototype, superseded 2026-10-30 (cirrus check, `baked-cirrus-report.md`): the takeover (40-60 km) scaled the
  volume's extinction by 1 - w and drew the flat levels' alpha x w behind it (dense volume shapes stayed opaque, the
  flat shapes filled the gaps at half alpha: grey translucent shapes, mean alpha +23 % at 50 km); a plain cross-fade of
  the two renders (tried) kept the mean but drew both uncorrelated shape sets at half alpha everywhere. The empty-block
  skip took the first sphere root ahead of t for the block's height exit; the roots' ~1 m float32 rounding let a ray
  just under a top shell miss both and skip to an x / y face (pixel holes in square rings in cirrus overhead).

## Baked prototype step 5 terminator ruling and the far field's entry switch (commits 85bd9f5 / earlier, superseded 2026-10-30 by step 6)
- What it was: the far field's column light (`cloud_far_cloudy_radiance`) turned its entry normal toward the Sun only
  while mu0 > 0 (`entry = mu0 > 0 ? cloud_entry_normal(...) : up`); below the cloud's local horizon it lit the bare
  slab at the 0.02 floor. Step 5 ruled the remaining day / night edge on the flat levels "the Earth's true shadow edge,
  as sharp as in the far-field reference" and only spread it over the column's heights (`CLOUD_BAKED_FLAT_TERMINATOR_MU`,
  kept). The flat levels drew nothing beyond the shown geometry's cap.
- Why superseded (step 6, user in VR): the switch dropped the light ~7 x at mu0 = 0 (entry cosine 0.15 -> 0.02): a
  step x 1.4 in a nadir view (3 000 km, overcast slab) and a hard cut toward the Sun at the terminator; physically the
  edge ends by the Sun's grazing-path transmittance, its disc's visible fraction and the twilight sky, all continuous.
  The old march's far tier keeps the switch (old march unchanged). A camera faster than the re-bake saw the clouds end
  at the old geometry's cap (a circle inside the disc): now the far field's column beyond it.

## Baked prototype step 7: the light bake's Sun path (06e8e43 and earlier, superseded 2026-10-30)
- What it was: the light bake integrated the Sun's optical depth through the baked shapes along the whole Sun path
  (32 quadratically spaced samples to the band's top, at most 300 km; the grid mean only beyond the levels).
- Why superseded: toward a 5 deg Sun through broken cirrus (debug ice slab 9-11 km, cf 0.5) the baked path was 1.14 x
  the old march's brightness per alpha (away from the Sun 1.01 x): on a grazing path of ~100 km through the layer the
  shapes' gaps let the Sun through, while the old march and the core take the shapes over the first 6 km and the grid
  mean beyond. Evidence: 128 samples -0.2 %; the shapes over 6 km only 1.14 -> 1.02 x (`baked-lowsun-report.md`).
- Also superseded by step 7: the far field's column light (flat levels, scaled-space far field) lit the clouds under
  a low Sun as the slab with the 0.15 entry turn alone; at the 40-60 km takeover from the Sun's side its cloud radiance
  was 0.49 / 0.40 / 0.39 / 0.42 x the volume levels' at Sun 15 / 8 / 5 / 2 deg (high Sun 0.90; brightest 30 % of the
  50 km crop at 8 deg 137 / 120 / 116 sRGB against the volume's 191 / 164 / 145), a dull pink-grey. The flat levels'
  early exit dropped every pixel whose scene depth lay in front of the 20 km shell's entry (striped holes at the limb).

## Baked prototype: cloud shadows from the old maps (59bd8ea and earlier, superseded 2026-10-30)
- What it was: with the baked volume drawing, the ground's cloud shadows and its march's air shadow still came from
  the old shadow maps: the near map (+-16 km) marched the old march's field shapes per frame, the far map (+-200 km) the
  grid mean as the beam cover of the cell fraction (no shapes), the slanted column beyond 120-140 km; the air shadow
  took the near map's shapes within 9-11 km and the grid mean beyond. The shadow pass cost ~0.6 ms per frame (stereo
  mock, G view; 1.3 ms in the mock's slow state).
- Why superseded: the shadows did not come from the shapes the baked march draws (its box-filtered bake with level-1
  detail; its own light bake), and beyond 9-11 km neither the ground nor the air saw any cloud's shape - crepuscular
  rays of clouds 20-40 km up-Sun and the shapes' shadows on distant ground were a smooth grid-mean dimming. Now S, the
  bake's line toward the Sun per column (`baked-shadow-report.md`).

## Removed 2026-10-30: cloud view cache, spike S, hybrid march, old world volume

The baked volume became the default of every start (user 2026-10-30); the direction view cache
(`cloud_view_cache*.gd`, `cloud_view_cache.gdshaderinc`, its test, `--ab_view_cache` / `--no_view_cache` /
`--shimmer_check` / `--jump_check` / `--frames_every`, the `cloud_view_cache` flag, PerfLog's cache columns), spike S
(`cloud_spike.gd` / `.gdshaderinc`, `--spike*`), the hybrid march (`cloud_hybrid.gdshaderinc`, `--cloud_method=hybrid*`)
and the old world volume (`cloud_world_volume*.gd`, `cloud_volume*.gdshaderinc`, its test, cloud_check sections 15-19,
`STARSIM_WORLD_VOLUME`, `--world_volume*`, `--ab_reference`, `--edge_check`, `--front_check`, the globals
`cloud_edge_curl`, `cloud_light_entry_samples`, `cloud_noise_curl` and the sub-cell statistics) were deleted in one
refactor commit. Their sections of the subsystem file follow verbatim. Before the switch the cloud kernels were built
serially at CloudLayer construction (~20 s of a GPU start; `.superpowers/sdd/loading-screen2-report.md`).

### Hybrid march (model)

- **Hybrid march (prototype, 2026-10-29; the user tests it live in VR)**: `CloudLayer.march_method`
  (`CloudEffect.MarchMethod`; key F7, user argument `--cloud_method=old|hybrid|hybrid_no_skip|hybrid_old_light`) and
  `CloudLayer.divisor_override` (F8: 1, 1.5, 2, 3; `--cloud_divisor=<x>`; the divisor may be fractional). Choosing a
  method turns the view cache off; the world volume's upkeep runs from the first hybrid frame on. The hybrid kernels
  (`cloud_hybrid.gdshaderinc`, defines `CLOUD_HYBRID`, `CLOUD_HYBRID_SKIP`, `CLOUD_HYBRID_VOLUME_LIGHT`; compiled on
  first use, 1-5 s each; `tests/hybrid_compile_check.gd` compiles all 24) are today's march - shape, detail and
  weather per sample, its step schedule, outline pass, temporal copy and upsample - with: (a) the skip of the volume's
  proven-empty ball behind a clear sample (distance field of the finest frame-0 level holding the point, the grid
  carried as a quadratic through three geodetic solves per 40 km, margin 2 m + 1e-4 x distance; no free-box exit), and
  (b) the light from one trilinear fetch of the volume's baked (tau_sun, tau_below, tau_above, tau_moon) at the step's
  light point (faded toward level + 1; beyond the volume the far tier's light path). The T7 world-volume march stays
  behind `--world_volume` (with the method "old").
  Desktop 1600 x 900, march kernel ms (cloud_pass_march, median of 20 frames; refine adds 3.4-6.7 ms):

  | view / divisor | old 3 / 2 / 1.5 / 1 | hybrid (vol. light + skip) | hybrid, old light + skip | vol. light, no skip (3 / 1) |
  |---|---|---|---|---|
  | V1 | 4.6 / 7.0 / 12.9 / 28.2 | 3.3 / 6.6 / 7.8 / 20.6 | 6.7 / 8.6 / 16.5 / 33.7 | - |
  | V3 | 6.8 / 11.0 / 18.3 / 39.3 | 5.9 / 8.5 / 15.9 / 33.0 | 8.3 / 15.1 / 25.3 / 51.1 | 4.6 / 27.1 |

  The skip works (V3 divisor 1: density fetches 5.97 G -> 2.12 G, empty steps 364 M -> 109 M, the image within 1 % of
  pixels of today's) but costs time (its data-dependent steps: the latency chain of the cost diagnosis); hence the
  method `hybrid_no_skip`. The volume light looks different: plastic domes on the deck from above, blotchy dark patches
  on cumulus flanks from the ground (the light volume's cells vs the per-sample shapes; the T7 march showed them too).

### Cloud view cache (validation)

- Cloud view cache fill (cloud view cache Task 3, cloud_check 15, `STARSIM_VIEW_CACHE_ONLY=1`): the march image of a
  static view twice bit-identical (a broken cumulus layer from the ground, a deck from above; 256^2, desktop preset);
  `STARSIM_MARCH_REFERENCE=<dir>` compares the march images (desktop preset; divisor 1, debug 0 / 12) with a directory
  written before a refactoring - the extraction of `cloud_shade_ray`: 0 differing pixels in all six. The whole cube
  filled at the full level from the camera vs the march at divisor 1, each pixel's ray sampled bilinearly through
  `CloudViewCacheLayout.texel_of`: face-aligned views (90 deg on 256^2 texels per face: pixel rays = texel rays; the
  zenith from the ground under cf 0.4, the nadir above a deck) luminance error median 0.000 / 0.217 %, p99 2.7 / 1.4 %
  (bound 1 / 5 %), T RMS 0.0039 / 0.0000 (0.01), cloud distance median 0.000 / 0.220 % (1 %) - the deck's 0.2 % is
  the static end (the fill ends at the ellipsoid, the depth-less check's march at the -500 m shell: other step
  lengths); general 60-deg views (448^2 texels per face) median 0.09 / 0.47 %, distance median 0.04 / 0.10 % asserted,
  p99 46 / 7.8 % and T RMS 0.021 / 0.020 printed: a ray against the bilinear mean of the four texel rays around it at
  the deterministic march's hard edges and step grain (the lookup's A / B on general views: below).
- Cloud view cache in the frame (cloud view cache Task 4, fix round 1): `cloud_vr_stereo_test` mirrors the parallax
  lookup - a cloud at 300 m / 2 km / 20 km seen from each eye (IPD 0.064 m) and from a camera 50 m off the anchor lands
  within 1e-12 eye px (35 px/deg; bound 0.25) when the cached distance is the cloud's in every direction; a flat deck
  through the point within 1e-8 px from 2 km on (0.61 px at 300 m, printed: inside the near shell's reach).
  `cloud_layer_test`: a view jump and a clock jump restart the cache (nothing above level 0 shown afterwards); after
  the view jump and after a layout re-creation the batches say covering until the view's sample rays show a level
  faded in, and not once converged; a 1 deg Sun step starts cross-fades (refreshes from their fade slot), none
  complete in the frame they start. `cloud_view_cache_test`: `completed_batch` / `set_tile_distances` equal
  `completed` / `set_tile_distance` per tile over 60 frames; `missing_in_cone` finds a cold cache, not a filled cone,
  and one missing tile at the cone's edge. `render_check --jump_check` (desktop 1600 x 900): every frame's cloud
  coverage (mean 1 - T) after a view jump and after a layout re-creation (render scale x 1.3) within 0.02 of the
  per-frame march's: deck 0.0188 / 0.0111, ground 0.0121 / 0.0063 (the remaining deviation: the coarse levels show a
  little less cloud than the full one). `render_check --shimmer_check` (deck view, cache full, 16 frames yawed +-0.3
  px): the frame-to-frame change's p99 0.237 vs the march's at divisor 1 0.257 (ratio 0.92, bound 1.10).
  `render_check --ab_view_cache` (cache full vs the march at divisor 1, desktop 1600 x 900, 2 x 2 blocks, luminance
  error / max(|march|, 5 % of the mean); controller ruling: the p99 over the blocks without scene geometry bounded by
  the same view's march against itself turned by 0.5 px, the median by 1 %): deck / ground / 20 km / dusk median 0.21
  / 0.07 / 0.16 / 0.12 % (met), p99 9.0 / 14.6 / 9.9 / 22.4 % against the calibration's (the max over four half-pixel turns,
  +-x azimuth and +-y elevation) 27.5 / 49.4 / 34.2 / 81.6 % - all met (the +-x turns alone: 16.5 / 43.0 / 8.2 / 66.1 %;
  the elevation turns are the larger ones); T RMS 0.009 / 0.007 / 0.022 / 0.007. Images
  `screenshots/cloud-view-cache/t4_<view>_ab_f2_{cache,march,ab_error}.png`.

### Spike S and the world volume (diagnostics)

- Spike S (world volume plan Task 2; throw-away, removed in Task 13; numbers in `vr-performance.md` "Spike S"):
  `render_check.gd --spike=a|b [--spike_scale=0.5|1.0]` (with `--stereo`, a slab `--cloud_view`, `--handover=0,0`)
  replaces the march by `CloudSpike` (`game/world/weather/cloud_spike.gd`, `game/shaders/cloud_spike.gdshaderinc`):
  the far tier alone, rays [2 km, end] from the cyclops point over the centre view (divisor 2 / 1), over scratch
  volumes built over the first frames (one dispatch per frame; render_check waits for `CloudSpike.ready`): an
  occupancy over the view's band region (200 m cells, 4^3 taps of `cloud_extinction_at` at each tap's own footprint +
  the lift-bound shape test, Chebyshev distance field capped at 16 cells, max extinction dilated by one cell; grid
  azimuthal-equidistant about the camera, model height; tiles of 2048 texels stacked in depth for D3D12's 3-D limit),
  for (b) a 100 m bake of the cell-mean extinction and a dummy RGBA16F light volume. Steps: sphere tracing where the
  distance is >= 2 cells, else min(Delta tau 0.3 from the max extinction, half a cell, 1/4 of the finest structure at
  the footprint) >= 2 m, one sample per step, constant light (per-ray Sun irradiance / 4 pi), today's termination; no air.
  (a) samples `cloud_extinction_at`, (b) one bake fetch + one light fetch. `--spike=ref` is the sanity check's
  reference: today's march with `CLOUD_SPIKE_CLIP_M` (its rays start at 2 km). The log prints the plan
  (`spike plan: ...`) and `spike centre <w>x<h> mean alpha a mean luminance l`; with `--counters` the channels carry
  extra counts in their high 16 bits (`tau_limited_steps`, `extinction_steps`, `structure_limited_steps`;
  `occupied_steps` = steps in occupied space, the dilated occupancy).
  Round 2 (Task 2b): `--spike_levers=1,2,4,s,t` / `--spike_stop=<T>` switch the step rule's levers (a bake max volume:
  the 3^3-texel max per bake cell, or minus the Chebyshev distance of a clear cell; Delta tau from it, sphere tracing on
  it, one analytic step per bake cell, trilinear / structure options); numbers in `vr-performance.md` round 2.
- World volume layout and upkeep schedule (world volume plan Task 3; pure logic, no GPU; headless
  `res://tests/cloud_world_volume_test.gd`, ~80 checks, prints `cloud_world_volume_test: PASS`):
  - `CloudWorldVolumeLayout` (`game/world/weather/cloud_world_volume_layout.gd`): 8 levels of 256 x 256 x 128 toroidal
    cells (cell sizes `CELL_H_M` 50 m .. 6.4 km, `CELL_V_M` 25 / 50 / 164.0625 m over the shell -500 .. 20 500 m); the
    horizontal axes are the azimuthal-equidistant projection about the frame origin on the Gaussian-radius sphere
    (geodetic normal of the point, so no flat-tangent miss), z = ellipsoid height (round trip < 1e-6 m). Render axes
    (polar axis +Y). A level's texture holds its STORAGE box (256 x 256 x 128 cells, `storage_box`); the WINDOW the
    lookup reads (`window_size`: 252 x 252, x 124 at levels 0-1, x 128 at levels 2-7) lies `GUARD_CELLS` = 2 [E]
    inside it on each side of every scrolling axis, so the slabs ahead of the camera are baked before the window
    reaches them (Task 5 review I2; no memory, 1.6 % of each level's reach). The window is centred on the camera's
    ground point and snapped to the cell (`window_origin`); the storage's target is the window grown by the guard
    (`storage_target`, not below the shell; `storage_range`: the storage origins that still hold a window). Levels 0-1
    are centred vertically on the camera height and clamped at the shell BOTTOM only (no top clamp, so level 0
    switches off at 22.1 km without a pop; `level_active`), and fade at their vertical window edges except at the
    clamped bottom (`fade_weight`, 8 cells inside the window; the GLSL mirror needs `origin` (the window's),
    `vertical_fade`, `clamped_bottom` from the lookup).
    `exposed_slabs` gives disjoint half-open boxes; `box_split` / `box_clip` are the shared box helpers.
    Max-extinction mip / distance field: `MIP_DIV` 4 -> `mip_size()` 64 x 64 x 32 texels per level.
  - `CloudWorldVolumeSchedule` (`..._schedule.gd`): `plan(camera, sun, moon, blend, cloud_top_m, jumped, delta_s,
    slot_ids)` -> `bake` ({level, from, to, snapshot_channel -1 = both | slot, spare}), `provisional` (+ `from_column`
    for the coarsest level), `light` (+ `moon`), `reframe`, `lookup` (state after the batch: per level origin /
    active / ready (filled) / shown / frame origin; `storage`; `slot_ready`; the spare while it fades, `build_spare`
    while it is built; `shown` sets the GPU's READY flag). Scroll (bake ahead of exposure, Task 5 review I2): a ready level's storage moves toward the window's
    target grown by the guard, as far as it still holds the shown window; the slabs it exposes are queued and baked
    earliest deadline first - a slab's deadline is the frames until the window, at the camera's per-axis speed
    (`MOTION_DECAY` 0.9 peak hold [E]), would reach it; the frame's scroll budget is the deadline-meeting rate x
    `SCROLL_MARGIN` 1.5 [E], capped at `SCROLL_CAP_MS` = 3.0 - 1.0 ms (the spike cap less the light), and the normal
    bake share the fills leave bakes further ahead. The window then advances toward the camera only onto baked cells
    (no frame reads an unbaked cell; a missed deadline makes the window wait = lag, counted in `windows_waiting`;
    a move of a whole storage extent restarts the level's fill). Nesting (Task 5 review round 2, N1 ruling): the
    windows advance coarse to fine, and a level nested in the next coarser one (`CloudWorldVolumeLayout.nested`: its
    window inside level k + 1's weight-1 region, 8.5 cells in) steps only onto positions that keep it nested - a finer
    level waits for its coarser one (lag, the spatial fade band covers the camera leaving it; ~54 cells of level k + 1
    of slack at the targets). A level is `shown` only while it is ready and the next coarser level is shown with this
    window nested (transitively; the coarsest: ready): a coarser level restarting its fill hides the finer ones with
    it, so the fade band, the bound and the free lookups never read a level that is not shown. Levels of two frames (a
    re-frame in progress) are not compared (carry to Task 9: the old frame's band reads level k + 1 with its own
    grid). Baked slabs are lit first. Fills (cold
    start, jump, level on) are baked under the budget strictly coarse to fine - the coarsest level too (Task 5: its
    8.4 M cells of 8448 taps take the GPU seconds), and nothing of the volume is shown before it is `ready`; a replaced
    snapshot slot is rebaked alone (`slot_ready`, the spare included; a level still filling rebakes only what its fill
    baked: no cell twice). The schedule keeps the layout's `origins` (`fade_weight`). Light: `CloudWorldVolumeLight` (below:
    filled levels only, write modes, settle). Spike frames (cold
    start, jump, re-frame, until `complete()`): 3 ms split bake / light, else 0.5 / 1.0 ms. The bake's cost unit is the
    tap (`CloudWorldVolumeLayout.cell_taps(k)`: the core's tap rule), its rate taps per ms per level
    (`bake_level_taps_per_ms`; light: per-level rates, below); `feedback_split(m)` calibrates each rate from its own GPU
    interval (Task 6 review round 1); start rates 2e7 taps/ms (half the measured, below) and 1e6 light cells/ms [E]; `budget_scale` (checks only) scales every budget. Measured (headless, at a test rate of 1e9 taps/ms): fill 90
    frames, `plan()` p99 0.28 ms at 7.8 km/s, orbital-speed bake 54.7 k cells/frame (= the layout formula). Scroll
    flights at the game's 4.3e7 taps/ms (`cloud_world_volume_test`, heading 30 deg, an independent model of the
    unbaked cells): 1 km/s at 10 km climbing 150 m/s (all 8 levels, 1400 frames) scroll bake mean 0.44 / worst
    0.50 ms, every plan <= 0.58 ms; 7.8 km/s at 120 km (levels 2-7, 1200 frames) mean 1.36 / worst 1.94 ms, every plan
    <= 2.04 ms; no window waits, no unbaked cell read. 7.8 km/s needs ~1-1.5 ms of scroll bake per frame on average
    (taps 528-8448 per cell), above the 0.5 ms normal bake share: in orbit the upkeep runs ~2-2.5 ms with light.
    Capped flights (budget x 0.1 / 0.3 at 7.8 km/s, x 0.05 at 1 km/s climbing, 1500 frames each: the cap binds as at
    ~2x time warp in orbit or on a slower GPU): the windows wait in 1495 / 1484 / 1486 frames (levels not ready after
    a whole-extent lag: 1360 / 53 / 796 level-frames), 0 unbaked cells read by a shown level, the shown levels nest
    in every frame (before the fix 542 / 907 level-frames outside level k + 1's weight-1 region, 626 shown over a
    level not ready); disabling the wait rule or the nesting rule fails the test.
- World volume, GPU side (world volume plan Task 5; `CloudWorldVolume`, `game/world/weather/cloud_world_volume.gd`;
  kernels `game/shaders/cloud_volume_bake.gdshaderinc`, lookups `game/shaders/cloud_volume.gdshaderinc`; fidelity
  flag `CloudLayer.world_volume` / global `cloud_world_volume`, default false until Task 13, on with
  `STARSIM_WORLD_VOLUME=1` or `--world_volume`; the march reads it from Task 7 on, below):
  - Textures per level (index 8: the transient spare of a re-frame, created with its first job, freed at the swap):
    density RGBA16F in 1/km (RG = rho, sigma_c of slot 0's snapshot, BA = slot 1's; `channel_of_slot(s)` = s: a
    slot's snapshot is replaced only without weight), light RGBA16F (Task 6, below), max-extinction mip R16F (1/km) and
    distance field R8 (blocks) at 64 x 64 x 32; ~1.07 GB (`texture_bytes()`).
  - Threads: CloudLayer plans each frame once the last batch was taken (held time and jumps kept), `submit()`s it;
    CloudEffect `run()`s it (with the pass's own camera) after the set 0 is built and before the march (GPU intervals
    `cloud_volume_upkeep`: the bake, max mip, distance field; `cloud_volume_light`: the light, Task 6), then
    adopts its lookup. `notify_view_jump` / `notify_clock_jump` / `show_now` restart the schedule (`forget()`). The
    cross-fade is gated: while the volume runs (`running()`: a batch adopted, the last one taken - frame-based), the
    weight moves onto a slot only once the adopted state holds the slot's snapshot in its channel pair
    (`slot_holds`). Stats: `CloudLayer.world_volume_stats()`.
  - Bake (`CLOUD_VOLUME_BAKE`): one 64-thread workgroup per cell, the taps shared and summed in shared memory in a fixed
    order (a rebake is bit-identical); the core's tap rule and point model of each snapshot (exact bilinear weights on
    the weather field like the CPU: the hardware's ~8-bit sub-texel weights put a 1-degree field ~1 % off); the cell
    box is the grid's own axes at its centre (e_x / e_y from the grid points half a cell away, e_z the normal x
    height); a snapshot whose field has no cloud in any texel the cell can read skips its taps (0 exactly).
    `same` slots bake once. A one-slot job (`snapshot_channel` 0 / 1) keeps the other pair.
  - Occupancy (`CLOUD_VOLUME_OCCUPANCY`): per baked box the mip blocks whose support (block + one-cell ring: the
    trilinear filter's reach) it touches get the bound max sigma_c (and rho) over the support where any rho > 0 in
    either snapshot, else 0 (x (1 + 2^-10): rounding up); then the whole level's toroidal Chebyshev distance (blocks,
    cap 8) to a block with a bound, separable x / y / z. Every block claimed free has no rho in its support: a sample
    of THAT level there reads 0 at any snapshot blend. The fade band blends level k + 1 (its trilinear support reaches
    ~3 level-k cells past its cloud), so the lookups take both levels there (Task 5 review I1):
    `cloud_volume_max_extinction` = max(level k's block bound, the bounds of every level k + 1 block it overlaps -
    `cloud_volume_up_blocks`, exact in 1/16 m: one block horizontally and from level 0 to 1, vertically level 1's
    200 m blocks straddle level 2's 656.25 m ones, two for ~30 %) when the block reaches the band,
    `cloud_volume_free_m` = min(level k's free ball, level k + 1's) when level k's free ball reaches the band (the free
    ball is clipped to the level's window). Level k + 1 is shown and holds level k's band (the schedule's nesting).
    Out of the shell the reads clamp, never wrap (review round 2, N2): levels 0-1 with a clamped bottom read their
    bottom cell below its centre (`cloud_volume_uvw`), the blocks clamp vertically as the reads
    (`cloud_volume_block_clamp`; the free space there is 0). Carry to Task 7: the sub-cell field must be 0 where trilinear rho = 0 and
    <= sigma_c elsewhere, else "dilated by the lift" does not hold - rerun the 1e4-point probe with the real
    per-sample density then. The distance field is recomputed whole for every level a batch touches (~0.05 ms each;
    up to 6 levels a frame at 7.8 km/s, `vr-performance.md`).
  - Grid numerics (preflight C22): `cloud_volume_grid_in(frame, offset)` and `cloud_volume_offset_of` work on the
    differences to the camera (geodetic fixed point for n - n_c, the AEQD by sinc / x-cosec series in theta^2, the
    camera's normal split hi / lo, a reference grid point the series reproduce exactly), adding the camera's grid point
    (hi + lo) last. Measured: 100 offsets with the frame 990 km away within 0.66e-3 of the containing cell (level 0:
    3 cm). `cloud_volume_level` skips levels of the other frame during a re-frame (the march needs both grids: Task 7).
  - Measured (cloud_check section 16, RTX 4090, Starbase, analog + ERA5 pair): cold start at the game's budget
    `cloud_volume_upkeep` median 1.34 / max 2.84 ms per frame, 4.3e7 taps/ms (5050 level-7 cells/ms); whole fills at
    40 x the budget 6.8-7.6e7 taps/ms. Per cell vs the CPU: water per weather cell (level 2) 0.9967-1.0005; single
    edge cells scatter (taps within the 8-bit filter error of the threshold or the fraction cut flip): up to 1.7 % of
    the cells beyond 1 %, all within 0.059 sigma_c. Occupancy (levels 0, 2, 5, cameras 2 m / 9 km): the distance
    field equals the ring-dilated reference on all 131072 blocks; 1e4 points in claimed-free blocks, a random point of
    each free ball: 0 with density (blend toward level k + 1 included). Fade band of every level 0-6 (each adjacent
    pair; levels 0-1 also at the window top), 1e4 points each: 0 with density in their free ball, 0 above the block
    bound (with the band ignored: 7-90 per level failed; with level k + 1's bound taken at the grid point's block only,
    as in fix round 1: level 1 failed with 6 / 1 points at the 2 m / 9 km cameras). Clamped bottom (levels 0-1, camera
    2 m): 1e4 columns read the bottom cell below its centre (before: 9960 / 9959 read the wrapped top storage cell).
- World volume light (world volume plan Task 6, review round 1 (controller ruling: alternative B, N 16); schedule
  `CloudWorldVolumeLight`, `game/world/weather/cloud_world_volume_light.gd`, called by the schedule; kernels
  `game/shaders/cloud_volume_light.gdshaderinc`, run by `CloudWorldVolume` after the bake, GPU interval
  `cloud_volume_light` with per level `cloud_volume_light_column_<k>` / `cloud_volume_light_cells_<k>`; read by the
  march from Task 7 on):
  - Texel (RGBA16F): tau_sun, tau_below (vertical, down to the shell's bottom: the transfer table's b), tau_above (up
    to the shell's top: the sky's source), tau_moon - optical depths through rho (cell-mean extinction, both snapshots
    blended by `cloud_blend`). Earth's shadow / Moon off: 6e4 (exp = 0; finite, so a filter across it stays finite).
  - No kernel reads any level's light (provisional light aside): only baked density. The levels are lit independently;
    one dispatch per level and kernel (jobs contiguous in the batch's buffer, each holding its first column / 4^3 block;
    a workgroup finds its job by binary search) - a dispatch per job serialised the jobs (one image written) at the
    latency of each job's longest ray.
  - `CLOUD_VOLUME_LIGHT_COLUMN` (a workgroup of 128 per column): tau_below / tau_above as exact trapezoid sums of the
    level's own column over its window (shared-memory prefix scan; the trilinear field along the geodetic vertical is
    linear between centres); beyond the window (levels 0-1) the coarser shown levels' density marched along the
    vertical, one midpoint sample per invocation above (up to the field's top) and one below (down to the shell's
    bottom), the grid-mean column if no coarser level holds them; levels 2-7 hold the outer half cell as the reads clamp.
  - `CLOUD_VOLUME_LIGHT` (a workgroup per 4^3 block, after a barrier): tau_sun / tau_moon by the distance-stepped march
    (a clipmap cone march): Earth's shadow when the straight ray meets the ground (Gaussian sphere of the grid); else
    from the cell centre CLOUD_LIGHT_LEVEL_CELLS = 32 cells of the cell's own level along the ray (a cell along the ray
    = the shorter of its horizontal and vertical crossing), then 32 cells of each coarser level, 2 midpoint samples per
    cell; a sample of segment j reads the finest level >= j whose window holds it (its own level, else a coarser one
    shown in either frame), faded toward the next coarser level as the march's density; the march ends at the shell's
    top or, lower, the field's cloud top (`environment::cloud_render_top_m` of both snapshots, carried in the job, plus
    1.5 of level 7's vertical cells of trilinear support: no density above, so it only shortens the march); beyond
    level 7's segment, or from a sample no window holds, the grid-mean column. Each segment's grid path is the
    quadratic through its exact grid points at its ends and middle (no geodetic transform per sample). Cost: a
    constant per level (2 x 32 samples x the levels on the path), shortened by the top exit. N = 32 (controller ruling,
    round 1c): with 16 a level-2 ray at a 15 deg Sun left level 2 after 3.3 km and read 400 m cells for the cumulus
    1.35-2 km up (p99 0.18).
  - `CLOUD_VOLUME_PROVISIONAL`: from the next coarser shown level's light at the cell centre, level 7 (or none shown)
    from the grid-mean column (tau_sun = tau_above x the path below the cloud top / the height to it).
  - Write modes: DIRECT (fresh cells; in spike frames - cold start, jump, re-frame - and for cells never shown with
    provisional light), RATE (refresh generations: each channel's exp(-tau) moves <= `LIGHT_STEP_T` 0.01 per pass, the
    step shortened by the half float's storage error; into or out of "no light" - Earth's shadow, the Moon off -
    directly; the GPU counts clamped cells per job, read back asynchronously, and a box that clamped any is served
    again once the level's queue is through - "settle" - until it clamps none), BLEND (a box SHOWN with provisional
    light outside a spike: the real light recomputed each plan and mixed in exp(-tau) by dt / the time left, linear
    over `PROVISIONAL_FADE_S` 0.25 s).
  - Schedule: light only for filled (`ready`) levels; per plan blends, fresh cells (coarse to fine), refresh
    generations one level after the other, fine to coarse (the nearest light first; equal shares over eight stale
    levels paid each dispatch's latency floor eight times a frame), provisional light once. Triggers: shadow shift
    H dtheta / sin^2 e < 1/4 cell, the Moon (Sun below -6 deg at the level's own centre), blend 0.02, and a coarser
    level turning drawn (M6: `light_epoch`, the coarser epochs summed - the rays read its density now). A level's
    refresh relights no other level. Jobs and refresh tiles are whole columns (32 x 32 x the storage height).
  - Cost model (`CloudWorldVolumeSchedule`, latency-aware, review round 1c): budgets in ms; a level's light lists
    cost its latency (`light_latency_of`, once a plan) plus its cells at the level's light-kernel rate plus its column
    cells at its column rate (`light_ms_of`). `feedback_split` learns them per level from the level's own GPU
    intervals over the last `LIGHT_COST_WINDOW` 45 samples: a dispatch of >= `LIGHT_CALIBRATION_MIN_CELLS` 65536 cells
    measures the light kernel's rate net of the latency (its time less the latency, taken as at least
    `LIGHT_MIN_THROUGHPUT_SHARE` 0.5 of it) and the column pass's as its column cells over its time; a smaller one
    measures the latency (its two lists' time less their units at the rates). Planned at the slow end: the rates'
    `LIGHT_RATE_QUANTILE` 0.1, the latency's `LIGHT_LATENCY_QUANTILE` 0.9, or the newest sample when it is worse (an
    overrun is adopted at once). Start: 1e6 cells/ms, 1e7 column cells/ms, latency `INITIAL_LIGHT_LATENCY_MS` 0.1 ms
    (measured: ~0.05 ms at level 7 to ~0.2-0.3 ms at levels 2-6, none at level 0, whose time is proportional to its
    cells); a level not measured yet takes the slowest measured level's cost. Growth (`light_cells_cap`): a level takes
    at most `LIGHT_GROWTH` 2 x its largest measured dispatch a plan, `LIGHT_PROBE_CELLS` 131072 before any - the model
    holds at the sizes it measured (a finer level's first jobs at the coarser levels' cost overran 2-3.5 x at N = 32);
    a cold start / jump starts the growth again from the probe (sizes measured on other content at another GPU load
    let level 0's first dispatches run 2 x over plan, 4.56 against 2.55 ms, in the full check).
    A level whose latency does not fit what is left of the budget is skipped and a level that fits is served (never
    an overrun) - but never for good (round 2, N1): where one whole column (and the latency) never fits the plan's
    whole light budget, the level is served partial columns (z slices of at least `LIGHT_PARTIAL_MIN_CELLS` 16 cells;
    the column pass is paid whole; `box_split_column_z`); a level whose light kernel no large dispatch measured for
    `LIGHT_PROBE_FRAMES` 45 plans while light is pending and whose share cannot reach `LIGHT_CALIBRATION_MIN_CELLS`
    gets a probe beyond its share (one level a plan, the longest unmeasured), a pair: first one small dispatch (one
    column, issued even when the stale latency estimate says it does not fit - the estimate is what is verified), which
    measures the latency again; once that sample arrived (or `LIGHT_PROBE_WAIT_FRAMES` 12 plans) the large one of that
    many cells, if the share still cannot reach it, bounded by the spike cap with the bake and the light both
    `BAKE_SPIKE_MARGIN` over their plan, which measures the rate. A level starved by a stale latency thus returns to
    normal service within ~2 probe periods (tested with 1.2 ms stale, 0.2 ms real); the probe leaves the level's
    growth bound untouched. A slow sample leaves the 45-sample window with new measurements (the latency quantile
    forgets a lone slow sample after ~10 faster ones).
    Note: at the measured rates (>= 1e4 cells/ms floor, column pass ~1e7 cells/ms) a level-0 column costs ~12 us, so
    the partial columns only act for slivers of rates far below today's; the latency path (probe) is the live one. Spike frames: the fill bake gets `SPIKE_BAKE_SHARE` 0.5 of the 3 ms cap, `SPIKE_BAKE_SHARE_LIGHT_DUE`
    0.2 while a filled level's light is due (provisional or blending: the cold start lights each level's whole window,
    up to ~70 ms of GPU at level 0 on the cloudy slab - beside a half-cap bake level 1 took 125 frames), and the light
    `LIGHT_SPIKE_SHARE` 0.85 of what the bake leaves (the bake's plan x 1.5). An upkeep over 2 x its plan halves the
    next plan once.
    - The bake's last work often lands after the light's first stamp (~0.2 ms in the first light list of a frame that
      baked - also in frames without light jobs; the sum bake + light is unchanged): the light's quantiles absorb it.
    - The test GPU's rates depend on its load (clock state): the cold start's small dispatches run at ~1e5 (level 0) to
      ~1e6 cells/ms, the 40 x settles at 0.4-5e6 - the model learns whichever holds.
  - Bake per level (review round 1): each level's bake jobs and the occupancy of the textures they touched in their own
    compute lists after a timestamp `cloud_volume_bake_<k>`; per-level rates in taps per ms (`bake_level_taps_per_ms`;
    the cost per tap differs between levels by up to ~10 x, occupancy included: 1.4-5e7 at levels 0-4, 0.5-1.8e8 at
    levels 5-7), each planned at the slowest of its last 45 measurements, an unmeasured level at the start rate 2e7.
  - Measured (cloud_check sections 17 / 18, RTX 4090; section 8's thick slab 1-2 km, tau ~10, and its broken cf 0.4
    field, at Starbase, camera 2 m; the reference: midpoint steps of a tenth of a cell through the finest level at
    each point):
    - Slab, Sun 45 deg: tau_sun worst 0.18 % (levels 0, 2, 5); tau_below / tau_above vs the CPU bake's column
      <= 0.17 %; tau_moon (Moon 40 deg) <= 0.22 %. Sun 10 deg: worst 1.87 % (the top cell 1.391 vs 1.393 at level 0,
      9.180 vs 9.192 at level 2, 10.039 vs 10.052 at level 5).
    - Broken field, exp(-tau_sun) error (4096 random cells per level inside the window, 0-2.5 km), p90 / p99:
      Sun 45 deg level 0 0.0000 / 0.020, level 2 0.0002 / 0.0021, level 5 0.0004 / 0.0020; Sun 15 deg level 0
      0.0001 / 0.50, level 2 0.0004 / 0.0060, level 5 0.0005 / 0.0028. Shadow edges (exp(-tau) = 1/2 crossings at
      500 m) 45 deg: levels 2 / 5 0.01 / 0.01 cells (level 0 none); 15 deg: level 0 3.8, level 2 0.03, level 5 0.02
      cells. Level 0's p99 is distant cloud-edge grazing at 50 m vs the coarser levels' resolution (Task 7's radiance
      check and `light_entry_samples` judge it).
    - Cold start at the game's budget (section 17): every level's light complete 17-39 frames after its density is
      shown (levels 7 .. 0: 18 / 23 / 18 / 19 / 17 / 19 / 33 / 39; asserted <= 45), upkeep max 2.60 ms alone, 5.04 ms in
      the full run. The cold start is a transition (T5 ruling c1: cold start / jumps run behind the load state under a
      raised budget): asserted <= `UPKEEP_TRANSITION_MS` 6 ms (2 x `UPKEEP_SPIKE_MS`); the 3 ms cap binds in-flight load
      (Sun motion, pair switch, scroll, re-frame; measured max 1.50 / 1.68 ms). The 5.04 ms frames came from a GPU
      clock-state change right after the bake ended (level 0 ran at half the rate the window had measured; the
      feedback arrives 2-3 frames late). The whole volume after 1255 frames. Light rates per level: `vr-performance.md`.
    - Earth's shadow (Sun 8 deg below the site): all 24.9 M texels of levels 0, 2, 5 with the Sun >= 6 deg below their
      horizon at 6e4. No NaN / infinity in any level after five fills. Sun 10 deg up: no Moon jobs. No sample of the
      march through the levels unheld (`light_fallbacks` 0).
  - The radiance checks deferred from Task 6 (C12 / C13) run with the march (Task 7, below: cloud_check section 19,
    `render_check --front_check`). The Moon's channel is read with the level's lookup `moon_weight` (below).
- World volume march (world volume plan Task 7; `game/shaders/cloud_volume_march.gdshaderinc`, kernel
  `CLOUD_VOLUME_FULL` compiled by `CloudEffect` with the march's files (one set 0) and the volume's lookups; GPU
  interval `cloud_volume_full`): with the flag on and a level shown (`CloudWorldVolume.march_uniforms`: the adopted
  state at the frame's camera in its own CloudVolume block) the frame is one ray per eye pixel over [0, scene], written
  into the eye's layer of the result (no temporal pass, no upsample: the full-resolution reference of Task 8's tiers);
  until a level is shown, and for the debug outputs it does not draw (3, 5, 6, 8-11), today's march. The curl texture
  (core `cloud_curl_field`: `CloudNoise.curl_rgb`, 32^3 RGB8, global `cloud_noise_curl`), the flags
  `CloudLayer.light_entry_samples` (4; global `cloud_light_entry_samples`) and `CloudLayer.edge_curl` (1.0; global
  `cloud_edge_curl`) are declared in `clouds.gdshaderinc` so every kernel of the march's files shares one set 0.
  - Steps (STOP-1 ruling): empty space by the free ball of the finest shown level (and of level k + 1 where the ball
    reaches k's fade band: then k's ball WITHOUT the window clip - beyond the window k + 1 alone holds the density -,
    so rays cross window edges in one step) or, further, the exit of the free box along the ray (horizontally linear /
    `CLOUD_VOLUME_MAX_SCALE`, vertically the exact crossings of the box's heights less `CLOUD_VOLUME_BOX_MARGIN` 0.4 %:
    rays grazing a base or a window top left the ball geometrically, p99 914 -> 157 empty steps at V1); occupied space
    one step to the exact exit of the level's cell (the cell picked with a 1e-4-cell nudge along the ray), capped by
    `CLOUD_STRUCTURE_TEXEL_FRACTION` of the finest sub-cell structure where the sub-cell field acts (a cell entered
    from clear or solid space whose sample shows the field acting is taken again from its start with the cap: one
    sample of a sharp field per cell drew the cells' faces as blocks; the first occupied step after an empty one is
    the structure's), >= 2 m; one density sample at the middle, Beer-Lambert over the step; the source linear between
    the step's ends in closed form, the ends' sources from light NODES `CLOUD_VOLUME_LIGHT_NODE_CELLS` 0.5 of the
    level's cell apart along the ray (or at a longer step's end), linear between them (Task 7 ruling R3: one light
    evaluation per half cell, not per structure step - V1 light fetches per occupied step 9.6 -> 0.46); silhouette
    bisection 4 x; termination as today (Sun-ray exception to 1e-9). The air shadow's loss is composited on every ray,
    with or without cloud on it (as `cloud_shade_ray`; skipping it on clear rays was V6_backlit's 34 % median). Error bound of the mid-step trilinear sample: the trilinear field along a ray inside a
    cell is a cubic, |tau_mid - tau| <= Delta L / 4 for a full-cell step (Delta the cell's corner spread).
  - Levels: the finest shown level whose window holds the point, each level in its own frame's grid during a
    re-frame (`cloud_volume_at`; cross-frame nesting and the spare's cross-fade are Task 9's); beyond the coarsest
    shown level the ray ends (Task 8 / 10: the far field).
  - Sub-cell field (decision 6; Task 7 ruling R1, the threshold form): density = sigma_c Phi((v - t) / s_e),
    t = -sqrt(s_e^2 + sigma_v^2) Phi^-1(c), c = rho / sigma_c (trilinear): the cell's cover kept as a cover at full
    in-cloud extinction (not spread into haze), graded (= the bake) where sigma_v -> 0, sharp where sigma_v >> s_e.
    v = w_s (s_f - s_cell) + P_g: the shape's residual above the cell's scale (s_cell at the cell's shape level, 0 below
    the 250 m texel; through a fade band the cell size mixed geometrically by the fade, ruling R5) plus the detail
    perturbation Gaussianised by the core's table (`CloudNoise.detail_gaussian`, the eroded Worley octaves are not
    Gaussian: with the Gaussian t alone the real cover was off by up to 0.053); sigma_v^2 = w_s^2 (R(l_cell) - R(l_f)) +
    sigma_P^2 from the core's statistics (`detail_level_mean / _variance`, `shape_residual_variance`; globals
    `cloud_detail_gaussian`, `cloud_detail_level_mean / _variance`, `cloud_shape_residual_variance_lo / _hi`);
    s_e = edge width / sqrt(20) (the smoothstep ramp's slope spread). Core test "The sub-cell threshold field keeps the
    cell's cover": Gaussian v: |mean - c| <= 3.3e-8 over c 0.05-0.95 and sigma_v / s_e 0-100; the real detail values
    Gaussianised 1.0e-3 / 1.2e-3 (states 3 / 1.3, 5e5 points: sampling noise 7e-4), with the real shape residual
    2.4e-3 / 3.6e-3. The curl distortion of the detail lookup (edge_curl x the level-0 detail's middle cell x w_0).
    0 where rho = 0, <= sigma_c, rho exactly with all weights 0. History (`docs/history/weather-clouds.md`): the
    spreading form sigma_c ramp(ramp^-1(c) + 4 c (1 - c) v / w) raised V3's alpha by 0.04 (water as haze).
  - Light per sample: the light volume's taus (faded as the density): the table / closed form with tau_sun and the
    columns, the sky with tau_above x the sky irradiance (refresh as today), the Moon's J x the levels' `moon_weight`
    (the schedule moves it toward the level's Moon flag over `MOON_RAMP_S` 1 s; rate-limited jobs leave tau_moon stale
    once the Moon is off). A tau of >= `CLOUD_VOLUME_UNLIT_TAU` (half the stored 6e4) is no light: J = 0 (the table's
    3D entry cosine is > 0 below the horizon). Surface correction (`light_entry_samples` 4) at each light node: where
    T (1 - e^-dtau) > 1e-3 and the cell is coarser than the footprint, tau_sun = 4 midpoints of the per-sample density
    over 2 cells (the shorter crossing) of the level whose light the sample reads (level k + 1 in the fade band) plus
    the volume's tau_sun there. (Ruling R3's "first 4 steps of the ray only" broke section 19's slab at Sun 20 deg:
    worst 11.8 % vs the converged light, bound 5 %; at the nodes 3.3 %.) `light_entry_samples` < 0 (checks): the
    converged per-sample light march to the shell's top.
  - Checks: cloud_check section 19 (`STARSIM_VOLUME_ONLY=19`); with `STARSIM_WORLD_VOLUME=1` the other sections'
    renders march the volume, filled and lit first (`_volume_settle_render`, 40 x the budget; a camera moved > 50 km is
    a view jump); `render_check --ab_reference / --edge_check / --front_check / --volume_fill_scale /
    --light_entry_samples / --edge_curl / --world_volume_unsplit` (header), the settle holding the volume's march
    (`CloudEffect.volume_march_hold`) until the scene and the volume are settled; `cloud_grain_check` with
    `STARSIM_WORLD_VOLUME=1` (ours: the volume, the reference: the reference path, grain at 1 px). Results: Task 7
    report (`vr-performance.md` "World volume march").
- Cell bake, CPU reference (world volume plan Task 4; the GPU bake of Task 5 is checked against it; `[weather]`
  `core/tests/test_cloud_volume_bake.cpp`):
  - `cloud_render_field.hpp` mirrors the GPU point model of ONE snapshot without the detail (`cloud_field` with the
    profile rule, flat-base cut, ice share, stratiform weight; `cloud_extinction_in_cell` state 0 and
    `cloud_extinction_prefiltered` without the sub-pixel jitter; tables read linearly in the fraction as the GPU's
    filter). Render axes, model-space height |M p| - a, the shader's latitude / longitude. `CloudShapeFrame` holds the
    three wrapped noise origins AND their anchor point (`anchor_earth_m`): the GPU differences positions from the
    camera / frame origin, so the origins alone do not place a point.
  - `cloud_volume_bake.hpp`: a cell = centre + three edge vectors (the grid's own axes can be passed); taps at the
    sub-box centres, rho = mean tap extinction, sigma_c = mean tap in-cloud extinction = grid mean / m (the extinction
    where the edge ramp is 1; grid mean / cf would let a cloud interior cell exceed it, rho <= sigma_c holds by
    construction). Tap rule (measured, ruling T4 Q2): a tap every 200 m across (4-8 per axis, 16 on the 6.4 km cells
    of level 7: with 8 there the weather-cell water missed by 0.3 %), every 5 m up (at least 16), the shape
    prefiltered (mips + prefilter tables) at the tap spacing beyond the 250 m texel. Taps per cell, levels 0..7: 256,
    256, 528, 528, 528, 2112, 2112, 8448 (levels 6 and 7 at lambda 0.68). The GPU bake's cost follows from these
    counts (each tap a full point evaluation: ~13 field fetches + 1-3 shape fetches).
  - Accuracy (cf 0.4 cumulus slab 1-2.5 km and cf 0.4 cirrus (ice) slab 8-10 km - the ice tile's 15.6 m vertical
    texel against the 5 m taps - vs the 16^3 stratified point mean at a 10 m footprint, 200 cells per level):
    levels 0-2 within 0.02 sigma_c per cell (cumulus max 0.006 / 0.005 / 0.011, cirrus 0.003 / 0.004 / 0.015);
    levels 3-7 are a deliberate band-limit simplification (fidelity note): the coarse cells keep the cover and water
    at weather-cell scale, not per cell - per-cell rms cumulus 0.004 / 0.008 / 0.004 / 0.017 / 0.009 sigma_c (max
    0.068 at level 6), cirrus 0.002 / 0.004 / 0.002 / 0.010 / 0.007 (bounds: 1.5 x rms, max 0.25). Per 0.25 deg
    weather cell the sum of the bakes / the dense point-model integral of the same cells (assigned by centre) is
    0.9985-1.0003 at every level 2-7 for the cumulus, the cf 0.95 deck and the cirrus slab (bound 0.996-1.004).
    The point model's own grid-mean water is kept only statistically over the 32 x 32 x 16 km shape tile: per
    0.25 deg weather cell it is 3-7 % off the grid mean (0.969 / 1.068 / 1.042 at cf 0.4 / 0.95 / 1; a 1.5 km slab
    samples one slice of the tile, +7 % over 1 deg at cf 0.4) - unchanged, not renormalised.
    Parent vs the volume-weighted finer cells (1 -> 2: the parts of the 50 m cells inside the 164 m parent): <= 0.014
    sigma_c on the dense tier, <= 0.051 on the coarse one (4 -> 5 and 6 -> 7 exactly 0: the same taps).
    With 4 x 4 taps at the mips' band limit (the spec's "few taps") levels 5-7 missed the dense mean by up to
    0.24 sigma_c and the weather-cell water by 3 %; 4 vertical taps missed level 2 by 0.1 sigma_c.
  - Binding (checks): `StarsimCore.weather_cloud_cell_bakes(snapshot_path, cells, shape_origins_earth_m) ->
    PackedVector2Array` ((rho, sigma_c) per cell; 12 doubles per cell: centre, e_x, e_y, e_z; origins 3 doubles = the
    anchor, or 12 = anchor + cumulus / stratiform / ice origins as uploaded); the snapshot's render field and the noise
    are kept per loaded snapshot (a file rewritten through StarsimCore - `weather_write_*`, `weather_build_snapshot`
    - is evicted from the load cache and baked anew; smoke_test checks it), cells baked on the build threads (64
    level-7 cells ~70 ms). Single cell:
    `weather_cloud_cell_bake(snapshot_path, centre_earth_m, cell_m: Vector2, shape_origins_earth_m) -> Vector2`
    (box east / north / up).

### World volume and view cache (open points)

- **World volume bake, coarse levels (deliberate band-limit simplification, ruling T4 Q2)**: cells of levels 3-7
  (400 m .. 6.4 km) keep the point model's cover and water at weather-cell scale, not per cell (per-cell scatter up to
  0.07 sigma_c, rms 0.017; Diagnostics "Cell bake"). The point model itself keeps the grid-mean water only over the
  shape tile (3-7 % off per 0.25 deg weather cell). The bake's tap counts (up to 8448 per level-7 cell) set the GPU
  bake's cost (Task 5 measures it).
- **Cloud view cache (in progress, `docs/superpowers/archive/specs/2026-10-27-cloud-view-cache-design.md`; Tasks 2-5 done,
  Task 6 VR acceptance open)**:
  `CloudViewCacheLayout` (cube map, tiles, 3 resolution levels, face size from the eyes' projection),
  `CloudViewCacheSchedule` (tile state, staleness, priority pick under a budget; `cloud_view_cache_test.gd`) and the
  fill (Task 3): `CloudViewCache` (`game/world/weather/cloud_view_cache.gd`) holds per level a cube texture of colour + T
  (RGBA16F) and one of the cloud distance (R32F) (3456 per face for the VR run: 0.86 GB full level, 1.13 GB all three;
  the fade slots 32 MB) and fills the texels of a tile list ((tile, level, slot) triples; one dispatch per level, push
  constants, the list sorted by level) with the kernel `CLOUD_CACHE_FILL` (`shaders/cloud_view_cache.gdshaderinc`: the
  march's code with `CLOUD_MARCH` + `CLOUD_CACHE_FILL`, its own images instead of the march's). Each texel's ray
  (`dir_of`'s mirror in GLSL) starts at the anchor - its offset from the camera in the fill's CloudView `eye_earth`
  (float64 difference on the CPU) - and ends at the ellipsoid / sea crossing seen from there, for an anchor below the
  ellipsoid (Starbase's ground lies ~25 m under it) at a sphere `CLOUD_CACHE_GROUND_BELOW_M` 2 m [E] under the anchor
  (before Task 4: no end - the downward rays marched the air and clouds beyond the limb, whose shadow loss then lay on
  the terrain), else the sky (1e30); never the frame's depth. The distance texel holds the alpha-weighted cloud distance,
  without cloud minus the ray's static end (at most 1e7 m). The texel angle at a face centre (2 / level face texels)
  is the detail's band limit. The march and the fill shade a ray with the one function `cloud_shade_ray`
  (`cloud_pass.gdshaderinc`: march, air shadow loss, debug outputs, the stored colour; with `cloud_ray_jitter`,
  `cloud_stored_color`, `cloud_eclipse_factors`); `cloud_march` and `CloudRay` take the ray's origin (`h_c`: its height).
  `cloud_ray_from` is origin-aware only in the fill (an anchor kilometres from the camera has its own height and up);
  the per-frame march keeps the camera's ray at compile time - a run-time branch on the offset let the GPU compiler
  re-associate the camera's path (4 % of the pixels changed, up to 30 % of the mean colour: the deterministic march's
  step grain flips with ulp-level changes). Main-thread bound of the schedule (controller ruling, tested as a maximum):
  <= 1.0 ms on every frame once level 0 is in view, a cold start / jump frame <= 10 ms - held by running the plan on a
  worker (fix round 1, below). The schedule is amortised all the same: the light rescan is a pass of
  `LIGHT_SCAN_BLOCKS` blocks per `mark_light` call (persistent cursor, `light_scan_pending()`); an anchor move starts an
  epoch (O(blocks)), tiles of older epochs are "off" (stale, parallax candidates) and counted exactly per block; `pick`
  expands at most `max_expansions` blocks per call (default `MAX_BLOCK_EXPANSIONS` 2; the planner asks for 1 = 64
  tiles), nearest to the view's cone first; `completed_batch` completes a pick's tiles and `set_tile_distances` takes a
  fill's read-back in one call each; `missing_in_cone` says whether a tile in a cone has no level yet.
  - **The frame (Task 4)**: the cache is the game's path (`CloudLayer.view_cache`, shader global `cloud_view_cache`,
    default on; off: the per-frame march at the preset's divisor - the reference, `render_check --no_view_cache`;
    cloud_check (but section 15) and cloud_far_check run the march; a debug output also shows the march).
    `CloudViewCachePlanner` (`cloud_view_cache_planner.gd`, owned by CloudLayer) plans on a worker (`WorkerThreadPool`,
    fix round 1: pick, completions and shown states cost 0.5-1.4 ms of GDScript a frame): the main thread (FrameProfiler
    section `cloud_view_cache`) hands the run the frame's inputs (camera, view, light, budget, the render thread's
    read-backs and fill times) and takes the last finished run's batch - one frame late; a run still going hands out
    nothing that frame. A run: the layout from the eyes' finest density (CloudEffect publishes 7 x 7 sample rays per eye
    and the face size every frame; at most `FACE_TEXELS_MAX` 4096 [E] for the 2 GB cap, re-created beyond a 10 % [E]
    change, built in the run), the anchor (the camera at a restart), the light state (Sun, weather weight and a
    generation that counts the light cache's resets and its first completion after one or a clock jump - tiles filled
    while it was still filling refresh once: they differed by whole tiles of 10-30 % in the ground view's A / B), up to
    `MAX_PICKS` 8 picks of one block each within `RUN_PICK_US` 3 ms [E] while the budget lasts, per tile the shown state
    the lookup reads (level, what it fades from, fade slot, the fade's start), and the covering flag. Restarts
    (everything back to nothing, level 0 of the view first): `World._view_jumped` (`notify_view_jump`),
    `SimClock.time_jumped` (`notify_clock_jump`) and a pinned snapshot (`show_now`); a requested restart hands out a
    reset batch at once (the frame shows the per-frame march until the restarted run's batch: never a tile of the old
    place or instant). A moving camera re-anchors progressively instead (Task 5, below). **Covering (controller ruling,
    no frame without clouds)**: while a tile in the view's cone (the sample rays' cone plus a tile radius;
    `missing_in_cone`) has no level, or a first fill there is still fading in, the batch says covering: the effect runs
    the per-frame march at the preset's divisor and the lookup keeps its result where a tile has nothing and fades a
    first fill in from it (after a start, a jump, a layout re-creation, tiles in view dropped by the re-anchor's
    eviction; ~20-45 frames, see `vr-performance.md`). Render thread (`CloudEffect._prepare_view_cache`,
    `_cache_lookup`): the batch's tiles filled (GPU timestamp `cloud_cache_fill_<fill id>`; a refresh at the same level
    first copies the tile's shown texels to its fade slot; each entry's nearest cloud distance by `atomicMin`, read back
    asynchronously), the shown states uploaded, while covering or with a cloud near the near shell's march (stamped
    `cloud_cache_march`, `+ _cover` while covering; fix round 1: covering runs the near shell's kernel - whole rays only
    where a tile has nothing), then per eye `CLOUD_CACHE_LOOKUP` into the pass's result (the composite unchanged; before
    the first batch and while a restart is pending the march alone shows the frame).
  - **The lookup (`cloud_cache_lookup(eye_offset_earth, dir_earth, out cloud_m)`, eye_offset_earth: the eye's offset
    from the head)**: the anchor's direction an eye ray reads is the root of r(q(s)) = |o + s dir| along the ray (o =
    eye - anchor, r the cached distance): two fixed-point steps, then secant steps while both ends saw cloud,
    `CACHE_PARALLAX_ITERATIONS` 3 [E] (fixed-point steps alone left 25 eye px of a deck at 300 m seen 50 m off the
    anchor). Each tile is reprojected from the anchor it was rendered from (Task 5): **one rule for the near shell and
    the lookup (`cloud_cache_pick`, fix round 1)** - the parallax lookup from the current anchor (the ring's offsets
    from the camera, float64 differences, in a buffer); where it lands on a tile of another anchor, again from that one,
    until the tile landed on shows the anchor solved from (`CACHE_ANCHOR_ITERATIONS` 3 [E]); a ray left unsettled lies
    where no tile holds its point (between tiles of different anchors: about |A - B| / r wide) - the near shell /
    covering march it whole and the lookup keeps that, a frame without them shows the last tile's edge (clamped). A deck
    at R_near seen from an anchor 100 / 200 / 300 m off lands within 1.2e-6 / 3.8e-4 / 0.011 eye px. The colour: the
    tile's finest shown level, bilinear in the cube, clamped to the tile's outer texel centres on a side whose neighbour
    lacks that level; cross-faded over `CACHE_CROSSFADE_S` 0.25 s [E] from nothing (T = 1, a first fill), the coarser
    level (a refinement) or the fade slot (a refresh: one Sun state per tile, never an abrupt switch; 1024 slots [E], a
    slot still in use ends that fade). A tile without a level shows the covering march's result (T = 1 when not
    covering; `cloud_cache_sample` returns the colour without the fade from nothing and its weight, `cloud_cache_lookup`
    mixes it with T = 1). What a tile fades from was rendered from its previous anchor: it is read at the same point of
    the eye ray seen from that anchor (the state names both ring entries: bits 18-24 / 25-31; the fade slot bits 8-17);
    the bilinear tap is clamped also where the neighbour shows another anchor. Scene geometry nearer than the cloud
    hides it by exp(-4 |ln(cloud / scene)|) (the upsample's depth weight); a texel without cloud keeps the air's shadow
    loss scaled by scene / the ray's end (a hill before the horizon).
  - **Budget (`CloudLayer.cache_budget_ms`, PRESETS `cache_budget_ms` / `cache_budget_floor_ms` / `frame_target_ms`)**:
    desktop 9.0 ms [E] fixed (one full-level deck tile alone takes up to ~8.4 ms and a fill's time is its slowest ray's;
    the ruled fill bound p99 <= 10 ms); VR adaptive (user-approved ruling): 22.2 ms - the rest of the frame's GPU time
    (95th percentile of 90 frames of the cache shown; a frame's rest = the span of its captured timestamps minus its own
    fill and its march with the cache shown, from the same frame's stamps) - the near shell where it is predicted (fix
    round 1: it runs in the last measured frame or a cloud is within its reach; the largest of its last 45 non-covering
    measurements, `CACHE_MARCH_FRAMES` [E]; never measured: the floor; a covering frame's whole rays are not predicted -
    the fill ends them, Task 4's ruling) - 2 ms, between 2 and 11.4 ms [E]. **Cost model (fix round 1)**: a fill's GPU
    time is set first by its slowest ray, then by its work - measured on the desktop deck view: a few tiles of the deck
    seen from above ~8 ms + ~6 us per work group of 8 x 8 texels, clear sky ~0.6 ms + ~0.3 us (both ~ latency x (1 +
    groups / ~1300)); a fill of 8 groups took as long as one of 1000. So each tile has a latency per level and a fill is
    predicted as max l_i + sum g_i l_i / `FILL_CAPACITY_GROUPS` 1024 [E]; a measured fill (its timestamp named by the
    batch's fill id: a frame whose stamps were not read loses only its own measurement) scales its tiles' latencies by
    measured / predicted; a level not yet measured takes the coarser one's times the level's learned ratio (median of
    the recent fills' medians; a finer level marches more detail), a tile never measured the largest of the recent
    fills' slowest at that level (`UNKNOWN_LATENCY_MS` 9 ms [E] before any). Tiles join a fill in priority order while
    the prediction fits the budget and the fill stays within `MAX_FILL_GROUPS` 1024 [E] work groups (larger fills were
    mispredicted by up to 3 x); the first tile always, and when it alone exceeds the budget the tiles that lengthen its
    fill by at most `FREE_RIDE` 1.1 [E]. `STARSIM_CACHE_FILL_TRACE=1` prints every measured fill (ms, prediction,
    levels, latency, budget). A single tile cannot be split: its slowest ray is the fill's floor (desktop deck view: one
    full-level deck tile up to ~8.4 ms; fills at the 9 ms budget p99 9.96 / 8.90 ms deck / ground).
  - **Near shell and progressive re-anchor (Task 5)**: R_near = `CloudLayer.near_radius_m()` (`NEAR_RADIUS_M` 2000 m
    [E], from the parallax error: a deck at R_near lands within 1.2e-6 eye px from the re-anchor's 100 m, 3.8e-4 px from
    an older anchor 200 m off and 0.011 px from 300 m - `cloud_vr_stereo_test`). The fill's rays start at R_near from
    their anchor (`CloudViewCache.fill` start_m; `cloud_march_start_m`) - the air shadow's loss is still taken over the
    whole ray, the near stretch in front of the cache's clouds at weight 1. **Near shell**: CloudLayer asks the core for
    the field's nearest cloud every frame (`StarsimCore.weather_cloud_nearest_m` ->
    `environment::cloud_render_nearest_m`: a lower bound - the nodes within reach plus the bilinear support that hold
    water, their cloudy height interval between the layer centres below and above; the active pair, both snapshots
    whatever their blend; 0 while the core's pair is not the slots') within R_near + the keep cap (1 km, below) +
    `NEAR_SHELL_MARGIN_M` 100 m [E]; the render thread runs the near shell when it is within R_near + the farthest shown
    anchor's offset + the margin, or the batch covers the view (`CacheMode.NEAR_SHELL`): the march kernel `CLOUD_MARCH`
    + `CLOUD_NEAR_SHELL` (variants centre / prefilter; no outline pass) at the preset's divisor marches each ray with
    the full march's own step layout up to its exit from the sphere of R_near around the anchor `cloud_cache_pick` names
    for the ray (`cloud_near_shell_end`: the tile states, the anchor ring and the cached distances bound at 8 / 9 /
    10-12, push constants; a tile without a level or an unsettled ray: the whole ray), and of the air loss only (1 -
    T_near) x its part in front of the near clouds (`cloud_shade_near`): composited by the lookup as near + T_near x
    cache, the near stretch's loss is front + T_near x behind, as the full march's. A first fill shows at once under the
    near shell (the march's rays end at the shell: no whole-ray result to fade from). The near kernel's push constant
    needs a list of its own (the RenderingDevice re-applies a list's push constant to the kernels bound after it).
    **Re-anchor** (`CloudViewCachePlanner`): once the camera is `REANCHOR_FRACTION` 0.05 [E] x R_near = 100 m from the
    anchor (`CloudViewCacheSchedule.reanchor_needed`) it becomes the anchor, the next entry of a ring of `ANCHOR_RING`
    128 [E]; every tile keeps what it shows from its own anchor (no restart, no tile dropped) until the schedule refills
    it from the new one (its parallax class by error x nearness - a tile without a known cloud joins it only once its
    anchor is `SKY_REFRESH_FRACTION` 0.5 [E] of its keep distance off, counted at the cap's distance, and only while a
    cloud is in the near shell's reach - and the staleness classes; blocks with tiles missing are expanded before those
    with a parallax only). **Refreshes because of motion at level 0 (fix round 2, controller ruling)**: a tile picked by
    the parallax class or rendered from another anchor is refilled at level 0 (1/16 of a full tile's rays) and refined
    by the finer-level classes from what the budget leaves; it fades from the finer level it showed (that level's texels
    stay until the tile is refined to it again: no fade slot); light / weather refreshes keep their level (fade slot).
    **Eviction by a tile's own parallax (fix round 1)**: a tile is dropped once the camera is farther from its anchor
    than `CloudViewCacheSchedule.keep_distance_m` = `ANCHOR_EVICT_RATIO` 0.1 [E] x its nearest cached cloud distance
    (200 m for a deck at R_near: the deck passage's measured best), at most `ANCHOR_KEEP_FRACTION` 0.5 [E] x R_near = 1
    km while a cloud is within the near shell's reach of the camera (`set_cloud_in_reach`, from CloudLayer's nearest
    cloud; fix round 2: the camera then stays inside the anchor's sphere). With no cloud in reach the near shell does
    not run and there is no cap: a clear tile (no cloud along its rays) needs no refresh for the camera's motion and is
    kept until its ring entry is reused. The planner keeps a tile list and the nearest cloud per ring entry and scans an
    entry only when the camera may have left its nearest one's keep distance, then hands the schedule the largest kept
    offset (`set_anchor_deviation`); the tiles of a ring entry about to be reused are dropped too
    (`CloudViewCacheSchedule.forget`; the near shell's kernel marches them whole in view). The light cache's reach stays
    camera-centred: with the anchor within 100 m of the camera (tiles of older anchors within 1 km while a cloud is
    near) its cascades (kilometres) hold the anchor's rays as the camera's - not tested separately.
    **Lookup under the near shell (fix round 2)**: the lookup keeps the near kernel's result alone exactly where that
    kernel marched the whole ray (`cloud_cache_pick` unsettled or no level); a first fill (fade weight 0 in the frame it
    is shown) is composited behind the shell's segment - keeping the segment alone dropped the clouds beyond R_near in
    the frames after a restart (`--jump_check` deck 0.23 / ground 0.17 since c625926).
  - **Validation and open (Task 5, fix round 2)**: `cloud_view_cache_test` (the re-anchor at 5 % of R_near, not
    below; through the planner a re-anchor restarts nothing, every tile that showed a level keeps one, refills take the
    new anchor; eviction by a tile's own parallax: a cloud at R_near / 4 / 20 R_near and clear sky kept at 5 / 15 / 45 /
    55 % of R_near off as [all] / [not R_near] / [only 20 R_near and sky] / [none], the schedule's bound the largest
    kept offset; clear sky joins the parallax class only past 500 m; motion refreshes at level 0, light refreshes at
    their level, a refresh fades from level 2 and the tile converges again; no cloud in reach: a clear tile kept 10 km
    off, a cloud at 20 R_near dropped by its parallax, no clear-sky refresh), `cloud_vr_stereo_test` (a tile reprojected
    from its own anchor 100 / 200 m off: a deck at R_near within 1.2e-6 / 3.8e-4 eye px, 1 km off at 5 R_near 3.8e-4;
    the pick across two / three anchors' tiles: every settled ray read from the anchor of the tile it reads within
    1e-6 px, the unsettled ones only where no tile holds the point), core `[weather]` "The render field's nearest
    cloud", cloud_check with `STARSIM_MARCH_REFERENCE` (the per-frame march bit for bit as before Task 5),
    `render_check --jump_check` PASS (deck 0.019 / 0.011, ground 0.012 / 0.006). `render_check --frames_every=0.5
    --frames_for=120` (the Starbase ascent through a slab 1.5-2.5 km, `--fly=25`, VR preset at render scale 1.65;
    the alpha bound 0.06 while the camera is within a layer's extent +- R_near, else 0.02; the first frame after each
    sample's reference march not counted; per phase: pad / in the layer / outside it): watched from the ground PASS
    (worst 0.012, GPU p99 15.3 / 14.9 ms); **on board FAIL**: GPU p99 pad 15.9 / in the layer 25.3 / above the deck
    23.7 ms (target 22.2), the deck passage's alpha worst 0.26 (bound 0.06). Over-target frames are the near shell's
    march (in the deck 11 ms mean, up to 15; above the deck covering 9 ms) plus the scene (~9.6 ms), not the fill (the
    budget sits at its 2 ms floor in the deck): the frame does not hold the near shell / covering march and a refresh
    rate that keeps the view's tiles on an anchor moving 150 m/s - open, a ruling for the controller. The re-anchor
    timing case of `cloud_view_cache_test` (pick <= 1.0 ms a frame) is flaky at 0.97-1.13 ms since the level-0 refreshes
    add the refinement frames (509 instead of 213 frames; ~0.35-0.45 ms per classified block, unchanged). The
    main-thread section `cloud_view_cache` over the 5400 flight frames: median 0.056 / p99 0.079 ms.
  - **Open (Task 4)**: the coarse levels show slightly less cloud than the full one (deck view: coverage 1.9 % below
    the march's for ~2 s after a restart, inside the 0.02 bound); near objects in front of shadowed air get the air
    loss of the whole cached ray scaled by distance only (with the near shell: a cloud texel hidden by nearer geometry
    drops the near stretch's loss too); the lookup's parallax is GPU-tested only at the anchor (the GDScript mirror
    covers off-anchor eyes).

## Removed 2026-10-30 (cleanup): the old march, F7 / F9, the light cache, the old shadow maps

The user ruled the baked volume the only cloud method (2026-10-30); the last state with the old march is the tag `clouds-old-march-last` (fe5edde). The sections below are the subsystem file's text of that state, verbatim.

### Model

- **Light**: the Sun's optical depth toward the light tau_light comes from the **light cache** (static clouds Tasks
  1-2; `CloudLightCache`, `CloudLightCacheSchedule`, `shaders/cloud_light_cache.gdshaderinc`): a x2 clipmap of 5
  cascades (8 / 16 / 32 / 64 / 128 km across, each 128 x 128 x 512 voxels over the shell -0.5 .. 20 km: 62.5 .. 1000 m
  across, 40 m up everywhere), R16F, two generation slots each (10 3D textures, 160 MB; the cache's VRAM budget 192 MB
  [E], controller ruling 2026-10-26), all on one Earth-fixed frame (the tangent plane at a ground point; re-framed
  after 1000 km or on any invalidation). Global voxel indices never move; each cascade's window is stored toroidally
  and re-anchored per axis when the camera's ground point is > 20 voxels from its centre - only the uncovered strip is
  refilled. The `CLOUD_LIGHT_FILL` kernel evaluates `cloud_light_optical_depth_with` at each voxel centre (96 near +
  32 far samples at the stratified midpoints, deterministic; detail band-limited at the voxel's size; the point solved
  for the march's exact model height; the field mixed at the generation's blend). The fill is progressive: the
  schedule hands the render thread exactly min(98 304, what is missing) voxels of jobs per frame (<= 0.5 ms GPU,
  measured below; whole layers, rows or part of a row of a strip), a full refresh of all 42 M voxels takes 427 frames
  (4.7 s at 90 Hz). A slot's held part is a rectangle growing strip by strip: a 16 x 16-voxel square around the ground
  point first (every cascade holds its ground point after 6 frames of a first fill), then strips <= 16 voxels wide
  along the side nearest the ground point; a strip in progress keeps its finished layers through a re-anchor; cascade
  k starts each strip at layer k x 512 / 5
  (wrapping), so the cascades of a first fill do not reach the costly in-cloud layers in the same frames. Each
  frame's budget goes first to the shown slots' missing parts, shared fairly (a cascade needing less than an equal
  share gets all of it, the rest split the remainder), then to fading-out slots, then to the one generation being
  built. Refresh: the Sun turned by 0.25 deg or the blend moved by 0.05 -> one cascade at a time (the stalest first,
  its shown slot complete) fills its other slot completely while the old one stays shown alone, then the lookup
  cross-fades over 0.5 s; a front that has not received a voxel takes the new light at once, a partly filled one is
  finished first. A clock jump (`CloudLayer.notify_clock_jump`, `SimClock.time_jumped`) or a change of quality /
  detail / debug output / noise / the content of a slot with weight invalidates everything: the march takes over until
  filled. A cascade whose shown light is more than 25 deg off the Sun drops it (a fading-out slot ends its fade, the
  front gives way to the generation in progress - shown as far as filled - or to an empty slot). Staleness bound
  (`cloud_light_cache_test.gd`): a light is shown until its cascade's next one has faded in, at most two full refreshes
  and a fade (~10 s): at 100x warp every cascade's shown light within 2.2-3.6 deg of the Sun (bound 4.4 deg), its
  ground point held in every frame; at 1000x (~42 deg unbounded) the 25-deg drop holds it (24.6-25.0 deg, ground
  point held in every frame). Flight: at 1 and 2 km/s every cascade keeps its ground point; at 7.8 km/s the
  re-anchor slabs of all cascades need ~1.8x the budget - cascades 2-4 keep it, 0 and 1 fall behind (held 6 / 53 % of
  the frames; the next cascade or the march takes their samples). Lookup (`cloud_cached_light_depth` -> (weighted
  cached tau, weight of the hybrid cascades, weight left) and the
  hybrid cascades' weights):
  cascade k has weight 1 within 34 voxels of the ground point (Chebyshev), falling to 0 at 42 (a band of 8 voxels:
  0.5 .. 8 km), the rest goes to the next cascade and after the last (beyond 34-42 km) to the per-sample march. The
  cache thus covers the volume march's full range (the near / far hand-over ends it at 34 km, static clouds Task 8). Cascade 0
  is read at the sample; cascades 1-4 are hybrid (controller ruling 2026-10-26): the sample marches the first 1 / 3 /
  2 / 2 km (cascades 1 / 2 / 3 / 4) of its light path through the shapes and adds the cascade's tau read where that
  march ends (the fill's tau starts there, so nothing is counted twice) - a far cascade's voxel can hold neither a
  cumulus' self-shadowing nor the sharp shadow of its neighbours. `cloud_near_light_depth`: one march per sample for
  all its hybrid cascades, each weighted with its own partial sum; 12 geometric intervals over the first km from the
  preset's first interval, then 100 m each (12 / 32 / 22 / 22 samples), the stratified midpoints. A clear-air
  early-out (stop where the field's column code above the sample is 0 at both ends of the segment) was exact on case 8
  but cost more than it saved (`vr-performance.md`) and is not used. The per-sample march
  (`cloud_light_optical_depth`: near samples over 6 km in geometric intervals from 20 m through the shapes, then far
  samples to the cloud top through the grid mean; stratified, unbiased over the jitter) - no seam; the weights depend
  only on the ground point, never on a re-anchor. The Moon always marches. Where the cache holds nothing the result is
  the march's bit for bit (one call site). Threads: `CloudLayer` plans every frame (main thread: the schedule's jobs and
  the lookup state they lead to, one batch; a batch with jobs not yet run holds the schedule back); `CloudEffect` runs
  the batch's jobs before the march and only then adopts its state (`CloudShaderPort.pack` overrides
  `cloud_light_cache_valid / _origin / _east / _north / _shell / _cascade0..4`). Fidelity flag
  `CloudLayer.light_cache_enabled` (global `cloud_light_cache_enabled`; off = the per-sample march everywhere, the
  reference; the cache keeps planning, so it is current when switched back on). The closed-form source (the fidelity
  flag's CLOSED_FORM) is the core's `cloud_scattering_source_3d`: TMS on the delta-Eddington source function of the
  surface the light entered through (entry cosine mu_e = clamp((t_v + mu0) / (tau_s + 1), max(mu0, 0.02), 1); a slab
  gives mu_e = mu0 exactly, a sunlit flank or a base under a Sun below the local horizon gets its true incidence; tilted
  independent column, Varnai & Davies 1999). The light march band-limits the detail at the larger of the pixel footprint
  and its interval. Plus the sky source (`cloud_sky_scattering_source`, vertical column). Column depths use one
  convention in core and shader: the covered area's in-cloud mean mean / cf (the A channel's), continuous through the
  layer centres. The air's Sun / sky irradiance at the cloud from `atmosphere.gdshaderinc`, re-evaluated every 500 m / 5
  % along a ray; the eclipse acts on both: the Sun through `atm_sunlit`, the sky light through `atm_sky_eclipse_factor`,
  evaluated per 8 x 8 work group at 5 km and 150 km along its centre ray and interpolated in distance (per sample it
  cost 10 ms at the eclipse epoch; the penumbra changes by ~3 % per 100 km).
- **Cloud pass** (`CloudEffect`, a compositor effect before the transparent pass; kernels in
  `shaders/cloud_pass.gdshaderinc` with `atmosphere.gdshaderinc` + `clouds.gdshaderinc` compiled as GLSL compute by
  `CloudShaderPort`, which turns their `global uniform`s into one uniform block + sampler bindings filled from
  `ShaderGlobals` - every script sets globals through `ShaderGlobals.set_value`, since
  `global_shader_parameter_get` is editor-only):
  1. raymarch at 1/2 resolution (desktop) or, in VR, once in a centre view covering both eyes at 1/3 (the shell
     -0.5..20 km to the scene depth, in VR read from the eyes' depth buffers; the block's depth in a checkerboard
     of nearest / farthest, or the block's mean when its four distances lie within 5 % (`CLOUD_ONE_SURFACE_SPREAD`
     [E]: one surface - the ground seen from far, where the nearest depth cut every other ray short of the low clouds
     by the block's depth extent, several km from 8 600 km; Task 10); coarse steps outside the field's cloudy region,
     fine steps inside. **Deterministic march** (fidelity flag `CloudLayer.deterministic_march`, shader global
     `cloud_deterministic_march`, default on; static clouds Task 3): an unchanged view gives a bit-identical image -
     the first step ends on an Earth-fixed height shell (spacing: the first coarse step's height extent, |dh/ds| at
     least `CLOUD_PHASE_MIN_CLIMB` 1e-3 [E]), every sample - the view's, the Moon's light march, the Sun's where no
     cache holds it - at its step's midpoint; a clear sample whose coarse step ends in the field's cloudy region ends
     the step there and marches on with fine steps (a fixed point skipped up to a coarse step of a cloud top or base
     in every frame); where a sample's extinction crosses `CLOUD_SILHOUETTE_EXTINCTION_PER_M` 1e-4 /m [E] from below,
     the entry is bisected `CLOUD_SILHOUETTE_BISECTIONS` 4 [E] times between the step's start and the sample and the
     in-cloud steps start there (the clear stretch skipped holds tau < 0.1); the detail is band-limited at the larger
     of the pixel footprint and `CLOUD_DETAIL_STEP_FRACTION` 0.5 [E] x the step (a level enters from one step per
     cell: finer detail at fixed points aliased into static salt-and-pepper holes in near cumuli). **Sub-samples**
     (static clouds Task 5c, fix round 1): a step longer than the structure step (`cloud_structure_step`:
     `CLOUD_STRUCTURE_TEXEL_FRACTION` 0.25 [E] of the distance over which the ray crosses one texel of the finest shape
     in the cell - cumulus 250 m across / 125 m high, stratiform 1 km / 125 m, cirrus 250 m / 16 m; a horizontal ray
     through cumulus every 62.5 m) is ceil(step / structure step) equal sub-steps, at most `CLOUD_MAX_SUBSTEPS` 6 [E]
     (longer steps: 1/6 of the step), each sampled at its midpoint in its own field cell (a thin cirrus sheet changes
     over a slant step: the step's cell lost 4 % of its alpha); the step's optical depth is their sum, its mean
     extinction that over the step (the step-weight test). The detail is band-limited at the larger of the pixel
     footprint and `CLOUD_DETAIL_STEP_FRACTION` x min(step, the structure step or step / 8) - continuous in the step
     length (at the sub-step itself it halved where a step crossed the structure step: a contour on near cumuli). One
     light sample per step at the sub-samples' extinction-weighted centre (also the step's distance for the air light).
     A silhouette is bisected from the first sub-sample over the threshold (between it and the one before); the thin
     cloud in front of it (the sub-steps taken and the stretch to the entry, at the bracket's clear end's extinction) is
     carried into the next step's optical depth if all of it is below the threshold - for a single-sample step too;
     cloud the ray left and re-enters within one step ends that step at the entry with its own light (carried, it
     darkened the next step's light into black specks). One point per step hit or skipped thin
     and broken cloud on long oblique paths with each ray's fixed step phase - static salt-and-pepper at the march pixel
     (the user's t7_cape view) that the jittered march's history had averaged away, gone with 8 x the steps; and the
     sub-threshold thin cloud before every entry was dropped. Not for the grid-mean debug outputs (1 / 2 / 6) nor the
     jittered march. The jittered march
     (the reference path, flag off): blue-noise start jitter, 64 x 64 void-and-cluster mask offset per frame by the
     golden ratio - the same for both eyes, the light march's jitter decorrelated per step; background through
     exp(-tau), scattered light through exp(-(1 - f) tau); colour = T_c L + (1 - T_bg) S_c with the air at the
     transmittance-weighted cloud distance; a step whose weight in the scattered light T_sc (1 - exp(-(1 - f) tau))
     exceeds 0.15 [E] (0.3 tried in Task 10, reverted) is marched again with steps of that weight (one sample deciding a whole deep step was the grain
     at cloud surfaces and at bases seen at a grazing angle; the step's intended length is compared, not the float32
     difference of the distances, whose 1 m ulp at 8 600 km retried the same step until the budget ran out - Task 10);
  1b. **outline pass** (static clouds Task 5b, fix round 1; the deterministic march only, not the debug outputs), two
     kernels. `CLOUD_OUTLINE` (one invocation per march pixel) copies the march's image (binding 6) into the centre's
     `refined` image (the temporal kernel takes it instead) and appends every pixel whose alpha differs from one of
     its 8 neighbours' by more than `CLOUD_EDGE_ALPHA_STEP` 0.25 [E] to the outline list (a storage buffer: count +
     packed pixels, the count reset by the CPU each frame). The march kernel with `CLOUD_EDGE_REFINE` (`CloudEffect`:
     its `_refine` variants) then runs one invocation per list entry (a direct 2-D dispatch for a full list - width capped at 65535 groups, the D3D12 limit per dimension; groups past
     the count end at once - the indirect dispatch over a GPU-written group count read the reset value on D3D12): full
     waves of outline pixels, which spread over the screen's work groups had kept whole groups waiting. Each entry
     gets 3 coverage-only rays (`cloud_march(..., coverage_only)`: the same steps and transmittance without light
     samples and air light, ended at `CLOUD_COVERAGE_MIN_TRANSMITTANCE` 0.01 [E]: alpha within 0.01 of the full
     march's) at fixed rotated-grid offsets that, with the centre ray, take four distinct eighths of a march pixel in x
     and in y ({-3, 0, 1, 2}/8) and average to the pixel centre in both axes (centre 0, (-3, 1), (1, 2), (2, -3) / 8
     march pixel: no shift of the outline). Alpha: the mean of the centre's and the
     three; the cloud distance alpha-weighted (the centre's from the march's depth image); the scene distance the
     block's as before. Colour: the centre ray's colour per alpha scaled to that alpha, or - where the centre's alpha
     is below `CLOUD_EDGE_COLOUR_ALPHA_RATIO` 0.5 [E] of the pixel's (a rim it barely touches) - the most opaque 3 x 3
     neighbour's (alpha > 0.25 on any outline). The binary edge of one centre ray per march pixel made outlines follow
     the march grid (stairs and beads of a march pixel that the Catmull-Rom upsample only rounded); now an outline
     pixel holds its covered fraction in quarters - smooth along the edge, still one march pixel wide across it
     (crisp), the same in every frame (no jitter, no noise pattern; the list's order varies, each entry's result does
     not). Cost: the VR proxy's cloud pass +26-46 % over the medians before Task 5b (`vr-performance.md`; was ~3x
     with four full marches per outline pixel in place; inside the accepted frame-rate cost, see the standing
     decisions in Open points);
  2. temporal accumulation per eye: reprojection of the cloud distance (sky: rotation only) with the camera's float64
     motion into the previous frame's clip space of the same eye, history clamped to the 3 x 3 neighbourhood; a running
     mean of weight 1 / n (n the frames the history holds, kept per pixel) down to 1/32 (an exponential mean of 0.1
     kept 23 % of one frame's noise); a clamped history keeps at most 4 frames. `World` resets the history on view and
     clock jumps (`_view_jumped`, `SimClock.time_jumped`). With the deterministic march the weight is 1: the kernel
     copies the frame (nothing to average);
  3. depth-aware upsampling (bilinear x exp(-4 |ln(D / D_i)|)) into a full-resolution array (layer = view); a VR eye's
     tent gather has the largest radius 2 with the jittered march (`CloudEffect.TENT_RADIUS_*`, the centre block's
     params.z). With the deterministic march the final gather (eyes and desktop) is a 4 x 4 Catmull-Rom filter (the
     centre block's params.w; depth weights as above, T clamped to [0, 1], colour >= 0; T >= 0.5 from 1 - the gathered
     alpha, below from the gathered T - the normalised sum of T = 1 taps with cancelling lobes flickered one half-float
     step below 1 on the moving sea, fix round 1 of Task 9): bilinear left 1-march-pixel
     stair steps on the binary cumulus outlines, Catmull-Rom rounds their corners at no march cost (Task 3 fix round 1).
     Debug outputs stay bilinear (the checks' quantities: the filter's overshoot raised section 8's worst pixels at
     dark silhouettes from 12 to 38 %). Where the taps' depth weights differ by more than a factor 2
     (`CLOUD_DEPTH_MIXED_WEIGHT` [E]: two surfaces, e.g. the march's nearest / farthest checkerboard along the
     terrain's horizon seen from altitude) only the kernel's positive part is used (static clouds Task 8 fix round 1):
     the negative lobes, renormalised over the taps of one depth, alternated with the checkerboard and dotted the
     horizon row with dark and bright pixels.
  The composite quad (`cloud_layer.gdshader` x T, its next pass `cloud_layer_light.gdshader` + the colour: first in the
  transparent pass, premultiplied; the images' .a is T, see the Sun's disk) blends it over the scene;
  plumes draw over it, MSAA resolves stay intact. Off beyond 1.5e7 m (scaled space: the far field below). A new size
  (resize, divisor) creates new images; the main thread shows the new result before the next frame (`CloudEffect.swap_output`:
  replacing it inside the render callback broke the composite's material for a frame), the old images are freed three
  render callbacks later (no renderer error, no frame without clouds). `CloudLayer.shutdown` frees the resident volumes.
- **Near / far hand-over in the cloud pass** (static clouds Task 8; `cloud_pass.gdshaderinc` `cloud_march` (two
  passes), `cloud_far_slab` / `cloud_far_column`; `CloudLayer.PRESETS` `handover_m` -> globals
  `cloud_handover_near_m` / `_far_m`, both presets 20 / 34 km [E]): the march ends at 34 km - inside the light cache's
  full weight (Task 2) and below the footprints that need the shape prefilter (VR 0.18 km at 34 km, f_s 3.5 km: the
  far cumulus' cover no longer beats with the 32 km tile, Task 3 follow-up). The far tier takes the ray from 20 km to the scene or the shell; over the
  band the far tier's weight w(d) = smoothstep(20 km, 34 km, d) (an empty band: a step; both beyond the scene: the
  march alone, 0 / 0: the far tier alone - the checks' switches, render_check `--handover`):
  - the march weights every step's share of the pixel (its light and its alpha after the clouds in front of it) by
    1 - w at the step's distance; a faded opacity per step compounded over the many steps inside one cloud like a
    faded density (a thick cloud stayed opaque to the band's end);
  - the far tier is a second pass of the same steps (`cloud_march` pass 1, over [20 km, end]; its steps weighted by w):
    the march's machinery - coarse steps spanning the segment, fine steps in the field's cloudy region that still
    reach its end within the preset's max steps, sub-samples, bisected silhouette entries, dense refinement, the
    deterministic step phase on Earth-fixed height shells - with the shapes band-limited at the pixel footprint
    (`cloud_sample_extinction`: beyond a shape texel, 250 m, the shape's box-filtered mips at that level -
    deterministic, no sub-pixel offset, spec decision 4 - the sub-samples' structure step scaled with the mip) and the
    light from `cloud_far_light_depth` (`cloud_sample_light_source`: no light cache there): 5 geometric intervals
    20 m .. 1.5 km [E] through the same shapes (the far cumuli's lit and shaded flanks; equal intervals put the first
    sample 250 m out, above a deck's top: 25 % dark; 3 intervals: 7 %), then the column's grid-mean depth above that
    point along the light. A separate simpler far march (fixed 0.5 % steps) needed the full step budget in the analog
    weather and still lost far deck against cloud_grain_check's 8 x reference (alpha -12 % in the deck view); a single
    slab per pixel (the column model, or the shapes sampled along its chord) drew far cumulus as flat slabs, lost a
    low layer's cloud under a higher one and needed per-cloud weight heuristics;
  - composite: the march's (1 - w)-weighted image plus the march's transmittance at 20 km times the far tier's
    w-weighted one - both march the same clouds, so a cloud in the band, or two behind each other, is the exact linear
    cross-fade of the two renders; the cloud distance weighted by each part's alpha;
  - the far tier's shapes fade to the far field's column model over the LATERAL pixel footprint at the ray's crossing
    of 2 km (`cloud_far_reference_m`; its lowest point when it stays above: continuous over the pixels) from 1.5 to
    3 km (`CloudLayer.PRESETS` `far_fade_m` -> global `cloud_far_pattern_fade_m` [E]; render_check `--far_fade`, the
    checks' 1e9 / 0 switches; static clouds Task 8 fix round 1): the shapes' mips stop at 1 km texels, so no shape is
    drawn beyond 3 km (spec decision 4: < 3.5 km) - a point sample of them at the limb's 2-11 km footprints was a
    speckled band. Seen from above the footprint counts up to 3 x (x max(1, 3 |cos|), `CLOUD_FAR_STEEP_FOOTPRINT_FACTOR`
    [E]): a steep ray crosses the layer within its footprint, so from low orbit at ~1 km the thresholded shapes flicker
    per pixel into grain, while a grazing ray averages them over km of its path. The start keeps the shapes at the
    desktop's far horizons from 9 and 20 km (~0.9 / ~1.3 km at its 2.6 mrad march pixels); VR's (5.4 mrad: ~1.8 /
    ~2.7 km) lie in the fade, mostly the column model with its sub-grid cover;
  - the column model (`cloud_far_slab`: the far field's slab placement along the ray, from above or from below, a
    crossing in front of the band taken at its start; `cloud_far_column`), weighted by w at the slab: the column's
    grid-mean depth and largest fraction from the column texels' cubic B-spline (`cloud_column_mean_depth_smooth`,
    `cloud_column_texel_smooth`: from orbit a 0.25 deg cell spans tens of pixels and the bilinear's creases drew a
    lattice of soft squares; the slab heights stay bilinear); the even cover C_v (the beam cover of the largest
    fraction at the view's cosine) distributed by the sub-grid cover (`cloud_far_cover`, spec decision 4's
    deterministic footprint average): the share of 4 x 8 fixed taps over the pixel's footprint x 2 across the view and
    / mu along it (at most 32 km) where the shapes' prefiltered value exceeds the prefilter threshold of C_v itself, at
    the mip of the taps' spacing - the pattern covers C_v of the layer, so the mean cover is C_v at every footprint and
    view angle (spec decision 3; fix round 2: the share of the fraction's threshold x C_v / f, clamped at 1, lost 9-15 %
    of the cover where whole footprints were covered; the deck's cover is C_v), half of it from a second, unaligned
    reading of the shape (Earth-fixed, turned 37 deg about (1, 2, 3), x 1.37) faded in from 1 to 3 km of footprint (the
    32 km tile's period showed as a regular lattice over the hundreds of km of an orbit view), and C_v itself beyond
    8-16 km footprints (the scaled-space far field's even cover); the in-cloud depth the even cover's; the radiance the
    plane-parallel slab's with the shapes' 3D Sun light (below, far field). From 420 km every pixel is the far
    tier's (the march segment is empty): below ~1 km
    footprints (nadir) the shapes' fade from above, then the column model with its sub-grid cover: a broken cumulus
    field down to the limb, no cell lattice, no speckle (`t8f_orbit*`).
  The debug outputs march the whole segment without the far tier (the checks' physical quantities of the volume). The
  column texture reaches the compute kernels through the same swizzled view as Godot's shaders (cover in .a).
- **Quality presets** [E]: desktop 128 coarse / 512 max steps, fine 1.2 %, light 6 near + 3 far samples; VR 96 / 384,
  2.5 %, 5 + 2 (static clouds Task 3: the deterministic march lost cumulus bases with 48 coarse steps; case 9 passes
  with the 2.5 % fine step; cost and the Task 10 profile in `vr-performance.md`: kept, every cheaper setting
  changes the look - user decision); both 3 detail levels; resolution 1/2 desktop, 1/3 of the VR centre view (VR performance plan part 2); temporal weight floor 1/32 /
  1/96 (jittered march only). Max steps 4 x the coarse ones (Task 5c fix round 1): with 192 the VR fine steps through
  a broken layer's cloudy region (mostly clear between the shapes) ran out before the far cloud - 7-16 % of the deck
  view from 9 km lost its far deck (alpha -8.5 % vs the 8 x reference); 384 and 768 gave the same image; the
  desktop's 256 lost 3.8 % of the deck's alpha (512: 1.3 %). The near / far hand-over band 20 / 34 km in both
  (above); the far tier's march takes at most the preset's max steps.
- **Equiangular VR centre image** (fidelity flag `CloudCentreView.equiangular`, default off; user argument
  `--cloud_equiangular`, or the environment `STARSIM_OM_EQUIANGULAR=1`; old march cost study 2026-10-29): the VR centre
  image (march, outline pass, refine) is uniform in angle per axis at the densest eye's density on the axis, instead of
  rectilinear (whose pixels shrink in angle toward the edges by 1 / (1 + t^2)). The march maps its image coordinate u to
  the projection's ndc through t = c + h ndc, angle = mix(atan(c - h), atan(c + h), (u + 1) / 2)
  (`cloud_equiangular_ndc`, c / h from the projection's corner rays; flag `CloudView.temporal.y`); the eyes' upsample
  inverts it with the same c / h (`CloudCentre.warp`, `CloudCentreView.warp`; `cloud_centre_position`). The pixel angle
  (band limit) is the angle span / rows. Pimax Dream Air at divisor 3: 924 x 747 instead of 1377 x 946 pixels
  (0.53 x); the axis resolution is unchanged, the periphery gets the axis' angular resolution instead of up to 3 x it
  (crops `screenshots/cloud-world-volume/t8_cmp_*_eq*.jpg`: 0.8-3.8 % of the eye pixels differ by more than 8 of 255,
  softer only toward the edges). Deterministic march only (the jittered march's reprojection assumes the rectilinear
  image). Unit check: `cloud_vr_stereo_test` (round trip march warp / eye inverse, pixel count, axis pixel angle).
  Cost in `vr-performance.md` "Old march in VR stereo".
- **March method (F7) and divisor (F8)** (2026-10-29 / 10-30): `CloudLayer.march_method` (`CloudEffect.MarchMethod`
  BAKED, the default of every start - Start-VR.cmd, Start-Desktop.cmd, the checks -, or OLD; user argument
  `--cloud_method=old|baked`); F7 (`CloudLayer.cycle_method`) cycles "Volumen" (the baked volume, equiangular centre
  image) -> "alter March" (rectilinear) -> "alter March, winkelgleich" -> back; the HUD line (`status_line`) shows "(Volumen
  wird gebacken)" until the first set is shown. `CloudLayer.divisor_override` (F8: 1, 1.5, 2, 3, 4; `--cloud_divisor=<x>`,
  fractional allowed) and the march cut (F9: 0 / 10 / 15 km, `cloud_march_cut_m`) are the other live toggles. The old
  march stays as the F7 reference and for the checks' debug outputs (cloud_check, cloud_grain_check, cloud_far_check
  set `march_method = OLD`).

### Validation

- Backlit slab transmitted radiance (Task 9b, cloud_check 14; fixture `tests/validation/weather/
  cloud_slab_transmitted_mc.csv` from `tools/validation/cloud_slab_monte_carlo.py --transmitted`, 1e7 photons per case,
  sigma 0.3-0.7 %): the GPU's bin means (debug output 2, light cache filled) vs Monte Carlo at tau 10 / 30, Sun zenith
  75 / 45 deg, view zenith 0 / 30 / 60 toward, across and away from the Sun: within 4.9 % (tolerance 7 % [E]). The
  cloud shadows on the air: section 14's backlit cumulus, crepuscular rays (+ ray-free control), segment sum vs the
  scene's single scattering, outline, front weight, no-negative and clear-sky cases (numbers in the Model section);
  core `[weather]`: `cloud_render_top_m`; core `[atmosphere]`: the single Rayleigh table = the raymarch's single
  Rayleigh within 1 % and below the scattering table's total.
- Reference values at Task 10 follow-up (ff62126 with 6 sub-samples and step weight 0.3 - reverted to 0.15,
  the far check re-run at 0.15 in sdd `final_fix/`; RTX 4090; sdd `t10_checks_e3/`): `cloud_check.gd` PASS, 208 samples, 393 s for the whole run (section 6 alone,
  `STARSIM_GROUND_ONLY=1`: 12 s; the Sun through the shipped analog June cumulus at Starbase: 6 disks through gaps at
  0.9999 of the cloudless disk, 2 behind cores at the march's T 3e-6 / 0, the mean transmittance of the shapes over 60
  directions 0.727 vs the grid mean 0.730); `cloud_far_check.gd` at step weight 0.15 (`final_fix/far_w015.log`): passes, hand-over fade column / shapes
  1.0936 (scattered cf 0.12 nadir) and 1.0849 (broken cf 0.4 low Sun 45 deg) of 10 % (0.3 gave 1.1085 / 1.1024:
  why it was reverted); speckle RMS vs the sub-pixel supersampled reference:
  limb from 420 km 5.0 %, 50 km horizon 9.4 % (limit 10 %), at the VR pixel angle (1.8 mrad, 3 x 3 reference) the limb
  6.2 % and the 50 km horizon 11.0 % (listed, not asserted: open point); `cloud_grain_check.gd` VR PASS (grain ratio
  1.01-1.05, alpha grain 0.995-1.165 (Cape the largest), RMS 0.033-0.057) and `--quality=desktop` PASS (grain
  1.01-1.05, alpha 0.97-1.03, covered / mean alpha within 0.9 %, RMS 0.036-0.081, ground the largest); stereo, layer,
  smoke and repeat_identical (ground, deck) green.
- Reference values at Task 10 (6b9149e; with 8 sub-samples and step weight 0.15): `cloud_far_check.gd`: slab radiance
  vs Monte Carlo worst 0.34 % (72 geometries), deck tau 10 inside / outside 2.68 % (grid mean), 0.22 % (far tier),
  8.16 % (converged shapes, limit 10 %), deck tau 2 3.17 / 0.60 / 1.54 %, broken cf 0.4 2.28 %, far field / converged
  volume 0.925 and 0.951 (limit 10 %), far tier / march 0.990 (both cases), cross-fade worst 2.48 % / 0.38 %,
  hand-over step 2.52 % / 0.58 % (limits 3 %), from 420 km 0 pixels differ.
- GPU `game/tests/cloud_check.gd` (the cloud pass; grid-mean field in debug outputs 1 / 2): background transmittance
  below / inside / above a tau-1 slab vs the CPU Beer-Lambert within 1.5 %; Sun radiance from above a tau-10 slab vs
  `slab_reflected_radiance` within 1.4 % with a converged light march (24 samples, 8 frames), the preset's 6-sample
  march +10 % at Sun 60 deg (Jensen bias of exp(-tau_s), tolerance 15 %); with the table vs
  `slab_reflected_radiance_mc` within 1.9 % (tolerance 3 %) at six geometries incl. a Sun 10 deg up seen forward (view
  75 deg: 0.368 /sr), the preset's march +5.7 % there; filtered coverage 0.1036 / 0.5057 / 0.9021 at
  full resolution (state 0, detail levels off for all numbers); climbing out in 50 m steps (temporal weight 1) <= 4.81 %
  of the brightest value per step; inside finite and non-black; transmittance to an occluder 2 km ahead inside the
  slab exp(-beta 2 km) within 0.7 % (depth -> distance); a near "mountain" edge through a half-resolution block: the
  occluder column T = 1.000, the sky column the slab's within 1.5 % (upsampling). (These sections run with the light
  cache on and the deterministic march; section 10 with the jittered one - it compares against independent jitter's
  noise.) Section 9 (static clouds Task 3): a cf 0.3 layer of the thick
  slab seen from 5 km above its top, straight down, 96 x 96 px at 30 deg; the deterministic march's 50 % background
  transmittance contour (debug 4) vs the converged reference (jittered, fine 0.2 %, 512 coarse / 1024 max steps, 16
  frames): farthest mismatched pixel 2 px desktop / 3 px VR from the reference's contour (limit one march pixel: 2 /
  3 px; reference contour 342 / 333 px); without the entry bisection 3 / 5 px (RED). `render_check.gd
  --repeat_identical=2` (VR preset, analog weather, Starbase 2024-06-15 21 UTC): two consecutive frames of the ground,
  9 km and 20 km views bit-identical (0 of 1 440 000 pixels differ; the jittered march: 410 808). Section 11 (static
  clouds Task 5b): the thick slab at cf 1 seen at its top's limb from 2 km above it, the camera rolled 7 deg (the limb
  a straight edge crossing the march grid at a shallow angle), VR preset (divisor 3), 192 x 96 px at 20 deg, the
  pass's result's alpha read back: over the 20 rows the edge crosses, the largest alpha step between neighbouring
  pixels 0.194 over 20 rows (limit 0.35 [E]; without the outline pass 0.407 - RED; four full sub-ray marches gave
  0.102), against the row's trend 0.019 (limit 0.03 [E]); a second render bit-identical; the same view at divisor 1, 2400 x 1800 (67 500 groups, beyond the 65535 of one dispatch
  dimension): the pass still acts (partial alphas on the rim rows). Section 8, the light cache vs the per-sample march with the cache's own sample count (96 + 32, stratified
  midpoints), on the game's deterministic march (one frame each, the same view samples; Task 3 fix round 1), at every
  pixel of a 64 x 64 view per cascade at its scale (camera 5 / 9.9
  / 18.8 / 36.5 / 72 km up, 80-81 deg down; the farthest cloud samples asserted inside that cascade's full weight;
  error relative to max(pixel, 5 % of the view mean)), the fill at 1/16 of the voxels per frame (complete after 16
  frames): a tau-10 slab (debug 2) worst pixel within 5 % (measured 0.07-0.71 % in every cascade); a cf 0.4 broken
  layer (debug 7, the shapes): the 99th percentile of the per-pixel error within 10 % and the worst pixel within 20 %
  (the interior / silhouette split, alpha < 0.3 within one march pixel, debug 4, is printed only). Worst pixel at Sun
  45 / 20 deg: cascade 0 2.78 / 6.71 %, 1 4.81 / 16.00 %, 2 6.00 / 11.45 %, 3 8.13 / 9.52 %, 4 9.95 / 12.08 % (p99
  1.6-6.8 %, mean errors 0.16-0.95 %). A self-test of the verdict (`_cache_case_ok`: a slab with a 10 % worst pixel
  fails) runs first. The cascade-1 view's failing pixels are interior pixels held by cascade 0
  (read at the sample, 62.5 m voxels) at the low Sun: the voxel interpolation of a cumulus' self-shadowing at a fixed
  sample - the jittered march's 8-frame mean had brought it to 9.58 %; without the entry bisection it stays 12.7 %
  (not the bisection). Open: a user decision (finer cascade 0 / a near march for it, or a tolerance). `info` lines:
  the worst pixel's cascade shares along its ray and the worst pixel by the cascade holding each pixel's slab middle;
  Task 2's runs (jittered): the 1-km near march left cascade 2 32 %, 3 39.6 %, 4 15.1 % there; a view beyond
  every cascade (complete fill) and the frames right after a clock
  jump bit-identical to the march; the fill at the game's budget (98 304 voxels) with the 95th percentile <= 0.5 ms over the
  427 frames of a full refresh from nothing (fix round 3: median, p95 and the worst frame printed; the worst frame is
  an isolated spike at a GPU clock-state transition in a lightly loaded test, 0.43-0.65 ms in fix rounds 1-2, median
  0.09-0.12, p95 0.32 ms; the per-frame times flip between ~0.09 and ~0.30 ms for alike jobs, the GPU's clock state;
  the per-frame worst under load is measured in the headset in Task 10). Round 3: median 0.099, worst 0.454 ms. Section 10 (no seam; the march
  with the cache's sample
  count): the row-to-row step of (cache - march) / mean along a line of pixels over each edge band within the image
  noise: 0 -> 1 0.046 / 0.071 %, 1 -> 2 0.030 / 0.050 %, 2 -> 3 0.021 / 0.048 %, 3 -> 4 0.061 / 0.074 %, 4 -> march
  0.051 / 0.502 %; the same view before / after a re-anchor of cascade 3: 0.0000 %. Measured on the way: 64 / 256 / 512
  voxels up worst 78 / 23 / 8 % (Task 1, broken, Sun 20 deg); 24 + 8 fill samples 30 % off the converged integral at a
  pixel, 48 + 16 10.6 %; a x8 layout (8 / 64 / 512 km) broken worst 42 / 193 % (64 km) and 129 / 1005 % (512 km); the x2
  layout read at the sample 12.7-107.7 %; the hybrid's near lengths and their measurements in
  `CloudLightCacheSchedule` (`NEAR_INTERVAL_M`). The
  earlier 3 -> 4 "seam" (0.29 % against 0.19 %) was the preset march's own pattern (6 + 3 samples) as reference, not a
  seam: against the cache's sample count the step is 0.061 %.
- GPU `game/tests/cloud_grain_check.gd` (static clouds Task 5c, fix round 1; the main scene, analog weather 2024-06-15
  21 UTC, VR preset) against the 8 x steps reference (768 / 3072) in four views - the Cape (28.57 N, 80.65 W, 20 deg
  up), the ground at Starbase, the deck from 9 km, cirrus from 9 km off Peru (18 S, 78 W, 15 deg up): the grain (mean
  |Laplacian| of the cloud image's premultiplied luminance at a spacing of one march pixel over the pixels with alpha
  > 0.1 and their four neighbours, relative to their mean luminance) and the same on alpha within 25 % [E] both ways
  (no salt-and-pepper, no blur); the covered pixels (alpha > 0.1), the mean alpha and the mean luminance within 3 %
  [E]; the RMS luminance difference over the cloud mask relative to the reference's mean there <= 0.1 [E] (the
  reference band-limits the near detail more finely: lobe-scale shading differences up to ~0.09 by design); the
  cloud image of the next frame bit-identical; the check fails if the reference does not settle. Values: see
  `.superpowers/sdd/2026-10-26-static-clouds/task-5c-report.md` (fix round 1); before it (cb65707) the deck's alpha
  -8.5 %, alpha grain 1.30, cover -5.3 %, RMS 0.14-0.17 on ground / deck / cirrus. `render_check --repeat_identical`
  at the Cape: the frames differ in the sea below the horizon only (y >= 825; the base too) - the check prints the
  differing pixels' bounding box.
- GPU `cloud_check.gd` section 6 (terrain_earth.gdshader on a quad 300 m from a camera 2 m up; slabs tau 1 / deck tau
  3 / broken cf 0.4; Sun 45 / 20 deg; debug output 5 = the three factors): with the near map of the grid mean
  (`CloudShadowEffect.grid_mean_only`) direct vs the CPU within 1.3 %, diffuse within 0.6 %, sky within 0.3 %, the
  rendered ground's luminance ratio weather on / off vs (T_s E_dir + T_sd E_sun,h + T_d E_sky) / (E_dir + E_sky) within
  0.9 % (tolerance 5 %); with the game's map (the shapes) under the cf 1 slabs the ground's total light vs the vehicle's
  CPU light at the same spot within 5.7 % (tolerance 10 %; under a deck tau 3 0.4 % / 0.1 %; the shapes' direct differs
  from the grid mean by up to 2.3x at Sun 20 deg through tau 1 - their low octaves vary along a 3 km path); with weather
  off land and sea bit-identical to the shader without its cloud blocks. atmosphere_check 86/86.
- GPU `cloud_check.gd` section 7 (fix round 1; `tests/cloud_light_probe.gdshader` evaluates the surfaces' cloud-light
  functions at receivers on annuli around the camera, means over eight sites ~55 km apart = independent realisations of
  the shape noise; 93 samples in all): the shapes' area mean of the direct Sun over the near disc (r < 8 km) vs the CPU's
  cell, cf 0.15 / 0.3 / 0.4 at Sun 45 / 8 deg: within 6.1 % (the worst, cf 0.3 at 45 deg; ground 0.795 / 0.657 vs CPU 0.777 / 0.649 at cf 0.15, 0.627 /
  0.404 vs 0.592 / 0.396 at cf 0.3, 0.517 / 0.277 vs 0.490 / 0.277 at cf 0.4; tolerance 8 %); the near map's mean vs the
  far map's over the same receivers likewise; the column terms (Sun's diffuse, sky) within 0.6 %; the vehicle's total light
  vs the ground's mean within 3.3 % (cf 0.4 at Sun 8 deg: 0.715 vs 0.714; tolerance 10 %; asserted, no longer only
  reported). Hand-overs: near -> far (9-11 km, annuli 8 / 10 / 12 km, a cf 0.3 layer, Sun 45 / 8 deg) steps 0.02 / 0.08 against
  an allowance of 0.05 + 3 standard errors over the sites (0.11-0.18 for the broken layer: one annulus holds only a few
  cells, so this check is weak; the disc check above carries the agreement); a deck: 0.003 / 0.005. Far -> the column
  slanted (110 / 130 / 150 km): steps <= 0.002, the far map within 0.003 of the column (tolerance 0.03). Before the
  beam-cover fit the same checks failed: the near mean was 10 % below the far / CPU at 45 deg and 40 % at 8 deg.
  `STARSIM_SLANT_TABLE=1` prints the fit table. Render: `tmp/weather_t9f_sunset_handover.png` (cf 0.4 layer, Sun 9 deg,
  looking away from the Sun and 4 deg down from 700 m: the ground at ~10 km): no ring.
- GPU `game/tests/cloud_far_check.gd` (Task 10; the game's scene, HDR, linear tone map, fixed exposure): the far
  field's slab radiance vs the core's `slab_reflected_radiance_mc` at 72 geometries (tau 0.5-30, Sun 0-80 deg, views
  0-75 deg forward / side / back) within 0.34 % (tolerance 3 %; `tests/cloud_far_probe.gdshader`); weather off: the
  scaled-space Earth drawn with `earth.gdshader`, bit-identical to the pre-Task-10 shader as one file (0 of 8.6e6
  bytes); the hand-over just inside / outside 1.5e7 m (2024-04-08 14:00 UTC, camera over 0 N 45 W, seven Earth points
  0-45 deg away, Sun 20-75 deg up): decks of tau 10 / 2 (cf 1, 1-2 km) with the volume on the grid mean (cloud debug
  output 6: the far field's physics without the shapes) within 2.95 % / 3.89 % (tolerance 5 %); the volume pass only
  inside, `earth_clouds.gdshader` only outside, the CPU light factors (1, 0, 1) outside; from 384 400 km with the ERA5
  hour of 2024-04-08 18:00 five cloudy cells (vertical transmittance < 0.05) 5.3-8.2x brighter with the weather on,
  five clear cells (no cloud within 1 deg) unchanged (ratio 1.0000). The game's cloud pass inside (static clouds
  Task 8: every cloud there lies beyond the near / far hand-over; its far tier draws them with the column model at
  these footprints) against the far field outside, tolerance 5 %: deck tau 10 within 0.22 %, tau 2 within 0.62 %,
  broken cf 0.4 (21 x 21 block means) within 2.30 % at the six points off the sub-camera point; the converged volume
  with the shapes (the march alone, 8 x its steps) against the far field (the column model's own error, tolerance 10 %,
  the M8 ruling) deck tau 10 within 8.15 % (before fix round 1 11.98 %), tau 2 within 1.49 % (a broken layer's volume
  beats with the 32 km tile at these ~17 km footprints, 17-32 % either way: section 6); the broken layer (cf 0.4) in
  a 1 deg view (~1 km march pixels resolve the shapes; fix round 2), the whole image's mean at nadir, far field /
  converged volume 0.925 (Sun 54 deg, 25 S 45 W) and 0.951 (Sun 24 deg, 0 N 95 W) (the plane-parallel slab alone
  0.736 at 24 deg), at 0 N 45 W (Sun 73 deg) 0.968 listed, not asserted (the ground alone steps 55 %); there the
  ground alone
  steps across the limit (weather off: 0.157 inside / 0.069 outside, the terrain's sea glint, which the scaled-space
  Earth lacks; seen through a broken layer's gaps it was 11 % of the pixel) - points whose ground steps by more than
  the tolerance are listed, not asserted. Every placement resets the cloud history and the frame counter and draws
  exactly 64 frames (Task 11). Static clouds Task 8, section 5 (`--only=handover`; the cloud pass's own image,
  desktop preset): from 420 km the game's image equals the far tier's alone (0 of 1.44e6 pixels differ); a broken
  layer (cf 0.4) and a deck (tau 10, 1-2 km) seen from 9 km (0 N 45 W, 10 deg down to the north, Sun ~60 deg up): the
  far tier's mean radiance at 30-50 km vs the march alone 0.9905 / 0.9900 (tolerance 10 %, the M8 ruling), the game's
  image the linear cross-fade of the two within 2.38 % / 0.39 % in every 2 km bin from 14 to 60 km (3 %), the band's
  own step between bins, measured on the game's image relative to the march's, 2.45 % / 0.57 % (3 %); a camera at
  20 km moved by 100 m: 4 interior cover flips in 646 130 covered pixels against 0 of a static pair (allowance 0.1 %
  of the covered pixels [E]). Section 6 (`--only=column`, fix rounds 1 and 2): from 420 km at nadir / 45 deg, the
  central square's means over three sites 1 deg apart (a single site's column cover is off by up to +-6 %: the
  sub-grid cover reads the 32 km tile at one height), tolerance 10 %; the column model vs the converged volume,
  radiance / alpha: broken cf 0.4 (Sun 73 deg) 0.986 / 0.993 and 0.967 / 0.936, deck tau 10 0.989 / 1.001 and
  1.007 / 1.001; held out of the 3D Sun light's fit: scattered cf 0.12 (Sun 73 deg) 1.013 / 1.059 and 0.949 / 0.931,
  broken cf 0.4 under a 24 deg Sun (0 N 95 W) 0.940 / 1.001 and 1.011 / 0.942 (at 8153ed5, the clamped sub-grid
  cover: radiance 0.857 / 0.924, 0.888 / 0.928, 0.838 / 0.960, alpha down to 0.861; `--no_3d_light`: radiance
  0.68-0.83); vs the far tier's shapes (the brightness across the fade) 1.041 / 1.023, 0.993 / 1.004, 1.094 / 1.015,
  1.002 / 1.084; no speckle - the game's image against its 2 x 2
  sub-pixel supersampled reference (the camera turned by whole screen pixels, aligned back), RMS over the far band:
  the limb from 420 km 5.0 % (fix round 1 6.0 %, before 16.1 %), the far horizon from 50 km 9.3 %
  (before 10.3 %; tolerance 10 % [E]).
  cloud_grain_check (the far pass included, its 8 x reference too): Cape / ground / deck / cirrus cover within 0.02 /
  1.00 / 0.16 / 0.10 %; the deck view's horizon row (fix round 1): 38 isolated outlier pixels within 3 rows of the
  terrain's horizon against ~116 expected from the rows 8-40 away (before the mixed-depth gather 272; limit 1.5 x +
  3).
- Image identity of the shape prefilter (`render_check.gd --fixed_exposure --deterministic=64`: the cloud history
  and frame counter reset, then exactly 64 frames; ERA5 2024-04-08 18:00; identity holds at the check window (1600 x
  900) and the views' FOV used there - a larger window or a narrower FOV shrinks the pixel angle and keeps the
  unfiltered kernel up to a higher view; a wider pixel prefilters from lower): the Starbase ground view and an ascent
  view at 25 km toward the horizon are bit-identical to the commit before (a7d52e1); two runs of a7d52e1 itself
  differ in 0 / at most 1 pixel by 1. The low-orbit oblique view (420 km) differs at the limb only, where the
  footprint exceeds f_s.

### Diagnostics

- March counters and ablations (world volume plan Task 0; `vr-performance.md` "Where the march's time goes"):
  `render_check.gd --counters` compiles the march kernels with `CLOUD_COUNTERS` (`CloudEffect.counters`, set before the
  CloudEffect exists) and, once settled, prints `render_check: counter <name> median m p90 a p99 b max c sum s` per
  march pixel for `density_fetches` (texture fetches of the view samples: field cell, shapes, detail, threshold /
  edge-mean tables, column tables; the air's tables are not counted), `light_fetches` (everything inside one sample's
  light-source evaluation: light cache, light march, transfer table, far tier's light path), `occupied_steps` (a march
  step whose sample lies inside cloud, extinction > 0; a step done again by the step-weight limit counts again) and
  `empty_steps`, plus `<out>_counter_<name>.png` (normalised to the maximum). The texels are `RGBA32UI` (density,
  light, occupied, empty; `CloudEffect.debug_centre_data("counters")`, set 1 binding `CloudEffect.COUNTERS_BINDING`
  24, not in the outline pass or the cache fill). `--ablate=<names>` / `STARSIM_CLOUD_ABLATE=<names>`
  (`CloudEffect.ablate`, names in `CloudEffect.ABLATIONS`) defines `CLOUD_ABLATE_<NAME>` in the same kernels: `density`
  (the view samples' shape, detail and tables -> the cell's grid mean), `light` (a sample's light-source evaluation ->
  constant), `air` (the atmosphere tables at the clouds -> constants), `field` (the field's flat-base / ice /
  stratiform fetches), `shape` / `detail` (everywhere, the light march included), `subsamples` (1 per step),
  `outline` (no outline pass), `air_shadow`, `far_tier` (no far pass). Each changes the image (read a time saved with
  the counters of its own run); with neither switch the kernels are the game's.
- `cloud_debug_output` 1 / 2 / 3 / 4: background transmittance (grid mean), Sun radiance (grid mean), coverage at
  `cloud_debug_height_m`, background transmittance with the shapes; 5: the terrain draws its cloud factors (direct,
  the Sun's diffuse share, the sky's) as its colour (`render_check.gd --cloud_debug=5 --fixed_exposure`); 6: the
  full image on the grid-mean field without the shapes (the far field's physics; `cloud_far_check.gd`); 7: as 2 with
  the shapes (the light cache vs the march on broken cloud, `cloud_check.gd` section 8).
- Light cache: `render_check.gd --no_light_cache` (the per-sample march, for before / after images);
  `CloudEffect.pinned_frame` (checks: the march's jitter of a fixed frame); with `CloudEffect.capture_timestamps`
  (`terrain_perf.gd --gpu_passes`) the fill is its own interval `cloud_light_fill`; `CloudLightCache.settled()`,
  `planned_fill_frames`, `last_fill_voxels`, `schedule.lookup_state()` (windows, held rectangles, weights);
  `schedule.budget_voxels` (the checks fill at 1/16 of the voxels per frame); `STARSIM_LIGHT_CACHE_ONLY=1` runs
  `cloud_check.gd`'s sections 8 and 10 alone (with per-row seam profiles and the fill's per-frame times),
  `STARSIM_CACHE_VIEWS=3,4` only those cascades' section-8 views. Schedule
  logic: `tests/cloud_light_cache_test.gd` (headless).
- `CloudShadowEffect.grid_mean_only` (checks): the near shadow map with the grid mean. `CloudShadowEffect.ran_frame`: the
  render callback's last run (`Engine.get_frames_drawn()`); `CloudLayer` takes the maps as valid (`cloud_shadow_valid`)
  only within 4 drawn frames of it (frames, not milliseconds: a slow check frame outlasted the old 250 ms window and
  switched the ground's / air's cloud shadows off and on in an unchanged scene - world-volume Task 7, R4).
- `cloud_check.gd` with `STARSIM_SUN_ONLY=1`: section 13 alone (36 checks): the cloudless disk's centre = the core's
  sun transmittance x `World.sun_disc_scene_value` within 3 % per channel (13 deg up: unclipped; 55 deg up: clipped
  with its colour kept); through slabs of vertical tau 2 / 5 / 7 / 8 / 9 / 10 / 20 at 55 deg the disk's part (the
  render with the disk less without it) within 5 % of the march's shapes transmittance (debug 4; and the two renders'
  half-float resolution at most half of it), at or above exp(-tau / mu) of the core, exactly 0 where the march's T is
  below the store (tau 20: and the core's T there too), the march's slant tau within 8 % of the core's, brighter than
  the terrain while expected to, the aureole >= 100 x (303 x) the radiance 60 deg off; near march and far tier alone
  (tau 5 / 20). `render_check.gd --sun_through_deck=<tau> --lat= --lon=` renders the view
  (`.superpowers/sdd/.../img/t9f_tau*.png`, `t9f_edge.png`); `--gpu_time[=120]` prints the root viewport's GPU render
  time (median, p99), `--render_scale=1.65` with `--resolution 2560x1440` the VR pixel proxy.
- `cloud_check.gd` with `STARSIM_OUTLINE_ONLY=1`: section 11 alone (`STARSIM_VERBOSE=1` prints the worst row's alpha).
- `cloud_check.gd` with `STARSIM_LCL_ONLY=1`: section 12 alone (the flat cloud base on a snapshot built from fields,
  `StarsimCore.weather_write_column_snapshot`: optical depth straight up 0.9993 of the core's column (1 % allowed), no
  cloud 100 m below the base (the fog column has it), above the base's layer as without the base texture).
- `cloud_grain_check.gd -- --only=cape` (or `ground`, `deck`, `cirrus`): one view of the grain check.

### Open points and known limits

- **Speckle margin at the VR pixel angle (Task 10)**: cloud_far_check's 50 km horizon band at the VR march pixel (5.4
  mrad, 3 x 3 reference) is 11.0 % RMS against the 10 % bound (9.4 % at the check window's 2.6 mrad pixels: a thin
  margin there too). The band is 5 rows; the RMS is the march pixel's outline position against the sub-pixel mean (no
  speckle visible, `t10b_far/t8fcheck_speckle_vr_horizon_from_50_km.png`), so it is listed, not asserted, and the bound
  is not widened. The limb at the VR angle (6.2 %) is asserted.
- **Fade reference point (Task 10, review Minor 3)**: the far tier's shapes fade at the footprint at the ray's 2 km
  crossing, or at its lowest point when it stays above. A ray from 9 km with its tangent point at 2 km (d = 299 km)
  meets 15 km high cloud at 706 km: the footprint there is 2.4 x the reference's (VR 5.4 mrad: 3.8 km against 1.6 km;
  the desktop's 2.6 mrad 1.8 against 0.8 km), i.e. past the shapes' 1 km mips where the fade has not started. Seen only
  as a far cirrus / Cb horizon from altitude in VR (the cirrus grain check passes); not changed - fading at the
  farthest cloud would change the look of the horizon cirrus (user decision).
- **Cloud shadows on the air (Task 9b)**: shapes' shadows (rays) only within the near map's 9-11 km; beyond, the far
  map's grid mean shades the air smoothly (no individual rays from far clouds). Cameras above the atmosphere's top
  (~100 km) get no air shadow: the loss switches off there during an ascent (the tables' parametrisation needs the ray
  moved to its top-of-atmosphere entry, as `atm_sky` does; not done). The outline pass takes the source's loss at this
  pixel's cloud distance (p99 1.7 %, max 7 % of the pixel's loss). Backlit rim / body: 2-4 x is the
  photographic range [E] (cloud-visual-reference.md 5.3), not a physical bound; the ratio rises with nearness to the
  Sun (the rim's forward peak) and the body's optical depth, so the 2-6 deg band is accepted at 2-5 x [E] (measured
  4.01-4.03), 6-10 / 10-15 deg at 2-4 x. Re-measured in Task 10 (cloud_check section 14's bands pass, 208 of 208). Twilight cloud shadows above the cloud
  top are not modelled: the ray clip to the cloud top assumes the Sun looks up from the air point; with the Sun
  below the point's horizon by dep the Sun ray first descends ~dep^2 R / 2 (2 deg: ~3.9 km), so the real shadow
  of a low deck at dusk lies above its top (review N3; a fix would clip at top + R (1 - cos dep)). Higher orders of the light scattered into
  the shadowed air are not followed (the multiple scattering is the clear sky's: slightly too bright in large shadows).
  The scaled-space far field has none.
- **Per-sample light march for thick slabs seen from below (Task 9b)**: with the light cache off (or a view the cache
  has not filled yet: the fallback) the transmitted radiance of a backlit slab is +25..+92 % above the Monte Carlo
  reference (tau 10 / 30, Sun zenith 45 / 75; `STARSIM_SLAB_NO_CACHE=1` in cloud_check 14); with the cache filled
  within 5 %. Open: the fallback's light-march length for low Suns.
- The stochastic light march (outside the light cache, and for the Moon) is unbiased in tau_s but exp(-tau_s) per
  sample is brighter on average (+10 % at the worst checked geometry with the preset's samples); more near samples
  reduce it.
- Light cache: the hybrid near march (1 / 3 / 2 / 2 km, 12-32 samples) costs the cloud pass +35-42 % in the VR proxy
  over round 3's 1 km in every cascade, which itself was +45-65 % over the pure lookup (`vr-performance.md`): over the
  2.5 ms budget in VR (accepted, see the standing decisions). The fill's GPU time per voxel depends on the GPU's clock state and the
  weather (real weather ~2x the slab's); at the budget (98 304 voxels) single frames of a first fill reach 0.43-0.65
  ms (clock-state transitions; `cloud_check` section 8 asserts the 95th percentile, the worst frame under load is
  an open headset measurement). Under 1000x
  warp every cascade's shown light lags the Sun by up to 25 deg (the drop bound; 100x: <= 3.6 deg); at 7.8 km/s
  cascades 0-1 fall behind their re-anchors. Beyond 34-42 km the light is marched (the volume march ends at <= 34
  km).
- Light (Task 9): the vehicles' light and the eye adaptation are the grid mean at one point (a vehicle in a gap
  between cumuli gets its cell's mean, not the shadow next to it; under a deck the same within 0.4 %); one light point
  as before (the booster far below the ship gets the followed body's light). Ambient on vehicles = Godot's clear-sky
  cubemap x one factor (its angular shape and the reflections stay the clear sky's). The shadow maps are plane-parallel
  inside clouds (a receiver inside one gets a linear share), the Moon's direct light the column's (no shapes). The
  sky's diffuse transmittance and the Sun's diffuse light are the local column's (no light from neighbouring columns,
  no cloud-side glow in gaps). The scaled-space Earth takes the column (the far field: no shapes, no shadow maps).
- Near / far hand-over (static clouds Task 8; standing decision, controller ruling 2026-10-27 on review I1): the far
  tier beyond the band is a second march pass through the shapes with its own cheaper per-sample light
  (`cloud_far_light_depth`), not a per-pixel column model - the cheaper variants failed the look (the column model
  alone drew far broken cumulus as an even haze ~25 % dark; a separate far march lost far deck, alpha -12 %). This
  supersedes Task 2's ruling that the light beyond the march is never marched per sample and that in orbit every cloud
  is the far field: the column model draws only beyond the far fade's footprints. The cost (+29-35 % VR proxy) is inside
  the accepted frame-rate cost. The far tier's light is the column's beyond 1.5 km of the light path (no light cache beyond the march:
  no far neighbours' shadows past 1.5 km); its shapes are band-limited at the pixel footprint (no detail beyond
  ~125 m footprints, as the march). The column model's sub-grid cover is the shape tile's (with a second unaligned
  reading): realistic cumulus texture from orbit, not the real clouds' positions below the 0.25 deg grid; its sun
  gain for broken cloud is a fit (above). The speckle check's far horizon from 50 km passes with a thin margin (9.4 %
  of 10 %; the shapes' march-pixel outline at ~1 km footprints dominates it). The scaled-space limit: the ground's sea
  glint steps there (terrain vs scaled-space Earth, weather off), seen through broken cloud.
- **March sub-samples (Task 5c fix round 1, Task 10)**: at most 6 per step (8 before Task 10); the step-weight limit stays 0.15
  (0.3 saved 4.7 / 5.2 ms but pushed two far hand-over fade cases to 1.108 / 1.102 of 10 %): the VR proxy's cloud pass
  at the ground / the deck 40.9 / 48.9 -> 36.9 / 45.3 ms
  (`vr-performance.md`), still over the budget like the base. 4 sub-samples (-9 ms) fail the grain check at the Cape
  (alpha grain 1.375 x the 8 x reference's, bound 1.25; the user's ruling to use 4 could not be kept without widening
  the bound), 5 pass with no margin (1.241), 6 at 1.16. One light sample per step.
- **Deterministic march (Task 3, outline pass Task 5b)**: a cloud edge is binary per ray; the outline pass (cloud
  pass 1b) averages the centre and three fixed coverage-only sub-rays on outline pixels, so outlines no longer follow the march grid (no stairs,
  `t5b_*` vs `t5f_*` images), but they stay one march pixel wide across the edge (crisp; at divisor 3 ~3 screen
  pixels), and a pixel's cover is resolved in quarters only. Some mid-distance cumuli (the 20 km view's frayed ones)
  keep static per-march-pixel holes inside - the shape field's own structure at the step / pixel scale sampled at
  fixed points (not the scene-depth step grid: deriving the grid from the layer segment changed nothing; not the
  upsample); interior pixels whose neighbours differ by more than 0.25 are marched again too, which softens but does
  not remove them. A checkerboard remains where the block's nearest / farthest scene distances alternate under a
  cloud (the sub-rays take the block's distance). On a rim pixel the neighbour's colour may come from a farther surface
  (bounded: it is taken only where the centre's alpha is below half the pixel's mean). Tried in Task 3 fix round 1 and dropped: a sub-pixel coverage at
  the first entry (2 x 2 taps over the footprint half a footprint deeper, 4 field lookups per ray) changed ~1 000 of
  1.44 M pixels by > 20/255 with no visible softening. Far cumulus fields at 20-40 km show faint horizontal bands (the
  steps' iso-distance shells). The VR cost is over the 2.5 ms budget (`vr-performance.md`, Task 10).

### Model: removed sentences of kept bullets

- (Shape prefilter at large pixel footprints (from 'Beyond that')) Beyond that, each ray reads the shape at a
  horizontal offset drawn from the pixel's tent filter (half-width f, faded in from f_s to 2 f_s; blue noise + the R4
  sequence per frame) and the temporal mean averages the pixel's transmittance over its footprint - the
  coverage-preserving mean in transmittance (a mean density would make broken cloud a fog); the cell (fraction,
  water) stays the true point's. The deterministic march (the default, static clouds Task 3) reads the footprint's
  centre (no offset, the same in every frame): the mips' box filter there is the footprint's mean cover (spec
  decision 4); the near / far hand-over (static clouds Task 8, below) ends the march at 34 km, so with the presets no
  march footprint reaches f_s any more (VR 5.4 mrad x 34 km = 0.18 km) and the prefilter kernel runs only for the
  debug outputs (they march the whole segment) or a band moved out (checks). Deeper mips (level 4, 4 km texels) were tried: their few features per horizontal
  slice of the tile showed as rings around the sub-camera point (cf 0.4 hand-over 10.7 % instead of 7.5 %). The
  code sits under `CLOUD_SHAPE_PREFILTER`: `CloudEffect` runs that march kernel only when a footprint on the frame's
  clouds can exceed f_s (largest pixel angle x (the horizon distances of the camera and of the 20 km cloud top on the
  largest radius of curvature + the light march's 6 km)), else the unfiltered kernel - bit for bit the code before
  (one added branch alone moved results by an ulp through the compiler). With the 1600 x 900 check window (pixel
  3.1e-3 rad at half resolution) that keeps the ground and an ascent up to ~30 km unfiltered; the limb from higher
  (an ascent near the Karman line, low orbit) is prefiltered where its footprint exceeds f_s (there the shape aliased
  as at the hand-over). Shadow maps never prefilter.
- (Light on the ground, vehicles and the eye (file list)) under the clouds", `cloud_light.gdshaderinc`, `cloud_shadow.gdshaderinc`). Three factors
- (Light on the ground: the shadow maps' fill (old kernel)) -0.5..20 km: 64 grid-mean samples find the cloudy part (every 312 m of height), 64 samples over it give (t_front,
    t_back, optical depth, cover); a receiver at t_r gets 1 - C + C exp(-od / C x clamp((t_back - t_r) / (t_back -
    t_front))) - exact below the clouds, linear inside them; the four texels' transmittances bilinear. The near map
    (+-16 km, 62.5 m texels) marches the visible clouds' density (the shapes, detail band-limited at the larger of the
    texel and the step, the shapes' mean for steps > 0.5-2 km: its cover and depth mix toward the cell's own - cover
    1 for the shapes, the beam's cover of the cell fraction for the mean part), the far map (+-200 km) the grid mean with
    the beam's cover of the largest cloud fraction on the line. Distance rule (horizontal distance of the receiver from the camera): near -> far over 9-11 km, far -> the
    column slanted to the Sun (1 - C + C exp(-tau / (C mu0))) over 120-140 km; smooth, no pop. World space: the same for
    both VR eyes. The Moon's direct light uses the slanted column. While the baked volume draws, both maps are filled
    from its own Sun transmittance instead (same texels; "Baked prototype: cloud shadows from the bake").
- (Cloud shadows on the air (mechanics with the old maps / old march)) (i / n)^2 [E], Bruneton's difference form per segment without a clamp per piece (a piece the tables' interpolation
  makes negative belongs to the telescoping sum; the front and behind parts are >= 0 as a whole), T_c = the mean of 3
  fixed points per segment [E] from the cloud shadow maps (`cloud_shadow_transmittance`: near map's shapes within
  9-11 km, far map's grid mean to 120-140 km, slanted column beyond; inside the near range the near map alone) - one
  midpoint per segment aliased the near map's shadows into regular stripes across the sky, 3 x 32 points look as 96
  segments. The segment's part in front of the clouds' weighted distance is seen directly, the rest through the
  background's T (linear split: a step at the midpoint stair-stepped the loss along the clouds' distance contours); the
  two parts are returned apart (loss = front + T x behind). The composite adds the air in front of a cloud at weight 1
  (`cloud_march`'s alpha x `atm_to_point_offset` to the clouds' distance plus T x the scene), the same weight as the
  loss's front part (cloud_check 14: 1.000). The pass's colour carries -L (the march clamps to +-65000; the composite's
  additive pass adds it). The outline pass (`CLOUD_EDGE_REFINE`) takes the source's loss off with the source's T and
  adds this pixel's with its own (one parts call per outline pixel at this pixel's cloud distance). The upsample's
  undershoot floor: the inner 2 x 2 taps of this pixel's scene depth (depth weight within `CLOUD_DEPTH_MIXED_WEIGHT` of
  the best), the larger of their lowest colour and this pixel's T x their lowest colour per T (`CLOUD_FLOOR_MIN_T`): a
  negative colour belongs to its T x scene, so a cloud body next to the shadowed sky cannot take the sky's -L; 0
  without air shadow, as before. Static (fixed points, `repeat_identical`), world space (both VR eyes), the eclipse's
  unocculted fraction applied. Weather off or no cloud in the Sun's way: L = 0 exactly (cloud_check 14: 0.0000 %).
- (Cloud shadows on the air (bit-identity note)) - The bit-identity of the cloud image (cloud_grain_check) is checked with the air shadow on, above the horizon (8
    rows margin): below it the sea's depth changes between frames and the loss is integrated to the scene.
- (Far field (facets: the old march's far tier)) views +0.17 ms even untaken; the old march's far tier, `CLOUD_MARCH` without `CLOUD_BAKED_MARCH`, unchanged):
- (The Sun's disk through cloud (stages)) Every stage (edge refine mean, temporal mix / clamp, tent /
  Catmull-Rom gather) is linear in T.
- (Baked volume: cloud shadows (the old kernel's cases)) distance rule (near -> far 9-11 km, far -> column 120-140 km) unchanged, the old kernel otherwise (before the first
  set, the old march, above 62 km); (b)
- (Baked volume: cloud shadows (STARSIM_BAKED_SHADOW=0)) `STARSIM_BAKED_SHADOW=0` keeps the old maps and air shadow (comparisons);
- (Kernel build (the old march's shaders)) the old march's 9 shaders from cached SPIR-V still took 6.8 s, from the binary 0.02 s).
- (Kernel build and fast start (start kernels, F7 switch)) At start only the temporal
  pass, the upsample, the active method's kernels (the baked volume's for the stereo march when an XR interface is up;
  the first frame corrects a wrong guess; a later change of the view count re-creates the baked volume for it) and the
  shadow maps' are built; once they are, the other method's build starts
  in the background. Shaders are created from the binaries on the render thread when first needed. No frame blocks on
  the workers (owners poll `done()`; only a freed set's predelete waits): until the kernels and the baked
  volume's first set are ready the frame shows no clouds (`CloudEffect._skip_frame` clears the result to T = 1); an F7
  switch to the old march before its build finished keeps drawing the baked volume until it has.
- (Kernel build and fast start (measurements)) "Wolken-Shader laden" while the active method's kernels build (`CloudLayer.kernels_status`, Main's hold); Boot's step
  before instantiating Main is "Hauptszene aufbauen". Measured (desktop RTX 4090, Start-Desktop args, engine start to
  "[Boot] done" = the first world frames): before 23.70 s (19.6 s of it the serial build in Main's instantiation), first
  start after 4.06 s (cache empty: the start kernels built in 1.2 s in parallel, the old march's 9 in 2.5 s in the
  background), second start 4.37 s (all from the cache in 0.02 s; the rest of the boot dominates). F7 to the old march:
  0.02 s with its build done; 1.69 s of baked frames when switched at frame 40 of a cold-cache run.
- (Baked volume (method switch)) - **Baked volume (the default since 2026-10-30; "Weg 2", 2026-10-29)**: `CloudBaked` (`cloud_baked.gd`,
  `cloud_baked.gdshaderinc`), `CloudEffect.MarchMethod.BAKED`, F7 state "Volumen" (next bullet but one for the
  toggles; `--cloud_method=old` starts on the old march). Shape and light are baked
- (Baked volume (compile check)) every kernel of both methods in `tests/cloud_compile_check.gd` (`-- --baked_only`).
- (Baked volume step 6 (the old march's far tier)) field's column light (`cloud_far_cloudy_radiance`, flat levels and scaled-space far field; the old march's far tier
  unchanged under `CLOUD_MARCH` without `CLOUD_BAKED_MARCH`) turns
- (cloud_layer_test (exit leak check)) Exit leak check: `render_check` (default and
  `--cloud_method=baked`) prints no "RID leaked" warning
- (render_check options) `--cloud_detail_levels=0..3`, `--cloud_scattering=table|closed`, `--no_light_cache`,
- (render_check identity) `--fixed_exposure --deterministic[=64]` for before / after image identity (cloud history and frame counter reset
  once settled, then exactly that many frames).
- (Kernel build diagnostics (old march line)) `CloudEffect: old march ready (waited for its build x s, shaders created in y s)` and `baked volume's shaders
  created in y s` on first use.
- (Kernel build diagnostics (compile check)) `tests/cloud_compile_check.gd` (GPU) builds and creates every kernel of both methods (desktop and stereo baked, the
  shadow maps') by polling, exit 1 on a failure. `render_check.gd --f7_at_frame=<n>`: the F7 switch at frame n.
- (Deferred minors (old march)) - Outline pass colour on semi-transparent rims (the centre ray's colour per alpha, or the most opaque neighbour's)
    is not checked against full marches (Task 5b M4); bound unknown.
- (Deferred minors (old march)) - Light cache: the outside-cascade check does not assert a complete fill; the cache's 62.5 m detail floor against the
    march's finer detail is not measured (section 8 runs detail off); one stale cache frame when the flag is switched
    back (Task 1).
- (Deferred minors (old march)) - `near_samples` is written from the main thread by cloud_check's diagnostics (render-thread contract, Task 2).
- (Deferred minors (old march)) - `cloud_structure_step` switches on a type weight > 0: any cirrus share makes the cell's step fine (a sharpness
    change at type-band edges); the redone dense step keeps the sub-loop's `outside` flag (Task 5c).
- (Deferred minors (old march)) - Temporal variance clip: fp32 cancellation on the jittered path only (Task 9).
- (Deferred minors (old march)) - cloud_far_check: the 50 km horizon speckle at the VR angle is capped at 13 % (measured 11.0 %).
- (Open: the vehicles' direct Sun) - The Sun's direct beam: the drawn disk follows the shapes (above exp(-tau / mu), the plane-parallel bias), the CPU's
  `weather_cloud_light` `sun_transmittance` is the grid mean's exp(-tau / mu) (tau 5 at 55 deg: 0.00235 vs the disk's
  0.00331). That CPU value lights the vehicles (direct Sun) and enters the eye adaptation (`Atmosphere`
  `cloud_light_at`); the ground and the clouds themselves take the GPU paths (shadow maps, march). Open: whether the
  vehicles' and the eye's direct Sun should follow the drawn shapes' transmittance at the light point.
