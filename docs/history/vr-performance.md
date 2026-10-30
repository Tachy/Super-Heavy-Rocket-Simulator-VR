# VR performance - history

Superseded text of `docs/subsystems/vr-performance.md`.

## Before the Task 10 follow-up (superseded 2026-10-27)

Cloud pass profile, "Options with a look trade" as first written (not applied, user decision open):

Options with a look trade (not applied; user decision, crops `screenshots/static_clouds/t10_opt_*.png`: base | 4
sub-samples | divisor 4):
- at most 4 sub-samples instead of 8: -9.3 / -8.9 ms (-23 / -19 %); the grain check passed with 4 in Task 5c (1.8 %
  vs the 8 x reference's alpha, 8: 0.8 %); visible: slightly more fine speckle in the far deck band.
- march divisor 4 instead of 3 (1.8 x fewer rays): -11.3 / -14.3 ms (-28 / -30 %); visible: softer near cumulus,
  blockier outlines of the lower deck.
- step-weight limit 0.3 instead of 0.15 (a dense step's light sampled for up to 30 % of the pixel): -4.7 / -5.2 ms
  (-12 / -11 %); crops `t10_opt_*_stepweight.png`: mean |difference| 0.5-0.7 of 255, the lobes' shading a little
  flatter; not checked against the reference cases (cloud_check 8 / 9, the grain check).

## Cloud view cache in the frame before Task 4 fix round 1 (superseded 2026-10-28)

> ~1 s at 11 ms per frame, the coarse level 0 (1/4 density) in ~2-3 frames.
>
> Convergence after a start / jump (the planner's restart; VR proxy): level 0 over the view after 0.32-0.37 s (20-23
> frames), the full level over the view after 3.2-3.3 s (191-199 frames), the whole cube after 12.8 / 16.8 s; the fill
> per frame median 4.0-4.4 ms (the level-0 / level-1 phases are capped by the pick's 128 tiles a call, not by the
> budget), max 31-34 ms (overshoot where the picks turn from clear-sky to deck tiles: the cost per full tile varies
> 0.1-1.6 ms). Desktop preset (1600 x 900, budget 6 ms, face 1984): view level 0 after 0.1-0.2 s, full level after
> 2.1-4.3 s, the whole cube after 6.4-13 s; fill median 4.0 ms, p90 7.5, p99 17 ms. Main thread (FrameProfiler section
> `cloud_view_cache`, desktop deck while filling): median 0.6 ms, p90 1.0, p99 1.4 ms, the first frame (cold pick) 3.0
> ms (the layout and schedule are built on a worker: 8 ms at face 1984 before); VR proxy median 0.8-0.9 ms, max 5.5-5.9
> ms (the cold pick at face 3200); 0 once converged.

## Superseded by cloud view cache Task 4 fix round 2 (2026-10-28)

> (1600 x 900, budget 6 ms, face 1984; `t4_rc.sh f1g <view>`): level 0 after 0.6 / 1.7 s (12 / 30 frames, ground /
> deck), covering ended 1.2 / 2.6 s, full level 3.2 / 5.9 s, cube 17.3 / 21.6 s; fill median 3.1 / 3.8 ms, p99 8.2 / 9.1
> ms, max 9.9 / 11.0 ms - over the fixed 6 ms because one full-level deck tile alone takes up to ~8.4 ms (a fill's time
> is its slowest ray's; `weather-clouds.md`, Budget); main thread max 0.19 / 0.20 ms.

## World volume light before Task 6 review round 1 (superseded 2026-10-28)

Superseded by the distance-stepped density march, one dispatch per level and the per-level cost model (vr-performance.md).
Text as it stood:

    ### World volume light (cloud world volume plan Task 6, 2026-10-28)

    GPU intervals now `cloud_volume_upkeep` (bake, max mip, distance field) and `cloud_volume_light` (provisional and real
    light); the upkeep is their sum. Desktop RTX 4090, `--xr-mode off`:
    - Light kernel rate (cloud_check section 17, section 8's thick slab, every ray through tau ~10: the costliest content
      measured): ~2.8e5 cells/ms on average over the cold start, ~6e4 cells/ms at level 0 (every cell within 2.6 km of
      the deck); clear-dominated refresh passes calibrate to 1-2e6 cells/ms. the spec's 1.1e6 cells/ms [E] holds only for
      mostly clear content: a full refresh of the 67 M cells takes ~50 ms of GPU at the clear rate (~1 s of 1 ms frames),
      ~0.25 s in the slab (~5 s at the 1.0 ms light budget: a refresh lags by seconds, never a front).
    - Cold start in the slab (light at the spike's share, the bake at 40 x): every level complete 16-51 frames after its
      density is shown (level 0: 51 frames = 1.13 s, above the 1 s target; levels 1-7 within it); light GPU median
      2.71 ms per frame (the 3 ms spike cap less the bake), max 14.7 ms (the rate calibration lags content changes).
    - `render_check --cloud_view=V1 --world_volume --gpu_time --gpu_passes` (fill and light awaited): cold start in 309
      frames, upkeep (bake + light) median 1.74 ms, p99 13.5 ms, max 21 ms; after it (still view, clock 1 x)
      `cloud_volume_upkeep` median 0.003 / p99 0.003 ms, `cloud_volume_light` 0.003 / 0.003 ms. V3: complete after 822
      frames (its cold-start upkeep series was not captured: one sample); after it 0.003 / 0.004 ms and 0.003 / 0.003 ms.
      The cold start's p99 / max exceed the 3 ms spike cap: per-frame cost follows content (clear vs cloudy cells, taps
      skipped by the clear test) faster than the 20 %-per-frame calibration - the gate (Task 11) measures it in VR.

## World volume light before Task 6 review round 1c (superseded 2026-10-29)

Superseded by N = 32 cells per level and the latency-aware cost model (vr-performance.md). Text as it stood:

    ### World volume light (cloud world volume plan Task 6 and its review round 1, 2026-10-28)

    GPU intervals `cloud_volume_upkeep` (bake, max mip, distance field; per level `cloud_volume_bake_<k>`) and
    `cloud_volume_light` (provisional and real light; per level `cloud_volume_light_column_<k>` /
    `cloud_volume_light_cells_<k>`); the upkeep is their sum. Light: the distance-stepped density march (16 cells per
    level), one dispatch per level and kernel. Desktop RTX 4090, `--xr-mode off`:
    - Light kernel rate, whole-level refresh on the thick slab (cloud_check section 18, every ray through tau ~10), cells
      per ms, levels 0 .. 7: 1.1e6, 3.6e6, 6.1e6, 5.0e6, 1.1e7, 3.9e6, 1.0e7, 1.1e7; the column pass 3.6-8.9e6 column
      cells per ms. A full refresh of the 67 M cells ~17 ms of GPU (~17 frames at the 1.0 ms light budget).
    - Latency floor: a dispatch costs at least its longest ray's march (~0.25-0.45 ms on cloudy content: 7-8 segments of
      32 dependent samples), so small dispatches run far below those rates (level 1's cold-start dispatches of ~170 k
      cells: ~3.5e5 cells/ms). The cost model is linear in cells (plus 0.05 ms per level); small dispatches only raise a
      rate; the refresh serves one level at a time. Before (a dispatch per job, equal shares over eight stale levels): a
      pair switch's light median 1.6 ms for a few thousand cells.
    - Cold start in the slab (section 17, light at 0.65 of what the bake leaves of the 3 ms spike cap): upkeep median
      1.26 ms, p99 1.90, max 2.27 ms (before round 1: max 14.7 ms; at 0.8 of it: max 2.71 ms, and 3.36 against 2.40 ms
      planned in the full check); level 0 complete 30 frames after its density is shown (<= 45 frames, 1 s); levels 2-7
      within 34-52 frames, level 1 48-98 frames over five runs (latency-bound, above).
    - Under load at the game's budget (section 18): the Sun at 100 x for 10 s (4.2 deg) light median 0.80 ms, upkeep max
      1.40 ms; a pair switch (analog + ERA5 -> ERA5 + ERA5, the weight 0.82 -> 0.50 over 10 s, every 0.02 relighting all
      levels) light median 0.92, p99 1.26, upkeep max 1.67 ms.
    - Bake rates per level (render_check V1 cold start, taps per ms, occupancy included): L0 3.7e7, L1 3.0e7, L2-L4
      3.7-4.3e7, L5-L7 0.5-1.0e8 (section 17's slab: L0 1.6e7 .. L7 1.8e8).
    - `render_check --cloud_view=V1 --world_volume --gpu_time --gpu_passes`: cold start complete after 1007 frames, upkeep
      median 1.22 ms, p99 1.84, max 2.23 ms (before round 1: p99 13.5, max 21 ms); after it `cloud_volume_upkeep` and
      `cloud_volume_light` median 0.003 ms. V3: complete after 823 frames (its cold-start series again one sample), after
      it 0.003 / 0.004 ms and 0.003 / 0.003 ms.

## Cloud view cache, spike S and the world volume (removed 2026-10-30)

The baked volume became the default; these sections of `docs/subsystems/vr-performance.md` follow verbatim (their
code is deleted, `docs/history/weather-clouds.md` "Removed 2026-10-30").

### Cloud view cache budget (cloud view cache plan Task 1, 2026-10-27)

`--no_cloud_pass` (user argument; render_check passes it through) skips the near cloud pass (`CloudLayer.pass_enabled`;
its shadow maps and light cache with it). Desktop proxy (`render_check --render_scale=1.65 --gpu_time=120 --gpu_passes
--cloud_quality=vr --fixed_exposure`, Starbase 2024-06-15T21:00 UTC, analog weather; scripts
`.superpowers/sdd/2026-10-27-cloud-view-cache/t1_prof.sh`), GPU frame median / p99:

| View | with the cloud pass | without | weather off |
|---|---|---|---|
| ground (az 90, el 25) | 44.9 / 47.2 ms (cloud pass 41.3: march 32.6, refine 8.5; shadow pass 1.5) | 1.6 / 3.2 ms | 1.6 / 2.7 ms |
| 9 km deck (el -10) | 49.8 / 56.0 ms (cloud pass 45.0: march 35.4, refine 9.6) | 2.5 / 6.3 ms | - |

Headset (Pimax Dream Air, 3534 x 2838 per eye, pad, weather on, 45 Hz, `data/perf/perf_20261027_2234*.csv` /
`_223635.csv`): with the cloud pass GPU 45.3 ms median (15-33 fps); without it GPU 7.4 ms median, 8.2 ms p90, 8.8 ms
max, the frame interval a steady 22.22 ms. **Cache budget at 45 Hz: 22.2 - 9 (scene, max) - 2 (margin [E]) ~ 11 ms
per frame** (the preset `cache_budget_ms`, Task 4). The per-frame pass's rate (1.3 M rays of the 1377 x 947 centre in
~45 ms, refine included) puts a full-resolution fill of the visible region (~9 x the rays) at ~0.4 s of GPU, i.e.
~1 s at 11 ms per frame (measured with the cache, Task 4 fix round 1: level 0 of the view in ~12 frames, below).

### Cloud view cache in the frame (cloud view cache Task 4, 2026-10-28)

The frame shows the world-fixed cache (fill under the budget, per-eye lookup; `docs/subsystems/weather-clouds.md`).
Desktop VR-pixel proxy, the same runs as Task 1 with the cache on (`t1_prof.sh cache <view>`; the VR preset: budget
cap 11.4 ms; face 3200 texels at the proxy's density), once the cache has nothing left to fill:

| View | GPU frame median / p99 | cloud pass (the lookup) | before (per-frame pass) |
|---|---|---|---|
| ground (az 90, el 25) | 2.56 / 4.99 ms | 0.10 / 0.52 ms | 44.9 / 47.2 ms |
| 9 km deck (el -10) | 3.63 / 8.27 ms | 0.11 / 0.46 ms | 49.8 / 56.0 ms |

Fix round 1 (2026-10-28; the plan on a worker, the covering march, the latency cost model; `t1_prof.sh f1final
<view>`, logs `.superpowers/sdd/2026-10-27-cloud-view-cache/t1_prof_<view>_f1final.log`). VR proxy (budget 11.4 ms,
face 3200): level 0 over the view after 0.58-0.63 s (12 frames; the frames are slow while the march covers), the
covering march ended after 1.21-1.22 s (23-24 frames), the full level over the view after 4.5-4.7 s (217-228 frames),
the whole cube after 27.2 s (~1580 frames); fill median 2.0 / 2.8 ms, p99 7.1 / 8.2 ms, max 8.0 / 9.3 ms (ground /
deck; budget + 2 ms never exceeded - the largest excess -3.4 / -2.1 ms); converged GPU frame 2.56 / 3.64 ms median.
Level 0 is bound by the fill cap (`MAX_FILL_GROUPS` 1024 work groups = 256 level-0 tiles, ~6 ms of GPU a frame) after
four frames under the unknown-latency prior (~64 tiles each). Main thread (FrameProfiler `cloud_view_cache`, every
frame incl. the restart): median 0.04 ms, p99 0.07 ms, max 0.09 / 0.21 ms (the plan runs on a worker). Desktop preset
(1600 x 900, budget 9 ms, face 1984; `t4_rc.sh f2t <view>`): deck level 0 after 0.45 s (8 frames), covering ended 0.97
s, full level 2.3 s, cube 14.6 s; fill median 3.1 / 3.5 ms, p99 8.9 / 10.0 ms, max 9.8 / 11.4 ms (ground / deck; one
full-level deck tile alone takes up to ~8.4 ms, a fill's time is its slowest ray's - `weather-clouds.md`, Budget);
main thread p99 0.06 / 0.07 ms, max 0.08 / 0.14 ms.

### Near shell and re-anchor (cloud view cache Task 5, 2026-10-28)

`render_check --frames_every=0.5 --frames_for=80` (Starbase ascent, `--autopilot --fly=25`, slab 1.5-2.5 km cover
0.6, VR preset, render scale 1.65 - the desktop VR proxy; 3600 flight frames of 1/45 s each): from on board (the near
shell in 3529 of them: the deck within R_near of the pad already) GPU frame median 8.7 ms, p99 27.5 ms, max 32.4 ms -
**the 22.2 ms target is missed in the deck passage** (near shell march + covering after evictions + fills); watched
from the ground 2 km east median 5.3 ms, p99 18.8 ms, max 24.1 ms. The near shell's march is the frame's steady cost
(not excluded from the adaptive budget's rest of the frame; only the covering march is). Logs
`.superpowers/sdd/2026-10-27-cloud-view-cache/t5_final_fly_{board,site}.log`.

Fix round 1 (the near shell predicted in the budget, covering by the near shell's kernel, eviction by a tile's own
parallax, `--frames_for=120`, the first frame after each sample's reference not counted): on board GPU median 12.3 /
p99 24.4 / max 30.1 ms - still over 22.2 ms: the rest of the frame p95 ~14 ms (median 8; `opaque_and_sky` 6-11 ms)
plus the near shell inside the deck 8-15 ms (`cloud_pass_march`) leave the fill its 2 ms floor; watched from the
ground median 6.9 / p99 15.2 / max 19.4 ms. Logs `r1_fly_{board,site}_final2.log`.

### Spike S: far tier, baked vs procedural density (cloud world volume plan Task 2, 2026-10-28)

The far tier alone (`CloudSpike`, `weather-clouds.md` Diagnostics): rays [2 km, end] from the cyclops point over the
union of both eye frusta (stereo harness, provisional Pimax projections, 0.5x: centre image 2196 x 1420 = 3.12 Mpx;
1.0x: 4392 x 2839), perfect occupancy (view-exact taps, 200 m cells, Chebyshev distance field), constant light, no air.
(a) per-step `cloud_extinction_at`; (b) one fetch of a 100 m bake + one dummy RGBA16F light fetch. Gate views V1 (pad,
slab 1-2.5 km cf 0.5) and V3 (9 km above slab 1.5-2.5 km), `--handover=0,0 --gpu_time=120 --gpu_passes`; sdd logs
`t2_<view>_<spike>_<tag>.log`, summary `t2_summary.txt`. March kernel ms, median / p99 (`c`: without counters):

| | V3 (a) | V3 (b) | V1 (a) | V1 (b) | V3 (b) 1.0x |
|---|---|---|---|---|---|
| march, no counters | 23.5 / 41.1 | **12.4 / 25.3** | 34.7 / 52.4 | 17.7 / 28.8 | 72.7 / 91.3 |
| march, counters | 24.9 / 44.3 | 13.1 / 26.6 | 39.8 / 61.2 | 19.0 / 34.9 | 48.7 / 102.6 |
| occupied steps / px median, p90, p99; mean | 124, 587, 1581; 219 | 121, 591, 1425; 215 | 227, 917, 2343; 358 | 228, 927, 2116; 356 | 122, 620, 1512; 227 |
| of them Delta-tau limited / with extinction > 0 | 97 % / 16 % | 97 % / 40 % | 72 % / 7 % | 72 % / 21 % | 94 % / 38 % |
| empty (sphere-traced) steps / px median | 1 | 1 | 0 | 0 | 1 |
| ps per occupied step (counter run) | 36.4 | 19.5 | 35.7 | 17.1 | 17.2 |
| density fetch ps / step (ablation `density`) | 27.6 | 9.4 | 27.7 | 7.7 | |
| light fetch ps / step (ablation `light`) | - | 2.1 | - | 2.4 | |

Today's march over the same far part (`--spike=ref`): V1 21.5 / 38.4 ms. Sanity (mean alpha of the centre image):
V3 ref 0.4655, (a) 0.4710, (b) 0.4763; V1 ref 0.3513, (a) 0.3558, (b) 0.3595; luminance (b) vs (a) +0.1 % / -0.5 %.
The 1.0x run without counters measured slower than with counters (72.7 vs 48.7 ms): run-to-run spread at that load
(both p99 > 90 ms); the per-step figure is from the counter run. Ablation `density` replaces the fetch by half the
occupancy's max extinction (no fetch): the rays end sooner (2.4e8 vs 6.7e8 steps in V3), so its split is per step.

Rules (spec G): (b) <= 5 ms in V3: no (12.4 ms). (b) >= 1.5x cheaper per occupied step than (a): yes (1.87x V3, 2.09x
V1). > 100 occupied steps per pixel in V3 despite perfect occupancy: yes (median 121, mean 215). (b) > 10 ms: yes - stop
and report. The cost is the step count: inside occupied cells the step is 0.3 / (the cell's dilated max extinction) -
real cumulus has ~0.05-0.15 /m, so 2-6 m steps also through the cells' clear parts (V3 (b): 60 % of the occupied steps
sample no extinction), down to the scaled termination (tau ~18 at k = 1 - g^2). At (b)'s 19.5 ps a step, 5 ms in V3 is
~82 occupied steps per pixel on average (2.6x fewer).

#### Spike S round 2: the step rule (Task 2b, 2026-10-28)

Same spike, variant (b) only, now with the recorded Pimax projections (centre image 2065 x 1420 = 2.93 Mpx at 0.5x;
the V3 region grew to 503 x 788 km). Levers (`render_check --spike_levers=1,2,4,s,t`, `--spike_stop=<T>`), on a
second volume over the bake grid (R16F, 100 m: the max of the 3^3 bake texels a trilinear fetch in the cell reads -
the exact sup of the interpolated field - or, for a clear cell, minus its Chebyshev distance to the nearest non-clear
cell, cap 8): **1** Delta tau 0.3 from that max, step ends at the bake cell's exit, clear bake cells stepped to their
exit; **2** sphere tracing on its distance; **3** termination at scaled T 0.03 instead of 0.01; **4** one step per
bake cell (to its exit, no Delta tau term: Beer-Lambert is exact for a constant-extinction segment, and with a constant
source the in-scatter S T (1 - e^{-k tau}) / k is exact for any profile, so only tau matters), **t** with 4 the
trilinear bake at the step's middle instead of the cell's mean (midpoint rule on a field linear between texel centres),
**s** with 4 the structure term kept (production adds sub-cell detail where w_k > 0). sdd logs `t2b_*.log`, summary
`t2b_summary.txt`; one run at a time. ms = march kernel median / p99; steps per pixel median / p90 / p99 / mean.

| V3 (b) | ms (counters) | ms (none) | occupied steps | empty | mean alpha | lum vs base |
|---|---|---|---|---|---|---|
| base (round 1 rule) | 22.7 / 28.4 | 18.8 / 25.4 | 137 / 610 / 1503 / 228 | 1 / 6 / 18 | 0.4683 | - |
| 1 | 11.8 / 17.9 | | 10 / 233 / 560 / 95 | 1 / 33 / 73 | 0.4684 | +0.0 % |
| 2 | 29.0 / 35.9 | | 116 / 469 / 1287 / 180 | 1 / 10 / 23 | 0.4683 | +0.0 % |
| 1,2 | 12.2 / 18.2 | 9.6 / 16.9 | 10 / 233 / 560 / 95 | 1 / 17 / 44 | 0.4684 | +0.0 % |
| 3 (T_sc 0.03) | 22.3 / 27.4 | | 121 / 588 / 1443 / 216 | 1 / 6 / 18 | 0.4683 | -1.9 % |
| 4 | 1.17 / 3.01 | | 3 / 16 / 41 / 6.5 | 1 / 33 / 73 | 0.4624 | -0.5 % |
| 1,2,4 | 1.07 / 2.63 | 1.03 / 2.64 | 3 / 16 / 41 / 6.5 | 1 / 17 / 45 | 0.4624 | -0.5 % |
| 1,2,4,t | 1.03 / 2.72 | 0.97 / 2.20 | 3 / 16 / 38 / 6.3 | 1 / 17 / 44 | 0.4678 | +0.3 % |
| 1,2,4,s | 1.86 / 4.55 | 1.80 / 4.42 | 7 / 48 / 101 / 19 | 1 / 17 / 45 | 0.4624 | -0.8 % |
| **1,2,4,s,t** | 1.93 / 4.69 | **1.85 / 4.99** | 7 / 47 / 98 / 19 | 1 / 17 / 44 | 0.4684 | +0.2 % |

V1 (b): base 25.7 / 33.2 (counters), 26.1 / 33.1 ms, steps 329 / 976 / 2212 / 391, alpha 0.3916; 1,2: 8.7 / 15.0 ms,
105 mean; 1,2,4: 1.03 / 1.30 ms, 6.6 mean, alpha 0.3861; 1,2,4,t: 1.08 / 1.43 ms, alpha 0.3910; 1,2,4,s: 3.88 / 7.89 ms,
59 mean; **1,2,4,s,t: 4.14 / 10.06 ms**, alpha 0.3916, lum +0.1 %. 1.0x V3: 1,2,4 3.0 / 5.5 ms; 1,2,4,s 11.8 / 13.9 ms.
References (today's march clipped, `--spike=ref`): V3 alpha 0.4554 (32.0 / 36.0 ms), V1 0.3840 (25.1 / 28.7 ms); all
(b) variants are within 0.014 alpha of them. Display-level difference vs the base (eye 0 PNG, 8-bit, max over RGB):
1,2,4,s,t V3 mean 0.12, p99 1, 0.05 % of pixels > 2 steps; V1 mean 0.04, p99 1, 0.00 %. Lever 3 (0.03): mean 0.43,
max 2 steps, for 5 % fewer steps. Lever 4 with the cell's mean (no t) shows the 100 m cells as stair-stepped edges near
R_near (V3 4 % of pixels > 2 steps, max 156); t removes them (with s: 0.05 %).
Base ms is noisier than round 1 (V3 14.3 ms in a first run, 18.8 ms in the clean matrix): the user works on the machine.

Rules (spec G) with 1,2,4,s,t: (b) <= 5 ms in V3: yes (1.85 ms median, p99 4.99) - build. > 100 occupied steps per
pixel in V3: no (median 7, mean 19). (b) > 10 ms: no. Per occupied step 34.6 ps (more work per step: two grid maps and a
max fetch), but 12x fewer steps. Lever 2 alone is slower than the base (its per-step overhead on every occupied step,
only 21 % fewer steps); lever 3 is not worth its luminance bias. The lever that closes the gap is 4 (+ t for the
image, + s for production's sub-cell detail); 1 and 2 matter only for clear bake cells under it.

### World volume upkeep (cloud world volume plan Task 5, 2026-10-28)

`cloud_check.gd` section 16 (desktop RTX 4090, `--xr-mode off`; Starbase, analog October 12 UTC + ERA5 2025-10-25
15:00 at blend 0.3, camera 2 m): GPU interval `cloud_volume_upkeep` (bake + max mip + distance field):
- cold start at the game's budget (spike frames: 1.5 ms bake), 45 frames: median 1.34 ms, max 2.84 ms; measured
  4.3e7 taps/ms = 5050 level-7 cells/ms (the bake's cost is content-dependent: a clear cell skips its taps);
- whole fills at 40 x the budget: 6.8-7.6e7 taps/ms.
- One thread per cell (first version) was latency-bound: a level-7 cell's 8448 taps take one thread ~6 ms, so a
  frame's small batch ran 6 ms median / 17 ms max at 1e6 taps/ms; one 64-thread group per cell fixed it.
- Consequences at the game's budget (estimates from the rate, not run): level 7 complete after ~7e10 taps / (1.5 ms x
  ~5e7) ~ 950 frames (~21 s at 45 Hz), the whole volume (~1.24e11 nominal taps) ~25 s; a pair switch's rebake of
  one channel pair (all levels, nominal taps) at the normal 0.5 ms bake budget ~1-2 min, the cross-fade held meanwhile. The
  spec's "full volume within ~1 s" is not met by the bake rate; the fallback (today's march) covers the cold start.
- Scroll (Task 5 fix round 1, bake ahead of exposure; `cloud_world_volume_test`, planned at 4.3e7 taps/ms, estimates
  from the rate): 1 km/s at 10 km climbing, all 8 levels: scroll bake mean 0.44 ms, worst 0.50 ms, every plan
  <= 0.58 ms; 7.8 km/s at 120 km, levels 2-7: mean 1.36 ms, worst 1.94 ms, every plan <= 2.04 ms (cap: 2.0 ms bake
  + 1.0 ms light). Before the fix one frame baked a level-k boundary crossing of every finer level at once: ~3.8 /
  5.4 / 12 ms at level 5 / 6 / 7. In orbit the steady upkeep is therefore ~2-2.5 ms with light, above the 1.5 ms
  normal budget (the coarse levels' 528-8448 taps per cell; the L7 tap cut is the lever).
- The distance field is recomputed whole for every level a batch touches (3 passes over 64 x 64 x 32 texels,
  ~0.05 ms per level): with amortised scroll slabs up to 6 levels a frame at 7.8 km/s, ~0.3 ms inside the upkeep
  interval (the bake-rate feedback absorbs it).

#### World volume light (cloud world volume plan Task 6 and its review rounds 1 / 1c, 2026-10-28 / 29)

GPU intervals `cloud_volume_upkeep` (bake, max mip, distance field; per level `cloud_volume_bake_<k>`) and
`cloud_volume_light` (provisional and real light; per level `cloud_volume_light_column_<k>` /
`cloud_volume_light_cells_<k>`); the upkeep is their sum. Light: the distance-stepped density march (32 cells per
level), one dispatch per level and kernel, planned by the latency-aware cost model (`weather-clouds.md`). Desktop
RTX 4090, `--xr-mode off`:
- Light kernel rate, whole-level refresh on the thick slab at 40 x the budget (cloud_check section 18, every ray
  through tau ~10), cells per ms, levels 0 .. 7: 3.8e5, 8.5e5, 2.4e6, 4.6e6, 4.5e6, 4.0e6, 3.2e6, 5.4e6; the column
  pass 3.1-8.8e6 column cells per ms. A full refresh of the 67 M cells ~46 ms of GPU light kernel (+ ~12 ms column).
- The test GPU's rates depend on its load: in the cold start at the game's budget (frames of 1-3 ms GPU) the same
  kernel runs at ~1e5 cells/ms at level 0, ~2-3e5 at level 1, ~0.5-1e6 at levels 2-7. Latency (a small dispatch's
  time less its cells at the rate): ~0.05 ms at level 7, ~0.2-0.3 ms at levels 2-6, none measurable at level 0. The
  bake's last work often lands in the light's first list (~0.2 ms, also in frames with no light job; the sum is
  unchanged).
- Cold start in the slab (section 17, the game's budget; the fill bake held to 0.2 of the 3 ms cap while a filled
  level's light is due, the light at 0.85 of what the bake leaves): every level's light complete 17-39 frames after
  its density is shown (levels 7 .. 0: 18 / 23 / 18 / 19 / 17 / 19 / 33 / 39; <= 45 frames asserted); upkeep median
  1.13 ms, p99 2.31, max 2.60 ms alone and 5.04 ms in the full run (the cold start is a transition: <= 6 ms =
  2 x the 3 ms cap asserted; the 3 ms cap binds in-flight load); the whole volume after 1255 frames (round 1b: 1228).
  The 5.04 ms frames are a GPU clock-state change right after the bake ends (level 0 at half the measured rate,
  feedback 2-3 frames late). A level whose latency alone exceeds the 1.0 ms light budget is probed after 45 plans
  (round 2: weather-clouds.md), so its cost is measured again.
- Under load at the game's budget (section 18): the Sun at 100 x for 10 s (4.2 deg) light median 0.75 ms, p99 1.26,
  upkeep max 1.50 ms; a pair switch (analog + ERA5 -> ERA5 + ERA5, the weight 0.82 -> 0.50 over 10 s, every 0.02
  relighting all levels) light median 0.67, p99 1.29, upkeep max 1.68 ms.
- Bake rates per level (render_check V1 cold start, taps per ms, occupancy included): L0 3.7e7, L1 4.6e7, L2-L4
  7.0-8.6e7, L5-L7 1.0-1.8e8 (section 17's slab: L0 1.7e7 .. L7 1.7e8).
- `render_check --cloud_view=V1 --world_volume --gpu_time --gpu_passes`: cold start complete after 881 frames, upkeep
  median 1.08 ms, p99 1.60, max 1.95 ms; after it `cloud_volume_upkeep` median 0.003 ms. V3: complete after 822
  frames (its cold-start series one sample), after it 0.003 / 0.003 ms.

### World volume march (cloud world volume plan Task 7, 2026-10-29)

`render_check --cloud_view=<V> --world_volume --volume_fill_scale=40 --counters --gpu_time=20 --gpu_passes
--fixed_exposure` (desktop RTX 4090, `--xr-mode off`, 1600 x 900 = 1.44 MP, one eye; GPU interval
`cloud_volume_full`, the full-resolution reference kernel `CLOUD_VOLUME_FULL`). Fetches per occupied step = (density +
light fetches) / occupied steps of eye 0 (the distance-field reads of the step rule are not counted).

| state | V1 ms | V1 fetches / occ. step | V1 occ. steps (sum) | V3 ms | V3 fetches / occ. step |
|---|---|---|---|---|---|
| round 0 (spreading sub-cell field, correction every step) | 95.6 | 18 (8.3 + 9.6) | 44.1 M | - | - |
| threshold field + correction on the first 4 evaluations | 139.6 | 8.4 (6.9 + 1.5) | 98.6 M | - | - |
| + light nodes (0.5 cell) | 141.1 | 7.8 (7.0 + 0.86) | 98.6 M | 300.2 | 9.2 (9.0 + 0.26) |
| + structure-capped first step after empty space, 1-fetch s_cell | 138.3 | 7.7 (6.8 + 0.85) | 99.4 M | 297.2 | 9.0 (8.7 + 0.25) |
| + surface correction at the light nodes (final) | 131.6 | 7.3 (6.8 + 0.46) | 99.4 M | 264.8 | 8.8 (8.7 + 0.06) |

- The threshold field keeps the gaps open (the spreading field's haze ended rays early): the occupied steps doubled
  (V1 mean 31 -> 69 per pixel; V3 202, p99 1904) - the steps through partial cover at the structure scale (0.25 x the
  finest active structure: 3.9 m at V1's near footprint, 62 m at V3's 40 km) dominate, worst at the horizon rows.
- Ablations at V1 (the state before the light nodes): light ablated 55 ms (of 143); air ablated 131; without the
  surface correction 117; no silhouette bisection: unchanged; structure fraction 0.5: 116 ms, 58 M steps. The light
  costs ~60 % of the pass although only ~1 light fetch per occupied step remains: register pressure / warp
  divergence of the large source code in the loop (the light nodes cut its evaluations 6 x without a time change).
- Stereo (22.5 MP per eye): not run (ruling R3: only with V1 desktop <= 15 ms). Extrapolated from V1's 91 ns per pixel:
  ~2.1 s per eye, V3 ~4.1 s - beyond the Windows TDR limit; the full-resolution kernel stays the tiers' reference
  (Task 8).
